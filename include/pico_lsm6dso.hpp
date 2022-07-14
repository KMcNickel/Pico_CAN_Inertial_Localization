#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "filters.hpp"

//#define PRINT_DEBUG_STATEMENTS

class PICO_LSM6DSO
{
    public:
        typedef enum {
            REGISTER_ADDRESS_WHO_AM_I   = 0x0F,
            REGISTER_ADDRESS_CTRL1_XL   = 0x10,
            REGISTER_ADDRESS_CTRL2_G    = 0x11,
            REGISTER_ADDRESS_CTRL3_C    = 0x12,
            REGISTER_ADDRESS_OUT_TEMP_L = 0x20,    //low byte, value is 16 bit
            REGISTER_ADDRESS_OUTX_L_G   = 0x22,    //low byte, value is 16 bit
            REGISTER_ADDRESS_OUTY_L_G   = 0x24,    //low byte, value is 16 bit
            REGISTER_ADDRESS_OUTZ_L_G   = 0x26,    //low byte, value is 16 bit
            REGISTER_ADDRESS_OUTX_L_A   = 0x28,    //low byte, value is 16 bit
            REGISTER_ADDRESS_OUTY_L_A   = 0x2A,    //low byte, value is 16 bit
            REGISTER_ADDRESS_OUTZ_L_A   = 0x2C,    //low byte, value is 16 bit
        } LSM6SDO_REGISTER_ADDRESSES;

        typedef enum {
            BDU_CONTINUOS 	 = 0x00,
            BDU_BLOCK_UPDATE = 0x40,
            BDU_MASK         = 0xBF 
        } LSM6DSO_BDU_t;

        typedef enum {
            ODR_DISABLE   = 0x00, 
            //ODR_1_6Hz     = 0xB0, // Low Power only
            ODR_12_5Hz    = 0x10, // Low Power only
            ODR_26Hz      = 0x20, // Low Power only
            ODR_52Hz      = 0x30, // Low Power only 
            ODR_104Hz     = 0x40, // Normal Mode
            ODR_208Hz     = 0x50, // Normal Mode
            ODR_416Hz     = 0x60, // High performance
            ODR_833Hz     = 0x70, // High Performance 
            ODR_1660Hz    = 0x80, // High Performance
            ODR_3330Hz    = 0x90, // High Performance
            ODR_6660Hz    = 0xA0, // High Performance
            ODR_MASK      = 0x0F
        } LSM6DSO_ODR_t;

        typedef enum {
            FS_XL_2g 		 = 0x00,
            FS_XL_16g  	     = 0x04,
            FS_XL_4g 		 = 0x08,
            FS_XL_8g 		 = 0x0C,
            FS_XL_MASK       = 0xF3
        } LSM6DSO_FS_XL_t;

        typedef enum {
            IF_INC_DISABLED  = 0x00,
            IF_INC_ENABLED 	 = 0x04,
            IF_INC_MASK      = 0xFB
        } LSM6DSO_IF_INC_t;

        typedef enum {
            FS_G_125dps      = 0x02,
            FS_G_250dps 	 = 0x00,
            FS_G_500dps      = 0x04,
            FS_G_1000dps 	 = 0x08,
            FS_G_2000dps 	 = 0x0C,
            FS_G_MASK        = 0xF0
        } LSM6DSO_FS_G_t;

        const uint8_t WhoAmIExpectedValue = 0x6C;

        float convertAccelValue(int8_t rawMSB, int8_t rawLSB, LSM6DSO_FS_XL_t range)
        {
            float multiple;
            switch(range)
            {
                case FS_XL_2g:
                    multiple = 0.061;
                    break;
                case FS_XL_4g:
                    multiple = 0.122;
                    break;
                case FS_XL_8g:
                    multiple = 0.244;
                    break;
                case FS_XL_16g:
                    multiple = 0.488;
                    break;
                default:
                    return -1;
                    break;
            }

            int16_t rawData = ((rawMSB << 8) | rawLSB);

            return (static_cast<float>(rawData)  * multiple) / 1000;


        }

        float convertGyroValue(int8_t rawMSB, int8_t rawLSB, LSM6DSO_FS_G_t range)
        {
            float multiple;
            switch(range)
            {
                case FS_G_125dps:
                    multiple = 4.375;
                    break;
                case FS_G_250dps:
                    multiple = 8.75;
                    break;
                case FS_G_500dps:
                    multiple = 17.5;
                    break;
                case FS_G_1000dps:
                    multiple = 35;
                    break;
                case FS_G_2000dps:
                    multiple = 70;
                    break;
                default:
                    return -1;
                    break;
            }

            int16_t rawData = ((rawMSB << 8) | rawLSB);
            
            return (static_cast<float>(rawData)  * multiple) / 1000;
        }

        float convertTempValue(int8_t rawMSB, int8_t rawLSB)
        {
            float result = rawMSB;
            result += rawLSB / 256;

            return result;
        }

        void nullAndOffsetData(float * value, LowPassFilter filter, float offset)
        {
            float tmpVal = *value;
            tmpVal += offset;
            *value = filter.addNewData(tmpVal);
        }

        void reg_write(spi_inst_t * spi, const uint cs, const uint8_t reg, const uint8_t data)
        {
            uint8_t msg[2];
            msg[0] = 0x00 | reg;
            msg[1] = data;

            gpio_put(cs, 0);
            spi_write_blocking(spi, msg, 2);
            sleep_us(1);
            gpio_put(cs, 1);
        }

        int reg_read(spi_inst_t * spi, const uint cs, const uint8_t reg, uint8_t * buf, const uint8_t nbytes)
        {
            int num_bytes_read = 0;

            if(nbytes < 1) return -1;

            uint8_t msg = 0x80 | reg;

            gpio_put(cs, 0);
            spi_write_blocking(spi, &msg, 1);
            num_bytes_read = spi_read_blocking(spi, 0, buf, nbytes);
            sleep_us(1);
            gpio_put(cs, 1);

            return num_bytes_read;
        }

        void reg_update(spi_inst_t * spi, const uint cs, const uint8_t reg, const uint8_t mask,  const uint8_t data)
        {
            uint8_t buf;
            reg_read(spi, cs, reg, &buf, 1);
        #ifdef PRINT_DEBUG_STATEMENTS
            printf("Register: 0x%X - Old Value: 0x%X\n", reg, buf);
        #endif
            buf &= mask;
            buf |= data;
        #ifdef PRINT_DEBUG_STATEMENTS
            printf("Value to write: 0x%X\n", buf);
        #endif
            reg_write(spi, cs, reg, buf);
        #ifdef PRINT_DEBUG_STATEMENTS
            reg_read(spi, cs, reg, &buf, 1);
            printf("Register: 0x%X - New Value: 0x%X\n", reg, buf);
        #endif
        }

        void readAllMeasurements(spi_inst_t * spi, uint cs, float * data)
        {
            getAllData(spi, cs, data);

            for(int i = 0; i < 7; i++)
            {
                nullAndOffsetData(&data[i], filters[i], 0/*nullCalibrationData[i]*/);
            }
            printf("Filtered: %+.6f, %+.6f, %+.6f, %+.6f, %+.6f, %+.6f, %+.6f\n",
                    data[0], data[1], data[2], data[3], data[4], data[5], data[6]);
        }

        bool initialize(spi_inst_t * spi, uint cs, LSM6DSO_ODR_t rate)
        {
            uint8_t data;
            refreshRate = rate;
            reg_read(spi, cs, REGISTER_ADDRESS_WHO_AM_I, &data, 1);

            reg_read(spi, cs, REGISTER_ADDRESS_WHO_AM_I, &data, 1);
            if(data != WhoAmIExpectedValue)
            {
                printf("ERROR: Could not communicate with LSM6DSO\n");
        #ifdef PRINT_DEBUG_STATEMENTS
                printf("Value of WHO_AM_I: %X\n", data);
        #endif
                return false;
            }

            reg_update(spi, cs, REGISTER_ADDRESS_CTRL1_XL, ODR_MASK, refreshRate);
            reg_update(spi, cs, REGISTER_ADDRESS_CTRL1_XL, FS_XL_MASK, accelRange);
            reg_update(spi, cs, REGISTER_ADDRESS_CTRL3_C, BDU_MASK, BDU_BLOCK_UPDATE);
            reg_update(spi, cs, REGISTER_ADDRESS_CTRL3_C, IF_INC_MASK, IF_INC_ENABLED);
            reg_update(spi, cs, REGISTER_ADDRESS_CTRL2_G, FS_G_MASK, gyroRange);
            reg_update(spi, cs, REGISTER_ADDRESS_CTRL2_G, ODR_MASK, refreshRate);

            for(int i = 0; i < 7; i++)
            {
                filters[i].setBeta(0.1);
            }

            calibrateIMUData(spi, cs);

            return true;
        }

        void calibrateIMUData(spi_inst_t * spi, uint cs)
        {
            float readings[7];
            float averages[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

            for(int i = 0; i < calibrationPoints; i++)
            {
                getAllData(spi, cs, readings);
                for(int j = 0; j < 7; j++) averages[j] += readings[j];
                sleep_ms(50);
            }
            
            for(int j = 0; j < 7; j++)
            {
                averages[j] /= calibrationPoints;
                nullCalibrationData[j] = 0.0 - averages[j];
            }

            //Override temperature with a static value (for now)
            nullCalibrationData[0] = 25.0;
        }

    private:
        void getAllData(spi_inst_t * spi, uint cs, float * data)
        {
            uint8_t buf[14];

            reg_read(spi, cs, REGISTER_ADDRESS_OUT_TEMP_L, buf, 14);
#ifdef PRINT_DEBUG_STATEMENTS
            printf("Data: 0x%X, 0x%X, 0x%X, 0x%X, 0x%X, 0x%X\n", data[0], data[1], data[2], data[3], data[4], data[5]);
#endif

            data[0] = convertTempValue(buf[1], buf[0]);
            data[1] = convertGyroValue(buf[3], buf[2], gyroRange);
            data[2] = convertGyroValue(buf[5], buf[4], gyroRange);
            data[3] = convertGyroValue(buf[7], buf[6], gyroRange);
            data[4] = convertAccelValue(buf[9], buf[8], accelRange);
            data[5] = convertAccelValue(buf[11], buf[10], accelRange);
            data[6] = convertAccelValue(buf[13], buf[12], accelRange);
        }

        LSM6DSO_ODR_t refreshRate;
        uint calibrationPoints = 25;
        float nullCalibrationData[7];
        LowPassFilter filters[7];
        LSM6DSO_FS_G_t gyroRange = FS_G_500dps;
        LSM6DSO_FS_XL_t accelRange = FS_XL_4g;
};
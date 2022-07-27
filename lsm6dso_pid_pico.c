#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "include/lsm6dso_reg.h"
#include "include/lsm6dso_pid_pico.h"

#define WHOAMI_VALUE 0x6C
#define TEMPERATURE_OFFSET 25

//Functions for the library to use to write to SPI
static int32_t platform_write(void *handle, uint8_t Reg, const uint8_t *Bufp, uint16_t len)
{
    uint8_t address = 0x00 | Reg;
    LSM6DSO_Pico * data = (LSM6DSO_Pico *) handle;

    gpio_put(data->cs_pin, 0);

    spi_write_blocking(data->spi_port, &address, 1);
    spi_write_blocking(data->spi_port, Bufp, len);

    //sleep_us(1);
    gpio_put(data->cs_pin, 1);

    return 0;
}

static int32_t platform_read(void *handle, uint8_t Reg, uint8_t *Bufp, uint16_t len)
{
    uint8_t address = 0x80 | Reg;
    LSM6DSO_Pico * data = (LSM6DSO_Pico *) handle;

    gpio_put(data->cs_pin, 0);

    spi_write_blocking(data->spi_port, &address, 1);
    uint num_bytes_read = spi_read_blocking(data->spi_port, 0, Bufp, len);

    //sleep_us(1);
    gpio_put(data->cs_pin, 1);

    if(num_bytes_read == len)
        return 0;
    else
        return 1;
}

void lsm6dso_init(LSM6DSO_Pico * handle, spi_inst_t * spi_port, uint cs_pin)
{
    handle->cs_pin = cs_pin;
    handle->spi_port = spi_port;

    gpio_init(handle->cs_pin);
    gpio_set_dir(handle->cs_pin, GPIO_OUT);
    gpio_put(handle->cs_pin, 1);

    handle->imu_device.write_reg = platform_write;
    handle->imu_device.read_reg = platform_read;
    handle->imu_device.handle = handle;
}

void lsm6dso_setupPort(LSM6DSO_Pico * handle, uint freq, uint sck_pin, uint tx_pin, uint rx_pin)
{
    spi_init(handle->spi_port, freq);

    spi_set_format(handle->spi_port, 8, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);
    
    gpio_set_function(sck_pin, GPIO_FUNC_SPI);
    gpio_set_function(tx_pin, GPIO_FUNC_SPI);
    gpio_set_function(rx_pin, GPIO_FUNC_SPI);
}

bool lsm6dso_check_communication(LSM6DSO_Pico * handle)
{
    uint8_t buf;
    lsm6dso_device_id_get(&(handle->imu_device), &buf);
    return buf == WHOAMI_VALUE;
}

void lsm6dso_setupDevice(LSM6DSO_Pico * handle, lsm6dso_odr_xl_t accelRate,
        lsm6dso_fs_xl_t accelRange, lsm6dso_odr_g_t gyroRate, lsm6dso_fs_g_t gyroRange)
{
    lsm6dso_reset_set(&(handle->imu_device), 1);

    handle->accelRange = accelRange;
    handle->gyroRange = gyroRange;

    lsm6dso_aux_xl_fs_mode_set(&(handle->imu_device), LSM6DSO_USE_SAME_XL_FS);
    lsm6dso_xl_data_rate_set(&(handle->imu_device), accelRate);
    lsm6dso_xl_full_scale_set(&(handle->imu_device), accelRange);
    lsm6dso_gy_data_rate_set(&(handle->imu_device), gyroRate);
    lsm6dso_gy_full_scale_set(&(handle->imu_device), gyroRange);
}

float convertAcceleration(int16_t raw, lsm6dso_fs_xl_t range)
{
    float multiple;
    switch(range)
    {
        case LSM6DSO_2g:      //We run in FS_XL_Mode = 0
            multiple = 0.061;
            break;
        case LSM6DSO_4g:
            multiple = 0.122;
            break;
        case LSM6DSO_8g:
            multiple = 0.244;
            break;
        case LSM6DSO_16g:
            multiple = 0.488;
            break;
    }

    return ((float) raw * multiple) / 1000.0;
}

float convertGyroscope(int16_t raw, lsm6dso_fs_g_t range)
{
    float multiple;
    switch(range)
    {
        case LSM6DSO_250dps:
            multiple = 8.75;
            break;
        case LSM6DSO_125dps:
            multiple = 4.375;
            break;
        case LSM6DSO_500dps:
            multiple = 17.5;
            break;
        case LSM6DSO_1000dps:
            multiple = 35.0;
            break;
        case LSM6DSO_2000dps:
            multiple = 70.0;
            break;
    }

    return ((float) raw * multiple) / 1000.0;
}

float convertTemperatureToCelsius(int16_t raw)
{
    int8_t rawMSB, rawLSB;

    rawMSB = (raw & 0xFF00) >> 8;
    rawLSB = (raw & 0x00FF) / 256.0;

    float result = (float) rawMSB;
    result += rawLSB;
    result += TEMPERATURE_OFFSET;

    return result;
}

float convertCelsiusToFarenheit(float celsius)
{
    return (celsius * 1.8) + 32;
}

bool lsm6dso_get_accel_data(LSM6DSO_Pico * handle, float * buf)
{
    lsm6dso_status_t status;
    int16_t raw[3];

    lsm6dso_status_get(&(handle->imu_device), NULL, &status);
    if(!status.drdy_xl) return 0;

    lsm6dso_acceleration_raw_get(&(handle->imu_device), raw);

    for(int i = 0; i < 3; i++)
        buf[i] = convertAcceleration(raw[i], handle->accelRange);

    return 1;
}

bool lsm6dso_get_gyro_data(LSM6DSO_Pico * handle, float * buf)
{
    lsm6dso_status_t status;
    int16_t raw[3];

    lsm6dso_status_get(&(handle->imu_device), NULL, &status);
    if(!status.drdy_g) return 0;

    lsm6dso_angular_rate_raw_get(&(handle->imu_device), raw);

    for(int i = 0; i < 3; i++)
        buf[i] = convertGyroscope(raw[i], handle->gyroRange);

    return 1;
}

bool lsm6dso_get_temp_data(LSM6DSO_Pico * handle, float * buf, LSM6DSO_Temperature_Type outputType)
{
    lsm6dso_status_t status;
    int16_t raw;

    lsm6dso_status_get(&(handle->imu_device), NULL, &status);
    if(!status.drdy_temp) return 0;

    lsm6dso_temperature_raw_get(&(handle->imu_device), &raw);

    *buf = convertTemperatureToCelsius(raw);
    if(outputType == TEMPERATURE_TYPE_FARENHEIT)
        *buf = convertCelsiusToFarenheit(*buf);

    return 1;
}

bool lsm6dso_get_2d_motion_data(LSM6DSO_Pico * handle, float * buf)
{
    lsm6dso_status_t status;
    int16_t raw[3];

    lsm6dso_status_get(&(handle->imu_device), NULL, &status);
    if(!status.drdy_xl) return 0;
    if(!status.drdy_g) return 0;

    lsm6dso_acceleration_raw_get(&(handle->imu_device), raw);

    buf[0] = convertAcceleration(raw[0], handle->accelRange);
    buf[1] = convertAcceleration(raw[1], handle->accelRange);

    lsm6dso_angular_rate_raw_get(&(handle->imu_device), raw);

    buf[2] = convertAcceleration(raw[2], handle->accelRange);

    return 1;
}

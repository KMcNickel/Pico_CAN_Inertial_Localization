#ifndef LSM6DSO_PID_PICO_H
#define LSM6DSO_PID_PICO_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "lsm6dso_reg.h"

typedef struct LSM6DSO_PID_Pico
{
        uint cs_pin;
        spi_inst_t * spi_port;
        stmdev_ctx_t imu_device;
        lsm6dso_fs_xl_t accelRange;
        lsm6dso_fs_g_t gyroRange;
} LSM6DSO_Pico;

typedef enum
{
        TEMPERATURE_TYPE_CELSIUS,
        TEMPERATURE_TYPE_FARENHEIT
} LSM6DSO_Temperature_Type;

void lsm6dso_init(LSM6DSO_Pico * handle, spi_inst_t * spi_port, uint cs_pin);
void lsm6dso_setupPort(LSM6DSO_Pico * handle, uint freq, uint sck_pin, uint tx_pin, uint rx_pin);
bool lsm6dso_check_communication(LSM6DSO_Pico * handle);
void lsm6dso_setupDevice(LSM6DSO_Pico * handle, lsm6dso_odr_xl_t accelRate,
        lsm6dso_fs_xl_t accelRange, lsm6dso_odr_g_t gyroRate, lsm6dso_fs_g_t gyroRange);
bool lsm6dso_get_accel_data(LSM6DSO_Pico * handle, float * buf);
bool lsm6dso_get_gyro_data(LSM6DSO_Pico * handle, float * buf);
bool lsm6dso_get_temp_data(LSM6DSO_Pico * handle, float * buf, LSM6DSO_Temperature_Type);

#ifdef __cplusplus
}
#endif

#endif
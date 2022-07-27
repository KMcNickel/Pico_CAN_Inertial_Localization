#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "include/mcp2515.h"
#include "include/lsm6dso_pid_pico.h"
#include "include/filters.hpp"
#include "include/dwm1001_uart_pico.h"

//CAN Interface
#define CAN_SCK_PIN 2
#define CAN_TX_PIN 3
#define CAN_RX_PIN 4
#define CAN_CS_PIN 5
//IMU Interface
#define IMU_SCK_PIN 10
#define IMU_TX_PIN 11
#define IMU_RX_PIN 12
#define IMU_CS_PIN 13
//UWB Interface
#define UWB_UART_TX_PIN 8
#define UWB_UART_RX_PIN 9
//STDIO Interface
#define STDIO_UART_TX_PIN 16
#define STDIO_UART_RX_PIN 17

#define STDIO_UART_PERIPHERAL uart0
#define UWB_UART_PERIPHERAL uart1
#define CANBUS_SPI_PERIPHERAL spi0
#define IMU_SPI_PERIPHERAL spi1

LSM6DSO_PID_Pico lsm6dso;
DWM1001_Device dwm1001;
LowPassFilter filter(0.015);

MCP2515 mcp2515(CANBUS_SPI_PERIPHERAL, CAN_CS_PIN, CAN_TX_PIN, CAN_RX_PIN, CAN_SCK_PIN, 500000);

void startupStdio()
{
    //Remove UART from the default pins and put it on the custom pins
    gpio_set_function(0, GPIO_FUNC_NULL);
    gpio_set_function(1, GPIO_FUNC_NULL);
    gpio_set_function(STDIO_UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(STDIO_UART_RX_PIN, GPIO_FUNC_UART);

    stdio_init_all();
}

void startupUWB()
{
    printf("Setting up DWM1001...\n");

    dwm1001_init(&dwm1001, UWB_UART_PERIPHERAL, UWB_UART_TX_PIN, UWB_UART_RX_PIN);
    if(!dwm1001_check_communication(&dwm1001))
    {
        printf("Failed to communicate with the DWM1001\n");
        while(1) sleep_ms(1);
    }

    printf("DWM1001 setup complete\n");
}

void startupIMU()
{
    printf("Setting up LSM6DSO...\n");

    lsm6dso_init(&lsm6dso, IMU_SPI_PERIPHERAL, IMU_CS_PIN);
    lsm6dso_setupPort(&lsm6dso, 10000000, IMU_SCK_PIN, IMU_TX_PIN, IMU_RX_PIN);
    if(!lsm6dso_check_communication(&lsm6dso))
    {
        printf("Failed to communicate with LSM6DSO\n");
        while(1) sleep_ms(1);
    }
    lsm6dso_setupDevice(&lsm6dso, LSM6DSO_XL_ODR_104Hz, LSM6DSO_8g, LSM6DSO_GY_ODR_104Hz, LSM6DSO_500dps);

    printf("LSM6DSO setup complete\n");
}

void startupCANBus()
{
    printf("Setting up MCP2515...\n");

    mcp2515.reset();
    mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
    mcp2515.setNormalMode();

    printf("MCP2515 setup complete\n");
}

void peripheralStartup()
{
    startupStdio();

    printf("Setting up Pins and Peripherals...\n");

    startupIMU();
    startupUWB();
    startupCANBus();

    printf("Pins and Peripherals setup complete\n");
}

int main ()
{
    dwm_pos_t position;
    dwm_loc_data_t location;
    location.p_pos = &position;
    peripheralStartup();

    while(true)
    {
        if(dwm1001_get_location(&dwm1001, &location))
        {
            printf("Location: X: %.2f, Y: %.2f, Z: %.2f, Qf: %d%%\r\n", position.x / 1000.0, position.y / 1000.0, position.z / 1000.0, position.qf);
        }
    }
}
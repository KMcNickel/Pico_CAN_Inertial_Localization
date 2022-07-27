#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "include/mcp2515.h"
#include "include/lsm6dso_pid_pico.h"
#include "include/filters.hpp"

#define CAN_NODE_ID 0x5
#define CAN_LOC_X_ADDRESS (CAN_NODE_ID << 5) | 0x1
#define CAN_LOC_Y_ADDRESS (CAN_NODE_ID << 5) | 0x2
#define CAN_LOC_Z_ADDRESS (CAN_NODE_ID << 5) | 0x3
#define CAN_LOC_T_ADDRESS (CAN_NODE_ID << 5) | 0x4

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
//STDIO Interface
#define STDIO_UART_TX_PIN 16
#define STDIO_UART_RX_PIN 17

#define STDIO_UART_PERIPHERAL uart0
#define CANBUS_SPI_PERIPHERAL spi0
#define IMU_SPI_PERIPHERAL spi1

LSM6DSO_PID_Pico lsm6dso;
repeating_timer_t canTimer;
LowPassFilter xFilter(0.015);
LowPassFilter yFilter(0.015);
LowPassFilter zFilter(0.015);
absolute_time_t lastRxTime;

float motionData[3] = {0.0, 0.0, 0.0};

MCP2515 mcp2515(CANBUS_SPI_PERIPHERAL, CAN_CS_PIN, CAN_TX_PIN, CAN_RX_PIN, CAN_SCK_PIN, 500000);

bool canTimerCallback(repeating_timer_t * timer)
{
    can_frame frame;
    int32_t deltaTime = absolute_time_diff_us(lastRxTime, get_absolute_time()) / 1000;

    frame.can_id = CAN_LOC_X_ADDRESS;
    frame.can_dlc = 4;
    memcpy(frame.data, xFilter.CurrentValue(), 4);
    mcp2515.sendMessage(&frame);
    frame.can_id = CAN_LOC_Y_ADDRESS;
    frame.can_dlc = 4;
    memcpy(frame.data, yFilter.CurrentValue(), 4);
    mcp2515.sendMessage(&frame);
    frame.can_id = CAN_LOC_Z_ADDRESS;
    frame.can_dlc = 4;
    memcpy(frame.data, zFilter.CurrentValue(), 4);
    mcp2515.sendMessage(&frame);
    frame.can_id = CAN_LOC_T_ADDRESS;
    frame.can_dlc = 4;
    memcpy(frame.data, &deltaTime, 4);
    mcp2515.sendMessage(&frame);

    return true;
}

void startupStdio()
{
    //Remove UART from the default pins and put it on the custom pins
    gpio_set_function(0, GPIO_FUNC_NULL);
    gpio_set_function(1, GPIO_FUNC_NULL);
    gpio_set_function(STDIO_UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(STDIO_UART_RX_PIN, GPIO_FUNC_UART);

    stdio_init_all();
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

void startupAlarms()
{
    alarm_pool_init_default();
    add_repeating_timer_ms(100, canTimerCallback, NULL, &canTimer);
}

void peripheralStartup()
{
    startupStdio();

    printf("Setting up Pins and Peripherals...\n");

    startupIMU();
    startupCANBus();

    printf("Pins and Peripherals setup complete\n");
}

int main ()
{
    peripheralStartup();

    while(!lsm6dso_get_2d_motion_data(&lsm6dso, motionData));

    xFilter.setInitialValue(motionData[0]);
    yFilter.setInitialValue(motionData[1]);
    zFilter.setInitialValue(motionData[2]);
    
    startupAlarms();

    while(true)
    {
        if(lsm6dso_get_2d_motion_data(&lsm6dso, motionData))
        {
            xFilter.addNewData(&motionData[0]);
            yFilter.addNewData(&motionData[1]);
            zFilter.addNewData(&motionData[2]);
            printf("Motion Data: %.4f, %.4f, %.4f\r\n", xFilter.getCurrentValue(), yFilter.getCurrentValue(), zFilter.getCurrentValue());

            lastRxTime = get_absolute_time();
        }
    }
}
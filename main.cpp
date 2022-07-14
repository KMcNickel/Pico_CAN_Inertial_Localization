#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "include/pico_lsm6dso.hpp"
#include "include/mcp2515.h"

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
//UART Interface
#define UART_TX_PIN 16
#define UART_RX_PIN 17

#define STDIO_UART_PERIPHERAL uart0
#define CANBUS_SPI_PERIPHERAL spi0
#define IMU_SPI_PERIPHERAL spi1

PICO_LSM6DSO lsm6dso;

MCP2515 mcp2515(CANBUS_SPI_PERIPHERAL, CAN_CS_PIN, CAN_TX_PIN, CAN_RX_PIN, CAN_SCK_PIN, 1000 * 1000);

void startupStdioUART()
{
    //Remove UART from the default pins and put it on the custom pins
    gpio_set_function(0, GPIO_FUNC_NULL);
    gpio_set_function(1, GPIO_FUNC_NULL);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

}

void startupIMU()
{
    gpio_init(IMU_CS_PIN);
    gpio_set_dir(IMU_CS_PIN, GPIO_OUT);
    gpio_put(IMU_CS_PIN, 1);

    spi_init(IMU_SPI_PERIPHERAL, 1000 * 1000);

    spi_set_format(IMU_SPI_PERIPHERAL, 8, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);
    
    gpio_set_function(IMU_SCK_PIN, GPIO_FUNC_SPI);
    gpio_set_function(IMU_TX_PIN, GPIO_FUNC_SPI);
    gpio_set_function(IMU_RX_PIN, GPIO_FUNC_SPI);

    printf("Setting up LSM6DSO...\n");
    if(!lsm6dso.initialize(IMU_SPI_PERIPHERAL, IMU_CS_PIN, lsm6dso.ODR_416Hz))
    {
        printf("Setup failed!");
        while(true);
    }
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
    startupStdioUART(); //We have to do this before stdio will work
    stdio_init_all();

    printf("Setting up Pins and Peripherals...\n");

    startupIMU();
    //startupCANBus();

    printf("Pins and Peripherals setup complete\n");
}

void calculateLinearVelocity(float acceleration, float * currentValue, uint timeMs)
{
    *currentValue = *currentValue + (acceleration * 9.8) * (timeMs / 1000.0);
}

int main ()
{
    float imuData[7];
    float temperature = 0.0;
    float linearVelocities[3] = {0.0, 0.0, 0.0};
    float angularVelocities[3] = {0.0, 0.0, 0.0};
    can_frame frame;

    peripheralStartup();

    sleep_ms(1000);

    while(true)
    {
        lsm6dso.readAllMeasurements(IMU_SPI_PERIPHERAL, IMU_CS_PIN, imuData);
        frame.can_id = 0x01;
        frame.can_dlc = 1;
        frame.data[0] += 1;
        //mcp2515.sendMessage(&frame);

        temperature = imuData[0];
        for(int i = 0; i < 3; i++)
        {
            calculateLinearVelocity(imuData[i + 1], &linearVelocities[i], 200);
            angularVelocities[i] = imuData[i + 4];
        }

        /*printf("Vals: %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f\n", temperature,
                linearVelocities[0], linearVelocities[1], linearVelocities[2],
                angularVelocities[0], angularVelocities[1], angularVelocities[2]);*/
        sleep_ms(10);
    }
}
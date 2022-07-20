#ifndef DWM1001_UART_PICO_H
#define DWM1001_UART_PICO_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "dwm_api.h"

typedef struct
{
    uart_inst_t * uart;
    dwm1001_ctx_t uwb_device;
} DWM1001_Device;

void dwm1001_init(DWM1001_Device * handle, uart_inst_t * uartPort, uint tx_pin, uint rx_pin);
bool dwm1001_check_communication(DWM1001_Device * handle);

#ifdef __cplusplus
}
#endif

#endif
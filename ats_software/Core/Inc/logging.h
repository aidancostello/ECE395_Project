#ifndef __LOGGING_H
#define __LOGGING_H

#include "stm32l4xx_hal.h"
#include "cmsis_os.h"

#define UART_TIMEOUT 1000
#define MUTEX_TIMEOUT portMAX_DELAY
#define DOUBLE_BUF_SIZE 50

// stores uart handle and uart mutex for future use
// must be called before any other log functions
void log_init(UART_HandleTypeDef* uart_handle, osMutexId_t* uart_mtx);

// wrapper around uart transmit, logs a buffer
HAL_StatusTypeDef log_transmit_buf(uint8_t* buf, uint16_t len);

// prints a string over uart
HAL_StatusTypeDef log_print(const char* str);

// logs a double, append is character to append on the end, 0 if no append
HAL_StatusTypeDef log_print_double(double val, uint8_t precision);

#endif
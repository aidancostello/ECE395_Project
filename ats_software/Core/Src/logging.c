#include "logging.h"

// holds uart and mutex pointers
static UART_HandleTypeDef* m_uart_handle = NULL;
static osMutexId_t* m_uart_mtx = NULL;

// helper function for logging doubles
static uint8_t double_to_string(double val, uint8_t precision, char* buf);

void log_init(UART_HandleTypeDef* uart_handle, osMutexId_t* uart_mtx) {
    m_uart_handle = uart_handle;
    m_uart_mtx = uart_mtx;
}

HAL_StatusTypeDef log_transmit_buf(char* buf, size_t len) {
    if (m_uart_handle == NULL || m_uart_mtx == NULL) { return HAL_ERROR; }

    osMutexAcquire(*m_uart_mtx, MUTEX_TIMEOUT);
    HAL_StatusTypeDef status = HAL_UART_Transmit(m_uart_handle, (uint8_t*)buf, len, UART_TIMEOUT);
    osMutexRelease(*m_uart_mtx);

    return status;
}

HAL_StatusTypeDef log_transmit_double(double val, uint8_t precision, char append) {
    if (m_uart_handle == NULL || m_uart_mtx == NULL) { return HAL_ERROR; }

    char buf[DOUBLE_BUF_SIZE];
    uint8_t buf_len = double_to_string(val, precision, buf);

    if (append != 0) {
        buf[buf_len] = append;
        buf_len++;
    }

    return log_transmit_buf(buf, (size_t)buf_len);
}

static uint8_t double_to_string(double val, uint8_t precision, char* buf) {
    int start_idx = 0;

    // check negative
    if (val < 0) {
        buf[start_idx] = '-';
        val *= -1;
        start_idx++;
    }

    // extract decimal and non decimal parts
    uint64_t non_dec = (uint64_t)val;
    double dec = val - non_dec;
    int curr_idx = start_idx;

    // handle non decimal part (reversed)
    while (non_dec != 0) {
        buf[curr_idx] = '0' + non_dec%10;
        non_dec /= 10;
        curr_idx++;
    }

    // unreverse decimal part
    int end_idx = curr_idx - 1;
    while (start_idx < end_idx) {
        char store = buf[start_idx];
        buf[start_idx] = buf[end_idx];
        buf[end_idx] = store;
        start_idx++;
        end_idx--;
    }

    // leading 0 if pure decimal
    if (val < 1) {
        buf[curr_idx] = '0';
        curr_idx++;
    }

    // decimal place only if precision is not 0
    if (precision != 0) {
        buf[curr_idx] = '.';
        curr_idx++;
    }

    // handle decimal part
    for (uint8_t i = 0; i < precision; i++) {
        dec *= 10;
        uint8_t digit = (uint8_t)dec;
        buf[curr_idx] = '0' + digit;
        dec = dec - digit;
        curr_idx++;
    }

    return curr_idx;
}
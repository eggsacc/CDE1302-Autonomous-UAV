#include "platform.h"

int64_t host_time_us;
uint8_t host_uart_tx[512];
size_t host_uart_tx_size;
static uint8_t rx[512];
static size_t rx_size;

HOST_EXPORT void host_set_time(int64_t time_us) { host_time_us = time_us; }
int uart_write_bytes(int port, const void *data, size_t size) {
    assert(port == UART_NUM_1 && size <= sizeof(host_uart_tx));
    memcpy(host_uart_tx, data, size);
    host_uart_tx_size = size;
    return (int)size;
}
int uart_read_bytes(int port, void *data, uint32_t size, TickType_t timeout) {
    (void)timeout; assert(port == UART_NUM_1);
    if (size > rx_size) size = (uint32_t)rx_size;
    memcpy(data, rx, size);
    rx_size -= size;
    memmove(rx, rx + size, rx_size);
    return (int)size;
}
int uart_get_buffered_data_len(int port, size_t *size) {
    assert(port == UART_NUM_1); *size = rx_size; return 0;
}
void host_uart_receive(const uint8_t *data, size_t size) {
    assert(rx_size == 0 && size <= sizeof(rx));
    memcpy(rx, data, size); rx_size = size;
}

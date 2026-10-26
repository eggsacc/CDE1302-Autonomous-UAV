/* Test-only ESP-IDF boundary. No real UART, networking, tasks or flight hardware.
 * The production nav_task.c, mavlink_task.c and traj_yaw.c compile unchanged.
 * Any attempted network setup or scheduler entry aborts this offline harness.
 */
#pragma once
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#ifdef _WIN32
#define HOST_EXPORT __declspec(dllexport)
#else
#define HOST_EXPORT __attribute__((visibility("default")))
#endif
#define CONFIG_DRONE_ID 2
#define CONFIG_PX4_TX_PIN 1
#define CONFIG_PX4_RX_PIN 2
#define CONFIG_MAVROS_BRIDGE_IN_PORT 8888
#define CONFIG_MAVROS_BRIDGE_OUT_PORT 14550
#define CONFIG_HOST_IPV4_ADDR "127.0.0.1"
#define CONFIG_TOF_FRONT_SENSOR_IDX 0
#define MALLOC_CAP_SPIRAM 0
#define ESP_INTR_FLAG_IRAM 0
#define ESP_ERROR_CHECK(expr) assert((expr) == 0)
#define configASSERT(expr) assert(expr)
static inline void host_log(const char *tag, const char *format, ...) {
    (void)tag; (void)format;
}
#define ESP_LOGI host_log
#define ESP_LOGW host_log
#define ESP_LOGE host_log
#define ESP_LOGD host_log

extern int64_t host_time_us;
static inline int64_t esp_timer_get_time(void) { return host_time_us; }
static inline void *heap_caps_calloc(size_t n, size_t size, int caps) {
    (void)caps; return calloc(n, size);
}
typedef uint32_t TickType_t;
typedef int *SemaphoreHandle_t;
#define portMAX_DELAY UINT32_MAX
#define pdMS_TO_TICKS(ms) ((TickType_t)(ms))
static inline SemaphoreHandle_t xSemaphoreCreateMutex(void) {
    return calloc(1, sizeof(int));
}
static inline int xSemaphoreTake(SemaphoreHandle_t m, TickType_t timeout) {
    (void)timeout; assert(m && !*m); *m = 1; return 1;
}
static inline int xSemaphoreGive(SemaphoreHandle_t m) {
    assert(m && *m); *m = 0; return 1;
}
static inline TickType_t xTaskGetTickCount(void) {
    return (TickType_t)(host_time_us / 1000);
}
static inline void vTaskDelayUntil(TickType_t *previous, TickType_t delay) {
    (void)previous; (void)delay; abort();
}

typedef struct {
    int baud_rate, data_bits, parity, stop_bits, flow_ctrl, source_clk;
} uart_config_t;
#define UART_NUM_1 1
#define UART_DATA_8_BITS 8
#define UART_PARITY_DISABLE 0
#define UART_STOP_BITS_1 1
#define UART_HW_FLOWCTRL_DISABLE 0
#define UART_SCLK_DEFAULT 0
#define UART_PIN_NO_CHANGE -1
int uart_write_bytes(int port, const void *data, size_t size);
int uart_read_bytes(int port, void *data, uint32_t size, TickType_t timeout);
int uart_get_buffered_data_len(int port, size_t *size);
void host_uart_receive(const uint8_t *data, size_t size);
extern uint8_t host_uart_tx[512];
extern size_t host_uart_tx_size;
static inline int uart_driver_install(int a, int b, int c, int d, void *e, int f) {
    (void)a; (void)b; (void)c; (void)d; (void)e; (void)f; abort();
}
static inline int uart_param_config(int port, const uart_config_t *config) {
    (void)port; (void)config; abort();
}
static inline int uart_set_pin(int a, int b, int c, int d, int e) {
    (void)a; (void)b; (void)c; (void)d; (void)e; abort();
}

/* POSIX socket headers are not available in the Windows host compiler.
 * These declarations compile the unused QGC bridge and prohibit network I/O.
 */
struct in_addr { uint32_t s_addr; };
struct sockaddr { uint16_t sa_family; char sa_data[14]; };
struct sockaddr_in { uint16_t sin_family, sin_port; struct in_addr sin_addr; };
#define AF_INET 2
#define SOCK_DGRAM 2
#define IPPROTO_IP 0
#define INADDR_ANY 0
#define F_GETFL 3
#define F_SETFL 4
#define O_NONBLOCK 0x800
#define socket host_socket
#define bind host_bind
#define close host_close
#define fcntl host_fcntl
#define sendto host_sendto
#define recvfrom host_recvfrom
#define inet_aton host_inet_aton
#define htons host_htons
#define htonl host_htonl
static inline int host_socket(int a, int b, int c) {
    (void)a; (void)b; (void)c; abort();
}
static inline int host_bind(int a, const struct sockaddr *b, size_t c) {
    (void)a; (void)b; (void)c; abort();
}
static inline int host_close(int a) { (void)a; abort(); }
static inline int host_fcntl(int a, int b, int c) {
    (void)a; (void)b; (void)c; abort();
}
static inline int host_sendto(int a, const void *b, size_t c, int d,
                              const struct sockaddr *e, size_t f) {
    (void)a; (void)b; (void)c; (void)d; (void)e; (void)f; abort();
}
static inline int host_recvfrom(int a, void *b, size_t c, int d, void *e, void *f) {
    (void)a; (void)b; (void)c; (void)d; (void)e; (void)f; abort();
}
static inline int host_inet_aton(const char *a, struct in_addr *b) {
    (void)a; (void)b; abort();
}
static inline uint16_t host_htons(uint16_t a) { (void)a; abort(); }
static inline uint32_t host_htonl(uint32_t a) { (void)a; abort(); }

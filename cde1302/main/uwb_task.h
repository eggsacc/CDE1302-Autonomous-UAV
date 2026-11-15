#pragma once
/*
 * uwb_task — reads a Nooploop LinkTrack UWB tag on UART2 (RX only) and decodes
 * its NLink frames with nlink_parser.c.
 *
 * Bench-test stage (see mission/UWB_ESP32_BENCH_TEST.md):
 *   - serial monitor: throughput, error counters and the newest decoded frame
 *     every CONFIG_UWB_LOG_PERIOD_MS;
 *   - CONFIG_UWB_DEBUG_UDP: every raw frame, paired with PX4's local position
 *     at that instant, plus all counters, streamed to the laptop
 *     (CONFIG_HOST_IPV4_ADDR : CONFIG_UWB_DEBUG_PORT) in the format of
 *     uwb_debug_pack.h; read with mission/tools/uwb_sniff.py --udp.
 *   - CONFIG_UWB_SEND_TO_PX4: once the laptop has sent an alignment, every
 *     good fix goes to PX4 as ODOMETRY (uwb_px4.h, mavlink_task.c).  Off by
 *     default; see mission/UWB_PX4_FLIGHT_TEST.md.
 *
 * The task starts before Wi-Fi (which blocks until connected), so the serial
 * log works with no router; the UDP stream begins once Wi-Fi is up.
 *
 * Enabled only with CONFIG_UWB_ENABLED (menuconfig → "UWB (LinkTrack)"); with
 * it off the firmware is exactly as before.
 */

#include <stdbool.h>
#include <stdint.h>

#include "nlink_parser.h"

#define UWB_TASK_CORE       0
#define UWB_TASK_PRIORITY   3       /* below mavlink_task (5), above wifi_task (2) */
#define UWB_TASK_STACK      4096

typedef struct {
    bool            running;          /* UART installed and task reading */
    nlink_reading_t last;             /* newest decoded reading (if frames_decoded > 0) */
    int64_t         last_rx_us;       /* esp_timer time that reading completed (0 = none) */
    uint32_t        frames_decoded;   /* TAG_FRAME0 / NODE_FRAME2 / NODE_FRAME3 */
    uint32_t        frames_other;     /* valid frames of other kinds (not decoded) */
    nlink_stats_t   parser;           /* bytes, checksum / length errors, skipped bytes */
    uint32_t        uart_overflows;   /* RX FIFO or ring buffer overflowed: bytes lost */
    uint32_t        uart_frame_errors;/* UART framing errors: usually a baud mismatch */
    uint32_t        debug_packets_sent;
    uint32_t        debug_send_errors;   /* sendto() failed */
    uint32_t        debug_records_dropped;/* frames not forwarded (no Wi-Fi yet / queue full / too long) */
    uint32_t        px4_fed;          /* fixes handed to mavlink_task */
    uint32_t        px4_rejected;     /* aligned, but bad precision / no PX4 position */
    uint32_t        align_received;   /* valid CMD_UWB_ALIGN datagrams */
    bool            aligned;          /* an alignment is in use */
} uwb_state_t;

/* Validate the pin settings and install the UART driver.  Returns false (and
 * logs why) if UWB is disabled or misconfigured; do not start the task then. */
bool uwb_task_init(void);

/* FreeRTOS task: xTaskCreatePinnedToCore(uwb_task, "uwb", UWB_TASK_STACK,
 * NULL, UWB_TASK_PRIORITY, NULL, UWB_TASK_CORE) after uwb_task_init(). */
void uwb_task(void *arg);

/* Thread-safe copy of the current state (zeroed if the task never started). */
void uwb_get_state(uwb_state_t *out);

/* Handle a CMD_UWB_ALIGN datagram (uwb_px4.h).  Ignored unless
 * CONFIG_UWB_SEND_TO_PX4 is on and the task is running. */
void uwb_handle_align_cmd(const uint8_t *buf, int len);

/* Forget the alignment, so nothing reaches PX4 until a new one arrives. */
void uwb_clear_alignment(void);

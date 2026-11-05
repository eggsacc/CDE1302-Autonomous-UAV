/*
 * uwb_debug_pack — byte layout of the UWB debug stream (ESP32 → laptop UDP).
 *
 * Pure C99 (no ESP-IDF).  The Python reader is parse_debug_packet() in
 * laptop/simple_uwb_calibration.py: change both together and bump UWB_DBG_VERSION.
 *
 * All multi-byte fields little-endian.  One UDP packet =
 *
 *   header (64 B)
 *     0  char[4]  magic "UWBD"
 *     4  u8       version (2)
 *     5  u8       drone id
 *     6  u16      record count
 *     8  u32      packet sequence number (gaps = packets lost on Wi-Fi)
 *    12  u32      ESP32 time when sent (ms since boot)
 *    16  u32 × 12 ESP32 counters: bytes, frames_decoded, frames_other,
 *                 bad_checksum, bad_length, skipped_bytes, uart_overflows,
 *                 uart_frame_errors, records_dropped (frames not forwarded),
 *                 px4_fed (fixes handed to PX4), px4_rejected (not sent: bad
 *                 precision / no PX4 position), align_received (valid
 *                 CMD_UWB_ALIGN datagrams)
 *   then `record count` records (28 B + frame)
 *     0  u64      ESP32 time the frame completed (µs since boot)
 *     8  f32 × 4  PX4 local x, y, z (m, NED) and heading (rad) at that time
 *    24  u16      age of that PX4 position (ms); 0xFFFF = none
 *    26  u16      frame length
 *    28  u8[]     the raw NLink frame, exactly as received
 */
#pragma once

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define UWB_DBG_VERSION       2
#define UWB_DBG_HEADER_SIZE   64
#define UWB_DBG_RECORD_FIXED  28
#define UWB_DBG_MAX_PACKET    1400      /* stays under one Ethernet/Wi-Fi MTU */
#define UWB_DBG_MAX_FRAME     256       /* longer frames are not forwarded */
#define UWB_DBG_PX4_AGE_NONE  0xFFFF

typedef struct {
    uint32_t bytes;
    uint32_t frames_decoded;
    uint32_t frames_other;
    uint32_t bad_checksum;
    uint32_t bad_length;
    uint32_t skipped_bytes;
    uint32_t uart_overflows;
    uint32_t uart_frame_errors;
    uint32_t records_dropped;
    uint32_t px4_fed;
    uint32_t px4_rejected;
    uint32_t align_received;
} uwb_dbg_counters_t;

typedef struct {
    uint64_t esp_time_us;
    float    px4_x, px4_y, px4_z, px4_heading;
    uint16_t px4_age_ms;
    uint16_t frame_len;
    uint8_t  frame[UWB_DBG_MAX_FRAME];
} uwb_dbg_record_t;

/* Write the header into buf (cap >= UWB_DBG_HEADER_SIZE).  Returns its size. */
size_t uwb_dbg_write_header(uint8_t *buf, uint8_t drone_id, uint16_t n_records,
                            uint32_t seq, uint32_t esp_time_ms,
                            const uwb_dbg_counters_t *c);

/* Append one record at buf[pos].  Returns the new end position, or 0 if the
 * record does not fit in `cap` (start a new packet then). */
size_t uwb_dbg_append_record(uint8_t *buf, size_t pos, size_t cap,
                             const uwb_dbg_record_t *r);

/* Overwrite the record count of a header already written at buf. */
void uwb_dbg_set_count(uint8_t *buf, uint16_t n_records);

#ifdef __cplusplus
}
#endif

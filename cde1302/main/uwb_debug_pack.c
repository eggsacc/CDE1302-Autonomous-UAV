/* uwb_debug_pack.c — see uwb_debug_pack.h for the byte layout. */
#include "uwb_debug_pack.h"

#include <string.h>

static void put_u16(uint8_t *b, uint16_t v)
{
    b[0] = (uint8_t)v;
    b[1] = (uint8_t)(v >> 8);
}

static void put_u32(uint8_t *b, uint32_t v)
{
    for (int i = 0; i < 4; i++) b[i] = (uint8_t)(v >> (8 * i));
}

static void put_u64(uint8_t *b, uint64_t v)
{
    for (int i = 0; i < 8; i++) b[i] = (uint8_t)(v >> (8 * i));
}

static void put_f32(uint8_t *b, float f)
{
    uint32_t u;
    memcpy(&u, &f, sizeof u);
    put_u32(b, u);
}

size_t uwb_dbg_write_header(uint8_t *buf, uint8_t drone_id, uint16_t n_records,
                            uint32_t seq, uint32_t esp_time_ms,
                            const uwb_dbg_counters_t *c)
{
    memcpy(buf, "UWBD", 4);
    buf[4] = UWB_DBG_VERSION;
    buf[5] = drone_id;
    put_u16(buf + 6, n_records);
    put_u32(buf + 8, seq);
    put_u32(buf + 12, esp_time_ms);
    const uint32_t v[12] = {c->bytes, c->frames_decoded, c->frames_other, c->bad_checksum,
                            c->bad_length, c->skipped_bytes, c->uart_overflows,
                            c->uart_frame_errors, c->records_dropped, c->px4_fed,
                            c->px4_rejected, c->align_received};
    for (int i = 0; i < 12; i++) put_u32(buf + 16 + 4 * i, v[i]);
    return UWB_DBG_HEADER_SIZE;
}

size_t uwb_dbg_append_record(uint8_t *buf, size_t pos, size_t cap,
                             const uwb_dbg_record_t *r)
{
    size_t len = r->frame_len > UWB_DBG_MAX_FRAME ? UWB_DBG_MAX_FRAME : r->frame_len;
    size_t end = pos + UWB_DBG_RECORD_FIXED + len;
    if (end > cap) return 0;
    uint8_t *b = buf + pos;
    put_u64(b, r->esp_time_us);
    put_f32(b + 8, r->px4_x);
    put_f32(b + 12, r->px4_y);
    put_f32(b + 16, r->px4_z);
    put_f32(b + 20, r->px4_heading);
    put_u16(b + 24, r->px4_age_ms);
    put_u16(b + 26, (uint16_t)len);
    memcpy(b + 28, r->frame, len);
    return end;
}

void uwb_dbg_set_count(uint8_t *buf, uint16_t n_records)
{
    put_u16(buf + 6, n_records);
}

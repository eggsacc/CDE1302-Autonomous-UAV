/*
 * Laptop test driver for main/uwb_debug_pack.c (with main/nlink_parser.c).
 *
 *   uwb_debug_host <capture.bin> <packets.out> <records_per_flush>
 *
 * Parses the capture with the drone's C parser, turns every frame into a
 * debug record with deterministic fake PX4 values (record i: x = 0.1 i,
 * y = -0.2 i, z = -0.5, heading = 0.01 i, age = i % 4 == 0 ? none : i % 1000),
 * and packs them exactly like uwb_task.c's flush_debug().  Each packet is
 * written as a little-endian u16 length followed by its bytes.  The pytest
 * suite reads them with mission/uwb_link.py and compares.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "nlink_parser.h"
#include "uwb_debug_pack.h"

#define MAX_RECORDS 4096

static uwb_dbg_record_t s_records[MAX_RECORDS];
static size_t s_n;

static void on_frame(const uint8_t *frame, size_t len, nlink_kind_t kind, void *ctx)
{
    (void)kind;
    (void)ctx;
    if (s_n >= MAX_RECORDS || len > UWB_DBG_MAX_FRAME) return;
    uwb_dbg_record_t *r = &s_records[s_n];
    r->esp_time_us = 1000000ULL + 20000ULL * s_n;
    r->px4_x = 0.1f * (float)s_n;
    r->px4_y = -0.2f * (float)s_n;
    r->px4_z = -0.5f;
    r->px4_heading = 0.01f * (float)s_n;
    r->px4_age_ms = (s_n % 4 == 0) ? UWB_DBG_PX4_AGE_NONE : (uint16_t)(s_n % 1000);
    r->frame_len = (uint16_t)len;
    memcpy(r->frame, frame, len);
    s_n++;
}

int main(int argc, char **argv)
{
    if (argc < 4) {
        fprintf(stderr, "usage: %s capture.bin packets.out records_per_flush\n", argv[0]);
        return 2;
    }
    size_t per_flush = (size_t)strtoul(argv[3], NULL, 10);
    if (per_flush == 0) per_flush = 1;

    FILE *in = fopen(argv[1], "rb");
    if (!in) { perror(argv[1]); return 2; }
    static nlink_parser_t parser;
    nlink_parser_init(&parser);
    uint8_t chunk[512];
    size_t n;
    while ((n = fread(chunk, 1, sizeof chunk, in)) > 0) {
        nlink_parser_feed(&parser, chunk, n, on_frame, NULL);
    }
    fclose(in);

    FILE *out = fopen(argv[2], "wb");
    if (!out) { perror(argv[2]); return 2; }
    static uint8_t pkt[UWB_DBG_MAX_PACKET];
    uint32_t seq = 0;
    size_t i = 0;
    while (i < s_n) {                       /* one "flush" of up to per_flush records */
        size_t flush_end = i + per_flush < s_n ? i + per_flush : s_n;
        do {
            uwb_dbg_counters_t c = {
                .bytes = (uint32_t)(1000 + i), .frames_decoded = (uint32_t)i,
                .frames_other = 7, .bad_checksum = 1, .bad_length = 2,
                .skipped_bytes = 3, .uart_overflows = 4, .uart_frame_errors = 5,
                .records_dropped = 6, .px4_fed = 8, .px4_rejected = 9, .align_received = 10,
            };
            size_t pos = uwb_dbg_write_header(pkt, 2, 0, seq++, (uint32_t)(5000 + i), &c);
            uint16_t count = 0;
            while (i < flush_end) {
                size_t end = uwb_dbg_append_record(pkt, pos, sizeof pkt, &s_records[i]);
                if (end == 0) break;
                pos = end;
                count++;
                i++;
            }
            uwb_dbg_set_count(pkt, count);
            uint8_t len_le[2] = {(uint8_t)pos, (uint8_t)(pos >> 8)};
            fwrite(len_le, 1, 2, out);
            fwrite(pkt, 1, pos, out);
        } while (i < flush_end);
    }
    fclose(out);
    printf("%lu records\n", (unsigned long)s_n);
    return 0;
}

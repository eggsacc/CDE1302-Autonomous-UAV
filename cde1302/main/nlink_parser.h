/*
 * nlink_parser — Nooploop NLink (LinkTrack UWB) frame finding and decoding.
 *
 * Pure C99, no ESP-IDF.  It was checked against a laptop parser on a real
 * capture from a LinkTrack P-A and on corrupted streams.
 *
 * Layout: UWB_documentation/NLink_V1.4.pdf.  Little-endian (confirmed on a
 * real P-A: supply voltage decodes to ~4.9 V, local_time steps 20 ms).
 *
 *   nlink_parser_t p;
 *   nlink_parser_init(&p);
 *   nlink_parser_feed(&p, bytes, n, on_frame, ctx);   // on_frame(frame, len, kind, ctx)
 *   ...
 *   nlink_reading_t r;
 *   if (nlink_decode(frame, len, &r) && r.has_fix) { use r.pos ... }
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Buffer must hold the largest accepted frame (ANCHOR_FRAME0 = 896 B,
 * length-field frames <= NLINK_MAX_VAR_LEN). */
#define NLINK_BUF_SIZE      2048
#define NLINK_MIN_VAR_LEN   7
#define NLINK_MAX_VAR_LEN   1024     /* longest variable-length frame accepted */
#define NLINK_MAX_ANCHORS   16       /* per decoded frame; extra blocks are counted */

typedef enum {
    NLINK_ANCHOR_FRAME0 = 0,
    NLINK_TAG_FRAME0,
    NLINK_NODE_FRAME0,
    NLINK_NODE_FRAME1,
    NLINK_NODE_FRAME2,
    NLINK_NODE_FRAME3,
    NLINK_NODE_FRAME4,
    NLINK_NODE_FRAME5,
    NLINK_NODE_FRAME6,
    NLINK_SETTING_FRAME0,
    NLINK_ERROR_FRAME0,
    NLINK_SYSTEM_COMMON_FRAME0,
    NLINK_KIND_COUNT
} nlink_kind_t;

/* Frame name, e.g. "NODE_FRAME2". */
const char *nlink_kind_name(nlink_kind_t kind);

typedef struct {
    uint32_t frames[NLINK_KIND_COUNT];  /* good frames per kind */
    uint32_t bad_checksum;              /* complete frames whose checksum failed */
    uint32_t bad_length;                /* length field out of bounds */
    uint32_t skipped_bytes;             /* bytes dropped while resynchronising */
    uint32_t total_bytes;
} nlink_stats_t;

typedef struct {
    uint8_t       buf[NLINK_BUF_SIZE];
    size_t        len;
    nlink_stats_t stats;
} nlink_parser_t;

/* Called once per complete, checksum-checked frame (header .. checksum). */
typedef void (*nlink_frame_cb)(const uint8_t *frame, size_t len,
                               nlink_kind_t kind, void *ctx);

void nlink_parser_init(nlink_parser_t *p);

/* Feed any number of bytes; calls cb for every frame completed.  Returns the
 * number of frames delivered. */
size_t nlink_parser_feed(nlink_parser_t *p, const uint8_t *data, size_t n,
                         nlink_frame_cb cb, void *ctx);

typedef struct {
    uint8_t role;
    uint8_t id;
    float   dis;        /* m */
    float   fp_rssi;    /* dB, first path (NaN for TAG_FRAME0) */
    float   rx_rssi;    /* dB, received (NaN for TAG_FRAME0) */
} nlink_anchor_t;

typedef struct {
    nlink_kind_t   kind;          /* TAG_FRAME0, NODE_FRAME2 or NODE_FRAME3 */
    uint8_t        role;
    uint8_t        id;
    bool           has_pose;      /* false for NODE_FRAME3 (ranges only) */
    float          pos[3];        /* m, module's anchor frame */
    float          vel[3];        /* m/s */
    float          eop[3];        /* m, estimated precision */
    bool           has_fix;       /* pose present, not "invalid", eop below its max */
    float          voltage;       /* V */
    uint32_t       local_time;    /* ms */
    uint32_t       system_time;   /* ms */
    uint8_t        n_anchors;     /* stored in anchors[] (TAG_FRAME0: always 8) */
    uint8_t        n_anchors_total; /* as reported by the frame (may exceed storage) */
    nlink_anchor_t anchors[NLINK_MAX_ANCHORS];
} nlink_reading_t;

/* Decode a frame a TAG outputs (TAG_FRAME0, NODE_FRAME2, NODE_FRAME3).
 * Returns false for any other kind or a malformed frame. */
bool nlink_decode(const uint8_t *frame, size_t len, nlink_reading_t *out);

#ifdef __cplusplus
}
#endif

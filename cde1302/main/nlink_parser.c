/*
 * nlink_parser.c — see nlink_parser.h.  Mirrors the LinkTrack NLink V1.4 spec
 * (same frame table, resync rule, checksum rules and no-fix rule) so the
 * laptop and the drone agree byte for byte; it was tested against a laptop parser
 * checks that on a real capture and on corrupted streams.
 */
#include "nlink_parser.h"

#include <math.h>
#include <string.h>

/* ---------------------------------------------------------------------------
 * Frame table: (header, function mark) -> kind, fixed length (0 = uint16
 * length field at bytes 2..3).  Frame types from NLink V1.4.
 * ------------------------------------------------------------------------- */
typedef struct {
    uint8_t      header;
    uint8_t      mark;
    nlink_kind_t kind;
    uint16_t     fixed_len;
} frame_type_t;

static const frame_type_t FRAME_TYPES[] = {
    {0x55, 0x00, NLINK_ANCHOR_FRAME0,        896},
    {0x55, 0x01, NLINK_TAG_FRAME0,           128},
    {0x55, 0x02, NLINK_NODE_FRAME0,            0},
    {0x55, 0x03, NLINK_NODE_FRAME1,            0},
    {0x55, 0x04, NLINK_NODE_FRAME2,            0},
    {0x55, 0x05, NLINK_NODE_FRAME3,            0},
    {0x55, 0x06, NLINK_NODE_FRAME4,            0},
    {0x55, 0x08, NLINK_NODE_FRAME5,            0},
    {0x55, 0x09, NLINK_NODE_FRAME6,            0},
    {0x54, 0x00, NLINK_SETTING_FRAME0,       128},
    {0x54, 0xFA, NLINK_ERROR_FRAME0,           0},
    {0x52, 0x00, NLINK_SYSTEM_COMMON_FRAME0,  32},
};
#define N_FRAME_TYPES (sizeof(FRAME_TYPES) / sizeof(FRAME_TYPES[0]))

static const char *const KIND_NAMES[NLINK_KIND_COUNT] = {
    "ANCHOR_FRAME0", "TAG_FRAME0", "NODE_FRAME0", "NODE_FRAME1", "NODE_FRAME2",
    "NODE_FRAME3", "NODE_FRAME4", "NODE_FRAME5", "NODE_FRAME6", "SETTING_FRAME0",
    "ERROR_FRAME0", "SYSTEM_COMMON_FRAME0",
};

const char *nlink_kind_name(nlink_kind_t kind)
{
    return (unsigned)kind < NLINK_KIND_COUNT ? KIND_NAMES[kind] : "UNKNOWN";
}

static const frame_type_t *lookup(uint8_t header, uint8_t mark)
{
    for (size_t i = 0; i < N_FRAME_TYPES; i++) {
        if (FRAME_TYPES[i].header == header && FRAME_TYPES[i].mark == mark) {
            return &FRAME_TYPES[i];
        }
    }
    return NULL;
}

static bool is_header_byte(uint8_t b)
{
    return b == 0x55 || b == 0x54 || b == 0x52;
}

/* ---------------------------------------------------------------------------
 * Little-endian field readers
 * ------------------------------------------------------------------------- */
static uint16_t rd_u16(const uint8_t *b) { return (uint16_t)(b[0] | (b[1] << 8)); }

static uint32_t rd_u32(const uint8_t *b)
{
    return (uint32_t)b[0] | ((uint32_t)b[1] << 8) | ((uint32_t)b[2] << 16)
           | ((uint32_t)b[3] << 24);
}

static int32_t rd_i24(const uint8_t *b)
{
    int32_t v = (int32_t)b[0] | ((int32_t)b[1] << 8) | ((int32_t)b[2] << 16);
    return (v & 0x800000) ? v - 0x1000000 : v;
}

static bool checksum_ok(nlink_kind_t kind, const uint8_t *f, size_t len)
{
    uint8_t sum = 0;
    for (size_t i = 0; i + 1 < len; i++) sum = (uint8_t)(sum + f[i]);
    if (sum == f[len - 1]) return true;
    /* NLink_V1.4.pdf lists ANCHOR_FRAME0's last byte as "Value = 0xEE". */
    return kind == NLINK_ANCHOR_FRAME0 && f[len - 1] == 0xEE;
}

/* ---------------------------------------------------------------------------
 * Stream parser
 * ------------------------------------------------------------------------- */
void nlink_parser_init(nlink_parser_t *p)
{
    memset(p, 0, sizeof *p);
}

static void drop(nlink_parser_t *p, size_t n)
{
    memmove(p->buf, p->buf + n, p->len - n);
    p->len -= n;
}

/* Index of the first known (header, mark) pair in the buffer, or -1. */
static long find_header(const nlink_parser_t *p)
{
    for (size_t i = 0; i + 1 < p->len; i++) {
        if (lookup(p->buf[i], p->buf[i + 1])) return (long)i;
    }
    return -1;
}

/* Extract every complete frame now in the buffer (nlink.py NLinkParser.feed). */
static size_t process(nlink_parser_t *p, nlink_frame_cb cb, void *ctx)
{
    size_t delivered = 0;
    for (;;) {
        long start = find_header(p);
        if (start < 0) {                    /* keep a lone trailing header byte */
            size_t keep = (p->len && is_header_byte(p->buf[p->len - 1])) ? 1 : 0;
            p->stats.skipped_bytes += (uint32_t)(p->len - keep);
            drop(p, p->len - keep);
            break;
        }
        if (start > 0) {
            p->stats.skipped_bytes += (uint32_t)start;
            drop(p, (size_t)start);
        }
        if (p->len < 2) break;
        const frame_type_t *t = lookup(p->buf[0], p->buf[1]);
        size_t length = t->fixed_len;
        if (length == 0) {
            if (p->len < 4) break;
            length = rd_u16(p->buf + 2);
            if (length < NLINK_MIN_VAR_LEN || length > NLINK_MAX_VAR_LEN) {
                p->stats.bad_length++;
                p->stats.skipped_bytes++;
                drop(p, 1);
                continue;
            }
        }
        if (p->len < length) break;         /* wait for the rest */
        if (checksum_ok(t->kind, p->buf, length)) {
            p->stats.frames[t->kind]++;
            if (cb) cb(p->buf, length, t->kind, ctx);
            delivered++;
            drop(p, length);
        } else {
            p->stats.bad_checksum++;
            p->stats.skipped_bytes++;
            drop(p, 1);                     /* maybe a false header: resync */
        }
    }
    return delivered;
}

size_t nlink_parser_feed(nlink_parser_t *p, const uint8_t *data, size_t n,
                         nlink_frame_cb cb, void *ctx)
{
    size_t delivered = 0;
    p->stats.total_bytes += (uint32_t)n;
    while (n > 0) {
        /* process() always leaves less than one maximum frame buffered, so
         * there is room for at least NLINK_BUF_SIZE - max frame bytes. */
        size_t room = NLINK_BUF_SIZE - p->len;
        size_t take = n < room ? n : room;
        memcpy(p->buf + p->len, data, take);
        p->len += take;
        data += take;
        n -= take;
        delivered += process(p, cb, ctx);
    }
    return delivered;
}

/* ---------------------------------------------------------------------------
 * Decoding
 * ------------------------------------------------------------------------- */
#define INVALID_I24  (-8388608)     /* the docs' "invalid" coordinate -8388 */
#define EOP_NO_FIX   255            /* eop byte at its maximum (2.55 m) */

/* Observed on a real P-A with no anchors: pos = (1, 1, 1), eop = 2.55 m on
 * every axis.  "No fix" = an invalid coordinate or an eop byte at its max. */
static void read_pose(nlink_reading_t *r, const uint8_t *pos, const uint8_t *vel,
                      const uint8_t *eop)
{
    bool fix = true;
    for (int k = 0; k < 3; k++) {
        int32_t raw = rd_i24(pos + 3 * k);
        r->pos[k] = raw / 1000.0f;
        r->vel[k] = rd_i24(vel + 3 * k) / 10000.0f;
        r->eop[k] = eop[k] / 100.0f;
        if (raw == INVALID_I24 || eop[k] == EOP_NO_FIX) fix = false;
    }
    r->has_pose = true;
    r->has_fix = fix;
}

/* Anchor blocks: role, id, dis(int24), fp_rssi, rx_rssi [, reserved]. */
static void read_ranges(nlink_reading_t *r, const uint8_t *f, size_t len,
                        size_t start, uint8_t count, size_t block)
{
    r->n_anchors_total = count;
    r->n_anchors = 0;
    for (uint8_t k = 0; k < count; k++) {
        size_t i = start + (size_t)k * block;
        if (i + block > len - 1) break;     /* whole block before the checksum */
        if (r->n_anchors >= NLINK_MAX_ANCHORS) break;
        nlink_anchor_t *a = &r->anchors[r->n_anchors++];
        a->role = f[i];
        a->id = f[i + 1];
        a->dis = rd_i24(f + i + 2) / 1000.0f;
        a->fp_rssi = f[i + 5] / -2.0f;
        a->rx_rssi = f[i + 6] / -2.0f;
    }
}

bool nlink_decode(const uint8_t *f, size_t len, nlink_reading_t *r)
{
    if (len < 2 || f[0] != 0x55) return false;
    memset(r, 0, sizeof *r);

    if (f[1] == 0x01) {                                    /* TAG_FRAME0 */
        if (len != 128) return false;
        r->kind = NLINK_TAG_FRAME0;
        r->id = f[2];
        r->role = f[3];
        read_pose(r, f + 4, f + 13, f + 117);
        r->local_time = rd_u32(f + 108);
        r->system_time = rd_u32(f + 112);
        r->voltage = rd_u16(f + 120) / 1000.0f;
        r->n_anchors = r->n_anchors_total = 8;
        for (int k = 0; k < 8; k++) {
            r->anchors[k].role = 0xFF;                     /* not in this frame */
            r->anchors[k].id = (uint8_t)k;
            r->anchors[k].dis = rd_i24(f + 22 + 3 * k) / 1000.0f;
            r->anchors[k].fp_rssi = NAN;
            r->anchors[k].rx_rssi = NAN;
        }
        return true;
    }
    if (len < 4 || rd_u16(f + 2) != len) return false;
    if (f[1] == 0x04) {                                    /* NODE_FRAME2 */
        if (len < 120) return false;
        r->kind = NLINK_NODE_FRAME2;
        r->role = f[4];
        r->id = f[5];
        r->system_time = rd_u32(f + 6);
        read_pose(r, f + 13, f + 22, f + 10);
        r->local_time = rd_u32(f + 102);
        r->voltage = rd_u16(f + 116) / 1000.0f;
        read_ranges(r, f, len, 119, f[118], 13);
        return true;
    }
    if (f[1] == 0x05) {                                    /* NODE_FRAME3 */
        if (len < 22) return false;
        r->kind = NLINK_NODE_FRAME3;
        r->role = f[4];
        r->id = f[5];
        r->local_time = rd_u32(f + 6);
        r->system_time = rd_u32(f + 10);
        r->voltage = rd_u16(f + 18) / 1000.0f;
        r->has_pose = false;
        r->has_fix = false;
        read_ranges(r, f, len, 21, f[20], 7);
        return true;
    }
    return false;
}

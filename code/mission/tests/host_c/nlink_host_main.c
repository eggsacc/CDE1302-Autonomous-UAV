/*
 * Laptop test driver for main/nlink_parser.c.
 *
 *   nlink_host <capture.bin> [chunk_bytes]
 *
 * Feeds the file to the parser `chunk_bytes` at a time (default 4096) and
 * prints one JSON object per frame, then one with the stats.  The pytest
 * suite runs the Python parser on the same bytes and compares.
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "nlink_parser.h"

static void print_num(float v)
{
    if (isnan(v)) {
        printf("null");
    } else {
        printf("%.9g", v);
    }
}

static void print_vec(const char *name, const float *v, int n)
{
    printf(",\"%s\":[", name);
    for (int i = 0; i < n; i++) {
        if (i) printf(",");
        print_num(v[i]);
    }
    printf("]");
}

static void on_frame(const uint8_t *frame, size_t len, nlink_kind_t kind, void *ctx)
{
    (void)ctx;
    printf("{\"kind\":\"%s\",\"len\":%u", nlink_kind_name(kind), (unsigned)len);
    nlink_reading_t r;
    if (nlink_decode(frame, len, &r)) {
        printf(",\"decoded\":true,\"role\":%u,\"id\":%u,\"has_pose\":%s,\"has_fix\":%s",
               r.role, r.id, r.has_pose ? "true" : "false", r.has_fix ? "true" : "false");
        printf(",\"local_time\":%lu,\"system_time\":%lu",
               (unsigned long)r.local_time, (unsigned long)r.system_time);
        printf(",\"voltage\":");
        print_num(r.voltage);
        if (r.has_pose) {
            print_vec("pos", r.pos, 3);
            print_vec("vel", r.vel, 3);
            print_vec("eop", r.eop, 3);
        }
        printf(",\"n_anchors_total\":%u,\"anchors\":[", r.n_anchors_total);
        for (int k = 0; k < r.n_anchors; k++) {
            const nlink_anchor_t *a = &r.anchors[k];
            printf("%s[%u,%u,", k ? "," : "", a->role, a->id);
            print_num(a->dis);
            printf(",");
            print_num(a->fp_rssi);
            printf(",");
            print_num(a->rx_rssi);
            printf("]");
        }
        printf("]");
    } else {
        printf(",\"decoded\":false");
    }
    printf("}\n");
}

int main(int argc, char **argv)
{
    if (argc < 2) {
        fprintf(stderr, "usage: %s capture.bin [chunk_bytes]\n", argv[0]);
        return 2;
    }
    size_t chunk = argc > 2 ? (size_t)strtoul(argv[2], NULL, 10) : 4096;
    if (chunk == 0) chunk = 1;
    FILE *fp = fopen(argv[1], "rb");
    if (!fp) {
        perror(argv[1]);
        return 2;
    }
    static nlink_parser_t parser;
    nlink_parser_init(&parser);
    unsigned char *buf = malloc(chunk);
    size_t n;
    while ((n = fread(buf, 1, chunk, fp)) > 0) {
        nlink_parser_feed(&parser, buf, n, on_frame, NULL);
    }
    fclose(fp);
    free(buf);

    const nlink_stats_t *st = &parser.stats;
    printf("{\"stats\":{\"frames\":{");
    int first = 1;
    for (int k = 0; k < NLINK_KIND_COUNT; k++) {
        if (!st->frames[k]) continue;
        printf("%s\"%s\":%lu", first ? "" : ",", nlink_kind_name((nlink_kind_t)k),
               (unsigned long)st->frames[k]);
        first = 0;
    }
    printf("},\"bad_checksum\":%lu,\"bad_length\":%lu,\"skipped_bytes\":%lu,"
           "\"total_bytes\":%lu}}\n",
           (unsigned long)st->bad_checksum, (unsigned long)st->bad_length,
           (unsigned long)st->skipped_bytes, (unsigned long)st->total_bytes);
    return 0;
}

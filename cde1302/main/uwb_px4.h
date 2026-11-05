/*
 * uwb_px4 — turn a UWB reading into a position in PX4's local frame.
 *
 * The laptop works out the alignment (laptop/simple_uwb_calibration.py) and sends it with
 * CMD_UWB_ALIGN; this file holds the parts that need no ESP-IDF:
 *
 *   p = R(theta) @ M @ u + t,  M = diag(1, -1) if mirror, else identity
 *
 * CMD_UWB_ALIGN, all multi-byte fields little-endian (15 B):
 *   0  u8   packet type (WIFI_PKT_CMD)
 *   1  u8   command (CMD_UWB_ALIGN)
 *   2  u8   flags: bit 0 = alignment valid (0 clears it), bit 1 = mirror
 *   3  f32  theta (rad)
 *   7  f32  tx (m)
 *  11  f32  ty (m)
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "nlink_parser.h"

#ifdef __cplusplus
extern "C" {
#endif

#define UWB_ALIGN_CMD_SIZE   15
#define UWB_ALIGN_FLAG_VALID 0x01
#define UWB_ALIGN_FLAG_MIRROR 0x02

typedef struct {
    bool  valid;
    float cos_t, sin_t;
    float mirror_sign;      /* -1 if the UWB y axis is flipped, else +1 */
    float tx, ty;
} uwb_xform_t;

/* Decode a CMD_UWB_ALIGN datagram.  Returns false, leaving *out unchanged, if
 * it is too short or a value is not finite.  Valid flag clear = "no alignment". */
bool uwb_align_parse(const uint8_t *buf, size_t len, uwb_xform_t *out);

/* UWB (x, y) in the anchor frame -> PX4 local (x = north, y = east). */
void uwb_xform_apply(const uwb_xform_t *xf, float ux, float uy, float *px, float *py);

/* May this reading be sent to PX4?  Needs a real x/y position with x/y
 * precision within max_eop_m.  The height and its precision are ignored (PX4
 * gets no height from UWB): the module often reports "no fix" only because its
 * z precision is at maximum, and dropping those leaves gaps that make PX4 stop
 * and restart UWB fusion, snapping to whatever reading comes next.  How much to
 * trust each reading is left to PX4, through uwb_xy_variance(). */
bool uwb_reading_usable(const nlink_reading_t *r, float max_eop_m);

/* Variance (m^2) of a reading's x/y position, from the module's precision. */
float uwb_xy_variance(const nlink_reading_t *r);

#ifdef __cplusplus
}
#endif

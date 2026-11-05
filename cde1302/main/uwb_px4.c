/* uwb_px4.c — see uwb_px4.h.  Pure C99, no ESP-IDF. */
#include "uwb_px4.h"

#include <math.h>
#include <string.h>

static float rd_f32(const uint8_t *p)
{
    uint32_t v = (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16
                 | (uint32_t)p[3] << 24;
    float f;
    memcpy(&f, &v, sizeof f);
    return f;
}

bool uwb_align_parse(const uint8_t *buf, size_t len, uwb_xform_t *out)
{
    if (len < UWB_ALIGN_CMD_SIZE) return false;
    uint8_t flags = buf[2];
    float theta = rd_f32(buf + 3);
    float tx = rd_f32(buf + 7);
    float ty = rd_f32(buf + 11);
    if (!(flags & UWB_ALIGN_FLAG_VALID)) {
        memset(out, 0, sizeof *out);
        return true;
    }
    if (!isfinite(theta) || !isfinite(tx) || !isfinite(ty)) return false;
    out->valid = true;
    out->cos_t = cosf(theta);
    out->sin_t = sinf(theta);
    out->mirror_sign = (flags & UWB_ALIGN_FLAG_MIRROR) ? -1.0f : 1.0f;
    out->tx = tx;
    out->ty = ty;
    return true;
}

void uwb_xform_apply(const uwb_xform_t *xf, float ux, float uy, float *px, float *py)
{
    float my = xf->mirror_sign * uy;
    *px = xf->cos_t * ux - xf->sin_t * my + xf->tx;
    *py = xf->sin_t * ux + xf->cos_t * my + xf->ty;
}

#define EOP_NO_FIX_M   2.55f    /* the module's "no estimate" value (raw 255) */
#define POS_LIMIT_M    1000.0f  /* beyond this the coordinate is the invalid marker */

bool uwb_reading_usable(const nlink_reading_t *r, float max_eop_m)
{
    for (int k = 0; k < 2; k++) {
        if (!isfinite(r->pos[k]) || fabsf(r->pos[k]) > POS_LIMIT_M) return false;
        if (r->eop[k] >= EOP_NO_FIX_M || r->eop[k] > max_eop_m) return false;
    }
    return r->has_pose;
}

float uwb_xy_variance(const nlink_reading_t *r)
{
    float e = r->eop[0] > r->eop[1] ? r->eop[0] : r->eop[1];
    return e * e;
}

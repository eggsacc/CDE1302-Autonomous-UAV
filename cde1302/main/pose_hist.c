/* pose_hist.c — see pose_hist.h.  Pure C99, no ESP-IDF. */
#include "pose_hist.h"

#include <math.h>

#define PI_F   3.14159265359f
#define TWO_PI 6.28318530718f

void pose_hist_init(pose_hist_t *h)
{
    h->head = 0;
    h->count = 0;
}

/* i-th sample, 0 = oldest. */
static const pose_sample_t *nth(const pose_hist_t *h, int i)
{
    int k = h->head - h->count + i;
    if (k < 0) k += POSE_HIST_N;
    return &h->s[k];
}

void pose_hist_push(pose_hist_t *h, const pose_sample_t *p)
{
    if (h->count > 0 && p->t_us <= nth(h, h->count - 1)->t_us) return;
    h->s[h->head] = *p;
    h->head = (h->head + 1) % POSE_HIST_N;
    if (h->count < POSE_HIST_N) h->count++;
}

static float wrap_pi(float a)
{
    a = fmodf(a + PI_F, TWO_PI);
    if (a < 0.0f) a += TWO_PI;
    return a - PI_F;
}

bool pose_hist_at(const pose_hist_t *h, int64_t t_us, int64_t max_gap_us,
                  pose_sample_t *out, float *yaw_rate, int64_t *gap_us)
{
    if (h->count == 0) return false;
    const pose_sample_t *first = nth(h, 0), *last = nth(h, h->count - 1);
    if (t_us < first->t_us) return false;

    const pose_sample_t *a, *b;
    if (t_us >= last->t_us) {
        if (t_us - last->t_us > max_gap_us) return false;
        b = last;
        a = h->count > 1 ? nth(h, h->count - 2) : last;
        *out = *last;
        out->t_us = t_us;
        *gap_us = t_us - last->t_us;
    } else {
        int lo = 0, hi = h->count - 1;           /* nth(lo) <= t < nth(hi) */
        while (hi - lo > 1) {
            int mid = (lo + hi) / 2;
            if (nth(h, mid)->t_us <= t_us) lo = mid; else hi = mid;
        }
        a = nth(h, lo);
        b = nth(h, hi);
        float f = (float)(t_us - a->t_us) / (float)(b->t_us - a->t_us);
        out->t_us = t_us;
        out->x = a->x + f * (b->x - a->x);
        out->y = a->y + f * (b->y - a->y);
        out->heading = wrap_pi(a->heading + f * wrap_pi(b->heading - a->heading));
        int64_t ga = t_us - a->t_us, gb = b->t_us - t_us;
        *gap_us = ga < gb ? ga : gb;
    }
    *yaw_rate = (b->t_us > a->t_us)
        ? wrap_pi(b->heading - a->heading) / ((float)(b->t_us - a->t_us) / 1e6f)
        : 0.0f;
    out->heading = wrap_pi(out->heading);
    return true;
}

/*
 * Laptop test driver for main/traj_gate.c.  Flies a simulated PX4 (P position
 * loop with a speed limit and a lag) along a path made of two straight legs:
 * leg_m North, then leg2_m East, sampled every 0.02 m (planned 0.1 m/s).
 *
 *   traj_gate_host <lookahead_m> <corner_m> <gain> <max_speed> <noise_m> <seconds>
 *                  <leg_m> <leg2_m> [freeze_s]
 *
 * noise_m adds position wobble to what the gate and the "PX4" see, like flow noise.
 * From freeze_s the position stops being fresh.  With leg2_m = 0 there is no corner.
 *
 * Output: done_s k x y end_err corner_err mean_speed speed_std max_gap
 *   corner_err: true distance to the corner when the target left it (-1 = never).
 *   speeds are over the middle of the first leg.
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "traj_gate.h"

#define TICK_S  0.05f
#define SPACING 0.02f
#define SAMPLE_S 0.2f

static int s_n1;        /* sample of the corner */

static void path(float k, float *x, float *y, void *ctx)
{
    (void)ctx;
    if (k <= (float)s_n1) {
        *x = k * SPACING;
        *y = 0.0f;
    } else {
        *x = (float)s_n1 * SPACING;
        *y = (k - (float)s_n1) * SPACING;
    }
}

/* Small repeatable wobble with a period of a few seconds. */
static float wobble(float t, float phase)
{
    return sinf(1.1f * t + phase) * 0.6f + sinf(2.7f * t + 2.0f * phase) * 0.4f;
}

int main(int argc, char **argv)
{
    if (argc < 9) return 2;
    float lookahead = (float)atof(argv[1]), corner_m = (float)atof(argv[2]);
    float gain = (float)atof(argv[3]), max_speed = (float)atof(argv[4]);
    float noise = (float)atof(argv[5]), seconds = (float)atof(argv[6]);
    float leg = (float)atof(argv[7]), leg2 = (float)atof(argv[8]);
    float freeze = argc > 9 ? (float)atof(argv[9]) : -1.0f;

    s_n1 = (int)(leg / SPACING + 0.5f);
    int n = s_n1 + (int)(leg2 / SPACING + 0.5f) + 1;

    traj_gate_t g;
    traj_gate_start(&g, n, SAMPLE_S, lookahead, corner_m);
    if (leg2 > 0.0f && corner_m > 0.0f) g.corner = s_n1;

    float x = 0, y = 0, vx = 0, vy = 0, done_s = -1, corner_err = -1, max_gap = 0;
    float sum = 0, sum2 = 0, prev_x = 0;
    int count = 0;

    for (int i = 0; i * TICK_S < seconds; i++) {
        float t = (float)i * TICK_S;
        float nx = x + noise * wobble(t, 0.0f), ny = y + noise * wobble(t, 1.3f);
        bool fresh = freeze < 0.0f || t < freeze;
        bool at_corner = g.corner == s_n1 && g.k >= (float)s_n1;
        traj_gate_result_t r = traj_gate_step(&g, TICK_S, nx, ny, fresh, path, NULL);
        if (at_corner && g.corner < 0 && corner_err < 0.0f) {
            corner_err = sqrtf((x - leg) * (x - leg) + y * y);
        }
        if (g.corner < 0) g.corner = n - 1;           /* what nav_task finds next */
        if (r == TRAJ_GATE_DONE && done_s < 0.0f) done_s = t;

        float tx, ty;
        path(g.k, &tx, &ty, NULL);
        float gap = sqrtf((tx - x) * (tx - x) + (ty - y) * (ty - y));
        if (gap > max_gap) max_gap = gap;

        float cx = gain * (tx - nx), cy = gain * (ty - ny);
        float c = sqrtf(cx * cx + cy * cy);
        if (c > max_speed && c > 0.0f) {
            cx *= max_speed / c;
            cy *= max_speed / c;
        }
        vx += (cx - vx) * 0.25f;                      /* about 0.2 s response */
        vy += (cy - vy) * 0.25f;
        x += vx * TICK_S;
        y += vy * TICK_S;

        if (x > 0.25f * leg && x < 0.75f * leg && i % 10 == 0) {
            float v = (x - prev_x) / (10.0f * TICK_S);
            sum += v;
            sum2 += v * v;
            count++;
        }
        if (i % 10 == 0) prev_x = x;
    }
    float mean = count ? sum / (float)count : 0.0f;
    float var = count ? sum2 / (float)count - mean * mean : 0.0f;
    float tx, ty;
    path((float)g.last, &tx, &ty, NULL);
    float end_err = sqrtf((x - tx) * (x - tx) + (y - ty) * (y - ty));
    printf("%.3f %.3f %.3f %.3f %.3f %.3f %.4f %.4f %.3f\n", (double)done_s, (double)g.k,
           (double)x, (double)y, (double)end_err, (double)corner_err, (double)mean,
           (double)sqrtf(var > 0.0f ? var : 0.0f), (double)max_gap);
    return 0;
}

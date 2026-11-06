/*
 * Laptop test driver for main/pose_hist.c.  Pushes 200 samples (more than the
 * ring holds), 20 ms apart from t = 1 s: x = 0.02 i, y = -0.01 i,
 * heading = 2.0 + 0.01 i (wrapped: crosses +-pi between samples 114 and 115), plus one out-of-order
 * sample that must be ignored.  Then looks up each time (us) given:
 *
 *   pose_hist_host <t_us> [<t_us> ...]
 *
 * Output per time: "1 x y heading yaw_rate gap_us" or "0".
 */
#include <stdio.h>
#include <stdlib.h>

#include "pose_hist.h"

int main(int argc, char **argv)
{
    static pose_hist_t h;
    pose_hist_init(&h);
    for (int i = 0; i < 200; i++) {
        float hd = 2.0f + 0.01f * (float)i;
        while (hd > 3.14159265f) hd -= 6.28318531f;
        pose_sample_t p = { 1000000 + (int64_t)i * 20000, 0.02f * (float)i,
                            -0.01f * (float)i, hd };
        pose_hist_push(&h, &p);
        if (i == 150) {
            pose_sample_t old = { 1000000, 99.0f, 99.0f, 0.0f };
            pose_hist_push(&h, &old);
        }
    }
    for (int a = 1; a < argc; a++) {
        pose_sample_t o;
        float rate;
        int64_t gap;
        if (pose_hist_at(&h, atoll(argv[a]), 100000, &o, &rate, &gap)) {
            printf("1 %.5f %.5f %.5f %.5f %lld\n", (double)o.x, (double)o.y,
                   (double)o.heading, (double)rate, (long long)gap);
        } else {
            printf("0\n");
        }
    }
    return 0;
}

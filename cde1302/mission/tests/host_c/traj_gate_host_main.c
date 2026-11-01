/*
 * Laptop test driver for main/traj_gate.c.  Flies a simulated drone along a
 * straight trajectory at a speed limit, and prints one line per run:
 *
 *   traj_gate_host <points> <sample_s> <leash_m> <max_speed> <ticks> [freeze_tick]
 *                  [corner_m] [corner_sample]
 *
 * The setpoint is at k * sample_s * 0.1 m along the line (0.1 m/s nominal).
 * From tick freeze_tick the simulated position stops being fresh.
 * With corner_sample the only corner is there, and the output gains
 * "<first tick k passed it> <error then>".
 * Output: "<k> <drone_m> <max_err_m> <result of the last tick> <done_tick>"
 */
#include <stdio.h>
#include <stdlib.h>

#include "traj_gate.h"

#define TICK_S 0.05f

int main(int argc, char **argv)
{
    if (argc < 6) return 2;
    int n = atoi(argv[1]);
    float sample_s = (float)atof(argv[2]);
    float leash = (float)atof(argv[3]);
    float max_speed = (float)atof(argv[4]);
    int ticks = atoi(argv[5]);
    int freeze = argc > 6 ? atoi(argv[6]) : -1;

    float corner_m = argc > 7 ? (float)atof(argv[7]) : 0.0f;
    int corner_at = argc > 8 ? atoi(argv[8]) : -1;

    traj_gate_t g;
    traj_gate_start(&g, n, sample_s, leash, corner_m);
    if (corner_at >= 0) g.corner = corner_at;
    int passed_tick = -1;
    float passed_err = 0.0f;
    float drone = 0.0f, max_err = 0.0f;
    int done_tick = -1;
    traj_gate_result_t res = TRAJ_GATE_WAIT;

    for (int t = 0; t < ticks; t++) {
        float sp = g.k * sample_s * 0.1f;
        float err = sp - drone;
        if (err > max_err) max_err = err;
        res = traj_gate_step(&g, TICK_S, err, freeze < 0 || t < freeze);
        if (g.corner < 0) g.corner = n - 1;              /* what nav_task finds next */
        if (corner_at >= 0 && passed_tick < 0 && g.k > (float)corner_at) {
            passed_tick = t;
            passed_err = err;
        }
        if (res == TRAJ_GATE_DONE) {
            if (done_tick < 0) done_tick = t;
            continue;
        }
        float step = err * 1.0f + 0.1f;                  /* P loop + feedforward */
        if (step > max_speed) step = max_speed;
        drone += step * TICK_S;
    }
    printf("%.4f %.4f %.4f %d %d %d %.4f\n", (double)g.k, (double)drone, (double)max_err,
           (int)res, done_tick, passed_tick, (double)passed_err);
    return 0;
}

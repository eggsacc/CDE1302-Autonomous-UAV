/*
 * traj_gate — decides how far along an uploaded trajectory the setpoint may go.
 *
 * Pure C99 (no ESP-IDF), so it is tested on the laptop
 * (mission/tests/test_traj_gate_c.py).
 *
 * With a leash the setpoint runs at full speed while the drone is within
 * TRAJ_GATE_FULL_SPEED of leash_m, then eases down to a stop at leash_m, so the
 * drone flies every lane and corner however slow it is, and "done" means it
 * really reached the last point.  Easing matters: switching hard between go
 * and stop makes the drone surge and fall back.  The check uses the position the
 * ESP32 reads from PX4 over UART, so Wi-Fi gaps do not affect it.
 * Corners are stricter than lanes: the setpoint stops on each one until the
 * drone is within corner_m, and the last point counts as a corner.
 * leash_m <= 0 plays on the clock, as if the drone always kept up.
 */
#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Longest time one step may count, so a stalled task cannot jump the setpoint. */
#define TRAJ_GATE_MAX_STEP_S 0.25f
/* Fraction of the leash at which the setpoint starts to slow down.  It is also
 * how close the drone must get to the last point to finish. */
#define TRAJ_GATE_FULL_SPEED 0.6f

typedef enum {
    TRAJ_GATE_MOVE,     /* the setpoint moved on */
    TRAJ_GATE_WAIT,     /* the setpoint waits for the drone */
    TRAJ_GATE_DONE,     /* finished: the drone is at the last point */
} traj_gate_result_t;

typedef struct {
    float k;            /* position along the trajectory, in samples */
    int   last;         /* index of the last sample */
    float sample_s;     /* time between samples at full speed */
    float leash_m;      /* <= 0: no position check */
    float rate;         /* last step's speed, 0..1 of the planned speed */
    float corner_m;     /* how close to get to a corner; <= 0: no corner stops */
    int   corner;       /* sample of the next corner; -1: caller must set the next */
} traj_gate_t;

void traj_gate_start(traj_gate_t *g, int n_points, float sample_s, float leash_m,
                     float corner_m);

/* One tick.  elapsed_s: time since the previous tick.  err_m: distance from the
 * drone to the current setpoint.  pos_ok: PX4's position is fresh. */
traj_gate_result_t traj_gate_step(traj_gate_t *g, float elapsed_s, float err_m, bool pos_ok);

#ifdef __cplusplus
}
#endif

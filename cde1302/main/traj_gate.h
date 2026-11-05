/*
 * traj_gate — decides where along an uploaded trajectory the target goes.
 *
 * Pure C99 (no ESP-IDF).
 *
 * The target sits lookahead_m ahead of the drone along the path.  PX4 flies at
 * about gain x gap, so a constant gap gives a steady speed, and a position wobble
 * of a few cm is small against the gap.  There is no clock: the target only moves
 * when the drone does, so the drone flies every lane however slow it is.
 *
 * Corners are stricter than lanes: the target stops on each one until the drone
 * is within corner_m, and the last point counts as a corner.
 *
 * lookahead_m <= 0 plays on the clock, as if the drone always kept up.
 * The check uses the position the ESP32 reads from PX4 over UART, so Wi-Fi gaps
 * do not affect it.
 */
#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Longest time one step may count, so a stalled task cannot jump the target. */
#define TRAJ_GATE_MAX_STEP_S 0.25f
/* The target moves at most this many times the planned speed. */
#define TRAJ_GATE_MAX_RATE   4.0f
/* Without corner stops, the last point counts as reached this close (fraction
 * of the lookahead). */
#define TRAJ_GATE_END_FRACTION 0.6f

typedef enum {
    TRAJ_GATE_MOVE,     /* the target may have moved on */
    TRAJ_GATE_WAIT,     /* the target waits for the drone */
    TRAJ_GATE_DONE,     /* finished: the drone is at the last point */
} traj_gate_result_t;

typedef struct {
    float k;            /* target position along the trajectory, in samples */
    int   last;         /* index of the last sample */
    float sample_s;     /* time between samples at the planned speed */
    float lookahead_m;  /* <= 0: play on the clock */
    float corner_m;     /* how close to get to a corner; <= 0: no corner stops */
    int   corner;       /* sample of the next corner; -1: caller must set the next */
} traj_gate_t;

/* Horizontal position of the path at sample k (may be fractional). */
typedef void (*traj_point_fn)(float k, float *x, float *y, void *ctx);

void traj_gate_start(traj_gate_t *g, int n_points, float sample_s, float lookahead_m,
                     float corner_m);

/* One tick.  elapsed_s: time since the previous tick.  drone_x/y: PX4's position.
 * pos_ok: that position is fresh.  Afterwards g->k is the target. */
traj_gate_result_t traj_gate_step(traj_gate_t *g, float elapsed_s, float drone_x, float drone_y,
                                  bool pos_ok, traj_point_fn point, void *ctx);

#ifdef __cplusplus
}
#endif

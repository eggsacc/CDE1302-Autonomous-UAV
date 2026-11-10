/*
 * traj_gate — decides where along an uploaded trajectory the target goes.
 *
 * Pure C99 (no ESP-IDF), so it is tested on the laptop
 * (mission/tests/test_traj_gate_c.py).
 *
 * The target sits lookahead_m ahead of the drone along the path.  PX4 flies at
 * about gain x gap, so a constant gap gives a steady speed, and a position wobble
 * of a few cm is small against the gap.  There is no clock: the target only moves
 * when the drone does, so the drone flies every lane however slow it is.
 *
 * Corners are stricter than lanes: the target stops on each one until the drone
 * is within corner_m, and the last point counts as a corner.
 *
 * Waypoint mode (traj_gate_start_waypoints): the target jumps straight to the
 * next corner and stays there until the drone is within corner_m, then jumps to
 * the one after.  PX4 then accelerates and slows down by itself, at up to
 * MPC_XY_VEL_MAX, instead of chasing a target that creeps along the path.
 *
 * Follow mode (traj_gate_start_follow): steady speed from velocity feedforward.
 * The carrot above makes PX4's speed = gain x gap, so anything that changes the
 * gap changes the speed: it brakes exponentially into every corner stop,
 * surges at each lane start, and stalls whenever the gate waits.  Here the gate instead tracks the drone's
 * PROGRESS (its closest point on the path, never backwards, at most
 * TRAJ_GATE_FOLLOW_RATE x the planned speed), puts the target just ahead of it
 * (TRAJ_GATE_FOLLOW_LEAD_S of planned travel, to cover the loop delay), and
 * outputs g->speed: the planned speed, ramped up at accel after a corner and
 * down along a braking curve to TRAJ_GATE_CORNER_SPEED by corner_m before the
 * next one (TRAJ_GATE_BRAKE_LAG_S early).  The caller sends it as velocity
 * feedforward along the path, so PX4's position error is only the sideways
 * correction plus a small lead.  Once the progress reaches a corner the
 * feedforward is 0: PX4 just holds the corner, until the drone is within
 * corner_m, or within 2 x corner_m after TRAJ_GATE_CORNER_SETTLE_S, or
 * TRAJ_GATE_CORNER_MAX_WAIT_S have passed.
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
/* Follow mode: progress moves at most this many times the planned speed. */
#define TRAJ_GATE_FOLLOW_RATE 2.0f
/* Follow mode: the target leads the drone's progress by this much planned travel. */
#define TRAJ_GATE_FOLLOW_LEAD_S 0.1f
/* Follow mode: speed the braking curve reaches at corner_m from a corner (m/s). */
#define TRAJ_GATE_CORNER_SPEED 0.05f
/* Follow mode: smallest corner radius (m), so the drone cannot creep forever. */
#define TRAJ_GATE_FOLLOW_MIN_CORNER_M 0.03f
/* Follow mode: start braking this much travel time early, for PX4's lag (s). */
#define TRAJ_GATE_BRAKE_LAG_S 0.3f
/* Follow mode, holding a corner: after this long, 2 x corner_m is close enough (s)... */
#define TRAJ_GATE_CORNER_SETTLE_S 1.0f
/* ...and after this long, move on whatever the distance (s).  A drone that
 * wobbles 5-7 cm or holds a steady offset never "lands" in a 5 cm circle. */
#define TRAJ_GATE_CORNER_MAX_WAIT_S 3.0f

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
    bool  waypoints;    /* target = the next corner itself */
    bool  follow;       /* follow mode: progress + feedforward speed */
    float accel;        /* follow: speed-up / slow-down rate (m/s^2) */
    float proj;         /* follow: the drone's progress (closest point), in samples */
    float speed;        /* follow: feedforward speed along the path (m/s), output */
    float wait_s;       /* follow: time the progress has been held at the corner (s) */
} traj_gate_t;

/* Waypoint mode: fly corner to corner, each reached within corner_m. */
void traj_gate_start_waypoints(traj_gate_t *g, int n_points, float corner_m);

/* Follow mode: steady speed with velocity feedforward (see above). */
void traj_gate_start_follow(traj_gate_t *g, int n_points, float sample_s, float corner_m,
                            float accel_mps2);

/* Horizontal position of the path at sample k (may be fractional). */
typedef void (*traj_point_fn)(float k, float *x, float *y, void *ctx);

void traj_gate_start(traj_gate_t *g, int n_points, float sample_s, float lookahead_m,
                     float corner_m);

/* One tick.  elapsed_s: time since the previous tick.  drone_x/y: PX4's position.
 * pos_ok: that position is fresh.  Afterwards g->k is the target (and in follow
 * mode g->speed the feedforward speed along the path at g->proj). */
traj_gate_result_t traj_gate_step(traj_gate_t *g, float elapsed_s, float drone_x, float drone_y,
                                  bool pos_ok, traj_point_fn point, void *ctx);

#ifdef __cplusplus
}
#endif

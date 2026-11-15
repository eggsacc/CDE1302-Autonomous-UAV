/* traj_gate.c — see traj_gate.h.  Pure C99, no ESP-IDF. */
#include "traj_gate.h"

#include <math.h>

#define SEARCH_STEP 0.25f       /* samples between distance checks along the path */

void traj_gate_start(traj_gate_t *g, int n_points, float sample_s, float lookahead_m,
                     float corner_m)
{
    g->k = 0.0f;
    g->last = n_points - 1;
    g->sample_s = sample_s;
    g->lookahead_m = lookahead_m;
    g->corner_m = corner_m;
    g->corner = g->last;
    g->waypoints = false;
}

#define WAYPOINT_MIN_RADIUS_M 0.05f

void traj_gate_start_waypoints(traj_gate_t *g, int n_points, float corner_m)
{
    traj_gate_start(g, n_points, 1.0f, 1.0f, corner_m);
    if (g->corner_m < WAYPOINT_MIN_RADIUS_M) g->corner_m = WAYPOINT_MIN_RADIUS_M;
    g->waypoints = true;
}

static float distance(float k, float drone_x, float drone_y, traj_point_fn point, void *ctx)
{
    float x, y;
    point(k, &x, &y, ctx);
    return sqrtf((x - drone_x) * (x - drone_x) + (y - drone_y) * (y - drone_y));
}

traj_gate_result_t traj_gate_step(traj_gate_t *g, float elapsed_s, float drone_x, float drone_y,
                                  bool pos_ok, traj_point_fn point, void *ctx)
{
    if (elapsed_s > TRAJ_GATE_MAX_STEP_S) elapsed_s = TRAJ_GATE_MAX_STEP_S;

    if (g->lookahead_m <= 0.0f) {                     /* on the clock */
        if (g->k >= (float)g->last) return TRAJ_GATE_DONE;
        g->k += elapsed_s / g->sample_s;
        if (g->k > (float)g->last) g->k = (float)g->last;
        return TRAJ_GATE_MOVE;
    }
    if (!pos_ok) return TRAJ_GATE_WAIT;

    float limit = (float)(g->corner >= 0 ? g->corner : g->last);
    if (g->waypoints) g->k = limit;                   /* straight to the next corner */
    float err = distance(g->k, drone_x, drone_y, point, ctx);

    if (g->k >= limit) {
        float near = g->corner_m > 0.0f ? g->corner_m
                                        : TRAJ_GATE_END_FRACTION * g->lookahead_m;
        if (err > near) return TRAJ_GATE_WAIT;
        if (g->k >= (float)g->last) return TRAJ_GATE_DONE;
        g->corner = -1;                               /* reached: on to the next one */
        return TRAJ_GATE_WAIT;
    }

    /* Slide the target along the path until it is lookahead_m from the drone. */
    float budget = TRAJ_GATE_MAX_RATE * elapsed_s / g->sample_s;
    while (budget > 0.0f && g->k < limit && err < g->lookahead_m) {
        float step = budget < SEARCH_STEP ? budget : SEARCH_STEP;
        g->k += step;
        if (g->k > limit) g->k = limit;
        budget -= step;
        err = distance(g->k, drone_x, drone_y, point, ctx);
    }
    return TRAJ_GATE_MOVE;
}

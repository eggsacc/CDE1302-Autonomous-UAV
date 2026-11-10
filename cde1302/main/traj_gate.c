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
    g->follow = false;
    g->accel = 0.0f;
    g->proj = 0.0f;
    g->speed = 0.0f;
}

void traj_gate_start_follow(traj_gate_t *g, int n_points, float sample_s, float corner_m,
                            float accel_mps2)
{
    traj_gate_start(g, n_points, sample_s, 1.0f, corner_m);
    if (g->corner_m < TRAJ_GATE_FOLLOW_MIN_CORNER_M) g->corner_m = TRAJ_GATE_FOLLOW_MIN_CORNER_M;
    g->accel = accel_mps2 > 0.0f ? accel_mps2 : 0.3f;
    g->follow = true;
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

static float path_gap(float a, float b, traj_point_fn point, void *ctx)
{
    float bx, by;
    point(b, &bx, &by, ctx);
    return distance(a, bx, by, point, ctx);
}

/* Planned speed around sample k (m/s): sample spacing / sample time.  The
 * trajectory may repeat its last point, so take the larger of two segments. */
static float planned_speed(const traj_gate_t *g, float k, traj_point_fn point, void *ctx)
{
    int i = (int)k;
    if (i > g->last - 1) i = g->last - 1;
    if (i < 0) i = 0;
    float d = path_gap((float)i, (float)(i + 1), point, ctx);
    if (i > 0) {
        float d0 = path_gap((float)(i - 1), (float)i, point, ctx);
        if (d0 > d) d = d0;
    }
    return d / g->sample_s;
}

static traj_gate_result_t follow_step(traj_gate_t *g, float elapsed_s, float drone_x,
                                      float drone_y, bool pos_ok, traj_point_fn point, void *ctx)
{
    if (!pos_ok) {                      /* no fresh position: ease off, don't jerk */
        g->speed -= g->accel * elapsed_s;
        if (g->speed < 0.0f) g->speed = 0.0f;
        return TRAJ_GATE_WAIT;
    }
    float limit = (float)(g->corner >= 0 ? g->corner : g->last);

    /* At the corner (or the end)? */
    if (distance(limit, drone_x, drone_y, point, ctx) <= g->corner_m) {
        g->proj = limit;
        g->k = limit;
        if (limit >= (float)g->last) {
            g->speed = 0.0f;
            return TRAJ_GATE_DONE;
        }
        g->corner = -1;                               /* on to the next one */
        return TRAJ_GATE_MOVE;
    }

    /* Progress: walk forward while the path gets closer to the drone. */
    float budget = TRAJ_GATE_FOLLOW_RATE * elapsed_s / g->sample_s;
    float err = distance(g->proj, drone_x, drone_y, point, ctx);
    while (budget > 0.0f && g->proj < limit) {
        float step = budget < SEARCH_STEP ? budget : SEARCH_STEP;
        float next = g->proj + step;
        if (next > limit) next = limit;
        float e = distance(next, drone_x, drone_y, point, ctx);
        if (e > err) break;
        g->proj = next;
        err = e;
        budget -= step;
    }

    /* Target: a little ahead of the progress, never past the corner. */
    float v_plan = planned_speed(g, g->proj, point, ctx);
    float lead = v_plan * TRAJ_GATE_FOLLOW_LEAD_S;
    float k = g->proj;
    while (k < limit) {
        float next = k + SEARCH_STEP;
        if (next > limit) next = limit;
        if (path_gap(g->proj, next, point, ctx) > lead) break;
        k = next;
    }
    g->k = k;

    /* Speed: ramp up at accel, capped by the plan and by the braking curve
     * that slows to TRAJ_GATE_CORNER_SPEED at corner_m before the corner. */
    float brake = path_gap(g->proj, limit, point, ctx) - g->corner_m;
    if (brake < 0.0f) brake = 0.0f;
    float v_stop = sqrtf(TRAJ_GATE_CORNER_SPEED * TRAJ_GATE_CORNER_SPEED
                         + 2.0f * g->accel * brake);
    float v = g->speed + g->accel * elapsed_s;
    if (v > v_plan) v = v_plan;
    if (v > v_stop) v = v_stop;
    g->speed = v > 0.0f ? v : 0.0f;
    return TRAJ_GATE_MOVE;
}

traj_gate_result_t traj_gate_step(traj_gate_t *g, float elapsed_s, float drone_x, float drone_y,
                                  bool pos_ok, traj_point_fn point, void *ctx)
{
    if (elapsed_s > TRAJ_GATE_MAX_STEP_S) elapsed_s = TRAJ_GATE_MAX_STEP_S;
    if (g->follow) return follow_step(g, elapsed_s, drone_x, drone_y, pos_ok, point, ctx);

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

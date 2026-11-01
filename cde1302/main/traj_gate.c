/* traj_gate.c — see traj_gate.h.  Pure C99, no ESP-IDF. */
#include "traj_gate.h"

void traj_gate_start(traj_gate_t *g, int n_points, float sample_s, float leash_m,
                     float corner_m)
{
    g->k = 0.0f;
    g->last = n_points - 1;
    g->sample_s = sample_s;
    g->leash_m = leash_m;
    g->rate = 1.0f;
    g->corner_m = corner_m;
    g->corner = g->last;
}

traj_gate_result_t traj_gate_step(traj_gate_t *g, float elapsed_s, float err_m, bool pos_ok)
{
    bool  gated = g->leash_m > 0.0f;
    float full  = TRAJ_GATE_FULL_SPEED * g->leash_m;     /* full speed up to here */

    g->rate = 1.0f;
    if (gated) {
        if (!pos_ok) {
            g->rate = 0.0f;
        } else if (err_m > full) {
            g->rate = (g->leash_m - err_m) / (g->leash_m - full);
            if (g->rate < 0.0f) g->rate = 0.0f;
        }
    }

    bool  corners = gated && g->corner_m > 0.0f;
    float near    = corners ? g->corner_m : full;

    if (g->k >= (float)g->last) {
        bool arrived = !gated || (pos_ok && err_m <= near);
        g->rate = 0.0f;
        return arrived ? TRAJ_GATE_DONE : TRAJ_GATE_WAIT;
    }
    if (corners && g->corner >= 0 && g->k >= (float)g->corner) {
        g->rate = 0.0f;
        if (!pos_ok || err_m > near) return TRAJ_GATE_WAIT;
        g->corner = -1;                     /* reached: on to the next one */
        return TRAJ_GATE_WAIT;
    }
    if (g->rate <= 0.0f) return TRAJ_GATE_WAIT;

    if (elapsed_s > TRAJ_GATE_MAX_STEP_S) elapsed_s = TRAJ_GATE_MAX_STEP_S;
    g->k += g->rate * elapsed_s / g->sample_s;
    if (g->k > (float)g->last) g->k = (float)g->last;
    if (corners && g->corner >= 0 && g->k > (float)g->corner) g->k = (float)g->corner;
    return TRAJ_GATE_MOVE;
}

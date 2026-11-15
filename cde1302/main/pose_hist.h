/*
 * pose_hist — short history of PX4 positions on the ESP32 clock, so a camera
 * frame can be paired with where the drone was when the frame was captured.
 *
 * Both times come from esp_timer, so Wi-Fi delay never enters the pairing.
 * Pure C99 (no ESP-IDF): tested on the laptop (mission/tests/test_pose_hist_c.py).
 * Not thread-safe: the caller holds a lock.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define POSE_HIST_N 128          /* ~2.5 s of LOCAL_POSITION_NED at 50 Hz */

typedef struct {
    int64_t t_us;                /* esp_timer time the sample arrived */
    float   x, y;                /* PX4 local NED (m) */
    float   heading;             /* rad, 0 = North, clockwise positive (any wrap) */
} pose_sample_t;

typedef struct {
    pose_sample_t s[POSE_HIST_N];
    int head;                    /* next slot to write */
    int count;
} pose_hist_t;

void pose_hist_init(pose_hist_t *h);

/* Add a sample; samples must arrive in time order (older ones are ignored). */
void pose_hist_push(pose_hist_t *h, const pose_sample_t *p);

/* Pose at time t_us, interpolated between the two samples around it (heading
 * the short way round).  A time up to max_gap_us after the newest sample uses
 * the newest.  yaw_rate: rad/s from those two samples.  gap_us: distance from
 * t_us to the nearer sample used.  False if t_us is outside the history. */
bool pose_hist_at(const pose_hist_t *h, int64_t t_us, int64_t max_gap_us,
                  pose_sample_t *out, float *yaw_rate, int64_t *gap_us);

#ifdef __cplusplus
}
#endif

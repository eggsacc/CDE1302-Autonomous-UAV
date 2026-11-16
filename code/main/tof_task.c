#include "tof_task.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_timer.h"

#include <math.h>
#include <string.h>
#include <float.h>

// ---------------------------------------------------------------------------
// Sensor angle table (body frame, degrees, CW from forward)
// Computed in tof_task_init() from TOF_FRONT_SENSOR_IDX.
// ---------------------------------------------------------------------------
static float SENSOR_ANGLES[TOF_SENSOR_COUNT];

// ---------------------------------------------------------------------------
// Shared scan state. The VL53L5CX ring and its driver were removed, so nothing
// writes it: the accessors below return an empty scan (no sensors, no ranges).
// ---------------------------------------------------------------------------
static tof_scan_t    s_scan;
static SemaphoreHandle_t s_scan_mutex;

// ---------------------------------------------------------------------------
// Compute minimum range (metres) from a stored tof_frame_t.
//
// Status 5/9: valid measurement — use reported distance.
// Status 255: out of range (nothing there) — ignored.
// Any other status: unreliable — treat as 0.3 m (conservative).
// Returns INFINITY only if every pixel is status 255 or out-of-range.
// ---------------------------------------------------------------------------
static float frame_min_range_m(const tof_frame_t *f)
{
    float min_m = INFINITY;
    for (int row = 4; row < 8; row++) {
        for (int col = 0; col < 8; col++) {
            int      i      = row * 8 + col;
            uint8_t  status = f->target_status[i];
            uint16_t dist   = f->distance_mm[i];

            float m;
            if (status == 255) {
                continue;
            } else if (status == 5 || status == 9) {
                if (dist < TOF_MIN_VALID_MM || dist > TOF_MAX_VALID_MM) continue;
                m = (float)dist / 1000.0f;
            } else {
                m = 0.30f;
            }
            if (m < min_m) min_m = m;
        }
    }
    return min_m;
}

// ---------------------------------------------------------------------------
// Angle helpers
// ---------------------------------------------------------------------------

// Normalise angle to [0, 360)
static float norm360(float a)
{
    a = fmodf(a, 360.0f);
    if (a < 0.0f) a += 360.0f;
    return a;
}

// Shortest angular distance (unsigned) between two angles, result in [0, 180]
static float angle_dist(float a, float b)
{
    float d = fabsf(norm360(a) - norm360(b));
    if (d > 180.0f) d = 360.0f - d;
    return d;
}

// True if sensor at 'sensor_center_deg' has any coverage overlap with
// the query arc [q_min_deg, q_max_deg].
//
// Logic: the sensor covers ±HALF_WIDTH around its center.
// The query covers from q_min to q_max (may cross 0°).
// Overlap exists if the closest point on the query arc to the sensor center
// is within HALF_WIDTH degrees.
static bool sensor_overlaps_sector(float sensor_center_deg,
                                   float q_min_deg, float q_max_deg)
{
    float sc = norm360(sensor_center_deg);
    float qn = norm360(q_min_deg);
    float qx = norm360(q_max_deg);

    // Check if the sensor center falls inside the query arc
    bool inside;
    if (qn <= qx) {
        inside = (sc >= qn && sc <= qx);
    } else {
        // arc crosses 0°
        inside = (sc >= qn || sc <= qx);
    }
    if (inside) return true;

    // Otherwise, check if either query endpoint is within HALF_WIDTH of sensor center
    return angle_dist(sc, qn) <= TOF_SENSOR_HALF_WIDTH_DEG ||
           angle_dist(sc, qx) <= TOF_SENSOR_HALF_WIDTH_DEG;
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

void tof_task_init(void)
{
    memset(&s_scan, 0, sizeof(s_scan));

    // Compute CCW sensor angles from front sensor index
    for (int i = 0; i < TOF_SENSOR_COUNT; i++) {
        SENSOR_ANGLES[i] = (float)(((TOF_FRONT_SENSOR_IDX - i) * 45 % 360 + 360) % 360);
    }

    s_scan_mutex = xSemaphoreCreateMutex();
    configASSERT(s_scan_mutex != NULL);
}

float tof_get_min_range_in_sector(float angle_min_deg, float angle_max_deg)
{
    xSemaphoreTake(s_scan_mutex, portMAX_DELAY);
    tof_scan_t snap = s_scan;
    xSemaphoreGive(s_scan_mutex);

    float min_m = INFINITY;
    for (int i = 0; i < TOF_SENSOR_COUNT; i++) {
        if (!snap.sensor_ok[i])  continue;
        if (!snap.frame[i].valid) continue;
        if (!sensor_overlaps_sector(SENSOR_ANGLES[i], angle_min_deg, angle_max_deg)) continue;

        float m = frame_min_range_m(&snap.frame[i]);
        if (m < min_m) min_m = m;
    }
    return min_m;
}

tof_scan_t tof_get_scan(void)
{
    xSemaphoreTake(s_scan_mutex, portMAX_DELAY);
    tof_scan_t copy = s_scan;
    xSemaphoreGive(s_scan_mutex);
    return copy;
}

bool tof_is_healthy(void)
{
    xSemaphoreTake(s_scan_mutex, portMAX_DELAY);
    // Find the most recently updated frame
    uint32_t latest_ms = 0;
    for (int i = 0; i < TOF_SENSOR_COUNT; i++) {
        if (s_scan.frame[i].valid && s_scan.frame[i].timestamp_ms > latest_ms) {
            latest_ms = s_scan.frame[i].timestamp_ms;
        }
    }
    xSemaphoreGive(s_scan_mutex);

    if (latest_ms == 0) return false;
    uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
    return (now_ms - latest_ms) < 500;
}

int tof_sensors_ok_count(void)
{
    xSemaphoreTake(s_scan_mutex, portMAX_DELAY);
    int count = 0;
    for (int i = 0; i < TOF_SENSOR_COUNT; i++)
        count += s_scan.sensor_ok[i];
    xSemaphoreGive(s_scan_mutex);
    return count;
}

tof_scan_collapsed_t tof_get_collapsed_scan(void)
{
    tof_scan_t snap = tof_get_scan();  /* thread-safe snapshot */

    tof_scan_collapsed_t out;
    for (int i = 0; i < TOF_SENSOR_COUNT * TOF_SENSOR_RESO; i++)
        out.ranges[i] = INFINITY;

    for (int s = 0; s < TOF_SENSOR_COUNT; s++) {
        if (!snap.sensor_ok[s] || !snap.frame[s].valid) continue;
        const tof_frame_t *f = &snap.frame[s];

        for (int col = 0; col < 8; col++) {
            /* Sensors right-side up: raw col 0 = RIGHT of FoV (from lens).
             * col 0 → +offset (CW), col 7 → −offset (CCW). */
            float angle_deg = SENSOR_ANGLES[s] + (3.5f - (float)col) * (45.0f / 8.0f);

            /* Rows 4-7 = physical upper half (sensors mounted upside-down) */
            for (int row = 4; row < 8; row++) {
                int      px     = row * 8 + col;
                uint8_t  status = f->target_status[px];
                uint16_t dist   = f->distance_mm[px];

                /* Status 255: out of range (nothing there) — leave INFINITY.
                 * Status 5/9: valid measurement — use reported distance.
                 * Anything else: unreliable — assume 0.3 m (conservative). */
                float dist_m;
                if (status == 255) {
                    continue;   /* open space, INFINITY is correct */
                } else if (status == 5 || status == 9) {
                    if (dist < TOF_MIN_VALID_MM || dist > TOF_MAX_VALID_MM) continue;
                    dist_m = (float)dist / 1000.0f;
                } else {
                    dist_m = 0.40f;  /* conservative fallback for error statuses */
                }

                float norm_angle = fmodf(angle_deg, 360.0f);
                if (norm_angle < 0.0f) norm_angle += 360.0f;
                int idx = (int)(norm_angle * (float)(TOF_SENSOR_COUNT * TOF_SENSOR_RESO) / 360.0f);
                if (idx >= TOF_SENSOR_COUNT * TOF_SENSOR_RESO) idx = TOF_SENSOR_COUNT * TOF_SENSOR_RESO - 1;

                if (dist_m < out.ranges[idx])
                    out.ranges[idx] = dist_m;
                // printf("(%d: %zu)", idx, dist);
            }
            // printf("\n");
        }
    }

    return out;
}

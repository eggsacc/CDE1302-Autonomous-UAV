#pragma once

#include <stdbool.h>

#include "esp_camera.h"
#include "sdkconfig.h"

/* On-demand JPEG camera preview for laptop/camera_stream.py, at the lowest
 * priority, so encoding only uses time the other tasks on its core leave
 * idle.  Without the onboard detector, core 1 is nearly idle (nav_task and
 * mission_task mostly sleep), so encoding goes there, away from the WiFi,
 * lwIP and camera driver tasks on core 0.  The stream follows the IP address
 * of the latest keepalive.  The stack is in PSRAM; jpge + send() peak at
 * ~4 KB. */
#if CONFIG_AT_ONBOARD_DETECT
#define CAMERA_STREAM_TASK_CORE      0
#else
#define CAMERA_STREAM_TASK_CORE      1
#endif
#define CAMERA_STREAM_TASK_PRIORITY  1
#define CAMERA_STREAM_TASK_STACK     6144

/* Called once by at_detect_task after the camera is initialised.  A failure
 * only logs a warning and leaves the preview off. */
void camera_stream_start(void);

/* True once the stream task is running. */
bool camera_stream_active(void);

/* esp_camera_fb_get() that never returns a frame left stale in the driver's
 * queue.  Used by both camera consumers; return the frame as usual. */
camera_fb_t *camera_fb_get_fresh(void);

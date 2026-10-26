#include "platform.h"
/* Include the real translation unit so tests can step its private nav_tick.
 * Do not copy, replace or reimplement trajectory playback in this harness. */
#include "nav_task.c"

HOST_EXPORT void host_nav_reset(void) {
    free(s_traj);
    free(s_yaw_upload.points);
    free(s_mutex);
    s_traj_id = 0;
    s_traj_n = s_traj_dt_ms = 0;
    s_traj_has_yaw = false;
    s_traj_t0_us = 0;
    nav_task_init();
}
HOST_EXPORT int host_nav_packet(const uint8_t *p, int n, int flying, uint8_t *reply) {
    return nav_traj_yaw_command(p, (size_t)n, flying != 0, reply);
}
HOST_EXPORT void host_nav_tick(void) {
    vfh_config_t config = VFH_DEFAULT_CONFIG;
    nav_tick(&config);
}
HOST_EXPORT int host_nav_state(void) { return nav_get_status().state; }
HOST_EXPORT void host_nav_cancel(void) { nav_cancel(); }
HOST_EXPORT void host_legacy_put(int id, int total, int offset, const void *xyz, int n) {
    nav_traj_put((uint8_t)id, (uint16_t)total, (uint16_t)offset, xyz, n);
}
HOST_EXPORT void host_legacy_start(int id, int dt_ms) {
    nav_traj_start((uint8_t)id, (uint16_t)dt_ms);
}

/* Unrelated sensors/map/Wi-Fi boundaries. Hardware is never accessed. */
bool wifi_is_connected(void) { return true; }
wifi_peer_list_t wifi_get_peers(void) { return (wifi_peer_list_t){0}; }
bool tof_is_healthy(void) { return false; }
tof_scan_collapsed_t tof_get_collapsed_scan(void) { return (tof_scan_collapsed_t){0}; }
void map_to_odom(float x, float y, float *out_x, float *out_y) {
    *out_x = x; *out_y = y;
}

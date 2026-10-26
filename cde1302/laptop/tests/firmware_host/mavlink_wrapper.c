#include "platform.h"
/* Real setpoint state, watchdog, serialisation and telemetry parser. */
#include "mavlink_task.c"

HOST_EXPORT void host_mav_reset(void) {
    free(s_sp_mutex);
    free(s_state_mutex);
    mavlink_task_init();
    host_uart_tx_size = 0;
}
static void receive_message(mavlink_message_t *message, uint32_t age_ms) {
    uint8_t bytes[MAVLINK_MAX_PACKET_LEN];
    uint16_t length = mavlink_msg_to_send_buffer(bytes, message);
    int64_t now = host_time_us;
    host_time_us -= (int64_t)age_ms * 1000;
    host_uart_receive(bytes, length);
    parse_incoming();
    host_time_us = now;
}
HOST_EXPORT void host_telemetry(float x, float y, float z, float yaw,
                                int armed, int mode, uint32_t pos_age,
                                uint32_t att_age, uint32_t hb_age) {
    mavlink_message_t message;
    mavlink_msg_local_position_ned_pack(PX4_SYSID, PX4_COMPID, &message,
                                       0, x, y, z, 0, 0, 0);
    receive_message(&message, pos_age);
    mavlink_msg_attitude_pack(PX4_SYSID, PX4_COMPID, &message,
                             0, 0, 0, yaw, 0, 0, 0);
    receive_message(&message, att_age);
    mavlink_msg_heartbeat_pack(PX4_SYSID, PX4_COMPID, &message,
                              MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_PX4,
                              armed ? MAV_MODE_FLAG_SAFETY_ARMED : 0,
                              (uint32_t)mode << 16, MAV_STATE_ACTIVE);
    receive_message(&message, hb_age);
}
HOST_EXPORT int host_setpoint(uint8_t *bytes, int capacity) {
    host_uart_tx_size = 0;
    send_setpoint();
    assert(capacity >= (int)host_uart_tx_size);
    memcpy(bytes, host_uart_tx, host_uart_tx_size);
    return (int)host_uart_tx_size;
}

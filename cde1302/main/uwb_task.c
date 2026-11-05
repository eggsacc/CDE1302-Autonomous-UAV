/*
 * uwb_task.c — see uwb_task.h.
 *
 * UART2 (RX only by default) → nlink_parser → uwb_state_t (+ serial log)
 *                                          ├→ debug records → UDP to the laptop
 *                                          └→ aligned fix → mavlink_task → PX4
 */
#include "uwb_task.h"

#include <inttypes.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#include "uwb_px4.h"

static SemaphoreHandle_t s_mutex;
static uwb_state_t       s_state;
static uwb_xform_t       s_xform;     /* guarded by s_mutex */

void uwb_get_state(uwb_state_t *out)
{
    if (s_mutex == NULL) {
        memset(out, 0, sizeof *out);
        return;
    }
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    *out = s_state;
    xSemaphoreGive(s_mutex);
}

void uwb_handle_align_cmd(const uint8_t *buf, int len)
{
#if CONFIG_UWB_SEND_TO_PX4
    uwb_xform_t xf;
    if (s_mutex == NULL || len < 0 || !uwb_align_parse(buf, (size_t)len, &xf)) return;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_xform = xf;
    s_state.aligned = xf.valid;
    s_state.align_received++;
    xSemaphoreGive(s_mutex);
#else
    (void)buf;
    (void)len;
#endif
}

void uwb_clear_alignment(void)
{
    if (s_mutex == NULL) return;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    memset(&s_xform, 0, sizeof s_xform);
    s_state.aligned = false;
    xSemaphoreGive(s_mutex);
}

#if CONFIG_UWB_ENABLED

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <errno.h>
#include "freertos/queue.h"
#include "lwip/inet.h"
#include "lwip/sockets.h"

#include "mavlink_task.h"
#include "tof_task.h"
#include "uwb_debug_pack.h"
#include "wifi_task.h"

static const char *TAG = "uwb";

#define UWB_UART          UART_NUM_2
#define UWB_RX_BUF        4096     /* driver ring buffer (B); ~0.6 s of 50 Hz NODE_FRAME2 */
#define UWB_EVENT_QUEUE   16
#define UWB_READ_CHUNK    256
#define UWB_PENDING_MAX   32       /* debug records buffered between UDP flushes */

static QueueHandle_t  s_uart_queue;
static nlink_parser_t s_parser;

/* ---------------------------------------------------------------------------
 * Pin checks: refuse pins the rest of the drone already uses.
 * ------------------------------------------------------------------------- */

/* XIAO ESP32-S3 Sense camera (main/at_detect.c, CAMERA_MODEL_XIAO_ESP32S3). */
static const int CAMERA_PINS[] = {10, 11, 12, 13, 14, 15, 16, 17, 18, 38, 39, 40, 47, 48};

static bool pin_ok(int pin, const char *name)
{
    if (!GPIO_IS_VALID_GPIO(pin)) {
        ESP_LOGE(TAG, "%s=%d is not a valid GPIO", name, pin);
        return false;
    }
    const char *why = NULL;
    if (pin == CONFIG_PX4_TX_PIN || pin == CONFIG_PX4_RX_PIN) why = "the PX4 UART";
    for (size_t i = 0; !why && i < sizeof CAMERA_PINS / sizeof CAMERA_PINS[0]; i++) {
        if (pin == CAMERA_PINS[i]) why = "the camera";
    }
    if (!why && pin >= 26 && pin <= 37) why = "flash / PSRAM";
    if (!why && (pin == 19 || pin == 20)) why = "USB";
    if (!why && (pin == 43 || pin == 44)) why = "the serial console (UART0)";
    if (!why && (pin == 0 || pin == 45 || pin == 46)) why = "a boot strapping pin";
#if TOF_ENABLED
    if (!why && (pin == CONFIG_TCA_SDA_GPIO_PORT || pin == CONFIG_TCA_SCL_GPIO_PORT)) {
        why = "the ToF I2C bus";
    }
#endif
    if (why) {
        ESP_LOGE(TAG, "%s=GPIO%d is used by %s; pick a free pin in menuconfig", name, pin, why);
        return false;
    }
    if (pin == 3) {
        ESP_LOGW(TAG, "%s=GPIO3 is a strapping pin (JTAG source); usually fine, "
                 "but prefer another free pin if you have one", name);
    }
    return true;
}

bool uwb_task_init(void)
{
    s_mutex = xSemaphoreCreateMutex();
    configASSERT(s_mutex != NULL);

    if (CONFIG_UWB_RX_PIN < 0) {
        ESP_LOGE(TAG, "UWB_ENABLED but UWB_RX_PIN is not set (menuconfig → UWB (LinkTrack)); "
                 "UWB not started");
        return false;
    }
    if (!pin_ok(CONFIG_UWB_RX_PIN, "UWB_RX_PIN")) return false;
    if (CONFIG_UWB_TX_PIN >= 0) {
        if (CONFIG_UWB_TX_PIN == CONFIG_UWB_RX_PIN) {
            ESP_LOGE(TAG, "UWB_TX_PIN and UWB_RX_PIN are both GPIO%d", CONFIG_UWB_RX_PIN);
            return false;
        }
        if (!pin_ok(CONFIG_UWB_TX_PIN, "UWB_TX_PIN")) return false;
    }

    const uart_config_t cfg = {
        .baud_rate  = CONFIG_UWB_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    /* IRAM ISR, like mavlink_task: at 921600 baud the 128-byte FIFO fills in
     * ~1.4 ms, so the interrupt must not wait for flash cache. */
    esp_err_t err = uart_driver_install(UWB_UART, UWB_RX_BUF, 0, UWB_EVENT_QUEUE,
                                        &s_uart_queue, ESP_INTR_FLAG_IRAM);
    if (err == ESP_OK) err = uart_param_config(UWB_UART, &cfg);
    if (err == ESP_OK) {
        err = uart_set_pin(UWB_UART,
                           CONFIG_UWB_TX_PIN >= 0 ? CONFIG_UWB_TX_PIN : UART_PIN_NO_CHANGE,
                           CONFIG_UWB_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "UART2 setup failed: %s; UWB not started", esp_err_to_name(err));
        return false;
    }
    /* Idle-high line: a pull-up keeps a disconnected/unpowered module from
     * looking like random noise (so "no bytes" really means no bytes). */
    gpio_pullup_en(CONFIG_UWB_RX_PIN);

    nlink_parser_init(&s_parser);
    s_state.running = true;
#if CONFIG_UWB_DEBUG_UDP
    ESP_LOGI(TAG, "UART2 RX=GPIO%d (module TX), TX=%s, %d baud; debug UDP -> %s:%d every %d ms",
             CONFIG_UWB_RX_PIN, CONFIG_UWB_TX_PIN >= 0 ? "connected" : "not used",
             CONFIG_UWB_BAUD, CONFIG_HOST_IPV4_ADDR, CONFIG_UWB_DEBUG_PORT,
             CONFIG_UWB_DEBUG_PERIOD_MS);
#else
    ESP_LOGI(TAG, "UART2 RX=GPIO%d (module TX), TX=%s, %d baud; debug UDP off",
             CONFIG_UWB_RX_PIN, CONFIG_UWB_TX_PIN >= 0 ? "connected" : "not used",
             CONFIG_UWB_BAUD);
#endif
    return true;
}

/* ---------------------------------------------------------------------------
 * Debug stream (UDP to the laptop)
 * ------------------------------------------------------------------------- */
#if CONFIG_UWB_DEBUG_UDP

static uwb_dbg_record_t s_pending[UWB_PENDING_MAX];
static int              s_n_pending;
static int              s_sock = -1;
static uint32_t         s_seq;
static uint8_t          s_pkt[UWB_DBG_MAX_PACKET];

static void queue_record(const uint8_t *frame, size_t len, int64_t now_us)
{
    bool drop = !wifi_is_connected() || s_n_pending >= UWB_PENDING_MAX
                || len > UWB_DBG_MAX_FRAME;
    if (drop) {
        xSemaphoreTake(s_mutex, portMAX_DELAY);
        s_state.debug_records_dropped++;
        xSemaphoreGive(s_mutex);
        return;
    }
    drone_state_t px4 = mavlink_get_state();
    uint32_t now_ms = (uint32_t)(now_us / 1000);
    uwb_dbg_record_t *r = &s_pending[s_n_pending++];
    r->esp_time_us = (uint64_t)now_us;
    r->px4_x = px4.x;
    r->px4_y = px4.y;
    r->px4_z = px4.z;
    r->px4_heading = px4.heading;
    if (px4.last_pos_ms == 0) {
        r->px4_age_ms = UWB_DBG_PX4_AGE_NONE;
    } else {
        uint32_t age = now_ms - px4.last_pos_ms;
        r->px4_age_ms = (uint16_t)(age < UWB_DBG_PX4_AGE_NONE ? age : UWB_DBG_PX4_AGE_NONE - 1);
    }
    r->frame_len = (uint16_t)len;
    memcpy(r->frame, frame, len);
}

static bool open_socket(void)
{
    if (s_sock >= 0) return true;
    s_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (s_sock < 0) {
        ESP_LOGW(TAG, "debug UDP socket failed (errno %d)", errno);
        return false;
    }
    return true;
}

static void send_packet(size_t len)
{
    struct sockaddr_in dest = {
        .sin_family = AF_INET,
        .sin_port   = htons(CONFIG_UWB_DEBUG_PORT),
    };
    inet_pton(AF_INET, CONFIG_HOST_IPV4_ADDR, &dest.sin_addr);
    int sent = sendto(s_sock, s_pkt, len, MSG_DONTWAIT,
                      (const struct sockaddr *)&dest, sizeof dest);
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    if (sent == (int)len) {
        s_state.debug_packets_sent++;
    } else {
        s_state.debug_send_errors++;
    }
    xSemaphoreGive(s_mutex);
}

/* Send everything queued since the last flush; a header-only packet if there
 * is nothing, so the laptop always sees the ESP32's counters. */
static void flush_debug(int64_t now_us)
{
    if (!wifi_is_connected() || !open_socket()) {
        s_n_pending = 0;
        return;
    }
    uwb_state_t st;
    uwb_get_state(&st);
    const uwb_dbg_counters_t c = {
        .bytes = st.parser.total_bytes,
        .frames_decoded = st.frames_decoded,
        .frames_other = st.frames_other,
        .bad_checksum = st.parser.bad_checksum,
        .bad_length = st.parser.bad_length,
        .skipped_bytes = st.parser.skipped_bytes,
        .uart_overflows = st.uart_overflows,
        .uart_frame_errors = st.uart_frame_errors,
        .records_dropped = st.debug_records_dropped,
        .px4_fed = st.px4_fed,
        .px4_rejected = st.px4_rejected,
        .align_received = st.align_received,
    };
    int i = 0;
    do {
        size_t pos = uwb_dbg_write_header(s_pkt, (uint8_t)CONFIG_DRONE_ID, 0, s_seq++,
                                          (uint32_t)(now_us / 1000), &c);
        uint16_t n = 0;
        while (i < s_n_pending) {
            size_t end = uwb_dbg_append_record(s_pkt, pos, sizeof s_pkt, &s_pending[i]);
            if (end == 0) break;            /* packet full: send, then continue */
            pos = end;
            n++;
            i++;
        }
        uwb_dbg_set_count(s_pkt, n);
        send_packet(pos);
    } while (i < s_n_pending);
    s_n_pending = 0;
}

#endif /* CONFIG_UWB_DEBUG_UDP */

/* ---------------------------------------------------------------------------
 * PX4
 * ------------------------------------------------------------------------- */

#if CONFIG_UWB_SEND_TO_PX4
static void feed_px4(const nlink_reading_t *r, int64_t t_us)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    uwb_xform_t xf = s_xform;
    xSemaphoreGive(s_mutex);
    if (!xf.valid) return;

    bool use = uwb_reading_usable(r, CONFIG_UWB_PX4_MAX_EOP_CM / 100.0f)
               && mavlink_position_valid();
    if (use) {
        float px, py;
        uwb_xform_apply(&xf, r->pos[0], r->pos[1], &px, &py);
        mavlink_set_vision_position(px, py, uwb_xy_variance(r), (uint64_t)t_us);
    }
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    if (use) {
        s_state.px4_fed++;
    } else {
        s_state.px4_rejected++;
    }
    xSemaphoreGive(s_mutex);
}
#endif /* CONFIG_UWB_SEND_TO_PX4 */

/* ---------------------------------------------------------------------------
 * Frames
 * ------------------------------------------------------------------------- */

static void on_frame(const uint8_t *frame, size_t len, nlink_kind_t kind, void *ctx)
{
    (void)kind;
    (void)ctx;
    int64_t now_us = esp_timer_get_time();
    nlink_reading_t r;
    bool ok = nlink_decode(frame, len, &r);
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    if (ok) {
        s_state.last = r;
        s_state.last_rx_us = now_us;
        s_state.frames_decoded++;
    } else {
        s_state.frames_other++;
    }
    xSemaphoreGive(s_mutex);
#if CONFIG_UWB_SEND_TO_PX4
    if (ok) feed_px4(&r, now_us);
#endif
#if CONFIG_UWB_DEBUG_UDP
    queue_record(frame, len, now_us);
#endif
}

/* ---------------------------------------------------------------------------
 * Serial log
 * ------------------------------------------------------------------------- */

static void log_status(const uwb_state_t *now, const uwb_state_t *prev, float period_s)
{
    uint32_t bytes = now->parser.total_bytes - prev->parser.total_bytes;
    uint32_t frames = now->frames_decoded - prev->frames_decoded;
    ESP_LOGI(TAG, "%" PRIu32 " B, %" PRIu32 " frames (%.1f/s) | checksum err %" PRIu32
             ", length err %" PRIu32 ", skipped %" PRIu32 " B, uart overflow %" PRIu32
             ", uart framing err %" PRIu32 " | debug sent %" PRIu32 ", send err %" PRIu32
             ", dropped %" PRIu32,
             bytes, frames, frames / period_s, now->parser.bad_checksum,
             now->parser.bad_length, now->parser.skipped_bytes, now->uart_overflows,
             now->uart_frame_errors, now->debug_packets_sent, now->debug_send_errors,
             now->debug_records_dropped);
#if CONFIG_UWB_SEND_TO_PX4
    ESP_LOGI(TAG, "PX4: %s, fed %" PRIu32 ", rejected %" PRIu32 ", alignments %" PRIu32,
             now->aligned ? "aligned" : "waiting for alignment", now->px4_fed,
             now->px4_rejected, now->align_received);
#endif
    if (bytes == 0) {
        ESP_LOGW(TAG, "no bytes on GPIO%d: module powered (3.6-5.5 V)? module TX -> "
                 "this pin? common GND?", CONFIG_UWB_RX_PIN);
        return;
    }
    if (frames == 0) {
        ESP_LOGW(TAG, "bytes arrive but no valid frames: baud mismatch (now %d; module "
                 "default 921600) or NMEA output selected?", CONFIG_UWB_BAUD);
        return;
    }
    const nlink_reading_t *r = &now->last;
    if (r->has_pose) {
        ESP_LOGI(TAG, "%s id %u: pos (%.3f, %.3f, %.3f) m %s, eop (%.2f, %.2f, %.2f) m, "
                 "%.3f V, local_time %" PRIu32 ", anchors %u/%u",
                 nlink_kind_name(r->kind), r->id, r->pos[0], r->pos[1], r->pos[2],
                 r->has_fix ? "FIX" : "NO FIX", r->eop[0], r->eop[1], r->eop[2],
                 r->voltage, r->local_time, r->n_anchors, r->n_anchors_total);
    } else {
        ESP_LOGI(TAG, "%s id %u: %.3f V, local_time %" PRIu32 ", anchors %u/%u",
                 nlink_kind_name(r->kind), r->id, r->voltage, r->local_time,
                 r->n_anchors, r->n_anchors_total);
    }
    for (int k = 0; k < r->n_anchors && k < 8; k++) {
        const nlink_anchor_t *a = &r->anchors[k];
        if (r->kind == NLINK_TAG_FRAME0 && a->dis == 0.0f) continue;
        ESP_LOGI(TAG, "  A%u: %.3f m, rx %.1f dB, first path %.1f dB",
                 a->id, a->dis, a->rx_rssi, a->fp_rssi);
    }
}

/* ---------------------------------------------------------------------------
 * Task
 * ------------------------------------------------------------------------- */

void uwb_task(void *arg)
{
    (void)arg;
    uint8_t buf[UWB_READ_CHUNK];
    const int64_t log_period_us = (int64_t)CONFIG_UWB_LOG_PERIOD_MS * 1000;
    int64_t next_log = esp_timer_get_time() + log_period_us;
#if CONFIG_UWB_DEBUG_UDP
    const int64_t send_period_us = (int64_t)CONFIG_UWB_DEBUG_PERIOD_MS * 1000;
    int64_t next_send = esp_timer_get_time() + send_period_us;
#endif
    uwb_state_t prev;
    uwb_get_state(&prev);

    for (;;) {
        uart_event_t ev;
        if (xQueueReceive(s_uart_queue, &ev, pdMS_TO_TICKS(20))) {
            switch (ev.type) {
            case UART_DATA: {
                size_t avail = 0;
                uart_get_buffered_data_len(UWB_UART, &avail);
                while (avail > 0) {
                    int n = uart_read_bytes(UWB_UART, buf,
                                            avail < sizeof buf ? avail : sizeof buf, 0);
                    if (n <= 0) break;
                    nlink_parser_feed(&s_parser, buf, (size_t)n, on_frame, NULL);
                    avail -= (size_t)n;
                }
                break;
            }
            case UART_FIFO_OVF:
            case UART_BUFFER_FULL:
                /* Bytes were lost; the parser resynchronises by itself. */
                uart_flush_input(UWB_UART);
                xQueueReset(s_uart_queue);
                xSemaphoreTake(s_mutex, portMAX_DELAY);
                s_state.uart_overflows++;
                xSemaphoreGive(s_mutex);
                break;
            case UART_FRAME_ERR:
            case UART_PARITY_ERR:
                xSemaphoreTake(s_mutex, portMAX_DELAY);
                s_state.uart_frame_errors++;
                xSemaphoreGive(s_mutex);
                break;
            default:
                break;
            }
        }
        xSemaphoreTake(s_mutex, portMAX_DELAY);
        s_state.parser = s_parser.stats;
        xSemaphoreGive(s_mutex);

        int64_t now = esp_timer_get_time();
#if CONFIG_UWB_DEBUG_UDP
        if (now >= next_send) {
            next_send = now + send_period_us;
            flush_debug(now);
        }
#endif
        if (now >= next_log) {
            uwb_state_t cur;
            uwb_get_state(&cur);
            log_status(&cur, &prev, (float)(now - next_log + log_period_us) / 1e6f);
            prev = cur;
            next_log = now + log_period_us;
        }
    }
}

#else  /* !CONFIG_UWB_ENABLED */

bool uwb_task_init(void)
{
    return false;
}

void uwb_task(void *arg)
{
    (void)arg;
    vTaskDelete(NULL);
}

#endif /* CONFIG_UWB_ENABLED */

#pragma once
/* Portable protocol/validation core: no RTOS, ESP-IDF or MAVLink dependencies.
 * Wire integers and IEEE754 binary32 are little endian. See experiment README.
 * Call only while nav's mutex is held; storage is supplied from PSRAM. */
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#define TY_VERSION 1
#define TY_MAX_POINTS 2400
#define TY_CHUNK_POINTS 60
#define TY_MIN_DT_MS 50
#define TY_MAX_RATE 1.0f       /* conservative engineering ceiling, NOT TA approval */
#define TY_MAX_SPEED 0.30f
#define TY_ALIGNMENT_RAD 0.0872664626f
#define TY_COORD_LIMIT_M 20.0f
#define TY_YAW_LIMIT_RAD 256.0f
#define TY_REPLY_SIZE 28
#define TY_CMD_CAPS 0x0a
#define TY_CMD_BEGIN 0x0b
#define TY_CMD_DATA 0x0c
#define TY_CMD_START 0x0d
#define TY_CMD_STATUS 0x0e

typedef enum {
    TY_CAPABLE=0, TY_UPLOADING=1, TY_READY=2, TY_PLAYING=3, TY_DONE=4,
    TY_BAD_PACKET=10, TY_BAD_VERSION=11, TY_BAD_ID=12, TY_BUSY=13,
    TY_INCOMPLETE=14, TY_CORRUPT=15, TY_LIMIT=16, TY_NOT_FLYING=17,
    TY_MISALIGNED=18, TY_CANCELLED=19
} ty_status_t;

typedef struct { float x, y, z, yaw; } ty_point_t;
typedef struct {
    ty_point_t *points;
    uint8_t received[(TY_MAX_POINTS+7)/8];
    uint32_t id, consumed_id, crc;
    uint16_t total, count, dt_ms;
    float max_rate;
    ty_status_t state;
} ty_upload_t;

void ty_init(ty_upload_t *u, ty_point_t *points);
uint32_t ty_crc32(const uint8_t *data, size_t size);
float ty_wrap(float angle);
/* Returns false for a malformed envelope or wrong target: no reply, no state change.
 * Only a valid START returning TY_PLAYING changes an upload into playback. */
bool ty_handle(ty_upload_t *u, const uint8_t *packet, size_t size,
               uint8_t drone_id, bool busy, bool can_start, float heading,
               uint8_t reply[TY_REPLY_SIZE]);
void ty_cancel(ty_upload_t *u);
/* Unwrapped interpolation; wrap only at the MAVLink boundary. */
float ty_interpolate_yaw(float a, float b, float fraction);

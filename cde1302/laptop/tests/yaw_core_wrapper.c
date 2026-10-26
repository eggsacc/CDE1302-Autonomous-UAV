#include "traj_yaw.h"
#ifdef _WIN32
#define EXPORT __declspec(dllexport)
#else
#define EXPORT __attribute__((visibility("default")))
#endif
static ty_point_t points[TY_MAX_POINTS];
static ty_upload_t upload;
EXPORT void yaw_reset(void) { ty_init(&upload,points); }
EXPORT int yaw_packet(const uint8_t *p,int n,int busy,int flying,float heading,uint8_t *reply) {
    return ty_handle(&upload,p,(size_t)n,2,busy!=0,flying!=0,heading,reply);
}
EXPORT float yaw_point(int i,int c) {
    const ty_point_t *p=&points[i];
    const float v[4]={p->x,p->y,p->z,p->yaw};
    return v[c];
}
EXPORT float yaw_interpolate(float a,float b,float f) { return ty_interpolate_yaw(a,b,f); }
EXPORT void yaw_cancel(void) { ty_cancel(&upload); }
EXPORT void yaw_done(void) { upload.state=TY_DONE; }

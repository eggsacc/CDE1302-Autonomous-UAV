#include "traj_yaw.h"
#include <float.h>
#include <math.h>
#include <string.h>

_Static_assert(sizeof(float)==4 && FLT_MANT_DIG==24, "IEEE754 float32 required");
_Static_assert(sizeof(ty_point_t)==16, "yaw point layout changed");
_Static_assert(11+TY_CHUNK_POINTS*16<=1024, "yaw packet exceeds UDP receive buffer");

static uint16_t rd16(const uint8_t *p) { return p[0] | (uint16_t)p[1]<<8; }
static uint32_t rd32(const uint8_t *p) {
    return p[0] | (uint32_t)p[1]<<8 | (uint32_t)p[2]<<16 | (uint32_t)p[3]<<24;
}
static float rdf(const uint8_t *p) {
    uint32_t bits=rd32(p); float f; memcpy(&f,&bits,4); return f;
}
static void wr16(uint8_t *p, uint16_t v) { p[0]=(uint8_t)v; p[1]=(uint8_t)(v>>8); }
static void wr32(uint8_t *p, uint32_t v) {
    for(int i=0;i<4;i++) p[i]=(uint8_t)(v>>(8*i));
}
static void wrf(uint8_t *p, float f) { uint32_t v; memcpy(&v,&f,4); wr32(p,v); }

uint32_t ty_crc32(const uint8_t *data, size_t size) {
    uint32_t c=0xffffffffu;
    for(size_t i=0;i<size;i++) {
        c^=data[i];
        for(int j=0;j<8;j++) c=(c>>1)^((0u-(c&1u))&0xedb88320u);
    }
    return ~c;
}
float ty_wrap(float a) {
    a=fmodf(a+3.14159265358979323846f,6.2831853071795864769f);
    if(a<0) a+=6.2831853071795864769f;
    return a-3.14159265358979323846f;
}
float ty_interpolate_yaw(float a,float b,float fraction) {
    return ty_wrap(a+fraction*(b-a));
}
void ty_init(ty_upload_t *u,ty_point_t *points) {
    memset(u,0,sizeof(*u)); u->points=points; u->state=TY_CAPABLE;
}
void ty_cancel(ty_upload_t *u) {
    if(u->id) u->consumed_id=u->id;
    u->state=TY_CANCELLED;
}

static ty_status_t verify(ty_upload_t *u) {
    if(u->state==TY_CORRUPT || u->state==TY_CANCELLED) return u->state;
    if(u->count!=u->total) return TY_INCOMPLETE;
    /* Re-encode explicitly, so the CRC is independent of host byte order. */
    uint32_t c=0xffffffffu;
    for(int i=0;i<u->total;i++) {
        const ty_point_t *p=&u->points[i];
        const float values[4]={p->x,p->y,p->z,p->yaw};
        for(int k=0;k<4;k++) {
            uint8_t bytes[4]; wrf(bytes,values[k]);
            for(int n=0;n<4;n++) {
                c^=bytes[n];
                for(int j=0;j<8;j++) c=(c>>1)^((0u-(c&1u))&0xedb88320u);
            }
        }
        if(i==0) {
            if(fabsf(p->x)>1e-5f || fabsf(p->y)>1e-5f || fabsf(p->z)>1e-5f)
                return TY_LIMIT;   /* no initial position jump */
        } else {
            const ty_point_t *a=&u->points[i-1];
            float hz=1000.0f/u->dt_ms;
            float dx=p->x-a->x,dy=p->y-a->y,dz=p->z-a->z;
            if(sqrtf(dx*dx+dy*dy+dz*dz)*hz>TY_MAX_SPEED+1e-5f ||
               fabsf(p->yaw-a->yaw)*hz>u->max_rate+1e-4f) return TY_LIMIT;
        }
    }
    if((~c)!=u->crc) { u->state=TY_CORRUPT; return TY_CORRUPT; }
    return TY_READY;
}

bool ty_handle(ty_upload_t *u,const uint8_t *p,size_t size,
               uint8_t drone_id,bool busy,bool can_start,float heading,uint8_t reply[TY_REPLY_SIZE]) {
    if(size<8 || p[0]!=2 || p[1]<TY_CMD_CAPS || p[1]>TY_CMD_STATUS ||
       p[3]!=drone_id || rd32(p+4)==0) return false;
    uint8_t cmd=p[1]; uint32_t id=rd32(p+4);
    ty_status_t result=TY_BAD_PACKET;
    if(p[2]!=TY_VERSION) result=TY_BAD_VERSION;
    else if(busy && cmd!=TY_CMD_CAPS) result=TY_BUSY;
    else if(cmd==TY_CMD_CAPS) {
        if(size==8) result=TY_CAPABLE;
    } else if(cmd==TY_CMD_BEGIN) {
        if(size==20) {
            uint16_t total=rd16(p+8),dt=rd16(p+10);
            float rate=rdf(p+12); uint32_t crc=rd32(p+16);
            if(u->state==TY_PLAYING) result=TY_BUSY;
            else if(id==u->consumed_id) result=TY_BAD_ID;
            else if(total<2 || total>TY_MAX_POINTS || dt<TY_MIN_DT_MS || dt>1000 ||
                    !isfinite(rate) || rate<=0 || rate>TY_MAX_RATE) result=TY_LIMIT;
            else if(id==u->id) {
                result=(total==u->total && dt==u->dt_ms && rate==u->max_rate && crc==u->crc)
                       ? u->state : TY_BAD_ID;
            } else {
                u->id=id; u->total=total; u->dt_ms=dt; u->max_rate=rate; u->crc=crc;
                u->count=0; memset(u->received,0,sizeof(u->received));
                result=u->state=TY_UPLOADING;
            }
        }
    } else if(id!=u->id) result=TY_BAD_ID;
    else if(cmd==TY_CMD_DATA) {
        if(u->state==TY_PLAYING) result=TY_BUSY;
        else if(u->state==TY_CORRUPT || u->state==TY_CANCELLED || u->state==TY_DONE)
            result=u->state;
        else if(size>=11) {
            uint16_t off=rd16(p+8); int n=p[10];
            if(n>0 && n<=TY_CHUNK_POINTS && (uint32_t)off+(unsigned)n<=u->total &&
               size==11+(size_t)n*16) {
                result=TY_UPLOADING;
                /* Validate the WHOLE chunk before writing anything. */
                for(int i=0;i<n;i++) {
                    ty_point_t point={rdf(p+11+16*i),rdf(p+15+16*i),
                                      rdf(p+19+16*i),rdf(p+23+16*i)};
                    bool seen=(u->received[(off+i)/8]>>( (off+i)%8))&1u;
                    if(!isfinite(point.x)||!isfinite(point.y)||!isfinite(point.z)||!isfinite(point.yaw)||
                       fabsf(point.x)>TY_COORD_LIMIT_M||fabsf(point.y)>TY_COORD_LIMIT_M||
                       fabsf(point.z)>TY_COORD_LIMIT_M||fabsf(point.yaw)>TY_YAW_LIMIT_RAD||
                       (seen && memcmp(&point,&u->points[off+i],sizeof(point)))) {
                        result=u->state=TY_CORRUPT; break;
                    }
                }
                if(result!=TY_CORRUPT) {
                    for(int i=0;i<n;i++) {
                        int k=off+i;
                        if(!(u->received[k/8]&(1u<<(k%8)))) {
                            u->points[k]=(ty_point_t){rdf(p+11+16*i),rdf(p+15+16*i),
                                                      rdf(p+19+16*i),rdf(p+23+16*i)};
                            u->received[k/8]|=(uint8_t)(1u<<(k%8)); u->count++;
                        }
                    }
                    if(u->count==u->total) result=u->state=verify(u);
                }
            }
        }
    } else if(cmd==TY_CMD_STATUS || cmd==TY_CMD_START) {
        if(size==8) {
            if(u->state==TY_PLAYING || u->state==TY_DONE || u->state==TY_CANCELLED)
                result=u->state; /* idempotent START; NEVER resets playback clock */
            else {
                result=verify(u);
                if(result==TY_READY && cmd==TY_CMD_START) {
                    if(!can_start) result=TY_NOT_FLYING;
                    else if(!isfinite(heading) || fabsf(ty_wrap(u->points[0].yaw-heading))>TY_ALIGNMENT_RAD)
                        result=TY_MISALIGNED;
                    else { result=u->state=TY_PLAYING; u->consumed_id=id; }
                }
            }
        }
    }
    reply[0]=5; reply[1]=TY_VERSION; reply[2]=drone_id; reply[3]=cmd;
    wr32(reply+4,id); wr16(reply+8,(uint16_t)result); wr16(reply+10,u->count);
    wr16(reply+12,TY_MAX_POINTS); wr16(reply+14,TY_MIN_DT_MS);
    wrf(reply+16,TY_MAX_RATE); wrf(reply+20,TY_ALIGNMENT_RAD); wrf(reply+24,heading);
    return true;
}

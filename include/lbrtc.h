#ifndef LBRTC_H
#define LBRTC_H

#include "ultra64.h"
#include "lb_rtc.h"

typedef struct
{
    /* 0x0 */ u8 dummy;
    /* 0x2 */ u8 rxsize;
    /* 0x1 */ u8 txsize;
    /* 0x3 */ u8 cmd;
    /* 0x4 */ u8 typeh;
    /* 0x5 */ u8 typel;
    /* 0x6 */ u8 status;
    /* 0x7 */ u8 unk7;
    /* 0x8 */ u8 unk8;
    /* 0x9 */ u8 unk9;
    /* 0xA */ u8 unkA;
    /* 0xB */ u8 unkB;
    /* 0xC */ u8 unkC;
}__OSContRTCFormat;

typedef struct
{
    /* 0x0 */ u8 dummy;
    /* 0x2 */ u8 rxsize;
    /* 0x1 */ u8 txsize;
    /* 0x3 */ u8 typeh;
    /* 0x4 */ u8 typel;
    /* 0x5 */ u8 data;
}__OSContRTC2Format;


typedef struct
{
    u8 unk0;
    u8 unk1;
}unkRTC;

typedef struct {
  u8 sec;
  u8 min;
  u8 hour;
  u8 day;
  u8 weekday;
  u8 month;
  u8 year;
  u8 enabled;
}lbRTC;

s32 func_800FE220_jp(OSMesgQueue* mq, lbRTC_time_c* time);
s32 func_800FE420_jp(OSMesgQueue* mq, lbRTC_time_c* time);
s32 func_800FE480_jp(OSMesgQueue* mq, u8* buf);
s32 func_800FE670_jp(OSMesgQueue* mq, lbRTC_time_c* time);
s32 func_800FE8D0_jp(OSMesgQueue* mq, u8 v, u8* buf);
s32 func_800FEABC_jp(u8 v, u8* buf);
s32 func_800FEBE0_jp(OSMesgQueue* mq, lbRTC_time_c* time );
s32 func_800FECD0_jp(OSMesgQueue* mq);
s32 func_800FEE10_jp(OSMesgQueue* mq, u8 v, u8* buf);
void func_800FEF9C_jp(u8 v);
s32 func_800FF060_jp(lbRTC_time_c* t0, lbRTC_time_c* t1);

extern OSTimer B_80152480_jp;
extern OSMesgQueue B_801524A0_jp;
extern OSMesg B_801524B8_jp;


#endif

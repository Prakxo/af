#include "lbrtc.h"
#include "lib/ultralib/src/io/controller.h"
#include "lib/ultralib/src/io/siint.h"

extern u16 total_days[2][12+1];

s32 func_800FEE10_jp(OSMesgQueue* mq, u8 v, u8* buf){
    s32 ret;
    u8* ptr;
    __OSContRTCFormat format;

    __osSiGetAccess();
    func_800FEF9C_jp(v);
    __osContLastCmd = 7;

    ret = __osSiRawStartDma(1, &__osEepPifRam);
    osRecvMesg(mq, NULL, OS_MESG_BLOCK);
    ret = __osSiRawStartDma(0, &__osEepPifRam);
    osRecvMesg(mq, NULL, OS_MESG_BLOCK);

    ptr = (u8*)&__osEepPifRam.ramarray[1];

    format = *(__OSContRTCFormat*)ptr;
    ret = format.rxsize & 0xc0;
    

    if(ret == 0){
        bcopy(&format, buf, 8);
        if (format.unkC & 1) {
            ret = 0x11;
        } else if (format.unkC & 1) {
            ret = 0x12;
        } else if (format.unkC & 0x80) {
            ret = 0x10;
        }
    }
    else if(ret & 0x80){
        ret = 1;
    }
    else{
        ret = 4;
    }

    __osSiRelAccess();
    return ret;
}

void func_800FEF9C_jp(u8 v){
    u8*  ptr = (u8*)&__osEepPifRam.ramarray;
    s32 i;
    __OSContRTCFormat format;

    __osEepPifRam.pifstatus = 1;

    format.dummy = 2;
    format.rxsize = 9;
    format.txsize = 7;
    format.cmd = v;

    for (i = 0; i < 4; i++) {
        *ptr++ = 0; 
    }

    *(__OSContRTCFormat*)ptr = format;

    ptr+= 0xD;
    ptr[0] = 0xFE;
}

//same as intervaldays but giving back minutes instead of days
s32 func_800FF060_jp(lbRTC_time_c* t0, lbRTC_time_c* t1){

    u8 year_leap_period = (t1->year - t0->year) / 4; /* Total 'leap years' (missing extra not divisible by 100, except when divisible by 400 rule) */
  u8 extra_years = (t1->year - t0->year) % 4; /* Non-leap year remainder */
  u8 less_leap = (t0->year % 4) == 0; /* Is the lesser year a leap year? */
  u8 over_leap = (t1->year % 4) == 0; /* Is the greater year a leap year? */
  u8 leap_add = ((4 - (t0->year % 4)) % 4) < extra_years; /* Add leap day when leap day occurs during 'extra years' */

    s32 days;
    s32 hour;
    s32 min;
  if (t0->year > t1->year) {
    return 0;
  }
  else {
    if (t0->year == t1->year) {
      if (t0->month > t1->month) {
        return 0;
      }
      else {
        if (t0->month == t1->month) {
          if (t0->day > t1->day) {
            return 0;
          }
          else {
            if (t0->day == t1->day) {
              if (t0->hour > t1->hour) {
                return 0;
              }
              else {
                if (t0->hour == t1->hour) {
                  if (t0->min > t1->min) {
                    return 0;
                  }
                }
              }
            }
          }
        }
      }
    }
  }

    days = year_leap_period * 1461 + extra_years * 365 + leap_add;
    days += t1->day - 1;
    days += total_days[over_leap][t1->month - 1];
    days -= t0->day - 1;
    days -= total_days[less_leap][t0->month - 1];

    hour = days * 24;
    hour = (t1->hour + hour) - t0->hour;

    min = hour * 60;
    min = (t1->min + min) - t0->min;
    
    return min;
}
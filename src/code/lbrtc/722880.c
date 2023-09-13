#include "lbrtc.h"

s32 func_800FEBE0_jp(OSMesgQueue* mq, lbRTC_time_c* time ){
    u32 ret;
    u8 rtc[1];
    u8 t; 
    u8 _480t;

    ret = func_800FE480_jp(mq, &_480t);
    if(ret == 0){
        ret = func_800FEE10_jp(mq, 0, &rtc[0]);
        if(ret == 0){
            rtc[0] &= ~1;  
            ret = func_800FE8D0_jp(mq, 0, &rtc[0]);
            if(ret == 0){
                ret=func_800FE8D0_jp(mq, 1, time); 
                if (ret == 0){
                    rtc[0] |= 1;  
                    ret = func_800FE8D0_jp(mq, 0, rtc);
                }
            }
        }
    }
    return ret;
}
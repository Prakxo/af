#include "lbrtc.h"

s32 func_800FE670_jp(OSMesgQueue* mq, lbRTC_time_c* time){
    s32 ret;
    u8 _E10t;
    u8 _480t;
    lbRTC rtc[1];
    unkRTC unk[1];
    
    
    
    ret = func_800FE480_jp(mq, &_480t);

    if(ret == 0){
        ret = func_800FEE10_jp(mq, 0, &_E10t);
        if(ret == 0){
            unk[0].unk0 &= ~2;
            unk[0].unk1 |= 4;
            ret = func_800FE8D0_jp(mq, 0 , &unk); 
            if(ret == 0x10){
                rtc[0].sec = (time->sec % 10) + (time->sec / 10) * 0x10;
                rtc[0].min = (time->min % 10) + (time->min / 10) * 0x10;
                rtc[0].hour = (time->hour % 10) + (time->hour / 10) * 0x10;
                rtc[0].day = (time->day % 10) + (time->day / 10) * 0x10;
                rtc[0].weekday = (time->weekday);
                rtc[0].month = (time->day % 10) + (time->day / 10) * 0x10;
                rtc[0].year = ((time->year % 10) + (((time->year % 100) / 10) * 0x10));
                
                if(time->year >= 2000){
                    unk[0].unk1 = 1;
                }
                else{
                    unk[0].unk1 = 0;
                }
                ret = func_800FE8D0_jp(mq, 2, &rtc);
                if(ret == 0x10){ 
                    unk[0].unk0 |= 2;
                    unk[0].unk1 &= ~4;
                    ret = func_800FE8D0_jp(mq, 0 , &unk); 
                }
            }
        } 
    }
    return ret;
}

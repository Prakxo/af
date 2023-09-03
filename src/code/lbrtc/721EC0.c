#include "lbrtc.h"
#include "attributes.h"

s32 func_800FE220_jp(OSMesgQueue* mq, lbRTC_time_c* time){
    lbRTC rtc;
    u8 _480t;
    UNUSED u8 t;
    s32 ret;
    u8 _E10t; 
    
    ret = func_800FE480_jp(mq, &_480t);

    if(ret == 0){
        ret = func_800FEE10_jp(mq, 2, &_E10t);
        if(ret == 0){
            ret = func_800FEE10_jp(mq, 2, &rtc);
            if(bcmp(&_E10t, &rtc, 8) != 0){
                ret = func_800FEE10_jp(mq, 2, &rtc);
            }
            time->sec = (rtc.sec >> 4) * 0xA + (rtc.sec & 0xF); 
            time->min = (rtc.min >> 4) * 0xA + (rtc.min & 0xF); 
            time->hour = (rtc.hour >> 4) * 0xA + (rtc.hour & 0xF); 
            time->day = (rtc.day >> 4) * 0xA + (rtc.day & 0xF); 
            time->weekday = (rtc.weekday); 
            time->month = (rtc.month >> 4) * 0xA + (rtc.month & 0xF); 
            
            if(rtc.enabled != 0){
                time->year = (rtc.year >> 4) * 0xA + (rtc.year & 0xF) + 2000;
            }
            else{
                time->year = (rtc.year >> 4) * 0xA + (rtc.year & 0xF) + 1901;
                
            }
        }
    }
    return ret;
}
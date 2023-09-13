#include "lbrtc.h"
#include "attributes.h"

s32 func_800FE220_jp(OSMesgQueue* mq, lbRTC_time_c* time){
    lbRTC rtc[1];
    u8 _480t;
    UNUSED u8 t[1];
    u8 _E10t;  
    s32 ret;
    
    
    ret = func_800FE480_jp(mq, &_480t);

    if(ret == 0){
        ret = func_800FEE10_jp(mq, 2, &_E10t);
        if(ret == 0){
            ret = func_800FEE10_jp(mq, 2, &rtc);
            if(bcmp(&_E10t, &rtc, 8) != 0){
                ret = func_800FEE10_jp(mq, 2, &rtc);
            }
            time->sec = (rtc[0].sec >> 4) * 0xA + (rtc[0].sec & 0xF); 
            time->min = (rtc[0].min >> 4) * 0xA + (rtc[0].min & 0xF); 
            time->hour = ((rtc[0].hour -0x80 )>> 4) * 0xA + ((rtc[0].hour - 0x80) & 0xF); 
            time->day = (rtc[0].day >> 4) * 0xA + (rtc[0].day & 0xF); 
            time->weekday = (rtc[0].weekday); 
            time->month = (rtc[0].month >> 4) * 0xA + (rtc[0].month & 0xF); 
            
            if(rtc[0].enabled != 0){
                time->year = (rtc[0].year >> 4) * 0xA + (rtc[0].year & 0xF) + 2000;
            }
            else{
                time->year = (rtc[0].year >> 4) * 0xA + (rtc[0].year & 0xF) + 1900;
                
            }
        }
    }
    return ret;
}
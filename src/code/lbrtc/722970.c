#include "lbrtc.h"

extern s32 D_801160C0_jp;


s32 func_800FECD0_jp(OSMesgQueue* mq){
    unkRTC rtc;
    s32 ret;
    u8 _480t;


    ret = func_800FE480_jp(mq, &_480t);
    if(ret == 0){
        if(D_801160C0_jp == 0){
            osCreateMesgQueue(&B_801524A0_jp, &B_801524B8_jp, 1);
             D_801160C0_jp = 1;
        }
        if(_480t & 0x80){
            ret = func_800FEE10_jp(mq, 0, &rtc);
            if((ret == 0) || (ret == 0x10)){ 
                rtc.unk1 &= ~6;
                ret = func_800FE8D0_jp(mq, 0 , &rtc);
            }
        }
        if(_480t & 1){
            ret = (_480t & 2) ? 0x13 : 0x11;
            
        }
        else if(_480t & 2){
            ret = 0x12;
        }
    }
    return ret;
}
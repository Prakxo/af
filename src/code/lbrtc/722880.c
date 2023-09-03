#include "lbrtc.h"

s32 func_800FEBE0_jp(OSMesgQueue* mq, lbRTC_time_c* time ){
    u32 ret;
    u8 _E10t;
    u8 _480t;
    

    ret = func_800FE480_jp(mq, &_480t);
    if(ret == 0){
        ret = func_800FEE10_jp(mq, 0, &_E10t);
        if(ret == 0){
            ((unkRTC*)_E10t)->unk0 &= ~1;  
            ret = func_800FE8D0_jp(mq, 0, &_E10t);
            if(ret == 0){
                func_800FE8D0_jp(mq, 1, (u8*)time);
                if (ret == 0){
                    ((unkRTC*)_E10t)->unk0 |= 1;  
                    ret = func_800FE8D0_jp(mq, 0, &_E10t);
                }
            }
        }
    }
    return ret;
}
#include "lbrtc.h"
#include "lib/ultralib/src/io/controller.h"
#include "lib/ultralib/src/io/siint.h"

s32 func_800FE420_jp(OSMesgQueue* mq, lbRTC_time_c* time){
    u8 buf;
    s32 ret;
     
    ret = func_800FE480_jp(mq, &buf);

    if(ret == 0){
        ret = func_800FEE10_jp(mq, 1, (u8*)time);
    }

     return ret;
     
}

s32 func_800FE480_jp(OSMesgQueue* mq, u8* buf){
    s32 ret = 0;
    s32 i;
    u8* ptr;
    __OSContRTC2Format requestformat;
    
    __osSiGetAccess();

    __osEepPifRam.pifstatus = 1;

    ptr = (u8*)&__osEepPifRam.ramarray;
    
    for (i = 0; i < 4; i++) {
        *ptr++ = 0;
    }

    requestformat.dummy = CONT_CMD_REQUEST_STATUS_TX;
    requestformat.txsize = CONT_CMD_REQUEST_STATUS_RX;
    requestformat.rxsize = 6;

    *(__OSContRTC2Format*)ptr = requestformat; 

    ptr += 6;

    ptr[0] = 0xFE;
    
    ret = __osSiRawStartDma(1, &__osEepPifRam);
    osRecvMesg(mq, NULL, OS_MESG_BLOCK);
    __osContLastCmd = 6;
    ret = __osSiRawStartDma(0, &__osEepPifRam);
    osRecvMesg(mq, NULL, OS_MESG_BLOCK);

    ptr = (u8*)&__osEepPifRam.ramarray[1];


    requestformat = *(__OSContRTC2Format*)ptr; 
    
    ret = requestformat.rxsize & 0xC0 & 0xFF;

    if(ret == 0){
        if(!(((requestformat.typel << 8) | requestformat.typeh) & 0xFFFF & 0x1000)){
            ret = 0xB;
        } 
        else{
            *buf = requestformat.data;
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
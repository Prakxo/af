#include "lbrtc.h"
#include "lib/ultralib/src/io/controller.h"
#include "lib/ultralib/src/io/siint.h"



s32 func_800FE8D0_jp(OSMesgQueue* mq, u8 v, u8* buf){
    s32 ret;
    u8* ptr;
    __OSContRTCFormat format;
    
    __osSiGetAccess();
    
    func_800FEABC_jp(v, buf);

    ret = __osSiRawStartDma(1, &__osEepPifRam);
    __osContLastCmd = 8;
    
    osRecvMesg(mq, NULL, OS_MESG_BLOCK);
    ret = __osSiRawStartDma(0, &__osEepPifRam);
    ptr = (u8*)&__osEepPifRam.ramarray[1];
    osRecvMesg(mq, NULL, OS_MESG_BLOCK);

    format = *(__OSContRTCFormat*)ptr;

    ret = CHNL_ERR(format);

    __osSiRelAccess();

    osSetTimer(&B_80152480_jp, 0, 0, &B_801524A0_jp, &B_801524B8_jp);
    osRecvMesg(&B_801524A0_jp, NULL, OS_MESG_BLOCK);

   if(ret == 0){
        bcopy(&format, buf, 8);
        if (format.unkC & 1) {
            ret = 0x11;
        } else if (format.unkC & 1) {
            ret = 0x12;
        } else if (format.unkC & 0x80) {
            ret = 0x10;
        }
    }else if (ret & 0x80) {
        ret = 1;
    } else {
        ret = 4;
    }
    
    __osSiRelAccess();
    return ret;
}


s32 func_800FEABC_jp(u8 v, u8* buf){
    s32 i;
    u8* ptr = (u8*)&__osEepPifRam;
    __OSContRTCFormat requestformat;

    for (i = 0; i < ARRLEN(__osEepPifRam.ramarray); i++) {
        __osEepPifRam.ramarray[i] = 0xFF;
    }

    __osEepPifRam.pifstatus = 1;

    requestformat.dummy = 0xA;
    requestformat.txsize = 1;
    requestformat.rxsize = 8;
    requestformat.cmd = v;

    for (i = 0; i < 8; i++){
        *buf++ = *ptr;
    }

    for (i = 0; i < 4; i++) {
        *ptr++ = 0; 
    }
    *(__OSContRTCFormat*)ptr = requestformat;

    ptr += 0xD;
    ptr[0] = 0xFE;
}
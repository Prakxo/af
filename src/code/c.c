#define BINANG_TO_DEG(binang) ((f32)(binang) * (180.0f / 0x8000))
#define SR_CE 0x00020000
#define SR_CH 0x00040000
#define gDPSetTextureConvert(pkt,type) gSPSetOtherMode(pkt, G_SETOTHERMODE_H, G_MDSFT_TEXTCONV, 3, type)
#define alSeqpGetChannelPriority alSeqpGetChlPriority
#define PI_DMA_BUFFER_SIZE 128
#define MI_INTR_MASK_CLR_SI (1 << 2)
#define gDPLoadMultiBlock_4bS(pkt,timg,tmem,rtile,fmt,width,height,pal,cms,cmt,masks,maskt,shifts,shiftt) { gDPSetTextureImage(pkt, fmt, G_IM_SIZ_16b, 1, timg); gDPSetTile(pkt, fmt, G_IM_SIZ_16b, 0, tmem, G_TX_LOADTILE, 0, cmt, maskt, shiftt, cms, masks, shifts); gDPLoadSync(pkt); gDPLoadBlock(pkt, G_TX_LOADTILE, 0, 0, (((width)*(height)+3)>>2)-1, 0 ); gDPPipeSync(pkt); gDPSetTile(pkt, fmt, G_IM_SIZ_4b, ((((width)>>1)+7)>>3), tmem, rtile, pal, cmt, maskt, shiftt, cms, masks, shifts); gDPSetTileSize(pkt, rtile, 0, 0, ((width)-1) << G_TEXTURE_IMAGE_FRAC, ((height)-1) << G_TEXTURE_IMAGE_FRAC); }
#define PFS_ERR_INCONSISTENT 3
#define gsSPSetGeometryMode(word) gsSPGeometryMode(0,(word))
#define SR_DE 0x00010000
#define G_RM_AA_ZB_OPA_DECAL RM_AA_ZB_OPA_DECAL(1)
#define gsSPTextureRectangle(xl,yl,xh,yh,tile,s,t,dsdx,dtdy) {{(_SHIFTL(G_TEXRECT, 24, 8) | _SHIFTL(xh, 12, 12) | _SHIFTL(yh, 0, 12)), (_SHIFTL(tile, 24, 3) | _SHIFTL(xl, 12, 12) | _SHIFTL(yl, 0, 12))}}, gsImmp1(G_RDPHALF_1, (_SHIFTL(s, 16, 16) | _SHIFTL(t, 0, 16))), gsImmp1(G_RDPHALF_2, (_SHIFTL(dsdx, 16, 16) | _SHIFTL(dtdy, 0, 16)))
#define AL_PHASE_DECAY 1
#define G_FOG 0x00010000
#define UT_BASE_NUM 16
#define gsSPBranchLessZ(dl,vtx,zval,near,far,flag) gsSPBranchLessZrg(dl, vtx, zval, near, far, flag, 0, G_MAXZ)
#define G_BRANCH_Z 0x04
#define ERR_OSPISTARTDMA_PIMGR 28
#define IS_KSEGDM(x) ((u32)(x) >= K0BASE && (u32)(x) < K2BASE)
#define SP_SET_YIELD SP_SET_SIG0
#define gsDPSetScissor(mode,ulx,uly,lrx,lry) {{ _SHIFTL(G_SETSCISSOR, 24, 8) | _SHIFTL((int)((float)(ulx)*4.0F), 12, 12) | _SHIFTL((int)((float)(uly)*4.0F), 0, 12), _SHIFTL(mode, 24, 2) | _SHIFTL((int)((float)(lrx)*4.0F), 12, 12) | _SHIFTL((int)((float)(lry)*4.0F), 0, 12) }}
#define NPCW_GET_WALK_NUM(x) (int)((x) / 3)
#define M_LAND_H_H 
#define MOTOR_START 1
#define OS_VI_GAMMA_DITHER_ON 0x0004
#define G_RM_AA_ZB_OPA_TERR2 RM_AA_ZB_OPA_TERR(2)
#define RM_AA_ZB_TEX_EDGE(clk) AA_EN | Z_CMP | Z_UPD | IM_RD | CVG_DST_CLAMP | CVG_X_ALPHA | ALPHA_CVG_SEL | ZMODE_OPA | TEX_EDGE | GBL_c ##clk(G_BL_CLR_IN, G_BL_A_IN, G_BL_CLR_MEM, G_BL_A_MEM)
#define K0BASE 0x80000000
#define PFS_ID_BANK_256K 0
#define SR_FR 0x04000000
#define G_MTX_MUL 0x00
#define gDPLoadTLUT_pal256(pkt,dram) { gDPSetTextureImage(pkt, G_IM_FMT_RGBA, G_IM_SIZ_16b, 1, dram); gDPTileSync(pkt); gDPSetTile(pkt, 0, 0, 0, 256, G_TX_LOADTILE, 0 , 0, 0, 0, 0, 0, 0); gDPLoadSync(pkt); gDPLoadTLUTCmd(pkt, G_TX_LOADTILE, 255); gDPPipeSync(pkt) }
#define ERR_OSPIRAWSTARTDMA_RANGE 25
#define OS_IM_RCP 0x00000401
#define _OS_THREAD_H_ 
#define OS_PFS_VERSION_HI (OS_PFS_VERSION >> 8)
#define gSPTextureRectangle(pkt,xl,yl,xh,yh,tile,s,t,dsdx,dtdy) _DW({ Gfx *_g = (Gfx *)(pkt); _g->words.w0 = (_SHIFTL(G_TEXRECT, 24, 8) | _SHIFTL(xh, 12, 12) | _SHIFTL(yh, 0, 12)); _g->words.w1 = (_SHIFTL(tile, 24, 3) | _SHIFTL(xl, 12, 12) | _SHIFTL(yl, 0, 12)); gImmp1(pkt, G_RDPHALF_1, (_SHIFTL(s, 16, 16) | _SHIFTL(t, 0, 16))); gImmp1(pkt, G_RDPHALF_2, (_SHIFTL(dsdx, 16, 16) | _SHIFTL(dtdy, 0, 16)));})
#define alCSPGetPan alCSPGetChlPan
#define ERR_OSPROFILEINIT_SIZ 65
#define OS_MESG_PRI_HIGH 1
#define G_CC_BLENDIA2 ENVIRONMENT, SHADE, COMBINED, SHADE, COMBINED, 0, SHADE, 0
#define G_RM_AA_TEX_TERR2 RM_AA_TEX_TERR(2)
#define G_RM_AA_OPA_TERR2 RM_AA_OPA_TERR(2)
#define G_IM_SIZ_4b_LOAD_BLOCK G_IM_SIZ_16b
#define RM_AA_SUB_SURF(clk) AA_EN | IM_RD | CVG_DST_FULL | ZMODE_OPA | ALPHA_CVG_SEL | GBL_c ##clk(G_BL_CLR_IN, G_BL_A_IN, G_BL_CLR_MEM, G_BL_A_MEM)
#define SP_STATUS_CPUSIGNAL SP_STATUS_SIG4
#define G_MWO_FOG 0x00
#define GIO_GIO_INTR_REG (GIO_BASE_REG+0x000)
#define gSPLoadGeometryMode(pkt,word) gSPGeometryMode((pkt),-1,(word))
#define alSeqpGetFXMix alSeqpGetChlFXMix
#define G_RM_ADD2 RM_ADD(2)
#define G_RDPPIPESYNC 0xe7
#define OS_EVENT_COUNTER 3
#define RI_COUNT_REG RI_REFRESH_REG
#define gdSPDefLookAt(rightx,righty,rightz,upx,upy,upz) { {{ {{0,0,0},0,{0,0,0},0,{rightx,righty,rightz},0}}, { {{0,0x80,0},0,{0,0x80,0},0,{upx,upy,upz},0}}} }
#define _OS_SYSTEM_H_ 
#define KDM_TO_PHYS(x) ((u32)(x)&0x1FFFFFFF)
#define TLBLO_PFNSHIFT 6
#define CONT_OVERRUN_ERROR 0x4
#define ERR_OSPISTARTDMA_DIR 30
#define OS_RG_ALIGN_DEFAULT OS_RG_ALIGN_8B
#define OS_VI_MPAL_HPN1 36
#define OS_VI_MPAL_HPN2 40
#define RGBA16_PIXEL_OPAQUE 1
#define ERR_ALSNDPDEALLOCATE 107
#define TRUE 1
#define OS_PFS_VERSION_LO (OS_PFS_VERSION & 255)
#define M_EVENT_H 
#define OS_VI_FPAL_LPN2 46
#define C0_COMPARE 11
#define G_RM_AA_SUB_TERR2 RM_AA_SUB_TERR(2)
#define SP_SET_HALT (1 << 1)
#define SINVALID 0x00000000
#define OS_VI_DITHER_FILTER_OFF 0x0080
#define OS_VI_BIT_NONINTERLACE 0x0001
#define OS_IM_RDBREAD 0x00004401
#define G_CCMUX_TEXEL0_ALPHA 8
#define OS_IM_RDBWRITE 0x00002401
#define AL_PAN_LEFT 0
#define SP_STATUS_DMA_BUSY (1 << 2)
#define gDPLoadTLUT(pkt,count,tmemaddr,dram) { gDPSetTextureImage(pkt, G_IM_FMT_RGBA, G_IM_SIZ_16b, 1, dram); gDPTileSync(pkt); gDPSetTile(pkt, 0, 0, 0, tmemaddr, G_TX_LOADTILE, 0 , 0, 0, 0, 0, 0, 0); gDPLoadSync(pkt); gDPLoadTLUTCmd(pkt, G_TX_LOADTILE, ((count)-1)); gDPPipeSync(pkt); }
#define _OS_CACHE_H_ 
#define M_COLLISION_OBJ_H 
#define EXC_II EXC_CODE(10)
#define GAME_APP_DATA_READY 10
#define SP_SCALE 0x00000010
#define G_TL_LOD (1 << G_MDSFT_TEXTLOD)
#define OC2_NONE (0)
#define C0_PRID 15
#define TLBLO_D 0x4
#define EXC_FPE EXC_CODE(15)
#define TLBLO_G 0x1
#define OS_PHYSICAL_TO_K0(x) (void *)(((u32)(x)+0x80000000))
#define INT8_MAX 0x7F
#define gDPLoadMultiTile(pkt,timg,tmem,rtile,fmt,siz,width,height,uls,ult,lrs,lrt,pal,cms,cmt,masks,maskt,shifts,shiftt) _DW({ gDPSetTextureImage(pkt, fmt, siz, width, timg); gDPSetTile(pkt, fmt, siz, (((((lrs)-(uls)+1) * siz ##_TILE_BYTES)+7)>>3), tmem, G_TX_LOADTILE, 0 , cmt, maskt, shiftt, cms, masks, shifts); gDPLoadSync(pkt); gDPLoadTile( pkt, G_TX_LOADTILE, (uls)<<G_TEXTURE_IMAGE_FRAC, (ult)<<G_TEXTURE_IMAGE_FRAC, (lrs)<<G_TEXTURE_IMAGE_FRAC, (lrt)<<G_TEXTURE_IMAGE_FRAC); gDPPipeSync(pkt); gDPSetTile(pkt, fmt, siz, (((((lrs)-(uls)+1) * siz ##_LINE_BYTES)+7)>>3), tmem, rtile, pal, cmt, maskt, shiftt, cms, masks, shifts); gDPSetTileSize(pkt, rtile, (uls)<<G_TEXTURE_IMAGE_FRAC, (ult)<<G_TEXTURE_IMAGE_FRAC, (lrs)<<G_TEXTURE_IMAGE_FRAC, (lrt)<<G_TEXTURE_IMAGE_FRAC); })
#define OS_ERROR_FMT "/usr/lib/PR/error.fmt"
#define TLBLO_V 0x2
#define gsSPLookAtX(l) gsDma2p( G_MOVEMEM,(l),sizeof(Light),G_MV_LIGHT,G_MVO_LOOKATX)
#define G_SETCOMBINE 0xfc
#define gsDPFullSync() gsDPNoParam(G_RDPFULLSYNC)
#define Z_CMP 0x10
#define VI_V_SYNC_REG (VI_BASE_REG + 0x18)
#define VEC_SET(V,X,Y,Z) V.x = X; V.y = Y; V.z = Z
#define CONFIG_BE_SHFT 15
#define C0_READI 0x1
#define gsDPSetColor(c,d) {{ _SHIFTL(c, 24, 8), (unsigned int)(d) }}
#define CONT_ERR_NOT_READY 12
#define GU_PARSE_STRING_TYPE 6
#define A_RIGHT 0x00
#define gDPSetBlendColor(pkt,r,g,b,a) DPRGBColor(pkt, G_SETBLENDCOLOR, r,g,b,a)
#define OS_EVENT_SP 4
#define gsSPInsertMatrix(where,num) ERROR!! gsSPInsertMatrix is no longer supported.
#define A_MIX 0x10
#define _OS_HOST_H_ 
#define CVG_DST_CLAMP 0
#define WATCHLO_VALIDMASK 0xfffffffb
#define PI_RD_LEN_REG (PI_BASE_REG + 0x08)
#define G_RM_XLU_SURF2 RM_XLU_SURF(2)
#define IS_KSEG1(x) ((u32)(x) >= K1BASE && (u32)(x) < K2BASE)
#define G_RM_ZB_XLU_DECAL RM_ZB_XLU_DECAL(1)
#define STICK_CORRECTED_SCALE (1.0f / (STICK_MAX - STICK_MIN))
#define DPC_STATUS_XBUS_DMEM_DMA (1 << 0)
#define INT16_MAX 0x7FFF
#define G_CC_BLENDRGBDECALA TEXEL0, SHADE, TEXEL0_ALPHA, SHADE, 0, 0, 0, TEXEL0
#define PPARITY_MASK 0x0001
#define OS_PFS_VERSION 0x0200
#define gSPCullDisplayList(pkt,vstart,vend) { Gfx *_g = (Gfx *)(pkt); _g->words.w0 = _SHIFTL(G_CULLDL, 24, 8) | _SHIFTL((vstart)*2, 0, 16); _g->words.w1 = _SHIFTL((vend)*2, 0, 16); }
#define PFS_READ 0
#define G_CYC_COPY (2 << G_MDSFT_CYCLETYPE)
#define UNUSED __attribute__((unused))
#define mPr_POCKETS_SLOT_COUNT 15
#define OC1_TYPE_10 (1 << 4)
#define A_MIXER 12
#define SR_RP 0x08000000
#define OS_IM_COUNTER 0x00008401
#define CHECK_BTN_ALL(state,combo) (~((state) | ~(combo)) == 0)
#define FLASH_STATUS_ERASE_OK 0
#define ERR_ALSNDPSETPRIORITY 111
#define G_CCMUX_COMBINED 0
#define A_SETLOOP 15
#define G_MV_PMTX 6
#define VI_ORIGIN_REG (VI_BASE_REG + 0x04)
#define EXC_OV EXC_CODE(12)
#define PFS_CREATE 2
#define SR_SR 0x00100000
#define M_KANKYO_H 
#define GAME_FAULT_SEND 14
#define SR_SX 0x00000040
#define AI_MIN_BIT_RATE 2
#define G_CC_DECALRGB2 0, 0, 0, COMBINED, 0, 0, 0, SHADE
#define ERR_OSSTARTTHREAD 3
#define G_CC_DECALRGBA 0, 0, 0, TEXEL0, 0, 0, 0, TEXEL0
#define VOICE_STATUS_START 1
#define SR_TS 0x00200000
#define gsDPSetFogColor(r,g,b,a) sDPRGBColor(G_SETFOGCOLOR, r,g,b,a)
#define Z_TRIG CONT_G
#define RI_LATENCY_REG (RI_BASE_REG + 0x14)
#define aSegment(pkt,s,b) { Acmd *_a = (Acmd *)pkt; _a->words.w0 = _SHIFTL(A_SEGMENT, 24, 8); _a->words.w1 = _SHIFTL(s, 24, 8) | _SHIFTL(b, 0, 24); }
#define M_PI 3.14159265358979323846
#define _gsDPLoadTextureBlockTile(timg,tmem,rtile,fmt,siz,width,height,pal,cms,cmt,masks,maskt,shifts,shiftt) gsDPSetTextureImage(fmt, siz ##_LOAD_BLOCK, 1, timg), gsDPSetTile(fmt, siz ##_LOAD_BLOCK, 0, tmem, G_TX_LOADTILE, 0 , cmt, maskt, shiftt, cms, masks, shifts), gsDPLoadSync(), gsDPLoadBlock(G_TX_LOADTILE, 0, 0, (((width)*(height) + siz ##_INCR) >> siz ##_SHIFT)-1, CALC_DXT(width, siz ##_BYTES)), gsDPPipeSync(), gsDPSetTile(fmt, siz, ((((width) * siz ##_LINE_BYTES)+7)>>3), tmem, rtile, pal, cmt, maskt, shiftt, cms, masks, shifts), gsDPSetTileSize(rtile, 0, 0, ((width)-1) << G_TEXTURE_IMAGE_FRAC, ((height)-1) << G_TEXTURE_IMAGE_FRAC)
#define gSPBranchList(pkt,dl) gDma1p(pkt,G_DL,dl,0,G_DL_NOPUSH)
#define OS_TV_MPAL 2
#define gsDPSetBlendColor(r,g,b,a) sDPRGBColor(G_SETBLENDCOLOR, r,g,b,a)
#define SR_UX 0x00000020
#define PFS_ONE_PAGE 8
#define G_CC_1CYUV2RGB TEXEL0, K4, K5, TEXEL0, 0, 0, 0, SHADE
#define TLBHI_PIDMASK 0xff
#define PADDR_SHIFT 4
#define gsDPSetAlphaDither(mode) gsSPSetOtherMode(G_SETOTHERMODE_H, G_MDSFT_ALPHADITHER, 2, mode)
#define MI_CLR_EBUS (1 << 9)
#define INT8_MIN (-0x80)
#define aSaveBuffer(pkt,s) { Acmd *_a = (Acmd *)pkt; _a->words.w0 = _SHIFTL(A_SAVEBUFF, 24, 8); _a->words.w1 = (unsigned int)(s); }
#define G_MDSFT_TEXTCONV 9
#define ERR_OSRECVMESG 8
#define ACTOR_FLAG_10 (1 << 4)
#define G_CC_MODULATERGBA G_CC_MODULATEIA
#define CONT_ADDR_CRC_ER 0x04
#define CONT_CARD_ON 0x01
#define CONT_START 0x1000
#define gDPLoadTile(pkt,t,uls,ult,lrs,lrt) gDPLoadTileGeneric(pkt, G_LOADTILE, t, uls, ult, lrs, lrt)
#define RI_BASE_REG 0x04700000
#define FPART(x) (qs1616(x) & 0xFFFF)
#define MQ_GET_COUNT(mq) ((mq)->validCount)
#define G_TEXRECT 0xe4
#define CONFIG_COHRNT_EXLWR 0x00000005
#define OS_VI_BIT_NTSC 0x0400
#define ERR_OSMAPTLB_INDEX 10
#define gsDPSetOtherMode(mode0,mode1) {{ _SHIFTL(G_RDPSETOTHERMODE,24,8)|_SHIFTL(mode0,0,24), (unsigned int)(mode1) }}
#define ALIGNMENT_H 
#define UNK_PTR void*
#define CONFIG_SB_SHFT 22
#define G_RM_ZB_PCL_SURF2 RM_ZB_PCL_SURF(2)
#define gDPLoadTextureTile(pkt,timg,fmt,siz,width,height,uls,ult,lrs,lrt,pal,cms,cmt,masks,maskt,shifts,shiftt) _DW({ gDPSetTextureImage(pkt, fmt, siz, width, timg); gDPSetTile(pkt, fmt, siz, (((((lrs)-(uls)+1) * siz ##_TILE_BYTES)+7)>>3), 0, G_TX_LOADTILE, 0 , cmt, maskt, shiftt, cms, masks, shifts); gDPLoadSync(pkt); gDPLoadTile( pkt, G_TX_LOADTILE, (uls)<<G_TEXTURE_IMAGE_FRAC, (ult)<<G_TEXTURE_IMAGE_FRAC, (lrs)<<G_TEXTURE_IMAGE_FRAC, (lrt)<<G_TEXTURE_IMAGE_FRAC); gDPPipeSync(pkt); gDPSetTile(pkt, fmt, siz, (((((lrs)-(uls)+1) * siz ##_LINE_BYTES)+7)>>3), 0, G_TX_RENDERTILE, pal, cmt, maskt, shiftt, cms, masks, shifts); gDPSetTileSize(pkt, G_TX_RENDERTILE, (uls)<<G_TEXTURE_IMAGE_FRAC, (ult)<<G_TEXTURE_IMAGE_FRAC, (lrs)<<G_TEXTURE_IMAGE_FRAC, (lrt)<<G_TEXTURE_IMAGE_FRAC); })
#define AL_BANK_VERSION 0x4231
#define gSPLightColor(pkt,n,col) { gMoveWd(pkt, G_MW_LIGHTCOL, G_MWO_a ##n, col); gMoveWd(pkt, G_MW_LIGHTCOL, G_MWO_b ##n, col); }
#define ERR_ALCSEQZEROVEL 129
#define MAXCONTROLLERS 4
#define G_TF_POINT (0 << G_MDSFT_TEXTFILT)
#define lbRTC_MINUTES_PER_HOUR 60
#define VI_MPAL_CLOCK 48628316
#define G_MTX_NOPUSH 0x00
#define _SHIFTL(v,s,w) ((unsigned int) (((unsigned int)(v) & ((0x01 << (w)) - 1)) << (s)))
#define IO_WRITE(addr,data) (*(vu32*)PHYS_TO_K1(addr)=(u32)(data))
#define MI_INTR_MASK_SET_AI (1 << 5)
#define ACTOR_FLAG_40 (1 << 6)
#define M_LIB_H 
#define __LIB_AUDIO__ 
#define CVG_DST_WRAP 0x100
#define G_TL_TILE (0 << G_MDSFT_TEXTLOD)
#define AL_PHASE_ATTACK 0
#define MI_CLR_RDRAM (1 << 12)
#define CHNL_ERR_MASK 0xC0
#define GIO_GIO_SYNC_REG (GIO_BASE_REG+0x400)
#define C0_SR 12
#define ERR_OSPROFILEINIT_ALN 63
#define G_RM_OPA_CI2 RM_OPA_CI(2)
#define G_MOVEWORD 0xdb
#define G_RM_RA_ZB_OPA_INTER2 RM_RA_ZB_OPA_INTER(2)
#define FTOFRAC8(x) ((int) MIN(((x) * (128.0f)), 127.0f) & 0xff)
#define G_IM_SIZ_32b_BYTES 4
#define MQ_IS_FULL(mq) (MQ_GET_COUNT(mq) >= (mq)->msgCount)
#define DPS_TBIST_REG (DPS_BASE_REG + 0x00)
#define G_RM_CLD_SURF2 RM_CLD_SURF(2)
#define SP_SET_SSTEP (1 << 6)
#define DPC_BASE_REG 0x04100000
#define G_RM_AA_ZB_TEX_INTER2 RM_AA_ZB_TEX_INTER(2)
#define INT32_MAX 0x7FFFFFFF
#define MI_INTR_MASK_SET_DP (1 << 11)
#define SP_SET_INTR_BREAK (1 << 8)
#define G_RM_AA_ZB_XLU_LINE2 RM_AA_ZB_XLU_LINE(2)
#define OS_PRIORITY_PIMGR 150
#define G_RM_AA_ZB_XLU_SURF RM_AA_ZB_XLU_SURF(1)
#define CONFIG_EC_3_2 0x7
#define SP_DMA_FULL_REG (SP_BASE_REG + 0x14)
#define VI_CTRL_SERRATE_ON 0x00040
#define OS_VI_FPAL_LAF2 49
#define VI_V_VIDEO_REG VI_V_START_REG
#define RCP_IMASKSHIFT 16
#define G_IMMCMDSIZ 64
#define ERR_ALSNDP_NO_VOICE 105
#define G_RM_AA_XLU_LINE2 RM_AA_XLU_LINE(2)
#define G_MODIFYVTX 0x02
#define M_ACTOR_H 
#define VOICE_STATUS_CANCEL 3
#define DEF_DIR_PAGES 2
#define ERR_OSPROFILESTOP_FLAG 68
#define G_RM_AA_ZB_SUB_TERR RM_AA_ZB_SUB_TERR(1)
#define PI_DOM2_ADDR1 0x05000000
#define CONT_ERR_DEVICE PFS_ERR_DEVICE
#define ALIGNED8 __attribute__ ((aligned (8)))
#define PI_DOM2_ADDR2 0x08000000
#define CVG_DST_SAVE 0x300
#define gsDPSetTextureFilter(type) gsSPSetOtherMode(G_SETOTHERMODE_H, G_MDSFT_TEXTFILT, 2, type)
#define PFS_FILE_EXT_LEN 4
#define G_IM_SIZ_32b 3
#define C_HINV 0x10
#define HOST_PROF_ACK 13
#define G_MTX_PUSH 0x01
#define G_TX_LOADTILE 7
#define gDPLoadSync(pkt) gDPNoParam(pkt, G_RDPLOADSYNC)
#define GU_PARSEGBI_SHOWDMA 8
#define OC1_TYPE_ALL (OC1_TYPE_8 | OC1_TYPE_10 | OC1_TYPE_20)
#define G_RM_OPA_SURF RM_OPA_SURF(1)
#define GU_BLINKRDP_EXTRACT 2
#define CACHERR_EE 0x04000000
#define ALIGN4(val) (((val) + 3) & ~3)
#define ALIGN8(val) (((val) + 7) & ~7)
#define RAD_TO_BINANG(radians) TRUNCF_BINANG((radians) * (0x8000 / (f32)M_PI))
#define FR_POS_FRUSTRATIO_1 0x0000ffff
#define gsDPTileSync() gsDPNoParam(G_RDPTILESYNC)
#define ERR_OSSPTASKLOAD_OUT 58
#define FTOFIX32(x) (long)((x) * (float)0x00010000)
#define RM_AA_ZB_XLU_SURF(clk) AA_EN | Z_CMP | IM_RD | CVG_DST_WRAP | CLR_ON_CVG | FORCE_BL | ZMODE_XLU | GBL_c ##clk(G_BL_CLR_IN, G_BL_A_IN, G_BL_CLR_MEM, G_BL_1MA)
#define ERR_ALSEQP_NO_SOUND 100
#define GU_PARSE_GBI_TYPE 1
#define gDPSetTextureFilter(pkt,type) gSPSetOtherMode(pkt, G_SETOTHERMODE_H, G_MDSFT_TEXTFILT, 2, type)
#define G_RDP_TRI_FILL_MASK 0x08
#define G_MVO_L1 (3*24)
#define G_MVO_L2 (4*24)
#define G_MVO_L3 (5*24)
#define G_MVO_L4 (6*24)
#define G_MVO_L5 (7*24)
#define G_MVO_L7 (9*24)
#define gsSP2Triangles(v00,v01,v02,flag0,v10,v11,v12,flag1) {{ (_SHIFTL(G_TRI2, 24, 8)| __gsSP1Triangle_w1f(v00, v01, v02, flag0)), __gsSP1Triangle_w1f(v10, v11, v12, flag1) }}
#define G_CC_BLENDRGBA TEXEL0, SHADE, TEXEL0_ALPHA, SHADE, 0, 0, 0, SHADE
#define DPC_CLR_XBUS_DMEM_DMA (1 << 0)
#define G_VTX 0x01
#define spInit spX2Init
#define UNK_H 
#define BINANG_TO_RAD_ALT(binang) (((f32)(binang) / 0x8000) * (f32)M_PI)
#define OS_TASK_DP_WAIT 0x0002
#define DPC_CLOCK_REG (DPC_BASE_REG + 0x10)
#define SVINDEXMASK 0x00000380
#define G_SETTILESIZE 0xf2
#define ACTOR_FLAG_400 (1 << 10)
#define ELEM_FLAG_2 (1 << 1)
#define OS_EVENT_CART 2
#define gsSPNoOp() gsDma0p(G_SPNOOP, 0, 0)
#define BOWTIE_VAL 0
#define G_TRI_SHADE_ZBUFF 0xcd
#define SP_SET_SIG1 (1 << 12)
#define SP_SET_SIG2 (1 << 14)
#define SP_SET_SIG3 (1 << 16)
#define SP_SET_SIG4 (1 << 18)
#define SP_SET_SIG6 (1 << 22)
#define SP_SET_SIG7 (1 << 24)
#define M_VIDTASK 3
#define OS_YIELD_AUDIO_SIZE 0x400
#define RM_ZB_OPA_SURF(clk) Z_CMP | Z_UPD | CVG_DST_FULL | ALPHA_CVG_SEL | ZMODE_OPA | GBL_c ##clk(G_BL_CLR_IN, G_BL_A_IN, G_BL_CLR_MEM, G_BL_A_MEM)
#define C0_TAGHI 29
#define RDRAM_BASE_REG 0x03F00000
#define AL_CMIDI_BLOCK_CODE 0xFE
#define G_MAXZ 0x03ff
#define G_MWO_MATRIX_YZ_YW_F 0x2c
#define G_MWO_MATRIX_YZ_YW_I 0x0c
#define G_MDSFT_BLENDMASK 0
#define RM_ZB_OVL_SURF(clk) Z_CMP | IM_RD | CVG_DST_SAVE | FORCE_BL | ZMODE_DEC | GBL_c ##clk(G_BL_CLR_IN, G_BL_A_IN, G_BL_CLR_MEM, G_BL_1MA)
#define AI_MIN_DAC_RATE 132
#define ERR_OSAISETFREQUENCY 14
#define gsSPViewport(v) gsDma2p( G_MOVEMEM, (v), sizeof(Vp), G_MV_VIEWPORT, 0)
#define OS_VI_FPAL_LAN2 48
#define EEPROM_TYPE_4K 0x01
#define FG_BLOCK_Z_NUM (BLOCK_Z_NUM - 4)
#define GAME_LOG_SEND 7
#define RM_AA_TEX_TERR(clk) AA_EN | IM_RD | CVG_DST_CLAMP | CVG_X_ALPHA | ALPHA_CVG_SEL | ZMODE_OPA | TEX_EDGE | GBL_c ##clk(G_BL_CLR_IN, G_BL_A_IN, G_BL_CLR_MEM, G_BL_1MA)
#define G_MDSFT_RENDERMODE 3
#define BLOCK_TOTAL_NUM (BLOCK_X_NUM * BLOCK_Z_NUM)
#define ERR_ALSNDPSETSOUND 110
#define OC1_40 (1 << 6)
#define OS_PM_1M 0x01fe000
#define gsDPLoadMultiBlock_4bS(timg,tmem,rtile,fmt,width,height,pal,cms,cmt,masks,maskt,shifts,shiftt) gsDPSetTextureImage(fmt, G_IM_SIZ_16b, 1, timg), gsDPSetTile(fmt, G_IM_SIZ_16b, 0, tmem, G_TX_LOADTILE, 0 , cmt, maskt, shiftt, cms, masks, shifts), gsDPLoadSync(), gsDPLoadBlock(G_TX_LOADTILE, 0, 0, (((width)*(height)+3)>>2)-1,0), gsDPPipeSync(), gsDPSetTile(fmt, G_IM_SIZ_4b, ((((width)>>1)+7)>>3), tmem, rtile, pal, cmt, maskt, shiftt, cms, masks, shifts), gsDPSetTileSize(rtile, 0, 0, ((width)-1) << G_TEXTURE_IMAGE_FRAC, ((height)-1) << G_TEXTURE_IMAGE_FRAC)
#define PI_DOMAIN1 0
#define PI_DOMAIN2 1
#define G_RM_AA_OPA_SURF RM_AA_OPA_SURF(1)
#define C_CDX 0xc
#define G_CCMUX_SCALE 6
#define SR_RE 0x02000000
#define OS_TASK_LOADABLE 0x0004
#define __bool_true_false_are_defined 1
#define BOOT_ADDRESS_EMU 0x20010000
#define ERR_ALSEQPUNMAP 123
#define AI_DRAM_ADDR_REG (AI_BASE_REG + 0x00)
#define gSPPopMatrix(pkt,n) gSPPopMatrixN((pkt), (n), 1)
#define RDRAM_REF_INTERVAL_REG (RDRAM_BASE_REG + 0x10)
#define PI_BSD_DOM1_LAT_REG (PI_BASE_REG + 0x14)
#define gMoveWd(pkt,index,offset,data) gDma1p((pkt), G_MOVEWORD, data, offset, index)
#define PFS_CORRUPTED 0x2
#define _FINALROM 1
#define G_TRI_SHADE_TXTR_ZBUFF 0xcf
#define MQ_IS_EMPTY(mq) (MQ_GET_COUNT(mq) == 0)
#define MI_INTR_MASK_SET_PI (1 << 9)
#define C0_TAGLO 28
#define ERR_OSPIRAWREADIO 19
#define aInterleave(pkt,l,r) { Acmd *_a = (Acmd *)pkt; _a->words.w0 = _SHIFTL(A_INTERLEAVE, 24, 8); _a->words.w1 = _SHIFTL(l, 16, 16) | _SHIFTL(r, 0, 16); }
#define SP_SET_RSPSIGNAL SP_SET_SIG3
#define gsDPSetKeyR(cR,sR,wR) {{ _SHIFTL(G_SETKEYR, 24, 8), _SHIFTL(wR, 16, 12) | _SHIFTL(cR, 8, 8) | _SHIFTL(sR, 0, 8) }}
#define GIO_BASE_REG 0x18000000
#define M_COLLISION_BG_H 
#define XUT_VEC (K0BASE+0x80)
#define G_IM_SIZ_32b_LOAD_BLOCK G_IM_SIZ_32b
#define OS_PM_4K 0x0000000
#define PI_BSD_DOM2_PWD_REG (PI_BASE_REG + 0x28)
#define OS_PM_4M 0x07fe000
#define AL_DEFAULT_PRIORITY 5
#define SP_WR_LEN_REG (SP_BASE_REG + 0x0C)
#define OS_PRIORITY_IDLE 0
#define _OS_FLASH_H_ 
#define AL_USEC_PER_FRAME 16000
#define ERR_OSVISETSPECIAL_VALUE 43
#define gSPInsertMatrix(pkt,where,num) ERROR!! gSPInsertMatrix is no longer supported.
#define G_RM_AA_TEX_EDGE RM_AA_TEX_EDGE(1)
#define CONT_ERR_INVALID PFS_ERR_INVALID
#define BINANG_ROT180(angle) ((s16)(angle + 0x8000))
#define TLBCTXT_VPNSHIFT 4
#define K2SIZE 0x20000000
#define OS_MESG_TYPE_BASE (10)
#define G_GEOMETRYMODE 0xd9
#define M_PLAY_H 
#define OS_PM_256K 0x007e000
#define aLoadADPCM(pkt,c,d) { Acmd *_a = (Acmd *)pkt; _a->words.w0 = _SHIFTL(A_LOADADPCM, 24, 8) | _SHIFTL(c, 0, 24); _a->words.w1 = (unsigned int) d; }
#define ERR_ALSEQPINVALIDPROG 121
#define VI_CURRENT_REG (VI_BASE_REG + 0x10)
#define RESET_BUTTON 0x80
#define gDPFullSync(pkt) gDPNoParam(pkt, G_RDPFULLSYNC)
#define AL_SEQBANK_VERSION 'S1'
#define DEG_TO_BINANG_ALT2(degrees) TRUNCF_BINANG(((degrees) * 0x10000) / 360.0f)
#define DEG_TO_BINANG_ALT3(degrees) ((degrees) * (0x8000 / 180.0f))
#define PFS_INITIALIZED 0x1
#define GBL_c1(m1a,m1b,m2a,m2b) (m1a) << 30 | (m1b) << 26 | (m2a) << 22 | (m2b) << 18
#define MI_INTR_MASK_SET_SI (1 << 3)
#define gdSPDefLights6(ar,ag,ab,r1,g1,b1,x1,y1,z1,r2,g2,b2,x2,y2,z2,r3,g3,b3,x3,y3,z3,r4,g4,b4,x4,y4,z4,r5,g5,b5,x5,y5,z5,r6,g6,b6,x6,y6,z6) { {{ {ar,ag,ab},0,{ar,ag,ab},0}}, {{{ {r1,g1,b1},0,{r1,g1,b1},0,{x1,y1,z1},0}}, {{ {r2,g2,b2},0,{r2,g2,b2},0,{x2,y2,z2},0}}, {{ {r3,g3,b3},0,{r3,g3,b3},0,{x3,y3,z3},0}}, {{ {r4,g4,b4},0,{r4,g4,b4},0,{x4,y4,z4},0}}, {{ {r5,g5,b5},0,{r5,g5,b5},0,{x5,y5,z5},0}}, {{ {r6,g6,b6},0,{r6,g6,b6},0,{x6,y6,z6},0}}} }
#define MI_INTR_MASK_SET_SP (1 << 1)
#define gDPSetColorDither(pkt,mode) gSPSetOtherMode(pkt, G_SETOTHERMODE_H, G_MDSFT_RGBDITHER, 2, mode)
#define ALIGN(s,align) (((u32)(s) + ((align)-1)) & ~((align)-1))
#define __gsSPLine3D_w1f(v0,v1,wd,flag) (((flag) == 0) ? __gsSPLine3D_w1(v0, v1, wd): __gsSPLine3D_w1(v1, v0, wd))
#define gSPBranchLessZraw(pkt,dl,vtx,zval) { Gfx *_g = (Gfx *)(pkt); _g->words.w0 = _SHIFTL(G_RDPHALF_1,24,8); _g->words.w1 = (unsigned int)(dl); _g = (Gfx *)(pkt); _g->words.w0 = (_SHIFTL(G_BRANCH_Z,24,8)| _SHIFTL((vtx)*5,12,12)|_SHIFTL((vtx)*2,0,12)); _g->words.w1 = (unsigned int)(zval); }
#define CONTROLLER3(game) (&(game)->input[2])
#define GU_PARSERDP_VERBOSE 1
#define M_VIEW_H 
#define CHNL_ERR_NORESP 0x80
#define ALIGNSZ (sizeof(long long))
#define _OS_RSP_H_ 
#define G_TRI_SHADE 0xcc
#define _GBI_H_ 
#define SP_Z 0x00000008
#define CONT_EEPROM_BUSY 0x80
#define G_CK_KEY (1 << G_MDSFT_COMBKEY)
#define TLBPGMASK_64K 0x1e000
#define gDPSetTile(pkt,fmt,siz,line,tmem,tile,palette,cmt,maskt,shiftt,cms,masks,shifts) _DW({ Gfx *_g = (Gfx *)(pkt); _g->words.w0 = _SHIFTL(G_SETTILE, 24, 8) | _SHIFTL(fmt, 21, 3) | _SHIFTL(siz, 19, 2) | _SHIFTL(line, 9, 9) | _SHIFTL(tmem, 0, 9); _g->words.w1 = _SHIFTL(tile, 24, 3) | _SHIFTL(palette, 20, 4) | _SHIFTL(cmt, 18, 2) | _SHIFTL(maskt, 14, 4) | _SHIFTL(shiftt, 10, 4) |_SHIFTL(cms, 8, 2) | _SHIFTL(masks, 4, 4) | _SHIFTL(shifts, 0, 4); })
#define OC2_TYPE_10 OC1_TYPE_10
#define TLBRAND_RANDSHIFT 0
#define G_IM_SIZ_4b_SHIFT 2
#define AL_FX_BIGROOM 2
#define G_MWO_bLIGHT_5 0x64
#define OS_CYCLES_TO_NSEC(c) (((u64)(c)*(1000000000LL/15625000LL))/(OS_CPU_COUNTER/15625000LL))
#define COLCHECK_FLAG_NONE (0)
#define G_TRI2 0x06
#define VOICE_STATUS_END 7
#define G_RM_ZB_XLU_SURF RM_ZB_XLU_SURF(1)
#define VI_CTRL_ANTIALIAS_MASK 0x00300
#define MI_INTR_MASK_SET_VI (1 << 7)
#define RAMROM_RELEASE_OFFSET 0xc
#define C_6A8180_H 
#define C0_WRITER 0x6
#define EXC_WATCH EXC_CODE(23)
#define aPan(pkt,f,d,s) { Acmd *_a = (Acmd *)pkt; _a->words.w0 = (_SHIFTL(A_PAN, 24, 8) | _SHIFTL(f, 16, 8) | _SHIFTL(d, 0, 16)); _a->words.w1 = (unsigned int)(s); }
#define G_DL_NOPUSH 0x01
#define L_JPAD CONT_LEFT
#define NEW_FLASH 1
#define GRDPCMD(x) (0xff-(x))
#define gSP2Triangles(pkt,v00,v01,v02,flag0,v10,v11,v12,flag1) _DW({ Gfx *_g = (Gfx *)(pkt); _g->words.w0 = (_SHIFTL(G_TRI2, 24, 8)| __gsSP1Triangle_w1f(v00, v01, v02, flag0)); _g->words.w1 = __gsSP1Triangle_w1f(v10, v11, v12, flag1); })
#define IM_RD 0x40
#define G_RM_AA_ZB_PCL_SURF2 RM_AA_ZB_PCL_SURF(2)
#define RCP_STAT_PRINT rmonPrintf("current=%x start=%x end=%x dpstat=%x spstat=%x\n", IO_READ(DPC_CURRENT_REG), IO_READ(DPC_START_REG), IO_READ(DPC_END_REG), IO_READ(DPC_STATUS_REG), IO_READ(SP_STATUS_REG))
#define ERR_OSPROFILESTART_TIME 66
#define G_CC_YUV2RGB TEXEL1, K4, K5, TEXEL1, 0, 0, 0, 0
#define gsSPLoadUcodeEx(uc_start,uc_dstart,uc_dsize) {{ _SHIFTL(G_RDPHALF_1,24,8), (unsigned int)(uc_dstart), }}, {{ _SHIFTL(G_LOAD_UCODE,24,8)| _SHIFTL((int)(uc_dsize)-1,0,16), (unsigned int)(uc_start), }}
#define gSPDmaWrite(pkt,dmem,dram,size) gSPDma_io((pkt),1,(dmem),(dram),(size))
#define SR_IBIT2 0x00000200
#define ERR_ALSEQOVERRUN 131
#define SP_STATUS_RSPSIGNAL SP_STATUS_SIG3
#define SP_TEXSHUF 0x00000200
#define OS_TASK_USR0 0x0010
#define gsDPLoadMultiBlock_4b(timg,tmem,rtile,fmt,width,height,pal,cms,cmt,masks,maskt,shifts,shiftt) gsDPSetTextureImage(fmt, G_IM_SIZ_16b, 1, timg), gsDPSetTile(fmt, G_IM_SIZ_16b, 0, tmem, G_TX_LOADTILE, 0 , cmt, maskt, shiftt, cms, masks, shifts), gsDPLoadSync(), gsDPLoadBlock(G_TX_LOADTILE, 0, 0, (((width)*(height)+3)>>2)-1, CALC_DXT_4b(width)), gsDPPipeSync(), gsDPSetTile(fmt, G_IM_SIZ_4b, ((((width)>>1)+7)>>3), tmem, rtile, pal, cmt, maskt, shiftt, cms, masks, shifts), gsDPSetTileSize(rtile, 0, 0, ((width)-1) << G_TEXTURE_IMAGE_FRAC, ((height)-1) << G_TEXTURE_IMAGE_FRAC)
#define VI_H_START_REG (VI_BASE_REG + 0x24)
#define G_MWO_NUMLIGHT 0x00
#define G_IM_FMT_CI 2
#define VI_H_VIDEO_REG VI_H_START_REG
#define RGBA8(r,g,b,a) ((((r) & 0xFF) << 24) | (((g) & 0xFF) << 16) | (((b) & 0xFF) << 8) | (((a) & 0xFF) << 0))
#define MI_INTR_PI (1 << 4)
#define ARRAY_COUNT(arr) (s32)(sizeof(arr) / sizeof(arr[0]))
#define DPC_BUFBUSY_REG (DPC_BASE_REG + 0x14)
#define G_CC_MODULATEI_PRIM TEXEL0, 0, PRIMITIVE, 0, 0, 0, 0, PRIMITIVE
#define NDEBUG 1
#define NPCW_MAX NPCW_GET_WALK_NUM(ANIMAL_NUM_MAX)
#define _OS_SI_H_ 
#define CONT_ERR_VOICE_NO_RESPONSE 15
#define SP_CLR_HALT (1 << 0)
#define C0_WATCHHI 19
#define G_SETBLENDCOLOR 0xf9
#define ERR_OSPROFILEINIT_STR 61
#define RI_RERROR_REG (RI_BASE_REG + 0x18)
#define A_SAVEBUFF 6
#define CACHERR_PIDX_MASK 0x00000007
#define VI_INTR_REG (VI_BASE_REG + 0x0C)
#define gsDma0p(c,s,l) {{ _SHIFTL((c), 24, 8) | _SHIFTL((l), 0, 24), (unsigned int)(s) }}
#define VI_PAL_CLOCK 49656530
#define AI_DACRATE_REG (AI_BASE_REG + 0x10)
#define C0_RFE 0x10
#define OS_APP_NMI_BUFSIZE 64
#define PIF_RAM_END 0x1FC007FF
#define ERR_OSGETTIME 74
#define gDPSetTextureDetail(pkt,type) gSPSetOtherMode(pkt, G_SETOTHERMODE_H, G_MDSFT_TEXTDETAIL, 2, type)
#define FLASH_PULSE 0x0c
#define RI_CURRENT_LOAD_REG (RI_BASE_REG + 0x08)
#define G_CCMUX_ENV_ALPHA 12
#define VOICE_WARN_NOT_FIT 0x4000
#define ALIGN128(val) (((val) + 0x7F) & ~0x7F)
#define NO_SOUND_ERR_MASK 0x01
#define _OS_MESSAGE_H_ 
#define gsSPLookAt(la) gsSPLookAtX(la), gsSPLookAtY((char *)(la)+16)
#define OS_IM_CART 0x00000c01
#define G_MV_POINT 12
#define G_MWO_POINT_ST 0x14
#define C0_RAND 1
#define FLASH_VERSION_MX_B_AND_D 0x00c2001d
#define C0_WRITEI 0x2
#define CONT_A 0x8000
#define CONT_B 0x4000
#define MI_INTR_MASK_REG (MI_BASE_REG + 0x0C)
#define G_SHADE 0x00000004
#define SI_STATUS_DMA_ERROR (1 << 3)
#define CONT_C 0x0002
#define E_VEC (K0BASE+0x180)
#define HOST_DATA_ACK 17
#define SR_CU0 0x10000000
#define SR_CU1 0x20000000
#define SR_CU3 0x80000000
#define OC2_TYPE_8 OC1_TYPE_8
#define lbRTC_HOURS_PER_DAY 24
#define G_RM_RA_OPA_SURF RM_RA_OPA_SURF(1)
#define C0_WATCHLO 18
#define SP_CLR_INTR (1 << 3)
#define gSPPerspNormalize(pkt,s) gMoveWd(pkt, G_MW_PERSPNORM, 0, (s))
#define _SHIFTR(v,s,w) ((unsigned int)(((unsigned int)(v) >> (s)) & ((0x01 << (w)) - 1)))
#define HOST_PRINTF_ACK 6
#define G_IM_FMT_IA 3
#define RM_AA_OPA_TERR(clk) AA_EN | IM_RD | CVG_DST_CLAMP | ZMODE_OPA | ALPHA_CVG_SEL | GBL_c ##clk(G_BL_CLR_IN, G_BL_A_IN, G_BL_CLR_MEM, G_BL_1MA)
#define G_CC_MODULATERGBDECALA_PRIM G_CC_MODULATEIDECALA_PRIM
#define OS_VI_FPAL_LPF1 43
#define OS_VI_FPAL_LPF2 47
#define GAME_EXIT 16
#define DPS_BASE_REG 0x04200000
#define L_CBUTTONS CONT_C
#define FLASH_VERSION_MEI 0x003200f1
#define PFS_INODE_SIZE_PER_PAGE 128
#define TLBLO_UNCACHED 0x10
#define G_TD_DETAIL (2 << G_MDSFT_TEXTDETAIL)
#define A_LEFT 0x02
#define OS_PRIORITY_MAX 255
#define UNREACHABLE 
#define GU_PARSE_ABI_TYPE 5
#define G_CC_MODULATEIA2 COMBINED, 0, SHADE, 0, COMBINED, 0, SHADE, 0
#define EEP16K_MAXBLOCKS 256
#define OS_VI_DIVOT_ON 0x0010
#define G_LIGHTING 0x00020000
#define G_TC_FILT (6 << G_MDSFT_TEXTCONV)
#define EXC_CPU EXC_CODE(11)
#define gDPSetCombineMode(pkt,a,b) gDPSetCombineLERP(pkt, a, b)
#define ERR_OSJAMMESG 7
#define ACTOR_FLAG_800 (1 << 11)
#define VI_NTSC_CLOCK 48681812
#define INT32_MIN (-0x80000000)
#define gsDPLoadTLUT_pal16(pal,dram) gsDPSetTextureImage(G_IM_FMT_RGBA, G_IM_SIZ_16b, 1, dram), gsDPTileSync(), gsDPSetTile(0, 0, 0, (256+(((pal)&0xf)*16)), G_TX_LOADTILE, 0 , 0, 0, 0, 0, 0, 0), gsDPLoadSync(), gsDPLoadTLUTCmd(G_TX_LOADTILE, 15), gsDPPipeSync()
#define G_IM_SIZ_4b_INCR 3
#define gsDPSetCombine(muxs0,muxs1) {{ _SHIFTL(G_SETCOMBINE, 24, 8) | _SHIFTL(muxs0, 0, 24), (unsigned int)(muxs1) }}
#define G_CC_HILITERGBDECALA2 ENVIRONMENT, COMBINED, TEXEL0, COMBINED, 0, 0, 0, TEXEL0
#define ELEM_FLAG_1 (1 << 0)
#define SP_FASTCOPY 0x00000020
#define PI_BSD_DOM1_RLS_REG (PI_BASE_REG + 0x20)
#define G_BL_CLR_BL 2
#define _GU_H_ 
#define ERR_ALSYN_NO_UPDATE 106
#define EXC_SYSCALL EXC_CODE(8)
#define G_CC_BLENDI ENVIRONMENT, SHADE, TEXEL0, SHADE, 0, 0, 0, SHADE
#define D_JPAD CONT_DOWN
#define G_TRI1 0x05
#define RCP_IMASK 0x003f0000
#define G_RM_AA_ZB_XLU_DECAL RM_AA_ZB_XLU_DECAL(1)
#define K1SIZE 0x20000000
#define OC1_NONE (0)
#define gsDPSetKeyGB(cG,sG,wG,cB,sB,wB) {{ (_SHIFTL(G_SETKEYGB, 24, 8) | _SHIFTL(wG, 12, 12) | _SHIFTL(wB, 0, 12)), (_SHIFTL(cG, 24, 8) | _SHIFTL(sG, 16, 8) | _SHIFTL(cB, 8, 8) | _SHIFTL(sB, 0, 8)) }}
#define G_RM_AA_ZB_TEX_TERR2 RM_AA_ZB_TEX_TERR(2)
#define ATTRIBUTES_H 
#define FPCSR_RM_RM 0x00000003
#define FPCSR_RM_RN 0x00000000
#define FPCSR_RM_RP 0x00000002
#define __gsSP1Triangle_w1(v0,v1,v2) (_SHIFTL((v0)*2,16,8)|_SHIFTL((v1)*2,8,8)|_SHIFTL((v2)*2,0,8))
#define OC2_TYPE_20 OC1_TYPE_20
#define FPCSR_RM_RZ 0x00000001
#define UNK_ARGS 
#define VI_CTRL_GAMMA_ON 0x00008
#define EXC_VCED EXC_CODE(31)
#define gsSPTexture(s,t,level,tile,on) {{ (_SHIFTL(G_TEXTURE,24,8) | _SHIFTL(BOWTIE_VAL,16,8) | _SHIFTL((level),11,3) | _SHIFTL((tile),8,3) | _SHIFTL((on),1,7)), (_SHIFTL((s),16,16) | _SHIFTL((t),0,16)) }}
#define EXC_VCEI EXC_CODE(14)
#define ERR_ALSEQNUMTRACKS 116
#define DPC_STATUS_CMD_BUSY (1 << 6)
#define gSPLight(pkt,l,n) gDma2p((pkt),G_MOVEMEM,(l),sizeof(Light),G_MV_LIGHT,(n)*24+24)
#define G_TX_LDBLK_MAX_TXL 2047
#define G_MWO_MATRIX_YX_YY_F 0x28
#define G_RM_RA_ZB_OPA_INTER RM_RA_ZB_OPA_INTER(1)
#define C0_COUNT 9
#define PFS_EOF 1
#define SADDR_SHIFT 4
#define OS_CLOCK_RATE 62500000LL
#define SP_IMEM_START 0x04001000
#define G_TEXTURE 0xd7
#define RM_AA_OPA_SURF(clk) AA_EN | IM_RD | CVG_DST_CLAMP | ZMODE_OPA | ALPHA_CVG_SEL | GBL_c ##clk(G_BL_CLR_IN, G_BL_A_IN, G_BL_CLR_MEM, G_BL_A_MEM)
#define ERR_OSCREATEMESGQUEUE 5
#define gSPSegment(pkt,segment,base) gMoveWd(pkt, G_MW_SEGMENT, (segment)*4, base)
#define PIF_RAM_START 0x1FC007C0
#define CHNL_ERR_FRAME 0x80
#define gSPSetLights0(pkt,name) { gSPNumLights(pkt,NUMLIGHTS_0); gSPLight(pkt,&name.l[0],1); gSPLight(pkt,&name.a,2); }
#define AI_LEN_REG (AI_BASE_REG + 0x04)
#define FLASH_VERSION_MX_PROTO_A 0x00c20000
#define lbRTC_YEAR_MIN 1901
#define alCSPSetPan alCSPSetChlPan
#define OS_VI_GAMMA_ON 0x0001
#define ZMODE_XLU 0x800
#define gDma0p(pkt,c,s,l) { Gfx *_g = (Gfx *)(pkt); _g->words.w0 = _SHIFTL((c), 24, 8) | _SHIFTL((l), 0, 24); _g->words.w1 = (unsigned int)(s); }
#define ACTOR_FLAG_20000000 (1 << 29)
#define RDRAM_0_DEVICE_ID 0
#define RDRAM_DEVICE_MANUF_REG (RDRAM_BASE_REG + 0x24)
#define gDPLoadTextureBlockYuvS(pkt,timg,fmt,siz,width,height,pal,cms,cmt,masks,maskt,shifts,shiftt) { gDPSetTextureImage(pkt, fmt, siz ##_LOAD_BLOCK, 1, timg); gDPSetTile(pkt, fmt, siz ##_LOAD_BLOCK, 0, 0, G_TX_LOADTILE, 0 , cmt, maskt, shiftt, cms, masks, shifts); gDPLoadSync(pkt); gDPLoadBlock(pkt, G_TX_LOADTILE, 0, 0, (((width)*(height) + siz ##_INCR) >> siz ##_SHIFT)-1,0); gDPPipeSync(pkt); gDPSetTile(pkt, fmt, siz, (((width) * 1)+7)>>3, 0, G_TX_RENDERTILE, pal, cmt, maskt, shiftt, cms, masks, shifts); gDPSetTileSize(pkt, G_TX_RENDERTILE, 0, 0, ((width)-1) << G_TEXTURE_IMAGE_FRAC, ((height)-1) << G_TEXTURE_IMAGE_FRAC); }
#define ACMD_SIZE 32
#define FPCSR_CE 0x00020000
#define ERR_OSCREATEVIMANAGER 49
#define FPCSR_CI 0x00001000
#define G_BL_1 2
#define FPCSR_CU 0x00002000
#define FPCSR_CV 0x00010000
#define FPCSR_CZ 0x00008000
#define RM_AA_ZB_OPA_DECAL(clk) AA_EN | Z_CMP | IM_RD | CVG_DST_WRAP | ALPHA_CVG_SEL | ZMODE_DEC | GBL_c ##clk(G_BL_CLR_IN, G_BL_A_IN, G_BL_CLR_MEM, G_BL_A_MEM)
#define OS_VI_FPAL_LPN1 42
#define gDma1p(pkt,c,s,l,p) _DW({ Gfx *_g = (Gfx *)(pkt); _g->words.w0 = (_SHIFTL((c), 24, 8) | _SHIFTL((p), 16, 8) | _SHIFTL((l), 0, 16)); _g->words.w1 = (unsigned int)(s); })
#define G_MW_NUMLIGHT 0x02
#define TLBCTXT_VPNMASK 0x7ffff0
#define _OS_AI_H_ 
#define PFS_SECTOR_SIZE (PFS_INODE_SIZE_PER_PAGE/PFS_SECTOR_PER_BANK)
#define gDPSetAlphaDither(pkt,mode) gSPSetOtherMode(pkt, G_SETOTHERMODE_H, G_MDSFT_ALPHADITHER, 2, mode)
#define STICK_MIN 9.899495f
#define gDma2p(pkt,c,adrs,len,idx,ofs) _DW({ Gfx *_g = (Gfx *)(pkt); _g->words.w0 = (_SHIFTL((c),24,8)|_SHIFTL(((len)-1)/8,19,5)| _SHIFTL((ofs)/8,8,8)|_SHIFTL((idx),0,8)); _g->words.w1 = (unsigned int)(adrs); })
#define BOOT_ADDRESS_ULTRA 0x80000400
#define M_FIELD_MAKE_H 
#define G_LOADTLUT 0xf0
#define RM_AA_DEC_LINE(clk) AA_EN | IM_RD | CVG_DST_FULL | CVG_X_ALPHA | ALPHA_CVG_SEL | FORCE_BL | ZMODE_OPA | GBL_c ##clk(G_BL_CLR_IN, G_BL_A_IN, G_BL_CLR_MEM, G_BL_1MA)
#define EXC_RADE EXC_CODE(4)
#define G_MDSFT_TEXTFILT 12
#define HANIWA_ITEM_HOLD_NUM 4
#define STICK_UNCORRECTED_SCALE (1.0f / STICK_MAX)
#define BUF_CTRL_SIZE ALIGNSZ
#define FPCSR_EI 0x00000080
#define AA_EN 0x8
#define FPCSR_EO 0x00000200
#define gsDPTextureRectangleFlip(xl,yl,xh,yh,tile,s,t,dsdx,dtdy) {{ (_SHIFTL(G_TEXRECTFLIP, 24, 8) | _SHIFTL(xh, 12, 12) | _SHIFTL(yh, 0, 12)), (_SHIFTL(tile, 24, 3) | _SHIFTL(xl, 12, 12) | _SHIFTL(yl, 0, 12)), }}, {{ _SHIFTL(s, 16, 16) | _SHIFTL(t, 0, 16), _SHIFTL(dsdx, 16, 16) | _SHIFTL(dtdy, 0, 16) }}
#define FPCSR_EU 0x00000100
#define FPCSR_EV 0x00000800
#define FPCSR_EZ 0x00000400
#define OS_EVENT_CPU_BREAK 10
#define IS_KUSEG(x) ((u32)(x) < K0BASE)
#define K0_TO_K1(x) ((u32)(x)|0xA0000000)
#define M_SNOWMAN_H 
#define R_VEC (K1BASE+0x1fc00000)
#define OS_EVENT_SP_BREAK 11
#define TXL2WORDS(txls,b_txl) MAX(1, ((txls)*(b_txl)/8))
#define RDRAM_1_END 0x003FFFFF
#define FPCSR_FI 0x00000004
#define C0_CAUSE 13
#define PI_DOMAIN2_REG PI_BSD_DOM2_LAT_REG
#define FPCSR_FS 0x01000000
#define FPCSR_FV 0x00000040
#define FPCSR_FZ 0x00000020
#define MI_SET_RDRAM (1 << 13)
#define gsImmp21(c,p0,p1,dat) {{ _SHIFTL((c), 24, 8) | _SHIFTL((p0), 8, 16) | _SHIFTL((p1), 0, 8), (unsigned int) (dat) }}
#define LB_RTC_H 
#define _ULTRA64_H_ 
#define G_RM_RA_OPA_SURF2 RM_RA_OPA_SURF(2)
#define RM_ZB_OPA_DECAL(clk) Z_CMP | CVG_DST_FULL | ALPHA_CVG_SEL | ZMODE_DEC | GBL_c ##clk(G_BL_CLR_IN, G_BL_A_IN, G_BL_CLR_MEM, G_BL_A_MEM)
#define _RAMROM_H 
#define gsDPSetPrimColor(m,l,r,g,b,a) {{ (_SHIFTL(G_SETPRIMCOLOR, 24, 8) | _SHIFTL(m, 8, 8) | _SHIFTL(l, 0, 8)), (_SHIFTL(r, 24, 8) | _SHIFTL(g, 16, 8) | _SHIFTL(b, 8, 8) | _SHIFTL(a, 0, 8)) }}
#define G_MV_MMTX 2
#define TWO_HEAD_ARENA_H 
#define _OS_RDP_H_ 
#define ERR_ALSEQSYSEX 119
#define CONT_JOYPORT 0x0004
#define G_SETTIMG 0xfd
#define G_CK_NONE (0 << G_MDSFT_COMBKEY)
#define G_RM_AA_PCL_SURF2 RM_AA_PCL_SURF(2)
#define SP_HIDDEN 0x00000004
#define SP_STATUS_HALT (1 << 0)
#define FLASH_PAGE_SIZE 0xf
#define PI_DOM_PGS_OFS 0x08
#define GU_PARSEGBI_NONEST 2
#define GU_PARSERDP_DUMPONLY 32
#define VI_CTRL_GAMMA_DITHER_ON 0x00004
#define GAME_YEAR_MIN 2000
#define G_CC_HILITERGB2 ENVIRONMENT, COMBINED, TEXEL0, COMBINED, 0, 0, 0, SHADE
#define OS_MESG_TYPE_LOOPBACK (OS_MESG_TYPE_BASE + 0)
#define LEO_BLOCK_MODE 1
#define MI_INTR_MASK_SI (1 << 1)
#define gSPModifyVertex(pkt,vtx,where,val) { Gfx *_g = (Gfx *)(pkt); _g->words.w0 = (_SHIFTL(G_MODIFYVTX,24,8)| _SHIFTL((where),16,8)|_SHIFTL((vtx)*2,0,16)); _g->words.w1 = (unsigned int)(val); }
#define AI_STATUS_FIFO_FULL (1 << 31)
#define RAMROM_PRINTF_ADDR (RAMROM_MSG_ADDR + (4*RAMROM_BUF_SIZE))
#define ALFailIf(condition,error) if (condition) { return; }
#define _gsDPLoadTextureBlock_4b(timg,tmem,fmt,width,height,pal,cms,cmt,masks,maskt,shifts,shiftt) gsDPSetTextureImage(fmt, G_IM_SIZ_16b, 1, timg), gsDPSetTile(fmt, G_IM_SIZ_16b, 0, tmem, G_TX_LOADTILE, 0 , cmt, maskt, shiftt, cms, masks, shifts), gsDPLoadSync(), gsDPLoadBlock(G_TX_LOADTILE, 0, 0, (((width)*(height)+3)>>2)-1, CALC_DXT_4b(width)), gsDPPipeSync(), gsDPSetTile(fmt, G_IM_SIZ_4b, ((((width)>>1)+7)>>3), tmem, G_TX_RENDERTILE, pal, cmt, maskt, shiftt, cms, masks, shifts), gsDPSetTileSize(G_TX_RENDERTILE, 0, 0, ((width)-1) << G_TEXTURE_IMAGE_FRAC, ((height)-1) << G_TEXTURE_IMAGE_FRAC)
#define SP_CLR_CPUSIGNAL SP_CLR_SIG4
#define G_TX_DXT_FRAC 11
#define SP_STATUS_YIELDED SP_STATUS_SIG1
#define gSPVertex(pkt,v,n,v0) _DW({ Gfx *_g = (Gfx *)(pkt); _g->words.w0 = _SHIFTL(G_VTX,24,8)|_SHIFTL((n),12,8)|_SHIFTL((v0)+(n),1,7); _g->words.w1 = (unsigned int)(v); })
#define SP_STATUS_BROKE (1 << 1)
#define gSPSprite2DBase(pkt,s) gDma1p(pkt, G_SPRITE2D_BASE, s, sizeof(uSprite), 0)
#define gsSPBranchLessZraw(dl,vtx,zval) {{ _SHIFTL(G_RDPHALF_1,24,8), (unsigned int)(dl), }}, {{ _SHIFTL(G_BRANCH_Z,24,8)|_SHIFTL((vtx)*5,12,12)|_SHIFTL((vtx)*2,0,12), (unsigned int)(zval), }}
#define G_MVO_L0 (2*24)
#define OS_VI_BIT_32PIXEL 0x0080
#define AL_FX_NONE 0
#define GAMEALLOC_H 
#define ERR_OSSENDMESG 6
#define OS_VI_MPAL_LPN1 28
#define ERR_OSPISTARTDMA_RANGE 34
#define CAUSE_CESHIFT 28
#define gsSPPopMatrixN(n,num) gsDma2p( G_POPMTX,(num)*64,64,2,0)
#define gDPSetCombine(pkt,muxs0,muxs1) { Gfx *_g = (Gfx *)(pkt); _g->words.w0 = _SHIFTL(G_SETCOMBINE, 24, 8) | _SHIFTL(muxs0, 0, 24); _g->words.w1 = (unsigned int)(muxs1); }
#define RDRAM_RESET_MODE 0
#define OS_MIN_STACKSIZE 72
#define G_MVO_L6 (8*24)
#define G_RM_AA_XLU_SURF RM_AA_XLU_SURF(1)
#define DEVICE_TYPE_INIT 7
#define OS_TASK_YIELDED 0x0001
#define OS_GBPAK_ROM_ID_SIZE 0x50
#define CONFIG_IC_SHFT 9
#define SP_TEXSHIFT 0x00000080
#define CAUSE_IPSHIFT 8
#define CONT_ERR_VOICE_WORD 14
#define gDPSetCombineLERP(pkt,a0,b0,c0,d0,Aa0,Ab0,Ac0,Ad0,a1,b1,c1,d1,Aa1,Ab1,Ac1,Ad1) _DW({ Gfx *_g = (Gfx *)(pkt); _g->words.w0 = _SHIFTL(G_SETCOMBINE, 24, 8) | _SHIFTL(GCCc0w0(G_CCMUX_ ##a0, G_CCMUX_ ##c0, G_ACMUX_ ##Aa0, G_ACMUX_ ##Ac0) | GCCc1w0(G_CCMUX_ ##a1, G_CCMUX_ ##c1), 0, 24); _g->words.w1 = (unsigned int)(GCCc0w1(G_CCMUX_ ##b0, G_CCMUX_ ##d0, G_ACMUX_ ##Ab0, G_ACMUX_ ##Ad0) | GCCc1w1(G_CCMUX_ ##b1, G_ACMUX_ ##Aa1, G_ACMUX_ ##Ac1, G_CCMUX_ ##d1, G_ACMUX_ ##Ab1, G_ACMUX_ ##Ad1)); })
#define DEG_TO_BINANG_ALT(degrees) TRUNCF_BINANG(((degrees) / 180.0f) * 0x8000)
#define VI_V_START_REG (VI_BASE_REG + 0x28)
#define gsSP1Triangle(v0,v1,v2,flag) {{ _SHIFTL(G_TRI1, 24, 8)|__gsSP1Triangle_w1f(v0, v1, v2, flag), 0 }}
#define gDPLoadTextureBlockYuv(pkt,timg,fmt,siz,width,height,pal,cms,cmt,masks,maskt,shifts,shiftt) { gDPSetTextureImage(pkt, fmt, siz ##_LOAD_BLOCK, 1, timg); gDPSetTile(pkt, fmt, siz ##_LOAD_BLOCK, 0, 0, G_TX_LOADTILE, 0 , cmt, maskt, shiftt, cms, masks, shifts); gDPLoadSync(pkt); gDPLoadBlock(pkt, G_TX_LOADTILE, 0, 0, (((width)*(height) + siz ##_INCR) >> siz ##_SHIFT) -1, CALC_DXT(width, siz ##_BYTES)); gDPPipeSync(pkt); gDPSetTile(pkt, fmt, siz, (((width) * 1)+7)>>3, 0, G_TX_RENDERTILE, pal, cmt, maskt, shiftt, cms, masks, shifts); gDPSetTileSize(pkt, G_TX_RENDERTILE, 0, 0, ((width)-1) << G_TEXTURE_IMAGE_FRAC, ((height)-1) << G_TEXTURE_IMAGE_FRAC); }
#define G_RM_AA_ZB_PCL_SURF RM_AA_ZB_PCL_SURF(1)
#define G_TD_CLAMP (0 << G_MDSFT_TEXTDETAIL)
#define G_ZS_PIXEL (0 << G_MDSFT_ZSRCSEL)
#define G_CC_PRIMITIVE 0, 0, 0, PRIMITIVE, 0, 0, 0, PRIMITIVE
#define ACTOR_FGNAME_GET_F000(fgName) (((fgName) & 0xF000) >> 12)
#define FLASH_STATUS_WRITE_BUSY 1
#define G_MWO_POINT_XYSCREEN 0x18
#define ACTOR_FLAG_1000000 (1 << 24)
#define G_MWO_MATRIX_XZ_XW_F 0x24
#define G_MWO_MATRIX_XZ_XW_I 0x04
#define gSPFogPosition(pkt,min,max) gMoveWd(pkt, G_MW_FOG, G_MWO_FOG, (_SHIFTL((128000/((max)-(min))),16,16) | _SHIFTL(((500-(min))*256/((max)-(min))),0,16)))
#define G_NOOP 0x00
#define PI_DOM1_ADDR1 0x06000000
#define PI_DOM1_ADDR2 0x10000000
#define PI_DOM1_ADDR3 0x1FD00000
#define _gsDPLoadTextureBlock(timg,tmem,fmt,siz,width,height,pal,cms,cmt,masks,maskt,shifts,shiftt) gsDPSetTextureImage(fmt, siz ##_LOAD_BLOCK, 1, timg), gsDPSetTile(fmt, siz ##_LOAD_BLOCK, 0, tmem, G_TX_LOADTILE, 0 , cmt, maskt, shiftt, cms, masks, shifts), gsDPLoadSync(), gsDPLoadBlock(G_TX_LOADTILE, 0, 0, (((width)*(height) + siz ##_INCR) >> siz ##_SHIFT)-1, CALC_DXT(width, siz ##_BYTES)), gsDPPipeSync(), gsDPSetTile(fmt, siz, ((((width) * siz ##_LINE_BYTES)+7)>>3), tmem, G_TX_RENDERTILE, pal, cmt, maskt, shiftt, cms, masks, shifts), gsDPSetTileSize(G_TX_RENDERTILE, 0, 0, ((width)-1) << G_TEXTURE_IMAGE_FRAC, ((height)-1) << G_TEXTURE_IMAGE_FRAC)
#define SP_CLR_SIG0 (1 << 9)
#define SP_CLR_SIG1 (1 << 11)
#define SP_CLR_SIG2 (1 << 13)
#define SP_CLR_SIG3 (1 << 15)
#define SP_CLR_SIG4 (1 << 17)
#define SP_CLR_SIG6 (1 << 21)
#define SP_CLR_SIG7 (1 << 23)
#define M_MAIL_H 
#define OS_GBPAK_GBCART_ON 0x80
#define G_IM_SIZ_16b_INCR 0
#define ERR_OSCREATETHREAD_PRI 2
#define PI_STATUS_DMA_BUSY (1 << 0)
#define SCLEANEXCL 0x00001000
#define RMON_DBG_BUF_SIZE 2048
#define NUMLIGHTS_0 1
#define NUMLIGHTS_1 1
#define NUMLIGHTS_2 2
#define NUMLIGHTS_3 3
#define NUMLIGHTS_5 5
#define NUMLIGHTS_6 6
#define NUMLIGHTS_7 7
#define G_LOAD_UCODE 0xdd
#define R_TRIG CONT_R
#define gsDPLoadTextureBlock(timg,fmt,siz,width,height,pal,cms,cmt,masks,maskt,shifts,shiftt) gsDPSetTextureImage(fmt, siz ##_LOAD_BLOCK, 1, timg), gsDPSetTile(fmt, siz ##_LOAD_BLOCK, 0, 0, G_TX_LOADTILE, 0 , cmt, maskt, shiftt, cms, masks, shifts), gsDPLoadSync(), gsDPLoadBlock(G_TX_LOADTILE, 0, 0, (((width)*(height) + siz ##_INCR) >> siz ##_SHIFT)-1, CALC_DXT(width, siz ##_BYTES)), gsDPPipeSync(), gsDPSetTile(fmt, siz, ((((width) * siz ##_LINE_BYTES)+7)>>3), 0, G_TX_RENDERTILE, pal, cmt, maskt, shiftt, cms, masks, shifts), gsDPSetTileSize(G_TX_RENDERTILE, 0, 0, ((width)-1) << G_TEXTURE_IMAGE_FRAC, ((height)-1) << G_TEXTURE_IMAGE_FRAC)
#define G_RDP_TRI_TXTR_MASK 0x02
#define gDPSetBlendMask(pkt,mask) gDPNoOp(pkt)
#define CONT_ERR_VOICE_MEMORY 13
#define G_RM_AA_ZB_XLU_LINE RM_AA_ZB_XLU_LINE(1)
#define G_AD_NOISE (2 << G_MDSFT_ALPHADITHER)
#define ERR_OSSPTASKLOAD_OUTSIZE 59
#define G_TT_NONE (0 << G_MDSFT_TEXTLUT)
#define ERR_OSVISWAPBUFFER_VIMGR 48
#define gDPSetMaskImage(pkt,i) gDPSetDepthImage(pkt, i)
#define C_6B8A70_H 
#define G_MW_PERSPNORM 0x0e
#define _G_CC_TWOCOLORTEX PRIMITIVE, SHADE, TEXEL0, SHADE, 0, 0, 0, SHADE
#define G_RM_VISCVG2 RM_VISCVG(2)
#define G_RM_AA_ZB_SUB_SURF2 RM_AA_ZB_SUB_SURF(2)
#define AI_MPAL_MIN_FREQ 3000
#define gDPSetAlphaCompare(pkt,type) gSPSetOtherMode(pkt, G_SETOTHERMODE_L, G_MDSFT_ALPHACOMPARE, 2, type)
#define gsSPGeometryMode(c,s) {{ (_SHIFTL(G_GEOMETRYMODE,24,8)|_SHIFTL(~(u32)(c),0,24)),(u32)(s) }}
#define PFS_MAX_BANKS 62
#define G_MWO_CLIP_RPX 0x14
#define K0SIZE 0x20000000
#define RM_ADD(clk) IM_RD | CVG_DST_SAVE | FORCE_BL | ZMODE_OPA | GBL_c ##clk(G_BL_CLR_IN, G_BL_A_FOG, G_BL_CLR_MEM, G_BL_1)
#define G_CC_HILITERGB PRIMITIVE, SHADE, TEXEL0, SHADE, 0, 0, 0, SHADE
#define RM_AA_ZB_PCL_SURF(clk) AA_EN | Z_CMP | Z_UPD | IM_RD | CVG_DST_CLAMP | ZMODE_OPA | G_AC_DITHER | GBL_c ##clk(G_BL_CLR_IN, G_BL_A_IN, G_BL_CLR_MEM, G_BL_1MA)
#define OS_GBPAK_POWER_OFF 0x00
#define KUBASE 0
#define OS_VI_NTSC_LAN1 2
#define C0_LLADDR 17
#define G_MWO_CLIP_RPY 0x1c
#define PFS_ID_3AREA 6
#define G_CCMUX_CENTER 6
#define VI_CTRL_PIXEL_ADV_MASK 0x01000
#define G_RDP_ADDR_FIXUP 3
#define spFinish spX2Finish
#define M_NPC_WALK_H 
#define G_RM_AA_SUB_SURF2 RM_AA_SUB_SURF(2)
#define DPC_STATUS_REG (DPC_BASE_REG + 0x0C)
#define G_TX_NOMIRROR 0
#define G_BZ_ORTHO 1
#define G_MWO_CLIP_RNY 0x0c
#define G_OFF (0)
#define ERR_OSVISETEVENT 46
#define G_BL_A_FOG 1
#define __USE_ISOC99 1
#define OS_GBPAK_GBCART_PULL 0x40
#define SP_CLR_SIG5 (1 << 19)
#define MI_INTR_MASK_CLR_AI (1 << 4)
#define RDRAM_1_CONFIG 0x00400
#define OS_VI_MPAL_LAF2 35
#define G_CV_K0 175
#define AI_PAL_MAX_FREQ 376000
#define G_CV_K2 -89
#define G_CV_K3 222
#define G_CV_K5 42
#define RAMROM_LOG_ADDR (RAMROM_MSG_ADDR + (5*RAMROM_BUF_SIZE))
#define G_AD_NOTPATTERN (1 << G_MDSFT_ALPHADITHER)
#define RM_AA_ZB_XLU_LINE(clk) AA_EN | Z_CMP | IM_RD | CVG_DST_CLAMP | CVG_X_ALPHA | ALPHA_CVG_SEL | FORCE_BL | ZMODE_XLU | GBL_c ##clk(G_BL_CLR_IN, G_BL_A_IN, G_BL_CLR_MEM, G_BL_1MA)
#define FLASH_START_ADDR 0x08000000
#define G_MDSFT_CYCLETYPE 20
#define AL_DEFAULT_VOICE 0
#define ERR_OSWRITEHOST_ADDR 72
#define DECR(x) ((x) == 0 ? 0 : --(x))
#define __ULTRAERROR_H__ 
#define G_CC_MODULATEIA_PRIM TEXEL0, 0, PRIMITIVE, 0, TEXEL0, 0, PRIMITIVE, 0
#define SP_IBIST_REG 0x04080004
#define G_CULL_BACK 0x00000400
#define ERR_OSSPTASKLOAD_DRAM 57
#define ACTOR_FLAG_800000 (1 << 23)
#define G_CC_CHROMA_KEY2 TEXEL0, CENTER, SCALE, 0, 0, 0, 0, 0
#define _G_CC_BLENDPEDECALA ENVIRONMENT, PRIMITIVE, TEXEL0, PRIMITIVE, 0, 0, 0, TEXEL0
#define G_MOVEMEM 0xdc
#define G_TEXTURE_IMAGE_FRAC 2
#define MI_INTR_MASK_CLR_DP (1 << 10)
#define gsSPClearGeometryMode(word) gsSPGeometryMode((word),0)
#define gDPWord(pkt,wordhi,wordlo) { Gfx *_g = (Gfx *)(pkt); gImmp1(pkt, G_RDPHALF_1, (unsigned int)(wordhi)); gImmp1(pkt, G_RDPHALF_2, (unsigned int)(wordlo)); }
#define NUM_DL(nb) ((nb)*DL_BM_OVERHEAD +DL_SPRITE_OVERHEAD)
#define SEGMENT_ADDR(num,off) (((num) << 24) + (off))
#define G_CC_DECALRGB 0, 0, 0, TEXEL0, 0, 0, 0, SHADE
#define DPC_CLR_PIPE_CTR (1 << 7)
#define VI_DRAM_ADDR_REG VI_ORIGIN_REG
#define gsDPParam(cmd,param) {{ _SHIFTL(cmd, 24, 8), (param) }}
#define G_CC_MODULATEI2 COMBINED, 0, SHADE, 0, 0, 0, 0, SHADE
#define ZMODE_INTER 0x400
#define CONT_UP 0x0800
#define OS_IM_SI 0x00020401
#define RDRAM_END RDRAM_1_END
#define BOOT_ADDRESS_COSIM 0x80002000
#define ERR_OSVIGETCURRENTMODE 36
#define RDRAM_0_END 0x001FFFFF
#define TLBLO_CACHSHIFT 3
#define PI_DOMAIN1_REG PI_BSD_DOM1_LAT_REG
#define G_RM_AA_SUB_TERR RM_AA_SUB_TERR(1)
#define G_CCMUX_COMBINED_ALPHA 7
#define gsSPTextureL(s,t,level,xparam,tile,on) {{ (_SHIFTL(G_TEXTURE,24,8) | _SHIFTL((xparam),16,8) | _SHIFTL((level),11,3) | _SHIFTL((tile),8,3) | _SHIFTL((on),1,7)), (_SHIFTL((s),16,16) | _SHIFTL((t),0,16)) }}
#define G_CC_MODULATERGBA2 G_CC_MODULATEIA2
#define gDPLoadMultiBlock_4b(pkt,timg,tmem,rtile,fmt,width,height,pal,cms,cmt,masks,maskt,shifts,shiftt) { gDPSetTextureImage(pkt, fmt, G_IM_SIZ_16b, 1, timg); gDPSetTile(pkt, fmt, G_IM_SIZ_16b, 0, tmem, G_TX_LOADTILE, 0, cmt, maskt, shiftt, cms, masks, shifts); gDPLoadSync(pkt); gDPLoadBlock(pkt, G_TX_LOADTILE, 0, 0, (((width)*(height)+3)>>2)-1, CALC_DXT_4b(width)); gDPPipeSync(pkt); gDPSetTile(pkt, fmt, G_IM_SIZ_4b, ((((width)>>1)+7)>>3), tmem, rtile, pal, cmt, maskt, shiftt, cms, masks, shifts); gDPSetTileSize(pkt, rtile, 0, 0, ((width)-1) << G_TEXTURE_IMAGE_FRAC, ((height)-1) << G_TEXTURE_IMAGE_FRAC); }
#define RM_AA_XLU_SURF(clk) AA_EN | IM_RD | CVG_DST_WRAP | CLR_ON_CVG | FORCE_BL | ZMODE_OPA | GBL_c ##clk(G_BL_CLR_IN, G_BL_A_IN, G_BL_CLR_MEM, G_BL_1MA)
#define PFS_BANK_LAPPED_BY 8
#define OLD_FLASH 0
#define OS_YIELD_DATA_SIZE 0xc00
#define MASS_HEAVY 0xFE
#define G_MWO_MATRIX_ZX_ZY_F 0x30
#define gsDPSetScissorFrac(mode,ulx,uly,lrx,lry) {{ _SHIFTL(G_SETSCISSOR, 24, 8) | _SHIFTL((int)((ulx)), 12, 12) | _SHIFTL((int)((uly)), 0, 12), _SHIFTL(mode, 24, 2) | _SHIFTL((int)(lrx), 12, 12) | _SHIFTL((int)(lry), 0, 12) }}
#define G_MWO_MATRIX_ZX_ZY_I 0x10
#define gsDPSetTextureConvert(type) gsSPSetOtherMode(G_SETOTHERMODE_H, G_MDSFT_TEXTCONV, 3, type)
#define G_RM_TEX_EDGE RM_TEX_EDGE(1)
#define G_TX_NOMASK 0
#define CONT_ERR_CONTRFAIL CONT_OVERRUN_ERROR
#define G_RM_PCL_SURF2 RM_PCL_SURF(2)
#define OS_CYCLES_TO_USEC(c) (((u64)(c)*(1000000LL/15625LL))/(OS_CPU_COUNTER/15625LL))
#define OS_VI_BIT_POINTSAMPLE 0x0020
#define TLBLO_CACHMASK 0x38
#define G_AC_THRESHOLD (1 << G_MDSFT_ALPHACOMPARE)
#define ERR_OSMALLOC 52
#define RDRAM_REF_ROW_REG (RDRAM_BASE_REG + 0x14)
#define RDRAM_1_DEVICE_ID 1
#define GPACK_ZDZ(z,dz) ((z) << 2 | (dz))
#define gsDma2p(c,adrs,len,idx,ofs) {{ (_SHIFTL((c),24,8)|_SHIFTL(((len)-1)/8,19,5)| _SHIFTL((ofs)/8,8,8)|_SHIFTL((idx),0,8)), (unsigned int)(adrs) }}
#define G_RM_RA_ZB_OPA_SURF2 RM_RA_ZB_OPA_SURF(2)
#define CONT_TYPE_MOUSE 0x0002
#define RM_ZB_XLU_SURF(clk) Z_CMP | IM_RD | CVG_DST_FULL | FORCE_BL | ZMODE_XLU | GBL_c ##clk(G_BL_CLR_IN, G_BL_A_IN, G_BL_CLR_MEM, G_BL_1MA)
#define G_IM_FMT_YUV 1
#define ERR_OSWRITEHOST_SIZE 73
#define OS_K0_TO_PHYSICAL(x) (u32)(((char *)(x)-0x80000000))
#define OS_VI_MPAL_LAN1 30
#define OS_VI_MPAL_LAN2 34
#define VI_CTRL_ANTIALIAS_MODE_1 0x00100
#define VI_CTRL_ANTIALIAS_MODE_2 0x00200
#define VI_CTRL_ANTIALIAS_MODE_3 0x00300
#define ECC_VEC (K0BASE+0x100)
#define RDRAM_LENGTH (2 * 512 * 2048)
#define gDPSetOtherMode(pkt,mode0,mode1) _DW({ Gfx *_g = (Gfx *)(pkt); _g->words.w0 = _SHIFTL(G_RDPSETOTHERMODE,24,8)|_SHIFTL(mode0,0,24); _g->words.w1 = (unsigned int)(mode1); })
#define SI_STATUS_RD_BUSY (1 << 1)
#define gDPLoadTextureTile_4b(pkt,timg,fmt,width,height,uls,ult,lrs,lrt,pal,cms,cmt,masks,maskt,shifts,shiftt) { gDPSetTextureImage(pkt, fmt, G_IM_SIZ_8b, ((width)>>1), timg); gDPSetTile(pkt, fmt, G_IM_SIZ_8b, (((((lrs)-(uls)+1)>>1)+7)>>3), 0, G_TX_LOADTILE, 0 , cmt, maskt, shiftt, cms, masks, shifts); gDPLoadSync(pkt); gDPLoadTile( pkt, G_TX_LOADTILE, (uls)<<(G_TEXTURE_IMAGE_FRAC-1), (ult)<<(G_TEXTURE_IMAGE_FRAC), (lrs)<<(G_TEXTURE_IMAGE_FRAC-1), (lrt)<<(G_TEXTURE_IMAGE_FRAC)); gDPPipeSync(pkt); gDPSetTile(pkt, fmt, G_IM_SIZ_4b, (((((lrs)-(uls)+1)>>1)+7)>>3), 0, G_TX_RENDERTILE, pal, cmt, maskt, shiftt, cms, masks, shifts); gDPSetTileSize(pkt, G_TX_RENDERTILE, (uls)<<G_TEXTURE_IMAGE_FRAC, (ult)<<G_TEXTURE_IMAGE_FRAC, (lrs)<<G_TEXTURE_IMAGE_FRAC, (lrt)<<G_TEXTURE_IMAGE_FRAC); }
#define OTHER_TYPES_H 
#define G_CULL_FRONT 0x00000200
#define DEVICE_TYPE_64DD 2
#define ERR_OSPISTARTDMA_PRI 29
#define SR_IMASK 0x0000ff00
#define AI_MPAL_MAX_FREQ 368000
#define VI_X_SCALE_REG (VI_BASE_REG + 0x30)
#define G_MWO_aLIGHT_7 0x90
#define gDPSetTextureLUT(pkt,type) gSPSetOtherMode(pkt, G_SETOTHERMODE_H, G_MDSFT_TEXTLUT, 2, type)
#define G_SETPRIMDEPTH 0xee
#define G_MW_LIGHTCOL 0x0a
#define gDPPipelineMode(pkt,mode) gSPSetOtherMode(pkt, G_SETOTHERMODE_H, G_MDSFT_PIPELINE, 1, mode)
#define PROJECTED_TO_SCREEN_Y(projectedPos,invW) ((projectedPos).y * (invW) * (-SCREEN_HEIGHT / 2) + (SCREEN_HEIGHT / 2))
#define G_RM_AA_ZB_TEX_EDGE2 RM_AA_ZB_TEX_EDGE(2)
#define G_MW_MATRIX 0x00
#define G_BL_CLR_MEM 1
#define DCACHE_LINEMASK (DCACHE_LINESIZE-1)
#define G_RM_ZB_PCL_SURF RM_ZB_PCL_SURF(1)
#define OS_LOG_FLOAT(x) (*(int *) &(x))
#define RAMROM_SIZE (0x1000000)
#define OS_MESG_TYPE_VRETRACE (OS_MESG_TYPE_BASE + 3)
#define WATCHLO_WTRAP 0x00000001
#define G_IM_SIZ_16b 2
#define C0_CACHE_ERR 27
#define TLBHI_VPN2SHIFT 13
#define OS_EVENT_FAULT 12
#define EEPROM_BLOCK_SIZE 8
#define RDRAM_ADDR_SELECT_REG (RDRAM_BASE_REG + 0x20)
#define RAMROM_FONTDATA_SIZE 1152
#define gDPSetCycleType(pkt,type) gSPSetOtherMode(pkt, G_SETOTHERMODE_H, G_MDSFT_CYCLETYPE, 2, type)
#define AL_HEAP_INIT 0
#define SP_STATUS_SIG0 (1 << 7)
#define SP_STATUS_SIG1 (1 << 8)
#define SP_STATUS_SIG2 (1 << 9)
#define SP_STATUS_SIG3 (1 << 10)
#define SP_STATUS_SIG4 (1 << 11)
#define SP_STATUS_SIG5 (1 << 12)
#define SP_STATUS_SIG6 (1 << 13)
#define SP_STATUS_SIG7 (1 << 14)
#define ERR_ALSEQPUNKNOWNMIDI 122
#define CONT_EEP16K 0x4000
#define gsSPBranchLessZrg(dl,vtx,zval,near,far,flag,zmin,zmax) {{ _SHIFTL(G_RDPHALF_1,24,8), (unsigned int)(dl), }}, {{ _SHIFTL(G_BRANCH_Z,24,8)|_SHIFTL((vtx)*5,12,12)|_SHIFTL((vtx)*2,0,12), G_DEPTOZSrg(zval, near, far, flag, zmin, zmax), }}
#define G_CC_REFLECTRGB ENVIRONMENT, 0, TEXEL0, SHADE, 0, 0, 0, SHADE
#define gsSPSegment(segment,base) gsMoveWd( G_MW_SEGMENT, (segment)*4, base)
#define UINTPTR_MAX 0xFFFFFFFF
#define gsDPSetTextureImage(f,s,w,i) gsSetImage(G_SETTIMG, f, s, w, i)
#define AI_STATUS_DMA_BUSY (1 << 30)
#define PFS_ERR_NOPACK 1
#define SP_TRANSPARENT 0x00000001
#define gSPEndDisplayList(pkt) _DW({ Gfx *_g = (Gfx *)(pkt); _g->words.w0 = _SHIFTL(G_ENDDL, 24, 8); _g->words.w1 = 0; })
#define _OS_VOICE_H_ 
#define VI_CTRL_PIXEL_ADV_1 0x01000
#define VI_CTRL_PIXEL_ADV_3 0x03000
#define OS_IM_SW1 0x00000501
#define SEGMENT_NUMBER(a) (((unsigned int)(a) << 4) >> 28)
#define G_RM_ZB_CLD_SURF RM_ZB_CLD_SURF(1)
#define OS_OTHERS 2
#define A_RATE 0x00
#define GU_PARSEGBI_FLTMTX 4
#define AL_FRAME_INIT -1
#define RAMROM_BOOTADDR_OFFSET 0x8
#define ERR_OSCREATEREGION_ALIGN 50
#define osSpTaskStart(tp) { osSpTaskLoad((tp)); osSpTaskStartGo((tp)); }
#define gDPSetRenderMode(pkt,c0,c1) gSPSetOtherMode(pkt, G_SETOTHERMODE_L, G_MDSFT_RENDERMODE, 29, (c0) | (c1))
#define G_TEXRECTFLIP 0xe5
#define PFS_ID_PAGE PFS_ONE_PAGE * 0
#define NOTE_OFF_ERR_MASK 0x02
#define ACTOR_FLAG_20 (1 << 5)
#define MI_INTR_MASK_CLR_PI (1 << 8)
#define RAMROM_MSG_HDR_SIZE (3*sizeof(long))
#define RM_AA_TEX_EDGE(clk) AA_EN | IM_RD | CVG_DST_CLAMP | CVG_X_ALPHA | ALPHA_CVG_SEL | ZMODE_OPA | TEX_EDGE | GBL_c ##clk(G_BL_CLR_IN, G_BL_A_IN, G_BL_CLR_MEM, G_BL_A_MEM)
#define DEVICE_TYPE_FLASH 8
#define aSetVolume(pkt,f,v,t,r) { Acmd *_a = (Acmd *)pkt; _a->words.w0 = (_SHIFTL(A_SETVOL, 24, 8) | _SHIFTL(f, 16, 16) | _SHIFTL(v, 0, 16)); _a->words.w1 = _SHIFTL(t, 16, 16) | _SHIFTL(r, 0, 16); }
#define ERR_OSPISTARTDMA_SIZE 33
#define AL_STOPPED 0
#define PFS_ID_2AREA 4
#define _UCODE_H_ 
#define OS_VI_BIT_ANTIALIAS 0x0010
#define PFS_ID_BROKEN 0x4
#define ACTOR_FLAG_20000 (1 << 17)
#define RAND(x) (guRandom()%x)
#define ADPCMVSIZE 8
#define OS_VI_NTSC_LPF1 1
#define RDRAM_CONFIG_REG (RDRAM_BASE_REG + 0x00)
#define MI_MODE_INIT (1 << 7)
#define G_CD_NOISE (2 << G_MDSFT_RGBDITHER)
#define ERR_ALHEAPCORRUPT 126
#define OVERLAY_DISP __gfxCtx->overlay.p
#define gSPTexture(pkt,s,t,level,tile,on) _DW({ Gfx *_g = (Gfx *)(pkt); _g->words.w0 = (_SHIFTL(G_TEXTURE,24,8) | _SHIFTL(BOWTIE_VAL,16,8) | _SHIFTL((level),11,3) | _SHIFTL((tile),8,3) | _SHIFTL((on),1,7)); _g->words.w1 = (_SHIFTL((s),16,16) | _SHIFTL((t),0,16)); })
#define _OS_EXCEPTION_H_ 
#define OS_VI_GAMMA_OFF 0x0002
#define CAUSE_SW1 0x00000100
#define CAUSE_SW2 0x00000200
#define MI_INTR_MASK_CLR_SP (1 << 0)
#define PRE_RENDER_H 
#define G_POPMTX 0xd8
#define aMix(pkt,f,g,i,o) { Acmd *_a = (Acmd *)pkt; _a->words.w0 = (_SHIFTL(A_MIXER, 24, 8) | _SHIFTL(f, 16, 8) | _SHIFTL(g, 0, 16)); _a->words.w1 = _SHIFTL(i,16, 16) | _SHIFTL(o, 0, 16); }
#define NO_VOICE_ERR_MASK 0x04
#define PI_STATUS_CLR_INTR (1 << 1)
#define VI_Y_SCALE_REG (VI_BASE_REG + 0x34)
#define C_FILL 0x14
#define ACTOR_FLAG_200000 (1 << 21)
#define G_RM_ZB_OPA_SURF2 RM_ZB_OPA_SURF(2)
#define OS_IM_ALL 0x003fff01
#define OS_RG_ALIGN_16B 16
#define F3DEX_GBI_2 1
#define gsDPLoadTextureBlock_4bS(timg,fmt,width,height,pal,cms,cmt,masks,maskt,shifts,shiftt) gsDPSetTextureImage(fmt, G_IM_SIZ_16b, 1, timg), gsDPSetTile(fmt, G_IM_SIZ_16b, 0, 0, G_TX_LOADTILE, 0 , cmt, maskt, shiftt, cms, masks, shifts), gsDPLoadSync(), gsDPLoadBlock(G_TX_LOADTILE, 0, 0, (((width)*(height)+3)>>2)-1,0), gsDPPipeSync(), gsDPSetTile(fmt, G_IM_SIZ_4b, ((((width)>>1)+7)>>3), 0, G_TX_RENDERTILE, pal, cmt, maskt, shiftt, cms, masks, shifts), gsDPSetTileSize(G_TX_RENDERTILE, 0, 0, ((width)-1) << G_TEXTURE_IMAGE_FRAC, ((height)-1) << G_TEXTURE_IMAGE_FRAC)
#define AL_CMIDI_CNTRL_LOOPEND 103
#define ERR_OSSETTIMER 76
#define OS_MESG_TYPE_COUNTER (OS_MESG_TYPE_BASE + 4)
#define CONT_ABSOLUTE 0x0001
#define G_MWO_bLIGHT_1 0x04
#define G_MWO_bLIGHT_2 0x1c
#define G_MWO_bLIGHT_3 0x34
#define G_MWO_bLIGHT_4 0x4c
#define G_MWO_bLIGHT_6 0x7c
#define G_MWO_bLIGHT_7 0x94
#define G_MWO_bLIGHT_8 0xac
#define ERR_OSFREE_REGION 53
#define gsDPSetRenderMode(c0,c1) gsSPSetOtherMode(G_SETOTHERMODE_L, G_MDSFT_RENDERMODE, 29, (c0) | (c1))
#define G_LINE3D 0x08
#define SP_SET_SIG0 (1 << 10)
#define gDPSetTileSize(pkt,t,uls,ult,lrs,lrt) gDPLoadTileGeneric(pkt, G_SETTILESIZE, t, uls, ult, lrs, lrt)
#define gsSP1Quadrangle(v0,v1,v2,v3,flag) {{ (_SHIFTL(G_QUAD, 24, 8)| __gsSP1Quadrangle_w1f(v0, v1, v2, v3, flag)), __gsSP1Quadrangle_w2f(v0, v1, v2, v3, flag) }}
#define MI_INTR_MASK_CLR_VI (1 << 6)
#define gSPLine3D(pkt,v0,v1,flag) { Gfx *_g = (Gfx *)(pkt); _g->words.w0 = _SHIFTL(G_LINE3D, 24, 8)| __gsSPLine3D_w1f(v0, v1, 0, flag); _g->words.w1 = 0; }
#define UT_VEC K0BASE
#define gDPTextureRectangleFlip(pkt,xl,yl,xh,yh,tile,s,t,dsdx,dtdy) { Gfx *_g = (Gfx *)(pkt); if (pkt); _g->words.w0 = (_SHIFTL(G_TEXRECTFLIP, 24, 8) | _SHIFTL(xh, 12, 12) | _SHIFTL(yh, 0, 12)); _g->words.w1 = (_SHIFTL(tile, 24, 3) | _SHIFTL(xl, 12, 12) | _SHIFTL(yl, 0, 12)); _g ++; _g->words.w0 = (_SHIFTL(s, 16, 16) | _SHIFTL(t, 0, 16)); _g->words.w1 = (_SHIFTL(dsdx, 16, 16) | _SHIFTL(dtdy, 0, 16)); }
#define gsSPBranchList(dl) gsDma1p( G_DL,dl,0,G_DL_NOPUSH)
#define G_MW_SEGMENT 0x06
#define SP_SET_SIG5 (1 << 20)
#define spDraw spX2Draw
#define LIBC_STDINT_H 
#define G_MDSFT_ZSRCSEL 2
#define _OS_CONVERT_H_ 
#define PDIRTYEXCL 0x00C0
#define MACROS_H 
#define RM_CLD_SURF(clk) IM_RD | CVG_DST_SAVE | FORCE_BL | ZMODE_OPA | GBL_c ##clk(G_BL_CLR_IN, G_BL_A_IN, G_BL_CLR_MEM, G_BL_1MA)
#define SI_STATUS_INTERRUPT (1 << 12)
#define LIGHT_1 1
#define LIGHT_2 2
#define LIGHT_4 4
#define LIGHT_7 7
#define G_MDSFT_TEXTDETAIL 17
#define MAX_BUFCOUNT 0x8000
#define __gsSP1Quadrangle_w2f(v0,v1,v2,v3,flag) (((flag) == 0) ? __gsSP1Triangle_w1(v0, v2, v3): ((flag) == 1) ? __gsSP1Triangle_w1(v1, v3, v0): ((flag) == 2) ? __gsSP1Triangle_w1(v2, v0, v1): __gsSP1Triangle_w1(v3, v1, v2))
#define M2CTX 1
#define AL_CMIDI_CNTRL_LOOPSTART 102
#define PI_BSD_DOM1_PWD_REG (PI_BASE_REG + 0x18)
#define gSPDmaRead(pkt,dmem,dram,size) gSPDma_io((pkt),0,(dmem),(dram),(size))
#define LIGHT_3 3
#define gsDPLoadMultiBlockS(timg,tmem,rtile,fmt,siz,width,height,pal,cms,cmt,masks,maskt,shifts,shiftt) gsDPSetTextureImage(fmt, siz ##_LOAD_BLOCK, 1, timg), gsDPSetTile(fmt, siz ##_LOAD_BLOCK, 0, tmem, G_TX_LOADTILE, 0 , cmt, maskt,shiftt, cms, masks, shifts), gsDPLoadSync(), gsDPLoadBlock(G_TX_LOADTILE, 0, 0, (((width)*(height) + siz ##_INCR) >> siz ##_SHIFT)-1, 0 ), gsDPPipeSync(), gsDPSetTile(fmt, siz, ((((width) * siz ##_LINE_BYTES)+7)>>3), tmem, rtile, pal, cmt, maskt, shiftt, cms, masks, shifts), gsDPSetTileSize(rtile, 0, 0, ((width)-1) << G_TEXTURE_IMAGE_FRAC, ((height)-1) << G_TEXTURE_IMAGE_FRAC)
#define ERR_OSVIGETNEXTFRAMEBUFFER 38
#define UNK_2C0_DISP __gfxCtx->unk_2C0.p
#define GIO_CART_INTR_REG (GIO_BASE_REG+0x800)
#define G_RM_AA_ZB_OPA_DECAL2 RM_AA_ZB_OPA_DECAL(2)
#define RAMROM_BOOTSTRAP_OFFSET 0x40
#define G_RDP_TRI_ZBUFF_MASK 0x01
#define _OS_PI_H_ 
#define PI_STATUS_REG (PI_BASE_REG + 0x10)
#define ICACHE_SIZE 0x4000
#define OS_VI_NTSC_LPN2 4
#define SI_STATUS_REG (SI_BASE_REG + 0x18)
#define OS_MAJOR_VERSION "2.0K"
#define ERR_ALSEQNOTMIDI0 115
#define _gDPLoadTextureBlock(pkt,timg,tmem,fmt,siz,width,height,pal,cms,cmt,masks,maskt,shifts,shiftt) { gDPSetTextureImage(pkt, fmt, siz ##_LOAD_BLOCK, 1, timg); gDPSetTile(pkt, fmt, siz ##_LOAD_BLOCK, 0, tmem, G_TX_LOADTILE, 0, cmt, maskt, shiftt, cms, masks, shifts); gDPLoadSync(pkt); gDPLoadBlock(pkt, G_TX_LOADTILE, 0, 0, (((width)*(height) + siz ##_INCR) >> siz ##_SHIFT)-1, CALC_DXT(width, siz ##_BYTES)); gDPPipeSync(pkt); gDPSetTile(pkt, fmt, siz, (((width) * siz ##_LINE_BYTES)+7)>>3, tmem, G_TX_RENDERTILE, pal, cmt, maskt, shiftt, cms, masks, shifts); gDPSetTileSize(pkt, G_TX_RENDERTILE, 0, 0, ((width)-1) << G_TEXTURE_IMAGE_FRAC, ((height)-1) << G_TEXTURE_IMAGE_FRAC); }
#define LIGHT_8 8
#define G_TF_AVERAGE (3 << G_MDSFT_TEXTFILT)
#define OS_FLAG_CPU_BREAK 1
#define LIGHT_5 5
#define G_SPNOOP 0xe0
#define G_MDSFT_PIPELINE 23
#define DPC_END_REG (DPC_BASE_REG + 0x04)

#define IS_ZERO(f) (fabsf(f) < 0.008f)

#define LIGHT_6 6
#define gDPLoadTileGeneric(pkt,c,tile,uls,ult,lrs,lrt) _DW({ Gfx *_g = (Gfx *)(pkt); _g->words.w0 = _SHIFTL(c, 24, 8) | _SHIFTL(uls, 12, 12) | _SHIFTL(ult, 0, 12); _g->words.w1 = _SHIFTL(tile, 24, 3) | _SHIFTL(lrs, 12, 12) | _SHIFTL(lrt, 0, 12); })
#define G_SC_ODD_INTERLACE 3
#define GU_PARSEGBI_DUMPONLY 32
#define gSP1Triangle(pkt,v0,v1,v2,flag) _DW({ Gfx *_g = (Gfx *)(pkt); _g->words.w0 = _SHIFTL(G_TRI1, 24, 8)| __gsSP1Triangle_w1f(v0, v1, v2, flag); _g->words.w1 = 0; })
#define UINT32_MAX 0xFFFFFFFF
#define SP_IBIST_DONE (1 << 2)
#define RDRAM_0_START 0x00000000
#define gSPNoOp(pkt) gDma0p(pkt, G_SPNOOP, 0, 0)
#define C0_CONFIG 16
#define gImmp2(pkt,c,p0,p1) { Gfx *_g = (Gfx *)(pkt); _g->words.w0 = _SHIFTL((c), 24, 8); _g->words.w1 = _SHIFTL((p0), 16, 16) | _SHIFTL((p1), 8, 8); }
#define gImmp3(pkt,c,p0,p1,p2) { Gfx *_g = (Gfx *)(pkt); _g->words.w0 = _SHIFTL((c), 24, 8); _g->words.w1 = (_SHIFTL((p0), 16, 16) | _SHIFTL((p1), 8, 8) | _SHIFTL((p2), 0, 8)); }
#define STOP_GAMESTATE(curState) do { Game* state = curState; state->running = false; } while(0)
#define G_AC_NONE (0 << G_MDSFT_ALPHACOMPARE)
#define OS_VI_PAL_HAF1 25
#define OS_VI_MPAL_LPF1 29
#define OS_VI_MPAL_LPF2 33
#define U_CBUTTONS CONT_E
#define G_CCMUX_SHADE 4
#define NUMLIGHTS_4 4
#define gDPSetScissor(pkt,mode,ulx,uly,lrx,lry) _DW({ Gfx *_g = (Gfx *)pkt; _g->words.w0 = _SHIFTL(G_SETSCISSOR, 24, 8) | _SHIFTL((int)((float)(ulx)*4.0F), 12, 12) | _SHIFTL((int)((float)(uly)*4.0F), 0, 12); _g->words.w1 = _SHIFTL(mode, 24, 2) | _SHIFTL((int)((float)(lrx)*4.0F), 12, 12) | _SHIFTL((int)((float)(lry)*4.0F), 0, 12); })
#define G_RM_AA_ZB_OPA_INTER2 RM_AA_ZB_OPA_INTER(2)
#define SP_RD_LEN_REG (SP_BASE_REG + 0x08)
#define CACHERR_SIDX_MASK 0x003ffff8
#define SADDRMASK 0xFFFFE000
#define TEX_EDGE 0x0000
#define G_RDP_TRI_SHADE_MASK 0x04
#define OS_READ 0
#define gsDPSetTile(fmt,siz,line,tmem,tile,palette,cmt,maskt,shiftt,cms,masks,shifts) {{ (_SHIFTL(G_SETTILE, 24, 8) | _SHIFTL(fmt, 21, 3) | _SHIFTL(siz, 19, 2) | _SHIFTL(line, 9, 9) | _SHIFTL(tmem, 0, 9)), (_SHIFTL(tile, 24, 3) | _SHIFTL(palette, 20, 4) | _SHIFTL(cmt, 18, 2) | _SHIFTL(maskt, 14, 4) | _SHIFTL(shiftt, 10, 4) | _SHIFTL(cms, 8, 2) | _SHIFTL(masks, 4, 4) | _SHIFTL(shifts, 0, 4)) }}
#define OS_GBPAK_RSTB_DETECTION 0x04
#define WATCHHI_VALIDMASK 0x0000000f
#define SP_OVERLAP 0x00000040
#define G_CCMUX_NOISE 7
#define A_NOAUX 0x00
#define _OS_TIME_H_ 
#define AI_CONTROL_DMA_ON 1
#define ERR_OSCREATETHREAD_SP 1
#define RI_REFRESH_REG (RI_BASE_REG + 0x10)
#define gsImmp1(c,p0) {{ _SHIFTL((c), 24, 8), (unsigned int)(p0) }}
#define U_JPAD CONT_UP
#define LAND_NAME_MURA_SIZE (LAND_NAME_SIZE + 2)
#define ERR_OSSETEVENTMESG 9
#define G_RM_AA_OPA_SURF2 RM_AA_OPA_SURF(2)
#define SP_CLR_RSPSIGNAL SP_CLR_SIG3
#define gSPSetGeometryMode(pkt,word) gSPGeometryMode((pkt),0,(word))
#define DPC_STATUS_CBUF_READY (1 << 7)
#define RM_OPA_SURF(clk) CVG_DST_CLAMP | FORCE_BL | ZMODE_OPA | GBL_c ##clk(G_BL_CLR_IN, G_BL_0, G_BL_CLR_IN, G_BL_1)
#define G_IM_SIZ_8b_SHIFT 1
#define EXC_DBE EXC_CODE(7)
#define G_CCMUX_TEXEL0 1
#define G_CCMUX_TEXEL1 2
#define A_INTERLEAVE 13
#define G_ACMUX_ENVIRONMENT 5
#define G_MWO_MATRIX_WZ_WW_F 0x3c
#define PROJECTED_TO_SCREEN_X(projectedPos,invW) ((projectedPos).x * (invW) * (SCREEN_WIDTH / 2) + (SCREEN_WIDTH / 2))
#define G_MWO_MATRIX_WZ_WW_I 0x1c
#define G_BL_A_MEM 1
#define G_CCMUX_LOD_FRACTION 13
#define RM_AA_ZB_SUB_TERR(clk) AA_EN | Z_CMP | Z_UPD | IM_RD | CVG_DST_FULL | ZMODE_OPA | ALPHA_CVG_SEL | GBL_c ##clk(G_BL_CLR_IN, G_BL_A_IN, G_BL_CLR_MEM, G_BL_1MA)
#define Z_UPD 0x20
#define AL_CMIDI_LOOPEND_CODE 0x2D
#define G_TRI_TXTR_ZBUFF 0xcb
#define RDRAM_GLOBAL_CONFIG 0x80000
#define SI_DRAM_ADDR_REG (SI_BASE_REG + 0x00)
#define OC1_1 (1 << 0)
#define OC1_2 (1 << 1)
#define OC1_4 (1 << 2)
#define MI_INTR_AI (1 << 2)
#define G_RM_XLU_SURF RM_XLU_SURF(1)
#define gSPScisTextureRectangle(pkt,xl,yl,xh,yh,tile,s,t,dsdx,dtdy) _DW({ Gfx *_g = (Gfx *)(pkt); _g->words.w0 = (_SHIFTL(G_TEXRECT, 24, 8) | _SHIFTL(MAX((s16)(xh),0), 12, 12) | _SHIFTL(MAX((s16)(yh),0), 0, 12)); _g->words.w1 = (_SHIFTL((tile), 24, 3) | _SHIFTL(MAX((s16)(xl),0), 12, 12) | _SHIFTL(MAX((s16)(yl),0), 0, 12)); gImmp1(pkt, G_RDPHALF_1, (_SHIFTL(((s) - (((s16)(xl) < 0) ? (((s16)(dsdx) < 0) ? (MAX((((s16)(xl)*(s16)(dsdx))>>7),0)) : (MIN((((s16)(xl)*(s16)(dsdx))>>7),0))) : 0)), 16, 16) | _SHIFTL(((t) - (((yl) < 0) ? (((s16)(dtdy) < 0) ? (MAX((((s16)(yl)*(s16)(dtdy))>>7),0)) : (MIN((((s16)(yl)*(s16)(dtdy))>>7),0))) : 0)), 0, 16))); gImmp1(pkt, G_RDPHALF_2, (_SHIFTL((dsdx), 16, 16) | _SHIFTL((dtdy), 0, 16))); })
#define G_RM_NOOP RM_NOOP(1)
#define ICACHE_LINEMASK (ICACHE_LINESIZE-1)
#define gDPSetPrimDepth(pkt,z,dz) gDPSetColor(pkt, G_SETPRIMDEPTH, _SHIFTL(z, 16, 16) | _SHIFTL(dz, 0, 16))
#define gSPForceMatrix(pkt,mptr) { gDma2p((pkt),G_MOVEMEM,(mptr),sizeof(Mtx),G_MV_MATRIX,0); gMoveWd((pkt), G_MW_FORCEMTX,0,0x00010000); }
#define RAMROM_GAME_OFFSET 0x1000
#define gsSPLoadGeometryMode(word) gsSPGeometryMode(-1,(word))
#define PFS_ERR_NO_GBCART 12
#define gsSPDma_io(flag,dmem,dram,size) {{ _SHIFTL(G_DMA_IO,24,8)|_SHIFTL((flag),23,1)| _SHIFTL((dmem)/8,13,10)|_SHIFTL((size)-1,0,12), (unsigned int)(dram) }}
#define GU_PARSEGBI_ROWMAJOR 1
#define PFS_ID_1AREA 3
#define ERR_OSFREE_ADDR 54
#define GAME_H 
#define G_CYC_2CYCLE (1 << G_MDSFT_CYCLETYPE)
#define SP_DRAM_STACK_SIZE8 (1024)
#define gsDPLoadTLUT(count,tmemaddr,dram) gsDPSetTextureImage(G_IM_FMT_RGBA, G_IM_SIZ_16b, 1, dram), gsDPTileSync(), gsDPSetTile(0, 0, 0, tmemaddr, G_TX_LOADTILE, 0 , 0, 0, 0, 0, 0, 0), gsDPLoadSync(), gsDPLoadTLUTCmd(G_TX_LOADTILE, ((count)-1)), gsDPPipeSync()
#define C0_MAJREVMASK 0xf0
#define M_PRIVATE_H 
#define G_RM_AA_ZB_OPA_TERR RM_AA_ZB_OPA_TERR(1)
#define OS_EVENT_SW1 0
#define OS_EVENT_SW2 1
#define OS_VI_PAL_HAN1 24
#define OS_VI_MPAL_LPN2 32
#define G_MWO_CLIP_RNX 0x04
#define PFS_PAGE_NOT_USED 3
#define G_SC_EVEN_INTERLACE 2
#define MI_INTR_DP (1 << 5)
#define gSPSetOtherMode(pkt,cmd,sft,len,data) _DW({ Gfx *_g = (Gfx *)(pkt); _g->words.w0 = (_SHIFTL(cmd,24,8)|_SHIFTL(32-(sft)-(len),8,8)| _SHIFTL((len)-1,0,8)); _g->words.w1 = (unsigned int)(data); })
#define G_SETFILLCOLOR 0xf7
#define G_CC_BLENDI2 ENVIRONMENT, SHADE, COMBINED, SHADE, 0, 0, 0, SHADE
#define OS_IM_PRENMI 0x00001401
#define gsDPLoadMultiTile(timg,tmem,rtile,fmt,siz,width,height,uls,ult,lrs,lrt,pal,cms,cmt,masks,maskt,shifts,shiftt) gsDPSetTextureImage(fmt, siz, width, timg), gsDPSetTile(fmt, siz, (((((lrs)-(uls)+1) * siz ##_TILE_BYTES)+7)>>3), tmem, G_TX_LOADTILE, 0 , cmt, maskt, shiftt, cms, masks, shifts), gsDPLoadSync(), gsDPLoadTile( G_TX_LOADTILE, (uls)<<G_TEXTURE_IMAGE_FRAC, (ult)<<G_TEXTURE_IMAGE_FRAC, (lrs)<<G_TEXTURE_IMAGE_FRAC, (lrt)<<G_TEXTURE_IMAGE_FRAC), gsDPPipeSync(), gsDPSetTile(fmt, siz, (((((lrs)-(uls)+1) * siz ##_LINE_BYTES)+7)>>3), tmem, rtile, pal, cmt, maskt, shiftt, cms, masks, shifts), gsDPSetTileSize(rtile, (uls)<<G_TEXTURE_IMAGE_FRAC, (ult)<<G_TEXTURE_IMAGE_FRAC, (lrs)<<G_TEXTURE_IMAGE_FRAC, (lrt)<<G_TEXTURE_IMAGE_FRAC)
#define G_CC_BLENDIA ENVIRONMENT, SHADE, TEXEL0, SHADE, TEXEL0, 0, SHADE, 0
#define ERR_OSSETTIME 75
#define NTLBENTRIES 31
#define OS_VI_BIT_PAL 0x0800
#define CALC_DXT_4b(width) (((1 << G_TX_DXT_FRAC) + TXL2WORDS_4b(width) - 1) / TXL2WORDS_4b(width))
#define SP_IBIST_CHECK (1 << 0)
#define gsMoveWd(index,offset,data) gsDma1p( G_MOVEWORD, data, offset, index)
#define PFS_ERR_DEVICE 11
#define G_RM_AA_PCL_SURF RM_AA_PCL_SURF(1)
#define C_6E9650_H 
#define G_ACMUX_0 7
#define gsDPLoadMultiBlock(timg,tmem,rtile,fmt,siz,width,height,pal,cms,cmt,masks,maskt,shifts,shiftt) gsDPSetTextureImage(fmt, siz ##_LOAD_BLOCK, 1, timg), gsDPSetTile(fmt, siz ##_LOAD_BLOCK, 0, tmem, G_TX_LOADTILE, 0 , cmt, maskt, shiftt, cms, masks, shifts), gsDPLoadSync(), gsDPLoadBlock(G_TX_LOADTILE, 0, 0, (((width)*(height) + siz ##_INCR) >> siz ##_SHIFT)-1, CALC_DXT(width, siz ##_BYTES)), gsDPPipeSync(), gsDPSetTile(fmt, siz, ((((width) * siz ##_LINE_BYTES)+7)>>3), tmem, rtile, pal, cmt, maskt, shiftt, cms, masks, shifts), gsDPSetTileSize(rtile, 0, 0, ((width)-1) << G_TEXTURE_IMAGE_FRAC, ((height)-1) << G_TEXTURE_IMAGE_FRAC)
#define gSPSetLights1(pkt,name) { gSPNumLights(pkt,NUMLIGHTS_1); gSPLight(pkt,&name.l[0],1); gSPLight(pkt,&name.a,2); }
#define gsDPPipeSync() gsDPNoParam(G_RDPPIPESYNC)
#define FILTER_WRAP 0
#define gSPClearGeometryMode(pkt,word) gSPGeometryMode((pkt),(word),0)
#define G_MW_CLIP 0x04
#define C_IINV 0x0
#define G_CYC_FILL (3 << G_MDSFT_CYCLETYPE)
#define SP_DMA_IMEM (1 << 12)
#define G_CC_ADDRGB 1, 0, TEXEL0, SHADE, 0, 0, 0, SHADE
#define M_HOME_H_H 
#define RI_MODE_REG (RI_BASE_REG + 0x00)
#define G_SPECIAL_1 0xd5
#define FLASH_STATUS_WRITE_OK 0
#define G_SPECIAL_3 0xd3
#define CACH_PD 0x1
#define CACH_PI 0x0
#define gSPSetLights6(pkt,name) { gSPNumLights(pkt,NUMLIGHTS_6); gSPLight(pkt,&name.l[0],1); gSPLight(pkt,&name.l[1],2); gSPLight(pkt,&name.l[2],3); gSPLight(pkt,&name.l[3],4); gSPLight(pkt,&name.l[4],5); gSPLight(pkt,&name.l[5],6); gSPLight(pkt,&name.a,7); }
#define gDPLoadTextureBlock(pkt,timg,fmt,siz,width,height,pal,cms,cmt,masks,maskt,shifts,shiftt) _DW({ gDPSetTextureImage(pkt, fmt, siz ##_LOAD_BLOCK, 1, timg); gDPSetTile(pkt, fmt, siz ##_LOAD_BLOCK, 0, 0, G_TX_LOADTILE, 0 , cmt, maskt, shiftt, cms, masks, shifts); gDPLoadSync(pkt); gDPLoadBlock(pkt, G_TX_LOADTILE, 0, 0, (((width)*(height) + siz ##_INCR) >> siz ##_SHIFT) -1, CALC_DXT(width, siz ##_BYTES)); gDPPipeSync(pkt); gDPSetTile(pkt, fmt, siz, (((width) * siz ##_LINE_BYTES)+7)>>3, 0, G_TX_RENDERTILE, pal, cmt, maskt, shiftt, cms, masks, shifts); gDPSetTileSize(pkt, G_TX_RENDERTILE, 0, 0, ((width)-1) << G_TEXTURE_IMAGE_FRAC, ((height)-1) << G_TEXTURE_IMAGE_FRAC); })
#define MI_BASE_REG 0x04300000
#define RM_AA_ZB_XLU_DECAL(clk) AA_EN | Z_CMP | IM_RD | CVG_DST_WRAP | CLR_ON_CVG | FORCE_BL | ZMODE_DEC | GBL_c ##clk(G_BL_CLR_IN, G_BL_A_IN, G_BL_CLR_MEM, G_BL_1MA)
#define TLBHI_PIDSHIFT 0
#define G_SETKEYR 0xeb
#define SP_CLR_BROKE (1 << 2)
#define _OS_GIO_H_ 
#define G_CULL_BOTH 0x00000600
#define ACTOR_FLAG_80000 (1 << 19)
#define CONFIG_BE 0x00008000
#define G_RM_AA_XLU_LINE RM_AA_XLU_LINE(1)
#define gsSPSetLights3(name) gsSPNumLights(NUMLIGHTS_3), gsSPLight(&name.l[0],1), gsSPLight(&name.l[1],2), gsSPLight(&name.l[2],3), gsSPLight(&name.a,4)
#define SR_IMASKSHIFT 8
#define OS_TASK_SP_ONLY 0x0008
#define RI_SELECT_REG (RI_BASE_REG + 0x0C)
#define G_BL_CLR_FOG 3
#define EXC_WADE EXC_CODE(5)
#define FLASH_SIZE 0x20000
#define _OS_DEBUG_H_ 
#define gSPDma_io(pkt,flag,dmem,dram,size) { Gfx *_g = (Gfx *)(pkt); _g->words.w0 = _SHIFTL(G_DMA_IO,24,8)|_SHIFTL((flag),23,1)| _SHIFTL((dmem)/8,13,10)|_SHIFTL((size)-1,0,12); _g->words.w1 = (unsigned int)(dram); }
#define CONFIG_CM 0x80000000
#define RM_RA_ZB_OPA_INTER(clk) AA_EN | Z_CMP | Z_UPD | CVG_DST_CLAMP | ALPHA_CVG_SEL | ZMODE_INTER | GBL_c ##clk(G_BL_CLR_IN, G_BL_A_IN, G_BL_CLR_MEM, G_BL_A_MEM)
#define G_CC_BLENDPEDECALA PRIMITIVE, ENVIRONMENT, TEXEL0, ENVIRONMENT, 0, 0, 0, TEXEL0
#define CONFIG_CU 0x00000008
#define G_CYC_1CYCLE (0 << G_MDSFT_CYCLETYPE)
#define ERR_ALBNKFNEW 113
#define START_BUTTON CONT_START
#define RDRAM_0_BASE_ADDRESS (RDRAM_0_DEVICE_ID * RDRAM_LENGTH)
#define CACH_SD 0x3
#define ERR_ALSNDPPLAY 109
#define CACH_SI 0x2
#define G_IM_SIZ_32b_TILE_BYTES 2
#define SR_IE 0x00000001
#define CONFIG_DB 0x00000010
#define CONFIG_DC 0x000001c0
#define G_CC_PASS2 0, 0, 0, COMBINED, 0, 0, 0, COMBINED
#define PFS_DATA_FULL 7
#define CLOSE_DISPS(gfxCtx) (void)0; } (void)0
#define lbRTC_SECONDS_PER_MINUTE 60
#define SI_PIF_ADDR_RD64B_REG (SI_BASE_REG + 0x04)
#define OS_FLAG_FAULT 2
#define RM_ZB_XLU_DECAL(clk) Z_CMP | IM_RD | CVG_DST_FULL | FORCE_BL | ZMODE_DEC | GBL_c ##clk(G_BL_CLR_IN, G_BL_A_IN, G_BL_CLR_MEM, G_BL_1MA)
#define gDPSetCombineKey(pkt,type) gSPSetOtherMode(pkt, G_SETOTHERMODE_H, G_MDSFT_COMBKEY, 1, type)
#define RMON_STACKSIZE 0x1000
#define OC1_TYPE_20 (1 << 5)
#define CONFIG_EB 0x00002000
#define CONFIG_EC 0x70000000
#define SP_STATUS_TASKDONE SP_STATUS_SIG2
#define G_CC_BLENDPE PRIMITIVE, ENVIRONMENT, TEXEL0, ENVIRONMENT, TEXEL0, 0, SHADE, 0
#define CONFIG_EM 0x00004000
#define G_MWO_MATRIX_YX_YY_I 0x08
#define CONFIG_EW 0x000c0000
#define C0_IMPMASK 0xff00
#define DPRGBColor(pkt,cmd,r,g,b,a) gDPSetColor(pkt, cmd, (_SHIFTL(r, 24, 8) | _SHIFTL(g, 16, 8) | _SHIFTL(b, 8, 8) | _SHIFTL(a, 0, 8)))
#define AL_PLAYING 1
#define FPCSR_CO 0x00004000
#define OS_NUM_EVENTS 15
#define A_MAIN 0x00
#define G_IM_SIZ_8b_INCR 1
#define PFS_ID_0AREA 1
#define G_CC_SHADEDECALA 0, 0, 0, SHADE, 0, 0, 0, TEXEL0
#define G_BL_0 3
#define OS_LOG_VERSION 1
#define _gDPLoadTextureBlock_4b(pkt,timg,tmem,fmt,width,height,pal,cms,cmt,masks,maskt,shifts,shiftt) { gDPSetTextureImage(pkt, fmt, G_IM_SIZ_16b, 1, timg); gDPSetTile(pkt, fmt, G_IM_SIZ_16b, 0, tmem, G_TX_LOADTILE, 0, cmt, maskt, shiftt, cms, masks, shifts); gDPLoadSync(pkt); gDPLoadBlock(pkt, G_TX_LOADTILE, 0, 0, (((width)*(height)+3)>>2)-1, CALC_DXT_4b(width)); gDPPipeSync(pkt); gDPSetTile(pkt, fmt, G_IM_SIZ_4b, ((((width)>>1)+7)>>3), tmem, G_TX_RENDERTILE, pal, cmt, maskt, shiftt, cms, masks, shifts); gDPSetTileSize(pkt, G_TX_RENDERTILE, 0, 0, ((width)-1) << G_TEXTURE_IMAGE_FRAC, ((height)-1) << G_TEXTURE_IMAGE_FRAC); }
#define G_IM_SIZ_4b_BYTES 0
#define gsDPLoadTile(t,uls,ult,lrs,lrt) gsDPLoadTileGeneric(G_LOADTILE, t, uls, ult, lrs, lrt)
#define G_IM_SIZ_8b_LINE_BYTES G_IM_SIZ_8b_BYTES
#define gDPSetFillColor(pkt,d) gDPSetColor(pkt, G_SETFILLCOLOR, (d))
#define _SIZE_T_DEF 
#define G_IM_SIZ_16b_SHIFT 0
#define gDPLoadTextureBlock_4bS(pkt,timg,fmt,width,height,pal,cms,cmt,masks,maskt,shifts,shiftt) { gDPSetTextureImage(pkt, fmt, G_IM_SIZ_16b, 1, timg); gDPSetTile(pkt, fmt, G_IM_SIZ_16b, 0, 0, G_TX_LOADTILE, 0, cmt, maskt, shiftt, cms, masks, shifts); gDPLoadSync(pkt); gDPLoadBlock(pkt, G_TX_LOADTILE, 0, 0, (((width)*(height)+3)>>2)-1, 0 ); gDPPipeSync(pkt); gDPSetTile(pkt, fmt, G_IM_SIZ_4b, ((((width)>>1)+7)>>3), 0, G_TX_RENDERTILE, pal, cmt, maskt, shiftt, cms, masks, shifts); gDPSetTileSize(pkt, G_TX_RENDERTILE, 0, 0, ((width)-1) << G_TEXTURE_IMAGE_FRAC, ((height)-1) << G_TEXTURE_IMAGE_FRAC); }
#define gsSPDisplayList(dl) gsDma1p( G_DL,dl,0,G_DL_PUSH)
#define RDRAM_0_CONFIG 0x00000
#define PFS_ERR_ID_FATAL 10
#define EEPROM_TYPE_16K 0x02
#define G_MVO_LOOKATX (0*24)
#define RAMROM_RMON_READ_ADDR (RAMROM_MSG_ADDR + (2*RAMROM_BUF_SIZE))
#define alCSPGetProgram alCSPGetChlProgram
#define gDPParam(pkt,cmd,param) { Gfx *_g = (Gfx *)(pkt); _g->words.w0 = _SHIFTL(cmd, 24, 8); _g->words.w1 = (param); }
#define gsSPSetLights1(name) gsSPNumLights(NUMLIGHTS_1), gsSPLight(&name.l[0],1), gsSPLight(&name.a,2)
#define ERR_OSPIREADIO 26
#define G_FILLRECT 0xf6
#define FR_NEG_FRUSTRATIO_1 0x00000001
#define FR_NEG_FRUSTRATIO_2 0x00000002
#define FR_NEG_FRUSTRATIO_3 0x00000003
#define FR_NEG_FRUSTRATIO_4 0x00000004
#define FR_NEG_FRUSTRATIO_5 0x00000005
#define FR_NEG_FRUSTRATIO_6 0x00000006
#define G_ROTATE_FRAC 16
#define RM_RA_OPA_SURF(clk) AA_EN | CVG_DST_CLAMP | ZMODE_OPA | ALPHA_CVG_SEL | GBL_c ##clk(G_BL_CLR_IN, G_BL_A_IN, G_BL_CLR_MEM, G_BL_A_MEM)
#define alHeapAlloc(hp,elem,size) alHeapDBAlloc(0, 0,(hp),(elem),(size))
#define PI_STATUS_RESET (1 << 0)
#define gDPNoParam(pkt,cmd) _DW({ Gfx *_g = (Gfx *)(pkt); _g->words.w0 = _SHIFTL(cmd, 24, 8); _g->words.w1 = 0; })
#define G_CC_HILITERGBA PRIMITIVE, SHADE, TEXEL0, SHADE, PRIMITIVE, SHADE, TEXEL0, SHADE
#define gsSPSetLights5(name) gsSPNumLights(NUMLIGHTS_5), gsSPLight(&name.l[0],1), gsSPLight(&name.l[1],2), gsSPLight(&name.l[2],3), gsSPLight(&name.l[3],4), gsSPLight(&name.l[4],5), gsSPLight(&name.a,6)
#define RI_WERROR_REG (RI_BASE_REG + 0x1C)
#define ERR_ALEVENTNOFREE 124
#define CONFIG_IB 0x00000020
#define CONFIG_IC 0x00000e00
#define G_CD_ENABLE G_CD_NOISE
#define ERR_ALSEQP_OFF_VOICE 103
#define PI_BSD_DOM2_PGS_REG (PI_BASE_REG + 0x2C)
#define G_DMACMDSIZ 128
#define GRAPH_ALLOC(gfxCtx,size) ((void*)((gfxCtx)->polyOpa.d = (Gfx*)((u8*)(gfxCtx)->polyOpa.d - ALIGN16(size))))
#define GRAPH_ALLOC_NO_ALIGN(gfxCtx,size) ((void*)((gfxCtx)->polyOpa.d = (Gfx*)((u8*)(gfxCtx)->polyOpa.d - size)))
#define _RMON_H_ 
#define DPC_START_REG (DPC_BASE_REG + 0x00)
#define G_TC_CONV (0 << G_MDSFT_TEXTCONV)
#define CONT_EEPROM 0x8000
#define OS_VI_NTSC_LPN1 0
#define gDPFillRectangle(pkt,ulx,uly,lrx,lry) _DW({ Gfx *_g = (Gfx *)(pkt); _g->words.w0 = (_SHIFTL(G_FILLRECT, 24, 8) | _SHIFTL((lrx), 14, 10) | _SHIFTL((lry), 2, 10)); _g->words.w1 = (_SHIFTL((ulx), 14, 10) | _SHIFTL((uly), 2, 10));})
#define C0_CONTEXT 4
#define CONT_D 0x0004
#define CONT_E 0x0008
#define CONT_F 0x0001
#define CONT_G 0x2000
#define G_CC_HILITERGBDECALA PRIMITIVE, SHADE, TEXEL0, SHADE, 0, 0, 0, TEXEL0
#define CONT_L 0x0020
#define CONT_R 0x0010
#define gDPSetColorImage(pkt,f,s,w,i) gSetImage(pkt, G_SETCIMG, f, s, w, i)
#define M_HVQTASK 6
#define gSPPopMatrixN(pkt,n,num) gDma2p((pkt),G_POPMTX,(num)*64,64,2,0)
#define gsDPSetDepthSource(src) gsSPSetOtherMode(G_SETOTHERMODE_L, G_MDSFT_ZSRCSEL, 1, src)
#define gsDPSetCombineKey(type) gsSPSetOtherMode(G_SETOTHERMODE_H, G_MDSFT_COMBKEY, 1, type)
#define gsDma1p(c,s,l,p) {{ (_SHIFTL((c), 24, 8) | _SHIFTL((p), 16, 8) | _SHIFTL((l), 0, 16)), (unsigned int)(s) }}
#define CONFIG_K0 0x00000007
#define G_TRI_FILL 0xc8
#define EXC_WMISS EXC_CODE(3)
#define G_MDSFT_RGBDITHER 6
#define FR_POS_FRUSTRATIO_2 0x0000fffe
#define FR_POS_FRUSTRATIO_3 0x0000fffd
#define FR_POS_FRUSTRATIO_4 0x0000fffc
#define FR_POS_FRUSTRATIO_6 0x0000fffa
#define _OS_TLB_H_ 
#define OS_GBPAK_POWER 0x01
#define G_BL_A_IN 0
#define gsDPLoadTLUT_pal256(dram) gsDPSetTextureImage(G_IM_FMT_RGBA, G_IM_SIZ_16b, 1, dram), gsDPTileSync(), gsDPSetTile(0, 0, 0, 256, G_TX_LOADTILE, 0 , 0, 0, 0, 0, 0, 0), gsDPLoadSync(), gsDPLoadTLUTCmd(G_TX_LOADTILE, 255), gsDPPipeSync()
#define AL_SUSTAIN 63
#define ERR_ALSEQTRACKHDR 118
#define Z64MATH_H 
#define G_ACMUX_PRIMITIVE 3
#define gsDPLoadBlock(tile,uls,ult,lrs,dxt) {{ (_SHIFTL(G_LOADBLOCK, 24, 8) | _SHIFTL(uls, 12, 12) | _SHIFTL(ult, 0, 12)), (_SHIFTL(tile, 24, 3) | _SHIFTL((MIN(lrs,G_TX_LDBLK_MAX_TXL)), 12, 12) | _SHIFTL(dxt, 0, 12)) }}
#define MI_INTR_SI (1 << 1)
#define G_TD_SHARPEN (1 << G_MDSFT_TEXTDETAIL)
#define MI_INTR_SP (1 << 0)
#define G_RM_ADD RM_ADD(1)
#define DPS_TBIST_GO (1 << 1)
#define G_CCMUX_SHADE_ALPHA 11
#define RM_AA_ZB_SUB_SURF(clk) AA_EN | Z_CMP | Z_UPD | IM_RD | CVG_DST_FULL | ZMODE_OPA | ALPHA_CVG_SEL | GBL_c ##clk(G_BL_CLR_IN, G_BL_A_IN, G_BL_CLR_MEM, G_BL_A_MEM)
#define G_CC_MODULATERGBDECALA G_CC_MODULATEIDECALA
#define RAMROM_APP_WRITE_ADDR (RAMROM_MSG_ADDR + (1*RAMROM_BUF_SIZE))
#define gsDPSetBlendMask(mask) gsDPNoOp()
#define aResample(pkt,f,p,s) { Acmd *_a = (Acmd *)pkt; _a->words.w0 = (_SHIFTL(A_RESAMPLE, 24, 8) | _SHIFTL(f, 16, 8) | _SHIFTL(p, 0, 16)); _a->words.w1 = (unsigned int)(s); }
#define gDPSetKeyR(pkt,cR,sR,wR) { Gfx *_g = (Gfx *)(pkt); _g->words.w0 = _SHIFTL(G_SETKEYR, 24, 8); _g->words.w1 = (_SHIFTL(wR, 16, 12) | _SHIFTL(cR, 8, 8) | _SHIFTL(sR, 0, 8)); }
#define VI_V_CURRENT_LINE_REG VI_CURRENT_REG
#define BINANG_TO_RAD_ALT2(binang) (((f32)(binang) * (f32)M_PI) / 0x8000)
#define gsSPSetLights0(name) gsSPNumLights(NUMLIGHTS_0), gsSPLight(&name.l[0],1), gsSPLight(&name.a,2)
#define gsSPSetLights2(name) gsSPNumLights(NUMLIGHTS_2), gsSPLight(&name.l[0],1), gsSPLight(&name.l[1],2), gsSPLight(&name.a,3)
#define gsSPSetLights4(name) gsSPNumLights(NUMLIGHTS_4), gsSPLight(&name.l[0],1), gsSPLight(&name.l[1],2), gsSPLight(&name.l[2],3), gsSPLight(&name.l[3],4), gsSPLight(&name.a,5)
#define G_CCMUX_TEXEL1_ALPHA 9
#define gsSPSetLights6(name) gsSPNumLights(NUMLIGHTS_6), gsSPLight(&name.l[0],1), gsSPLight(&name.l[1],2), gsSPLight(&name.l[2],3), gsSPLight(&name.l[3],4), gsSPLight(&name.l[4],5), gsSPLight(&name.l[5],6), gsSPLight(&name.a,7)
#define gsSPSetLights7(name) gsSPNumLights(NUMLIGHTS_7), gsSPLight(&name.l[0],1), gsSPLight(&name.l[1],2), gsSPLight(&name.l[2],3), gsSPLight(&name.l[3],4), gsSPLight(&name.l[4],5), gsSPLight(&name.l[5],6), gsSPLight(&name.l[6],7), gsSPLight(&name.a,8)
#define G_PM_NPRIMITIVE (0 << G_MDSFT_PIPELINE)
#define gsDPSetAlphaCompare(type) gsSPSetOtherMode(G_SETOTHERMODE_L, G_MDSFT_ALPHACOMPARE, 2, type)
#define G_RM_AA_ZB_TEX_INTER RM_AA_ZB_TEX_INTER(1)
#define RDRAM_MODE_REG (RDRAM_BASE_REG + 0x0c)
#define G_RM_AA_ZB_OPA_INTER RM_AA_ZB_OPA_INTER(1)
#define SP_SET_TASKDONE SP_SET_SIG2
#define AI_NTSC_MIN_FREQ 3000
#define gSetImage(pkt,cmd,fmt,siz,width,i) _DW({ Gfx *_g = (Gfx *)(pkt); _g->words.w0 = _SHIFTL(cmd, 24, 8) | _SHIFTL(fmt, 21, 3) | _SHIFTL(siz, 19, 2) | _SHIFTL((width)-1, 0, 12); _g->words.w1 = (unsigned int)(i); })
#define G_RM_AA_ZB_XLU_DECAL2 RM_AA_ZB_XLU_DECAL(2)
#define G_CC_MODULATERGBA_PRIM2 G_CC_MODULATEIA_PRIM2
#define OS_WRITE 1
#define RDRAM_1_BASE_ADDRESS (RDRAM_1_DEVICE_ID * RDRAM_LENGTH)
#define OS_VI_FPAL_LAN1 44
#define s82 s16
#define s84 s32
#define MI_INTR_VI (1 << 3)
#define s88 s64
#define SET_NEXT_GAMESTATE(curState,nextInit,nextSize) do { Game* state = curState; (state)->init = nextInit; (state)->size = nextSize; } while (0)
#define OS_VI_NTSC_LAN2 6
#define OS_VI_PAL_HPF2 27
#define G_IM_SIZ_16b_LINE_BYTES G_IM_SIZ_16b_BYTES
#define DPS_TBIST_CHECK (1 << 0)
#define ERR_OSPIRAWSTARTDMA_DEVADDR 22
#define C_HWBINV 0x14
#define DPC_TMEM_REG (DPC_BASE_REG + 0x1C)
#define ERR_OSDPSETNEXTBUFFER_ADDR 17
#define gsSPDmaRead(dmem,dram,size) gsSPDma_io(0,(dmem),(dram),(size))
#define gsDPLoadTextureBlockS(timg,fmt,siz,width,height,pal,cms,cmt,masks,maskt,shifts,shiftt) gsDPSetTextureImage(fmt, siz ##_LOAD_BLOCK, 1, timg), gsDPSetTile(fmt, siz ##_LOAD_BLOCK, 0, 0, G_TX_LOADTILE, 0 , cmt, maskt,shiftt, cms, masks, shifts), gsDPLoadSync(), gsDPLoadBlock(G_TX_LOADTILE, 0, 0, (((width)*(height) + siz ##_INCR) >> siz ##_SHIFT)-1, 0 ), gsDPPipeSync(), gsDPSetTile(fmt, siz, ((((width) * siz ##_LINE_BYTES)+7)>>3), 0, G_TX_RENDERTILE, pal, cmt, maskt, shiftt, cms, masks, shifts), gsDPSetTileSize(G_TX_RENDERTILE, 0, 0, ((width)-1) << G_TEXTURE_IMAGE_FRAC, ((height)-1) << G_TEXTURE_IMAGE_FRAC)
#define SP_STATUS_SSTEP (1 << 5)
#define FALLTHROUGH __attribute__((fallthrough))
#define _OS_ERROR_H_ 
#define G_MV_MATRIX 14
#define G_CLIPPING 0x00800000
#define CVG_X_ALPHA 0x1000
#define G_PM_1PRIMITIVE (1 << G_MDSFT_PIPELINE)
#define SP_DRAM_STACK_SIZE64 (SP_DRAM_STACK_SIZE8 >> 3)
#define _OS_VERSION_H_ 
#define A_VOL 0x04
#define CAUSE_BD 0x80000000
#define AL_FX_CUSTOM 6
#define OS_LOG_MAGIC 0x20736a73
#define AL_CMIDI_CNTRL_LOOPCOUNT_SM 104
#define VI_LEAP_REG (VI_BASE_REG + 0x20)
#define gsSPPerspNormalize(s) gsMoveWd( G_MW_PERSPNORM, 0, (s))
#define OPEN_DISPS(gfxCtx) { GraphicsContext* __gfxCtx = gfxCtx; s32 __dispPad UNUSED
#define CONFIG_SB 0x00c00000
#define CONFIG_SC 0x00020000
#define gDPSetTextureLOD(pkt,type) gSPSetOtherMode(pkt, G_SETOTHERMODE_H, G_MDSFT_TEXTLOD, 1, type)
#define CONFIG_SM 0x00010000
#define G_RM_AA_ZB_XLU_INTER2 RM_AA_ZB_XLU_INTER(2)
#define BINANG_ADD(a,b) ((s16)(a + b))
#define CONFIG_SW 0x00100000
#define GCCc0w0(saRGB0,mRGB0,saA0,mA0) (_SHIFTL((saRGB0), 20, 4) | _SHIFTL((mRGB0), 15, 5) | _SHIFTL((saA0), 12, 3) | _SHIFTL((mA0), 9, 3))
#define GCCc0w1(sbRGB0,aRGB0,sbA0,aA0) (_SHIFTL((sbRGB0), 28, 4) | _SHIFTL((aRGB0), 15, 3) | _SHIFTL((sbA0), 12, 3) | _SHIFTL((aA0), 9, 3))
#define BLOCK_Z_NUM 10
#define TRUNCF_BINANG(f) (s16)(s32)(f)
#define ERR_OSAISETNEXTBUFFER_ENDADDR 132
#define ADPCMFSIZE 16
#define mPr_DELIVERY_QUEST_NUM mPr_POCKETS_SLOT_COUNT
#define RM_AA_SUB_TERR(clk) AA_EN | IM_RD | CVG_DST_FULL | ZMODE_OPA | ALPHA_CVG_SEL | GBL_c ##clk(G_BL_CLR_IN, G_BL_A_IN, G_BL_CLR_MEM, G_BL_1MA)
#define gImmp21(pkt,c,p0,p1,dat) _DW({ Gfx *_g = (Gfx *)(pkt); _g->words.w0 = (_SHIFTL((c), 24, 8) | _SHIFTL((p0), 8, 16) | _SHIFTL((p1), 0, 8)); _g->words.w1 = (unsigned int) (dat); })
#define RM_OPA_CI(clk) CVG_DST_CLAMP | ZMODE_OPA | GBL_c ##clk(G_BL_CLR_IN, G_BL_0, G_BL_CLR_IN, G_BL_1)
#define PIF_ROM_END 0x1FC007BF
#define gsSPTextureRectangleFlip(xl,yl,xh,yh,tile,s,t,dsdx,dtdy) {{(_SHIFTL(G_TEXRECTFLIP, 24, 8) | _SHIFTL(xh, 12, 12) | _SHIFTL(yh, 0, 12)), (_SHIFTL(tile, 24, 3) | _SHIFTL(xl, 12, 12) | _SHIFTL(yl, 0, 12))}}, gsImmp1(G_RDPHALF_1, (_SHIFTL(s, 16, 16) | _SHIFTL(t, 0, 16))), gsImmp1(G_RDPHALF_2, (_SHIFTL(dsdx, 16, 16) | _SHIFTL(dtdy, 0, 16)))
#define OS_MESG_BLOCK 1
#define CONFIG_EC_1_1 0x6
#define DL_SPRITE_OVERHEAD (24)
#define ACTOR_FLAG_100 (1 << 8)
#define gsSPLightColor(n,col) gsMoveWd(G_MW_LIGHTCOL, G_MWO_a ##n, col), gsMoveWd(G_MW_LIGHTCOL, G_MWO_b ##n, col)
#define ARRAY_COUNTU(arr) (u32)(sizeof(arr) / sizeof(arr[0]))
#define ERR_OSPIWRITEIO 27
#define AI_BITRATE_REG (AI_BASE_REG + 0x14)
#define ERR_OSAISETNEXTBUFFER_SIZE 16
#define PI_DRAM_ADDR_REG (PI_BASE_REG + 0x00)
#define SP_CLR_TASKDONE SP_CLR_SIG2
#define SP_DMA_BUSY_REG (SP_BASE_REG + 0x18)
#define G_RM_AA_ZB_SUB_TERR2 RM_AA_ZB_SUB_TERR(2)
#define A_LOADBUFF 4
#define _SP_H_ 
#define G_CC_HILITERGBPASSA2 ENVIRONMENT, COMBINED, TEXEL0, COMBINED, 0, 0, 0, COMBINED
#define C0_BADVADDR 8
#define UINT64_MAX 0xFFFFFFFFFFFFFFFF
#define qs1616(e) ((s32)((e)*0x00010000))
#define gsSPClipRatio(r) gsMoveWd(G_MW_CLIP, G_MWO_CLIP_RNX, FR_NEG_ ##r), gsMoveWd(G_MW_CLIP, G_MWO_CLIP_RNY, FR_NEG_ ##r), gsMoveWd(G_MW_CLIP, G_MWO_CLIP_RPX, FR_POS_ ##r), gsMoveWd(G_MW_CLIP, G_MWO_CLIP_RPY, FR_POS_ ##r)
#define OS_VI_PAL_HPN1 22
#define OS_VI_PAL_HPN2 26
#define G_RM_FOG_PRIM_A GBL_c1(G_BL_CLR_FOG, G_BL_A_FOG, G_BL_CLR_IN, G_BL_1MA)
#define ERR_ALSEQP_NO_VOICE 101
#define G_MDSFT_COMBKEY 8
#define OS_LOG_MAX_ARGS 16
#define G_TEXTURE_ENABLE 0x00000000
#define GU_PARSE_MEM_BLOCK 4
#define gsDPSetTextureLOD(type) gsSPSetOtherMode(G_SETOTHERMODE_H, G_MDSFT_TEXTLOD, 1, type)
#define UNK_RET s32
#define G_RM_AA_ZB_SUB_SURF RM_AA_ZB_SUB_SURF(1)
#define RDRAM_DEVICE_TYPE_REG (RDRAM_BASE_REG + 0x00)
#define OS_MESG_NOBLOCK 0
#define gSPSprite2DDraw(pkt,px,py) { Gfx *_g = (Gfx *)(pkt); _g->words.w0 = (_SHIFTL(G_SPRITE2D_DRAW, 24, 8)); _g->words.w1 = (_SHIFTL((px), 16, 16) | _SHIFTL((py), 0, 16)); }
#define alCSPSetFXMix alCSPSetChlFXMix
#define gDPLoadTLUTCmd(pkt,tile,count) _DW({ Gfx *_g = (Gfx *)pkt; _g->words.w0 = _SHIFTL(G_LOADTLUT, 24, 8); _g->words.w1 = _SHIFTL((tile), 24, 3) | _SHIFTL((count), 14, 10);})
#define SECC_MASK 0x0000007f
#define SIZE_EXCVEC 0x80
#define GIMMCMD(x) (G_IMMFIRST-(x))
#define SP_DRAM_ADDR_REG (SP_BASE_REG + 0x04)
#define G_CC_BLENDIDECALA ENVIRONMENT, SHADE, TEXEL0, SHADE, 0, 0, 0, TEXEL0
#define C_6DB420_H 
#define C_HSV 0x1c
#define SP_SET_YIELDED SP_SET_SIG1
#define RM_VISCVG(clk) IM_RD | FORCE_BL | GBL_c ##clk(G_BL_CLR_IN, G_BL_0, G_BL_CLR_BL, G_BL_A_MEM)
#define MAX_RATIO 1.99996
#define G_MWO_aLIGHT_1 0x00
#define G_MWO_aLIGHT_2 0x18
#define G_MWO_aLIGHT_3 0x30
#define G_MWO_aLIGHT_4 0x48
#define G_MWO_aLIGHT_5 0x60
#define G_MWO_aLIGHT_6 0x78
#define G_MWO_aLIGHT_8 0xa8
#define gSPSetLights2(pkt,name) { gSPNumLights(pkt,NUMLIGHTS_2); gSPLight(pkt,&name.l[0],1); gSPLight(pkt,&name.l[1],2); gSPLight(pkt,&name.a,3); }
#define gSPSetLights3(pkt,name) { gSPNumLights(pkt,NUMLIGHTS_3); gSPLight(pkt,&name.l[0],1); gSPLight(pkt,&name.l[1],2); gSPLight(pkt,&name.l[2],3); gSPLight(pkt,&name.a,4); }
#define gSPSetLights4(pkt,name) { gSPNumLights(pkt,NUMLIGHTS_4); gSPLight(pkt,&name.l[0],1); gSPLight(pkt,&name.l[1],2); gSPLight(pkt,&name.l[2],3); gSPLight(pkt,&name.l[3],4); gSPLight(pkt,&name.a,5); }
#define gSPSetLights5(pkt,name) { gSPNumLights(pkt,NUMLIGHTS_5); gSPLight(pkt,&name.l[0],1); gSPLight(pkt,&name.l[1],2); gSPLight(pkt,&name.l[2],3); gSPLight(pkt,&name.l[3],4); gSPLight(pkt,&name.l[4],5); gSPLight(pkt,&name.a,6); }
#define gSPSetLights7(pkt,name) { gSPNumLights(pkt,NUMLIGHTS_7); gSPLight(pkt,&name.l[0],1); gSPLight(pkt,&name.l[1],2); gSPLight(pkt,&name.l[2],3); gSPLight(pkt,&name.l[3],4); gSPLight(pkt,&name.l[4],5); gSPLight(pkt,&name.l[5],6); gSPLight(pkt,&name.l[6],7); gSPLight(pkt,&name.a,8); }
#define CONFIG_UNCACHED 0x00000002
#define AI_NTSC_MAX_FREQ 368000
#define RGBA16_GET_B(pixel) (((pixel) >> 1) & 0x1F)
#define ACTOR_FLAG_200 (1 << 9)
#define VI_CTRL_TYPE_16 0x00002
#define gsSPLine3D(v0,v1,flag) {{ _SHIFTL(G_LINE3D, 24, 8)|__gsSPLine3D_w1f(v0, v1, 0, flag), 0 }}
#define MI_SET_INIT (1 << 8)
#define PINVALID 0x0000
#define gsSPLoadUcodeL(ucode) gsSPLoadUcode(OS_K0_TO_PHYSICAL(& ##ucode ##TextStart), OS_K0_TO_PHYSICAL(& ##ucode ##DataStart))
#define PI_STATUS_ERROR (1 << 2)
#define _ABI_H_ 
#define gsSPVertex(v,n,v0) {{ (_SHIFTL(G_VTX,24,8)|_SHIFTL((n),12,8)|_SHIFTL((v0)+(n),1,7)), (unsigned int)(v) }}
#define G_CD_BAYER (1 << G_MDSFT_RGBDITHER)
#define PFS_MOTOR_INITIALIZED 0x8
#define SP_CUTOUT 0x00000002
#define G_CD_MAGICSQ (0 << G_MDSFT_RGBDITHER)
#define G_CC_MODULATEIA_PRIM2 COMBINED, 0, PRIMITIVE, 0, COMBINED, 0, PRIMITIVE, 0
#define HANIWA_MESSAGE_LEN 64
#define C_HWB 0x18
#define PROF_MIN_INTERVAL 50
#define G_MWO_SEGMENT_1 0x01
#define G_MWO_SEGMENT_2 0x02
#define G_MWO_SEGMENT_3 0x03
#define G_MWO_SEGMENT_5 0x05
#define G_LOD 0x00100000
#define G_MWO_SEGMENT_7 0x07
#define G_MWO_SEGMENT_8 0x08
#define G_MWO_SEGMENT_9 0x09
#define G_MWO_SEGMENT_A 0x0a
#define G_MWO_SEGMENT_B 0x0b
#define G_MWO_SEGMENT_C 0x0c
#define G_MWO_SEGMENT_D 0x0d
#define G_MWO_SEGMENT_E 0x0e
#define G_MWO_SEGMENT_F 0x0f
#define CACHERR_PIDX_SHIFT 12
#define gsDPSetTextureLUT(type) gsSPSetOtherMode(G_SETOTHERMODE_H, G_MDSFT_TEXTLUT, 2, type)
#define RGBA16_GET_G(pixel) (((pixel) >> 6) & 0x1F)
#define G_MWO_MATRIX_XX_XY_F 0x20
#define SYS_MATH_3D_H 
#define LIGHT_DISP __gfxCtx->light.p
#define ERR_OSPIRAWSTARTDMA_DIR 21
#define OS_PM_16K 0x0006000
#define OS_PM_16M 0x1ffe000
#define M_QUEST_H 
#define gsSPDmaWrite(dmem,dram,size) gsSPDma_io(1,(dmem),(dram),(size))
#define LAND_NAME_SIZE 6
#define ERR_OSSPTASKLOAD_YIELD 60
#define OS_TV_NTSC 1
#define gSPSprite2DScaleFlip(pkt,sx,sy,fx,fy) { Gfx *_g = (Gfx *)(pkt); _g->words.w0 = (_SHIFTL(G_SPRITE2D_SCALEFLIP, 24, 8) | _SHIFTL((fx), 8, 8) | _SHIFTL((fy), 0, 8)); _g->words.w1 = (_SHIFTL((sx), 16, 16) | _SHIFTL((sy), 0, 16)); }
#define SP_CLR_INTR_BREAK (1 << 7)
#define A_SETBUFF 8
#define UNK_SIZE 1
#define INTPTR_MAX 0x7FFFFFFF
#define DPS_TBIST_DONE (1 << 2)
#define SP_STATUS_YIELD SP_STATUS_SIG0
#define OS_MESG_TYPE_EDMAWRITE (OS_MESG_TYPE_BASE + 6)
#define DPC_STATUS_END_VALID (1 << 9)
#define MI_CLR_DP_INTR (1 << 11)
#define ERR_ALMODDELAYOVERFLOW 133
#define _OS_LIBC_H_ 
#define RM_RA_ZB_OPA_SURF(clk) AA_EN | Z_CMP | Z_UPD | CVG_DST_CLAMP | ZMODE_OPA | ALPHA_CVG_SEL | GBL_c ##clk(G_BL_CLR_IN, G_BL_A_IN, G_BL_CLR_MEM, G_BL_A_MEM)
#define G_TEXTURE_GEN_LINEAR 0x00080000
#define A_AUX 0x08
#define OC1_TYPE_8 (1 << 3)
#define mPr_FOREIGN_MAP_COUNT 8
#define gsSPModifyVertex(vtx,where,val) {{ _SHIFTL(G_MODIFYVTX,24,8)| _SHIFTL((where),16,8)|_SHIFTL((vtx)*2,0,16), (unsigned int)(val) }}
#define A_BUTTON CONT_A
#define OS_VI_DITHER_FILTER_ON 0x0040
#define G_SPECIAL_2 0xd4
#define CONT_ERR_NO_CONTROLLER PFS_ERR_NOPACK
#define DEVICE_TYPE_CART 0
#define __gsSPLine3D_w1(v0,v1,wd) (_SHIFTL((v0)*2,16,8)|_SHIFT((v1)*2,8,8)|_SHIFT((wd),0,8))
#define gsSPFogFactor(fm,fo) gsMoveWd(G_MW_FOG, G_MWO_FOG, (_SHIFTL(fm,16,16) | _SHIFTL(fo,0,16)))
#define ERR_OSDPSETNEXTBUFFER_SIZE 18
#define EXC_TRAP EXC_CODE(13)
#define gSPBranchLessZ(pkt,dl,vtx,zval,near,far,flag) gSPBranchLessZrg(pkt, dl, vtx, zval, near, far, flag, 0, G_MAXZ)
#define ERR_OSVISETYSCALE_VALUE 41
#define DPC_PIPEBUSY_REG (DPC_BASE_REG + 0x18)
#define CC_CHECK 1-DNON_MATCHING
#define RAMROM_CLOCKRATE_OFFSET 0x4
#define SP_CLR_YIELDED SP_CLR_SIG1
#define gsSPSprite2DBase(s) gsDma1p(G_SPRITE2D_BASE, s, sizeof(uSprite), 0)
#define RDRAM_STANDBY_MODE 2
#define CONTROLLER2(game) (&(game)->input[1])
#define CAUSE_IP3 0x00000400
#define CAUSE_IP4 0x00000800
#define CAUSE_IP5 0x00001000
#define CAUSE_IP6 0x00002000
#define CAUSE_IP7 0x00004000
#define CAUSE_IP8 0x00008000
#define VOICE_WARN_TOO_SMALL 0x0400
#define PFS_WRITE 1
#define G_CC_MODULATERGBA_PRIM G_CC_MODULATEIA_PRIM
#define SR_IBIT1 0x00000100
#define SR_IBIT3 0x00000400
#define SR_IBIT4 0x00000800
#define SR_IBIT5 0x00001000
#define SR_IBIT6 0x00002000
#define SR_IBIT7 0x00004000
#define SR_IBIT8 0x00008000
#define G_RM_PASS GBL_c1(G_BL_CLR_IN, G_BL_0, G_BL_CLR_IN, G_BL_1)
#define GU_PARSERDP_PRAREA 2
#define ERR_ALSEQNOTMIDI 114
#define G_CC_ADDRGBDECALA 1, 0, TEXEL0, SHADE, 0, 0, 0, TEXEL0
#define ERR_OSGETREGIONBUFCOUNT 55
#define G_CC_MODULATEI_PRIM2 COMBINED, 0, PRIMITIVE, 0, 0, 0, 0, PRIMITIVE
#define AL_STOPPING 2
#define gsDPFillRectangle(ulx,uly,lrx,lry) {{ (_SHIFTL(G_FILLRECT, 24, 8) | _SHIFTL((lrx), 14, 10) | _SHIFTL((lry), 2, 10)), (_SHIFTL((ulx), 14, 10) | _SHIFTL((uly), 2, 10)) }}
#define DPS_TBIST_FAILED 0x7F8
#define G_SETOTHERMODE_L 0xe2
#define G_MDSFT_TEXTPERSP 19
#define POLY_OPA_DISP __gfxCtx->polyOpa.p
#define TLBCTXT_BASEMASK 0xff800000
#define gsDPPipelineMode(mode) gsSPSetOtherMode(G_SETOTHERMODE_H, G_MDSFT_PIPELINE, 1, mode)
#define C0_ECC 26
#define TXL2WORDS_4b(txls) MAX(1, ((txls)/16))
#define gsSPCullDisplayList(vstart,vend) {{ _SHIFTL(G_CULLDL, 24, 8) | _SHIFTL((vstart)*2, 0, 16), _SHIFTL((vend)*2, 0, 16) }}
#define K1_TO_PHYS(x) ((u32)(x)&0x1FFFFFFF)
#define A_INIT 0x01
#define RAMROM_USER_DATA_SIZE (RAMROM_MSG_SIZE-RAMROM_MSG_HDR_SIZE)
#define OS_STATE_WAITING (1 << 3)
#define VI_STATUS_REG VI_CONTROL_REG
#define __gsSP1Quadrangle_w1f(v0,v1,v2,v3,flag) (((flag) == 0) ? __gsSP1Triangle_w1(v0, v1, v2): ((flag) == 1) ? __gsSP1Triangle_w1(v1, v2, v3): ((flag) == 2) ? __gsSP1Triangle_w1(v2, v3, v0): __gsSP1Triangle_w1(v3, v0, v1))
#define R_JPAD CONT_RIGHT
#define PFS_ERR_NEW_PACK 2
#define aSetLoop(pkt,a) { Acmd *_a = (Acmd *)pkt; _a->words.w0 = _SHIFTL(A_SETLOOP, 24, 8); _a->words.w1 = (unsigned int)(a); }
#define G_IM_FMT_I 4
#define gsDPNoParam(cmd) {{ _SHIFTL(cmd, 24, 8), 0 }}
#define HOME_MAILBOX_SIZE 10
#define A_SEGMENT 7
#define G_RM_PCL_SURF RM_PCL_SURF(1)
#define G_ACMUX_LOD_FRACTION 0
#define gdSPDefLights0(ar,ag,ab) { {{ {ar,ag,ab},0,{ar,ag,ab},0}}, {{{ { 0, 0, 0},0,{ 0, 0, 0},0,{ 0, 0, 0},0}}} }
#define gdSPDefLights1(ar,ag,ab,r1,g1,b1,x1,y1,z1) { {{ {ar,ag,ab},0,{ar,ag,ab},0}}, {{{ {r1,g1,b1},0,{r1,g1,b1},0,{x1,y1,z1},0}}} }
#define gdSPDefLights2(ar,ag,ab,r1,g1,b1,x1,y1,z1,r2,g2,b2,x2,y2,z2) { {{ {ar,ag,ab},0,{ar,ag,ab},0}}, {{{ {r1,g1,b1},0,{r1,g1,b1},0,{x1,y1,z1},0}}, {{ {r2,g2,b2},0,{r2,g2,b2},0,{x2,y2,z2},0}}} }
#define gdSPDefLights4(ar,ag,ab,r1,g1,b1,x1,y1,z1,r2,g2,b2,x2,y2,z2,r3,g3,b3,x3,y3,z3,r4,g4,b4,x4,y4,z4) { {{ {ar,ag,ab},0,{ar,ag,ab},0}}, {{{ {r1,g1,b1},0,{r1,g1,b1},0,{x1,y1,z1},0}}, {{ {r2,g2,b2},0,{r2,g2,b2},0,{x2,y2,z2},0}}, {{ {r3,g3,b3},0,{r3,g3,b3},0,{x3,y3,z3},0}}, {{ {r4,g4,b4},0,{r4,g4,b4},0,{x4,y4,z4},0}}} }
#define gdSPDefLights5(ar,ag,ab,r1,g1,b1,x1,y1,z1,r2,g2,b2,x2,y2,z2,r3,g3,b3,x3,y3,z3,r4,g4,b4,x4,y4,z4,r5,g5,b5,x5,y5,z5) { {{ {ar,ag,ab},0,{ar,ag,ab},0}}, {{{ {r1,g1,b1},0,{r1,g1,b1},0,{x1,y1,z1},0}}, {{ {r2,g2,b2},0,{r2,g2,b2},0,{x2,y2,z2},0}}, {{ {r3,g3,b3},0,{r3,g3,b3},0,{x3,y3,z3},0}}, {{ {r4,g4,b4},0,{r4,g4,b4},0,{x4,y4,z4},0}}, {{ {r5,g5,b5},0,{r5,g5,b5},0,{x5,y5,z5},0}}} }
#define G_AD_PATTERN (0 << G_MDSFT_ALPHADITHER)
#define gdSPDefLights7(ar,ag,ab,r1,g1,b1,x1,y1,z1,r2,g2,b2,x2,y2,z2,r3,g3,b3,x3,y3,z3,r4,g4,b4,x4,y4,z4,r5,g5,b5,x5,y5,z5,r6,g6,b6,x6,y6,z6,r7,g7,b7,x7,y7,z7) { {{ {ar,ag,ab},0,{ar,ag,ab},0}}, {{{ {r1,g1,b1},0,{r1,g1,b1},0,{x1,y1,z1},0}}, {{ {r2,g2,b2},0,{r2,g2,b2},0,{x2,y2,z2},0}}, {{ {r3,g3,b3},0,{r3,g3,b3},0,{x3,y3,z3},0}}, {{ {r4,g4,b4},0,{r4,g4,b4},0,{x4,y4,z4},0}}, {{ {r5,g5,b5},0,{r5,g5,b5},0,{x5,y5,z5},0}}, {{ {r6,g6,b6},0,{r6,g6,b6},0,{x6,y6,z6},0}}, {{ {r7,g7,b7},0,{r7,g7,b7},0,{x7,y7,z7},0}}} }
#define OS_TASK_USR1 0x0020
#define OS_TASK_USR2 0x0040
#define OS_TASK_USR3 0x0080
#define gDPSetColor(pkt,c,d) _DW({ Gfx *_g = (Gfx *)(pkt); _g->words.w0 = _SHIFTL(c, 24, 8); _g->words.w1 = (unsigned int)(d); })
#define INTPTR_MIN (-0x80000000)
#define FMOD(x,y) ((x) - ((s32)((x) * (1.0f / (y))) * (f32)(y)))
#define G_TT_IA16 (3 << G_MDSFT_TEXTLUT)
#define PI_BASE_REG 0x04600000
#define gSPLoadUcodeL(pkt,ucode) gSPLoadUcode((pkt), OS_K0_TO_PHYSICAL(& ##ucode ##TextStart), OS_K0_TO_PHYSICAL(& ##ucode ##DataStart))
#define ACTOR_FLAG_1000 (1 << 12)
#define M_CONTROLLER_H 
#define GAME_PROF_SEND 12
#define G_BL_A_SHADE 2
#define G_SETFOGCOLOR 0xf8
#define G_CCMUX_0 31
#define GAME_DBG_DATA_SEND 3
#define gSPLookAtX(pkt,l) gDma2p((pkt),G_MOVEMEM,(l),sizeof(Light),G_MV_LIGHT,G_MVO_LOOKATX)
#define gSPLookAtY(pkt,l) gDma2p((pkt),G_MOVEMEM,(l),sizeof(Light),G_MV_LIGHT,G_MVO_LOOKATY)
#define alSeqpGetProgram alSeqpGetChlProgram
#define DPS_TEST_MODE_REG (DPS_BASE_REG + 0x04)
#define gsDPWord(wordhi,wordlo) gsImmp1(G_RDPHALF_1, (unsigned int)(wordhi)), gsImmp1(G_RDPHALF_2, (unsigned int)(wordlo))
#define ALFlagFailIf(condition,flag,error) if (condition) { return; }
#define G_RM_CLD_SURF RM_CLD_SURF(1)
#define gDPScisFillRectangle(pkt,ulx,uly,lrx,lry) { Gfx *_g = (Gfx *)(pkt); _g->words.w0 = (_SHIFTL(G_FILLRECT, 24, 8) | _SHIFTL(MAX((lrx),0), 14, 10) | _SHIFTL(MAX((lry),0), 2, 10)); _g->words.w1 = (_SHIFTL(MAX((ulx),0), 14, 10) | _SHIFTL(MAX((uly),0), 2, 10)); }
#define ERR_OSPIRAWSTARTDMA_SIZE 24
#define G_TF_BILERP (2 << G_MDSFT_TEXTFILT)
#define G_MWO_MATRIX_XX_XY_I 0x00
#define gsDPSetMaskImage(i) gsDPSetDepthImage(i)
#define G_DL_PUSH 0x00
#define INT16_MIN (-0x8000)
#define ACTOR_FLAG_100000 (1 << 20)
#define ACTOR_FLAG_8000 (1 << 15)
#define TLBLO_EXLWR 0x28
#define PI_CART_ADDR_REG (PI_BASE_REG + 0x04)
#define G_TX_CLAMP 0x2
#define RM_ZB_PCL_SURF(clk) Z_CMP | Z_UPD | CVG_DST_FULL | ZMODE_OPA | G_AC_DITHER | GBL_c ##clk(G_BL_CLR_IN, G_BL_0, G_BL_CLR_IN, G_BL_1)
#define ALIGN_MASK(n) (~((n) - 1))
#define AL_FX_FLANGE 4
#define FLASH_VERSION_MX_A 0x00c20001
#define A_OUT 0x02
#define ERR_OSPROFILEINIT_CNT 62
#define C0_ENTRYHI 10
#define ABS(x) (((x) >= 0) ? (x): -(x))
#define SR_KX 0x00000080
#define OS_PRIORITY_APPMAX 127
#define G_RM_RA_ZB_OPA_SURF RM_RA_ZB_OPA_SURF(1)
#define PFS_FILE_NAME_LEN 16
#define gDPLoadBlock(pkt,tile,uls,ult,lrs,dxt) _DW({ Gfx *_g = (Gfx *)(pkt); _g->words.w0 = (_SHIFTL(G_LOADBLOCK, 24, 8) | _SHIFTL(uls, 12, 12) | _SHIFTL(ult, 0, 12)); _g->words.w1 = (_SHIFTL(tile, 24, 3) | _SHIFTL((MIN(lrs,G_TX_LDBLK_MAX_TXL)), 12, 12) | _SHIFTL(dxt, 0, 12)); })
#define G_TRI_TXTR 0xca
#define ACTOR_FLAG_1 (1 << 0)
#define NULL 0
#define ACTOR_FLAG_8 (1 << 3)
#define SP_STATUS_IO_FULL (1 << 4)
#define _G_CC_SPARSEST PRIMITIVE, TEXEL0, LOD_FRACTION, TEXEL0, PRIMITIVE, TEXEL0, LOD_FRACTION, TEXEL0
#define G_BL_CLR_IN 0
#define OS_EVENT_PRENMI 14
#define GAME_YEAR_MAX 2032
#define RM_ZB_CLD_SURF(clk) Z_CMP | IM_RD | CVG_DST_SAVE | FORCE_BL | ZMODE_XLU | GBL_c ##clk(G_BL_CLR_IN, G_BL_A_IN, G_BL_CLR_MEM, G_BL_1MA)
#define ERR_ALSNDPSETPAR 112
#define _RCP_H_ 
#define G_MTX 0xda
#define gSPFogFactor(pkt,fm,fo) gMoveWd(pkt, G_MW_FOG, G_MWO_FOG, (_SHIFTL(fm,16,16) | _SHIFTL(fo,0,16)))
#define RM_TEX_EDGE(clk) CVG_DST_CLAMP | CVG_X_ALPHA | ALPHA_CVG_SEL | FORCE_BL | ZMODE_OPA | TEX_EDGE | AA_EN | GBL_c ##clk(G_BL_CLR_IN, G_BL_0, G_BL_CLR_IN, G_BL_1)
#define M_NPC_SCHEDULE_H 
#define THA_GA_H 
#define gDPLoadTextureBlock_4b(pkt,timg,fmt,width,height,pal,cms,cmt,masks,maskt,shifts,shiftt) _DW({ gDPSetTextureImage(pkt, fmt, G_IM_SIZ_16b, 1, timg); gDPSetTile(pkt, fmt, G_IM_SIZ_16b, 0, 0, G_TX_LOADTILE, 0, cmt, maskt, shiftt, cms, masks, shifts); gDPLoadSync(pkt); gDPLoadBlock(pkt, G_TX_LOADTILE, 0, 0, (((width)*(height)+3)>>2)-1, CALC_DXT_4b(width)); gDPPipeSync(pkt); gDPSetTile(pkt, fmt, G_IM_SIZ_4b, ((((width)>>1)+7)>>3), 0, G_TX_RENDERTILE, pal, cmt, maskt, shiftt, cms, masks, shifts); gDPSetTileSize(pkt, G_TX_RENDERTILE, 0, 0, ((width)-1) << G_TEXTURE_IMAGE_FRAC, ((height)-1) << G_TEXTURE_IMAGE_FRAC); })
#define OS_VI_PAL_HPF1 23
#define HOST_APP_CMD_READY 9
#define OS_PRIORITY_VIMGR 254
#define OS_MINOR_VERSION 0
#define OS_PRIORITY_RMONSPIN 200
#define G_RDPHALF_1 0xe1
#define G_RDPHALF_2 0xf1
#define G_CC_REFLECTRGBDECALA ENVIRONMENT, 0, TEXEL0, SHADE, 0, 0, 0, TEXEL0
#define C0_PROBE 0x8
#define BUF_FREE_WO_NEXT 0x8000
#define EXC_BREAK EXC_CODE(9)
#define OS_PIM_STACKSIZE 4096
#define __attribute__(x) 
#define VI_CTRL_PIXEL_ADV_2 0x02000
#define TLBRAND_RANDMASK 0x3f
#define alSeqpSetProgram alSeqpSetChlProgram
#define OS_VI_BIT_DEFLICKINTERLACE 0x0008
#define G_MVO_LOOKATY (1*24)
#define CAUSE_IPMASK 0x0000FF00
#define G_RM_AA_ZB_TEX_TERR RM_AA_ZB_TEX_TERR(1)
#define _SHIFT _SHIFTL
#define M_DTOR (3.14159265358979323846/180.0)
#define SP_UCODE_SIZE 4096
#define ERR_OSPIRAWSTARTDMA_ADDR 23
#define OS_VI_BIT_LORES 0x0100
#define gsDPSetDepthImage(i) gsSetImage(G_SETZIMG, 0, 0, 1, i)
#define ERR_OSSETTLBASID 13
#define BLOCK_X_NUM 7
#define G_RDPLOADSYNC 0xe6
#define AL_PHASE_SUSTAIN 2
#define gImmp0(pkt,c) { Gfx *_g = (Gfx *)(pkt); _g->words.w0 = _SHIFTL((c), 24, 8); }
#define WATCHLO_ADDRMASK 0xfffffff8
#define ERR_OSAISETNEXTBUFFER_ADDR 15
#define OS_NSEC_TO_CYCLES(n) (((u64)(n)*(OS_CPU_COUNTER/15625000LL))/(1000000000LL/15625000LL))
#define gImmp1(pkt,c,p0) _DW({ Gfx *_g = (Gfx *)(pkt); _g->words.w0 = _SHIFTL((c), 24, 8); _g->words.w1 = (unsigned int)(p0); })
#define PI_DOM_LAT_OFS 0x00
#define gsDPNoOp() gsDPNoParam(G_NOOP)
#define C0_MINREVMASK 0xf
#define GU_PARSEGBI_ALLMTX 16
#define OS_VI_FPAL_HAF1 53
#define G_BZ_PERSP 0
#define aDMEMMove(pkt,i,o,c) { Acmd *_a = (Acmd *)pkt; _a->words.w0 = _SHIFTL(A_DMEMMOVE, 24, 8) | _SHIFTL(i, 0, 24); _a->words.w1 = _SHIFTL(o, 16, 16) | _SHIFTL(c, 0, 16); }
#define C0_EPC 14
#define gDPSetFogColor(pkt,r,g,b,a) DPRGBColor(pkt, G_SETFOGCOLOR, r,g,b,a)
#define G_RM_NOOP2 RM_NOOP(2)
#define gDPSetDepthImage(pkt,i) gSetImage(pkt, G_SETZIMG, 0, 0, 1, i)
#define CONT_RELATIVE 0x0002
#define PIF_ROM_START 0x1FC00000
#define ERR_OSVISWAPBUFFER_ADDR 47
#define COLCHECK_FLAG_1 (1 << 0)
#define IS_KPTESEG(x) ((u32)(x) >= KPTE_SHDUBASE)
#define gDPLoadTLUT_pal16(pkt,pal,dram) _DW({ gDPSetTextureImage(pkt, G_IM_FMT_RGBA, G_IM_SIZ_16b, 1, dram); gDPTileSync(pkt); gDPSetTile(pkt, 0, 0, 0, (256+(((pal)&0xf)*16)), G_TX_LOADTILE, 0 , 0, 0, 0, 0, 0, 0); gDPLoadSync(pkt); gDPLoadTLUTCmd(pkt, G_TX_LOADTILE, 15); gDPPipeSync(pkt); })
#define AI_CONTROL_DMA_OFF 0
#define SR_KSU_USR 0x00000010
#define SP_SET_CPUSIGNAL SP_SET_SIG4
#define RP(x) rp->r_ ##x
#define UNITY_PITCH 0x8000
#define G_RM_AA_ZB_DEC_LINE2 RM_AA_ZB_DEC_LINE(2)
#define SI_STATUS_DMA_BUSY (1 << 0)
#define OS_VI_BIT_16PIXEL 0x0040
#define gsSPFogPosition(min,max) gsMoveWd(G_MW_FOG, G_MWO_FOG, (_SHIFTL((128000/((max)-(min))),16,16) | _SHIFTL(((500-(min))*256/((max)-(min))),0,16)))
#define MakeTexRect(xh,yh,flip,tile,xl,yl,s,t,dsdx,dtdy) G_TEXRECT, xh, yh, 0, flip, 0, tile, xl, yl, s, t, dsdx, dtdy
#define ERR_ALSEQP_POLY_VOICE 104
#define gSP1Quadrangle(pkt,v0,v1,v2,v3,flag) _DW({ Gfx *_g = (Gfx *)(pkt); _g->words.w0 = (_SHIFTL(G_QUAD, 24, 8)| __gsSP1Quadrangle_w1f(v0, v1, v2, v3, flag)); _g->words.w1 = __gsSP1Quadrangle_w2f(v0, v1, v2, v3, flag); })
#define C0_REVMASK 0xff
#define SP_DMA_DMEM (0 << 12)
#define ACTOR_FLAG_80000000 (1 << 31)
#define ERR_ALSEQTIME 117
#define gDPSetDepthSource(pkt,src) gSPSetOtherMode(pkt, G_SETOTHERMODE_L, G_MDSFT_ZSRCSEL, 1, src)
#define G_IM_SIZ_4b_LINE_BYTES G_IM_SIZ_4b_BYTES
#define gsDPLoadMultiTile_4b(timg,tmem,rtile,fmt,width,height,uls,ult,lrs,lrt,pal,cms,cmt,masks,maskt,shifts,shiftt) gsDPSetTextureImage(fmt, G_IM_SIZ_8b, ((width)>>1), timg), gsDPSetTile(fmt, G_IM_SIZ_8b, (((((lrs)-(uls)+1)>>1)+7)>>3), tmem, G_TX_LOADTILE, 0 , cmt, maskt, shiftt, cms, masks, shifts), gsDPLoadSync(), gsDPLoadTile( G_TX_LOADTILE, (uls)<<(G_TEXTURE_IMAGE_FRAC-1), (ult)<<(G_TEXTURE_IMAGE_FRAC), (lrs)<<(G_TEXTURE_IMAGE_FRAC-1), (lrt)<<(G_TEXTURE_IMAGE_FRAC)), gsDPPipeSync(), gsDPSetTile(fmt, G_IM_SIZ_4b, (((((lrs)-(uls)+1)>>1)+7)>>3), tmem, rtile, pal, cmt, maskt, shiftt, cms, masks, shifts), gsDPSetTileSize(rtile, (uls)<<G_TEXTURE_IMAGE_FRAC, (ult)<<G_TEXTURE_IMAGE_FRAC, (lrs)<<G_TEXTURE_IMAGE_FRAC, (lrt)<<G_TEXTURE_IMAGE_FRAC)
#define VI_H_SYNC_LEAP_REG VI_LEAP_REG
#define G_RM_AA_DEC_LINE2 RM_AA_DEC_LINE(2)
#define gsImmp0(c) {{ _SHIFTL((c), 24, 8) }}
#define RM_NOOP(clk) GBL_c ##clk(0, 0, 0, 0)
#define gsImmp2(c,p0,p1) {{ _SHIFTL((c), 24, 8), _SHIFTL((p0), 16, 16) | _SHIFTL((p1), 8, 8)}}
#define gsImmp3(c,p0,p1,p2) {{ _SHIFTL((c), 24, 8), (_SHIFTL((p0), 16, 16) | _SHIFTL((p1), 8, 8) | _SHIFTL((p2), 0, 8))}}
#define AI_CONTROL_REG (AI_BASE_REG + 0x08)
#define RM_AA_ZB_TEX_TERR(clk) AA_EN | Z_CMP | Z_UPD | IM_RD | CVG_DST_CLAMP | CVG_X_ALPHA | ALPHA_CVG_SEL | ZMODE_OPA | TEX_EDGE | GBL_c ##clk(G_BL_CLR_IN, G_BL_A_IN, G_BL_CLR_MEM, G_BL_1MA)
#define gDPLoadTextureBlockS(pkt,timg,fmt,siz,width,height,pal,cms,cmt,masks,maskt,shifts,shiftt) { gDPSetTextureImage(pkt, fmt, siz ##_LOAD_BLOCK, 1, timg); gDPSetTile(pkt, fmt, siz ##_LOAD_BLOCK, 0, 0, G_TX_LOADTILE, 0 , cmt, maskt, shiftt, cms, masks, shifts); gDPLoadSync(pkt); gDPLoadBlock(pkt, G_TX_LOADTILE, 0, 0, (((width)*(height) + siz ##_INCR) >> siz ##_SHIFT)-1,0); gDPPipeSync(pkt); gDPSetTile(pkt, fmt, siz, (((width) * siz ##_LINE_BYTES)+7)>>3, 0, G_TX_RENDERTILE, pal, cmt, maskt, shiftt, cms, masks, shifts); gDPSetTileSize(pkt, G_TX_RENDERTILE, 0, 0, ((width)-1) << G_TEXTURE_IMAGE_FRAC, ((height)-1) << G_TEXTURE_IMAGE_FRAC); }
#define SP_IBIST_GO (1 << 1)
#define G_RM_AA_TEX_EDGE2 RM_AA_TEX_EDGE(2)
#define BLOCKSIZE 32
#define alCSPGetFXMix alCSPGetChlFXMix
#define ALIGNOFFST (ALIGNSZ-1)
#define G_CD_DISABLE (3 << G_MDSFT_RGBDITHER)
#define gDPSetScissorFrac(pkt,mode,ulx,uly,lrx,lry) _DW({ Gfx *_g = (Gfx *)pkt; _g->words.w0 = _SHIFTL(G_SETSCISSOR, 24, 8) | _SHIFTL((int)((ulx)), 12, 12) | _SHIFTL((int)((uly)), 0, 12); _g->words.w1 = _SHIFTL(mode, 24, 2) | _SHIFTL((int)((lrx)), 12, 12) | _SHIFTL((int)((lry)), 0, 12); })
#define aADPCMdec(pkt,f,s) { Acmd *_a = (Acmd *)pkt; _a->words.w0 = _SHIFTL(A_ADPCM, 24, 8) | _SHIFTL(f, 16, 8); _a->words.w1 = (unsigned int)(s); }
#define VOICE_STATUS_BUSY 5
#define OS_RG_ALIGN_2B 2
#define OS_MESG_PRI_NORMAL 0
#define A_POLEF 14
#define gDPLoadMultiTile_4b(pkt,timg,tmem,rtile,fmt,width,height,uls,ult,lrs,lrt,pal,cms,cmt,masks,maskt,shifts,shiftt) _DW({ gDPSetTextureImage(pkt, fmt, G_IM_SIZ_8b, ((width)>>1), timg); gDPSetTile(pkt, fmt, G_IM_SIZ_8b, (((((lrs)-(uls)+1)>>1)+7)>>3), tmem, G_TX_LOADTILE, 0 , cmt, maskt, shiftt, cms, masks, shifts); gDPLoadSync(pkt); gDPLoadTile( pkt, G_TX_LOADTILE, (uls)<<(G_TEXTURE_IMAGE_FRAC-1), (ult)<<(G_TEXTURE_IMAGE_FRAC), (lrs)<<(G_TEXTURE_IMAGE_FRAC-1), (lrt)<<(G_TEXTURE_IMAGE_FRAC)); gDPPipeSync(pkt); gDPSetTile(pkt, fmt, G_IM_SIZ_4b, (((((lrs)-(uls)+1)>>1)+7)>>3), tmem, rtile, pal, cmt, maskt, shiftt, cms, masks, shifts); gDPSetTileSize(pkt, rtile, (uls)<<G_TEXTURE_IMAGE_FRAC, (ult)<<G_TEXTURE_IMAGE_FRAC, (lrs)<<G_TEXTURE_IMAGE_FRAC, (lrt)<<G_TEXTURE_IMAGE_FRAC); })
#define VI_CONTROL_REG (VI_BASE_REG + 0x00)
#define SI_PIF_ADDR_WR64B_REG (SI_BASE_REG + 0x10)
#define gSPTextureRectangleFlip(pkt,xl,yl,xh,yh,tile,s,t,dsdx,dtdy) { Gfx *_g = (Gfx *)(pkt); _g->words.w0 = (_SHIFTL(G_TEXRECTFLIP, 24, 8) | _SHIFTL(xh, 12, 12) | _SHIFTL(yh, 0, 12)); _g->words.w1 = (_SHIFTL(tile, 24, 3) | _SHIFTL(xl, 12, 12) | _SHIFTL(yl, 0, 12)); gImmp1(pkt, G_RDPHALF_1, (_SHIFTL(s, 16, 16) | _SHIFTL(t, 0, 16))); gImmp1(pkt, G_RDPHALF_2, (_SHIFTL(dsdx, 16, 16) | _SHIFTL(dtdy, 0, 16))); }
#define gSPClipRatio(pkt,r) { gMoveWd(pkt, G_MW_CLIP, G_MWO_CLIP_RNX, FR_NEG_ ##r); gMoveWd(pkt, G_MW_CLIP, G_MWO_CLIP_RNY, FR_NEG_ ##r); gMoveWd(pkt, G_MW_CLIP, G_MWO_CLIP_RPX, FR_POS_ ##r); gMoveWd(pkt, G_MW_CLIP, G_MWO_CLIP_RPY, FR_POS_ ##r); }
#define G_SETCONVERT 0xec
#define gsDPLoadSync() gsDPNoParam(G_RDPLOADSYNC)
#define AL_MAX_PRIORITY 127
#define OS_RG_ALIGN_4B 4
#define OS_STATE_STOPPED (1 << 0)
#define PSTATEMASK 0x00C0
#define ERR_OSPROFILESTOP_TIMER 69
#define MI_INTR_REG (MI_BASE_REG + 0x08)
#define sDPRGBColor(cmd,r,g,b,a) gsDPSetColor(cmd, (_SHIFTL(r, 24, 8) | _SHIFTL(g, 16, 8) | _SHIFTL(b, 8, 8) | _SHIFTL(a, 0, 8)))
#define gsDPSetCycleType(type) gsSPSetOtherMode(G_SETOTHERMODE_H, G_MDSFT_CYCLETYPE, 2, type)
#define G_LOADTILE 0xf4
#define CONTROLLER1(game) (&(game)->input[0])
#define DPC_CLR_FREEZE (1 << 2)
#define gsDPSetColorDither(mode) gsSPSetOtherMode(G_SETOTHERMODE_H, G_MDSFT_RGBDITHER, 2, mode)
#define CONT_NO_RESPONSE_ERROR 0x8
#define OS_VI_FPAL_HAN1 52
#define PCLEANEXCL 0x0080
#define lbRTC_IS_LEAPYEAR(year) (((year % 4) == 0 && ((year % 100) != 0)) || ((year % 400) == 0))
#define TLBPGMASK_4K 0x0
#define _OS_GBPAK_H_ 
#define G_CC_MODULATERGB G_CC_MODULATEI
#define DEVICE_TYPE_BULK 1
#define PI_STATUS_IO_BUSY (1 << 1)
#define HOST_DBG_DATA_ACK 4
#define MOTOR_STOP 0
#define OS_VI_MPAL_LAF1 31
#define gDPNoOpTag(pkt,tag) gDPParam(pkt, G_NOOP, tag)
#define FPCSR_FO 0x00000010
#define IS_KSEG2(x) ((u32)(x) >= K2BASE && (u32)(x) < KPTE_SHDUBASE)
#define G_CC_MODULATERGB_PRIM G_CC_MODULATEI_PRIM
#define CONTROLLER4(game) (&(game)->input[3])
#define L_TRIG CONT_L
#define aSetBuffer(pkt,f,i,o,c) { Acmd *_a = (Acmd *)pkt; _a->words.w0 = (_SHIFTL(A_SETBUFF, 24, 8) | _SHIFTL(f, 16, 8) | _SHIFTL(i, 0, 16)); _a->words.w1 = _SHIFTL(o, 16, 16) | _SHIFTL(c, 0, 16); }
#define G_RM_RA_ZB_OPA_DECAL RM_RA_ZB_OPA_DECAL(1)
#define AL_HEAP_DEBUG 1
#define ACTOR_FLAG_8000000 (1 << 27)
#define VI_H_WIDTH_REG VI_WIDTH_REG
#define RDRAM_DEVICE_ID_REG (RDRAM_BASE_REG + 0x04)
#define gsSPPopMatrix(n) gsSPPopMatrixN( (n), 1)
#define alSeqpGetPan alSeqpGetChlPan
#define G_TRI_SHADE_TXTR 0xce
#define FG_BLOCK_TOTAL_NUM (FG_BLOCK_X_NUM * FG_BLOCK_Z_NUM)
#define M_NPC_H 
#define FPCSR_FU 0x00000008
#define RDRAM_DELAY_REG (RDRAM_BASE_REG + 0x08)
#define INT64_MIN (-0x8000000000000000)
#define OC2_1 (1 << 0)
#define OC2_2 (1 << 1)
#define OC2_4 (1 << 2)
#define OS_IM_AI 0x00040401
#define OS_RG_ALIGN_8B 8
#define CONT_TYPE_MASK 0x1f07
#define DCACHE_LINESIZE 16
#define UT_Z_NUM UT_BASE_NUM
#define SEGMENT_OFFSET(a) ((unsigned int)(a) & 0x00ffffff)
#define __gsSP1Triangle_w1f(v0,v1,v2,flag) (((flag) == 0) ? __gsSP1Triangle_w1(v0, v1, v2): ((flag) == 1) ? __gsSP1Triangle_w1(v1, v2, v0): __gsSP1Triangle_w1(v2, v0, v1))
#define gsDPSetTexturePersp(type) gsSPSetOtherMode(G_SETOTHERMODE_H, G_MDSFT_TEXTPERSP, 1, type)
#define G_CV_K1 -43
#define RAD_TO_DEG(radians) ((radians) * (180.0f / (f32)M_PI))
#define ELEM_FLAG_NONE (0)
#define VOICE_WARN_TOO_LARGE 0x0800
#define SP_FRACPOS 0x00000100
#define AL_PHASE_SUSTREL 4
#define WATCHLO_RTRAP 0x00000002
#define G_AC_DITHER (3 << G_MDSFT_ALPHACOMPARE)
#define G_CV_K4 114
#define VI_CTRL_DITHER_FILTER_ON 0x10000
#define gsSPLineW3D(v0,v1,wd,flag) {{ _SHIFTL(G_LINE3D, 24, 8)|__gsSPLine3D_w1f(v0, v1, wd, flag), 0 }}
#define VOICE_STATUS_READY 0
#define TLBHI_NPID 255
#define G_SETTILE 0xf5
#define OS_IM_DP 0x00200401
#define G_LOADBLOCK 0xf3
#define EXC_CODE(x) ((x)<<2)
#define CHNL_ERR_OVERRUN 0x40
#define ZMODE_DEC 0xc00
#define ACTOR_FLAG_10000000 (1 << 28)
#define FORCE_BL 0x4000
#define F3DEX_GBI 
#define C_IWBINV 0x0
#define GU_PARSE_READY 3
#define FILTER_CLAMP 1
#define G_ZS_PRIM (1 << G_MDSFT_ZSRCSEL)
#define SP_STATUS_INTR_BREAK (1 << 6)
#define PFS_INODE_DIST_MAP (PFS_BANK_LAPPED_BY * PFS_SECTOR_PER_BANK)
#define FALSE 0
#define SNOWMAN_SAVE_COUNT 3
#define LIBC_STDBOOL_H 
#define ERR_OSVISETYSCALE_VIMGR 42
#define G_MDSFT_BLENDER 16
#define UNK_FUN_ARG void(*)(void)
#define G_MWO_SEGMENT_0 0x00
#define IO_READ(addr) (*(vu32*)PHYS_TO_K1(addr))
#define ERR_OSSETTHREADPRI 4
#define THPROF_IDMAX 64
#define PFS_ERR_CONTRFAIL CONT_OVERRUN_ERROR
#define MI_CLR_INIT (1 << 7)
#define HOST_DBG_CMD_READY 2
#define gDPSetKeyGB(pkt,cG,sG,wG,cB,sB,wB) { Gfx *_g = (Gfx *)(pkt); _g->words.w0 = (_SHIFTL(G_SETKEYGB, 24, 8) | _SHIFTL(wG, 12, 12) | _SHIFTL(wB, 0, 12)); _g->words.w1 = (_SHIFTL(cG, 24, 8) | _SHIFTL(sG, 16, 8) | _SHIFTL(cB, 8, 8) | _SHIFTL(sB, 0, 8)); }
#define G_TRI_FILL_ZBUFF 0xc9
#define ERR_OSUNMAPTLB 12
#define G_ACMUX_PRIM_LOD_FRAC 6
#define osMotorStart(x) __osMotorAccess((x), MOTOR_START)
#define DEG_TO_BINANG(degrees) TRUNCF_BINANG((degrees) * (0x8000 / 180.0f))
#define K1_TO_K0(x) ((u32)(x)&0x9FFFFFFF)
#define SR_ERL 0x00000004
#define DPC_CLR_CMD_CTR (1 << 8)
#define G_TX_WRAP 0
#define _MBI_H_ 
#define G_MWO_SEGMENT_6 0x06
#define G_RM_AA_ZB_OPA_SURF2 RM_AA_ZB_OPA_SURF(2)
#define gSPLineW3D(pkt,v0,v1,wd,flag) { Gfx *_g = (Gfx *)(pkt); _g->words.w0 = _SHIFTL(G_LINE3D, 24, 8)| __gsSPLine3D_w1f(v0, v1, wd, flag); _g->words.w1 = 0; }
#define FLASH_STATUS_WRITE_ERROR -1
#define AL_FX_ECHO 5
#define A_CONTINUE 0x00
#define FLASH_VERSION_MX_C 0x00c2001e
#define G_RM_AA_ZB_XLU_INTER RM_AA_ZB_XLU_INTER(1)
#define BINANG_TO_RAD(binang) ((f32)(binang) * ((f32)M_PI / 0x8000))
#define G_IM_SIZ_8b_TILE_BYTES G_IM_SIZ_8b_BYTES
#define DEVICE_TYPE_SRAM 3
#define PI_DOM_RLS_OFS 0x0C
#define FLASH_STATUS_ERASE_BUSY 2
#define G_RM_OPA_CI RM_OPA_CI(1)
#define RM_AA_PCL_SURF(clk) AA_EN | IM_RD | CVG_DST_CLAMP | ZMODE_OPA | G_AC_DITHER | GBL_c ##clk(G_BL_CLR_IN, G_BL_A_IN, G_BL_CLR_MEM, G_BL_1MA)
#define G_CCMUX_1 6
#define G_CC_TEMPLERP TEXEL1, TEXEL0, PRIM_LOD_FRAC, TEXEL0, TEXEL1, TEXEL0, PRIM_LOD_FRAC, TEXEL0
#define gDPTileSync(pkt) gDPNoParam(pkt, G_RDPTILESYNC)
#define FG_BLOCK_X_NUM (BLOCK_X_NUM - 2)
#define DPS_BUFTEST_DATA_REG (DPS_BASE_REG + 0x0C)
#define gsDPSetColorImage(f,s,w,i) gsSetImage(G_SETCIMG, f, s, w, i)
#define ACTOR_FLAG_4000000 (1 << 26)
#define G_MW_FOG 0x08
#define VI_CTRL_TYPE_32 0x00003
#define DPC_STATUS_DMA_BUSY (1 << 8)
#define PI_BSD_DOM1_PGS_REG (PI_BASE_REG + 0x1C)
#define RM_AA_ZB_OPA_TERR(clk) AA_EN | Z_CMP | Z_UPD | IM_RD | CVG_DST_CLAMP | ZMODE_OPA | ALPHA_CVG_SEL | GBL_c ##clk(G_BL_CLR_IN, G_BL_A_IN, G_BL_CLR_MEM, G_BL_1MA)
#define RDRAM_START RDRAM_0_START
#define G_RM_TEX_EDGE2 RM_TEX_EDGE(2)
#define G_RM_AA_SUB_SURF RM_AA_SUB_SURF(1)
#define M_HVQMTASK 7
#define RAD_TO_BINANG_ALT2(radians) TRUNCF_BINANG(((radians) * 0x8000) / (f32)M_PI)
#define SR_IMASK0 0x0000ff00
#define SR_IMASK1 0x0000fe00
#define SR_IMASK2 0x0000fc00
#define SR_IMASK3 0x0000f800
#define SR_IMASK4 0x0000f000
#define SR_IMASK5 0x0000e000
#define SR_IMASK7 0x00008000
#define SR_IMASK8 0x00000000
#define SR_BEV 0x00400000
#define C0_INX 0
#define UINT8_MAX 0xFF
#define RM_AA_XLU_LINE(clk) AA_EN | IM_RD | CVG_DST_CLAMP | CVG_X_ALPHA | ALPHA_CVG_SEL | FORCE_BL | ZMODE_OPA | GBL_c ##clk(G_BL_CLR_IN, G_BL_A_IN, G_BL_CLR_MEM, G_BL_1MA)
#define GU_PARSE_RDP_TYPE 2
#define aEnvMixer(pkt,f,s) { Acmd *_a = (Acmd *)pkt; _a->words.w0 = _SHIFTL(A_ENVMIXER, 24, 8) | _SHIFTL(f, 16, 8); _a->words.w1 = (unsigned int)(s); }
#define ERR_OSMAPTLB_ASID 11
#define DPC_CLR_FLUSH (1 << 4)
#define RM_AA_ZB_TEX_INTER(clk) AA_EN | Z_CMP | Z_UPD | IM_RD | CVG_DST_CLAMP | CVG_X_ALPHA | ALPHA_CVG_SEL | ZMODE_INTER | TEX_EDGE | GBL_c ##clk(G_BL_CLR_IN, G_BL_A_IN, G_BL_CLR_MEM, G_BL_A_MEM)
#define RM_XLU_SURF(clk) IM_RD | CVG_DST_FULL | FORCE_BL | ZMODE_OPA | GBL_c ##clk(G_BL_CLR_IN, G_BL_A_IN, G_BL_CLR_MEM, G_BL_1MA)
#define gsSPLight(l,n) gsDma2p( G_MOVEMEM,(l),sizeof(Light),G_MV_LIGHT,(n)*24+24)
#define G_RM_AA_ZB_OPA_SURF RM_AA_ZB_OPA_SURF(1)
#define SR_EXL 0x00000002
#define gdSPDefMtx(xx,yx,zx,wx,xy,yy,zy,wy,xz,yz,zz,wz,xw,yw,zw,ww) {{ (IPART(xx) << 0x10) | IPART(xy), (IPART(xz) << 0x10) | IPART(xw), (IPART(yx) << 0x10) | IPART(yy), (IPART(yz) << 0x10) | IPART(yw), (IPART(zx) << 0x10) | IPART(zy), (IPART(zz) << 0x10) | IPART(zw), (IPART(wx) << 0x10) | IPART(wy), (IPART(wz) << 0x10) | IPART(ww), (FPART(xx) << 0x10) | FPART(xy), (FPART(xz) << 0x10) | FPART(xw), (FPART(yx) << 0x10) | FPART(yy), (FPART(yz) << 0x10) | FPART(yw), (FPART(zx) << 0x10) | FPART(zy), (FPART(zz) << 0x10) | FPART(zw), (FPART(wx) << 0x10) | FPART(wy), (FPART(wz) << 0x10) | FPART(ww), }}
#define C0_WIRED 6
#define OS_SIM_STACKSIZE 4096
#define gDPSetHilite2Tile(pkt,tile,hilite,width,height) gDPSetTileSize(pkt, tile, (hilite)->h.x2 & 0xfff, (hilite)->h.y2 & 0xfff, ((((width)-1)*4)+(hilite)->h.x2) & 0xfff, ((((height)-1)*4)+(hilite)->h.y2) & 0xfff)
#define G_CC_SHADE 0, 0, 0, SHADE, 0, 0, 0, SHADE
#define G_RM_FOG_SHADE_A GBL_c1(G_BL_CLR_FOG, G_BL_A_SHADE, G_BL_CLR_IN, G_BL_1MA)
#define G_ACMUX_1 6
#define KUSIZE 0x80000000
#define ACTOR_FLAG_2000000 (1 << 25)
#define G_CC_MODULATERGB2 G_CC_MODULATEI2
#define G_RM_VISCVG RM_VISCVG(1)
#define HOST_PROF_REQ 11
#define OS_IM_PI 0x00100401
#define ERR_OSCREATEPIMANAGER 35
#define PI_WR_LEN_REG (PI_BASE_REG + 0x0C)
#define G_MV_LIGHT 10
#define G_IM_SIZ_8b_BYTES 1
#define G_RM_AA_ZB_DEC_LINE RM_AA_ZB_DEC_LINE(1)
#define G_SETENVCOLOR 0xfb
#define M_SUBMENU_H 
#define OS_VI_FPAL_HPF1 51
#define OS_VI_FPAL_HPF2 55
#define SP_STATUS_REG (SP_BASE_REG + 0x10)
#define GBL_c2(m1a,m1b,m2a,m2b) (m1a) << 28 | (m1b) << 24 | (m2a) << 20 | (m2b) << 16
#define TLBLO_NONCOHRNT 0x18
#define RM_AA_ZB_OPA_SURF(clk) AA_EN | Z_CMP | Z_UPD | IM_RD | CVG_DST_CLAMP | ZMODE_OPA | ALPHA_CVG_SEL | GBL_c ##clk(G_BL_CLR_IN, G_BL_A_IN, G_BL_CLR_MEM, G_BL_A_MEM)
#define GFX_H 
#define gSPLoadUcodeEx(pkt,uc_start,uc_dstart,uc_dsize) _DW({ Gfx *_g = (Gfx *)(pkt); _g->words.w0 = _SHIFTL(G_RDPHALF_1,24,8); _g->words.w1 = (unsigned int)(uc_dstart); _g = (Gfx *)(pkt); _g->words.w0 = (_SHIFTL(G_LOAD_UCODE,24,8)| _SHIFTL((int)(uc_dsize)-1,0,16)); _g->words.w1 = (unsigned int)(uc_start); })
#define OS_STATE_RUNNING (1 << 2)
#define RAMROM_PIF2BOOTSTRAP_OFFSET 0x1000
#define gsDPSetTileSize(t,uls,ult,lrs,lrt) gsDPLoadTileGeneric(G_SETTILESIZE, t, uls, ult, lrs, lrt)
#define OS_VI_PAL_LPF2 19
#define RAMROM_MSG_SIZE (RAMROM_BUF_SIZE*6)
#define AL_FX_BUFFER_SIZE 8192
#define G_ACMUX_TEXEL0 1
#define G_ACMUX_TEXEL1 2
#define ZMODE_OPA 0
#define RI_CONFIG_REG (RI_BASE_REG + 0x04)
#define G_MW_FORCEMTX 0x0c
#define OS_IM_SP 0x00010401
#define PFS_ID_BANK_1M 4
#define G_TX_NOLOD 0
#define AL_KEY_MAX 127
#define gsSPNumLights(n) gsMoveWd( G_MW_NUMLIGHT, G_MWO_NUMLIGHT, NUML(n))
#define G_MWO_MATRIX_WX_WY_F 0x38
#define G_MWO_MATRIX_WX_WY_I 0x18
#define AL_GAIN_CHANGE_TIME 1000
#define TLBINX_INXMASK 0x3f
#define G_MTX_PROJECTION 0x04
#define gDPSetTextureImage(pkt,f,s,w,i) gSetImage(pkt, G_SETTIMG, f, s, w, i)
#define SP_UCODE_DATA_SIZE 2048
#define BOOT_ADDRESS_INDY 0x88100000
#define G_RM_ZB_XLU_SURF2 RM_ZB_XLU_SURF(2)
#define CACHERR_EB 0x02000000
#define CACHERR_EC 0x40000000
#define CACHERR_ED 0x20000000
#define SI_BASE_REG 0x04800000
#define CACHERR_EI 0x01000000
#define LEO_TRACK_MODE 2
#define FIX32TOF(x) ((float)(x) * (1.0f / (float)0x00010000))
#define CACHERR_ER 0x80000000
#define CACHERR_ES 0x08000000
#define CACHERR_ET 0x10000000
#define G_MTX_MODELVIEW 0x00
#define OS_ERROR_MAGIC 0x6b617479
#define RM_AA_ZB_DEC_LINE(clk) AA_EN | Z_CMP | IM_RD | CVG_DST_SAVE | CVG_X_ALPHA | ALPHA_CVG_SEL | FORCE_BL | ZMODE_DEC | GBL_c ##clk(G_BL_CLR_IN, G_BL_A_IN, G_BL_CLR_MEM, G_BL_1MA)
#define gsSPSetOtherMode(cmd,sft,len,data) {{ _SHIFTL(cmd,24,8)|_SHIFTL(32-(sft)-(len),8,8)|_SHIFTL((len)-1,0,8), (unsigned int)(data) }}
#define TLBCTXT_BASESHIFT 23
#define C0_ERROR_EPC 30
#define AL_FX_CHORUS 3
#define OS_IM_VI 0x00080401
#define G_MAXFBZ 0x3fff
#define PFS_ERR_BAD_DATA 6
#define CB(x) ((x) * (x) * (x))
#define GDMACMD(x) (x)
#define TLBPGMASK_16K 0x6000
#define NORETURN __attribute__((noreturn))
#define AL_CMIDI_LOOPSTART_CODE 0x2E
#define DPC_STATUS_START_GCLK (1 << 3)
#define DCACHE_SIZE 0x2000
#define EXC_IBE EXC_CODE(6)
#define gsDPLoadTextureBlock_4b(timg,fmt,width,height,pal,cms,cmt,masks,maskt,shifts,shiftt) gsDPSetTextureImage(fmt, G_IM_SIZ_16b, 1, timg), gsDPSetTile(fmt, G_IM_SIZ_16b, 0, 0, G_TX_LOADTILE, 0 , cmt, maskt, shiftt, cms, masks, shifts), gsDPLoadSync(), gsDPLoadBlock(G_TX_LOADTILE, 0, 0, (((width)*(height)+3)>>2)-1, CALC_DXT_4b(width)), gsDPPipeSync(), gsDPSetTile(fmt, G_IM_SIZ_4b, ((((width)>>1)+7)>>3), 0, G_TX_RENDERTILE, pal, cmt, maskt, shiftt, cms, masks, shifts), gsDPSetTileSize(G_TX_RENDERTILE, 0, 0, ((width)-1) << G_TEXTURE_IMAGE_FRAC, ((height)-1) << G_TEXTURE_IMAGE_FRAC)
#define PFS_SECTOR_PER_BANK 32
#define CLR_ON_CVG 0x80
#define G_MV_VIEWPORT 8
#define RAMROM_RMON_WRITE_ADDR (RAMROM_MSG_ADDR + (3*RAMROM_BUF_SIZE))
#define ICACHE_LINESIZE 32
#define SP_IBIST_FAILED 0x78
#define ACTOR_FLAG_10000 (1 << 16)
#define spScissor spX2Scissor
#define ALIGNOF_MASK(x) ALIGN_MASK(ALIGNOF(x))
#define gsSPMatrix(m,p) gsDma2p( G_MTX,(m),sizeof(Mtx),(p)^G_MTX_PUSH,0)
#define G_SC_NON_INTERLACE 0
#define aLoadBuffer(pkt,s) { Acmd *_a = (Acmd *)pkt; _a->words.w0 = _SHIFTL(A_LOADBUFF, 24, 8); _a->words.w1 = (unsigned int)(s); }
#define GU_PARSERDP_PRHISTO 4
#define gsDPLoadTileGeneric(c,tile,uls,ult,lrs,lrt) {{ _SHIFTL(c, 24, 8) | _SHIFTL(uls, 12, 12) | _SHIFTL(ult, 0, 12), _SHIFTL(tile, 24, 3) | _SHIFTL(lrs, 12, 12) | _SHIFTL(lrt, 0, 12)}}
#define AI_BASE_REG 0x04500000
#define OS_VI_FPAL_HPN1 50
#define OS_VI_FPAL_HPN2 54
#define AI_PAL_MIN_FREQ 3050
#define DL_BM_OVERHEAD (12)
#define RAMROM_MSG_ADDR (RAMROM_SIZE - RAMROM_MSG_SIZE)
#define D_CBUTTONS CONT_D
#define G_CCMUX_ENVIRONMENT 5
#define G_IM_SIZ_32b_LINE_BYTES 2
#define OS_VI_GAMMA_DITHER_OFF 0x0008
#define GCCc1w0(saRGB1,mRGB1) (_SHIFTL((saRGB1), 5, 4) | _SHIFTL((mRGB1), 0, 5))
#define GCCc1w1(sbRGB1,saA1,mA1,aRGB1,sbA1,aA1) (_SHIFTL((sbRGB1), 24, 4) | _SHIFTL((saA1), 21, 3) | _SHIFTL((mA1), 18, 3) | _SHIFTL((aRGB1), 6, 3) | _SHIFTL((sbA1), 3, 3) | _SHIFTL((aA1), 0, 3))
#define FPCSR_C 0x00800000
#define OS_VI_BIT_NORMALINTERLACE 0x0004
#define C_ILT 0x4
#define ERR_OSGETREGIONBUFSIZE 56
#define G_IM_SIZ_4b 0
#define G_CCMUX_PRIMITIVE_ALPHA 10
#define ERR_OSVISETSPECIAL_VIMGR 44
#define alSeqpSetFXMix alSeqpSetChlFXMix
#define PFS_DIR_FULL 8
#define AL_KEY_MIN 0
#define FPCSR_RM_MASK 0x00000003
#define NUM_SEGMENTS (16)
#define MI_INTR_MASK_AI (1 << 2)
#define G_IM_SIZ_8b_LOAD_BLOCK G_IM_SIZ_16b
#define G_MDSFT_ALPHADITHER 4
#define SR_IMASK6 0x0000c000
#define _COLOR_H_ 
#define G_SETSCISSOR 0xed
#define G_RDPFULLSYNC 0xe9
#define G_RM_OPA_SURF2 RM_OPA_SURF(2)
#define G_RM_ZB_OPA_DECAL RM_ZB_OPA_DECAL(1)
#define gsDPSetCombineLERP(a0,b0,c0,d0,Aa0,Ab0,Ac0,Ad0,a1,b1,c1,d1,Aa1,Ab1,Ac1,Ad1) {{ _SHIFTL(G_SETCOMBINE, 24, 8) | _SHIFTL(GCCc0w0(G_CCMUX_ ##a0, G_CCMUX_ ##c0, G_ACMUX_ ##Aa0, G_ACMUX_ ##Ac0) | GCCc1w0(G_CCMUX_ ##a1, G_CCMUX_ ##c1), 0, 24), (unsigned int)(GCCc0w1(G_CCMUX_ ##b0, G_CCMUX_ ##d0, G_ACMUX_ ##Ab0, G_ACMUX_ ##Ad0) | GCCc1w1(G_CCMUX_ ##b1, G_ACMUX_ ##Aa1, G_ACMUX_ ##Ac1, G_CCMUX_ ##d1, G_ACMUX_ ##Ab1, G_ACMUX_ ##Ad1)) }}
#define CONFIG_EC_2_1 0x0
#define AL_PHASE_RELEASE 3
#define ERR_ALSEQP_MAP_VOICE 102
#define RAD_TO_BINANG_ALT(radians) TRUNCF_BINANG(((radians) / (f32)M_PI) * 0x8000)
#define SP_BASE_REG 0x04040000
#define B_BUTTON CONT_B
#define M_COMMON_DATA_H 
#define DPC_CURRENT_REG (DPC_BASE_REG + 0x08)
#define OS_PHYSICAL_TO_K1(x) (void *)(((u32)(x)+0xa0000000))
#define AL_EVTQ_END 0x7fffffff
#define lbRTC_YEAR_MAX 2099
#define CONFIG_DC_SHFT 6
#define DPC_STATUS_FREEZE (1 << 1)
#define G_RM_ZB_XLU_DECAL2 RM_ZB_XLU_DECAL(2)
#define M_LIGHTS_H 
#define TLBHI_VPN2MASK 0xffffe000
#define ERR_OSCREATEREGION_SIZE 51
#define SR_CU2 0x40000000
#define alCSPSetProgram alCSPSetChlProgram
#define G_IM_SIZ_16b_TILE_BYTES G_IM_SIZ_16b_BYTES
#define CAUSE_EXCSHIFT 2
#define gsDPNoOpTag(tag) gsDPParam(G_NOOP, tag)
#define A_CLEARBUFF 2
#define ERR_OSREADHOST_SIZE 71
#define gsDPSetFillColor(d) gsDPSetColor(G_SETFILLCOLOR, (d))
#define MI_INIT_MODE_REG (MI_BASE_REG + 0x00)
#define MI_INTR_MASK_DP (1 << 5)
#define G_IM_SIZ_8b 1
#define SP_CLR_YIELD SP_CLR_SIG0
#define G_CC_HILITERGBA2 ENVIRONMENT, COMBINED, TEXEL0, COMBINED, ENVIRONMENT, COMBINED, TEXEL0, COMBINED
#define OS_GBPAK_RSTB_STATUS 0x08
#define G_RM_ZB_OPA_SURF RM_ZB_OPA_SURF(1)
#define gsSPForceMatrix(mptr) gsDma2p(G_MOVEMEM,(mptr),sizeof(Mtx),G_MV_MATRIX,0), gsMoveWd(G_MW_FORCEMTX,0,0x00010000)
#define G_DEPTOZSrg(zval,near,far,flag,zmin,zmax) (((unsigned int)FTOFIX32(((flag) == G_BZ_PERSP ? (1.0f-(float)(near)/(float)(zval)) / (1.0f-(float)(near)/(float)(far )) : ((float)(zval) - (float)(near)) / ((float)(far ) - (float)(near))))) * (((int)((zmax) - (zmin)))&~1) + (int)FTOFIX32(zmin))
#define OS_VI_NTSC_HAF1 11
#define _OS_MOTOR_H_ 
#define gDPLoadMultiBlockS(pkt,timg,tmem,rtile,fmt,siz,width,height,pal,cms,cmt,masks,maskt,shifts,shiftt) { gDPSetTextureImage(pkt, fmt, siz ##_LOAD_BLOCK, 1, timg); gDPSetTile(pkt, fmt, siz ##_LOAD_BLOCK, 0, tmem, G_TX_LOADTILE, 0 , cmt, maskt, shiftt, cms, masks, shifts); gDPLoadSync(pkt); gDPLoadBlock(pkt, G_TX_LOADTILE, 0, 0, (((width)*(height) + siz ##_INCR) >> siz ##_SHIFT)-1,0); gDPPipeSync(pkt); gDPSetTile(pkt, fmt, siz, (((width) * siz ##_LINE_BYTES)+7)>>3, tmem, rtile, pal, cmt, maskt, shiftt, cms, masks, shifts); gDPSetTileSize(pkt, rtile, 0, 0, ((width)-1) << G_TEXTURE_IMAGE_FRAC, ((height)-1) << G_TEXTURE_IMAGE_FRAC); }
#define ALIGN256(val) (((val) + 0xFF) & ~0xFF)
#define gsSetImage(cmd,fmt,siz,width,i) {{ _SHIFTL(cmd, 24, 8) | _SHIFTL(fmt, 21, 3) | _SHIFTL(siz, 19, 2) | _SHIFTL((width)-1, 0, 12), (unsigned int)(i) }}
#define G_CCMUX_K4 7
#define G_CCMUX_K5 15
#define G_RM_ZB_CLD_SURF2 RM_ZB_CLD_SURF(2)
#define HOST_FAULT_ACK 15
#define gDPSetHilite1Tile(pkt,tile,hilite,width,height) gDPSetTileSize(pkt, tile, (hilite)->h.x1 & 0xfff, (hilite)->h.y1 & 0xfff, ((((width)-1)*4)+(hilite)->h.x1) & 0xfff, ((((height)-1)*4)+(hilite)->h.y1) & 0xfff)
#define G_RM_ZB_OVL_SURF RM_ZB_OVL_SURF(1)
#define G_DEPTOZS(zval,near,far,flag) G_DEPTOZSrg(zval, near, far, flag, 0, G_MAXZ)
#define _G_CC_BLENDPE ENVIRONMENT, PRIMITIVE, TEXEL0, PRIMITIVE, TEXEL0, 0, SHADE, 0
#define ERR_ALHEAPFIRSTBLOCK 127
#define OS_PRIORITY_RMON 250
#define CALC_DXT(width,b_txl) (((1 << G_TX_DXT_FRAC) + TXL2WORDS(width, b_txl) - 1) / TXL2WORDS(width, b_txl))
#define G_RM_AA_TEX_TERR RM_AA_TEX_TERR(1)
#define ALIGNOF(x) __alignof__(x)
#define CLAMP(x,min,max) ((x) < (min) ? (min) : (x) > (max) ? (max) : (x))
#define PFS_PAGE_NOT_EXIST 2
#define osMotorStop(x) __osMotorAccess((x), MOTOR_STOP)
#define G_SETOTHERMODE_H 0xe3
#define alCSPSetChannelPriority alCSPSetChlPriority
#define C_IST 0x8
#define ALPHA_CVG_SEL 0x2000
#define SR_ITS 0x01000000
#define OS_VI_PAL_LAF1 17
#define OS_VI_PAL_LAF2 21
#define CONFIG_NONCOHRNT 0x00000003
#define ERR_OSPIRAWWRITEIO 20
#define GU_BLINKRDP_HILITE 1
#define CONT_DOWN 0x0400
#define ACTOR_FLAG_2 (1 << 1)
#define RGBA16_GET_R(pixel) (((pixel) >> 11) & 0x1F)
#define ACTOR_FLAG_4 (1 << 2)
#define ERR_OSSTOPTIMER 77
#define OS_MESG_TYPE_DMAWRITE (OS_MESG_TYPE_BASE + 2)
#define _SIZE_T 
#define A_ENVMIXER 3
#define EXC_INT EXC_CODE(0)
#define IPART(x) ((qs1616(x) >> 16) & 0xFFFF)
#define GAME_PRINTF_SEND 5
#define gsDPLoadTextureTile_4b(timg,fmt,width,height,uls,ult,lrs,lrt,pal,cms,cmt,masks,maskt,shifts,shiftt) gsDPSetTextureImage(fmt, G_IM_SIZ_8b, ((width)>>1), timg), gsDPSetTile(fmt, G_IM_SIZ_8b, (((((lrs)-(uls)+1)>>1)+7)>>3), 0, G_TX_LOADTILE, 0 , cmt, maskt, shiftt, cms, masks, shifts), gsDPLoadSync(), gsDPLoadTile( G_TX_LOADTILE, (uls)<<(G_TEXTURE_IMAGE_FRAC-1), (ult)<<(G_TEXTURE_IMAGE_FRAC), (lrs)<<(G_TEXTURE_IMAGE_FRAC-1), (lrt)<<(G_TEXTURE_IMAGE_FRAC)), gsDPPipeSync(), gsDPSetTile(fmt, G_IM_SIZ_4b, (((((lrs)-(uls)+1)>>1)+7)>>3), 0, G_TX_RENDERTILE, pal, cmt, maskt, shiftt, cms, masks, shifts), gsDPSetTileSize(G_TX_RENDERTILE, (uls)<<G_TEXTURE_IMAGE_FRAC, (ult)<<G_TEXTURE_IMAGE_FRAC, (lrs)<<G_TEXTURE_IMAGE_FRAC, (lrt)<<G_TEXTURE_IMAGE_FRAC)
#define RAMROM_BUF_SIZE (4096)
#define UT_TOTAL_NUM (UT_X_NUM * UT_Z_NUM)
#define ERR_OSVISETXSCALE_VALUE 39
#define G_SETKEYGB 0xea
#define G_IM_SIZ_16b_BYTES 2
#define G_MDSFT_TEXTLOD 16
#define CONT_CARD_PULL 0x02
#define VI_CTRL_DIVOT_ON 0x00010
#define OS_TV_PAL 0
#define SP_SEMAPHORE_REG (SP_BASE_REG + 0x1C)
#define G_RM_ZB_OVL_SURF2 RM_ZB_OVL_SURF(2)
#define OS_MESG_TYPE_EDMAREAD (OS_MESG_TYPE_BASE + 5)
#define MI_MODE_REG MI_INIT_MODE_REG
#define K2BASE 0xC0000000
#define G_CC_INTERFERENCE TEXEL0, 0, TEXEL1, 0, TEXEL0, 0, TEXEL1, 0
#define UT_X_NUM UT_BASE_NUM
#define G_IM_SIZ_16b_LOAD_BLOCK G_IM_SIZ_16b
#define OS_DCACHE_ROUNDUP_SIZE(x) (u32)(((((u32)(x)+0xf)/0x10)*0x10))
#define ERR_ALHEAPNOFREE 125
#define gsDPSetTextureDetail(type) gsSPSetOtherMode(G_SETOTHERMODE_H, G_MDSFT_TEXTDETAIL, 2, type)
#define G_CC_MODULATEI TEXEL0, 0, SHADE, 0, 0, 0, 0, SHADE
#define _OS_EEPROM_H_ 
#define CONFIG_EP 0x0f000000
#define _gDPLoadTextureBlockTile(pkt,timg,tmem,rtile,fmt,siz,width,height,pal,cms,cmt,masks,maskt,shifts,shiftt) { gDPSetTextureImage(pkt, fmt, siz ##_LOAD_BLOCK, 1, timg); gDPSetTile(pkt, fmt, siz ##_LOAD_BLOCK, 0, tmem, G_TX_LOADTILE, 0, cmt, maskt, shiftt, cms, masks, shifts); gDPLoadSync(pkt); gDPLoadBlock(pkt, G_TX_LOADTILE, 0, 0, (((width)*(height) + siz ##_INCR) >> siz ##_SHIFT)-1, CALC_DXT(width, siz ##_BYTES)); gDPPipeSync(pkt); gDPSetTile(pkt, fmt, siz, (((width) * siz ##_LINE_BYTES)+7)>>3, tmem, rtile, pal, cmt, maskt, shiftt, cms, masks, shifts); gDPSetTileSize(pkt, rtile, 0, 0, ((width)-1) << G_TEXTURE_IMAGE_FRAC, ((height)-1) << G_TEXTURE_IMAGE_FRAC); }
#define OS_EVENT_AI 6
#define PFS_WRITTEN 2
#define _SPTASK_H_ 
#define UNKNOWN_STRUCTS_H 
#define SQ(x) ((x) * (x))
#define LIBU64_PAD_H 
#define gSPLoadUcode(pkt,uc_start,uc_dstart) gSPLoadUcodeEx((pkt), (uc_start), (uc_dstart), SP_UCODE_DATA_SIZE)
#define VI_V_BURST_REG (VI_BASE_REG + 0x2C)
#define CONT_TYPE_NORMAL 0x0005
#define SDIRTYEXCL 0x00001400
#define OS_VI_NTSC_HAN1 10
#define SP_DMEM_END 0x04000FFF
#define gSPMatrix(pkt,m,p) gDma2p((pkt),G_MTX,(m),sizeof(Mtx),(p)^G_MTX_PUSH,0)
#define G_SETZIMG 0xfe
#define ERR_OSPISTARTDMA_DEVADDR 31
#define FLASH_REL_DURATION 0x2
#define alSeqpSetChannelPriority alSeqpSetChlPriority
#define ACTOR_FLAG_40000000 (1 << 30)
#define G_TC_FILTCONV (5 << G_MDSFT_TEXTCONV)
#define RAMROM_APP_READ_ADDR (RAMROM_MSG_ADDR + (0*RAMROM_BUF_SIZE))
#define RDRAM_MIN_INTERVAL_REG (RDRAM_BASE_REG + 0x1c)
#define gsSPLookAtY(l) gsDma2p( G_MOVEMEM,(l),sizeof(Light),G_MV_LIGHT,G_MVO_LOOKATY)
#define EXC_RMISS EXC_CODE(2)
#define alSeqpSetPan alSeqpSetChlPan
#define _DW(macro) do {macro} while (0)
#define OS_VIM_STACKSIZE 4096
#define PI_SET_RESET PI_STATUS_RESET
#define G_DL 0xde
#define AI_STATUS_REG (AI_BASE_REG + 0x0C)
#define INT64_MAX 0x7FFFFFFFFFFFFFFF
#define K0_TO_PHYS(x) ((u32)(x)&0x1FFFFFFF)
#define PFS_LABEL_AREA 7
#define R_CBUTTONS CONT_F
#define OS_PRIORITY_SIMGR 140
#define G_SETPRIMCOLOR 0xfa
#define IS_KSEG0(x) ((u32)(x) >= K0BASE && (u32)(x) < K1BASE)
#define OS_EVENT_DP 9
#define SP_IBIST_CLEAR (1 << 2)
#define SR_KSU_KER 0x00000000
#define AL_DEFAULT_FXMIX 0
#define G_CC_MODULATERGB_PRIM2 G_CC_MODULATEI_PRIM2
#define OS_VI_PAL_LAN1 16
#define OS_VI_PAL_LAN2 20
#define OS_VI_MPAL_HAF1 39
#define gDPTextureRectangle(pkt,xl,yl,xh,yh,tile,s,t,dsdx,dtdy) { Gfx *_g = (Gfx *)(pkt); if (pkt); _g->words.w0 = (_SHIFTL(G_TEXRECT, 24, 8) | _SHIFTL(xh, 12, 12) | _SHIFTL(yh, 0, 12)); _g->words.w1 = (_SHIFTL(tile, 24, 3) | _SHIFTL(xl, 12, 12) | _SHIFTL(yl, 0, 12)); _g ++; _g->words.w0 = (_SHIFTL(s, 16, 16) | _SHIFTL(t, 0, 16)); _g->words.w1 = (_SHIFTL(dsdx, 16, 16) | _SHIFTL(dtdy, 0, 16)); }
#define G_ENDDL 0xdf
#define ANIMAL_NUM_MAX 15
#define G_IM_SIZ_DD 5
#define gSPLookAt(pkt,la) _DW({ gSPLookAtX(pkt,la); gSPLookAtY(pkt,(char *)(la)+16); })
#define MI_INTR_MASK_PI (1 << 4)
#define G_MDSFT_TEXTLUT 14
#define M_PAUSE_H 
#define A_SETVOL 9
#define TLBCTXT_BASEBITS 9
#define A_DMEMMOVE 10
#define GPACK_RGBA5551(r,g,b,a) ((((r)<<8) & 0xf800) | (((g)<<3) & 0x7c0) | (((b)>>2) & 0x3e) | ((a) & 0x1))
#define CAUSE_CEMASK 0x30000000
#define C0_MAJREVSHIFT 4
#define MASS_IMMOVABLE 0xFF
#define PI_BSD_DOM2_LAT_REG (PI_BASE_REG + 0x24)
#define gDPSetTexturePersp(pkt,type) gSPSetOtherMode(pkt, G_SETOTHERMODE_H, G_MDSFT_TEXTPERSP, 1, type)
#define OS_STATE_RUNNABLE (1 << 1)
#define SCREEN_HEIGHT 240
#define A_LOADADPCM 11
#define spMove spX2Move
#define ACTOR_FLAG_40000 (1 << 18)
#define CHNL_ERR_COLLISION 0x40
#define AL_PAN_CENTER 64
#define PHYS_TO_K0(x) ((u32)(x)|0x80000000)
#define PHYS_TO_K1(x) ((u32)(x)|0xA0000000)
#define G_MWO_SEGMENT_4 0x04
#define ERR_ALSNDPDELETE 108
#define AL_PAN_RIGHT 127
#define SSTATEMASK 0x00001c00
#define PFS_BANKS_256K 1
#define gsDPSetCombineMode(a,b) gsDPSetCombineLERP(a, b)
#define gSPViewport(pkt,v) gDma2p((pkt), G_MOVEMEM, (v), sizeof(Vp), G_MV_VIEWPORT, 0)
#define G_TP_PERSP (1 << G_MDSFT_TEXTPERSP)
#define RDRAM_RAS_INTERVAL_REG (RDRAM_BASE_REG + 0x18)
#define G_DMA_IO 0xd6
#define _ULTRATYPES_H_ 
#define OS_VI_DIVOT_OFF 0x0020
#define MI_INTR_MASK_SP (1 << 0)
#define G_MWO_POINT_RGBA 0x10
#define DPC_STATUS_TMEM_BUSY (1 << 4)
#define CONT_LEFT 0x0200
#define DPC_STATUS_PIPE_BUSY (1 << 5)
#define G_RDPTILESYNC 0xe8
#define CONFIG_SS 0x00200000
#define DPC_CLR_TMEM_CTR (1 << 6)
#define MI_MODE_RDRAM (1 << 9)
#define ERR_ALCSPVNOTFREE 130
#define G_BL_1MA 0
#define TLBINX_PROBE 0x80000000
#define gsDPSetPrimDepth(z,dz) gsDPSetColor(G_SETPRIMDEPTH, _SHIFTL(z, 16, 16) | _SHIFTL(dz, 0, 16))
#define gsDPTextureRectangle(xl,yl,xh,yh,tile,s,t,dsdx,dtdy) {{ (_SHIFTL(G_TEXRECT, 24, 8) | _SHIFTL(xh, 12, 12) | _SHIFTL(yh, 0, 12)), (_SHIFTL(tile, 24, 3) | _SHIFTL(xl, 12, 12) | _SHIFTL(yl, 0, 12)), }}, {{ _SHIFTL(s, 16, 16) | _SHIFTL(t, 0, 16), _SHIFTL(dsdx, 16, 16) | _SHIFTL(dtdy, 0, 16) }}
#define VI_WIDTH_REG (VI_BASE_REG + 0x08)
#define OS_VI_BIT_HIRES 0x0200
#define CVG_DST_FULL 0x200
#define DEG_TO_RAD(degrees) ((degrees) * ((f32)M_PI / 180.0f))
#define G_CC_MODULATEIDECALA_PRIM TEXEL0, 0, PRIMITIVE, 0, 0, 0, 0, TEXEL0
#define _REGION_H_ 
#define RDRAM_1_START 0x00200000
#define mPr_ERRAND_QUEST_NUM 5
#define gsSPEndDisplayList() {{ _SHIFTL(G_ENDDL, 24, 8), 0 }}
#define G_RDPSETOTHERMODE 0xef
#define AI_MAX_BIT_RATE 16
#define MI_INTR_MASK_VI (1 << 3)
#define RM_AA_ZB_OPA_INTER(clk) AA_EN | Z_CMP | Z_UPD | IM_RD | CVG_DST_CLAMP | ALPHA_CVG_SEL | ZMODE_INTER | GBL_c ##clk(G_BL_CLR_IN, G_BL_A_IN, G_BL_CLR_MEM, G_BL_A_MEM)
#define OS_CPU_COUNTER (OS_CLOCK_RATE*3/4)
#define OS_MESG_TYPE_DMAREAD (OS_MESG_TYPE_BASE + 1)
#define MI_SET_EBUS (1 << 10)
#define A_SPNOOP 0
#define SP_STATUS_DMA_FULL (1 << 3)
#define HOST_PIACCESS_REQ 1
#define CAUSE_EXCMASK 0x0000007C
#define gsSPSprite2DDraw(px,py) {{ (_SHIFTL(G_SPRITE2D_DRAW, 24, 8)), (_SHIFTL((px), 16, 16) | _SHIFTL((py), 0, 16)) }}
#define MI_MODE_EBUS (1 << 8)
#define DPC_SET_FREEZE (1 << 3)
#define OS_USEC_TO_CYCLES(n) (((u64)(n)*(OS_CPU_COUNTER/15625LL))/(1000000LL/15625LL))
#define _LANGUAGE_C 1
#define gSPTextureL(pkt,s,t,level,xparam,tile,on) { Gfx *_g = (Gfx *)(pkt); _g->words.w0 = (_SHIFTL(G_TEXTURE,24,8) | _SHIFTL((xparam),16,8) | _SHIFTL((level),11,3) | _SHIFTL((tile),8,3) | _SHIFTL((on),1,7)); _g->words.w1 = (_SHIFTL((s),16,16) | _SHIFTL((t),0,16)); }
#define ALIGN16(val) (((val) + 0xF) & ~0xF)
#define SP_EXTERN 0x00000400
#define OS_VI_MPAL_HAN1 38
#define gDPSetPrimColor(pkt,m,l,r,g,b,a) _DW({ Gfx *_g = (Gfx *)(pkt); _g->words.w0 = (_SHIFTL(G_SETPRIMCOLOR, 24, 8) | _SHIFTL(m, 8, 8) | _SHIFTL(l, 0, 8)); _g->words.w1 = (_SHIFTL(r, 24, 8) | _SHIFTL(g, 16, 8) | _SHIFTL(b, 8, 8) | _SHIFTL(a, 0, 8)); })
#define gDPSetConvert(pkt,k0,k1,k2,k3,k4,k5) { Gfx *_g = (Gfx *)(pkt); _g->words.w0 = (_SHIFTL(G_SETCONVERT, 24, 8) | _SHIFTL(k0, 13, 9) | _SHIFTL(k1, 4, 9) | _SHIFTR(k2, 5, 4)); _g->words.w1 = (_SHIFTL(k2, 27, 5) | _SHIFTL(k3, 18, 9) | _SHIFTL(k4, 9, 9) | _SHIFTL(k5, 0, 9)); }
#define gdSPDefLights3(ar,ag,ab,r1,g1,b1,x1,y1,z1,r2,g2,b2,x2,y2,z2,r3,g3,b3,x3,y3,z3) { {{ {ar,ag,ab},0,{ar,ag,ab},0}}, {{{ {r1,g1,b1},0,{r1,g1,b1},0,{x1,y1,z1},0}}, {{ {r2,g2,b2},0,{r2,g2,b2},0,{x2,y2,z2},0}}, {{ {r3,g3,b3},0,{r3,g3,b3},0,{x3,y3,z3},0}}} }
#define VI_TIMING_REG VI_BURST_REG
#define gDPSetEnvColor(pkt,r,g,b,a) DPRGBColor(pkt, G_SETENVCOLOR, r,g,b,a)
#define G_RDPCMDSIZ 64
#define VOICE_WARN_TOO_NOISY 0x8000
#define gSPDisplayList(pkt,dl) gDma1p(pkt,G_DL,dl,0,G_DL_PUSH)
#define G_CULLDL 0x03
#define OS_EVENT_THREADSTATUS 13
#define TLBINX_INXSHIFT 0
#define PFS_ERR_INVALID 5
#define OS_VI_NTSC_LPF2 5
#define NUML(n) ((n)*24)
#define gDPLoadMultiBlock(pkt,timg,tmem,rtile,fmt,siz,width,height,pal,cms,cmt,masks,maskt,shifts,shiftt) { gDPSetTextureImage(pkt, fmt, siz ##_LOAD_BLOCK, 1, timg); gDPSetTile(pkt, fmt, siz ##_LOAD_BLOCK, 0, tmem, G_TX_LOADTILE, 0, cmt, maskt, shiftt, cms, masks, shifts); gDPLoadSync(pkt); gDPLoadBlock(pkt, G_TX_LOADTILE, 0, 0, (((width)*(height) + siz ##_INCR) >> siz ##_SHIFT)-1, CALC_DXT(width, siz ##_BYTES)); gDPPipeSync(pkt); gDPSetTile(pkt, fmt, siz, (((width) * siz ##_LINE_BYTES)+7)>>3, tmem, rtile, pal, cmt, maskt, shiftt, cms, masks, shifts); gDPSetTileSize(pkt, rtile, 0, 0, ((width)-1) << G_TEXTURE_IMAGE_FRAC, ((height)-1) << G_TEXTURE_IMAGE_FRAC); }
#define UNK_2B0_DISP __gfxCtx->unk_2B0.p
#define G_CCMUX_PRIM_LOD_FRAC 14
#define BINANG_SUB(a,b) ((s16)(a - b))
#define CONFIG_EC_3_1 0x1
#define A_RESAMPLE 5
#define G_RM_AA_OPA_TERR RM_AA_OPA_TERR(1)
#define G_MDSFT_COLORDITHER 22
#define gsDPLoadTextureTile(timg,fmt,siz,width,height,uls,ult,lrs,lrt,pal,cms,cmt,masks,maskt,shifts,shiftt) gsDPSetTextureImage(fmt, siz, width, timg), gsDPSetTile(fmt, siz, (((((lrs)-(uls)+1) * siz ##_TILE_BYTES)+7)>>3), 0, G_TX_LOADTILE, 0 , cmt, maskt, shiftt, cms, masks, shifts), gsDPLoadSync(), gsDPLoadTile( G_TX_LOADTILE, (uls)<<G_TEXTURE_IMAGE_FRAC, (ult)<<G_TEXTURE_IMAGE_FRAC, (lrs)<<G_TEXTURE_IMAGE_FRAC, (lrt)<<G_TEXTURE_IMAGE_FRAC), gsDPPipeSync(), gsDPSetTile(fmt, siz, (((((lrs)-(uls)+1) * siz ##_LINE_BYTES)+7)>>3), 0, G_TX_RENDERTILE, pal, cmt, maskt, shiftt, cms, masks, shifts), gsDPSetTileSize(G_TX_RENDERTILE, (uls)<<G_TEXTURE_IMAGE_FRAC, (ult)<<G_TEXTURE_IMAGE_FRAC, (lrs)<<G_TEXTURE_IMAGE_FRAC, (lrt)<<G_TEXTURE_IMAGE_FRAC)
#define G_ON (1)
#define gsSPSprite2DScaleFlip(sx,sy,fx,fy) {{ (_SHIFTL(G_SPRITE2D_SCALEFLIP, 24, 8) | _SHIFTL((fx), 8, 8) | _SHIFTL((fy), 0, 8)), (_SHIFTL((sx), 16, 16) | _SHIFTL((sy), 0, 16)) }}
#define AL_MAX_CHANNELS 16
#define SR_KSU_SUP 0x00000008
#define _OS_H_ 
#define G_IM_FMT_RGBA 0
#define gsSPLoadUcode(uc_start,uc_dstart) gsSPLoadUcodeEx((uc_start), (uc_dstart), SP_UCODE_DATA_SIZE)
#define PLAYER_NUM 4
#define OS_K1_TO_PHYSICAL(x) (u32)(((char *)(x)-0xa0000000))
#define gSPNumLights(pkt,n) gMoveWd(pkt, G_MW_NUMLIGHT, G_MWO_NUMLIGHT, NUML(n))
#define CONT_RIGHT 0x0100
#define LEO_SECTOR_MODE 3
#define PFS_GBPAK_INITIALIZED 0x10
#define SP_DMEM_START 0x04000000
#define OS_EVENT_PI 8
#define TLBWIRED_WIREDMASK 0x3f
#define FR_POS_FRUSTRATIO_5 0x0000fffb
#define SP_CLR_SSTEP (1 << 5)
#define EEPROM_MAXBLOCKS 64
#define UINT16_MAX 0xFFFF
#define G_MDSFT_ALPHACOMPARE 0
#define ACTOR_FLAG_400000 (1 << 22)
#define G_RM_ZB_OPA_DECAL2 RM_ZB_OPA_DECAL(2)
#define DPC_SET_FLUSH (1 << 5)
#define MAX(a,b) ((a) > (b) ? (a) : (b))
#define M_AUDTASK 2
#define G_TEXTURE_GEN 0x00040000
#define ERR_OSVISETXSCALE_VIMGR 40
#define STICK_MAX 61.0f
#define PI_DOM_PWD_OFS 0x04
#define ERR_ALCSEQZEROSTATUS 128
#define AL_PHASE_NOTEON 0
#define C0_ENTRYLO0 2
#define C0_ENTRYLO1 3
#define RAMROM_CLOCKRATE_MASK 0xfffffff0
#define ERR_OSVIGETCURRENTFRAMEBUFFER 37
#define OS_VI_FPAL_LAF1 45
#define MI_NOOP_REG MI_VERSION_REG
#define G_RM_RA_ZB_OPA_DECAL2 RM_RA_ZB_OPA_DECAL(2)
#define OS_VI_NTSC_LAF1 3
#define AL_VOL_FULL 127
#define OS_EVENT_SI 5
#define OS_VI_NTSC_LAF2 7
#define G_IM_SIZ_4b_TILE_BYTES G_IM_SIZ_4b_BYTES
#define ERR_ALSEQMETA 120
#define G_ZBUFFER 0x00000001
#define DPC_STATUS_START_VALID (1 << 10)
#define AL_CMIDI_CNTRL_LOOPCOUNT_BIG 105
#define G_TEXTURE_SCALE_FRAC 16
#define HOST_LOG_ACK 8
#define G_QUAD 0x07
#define CONT_TYPE_VOICE 0x0100
#define AL_HEAP_MAGIC 0x20736a73
#define gsDPSetEnvColor(r,g,b,a) sDPRGBColor(G_SETENVCOLOR, r,g,b,a)
#define OS_IM_NONE 0x00000001
#define gsDPSetConvert(k0,k1,k2,k3,k4,k5) {{ (_SHIFTL(G_SETCONVERT, 24, 8) | _SHIFTL(k0, 13, 9) | _SHIFTL(k1, 4, 9) | _SHIFTR(k2, 5, 4)), (_SHIFTL(k2, 27, 5) | _SHIFTL(k3, 18, 9) | _SHIFTL(k4, 9, 9) | _SHIFTL(k5, 0, 9)) }}
#define PI_CLR_INTR PI_STATUS_CLR_INTR
#define gDPPipeSync(pkt) gDPNoParam(pkt, G_RDPPIPESYNC)
#define ERR_OSREADHOST_ADDR 70
#define G_MWO_POINT_ZSCREEN 0x1c
#define K1BASE 0xA0000000
#define __R4300_H__ 
#define RDRAM_ACTIVE_MODE 1
#define RM_PCL_SURF(clk) CVG_DST_FULL | FORCE_BL | ZMODE_OPA | G_AC_DITHER | GBL_c ##clk(G_BL_CLR_IN, G_BL_0, G_BL_CLR_IN, G_BL_1)
#define OS_VI_NTSC_HPF1 9
#define OS_VI_NTSC_HPF2 13
#define DPC_STATUS_FLUSH (1 << 2)
#define ACTOR_FLAG_80 (1 << 7)
#define A_ADPCM 1
#define SYS_MATRIX_H 
#define POLY_XLU_DISP __gfxCtx->polyXlu.p
#define ERR_OSPROFILEINIT_ORD 64
#define PADDRMASK 0xFFFFFF00
#define ERR_OSPROFILESTART_FLAG 67
#define DPS_TBIST_CLEAR (1 << 2)
#define G_CCMUX_PRIMITIVE 3
#define OS_EVENT_VI 7
#define _MIPS_SZLONG 32
#define SP_PC_REG 0x04080000
#define EXC_MOD EXC_CODE(1)
#define OS_IM_CPU 0x0000ff01
#define PFS_ERR_EXIST 9
#define MI_VERSION_REG (MI_BASE_REG + 0x04)
#define M_GFXTASK 1
#define OS_PM_64K 0x001e000
#define RM_RA_ZB_OPA_DECAL(clk) AA_EN | Z_CMP | CVG_DST_WRAP | ALPHA_CVG_SEL | ZMODE_DEC | GBL_c ##clk(G_BL_CLR_IN, G_BL_A_IN, G_BL_CLR_MEM, G_BL_A_MEM)
#define gSPGeometryMode(pkt,c,s) _DW({ Gfx *_g = (Gfx *)(pkt); _g->words.w0 = _SHIFTL(G_GEOMETRYMODE,24,8)|_SHIFTL(~(u32)(c),0,24); _g->words.w1 = (u32)(s); })
#define SP_IMEM_END 0x04001FFF
#define SP_MEM_ADDR_REG (SP_BASE_REG + 0x00)
#define OS_VI_PAL_LPF1 15
#define G_RM_AA_DEC_LINE RM_AA_DEC_LINE(1)
#define TLBLO_PFNMASK 0x3fffffc0
#define VI_BASE_REG 0x04400000
#define G_TX_RENDERTILE 0
#define ERR_OSVISETMODE 45
#define G_MTX_LOAD 0x02
#define VI_BURST_REG (VI_BASE_REG + 0x14)
#define gsDPLoadTLUTCmd(tile,count) {{ _SHIFTL(G_LOADTLUT, 24, 8), _SHIFTL((tile), 24, 3) | _SHIFTL((count), 14, 10) }}
#define G_ACMUX_COMBINED 0
#define MIN(a,b) ((a) < (b) ? (a) : (b))
#define G_AD_DISABLE (3 << G_MDSFT_ALPHADITHER)
#define G_IM_SIZ_32b_INCR 0
#define gDPNoOp(pkt) gDPNoParam(pkt, G_NOOP)
#define _OS_CONT_H_ 
#define ERR_OSPISTARTDMA_ADDR 32
#define PI_BSD_DOM2_RLS_REG (PI_BASE_REG + 0x30)
#define C0_PAGEMASK 5
#define VI_V_INTR_REG VI_INTR_REG
#define CONT_RANGE_ERROR -1
#define G_TT_RGBA16 (2 << G_MDSFT_TEXTLUT)
#define alCSPGetChannelPriority alCSPGetChlPriority
#define _OS_VI_H_ 
#define ALIGN64(val) (((val) + 0x3F) & ~0x3F)
#define G_RM_AA_ZB_XLU_SURF2 RM_AA_ZB_XLU_SURF(2)
#define RM_AA_ZB_XLU_INTER(clk) AA_EN | Z_CMP | IM_RD | CVG_DST_WRAP | CLR_ON_CVG | FORCE_BL | ZMODE_INTER | GBL_c ##clk(G_BL_CLR_IN, G_BL_A_IN, G_BL_CLR_MEM, G_BL_1MA)
#define _OS_REG_H_ 
#define OS_VI_MPAL_HPF2 41
#define spSetZ spX2SetZ
#define AL_FX_SMALLROOM 1
#define OS_DCACHE_ROUNDUP_ADDR(x) (void *)(((((u32)(x)+0xf)/0x10)*0x10))
#define aClearBuffer(pkt,d,c) { Acmd *_a = (Acmd *)pkt; _a->words.w0 = _SHIFTL(A_CLEARBUFF, 24, 8) | _SHIFTL(d, 0, 24); _a->words.w1 = (unsigned int)(c); }
#define G_CC_MODULATEIDECALA TEXEL0, 0, SHADE, 0, 0, 0, 0, TEXEL0
#define SCREEN_WIDTH 320
#define G_SHADING_SMOOTH 0x00200000
#define G_RM_AA_XLU_SURF2 RM_AA_XLU_SURF(2)
#define OS_IM_SW2 0x00000601
#define THPROF_STACKSIZE 256
#define DPS_BUFTEST_ADDR_REG (DPS_BASE_REG + 0x08)
#define DPC_SET_XBUS_DMEM_DMA (1 << 1)
#define AI_MAX_DAC_RATE 16384
#define G_MWO_MATRIX_ZZ_ZW_F 0x34
#define G_ACMUX_SHADE 4
#define G_MWO_MATRIX_ZZ_ZW_I 0x14
#define G_SCALE_FRAC 8
#define RAMROM_FONTDATA_OFFSET 0xb70
#define aPoleFilter(pkt,f,g,s) { Acmd *_a = (Acmd *)pkt; _a->words.w0 = (_SHIFTL(A_POLEF, 24, 8) | _SHIFTL(f, 16, 8) | _SHIFTL(g, 0, 16)); _a->words.w1 = (unsigned int)(s); }
#define ALIGN32(val) (((val) + 0x1F) & ~0x1F)
#define SP_SET_INTR (1 << 4)
#define G_CC_MODULATEIA TEXEL0, 0, SHADE, 0, TEXEL0, 0, SHADE, 0
#define VI_H_SYNC_REG (VI_BASE_REG + 0x1C)
#define OS_VI_NTSC_HPN1 8
#define OS_VI_NTSC_HPN2 12
#define PFS_ERR_NEW_GBCART 13
#define SR_CUMASK 0xf0000000
#define SR_KSU_MASK 0x00000018
#define G_IM_SIZ_32b_SHIFT 0
#define G_SETCIMG 0xff
#define G_TX_MIRROR 0x1
#define ACTOR_FLAG_4000 (1 << 14)
#define OS_GBPAK_POWER_ON 0x01
#define G_CC_TRILERP TEXEL1, TEXEL0, LOD_FRACTION, TEXEL0, TEXEL1, TEXEL0, LOD_FRACTION, TEXEL0
#define OS_VI_BIT_INTERLACE 0x0002
#define G_TP_NONE (0 << G_MDSFT_TEXTPERSP)
#define FLASH_LATENCY 0x5
#define __log__ 
#define DPC_CLR_CLOCK_CTR (1 << 9)
#define C0_IMPSHIFT 8
#define G_RM_AA_ZB_TEX_EDGE RM_AA_ZB_TEX_EDGE(1)
#define ACTOR_FLAG_2000 (1 << 13)
#define gSPBranchLessZrg(pkt,dl,vtx,zval,near,far,flag,zmin,zmax) { Gfx *_g = (Gfx *)(pkt); _g->words.w0 = _SHIFTL(G_RDPHALF_1,24,8); _g->words.w1 = (unsigned int)(dl); _g = (Gfx *)(pkt); _g->words.w0 = (_SHIFTL(G_BRANCH_Z,24,8)| _SHIFTL((vtx)*5,12,12)|_SHIFTL((vtx)*2,0,12)); _g->words.w1 = G_DEPTOZSrg(zval, near, far, flag, zmin, zmax); }
#define _OS_PFS_H_ 
#define OS_VI_PAL_LPN1 14
#define OS_VI_PAL_LPN2 18
#define A_LOOP 0x02
#define OS_VI_MPAL_HPF1 37
#define FLASH_STATUS_ERASE_ERROR -1
typedef unsigned char u8;
typedef unsigned short u16;
typedef unsigned long u32;
typedef unsigned long long u64;
typedef signed char s8;
typedef short s16;
typedef long s32;
typedef long long s64;
typedef volatile unsigned char vu8;
typedef volatile unsigned short vu16;
typedef volatile unsigned long vu32;
typedef volatile unsigned long long vu64;
typedef volatile signed char vs8;
typedef volatile short vs16;
typedef volatile long vs32;
typedef volatile long long vs64;
typedef float f32;
typedef double f64;
typedef unsigned int size_t;
typedef s32 OSPri;
typedef s32 OSId;
typedef union {
    struct {
        f32 f_odd;
        f32 f_even;
    } f;
    f64 d;
} __OSfp;
typedef struct {
    u64 at, v0, v1, a0, a1, a2, a3;
    u64 t0, t1, t2, t3, t4, t5, t6, t7;
    u64 s0, s1, s2, s3, s4, s5, s6, s7;
    u64 t8, t9;
    u64 gp, sp, s8, ra;
    u64 lo, hi;
    u32 sr, pc, cause, badvaddr, rcp;
    u32 fpcsr;
    __OSfp fp0, fp2, fp4, fp6, fp8, fp10, fp12, fp14;
    __OSfp fp16, fp18, fp20, fp22, fp24, fp26, fp28, fp30;
} __OSThreadContext;
typedef struct {
    u32 flag;
    u32 count;
    u64 time;
} __OSThreadprofile_s;
typedef struct OSThread_s {
    struct OSThread_s *next;
    OSPri priority;
    struct OSThread_s **queue;
    struct OSThread_s *tlnext;
    u16 state;
    u16 flags;
    OSId id;
    int fp;
    __OSThreadprofile_s *thprof;
    __OSThreadContext context;
} OSThread;
extern void osCreateThread(OSThread *, OSId, void (*)(void *), void *, void *, OSPri);
extern void osDestroyThread(OSThread *);
extern void osYieldThread(void);
extern void osStartThread(OSThread *);
extern void osStopThread(OSThread *);
extern OSId osGetThreadId(OSThread *);
extern void osSetThreadPri(OSThread *, OSPri);
extern OSPri osGetThreadPri(OSThread *);
typedef u32 OSEvent;
typedef void *OSMesg;
typedef struct OSMesgQueue_s {
    OSThread *mtqueue;
    OSThread *fullqueue;
    s32 validCount;
    s32 first;
    s32 msgCount;
    OSMesg *msg;
} OSMesgQueue;
extern void osCreateMesgQueue(OSMesgQueue *, OSMesg *, s32);
extern s32 osSendMesg(OSMesgQueue *, OSMesg, s32);
extern s32 osJamMesg(OSMesgQueue *, OSMesg, s32);
extern s32 osRecvMesg(OSMesgQueue *, OSMesg *, s32);
extern void osSetEventMesg(OSEvent, OSMesgQueue *, OSMesg);
typedef u32 OSIntMask;
typedef u32 OSHWIntr;
extern OSIntMask osGetIntMask(void);
extern OSIntMask osSetIntMask(OSIntMask);
typedef u32 OSPageMask;
extern void osMapTLB(s32, OSPageMask, void *, u32, u32, s32);
extern void osMapTLBRdb(void);
extern void osUnmapTLB(s32);
extern void osUnmapTLBAll(void);
extern void osSetTLBASID(s32);
typedef struct {
    u32 errStatus;
    void *dramAddr;
    void *C2Addr;
    u32 sectorSize;
    u32 C1ErrNum;
    u32 C1ErrSector[4];
} __OSBlockInfo;
typedef struct {
    u32 cmdType;
    u16 transferMode;
    u16 blockNum;
    s32 sectorNum;
    u32 devAddr;
    u32 bmCtlShadow;
    u32 seqCtlShadow;
    __OSBlockInfo block[2];
} __OSTranxInfo;
typedef struct OSPiHandle_s {
    struct OSPiHandle_s *next;
    u8 type;
    u8 latency;
    u8 pageSize;
    u8 relDuration;
    u8 pulse;
    u8 domain;
    u32 baseAddress;
    u32 speed;
    __OSTranxInfo transferInfo;
} OSPiHandle;
typedef struct {
    u8 type;
    u32 address;
} OSPiInfo;
typedef struct {
    u16 type;
    u8 pri;
    u8 status;
    OSMesgQueue *retQueue;
} OSIoMesgHdr;
typedef struct {
    OSIoMesgHdr hdr;
    void *dramAddr;
    u32 devAddr;
    u32 size;
    OSPiHandle *piHandle;
} OSIoMesg;
typedef struct {
    s32 active;
    OSThread *thread;
    OSMesgQueue *cmdQueue;
    OSMesgQueue *evtQueue;
    OSMesgQueue *acsQueue;
    s32 (*dma)(s32, u32, void *, u32);
    s32 (*edma)(OSPiHandle *, s32, u32, void *, u32);
} OSDevMgr;
extern OSPiHandle *__osPiTable;
extern u32 osPiGetStatus(void);
extern s32 osPiGetDeviceType(void);
extern s32 osPiWriteIo(u32, u32);
extern s32 osPiReadIo(u32, u32 *);
extern s32 osPiStartDma(OSIoMesg *, s32, s32, u32, void *, u32, OSMesgQueue *);
extern void osCreatePiManager(OSPri, OSMesgQueue *, OSMesg *, s32);
extern OSPiHandle *osCartRomInit(void);
extern OSPiHandle *osLeoDiskInit(void);
extern OSPiHandle *osDriveRomInit(void);
extern s32 osEPiDeviceType(OSPiHandle *, OSPiInfo *);
extern s32 osEPiWriteIo(OSPiHandle *, u32 , u32 );
extern s32 osEPiReadIo(OSPiHandle *, u32 , u32 *);
extern s32 osEPiStartDma(OSPiHandle *, OSIoMesg *, s32);
extern s32 osEPiLinkHandle(OSPiHandle *);
typedef struct {
    u32 ctrl;
    u32 width;
    u32 burst;
    u32 vSync;
    u32 hSync;
    u32 leap;
    u32 hStart;
    u32 xScale;
    u32 vCurrent;
} OSViCommonRegs;
typedef struct {
    u32 origin;
    u32 yScale;
    u32 vStart;
    u32 vBurst;
    u32 vIntr;
} OSViFieldRegs;
typedef struct {
    u8 type;
    OSViCommonRegs comRegs;
    OSViFieldRegs fldRegs[2];
} OSViMode;
extern OSViMode osViModeTable[];
extern OSViMode osViModeNtscLpn1;
extern OSViMode osViModeNtscLpf1;
extern OSViMode osViModeNtscLan1;
extern OSViMode osViModeNtscLaf1;
extern OSViMode osViModeNtscLpn2;
extern OSViMode osViModeNtscLpf2;
extern OSViMode osViModeNtscLan2;
extern OSViMode osViModeNtscLaf2;
extern OSViMode osViModeNtscHpn1;
extern OSViMode osViModeNtscHpf1;
extern OSViMode osViModeNtscHan1;
extern OSViMode osViModeNtscHaf1;
extern OSViMode osViModeNtscHpn2;
extern OSViMode osViModeNtscHpf2;
extern OSViMode osViModePalLpn1;
extern OSViMode osViModePalLpf1;
extern OSViMode osViModePalLan1;
extern OSViMode osViModePalLaf1;
extern OSViMode osViModePalLpn2;
extern OSViMode osViModePalLpf2;
extern OSViMode osViModePalLan2;
extern OSViMode osViModePalLaf2;
extern OSViMode osViModePalHpn1;
extern OSViMode osViModePalHpf1;
extern OSViMode osViModePalHan1;
extern OSViMode osViModePalHaf1;
extern OSViMode osViModePalHpn2;
extern OSViMode osViModePalHpf2;
extern OSViMode osViModeMpalLpn1;
extern OSViMode osViModeMpalLpf1;
extern OSViMode osViModeMpalLan1;
extern OSViMode osViModeMpalLaf1;
extern OSViMode osViModeMpalLpn2;
extern OSViMode osViModeMpalLpf2;
extern OSViMode osViModeMpalLan2;
extern OSViMode osViModeMpalLaf2;
extern OSViMode osViModeMpalHpn1;
extern OSViMode osViModeMpalHpf1;
extern OSViMode osViModeMpalHan1;
extern OSViMode osViModeMpalHaf1;
extern OSViMode osViModeMpalHpn2;
extern OSViMode osViModeMpalHpf2;
extern OSViMode osViModeFpalLpn1;
extern OSViMode osViModeFpalLpf1;
extern OSViMode osViModeFpalLan1;
extern OSViMode osViModeFpalLaf1;
extern OSViMode osViModeFpalLpn2;
extern OSViMode osViModeFpalLpf2;
extern OSViMode osViModeFpalLan2;
extern OSViMode osViModeFpalLaf2;
extern OSViMode osViModeFpalHpn1;
extern OSViMode osViModeFpalHpf1;
extern OSViMode osViModeFpalHan1;
extern OSViMode osViModeFpalHaf1;
extern OSViMode osViModeFpalHpn2;
extern OSViMode osViModeFpalHpf2;
extern u32 osViGetStatus(void);
extern u32 osViGetCurrentMode(void);
extern u32 osViGetCurrentLine(void);
extern u32 osViGetCurrentField(void);
extern void *osViGetCurrentFramebuffer(void);
extern void *osViGetNextFramebuffer(void);
extern void osViSetXScale(f32);
extern void osViSetYScale(f32);
extern void osViExtendVStart(u32);
extern void osViSetSpecialFeatures(u32);
extern void osViSetMode(OSViMode *);
extern void osViSetEvent(OSMesgQueue *, OSMesg, u32);
extern void osViSwapBuffer(void *);
extern void osViBlack(u8);
extern void osViFade(u8, u16);
extern void osViRepeatLine(u8);
extern void osCreateViManager(OSPri);
extern u32 osAiGetStatus(void);
extern u32 osAiGetLength(void);
extern s32 osAiSetFrequency(u32);
extern s32 osAiSetNextBuffer(void *, u32);
typedef u64 OSTime;
typedef struct OSTimer_s {
 struct OSTimer_s *next;
 struct OSTimer_s *prev;
 OSTime interval;
 OSTime value;
 OSMesgQueue *mq;
 OSMesg msg;
} OSTimer;
extern OSTime osGetTime(void);
extern void osSetTime(OSTime);
extern int osSetTimer(OSTimer *, OSTime, OSTime,
       OSMesgQueue *, OSMesg);
extern int osStopTimer(OSTimer *);
typedef struct {
 u16 type;
 u8 status;
 u8 errno;
}OSContStatus;
typedef struct {
 u16 button;
 s8 stick_x;
 s8 stick_y;
 u8 errno;
} OSContPad;
typedef struct {
 void *address;
 u8 databuffer[32];
        u8 addressCrc;
 u8 dataCrc;
 u8 errno;
} OSContRamIo;
extern s32 osContInit(OSMesgQueue *, u8 *, OSContStatus *);
extern s32 osContReset(OSMesgQueue *, OSContStatus *);
extern s32 osContStartQuery(OSMesgQueue *);
extern s32 osContStartReadData(OSMesgQueue *);
extern s32 osContSetCh(u8);
extern void osContGetQuery(OSContStatus *);
extern void osContGetReadData(OSContPad *);
typedef struct {
 int status;
 OSMesgQueue *queue;
 int channel;
 u8 id[32];
 u8 label[32];
 int version;
 int dir_size;
 int inode_table;
 int minode_table;
 int dir_table;
 int inode_start_page;
 u8 banks;
 u8 activebank;
} OSPfs;
typedef struct {
 u32 file_size;
   u32 game_code;
   u16 company_code;
   char ext_name[4];
   char game_name[16];
} OSPfsState;
extern s32 osPfsInitPak(OSMesgQueue *, OSPfs *, int);
extern s32 osPfsRepairId(OSPfs *);
extern s32 osPfsInit(OSMesgQueue *, OSPfs *, int);
extern s32 osPfsReFormat(OSPfs *, OSMesgQueue *, int);
extern s32 osPfsChecker(OSPfs *);
extern s32 osPfsAllocateFile(OSPfs *, u16, u32, u8 *, u8 *, int, s32 *);
extern s32 osPfsFindFile(OSPfs *, u16, u32, u8 *, u8 *, s32 *);
extern s32 osPfsDeleteFile(OSPfs *, u16, u32, u8 *, u8 *);
extern s32 osPfsReadWriteFile(OSPfs *, s32, u8, int, int, u8 *);
extern s32 osPfsFileState(OSPfs *, s32, OSPfsState *);
extern s32 osPfsGetLabel(OSPfs *, u8 *, int *);
extern s32 osPfsSetLabel(OSPfs *, u8 *);
extern s32 osPfsIsPlug(OSMesgQueue *, u8 *);
extern s32 osPfsFreeBlocks(OSPfs *, s32 *);
extern s32 osPfsNumFiles(OSPfs *, s32 *, s32 *);
typedef struct {
  u16 fixed1;
  u16 start_address;
  u8 nintendo_chr[0x30];
  u8 game_title[16];
  u16 company_code;
  u8 body_code;
  u8 cart_type;
  u8 rom_size;
  u8 ram_size;
  u8 country_code;
  u8 fixed2;
  u8 version;
  u8 isum;
  u16 sum;
} OSGbpakId;
extern s32 osGbpakInit(OSMesgQueue *, OSPfs *, int);
extern s32 osGbpakPower(OSPfs *, s32);
extern s32 osGbpakGetStatus(OSPfs *, u8 *);
extern s32 osGbpakReadWrite(OSPfs *, u16, u16, u8 *, u16);
extern s32 osGbpakReadId(OSPfs *, OSGbpakId *, u8 *);
extern s32 osGbpakCheckConnector(OSPfs *, u8 *);
typedef struct {
  OSMesgQueue *__mq;
  int __channel;
  s32 __mode;
  u8 cmd_status;
} OSVoiceHandle;
typedef struct {
  u16 warning;
  u16 answer_num;
  u16 voice_level;
  u16 voice_sn;
  u16 voice_time;
  u16 answer[5];
  u16 distance[5];
} OSVoiceData;
extern s32 osVoiceInit(OSMesgQueue *, OSVoiceHandle *, int);
extern s32 osVoiceCheckWord(u8 *data);
extern s32 osVoiceClearDictionary(OSVoiceHandle *, u8);
extern s32 osVoiceControlGain(OSVoiceHandle *, s32, s32);
extern s32 osVoiceSetWord(OSVoiceHandle *, u8 *);
extern s32 osVoiceStartReadData(OSVoiceHandle *);
extern s32 osVoiceStopReadData(OSVoiceHandle *);
extern s32 osVoiceGetReadData(OSVoiceHandle *, OSVoiceData *);
extern s32 osVoiceMaskDictionary(OSVoiceHandle *, u8 *, int);
extern void osVoiceCountSyllables(u8 *, u32 *);
extern void osInvalDCache(void *, s32);
extern void osInvalICache(void *, s32);
extern void osWritebackDCache(void *, s32);
extern void osWritebackDCacheAll(void);
typedef struct {
 u16 *histo_base;
 u32 histo_size;
 u32 *text_start;
 u32 *text_end;
} OSProf;
extern void osProfileInit(OSProf *, u32 profcnt);
extern void osProfileStart(u32);
extern void osProfileFlush(void);
extern void osProfileStop(void);
extern void osThreadProfileClear(OSId);
extern void osThreadProfileInit(void);
extern void osThreadProfileStart(void);
extern void osThreadProfileStop(void);
extern u32 osThreadProfileReadCount(OSId);
extern u32 osThreadProfileReadCountTh(OSThread*);
extern OSTime osThreadProfileReadTime(OSId);
extern OSTime osThreadProfileReadTimeTh(OSThread*);
extern u32 osGetCount(void);
extern s32 osRomType;
extern void *osRomBase;
extern s32 osTvType;
extern s32 osResetType;
extern s32 osCicId;
extern s32 osVersion;
extern u32 osMemSize;
extern s32 osAppNMIBuffer[];
extern u64 osClockRate;
extern OSIntMask __OSGlobalIntMask;
extern void osInitialize(void);
extern void osExit(void);
extern u32 osGetMemSize(void);
extern s32 osAfterPreNMI(void);
extern s32 osEepromProbe(OSMesgQueue *);
extern s32 osEepromRead(OSMesgQueue *, u8, u8 *);
extern s32 osEepromWrite(OSMesgQueue *, u8, u8 *);
extern s32 osEepromLongRead(OSMesgQueue *, u8, u8 *, int);
extern s32 osEepromLongWrite(OSMesgQueue *, u8, u8 *, int);
extern OSPiHandle *osFlashReInit(u8 latency, u8 pulse,
                 u8 page_size, u8 rel_duration, u32 start);
extern OSPiHandle *osFlashInit(void);
extern void osFlashReadStatus(u8 *flash_status);
extern void osFlashReadId(u32 *flash_type, u32 *flash_maker);
extern void osFlashClearStatus(void);
extern s32 osFlashAllErase(void);
extern s32 osFlashSectorErase(u32 page_num);
extern s32 osFlashWriteBuffer(OSIoMesg *mb, s32 priority,
                void *dramAddr, OSMesgQueue *mq);
extern s32 osFlashWriteArray(u32 page_num);
extern s32 osFlashReadArray(OSIoMesg *mb, s32 priority, u32 page_num,
                void *dramAddr, u32 n_pages, OSMesgQueue *mq);
extern void osFlashChange(u32 flash_num);
extern void osFlashAllEraseThrough(void);
extern void osFlashSectorEraseThrough(u32 page_num);
extern s32 osFlashCheckEraseEnd(void);
extern void __osInitialize_common(void);
extern s32 osTestHost(void);
extern void osReadHost(void *, u32);
extern void osWriteHost(void *, u32);
extern void osAckRamromRead(void);
extern void osAckRamromWrite(void);
extern void osInitRdb(u8 *sendBuf, u32 sendSize);
extern u32 osVirtualToPhysical(void *);
extern void * osPhysicalToVirtual(u32);
extern u32 osDpGetStatus(void);
extern void osDpSetStatus(u32);
extern void osDpGetCounters(u32 *);
extern s32 osDpSetNextBuffer(void *, u64);
extern s32 osMotorInit(OSMesgQueue *, OSPfs *, int);
extern s32 __osMotorAccess(OSPfs *, s32);
extern void bcopy(const void *, void *, int);
extern int bcmp(const void *, const void *, int);
extern void bzero(void *, int);
extern int sprintf(char *s, const char *fmt, ...);
extern void osSyncPrintf(const char *fmt, ...);
typedef struct _Region_s {
 u8 *r_startBufferAddress;
 u8 *r_endAddress;
 s32 r_bufferSize;
 s32 r_bufferCount;
 u16 r_freeList;
 u16 r_alignSize;
} OSRegion;
extern void *osCreateRegion(void *, u32, u32, u32);
extern void *osMalloc(void *);
extern void osFree(void *, void *);
extern s32 osGetRegionBufCount(void *);
extern s32 osGetRegionBufSize(void *);
extern void rmonMain( void * );
extern void rmonPrintf( const char *, ... );
typedef struct {
 u32 type;
 u32 flags;
 u64 *ucode_boot;
 u32 ucode_boot_size;
 u64 *ucode;
 u32 ucode_size;
 u64 *ucode_data;
 u32 ucode_data_size;
 u64 *dram_stack;
 u32 dram_stack_size;
 u64 *output_buff;
 u64 *output_buff_size;
 u64 *data_ptr;
 u32 data_size;
 u64 *yield_data_ptr;
 u32 yield_data_size;
} OSTask_t;
typedef union {
    OSTask_t t;
    long long int force_structure_alignment;
} OSTask;
typedef u32 OSYieldResult;
extern void osSpTaskLoad(OSTask *tp);
extern void osSpTaskStartGo(OSTask *tp);
extern void osSpTaskYield(void);
extern OSYieldResult osSpTaskYielded(OSTask *tp);
typedef struct {
 short ob[3];
 unsigned short flag;
 short tc[2];
 unsigned char cn[4];
} Vtx_t;
typedef struct {
 short ob[3];
 unsigned short flag;
 short tc[2];
 signed char n[3];
 unsigned char a;
} Vtx_tn;
typedef union {
    Vtx_t v;
    Vtx_tn n;
    long long int force_structure_alignment;
} Vtx;
typedef struct {
  void *SourceImagePointer;
  void *TlutPointer;
  short Stride;
  short SubImageWidth;
  short SubImageHeight;
  char SourceImageType;
  char SourceImageBitSize;
  short SourceImageOffsetS;
  short SourceImageOffsetT;
  char dummy[4];
} uSprite_t;
typedef union {
  uSprite_t s;
  long long int force_structure_allignment[3];
} uSprite;
typedef struct {
 unsigned char flag;
 unsigned char v[3];
} Tri;
typedef long Mtx_t[4][4];
typedef union {
    Mtx_t m;
    long long int force_structure_alignment;
} Mtx;
typedef struct {
 short vscale[4];
 short vtrans[4];
} Vp_t;
typedef union {
    Vp_t vp;
    long long int force_structure_alignment;
} Vp;
typedef struct {
  unsigned char col[3];
  char pad1;
  unsigned char colc[3];
  char pad2;
  signed char dir[3];
  char pad3;
} Light_t;
typedef struct {
    unsigned char col[3];
    unsigned char kc;
    unsigned char colc[3];
    unsigned char kl;
    short pos[3];
    unsigned char kq;
} PointLight_t;
typedef struct {
  unsigned char col[3];
  char pad1;
  unsigned char colc[3];
  char pad2;
} Ambient_t;
typedef struct {
  int x1,y1,x2,y2;
} Hilite_t;
typedef union {
    Light_t l;
    PointLight_t p;
    long long int force_structure_alignment[2];
} Light;
typedef union {
    Ambient_t l;
    long long int force_structure_alignment[1];
} Ambient;
typedef struct {
    Ambient a;
    Light l[7];
} Lightsn;
typedef struct {
    Ambient a;
    Light l[1];
} Lights0;
typedef struct {
    Ambient a;
    Light l[1];
} Lights1;
typedef struct {
    Ambient a;
    Light l[2];
} Lights2;
typedef struct {
    Ambient a;
    Light l[3];
} Lights3;
typedef struct {
    Ambient a;
    Light l[4];
} Lights4;
typedef struct {
    Ambient a;
    Light l[5];
} Lights5;
typedef struct {
    Ambient a;
    Light l[6];
} Lights6;
typedef struct {
    Ambient a;
    Light l[7];
} Lights7;
typedef struct {
    Light l[2];
} LookAt;
typedef union {
    Hilite_t h;
    long int force_structure_alignment[4];
} Hilite;
typedef struct {
 int cmd:8;
 unsigned int par:8;
 unsigned int len:16;
 unsigned int addr;
} Gdma;
typedef struct {
  int cmd:8;
  int pad:24;
  Tri tri;
} Gtri;
typedef struct {
  int cmd:8;
  int pad1:24;
  int pad2:24;
  unsigned char param:8;
} Gpopmtx;
typedef struct {
  int cmd:8;
  int pad0:8;
  int mw_index:8;
  int number:8;
  int pad1:8;
  int base:24;
} Gsegment;
typedef struct {
  int cmd:8;
  int pad0:8;
  int sft:8;
  int len:8;
  unsigned int data:32;
} GsetothermodeL;
typedef struct {
  int cmd:8;
  int pad0:8;
  int sft:8;
  int len:8;
  unsigned int data:32;
} GsetothermodeH;
typedef struct {
  unsigned char cmd;
  unsigned char lodscale;
  unsigned char tile;
  unsigned char on;
  unsigned short s;
  unsigned short t;
} Gtexture;
typedef struct {
  int cmd:8;
  int pad:24;
  Tri line;
} Gline3D;
typedef struct {
  int cmd:8;
  int pad1:24;
  short int pad2;
  short int scale;
} Gperspnorm;
typedef struct {
                int cmd:8;
                unsigned int fmt:3;
                unsigned int siz:2;
                unsigned int pad:7;
                unsigned int wd:12;
                unsigned int dram;
} Gsetimg;
typedef struct {
  int cmd:8;
  unsigned int muxs0:24;
  unsigned int muxs1:32;
} Gsetcombine;
typedef struct {
  int cmd:8;
  unsigned char pad;
  unsigned char prim_min_level;
  unsigned char prim_level;
  unsigned long color;
} Gsetcolor;
typedef struct {
  int cmd:8;
  int x0:10;
  int x0frac:2;
  int y0:10;
  int y0frac:2;
  unsigned int pad:8;
  int x1:10;
  int x1frac:2;
  int y1:10;
  int y1frac:2;
} Gfillrect;
typedef struct {
  int cmd:8;
  unsigned int fmt:3;
  unsigned int siz:2;
  unsigned int pad0:1;
  unsigned int line:9;
  unsigned int tmem:9;
  unsigned int pad1:5;
  unsigned int tile:3;
  unsigned int palette:4;
  unsigned int ct:1;
  unsigned int mt:1;
  unsigned int maskt:4;
  unsigned int shiftt:4;
  unsigned int cs:1;
  unsigned int ms:1;
  unsigned int masks:4;
  unsigned int shifts:4;
} Gsettile;
typedef struct {
  int cmd:8;
  unsigned int sl:12;
  unsigned int tl:12;
  int pad:5;
  unsigned int tile:3;
  unsigned int sh:12;
  unsigned int th:12;
} Gloadtile;
typedef Gloadtile Gloadblock;
typedef Gloadtile Gsettilesize;
typedef Gloadtile Gloadtlut;
typedef struct {
  unsigned int cmd:8;
  unsigned int xl:12;
  unsigned int yl:12;
  unsigned int pad1:5;
  unsigned int tile:3;
  unsigned int xh:12;
  unsigned int yh:12;
  unsigned int s:16;
  unsigned int t:16;
  unsigned int dsdx:16;
  unsigned int dtdy:16;
} Gtexrect;
typedef struct {
    unsigned long w0;
    unsigned long w1;
    unsigned long w2;
    unsigned long w3;
} TexRect;
typedef struct {
 unsigned int w0;
 unsigned int w1;
} Gwords;
typedef union {
 Gwords words;
 Gdma dma;
 Gtri tri;
 Gline3D line;
 Gpopmtx popmtx;
 Gsegment segment;
 GsetothermodeH setothermodeH;
 GsetothermodeL setothermodeL;
 Gtexture texture;
 Gperspnorm perspnorm;
 Gsetimg setimg;
 Gsetcombine setcombine;
 Gsetcolor setcolor;
 Gfillrect fillrect;
 Gsettile settile;
 Gloadtile loadtile;
 Gsettilesize settilesize;
 Gloadtlut loadtlut;
        long long int force_structure_alignment;
} Gfx;
typedef struct {
   unsigned int cmd:8;
 unsigned int flags:8;
 unsigned int gain:16;
 unsigned int addr;
} Aadpcm;
typedef struct {
   unsigned int cmd:8;
 unsigned int flags:8;
 unsigned int gain:16;
 unsigned int addr;
} Apolef;
typedef struct {
   unsigned int cmd:8;
 unsigned int flags:8;
 unsigned int pad1:16;
 unsigned int addr;
} Aenvelope;
typedef struct {
   unsigned int cmd:8;
 unsigned int pad1:8;
 unsigned int dmem:16;
 unsigned int pad2:16;
 unsigned int count:16;
} Aclearbuff;
typedef struct {
   unsigned int cmd:8;
 unsigned int pad1:8;
 unsigned int pad2:16;
 unsigned int inL:16;
        unsigned int inR:16;
} Ainterleave;
typedef struct {
   unsigned int cmd:8;
 unsigned int pad1:24;
 unsigned int addr;
} Aloadbuff;
typedef struct {
   unsigned int cmd:8;
 unsigned int flags:8;
 unsigned int pad1:16;
 unsigned int addr;
} Aenvmixer;
typedef struct {
   unsigned int cmd:8;
 unsigned int flags:8;
 unsigned int gain:16;
 unsigned int dmemi:16;
 unsigned int dmemo:16;
} Amixer;
typedef struct {
   unsigned int cmd:8;
 unsigned int flags:8;
 unsigned int dmem2:16;
 unsigned int addr;
} Apan;
typedef struct {
   unsigned int cmd:8;
 unsigned int flags:8;
 unsigned int pitch:16;
 unsigned int addr;
} Aresample;
typedef struct {
   unsigned int cmd:8;
 unsigned int flags:8;
 unsigned int pad1:16;
 unsigned int addr;
} Areverb;
typedef struct {
   unsigned int cmd:8;
 unsigned int pad1:24;
 unsigned int addr;
} Asavebuff;
typedef struct {
   unsigned int cmd:8;
 unsigned int pad1:24;
 unsigned int pad2:2;
 unsigned int number:4;
 unsigned int base:24;
} Asegment;
typedef struct {
   unsigned int cmd:8;
 unsigned int flags:8;
 unsigned int dmemin:16;
 unsigned int dmemout:16;
 unsigned int count:16;
} Asetbuff;
typedef struct {
   unsigned int cmd:8;
 unsigned int flags:8;
 unsigned int vol:16;
 unsigned int voltgt:16;
 unsigned int volrate:16;
} Asetvol;
typedef struct {
    unsigned int cmd:8;
    unsigned int pad1:8;
    unsigned int dmemin:16;
    unsigned int dmemout:16;
    unsigned int count:16;
} Admemmove;
typedef struct {
    unsigned int cmd:8;
    unsigned int pad1:8;
    unsigned int count:16;
    unsigned int addr;
} Aloadadpcm;
typedef struct {
    unsigned int cmd:8;
    unsigned int pad1:8;
    unsigned int pad2:16;
    unsigned int addr;
} Asetloop;
typedef struct {
 unsigned int w0;
 unsigned int w1;
} Awords;
typedef union {
 Awords words;
 Aadpcm adpcm;
        Apolef polef;
 Aclearbuff clearbuff;
 Aenvelope envelope;
        Ainterleave interleave;
 Aloadbuff loadbuff;
        Aenvmixer envmixer;
 Aresample resample;
 Areverb reverb;
 Asavebuff savebuff;
 Asegment segment;
 Asetbuff setbuff;
 Asetvol setvol;
        Admemmove dmemmove;
        Aloadadpcm loadadpcm;
        Amixer mixer;
        Asetloop setloop;
        long long int force_union_align;
} Acmd;
typedef short ADPCM_STATE[16];
typedef short POLEF_STATE[4];
typedef short RESAMPLE_STATE[16];
typedef short ENVMIX_STATE[40];
typedef s32 ALMicroTime;
typedef u8 ALPan;
typedef struct ALLink_s {
    struct ALLink_s *next;
    struct ALLink_s *prev;
} ALLink;
void alUnlink(ALLink *element);
void alLink(ALLink *element, ALLink *after);
typedef s32 (*ALDMAproc)(s32 addr, s32 len, void *state);
typedef ALDMAproc (*ALDMANew)(void *state);
void alCopy(void *src, void *dest, s32 len);
typedef struct {
    u8 *base;
    u8 *cur;
    s32 len;
    s32 count;
} ALHeap;
void alHeapInit(ALHeap *hp, u8 *base, s32 len);
void *alHeapDBAlloc(u8 *file, s32 line, ALHeap *hp, s32 num, s32 size);
s32 alHeapCheck(ALHeap *hp);
typedef u8 ALFxId;
typedef void *ALFxRef;
enum {AL_ADPCM_WAVE = 0,
         AL_RAW16_WAVE};
typedef struct {
    s32 order;
    s32 npredictors;
    s16 book[1];
} ALADPCMBook;
typedef struct {
    u32 start;
    u32 end;
    u32 count;
    ADPCM_STATE state;
} ALADPCMloop;
typedef struct {
    u32 start;
    u32 end;
    u32 count;
} ALRawLoop;
typedef struct {
    ALMicroTime attackTime;
    ALMicroTime decayTime;
    ALMicroTime releaseTime;
    u8 attackVolume;
    u8 decayVolume;
} ALEnvelope;
typedef struct {
    u8 velocityMin;
    u8 velocityMax;
    u8 keyMin;
    u8 keyMax;
    u8 keyBase;
    s8 detune;
} ALKeyMap;
typedef struct {
    ALADPCMloop *loop;
    ALADPCMBook *book;
} ALADPCMWaveInfo;
typedef struct {
    ALRawLoop *loop;
} ALRAWWaveInfo;
typedef struct ALWaveTable_s {
    u8 *base;
    s32 len;
    u8 type;
    u8 flags;
    union {
        ALADPCMWaveInfo adpcmWave;
        ALRAWWaveInfo rawWave;
    } waveInfo;
} ALWaveTable;
typedef struct ALSound_s {
    ALEnvelope *envelope;
    ALKeyMap *keyMap;
    ALWaveTable *wavetable;
    ALPan samplePan;
    u8 sampleVolume;
    u8 flags;
} ALSound;
typedef struct {
    u8 volume;
    ALPan pan;
    u8 priority;
    u8 flags;
    u8 tremType;
    u8 tremRate;
    u8 tremDepth;
    u8 tremDelay;
    u8 vibType;
    u8 vibRate;
    u8 vibDepth;
    u8 vibDelay;
    s16 bendRange;
    s16 soundCount;
    ALSound *soundArray[1];
} ALInstrument;
typedef struct ALBank_s {
    s16 instCount;
    u8 flags;
    u8 pad;
    s32 sampleRate;
    ALInstrument *percussion;
    ALInstrument *instArray[1];
} ALBank;
typedef struct {
    s16 revision;
    s16 bankCount;
    ALBank *bankArray[1];
} ALBankFile;
void alBnkfNew(ALBankFile *f, u8 *table);
typedef struct {
    u8 *offset;
    s32 len;
} ALSeqData;
typedef struct {
    s16 revision;
    s16 seqCount;
    ALSeqData seqArray[1];
} ALSeqFile;
void alSeqFileNew(ALSeqFile *f, u8 *base);
typedef ALMicroTime (*ALVoiceHandler)(void *);
typedef struct {
    s32 maxVVoices;
    s32 maxPVoices;
    s32 maxUpdates;
    s32 maxFXbusses;
    void *dmaproc;
    ALHeap *heap;
    s32 outputRate;
    ALFxId fxType;
    s32 *params;
} ALSynConfig;
typedef struct ALPlayer_s {
    struct ALPlayer_s *next;
    void *clientData;
    ALVoiceHandler handler;
    ALMicroTime callTime;
    s32 samplesLeft;
} ALPlayer;
typedef struct ALVoice_s {
    ALLink node;
    struct PVoice_s *pvoice;
    ALWaveTable *table;
    void *clientPrivate;
    s16 state;
    s16 priority;
    s16 fxBus;
    s16 unityPitch;
} ALVoice;
typedef struct ALVoiceConfig_s {
    s16 priority;
    s16 fxBus;
    u8 unityPitch;
} ALVoiceConfig;
typedef struct {
    ALPlayer *head;
    ALLink pFreeList;
    ALLink pAllocList;
    ALLink pLameList;
    s32 paramSamples;
    s32 curSamples;
    ALDMANew dma;
    ALHeap *heap;
    struct ALParam_s *paramList;
    struct ALMainBus_s *mainBus;
    struct ALAuxBus_s *auxBus;
    struct ALFilter_s *outputFilter;
    s32 numPVoices;
    s32 maxAuxBusses;
    s32 outputRate;
    s32 maxOutSamples;
} ALSynth;
void alSynNew(ALSynth *s, ALSynConfig *config);
void alSynDelete(ALSynth *s);
void alSynAddPlayer(ALSynth *s, ALPlayer *client);
void alSynRemovePlayer(ALSynth *s, ALPlayer *client);
s32 alSynAllocVoice(ALSynth *s, ALVoice *v, ALVoiceConfig *vc);
void alSynFreeVoice(ALSynth *s, ALVoice *voice);
void alSynStartVoice(ALSynth *s, ALVoice *voice, ALWaveTable *w);
void alSynStartVoiceParams(ALSynth *s, ALVoice *voice, ALWaveTable *w,
                              f32 pitch, s16 vol, ALPan pan, u8 fxmix,
                              ALMicroTime t);
void alSynStopVoice(ALSynth *s, ALVoice *voice);
void alSynSetVol(ALSynth *s, ALVoice *v, s16 vol, ALMicroTime delta);
void alSynSetPitch(ALSynth *s, ALVoice *voice, f32 ratio);
void alSynSetPan(ALSynth *s, ALVoice *voice, ALPan pan);
void alSynSetFXMix(ALSynth *s, ALVoice *voice, u8 fxmix);
void alSynSetPriority(ALSynth *s, ALVoice *voice, s16 priority);
s16 alSynGetPriority(ALSynth *s, ALVoice *voice);
ALFxRef *alSynAllocFX(ALSynth *s, s16 bus, ALSynConfig *c, ALHeap *hp);
ALFxRef alSynGetFXRef(ALSynth *s, s16 bus, s16 index);
void alSynFreeFX(ALSynth *s, ALFxRef *fx);
void alSynSetFXParam(ALSynth *s, ALFxRef fx, s16 paramID, void *param);
typedef struct {
    ALSynth drvr;
} ALGlobals;
extern ALGlobals *alGlobals;
void alInit(ALGlobals *glob, ALSynConfig *c);
void alClose(ALGlobals *glob);
Acmd *alAudioFrame(Acmd *cmdList, s32 *cmdLen, s16 *outBuf, s32 outLen);
enum ALMsg {
    AL_SEQ_REF_EVT,
    AL_SEQ_MIDI_EVT,
    AL_SEQP_MIDI_EVT,
    AL_TEMPO_EVT,
    AL_SEQ_END_EVT,
    AL_NOTE_END_EVT,
    AL_SEQP_ENV_EVT,
    AL_SEQP_META_EVT,
    AL_SEQP_PROG_EVT,
    AL_SEQP_API_EVT,
    AL_SEQP_VOL_EVT,
    AL_SEQP_LOOP_EVT,
    AL_SEQP_PRIORITY_EVT,
    AL_SEQP_SEQ_EVT,
    AL_SEQP_BANK_EVT,
    AL_SEQP_PLAY_EVT,
    AL_SEQP_STOP_EVT,
    AL_SEQP_STOPPING_EVT,
    AL_TRACK_END,
    AL_CSP_LOOPSTART,
    AL_CSP_LOOPEND,
    AL_CSP_NOTEOFF_EVT,
    AL_TREM_OSC_EVT,
    AL_VIB_OSC_EVT
};
enum AL_MIDIstatus {
    AL_MIDI_ChannelMask = 0x0F,
    AL_MIDI_StatusMask = 0xF0,
    AL_MIDI_ChannelVoice = 0x80,
    AL_MIDI_NoteOff = 0x80,
    AL_MIDI_NoteOn = 0x90,
    AL_MIDI_PolyKeyPressure = 0xA0,
    AL_MIDI_ControlChange = 0xB0,
    AL_MIDI_ChannelModeSelect = 0xB0,
    AL_MIDI_ProgramChange = 0xC0,
    AL_MIDI_ChannelPressure = 0xD0,
    AL_MIDI_PitchBendChange = 0xE0,
    AL_MIDI_SysEx = 0xF0,
    AL_MIDI_SystemCommon = 0xF1,
    AL_MIDI_TimeCodeQuarterFrame = 0xF1,
    AL_MIDI_SongPositionPointer = 0xF2,
    AL_MIDI_SongSelect = 0xF3,
    AL_MIDI_Undefined1 = 0xF4,
    AL_MIDI_Undefined2 = 0xF5,
    AL_MIDI_TuneRequest = 0xF6,
    AL_MIDI_EOX = 0xF7,
    AL_MIDI_SystemRealTime = 0xF8,
    AL_MIDI_TimingClock = 0xF8,
    AL_MIDI_Undefined3 = 0xF9,
    AL_MIDI_Start = 0xFA,
    AL_MIDI_Continue = 0xFB,
    AL_MIDI_Stop = 0xFC,
    AL_MIDI_Undefined4 = 0xFD,
    AL_MIDI_ActiveSensing = 0xFE,
    AL_MIDI_SystemReset = 0xFF,
    AL_MIDI_Meta = 0xFF
};
enum AL_MIDIctrl {
    AL_MIDI_VOLUME_CTRL = 0x07,
    AL_MIDI_PAN_CTRL = 0x0A,
    AL_MIDI_PRIORITY_CTRL = 0x10,
    AL_MIDI_FX_CTRL_0 = 0x14,
    AL_MIDI_FX_CTRL_1 = 0x15,
    AL_MIDI_FX_CTRL_2 = 0x16,
    AL_MIDI_FX_CTRL_3 = 0x17,
    AL_MIDI_FX_CTRL_4 = 0x18,
    AL_MIDI_FX_CTRL_5 = 0x19,
    AL_MIDI_FX_CTRL_6 = 0x1A,
    AL_MIDI_FX_CTRL_7 = 0x1B,
    AL_MIDI_FX_CTRL_8 = 0x1C,
    AL_MIDI_FX_CTRL_9 = 0x1D,
    AL_MIDI_SUSTAIN_CTRL = 0x40,
    AL_MIDI_FX1_CTRL = 0x5B,
    AL_MIDI_FX3_CTRL = 0x5D
};
enum AL_MIDImeta {
    AL_MIDI_META_TEMPO = 0x51,
    AL_MIDI_META_EOT = 0x2f
};
typedef struct {
    u8 *curPtr;
    s32 lastTicks;
    s32 curTicks;
    s16 lastStatus;
} ALSeqMarker;
typedef struct {
    s32 ticks;
    u8 status;
    u8 byte1;
    u8 byte2;
    u32 duration;
} ALMIDIEvent;
typedef struct {
    s32 ticks;
    u8 status;
    u8 type;
    u8 len;
    u8 byte1;
    u8 byte2;
    u8 byte3;
} ALTempoEvent;
typedef struct {
    s32 ticks;
    u8 status;
    u8 type;
    u8 len;
} ALEndEvent;
typedef struct {
    struct ALVoice_s *voice;
} ALNoteEvent;
typedef struct {
    struct ALVoice_s *voice;
    ALMicroTime delta;
    u8 vol;
} ALVolumeEvent;
typedef struct {
    s16 vol;
} ALSeqpVolEvent;
typedef struct {
    ALSeqMarker *start;
    ALSeqMarker *end;
    s32 count;
} ALSeqpLoopEvent;
typedef struct {
    u8 chan;
    u8 priority;
} ALSeqpPriorityEvent;
typedef struct {
    void *seq;
} ALSeqpSeqEvent;
typedef struct {
    ALBank *bank;
} ALSeqpBankEvent;
typedef struct {
    struct ALVoiceState_s *vs;
    void *oscState;
    u8 chan;
} ALOscEvent;
typedef struct {
    s16 type;
    union {
        ALMIDIEvent midi;
        ALTempoEvent tempo;
        ALEndEvent end;
        ALNoteEvent note;
        ALVolumeEvent vol;
        ALSeqpLoopEvent loop;
        ALSeqpVolEvent spvol;
 ALSeqpPriorityEvent sppriority;
 ALSeqpSeqEvent spseq;
 ALSeqpBankEvent spbank;
        ALOscEvent osc;
    } msg;
} ALEvent;
typedef struct {
    ALLink node;
    ALMicroTime delta;
    ALEvent evt;
} ALEventListItem;
typedef struct {
    ALLink freeList;
    ALLink allocList;
    s32 eventCount;
} ALEventQueue;
void alEvtqNew(ALEventQueue *evtq, ALEventListItem *items,
                          s32 itemCount);
ALMicroTime alEvtqNextEvent(ALEventQueue *evtq, ALEvent *evt);
void alEvtqPostEvent(ALEventQueue *evtq, ALEvent *evt,
                                ALMicroTime delta);
void alEvtqFlush(ALEventQueue *evtq);
void alEvtqFlushType(ALEventQueue *evtq, s16 type);
typedef struct ALVoiceState_s {
    struct ALVoiceState_s *next;
    ALVoice voice;
    ALSound *sound;
    ALMicroTime envEndTime;
    f32 pitch;
    f32 vibrato;
    u8 envGain;
    u8 channel;
    u8 key;
    u8 velocity;
    u8 envPhase;
    u8 phase;
    u8 tremelo;
    u8 flags;
} ALVoiceState;
typedef struct {
    ALInstrument *instrument;
    s16 bendRange;
    ALFxId fxId;
    ALPan pan;
    u8 priority;
    u8 vol;
    u8 fxmix;
    u8 sustain;
    f32 pitchBend;
} ALChanState;
typedef struct ALSeq_s {
    u8 *base;
    u8 *trackStart;
    u8 *curPtr;
    s32 lastTicks;
    s32 len;
    f32 qnpt;
    s16 division;
    s16 lastStatus;
} ALSeq;
typedef struct {
    u32 trackOffset[16];
    u32 division;
} ALCMidiHdr;
typedef struct ALCSeq_s {
    ALCMidiHdr *base;
    u32 validTracks;
    f32 qnpt;
    u32 lastTicks;
    u32 lastDeltaTicks;
    u32 deltaFlag;
    u8 *curLoc[16];
    u8 *curBUPtr[16];
    u8 curBULen[16];
    u8 lastStatus[16];
    u32 evtDeltaTicks[16];
} ALCSeq;
typedef struct {
    u32 validTracks;
    s32 lastTicks;
    u32 lastDeltaTicks;
    u8 *curLoc[16];
    u8 *curBUPtr[16];
    u8 curBULen[16];
    u8 lastStatus[16];
    u32 evtDeltaTicks[16];
} ALCSeqMarker;
typedef struct {
    s32 maxVoices;
    s32 maxEvents;
    u8 maxChannels;
    u8 debugFlags;
    ALHeap *heap;
    void *initOsc;
    void *updateOsc;
    void *stopOsc;
} ALSeqpConfig;
typedef ALMicroTime (*ALOscInit)(void **oscState,f32 *initVal, u8 oscType,
                                   u8 oscRate, u8 oscDepth, u8 oscDelay);
typedef ALMicroTime (*ALOscUpdate)(void *oscState, f32 *updateVal);
typedef void (*ALOscStop)(void *oscState);
typedef struct {
    ALPlayer node;
    ALSynth *drvr;
    ALSeq *target;
    ALMicroTime curTime;
    ALBank *bank;
    s32 uspt;
    s32 nextDelta;
    s32 state;
    u16 chanMask;
    s16 vol;
    u8 maxChannels;
    u8 debugFlags;
    ALEvent nextEvent;
    ALEventQueue evtq;
    ALMicroTime frameTime;
    ALChanState *chanState;
    ALVoiceState *vAllocHead;
    ALVoiceState *vAllocTail;
    ALVoiceState *vFreeList;
    ALOscInit initOsc;
    ALOscUpdate updateOsc;
    ALOscStop stopOsc;
    ALSeqMarker *loopStart;
    ALSeqMarker *loopEnd;
    s32 loopCount;
} ALSeqPlayer;
typedef struct {
    ALPlayer node;
    ALSynth *drvr;
    ALCSeq *target;
    ALMicroTime curTime;
    ALBank *bank;
    s32 uspt;
    s32 nextDelta;
    s32 state;
    u16 chanMask;
    s16 vol;
    u8 maxChannels;
    u8 debugFlags;
    ALEvent nextEvent;
    ALEventQueue evtq;
    ALMicroTime frameTime;
    ALChanState *chanState;
    ALVoiceState *vAllocHead;
    ALVoiceState *vAllocTail;
    ALVoiceState *vFreeList;
    ALOscInit initOsc;
    ALOscUpdate updateOsc;
    ALOscStop stopOsc;
} ALCSPlayer;
void alSeqNew(ALSeq *seq, u8 *ptr, s32 len);
void alSeqNextEvent(ALSeq *seq, ALEvent *event);
s32 alSeqGetTicks(ALSeq *seq);
f32 alSeqTicksToSec(ALSeq *seq, s32 ticks, u32 tempo);
u32 alSeqSecToTicks(ALSeq *seq, f32 sec, u32 tempo);
void alSeqNewMarker(ALSeq *seq, ALSeqMarker *m, u32 ticks);
void alSeqSetLoc(ALSeq *seq, ALSeqMarker *marker);
void alSeqGetLoc(ALSeq *seq, ALSeqMarker *marker);
void alCSeqNew(ALCSeq *seq, u8 *ptr);
void alCSeqNextEvent(ALCSeq *seq,ALEvent *evt);
s32 alCSeqGetTicks(ALCSeq *seq);
f32 alCSeqTicksToSec(ALCSeq *seq, s32 ticks, u32 tempo);
u32 alCSeqSecToTicks(ALCSeq *seq, f32 sec, u32 tempo);
void alCSeqNewMarker(ALCSeq *seq, ALCSeqMarker *m, u32 ticks);
void alCSeqSetLoc(ALCSeq *seq, ALCSeqMarker *marker);
void alCSeqGetLoc(ALCSeq *seq, ALCSeqMarker *marker);
f32 alCents2Ratio(s32 cents);
void alSeqpNew(ALSeqPlayer *seqp, ALSeqpConfig *config);
void alSeqpDelete(ALSeqPlayer *seqp);
void alSeqpSetSeq(ALSeqPlayer *seqp, ALSeq *seq);
ALSeq *alSeqpGetSeq(ALSeqPlayer *seqp);
void alSeqpPlay(ALSeqPlayer *seqp);
void alSeqpStop(ALSeqPlayer *seqp);
s32 alSeqpGetState(ALSeqPlayer *seqp);
void alSeqpSetBank(ALSeqPlayer *seqp, ALBank *b);
void alSeqpSetTempo(ALSeqPlayer *seqp, s32 tempo);
s32 alSeqpGetTempo(ALSeqPlayer *seqp);
s16 alSeqpGetVol(ALSeqPlayer *seqp);
void alSeqpSetVol(ALSeqPlayer *seqp, s16 vol);
void alSeqpLoop(ALSeqPlayer *seqp, ALSeqMarker *start, ALSeqMarker *end, s32 count);
void alSeqpSetChlProgram(ALSeqPlayer *seqp, u8 chan, u8 prog);
s32 alSeqpGetChlProgram(ALSeqPlayer *seqp, u8 chan);
void alSeqpSetChlFXMix(ALSeqPlayer *seqp, u8 chan, u8 fxmix);
u8 alSeqpGetChlFXMix(ALSeqPlayer *seqp, u8 chan);
void alSeqpSetChlVol(ALSeqPlayer *seqp, u8 chan, u8 vol);
u8 alSeqpGetChlVol(ALSeqPlayer *seqp, u8 chan);
void alSeqpSetChlPan(ALSeqPlayer *seqp, u8 chan, ALPan pan);
ALPan alSeqpGetChlPan(ALSeqPlayer *seqp, u8 chan);
void alSeqpSetChlPriority(ALSeqPlayer *seqp, u8 chan, u8 priority);
u8 alSeqpGetChlPriority(ALSeqPlayer *seqp, u8 chan);
void alSeqpSendMidi(ALSeqPlayer *seqp, s32 ticks, u8 status, u8 byte1, u8 byte2);
void alCSPNew(ALCSPlayer *seqp, ALSeqpConfig *config);
void alCSPDelete(ALCSPlayer *seqp);
void alCSPSetSeq(ALCSPlayer *seqp, ALCSeq *seq);
ALCSeq *alCSPGetSeq(ALCSPlayer *seqp);
void alCSPPlay(ALCSPlayer *seqp);
void alCSPStop(ALCSPlayer *seqp);
s32 alCSPGetState(ALCSPlayer *seqp);
void alCSPSetBank(ALCSPlayer *seqp, ALBank *b);
void alCSPSetTempo(ALCSPlayer *seqp, s32 tempo);
s32 alCSPGetTempo(ALCSPlayer *seqp);
s16 alCSPGetVol(ALCSPlayer *seqp);
void alCSPSetVol(ALCSPlayer *seqp, s16 vol);
void alCSPSetChlProgram(ALCSPlayer *seqp, u8 chan, u8 prog);
s32 alCSPGetChlProgram(ALCSPlayer *seqp, u8 chan);
void alCSPSetChlFXMix(ALCSPlayer *seqp, u8 chan, u8 fxmix);
u8 alCSPGetChlFXMix(ALCSPlayer *seqp, u8 chan);
void alCSPSetChlPan(ALCSPlayer *seqp, u8 chan, ALPan pan);
ALPan alCSPGetChlPan(ALCSPlayer *seqp, u8 chan);
void alCSPSetChlVol(ALCSPlayer *seqp, u8 chan, u8 vol);
u8 alCSPGetChlVol(ALCSPlayer *seqp, u8 chan);
void alCSPSetChlPriority(ALCSPlayer *seqp, u8 chan, u8 priority);
u8 alCSPGetChlPriority(ALCSPlayer *seqp, u8 chan);
void alCSPSendMidi(ALCSPlayer *seqp, s32 ticks, u8 status,
                       u8 byte1, u8 byte2);
typedef struct {
    s32 maxSounds;
    s32 maxEvents;
    ALHeap *heap;
} ALSndpConfig;
typedef struct {
    ALPlayer node;
    ALEventQueue evtq;
    ALEvent nextEvent;
    ALSynth *drvr;
    s32 target;
    void *sndState;
    s32 maxSounds;
    ALMicroTime frameTime;
    ALMicroTime nextDelta;
    ALMicroTime curTime;
} ALSndPlayer;
typedef s16 ALSndId;
void alSndpNew(ALSndPlayer *sndp, ALSndpConfig *c);
void alSndpDelete(ALSndPlayer *sndp);
ALSndId alSndpAllocate(ALSndPlayer *sndp, ALSound *sound);
void alSndpDeallocate(ALSndPlayer *sndp, ALSndId id);
void alSndpSetSound(ALSndPlayer *sndp, ALSndId id);
ALSndId alSndpGetSound(ALSndPlayer *sndp);
void alSndpPlay(ALSndPlayer *sndp);
void alSndpPlayAt(ALSndPlayer *sndp, ALMicroTime delta);
void alSndpStop(ALSndPlayer *sndp);
void alSndpSetVol(ALSndPlayer *sndp, s16 vol);
void alSndpSetPitch(ALSndPlayer *sndp, f32 pitch);
void alSndpSetPan(ALSndPlayer *sndp, ALPan pan);
void alSndpSetPriority(ALSndPlayer *sndp, ALSndId id, u8 priority);
void alSndpSetFXMix(ALSndPlayer *sndp, u8 mix);
s32 alSndpGetState(ALSndPlayer *sndp);
typedef struct {
 unsigned char *base;
 int fmt, siz;
 int xsize, ysize;
 int lsize;
 int addr;
 int w, h;
 int s, t;
} Image;
typedef struct {
 float col[3];
 float pos[3];
 float a1, a2;
} PositionalLight;
extern int guLoadTextureBlockMipMap(Gfx **glist, unsigned char *tbuf, Image *im,
  unsigned char startTile, unsigned char pal, unsigned char cms,
  unsigned char cmt, unsigned char masks, unsigned char maskt,
  unsigned char shifts, unsigned char shiftt, unsigned char cfs,
  unsigned char cft);
extern int guGetDPLoadTextureTileSz (int ult, int lrt);
extern void guDPLoadTextureTile (Gfx *glistp, void *timg,
   int texl_fmt, int texl_size,
   int img_width, int img_height,
   int uls, int ult, int lrs, int lrt,
   int palette,
   int cms, int cmt,
   int masks, int maskt,
   int shifts, int shiftt);
extern void guMtxIdent(Mtx *m);
extern void guMtxIdentF(float mf[4][4]);
extern void guOrtho(Mtx *m, float l, float r, float b, float t,
      float n, float f, float scale);
extern void guOrthoF(float mf[4][4], float l, float r, float b, float t,
       float n, float f, float scale);
extern void guFrustum(Mtx *m, float l, float r, float b, float t,
        float n, float f, float scale);
extern void guFrustumF(float mf[4][4], float l, float r, float b, float t,
         float n, float f, float scale);
extern void guPerspective(Mtx *m, u16 *perspNorm, float fovy,
     float aspect, float near, float far, float scale);
extern void guPerspectiveF(float mf[4][4], u16 *perspNorm, float fovy,
      float aspect, float near, float far, float scale);
extern void guLookAt(Mtx *m,
   float xEye, float yEye, float zEye,
   float xAt, float yAt, float zAt,
   float xUp, float yUp, float zUp);
extern void guLookAtF(float mf[4][4], float xEye, float yEye, float zEye,
        float xAt, float yAt, float zAt,
        float xUp, float yUp, float zUp);
extern void guLookAtReflect(Mtx *m, LookAt *l,
   float xEye, float yEye, float zEye,
   float xAt, float yAt, float zAt,
   float xUp, float yUp, float zUp);
extern void guLookAtReflectF(float mf[4][4], LookAt *l,
        float xEye, float yEye, float zEye,
        float xAt, float yAt, float zAt,
        float xUp, float yUp, float zUp);
extern void guLookAtHilite(Mtx *m, LookAt *l, Hilite *h,
                float xEye, float yEye, float zEye,
                float xAt, float yAt, float zAt,
                float xUp, float yUp, float zUp,
                float xl1, float yl1, float zl1,
                float xl2, float yl2, float zl2,
  int twidth, int theight);
extern void guLookAtHiliteF(float mf[4][4], LookAt *l, Hilite *h,
  float xEye, float yEye, float zEye,
  float xAt, float yAt, float zAt,
  float xUp, float yUp, float zUp,
  float xl1, float yl1, float zl1,
  float xl2, float yl2, float zl2,
  int twidth, int theight);
extern void guLookAtStereo(Mtx *m,
   float xEye, float yEye, float zEye,
   float xAt, float yAt, float zAt,
   float xUp, float yUp, float zUp,
   float eyedist);
extern void guLookAtStereoF(float mf[4][4],
         float xEye, float yEye, float zEye,
         float xAt, float yAt, float zAt,
         float xUp, float yUp, float zUp,
   float eyedist);
extern void guRotate(Mtx *m, float a, float x, float y, float z);
extern void guRotateF(float mf[4][4], float a, float x, float y, float z);
extern void guRotateRPY(Mtx *m, float r, float p, float y);
extern void guRotateRPYF(float mf[4][4], float r, float p, float h);
extern void guAlign(Mtx *m, float a, float x, float y, float z);
extern void guAlignF(float mf[4][4], float a, float x, float y, float z);
extern void guScale(Mtx *m, float x, float y, float z);
extern void guScaleF(float mf[4][4], float x, float y, float z);
extern void guTranslate(Mtx *m, float x, float y, float z);
extern void guTranslateF(float mf[4][4], float x, float y, float z);
extern void guPosition(Mtx *m, float r, float p, float h, float s,
         float x, float y, float z);
extern void guPositionF(float mf[4][4], float r, float p, float h, float s,
   float x, float y, float z);
extern void guMtxF2L(float mf[4][4], Mtx *m);
extern void guMtxL2F(float mf[4][4], Mtx *m);
extern void guMtxCatF(float m[4][4], float n[4][4], float r[4][4]);
extern void guMtxCatL(Mtx *m, Mtx *n, Mtx *res);
extern void guMtxXFMF(float mf[4][4], float x, float y, float z,
        float *ox, float *oy, float *oz);
extern void guMtxXFML(Mtx *m, float x, float y, float z,
        float *ox, float *oy, float *oz);
extern void guNormalize(float *x, float *y, float *z);
void guPosLight(PositionalLight *pl, Light *l,
                float xOb, float yOb, float zOb);
void guPosLightHilite(PositionalLight *pl1, PositionalLight *pl2,
                Light *l1, Light *l2,
                LookAt *l, Hilite *h,
                float xEye, float yEye, float zEye,
                float xOb, float yOb, float zOb,
                float xUp, float yUp, float zUp,
                int twidth, int theight);
extern int guRandom(void);
extern float sinf(float angle);
extern float cosf(float angle);
extern signed short sins (unsigned short angle);
extern signed short coss (unsigned short angle);
extern float sqrtf(float value);
#pragma intrinsic sqrtf
extern void guParseRdpDL(u64 *rdp_dl, u64 nbytes, u8 flags);
extern void guParseString(char *StringPointer, u64 nbytes);
extern void
guBlinkRdpDL(u64 *rdp_dl_in, u64 nbytes_in,
             u64 *rdp_dl_out, u64 *nbytes_out,
             u32 x, u32 y, u32 radius,
             u8 red, u8 green, u8 blue,
             u8 flags);
extern void guParseGbiDL(u64 *gbi_dl, u32 nbytes, u8 flags);
extern void guDumpGbiDL(OSTask *tp,u8 flags);
typedef struct {
    int dataSize;
    int dlType;
    int flags;
    u32 paddr;
} guDLPrintCB;
void guSprite2DInit(uSprite *SpritePointer,
      void *SourceImagePointer,
      void *TlutPointer,
      int Stride,
      int SubImageWidth,
      int SubImageHeight,
      int SourceImageType,
      int SourceImageBitSize,
      int SourceImageOffsetS,
      int SourceImageOffsetT);
typedef struct {
    long type;
    long length;
    long magic;
    char userdata[(((4096)*6)-(3*sizeof(long)))];
} RamRomBuffer;
struct bitmap {
 s16 width;
 s16 width_img;
 s16 s;
 s16 t;
 void *buf;
 s16 actualHeight;
 s16 LUToffset;
};
typedef struct bitmap Bitmap;
struct sprite {
 s16 x,y;
 s16 width, height;
 f32 scalex, scaley;
 s16 expx, expy;
 u16 attr;
 s16 zdepth;
 u8 red;
 u8 green;
 u8 blue;
 u8 alpha;
 s16 startTLUT;
 s16 nTLUT;
 int *LUT;
 s16 istart;
 s16 istep;
 s16 nbitmaps;
 s16 ndisplist;
 s16 bmheight;
 s16 bmHreal;
 u8 bmfmt;
 u8 bmsiz;
 Bitmap *bitmap;
 Gfx *rsp_dl;
 Gfx *rsp_dl_next;
 s16 frac_s,
  frac_t;
};
typedef struct sprite Sprite;
void spSetAttribute (Sprite *sp, s32 attr);
void spClearAttribute (Sprite *sp, s32 attr);
void spX2Move (Sprite *sp, s32 x, s32 y);
void spScale (Sprite *sp, f32 sx, f32 sy);
void spX2SetZ (Sprite *sp, s32 z );
void spColor (Sprite *sp, u8 red, u8 green, u8 blue, u8 alpha);
Gfx *spX2Draw (Sprite *sp);
void spX2Init( Gfx **glistp );
void spX2Scissor( s32 xmin, s32 xmax, s32 ymin, s32 ymax );
void spX2Finish( Gfx **glistp );
extern long long int rspbootTextStart[], rspbootTextEnd[];
extern long long int gspFast3DTextStart[], gspFast3DTextEnd[];
extern long long int gspFast3DDataStart[], gspFast3DDataEnd[];
extern long long int gspFast3D_dramTextStart[], gspFast3D_dramTextEnd[];
extern long long int gspFast3D_dramDataStart[], gspFast3D_dramDataEnd[];
extern long long int gspFast3D_fifoTextStart[], gspFast3D_fifoTextEnd[];
extern long long int gspFast3D_fifoDataStart[], gspFast3D_fifoDataEnd[];
extern long long int gspF3DNoNTextStart[], gspF3DNoNTextEnd[];
extern long long int gspF3DNoNDataStart[], gspF3DNoNDataEnd[];
extern long long int gspF3DNoN_dramTextStart[];
extern long long int gspF3DNoN_dramTextEnd[];
extern long long int gspF3DNoN_dramDataStart[];
extern long long int gspF3DNoN_dramDataEnd[];
extern long long int gspF3DNoN_fifoTextStart[];
extern long long int gspF3DNoN_fifoTextEnd[];
extern long long int gspF3DNoN_fifoDataStart[];
extern long long int gspF3DNoN_fifoDataEnd[];
extern long long int gspLine3DTextStart[], gspLine3DTextEnd[];
extern long long int gspLine3DDataStart[], gspLine3DDataEnd[];
extern long long int gspLine3D_dramTextStart[], gspLine3D_dramTextEnd[];
extern long long int gspLine3D_dramDataStart[], gspLine3D_dramDataEnd[];
extern long long int gspLine3D_fifoTextStart[], gspLine3D_fifoTextEnd[];
extern long long int gspLine3D_fifoDataStart[], gspLine3D_fifoDataEnd[];
extern long long int gspSprite2DTextStart[], gspSprite2DTextEnd[];
extern long long int gspSprite2DDataStart[], gspSprite2DDataEnd[];
extern long long int gspSprite2D_dramTextStart[], gspSprite2D_dramTextEnd[];
extern long long int gspSprite2D_dramDataStart[], gspSprite2D_dramDataEnd[];
extern long long int gspSprite2D_fifoTextStart[], gspSprite2D_fifoTextEnd[];
extern long long int gspSprite2D_fifoDataStart[], gspSprite2D_fifoDataEnd[];
extern long long int aspMainTextStart[], aspMainTextEnd[];
extern long long int aspMainDataStart[], aspMainDataEnd[];
extern long long int gspF3DEX_fifoTextStart[], gspF3DEX_fifoTextEnd[];
extern long long int gspF3DEX_fifoDataStart[], gspF3DEX_fifoDataEnd[];
extern long long int gspF3DEX_NoN_fifoTextStart[], gspF3DEX_NoN_fifoTextEnd[];
extern long long int gspF3DEX_NoN_fifoDataStart[], gspF3DEX_NoN_fifoDataEnd[];
extern long long int gspF3DLX_fifoTextStart[], gspF3DLX_fifoTextEnd[];
extern long long int gspF3DLX_fifoDataStart[], gspF3DLX_fifoDataEnd[];
extern long long int gspF3DLX_NoN_fifoTextStart[], gspF3DLX_NoN_fifoTextEnd[];
extern long long int gspF3DLX_NoN_fifoDataStart[], gspF3DLX_NoN_fifoDataEnd[];
extern long long int gspF3DLX_Rej_fifoTextStart[], gspF3DLX_Rej_fifoTextEnd[];
extern long long int gspF3DLX_Rej_fifoDataStart[], gspF3DLX_Rej_fifoDataEnd[];
extern long long int gspF3DLP_Rej_fifoTextStart[], gspF3DLP_Rej_fifoTextEnd[];
extern long long int gspF3DLP_Rej_fifoDataStart[], gspF3DLP_Rej_fifoDataEnd[];
extern long long int gspL3DEX_fifoTextStart[], gspL3DEX_fifoTextEnd[];
extern long long int gspL3DEX_fifoDataStart[], gspL3DEX_fifoDataEnd[];
extern long long int gspF3DEX2_fifoTextStart[], gspF3DEX2_fifoTextEnd[];
extern long long int gspF3DEX2_fifoDataStart[], gspF3DEX2_fifoDataEnd[];
extern long long int gspF3DEX2_NoN_fifoTextStart[],gspF3DEX2_NoN_fifoTextEnd[];
extern long long int gspF3DEX2_NoN_fifoDataStart[],gspF3DEX2_NoN_fifoDataEnd[];
extern long long int gspF3DEX2_Rej_fifoTextStart[],gspF3DEX2_Rej_fifoTextEnd[];
extern long long int gspF3DEX2_Rej_fifoDataStart[],gspF3DEX2_Rej_fifoDataEnd[];
extern long long int gspF3DLX2_Rej_fifoTextStart[],gspF3DLX2_Rej_fifoTextEnd[];
extern long long int gspF3DLX2_Rej_fifoDataStart[],gspF3DLX2_Rej_fifoDataEnd[];
extern long long int gspL3DEX2_fifoTextStart[], gspL3DEX2_fifoTextEnd[];
extern long long int gspL3DEX2_fifoDataStart[], gspL3DEX2_fifoDataEnd[];
extern long long int gspF3DEX2_xbusTextStart[], gspF3DEX2_xbusTextEnd[];
extern long long int gspF3DEX2_xbusDataStart[], gspF3DEX2_xbusDataEnd[];
extern long long int gspF3DEX2_NoN_xbusTextStart[],gspF3DEX2_NoN_xbusTextEnd[];
extern long long int gspF3DEX2_NoN_xbusDataStart[],gspF3DEX2_NoN_xbusDataEnd[];
extern long long int gspF3DEX2_Rej_xbusTextStart[],gspF3DEX2_Rej_xbusTextEnd[];
extern long long int gspF3DEX2_Rej_xbusDataStart[],gspF3DEX2_Rej_xbusDataEnd[];
extern long long int gspF3DLX2_Rej_xbusTextStart[],gspF3DLX2_Rej_xbusTextEnd[];
extern long long int gspF3DLX2_Rej_xbusDataStart[],gspF3DLX2_Rej_xbusDataEnd[];
extern long long int gspL3DEX2_xbusTextStart[], gspL3DEX2_xbusTextEnd[];
extern long long int gspL3DEX2_xbusDataStart[], gspL3DEX2_xbusDataEnd[];
typedef void (*OSErrorHandler)(s16, s16, ...);
OSErrorHandler osSetErrorHandler(OSErrorHandler);
typedef struct {
    u32 magic;
    u32 len;
    u32 *base;
    s32 startCount;
    s32 writeOffset;
} OSLog;
typedef struct {
    u32 magic;
    u32 timeStamp;
    u16 argCount;
    u16 eventID;
} OSLogItem;
typedef struct {
    u32 magic;
    u32 version;
} OSLogFileHdr;
void osCreateLog(OSLog *log, u32 *base, s32 len);
void osLogEvent(OSLog *log, s16 code, s16 numArgs, ...);
void osFlushLog(OSLog *log);
u32 osLogFloat(f32);
extern void osDelay(int count);
typedef struct {
              s16 x;
              s16 z;
} Vec2s;
typedef struct {
              f32 x;
              f32 z;
} Vec2f;

typedef struct {
              u16 x;
              u16 y;
              u16 z;
} Vec3us;
typedef struct {
              s32 x;
              s32 y;
              s32 z;
} Vec3i;

typedef struct {
    u32 cont:   1;
    u32 type:   4;
    u32 offset: 11;
    s32 value:  16;
} ValueSet;

typedef struct xyz_t{
    f32 x,y,z;
}xyz_t;



typedef struct s_xyz{
    s16 x,y,z;
}s_xyz;

typedef float MtxF_t[4][4];
typedef union {
    MtxF_t mf;
    struct {
        float xx, yx, zx, wx, xy, yy, zy, wy, xz, yz, zz, wz, xw, yw, zw, ww;
    };
} MtxF;
typedef s32 (*UNK_FUN_PTR)();
struct Game_Play;
void mem_copy(u8* dst, u8* src, u32 size);
s32 mem_clear(u8* ptr, size_t size, u8 value);
f32 cos_s(s16 angle);
f32 sin_s(s16 angle);
s32 chase_f(f32* arg0, f32 arg1, f32 arg2);
void xyz_t_move(xyz_t* dst, xyz_t* src);
void xyz_t_move_s_xyz(xyz_t* dst, s_xyz* src);
f32 search_position_distanceXZ(xyz_t* arg0, xyz_t* arg1);
s16 search_position_angleY(xyz_t* arg0, xyz_t* arg1);
f32 add_calc(f32* pValue, f32 target, f32 fraction, f32 maxStep, f32 minStep);
s32 none_proc1(void);
s32 _Game_play_isPause(struct Game_Play* game_play);
void *Lib_SegmentedToVirtual(void *ptr);
f32 check_percent_abs(f32 x, f32 min, f32 max, f32 scale, s32 shift_by_min);
typedef signed int intptr_t;
typedef unsigned int uintptr_t;
typedef struct TwoHeadArena {
               size_t size;
               void* start;
               void* head;
               void* tail;
} TwoHeadArena;
void* THA_getHeadPtr(TwoHeadArena* tha);
void THA_setHeadPtr(TwoHeadArena* tha, void* newHead);
void* THA_getTailPtr(TwoHeadArena* tha);
void* THA_nextPtrN(TwoHeadArena* tha, size_t size);
void* THA_nextPtr1(TwoHeadArena* tha);
void* THA_alloc(TwoHeadArena* tha, size_t size);
void* THA_alloc16(TwoHeadArena* tha, size_t size);
void* THA_allocAlign(TwoHeadArena* tha, size_t size, uintptr_t mask);
s32 THA_getFreeBytes(TwoHeadArena* tha);
u32 THA_isCrash(TwoHeadArena* tha);
void THA_init(TwoHeadArena* tha);
void THA_ct(TwoHeadArena* tha, void* start, size_t size);
void THA_dt(TwoHeadArena* tha);
typedef union TwoHeadGfxArena {
    struct {
                  size_t size;
                  void* start;
                  Gfx* p;
                  void* d;
    };
              TwoHeadArena tha;
} TwoHeadGfxArena;
void THA_GA_ct(TwoHeadGfxArena* thga, void* start, size_t size);
void THGA_dt(TwoHeadGfxArena* thga);
u32 THA_GA_isCrash(TwoHeadGfxArena* thga);
void THA_GA_init(TwoHeadGfxArena* thga);
s32 THA_GA_getFreeBytes(TwoHeadGfxArena* thga);
Gfx* THA_GA_getHeadPtr(TwoHeadGfxArena* thga);
void THA_GA_setHeadPtr(TwoHeadGfxArena* thga, Gfx* newHead);
void* THA_GA_getTailPtr(TwoHeadGfxArena* thga);
Gfx* THA_GA_nextPtrN(TwoHeadGfxArena* thga, size_t num);
Gfx* THA_GA_nextPtr1(TwoHeadGfxArena* thga);
Gfx* THA_GA_NEXT_DISP(TwoHeadGfxArena* thga);
void* THA_GA_alloc(TwoHeadGfxArena* thga, size_t size);
Mtx* THA_GA_allocMtxN(TwoHeadGfxArena* thga, size_t num);
Mtx* THA_GA_allocMtx1(TwoHeadGfxArena* thga);
Vtx* THA_GA_allocVtxN(TwoHeadGfxArena* thga, u32 num);
Vtx* THA_GA_allocVtx1(TwoHeadGfxArena* thga);

typedef struct OSScTask {
    /* 0x00 */ struct OSScTask* next;
    /* 0x04 */ u32              state;
    /* 0x08 */ u32              flags;
    /* 0x0C */ void*            framebuffer;
    /* 0x10 */ OSTask           list;
    /* 0x50 */ OSMesgQueue*     msgQ;
    /* 0x54 */ OSMesg           msg;
} OSScTask; // size = 0x58

typedef struct {
    /* 0x00 */ u16* fb1;
    /* 0x04 */ u16* swapBuffer;
    /* 0x08 */ u8 unk8;
    /* 0x09 */ u8 unk9;
    /* 0x0A */ u8 unkA; 
    /* 0x0B */ u8 unkB;
    /* 0x0C */ OSViMode* viMode;
    /* 0x10 */ u32 unk10;
    /* 0x14 */ f32 xScale;
    /* 0x18 */ f32 yScale;
} CfbInfo; // size = 0x1C

typedef void (*GraphicsCallback)(struct GraphicsContext*, void*);
typedef struct GraphicsContext {
                Gfx* polyOpaBuffer;
                Gfx* polyXluBuffer;
                void* unk_008;
                void* unk_00C;
                Gfx* overlayBuffer;
                Gfx* fontBuffer;
                Gfx* shadowBuffer;
                Gfx* lightBuffer;
                Gfx* gfxSave;
                s8 unk_024[0x20];
                OSMesg graphReplyMesgBuf[8];
                OSMesgQueue* schedMesgQueue;
                OSMesgQueue graphReplyMesgQueue;
                OSScTask task;
                /* 0x0D4 */ char unk_D4[0xB0];
                Gfx* workBuffer;
                TwoHeadGfxArena work;
                s8 unk_190[0xC0]; 
                OSViMode* unk_25C;
                s8 unk_260[0x20];
                TwoHeadGfxArena overlay;
                TwoHeadGfxArena polyOpa;
                TwoHeadGfxArena polyXlu;
                TwoHeadGfxArena font;
                TwoHeadGfxArena shadow;
                TwoHeadGfxArena light;
                s32 frameCounter;
                u16* unk_2E4;
                s8 unk_2E8[0x4];
                u32 unk_2EC;
                u8 unk_2F0;
                s8 unk_2F1[0x1];
                u8 unk_2F2;
                u8 cfbBank;
                GraphicsCallback unk_2F4;
                void* unk_2F8; 
                f32 unk_2FC;
                f32 unk_300;
                Gfx* unk_304;
}GraphicsContext;
typedef struct Sphere {
              s_xyz center;
              s16 radius;
} Sphere;
typedef struct Triangle {
               xyz_t vertices[3];
} Triangle;
typedef struct Triangle3 {
               Triangle unk_00;
               xyz_t unk_24;
               f32 unk_30;
} Triangle3;
typedef struct Pipe {
              s16 radius;
              s16 unk_2;
              s16 yShift;
              s_xyz pos;
} Pipe;
void Math3DPlane(xyz_t* arg0, xyz_t* arg1, xyz_t* arg2, f32* arg3, f32* arg4, f32* arg5, f32* arg6);
s32 Math3D_sphereCrossTriangle3_cp(Sphere* sphere, Triangle3* triangle3, xyz_t* arg2);
s32 Math3D_pipeCrossTriangle_cp(Pipe* pipe, Triangle* arg1, xyz_t* arg2);
s32 Math3D_sphereCrossSphere_cl(Sphere* sphere1, Sphere* sphere2, f32* arg2);
s32 Math3D_sphereVsPipe_cl(Sphere* sphere, Pipe* pipe, f32* arg2);
s32 Math3D_pipeVsPipe_cl(Pipe* pipe1, Pipe* pipe2, f32* arg2);
extern xyz_t ZeroVec;
struct Actor;
struct Game_Play;
typedef enum ColliderShape {
            COLSHAPE_JNTSPH,
            COLSHAPE_PIPE,
            COLSHAPE_TRIS,
            COLSHAPE_MAX
} ColliderShape;
typedef enum ColMassType {
            MASSTYPE_IMMOVABLE,
            MASSTYPE_HEAVY,
            MASSTYPE_NORMAL
} ColMassType;
typedef struct ClObjElem {
              u8 flags;
} ClObjElem;
typedef struct ClObj_Properties {
              u8 ocFlags1;
              u8 ocFlags2;
              u8 shape;
} ClObj_Properties;
typedef struct ClObj {
              struct Actor* actor;
              struct Actor* oc;
              ClObj_Properties prop;
} ClObj;
typedef struct ClObjJntSphElemAttr_Init {
              u8 unk_0;
              Sphere unk_2;
              s16 unk_A;
} ClObjJntSphElemAttr_Init;
typedef struct ClObjJntSphElem_Init {
              ClObjElem elem;
              ClObjJntSphElemAttr_Init attr;
} ClObjJntSphElem_Init;
typedef struct ClObjJntSph_Init {
              ClObj_Properties prop;
              s32 count;
              ClObjJntSphElem_Init* elements;
} ClObjJntSph_Init;
typedef struct ClObjJntSphElemAttr {
               Sphere unk_00;
               Sphere unk_08;
               f32 unk_10;
               u8 unk_14;
} ClObjJntSphElemAttr;
typedef struct ClObjJntSphElem {
               ClObjElem elem;
               ClObjJntSphElemAttr attr;
} ClObjJntSphElem;
typedef struct ClObjJntSph {
               ClObj base;
               s32 count;
               ClObjJntSphElem* elements;
} ClObjJntSph;
typedef struct ClObjPipeAttr {
              Pipe dim;
} ClObjPipeAttr;
typedef struct ClObjPipe_Init {
              ClObj_Properties prop;
              ClObjElem elem;
              ClObjPipeAttr attr;
} ClObjPipe_Init;
typedef struct ClObjPipe {
               ClObj base;
               ClObjElem element;
               ClObjPipeAttr attribute;
} ClObjPipe;
typedef struct ClObjTrisElemAttr_Init {
               Triangle unk_00;
} ClObjTrisElemAttr_Init;
typedef struct ClObjTrisElemAttr {
               Triangle3 unk_00;
               xyz_t unk_34;
} ClObjTrisElemAttr;
typedef struct ClObjTrisElem {
               ClObjElem elem;
               ClObjTrisElemAttr attr;
} ClObjTrisElem;
typedef struct ClObjTris {
               ClObj base;
               s32 count;
               ClObjTrisElem* elements;
} ClObjTris;
typedef struct ClObjTrisElem_Init {
               ClObjElem elem;
               ClObjTrisElemAttr_Init attr;
} ClObjTrisElem_Init;
typedef struct ClObjTris_Init {
              ClObj_Properties prop;
              s32 count;
              ClObjTrisElem_Init* elem;
} ClObjTris_Init;
typedef struct CollisionCheck {
               u16 flags;
               s32 ocColCount;
               ClObj *ocColliders[50];
} CollisionCheck;
typedef struct CollisionCheck_Status_Init {
              u8 unk_0;
              s16 unk_2;
              s16 unk_4;
              s16 unk_6;
              u8 mass;
} CollisionCheck_Status_Init;
typedef struct CollisionCheck_Status {
               xyz_t displacement;
               s16 unk_0C;
               s16 unk_0E;
               s16 unk_10;
               u8 mass;
               u8 unk_13;
               u8 unk_14;
               u8 unk_15;
               u8 unk_16;
               u8 unk_17;
} CollisionCheck_Status;
s32 ClObjJntSph_ct(struct Game_Play* game_play, ClObjJntSph* colJntSph);
s32 ClObjJntSph_dt_nzf(struct Game_Play* game_play, ClObjJntSph* colJntSph);
s32 ClObjJntSph_set5_nzm(struct Game_Play* game_play, ClObjJntSph* colJntSph, struct Actor* actor, ClObjJntSph_Init* init, ClObjJntSphElem elements[]);
s32 ClObjPipe_ct(struct Game_Play* game_play, ClObjPipe* colPipe);
s32 ClObjPipe_dt(struct Game_Play* game_play, ClObjPipe* colPipe);
s32 ClObjPipe_set5(struct Game_Play* game_play, ClObjPipe* colPipe, struct Actor* actor, ClObjPipe_Init* init);
s32 ClObjTris_ct(struct Game_Play* game_play, ClObjTris* colTris);
s32 ClObjTris_dt_nzf(struct Game_Play* game_play, ClObjTris* colTris);
s32 ClObjTris_set5_nzm(struct Game_Play* game_play, ClObjTris* colTris, struct Actor* actor, ClObjTris_Init* init, ClObjTrisElem elements[]);
void CollisionCheck_ct(struct Game_Play* game_play, CollisionCheck* colCheck);
void CollisionCheck_dt(struct Game_Play* game_play, CollisionCheck* arg1);
void CollisionCheck_clear(struct Game_Play* game_play, CollisionCheck* colCheck);
s32 CollisionCheck_setOC(struct Game_Play* game_play, CollisionCheck* colCheck, ClObj* cl);
void CollisionCheck_OC(struct Game_Play* game_play, CollisionCheck* colCheck);
void CollisionCheck_OCC(struct Game_Play* game_play, CollisionCheck* colCheck);
s32 CollisionCheck_setOCC(struct Game_Play* game_play, CollisionCheck* colCheck, ClObj* cl);
void CollisionCheck_Status_ct(CollisionCheck_Status* status);
void CollisionCheck_Status_Clear(CollisionCheck_Status* status);
void CollisionCheck_Status_set3(CollisionCheck_Status* status, CollisionCheck_Status_Init* init);
void CollisionCheck_Uty_ActorWorldPosSetPipeC(struct Actor* actor, ClObjPipe* colPipe);
s32 CollisionCheck_Uty_setTrisPos_ad(struct Game_Play* game_play, ClObjTris* colTris, s32 index, ClObjTrisElemAttr_Init* init);
typedef struct mEv_event_save_c {
               char unk00[0x9C];
} mEv_event_save_c;
typedef struct Event {
               u8 unk_00;
               u8 unk_01;
               u8 unk_02;
               u8 unk_03;
               s16 unk_04;
               s16 unk_06;
               s32 unk_08;
               s32 unk_0C;
} Event;
void mEv_ClearEventSaveInfo(mEv_event_save_c* event_save);
void mEv_ClearEventInfo(void);
s32 mEv_CheckFirstJob(void);
s32 mEv_CheckFirstIntro(void);
void mEv_SetGateway(void);
void mEv_UnSetGateway(void);
s32 mEv_CheckTitleDemo(void);
void mEv_init(Event* event);
void mEv_init_force(void*);
void mEv_2nd_init(Event* event);
void mEv_run(Event* event);
void mEv_finish(Event* event);
void mEv_clear_status(s32, s16);
s32 mEv_check_status(s32, s16);
typedef struct LandName {
              s8 unk_0[0x6];
} LandName;
void mLd_CopyLandName(LandName* arg0, LandName* arg1);
void mLd_LandDataInit(void);
struct Private_c;
struct mPr_map_info_c;
struct mLd_land_info_c;
struct mPr_mother_mail_info_c;
typedef struct PlayerName {
              s8 unk_0[0x6];
} PlayerName;
typedef struct PersonalID {
              PlayerName unk_0;
              LandName unk_6;
              u16 unk_C;
              u16 unk_E;
} PersonalID;
void mPr_CopyPlayerName(PlayerName* dst, PlayerName* src);
void mPr_ClearPersonalID(PersonalID* arg0);
void mPr_CopyPersonalID(PersonalID* arg0, PersonalID* arg1);
s32 mPr_CheckCmpPersonalID(PersonalID* arg0, PersonalID* arg1);
void mPr_ClearPrivateInfo(struct Private_c* private);
void mPr_InitPrivateInfo(struct Private_c* private);
s32 mPr_CheckPrivate(struct Private_c* private);
void mPr_SetPossessionItem(struct Private_c* priv, int idx, u16 item, u32 cond);
s32 func_800B8D64_jp(u8 player_no, s32 arg1);
void mPr_ClearMotherMailInfo(struct mPr_mother_mail_info_c* arg0);
void func_800B9B2C_jp(void);
void mPr_SendForeingerAnimalMail(struct Private_c* now_private);
void mPr_StartSetCompleteTalkInfo(void);
void mPr_RenewalMapInfo(struct mPr_map_info_c* maps, s32 count, struct mLd_land_info_c* land_info);
struct struct_func_8085CE18_jp_arg4;
typedef struct mMl_get_npcinfo_from_mail_name_arg0 {
              u16 unk_0;
              u16 unk_2;
              LandName unk_4;
              u8 unk_A;
              u8 unk_B;
              u8 unk_C;
} mMl_get_npcinfo_from_mail_name_arg0;
typedef struct mMl_unk_00 {
               PersonalID unk_00;
               u8 unk_10;
} mMl_unk_00;
typedef struct mMl_unk_2A {
               s8 unk_00[0x7A];
} mMl_unk_2A;
typedef struct MailHeaderCommon {
               s8 unk_00;
               u8 unk_01;
               u8 unk_02[10];
               u8 unk_0C[16];
} MailHeaderCommon;
typedef struct Mail {
               mMl_unk_00 unk_00;
               mMl_unk_00 unk_12;
               s8 unk_24[0x2];
               u8 unk_26;
               s8 unk_27[0x1];
               u8 unk_28;
               u8 unk_29;
               mMl_unk_2A unk_2A;
} Mail;
s32 mMl_strlen(const char* arg0, s32 size, char c);
s32 mMl_strlen2(s32* arg0, const char* arg1, s32 size, char c);
void mMl_strcpy_back(char* dst, const char* src, s32 size);
void mMl_clear_mail_header(Mail* mail);
void mMl_clear_mail(Mail* mail);
void mMl_clear_mail_box(Mail* mail, s32 arg1);
s32 mMl_check_not_used_mail(Mail* mail);
void mMl_copy_header_name(mMl_unk_00* arg0, mMl_unk_00* arg1);
void mMl_set_from_name(Mail* arg0, Mail* arg1);
void mMl_set_to_name(Mail* arg0, Mail* arg1);
void mMl_set_to_plname(Mail* arg0, Mail* arg1);
void mMl_set_playername(Mail* mail, PersonalID* arg1);
void mMl_init_mail(Mail* mail, PersonalID* arg1);
s32 mMl_chk_mail_free_space(Mail mail[], s32 arg1);
s32 mMl_use_mail_space(Mail mail[], s32 arg1, PersonalID* arg2);
s32 mMl_count_use_mail_space(Mail mail[], s32 arg1);
void mMl_copy_mail(Mail* dst, Mail* src);
void mMl_clear_mail_header_common(MailHeaderCommon* arg0);
void mMl_copy_mail_header_common(MailHeaderCommon* arg0, MailHeaderCommon* arg1);
void mMl_set_mail_name_npcinfo(mMl_unk_00* arg0, mMl_get_npcinfo_from_mail_name_arg0* arg1);
s32 mMl_get_npcinfo_from_mail_name(mMl_get_npcinfo_from_mail_name_arg0* arg0, mMl_unk_00* arg1);
s32 mMl_hunt_for_send_address(Mail* mail);
s32 mMl_check_send_mail(struct struct_func_8085CE18_jp_arg4* arg0);
s32 mMl_check_set_present_myself(Mail* mail);
typedef u32 RomOffset;
typedef void* TexturePtr;
struct Game_Play;
struct Submenu;
struct struct_8085E9B0;
typedef enum mSMMoveProcIndex {
            MSM_MOVE_PROC_WAIT,
            MSM_MOVE_PROC_PREWAIT,
            MSM_MOVE_PROC_LINKWAIT,
            MSM_MOVE_PROC_PLAY,
            MSM_MOVE_PROC_END,
            MSM_MOVE_PROC_MAX
} mSMMoveProcIndex;
typedef enum InventoryItemList {
             INVENTORY_ITEM_LIST_0,
             INVENTORY_ITEM_LIST_1,
             INVENTORY_ITEM_LIST_ENTRUST,
             INVENTORY_ITEM_LIST_3,
             INVENTORY_ITEM_LIST_4,
             INVENTORY_ITEM_LIST_SELL,
             INVENTORY_ITEM_LIST_GIVE,
             INVENTORY_ITEM_LIST_7,
             INVENTORY_ITEM_LIST_TAKE,
             INVENTORY_ITEM_LIST_FURNITURE,
             INVENTORY_ITEM_LIST_MINIDISK,
             INVENTORY_ITEM_LIST_SHRINE,
             INVENTORY_ITEM_LIST_C,
             INVENTORY_ITEM_LIST_ECHANGE,
             INVENTORY_ITEM_LIST_E,
             INVENTORY_ITEM_LIST_F,
             INVENTORY_ITEM_LIST_MAX
} InventoryItemList;
typedef enum SubmenuProgramId {
             SUBMENU_PROGRAM_0,
             SUBMENU_PROGRAM_1,
             SUBMENU_PROGRAM_2,
             SUBMENU_PROGRAM_3,
             SUBMENU_PROGRAM_LEDIT,
             SUBMENU_PROGRAM_MAP,
             SUBMENU_PROGRAM_6,
             SUBMENU_PROGRAM_7,
             SUBMENU_PROGRAM_8,
             SUBMENU_PROGRAM_9,
             SUBMENU_PROGRAM_10,
             SUBMENU_PROGRAM_11,
             SUBMENU_PROGRAM_BOARD,
             SUBMENU_PROGRAM_13,
             SUBMENU_PROGRAM_14,
             SUBMENU_PROGRAM_15,
             SUBMENU_PROGRAM_16,
             SUBMENU_PROGRAM_17,
             SUBMENU_PROGRAM_18,
             SUBMENU_PROGRAM_19,
             SUBMENU_PROGRAM_CATALOG,
             SUBMENU_PROGRAM_MAX,
} SubmenuProgramId;
typedef void (*SubmenuMoveFunc)(struct Submenu*);
typedef void (*SubmenuDrawFunc)(struct Submenu*, struct Game_Play*);
typedef struct Submenu {
               s32 unk_00;
               SubmenuProgramId programId;
               SubmenuProgramId unk_08;
               mSMMoveProcIndex moveProcIndex;
               s32 unk_10;
               s32 unk_14;
               s32 unk_18;
               s32 unk_1C;
               s32 unk_20;
               void* linkedAllocStart;
               void* linkedAllocEnd;
               struct struct_8085E9B0* unk_2C;
               SubmenuMoveFunc move;
               SubmenuDrawFunc draw;
               Mail mail;
               u8 unk_DC;
               u8 unk_DD;
               u8 unk_DE;
               u8 unk_DF;
               u16 unk_E0;
               u8 unk_E2;
               u8 unk_E3;
               xyz_t unk_E4;
} Submenu;
s32 SubmenuArea_IsPlayer(void);
void* mSM_ovlptr_dllcnv(void* vram, Submenu* submenu);
void mSM_submenu_ovlptr_init(struct Game_Play* game_play);
void mSM_submenu_ovlptr_cleanup(Submenu* submenu);
void load_player(Submenu* submenu);
void mSM_submenu_ct(Submenu* submenu);
void mSM_submenu_dt(Submenu* submenu);
void mSM_open_submenu(Submenu* submenu, SubmenuProgramId programId, s32 arg2, s32 arg3);
void mSM_open_submenu_new(Submenu* submenu, SubmenuProgramId programId, s32 arg2, s32 arg3, s32 arg4);
void mSM_open_submenu_new2(Submenu* submenu, SubmenuProgramId programId, s32 arg2, s32 arg3, s32 arg4, s32 arg5);
void mSM_submenu_ctrl(struct Game_Play* game_play);
void mSM_submenu_move(Submenu* submenu);
void mSM_submenu_draw(Submenu* submenu, struct Game_Play* game_play);
u32 mSM_check_open_inventory_itemlist(InventoryItemList itemlist, s32 arg1) ;
typedef struct GameAllocEntry {
              struct GameAllocEntry* next;
              struct GameAllocEntry* prev;
              size_t size;
              s8 unk_0C[0x4];
} GameAllocEntry;
typedef struct GameAlloc {
               GameAllocEntry base;
               GameAllocEntry* head;
} GameAlloc;
void func_800D3720_jp(GameAlloc* this);
void* gamealloc_malloc(GameAlloc* this, size_t size);
void gamealloc_free(GameAlloc* this, void* ptr);
void gamealloc_cleanup(GameAlloc* this);
void gamealloc_init(GameAlloc* this);
typedef struct Input {
               OSContPad cur;
               OSContPad prev;
               OSContPad press;
               OSContPad rel;
} Input;
void pad_init(Input* input);
void pad_cleanup(void);
void pad_flush(Input* input);
s32 pad_push_only(Input* input, u16 value);
s32 pad_push_also(Input* input, u16 key);
s32 pad_on_trigger(Input* input, u16 key);
s32 pad_off_trigger(Input* input, u16 key);
u16 pad_button(Input* input);
u16 pad_trigger(Input* input);
s8 pad_physical_stick_x(Input* input);
s8 pad_physical_stick_y(Input* input);
void pad_set_logical_stick(Input* input, s32 x, s32 y);
s8 pad_logical_stick_x(Input* input);
s8 pad_logical_stick_y(Input* input);
s8 pad_stick_x(Input* input);
s8 pad_stick_y(Input* input);
void pad_correct_stick(Input* input);
typedef struct Controller {
               f32 moveX;
               f32 moveY;
               f32 moveR;
               s16 moveAngle;
               f32 lastMoveX;
               f32 lastMoveY;
               f32 lastMoveR;
               s16 lastMoveAngle;
               f32 adjustedX;
               f32 adjustedY;
               f32 adjustedR;
               f32 lastAdjustedX;
               f32 lastAdjustedY;
               f32 lastAdjustedR;
} Controller;
struct Game;
void mCon_ct(struct Game* game);
void mCon_dt(struct Game* game);
void mCon_calc(Controller* controller, f32 x, f32 y);
void mCon_main(struct Game* game);
s32 chkButton(u16 mask);
u16 getButton(void);
s32 chkTrigger(u16 mask);
u16 getTrigger(void);
s32 getJoystick_X(void);
s32 getJoystick_Y(void);
struct Game;
struct GraphicsContext;
struct struct_80145020_jp;
typedef enum GameStateId {
           GAMESTATE_FIRST_GAME,
           GAMESTATE_MAP_SELECT,
           GAMESTATE_PLAY,
           GAMESTATE_SECOND_GAME,
           GAMESTATE_00743CD0,
           GAMESTATE_TRADEMARK,
           GAMESTATE_PLAYER_SELECT,
           GAMESTATE_SAVE_MENU,
           GAMESTATE_FAMICOM_EMU,
           GAMESTATE_PRENMI,
               GAMESTATE_ID_MAX
} GameStateId;
typedef void (*GameStateFunc)(struct Game* game);
typedef struct {
               void* loadedRamAddr;
               RomOffset vromStart;
               RomOffset vromEnd;
               void* vramStart;
               void* vramEnd;
               void* unk_14;
               GameStateFunc init;
               GameStateFunc destroy;
               void* unk_20;
               void* unk_24;
               s32 unk_28;
               size_t instanceSize;
} GameStateOverlay;
typedef struct Game {
               struct GraphicsContext* gfxCtx;
               GameStateFunc main;
               GameStateFunc destroy;
               GameStateFunc init;
               size_t size;
               Input input[4];
               s32 unk_74;
               TwoHeadArena heap;
               GameAlloc alloc;
               u8 unk_9C;
               u8 unk_9D;
               u8 disableDisplay;
               u8 running;
               s32 unk_A0;
               s32 disablePrenmi;
               Controller controller;
} Game;

typedef struct CollisionBGCheck_WallInfo {
  s16 angleY;
  s16 type;
} CollisionBGCheck_WallInfo;

typedef struct CollisionBGCheck_result{
    u32 onGround:1;
    u32 unk:5;
    u32 hitWall:5;
    u32 hitWallCount:3;
    u32 unk2:18;
}CollisionBGCheck_result;

typedef struct CollisionBGCheck{
    u8 unk0[0x11];
    CollisionBGCheck_result result;
    u8 unk14[0xC];
    CollisionBGCheck_WallInfo wallInfo[2];
    s16 pad;
}CollisionBGCheck;
void func_800D2E00_jp(Game* game);
void func_800D2E58_jp(u16 button, Gfx** gfxP);
void game_debug_draw_last(Game* game, struct GraphicsContext* gfxCtx);
void game_draw_first(struct GraphicsContext* gfxCtx);
void game_draw_last(struct GraphicsContext* gfxCtx);
void game_get_controller(Game* game);
void SetGameFrame(s32 divisor);
void game_main(Game* game);
void game_init_hyral(Game* game, size_t size);
void game_resize_hyral(Game* game, size_t size);
void game_ct(Game* game, GameStateFunc init, struct GraphicsContext* gfxCtx);
void game_dt(Game* game);
GameStateFunc game_get_next_game_init(Game* game);
size_t game_get_next_game_class_size(Game* game);
s32 game_is_doing(Game* game);
s32 game_getFreeBytes(Game* game);
void game_goto_next_game_play(Game* game);
void game_goto_next_game_name_famicom_emu(Game* game);
extern Game* gamePT;
extern struct struct_80145020_jp B_80145020_jp;
extern s16 B_80145040_jp;
extern Game* game_class_p;
extern u8 game_GameFrame;
extern f32 game_GameFrameF;
extern f32 game_GameFrame_2F;
extern f32 game_GameFrame__1F;
extern GameStateOverlay game_dlftbls[GAMESTATE_ID_MAX];
extern GameStateId game_dlftbls_num;
struct Actor;
struct Game_Play;
struct ActorEntry;
struct ActorOverlay;
struct struct_801161E8_jp;
struct LightsN;
struct GraphicsContext;
typedef enum AllocType {
            ALLOCTYPE_NORMAL,
            ALLOCTYPE_ABSOLUTE,
            ALLOCTYPE_PERMANENT
} AllocType;
typedef enum ActorPart {
            ACTOR_PART_0,
            ACTOR_PART_1,
            ACTOR_PART_PLAYER,
            ACTOR_PART_NPC,
            ACTOR_PART_4,
            ACTOR_PART_5,
            ACTOR_PART_6,
            ACTOR_PART_7,
            ACTOR_PART_MAX
} ActorPart;
typedef enum ActorFootIndex {
            FOOT_LEFT,
            FOOT_RIGHT,
            FOOT_MAX
} ActorFootIndex;
typedef enum FgNameF000 {
              FGNAME_F000_0 = 0x0,
              FGNAME_F000_5 = 0x5,
              FGNAME_F000_8 = 0x8,
              FGNAME_F000_D = 0xD,
              FGNAME_F000_E
} FgNameF000;
typedef void (*ActorFunc)(struct Actor* this, struct Game_Play* game_play);
typedef struct ActorProfile {
               s16 name;
               u8 part;
               u32 flags;
               u16 unk_08;
               s16 objectId;
               size_t instanceSize;
               ActorFunc ct;
               ActorFunc dt;
               ActorFunc update;
               ActorFunc draw;
               ActorFunc save;
} ActorProfile;
typedef struct PosRot {
               xyz_t pos;
               s_xyz rot;
} PosRot;
typedef void (*Shape_Info_unk_0C)(struct Actor*, struct LightsN*, struct Game_Play*);
typedef struct Shape_Info {
               s_xyz rot;
               f32 unk_08;
               Shape_Info_unk_0C unk_0C;
               f32 unk_10;
               f32 unk_14;
               f32 unk_18;
               f32 unk_1C;
               s32 unk_20;
               xyz_t* unk_24;
               s32 unk_28;
               s8 unk_2C;
               s8 unk_2D;
               s8 unk_2E;
               s8 unk_2F[0x1];
               xyz_t feetPos[FOOT_MAX];
} Shape_Info;
typedef struct Actor {
                s16 name;
                u8 part;
                u8 unk_003;
                u16 unk_004;
                u16 fgName;
                s8 unk_008;
                s8 unk_009;
                s16 unk_00A;
                PosRot home;
                u32 flags;
                s16 params;
                s16 unk_026;
                PosRot world;
                xyz_t prevPos;
                PosRot eye;
                xyz_t scale;
                xyz_t velocity;
                f32 speed;
                f32 gravity;
                f32 terminalVelocity;
                s32 unk_080;
                CollisionBGCheck bgCheck;
                s8 unk_0B4[0x1];
                u8 isDrawn;
                s16 yawTowardsPlayer;
                f32 xyzDistToPlayerSq;
                f32 xzDistToPlayer;
                f32 playerHeightRel;
                CollisionCheck_Status colStatus;
                Shape_Info shape;
                xyz_t projectedPos;
                f32 projectedW;
                f32 uncullZoneScale;
                f32 uncullZoneDownward;
                f32 unk_13C;
                f32 unk_140;
                f32 unk_144;
                u8 unk_148;
                u8 unk_149;
                s8 unk_14A[0x2];
                struct Actor* parent;
                struct Actor* child;
                struct Actor* prev;
                struct Actor* next;
                ActorFunc ct;
                ActorFunc dt;
                ActorFunc update;
                ActorFunc draw;
                ActorFunc save;
                struct ActorOverlay* overlayEntry;
} Actor;
typedef struct Part_Break {
               MtxF* matrices;
               s16* objectIds;
               s16 count;
               Gfx** dLists;
               s32 val;
               s32 prevLimbIndex;
} Part_Break;
typedef struct ActorListEntry {
              s32 unk_0;
              Actor* head;
} ActorListEntry;
typedef struct ActorInfo {
               s32 unk_00;
               ActorListEntry actorLists[ACTOR_PART_MAX];
} ActorInfo;
void projection_pos_set(struct Game_Play* game_play, xyz_t* worldPos, xyz_t* projectedPos, f32* invW);
void Actor_world_to_eye(Actor* actor, f32 arg1);
void Actor_position_move(Actor* actor);
void Actor_position_speed_set(Actor* actor);
void Actor_position_moveF(Actor* actor);
s32 Actor_player_look_direction_check(Actor* actor, s16 maxAngleDiff, struct Game_Play* game_play);
void Actor_display_position_set(struct Game_Play* game_play, Actor* actor, s16* x, s16* y);
void Shape_Info_init(Actor* actor, f32 arg1, Shape_Info_unk_0C arg2, f32 arg3, f32 arg4);
void Actor_foot_shadow_pos_set(Actor* actor, s32 limbIndex, s32 leftFootIndex, xyz_t* leftFootPos, s32 rightFootIndex, xyz_t* rightFootPos);
void Actor_delete(Actor* actor);
s32 Actor_draw_actor_no_culling_check(Actor* actor);
s32 Actor_draw_actor_no_culling_check2(Actor* actor, xyz_t* arg1, f32 arg2);
void Actor_cull_check(Actor* actor);
void Actor_delete_check(Actor* actor, struct Game_Play* game_play);
void Actor_info_ct(struct Game_Play* game_play2, ActorInfo* actorInfo, struct ActorEntry* actorEntry);
void Actor_info_dt(ActorInfo* actorInfo, struct Game_Play* game_play);
void Actor_info_call_actor(struct Game_Play* game_play, ActorInfo* actorInfo);
void Actor_info_draw_actor(struct Game_Play* game_play, ActorInfo* actorInfo);
void Actor_free_overlay_area(struct ActorOverlay* overlayEntry);
void Actor_get_overlay_area(struct ActorOverlay* overlayEntry, const struct struct_801161E8_jp* arg1, size_t overlaySize);
void Actor_init_actor_class(Actor* actor, ActorProfile* profile, struct ActorOverlay* overlayEntry, struct Game_Play* game_play, s32 arg4, f32 x, f32 y, f32 z, s16 rotX, s16 rotY, s16 rotZ, s8 argB, s8 argC, s16 argD, u16 fgName, s16 params);
Actor* Actor_info_make_actor(ActorInfo* actorInfo, struct Game_Play* game_play, s16 actorId, f32 x, f32 y, f32 z, s16 rotX, s16 rotY, s16 rotZ, s8 arg9, s8 argA, s16 argB, u16 fgName, s16 params, s8 argE, s32 argF);
Actor* Actor_info_make_child_actor(ActorInfo* actorInfo, Actor* arg1, struct Game_Play* game_play, s16 actorId, f32 x, f32 y, f32 z, s16 rotX, s16 rotY, s16 rotZ, s16 argA, u16 fgName, s16 params, s32 argD);
void restore_fgdata(Actor* actor, struct Game_Play* game_play);
void restore_fgdata_one(Actor* actor, struct Game_Play* game_play);
void restore_fgdata_all(struct Game_Play* game_play);
void Actor_info_save_actor(struct Game_Play* game_play);
Actor* Actor_info_delete(ActorInfo* actorInfo, Actor* actor, struct Game_Play* game_play);
Actor* Actor_info_name_search(ActorInfo* actorInfo, s16 name, ActorPart part);
Actor* Actor_info_fgName_search(ActorInfo* actorInfo, u16 fgName, ActorPart part);
void Part_Break_init(Part_Break* partBreak, s32 count, s32 arg2);
Gfx* HiliteReflect_new(xyz_t* object, xyz_t* eye, xyz_t* lightDir, struct GraphicsContext* gfxCtx, Gfx* gfx, Hilite** hilite);
Hilite* HiliteReflect_init(xyz_t* object, xyz_t* eye, xyz_t* lightDir, struct GraphicsContext* gfxCtx);
Hilite* HiliteReflect_xlu_init(xyz_t* object, xyz_t* eye, xyz_t* lightDir, struct GraphicsContext* gfxCtx);
Hilite* HiliteReflect_light_init(xyz_t* object, xyz_t* eye, xyz_t* lightDir, struct GraphicsContext* gfxCtx);
Hilite* Setpos_HiliteReflect_init(xyz_t* object, struct Game_Play* game_play);
Hilite* Setpos_HiliteReflect_xlu_init(xyz_t* object, struct Game_Play* game_play);
Hilite* Setpos_HiliteReflect_light_init(xyz_t* object, struct Game_Play* game_play);
struct Actor;
struct Global_light;
typedef void (*Kankyo_unk_C0)(struct Actor*);
typedef struct Kankyo {
               s8 unk_00[0x2];
               s8 unk_02;
               s8 unk_03;
               s8 unk_04;
               s8 unk_05[0x3];
               s8 unk_08[0xA6];
               u8 unk_AE;
               u8 unk_AF;
               u8 unk_B0;
               s8 unk_B1[0xF];
               Kankyo_unk_C0 unk_C0;
               s8 unk_C4[0x4];
} Kankyo;
void Global_kankyo_ct(struct Game_Play* game_play, struct Kankyo* kankyo);
void Global_kankyo_set(struct Game_Play* game_play, struct Kankyo* kankyo, struct Global_light* arg2);
void mEnv_WindMove(void);
void mEnv_DecideWeather_NormalGameStart(void);
typedef struct ListAlloc {
              struct ListAlloc* prev;
              struct ListAlloc* next;
} ListAlloc;
typedef struct PreRender {
               u16 unk_00;
               u16 unk_02;
               u16 unk_04;
               u16 unk_06;
               s8 unk_08[0x8];
               void* unk_10;
               void* unk_14;
               void* unk_18;
               s8 unk_1C[0x34];
} PreRender;
void PreRender_setup_savebuf(PreRender* render, s32 arg1, s32 arg2, s32 arg3, s32 arg4, s32 arg5);
void PreRender_init(PreRender* render);
void PreRender_setup_renderbuf(PreRender* render, s32 arg1, s32 arg2, void* arg3, void* arg4);
void PreRender_cleanup(PreRender* render);
void PreRender_CopyRGBC(PreRender* render, Gfx** gfxP, s32 arg2, s32 arg3);
void PreRender_saveFrameBuffer(PreRender* render, Gfx** gfx);
void PreRender_saveCVG(PreRender* render, Gfx** gfx);
void PreRender_loadFrameBufferCopy(PreRender* render, Gfx** gfx);
void PreRender_ConvertFrameBuffer_fg(PreRender* render);
typedef enum bool {
    false,
    true
} bool;
typedef struct Struct_8010EAA0 {
               RomOffset unk_00;
               RomOffset unk_04;
               char unk08[0xB];
               u8 unk_13;
} Struct_8010EAA0;
typedef struct struct_801161E8_jp {
              s8 unk_0[0x8];
} struct_801161E8_jp;
typedef struct CommonData_unk_1004C_unk_14_arg0 {
               s16 unk_00;
               s16 unk_02;
               s8 unk_04[0x60];
} CommonData_unk_1004C_unk_14_arg0;
typedef struct Game_Play_unk_0110_unk_0000 {
               s16 unk_00;
               s8 unk_02[0x2];
               void* segment;
               s8 unk_08[0x48];
               s16 unk_50;
               s8 unk_52[0x2];
} Game_Play_unk_0110_unk_0000;
typedef struct Game_Play_unk_0110 {
                 Game_Play_unk_0110_unk_0000 unk_0000[1];
                 s8 unk_0054[0x17A8];
                 s32 unk_17FC;
                 s8 unk_1800[0x18];
                 void* unk_1818;
} Game_Play_unk_0110;
typedef struct ShadowData {
               u32 numberOfVertices;
               u8* vertexFixFlagTable;
               f32 size;
               Vtx* vertices;
               Gfx* model;
} ShadowData;
typedef struct struct_809AEFA4 {
               void* unk_00;
               void* unk_04;
               s32 unk_08;
               void* unk_0C;
               void* unk_10;
               void* unk_14;
} struct_809AEFA4;
typedef struct struct_func_8085CE18_jp_arg4 {
               s8 unk_00[0x24];
               u16 unk_24;
               u8 unk_26;
} struct_func_8085CE18_jp_arg4;
struct GraphicsContext;
struct Game;
typedef enum {
            MTXMODE_NEW,
            MTXMODE_APPLY
} MatrixMode;
void new_Matrix(struct Game* game);
void Matrix_push(void);
void Matrix_pull(void);
void Matrix_get(MtxF* dest);
void Matrix_put(MtxF* src);
MtxF* get_Matrix_now(void);
void Matrix_mult(MtxF* mf, u8 mode);
void Matrix_translate(f32 x, f32 y, f32 z, u8 mode);
void Matrix_scale(f32 x, f32 y, f32 z, u8 mode);
void Matrix_RotateX(s16 x, MatrixMode mode);
void Matrix_RotateY(s16 y, MatrixMode mode);
void Matrix_RotateZ(s16 z, MatrixMode mode);
void Matrix_rotateXYZ(s16 x, s16 y, s16 z, MatrixMode mode);
void Matrix_softcv3_mult(xyz_t* translation, s_xyz* rot);
void Matrix_softcv3_load(f32 x, f32 y, f32 z, s_xyz* rot);
Mtx* _MtxF_to_Mtx(MtxF* src, Mtx* dest);
Mtx* _Matrix_to_Mtx(Mtx* dest);
Mtx* _Matrix_to_Mtx_new(struct GraphicsContext* gfxCtx);
void _MtxF_to_Mtx_new(MtxF* src, struct GraphicsContext* gfxCtx);
void Matrix_Position(xyz_t* src, xyz_t* dest);
void Matrix_Position_Zero(xyz_t* dest);
void Matrix_Position_VecX(f32 x, xyz_t* dest);
void Matrix_Position_VecY(f32 y, xyz_t* dest);
void Matrix_Position_VecZ(f32 z, xyz_t* dest);
void Matrix_copy_MtxF(MtxF* dest, MtxF* src);
void Matrix_MtxtoMtxF(Mtx* src, MtxF* dest);
void Matrix_MtxF_Position2(xyz_t* src, xyz_t* dest, MtxF* mf);
void Matrix_reverse(MtxF* mf);
void Matrix_rotate_scale_exchange(MtxF* mf);
void Matrix_to_rotate_new(MtxF* src, s_xyz* dest, s32 nonUniformScale);
void Matrix_to_rotate2_new(MtxF* src, s_xyz* dest, s32 nonUniformScale);
void Matrix_RotateVector(s16 angle, xyz_t* axis, u8 mode);
void suMtxMakeTS(Mtx* mtx, f32 scaleX, f32 scaleY, f32 scaleZ, f32 translateX, f32 translateY, f32 translateZ);
void suMtxMakeSRT(Mtx* mtx, f32 scaleX, f32 scaleY, f32 scaleZ, s16 rotX, s16 rotY, s16 rotZ, f32 translateX,
                  f32 translateY, f32 translateZ);
void suMtxMakeSRT_ZXY(Mtx* mtx, f32 scaleX, f32 scaleY, f32 scaleZ, s16 rotX, s16 rotY, s16 rotZ, f32 translateX,
                      f32 translateY, f32 translateZ);
extern Mtx Mtx_clear;
typedef u8 lbRTC_sec_t;
typedef u8 lbRTC_min_t;
typedef u8 lbRTC_hour_t;
typedef u8 lbRTC_day_t;
typedef u8 lbRTC_weekday_t;
typedef u8 lbRTC_month_t;
typedef u16 lbRTC_year_t;
typedef struct lbRTC_time_c {
             lbRTC_sec_t sec;
             lbRTC_min_t min;
             lbRTC_hour_t hour;
             lbRTC_day_t day;
             lbRTC_weekday_t weekday;
             lbRTC_month_t month;
             lbRTC_year_t year;
} lbRTC_time_c;
typedef struct lbRTC_ymd_t {
               lbRTC_year_t year;
               lbRTC_month_t month;
               lbRTC_day_t day;
} lbRTC_ymd_t;
typedef enum WEEKDAYS {
            lbRTC_WEEKDAYS_BEGIN = 0,
            lbRTC_SUNDAY = lbRTC_WEEKDAYS_BEGIN,
            lbRTC_MONDAY,
            lbRTC_TUESDAY,
            lbRTC_WEDNESDAY,
            lbRTC_THURSDAY,
            lbRTC_FRIDAY,
            lbRTC_SATURDAY,
            lbRTC_WEEK,
            lbRTC_WEEKDAYS_MAX = lbRTC_WEEK
} WEEKDAYS;
typedef enum MONTHS {
            lbRTC_MONTHS_BEGIN = 0,
            lbRTC_JANUARY = 1,
            lbRTC_FEBRUARY,
            lbRTC_MARCH,
            lbRTC_APRIL,
            lbRTC_MAY,
            lbRTC_JUNE,
            lbRTC_JULY,
            lbRTC_AUGUST,
            lbRTC_SEPTEMBER,
             lbRTC_OCTOBER,
             lbRTC_NOVEMBER,
             lbRTC_DECEMBER,
             lbRTC_MONTHS_MAX = lbRTC_DECEMBER
} MONTHS;
typedef enum RTC_EQUALITY {
    lbRTC_LESS = -1,
    lbRTC_EQUAL = 0,
    lbRTC_OVER = 1
} RTC_EQUALITY;
typedef enum RTC_EQUALITY_FLAGS {
              lbRTC_CHECK_NONE = 0,
              lbRTC_CHECK_SECONDS = 1 << 0,
              lbRTC_CHECK_MINUTES = 1 << 1,
              lbRTC_CHECK_HOURS = 1 << 2,
              lbRTC_CHECK_WEEKDAYS = 1 << 3,
              lbRTC_CHECK_DAYS = 1 << 4,
              lbRTC_CHECK_MONTHS = 1 << 5,
              lbRTC_CHECK_YEARS = 1 << 6,
    lbRTC_CHECK_ALL = lbRTC_CHECK_SECONDS |
                      lbRTC_CHECK_MINUTES |
                      lbRTC_CHECK_HOURS |
                      lbRTC_CHECK_WEEKDAYS |
                      lbRTC_CHECK_DAYS |
                      lbRTC_CHECK_MONTHS |
                      lbRTC_CHECK_YEARS
} RTC_EQUALITY_FLAGS;
struct Game;
typedef struct mFM_fg_c {
              u16 items[16][16];
} mFM_fg_c;
void func_80087004_jp(void);
void func_80087280_jp(void);
void mFM_SetBlockKindLoadCombi(struct Game* game);
void mFM_InitFgCombiSaveData(struct Game* game);
void mFM_RenewalReserve(void);
typedef enum mHm_ROOM{
            mHm_ROOM_MAIN,
            mHm_ROOM_UPPER,
            mHm_ROOM_BASEMENT,
            mHm_ROOM_NUM
} mHm_ROOM;
typedef enum mHm_ROOMTYPE {
            mHm_ROOMTYPE_SMALL,
            mHm_ROOMTYPE_MEDIUM,
            mHm_ROOMTYPE_LARGE,
            mHm_ROOMTYPE_COTTAGE,
            mHm_ROOMTYPE_NUM
} mHm_ROOMTYPE;
typedef enum mHm_HOMESIZE {
            mHm_HOMESIZE_SMALL,
            mHm_HOMESIZE_MEDIUM,
            mHm_HOMESIZE_LARGE,
            mHm_HOMESIZE_UPPER,
            mHm_HOMESIZE_STATUE,
            mHm_HOMESIZE_NUM
}mHm_HOMESIZE;
typedef struct Haniwa_Item_c {
               u16 item;
               s16 exchange_type;
               u16 extra_data;
} Haniwa_Item_c;
typedef struct Haniwa_c {
               Haniwa_Item_c items[4];
               u8 message[64];
               u32 bells;
} Haniwa_c;
typedef struct mHm_hs_c {
                PersonalID unk_000;
                s8 unk_010[0x12];
                u8 unk_022;
                s8 unk023[0x1];
                u8 unk_024;
                s8 unk025[0xABB];
                Haniwa_c haniwa;
                s8 unkB3C[0xC];
} mHm_hs_c;
typedef struct mLd_land_info_c {
    char unk00[0xA];
} mLd_land_info_c;
typedef struct NpcList {
               char unk00[0x10];
               xyz_t position;
               char unk14[0x1C];
} NpcList;
typedef struct AnmHome_c {
               u8 typeUnused;
               u8 blockX;
               u8 blockZ;
               u8 utX;
               u8 utZ;
} AnmHome_c;
typedef struct AnmPersonalId_c {
               u16 npcId;
               u16 landId;
               u8 landName[6];
               u8 nameId;
               u8 looks;
} AnmPersonalID_c;
typedef struct Animal_c {
                AnmPersonalID_c id;
                char unk00C[0x4D4];
                AnmHome_c homeInfo;
                char unk4E5[0x43];
} Animal_c;
typedef enum NpcLooks{
            NPC_LOOKS_GIRL,
            NPC_LOOKS_KO_GIRL,
            NPC_LOOKS_BOY,
            NPC_LOOKS_SPORT_MAN,
            NPC_LOOKS_GRIM_MAN,
            NPC_LOOKS_NANIWA_LADY,
            NPC_LOOKS_NUM
}NpcLooks;
void mNpc_ClearAnimalPersonalID(AnmPersonalID_c*);
s32 mNpc_CheckFreeAnimalPersonalID(AnmPersonalID_c*);
void mNpc_CopyAnimalPersonalID(AnmPersonalID_c*, AnmPersonalID_c*);
s32 mNpc_CheckCmpAnimalPersonalID(AnmPersonalID_c*, AnmPersonalID_c*);
void mNpc_RenewalAnimalMemory(void);
s32 mNpc_CheckFreeAnimalInfo(Animal_c*);
void func_800A91DC_jp(void);
void func_800AA124_jp(void);
s32 mNpc_GetLooks(u16 arg0);
void func_800AB054_jp(void);
void func_800AB09C_jp(void);
void mNpc_SetNpcList(NpcList* npclist, Animal_c* animals, s32 count, s32 malloc_flag);
void mNpc_SetNpcinfo(struct Actor* actor, s8 arg1);
s32 mNpc_GetInAnimalP(void);
void mNpc_SetRemoveAnimalNo(Animal_c* animal);
void mNpc_SendRegisteredGoodbyMail(void);
void mNpc_SetReturnAnimal(s32 arg0);
void mNpc_GetNpcWorldNameAnm(PlayerName* arg0, struct mMl_get_npcinfo_from_mail_name_arg0* arg1);
void mNpc_InitNpcAllInfo(s32 arg0);
void func_800AD9FC_jp(void);
void mNpc_ClearTalkInfo(void);
typedef struct NpcWalk {
               char unk00[0x7C];
} NpcWalk;
typedef struct NpcWalkGoalData {
              u8* types;
              u8 count;
              s32 time;
} NpcWalkGoalData;
typedef struct NpcWalkGoalDataTable {
              NpcWalkGoalData* data;
              int count;
} NpcWalkGoalDataTable;
typedef struct NpcWalkAppear {
              u8 status;
              u8 way;
} NpcWalkAppear;
typedef struct NpcWalkInfo {
               AnmPersonalID_c id;
               s32 idx;
               u8 status;
               u8 type;
               u8 goalX;
               u8 goalZ;
               NpcWalkAppear appearInfo;
} NpcWalkInfo;
typedef struct NpcWalking {
               NpcWalkInfo info[(int)((15) / 3)];
               u16 idxUse;
               u8 infoUse;
               u8 infoMax;
} NpcWalking;
typedef enum WalkGoal{
            NPCW_GOAL_SHRINE,
            NPCW_GOAL_HOME,
            NPCW_GOAL_ALONE,
            NPCW_GOAL_MY_HOME,
            NPCW_GOAL_NUM
}WalkGoal;
typedef enum WalkGoalBlock{
            NPCW_GOAL_BLOCK_SHRINE,
            NPCW_GOAL_BLOCK_HOME,
            NPCW_GOAL_BLOCK_NUM
}WalkGoalBlock;
typedef enum WalkAppearStatus {
            NPCW_APPEAR_STATUS_0,
            NPCW_APPEAR_STATUS_1,
            NPCW_APPEAR_STATUS_NUM
}WalkAppearStatus;
typedef enum WalkAppearDirection {
            NPCW_APPEAR_WAY_UP,
            NPCW_APPEAR_WAY_DOWN,
            NPCW_APPEAR_WAY_LEFT,
            NPCW_APPEAR_WAY_RIGHT,
            NPCW_APPEAR_WAY_NUM
}WalkAppearDirection;
typedef enum WalkInfoStatus {
            NPCW_INFO_STATUS_0,
            NPCW_INFO_STATUS_WALKING,
            NPCW_INFO_STATUS_2,
            NPCW_INFO_STATUS_3,
            NPCW_INFO_STATUS_4,
            NPCW_INFO_STATUS_NUM
}WalkInfoStatus;
NpcWalkGoalData* mNpcW_GetGoalDataInfo(s32 looks, s32 time);
void mNpcW_ClearNpcWalkAppear(NpcWalkAppear* info);
void mNpcW_ClearNpcWalkInfo(NpcWalkInfo* info , s32 count);
void mNpcW_ClearNpcWalk(NpcWalking* walk);
s32 mNpcW_CheckFreeNpcWalkInfo(NpcWalkInfo* info );
s32 mNpcW_GetFreeNpcWalkInfoIdx(NpcWalkInfo* info , s32 num );
s32 mNpcW_GetNpcWalkInfoIdx(NpcWalkInfo* info , s32 num , AnmPersonalID_c* anmId );
s32 mNpcW_GetNpcWalkInfoIdxbyIdx(NpcWalkInfo* info , s32 num , s32 idx );
s32 mNpcW_DecideNpc(Animal_c* animal , u16 used );
void mNpcW_SetNpcWalkInfo(NpcWalkInfo* info , Animal_c* animal , s32 idx );
s32 mNpcW_ChangeNpcWalk(NpcWalking* walk , NpcWalkInfo* info );
s32 mNpcW_GetAloneBlock(u8* goalBlockX , u8* goalBlockZ );
void mNpcW_GetBlockXZNumExceptHome(s32* goalBlockX , s32* goalBlockZ , Animal_c* animal );
s32 mNpcW_CheckDiffBlockWalkNpcHome(s32 blockX , s32 blockZ , NpcWalkInfo* info );
void mNpcW_SetHomeBlockSource(NpcWalking* walk , Animal_c* animal );
void mNpcW_InitGoalBlockSource(NpcWalking* walk , Animal_c* animal );
void mNpcW_SetGoalBlock(NpcWalkInfo* info );
void mNpcW_InitNpcWalk(NpcWalking* walk );
s32 mNpcW_GetAppearStatusWay(u8* status , u8* way , Animal_c* animal );
s32* mNpcW_GetArriveStayCountP(s32 idx );
s32 mNpcW_GetWalkInfoStatusGoalAnimalIdx(s32* status , s32* goal , s32 idx );
s32 mNpcW_GetNearGate(s32* targetUtX , s32* targetUtZ , s32 blockX , s32 blockZ , s32 utX , s32 utZ );
typedef enum NpsScheduleType{
            NPS_SCHEDULE_FIELD,
            NPS_SCHEDULE_IN_HOUSE,
            NPS_SCHEDULE_SLEEP,
            NPS_SCHEDULE_STAND,
            NPS_SCHEDULE_WANDER,
            NPS_SCHEDULE_SPECIAL,
            NPS_SCHEDULE_TYPE_NUM
} NpsScheduleType;
typedef struct NpsScheduleData {
               NpsScheduleType type;
               s32 endTime;
} NpsScheduleData;
typedef struct NpsScheduleDataTable {
               s32 count;
               NpsScheduleData* scheduleData;
} NpsScheduleDataTable;
typedef struct NpsSchedule{
               AnmPersonalID_c* id;
               NpsScheduleDataTable* dataTable;
               u8 currentType;
               u8 forcedType;
               u8 savedType;
               s32 forcedTimer;
}NpsSchedule;
NpsSchedule* mNPS_get_schedule_area(AnmPersonalID_c* id);
void mNPS_set_schedule_area(AnmPersonalID_c* id);
void mNPS_reset_schedule_area(AnmPersonalID_c* id);
void mNPS_schedule_manager_sub0(void);
void mNPS_schedule_manager_sub(void);
void mNPS_schedule_manager(void);
void mNPS_set_all_schedule_area(void);
typedef struct mQst_not_saved_c {
             s32 work;
             u8 h;
} mQst_not_saved_c;
typedef struct mQst_delivery_c {
             char unk00[0x24];
} mQst_delivery_c;
typedef struct mQst_errand_c {
             char unk00[0x50];
} mQst_errand_c;
typedef enum mPr_SEX {
            mPr_SEX_MALE,
            mPr_SEX_FEMALE,
            mPr_SEX_NUM
} mPr_SEX;
typedef enum mPr_ITEM_COND{
            mPr_ITEM_COND_NORMAL,
            mPr_ITEM_COND_PRESENT,
            mPr_ITEM_COND_QUEST,
            mPr_ITEM_COND_NUM
} mPr_ITEM_COND;
typedef struct Private_Sub_A86 {
               char unk00[0x3];
               u8 unk_03;
               char unk04[0x1];
               u8 unk_05;
               u16 unk_06;
               u8 unk_08;
} Private_Sub_A86;
typedef struct mPr_map_info_c {
             char unk00[0x8];
} mPr_map_info_c;
typedef struct PrivateInventory {
               u16 pockets[15];
               u8 lotto_ticket_expiry_month;
               u8 lotto_ticket_mail_storage;
               u32 item_conditions;
               u32 wallet;
               u32 loan;
} PrivateInventory;
typedef struct Private_c {
                s8 unk000[0x10];
                s8 gender;
                s8 unk011[0x3];
                PrivateInventory inventory;
                mQst_delivery_c deliveries[15];
                mQst_errand_c errands[5];
                s8 unk_3EC[0x2];
                MailHeaderCommon unk_3EE;
                Mail unk_40A[10];
                s8 unk_A72[0x2];
                u8 exists;
                s8 unkA75[0x11];
                Private_Sub_A86 unk_A86;
                s8 unkA8F[0xF8];
                mPr_map_info_c maps[8];
                s8 unkBC8[0x8];
} Private_c;
typedef struct SnowmanData {
               u8 exists;
               u8 headSize;
               u8 bodySize;
               u8 unk3;
} SnowmanData;
typedef struct SnowmanInfo {
               s32 unk0;
               xyz_t position;
} SnowmanInfo;
s32 mSN_check_life(u16* name, s32 daysElapsed);
void mSN_ClearSnowmanData(u16* name, s32 snowmanIndex);
s32 mSN_ClearSnowman(u16* name);
s32 mSN_MeltSnowman(u16* name, s32 daysElapsed);
s32 mSN_get_free_space(void);
void mSN_regist_snowman_society(SnowmanInfo* snowmanInfo);
void mSN_decide_msg(void);
void mSN_snowman_init(void);
struct Actor;
struct ActorOverlay;
struct struct_801161E8_jp;
struct Game_Play_unk_0110_unk_0000;
struct CommonData_unk_1004C_unk_14_arg0;
struct Game_Play;
struct struct_809AEFA4;
typedef s32 (*CommonData_unk_1004C_unk_04)(struct ActorOverlay*, const struct struct_801161E8_jp*, size_t, s32);
typedef s32 (*CommonData_unk_1004C_unk_08)(void);
typedef void* (*CommonData_unk_1004C_unk_0C)(size_t, const struct struct_801161E8_jp*, s32);
typedef s32 (*CommonData_unk_1004C_unk_10)(struct Actor*);
typedef s32 (*CommonData_unk_1004C_unk_14)(struct CommonData_unk_1004C_unk_14_arg0*, u16);
typedef s32 (*CommonData_unk_1004C_unk_BC)(struct Actor*, struct Game_Play*);
typedef void (*CommonData_unk_1004C_unk_C0)(struct Actor*, struct Game_Play*, struct struct_809AEFA4*);
typedef s32 (*CommonData_unk_1004C_unk_C4)(struct Actor*, struct Game_Play*);
typedef s32 (*CommonData_unk_1004C_unk_C8)(struct Actor*, struct Game_Play*);
typedef s32 (*CommonData_unk_1004C_unk_CC)(struct Actor*, struct Game_Play*);
typedef s32 (*CommonData_unk_1004C_unk_D0)(void);
typedef s32 (*CommonData_unk_1004C_unk_E4)(void);
typedef s32 (*CommonData_unk_1004C_unk_EC)(struct Game_Play_unk_0110_unk_0000*, s16, s16);
typedef s32 (*CommonData_unk_1004C_unk_F0)(struct Game_Play_unk_0110_unk_0000*, struct Actor*);
typedef s32 (*CommonData_unk_1004C_unk_F4)(struct Game_Play_unk_0110_unk_0000*, struct Actor*);
typedef s32 (*CommonData_unk_1004C_unk_118)(struct Actor*);

typedef int (*CommonData_unk_1004C_unk_00)(struct Game_Play*, u16, s8, s32, s16, s32, s32, s32, s32);
typedef struct CommonData_unk_1004C {
                CommonData_unk_1004C_unk_00 unk_00;
                CommonData_unk_1004C_unk_04 unk_04;
                CommonData_unk_1004C_unk_08 unk_08;
                CommonData_unk_1004C_unk_0C unk_0C;
                CommonData_unk_1004C_unk_10 unk_10;
                CommonData_unk_1004C_unk_14 unk_14;
                s8 unk_18[0xA4];
                CommonData_unk_1004C_unk_BC unk_BC;
                CommonData_unk_1004C_unk_C0 unk_C0;
                CommonData_unk_1004C_unk_C4 unk_C4;
                CommonData_unk_1004C_unk_C8 unk_C8;
                CommonData_unk_1004C_unk_CC unk_CC;
                CommonData_unk_1004C_unk_D0 unk_D0;
                s8 unk_D4[0x10];
                CommonData_unk_1004C_unk_E4 unk_E4;
                s8 unk_E8[0x4];
                CommonData_unk_1004C_unk_EC unk_EC;
                CommonData_unk_1004C_unk_F0 unk_F0;
                CommonData_unk_1004C_unk_F4 unk_F4;
                s8 unk_F8[0x20];
                CommonData_unk_1004C_unk_118 unk_118;
} CommonData_unk_1004C;
typedef s32 (*CommonData_unk_10078_unk_00)(s32);
typedef s32 (*CommonData_unk_10078_unk_04)(s32);
typedef s32 (*CommonData_unk_10078_unk_08)(s32);
typedef struct CommonData_unk_10078 {
               CommonData_unk_10078_unk_00 unk_00;
               CommonData_unk_10078_unk_04 unk_04;
               CommonData_unk_10078_unk_08 unk_08;
} CommonData_unk_10078;
typedef s32 (*CommonData_unk_10098_unk_4)(struct ActorOverlay*, size_t);
typedef s32 (*CommonData_unk_10098_unk_8)(void);
typedef void* (*CommonData_unk_10098_unk_0C)(void);
typedef s32 (*CommonData_unk_10098_unk_10)(struct Actor*);
typedef s32 (*CommonData_unk_10098_unk_A8)(void*, s32, u16 name, Actor* actor);
typedef s32 (*CommonData_unk_10098_unk_AC)(u16);
typedef s32 (*CommonData_unk_10098_unk_450)(u16);
typedef struct CommonData_unk_10098 {
               s8 unk_00[0x4];
               CommonData_unk_10098_unk_4 unk_4;
               CommonData_unk_10098_unk_8 unk_08;
               CommonData_unk_10098_unk_0C unk_0C;
               CommonData_unk_10098_unk_10 unk_10;
                s8 unk_14[0x94];
                CommonData_unk_10098_unk_A8 unk_A8;
                CommonData_unk_10098_unk_AC unk_AC;
                s32 unk_B0;
                s8 pad[0x450-0xb4];
                CommonData_unk_10098_unk_450 unk_450;
                s32 unk_454;
                s8 pad2[0x86C-0x458];
                s32 unk_86C;
} CommonData_unk_10098;
typedef struct mPr_mother_mail_info_c {
               s8 unk_00[0xE];
} mPr_mother_mail_info_c;
typedef enum Season {
            SPRING,
            SUMMER,
            FALL,
            WINTER,
} Season;
typedef struct Time_c {
               u32 season;
               u32 termIdx;
               s16 bgitemProfile;
               s16 bgitemBank;
               s32 nowSec;
               lbRTC_time_c rtcTime;
               s16 radMin;
               s16 radHour;
               u8 timeSignal;
               u8 underSec;
               u8 disp;
               u8 rtcCrashed;
               s32 rtcEnabled;
               s32 addSec;
               s32 addIdx;
} Time_c;
typedef struct FamicomEmuCommonData {
               s16 unk00;
               s16 unk02;
               s16 unk04;
               s16 unk06;
               s16 unk08;
               s16 unk0A;
               s16 unk0C;
               s16 unk0E;
               s16 unk10;
               s16 unk12;
               s16 unk14;
               s16 unk16;
               s16 unk18;
               s16 unk1A;
               s16 unk1C;
               s16 unk1E;
               s16 unk20;
               s16 unk22;
               s16 unk24;
}FamicomEmuCommonData;
typedef void (*CommonData_100E4_Func)(struct Game_Play*);

typedef void (*unk00)(s32, xyz_t, s32, short, struct Game*, u16, s16, s16);
typedef void (*unk30)(f32*,s16,s16,s16);
typedef void* (*unkAC)(s32);
typedef void (*unkA8)(void*,s32,s32,Actor*);
typedef void* (*unk450)(s32);
typedef struct CommonData_unk_1009C{
    unk00 unk00Proc;
    s8 pad[0x2C];
    unk30 unk30Proc;
}CommonData_unk_1009C;

typedef struct WeatherPrv{
    xyz_t pos;
    xyz_t speed;
    f32 currentY;
    f32 unk1C;
    s16 timer;
    s16 work[5]; 
    u8 use;
    u8 id;
    u8 status;
}WeatherPrv;

typedef struct Weather Weather;

typedef void (*unk4)(Actor*, s16, s16);
typedef void (*ChangeWeatherProc)(Actor* weather, s16 status, s16 level);
typedef int (*GetWeatherPrvvNumProc)(Actor* weather);
typedef void (*RemoveWeatherPrvProc)(Actor* weather, int id);
typedef WeatherPrv* (*GetWeatherPrvProc)(u8 status, s16 timer, xyz_t* pos, xyz_t* speed, Actor* weather, int id);
typedef int (*WeatherSoundEffectProc)();
typedef void (*ChangeWeatherInstanceProc)(Weather* weather, s16 status, s16 level);

typedef struct WeatherClip{
    Weather* actor;
    unk4 unk4proc;
    ChangeWeatherProc changeWeather;
    GetWeatherPrvvNumProc getPrvNum;
    RemoveWeatherPrvProc removePrv;
    GetWeatherPrvProc getPrv;
    WeatherSoundEffectProc stopSound;
    WeatherSoundEffectProc startSound;
    ChangeWeatherInstanceProc changeWeatherInstance;
}WeatherClip; 


typedef void (*MakeWeatherProc)(Actor*,Game*);
typedef void (*CtWeatherProc)(WeatherPrv*, Game*);
typedef void (*MvWeatherProc)(WeatherPrv*, Game*);
typedef void (*StWeatherProc)(Game*);
typedef void (*DwWeatherProc)(WeatherPrv*, Game*);

typedef struct WeatherProfile{
    MakeWeatherProc mk;
    CtWeatherProc ct;
    MvWeatherProc mv;
    StWeatherProc st;
    DwWeatherProc dw;
}WeatherProfile;

typedef struct WeatherOvlInfo {
    /* 0x00 */ RomOffset vromStart;
    /* 0x04 */ RomOffset vromEnd;
    /* 0x08 */ void* vramStart;
    /* 0x0C */ void* vramEnd;
    /* 0x10 */ void* ovlTypeEnd;
} WeatherOvlInfo; // size = 0x14

typedef struct WeatherDmaInfo{
    /* 0x00 */ RomOffset vromStart;
    /* 0x04 */ RomOffset vromEnd;
}WeatherDmaInfo;

WeatherOvlInfo weatherOvlTable[4];

WeatherDmaInfo* weatherDmaTable[5];
void ovlmgr_Load(RomOffset vromStart, RomOffset vromEnd, void* vramStart, void* vramEnd, void* allocatedRamAddr);
s32 DmaMgr_RequestSyncDebug(void* vram, RomOffset vrom, size_t size, const char* filename, s32 line);
#define FI_GET_TYPE(field_id) ((field_id) & 0xF000)
struct Weather{
    /* 0x000 */ Actor actor;
    /* 0x174 */ WeatherProfile* currentProfile; 
    /* 0x178 */ s16 currentStatus;
    /* 0x17A */ s16 nextStatus;
    /* 0x17C */ s16 counter;
    /* 0x17E */ s16 currentLevel;
    /* 0x180 */ s16 currentAimLevel;
    /* 0x182 */ s16 nextLevel;
    /* 0x184 */ u8* segment;
    /* 0x188 */ xyz_t pos;
    /* 0x194 */ WeatherPrv* priv;
    /* 0x198 */ void* allocatedSize; 
    /* 0x19C */ u8 requestChange;
    /* 0x19E */ s16 unk19E;
    /* 0x1A0 */ WeatherClip clip; 
    /* 0x1C4 */ s16 timer;
    /* 0x1C6 */ s16 timer2;
    /* 0x1C8 */ xyz_t windInfo;
    /* 0x1D4 */ s16 lightningTimer;
    /* 0x1D6 */ s16 lightningTimer2;
    /* 0x1D8 */ s16 currentSoundEffect;
    /* 0x1DA */ s16 umbrellaFlag;
    /* 0x1DC */ s16 currentYAngle;
    /* 0x1DE */ s16 soundFlag;
    /* 0x1E0 */ s16 startSoundEffect;
    /* 0x1E2 */ s16 stopSoundEffect;
    /* 0x1E4 */ s16 basementEvent;
}; 
void sAdo_SysTrgStart(u16);
#define RANDOM_F(n) (fqrand() * (f32)(n))
typedef struct CommonData {
                  u8 unk00000[0x14];
                  s32 sceneNo;
                  u8 nowNpcMax;
                  u8 removeAnimalIdx;
                  u8 unk1A[0x20 - 0x1A];
                  Private_c private[4];
                  mLd_land_info_c land_info;
                  u8 unk02F6A[0x61E];
                  mHm_hs_c homes[4];
                  mFM_fg_c fg[(10 - 4)][(7 - 2)];
                  u8 unk09EA8[0x70];
                  Animal_c animals[15];
                  u8 unk0EC70[0x134];
                  mEv_event_save_c event_save_data;
                  u8 unk0EE40[0x118];
                  u16 fruit;
                  s8 unk_0EF5A[0x12];
                  Mail unk_0EF6C[5];
                  s8 unk_0F2A0[0x17C];
                  SnowmanData snowmanData[3];
                  s8 unk_F428[0x10];
                  u8 station_type;
                  u8 saveWeather;
                  u8 unk0F439[0x2];
                  u16 deposit[(7 - 2) * (10 - 4)][16];
                  lbRTC_time_c unk_0F7FC;
                  mPr_mother_mail_info_c mother_mail[4];
                  u8 unk0F83C[0x8];
                  FamicomEmuCommonData famicom_emu_common_data;
                  u8 unk0F86A[0x32];
                  lbRTC_time_c unk_0F89C;
                  lbRTC_time_c unk_0F8A4;
                  s8 unk0F8AC;
                  u8 snowmanYear;
                  u8 snowmanMonth;
                  u8 snowmanDay;
                  u8 snowmanHour;
                  s8 unk0F8B1[0x74F];
                  u8 unk_10000;
                  u8 unk_10001;
                  u8 unk10002[0x1];
                  u8 player_no;
                  s32 unk_10004;
                  s8 unk_10008[0x44];
                  CommonData_unk_1004C *unk_1004C;
                  s8 unk_10050[0x28];
                  CommonData_unk_10078 *unk_10078;
                  s8 unk_1007C[8];
                  WeatherClip* weatherClip;
                  s8 unk_10088[0x10];
                  CommonData_unk_10098 *unk_10098;
                  CommonData_unk_1009C* unk1009C; 
                  u8 unk100A0[0x44];
                  CommonData_100E4_Func* unk_100E4;
                  u8 unk100E8[0x24];
                  Time_c time;
                  Private_c* now_private;
                  u8 unk1013C[0x4];
                  u8 unk_10140;
                  u8 fish_location;
                  u8 unk10142[0x7];
                  u8 unk_10149;
                  u8 unk_1014A;
                  u8 unk_1014B;
                  s8 unk_1014C[0x2];
                  s16 unk_1014E;
                  s8 unk_10150[0x10];
                  NpcList npclist[15];
                  u16 house_owner_name;
                  u16 last_field_id;
                  s8 unk_104AC[0x1];
                  u8 unk_104AD;
                  s8 unk_104AE[0x2];
                  s8 unk_104B0[0xBC];
                  /* 0x01056C */ s16 weather;
                  /* 0x01056E */ s16 weatherIntensity;
                  /* 0x010570 */ lbRTC_time_c weatherTime;
                  /* 0x010578 */ s_xyz wind;
                  /* 0x010580 */ f32 windSpeed;
                  s8 unk10584[0x14];
                  mQst_not_saved_c quest;
                  u32 scene_from_title_demo;
                  NpsSchedule npcSchedule[15];
                  NpcWalking npcWalk;
                  s8 unk_10710[0x3C];
                  s32 unk_1074C;
                  s8 unk_10750[0x66];
                  s16 unk_107B6;
                  u8 unk107B8[0x28];
                  s8 player_decoy_flag;
                  u8 unk107E1[0x3];
                  s16 unk_107E4;
                  u8 unk107E6[0x254];
                  u8 goki_shocked_flag;
                  s8 unk_10A3B[0x1];
                  s8 unk_10A3C[0x3];
                  u8 trainExists;
                  s8 unk_10A40[0x4];
                  u8 currentTrainAction;
                  s8 unk_10A48[0x4];
                  f32 currentTrainSpeed;
                  xyz_t currentTrainPosition;
                  s8 unk_10A5C[0x12];
                  u8 unk_10A68;
                  s8 unk_10A69[0x3];
                  s8 unk_10A6C[0x14];
                  s8 unk_10A80[0x2];
                  s16 unk_10A82;
                  u8 unk10A84[0x2C];
                  u8 unk_10AB0;   
                  s8 unk_10AB1;
                  s16 currentSoundEffect;
                  s8 unk_10AB3[0x7];
} CommonData;
void common_data_reinit(void);
void common_data_init(void);
void common_data_clear(void);

extern CommonData common_data;
struct Game_Play;
struct GraphicsContext;

#define UNK_TYPE s32
#define UNK_TYPE1 s8
#define UNK_TYPE2 s16
typedef struct Game_Play1938 {
    /* 0x000 */ UNK_TYPE1 unk_000[0x8];
    /* 0x008 */ s32 unk_008;
    /* 0x00C */ s32 unk_00C;
    /* 0x010 */ s32 unk_010;
    /* 0x018 */ s32 unk_014;
    /* 0x01C */ UNK_TYPE1 unk_018[0x10];
    /* 0x028 */ xyz_t unk_028;
    /* 0x034 */ UNK_TYPE1 unk_034[0x1C];
    /* 0x050 */ Vp vp;
    /* 0x060 */ Mtx unk_060;
    /* 0x0A0 */ Mtx unk_0A0;
    /* 0x0E0 */ UNK_TYPE1 unk_0E0[0x40];
    /* 0x120 */ s32 unk_120;
    /* 0x124 */ UNK_TYPE1 unk_124[0x4];
} Game_Play1938; // size = 0x128
typedef struct ScissorViewArg1 {
               s32 unk_00;
               s32 unk_04;
               s32 unk_08;
               s32 unk_0C;
} ScissorViewArg1;
void initView(Game_Play1938* arg0, struct GraphicsContext* gfxCtx);
void setScissorView(Game_Play1938* arg0, ScissorViewArg1* arg1);
void showView(Game_Play1938* arg0, s32 arg1, struct Game_Play* game_play);
void showView1(Game_Play1938* arg0, s32 arg1, Gfx** gfx);
typedef struct {
              u8 r;
              u8 g;
              u8 b;
} Color_RGB8;
typedef struct {
              u8 r;
              u8 g;
              u8 b;
              u8 a;
} Color_RGBA8;
typedef struct {
              s16 r;
              s16 g;
              s16 b;
} Color_RGB16;
typedef union {
    struct {
        u8 r, g, b, a;
    };
    u32 rgba;
} Color_RGBA8_u32;
typedef struct {
    f32 r, g, b, a;
} Color_RGBAf;
typedef struct {
    u32 r, g, b, a;
} Color_RGBAu32;
typedef union {
    struct {
        u16 r : 5;
        u16 g : 5;
        u16 b : 5;
        u16 a : 1;
    };
    u16 rgba;
} Color_RGBA16;
typedef union {
    struct {
        u32 r : 5;
        u32 g : 5;
        u32 b : 5;
        u32 a : 1;
    };
    u16 rgba;
} Color_RGBA16_2;
typedef union{
    struct {
        u32 r : 3;
        u32 g : 3;
        u32 b : 3;
        u32 a : 5;
    };
    u16 rgba;
} Color_RGBA14;
struct GraphicsContext;
struct Game_Play;
typedef struct LightPoint {
              s16 x;
              s16 y;
              s16 z;
              u8 color[3];
              u8 drawGlow;
              s16 radius;
} LightPoint;
typedef struct LightDiffuse {
              s8 x;
              s8 y;
              s8 z;
              u8 color[3];
} LightDiffuse;
typedef union {
    LightPoint point;
    LightDiffuse diffuse;
} LightParams;
typedef struct Lights {
               u8 type;
               LightParams lights;
} Lights;
typedef struct LightNode {
              Lights* info;
              struct LightNode* prev;
              struct LightNode* next;
} LightNode;
typedef struct LightBuffer {
               s32 current;
               s32 idx;
               LightNode lights[32];
} LightBuffer;
typedef struct LightsN {
               u8 diffuse_count;
               Lightsn lights;
}LightsN;
typedef struct Global_light {
              LightNode* list;
              u8 ambientColor[3];
              u8 fogColor[3];
              s16 fogNear;
              s16 fogFar;
} Global_light;
typedef void (*light_point_proc)(LightsN*, LightParams*, xyz_t*);
typedef void (*light_P_point_proc)(LightsN*, LightParams*, xyz_t*);
void point_data_set(Lights* lights, s16 x, s16 y, s16 z, u8 r, u8 g, u8 b, s16 radius, s32 type);
void Light_point_ct(Lights* lights, s16 x, s16 y, s16 z, u8 r, u8 g, u8 b, s16 radius);
void Light_point2_ct(Lights* lights, s16 x, s16 y, s16 z, u8 r, u8 g, u8 b, s16 radius);
void Light_point_color_set(Lights* lights, u8 r, u8 g, u8 b, s16 radius);
void Light_diffuse_ct(Lights* lights, s8 x, s8 y, s8 z, u8 r, u8 g, u8 b);
void LightsN_ct(LightsN* lights, u8 r, u8 g, u8 b);
void LightsN_disp(LightsN* lights, struct GraphicsContext* gfxCtx);
Light* LightsN_new_diffuse(LightsN* lights);
void LightsN__point_proc(LightsN* lights, LightParams* lightInfo, xyz_t* point);
void LightsN__P_point_proc(LightsN* lights, LightParams* lightInfo, xyz_t* pos);
void LightsN__diffuse_proc(LightsN* lights, LightParams* lightInfo, xyz_t* pos);
void LightsN_list_check(LightsN* lights, LightNode* node, xyz_t* pos);
LightNode* Light_list_buf_new(void);
void Light_list_buf_delete(LightNode* lightNode);
void Global_light_ct(Global_light* glight);
void Global_light_ambient_set(Global_light* glight, u8 r, u8 g, u8 b);
void Global_light_fog_set(Global_light* glight, u8 r, u8 g, u8 b, s16 near, s16 far);
LightsN* Global_light_read(Global_light* glight, struct GraphicsContext* gfxCtx);
void Global_light_list_ct(Global_light* glight);
void Global_light_list_dt(Global_light* glight);
LightNode* Global_light_list_new(struct Game_Play* play, Global_light* glight, Lights* light);
void Global_light_list_delete(Global_light* glight, LightNode* lightNode);
LightsN* new_Lights(struct GraphicsContext* gfxCtx, u8 ambient_r, u8 ambient_g, u8 ambient_b, u8 count, u8 light_r,
                          u8 light_g, u8 light_b, s8 dir_x, s8 dir_y, s8 dir_z);
LightsN* new_LightsN(struct GraphicsContext* gfxCtx, u8 r, u8 g, u8 b);
void Light_list_point_draw(struct Game_Play* game_play);
typedef struct mCoBG_OffsetTable {
               u8 unitAttribute;
               s8 crOffset;
               s8 luOffset;
               s8 ldOffset;
               s8 rdOffset;
               s8 ruOffset;
               s8 slateSwitch;
} mCoBG_OffsetTable;
void func_8006BB64_jp(void);
void mCoBG_CalcTimerDecalCircle(void);
void func_8006C8D0_jp(void);
f32 mCoBG_GetBgY_OnlyCenter_FromWpos2(xyz_t arg0, f32 arg1);
s32 mCoBG_SearchWaterLimitDistN(s32* arg0, xyz_t arg1, s16 arg4, f32 arg5, s32 arg6);
void mCoBG_SetPluss5PointOffset_file(xyz_t pos, mCoBG_OffsetTable offsetData, char* file, s32 line);
typedef struct Pause {
              s32 enabled;
              s32 timer;
} Pause;
void Pause_ct(Pause* pause);
s32 Pause_proc(Pause* pause, Input* input);
struct Game_Play;
typedef struct Game_Play2128 {
               char unk00[0x10];
} Game_Play2128;
void fbdemo_fade_init(void* arg0);
void fbdemo_fade_move(void* arg0, u8 arg1);
void fbdemo_fade_draw(void* arg0, Gfx** gfx);
void fbdemo_fade_startup(void* arg0);
void fbdemo_fade_settype(void* arg0, s32 arg1);
void fbdemo_fade_setcolor_rgba8888(void* arg0, s32 arg1);
s32 fbdemo_fade_is_finish(void* arg0, struct Game_Play* game_play);
struct Game_Play;
struct Game_Play_unk_0110_unk_0000;
struct Game_Play;

typedef struct ActorEntry {
    /* 0x0 */ s16 id;
    /* 0x2 */ s_xyz pos;
    /* 0x8 */ s_xyz rot;
    /* 0xE */ s16 params;
} ActorEntry; // size = 0x10

typedef struct ObjectStatus {
    /* 0x00 */ s16 id;
    /* 0x04 */ void* segment;
    /* 0x08 */ s8 unk_08;
    /* 0x0C */ uintptr_t vrom;
    /* 0x10 */ size_t size;
    /* 0x14 */ s8 unk14;
    /* 0x18 */ char pad18[0x1C];
    /* 0x34 */ OSMesgQueue unk34;
    /* 0x4C */ s8 unk4C;
    /* 0x50 */ s16 unk50;
    /* 0x52 */ u8 unk52;
    /* 0x53 */ u8 unk53;
} ObjectStatus; // size = 0x54

#define OBJECT_EXCHANGE_BANK_MAX 73 // Name taken from oot debug string

typedef struct ObjectExchangeBank {
    /* 0x0000 */ ObjectStatus status[OBJECT_EXCHANGE_BANK_MAX]; // Name taken from oot debug string
    /* 0x17F4 */ s32 num; // Name taken from oot debug string
    /* 0x17F8 */ UNK_TYPE unk17F8;
    /* 0x17FC */ UNK_TYPE unk17FC;
    /* 0x1800 */ UNK_TYPE unk1800;
    /* 0x1804 */ UNK_TYPE unk1804;
    /* 0x1808 */ UNK_TYPE unk1808;
    /* 0x180C */ UNK_TYPE unk180C;
    /* 0x1810 */ UNK_TYPE unk1810;
    /* 0x1814 */ UNK_TYPE unk1814;
    /* 0x1818 */ UNK_PTR unk1818;
    /* 0x181C */ UNK_TYPE unk181C;
    /* 0x1820 */ UNK_TYPE unk1820;
} ObjectExchangeBank; // size = 0x1824

typedef struct {
    /* 0x0000 */ u8 unk0;
    /* 0x0001 */ u8 unk1;
    /* 0x0004 */ UNK_PTR unk4;
} MSceneUnkStruct;

s32 func_800C59B0_jp(ObjectExchangeBank* objectExchangeBank);
s32 func_800C5A08_jp(ObjectExchangeBank* objectExchangeBank);
s32 func_800C5A60_jp(ObjectExchangeBank* objectExchangeBank);
s32 func_800C5AA0_jp(ObjectStatus* objectStatus, ObjectExchangeBank* objectExchangeBank, s16 objectBankIndex);
void func_800C5B30_jp(ObjectStatus* objectStatus);
void mSc_clear_bank_status(ObjectStatus* objectStatus);
s32 func_800C5B74_jp(ObjectExchangeBank* objectExchangeBank, s16 id);
void Object_Exchange_keep_new_Player(s32 arg0);
u32 mSc_secure_exchange_keep_bank(ObjectExchangeBank* objectExchangeBank, s16 id, s32 size);
void func_800C5D68_jp(ObjectExchangeBank*);
void func_800C5E10_jp(ObjectExchangeBank* objectExchangeBank);
// void func_800C5EA0_jp();
s32 func_800C5F0C_jp(ObjectStatus* objectStatus, ObjectExchangeBank* objectExchangeBank);
void mSc_dmacopy_data_bank(ObjectExchangeBank* objectExchangeBank);
s32 mSc_bank_regist_check(ObjectExchangeBank* objectExchangeBank, s16 id);
s32 func_800C6144_jp(ObjectExchangeBank* objectExchangeBank, s16 arg1);
void mSc_regist_initial_exchange_bank(struct Game_Play* game_play);
void func_800C62C4_jp(ObjectStatus*, ObjectExchangeBank*, s32);
void mSc_dmacopy_all_exchange_bank(ObjectExchangeBank* objectExchangeBank);
void mSc_data_bank_ct(struct Game_Play* game_play, ObjectExchangeBank* objectExchangeBank);
void mSc_decide_exchange_bank(ObjectExchangeBank* objectExchangeBank);
// void func_800C6690_jp();
void Scene_ct(struct Game_Play* game_play, void* arg1);
// void func_800C6960_jp();
void Scene_Proc_Ctrl_Actor_Ptr(struct Game_Play* game_play, MSceneUnkStruct* arg1);
void Scene_Proc_Actor_Ptr(struct Game_Play* game_play, MSceneUnkStruct* arg1);
void Scene_Proc_Object_Exchange_Bank_Ptr(struct Game_Play* game_play, MSceneUnkStruct* arg1);
void Scene_Proc_Door_Data_Ptr(struct Game_Play* game_play, MSceneUnkStruct* arg1);
void Door_info_ct(s8* arg0);
void Scene_Proc_Sound(s8 arg0, s8 arg1);
// void func_800C6AE0_jp();
// void func_800C6B50_jp();
// void func_800C6BB0_jp();
// void func_800C6BD4_jp();
// void func_800C6BF8_jp();
// void func_800C6C10_jp();
// void func_800C6D14_jp();
// void func_800C6D5C_jp();
// void func_800C6E14_jp();

struct Actor;
struct ActorEntry;
struct Game_Play;

typedef void (*Game_Play_unk_1C58)(struct Actor*);
typedef UNK_RET (*Game_Play_unk_2208)(struct Actor*, struct Game_Play*);

typedef void (*Game_PlayUnkFunc_00)(void*);
typedef void (*Game_PlayUnkFunc_04)(void*, struct Game_Play*);
typedef void (*Game_PlayUnkFunc_08)(void*, u8);
typedef void (*Game_PlayUnkFunc_0C)(void*, Gfx**);
typedef void (*Game_PlayUnkFunc_10)(void*);
typedef void (*Game_PlayUnkFunc_14)(void*, s32);
typedef void (*Game_PlayUnkFunc_18)(void*, s32);
typedef void (*Game_PlayUnkFunc_1C)(void);
typedef s32 (*Game_PlayUnkFunc_20)(void*, struct Game_Play*);

typedef struct Game_PlayUnkFuncsStruct {
    /* 0x00 */ Game_PlayUnkFunc_00 unk_00;
    /* 0x04 */ Game_PlayUnkFunc_04 unk_04;
    /* 0x08 */ Game_PlayUnkFunc_08 unk_08;
    /* 0x0C */ Game_PlayUnkFunc_0C unk_0C;
    /* 0x10 */ Game_PlayUnkFunc_10 unk_10;
    /* 0x14 */ Game_PlayUnkFunc_14 unk_14;
    /* 0x18 */ Game_PlayUnkFunc_18 unk_18;
    /* 0x1C */ Game_PlayUnkFunc_1C unk_1C;
    /* 0x20 */ Game_PlayUnkFunc_20 unk_20;
} Game_PlayUnkFuncsStruct; // size = 0x24

typedef struct Game_Play_Unk_1EE8 {
    /* 0x000 */ char unk000[0x218];
    /* 0x218 */ s32 unk_218;
    /* 0x21C */ Game_PlayUnkFuncsStruct unk_21C;
} Game_Play_Unk_1EE8; // size = 0x240

typedef struct Camera2{
    xyz_t unk0;
    xyz_t unk4;
    u8 pad[0x120];
}Camera2;

typedef struct Game_Play {
    /* 0x0000 */ Game state;
    /* 0x00E0 */ s16 unk_00E0;
    /* 0x00E2 */ s8 unk_00E2[0x2];
    /* 0x00E4 */ s8 unk_00E4;
    /* 0x00E5 */ s8 unk_00E5;
    /* 0x00E6 */ s8 unk_00E6[0x2];
    /* 0x00E8 */ s8 unk_00E8[0x24];
    /* 0x010C */ void* unk_010C;
    /* 0x0110 */ ObjectExchangeBank objectExchangeBank;
    /* 0x1938 */ Game_Play1938 unk_1938;
    /* 0x1A60 */ Camera2 camera;
    /* 0x1B98 */ Kankyo kankyo;
    /* 0x1C60 */ Global_light glight;
    /* 0x1C70 */ Pause pause;
    /* 0x1C78 */ ActorInfo actorInfo;
    /* 0x1CBC */ Submenu submenu;
    /* 0x1DAC */ s8 unk_1DAC;
    /* 0x1DAD */ s8 unk_1DAD[0x3];
    /* 0x1DB0 */ s8 unk_1DB0[0x10];
    /* 0x1DC0 */ PreRender unk_1DC0;
    /* 0x1E10 */ s32 unk_1E10;
    /* 0x1E14 */ UNK_PTR unk_1E14;
    /* 0x1E18 */ s32 unk_1E18;
    /* 0x1E1C */ MtxF viewProjectionMtxF;
    /* 0x1E5C */ MtxF billboardMtxF;
    /* 0x1E9C */ Mtx* unk_1E9C;
    /* 0x1EA0 */ s32 unk_1EA0;
    /* 0x1EA4 */ s8 unk_1EA4[0x1];
    /* 0x1EA5 */ u8 unk_1EA5;
    /* 0x1EA6 */ u8 unk_1EA6;
    /* 0x1EA7 */ u8 unk_1EA7;
    /* 0x1EA8 */ struct ActorEntry* unk_1EA8;
    /* 0x1EAC */ struct ActorEntry* unk_1EAC;
    /* 0x1EB0 */ s16* unk_1EB0;
    /* 0x1EB4 */ s16* unk_1EB4;
    /* 0x1EB8 */ s32 unk_1EB8;
    /* 0x1EBC */ Event event;
    /* 0x1ECC */ s8 unk_1ECC[0x14];
    /* 0x1EE0 */ u8 unk_1EE0;
    /* 0x1EE1 */ u8 unk_1EE1;
    /* 0x1EE2 */ u8 unk_1EE2;
    /* 0x1EE3 */ u8 unk_1EE3;
    /* 0x1EE4 */ s8 unk_1EE4[0x4];
    /* 0x1EE8 */ Game_Play_Unk_1EE8 unk_1EE8;
    /* 0x2128 */ Game_Play2128 unk_2128;
    /* 0x2138 */ CollisionCheck unk_2138;
    /* 0x2208 */ Game_Play_unk_2208 unk_2208;
    /* 0x220C */ s32 unk_220C;
    /* 0x2210 */ Struct_8010EAA0* unk_2210;
    /* 0x2214 */ s8 unk_2214[0x1FC];
} Game_Play; // size = 0x2410

void play_init(Game* game);
void play_cleanup(Game* game);
Mtx* _Matrix_to_Mtx_new(struct GraphicsContext* gfxCtx);

extern uintptr_t gSegments[NUM_SEGMENTS];
#define RDRAM_CACHED 0x80000000



// TODO: After uintptr_t cast change should have an AVOID_UB target that just toggles the KSEG0 bit in the address rather than add/sub 0x80000000
#define PHYSICAL_TO_VIRTUAL(addr) ((uintptr_t)(addr) + RDRAM_CACHED)
#define VIRTUAL_TO_PHYSICAL(addr) (uintptr_t)((u8*)(addr) - RDRAM_CACHED)
#define SEGMENTED_TO_VIRTUAL(addr) (void*)(PHYSICAL_TO_VIRTUAL(gSegments[SEGMENT_NUMBER(addr)]) + SEGMENT_OFFSET(addr))


s16 atans_table(f32,f32); 
f32 mCoBG_CheckBallRollingArea(s16 angle, const xyz_t* wpos);
extern s32 mCoBG_CheckAttribute_BallRolling(s16* angles, const xyz_t* wpos);
f32 mCoBG_GetBgY_AngleS_FromWpos(s_xyz*, xyz_t, f32);
extern int mFI_Wpos2BkandUtNuminBlock(int* bx, int* bz, int* ut_x, int* ut_z, xyz_t wpos);
extern int mNpc_CheckNpcSet(int bx, int bz, int ut_x, int ut_z);
extern void mFI_BkandUtNum2CenterWpos(xyz_t* wpos, int bx, int bz, int ut_x, int ut_z);
#define FI_UT_WORLDSIZE_Z 40
#define FI_UT_WORLDSIZE_Z_F (f32)FI_UT_WORLDSIZE_Z
#define FI_UT_WORLDSIZE_X 40
#define FI_UT_WORLDSIZE_X_F (f32)FI_UT_WORLDSIZE_Z
#define FI_UT_BASE_SIZE 40
#define FI_UT_BASE_SIZE_F ((f32)FI_UT_BASE_SIZE)

u16* mFI_GetUnitFG(xyz_t);
typedef s32 (*SetMgrGetEndPosProc)(struct Game*, xyz_t*);

typedef struct Player {
    /* 0x0000 */ Actor actor;
    /* 0x0174 */ s8 unk_0174[0xB7C];
    /* 0x0CF0*/ s32 unk_0CF0;
    /* 0x0CF4*/ s32 unk_0CF4;
    /* 0x0CF8 */ s8 unk_0CF8[0x8];
    /* 0x0D00 */ s32 unk_0D00;
    /* 0x0D04 */ s32 unk_0D04;
    /* 0x0D08 */ s32 unk_0D08;
    /* 0x0D0C */ s8 unk_0D0C[0xA0];
    /* 0x0DAC */ s32 unk_0DAC;
    /* 0x0DB0 */ s32 unk_0DB0;
    /* 0x0DB4 */ s8 unk_0DB4[0x8];
    /* 0x0DBC */ s32 unk_0DBC[2];
    /* 0x0DC4 */ s8 unk_0DC4[0x18];
    /* 0x0DDC */ s32 unk_0DDC[2];
    /* 0x0DE4 */ s32 unk_0DE4[2];
    /* 0x0DEC */ s8 unk_0DEC[0x160];
    /* 0x0F4C */ ClObjTris colliderTris1;
    /* 0x0F60 */ s8 unk_0F60[0x44];
    /* 0x0FA4 */ ClObjTris colliderTris2;
    /* 0x0FB8 */ s8 unk_0FB8[0x50];
    /* 0x1008 */ ClObjPipe colliderPipe;
    /* 0x1024 */ s8 unk_1024[0x220];
    /* 0x1024 */ SetMgrGetEndPosProc getEndPos;
    /* 0x1048 */ s8 unk_1048[0x70];
    /* 0x12B8 */ s32 unk_12B8;
    /* 0x12BC */ s32 unk_12BC;
    /* 0x12C0 */ u16 unk_12C0;
    /* 0x12C2 */ s8 unk_12C2[0x2];
    /* 0x12C4 */ s8 unk_12C4[0x14];
} Player; // size = 0x12D8
extern f32 Math3d_normalizeXyz_t(xyz_t* vec);
 Player* get_player_actor_withoutCheck(struct Game_Play* game_play);
extern f32 search_position_distance(const xyz_t* const pos, const xyz_t* const target);
void add_calc0(f32* pValue, f32 fraction, f32 step);

#define ENV_SAVE_GET_WEATHER_TYPE(w) (((w) & 0xF0) >> 4) 
#define ENV_SAVE_GET_WEATHER_INTENSITY(w) ((w) & 0xF)

s32 mEnv_DecideWindDirect(s_xyz*, s16, s16);

xyz_t* Camera2_getCenterPos_p();

typedef enum {
    /* 0 */ DEBUG_REG_START = 0,
    /* 0 */ DEBUG_REG_REG = DEBUG_REG_START, 
    /* 1 */ DEBUG_REG_SREG,
    /* 2 */ DEBUG_REG_OREG,
    /* 3 */ DEBUG_REG_PREG,
    /* 4 */ DEBUG_REG_QREG,
    /* 5 */ DEBUG_REG_MREG,
    /* 6 */ DEBUG_REG_SBREG,
    /* 7 */ DEBUG_REG_DREG,
    /* 8 */ DEBUG_REG_UREG,
    /* 9 */ DEBUG_REG_IREG,
    /* 10 */ DEBUG_REG_ZREG,
    /* 11 */ DEBUG_REG_CRV,
    /* 12 */ DEBUG_REG_NS1,
    /* 13 */ DEBUG_REG_SND,
    /* 14 */ DEBUG_REG_XREG,
    /* 15 */ DEBUG_REG_CRV2,
    /* 16 */ DEBUG_REG_DEMOREG,
    /* 17 */ DEBUG_REG_TREG,
    /* 18 */ DEBUG_REG_WREG,
    /* 19 */ DEBUG_REG_AREG,
    /* 20 */ DEBUG_REG_VREG,
    /* 21 */ DEBUG_REG_HREG,
    /* 22 */ DEBUG_REG_GREG,
    /* 23 */ DEBUG_REG_mREG,
    /* 24 */ DEBUG_REG_nREG,
    /* 25 */ DEBUG_REG_BREG,
    /* 26 */ DEBUG_REG_DORO,
    /* 27 */ DEBUG_REG_kREG,
    /* 28 */ DEBUG_REG_BAK,
    /* 29 */ DEBUG_REG_PLAYERREG,
    /* 30 */ DEBUG_REG_NMREG,

    /* 31 */ DEBUG_REG_NIIREG,
    /* 32 */ DEBUG_REG_GENREG,
    /* 33 */ DEBUG_REG_MYKREG,
    /* 34 */ DEBUG_REG_CAMREG,
    /* 35 */ DEBUG_REG_SAKREG,
    /* 36 */ DEBUG_REG_TAKREG,
    /* 37 */ DEBUG_REG_PL2REG,

    /* 38 */ DEBUG_REG_MAX
} DEBUG_REG;

#define DEBUG_REG_SIZE 16
#define DEBUG_REG_GROUP 6

#define DEBUG_REG_COUNT (DEBUG_REG_SIZE * DEBUG_REG_GROUP * DEBUG_REG_MAX)

typedef struct Debug_mode {
    /* 0x00 */ u8 mode;
    /* 0x01 */ u8 type;
    /* 0x02 */ s8 inputR;
    /* 0x03 */ s8 keyWait;

    /* 0x04 */ s32 oldKey;
    /* 0x08 */ s32 pad[3];

    /* 0x14 */ s16 r[DEBUG_REG_SIZE * DEBUG_REG_GROUP * DEBUG_REG_MAX];
} Debug_mode; // size = 0x1C94

extern Debug_mode* debug_mode;


#define REGADDR(reg, idx) (debug_mode->r[DEBUG_REG_SIZE * DEBUG_REG_GROUP * DEBUG_REG_##reg + (idx)])

#define GETREG(reg, idx) (REGADDR(reg, idx))

#define REG(r) GETREG(REG,r)
#define SREG(r) GETREG(SREG,r)
#define OREG(r) GETREG(OREG,r)
#define PREG(r) GETREG(PREG,r)
#define QREG(r) GETREG(QREG,r)
#define MREG(r) GETREG(MREG,r)
#define SBREG(r) GETREG(SBREG,r)
#define DREG(r) GETREG(DREG,r)
#define UREG(r) GETREG(UREG,r)
#define IREG(r) GETREG(IREG,r)
#define ZREG(r) GETREG(ZREG,r)
#define CRV(r) GETREG(CRV,r)
#define NS1(r) GETREG(NS1,r)
#define SND(r) GETREG(SND,r)
#define XREG(r) GETREG(XREG,r)
#define CRV2(r) GETREG(CRV2,r)
#define DEMOREG(r) GETREG(DEMOREG,r)
#define TREG(r) GETREG(TREG,r)
#define WREG(r) GETREG(WREG,r)
#define AREG(r) GETREG(AREG,r)
#define VREG(r) GETREG(VREG,r)
#define HREG(r) GETREG(HREG,r)
#define GREG(r) GETREG(GREG,r)
#define mREG(r) GETREG(mREG,r)
#define nREG(r) GETREG(nREG,r)
#define BREG(r) GETREG(BREG,r)
#define DORO(r) GETREG(DORO,r)
#define kREG(r) GETREG(kREG,r)
#define BAK(r) GETREG(BAK,r)
#define PLAYERREG(r) GETREG(PLAYERREG,r)
#define NMREG(r) GETREG(NMREG,r)
#define NIIREG(r) GETREG(NIIREG,r)
#define GENREG(r) GETREG(GENREG,r)
#define MYKREG(r) GETREG(MYKREG,r)
#define CAMREG(r) GETREG(CAMREG,r)
#define SAKKREG(r) GETREG(SAKKREG,r)
#define TAKREG(r) GETREG(TAKREG,r)
#define PL2REG(r) GETREG(PL2REG,r)


typedef void (*FaultClientCallback)(void*, void*);

// TODO: taken from MM, needs to be verified
typedef struct FaultClient {
    /* 0x0 */ struct FaultClient* next;
    /* 0x4 */ FaultClientCallback callback;
    /* 0x8 */ void* arg0;
    /* 0xC */ void* arg1;
} FaultClient; // size = 0x10


typedef uintptr_t (*FaultAddrConvClientCallback)(uintptr_t, void*);

// TODO: taken from MM, needs to be verified
typedef struct FaultAddrConvClient {
    /* 0x0 */ struct FaultAddrConvClient* next;
    /* 0x4 */ FaultAddrConvClientCallback callback;
    /* 0x8 */ void* arg;
} FaultAddrConvClient; // size = 0xC

void fault_AddClient(FaultClient* client, FaultClientCallback callback, void* arg0, void* arg1);
void fault_RemoveClient(FaultClient* client);
void fault_AddressConverterAddClient(FaultAddrConvClient* client, FaultAddrConvClientCallback callback, void* arg);
void fault_AddressConverterRemoveClient(FaultAddrConvClient* client);
void fault_WaitForInput(void);
void fault_FillScreenBlack(void);
void fault_SetFrameBuffer(void* fb, u16 w, u16 h);
void fault_Init(void);
NORETURN void fault_AddHungupAndCrashImpl(const char* exp1, const char* exp2);
NORETURN void fault_AddHungupAndCrash(const char* file, s32 line);

#define WORK_DISP __gfxCtx->work.p
#define SHADOW_DISP __gfxCtx->shadow.p
#define FONT_DISP __gfxCtx->font.p

typedef struct Game_Prenmi {
    /* 0x00 */ Game state;
    /* 0xE0 */ UNK_TYPE1 unk00[0xE8 - 0xE0];
} Game_Prenmi; // size = 0xE8

void func_800D79F4_jp(void);
void* malloc(size_t size);
u32 GetCurrentMilliseconds(void);
void game_dt(Game* game);

void func_8005578C_jp(GameStateOverlay*); 

void func_800D8644_jp(void); 

s32 func_800D9700_jp(void);

void* func_800D9750_jp(void);

s64* ucode_GetRspBootTextStart(void);
size_t ucode_GetRspBootTextSize(void);
s64* ucode_GetPolyTextStart(void);
s64* ucode_GetPolyDataStart(void);
s64* ucode_GetSpriteTextStart(void);
s64* ucode_GetSpireDataStart(void);

struct Game_Play;

typedef enum AnimationMode {
    /* 0 */ ANIMATION_STOP,
    /* 1 */ ANIMATION_REPEAT
} AnimationMode;

typedef struct {
    /* 0x00 */ f32 start;
    /* 0x04 */ f32 end;
    /* 0x08 */ f32 duration;
    /* 0x0C */ f32 speed;
    /* 0x10 */ f32 currentFrame;
    /* 0x14 */ AnimationMode mode;
} FrameControl; // size = 0x18

typedef struct {
    /* 0x0 */ Gfx* shape;
    /* 0x4 */ u8 numberOfChildren;
    /* 0x5 */ u8 displayBufferFlag;
    /* 0x6 */ s_xyz translation;
} JointElemR; // size = 0xC

typedef struct {
    /* 0x00 */ u8 numberOfJoints;
    /* 0x01 */ u8 unk01;
    /* 0x04 */ JointElemR* jointElemTable;
} BaseSkeletonR; // size = 0x8

// original name unknown
typedef struct {
    /* 0x00 */ s16 frame;
    /* 0x02 */ s16 value;
    /* 0x04 */ s16 velocity;
} Keyframe; // size = 0x06

typedef struct {
    /* 0x00 */ u8* constKeyCheckBitTable;
    /* 0x04 */ Keyframe* dataSource;
    /* 0x08 */ s16* keyframeNumber;
    /* 0x0C */ s16* constValueTable;
    /* 0x10 */ s16 unk10;
    /* 0x12 */ s16 duration;
} BaseAnimationR; // size = 0x14

typedef struct {
    /* 0x00 */ s32 transformationFlag;
    /* 0x04 */ xyz_t baseWorldPosition;
    /* 0x10 */ s16 baseAngleY;
    /* 0x14 */ xyz_t baseShapeTranslation;
    /* 0x20 */ s_xyz baseShapeRotation;
    /* 0x26 */ s_xyz updatedBaseShapeRotation;
    /* 0x2C */ f32 counter;
    /* 0x30 */ xyz_t shapeWorldPositionCorrection;
    /* 0x3C */ s16 shapeAngleCorrection;
} AnimationMove; // size = 0x40

typedef struct {
    /* 0x00 */ FrameControl frameControl;
    /* 0x18 */ BaseSkeletonR* skeleton;
    /* 0x1C */ BaseAnimationR* animation;
    /* 0x20 */ f32 morphCounter;
    /* 0x24 */ s_xyz* jointTable;
    /* 0x28 */ s_xyz* morphTable;
    /* 0x2C */ s_xyz* diffRotTable;
    /* 0x30 */ AnimationMove animationMove;
} SkeletonInfoR; // size = 0x70

typedef struct {
    /* 0x00 */ SkeletonInfoR* skeletonInfo;
    /* 0x04 */ u8* constKeyCheckBitTable;
    /* 0x08 */ s16* constValueTable;
    /* 0x0C */ Keyframe* dataSource;
    /* 0x10 */ s16* keyframeNumber;
    /* 0x14 */ s32 keyframeNumberIndex;
    /* 0x18 */ s32 ckcbIndex;
    /* 0x1C */ s32 keyframeStartIndex;
} SkeletonInfoRCombineWork; // size = 0x20

typedef s32 (*DrawCallback)(struct Game_Play* game_play, SkeletonInfoR* skeletonInfo, s32 jointIndex, Gfx** dlist, u8* displayBufferFlag, void*, s_xyz* rotation, xyz_t* translation);

void cKF_FrameControl_zeroClear(FrameControl* frameControl);
void cKF_FrameControl_ct(FrameControl* frameControl);
void cKF_FrameControl_setFrame(FrameControl* frameControl, f32 start, f32 end, f32 duration, f32 currentFrame,
                               f32 speed, AnimationMode mode);
s32 cKF_FrameControl_passCheck(FrameControl* frameControl, f32 compareFrame, f32* remainder);
s32 cKF_FrameControl_passCheck_now(FrameControl* frameControl, f32 compareFrame);
s32 cKF_FrameControl_stop_proc(FrameControl* frameControl);
s32 cKF_FrameControl_repeat_proc(FrameControl* frameControl);
s32 cKF_FrameControl_play(FrameControl* frameControl);
f32 cKF_HermitCalc(f32 t, f32 duration, f32 p0, f32 p1, f32 v0, f32 v1);
s16 cKF_KeyCalc(s16 startIndex, s16 sequenceLength, Keyframe* dataSource, f32 currentFrame);
void cKF_SkeletonInfo_subRotInterpolation(f32 t, s16* out, s16 jointRotation, s16 morphRotation);
void cKF_SkeletonInfo_morphST(s16* joint, s16* morph, f32 t);
void cKF_SkeletonInfo_R_zeroClear(SkeletonInfoR* skeletonInfo);
void cKF_SkeletonInfo_R_ct(SkeletonInfoR* skeletonInfo, BaseSkeletonR* skeleton, BaseAnimationR* animation,
                           s_xyz* jointTable, s_xyz* morphTable);
void cKF_SkeletonInfo_R_dt(SkeletonInfoR* skeletonInfo);
void cKF_SkeletonInfo_R_init_standard_stop(SkeletonInfoR* skeletonInfo, BaseAnimationR* animation, s_xyz* diffRotTable);
void cKF_SkeletonInfo_R_init_standard_stop_speedset(SkeletonInfoR* skeletonInfo, BaseAnimationR* animation,
                                                    s_xyz* diffRotTable, f32 speed);
void cKF_SkeletonInfo_R_init_standard_stop_morph(SkeletonInfoR* skeletonInfo, BaseAnimationR* animation,
                                                 s_xyz* diffRotTable, f32 morphCounter);
void cKF_SkeletonInfo_R_init_standard_repeat(SkeletonInfoR* skeletonInfo, BaseAnimationR* animation,
                                             s_xyz* diffRotTable);
void cKF_SkeletonInfo_R_init_standard_repeat_speedset(SkeletonInfoR* skeletonInfo, BaseAnimationR* animation,
                                                      s_xyz* diffRotTable, f32 speed);
void cKF_SkeletonInfo_R_init_standard_repeat_morph(SkeletonInfoR* skeletonInfo, BaseAnimationR* animation,
                                                   s_xyz* diffRotTable, f32 morphCounter);
void cKF_SkeletonInfo_R_init(SkeletonInfoR* skeletonInfo, BaseSkeletonR* skeleton, BaseAnimationR* animation,
                             f32 startFrame, f32 endFrame, f32 currentFrame, f32 speed, f32 morphCounter,
                             AnimationMode mode, s_xyz* diffRotTable);
void cKF_SkeletonInfo_R_setAnim(SkeletonInfoR* skeletonInfo, BaseAnimationR* animation);
void cKF_SkeletonInfo_R_morphJoint(SkeletonInfoR* skeletonInfo);
s32 cKF_SkeletonInfo_R_play(SkeletonInfoR* skeletonInfo);
void cKF_Si3_draw_SV_R_child(struct Game_Play* game_play, SkeletonInfoR* skeletonInfo, s32* jointIndex,
                             DrawCallback beforeCallback, DrawCallback afterCallback, void* arg, Mtx** mtx);
void cKF_Si3_draw_R_SV(struct Game_Play* game_play, SkeletonInfoR* skeletonInfo, Mtx* mtx, DrawCallback beforeCallback,
                       DrawCallback afterCallback, void* arg);
void cKF_SkeletonInfo_R_init_standard_repeat_speedsetandmorph(SkeletonInfoR* skeletonInfo, BaseAnimationR* animation,
                                                              s_xyz* diffRotTable, f32 speed, f32 morphCounter);
void cKF_SkeletonInfo_R_init_standard_repeat_setframeandspeedandmorph(SkeletonInfoR* skeletonInfo,
                                                                      BaseAnimationR* animation, s_xyz* diffRotTable,
                                                                      f32 currentFrame, f32 speed, f32 morphCounter);
void cKF_SkeletonInfo_R_init_standard_setframeandspeedandmorphandmode(SkeletonInfoR* skeletonInfo,
                                                                      BaseAnimationR* animation, s_xyz* diffRotTable,
                                                                      f32 currentFrame, f32 speed, f32 morphCounter,
                                                                      AnimationMode mode);
void cKF_SkeletonInfo_R_init_reverse_setspeedandmorphandmode(SkeletonInfoR* skeletonInfo, BaseAnimationR* animation,
                                                             s_xyz* diffRotTable, f32 currentFrame, f32 speed,
                                                             AnimationMode mode);
void func_80053384_jp(s32 arg0, s32 arg1, s32 arg2, s32 arg3, s32 arg4, s32 arg5, UNK_PTR* arg6, UNK_PTR* arg7,
                      UNK_PTR* arg8);
void cKF_SkeletonInfo_R_combine_work_set(SkeletonInfoRCombineWork* combineWork, SkeletonInfoR* skeletonInfo);
void cKF_SkeletonInfo_R_combine_translation(s16** joint, u32* flag, SkeletonInfoRCombineWork* combineWork, s8* arg3);
void cKF_SkeletonInfo_R_combine_rotation(s16** joint, u32* flag, SkeletonInfoRCombineWork* combineWork, s8* arg3);
s32 cKF_SkeletonInfo_R_combine_play(SkeletonInfoR* skeletonInfo1, SkeletonInfoR* skeletonInfo2, s32 arg2, s32 arg3,
                                    s32 arg4, s32 arg5, s8* flag);
void cKF_SkeletonInfo_R_T_combine_play(s32* arg0, s32* arg1, s32* arg2, SkeletonInfoR* skeletonInfo1,
                                       SkeletonInfoR* skeletonInfo2, SkeletonInfoR* skeletonInfo3, s32 arg6, s32 arg7,
                                       s32 arg8, s32 arg9, s32 argA, s32 argB, s8* flag);
void cKF_SkeletonInfo_R_Animation_Set_base_shape_trs(SkeletonInfoR* skeletonInfo, f32 translationX, f32 translationY,
                                                     f32 translationZ, s16 rotX, s16 rotY, s16 rotZ);
void cKF_SkeletonInfo_R_AnimationMove_ct_base(xyz_t* arg0, xyz_t* arg1, s16 arg2, s16 arg3, f32 arg4,
                                              SkeletonInfoR* skeletonInfo, s32 transformationFlag);
void cKF_SkeletonInfo_R_AnimationMove_dt(SkeletonInfoR* skeletonInfo);
void cKF_SkeletonInfo_R_AnimationMove_base(xyz_t* arg0, s_xyz* arg1, xyz_t* arg2, s16 arg3,
                                           SkeletonInfoR* skeletonInfo);
void cKF_SkeletonInfo_R_AnimationMove_CulcTransToWorld(xyz_t* arg0, xyz_t* arg1, f32 arg2, f32 arg3, f32 arg4, s16 arg5,
                                                       xyz_t* arg6, SkeletonInfoR* skeleton, s32 arg8);


s32 mFI_Wpos2BlockNum(s32*, s32*, xyz_t);

void _texture_z_light_fog_prim_npc(GraphicsContext*);

typedef struct Train0 {
    Actor actor;
    s32 steamAvailable;
    SkeletonInfoR keyframe;
    s32 steamTimer;
    s_xyz work[15];
    s_xyz target[15];
    u8 pad4[0x2b4 - 0x2A0];
    s32 trainAction;
    u8 pad5[0x2BC - 0x2B8];
    s32 effectNum;
    s32 unk2c0;
    Actor* trainActorP;
    f32 train1Pos;
    f32 trainSpeed;
    u8 pad6[0x2D8 - 0x2CC];
} Train0;

extern BaseSkeletonR cKF_bs_r_obj_train1_1;
extern BaseAnimationR cKF_ba_r_obj_train1_1; 

extern Gfx obj_train1_2_model[];


void func_805c04e0(Actor* actor, Game_Play* play){
    void* train = common_data.unk_10098->unk_AC(16);
    u16* pal = common_data.unk_10098->unk_450(0x39);
    s32 pad; 
    GraphicsContext* graph = play->state.gfxCtx;
    s32 alloc;
    Mtx* mtx;  
    Train0* train0 = (Train0*)actor;
    Gfx* gfx;

 
    if(1);
    
    mtx = GRAPH_ALLOC_NO_ALIGN(graph, sizeof(Mtx) * train0->keyframe.skeleton->unk01);

    if (mtx != NULL){
        _texture_z_light_fog_prim_npc(graph);
        Matrix_push();
        Matrix_translate(train0->train1Pos, train0->actor.world.pos.y, 
            train0->actor.world.pos.z, 0);
        Matrix_scale(train0->actor.scale.x, train0->actor.scale.y, 
            train0->actor.scale.z, 1);

        OPEN_DISPS(graph);
        gfx = POLY_OPA_DISP;
        
        gSPMatrix(gfx++, _Matrix_to_Mtx_new(graph), G_MTX_NOPUSH | G_MTX_LOAD | G_MTX_MODELVIEW);
        gSegments[6] = (uintptr_t)OS_K0_TO_PHYSICAL(train);
        
        gSPSegment(gfx++, 6, train); 
        gSPSegment(gfx++, 8, pal); 
        
        gDPLoadTLUT_pal16(gfx++, 15, pal);
        gSPDisplayList(gfx++, obj_train1_2_model);
        
        POLY_OPA_DISP = gfx;
        CLOSE_DISPS(graph);
        
        Matrix_pull();
        cKF_Si3_draw_R_SV(play, &train0->keyframe, mtx, NULL,NULL,train0);
    }
}
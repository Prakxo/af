#ifndef M_MSG_MAIN_H
#define M_MSG_MAIN_H

#include "ultra64.h"
#include "unk.h"
#include "m_choice_main.h"

struct Game_Play;
struct Actor;
struct Color_RGBA8;

typedef struct MessageWindow {
    /* 0x000 */ UNK_TYPE1 unk_000[0x1B0];
    /* 0x1B0 */ Choice choiceWindow;
} MessageWindow; // size >= 0x26C

MessageWindow* mMsg_Get_base_window_p(void);
// void func_8009D200_jp();
// void func_8009D210_jp();
s32 mMsg_Check_main_index(MessageWindow*, s32);
// void func_8009D274_jp();
UNK_RET mMsg_Check_not_series_main_wait(UNK_PTR arg0);
UNK_RET mMsg_Check_main_hide(UNK_PTR arg0);
// void func_8009D308_jp();
// void func_8009D3B4_jp();
s32 mMsg_request_main_appear(MessageWindow*, struct Actor*, s32 , struct Color_RGBA8*,
                                    s32, s32);
// void func_8009D458_jp();
// void func_8009D4A0_jp();
// void func_8009D4F0_jp();
// void func_8009D510_jp();
// void func_8009D530_jp();
// void func_8009D578_jp();
// void func_8009D5CC_jp();
// void func_8009D620_jp();
// void func_8009D640_jp();
// void func_8009D688_jp();
void mMsg_Set_free_str(UNK_PTR arg0, s32 arg1, UNK_PTR arg2, s32 arg3); 
// void func_8009D820_jp();
void mMsg_Set_item_str(UNK_PTR arg0, s32 arg1, char* str, s32 size);
// void func_8009D9A4_jp();
// void func_8009DA1C_jp();
void mMsg_Set_mail_str(UNK_PTR arg0, s32 arg1, char* str, s32 size);
// void func_8009DBA4_jp();
// void func_8009DBB0_jp();
// void func_8009DBE0_jp();
// void func_8009DC04_jp();
// void func_8009DC2C_jp();
// void func_8009DCD0_jp();
// void func_8009DD04_jp();
// void func_8009DD2C_jp();
// void func_8009DD64_jp();
// void func_8009DD8C_jp();
// void func_8009DDB8_jp();
// void func_8009DDE4_jp();
// void func_8009DE28_jp();
// void func_8009DE6C_jp();
// void func_8009DE94_jp();
// void func_8009DEE8_jp();
// void func_8009DF1C_jp();
// void func_8009DF90_jp();
// void func_8009DFBC_jp();
// void func_8009DFF4_jp();
// void func_8009E01C_jp();
// void func_8009E084_jp();
// void func_8009E0B0_jp();
// void func_8009E118_jp();
// void func_8009E144_jp();
// void func_8009E17C_jp();
// void func_8009E1A4_jp();
// void func_8009E1D0_jp();
// void func_8009E1F8_jp();
// void func_8009E218_jp();
// void func_8009E264_jp();
// void func_8009E28C_jp();
// void func_8009E2D0_jp();
// void func_8009E338_jp();
// void func_8009E344_jp();
// void func_8009E374_jp();
// void func_8009E388_jp();
// void func_8009E4C8_jp();
// void func_8009E558_jp();
// void func_8009E658_jp();
// void func_8009E6B8_jp();
// void func_8009E6C4_jp();
// void func_8009E6D4_jp();
// void func_8009E6F8_jp();
size_t mMsg_Get_Length_String(char*, s32);
// void func_8009E908_jp();
// void func_8009E94C_jp();
// void func_8009E970_jp();
// void func_8009E990_jp();
// void func_8009E9A4_jp();
// void func_8009E9B4_jp();
// void func_8009E9C0_jp();
// void func_8009E9D0_jp();
// void func_8009E9DC_jp();
// void func_8009E9E8_jp();
// void func_8009E9F8_jp();
// void func_8009EA04_jp();
// void func_8009EA18_jp();
// void func_8009EA2C_jp();
// void func_8009EB44_jp();
// void func_8009EBB0_jp();
s32 mMsg_CopyPlayerName(char*, s32, s32);
s32 mMsg_CopyTalkName(struct Actor*, char*, s32, s32);
s32 mMsg_CopyTail(struct Actor*, char*, s32, s32);
s32 mMsg_CopyYear(char*, s32, s32);
s32 mMsg_CopyMonth(char*, s32, s32);
s32 mMsg_CopyWeek(char*, s32, s32);
s32 mMsg_CopyDay(char*, s32, s32);
s32 mMsg_CopyHour(char*, s32, s32);
s32 mMsg_CopyMin(char*, s32, s32);
s32 mMsg_CopySec(char*, s32, s32);
s32 mMsg_CopyFree(MessageWindow*, s32, char*, s32, s32);
// void func_8009F2EC_jp();
s32 mMsg_CopyDetermination(MessageWindow*, char*, s32, s32);
s32 mMsg_CopyCountryName(char*,s32,s32);
s32 mMsg_CopyRamdomNumber2(char*,s32,s32);
s32 mMsg_CopyItem(MessageWindow*, s32, char*, s32, s32);
// void func_8009F670_jp();
// void func_8009F730_jp();
// void func_8009F75C_jp();
// void func_8009F790_jp();
// void func_8009F7CC_jp();
// void func_8009F800_jp();
// void func_8009F830_jp();
// void func_8009F864_jp();
// void func_8009F8AC_jp();
// void func_8009F9D8_jp();
// void func_8009F9F8_jp();
// void func_8009FA18_jp();
// void func_8009FA38_jp();
// void func_8009FA58_jp();
// void func_8009FAD4_jp();
// void func_8009FB2C_jp();
// void func_8009FB54_jp();
// void func_8009FBD8_jp();
// void func_8009FC2C_jp();
// void func_8009FC5C_jp();
// void func_8009FCB8_jp();
// void func_8009FCE0_jp();
void mMsg_sound_spec_change_voice(UNK_PTR arg0);
// void func_8009FD80_jp();
// void func_8009FDA0_jp();
// void func_8009FDF8_jp();
// void func_8009FE4C_jp();
// void func_8009FE6C_jp();
// void func_8009FE90_jp();
// void func_8009FF24_jp();
// void func_8009FF68_jp();
// void func_8009FFB0_jp();
// void func_800A0110_jp();
// void func_800A014C_jp();
// void func_800A0184_jp();
// void func_800A01C8_jp();
// void func_800A03B0_jp();
// void func_800A0478_jp();
// void func_800A04E4_jp();
// void func_800A054C_jp();
// void func_800A05A8_jp();
// void func_800A05D8_jp();
// void func_800A0614_jp();
// void func_800A0650_jp();
// void func_800A06AC_jp();
// void func_800A0770_jp();
// void func_800A07E8_jp();
// void func_800A0864_jp();
// void func_800A08B0_jp();
// void func_800A08F8_jp();
// void func_800A0964_jp();
// void func_800A0984_jp();
// void func_800A09A4_jp();
// void func_800A09C4_jp();
// void func_800A09E4_jp();
// void func_800A0A04_jp();
// void func_800A0A60_jp();
// void func_800A0B14_jp();
// void func_800A0B34_jp();
// void func_800A0B54_jp();
// void func_800A0B74_jp();
// void func_800A0B94_jp();
// void func_800A0BB4_jp();
// void func_800A0D94_jp();
// void func_800A0DB4_jp();
// void func_800A0DD4_jp();
// void func_800A0DF4_jp();
// void func_800A0FCC_jp();
// void func_800A0FEC_jp();
// void func_800A100C_jp();
// void func_800A102C_jp();
// void func_800A1078_jp();
// void func_800A10D8_jp();
// void func_800A1124_jp();
// void func_800A1170_jp();
// void func_800A11B4_jp();
// void func_800A11F8_jp();
// void func_800A123C_jp();
// void func_800A1280_jp();
// void func_800A12C4_jp();
// void func_800A1308_jp();
// void func_800A134C_jp();
// void func_800A1394_jp();
// void func_800A141C_jp();
// void func_800A1444_jp();
// void func_800A1468_jp();
// void func_800A148C_jp();
// void func_800A14B4_jp();
// void func_800A14DC_jp();
// void func_800A1500_jp();
// void func_800A1528_jp();
// void func_800A1550_jp();
// void func_800A1578_jp();
// void func_800A15A0_jp();
// void func_800A15C8_jp();
// void func_800A15F0_jp();
// void func_800A1618_jp();
// void func_800A1640_jp();
// void func_800A1668_jp();
// void func_800A1690_jp();
// void func_800A16B8_jp();
// void func_800A16E0_jp();
// void func_800A1708_jp();
// void func_800A1730_jp();
// void func_800A1774_jp();
// void func_800A17B8_jp();
// void func_800A17FC_jp();
// void func_800A1844_jp();
// void func_800A186C_jp();
// void func_800A1894_jp();
// void func_800A18BC_jp();
// void func_800A18E4_jp();
// void func_800A190C_jp();
// void func_800A1954_jp();
// void func_800A197C_jp();
// void func_800A19C4_jp();
// void func_800A19E4_jp();
// void func_800A1A04_jp();
// void func_800A1A24_jp();
// void func_800A1A44_jp();
// void func_800A1A64_jp();
// void func_800A1A84_jp();
// void func_800A1AA4_jp();
// void func_800A1AC4_jp();
// void func_800A1AE4_jp();
// void func_800A1B04_jp();
// void func_800A1B4C_jp();
// void func_800A1B6C_jp();
// void func_800A1B8C_jp();
// void func_800A1BAC_jp();
// void func_800A1BCC_jp();
// void func_800A1BEC_jp();
// void func_800A1C24_jp();
// void func_800A1CA4_jp();
// void func_800A1CDC_jp();
// void func_800A1D14_jp();
// void func_800A1D4C_jp();
// void func_800A1DD0_jp();
// void func_800A1E38_jp();
// void func_800A1EA0_jp();
// void func_800A1F1C_jp();
// void func_800A1F7C_jp();
// void func_800A1FB4_jp();
// void func_800A2000_jp();
// void func_800A204C_jp();
// void func_800A2098_jp();
// void func_800A20E0_jp();
// void func_800A2150_jp();
// void func_800A21C0_jp();
// void func_800A223C_jp();
// void func_800A24A8_jp();
// void func_800A24D0_jp();
// void func_800A2538_jp();
// void func_800A2588_jp();
// void func_800A2620_jp();
// void func_800A2648_jp();
// void func_800A268C_jp();
// void func_800A272C_jp();
// void func_800A2784_jp();
// void func_800A27FC_jp();
// void func_800A287C_jp();
// void func_800A289C_jp();
// void func_800A28DC_jp();
// void func_800A2904_jp();
// void func_800A2948_jp();
// void func_800A29B0_jp();
// void func_800A2AB0_jp();
// void func_800A2AD0_jp();
// void func_800A2BB0_jp();
// void func_800A2C4C_jp();
// void func_800A2CE4_jp();
// void func_800A2EAC_jp();
// void func_800A3190_jp();
// void func_800A31D8_jp();
// void func_800A3220_jp();
void mMsg_ct(struct Game_Play* game_play);
void func_800A3304_jp(struct Game_Play* game_play);
// void func_800A332C_jp();
void mMsg_Main(struct Game_Play* game_play);
void mMsg_Draw(struct Game_Play* game_play);
// void func_800A4448_jp();
// void func_800A44D4_jp();
// void func_800A4508_jp();
// void func_800A456C_jp();
// void func_800A4614_jp();
// void func_800A46F0_jp();
// void func_800A49E8_jp();
// void func_800A4A84_jp();
// void func_800A4B00_jp();
void func_800A4D10_jp(void);
// void func_800A4DC8_jp();
// void func_800A4E74_jp();
// void func_800A4F38_jp();
// void func_800A4FD0_jp();
// void func_800A50AC_jp();
// void func_800A518C_jp();
// void func_800A524C_jp();
// void func_800A5460_jp();

#endif

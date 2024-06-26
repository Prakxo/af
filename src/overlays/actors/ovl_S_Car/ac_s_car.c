#include "ac_s_car.h"
#include "m_actor_dlftbls.h"
#include "m_object.h"
#include "overlays/gamestates/ovl_play/m_play.h"

void aSCR_actor_ct(Actor* thisx, Game_Play* game_play);
void aSCR_actor_dt(Actor* thisx, Game_Play* game_play);
void aSCR_actor_init(Actor* thisx, Game_Play* game_play);
void aSCR_actor_draw(Actor* thisx, Game_Play* game_play);

#if 0
ActorProfile S_Car_Profile = {
    /* */ ACTOR_S_CAR,
    /* */ ACTOR_PART_0,
    /* */ 0,
    /* */ 0x5828,
    /* */ GAMEPLAY_KEEP,
    /* */ sizeof(S_Car),
    /* */ aSCR_actor_ct,
    /* */ aSCR_actor_dt,
    /* */ aSCR_actor_init,
    /* */ aSCR_actor_draw,
    /* */ NULL,
};
#endif

#pragma GLOBAL_ASM("asm/jp/nonmatchings/overlays/actors/ovl_S_Car/ac_s_car/aSCR_actor_ct.s")

#pragma GLOBAL_ASM("asm/jp/nonmatchings/overlays/actors/ovl_S_Car/ac_s_car/aSCR_actor_dt.s")

#pragma GLOBAL_ASM("asm/jp/nonmatchings/overlays/actors/ovl_S_Car/ac_s_car/func_80A09928_jp.s")

#pragma GLOBAL_ASM("asm/jp/nonmatchings/overlays/actors/ovl_S_Car/ac_s_car/func_80A09A88_jp.s")

#pragma GLOBAL_ASM("asm/jp/nonmatchings/overlays/actors/ovl_S_Car/ac_s_car/aSCR_actor_init.s")

#pragma GLOBAL_ASM("asm/jp/nonmatchings/overlays/actors/ovl_S_Car/ac_s_car/func_80A09C9C_jp.s")

#pragma GLOBAL_ASM("asm/jp/nonmatchings/overlays/actors/ovl_S_Car/ac_s_car/aSCR_actor_draw.s")

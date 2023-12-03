#ifndef AC_TRAIN_DOOR_H
#define AC_TRAIN_DOOR_H

#include "ultra64.h"
#include "m_actor.h"
#include "c_keyframe.h"
#include "unk.h"


typedef struct TrainDoor {
    Actor actor;
    u8 unk_174 [0x17C - 0x174];
    SkeletonInfoR keyframe;
    u8 unk_1ED [0x210 - 0x1ed];
    s_xyz work[4];
    s_xyz target[4];
    s32 idle;
} TrainDoor;

#endif

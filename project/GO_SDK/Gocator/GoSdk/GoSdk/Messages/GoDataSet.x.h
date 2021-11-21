/** 
 * @file    GoDataSet.x.h
 *
 * @internal
 * Copyright (C) 2016-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef GO_SDK_DATA_SET_X_H
#define GO_SDK_DATA_SET_X_H

#include <kApi/Data/kArrayList.h>
#include <kApi/Io/kSerializer.h>

typedef struct GoDataSetClass
{
    kObjectClass base; 
    k32u senderId; 
    kArrayList content; 
} GoDataSetClass; 

kDeclareClassEx(Go, GoDataSet, kObject)

GoFx(kStatus) GoDataSet_Construct(GoDataSet* set, kAlloc allocator);
GoFx(kStatus) GoDataSet_Init(GoDataSet set, kType type, kAlloc alloc);
GoFx(kStatus) GoDataSet_VInitClone(GoDataSet set, GoDataSet source, kAlloc alloc); 
GoFx(kStatus) GoDataSet_VRelease(GoDataSet set);
GoFx(kStatus) GoDataSet_Allocate(GoDataSet set, kSize minimumCapacity);
GoFx(kStatus) GoDataSet_Add(GoDataSet set, kObject item);
GoFx(kSize) GoDataSet_VSize(GoDataSet set); 
GoFx(kStatus) GoDataSet_WriteDat6V0(GoDataSet set, kSerializer serializer);
GoFx(kStatus) GoDataSet_ReadDat6V0(GoDataSet set, kSerializer serializer, kAlloc alloc);
GoFx(kStatus) GoDataSet_ReadDat6V0Header(GoDataSet set, kSerializer serializer, k32u* senderId);

#define GoDataSet_SetSenderId_(D, V)         (xGoDataSet_CastRaw(D)->senderId = (V), kOK)
#define GoDataSet_SetContent_(D, V)          (xGoDataSet_CastRaw(D)->content = (V), kOK)
#define GoDataSet_Content_(D)                (xGoDataSet_CastRaw(D)->content)

#endif

/** 
 * @file    GoMultiplexBank.x.h
 *
 * @internal
 * Copyright (C) 2016-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
 
#ifndef GO_MULTIPLEXBANK_X_H
#define GO_MULTIPLEXBANK_X_H

#include <kApi/Data/kXml.h>

typedef struct GoMultiplexBankClass
{
    kObjectClass base;
    kObject sensor;

    kXml xml;
    kXmlItem xmlItem;

    k32u id;
    kArrayList sensorList;  //of k32u ids
} GoMultiplexBankClass;

kDeclareClassEx(Go, GoMultiplexBank, kObject)

GoFx(kStatus) GoMultiplexBank_Construct(GoMultiplexBank* bank, k32u id, kAlloc alloc);
GoFx(kStatus) GoMultiplexBank_Init(GoMultiplexBank bank, kType type, k32u id, kAlloc alloc);
GoFx(kStatus) GoMultiplexBank_VRelease(GoMultiplexBank bank);

#endif

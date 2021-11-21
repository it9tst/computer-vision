/** 
 * @file    GoTracheid.x.h
 *
 * @internal
 * Copyright (C) 2016-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef GO_TRACHEID_X_H
#define GO_TRACHEID_X_H

#include <kApi/Data/kXml.h>

typedef struct GoTracheidClass
{
    kObjectClass base;
    kObject sensor;

    kXml xml;
    kXmlItem xmlItem;

    kBool used;

    GoElement64f exposure;

    k32s camera0Threshold;
    k32s camera1Threshold;

} GoTracheidClass;

kDeclareClassEx(Go, GoTracheid, kObject)

GoFx(kStatus) GoTracheid_Construct(GoTracheid* transform, kObject sensor, kAlloc allocator);

GoFx(kStatus) GoTracheid_Init(GoTracheid transform, kType type, kObject sensor, kAlloc alloc);
GoFx(kStatus) GoTracheid_VRelease(GoTracheid transform);

GoFx(kStatus) GoTracheid_Read(GoTracheid transform, kXml xml, kXmlItem item);
GoFx(kStatus) GoTracheid_Write(GoTracheid transform, kXml xml, kXmlItem item); 

#endif

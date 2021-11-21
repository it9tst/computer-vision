/** 
 * @file    GoExtParams.x.h
 *
 * @internal
 * Copyright (C) 2017-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef GO_EXT_PARAMS_X_H
#define GO_EXT_PARAMS_X_H

#include <kApi/Data/kXml.h>

kDeclareClassEx(Go, GoExtParams, kObject)

#define GoExtParams_Class_(PARAM)                     (kCastClass_(GoExtParams, PARAM))

typedef struct GoExtParamsClass
{
    kObjectClass base;
    kArrayList params;
    kObject sensor;
    kXml xml;
    kArrayList nodesToMerge;
} GoExtParamsClass;

GoFx(kStatus) GoExtParams_Construct(GoExtParams* params, kObject sensor, kAlloc allocator);
GoFx(kStatus) GoExtParams_Init(GoExtParams param, kType type, kObject sensor, kAlloc alloc);
GoFx(kStatus) GoExtParams_VRelease(GoExtParams value);

GoFx(kStatus) GoExtParams_Read(GoExtParams param, kXml xml, kXmlItem item);
GoFx(kStatus) GoExtParams_Write(GoExtParams param, kXml xml, kXmlItem item); 

#endif

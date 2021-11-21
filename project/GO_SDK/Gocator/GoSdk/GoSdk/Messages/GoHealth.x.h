/** 
 * @file    GoHealthMsg.x.h
 *
 * @internal
 * Copyright (C) 2016-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef GO_SDK_HEALTH_X_H
#define GO_SDK_HEALTH_X_H

#include <kApi/Data/kArray1.h>

kDeclareEnumEx(Go, GoHealthIndicatorId, kValue)
kDeclareValueEx(Go, GoIndicator, kValue)

typedef struct GoHealthMsgClass
{
    kObjectClass base; 
    GoDataSource source; 
    kArray1 indicators; 
} GoHealthMsgClass; 

kDeclareClassEx(Go, GoHealthMsg, kObject)

GoFx(kStatus) GoHealthMsg_Construct(GoHealthMsg* msg, kAlloc allocator);
GoFx(kStatus) GoHealthMsg_Init(GoHealthMsg msg, kType type, kAlloc alloc);
GoFx(kStatus) GoHealthMsg_VInitClone(GoHealthMsg msg, GoHealthMsg source, kAlloc alloc); 
GoFx(kStatus) GoHealthMsg_Allocate(GoHealthMsg msg, kSize count);
GoFx(kStatus) GoHealthMsg_VRelease(GoHealthMsg msg);
GoFx(kSize) GoHealthMsg_VSize(GoHealthMsg msg); 
GoFx(kStatus) GoHealthMsg_WriteV0(GoHealthMsg msg, kSerializer serializer);
GoFx(kStatus) GoHealthMsg_ReadV0(GoHealthMsg msg, kSerializer serializer, kAlloc alloc);

#define GoHealthMsg_SetContent_(D, V)            (xGoHealthMsg_CastRaw(D)->indicators = (V), kOK)
#define GoHealthMsg_Content_(D)                  (xGoHealthMsg_CastRaw(D)->indicators)
#define GoHealthMsg_SetSource_(D, V)             (xGoHealthMsg_CastRaw(D)->source = (V), kOK)

#define GO_HEALTH_ANALOG_DROPS_V1                       (2501)      ///< Deprecated: Current number of analog output drops.
#define GO_HEALTH_DIGITAL_DROPS_V1                      (2601)      ///< Deprecated: Current number of digital output drops.
#define GO_HEALTH_SERIAL_DROPS_V1                       (2701)      ///< Deprecated: Current number of serial output drops.

#endif

/** 
 * @file    kBackTrace.x.h
 *
 * @internal
 * Copyright (C) 2014-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_BACK_TRACE_X_H
#define K_API_BACK_TRACE_X_H

#include <kApi/Data/kList.h>

#define xkBACK_TRACE_CAPACITY                    (256)               ///< Maximum supported call stack depth. 

typedef struct kBackTraceClass
{
    kObjectClass base;
    kPointer functions[xkBACK_TRACE_CAPACITY];           //Function call pointers. 
    kSize depth;                                        //Total call stack depth. 
    kSize skip;                                         //Number of frames to omit in reported depth. 
} kBackTraceClass; 

kDeclareClassEx(k, kBackTrace, kObject)

/* 
* Private methods. 
*/

kFx(kStatus) xkBackTrace_Init(kBackTrace trace, kType type, kAlloc alloc); 
kFx(kStatus) xkBackTrace_VClone(kBackTrace trace, kObject source, kAlloc valueAlloc, kObject context); 
kFx(kStatus) xkBackTrace_VRelease(kBackTrace trace); 

kFx(kBool) xkBackTrace_VEquals(kBackTrace trace, kObject other); 
kFx(kSize) xkBackTrace_VHashCode(kBackTrace trace); 

#endif

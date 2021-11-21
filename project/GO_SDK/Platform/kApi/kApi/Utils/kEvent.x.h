/** 
 * @file    kEvent.x.h
 *
 * @internal
 * Copyright (C) 2012-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_EVENT_X_H
#define K_API_EVENT_X_H

#include <kApi/Data/kList.h>

typedef struct kEventClass
{
    kObjectClass base;
    kList listeners;                //List of event listeners -- kList<kCallback>.
    kListItem notifyIt;             //List iterator used during notification.
} kEventClass; 

kDeclareClassEx(k, kEvent, kObject)

/* 
* Private methods. 
*/

kFx(kStatus) xkEvent_Init(kEvent evnt, kType type, kAlloc allocator); 

kFx(kStatus) xkEvent_VClone(kEvent evnt, kEvent source, kAlloc valueAlloc, kObject context); 

kFx(kStatus) xkEvent_VRelease(kEvent evnt); 

#endif

/** 
 * @file    kPeriodic.x.h
 *
 * @internal
 * Copyright (C) 2010-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_PERIODIC_X_H
#define K_API_PERIODIC_X_H

#include <kApi/Threads/kThread.h>

typedef struct kPeriodicClass
{
    kObjectClass base; 
    kLock lock; 
    kThread thread; 
    k64u startTime; 
    kSemaphore semaphore; 
    kAtomic32s quit; 
    kBool enabled; 
    k64u period; 
    k64u counter; 
    kBool inProgress; 
    kPeriodicElapsedFx onElapsed; 
    kPointer onElapsedContext;
} kPeriodicClass;

kDeclareClassEx(k, kPeriodic, kObject)

/* 
* Private methods.
*/

kFx(kStatus) xkPeriodic_Construct(kPeriodic* timer, const kChar* name, kThreadPriorityClass priorityClass, k32s priorityOffset, kAlloc allocator);

kFx(kStatus) xkPeriodic_Init(kPeriodic timer, kType type, const kChar* name, kThreadPriorityClass priorityClass, k32s priorityOffset, kAlloc allocator);
kFx(kStatus) xkPeriodic_VRelease(kPeriodic timer); 

kFx(kStatus) xkPeriodic_ThreadEntry(kPeriodic timer); 

kFx(kPeriodicElapsedFx) xkPeriodic_Handler(kPeriodic timer);
kFx(kPointer) xkPeriodic_HandlerContext(kPeriodic timer);


/* 
* Deprecated (Stage 1): not recommended for further use, but not yet announced via kDeprecate
*/

//[Deprecated] Update to use a support constructor overload.
kInlineFx(kStatus) kPeriodic_Construct(kPeriodic* timer, kAlloc allocator)
{
    return xkPeriodic_Construct(timer, "kPeriodic.Legacy", kTHREAD_PRIORITY_CLASS_NORMAL, 0, allocator);
}

//[Deprecated] Update to use a support constructor overload.
kInlineFx(kStatus) kPeriodic_ConstructEx(kPeriodic* timer, kSize stackSize, const kChar* name, k32s priority, kAlloc allocator)
{
    kThreadPriorityClass priorityClass = kTHREAD_PRIORITY_CLASS_NORMAL;
    k32s priorityOffset = 0;

    xkThread_ConvertLegacyPriorty(priority, &priorityClass, &priorityOffset);

    return xkPeriodic_Construct(timer, name, priorityClass, priorityOffset, allocator);
}

#endif

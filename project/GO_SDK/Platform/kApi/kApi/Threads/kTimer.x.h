/** 
 * @file    kTimer.x.h
 *
 * @internal
 * Copyright (C) 2005-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_TIMER_X_H
#define K_API_TIMER_X_H

#define xkTIMER_MIN_YEARS                (3)                       ///< minimum years that should be safely representable
#define xkTIMER_SECONDS_PER_YEAR         (365*24*60*60)            ///< approximate seconds per year

typedef struct kTimerStatic
{
    k32u placeholder;       //unused
} kTimerStatic; 

typedef struct kTimerVTable
{
    kObjectVTable base; 
} kTimerVTable; 

typedef struct kTimerClass
{
    kObjectClass base; 
    kBool isStopped;            //< is the timer currently running?
    k64u startTime;             //< time when the timer is started 
    k64u expiryTime;            //< time when the timer expires 
    k64u stopTime;              //< time when the timer is stopped 
} kTimerClass;

kDeclareFullClassEx(k, kTimer, kObject)

/* 
* Private methods. 
*/

kFx(kStatus) xkTimer_InitStatic(); 
kFx(kStatus) xkTimer_ReleaseStatic(); 

kFx(kStatus) xkTimer_Init(kTimer timer, kType type, kAlloc allocator); 

kFx(kStatus) xkTimer_ConfigureDefaults(); 
kFx(k64u) xkTimer_DefaultTickQuery(); 

kInlineFx(k64u) xkTimer_Ticks() 
{ 
    return kApiLib_TimerQueryHandler()(); 
}

kInlineFx(k64u) xkTimer_FromTicks(k64u ticks) 
{ 
    return ticks * kApiLib_TimerMultiplier() / kApiLib_TimerDivider(); 
}

#endif

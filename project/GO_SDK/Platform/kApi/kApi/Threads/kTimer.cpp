/** 
 * @file    kTimer.cpp
 *
 * @internal
 * Copyright (C) 2005-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#define K_PLATFORM
#include <kApi/Threads/kTimer.h>
#include <time.h>

kBeginFullClassEx(k, kTimer)
kEndFullClassEx()

kFx(kStatus) xkTimer_InitStatic()
{
    //if a custom timer handler has not been installed, set defaults
    if (!kApiLib_HasTimerQueryHandler())
    {
        kCheck(xkTimer_ConfigureDefaults()); 
    }

    return kOK; 
}

kFx(kStatus) xkTimer_ReleaseStatic()
{
    return kOK; 
}

kFx(kStatus) kTimer_Construct(kTimer *timer, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback(allocator);
    kType type = kTypeOf(kTimer); 
    kStatus status; 

    kCheck(kAlloc_GetObject(alloc, type, timer)); 

    if (!kSuccess(status = xkTimer_Init(*timer, type, alloc)))
    {
        kAlloc_FreeRef(alloc, timer); 
    }

    return status; 
} 

kFx(kStatus) xkTimer_Init(kTimer timer, kType type, kAlloc allocator)
{
    kObjR(kTimer, timer); 

    kCheck(kObject_Init(timer, type, allocator)); 

    obj->isStopped = kTRUE; 
    obj->startTime = 0; 
    obj->expiryTime = 0; 
    obj->stopTime = 0; 
    
    return kOK; 
}

kFx(kStatus) kTimer_Start(kTimer timer, k64u totalTime)
{
    kObj(kTimer, timer); 

    obj->startTime = kTimer_Now(); 

    if (totalTime == kINFINITE)
    {
        obj->expiryTime = kINFINITE; 
    }
    else
    {
        obj->expiryTime = obj->startTime + totalTime; 
    }

    obj->isStopped = kFALSE; 

    return kOK; 
}

kFx(kStatus) kTimer_Stop(kTimer timer)
{
    kObj(kTimer, timer); 

    if (!obj->isStopped)
    {
        obj->stopTime = kTimer_Now(); 
        obj->isStopped = kTRUE; 
    }

    return kOK; 
}

kFx(kBool) kTimer_IsStarted(kTimer timer)
{
    kObj(kTimer, timer); 

    return !obj->isStopped; 
}

kFx(kBool) kTimer_IsExpired(kTimer timer)
{
    kObj(kTimer, timer); 

    if (obj->expiryTime == kINFINITE)
    {
        return kFALSE; 
    }
    else
    {
        k64u count = (!obj->isStopped) ? kTimer_Now() : obj->stopTime;   
        return count > obj->expiryTime; 
    }
}

kFx(k64u) kTimer_Elapsed(kTimer timer)
{
    kObj(kTimer, timer); 
    k64u count = (!obj->isStopped) ? kTimer_Now() : obj->stopTime; 

    return count - obj->startTime; 
}

kFx(k64u) kTimer_Remaining(kTimer timer)
{
    kObj(kTimer, timer); 

    if (obj->expiryTime == kINFINITE)
    {
        return kINFINITE; 
    }
    else
    {
        k64u count = (!obj->isStopped) ? kTimer_Now() : obj->stopTime; 
        return (count >= obj->expiryTime) ? 0 : (obj->expiryTime - count);
    }
}

#if defined(K_WINDOWS)

/**
 * The Windows performance counter can have a very high frequency, 
 * which requires scaling our timer multiplier/divider to avoid overflow. 
 */
kFx(kStatus) xkTimer_ConfigureDefaults()
{
    k64u multiplier = 1000000; 
    k64u divider = 0; 
    k64f maxTicks = 0; 
    k64f maxProduct = 0; 

    if (!QueryPerformanceFrequency((LARGE_INTEGER*)&divider))
    {
        return kERROR_OS;  
    }
    
    maxTicks = xkTIMER_MIN_YEARS * xkTIMER_SECONDS_PER_YEAR * (k64f)divider; 

    kCheckState(maxTicks < (k64f)(k64S_MAX)); 

    maxProduct = maxTicks * multiplier; 

    while (maxProduct > (k64f)(k64S_MAX))
    {
        divider /= 10; 
        multiplier /= 10; 

        maxProduct = maxTicks * multiplier; 
    } 

    kCheckState((multiplier > 0) && (divider > 0)); 

    kCheck(kApiLib_SetTimerQueryHandler(xkTimer_DefaultTickQuery)); 
    kCheck(kApiLib_SetTimerScale(multiplier, divider)); 

    return kOK; 
}

kFx(k64u) xkTimer_DefaultTickQuery()
{
    k64u count; 

    if (!QueryPerformanceCounter((LARGE_INTEGER*)&count))
    {
        return 0; 
    }
    
    return count; 
}

#elif defined(K_DARWIN)

kFx(kStatus) xkTimer_ConfigureDefaults()
{
    kCheck(kApiLib_SetTimerQueryHandler(xkTimer_DefaultTickQuery)); 
    kCheck(kApiLib_SetTimerScale(1, 1)); 

    return kOK;
}

kFx(k64u) xkTimer_DefaultTickQuery()
{
    struct timeval tv;
    
    if (gettimeofday(&tv, kNULL) != 0)
    {
        return 0; 
    }

    return k64U(1000000)*tv.tv_sec + tv.tv_usec; 
}

#elif defined(K_POSIX) || defined(K_VX_KERNEL)

kFx(kStatus) xkTimer_ConfigureDefaults()
{
    kCheck(kApiLib_SetTimerQueryHandler(xkTimer_DefaultTickQuery)); 
    kCheck(kApiLib_SetTimerScale(1, 1)); 

    return kOK; 
}

kFx(k64u) xkTimer_DefaultTickQuery()
{
    struct timespec tp; 
    
    if (clock_gettime(CLOCK_MONOTONIC, &tp) != 0)
    {
        return 0;                  
    }

    return k64U(1000000)*tp.tv_sec + tp.tv_nsec/k64U(1000); 
}

#else 

kFx(kStatus) xkTimer_ConfigureDefaults()
{
    return kOK; 
}

kFx(k64u) xkTimer_DefaultTickQuery()
{
    return 0;
}

#endif

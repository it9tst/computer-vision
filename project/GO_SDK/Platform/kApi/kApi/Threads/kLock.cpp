/** 
 * @file    kLock.cpp
 * @brief   Declares the kLock class. 
 *
 * @internal
 * Copyright (C) 2005-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#define K_PLATFORM
#include <kApi/Threads/kLock.h>

kBeginValueEx(k, kLockOption)
    kAddEnumerator(kLockOption, kLOCK_OPTION_TIMEOUT)
    kAddEnumerator(kLockOption, xkLOCK_OPTION_PRIORITY_INHERITANCE)
kEndValueEx()

kBeginClassEx(k, kLock)
    kAddPrivateVMethod(kLock, kObject, VRelease)
kEndClassEx()

kFx(kStatus) kLock_Construct(kLock* lock, kAlloc allocator)
{
    return kLock_ConstructEx(lock, kFALSE, allocator);
} 

kFx(kStatus) kLock_ConstructEx(kLock* lock, kLockOption options, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback(allocator);
    kType type = kTypeOf(kLock);
    kStatus status;

    kCheck(kAlloc_GetObject(alloc, type, lock));

    if (!kSuccess(status = xkLock_Init(*lock, options, type, alloc)))
    {
        kAlloc_FreeRef(alloc, lock);
    }

    return status;
}

#if defined(K_WINDOWS)

kFx(kStatus) xkLock_Init(kLock lock, kLockOption options, kType type, kAlloc allocator)
{
    kObjR(kLock, lock);

    kCheck(kObject_Init(lock, type, allocator)); 

    obj->options = options;
    kZero(obj->criticalSection); 
    kZero(obj->mutex);

    //on Windows, critical sections are faster than mutexes; accordingly, we'll 
    //only use a mutex when the lock needs to support timeouts.
    //priority inheritance isn't supported, but Windows supports random boosting by default
    if ((obj->options & kLOCK_OPTION_TIMEOUT) != 0)
    {
        if (kIsNull(obj->mutex = CreateMutex(NULL, FALSE, NULL)))
        {
            kObject_VRelease(lock);
            return kERROR_OS;
        }
    }
    else 
    {
        InitializeCriticalSection(&obj->criticalSection);
    }

    return kOK; 
}

kFx(kStatus) xkLock_VRelease(kLock lock)
{
    kObj(kLock, lock); 
    
    if ((obj->options & kLOCK_OPTION_TIMEOUT) != 0)
    {
        if (!kIsNull(obj->mutex))
        {
            CloseHandle(obj->mutex);
        }

        obj->mutex = kNULL;
    }
    else
    {
        DeleteCriticalSection(&obj->criticalSection); 
    }

    kCheck(kObject_VRelease(lock)); 

    return kOK; 
}

kFx(kStatus) kLock_Enter(kLock lock)
{
    kObj(kLock, lock);

    if ((obj->options & kLOCK_OPTION_TIMEOUT) != 0)
    {
        DWORD waitResult = WaitForSingleObject(obj->mutex, kOS_INFINITE);

        if (waitResult == WAIT_TIMEOUT)
        {
            return kERROR_TIMEOUT;
        }
        else if (waitResult != WAIT_OBJECT_0)
        {
            return kERROR_OS;
        }
    }
    else
    {
        EnterCriticalSection(&obj->criticalSection);
    }

    return kOK;
}

kFx(kStatus) kLock_EnterEx(kLock lock, k64u timeout)
{
    kObj(kLock, lock);

    kAssert((obj->options & kLOCK_OPTION_TIMEOUT) != 0 || (timeout == kINFINITE)); 

    if ((obj->options & kLOCK_OPTION_TIMEOUT) != 0)
    {
        DWORD osTimeout = (DWORD)xkTimeToKernelTime(timeout);
        DWORD waitResult = WaitForSingleObject(obj->mutex, osTimeout);

        if (waitResult == WAIT_TIMEOUT)
        {
            return kERROR_TIMEOUT;
        }
        else if (waitResult != WAIT_OBJECT_0)
        {
            return kERROR_OS;
        }
    }
    else
    {
        EnterCriticalSection(&obj->criticalSection);
    }

    return kOK;
}

kFx(kStatus) kLock_Exit(kLock lock)
{
    kObj(kLock, lock); 

    if ((obj->options & kLOCK_OPTION_TIMEOUT) != 0)
    {
        if (!ReleaseMutex(obj->mutex))
        {
            return kERROR_OS;
        }
    }
    else
    {
        LeaveCriticalSection(&obj->criticalSection);
    }

    return kOK;
}

#elif defined(K_TI_BIOS)

//Note: this implementation does not support priority inheritance
kFx(kStatus) xkLock_Init(kLock lock, kLockOption options, kType type, kAlloc allocator)
{
    kObjR(kLock, lock); 
    Semaphore_Params params; 
    xdc_runtime_Error_Block eb; 

    kCheck(kObject_Init(lock, type, allocator)); 

    obj->owner = kNULL; 
    obj->handle = kNULL;
    obj->count = 0;
    obj->options = options;

    xdc_runtime_Error_init(&eb);

    ti_sysbios_knl_Semaphore_Params_init(&params);
    params.mode = Semaphore_Mode_BINARY;
   
    if (kIsNull(obj->handle = ti_sysbios_knl_Semaphore_create(1, &params, &eb)))
    {
        kObject_VRelease(lock); 
        return kERROR_OS; 
    }
    
    return kOK; 
}

kFx(kStatus) xkLock_VRelease(kLock lock)
{
    kObj(kLock, lock); 
    
    if (!kIsNull(obj->handle))
    {
         ti_sysbios_knl_Semaphore_delete(&obj->handle);
    }

    kCheck(kObject_VRelease(lock)); 

    return kOK; 
}

kFx(kStatus) kLock_Enter(kLock lock)
{
    kObj(kLock, lock); 
    ti_sysbios_knl_Task_Handle self = ti_sysbios_knl_Task_self(); 

    //this isn't great general-purpose code; if it were intended to work on a wide variety of architectures, 
    //we would use kAtomicPointer for accessing obj->owner. however, the approach below is measurably faster 
    //and is essentially what TI does in the SYS/BIOS gate module. if it's good enough for TI, it's good enough for us. 
    if (obj->owner == self)
    {
        obj->count++; 
        return kOK; 
    }
    else if (ti_sysbios_knl_Semaphore_pend(obj->handle, kOS_INFINITE))
    {
        obj->owner = self; 
        obj->count++; 
        return kOK; 
    }
    else
    {
        return kERROR_TIMEOUT; 
    }
}

kFx(kStatus) kLock_EnterEx(kLock lock, k64u timeout)
{
    kObj(kLock, lock); 
    k32u osTimeout = (k32u) xkTimeToKernelTime(timeout); 
    ti_sysbios_knl_Task_Handle self = ti_sysbios_knl_Task_self(); 

    kAssert(((obj->options & kLOCK_OPTION_TIMEOUT) != 0) || (timeout == kINFINITE)); 

    //this isn't great general-purpose code; if it were intended to work on a wide variety of architectures, 
    //we would use kAtomicPointer for accessing obj->owner. however, the approach below is measurably faster 
    //and is essentially what TI does in the SYS/BIOS gate module. if it's good enough for TI, it's good enough for us. 
    if (obj->owner == self)
    {
        obj->count++; 
        return kOK; 
    }
    else if (ti_sysbios_knl_Semaphore_pend(obj->handle, osTimeout))
    {
        obj->owner = self; 
        obj->count++; 
        return kOK; 
    }
    else
    {
        return kERROR_TIMEOUT; 
    }
}

kFx(kStatus) kLock_Exit(kLock lock)
{
    kObj(kLock, lock); 

    kAssert(obj->count != 0); 

    obj->count--; 

    if (obj->count == 0)
    {
        obj->owner = kNULL;
        ti_sysbios_knl_Semaphore_post(obj->handle);        
    }

    return kOK;
}

#elif defined(K_VX_KERNEL)

kFx(kStatus) xkLock_Init(kLock lock, kLockOption options, kType type, kAlloc allocator)
{
    kObjR(kLock, lock); 


    kCheck(kObject_Init(lock, type, allocator)); 
       
    obj->options = options;

    if ((options & xkLOCK_OPTION_PRIORITY_INHERITANCE) != 0)
    {
        obj->id = semMCreate(SEM_Q_PRIORITY | SEM_INVERSION_SAFE); 
    }
    else
    {
        obj->id = semMCreate(SEM_Q_FIFO); 
    }
    
    if (obj->id == SEM_ID_NULL)
    {
        kObject_VRelease(lock); 
        return kERROR; 
    }

    return kOK; 
}

kFx(kStatus) xkLock_VRelease(kLock lock)
{
    kObj(kLock, lock); 
    
    if (obj->id != SEM_ID_NULL)
    {
        semDelete(obj->id);
    }

    kCheck(kObject_VRelease(lock)); 

    return kOK; 
}

kFx(kStatus) kLock_Enter(kLock lock)
{
    kObj(kLock, lock);
 
    return (semMTake_inline(obj->id, kOS_INFINITE, SEM_NO_ID_VALIDATE | SEM_NO_ERROR_CHECK | SEM_NO_SYSTEM_VIEWER) == OK) ? kOK : kERROR_TIMEOUT;      
}

kFx(kStatus) kLock_EnterEx(kLock lock, k64u timeout)
{
    kObj(kLock, lock);
    _Vx_ticks_t osTimeout = (_Vx_ticks_t) xkTimeToKernelTime(timeout); 

    kAssert(((obj->options & kLOCK_OPTION_TIMEOUT) != 0) || (timeout == kINFINITE)); 

    return (semMTake_inline(obj->id, osTimeout, SEM_NO_ID_VALIDATE | SEM_NO_ERROR_CHECK | SEM_NO_SYSTEM_VIEWER) == OK) ? kOK : kERROR_TIMEOUT;      
}

kFx(kStatus) kLock_Exit(kLock lock)
{
    kObj(kLock, lock); 

    return (semMGive_inline(obj->id, SEM_NO_ID_VALIDATE | SEM_NO_ERROR_CHECK | SEM_NO_EVENT_SEND | SEM_NO_SYSTEM_VIEWER ) == OK) ? kOK : kERROR_OS; 
}

#elif defined(K_POSIX)

kFx(kStatus) xkLock_Init(kLock lock, kLockOption options, kType type, kAlloc allocator)
{
    kObjR(kLock, lock); 
    pthread_mutexattr_t mutexattr; 
    kBool attrInit = kFALSE; 
    kBool mutexInit = kFALSE; 
    kStatus status; 
    
    kCheck(kObject_Init(lock, type, allocator)); 

    kZero(obj->mutex);
    obj->options = options;

    kTry
    {
        kTest(pthread_mutexattr_init(&mutexattr) == 0); 
        attrInit = kTRUE; 

        kTest(pthread_mutexattr_settype(&mutexattr, PTHREAD_MUTEX_RECURSIVE) == 0);

        if ((obj->options & xkLOCK_OPTION_PRIORITY_INHERITANCE) != 0)
        {
            kTest(pthread_mutexattr_setprotocol(&mutexattr, PTHREAD_PRIO_INHERIT) == 0);
        }

        kTest(pthread_mutex_init(&obj->mutex, &mutexattr) == 0);
        mutexInit = kTRUE; 
    }
    kCatchEx(&status)
    {
        if (mutexInit) pthread_mutex_destroy(&obj->mutex);
        
        kObject_VRelease(lock); 

        kEndCatchEx(status); 
    }
    kFinallyEx
    {
        if (attrInit) pthread_mutexattr_destroy(&mutexattr); 

        kEndFinallyEx();
    }

    return kOK; 
}

kFx(kStatus) xkLock_VRelease(kLock lock)
{
    kObj(kLock, lock); 
    
    kCheck(pthread_mutex_destroy(&obj->mutex) == 0);

    kCheck(kObject_VRelease(lock)); 

    return kOK; 
}

kFx(kStatus) kLock_Enter(kLock lock)
{
    kObj(kLock, lock);

    kCheckTrue(pthread_mutex_lock(&obj->mutex) == 0, kERROR_OS);

    return kOK; 
}

kFx(kStatus) kLock_EnterEx(kLock lock, k64u timeout)
{
    kObj(kLock, lock);
    struct timespec tm;
    int result = 0;

    kAssert(((obj->options & kLOCK_OPTION_TIMEOUT) != 0) || (timeout == kINFINITE)); 

    if (timeout == kINFINITE)
    {
        kCheckTrue(pthread_mutex_lock(&obj->mutex) == 0, kERROR_OS);
    }
    else
    {
        kCheck(xkFormatTimeout(timeout, &tm));

        result = pthread_mutex_timedlock(&obj->mutex, &tm);

        if (result == ETIMEDOUT)
        {
            return kERROR_TIMEOUT;
        }
        else if (result != 0)
        {
            return kERROR_OS; 
        }
    }

    return kOK; 
}

kFx(kStatus) kLock_Exit(kLock lock)
{
    kObj(kLock, lock); 

    kCheck(pthread_mutex_unlock(&obj->mutex) == 0);

    return kOK;
}

#endif

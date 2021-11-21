/** 
 * @file    kSemaphore.cpp
 *
 * @internal
 * Copyright (C) 2010-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#define K_PLATFORM
#include <kApi/Threads/kSemaphore.h>

kBeginClassEx(k, kSemaphore)
    kAddPrivateVMethod(kSemaphore, kObject, VRelease)
kEndClassEx()

kFx(kStatus) kSemaphore_Construct(kSemaphore* semaphore, kSize initialCount, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback(allocator);
    kType type = kTypeOf(kSemaphore); 
    kStatus status; 

    kCheck(kAlloc_GetObject(alloc, type, semaphore)); 

    if (!kSuccess(status = xkSemaphore_Init(*semaphore, type, initialCount, alloc)))
    {
        kAlloc_FreeRef(alloc, semaphore); 
    }

    return status; 
} 

#if defined(K_WINDOWS)

kFx(kStatus) xkSemaphore_Init(kSemaphore semaphore, kType type, kSize initialCount, kAlloc allocator)
{
    kObjR(kSemaphore, semaphore); 
    
    kCheck(kObject_Init(semaphore, type, allocator)); 

    if (kIsNull(obj->handle = CreateSemaphore(kNULL, (LONG)initialCount, kSEMAPHORE_MAX_VALUE, kNULL)))
    {
        kObject_VRelease(semaphore); 
        return kERROR; 
    }

    return kOK; 
}

kFx(kStatus) xkSemaphore_VRelease(kSemaphore semaphore)
{
    kObj(kSemaphore, semaphore); 
    
    kCheck(CloseHandle(obj->handle));
    kCheck(kObject_VRelease(semaphore)); 
    
    return kOK; 
}

kFx(kStatus) kSemaphore_Post(kSemaphore semaphore)
{
    kObj(kSemaphore, semaphore); 

    kCheck(ReleaseSemaphore(obj->handle, 1, kNULL)); 

    return kOK;
}

kFx(kStatus) kSemaphore_Wait(kSemaphore semaphore, k64u timeout)
{
    kObj(kSemaphore, semaphore); 
    DWORD osTimeout = (DWORD) xkTimeToKernelTime(timeout); 

    switch (WaitForSingleObject(obj->handle, osTimeout))
    {
        case WAIT_OBJECT_0:     return kOK; 
        case WAIT_TIMEOUT:      return kERROR_TIMEOUT; 
        default:                return kERROR; 
    }           
}

#elif defined(K_TI_BIOS)

kFx(kStatus) xkSemaphore_Init(kSemaphore semaphore, kType type, kSize initialCount, kAlloc allocator)
{
    kObjR(kSemaphore, semaphore); 
    Semaphore_Params params; 
    xdc_runtime_Error_Block eb; 

    kCheck(kObject_Init(semaphore, type, allocator)); 

    obj->handle = kNULL; 

    xdc_runtime_Error_init(&eb);

    ti_sysbios_knl_Semaphore_Params_init(&params);
    params.mode = Semaphore_Mode_COUNTING;

    if (kIsNull(obj->handle = ti_sysbios_knl_Semaphore_create((k32s)initialCount, &params, &eb)))
    {
        kObject_VRelease(semaphore); 
        return kERROR_OS; 
    }

    return kOK; 
}

kFx(kStatus) xkSemaphore_VRelease(kSemaphore semaphore)
{
    kObj(kSemaphore, semaphore); 
    
    if (!kIsNull(obj->handle))
    {
        ti_sysbios_knl_Semaphore_delete(&obj->handle);
    }

    kCheck(kObject_VRelease(semaphore)); 
    
    return kOK; 
}

kFx(kStatus) kSemaphore_Post(kSemaphore semaphore)
{
    kObj(kSemaphore, semaphore); 

    ti_sysbios_knl_Semaphore_post(obj->handle); 

    return kOK;
}

kFx(kStatus) kSemaphore_Wait(kSemaphore semaphore, k64u timeout)
{
    kObj(kSemaphore, semaphore); 
    k32u osTimeout = (k32u) xkTimeToKernelTime(timeout); 

    return ti_sysbios_knl_Semaphore_pend(obj->handle, osTimeout) ? kOK : kERROR_TIMEOUT; 
}

#elif defined(K_VX_KERNEL)

kFx(kStatus) xkSemaphore_Init(kSemaphore semaphore, kType type, kSize initialCount, kAlloc allocator)
{
    kObjR(kSemaphore, semaphore); 
    
    kCheck(kObject_Init(semaphore, type, allocator)); 
     
    obj->id = semCCreate(SEM_Q_FIFO, (int)initialCount); 

    if (obj->id == SEM_ID_NULL)
    {
        kObject_VRelease(semaphore); 
        return kERROR; 
    }

    return kOK; 
}

kFx(kStatus) xkSemaphore_VRelease(kSemaphore semaphore)
{
    kObj(kSemaphore, semaphore); 
    
    semDelete(obj->id); 
    
    kCheck(kObject_VRelease(semaphore)); 
    
    return kOK; 
}

kFx(kStatus) kSemaphore_Post(kSemaphore semaphore)
{
    kObj(kSemaphore, semaphore); 

    return (semGive(obj->id) == OK) ? kOK : kERROR_OS; 
}

kFx(kStatus) kSemaphore_Wait(kSemaphore semaphore, k64u timeout)
{
    kObj(kSemaphore, semaphore); 
    _Vx_ticks_t osTimeout = (_Vx_ticks_t) xkTimeToKernelTime(timeout); 

    return (semTake(obj->id, osTimeout) == OK) ? kOK : kERROR_TIMEOUT;            
}

#elif defined(K_POSIX)

kFx(kStatus) xkSemaphore_Init(kSemaphore semaphore, kType type, kSize initialCount, kAlloc allocator)
{
    kObjR(kSemaphore, semaphore); 

    kCheckArgs(initialCount <= k32U_MAX);

    kCheck(kObject_Init(semaphore, type, allocator));     
        
    if (sem_init(&obj->sem, 0, (k32u)initialCount) != 0)
    {
        kObject_VRelease(semaphore);
        return kERROR_OS;
    }
    
    return kOK; 
}

kFx(kStatus) xkSemaphore_VRelease(kSemaphore semaphore)
{
    kObj(kSemaphore, semaphore); 
    
    kCheckTrue(sem_destroy(&obj->sem) == 0, kERROR_OS);
    
    kCheck(kObject_VRelease(semaphore)); 
    
    return kOK; 
}

kFx(kStatus) kSemaphore_Post(kSemaphore semaphore)
{
    kObj(kSemaphore, semaphore); 

    kCheckTrue(sem_post(&obj->sem) == 0, kERROR_OS);

    return kOK;
}

kFx(kStatus) kSemaphore_Wait(kSemaphore semaphore, k64u timeout)
{
    kObj(kSemaphore, semaphore); 
    
    if (timeout == kINFINITE)
    {
        kCheckTrue(sem_wait(&obj->sem) == 0, kERROR_OS);
    }
    else
    {
        struct timespec tm; 

        kCheck(xkFormatTimeout(timeout, &tm)); 

        kCheckTrue(sem_timedwait(&obj->sem, &tm) == 0, kERROR_TIMEOUT);
    }
    
    return kOK;
}

#endif

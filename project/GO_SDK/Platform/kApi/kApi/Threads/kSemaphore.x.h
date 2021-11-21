/** 
 * @file    kSemaphore.x.h
 *
 * @internal
 * Copyright (C) 2010-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_SEMAPHORE_X_H
#define K_API_SEMAPHORE_X_H

kDeclareClassEx(k, kSemaphore, kObject)

#if defined(K_PLATFORM)

#if defined(K_WINDOWS)

#   define kSEMAPHORE_MAX_VALUE             (0x7FFFFFFF)

#   define xkSemaphorePlatformFields()       \
        HANDLE handle; 

#elif defined(K_TI_BIOS)

#   define xkSemaphorePlatformFields()       \
        Semaphore_Handle handle;

#elif defined(K_VX_KERNEL)

#   define xkSemaphorePlatformFields()       \
        SEM_ID id;

#elif defined(K_POSIX)

#   include <kApi/Threads/kLock.h>

#   define xkSemaphorePlatformFields()       \
        sem_t sem;

#endif

typedef struct kSemaphoreClass
{
    kObjectClass base; 
    xkSemaphorePlatformFields()
} kSemaphoreClass; 

/* 
* Private methods. 
*/

kFx(kStatus) xkSemaphore_Init(kSemaphore semaphore, kType type, kSize initialCount, kAlloc allocator); 
kFx(kStatus) xkSemaphore_VRelease(kSemaphore semaphore); 

#if defined(K_TI_BIOS)

    kInlineFx(kStatus) kSemaphore_PostInline(kSemaphore semaphore)
    {
        kObj(kSemaphore, semaphore);

        ti_sysbios_knl_Semaphore_post(obj->handle);

        return kOK; 
    }

#endif

#endif

#endif

/** 
 * @file    kLock.x.h
 *
 * @internal
 * Copyright (C) 2005-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_LOCK_X_H
#define K_API_LOCK_X_H

kDeclareValueEx(k, kLockOption, kValue)
kDeclareClassEx(k, kLock, kObject)

#define xkLOCK_OPTION_PRIORITY_INHERITANCE          (0x2)       ///< Priority of thread that acquires the lock is increased to priority of highest priority waiting thread. Not supported on all platforms.

#if defined(K_PLATFORM)

#if defined(K_WINDOWS)

#   define xkLockPlatformFields()            \
        CRITICAL_SECTION criticalSection;   \
        HANDLE mutex;

#elif defined(K_TI_BIOS)

#   define xkLockPlatformFields()                            \
        volatile ti_sysbios_knl_Task_Handle owner;          \
        k32u count;                                         \
        Semaphore_Handle handle;

#elif defined(K_VX_KERNEL)

#   define xkLockPlatformFields()            \
        SEM_ID id;

#else

#   define xkLockPlatformFields()            \
        pthread_mutex_t mutex;        

#endif

typedef struct kLockClass
{
    kObjectClass base; 
    kLockOption options;           //lock options
    xkLockPlatformFields()
} kLockClass;

/* 
* Private methods. 
*/

kFx(kStatus) xkLock_Init(kLock lock, kLockOption options, kType type, kAlloc allocator); 
kFx(kStatus) xkLock_VRelease(kLock lock); 

#endif

#endif

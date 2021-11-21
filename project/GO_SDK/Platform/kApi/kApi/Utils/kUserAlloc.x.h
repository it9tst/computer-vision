/** 
 * @file    kUserAlloc.x.h
 *
 * @internal
 * Copyright (C) 2005-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_USER_ALLOC_X_H
#define K_API_USER_ALLOC_X_H

typedef struct kUserAllocClass
{
    kAllocClass base;
    kApiMemAllocFx allocFx;         //User allocation function.
    kApiMemFreeFx freeFx;           //User free function. 
    kPointer provider;              //User context pointer. 
} kUserAllocClass;

kDeclareClassEx(k, kUserAlloc, kAlloc)

/* 
* Private methods. 
*/

kFx(kStatus) xkUserAlloc_Init(kUserAlloc object, kType type, kApiMemAllocFx allocFx, kApiMemFreeFx freeFx, kPointer provider, kAlloc alloc); 
kFx(kStatus) xkUserAlloc_VRelease(kUserAlloc object); 

kFx(kStatus) xkUserAlloc_VGet(kUserAlloc object, kSize size, void* mem, kMemoryAlignment alignment);
kFx(kStatus) xkUserAlloc_VFree(kUserAlloc object, void* mem); 

#endif

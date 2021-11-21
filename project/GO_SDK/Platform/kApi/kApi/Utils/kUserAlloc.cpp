/** 
 * @file    kUserAlloc.cpp
 *
 * @internal
 * Copyright (C) 2005-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#define K_PLATFORM
#include <kApi/Utils/kUserAlloc.h>
#include <kApi/kApiLib.h>

kBeginClassEx(k, kUserAlloc)
    kAddPrivateVMethod(kUserAlloc, kObject, VRelease)
    kAddPrivateVMethod(kUserAlloc, kAlloc, VGet)
    kAddPrivateVMethod(kUserAlloc, kAlloc, VFree)
kEndClassEx()

kFx(kStatus) kUserAlloc_Construct(kUserAlloc* object, kApiMemAllocFx allocFx, kApiMemFreeFx freeFx, kPointer provider, kAlloc allocator)
{
    kUserAlloc output = kNULL; 
    kType type = kTypeOf(kUserAlloc); 
    kStatus status; 
    
    if (kIsNull(allocator))
    {
        //this type can be used before the type system is fully initialized
        kCheck(xkSysMemAlloc(sizeof(kUserAllocClass), &output)); 
    }
    else
    {
        kCheck(kAlloc_GetObject(allocator, kTypeOf(kUserAlloc), &output)); 
    }

    if (!kSuccess(status = xkUserAlloc_Init(output, type, allocFx, freeFx, provider, allocator)))
    {
        xkSysMemFree(output); 
        return status; 
    }

    *object = output;

    return kOK; 
} 

kFx(kStatus) xkUserAlloc_Init(kUserAlloc object, kType type, kApiMemAllocFx allocFx, kApiMemFreeFx freeFx, kPointer provider, kAlloc alloc)
{
    kObjR(kUserAlloc, object); 

    kCheck(kAlloc_Init(object, type, alloc)); 

    obj->allocFx = allocFx; 
    obj->freeFx = freeFx; 
    obj->provider = provider; 

    return kOK; 
}

kFx(kStatus) xkUserAlloc_VRelease(kUserAlloc object)
{
    kCheck(kAlloc_VRelease(object)); 
    
    return kOK; 
}

kFx(kStatus) xkUserAlloc_VGet(kUserAlloc object, kSize size, void* mem, kMemoryAlignment alignment)
{
    kObj(kUserAlloc, object); 

    return obj->allocFx(obj->provider, size, mem, alignment); 
}

kFx(kStatus) xkUserAlloc_VFree(kUserAlloc object, void* mem)
{
    kObj(kUserAlloc, object); 

    return obj->freeFx(obj->provider, mem); 
}

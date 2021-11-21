/** 
 * @file    kAlloc.cpp
 *
 * @internal
 * Copyright (C) 2005-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <kApi/kAlloc.h>
#include <kApi/kApiLib.h>
#include <kApi/Utils/kUserAlloc.h>
#include <kApi/Utils/kDebugAlloc.h>
#include <kApi/Utils/kUtils.h>

kBeginFullClassEx(k, kAlloc)
    kAddVMethod(kAlloc, kObject, VRelease)
    kAddVMethod(kAlloc, kAlloc, VGet)
    kAddVMethod(kAlloc, kAlloc, VFree)
    kAddVMethod(kAlloc, kAlloc, VCopy)
kEndFullClassEx()

kFx(kStatus) xkAlloc_InitStatic()
{
    kAllocStatic* sobj = xkAlloc_Static(); 
    kApiAllocConstructFx constructHandler = kApiLib_AppAllocConstructHandler();

    //initialize leak detection
    kApiLib_SetLeaksDetected(0); 

    //construct system memory allocator
    kCheck(kUserAlloc_Construct(&sobj->systemAlloc, kApiLib_MemAllocHandler(), kApiLib_MemFreeHandler(), kApiLib_MemAllocProvider(), kNULL)); 

    //create the application memory allocator
    kCheck(constructHandler(&sobj->appAlloc, sobj->systemAlloc)); 

    //in debug builds, a debug allocator is layered on top of the app allocator
    if (K_DEBUG_ENABLED)
    {
        kCheck(kDebugAlloc_Construct(&sobj->appAlloc, "App", sobj->appAlloc, sobj->appAlloc)); 
    }

    return kOK; 
}

kFx(kStatus) xkAlloc_EndInitStatic()
{
    kAllocStatic* sobj = xkAlloc_Static(); 

    if (K_DEBUG_ENABLED)
    {
        kCheck(kAssembly_AddUnloadedHandler(xkAlloc_OnAssemblyUnloaded, kNULL));        
    }

    sobj->assembly = kAssemblyOf(kApiLib); 

    return kOK; 
}

kFx(kStatus) xkAlloc_ReleaseStatic()
{
    kAllocStatic* sobj = xkAlloc_Static(); 
    kApiAllocDestroyFx destroyHandler = kApiLib_AppAllocDestroyHandler();

    if (kType_StaticInitialized(kTypeOf(kAssembly)))
    {
        kAssembly_RemoveUnloadedHandler(xkAlloc_OnAssemblyUnloaded, kNULL);        
    }

    kCheck(xkAlloc_FinalizeDebug(&sobj->appAlloc)); 

    kCheck(destroyHandler(sobj->appAlloc)); 

    kCheck(kObject_Destroy(sobj->systemAlloc)); 

    return kOK; 
}

kFx(kStatus) xkAlloc_DefaultConstructAppAlloc(kAlloc* appAlloc, kAlloc systemAlloc)
{
    *appAlloc = systemAlloc; 

    return kOK; 
}

kFx(kStatus) xkAlloc_DefaultDestroyAppAlloc(kAlloc appAlloc)
{
    return kOK; 
}

kFx(kStatus) xkAlloc_OnAssemblyUnloaded(kPointer unused, kAssembly assembly, kPointer args)
{
    kAllocStatic* sobj = xkAlloc_Static(); 

    //detect any memory allocations that appear to be leaked objects
    if (kObject_Is(sobj->appAlloc, kTypeOf(kDebugAlloc)))
    {
        kCheck(kDebugAlloc_DetectLeakedAssemblyObjects(sobj->appAlloc, 0, assembly)); 
    }

    return kOK; 
}

kFx(kStatus) xkAlloc_FinalizeDebug(kAlloc* alloc)
{
    kAllocStatic* sobj = xkAlloc_Static(); 

    if (kObject_Is(*alloc, kTypeOf(kDebugAlloc)))
    {
        kAlloc innerAlloc = kObject_Alloc(*alloc); 

        //detect any leaked kApiLib objects
        kCheck(kDebugAlloc_DetectLeakedAssemblyObjects(*alloc, 0, sobj->assembly)); 

        //log all outstanding allocations
        if (kDebugAlloc_Allocated(*alloc) > 0)
        {
            kApiLib_SetLeaksDetected(kDebugAlloc_Allocated(*alloc)); 

            //emit leaks to the kApi log handler, if leak logging is enabled
            if (kApiLib_LeakLoggingEnabled())
            {
                kCheck(kDebugAlloc_LogAllocations(*alloc, 0)); 
                kCheck(kDebugAlloc_Clear(*alloc)); 
            }
        }

        kCheck(kDestroyRef(alloc)); 
        *alloc = innerAlloc; 
    }

    return kOK; 
}

kFx(kStatus) kAlloc_Init(kAlloc alloc, kType type, kAlloc allocator)
{
    kObjR(kAlloc, alloc);

    kCheck(kObject_Init(alloc, type, allocator));

    obj->traits = 0;   //default traits

    return kOK;
}

kFx(kStatus) kAlloc_VRelease(kAlloc alloc)
{
    return kObject_VRelease(alloc); 
}

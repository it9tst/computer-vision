/** 
 * @file    kAlloc.x.h
 *
 * @internal
 * Copyright (C) 2005-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_ALLOC_X_H
#define K_API_ALLOC_X_H

typedef struct kAllocStatic
{
    kAlloc systemAlloc;             //Allocates directly from the underlying system (no wrappers).
    kAlloc appAlloc;                //Created for use by application (layered on systemAlloc).
    kAssembly assembly;             //kApi assembly reference. 
} kAllocStatic;

typedef struct kAllocClass
{
    kObjectClass base; 
    kAllocTrait traits;             //Bitset of allocator traits.
} kAllocClass;

typedef struct kAllocVTable
{
    kObjectVTable base; 
    kStatus (kCall* VGet)(kAlloc alloc, kSize size, void* mem, kMemoryAlignment alignment);
    kStatus (kCall* VFree)(kAlloc alloc, void* mem); 
    kStatus (kCall* VCopy)(kAlloc alloc, kAlloc destAlloc, void* dest, kAlloc srcAlloc, const void* src, kSize size, kObject context);
} kAllocVTable; 

kDeclareFullClassEx(k, kAlloc, kObject)

/*
 * Forward declarations.
 */

kInlineFx(kBool) kAlloc_CanGetObject(kAlloc alloc);

/* 
 * Private methods. 
 */

kFx(kStatus) xkAlloc_InitStatic(); 
kFx(kStatus) xkAlloc_EndInitStatic(); 
kFx(kStatus) xkAlloc_ReleaseStatic(); 

kFx(kStatus) xkAlloc_DefaultConstructAppAlloc(kAlloc* appAlloc, kAlloc systemAlloc); 
kFx(kStatus) xkAlloc_DefaultDestroyAppAlloc(kAlloc appAlloc); 

kFx(kStatus) xkAlloc_OnAssemblyUnloaded(kPointer unused, kAssembly assembly, kPointer args);  

kFx(kStatus) xkAlloc_FinalizeDebug(kAlloc* alloc); 

kInlineFx(kAlloc) xkAlloc_SelectCopyAlloc(kAlloc destAlloc, kAlloc srcAlloc, kObject context)
{
    kAllocTrait destTraits = kAlloc_Traits(destAlloc); 
    kAllocTrait srcTraits = kAlloc_Traits(srcAlloc); 

    if (kAllocTrait_IsForeign(destTraits | srcTraits))
    {
        return kAllocTrait_IsForeign(destTraits) ? destAlloc : srcAlloc; 
    }
    else if (kAllocTrait_SupportsContext(destTraits | srcTraits))
    {
        return kAllocTrait_SupportsContext(destTraits) ? destAlloc : srcAlloc; 
    }
    else
    {
        return destAlloc;
    }
}

#endif

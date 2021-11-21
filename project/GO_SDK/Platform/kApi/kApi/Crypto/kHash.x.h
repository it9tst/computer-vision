/** 
 * @file    kHash.x.h
 *
 * @internal
 * Copyright (C) 2017-2021 by LMI Technologies Inc.  All rights reserved.
 */
#ifndef K_API_HASH_X_H
#define K_API_HASH_X_H

typedef struct kHashClass
{
    kObjectClass base; 
} kHashClass;

typedef struct kHashVTable
{
    kObjectVTable base; 

    kStatus (kCall* VUpdate)(kHash hash, const void* buffer, kSize size);
    kStatus (kCall* VDigest)(kHash hash, void* buffer, kSize size);
    kStatus (kCall* VClear)(kHash hash);
    kSize (kCall* VDigestSize)(kHash hash);
} kHashVTable; 

kDeclareVirtualClassEx(k, kHash, kObject)

/* 
* Private methods. 
*/

kFx(kStatus) xkHash_Init(kHash hash, kType type, kAlloc allocator);

kInlineFx(kStatus) xkHash_VRelease(kHash hash)
{
    return kObject_VRelease(hash);
}

kInlineFx(kStatus) xkHash_VUpdate(kHash hash, const void* buffer, kSize size)
{
    return kERROR_UNIMPLEMENTED;
}

kInlineFx(kStatus) xkHash_VDigest(kHash hash, void* buffer, kSize size)
{
    return kERROR_UNIMPLEMENTED;
}

kInlineFx(kStatus) xkHash_VClear(kHash hash)
{
    return kERROR_UNIMPLEMENTED;
}

kInlineFx(kSize) xkHash_VDigestSize(kHash hash)
{
    return 0;
}

#endif

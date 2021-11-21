/** 
 * @file    kHash.cpp
 *
 *
 * @internal
 * Copyright (C) 2017-2021 by LMI Technologies Inc.  All rights reserved.
 */

#include <kApi/Crypto/kHash.h>
#include <kApi/Data/kString.h>

kBeginVirtualClassEx(k, kHash)
    kAddFlags(kHash, kTYPE_FLAGS_ABSTRACT)

    kAddPrivateVMethod(kHash, kObject, VRelease)
    kAddPrivateVMethod(kHash, kHash, VDigest)
    kAddPrivateVMethod(kHash, kHash, VUpdate)
    kAddPrivateVMethod(kHash, kHash, VClear)
    kAddPrivateVMethod(kHash, kHash, VDigestSize)

kEndVirtualClassEx()

kFx(kStatus) xkHash_Init(kHash hash, kType type, kAlloc allocator)
{
    kCheck(kObject_Init(hash, type, allocator)); 


    return kOK; 
}

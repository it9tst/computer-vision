/** 
 * @file    kHash.h
 * @brief   Declares the kHash class. 
 *
 * @internal
 * Copyright (C) 2017-2021 by LMI Technologies Inc.  All rights reserved.
 */
#ifndef K_API_HASH_H
#define K_API_HASH_H

#include <kApi/kApiDef.h>
#include <kApi/Crypto/kHash.x.h>

/**
 * @class   kHash
 * @extends kObject
 * @ingroup kFireSync-Hash
 * @brief   Abstract base class providing hash functionality.
 */
//typedef kObject kHash;   --forward-declared in kApiDef.x.h

/** 
 * Updates the hash with the data.
 *
 * Repeated calls are equivalent to a single call with the concatenation 
 * of all the arguments. 
 * 
 * @public              @memberof kHash
 * @param   hash        Hash object. 
 * @param   buffer      Buffer containing the data.
 * @param   size        Size of the buffer, in bytes.
 * @return              Operation status. 
 */
kInlineFx(kStatus) kHash_Update(kHash hash, const void* buffer, kSize size)
{
    return xkHash_VTable(hash)->VUpdate(hash, buffer, size);
}

/** 
 * Returns the digest. 
 * 
 * The size of the buffer in bytes must be at least kHash_DigestSize(). A call of
 * this function does not change the internal buffers.
 * 
 * @public              @memberof kHash
 * @param   hash        Hash object. 
 * @param   buffer      Receives the digest for the data.
 * @param   size        Size of the buffer in bytes.
 * @return              Operation status. 
 */
kInlineFx(kStatus) kHash_Digest(kHash hash, void* buffer, kSize size)
{
    return xkHash_VTable(hash)->VDigest(hash, buffer, size);
}

/** 
 * Clears the internal buffer so instance can be updated with new data.
 * 
 * @public              @memberof kHash
 * @param   hash        Hash object. 
 * @return              Operation status. 
 */
kInlineFx(kStatus) kHash_Clear(kHash hash)
{
    return xkHash_VTable(hash)->VClear(hash);
}

/** 
 * Returns the message digest length in bytes.
 * 
 * @public              @memberof kHash
 * @param   hash        Hash object. 
 * @return              Digest length in bytes. 
 */
kInlineFx(kSize) kHash_DigestSize(kHash hash)
{
    return xkHash_VTable(hash)->VDigestSize(hash);
}

#endif

/** 
 * @file    kSha1Hash.h
 * @brief   Declares the kSha1Hash class. 
 *
 * @internal
 * Copyright (C) 2017-2021 by LMI Technologies Inc.  All rights reserved.
 */
#ifndef K_API_SHA1_HASH_H
#define K_API_SHA1_HASH_H

#include <kApi/kApiDef.h>
#include <kApi/Crypto/kHash.h>
#include <kApi/Crypto/kSha1Hash.x.h>

/**
 * @class   kSha1Hash
 * @extends kHash
 * @ingroup kFireSync-Hash
 * @brief   Sha1 hash implementation.
 */
//typedef kHash kSha1Hash;   --forward-declared in kFsDef.x.h

/** 
 * Constructs a kSha1Hash instance. 
 * 
 * @public                  @memberof kSha1Hash
 * @param   hash            Receives constructed sha1 object. 
 * @param   allocator       Memory allocator (or kNULL for default). 
 * @return                  Operation status. 
 */
kFx(kStatus) kSha1Hash_Construct(kSha1Hash* hash, kAlloc allocator);

#endif

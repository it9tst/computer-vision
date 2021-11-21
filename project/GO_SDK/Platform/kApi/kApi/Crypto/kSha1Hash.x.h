/** 
 * @file    kSha1Hash.x.h
 *
 * @internal
 * Copyright (C) 2017-2021 by LMI Technologies Inc.  All rights reserved.
 */
#ifndef K_API_SHA1_HASH_X_H
#define K_API_SHA1_HASH_X_H

#define xkSHA1_HASH_DIGEST_SIZE       (20)
#define xkSHA1_HASH_STATE_SIZE        (5)
#define xkSHA1_HASH_COUNT_SIZE        (2)
#define xkSHA1_HASH_BUFFER_SIZE       (64)

typedef struct kSha1HashClass
{
    kHashClass base;

    k32u state[xkSHA1_HASH_STATE_SIZE];
    kSize count[xkSHA1_HASH_COUNT_SIZE];
    kByte buffer[xkSHA1_HASH_BUFFER_SIZE];
} kSha1HashClass;

kDeclareClassEx(k, kSha1Hash, kHash)

/* 
* Private methods. 
*/

kFx(kStatus) xkSha1Hash_Init(kSha1Hash, kType type, kAlloc alloc); 
kFx(kStatus) xkSha1Hash_VRelease(kSha1Hash hash);

kFx(kStatus) xkSha1Hash_VUpdate(kSha1Hash hash, const void* buffer, kSize size);
kFx(kStatus) xkSha1Hash_VDigest(kSha1Hash hash, void* buffer, kSize size);
kFx(kStatus) xkSha1Hash_VClear(kSha1Hash hash);

kFx(kSize) xkSha1Hash_VDigestSize(kSha1Hash hash);

// Transforms the state. State must point to at least 5 bytes, buffer to 64 bytes.
kFx(kStatus) xkSha1Hash_UpdateBuffer(kSize* count, k32u* state, kByte* bufferToUpdate, const void* buffer, kSize size);

kFx(void) xkSha1Hash_Transform(k32u* state5, const kByte* buffer64);

#endif

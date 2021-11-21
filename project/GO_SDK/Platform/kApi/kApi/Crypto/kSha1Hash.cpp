/** 
 * @file    kSha1Hash.cpp
 *
 * @internal
 * Copyright (C) 2017-2021 by LMI Technologies Inc.  All rights reserved.
 * This code is inspired by the Sha1 implementation in Ghostscript (which is in Public Domain)
 * http://svn.ghostscript.com/jbig2dec/trunk/sha1.c
 */
#include <kApi/Crypto/kSha1Hash.h>

#include <kApi/Data/kString.h>

kBeginClassEx(k, kSha1Hash)
    kAddPrivateVMethod(kSha1Hash, kObject, VRelease)
    kAddPrivateVMethod(kSha1Hash, kHash, VUpdate)
    kAddPrivateVMethod(kSha1Hash, kHash, VDigest)
    kAddPrivateVMethod(kSha1Hash, kHash, VClear)
    kAddPrivateVMethod(kSha1Hash, kHash, VDigestSize)
kEndClassEx()

/* SHA1 initialization vectors*/
static const k32u kSha1Hash_sha1state[] = { 0x67452301, 0xEFCDAB89, 0x98BADCFE, 0x10325476, 0xc3d2e1f0 };
static const k32u kSha1Hash_sha1k[]     = { 0x5a827999, 0x6ed9eba1, 0x8f1bbcdc, 0xca62c1d6 };

#define kSHA1_ROL32(val,bits) (((val)<<(bits))|((val)>>(32-(bits))))

#if K_ENDIANNESS == kENDIANNESS_BIG
#   define kSHA1_BLK0(i) block[i]
#else
#   define kSHA1_BLK0(i) (block[i] = (kSHA1_ROL32(block[i], 24) & 0xFF00FF00) |(kSHA1_ROL32(block[i], 8) & 0x00FF00FF))
#endif

#define kSHA1_BLK(i) (block[i & 15] = \
    kSHA1_ROL32(block[(i + 13) & 15] ^ block[(i+8) & 15] ^ block[(i + 2) & 15] ^ block[i&15], 1))

#define kSHA1_R0(v,w,x,y,z,i) z+=((w&(x^y))^y)     + kSHA1_BLK0(i) + kSha1Hash_sha1k[0] + kSHA1_ROL32(v,5); w = kSHA1_ROL32(w,30);
#define kSHA1_R1(v,w,x,y,z,i) z+=((w&(x^y))^y)     + kSHA1_BLK(i)  + kSha1Hash_sha1k[0] + kSHA1_ROL32(v,5); w = kSHA1_ROL32(w,30);
#define kSHA1_R2(v,w,x,y,z,i) z+=(w^x^y)           + kSHA1_BLK(i)  + kSha1Hash_sha1k[1] + kSHA1_ROL32(v,5); w = kSHA1_ROL32(w,30);
#define kSHA1_R3(v,w,x,y,z,i) z+=(((w|x)&y)|(w&x)) + kSHA1_BLK(i)  + kSha1Hash_sha1k[2] + kSHA1_ROL32(v,5); w = kSHA1_ROL32(w,30);
#define kSHA1_R4(v,w,x,y,z,i) z+=(w^x^y)           + kSHA1_BLK(i)  + kSha1Hash_sha1k[3] + kSHA1_ROL32(v,5); w = kSHA1_ROL32(w,30);

kFx(kStatus) kSha1Hash_Construct(kSha1Hash* hash, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback(allocator);
    kStatus status;

    kCheck(kAlloc_GetObject(alloc, kTypeOf(kSha1Hash), hash));

    if (!kSuccess(status = xkSha1Hash_Init(*hash, kTypeOf(kSha1Hash), alloc)))
    {
        kAlloc_FreeRef(alloc, hash);
    }

    return status;
}

kFx(kStatus) xkSha1Hash_Init(kSha1Hash hash, kType type, kAlloc alloc)
{
    kObjR(kSha1Hash, hash);
    kStatus status = kOK;  

    kCheck(xkHash_Init(hash, type, alloc)); 

    kTry
    {
        kTest(kHash_Clear(hash));
    }
    kCatch(&status)
    {
        xkSha1Hash_VRelease(hash);
        kEndCatch(status);
    }
    
    return kOK;
}

kFx(kStatus) xkSha1Hash_VRelease(kSha1Hash hash)
{
    return xkHash_VRelease(hash);
}

kFx(kStatus) xkSha1Hash_VUpdate(kSha1Hash hash, const void* buffer, kSize size)
{
    kObj(kSha1Hash, hash);
    
    return xkSha1Hash_UpdateBuffer(obj->count, obj->state, obj->buffer, buffer, size);
}

kFx(kStatus) xkSha1Hash_VDigest(kSha1Hash hash, void* buffer, kSize size)
{
    kObj(kSha1Hash, hash);
    kByte finalCount[8];
    kSize i;
    k32u state[xkSHA1_HASH_STATE_SIZE];
    kSize count[xkSHA1_HASH_COUNT_SIZE];
    kByte hashBuffer[xkSHA1_HASH_BUFFER_SIZE];
    kByte* bufferWriter = (kByte*) buffer; 
    
    if (size < kHash_DigestSize(hash))
    {
        return kERROR_INCOMPLETE; 
    }

    /* Copy current state to local buffers so kHash_Update still can be called afterwards. */
    kCheck(kMemCopy(state, obj->state, sizeof(state)));
    kCheck(kMemCopy(count, obj->count, sizeof(count)));
    kCheck(kMemCopy(hashBuffer, obj->buffer, xkSHA1_HASH_BUFFER_SIZE));

    /* Take care of endianess */
    for (i = 0; i < kCountOf(finalCount); i++) 
    {
        finalCount[i] = (k8u)((count[(i >= 4 ? 0 : 1)] >> ((3 - (i & 3)) * 8)) & 255); 
    }
    
    /* Add padding */
    kCheck(xkSha1Hash_UpdateBuffer(count, state, hashBuffer, "\200", 1));
    
    while ((count[0] & 504) != 448) 
    {
        kCheck(xkSha1Hash_UpdateBuffer(count, state, hashBuffer, "\0", 1));
    }
    
    /* Could cause a call of xkSha1Hash_Transform */
    kCheck(xkSha1Hash_UpdateBuffer(count, state, hashBuffer, finalCount, kCountOf(finalCount)));
    
    /* Take care of endianess */
    for (i = 0; i < xkSHA1_HASH_DIGEST_SIZE; i++) 
    {
        bufferWriter[i] = (kByte)((state[i >> 2] >> ((3 - (i & 3)) * 8)) & 255);
    }

    return kOK;
}

kFx(kSize) xkSha1Hash_VDigestSize(kSha1Hash hash)
{
    return xkSHA1_HASH_DIGEST_SIZE;
}

kFx(kStatus) xkSha1Hash_VClear(kSha1Hash hash)
{
    kObj(kSha1Hash, hash);
    
    kCheck(kMemCopy(obj->state, kSha1Hash_sha1state, sizeof(obj->state)));
    kCheck(kMemSet(obj->count, 0, sizeof(obj->count)));
    kCheck(kMemSet(obj->buffer, 0, xkSHA1_HASH_BUFFER_SIZE));

    return kOK;
}

kFx(void) xkSha1Hash_Transform(k32u* state5, const kByte *buffer64)
{
    k32u* block = (k32u*)buffer64;

    k32u a = state5[0];
    k32u b = state5[1];
    k32u c = state5[2];
    k32u d = state5[3];
    k32u e = state5[4];
    
    kSHA1_R0(a, b, c, d, e, 0);  kSHA1_R0(e, a, b, c, d, 1);  kSHA1_R0(d, e, a, b, c, 2);  kSHA1_R0(c, d, e, a, b, 3);
    kSHA1_R0(b, c, d, e, a, 4);  kSHA1_R0(a, b, c, d, e, 5);  kSHA1_R0(e, a, b, c, d, 6);  kSHA1_R0(d, e, a, b, c, 7);
    kSHA1_R0(c, d, e, a, b, 8);  kSHA1_R0(b, c, d, e, a, 9);  kSHA1_R0(a, b, c, d, e, 10); kSHA1_R0(e, a, b, c, d, 11);
    kSHA1_R0(d, e, a, b, c, 12); kSHA1_R0(c, d, e, a, b, 13); kSHA1_R0(b, c, d, e, a, 14); kSHA1_R0(a, b, c, d, e, 15);
    kSHA1_R1(e, a, b, c, d, 16); kSHA1_R1(d, e, a, b, c, 17); kSHA1_R1(c, d, e, a, b, 18); kSHA1_R1(b, c, d, e, a, 19);
    kSHA1_R2(a, b, c, d, e, 20); kSHA1_R2(e, a, b, c, d, 21); kSHA1_R2(d, e, a, b, c, 22); kSHA1_R2(c, d, e, a, b, 23);
    kSHA1_R2(b, c, d, e, a, 24); kSHA1_R2(a, b, c, d, e, 25); kSHA1_R2(e, a, b, c, d, 26); kSHA1_R2(d, e, a, b, c, 27);
    kSHA1_R2(c, d, e, a, b, 28); kSHA1_R2(b, c, d, e, a, 29); kSHA1_R2(a, b, c, d, e, 30); kSHA1_R2(e, a, b, c, d, 31);
    kSHA1_R2(d, e, a, b, c, 32); kSHA1_R2(c, d, e, a, b, 33); kSHA1_R2(b, c, d, e, a, 34); kSHA1_R2(a, b, c, d, e, 35);
    kSHA1_R2(e, a, b, c, d, 36); kSHA1_R2(d, e, a, b, c, 37); kSHA1_R2(c, d, e, a, b, 38); kSHA1_R2(b, c, d, e, a, 39);
    kSHA1_R3(a, b, c, d, e, 40); kSHA1_R3(e, a, b, c, d, 41); kSHA1_R3(d, e, a, b, c, 42); kSHA1_R3(c, d, e, a, b, 43);
    kSHA1_R3(b, c, d, e, a, 44); kSHA1_R3(a, b, c, d, e, 45); kSHA1_R3(e, a, b, c, d, 46); kSHA1_R3(d, e, a, b, c, 47);
    kSHA1_R3(c, d, e, a, b, 48); kSHA1_R3(b, c, d, e, a, 49); kSHA1_R3(a, b, c, d, e, 50); kSHA1_R3(e, a, b, c, d, 51);
    kSHA1_R3(d, e, a, b, c, 52); kSHA1_R3(c, d, e, a, b, 53); kSHA1_R3(b, c, d, e, a, 54); kSHA1_R3(a, b, c, d, e, 55);
    kSHA1_R3(e, a, b, c, d, 56); kSHA1_R3(d, e, a, b, c, 57); kSHA1_R3(c, d, e, a, b, 58); kSHA1_R3(b, c, d, e, a, 59);
    kSHA1_R4(a, b, c, d, e, 60); kSHA1_R4(e, a, b, c, d, 61); kSHA1_R4(d, e, a, b, c, 62); kSHA1_R4(c, d, e, a, b, 63);
    kSHA1_R4(b, c, d, e, a, 64); kSHA1_R4(a, b, c, d, e, 65); kSHA1_R4(e, a, b, c, d, 66); kSHA1_R4(d, e, a, b, c, 67);
    kSHA1_R4(c, d, e, a, b, 68); kSHA1_R4(b, c, d, e, a, 69); kSHA1_R4(a, b, c, d, e, 70); kSHA1_R4(e, a, b, c, d, 71);
    kSHA1_R4(d, e, a, b, c, 72); kSHA1_R4(c, d, e, a, b, 73); kSHA1_R4(b, c, d, e, a, 74); kSHA1_R4(a, b, c, d, e, 75);
    kSHA1_R4(e, a, b, c, d, 76); kSHA1_R4(d, e, a, b, c, 77); kSHA1_R4(c, d, e, a, b, 78); kSHA1_R4(b, c, d, e, a, 79);

    state5[0] += a;
    state5[1] += b;
    state5[2] += c;
    state5[3] += d;
    state5[4] += e;
}

kFx(kStatus) xkSha1Hash_UpdateBuffer(kSize* count2, k32u* state5, kByte* bufferToUpdate64, const void* buffer, kSize size)
{
    const kByte* data = (const kByte*) buffer;
    kSize i, j;

    j = (count2[0] >> 3) & 63;

    if ((count2[0] += size << 3) < (size << 3))
    {
        count2[1]++;
    }

    count2[1] += (size >> 29);

    if ((j + size) > 63)
    {
        kCheck(kMemCopy(&bufferToUpdate64[j], data, (64 - j)));

        xkSha1Hash_Transform(state5, bufferToUpdate64);

        for (i = 64 - j; i + 63 < size; i += 64)
        {
            xkSha1Hash_Transform(state5, data + i);
        }

        j = 0;
    }
    else
    {
        i = 0;
    }

    kCheck(kMemCopy(&bufferToUpdate64[j], &data[i], size - i));

    return kOK;
}

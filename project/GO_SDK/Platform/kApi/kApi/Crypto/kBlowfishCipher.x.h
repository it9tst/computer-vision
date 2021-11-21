/** 
 * @file    kBlowfishCipher.x.h
 *
 * @internal
 * Copyright (C) 2016-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_BLOWFISH_CIPHER_X_H
#define K_API_BLOWFISH_CIPHER_X_H

// Blowfish block size
#define xkBLOWFISH_CIPHER_BLOCK_SIZE                  (8)

typedef struct kBlowfishCipherClass
{
    kCipherClass base;

    k32u P[18];         // P-array
    k32u S0[256];       // S-box 0
    k32u S1[256];       // S-box 1
    k32u S2[256];       // S-box 2
    k32u S3[256];       // S-box 3

} kBlowfishCipherClass;

kDeclareClassEx(k, kBlowfishCipher, kCipher)

/* 
* Private methods. 
*/

kFx(kStatus) xkBlowfishCipher_Init(kBlowfishCipher blowfish, kType type, const kByte* key, kSize length, kCipherPadding padding, kCipherMode cipherMode, kAlloc alloc); 

kInlineFx(kStatus) xkBlowfishCipher_VRelease(kBlowfishCipher cipher)
{
    return xkCipher_VRelease(cipher);
}

kFx(kStatus) xkBlowfishCipher_VEncrypt(kBlowfishCipher blowfish, const void* data, kSize dataLength, kArray1 result);
kFx(kStatus) xkBlowfishCipher_VDecrypt(kBlowfishCipher blowfish, const void* data, kSize dataLength, kArray1 result);

kInlineFx(kSize) xkBlowfishCipher_VBlocksize(kBlowfishCipher blowfish)
{
    return xkBLOWFISH_CIPHER_BLOCK_SIZE;
}

/**
* [Private] Encrypts a 64-bit message block.
*
* @public                   @memberof kBlowfish
* @param   blowfish         kBlowfish object.
* @param   left             Left partition (32-bits).
* @param   right            Right partition (32-bits).
* @param   leftOut          Encrypted left partition.
* @param   rightOut         Encrypted right partition.
* @return                   Operation status.
*/
kFx(kStatus) xkBlowfishCipher_EncryptBlock(kBlowfishCipher blowfish, const k32u* left, const k32u* right, k32u* leftOut, k32u* rightOut);

/**
* [Private] Decrypts a 64-bit message block.
*
* @public                   @memberof kBlowfish
* @param   blowfish         kBlowfish object.
* @param   left             Left partition (32-bits).
* @param   right            Right partition (32-bits).
* @param   leftOut          Decrypted left partition.
* @param   rightOut         Decrypted right partition.
* @return                   Operation status.
*/
kFx(kStatus) xkBlowfishCipher_DecryptBlock(kBlowfishCipher blowfish, const k32u* left, const k32u* right, k32u* leftOut, k32u* rightOut);

kInlineFx(k32u) xkBlowfishCipher_Fprime(kBlowfishCipher blowfish, k32u a, k32u b, k32u c, k32u d)
{
    kObj(kBlowfishCipher, blowfish);

    return ((obj->S0[a] + obj->S1[b]) ^ obj->S2[c]) + obj->S3[d]; 
}

kInlineFx(k32u) xkBlowfishCipher_F(kBlowfishCipher blowfish, k32u x)
{
    kObj(kBlowfishCipher, blowfish);

    return xkBlowfishCipher_Fprime(blowfish, (x >> 24) & 0xFF, (x >> 16) & 0xFF, (x >> 8) & 0xFF, x & 0xFF);  
}

kInlineFx(void) xkBlowfishCipher_Round(kBlowfishCipher blowfish, k32u n, k32u* xL, k32u* xR)
{
    kObj(kBlowfishCipher, blowfish);
    k32u t; 

    *xL ^= obj->P[n]; 
    t = *xL; 
    *xL = xkBlowfishCipher_F(blowfish, *xL) ^ *xR; 
    *xR = t; 
}

/* 
* Deprecated (Stage 1): not recommended for further use, but not yet announced via kDeprecate
*/

//[Deprecated] Was never a public method. Use public kCipher interface.
#define kBlowfishCipher_EncryptBlock xkBlowfishCipher_EncryptBlock 

//[Deprecated] Was never a public method. Use public kCipher interface.
#define kBlowfishCipher_DecryptBlock xkBlowfishCipher_DecryptBlock 


#endif

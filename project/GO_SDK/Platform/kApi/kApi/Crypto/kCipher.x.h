/** 
 * @file    kCipher.x.h
 *
 * @internal
 * Copyright (C) 2016-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_CIPHER_X_H
#define K_API_CIPHER_X_H

kDeclareEnumEx(k, kCipherPadding, kValue)
kDeclareEnumEx(k, kCipherMode, kValue)

typedef struct kCipherClass
{
    kObjectClass base; 

    kCipherPadding padding;
    kCipherMode  mode;
} kCipherClass;

typedef struct kCipherVTable
{
    kObjectVTable base; 

    kStatus (kCall* VEncrypt)(kCipher cipher, const void* data, kSize dataLength, kArray1 result);
    kStatus (kCall* VDecrypt)(kCipher cipher, const void* data, kSize dataLength, kArray1 result);
    kSize (kCall* VBlocksize)(kCipher cipher);
} kCipherVTable; 

kDeclareVirtualClassEx(k, kCipher, kObject)

/* 
* Private methods. 
*/

kFx(kStatus) xkCipher_Init(kCipher cipher, kType type, kCipherPadding padding, kCipherMode mode, kAlloc allocator);

kInlineFx(kStatus) xkCipher_VRelease(kCipher cipher)
{
    return kObject_VRelease(cipher);
}

kInlineFx(kStatus) xkCipher_VEncrypt(kCipher cipher, const void* data, kSize dataLength, kArray1 result)
{
    return kERROR_UNIMPLEMENTED; 
}

kInlineFx(kStatus) xkCipher_VDecrypt(kCipher cipher, const void* data, kSize dataLength, kArray1 result)
{
    return kERROR_UNIMPLEMENTED; 
}

kInlineFx(kSize) xkCipher_VBlocksize(kCipher cipher)
{
    return 0;
}

kFx(kSize) xkCipher_MinimalPaddingSize(kCipher cipher, kSize paddingSize);
kFx(kStatus) xkCipher_AddPadding(kCipher blowfish, kArray1 encryptArray, kSize paddingByteCount);


#endif

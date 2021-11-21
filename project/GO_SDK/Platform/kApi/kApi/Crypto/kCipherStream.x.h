/** 
 * @file    kCipherStream.x.h
 *
 * @internal
 * Copyright (C) 2016-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_CIPHERSTREAM_X_H
#define K_API_CIPHERSTREAM_X_H

#include <kApi/Io/kStream.h>

typedef struct kCipherStreamClass
{
    kStreamClass base;

    kByte* writeBuffer;
    kByte* readBuffer;
    kSize bufferSize;
    kSize writeBufferUsed;
    kSize readBufferUsed;

    kArray1 encryptArray;
    kArray1 decryptArray;

    kStream stream;
    kCipher cipher;
} kCipherStreamClass; 

kDeclareClassEx(k, kCipherStream, kStream)
        
/* 
* Private methods. 
*/

kFx(kStatus) xkCipherStream_Init(kCipherStream cipherStream, kType type, kStream stream, kCipher cipher, kAlloc alloc);
kFx(kStatus) xkCipherStream_VRelease(kCipherStream cipherStream);

kFx(kStatus) xkCipherStream_VReadSomeImpl(kCipherStream cipherStream, void* buffer, kSize minCount, kSize maxCount, kSize* bytesRead);
kFx(kStatus) xkCipherStream_VWriteImpl(kCipherStream cipherStream, const void* buffer, kSize size);
kFx(kStatus) xkCipherStream_VFlush(kCipherStream cipherStream);


#endif

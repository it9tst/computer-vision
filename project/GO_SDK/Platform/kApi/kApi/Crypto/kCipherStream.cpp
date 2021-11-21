/** 
 * @file    kCipherStream.cpp
 *
 * @internal
 * Copyright (C) 2016-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */

#include <kApi/Crypto/kCipherStream.h>
#include <kApi/Crypto/kCipher.h>
#include <kApi/Data/kArray1.h>

kBeginClassEx(k, kCipherStream)
    kAddPrivateVMethod(kCipherStream, kObject, VRelease)
    kAddPrivateVMethod(kCipherStream, kStream, VReadSomeImpl)
    kAddPrivateVMethod(kCipherStream, kStream, VWriteImpl)
    kAddPrivateVMethod(kCipherStream, kStream, VFlush)
kEndClassEx()

kFx(kStatus) kCipherStream_Construct(kCipherStream *cipherStream, kStream stream, kCipher cipher, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback(allocator);    
    kStatus status; 

    kCheck(kAlloc_GetObject(alloc, kTypeOf(kCipherStream), cipherStream));

    if (!kSuccess(status = xkCipherStream_Init(*cipherStream, kTypeOf(kCipherStream), stream, cipher, alloc)))
    {
        kAlloc_FreeRef(alloc, cipherStream); 
    }

    return status; 
} 

kFx(kStatus) xkCipherStream_Init(kCipherStream cipherStream, kType type, kStream stream, kCipher cipher, kAlloc alloc)
{
    kObjR(kCipherStream, cipherStream);
    kStatus status = kOK;  

    kCheck(kStream_Init(cipherStream, type, alloc));
    
    obj->writeBuffer = kNULL;
    obj->readBuffer = kNULL;
    obj->bufferSize = kCipher_Blocksize(cipher);
    obj->writeBufferUsed = 0;
    obj->readBufferUsed = 0;
    obj->encryptArray = kNULL;
    obj->decryptArray = kNULL;
    obj->stream = stream;
    obj->cipher = cipher;

    kTry
    {
        kTest(kObject_GetMem(cipherStream, obj->bufferSize, &obj->writeBuffer));
        kTest(kObject_GetMem(cipherStream, obj->bufferSize, &obj->readBuffer));

        kTest(kArray1_Construct(&obj->encryptArray, kTypeOf(kByte), 0, kObject_Alloc(cipherStream)));
        kTest(kArray1_Construct(&obj->decryptArray, kTypeOf(kByte), 0, kObject_Alloc(cipherStream)));
    }
    kCatch(&status)
    {
        xkCipherStream_VRelease(cipherStream);
        kEndCatch(status);
    }

    return kOK; 
}

kFx(kStatus) xkCipherStream_VRelease(kCipherStream cipherStream)
{
    kObj(kCipherStream, cipherStream);

    kCheck(kCipherStream_FlushFinal(cipherStream));
    
    kDestroyRef(&obj->encryptArray);
    kDestroyRef(&obj->decryptArray);

    kCheck(kObject_FreeMemRef(cipherStream, &obj->writeBuffer));
    kCheck(kObject_FreeMemRef(cipherStream, &obj->readBuffer));

    kCheck(kObject_VRelease(cipherStream));

    return kOK;
}

kFx(kStatus) xkCipherStream_VFlush(kCipherStream cipherStream)
{
    kObj(kCipherStream, cipherStream);

    return kStream_Flush(obj->stream);
}

kFx(kStatus) kCipherStream_FlushFinal(kCipherStream cipherStream)
{
    kObj(kCipherStream, cipherStream);

    kTry
    {
        if (obj->writeBufferUsed != 0)
        {
            kTest(kCipher_Encrypt(obj->cipher, obj->writeBuffer, obj->writeBufferUsed, obj->encryptArray));
            kTest(kStream_Write(obj->stream, kArray1_Data(obj->encryptArray), kArray1_Length(obj->encryptArray)));
        }

        kTest(kStream_Flush(obj->stream));
    }
    kFinally
    {
        kEndFinally();
    }

    return kOK;
}

kFx(kStatus) xkCipherStream_VReadSomeImpl(kCipherStream cipherStream, void* buffer, kSize minCount, kSize maxCount, kSize* bytesRead)
{
    kObj(kCipherStream, cipherStream);
    kSize i;
    kByte* writer = (kByte*) buffer; 

    kTry
    {
        for (i = 0; i < minCount; i++)
        {
            if (obj->readBufferUsed == 0)
            {
                kTest(kStream_Read(obj->stream, obj->readBuffer, obj->bufferSize));
                kTest(kCipher_Decrypt(obj->cipher, obj->readBuffer, obj->bufferSize, obj->decryptArray));

                kMemCopy(obj->readBuffer, kArray1_Data(obj->decryptArray), obj->bufferSize);
                obj->readBufferUsed = obj->bufferSize;
            }

            writer[i] = obj->readBuffer[obj->bufferSize - obj->readBufferUsed--];
        }
    }
    kFinally
    {
        kEndFinally();
    }

    if (!kIsNull(bytesRead))
    {
        *bytesRead = minCount;
    }

    return kOK;
}

kFx(kStatus) xkCipherStream_VWriteImpl(kCipherStream cipherStream, const void* buffer, kSize size)
{
    kObj(kCipherStream, cipherStream);
    const kByte* reader = (const kByte*) buffer; 
    kSize i;

    kTry
    {
        for (i = 0; i < size; i++)
        {
            obj->writeBuffer[obj->writeBufferUsed++] = reader[i];
        
            if (obj->writeBufferUsed == obj->bufferSize)
            {
                kTest(kCipher_Encrypt(obj->cipher, obj->writeBuffer, obj->bufferSize, obj->encryptArray));
                kTest(kStream_Write(obj->stream, kArray1_Data(obj->encryptArray), kArray1_Length(obj->encryptArray)));

                obj->writeBufferUsed = 0;
            }
        }
    }
    kFinally
    {
        kEndFinally();
    }

    return kOK;
}

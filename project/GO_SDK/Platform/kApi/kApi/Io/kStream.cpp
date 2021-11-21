/** 
 * @file    kStream.cpp
 *
 * @internal
 * Copyright (C) 2005-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <kApi/Io/kStream.h>
#include <kApi/Data/kArray1.h>

kBeginVirtualClassEx(k, kStream)

    kAddFlags(kStream, kTYPE_FLAGS_ABSTRACT)

    kAddVMethod(kStream, kStream, VReadSomeImpl)
    kAddVMethod(kStream, kStream, VReadImpl)     
    kAddVMethod(kStream, kStream, VWriteImpl)
    kAddVMethod(kStream, kStream, VSeek)
    kAddVMethod(kStream, kStream, VFlush)
    kAddVMethod(kStream, kStream, VFill)

kEndVirtualClassEx()

kFx(kStatus) kStream_CopyAll(kStream stream, kStream source)
{
    kByte buffer[256]; 
    kSize bytesRead = 0; 

    while (kSuccess(kStream_ReadSome(source, buffer, 1, kCountOf(buffer), &bytesRead)))
    {
        kCheck(kStream_Write(stream, buffer, bytesRead)); 
    }
  
    return kOK; 
}

kFx(kStatus) kStream_CopyEx(kStream stream, kStream source, kSize size, kCallbackFx progress, kPointer context)
{
    const kSize bufferSize = 64*1024;
    kArray2 buffer = kNULL;
    kSize bytesCopied = 0; 
    k64u updateTime = k64U_NULL; 

    kTry
    {
        kTest(kArray1_Construct(&buffer, kTypeOf(kByte), bufferSize, kObject_Alloc(stream)));

        while (bytesCopied < size)
        {
            kSize bytesRemaining = size - bytesCopied; 
            kSize copySize = kMin_(bytesRemaining, bufferSize); 

            kTest(kStream_Read(source, kArray1_DataT(buffer, kByte), copySize)); 
            kTest(kStream_Write(stream, kArray1_DataT(buffer, kByte), copySize)); 

            bytesCopied += copySize; 

            xkUpdateProgress(progress, context, kNULL, &updateTime, (k32u)(100*bytesCopied/size)); 
        }

        xkUpdateProgress(progress, context, kNULL, kNULL, 100); 
  
    }
    kFinally
    {
        kDestroyRef(&buffer);
        kEndFinally();
    }

    return kOK; 
}

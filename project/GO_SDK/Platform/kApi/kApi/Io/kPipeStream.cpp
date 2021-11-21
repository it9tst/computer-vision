/** 
 * @file    kPipeStream.cpp
 *
 * @internal
 * Copyright (C) 2018-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */

#define K_PLATFORM
#include <kApi/Io/kPipeStream.h>

#include <kApi/Utils/kProcess.h>

#if defined(K_LINUX)
#   include <unistd.h>
#endif

kBeginClassEx(k, kPipeStream)
    kAddVMethod(kPipeStream, kObject, VRelease)
    kAddVMethod(kPipeStream, kStream, VReadSomeImpl)
    kAddVMethod(kPipeStream, kStream, VWriteImpl)
kEndClassEx()

kFx(kStatus) kPipeStream_VRelease(kPipeStream pipe)
{
    kObj(kPipeStream, pipe);
    
    return kObject_VRelease(pipe);
}

#if defined(K_WINDOWS) || defined(K_LINUX)

kFx(kStatus) kPipeStream_ConstructFromHandle(kPipeStream* pipe, kPipeStreamHandle handle, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback(allocator);
    kStatus status = kOK;

    kCheck(kAlloc_GetObject(alloc, kTypeOf(kPipeStream), pipe));

    if (!kSuccess(status = kPipeStream_Init(*pipe, kTypeOf(kPipeStream), handle, alloc)))
    {
        kAlloc_FreeRef(alloc, pipe);
    }

    return status;
}

kFx(kStatus) kPipeStream_Init(kPipeStream pipe, kType type, kPipeStreamHandle handle, kAlloc alloc)
{
    kObjR(kPipeStream, pipe);

    kCheck(kStream_Init(pipe, type, alloc));
 
    obj->handle = handle;

    return kOK;
}

#endif

#if defined(K_WINDOWS)

kFx(kStatus) kPipeStream_VReadSomeImpl(kPipeStream pipe, void* buffer, kSize minCount, kSize maxCount, kSize* bytesRead)
{
    kObj(kPipeStream, pipe);
    DWORD readCount = 0;
    DWORD totalReadCount = 0;

    while (totalReadCount < minCount)
    {
        void* pos = (kChar*)buffer + totalReadCount;

        if ((!ReadFile(obj->handle, pos, (DWORD)(maxCount - totalReadCount), &readCount, NULL)) || (readCount == 0))
        {
            return kERROR_OS;
        }

        totalReadCount += readCount;
    }

    if (!kIsNull(bytesRead))
    {
        *bytesRead = totalReadCount;
    }

    return kOK;
}

kFx(kStatus) kPipeStream_VWriteImpl(kPipeStream pipe, const void* buffer, kSize size)
{
    kObj(kPipeStream, pipe);
    DWORD byteCount = 0;
    DWORD totalByteCount = 0;

    while (totalByteCount < size)
    {
        void* pos = (kChar*)buffer + totalByteCount;

        if (!WriteFile(obj->handle, pos, (DWORD)(size - totalByteCount), &byteCount, NULL))
        {
            return kERROR_OS;
        }

        totalByteCount += byteCount;
    }
    
    return kOK;
}

#elif defined(K_LINUX)

kFx(kStatus) kPipeStream_VReadSomeImpl(kPipeStream pipe, void* buffer, kSize minCount, kSize maxCount, kSize* bytesRead)
{
    kObj(kPipeStream, pipe);
    k32s readCount = 0; 
    kSize totalReadCount = 0;

    while (totalReadCount < minCount)
    {
        void* pos = (kChar*)buffer + totalReadCount;

        if ((readCount = read(obj->handle, pos, maxCount - totalReadCount)) <= 0)
        {
            return kERROR_OS;
        }

        totalReadCount += (kSize) readCount;
    }

    if (!kIsNull(bytesRead))
    {
        *bytesRead = totalReadCount;
    }

    return kOK;
}

kFx(kStatus) kPipeStream_VWriteImpl(kPipeStream pipe, const void* buffer, kSize size)
{
    kObj(kPipeStream, pipe);
    k32s byteCount = 0;
    kSize totalByteCount = 0;

    while (totalByteCount < size)
    {
        void* pos = (kChar*)buffer + totalByteCount;

        if ((byteCount = write(obj->handle, pos, size - totalByteCount)) < 0)
        {
            return kERROR_OS;
        }

        totalByteCount += (kSize) byteCount;
    }

    return kOK;
}

#else

kFx(kStatus) kPipeStream_Init(kPipeStream pipe, kType type, kPipeStreamHandle handle, kAlloc alloc)
{
    return kERROR_UNIMPLEMENTED;
}

kFx(kStatus) kPipeStream_VReadSomeImpl(kPipeStream pipe, void* buffer, kSize minCount, kSize maxCount, kSize* bytesRead)
{
    kObj(kPipeStream, pipe);
    
    return kERROR_UNIMPLEMENTED;
}

kFx(kStatus) kPipeStream_VWriteImpl(kPipeStream pipe, const void* buffer, kSize size)
{
    kObj(kPipeStream, pipe);
    
    return kERROR_UNIMPLEMENTED;
}

#endif 

kFx(kStream) kStdIn()
{
    return kProcess_AppStdIn();
}

kFx(kStream) kStdOut()
{
    return kProcess_AppStdOut();
}

kFx(kStream) kStdErr()
{
    return kProcess_AppStdErr();
}

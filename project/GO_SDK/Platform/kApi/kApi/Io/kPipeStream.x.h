/** 
 * @file    kPipeStream.x.h
 *
 * @internal
 * Copyright (C) 2018-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */

#ifndef K_API_PIPE_STREAM_X_H
#define K_API_PIPE_STREAM_X_H

#include <kApi/Io/kStream.h>

#if defined(K_PLATFORM) 

#if defined(K_WINDOWS)

#   define kPipeStreamHandle HANDLE

#else

#   define kPipeStreamHandle k32s

#endif

typedef struct kPipeStreamClass
{
    kStreamClass base;
    kPipeStreamHandle handle;

} kPipeStreamClass;

kFx(kStatus) kPipeStream_ConstructFromHandle(kPipeStream* pipe, kPipeStreamHandle handle, kAlloc allocator);

kFx(kStatus) kPipeStream_Init(kPipeStream pipe, kType type, kPipeStreamHandle handle, kAlloc alloc);

#endif

kFx(kStatus) kPipeStream_VRelease(kPipeStream pipe);

kFx(kStatus) kPipeStream_VReadSomeImpl(kPipeStream pipe, void* buffer, kSize minCount, kSize maxCount, kSize* bytesRead);
kFx(kStatus) kPipeStream_VWriteImpl(kPipeStream pipe, const void* buffer, kSize size);

kDeclareClassEx(k, kPipeStream, kStream)

#endif

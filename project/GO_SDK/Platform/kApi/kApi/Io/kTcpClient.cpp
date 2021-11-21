/** 
 * @file    kTcpClient.cpp
 *
 * @internal
 * Copyright (C) 2008-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#define K_PLATFORM
#include <kApi/Io/kTcpClient.h>
#include <kApi/Io/kSocket.h>
#include <kApi/Threads/kTimer.h>

kBeginClassEx(k, kTcpClient)
    kAddPrivateVMethod(kTcpClient, kObject, VRelease)
    kAddPrivateVMethod(kTcpClient, kStream, VReadSomeImpl)
    kAddPrivateVMethod(kTcpClient, kStream, VWriteImpl)
    kAddPrivateVMethod(kTcpClient, kStream, VFlush)
    kAddPrivateVMethod(kTcpClient, kStream, VFill)
    kAddPrivateVMethod(kTcpClient, kStream, VSeek)
kEndClassEx()

kFx(kStatus) kTcpClient_Construct(kTcpClient* client, kIpVersion ipVersion, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback(allocator);
    kStatus status; 

    kCheck(kAlloc_GetObject(alloc, kTypeOf(kTcpClient), client)); 

    if (!kSuccess(status = xkTcpClient_Init(*client, kTypeOf(kTcpClient), ipVersion, alloc)))
    {
        kAlloc_FreeRef(alloc, client); 
    }

    return status; 
} 

kFx(kStatus) xkTcpClient_ConstructFromSocket(kTcpClient* client, kSocket socket, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback(allocator);
    kType type = kTypeOf(kTcpClient); 
    kStatus status; 

    kCheck(kAlloc_GetObject(alloc, type, client)); 

    if (!kSuccess(status = xkTcpClient_InitFromSocket(*client, type, socket, alloc)))
    {
        kAlloc_FreeRef(alloc, client); 
    }

    return status; 
} 

kFx(kStatus) xkTcpClient_Init(kTcpClient client, kType type, kIpVersion ipVersion, kAlloc allocator)
{
    kSocket socket = kNULL; 
    kStatus status;     

    kTry
    {
        kTest(kSocket_Construct(&socket, kIP_VERSION_4, kSOCKET_TYPE_TCP, allocator));
        kTest(kSocket_Bind(socket, kIpAddress_Any(ipVersion), kIP_PORT_ANY));

        kTest(xkTcpClient_InitFromSocket(client, type, socket, allocator));
    }
    kCatch(&status)
    {
        kDestroyRef(&socket); 
        kEndCatch(status); 
    }
    
    return kOK; 
}

kFx(kStatus) xkTcpClient_InitFromSocket(kTcpClient client, kType type, kSocket socket, kAlloc allocator)
{
    kObjR(kTcpClient, client); 
    kStatus status; 

    kCheckArgs(!kIsNull(socket)); 
    
    kCheck(kStream_Init(client, type, allocator)); 
    
    obj->socket = kNULL;
    obj->writeTimeout = 0;
    obj->readTimeout = 0;
    obj->socketReadBufferSize = 0;
    obj->cancelQuery.function = kNULL;
    obj->cancelQuery.receiver = kNULL;
    obj->isCancelled = 0;
    obj->timedOut = kFALSE;
    obj->isSeekEnabled = kFALSE;

    kTry
    {
        kTest(kSocket_SetBlocking(socket, kFALSE)); 

        obj->socket = socket;
        obj->socketReadBufferSize = xkSocket_ReadBuffer(socket); 
    }
    kCatch(&status)
    {
        xkTcpClient_VRelease(client); 
        kEndCatch(status); 
    }
  
    return kOK; 
}

kFx(kStatus) xkTcpClient_VRelease(kTcpClient client)
{
    kObj(kTcpClient, client); 
        
    kCheck(kObject_Destroy(obj->socket));    
    
    kCheck(kObject_FreeMem(client, obj->base.readBuffer));   
    kCheck(kObject_FreeMem(client, obj->base.writeBuffer));

    kCheck(kStream_VRelease(client)); 
        
    return kOK;
}

kFx(kStatus) kTcpClient_SetWriteBuffers(kTcpClient client, kSSize socketSize, kSSize clientSize)
{
    kObj(kTcpClient, client); 
    
    kCheck(kStream_Flush(client)); 
    
    if (socketSize >= 0)
    {
        kCheck(kSocket_SetWriteBuffer(obj->socket, (kSize)socketSize));
    }

    if (clientSize >= 0)
    {
        kCheck(kObject_FreeMemRef(client, &obj->base.writeBuffer)); 
        
        obj->base.writeCapacity = 0; 
        obj->base.writeBegin = 0; 
        obj->base.writeEnd = 0; 

        if (clientSize > 0)
        {
            kCheck(kObject_GetMem(client, (kSize)clientSize, &obj->base.writeBuffer)); 
            obj->base.writeCapacity = (kSize)clientSize; 
            obj->base.writeEnd = (kSize)clientSize; 
        }
    }
    
    return kOK;
}

kFx(kStatus) kTcpClient_SetReadBuffers(kTcpClient client, kSSize socketSize, kSSize clientSize)
{
    kObj(kTcpClient, client); 

    kCheckState((obj->base.readEnd - obj->base.readBegin) == 0);

    if (socketSize >= 0)
    {
        kCheck(kSocket_SetReadBuffer(obj->socket, (kSize)socketSize));
        obj->socketReadBufferSize = xkSocket_ReadBuffer(obj->socket); 
    }

    if (clientSize >= 0)
    {
        kCheck(kObject_FreeMemRef(client, &obj->base.readBuffer)); 

        obj->base.readCapacity = 0; 
        obj->base.readBegin = 0; 
        obj->base.readEnd = 0; 

        if (clientSize > 0)
        {
            kCheck(kObject_GetMem(client, (kSize)clientSize, &obj->base.readBuffer));                 
            obj->base.readCapacity = (kSize)clientSize; 
        }
    }
    
    return kOK;
}

kFx(kStatus) kTcpClient_SetWriteTimeout(kTcpClient client, k64u timeout)
{
    kObj(kTcpClient, client); 

    obj->writeTimeout = timeout; 

    return kOK; 
}

kFx(kStatus) kTcpClient_SetReadTimeout(kTcpClient client, k64u timeout)
{
    kObj(kTcpClient, client); 

    obj->readTimeout = timeout; 

    return kOK; 
}

kFx(kStatus) kTcpClient_SetNoDelay(kTcpClient client, kBool noDelay)
{
    kObj(kTcpClient, client); 

    return kSocket_SetNoDelay(obj->socket, noDelay); 
}

kFx(kStatus) kTcpClient_EnableSeek(kTcpClient client, kBool enabled)
{
    kObj(kTcpClient, client); 

    obj->isSeekEnabled = enabled; 

    return kOK; 
}

kFx(kStatus) kTcpClient_SetCancelHandler(kTcpClient client, kCallbackFx function, kPointer receiver)
{
    kObj(kTcpClient, client); 
        
    obj->cancelQuery.function = function; 
    obj->cancelQuery.receiver = receiver; 

    return kOK; 
}

kFx(kStatus) kTcpClient_Cancel(kTcpClient client)
{
    kObj(kTcpClient, client); 
        
    kAtomic32s_Exchange(&obj->isCancelled, kTRUE);

    return kOK; 
}

kFx(kStatus) kTcpClient_Connect(kTcpClient client, kIpAddress address, k32u port, k64u timeout)
{
    kObj(kTcpClient, client); 
   
    return kSocket_Connect(obj->socket, address, port, timeout);
}

kFx(kStatus) kTcpClient_BeginConnect(kTcpClient client, kIpAddress address, k32u port)
{
    kObj(kTcpClient, client); 
   
    return kSocket_BeginConnect(obj->socket, address, port);
}

kFx(kStatus) kTcpClient_EndConnect(kTcpClient client, k64u timeout)
{
    kObj(kTcpClient, client); 
   
    return kSocket_EndConnect(obj->socket, timeout);
}

kFx(kStatus) kTcpClient_Shutdown(kTcpClient client)
{
    kObj(kTcpClient, client); 
   
    return kSocket_Shutdown(obj->socket);
}

kFx(kStatus) kTcpClient_Wait(kTcpClient client, k64u timeout)
{
    kObj(kTcpClient, client); 
    
    if (kTcpClient_Available(client) > 0)
    {
        return kOK;
    }
    else
    {
/* 
 * FSS-288: Wind River APPNOTE_000531 suggests to use low water marks with Vx Works. 
 */
#if defined(K_VX_KERNEL)
        xkSocket_SetReadLowWater(obj->socket, 1);
#endif

        return xkSocket_WaitEx(obj->socket, kSOCKET_EVENT_READ, timeout);    
    }
}

kFx(kStatus) xkTcpClient_VFlush(kTcpClient client)
{
    kObj(kTcpClient, client); 
        
    if (obj->base.writeBegin > 0)
    {
        kCheck(xkTcpClient_WriteAll(client, obj->base.writeBuffer, obj->base.writeBegin));
        obj->base.writeBegin = 0; 
    }

    if (obj->isSeekEnabled && (obj->base.readBegin > 0))
    {
        kMemMove(obj->base.readBuffer, &obj->base.readBuffer[obj->base.readBegin], obj->base.readEnd - obj->base.readBegin); 
        obj->base.readEnd = (obj->base.readEnd - obj->base.readBegin); 
        obj->base.readBegin = 0; 
    }

    return kOK;
}

kFx(kStatus) xkTcpClient_VFill(kTcpClient client)
{
    kObj(kTcpClient, client); 
    kSize mediumCount; 

    kCheck(xkTcpClient_ReadAtLeast(client, &obj->base.readBuffer[0], 1, obj->base.readCapacity, &mediumCount));

    obj->base.readBegin = 0; 
    obj->base.readEnd = mediumCount;

    return kOK; 
}

kFx(kStatus) xkTcpClient_VSeek(kTcpClient client, k64s offset, kSeekOrigin origin)
{
    kObj(kTcpClient, client); 

    switch (origin)
    {
    case kSEEK_ORIGIN_BEGIN:
        offset -= (k64s)kStream_BytesRead(client); 
        break;
    case kSEEK_ORIGIN_CURRENT:
        break;
    case kSEEK_ORIGIN_END:
        return kERROR_PARAMETER;
    }

    if (offset >= 0)
    {
        kByte buffer[256]; 
        
        //support forward read seek by reading and discarding
        while (offset > 0)
        {
            k64u chunkSize = kMin_((k64u)offset, sizeof(buffer));

            kCheck(kStream_Read(client, buffer, (kSize)chunkSize));

            offset -= chunkSize;
        }
    }
    else if (((k64s)obj->base.readBegin + offset) >= 0)
    {
        //support reverse read seek by adjusting buffer pointers...
        obj->base.readBegin += (kSSize) offset; 
    }
    else
    {
        //...unless the seek destination is already out of buffer
        return kERROR_PARAMETER; 
    }

    return kOK; 
}

kFx(kStatus) xkTcpClient_VReadSomeImpl(kTcpClient client, void* buffer, kSize minCount, kSize maxCount, kSize* bytesRead)
{
    kObj(kTcpClient, client); 
    kByte* dest = (kByte*) buffer;  
    kSize readCount = 0; 
    kSize copyCount, mediumCount; 
    
    kCheckArgs(minCount <= maxCount); 

    //consume any bytes in the read buffer first
    if ((obj->base.readEnd - obj->base.readBegin) > 0)
    {
        copyCount = kMin_(maxCount, obj->base.readEnd - obj->base.readBegin);

        kMemCopy(dest, &obj->base.readBuffer[obj->base.readBegin], copyCount);     
        
        obj->base.readBegin += copyCount; 
        readCount += copyCount; 
    }

    //if the request is not yet satisfied
    if (readCount < minCount)
    {
        //if the request is larger than the internal read buffer, read directly into the caller's buffer; else read into the internal buffer
        if ((maxCount - readCount) > obj->base.readCapacity)
        {            
            kCheck(xkTcpClient_ReadAtLeast(client, &dest[readCount], minCount-readCount, maxCount-readCount, &mediumCount));

            obj->base.readBegin = obj->base.readEnd = 0; 
            readCount += mediumCount; 
        }
        else 
        {
            //if seek is not enabled, prefer to write data at the beginning of the buffer
            if (!obj->isSeekEnabled || ((maxCount - readCount) > (obj->base.readCapacity - obj->base.readEnd)))
            {
                obj->base.readBegin = obj->base.readEnd = 0; 
            }

            kCheck(xkTcpClient_ReadAtLeast(client, &obj->base.readBuffer[obj->base.readEnd], minCount - readCount, obj->base.readCapacity - obj->base.readEnd, &mediumCount));    

            copyCount = kMin_(mediumCount, maxCount - readCount); 

            kCheck(kMemCopy(&dest[readCount], &obj->base.readBuffer[obj->base.readEnd], copyCount)); 

            obj->base.readBegin += copyCount; 
            obj->base.readEnd += mediumCount; 
            readCount += copyCount; 
        }
    }

    if (!kIsNull(bytesRead))
    {
        *bytesRead = readCount; 
    }

    return kOK; 
}

kFx(kStatus) xkTcpClient_VWriteImpl(kTcpClient client, const void* buffer, kSize size)
{
    kObj(kTcpClient, client); 

    //if the internal write buffer already has content, but not enough free space, flush
    if ((obj->base.writeBegin > 0) && ((obj->base.writeEnd - obj->base.writeBegin) < size))
    {
        kCheck(kStream_Flush(client)); 
    }

    //if the write is larger than the internal write buffer, write directly to the medium; else write to the internal buffer
    if ((obj->base.writeEnd - obj->base.writeBegin) < size)
    {
        kCheck(xkTcpClient_WriteAll(client, (const kByte*) buffer, size)); 
    }
    else
    {
        kCheck(kMemCopy(&obj->base.writeBuffer[obj->base.writeBegin], buffer, size)); 
        obj->base.writeBegin += size; 
    }

    return kOK; 
}

kFx(kStatus) xkTcpClient_ReadAtLeast(kTcpClient client, kByte* buffer, kSize minCount, kSize maxCount, kSize* bytesRead)
{
    kObj(kTcpClient, client); 
    k64u startTime = (obj->readTimeout == 0) ? 0 : kTimer_Now(); 
    kBool hasTimedOut = kFALSE; 
    kStatus result; 
    kSize totalRead = 0; 
    kSize read; 

    kCheck(kSocket_SetEvents(obj->socket, kSOCKET_EVENT_READ)); 

    do
    {
        kCheck(kTcpClient_Status(client)); 

        hasTimedOut = (obj->readTimeout == 0) ? kFALSE : ((kTimer_Now() - startTime) >= obj->readTimeout); 

/* 
 * FSS-288: Wind River APPNOTE_000531 suggests to use low water marks with Vx Works. This decreases 
 * CPU utilization when receiving data.
 */
#if defined(K_VX_KERNEL)
    {
        k32s lowWater = (k32s)(minCount - totalRead);
        lowWater = kMin_(lowWater, (k32s)obj->socketReadBufferSize/2); 
        xkSocket_SetReadLowWater(obj->socket, lowWater);
    }
#endif

        result = kSocket_Wait(obj->socket, xkTCP_CLIENT_CANCEL_QUERY_INTERVAL); 

        if (result == kOK)
        {
            kCheck(kSocket_Read(obj->socket, &buffer[totalRead], maxCount-totalRead, &read));         
            obj->base.bytesRead += (k64u) read; 
            totalRead += read; 
        }
        else if ((result == kERROR_TIMEOUT) && !kIsNull(obj->cancelQuery.function))
        {
            if (!kSuccess(obj->cancelQuery.function(obj->cancelQuery.receiver, client, kNULL)))
            {
                kAtomic32s_Exchange(&obj->isCancelled, kTRUE);
                return kERROR_ABORT; 
            }
        }
        else if (result != kERROR_TIMEOUT)
        {
            return result; 
        }
    }
    while ((totalRead < minCount) && !hasTimedOut); 

    if (totalRead < minCount)
    {
        obj->timedOut = kTRUE; 
        return kERROR_TIMEOUT; 
    }

    *bytesRead = totalRead; 

    return kOK;  
}

kFx(kStatus) xkTcpClient_WriteAll(kTcpClient client, const kByte* buffer, kSize count)
{
    kObj(kTcpClient, client); 
    k64u startTime = (obj->writeTimeout == 0) ? 0 : kTimer_Now(); 
    kBool hasTimedOut = kFALSE; 
    kStatus result; 
    kSize totalWritten = 0; 
    kSize written; 
    kChar peekByte;

    do
    {
        kCheck(kTcpClient_Status(client)); 

        hasTimedOut = (obj->writeTimeout == 0) ? kFALSE : ((kTimer_Now() - startTime) >= obj->writeTimeout); 

        result = xkSocket_WaitEx(obj->socket, kSOCKET_EVENT_WRITE, xkTCP_CLIENT_CANCEL_QUERY_INTERVAL);
     
        if (result == kOK)
        {
            kCheck(kSocket_Write(obj->socket, &buffer[totalWritten], count-totalWritten, &written));         
            obj->base.bytesWritten += (k64u) written; 
            totalWritten += written; 
        }
        else if (result == kERROR_TIMEOUT) 
        {
            if (kSuccess(xkSocket_WaitEx(obj->socket, kSOCKET_EVENT_READ, 0)))
            {
                kCheck(xkSocket_Peek(obj->socket, &peekByte, sizeof(peekByte), kNULL));
            }

            if (!kIsNull(obj->cancelQuery.function))
            {
                if (!kSuccess(obj->cancelQuery.function(obj->cancelQuery.receiver, client, kNULL)))
                {
                    kAtomic32s_Exchange(&obj->isCancelled, kTRUE);
                    return kERROR_ABORT;
                }
            }
        }
        else
        {
            return result; 
        }
    }
    while ((totalWritten < count) && !hasTimedOut); 
  
    if (totalWritten < count)
    {
        obj->timedOut = kTRUE; 
        return kERROR_TIMEOUT; 
    }

    return kOK; 
}

kFx(kSocket) kTcpClient_Socket(kTcpClient client)
{ 
    kObj(kTcpClient, client); 

    return obj->socket; 
}

kFx(kSize) kTcpClient_Available(kTcpClient client)
{
    kObj(kTcpClient, client); 

    return obj->base.readEnd - obj->base.readBegin;
}

kFx(kStatus) kTcpClient_LocalEndPoint(kTcpClient client, kIpEndPoint* endPoint)
{
    kObj(kTcpClient, client); 
        
    return kSocket_LocalEndPoint(obj->socket, endPoint);
}

kFx(kStatus) kTcpClient_RemoteEndPoint(kTcpClient client, kIpEndPoint* endPoint)
{
    kObj(kTcpClient, client); 

    return kSocket_RemoteEndPoint(obj->socket, endPoint);
}

kFx(kStatus) kTcpClient_Status(kTcpClient client)
{
    kObj(kTcpClient, client); 

    if (kAtomic32s_Get(&obj->isCancelled))
    {
        return kERROR_ABORT; 
    }
    else if (obj->timedOut)
    {
        return kERROR_TIMEOUT; 
    }
    else
    {
        return xkSocket_Status(obj->socket); 
    }
}

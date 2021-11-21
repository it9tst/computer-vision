/** 
 * @file    kWebSocket.cpp
 *
 * @internal
 * Copyright (C) 2017-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */

#include <kApi/Io/kWebSocket.h>

#include <kApi/Crypto/kSha1Hash.h>
#include <kApi/Data/kArray1.h>
#include <kApi/Data/kString.h>
#include <kApi/Io/kHttpServerChannel.h>
#include <kApi/Io/kHttpServerRequest.h>
#include <kApi/Io/kHttpServerResponse.h>
#include <kApi/Io/kMemory.h>
#include <kApi/Io/kSerializer.h>
#include <kApi/Threads/kLock.h>
#include <kApi/Threads/kPeriodic.h>
#include <kApi/Threads/kTimer.h>

kBeginClassEx(k, kWebSocket)
    kAddPrivateVMethod(kWebSocket, kObject, VRelease)
    kAddPrivateVMethod(kWebSocket, kStream, VReadSomeImpl)
    kAddPrivateVMethod(kWebSocket, kStream, VWriteImpl)
    kAddPrivateVMethod(kWebSocket, kStream, VFill)

kEndClassEx()

kBeginEnumEx(k, kWebSocketDataType)
    kAddEnumerator(kWebSocketDataType, kWEB_SOCKET_DATA_TYPE_NULL)
    kAddEnumerator(kWebSocketDataType, kWEB_SOCKET_DATA_TYPE_UTF8)
    kAddEnumerator(kWebSocketDataType, kWEB_SOCKET_DATA_TYPE_BINARY)
kEndEnumEx()

kFx(kStatus) kWebSocket_Construct(kWebSocket* webSocket, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback(allocator);
    kType type = kTypeOf(kWebSocket);
    kStatus status;

    kCheck(kAlloc_GetObject(alloc, type, webSocket));

    if (!kSuccess(status = xkWebSocket_Init(*webSocket, type, kNULL, alloc)))
    {
        kAlloc_FreeRef(alloc, webSocket);
    }

    return status;
}

kFx(kStatus) xkWebSocket_ConstructFromRequest(kWebSocket* webSocket, kHttpServerChannel channel, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback(allocator);
    kType type = kTypeOf(kWebSocket);
    kStatus status;

    kCheck(kAlloc_GetObject(alloc, type, webSocket));

    if (!kSuccess(status = xkWebSocket_Init(*webSocket, type, channel, alloc)))
    {
        kAlloc_FreeRef(alloc, webSocket);
    }

    return status;
}

kFx(kStatus) xkWebSocket_Init(kWebSocket webSocket, kType type, kHttpServerChannel channel, kAlloc allocator)
{
    kObjR(kWebSocket, webSocket);
    kStatus status;
    
    kCheck(kStream_Init(webSocket, type, allocator));

    obj->isClient = kIsNull(channel);
    obj->shouldMaskSend = obj->isClient;
    obj->state = 0;
    obj->lock = kNULL;
    obj->tcpClient = kNULL;
    obj->serializer = kNULL;
    obj->maskBuffer = kNULL;
    obj->sendInfo.sendType = kWEB_SOCKET_DATA_TYPE_BINARY; 
    obj->sendInfo.inMessage = kFALSE;
    kZero(obj->recvInfo);
    obj->lastPongTime = 0;
    obj->backgroundTimer = kNULL;

    kTry
    {
        kTest(kLock_Construct(&obj->lock, allocator));

        if (kIsNull(channel))
        {
            kTest(kTcpClient_Construct(&obj->tcpClient, kIP_VERSION_4, allocator));  
        }
        else
        {
            kTest(xkWebSocket_ProcessHttpUpgradeRequest(webSocket, channel));
            kTest(kHttpServerChannel_DetachClient(channel, &obj->tcpClient));

            obj->state = xkWEB_SOCKET_STATE_CONNECTED; 
        }

        //configure client buffers to support efficient frame header parsing/formatting
        kTest(kTcpClient_SetWriteBuffers(obj->tcpClient, -1, xkWEB_SOCKET_TCP_CLIENT_WRITE_BUFFER)); 
        kTest(kTcpClient_SetReadBuffers(obj->tcpClient, -1, xkWEB_SOCKET_TCP_CLIENT_READ_BUFFER)); 

        if (obj->shouldMaskSend)
        {
            kTest(kArray1_Construct(&obj->maskBuffer, kTypeOf(kByte), xkWEB_SOCKET_MASK_BUFFER_SIZE, allocator)); 
        }

        kTest(kSerializer_Construct(&obj->serializer, obj->tcpClient, kNULL, allocator));
        kTest(kSerializer_SetEndianness(obj->serializer, kENDIANNESS_BIG)); 

        kTest(kPeriodic_Construct(&obj->backgroundTimer, allocator));
        kTest(kPeriodic_Start(obj->backgroundTimer, xkWEB_SOCKET_BACKGROUND_TIMER_PERIOD, xkWebSocket_OnBackgroundTimer, webSocket));
    }
    kCatch(&status)
    {
        xkWebSocket_VRelease(webSocket);

        kEndCatch(status);
    }

    return kOK;
}

kFx(kStatus) xkWebSocket_VRelease(kWebSocket webSocket)
{
    kObj(kWebSocket, webSocket);
    
    kCheck(kDestroyRef(&obj->backgroundTimer));
    
    kCheck(kDestroyRef(&obj->serializer));
    kCheck(kDestroyRef(&obj->tcpClient));
    
    kCheck(kDestroyRef(&obj->lock));

    kCheck(kDestroyRef(&obj->maskBuffer));

    kCheck(kObject_FreeMem(webSocket, obj->base.readBuffer));
    kCheck(kObject_FreeMem(webSocket, obj->base.writeBuffer));

    kCheck(kStream_VRelease(webSocket)); 

    return kOK;
}

kFx(kStatus) kWebSocket_SetWriteBuffers(kWebSocket webSocket, kSSize socketSize, kSSize clientSize)
{
    kObj(kWebSocket, webSocket);
 
    kLock_Enter(obj->lock);

    kTry
    {    
        kTestState(obj->base.writeBegin == 0);  

        if (socketSize >= 0)
        {
            kTest(kTcpClient_SetWriteBuffers(obj->tcpClient, socketSize, -1));
        }

        if (clientSize >= 0)
        {
            kTest(kObject_FreeMemRef(webSocket, &obj->base.writeBuffer));

            obj->base.writeCapacity = 0;
            obj->base.writeBegin = 0;
            obj->base.writeEnd = 0;

            if (clientSize > 0)
            {
                kTest(kObject_GetMem(webSocket, (kSize)clientSize, &obj->base.writeBuffer));
                obj->base.writeCapacity = (kSize)clientSize;
                obj->base.writeEnd = (kSize)clientSize;
            }
        }    
    }
    kFinally
    {
        kLock_Exit(obj->lock);

        kEndFinally();
    }

    return kOK;
}

kFx(kStatus) kWebSocket_SetReadBuffers(kWebSocket webSocket, kSSize socketSize, kSSize clientSize)
{
    kObj(kWebSocket, webSocket);

    kLock_Enter(obj->lock);

    kTry
    {
        kTestState((obj->base.readEnd - obj->base.readBegin) == 0);

        if (socketSize >= 0)
        {
            kTest(kTcpClient_SetReadBuffers(obj->tcpClient, socketSize, -1));
        }

        if (clientSize >= 0)
        {
            kTest(kObject_FreeMemRef(webSocket, &obj->base.readBuffer));

            obj->base.readCapacity = 0;
            obj->base.readBegin = 0;
            obj->base.readEnd = 0;

            if (clientSize > 0)
            {
                kTest(kObject_GetMem(webSocket, (kSize)clientSize, &obj->base.readBuffer));
                obj->base.readCapacity = (kSize)clientSize;
            }
        }
    }
    kFinally
    {
        kLock_Exit(obj->lock);

        kEndFinally();
    } 

    return kOK;
}

kFx(kStatus) kWebSocket_Connect(kWebSocket webSocket, kIpAddress address, k32u port, const kChar* host, const kChar* uri, k64u timeout)
{
    kObjR(kWebSocket, webSocket);
    kByte key[16] = { 0 };
    kStatus status; 

    kLock_Enter(obj->lock); 

    kTry
    {
        kTestState(obj->state == xkWEB_SOCKET_STATE_INITIALIZED); 

        kTest(kTcpClient_Connect(obj->tcpClient, address, port, timeout));

        obj->state = xkWEB_SOCKET_STATE_CONNECTING;
 
        kTest(kRandomBytes(key, sizeof(key)));

        kTest(xkWebSocket_SendHttpUpgradeRequest(webSocket, host, port, uri, key, sizeof(key)));
        kTest(xkWebSocket_ReceiveHttpUpgradeResponse(webSocket, key, sizeof(key)));

        obj->state = xkWEB_SOCKET_STATE_CONNECTED;
    }
    kCatchEx(&status)
    {
        if (obj->state == xkWEB_SOCKET_STATE_CONNECTING)
        {
            kTcpClient_Shutdown(obj->tcpClient); 
        }

        obj->state = xkWEB_SOCKET_STATE_CLOSED;

        kEndCatchEx(status);
    }
    kFinallyEx
    {
        kLock_Exit(obj->lock); 

        kEndFinallyEx(); 
    }

    return kOK;
}

kFx(kStatus) kWebSocket_Close(kWebSocket webSocket)
{
    kObj(kWebSocket, webSocket);
    kStatus status; 
 
    kLock_Enter(obj->lock);

    kTry
    {    
        if (xkWebSocket_IsValid(webSocket))
        {
            kTest(xkWebSocket_WriteFrame(webSocket, kNULL, 0, xkWEB_SOCKET_OP_CODE_CLOSE, kTRUE)); 

            obj->state = xkWEB_SOCKET_STATE_CLOSING;

            //discard frames until close response received (close response generates error)
            while (kSuccess(kWebSocket_Receive(webSocket, xkWEB_SOCKET_CLOSE_TIMEOUT))); 
        }
    }
    kCatchEx(&status)
    {
        kEndCatchEx(kOK);
    }
    kFinallyEx
    {
        obj->state = xkWEB_SOCKET_STATE_CLOSED; 

        kTcpClient_Shutdown(obj->tcpClient);

        kLock_Exit(obj->lock);

        kEndFinallyEx();
    }

    return kOK; 
}

kFx(kStatus) kWebSocket_SetSendType(kWebSocket webSocket, kWebSocketDataType type)
{
    kObj(kWebSocket, webSocket);

    kLock_Enter(obj->lock);

    kTry
    {
       kTestState(!obj->sendInfo.inMessage); 

       obj->sendInfo.sendType = type; 
    }
    kFinally
    {
        kLock_Exit(obj->lock);

        kEndFinally();
    }

    return kOK;
}

kFx(kStatus) kWebSocket_Send(kWebSocket webSocket)
{
    kObj(kWebSocket, webSocket);

    kLock_Enter(obj->lock);

    kTry
    {
        kTestState(xkWebSocket_IsValid(webSocket));

        kTest(xkWebSocket_WriteDataFrame(webSocket, obj->base.writeBuffer, obj->base.writeBegin, kTRUE));
        
        obj->base.writeBegin = 0;
    }
    kFinally
    {
        kLock_Exit(obj->lock);

        kEndFinally();
    }

    return kOK; 
}

kFx(kStatus) kWebSocket_Receive(kWebSocket webSocket, k64u timeout)
{
    kObj(kWebSocket, webSocket);

    kLock_Enter(obj->lock);

    kTry
    {
        kTest(xkWebSocket_IsValid(webSocket));

        //discard any partially read-out messages
        kTest(xkWebSocket_DiscardReceive(webSocket)); 

        //wait for the next data message header to arrive
        kTest(xkWebSocket_BeginReceive(webSocket, timeout)); 

        obj->recvInfo.inMessage = kTRUE; 

        //latch receive data type, to prevent being overwritten
        obj->recvInfo.lastRecvType = obj->recvInfo.recvType; 
    }
    kFinally
    {
        kLock_Exit(obj->lock);

        kEndFinally();
    }
    
    return kOK;
}

kFx(kStatus) kWebSocket_WriteMessage(kWebSocket webSocket, const void* buffer, kSize size)
{
    kObj(kWebSocket, webSocket);

    kLock_Enter(obj->lock);

    kTry
    {
        kTestState(xkWebSocket_IsValid(webSocket));

        kTestState(obj->base.writeBegin == 0); 
        kTestState(!obj->sendInfo.inMessage); 

        kTest(xkWebSocket_WriteDataFrame(webSocket, (const kByte*) buffer, size, kTRUE));
    }
    kFinally
    {
        kLock_Exit(obj->lock);

        kEndFinally();
    }

    return kOK;
}

kFx(kStatus) kWebSocket_ReadMessage(kWebSocket webSocket, kMemory stream, k64u timeout)
{
    kObj(kWebSocket, webSocket);
    kStatus readStatus = kOK; 
  
    kLock_Enter(obj->lock);

    kTry
    {
        kTestState(xkWebSocket_IsValid(webSocket));

        kTest(kWebSocket_Receive(webSocket, timeout));

        while (readStatus != kERROR_NOT_FOUND)
        {
            kSize bufferPosition = (kSize)kMemory_Position(stream);
            kSize bytesRead = 0;

            if (kMemory_Capacity(stream) == bufferPosition)
            {
                kTest(kMemory_Reserve(stream, kMax_(256, 2 * kMemory_Capacity(stream))));
            }

            readStatus = kStream_ReadSome(webSocket, kMemory_At(stream, bufferPosition), 1, kMemory_Capacity(stream) - bufferPosition, &bytesRead);

            if (readStatus != kERROR_NOT_FOUND)
            {
                kTest(readStatus);
            }

            kTest(kMemory_SetLength(stream, bufferPosition + bytesRead));
            kTest(kStream_Seek(stream, 0, kSEEK_ORIGIN_END));
        }
    }
    kFinally
    {
        kLock_Exit(obj->lock);

        kEndFinally();
    }
 
    return kOK;
}

kFx(kStatus) kWebSocket_Cancel(kWebSocket webSocket)
{
    kObj(kWebSocket, webSocket);

    //kTcpClient_Cancel is thread-safe; no need to lock
    return kTcpClient_Cancel(obj->tcpClient); 
}

kFx(kStatus) kWebSocket_SendPing(kWebSocket webSocket)
{
    kObj(kWebSocket, webSocket);

    kLock_Enter(obj->lock);

    kTry
    {
       kTestState(xkWebSocket_IsValid(webSocket)); 

       kTest(xkWebSocket_WriteFrame(webSocket, kNULL, 0, xkWEB_SOCKET_OP_CODE_PING, kTRUE)); 
    }
    kFinally
    {
        kLock_Exit(obj->lock);

        kEndFinally();
    }

    return kOK;
}

kFx(k64u) kWebSocket_LastPong(kWebSocket webSocket)
{
    kObj(kWebSocket, webSocket);
    k64u lastPong; 

    kLock_Enter(obj->lock);
    {
        lastPong = obj->lastPongTime; 
    }
    kLock_Exit(obj->lock);

    return lastPong; 
}

kFx(kWebSocketDataType) kWebSocket_SendType(kWebSocket webSocket)
{
    kObj(kWebSocket, webSocket);
    kWebSocketDataType sendType;

    kLock_Enter(obj->lock);
    {
        sendType = obj->sendInfo.sendType; 
    }
    kLock_Exit(obj->lock);

    return sendType; 
}

kFx(kWebSocketDataType) kWebSocket_ReceiveType(kWebSocket webSocket)
{
    kObj(kWebSocket, webSocket);
    kWebSocketDataType recvType;

    kLock_Enter(obj->lock);
    {
        recvType = obj->recvInfo.lastRecvType; 
    }
    kLock_Exit(obj->lock);

    return recvType; 
}

kFx(kStatus) xkWebSocket_VReadSomeImpl(kWebSocket webSocket, void* buffer, kSize minCount, kSize maxCount, kSize* bytesRead)
{
    kObj(kWebSocket, webSocket);
    kSize readCount = 0; 

    kLock_Enter(obj->lock);

    kTry
    {
        kByte* dest = (kByte*) buffer;  
        kSize copyCount, mediumCount; 
    
        kTestState(xkWebSocket_IsValid(webSocket));
        kTestArgs(minCount <= maxCount); 

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
                kTest(xkWebSocket_ReadAtLeast(webSocket, &dest[readCount], minCount - readCount, maxCount - readCount, &mediumCount));

                obj->base.readBegin = obj->base.readEnd = 0;
                readCount += mediumCount;
            }
            else
            {
                obj->base.readBegin = obj->base.readEnd = 0;

                kTest(xkWebSocket_ReadAtLeast(webSocket, &obj->base.readBuffer[obj->base.readEnd], minCount - readCount, obj->base.readCapacity - obj->base.readEnd, &mediumCount));

                copyCount = kMin_(mediumCount, maxCount - readCount);

                kTest(kMemCopy(&dest[readCount], &obj->base.readBuffer[obj->base.readEnd], copyCount));

                obj->base.readBegin += copyCount;
                obj->base.readEnd += mediumCount;
                readCount += copyCount;
            }
        }
    }
    kFinally
    {
        if (!kIsNull(bytesRead))
        {
            *bytesRead = readCount;
        }

        kLock_Exit(obj->lock);

        kEndFinally();
    }
    
    return kOK;
}

kFx(kStatus) xkWebSocket_VWriteImpl(kWebSocket webSocket, const void* buffer, kSize size)
{
    kObj(kWebSocket, webSocket);

    kLock_Enter(obj->lock);

    kTry
    {
        kTestState(xkWebSocket_IsValid(webSocket));

        //if the internal write buffer already has content, but not enough free space, flush frame
        if ((obj->base.writeBegin > 0) && ((obj->base.writeEnd - obj->base.writeBegin) < size))
        {
            kTest(xkWebSocket_WriteDataFrame(webSocket, obj->base.writeBuffer, obj->base.writeBegin, kFALSE));
            obj->base.writeBegin = 0;
        }

        //if the write is larger than the internal write buffer, write directly to the medium; else write to the internal buffer
        if ((obj->base.writeEnd - obj->base.writeBegin) < size)
        {
            kTest(xkWebSocket_WriteDataFrame(webSocket, (const kByte*) buffer, size, kFALSE));
        }
        else
        {
            kTest(kMemCopy(&obj->base.writeBuffer[obj->base.writeBegin], buffer, size));
            obj->base.writeBegin += size;
        }
    }
    kFinally
    {
        kLock_Exit(obj->lock);

        kEndFinally();
    }

    return kOK;
}

kFx(kStatus) xkWebSocket_VFill(kWebSocket webSocket)
{
    kObj(kWebSocket, webSocket);
    kSize mediumCount; 

    kLock_Enter(obj->lock);

    kTry
    { 
        kTestState(xkWebSocket_IsValid(webSocket));

        kTest(xkWebSocket_ReadAtLeast(webSocket, &obj->base.readBuffer[0], 1, obj->base.readCapacity, &mediumCount));

        obj->base.readBegin = 0;
        obj->base.readEnd = mediumCount;
    }
    kFinally
    {
        kLock_Exit(obj->lock);

        kEndFinally();
    }
    
    return kOK;
}

kFx(kStatus) xkWebSocket_SendHttpUpgradeRequest(kWebSocket webSocket, const kChar* host, k32u port, const kChar* uri, const kByte* key, kSize keyLength)
{
    kObjR(kWebSocket, webSocket);
    kString header = kNULL;
    kString keyString = kNULL;
 
    kTry
    {
        kTest(kString_Construct(&keyString, kNULL, kObject_Alloc(webSocket)));
        kTest(kString_Construct(&header, kNULL, kObject_Alloc(webSocket))); 
        
        kTest(kBase64Encode(key, keyLength, keyString));
 
        kTest(kString_Setf(header, "GET %s HTTP/1.1\r\n", uri)); 
        kTest(kString_Addf(header, "HOST: %s:%u\r\n", host, port));
        kTest(kString_Add(header, "Upgrade: websocket\r\n"));
        kTest(kString_Add(header, "Connection: Upgrade\r\n"));
        kTest(kString_Addf(header, "Sec-WebSocket-Key: %s\r\n", kString_Chars(keyString)));
        kTest(kString_Add(header, "Sec-WebSocket-Version: 13\r\n\r\n"));

        kTest(kStream_Write(obj->tcpClient, kString_Chars(header), kString_Length(header)));
        kTest(kStream_Flush(obj->tcpClient));
    }
    kFinally
    {
        kObject_Destroy(header);
        kObject_Destroy(keyString);

        kEndFinally();
    }

    return kOK; 
}

kFx(kStatus) xkWebSocket_ReceiveHttpUpgradeResponse(kWebSocket webSocket, const kByte* key, kSize keyLength)
{
    kObjR(kWebSocket, webSocket);
    const kChar endOfResponse[] = { '\r', '\n', '\r', '\n'  }; 
    kSize responsePos = 0;
    kSize bytesRead = 0; 

    //TODO: we currently discard the response; should parse and verify its contents
    do 
    {
        kChar nextByte; 

        //TODO: restructure to use kTcpClient_Wait, with timeout?
        kCheck(kStream_Read(obj->tcpClient, &nextByte, 1));  
        
        if (nextByte == endOfResponse[responsePos])
        {
            responsePos++; 
        }
        else 
        {
            responsePos = (nextByte == endOfResponse[0]) ? 1u : 0u; 
        }
    } 
    while ((++bytesRead < xkWEB_SOCKET_MAX_HTTP_RESPONSE_LENGTH) && (responsePos < sizeof(endOfResponse)));

    return (responsePos == sizeof(endOfResponse)) ? kOK : kERROR_STREAM; 
}

kFx(kStatus) xkWebSocket_ProcessHttpUpgradeRequest(kWebSocket webSocket, kHttpServerChannel channel)
{
    kObjR(kWebSocket, webSocket);
    kHttpServerRequest request = kHttpServerChannel_Request(channel);
    kHttpServerRequest response = kHttpServerChannel_Response(channel);
    const kChar* key = kNULL;
    kString acceptKey = kNULL;
    kHash hash = kNULL;
    kByte* digest = kNULL;
    
    kTry
    {
        kTestState(!kIsNull(key = kHttpServerRequest_FindHeaderValue(request, "Sec-WebSocket-Key")));

        kTest(kSha1Hash_Construct(&hash, kObject_Alloc(webSocket))); 
        kTest(kObject_GetMem(webSocket, kHash_DigestSize(hash), &digest));

        kTest(kString_Construct(&acceptKey, key, kObject_Alloc(webSocket)));
        kTest(kString_Add(acceptKey, xkWEB_SOCKET_HANDSHAKE_GUID));

        kTest(kHash_Update(hash, kString_Chars(acceptKey), kString_Length(acceptKey)));
        kTest(kHash_Digest(hash, digest, kHash_DigestSize(hash)));

        kTest(kBase64Encode(digest, kHash_DigestSize(hash), acceptKey));

        kTest(kHttpServerResponse_SetStatus(response, kHTTP_STATUS_SWITCHING_PROTOCOLS));
        kTest(kHttpServerResponse_SetHeader(response, "Upgrade", "websocket"));
        kTest(kHttpServerResponse_SetHeader(response, "Connection", "Upgrade"));
        kTest(kHttpServerResponse_SetHeader(response, "Sec-WebSocket-Accept", kString_Chars(acceptKey)));
    }
    kFinally
    {
        kObject_Destroy(acceptKey);
        kObject_Destroy(hash);
        kObject_FreeMem(webSocket, digest);

        kEndFinally();
    }

    return kOK;
}

kFx(kStatus) xkWebSocket_WriteDataFrame(kWebSocket webSocket, const kByte* buffer, kSize count, kBool finalFrame)
{
    kObj(kWebSocket, webSocket);
    k8u opCode; 

    if (obj->sendInfo.inMessage)
    {
        opCode = xkWEB_SOCKET_OP_CODE_CONTINUE; 
    }
    else if (obj->sendInfo.sendType == kWEB_SOCKET_DATA_TYPE_UTF8)
    {
        opCode = xkWEB_SOCKET_OP_CODE_UTF8; 
    }
    else
    {
        opCode = xkWEB_SOCKET_OP_CODE_BINARY; 
    }

    kCheck(xkWebSocket_WriteFrame(webSocket, buffer, count, opCode, finalFrame)); 

    obj->base.bytesWritten += count; 

    obj->sendInfo.inMessage = !finalFrame; 

    return kOK; 
}

kFx(kStatus) xkWebSocket_WriteFrame(kWebSocket webSocket, const kByte* buffer, kSize count, k8u opCode, kBool finalFrame)
{
    kObj(kWebSocket, webSocket);
    kByte maskKey[4]; 
    
    kCheck(kRandomBytes(maskKey, sizeof(maskKey)));

    kCheck(xkWebSocket_WriteHeader(webSocket, opCode, finalFrame, obj->shouldMaskSend, maskKey, sizeof(maskKey), count)); 
    
    if (!obj->shouldMaskSend)
    {
        kCheck(kStream_Write(obj->tcpClient, buffer, count)); 
    }
    else
    {
        kSize sent = 0; 
        kSize maskBufferSize = kArray1_Length(obj->maskBuffer); 
        kByte* maskBuffer = kArray1_DataT(obj->maskBuffer, kByte); 

        while (sent != count)
        {
            kSize chunkSize = kMin_(count-sent, maskBufferSize); 

            kCheck(xkWebSocket_ApplyMask(webSocket, &buffer[sent], maskBuffer, chunkSize, maskKey, sizeof(maskKey), sent)); 
           
            kCheck(kStream_Write(obj->tcpClient, maskBuffer, chunkSize)); 

            sent += chunkSize; 
        }       
    }

    kCheck(kStream_Flush(obj->tcpClient)); 

    return kOK;  
}

kFx(kStatus) xkWebSocket_WriteHeader(kWebSocket webSocket, k8u opCode, kBool finalFrame, kBool shouldMask, const kByte* maskKey, kSize maskKeySize, kSize payloadSize)
{
    kObj(kWebSocket, webSocket);
    k8u payloadSizeCode; 
    
    kCheck(kSerializer_Write8u(obj->serializer, (k8u) (xkWebSocket_SetFin(finalFrame) | xkWebSocket_SetOpCode(opCode)))); 
  
    if      (payloadSize < 126)             payloadSizeCode = (k8u) payloadSize;    
    else if (payloadSize <= k16U_MAX)       payloadSizeCode = xkWEB_SOCKET_16_BIT_PAYLOAD;
    else                                    payloadSizeCode = xkWEB_SOCKET_64_BIT_PAYLOAD;

    kCheck(kSerializer_Write8u(obj->serializer, (k8u) (xkWebSocket_SetMaskEnabled(shouldMask) | xkWebSocket_SetPayloadCode(payloadSizeCode)))); 

    if (payloadSizeCode == xkWEB_SOCKET_16_BIT_PAYLOAD)
    {
        kCheck(kSerializer_Write16u(obj->serializer, (k16u) payloadSize));
    }
    else if (payloadSizeCode == xkWEB_SOCKET_64_BIT_PAYLOAD)
    {
        kCheck(kSerializer_Write64u(obj->serializer, (k64u) payloadSize));
    }

    if (shouldMask)
    {            
        kCheck(kSerializer_WriteByteArray(obj->serializer, maskKey, maskKeySize));
    }

    kCheck(kSerializer_Flush(obj->serializer));

    return kOK; 
}

kFx(kStatus) xkWebSocket_DiscardReceive(kWebSocket webSocket)
{
    kObj(kWebSocket, webSocket);
    kByte discard[256];
    kSize bytesRead; 

    //discard buffered bytes
    obj->base.readBegin = obj->base.readEnd = 0; 

    //discard partially-unread-message bytes
    if (obj->recvInfo.inMessage)
    {
        while (kSuccess(xkWebSocket_ReadAtLeast(webSocket, discard, 1, sizeof(discard), &bytesRead))); 
    }

    return kOK; 
}

kFx(kStatus) xkWebSocket_BeginReceive(kWebSocket webSocket, k64u timeout)
{
    kObj(kWebSocket, webSocket);

    while (!obj->recvInfo.inDataFrame && kSuccess(kTcpClient_Wait(obj->tcpClient, timeout)))
    {
        kCheck(xkWebSocket_ProcessHeader(webSocket));  
    }

    return (obj->recvInfo.inDataFrame) ? kOK : kERROR_TIMEOUT;
}

kFx(kStatus) xkWebSocket_ProcessHeader(kWebSocket webSocket)
{
    kObj(kWebSocket, webSocket);
    kStatus exception; 

    kTry
    {
        kTest(xkWebSocket_ParseHeader(webSocket));
        kTest(xkWebSocket_InterpretHeader(webSocket));

        if (!obj->recvInfo.inDataFrame)
        {
            kTest(xkWebSocket_ProcessControlFrame(webSocket));
        }
    }
    kCatch(&exception)
    {
        if (exception == kERROR_CLOSED)
        {
            obj->state = xkWEB_SOCKET_STATE_CLOSED; 
        }

        kEndCatch(exception);
    }

    return kOK; 
}

kFx(kStatus) xkWebSocket_ParseHeader(kWebSocket webSocket)
{
    kObj(kWebSocket, webSocket);
    k8u header0 = 0;
    k8u header1 = 0; 
    k8u payloadCode = 0; 

    kCheck(kSerializer_Read8u(obj->serializer, &header0)); 
    kCheck(kSerializer_Read8u(obj->serializer, &header1)); 

    obj->recvInfo.opCode = xkWebSocket_OpCode(header0);
    obj->recvInfo.isLastFrame = xkWebSocket_Fin(header0);
    obj->recvInfo.isMasked = xkWebSocket_MaskEnabled(header1);

    payloadCode = (k8u) xkWebSocket_PayloadCode(header1);

    //if the payload size doesn't fit in the second header byte, 
    //the next 16 or 64 bits will contain the correct payload size
    if (payloadCode < xkWEB_SOCKET_16_BIT_PAYLOAD)
    {
        obj->recvInfo.payloadSize = (kSize) payloadCode; 
    }
    else if (payloadCode == xkWEB_SOCKET_16_BIT_PAYLOAD)
    {
        k16u payload;

        kCheck(kSerializer_Read16u(obj->serializer, &payload)); 
               
        obj->recvInfo.payloadSize = (kSize)payload;
    }
    else if (payloadCode == xkWEB_SOCKET_64_BIT_PAYLOAD)
    {
        k64u payload;

        kCheck(kSerializer_Read64u(obj->serializer, &payload)); 

        obj->recvInfo.payloadSize = (kSize)payload;
    }
    else
    {
        kCheck(kERROR_STREAM); 
    }

    if (obj->recvInfo.isMasked)
    {
        kCheck(kSerializer_ReadByteArray(obj->serializer, &obj->recvInfo.maskKey[0], sizeof(obj->recvInfo.maskKey)));  
    }

    return kOK;
}

kFx(kStatus) xkWebSocket_InterpretHeader(kWebSocket webSocket)
{
    kObj(kWebSocket, webSocket);
    kBool isFirstDataFrame = (obj->recvInfo.opCode == xkWEB_SOCKET_OP_CODE_UTF8) || (obj->recvInfo.opCode == xkWEB_SOCKET_OP_CODE_BINARY); 
    kBool isNextDataFrame = (obj->recvInfo.opCode == xkWEB_SOCKET_OP_CODE_CONTINUE); 
    
    obj->recvInfo.isFirstDataFrame = isFirstDataFrame; 
    obj->recvInfo.inDataFrame = isFirstDataFrame || isNextDataFrame; 
    obj->recvInfo.dataRead = 0; 

    if (obj->recvInfo.opCode == xkWEB_SOCKET_OP_CODE_UTF8)
    {
        obj->recvInfo.recvType = kWEB_SOCKET_DATA_TYPE_UTF8; 
    }
    else if (obj->recvInfo.opCode == xkWEB_SOCKET_OP_CODE_BINARY)
    {
        obj->recvInfo.recvType = kWEB_SOCKET_DATA_TYPE_BINARY; 
    }

    return kOK;
}

kFx(kStatus) xkWebSocket_ProcessControlFrame(kWebSocket webSocket)
{
    kObj(kWebSocket, webSocket);
    kByte payload[xkWEB_SOCKET_MAX_CONTROL_PAYLOAD];
    kSize payloadSize = obj->recvInfo.payloadSize; 

    kCheckTrue(payloadSize <= xkWEB_SOCKET_MAX_CONTROL_PAYLOAD, kERROR_STREAM);

    //control frame payload must be less than or equal to 125 bytes; control frames cannot be fragmented
    kCheck(kStream_Read(obj->tcpClient, payload, payloadSize)); 

    if (obj->recvInfo.isMasked)
    {
        kCheck(xkWebSocket_ApplyMask(webSocket, payload, payload, obj->recvInfo.payloadSize, obj->recvInfo.maskKey, sizeof(obj->recvInfo.maskKey), 0));
    }

    switch (obj->recvInfo.opCode)
    {
    case xkWEB_SOCKET_OP_CODE_CLOSE:     kCheck(xkWebSocket_ProcessClose(webSocket, payload, payloadSize));        break;
    case xkWEB_SOCKET_OP_CODE_PING:      kCheck(xkWebSocket_ProcessPing(webSocket, payload, payloadSize));         break;
    case xkWEB_SOCKET_OP_CODE_PONG:      kCheck(xkWebSocket_ProcessPong(webSocket, payload, payloadSize));         break;
    default:                                                                                                     break;
    }

    return kOK; 
}

kFx(kStatus) xkWebSocket_ProcessPing(kWebSocket webSocket, const kByte* payload, kSize payloadSize)
{
    kObj(kWebSocket, webSocket);
 
    kCheck(xkWebSocket_WriteFrame(webSocket, payload, payloadSize, xkWEB_SOCKET_OP_CODE_PONG, kTRUE)); 
    
    return kOK; 
}

kFx(kStatus) xkWebSocket_ProcessPong(kWebSocket webSocket, const kByte* payload, kSize payloadSize)
{
    kObj(kWebSocket, webSocket);
    
    obj->lastPongTime = kTimer_Now(); 

    return kOK; 
}

kFx(kStatus) xkWebSocket_ProcessClose(kWebSocket webSocket, const kByte* payload, kSize payloadSize)
{
    kObj(kWebSocket, webSocket);

    obj->state = xkWEB_SOCKET_STATE_CLOSING; 

    xkWebSocket_WriteFrame(webSocket, payload, payloadSize, xkWEB_SOCKET_OP_CODE_CLOSE, kTRUE); 

    kTcpClient_Shutdown(obj->tcpClient); 

    obj->state = xkWEB_SOCKET_STATE_CLOSED; 

    return kERROR_CLOSED; 
}

kFx(kStatus) xkWebSocket_ReadAtLeast(kWebSocket webSocket, kByte* buffer, kSize minCount, kSize maxCount, kSize* bytesRead)
{
    kObj(kWebSocket, webSocket);
    kSize totalBytesRead = 0;
    kStatus exception; 

    kTry
    {
        while (totalBytesRead < minCount)
        {
            kSize payloadRemaining, minRequest, maxRequest;
            kSize currentBytesRead = 0;

            if (!obj->recvInfo.inMessage)
            {
                //generate special end-of-messsage error code
                kThrow(kERROR_NOT_FOUND); 
            }

            while (!obj->recvInfo.inDataFrame)
            {
                kTest(xkWebSocket_ProcessHeader(webSocket));
            }

            payloadRemaining = obj->recvInfo.payloadSize - obj->recvInfo.dataRead;
            minRequest = kMin_(minCount, payloadRemaining);
            maxRequest = kMin_(maxCount, payloadRemaining);

            kTest(kStream_ReadSome(obj->tcpClient, &buffer[totalBytesRead], minRequest, maxRequest, &currentBytesRead)); 

            obj->base.bytesRead += currentBytesRead; 

            if (obj->recvInfo.isMasked)
            {
                kTest(xkWebSocket_ApplyMask(webSocket, &buffer[totalBytesRead], &buffer[totalBytesRead], currentBytesRead,
                    obj->recvInfo.maskKey, sizeof(obj->recvInfo.maskKey), obj->recvInfo.dataRead));
            }

            totalBytesRead += currentBytesRead;
            obj->recvInfo.dataRead += currentBytesRead;

            if ((obj->recvInfo.dataRead == obj->recvInfo.payloadSize))
            {
                obj->recvInfo.inDataFrame = kFALSE; 

                if (obj->recvInfo.isLastFrame)
                {
                    obj->recvInfo.inMessage = kFALSE; 
                }
            }
        }
    }
    kCatchEx(&exception)
    {
        if (exception == kERROR_CLOSED)
        {
            obj->state = xkWEB_SOCKET_STATE_CLOSED; 
        }

        kEndCatchEx(exception);
    }
    kFinallyEx
    {
        if (!kIsNull(bytesRead))
        {
            *bytesRead = totalBytesRead;
        }

        kEndFinallyEx(); 
    }

    return kOK; 
}

kFx(kStatus) xkWebSocket_ApplyMask(kWebSocket webSocket, const kByte* src, kByte* dest, kSize count, kByte* maskKey, kSize maskKeySize, kSize initialPosition)
{
    kSize i; 

    kCheckArgs(maskKeySize == 4); 

    //frame masking algorithm, as per RFC 6455
    for (i = 0; i < count; ++i)
    {
        dest[i] = (kByte)(src[i] ^ maskKey[(initialPosition + i) & 0x3]);
    }

    return kOK; 
}

kFx(kBool) xkWebSocket_IsValid(kWebSocket webSocket)
{
    kObj(kWebSocket, webSocket);

    return (obj->state == xkWEB_SOCKET_STATE_CONNECTED) && !kIsError(kTcpClient_Status(obj->tcpClient)); 
}

kFx(kStatus) xkWebSocket_OnBackgroundTimer(kWebSocket webSocket, kPeriodic timer)
{
    kObj(kWebSocket, webSocket);

    kLock_Enter(obj->lock);
    {
        if (xkWebSocket_IsValid(webSocket))
        {
            xkWebSocket_BeginReceive(webSocket, 0);
        }
    }
    kLock_Exit(obj->lock);
    
    return kOK;
}

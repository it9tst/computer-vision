/** 
 * @file    kSocket.cpp
 *
 * @internal
 * Copyright (C) 2004-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#define K_PLATFORM
#include <kApi/Io/kSocket.h>

/*
 * kSocketType
 */
kBeginEnumEx(k, kSocketType)
    kAddEnumerator(kSocketType, kSOCKET_TYPE_TCP)
    kAddEnumerator(kSocketType, kSOCKET_TYPE_UDP)
kEndEnumEx()

/*
 * kSocketWait
 */
kBeginEnumEx(k, kSocketEvent)
    kAddEnumerator(kSocketEvent, kSOCKET_EVENT_READ)
    kAddEnumerator(kSocketEvent, kSOCKET_EVENT_WRITE)
    kAddEnumerator(kSocketEvent, kSOCKET_EVENT_EXCEPT)
kEndEnumEx()

/*
 * kSocket
 */

kBeginClassEx(k, kSocket)
    kAddPrivateVMethod(kSocket, kObject, VRelease)
kEndClassEx()

kFx(kStatus) kSocket_WaitAny(const kSocket* sockts, kSize count, k64u timeout)
{    
    struct timeval tv;
    fd_set readSet, writeSet, exceptSet;
    xkSocketHandle highest = 0;
    int result;
    kSize i;    

    kCheckArgs(count <= FD_SETSIZE); 

    FD_ZERO(&readSet); 
    FD_ZERO(&writeSet); 
    FD_ZERO(&exceptSet); 

    for (i = 0; i < count; i++)
    {
        kObj(kSocket, sockts[i]); 

        if (obj->handle > highest)
        {
            highest = obj->handle;
        }

        if (obj->eventTypes & kSOCKET_EVENT_READ)      FD_SET(obj->handle, &readSet); 
        if (obj->eventTypes & kSOCKET_EVENT_WRITE)     FD_SET(obj->handle, &writeSet); 
        if (obj->eventTypes & kSOCKET_EVENT_EXCEPT)    FD_SET(obj->handle, &exceptSet); 

        obj->eventStatus = 0; 
    }

    result = xkSocket_Select((int) (highest) + 1, &readSet, &writeSet, &exceptSet, xkSocket_FormatTimeVal(&tv, timeout));

    if (result == 0) 
    {
        return kERROR_TIMEOUT;
    }
    else if (result < 0)
    {
        return kERROR_STREAM;
    }

    for (i = 0; i < count; i++)
    {
        kObj(kSocket, sockts[i]); 

        if ((obj->eventTypes & kSOCKET_EVENT_READ)  && FD_ISSET(obj->handle, &readSet))      obj->eventStatus |= kSOCKET_EVENT_READ; 
        if ((obj->eventTypes & kSOCKET_EVENT_WRITE) && FD_ISSET(obj->handle, &writeSet))     obj->eventStatus |= kSOCKET_EVENT_WRITE;  
        if ((obj->eventTypes & kSOCKET_EVENT_EXCEPT) && FD_ISSET(obj->handle, &exceptSet))   obj->eventStatus |= kSOCKET_EVENT_EXCEPT; 
    }

    return kOK;
}

kFx(struct timeval*) xkSocket_FormatTimeVal(struct timeval* tv, k64u timeout)
{
    if (timeout == kINFINITE)
    {
        return kNULL; 
    }
  
    tv->tv_sec = (long) timeout / 1000000;
    tv->tv_usec = (int) (timeout % 1000000);
   
    return tv;  
}

kFx(kStatus) kSocket_Construct(kSocket* sockt, kIpVersion ipVersion, kSocketType socketType, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback(allocator);
    kType type = kTypeOf(kSocket); 
    kStatus status; 

    kCheck(kAlloc_GetObject(alloc, type, sockt)); 

    if (!kSuccess(status = xkSocket_Init(*sockt, type, ipVersion, socketType, alloc)))
    {
        kAlloc_FreeRef(alloc, sockt); 
    }

    return status; 
} 

kFx(kStatus) xkSocket_ConstructFromHandle(kSocket* sockt, kIpVersion ipVersion, kSocketType socktType, xkSocketHandle handle, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback(allocator);
    kType type = kTypeOf(kSocket); 
    kStatus status; 

    kCheck(kAlloc_GetObject(alloc, type, sockt)); 

    if (!kSuccess(status = xkSocket_InitFromHandle(*sockt, type, ipVersion, socktType, handle, alloc)))
    {
        kAlloc_FreeRef(alloc, sockt); 
    }

    return status; 
} 

kFx(kStatus) xkSocket_Init(kSocket sockt, kType type, kIpVersion ipVersion, kSocketType socketType, kAlloc allocator)
{
    if (ipVersion != kIP_VERSION_4)
    {
        return kERROR_UNIMPLEMENTED; 
    }
    else
    {
        k32s sockFamily = AF_INET; 
        k32s sockType = (socketType == kSOCKET_TYPE_TCP) ? SOCK_STREAM : SOCK_DGRAM; 
        xkSocketHandle handle = socket(sockFamily, sockType, 0);
        kStatus status = kOK;

        if (handle == xkSOCKET_INVALID_SOCKET)
        {
            status = kERROR_NETWORK; 
        }
        else if (!kSuccess(status = xkSocket_InitFromHandle(sockt, type, ipVersion, socketType, handle, allocator)))
        {
            xkSocket_CloseHandle(handle); 
        }

        return status; 
    }
}

kFx(kStatus) xkSocket_InitFromHandle(kSocket sockt, kType type, kIpVersion ipVersion, kSocketType socktType, xkSocketHandle handle, kAlloc allocator)
{
    kObjR(kSocket, sockt);  

    kCheckArgs(handle != xkSOCKET_INVALID_SOCKET); 
    
    kCheck(kObject_Init(sockt, type, allocator)); 

    obj->ipVersion = ipVersion; 
    obj->socketType = socktType; 
    obj->handle = handle; 
    obj->eventTypes = kSOCKET_EVENT_READ; 
    obj->eventStatus = 0; 
    obj->status = kOK;
    obj->isBlocking = kTRUE; 

    return kOK; 
}

kFx(kStatus) xkSocket_VRelease(kSocket sockt)
{
    kObj(kSocket, sockt); 
    
    if (obj->handle != xkSOCKET_INVALID_SOCKET)
    {
        kSocket_Shutdown(sockt); 

        kCheck(xkSocket_CloseHandle(obj->handle)); 
    }

    kCheck(kObject_VRelease(sockt)); 

    return kOK;
}

kFx(kStatus) kSocket_Bind(kSocket sockt, kIpAddress address, k32u port)
{
    kObj(kSocket, sockt); 
    struct sockaddr_in sockAddr;

    kCheck(obj->status);
    kCheckArgs(address.version == kIP_VERSION_4); 
    
    kCheck(xkIpAddress_ToSockAddr(address, port, &sockAddr));           
    kCheck(bind(obj->handle, (struct sockaddr*)&sockAddr, sizeof(sockAddr)) == 0); 

    return kOK; 
}

kFx(kStatus) kSocket_Connect(kSocket sockt, kIpAddress address, k32u port, k64u timeout)
{
    kCheck(kSocket_BeginConnect(sockt, address, port)); 
    kCheck(kSocket_EndConnect(sockt, timeout)); 
 
    return kOK;
}

kFx(kStatus) kSocket_BeginConnect(kSocket sockt, kIpAddress address, k32u port)
{
    kObj(kSocket, sockt); 
    struct sockaddr_in sockAddr;
    kStatus exception;

    kTry
    {
        kTest(obj->status);
        kTestArgs(address.version == kIP_VERSION_4);

        kTest(xkIpAddress_ToSockAddr(address, port, &sockAddr));

        obj->wasBlocking = obj->isBlocking;
        kTest(kSocket_SetBlocking(sockt, kFALSE));

        obj->status = kERROR_IN_PROGRESS;
        obj->beginConnectResult = connect(obj->handle, (struct sockaddr*)&sockAddr, sizeof(sockAddr));
        obj->beginConnectLastError = xkSocket_GetLastError(); 
    }
    kCatch(&exception)
    {
        obj->status = exception; 

        kEndCatch(kOK); 
    }

    return kOK;
}

kFx(kStatus) kSocket_EndConnect(kSocket sockt, k64u timeout)
{
    kObj(kSocket, sockt); 
    fd_set exceptSet; 
    fd_set writeSet; 
    int selectResult = 0; 
    struct timeval tv; 
    kStatus exception;

    kTry
    {     
        if (obj->status != kERROR_IN_PROGRESS)
        {
            kThrow(obj->status); 
        }

        obj->status = kOK; 

        if (obj->beginConnectResult == xkSOCKET_ERROR)
        {
            if (obj->beginConnectLastError == xkSOCKET_WOULD_BLOCK || obj->beginConnectLastError == xkSOCKET_IN_PROGRESS)
            {                
                FD_ZERO(&writeSet); 
                FD_ZERO(&exceptSet); 

                FD_SET(obj->handle, &writeSet); 
                FD_SET(obj->handle, &exceptSet); 

                selectResult = xkSocket_Select((int)(obj->handle)+1, NULL, &writeSet, &exceptSet, xkSocket_FormatTimeVal(&tv, timeout)); 

                if ((selectResult == 1) && !FD_ISSET(obj->handle, &exceptSet))
                {
                    int errorCode; 
                    socklen_t errorSize = sizeof(int); 

                    kTest(getsockopt(obj->handle, SOL_SOCKET, SO_ERROR, (char*)&errorCode, &errorSize) == 0); 

                    if (errorCode != 0)
                    {
                        kThrow(kERROR_NETWORK); 
                    }
                }
                else if (selectResult == 0)
                {
                    kThrow(kERROR_TIMEOUT); 
                }
                else
                {
                    kThrow(kERROR_NETWORK); 
                }               
            }
            else
            {
                kThrow(kERROR_NETWORK);
            }
        }   
    
        if (obj->wasBlocking)
        {
            kTest(kSocket_SetBlocking(sockt, obj->wasBlocking));
        }
    }
    kCatch(&exception)
    {
        obj->status = exception;
        kEndCatch(exception); 
    }

    return kOK;
}

kFx(kStatus) kSocket_Accept(kSocket sockt, kSocket* connection, kAlloc allocator)
{
    kObj(kSocket, sockt); 
    xkSocketHandle handle = xkSOCKET_INVALID_SOCKET; 
    kStatus status = kOK; 

    kCheck(obj->status);
        
    if ((handle = accept(obj->handle, 0, 0)) == xkSOCKET_INVALID_SOCKET)
    {
        *connection = kNULL; 
    }
    else if (!kSuccess(status = xkSocket_ConstructFromHandle(connection, obj->ipVersion, kSOCKET_TYPE_TCP, handle, allocator)))
    {
        xkSocket_CloseHandle(handle); 
    }

    return status; 
}

kFx(kStatus) kSocket_JoinMulticastGroup(kSocket socket, kIpAddress group, kIpAddress iface)
{
    kObj(kSocket, socket);
    struct ip_mreq request; 
 
    request.imr_multiaddr.s_addr = kIpAddress_ToNet32u(group); 
    request.imr_interface.s_addr = kIpAddress_ToNet32u(iface); 
  
    kCheck(setsockopt(obj->handle, IPPROTO_IP, IP_ADD_MEMBERSHIP, (char*) &request, sizeof(request)) != xkSOCKET_ERROR);

    return kOK;
}

kFx(kStatus) kSocket_LeaveMulticastGroup(kSocket socket, kIpAddress group, kIpAddress iface)
{
    kObj(kSocket, socket);
    struct ip_mreq request; 
 
    request.imr_multiaddr.s_addr = kIpAddress_ToNet32u(group); 
    request.imr_interface.s_addr = kIpAddress_ToNet32u(iface); 
  
    kCheck(setsockopt(obj->handle, IPPROTO_IP, IP_DROP_MEMBERSHIP, (char*) &request, sizeof(request)) != xkSOCKET_ERROR);

    return kOK;
}

kFx(kStatus) kSocket_Listen(kSocket sockt, kSize backlog)
{ 
    kObj(kSocket, sockt); 

    kCheck(obj->status);

    kCheck(listen(obj->handle, (int) backlog) != xkSOCKET_ERROR); 

    return kOK;
}

kFx(kStatus) kSocket_Shutdown(kSocket sockt)
{ 
    kObj(kSocket, sockt); 

    if (!kIsError(obj->status))
    {
        xkSocket_ShutdownHandle(obj->handle);  

        obj->status = kERROR_CLOSED; 
    }

    return kOK;
}

kFx(kStatus) kSocket_SetEvents(kSocket sockt, kSocketEvent events)
{
    kObj(kSocket, sockt); 

    obj->eventTypes = events;       

    return kOK; 
}

kFx(kSocketEvent) kSocket_Events(kSocket sockt)
{
    kObj(kSocket, sockt); 

    return obj->eventStatus; 
}

kFx(kStatus) kSocket_Wait(kSocket sockt, k64u timeout)
{
    kObj(kSocket, sockt);

     return xkSocket_WaitEx(sockt, obj->eventTypes, timeout);
}

kFx(kStatus) xkSocket_WaitEx(kSocket sockt, kSocketEvent events, k64u timeout)
{
    kObj(kSocket, sockt); 
    struct timeval tv;
    fd_set readSet, writeSet, exceptSet;  
    kStatus result; 

    kCheck(obj->status);

    FD_ZERO(&readSet);
    FD_ZERO(&writeSet);
    FD_ZERO(&exceptSet);

    if (events & kSOCKET_EVENT_READ)      FD_SET(obj->handle, &readSet);
    if (events & kSOCKET_EVENT_WRITE)     FD_SET(obj->handle, &writeSet);
    if (events & kSOCKET_EVENT_EXCEPT)    FD_SET(obj->handle, &exceptSet);

    obj->eventStatus = 0; 

    result = xkSocket_Select((int)(obj->handle)+1, &readSet, &writeSet, &exceptSet, xkSocket_FormatTimeVal(&tv, timeout)); 

    if (result == 0) 
    {
        return kERROR_TIMEOUT;
    }
    else if (result < 0)
    {
        obj->status = kERROR_STREAM; 
        return obj->status;
    }

    if ((events & kSOCKET_EVENT_READ) && FD_ISSET(obj->handle, &readSet))    obj->eventStatus |= kSOCKET_EVENT_READ;
    if ((events & kSOCKET_EVENT_WRITE) && FD_ISSET(obj->handle, &writeSet))   obj->eventStatus |= kSOCKET_EVENT_WRITE;
    if ((events & kSOCKET_EVENT_EXCEPT) && FD_ISSET(obj->handle, &exceptSet))  obj->eventStatus |= kSOCKET_EVENT_EXCEPT;

    return kOK; 
}


kFx(kStatus) xkSocket_Peek(kSocket sockt, void* buffer, kSize size, kSize* read)
{
    return xkSocket_ReadImpl(sockt, buffer, size, read, xkSOCKET_MSG_PEEK);
}

kFx(kStatus) kSocket_Read(kSocket sockt, void* buffer, kSize size, kSize* read)
{
    return xkSocket_ReadImpl(sockt, buffer, size, read, 0);
}

kFx(kStatus) xkSocket_ReadImpl(kSocket sockt, void* buffer, kSize size, kSize* read, k32s options)
{
    kObj(kSocket, sockt);
    kSSize result;
    k32s socketError;
    kStatus opStatus = kOK;

    kCheck(obj->status);

    if (size == 0)
    {
        return kOK;
    }

    result = xkSocket_Recv(sockt, buffer, size, options);

    if (result > 0)
    {
        if (!kIsNull(read))
        {
            *read = (kSize)result;
        }
    }
    else if (result == 0)
    {
        opStatus = kERROR_CLOSED;
        obj->status = kERROR_CLOSED;
    }
    else
    {
        socketError = xkSocket_GetLastError();

        if ((socketError == xkSOCKET_TIMEDOUT) || (socketError == xkSOCKET_WOULD_BLOCK))
        {
            if (!obj->isBlocking)
            {
                opStatus = kERROR_BUSY;
            }
            else
            {
                opStatus = kERROR_TIMEOUT;

                if (obj->socketType == kSOCKET_TYPE_TCP)
                {
                    obj->status = kERROR_TIMEOUT;
                }
            }
        }
        else
        {
            opStatus = kERROR_STREAM;

            if (obj->socketType == kSOCKET_TYPE_TCP)
            {
                obj->status = kERROR_STREAM;
            }
        }
    }

    return opStatus;
}

kFx(kStatus) kSocket_ReadFrom(kSocket sockt, kIpEndPoint *endPoint, void* buffer, kSize size, kSize* read)
{
    kObj(kSocket, sockt); 
    struct sockaddr_in sockAddr;
    kSSize result; 
    k32s socketError; 
    kStatus opStatus = kOK; 

    kCheck(obj->status);

    result = xkSocket_RecvFrom(sockt, buffer, size, &sockAddr);
    
    if (result > 0)
    {
        if (!kIsNull(endPoint))
        {
            kCheck(xkIpAddress_FromSockAddr(&sockAddr, &endPoint->address, &endPoint->port));
        }
        if (!kIsNull(read))
        {
            *read = (kSize) result; 
        }
    }
    else if (result == 0)
    {
        opStatus = kERROR_CLOSED; 
        obj->status = kERROR_CLOSED; 
    }
    else
    {
        socketError = xkSocket_GetLastError(); 
        
        if ((socketError == xkSOCKET_TIMEDOUT) || (socketError == xkSOCKET_WOULD_BLOCK))
        {
            if (!obj->isBlocking)
            {
                opStatus = kERROR_BUSY; 
            }
            else
            {
                opStatus = kERROR_TIMEOUT; 

                if (obj->socketType == kSOCKET_TYPE_TCP)
                {
                    obj->status = kERROR_TIMEOUT;
                }
            }
        }
        else if (socketError == xkSOCKET_MSGSIZE)
        {
            opStatus = kERROR_INCOMPLETE; 
        }
        else
        {
            opStatus = kERROR_STREAM; 

            if (obj->socketType == kSOCKET_TYPE_TCP)
            {
                obj->status = kERROR_STREAM; 
            }
        }
    }
    
    return opStatus;
}

kFx(kStatus) kSocket_ReadFromEx(kSocket sockt, kIpEndPoint *endPoint, kSize* interfaceIndex, void* buffer, kSize size, kSize* read)
{
    kObj(kSocket, sockt);
    struct sockaddr_in sockAddr;
    kSSize result;
    k32s socketError;
    kStatus opStatus = kOK;

    kCheck(obj->status);
    kCheckTrue(obj->socketType == kSOCKET_TYPE_UDP, kERROR_STATE);

    result = xkSocket_RecvMessage(sockt, buffer, size, &sockAddr, interfaceIndex);

    if (result > 0)
    {
        if (!kIsNull(endPoint))
        {
            kCheck(xkIpAddress_FromSockAddr(&sockAddr, &endPoint->address, &endPoint->port));
        }
        if (!kIsNull(read))
        {
            *read = (kSize)result;
        }
    }
    else
    {
        socketError = xkSocket_GetLastError();

        if ((socketError == xkSOCKET_TIMEDOUT) || (socketError == xkSOCKET_WOULD_BLOCK))
        {
            if (!obj->isBlocking)
            {
                opStatus = kERROR_BUSY;
            }
            else
            {
                opStatus = kERROR_TIMEOUT;
            }
        }
        else if (socketError == xkSOCKET_MSGSIZE)
        {
            opStatus = kERROR_INCOMPLETE;
        }
        else
        {
            opStatus = kERROR_STREAM;
        }
    }

    return opStatus;
}

kFx(kStatus) kSocket_Write(kSocket sockt, const void* buffer, kSize size, kSize* written)
{
    kObj(kSocket, sockt); 
    kSSize result; 
    k32s socketError; 
    kStatus opStatus = kOK; 

    kCheck(obj->status);

    *written = 0; 

    do 
    {        
        result = xkSocket_Send(sockt, (const kByte*)buffer + *written, size - *written);

        if (result >= 0)
        {
            *written += (kSize) result; 
        }
        else
        {
            socketError = xkSocket_GetLastError(); 

            if ((socketError == xkSOCKET_TIMEDOUT) || (socketError == xkSOCKET_WOULD_BLOCK))
            {
                if (!obj->isBlocking)
                {
                    opStatus = kERROR_BUSY;
                }
                else
                {
                    opStatus = kERROR_TIMEOUT;

                    if (obj->socketType == kSOCKET_TYPE_TCP)
                    {
                        obj->status = kERROR_TIMEOUT;
                    }
                }
            }
            else
            {
                opStatus = kERROR_STREAM; 

                if (obj->socketType == kSOCKET_TYPE_TCP)
                {
                    obj->status = kERROR_STREAM; 
                }
            }
        }
    }
    while (obj->isBlocking && !kIsError(opStatus) && (*written < size)); 

    return opStatus;
}

kFx(kStatus) kSocket_WriteTo(kSocket sockt, kIpAddress address, k32u port, const void* buffer, kSize size)
{
    kObj(kSocket, sockt); 
    struct sockaddr_in sockAddr;
    kSSize result; 
    k32s socketError; 
    kStatus opStatus = kOK; 

    kCheck(obj->status);
    kCheckArgs(address.version == kIP_VERSION_4); 

    kCheck(xkIpAddress_ToSockAddr(address, port, &sockAddr));

    result = xkSocket_SendTo(sockt, buffer, size, &sockAddr);

    if (result < 0)
    {
        socketError = xkSocket_GetLastError(); 

        if ((socketError == xkSOCKET_TIMEDOUT) || (socketError == xkSOCKET_WOULD_BLOCK))
        {
            if (!obj->isBlocking)
            {
                opStatus = kERROR_BUSY; 
            }
            else
            {
                opStatus = kERROR_TIMEOUT; 

                if (obj->socketType == kSOCKET_TYPE_TCP)
                {
                    obj->status = kERROR_TIMEOUT;
                }
            }
        }
        else if (socketError == xkSOCKET_MSGSIZE)
        {
            opStatus = kERROR_INCOMPLETE; 
        }
        else
        {
            opStatus = kERROR_STREAM; 

            if (obj->socketType == kSOCKET_TYPE_TCP)
            {
                obj->status = kERROR_STREAM; 
            }
        }
    }
    else if ((kSize)result != size)
    {
        opStatus = kERROR_INCOMPLETE; 
    }   

    return opStatus; 
}

kFx(kStatus) kSocket_EnableBroadcast(kSocket sockt, kBool broadcast)
{
    kObj(kSocket, sockt); 
    kBool value = (broadcast) ? kTRUE : kFALSE;
    
    kCheck(obj->status);

    kCheck(setsockopt(obj->handle, SOL_SOCKET, SO_BROADCAST, (char*)&value, sizeof(kBool)) != xkSOCKET_ERROR); 

    return kOK;   
}

kFx(kStatus) kSocket_SetNoDelay(kSocket sockt, kBool noDelay)
{
    kObj(kSocket, sockt); 
    kBool value = noDelay; 
    
    kCheck(obj->status);

    kCheck(setsockopt(obj->handle, IPPROTO_TCP, TCP_NODELAY, (char*)&value, sizeof(kBool)) != xkSOCKET_ERROR); 

    return kOK;   
}

kFx(kStatus) kSocket_SetLingerTime(kSocket sockt, k64u lingerTime)
{
    kObj(kSocket, sockt); 
    struct linger linger;

    kCheck(obj->status);

    linger.l_onoff = 1;
    linger.l_linger = (unsigned short) ((lingerTime + 999999)/1000000);

    kCheck(setsockopt(obj->handle, SOL_SOCKET, SO_LINGER, (char*) &linger, sizeof(struct linger)) != xkSOCKET_ERROR); 
   
    return kOK;   
}

kFx(kStatus) xkSocket_SetWriteLowWater(kSocket sockt, kSize size)
{
    kObj(kSocket, sockt); 
    int value = (int) size;

    kCheck(obj->status);
        
    kCheck(setsockopt(obj->handle, SOL_SOCKET, SO_SNDLOWAT, (char*)&value, sizeof(int)) != xkSOCKET_ERROR); 
    
    return kOK;
}

kFx(kStatus) xkSocket_SetReadLowWater(kSocket sockt, kSize size)
{
    kObj(kSocket, sockt); 
    int value = (int) kMin_(size, xkSOCKET_MAX_BUFFER); 

    kCheck(obj->status);

    kCheck(setsockopt(obj->handle, SOL_SOCKET, SO_RCVLOWAT, (char*)&value, sizeof(int)) != xkSOCKET_ERROR); 
    
    return kOK;  
}

kFx(kSize) xkSocket_WriteBuffer(kSocket sockt)
{
    kObj(kSocket, sockt); 
    int optValue = 0; 
    socklen_t optLen = sizeof(optValue); 
        
    if (getsockopt(obj->handle, SOL_SOCKET, SO_SNDBUF, (char*)&optValue, &optLen) == xkSOCKET_ERROR)
    {
        return 0; 
    }
    
    return (kSize)optValue; 
}

kFx(kSize) xkSocket_ReadBuffer(kSocket sockt)
{
    kObj(kSocket, sockt); 
    int optValue = 0; 
    socklen_t optLen = sizeof(optValue); 
    
    if (getsockopt(obj->handle, SOL_SOCKET, SO_RCVBUF, (char*)&optValue, &optLen) == xkSOCKET_ERROR)
    {
        return 0; 
    }
    
    return (kSize)optValue; 
}

kFx(kStatus) kSocket_LocalEndPoint(kSocket sockt, kIpEndPoint* endPoint)
{
    kObj(kSocket, sockt); 
    struct sockaddr_in sockAddr;
    socklen_t sockAddrLength = (socklen_t) sizeof(sockAddr);
    
    kCheck(obj->status);

    kCheck(getsockname(obj->handle, (struct sockaddr*)&sockAddr, &sockAddrLength) != xkSOCKET_ERROR);    
    kCheck(xkIpAddress_FromSockAddr(&sockAddr, &endPoint->address, &endPoint->port));
 
    return kOK;
}

kFx(kStatus) kSocket_RemoteEndPoint(kSocket sockt, kIpEndPoint* endPoint)
{
    kObj(kSocket, sockt); 
    struct sockaddr_in sockAddr;
    socklen_t sockAddrLength = (socklen_t) sizeof(sockAddr);
     
    kCheck(obj->status);

    kCheck(getpeername(obj->handle, (struct sockaddr*)&sockAddr, &sockAddrLength) == 0); 
    kCheck(xkIpAddress_FromSockAddr(&sockAddr,&endPoint->address, &endPoint->port));
    
    return kOK;
}

kFx(kStatus) kSocket_Status(kSocket sockt)
{
    kObj(kSocket, sockt); 

    return obj->status; 
}

kFx(kSize) kSocket_Handle(kSocket sockt)
{
    kObj(kSocket, sockt); 

    return (kSize) obj->handle;
}

#if defined(K_WINDOWS)

kFx(kStatus) kSocket_EnableReuseAddress(kSocket sockt, kBool reuse)
{
    kObj(kSocket, sockt); 
    int option = reuse;
    
    kCheck(obj->status);

    //FSS-1243: according to semi-credible internet gossip (and backed up by informal testing) Windows 
    //appears to ignore existing connections in the TIME_WAIT state by default when binding. as such, there 
    //doesn't appear to be any valid reason to use SO_REUSEADDR on Windows with TCP sockets (we don't consider
    //"port stealing" to be valid). for UDP sockets, SO_REUSEADDR is required for broacast/multicast 
    //receive, so it's valid to use
    if (obj->socketType == kSOCKET_TYPE_UDP)
    {
        kCheckState(setsockopt(obj->handle, SOL_SOCKET, SO_REUSEADDR, (char*)&option, sizeof(option)) != xkSOCKET_ERROR);
    }

    return kOK;   
}

kFx(kSSize) xkSocket_Recv(kSocket sockt, void* buffer, kSize size, k32s options)
{
    kObj(kSocket, sockt); 

    return recv(obj->handle, (char*)buffer, (int)kMin_(size, xkSOCKET_MAX_IO_SIZE), options);
}

kFx(kSSize) xkSocket_RecvFrom(kSocket sockt, void* buffer, kSize size, struct sockaddr_in* from)
{
    kObj(kSocket, sockt); 
    int sockAddrLength = sizeof(struct sockaddr_in);

    return recvfrom(obj->handle, (char*)buffer, (int) kMin_(size, xkSOCKET_MAX_IO_SIZE), 0, (struct sockaddr*)from, &sockAddrLength); 
}

kFx(kSSize) xkSocket_RecvMessage(kSocket sockt, void* buffer, kSize size, struct sockaddr_in* from, kSize* interfaceIndex)
{
    kObj(kSocket, sockt);
    GUID WSARecvMsgGUID = WSAID_WSARECVMSG;
    WSACMSGHDR* messageHeader = kNULL;
    LPFN_WSARECVMSG recvMsgFx = kNULL;
    WSABUF iovec[1] = { 0 };
    WSAMSG message = { 0 };
    DWORD bytesRead = 0;
    kChar pktbuf[1024];
    
    *interfaceIndex = kSIZE_NULL;

    if (WSAIoctl(obj->handle, SIO_GET_EXTENSION_FUNCTION_POINTER,
        &WSARecvMsgGUID, sizeof(WSARecvMsgGUID), 
        &recvMsgFx, sizeof(recvMsgFx), 
        &bytesRead, NULL, NULL) != 0)
    {
        return -1;
    }

    from->sin_family = AF_INET;
    from->sin_port = htons(0);

    iovec[0].buf = (kChar*)buffer;
    iovec[0].len = (ULONG)size;

    message.name = (struct sockaddr*)from;
    message.namelen = sizeof(sockaddr_in);
    message.lpBuffers = &iovec[0];
    message.dwBufferCount = 1;
    message.Control.buf = pktbuf;
    message.Control.len = kCountOf(pktbuf);
    message.dwFlags = 0;
    
    if (recvMsgFx(obj->handle, &message, &bytesRead, NULL, NULL) != 0)
    {
        return -1;
    }

    messageHeader = WSA_CMSG_FIRSTHDR(&message);
    if (!kIsNull(messageHeader) && (messageHeader->cmsg_type == IP_PKTINFO))
    {
        IN_PKTINFO* pktInfo = (IN_PKTINFO*)WSA_CMSG_DATA(messageHeader);
        *interfaceIndex = pktInfo->ipi_ifindex;
    }
    
    return bytesRead;
}

kFx(kSSize) xkSocket_Send(kSocket sockt, const void* buffer, kSize size)
{
    kObj(kSocket, sockt); 

    return send(obj->handle, (const char*)buffer, (int) kMin_(size, xkSOCKET_MAX_IO_SIZE), 0); 
}

kFx(kSSize) xkSocket_SendTo(kSocket sockt, const void* buffer, kSize size, const struct sockaddr_in* to)
{
    kObj(kSocket, sockt); 
    int sockAddrLength = sizeof(struct sockaddr_in);

    return sendto(obj->handle, (const char*)buffer, (int) kMin_(size, xkSOCKET_MAX_IO_SIZE), 0, (const struct sockaddr*)to, sockAddrLength);
}

kFx(k32s) xkSocket_GetLastError()
{
    return WSAGetLastError();
}

kFx(kStatus) xkSocket_ShutdownHandle(xkSocketHandle socktHandle)
{
     shutdown(socktHandle, SD_BOTH); 

     return kOK; 
}

kFx(kStatus) xkSocket_CloseHandle(xkSocketHandle socktHandle)
{    
    if (closesocket(socktHandle) != 0)
    {
        return kERROR_NETWORK; 
    }

    return kOK;
}

kFx(kStatus) kSocket_SetBlocking(kSocket sockt, kBool isBlocking)
{
    kObj(kSocket, sockt); 
    unsigned long nonBlocking = (unsigned long) !isBlocking;
       
    kCheck(ioctlsocket(obj->handle, FIONBIO, &nonBlocking) == 0);

    obj->isBlocking = isBlocking; 
    
    return kOK;
}

kFx(kStatus) kSocket_SetWriteBuffer(kSocket sockt, kSize size)
{
    kObj(kSocket, sockt); 
    int value = (int) kMin_(size, xkSOCKET_MAX_BUFFER); 

    kCheck(obj->status);
        
    kCheck(setsockopt(obj->handle, SOL_SOCKET, SO_SNDBUF, (char*)&value, sizeof(int)) != xkSOCKET_ERROR); 
    
    return kOK;
}

kFx(kStatus) kSocket_SetReadBuffer(kSocket sockt, kSize size)
{
    kObj(kSocket, sockt); 
    int value = (int) kMin_(size, xkSOCKET_MAX_BUFFER); 

    kCheck(obj->status);

    kCheck(setsockopt(obj->handle, SOL_SOCKET, SO_RCVBUF, (char*)&value, sizeof(int)) != xkSOCKET_ERROR); 
    
    return kOK;  
}

kFx(kStatus) kSocket_SetWriteTimeout(kSocket sockt, k64u timeout)
{
    kObj(kSocket, sockt); 
    int value = (int) (timeout + 999)/1000;

    kCheck(setsockopt(obj->handle, SOL_SOCKET, SO_SNDTIMEO, (char*)&value, sizeof(int)) != xkSOCKET_ERROR); 

    return kOK;  
} 

kFx(kStatus) kSocket_SetReadTimeout(kSocket sockt, k64u timeout)
{
    kObj(kSocket, sockt); 
    int value = (int) (timeout + 999)/1000;
    
    kCheck(setsockopt(obj->handle, SOL_SOCKET, SO_RCVTIMEO, (char*)&value, sizeof(int)) != xkSOCKET_ERROR); 

    return kOK;  
}

kFx(kStatus) kSocket_EnablePacketInfo(kSocket sockt, kBool enabled)
{
    kObj(kSocket, sockt);
    
    kCheck(obj->status);
    kCheckTrue(obj->socketType == kSOCKET_TYPE_UDP, kERROR_STATE);

    kCheck(setsockopt(obj->handle, IPPROTO_IP, IP_PKTINFO, (char*)&enabled, sizeof(kBool)) != xkSOCKET_ERROR);

    return kOK;
}

kFx(kStatus) kSocket_BindToDevice(kSocket socket, const kChar* interfaceName)
{
    //not supported; has no effect
    return kOK; 
}

#elif defined (K_TI_BIOS)

kFx(kStatus) kSocket_EnableReuseAddress(kSocket sockt, kBool reuse)
{
    kObj(kSocket, sockt); 
    kBool value = reuse; 
    
    kCheck(obj->status);

    kCheckState(setsockopt(obj->handle, SOL_SOCKET, SO_REUSEADDR, (char*)&value, sizeof(kBool)) != xkSOCKET_ERROR);

    return kOK;   
}

kFx(kSSize) xkSocket_Recv(kSocket sockt, void* buffer, kSize size, k32s options)
{
    kObj(kSocket, sockt); 
    return recv(obj->handle, buffer, size, options); 
}

kFx(kSSize) xkSocket_RecvFrom(kSocket sockt, void* buffer, kSize size, struct sockaddr_in* from)
{
    kObj(kSocket, sockt); 
    socklen_t sockAddrLength = sizeof(struct sockaddr_in);

    return recvfrom(obj->handle, buffer, size, 0, (struct sockaddr*)from, &sockAddrLength); 
}

kFx(kSSize) xkSocket_RecvMessage(kSocket sockt, void* buffer, kSize size, struct sockaddr_in* from, kSize* interfaceIndex)
{
    kObj(kSocket, sockt);

    *interfaceIndex = 2;   //convention

    return xkSocket_RecvFrom(sockt, buffer, size, from);
}

kFx(kSSize) xkSocket_Send(kSocket sockt, const void* buffer, kSize size)
{
    kObj(kSocket, sockt); 

    return send(obj->handle, (void*)buffer, size, 0); 
}

kFx(kSSize) xkSocket_SendTo(kSocket sockt, const void* buffer, kSize size, const struct sockaddr_in* to)
{
    kObj(kSocket, sockt); 
    socklen_t sockAddrLength = (socklen_t)sizeof(struct sockaddr_in);

    return sendto(obj->handle, (void*)buffer, size, 0, (struct sockaddr*)to, sockAddrLength);
}

kFx(k32s) xkSocket_GetLastError()
{
    return fdError();
}

kFx(kStatus) xkSocket_ShutdownHandle(xkSocketHandle socktHandle)
{    
    struct linger linger;

    linger.l_onoff = 1;
    linger.l_linger = 0;

    //SYS/BIOS doesn't seem to implement shutdown properly, so we set the linger time to zero to force connection reset
    setsockopt(socktHandle, SOL_SOCKET, SO_LINGER, (char*) &linger, sizeof(struct linger)); 
  
    shutdown(socktHandle, SHUT_RDWR); 

    return kOK; 
}

kFx(kStatus) xkSocket_CloseHandle(xkSocketHandle socktHandle)
{
    fdClose(socktHandle);

    return kOK; 
}

kFx(kStatus) kSocket_SetBlocking(kSocket sockt, kBool isBlocking)
{
    kObj(kSocket, sockt); 
    int blockingOpt = (int) isBlocking; 

    kCheck(setsockopt(obj->handle, SOL_SOCKET, SO_BLOCKING, &blockingOpt, sizeof(blockingOpt)) != xkSOCKET_ERROR);

    obj->isBlocking = isBlocking;
    
    return kOK;     
}

kFx(kStatus) kSocket_SetWriteBuffer(kSocket sockt, kSize size)
{
    kObj(kSocket, sockt); 
    int value = (int) kMin_(size, xkSOCKET_MAX_BUFFER); 

    kCheck(obj->status);

    //using this option with a UDP socket generates "00000.553 mmFree: Double Free" condition in NDK stack (1.93)
    if (obj->socketType != kSOCKET_TYPE_UDP)
    {
        kCheckTrue(setsockopt(obj->handle, SOL_SOCKET, SO_SNDBUF, (char*)&value, sizeof(int)) != xkSOCKET_ERROR, kERROR_OS); 
    }
    
    return kOK;
}

kFx(kStatus) kSocket_SetReadBuffer(kSocket sockt, kSize size)
{
    kObj(kSocket, sockt); 
    int value = (int) kMin_(size, xkSOCKET_MAX_BUFFER); 

    kCheck(obj->status);

    //using this option with a UDP socket generates "00000.553 mmFree: Double Free" condition in NDK stack (1.93)
    if (obj->socketType != kSOCKET_TYPE_UDP)
    {
        kCheckTrue(setsockopt(obj->handle, SOL_SOCKET, SO_RCVBUF, (char*)&value, sizeof(int)) != xkSOCKET_ERROR, kERROR_OS); 
    }
    
    return kOK;  
}

kFx(kStatus) kSocket_SetWriteTimeout(kSocket sockt, k64u timeout)
{
    kObj(kSocket, sockt); 
    struct timeval tv; 

    kCheck(setsockopt(obj->handle, SOL_SOCKET, SO_SNDTIMEO, xkSocket_FormatTimeVal(&tv, timeout), sizeof(tv)) != xkSOCKET_ERROR);
    
    return kOK;  
} 

kFx(kStatus) kSocket_SetReadTimeout(kSocket sockt, k64u timeout)
{
    kObj(kSocket, sockt); 
    struct timeval tv;

    kCheck(setsockopt(obj->handle, SOL_SOCKET, SO_RCVTIMEO, xkSocket_FormatTimeVal(&tv, timeout), sizeof(tv)) != xkSOCKET_ERROR);
    
    return kOK;  
}

kFx(kStatus) kSocket_EnablePacketInfo(kSocket sockt, kBool enabled)
{
    kObj(kSocket, sockt);
    
    return kOK;
}

kFx(kStatus) kSocket_BindToDevice(kSocket socket, const kChar* interfaceName)
{
    //not supported; has no effect
    return kOK; 
}

#elif defined (K_VX_KERNEL)

kFx(kStatus) kSocket_EnableReuseAddress(kSocket sockt, kBool reuse)
{
    kObj(kSocket, sockt); 
    kBool value = reuse; 
    
    kCheck(obj->status);

    kCheckState(setsockopt(obj->handle, SOL_SOCKET, SO_REUSEADDR, (char*)&value, sizeof(kBool)) != xkSOCKET_ERROR);

    return kOK;   
}

kFx(kSSize) xkSocket_Recv(kSocket sockt, void* buffer, kSize size, k32s options)
{
    kObj(kSocket, sockt); 
    return recv(obj->handle, buffer, size, options); 
}

kFx(kSSize) xkSocket_RecvFrom(kSocket sockt, void* buffer, kSize size, struct sockaddr_in* from)
{
    kObj(kSocket, sockt); 
    int sockAddrLength = (int) sizeof(struct sockaddr_in);

    return recvfrom(obj->handle, buffer, size, 0, (struct sockaddr*)from, &sockAddrLength); 
}

kFx(kSSize) xkSocket_RecvMessage(kSocket sockt, void* buffer, kSize size, struct sockaddr_in* from, kSize* interfaceIndex)
{
    kObj(kSocket, sockt);
    struct msghdr message;
    struct iovec iovec[1];
    kChar pktbuf[1024];
    kSSize read = 0;
    struct cmsghdr* cmptr = kNULL;

    *interfaceIndex = kSIZE_NULL;

    iovec[0].iov_base = (kChar*)buffer;
    iovec[0].iov_len = size;

    message.msg_name = from;
    message.msg_namelen = sizeof(struct sockaddr_in);
    message.msg_iov = iovec;
    message.msg_iovlen = 1;
    message.msg_control = pktbuf;
    message.msg_controllen = kCountOf(pktbuf);

    read = recvmsg(obj->handle, &message, 0);

    cmptr = CMSG_FIRSTHDR(&message);
    if (!kIsNull(cmptr) && (cmptr->cmsg_type == IP_PKTINFO))
    {
        struct in_pktinfo* pktInfo = (struct in_pktinfo*)CMSG_DATA(cmptr);
        *interfaceIndex = pktInfo->ipi_ifindex;
    }

    return read;
}

kFx(kSSize) xkSocket_Send(kSocket sockt, const void* buffer, kSize size)
{
    kObj(kSocket, sockt); 

    return send(obj->handle, buffer, size, 0); 
}

kFx(kSSize) xkSocket_SendTo(kSocket sockt, const void* buffer, kSize size, const struct sockaddr_in* to)
{
    kObj(kSocket, sockt); 
    int sockAddrLength = sizeof(struct sockaddr_in);

    return sendto(obj->handle, buffer, size, 0, (const struct sockaddr*)to, sockAddrLength);
}

kFx(k32s) xkSocket_GetLastError()
{
    return errno;
}

kFx(kStatus) xkSocket_ShutdownHandle(xkSocketHandle socktHandle)
{
     shutdown(socktHandle, SHUT_RDWR); 

     return kOK; 
}

kFx(kStatus) xkSocket_CloseHandle(xkSocketHandle socktHandle)
{
    close(socktHandle);

    return kOK; 
}

kFx(kStatus) kSocket_SetBlocking(kSocket sockt, kBool isBlocking)
{
    kObj(kSocket, sockt); 
    int nonBlocking = !isBlocking; 
        
    if (ioctl(obj->handle, FIONBIO, (int)&nonBlocking) == ERROR)
    {
        return kERROR_OS; 
    }
 
    obj->isBlocking = isBlocking;

    return kOK;
}

kFx(kStatus) kSocket_SetWriteBuffer(kSocket sockt, kSize size)
{
    kObj(kSocket, sockt); 
    int value = (int) kMin_(size, xkSOCKET_MAX_BUFFER); 

    kCheck(obj->status);
        
    kCheck(setsockopt(obj->handle, SOL_SOCKET, SO_SNDBUF, (char*)&value, sizeof(int)) != xkSOCKET_ERROR); 
    
    return kOK;
}

kFx(kStatus) kSocket_SetReadBuffer(kSocket sockt, kSize size)
{
    kObj(kSocket, sockt); 
    int value = (int) kMin_(size, xkSOCKET_MAX_BUFFER); 

    kCheck(obj->status);

    kCheck(setsockopt(obj->handle, SOL_SOCKET, SO_RCVBUF, (char*)&value, sizeof(int)) != xkSOCKET_ERROR); 
    
    return kOK;  
}

kFx(kStatus) kSocket_SetWriteTimeout(kSocket sockt, k64u timeout)
{
    /*
    kObj(kSocket, sockt); 
    struct timeval tv;
    int result = setsockopt(obj->handle, SOL_SOCKET, SO_SNDTIMEO, xkSocket_FormatTimeVal(&tv, timeout), sizeof(tv)); 

    kCheck(result != xkSOCKET_ERROR);
    */
    
   /* 
    * TODO: This option doesn't appear to be implemented in VxWorks 6.9; code above compiles, but always return an error.
    * For now, we'll just return kOK; should file a bug report with Wind River, or just remove kSocket timeout feature if not needed.    
    */
    
    return kOK;  
} 

kFx(kStatus) kSocket_SetReadTimeout(kSocket sockt, k64u timeout)
{
    kObj(kSocket, sockt); 
    struct timeval tv;

    kCheck(setsockopt(obj->handle, SOL_SOCKET, SO_RCVTIMEO, xkSocket_FormatTimeVal(&tv, timeout), sizeof(tv)) != xkSOCKET_ERROR);
    
    return kOK;  
}

kFx(kStatus) kSocket_EnablePacketInfo(kSocket sockt, kBool enabled)
{
    kObj(kSocket, sockt);
    
    kCheck(obj->status);
    kCheckTrue(obj->socketType == kSOCKET_TYPE_UDP, kERROR_STATE);

    kCheck(setsockopt(obj->handle, IPPROTO_IP, IP_PKTINFO, (char*)&enabled, sizeof(kBool)) != xkSOCKET_ERROR);

    return kOK;
}

#elif defined (K_POSIX)

kFx(kStatus) kSocket_EnableReuseAddress(kSocket sockt, kBool reuse)
{
    kObj(kSocket, sockt); 
    kBool value = reuse; 
    
    kCheck(obj->status);

    kCheckState(setsockopt(obj->handle, SOL_SOCKET, SO_REUSEADDR, (char*)&value, sizeof(kBool)) != xkSOCKET_ERROR);

    return kOK;   
}

kFx(kSSize) xkSocket_Recv(kSocket sockt, void* buffer, kSize size, k32s options)
{
    kObj(kSocket, sockt); 
    return recv(obj->handle, buffer, size, options); 
}

kFx(kSSize) xkSocket_RecvFrom(kSocket sockt, void* buffer, kSize size, struct sockaddr_in* from)
{
    kObj(kSocket, sockt); 
    unsigned int sockAddrLength = sizeof(struct sockaddr_in);

    return recvfrom(obj->handle, buffer, size, 0, (struct sockaddr*)from, &sockAddrLength); 
}

kFx(kSSize) xkSocket_RecvMessage(kSocket sockt, void* buffer, kSize size, struct sockaddr_in* from, kSize* interfaceIndex)
{
    kObj(kSocket, sockt); 
    struct msghdr message;
    struct iovec iovec[1];
    kChar pktbuf[1024];
    kSSize read = 0;
    struct cmsghdr* cmptr = kNULL;

    *interfaceIndex = kSIZE_NULL;

    iovec[0].iov_base = buffer;
    iovec[0].iov_len = size;

    message.msg_name = from;
    message.msg_namelen = sizeof(struct sockaddr_in);
    message.msg_iov = iovec;
    message.msg_iovlen = 1;
    message.msg_control = pktbuf;
    message.msg_controllen = kCountOf(pktbuf);

    read = recvmsg(obj->handle, &message, 0);

    cmptr = CMSG_FIRSTHDR(&message);
    if (!kIsNull(cmptr) && (cmptr->cmsg_type == IP_PKTINFO))
    {
        struct in_pktinfo* pktInfo = (struct in_pktinfo*)CMSG_DATA(cmptr);
        *interfaceIndex = pktInfo->ipi_ifindex;
    }

    return read;
}

kFx(kSSize) xkSocket_Send(kSocket sockt, const void* buffer, kSize size)
{
    kObj(kSocket, sockt); 

    return send(obj->handle, buffer, size, MSG_NOSIGNAL); 
}

kFx(kSSize) xkSocket_SendTo(kSocket sockt, const void* buffer, kSize size, const struct sockaddr_in* to)
{
    kObj(kSocket, sockt); 
    int sockAddrLength = sizeof(struct sockaddr_in);

#if defined(K_QNX)
    return sendto(obj->handle, buffer, (int)kMin_(size, xkSOCKET_MAX_BUFFER), MSG_DONTROUTE, (const struct sockaddr*)to, sockAddrLength);
#else
    return sendto(obj->handle, buffer, size, MSG_NOSIGNAL, (const struct sockaddr*)to, sockAddrLength);
#endif
}

kFx(k32s) xkSocket_GetLastError()
{
    return errno;
}

kFx(kStatus) xkSocket_ShutdownHandle(xkSocketHandle socktHandle)
{
     shutdown(socktHandle, SHUT_RDWR); 

     return kOK; 
}

kFx(kStatus) xkSocket_CloseHandle(xkSocketHandle socktHandle)
{
    close(socktHandle);

    return kOK; 
}

kFx(kStatus) kSocket_SetBlocking(kSocket sockt, kBool isBlocking)
{
    kObj(kSocket, sockt); 
    int flags = fcntl(obj->handle, F_GETFL);

    if (isBlocking)
    {
        fcntl(obj->handle, F_SETFL, flags &~O_NONBLOCK);
    }
    else
    {
        fcntl(obj->handle, F_SETFL, O_NONBLOCK);
    }

    obj->isBlocking = isBlocking;

    return kOK;
}

kFx(kStatus) kSocket_SetWriteBuffer(kSocket sockt, kSize size)
{
    kObj(kSocket, sockt); 
    int value = (int) kMin_(size, xkSOCKET_MAX_BUFFER); 

    kCheck(obj->status);
        
    kCheck(setsockopt(obj->handle, SOL_SOCKET, SO_SNDBUF, (char*)&value, sizeof(int)) != xkSOCKET_ERROR); 
    
    return kOK;
}

kFx(kStatus) kSocket_SetReadBuffer(kSocket sockt, kSize size)
{
    kObj(kSocket, sockt); 
    int value = (int) kMin_(size, xkSOCKET_MAX_BUFFER); 

    kCheck(obj->status);

    kCheck(setsockopt(obj->handle, SOL_SOCKET, SO_RCVBUF, (char*)&value, sizeof(int)) != xkSOCKET_ERROR); 
    
    return kOK;  
}

kFx(kStatus) kSocket_SetWriteTimeout(kSocket sockt, k64u timeout)
{
    kObj(kSocket, sockt); 
    struct timeval tv; 

    kCheck(setsockopt(obj->handle, SOL_SOCKET, SO_SNDTIMEO, xkSocket_FormatTimeVal(&tv, timeout), sizeof(tv)) != xkSOCKET_ERROR);
    
    return kOK;  
} 

kFx(kStatus) kSocket_SetReadTimeout(kSocket sockt, k64u timeout)
{
    kObj(kSocket, sockt); 
    struct timeval tv;

    kCheck(setsockopt(obj->handle, SOL_SOCKET, SO_RCVTIMEO, xkSocket_FormatTimeVal(&tv, timeout), sizeof(tv)) != xkSOCKET_ERROR);
    
    return kOK;  
}

kFx(kStatus) kSocket_EnablePacketInfo(kSocket sockt, kBool enabled)
{
    kObj(kSocket, sockt);
    
    kCheck(obj->status);
    kCheckTrue(obj->socketType == kSOCKET_TYPE_UDP, kERROR_STATE);

    kCheck(setsockopt(obj->handle, IPPROTO_IP, IP_PKTINFO, (char*)&enabled, sizeof(kBool)) != xkSOCKET_ERROR);

    return kOK;
}

#if !defined(K_DARWIN)

kFx(kStatus) kSocket_BindToDevice(kSocket sockt, const kChar* interfaceName)
{
    kObj(kSocket, sockt); 
    struct ifreq sender;

    kCheckArgs(interfaceName[0] != 0); 

    kMemSet(&sender, 0, sizeof(sender));       
    kStrCopy(sender.ifr_name, kCountOf(sender.ifr_name), interfaceName); 

    kCheck(setsockopt(obj->handle, SOL_SOCKET, SO_BINDTODEVICE, &sender, sizeof(sender)) != xkSOCKET_ERROR);

    return kOK;
}
#endif

#endif


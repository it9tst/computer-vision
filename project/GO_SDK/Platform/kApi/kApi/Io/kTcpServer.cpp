/** 
 * @file    kTcpServer.cpp
 *
 * @internal
 * Copyright (C) 2008-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <kApi/Io/kTcpServer.h>
#include <kApi/Io/kSocket.h>
#include <kApi/Io/kTcpClient.h>

kBeginClassEx(k, kTcpServer)
    kAddPrivateVMethod(kTcpServer, kObject, VRelease)
kEndClassEx()

kFx(kStatus) kTcpServer_Construct(kTcpServer* server, kIpVersion ipVersion, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback(allocator);
    kType type = kTypeOf(kTcpServer); 
    kStatus status; 

    kCheck(kAlloc_GetObject(alloc, type, server)); 

    if (!kSuccess(status = xkTcpServer_Init(*server, type, ipVersion, alloc)))
    {
        kAlloc_FreeRef(alloc, server); 
    }

    return status; 
} 

kFx(kStatus) xkTcpServer_Init(kTcpServer server, kType type, kIpVersion ipVersion, kAlloc allocator)
{
    kObjR(kTcpServer, server);
    kStatus status; 

    kCheck(kObject_Init(server, type, allocator)); 

    obj->socket = kNULL; 
    obj->clientWriteBufferSize = -1; 
    obj->clientReadBufferSize = -1; 

    kTry
    {
        kTest(kSocket_Construct(&obj->socket, ipVersion, kSOCKET_TYPE_TCP, allocator)); 
    }
    kCatch(&status)
    {
        xkTcpServer_VRelease(server); 
        kEndCatch(status); 
    }
    
    return kOK;     
}

kFx(kStatus) xkTcpServer_VRelease(kTcpServer server)
{
    kObj(kTcpServer, server); 

    kCheck(kObject_Destroy(obj->socket));

    kCheck(kObject_VRelease(server)); 

    return kOK;   
}

kFx(kStatus) kTcpServer_SetWriteBuffers(kTcpServer server, kSSize socketSize, kSSize clientSize)
{
    kObj(kTcpServer, server); 

    if (socketSize >= 0)
    {
        //SYS/BIOS (and possibly other systems) does not permit buffer sizes to be set after a 
        //connection is accepted. The correct way to set the buffer size is to set the option on the 
        //server socket and allow the option to be inherited by accepted sockets.
        kCheck(kSocket_SetWriteBuffer(obj->socket, (kSize)socketSize)); 
    }

    obj->clientWriteBufferSize = clientSize; 

    return kOK; 
}

kFx(kStatus) kTcpServer_SetReadBuffers(kTcpServer server, kSSize socketSize, kSSize clientSize)
{
    kObj(kTcpServer, server); 

    if (socketSize >= 0)
    {
        //SYS/BIOS (and possibly other systems) does not permit buffer sizes to be set after a 
        //connection is accepted. The correct way to set the buffer size is to set the option on the 
        //server socket and allow the option to be inherited by accepted sockets. 
        kCheck(kSocket_SetReadBuffer(obj->socket, (kSize)socketSize)); 
    }

    obj->clientReadBufferSize = clientSize; 

    return kOK; 
}

kFx(kStatus) kTcpServer_Listen(kTcpServer server, kIpAddress address, k32u port, kSize backlog)
{
    kObj(kTcpServer, server); 
         
    kCheck(kSocket_Bind(obj->socket, address, port));
    kCheck(kSocket_Listen(obj->socket, backlog));
 
    return kOK;
}
 
kFx(kStatus) kTcpServer_Accept(kTcpServer server, k64u timeout, kTcpClient* client, kAlloc allocator)
{
    kObj(kTcpServer, server); 
    kSocket connection = kNULL; 
    kStatus status;

    *client = kNULL;
    
    kTry
    {
        kTest(kSocket_Wait(obj->socket, timeout)); 
        kTest(kSocket_Accept(obj->socket, &connection, allocator)); 
        
        if (!kIsNull(connection))
        {
            kTest(xkTcpClient_ConstructFromSocket(client, connection, allocator));  
            connection = kNULL; 

            if (obj->clientWriteBufferSize >= 0)
            {
                kTest(kTcpClient_SetWriteBuffers(*client, -1, obj->clientWriteBufferSize));  
            }

            if (obj->clientReadBufferSize >= 0)
            {
                kTest(kTcpClient_SetReadBuffers(*client, -1, obj->clientReadBufferSize));  
            }
        }
    }        
    kCatch(&status)
    {
        kObject_Destroy(connection); 
        kDestroyRef(client); 

        kEndCatch(status); 
    }

    return kOK;
}
 
kFx(kStatus) kTcpServer_EnableReuseAddress(kTcpServer server, kBool reuse)
{
    kObj(kTcpServer, server); 
    
    return kSocket_EnableReuseAddress(obj->socket, reuse);
}

kFx(kSocket) kTcpServer_Socket(kTcpServer server)
{
    kObj(kTcpServer, server); 

    return obj->socket;
}

kFx(kStatus) kTcpServer_LocalEndPoint(kTcpServer server, kIpEndPoint* endPoint)
{
    kObj(kTcpServer, server); 
        
    return kSocket_LocalEndPoint(obj->socket, endPoint);
}

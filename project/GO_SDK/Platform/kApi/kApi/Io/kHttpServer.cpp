/** 
 * @file    kHttpServer.cpp
 *
 * @internal
 * Copyright (C) 2013-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <kApi/Io/kHttpServer.h>
#include <kApi/Data/kList.h>
#include <kApi/Io/kHttpServerChannel.h>
#include <kApi/Io/kTcpServer.h>
#include <kApi/Threads/kThread.h>

kBeginClassEx(k, kHttpServer)
    kAddPrivateVMethod(kHttpServer, kObject, VRelease)
kEndClassEx()

kFx(kStatus) kHttpServer_Construct(kHttpServer* server, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback(allocator);
    kType type = kTypeOf(kHttpServer); 
    kStatus status; 

    kCheck(kAlloc_GetObject(alloc, type, server)); 

    if (!kSuccess(status = xkHttpServer_Init(*server, type, alloc)))
    {
        kAlloc_FreeRef(alloc, server); 
    }

    return status; 
} 

kFx(kStatus) xkHttpServer_Init(kHttpServer server, kType type, kAlloc allocator)
{
    kObjR(kHttpServer, server); 
    
    kCheck(kObject_Init(server, type, allocator)); 

    obj->localEndPoint.address = kIpAddress_AnyV4(); 
    obj->localEndPoint.port = xkHTTP_SERVER_DEFAULT_PORT; 
    obj->channelCapacity = xkHTTP_SERVER_DEFAULT_CHANNEL_CAPACITY; 
    obj->socketWriteBufferSize = xkHTTP_SERVER_DEFAULT_SOCKET_WRITE_BUFFER; 
    obj->clientWriteBufferSize = xkHTTP_SERVER_DEFAULT_CLIENT_WRITE_BUFFER; 
    obj->socketReadBufferSize = xkHTTP_SERVER_DEFAULT_SOCKET_READ_BUFFER; 
    obj->clientReadBufferSize = xkHTTP_SERVER_DEFAULT_CLIENT_READ_BUFFER; 
    obj->handler.function = kNULL;
    obj->handler.receiver = kNULL;
    obj->listener = kNULL;
    obj->thread = kNULL;
    obj->shouldQuit = 0;
    obj->channels = kNULL;
    obj->backlog = xkHTTP_SERVER_DEFAULT_LISTEN_BACKLOG;

    return kOK;     
}

kFx(kStatus) xkHttpServer_VRelease(kHttpServer server)
{
    kHttpServer_Stop(server); 

    kCheck(kObject_VRelease(server)); 

    return kOK;   
}

kFx(kStatus) kHttpServer_SetAddress(kHttpServer server, kIpAddress address)
{
    kObj(kHttpServer, server); 

    kCheckState(kIsNull(obj->thread)); 

    obj->localEndPoint.address = address; 

    return kOK; 
}

kFx(kStatus) kHttpServer_SetPort(kHttpServer server, k32u port)
{
    kObj(kHttpServer, server); 

    kCheckState(kIsNull(obj->thread)); 

    obj->localEndPoint.port = port; 

    return kOK; 
}

kFx(kStatus) kHttpServer_SetMaxConnections(kHttpServer server, kSize capacity)
{
    kObj(kHttpServer, server); 

    kCheckState(kIsNull(obj->thread)); 

    obj->channelCapacity = capacity; 

    return kOK; 
}

kFx(kStatus) kHttpServer_SetBacklog(kHttpServer server, kSize backlog)
{
    kObj(kHttpServer, server);

    kCheckState(kIsNull(obj->thread));

    obj->backlog = backlog;

    return kOK;
}

kFx(kStatus) kHttpServer_SetWriteBuffers(kHttpServer server, kSSize socketSize, kSSize clientSize)
{
    kObj(kHttpServer, server); 

    kCheckState(kIsNull(obj->thread)); 

    if (socketSize >= 0)
    {
        obj->socketWriteBufferSize = socketSize; 
    }

    if (clientSize >= 0)
    {
        obj->clientWriteBufferSize = clientSize; 
    }

    return kOK; 
}

kFx(kStatus) kHttpServer_SetReadBuffers(kHttpServer server, kSSize socketSize, kSSize clientSize)
{
    kObj(kHttpServer, server); 

    kCheckState(kIsNull(obj->thread)); 

    if (socketSize >= 0)
    {
        obj->socketReadBufferSize = socketSize; 
    }

    if (clientSize >= 0)
    {
        obj->clientReadBufferSize = clientSize; 
    }

    return kOK; 
}

kFx(kStatus) kHttpServer_SetHandler(kHttpServer server, kCallbackFx function, kPointer receiver)
{
    kObj(kHttpServer, server); 

    kCheckState(kIsNull(obj->thread)); 

    obj->handler.function = function; 
    obj->handler.receiver = receiver; 

    return kOK; 
}

kFx(kStatus) kHttpServer_Start(kHttpServer server)
{
    kObj(kHttpServer, server); 
    kStatus status; 

    kCheckState(kIsNull(obj->thread)); 
    kCheckState(!kIsNull(obj->handler.function)); 

    kAtomic32s_Exchange(&obj->shouldQuit, kFALSE); 

    kTry
    {
        kTest(kList_Construct(&obj->channels, kTypeOf(kObject), obj->channelCapacity, kObject_Alloc(server))); 

        //configure server socket
        kTest(kTcpServer_Construct(&obj->listener, obj->localEndPoint.address.version, kObject_Alloc(server))); 

        kTest(kTcpServer_SetReadBuffers(obj->listener, obj->socketReadBufferSize, obj->clientReadBufferSize)); 
        kTest(kTcpServer_SetWriteBuffers(obj->listener, obj->socketWriteBufferSize, obj->clientWriteBufferSize)); 
        kTest(kTcpServer_EnableReuseAddress(obj->listener, kTRUE)); 

        kTest(kTcpServer_Listen(obj->listener, obj->localEndPoint.address, obj->localEndPoint.port, obj->backlog));

        //start thread to accept incoming connections
        kTest(kThread_Construct(&obj->thread, kObject_Alloc(server))); 
        kTest(kThread_StartEx(obj->thread, xkHttpServer_ThreadEntry, server, 0, "kHttpServer", 0)); 
    }
    kCatch(&status)
    {
        kHttpServer_Stop(server); 
        kEndCatch(status); 
    }
        
    return kOK; 
}

kFx(kStatus) kHttpServer_Stop(kHttpServer server)
{
    kObj(kHttpServer, server); 

    if (!kIsNull(obj->thread))
    {
        kAtomic32s_Exchange(&obj->shouldQuit, kTRUE); 
        kCheck(kDestroyRef(&obj->thread)); 
    }

    kCheck(kDestroyRef(&obj->listener)); 
    kCheck(kDisposeRef(&obj->channels));

    return kOK; 
}

kFx(kStatus) xkHttpServer_ThreadEntry(kHttpServer server)
{
    kObj(kHttpServer, server); 

    //run until the server is stopped
    while (!kAtomic32s_Get(&obj->shouldQuit))
    {
        kTcpClient client = kNULL; 
        kHttpServerChannel channel = kNULL; 

        //wait to accept a new connection; the wait time is limited to xkHTTP_SERVER_QUIT_QUERY_PERIOD, so that the 
        //'shouldQuit' flag is periodically polled
        if (kSuccess(kTcpServer_Accept(obj->listener, xkHTTP_SERVER_QUIT_QUERY_PERIOD, &client, kObject_Alloc(server))))
        {
            if (!kIsNull(client))
            {
                //construct a new channel object to represent the connection, and add it to the list of active channels
                if (!kSuccess(xkHttpServerChannel_Construct(&channel, server, client, kObject_Alloc(server))))
                {
                    kObject_Destroy(client);                      
                }
                else
                {
                    xkHttpServer_AddChannel(server, channel); 
                }
            }
        }
    }

    return kOK;
}

kFx(kStatus) xkHttpServer_AddChannel(kHttpServer server, kHttpServerChannel channel)
{
    kObj(kHttpServer, server); 
    kStatus status; 

    //add new channel to channel list
    if (!kSuccess(status = kList_AddT(obj->channels, &channel, kNULL)))
    {
        kObject_Destroy(channel); 
        return status; 
    }

    //remove any closed channels, or channels in excess of max connections
    kCheck(xkHttpServer_CullChannels(server)); 

    return kOK; 
}

kFx(kStatus) xkHttpServer_CullChannels(kHttpServer server)
{
    kObj(kHttpServer, server); 
    kListItem channelIt = 0; 

    //remove any inactive channels
    channelIt = kList_First(obj->channels); 
    while (!kIsNull(channelIt))
    {
        kListItem current = channelIt; 
        kHttpServerChannel channel = kList_AsT(obj->channels, current, kObject); 

        channelIt = kList_Next(obj->channels, channelIt); 

        if (xkHttpServerChannel_IsClosed(channel))
        {
            kCheck(kList_Remove(obj->channels, current)); 
            kCheck(kObject_Destroy(channel)); 
        }
    }

    //remove oldest remaining channels until less than capacity
    while (kList_Count(obj->channels) > obj->channelCapacity)
    {      
        kListItem current = kList_First(obj->channels); 
        kHttpServerChannel channel = kList_AsT(obj->channels, current, kObject); 

        kCheck(kList_Remove(obj->channels, current)); 
        kCheck(kObject_Destroy(channel)); 
    }

    return kOK; 
}

kFx(kStatus) kHttpServer_LocalEndPoint(kHttpServer server, kIpEndPoint* endPoint)
{
    kObj(kHttpServer, server); 

    kCheckState(!kIsNull(obj->listener)); 

    kCheck(kTcpServer_LocalEndPoint(obj->listener, endPoint)); 
    
    return kOK; 
}

kFx(kCallback) xkHttpServer_Handler(kHttpServer server)
{
    kObj(kHttpServer, server); 

    return obj->handler; 
}

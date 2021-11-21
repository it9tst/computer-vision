/** 
 * @file    kHttpServerChannel.cpp
 *
 * @internal
 * Copyright (C) 2013-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <kApi/Io/kHttpServerChannel.h>
#include <kApi/Io/kHttpServer.h>
#include <kApi/Io/kHttpServerRequest.h>
#include <kApi/Io/kHttpServerResponse.h>
#include <kApi/Io/kTcpClient.h>
#include <kApi/Io/kWebSocket.h>
#include <kApi/Threads/kLock.h>
#include <kApi/Threads/kThread.h>

kBeginClassEx(k, kHttpServerChannel)
    kAddPrivateVMethod(kHttpServerChannel, kObject, VRelease)
kEndClassEx()

kFx(kStatus) xkHttpServerChannel_Construct(kHttpServerChannel* channel, kHttpServer server, kTcpClient client, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback(allocator);
    kType type = kTypeOf(kHttpServerChannel); 
    kStatus status; 

    kCheck(kAlloc_GetObject(alloc, type, channel)); 

    if (!kSuccess(status = xkHttpServerChannel_Init(*channel, type, server, client, alloc)))
    {
        kAlloc_FreeRef(alloc, channel); 
    }

    return status; 
} 

kFx(kStatus) xkHttpServerChannel_Init(kHttpServerChannel channel, kType type, kHttpServer server, kTcpClient client, kAlloc allocator)
{
    kObjR(kHttpServerChannel, channel); 
    kStatus status; 

    kCheck(kObject_Init(channel, type, allocator)); 

    obj->server = server; 
    obj->client = client; 
    obj->clientLock = kNULL;
    obj->thread = kNULL;
    obj->request = kNULL;
    obj->response = kNULL;
    
    kTry
    {
        kTest(kLock_Construct(&obj->clientLock, allocator)); 

        kTest(xkHttpServerRequest_Construct(&obj->request, channel, allocator)); 
        kTest(xkHttpServerResponse_Construct(&obj->response, channel, allocator)); 

        //start thread to process incoming http requests
        kTest(kThread_Construct(&obj->thread, allocator)); 
        kTest(kThread_StartEx(obj->thread, xkHttpServerChannel_ThreadEntry, channel, 0, "kHttpServerChannel", 0)); 
    }
    kCatch(&status)
    {
        xkHttpServerChannel_VRelease(channel); 
        kEndCatch(status); 
    }

    return kOK;     
}

kFx(kStatus) xkHttpServerChannel_VRelease(kHttpServerChannel channel)
{
    kObj(kHttpServerChannel, channel); 

    //abort any ongoing communication operations
    kCheck(xkHttpServerChannel_CancelClient(channel)); 

    kCheck(kObject_Destroy(obj->thread)); 
    kCheck(kObject_Destroy(obj->client)); 
    kCheck(kObject_Destroy(obj->request)); 
    kCheck(kObject_Destroy(obj->response)); 
    kCheck(kObject_Destroy(obj->clientLock)); 

    kCheck(kObject_VRelease(channel)); 

    return kOK;   
}

kFx(kStatus) xkHttpServerChannel_ThreadEntry(kHttpServerChannel channel)
{
    kObj(kHttpServerChannel, channel); 
    kStatus result; 

    //process messages until: 
    //a) the user-provided callback requests close ('connection: closed'), in which case obj->client will be null
    //b) the channel stream is detached by the user-provided callback, in which case obj->client will be null
    //c) the channel stream is closed unexpectedly by the client, in which case 'result' will most likely be kERROR_CLOSED
    //d) the channel stream is cancelled by the server (parent), in which case 'result' will most likely be kERROR_ABORT
    //e) an unexpected/unhandled error occurs  
    do
    {
        result = xkHttpServerChannel_ProcessMessage(channel); 
    }
    while (kSuccess(result) && !kIsNull(obj->client)); 

    //ensure that the channel stream is closed, if it isn't already
    kCheck(xkHttpServerChannel_DestroyClient(channel)); 

    return result;   
}

kFx(kStatus) xkHttpServerChannel_ProcessMessage(kHttpServerChannel channel)
{
    kObj(kHttpServerChannel, channel); 
    kCallback handler = xkHttpServer_Handler(obj->server); 
    kStatus status = kOK; 

    //process request line and headers
    kCheck(xkHttpServerRequest_Begin(obj->request));    

    //if requested, automatically send a 100-continue message
    if (xkHttpServerRequest_ExpectsContinue(obj->request))
    {
        kCheck(xkHttpServerChannel_SendContinue(channel));    
    }

    //initialize response 
    kCheck(xkHttpServerResponse_Begin(obj->response)); 

    //invoke user callback to perform message processing
    status = handler.function(handler.receiver, obj->server, channel);

    //if the stream wasn't detached by the user callback...
    if (!kIsNull(obj->client))
    {
        if (kSuccess(status))
        {
            //if processing completed normally, finalize the response
            kCheck(xkHttpServerResponse_End(obj->response)); 
        }
        else if (!xkHttpServerResponse_MessageStarted(obj->response) && kSuccess(kTcpClient_Status(obj->client)))
        {
            //if an unhandled processing error occurred, the stream is still valid, and response headers haven't 
            //already been transmitted, discard any partially-formatted response and send an 'internal server error' 
            //message instead
            kCheck(xkHttpServerResponse_Begin(obj->response)); 
            kCheck(kHttpServerResponse_SetStatus(obj->response, kHTTP_STATUS_INTERNAL_SERVER_ERROR)); 
            kCheck(kHttpServerResponse_SetClosed(obj->response, kTRUE)); 
            kCheck(xkHttpServerResponse_End(obj->response)); 
        }

        //if the response included a 'connection: closed' header, close the tcp connection here
        if (xkHttpServerResponse_Closed(obj->response))
        {
            kCheck(xkHttpServerChannel_DestroyClient(channel)); 
        }
    }

    return status; 
}

kFx(kStatus) xkHttpServerChannel_SendContinue(kHttpServerChannel channel)
{
    kObj(kHttpServerChannel, channel); 

    kCheck(xkHttpServerResponse_Begin(obj->response)); 
    kCheck(kHttpServerResponse_SetStatus(obj->response, kHTTP_STATUS_CONTINUE)); 
    kCheck(xkHttpServerResponse_End(obj->response)); 

    return kOK; 
}

kFx(kHttpServerRequest) kHttpServerChannel_Request(kHttpServerChannel channel)
{
    kObj(kHttpServerChannel, channel); 

    return obj->request; 
}

kFx(kHttpServerResponse) kHttpServerChannel_Response(kHttpServerChannel channel)
{
    kObj(kHttpServerChannel, channel); 

    return obj->response; 
}

kFx(kStatus) xkHttpServerChannel_CancelClient(kHttpServerChannel channel)
{
    kObj(kHttpServerChannel, channel); 

    //check client lock for null because this function can be called during init failure clean-up
    if (!kIsNull(obj->clientLock))
    {
        kLock_Enter(obj->clientLock); 
        {
            if (!kIsNull(obj->client))
            {
                kTcpClient_Cancel(obj->client); 
            }           
        }
        kLock_Exit(obj->clientLock); 
    }

    return kOK;   
}

kFx(kStatus) xkHttpServerChannel_DestroyClient(kHttpServerChannel channel)
{
    kObj(kHttpServerChannel, channel); 

    kLock_Enter(obj->clientLock); 
    {
        kDestroyRef(&obj->client); 
    }
    kLock_Exit(obj->clientLock); 

    return kOK;   
}

kFx(kStatus) kHttpServerChannel_DetachClient(kHttpServerChannel channel, kTcpClient* client)
{
    kObj(kHttpServerChannel, channel); 

    //finalize the outgoing response before surrendering control of the client
    kCheck(xkHttpServerResponse_End(obj->response)); 

    kLock_Enter(obj->clientLock); 
    {
        *client = obj->client; 
        obj->client = kNULL; 
    }
    kLock_Exit(obj->clientLock); 

    return kOK;   
}

kFx(kBool) xkHttpServerChannel_IsClosed(kHttpServerChannel channel)
{
    kObj(kHttpServerChannel, channel); 

    return kSuccess(kThread_Join(obj->thread, 0, kNULL)); 
}

kFx(kTcpClient) xkHttpServerChannel_Client(kHttpServerChannel channel)
{
    kObj(kHttpServerChannel, channel); 

    return obj->client; 
}

kFx(kStatus) xkHttpServerChannel_NormalizeHeaderCaps(kChar* str)
{    
    if (*str != 0)
    {        
        *str = kChar_ToUpper(*str); 

        while (!kIsNull(str = strchr(str, '-')))
        {
            str++; 
                
            if (*str != 0)
            {        
                *str = kChar_ToUpper(*str); 
            }
        }
    }

    return kOK; 
}

kFx(kStatus) kHttpServerChannel_ProcessWebSocketRequest(kHttpServerChannel channel, kBool shouldAccept, kWebSocket* webSocket)
{
    kObj(kHttpServerChannel, channel); 
    kHttpServerResponse request = kHttpServerChannel_Request(channel);
    kHttpServerResponse response = kHttpServerChannel_Response(channel);
    const kChar* headerValue = kNULL;
    k32s version;

    if (kIsNull(headerValue = kHttpServerRequest_FindHeaderValue(request, "Sec-WebSocket-Version")) || 
        !kSuccess(k32s_Parse(&version, headerValue)))
    {
        kCheck(kHttpServerResponse_SetStatus(response, kHTTP_STATUS_BAD_REQUEST));
        kCheck(kHttpServerResponse_SetClosed(response, kTRUE));

        return kOK;
    }
        
    if (version != xkWEB_SOCKET_PROTOCOL_VERSION)
    {
        kCheck(kHttpServerResponse_SetStatus(response, kHTTP_STATUS_UPGRADE_REQUIRED));
        kCheck(kHttpServerResponse_SetHeader(response, " Sec-WebSocket-Version", "13"));

        return kOK;
    }

    if (kIsNull(headerValue = kHttpServerRequest_FindHeaderValue(request, "Sec-WebSocket-Key")))
    {
        kCheck(kHttpServerResponse_SetStatus(response, kHTTP_STATUS_BAD_REQUEST));
        kCheck(kHttpServerResponse_SetClosed(response, kTRUE));

        return kOK;
    }

    if (shouldAccept)
    {
        return xkWebSocket_ConstructFromRequest(webSocket, channel, kNULL);
    }
    else
    {
        kCheck(kHttpServerResponse_SetStatus(obj->response, kHTTP_STATUS_BAD_REQUEST));
        kCheck(kHttpServerResponse_SetClosed(obj->response, kTRUE));
    }

    return kOK;
}

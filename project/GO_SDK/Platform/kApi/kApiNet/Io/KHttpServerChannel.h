// 
// KHttpServerChannel.h
// 
// Copyright (C) 2014-2021 by LMI Technologies Inc.
// Licensed under the MIT License.
// Redistributed files must retain the above copyright notice.
// 
#ifndef K_API_NET_HTTP_SERVER_CHANNEL_H
#define K_API_NET_HTTP_SERVER_CHANNEL_H

#include <kApi/Io/kHttpServerChannel.h>
#include "kApiNet/Io/KHttpServerRequest.h"
#include "kApiNet/Io/KHttpServerResponse.h"
#include "kApiNet/Io/KTcpClient.h"
#include "kApiNet/Io/KWebSocket.h"
#include "kApiNet/KAlloc.h"

namespace Lmi3d
{
    namespace Zen 
    {
        namespace Io
        {
            /// <summary>Represents the server side of an HTTP connection.</summary>
            ///
            /// <remarks>
            /// <para>Default KRefStyle: None</para>
            /// </remarks>
            public ref class KHttpServerChannel : public KObject
            {
                KDeclareNoneClass(KHttpServerChannel, kHttpServerChannel)

            public:
                /// <summary>Initializes a new instance of the KHttpServerChannel class with the specified Zen object handle.</summary>           
                /// <param name="handle">Zen object handle.</param>
                KHttpServerChannel(IntPtr handle)
                    : KObject(handle, DefaultRefStyle)
                {}

                /// <summary>Gets a KHttpServerRequest object that can be used to parse an incoming request.</summary>
                property KHttpServerRequest^ Request
                {
                    KHttpServerRequest^ get() { return gcnew KHttpServerRequest(IntPtr(kHttpServerChannel_Request(Handle))); }                    
                }

                /// <summary>Gets a KHttpServerResponse object that can be used to format an outgoing response.</summary>
                property KHttpServerResponse^ Response
                {
                    KHttpServerResponse^ get() { return gcnew KHttpServerResponse(IntPtr(kHttpServerChannel_Response(Handle))); }
                }

                /// <summary>Transfers ownership of the underlying TCP client object associated with this channel.</summary>
                /// 
                /// <remarks>
                /// <para>Use this method to assume control of the channel's TCP client object. This function should only be called
                /// after formatting an HTTP response, just prior to returning from a request processing callback.</para>
                /// 
                /// <para>The primary purpose of this method is to support the WebSocket protocol. If a WebSocket request is
                /// received, the server can format a 101-switching-protocols response, then call this method to take
                /// control of the client for subsequent communication.</para>
                /// 
                /// <para>Use KObject.Dispose to free the TCP client object when it is no longer needed.</para>
                /// </remarks>
                /// 
                /// <returns>TCP client object.</returns>
                KTcpClient^ DetachClient()
                {
                    kTcpClient client; 

                    KCheck(kHttpServerChannel_DetachClient(Handle, &client)); 

                    return gcnew KTcpClient(IntPtr(client)); 
                }

                /// <summary>Processes a websocket request.</summary>
                /// 
                /// <remarks>
                /// <para>This function encapsulates WebSocket HTTP handshaking. Note that a WebSocket object 
                /// will not be always be created, even in the event of success(e.g., if the upgrade response
                /// indicates that the client should request a different version). </para>
                /// 
                /// <para>In any case, a response will be generated and the HTTP server channel object should not be 
                /// used again after this function returns.</para>
                /// 
                /// </remarks>
                /// 
                /// <returns>Websocket client object.</returns>
                KWebSocket^ ProcessWebSocketRequest(kBool shouldAccept)
                {
                    kWebSocket webSocket;

                    KCheck(kHttpServerChannel_ProcessWebSocketRequest(Handle, shouldAccept, &webSocket));

                    return kIsNull(webSocket) ? nullptr : gcnew KWebSocket(IntPtr(webSocket));
                }

            protected:
                KHttpServerChannel() : KObject(DefaultRefStyle) {}
            };
        }
    }
}

#endif

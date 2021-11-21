// 
// KHttpServer.h
// 
// Copyright (C) 2014-2021 by LMI Technologies Inc.
// Licensed under the MIT License.
// Redistributed files must retain the above copyright notice.
// 
#ifndef K_API_NET_HTTP_SERVER_H
#define K_API_NET_HTTP_SERVER_H

#include <kApi/Io/kHttpServer.h>
#include "kApiNet/KAlloc.h"
#include "kApiNet/Io/KNetwork.h"
#include "kApiNet/Io/KHttpServerChannel.h"

namespace Lmi3d
{
    namespace Zen 
    {
        namespace Io
        {
     
            /// <summary>Implements a simple HTTP server. <para/> Requires manual disposal.</summary>
            /// 
            /// <remarks>
            /// <para>The KHttpServer class implements a simple HTTP 1.1 server framework. The Start method 
            /// accepts a delegate to process remote requests. When a request is received, this delegate is invoked 
            /// with an 'args' parameter that contains a KHttpServerChannel instance, which, in turn, can be used to 
            /// access KHttpServerRequest and KHttpServerResponse objects. These request/response objects can be used 
            /// to parse an incoming request and to format an outgoing response, respectively.</para>
            ///
            /// <para>Requests are processed asynchronously and the request processing delegate should be prepared to 
            /// handle multiple simultaneous requests. Use to Stop method to shut down the server and cease receiving 
            /// requests.</para>
            /// 
            /// <para>KHttpServer uses a simple thread-per-connection model, where the maximum number of simultaneous connections
            /// can be configured using the SetMaxConnections method. Accordingly, KHttpServer is better suited
            /// to small/embedded applications than large-scale web applications.</para>
            /// </remarks>
            public ref class KHttpServer : public KObject
            {
                KDeclareClass(KHttpServer, kHttpServer)

            public:
                /// <summary>Initializes a new instance of the KHttpServer class with the specified Zen object handle.</summary>           
                /// <param name="handle">Zen object handle.</param>
                KHttpServer(IntPtr handle)
                    : KObject(handle, DefaultRefStyle)
                {}

                /// <summary>Initializes a new instance of the KHttpServer class.</summary>           
                KHttpServer()
                     : KObject(DefaultRefStyle)
                {
                    kHttpServer handle = kNULL;

                    KCheck(kHttpServer_Construct(&handle, kNULL));

                    Handle = handle;
                }

                /// <inheritdoc cref="KHttpServer()" />
                /// <param name="allocator">Memory allocator</param>
                KHttpServer(KAlloc^ allocator)
                     : KObject(DefaultRefStyle)
                {
                    kHttpServer handle = kNULL;

                    KCheck(kHttpServer_Construct(&handle, KToHandle(allocator))); 

                    Handle = handle;
                }
                         
                /// <summary>Starts the server.</summary>
                ///
                /// <remarks>
                /// The registered handler must be unregistered when no longer needed, else the underlying server
                /// will retain a reference to it, possibly resulting in a CLR object leak. The handler will be automatically 
                /// unregistered if KHttpServer.Stop is used to stop the server, or if KObject.Dispose 
                /// is used to destroy the server.
                /// </remarks>
                ///
                /// <param name="address">Local IP address to which the server should bind.</param>
                /// <param name="port">Local port number to which the server should bind.</param>
                /// <param name="capacity">Maximum simultaneous connection count.</param>
                /// <param name="handler">HTTP request handler.</param> 
                void Start(KIpAddress address, k32s port, k32s capacity, Action<KHttpServer^, KHttpServerChannel^>^ handler)
                {
                    kStatus status; 

                    KCheckArgs(handler != nullptr); 

                    if (!kIsNull(xkHttpServer_Handler(Handle).function))
                    {
                        throw gcnew KException(kERROR_STATE);
                    }

                    KCheck(kHttpServer_SetAddress(Handle, (kIpAddress)address)); 
                    KCheck(kHttpServer_SetPort(Handle, (k32u)port));
                    KCheck(kHttpServer_SetMaxConnections(Handle, (k32s)capacity)); 

                    SetHandler(handler); 

                    if (!kSuccess(status = kHttpServer_Start(Handle)))
                    {
                        ClearHandler(); 
                        throw gcnew KException(status);
                    }
                }

                /// <summary>Stops the server.</summary>
                /// 
                /// <remarks>
                /// All asynchronous activities performed by the server will be stopped before this method returns.
                /// </remarks>
                void Stop()
                {
                    KCheck(kHttpServer_Stop(Handle));

                    ClearHandler();
                }

                /// <summary>Gets the local end-point for a running server.</summary>
                property KIpEndPoint LocalEndPoint
                {
                    KIpEndPoint get()
                    {
                        KIpEndPoint endPoint;

                        KCheck(kHttpServer_LocalEndPoint(Handle, (kIpEndPoint*)&endPoint)); 

                        return endPoint;
                    }
                }

            protected:

                //ensures that callbacks are unhooked
                virtual void OnDisposing() override
                {
                    Stop();
                }

            private:
                void SetHandler(Action<KHttpServer^, KHttpServerChannel^>^ handler)
                {
                    KCallbackFx^ thunk = gcnew KCallbackFx(this, &KHttpServer::OnRequest); 
                    KCallbackState^ context = gcnew KCallbackState(thunk, handler);

                    KCheck(kHttpServer_SetHandler(Handle, (kCallbackFx)context->NativeFunction, context->NativeContext));
                }

                void ClearHandler()
                {
                    kCallback nativeHandler = xkHttpServer_Handler(Handle); 

                    KCheck(kHttpServer_SetHandler(Handle, kNULL, kNULL));

                    KCallbackState::Dispose(nativeHandler.receiver);
                }

                kStatus OnRequest(kPointer receiver, kPointer sender, kPointer args)
                {
                    kStatus status = kOK;

                    try
                    {
                        KCallbackState^ context = KCallbackState::FromNativeContext(receiver);
                        Action<KHttpServer^, KHttpServerChannel^>^ handler = (Action<KHttpServer^, KHttpServerChannel^>^) context->Handler;
                        KHttpServerChannel^ channel = gcnew KHttpServerChannel(IntPtr(args));
                        KHttpServer^ server = gcnew KHttpServer(IntPtr(sender));

                        handler(server, channel);
                    }
                    catch (KException^ e)
                    {
                        status = e->Status;
                    }
                    catch (...)
                    {
                        status = kERROR;
                    }

                    return status;
                }

            };
        }
    }
}

#endif

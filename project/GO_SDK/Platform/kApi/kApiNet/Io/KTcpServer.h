// 
// KTcpServer.h
// 
// Copyright (C) 2014-2021 by LMI Technologies Inc.
// Licensed under the MIT License.
// Redistributed files must retain the above copyright notice.
// 
#ifndef K_API_NET_TCP_SERVER_H
#define K_API_NET_TCP_SERVER_H

#include <kApi/Io/kTcpServer.h>
#include "kApiNet/KAlloc.h"
#include "kApiNet/Io/KTcpClient.h"

namespace Lmi3d
{
    namespace Zen 
    {
        namespace Io
        {
            /// <summary>Represents a TCP server. <para/> Requires manual disposal.</summary>
            public ref class KTcpServer : public KObject
            {
                KDeclareClass(KTcpServer, kTcpServer)

            public:
                /// <summary>Initializes a new instance of the KTcpServer class with the specified Zen object handle.</summary>           
                /// <param name="handle">Zen object handle.</param>
                KTcpServer(IntPtr handle)
                    : KObject(handle, DefaultRefStyle)
                {}

                /// <summary>Initializes a new instance of the KTcpServer class.</summary>           
                /// 
                /// <param name="ipVersion">Internet Protocol version.</param>
                KTcpServer(KIpVersion ipVersion)
                    :KObject(DefaultRefStyle)
                {
                    kTcpServer handle = kNULL;

                    KCheck(kTcpServer_Construct(&handle, ipVersion, kNULL));

                    Handle = handle;
                }

                /// <inheritdoc cref="KTcpServer(KIpVersion)" />
                /// <param name="allocator">Memory allocator</param>
                KTcpServer(KIpVersion ipVersion, KAlloc^ allocator)
                    :KObject(DefaultRefStyle)
                {
                    kTcpServer handle = kNULL;

                    KCheck(kTcpServer_Construct(&handle, ipVersion, KToHandle(allocator)));

                    Handle = handle;
                }

                /// <summary>Sets the size of the client write buffer for accepted clients.</summary>
                /// 
                /// <remarks>
                /// <para>Client buffers improve the efficiency of the client when performing several small write operations.</para>
                /// 
                /// <para>By default, the client write buffer size is zero.</para>
                /// </remarks>
                property k64s ClientWriteBufferSize
                {
                    //add get when available

                    void set(k64s size)
                    {
                        KCheck(kTcpServer_SetWriteBuffers(Handle, -1, (kSize)size));
                    }
                }

                /// <summary>Sets the size of the client read buffer for accepted clients.</summary>
                /// 
                /// <remarks>
                /// <para>Client buffers improve the efficiency of the client when performing several small read operations.</para>
                /// 
                /// <para>By default, the client read buffer size is zero.</para>
                /// </remarks>
                property k64s ClientReadBufferSize
                {
                    //add get when available

                    void set(k64s size)
                    {
                        KCheck(kTcpServer_SetReadBuffers(Handle, -1, (kSize)size));
                    }
                }

                /// <summary>Sets the size of the underlying socket write buffer for accepted clients.</summary>
                /// 
                /// <remarks>
                /// <para>Socket buffers decouple the sender and receiver, so that the sender does not need to block
                /// while waiting for the receiver to receive all bytes.</para>
                /// 
                /// <para>The default socket write buffer size is determined by the underlying operating system.</para>
                /// </remarks>
                property k64s SocketWriteBufferSize
                {
                    //add get when available

                    void set(k64s size)
                    {
                        KCheck(kTcpServer_SetWriteBuffers(Handle, (kSize)size, -1));
                    }
                }

                /// <summary>Sets the size of the underlying socket read buffer for accepted clients.</summary>
                /// 
                /// <remarks>
                /// <para>Socket buffers decouple the sender and receiver, so that the sender does not need to block
                /// while waiting for the receiver to receive all bytes.</para>
                /// 
                /// <para>The default socket read buffer size is determined by the underlying operating system.</para>
                /// </remarks>
                property k64s SocketReadBufferSize
                {
                    //add get when available

                    void set(k64s size)
                    {
                        KCheck(kTcpServer_SetReadBuffers(Handle, (kSize)size, -1));
                    }
                }

                /// <summary>Sets a value that determines whether a local end point can be reused within a short period of time.</summary>
                /// 
                /// <remarks>
                /// The option is typically used to allow a server to rebind to a local end point
                /// while a previous socket with the same local end point is in the TIME_WAIT state.
                /// This can be useful when a server must be stopped and started within a brief interval,
                /// but there is a small risk that packets with identical source/destination information
                /// could be misdirected to the new socket.
                /// </remarks>
                property bool AddressReuseEnabled
                {
                    //add get when available

                    void set(bool enabled)
                    {
                        KCheck(kTcpServer_EnableReuseAddress(Handle, enabled));
                    }
                }

                /// <summary>Places the server into the listening state, to monitor for incoming connection requests.</summary>
                /// 
                /// <remarks>
                /// The server can be placed in the listening state only once per KTcpServer object. After
                /// the server is shut down, the KTcpServer object cannot be used to listen again on another port.
                /// </remarks>
                /// 
                /// <param name="address">A local IP address to which the server should bind, or KIpAddress.Any().</param>
                /// <param name="port">A local port number to which the server should bind, or KIpEndPoint.AnyPort.</param>
                /// <param name="backlog">The maximum number of pending connection requests to enqueue.</param>
                void Listen(KIpAddress address, k32s port, k64s backlog)
                {
                    KCheck(kTcpServer_Listen(Handle, (kIpAddress)address, (k32u)port, (kSize)backlog)); 
                }

                /// <summary>Blocks until an incoming connection is established, or the specified timeout interval elapses.</summary>
                /// 
                /// <remarks>
                /// The returned connection object can be null if the connection was closed by the remote client
                /// before being accepted.
                /// </remarks>
                /// 
                /// <param name="timeout">The timeout interval.</param>
                /// <returns>KTcpClient object representing the newly-established connection, or null.</returns>
                KTcpClient^ Accept(k64s timeout)
                {
                    return Accept(timeout, nullptr); 
                }

                /// <inheritdoc cref="Accept(k64s)" />
                /// <param name="allocator">Memory allocator for the accepted socket.</param>
                KTcpClient^ Accept(k64s timeout, KAlloc^ allocator)
                {
                    kTcpClient client = kNULL;

                    KCheck(kTcpServer_Accept(Handle, (k64u)timeout, &client, KToHandle(allocator)));

                    return kIsNull(client) ? nullptr : gcnew KTcpClient(IntPtr(client));
                }

                /// <summary>Gets the underlying KSocket object.</summary>
                property KSocket^ Socket
                {
                    KSocket^ get() { return gcnew KSocket(IntPtr(kTcpServer_Socket(Handle))); }
                }

                /// <summary>Gets the local end point for a listening server.</summary>
                property KIpEndPoint LocalEndPoint
                {
                    KIpEndPoint get()
                    {
                        KIpEndPoint endPoint;

                        KCheck(kTcpServer_LocalEndPoint(Handle, (kIpEndPoint*)&endPoint));

                        return endPoint;
                    }
                }


            protected:
                KTcpServer(KRefStyle refStyle) : KObject(refStyle){}
            };
        }
    }
}

#endif

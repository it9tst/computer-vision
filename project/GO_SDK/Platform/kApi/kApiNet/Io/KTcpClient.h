// 
// KTcpClient.h
// 
// Copyright (C) 2014-2021 by LMI Technologies Inc.
// Licensed under the MIT License.
// Redistributed files must retain the above copyright notice.
// 
#ifndef K_API_NET_TCP_CLIENT_H
#define K_API_NET_TCP_CLIENT_H

#include <kApi/Io/kTcpClient.h>
#include "kApiNet/KAlloc.h"
#include "kApiNet/Io/KStream.h"
#include "kApiNet/Io/KSocket.h"

namespace Lmi3d
{
    namespace Zen 
    {
        namespace Io
        {
            /// <summary>Represents a TCP client. <para/> Requires manual disposal.</summary>
            public ref class KTcpClient : public KStream
            {
                KDeclareClass(KTcpClient, kTcpClient)

            public:
                /// <summary>Initializes a new instance of the KTcpClient class with the specified Zen object handle.</summary>           
                /// <param name="handle">Zen object handle.</param>
                KTcpClient(IntPtr handle)
                    : KStream(handle, DefaultRefStyle)
                {}

                /// <summary>Initializes a new instance of the KTcpClient class.</summary>           
                /// 
                /// <param name="ipVersion">Internet Protocol version.</param>
                KTcpClient(KIpVersion ipVersion)
                    :KStream(DefaultRefStyle)
                {

                    kTcpClient handle = kNULL;

                    KCheck(kTcpClient_Construct(&handle, ipVersion, kNULL));

                    Handle = handle;
                }

                /// <inheritdoc cref="KTcpClient(KIpVersion)" />
                ///
                /// <param name="allocator">Memory allocator.</param>
                KTcpClient(KIpVersion ipVersion, KAlloc^ allocator)
                    :KStream(DefaultRefStyle)
                {
                    kTcpClient handle = kNULL;

                    KCheck(kTcpClient_Construct(&handle, ipVersion, KToHandle(allocator))); 

                    Handle = handle;
                }

                /// <summary>Sets the size of the client write buffer.</summary>
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
                        KCheck(kTcpClient_SetWriteBuffers(Handle, -1, (kSize)size)); 
                    }
                }

                /// <summary>Sets the size of the client read buffer.</summary>
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
                        KCheck(kTcpClient_SetReadBuffers(Handle, -1, (kSize)size));
                    }
                }

                /// <summary>Sets the size of the underlying socket write buffer.</summary>
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
                        KCheck(kTcpClient_SetWriteBuffers(Handle, (kSize)size, -1));
                    }
                }

                /// <summary>Sets the size of the underlying socket read buffer.</summary>
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
                        KCheck(kTcpClient_SetReadBuffers(Handle, (kSize)size, -1));
                    }
                }

                /// <summary>Sets the timeout duration (microseconds) for write operations.</summary>
                /// 
                /// <remarks>
                /// By default, kTcpClient objects do not use a timeout interval and can block indefinitely.
                /// </remarks>
                property k64s WriteTimeout
                {
                    //add get when available

                    void set(k64s timeout)
                    {
                        KCheck(kTcpClient_SetWriteTimeout(Handle, (k64s)timeout));
                    }
                }

                /// <summary>Sets the timeout duration (microseconds) for read operations.</summary>
                /// 
                /// <remarks>
                /// By default, kTcpClient objects do not use a timeout interval and can block indefinitely.
                /// </remarks>
                property k64s ReadTimeout
                {
                    //add get when available

                    void set(k64s timeout)
                    {
                        KCheck(kTcpClient_SetReadTimeout(Handle, (k64s)timeout));
                    }
                }

                /// <summary>Enables limited support for forward/reverse seek operations. </summary>
                /// 
                property bool EnableSeek
                {
                    //add get when available

                    void set(bool enable)
                    {
                        KCheck(kTcpClient_EnableSeek(Handle, enable)); 
                    }
                }

                /// <summary>Cancels any pending I/O operations.</summary>
                /// 
                /// <remarks>
                /// This method is thread-safe.
                /// </remarks>
                void Cancel()
                {
                    KCheck(kTcpClient_Cancel(Handle)); 
                }

                /// <summary>Connects to a remote end point.</summary>
                /// 
                /// <remarks>
                /// A connection can be established only once per KTcpClient object.
                /// </remarks>
                /// 
                /// <param name="address">The remote IP address.</param>
                /// <param name="port">The remote port number.</param>
                /// <param name="timeout">Timeout interval, in microseconds.</param>
                /// <exception cref="KException">Thrown if connection not established.</exception>
                void Connect(KIpAddress address, k32s port, k64s timeout)
                {
                    KCheck(kTcpClient_Connect(Handle, (kIpAddress)address, (k32s)port, (k64u)timeout));                    
                }

                /// <summary>Attempts to connect to a remote end point.</summary>
                /// 
                /// <remarks>
                /// A connection can be established only once per KTcpClient object.
                /// </remarks>
                /// 
                /// <param name="address">The remote IP address.</param>
                /// <param name="port">The remote port number.</param>
                /// <param name="timeout">Timeout interval, in microseconds.</param>
                /// <returns>true if connection established; otherwise false.</returns>
                bool TryConnect(KIpAddress address, k32s port, k64s timeout)
                {
                    return KToBool(kSuccess(kTcpClient_Connect(Handle, (kIpAddress)address, (k32s)port, (k64u)timeout))); 
                }

                /// <summary>Waits until the client has bytes to read or until the specified timeout period elapses.</summary>
                /// 
                /// <param name="timeout">Timeout interval, in microseconds.</param>
                /// <returns>true if client is ready to read; otherwise false.</returns>
                bool Wait(k64s timeout)
                {
                    return KToBool(kSuccess(kTcpClient_Wait(Handle, (k64u)timeout))); 
                }

                /// <summary>Gets the underlying KSocket object.</summary>
                property KSocket^ Socket
                {
                    KSocket^ get() { return gcnew KSocket(IntPtr(kTcpClient_Socket(Handle))); }
                }

                /// <summary>Gets the number of bytes currently enqueued and available for reading.</summary>
                /// 
                /// <remarks>
                /// This property reports the count of bytes enqueued in the client's internal read buffer. This does
                /// not include any data enqueued in the underlying socket's read buffer.
                /// </remarks>
                property k64s Available
                {
                    k64s get() { return (k64s)kTcpClient_Available(Handle); }
                }

                /// <summary>Gets the local end point for a connected client.</summary>
                property KIpEndPoint LocalEndPoint
                {
                    KIpEndPoint get()
                    {
                        KIpEndPoint endPoint; 

                        KCheck(kTcpClient_LocalEndPoint(Handle, (kIpEndPoint*)&endPoint)); 

                        return endPoint;
                    }
                }

                /// <summary>Gets the remote end point for a connected client.</summary>
                property KIpEndPoint RemoteEndPoint
                {
                    KIpEndPoint get()
                    {
                        KIpEndPoint endPoint;

                        KCheck(kTcpClient_RemoteEndPoint(Handle, (kIpEndPoint*)&endPoint));

                        return endPoint;
                    }
                }

                /// <summary>Reports whether the client has internal errors that will prevent success of future communication attempts.</summary>
                property bool HasError
                {
                    bool get()
                    {
                        return KToBool(kIsError(kTcpClient_Status(Handle))); 
                    }
                }

            protected:
                KTcpClient() : KStream(DefaultRefStyle) {}
            };
        }
    }
}

#endif

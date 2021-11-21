// 
// KUdpClient.h
// 
// Copyright (C) 2014-2021 by LMI Technologies Inc.
// Licensed under the MIT License.
// Redistributed files must retain the above copyright notice.
// 
#ifndef K_API_NET_UDP_CLIENT_H
#define K_API_NET_UDP_CLIENT_H

#include <kApi/Io/kUdpClient.h>
#include "kApiNet/KAlloc.h"
#include "kApiNet/Io/KStream.h"
#include "kApiNet/Io/KSocket.h"

namespace Lmi3d
{
    namespace Zen 
    {
        namespace Io
        {
            /// <summary>Represents a UDP client. <para/> Requires manual disposal.</summary>
            public ref class KUdpClient : public KStream
            {
                KDeclareClass(KUdpClient, kUdpClient)

            public:
                /// <summary>Initializes a new instance of the KUdpClient class with the specified Zen object handle.</summary>           
                /// <param name="handle">Zen object handle.</param>
                KUdpClient(IntPtr handle)
                    : KStream(handle, DefaultRefStyle)
                {}

                /// <summary>Initializes a new instance of the KUdpClient class.</summary>     
                /// 
                /// <param name="ipVersion">Internet Protocol version.</param>
                KUdpClient(KIpVersion ipVersion)
                    :KStream(DefaultRefStyle)
                {
                    kUdpClient handle = kNULL;

                    KCheck(kUdpClient_Construct(&handle, ipVersion, kNULL));

                    Handle = handle;
                }

                /// <inheritdoc cref="KUdpClient(KIpVersion)" />
                /// 
                /// <param name="allocator">Memory allocator (or kNULL for default).</param>
                KUdpClient(KIpVersion ipVersion, KAlloc^ allocator)
                    :KStream(DefaultRefStyle)
                {
                    kUdpClient handle = kNULL;

                    KCheck(kUdpClient_Construct(&handle, ipVersion, KToHandle(allocator)));

                    Handle = handle;
                }

                /// <summary>Binds the client to a local IP address and/or port.</summary>
                /// 
                /// <param name="address">A local IP address, or KIpAddress.Any().</param>
                /// <param name="port">A local port number, or KIpEndPoint.AnyPort.</param>
                void Bind(KIpAddress address, k32s port)
                {
                    KCheck(kUdpClient_Bind(Handle, (kIpAddress)address, (k32u)port)); 
                }

                /// <summary>Blocks until a datagram is received into the provided buffer (or until a timeout occurs).</summary>
                /// 
                /// <remarks>
                /// This method can be used when read buffering is disabled to read a datagram directly from the
                /// underlying socket.
                /// </remarks>
                /// 
                /// <param name="endPoint">Receives the end point of the sender.</param>
                /// <param name="buffer">Destination for received bytes.</param>
                /// <param name="capacity">Maximum count of bytes to read.</param>
                /// <param name="timeout">Timeout, in microseconds.</param>
                /// <returns>Size of the received datagram.</returns>
                k32s ReadFrom([Out] KIpEndPoint% endPoint, IntPtr buffer, k32s capacity, k64s timeout)
                {
                    KIpEndPoint sender; 
                    kSize received; 

                    KCheck(kUdpClient_ReadFrom(Handle, (kIpEndPoint*)&sender, buffer.ToPointer(), (kSize)capacity, &received, (k64s)timeout)); 

                    endPoint = sender; 

                    return (k32s)received; 
                }

                /// <inheritdoc cref="ReadFrom(KIpEndPoint%%, IntPtr, k32s, k64s)" />
                k32s ReadFrom([Out] KIpEndPoint% endPoint, array<kByte>^ buffer, k64s timeout)
                {
                    pin_ptr<Byte> pinnedBuffer = &buffer[0]; 

                    return ReadFrom(endPoint, IntPtr(pinnedBuffer), buffer->Length, timeout); 
                }

                /// <summary>Blocks until the provided datagram is written to the underlying socket (or until a timeout occurs).</summary>
                /// 
                /// <remarks>
                /// This method can be used when write buffering is disabled to write a datagram directly to the
                /// underlying socket.
                /// </remarks>
                /// 
                /// <param name="buffer">Bytes to be written.</param>
                /// <param name="size">Count of bytes to be written.</param>
                /// <param name="address">IP address of the recipient.</param>
                /// <param name="port">Port number of the recipient.</param>
                /// <param name="timeout">Timeout, in microseconds.</param>
                void WriteTo(KIpAddress address, k32s port, IntPtr buffer, k32s size, k64s timeout)
                {
                    KCheck(kUdpClient_WriteTo(Handle, buffer.ToPointer(), (kSize)size, (kIpAddress)address, (k32u)port, (k32s)timeout));
                }

                /// <inheritdoc cref="WriteTo(KIpAddress, k32s, IntPtr, k32s, k64s)" />
                void WriteTo(KIpAddress address, k32s port, array<kByte>^ buffer, k32s size, k64s timeout)
                {
                    pin_ptr<Byte> pinnedBuffer = &buffer[0]; 

                    WriteTo(address, port, IntPtr(pinnedBuffer), size, timeout); 
                }

                /// <summary>Blocks until a datagram is received into the client read buffer (or until a timeout occurs).</summary>
                /// 
                /// <remarks>
                /// The Receive method is used to receive a datagram into KUdpClient's internal read buffer.
                /// Once the datagram has been received, the KStream.Read method can be used to read out the
                /// datagram.
                /// </remarks>
                /// 
                /// <param name="endPoint">Receives the end point of the sender.</param>
                /// <param name="timeout">Timeout, in microseconds.</param>
                /// <returns>Size of the received datagram.</returns>
                k32s Receive([Out] KIpEndPoint% endPoint, k64s timeout)
                {
                    KIpEndPoint sender; 
                    kSize received;

                    KCheck(kUdpClient_Receive(Handle, (kIpEndPoint*)&sender, &received, (k64u)timeout));

                    endPoint = sender; 

                    return (k32s)received;
                }

                /// <summary>Blocks until the datagram in client's internal write buffer is written to the underlying socket
                /// (or until a timeout occurs).</summary>
                /// 
                /// <remarks>
                /// <para>The Send method is used to send a datagram that has been written into KUdpClient's internal
                /// write buffer. Bytes are written into KUdpClient's internal buffer via the KStream.Write method.</para>
                /// 
                /// <para>The 'clear' argument determines whether the internal write buffer is reset after sending the message, 
                /// or whether it retains the datagram for subsequent retransmission.</para>
                /// </remarks>
                /// 
                /// <param name="address">IP address of the recipient.</param>
                /// <param name="port">Port number of the recipient.</param>
                /// <param name="timeout">Timeout, in microseconds.</param>
                /// <param name="clear">Specifies whether the internal write buffer pointer is updated.</param>
                void Send(KIpAddress address, k32s port, k64s timeout, bool clear)
                {
                    KCheck(kUdpClient_Send(Handle, (kIpAddress)address, (k32u)port, (k64u)timeout, clear)); 
                }

                /// <summary>Clears the internal write buffer state.</summary>
                void Clear()
                {
                    KCheck(kUdpClient_Clear(Handle)); 
                }

                /// <summary>Enables or disables broadcasting.</summary>
                /// 
                /// <remarks>
                /// <para>If broadcasting is enabled, sending to KIpAddress.BroadcastV4() will broadcast a datagram
                /// on the subnet associated with the IPv4 address to which the client is bound.</para>
                /// 
                /// <para>Broadcasts are disabled by default.</para>
                /// </remarks>
                property bool BroadcastEnabled
                {
                    //add get when available

                    void set(bool enabled)
                    {
                        KCheck(kUdpClient_EnableBroadcast(Handle, enabled));
                    }
                }

                /// <summary>Enables or disables reuse of a local end point within a short period of time.</summary>
                /// 
                /// <remarks>
                /// <para>This option is typically used to allow a server to rebind to a local end point
                /// while a previous socket with the same local end point is in the TIME_WAIT state.
                /// This can be useful when a server must be stopped and started within a brief interval,
                /// but there is a small risk that packets with identical source/destination information
                /// could be misdirected to the new socket.</para>
                /// 
                /// <para>This option is disabled by default.</para>
                /// </remarks>
                property bool AddressReuseEnabled
                {
                    //add get when available

                    void set(bool enabled)
                    {
                        KCheck(kUdpClient_EnableReuseAddress(Handle, enabled));
                    }
                }

                /// <summary>Sets the size of the client write buffer.</summary>
                /// 
                /// <remarks>
                /// <para>A client write buffer enables the sender to formulate a datagram over multiple writes, 
                /// rather than supplying the entire datagram in a single write call.</para>
                /// 
                /// <para>If the client buffer size is greater than zero, use the KUdpClient.Send method to
                /// send the datagram when writing is complete. If the client buffer size is zero, a datagram
                /// can be sent immediately (without buffering) using KUdpClient.WriteTo.</para>
                /// 
                /// <para>By default, the client buffer size is zero and the socket buffer size is determined by the
                /// underlying operating system.</para>
                /// </remarks>
                property k64s ClientWriteBufferSize
                {
                    //add get when available

                    void set(k64s size)
                    {
                        KCheck(kUdpClient_SetWriteBuffers(Handle, -1, (kSize)size));
                    }
                }

                /// <summary>Sets the size of the client read buffer.</summary>
                /// 
                /// <remarks>
                /// <para>A client buffer enables the client to read a datagram over multiple read calls, rather than 
                /// receiving the entire datagram in a single read call.</para>
                /// 
                /// <para>If the client buffer size is greater than zero, use the KUdpClient.Receive method to
                /// receive a datagram before calling KUdpClient.Read. If the client buffer size is zero, a
                /// complete datagram can be received (without buffering) using KUdpClient.ReadFrom.</para>
                /// </remarks>
                property k64s ClientReadBufferSize
                {
                    //add get when available

                    void set(k64s size)
                    {
                        KCheck(kUdpClient_SetReadBuffers(Handle, -1, (kSize)size));
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
                        KCheck(kUdpClient_SetWriteBuffers(Handle, (kSize)size, -1));
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
                        KCheck(kUdpClient_SetReadBuffers(Handle, (kSize)size, -1));
                    }
                }

                /// <summary>Gets the underlying KSocket object.</summary>
                property KSocket^ Socket
                {
                    KSocket^ get() { return gcnew KSocket(IntPtr(kUdpClient_Socket(Handle))); }
                }

                /// <summary>Gets the local end point for a bound client.</summary>
                property KIpEndPoint LocalEndPoint
                {
                    KIpEndPoint get()
                    {
                        KIpEndPoint endPoint;

                        KCheck(kUdpClient_LocalEndPoint(Handle, (kIpEndPoint*)&endPoint));

                        return endPoint;
                    }
                }

            protected:
                KUdpClient() : KStream(DefaultRefStyle) {}
            };
        }
    }
}

#endif

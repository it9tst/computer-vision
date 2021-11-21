// 
// KSocket.h
// 
// Copyright (C) 2014-2021 by LMI Technologies Inc.
// Licensed under the MIT License.
// Redistributed files must retain the above copyright notice.
// 
#ifndef K_API_NET_SOCKET_H
#define K_API_NET_SOCKET_H

#include <kApi/Io/kSocket.h>
#include "kApiNet/KAlloc.h"
#include "kApiNet/Io/KNetwork.h"

namespace Lmi3d
{
    namespace Zen 
    {
        namespace Io
        {
            /// <summary>Represents the underlying type of a socket.</summary>       
            public value struct KSocketType
            {
                KDeclareEnum(KSocketType, kSocketType)

                /// <summary>TCP Socket.</summary>
                literal k32s Tcp = kSOCKET_TYPE_TCP;

                /// <summary>UDP Socket.</summary>
                literal k32s Udp = kSOCKET_TYPE_UDP;
            };

            /// <summary>Represents a set of socket event types.</summary>       
            public value struct KSocketEvent
            {
                KDeclareEnum(KSocketEvent, kSocketEvent)

                /// <summary>Socket ready for reading/accepting.</summary>
                literal k32s Read = kSOCKET_EVENT_READ;

                /// <summary>Socket potentially ready for writing.</summary>
                literal k32s Write = kSOCKET_EVENT_WRITE;

                /// <summary>Socket has an unexpected condition.</summary>
                literal k32s Except = kSOCKET_EVENT_EXCEPT;
            };

            /// <summary>Represents a network socket. <para/> Requires manual disposal.</summary>
            public ref class KSocket : public KObject
            {
                KDeclareClass(KSocket, kSocket)

            public:
                /// <summary>Initializes a new instance of the KSocket class with the specified Zen object handle.</summary>           
                /// <param name="handle">Zen object handle.</param>
                KSocket(IntPtr handle)
                    : KObject(handle, DefaultRefStyle)
                {}

                /// <summary>Constructs a kSocket object.</summary>
                /// 
                /// <remarks>
                /// Note: Methods in the socket API are not thread-safe, unless otherwise noted.
                /// </remarks>
                /// 
                /// <param name="ipVersion">Internet Protocol version.</param>
                /// <param name="socketType">The type of socket to create (i.e. TCP or UDP).</param>
                KSocket(KIpVersion ipVersion, KSocketType socketType)
                    :KObject(DefaultRefStyle)
                {
                    kSocket handle = kNULL;

                    KCheck(kSocket_Construct(&handle, ipVersion, socketType, kNULL));

                    Handle = handle;
                }

                /// <inheritdoc cref="KSocket(KIpVersion, KSocketType)" />
                /// <param name="allocator">Memory allocator.</param>
                KSocket(KIpVersion ipVersion, KSocketType socketType, KAlloc^ allocator)
                    :KObject(DefaultRefStyle)
                {
                    kSocket handle = kNULL;

                    KCheck(kSocket_Construct(&handle, ipVersion, socketType, KToHandle(allocator))); 

                    Handle = handle;
                }

                /// <summary>Waits until an event occurs on one or more sockets.</summary>
                /// 
                /// <remarks>
                /// <para>Before calling this method, use KSocket.SetEvents to specify the events that the sockets
                /// should wait on. After calling this method, use the KSocket.Events property to determine
                /// which events have occurred.</para>
                /// </remarks>
                /// 
                /// <param name="sockets">An array of sockets to wait on.</param>
                /// <param name="timeout">Timeout, in microseconds.</param>
                /// <returns>true if one or more sockets are ready; otherwise false.</returns>
                static bool WaitAny(array<KSocket^>^ sockets, k64u timeout)
                {
                    array<kObject>^ handles = KInternalUtils::ToHandleArray(sockets); 
                    pin_ptr<kObject> pinnedHandles = &handles[0]; 

                    kStatus result = kSocket_WaitAny(pinnedHandles, sockets->Length, (k64u)timeout);

                    switch(result)
                    {
                    case kOK:                   return true; 
                    case kERROR_TIMEOUT:        return false; 
                    default:                    throw gcnew KException(result); 
                    }
                }

                /// <summary>Binds the socket to a local IP address and/or port.</summary>
                /// 
                /// <param name="address">A local IP address, or address generated using KIpAddress.Any().</param>
                /// <param name="port">A local port number, or KIpEndPoint.AnyPort.</param>
                void Bind(KIpAddress address, k32s port)
                {
                    KCheck(kSocket_Bind(Handle, (kIpAddress)address, port)); 
                }

                /// <summary>Connects the socket to a remote end point.</summary>
                /// 
                /// <remarks>
                /// A connection can be attempted only once per socket object.
                /// </remarks>
                /// 
                /// <param name="address">The remote IP address.</param>
                /// <param name="port">The remote port number.</param>
                /// <param name="timeout">The timeout interval, in microseconds.</param>
                void Connect(KIpAddress address, k32s port, k64s timeout)
                {
                    KCheck(kSocket_Connect(Handle, (kIpAddress)address, (k32u)port, (k64u)timeout));
                }

                /// <summary>Places the socket into a listening state, to monitor for incoming connection requests.</summary>
                /// 
                /// <param name="backlog">The maximum number of pending connection requests to enqueue.</param>
                void Listen(k32s backlog)
                {
                    KCheck(kSocket_Listen(Handle, (k32u)backlog)); 
                }

                /// <summary>Blocks until an incoming connection request is accepted.</summary>
                /// 
                /// <remarks>
                /// The returned socket object can be null if the connection was closed by the remote client 
                /// before being accepted.
                /// </remarks>
                ///
                /// <returns>A socket object representing the newly-established connection, or null.</returns>
                KSocket^ Accept()
                {
                    return Accept(nullptr); 
                }
                
                /// <inheritdoc cref="Accept()" />
                /// <param name="allocator">Memory allocator for new socket.</param>
                KSocket^ Accept(KAlloc^ allocator)
                {
                    kSocket connection = kNULL; 

                    KCheck(kSocket_Accept(Handle, &connection, KToHandle(allocator))); 

                    return kIsNull(connection) ? nullptr : gcnew KSocket(IntPtr(connection)); 
                }

                /// <summary>Waits for a socket event.</summary>
                /// 
                /// <remarks>
                /// <para>Before calling this method, use KSocket.SetEvents to specify the events that the socket
                /// should wait on. After calling this method, use the kSocket.Events property to determine
                /// which events have occurred.</para>
                /// </remarks>
                /// 
                /// <param name="timeout">Timeout, in microseconds.</param>
                /// <returns>true if one or more events occurred before the end of the timeout period; false otherwise.</returns>
                bool Wait(k64s timeout)
                {
                    kStatus status = kSocket_Wait(Handle, (k64u)timeout); 

                    switch (status)
                    {
                        case kOK:               return true;
                        case kERROR_TIMEOUT:    return false;
                        default:                throw gcnew KException(status); 
                    }
                }

                /// <summary>Reads one or more bytes.</summary>
                /// 
                /// <remarks>
                /// <para>In blocking mode, this method will block until at least one byte is received (or a read timeout occurs).</para>
                /// 
                /// <para>In non-blocking mode, this method will read at least one byte. KSocket.Wait should be used to determine
                /// when this method can be called successfully.</para>
                /// 
                /// <para>If the socket was closed by the remote peer, this method will return 0.</para>
                /// </remarks>
                /// 
                /// <param name="buffer">Buffer to receive bytes.</param>
                /// <param name="size">The maximum number of bytes to read.</param>
                /// <returns>The number of bytes that were read; 0 if the connection was closed.</returns>
                k64s Read(IntPtr buffer, k64s size)
                {
                    kSize bytesRead = 0; 

                    kStatus status = kSocket_Read(Handle, buffer.ToPointer(), (kSize)size, &bytesRead); 

                    switch(status)
                    {
                    case kOK:               return (k64s)bytesRead;
                    case kERROR_CLOSED:     return 0;
                    default:                throw gcnew KException(status);
                    }
                }

                /// <inheritdoc cref="Read(IntPtr, k64s)" />
                /// 
                /// <param name="start">Offset into the buffer.</param>
                /// <param name="maxCount">The maximum number of bytes to read.</param>
                k32s Read(array<kByte>^ buffer, k32s start, k32s maxCount)
                {
                    pin_ptr<kByte> pinnedBuffer = &buffer[start]; 

                    return (k32s) Read(IntPtr(pinnedBuffer), maxCount); 
                }

                /// <summary>Reads a datagram.</summary>
                /// 
                /// <remarks>
                /// <para>Reads a datagram.</para>
                ///
                /// <para>In blocking mode, this method will block until a datagram is read (or a read timeout occurs).</para>
                /// 
                /// <para>In non-blocking mode, kSocket_Wait should be used to determine when this method can be called successfully.</para> 
                /// </remarks>
                /// 
                /// <param name="endPoint">Receives the address of the sender.</param>
                /// <param name="buffer">Buffer to receive bytes.</param>
                /// <param name="size">The maximum number of bytes to read.</param>
                /// <returns>The number of bytes that were read; 0 if the connection was closed.</returns>
                k64s ReadFrom([Out] KIpEndPoint% endPoint, IntPtr buffer, k64s size)
                {
                    kSize bytesRead = 0; 
                    kIpEndPoint sender; 

                    kStatus status = kSocket_ReadFrom(Handle, &sender, buffer.ToPointer(), (kSize)size, &bytesRead); 

                    endPoint = KIpEndPoint(&sender); 

                    switch(status)
                    {
                    case kOK:               return (k64s)bytesRead;
                    case kERROR_CLOSED:     return 0;
                    default:                throw gcnew KException(status);
                    }
                }

                /// <inheritdoc cref="ReadFrom(KIpEndPoint%%, IntPtr, k64s)" />
                /// 
                /// <param name="start">Offset into the buffer.</param>
                /// <param name="maxCount">The maximum number of bytes to read.</param>
                k32s ReadFrom([Out] KIpEndPoint% endPoint, array<kByte>^ buffer, k32s start, k32s maxCount)
                {
                    pin_ptr<kByte> pinnedBuffer = &buffer[start]; 
                    
                    return (k32s) ReadFrom(endPoint, IntPtr(pinnedBuffer), maxCount); 
                }

                /// <summary>Writes one or more bytes.</summary>
                /// 
                /// <remarks>
                /// <para>In blocking mode, this method will block until all bytes are written (or a write timeout occurs).</para>
                /// 
                /// <para>In non-blocking mode, this method will write zero or more bytes. KSocket.Wait can be used to
                /// determine when buffer space is available for a write operation, increasing the odds that bytes
                /// can be written successfully.</para>
                /// </remarks>
                /// 
                /// <param name="buffer">Buffer of bytes to write.</param>
                /// <param name="size">The number of bytes to write.</param>
                /// <returns>The number of bytes that were written.</returns>
                k64s Write(IntPtr buffer, k64s size)
                {
                    kSize bytesWritten; 

                    KCheck(kSocket_Write(Handle, buffer.ToPointer(), (kSize)size, &bytesWritten)); 

                    return (k64s)bytesWritten; 
                }

                /// <inheritdoc cref="Write(IntPtr, k64s)" />
                /// <param name="start">Offset into buffer.</param>
                /// <param name="maxCount">The number of bytes to write.</param>
                k32s Write(array<kByte>^ buffer, k32s start, k32s maxCount)
                {
                    pin_ptr<kByte> pinnedBuffer = &buffer[start]; 

                    return (k32s)Write(IntPtr(pinnedBuffer), maxCount);
                }

                /// <summary>Sends a datagram.</summary>
                /// 
                /// <remarks>
                /// <para>In blocking mode, this method will block until the datagram is sent (or a write timeout occurs).</para>
                /// 
                /// <para>In non-blocking mode, use kSocket_Wait to determine when this method can be called successfully.</para>
                /// </remarks>
                /// 
                /// <param name="address">IP address of the recipient.</param>
                /// <param name="port">Port number of the recipient.</param>
                /// <param name="buffer">Buffer containing the datagram to send.</param>
                /// <param name="size">The number of bytes in the supplied buffer.</param>
                void WriteTo(KIpAddress address, k32s port, IntPtr buffer, k64s size)
                {
                    KCheck(kSocket_WriteTo(Handle, (kIpAddress)address, (k32u)port, buffer.ToPointer(), (kSize)size)); 
                }

                /// <inheritdoc cref="WriteTo(KIpAddress, k32s, IntPtr, k64s)" />
                /// <param name="start">Offset into buffer.</param>
                void WriteTo(KIpAddress address, k32s port, array<kByte>^ buffer, k32s start, k32s size)
                {
                    pin_ptr<kByte> pinnedBuffer = &buffer[start]; 

                    return WriteTo(address, port, IntPtr(pinnedBuffer), size);
                }

                /// <summary>Sets the event types that a socket will wait on.</summary>
                /// 
                /// <remarks>
                /// By default, sockets wait on read events.
                /// </remarks>
                /// 
                /// <param name="events">One or more event types to wait on.</param>
                /// <returns>Operation status.</returns>
                void SetEvents(KSocketEvent events)
                {
                    KCheck(kSocket_SetEvents(Handle, events)); 
                }

                /// <summary>Gets the events detected during the most recent wait operation.</summary>
                /// 
                /// <returns>Detected events.</returns>
                property KSocketEvent Events
                {
                    KSocketEvent get() { return kSocket_Events(Handle); }
                }

                /// <summary>Sets whether the socket will block on read/write requests.</summary>
                /// 
                /// <remarks>
                /// By default, sockets are created in blocking mode.
                /// </remarks>
                property bool IsBlocking
                {
                    //add get when available

                    void set(bool isBlocking)
                    {
                        KCheck(kSocket_SetBlocking(Handle, isBlocking));
                    }
                }

                /// <summary>Sets the size of the write buffer used by the underlying operating system.</summary>
                property k64s WriteBufferSize
                {
                    //add get when available

                    void set(k64s size)
                    {
                        KCheck(kSocket_SetWriteBuffer(Handle, (kSize)size));
                    }
                }

                /// <summary>Sets the size of the read buffer used by the underlying operating system.</summary>
                property k64s ReadBufferSize
                {
                    //add get when available

                    void set(k64s size)
                    {
                        KCheck(kSocket_SetReadBuffer(Handle, (kSize)size));
                    }
                }

                /// <summary>Sets the timeout duration (microseconds) for blocking write operations.</summary>
                /// 
                /// <remarks>
                /// By default, socket objects do not use a timeout interval and can block indefinitely.
                /// </remarks>
                property k64s WriteTimeout
                {
                    //add get when available

                    void set(k64s timeout)
                    {
                        KCheck(kSocket_SetWriteTimeout(Handle, (k64u)timeout));
                    }
                }

                /// <summary>Sets the timeout duration (microseconds) for blocking read operations.</summary>
                /// 
                /// <remarks>
                /// By default, socket objects do not use a timeout interval and can block indefinitely.
                /// </remarks>
                property k64s ReadTimeout
                {
                    //add get when available

                    void set(k64s timeout)
                    {
                        KCheck(kSocket_SetReadTimeout(Handle, (k64u)timeout));
                    }
                }

                /// <summary>Sets a value that determines whether datagram broadcasting is enabled.</summary>
                /// 
                /// <remarks>
                /// This method is typically used in conjunction with a UDP socket that is bound
                /// to a local address. Sending to kIpAddress_BroacastV4() will broadcast a
                /// datagram on the subnet associated with the bound IPv4 address.
                /// </remarks>
                property bool BroadcastEnabled
                {
                    //add get when available

                    void set(bool enabled)
                    {
                        KCheck(kSocket_EnableBroadcast(Handle, enabled));
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
                        KCheck(kSocket_EnableReuseAddress(Handle, enabled));
                    }
                }
        
                /// <summary>Sets a value that determines whether the Nagle algorithm is enabled.</summary>
                /// 
                /// <remarks>
                /// The Nagle algorithm is enabled by default. When enabled, small segments of outbound
                /// data are coalesced over a brief time period in order to improve network efficiency.
                /// </remarks>
                property bool NoDelayEnabled
                {
                    //add get when available

                    void set(bool enabled)
                    {
                        KCheck(kSocket_SetNoDelay(Handle, enabled));
                    }
                }

                /// <summary>Sets the duration (microseconds) that a TCP connection can remain open when the socket is closed
                /// in order to ensure that all outbound bytes are transmitted to the receiver.</summary>
                /// 
                /// <remarks>
                /// If the linger time is not set, then a default linger time will be selected by
                /// the underlying operating system.
                /// </remarks>
                property k64s LingerTime
                {
                    //add get when available

                    void set(k64s lingerTime)
                    {
                        KCheck(kSocket_SetLingerTime(Handle, (k64u)lingerTime));
                    }
                }

                /// Gets the local end point for a connected socket.
                property KIpEndPoint LocalEndPoint
                {
                    KIpEndPoint get() 
                    {
                        KIpEndPoint endPoint; 

                        KCheck(kSocket_LocalEndPoint(Handle, (kIpEndPoint*)&endPoint)); 

                        return endPoint; 
                    }
                }

                /// Returns the remote end point for a connected socket.
                property KIpEndPoint RemoteEndPoint
                {
                    KIpEndPoint get()
                    {
                        KIpEndPoint endPoint;

                        KCheck(kSocket_RemoteEndPoint(Handle, (kIpEndPoint*)&endPoint));

                        return endPoint;
                    }
                }

                /// Gets a value that indicates whether internal errors will prevent success of future communication attempts.
                property bool HasErrors
                {
                    bool get()
                    {
                        return KToBool(kIsError(kSocket_Status(Handle))); 
                    }
                }

            protected:
                KSocket(KRefStyle refStyle) : KObject(refStyle){}
            };
        }
    }
}

#endif

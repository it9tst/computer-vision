/** 
 * @file    kSocket.h
 * @brief   Declares the kSocket class. 
 *
 * @internal
 * Copyright (C) 2004-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_SOCKET_H
#define K_API_SOCKET_H

#include <kApi/Io/kNetwork.h>

/**
 * @struct  kSocketType
 * @extends kValue
 * @ingroup kApi-Io  
 * @brief   Represents the underlying type of a socket.
 */
typedef k32s kSocketType; 

/** @relates kSocketType @{ */
#define kSOCKET_TYPE_TCP        (0)     ///< TCP Socket.
#define kSOCKET_TYPE_UDP        (1)     ///< UDP Socket.
/** @} */

/**
 * @struct  kSocketEvent
 * @ingroup kApi-Io  
 * @brief   Represents a set of socket event types.
 */
typedef k32s kSocketEvent;

/** @relates kSocketEvent @{ */
#define kSOCKET_EVENT_READ         (1)              ///< Socket ready for reading/accepting. 
#define kSOCKET_EVENT_WRITE        (2)              ///< Socket potentially ready for writing. 
#define kSOCKET_EVENT_EXCEPT       (4)              ///< Socket has an unexpected condition.
/** @} */

#include <kApi/Io/kSocket.x.h>

/**
 * @class   kSocket
 * @extends kObject
 * @ingroup kApi-Io
 * @brief   Represents a network socket.
 */
//typedef kObject kSocket;        --forward-declared in kApiDef.x.h 

/** 
 * Waits until an event occurs on one or more sockets. 
 *
 * Before calling this function, use kSocket_SetEvents to specify the events that the sockets 
 * should wait on. After calling this function, use the kSocket_Events function to determine 
 * which events have occurred. 
 * 
 * This function will return kERROR_TIMEOUT if no socket events occur by the end of the timeout period. 
 *
 * @public              @memberof kSocket
 * @param   sockets     An array of sockets to wait on. 
 * @param   count       The number of sockets in the array. 
 * @param   timeout     Timeout, in microseconds. 
 * @return              Operation status. 
 */
kFx(kStatus) kSocket_WaitAny(const kSocket* sockets, kSize count, k64u timeout);

/** 
 * Constructs a kSocket object.
 * 
 * Note: Methods in the socket API are not thread-safe, unless otherwise noted. 
 *
 * @public              @memberof kSocket
 * @param   socket      Destination for the constructed object handle. 
 * @param   ipVersion   Internet Protocol version.
 * @param   socketType  The type of socket to create (i.e. TCP or UDP). 
 * @param   allocator   Memory allocator (or kNULL for default). 
 * @return              Operation status. 
 */
kFx(kStatus) kSocket_Construct(kSocket* socket, kIpVersion ipVersion, kSocketType socketType, kAlloc allocator);

/** 
 * Binds the socket to a local IP address and/or port. 
 *
 * @public              @memberof kSocket
 * @param   socket      Destination for the constructed object handle. 
 * @param   address     A local IP address, or kIpAddress_Any(). 
 * @param   port        A local port number, or kIP_PORT_ANY.
 * @return              Operation status. 
 */
kFx(kStatus) kSocket_Bind(kSocket socket, kIpAddress address, k32u port);

/** 
 * Connects the socket to a remote end point. 
 *
 * Connection can be attempted only once per socket object.
 *
 * @public              @memberof kSocket
 * @param   socket      Socket object. 
 * @param   address     The remote IP address. 
 * @param   port        The remote port number.
 * @param   timeout     The timeout interval, in microseconds. 
 * @return              Operation status. 
 */
kFx(kStatus) kSocket_Connect(kSocket socket, kIpAddress address, k32u port, k64u timeout); 

/** 
 * Begins connecting the socket to a remote end point. 
 *
 * This method can be used in conjuction with kSocket_EndConnect to enable useful work to be performed 
 * while waiting for socket connection to complete. If BeginConnect completes successfully, the EndConnect 
 * method must be called at some point thereafter. After calling BeginConnect, no other methods may be 
 * called on this object before EndConnect (though the object could be safely destroyed, if desired). 
 * 
 * Connection can be attempted only once per socket object.
 *
 * @public              @memberof kSocket
 * @param   socket      Socket object. 
 * @param   address     The remote IP address. 
 * @param   port        The remote port number.
 * @return              Operation status. 
 * @see                 kSocket_EndConnect
 */
kFx(kStatus) kSocket_BeginConnect(kSocket socket, kIpAddress address, k32u port); 

/** 
 * Completes a connection operation that was started with kSocket_BeginConnect. 
 *
 * @public              @memberof kSocket
 * @param   socket      Socket object. 
 * @param   timeout     The timeout interval, in microseconds. 
 * @return              Operation status. 
 * @see                 kSocket_BeginConnect
 */
kFx(kStatus) kSocket_EndConnect(kSocket socket, k64u timeout); 

/** 
 * Places the socket into a listening state, to monitor for incoming connection requests.
 *
 * @public              @memberof kSocket
 * @param   socket      Socket object. 
 * @param   backlog     The maximum number of pending connection requests to enqueue. 
 * @return              Operation status. 
 */
kFx(kStatus) kSocket_Listen(kSocket socket, kSize backlog); 

/** 
 * Blocks until an incoming connection request is accepted. 
 *
 * @public              @memberof kSocket
 * @param   socket      A socket object in the listening state.
 * @param   connection  Receives a socket object representing the newly-established connection, or kNULL.
 * @param   allocator   Memory allocator (or kNULL for default). 
 * @return              Operation status. 
 */
kFx(kStatus) kSocket_Accept(kSocket socket, kSocket* connection, kAlloc allocator);

/** 
 * Joins the specified multicast group on the specified interface.
 * 
 * In order for multicast sockets to behave similarly across all platforms, the following practices 
 * are recommended: 
 * * The receiver should bind on the group port and 'any' address (kIpAddress_AnyV4()).
 * * The receiver should join the multicast group, specifying the group address and the 
 *   desired local interface address on which to listen for incoming multicast datagrams. 
 * * The sender should bind on the desired local interface address for outgoing 
 *   multicast datagrams. The sender can use any port (e.g., an ephemeral port via kIP_PORT_ANY).  
 * 
 * Note that while some platforms support binding the receiver to the group address (e.g., Linux) 
 * other platforms do not (e.g., Windows). Accordingly, the advice given above is to bind the receiver 
 * to 'any' address, which works on all platforms. Unfortunately, this means that any unicast or 
 * broadcast datagrams that are addressed to the same port identifier may also be received by the 
 * multicast socket.  
 *
 * @public              @memberof kSocket
 * @param   socket      Socket object. 
 * @param   group       Multicast group address. 
 * @param   iface       Local interface address. 
 * @return              Operation status. 
 */
kFx(kStatus) kSocket_JoinMulticastGroup(kSocket socket, kIpAddress group, kIpAddress iface); 

/** 
 * Leaves the specified multicast group on the specified interface.
 *
 * @public              @memberof kSocket
 * @param   socket      Socket object. 
 * @param   group       Multicast group address. 
 * @param   iface       Local interface address (or kIpAddress_Any() for default). 
 * @return              Operation status. 
 */
kFx(kStatus) kSocket_LeaveMulticastGroup(kSocket socket, kIpAddress group, kIpAddress iface); 

/** 
 * Begins shutdown of socket communication.
 *
 * Shutdown will occur automatically when the socket is destroyed. This method can be used to 
 * explicitly initiate shutdown before the socket is destroyed. 
 * 
 * After shutdown, any additional socket operations will fail. 
 * 
 * @public              @memberof kSocket
 * @param   socket      Socket object. 
 * @return              Operation status. 
 */
kFx(kStatus) kSocket_Shutdown(kSocket socket); 

/** 
 * Waits for a socket event. 
 *
 * Before calling this function, use kSocket_SetEvents to specify the events that the socket 
 * should wait on. After calling this function, use the kSocket_Events function to determine 
 * which events have occurred. 
 * 
 * This function will return kERROR_TIMEOUT if no events occur by the end of the timeout period. 
 *
 * @public              @memberof kSocket
 * @param   socket      Socket object. 
 * @param   timeout     Timeout, in microseconds. 
 * @return              Operation status. 
 */
kFx(kStatus) kSocket_Wait(kSocket socket, k64u timeout); 

/** 
 * Reads one or more bytes. 
 *
 * In blocking mode, this function will block until at least one byte is received (or a read timeout occurs). 
 * 
 * In non-blocking mode, this function will read at least one byte. kSocket_Wait should be used to determine
 * when this function can be called successfully. 
 * 
 * If the socket was closed by the remote peer, this function will return kERROR_CLOSED. 
 *
 * @public              @memberof kSocket
 * @param   socket      Socket object. 
 * @param   buffer      Buffer to receive bytes. 
 * @param   size        The maximum number of bytes to read.
 * @param   read        The number of bytes that were read.
 * @return              Operation status. 
 */
kFx(kStatus) kSocket_Read(kSocket socket, void* buffer, kSize size, kSize* read);

/** 
 * Reads a datagram. 
 * 
 * In blocking mode, this function will block until a datagram is read (or a read timeout occurs). 
 * 
 * In non-blocking mode, kSocket_Wait should be used to determine when this function can be called successfully. 
 *
 * @public              @memberof kSocket
 * @param   socket      Socket object. 
 * @param   endPoint    The address of the sender. 
 * @param   buffer      Buffer to receive the datagram. 
 * @param   size        The maximum number of bytes to return.
 * @param   read        The number of bytes that were read.
 * @return              Operation status. 
 */
kFx(kStatus) kSocket_ReadFrom(kSocket socket, kIpEndPoint *endPoint, void* buffer, kSize size, kSize* read); 

/** 
 * Reads a datagram and associated metadata. 
 * 
 * In blocking mode, this function will block until a datagram is read (or a read timeout occurs). 
 * 
 * In non-blocking mode, kSocket_Wait should be used to determine when this function can be called successfully. 
 * 
 * The kNetworkInfo_FindAdapterById method can be used to translate the unique numeric adapter identifier 
 * provided by this method to a network adapter name.
 *
 * @public                  @memberof kSocket
 * @param   socket          Socket object. 
 * @param   endPoint        Receives the destination address of the datagram. 
 * @param   adapterId       Receives the unique numeric identifier associated with the local network adapter on which the datagram was received (or kSIZE_NULL, if unknown).
 * @param   buffer          Buffer to receive the datagram.
 * @param   size            The maximum number of bytes to return.
 * @param   read            The number of bytes that were read.
 * @return                  Operation status. 
 * @see                     kNetworkInfo_FindAdapterById, kNetworkAdapter_Id
 */
kFx(kStatus) kSocket_ReadFromEx(kSocket socket, kIpEndPoint *endPoint, kSize* adapterId, void* buffer, kSize size, kSize* read); 

/** 
 * Writes one or more bytes. 
 * 
 * In blocking mode, this function will block until all bytes are written (or a write timeout occurs). 
 * 
 * In non-blocking mode, this function will write zero or more bytes. kSocket_Wait can be used to 
 * determine when buffer space is available for a write operation, increasing the odds that bytes 
 * can be written successfully.
 *
 * @public              @memberof kSocket
 * @param   socket      Socket object. 
 * @param   buffer      Buffer of bytes to write. 
 * @param   size        The number of bytes to write.
 * @param   written     The number of bytes that were written.
 * @return              Operation status. 
 */
kFx(kStatus) kSocket_Write(kSocket socket, const void* buffer, kSize size, kSize* written); 
      
/** 
 * Sends a datagram. 
 * 
 * In blocking mode, this function will block until the datagram is sent (or a write timeout occurs). 
 * 
 * In non-blocking mode, use kSocket_Wait to determine when this function can be called successfully. 
 * 
 * @public              @memberof kSocket
 * @param   socket      Socket object. 
 * @param   address     IP address of the recipient. 
 * @param   port        Port number of the recipient. 
 * @param   buffer      Buffer containing the datagram to send.
 * @param   size        The number of bytes in the supplied buffer.
 * @return              Operation status. 
 */
kFx(kStatus) kSocket_WriteTo(kSocket socket, kIpAddress address, k32u port, const void* buffer, kSize size); 

/** 
 * Sets the event types that a socket will wait on. 
 *
 * By default, sockets wait on read events. 
 * 
 * @public              @memberof kSocket
 * @param   socket      Socket object. 
 * @param   events      One or more event types to wait on. 
 * @return              Operation status. 
 */
kFx(kStatus) kSocket_SetEvents(kSocket socket, kSocketEvent events); 

/** 
 * Gets the events detected during the most recent wait operation. 
 *
 * @public              @memberof kSocket
 * @param   socket      Socket object. 
 * @return              Detected events. 
 */
kFx(kSocketEvent) kSocket_Events(kSocket socket); 

/** 
 * Determines whether the socket will block on read/write requests. 
 * 
 * By default, sockets are created in blocking mode. 
 *
 * @public                  @memberof kSocket
 * @param   socket          Socket object. 
 * @param   isBlocking      If kTRUE, the socket will be placed in blocking mode. 
 * @return                  Operation status. 
 */
kFx(kStatus) kSocket_SetBlocking(kSocket socket, kBool isBlocking);

/** 
 * Sets the size of the write buffer used by the underlying operating system.
 *
 * @public              @memberof kSocket
 * @param   socket      Socket object. 
 * @param   size        Size of the write buffer.
 * @return              Operation status. 
 */
kFx(kStatus) kSocket_SetWriteBuffer(kSocket socket, kSize size); 

/** 
 * Sets the size of the read buffer used by the underlying operating system.
 *
 * @public              @memberof kSocket
 * @param   socket      Socket object. 
 * @param   size        Size of the read buffer.
 * @return              Operation status. 
 */
kFx(kStatus) kSocket_SetReadBuffer(kSocket socket, kSize size);

/** 
 * Sets the timeout duration for blocking write operations.  
 *
 * By default, socket objects do not use a timeout interval and can block indefinitely.
 *
 * @public              @memberof kSocket
 * @param   socket      Socket object. 
 * @param   timeout     Timeout value, in microseconds.
 * @return              Operation status. 
 */
kFx(kStatus) kSocket_SetWriteTimeout(kSocket socket, k64u timeout);

/** 
 * Sets the timeout duration for blocking read operations. 
 *
 * By default, kSocket objects do not use a timeout interval and can block indefinitely.
 *
 * @public              @memberof kSocket
 * @param   socket      Socket object. 
 * @param   timeout     Timeout value, in microseconds.
 * @return              Operation status. 
 */
kFx(kStatus) kSocket_SetReadTimeout(kSocket socket, k64u timeout);

/** 
 * Enables or disables datagram broadcasting.
 *
 * This function is typically used in conjunction with a UDP socket that is bound 
 * to a local address. Sending to kIpAddress_BroacastV4() will broadcast a 
 * datagram on the subnet associated with the bound IPv4 address.
 *
 * @public              @memberof kSocket
 * @param   socket      Socket object. 
 * @param   broadcast   kTRUE to enable broadcasts; kFALSE otherwise.
 * @return              Operation status. 
 */
kFx(kStatus) kSocket_EnableBroadcast(kSocket socket, kBool broadcast); 

/** 
 * Enables or disables reuse of a local end point within a short period of time.
 *
 * The option is typically used to allow a server to rebind to a local end point 
 * while a previous socket with the same local end point is in the TIME_WAIT state. 
 * This can be useful when a server must be stopped and started within a brief interval, 
 * but there is a small risk that packets with identical source/destination information 
 * could be misdirected to the new socket. 
 *
 * @public              @memberof kSocket
 * @param   socket      Socket object. 
 * @param   reuse       kTRUE to enable reuse of IP addresses; kFALSE otherwise.
 * @return              Operation status. 
 */
kFx(kStatus) kSocket_EnableReuseAddress(kSocket socket, kBool reuse); 

/** 
 * Can be used to disable the Nagle algorithm.
 *
 * The Nagle algorithm is enabled by default. When enabled, small segments of outbound 
 * data are coalesced over a brief time period in order to improve network efficiency. 
 *
 * @public              @memberof kSocket
 * @param   socket      Socket object. 
 * @param   noDelay     kTRUE to disable the Nagle algorithm; kFALSE to enable. 
 * @return              Operation status. 
 */
kFx(kStatus) kSocket_SetNoDelay(kSocket socket, kBool noDelay); 

/** 
 * Specifies the duration that a TCP connection can remain open when the socket is closed
 * in order to ensure that all outbound bytes are transmitted to the receiver. 
 * 
 * If the linger time is not set, then a default linger time will be selected by 
 * the underlying operating system.
 *
 * @public              @memberof kSocket
 * @param   socket      Socket object. 
 * @param   lingerTime  Linger time, in microseconds (0 for immediate closure). 
 * @return              Operation status. 
 */
kFx(kStatus) kSocket_SetLingerTime(kSocket socket, k64u lingerTime); 

/** 
 * Enables or disables interface index information for datagrams.
 *
 * This function can only be used in conjunction with a UDP socket. 
 * Index information must be enabled before using kSocket_ReadFromEx.
 *
 * @public              @memberof kSocket
 * @param   socket      Socket object. 
 * @param   enabled     kTRUE to enable package information; kFALSE otherwise.
 * @return              Operation status. 
 */
kFx(kStatus) kSocket_EnablePacketInfo(kSocket socket, kBool enabled); 

/** 
 * Binds the socket to a specific network interface.
 *
 * This function was introduced to work around a UDP broadcast issue on QNX, and is not 
 * supported on all platforms. 
 *
 * @public                  @memberof kSocket
 * @param   socket          Socket object. 
 * @param   interfaceName   Device name. 
 * @return                  Operation status. 
 */
kFx(kStatus) kSocket_BindToDevice(kSocket socket, const kChar* interfaceName); 

/** 
 * Returns the local end point for a bound socket. 
 *
 * @public              @memberof kSocket
 * @param   socket      Socket object. 
 * @param   endPoint    Local end point.
 * @return              Operation status. 
 */
kFx(kStatus) kSocket_LocalEndPoint(kSocket socket, kIpEndPoint* endPoint); 

/** 
 *  Returns the remote end point for a connected socket. 
 *
 * @public              @memberof kSocket
 * @param   socket      Socket object. 
 * @param   endPoint    Remote end point.
 * @return              Operation status. 
 */
kFx(kStatus) kSocket_RemoteEndPoint(kSocket socket, kIpEndPoint* endPoint);

/** 
 * Reports any internal errors that will prevent success of future communication attempts.
 * 
 * @public              @memberof kSocket
 * @param   socket      Socket object. 
 * @return              Socket status. 
 */
kFx(kStatus) kSocket_Status(kSocket socket); 

/**
 * Provides access to the underlying O/S socket handle. 
 * 
 * It is strongly recommended to avoid making direct use of the underlying O/S handle. 
 * Manipulating this handle may cause subsequent kSocket methods to behave incorrectly. 
 * This method may be removed in future releases. Use at your own risk. 
 * 
 * @public              @memberof kSocket
 * @param   socket      Socket object. 
 * @return              Socket handle.  
 */
kFx(kSize) kSocket_Handle(kSocket socket);

#endif

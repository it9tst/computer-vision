/** 
 * @file    kTcpClient.h
 * @brief   Declares the kTcpClient class. 
 *
 * @internal
 * Copyright (C) 2008-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_TCP_CLIENT_H
#define K_API_TCP_CLIENT_H

#include <kApi/Io/kNetwork.h>
#include <kApi/Io/kTcpClient.x.h>

/**
 * @class   kTcpClient
 * @extends kStream
 * @ingroup kApi-Io
 * @brief   Represents a TCP client.
 * 
 */
//typedef kStream kTcpClient;         --forward-declared in kApiDef.x.h 

/** 
 * Constructs a TCP client object.
 *
 * @public              @memberof kTcpClient
 * @param   client      Destination for the constructed object handle. 
 * @param   ipVersion   Internet Protocol version.
 * @param   allocator   Memory allocator (or kNULL for default). 
 * @return              Operation status. 
 */
kFx(kStatus) kTcpClient_Construct(kTcpClient* client, kIpVersion ipVersion, kAlloc allocator);

/** 
 * Sets the size of write buffers.
 * 
 * Socket buffers decouple the sender and receiver, so that the sender does not need to block
 * while waiting for the receiver to receive all bytes. Client buffers improve the efficiency 
 * of the client when performing several small write operations. 
 * 
 * By default, the client buffer size is zero and the socket buffer size is determined by the 
 * underlying operating system. 
 * 
 * On some systems, server-side socket buffer sizes must be determined before kTcpClient
 * objects are created in kTcpServer_Accept. Accordingly, server-side implementations should use 
 * kTcpServer_SetWriteBuffers to set buffer sizes instead of kTcpClient_SetWriteBuffers.
 *
 * @public              @memberof kTcpClient
 * @param   client      TCP client object. 
 * @param   socketSize  Size of the write buffer maintained by the underlying socket (-1 to leave unchanged). 
 * @param   clientSize  Size of the write buffer maintained by the client object (-1 to leave unchanged).
 * @return              Operation status. 
 * @see                 kTcpServer_SetWriteBuffers
 */
kFx(kStatus) kTcpClient_SetWriteBuffers(kTcpClient client, kSSize socketSize, kSSize clientSize); 

/** 
 * Sets the size of read buffers. 
 * 
 * Socket buffers decouple the sender and receiver, so that the sender does not need to block
 * while waiting for the receiver to receive all bytes. Client buffers improve the efficiency 
 * of the client when performing several small read operations. 
 *
 * On some systems, server-side socket buffer sizes must be determined before kTcpClient
 * objects are created in kTcpServer_Accept. Accordingly, server-side implementations should use 
 * kTcpServer_SetReadBuffers to set buffer sizes instead of kTcpClient_SetReadBuffers.
 *
 * @public              @memberof kTcpClient
 * @param   client      TCP client object. 
 * @param   socketSize  Size of the read buffer maintained by the underlying socket (-1 to leave unchanged).
 * @param   clientSize  Size of the read buffer maintained by the client object (-1 to leave unchanged).
 * @see                 kTcpServer_SetReadBuffers
 * @return              Operation status. 
 */
kFx(kStatus) kTcpClient_SetReadBuffers(kTcpClient client, kSSize socketSize, kSSize clientSize);

/** 
 * Sets the timeout duration for write operations.  
 *
 * By default, kTcpClient objects do not use a timeout interval and can block indefinitely.
 *
 * @public              @memberof kTcpClient
 * @param   client      TCP client object. 
 * @param   timeout     Timeout value, in microseconds.
 * @return              Operation status. 
 */
kFx(kStatus) kTcpClient_SetWriteTimeout(kTcpClient client, k64u timeout);

/** 
 * Sets the timeout duration for read operations. 
 *
 * By default, kTcpClient objects do not use a timeout interval and can block indefinitely.
 *
 * @public              @memberof kTcpClient
 * @param   client      TCP client object. 
 * @param   timeout     Timeout value, in microseconds.
 * @return              Operation status. 
 */
kFx(kStatus) kTcpClient_SetReadTimeout(kTcpClient client, k64u timeout);

/** 
 * Can be used to disable the Nagle algorithm.
 *
 * The Nagle algorithm is enabled by default. When enabled, small segments of outbound 
 * data are coalesced over a brief time period in order to improve network efficiency. 
 *
 * @public              @memberof kTcpClient
 * @param   client      TCP client object. 
 * @param   noDelay     kTRUE to disable the Nagle algorithm; kFALSE to enable. 
 * @return              Operation status. 
 */
kFx(kStatus) kTcpClient_SetNoDelay(kTcpClient client, kBool noDelay);

/** 
 * Enables limited support for forward/reverse seek operations. 
 *
 * kTcpClient supports seek operations with respect to the stream read pointer. The stream write 
 * pointer is unaffected. Seek operations can be relative to kSEEK_ORIGIN_BEGIN or kSEEK_ORIGIN_CURRENT. 
 * The value reported by the kStream_BytesRead method is affected by seek operations.
 * 
 * Seek support is most commonly required when client logic that was written for a random-access 
 * stream (e.g., kFile) will be reused with a kTcpClient object. Forward seek operations are relatively 
 * straightforward. Reverse seek operations are more complex and limited, but can still occasionally
 * be useful (e.g., to read a file header for file-type detection, and then restart from the beginning). 
 *
 * Forward seek is supported by reading and discarding bytes from the underlying stream. There is no limit 
 * to forward seek distance.  
 * 
 * Reverse seek is supported by "unreading" bytes from the client buffer. Accordingly, in order to support 
 * reverse seek, kTcpClient_SetReadBuffers must be invoked with a non-zero clientSize argument. Reverse 
 * seek distance is limited by the number of buffered bytes that have previously been read, but have not yet 
 * been overwritten by subsequent operations affecting the read buffer.  In client code that uses 
 * reverse seek, it is recommended to call kStream_Flush just prior to reading bytes that will later be 
 * reversed. Doing so ensures that maximum buffer space will available for reverse seek. 
 *
 * Seek support can slightly reduce stream efficiency and is therefore disabled by default.
 * 
 * @public              @memberof kTcpClient
 * @param   client      TCP client object. 
 * @param   enabled     kTRUE to enable forward/reverse seek support. 
 * @return              Operation status. 
 */
kFx(kStatus) kTcpClient_EnableSeek(kTcpClient client, kBool enabled);

/** 
 * Sets a cancel query handler, which can be used to asynchronously terminate read/write operations.
 * 
 * kTcpClient_SetCancelHandler and kTcpClient_Cancel represent alternative approaches to I/O cancellation; 
 * use one or the other approach, but not both. 
 * 
 * @public              @memberof kTcpClient
 * @param   client      TCP client object. 
 * @param   function    Callback function.
 * @param   receiver    Callback context.
 * @return              Operation status. 
 */
kFx(kStatus) kTcpClient_SetCancelHandler(kTcpClient client, kCallbackFx function, kPointer receiver);

/** 
 * Cancels any pending I/O operations. 
 * 
 * kTcpClient_SetCancelHandler and kTcpClient_Cancel represent alternative approaches to I/O cancellation; 
 * use one or the other approach, but not both. 
 *
 * This method is thread-safe.
 * 
 * @public              @memberof kTcpClient
 * @return              Operation status. 
 */
kFx(kStatus) kTcpClient_Cancel(kTcpClient client);

/** 
 * Connects to a remote end point. 
 * 
 * Connection can be attempted only once per client object.
 *
 * @public              @memberof kTcpClient
 * @param   client      TCP client object. 
 * @param   address     The remote IP address.  
 * @param   port        The remote port number.
 * @param   timeout     Timeout interval, in microseconds.
 * @return              Operation status. 
 */
kFx(kStatus) kTcpClient_Connect(kTcpClient client, kIpAddress address, k32u port, k64u timeout); 

/** 
 * Begins connecting to a remote end point. 
 *
 * This method can be used in conjuction with kTcpClient_EndConnect to enable useful work to be performed 
 * while waiting for connection to complete. If BeginConnect completes successfully, the EndConnect 
 * method must be called at some point thereafter. After calling BeginConnect, no other methods may be 
 * called on this object before EndConnect (though the object could be safely destroyed, if desired). 
 * 
 * Connection can be attempted only once per client object.
 *
 * @public              @memberof kTcpClient
 * @param   client      TCP client object. 
 * @param   address     The remote IP address. 
 * @param   port        The remote port number.
 * @return              Operation status. 
 * @see                 kTcpClient_EndConnect
 */
kFx(kStatus) kTcpClient_BeginConnect(kTcpClient client, kIpAddress address, k32u port); 

/** 
 * Completes a connection operation that was started with kTcpClient_BeginConnect. 
 *
 * @public              @memberof kTcpClient
 * @param   client      TCP client object. 
 * @param   timeout     The timeout interval, in microseconds. 
 * @return              Operation status. 
 * @see                 kTcpClient_BeginConnect
 */
kFx(kStatus) kTcpClient_EndConnect(kTcpClient client, k64u timeout); 

/** 
 * Begins shutdown of communication.
 *
 * Shutdown will occur automatically when the client is destroyed. This method can be used to 
 * explicitly initiate shutdown before the client is destroyed. 
 * 
 * After shutdown, any additional I/O operations will fail. 
 * 
 * @public              @memberof kTcpClient
 * @param   client      TCP client object. 
 * @return              Operation status. 
 */
kFx(kStatus) kTcpClient_Shutdown(kTcpClient client); 

/** 
 * Waits until the client has bytes to read or until the specified timeout period elapses. 
 * 
 * This function will return kERROR_TIMEOUT in the event that the client is not ready for reading by 
 * the end of the timeout period. 
 *
 * @public              @memberof kTcpClient
 * @param   client      TCP client object. 
 * @param   timeout     Timeout interval, in microseconds.
 * @return              Operation status. 
 */
kFx(kStatus) kTcpClient_Wait(kTcpClient client, k64u timeout);

/** 
 * Returns the underlying kSocket object.
 *
 * @public              @memberof kTcpClient
 * @param   client      TCP client object. 
 * @return              Operation status. 
 */
kFx(kSocket) kTcpClient_Socket(kTcpClient client); 

/** 
 * Returns the number of bytes currently enqueued and available for reading. 
 * 
 * This function returns the count of bytes enqueued in the client's internal read buffer. This does 
 * not include any data enqueued in the underlying socket's read buffer.
 *
 * @public              @memberof kTcpClient
 * @param   client      TCP client object. 
 * @return              Count of bytes available for reading.
 */
kFx(kSize) kTcpClient_Available(kTcpClient client); 

/** 
 * Returns the local end point for a connected client.
 * 
 * @public              @memberof kTcpClient
 * @param   client      TCP client object. 
 * @param   endPoint    Local end point.
 * @return              Operation status. 
 */
kFx(kStatus) kTcpClient_LocalEndPoint(kTcpClient client, kIpEndPoint* endPoint);

/** 
 * Returns the remote end point for a connected client. 
 * 
 * @public              @memberof kTcpClient
 * @param   client      TCP client object. 
 * @param   endPoint    Remote end point.
 * @return              Operation status. 
 */
kFx(kStatus) kTcpClient_RemoteEndPoint(kTcpClient client, kIpEndPoint* endPoint); 

/** 
 * Reports any internal errors that will prevent success of future communication attempts.
 * 
 * @public              @memberof kTcpClient
 * @param   client      TCP client object. 
 * @return              Client status. 
 */
kFx(kStatus) kTcpClient_Status(kTcpClient client); 

#endif

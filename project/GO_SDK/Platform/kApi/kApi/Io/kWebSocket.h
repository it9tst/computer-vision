/** 
 * @file    kWebSocket.h
 * @brief   Declares the kWebSocket class. 
 *
 * @internal
 * Copyright (C) 2017-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_WEBSOCKET_H
#define K_API_WEBSOCKET_H

#include <kApi/kApiDef.h>
#include <kApi/Io/kNetwork.h>

/**
 * @struct  kWebSocketDataType
 * @extends kValue
 * @ingroup kApi-Io
 * @brief   Represents Websocket message content type.
 */
typedef k32s kWebSocketDataType; 

/** @relates kWebSocketDataType @{ */
#define kWEB_SOCKET_DATA_TYPE_NULL                    (0x0)       ///< Unknown content type.
#define kWEB_SOCKET_DATA_TYPE_UTF8                    (0x1)       ///< UTF-8 content.
#define kWEB_SOCKET_DATA_TYPE_BINARY                  (0x2)       ///< Binary content.
/** @} */

#include <kApi/Io/kWebSocket.x.h>

/**
 * @class   kWebSocket
 * @extends kStream
 * @ingroup kApi-Io
 * @brief   Represents a WebSocket.
 *
 * The kWebSocket class implements support for WebSocket protocol (RFC 6455). This protocol enables client 
 * software, typically running in a web browser, to communicate with servers more efficiently than would otherwise 
 * be possible over HTTP.  
 * 
 * kWebSocket can be used in either a client or a server role. In a server context, a kWebSocket instance can be created 
 * from an HTTP request as follows: 
 * 
 @code {.c}
kStatus App_HttpRequestHandler(App app, kHttpServer server, kHttpServerChannel channel)
{
    kWebSocket webSocket = kNULL;
    kStatus status; 

    if (kHttpServerRequest_IsWebSocketUpgrade(kHttpServerChannel_Request(channel)))
    {
        kCheck(kHttpServerChannel_ProcessWebSocketRequest(channel, kTRUE, &webSocket)); 

        if (!kIsNull(webSocket))
        {
            //create a thread to handle the websocket connection
            ...
        }
    }
 
    return kOK; 
}
 @endcode
 * 
 * The WebSocket protocol is message-based. kWebSocket supports reading/writing complete messages using 
 * the kWebSocket_ReadMessage and kWebSocket_WriteMessage methods. 
 * 
 * The WebSocket protocol also supports subdividing messages into multiple frames, enabling endpoints to 
 * stream messages of unknown length. Accordingly, kWebSocket inherits from the kStream base class
 * and includes special methods that can be used in conjunction with kStream methods
 * to support streaming reads/writes.
 @code {.c}
//Example: streaming read
kStatus ReadMessage(kWebSocket webSocket)
{
    kByte buffer[256]; 
    kSize bytesRead;

    //wait for a new message to arrive (1s timeout)
    kCheck(kWebSocket_Receive(webSocket, 1000000)); 

    //read out the message; kStream_ReadSome will return kERROR_NOT_FOUND at the end of the message
    while (kSuccess(kStream_ReadSome(webSocket, &buffer[0], 1, sizeof(buffer), &bytesRead)))
    {
        //process content
        ...
    }

    return kOK; 
}

//Example: object serialization
kStatus WriteObject(kWebSocket webSocket, kObject object)
{
    kDat6Serializer writer = kNULL; 

    kTry
    {
        //attach serializer to websocket stream
        kTest(kDat6Serializer_Construct(&writer, webSocket, kNULL));      

        //serialize object; will write to websocket, generating frames as needed
        kTest(kSerializer_WriteObject(writer, object)); 

        //end the current websocket message (send final frame) and flush 
        kTest(kWebSocket_Send(webSocket));       
    }
    kFinally
    {
        kObject_Destroy(writer); 

        kEndFinally(); 
    }

    return kOK; 
}

 @endcode
 */
//typedef kStream kWebSocket;         --forward-declared in kApiDef.x.h 

/** 
 * Constructs a WebSocket that can be used to connect to a server. 
 *
 * @public              @memberof kWebSocket
 * @param   webSocket   Destination for constructed object handle. 
 * @param   allocator   Memory allocator (or kNULL for default). 
 * @return              Operation status. 
 */
kFx(kStatus) kWebSocket_Construct(kWebSocket* webSocket, kAlloc allocator);

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
 * @public              @memberof kWebSocket
 * @param   webSocket   WebSocket object. 
 * @param   socketSize  Size of the write buffer maintained by the underlying socket (-1 to leave unchanged). 
 * @param   clientSize  Size of the write buffer maintained by the client object (-1 to leave unchanged).
 * @return              Operation status. 
 */
kFx(kStatus) kWebSocket_SetWriteBuffers(kWebSocket webSocket, kSSize socketSize, kSSize clientSize);

/** 
 * Sets the size of the read buffers. 
 *
 * Socket buffers decouple the sender and receiver, so that the sender does not need to block
 * while waiting for the receiver to receive all bytes. Client buffers improve the efficiency 
 * of the client when performing several small read operations. 
 *
 * @public              @memberof kWebSocket
 * @param   webSocket   WebSocket object. 
 * @param   socketSize  Size of the read buffer maintained by the underlying socket (-1 to leave unchanged).
 * @param   clientSize  Size of the read buffer maintained by the client object (-1 to leave unchanged).
 * @return              Operation status. 
 */
kFx(kStatus) kWebSocket_SetReadBuffers(kWebSocket webSocket, kSSize socketSize, kSSize clientSize);

/** 
 * Connects a WebSocket to an HTTP server. 
 *
 * @public              @memberof kWebSocket
 * @param   webSocket   WebSocket object. 
 * @param   address     HTTP server address.
 * @param   port        HTTP server port number.
 * @param   host        HTTP request host (distinguishes between multiple hosts at same address). 
 * @param   uri         HTTP request URI.
 * @param   timeout     Timeout, in microseconds.
 * @return              Operation status. 
 */
kFx(kStatus) kWebSocket_Connect(kWebSocket webSocket, kIpAddress address, k32u port, const kChar* host, const kChar* uri, k64u timeout);

/** 
 * Closes the WebSocket connection. 
 *
 * Use of this method is optional. Websockets can technically be closed by destroying the 
 * underlying TCP client. The Close method (which implements a closing handshake) is 
 * recommended in RFC 6455 to achieve a "clean" close. 
 * 
 * @public              @memberof kWebSocket
 * @param   webSocket   WebSocket object. 
 * @return              Operation status.
 */
kFx(kStatus) kWebSocket_Close(kWebSocket webSocket);

/** 
 * Sets the data transfer type used for send operations. 
 * 
 * In UTF-8 mode, it is the caller's responsibility to ensure that message 
 * data is valid UTF-8.
 * 
 * If this method is not called, kWebSocket defaults to Binary mode. 
 * 
 * The send type cannot be changed while a message is in progress (i.e., between 
 * kStream_Write and kWebSocket_Send). 
 * 
 * @public              @memberof kWebSocket
 * @param   webSocket   WebSocket object. 
 * @param   type        Frame type. 
 * @return              Operation status. 
 */
kFx(kStatus) kWebSocket_SetSendType(kWebSocket webSocket, kWebSocketDataType type);

/** 
 * Sends any buffered data and ends the current message. 
 * 
 * Websocket communication consists of <em>messages</em>, where each message can be transmitted 
 * in one or more <em>frames</em>. 
 * 
 * The kStream_Write method can be used to write data to a WebSocket. If the WebSocket 
 * has a client buffer and the buffer is filled, the buffered data will be sent as a WebSocket frame. 
 * If the WebSocket does not have a client buffer, each call to kStream_Write will generate a 
 * WebSocket frame. 
 * 
 * The kWebSocket_Send method generates an end-of-message frame that includes any data remaining
 * in the WebSocket buffer. If no data remains, an empty end-of-message frame will be generated. 
 * All message data is flushed to the underlying TCP object as part of kWebSocket_Send; 
 * the kStream_Flush method has no effect.  
 * 
 * @public              @memberof kWebSocket
 * @param   webSocket   WebSocket object. 
 * @return              Operation status. 
 */
kFx(kStatus) kWebSocket_Send(kWebSocket webSocket);

/** 
 * Blocks until a new message is available for reading.  
 *
 * The Receive method should be used to wait for a new message. Once a new message 
 * has been detected, kStream methods can be used to read out the data. When the message
 * has been completely read out, additional kStream read calls will return kERROR_NOT_FOUND. 
 * At this point, kWebSocket_Receive can be used to wait for another message. 
 * 
 * Multi-frame WebSocket messages will be treated as a single message. If kWebSocket_Receive 
 * is called before the previous message has been completely read out, any remaining data 
 * in the previous message will be discarded before waiting for a new message. 
 *
 * If this function returns successfully, the kWebSocket_ReceiveType method can optionally be 
 * used to determine the type of incoming data. 
 *  
 * @public              @memberof kWebSocket
 * @param   webSocket   WebSocket object. 
 * @param   timeout     Timeout, in microseconds.
 * @return              Operation status. 
 */
kFx(kStatus) kWebSocket_Receive(kWebSocket webSocket, k64u timeout);

/** 
 * Writes a complete WebSocket message.
 *
 * This method can be used as an alternative to kStream_Write/kWebSocket_Send.
 * 
 * This method can be used to generate a WebSocket message from a single input buffer. 
 * The WebSocket's internal client buffer is bypassed. 
 *
 * @public              @memberof kWebSocket
 * @param   webSocket   WebSocket object. 
 * @param   buffer      Bytes to be written.
 * @param   size        Count of bytes to be written.
 * @return              Operation status. 
 */
kFx(kStatus) kWebSocket_WriteMessage(kWebSocket webSocket, const void* buffer, kSize size);

/** 
 * Blocks until a whole message has been read (or until a timeout occurs). 
 * 
 * This method can be used as an alternative to kWebSocket_Receive/kStream_Read.
 * 
 * The 'memory' argument provides an interface to accept the received message data. This 
 * enables the caller to accept a message without knowing the message size beforehand. A 
 * kMemory object can be used to accept data into an arbitrary memory location. 
 * 
 * The timeout parameter determines the length of time to wait for the first message 
 * header to arrive.
 *
 * @public              @memberof kWebSocket
 * @param   webSocket   WebSocket object. 
 * @param   memory      Memory stream object that will receive message data.
 * @param   timeout     Timeout, in microseconds.
 * @return              Operation status. 
 */
kFx(kStatus) kWebSocket_ReadMessage(kWebSocket webSocket, kMemory memory, k64u timeout);

/** 
 * Cancels any pending I/O operations. 
 * 
 * This method is thread-safe.
 * 
 * @public              @memberof kWebSocket
 * @return              Operation status. 
 */
kFx(kStatus) kWebSocket_Cancel(kWebSocket webSocket);

/** 
 * Sends a ping message. 
 * 
 * Use the LastPong method to determine if/when a corresponding pong message 
 * has been received.
 *
 * @public              @memberof kWebSocket
 * @param   webSocket   WebSocket object. 
 * @return              Operation status.
 */
kFx(kStatus) kWebSocket_SendPing(kWebSocket webSocket);

/** 
 * Reports the time at which the most recent pong message arrived. 
 * 
 * @public              @memberof kWebSocket
 * @param   webSocket   WebSocket object. 
 * @return              Time at which last pong arrived, in microseconds (or 0, if never). 
 */
kFx(k64u) kWebSocket_LastPong(kWebSocket webSocket);

/** 
 * Reports the message data type currently used for sending.
 * 
 * @public              @memberof kWebSocket
 * @param   webSocket   WebSocket object. 
 * @return              Frame type used for sending.
 */
kFx(kWebSocketDataType) kWebSocket_SendType(kWebSocket webSocket);

/** 
 * Reports the data type of the most recently received data message. 
 * 
 * @public              @memberof kWebSocket
 * @param   webSocket   WebSocket object. 
 * @return              Frame type of the last received message.
 */
kFx(kWebSocketDataType) kWebSocket_ReceiveType(kWebSocket webSocket);

#endif

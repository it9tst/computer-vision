// 
// KWebSocket.h
// 
// Copyright (C) 2017-2021 by LMI Technologies Inc.
// Licensed under the MIT License.
// Redistributed files must retain the above copyright notice.
// 
#ifndef K_API_NET_WEB_SOCKET_H
#define K_API_NET_WEB_SOCKET_H

#include <kApi/Io/kWebSocket.h>
#include "kApiNet/KAlloc.h"
#include "kApiNet/Io/KMemory.h"

namespace Lmi3d
{
    namespace Zen 
    {
        namespace Io
        {
            /// <summary>Represents frame type for WebSockets.</summary>
            public value struct KWebSocketDataType
            {
                KDeclareEnum(KWebSocketDataType, kWebSocketDataType)

                    /// <summary>Unknown frame type.</summary>
                    literal k32s DATA_TYPE_NULL = kWEB_SOCKET_DATA_TYPE_NULL;
                    /// <summary>UTF-8 frame type.</summary>
                    literal k32s DATA_TYPE_UTF8 = kWEB_SOCKET_DATA_TYPE_UTF8;
                    /// <summary>Binary frame type.</summary>
                    literal k32s DATA_TYPE_BINARY = kWEB_SOCKET_DATA_TYPE_BINARY;
            };
     
            /// <summary>Represents a websocket client. <para/> Requires manual disposal.</summary>
            public ref class KWebSocket : public KStream
            {
                KDeclareClass(KWebSocket, kWebSocket)

            public:
                /// <summary>Initializes a new instance of the KWebSocket class with the specified Zen object handle.</summary>           
                /// <param name="handle">Zen object handle.</param>
                KWebSocket(IntPtr handle)
                    : KStream(handle, DefaultRefStyle)
                {}

                /// <summary>Initializes a new instance of the KWebSocket class.</summary>           
                KWebSocket()
                     : KStream(DefaultRefStyle)
                {
                    kWebSocket handle = kNULL;

                    KCheck(kWebSocket_Construct(&handle, kNULL));

                    Handle = handle;
                }

                /// <inheritdoc cref="KWebSocket()" />
                /// <param name="allocator">Memory allocator</param>
                KWebSocket(KAlloc^ allocator)
                     : KStream(DefaultRefStyle)
                {
                    kWebSocket handle = kNULL;

                    KCheck(kWebSocket_Construct(&handle, KToHandle(allocator))); 

                    Handle = handle;
                }
                    
                /// <summary>Sets the size of write buffers.</summary>
                /// 
                /// <remarks>
                /// <para>Socket buffers decouple the sender and receiver, so that the sender does not need to block
                /// while waiting for the receiver to receive all bytes.Client buffers improve the efficiency
                /// of the client when performing several small write operations.</para>
                ///
                /// <para>By default, the client buffer size is zero and the socket buffer size is determined by the
                /// underlying operating system.</para>
                /// </remarks>
                /// 
                /// <param name="socketSize">Size of the write buffer maintained by the underlying socket (-1 to leave unchanged).</param>
                /// <param name="clientSize">Size of the write buffer maintained by the client object (-1 to leave unchanged).</param>
                void SetWriteBuffers(kSSize socketSize, kSSize clientSize)
                {
                    KCheck(kWebSocket_SetWriteBuffers(Handle, socketSize, clientSize));
                }

                /// <summary>Sets the size of read buffers.</summary>
                /// 
                /// <remarks>
                /// Socket buffers decouple the sender and receiver, so that the sender does not need to block
                /// while waiting for the receiver to receive all bytes.Client buffers improve the efficiency
                /// of the client when performing several small write operations.
                /// </remarks>
                /// 
                /// <param name="socketSize">Size of the read buffer maintained by the underlying socket (-1 to leave unchanged).</param>
                /// <param name="clientSize">Size of the read buffer maintained by the client object (-1 to leave unchanged).</param>
                void SetReadBuffers(kSSize socketSize, kSSize clientSize)
                {
                    KCheck(kWebSocket_SetReadBuffers(Handle, socketSize, clientSize));
                }

                /// <summary>Connects a WebSocket to an HTTP server.</summary>
                /// 
                /// <param name="address">HTTP server address.</param>
                /// <param name="port">HTTP server port number.</param>
                /// <param name="host">HTTP request host (distinguishes between multiple hosts at same address).</param>
                /// <param name="uri">HTTP request URI.</param>
                /// <param name="timeout">Timeout, in microseconds.</param>
                void Connect(KIpAddress address, k32u port, String^ host, String^ uri, k64s timeout)
                {
                    KString hostStr(host);
                    KString uriStr(uri);

                    KCheck(kWebSocket_Connect(Handle, (kIpAddress)address, port, hostStr.CharPtr, uriStr.CharPtr, (k64u)timeout));
                }

                /// <summary>Closes the WebSocket connection.</summary>
                /// 
                /// <remarks>
                /// Use of this method is optional. Websockets can technically be closed by destroying the 
                /// underlying TCP client.The Close method(which implements a closing handshake) is
                /// recommended in RFC 6455 to achieve a "clean" close.
                /// </remarks>
                void Close(kWebSocket webSocket)
                {
                    KCheck(kWebSocket_Close(Handle));
                }

                /// <summary>Sets the data transfer type used for send operations. </summary>
                /// 
                /// <remarks>
                /// <para>If the type is kWEB_SOCKET_DATA_TYPE_UTF8, any data sent should be valid UTF-8.</para>
                ///
                /// <para>The data type cannot be changed while a message is being streamed (i.e., between 
                /// kStream_Write and kWebSocket_Send).</para>
                /// </remarks>
                /// <param name="type">Frame type.</param>
                void SetSendType(KWebSocketDataType type)
                {
                    KCheck(kWebSocket_SetSendType(Handle, type));
                }

                /// <summary>Sends any buffered data and ends the current message.</summary>
                /// 
                /// <remarks>
                /// <para>Websocket communication consists of messages, where each message can be transmitted
                /// in one or more frames.</para>
                /// <para>The kStream_Write method can be used to write data to a WebSocket.If the WebSocket
                /// has a client buffer and the buffer is filled, the buffered data will be sent as a WebSocket frame.
                /// If the WebSocket does not have a client buffer, each call to kStream_Write will generate a
                /// WebSocket frame.</para>
                /// <para>The kWebSocket_Send method generates an end - of - message frame that includes any data remaining
                /// in the WebSocket buffer.If no data remains, an empty end - of - message frame will be generated.
                /// All message data is automatically flushed to the underlying TCP object as part of kWebSocket_Send;
                /// the stream does not need to be explicitly flushed.</para>
                /// </remarks>
                void Send()
                {
                    KCheck(kWebSocket_Send(Handle));
                }

                /// <summary>Blocks until a new message is available for reading.</summary>
                /// 
                /// <remarks>
                /// <para>The Receive method should be used to wait for a new message. Once a new message 
                /// has been detected, kStream methods can be used to read out the data.When the message
                /// has been completely read out, additional kStream read calls will return kERROR_NOT_FOUND.
                /// At this point, kWebSocket_Receive can be used to wait for another message. </para>
                /// <para>Multi-frame WebSocket messages will be treated as a single message. If kWebSocket_Receive 
                /// is called before the previous message has been completely read out, any remaining data
                /// in the previous message will be discarded before waiting for a new message.</para>
                /// <para>If this function returns successfully, the kWebSocket_ReceiveType method can optionally be 
                /// used to determine the type of incoming data.</para>
                /// </remarks>
                /// <param name="timeout">Timeout, in microseconds.</param>
                void Receive(k64s timeout)
                {
                    KCheck(kWebSocket_Receive(Handle, (k64u)timeout));
                }

                /// <summary>Writes a complete WebSocket message.</summary>
                /// 
                /// <remarks>
                /// <para>This method can be used as an alternative to kStream_Write/kWebSocket_Send.</para>
                /// <para>This method can be used to generate a WebSocket message from a single input buffer. 
                /// The WebSocket's internal client buffer is bypassed.</para>
                /// </remarks>
                /// <param name="buffer">Bytes to be written.</param>
                /// <param name="size">Count of bytes to be written.</param>
                void WriteMessage(IntPtr buffer, k64s size)
                {
                    KCheck(kWebSocket_WriteMessage(Handle, buffer.ToPointer(), (kSize)size));
                }

                /// <inheritdoc cref="WriteMessage(IntPtr, k64s)" />
                void WriteMessage(array<kByte>^ buffer, k32s size)
                {
                    pin_ptr<Byte> pinnedBuffer = &buffer[0]; 

                    WriteMessage(IntPtr(pinnedBuffer), size);
                }

                /// <summary>Blocks until a whole message has been read (or until a timeout occurs).</summary>
                /// 
                /// <remarks>
                /// <para>This method can be used as an alternative to kWebSocket_Receive/kStream_Read.</para>
                /// <para>The 'memory' argument provides an interface to accept the received message data. This 
                /// enables the caller to accept a message without knowing the message size beforehand.A
                /// kMemory object can be used to accept data into an arbitrary memory location.</para>
                /// <para>The timeout parameter determines the length of time to wait for the first message 
                /// header to arrive.</para>
                /// </remarks>
                /// <param name="memory">Memory stream object that will receive message data.</param>
                /// <param name="timeout">Timeout, in microseconds.</param>
                void ReadMessage(KMemory^ memory, k64u timeout)
                {
                    KCheck(kWebSocket_ReadMessage(Handle, KToHandle(memory), timeout));
                }

                /// <summary>Cancels any pending I/O operations. </summary>
                /// 
                /// <remarks>
                /// This method is thread-safe.
                /// </remarks>
                void Cancel()
                {
                     KCheck(kWebSocket_Cancel(Handle));
                }

                /// <summary>Sends a ping message.</summary>
                /// 
                /// <remarks>
                /// Use the LastPong method to determine if/when a corresponding pong message 
                /// has been received.
                /// </remarks>
                void SendPing()
                {
                    KCheck(kWebSocket_SendPing(Handle));
                }

                /// <summary>Reports the time at which the most recent pong message arrived.</summary>
                /// 
                /// <returns>Time at which last pong arrived, in microseconds (or 0, if never).</returns>
                property k64u LastPong
                {
                    k64u get() { return kWebSocket_LastPong(Handle); }
                }

                /// <summary>Reports the message data type currently used for sending.</summary>
                /// 
                /// <returns>Frame type used for sending.</returns>
                KWebSocketDataType SendType()
                {
                    return kWebSocket_SendType(Handle);
                }

                /// <summary>Reports the data type of the most recently received data message. </summary>
                /// 
                /// <returns>Frame type of the last received message.</returns>
                property KWebSocketDataType ReceiveType
                {
                    KWebSocketDataType get() { return kWebSocket_ReceiveType(Handle); }
                }

            protected:

            private:
                
            };
        }
    }
}

#endif

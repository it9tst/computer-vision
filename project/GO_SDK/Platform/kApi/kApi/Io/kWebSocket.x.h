/** 
 * @file    kWebSocket.x.h
 *
 * @internal
 * Copyright (C) 2017-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_WEBSOCKET_X_H
#define K_API_WEBSOCKET_X_H

#include <kApi/Io/kStream.h>
#include <kApi/Io/kTcpClient.h>

kDeclareEnumEx(k, kWebSocketDataType, kValue)

#define xkWEB_SOCKET_HANDSHAKE_GUID              "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"          // GUID used in HTTP upgrade handshake (defined by RFC 6455)

#define xkWEB_SOCKET_OP_CODE_CONTINUE            (0x00)              //frame is a continuation of previous data frame
#define xkWEB_SOCKET_OP_CODE_UTF8                (0x01)              //first frame in a sequence of utf8 data frames
#define xkWEB_SOCKET_OP_CODE_BINARY              (0x02)              //first frame in a sequence of binary data frames
#define xkWEB_SOCKET_OP_CODE_CLOSE               (0x08)              //close request and/or response frame
#define xkWEB_SOCKET_OP_CODE_PING                (0x09)              //ping frame (expects pong response)
#define xkWEB_SOCKET_OP_CODE_PONG                (0x0A)              //pong frame (pong response, or unsolicted)

#define xkWEB_SOCKET_16_BIT_PAYLOAD              (126)               //payload code indicating 16-bit payload size (bits 6..0 in header byte 1) 
#define xkWEB_SOCKET_64_BIT_PAYLOAD              (127)               //payload code indicating 64-bit payload size (bits 6..0 in header byte 1)

kInlineFx(k8u) xkWebSocket_SetFin(kBool fin)            { return (k8u) (((fin) & 0x01) << 7); }       //sets end-of-message (FIN) bit in header 0
kInlineFx(k8u) xkWebSocket_SetOpCode(k8u op)            { return (k8u) (((op)  & 0x0F) << 0); }       //sets opcode field in header 0
kInlineFx(k8u) xkWebSocket_SetMaskEnabled(kBool en)     { return (k8u) (((en)  & 0x01) << 7); }       //sets mask enabled bit in header 1
kInlineFx(k8u) xkWebSocket_SetPayloadCode(k8u pay)      { return (k8u) (((pay) & 0x7F) << 0); }       //sets payload code field in header 1                                                       

kInlineFx(kBool) xkWebSocket_Fin(k8u h0)                { return (kBool) (((h0) >> 7) & 0x01); }      //gets end-of-message (FIN) bit from header 0
kInlineFx(k8u) xkWebSocket_OpCode(k8u h0)               { return (k8u)   (((h0) >> 0) & 0x0F); }      //gets opcode field from header 0
kInlineFx(kBool) xkWebSocket_MaskEnabled(k8u h1)        { return (kBool) (((h1) >> 7) & 0x01); }      //gets mask enabled from in header 1
kInlineFx(kSize) xkWebSocket_PayloadCode(k8u h1)        { return (kSize) (((h1) >> 0) & 0x7F); }      //gets payload code field from header 1

#define xkWEB_SOCKET_MAX_CONTROL_PAYLOAD         (0x7D)              //maximum payload size for control messages, as stipulated by RFC 6455

#define xkWEB_SOCKET_BACKGROUND_TIMER_PERIOD     (200000)            //period of background timer that polls for incoming control messages (e.g, ping, close), in microseconds

#define xkWEB_SOCKET_CLOSE_TIMEOUT               (5000000)           //maximum amount of time to wait for remote host to respond to close command, in microseconds

#define xkWEB_SOCKET_MAX_HTTP_RESPONSE_LENGTH    (8*1024)            //maximum expected length of HTTP websocket upgrade response, in bytes

#define xkWEB_SOCKET_MASK_BUFFER_SIZE            (32*1024)           //size of temp buffer used for masking outbound data

#define xkWEB_SOCKET_TCP_CLIENT_READ_BUFFER      (16)                //client read buffer for underlying kTcpClient object; just large enough for frame headers
#define xkWEB_SOCKET_TCP_CLIENT_WRITE_BUFFER     (0)                 //client write buffer for underlying kTcpClient object; no buffer needed (kSerializer has built-in write buffer)

//connection state of kWebSocket
typedef k32s xkWebSocketState; 

#define xkWEB_SOCKET_STATE_INITIALIZED           (0)                 //created, not yet connected
#define xkWEB_SOCKET_STATE_CONNECTING            (1)                 //TCP connection established, not yet upgraded
#define xkWEB_SOCKET_STATE_CONNECTED             (2)                 //connected, upgraded, ready to use
#define xkWEB_SOCKET_STATE_CLOSING               (3)                 //close request sent
#define xkWEB_SOCKET_STATE_CLOSED                (4)                 //closed (close request received, or TCP client closed)

#define xkWEB_SOCKET_PROTOCOL_VERSION            (13)                //protocol version supported; correspoonds to RFC 6455 

//receive state
typedef struct kWebSocketRecvInfo
{
    k8u opCode;                         //last received opcode 
    kBool isMasked;                     //last received mask-enable field
    kBool isLastFrame;                  //last received FIN bit 
    kSize payloadSize;                  //last received payload size
    kByte maskKey[4];                   //last received mask key

    kBool inMessage;                    //are we currently in a data frame sequence?
    kBool inDataFrame;                  //are we currently reading out a data frame?
    kBool isFirstDataFrame;             //is the current frame the first frame in a data frame sequence?
    kSize dataRead;                     //amount of payload data that has been read out from the current frame (if a data frame)
    kWebSocketDataType recvType;        //data type of the current data frame sequence
    kWebSocketDataType lastRecvType;    //data type latched at most recent call to kWebSocket_Receive
} kWebSocketRecvInfo;

//send state
typedef struct kWebSocketSendInfo
{
    kWebSocketDataType sendType;        //data type of the current data frame sequence
    kBool inMessage;                    //are we currently in a data frame sequence
} kWebSocketSendInfo;

typedef struct kWebSocketClass
{
    kStreamClass base; 

    kBool isClient;                     //does this object represent the client or server side?
    kBool shouldMaskSend;               //does outbound data need to be masked?
    xkWebSocketState state;              //current state of web socket

    kLock lock;                         //protects TCP client, send/recv info

    kTcpClient tcpClient;               //underlying TCP client
    kSerializer serializer;             //bound to TCP client; used to parse/format headers

    kArray1 maskBuffer;                 //buffer used to mask data, when masking required

    kWebSocketSendInfo sendInfo;        //state information for outbound data
    kWebSocketRecvInfo recvInfo;        //state information for inbound data

    k64u lastPongTime;                  //time at which last pong was detected

    kPeriodic backgroundTimer;          //periodic background timer to handle incoming control messages

} kWebSocketClass;

kDeclareClassEx(k, kWebSocket, kStream)

kFx(kStatus) xkWebSocket_ConstructFromRequest(kWebSocket* webSocket, kHttpServerChannel channel, kAlloc allocator);

kFx(kStatus) xkWebSocket_Init(kWebSocket webSocket, kType type, kHttpServerChannel channel, kAlloc allocator);
kFx(kStatus) xkWebSocket_VRelease(kWebSocket webSocket);

kFx(kStatus) xkWebSocket_VReadSomeImpl(kWebSocket webSocket, void* buffer, kSize minCount, kSize maxCount, kSize* bytesRead);
kFx(kStatus) xkWebSocket_VWriteImpl(kWebSocket webSocket, const void* buffer, kSize size);
kFx(kStatus) xkWebSocket_VFlush(kWebSocket webSocket);
kFx(kStatus) xkWebSocket_VFill(kWebSocket webSocket);

kFx(kStatus) xkWebSocket_SendHttpUpgradeRequest(kWebSocket webSocket, const kChar* host, k32u port, const kChar* uri, const kByte* key, kSize keyLength); 
kFx(kStatus) xkWebSocket_ReceiveHttpUpgradeResponse(kWebSocket webSocket, const kByte* key, kSize keyLength); 
kFx(kStatus) xkWebSocket_ProcessHttpUpgradeRequest(kWebSocket webSocket, kHttpServerChannel channel); 

kFx(kStatus) xkWebSocket_WriteDataFrame(kWebSocket webSocket, const kByte* buffer, kSize count, kBool finalFrame); 
kFx(kStatus) xkWebSocket_WriteFrame(kWebSocket webSocket, const kByte* buffer, kSize count, k8u opCode, kBool finalFrame); 
kFx(kStatus) xkWebSocket_WriteHeader(kWebSocket webSocket, k8u opCode, kBool finalFrame, kBool shouldMask, const kByte* maskKey, kSize maskKeySize, kSize payloadSize); 

kFx(kStatus) xkWebSocket_DiscardReceive(kWebSocket webSocket);
kFx(kStatus) xkWebSocket_BeginReceive(kWebSocket webSocket, k64u timeout);
kFx(kStatus) xkWebSocket_ProcessHeader(kWebSocket webSocket);
kFx(kStatus) xkWebSocket_ParseHeader(kWebSocket webSocket); 
kFx(kStatus) xkWebSocket_InterpretHeader(kWebSocket webSocket); 

kFx(kStatus) xkWebSocket_ProcessControlFrame(kWebSocket webSocket);
kFx(kStatus) xkWebSocket_ProcessPing(kWebSocket webSocket, const kByte* payload, kSize payloadSize);
kFx(kStatus) xkWebSocket_ProcessPong(kWebSocket webSocket, const kByte* payload, kSize payloadSize);
kFx(kStatus) xkWebSocket_ProcessClose(kWebSocket webSocket, const kByte* payload, kSize payloadSize);

kFx(kStatus) xkWebSocket_ReadAtLeast(kWebSocket webSocket, kByte* buffer, kSize minCount, kSize maxCount, kSize* bytesRead); 

kFx(kStatus) xkWebSocket_ApplyMask(kWebSocket webSocket, const kByte* src, kByte* dest, kSize count, kByte* maskKey, kSize maskKeySize, kSize initialPosition); 

kFx(kBool) xkWebSocket_IsValid(kWebSocket webSocket); 

kFx(kStatus) xkWebSocket_OnBackgroundTimer(kWebSocket webSocket, kPeriodic timer);

#endif

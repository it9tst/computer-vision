/** 
 * @file    kSocket.x.h
 *
 * @internal
 * Copyright (C) 2004-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_SOCKET_X_H
#define K_API_SOCKET_X_H

kDeclareEnumEx(k, kSocketType, kValue)
kDeclareEnumEx(k, kSocketEvent, kValue)
kDeclareClassEx(k, kSocket, kObject)

#define xkSOCKET_MAX_WAIT_GROUP              (64)
#define xkSOCKET_WAIT_COUNT                  (3)

#define xkSOCKET_MSG_PEEK                    (MSG_PEEK)

#define xkSOCKET_TYPE_NETLINK                (2)     ///< Netlink socket.

#if defined (K_PLATFORM)

#if defined (K_WINDOWS)
#   define xkSocketHandle                SOCKET
#   define xkSocket_Select               select
#   define xkSOCKET_INVALID_SOCKET       INVALID_SOCKET
#   define xkSOCKET_ERROR                SOCKET_ERROR
#   define xkSOCKET_WOULD_BLOCK          WSAEWOULDBLOCK
#   define xkSOCKET_SHUTDOWN_BOTH        SD_BOTH       
#   define xkSOCKET_IN_PROGRESS          (0)
#   define xkSOCKET_TIMEDOUT             WSAETIMEDOUT
#   define xkSOCKET_MSGSIZE              WSAEMSGSIZE
#   define xkSOCKET_MAX_IO_SIZE          (0x7FFFFFFF)
#   define xkSOCKET_MAX_BUFFER           (0xFFFFFFFF)

#elif defined (K_TI_BIOS)
#   define xkSocketHandle                SOCKET
#   define socklen_t                    int
#   define xkSocket_Select               fdSelect
#   define xkSOCKET_INVALID_SOCKET       INVALID_SOCKET
#   define xkSOCKET_ERROR                SOCKET_ERROR
#   define xkSOCKET_SHUTDOWN_BOTH        SHUT_RDWR
#   define xkSOCKET_WOULD_BLOCK          EWOULDBLOCK
#   define xkSOCKET_IN_PROGRESS          EINPROGRESS
#   define xkSOCKET_TIMEDOUT             ETIMEDOUT
#   define xkSOCKET_MSGSIZE              ENOMEM
#   define xkSOCKET_MAX_BUFFER           (0xFFFFFFFF)

#elif defined (K_VX_KERNEL)
#   define xkSocketHandle                unsigned int
#   define xkSocket_Select               select
#   define xkSOCKET_INVALID_SOCKET       (xkSocketHandle)(~0)
#   define xkSOCKET_ERROR                (ERROR)
#   define xkSOCKET_SHUTDOWN_BOTH        SHUT_RDWR
#   define xkSOCKET_WOULD_BLOCK          EWOULDBLOCK
#   define xkSOCKET_IN_PROGRESS          EINPROGRESS
#   define xkSOCKET_TIMEDOUT             EAGAIN
#   define xkSOCKET_MSGSIZE              ENOMEM
#   define xkSOCKET_MAX_BUFFER           (0xFFFFFFFF) 

#elif defined (K_POSIX) || defined (K_QNX)
#   define xkSocketHandle                unsigned int
#   define xkSocket_Select               select
#   define xkSOCKET_INVALID_SOCKET       (xkSocketHandle)(~0)
#   define xkSOCKET_ERROR                (-1)
#   define xkSOCKET_SHUTDOWN_BOTH        SHUT_RDWR
#   define xkSOCKET_WOULD_BLOCK          EWOULDBLOCK
#   define xkSOCKET_IN_PROGRESS          EINPROGRESS
#   define xkSOCKET_TIMEDOUT             EAGAIN
#   define xkSOCKET_MSGSIZE              ENOMEM
#   if defined (K_QNX)
#       define xkSOCKET_MAX_BUFFER       (0x0001FFFF)
#   else 
#       define xkSOCKET_MAX_BUFFER       (0xFFFFFFFF)
#   endif

#endif

typedef struct kSocketClass
{
    kObjectClass base;      
    kIpVersion ipVersion;           //Internet Protocol version.
    kSocketType socketType;         //Type of socket (TCP, UDP). 
    xkSocketHandle handle;          //OS socket handle. 
    kSocketEvent eventTypes;        //Events to use with socket wait operations.
    kSocketEvent eventStatus;       //Events detected during most recent wait call.
    kStatus status;                 //Current socket error status.
    kBool isBlocking;               //Is the socket in blocking mode?
    kBool wasBlocking;              //Used to save desired 'isBlocking' state in BeginConnect/EndConnect. 
    k32s beginConnectResult;        //Result of socket connect; used in BeginConnect/EndConnect.
    k32s beginConnectLastError;     //Result of GetLastError after socket connect; used in BeginConnect/EndConnect.
} kSocketClass;

/* 
* Private methods. 
*/

/**
* Reads one or more bytes without removing them from the socket buffer.
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
kFx(kStatus) xkSocket_Peek(kSocket socket, void* buffer, kSize size, kSize* read);

kFx(kStatus) xkSocket_WaitEx(kSocket socket, kSocketEvent events, k64u timeout);

kFx(kStatus) xkSocket_ConstructFromHandle(kSocket* sockt, kIpVersion ipVersion, kSocketType socktType, xkSocketHandle handle, kAlloc allocator); 
kFx(kStatus) xkSocket_Init(kSocket sockt, kType type, kIpVersion ipVersion, kSocketType socktType, kAlloc allocator); 
kFx(kStatus) xkSocket_InitFromHandle(kSocket sockt, kType type, kIpVersion ipVersion, kSocketType socktType, xkSocketHandle handle, kAlloc allocator); 
kFx(kStatus) xkSocket_VRelease(kSocket sockt); 

kFx(kStatus) xkSocket_ReadImpl(kSocket sockt, void* buffer, kSize size, kSize* read, k32s options); 

kFx(k32s) xkSocket_GetLastError();
kFx(kSSize) xkSocket_Recv(kSocket sockt, void* buffer, kSize size, k32s options);
kFx(kSSize) xkSocket_RecvFrom(kSocket sockt, void* buffer, kSize size, struct sockaddr_in* from);
kFx(kSSize) xkSocket_RecvMessage(kSocket sockt, void* buffer, kSize size, struct sockaddr_in* from, kSize* interfaceIndex);
kFx(kSSize) xkSocket_Send(kSocket sockt, const void* buffer, kSize size);
kFx(kSSize) xkSocket_SendTo(kSocket sockt, const void* buffer, kSize size, const struct sockaddr_in* to);
kFx(kStatus) xkSocket_ShutdownHandle(xkSocketHandle socktHandle);
kFx(kStatus) xkSocket_CloseHandle(xkSocketHandle socktHandle);
kFx(struct timeval*) xkSocket_FormatTimeVal(struct timeval* tv, k64u timeout); 

kFx(kStatus) xkSocket_SetWriteLowWater(kSocket socket, kSize size); 
kFx(kStatus) xkSocket_SetReadLowWater(kSocket socket, kSize size); 
kFx(kSize) xkSocket_WriteBuffer(kSocket sockt); 
kFx(kSize) xkSocket_ReadBuffer(kSocket sockt); 

kInlineFx(kStatus) xkSocket_Status(kSocket sockt)
{
    kObj(kSocket, sockt);

    return obj->status;
}

#endif

/* 
* Deprecated (Stage 1): not recommended for further use, but not yet announced via kDeprecate
*/

//[Deprecated] Not a public constant, but can be replaced with xkSOCKET_INVALID_SOCKET
#define kSOCKET_INVALID_SOCKET xkSOCKET_INVALID_SOCKET   

#endif

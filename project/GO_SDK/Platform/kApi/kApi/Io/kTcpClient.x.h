/** 
 * @file    kTcpClient.x.h
 *
 * @internal
 * Copyright (C) 2008-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_TCP_CLIENT_X_H
#define K_API_TCP_CLIENT_X_H

#include <kApi/Io/kStream.h>

#define xkTCP_CLIENT_CANCEL_QUERY_INTERVAL       (100000)

typedef struct kTcpClientClass
{
    kStreamClass base; 
    kSocket socket;                     //Socket object.        
    k64u writeTimeout;                  //Timeout for write operations (microseconds).
    k64u readTimeout;                   //Timeout for read operations (microseconds).
    kSize socketReadBufferSize;         //Size of read buffer used by underlying socket.
    kCallback cancelQuery;              //User-provided cancellation handler. 
    kAtomic32s isCancelled;             //Has I/O been cancelled by client owner?
    kBool timedOut;                     //Has client experienced a timeout error?
    kBool isSeekEnabled;                //Provide limited support for seek operations?
} kTcpClientClass;

kDeclareClassEx(k, kTcpClient, kStream)

/* 
* Private methods. 
*/

kFx(kStatus) xkTcpClient_ConstructFromSocket(kTcpClient* client, kSocket socket, kAlloc allocator); 
kFx(kStatus) xkTcpClient_Init(kTcpClient client, kType type, kIpVersion ipVersion, kAlloc allocator); 
kFx(kStatus) xkTcpClient_InitFromSocket(kTcpClient client, kType type, kSocket socket, kAlloc allocator); 
kFx(kStatus) xkTcpClient_VRelease(kTcpClient client);

kFx(kStatus) xkTcpClient_VReadSomeImpl(kTcpClient client, void* buffer, kSize minCount, kSize maxCount, kSize* bytesRead);
kFx(kStatus) xkTcpClient_VWriteImpl(kTcpClient client, const void* buffer, kSize size);
kFx(kStatus) xkTcpClient_VSeek(kTcpClient client, k64s offset, kSeekOrigin origin); 
kFx(kStatus) xkTcpClient_VFlush(kTcpClient client);
kFx(kStatus) xkTcpClient_VFill(kTcpClient client);

kFx(kStatus) xkTcpClient_ReadAtLeast(kTcpClient client, kByte* buffer, kSize minCount, kSize maxCount, kSize* bytesRead); 
kFx(kStatus) xkTcpClient_WriteAll(kTcpClient client, const kByte* buffer, kSize count);

#endif

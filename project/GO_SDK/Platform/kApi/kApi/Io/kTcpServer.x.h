/** 
 * @file    kTcpServer.x.h
 *
 * @internal
 * Copyright (C) 2008-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_TCP_SERVER_X_H
#define K_API_TCP_SERVER_X_H

typedef struct kTcpServerClass
{
    kObjectClass base; 
    kSocket socket;                     //Socket object. 
    kSSize clientWriteBufferSize;       //Client write buffer size for accepted sockets.
    kSSize clientReadBufferSize;        //Client read buffer size for accepted sockets.
} kTcpServerClass;

kDeclareClassEx(k, kTcpServer, kObject)

/* 
* Private methods. 
*/

kFx(kStatus) xkTcpServer_Init(kTcpServer server, kType type, kIpVersion ipVersion, kAlloc allocator); 
kFx(kStatus) xkTcpServer_VRelease(kTcpServer server);

#endif

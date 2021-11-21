/** 
 * @file    kUdpClient.x.h
 *
 * @internal
 * Copyright (C) 2008-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_UDP_CLIENT_X_H
#define K_API_UDP_CLIENT_X_H

#include <kApi/Io/kStream.h>

typedef struct kUdpClientClass
{
    kStreamClass base; 
    kSocket socket;                 //Socket object. 
    kBool isBroacastReceiver;       //Work-around for SYS/BIOS; see kUdpClient_EnableBroadcastReceive comments. 
} kUdpClientClass;

kDeclareClassEx(k, kUdpClient, kStream)

kFx(kStatus) xkUdpClient_Init(kUdpClient client, kType type, kIpVersion ipVersion, kAlloc allocator); 
kFx(kStatus) xkUdpClient_VRelease(kUdpClient client);

kFx(kStatus) xkUdpClient_VReadSomeImpl(kUdpClient client, void* buffer, kSize minCount, kSize maxCount, kSize* bytesRead);
kFx(kStatus) xkUdpClient_VWriteImpl(kUdpClient client, const void* buffer, kSize size);
kFx(kStatus) xkUdpClient_VFlush(kUdpClient client);
kFx(kStatus) xkUdpClient_VFill(kUdpClient client);

#endif

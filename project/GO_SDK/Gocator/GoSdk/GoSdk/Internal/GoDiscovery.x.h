/**
 * @file    GoDiscovery.x.h
 *
 * @internal
 * Copyright (C) 2016-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef GO_SDK_DISCOVERY_X_H
#define GO_SDK_DISCOVERY_X_H

#include <GoSdk/Internal/GoDiscovery.h>
#include <GoSdk/GoSdkReservedPorts.h>
#include <kApi/Threads/kPeriodic.h>
#include <kApi/Threads/kThread.h>
#include <kApi/Io/kUdpClient.h>
#include <kApi/Io/kSerializer.h>
#include <kApi/Data/kArrayList.h>
#include <kApi/Threads/kTimer.h>
#include <kApi/Threads/kLock.h>
#include <kApi/Io/kUdpClient.h>
#include <kApi/Data/kMap.h>
#include <kApi/Io/kUdpClient.h>
#include <kApi/Io/kSocket.h>
#include <kApi/Io/kMemory.h>

kDeclareValueEx(Go, GoDiscoveryInfo, kValue)

typedef struct GoDiscoveryInterface
{
    kIpAddress  address;
    kUdpClient  client;
    kSerializer writer;
    kBool       enabled;
} GoDiscoveryInterface;

kDeclareValueEx(Go, GoDiscoveryInterface, kValue)

/*
 * GoDiscovery class
 */

#define GO_DISCOVERY_PORT                           (GO_SDK_RESERVED_PORT_DISCOVERY_PROTOCOL)
#define GO_DISOVERY_SIGNATURE                       (0x504455494D4CLL)
#define GO_DISCOVERY_MAX_INTERFACES                 (32)

// Use a large buffer to reliably support a large number of
// devices, especially with compat mode (multiple commands).
#define GO_DISCOVERY_RECV_BUFFER_SIZE               (1024 * 1024)

#define GO_DISCOVERY_GET_ADDRESS                    (0x0001)
#define GO_DISCOVERY_GET_ADDRESS_SIZE               (32)
#define GO_DISCOVERY_GET_ADDRESS_REPLY              (0x1001)
#define GO_DISCOVERY_GET_ADDRESS_TIMEOUT            (500000)

#define GO_DISCOVERY_SET_ADDRESS                    (0x0002)
#define GO_DISCOVERY_SET_ADDRESS_SIZE               (72)
#define GO_DISCOVERY_SET_ADDRESS_REPLY              (0x1002)
#define GO_DISCOVERY_SET_ADDRESS_TIMEOUT            (6000000)

#define GO_DISCOVERY_SET_ADDRESS_TEST               (0x0E00)
#define GO_DISCOVERY_SET_ADDRESS_TEST_SIZE          (72)
#define GO_DISCOVERY_SET_ADDRESS_TEST_REPLY         (0x1E00)
#define GO_DISCOVERY_SET_ADDRESS_TEST_TIMEOUT       (6000000)

#define GO_DISCOVERY_GET_INFO                       (0x0005)
#define GO_DISCOVERY_GET_INFO_SIZE                  (32)
#define GO_DISCOVERY_GET_INFO_REPLY                 (0x1005)
#define GO_DISCOVERY_GET_INFO_REPLY_ATTR_SIZE       (65)
#define GO_DISCOVERY_GET_INFO_TIMEOUT               (5000000)


typedef struct GoDiscoveryClass
{
    kObjectClass base;

    kAtomic32s compatEnabled;

    k32u localPort;
    k64u enumPeriod;
    kCallback onEnumerate;

    kArrayList infoList;

    kArrayList interfaces;
    kUdpClient receiver;
    kMemory readerBuffer; // Need buffer to read the same data multiple times.
    kSerializer reader;

    kPeriodic eventTimer;
    kTimer stopwatch;
    kBool enumPending;
    kLock enumLock;

    kMap sensorInfoMap; // <k32u, GoDiscoveryInfo>. Used for sorting replies.
} GoDiscoveryClass;

kDeclareClassEx(Go, GoDiscovery, kObject)

// --------------- GoDiscoveryInfo Getter Functions --------------------
GoFx(k32u) GoDiscoveryInfo_Id(const GoDiscoveryInfo* info);
GoFx(GoAddressInfo) GoDiscoveryInfo_Address(const GoDiscoveryInfo* info);
GoFx(GoPortInfo) GoDiscoveryInfo_Ports(const GoDiscoveryInfo* info);
GoFx(kVersion) GoDiscoveryInfo_Version(const GoDiscoveryInfo* info);
GoFx(GoDiscoveryOpMode) GoDiscoveryInfo_OpMode(const GoDiscoveryInfo* info);
GoFx(kIpAddress) GoDiscoveryInfo_AccelSensorIpAddress(const GoDiscoveryInfo* info);
GoFx(kIpAddress) GoDiscoveryInfo_DiscoveryServerAddress(const GoDiscoveryInfo* info);
// Private GoDiscoveryInfo_Model() was discontinued in favor of GoDiscoveryInfo_PartNumber, ModelNumber, and ModelDisplayName
GoFx(kStatus) GoDiscoveryInfo_PartNumber(const GoDiscoveryInfo* info, kChar* partNumber, kSize capacity);
GoFx(kStatus) GoDiscoveryInfo_ModelNumber(const GoDiscoveryInfo* info, kChar* modelNumber, kSize capacity);
GoFx(kStatus) GoDiscoveryInfo_ModelDisplayName(const GoDiscoveryInfo* info, kChar* modelDisplayName, kSize capacity);
GoFx(kStatus) GoDiscoveryInfo_Family(const GoDiscoveryInfo* info, kChar* family, kSize capacity);
// ----------- End GoDiscoveryInfo Getter Functions -------------

GoFx(kStatus) GoDiscovery_Init(GoDiscovery discovery, kType type, kBool enableAutoDiscovery, kAlloc alloc);
GoFx(kStatus) GoDiscovery_VRelease(GoDiscovery discovery);

GoFx(kStatus) GoDiscovery_ReadIpAddress(kSerializer serializer, kIpAddress* address);

GoFx(kStatus) GoDiscovery_ReceiveReply(GoDiscovery discovery, kIpEndPoint* endPoint, k64u timeout);

GoFx(kStatus) GoDiscovery_GetReceivedResponseId(GoDiscovery discovery, k64s* responseId);
GoFx(void) GoDiscovery_ParseReceivedEnumReply(GoDiscovery discovery, kIpEndPoint* remoteEndPoint);

GoFx(kStatus) GoDiscovery_ParseGetAddressReply(GoDiscovery discovery, GoDiscoveryInfo* info, kIpEndPoint* remoteEndPoint);
GoFx(kStatus) GoDiscovery_ParseSetAddressReply(GoDiscovery discovery, k32u* deviceId);
GoFx(kStatus) GoDiscovery_ParseSetTestReply(GoDiscovery discovery, k32u* deviceId);
GoFx(kStatus) GoDiscovery_ParseGetInfoReply(GoDiscovery discovery, GoDiscoveryExtInfo* info, kAlloc allocator);

GoFx(kStatus) GoDiscovery_BeginEnum(GoDiscovery discovery);
GoFx(kStatus) GoDiscovery_EndEnum(GoDiscovery discovery, kArrayList list);

GoFx(kStatus) GoDiscovery_SendGetAddress(GoDiscovery discovery, GoDiscoveryInterface* iface);
GoFx(kStatus) GoDiscovery_SendGetExtendedInfo(GoDiscovery discovery, GoDiscoveryInterface* iface);
GoFx(kStatus) GoDiscovery_ProcessGetAddressReply(GoDiscovery discovery, const GoDiscoveryInfo* info);
GoFx(kStatus) GoDiscovery_ProcessGetExtendedInfoReply(GoDiscovery discovery, GoDiscoveryExtInfo extInfo, kIpEndPoint* remoteEndPoint);

GoFx(kStatus) GoDiscovery_OnEnumElapsed(GoDiscovery discovery, kPeriodic timer);
GoFx(kStatus) GoDiscovery_OnEnumElapsedRecv(GoDiscovery discovery, kPeriodic timer);

GoFx(kStatus) GoDiscovery_SetAddressTest(GoDiscovery discovery, k32u deviceId, const GoAddressInfo* address);
GoFx(kStatus) GoDiscovery_ParseSetAddressReplyCommon(GoDiscovery discovery, k64s replyMsgId, k32u* deviceId);

#endif

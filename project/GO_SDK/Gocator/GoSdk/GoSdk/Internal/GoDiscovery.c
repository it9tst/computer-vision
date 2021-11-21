/**
 * @file    GoDiscovery.c
 *
 * @internal
 * Copyright (C) 2016-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <GoSdk/Internal/GoDiscovery.h>
#include <kApi/Utils/kUtils.h>

kBeginValueEx(Go, GoDiscoveryInfo)
kEndValueEx()

kBeginValueEx(Go, GoDiscoveryInterface)
    kAddField(GoDiscoveryInterface, kIpAddress, address)
    kAddField(GoDiscoveryInterface, kUdpClient, client)
    kAddField(GoDiscoveryInterface, kSerializer, writer)
    kAddField(GoDiscoveryInterface, kBool, enabled)
kEndValueEx()

/*
 * GoDiscovery class
 */

kBeginClassEx(Go, GoDiscovery)
    kAddVMethod(GoDiscovery, kObject, VRelease)
kEndClassEx()

// For debugging discovery protocol.
#ifdef DEBUG_GO_DISCOVERY
#include <stdio.h>

#define DEBUG_PRINT printf
#define DEBUG_PRINT_ENDPOINT PrintEndPoint

void PrintAddressPort(kIpAddress ipAddress, k32u port)
{
    kText16 ipAddrStr;

    kIpAddress_Format(ipAddress, ipAddrStr, kCountOf(ipAddrStr));
    DEBUG_PRINT("   @@@ socket %s:%u\n", ipAddrStr, port);
}

void PrintEndPoint(kIpEndPoint* endPoint)
{
    PrintAddressPort(endPoint->address, endPoint->port);
}

void PrintUdpClient(kUdpClient udpClient)
{
    kSocket listenerSocket;
    kIpEndPoint localEndPoint;

    listenerSocket = kUdpClient_Socket(udpClient);
    if (!kSuccess(kSocket_LocalEndPoint(listenerSocket, &localEndPoint)))
    {
        DEBUG_PRINT("   **** Error getting interface local endpoint for client %p\n", udpClient);
    }
    else
    {
        PrintEndPoint(&localEndPoint);
    }

}
#else
#define DEBUG_PRINT
#define DEBUG_PRINT_ENDPOINT
#endif

// --------------- GoDiscoveryInfo Utility Functions --------------------
GoFx(kStatus) GoDiscoveryInfo_Init(GoDiscoveryInfo* info)
{
    return (kMemSet(info, 0, sizeof(GoDiscoveryInfo)));
}

GoFx(k32u) GoDiscoveryInfo_Id(const GoDiscoveryInfo* info)
{
    return info->id;
}

GoFx(GoAddressInfo) GoDiscoveryInfo_Address(const GoDiscoveryInfo* info)
{
    return info->address;
}

GoFx(GoPortInfo) GoDiscoveryInfo_Ports(const GoDiscoveryInfo* info)
{
    return info->ports;
}

GoFx(kVersion) GoDiscoveryInfo_Version(const GoDiscoveryInfo* info)
{
    return info->version;
}

GoFx(GoDiscoveryOpMode) GoDiscoveryInfo_OpMode(const GoDiscoveryInfo* info)
{
    return info->opMode;
}

GoFx(kIpAddress) GoDiscoveryInfo_AccelSensorIpAddress(const GoDiscoveryInfo* info)
{
    return info->accelSensorIpAddress;
}

GoFx(kIpAddress) GoDiscoveryInfo_DiscoveryServerAddress(const GoDiscoveryInfo* info)
{
    return info->discoveryServerAddress;
}

GoFx(kStatus) GoDiscoveryInfo_PartNumber(const GoDiscoveryInfo* info, kChar* partNumber, kSize capacity)
{
    kCheck(kStrCopy(partNumber, capacity, info->partNumber));

    return kOK;
}

GoFx(kStatus) GoDiscoveryInfo_ModelNumber(const GoDiscoveryInfo* info, kChar* modelNumber, kSize capacity)
{
    kCheck(kStrCopy(modelNumber, capacity, info->modelNumber));

    return kOK;
}

GoFx(kStatus) GoDiscoveryInfo_ModelDisplayName(const GoDiscoveryInfo* info, kChar* modelDisplayName, kSize capacity)
{
    kCheck(kStrCopy(modelDisplayName, capacity, info->modelDisplayName));

    return kOK;
}

GoFx(kStatus) GoDiscoveryInfo_Family(const GoDiscoveryInfo* info, kChar* family, kSize capacity)
{
    kCheck(kStrCopy(family, capacity, info->family));

    return kOK;
}

// ----------- End GoDiscoveryInfo Getter Functions -------------

GoFx(kStatus) GoDiscovery_Construct(GoDiscovery* discovery, kBool enableAutoDiscovery, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback(allocator);
    kStatus status;

    kCheck(kAlloc_GetObject(alloc, kTypeOf(GoDiscovery), discovery));

    if (!kSuccess(status = GoDiscovery_Init(*discovery, kTypeOf(GoDiscovery), enableAutoDiscovery, alloc)))
    {
        kAlloc_FreeRef(alloc, discovery);
    }

    return status;
}

GoFx(kStatus) GoDiscovery_Init(GoDiscovery discovery, kType type, kBool enableAutoDiscovery, kAlloc alloc)
{
    kObjR(GoDiscovery, discovery);
    kArrayList addresses = kNULL;
    kStatus status;
    kIpEndPoint localEndPoint;
    kIpEntry localIpEntry;
    kSize i;

    kCheck(kObject_Init(discovery, type, alloc));
    kAtomic32s_Init(&obj->compatEnabled, 0);
    obj->localPort = 0;
    obj->enumPeriod = 0;
    obj->onEnumerate.function = kNULL;
    obj->onEnumerate.receiver = kNULL;
    kZero(obj->infoList);
    kZero(obj->interfaces);
    kZero(obj->receiver);
    kZero(obj->readerBuffer);
    kZero(obj->reader);
    kZero(obj->eventTimer);
    kZero(obj->stopwatch);
    obj->enumPending = kFALSE;
    kZero(obj->enumLock);
    kZero(obj->sensorInfoMap);

    kTry
    {
        kAtomic32s_Exchange(&obj->compatEnabled, 0);

        kTest(kMap_Construct(&obj->sensorInfoMap, kTypeOf(k32u), kTypeOf(GoDiscoveryInfo), 0, alloc));
        kTest(kArrayList_Construct(&obj->infoList, kTypeOf(GoDiscoveryInfo), 0, alloc));
        kTest(kArrayList_Construct(&obj->interfaces, kTypeOf(GoDiscoveryInterface), 0, alloc));

        kTest(kUdpClient_Construct(&obj->receiver, kIP_VERSION_4, alloc));
        kTest(kUdpClient_EnableReuseAddress(obj->receiver, kTRUE));

        // Keep the QNX buffer size the same, in case the exact choice is important.
        // If the increase in discovery traffic from compat mode is a problem, user
        // can disable it with GoSystem_EnableDiscoveryCompatibility.
#if defined(K_QNX)
        kTest(kUdpClient_SetReadBuffers(obj->receiver, 65 * 1024, 2048));
#else
        kTest(kUdpClient_SetReadBuffers(obj->receiver, GO_DISCOVERY_RECV_BUFFER_SIZE, 2048));
#endif

        kTest(kUdpClient_Bind(obj->receiver, kIpAddress_AnyV4(), kIP_PORT_ANY));

        kTest(kUdpClient_LocalEndPoint(obj->receiver, &localEndPoint));
        obj->localPort = localEndPoint.port;

        kTest(kMemory_Construct(&obj->readerBuffer, alloc));
        kTest(kSerializer_Construct(&obj->reader, obj->readerBuffer, kNULL, alloc));

        kTest(kPeriodic_Construct(&obj->eventTimer, alloc));
        kTest(kTimer_Construct(&obj->stopwatch, alloc));

        kTest(kLock_Construct(&obj->enumLock, alloc));

        kTest(kArrayList_Construct(&addresses, kTypeOf(kIpEntry), 0, alloc));
        kTest(kNetwork_LocalIpInterfaces(addresses));

#if !defined(K_QNX) && !defined(K_POSIX)
        // Add localhost interfaces except for QNX & Linux systems.
        kTest(kStrCopy(localIpEntry.name, kCountOf(localIpEntry.name), "localhost"));
        localIpEntry.address = kIpAddress_LoopbackV4();
        kTest(kArrayList_AddT(addresses, &localIpEntry));
#endif

        for (i = 0; i < kArrayList_Count(addresses); ++i)
        {
            GoDiscoveryInterface iface;
            kIpEntry entry;

            entry = kArrayList_AsT(addresses, i, kIpEntry);
            iface.address = entry.address;
            iface.client = kNULL;
            iface.writer = kNULL;
            iface.enabled = enableAutoDiscovery;

            kTest(kUdpClient_Construct(&iface.client, kIP_VERSION_4, alloc));
            kTest(kUdpClient_EnableBroadcast(iface.client, kTRUE));
            kTest(kUdpClient_EnableReuseAddress(iface.client, kTRUE));

            if (kSuccess(kUdpClient_Bind(iface.client, iface.address, obj->localPort)))
            {
                kTest(kUdpClient_SetWriteBuffers(iface.client, -1, 2048));
                kTest(kSerializer_Construct(&iface.writer, iface.client, kNULL, alloc));
                kTest(kArrayList_AddT(obj->interfaces, &iface));

#if defined(K_QNX)
                kTest(kSocket_BindToDevice(kUdpClient_Socket(iface.client), entry.name));
#endif
            }
            else
            {
                kDestroyRef(&iface.client);
            }
        }
    }
    kCatchEx(&status)
    {
        GoDiscovery_VRelease(discovery);

        kEndCatchEx(status);
    }
    kFinallyEx
    {
        kDisposeRef(&addresses);

        kEndFinallyEx();
    }

    return kOK;
}

GoFx(kStatus) GoDiscovery_VRelease(GoDiscovery discovery)
{
    kObj(GoDiscovery, discovery);
    kSize i;

    if (!kIsNull(obj->eventTimer))
    {
        kCheck(GoDiscovery_StopEnum(discovery));
    }

    kCheck(kDisposeRef(&obj->infoList));
    kCheck(kDisposeRef(&obj->sensorInfoMap));

    if (!kIsNull(obj->interfaces))
    {
        for (i = 0; i < kArrayList_Count(obj->interfaces); ++i)
        {
            GoDiscoveryInterface* iface = kArrayList_AtT(obj->interfaces, i, GoDiscoveryInterface);

            kCheck(kDestroyRef(&iface->writer));
            kCheck(kDestroyRef(&iface->client));
        }

        kCheck(kDisposeRef(&obj->interfaces));
    }

    kCheck(kDestroyRef(&obj->enumLock));

    kCheck(kDestroyRef(&obj->reader));
    kCheck(kDestroyRef(&obj->readerBuffer));
    kCheck(kDestroyRef(&obj->receiver));

    kCheck(kDestroyRef(&obj->eventTimer));
    kCheck(kDestroyRef(&obj->stopwatch));

    return kObject_VRelease(discovery);
}

GoFx(kStatus) GoDiscovery_WriteIpAddress(kSerializer serializer, kIpAddress address)
{
    kByte bytes[8];

    bytes[0] = 0;
    bytes[1] = 0;
    bytes[2] = 0;
    bytes[3] = 0;
    bytes[4] = (address.address[0] & 0xFF);
    bytes[5] = (address.address[1] & 0xFF);
    bytes[6] = (address.address[2] & 0xFF);
    bytes[7] = (address.address[3] & 0xFF);

    kCheck(kSerializer_WriteByteArray(serializer, bytes, kCountOf(bytes)));

    return kOK;
}

GoFx(kStatus) GoDiscovery_ReadIpAddress(kSerializer serializer, kIpAddress* address)
{
    kByte bytes[8];
    kSize i;

    kCheck(kSerializer_ReadByteArray(serializer, bytes, kCountOf(bytes)));

    address->address[0] = bytes[4];
    address->address[1] = bytes[5];
    address->address[2] = bytes[6];
    address->address[3] = bytes[7];

    for (i = 4; i < 16; i++)
    {
        address->address[i] = 0;
    }

    address->version = kIP_VERSION_4;

    return kOK;
}

GoFx(kStatus) GoDiscovery_Enumerate(GoDiscovery discovery, kArrayList infoList)
{
    kObj(GoDiscovery, discovery);

    kCheck(kLock_Enter(obj->enumLock));

    kTry
    {
        kTest(GoDiscovery_BeginEnum(discovery));
        kTest(kThread_Sleep(GO_DISCOVERY_GET_ADDRESS_TIMEOUT));
        kTest(GoDiscovery_EndEnum(discovery, infoList));
    }
    kFinally
    {
        kLock_Exit(obj->enumLock);

        kEndFinally();
    }

    return kOK;
}

GoFx(kStatus) GoDiscovery_GetAddress(GoDiscovery discovery, k32u deviceId, GoAddressInfo* address)
{
    kObj(GoDiscovery, discovery);
    GoDiscoveryInfo info;
    kIpEndPoint remoteEndPoint;
    kSize i;
    kStatus sendStatus = kERROR_NETWORK;
    kStatus recvStatus = kERROR_NOT_FOUND;

    kCheck(kLock_Enter(obj->enumLock));
    kTry
    {
        for (i = 0; i < kArrayList_Count(obj->interfaces); ++i)
        {
            GoDiscoveryInterface* iface = kArrayList_AtT(obj->interfaces, i, GoDiscoveryInterface);

            if (iface->enabled)
            {
                kTest(kSerializer_Write64s(iface->writer, GO_DISCOVERY_GET_ADDRESS_SIZE));
                kTest(kSerializer_Write64s(iface->writer, GO_DISCOVERY_GET_ADDRESS));
                kTest(kSerializer_Write64s(iface->writer, GO_DISOVERY_SIGNATURE));
                kTest(kSerializer_Write64s(iface->writer, deviceId));

                kTest(kSerializer_Flush(iface->writer));

                if (kSuccess(kUdpClient_Send(iface->client, kIpAddress_BroadcastV4(), GO_DISCOVERY_PORT, GO_DISCOVERY_GET_ADDRESS_TIMEOUT, kTRUE)))
                {
                    sendStatus = kOK;
                }
            }
        }

        kTest(sendStatus);
        kTest(kTimer_Start(obj->stopwatch, GO_DISCOVERY_GET_ADDRESS_TIMEOUT));

        do
        {
            if (kSuccess(GoDiscovery_ReceiveReply(discovery, &remoteEndPoint, kTimer_Remaining(obj->stopwatch)))
                && remoteEndPoint.port == GO_DISCOVERY_PORT
                && kSuccess(GoDiscovery_ParseGetAddressReply(discovery, &info, &remoteEndPoint))
                && info.id == deviceId)
            {
                *address = info.address;
                recvStatus = kOK;

                DEBUG_PRINT("%s: sensor %u remote end point\n", __FUNCTION__, deviceId);
                DEBUG_PRINT_ENDPOINT(&remoteEndPoint);

                break;
            }
        } while (!kTimer_IsExpired(obj->stopwatch));
    }
    kFinally
    {
        kLock_Exit(obj->enumLock);

        kEndFinally();
    }

    return recvStatus;
}

GoFx(kStatus) GoDiscovery_SetAddress(GoDiscovery discovery, k32u deviceId, const GoAddressInfo* address)
{
    kObj(GoDiscovery, discovery);
    k32u responseDeviceId;
    kIpEndPoint remoteEndPoint;
    kIpAddress zero;
    kSize i;
    kStatus sendStatus = kERROR_NETWORK;

    for (i = 0; i < 16; i++)
    {
        zero.address[i] = 0;
    }

    for (i = 0; i < kArrayList_Count(obj->interfaces); ++i)
    {
        GoDiscoveryInterface* iface = kArrayList_AtT(obj->interfaces, i, GoDiscoveryInterface);

        if (iface->enabled)
        {
            kCheck(kSerializer_Write64s(iface->writer, GO_DISCOVERY_SET_ADDRESS_SIZE));
            kCheck(kSerializer_Write64s(iface->writer, GO_DISCOVERY_SET_ADDRESS));
            kCheck(kSerializer_Write64s(iface->writer, GO_DISOVERY_SIGNATURE));
            kCheck(kSerializer_Write64s(iface->writer, deviceId));
            kCheck(kSerializer_Write64s(iface->writer, address->useDhcp));

            kCheck(GoDiscovery_WriteIpAddress(iface->writer, (address->useDhcp) ? kIpAddress_AnyV4() : address->address));
            kCheck(GoDiscovery_WriteIpAddress(iface->writer, (address->useDhcp) ? kIpAddress_AnyV4() : address->mask));
            kCheck(GoDiscovery_WriteIpAddress(iface->writer, (address->useDhcp) ? kIpAddress_AnyV4() : address->gateway));
            kCheck(GoDiscovery_WriteIpAddress(iface->writer, (address->useDhcp) ? kIpAddress_AnyV4() : zero));

            kCheck(kSerializer_Flush(iface->writer));
            if (kSuccess(kUdpClient_Send(iface->client, kIpAddress_BroadcastV4(), GO_DISCOVERY_PORT, GO_DISCOVERY_SET_ADDRESS_TIMEOUT, kTRUE)))
            {
                sendStatus = kOK;
            }
        }
    }

    kCheck(sendStatus);
    kCheck(kTimer_Start(obj->stopwatch, GO_DISCOVERY_SET_ADDRESS_TIMEOUT));

    do
    {
        if (kSuccess(GoDiscovery_ReceiveReply(discovery, &remoteEndPoint, kTimer_Remaining(obj->stopwatch)))
            && remoteEndPoint.port == GO_DISCOVERY_PORT
            && kSuccess(GoDiscovery_ParseSetAddressReply(discovery, &responseDeviceId))
            && responseDeviceId == deviceId)
        {
            return kOK;
        }
    } while (!kTimer_IsExpired(obj->stopwatch));

    return kERROR_NOT_FOUND;
}

GoFx(kStatus) GoDiscovery_SetAddressTest(GoDiscovery discovery, k32u deviceId, const GoAddressInfo* address)
{
    kObj(GoDiscovery, discovery);
    k32u responseDeviceId;
    kIpEndPoint remoteEndPoint;
    kIpAddress zero;
    kSize i;

    for (i = 0; i < 16; i++)
    {
        zero.address[i] = 0;
    }

    for (i = 0; i < kArrayList_Count(obj->interfaces); ++i)
    {
        GoDiscoveryInterface* iface = kArrayList_AtT(obj->interfaces, i, GoDiscoveryInterface);

        if (iface->enabled)
        {
            kCheck(kSocket_Connect(kUdpClient_Socket(iface->client), kIpAddress_BroadcastV4(), GO_DISCOVERY_PORT, 1000000));

            kCheck(kSerializer_Write64s(iface->writer, GO_DISCOVERY_SET_ADDRESS_SIZE));
            kCheck(kSerializer_Write64s(iface->writer, GO_DISCOVERY_SET_ADDRESS_TEST));
            kCheck(kSerializer_Write64s(iface->writer, GO_DISOVERY_SIGNATURE));
            kCheck(kSerializer_Write64s(iface->writer, deviceId));
            kCheck(kSerializer_Write64s(iface->writer, address->useDhcp));

            kCheck(GoDiscovery_WriteIpAddress(iface->writer, (address->useDhcp) ? kIpAddress_AnyV4() : address->address));
            kCheck(GoDiscovery_WriteIpAddress(iface->writer, (address->useDhcp) ? kIpAddress_AnyV4() : address->mask));
            kCheck(GoDiscovery_WriteIpAddress(iface->writer, (address->useDhcp) ? kIpAddress_AnyV4() : address->gateway));
            kCheck(GoDiscovery_WriteIpAddress(iface->writer, (address->useDhcp) ? kIpAddress_AnyV4() : zero));

            kCheck(kSerializer_Flush(iface->writer));
            kCheck(kUdpClient_Send(iface->client, kIpAddress_BroadcastV4(), GO_DISCOVERY_PORT, GO_DISCOVERY_SET_ADDRESS_TIMEOUT, kTRUE));
        }
    }

    kCheck(kTimer_Start(obj->stopwatch, GO_DISCOVERY_SET_ADDRESS_TIMEOUT));

    do
    {
        if (kSuccess(GoDiscovery_ReceiveReply(discovery, &remoteEndPoint, kTimer_Remaining(obj->stopwatch)))
            && remoteEndPoint.port == GO_DISCOVERY_PORT
            && kSuccess(GoDiscovery_ParseSetTestReply(discovery, &responseDeviceId))
            && responseDeviceId == deviceId)
        {
            return kOK;
        }
    } while (!kTimer_IsExpired(obj->stopwatch));

    return kERROR_NOT_FOUND;
}

GoFx(kStatus) GoDiscovery_GetExtendedInfo(GoDiscovery discovery, k32u deviceId, GoDiscoveryExtInfo* info, kAlloc allocator)
{
    kObj(GoDiscovery, discovery);
    kIpEndPoint remoteEndPoint;
    kSize i;
    kStatus sendStatus = kERROR_NETWORK;
    kStatus recvStatus = kERROR_NOT_FOUND;

    allocator = kAlloc_Fallback(allocator);

    kCheck(kLock_Enter(obj->enumLock));

    kTry
    {

        for (i = 0; i < kArrayList_Count(obj->interfaces); ++i)
        {
            GoDiscoveryInterface* iface = kArrayList_AtT(obj->interfaces, i, GoDiscoveryInterface);

            if (iface->enabled)
            {
                kTest(kSerializer_Write32u(iface->writer, GO_DISCOVERY_GET_INFO_SIZE));
                kTest(kSerializer_Write32u(iface->writer, 0));
                kTest(kSerializer_Write32u(iface->writer, GO_DISCOVERY_GET_INFO));
                kTest(kSerializer_Write32u(iface->writer, 0));
                kTest(kSerializer_Write64u(iface->writer, GO_DISOVERY_SIGNATURE));
                kTest(kSerializer_Write32u(iface->writer, deviceId));
                kTest(kSerializer_Write32u(iface->writer, 0));

                kTest(kSerializer_Flush(iface->writer));

                if (kSuccess(kUdpClient_Send(iface->client, kIpAddress_BroadcastV4(), GO_DISCOVERY_PORT, GO_DISCOVERY_GET_INFO_TIMEOUT, kTRUE)))
                {
                    sendStatus = kOK;
                }
            }
        }

        kTest(sendStatus);
        kTest(kTimer_Start(obj->stopwatch, GO_DISCOVERY_GET_INFO_TIMEOUT));

        do
        {
            if (kSuccess(GoDiscovery_ReceiveReply(discovery, &remoteEndPoint, kTimer_Remaining(obj->stopwatch)))
                && remoteEndPoint.port == GO_DISCOVERY_PORT
                && kSuccess(GoDiscovery_ParseGetInfoReply(discovery, info, allocator))
                && GoDiscoveryExtInfo_Id(*info) == deviceId)
            {
                DEBUG_PRINT("%s: sensor %u remote end point\n", __FUNCTION__, deviceId);
                DEBUG_PRINT_ENDPOINT(&remoteEndPoint);

                recvStatus = kOK;
                break;
            }
            else
            {
                if (!kIsNull(*info))
                {
                    kDisposeRef(info);
                }
            }
        } while (!kTimer_IsExpired(obj->stopwatch));

    }
    kFinally
    {
        kLock_Exit(obj->enumLock);

        kEndFinally();
    }

    return recvStatus;
}

GoFx(kStatus) GoDiscovery_ReceiveReply(GoDiscovery discovery, kIpEndPoint* endPoint, k64u timeout)
{
    kObj(GoDiscovery, discovery);
    kSize readSize;

    // Copy into a kStream whose pointer can be reset and the content read again.
    kCheck(kUdpClient_Receive(obj->receiver, endPoint, &readSize, timeout));

    kCheck(kStream_Seek(obj->readerBuffer, 0, kSEEK_ORIGIN_BEGIN));
    kCheck(kStream_Copy(obj->readerBuffer, obj->receiver, readSize));

    kCheck(kStream_Seek(obj->readerBuffer, 0, kSEEK_ORIGIN_BEGIN));

    return kOK;
}

GoFx(kStatus) GoDiscovery_ParseGetAddressReply(GoDiscovery discovery, GoDiscoveryInfo* info, kIpEndPoint* remoteEndPoint)
{
    kObj(GoDiscovery, discovery);
    k64s responseId, signature, status, deviceId, useDhcp;
    kIpAddress tempIp;

    kCheck(kMemSet(info, 0, sizeof(info)));

    // Read and store the message attributes data length ("attrSize")
    // to determine how many attributes to read.
    // The size field is included in the message size count stored in the size field.
    //
    // If this fails, the EndRead() could assert so skip the EndRead() if BeginRead() fails.
    kCheck(kSerializer_BeginRead(obj->reader, kTypeOf(k64u), kTRUE));

    kTry
    {
        kTest(kSerializer_Read64s(obj->reader, &responseId));
        // The 5.1 and newer SDK sends both a GET ADDRESS and GET INFO
        // discovery request. A 5.1 and newer sensor firmware version
        // supports both requests and therefore sends back both replies.
        // In the caller to this function, a reply is assumed to be a GET ADDRESS
        // reply and if it is not, then the reply is parsed as a GET
        // INFO reply.
        // So it is valid that the response id is not an GET ADDRESS REPLY so
        // it is not a real error but an expected scenario.
        kTest(responseId == GO_DISCOVERY_GET_ADDRESS_REPLY);

        kTest(kSerializer_Read64s(obj->reader, &status));
        kTest(status);

        kTest(kSerializer_Read64s(obj->reader, &signature));
        kTest(signature == GO_DISOVERY_SIGNATURE);

        kTest(kSerializer_Read64s(obj->reader, &deviceId));
        info->id = (k32u) deviceId;

        kTest(kSerializer_Read64s(obj->reader, &useDhcp));
        info->address.useDhcp = (kBool) useDhcp;

        kTest(GoDiscovery_ReadIpAddress(obj->reader, &info->address.address));
        kTest(GoDiscovery_ReadIpAddress(obj->reader, &info->address.mask));
        kTest(GoDiscovery_ReadIpAddress(obj->reader, &info->address.gateway));
        kTest(GoDiscovery_ReadIpAddress(obj->reader, &tempIp));

        // The ports must be set to a non-zero value to indicate they do not
        // have a valid value. All the remaining fields in "info" can remain as zeroes.
        info->ports.controlPort = k16U_NULL;
        info->ports.dataPort = k16U_NULL;
        info->ports.healthPort = k16U_NULL;
        info->ports.upgradePort = k16U_NULL;
        info->ports.webPort = k16U_NULL;

        info->discoveryServerAddress = remoteEndPoint->address;
    }
    kFinally
    {
        // Automatically flush any unread data and free serializer read section.
        // Ignore errors.
        kSerializer_EndRead(obj->reader);

        kEndFinally();
    }

    return kOK;
}

GoFx(kStatus) GoDiscovery_ParseSetAddressReplyCommon(GoDiscovery discovery, k64s replyMsgId, k32u* deviceId)
{
    kObj(GoDiscovery, discovery);
    k64s responseDeviceId;
    k64s responseSignature;
    k64s responseId;
    k64s responseStatus;

    // Read and store the message attributes data length ("attrSize")
    // to determine how many attributes to read.
    // The size field is included in the message size count stored in the size field.
    //
    // If this fails, the EndRead() could assert so skip the EndRead() if BeginRead() fails.
    kCheck(kSerializer_BeginRead(obj->reader, kTypeOf(k64u), kTRUE));

    kTry
    {
        kTest(kSerializer_Read64s(obj->reader, &responseId));
        kTest(responseId == replyMsgId);

        kTest(kSerializer_Read64s(obj->reader, &responseStatus));
        kTest(responseStatus);

        kTest(kSerializer_Read64s(obj->reader, &responseSignature));
        kTest(responseSignature == GO_DISOVERY_SIGNATURE);

        kTest(kSerializer_Read64s(obj->reader, &responseDeviceId));

        *deviceId = (k32u) responseDeviceId;
    }
    kFinally
    {
        // Automatically flush any unread data and free serializer read section.
        // Ignore errors.
        kSerializer_EndRead(obj->reader);

        kEndFinally();
    }

    return kOK;
}

GoFx(kStatus) GoDiscovery_ParseSetAddressReply(GoDiscovery discovery, k32u* deviceId)
{
    return(GoDiscovery_ParseSetAddressReplyCommon(discovery, GO_DISCOVERY_SET_ADDRESS_REPLY, deviceId));
}

GoFx(kStatus) GoDiscovery_ParseSetTestReply(GoDiscovery discovery, k32u* deviceId)
{
    return(GoDiscovery_ParseSetAddressReplyCommon(discovery, GO_DISCOVERY_SET_ADDRESS_TEST_REPLY, deviceId));
}

GoFx(kStatus) GoDiscovery_ParseGetInfoReply(GoDiscovery discovery, GoDiscoveryExtInfo* info, kAlloc allocator)
{
    kObj(GoDiscovery, discovery);

    kCheck(GoDiscoveryExtInfo_Construct(info, allocator));
    kCheck(GoDiscoveryExtInfo_Read(*info, obj->reader, allocator));

    return kOK;
}

GoFx(kStatus) GoDiscovery_SetEnumPeriod(GoDiscovery discovery, k64u period)
{
    kObj(GoDiscovery, discovery);

    obj->enumPeriod = period;

    return kOK;
}

GoFx(kStatus) GoDiscovery_SetEnumHandler(GoDiscovery discovery, GoDiscoveryEnumFx function, kPointer receiver)
{
    kObj(GoDiscovery, discovery);

    obj->onEnumerate.function = (kCallbackFx) function;
    obj->onEnumerate.receiver = receiver;

    return kOK;
}

GoFx(kStatus) GoDiscovery_StartEnum(GoDiscovery discovery, kBool waitFirst)
{
    kObj(GoDiscovery, discovery);

    kCheckState(!kIsNull(obj->onEnumerate.function));
    kCheckState(!kPeriodic_Enabled(obj->eventTimer));

    if (waitFirst)
    {
        kCheck(GoDiscovery_Enumerate(discovery, obj->infoList));
        kCheck(obj->onEnumerate.function(obj->onEnumerate.receiver, discovery, obj->infoList));
    }

    kCheck(kPeriodic_Start(obj->eventTimer, 0, GoDiscovery_OnEnumElapsed, discovery));

    return kOK;
}

GoFx(kStatus) GoDiscovery_StopEnum(GoDiscovery discovery)
{
    kObj(GoDiscovery, discovery);

    kCheck(kPeriodic_Stop(obj->eventTimer));

    return kOK;
}

GoFx(kStatus) GoDiscovery_BeginEnum(GoDiscovery discovery)
{
    kObj(GoDiscovery, discovery);
    kSize i;

    for (i = 0; i < kArrayList_Count(obj->interfaces); ++i)
    {
        GoDiscoveryInterface* iface = kArrayList_AtT(obj->interfaces, i, GoDiscoveryInterface);

        if (iface->enabled)
        {
            if (kAtomic32s_Get(&obj->compatEnabled) == 1)
            {
                kCheck(GoDiscovery_SendGetAddress(discovery, iface));
            }

            kCheck(GoDiscovery_SendGetExtendedInfo(discovery, iface));
        }
    }

    return kOK;
}

// This function must pull every receive message out of the socket and
// process them all, even if one or more of them fail to parse correctly.
// Only when there are no more messages in the socket then this function
// should return.
// Do not return prematurely because of parsing related errors (GOC-13024)
// except for socket errors that prevent reading replies from the socket.
GoFx(kStatus) GoDiscovery_EndEnum(GoDiscovery discovery, kArrayList list)
{
    kObj(GoDiscovery, discovery);
    kMapItem mapItem;

    // Clear map in anticipation of processing enumeration replies.
    kCheck(kMap_Clear(obj->sensorInfoMap));

    kTry
    {
        while (kSuccess(kSocket_Wait(kUdpClient_Socket(obj->receiver), 0)))
        {
            kIpEndPoint remoteEndPoint;

            // If can't receive a pending reply, then it is a serious enough
            // error to not bother trying to empty the socket. So bail out on error.
            kTest(GoDiscovery_ReceiveReply(discovery, &remoteEndPoint, 1000000));

            if (remoteEndPoint.port == GO_DISCOVERY_PORT)
            {
                // Don't stop processing replies on an error. Instead, keep
                // processing replies until the socket is empty.
                GoDiscovery_ParseReceivedEnumReply(discovery, &remoteEndPoint);
            }
        }
    }
    kFinally
    {
        kEndFinally();
    }

    // Do this only if the enumeration reply parsed successfully.
    kCheck(kArrayList_Clear(list));

    for (mapItem = kMap_First(obj->sensorInfoMap);
        mapItem != kNULL;
        mapItem = kMap_Next(obj->sensorInfoMap, mapItem))
    {
        kCheck(kArrayList_Add(list, kMap_Value(obj->sensorInfoMap, mapItem)));
    }

    return kOK;
}

GoFx(kStatus) GoDiscovery_ProcessGetAddressReply(GoDiscovery discovery, const GoDiscoveryInfo* info)
{
    kObj(GoDiscovery, discovery);

    if (!kSuccess(kMap_Find(obj->sensorInfoMap, &info->id, kNULL)))
    {
        kCheck(kMap_Add(obj->sensorInfoMap, &info->id, info));
    }

    return kOK;
}

GoFx(kStatus) GoDiscovery_ProcessGetExtendedInfoReply(GoDiscovery discovery, GoDiscoveryExtInfo extInfo, kIpEndPoint* remoteEndPoint)
{
    kObj(GoDiscovery, discovery);
    GoDiscoveryInfo info;

    kCheck(kMemSet(&info, 0, sizeof(info)));

    // Copy most of the extended discovery information so that the sensor
    // object can be populated with the information later in the enumeration
    // process.
    info.id         = GoDiscoveryExtInfo_Id(extInfo);
    info.address    = GoDiscoveryExtInfo_Address(extInfo);
    info.ports      = GoDiscoveryExtInfo_Ports(extInfo);
    info.version    = GoDiscoveryExtInfo_Version(extInfo);
    info.opMode     = GoDiscoveryExtInfo_OpMode(extInfo);
    info.accelSensorIpAddress = GoDiscoveryExtInfo_AccelSensorIpAddress(extInfo);
    info.discoveryServerAddress = remoteEndPoint->address;

    // Call to GoDiscoveryExtInfo_CompleteModel() is deprecated; we more clearly provide
    // PartNumber, ModelNumber, and ModelDisplayName instead.
    // NOTE: the below functions may not fill in the fields if they are misssing from
    // the discovery protocol.
    kCheck(GoDiscoveryExtInfo_PartNumber(extInfo, info.partNumber, kCountOf(info.partNumber)));
    kCheck(GoDiscoveryExtInfo_ModelNumber(extInfo, info.modelNumber, kCountOf(info.modelNumber)));
    kCheck(GoDiscoveryExtInfo_ModelDisplayName(extInfo, info.modelDisplayName, kCountOf(info.modelDisplayName)));
    kCheck(GoDiscoveryExtInfo_Family(extInfo, info.family, kCountOf(info.family)));

    kCheck(kMap_Replace(obj->sensorInfoMap, &info.id, &info));

    return kOK;
}

GoFx(kStatus) GoDiscovery_SendGetAddress(GoDiscovery discovery, GoDiscoveryInterface* iface)
{
    kCheck(kSerializer_Write64s(iface->writer, GO_DISCOVERY_GET_ADDRESS_SIZE));
    kCheck(kSerializer_Write64s(iface->writer, GO_DISCOVERY_GET_ADDRESS));
    kCheck(kSerializer_Write64s(iface->writer, GO_DISOVERY_SIGNATURE));
    kCheck(kSerializer_Write64s(iface->writer, 0));

    kCheck(kSerializer_Flush(iface->writer));

    // Don't care about errors; some interfaces may not allow sending.
    (void)kUdpClient_Send(iface->client, kIpAddress_BroadcastV4(), GO_DISCOVERY_PORT, GO_DISCOVERY_GET_ADDRESS_TIMEOUT, kTRUE);

    return kOK;
}

GoFx(kStatus) GoDiscovery_SendGetExtendedInfo(GoDiscovery discovery, GoDiscoveryInterface* iface)
{
    kCheck(kSerializer_Write64s(iface->writer, GO_DISCOVERY_GET_INFO_SIZE));
    kCheck(kSerializer_Write64s(iface->writer, GO_DISCOVERY_GET_INFO));
    kCheck(kSerializer_Write64s(iface->writer, GO_DISOVERY_SIGNATURE));
    kCheck(kSerializer_Write64s(iface->writer, 0));

    kCheck(kSerializer_Flush(iface->writer));

    // Don't care about errors; some interfaces may not allow sending.
    (void)kUdpClient_Send(iface->client, kIpAddress_BroadcastV4(), GO_DISCOVERY_PORT, GO_DISCOVERY_GET_INFO_TIMEOUT, kTRUE);

    return kOK;
}

GoFx(kStatus) GoDiscovery_OnEnumElapsed(GoDiscovery discovery, kPeriodic timer)
{
    kObj(GoDiscovery, discovery);

    if (!obj->enumPending)
    {
        kStatus status = kOK;

        kCheck(kLock_Enter(obj->enumLock));

        kTry
        {
            kTest(kTimer_Start(obj->stopwatch, obj->enumPeriod));
            kTest(GoDiscovery_BeginEnum(discovery));
            obj->enumPending = kTRUE;

            kTest(kPeriodic_Start(obj->eventTimer, GO_DISCOVERY_GET_ADDRESS_TIMEOUT, GoDiscovery_OnEnumElapsed, discovery));
        }
        kCatch(&status)
        {
            kLock_Exit(obj->enumLock);

            kEndCatch(status);
        }
    }
    else
    {
        kCheck(GoDiscovery_OnEnumElapsedRecv(discovery, timer));
    }

    return kOK;
}

GoFx(kStatus) GoDiscovery_OnEnumElapsedRecv(GoDiscovery discovery, kPeriodic timer)
{
    kObj(GoDiscovery, discovery);
    kStatus exception;

    kTry
    {
        kTest(GoDiscovery_EndEnum(discovery, obj->infoList));
        kTest(obj->onEnumerate.function(obj->onEnumerate.receiver, discovery, obj->infoList));
    }
    kCatchEx(&exception)
    {
        DEBUG_PRINT("%s: error %d processing enumeration end", __FUNCTION__, exception);

        // The periodic enumeration cycle must never stop because of errors so
        // ignore the error.
        kEndCatchEx(kOK);
    }
    kFinallyEx
    {
        // Must always clear the enumPending flag and schedule enumeration
        // to start again on the next enumeration interval.
        // Don't check for return errors as there is no recovery action possible.
        obj->enumPending = kFALSE;
        kPeriodic_Start(obj->eventTimer, kTimer_Remaining(obj->stopwatch), GoDiscovery_OnEnumElapsed, discovery);

        kLock_Exit(obj->enumLock);

        kEndFinallyEx();
    }

    return kOK;
}

GoFx(kStatus) GoDiscovery_SetOneInterface(GoDiscovery discovery, kIpAddress* address, kBool enable)
{
    kObj(GoDiscovery, discovery);
    kSize i;

    for (i = 0; i < kArrayList_Count(obj->interfaces); ++i)
    {
        GoDiscoveryInterface* iface = kArrayList_AtT(obj->interfaces, i, GoDiscoveryInterface);

        if (kIpAddress_Equals(*address, iface->address))
        {
            iface->enabled = enable;
        }
    }

    return kOK;
}

GoFx(kStatus) GoDiscovery_SetAllInterface(GoDiscovery discovery, kBool enable)
{
    kObj(GoDiscovery, discovery);
    kSize i;

    for (i = 0; i < kArrayList_Count(obj->interfaces); ++i)
    {
        GoDiscoveryInterface* iface = kArrayList_AtT(obj->interfaces, i, GoDiscoveryInterface);

        iface->enabled = enable;
    }

    return kOK;
}

GoFx(kStatus) GoDiscovery_EnableCompatMode(GoDiscovery discovery, kBool enable)
{
    kObj(GoDiscovery, discovery);

    kAtomic32s_Exchange(&obj->compatEnabled, enable ? 1 : 0);

    return kOK;
}

GoFx(kBool) GoDiscovery_CompatModeEnabled(GoDiscovery discovery)
{
    kObj(GoDiscovery, discovery);

    return kAtomic32s_Get(&obj->compatEnabled) == 1;
}

// This function reads the response id from the received reply in the
// serializer input.
GoFx(kStatus) GoDiscovery_GetReceivedResponseId(GoDiscovery discovery, k64s* responseId)
{
    kObj(GoDiscovery, discovery);
    kStatus exception;
    k64s localResponseId;

    // Read and store the message attributes data length ("attrSize")
    // to determine how many attributes to read.
    // The size field is included in the message size count stored in the size field.
    //
    // If this fails, the EndRead() could assert so skip the EndRead() if BeginRead() fails.
    kCheck(kSerializer_BeginRead(obj->reader, kTypeOf(k64u), kTRUE));

    kTry
    {

        kTest(kSerializer_Read64s(obj->reader, &localResponseId));

        *responseId = localResponseId;
    }
    kCatchEx(&exception)
    {
        kLogf("%s: error %d getting response id", __FUNCTION__, exception);

        kEndCatchEx(exception);
    }
    kFinallyEx
    {
        // Automatically flush any unread data and free serializer read section.
        // Ignore errors.
        kSerializer_EndRead(obj->reader);

        // Reset the received message buffer to the beginning after
        // getting the response id so that functions to parse the contents
        // can be reused.
        kStream_Seek(obj->readerBuffer, 0, kSEEK_ORIGIN_BEGIN);

        kEndFinallyEx();
    }

    return kOK;
}

GoFx(void) GoDiscovery_ParseReceivedEnumReply(GoDiscovery discovery, kIpEndPoint* remoteEndPoint)
{
    kObj(GoDiscovery, discovery);
    k64s responseId;
    GoDiscoveryExtInfo extInfo = kNULL;
    GoDiscoveryInfo info;
    kStatus status;

    if (kSuccess(GoDiscovery_GetReceivedResponseId(discovery, &responseId)))
    {
        // When compatibility mode is on, the reply could either be a GetAddress reply, or
        // a GetExtendedInfo reply.
        if (responseId == GO_DISCOVERY_GET_ADDRESS_REPLY)
        {
            if (!kSuccess(GoDiscoveryInfo_Init(&info)))
            {
                // Skip this message and go on to the next message.
                return;
            }

            if (kSuccess(GoDiscovery_ParseGetAddressReply(discovery, &info, remoteEndPoint)))
            {
                status = GoDiscovery_ProcessGetAddressReply(discovery, &info);
                if (kSuccess(status))
                {
                    DEBUG_PRINT("%s: sensor %u GetAddressReply remote end point\n", __FUNCTION__, info.id);
                    DEBUG_PRINT_ENDPOINT(remoteEndPoint);
                }
                else
                {
                    // Log error but keep processing all the received messages.
                    DEBUG_PRINT("%s: error %d processing get address reply for sensor %u\n", __FUNCTION__, status, info.id);
                }
            }
        }
        else if (responseId == GO_DISCOVERY_GET_INFO_REPLY)
        {
            if (kSuccess(GoDiscovery_ParseGetInfoReply(discovery, &extInfo, kObject_Alloc(discovery))))
            {
                status = GoDiscovery_ProcessGetExtendedInfoReply(discovery, extInfo, remoteEndPoint);
                if (kSuccess(status))
                {
                    DEBUG_PRINT("%s: sensor %u ExtInfoReply remote end point\n", __FUNCTION__, GoDiscoveryExtInfo_Id(extInfo));
                    DEBUG_PRINT_ENDPOINT(remoteEndPoint);
                }
                else
                {
                    // Log error but keep processing all the received messages.
                    DEBUG_PRINT("%s: error %d processing extended info reply for sensor %u\n", __FUNCTION__, status, GoDiscoveryExtInfo_Id(extInfo));
                }
            }

            // Ignore error and continue on to the next reply message in the socket.
            (void) kDisposeRef(&extInfo);
        }
        else
        {
            // Probably got a response for a request sent by a different client thread.
            // The enumeration code should be lenient on what it receives because
            // the network socket is shared with other discovery request APIs
            // (probably not a good idea to begin with. Is there a backwards
            // compatibility reason for not changing that since the APIs are all
            // internal APIs).
            // Enumeration runs periodically so if one run doesn't get what it
            // is expecting, it can try again next time.
            DEBUG_PRINT("%s: received unexpected response id %#x for enumeration\n", __FUNCTION__, responseId);
        }
    }
}


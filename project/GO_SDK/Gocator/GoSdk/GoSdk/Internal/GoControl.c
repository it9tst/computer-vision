/**
 * @file    GoControl.c
 *
 * @internal
 * Copyright (C) 2016-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <GoSdk/Internal/GoControl.h>
#include <kApi/Utils/kUtils.h>
#include <kApi/Data/kImage.h>
#include <kApi/Io/kDat6Serializer.h>

kBeginClassEx(Go, GoControl)
    kAddVMethod(GoControl, kObject, VRelease)
kEndClassEx()

GoFx(kStatus) GoControl_Construct(GoControl* control, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback(allocator);
    kStatus status;

    kCheck(kAlloc_GetObject(alloc, kTypeOf(GoControl), control));

    if (!kSuccess(status = GoControl_Init(*control, kTypeOf(GoControl), alloc)))
    {
        kAlloc_FreeRef(alloc, control);
    }

    return status;
}

GoFx(kStatus) GoControl_Init(GoControl control, kType type, kAlloc alloc)
{
    kObjR(GoControl, control);

    kCheck(kObject_Init(control, type, alloc));
    obj->onCancel.function = kNULL;
    obj->onCancel.receiver = kNULL;
    kZero(obj->protocolVersion);
    obj->busy = kFALSE;
    kZero(obj->channels);
    kZero(obj->streamThread);
    kAtomic32s_Init(&obj->streamIsCancelled, kFALSE);

    obj->isCompatible = kFALSE;
    obj->remoteAddress = kIpAddress_AnyV4();

    obj->control = &obj->channels[GO_CONTROL_CHANNEL_CONTROL];
    obj->control->port = GO_CONTROL_PORT_CONTROL;
    obj->control->parent = control;

    obj->upgrade = &obj->channels[GO_CONTROL_CHANNEL_UPGRADE];
    obj->upgrade->port = GO_CONTROL_PORT_UPGRADE;
    obj->upgrade->parent = control;

    return kOK;
}

GoFx(kStatus) GoControl_VRelease(GoControl control)
{
    kCheck(GoControl_Close(control));

    kCheck(kObject_VRelease(control));

    return kOK;
}

GoFx(kStatus) GoControl_SetRemoteAddress(GoControl control, kIpAddress address)
{
    kObj(GoControl, control);

    obj->remoteAddress = address;

    return kOK;
}

GoFx(kStatus) GoControl_SetCancelHandler(GoControl control, kCallbackFx function, kPointer receiver)
{
    kObj(GoControl, control);

    obj->onCancel.function = function;
    obj->onCancel.receiver = receiver;

    return kOK;
}

GoFx(kStatus) GoControl_SetControlPort(GoControl control, k32u port)
{
    kObj(GoControl, control);

    obj->control->port = port;

    return kOK;
}

GoFx(k32u) GoControl_ControlPort(GoControl control)
{
    kObj(GoControl, control);

    return obj->control->port;
}

GoFx(kStatus) GoControl_SetUpgradePort(GoControl control, k32u port)
{
    kObj(GoControl, control);

    obj->upgrade->port = port;

    return kOK;
}

GoFx(k32u) GoControl_UpgradePort(GoControl control)
{
    kObj(GoControl, control);

    return obj->upgrade->port;
}

GoFx(kStatus) GoControl_InitChannel(GoControl control, GoControlChannel* channel)
{
    kStatus exception = kOK;

    kTry
    {
        kTest(kTcpClient_Construct(&channel->client, kIP_VERSION_4, kObject_Alloc(control)));
        kTest(kTcpClient_SetCancelHandler(channel->client, GoControl_CancelHandler, channel));
        kTest(kTcpClient_SetReadBuffers(channel->client, -1, 4096));
        kTest(kSerializer_Construct(&channel->serializer, channel->client, kTypeOf(kDat6Serializer), kObject_Alloc(control)));

        kTest(kTimer_Construct(&channel->timer, kObject_Alloc(control)));
    }
    kCatch(&exception)
    {
        GoControl_Close(control);
        kEndCatch(exception);
    }

    return kOK;
}

GoFx(kStatus) GoControl_Open(GoControl control, kIpAddress address, k32u controlPort, k32u upgradePort)
{
    kObj(GoControl, control);
    kStatus status;
    k32u i;

    kCheck(GoControl_Close(control));

    kTry
    {
        // New connection being set up, so clear history of previous transactions.
        obj->busy = kFALSE;
        kAtomic32s_Exchange(&obj->streamIsCancelled, kFALSE);

        obj->remoteAddress = address;

        for (i = 0; i < GO_CONTROL_CHANNEL_COUNT; ++i)
        {
            GoControlChannel* channel = &obj->channels[i];

            kTest(GoControl_InitChannel(control, channel));
        }
        //set teh ports
        obj->channels[GO_CONTROL_CHANNEL_CONTROL].port = controlPort;
        obj->channels[GO_CONTROL_CHANNEL_UPGRADE].port = upgradePort;

        kTest(GoControl_ConnectChannel(control, obj->control));

        kTest(GoControl_DetectProtocolVersion(control, &obj->protocolVersion));

        obj->isCompatible = (kVersion_Major(obj->protocolVersion) == kVersion_Major(GoSdk_ProtocolVersion()));
    }
    kCatch(&status)
    {
        GoControl_Close(control);
        kEndCatch(status);
    }

    return kOK;
}

GoFx(kStatus) GoControl_DetectProtocolVersion(GoControl control, kVersion* version)
{
    kObj(GoControl, control);
    GoControlChannel* channel = obj->control;
    k32u size;
    k32u reserved;
    k64u commandId64u;
    k64s status64s;
    k64u major64u, minor64u;

    kCheck(kTimer_Start(channel->timer, GO_CONTROL_GET_PROTOCOL_VERSION_OLD_TIMEOUT));

    kCheck(kSerializer_BeginWrite(channel->serializer, kTypeOf(k32u), kTRUE));
    kCheck(kSerializer_Write32u(channel->serializer, 0));
    kCheck(kSerializer_Write16u(channel->serializer, GO_CONTROL_GET_PROTOCOL_VERSION_OLD));
    kCheck(kSerializer_Write16u(channel->serializer, 0));
    kCheck(kSerializer_Write16u(channel->serializer, 0));
    kCheck(kSerializer_Write16u(channel->serializer, 0));

    kCheck(kSerializer_EndWrite(channel->serializer));
    kCheck(kSerializer_Flush(channel->serializer));

    kCheck(GoControl_Wait(control, channel));

    kCheck(kSerializer_Read32u(channel->serializer, &size));

    if (size == 40)
    {
        kCheck(kSerializer_Read32u(channel->serializer, &reserved));
        kCheck(kSerializer_Read64u(channel->serializer, &commandId64u));
        kCheck(kSerializer_Read64s(channel->serializer, &status64s));
        kCheck(kSerializer_Read64u(channel->serializer, &major64u));
        kCheck(kSerializer_Read64u(channel->serializer, &minor64u));

        *version = kVersion_Create((k32u) major64u, (k32u) minor64u, 0, 0);
    }
    else
    {
        kCheck(kSerializer_AdvanceRead(channel->serializer, size - 4));

        kCheck(GoControl_GetProtocolVersion(control, version));
    }

    return kOK;
}

GoFx(kStatus) GoControl_Close(GoControl control)
{
    kObj(GoControl, control);
    k32u i;

    for (i = 0; i < GO_CONTROL_CHANNEL_COUNT; ++i)
    {
        GoControlChannel* channel = &obj->channels[i];

        kCheck(kDestroyRef(&channel->timer));

        kCheck(kDestroyRef(&channel->serializer));
        kCheck(kDestroyRef(&channel->client));

        channel->isConnected = kFALSE;
    }

    return kOK;
}

GoFx(kBool) GoControl_IsConnected(GoControl control)
{
    kObj(GoControl, control);
    k32u i;

    //GoControl is considered connected if any channel is connected
    for (i = 0; i < GO_CONTROL_CHANNEL_COUNT; ++i)
    {
        GoControlChannel* channel = &obj->channels[i];
        if(channel->isConnected)
        {
            return kTRUE;
        }
    }
    return kFALSE;
}

GoFx(kStatus) GoControl_ConnectChannel(GoControl control, GoControlChannel* channel)
{
    kObj(GoControl, control);

    if (!channel->isConnected)
    {
        kCheck(kTcpClient_Connect(channel->client, obj->remoteAddress, channel->port, GO_CONTROL_CONNECT_TIMEOUT));
        channel->isConnected = kTRUE;
    }

    return kOK;
}

GoFx(kStatus) GoControl_BeginLegacyCommand(GoControl control, GoControlChannel* channel, k16u commandId, k64u timeout)
{
    kObj(GoControl, control);

    channel->commandId = commandId;

    kCheck(kTimer_Start(channel->timer, timeout));

    kCheck(kSerializer_Reset(channel->serializer));

    kCheck(kSerializer_BeginWrite(channel->serializer, kTypeOf(k64u), kTRUE));
    kCheck(kSerializer_Write64u(channel->serializer, commandId));

    return kOK;
}

GoFx(kStatus) GoControl_BeginCommand(GoControl control, GoControlChannel* channel, k16u commandId, k64u timeout)
{
    kObj(GoControl, control);

    if (obj->busy)
    {
        kCheck(GoControl_Receive(control, channel));
        kCheck(GoControl_EndResponse(control, channel));
    }

    channel->commandId = commandId;

    kCheck(kTimer_Start(channel->timer, timeout));

    kCheck(kSerializer_Reset(channel->serializer));

    kCheck(kSerializer_BeginWrite(channel->serializer, kTypeOf(k32u), kTRUE));
    kCheck(kSerializer_Write16u(channel->serializer, commandId));

    return kOK;
}

GoFx(kStatus) GoControl_CancelHandler(kPointer channel, kObject sender, kPointer args)
{
    GoControlChannel* channelObj = channel;
    kObjR(GoControl, channelObj->parent);

    if (kAtomic32s_Get(&obj->streamIsCancelled))
    {
        // This unblocks the read calls when there is no data.
        return kERROR_ABORT;
    }
    else if (kTimer_IsExpired(channelObj->timer))
    {
        return kERROR_TIMEOUT;
    }
    else if (obj->onCancel.function)
    {
        return obj->onCancel.function(obj->onCancel.receiver, channelObj->parent, kNULL);
    }

    return kOK;
}

GoFx(kStatus) GoControl_Send(GoControl control, GoControlChannel* channel)
{
    kObj(GoControl, control);

    kCheck(kSerializer_EndWrite(channel->serializer));
    kCheck(kSerializer_Flush(channel->serializer));

    obj->busy = kTRUE;

    return kOK;
}

GoFx(kStatus) GoControl_Wait(GoControl control, GoControlChannel* channel)
{
    kObj(GoControl, control);
    k64s remaining = kTimer_Remaining(channel->timer);
    kStatus result;

    do
    {
        k64u timeout = kMin_(remaining, GO_CONTROL_CANCEL_QUERY_INTERVAL);

        if (kSuccess(result = kTcpClient_Wait(channel->client, timeout)))
        {
            return kOK;
        }
        else if ((result == kERROR_TIMEOUT) && !kIsNull(obj->onCancel.function))
        {
            kCheck(obj->onCancel.function(obj->onCancel.receiver, control, kNULL));
        }
        else if (result != kERROR_TIMEOUT)
        {
            return result;
        }
    }
    while ((remaining = kTimer_Remaining(channel->timer)) > 0);

    return kERROR_TIMEOUT;
}

GoFx(kStatus) GoControl_Receive(GoControl control, GoControlChannel* channel)
{
    kObj(GoControl, control);
    k16u responseId;
    k32s responseStatus;

    kCheck(GoControl_Wait(control, channel));

    kCheck(kSerializer_BeginRead(channel->serializer, kTypeOf(k32u), kTRUE));
    obj->busy = kFALSE;

    kCheck(kSerializer_Read16u(channel->serializer, &responseId));
    kCheck(kSerializer_Read32s(channel->serializer, &responseStatus));

    kCheck(responseId == channel->commandId);
    if (!kSuccess(responseStatus))
    {
        kCheck(GoControl_EndResponse(control, channel));    //handles situations where there may be additional fields
    }

    kCheck(responseStatus);

    return kOK;
}

GoFx(kStatus) GoControl_LegacyReceive(GoControl control, GoControlChannel* channel)
{
    kObj(GoControl, control);
    k64u responseId;
    k64u responseStatus;

    kCheck(GoControl_Wait(control, channel));

    kCheck(kSerializer_BeginRead(channel->serializer, kTypeOf(k64u), kTRUE));
    obj->busy = kFALSE;

    kCheck(kSerializer_Read64u(channel->serializer, &responseId));
    kCheck(kSerializer_Read64u(channel->serializer, &responseStatus));

    kCheck(responseId == channel->commandId);
    if (!kSuccess((kStatus)responseStatus))
    {
        kCheck(GoControl_EndResponse(control, channel));    //handles situations where there may be additional fields
    }

    kCheck(responseStatus);

    return kOK;
}

GoFx(kStatus) GoControl_LegacySendAndReceive(GoControl control, GoControlChannel* channel)
{
    kCheck(GoControl_Send(control, channel));
    kCheck(GoControl_LegacyReceive(control, channel));
    return kOK;
}

GoFx(kStatus) GoControl_SendAndReceive(GoControl control, GoControlChannel* channel)
{
    kCheck(GoControl_Send(control, channel));
    kCheck(GoControl_Receive(control, channel));
    return kOK;
}

GoFx(kStatus) GoControl_EndResponse(GoControl control, GoControlChannel* channel)
{
    kCheck(kSerializer_EndRead(channel->serializer));
    return kOK;
}

GoFx(kStatus) GoControl_GetProtocolVersion(GoControl control, kVersion* version)
{
    kObj(GoControl, control);
    GoControlChannel* channel = obj->control;
    k8u major, minor;

    kCheck(GoControl_BeginCommand(control, channel, GO_CONTROL_GET_VERSION, GO_CONTROL_GET_VERSION_TIMEOUT));
    kCheck(GoControl_SendAndReceive(control, channel));

    kCheck(kSerializer_Read8u(channel->serializer, &major));
    kCheck(kSerializer_Read8u(channel->serializer, &minor));

    *version = kVersion_Create(major, minor, 0, 0);

    kCheck(GoControl_EndResponse(control, channel));

    return kOK;
}

GoFx(kStatus) GoControl_Login(GoControl control, GoUser user, const kChar* password)
{
    kObj(GoControl, control);
    GoControlChannel* channel = obj->control;
    kText64 passwordText = { 0 };

    kCheckState(obj->isCompatible);

    kCheck(kStrCopy(passwordText, kCountOf(passwordText), password));

    kCheck(GoControl_BeginCommand(control, channel, GO_CONTROL_LOG_IN, GO_CONTROL_LOG_IN_TIMEOUT));

    kCheck(kSerializer_Write32s(channel->serializer, user));
    kCheck(kSerializer_WriteCharArray(channel->serializer, passwordText, kCountOf(passwordText)));

    kCheck(GoControl_SendAndReceive(control, channel));

    kCheck(GoControl_EndResponse(control, channel));

    return kOK;
}

GoFx(kStatus) GoControl_ChangePassword(GoControl control, GoUser user, const kChar* password)
{
    kObj(GoControl, control);
    GoControlChannel* channel = obj->control;
    kText64 passwordText = { 0 };

    kCheckState(obj->isCompatible);

    kCheck(kStrCopy(passwordText, kCountOf(passwordText), password));

    kCheck(GoControl_BeginCommand(control, channel, GO_CONTROL_CHANGE_PASSWORD, GO_CONTROL_CHANGE_PASSWORD_TIMEOUT));

    kCheck(kSerializer_Write32s(channel->serializer, user));
    kCheck(kSerializer_WriteCharArray(channel->serializer, passwordText, kCountOf(passwordText)));

    kCheck(GoControl_SendAndReceive(control, channel));

    kCheck(GoControl_EndResponse(control, channel));

    return kOK;
}

GoFx(kStatus) GoControl_RestoreFactory(GoControl control, kBool restoreAddress)
{
    kObj(GoControl, control);
    GoControlChannel* channel = obj->control;

    kCheck(GoControl_BeginCommand(control, channel, GO_CONTROL_RESTORE_FACTORY, GO_CONTROL_RESTORE_FACTORY_TIMEOUT));

    kCheck(kSerializer_Write8u(channel->serializer, (k8u) restoreAddress));

    kCheck(GoControl_SendAndReceive(control, channel));

    kCheck(GoControl_EndResponse(control, channel));

    return kOK;
}

GoFx(kStatus) GoControl_Reset(GoControl control)
{
    kObj(GoControl, control);
    GoControlChannel* channel = obj->control;

    kCheck(GoControl_BeginCommand(control, channel, GO_CONTROL_RESET, GO_CONTROL_RESET_TIMEOUT));
    kCheck(GoControl_SendAndReceive(control, channel));

    kCheck(GoControl_EndResponse(control, channel));

    return kOK;
}

GoFx(kStatus) GoControl_ResetEncoder(GoControl control)
{
    kObj(GoControl, control);
    GoControlChannel* channel = obj->control;

    kCheck(GoControl_BeginCommand(control, channel, GO_CONTROL_RESET_ENCODER, GO_CONTROL_RESET_ENCODER_TIMEOUT));
    kCheck(GoControl_SendAndReceive(control, channel));

    kCheck(GoControl_EndResponse(control, channel));

    return kOK;
}

GoFx(kStatus) GoControl_BeginUpgrade(GoControl control, void* data, kSize size)
{
    kObj(GoControl, control);
    GoControlChannel* channel = obj->upgrade;

    // upgrade channel is connected on demand
    if (!(channel->isConnected) && kIsNull(channel->client))
    {
        kCheck(GoControl_InitChannel(control, channel));
    }

    kCheck(GoControl_ConnectChannel(control, channel));
    kCheck(GoControl_BeginLegacyCommand(control, channel, GO_CONTROL_BEGIN_UPGRADE, GO_CONTROL_BEGIN_UPGRADE_TIMEOUT));

    kCheck(kSerializer_Write64s(channel->serializer, (k64s) size));
    kCheck(kSerializer_WriteByteArray(channel->serializer, data, size));
    kCheck(GoControl_LegacySendAndReceive(control, channel));
    kCheck(GoControl_EndResponse(control, channel));

    return kOK;
}

GoFx(kStatus) GoControl_GetUpgradeStatus(GoControl control, kBool* complete, kBool* succeeded, k32s* progress)
{
    kObj(GoControl, control);
    k32s stage, prgrss;

    kCheck(GoControl_GetUpgradeStatusEx(control, &stage, &prgrss));

    *complete = (stage < 1 || stage == 2);
    *succeeded = (stage == 0);
    *progress = (k32s) prgrss;

    return kOK;
}

GoFx(kStatus) GoControl_GetUpgradeStatusEx(GoControl control, k32s* stage, k32s* progress)
{
    kObj(GoControl, control);
    GoControlChannel* channel = obj->upgrade;
    k64s stg, prgrss;

    if (!(channel->isConnected) && kIsNull(channel->client))
    {
        kCheck(GoControl_InitChannel(control, channel));
    }

    // upgrade channel is connected on demand
    kCheck(GoControl_ConnectChannel(control, channel));

    kCheck(GoControl_BeginLegacyCommand(control, channel, GO_CONTROL_GET_UPGRADE_STATUS, GO_CONTROL_GET_UPGRADE_STATUS_TIMEOUT));
    kCheck(GoControl_LegacySendAndReceive(control, channel));

    kCheck(kSerializer_Read64s(channel->serializer, &stg));
    kCheck(kSerializer_Read64s(channel->serializer, &prgrss));

    *stage = (k32s) stg;
    *progress = (k32s) prgrss;

    kCheck(GoControl_EndResponse(control, channel));

    return kOK;
}

GoFx(kStatus) GoControl_BeginAlignment(GoControl control)
{
    kObj(GoControl, control);
    GoControlChannel* channel = obj->control;

    kCheckState(obj->isCompatible);

    kCheck(GoControl_BeginCommand(control, channel, GO_CONTROL_START_ALIGNMENT, GO_CONTROL_START_ALIGNMENT_TIMEOUT));

    kCheck(GoControl_Send(control, channel));

    return kOK;
}

GoFx(kStatus) GoControl_EndAlignment(GoControl control)
{
    kObj(GoControl, control);
    GoControlChannel* channel = obj->control;
    k32s opId;

    kCheck(GoControl_Receive(control, channel));

    kCheck(kSerializer_Read32s(channel->serializer, &opId));

    kCheck(GoControl_EndResponse(control, channel));

    return kOK;
}

GoFx(kStatus) GoControl_ClearAlignment(GoControl control)
{
    kObj(GoControl, control);
    GoControlChannel* channel = obj->control;

    kCheck(GoControl_BeginCommand(control, channel, GO_CONTROL_CLEAR_ALIGNMENT, GO_CONTROL_CLEAR_ALIGNMENT_TIMEOUT));
    kCheck(GoControl_SendAndReceive(control, channel));

    kCheck(GoControl_EndResponse(control, channel));

    return kOK;
}

GoFx(kStatus) GoControl_SetAlignmentReference(GoControl control, GoAlignmentRef reference)
{
    kObj(GoControl, control);
    GoControlChannel* channel = obj->control;

    kCheckState(obj->isCompatible);

    kCheck(GoControl_BeginCommand(control, channel, GO_CONTROL_SET_ALIGNMENT_REFERENCE, GO_CONTROL_SET_ALIGNMENT_REFERENCE_TIMEOUT));
    kCheck(kSerializer_Write32s(channel->serializer, reference));

    kCheck(GoControl_SendAndReceive(control, channel));

    kCheck(GoControl_EndResponse(control, channel));

    return kOK;
}

GoFx(kStatus) GoControl_GetAlignmentReference(GoControl control, GoAlignmentRef* reference)
{
    kObj(GoControl, control);
    GoControlChannel* channel = obj->control;
    kText16 modeBuffer = { 0 };

    kCheckState(obj->isCompatible);

    kCheck(GoControl_BeginCommand(control, channel, GO_CONTROL_GET_ALIGNMENT_REFERENCE, GO_CONTROL_GET_ALIGNMENT_REFERENCE_TIMEOUT));

    kCheck(GoControl_SendAndReceive(control, channel));

    kCheck(kSerializer_Read32s(channel->serializer, reference));

    kCheck(GoControl_EndResponse(control, channel));

    return kOK;
}

GoFx(kStatus) GoControl_BeginExposureAutoSet(GoControl control, GoRole role)
{
    kObj(GoControl, control);
    GoControlChannel* channel = obj->control;

    kCheckState(obj->isCompatible);

    kCheck(GoControl_BeginCommand(control, channel, GO_CONTROL_START_EXPOSURE_AUTO_SET, GO_CONTROL_START_EXPOSURE_AUTO_SET_TIMEOUT));
    kCheck(kSerializer_Write32s(channel->serializer, (k32s)role));

    kCheck(GoControl_Send(control, channel));

    return kOK;
}

GoFx(kStatus) GoControl_EndExposureAutoSet(GoControl control)
{
    kObj(GoControl, control);
    GoControlChannel* channel = obj->control;
    k32s opId;

    kCheck(GoControl_Receive(control, channel));

    kCheck(kSerializer_Read32s(channel->serializer, &opId));

    kCheck(GoControl_EndResponse(control, channel));

    return kOK;
}

GoFx(kStatus) GoControl_BeginStart(GoControl control)
{
    kObj(GoControl, control);
    GoControlChannel* channel = obj->control;

    kCheckState(obj->isCompatible);

    kCheck(GoControl_BeginCommand(control, channel, GO_CONTROL_START, GO_CONTROL_START_TIMEOUT));

    kCheck(GoControl_Send(control, channel));

    return kOK;
}

GoFx(kStatus) GoControl_EndStart(GoControl control)
{
    kObj(GoControl, control);
    GoControlChannel* channel = obj->control;

    kCheck(GoControl_Receive(control, channel));

    kCheck(GoControl_EndResponse(control, channel));

    return kOK;
}

GoFx(kStatus) GoControl_BeginScheduledStart(GoControl control, k64s value)
{
    kObj(GoControl, control);
    GoControlChannel* channel = obj->control;

    kCheckState(obj->isCompatible);

    kCheck(GoControl_BeginCommand(control, channel, GO_CONTROL_SCHEDULED_START, GO_CONTROL_SCHEDULED_START_TIMEOUT));
    kCheck(kSerializer_Write64s(channel->serializer, value));

    kCheck(GoControl_Send(control, channel));

    return kOK;
}

GoFx(kStatus) GoControl_EndScheduledStart(GoControl control)
{
    kObj(GoControl, control);
    GoControlChannel* channel = obj->control;

    kCheck(GoControl_Receive(control, channel));

    kCheck(GoControl_EndResponse(control, channel));

    return kOK;
}

GoFx(kStatus) GoControl_BeginStop(GoControl control)
{
    kObj(GoControl, control);
    GoControlChannel* channel = obj->control;

    kCheck(GoControl_BeginCommand(control, channel, GO_CONTROL_STOP, GO_CONTROL_STOP_TIMEOUT));
    kCheck(GoControl_Send(control, channel));

    return kOK;
}

GoFx(kStatus) GoControl_EndStop(GoControl control)
{
    kObj(GoControl, control);
    GoControlChannel* channel = obj->control;

    kCheck(GoControl_Receive(control, channel));

    kCheck(GoControl_EndResponse(control, channel));

    return kOK;
}

GoFx(kStatus) GoControl_BeginSnapshot(GoControl control)
{
    kObj(GoControl, control);
    GoControlChannel* channel = obj->control;

    kCheckState(obj->isCompatible);

    kCheck(GoControl_BeginCommand(control, channel, GO_CONTROL_SNAPSHOT, GO_CONTROL_SNAPSHOT_TIMEOUT));

    kCheck(GoControl_Send(control, channel));

    return kOK;
}

GoFx(kStatus) GoControl_EndSnapshot(GoControl control)
{
    kObj(GoControl, control);
    GoControlChannel* channel = obj->control;

    kCheck(GoControl_Receive(control, channel));

    kCheck(GoControl_EndResponse(control, channel));

    return kOK;
}

GoFx(kStatus) GoControl_ReadFileList(GoControl control, kArrayList files, const kChar* extensionFilter)
{
    kObj(GoControl, control);
    GoControlChannel* channel = obj->control;
    k32u count;
    kText64 filter = { 0 };
    kText64 name = { 0 };
    k32u i;

    kCheckState(obj->isCompatible);

    kCheck(kArrayList_Allocate(files, kTypeOf(kText64), 0));

    if (extensionFilter)
    {
        kCheck(kStrCopy(filter, sizeof(filter), extensionFilter));
    }

    kCheck(GoControl_BeginCommand(control, channel, GO_CONTROL_GET_FILE_LIST, GO_CONTROL_GET_FILE_LIST_TIMEOUT));
    kCheck(kSerializer_WriteCharArray(channel->serializer, filter, kCountOf(filter)));

    kCheck(GoControl_SendAndReceive(control, channel));
    kCheck(kSerializer_Read32u(channel->serializer, &count));

    for (i = 0; i < count; ++i)
    {
        kCheck(kSerializer_ReadCharArray(channel->serializer, name, kCountOf(name)));
        kCheck(kArrayList_AddT(files, &name));
    }

    kCheck(GoControl_EndResponse(control, channel));


    return kOK;
}

GoFx(kStatus) GoControl_ReadFile(GoControl control, const kChar* fileName, kByte** data, kSize* size, kAlloc allocator)
{
    kObj(GoControl, control);
    kAlloc alloc = kAlloc_Fallback(allocator);
    GoControlChannel* channel = obj->control;
    kText64 name = { 0 };
    kByte* output = kNULL;
    k32u temp;
    kSize fileSize;
    kStatus status;

    kCheckState(obj->isCompatible);

    kTry
    {
        kTest(kStrCopy(name, kCountOf(name), fileName));

        kTest(GoControl_BeginCommand(control, channel, GO_CONTROL_READ_FILE, GO_CONTROL_READ_FILE_TIMEOUT));
        kTest(kSerializer_WriteCharArray(channel->serializer, name, kCountOf(name)));

        kTest(GoControl_SendAndReceive(control, channel));

        kTest(kSerializer_Read32u(channel->serializer, &temp));
        fileSize = (kSize)temp;

        kTest(kAlloc_Get(alloc, fileSize, &output));

        kTest(kSerializer_ReadByteArray(channel->serializer, output, fileSize));

        kTest(GoControl_EndResponse(control, channel));

        *data = output;
        *size = fileSize;
    }
    kCatch(&status)
    {
        kAlloc_Free(alloc, output);
        kEndCatch(status);
    }

    return kOK;
}

GoFx(kStatus) GoControl_ClearLog(GoControl control)
{
    kObj(GoControl, control);
    GoControlChannel* channel = obj->control;

    kCheckState(obj->isCompatible);

    kCheck(GoControl_BeginCommand(control, channel, GO_CONTROL_CLEAR_LOG, GO_CONTROL_DEFAULT_TIMEOUT));

    kCheck(GoControl_Send(control, channel));

    return kOK;
}

GoFx(kStatus) GoControl_WriteFile(GoControl control, const kChar* fileName, const kByte* data, kSize size)
{
    kObj(GoControl, control);
    GoControlChannel* channel = obj->control;
    kText64 name = { 0 };

    kCheckState(obj->isCompatible);

    kCheck(kStrCopy(name, kCountOf(name), fileName));

    kCheck(GoControl_BeginCommand(control, channel, GO_CONTROL_WRITE_FILE, GO_CONTROL_WRITE_FILE_TIMEOUT));
    kCheck(kSerializer_WriteCharArray(channel->serializer, name, kCountOf(name)));
    kCheck(kSerializer_Write32u(channel->serializer, (k32u)size));
    kCheck(kSerializer_WriteByteArray(channel->serializer, data, size));

    kCheck(GoControl_SendAndReceive(control, channel));

    kCheck(GoControl_EndResponse(control, channel));

    return kOK;
}

GoFx(kStatus) GoControl_CopyFile(GoControl control, const kChar* source, const kChar* destination)
{
    kObj(GoControl, control);
    GoControlChannel* channel = obj->control;
    kText64 srcName = { 0 };
    kText64 destName = { 0 };

    kCheckState(obj->isCompatible);

    kCheck(kStrCopy(srcName, kCountOf(srcName), source));
    kCheck(kStrCopy(destName, kCountOf(destName), destination));

    kCheck(GoControl_BeginCommand(control, channel, GO_CONTROL_COPY_FILE, GO_CONTROL_COPY_FILE_TIMEOUT));
    kCheck(kSerializer_WriteCharArray(channel->serializer, srcName, kCountOf(srcName)));
    kCheck(kSerializer_WriteCharArray(channel->serializer, destName, kCountOf(destName)));

    kCheck(GoControl_SendAndReceive(control, channel));

    kCheck(GoControl_EndResponse(control, channel));

    return kOK;
}

GoFx(kStatus) GoControl_DeleteFile(GoControl control, const kChar* fileName)
{
    kObj(GoControl, control);
    GoControlChannel* channel = obj->control;
    kText64 name = { 0 };

    kCheckState(obj->isCompatible);

    kCheck(kStrCopy(name, kCountOf(name), fileName));

    kCheck(GoControl_BeginCommand(control, channel, GO_CONTROL_DELETE_FILE, GO_CONTROL_DELETE_FILE_TIMEOUT));
    kCheck(kSerializer_WriteCharArray(channel->serializer, name, kCountOf(name)));

    kCheck(GoControl_SendAndReceive(control, channel));

    kCheck(GoControl_EndResponse(control, channel));

    return kOK;
}

GoFx(kStatus) GoControl_GetUserStorageFree(GoControl control, k64u* spaceAvailable)
{
    kObj(GoControl, control);
    GoControlChannel* channel = obj->control;
    k64u temp = 0;

    kCheckState(obj->isCompatible);

    kCheck(GoControl_BeginCommand(control, channel, GO_CONTROL_USER_STORAGE_FREE, GO_CONTROL_USER_STORAGE_FREE_TIMEOUT));

    kCheck(GoControl_SendAndReceive(control, channel));

    kCheck(kSerializer_Read64u(channel->serializer, &temp));
    *spaceAvailable = temp;

    kCheck(GoControl_EndResponse(control, channel));

    return kOK;
}

GoFx(kStatus) GoControl_GetUserStorageUsed(GoControl control, k64u* spaceUsed)
{
    kObj(GoControl, control);
    GoControlChannel* channel = obj->control;
    k64u temp = 0;

    kCheckState(obj->isCompatible);

    kCheck(GoControl_BeginCommand(control, channel, GO_CONTROL_USER_STORAGE_USED, GO_CONTROL_USER_STORAGE_USED_TIMEOUT));

    kCheck(GoControl_SendAndReceive(control, channel));

    kCheck(kSerializer_Read64u(channel->serializer, &temp));
    *spaceUsed = temp;

    kCheck(GoControl_EndResponse(control, channel));

    return kOK;
}

GoFx(kStatus) GoControl_GetDefaultJob(GoControl control, kChar* fileName, kSize capacity)
{
    kObj(GoControl, control);
    GoControlChannel* channel = obj->control;
    kText64 name = { 0 };

    kCheckState(obj->isCompatible);

    kCheck(GoControl_BeginCommand(control, channel, GO_CONTROL_GET_DEFAULT_JOB, GO_CONTROL_GET_DEFAULT_JOB_TIMEOUT));

    kCheck(GoControl_SendAndReceive(control, channel));

    kCheck(kSerializer_ReadCharArray(channel->serializer, name, kCountOf(name)));
    kCheck(kStrCopy(fileName, capacity, name));

    kCheck(GoControl_EndResponse(control, channel));

    return kOK;
}

GoFx(kStatus) GoControl_SetDefaultJob(GoControl control, const kChar* fileName)
{
    kObj(GoControl, control);
    GoControlChannel* channel = obj->control;
    kText64 name = { 0 };

    kCheckState(obj->isCompatible);

    kCheck(kStrCopy(name, kCountOf(name), fileName));

    kCheck(GoControl_BeginCommand(control, channel, GO_CONTROL_SET_DEFAULT_JOB, GO_CONTROL_SET_DEFAULT_JOB_TIMEOUT));
    kCheck(kSerializer_WriteCharArray(channel->serializer, name, kCountOf(name)));

    kCheck(GoControl_SendAndReceive(control, channel));

    kCheck(GoControl_EndResponse(control, channel));

    return kOK;
}

GoFx(kStatus) GoControl_GetLoadedJob(GoControl control, kChar* fileName, kSize capacity, kBool* isModified)
{
    kObj(GoControl, control);
    GoControlChannel* channel = obj->control;
    kText64 name = { 0 };
    k8u modified;

    kCheckState(obj->isCompatible);

    kCheck(GoControl_BeginCommand(control, channel, GO_CONTROL_GET_LOADED_JOB, GO_CONTROL_GET_LOADED_FILE_TIMEOUT));

    kCheck(GoControl_SendAndReceive(control, channel));

    kCheck(kSerializer_ReadCharArray(channel->serializer, name, kCountOf(name)));
    kCheck(kStrCopy(fileName, capacity, name));

    kCheck(kSerializer_Read8u(channel->serializer, &modified));
    *isModified = (kBool) modified;

    kCheck(GoControl_EndResponse(control, channel));

    return kOK;
}

GoFx(kStatus) GoControl_GetSensorInfo(GoControl control, GoSensorInfo localInfo, kArrayList remoteInfoList)
{
    kObj(GoControl, control);
    GoControlChannel* channel = obj->control;
    k32s i, remoteCount;
    kStatus status = kOK;

    kCheckState(obj->isCompatible);

    kCheck(GoControl_BeginCommand(control, channel, GO_CONTROL_GET_SYSTEM_INFO, GO_CONTROL_GET_SYSTEM_INFO_TIMEOUT));

    kCheck(GoControl_SendAndReceive(control, channel));


    //
    kCheck(GoSensorInfo_Read(localInfo, channel->serializer));

    kCheck(kSerializer_Read32s(channel->serializer, &remoteCount));

    kCheck(kArrayList_Purge(remoteInfoList));

    for (i = 0; i < remoteCount; i++)
    {
        GoSensorInfo remoteInfo = kNULL;

        kTry
        {
            kTest(GoSensorInfo_Construct(&remoteInfo, kObject_Alloc(control)));
            kTest(GoSensorInfo_Read(remoteInfo, channel->serializer));
            kTest(kArrayList_AddT(remoteInfoList, &remoteInfo));
        }
        kCatch(&status)
        {
            kDestroyRef(&remoteInfo);
            kEndCatch(status);
        }
    }

    kCheck(GoControl_EndResponse(control, channel));

    return kOK;
}

GoFx(kStatus) GoControl_GetSensorInfoV2(GoControl control, GoSensorInfo localInfo, kArrayList remoteInfoList, kArrayList buddyInfoList)
{
    kObj(GoControl, control);
    GoControlChannel* channel = obj->control;
    k32u i, remoteCount, buddyCount;
    k16u localInfoSize, remoteInfoSize, buddyInfoSize;
    kStatus status = kOK;

    kCheckState(obj->isCompatible);

    kCheck(GoControl_BeginCommand(control, channel, GO_CONTROL_GET_SYSTEM_INFO_2, GO_CONTROL_GET_SYSTEM_INFO_2_TIMEOUT));

    kCheck(GoControl_SendAndReceive(control, channel));

    kCheck(kSerializer_Read16u(channel->serializer, &localInfoSize));

    //read info
    kCheck(GoSensorInfo_ReadV2(localInfo, channel->serializer, localInfoSize, kTRUE));

    kCheck(kSerializer_Read32u(channel->serializer, &remoteCount));

    kCheck(kSerializer_Read16u(channel->serializer, &remoteInfoSize));

    kCheck(kArrayList_Purge(remoteInfoList));

    for (i = 0; i < remoteCount; i++)
    {
        GoSensorInfo remoteInfo = kNULL;

        kTry
        {
            kTest(GoSensorInfo_Construct(&remoteInfo, kObject_Alloc(control)));
            kTest(GoSensorInfo_ReadV2(remoteInfo, channel->serializer, remoteInfoSize, kFALSE));
            kTest(kArrayList_AddT(remoteInfoList, &remoteInfo));
        }
        kCatch(&status)
        {
            kDestroyRef(&remoteInfo);
            kEndCatch(status);
        }
    }

    kCheck(kSerializer_Read32u(channel->serializer, &buddyCount));

    kCheck(kSerializer_Read16u(channel->serializer, &buddyInfoSize));

    kCheck(kArrayList_Purge(buddyInfoList));

    if (!kIsNull(buddyInfoList) && buddyCount > 0)
    {
        GoBuddyInfo buddyInfo;
        for (i = 0; i < buddyCount; i++)
        {
            k32u bytesRead = 8;
            kCheck(kSerializer_Read32u(channel->serializer, &buddyInfo.id));
            kCheck(kSerializer_Read32s(channel->serializer, &buddyInfo.state));
            kCheck(kArrayList_AddT(buddyInfoList, &buddyInfo));

            kCheck(kSerializer_AdvanceRead(channel->serializer, buddyInfoSize - bytesRead));
        }
    }

    kCheck(GoControl_EndResponse(control, channel));

    return kOK;
}

GoFx(kVersion) GoControl_ProtocolVersion(GoControl control)
{
    kObj(GoControl, control);
    return obj->protocolVersion;
}

GoFx(kBool) GoControl_IsCompatible(GoControl control)
{
    kObj(GoControl, control);
    return obj->isCompatible;
}

GoFx(kStatus) GoControl_GetStates(GoControl control, GoStates* states)
{
    kObj(GoControl, control);
    GoControlChannel* channel = obj->control;
    k32u itemCount;
    k32u temp32u;

    kCheckState(obj->isCompatible);

    // Important to zero because some fields may be skipped due to compat logic.
    kCheck(kMemZero(states, sizeof(GoStates)));

    kCheck(GoControl_BeginCommand(control, channel, GO_CONTROL_GET_STATES, GO_CONTROL_GET_STATES_TIMEOUT));

    kCheck(GoControl_SendAndReceive(control, channel));

    //read states
    kCheck(kSerializer_Read32u(channel->serializer, &itemCount));     //item count
    kCheck(kSerializer_Read32s(channel->serializer, &states->sensorState));
    kCheck(kSerializer_Read32s(channel->serializer, &states->loginType));
    kCheck(kSerializer_Read32s(channel->serializer, &states->alignmentReference));
    kCheck(kSerializer_Read32s(channel->serializer, &states->alignmentState));
    kCheck(kSerializer_Read32s(channel->serializer, &states->recordingEnabled));
    kCheck(kSerializer_Read32s(channel->serializer, &states->playbackSource));
    kCheck(kSerializer_Read32u(channel->serializer, &states->uptimeSec));
    kCheck(kSerializer_Read32u(channel->serializer, &states->uptimeMicrosec));
    kCheck(kSerializer_Read32u(channel->serializer, &states->playbackPos));
    kCheck(kSerializer_Read32u(channel->serializer, &states->playbackCount));

    kCheck(kSerializer_Read32u(channel->serializer, &temp32u));
    states->autoStartEnabled = (kBool)temp32u;

    if (itemCount > 11) // backwards compatibility support
    {
        kCheck(kSerializer_Read32u(channel->serializer, &temp32u));
        states->isAccelerator = (kBool)temp32u;
    }

    if (itemCount > 12) // backwards compatibility support
    {
        kCheck(kSerializer_Read32u(channel->serializer, &temp32u));
        states->voltage = (GoVoltageSetting)temp32u;
    }

    if (itemCount > 13) // backwards compatibility support
    {
        kCheck(kSerializer_Read32u(channel->serializer, &temp32u));
        states->cableLength = temp32u;
    }

    if (itemCount > 14) // Backwards compatibility for quickedit state
    {
        kCheck(kSerializer_Read32u(channel->serializer, &temp32u));
        states->quickEditEnabled = (kBool)temp32u;
    }

    if (itemCount > 15) // Backwards compatibility for security state
    {
        kCheck(kSerializer_Read32u(channel->serializer, &temp32u));
        states->security = (k32s)temp32u;
    }

    if (itemCount > 16) // Backwards compatibility for branding type
    {
        kCheck(kSerializer_Read32u(channel->serializer, &temp32u));
        states->brandingType = (k32s)temp32u;
    }

    kCheck(GoControl_EndResponse(control, channel));

    return kOK;
}

GoFx(kStatus) GoControl_GetScanMode(GoControl control, GoMode* mode)
{
    kObj(GoControl, control);
    GoControlChannel* channel = obj->control;
    kText16 modeBuffer = { 0 };

    kCheckState(obj->isCompatible);

    kCheck(GoControl_BeginCommand(control, channel, GO_CONTROL_GET_MODE, GO_CONTROL_GET_MODE_TIMEOUT));

    kCheck(GoControl_SendAndReceive(control, channel));

    kCheck(kSerializer_Read32s(channel->serializer, mode));

    kCheck(GoControl_EndResponse(control, channel));

    return kOK;
}

GoFx(kStatus) GoControl_ChangeBuddy(GoControl control, kBool add, k32u buddyId)
{
    kObj(GoControl, control);
    GoControlChannel* channel = obj->control;

    kCheckState(obj->isCompatible);

    kCheck(GoControl_BeginCommand(control, channel, GO_CONTROL_CHANGE_BUDDY, GO_CONTROL_CHANGE_BUDDY_TIMEOUT));

    if (add)
    {
        kCheck(kSerializer_Write32u(channel->serializer, buddyId));
    }
    else
    {
        kCheck(kSerializer_Write32u(channel->serializer, 0));
    }

    kCheck(GoControl_SendAndReceive(control, channel));

    kCheck(GoControl_EndResponse(control, channel));

    return kOK;
}

GoFx(kStatus) GoControl_AssignBuddies(GoControl control, const k32u* buddyIds, kSize count)
{
    kObj(GoControl, control);
    GoControlChannel* channel = obj->control;
    kSize index;

    kCheckState(obj->isCompatible);

    kCheck(GoControl_BeginCommand(control, channel, GO_CONTROL_ASSIGN_BUDDIES, GO_CONTROL_ASSIGN_BUDDIES_TIMEOUT));

    kCheck(kSerializer_Write32u(channel->serializer, (k32u) count));

    for (index = 0; index < count; index++)
    {
        kCheck(kSerializer_Write32u(channel->serializer, buddyIds[index]));
    }

    kCheck(GoControl_SendAndReceive(control, channel));

    kCheck(GoControl_EndResponse(control, channel));

    return kOK;
}

GoFx(kStatus) GoControl_RemoveBuddiesById(GoControl control, const k32u* buddyIds, kSize count)
{
    kObj(GoControl, control);
    GoControlChannel* channel = obj->control;
    kSize index;

    kCheckState(obj->isCompatible);
    kCheckArgs(count > 0);

    kCheck(GoControl_BeginCommand(control, channel, GO_CONTROL_REMOVE_BUDDIES_BY_ID, GO_CONTROL_REMOVE_BUDDIES_TIMEOUT));

    kCheck(kSerializer_Write32u(channel->serializer, (k32u)count));

    for (index = 0; index < count; index++)
    {
        kCheck(kSerializer_Write32u(channel->serializer, buddyIds[index]));
    }

    kCheck(GoControl_SendAndReceive(control, channel));

    kCheck(GoControl_EndResponse(control, channel));

    return kOK;
}

GoFx(kStatus) GoControl_RemoveBuddies(GoControl control, const k32u* buddyIndices, kSize count)
{
    kObj(GoControl, control);
    GoControlChannel* channel = obj->control;
    kSize index;

    kCheckState(obj->isCompatible);
    kCheckArgs(count > 0);

    kCheck(GoControl_BeginCommand(control, channel, GO_CONTROL_REMOVE_BUDDIES, GO_CONTROL_REMOVE_BUDDIES_TIMEOUT));

    kCheck(kSerializer_Write32u(channel->serializer, (k32u) count));

    for (index = 0; index < count; index++)
    {
        kCheck(kSerializer_Write32u(channel->serializer, buddyIndices[index]));
    }

    kCheck(GoControl_SendAndReceive(control, channel));

    kCheck(GoControl_EndResponse(control, channel));

    return kOK;
}

GoFx(kStatus) GoControl_GetTimestamp(GoControl control, k64u* time)
{
    kObj(GoControl, control);
    GoControlChannel* channel = obj->control;

    kCheckState(obj->isCompatible);

    kCheck(GoControl_BeginCommand(control, channel, GO_CONTROL_GET_TIME_STAMP, GO_CONTROL_GET_TIME_STAMP_TIMEOUT));
    kCheck(GoControl_SendAndReceive(control, channel));

    kCheck(kSerializer_Read64u(channel->serializer, time));

    kCheck(GoControl_EndResponse(control, channel));

    return kOK;
}

GoFx(kStatus) GoControl_GetEncoder(GoControl control, k64s* encoder)
{
    kObj(GoControl, control);
    GoControlChannel* channel = obj->control;

    kCheckState(obj->isCompatible);

    kCheck(GoControl_BeginCommand(control, channel, GO_CONTROL_GET_ENCODER, GO_CONTROL_GET_ENCODER_TIMEOUT));
    kCheck(GoControl_SendAndReceive(control, channel));

    kCheck(kSerializer_Read64s(channel->serializer, encoder));

    kCheck(GoControl_EndResponse(control, channel));

    return kOK;
}

GoFx(kStatus) GoControl_Trigger(GoControl control)
{
    kObj(GoControl, control);
    GoControlChannel* channel = obj->control;

    kCheckState(obj->isCompatible);

    kCheck(GoControl_BeginCommand(control, channel, GO_CONTROL_TRIGGER, GO_CONTROL_TRIGGER_TIMEOUT));
    kCheck(GoControl_SendAndReceive(control, channel));

    kCheck(GoControl_EndResponse(control, channel));

    return kOK;
}

GoFx(kStatus) GoControl_Backup(GoControl control, kByte** fileData, kSize* size, kAlloc allocator)
{
    kObj(GoControl, control);
    kAlloc alloc = kAlloc_Fallback(allocator);
    GoControlChannel* channel = obj->control;
    kByte* output = kNULL;
    k32u temp;
    kSize fileSize;
    kStatus status;

    kCheckState(obj->isCompatible);

    kTry
    {
        kTest(GoControl_BeginCommand(control, channel, GO_CONTROL_BACKUP, GO_CONTROL_BACKUP_TIMEOUT));
        kTest(GoControl_SendAndReceive(control, channel));

        kTest(kSerializer_Read32u(channel->serializer, &temp));
        fileSize = (kSize)temp;

        kTest(kAlloc_Get(alloc, fileSize, &output));

        kTest(kSerializer_ReadByteArray(channel->serializer, output, fileSize));

        kTest(GoControl_EndResponse(control, channel));

        *fileData = output;
        *size = fileSize;
    }
    kCatch(&status)
    {
        kAlloc_Free(alloc, output);
        kEndCatch(status);
    }

    return kOK;
}

GoFx(kStatus) GoControl_Restore(GoControl control, const kByte* fileData, kSize size)
{
    kObj(GoControl, control);
    GoControlChannel* channel = obj->control;

    kCheckState(obj->isCompatible);

    kCheck(GoControl_BeginCommand(control, channel, GO_CONTROL_RESTORE, GO_CONTROL_RESTORE_TIMEOUT));
    kCheck(kSerializer_Write32u(channel->serializer, (k32u)size));
    kCheck(kSerializer_WriteByteArray(channel->serializer, fileData, size));

    kCheck(GoControl_SendAndReceive(control, channel));

    kCheck(GoControl_EndResponse(control, channel));

    return kOK;
}

GoFx(kStatus) GoControl_ScheduleDigital(GoControl control, k16u index, k64s target, k8u value)
{
    kObj(GoControl, control);
    GoControlChannel* channel = obj->control;

    kCheckState(obj->isCompatible);

    kCheck(GoControl_BeginCommand(control, channel, GO_CONTROL_SCHEDULE_DIGITAL, GO_CONTROL_SCHEDULE_DIGITAL_TIMEOUT));
    kCheck(kSerializer_Write16u(channel->serializer, index));
    kCheck(kSerializer_Write64s(channel->serializer, target));
    kCheck(kSerializer_Write8u(channel->serializer, value));

    kCheck(GoControl_SendAndReceive(control, channel));

    kCheck(GoControl_EndResponse(control, channel));

    return kOK;
}

GoFx(kStatus) GoControl_ScheduleAnalog(GoControl control, k16u index, k64s target, k32s value)
{
    kObj(GoControl, control);
    GoControlChannel* channel = obj->control;

    kCheckState(obj->isCompatible);

    kCheck(GoControl_BeginCommand(control, channel, GO_CONTROL_SCHEDULE_ANALOG, GO_CONTROL_SCHEDULE_ANALOG_TIMEOUT));
    kCheck(kSerializer_Write16u(channel->serializer, index));
    kCheck(kSerializer_Write64s(channel->serializer, target));
    kCheck(kSerializer_Write32s(channel->serializer, value));

    kCheck(GoControl_SendAndReceive(control, channel));

    kCheck(GoControl_EndResponse(control, channel));

    return kOK;
}

GoFx(kStatus) GoControl_SetRecordingEnabled(GoControl control, kBool enable)
{
    kObj(GoControl, control);
    GoControlChannel* channel = obj->control;

    kCheckState(obj->isCompatible);

    kCheck(GoControl_BeginCommand(control, channel, GO_CONTROL_SET_RECORDING_ENABLED, GO_CONTROL_DEFAULT_TIMEOUT));
    kCheck(kSerializer_Write8u(channel->serializer, (k8u)enable));

    kCheck(GoControl_SendAndReceive(control, channel));

    kCheck(GoControl_EndResponse(control, channel));

    return kOK;
}

GoFx(kStatus) GoControl_GetRecordingEnabled(GoControl control, kBool* enabled)
{
    kObj(GoControl, control);
    GoControlChannel* channel = obj->control;
    k8u tempVal;

    kCheckState(obj->isCompatible);

    kCheck(GoControl_BeginCommand(control, channel, GO_CONTROL_GET_RECORDING_ENABLED, GO_CONTROL_DEFAULT_TIMEOUT));

    kCheck(GoControl_SendAndReceive(control, channel));
    kCheck(kSerializer_Read8u(channel->serializer, &tempVal));

    kCheck(GoControl_EndResponse(control, channel));

    if (tempVal == 1)
    {
        *enabled = kTRUE;
    }
    else
    {
        *enabled = kFALSE;
    }

    return kOK;
}

GoFx(kStatus) GoControl_SetInputSource(GoControl control, GoDataSource source)
{
    kObj(GoControl, control);
    GoControlChannel* channel = obj->control;

    kCheckState(obj->isCompatible);

    kCheck(GoControl_BeginCommand(control, channel, GO_CONTROL_SET_INPUT_SOURCE, GO_CONTROL_SET_INPUT_SOURCE_TIMEOUT));
    kCheck(kSerializer_Write32s(channel->serializer, source));

    kCheck(GoControl_SendAndReceive(control, channel));

    kCheck(GoControl_EndResponse(control, channel));

    return kOK;
}

GoFx(kStatus) GoControl_GetInputSource(GoControl control, GoInputSource* source)
{
    kObj(GoControl, control);
    GoControlChannel* channel = obj->control;

    kCheckState(obj->isCompatible);

    kCheck(GoControl_BeginCommand(control, channel, GO_CONTROL_GET_INPUT_SOURCE, GO_CONTROL_GET_INPUT_SOURCE_TIMEOUT));

    kCheck(GoControl_SendAndReceive(control, channel));
    kCheck(kSerializer_Read32s(channel->serializer, source));

    kCheck(GoControl_EndResponse(control, channel));

    return kOK;
}

GoFx(kStatus) GoControl_ClearReplayData(GoControl control)
{
    kObj(GoControl, control);
    GoControlChannel* channel = obj->control;

    kCheckState(obj->isCompatible);

    kCheck(GoControl_BeginCommand(control, channel, GO_CONTROL_CLEAR_REPLAY_DATA, GO_CONTROL_DEFAULT_TIMEOUT));

    kCheck(GoControl_SendAndReceive(control, channel));

    kCheck(GoControl_EndResponse(control, channel));

    return kOK;
}

GoFx(kStatus) GoControl_Simulate(GoControl control, kBool* isBufferValid)
{
    kObj(GoControl, control);
    GoControlChannel* channel = obj->control;
    k8u bufferValid;

    kCheckState(obj->isCompatible);

    kCheck(GoControl_BeginCommand(control, channel, GO_CONTROL_SIMULATE, GO_CONTROL_DEFAULT_TIMEOUT));
    kCheck(GoControl_SendAndReceive(control, channel));
    kCheck(kSerializer_Read8u(channel->serializer, &bufferValid));
    kCheck(GoControl_EndResponse(control, channel));

    if (bufferValid == 0)
    {
        *isBufferValid = kFALSE;
    }
    else
    {
        *isBufferValid = kTRUE;
    }

    return kOK;
}

GoFx(kStatus) GoControl_ClearMeasurementStats(GoControl control)
{
    kObj(GoControl, control);
    GoControlChannel* channel = obj->control;

    kCheckState(obj->isCompatible);

    kCheck(GoControl_BeginCommand(control, channel, GO_CONTROL_CLEAR_MEASUREMENT_STATS, GO_CONTROL_DEFAULT_TIMEOUT));

    kCheck(GoControl_SendAndReceive(control, channel));

    kCheck(GoControl_EndResponse(control, channel));

    return kOK;
}

GoFx(kStatus) GoControl_PlaybackSeek(GoControl control, kSize position)
{
    kObj(GoControl, control);
    GoControlChannel* channel = obj->control;

    kCheckState(obj->isCompatible);

    kCheck(GoControl_BeginCommand(control, channel, GO_CONTROL_PLAYBACK_SEEK, GO_CONTROL_DEFAULT_TIMEOUT));
    kCheck(kSerializer_Write32u(channel->serializer, (k32u)position));

    kCheck(GoControl_SendAndReceive(control, channel));

    kCheck(GoControl_EndResponse(control, channel));

    return kOK;
}

GoFx(kStatus) GoControl_PlaybackStep(GoControl control, GoSeekDirection direction)
{
    kObj(GoControl, control);
    GoControlChannel* channel = obj->control;

    kCheckState(obj->isCompatible);

    kCheck(GoControl_BeginCommand(control, channel, GO_CONTROL_STEP_PLAYBACK, GO_CONTROL_DEFAULT_TIMEOUT));
    kCheck(kSerializer_Write32s(channel->serializer, direction));

    kCheck(GoControl_SendAndReceive(control, channel));

    kCheck(GoControl_EndResponse(control, channel));

    return kOK;
}

GoFx(kStatus) GoControl_PlaybackPosition(GoControl control, kSize* position, kSize* count)
{
    kObj(GoControl, control);
    GoControlChannel* channel = obj->control;
    k32u temp;
    kCheckState(obj->isCompatible);

    kCheck(GoControl_BeginCommand(control, channel, GO_CONTROL_PLAYBACK_POSITION, GO_CONTROL_PLAYBACK_POSITION_TIMEOUT));

    kCheck(GoControl_SendAndReceive(control, channel));
    kCheck(kSerializer_Read32u(channel->serializer, &temp));
    *position = (kSize)temp;

    kCheck(kSerializer_Read32u(channel->serializer, &temp));
    *count = (kSize)temp;

    kCheck(GoControl_EndResponse(control, channel));

    return kOK;
}

GoFx(kStatus) GoControl_ReadChunkedData(GoControl control, const kChar* dstFileName)
{
    kObj(GoControl, control);
    GoControlChannel* channel = obj->control;
    kMemory outputBuffer = kNULL;
    kByte* currentChunkData = kNULL;
    k32u temp, currentProgress;
    kSize chunkDataSize = 0;
    k32u chunkSize;
    k16u chunkType;
    k32s chunkStatus;
    k32u dataSize;
    kBool bufferInitialized = kFALSE;
    kFile file = kNULL;
    kBool done = kFALSE;

    kTry
    {
        kTest(kMemory_Construct(&outputBuffer, kObject_Alloc(control)));

        while (!done)
        {
            kTest(kSerializer_Read32u(channel->serializer, &chunkSize));
            kTest(kSerializer_Read16u(channel->serializer, &chunkType));
            kTest(kSerializer_Read32s(channel->serializer, &chunkStatus));
            kTest(kSerializer_Read32u(channel->serializer, &temp));
            kTest(kSerializer_Read32u(channel->serializer, &currentProgress));
            kTest(kSerializer_Read32u(channel->serializer, &dataSize));

            kTestArgs(chunkType == GO_CONTROL_CONTINUE && chunkStatus == kOK);

            if (dataSize == 0)
            {
                done = kTRUE;
                kTest(kAlloc_FreeRef(kObject_Alloc(control), &currentChunkData));
            }
            else
            {
                if (!bufferInitialized)
                {
                    chunkDataSize = (kSize)dataSize;
                    kTest(kMemory_Allocate(outputBuffer, chunkDataSize));
                    kTest(kAlloc_Get(kObject_Alloc(control), chunkDataSize, &currentChunkData));

                    bufferInitialized = kTRUE;
                }

                if (chunkDataSize != (kSize)dataSize)
                {
                    kTest(kAlloc_FreeRef(kObject_Alloc(control), &currentChunkData));
                    chunkDataSize = (kSize)dataSize;
                    kTest(kAlloc_Get(kObject_Alloc(control), chunkDataSize, &currentChunkData));
                }

                kTest(kSerializer_ReadByteArray(channel->serializer, currentChunkData, chunkDataSize));
                kTest(kStream_Write(outputBuffer, currentChunkData, chunkDataSize));
            }
        }

        kTest(kFile_Construct(&file, dstFileName, kFILE_MODE_WRITE, kObject_Alloc(control)));
        kTest(kStream_Write(file, kMemory_At(outputBuffer, 0), (kSize)kMemory_Length(outputBuffer)));
        kTest(kFile_Close(file));
    }
    kFinally
    {
        kAlloc_FreeRef(kObject_Alloc(control), &currentChunkData);
        kDestroyRef(&file);
        kDestroyRef(&outputBuffer);
        kEndFinally();
    }

    return kOK;
}

GoFx(kStatus) GoControl_ReadChunkedDataStream(kPointer context)
{
    GoControlChunkedDataStreamInfo* info = (GoControlChunkedDataStreamInfo*)context;
    GoControl control = xGoControl_Cast(info->control);
    kObj(GoControl, control);
    GoControlChannel* channel = obj->control;
    kByte* currentChunkData = kNULL;
    k32u temp, currentProgress;
    kSize chunkDataSize = 0;
    k32u chunkSize;
    k16u chunkType;
    k32s chunkStatus;
    k32u dataSize;
    kBool bufferInitialized = kFALSE;
    kFile file = kNULL;
    kBool done = kFALSE;
    kAlloc alloc = kObject_Alloc(control);

    kTry
    {
        kTest(kFile_Construct(&file, info->destPath, kFILE_MODE_WRITE, alloc));

        while (!done)
        {
            kTest(kSerializer_Read32u(channel->serializer, &chunkSize));
            kTest(kSerializer_Read16u(channel->serializer, &chunkType));
            kTest(kSerializer_Read32s(channel->serializer, &chunkStatus));
            kTest(kSerializer_Read32u(channel->serializer, &temp));
            kTest(kSerializer_Read32u(channel->serializer, &currentProgress));
            kTest(kSerializer_Read32u(channel->serializer, &dataSize));

            kTestArgs(chunkType == GO_CONTROL_CONTINUE && chunkStatus == kOK);

            if (chunkDataSize != (kSize)dataSize)
            {
                kTest(kAlloc_FreeRef(alloc, &currentChunkData));
                chunkDataSize = (kSize)dataSize;
                kTest(kAlloc_Get(alloc, chunkDataSize, &currentChunkData));
            }

            kTest(kSerializer_ReadByteArray(channel->serializer, currentChunkData, chunkDataSize));
            kTest(kStream_Write(file, currentChunkData, chunkDataSize));

            done = GoControl_IsStreamingStopped(control);
        }

        kTest(kSerializer_Flush(channel->serializer));

        kTest(kFile_Close(file));
    }
    kFinally
    {
        kAlloc_FreeRef(alloc, &currentChunkData);
        kAlloc_Free(alloc, context);
        kDestroyRef(&file);
        kEndFinally();
    }

    return kOK;
}

GoFx(kStatus) GoControl_StartStreamingChunkedData(GoControl control, const kChar* dstFileName)
{
    kObj(GoControl, control);
    kStatus exception = kOK;
    kAlloc alloc = kObject_Alloc(control);
    GoControlChunkedDataStreamInfo* context = kNULL;

    kTry
    {
        kTest(kThread_Construct(&obj->streamThread, kObject_Alloc(control)));
        kTest(kAlloc_Get(alloc, sizeof(GoControlChunkedDataStreamInfo), &context));

        context->control = control;
        kTest(kStrCopy(context->destPath, sizeof(context->destPath), dstFileName));

        kTest(kThread_Start(obj->streamThread, GoControl_ReadChunkedDataStream, (kPointer)context)); // context will get freed by GoControl_ReadChunkedDataStream
    }
    kCatch(&exception)
    {
        if (obj->streamThread)
        {
            // Leave cancel flag on, because the stream can have partially read data and
            // therefore no longer useable.
            kAtomic32s_Exchange(&obj->streamIsCancelled, kTRUE);
            kDestroyRef(&obj->streamThread);
        }

        if (!kIsNull(context)) { kAlloc_Free(alloc, context); }

        kEndCatch(exception);
    }

    return kOK;
}

GoFx(kStatus) GoControl_StopStreaming(GoControl control)
{
    kObj(GoControl, control);
    kAlloc alloc = kObject_Alloc(control);

    // Note we leave the cancel flag on. The command channel can
    // no longer be used. This is OK because there can be partially
    // streamed data over the channel, making it unusable anyway.
    // Reconnection will reset this flag.
    kAtomic32s_Exchange(&obj->streamIsCancelled, kTRUE);

    // wait until the thread actually shuts down
    kCheck(kThread_Join(obj->streamThread, kINFINITE, kNULL));

    // clean up the thread
    kDestroyRef(&obj->streamThread);

    return kOK;
}

GoFx(kBool) GoControl_IsStreamingStopped(GoControl control)
{
    kObj(GoControl, control);

    return kAtomic32s_Get(&obj->streamIsCancelled);
}

GoFx(kStatus) GoControl_ExportBitmap(GoControl control,
                                     GoReplayExportSourceType type,
                                     GoDataSource source,
                                     const kChar* dstFileName)
{
    kObj(GoControl, control);
    GoControlChannel* channel = obj->control;
    k32u temp, currentProgress;

    kCheckState(obj->isCompatible);

    if (type == GO_REPLAY_EXPORT_SOURCE_PRIMARY)
    {
        return kERROR;
    }

    kCheck(GoControl_BeginCommand(control, channel, GO_CONTROL_EXPORT_BITMAP, GO_CONTROL_EXPORT_BITMAP_TIMEOUT)); //no timeout
    kCheck(kSerializer_Write32s(channel->serializer, type));
    kCheck(kSerializer_Write32s(channel->serializer, source));
    kCheck(GoControl_SendAndReceive(control, channel));

    kCheck(kSerializer_Read32u(channel->serializer, &temp));
    kCheck(kSerializer_Read32u(channel->serializer, &currentProgress));

    kCheck(GoControl_EndResponse(control, channel));

    kCheck(GoControl_ReadChunkedData(control, dstFileName));

    return kOK;
}

GoFx(kStatus) GoControl_ReadFileStreamed(GoControl control, const kChar* srcFileName, const kChar* dstFileName, kBool continuous)
{
    kObj(GoControl, control);
    GoControlChannel* channel = obj->control;
    k32u temp, currentProgress;
    kText64 name;

    kCheckState(obj->isCompatible);

    kCheck(GoControl_BeginCommand(control, channel, GO_CONTROL_READ_FILE_STREAMED, GO_CONTROL_READ_FILE_STREAMED_TIMEOUT)); //no timeout
    kCheck(kStrCopy(name, kCountOf(name), srcFileName));
    kCheck(kSerializer_WriteCharArray(channel->serializer, name, kCountOf(name)));
    kCheck(GoControl_SendAndReceive(control, channel));

    kCheck(kSerializer_Read32u(channel->serializer, &temp)); //progress total
    kCheck(kSerializer_Read32u(channel->serializer, &currentProgress));

    kCheck(GoControl_EndResponse(control, channel));

    if (continuous)
    {
        kCheck(GoControl_StartStreamingChunkedData(control, dstFileName));
    }
    else
    {
        kCheck(GoControl_ReadChunkedData(control, dstFileName));
    }

    return kOK;
}

GoFx(kStatus) GoControl_ExportCsv(GoControl control, const kChar* dstFileName)
{
    kObj(GoControl, control);
    GoControlChannel* channel = obj->control;
    k32u temp, currentProgress;

    kCheckState(obj->isCompatible);

    kCheck(GoControl_BeginCommand(control, channel, GO_CONTROL_EXPORT_CSV, GO_CONTROL_EXPORT_CSV_TIMEOUT)); //no timeout
    kCheck(GoControl_SendAndReceive(control, channel));

    kCheck(kSerializer_Read32u(channel->serializer, &temp)); //progress total
    kCheck(kSerializer_Read32u(channel->serializer, &currentProgress));

    kCheck(GoControl_EndResponse(control, channel));

    kCheck(GoControl_ReadChunkedData(control, dstFileName));

    return kOK;
}

GoFx(kStatus) GoControl_SetAutoStartEnabled(GoControl control, kBool enable)
{
    kObj(GoControl, control);
    GoControlChannel* channel = obj->control;

    kCheckState(obj->isCompatible);

    kCheck(GoControl_BeginCommand(control, channel, GO_CONTROL_SET_AUTOSTART_ENABLED, GO_CONTROL_SET_AUTOSTART_ENABLED_TIMEOUT));
    kCheck(kSerializer_Write8u(channel->serializer, (k8u)enable));

    kCheck(GoControl_SendAndReceive(control, channel));

    kCheck(GoControl_EndResponse(control, channel));

    return kOK;
}

GoFx(kStatus) GoControl_GetAutoStartEnabled(GoControl control, kBool* enabled)
{
    kObj(GoControl, control);
    GoControlChannel* channel = obj->control;
    kText64 name = { 0 };
    k8u tempEnable = kFALSE;

    kCheckState(obj->isCompatible);

    kCheck(GoControl_BeginCommand(control, channel, GO_CONTROL_GET_AUTOSTART_ENABLED, GO_CONTROL_GET_AUTOSTART_ENABLED_TIMEOUT));

    kCheck(GoControl_SendAndReceive(control, channel));

    kCheck(kSerializer_Read8u(channel->serializer, &tempEnable));

    if (tempEnable != 0)
    {
        *enabled = kTRUE;
    }
    else
    {
        *enabled = kFALSE;
    }

    kCheck(GoControl_EndResponse(control, channel));

    return kOK;
}

GoFx(kStatus) GoControl_SetVoltage(GoControl control, GoVoltageSetting voltage, k32u cableLength)
{
    kObj(GoControl, control);
    GoControlChannel* channel = obj->control;

    kCheckState(obj->isCompatible);

    kCheck(GoControl_BeginCommand(control, channel, GO_CONTROL_SET_VOLTAGE, GO_CONTROL_SET_VOLTAGE_TIMEOUT));
    kCheck(kSerializer_Write16u(channel->serializer, voltage));
    kCheck(kSerializer_Write32u(channel->serializer, cableLength));

    kCheck(GoControl_SendAndReceive(control, channel));

    kCheck(GoControl_EndResponse(control, channel));

    return kOK;
}

GoFx(kStatus) GoControl_GetVoltage(GoControl control, GoVoltageSetting *voltage, k32u *cableLength)
{
    kObj(GoControl, control);
    GoControlChannel* channel = obj->control;

    k16u v;
    k32u cl;

    kCheckState(obj->isCompatible);

    kCheck(GoControl_BeginCommand(control, channel, GO_CONTROL_GET_VOLTAGE, GO_CONTROL_GET_VOLTAGE_TIMEOUT));

    kCheck(GoControl_SendAndReceive(control, channel));

    kCheck(kSerializer_Read16u(channel->serializer, &v));
    kCheck(kSerializer_Read32u(channel->serializer, &cl));

    kCheck(GoControl_EndResponse(control, channel));

    if (voltage) *voltage = v;
    if (cableLength) *cableLength = cl;

    return kOK;
}

GoFx(kStatus) GoControl_SetQuickEditEnabled(GoControl control, kBool enable)
{
    kObj(GoControl, control);
    GoControlChannel* channel = obj->control;

    kCheckState(obj->isCompatible);

    kCheck(GoControl_BeginCommand(control, channel, GO_CONTROL_SET_QUICK_EDIT_ENABLED, GO_CONTROL_DEFAULT_TIMEOUT));
    kCheck(kSerializer_Write8u(channel->serializer, (k8u)enable));

    kCheck(GoControl_SendAndReceive(control, channel));

    kCheck(GoControl_EndResponse(control, channel));

    return kOK;
}

GoFx(kStatus) GoControl_GetQuickEditEnabled(GoControl control, kBool* enabled)
{
    kObj(GoControl, control);
    GoControlChannel* channel = obj->control;
    kText64 name = { 0 };
    k8u tempEnable = kFALSE;

    kCheckState(obj->isCompatible);

    kCheck(GoControl_BeginCommand(control, channel, GO_CONTROL_GET_QUICK_EDIT_ENABLED, GO_CONTROL_DEFAULT_TIMEOUT));

    kCheck(GoControl_SendAndReceive(control, channel));

    kCheck(kSerializer_Read8u(channel->serializer, &tempEnable));

    if (tempEnable != 0)
    {
        *enabled = kTRUE;
    }
    else
    {
        *enabled = kFALSE;
    }

    kCheck(GoControl_EndResponse(control, channel));

    return kOK;
}

GoFx(kStatus) GoControl_CreateModel(GoControl control, const kChar* name)
{
    kObj(GoControl, control);
    GoControlChannel* channel = obj->control;
    kText64 fileName = { 0 };

    kCheckState(obj->isCompatible);

    kCheck(GoControl_BeginCommand(control, channel, GO_CONTROL_CREATE_MODEL, GO_CONTROL_CREATE_MODEL_TIMEOUT));

    kCheck(kStrCopy(fileName, sizeof(fileName), name));
    kCheck(kSerializer_WriteCharArray(channel->serializer, fileName, 64));

    kCheck(GoControl_SendAndReceive(control, channel));

    kCheck(GoControl_EndResponse(control, channel));

    return kOK;
}

GoFx(kStatus) GoControl_DetectModelEdges(GoControl control, const kChar* name, k16u sensitivity)
{
    kObj(GoControl, control);
    GoControlChannel* channel = obj->control;
    kText64 fileName = { 0 };

    kCheckState(obj->isCompatible);

    kCheck(GoControl_BeginCommand(control, channel, GO_CONTROL_DETECT_MODEL_EDGES, GO_CONTROL_DETECT_MODEL_EDGES_TIMEOUT));

    kCheck(kStrCopy(fileName, sizeof(fileName), name));
    kCheck(kSerializer_WriteCharArray(channel->serializer, fileName, 64));
    kCheck(kSerializer_Write16u(channel->serializer, sensitivity));
    kCheck(GoControl_SendAndReceive(control, channel));

    kCheck(GoControl_EndResponse(control, channel));

    return kOK;
}

GoFx(kStatus) GoControl_ListDirectory(GoControl control, const kChar* extension, const kChar* root, kBool isRecursive, kArrayList fileList)
{
    kObj(GoControl, control);
    GoControlChannel* channel = obj->control;
    kText64 textString = { 0 };
    kText64 filter = { 0 };
    k32u fileNameCount;
    kSize i;

    kCheckState(obj->isCompatible);

    kCheck(kArrayList_Allocate(fileList, kTypeOf(kText64), 0));

    kCheck(GoControl_BeginCommand(control, channel, GO_CONTROL_LIST_DIRECTORY, GO_CONTROL_LIST_DIRECTORY_TIMEOUT));

    if (extension) {
        kCheck(kStrCopy(filter, sizeof(filter), extension));
    }
    kCheck(kSerializer_WriteCharArray(channel->serializer, filter, kCountOf(filter)));

    kCheck(kStrCopy(textString, sizeof(textString), root));
    kCheck(kSerializer_WriteCharArray(channel->serializer, textString, 64));

    kCheck(kSerializer_Write8u(channel->serializer, (k8u)isRecursive));

    kCheck(GoControl_SendAndReceive(control, channel));

    kCheck(kSerializer_Read32u(channel->serializer, &fileNameCount));

    for (i = 0; i < fileNameCount; i++)
    {
        kCheck(kSerializer_ReadCharArray(channel->serializer, textString, 64));
        kCheck(kArrayList_AddT(fileList, &textString));
    }

    kCheck(GoControl_EndResponse(control, channel));

    return kOK;
}

GoFx(kStatus) GoControl_SetIsEmulator(GoControl control, kBool value)
{
    kObj(GoControl, control);
    GoControlChannel* channel = obj->control;

    kCheckState(obj->isCompatible);

    kCheck(GoControl_BeginCommand(control, channel, GO_CONTROL_SET_IS_EMULATOR, GO_CONTROL_SET_IS_EMULATOR_TIMEOUT));
    kCheck(kSerializer_Write8u(channel->serializer, (k8u)value));
    kCheck(GoControl_SendAndReceive(control, channel));
    kCheck(GoControl_EndResponse(control, channel));

    return kOK;
}

GoFx(kStatus) GoControl_SetReplayProtectionEnabled(GoControl control, kBool enable)
{
    kObj(GoControl, control);
    GoControlChannel* channel = obj->control;

    kCheckState(obj->isCompatible);

    kCheck(GoControl_BeginCommand(control, channel, GO_CONTROL_SET_REPLAY_PROTECTION_ENABLED, GO_CONTROL_SET_REPLAY_PROTECTION_ENABLED_TIMEOUT));
    kCheck(kSerializer_Write8u(channel->serializer, (k8u)enable));
    kCheck(GoControl_SendAndReceive(control, channel));
    kCheck(GoControl_EndResponse(control, channel));

    return kOK;
}

GoFx(kStatus) GoControl_GetReplayProtectionEnabled(GoControl control, kBool* enabled)
{
    kObj(GoControl, control);
    GoControlChannel* channel = obj->control;
    kText64 name = { 0 };
    k8u tempEnable = kFALSE;

    kCheckState(obj->isCompatible);

    kCheck(GoControl_BeginCommand(control, channel, GO_CONTROL_GET_REPLAY_PROTECTION_ENABLED, GO_CONTROL_GET_REPLAY_PROTECTION_ENABLED_TIMEOUT));
    kCheck(GoControl_SendAndReceive(control, channel));
    kCheck(kSerializer_Read8u(channel->serializer, &tempEnable));
    kCheck(GoControl_EndResponse(control, channel));

    *enabled = !!tempEnable;

    return kOK;
}

GoFx(kStatus) GoControl_AddTool(GoControl control, const kChar* type, const kChar* name)
{
    kObj(GoControl, control);
    GoControlChannel* channel = obj->control;
    kText64 tempText;

    kCheckState(obj->isCompatible);
    kCheck(kStrCopy(tempText, kCountOf(tempText), type));

    kCheck(GoControl_BeginCommand(control, channel, GO_CONTROL_ADD_TOOL, GO_CONTROL_ADD_TOOL_TIMEOUT));

    kCheck(kSerializer_WriteCharArray(channel->serializer, tempText, kCountOf(tempText)));
    kCheck(kStrCopy(tempText, kCountOf(tempText), name));
    kCheck(kSerializer_WriteCharArray(channel->serializer, tempText, kCountOf(tempText)));

    kCheck(GoControl_SendAndReceive(control, channel));

    kCheck(GoControl_EndResponse(control, channel));

    return kOK;
}

GoFx(kStatus) GoControl_AddMeasurement(GoControl control, kSize index, const kChar* type, const kChar* name)
{
    kObj(GoControl, control);
    GoControlChannel* channel = obj->control;
    kText64 tempText;

    kCheckState(obj->isCompatible);
    kCheck(kStrCopy(tempText, kCountOf(tempText), type));

    kCheck(GoControl_BeginCommand(control, channel, GO_CONTROL_ADD_MEASUREMENT, GO_CONTROL_ADD_MEASUREMENT_TIMEOUT));

    kCheck(kSerializer_Write32u(channel->serializer, (k32u)index));
    kCheck(kSerializer_WriteCharArray(channel->serializer, tempText, kCountOf(tempText)));

    kCheck(kStrCopy(tempText, kCountOf(tempText), name));
    kCheck(kSerializer_WriteCharArray(channel->serializer, tempText, kCountOf(tempText)));

    kCheck(GoControl_SendAndReceive(control, channel));

    kCheck(GoControl_EndResponse(control, channel));

    return kOK;
}

GoFx(kStatus) GoControl_SetRamImageEnabled(GoControl control, kBool enable)
{
    kObj(GoControl, control);
    GoControlChannel* channel = obj->control;

    kCheckState(obj->isCompatible);
    kCheck(GoControl_BeginCommand(control, channel, GO_CONTROL_ENABLE_RAM_IMAGE, GO_CONTROL_ENABLE_RAM_IMAGE_TIMEOUT));

    kCheck(kSerializer_Write8u(channel->serializer, (k8u)enable));

    kCheck(GoControl_SendAndReceive(control, channel));
    kCheck(GoControl_EndResponse(control, channel));

    return kOK;
}

GoFx(kStatus) GoControl_GetRamImageEnabled(GoControl control, kBool* enabled)
{
    kObj(GoControl, control);
    GoControlChannel* channel = obj->control;
    k8u tempEnable;

    kCheckState(obj->isCompatible);
    kCheck(GoControl_BeginCommand(control, channel, GO_CONTROL_RAM_IMAGE_ENABLED, GO_CONTROL_ENABLE_RAM_IMAGE_TIMEOUT));

    kCheck(GoControl_SendAndReceive(control, channel));

    kCheck(kSerializer_Read8u(channel->serializer, &tempEnable));

    if (tempEnable != 0)
    {
        *enabled = kTRUE;
    }
    else
    {
        *enabled = kFALSE;
    }

    kCheck(GoControl_EndResponse(control, channel));

    return kOK;
}

GoFx(kStatus) GoControl_WriteRamImage(GoControl control, GoRole role, kSize cameraIndex, kSize stateIndex, kSize frameIndex, kImage image)
{
    kObj(GoControl, control);
    GoControlChannel* channel = obj->control;
    kImage tempImage = kNULL;
    k32s pixelType = 0;

    kCheckState(obj->isCompatible);

    kCheck(GoControl_BeginCommand(control, channel, GO_CONTROL_WRITE_RAM_IMAGE, GO_CONTROL_WRITE_RAM_IMAGE_TIMEOUT));

    kCheck(kSerializer_Write32s(channel->serializer, role));
    kCheck(kSerializer_Write32u(channel->serializer, (k32u)cameraIndex));
    kCheck(kSerializer_Write32u(channel->serializer, (k32u)stateIndex));
    kCheck(kSerializer_Write32u(channel->serializer, (k32u)frameIndex));
    kCheck(kSerializer_WriteObject(channel->serializer, image));

    kCheck(GoControl_SendAndReceive(control, channel));
    kCheck(GoControl_EndResponse(control, channel));

    return kOK;
}

GoFx(kStatus) GoControl_RuntimeVariableCount(GoControl control, kSize* count)
{
    kObj(GoControl, control);
    GoControlChannel* channel = obj->control;
    k32u varCount;

    kCheckState(obj->isCompatible);
    kCheckArgs(!kIsNull(count));

    kCheck(GoControl_BeginCommand(control, channel, GO_CONTROL_GET_RUNTIME_VARIABLE_COUNT, GO_CONTROL_GET_RUNTIME_VARIABLE_COUNT_TIMEOUT));

    kCheck(GoControl_SendAndReceive(control, channel));
    kCheck(kSerializer_Read32u(channel->serializer, &varCount));

    *count = (kSize)varCount;

    kCheck(GoControl_EndResponse(control, channel));

    return kOK;
}

GoFx(kStatus) GoControl_GetRuntimeVariables(GoControl control, kSize startIndex, kSize length, k32s* values)
{
    kObj(GoControl, control);
    GoControlChannel* channel = obj->control;
    kSize i;
    k32u temp;

    kCheckState(obj->isCompatible);
    kCheckArgs(!kIsNull(values));

    kCheck(GoControl_BeginCommand(control, channel, GO_CONTROL_GET_RUNTIME_VARIABLES, GO_CONTROL_GET_RUNTIME_VARIABLES_TIMEOUT));

    kCheck(kSerializer_Write32u(channel->serializer, (k32u)startIndex));
    kCheck(kSerializer_Write32u(channel->serializer, (k32u)length));
    kCheck(GoControl_SendAndReceive(control, channel));

    kCheck(kSerializer_Read32u(channel->serializer, &temp));
    kCheck(kSerializer_Read32u(channel->serializer, &temp));

    for (i = 0; i < length; i++)
    {
        kCheck(kSerializer_Read32s(channel->serializer, &values[i]));
    }

    kCheck(GoControl_EndResponse(control, channel));

    return kOK;
}

GoFx(kStatus) GoControl_SetRuntimeVariables(GoControl control, kSize startIndex, kSize length, const k32s* values)
{
    kObj(GoControl, control);
    GoControlChannel* channel = obj->control;
    kSize i;

    kCheckState(obj->isCompatible);
    kCheckArgs(!kIsNull(values));

    kCheck(GoControl_BeginCommand(control, channel, GO_CONTROL_SET_RUNTIME_VARIABLES, GO_CONTROL_SET_RUNTIME_VARIABLES_TIMEOUT));

    kCheck(kSerializer_Write32u(channel->serializer, (k32u)startIndex));
    kCheck(kSerializer_Write32u(channel->serializer, (k32u)length));

    for (i = 0; i < length; i++)
    {
        kCheck(kSerializer_Write32s(channel->serializer, values[i]));
    }

    kCheck(GoControl_SendAndReceive(control, channel));

    kCheck(GoControl_EndResponse(control, channel));

    return kOK;
}

GoFx(kStatus) GoControl_SetLeanAndMean(GoControl control, kBool leanAndMean)
{
    kObj(GoControl, control);
    GoControlChannel* channel = obj->control;

    kCheckState(obj->isCompatible);
    kCheck(GoControl_BeginCommand(control, channel, GO_CONTROL_SET_LEAN_AND_MEAN, GO_CONTROL_SET_LEAN_AND_MEAN_TIMEOUT));

    kCheck(kSerializer_Write8u(channel->serializer, (k8u)leanAndMean));

    kCheck(GoControl_SendAndReceive(control, channel));
    kCheck(GoControl_EndResponse(control, channel));

    return kOK;
}

GoFx(kStatus) GoControl_SetSecurityLevel(GoControl control, GoSecurityLevel security)
{
    kObj(GoControl, control);
    GoControlChannel* channel = obj->control;

    kCheckState(obj->isCompatible);

    kCheck(GoControl_BeginCommand(control, channel, GO_CONTROL_SET_SECURITY_LEVEL, GO_CONTROL_SECURITY_LEVEL_TIMEOUT));
    kCheck(kSerializer_Write32s(channel->serializer, security));

    kCheck(GoControl_SendAndReceive(control, channel));
    kCheck(GoControl_EndResponse(control, channel));

    return kOK;
}

GoFx(kStatus) GoControl_SetFlag(GoControl control, const kChar* name, const kChar* value)
{
    kObj(GoControl, control);
    GoControlChannel* channel = obj->control;
    kText256 nameBuffer;
    k32u valueLen;

    kCheckState(obj->isCompatible);

    kCheckArgs(kStrCopy(nameBuffer, kCountOf(nameBuffer), name));
    valueLen = (k32u)kStrLength(value);

    kCheck(GoControl_BeginCommand(control, channel, GO_CONTROL_SET_FLAG, GO_CONTROL_SET_FLAG_TIMEOUT));

    kCheck(kSerializer_WriteCharArray(channel->serializer, nameBuffer, kCountOf(nameBuffer)));
    kCheck(kSerializer_Write32u(channel->serializer, valueLen));
    kCheck(kSerializer_WriteCharArray(channel->serializer, value, valueLen));

    kCheck(GoControl_SendAndReceive(control, channel));
    kCheck(GoControl_EndResponse(control, channel));

    return kOK;
}

GoFx(kStatus) GoControl_GetFlag(GoControl control, const kChar* name, kString value)
{
    kObj(GoControl, control);
    GoControlChannel* channel = obj->control;
    kText256 nameBuffer;
    k32u valueLen;

    kCheckState(obj->isCompatible);

    kCheckArgs(kStrCopy(nameBuffer, kCountOf(nameBuffer), name));

    kCheck(GoControl_BeginCommand(control, channel, GO_CONTROL_GET_FLAG, GO_CONTROL_GET_FLAG_TIMEOUT));

    kCheck(kSerializer_WriteCharArray(channel->serializer, nameBuffer, kCountOf(nameBuffer)));

    kCheck(GoControl_SendAndReceive(control, channel));

    kCheck(kSerializer_Read32u(channel->serializer, &valueLen));
    kCheck(kString_SetLength(value, (kSize)valueLen));
    kCheck(kSerializer_ReadCharArray(channel->serializer, kString_Chars(value), valueLen));

    kCheck(GoControl_EndResponse(control, channel));

    return kOK;
}

GoFx(kStatus) GoControl_AddNewTool(GoControl control, const kChar* type, const kChar* name)
{
    kObj(GoControl, control);
    GoControlChannel* channel = obj->control;
    kText64 tempText;

    kCheckState(obj->isCompatible);
    kCheck(kStrCopy(tempText, kCountOf(tempText), type));

    kCheck(GoControl_BeginCommand(control, channel, GO_CONTROL_ADD_NEW_TOOL, GO_CONTROL_ADD_TOOL_TIMEOUT));

    kCheck(kSerializer_WriteCharArray(channel->serializer, tempText, kCountOf(tempText)));
    kCheck(kStrCopy(tempText, kCountOf(tempText), name));
    kCheck(kSerializer_WriteCharArray(channel->serializer, tempText, kCountOf(tempText)));

    kCheck(GoControl_SendAndReceive(control, channel));

    kCheck(GoControl_EndResponse(control, channel));

    return kOK;
}

GoFx(kStatus) GoControl_DeleteNewTool(GoControl control, const kChar* type, const kChar* name)
{
    kObj(GoControl, control);
    GoControlChannel* channel = obj->control;
    kText64 tempText;

    kCheckState(obj->isCompatible);
    kCheck(kStrCopy(tempText, kCountOf(tempText), type));

    kCheck(GoControl_BeginCommand(control, channel, GO_CONTROL_DELETE_NEW_TOOL, GO_CONTROL_ADD_TOOL_TIMEOUT));

    kCheck(kSerializer_WriteCharArray(channel->serializer, tempText, kCountOf(tempText)));
    kCheck(kStrCopy(tempText, kCountOf(tempText), name));
    kCheck(kSerializer_WriteCharArray(channel->serializer, tempText, kCountOf(tempText)));

    kCheck(GoControl_SendAndReceive(control, channel));

    kCheck(GoControl_EndResponse(control, channel));

    return kOK;
}

GoFx(kStatus) GoControl_SetDataTime(GoControl control, kDateTime dateTime)
{
    kObj(GoControl, control);
    GoControlChannel* channel = obj->control;

    kCheckState(obj->isCompatible);

    kCheck(GoControl_BeginCommand(control, channel, GO_CONTROL_SET_DATETIME, GO_CONTROL_SET_DATETIME_TIMEOUT));
    kCheck(kSerializer_Write64s(channel->serializer, dateTime));

    kCheck(GoControl_SendAndReceive(control, channel));
    kCheck(GoControl_EndResponse(control, channel));

    return kOK;
}
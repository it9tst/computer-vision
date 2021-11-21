/**
 * @file    GoSystem.c
 *
 * @internal
 * Copyright (C) 2016-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <GoSdk/GoSystem.h>
#include <kApi/Utils/kUtils.h>

kBeginClassEx(Go, GoSystem)
    kAddVMethod(GoSystem, kObject, VRelease)
kEndClassEx()

GoFx(kStatus) GoSystem_Construct(GoSystem* system, kAlloc allocator)
{
    return(GoSystem_ConstructHelper(system, kTRUE, allocator));
}

GoFx(kStatus) GoSystem_ConstructEx(GoSystem* system, kAlloc allocator)
{
    return(GoSystem_ConstructHelper(system, kFALSE, allocator));
}

GoFx(kStatus) GoSystem_ConstructHelper(GoSystem* system, kBool enableAutoDiscovery, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback(allocator);
    kStatus status;

    kCheck(kAlloc_GetObject(alloc, kTypeOf(GoSystem), system));

    if (!kSuccess(status = GoSystem_Init(*system, kTypeOf(GoSystem), enableAutoDiscovery, alloc)))
    {
        kAlloc_FreeRef(alloc, system);
    }

    return status;
}

GoFx(kStatus) GoSystem_Init(GoSystem system, kType type, kBool enableAutoDiscovery, kAlloc alloc)
{
    kObjR(GoSystem, system);
    kStatus status;

    kCheck(kObject_Init(system, type, alloc));
    kZero(obj->stateLock);
    kZero(obj->timer);
    obj->discovery = kNULL;
    kZero(obj->healthCheck);
    kZero(obj->allSensors);
    kZero(obj->onlineSensors);
    kZero(obj->tempList);
    obj->dataQuit = kFALSE;
    kZero(obj->dataQueue);
    kZero(obj->dataThread);
    obj->onData.function = kNULL;
    obj->onData.receiver = kNULL;
    obj->healthQuit = kFALSE;
    kZero(obj->healthQueue);
    kZero(obj->healthThread);
    obj->onHealth.function = kNULL;
    obj->onHealth.receiver = kNULL;
    kZero(obj->bankList);

    kTry
    {
        kTest(kLock_Construct(&obj->stateLock, alloc));
        kTest(kTimer_Construct(&obj->timer, alloc));

        kTest(GoDiscovery_Construct(&obj->discovery, enableAutoDiscovery, alloc));
        kTest(GoDiscovery_SetEnumPeriod(obj->discovery, GO_SYSTEM_DISCOVERY_PERIOD));
        kTest(GoDiscovery_SetEnumHandler(obj->discovery, GoSystem_OnDiscovery, system));
        // Modified as part of GOC-13351
        kTest(GoDiscovery_EnableCompatMode(obj->discovery, kFALSE));

        kTest(kPeriodic_Construct(&obj->healthCheck, alloc));

        kTest(kArrayList_Construct(&obj->allSensors, kTypeOf(GoSensor), 0, alloc));
        kTest(kArrayList_Construct(&obj->onlineSensors, kTypeOf(GoSensor), 0, alloc));
        kTest(kArrayList_Construct(&obj->bankList, kTypeOf(GoMultiplexBank), 0, alloc));
        kTest(kArrayList_Construct(&obj->tempList, kTypeOf(GoSensor), 0, alloc));

        kTest(kMsgQueue_Construct(&obj->dataQueue, kTypeOf(GoDataSet), alloc));
        kTest(kMsgQueue_SetMaxSize(obj->dataQueue, GO_SYSTEM_DEFAULT_DATA_CAPACITY));
        kTest(kMsgQueue_Construct(&obj->healthQueue, kTypeOf(GoDataSet), alloc));
        kTest(kMsgQueue_SetMaxSize(obj->healthQueue, GO_SYSTEM_DEFAULT_HEALTH_CAPACITY));

        kTest(kPeriodic_Start(obj->healthCheck, GO_SYSTEM_HEALTH_CHECK_PERIOD, GoSystem_OnHealthCheck, system));

        if (enableAutoDiscovery)
        {
            kTest(GoDiscovery_StartEnum(obj->discovery, kTRUE));
            kTest(GoSystem_Refresh(system));
        }
    }
    kCatch(&status)
    {
        GoSystem_VRelease(system);
        kEndCatch(status);
    }

    return kOK;
}

GoFx(kStatus) GoSystem_VRelease(GoSystem system)
{
    kObj(GoSystem, system);

    kCheck(GoSystem_SetDataHandler(system, kNULL, kNULL));
    kCheck(GoSystem_SetHealthHandler(system, kNULL, kNULL));

    if (!kIsNull(obj->healthCheck))
    {
        kCheck(kPeriodic_Stop(obj->healthCheck));
    }

    kCheck(kDestroyRef(&obj->healthCheck));
    kCheck(GoDiscovery_StopEnum(obj->discovery));
    kCheck(kDestroyRef(&obj->discovery));

    kCheck(kDestroyRef(&obj->onlineSensors)); //under normal circumstances, kDisposeRef would be called
                                                //on these allSensors and onlineSensors lists, but they share
                                                //GoSensor objects
    kCheck(kDisposeRef(&obj->allSensors));
    kCheck(kDestroyRef(&obj->tempList));
    kCheck(kDestroyRef(&obj->bankList));

    kCheck(kDisposeRef(&obj->dataQueue));
    kCheck(kDisposeRef(&obj->healthQueue));

    kCheck(kDestroyRef(&obj->timer));
    kCheck(kDestroyRef(&obj->stateLock));

    return kObject_VRelease(system);
}

GoFx(kStatus) GoSystem_OnDiscovery(GoSystem system, GoDiscovery discovery, kArrayList list)
{
    kObj(GoSystem, system);
    GoSensor newSensor = kNULL;
    kSize i;

    kAssert(kArrayList_ItemType(list) == kTypeOf(GoDiscoveryInfo));

    kLock_Enter(obj->stateLock);

    kTry
    {
        kSize currentCount = kArrayList_Count(obj->allSensors);
        kSize incomingCount = kArrayList_Count(list);

        for (i = 0; i < currentCount; ++i)
        {
            GoSensor sensor = kArrayList_AsT(obj->allSensors, i, GoSensor);
            kTest(GoSensor_UpdateDiscoveryInfo(sensor, kNULL));
        }

        for (i = 0; i < incomingCount; ++i)
        {
            GoDiscoveryInfo* discInfo = kArrayList_AtT(list, i, GoDiscoveryInfo);
            GoSensor sensor = kNULL;

            if (!kSuccess(GoSystem_FindSensorAll(system, discInfo->id, &sensor)))
            {
                kTest(GoSensor_Construct(&newSensor, system, discInfo, kObject_Alloc(system)));
                kTest(kArrayList_AddT(obj->allSensors, &newSensor));

                sensor = newSensor;
                newSensor = kNULL;
            }
            
            kTest(GoSensor_UpdateDiscoveryInfo(sensor, discInfo));
        }

        for (i = 0; i < currentCount; ++i)
        {
            GoSensor sensor = kArrayList_AsT(obj->allSensors, i, GoSensor);
            kTest(GoSensor_UpdateDiscoveryCycle(sensor));
        }
    }
    kFinally
    {
        kObject_Destroy(newSensor);
        kLock_Exit(obj->stateLock);
        kEndFinally();
    }

    return kOK;
}

GoFx(kStatus) GoSystem_OnHealthCheck(GoSystem system, kPeriodic timer)
{
    kObj(GoSystem, system);
    kSize i;

    kLock_Enter(obj->stateLock);

    kTry
    {
        for (i = 0; i < kArrayList_Count(obj->onlineSensors); ++i)
        {
            GoSensor sensor = kArrayList_AsT(obj->onlineSensors, i, GoSensor);
            kTest(GoSensor_CheckHealth(sensor));
        }
    }
    kFinally
    {
        kLock_Exit(obj->stateLock);
        kEndFinally();
    }

    return kOK;
}

GoFx(kBool) GoSystem_HasChanges(GoSystem system)
{
    kObj(GoSystem, system);
    kBool hasChanges = kFALSE;
    kSize i;

    kLock_Enter(obj->stateLock);
    {
        if (kArrayList_Count(obj->onlineSensors) != kArrayList_Count(obj->allSensors))
        {
            hasChanges = kTRUE;
        }
        else
        {
            for (i = 0; i < kArrayList_Count(obj->allSensors); ++i)
            {
                GoSensor sensor = kArrayList_AsT(obj->allSensors, i, GoSensor);
                GoState state = GoSensor_KnownState(sensor);       // avoids communication

                if (GoState_ShouldRefresh(state))
                {
                    hasChanges = kTRUE;
                    break;
                }
            }
        }
    }
    kLock_Exit(obj->stateLock);

    return hasChanges;
}

GoFx(kStatus) GoSystem_SensorRefreshAll(GoSystem system)
{
    kObj(GoSystem, system);
    kSize i = 0;

    for (i = 0; i < kArrayList_Count(obj->allSensors); ++i)
    {
        kCheck(GoSensor_Refresh(kArrayList_AsT(obj->allSensors, i, GoSensor)));
    }

    return kOK;
}

GoFx(kStatus) GoSystem_Refresh(GoSystem system)
{
    kObj(GoSystem, system);
    kArrayList infoList = kNULL;

    kTry
    {
        kTest(kArrayList_Construct(&infoList, kTypeOf(GoDiscoveryInfo), 10, kNULL));

        kTest(GoDiscovery_Enumerate(obj->discovery, infoList));
        kTest(GoSystem_OnDiscovery(system, obj->discovery, infoList));
    }
    kFinally
    {
        kCheck(kObject_Destroy(infoList));
        kEndFinally();
    }

    kCheck(GoSystem_SensorRefreshAll(system));

    kCheck(GoSystem_RefreshSensorList(system));

    return kOK;
}

GoFx(kStatus) GoSystem_RefreshSensorList(GoSystem system)
{
    kObj(GoSystem, system);
    kSize i = 0;

    kLock_Enter(obj->stateLock);

    kTry
    {
        kTest(kArrayList_Clear(obj->onlineSensors));

        while (i < kArrayList_Count(obj->allSensors))
        {
            GoSensor sensor = kArrayList_AsT(obj->allSensors, i, GoSensor);
            GoState state = GoSensor_State(sensor);

            if (state == GO_STATE_OFFLINE)
            {
                kTest(kArrayList_Discard(obj->allSensors, i));
                kTest(kObject_Destroy(sensor));
            }
            else
            {
                kTest(kArrayList_AddT(obj->onlineSensors, &sensor));
                i++;
            }
        }
    }
    kFinally
    {
        kLock_Exit(obj->stateLock);
        kEndFinally();
    }

    return kOK;
}

GoFx(kStatus) GoSystem_Connect(GoSystem system)
{
    kObj(GoSystem, system);
    kSize i;

    for (i = 0; i < kArrayList_Count(obj->onlineSensors); ++i)
    {
        kCheck(GoSensor_Connect(kArrayList_AsT(obj->onlineSensors, i, GoSensor)));
    }

    return kOK;
}

GoFx(kStatus) GoSystem_Disconnect(GoSystem system)
{
    kObj(GoSystem, system);
    kSize i;

    for (i = 0; i < kArrayList_Count(obj->onlineSensors); ++i)
    {
        kCheck(GoSensor_Disconnect(kArrayList_AsT(obj->onlineSensors, i, GoSensor)));
    }

    return kOK;
}

GoFx(kVersion) GoSystem_ProtocolVersion()
{
    return GoSdk_ProtocolVersion();
}

GoFx(kVersion) GoSystem_SdkVersion()
{
    return GoSdk_Version();
}

GoFx(kStatus) GoSystem_Start(GoSystem system)
{
    kObj(GoSystem, system);
    kSize i;

    kCheck(kArrayList_Allocate(obj->tempList, kTypeOf(GoSensor), 0));

    for (i = 0; i < kArrayList_Count(obj->onlineSensors); ++i)
    {
        GoSensor sensor = kArrayList_AsT(obj->onlineSensors, i, GoSensor);

        if (GoSensor_IsConnected(sensor)
            && GoSensor_State(sensor) == GO_STATE_READY)
        {
            kCheck(GoSensor_BeginStart(sensor));
            kCheck(kArrayList_AddT(obj->tempList, &sensor));
        }
    }

    for (i = 0; i < kArrayList_Count(obj->tempList); ++i)
    {
        GoSensor sensor = kArrayList_AsT(obj->tempList, i, GoSensor);

        kCheck(GoSensor_EndStart(sensor));
    }

    return kOK;
}

GoFx(kStatus) GoSystem_ScheduledStart(GoSystem system, k64s value)
{
    kObj(GoSystem, system);
    kSize i;

    kCheck(kArrayList_Allocate(obj->tempList, kTypeOf(GoSensor), 0));

    for (i = 0; i < kArrayList_Count(obj->onlineSensors); ++i)
    {
        GoSensor sensor = kArrayList_AsT(obj->onlineSensors, i, GoSensor);

        if (GoSensor_IsConnected(sensor)
            && GoSensor_State(sensor) == GO_STATE_READY)
        {
            kCheck(GoSensor_BeginScheduledStart(sensor, value));
            kCheck(kArrayList_AddT(obj->tempList, &sensor));
        }
    }

    for (i = 0; i < kArrayList_Count(obj->tempList); ++i)
    {
        GoSensor sensor = kArrayList_AsT(obj->tempList, i, GoSensor);

        kCheck(GoSensor_EndScheduledStart(sensor));
    }

    return kOK;
}

GoFx(kStatus) GoSystem_StartAlignment(GoSystem system)
{
    kObj(GoSystem, system);
    kSize i;

    kCheck(kArrayList_Allocate(obj->tempList, kTypeOf(GoSensor), 0));

    for (i = 0; i < kArrayList_Count(obj->onlineSensors); ++i)
    {
        GoSensor sensor = kArrayList_AsT(obj->onlineSensors, i, GoSensor);

        if (GoSensor_IsConnected(sensor)
            && GoSensor_State(sensor) == GO_STATE_READY)
        {
            kCheck(GoSensor_BeginAlign(sensor));
            kCheck(kArrayList_AddT(obj->tempList, &sensor));
        }
    }

    for (i = 0; i < kArrayList_Count(obj->tempList); ++i)
    {
        GoSensor sensor = kArrayList_AsT(obj->tempList, i, GoSensor);

        kCheck(GoSensor_EndStart(sensor));
    }

    return kOK;
}


GoFx(kStatus) GoSystem_StartExposureAutoSet(GoSystem system)
{
    kObj(GoSystem, system);
    kSize i;

    kCheck(kArrayList_Allocate(obj->tempList, kTypeOf(GoSensor), 0));

    for (i = 0; i < kArrayList_Count(obj->onlineSensors); ++i)
    {
        GoSensor sensor = kArrayList_AsT(obj->onlineSensors, i, GoSensor);

        if (GoSensor_IsConnected(sensor)
            && GoSensor_State(sensor) == GO_STATE_READY
            && GoSensor_Role(sensor) == GO_ROLE_MAIN)
        {
            kCheck(GoSensor_BeginExposureAutoSet(sensor, GO_ROLE_MAIN));

            kCheck(kArrayList_AddT(obj->tempList, &sensor));
        }
    }

    for (i = 0; i < kArrayList_Count(obj->tempList); ++i)
    {
        GoSensor sensor = kArrayList_AsT(obj->tempList, i, GoSensor);

        kCheck(GoSensor_EndStart(sensor));
    }

    // Exposure auto set all buddy sensors
    kCheck(kArrayList_Clear(obj->tempList));

    for (i = 0; i < kArrayList_Count(obj->onlineSensors); ++i)
    {
        GoSensor sensor = kArrayList_AsT(obj->onlineSensors, i, GoSensor);

        if (GoSensor_IsConnected(sensor)
            && GoSensor_State(sensor) == GO_STATE_READY
            && GoSensor_Role(sensor) == GO_ROLE_MAIN
            && GoSensor_HasBuddy(sensor))
        {
            kCheck(GoSensor_BeginExposureAutoSet(sensor, GO_ROLE_BUDDY));

            kCheck(kArrayList_AddT(obj->tempList, &sensor));
        }
    }

    for (i = 0; i < kArrayList_Count(obj->tempList); ++i)
    {
        GoSensor sensor = kArrayList_AsT(obj->tempList, i, GoSensor);

        kCheck(GoSensor_EndStart(sensor));
    }

    return kOK;
}

GoFx(kStatus) GoSystem_Stop(GoSystem system)
{
    kObj(GoSystem, system);
    kSize i;

    kCheck(kArrayList_Allocate(obj->tempList, kTypeOf(GoSensor), 0));

    for (i = 0; i < kArrayList_Count(obj->onlineSensors); ++i)
    {
        GoSensor sensor = kArrayList_AsT(obj->onlineSensors, i, GoSensor);

        if (GoSensor_IsConnected(sensor)
            && GoSensor_IsResponsive(sensor))
        {
            kCheck(GoSensor_BeginStop(sensor));
            kCheck(kArrayList_AddT(obj->tempList, &sensor));
        }
    }

    for (i = 0; i < kArrayList_Count(obj->tempList); ++i)
    {
        GoSensor sensor = kArrayList_AsT(obj->tempList, i, GoSensor);

        kCheck(GoSensor_EndStop(sensor));
    }

    return kOK;
}

GoFx(kStatus) GoSystem_Reset(GoSystem system, kBool wait)
{
    kObj(GoSystem, system);
    kSize i;

    kCheck(kArrayList_Allocate(obj->tempList, kTypeOf(GoSensor), 0));

    for (i = 0; i < kArrayList_Count(obj->onlineSensors); ++i)
    {
        GoSensor sensor = kArrayList_AsT(obj->onlineSensors, i, GoSensor);

        if (GoSensor_IsConnected(sensor) && (GoSensor_State(sensor) != GO_STATE_BUSY))
        {
            kCheck(GoSensor_Reset(sensor, kFALSE));
            kCheck(kArrayList_AddT(obj->tempList, &sensor));
        }
    }

    if (wait)
    {
        kCheck(kTimer_Start(obj->timer, GO_SYSTEM_RESET_TIMEOUT));

        for (i = 0; i < kArrayList_Count(obj->tempList); ++i)
        {
            GoSensor sensor = kArrayList_AsT(obj->tempList, i, GoSensor);

            kCheck(GoSensor_WaitForReconnect(sensor, kTimer_Remaining(obj->timer)));
        }
    }

    return kOK;
}

GoFx(kStatus) GoSystem_Cancel(GoSystem system)
{
    kObj(GoSystem, system);
    kSize i;

    for (i = 0; i < kArrayList_Count(obj->onlineSensors); ++i)
    {
        kCheck(GoSensor_Cancel(kArrayList_AsT(obj->onlineSensors, i, GoSensor)));
    }

    return kOK;
}

GoFx(kSize) GoSystem_SensorCount(GoSystem system)
{
    kObj(GoSystem, system);
    return kArrayList_Count(obj->onlineSensors);
}

GoFx(GoSensor) GoSystem_SensorAt(GoSystem system, kSize index)
{
    kObj(GoSystem, system);

    kAssert(index < GoSystem_SensorCount(system));

    return kArrayList_AsT(obj->onlineSensors, index, GoSensor);
}

GoFx(kStatus) GoSystem_FindSensorById(GoSystem system, k32u id, GoSensor* sensor)
{
    kObj(GoSystem, system);
    kSize i;

    for (i = 0; i < kArrayList_Count(obj->onlineSensors); ++i)
    {
        GoSensor sensorAt = kArrayList_AsT(obj->onlineSensors, i, GoSensor);

        if (GoSensor_Id(sensorAt) == id)
        {
            *sensor = sensorAt;
            return kOK;
        }
    }

    return kERROR_NOT_FOUND;
}

GoFx(kStatus) GoSystem_FindSensorByIpAddress(GoSystem system, const kIpAddress* address, GoSensor* sensor)
{
    kObj(GoSystem, system);
    kSize i;

    for (i = 0; i < kArrayList_Count(obj->onlineSensors); ++i)
    {
        GoSensor sensorAt = kArrayList_AsT(obj->onlineSensors, i, GoSensor);
        GoAddressInfo addressInfo;

        kCheck(GoSensor_Address(sensorAt, &addressInfo));

        if (kIpAddress_Equals(addressInfo.address, *address))
        {
            *sensor = sensorAt;
            return kOK;
        }
    }

    return kERROR_NOT_FOUND;
}

GoFx(kStatus) GoSystem_FindSensorAll(GoSystem system, kSize id, GoSensor* sensor)
{
    kObj(GoSystem, system);
    kSize i;

    for (i = 0; i < kArrayList_Count(obj->allSensors); ++i)
    {
        GoSensor sensorAt = kArrayList_AsT(obj->allSensors, i, GoSensor);

        if (GoSensor_Id(sensorAt) == id)
        {
            *sensor = sensorAt;
            return kOK;
        }
    }

    return kERROR_NOT_FOUND;
}

GoFx(kStatus) GoSystem_Timestamp(GoSystem system, k64u* time)
{
    kObj(GoSystem, system);
    kSize i;

    for (i = 0; i < kArrayList_Count(obj->onlineSensors); ++i)
    {
        GoSensor sensor = kArrayList_AsT(obj->onlineSensors, i, GoSensor);
        GoState state = GoSensor_State(sensor);

        if (GoState_IsNormal(state))
        {
            return GoSensor_Timestamp(sensor, time);
        }
    }

    return kERROR_NOT_FOUND;
}

GoFx(kStatus) GoSystem_Encoder(GoSystem system, k64s* encoder)
{
    kObj(GoSystem, system);
    kSize i;

    for (i = 0; i < kArrayList_Count(obj->onlineSensors); ++i)
    {
        GoSensor sensor = kArrayList_AsT(obj->onlineSensors, i, GoSensor);
        GoState state = GoSensor_State(sensor);

        if (GoState_IsNormal(state))
        {
            return GoSensor_Encoder(sensor, encoder);
        }
    }

    return kERROR_NOT_FOUND;
}

GoFx(kStatus) GoSystem_LockState(GoSystem system)
{
    kObj(GoSystem, system);
    return kLock_Enter(obj->stateLock);
}

GoFx(kStatus) GoSystem_UnlockState(GoSystem system)
{
    kObj(GoSystem, system);
    return kLock_Exit(obj->stateLock);
}

GoFx(kStatus) GoSystem_SetDataHandler(GoSystem system, GoDataFx function, kPointer receiver)
{
    kObj(GoSystem, system);

    obj->dataQuit = kTRUE;
    kCheck(kDestroyRef(&obj->dataThread));

    obj->dataQuit = kFALSE;
    obj->onData.function = function;
    obj->onData.receiver = receiver;

    if (obj->onData.function)
    {
        kCheck(kThread_Construct(&obj->dataThread, kObject_Alloc(system)));
        kCheck(kThread_Start(obj->dataThread, GoSystem_DataThreadEntry, system));
    }

    return kOK;
}

GoFx(kStatus) GoSystem_SetDataCapacity(GoSystem system, kSize capacity)
{
    kObj(GoSystem, system);

    kCheckArgs(0 < capacity && capacity <= kSIZE_MAX);
    kCheck(kMsgQueue_SetMaxSize(obj->dataQueue, capacity));

    return kOK;
}

GoFx(kSize) GoSystem_DataCapacity(GoSystem system)
{
    kObj(GoSystem, system);
    return kMsgQueue_MaxSize(obj->dataQueue);
}

GoFx(kStatus) GoSystem_EnableData(GoSystem system, kBool enable)
{
    kObj(GoSystem, system);
    kSize i;

    for (i = 0; i < kArrayList_Count(obj->onlineSensors); ++i)
    {
        GoSensor sensor = kArrayList_AsT(obj->onlineSensors, i, GoSensor);
        GoState state = GoSensor_State(sensor);

        // Skip sensor if it is not in correct state and go on to the
        // next sensor in the list. Don't bail out of this loop
        // because of this.
        if (GoState_IsNormal(state))
        {
            kCheck(GoSensor_EnableData(sensor, enable));
        }
    }

    return kOK;
}

GoFx(kStatus) GoSystem_ClearData(GoSystem system)
{
    kObj(GoSystem, system);
    kObject data = kNULL;
    kSize i;

    kCheck(kArrayList_Allocate(obj->tempList, kTypeOf(GoSensor), 0));

    for (i = 0; i < kArrayList_Count(obj->onlineSensors); ++i)
    {
        GoSensor sensor = kArrayList_AsT(obj->onlineSensors, i, GoSensor);

        if (GoSensor_DataEnabled(sensor))
        {
            kCheck(GoSensor_EnableData(sensor, kFALSE));
            kCheck(kArrayList_AddT(obj->tempList, &sensor));
        }
    }

    while (kSuccess(kMsgQueue_RemoveT(obj->dataQueue, &data, 0)))
    {
        kCheck(kObject_Dispose(data));
    }

    for (i = 0; i < kArrayList_Count(obj->tempList); ++i)
    {
        GoSensor sensor = kArrayList_AsT(obj->tempList, i, GoSensor);
        kCheck(GoSensor_EnableData(sensor, kTRUE));
    }

    return kOK;
}

GoFx(kStatus) GoSystem_ReceiveData(GoSystem system, GoDataSet* data, k64u timeout)
{
    kObj(GoSystem, system);

    return kMsgQueue_RemoveT(obj->dataQueue, data, timeout);
}

GoFx(kStatus) GoSystem_DataThreadEntry(GoSystem system)
{
    kObj(GoSystem, system);
    GoDataSet data = kNULL;
    kStatus status;

    while (!obj->dataQuit)
    {
        if (kSuccess(status = kMsgQueue_RemoveT(obj->dataQueue, &data, GO_SYSTEM_QUIT_QUERY_INTERVAL)))
        {
            kCheck(obj->onData.function(obj->onData.receiver, system, data));
        }
        else if (status != kERROR_TIMEOUT)
        {
            kCheck(status);
        }
    }

    return kOK;
}

GoFx(kStatus) GoSystem_OnData(GoSystem system, GoSensor sensor, GoDataSet data)
{
    kObj(GoSystem, system);

    kCheck(kMsgQueue_AddT(obj->dataQueue, &data));

    return kOK;
}

GoFx(kStatus) GoSystem_SetHealthHandler(GoSystem system, GoDataFx function, kPointer receiver)
{
    kObj(GoSystem, system);

    obj->healthQuit = kTRUE;
    kCheck(kDestroyRef(&obj->healthThread));

    obj->healthQuit = kFALSE;
    obj->onHealth.function = function;
    obj->onHealth.receiver = receiver;

    if (obj->onHealth.function)
    {
        kCheck(kThread_Construct(&obj->healthThread, kObject_Alloc(system)));
        kCheck(kThread_Start(obj->healthThread, GoSystem_HealthThreadEntry, system));
    }

    return kOK;
}

GoFx(kStatus) GoSystem_SetHealthCapacity(GoSystem system, kSize capacity)
{
    kObj(GoSystem, system);

    kCheck(kMsgQueue_SetMaxSize(obj->healthQueue, capacity));

    return kOK;
}

GoFx(kSize) GoSystem_HealthCapacity(GoSystem system)
{
    kObj(GoSystem, system);
    return kMsgQueue_MaxSize(obj->healthQueue);
}

GoFx(kStatus) GoSystem_ReceiveHealth(GoSystem system, GoDataSet* health, k64u timeout)
{
    kObj(GoSystem, system);

    return kMsgQueue_RemoveT(obj->healthQueue, health, timeout);
}

GoFx(kStatus) GoSystem_ClearHealth(GoSystem system)
{
    kObj(GoSystem, system);
    kObject health = kNULL;

    while (kSuccess(kMsgQueue_RemoveT(obj->healthQueue, &health, 0)))
    {
        kCheck(kObject_Dispose(health));
    }

    return kOK;
}

GoFx(kStatus) GoSystem_HealthThreadEntry(GoSystem system)
{
    kObj(GoSystem, system);
    GoDataSet health = kNULL;
    kStatus status;

    while (!obj->healthQuit)
    {
        if (kSuccess(status = kMsgQueue_RemoveT(obj->healthQueue, &health, GO_SYSTEM_QUIT_QUERY_INTERVAL)))
        {
            kCheck(obj->onHealth.function(obj->onHealth.receiver, system, health));
        }
        else if (status != kERROR_TIMEOUT)
        {
            kCheck(status);
        }
    }

    return kOK;
}

GoFx(kStatus) GoSystem_OnHealth(GoSystem system, GoSensor sensor, GoDataSet health)
{
    kObj(GoSystem, system);

    kCheck(kMsgQueue_AddT(obj->healthQueue, &health));

    return kOK;
}

GoFx(kSize) GoSystem_MultiplexBankCount(GoSystem system)
{
    kObj(GoSystem, system);

    return kArrayList_Count(obj->bankList);
}

GoFx(GoMultiplexBank) GoSystem_MultiplexBankAt(GoSystem system, kSize index)
{
    kObj(GoSystem, system);

    kAssert(index < kArrayList_Count(obj->bankList));

    return kArrayList_AsT(obj->bankList, index, GoMultiplexBank);
}

GoFx(kStatus) GoSystem_AddMultiplexBank(GoSystem system, GoMultiplexBank* bank)
{
    kObj(GoSystem, system);
    GoMultiplexBank newBank = kNULL;
    kStatus exception = kOK;
    k32u nextId = 0;
    k32u maxId = 0;
    k32u currentItemId;
    kBool unusedIdFound = kFALSE;
    kSize i;

    kTry
    {
        for (i = 0; i < kArrayList_Count(obj->bankList); i++)
        {
            currentItemId = GoMultiplexBank_Id(kArrayList_AsT(obj->bankList, i, GoMultiplexBank));

            if (currentItemId > maxId)
            {
                maxId = currentItemId;
            }
        }

        nextId = maxId + 1;

        kTest(GoMultiplexBank_Construct(&newBank, nextId, kObject_Alloc(system)));
        kTest(kArrayList_AddT(obj->bankList, &newBank));
    }
    kCatch(&exception)
    {
        kDestroyRef(&newBank);
        kEndCatch(exception);
    }

    if (!kIsNull(bank))
    {
        *bank = newBank;
    }

    return kOK;
}

GoFx(kStatus) GoSystem_RemoveMultiplexBank(GoSystem system, kSize index)
{
    kObj(GoSystem, system);
    GoMultiplexBank bankToDestroy = kNULL;

    kCheckArgs(index < kArrayList_Count(obj->bankList));

    kCheck(kArrayList_RemoveT(obj->bankList, index, &bankToDestroy));
    kDestroyRef(&bankToDestroy);

    return kOK;
}

GoFx(kStatus) GoSystem_ClearMultiplexBanks(GoSystem system)
{
    kObj(GoSystem, system);
    GoMultiplexBank bank = kNULL;

    while (kArrayList_Count(obj->bankList) != 0)
    {
        kCheck(kArrayList_RemoveT(obj->bankList, 0, &bank));
        kCheck(kDestroyRef(&bank)); //sensor objects within the bank are not destroyed
    }

    return kOK;
}

GoFx(kStatus) GoSystem_UpdateMultiplexDelay(GoSystem system)
{
    kObj(GoSystem, system);
    kSize i, j;
    k64f exposureDuration = 0.0;
    kBool tempConnection = kFALSE;

    for (i = 0; i < kArrayList_Count(obj->bankList); i++)
    {
        GoMultiplexBank currentBank = kArrayList_AsT(obj->bankList, i, GoMultiplexBank);
        k64f currentExposureDuration;

        for (j = 0; j < GoMultiplexBank_SensorCount(currentBank); j++)
        {
            GoSensor sensor = GoMultiplexBank_SensorAt(currentBank, j);
            GoLayout layout = kNULL;

            if (!GoSensor_IsConnected(sensor))
            {
                kCheck(GoSensor_Connect(sensor));
                tempConnection = kTRUE;
            }

            layout = GoSetup_Layout(GoSensor_Setup(sensor));

            kCheck(GoLayout_EnableMultiplexSingle(layout, kTRUE));
            kCheck(GoLayout_SetMultiplexSingleDelay(layout, exposureDuration));

            if (tempConnection)
            {
                kCheck(GoSensor_Disconnect(sensor));
                tempConnection = kFALSE;
            }
        }

        currentExposureDuration = GoSystem_MaxBankExposureDuration(system, currentBank);
        exposureDuration += currentExposureDuration;
    }

    return kOK;
}

GoFx(k64f) GoSystem_MaxMinimumMultiplexPeriod(GoSystem system)
{
    kObj(GoSystem, system);
    kSize i, j;
    kBool tempConnection = kFALSE;
    k64f maxPeriod = 0.0;

    for (i = 0; i < kArrayList_Count(obj->bankList); i++)
    {
        GoMultiplexBank currentBank = kArrayList_AsT(obj->bankList, i, GoMultiplexBank);
        k64f periodMin;

        for (j = 0; j < GoMultiplexBank_SensorCount(currentBank); j++)
        {
            GoSensor sensor = GoMultiplexBank_SensorAt(currentBank, j);
            GoLayout layout = kNULL;

            if (!GoSensor_IsConnected(sensor))
            {
                kCheck(GoSensor_Connect(sensor));
                tempConnection = kTRUE;
            }

            layout = GoSetup_Layout(GoSensor_Setup(sensor));

            periodMin = GoLayout_MultiplexSinglePeriodMin(layout);

            if (tempConnection)
            {
                kCheck(GoSensor_Disconnect(sensor));
                tempConnection = kFALSE;
            }

            if (periodMin > maxPeriod)
            {
                maxPeriod = periodMin;
            }
        }
    }

    return maxPeriod;
}

GoFx(kStatus) GoSystem_UpdateMultiplexPeriod(GoSystem system, k64f period)
{
    kObj(GoSystem, system);
    kSize i, j;
    k64f exposureDuration = 0.0;
    kBool tempConnection = kFALSE;
    k64f maxPeriod = GoSystem_MaxMinimumMultiplexPeriod(system);

    if (period == 0.0)
    {
        period = maxPeriod;
    }
    else
    {
        if (period < maxPeriod)
        {
            return kERROR_PARAMETER;
        }
    }

    for (i = 0; i < kArrayList_Count(obj->bankList); i++)
    {
        GoMultiplexBank currentBank = kArrayList_AsT(obj->bankList, i, GoMultiplexBank);
        k64f periodMin;

        for (j = 0; j < GoMultiplexBank_SensorCount(currentBank); j++)
        {
            GoSensor sensor = GoMultiplexBank_SensorAt(currentBank, j);
            GoLayout layout = kNULL;

            if (!GoSensor_IsConnected(sensor))
            {
                kCheck(GoSensor_Connect(sensor));
                tempConnection = kTRUE;
            }

            layout = GoSetup_Layout(GoSensor_Setup(sensor));

            periodMin = GoLayout_SetMultiplexSinglePeriod(layout, period);

            if (tempConnection)
            {
                kCheck(GoSensor_Disconnect(sensor));
                tempConnection = kFALSE;
            }

            if (periodMin > maxPeriod)
            {
                maxPeriod = periodMin;
            }
        }
    }

    return kOK;
}

GoFx(k64f) GoSystem_MaxBankExposureDuration(GoSystem system, GoMultiplexBank bank)
{
    kObj(GoMultiplexBank, bank);
    k64f maxExposureDuration = 0.0;
    k64f exposureDurationToCompare = 0.0;
    kBool tempConnection = kFALSE;
    kSize i;

    for (i = 0; i < GoMultiplexBank_SensorCount(bank); i++)
    {
        GoSensor sensor = GoMultiplexBank_SensorAt(bank, i);
        GoSetup setup = kNULL;

        if (!GoSensor_IsConnected(sensor))
        {
            kCheck(GoSensor_Connect(sensor));
            tempConnection = kTRUE;
        }

        setup = GoSensor_Setup(sensor);

        exposureDurationToCompare = GoLayout_MultiplexSingleExposureDuration(GoSetup_Layout(setup));

        if (maxExposureDuration < exposureDurationToCompare)
        {
            maxExposureDuration = exposureDurationToCompare;
        }

        if (tempConnection)
        {
            kCheck(GoSensor_Disconnect(sensor));
            tempConnection = kFALSE;
        }
    }

    return maxExposureDuration;
}

GoFx(kStatus) GoSystem_UpdateAllMultiplexParameters(GoSystem system, k64f period)
{
    kCheck(GoSystem_UpdateMultiplexDelay(system));
    kCheck(GoSystem_UpdateMultiplexPeriod(system, period));

    return kOK;
}

GoFx(kStatus) GoSystem_GetExtendedDiscoveryInfo(GoSystem system, k32u deviceId, GoDiscoveryExtInfo* info, kAlloc allocator)
{
    kObj(GoSystem, system);

    kCheck(GoDiscovery_GetExtendedInfo(obj->discovery, deviceId, info, allocator));

    return kOK;
}

GoFx(kStatus) GoSystem_AddSensor(GoSystem system,const GoAddressInfo* addressInfo,const GoPortInfo* portInfo, GoSensor* sensor)
{
    kStatus status;
    GoSensorClass* obj;

    kTry
    {
        kTest(GoSensor_Construct(sensor, system, kNULL, kObject_Alloc(system)));
        
        obj = *sensor;

        obj->address = *addressInfo;
        obj->dataPort = portInfo->dataPort;
        obj->healthPort = portInfo->healthPort;

        kTest(GoControl_SetRemoteAddress(obj->control, addressInfo->address));
        kTest(GoControl_SetControlPort(obj->control, portInfo->controlPort));
        kTest(GoControl_SetUpgradePort(obj->control, portInfo->upgradePort));
    }
    kCatch(&status)
    {
        GoSensor_VRelease(sensor);
        kEndCatch(status);
    }  

    return kOK;
}


GoFx(kStatus) GoSystem_SetOneDiscoveryInterface(GoSystem system, kIpAddress* address, kBool enable)
{
    kObj(GoSystem, system);

    return(GoDiscovery_SetOneInterface(obj->discovery, address, enable));
}

GoFx(kStatus) GoSystem_SetAllDiscoveryInterface(GoSystem system, kBool enable)
{
    kObj(GoSystem, system);

    return(GoDiscovery_SetAllInterface(obj->discovery, enable));
}

GoFx(kStatus) GoSystem_EnableDiscoveryCompatibility(GoSystem system, kBool enable)
{
    kObj(GoSystem, system);

    return GoDiscovery_EnableCompatMode(obj->discovery, enable);
}

GoFx(kBool) GoSystem_DiscoveryCompatibilityEnabled(GoSystem system)
{
    kObj(GoSystem, system);

    return GoDiscovery_CompatModeEnabled(obj->discovery);
}

GoFx(kStatus) GoSystem_StartDiscovery(GoSystem system)
{
    kObj(GoSystem, system);

    kCheck(GoDiscovery_StartEnum(obj->discovery, kTRUE));

    kCheck(GoSystem_SensorRefreshAll(system));

    kCheck(GoSystem_RefreshSensorList(system));

    return kOK;
}

GoFx(kStatus) GoSystem_EnablePtp(GoSystem system)
{
    return kOK;
}

/**
 * @file    GoAcceleratorMgr.c
 *
 * This file defines the GoAcceleratorManager class.
 * This class is responsible for accelerating one or more sensors,
 * and monitoring the accelerated sensors.
 *
 * This class must be used in conjunction with the program
 * GoXProcess which handles the details of the acceleration process.
 *
 * The manager manager can be configured after it has been
 * constructed, but before starting the the manager. Once started, the
 * manager manager cannot be configured again, even after stopping
 * the manager manager.
 *
 * To change the manager manager, the old manager must be released
 * and a new acceleration manager must be constructed.
 *
 * <sample code showing usage of this class.>
 *
 * @internal
 * Copyright (C) 2018 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <GoSdk/GoAcceleratorMgr.h>
#include <GoSdk/GoSensor.h>
#include <kApi/Io/kPath.h>
#include <kApi/Threads/kMsgQueue.h>


// This definition needed only because of SDKNET wrapper definitions.
kBeginValueEx(Go, GoAcceleratorMgrAccelUpdate)
kEndValueEx()
kBeginValueEx(Go, GoAcceleratorMgrSensorParam)
kEndValueEx()
kBeginValueEx(Go, GoAcceleratorMgrSensorBackup)
kEndValueEx()
kBeginEnumEx(Go, GoAcceleratorMgrAccelEvents)
kEndEnumEx()

kBeginValueEx(Go, GoAcceleratorMgrSensorEntry)
kEndValueEx()

kBeginValueEx(Go, GoAcceleratorMgrWorkerMsg)
kEndValueEx()

kBeginValueEx(Go, GoAcceleratorMgrSensorInfo)
kEndValueEx()

kBeginClassEx(Go, GoAcceleratorMgr)
    kAddVMethod(GoAcceleratorMgr, kObject, VRelease)
kEndClassEx()

GoFx(kStatus) GoAcceleratorMgr_Construct(GoAcceleratorMgr* manager, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback(allocator);
    kStatus status;

    kCheck(kAlloc_GetObject(alloc, kTypeOf(GoAcceleratorMgr), manager));

    if (!kSuccess(status = GoAcceleratorMgr_Init(*manager, kTypeOf(GoAcceleratorMgr), alloc)))
    {
        kAlloc_FreeRef(alloc, manager);
    }

    return status;
}

GoFx(kStatus) GoAcceleratorMgr_Init(GoAcceleratorMgr manager, kType type, kAlloc alloc)
{
    kObjR(GoAcceleratorMgr, manager);
    kStatus exception;

    kCheck(kObject_Init(manager, type, alloc));
    obj->system = kNULL;
    kZero(obj->options);
    kZero(obj->threads);
    kZero(obj->updateCallback);
    kZero(obj->sensors);
    obj->portAlloc = kNULL;

    kTry
    {
        obj->isRunning              = kFALSE;
        obj->options.autoRecoverSensor = kTRUE;

        kTest(GoAccelSensorPortAlloc_Construct(&obj->portAlloc, alloc));

        kTest(kLock_Construct(&obj->updateCallback.lock, alloc));

        // List of sensors selected for acceleration.
        kTest(GoAcceleratorMgr_InitSensorBlock(manager, alloc));

        kTest(GoAcceleratorMgr_InitThreadBlock(manager, alloc));
    }
    kCatch(&exception)
    {
        GoAcceleratorMgr_ReleaseThreadBlock(manager);
        GoAcceleratorMgr_ReleaseSensorBlock(manager);

        kDestroyRef(&obj->updateCallback.lock);
        kDestroyRef(&obj->portAlloc);

        kEndCatch(exception);
    }

    return kOK;
}

GoFx(kStatus) GoAcceleratorMgr_VRelease(GoAcceleratorMgr manager)
{
    kObj(GoAcceleratorMgr, manager);

    // Stop the threads before destructing them.
    kCheck(GoAcceleratorMgr_Stop(manager));

    kCheck(GoAcceleratorMgr_ReleaseThreadBlock(manager));
    kCheck(GoAcceleratorMgr_ReleaseSensorBlock(manager));

    kCheck(kDestroyRef(&obj->updateCallback.lock));
    kCheck(kDestroyRef(&obj->portAlloc));

    return kObject_VRelease(manager);
}

GoFx(kStatus) GoAcceleratorMgr_SetSystem(GoAcceleratorMgr manager, GoSystem system)
{
    kObjR(GoAcceleratorMgr, manager);

    obj->system = system;

    return kOK;
}

// This function converts a command id to its corresponding name.
// It is useful for log messages.
GoFx(const kChar*) GoAcceleratorMgr_GetCommandName(GoAcceleratorMgrWorkerCmd cmd)
{
    const kChar* name;

    switch (cmd)
    {
    case ACCELERATOR_MGR_NO_CMD:
        name = "No Cmd";
        break;
    case ACCELERATOR_MGR_START:
        name = "Start";
        break;
    case ACCELERATOR_MGR_STOP:
        name = "Stop";
        break;
    case ACCELERATOR_MGR_RESTORE_START:
        name = "Restore";
        break;
    case ACCELERATOR_MGR_RESTART:
        name = "Restart";
        break;
    case ACCELERATOR_MGR_EVENT:
        name = "Event";
        break;
    default:
        name = "Unknown Cmd";
        break;
    }

    return name;
}

// This function returns the names of accelerator event. It is useful
// for log messages.
GoFx(const kChar*) GoAcceleratorMgr_GetEventName(GoAcceleratorEventType event)
{
    const kChar* name;

    switch (event)
    {
    case GO_ACCELERATOR_EVENT_NONE:
        name = "None";
        break;
    case GO_ACCELERATOR_EVENT_TERMINATE:
        name = "Terminate";
        break;
    case GO_ACCELERATOR_EVENT_ACCELERATING:
        name = "Accelerating";
        break;
    case GO_ACCELERATOR_EVENT_SUCCESS:
        name = "Success";
        break;
    case GO_ACCELERATOR_EVENT_DECELERATING:
        name = "Decelerating";
        break;
    case GO_ACCELERATOR_EVENT_STOPPED:
        name = "Stopped";
        break;
    case GO_ACCELERATOR_EVENT_DISCONNECT:
        name = "Disconnect";
        break;
    case GO_ACCELERATOR_EVENT_DECELERATED:
        name = "Decelerated";
        break;
    case GO_ACCELERATOR_EVENT_PROCESS_STOPPED:
        name = "Stopped";
        break;
    default:
        name = "Unknown Event";
        break;
    }

    return name;
}

GoFx(kStatus) GoAcceleratorMgr_Start(GoAcceleratorMgr manager)
{
    kObj(GoAcceleratorMgr, manager);

    // Don't allow a start until the system object has been assigned.
    kCheckState(!kIsNull(obj->system));

    if (!obj->isRunning)
    {    // Start the threads.
        kCheck(kThread_Start(obj->threads.workerThread, GoAcceleratorMgr_WorkerThread, manager));
        kCheck(kThread_Start(obj->threads.eventThread, GoAcceleratorMgr_EventThread, manager));
        kCheck(kThread_Start(obj->threads.callbackThread, GoAcceleratorMgr_CallbackThread, manager));

        obj->isRunning = kTRUE;
    }

    return kOK;
}

// This function sends a dummy event to the event thread to unblock the
// event thread and make the thread exit.
GoFx(kStatus) GoAcceleratorMgr_StopEventThread(GoAcceleratorMgr manager)
{
    kObj(GoAcceleratorMgr, manager);
    GoAcceleratorEventMsg eventMsg;
    kIpEndPoint myAddr;

    eventMsg.sensorId = 0;  // dummy sensor id which is ignored.
    eventMsg.event = GO_ACCELERATOR_EVENT_NONE;

    // Send event to myself.
    myAddr.address  = kIpAddress_LoopbackV4();
    myAddr.port     = GO_ACCELERATOR_PORT_EVENT;

    kCheck(GoAcceleratorMgr_SendEvent(manager, &myAddr, &eventMsg));

    return kOK;
}

GoFx(kStatus) GoAcceleratorMgr_Stop(GoAcceleratorMgr manager)
{
    kObj(GoAcceleratorMgr, manager);
    GoAcceleratorMgrWorkerMsg msg = { ACCELERATOR_MGR_NO_CMD };

    if (obj->isRunning)
    {
        kCheck(kMsgQueue_AddT(obj->threads.callbackQueue, &msg));
        kCheck(kThread_Join(obj->threads.callbackThread, GO_ACCELERATOR_MGR_TIMEOUT_THREAD_JOIN, kNULL));

        kCheck(GoAcceleratorMgr_StopEventThread(manager));
        kCheck(kThread_Join(obj->threads.eventThread, GO_ACCELERATOR_MGR_TIMEOUT_THREAD_JOIN, kNULL));

        kCheck(kMsgQueue_AddT(obj->threads.workerQueue, &msg));
        kCheck(kThread_Join(obj->threads.workerThread, GO_ACCELERATOR_MGR_TIMEOUT_THREAD_JOIN, kNULL));

        // TODO: purge all allocated ports.

        obj->isRunning = kFALSE;

        // TODO: Need to handle case where another thread sees the running
        // flag is true, before this function sets it to false, and
        // enqueues msg after this function has cleared the queue?
        while (kMsgQueue_Remove(obj->threads.workerQueue, &msg, 0) == kOK)
        {
            // No action. Just discard the message.
        }
    }

    return kOK;
}

// This function is used for new acceleration request generated by the external
// client.
GoFx(kStatus) GoAcceleratorMgr_Accelerate(GoAcceleratorMgr manager, k32u sensorId, GoAcceleratorMgrSensorParam* param)
{
    kObj(GoAcceleratorMgr, manager);
    GoAcceleratorMgrWorkerMsg msg;

    kCheckState(obj->isRunning);

    kCheckArgs(sensorId != 0);
    kCheckArgs(!kIsNull(param));

    // Validate the current sensor state.
    kCheck(GoAcceleratorMgr_ValidateSensorState(manager, sensorId));

    // Validate client provided non-port parameters.
    kCheckArgs(GoAcceleratorMgr_ValidateSensorParam(manager, param));

    // Check if sensor is in the process of being accelerated by this host,
    // but not yet fully accelerated.
    // This check is not definitive because a duplicate request can be made
    // before the worker thread has had a chance to run to process the first request.
    // This check is just a quick check that should work most of the time.
    // The definitive check is to do it within the worker thread because that
    // thread serializes all acceleration requests.
    //
    // TODO: return success if it is a duplicate request?
    kCheckArgs(GoAcceleratorMgr_IsSensorAcceleratedByLocalHost(manager, sensorId) == kFALSE);

    // Allocate ports for the accelerated sensor to use on the accelerator
    // host.
    // The ports could either be the port numbers provided by the client, or
    // allocated for the client.
    kCheck(GoAccelSensorPortAlloc_AllocatePorts(obj->portAlloc, sensorId, &param->ports));

    // Queue up the validated request for the worker thread to do the work.
    msg.command = ACCELERATOR_MGR_START;
    msg.sensorId = sensorId;
    msg.data.param = *param;
    kCheck(kMsgQueue_AddT(obj->threads.workerQueue, &msg));

    return kOK;
}

// This function is used during an application restore accelerated sensors
// retrieved from persisted configuration.
// The configuration contains sensors that were configured by external
// client to accelerate prior to the application restarting.
// Some sanity checks needed for the original requests from the external client
// are bypassed, on the assumption that if the sensor is in the persisted
// configuration, then the original request must have passed the
// sanity checks.
// If external conditions have changed such that the sanity tests now fail,
// then this function will allow the failure to happen rather than reject the
// acceleration request.
GoFx(kStatus) GoAcceleratorMgr_RestoreAccelerate(GoAcceleratorMgr manager, k32u sensorId, GoAcceleratorMgrSensorParam* param)
{
    kObj(GoAcceleratorMgr, manager);
    GoAcceleratorMgrWorkerMsg msg;

    kCheckState(obj->isRunning);

    kCheckArgs(sensorId != 0);
    kCheckArgs(!kIsNull(param));

    // Need to reserve these ports in the port allocator to prevent any other
    // sensor from using them.
    // Should never fail unless the ports are corrupted or somehow changed
    // from their original valid values.
    kCheck(GoAccelSensorPortAlloc_AllocateRestoredPorts(obj->portAlloc, sensorId, &param->ports));

    // Restored sensor acceleration request is treated as a START request, rather
    // than a RESTART request because this is the first time, since boot up,
    // the acceleration is attempted (ie. it is not a retry operation caused
    // by an acceleration failure).
    msg.command = ACCELERATOR_MGR_RESTORE_START;
    msg.sensorId = sensorId;
    msg.data.param = *param;
    kCheck(kMsgQueue_AddT(obj->threads.workerQueue, &msg));

    return kOK;
}

GoFx(kStatus) GoAcceleratorMgr_Decelerate(GoAcceleratorMgr manager, k32u sensorId)
{
    kObj(GoAcceleratorMgr, manager);
    GoAcceleratorMgrWorkerMsg msg;

    kCheckState(obj->isRunning);

    kCheckArgs(sensorId != 0);
    kCheckArgs(GoAcceleratorMgr_IsSensorAcceleratedByLocalHost(manager, sensorId) == kTRUE);

    // Queue up the validated request for the worker thread to do the work.
    msg.command = ACCELERATOR_MGR_STOP;
    msg.sensorId = sensorId;
    kCheck(kMsgQueue_AddT(obj->threads.workerQueue, &msg));

    return kOK;
}

// Used for unit testing.
GoFx(kStatus) GoAcceleratorMgr_AccelerationStatus(GoAcceleratorMgr manager, k32u sensorId, GoSensorAccelStatus* status)
{
    kObj(GoAcceleratorMgr, manager);
    kSize sensorCount;
    kSize i;
    GoAcceleratorMgrSensorEntry* sensorInfo;
    kStatus retStatus = kERROR_NOT_FOUND;

    kCheckState(obj->isRunning);

    kCheckArgs(sensorId != 0);
    kCheckArgs(!kIsNull(status));

    // Must always have exclusive access to the list of attached sensors.
    kCheck(kLock_Enter(obj->sensors.lock));
    kTry
    {
        sensorCount = kArrayList_Count(obj->sensors.attachedSensors);

        for (i = 0; i < sensorCount; i++)
        {
            sensorInfo = kArrayList_AtT(obj->sensors.attachedSensors, i, GoAcceleratorMgrSensorEntry);
            if (sensorInfo->sensorId == sensorId)
            {
                // Found sensor in the accelerated sensor list.
                *status = sensorInfo->status;
                retStatus = kOK;
                break;
            }
        }
    }
    kFinally
    {
        kLock_Exit(obj->sensors.lock);

        kEndFinally();
    }

    return retStatus;
}


GoFx(kStatus) GoAcceleratorMgr_ListSensors(GoAcceleratorMgr manager, kArrayList sensorList)
{
    kObj(GoAcceleratorMgr, manager);
    kSize sensorCount;
    kSize i;
    GoAcceleratorMgrSensorEntry* srcEntry;
    GoAcceleratorMgrSensorInfo outputEntry;

    kCheckState(obj->isRunning);

    kCheckArgs(!kIsNull(sensorList));
    kCheckArgs(kArrayList_ItemType(sensorList) == kTypeOf(GoAcceleratorMgrSensorInfo));

    // Must always have exclusive access to the list of attached sensors.
    kCheck(kLock_Enter(obj->sensors.lock));
    kTry
    {
        sensorCount = kArrayList_Count(obj->sensors.attachedSensors);

        for (i = 0; i < sensorCount; i++)
        {
            srcEntry = kArrayList_AtT(obj->sensors.attachedSensors, i, GoAcceleratorMgrSensorEntry);

            outputEntry.sensorId                 = srcEntry->sensorId;
            outputEntry.status                   = srcEntry->status;
            outputEntry.param.ports              = srcEntry->portInfo;
            outputEntry.param.platformIpAddress  = srcEntry->platformIpAddress;

            outputEntry.backup.ports             = srcEntry->backup.ports;
            outputEntry.backup.sensorAddressInfo = srcEntry->backup.sensorAddressInfo;

            // Provide more detail to the client, where possible, on why
            // acceleration stopped.
            if (outputEntry.status == GO_SENSOR_ACCEL_STATUS_STOPPED)
            {
                // Get more details on why acceleration stopped to send to client.
                outputEntry.status = GoAcceleratorMgr_RefineAccelStoppedStatus(manager, outputEntry.sensorId);
            }

            kTest(kArrayList_AddT(sensorList, &outputEntry));
        }
    }
    kFinally
    {
        kLock_Exit(obj->sensors.lock);

        kEndFinally();
    }

    return kOK;
}

GoFx(kStatus) GoAcceleratorMgr_SetAccelUpdateHandler(GoAcceleratorMgr manager, kCallbackFx function, kPointer context)
{
    kObj(GoAcceleratorMgr, manager);

    kCheck(kLock_Enter(obj->updateCallback.lock));

    obj->updateCallback.onAccelUpdate.function = function;
    obj->updateCallback.onAccelUpdate.receiver = context;

    kCheck(kLock_Exit(obj->updateCallback.lock));

    return kOK;
}

// This function allows client to change the port range used for accelerating
// a sensor. Must check that no sensors are currently accelerated before
// allowing client to make the change.
GoFx(kStatus) GoAcceleratorMgr_SetPortRange(GoAcceleratorMgr manager, k16u startPort, k16u endPort)
{
    kObj(GoAcceleratorMgr, manager);

    // TODO: stop accelerator manager first?
    // For now, don't allow port range configuration changes while accelerator
    // manager is accelerating sensors.
    if (GoAcceleratorMgr_SensorEntryCount(manager))
    {
        kLogf("%s: error - not allowed to reconfigure port range while accelerating sensors", __FUNCTION__);
        return kERROR_STATE;
    }

    kCheck(GoAccelSensorPortAlloc_SetPortRange(obj->portAlloc, startPort, endPort));

    return kOK;
}

// This function returns the currently configured port range used for accelerating
// sensors.
GoFx(kStatus) GoAcceleratorMgr_GetPortRange(GoAcceleratorMgr manager, k16u* startPort, k16u* endPort)
{
    kObj(GoAcceleratorMgr, manager);

    if (kIsNull(startPort) || kIsNull(endPort))
    {
        kLogf("%s: error - received null arguments for ports", __FUNCTION__);

        return kERROR_PARAMETER;
    }

    kCheck(GoAccelSensorPortAlloc_GetPortRange(obj->portAlloc, startPort, endPort));

    return kOK;
}

// This function returns the min and max limits of the port range that can be
// configured, and also the minimum number of ports the range must support.
GoFx(kStatus) GoAcceleratorMgr_GetPortRangeLimits(GoAcceleratorMgr manager, k16u* startLimit, k16u* endLimit, k16u* minNumPorts)
{
    kObj(GoAcceleratorMgr, manager);

    if (kIsNull(startLimit) || kIsNull(endLimit) || kIsNull(minNumPorts))
    {
        kLogf("%s: error - received null arguments for port range limits", __FUNCTION__);

        return kERROR_PARAMETER;
    }

    kCheck(GoAccelSensorPortAlloc_GetPortRangeLimits(obj->portAlloc, startLimit, endLimit, minNumPorts));

    return kOK;
}

// This public function returns the number of accelerated sensors which this
// accelerator manager object is currently managing.
GoFx(kSize) GoAcceleratorMgr_AccelSensorCount(GoAcceleratorMgr manager)
{
    kObj(GoAcceleratorMgr, manager);

    // Use an existing internal function to return the count.
    return (GoAcceleratorMgr_SensorEntryCount(manager));
}

// ====================== Private Functions ====================================
GoFx(kStatus) GoAcceleratorMgr_InitSensorBlock(GoAcceleratorMgr manager, kAlloc alloc)
{
    kObj(GoAcceleratorMgr, manager);

    kCheck(kLock_Construct(&obj->sensors.lock, alloc));
    kCheck(kArrayList_Construct(&obj->sensors.attachedSensors, kTypeOf(GoAcceleratorMgrSensorEntry), 0, alloc));

    return kOK;
}

GoFx(kStatus) GoAcceleratorMgr_ReleaseSensorBlock(GoAcceleratorMgr manager)
{
    kObj(GoAcceleratorMgr, manager);

    // Clean up all the objects within the sensor list before deleting the list.
    kCheck(GoAcceleratorMgr_ReleaseAllSensorEntryObject(manager));

    kCheck(kDestroyRef(&obj->sensors.attachedSensors));
    kCheck(kDestroyRef(&obj->sensors.lock));

    return kOK;
}

GoFx(kStatus) GoAcceleratorMgr_CreateUdpClient(GoAcceleratorMgr manager, kAlloc alloc)
{
    kObj(GoAcceleratorMgr, manager);

    kCheck(kUdpClient_Construct(&obj->threads.eventUdpClient, kIP_VERSION_4, kObject_Alloc(manager)));

    kCheck(kUdpClient_EnableReuseAddress(obj->threads.eventUdpClient, kTRUE));

    // Because this object is the event server, bind to the well known
    // accelerator event port.
    kCheck(kUdpClient_Bind(obj->threads.eventUdpClient, kIpAddress_LoopbackV4(), GO_ACCELERATOR_PORT_EVENT));

    return kOK;
}

GoFx(kStatus) GoAcceleratorMgr_InitThreadBlock(GoAcceleratorMgr manager, kAlloc alloc)
{
    kObj(GoAcceleratorMgr, manager);

    // Create worker thread and queue.
    kCheck(kMsgQueue_Construct(&obj->threads.workerQueue, kTypeOf(GoAcceleratorMgrWorkerMsg), alloc));
    kCheck(kThread_Construct(&obj->threads.workerThread, alloc));

    // Create event thread and udp client.
    kCheck(GoAcceleratorMgr_CreateUdpClient(manager, alloc));
    kCheck(kThread_Construct(&obj->threads.eventThread, alloc));

    // Create callback thread and queue. Use the same message format as the
    // worker thread because the message already includes all the information
    // needed by the callback thread.
    kCheck(kMsgQueue_Construct(&obj->threads.callbackQueue, kTypeOf(GoAcceleratorMgrWorkerMsg), alloc));
    kCheck(kThread_Construct(&obj->threads.callbackThread, alloc));

    return kOK;
}

GoFx(kStatus) GoAcceleratorMgr_ReleaseThreadBlock(GoAcceleratorMgr manager)
{
    kObj(GoAcceleratorMgr, manager);

    // Create callback thread and queue.
    kCheck(kDestroyRef(&obj->threads.callbackThread));
    kCheck(kDestroyRef(&obj->threads.callbackQueue));

    kCheck(kDestroyRef(&obj->threads.eventUdpClient));
    kCheck(kDestroyRef(&obj->threads.eventThread));

    kCheck(kDestroyRef(&obj->threads.workerThread));
    kCheck(kDestroyRef(&obj->threads.workerQueue));

    return kOK;
}

// This function does basic sanity checking on parameters received from user
// request and tries to allocate ports if client requested auto port allocation.
GoFx(kBool) GoAcceleratorMgr_ValidateSensorParam(GoAcceleratorMgr manager, GoAcceleratorMgrSensorParam* param)
{
    kObj(GoAcceleratorMgr, manager);
    kBool isValid = kTRUE;

    if (param->platformIpAddress.version != kIP_VERSION_4)
    {
        isValid = kFALSE;
    }

    return isValid;
}

GoFx(void) GoAcceleratorMgr_InitSensorInfo(GoAcceleratorMgr manager, GoAcceleratorMgrWorkerMsg* msg, GoAcceleratorMgrSensorEntry* sensorInfo)
{
    kObj(GoAcceleratorMgr, manager);

    (void) kMemSet(sensorInfo, 0, sizeof(*sensorInfo));

    sensorInfo->sensorId                    = msg->sensorId;
    // Initialize status to acceleration in progress state because sensor info
    // initialization is only called when starting to accelerate a sensor.
    //
    // TODO: should setting status to accelerating wait until the accelerating
    // event is received from the acceleration process? Otherwise there are
    // two places where ACCERATING status set:
    //  - here when initializing senorinfo structure, and
    //  - when the acceleration process is actually starting accelerate a sensor.
    // Could set the status
    // to something else here, like "starting initiated".
    sensorInfo->status = GO_SENSOR_ACCEL_STATUS_ACCELERATING;
    sensorInfo->platformIpAddress           = msg->data.param.platformIpAddress;
    sensorInfo->portInfo                    = msg->data.param.ports;

    // Skipped backup initialization.

    sensorInfo->sensorProcess = kNULL;
    sensorInfo->eventRemoteAddr.address = kIpAddress_LoopbackV4();
    sensorInfo->eventRemoteAddr.port = kIP_PORT_ANY;
    sensorInfo->lastStartTime = 0;
    sensorInfo->stoppedReason= ACCELERATOR_MGR_ACCEL_STOPPED_NONE;
    sensorInfo->raisedStopFlag = kFALSE;
}

GoFx(void) GoAcceleratorMgr_SetStoppedReason(GoAcceleratorMgr manager, GoAcceleratorMgrSensorEntry* sensorInfo, GoAcceleratorMgrStoppedReason stoppedReason)
{
    kObj(GoAcceleratorMgr, manager);

    sensorInfo->stoppedReason = stoppedReason;
}

GoFx(void) GoAcceleratorMgr_ClearStoppedReason(GoAcceleratorMgr manager, GoAcceleratorMgrSensorEntry* sensorInfo)
{
    kObj(GoAcceleratorMgr, manager);

    sensorInfo->stoppedReason = ACCELERATOR_MGR_ACCEL_STOPPED_NONE;
}

// This function records the current time when sensor is started or restarted.
GoFx(void) GoAcceleratorMgr_UpdateSensorLastStartTime(GoAcceleratorMgr manager, GoAcceleratorMgrSensorEntry* sensorInfo)
{
    kObj(GoAcceleratorMgr, manager);

    sensorInfo->lastStartTime = kTimer_Now();
}

// This function updates the sensor entry with information relevant to a successful
// acceleration. Calling this function if acceleration fails is not meaningful.
GoFx(void) GoAcceleratorMgr_UpdateSensorInfoAfterAccel(GoAcceleratorMgr manager, GoAcceleratorMgrSensorEntry* sensorInfo)
{
    kObj(GoAcceleratorMgr, manager);

    sensorInfo->backup = sensorInfo->backup;
    sensorInfo->sensorProcess = sensorInfo->sensorProcess;
}

// This function backs up sensor object information that will be overwritten
// during the acceleration process.
// The sensor is restored with this back up information when it is unaccelerated.
GoFx(void) GoAcceleratorMgr_BackupSensorInfo(GoAcceleratorMgr manager, GoSensor sensor, GoAcceleratorMgrSensorEntry* sensorInfo)
{
    kObj(GoAcceleratorMgr, manager);

    // Backup the ports for sensor object and sensor's control object before they
    // are modified to point to accelerator host.
    // Only the following ports need backing up. The web port does
    // not need backing up because the sensor object does not store
    // the web port, but we set it to the expected http server default.
    //  - control (from GoSensor)
    //  - upgrade (from GoSensor)
    //  - public data (from GoSensor)
    //  - health (from GoSensor)
    sensorInfo->backup.ports.controlPort     = (k16u) GoSensor_ControlPort(sensor);
    sensorInfo->backup.ports.upgradePort     = (k16u) GoSensor_UpgradePort(sensor);
    sensorInfo->backup.ports.healthPort      = (k16u) GoSensor_HealthPort(sensor);
    sensorInfo->backup.ports.publicDataPort  = (k16u) GoSensor_DataPort(sensor);
    sensorInfo->backup.ports.webPort         = GO_SDK_RESERVED_PORT_DEFAULT_SENSOR_HTTP_SERVER;

    // Save the physical sensor's configured IP address.
    sensorInfo->backup.sensorAddressInfo     = *GoSensor_AddressInfo(sensor);
}

GoFx(kStatus) GoAcceleratorMgr_PointSensorObjectToLocalHost(GoAcceleratorMgr manager, GoAcceleratorMgrSensorEntry* sensorInfo, GoSensor sensor)
{
    kObj(GoAcceleratorMgr, manager);

    // Update the sensor address to point to the local accelerator host
    // (use loopback for now).
    GoSensor_AddressInfo(sensor)->address = kIpAddress_LoopbackV4();

    // Update the ports used by sensor object. Private data port and web port are not
    // part of the sensor object.
    kCheck(GoSensor_SetControlPort(sensor, (k32u) sensorInfo->portInfo.controlPort));
    kCheck(GoSensor_SetUpgradePort(sensor, (k32u) sensorInfo->portInfo.upgradePort));
    kCheck(GoSensor_SetHealthPort(sensor, (k32u) sensorInfo->portInfo.healthPort));
    kCheck(GoSensor_SetDataPort(sensor, (k32u) sensorInfo->portInfo.publicDataPort));

    return kOK;
}

GoFx(kStatus) GoAcceleratorMgr_PointSensorObjectToDevice(GoAcceleratorMgr manager, GoAcceleratorMgrSensorEntry* sensorInfo, GoSensor sensor)
{
    kObj(GoAcceleratorMgr, manager);

    // Restore the ports used by sensor object. Private data port and web port are not
    // part of the control object.
    kCheck(GoSensor_SetControlPort(sensor, (k32u) sensorInfo->backup.ports.controlPort));
    kCheck(GoSensor_SetUpgradePort(sensor, (k32u) sensorInfo->backup.ports.upgradePort));
    kCheck(GoSensor_SetHealthPort(sensor, (k32u) sensorInfo->backup.ports.healthPort));
    kCheck(GoSensor_SetDataPort(sensor, (k32u) sensorInfo->backup.ports.publicDataPort));

    // Restore the sensor address to point to the local accelerator host.
    *GoSensor_AddressInfo(sensor) = sensorInfo->backup.sensorAddressInfo;

    return kOK;
}

GoFx(kStatus) GoAcceleratorMgr_SetupProcessArguments(GoAcceleratorMgr manager, GoAcceleratorMgrSensorEntry* sensorInfo)
{
    kObj(GoAcceleratorMgr, manager);
    kText64 processArg;
    kText16 ipAddrStr;
    kProcess process;

    process = sensorInfo->sensorProcess;

    // The order of arguments must match the order expected by the
    // acceleration process executable.
    kCheck(kStrPrintf(processArg, kCountOf(processArg), "%u", sensorInfo->sensorId));
    kCheck(kProcess_AddArgument(process, (kChar*) processArg));
    kCheck(kStrPrintf(processArg, kCountOf(processArg), "%u", sensorInfo->portInfo.controlPort));
    kCheck(kProcess_AddArgument(process, (kChar*) processArg));
    kCheck(kStrPrintf(processArg, kCountOf(processArg), "%u", sensorInfo->portInfo.upgradePort));
    kCheck(kProcess_AddArgument(process, (kChar*) processArg));
    kCheck(kStrPrintf(processArg, kCountOf(processArg), "%u", sensorInfo->portInfo.healthPort));
    kCheck(kProcess_AddArgument(process, (kChar*) processArg));
    kCheck(kStrPrintf(processArg, kCountOf(processArg), "%u", sensorInfo->portInfo.privateDataPort));
    kCheck(kProcess_AddArgument(process, (kChar*) processArg));
    kCheck(kStrPrintf(processArg, kCountOf(processArg), "%u", sensorInfo->portInfo.publicDataPort));
    kCheck(kProcess_AddArgument(process, (kChar*) processArg));
    kCheck(kStrPrintf(processArg, kCountOf(processArg), "%u", sensorInfo->portInfo.webPort));
    kCheck(kProcess_AddArgument(process, (kChar*) processArg));

    // IP address is passed as a string in dotted notation.
    kCheck(kIpAddress_Format(sensorInfo->platformIpAddress, ipAddrStr, kCountOf(ipAddrStr)));
    kCheck(kStrPrintf(processArg, kCountOf(processArg), "%s", ipAddrStr));
    kCheck(kProcess_AddArgument(process, (kChar*) processArg));

    return kOK;
}

GoFx(kStatus) GoAcceleratorMgr_StartAccelerationProcess(GoAcceleratorMgr manager, GoAcceleratorMgrSensorEntry* sensorInfo)
{
    kObj(GoAcceleratorMgr, manager);
    kChar dirPath[kPATH_MAX];
    kStatus exception;

    // Initialize to null so the free call works even if the construction fails.
    sensorInfo->sensorProcess = kNULL;

    kTry
    {
        kTest(kPath_Application(GO_ACCELERATOR_MGR_ACCEL_PROCESS_EXECUTABLE, dirPath, kCountOf(dirPath)));
        kTest(kProcess_Construct(&sensorInfo->sensorProcess, dirPath, kObject_Alloc(manager)));

        kTest(GoAcceleratorMgr_SetupProcessArguments(manager, sensorInfo));
        kTest(kProcess_Start(sensorInfo->sensorProcess));
    }
    kCatch(&exception)
    {
        kDestroyRef(&sensorInfo->sensorProcess);

        kEndCatch(exception);
    }

    return kOK;
}

// This function is used by the worker thread to start accelerating a sensor.
GoFx(kStatus) GoAcceleratorMgr_WorkerStartAcceleration(GoAcceleratorMgr manager, GoAcceleratorMgrWorkerMsg* msg)
{
    kObj(GoAcceleratorMgr, manager);
    GoSensor sensor = kNULL;
    GoAcceleratorMgrSensorEntry* sensorEntryPtr = kNULL;
    kStatus status;
    kStatus exception;
    k32u    step = 1;  // used in log messages to help debug which step failed.

    // Restart commands are triggered by sensors already on the accelerated
    // sensor list, so skip this check for restart commands.
    if ((msg->command == ACCELERATOR_MGR_START) || (msg->command == ACCELERATOR_MGR_RESTORE_START))
    {
        // Need to do this check within the worker thread to definitively
        // conclude if request is a duplicate request or not.
        // Ignore request if it is a duplicate.
        if (GoAcceleratorMgr_IsSensorAcceleratedByLocalHost(manager, msg->sensorId))
        {
            // Release the ports allocated to this request.
            kCheck(GoAccelSensorPortAlloc_RemoveAllocatedPorts(obj->portAlloc, msg->sensorId, &msg->data.param.ports));

            return kOK;
        }
    }

    kTry
    {
        // Add to sensor list only if this is a start or restore command.
        kTest(GoAcceleratorMgr_AddNewSensorToList(manager, msg));

        step++;

        // Further updates to the sensor info will be directly into the
        // sensor list's entry.
        // It is important that the entry is not deleted during the acceleration
        // process. This is guaranteed because this thread is the only thread
        // that deletes a sensor entry so there is no need for multi-threading
        // issue with the sensor entry's existence.
        kTest(GoAcceleratorMgr_FindSensorEntry(manager, msg->sensorId, kNULL, &sensorEntryPtr));

        // Save the desired operation for the sensor.
        sensorEntryPtr->cmdInProgress = msg->command;

        step++;

        // Port check is needed when restoring sensor or restarting sensor because
        // there is no guarantee that the ports originally free are still free
        // when the system reboots or has to retry accelerating the sensor
        // after recovering from an error (eg. crash).
        kTest(GoAcceleratorMgr_CheckRestoredSensorPortAvailable(manager, msg, sensorEntryPtr));

        // Record the time when acceleration is attempted for all acceleration
        // attempts.
        GoAcceleratorMgr_UpdateSensorLastStartTime(manager, sensorEntryPtr);

        step++;

        // Share the sensor object to prevent it from being released
        // while it is being used.
        // This also validates that the sensor did not go away between the time the
        // start request was received and the worker thread handles the start request.
        kTest(GoAcceleratorMgr_ShareSensor(manager, msg->sensorId, &sensor));

        step++;

        // Backup information from the sensor object into the sensor list entry
        // after sensor prevented from being released.
        //
        // NOTE: The backup sensor info is still needed for the case of a 
        // "restored sensor running an incompatible firmware version" (described 
        // in the comment preceding the call to GoAcceleratorMgr_ValidateSensorVersion()).
        //
        // Without the backup information, the changes that made the direct acceleration
        // link available (done when the 2nd GoMax ethernet port was added) has 
        // no values to fall back to (see GOC-12969).
        GoAcceleratorMgr_BackupSensorInfo(manager, sensor, sensorEntryPtr);

        step++;

        // This version check is needed to be placed here to handle the scenario
        // where a restored sensor is running an incompatible firmware version.
        // The requirements for this scenario are:
        // - sensor must be added to the accelerator sensor list.
        // - sensor acceleration is allowed to fail/be denied.
        //
        // The acceleration process does not check the SDK version so the version
        // check criteria cannot be put into that process's code.
        // Normally, new requests would be rejected and the sensor would not be
        // put into configuration persistence or acceleratd sensor list.
        // However, for a restore request, the requirements (as noted above)
        // are different from a new request.
        //
        // This check is redundant for a new request since new requests must
        // have passed this check to get into the worker thread, but there is no
        // harm in running it again, just to avoid having to differentiate
        // between a new or restore request.
        kTest(GoAcceleratorMgr_ValidateSensorVersion(manager, GoSdk_Version(), 
                                                     GoSensor_FirmwareVersion(sensor)));

        step++;

        // Ensure sensor is disconnected first before trying to accelerate.
        // Do not call this function with the GoSystem object state locked
        // because that can cause intermittent deadlocks!!
        kTest(GoSensor_Disconnect(sensor));

        step++;

        // Start acceleration process.
        status = GoAcceleratorMgr_StartAccelerationProcess(manager, sensorEntryPtr);

        // Want to log process start up failure explicitly.
        if (!kSuccess(status))
        {
            kLogf("%s: error %d starting accelerator process for sensor %u",
                __FUNCTION__,
                status,
                sensorEntryPtr->sensorId);
        }

        // Don't update sensor object to point to non-existent accelerator host
        // if acceleration process start up fails.
        kTest(status);

        step++;

        // Back up the sensor object info that will be overwritten, and store
        // the process object handle.
        GoAcceleratorMgr_UpdateSensorInfoAfterAccel(manager, sensorEntryPtr);

        // Now update the SDK sensor object to point to the accelerator host
        // instead of the physical sensor so that all future sensor connections
        // will be to the accelerator host.
        //
        // Don't update the sensor object acceleration state. It will be
        // done automatically during sensor discovery.
        kTest(GoAcceleratorMgr_PointSensorObjectToLocalHost(manager, sensorEntryPtr, sensor));
    }
    kCatchEx(&exception)
    {
        // Acceleration failed. Set the status value to indicate the failed
        // start. Simulate a dummy event to update the status.
        GoAcceleratorMgr_UpdateSensorEntryFromEvent(manager, msg->sensorId, GO_SENSOR_ACCEL_STATUS_STOPPED, kNULL);

        // Error log for all acceleration failures.
        kLogf("%s: error %d accelerating sensor %u at step %u", __FUNCTION__, exception, msg->sensorId, step);

        kEndCatchEx(exception);
    }
    kFinallyEx
    {
        if (!kIsNull(sensor))
        {
            // Finished with the sensor object, so can unlock sensor object.
            GoAcceleratorMgr_UnshareSensor(manager, sensor);
        }

        kEndFinallyEx();
    }

    return kOK;
}

GoFx(kStatus) GoAcceleratorMgr_TerminateGraceful(GoAcceleratorMgr manager, GoAcceleratorMgrSensorEntry* sensorInfo)
{
    kObj(GoAcceleratorMgr, manager);
    GoAcceleratorEventMsg eventMsg;

    eventMsg.sensorId = sensorInfo->sensorId;
    eventMsg.event = GO_ACCELERATOR_EVENT_TERMINATE;

    kCheck(GoAcceleratorMgr_SendEvent(manager, &sensorInfo->eventRemoteAddr, &eventMsg));

    return kOK;
}

// This function updates the cached sensor object's state to mark it as
// available.
// The list of online sensor objects is updated on demand by SDK application,
// so until the list is updated, the cached sensor object state would still
// indicate the sensor is accelerated, even though it is no longer
// accelerated. So there is a finite time between when the sensor acceleration is
// stopped by this host, and when the sensor object is refreshed.
// This function is to update the sensor object state so that during this
// finite time to correctly reflect the sensor is available (ie. no longer
// accelerated).,
GoFx(kStatus) GoAcceleratorMgr_MarkSensorAvailable(GoAcceleratorMgr manager, GoSensor sensor)
{
    kObj(GoAcceleratorMgr, manager);
    kStatus exception;

    kTry
    {
        kTest(GoSensor_SetAccelOpMode(sensor, GO_DISCOVERY_OP_MODE_STANDALONE));
        kTest(GoSensor_SetAccelState(sensor, GO_SENSOR_ACCEL_STATE_AVAILABLE));
    }
    kCatch(&exception)
    {
        kLogf("%s: error %d marking sensor available", __FUNCTION__, exception);

        kEndCatch(exception);
    }

    return kOK;
}

GoFx(kStatus) GoAcceleratorMgr_SendEventToWorkerThread(GoAcceleratorMgr manager, k32u sensorId, GoAcceleratorMgrAccelEvents accelEvent)
{
    kObj(GoAcceleratorMgr, manager);
    GoAcceleratorMgrWorkerMsg msg;

    msg.command = ACCELERATOR_MGR_EVENT;
    msg.sensorId = sensorId;
    msg.data.eventData.event = accelEvent;

    // Don't need to initialize remote adddress because this event
    // will not update the sensor list entries.

    kCheck(kMsgQueue_AddT(obj->threads.workerQueue, &msg));

    return kOK;
}

// Stopping acceleration under error scenarios may encounter errors along each
// of the steps. However, not all the steps dependent on each other, so the
// strategy here is to forge ahead and attempt all the steps, even if some
// of the steps fail.
// The desired end result is a sensor that is on the free/unowned list so that
// use can try to accelerate it again.
GoFx(kStatus) GoAcceleratorMgr_WorkerStopAcceleration(GoAcceleratorMgr manager, GoAcceleratorMgrWorkerMsg* msg)
{
    kObj(GoAcceleratorMgr, manager);
    GoSensor sensor = kNULL;
    GoAcceleratorMgrSensorEntry* entryPtr = kNULL;
    GoAcceleratorMgrSensorEntry  sensorInfo;
    kSize index;
    kStatus status;
    kStatus exception;

    // Prevent sensor from being released while it is being used.
    // If this operation fails (eg. sensor disappeared),
    // don't exit immediately because there are things that can still be done
    // without the sensor object.
    status = GoAcceleratorMgr_ShareSensor(manager, msg->sensorId, &sensor);
    if (!kSuccess(status))
    {
        kLogf("%s: status %d getting sensor %u object",
            __FUNCTION__,
            status,
            msg->sensorId);
    }

    kTry
    {
        // Don't need to lock the sensor list because the worker thread is the
        // only thread that deletes a sensor entry, so it knows when the
        // entry is gone or not.
        kTest(GoAcceleratorMgr_FindSensorEntry(manager, msg->sensorId, &index, &entryPtr));
        sensorInfo = *entryPtr;

        // Set state to decelerating. This is the only non-read operation to the
        // sensor entry prior to deleting the entry.
        //
        // TODO: Should this even be done here or should the status wait for
        // the decelerating event from the acceleration process to set the
        // status. Otherwise there are 2 places that set DECELERATING status:
        //  - here in this stop acceleration function, and
        //  - when accelerating process sends the event while it is actually
        //    going through its shutdown code.
        // Perhaps set the status to something else like "stopping initiated".
        entryPtr->status = GO_SENSOR_ACCEL_STATUS_DECELERATING;

        // Update sensor object only if able to share it. Otherwise skip it
        // and continue with the other steps in decelerating a sensor.
        if (sensor)
        {
            // Ensure sensor is disconnected first.
            // Ignore error to continue cleaning up.
            GoSensor_Disconnect(sensor);

            // Restore the sensor object's pre-acceleration settings to point
            // back to the physical sensor.
            // Ignore errors to continue cleaning up.
            GoAcceleratorMgr_PointSensorObjectToDevice(manager, &sensorInfo, sensor);
        }

        // Attempt graceful termination first instead of forceful termination.
        // If process is missing, this attempt will likely fail, so ignore
        // error to continue processing.
        status = GoAcceleratorMgr_TerminateGraceful(manager, &sensorInfo);
        if (!kSuccess(status))
        {
            kLogf("%s: error %d initiating graceful termination for sensor %u",
                __FUNCTION__,
                status,
                msg->sensorId);
        }

        // Need to check if a process object was created for this sensor or not.
        // A sensor on the accelerated sensor list might not have a process object
        // if the sensor restore failed before the process object is created.
        // So can skip terminating the process and continue with the rest of the
        // clean up.
        if (!kIsNull(sensorInfo.sensorProcess))
        {
            // This will do a forced termination after the timeout if the graceful
            // termination does not work.
            // Ignore error to continue processing.
            status = kProcess_Wait(sensorInfo.sensorProcess, GO_ACCELERATOR_MGR_TIMEOUT_GRACEFUL_TERMINATION, kNULL);
            if (!kSuccess(status))
            {
                kLogf("%s: error %d waiting for sensor %u exit",
                    __FUNCTION__,
                    status,
                    msg->sensorId);
            }
        }

        // After sensor has been unaccelerated, free up the ports allocated
        // for the sensor.
        // Note the port info reference points to the sensor entry in the
        // sensor list so free the ports before deleting the entry
        // in the sensor list.
        kTest(GoAccelSensorPortAlloc_RemoveAllocatedPorts(obj->portAlloc, msg->sensorId, &sensorInfo.portInfo));

        // Delete sensor entry in sensor list. Don't reference the
        // entry, especially through a pointer, anymore!
        kTest(GoAcceleratorMgr_DeleteSensorEntryAt(manager, index));

        // Update the cached sensor object state to indicate it is no longer
        // accelerated. Do this only after sensor has been
        // removed from the accelerated sensor list, to prevent the sensor
        // from appearing in both accelerated and unowned lists.
        if (sensor)
        {
            kTest(GoAcceleratorMgr_MarkSensorAvailable(manager, sensor));
        }

        // The acceleration process does not send a "decelerated" event
        // because it is not necessary given that the process wait call
        // returns only after the process has been terminated. So the return
        // from the wait call is sufficient to know the sensor is decelerated.
        // Notify the SDK client update handler sensor is decelerated.
        // Do this by sending the event to the worker thread so that
        // the decelerated event is handled after any other events received
        // from the acceleration process while it was terminating.
        kTest(GoAcceleratorMgr_SendEventToWorkerThread(manager, msg->sensorId, GO_ACCELERATOR_EVENT_DECELERATED));
    }
    kCatchEx(&exception)
    {
        kLogf("%s: error %d stopping acceleration for sensor %u",
            __FUNCTION__,
            exception,
            msg->sensorId);

        kEndCatchEx(exception);
    }
    kFinallyEx
    {
        // Finished with the sensor object, so can unlock sensor object.
        if (sensor)
        {
            GoAcceleratorMgr_UnshareSensor(manager, sensor);
        }

        kEndFinallyEx();
    }

    return kOK;
}

GoFx(kStatus) GoAcceleratorMgr_WorkerHandleEvent(GoAcceleratorMgr manager, GoAcceleratorMgrWorkerMsg* msg)
{
    kObj(GoAcceleratorMgr, manager);
    GoSensorAccelStatus accelStatus;
    kBool updateSensorList = kTRUE;
    kBool postToCallback = kTRUE;

    kLogf("%s: event %#x (%s)",
        __FUNCTION__,
        msg->data.eventData.event,
        GoAcceleratorMgr_GetEventName(msg->data.eventData.event));

    // Initialize to something to suppress compiler warning about potentially
    // using an uninitialized variable.
    accelStatus = GO_SENSOR_ACCEL_STATUS_SUCCESS;

    switch (msg->data.eventData.event)
    {
    case GO_ACCELERATOR_EVENT_ACCELERATING:
        accelStatus = GO_SENSOR_ACCEL_STATUS_ACCELERATING;
        break;
    case GO_ACCELERATOR_EVENT_SUCCESS:
        accelStatus = GO_SENSOR_ACCEL_STATUS_SUCCESS;
        break;
    case GO_ACCELERATOR_EVENT_DECELERATING:
        accelStatus = GO_SENSOR_ACCEL_STATUS_DECELERATING;
        break;
    case GO_ACCELERATOR_EVENT_STOPPED:
        accelStatus = GO_SENSOR_ACCEL_STATUS_STOPPED;
        break;
    case GO_ACCELERATOR_EVENT_DISCONNECT:
        accelStatus = GO_SENSOR_ACCEL_STATUS_MISSING;
        break;
    case GO_ACCELERATOR_EVENT_DECELERATED:
        // Event raised internally by the Accelerator Manager. Fall through.
    case GO_ACCELERATOR_EVENT_PROCESS_STOPPED:
        // Event raised internally by the Accelerator Manager.
        //
        // Don't update sensor list. Just invoke the update handler callback by
        // sending event to the callback thread.
        updateSensorList = kFALSE;
        break;
    default:
        // Ignore event by doing nothing.
        updateSensorList = kFALSE;
        postToCallback = kFALSE;
        break;
    }

    if (updateSensorList)
    {
        kCheck(GoAcceleratorMgr_UpdateSensorEntryFromEvent(manager, msg->sensorId, accelStatus, &msg->data.eventData.remoteAddr));
    }

    if (postToCallback)
    {
        // Forward to callback thread about the event.
        kCheck(kMsgQueue_AddT(obj->threads.callbackQueue, msg));
    }

    return kOK;
}

// This thread is responsible for all the work done by the accelerator manager,
// such as starting and stopping sensor acceleration and adding/deleting
// entries into the sensor list.
// Using a single thread to do this helps reduce thread synchronization issues
// for these sensor operations.
GoFx(kStatus) kCall GoAcceleratorMgr_WorkerThread(GoAcceleratorMgr manager)
{
    kObj(GoAcceleratorMgr, manager);
    kStatus status;
    GoAcceleratorMgrWorkerMsg msg;
    k64u queueWaitTime;
    k64u nextMonitorTime;

    // Set up when to monitor accelerator process.
    nextMonitorTime = kTimer_Now() + GO_ACCELERATOR_MGR_MONITOR_PERIOD;

    // Set the queue to timeout to trigger monitoring action.
    queueWaitTime = GO_ACCELERATOR_MGR_MONITOR_PERIOD;

    // Loop forever until get a null command.
    while (kTRUE)
    {
        if (kSuccess(status = kMsgQueue_Remove(obj->threads.workerQueue, &msg, queueWaitTime)))
        {
            kLogf("%s: processing command <%s> for sensor %u",
                __FUNCTION__,
                GoAcceleratorMgr_GetCommandName(msg.command),
                msg.sensorId);

            if (msg.command == ACCELERATOR_MGR_NO_CMD)
            {
                // Received command to stop thread. Exit loop.
                break;
            }

            // Ignore errors to keep thread running.
            switch (msg.command)
            {
            case ACCELERATOR_MGR_START:
                // Fall through.
            case ACCELERATOR_MGR_RESTART:
                // Fall through
            case ACCELERATOR_MGR_RESTORE_START:
                GoAcceleratorMgr_WorkerStartAcceleration(manager, &msg);
                break;
            case ACCELERATOR_MGR_STOP:
                GoAcceleratorMgr_WorkerStopAcceleration(manager, &msg);
                break;
            case ACCELERATOR_MGR_EVENT:
                GoAcceleratorMgr_WorkerHandleEvent(manager, &msg);
                break;
            default:
                // Ignore unknown command.
                break;
            }  // switch
        }

        // Check if it is time to monitor the accelerator processes whenever
        // this thread runs.
        // Do sensor health monitoring __after__ handling whatever message woke
        // up the thread.
        //
        // TODO: should monitoring be done as an worker thread command like
        //       all the start/stop/events?
        if (GoAcceleratorMgr_IsTimeToMonitorSensor(manager, nextMonitorTime))
        {
            // Time to check the health of all the sensors configured for acceleration.
            if (!kSuccess(status = GoAcceleratorMgr_CheckSensorAcceleration(manager)))
            {
                // Don't exit thread because of this error.
                kLogf("%s: error %d checking sensor acceleration", __FUNCTION__, status);
            }

            // After finishing the sensor health monitoring, set up the next
            // time when to do monitoring again.
            nextMonitorTime = kTimer_Now() + GO_ACCELERATOR_MGR_MONITOR_PERIOD;
            queueWaitTime = GO_ACCELERATOR_MGR_MONITOR_PERIOD;
        }
        else
        {
            // Didn't do monitoring, so adjust the queue wait time to reach
            // the next monitoring period.
            queueWaitTime = nextMonitorTime - kTimer_Now();
        }

        // The message is not an object and does not have objects embedded, so
        // there is nothing in the message to free or dispose of.
    }  // while

    return kOK;
}

// This thread is responsible for waiting for events from all the acceleration
// (child) process. It delegates all processing of the events to the worker
// thread.
GoFx(kStatus) kCall GoAcceleratorMgr_EventThread(GoAcceleratorMgr manager)
{
    kObj(GoAcceleratorMgr, manager);
    GoAcceleratorMgrWorkerMsg   workerMsg;
    kStatus                     status;

    // Loop forever until told to stop with a dummy message.
    while (kTRUE)
    {
        // This call blocks waiting for a input.
        status = GoAcceleratorMgr_ReceiveEvent(manager, &workerMsg);
        if (kSuccess(status))
        {
            // Check for event used to terminate this thread.
            if ((workerMsg.sensorId == 0) && (workerMsg.data.eventData.event == GO_ACCELERATOR_EVENT_NONE))
            {
                // Thread should stop.
                break;
            }

            // Forward to worker thread to handle the event.
            kMsgQueue_AddT(obj->threads.workerQueue, &workerMsg);
        }
    }  // while

    return kOK;
}

// This thread is responsible to calling the SDK user update callback
// function. A dedicated thread is used for that in case the
// callback runs for a long time or makes blocking calls.
// If the callback were called by one of the other internal threads, it
// may cause latency in processing acceleration functionality.
GoFx(kStatus) kCall GoAcceleratorMgr_CallbackThread(GoAcceleratorMgr manager)
{
    kObj(GoAcceleratorMgr, manager);
    GoAcceleratorMgrWorkerMsg msg;
    GoAcceleratorMgrAccelUpdate updateStatus;
    kBool doUpdate;

    // Loop forever until get a null command.
    while (kTRUE)
    {
        // TODO: should thread exit on a queue wait error?
        kCheck(kMsgQueue_Remove(obj->threads.callbackQueue, &msg, kINFINITE));

        if (msg.command == ACCELERATOR_MGR_NO_CMD)
        {
            // Received command to stop thread. Exit loop.
            break;
        }

        // Ignore errors to keep thread running until thread is told to exit.
        if (msg.command == ACCELERATOR_MGR_EVENT)
        {
            if (kSuccess(kLock_Enter(obj->updateCallback.lock)))
            {
                if (!kIsNull(obj->updateCallback.onAccelUpdate.function))
                {
                    doUpdate = kTRUE;

                    kLogf("%s: event %#x", __FUNCTION__, msg.data.eventData.event);

                    switch (msg.data.eventData.event)
                    {
                    case GO_ACCELERATOR_EVENT_ACCELERATING:
                        updateStatus.accelEvent = GO_ACCELERATOR_MGR_EVENT_ACCELERATING;
                        break;
                    case GO_ACCELERATOR_EVENT_SUCCESS:
                        updateStatus.accelEvent = GO_ACCELERATOR_MGR_EVENT_ACCELERATED;
                        break;
                    case GO_ACCELERATOR_EVENT_DECELERATING:
                        updateStatus.accelEvent = GO_ACCELERATOR_MGR_EVENT_DECELERATING;
                        break;
                    case GO_ACCELERATOR_EVENT_STOPPED:
                        updateStatus.accelEvent = GO_ACCELERATOR_MGR_EVENT_STOPPED;
                        break;
                    case GO_ACCELERATOR_EVENT_DISCONNECT:
                        updateStatus.accelEvent = GO_ACCELERATOR_MGR_EVENT_DISCONNECTED;
                        break;
                    case GO_ACCELERATOR_EVENT_DECELERATED:
                        updateStatus.accelEvent = GO_ACCELERATOR_MGR_EVENT_DECELERATED;
                        break;
                    case GO_ACCELERATOR_EVENT_PROCESS_STOPPED:
                        updateStatus.accelEvent = GO_ACCELERATOR_MGR_EVENT_PROCESS_STOPPED;
                        break;
                    default:
                        // Unexpected event.
                        doUpdate = kFALSE;
                        break;
                    }

                    if (doUpdate)
                    {
                        updateStatus.sensorId = msg.sensorId;
                        obj->updateCallback.onAccelUpdate.function(obj->updateCallback.onAccelUpdate.receiver, manager, &updateStatus);
                    }
                }
                kLock_Exit(obj->updateCallback.lock);
            }
        }

        // The message is not an object and does not have objects embedded, so
        // there is nothing in the message to free or dispose of.
    }  // while

    return kOK;
}

// This function adds a new sensor to the accelerated sensor list, if
// the acceleration request is a START or RESTORE request.
// Don't add on a restart command because sensor is already on the accelerated
// sensor list.
GoFx(kStatus) GoAcceleratorMgr_AddNewSensorToList(GoAcceleratorMgr manager, GoAcceleratorMgrWorkerMsg* msg)
{
    kObj(GoAcceleratorMgr, manager);

    if ((msg->command == ACCELERATOR_MGR_START) || (msg->command == ACCELERATOR_MGR_RESTORE_START))
    {
        GoAcceleratorMgrSensorEntry sensorInfo;

        // Once selected for acceleration, the sensor should remain in the
        // accelerated list even if acceleration encounters an error.
        //
        // Initialize sensor info with parameters from client.
        GoAcceleratorMgr_InitSensorInfo(manager, msg, &sensorInfo);

        // Add to the sensor list as soon as possible to reduce likelihood
        // that queries of the accelerated sensor list won't return an error
        // because the sensor was not found in the list.
        kCheck(GoAcceleratorMgr_AddSensorEntry(manager, &sensorInfo));
    }

    return kOK;
}

GoFx(kStatus) GoAcceleratorMgr_AddSensorEntry(GoAcceleratorMgr manager, GoAcceleratorMgrSensorEntry* sensorInfo)
{
    kObj(GoAcceleratorMgr, manager);

    // Probably no need to lock the list, but do it for consistency.
    kCheck(kLock_Enter(obj->sensors.lock));

    kTry
    {
        kTest(kArrayList_AddT(obj->sensors.attachedSensors, sensorInfo));
    }
    kFinally
    {
        kCheck(kLock_Exit(obj->sensors.lock));

        kEndFinally();
    }

    return kOK;
}

GoFx(kStatus) GoAcceleratorMgr_DeleteSensorEntryAt(GoAcceleratorMgr manager, kSize index)
{
    kObj(GoAcceleratorMgr, manager);
    GoAcceleratorMgrSensorEntry* entry;

    // Need to lock the list to prevent a reader accessing a deleted entry.
    kCheck(kLock_Enter(obj->sensors.lock));

    kTry
    {
        // Free any objects embedded in the entry before removing it from the
        // list.
        entry = kArrayList_AtT(obj->sensors.attachedSensors, index, GoAcceleratorMgrSensorEntry);
        kTest(GoAcceleratorMgr_ReleaseSensorEntryObject(manager, entry));
        kTest(kArrayList_Remove(obj->sensors.attachedSensors, index, kNULL));
    }
    kFinally
    {
        kCheck(kLock_Exit(obj->sensors.lock));

        kEndFinally();
    }

    return kOK;
}

GoFx(kSize) GoAcceleratorMgr_SensorEntryCount(GoAcceleratorMgr manager)
{
    kObj(GoAcceleratorMgr, manager);
    kSize sensorCount = 0;

    if (kSuccess(kLock_Enter(obj->sensors.lock)))
    {
        sensorCount = kArrayList_Count(obj->sensors.attachedSensors);
        kLock_Exit(obj->sensors.lock);
    }

    return sensorCount;
}

#if 0
GoFx(kStatus) GoAcceleratorMgr_DeleteSensorEntry(GoAcceleratorMgr manager, k32u sensorId)
{
    kObj(GoAcceleratorMgr, manager);
    kSize sensorCount;
    kSize i;
    GoAcceleratorMgrSensorEntry* sensorInfo;

    // Need to lock the list to prevent a reader accessing a deleted entry.
    kCheck(kLock_Enter(obj->sensors.lock));
    kTry
    {
        sensorCount = kArrayList_Count(obj->sensors.attachedSensors);
        for (i = 0; i < sensorCount; i++)
        {
            sensorInfo = kArrayList_AtT(obj->sensors.attachedSensors, i, GoAcceleratorMgrSensorEntry);
            if (sensorInfo->sensorId == sensorId)
            {
                kTest(kArrayList_Remove(obj->sensors.attachedSensors, i, kNULL));
                break;
            }
        }
    }
    kFinally
    {
        kCheck(kLock_Exit(obj->sensors.lock));

        kEndFinally();
    }

    return kOK;
}
#endif

// This function updates the sensor entry status and remote address location
// obtained from events sent by the remote process.
GoFx(kStatus) GoAcceleratorMgr_UpdateSensorEntryFromEvent(GoAcceleratorMgr manager, k32u sensorId, GoSensorAccelStatus newStatus, kIpEndPoint* remoteAddr)
{
    kObj(GoAcceleratorMgr, manager);
    kSize sensorCount;
    kSize i;
    GoAcceleratorMgrSensorEntry* entryPtr;

    // Need to lock the list to ensure reader won't read partially updated data.
    kCheck(kLock_Enter(obj->sensors.lock));

    sensorCount = kArrayList_Count(obj->sensors.attachedSensors);
    for (i = 0; i < sensorCount; i++)
    {
        entryPtr = kArrayList_AtT(obj->sensors.attachedSensors, i, GoAcceleratorMgrSensorEntry);
        if (entryPtr->sensorId == sensorId)
        {
            entryPtr->status = newStatus;

            if (remoteAddr)
            {
                entryPtr->eventRemoteAddr = *remoteAddr;
            }

            break;
        }
    }

    kCheck(kLock_Exit(obj->sensors.lock));

    return kOK;
}

// Check if acceleration by this host is in progress but not yet completed.
// This function looks inside the accelerated sensor list to see if the specified
// sensor is on the list or not. This function does not check if sensor has
// been accelerated by another host.
GoFx(kBool) GoAcceleratorMgr_IsSensorAcceleratedByLocalHost(GoAcceleratorMgr manager, k32u sensorId)
{
    kObj(GoAcceleratorMgr, manager);
    kSize sensorCount;
    kSize i;
    GoAcceleratorMgrSensorEntry* entryPtr;
    kBool isAccelerated = kFALSE;

    kCheck(kLock_Enter(obj->sensors.lock));

    sensorCount = kArrayList_Count(obj->sensors.attachedSensors);
    for (i = 0; i < sensorCount; i++)
    {
        entryPtr = kArrayList_AtT(obj->sensors.attachedSensors, i, GoAcceleratorMgrSensorEntry);
        if (entryPtr->sensorId == sensorId)
        {
            isAccelerated = kTRUE;
            break;
        }
    }

    kCheck(kLock_Exit(obj->sensors.lock));

    return isAccelerated;
}

// This function returns a pointer to the sensor info entry and its index.
// CAUTION:
// The list is not locked in this call because the typical use case
// for this function is that the caller has to do several things with the
// entry.
// So caller is responsible for locking this list before calling this function.
GoFx(kStatus) GoAcceleratorMgr_FindSensorEntry(GoAcceleratorMgr manager, k32u sensorId, kSize* index, GoAcceleratorMgrSensorEntry** sensorInfo)
{
    kObj(GoAcceleratorMgr, manager);
    kSize sensorCount;
    kSize i;
    GoAcceleratorMgrSensorEntry* entry;
    kStatus status = kOK;

    // Caller is responsible for locking the list.

    sensorCount = kArrayList_Count(obj->sensors.attachedSensors);
    for (i = 0; i < sensorCount; i++)
    {
        entry = kArrayList_AtT(obj->sensors.attachedSensors, i, GoAcceleratorMgrSensorEntry);
        if (entry->sensorId == sensorId)
        {
            if (!kIsNull(index))
            {
                *index = i;
            }
            if (!kIsNull(sensorInfo))
            {
                *sensorInfo = entry;
            }
            break;
        }
    }

    if (i == sensorCount)
    {
        // Sensor not found.
        status = kERROR_NOT_FOUND;
    }

    return status;
}

// This function gets a acceleration stop reason from the accelerated sensor list
// entry.
GoFx(kStatus) GoAcceleratorMgr_GetSensorStoppedReason(GoAcceleratorMgr manager, k32u sensorId, GoAcceleratorMgrStoppedReason* stoppedReason)
{
    kObj(GoAcceleratorMgr, manager);
    kSize sensorCount;
    kSize i;
    GoAcceleratorMgrSensorEntry* entry;
    kStatus status = kOK;

    kCheck(kLock_Enter(obj->sensors.lock));

    sensorCount = kArrayList_Count(obj->sensors.attachedSensors);
    for (i = 0; i < sensorCount; i++)
    {
        entry = kArrayList_AtT(obj->sensors.attachedSensors, i, GoAcceleratorMgrSensorEntry);
        if (entry->sensorId == sensorId)
        {
            *stoppedReason = entry->stoppedReason;
            break;
        }
    }

    if (i == sensorCount)
    {
        // Sensor not found.
        status = kERROR_NOT_FOUND;
    }

    kCheck(kLock_Exit(obj->sensors.lock));

    return status;
}

GoFx(kStatus) GoAcceleratorMgr_ReleaseSensorEntryObject(GoAcceleratorMgr manager, GoAcceleratorMgrSensorEntry* entry)
{
    kCheck(kDisposeRef(&entry->sensorProcess));

    return kOK;
}

GoFx(kStatus) GoAcceleratorMgr_ReleaseAllSensorEntryObject(GoAcceleratorMgr manager)
{
    kObj(GoAcceleratorMgr, manager);
    kSize sensorCount;
    kSize i;
    GoAcceleratorMgrSensorEntry* entry;

    // Need to lock the list to ensure reader won't read partially updated data.
    kCheck(kLock_Enter(obj->sensors.lock));

    kTry
    {
        sensorCount = kArrayList_Count(obj->sensors.attachedSensors);
        for (i = 0; i < sensorCount; i++)
        {
            entry = kArrayList_AtT(obj->sensors.attachedSensors, i, GoAcceleratorMgrSensorEntry);
            kTest(GoAcceleratorMgr_ReleaseSensorEntryObject(manager, entry));
        }
    }
    kFinally
    {
        kLock_Exit(obj->sensors.lock);

        kEndFinally();
    }

    return kOK;
}

// This function returns a handle to a sensor object and shares the sensor object
// to prevent it from being released, such as during discovery if the sensor
// disappears. By share a sensor of interest, then it is not necessary
// to lock the entire system state as this may cause deadlocks (eg. an operation
// that joins a thread which happens to be processing the sensor and needs to get the
// system state lock).
//
// ** This call _DOES NOT_ prevent the object from being modified!!
//
// Must call the corresponding UnshareSensor() to release the share, which may
// result in the sensor being released.
GoFx(kStatus) GoAcceleratorMgr_ShareSensor(GoAcceleratorMgr manager, k32u sensorId, GoSensor* sensor)
{
    kObj(GoAcceleratorMgr, manager);

    // Lock state first to ensure the following steps are atomic.
    kCheck(GoSystem_LockState(obj->system));

    kTry
    {
        kTest(GoSystem_FindSensorById(obj->system, sensorId, sensor));

        // The sharing prevents the object from being released because the object
        // knows there is a user currently using it. Once shared, the system
        // state lock is not needed anymore.
        kTest(kObject_Share(*sensor));
    }
    kFinally
    {
        // Don't log errors because this function is called quite often and
        // logging errors here would generate too many error logs.

        // Don't return an error if unlock fails. More important to get and
        // process the sensor object if sensor was found.
        GoSystem_UnlockState(obj->system);

        kEndFinally();
    }

    return kOK;
}

// This function must be called if the ShareSensor() was called to get and share
// a sensor object. That sensor object must be passed into this function.
// Only call this function with a locked sensor.
// ** DO NOT ** call this function with an unshared sensor object.
GoFx(kStatus) GoAcceleratorMgr_UnshareSensor(GoAcceleratorMgr manager, GoSensor sensor)
{
    // This might release the object if no other user is using it.
    // Otherwise, the object continues to exist, but with one less user sharing
    // it.
    kCheck(kObject_Destroy(sensor));

    return kOK;
}

// This function sends an event message to the given remote address.
GoFx(kStatus) GoAcceleratorMgr_SendEvent(GoAcceleratorMgr manager, kIpEndPoint* remoteAddr, GoAcceleratorEventMsg* eventMsg)
{
    kObj(GoAcceleratorMgr, manager);

    kCheck(kUdpClient_WriteTo(obj->threads.eventUdpClient, eventMsg, sizeof(*eventMsg), remoteAddr->address, remoteAddr->port, GO_ACCELERATOR_EVENT_OP_TIMEOUT));

    return kOK;
}

// This function waits forever for an event from any remote address. When an
// event is received, it constructs a worker thread message using
// the event information for the caller to use.
GoFx(kStatus) GoAcceleratorMgr_ReceiveEvent(GoAcceleratorMgr manager, GoAcceleratorMgrWorkerMsg* workerMsg)
{
    kObj(GoAcceleratorMgr, manager);
    GoAcceleratorEventMsg   msg;
    kSize                   bytesRead;
    kStatus                 exception;

    kTry
    {
        // Wait forever for any event from any accelerating process.
        // Save the remote address information.
        kTest(kUdpClient_ReadFrom(obj->threads.eventUdpClient, &workerMsg->data.eventData.remoteAddr, &msg, sizeof(msg), &bytesRead, kINFINITE));
        kTestTrue(bytesRead == sizeof(msg), kERROR_INCOMPLETE);

        workerMsg->command = ACCELERATOR_MGR_EVENT;
        workerMsg->sensorId = msg.sensorId;
        workerMsg->data.eventData.event = msg.event;

        kLogf("%s: event %#x", __FUNCTION__, msg.event);
    }
    kCatch(&exception)
    {
        kLogf("%s: error %d receiving events", __FUNCTION__, exception);

        kEndCatch(exception);
    }

    return kOK;
}

// This function does the actual validation that a sensor's software version
// is compatible. The check is the same one that the external GUI does, which is
// to require an exact match between the sensor firmwware and the
// SDK version. The sensor firmware version is matched against the SDK
// because the accelerator application is a SDK application.
// This is much more stringent than the acceleration process's check
// which only requires matching the major and minor version numbers between
// the host's Gocator firmwware version and the physical sensor firmware version.
GoFx(kStatus) GoAcceleratorMgr_ValidateSensorVersion(GoAcceleratorMgr manager, kVersion sdkVersion, kVersion sensorVersion)
{
    kStatus status = kOK;

    if (kVersion_Compare(sdkVersion, sensorVersion) != 0)
    {
        status = kERROR_VERSION;

        kLogf("%s: error mismatched firmware versions, accelerator: %#x, sensor: %#x",
            __FUNCTION__,
            sdkVersion,
            sensorVersion);
    }

    return status;
}

// This function validates the sensor is in an acceptable state:
// - currently not accelerated
// - and has compatible firmware version for the SDK.
GoFx(kStatus) GoAcceleratorMgr_ValidateSensorState(GoAcceleratorMgr manager, k32u sensorId)
{
    kObj(GoAcceleratorMgr, manager);
    kStatus     exception;
    GoSensor    sensor = kNULL;

    kCheck(GoAcceleratorMgr_ShareSensor(manager, sensorId, &sensor));

    kTry
    {
        kTest(GoAcceleratorMgr_ValidateSensorVersion(manager, GoSdk_Version(), 
                                                     GoSensor_FirmwareVersion(sensor)));

        kTestState(GoSensor_AccelOpMode(sensor) == GO_DISCOVERY_OP_MODE_STANDALONE);
    }
    kCatchEx(&exception)
    {
        kLogf("%s: error %d validating sensor", __FUNCTION__, exception);

        kEndCatchEx(exception);
    }
    kFinallyEx
    {
        GoAcceleratorMgr_UnshareSensor(manager, sensor);

        kEndFinallyEx();
    }

    return kOK;
}

// This function checks if the monitoring period has been reached.
// Instead of an exact match in time, allow monitoring to start if the
// current time is close enough to the scheduled time. This is to avoid
// sleeping for very short intervals of time.
GoFx(kBool) GoAcceleratorMgr_IsTimeToMonitorSensor(GoAcceleratorMgr manager, k64u nextMonitorTime)
{
    kObj(GoAcceleratorMgr, manager);
    kBool doMonitor = kFALSE;
    k64u  nowTime;

    nowTime = kTimer_Now();

    if (nowTime >= nextMonitorTime)
    {
        // Current time has past the scheduled monitor time.
        doMonitor = kTRUE;
    }
    else if ((nextMonitorTime - nowTime ) <= GO_ACCELERATOR_MGR_MONITOR_PERIOD_DELTA)
    {
        // Current time has not reached the next monitor time, but is close
        // enough.
        doMonitor = kTRUE;
    }

    return doMonitor;
}

// This function tests whether a sensor's acceleration processes has had
// enough time to start up and become active.
GoFx(kBool) GoAcceleratorMgr_AcceleratorShouldHaveStarted(GoAcceleratorMgr manager, GoAcceleratorMgrSensorEntry* srcEntry)
{
    kObj(GoAcceleratorMgr, manager);
    kBool shouldHaveStarted = kFALSE;

    if ((kTimer_Now() - srcEntry->lastStartTime) >= GO_ACCELERATOR_MGR_PROCESS_ALIVE_WAIT_TIME)
    {
        shouldHaveStarted = kTRUE;
    }

    return shouldHaveStarted;
}

// This function tries to accelerate the sensor entry again, if the sensor
// is present on the network and is running in standalone mode.
// NOTE: currently can't detect if the standalone sensor is reachable or not
// so this function may try to accelerate an unreachable sensor.
GoFx(kStatus) GoAcceleratorMgr_RestartAcceleration(GoAcceleratorMgr manager, GoAcceleratorMgrSensorEntry* srcEntry)
{
    kObj(GoAcceleratorMgr, manager);
    kStatus exception;
    GoSensor sensor = kNULL;
    GoAcceleratorMgrWorkerMsg msg;

    kLogf("%s: re-accelerate sensor %u", __FUNCTION__, srcEntry->sensorId);

    kTry
    {
        // Check if sensor is missing or not. The sensor discovery mechanism
        // is run periodically so there can be a lag in detecting sensor presence
        // or absence.
        if (kSuccess(GoAcceleratorMgr_ShareSensor(manager, srcEntry->sensorId, &sensor)))
        {
            // Sensor is on the network. The operational mode information comes
            // from the discovery mechanism which is run periodically, so it
            // possible that the mode has not yet been updated by the discovery
            // mechanism, even though the physical sensor already has reverted
            // to standalone mode (and is therefore available for acceleration).
            //
            // TODO: is there a way to eliminate sensors that are
            // not reachable on the network?
            if (GoSensor_AccelOpMode(sensor) == GO_DISCOVERY_OP_MODE_STANDALONE)
            {
                // Sensor is standalone. Try to accelerate this sensor.
                // Recreating a non-existent or dead accelerator process to
                // re-accelerate a sensor is a RESTART request, NOT a START request.
                // No need to clean up the accelerator process object here.
                // Worker thread is responsible for maintaining it.
                msg.command = ACCELERATOR_MGR_RESTART;
                msg.sensorId = srcEntry->sensorId;
                msg.data.param.ports = srcEntry->portInfo;
                msg.data.param.platformIpAddress = srcEntry->platformIpAddress;
                kTest(kMsgQueue_AddT(obj->threads.workerQueue, &msg));
            }
            else
            {
                kLogf("%s: cannot re-accelerate sensor %u - op mode (%d) not standalone",
                    __FUNCTION__,
                    srcEntry->sensorId,
                    GoSensor_AccelOpMode(sensor));
            }
        }
        else
        {
            kLogf("%s: cannot re-accelerate sensor %u - missing",
                __FUNCTION__,
                srcEntry->sensorId);
        }
    }
    kCatchEx(&exception)
    {
        kLogf("%s: error %d restarting acceleration process", __FUNCTION__, exception);

        kEndCatchEx(exception);
    }
    kFinallyEx
    {
        if (!kIsNull(sensor))
        {
            GoAcceleratorMgr_UnshareSensor(manager, sensor);
        }

        kEndFinallyEx();
    }

    return kOK;
}

// This function checks if acceleration has stopped unexpectedly when
// acceleration is expected to be in progress.
// Use cases include:
// - acceleration procedure stopped early before attempting to start the
//   acceleration process. In this case, the status was already set to
//   Stopped by the worker thread so this function does not need to set it
//   to stop.
// - acceleration process started but ended prematurely, either because it
//   encountered an error during start up or it died/got killed/etc.
// - acceleration process terminated gracefully by using the kStudio Reset/Restart Software
//   operation.
GoFx(kBool) GoAcceleratorMgr_IsAccelerationProcessStopped(GoAcceleratorMgr manager, GoAcceleratorMgrSensorEntry* srcEntry)
{
    kObj(GoAcceleratorMgr, manager);
    kBool stopped = kFALSE;

    // If sensor should be accelerated, check that the acceleration process
    // is present.
    if (GoAcceleratorMgr_ShouldBeAccelerated(srcEntry->cmdInProgress))
    {
        // Look for sensors which had an accelerator process object created,
        // but the underlying process has stopped/exited/killed/terminated.
        // Don't use check for a null process object handle as an indication
        // that acceleration stopped because a sensor in the process of being
        // accelerated might not have an object handle created yet.
        //
        // TODO: why would the object not exist yet since the sensor is checked
        // only after having waited a fixed period of time from the start
        // operation before monitoring the sensor?
        if (srcEntry->sensorProcess)
        {
            if (!kProcess_IsAlive(srcEntry->sensorProcess))
            {
                // Acceleration process terminated for whatever reason.
                stopped = kTRUE;
            }
            else
            {
                // Process is alive. Clear the raise event flag so that the next time
                // the process terminates, a process stop event is generated.
                srcEntry->raisedStopFlag = kFALSE;
            }
        }
    }

    return stopped;
}

// This function monitors the health of each accelerated sensor's acceleration process.
// The check is limited to checking if it is running or not for each sensor on the
// accelerated sensor list.
// If an acceleration process terminates,  then this function triggers an acceleration
// request to recreate the accelerator process to accelerate the sensor.
//
// Note: The accelerator process object is not created if the acceleration
//       request failed in the worker thread prior to creating the object.
// Note: The accelerator process may have terminated if it could not accelerate the
//       sensor.
// Note: the accelerator process can be gracefully terminated by another entity
//       besides the parent process. Currently, kStudio is the only other entity.
// Note: Clean up of the process object should be done within the worker thread
//       to avoid having multiple threads update the sensor entry.
GoFx(kStatus) GoAcceleratorMgr_CheckSensorAcceleration(GoAcceleratorMgr manager)
{
    kObj(GoAcceleratorMgr, manager);
    kStatus     exception;
    kSize       sensorCount;
    kSize       i;
    GoAcceleratorMgrSensorEntry* srcEntry;

    // Must always have exclusive access to the list of attached sensors.
    kCheck(kLock_Enter(obj->sensors.lock));

    kTry
    {
        sensorCount = kArrayList_Count(obj->sensors.attachedSensors);

        for (i = 0; i < sensorCount; i++)
        {
            srcEntry = kArrayList_AtT(obj->sensors.attachedSensors, i, GoAcceleratorMgrSensorEntry);

            // Only check the sensors whose acceleration process has had enough
            // time to become active. If not enough time has elapsed for the
            // acceleration process to run, then the following check can return
            // a false indication that acceleration stopped.
            if (GoAcceleratorMgr_AcceleratorShouldHaveStarted(manager, srcEntry))
            {
                if (GoAcceleratorMgr_IsAccelerationProcessStopped(manager, srcEntry))
                {
                    kLogf("%s: sensor %u stopped accelerating", __FUNCTION__, srcEntry->sensorId);

                    // Dead accelerator process.
                    // Note that sensor eventually will revert back to
                    // standalone mode. No acceleration is happening during
                    // this time.
                    srcEntry->status = GO_SENSOR_ACCEL_STATUS_STOPPED;
                }

                // Restart any stopped sensor.
                // Only stopped sensors are restarted.
                if (srcEntry->status == GO_SENSOR_ACCEL_STATUS_STOPPED)
                {
                    // Raise this condition once per process death.
                    // Raising the event here handles all cases where the
                    // process terminated, both gracefully (eg. process code
                    // exited through an error path) or forcibly (eg. process killed
                    // by user or operating system).
                    if (!srcEntry->raisedStopFlag)
                    {
                        kTest(GoAcceleratorMgr_SendEventToWorkerThread(manager, srcEntry->sensorId, GO_ACCELERATOR_EVENT_PROCESS_STOPPED));

                        kLogf("%s: raise stop event for sensor %u", __FUNCTION__, srcEntry->sensorId);

                        srcEntry->raisedStopFlag = kTRUE;
                    }

                    // Clean up the process object since there is no running
                    // process associated with it.
                    // Don't let error stop the loop.
                    kDestroyRef(&srcEntry->sensorProcess);

                    // Don't let error stop the loop. Continue going through
                    // all the sensors.
                    GoAcceleratorMgr_RestartAcceleration(manager, srcEntry);
                }
            }
        }
    }
    kCatchEx(&exception)
    {
        kLogf("%s: error %d scanning sensors", __FUNCTION__, exception);

        kEndCatchEx(exception);
    }
    kFinallyEx
    {
        kLock_Exit(obj->sensors.lock);

        kEndFinallyEx();
    }

    return kOK;
}

// This function checks if the restored sensor's configured ports are available
// or not on the netowrk.
// Note that passing the check does not guarantee the ports will still be
// available when they are needed later on.
GoFx(kStatus) GoAcceleratorMgr_CheckRestoredSensorPortAvailable(GoAcceleratorMgr manager, GoAcceleratorMgrWorkerMsg* msg, GoAcceleratorMgrSensorEntry* sensorEntryPtr)
{
    kObj(GoAcceleratorMgr, manager);
    kStatus status = kOK;

    if ((msg->command == ACCELERATOR_MGR_RESTORE_START) || (msg->command == ACCELERATOR_MGR_RESTART))
    {
        // Port availability check is usually done before the request enters the
        // worker thread to to reject new requests.
        // However, when restoring sensors from configuration, the sensors
        // *MUST* be added back to the accelerated sensor list, which implies
        // the request *MUST* be accepted into the worker thread first.
        // So the port free check here is for restored sensors.
        // Need to skip the allocation check because the ports were added
        // to the allocator already.
        if (!GoAccelSensorPortAlloc_UserSelectedPortsFree(obj->portAlloc, &msg->data.param.ports, kFALSE))
        {
            kLogf("%s: port in use detected for sensor %u", __FUNCTION__, msg->sensorId);

            GoAcceleratorMgr_SetStoppedReason(manager, sensorEntryPtr, ACCELERATOR_MGR_ACCEL_STOPPED_PORT_IN_USE);

            // If event generation succeeded, return with an error because
            // ports are not free.
            status = kERROR_NETWORK;
        }
        else
        {
            // Clear the stop reason now that the ports are available.
            GoAcceleratorMgr_ClearStoppedReason(manager, sensorEntryPtr);
        }
    }

    return status;
}

// This function is called if the acceleration status indicates the accelerator
// process is not running (ie. stopped) for whatever reason.
// This function tries to determine a more specific reason for why the process
// is not running. This is done by querying the sensor object to see what its
// status is, and checking the stopped reason field in the sensor list.
// The more detailed reason is for consumption by the external client only.
// The SDK does not use it. SDK just needs/uses the generic STOPPED status.
GoFx(GoSensorAccelStatus) GoAcceleratorMgr_RefineAccelStoppedStatus(GoAcceleratorMgr manager, k32u sensorId)
{
    kObj(GoAcceleratorMgr, manager);
    kStatus     status;
    GoSensor    sensor = kNULL;
    GoSensorAccelStatus accelStatus = GO_SENSOR_ACCEL_STATUS_STOPPED;
    GoSensorAccelState  accelState;
    GoAcceleratorMgrStoppedReason stoppedReason = ACCELERATOR_MGR_ACCEL_STOPPED_NONE;

    // First check for system issue.
    if (kSuccess(GoAcceleratorMgr_GetSensorStoppedReason(manager, sensorId, &stoppedReason)))
    {
        if (stoppedReason == ACCELERATOR_MGR_ACCEL_STOPPED_PORT_IN_USE)
        {
            accelStatus = GO_SENSOR_ACCEL_STATUS_STOPPED_PORT_IN_USE;
        }
        else if (stoppedReason == ACCELERATOR_MGR_ACCEL_STOPPED_UNREACHABLE)
        {
            accelStatus = GO_SENSOR_ACCEL_STATUS_STOPPED_UNREACHABLE;
        }
    }

    // If acceleration status is still not refined, check the sensor object.
    if (accelStatus == GO_SENSOR_ACCEL_STATUS_STOPPED)
    {
        status = GoAcceleratorMgr_ShareSensor(manager, sensorId, &sensor);
        if (status == kERROR_NOT_FOUND)
        {
            // Sensor is just plain missing. Can't get any more specific reason
            // why accelerating stopped.
            accelStatus = GO_SENSOR_ACCEL_STATUS_MISSING;
        }
        else if (kSuccess(status) && !kIsNull(sensor))
        {
            accelState = GoSensor_AccelState(sensor);
            switch (accelState)
            {
            case GO_SENSOR_ACCEL_STATE_AVAILABLE:
                accelStatus = GO_SENSOR_ACCEL_STATUS_STOPPED_AVAILABLE;
                break;
            case GO_SENSOR_ACCEL_STATE_ACCELERATED:
                // Sensor either:
                // - has been taken over by accelerator host, or
                // - was accelerated by this host but the process died.
                // If accelerated by this host, then the status should just be
                // STOPPED.
                if (!GoAcceleratorMgr_IsSensorAcceleratedByLocalHost(manager, sensorId))
                {
                    accelStatus = GO_SENSOR_ACCEL_STATUS_STOPPED_ACCELERATED_BY_OTHER;
                }
                break;
            case GO_SENSOR_ACCEL_STATE_FW_MISMATCH:
                accelStatus = GO_SENSOR_ACCEL_STATUS_STOPPED_FW_MISMATCH;
                break;
            default:
                kLogf(
                    "%s: unexpected sensor accelerated state %u while accelerator process stopped",
                    __FUNCTION__,
                    accelState);

                // Set status to the generic stopped status by default.
                accelStatus = GO_SENSOR_ACCEL_STATUS_STOPPED;
                break;
            }

            // Don't care about return code. Want to return the acceleration status.
            (void) GoAcceleratorMgr_UnshareSensor(manager, sensor);
        }
    }

    return accelStatus;
}

// This function determines if a command means a sensor should be accelerated
// or not.
GoFx(kBool) GoAcceleratorMgr_ShouldBeAccelerated(GoAcceleratorMgrWorkerCmd cmd)
{
    kBool shouldBeAccelerated;

    switch (cmd)
    {
    case ACCELERATOR_MGR_START:
    case ACCELERATOR_MGR_RESTART:
    case ACCELERATOR_MGR_RESTORE_START:
        shouldBeAccelerated = kTRUE;
        break;
    default:
        shouldBeAccelerated = kFALSE;
        break;
    }

    return shouldBeAccelerated;
}

/**
 * @file    GoAcceleratorMgr.x.h
 *
 * Header file for the GoAcceleratorMgr class's private declarations.
 *
 * @internal
 * Copyright (C) 2018-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef GO_ACCELERATOR_MGR_X_H
#define GO_ACCELERATOR_MGR_X_H

#include <kApi/Utils/kProcess.h>
#include <kApi/Data/kArrayList.h>

kDeclareValueEx(Go, GoAcceleratorMgrWorkerMsg, kValue)
kDeclareValueEx(Go, GoAcceleratorMgrSensorEntry, kValue)
kDeclareValueEx(Go, GoAcceleratorMgrSensorInfo, kValue)

// This definition needed only because of SDKNET wrapper definitions.
kDeclareValueEx(Go, GoAcceleratorMgrAccelUpdate, kValue)
kDeclareValueEx(Go, GoAcceleratorMgrSensorParam, kValue)
kDeclareValueEx(Go, GoAcceleratorMgrSensorBackup, kValue)
kDeclareEnumEx(Go, GoAcceleratorMgrAccelEvents, kValue)

// Name of the accelerator process executable.
// Firesync will append an appropriate suffix for the executable depending
// on what platform this code is running on.
#define GO_ACCELERATOR_MGR_ACCEL_PROCESS_EXECUTABLE "GoXProcess"

// Time to wait for a graceful termination of acceleration/child process.
// The acceleration process start up takes a finite amount of time to complete
// before reaching the point where it checks for graceful termination.
// So make this timeout long enough to allow the acceleration process to
// reach the point where it waits for graceful termination.
// Set to 15 seconds. After this timeout, a forced termination is triggered.
#define GO_ACCELERATOR_MGR_TIMEOUT_GRACEFUL_TERMINATION     15000000

// Timeouts for waiting for internal threads to terminate. Set to 2 seconds.
#define GO_ACCELERATOR_MGR_TIMEOUT_THREAD_JOIN              2000000

// How often to check the health of sensors, in microseconds. Set to 5 sec.
#define GO_ACCELERATOR_MGR_MONITOR_PERIOD                   5000000

// How close the current time is to the expected next health monitoring time
// should a monitor scan be started. Units in microseconds. Set to 100 ms.
#define GO_ACCELERATOR_MGR_MONITOR_PERIOD_DELTA             100000

// This time specifies how long is delay between initiating the acceleration
// procedure and when the acceleration process should be alive and running.
// This wait time is to allow the acceleration process an opportunity
// to run before a health check is performed to see if the process
// is running or not.
// If this delay is too short, false acceleration failure is reported.
// Units in microseconds. Set to 1 sec.
#define GO_ACCELERATOR_MGR_PROCESS_ALIVE_WAIT_TIME          1000000

typedef enum GoAcceleratorMgrWorkerCmd
{
    ACCELERATOR_MGR_NO_CMD = 0,
    ACCELERATOR_MGR_START,
    ACCELERATOR_MGR_STOP,
    ACCELERATOR_MGR_RESTORE_START,
    ACCELERATOR_MGR_RESTART,
    ACCELERATOR_MGR_EVENT
} GoAcceleratorMgrWorkerCmd;

// This enumeration contains reasons that a sensor has stopped acceleration,
// but whose reasons cannot be determined by checking the sensor object status.
// Reasons like firmware mismatch or accelerated by another host can be
// determined from the sensor object, so are not included in this list.
// The reasons typically reflect system issues instead of sensor specific issues.
typedef enum GoAcceleratorMgrStoppedReason
{
    ACCELERATOR_MGR_ACCEL_STOPPED_NONE = 0,
    ACCELERATOR_MGR_ACCEL_STOPPED_PORT_IN_USE,
    ACCELERATOR_MGR_ACCEL_STOPPED_UNREACHABLE
} GoAcceleratorMgrStoppedReason;

// Structure for an event received from remote process.
// Fields:
//  event:      Event identifier.
//  remoteAddr: Socket information about the event sender.
typedef struct GoAcceleratorMgrEventData
{
    GoAcceleratorEventType  event;
    kIpEndPoint             remoteAddr;
} GoAcceleratorMgrEventData;

// Structure for the messages sent to the worker thread.
// Fields:
//  command:        Specifies the work the worker thread should do.
//  sensorId:       Identifier of the sensor device.
//  param:          Parameters given to the accelerator manager by SDK client.
//  eventData:      Data associated with an event.
typedef struct GoAcceleratorMgrWorkerMsg
{
    GoAcceleratorMgrWorkerCmd   command;
    k32u                        sensorId;
    union
    {
        GoAcceleratorMgrSensorParam param;
        GoAcceleratorMgrEventData   eventData;
    } data;
} GoAcceleratorMgrWorkerMsg;

// TODO: Change this into a class to simplify cleanup. This structure contains
// a handle to a kProcess object that needs to be freed whenever a sensor entry
// removed from the sensor list.
//
// Structure of information about an accelerated sensor.
// Fields:
//  sensorId:       Identifier of the sensor.
//  cmdInProgress:  The desired operation to apply to the sensor.
//  status:         Acceleration status.
//  platformIpAddress: The accelerator platform's interface IP address to which an
//                     accelerated sensor is bound. This is needed if the
//                     platform has more than one interface through which the
//                     platform's accelerator virtual sensor can be accessed.
//  portInfo:       Ports used by the accelerated sensor.
//  backup:         Holds original sensor object state that needs to be
//                  changed when sensor is accelerated. When sensor is no longer
//                  accelerated, the sensor object state is restored back to
//                  its original values using data stored in this field.
//  sensorProcess:  Object handle to the child process on the platform that is
//                  running the accelerated sensor's processing code (Ie. the
//                  virtual sensor).
//  eventRemoteAddr: Socket of the child/accelerating process used to communicate events.
//                   The acceleration manager (server side) needs this to send
//                   events to the child/accelerating process.
//  lastStartTime:  timestamp of when acceleration was attempted for this sensor.
//  stoppedReason:  reason why sensor acceleration stopped.
//  raisedStopFlag: flag to indicate the if acceleration stopped (because
//                  process is not running) event has been raised or not.
//                  This flag is used to prevent continuously raising
//                  the event if sensor can never be accelerated, such as if
//                  sensor is in a acceleration start up loop
//                  (eg. sensor subnet on an unreachable subnet).
typedef struct GoAcceleratorMgrSensorEntry
{
    k32u                            sensorId;
    GoAcceleratorMgrWorkerCmd       cmdInProgress;
    GoSensorAccelStatus             status;
    kIpAddress                      platformIpAddress;
    GoAccelSensorPortAllocPorts     portInfo;
    GoAcceleratorMgrSensorBackup    backup;
    kProcess                        sensorProcess;
    kIpEndPoint                     eventRemoteAddr;
    k64u                            lastStartTime;
    GoAcceleratorMgrStoppedReason   stoppedReason;
    kBool                           raisedStopFlag;
} GoAcceleratorMgrSensorEntry;

// Structure to hold operational functionality options that the accelerator
// manager can support and be configured with. Having this option block
// hopefully avoids having to change the constructor signature in the future,
// while allowwing SDK client to enable or disable an operational functionality.
// Fields:
//  autoRecoverSensor:  Accelerator manager automatically recreates accelerated sensor process
//                      if it the process is not running.
typedef struct GoAcceleratorMgrOptions
{
//    kBool monitorSensor;
//    kBool asyncOperation;
//    kBool persistSensorLog;
    kBool autoRecoverSensor;
} GoAcceleratorMgrOptions;

// Structure to provide a convenient aggregation of thread related structures
// used by the accelerator manager.
// Fields:
//  workerThread:       Thread handle for the main worker thread.
//  workerQueue:        Message queue through which requestes are sent to the worker
//                      thread.
//  eventThread:        Thread handle for the thread that receives events from
//                      all the accelerator child processes.
//  eventUdpClient:     This process serves as the event server listening for
//                      events from the acceleration (child) processes, and
//                      for sending termination events to the child processes
//                      for graceful termination. This field is the handle
//                      for a UDP client connection.
//  callbackThread:     Thread handle for the thread dedicated to running
//                      SDK application's update handler. A dedicated thread is
//                      is used to avoid blocking the internal working threads if
//                      the callback makes blocking calls or runs for a long time.
//  callbackQueue:      Message queue through which requestes are sent to the
//                      callback thread.
typedef struct GoAcceleratorMgrThreadBlock
{
    kThread     workerThread;
    kMsgQueue   workerQueue;

    kThread     eventThread;
    kUdpClient  eventUdpClient;

    kThread     callbackThread;
    kMsgQueue   callbackQueue;

//    kThread     monitorThread;
//    kThread sensorLoggerThread;
} GoAcceleratorMgrThreadBlock;

// Structure to manage the list of accelerated sensors.
// Fields:
//  attachedSensors:    List of sensors configured for acceleration.
//  lock:               Lock to provide mutual exclusion and thread safety when
//                      accessing the sensor list.
typedef struct GoAcceleratorMgrSensorListBlock
{
    kArrayList  attachedSensors;   // GoAcceleratorMgrSensorEntry
    kLock       lock;
} GoAcceleratorMgrSensorListBlock;

// Structure to manage the acceleration event update callback.
// Fields:
//  onAccelUpdate:  Callback bound in by SDK user.
//  lock:           Lock to provide mutual exclusion when accessing the callback.
typedef struct GoAcceleratorMgrCallbackBlock
{
    kCallback   onAccelUpdate;
    kLock       lock;
} GoAcceleratorMgrCallbackBlock;

typedef struct GoAcceleratorMgrClass
{
    kObjectClass base;

    kBool       isRunning;
    GoSystem    system;
    GoAcceleratorMgrOptions     options;
    GoAcceleratorMgrThreadBlock threads;

    GoAcceleratorMgrCallbackBlock   updateCallback;

    GoAcceleratorMgrSensorListBlock sensors;
    GoAccelSensorPortAlloc          portAlloc;
} GoAcceleratorMgrClass;

kDeclareClassEx(Go, GoAcceleratorMgr, kObject)


GoFx(kStatus) GoAcceleratorMgr_Init(GoAcceleratorMgr accelerator, kType type, kAlloc alloc);
GoFx(kStatus) GoAcceleratorMgr_VRelease(GoAcceleratorMgr accelerator);

GoFx(kStatus) GoAcceleratorMgr_InitSensorBlock(GoAcceleratorMgr manager, kAlloc alloc);
GoFx(kStatus) GoAcceleratorMgr_ReleaseSensorBlock(GoAcceleratorMgr manager);
GoFx(kStatus) GoAcceleratorMgr_CreateUdpClient(GoAcceleratorMgr manager, kAlloc alloc);
GoFx(kStatus) GoAcceleratorMgr_InitThreadBlock(GoAcceleratorMgr manager, kAlloc alloc);
GoFx(kStatus) GoAcceleratorMgr_ReleaseThreadBlock(GoAcceleratorMgr manager);

GoFx(kBool) GoAcceleratorMgr_ValidateSensorParam(GoAcceleratorMgr manager, GoAcceleratorMgrSensorParam* param);
GoFx(kStatus) GoAcceleratorMgr_ValidateSensorVersion(GoAcceleratorMgr manager, kVersion sdkVersion, kVersion sensorVersion);
GoFx(kStatus) GoAcceleratorMgr_ValidateSensorState(GoAcceleratorMgr manager, k32u sensorId);
GoFx(void) GoAcceleratorMgr_BackupSensorInfo(GoAcceleratorMgr manager, GoSensor sensor, GoAcceleratorMgrSensorEntry* sensorInfo);
GoFx(void) GoAcceleratorMgr_InitSensorInfo(GoAcceleratorMgr manager, GoAcceleratorMgrWorkerMsg* msg, GoAcceleratorMgrSensorEntry* sensorInfo);
GoFx(kStatus) GoAcceleratorMgr_PointSensorObjectToLocalHost(GoAcceleratorMgr manager, GoAcceleratorMgrSensorEntry* sensorInfo, GoSensor sensor);
GoFx(kStatus) GoAcceleratorMgr_PointSensorObjectToDevice(GoAcceleratorMgr manager, GoAcceleratorMgrSensorEntry* sensorInfo, GoSensor sensor);

GoFx(kStatus) GoAcceleratorMgr_SetupProcessArguments(GoAcceleratorMgr manager, GoAcceleratorMgrSensorEntry* sensorInfo);
GoFx(kStatus) GoAcceleratorMgr_StartAccelerationProcess(GoAcceleratorMgr manager, GoAcceleratorMgrSensorEntry* sensorInfo);
GoFx(kStatus) GoAcceleratorMgr_WorkerStartAcceleration(GoAcceleratorMgr manager, GoAcceleratorMgrWorkerMsg* msg);

GoFx(kStatus) GoAcceleratorMgr_MarkSensorAvailable(GoAcceleratorMgr manager, GoSensor sensor);
GoFx(kStatus) GoAcceleratorMgr_TerminateGraceful(GoAcceleratorMgr manager, GoAcceleratorMgrSensorEntry* sensorInfo);
GoFx(kStatus) GoAcceleratorMgr_SendEventToWorkerThread(GoAcceleratorMgr manager, k32u sensorId, GoAcceleratorMgrAccelEvents accelEvent);
GoFx(kStatus) GoAcceleratorMgr_WorkerStopAcceleration(GoAcceleratorMgr manager, GoAcceleratorMgrWorkerMsg* msg);
GoFx(kStatus) GoAcceleratorMgr_WorkerHandleEvent(GoAcceleratorMgr manager, GoAcceleratorMgrWorkerMsg* msg);

GoFx(kStatus) kCall GoAcceleratorMgr_WorkerThread(GoAcceleratorMgr manager);
GoFx(kStatus) kCall GoAcceleratorMgr_EventThread(GoAcceleratorMgr manager);
GoFx(kStatus) kCall GoAcceleratorMgr_CallbackThread(GoAcceleratorMgr manager);

GoFx(kStatus) GoAcceleratorMgr_AddNewSensorToList(GoAcceleratorMgr manager, GoAcceleratorMgrWorkerMsg* msg);
GoFx(kStatus) GoAcceleratorMgr_AddSensorEntry(GoAcceleratorMgr manager, GoAcceleratorMgrSensorEntry* sensorInfo);
GoFx(kStatus) GoAcceleratorMgr_DeleteSensorEntryAt(GoAcceleratorMgr manager, kSize index);
GoFx(kSize) GoAcceleratorMgr_SensorEntryCount(GoAcceleratorMgr manager);
//GoFx(kStatus) GoAcceleratorMgr_DeleteSensorEntry(GoAcceleratorMgr manager, k32u sensorId);
GoFx(kStatus) GoAcceleratorMgr_UpdateSensorEntryFromEvent(GoAcceleratorMgr manager, k32u sensorId, GoSensorAccelStatus newStatus, kIpEndPoint* remoteAddr);
GoFx(kBool) GoAcceleratorMgr_IsSensorAcceleratedByLocalHost(GoAcceleratorMgr manager, k32u sensorId);
GoFx(kStatus) GoAcceleratorMgr_FindSensorEntry(GoAcceleratorMgr manager, k32u sensorId, kSize* index, GoAcceleratorMgrSensorEntry** sensorInfo);
GoFx(void) GoAcceleratorMgr_UpdateSensorInfoAfterAccel(GoAcceleratorMgr manager, GoAcceleratorMgrSensorEntry* sensorInfo);
GoFx(void) GoAcceleratorMgr_UpdateSensorLastStartTime(GoAcceleratorMgr manager, GoAcceleratorMgrSensorEntry* sensorInfo);

GoFx(kStatus) GoAcceleratorMgr_ReleaseSensorEntryObject(GoAcceleratorMgr manager, GoAcceleratorMgrSensorEntry* entry);
GoFx(kStatus) GoAcceleratorMgr_ReleaseAllSensorEntryObject(GoAcceleratorMgr manager);

GoFx(kStatus) GoAcceleratorMgr_ShareSensor(GoAcceleratorMgr manager, k32u sensorId, GoSensor* sensor);
GoFx(kStatus) GoAcceleratorMgr_UnshareSensor(GoAcceleratorMgr manager, GoSensor sensor);

GoFx(kStatus) GoAcceleratorMgr_ReceiveEvent(GoAcceleratorMgr manager, GoAcceleratorMgrWorkerMsg* workerMsg);
GoFx(kStatus) GoAcceleratorMgr_SendEvent(GoAcceleratorMgr manager, kIpEndPoint* remoteAddr, GoAcceleratorEventMsg* eventMsg);

GoFx(kStatus) GoAcceleratorMgr_StopEventThread(GoAcceleratorMgr manager);
GoFx(kStatus) GoAcceleratorMgr_Stop(GoAcceleratorMgr manager);

GoFx(kStatus) GoAcceleratorMgr_CheckSensorAcceleration(GoAcceleratorMgr manager);
GoFx(kBool) GoAcceleratorMgr_IsAccelerationProcessStopped(GoAcceleratorMgr manager, GoAcceleratorMgrSensorEntry* srcEntry);
GoFx(kStatus) GoAcceleratorMgr_RestartAcceleration(GoAcceleratorMgr manager, GoAcceleratorMgrSensorEntry* srcEntry);
GoFx(kBool) GoAcceleratorMgr_AcceleratorShouldHaveStarted(GoAcceleratorMgr manager, GoAcceleratorMgrSensorEntry* srcEntry);
GoFx(kBool) GoAcceleratorMgr_IsTimeToMonitorSensor(GoAcceleratorMgr manager, k64u nextMonitorTime);

GoFx(kStatus) GoAcceleratorMgr_CheckRestoredSensorPortAvailable(GoAcceleratorMgr manager, GoAcceleratorMgrWorkerMsg* msg, GoAcceleratorMgrSensorEntry* sensorEntryPtr);
GoFx(void) GoAcceleratorMgr_SetStoppedReason(GoAcceleratorMgr manager, GoAcceleratorMgrSensorEntry* sensorInfo, GoAcceleratorMgrStoppedReason stoppedReason);
GoFx(void) GoAcceleratorMgr_ClearStoppedReason(GoAcceleratorMgr manager, GoAcceleratorMgrSensorEntry* sensorInfo);
GoFx(kStatus) GoAcceleratorMgr_GetSensorStoppedReason(GoAcceleratorMgr manager, k32u sensorId, GoAcceleratorMgrStoppedReason* stoppedReason);
GoFx(GoSensorAccelStatus) GoAcceleratorMgr_RefineAccelStoppedStatus(GoAcceleratorMgr manager, k32u sensorId);
GoFx(kBool) GoAcceleratorMgr_ShouldBeAccelerated(GoAcceleratorMgrWorkerCmd cmd);

GoFx(const kChar*) GoAcceleratorMgr_GetCommandName(GoAcceleratorMgrWorkerCmd cmd);
GoFx(const kChar*) GoAcceleratorMgr_GetEventName(GoAcceleratorEventType event);

/**
* Get the acceleration status of the sensor that was accelerated. If the
* GoAcceleratorMgr_Accelerate() API was never called for the sensor,
* then this API returns an kERROR_NOT_FOUND.
* Used for unit testing.
*
* @public                 @memberof GoAcceleratorMgr
* @param   manager        GoAcceleratorMgr object.
* @param   sensorId       Identifier of the sensor.
* @param   status         Acceleration status of the sensor returned to the caller.
* @return                 Operation status.
*/
GoFx(kStatus) GoAcceleratorMgr_AccelerationStatus(GoAcceleratorMgr manager, k32u sensorId, GoSensorAccelStatus* status);

/**
* Restore a sensor to accelerate state. This function should only be used to
* accelerate a sensor that was successfully accelerated in the past with
* GoAcceleratorMgr_Accelerate(). This function is intended to be used when
* accelerate a sensor from a persistent configuration following a system boot
* up. The persistent configuration is assumed to contain valid sensor parameters
* obtained from the original call to GoAcceleratorMgr_Accelerate().
*
* WARNING: USE THIS API ONLY TO RESTORE SENSORS THAT WERE ALREADY CONFIGURED
* FOR ACCELERATION BEFORE.
* For new requests, use GoAcceleratorMgr_Accelerate() instead.
*
* @public                 @memberof GoAcceleratorMgr
* @param   manager        GoAcceleratorMgr object.
* @param   sensorId       Identifier of the sensor.
* @param   param          Structure of parameters to use to accelerate the sensor.
* @return                 Operation status.
*/
GoFx(kStatus) GoAcceleratorMgr_RestoreAccelerate(GoAcceleratorMgr manager, k32u sensorId, GoAcceleratorMgrSensorParam* param);

#endif  // GO_ACCELERATOR_MGR_X_H

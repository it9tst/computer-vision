/**
 * @file    GoSensor.x.h
 *
 * @internal
 * Copyright (C) 2016-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef GO_SDK_SENSOR_X_H
#define GO_SDK_SENSOR_X_H

#include <GoSdk/Internal/GoControl.h>
#include <GoSdk/Internal/GoDiscovery.h>
#include <GoSdk/Internal/GoReceiver.h>
#include <GoSdk/GoReplay.h>
#include <GoSdk/Tools/GoTools.h>
#include <GoSdk/GoTransform.h>
#include <kApi/Threads/kLock.h>
#include <kApi/Data/kArrayList.h>

#define GO_SENSOR_CONFIG_SCHEMA_VERSION             (101)
#define GO_SENSOR_LIVE_CONFIG_NAME                  "_live.cfg"

#define GO_SENSOR_TRANSFORM_SCHEMA_VERSION          (100)
#define GO_SENSOR_LIVE_TRANSFORM_NAME               "_live.tfm"

#define GO_SENSOR_JOB_VERSION                       (101)

#define GO_SENSOR_CONSISTENCY_INTERVAL              (4000000)           //hold time, after foreground info is updated,
                                                                        //before consistency with background info is checked (us)

#define GO_SENSOR_UPGRADE_TIMEOUT                   (120000000)         //timeout for sensor upgrade operations (us)

#define GO_SENSOR_DATA_SOCKET_BUFFER                (65536)             //socket read buffer size for data connections
#define GO_SENSOR_DATA_STREAM_BUFFER                (65536)             //client read buffer size for data connections

#define GO_SENSOR_HEALTH_SOCKET_BUFFER              (16384)             //socket read buffer size for health connections
#define GO_SENSOR_HEALTH_STREAM_BUFFER              (16384)             //client read buffer size for health connections

#define GO_SENSOR_HEALTH_PORT                       (3194)              //gocator health port number
#define GO_SENSOR_DATA_PORT                         (3196)              //gocator data port number

#define GO_SENSOR_DISCOVERY_CHECK_COUNT             (3)                 //number of failed discovery checks before sensor considered absent
#define GO_SENSOR_HEALTH_CHECK_COUNT                (3)                 //number of failed health checks before sensor considered absent

#define GO_FILE_GEOCAL                              "GeoCal.xml"

// This structure contains information related to acceleration of sensor.
// The fields require getting additional information about the
// main controller/host that is not available from sensor discovery
// itself.
// It is possible to get some of the information in this structure
// by connecting to the sensor to get its local information, but it is not
// possible to connect to a sensor accelerated by another host (since that host
// returns a localhost (127.0.0.1) IP address for the accelerated sensor.
// So instead, get all the needed information from the discovery protocol's
// extended information as this bypasses requirement to connect to the sensor.
// Side effect is that some information in this structure is duplicated by other
// fields in the class, after SDK connects to the sensor.
//
// Fields:
//  opMode          - Operational mode of main controller.
//  state           - Sensor acceleration state.
//  version         - software version the discovery server is running.
//  model           - model name of the sensor.
//  portInfo        - Ports used by accelerator. Sensor class doesn't have
//                    a field for web port but discovery protocol will give that.
//                    GoSensor object doesn't update its other port fields until sensor object is
//                    is connected the sensor itself. So this field provides port information
//                    without having to connect to the sensor.
//                    For sensors accelerated by another host, it is tricky connecting
//                    SDK sensor object to the accelerated sensor.
//  accelSensorIp   - IP address of sensor (same as the "address" field in the
//                    class). For accelerated sensor, this field
//                    holds the physical sensor's IP address, because the "address"
//                    field would otherwise have the localhost address returned
//                    by the accelerator host.
typedef struct GoSensorAccelInfo
{
    GoDiscoveryOpMode       opMode;
    GoSensorAccelState      state;
    GoPortInfo              portInfo;
    // TODO: confirm what the GoAddressInfo contains in the class for the various acceleration scenarios.
    kIpAddress              accelSensorIp;
} GoSensorAccelInfo;

// This structure contains information learned through discovery,
// that aids in display listings of sensors related to acceleration of sensor.
//
// It is possible to get some of the information in this structure
// by connecting to the sensor to get its local information, but it 
// currently causes issues in a multi-sensor listing context where
// the system must be locked/unlocked repeatedly (ie. GoMax).  
// If a sensor is disconnected either explicitly or as a fallback during 
// a failed connect attempt, it can deadlock the program while waiting for 
// other threads to shutdown during the disconnect (which themselvses also 
// waiting on a lock to continue, for example the thread maanaging health
// information receives).
//
// So instead, get all the needed information from the discovery protocol's
// extended information as this bypasses requirement to connect to the sensor.
// Side effect is that some information in this structure is duplicated by other
// information received via the GetSystemInfoV2 upon connect, but we will 
// always return the discovery info in such cases for now.
//
typedef struct GoSensorDiscoveryInfo
{
    kVersion    firmwareVersion;                //firmware version
    kText32     partNumber;                     //sensor part number (renamed from model for clarity) 
    kText32     modelNumber;                    //sensor model number (for parsing)
    kText32     modelDisplayName;               //sensor model display name (for display)
    kText32     family;                         //sensor model family
} GoSensorDiscoveryInfo;

typedef struct GoSensorClass
{
    kObjectClass base;

    kObject system;                             //system object (parent)

    k32u deviceId;                              //serial number
    kVersion protocolVersion;                   //remote protocol version

    //begin synchronized
    //(system state lock)
    kBool discoveryHistory[8];                  //discovery presence history (^2)
    k64u discoveryCount;                        //discovery presence history count
    kBool healthCheckEnabled;                   //is health checking currently enabled?
    kBool healthHistory[8];                     //health presence history (^2)
    k64u healthCheckCount;                      //health presence history count
    k64u healthMsgCount;                        //count of all health messages received from sensor
    k64u previousHealthMsgCount;                //count of all health messages at last health check
    k64u sensorInfoTime;                        //time of last foreground update of sensor info
    GoAddressInfo address;                      //current network address
    GoRole role;                                //sensor role, as reported via sensor info
    k32u buddyId;                               //buddy identifier, as reported via sensor info
    GoAlignmentState alignmentState;            //alignment state
    GoState runState;                           //sensor state, as reported by sensor
    GoUser user;                                //current signed-in user
    kBool isResetting;                          //is the sensor currently resetting?
    kBool isCancelled;                          //was i/o cancelled by user?
    kBool isConnected;                          //is sensor currently connected?
    kBool isCompatible;                         //is sensor protocol compatible with client protocol?
    kBool isUpgrading;                          //is sensor currently being upgraded?
    //end synchronized

    kArrayList fileList;                        //list of sensor file names (kArrayList<kText64>)
    kBool fileListValid;                        //is file list valid?
    kArrayList directoryList;                   //list of directory file names (kArrayList<kText64>)
    kBool isDirectoryListValid;                 //is the current directory list valid?

    kArrayList partModelList;                   //list of part model configurations in the live job (of type GoPartModel)
    kBool isSyncPartModels;                     //is the part model list currently being synchronized?

    kXml configXml;                             //the last config retrieved. Used for forwards compatible config writes.
    kXml configXmlItem;                         //node reference from the last config retrieved.
    kBool configValid;                          //is config valid?
    kBool configModified;                       //has config been locally modified?
    kBool isSyncConfig;                         //is the config currently being synchronized?
    kBool isFlushConfig;                        //is the config currently being flushed? Note: disconnection flushes the config without syncing it which is why this is needed.

    kXml transformXml;                          //the last transform retrieved. Used for forwards compatible transform writes.
    kXml transformXmlItem;                      //node reference from the last transform retrieved.
    kBool transformValid;                       //is transform valid?
    kBool transformModified;                    //has transform been locally modified?
    kBool isSyncTransform;                      //is the transform currently being synchronized?

    kTimer timer;                               //utility timer (upgrades, etc.)
    GoSensorInfo localSensorInfo;               //temp variable, used during info read
    kArrayList remoteSensorInfo;                //of type GoSensorInfo - represents visible remote sensor data
    kBool infoValid;                            //is info valid?
    kArrayList buddySensorInfo;                 //of type GoBuddyInfo - represents buddy sensor info

    kPeriodic resetTimer;                       //provides a timed event to end reset state

    GoControl control;                          //control/upgrade connection
    GoReceiver data;                            //data connection
    k32u dataPort;                              //data port
    GoSensorDataSetFx onDataSet;                //callback to a custom data handling function
    kPointer onDataSetContext;                  //context to be passed into the onDataSet function
    GoReceiver health;                          //health connection
    k32u healthPort;                            //health port

    k32u controlPort;                           //Control port
    k32u upgradePort;                           //Upgrade port

    GoSetup setup;                              //setup module
    GoTools tools;                              //tools module
    GoOutput output;                            //output module
    GoTransform transform;                      //transformation module
    GoReplay replay;                            //replay module

    GoSensorAccelInfo accelInfo;                //sensor acceleration related information.
    GoSensorDiscoveryInfo discoveryInfo;        //sensor information learned via discovery. 
} GoSensorClass;

kDeclareClassEx(Go, GoSensor, kObject)

//system - a GoSystem handle
GoFx(kStatus) GoSensor_Construct(GoSensor* sensor, kPointer system, const GoDiscoveryInfo* discoveryInfo, kAlloc allocator);

GoFx(kStatus) GoSensor_Init(GoSensor sensor, kType type, kPointer system, const GoDiscoveryInfo* discoveryInfo, kAlloc alloc);
GoFx(kStatus) GoSensor_VRelease(GoSensor sensor);

GoFx(kStatus) GoSensor_SyncConfig(GoSensor sensor);
GoFx(kStatus) GoSensor_SetConnected(GoSensor sensor, kBool isConnected, kBool isCompatible);

GoFx(kStatus) GoSensor_RefreshSystem(GoSensor sensor);

GoFx(kStatus) GoSensor_Invalidate(GoSensor sensor);

GoFx(kStatus) GoSensor_CacheConfig(GoSensor sensor);
GoFx(kStatus) GoSensor_FlushConfig(GoSensor sensor);
GoFx(kStatus) GoSensor_InvalidateConfig(GoSensor sensor);
GoFx(kBool) GoSensor_ConfigValid(GoSensor sensor);
GoFx(kStatus) GoSensor_SetConfigModified(GoSensor sensor);
GoFx(kBool) GoSensor_ConfigModified(GoSensor sensor);
GoFx(kStatus) GoSensor_ReadConfig(GoSensor sensor);
GoFx(kStatus) GoSensor_WriteConfig(GoSensor sensor);
GoFx(kStatus) GoSensor_GetLiveConfig(GoSensor sensor, kXml* xml, kAlloc allocator);
GoFx(kStatus) GoSensor_SetLiveConfig(GoSensor sensor, kXml xml);

GoFx(kStatus) GoSensor_SyncTransform(GoSensor sensor);
GoFx(kStatus) GoSensor_CacheTransform(GoSensor sensor);
GoFx(kStatus) GoSensor_FlushTransform(GoSensor sensor);
GoFx(kStatus) GoSensor_InvalidateTransform(GoSensor sensor);
GoFx(kBool) GoSensor_TransformValid(GoSensor sensor);
GoFx(kStatus) GoSensor_SetTransformModified(GoSensor sensor);
GoFx(kBool) GoSensor_TransformModified(GoSensor sensor);
GoFx(kStatus) GoSensor_ReadTransform(GoSensor sensor);
GoFx(kStatus) GoSensor_WriteTransform(GoSensor sensor);
GoFx(kStatus) GoSensor_GetLiveTransform(GoSensor sensor, kXml* xml, kAlloc allocator);
GoFx(kStatus) GoSensor_SetLiveTransform(GoSensor sensor, kXml xml);

GoFx(kStatus) GoSensor_SyncPartModels(GoSensor sensor);
GoFx(kStatus) GoSensor_InvalidatePartModels(GoSensor sensor);
GoFx(kStatus) GoSensor_CachePartModels(GoSensor sensor);
GoFx(kStatus) GoSensor_FlushPartModels(GoSensor sensor);
GoFx(kStatus) GoSensor_ReadPartModels(GoSensor sensor);
GoFx(kStatus) GoSensor_ReadPartModel(GoSensor sensor, GoPartModel model);
GoFx(kStatus) GoSensor_WritePartModel(GoSensor sensor, GoPartModel model);

GoFx(kStatus) GoSensor_CacheFileList(GoSensor sensor);
GoFx(kStatus) GoSensor_ReadFileList(GoSensor sensor);
GoFx(kStatus) GoSensor_InvalidateFileList(GoSensor sensor);
GoFx(kBool) GoSensor_FileListValid(GoSensor sensor);

GoFx(kStatus) GoSensor_CacheDirectoryList(GoSensor sensor, const kChar* extensionFilter, const kChar* path, kBool isRecursive);
GoFx(kStatus) GoSensor_ListDirectory(GoSensor sensor, const kChar* extension, const kChar* root, kBool isRecursive, kArrayList fileList);
GoFx(kStatus) GoSensor_InvalidateDirectoryList(GoSensor sensor);
GoFx(kBool) GoSensor_DirectoryListValid(GoSensor sensor);

GoFx(kStatus) GoSensor_CacheInfo(GoSensor sensor);
GoFx(kStatus) GoSensor_ReadInfo(GoSensor sensor);
GoFx(kBool) GoSensor_InfoValid(GoSensor sensor);
GoFx(kStatus) GoSensor_InvalidateInfo(GoSensor sensor);

GoFx(kStatus) GoSensor_InvalidateRole(GoSensor sensor);

GoFx(kStatus) GoSensor_InvalidateBuddyId(GoSensor sensor);
GoFx(kStatus) GoSensor_InvalidateBuddyInfoList(GoSensor sensor);

GoFx(GoState) GoSensor_KnownState(GoSensor sensor);
GoFx(kBool) GoSensor_IsReadable(GoSensor sensor);
GoFx(kBool) GoSensor_IsConfigurable(GoSensor sensor);
GoFx(kBool) GoSensor_IsReady(GoSensor sensor);
GoFx(kBool) GoSensor_IsRunning(GoSensor sensor);
GoFx(kBool) GoSensor_IsNormal(GoSensor sensor);
GoFx(kBool) GoSensor_IsCancelled(GoSensor sensor);
GoFx(kBool) GoSensor_ConsistencyIntervalElapsed(GoSensor sensor);

GoFx(kBool) GoSensor_DataEnabled(GoSensor sensor);

GoFx(kStatus) GoSensor_EnableHealth(GoSensor sensor, kBool enable);
GoFx(kStatus) GoSensor_CheckHealth(GoSensor sensor);
GoFx(kStatus) GoSensor_UpdateHealthInfo(GoSensor sensor, GoDataSet healthSet);

GoFx(kBool) GoSensor_IsDiscoveryOnline(GoSensor sensor);
GoFx(kBool) GoSensor_IsHealthOnline(GoSensor sensor);

GoFx(kStatus) GoSensor_OnCancelQuery(GoSensor sensor, kObject sender, kPointer args);

GoFx(kStatus) GoSensor_LockState(GoSensor sensor);
GoFx(kStatus) GoSensor_UnlockState(GoSensor sensor);

GoFx(kStatus) GoSensor_BeginReset(GoSensor sensor);
GoFx(kStatus) GoSensor_OnResetHoldComplete(GoSensor sensor, kPeriodic timer);

GoFx(kStatus) GoSensor_WaitForReboot(GoSensor sensor, k64u timeout);
GoFx(kStatus) GoSensor_WaitForReconnect(GoSensor sensor, k64u timeout);

GoFx(kStatus) GoSensor_BeginStart(GoSensor sensor);
GoFx(kStatus) GoSensor_EndStart(GoSensor sensor);

GoFx(kStatus) GoSensor_BeginScheduledStart(GoSensor sensor, k64s value);
GoFx(kStatus) GoSensor_EndScheduledStart(GoSensor sensor);

GoFx(kStatus) GoSensor_BeginStop(GoSensor sensor);
GoFx(kStatus) GoSensor_EndStop(GoSensor sensor);

GoFx(kStatus) GoSensor_BeginSnapshot(GoSensor sensor);
GoFx(kStatus) GoSensor_EndSnapshot(GoSensor sensor);

GoFx(kStatus) GoSensor_BeginAlign(GoSensor sensor);
GoFx(kStatus) GoSensor_EndAlign(GoSensor sensor);

GoFx(kStatus) GoSensor_BeginExposureAutoSet(GoSensor sensor, GoRole role);
GoFx(kStatus) GoSensor_EndExposureAutoSet(GoSensor sensor);

GoFx(kStatus) GoSensor_OnData(GoSensor sensor, GoReceiver receiver, kSerializer reader);
GoFx(kStatus) GoSensor_OnHealth(GoSensor sensor, GoReceiver receiver, kSerializer reader);

GoFx(kStatus) GoSensor_BeginUpgrade(GoSensor sensor, const kChar* sourcePath);
GoFx(kStatus) GoSensor_EndUpgrade(GoSensor sensor);

GoFx(kStatus) GoSensor_AddTool(GoSensor sensor, const kChar* type, const kChar* name);
GoFx(kStatus) GoSensor_AddMeasurement(GoSensor sensor, kSize index, const kChar* type, const kChar* name);

GoFx(kStatus) GoSensor_PostUpgrade(GoSensor sensor);

GoFx(kStatus) GoSensor_EnableRamImage(GoSensor sensor, kBool enable);
GoFx(kStatus) GoSensor_RamImageEnabled(GoSensor sensor);
GoFx(kStatus) GoSensor_WriteRamImage(GoSensor sensor, GoRole role, kBool resizeActiveArea, kSize cameraIndex, kSize stateIndex, kSize frameIndex, const kChar* imagePath);

GoFx(kStatus) GoSensor_UpdateDiscoveryInfo(GoSensor sensor, const GoDiscoveryInfo* info);
GoFx(kStatus) GoSensor_UpdateDiscoveryCycle(GoSensor sensor);

GoFx(GoAddressInfo*) GoSensor_AddressInfo(GoSensor sensor);

GoFx(GoControl) GoSensor_Control(GoSensor sensor);

GoFx(kStatus) GoSensor_SetAccelState(GoSensor sensor, GoSensorAccelState accelState);
GoFx(kStatus) GoSensor_SetAccelOpMode(GoSensor sensor, GoDiscoveryOpMode opMode);
GoFx(kStatus) GoSensor_SetAccelPortInfo(GoSensor sensor, GoPortInfo* portInfo);
GoFx(kStatus) GoSensor_SetAccelSensorIpAddress(GoSensor sensor, kIpAddress* ipAddress);
GoFx(kStatus) GoSensor_UpdateAccelSensorIpAddr(GoSensor sensor, const GoDiscoveryInfo* info, GoDiscoveryOpMode opMode);
GoFx(kStatus) GoSensor_UpdateAccelState(GoSensor sensor, const GoDiscoveryInfo* info, GoDiscoveryOpMode opMode);
GoFx(kBool) GoSensor_UseDiscoveryServerAddr(GoSensor sensor);

GoFx(kStatus) GoSensor_WaitForBuddies(GoSensor sensor, k64u timeout);
GoFx(kStatus) GoSensor_AppendBuddy(GoSensor sensor, GoSensor buddy);
GoFx(kStatus) GoSensor_CreateBuddyList(GoSensor sensor, kArrayList* buddyIds);

GoFx(kStatus) GoSensor_SetTimeDate(GoSensor sensor);

// Helper functions to fill information for sensors with older firmware and/or older id.xml's.
GoFx(GoFamily) GoSensor_FillFamily(GoSensor sensor);

/**
* Update the sensor's acceleration information.
*
* @public              @memberof GoSensor
* @param   sensor      GoSensor object.
* @param   info        Discovery protocol information object.
* @return              Status of operation
*/
GoFx(kStatus) GoSensor_UpdateAccelInfo(GoSensor sensor, const GoDiscoveryInfo* info);

/**
 * Gets the buddy sensor.
 *
 * @deprecated          As buddied devices with firmware 4.2 and newer are
 *                      no longer discoverable, this function will return
 *                      kNULL if called with a connected sensor.
 * @public              @memberof GoSensor
 * @version             Introduced in firmware 4.0.10.27
 * @param   sensor      GoSensor object.
 * @return              Buddy sensor (or kNULL if not assigned or not online).
 */
GoFx(GoSensor) GoSensor_Buddy(GoSensor sensor);

/**
 * @deprecated Gets the model associated with this sensor.
 *             Use GoSensor_PartNumber() or GoSensor_ModelDisplayName().
 *
 * @public              @memberof GoSensor
 * @version             Introduced in firmware 4.0.10.27
 * @param   sensor      GoSensor object.
 * @param   model       Receives sensor model.
 * @param   capacity    Capacity of model buffer.
 * @return              Operation status.
 */
GoFx(kStatus) GoSensor_Model(GoSensor sensor, kChar* model, kSize capacity);

/**
 * @deprecated Gets the model number associated with this sensor.
 *             For internal use only to retrieve the LMI model number of the sensor.
 *
 * @public                      @memberof GoSensorInfo
 * @version                     Introduced in firmware 5.3.17.23
 * @param   sensor              GoSensor object.
 * @param   modelNumber         A character array pointer.
 * @param   capacity            The character array capacity.
 * @return                      Operation status.
 */
GoFx(kStatus) GoSensor_ModelNumber(GoSensor sensor, kChar* modelNumber, kSize capacity);

#endif

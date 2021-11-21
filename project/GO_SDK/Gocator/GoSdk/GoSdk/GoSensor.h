/**
 * @file    GoSensor.h
 * @brief   Declares the GoSensor class.
 *
 * @internal
 * Copyright (C) 2016-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef GO_SENSOR_H
#define GO_SENSOR_H

#include <GoSdk/GoSdkDef.h>
#include <GoSdk/GoSetup.h>
#include <GoSdk/GoPartModel.h>
#include <GoSdk/GoReplay.h>
#include <GoSdk/GoTransform.h>
#include <GoSdk/GoSensorInfo.h>
#include <GoSdk/GoGeoCal.h>
#include <GoSdk/Messages/GoDataSet.h>
#include <GoSdk/Messages/GoDiscoveryExtInfo.h>
#include <GoSdk/Outputs/GoOutput.h>
#include <GoSdk/Tools/GoTools.h>

#define GO_SENSOR_LIVE_JOB_NAME                     "_live.job"         //<<< Represents the active live job on the sensor
#define GO_SENSOR_LIVE_LOG_NAME                     "_live.log"         //<<< Represents the log file on the sensor
#define GO_SENSOR_LIVE_REPLAY                       "_live.rec"         //<<< Represents the current recording on the sensor
#define GO_SENSOR_LIVE_REPLAY_STREAM                "_livestream.rec"   //<<< Represents the current recording stream on the sensor

/**
 * @class   GoSensor
 * @extends kObject
 * @ingroup GoSdk
 * @brief   Represents a Gocator sensor.
 */
typedef kObject GoSensor;

/**
 * Initiates a sensor configuration, model file, and transformation synchronization
 * if modifications are present.
 *
 * @public              @memberof GoSetup
 * @version             Introduced in firmware 4.0.10.27
 * @param   sensor      GoSensor object.
 * @return              Operation status.
 */
GoFx(kStatus) GoSensor_Flush(GoSensor sensor);

/**
 * Configures a sensor's network address settings.
 * 
 * WARNING! This operation writes to flash storage.
 * Review the user manual for implications.
 *
 * This function uses UDP broadcasts for sensor configuration; the sensor does not need to
 * be connected, and can be on a different subnet than the client.
 *
 * The sensor will automatically reboot if the address is successfully changed.
 *
 * @public              @memberof GoSensor
 * @version             Introduced in firmware 4.0.10.27
 * @param   sensor      GoSensor object.
 * @param   info        New address settings.
 * @param   wait        Should this function block until the sensor finishes rebooting?
 * @return              Operation status.
 * @remark              Calling this function will result in writing operations to flash storage. Should the process be disrupted due to power loss or other factors, the sensor may enter Rescue mode.
 */
GoFx(kStatus) GoSensor_SetAddress(GoSensor sensor, const GoAddressInfo* info, kBool wait);

/**
 * Retrieves the sensor's network address settings.
 *
 * @public              @memberof GoSensor
 * @version             Introduced in firmware 4.0.10.27
 * @param   sensor      GoSensor object.
 * @param   info        Receives current address configuration.
 * @return              Operation status.
 */
GoFx(kStatus) GoSensor_Address(GoSensor sensor, GoAddressInfo* info);

/**
 * Creates a connection to the sensor.
 *
 * @public              @memberof GoSensor
 * @version             Introduced in firmware 4.0.10.27
 * @param   sensor      GoSensor object.
 * @return              Operation status.
 */
GoFx(kStatus) GoSensor_Connect(GoSensor sensor);

/**
 * Disconnects from the sensor. Do not call this function when state is locked
 * with GoSensor_LockState().
 *
 * @public              @memberof GoSensor
 * @version             Introduced in firmware 4.0.10.27
 * @param   sensor      GoSensor object.
 * @return              Operation status.
 */
GoFx(kStatus) GoSensor_Disconnect(GoSensor sensor);

/**
 * Reports whether the sensor is currently connected. If sensors are temporarily unreachable, they do not leave
 * isConnected state. You can use GoSensor_IsResponsive for this and optionaly call GoSensor_Disconnect if
 * unresponsive for too long.
 *
 * @public              @memberof GoSensor
 * @version             Introduced in firmware 4.0.10.27
 * @param   sensor      GoSensor object.
 * @return              kTRUE if the sensor is connected, kFALSE otherwise.
 */
GoFx(kBool) GoSensor_IsConnected(GoSensor sensor);

/**
 * Reports whether the sensor is currently responsive.
 *
 * @public              @memberof GoSensor
 * @version             Introduced in firmware 5.0.2.0
 * @param   sensor      GoSensor object.
 * @return              kTRUE if the sensor is responsive kFALSE otherwise.
 */
GoFx(kBool) GoSensor_IsResponsive(GoSensor sensor);

/**
 * Reports whether the connected sensor's protocol version is compatible with the SDK's protocol version.
 *
 * @public              @memberof GoSensor
 * @version             Introduced in firmware 4.4.4.14
 * @param   sensor      GoSensor object.
 * @return              kTRUE if the sensor is compatible, kFALSE otherwise.
 */
GoFx(kBool) GoSensor_IsCompatible(GoSensor sensor);

/**
 * Refreshes sensor state.
 *
 * Unresponsive sensors will be disconnected, and canceled sensors will be reconnected.
 * Sensors in any other state will discard all locally-cached information.
 *
 * This function should be used to update sensors in the GO_SENSOR_INCONSISTENT state. This
 * state can arise due to buddy changes performed by remote sensors (e.g. a main sensor boots
 * and claims ownership of a buddy sensor, but the buddy sensor has already been detected and
 * loaded as a main sensor by the client).
 *
 * @public              @memberof GoSensor
 * @version             Introduced in firmware 4.0.10.27
 * @param   sensor      GoSensor object.
 * @return              kTRUE if the sensor is connected; kFALSE otherwise.
 */
GoFx(kStatus) GoSensor_Refresh(GoSensor sensor);

/**
 * Assigns a buddy sensor.
 *
 * This function is asynchronous, use GoSensor_AddBuddyBlocking() for synchronous version.
 *
 * NOTE: The provided buddy sensor handle must already be connected.
 *
 * @public              @memberof GoSensor
 * @version             Introduced in firmware 4.0.10.27
 * @param   sensor      GoSensor object.
 * @param   buddy       Sensor to be assigned as buddy.
 * @return              Operation status.
 * @see                 GoSensor_Connect, GoSensor_HasBuddy, GoSensor_BuddyId, GoSensor_RemoveBuddy
 */
GoFx(kStatus) GoSensor_AddBuddy(GoSensor sensor, GoSensor buddy);

/**
* Synchronously assigns a buddy sensor.
*
* NOTE: The provided buddy sensor handle must already be connected.
*
* @public              @memberof GoSensor
* @version             Introduced in firmware 4.8.1.65
* @param   sensor      GoSensor object.
* @param   buddy       Sensor to be assigned as buddy.
* @return              Operation status.
* @see                 GoSensor_Connect, GoSensor_HasBuddy, GoSensor_BuddyId, GoSensor_RemoveBuddy
*/
GoFx(kStatus) GoSensor_AddBuddyBlocking(GoSensor sensor, GoSensor buddy);

/**
 * Removes the current buddy sensor.
 *
 * @public              @memberof GoSensor
 * @version             Introduced in firmware 4.0.10.27
 * @param   sensor      GoSensor object.
 * @return              Operation status.
 */
GoFx(kStatus) GoSensor_RemoveBuddy(GoSensor sensor);

/**
 * Reports whether a buddy had been assigned.
 *
 * @public              @memberof GoSensor
 * @version             Introduced in firmware 4.0.10.27
 * @param   sensor      GoSensor object.
 * @return              kTRUE if sensor has a buddy; kFALSE otherwise.
 */
GoFx(kBool) GoSensor_HasBuddy(GoSensor sensor);

/**
 * Gets the buddy sensor's device ID.
 *
 * @public              @memberof GoSensor
 * @version             Introduced in firmware 4.0.10.27
 * @param   sensor      GoSensor object.
 * @return              Buddy device ID (or k32U_NULL if not assigned).
 */
GoFx(k32u) GoSensor_BuddyId(GoSensor sensor);

/**
 * Gets the sensor's scan mode.
 *
 * @public              @memberof GoSensor
 * @version             Introduced in firmware 4.0.10.27
 * @param    sensor     GoSensor object.
 * @return              Scan mode.
 */
GoFx(GoMode) GoSensor_ScanMode(GoSensor sensor);

/**
 * Enables or disables the sensor's data channel. After using GoSensor_EnableData function, SDK application is responsible 
 * for disposing of the GoDataSet objects that hold the data received from the sensor on the data connection.
 *
 * @public              @memberof GoSensor
 * @version             Introduced in firmware 4.0.10.27
 * @param   sensor      GoSensor object.
 * @param   enable      kTRUE to enable, or kFALSE to disable.
 * @return              Operation status.
 */
GoFx(kStatus) GoSensor_EnableData(GoSensor sensor, kBool enable);

/**
 * Starts the sensor.
 *
 * @public              @memberof GoSensor
 * @version             Introduced in firmware 4.0.10.27
 * @param   sensor      GoSensor object.
 * @return              Operation status.
 */
GoFx(kStatus) GoSensor_Start(GoSensor sensor);

/**
 * Checks if the sensor is ready to start, if all assigned buddies are connected.
 *
 * @public                 @memberof GoSensor
 * @version             Introduced in firmware 5.2.18.3
 * @param   sensor         GoSensor object.
 * @return                 Operation status.
 */
GoFx(kBool) GoSensor_CanStart(GoSensor sensor);

/**
 * Starts the sensor at a scheduled value.
 *
 * @public              @memberof GoSensor
 * @version             Introduced in firmware 4.1.3.106
 * @param   sensor      GoSensor object.
 * @param   value       Scheduled start value. uS when time triggered and ticks when encoder triggered.
 * @return              Operation status.
 * @see                 GoSetup_SetTriggerSource, GoSetup_TriggerSource, GoSystem_Timestamp, GoSystem_Encoder
 */
GoFx(kStatus) GoSensor_ScheduledStart(GoSensor sensor, k64s value);

/**
 * Stops the sensor.
 *
 * @public              @memberof GoSensor
 * @version             Introduced in firmware 4.0.10.27
 * @param   sensor      GoSensor object.
 * @return              Operation status.
 */
GoFx(kStatus) GoSensor_Stop(GoSensor sensor);

/**
 * Performs a sensor snapshot.
 *
 * A snapshot starts the sensor, takes a scan, and immediately stops the sensor.
 * Because the sensor is stopped after every snapshot, some asynchronous activities such
 * as digital output may not have enough time to occur. Algorithms that maintain memory
 * between scans would also be reset for every snapshot. The only guarantee that 
 * can be made is that the data is delivered over the SDK.
 *
 * Because of these limitations, GoSensor_Trigger should usually be used instead.
 *
 * @public              @memberof GoSensor
 * @version             Introduced in firmware 5.2.1.x
 * @param   sensor      GoSensor object.
 * @return              Operation status.
 */
GoFx(kStatus) GoSensor_Snapshot(GoSensor sensor);

/**
 * Perform alignment using the configured alignment type and target.
 *
 * NOTE: This operation will result in a sensor start for the duration of the
 * alignment. It can be canceled via GoSensor_Stop. This function's operation
 * status does not correspond to the actual alignment result. In order to
 * retrieve the alignment result, you must enable the data channel before calling
 * this function, receive an alignment data message and then check its status.
 * 
 * WARNING! This operation may (depending on alignment reference) write to flash storage.
 * Review the user manual for implications.
 *
 * @public              @memberof GoSensor
 * @version             Introduced in firmware 4.0.10.27
 * @param   sensor      GoSensor object.
 * @return              Operation status.
 * @see                 GoSensor_EnableData, GoSystem_ReceiveData, GoSetup_AlignmentType, GoSetup_AlignmentMovingTarget, GoSetup_AlignmentStationaryTarget, GoAlignMsg_Status, GoSensor_Stop
 * @remark              Calling this function will result in writing operations to flash storage. Should the process be disrupted due to power loss or other factors, the sensor may enter Rescue mode.
 */
GoFx(kStatus) GoSensor_Align(GoSensor sensor);


/**
 * Clears the current sensor alignment.
 * 
 * WARNING! This operation writes to flash storage.
 * Review the user manual for implications.
 *
 * @public              @memberof GoSensor
 * @version             Introduced in firmware 4.1.3.106
 * @param   sensor      GoSensor object.
 * @return              Operation status.
 * @see                 GoSensor_Align
 * @remark              Calling this function will result in writing operations to flash storage. Should the process be disrupted due to power loss or other factors, the sensor may enter Rescue mode.
 */
GoFx(kStatus) GoSensor_ClearAlignment(GoSensor sensor);


/**
 * Perform an exposure auto set.
 *
 * NOTE: This operation will result in a sensor start for the duration of the
 * exposure AutoSet. A successful operation status does NOT modify the configuration.
 * You must retrieve the resulting exposure value and set it for the appropriate
 * exposure setting. This involves enabling the data connection prior to running
 * exposure auto set and then receiving an exposure auto set message, which
 * you can then use to query the status and access the resulting value.
 *
 * @public              @memberof GoSensor
 * @version             Introduced in firmware 4.0.10.27
 * @param   sensor      GoSensor object.
 * @param   role        Determines which device to apply changes to. Use GO_ROLE_MAIN or GOROLE_BUDDYIDX(buddyidx). See GoRole for more details.
 * @return              Operation status.
 * @see                 GoRole, GoSensor_EnableData, GoSystem_ReceiveData, GoExposureCalMsg_Status, GoExposureCalMsg_Exposure
 */
GoFx(kStatus) GoSensor_ExposureAutoSet(GoSensor sensor, GoRole role);

/**
 * Gets the alignment state of the sensor.
 *
 * @public              @memberof GoSensor
 * @version             Introduced in firmware 4.0.10.27
 * @param   sensor      GoSensor object.
 * @return              A GoAlignmentState.
 */
GoFx(GoAlignmentState) GoSensor_AlignmentState(GoSensor sensor);


/**
 * Sets the alignment reference of the sensor.
 *
 * @public              @memberof GoSensor
 * @version             Introduced in firmware 4.0.10.27
 * @param   sensor      GoSensor object.
 * @param   reference   The alignment reference value to set.
 * @return              Operation status.
 */
GoFx(kStatus) GoSensor_SetAlignmentReference(GoSensor sensor, GoAlignmentRef reference);

/**
 * Gets the alignment reference of the sensor.
 *
 * @public              @memberof GoSensor
 * @version             Introduced in firmware 4.0.10.27
 * @param   sensor      GoSensor object.
 * @param   reference   A pointer that will hold the current alignment reference value.
 * @return              Operation status.
 */
GoFx(kStatus) GoSensor_AlignmentReference(GoSensor sensor, GoAlignmentRef* reference);

/**
 * Reboots the main sensor and any connected buddy sensors.
 *
 * @public              @memberof GoSensor
 * @version             Introduced in firmware 4.0.10.27
 * @param   sensor      GoSensor object.
 * @param   wait        kTRUE to wait for reboot and then reconnect.
 * @return              Operation status.
 */
GoFx(kStatus) GoSensor_Reset(GoSensor sensor, kBool wait);

/**
 * Resets the encoder value. NOTE: This is only possible with a direct encoder
 * connection to a sensor. Resetting the encoder value when connected to a Master
 * device will not work.
 *
 * @public              @memberof GoSensor
 * @version             Introduced in firmware 4.5.3.57
 * @param   sensor      GoSensor object.
 * @return              Operation status.
 */
GoFx(kStatus) GoSensor_ResetEncoder(GoSensor sensor);

/**
 * Aborts ongoing sensor communication.
 *
 * This method asynchronously aborts ongoing communication; the next time that any
 * I/O operation blocks for an extended period of time, it will be terminated.  This method
 * is thread-safe.
 *
 * In order to resume communication, call GoSensor_Refresh or GoSensor_Connect.
 *
 * @public                  @memberof GoSensor
 * @version                 Introduced in firmware 4.0.10.27
 * @param    sensor         GoSensor object.
 * @return   Operation status.
 */
GoFx(kStatus) GoSensor_Cancel(GoSensor sensor);

/**
 * Gets the current time stamp (common among all synchronized sensors).
 *
 * @public              @memberof GoSensor
 * @version             Introduced in firmware 4.0.10.27
 * @param   sensor      GoSensor object.
 * @param   time        Receives the current time stamp(us).
 * @return              Operation status.
 */
GoFx(kStatus) GoSensor_Timestamp(GoSensor sensor, k64u* time);

/**
 * Gets the current encoder count from the sensor. This is useful when an encoder value is needed
 * while the sensor is not started and thus not producing data with encoder values in the stamp.
 *
 * Note that this function sends a command to the sensor, where the current encoder value is read
 * and then sent back in a reply message. So this is not the most accurate method of reading the 
 * encoder value. To read the encoder value of a specific frame of data, use the encoder value
 * from the GoStamp received as GO_DATA_MESSAGE_TYPE_STAMP.
 *
 * @public              @memberof GoSensor
 * @version             Introduced in firmware 4.0.10.27
 * @param   sensor      GoSensor object.
 * @param   encoder     Receives the encoder count (ticks).
 * @return              Operation status.
 */
GoFx(kStatus) GoSensor_Encoder(GoSensor sensor, k64s* encoder);

/**
 * Sends a software trigger to the sensor.
 *
 * This method is used in conjunction with sensors that are configured to accept
 * software triggers. The sensor must be running (e.g. by calling GoSensor_Start)
 * for triggers to be accepted.
 * 
 * When the trigger mode is set to Software, this command will trigger individual 
 * frames in Profile or Surface mode. For G2 sensors with other trigger modes, 
 * this command can also be used to trigger Fixed Length surface generation when 
 * the Fixed Length Start Trigger option is set to "Software".
 * 
 * @public              @memberof GoSensor
 * @version             Introduced in firmware 4.0.10.27
 * @param   sensor      GoSensor object.
 * @return              Operation status.
 * @see                 GoSetup_TriggerSource, GoSetup_SetTriggerSource
 */
GoFx(kStatus) GoSensor_Trigger(GoSensor sensor);

/**
 * Schedules a digital output.
 *
 * This method requires that the output is configured to trigger on software control.
 *
 * @public              @memberof GoSensor
 * @version             Introduced in firmware 4.0.10.27
 * @param   sensor      GoSensor object.
 * @param   index       The digital output index.
 * @param   target      The time or position target (us or mm), depending on GoDomain.  Ignored if
 *                      GoDigital_ScheduleEnabled is false or GoDigital_SignalType is pulsed.
 * @param   value       The value of scheduled output (0-Low or 1-High). Ignored if output
 *                      GoDigital_SignalType is pulsed.
 * @return              Operation status.
 */
GoFx(kStatus) GoSensor_EmitDigital(GoSensor sensor, k16u index, k64s target, k8u value);

/**
 * Schedules an analog output.
 *
 * This method requires that the output be configured to trigger on software control.
 * NOTE: It is not possible to schedule a continuous output. The operation will
 * fail accordingly if attempted.
 *
 * @public              @memberof GoSensor
 * @version             Introduced in firmware 4.0.10.27
 * @param   sensor      GoSensor object.
 * @param   index       The analog output index.
 * @param   target      The time or position target (us or mm), depending on GoDomain.  Ignored if
 *                      GoAnalog_ScheduleEnabled is false.
 * @param   value       The value of the scheduled output (uA).
 * @return              Operation status.
 */
GoFx(kStatus) GoSensor_EmitAnalog(GoSensor sensor, k16u index, k64s target, k32s value);

/**
 * Gets the number of files available from the connected sensor. This includes files that may not be visible through the web interface.
 *
 * @public              @memberof GoSensor
 * @version             Introduced in firmware 4.0.10.27
 * @param   sensor      GoSensor object.
 * @return              File count.
 */
GoFx(kSize) GoSensor_FileCount(GoSensor sensor);

/**
 * Gets the file name at the specified index.
 *
 * @public              @memberof GoSensor
 * @version             Introduced in firmware 4.0.10.27
 * @param   sensor      GoSensor object.
 * @param   index       Index of the desired file name.
 * @param   name        Receives the name of the file.
 * @param   capacity    Capacity of the file name buffer.
 * @return              Operation status.
 */
GoFx(kStatus) GoSensor_FileNameAt(GoSensor sensor, kSize index, kChar* name, kSize capacity);

/**
 * Uploads a file to the connected sensor. The following is a list of
 * macros representing common files used in this operation:
 *
 * @li                  GO_SENSOR_LIVE_JOB_NAME - Represents "_live.job".
 * @li                  GO_SENSOR_LIVE_REPLAY   - Represents the recording file, "_live.rec".
 * 
 * WARNING! This operation writes to flash storage.
 * Review the user manual for implications.
 *
 * @public                  @memberof GoSensor
 * @version                 Introduced in firmware 4.0.10.27
 * @param   sensor          GoSensor object.
 * @param   sourcePath      Source file system path for the file to be uploaded.
 * @param   destName        Destination name for the uploaded file (maximum 63 characters).
 * @return                  Operation status.
 * @remark                  Calling this function will result in writing operations to flash storage if the destination is not a live file. Should the process be disrupted due to power loss or other factors, the sensor may enter Rescue mode.
 */
GoFx(kStatus) GoSensor_UploadFile(GoSensor sensor, const kChar* sourcePath, const kChar* destName);

/**
 * Downloads a file from the connected sensor. The following is a list of
 * macros representing common files used in this operation:
 *
 * @li                  GO_SENSOR_LIVE_JOB_NAME - Represents "_live.job".
 * @li                  GO_SENSOR_LIVE_REPLAY   - Represents the recording file, "_live.rec".
 *
 * File names of saved jobs, as well as a support file "support.gs", may also be specified.
 *
 * @public                  @memberof GoSensor
 * @version                 Introduced in firmware 4.0.10.27
 * @param   sensor          GoSensor object.
 * @param   sourceName      Source name of the file to be downloaded.
 * @param   destPath        Destination file system path for the file to be downloaded.
 * @return                  Operation status.
 */
GoFx(kStatus) GoSensor_DownloadFile(GoSensor sensor, const kChar* sourceName, const kChar* destPath);

/**
 * Copies a file within the connected sensor.
 *
 * If the destination file name is GO_SENSOR_LIVE_JOB_NAME, then the configuration job
 * file used by the sensor is changed (ie. a job/load switch operation takes place).
 * To do a job load/switch, the sensor is first stopped before the job load/switch.
 * After the job load/switch, the sensor remains stopped, to match the behaviour
 * of loading/switching a job file from the GUI.
 * 
 * WARNING! This operation writes to flash storage.
 * Review the user manual for implications.
 *
 * NOTE: if doing a job load/switch, this API leaves the sensor stopped. User must
 * explicitly start the sensor again.
 *
 * @public                  @memberof GoSensor
 * @version                 Introduced in firmware 4.0.10.27
 * @param   sensor          GoSensor object.
 * @param   sourceName      Source name for the file to be copied.
 * @param   destName        Destination name for the file (maximum 63 characters).
 * @return                  Operation status.
 * @remark                  Calling this function can result in writing operations to flash storage if the destination is not a live file(e.g. "_live.job"). Should the process be disrupted due to power loss or other factors, the sensor may enter Rescue mode.
 * @see                     GoSensor_UploadFile()
 */
GoFx(kStatus) GoSensor_CopyFile(GoSensor sensor, const kChar* sourceName, const kChar* destName);

/**
 * Deletes a file within the connected sensor.
 *
 * @public              @memberof GoSensor
 * @version             Introduced in firmware 4.0.10.27
 * @param   sensor      GoSensor object.
 * @param   name        Name of the file to be deleted.
 * @return              Operation status.
 * @remark              Calling this function will result in writing operations to flash storage. Should the process be disrupted due to power loss or other factors, the sensor may enter Rescue mode.
 */
GoFx(kStatus) GoSensor_DeleteFile(GoSensor sensor, const kChar* name);

/**
 * Checks whether the specified file is present on the sensor.
 * 
 * WARNING! This operation writes to flash storage.
 * Review the user manual for implications.
 *
 * @public              @memberof GoSensor
 * @version             Introduced in firmware 4.0.10.27
 * @param   sensor      GoSensor object.
 * @param   name        Name of the file to be checked.
 * @return              Operation status.
 */
GoFx(kBool) GoSensor_FileExists(GoSensor sensor, const kChar* name);

/**
 * Gets the available storage space remaining for user files.
 *
 * @public              @memberof GoSensor
 * @version
 * @param   sensor      GoSensor object.
 * @return              Storage space available.
 */
GoFx(k64u) GoSensor_UserStorageFree(GoSensor sensor);

/**
 * Gets the storage space used for user files.
 *
 * @public              @memberof GoSensor
 * @version
 * @param   sensor      GoSensor object.
 * @return              Storage space used.
 */
GoFx(k64u) GoSensor_UserStorageUsed(GoSensor sensor);

/**
 * Sets a default job file to be loaded on boot.
 * 
 * WARNING! This operation writes to flash storage.
 * Review the user manual for implications.
 *
 * @public                  @memberof GoSensor
 * @version                 Introduced in firmware 4.0.10.27
 * @param   sensor          GoSensor object.
 * @param   fileName        Name of the default file.
 * @return                  Operation status.
 * @remark                  Calling this function will result in writing operations to flash storage. Should the process be disrupted due to power loss or other factors, the sensor may enter Rescue mode.
 */
GoFx(kStatus) GoSensor_SetDefaultJob(GoSensor sensor, const kChar* fileName);

/**
 * Gets the name of the default job file to be loaded on boot.
 *
 * @public                  @memberof GoSensor
 * @version                 Introduced in firmware 4.0.10.27
 * @param   sensor          GoSensor object.
 * @param   fileName        Receives name of the default file.
 * @param   capacity        Name buffer capacity.
 * @return                  Operation status.
 */
GoFx(kStatus) GoSensor_DefaultJob(GoSensor sensor, kChar* fileName, kSize capacity);

/**
 * Gets the name of the loaded job file and whether it has been modified since loading.
 *
 * @public                  @memberof GoSensor
 * @version                 Introduced in firmware 4.0.10.27
 * @param   sensor          GoSensor object.
 * @param   fileName        Receives name of the loaded file.
 * @param   capacity        Name buffer capacity.
 * @param   changed         Receives the status of whether the file has changed.
 * @return                  Operation status.
 */
GoFx(kStatus) GoSensor_LoadedJob(GoSensor sensor, kChar* fileName, kSize capacity, kBool* changed);

/**
 * Logs into the sensor using the specified user name and password.
 *
 * Logging in is not required in order to programmatically control a sensor. The Gocator log-in feature is
 * intended only to support administrative user interfaces, by allowing the username and password to be
 * stored in the sensor.
 *
 * @public                  @memberof GoSensor
 * @version                 Introduced in firmware 4.0.10.27
 * @param   sensor          GoSensor object.
 * @param   user            User account.
 * @param   password        User password.
 * @return                  Operation status.
 */
GoFx(kStatus) GoSensor_LogIn(GoSensor sensor, GoUser user, const kChar* password);

/**
 * Changes the password associated with the specified user account.
 * 
 * WARNING! This operation writes to flash storage.
 * Review the user manual for implications.
 *
 * @public                  @memberof GoSensor
 * @version                 Introduced in firmware 4.0.10.27
 * @param   sensor          GoSensor object.
 * @param   user            User account.
 * @param   password        New password.
 * @return                  Operation status.
 * @remark                  Calling this function will result in writing operations to flash storage. Should the process be disrupted due to power loss or other factors, the sensor may enter Rescue mode.
 */
GoFx(kStatus) GoSensor_ChangePassword(GoSensor sensor, GoUser user, const kChar* password);

/**
 * Upgrades sensor firmware.
 *
 * This function will block until the upgrade is completed.
 * 
 * WARNING! This operation writes to flash storage.
 * Review the user manual for implications.
 *
 * This function results in flash storage modifications. If modifications are interrupted due to
 * power loss, the sensor may reboot into Rescue mode.
 *
 * The sensor does not need to be connected to perform an upgrade.
 *
 * @public                  @memberof GoSensor
 * @version                 Introduced in firmware 4.0.10.27 and updated in 4.3.3.124
 * @param   sensor          GoSensor object.
 * @param   sourcePath      Local file system path to the upgrade file.
 * @param   onUpdate        Callback function to receive progress updates, or kNULL.
 * @param   context         Context handle to be passed to the upgrade callback.
 * @return                  Operation status.
 */
GoFx(kStatus) GoSensor_Upgrade(GoSensor sensor, const kChar* sourcePath, GoUpgradeFx onUpdate, kPointer context);

/**
 * Creates a backup of sensor files and downloads the backup to the specified location.
 *
 * @public                  @memberof GoSensor
 * @version                 Introduced in firmware 4.0.10.27
 * @param   sensor          GoSensor object.
 * @param   destPath        Local file system path for the saved backup file.
 * @return                  Operation status.
 */
GoFx(kStatus) GoSensor_Backup(GoSensor sensor, const kChar* destPath);

/**
 * Restores a backup of sensor files.
 * 
 * WARNING! This operation writes to flash storage.
 * Review the user manual for implications.
 *
 * @public                  @memberof GoSensor
 * @version                 Introduced in firmware 4.0.10.27
 * @param   sensor          GoSensor object.
 * @param   sourcePath      Local file system path of the saved backup file.
 * @return                  Operation status.
 * @remark                  Calling this function will result in writing operations to flash storage. Should the process be disrupted due to power loss or other factors, the sensor may enter Rescue mode.
 */
GoFx(kStatus) GoSensor_Restore(GoSensor sensor, const kChar* sourcePath);

/**
 * Restores factory default settings.
 * 
 * WARNING! This operation writes to flash storage.
 * Review the user manual for implications.
 *
 * @public                      @memberof GoSensor
 * @version                     Introduced in firmware 4.0.10.27
 * @param   sensor              GoSensor object.
 * @param   restoreAddress      kTRUE to restore the factory default IP address; False otherwise.
 * @return                      Operation status.
 * @remark                      Calling this function will result in writing operations to flash storage. Should the process be disrupted due to power loss or other factors, the sensor may enter Rescue mode.
 */
GoFx(kStatus) GoSensor_RestoreDefaults(GoSensor sensor, kBool restoreAddress);

/**
 * Gets the GoSetup instance associated with the sensor.
 *
 * @public              @memberof GoSensor
 * @version             Introduced in firmware 4.0.10.27
 * @param   sensor      GoSensor object.
 * @return              A GoSetup object module
 */
GoFx(GoSetup) GoSensor_Setup(GoSensor sensor);

/**
 * Gets the sensor's tools module, used for measurement configuration.
 *
 * @public              @memberof GoSensor
 * @version             Introduced in firmware 4.0.10.27
 * @param   sensor      GoSensor object.
 * @return              Measurement configuration module.
 */
GoFx(GoTools) GoSensor_Tools(GoSensor sensor);

/**
 * Gets the output module, used for output configuration.
 *
 * @public              @memberof GoSensor
 * @version             Introduced in firmware 4.0.10.27
 * @param   sensor      GoSensor object.
 * @return              Output configuration module.
 */
GoFx(GoOutput) GoSensor_Output(GoSensor sensor);

/**
 * Gets the transform module, used for transformation configuration.
 *
 * @public              @memberof GoSensor
 * @version             Introduced in firmware 4.0.10.27
 * @param   sensor      GoSensor object.
 * @return              Transformation configuration module.
 */
GoFx(GoTransform) GoSensor_Transform(GoSensor sensor);

/**
 * Gets the replay module, used for replay configuration.
 *
 * @public              @memberof GoSensor
 * @version             Introduced in firmware 4.5.3.57
 * @param   sensor      GoSensor object.
 * @return              Replay configuration module.
 */
GoFx(GoReplay) GoSensor_Replay(GoSensor sensor);

/**
 * Gets the device identifier associated with this sensor.
 *
 * @public              @memberof GoSensor
 * @version             Introduced in firmware 4.0.10.27
 * @param   sensor      GoSensor object.
 * @return              The sensor's device ID.
 */
GoFx(k32u) GoSensor_Id(GoSensor sensor);

/**
 * Gets the part number associated with this sensor.
 *
 * @public              @memberof GoSensor
 * @version             Introduced in firmware 5.3.17.23
 * @param   sensor      GoSensor object.
 * @param   partNumber  A character array pointer.
 * @param   capacity    The character array capacity.
 * @return              Operation status.
 */
GoFx(kStatus) GoSensor_PartNumber(GoSensor sensor, kChar* partNumber, kSize capacity);

/** 
 * Gets the model display name associated with this sensor.
 *
 * @public                      @memberof GoSensorInfo
 * @version                     Introduced in firmware 5.3.17.23
 * @param   sensor              GoSensor object.
 * @param   modelDisplayName    A character array pointer.
 * @param   capacity            The character array capacity.
 * @return                      Operation status.
 */
GoFx(kStatus) GoSensor_ModelDisplayName(GoSensor sensor, kChar* modelDisplayName, kSize capacity);

/**
 * Reports the current state of the sensor.
 *
 * @public              @memberof GoSensor
 * @version             Introduced in firmware 4.0.10.27
 * @param   sensor      GoSensor object.
 * @return              Sensor state.
 */
GoFx(GoState) GoSensor_State(GoSensor sensor);

/**
 * Reports the current states of the sensor.
 *
 * @public              @memberof GoSensor
 * @version             Introduced in firmware 4.0.10.27
 * @param   sensor      GoSensor object.
 * @param   states      Reference to updated states.
 * @return              Operation status.
 */
GoFx(kStatus) GoSensor_States(GoSensor sensor, GoStates* states);

/**
 * Gets the sensor's current role within the system.
 *
 * @public              @memberof GoSensor
 * @version             Introduced in firmware 4.0.10.27
 * @param   sensor      GoSensor object.
 * @return              Sensor role.
 */
GoFx(GoRole) GoSensor_Role(GoSensor sensor);

/**
 * Reports the user account associated with the current user.
 *
 * @public              @memberof GoSensor
 * @version             Introduced in firmware 4.0.10.27
 * @param   sensor      GoSensor object.
 * @return              User account id.
 */
GoFx(GoUser) GoSensor_User(GoSensor sensor);

/**
 * Gets the sensor's protocol version.
 *
 * @public              @memberof GoSensor
 * @version             Introduced in firmware 4.0.10.27
 * @param   sensor      GoSensor object.
 * @return              Protocol version.
 */
GoFx(kVersion) GoSensor_ProtocolVersion(GoSensor sensor);

/**
 * Gets the sensor's firmware version.
 *
 * @public              @memberof GoSensor
 * @version             Introduced in firmware 4.0.10.27
 * @param   sensor      GoSensor object.
 * @return              Firmware version.
 */
GoFx(kVersion) GoSensor_FirmwareVersion(GoSensor sensor);

/**
 * Sets the recording state of the sensor.
 *
 * @public              @memberof GoSensor
 * @version             Introduced in firmware 4.0.10.27
 * @param   sensor      GoSensor object.
 * @param   enable      Enables or disables recording.
 * @return              Operation status.
 */
GoFx(kStatus) GoSensor_EnableRecording(GoSensor sensor, kBool enable);

/**
 * Gets the recording state of the sensor.
 *
 * @public              @memberof GoSensor
 * @version             Introduced in firmware 4.0.10.27
 * @param   sensor      GoSensor object.
 * @return              kTRUE if recording is enabled. kFALSE otherwise.
 */
GoFx(kBool) GoSensor_RecordingEnabled(GoSensor sensor);

/**
 * Sets the input source of the sensor.
 *
 * @public              @memberof GoSensor
 * @version             Introduced in firmware 4.0.10.27
 * @param   sensor      GoSensor object.
 * @param   source      The input source to use.
 * @return              Operation status.
 */
GoFx(kStatus) GoSensor_SetInputSource(GoSensor sensor, GoInputSource source);

/**
 * Gets the input source currently used by the sensor.
 *
 * @public              @memberof GoSensor
 * @version             Introduced in firmware 4.0.10.27
 * @param   sensor      GoSensor object.
 * @return              A GoInputSource.
 */
GoFx(GoInputSource) GoSensor_InputSource(GoSensor sensor);

/**
 * Simulates the current frame in the live recording buffer.
 *
 * @public                  @memberof GoSensor
 * @version                 Introduced in firmware 4.0.10.27
 * @param   sensor          GoSensor object.
 * @param   isBufferValid   kTRUE if the source simulation buffer was valid. kFALSE otherwise.
 * @return              Operation status.
 */
GoFx(kStatus) GoSensor_Simulate(GoSensor sensor, kBool* isBufferValid);


/**
 * Advances one frame from the current replay position.
 *
 * @public              @memberof GoSensor
 * @version             Introduced in firmware 4.0.10.27
 * @param   sensor      GoSensor object.
 * @param   direction   Direction with which to step.
 * @return              Operation status.
 */
GoFx(kStatus) GoSensor_PlaybackStep(GoSensor sensor, GoSeekDirection direction);

/**
 * Sets the current frame position for a replay.
 *
 * @public              @memberof GoSensor
 * @version             Introduced in firmware 4.0.10.27
 * @param   sensor      GoSensor object.
 * @param   position    The frame position to seek.
 * @return              Operation status.
 */
GoFx(kStatus) GoSensor_PlaybackSeek(GoSensor sensor, kSize position);

/**
 * Gets the current replay frame position.
 *
 * @public              @memberof GoSensor
 * @version             Introduced in firmware 4.0.10.27
 * @param   sensor      GoSensor object.
 * @param   position    The current frame position index.
 * @param   count       The frame count.
 * @return              Operation status.
 */
GoFx(kStatus) GoSensor_PlaybackPosition(GoSensor sensor, kSize* position, kSize* count);

/**
 * Clears the replay buffer.
 *
 * @public              @memberof GoSensor
 * @version             Introduced in firmware 4.0.10.27
 * @param   sensor      GoSensor object.
 * @return              Operation status.
 */
GoFx(kStatus) GoSensor_ClearReplayData(GoSensor sensor);

/**
 * Resets the measurement statistics reported by the health channel
 *
 * @public              @memberof GoSensor
 * @version             Introduced in firmware 4.0.10.27
 * @param   sensor      GoSensor object.
 * @return              Operation status.
 */
GoFx(kStatus) GoSensor_ClearMeasurementStats(GoSensor sensor);

/**
 * Exports the current frame of a replay in the form of a bitmap.
 *
 * @public              @memberof GoSensor
 * @version             Introduced in firmware 4.0.10.27
 * @param   sensor      GoSensor object.
 * @param   type        The type of data to export.
 * @param   source      The device data source to export from.
 * @param   dstFileName The destination file name of the exported bitmap file.
 * @return              Operation status.
 */
GoFx(kStatus) GoSensor_ExportBitmap(GoSensor sensor,
                                    GoReplayExportSourceType type,
                                    GoDataSource source,
                                    const kChar* dstFileName);

/**
 * Exports replay data in CSV format.
 *
 * @public              @memberof GoSensor
 * @version             Introduced in firmware 4.0.10.27
 * @param   sensor      GoSensor object.
 * @param   dstFileName The destination file name of the exported CSV file.
 * @return              Operation status.
 */
GoFx(kStatus) GoSensor_ExportCsv(GoSensor sensor, const kChar* dstFileName);

/**
 * Returns an enumerator value representing the current sensor's family.
 *
 * @public              @memberof GoSensor
 * @version             Introduced in firmware 4.0.10.27
 * @param   sensor      GoSensor object.
 * @return              A GoFamily value.
 */
GoFx(GoFamily) GoSensor_Family(GoSensor sensor);

/**
 * Clears the log file (_live.log).
 *
 * @public              @memberof GoSensor
 * @version             Introduced in firmware 4.0.10.27
 * @param   sensor      GoSensor object.
 * @return              Operation status.
 */
GoFx(kStatus) GoSensor_ClearLog(GoSensor sensor);

/**
 * Sets the AutoStart enabled state of the sensor.
 *
 * @public              @memberof GoSensor
 * @version             Introduced in firmware 4.0.10.27
 * @param   sensor      GoSensor object.
 * @param   enable      The AutoStart enabled state to use.
 * @return              Operation status.
 */
GoFx(kStatus) GoSensor_EnableAutoStart(GoSensor sensor, kBool enable);

/**
 * Gets the AutoStart enabled state currently used by the sensor.
 *
 * @public              @memberof GoSensor
 * @version             Introduced in firmware 4.0.10.27
 * @param   sensor      GoSensor object.
 * @return              kTRUE if auto start is enabled and kFALSE otherwise.
 */
GoFx(kBool) GoSensor_AutoStartEnabled(GoSensor sensor);

/**
* Set sensor voltage settings (only on G3210)
*
* @public              @memberof GoSensor
* @version             Introduced in firmware 4.7.5.25
* @param   sensor      GoSensor object.
* @param   voltage     Either 48V or 24V operation
* @param   cableLength When in 24V operation mode the cable length (meter) must also be supplied
* @return              Operation status.
*/
GoFx(kStatus) GoSensor_SetVoltage(GoSensor sensor, GoVoltageSetting voltage, k64f cableLength);

/**
* Get the sensor voltage settings (only on G3210)
*
* @public              @memberof GoSensor
* @version             Introduced in firmware 4.7.5.25
* @param   sensor      GoSensor object.
* @param   voltage     Destination to store voltage (can be kNULL)
* @param   cableLength Destination to store cable length (meter) (can be kNULL)
* @return              Operation status.
*/
GoFx(kStatus) GoSensor_GetVoltage(GoSensor sensor, GoVoltageSetting *voltage, k64f *cableLength);

/**
* Sets the quick edit state of the sensor.
*
* @public              @memberof GoSensor
* @version             Introduced in firmware 4.7.11.5
* @param   sensor      GoSensor object.
* @param   enable      The Quick Edit enabled state to use.
* @return              Operation status.
*/
GoFx(kStatus) GoSensor_EnableQuickEdit(GoSensor sensor, kBool enable);

/**
* Gets the quick edit state of the sensor
*
* @public              @memberof GoSensor
* @version             Introduced in firmware 4.7.11.5
* @param   sensor      GoSensor object.
* @return              kTRUE if Quick Edit is enabled, kFALSE otherwise.
*/
GoFx(kBool) GoSensor_QuickEditEnabled(GoSensor sensor);

/**
 * Gets the count of remote sensor information held by the sensor.
 *
 * @public              @memberof GoSensor
 * @version             Introduced in firmware 4.0.10.27
 * @param   sensor      GoSensor object.
 * @return              The remote sensor information count.
 */
GoFx(kSize) GoSensor_RemoteInfoCount(GoSensor sensor);

/**
 * Gets the remote sensor information at the given index.
 *
 * @public              @memberof GoSensor
 * @version             Introduced in firmware 4.0.10.27
 * @param   sensor      GoSensor object.
 * @param   index       The index of the remote sensor information to retrieve.
 * @return              A handle to the selected remote sensor information or kNULL if an invalid index is provided.
 */
GoFx(GoSensorInfo) GoSensor_RemoteInfoAt(GoSensor sensor, kSize index);

/**
 * Return the number of files contained in the specified path with an optional extension filter applied.
 *
 * @public                      @memberof GoSensor
 * @version             Introduced in firmware 4.1.3.106
 * @param   sensor              GoSensor object.
 * @param   extensionFilter     An optional extension filter to apply. Send "" to not apply a filter.
 * @param   path                The file system path to retrieve the file count for.
 * @param   isRecursive         kTRUE to include files in sub-folders of the path and kFALSE to only count files in the immediate path.
 * @return                      The number of files contained in the specified path with an optional extension filter applied.
 */
GoFx(kSize) GoSensor_DirectoryFileCount(GoSensor sensor, const kChar* extensionFilter, const kChar* path, kBool isRecursive);

/**
 * Retrieves the file name at the specified index for a given path and optional extension filter.
 *
 * @public                      @memberof GoSensor
 * @version                     Introduced in firmware 4.1.3.106
 * @param   sensor              GoSensor object.
 * @param   extensionFilter     An optional extension filter to apply. Send "" to not apply a filter.
 * @param   path                The file system path to retrieve the file count for.
 * @param   isRecursive         kTRUE to include files in sub-folders of the path and kFALSE to only count files in the immediate path.
 * @param   index               The index of the file name to retrieve.
 * @param   fileName            The pointer to a character array of which to store the retrieve file name.
 * @param   capacity            The maximum capacity of the character array.
 * @return                      Operation status.
 */
GoFx(kStatus) GoSensor_DirectoryFileNameAt(GoSensor sensor, const kChar* extensionFilter, const kChar* path, kBool isRecursive, kSize index, kChar* fileName, kSize capacity);

/**
 * Defines the signature for a custom data message handler.
 *
 * @public              @memberof GoSensor
 * @version             Introduced in firmware 4.1.3.106
 * @param   context     A pointer to an application context to use with the provided data set.
 * @param   sensor      GoSensor object.
 * @param   dataSet     The data set.
 * @return              Operation status.
 * @see                 GoSensor_SetDataHandler
 */
typedef kStatus (kCall* GoSensorDataSetFx)(kPointer context, GoSensor sensor, GoDataSet dataSet);

/**
 * Sets the data callback function to be used upon receipt of data.
 *
 * @public              @memberof GoSensor
 * @version             Introduced in firmware 4.1.3.106
 * @param   sensor      GoSensor object.
 * @param   function    The function pointer to use when data has been received from the sensor. Send kNULL to revert back to the GoSystem data set storage behavior.
 * @param   context     The context to use with the function pointer.
 * @return              Operation status.
 * @see                 GoSensorDataSetFx
 */
GoFx(kStatus) GoSensor_SetDataHandler(GoSensor sensor, GoSensorDataSetFx function, kPointer context);


/**
 * Creates a part matching model based on the current part data.
 *
 * @public              @memberof GoSensor
 * @version             Introduced in firmware 4.2.4.7
 * @param   sensor      GoSensor object.
 * @param   name        The intended name of the part match model.
 * @return              Operation status.
 */
GoFx(kStatus) GoSensor_PartMatchCreateModel(GoSensor sensor, const kChar* name);

/**
 * Detect the edges of the specified part model.
 *
 * @public              @memberof GoSensor
 * @version             Introduced in firmware 4.2.4.7
 * @param   sensor      GoSensor object.
 * @param   name        The name of the part match model to detect edges on.
 * @param   sensitivity The sensitivity to use for model edge detection.
 * @return              Operation status.
 */
GoFx(kStatus) GoSensor_PartMatchDetectModelEdges(GoSensor sensor, const kChar* name, k16u sensitivity);

/**
 * Returns a handle to a part model based on a given name.
 *
 * @public              @memberof GoSensor
 * @version             Introduced in firmware 4.2.4.7
 * @param   sensor      GoSensor object.
 * @param   name        The name of the part match model to retrieve.
 * @return              A GoPartModel handle.
 */
GoFx(GoPartModel) GoSensor_PartMatchModel(GoSensor sensor, const kChar* name);

/**
 * Returns the number of part match models present in the currently loaded job.
 *
 * @public              @memberof GoSensor
 * @version             Introduced in firmware 4.2.4.7
 * @param   sensor      GoSensor object.
 * @return              The count of part match models in the currently loaded job.
 */
GoFx(kSize) GoSensor_PartMatchModelCount(GoSensor sensor);

/**
 * Returns a handle to a part model based on a given index.
 *
 * @public              @memberof GoSensor
 * @version             Introduced in firmware 4.2.4.7
 * @param   sensor      GoSensor object.
 * @param   index       The index of the part model to retrieve.
 * @return              A GoPartModel handle.
 */
GoFx(GoPartModel) GoSensor_PartMatchModelAt(GoSensor sensor, kSize index);

/**
 * Sets the runtime variables from the provided starting index to the specified length
 * with the values contained in the provided array.
 *
 * @public              @memberof GoSensor
 * @version             Introduced in firmware 4.5.3.57
 * @param   sensor      GoSensor object.
 * @param   startIndex  The starting index of the runtime variable values to set.
 * @param   length      The number of runtime variables to set/the length of the array parameter.
 * @param   values      A reference to an array which contains the intended runtime variable values.
 * @return              Operation status.
 */
GoFx(kStatus) GoSensor_SetRuntimeVariables(GoSensor sensor, kSize startIndex, kSize length, k32s* values);

/**
 * Returns the number of runtime variables available on the device.
 *
 * @public              @memberof GoSensor
 * @version             Introduced in firmware 4.5.3.57
 * @param   sensor      GoSensor object.
 * @return              The runtime variable count of the device.
 */
GoFx(kSize) GoSensor_GetRuntimeVariableCount(GoSensor sensor);

/**
 * Gets the values associated with a given runtime variable starting index and length.
 *
 * @public              @memberof GoSensor
 * @version             Introduced in firmware 4.5.3.57
 * @param   sensor      GoSensor object.
 * @param   startIndex  The starting index of the runtime variable values to retrieve.
 * @param   length      The number of runtime variables to retrieve.
 * @param   values      A reference to an array which will hold the retrieved runtime variable values.
 * @return              Operation status.
 */
GoFx(kStatus) GoSensor_GetRuntimeVariables(GoSensor sensor, kSize startIndex, kSize length, k32s* values);

/**
 * Gets the value associated with a given runtime variable index.
 *
 * @public              @memberof GoSensor
 * @version             Introduced in firmware 4.5.3.57
 * @param   sensor      GoSensor object.
 * @param   index       The index of the runtime variable value to retrieve.
 * @param   value       A reference to be updated with the runtime variable value.
 * @return              Operation status.
 */
GoFx(kStatus) GoSensor_GetRuntimeVariableAt(GoSensor sensor, kSize index, k32s* value);

/**
 * Starts recording to a stream.
 *
 * NOTE: Before calling this function, the sensor should already be running and
 *       recording. GoSensor_StopRecordingStream() should be called before executing
 *       any other sensor commands, with the exception of GoSensor_IsRecordingStreaming().
 *
 * @public              @memberof GoSensor
 * @version             Introduced in firmware 4.6.4.66
 * @param   sensor      GoSensor object.
 * @param   destFile    A local file path to store the .rec data.
 * @return              Operation status.
 */
GoFx(kStatus) GoSensor_StartRecordingStream(GoSensor sensor, kChar* destFile);

/**
 * Stops recording to a stream.
 *
 * @public              @memberof GoSensor
 * @version             Introduced in firmware 4.6.4.66
 * @param   sensor      GoSensor object.
 * @return              Operation status.
 */
GoFx(kStatus) GoSensor_StopRecordingStream(GoSensor sensor);

/**
 * Reports whether or not recording is streaming.
 *
 * @public              @memberof GoSensor
 * @version             Introduced in firmware 4.6.4.66
 * @param   sensor      GoSensor object.
 * @return              kTRUE if recording is streaming, kFALSE otherwise.
 */
GoFx(kBool) GoSensor_IsRecordingStreaming(GoSensor sensor);

/**
* Gets the buddy at a given index.
*
* @public              @memberof GoSensorInfo
* @version             Introduced in firmware 4.6.4.66
* @param   info        GoSensorInfo object.
* @param   index       index of buddy to retrieve.
* @return              Device state.
*/
GoFx(GoSensorInfo) GoSensor_BuddiesAt(GoSensor info, k32u index);

/**
* Gets the number of buddies in the sytem.
*
* @public              @memberof GoSensor
* @version             Introduced in firmware 4.6.4.66
* @param   info        GoSensor object.
* @return              Device state.
*/
GoFx(kSize) GoSensor_BuddiesCount(GoSensor info);

/**
* returns true if the system has any buddies.
*
* @public              @memberof GoSensor
* @version             Introduced in firmware 4.6.4.66
* @param   info        GoSensor object.
* @return              Device state.
*/
GoFx(kBool) GoSensor_HasBuddies(GoSensor info);

/**
* returns the GeoCal object for querying or Null if it does not exist.
*
* @public              @memberof GoSensor
* @version             Introduced in firmware 4.7.0.130
* @param   info        GoSensor object.
* @param   geoCal      Output: A reference to be updated with the GoGeoCal object
* @return              Status of operation
*/
GoFx(kStatus) GoSensor_GeoCal(GoSensor info, GoGeoCal *geoCal);

/**
 ** Sets the Data Port.
 *
 * @public              @memberof GoSensor
 * @version             Introduced in firmware 4.8.2.76
 * @param   sensor      GoSensor object.
 * @param   port        Port number to be used
 * @return              Status of operation
 */
GoFx(kStatus) GoSensor_SetDataPort(GoSensor sensor, k32u port);

/**
 * returns the Data Port.
 *
 * @public              @memberof GoSensor
 * @version             Introduced in firmware 4.8.2.76
 * @param   sensor      GoSensor object.
 * @return              Port Number
 */
GoFx(k32u) GoSensor_DataPort(GoSensor sensor);

/**
 * Sets the Health Port.
 *
 * @public              @memberof GoSensor
 * @version             Introduced in firmware 4.8.2.76
 * @param   sensor      GoSensor object.
 * @param   port        Port number to be used
 * @return              Status of operation
 */
GoFx(kStatus) GoSensor_SetHealthPort(GoSensor sensor, k32u port);

/**
 * returns the Health Port.
 *
 * @public              @memberof GoSensor
 * @version             Introduced in firmware 4.8.2.76
 * @param   sensor      GoSensor object.
 * @return              Port Number
 */
GoFx(k32u) GoSensor_HealthPort(GoSensor sensor);

/**
 ** Sets the Control Port.
 *
 * @public              @memberof GoSensor
 * @version             Introduced in firmware 4.8.2.76
 * @param   sensor      GoSensor object.
 * @param   port        Port number to be used
 * @return              Status of operation
 */
GoFx(kStatus) GoSensor_SetControlPort(GoSensor sensor, k32u port);

/**
 * returns the Control Port.
 *
 * @public              @memberof GoSensor
 * @version             Introduced in firmware 4.8.2.76
 * @param   sensor      GoSensor object.
 * @return              Port Number
 */
GoFx(k32u) GoSensor_ControlPort(GoSensor sensor);

/**
 * Sets the Upgrade Port.
 *
 * @public              @memberof GoSensor
 * @version             Introduced in firmware 4.8.2.76
 * @param   sensor      GoSensor object.
 * @param   port        Port number to be used
 * @return              Status of operation
 */
GoFx(kStatus) GoSensor_SetUpgradePort(GoSensor sensor, k32u port);

/**
 * returns the Upgrade Port.
 *
 * @public              @memberof GoSensor
 * @version             Introduced in firmware 4.8.2.76
 * @param   sensor      GoSensor object.
 * @return              Port Number
 */
GoFx(k32u) GoSensor_UpgradePort(GoSensor sensor);

/**
* Waits for all buddies to be connected within a specific timeout.
*
* @public              @memberof GoSensor
 * @version             Introduced in firmware 5.2.18.3
* @param   sensor      GoSensor object.
* @param   timeout     a timeout in milliseconds to wait for all required buddies to get connected
* @return              Operational status: kOK if all required buddies are connected within specified timeout; otherwise kERROR_TIMEOUT
*/
GoFx(kStatus) GoSensor_WaitForBuddies(GoSensor sensor, k64u timeout);

/**
* Connects and logs into the sensor using the specified user name and password.
* Once security protection is enabled logging in is required in order to programmatically control a sensor.
* This must be used to connect to the sensor instead of anonymous connect method that fails in case security is enabled.
*
* @public              @memberof GoSensor
 * @version             Introduced in firmware 5.2.18.3
* @param   sensor      GoSensor object.
* @param   user        User account
* @param   password    User password
* @return              Operational status: kOK if funcions could complete with given login information;
*                      otherwise GS_ERROR_AUTHENTICATION indicating unauthorized access
*/
GoFx(kStatus) GoSensor_ConnectAndLogin(GoSensor sensor, GoUser user, const kChar* password);

/**
* Sets sensor's security level. This will enabled/disable security protection.
* Only authorized users can change security level that can deny access to specific objects/files to anonymous users
*
* @public              @memberof GoSensor
 * @version             Introduced in firmware 5.2.18.3
* @param   sensor      GoSensor object.
* @param   security    new security level
* @return              Operational status: kOK if funcions could complete with given login information;
*                      otherwise GS_ERROR_AUTHENTICATION indicating unauthorized access
*/
GoFx(kStatus) GoSensor_SetSecurityLevel(GoSensor sensor, GoSecurityLevel security);

/**
 * Sets a sensor flag value.
 *
 * Sensor flags are an advanced feature that gives the user control over some internal
 * system parameters and experimental features. These flags are provided only for the
 * purposes of experimentation and demonstration, and may be changed or removed from
 * one firwmare version to another.
 *
 * @public              @memberof GoSensor
 * @version             Introduced in firmware 5.2.18.3
 * @param   sensor      GoControl object.
 * @param   name        Name of the flag.
 * @param   value       Value of the flag (in text).
 * @return              Operation status.
 */
GoFx(kStatus) GoSensor_SetFlag(GoSensor sensor, const kChar* name, const kChar* value);

/**
 * Gets a sensor flag value. See GoSensor_SetFlag for more details.
 *
 * @public              @memberof GoSensor
 * @version             Introduced in firmware 5.2.18.3
 * @param   sensor      GoControl object.
 * @param   name        Name of the flag.
 * @param   value       String object to receive the value (in text).
 * @return              Operation status.
 */
GoFx(kStatus) GoSensor_GetFlag(GoSensor sensor, const kChar* name, kString value);

/**
* Returns the acceleration state of a sensor. The state can be used by
* acceleration applications to determine if the
* sensor is available for acceleration or is already accelerated by a host.
*
* @public              @memberof GoSensor
 * @version             Introduced in firmware 5.2.18.3
* @param   sensor      GoSensor object.
* @return              Acceleration state.
*/
GoFx(GoSensorAccelState) GoSensor_AccelState(GoSensor sensor);

/**
* Returns the port numbers used by an accelerated sensor, including web port.
* Can be used by acceleration applications to get the ports used by an
* accelerated sensor.
*
* @public              @memberof GoSensor
 * @version             Introduced in firmware 5.2.18.3
* @param   sensor      GoSensor object.
* @return              Port numbers used by sensor.
*/
GoFx(GoPortInfo) GoSensor_AccelPortInfo(GoSensor sensor);

/**
* Returns the operational mode of the sensor. This can be used by an
* acceleration application to determine if the sensor is a virtual sensor,
* standalone sensor or an accelerated sensor mode.
*
* @public              @memberof GoSensor
 * @version             Introduced in firmware 5.2.18.3
* @param   sensor      GoSensor object.
* @return              Sensor operational mode.
*/
GoFx(GoDiscoveryOpMode) GoSensor_AccelOpMode(GoSensor sensor);

/**
* Returns the physical sensor IP address when sensor is accelerated.
*
* @public              @memberof GoSensor
 * @version             Introduced in firmware 5.2.18.3
* @param   sensor      GoSensor object.
* @param   ipAddress   Destination to store the IP Address.
* @return              Operational status.
*/
GoFx(kStatus) GoSensor_AccelSensorIpAddress(GoSensor sensor, kIpAddress* ipAddress);

#include <GoSdk/GoSensor.x.h>

#endif

//
// GoSensor.h
//
// Copyright (C) 2016-2021 by LMI Technologies Inc.
// Licensed under the MIT License.
// Redistributed files must retain the above copyright notice.
//
#ifndef GO_SDK_NET_SENSOR_H
#define GO_SDK_NET_SENSOR_H

#include <GoSdk/GoSensor.h>
#include <GoSdk/GoSystem.h>
#include <GoSdkNet/GoSetup.h>
#include <GoSdkNet/Outputs/GoOutput.h>
#include <GoSdkNet/GoTransform.h>
#include <GoSdkNet/GoGeoCal.h>
#include <GoSdkNet/GoSensorInfo.h>
#include <GoSdkNet/GoPartModel.h>
#include <GoSdkNet/Tools/GoTools.h>
#include <GoSdkNet/GoSdkDef.h>
#include <GoSdkNet/GoReplay.h>
#include <GoSdkNet/Messages/GoDiscoveryExtInfo.h>

namespace Lmi3d
{
    namespace GoSdk
    {
        /// <summary>Represents a sensor.</summary>
        public ref class GoSensor : public KObject
        {
            KDeclareClass(GoSensor, GoSensor)

            /// <summary>Initializes a new instance of the GoSensor class with the specified Zen object handle.</summary>
            /// <param name="handle">Zen object handle.</param>
            GoSensor(IntPtr handle)
                : KObject(handle)
            {}

            /// <summary>Initiates a sensor configuration, model file, and transformation synchronization if modifications are present.</summary>
            void Flush()
            {
                KCheck(::GoSensor_Flush(Handle));
            }

            /// <summary>
            /// Configures a sensor's network address settings.
            /// WARNING! This function writes to flash storage. Review the user manual for implications.
            /// </summary>
            /// <remarks>
            /// This function uses UDP broadcasts for sensor configuration; the sensor does not need to
            /// be connected, and can be on a different subnet than the client.
            ///
            /// The sensor will automatically reboot if the address is successfully changed.
            ///
            /// This function results in flash storage modifications. If modifications are interrupted due to
            /// power loss, the sensor may reboot into Rescue mode.
            /// </remarks>
            ///
            /// <param name="addressInfo">New address settings.</param>
            /// <param name="wait">Should this function block until the sensor finishes rebooting?</param>
            void SetAddress(GoAddressInfo addressInfo, bool wait)
            {
                ::GoAddressInfo info = addressInfo.ToNative();
                KCheck(::GoSensor_SetAddress(Handle, &info, wait));
            }

            /// <summary>Retrieves the sensor's network address settings.</summary>
            ///
            /// <returns>Network address.</returns>
            GoAddressInfo Address()
            {
                ::GoAddressInfo info;

                KCheck(::GoSensor_Address(Handle, &info));

                return GoAddressInfo(&info);
            }

            /// <summary>Creates a connection to the sensor.</summary>
            void Connect()
            {
                KCheck(::GoSensor_Connect(Handle));
            }

            /// <summary>Disconnects from the sensor.</summary>
            void Disconnect()
            {
                KCheck(::GoSensor_Disconnect(Handle));
            }

            /// <summary> Reports whether the sensor is currently connected. If sensors are temporarily unreachable, they do not leave
            ///    isConnected state.You can use IsResponsive for this and optionaly call Disconnect if
            ///     unresponsive for too long. </summary>
            ///
            /// <returns>true if the sensor is connected; false otherwise.</returns>
            bool IsConnected()
            {
                return KToBool(::GoSensor_IsConnected(Handle));
            }

            /// <summary>Reports whether the sensor is currently connected.</summary>
            ///
            /// <returns>true if the sensor is connected; false otherwise.</returns>
            bool IsResponsive()
            {
                return KToBool(::GoSensor_IsResponsive(Handle));
            }

            /// <summary>Reports whether the connected sensor's protocol version is compatible with the SDK's protocol version.</summary>
            ///
            /// <returns>true if the sensor is compatible; false otherwise.</returns>
            bool IsCompatible()
            {
                return KToBool(::GoSensor_IsCompatible(Handle));
            }

            /// <summary>Refreshes sensor state.</summary>
            ///
            /// <remarks>
            /// Unresponsive sensors will be disconnected, and cancelled sensors will be reconnected.
            /// Sensors in any other state will discard all locally-cached information.
            ///
            /// This function should be used to update sensors in the GoState.Inconsistent state. This
            /// state can arise due to buddy changes performed by remote sensors (e.g. a main sensor boots
            /// and claims ownership of a buddy sensor, but the buddy sensor has already been detected and
            /// loaded as a main sensor by the client).
            /// </remarks>
            ///
            void Refresh()
            {
                KCheck(::GoSensor_Refresh(Handle));
            }

            /// <summary>Get a buddy by index.</summary>
            ///
            /// <returns>true if sensor has a buddy; false otherwise.</returns>
            GoSensorInfo^ BuddiesAt(k32u index)
            {
                ::GoSensorInfo info = ::GoSensor_BuddiesAt(Handle, index);

                if (kIsNull(info)) { return nullptr; }

                return KToObject<GoSensorInfo^>(info);
            }

            /// <summary>Get number of buddies.</summary>
            ///
            /// <returns>true if sensor has a buddy; false otherwise.</returns>
            kSize BuddiesCount()
            {
                return ::GoSensor_BuddiesCount(Handle);
            }

            /// <summary>Retunrs true if sensor has buddies.</summary>
            ///
            /// <returns>true if sensor has a buddy; false otherwise.</returns>
            bool HasBuddies()
            {
                return ::GoSensor_HasBuddies(Handle);
            }

            /// <summary>Assigns a buddy sensor.</summary>
            ///
            /// <remarks>
            /// NOTE: The provided buddy sensor handle must already be connected.
            /// </remarks>
            ///
            void AddBuddy(GoSensor^ buddy)
            {
                KCheck(::GoSensor_AddBuddy(Handle, buddy->Handle));
            }

            /// <summary>Removes the current buddy sensor.</summary>
            void RemoveBuddy()
            {
                KCheck(::GoSensor_RemoveBuddy(Handle));
            }

            /// <summary>Reports whether a buddy had been assigned.</summary>
            ///
            /// <returns>true if sensor has a buddy; false otherwise.</returns>
            bool HasBuddy()
            {
                return KToBool(::GoSensor_HasBuddy(Handle));
            }

            /// <summary>Buddy device ID (or k32u::Null if not assigned).</summary>
            property k32u BuddyId
            {
                k32u get() { return ::GoSensor_BuddyId(Handle); }
            }

            /// <summary>Gets the sensor's scan mode.</summary>
            property GoMode ScanMode
            {
                GoMode get() { return (GoMode) ::GoSensor_ScanMode(Handle); }
            }

            /// <summary>Enables or disables the sensor's data channel.</summary>
            ///
            /// <remarks>
            /// After using EnableData function, SDKNet application is responsible for disposing of the GoDataSet objects that hold the data received from the sensor on the data connection.
            /// </remarks>
            ///
            /// <param name="enable">true to enable, or false to disable.</param>
            void EnableData(bool enable)
            {
                KCheck(::GoSensor_EnableData(Handle, enable));
            }

            /// <summary>Starts the sensor.</summary>
            void Start()
            {
                KCheck(::GoSensor_Start(Handle));
            }

            bool CanStart()
            {
                return(::GoSensor_CanStart(Handle));
            }

            /// <summary>Starts the sensor at a scheduled value relating to the current time or encoder value as reported by the sensor.</summary>
            ///
            /// <param name="value">Scheduled start value. uS when time triggered and ticks when encoder triggered.</param>
            void ScheduledStart(k64s value)
            {
                KCheck(::GoSensor_ScheduledStart(Handle, value));
            }

            /// <summary>Stops the sensor.</summary>
            void Stop()
            {
                KCheck(::GoSensor_Stop(Handle));
            }

            /// <summary>Performs a sensor snapshot.</summary>
            void Snapshot()
            {
                KCheck(::GoSensor_Snapshot(Handle));
            }

            /// <summary>Perform alignment using the configured alignment type and target.</summary>
            ///
            /// <remarks>
            /// NOTE: This operation will result in a sensor start for the duration of the
            /// alignment. It can be canceled via GoSensor_Stop. This function's operation
            /// status does not correspond to the actual alignment result. In order to
            /// retrieve the alignment result, you must enable the data channel before calling
            /// this function, receive an alignment data message and then check its status.
            ///
            /// Calling this function will result in writing operations to flash storage. Should the process be disrupted due to power loss or other factors, the sensor may enter Rescue mode.
            /// </remarks>
            ///
            void Align()
            {
                KCheck(::GoSensor_Align(Handle));
            }

            /// <summary>
            /// Clears the current sensor alignment.
            /// WARNING! This function writes to flash storage. Review the user manual for implications.
            /// </summary>
            /// <remarks>
            /// Calling this function will result in writing operations to flash storage. Should the process be disrupted due to power loss or other factors, the sensor may enter Rescue mode.
            /// </remarks>
            ///
            void ClearAlignment()
            {
                KCheck(::GoSensor_ClearAlignment(Handle));
            }

            /// <summary>Perform an exposure auto set.</summary>
            ///
            /// <remarks>
            /// NOTE: This operation will result in a sensor start for the duration of the
            /// exposure AutoSet. A successful operation status does NOT modify the configuration.
            /// You must retrieve the resulting exposure value and set it for the appropriate
            /// exposure setting. This involves enabling the data connection prior to running
            /// exposure auto set and then receiving an exposure auto set message, which
            /// you can then use to query the status and access the resulting value.
            /// </remarks>
            ///
            void ExposureAutoSet(GoRole role)
            {
                KCheck(::GoSensor_ExposureAutoSet(Handle, role));
            }

            /// <summary>Gets the number of files available from the connected sensor.</summary>
            property GoAlignmentState AlignmentState
            {
                GoAlignmentState get() { return (GoAlignmentState) ::GoSensor_AlignmentState(Handle); }
            }

            /// <summary>The alignment reference of the sensor.</summary>
            property GoAlignmentRef AlignmentReference
            {
                GoAlignmentRef get()
                {
                   ::GoAlignmentRef reference;
                   KCheck(::GoSensor_AlignmentReference(Handle, &reference));
                   return (GoAlignmentRef) reference;
                }

                void set(GoAlignmentRef reference)  { KCheck(::GoSensor_SetAlignmentReference(Handle, reference)); }
            }

            /// <summary>Reboots the sensor.</summary>
            ///
            /// <param name="wait">true to wait for reboot and then reconnect.</param>
            void Reset(bool wait)
            {
                KCheck(::GoSensor_Reset(Handle, wait));
            }

            /// <summary>Resets the encoder value.</summary>
            /// <remarks>
            /// NOTE: This is only possible with a direct encoder connection to a sensor.
            /// Resetting the encoder value when connected to a Master device will not work.
            /// </remarks>
            void ResetEncoder()
            {
                KCheck(::GoSensor_ResetEncoder(Handle));
            }

            /// <summary>Aborts ongoing sensor communication.</summary>
            ///
            /// <remarks>
            /// This method asynchronously aborts ongoing communication; the next time that any
            /// I/O operation blocks for an extended period of time, it will be terminated.  This method
            /// is thread-safe.
            ///
            /// In order to resume communication, call the function Refresh or Connect.
            /// </remarks>
            ///
            void Cancel()
            {
                KCheck(::GoSensor_Cancel(Handle));
            }

            /// <summary>Gets the current time stamp (common among all synchronized sensors).</summary>
            ///
            /// <returns>Timestamp value.</returns>
            k64s Timestamp()
            {
                k64u time;

                KCheck(::GoSensor_Timestamp(Handle, &time));

                return time;
            }

            /// <summary>Gets the current encoder count.</summary>
            ///
            /// <returns>Encoder value.</returns>
            k64s Encoder()
            {
                k64s encoder;

                KCheck(::GoSensor_Encoder(Handle, &encoder));

                return encoder;
            }

            /// <summary>Sends a software trigger to the sensor.</summary>
            ///
            /// <remarks>
            /// This method is used in conjunction with sensors that are configured to accept
            /// software triggers. The sensor must be running (e.g. by calling GoSensor_Start)
            /// for triggers to be accepted.
            /// 
            /// When the trigger mode is set to Software, this command will trigger individual 
            /// frames in Profile or Surface mode. For G2 sensors with other trigger modes, 
            /// this command can also be used to trigger Fixed Length surface generation when 
            /// the Fixed Length Start Trigger option is set to "Software".
            /// </remarks>
            ///
            void Trigger()
            {
                KCheck(::GoSensor_Trigger(Handle));
            }

            /// <summary>Schedules a digital output.</summary>
            ///
            /// <remarks>
            /// This method requires that the output is configured to trigger on software control.
            /// </remarks>
            ///
            /// <param name="index">The digital output index.</param>
            /// <param name="target">The time or position target (us or mm), depending on GoDomain. Ignored if
            /// GoDigital_ScheduleEnabled is false or GoDigital_SignalType is pulsed.</param>
            /// <param name="value">The value of scheduled output (0-Low or 1-High). Ignored if output
            /// GoDigital_SignalType is pulsed.</param>
            void EmitDigital(k16u index, k64s target, k8u value)
            {
                KCheck(::GoSensor_EmitDigital(Handle, index, target, value));
            }

            /// <summary>Schedules an analog output.</summary>
            ///
            /// <remarks>
            /// This method requires that the output is configured to trigger on software control.
            ///
            /// NOTE: It is not possible to schedule a continuous output. The operation will
            /// fail accordingly if attempted.
            /// </remarks>
            ///
            /// <param name="index">The analog output index.</param>
            /// <param name="target">The time or position target (us or mm), depending on GoDomain. Ignored if
            /// GoAnalog_ScheduleEnabled is false.</param>
            /// <param name="value">The value of the scheduled output (mA).</param>
            void EmitAnalog(k16u index, k64s target, k32s value)
            {
                KCheck(::GoSensor_EmitAnalog(Handle, index, target, value));
            }

            /// <summary>Gets the number of files available from the connected sensor.</summary>
            property k32s FileCount
            {
                k32s get() { return (k32s) ::GoSensor_FileCount(Handle); }
            }

            /// <summary>Gets the file name at the specified index.</summary>
            ///
            /// <param name="index">Index of the desired file name.</param>
            /// <returns>File Name.</returns>
            String^ GetFileName(k64s index)
            {
                kText256 fileName;

                KCheck(::GoSensor_FileNameAt(Handle, (kSize)index, fileName, kCountOf(fileName)));

                return KToString(fileName);
            }

            /// <summary>
            /// Uploads a file to the connected sensor.
            /// WARNING! This function writes to flash storage. Review the user manual for implications.
            /// </summary>
            /// <remarks>
            /// This function results in flash storage modifications. If modifications are interrupted due to
            /// power loss, the sensor may reboot into Rescue mode.
            /// </remarks>
            ///
            /// <param name="sourcePath">Source file system path for the file to be uploaded.</param>
            /// <param name="destName">Destination name for the uploaded file (maximum 63 characters).</param>
            void UploadFile(String^ sourcePath, String^ destName)
            {
                KString strA(sourcePath);
                KString strB(destName);

                KCheck(::GoSensor_UploadFile(Handle, strA.CharPtr, strB.CharPtr));
            }

            /// <summary>Downloads a file from the connected sensor.</summary>
            ///
            /// <param name="sourceName">Source name of the file to be downloaded.</param>
            /// <param name="destPath">Destination file system path for the file to be downloaded.</param>
            void DownloadFile(String^ sourceName, String^ destPath)
            {
                KString strA(sourceName);
                KString strB(destPath);

                KCheck(::GoSensor_DownloadFile(Handle, strA.CharPtr, strB.CharPtr));
            }

            /// <summary>
            /// Copies a file within the connected sensor.
            ///
            /// WARNING! This function writes to flash storage. Review the user manual for implications.
            /// </summary>
            /// <remarks>
            /// This function can result in flash storage modifications if the destination is not a live file (e.g. "_live.job").
            /// If modifications are interrupted due to power loss, the sensor may reboot into Rescue mode.
            ///
            /// To upload files to the sensor, use GoSensor_UploadFile().
            /// </remarks>
            ///
            /// <param name="sourceName">Source name for the file to be copied.</param>
            /// <param name="destName">Destination name for the file (maximum 63 characters).</param>
            void CopyFile(String^ sourceName, String^ destName)
            {
                KString strA(sourceName);
                KString strB(destName);

                KCheck(::GoSensor_CopyFile(Handle, strA.CharPtr, strB.CharPtr));
            }

            /// <summary>
            /// Deletes a file within the connected sensor.
            /// WARNING! This function writes to flash storage. Review the user manual for implications.
            /// </summary>
            /// <remarks>
            /// This function results in flash storage modifications. If modifications are interrupted due to
            /// power loss, the sensor may reboot into Rescue mode.
            /// </remarks>
            ///
            /// <param name="name">Name of the file to be deleted.</param>
            void DeleteFile(String^ name)
            {
                KString strA(name);

                KCheck(::GoSensor_DeleteFile(Handle, strA.CharPtr));
            }

            /// <summary>Checks whether the specified file is present on the sensor.</summary>
            ///
            /// <param name="name">Name of the file to be checked.</param>
            /// <returns>true if the file exists; false otherwise.</returns>
            bool FileExists(String^ name)
            {
                KString strA(name);

                return KToBool(::GoSensor_FileExists(Handle, strA.CharPtr));
            }

            /// <summary>
            /// A default job file to be loaded on boot.
            /// WARNING! Setting the default job writes to flash storage. Review the user manual for implications.
            /// </summary>
            /// <remarks>
            /// Setting this property will result in writing operations to flash storage. Should the process be disrupted due to power loss or other factors, the sensor may enter Rescue mode.
            /// </remarks>
            ///
            property String^ DefaultJob
            {
                String^ get()
                {
                    kText256 fileName;

                    KCheck(::GoSensor_DefaultJob(Handle, fileName, kCountOf(fileName)));

                    return KToString(fileName);
                }

                void set(String^ fileName)
                {
                    KString str(fileName);

                    KCheck(::GoSensor_SetDefaultJob(Handle, str.CharPtr));
                }
            }

            /// <summary>Gets the name of the loaded job file and whether it has been modified since loading.</summary>
            ///
            /// <param name="changed">Receives the status of whether the file has changed.</param>
            void LoadedJob(String^% fileName, bool% changed)
            {
                kText256 fileNameA;
                kBool changedA;

                KCheck(::GoSensor_LoadedJob(Handle, fileNameA, kCountOf(fileNameA), &changedA));

                changed = KToBool(changedA);
                fileName = KToString(fileNameA);
            }

            /// <summary>Logs into the sensor using the specified user name and password.</summary>
            ///
            /// <remarks>
            /// Logging in is not required in order to programmatically control a sensor. The Gocator log-in feature is
            /// intended only to support administrative user interfaces, by allowing the username and password to be
            /// stored in the sensor.
            /// </remarks>
            ///
            /// <param name="user">User account.</param>
            /// <param name="password">User password.</param>
            void LogIn(GoUser user, String^ password)
            {
                KString str(password);

                KCheck(::GoSensor_LogIn(Handle, user, str.CharPtr));
            }

            /// <summary>Connects and logs into the sensor using the specified user name and password.</summary>
            ///
            /// <remarks>
            /// Once security protection is enabled logging in is required in order to programmatically control a sensor.
            /// This must be used to connect to the sensor instead of anonymous connect method that fails in case security is enabled.
            /// </remarks>
            ///
            /// <param name="user">User account.</param>
            /// <param name="password">User password.</param>
            void ConnectAndLogIn(GoUser user, String^ password)
            {
                KString str(password);

                KCheck(::GoSensor_ConnectAndLogin(Handle, user, str.CharPtr));
            }

            /// <summary>Sets sensor's security level.</summary>
            ///
            /// <remarks>
            /// This will enabled/disable security protection.
            /// Only authorized users can change security level that can deny access to specific objects/files to anonymous users
            /// </remarks>
            ///
            /// <param name="security">new security level rto be set.</param>
            void SetSecurityLevel(k32u security)
            {
                KCheck(::GoSensor_SetSecurityLevel(Handle, security));
            }

            /// <summary>
            /// Changes the password associated with the specified user account.
            /// WARNING! This function writes to flash storage. Review the user manual for implications.
            /// </summary>
            /// <remarks>
            /// Calling this function will result in writing operations to flash storage. Should the process be disrupted due to power loss or other factors, the sensor may enter Rescue mode.
            /// </remarks>
            ///
            /// <param name="user">User account.</param>
            /// <param name="password">User password.</param>
            void ChangePassword(GoUser user, String^ password)
            {
                KString str(password);

                KCheck(::GoSensor_ChangePassword(Handle, user, str.CharPtr));
            }

            /// <summary>Delegate for a GoSensor upgrade event handler.</summary>
            delegate void UpgradeFx(double progress);

            /// <summary>
            /// Upgrades sensor firmware.
            /// WARNING! This function writes to flash storage. Review the user manual for implications.
            /// </summary>
            /// <remarks>
            /// This function will block until the upgrade is completed.
            ///
            /// This function results in flash storage modifications. If modifications are interrupted due to
            /// power loss, the sensor may reboot into Rescue mode.
            ///
            /// The sensor does not need to be connected to perform an upgrade.
            /// </remarks>
            /// <param name="sourcePath">Local file system path to the upgrade file.</param>
            /// <param name="handler">Handler (can be null).</param>
            void Upgrade(String^ sourcePath, UpgradeFx^ handler)
            {
                KString strA(sourcePath);
                KCallbackFx^ thunk = gcnew KCallbackFx(this, &GoSensor::OnProgress);
                KCallbackState^ session = gcnew KCallbackState(thunk, handler);
                kStatus status;

                if (!kSuccess(status = ::GoSensor_Upgrade(Handle, strA.CharPtr, (::GoUpgradeFx)session->NativeFunction, session->NativeContext)))
                {
                    delete session;
                    throw gcnew KException(status);
                }
            }

            /// <summary>Creates a backup of sensor files and downloads the backup to the specified location.</summary>
            ///
            /// <param name="destPath">Local file system path for the saved backup file.</param>
            void Backup(String^ destPath)
            {
                KString strA(destPath);

                KCheck(::GoSensor_Backup(Handle, strA.CharPtr));
            }

            /// <summary>
            /// Restores a backup of sensor files.
            /// WARNING! This function writes to flash storage. Review the user manual for implications.
            /// </summary>
            /// <remarks>
            /// This function results in flash storage modifications. If modifications are interrupted due to
            /// power loss, the sensor may reboot into Rescue mode.
            /// </remarks>
            /// <param name="sourcePath">Local file system path of the saved backup file.</param>
            void Restore(String^ sourcePath)
            {
                KString strA(sourcePath);

                KCheck(::GoSensor_Restore(Handle, strA.CharPtr));
            }

            /// <summary>
            /// Restores factory default settings.
            /// WARNING! This function writes to flash storage. Review the user manual for implications.
            /// </summary>
            /// <remarks>
            /// This function results in flash storage modifications. If modifications are interrupted due to
            /// power loss, the sensor may reboot into Rescue mode.
            /// </remarks>
            /// <param name="restoreAddress">true to restore the factory default IP address; false otherwise.</param>
            void RestoreDefaults(bool restoreAddress)
            {
                KCheck(::GoSensor_RestoreDefaults(Handle, restoreAddress));
            }

            /// <summary>Gets a GoSetup instance associated with the sensor.</summary>
            property GoSetup^ Setup
            {
                GoSetup^ get() { return KToObject<GoSetup^>(::GoSensor_Setup(Handle)); }
            }

            /// <summary>Gets the sensor's tools module, used for measurement configuration.</summary>
            property GoSdk::Tools::GoTools^ Tools
            {
                GoSdk::Tools::GoTools^ get() { return KToObject<GoSdk::Tools::GoTools^>(::GoSensor_Tools(Handle)); }
            }

            /// <summary>Gets the output module, used for output configuration.</summary>
            property Outputs::GoOutput^ Output
            {
                Outputs::GoOutput^ get() { return KToObject<Outputs::GoOutput^>(::GoSensor_Output(Handle)); }
            }

            /// <summary>Gets the transform module, used for transformation configuration.</summary>
            property GoTransform^ Transform
            {
                GoTransform^ get() { return KToObject<GoTransform^>(::GoSensor_Transform(Handle)); }
            }

            /// <summary>Gets the device identifier associated with this sensor.</summary>
            property k32u Id
            {
                k32u get() { return (k32u) ::GoSensor_Id(Handle); }
            }

            /// <summary>Gets the model associated with this sensor.
            /// DEPRECATED: this property is deprecated as of 5.3. Use the PartNumber property instead.</summary>
            property String^ Model
            {
                String^ get()
                {
                    kText64 model;

                    KCheck(::GoSensor_Model(Handle, model, kCountOf(model)));

                    return KToString(model);
                }
            }

           /// <summary>Gets the part number associated with this sensor.</summary>
            property String^ PartNumber
            {
                String^ get()
                {
                    kText64 partNumber;

                    KCheck(::GoSensor_PartNumber(Handle, partNumber, kCountOf(partNumber)));

                    return KToString(partNumber);
                }
            }

           /// <summary>Gets the model number associated with this sensor.</summary>
            property String^ ModelNumber
            {
                String^ get()
                {
                    kText64 modelNumber;

                    KCheck(::GoSensor_ModelNumber(Handle, modelNumber, kCountOf(modelNumber)));

                    return KToString(modelNumber);
                }
            }

            /// <summary>Gets the model display name associated with this sensor (not for parsing).</summary>
            property String^ ModelDisplayName
            {
                String^ get()
                {
                    kText64 modelDisplayName;

                    KCheck(::GoSensor_ModelDisplayName(Handle, modelDisplayName, kCountOf(modelDisplayName)));

                    return KToString(modelDisplayName);
                }
            }

            /// <summary>Reports the current state of the sensor.</summary>
            property GoState State
            {
                GoState get() { return (GoState) ::GoSensor_State(Handle); }
            }

            /// <summary>Reports the current states of the sensor.</summary>
            property GoStates States
            {
                GoStates get()
                {
                    ::GoStates states;

                    KCheck(::GoSensor_States(Handle, &states));

                    return GoStates(&states);
                }
            }

            /// <summary>Gets the sensor's current role within the system.</summary>
            property GoRole Role
            {
                GoRole get() { return (GoRole) ::GoSensor_Role(Handle); }
            }

            /// <summary>Reports the user account associated with the current user.</summary>
            property GoUser User
            {
                GoUser get() { return (GoUser) ::GoSensor_User(Handle); }
            }

            /// <summary>Gets the sensor's protocol version.</summary>
            property KVersion ProtocolVersion
            {
                KVersion get() { return (KVersion) ::GoSensor_ProtocolVersion(Handle); }
            }

            /// <summary>Gets the sensor's firmware version.</summary>
            property KVersion FirmwareVersion
            {
                KVersion get() { return (KVersion) ::GoSensor_FirmwareVersion(Handle); }
            }

            /// <summary>Returns the acceleration state of a sensor.
            /// The state can be used by acceleration applications to determine
            /// if the sensor is available for acceleration or is already
            /// accelerated by a host.</summary>
            property GoSensorAccelState AccelerationState
            {
                GoSensorAccelState get()
                {
                    return (GoSensorAccelState) ::GoSensor_AccelState(Handle);
                }
            }

            /// <summary>Returns the port numbers used by an accelerated sensor,
            /// including web port. Can be used by acceleration applications to
            /// get the ports used by an accelerated sensor.</summary>
            property GoPortInfo AccelerationPortInfo
            {
                GoPortInfo get()
                {
                    return (GoPortInfo) ::GoSensor_AccelPortInfo(Handle);
                }
            }

            /// <summary>Returns the operational mode of the sensor.
            /// This can be used by an acceleration application to determine if
            /// the sensor is a virtual sensor, standalone sensor or an
            /// accelerated sensor mode.</summary>
            property GoDiscoveryOpMode AccelerationOpMode
            {
                GoDiscoveryOpMode get()
                {
                    return (GoDiscoveryOpMode) ::GoSensor_AccelOpMode(Handle);
                }
            }

            /// <summary>Returns the physical sensor IP address when sensor is accelerated.</summary>
            property KIpAddress AccelerationIpAddress
            {
                KIpAddress get()
                {
                    kIpAddress ipAddr;

                    KCheck(::GoSensor_AccelSensorIpAddress(Handle, &ipAddr));

                    return (KIpAddress) ipAddr;
                }
            }

            /// <summary>The recording state of the sensor.</summary>
            property bool RecordingEnabled
            {
                bool get() { return KToBool(::GoSensor_RecordingEnabled(Handle)); }
                void set(bool enable) { KCheck(::GoSensor_EnableRecording(Handle, enable)); }
            }

            /// <summary>Gets the input source currently used by the sensor.</summary>
            property GoInputSource InputSource
            {
                GoInputSource get() { return (GoInputSource) ::GoSensor_InputSource(Handle); }
                void set(GoInputSource source) { KCheck(::GoSensor_SetInputSource(Handle, source)); }
            }

            /// <summary>Simulates the current frame in the live recording buffer.</summary>
            ///
            /// <returns>true if the source simulation buffer was valid. false otherwise.</returns>
            bool Simulate()
            {
                kBool isBufferValid;

                KCheck(::GoSensor_Simulate(Handle, &isBufferValid));

                return KToBool(isBufferValid);
            }

            /// <summary>Advances one frame from the current replay position.</summary>
            ///
            /// <param name="direction">Direction with which to step.</param>
            void PlaybackStep(GoSeekDirection direction)
            {
                KCheck(::GoSensor_PlaybackStep(Handle, direction));
            }

            /// <summary>Sets the current frame position for a replay.</summary>
            ///
            /// <param name="position">The frame position to seek.</param>
            void PlaybackSeek(k64s position)
            {
                KCheck(::GoSensor_PlaybackSeek(Handle, (kSize) position));
            }

            /// <summary>Gets the current replay frame position.</summary>
            ///
            /// <param name="position">The current frame position index.</param>
            /// <param name="count">The frame count.</param>
            void PlaybackPosition(k64s% position, k64s% count)
            {
                kSize positionA;
                kSize countA;

                KCheck(::GoSensor_PlaybackPosition(Handle, &positionA, &countA));

                position = (k64s) positionA;
                count = (k64s) countA;
            }

            /// <summary>Clears the replay buffer.</summary>
            void ClearReplayData()
            {
                KCheck(::GoSensor_ClearReplayData(Handle));
            }

            /// <summary>Resets the measurement statistics reported by the health channel</summary>
            void ClearMeasurementStats()
            {
                KCheck(::GoSensor_ClearMeasurementStats(Handle));
            }

            /// <summary>Exports the current frame of a replay in the form of a bitmap.</summary>
            ///
            /// <param name="type">The type of data to export.</param>
            /// <param name="source">The device data source to export from.</param>
            /// <param name="dstFileName">The destination file name of the exported bitmap file.</param>
            void ExportBitmap(GoReplayExportSourceType type, GoDataSource source, String^ dstFileName)
            {
                KString dstFileNameA(dstFileName);

                KCheck(::GoSensor_ExportBitmap(Handle, type, source, dstFileNameA.CharPtr));
            }

            /// <summary>Exports replay data in CSV format.</summary>
            ///
            /// <param name="dstFileName">The destination file name of the exported CSV file.</param>
            void ExportCsv(String^ dstFileName)
            {
                KString dstFileNameA(dstFileName);

                KCheck(::GoSensor_ExportCsv(Handle, dstFileNameA.CharPtr));
            }

            /// <summary>An enumerator value representing the current sensor's family.</summary>
            property GoFamily Family
            {
                GoFamily get() { return (GoFamily) ::GoSensor_Family(Handle); }
            }

            /// <summary>Clears the log file (_live.log).</summary>
            void ClearLog()
            {
                KCheck(::GoSensor_ClearLog(Handle));
            }

            /// <summary>The AutoStart enabled state of the sensor.</summary>
            property bool AutoStart
            {
                bool get() { return KToBool(::GoSensor_AutoStartEnabled(Handle)); }
                void set(bool enable) { KCheck(::GoSensor_EnableAutoStart(Handle, enable)); }
            }

            /// <summary>Set the sensor voltage settings</summary>
            /// <param name="voltage">The supplied voltage 48V or 24V</param>
            /// <param name="cableLength">When in 24V operation, power cable length (meter) must also be provided</param>
            void SetVoltage(GoVoltageSetting voltage, k64f cableLength)
            {
                KCheck(::GoSensor_SetVoltage(Handle, (::GoVoltageSetting)voltage, cableLength));
            }

            /// <summary>The Quick Edit enabled state of the sensor.</summary>
            property bool QuickEdit
            {
                bool get() { return KToBool(::GoSensor_QuickEditEnabled(Handle)); }
                void set(bool enable) { KCheck(::GoSensor_EnableQuickEdit(Handle, enable)); }
            }

            /// <summary>The count of remote sensor information held by the sensor.</summary>
            property k64s RemoteInfoCount
            {
                k64s get() { return (k64s) ::GoSensor_RemoteInfoCount(Handle); }
            }

            /// <summary>Gets the remote sensor information at the given index.</summary>
            ///
            /// <param name="index">The index of the remote sensor information to retrieve.</param>
            ///
            /// <returns>A handle to the selected remote sensor information or null if an invalid index is provided.</returns>
            GoSensorInfo^ GetRemoteInfo(k64s index)
            {
                return KToObject<GoSensorInfo^>(::GoSensor_RemoteInfoAt(Handle, (kSize)index));
            }

            /// <summary>Return the number of files contained in the specified path with an optional extension filter applied.</summary>
            ///
            /// <param name="extensionFilter">An optional extension filter to apply. Send "" to not apply a filter.</param>
            /// <param name="path">The file system path to retrieve the file count for.</param>
            /// <param name="isRecursive">true to include files in sub-folders of the path and false to only count files in the immediate path.</param>
            ///
            /// <returns>The number of files contained in the specified path with an optional extension filter applied.</returns>
            k64s DirectoryFileCount(String^ extensionFilter, String^ path, bool isRecursive)
            {
                KString extensionFilterA(extensionFilter);
                KString pathA(path);

                return (k64s) ::GoSensor_DirectoryFileCount(Handle, extensionFilterA.CharPtr, pathA.CharPtr, isRecursive);
            }

            /// <summary>Return the number of files contained in the specified path with an optional extension filter applied.</summary>
            ///
            /// <param name="extensionFilter">An optional extension filter to apply. Send "" to not apply a filter.</param>
            /// <param name="path">The file system path to retrieve the file count for.</param>
            /// <param name="isRecursive">true to include files in sub-folders of the path and false to only count files in the immediate path.</param>
            ///
            /// <returns>The retrieved file name.</returns>
            String^ GetDirectoryFileName(String^ extensionFilter, String^ path, bool isRecursive, k64s index)
            {
                KString extensionFilterA(extensionFilter);
                KString pathA(path);
                kText256 fileName;

                KCheck(::GoSensor_DirectoryFileNameAt(Handle, extensionFilterA.CharPtr, pathA.CharPtr, isRecursive, (kSize)index, fileName, kCountOf(fileName)));

                return KToString(fileName);
            }

            /// <summary>Delegate for a custom data message handler.</summary>
            /// <remarks>
            /// Sensor data messages can be received synchronously using this function or asynchronously by
            /// registering a callback with the SetDataHandler function.
            ///
            /// NOTE: Data received with this function must be destroyed after use,
            ///       otherwise a memory leak will result.
            ///
            /// </remarks>
            delegate void DataFx(Messages::GoDataSet^ dataSet);

            /// <summary>Sets the data callback delegate to be used upon receipt of data.</summary>
            ///
            /// <param name="handler">The delegate to use when data has been received from the sensor. Send null to revert back to the GoSystem data set storage behavior.</param>
            void SetDataHandler(DataFx^ handler)
            {
                if (handler)
                {
                    KCallbackFx^ thunk = gcnew KCallbackFx(this, &GoSensor::OnDataSet);
                    KCallbackState^ session = gcnew KCallbackState(thunk, handler);
                    kStatus status;

                    if (!kSuccess(status = ::GoSensor_SetDataHandler(Handle, (::GoSensorDataSetFx)session->NativeFunction, session->NativeContext)))
                    {
                        delete session;
                        throw gcnew KException(status);
                    }
                }
                else
                {
                    KCheck(::GoSensor_SetDataHandler(Handle, kNULL, kNULL));
                }
            }

            /// <summary>Creates a part matching model based on the current part data.</summary>
            ///
            /// <param name="name">The intended name of the part match model.</param>
            void PartMatchCreateModel(String^ name)
            {
                KString nameA(name);

                KCheck(::GoSensor_PartMatchCreateModel(Handle, nameA.CharPtr));
            }

            /// <summary>Detect the edges of the specified part model.</summary>
            ///
            /// <param name="name">The name of the part match model to detect edges on.</param>
            /// <param name="sensitivity">The sensitivity to use for model edge detection.</param>
            void PartMatchDetectModelEdges(String^ name, k16u sensitivity)
            {
                KString nameA(name);

                KCheck(::GoSensor_PartMatchDetectModelEdges(Handle, nameA.CharPtr, sensitivity));
            }

            /// <summary>Returns a handle to a part model based on a given name.</summary>
            ///
            /// <param name="name">The name of the part match model to retrieve.</param>
            ///
            /// <returns>A GoPartModel handle.</returns>
            ///
            GoPartModel^ GetPartMatchModel(String^ name)
            {
                KString nameA(name);

                return KToObject<GoPartModel^>(::GoSensor_PartMatchModel(Handle, nameA.CharPtr));
            }

            /// <summary>Control port
            property k32u ControlPort
            {
                void set(k32u port) { KCheck(::GoSensor_SetControlPort(Handle, port)); }
                k32u get() { return ::GoSensor_ControlPort(Handle); }
            }

            /// <summary>Data channel port
            property k32u DataPort
            {
                void set(k32u port) { KCheck(::GoSensor_SetDataPort(Handle, port)); }
                k32u get() { return ::GoSensor_DataPort(Handle); }
            }

            /// <summary>Health channel port
            property k32u HealthPort
            {
                void set(k32u port) { KCheck(::GoSensor_SetHealthPort(Handle, port)); }
                k32u get() { return ::GoSensor_HealthPort(Handle); }
            }

            /// <summary>The number of part match models present in the currently loaded job.</summary>
            property k64s PartMatchModelCount
            {
                k64s get() { return (k64s) ::GoSensor_PartMatchModelCount(Handle); }
            }

            /// <summary>Returns a handle to a part model based on a given index.</summary>
            ///
            /// <param name="index">The index of the part model to retrieve.</param>
            ///
            /// <returns>A GoPartModel handle.</returns>
            ///
            GoPartModel^ GetPartMatchModel(k64s index)
            {
                return KToObject<GoPartModel^>(::GoSensor_PartMatchModelAt(Handle, (kSize)index));
            }

            /// <summary>Detect the edges of the specified part model.</summary>
            ///
            /// <param name="startIndex">The starting index of the runtime variable values to set.</param>
            /// <param name="values">An array which contains the intended runtime variable values.</param>
            void SetRuntimeVariables(k64s startIndex, array<k32s>^ values)
            {
                KCheckArgs(values->Length >= 1);

                pin_ptr<k32s> valuesPtr = &values[0];

                KCheck(::GoSensor_SetRuntimeVariables(Handle, (kSize) startIndex, values->Length, valuesPtr));
            }

            /// <summary>The number of runtime variables available on the device.</summary>
            property k64s RuntimeVariableCount
            {
                k64s get() { return (k64s) ::GoSensor_GetRuntimeVariableCount(Handle); }
            }

            /// <summary>Gets the values associated with a given runtime variable starting index and length.</summary>
            ///
            /// <param name="startIndex">The starting index of the runtime variable values to retrieve.</param>
            /// <param name="length">The number of runtime variables to retrieve.</param>
            ///
            /// <returns>An array which will hold the retrieved runtime variable values.</returns>
            //
            array<k32s>^ GetRuntimeVariables(k64s startIndex, k64s length)
            {
                KCheckArgs(length >= 1);

                array<k32s>^ runtimeVars = gcnew array<k32s>((int)length);
                pin_ptr<k32s> runtimeVarsPtr = &runtimeVars[0];

                KCheck(::GoSensor_GetRuntimeVariables(Handle, (kSize) startIndex, (kSize) length, runtimeVarsPtr));

                return runtimeVars;
            }

            /// <summary>Gets the values associated with a given runtime variable starting index and length.</summary>
            ///
            /// <param name="index">The index of the runtime variable value to retrieve.</param>
            ///
            /// <returns>The runtime variable value.</returns>
            //
            k32s GetRuntimeVariable(k64s index)
            {
                k32s runtimeVar;

                KCheck(::GoSensor_GetRuntimeVariableAt(Handle, (kSize)index, &runtimeVar));

                return runtimeVar;
            }

            /// <summary>Starts recording to a stream.</summary>
            ///
            /// <remarks>
            /// Before calling this function, the sensor should already be running and
            /// recording.GoSensor_StopRecordingStream() should be called before executing
            /// any other sensor commands, with the exception of GoSensor_IsRecordingStreaming().
            /// </remarks>
            ///
            /// <param name="destFile">A local file path to store the .rec data.</param>
            void StartRecordingStream(String^ destFile)
            {
                KString destFileA(destFile);

                KCheck(::GoSensor_StartRecordingStream(Handle, destFileA.CharPtr));
            }

            /// <summary>Starts recording to a stream.</summary>
            void StopRecordingStream()
            {
                KCheck(::GoSensor_StopRecordingStream(Handle));
            }

            /// <summary>Reports whether or not recording is streaming.</summary>
            ///
            /// <returns>true if recording is streaming; false otherwise.</returns>
            bool IsRecordingStreaming()
            {
                return KToBool(::GoSensor_IsRecordingStreaming(Handle));
            }

            /// <summary>Returns the GeoCal object for this sensor</summary>
            GoGeoCal^ GeoCal()
            {
                return gcnew GoGeoCal(this);
            }

            /// <summary>Sets a sensor flag.</summary>
            ///
            /// <param name="name">The name of the flag.</param>
            /// <param name="value">The value of the flag.</param>
            void SetFlag(String^ name, String^ value)
            {
                KString nameStr(name);
                KString valueStr(value);

                KCheck(::GoSensor_SetFlag(Handle, nameStr.CharPtr, valueStr.CharPtr));
            }

            /// <summary>Gets a sensor flag.</summary>
            ///
            /// <param name="name">The name of the flag.</param>
            ///
            /// <returns>The value of the flag.</returns>
            String^ GetFlag(String^ name)
            {
                KString nameStr(name);
                KString valueStr;

                KCheck(::GoSensor_GetFlag(Handle, nameStr.CharPtr, (kString)valueStr.ToHandle()));

                return KToString(valueStr.CharPtr);
            }

            /// <summary>Synchronously assigns a buddy sensor.</summary>
            /// <param name=buddy>Sensor to be assigned as buddy.</param>
            void AddBuddyBlocking(GoSensor^ buddy)
            {
                KCheck(::GoSensor_AddBuddyBlocking(Handle, buddy->Handle));
            }

            /// <summary>Get the sensor voltage settings (only on G3210)</summary>
            /// <param name=voltage>Destination to store voltage (can be kNULL)</param>
            /// <param name=cableLength>Destination to store cable length (meter) (can be kNULL)</param>
            void GetVoltage([Out] GoVoltageSetting% voltage, [Out] K64f% cableLength)
            {
                k64f length;
                ::GoVoltageSetting volts;
                KCheck(::GoSensor_GetVoltage(Handle, &volts, &length));
                voltage = (GoVoltageSetting)volts;
                cableLength = length;
            }

            /// <summary>Gets the replay module, used for replay configuration.</summary>
            property GoReplay^ Replay
            {
                GoReplay^ get()
                {
                    ::GoReplay replay = ::GoSensor_Replay(Handle);

                    return (kIsNull(replay)) ? nullptr : KToObject<GoReplay^>(replay);
                }
            }

            /// <summary>Gets/Sets the Upgrade Port.</summary>
            property k32u UpgradePort
            {
                k32u get()              { return (k32u)::GoSensor_UpgradePort(Handle); }
                void set(k32u value)    { KCheck(::GoSensor_SetUpgradePort(Handle, value)); }
            }

            /// <summary>Gets the available storage space remaining for user files.</summary>
            property k64u UserStorageFree
            {
                k64u get()  { return (k64u)::GoSensor_UserStorageFree(Handle); }
            }

            /// <summary>Gets the storage space used for user files.</summary>
            property k64u UserStorageUsed
            {
                k64u get()  { return (k64u)::GoSensor_UserStorageUsed(Handle); }
            }

            /// <summary>Waits for all buddies to be connected within a specific timeout.</summary>
            /// <param name=timeout>a timeout in milliseconds to wait for all required buddies to get connected </param>
            void WaitForBuddies(k64u timeout)
            {
                KCheck(::GoSensor_WaitForBuddies(Handle, timeout));
            }

        private:

            kStatus OnProgress(kPointer receiver, kPointer sender, kPointer args)
            {
                KCallbackState^ context = KCallbackState::FromNativeContext(receiver);
                UpgradeFx^ handler = (UpgradeFx^)context->Handler;
                ::GoUpgradeFxArgs* upgradeArgs = reinterpret_cast<::GoUpgradeFxArgs*>(args);
                kStatus status = kOK;

                try
                {
                    if (handler)
                    {
                        handler(upgradeArgs->progress);
                    }
                }
                catch (KException^ e)
                {
                    status = e->Status;
                }
                catch (...)
                {
                    status = kERROR;
                }

                return status;
            }

            kStatus OnDataSet(kPointer receiver, kPointer sender, kPointer args)
            {
                KCallbackState^ context = KCallbackState::FromNativeContext(receiver);
                DataFx^ handler = (DataFx^)context->Handler;
                kStatus status = kOK;

                try
                {
                    if (handler)
                    {
                        handler(reinterpret_cast<Messages::GoDataSet^>(KToObject<KObject^>(args)));
                    }
                }
                catch (KException^ e)
                {
                    status = e->Status;
                }
                catch (...)
                {
                    status = kERROR;
                }

                return status;
            }
        };
    }
}

#endif

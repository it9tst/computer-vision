//
// GoSystem.h
//
// Copyright (C) 2016-2021 by LMI Technologies Inc.
// Licensed under the MIT License.
// Redistributed files must retain the above copyright notice.
//
#ifndef GO_SDK_NET_SYSTEM_H
#define GO_SDK_NET_SYSTEM_H

#include <GoSdk/GoSystem.h>
#include <GoSdkNet/Messages/GoDataSet.h>
#include <GoSdkNet/Messages/GoDiscoveryExtInfo.h>
#include <GoSdkNet/GoSdkDef.h>

namespace Lmi3d
{
    namespace GoSdk
    {
        /// <summary>Represents a system of Gocator devices.</summary>
        public ref class GoSystem : public KObject
        {
            KDeclareClass(GoSystem, GoSystem)

            /// <summary>Initializes a new instance of the GoSystem class with the specified Zen object handle.</summary>
            /// <param name="handle">Zen object handle.</param>
            GoSystem(IntPtr handle)
                : KObject(handle)
            {}

            /// <summary>Initializes a new instance of the GoSystem class with default discovery providers.</summary>
            ///
            /// <remarks>
            /// During construction, a GoSystem object uses Sensor Discovery Protocol to locate sensors.
            /// The list of detected sensors can be accessed using the SensorCount and GetSensor
            /// functions.
            /// </remarks>
            ///
            GoSystem()
            {
                ::GoSystem handle = kNULL;

                KCheck(::GoSystem_Construct(&handle, kNULL));

                Handle = handle;
            }

            /// <summary>Initializes a new instance of the GoSystem class without enabling automatic gocator discovery.</summary>
            ///
            /// <remarks>
            /// During construction, a GoSystem object uses Gocator Discovery Protocol to locate sensors.
            /// The list of detected sensors can be accessed using the SensorCount and GetSensor
            /// functions.
            /// </remarks>
            ///
            /// <param name="enableAutoDiscovery">Enable or disable automatic discovery of gocators using the Gocator Discovery Protocol</param>
            GoSystem(bool enableAutoDiscovery)
            {
                ::GoSystem handle = kNULL;

                if (enableAutoDiscovery)
                {
                    KCheck(::GoSystem_Construct(&handle, kNULL));
                }
                else
                {
                    KCheck(::GoSystem_ConstructEx(&handle, kNULL));
                }

                Handle = handle;
            }

            /// <inheritdoc cref="GoSystem()" />
            ///
            /// <remarks>
            /// During construction, a GoSystem object uses Sensor Discovery Protocol to locate sensors.
            /// The list of detected sensors can be accessed using the SensorCount and GetSensor
            /// functions.
            /// </remarks>
            ///
            /// <param name="allocator">Memory allocator</param>
            GoSystem(KAlloc^ allocator)
            {
                ::GoSystem handle = kNULL;

                KCheck(::GoSystem_Construct(&handle, KToHandle(allocator)));

                Handle = handle;
            }

            /// <summary>Initializes a new instance of the GoSystem class without enabling automatic gocator discovery.</summary>
            ///
            /// <remarks>
            /// During construction, a GoSystem object uses Gocator Discovery Protocol to locate sensors.
            /// The list of detected sensors can be accessed using the SensorCount and GetSensor
            /// functions.
            /// </remarks>
            ///
            /// <param name="enableAutoDiscovery">Enable or disable automatic discovery of gocators using the Gocator Discovery Protocol</param>
            /// <param name="allocator">Memory allocator</param>
            GoSystem(bool enableAutoDiscovery, KAlloc^ allocator)
            {
                ::GoSystem handle = kNULL;

                if (enableAutoDiscovery)
                {
                    KCheck(::GoSystem_Construct(&handle, KToHandle(allocator)));
                }
                else
                {
                    KCheck(::GoSystem_ConstructEx(&handle, KToHandle(allocator)));
                }

                Handle = handle;
            }

            /// <summary>Establishes control connections to all sensors.</summary>
            ///
            /// <remarks>
            /// A control connection is required before calling any sensor function except GoSensor_State,
            /// GoSensor_Address, or GoSensor_SetAddress.
            /// </remarks>
            ///
            void Connect()
            {
                KCheck(::GoSystem_Connect(Handle));
            }

            /// <summary>Terminates control connections to all sensors.</summary>
            void Disconnect()
            {
                KCheck(::GoSystem_Disconnect(Handle));
            }

            /// <summary>Reports whether the system has changes that require a refresh.</summary>
            ///
            /// <remarks>
            /// Sensors can undergo autonomous state changes that require client state to be refreshed
            /// (e.g. sensor goes offline). The HasChanges function can be used to determine
            /// if such changes have occurred.
            ///
            /// The HasChanges function does not perform communication, and consequently, will not
            /// require the caller to block for a long duration. If changes are detected, the Refresh
            /// function can be called to resolve the changes.
            ///
            /// This method is thread-safe.
            /// </remarks>
            ///
            /// <returns>true if the system has changes.</returns>
            bool HasChanges()
            {
                return KToBool(::GoSystem_HasChanges(Handle));
            }

            /// <summary>Updates client state to reflect any changes that have occurred in the sensor network.</summary>
            ///
            /// <remarks>
            /// Sensors can undergo autonomous state changes that require client state to be refreshed
            /// (e.g. sensor goes offline). The Refresh function resynchronizes client state
            /// with the current state of the sensor network.
            ///
            /// Calling this function may destroy or modify local sensor objects. The HasChanges
            /// function can be used prior to calling Refresh, to determine whether a refresh is
            /// needed.
            /// </remarks>
            ///
            void Refresh()
            {
                KCheck(::GoSystem_Refresh(Handle));
            }

            /// <summary>Reports the Sensor Protocol version implemented by this library.</summary>
            ///
            /// <remarks>
            /// A Sensor Protocol version number has a major component and a minor component.
            /// If the major version implemented by this library is the same as the major
            /// version implemented by a sensor device, then communication can proceed.  Otherwise,
            /// the sensor should be upgraded or a newer version of this library should be obtained.
            /// Sensors with incompatible major versions will be reported as being in the
            /// incompatible state upon connection.
            ///
            /// If major versions match, but the minor version implemented by this library
            /// is lower than the minor version implemented by the sensor, then some sensor features
            /// will not be accessible through this library.
            ///
            /// If major versions match, but the minor version implemented by this library
            /// is higher than the minor version implemented by the sensor, then some features
            /// exposed by this library may be unimplemented by the sensor.
            ///
            /// This method is thread-safe.
            /// </remarks>
            ///
            /// <returns>Protocol version implemented by this library.</returns>
            KVersion ProtocolVersion()
            {
                return KVersion(::GoSystem_ProtocolVersion());
            }

            /// <summary>Reports the sensor firmware version that was built alongside this library.</summary>
            ///
            /// <remarks>
            /// A sensor SDK version number has a major component and a minor component.
            /// If the major version implemented by this library is the same as the major
            /// version implemented by a sensor device, then communication can proceed.  Otherwise,
            /// the sensor should be upgraded or a version of this library matching the sensor
            /// should be obtained.
            ///
            /// When it comes to the matter of mismatched SDK and firmware versions, so long
            /// as the protocol versions match up, they should be compatible.
            /// </remarks>
            ///
            /// <returns>Firmware version that was built alongside this library.</returns>
            KVersion SdkVersion()
            {
                return KVersion(::GoSystem_SdkVersion());
            }

            /// <summary>Delegate for a GoSystem data/health handler.</summary>
            /// <remarks>
            /// NOTE: Data received with this delegate must be destroyed after use,
            ///       otherwise a memory leak will result.
            /// </remarks>
            ///
            delegate void DataFx(KObject^ data);

            /// <summary>Sets a callback function that can be used to receive sensor data messages asynchronously.</summary>
            ///
            /// <remarks>
            /// Sensor data messages can be received synchronously using the ReceiveData function
            /// or asynchronously by registering a callback function.  If a callback function is registered,
            /// a background thread will be created to perform notifications. After using SetDataHandler function, 
            /// SDKNet application is responsible for disposing of the GoDataSet objects that hold the data received from
            /// the sensor on the data connection.
            ///
            /// To unregister a previously-registered data handler, call this function using null in place
            /// of the callback function argument.
            /// </remarks>
            ///
            /// <param name="handler">Data callback function (or null to unregister).</param>
            void SetDataHandler(DataFx^ handler)
            {
                if (handler)
                {
                    KCallbackFx^ thunk = gcnew KCallbackFx(this, &GoSystem::OnData);
                    KCallbackState^ session = gcnew KCallbackState(thunk, handler);
                    kStatus status;

                    if (!kSuccess(status = ::GoSystem_SetDataHandler(Handle, (::GoDataFx)session->NativeFunction, session->NativeContext)))
                    {
                        delete session;
                        throw gcnew KException(status);
                    }
                }
                else
                {
                    KCheck(::GoSystem_SetDataHandler(Handle, kNULL, kNULL));
                }
            }

            /// <summary>Sets the maximum amount of memory that can be used to buffer received data messages.</summary>
            ///
            /// <remarks>
            /// Received data messages are enqueued until they are accepted by the caller. This function
            /// determines the maximum size, in bytes, of enqueued messages. The default maximum size is 50 MB.
            /// </remarks>
            ///
            property k64s DataCapacity
            {
                k64s get()                  { return (k64s) ::GoSystem_DataCapacity(Handle); }
                void set(k64s capacity)     { KCheck(::GoSystem_SetDataCapacity(Handle, (kSize)capacity)); }
            }

            /// <summary>Establishes data connections to all connected sensors currently in the ready or running states.</summary>
            ///
            /// <remarks>
            /// Data connections are not automatically established when sensor control connection are established.
            /// Use this function (or GoSensor_EnableData) to enable/disable data connections.  After using EnableData function, 
            /// SDKNet application is responsible for disposing of the GoDataSet objects that hold the data received from the sensor on the data connection.
            /// </remarks>
            ///
            /// <param name="enable">true to enable data connections; false to disable.</param>
            void EnableData(bool enable)
            {
                KCheck(::GoSystem_EnableData(Handle, enable));
            }

            /// <summary>Clears any buffered data messages.</summary>
            ///
            /// <remarks>
            /// When stopping and then restarting a system, it may be desirable to ensure that no messages from
            /// the previous session remain in any buffers. The ClearData function closes any open data
            /// channels, destroys any received messages, and then reopens data channels.
            /// </remarks>
            ///
            void ClearData()
            {
                KCheck(::GoSystem_ClearData(Handle));
            }

            /// <summary>Receives a set of sensor data messages.</summary>
            ///
            /// <remarks>
            /// Sensor data messages can be received synchronously using this function or asynchronously by
            /// registering a callback with the SetDataHandler function.
            ///
            /// NOTE: Data received with this function must be destroyed after use,
            ///       otherwise a memory leak will result.
            /// </remarks>
            ///
            /// <param name="timeout">Duration to wait for messages, in microseconds.</param>
            /// <returns>Data Set.</returns>
            Messages::GoDataSet^ ReceiveData(k64s timeout)
            {
                ::GoDataSet data = kNULL;

                KCheck(::GoSystem_ReceiveData(Handle, &data, timeout));

                return KToObject<Messages::GoDataSet^>(data);
            }

            /// <summary>Sets a callback function that can be used to receive sensor health messages asynchronously.</summary>
            ///
            /// <remarks>
            /// Sensor health messages can be received synchronously using the ReceiveHealth function
            /// or asynchronously by registering a callback function. If a callback function is registered,
            /// a background thread will be created to perform notifications. After using SetDataHandler function, 
            /// SDKNet application is responsible for disposing of the GoDataSet objects that hold the data received from
            /// the sensor on the data connection.
            ///
            /// To unregister a previously-registered health handler, call this function using null in place
            /// of the callback function argument.
            /// </remarks>
            ///
            /// <param name="handler">Health callback function (or null to unregister).</param>
            void SetHealthHandler(DataFx^ handler)
            {
                if (handler)
                {
                    KCallbackFx^ thunk = gcnew KCallbackFx(this, &GoSystem::OnData);
                    KCallbackState^ session = gcnew KCallbackState(thunk, handler);
                    kStatus status;

                    if (!kSuccess(status = ::GoSystem_SetHealthHandler(Handle, (::GoDataFx)session->NativeFunction, session->NativeContext)))
                    {
                        delete session;
                        throw gcnew KException(status);
                    }
                }
                else
                {
                    KCheck(::GoSystem_SetDataHandler(Handle, kNULL, kNULL));
                }
            }

            /// <summary>Receives a set of sensor health messages.</summary>
            ///
            /// <remarks>
            /// Sensor health messages can be received synchronously using this function or asynchronously by
            /// registering a callback with the SetHealthHandler function.
            /// </remarks>
            ///
            /// <param name="timeout">Duration to wait for messages, in microseconds.</param>
            /// <returns>Data Set.</returns>
            Messages::GoDataSet^ ReceiveHealth(k64s timeout)
            {
                ::GoDataSet data = kNULL;

                KCheck(::GoSystem_ReceiveHealth(Handle, &data, timeout));

                return KToObject<Messages::GoDataSet^>(data);
            }

            /// <summary>Clears any buffered health messages.</summary>
            void ClearHealth()
            {
                KCheck(::GoSystem_ClearHealth(Handle));
            }

            /// <summary>Starts all sensors that are currently in the ready state.</summary>
            void Start()
            {
                KCheck(::GoSystem_Start(Handle));
            }

            /// <summary>Starts all sensors that are currently in the ready state at a scheduled value.</summary>
            ///
            /// <param name="value">The value at which to start the sensor. It is in uS when time triggered and ticks when encoder triggered.</param>
            void ScheduledStart(k64s value)
            {
                KCheck(::GoSystem_ScheduledStart(Handle, value));
            }

            /// <summary>
            /// Performs alignment for sensors in the ready state.
            /// WARNING! This function may write to flash storage. Review the user manual for implications.
            /// </summary>
            /// <remarks>
            /// NOTE: This operation will result in sensor starts for the duration of the
            /// alignment. It can be canceled via GoSystem_Stop. This function's operation
            /// status does not correspond to the actual alignment result. In order to
            /// retrieve the alignment result, you must enable the data channel before calling
            /// this function, receive an alignment data message and then check its status.
            /// </remarks>
            ///
            void StartAlignment()
            {
                KCheck(::GoSystem_StartAlignment(Handle));
            }

            /// <summary>Performs exposure auto set for sensors in the ready state.</summary>
            ///
            /// <remarks>
            /// NOTE: This operation will result in sensor starts for the duration of the
            /// exposure AutoSet. A successful operation status does NOT modify the configuration.
            /// You must retrieve the resulting exposure values and set it for the appropriate
            /// exposure setting. This involves enabling the data connection prior to running
            /// exposure auto set and then receiving an exposure auto set message, which
            /// you can then use to query the status and access the resulting value.
            /// </remarks>
            ///
            void StartExposureAutoSet()
            {
                KCheck(::GoSystem_StartExposureAutoSet(Handle));
            }

            /// <summary>Stops all connected sensors.</summary>
            void Stop()
            {
                KCheck(::GoSystem_Stop(Handle));
            }

            /// <summary>Reboots all connected sensors.</summary>
            ///
            /// <param name="wait">true to wait for reboot complete; false to return immediately.</param>
            void Reset(bool wait)
            {
                KCheck(::GoSystem_Reset(Handle, wait));
            }

            /// <summary>Aborts ongoing sensor communication.</summary>
            ///
            /// <remarks>
            /// This method asynchronously aborts ongoing communication; the next time that any
            /// I/O operation blocks for an extended period of time, it will be terminated. This method
            /// is thread-safe.
            ///
            /// In order to resume communication, call Refresh or Connect.
            /// </remarks>
            ///
            void Cancel()
            {
                KCheck(::GoSystem_Cancel(Handle));
            }

            /// <summary>Gets the Discovery channel information for the given device ID (if the device is present and the command is supported)</summary>
            ///
            /// <param name="deviceId">The ID of the device with which to retrieve Discovery channel information.</param>
            ///
            /// <returns>The sensor's constructed discovery information.</returns>
            ///
            Messages::GoDiscoveryExtInfo^ GetExtendedDiscoveryInfo(k32u deviceId)
            {
                ::GoDiscoveryExtInfo info;

                KCheck(::GoSystem_GetExtendedDiscoveryInfo(Handle, deviceId, &info, kNULL));

                return KToObject<Messages::GoDiscoveryExtInfo^>(info);
            }

            /// <summary>Gets the number of nodes in the system.</summary>
            property k64s SensorCount
            {
                k64s get() { return (k64s) ::GoSystem_SensorCount(Handle); }
            }

            /// <summary>Gets the sensor object at the specified index.</summary>
            ///
            /// <param name="index">Sensor index.</param>
            /// <returns>Sensor instance.</returns>
            GoSensor^ GetSensor(k64s index)
            {
                return KToObject<GoSensor^>(::GoSystem_SensorAt(Handle, (kSize)index));
            }

            /// <summary>Gets the sensor object with the specified device id (serial number).</summary>
            ///
            /// <remarks>
            /// Throws KStatus.ErrorNotFound if not found.
            /// </remarks>
            ///
            /// <param name="id">Device identifier.</param>
            /// <returns>Sensor instance.</returns>
            GoSensor^ FindSensorById(k32u id)
            {
                ::GoSensor sensor;

                KCheck(::GoSystem_FindSensorById(Handle, id, &sensor));

                return KToObject<GoSensor^>(sensor);
            }

            /// <summary>Gets the sensor object with the specified IP address.</summary>
            ///
            /// <remarks>
            /// Throws KStatus.ErrorNotFound if not found.
            /// </remarks>
            ///
            /// <param name="address">Sensor IP address.</param>
            /// <returns>Sensor instance.</returns>
            GoSensor^ FindSensorByIpAddress(KIpAddress address)
            {
                ::GoSensor sensor;

                KCheck(::GoSystem_FindSensorByIpAddress(Handle, (kIpAddress*) &address, &sensor));

                return KToObject<GoSensor^>(sensor);
            }

            /// <summary>Gets the current time stamp from the sensor network.</summary>
            ///
            /// <returns>Timestamp value.</returns>
            k64s Timestamp()
            {
                k64u time;

                KCheck(::GoSystem_Timestamp(Handle, &time));

                return time;
            }

            /// <summary>Gets the current encoder value from the sensor network.</summary>
            ///
            /// <returns>Encoder value.</returns>
            k64s Encoder()
            {
                k64s encoder;

                KCheck(::GoSystem_Encoder(Handle, &encoder));

                return encoder;
            }

            /// <summary>The current number of defined multiplex banks.</summary>
            property k64s MultiplexBankCount
            {
                k64s get() { return (k64s)::GoSystem_MultiplexBankCount(Handle); }
            }

            /// <summary>Gets a multiplex bank corresponding to the given index.</summary>
            ///
            /// <param name="index">The index of the multiplex bank to retrieve.</param>
            ///
            /// <returns>The multiplex bank at the given index or null if an invalid index is given.</returns>
            GoMultiplexBank^ GetMultiplexBank(k64s index)
            {
                return KToObject<GoMultiplexBank^>(::GoSystem_MultiplexBankAt(Handle, (kSize)index));
            }

            /// <summary>Gets a multiplex bank corresponding to the given index.</summary>
            ///
            /// <returns>A multiplex bank.</returns>
            GoMultiplexBank^ AddMultiplexBank()
            {
                ::GoMultiplexBank bank;
                KCheck(::GoSystem_AddMultiplexBank(Handle, &bank));

                return KToObject<GoMultiplexBank^>(bank);
            }

            /// <summary>Removes a multiplex bank at the given index.</summary>
            ///
            /// <param name="index">The index of the multiplex bank to remove.</param>
            void RemoveMultiplexBank(k64s index)
            {
                KCheck(::GoSystem_RemoveMultiplexBank(Handle, (kSize)index));
            }

            /// <summary>Removes all multiplex banks.</summary>
            void ClearMultiplexBanks()
            {
                KCheck(::GoSystem_ClearMultiplexBanks(Handle));
            }

            /// <summary>Gets the maximum sensor exposure duration in the given multiplex bank.</summary>
            /// <param name="bank">The multiplex bank to check.</param>
            /// <returns>The maximum sensor exposure duration contained in the multiplexing bank.</returns>
            k64f MaxBankExposureDuration(GoMultiplexBank^ bank)
            {
                return ::GoSystem_MaxBankExposureDuration(Handle, (::GoMultiplexBank)bank);
            }

            /// <summary>Automatically update the single multiplexing delay and period configuration for all sensors contained in all defined multiplex banks.</summary>
            ///
            /// <param name="period">The exposure period to set among all multiplexed sensors. A value of 0.0 will result in automatic period determination.</param>
            void UpdateAllMultiplexParameters(k64f period)
            {
                KCheck(::GoSystem_UpdateAllMultiplexParameters(Handle, period));
            }

            /// <summary>Automatically update the multiplexing single delay configuration for all sensors contained in all defined multiplex banks.</summary>
            void UpdateMultiplexDelay()
            {
                KCheck(::GoSystem_UpdateMultiplexDelay(Handle));
            }

            /// <summary>The maximum value of all multiplexed sensor's minimum multiplexing periods.</summary>
            property k64f MaxMinimumMultiplexPeriod
            {
                k64f get()                  { return ::GoSystem_MaxMinimumMultiplexPeriod(Handle); }
            }

            /// <summary>Automatically update the multiplexing single period configuration for all sensors contained in all defined multiplex banks.</summary>
            ///
            /// <param name="period">The exposure period to set among all multiplexed sensors. A value of 0.0 will result in automatic period determination.</param>
            void UpdateMultiplexPeriod(k64f period)
            {
                KCheck(::GoSystem_UpdateMultiplexPeriod(Handle, period));
            }

            /// <summary>Set one interface to either allow or not allow the Gocator Discovery Protocol to use.</summary>
            ///
            /// <param name="address">IP address of the interface over which the Gocator Discovery Protocol is allowed or not allowed to use.</param>
            /// <param name="enable">Set to true: allow protocol to use this interface. Set to false: to not allow the protocol to use this interface.</param>
            void SetOneDiscoveryInterface(KIpAddress address, bool enable)
            {
                KCheck(::GoSystem_SetOneDiscoveryInterface(Handle, (kIpAddress*) &address, enable));
            }

            /// <summary>Set all interfaces to either allow or not allow the Gocator Discovery Protocol to use.</summary>
            ///
            /// <param name="enable">Set to true: allow protocol to use the interface(s). Set to false: to not allow the protocol to use the interface(s).</param>
            void SetAllDiscoveryInterface(bool enable)
            {
                KCheck(::GoSystem_SetAllDiscoveryInterface(Handle, enable));
            }

            /// <summary>Enable or disable compatibility mode for the discovery service.</summary>
            ///
            /// <param name="enable">Set to true: allow compatibility mode. Set to false: to not allow compatibility mode.</param>
            void EnableDiscoveryCompatibility(bool enable)
            {
                KCheck(::GoSystem_EnableDiscoveryCompatibility(Handle, enable));
            }

            /// <summary>Returns whether or not compatibility mode is enabled.</summary>
            ///
            /// <returns>true if compatibility mode is enabled.</returns>
            bool DiscoveryCompatibilityEnabled()
            {
                return KToBool(::GoSystem_DiscoveryCompatibilityEnabled(Handle));
            }

            /// <summary>Start running the Gocator Discovery Protocol to find/discover sensors.</summary>
            void StartDiscovery()
            {
                KCheck(::GoSystem_StartDiscovery(Handle));
            }

            /// <summary>Adds a sensor by passing a custom address and port info, this doesnt require discovery.</summary>
            ///
            /// <param name="addressInfo">GoAddressInfo object containing IP to connnect on.</param>
            /// <param name="portInfo">GoPortInfo object containing custom port values.</param>
            GoSensor^ AddSensor(GoAddressInfo addressInfo, GoPortInfo portInfo)
            {
                ::GoSensor sensor = kNULL;
                ::GoAddressInfo localAddrInfo = (::GoAddressInfo)addressInfo;
                ::GoPortInfo localPortInfo = (::GoPortInfo)portInfo;
                KCheck(::GoSystem_AddSensor(Handle, &localAddrInfo, &localPortInfo, &sensor));

                return KToObject<GoSensor^>(sensor);
            }

            /// <summary>Lock the system state to ensure thread safety while
            /// reading/modifying the GoSystem class's list of sensors.
            /// Call this to lock the state before retrieving and using
            /// sensor object handles to ensure the sensor object is not deleted
            /// because another thread refreshed the system. Used in
            /// multi-threaded SDK applications.</summary>
            void LockState()
            {
                KCheck(::GoSystem_LockState(Handle));
            }

            /// <summary>Unlock the system state to allow other threads to
            /// read/modify the GoSystem class's list of sensors.
            /// Used in multi-threaded SDK applications.</summary>
            void UnlockState()
            {
                KCheck(::GoSystem_UnlockState(Handle));
            }

        private:

            kStatus OnData(kPointer receiver, kPointer sender, kPointer args)
            {
                KCallbackState^ context = KCallbackState::FromNativeContext(receiver);
                DataFx^ handler = (DataFx^)context->Handler;
                kStatus status = kOK;

                try
                {
                    if (handler)
                    {
                        GoDataSet dataSet = (GoDataSet)args;
                        KObject^ obj = KToObject<KObject^>(args);
                        handler(obj);
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

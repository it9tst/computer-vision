//
// GoAcceleratorMgr.h
//
// Copyright (C) 2016-2021 by LMI Technologies Inc.
// Licensed under the MIT License.
// Redistributed files must retain the above copyright notice.
//
#ifndef GO_SDK_NET_ACCELERATOR_MGR_H
#define GO_SDK_NET_ACCELERATOR_MGR_H

#include <GoSdk/GoAcceleratorMgr.h>
#include <GoSdkNet/GoSdkDef.h>

namespace Lmi3d
{
    namespace GoSdk
    {
        namespace AcceleratorMgr
        {
            ///<summary>Represents an GoAcceleratorMgr events passed into the acceleration
            ///update callback handler bound in by SDK client.</summary>
            public value struct GoAcceleratorMgrAccelEvents
            {
                KDeclareEnum(GoAcceleratorMgrAccelEvents, ::GoAcceleratorMgrAccelEvents)

                /// <summary>Sensor acceleration is in progress.</summary>
                literal k32s EVENT_ACCELERATING = GO_ACCELERATOR_MGR_EVENT_ACCELERATING;

                /// <summary>Sensor is accelerated successfully.</summary>
                literal k32s EVENT_ACCELERATED = GO_ACCELERATOR_MGR_EVENT_ACCELERATED;

                /// <summary>Sensor deceleration is in progress.</summary>
                literal k32s EVENT_DECELERATING = GO_ACCELERATOR_MGR_EVENT_DECELERATING;

                /// <summary>Sensor is no longer accelerated.</summary>
                literal k32s EVENT_DECELERATED = GO_ACCELERATOR_MGR_EVENT_DECELERATED;

                /// <summary>Sensor acceleration stopped or failed to start.</summary>
                literal k32s EVENT_STOPPED = GO_ACCELERATOR_MGR_EVENT_STOPPED;

                /// <summary>Accelerated sensor is disconnected from network.</summary>
                literal k32s EVENT_DISCONNECTED = GO_ACCELERATOR_MGR_EVENT_DISCONNECTED;

                /// <summary>Accelerated process stopped unexpectedly.</summary>
                literal k32s EVENT_PROCESS_STOPPED = GO_ACCELERATOR_MGR_EVENT_PROCESS_STOPPED;

            };

            [StructLayout(LayoutKind::Explicit, Size = sizeof(::GoAcceleratorMgrAccelUpdate))]
            public value struct GoAcceleratorMgrAccelUpdate
            {
                KDeclareStruct(GoAcceleratorMgrAccelUpdate, GoAcceleratorMgrAccelUpdate)

                [FieldOffset(offsetof(::GoAcceleratorMgrAccelUpdate, sensorId))]
                k32u SensorId;

                [FieldOffset(offsetof(::GoAcceleratorMgrAccelUpdate, accelEvent))]
                GoAcceleratorMgrAccelEvents AccelEvent;
            };

            [StructLayout(LayoutKind::Explicit, Size = sizeof(::GoAccelSensorPortAllocPorts))]
            public value struct GoAccelSensorPortAllocPorts
            {
                KDeclareStruct(GoAccelSensorPortAllocPorts, GoAccelSensorPortAllocPorts)

                [FieldOffset(offsetof(::GoAccelSensorPortAllocPorts, controlPort))]
                k16u ControlPort;

                [FieldOffset(offsetof(::GoAccelSensorPortAllocPorts, upgradePort))]
                k16u UpgradePort;

                [FieldOffset(offsetof(::GoAccelSensorPortAllocPorts, healthPort))]
                k16u HealthPort;

                [FieldOffset(offsetof(::GoAccelSensorPortAllocPorts, privateDataPort))]
                k16u PrivateDataPort;

                [FieldOffset(offsetof(::GoAccelSensorPortAllocPorts, publicDataPort))]
                k16u PublicDataPort;

                [FieldOffset(offsetof(::GoAccelSensorPortAllocPorts, webPort))]
                k16u WebPort;
            };

            [StructLayout(LayoutKind::Explicit, Size = sizeof(::GoAcceleratorMgrSensorParam))]
            public value struct GoAcceleratorMgrSensorParam
            {
                KDeclareStruct(GoAcceleratorMgrSensorParam, GoAcceleratorMgrSensorParam)

                [FieldOffset(offsetof(::GoAcceleratorMgrSensorParam, ports))]
                GoAccelSensorPortAllocPorts Ports;

                [FieldOffset(offsetof(::GoAcceleratorMgrSensorParam, platformIpAddress))]
                KIpAddress PlatformIpAddress;
            };

            [StructLayout(LayoutKind::Explicit, Size = sizeof(::GoAcceleratorMgrSensorBackup))]
            public value struct GoAcceleratorMgrSensorBackup
            {
                KDeclareStruct(GoAcceleratorMgrSensorBackup, GoAcceleratorMgrSensorBackup)

                [FieldOffset(offsetof(::GoAcceleratorMgrSensorBackup, ports))]
                GoAccelSensorPortAllocPorts Ports;

                [FieldOffset(offsetof(::GoAcceleratorMgrSensorBackup, sensorAddressInfo))]
                GoAddressInfo SensorAddressInfo;
            };


            [StructLayout(LayoutKind::Explicit, Size = sizeof(::GoAcceleratorMgrSensorInfo))]
            public value struct GoAcceleratorMgrSensorInfo
            {
                KDeclareStruct(GoAcceleratorMgrSensorInfo, GoAcceleratorMgrSensorInfo)

                [FieldOffset(offsetof(::GoAcceleratorMgrSensorInfo, sensorId))]
                k32u SensorId;

                [FieldOffset(offsetof(::GoAcceleratorMgrSensorInfo, status))]
                GoSensorAccelStatus Status;

                [FieldOffset(offsetof(::GoAcceleratorMgrSensorInfo, param))]
                GoAcceleratorMgrSensorParam Param;

                [FieldOffset(offsetof(::GoAcceleratorMgrSensorInfo, backup))]
                GoAcceleratorMgrSensorBackup Backup;
            };

            /// <summary>Represents an accelerator instance.</summary>
            public ref class GoAcceleratorMgr : public KObject
            {
                KDeclareClass(GoAcceleratorMgr, GoAcceleratorMgr)

                /// <summary>Initializes a new instance of the GoAcceleratorMgr class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoAcceleratorMgr(IntPtr handle)
                    : KObject(handle)
                {}

                /// <summary>Initializes a new instance of the GoAcceleratorMgr class.</summary>
                GoAcceleratorMgr()
                {
                    ::GoAcceleratorMgr handle = kNULL;

                    KCheck(::GoAcceleratorMgr_Construct(&handle, kNULL));

                    Handle = handle;
                }

                /// <inheritdoc cref="GoAcceleratorMgr()" />
                /// <param name="allocator">Memory allocator</param>
                GoAcceleratorMgr(KAlloc^ allocator)
                {
                    ::GoAcceleratorMgr handle = kNULL;

                    KCheck(::GoAcceleratorMgr_Construct(&handle, KToHandle(allocator)));

                    Handle = handle;
                }

                /// <summary>Assigns the SDK system object to the accelerator manager object. This must be
                /// done before starting the accelerator manager.</summary>
                void SetSystem(GoSystem^ system)
                {
                    KCheck(::GoAcceleratorMgr_SetSystem(Handle, KToHandle(system)));
                }

                /// <summary>Starts the accelerator manager object after it has been configured.</summary>
                void Start()
                {
                    KCheck(::GoAcceleratorMgr_Start(Handle));
                }

                /// <summary>Accelerate the specified sensor with the given set of parameters.</summary>
                void Accelerate(k32u sensorId, GoAcceleratorMgrSensorParam param)
                {
                    ::GoAcceleratorMgrSensorParam parameters = (::GoAcceleratorMgrSensorParam) param;

                    KCheck(::GoAcceleratorMgr_Accelerate(Handle, sensorId, &parameters));
                }

                /// <summary>Decelerate (unaccelerate) a sensor.</summary>
                void Decelerate(k32u sensorId)
                {
                    KCheck(::GoAcceleratorMgr_Decelerate(Handle, sensorId));
                }

                /// <summary>Get a list of accelerated sensors. The list is returned in an array list
                /// of GoAcceleratorMgrSensorInfo.Caller must construct the list and
                /// pass the list as an argument to this API.The API will fill this list.</summary>
                KArrayList^ ListSensors()
                {
                    KArrayList^ output = gcnew KArrayList(GoAcceleratorMgrSensorInfo::KTypeId);

                    KCheck(::GoAcceleratorMgr_ListSensors(Handle, KToHandle(output)));

                    return output;
                }

                /// <summary>Delegate for a GoAcceleratorMgr connectin status update handler.</summary>
                delegate void UpdateFx(GoAcceleratorMgrAccelUpdate value);

                /// <summary>Registers a callback function that can be used to receive accelerator connection status messages.</summary>
                ///
                /// <remarks>Various locks may be held by the internal thread that invokes the callback; accordingly, the handler function
                /// should avoid blocking the thread.</remarks>
                ///
                /// <param name="handler">Accelerator Update handler function</param>
                void SetAcceleratorUpdateHandler(UpdateFx^ handler)
                {
                    KCallbackFx^ thunk = gcnew KCallbackFx(this, &GoAcceleratorMgr::OnAcceleratorUpdate);
                    KCallbackState^ session = gcnew KCallbackState(thunk, handler);

                    KCheck(::GoAcceleratorMgr_SetAccelUpdateHandler(Handle, (kCallbackFx)session->NativeFunction, session->NativeContext));
                }

                /// <summary>Change the range of port numbers to assign to accelerated sensors.
                /// the port numbers range must be within the port range limits(see
                /// GoAcceleratorMgr_GetPortRangeLimits()).
                /// Changes to port range is allowed only if no sensor is configured for
                /// acceleration.
                /// Configure the port range if your network places limits on what ports are
                /// permitted for communication use.</summary>
                void SetPortRange(K16u startPort, K16u endPort)
                {
                    KCheck(::GoAcceleratorMgr_SetPortRange(Handle, (k16u) startPort, (k16u) endPort));
                }

                /// <summary>Get the range of port numbers to assign to accelerated sensors.</summary>
                void GetPortRange(K16u% start, K16u% end)
                {
                    k16u startPort;
                    k16u endPort;

                    KCheck(::GoAcceleratorMgr_GetPortRange(Handle, &startPort, &endPort));

                    start = startPort;
                    end = endPort;
                }

                /// <summary>Get the minimum and maximum limit values to which the port number range can
                /// be configured and the minimum number of ports that a port range must have.</summary>
                void GetPortRangeLimits(K16u% start, K16u% end, K16u% minPorts)
                {
                    k16u startPort;
                    k16u endPort;
                    k16u minNumPorts;

                    KCheck(::GoAcceleratorMgr_GetPortRangeLimits(Handle, &startPort, &endPort, &minNumPorts));

                    start = startPort;
                    end = endPort;
                    minPorts = minNumPorts;
                }

                /// <summary>Get the number of sensors which the accelerator manager has been configured
                /// to accelerate. The count includes sensors whose acceleration may have failed
                /// for whatever reason.</summary>
                KSize AccelSensorCount()
                {
                    return (KSize(::GoAcceleratorMgr_AccelSensorCount(Handle)));
                }


            private:
                kStatus OnAcceleratorUpdate(kPointer receiver, kPointer sender, kPointer args)
                {
                    KCallbackState^ context = KCallbackState::FromNativeContext(receiver);
                    UpdateFx^ handler = (UpdateFx^)context->Handler;
                    kStatus status = kOK;

                    try
                    {
                        if (handler)
                        {
                            ::GoAcceleratorMgrAccelUpdate value = *(::GoAcceleratorMgrAccelUpdate*)args;
                            handler(GoAcceleratorMgrAccelUpdate(&value));
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
}

#endif

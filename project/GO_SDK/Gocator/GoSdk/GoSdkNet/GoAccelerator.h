// 
// GoAccelerator.h
// 
// Copyright (C) 2016-2021 by LMI Technologies Inc.
// Licensed under the MIT License.
// Redistributed files must retain the above copyright notice.
// 
#ifndef GO_SDK_NET_ACCELERATOR_H
#define GO_SDK_NET_ACCELERATOR_H

#include <GoSdk/GoAccelerator.h>
#include <GoSdkNet/GoSdkDef.h>

namespace Lmi3d
{
    namespace GoSdk
    {
        /// <summary>Represents an accelerator instance.</summary>
        public ref class GoAccelerator : public KObject
        {
            KDeclareClass(GoAccelerator, GoAccelerator)

            /// <summary>Initializes a new instance of the GoAccelerator class with the specified Zen object handle.</summary>
            /// <param name="handle">Zen object handle.</param>
            GoAccelerator(IntPtr handle)
                : KObject(handle)
            {}

            /// <summary>Initializes a new instance of the GoAccelerator class.</summary>
            GoAccelerator()
            {
                ::GoAccelerator handle = kNULL;

                KCheck(::GoAccelerator_Construct(&handle, kNULL));

                Handle = handle;
            }

            /// <inheritdoc cref="GoAccelerator()" />
            /// <param name="allocator">Memory allocator</param>
            GoAccelerator(KAlloc^ allocator)
            {
                ::GoAccelerator handle = kNULL;

                KCheck(::GoAccelerator_Construct(&handle, KToHandle(allocator)));

                Handle = handle;
            }

            /// <summary>Reports whether or not the accelerator has been started.</summary>
            property bool IsRunning
            {
                bool get()           { return KToBool(::GoAccelerator_IsRunning(Handle)); }
            }
            
            /// <summary>Reports whether or not the specified sensor is attached to this accelerator.</summary>
            /// <returns>true if attached, false otherwise</returns>
            bool IsAttached(GoSensor^ sensor)
            {
                return KToBool(::GoAccelerator_IsAttached(Handle, KToHandle(sensor)));
            }

            /// <summary>Starts the accelerator service.</summary>
            void Start()
            {
                KCheck(::GoAccelerator_Start(Handle));
            }

            /// <summary>Attaches the accelerator to a sensor.</summary>
            /// <remarks>Note that subsequent operations on the GoSensor instance will be routed through the accelerator.</remarks>
            void Attach(GoSensor^ sensor)
            {
                KCheck(::GoAccelerator_Attach(Handle, KToHandle(sensor)));
            }

            /// <summary>Detaches a sensor from the accelerator.</summary>
            /// <remarks>Note that subsequent operations on the GoSensor instance will no longer be routed through the accelerator.</remarks>
            void Detach(GoSensor^ sensor)
            {
                KCheck(::GoAccelerator_Detach(Handle, KToHandle(sensor)));
            }

            /// <summary>Stops the accelerator service.</summary>
            /// <remarks>Note that all attached GoSensor instances should be detached before stopping the accelerator.</remarks>
            void Stop()
            {
                KCheck(::GoAccelerator_Stop(Handle));
            }

            /// <summary>Delegate for a GoAccelerator connectin status update handler.</summary>
            delegate void UpdateFx(GoAcceleratorConnectionStatus value);

            /// <summary>Registers a callback function that can be used to receive accelerator connection status messages.</summary>
            /// 
            /// <remarks>Various locks may be held by the internal thread that invokes the callback; accordingly, the handler function
            /// should avoid blocking the thread.</remarks>
            /// 
            /// <param name="handler">Accelerator Update handler function</param>
            void SetAcceleratorUpdateHandler(UpdateFx^ handler)
            {
                KCallbackFx^ thunk = gcnew KCallbackFx(this, &GoAccelerator::OnAcceleratorUpdate);
                KCallbackState^ session = gcnew KCallbackState(thunk, handler);

                KCheck(GoAccelerator_SetAcceleratorUpdateHandler(Handle, (kCallbackFx)session->NativeFunction, session->NativeContext));
            }

            /// <summary>The IP address.</summary>
            property KIpAddress Address
            {
                KIpAddress get()           { return (KIpAddress)::GoAccelerator_IpAddress(Handle); }
                void set(KIpAddress value) { KCheck(::GoAccelerator_SetIpAddress(Handle, value.ToNative())); }
            }

            /// <summary>The control port.</summary>
            property k32u ControlPort
            {
                k32u get()           { return (k32u)::GoAccelerator_ControlPort(Handle); }
                void set(k32u value) { KCheck(::GoAccelerator_SetControlPort(Handle, value)); }
            }

            /// <summary>The health port.</summary>
            property k32u HealthPort
            {
                k32u get()           { return (k32u)::GoAccelerator_HealthPort(Handle); }
                void set(k32u value) { KCheck(::GoAccelerator_SetHealthPort(Handle, value)); }
            }

            /// <summary>The upgrade port.</summary>
            property k32u UpgradePort
            {
                k32u get()           { return (k32u)::GoAccelerator_UpgradePort(Handle); }
                void set(k32u value) { KCheck(::GoAccelerator_SetUpgradePort(Handle, value)); }
            }

            /// <summary>The web server port.</summary>
            property k32u WebPort
            {
                k32u get()           { return (k32u)::GoAccelerator_WebPort(Handle); }
                void set(k32u value) { KCheck(::GoAccelerator_SetWebPort(Handle, value)); }
            }

            /// <summary>The private data port.</summary>
            property k32u PrivateDataPort
            {
                k32u get()           { return (k32u)::GoAccelerator_PrivateDataPort(Handle); }
                void set(k32u value) { KCheck(::GoAccelerator_SetPrivateDataPort(Handle, value)); }
            }

            /// <summary>The public data port.</summary>
            property k32u PublicDataPort
            {
                k32u get()           { return (k32u)::GoAccelerator_PublicDataPort(Handle);}
                void set(k32u value) { KCheck(::GoAccelerator_SetPublicDataPort(Handle, value)); }
            }

            /// <summary></summary>
            property GoSensor^ Sensor
            {
                GoSensor^ get() 
                { 
                    ::GoSensor sensor = ::GoAccelerator_Sensor(Handle); 

                    if (kIsNull(sensor)) { return nullptr; }

                    return KToObject<GoSensor^>(sensor);
                }
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
                        ::GoAcceleratorConnectionStatus value = *(::GoAcceleratorConnectionStatus*)args;
                        handler(GoAcceleratorConnectionStatus(value));
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

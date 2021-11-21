// 
// GoOutput.h
// 
// Copyright (C) 2016-2021 by LMI Technologies Inc.
// Licensed under the MIT License.
// Redistributed files must retain the above copyright notice.
// 
#ifndef GO_SDK_NET_OUTPUT_H
#define GO_SDK_NET_OUTPUT_H

#include <GoSdk/Outputs/GoOutput.h>
#include <GoSdkNet/Outputs/GoAnalog.h>
#include <GoSdkNet/Outputs/GoDigital.h>
#include <GoSdkNet/Outputs/GoEthernet.h>
#include <GoSdkNet/Outputs/GoSerial.h>
#include <GoSdkNet/GoSdkDef.h>

namespace Lmi3d
{
    namespace GoSdk
    {
        namespace Outputs
        {
            /// <summary>Represents output configuration.</summary>
            public ref class GoOutput : public KObject
            {
                KDeclareClass(GoOutput, GoOutput)
            
                /// <summary>Initializes a new instance of the GoOutput class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoOutput(IntPtr handle)
                    : KObject(handle)
                {}
            
                /// <summary>Initializes a new instance of the GoOutput class.</summary>
                /// <param name="sensor">A GoSensor instance.</param>
                GoOutput(GoSensor^ sensor)
                {
                    ::GoOutput handle = kNULL;
            
                    KCheck(::GoOutput_Construct(&handle, KToHandle(sensor), kNULL));
            
                    Handle = handle;
                }
            
                /// <inheritdoc cref="GoOutput(GoSensor^ sensor)" />
                /// <param name="allocator">Memory allocator</param>
                GoOutput(GoSensor^ sensor, KAlloc^ allocator)
                {
                    ::GoOutput handle = kNULL;
            
                    KCheck(::GoOutput_Construct(&handle, KToHandle(sensor), KToHandle(allocator)));
            
                    Handle = handle;
                }
                
                /// <summary>Gets the Ethernet output configuration object.</summary>
                 [Obsolete("Use GetEthernetAt() instead.")]
                /// <returns>Ethernet configuration object.</returns>
                GoEthernet^ GetEthernet()
                {
                    return KToObject<GoEthernet^>(::GoOutput_Ethernet(Handle));
                }

                /// <summary>Gets the count of Ethernet output configuration objects.</summary>
                property k32u EthernetCount
                {
                    k32u get()           { return (k32u) ::GoOutput_EthernetCount(Handle); }
                }
                
                /// <summary>Gets the Ethernet output configuration object at the specified index.</summary>
                /// <param name="index">Ethernet output index.</param>
                /// <returns>Ethernet configuration object.</returns>
                GoEthernet^ GetEthernetAt(k64s index)
                {
                    return KToObject<GoEthernet^>(::GoOutput_EthernetAt(Handle, (kSize)index));
                }
                
                /// <summary>Gets the Serial output configuration object.</summary>
                [Obsolete("Use GetSerialAt() instead.")]
                /// <returns>Serial configuration object.</returns>
                GoSerial^ GetSerial()
                {
                    return KToObject<GoSerial^>(::GoOutput_Serial(Handle));
                }

                 /// <summary>Gets the count of Serial output configuration objects.</summary>
                property k32u SerialCount
                {
                    k32u get()           { return (k32u) ::GoOutput_SerialCount(Handle); }
                }
                
                /// <summary>Gets the Serial output configuration object at the specified index.</summary>
                /// <param name="index">Serial output index.</param>
                /// <returns>Serial configuration object.</returns>
                GoSerial^ GetSerialAt(k64s index)
                {
                    return KToObject<GoSerial^>(::GoOutput_SerialAt(Handle, (kSize)index));
                }
            
                /// <summary>The count of Digital output configuration objects.</summary>
                property k32u DigitalCount
                {
                    k32u get()           { return (k32u) ::GoOutput_DigitalCount(Handle); }
                }
                
                /// <summary>Gets the Digital output configuration object at the specified index.</summary>
                /// <param name="index">Digital output index.</param>
                [Obsolete("Use GetDigitalAt() instead.")]
                /// <returns>Digital output configuration object.</returns>
                GoDigital^ GetDigital(k64s index)
                {
                    return GetDigitalAt(index);
                }

                /// <summary>Gets the Digital output configuration object at the specified index.</summary>
                /// <param name="index">Digital output index.</param>
                /// <returns>Digital output configuration object.</returns>
                GoDigital^ GetDigitalAt(k64s index)
                {
                    return KToObject<GoDigital^>(::GoOutput_DigitalAt(Handle, (kSize)index));
                }
                
                /// <summary>Gets the Analog output configuration object.</summary>
                [Obsolete("Use GetAnalogAt() instead.")]
                /// <returns>Analog configuration object.</returns>
                GoAnalog^ GetAnalog()
                {
                    return KToObject<GoAnalog^>(::GoOutput_Analog(Handle));
                }

                /// <summary>Gets the count of Analog output configuration objects.</summary>
                property k32u AnalogCount
                {
                    k32u get()           { return (k32u) ::GoOutput_AnalogCount(Handle); }
                }
                
                /// <summary>Gets the Analog output configuration object at the specified index.</summary>
                /// <param name="index">Analog output index.</param>
                /// <returns>Analog configuration object.</returns>
                GoAnalog^ GetAnalogAt(k64s index)
                {
                    return KToObject<GoAnalog^>(::GoOutput_AnalogAt(Handle, (kSize)index));
                }
            
            };
        }
    }
}

#endif

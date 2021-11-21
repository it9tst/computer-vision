//
// GoDiscoveryExtInfo.h
//
// Copyright (C) 2016-2021 by LMI Technologies Inc.
// Licensed under the MIT License.
// Redistributed files must retain the above copyright notice.
//
#ifndef GO_SDK_NET_DISCOVERY_EXT_INFO_H
#define GO_SDK_NET_DISCOVERY_EXT_INFO_H

#include <GoSdk/Messages/GoDiscoveryExtInfo.h>
#include <GoSdkNet/GoSdkDef.h>

namespace Lmi3d
{
    namespace GoSdk
    {
        namespace Messages
        {
            /// <summary>Represents a property returned in an extended Discovery information message.</summary>
            [StructLayout(LayoutKind::Explicit, Size = sizeof(::GoDiscoveryProperty))]
            public value struct GoDiscoveryProperty
            {
                KDeclareStruct(GoDiscoveryProperty, GoDiscoveryProperty)

                [FieldOffset(offsetof(::GoDiscoveryProperty, name))]
                KText256 Name;

                [FieldOffset(offsetof(::GoDiscoveryProperty, value))]
                KText256 Value;
            };

            /// <summary>Represents an extended Discovery Information object.</summary>
            public ref class GoDiscoveryExtInfo : public KObject
            {
                KDeclareClass(GoDiscoveryExtInfo, GoDiscoveryExtInfo)

                /// <summary>Initializes a new instance of the GoDiscoveryExtInfo class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoDiscoveryExtInfo(IntPtr handle)
                    : KObject(handle)
                {}

                /// <summary>Initializes a new instance of the GoDiscoveryExtInfo class.</summary>
                GoDiscoveryExtInfo()
                {
                    ::GoDiscoveryExtInfo handle = kNULL;

                    KCheck(::GoDiscoveryExtInfo_Construct(&handle, kNULL));

                    Handle = handle;
                }

                /// <inheritdoc cref="GoDiscoveryExtInfo()" />
                /// <param name="allocator">Memory allocator</param>
                GoDiscoveryExtInfo(KAlloc^ allocator)
                {
                    ::GoDiscoveryExtInfo handle = kNULL;

                    KCheck(::GoDiscoveryExtInfo_Construct(&handle, KToHandle(allocator)));

                    Handle = handle;
                }

                /// <summary>The source device ID.</summary>
                property k32u Id
                {
                    k32u get()           { return (k32u) ::GoDiscoveryExtInfo_Id(Handle); }
                }

                /// <summary>The address information returned by the source device.</summary>
                property GoAddressInfo Address
                {
                    GoAddressInfo get()           { return (GoAddressInfo) ::GoDiscoveryExtInfo_Address(Handle); }
                }

                /// <summary>A set of ports used by the source device.</summary>
                property GoPortInfo Ports
                {
                    GoPortInfo get()           { return (GoPortInfo) ::GoDiscoveryExtInfo_Ports(Handle); }
                }

                /// <summary>The application version of the source device.</summary>
                property KVersion Version
                {
                    KVersion get()           { return (KVersion) ::GoDiscoveryExtInfo_Version(Handle); }
                }

                /// <summary>The up time of the source device.</summary>
                property k64u UpTime
                {
                    k64u get()           { return (k64u) ::GoDiscoveryExtInfo_UpTime(Handle); }
                }

                /// <summary>The number of properties present in the extended discovery information message.</summary>
                property k64s PropertyCount
                {
                    k64s get()           { return (k64s) ::GoDiscoveryExtInfo_PropertyCount(Handle); }
                }

                /// <summary>Retrieves the property associated with the given index.</summary>
                GoDiscoveryProperty GetProperty(k64s index)
                {
                   const ::GoDiscoveryProperty* property = GoDiscoveryExtInfo_PropertyAt(Handle, (kSize)index);

                   if(kIsNull(property)) { throw gcnew KException(kERROR_NOT_FOUND); }

                   return (GoDiscoveryProperty)*property;
                }

                /// <summary>Retrieves the operational mode of the main controller/host. The host can be:
                ///   a. a virtual sensor
                ///   b. a standalone sensor
                ///   c. accelerating a sensor.</summary>
                GoDiscoveryOpMode GetOpMode()
                {
                    return (GoDiscoveryOpMode) ::GoDiscoveryExtInfo_OpMode(Handle);
                }

                /// <summary>Retrieves the IP address of the accelerated the sensor when operational mode indicates the host is accelerating a sensor.</summary>
                KIpAddress GetAccelSensorIpAddress()
                {
                    return (KIpAddress) ::GoDiscoveryExtInfo_AccelSensorIpAddress(Handle);
                }

                /// <summary>Gets the sensor model name.
                /// DEPRECATED: this property is deprecated as of 5.3. Use the PartNumber property instead.</summary>
                property String^ CompleteModel
                {
                    String^ get() 
                    {
                        kText64 model;
                        
                        KCheck(::GoDiscoveryExtInfo_CompleteModel(Handle, model, kCountOf(model)));
                        
                        return KToString(model);
                    }
                }

                /// <summary>Gets the sensor part number.</summary>
                property String^ PartNumber 
                {
                    String^ get() 
                    {
                        kText64 partNumber;
                        
                        KCheck(::GoDiscoveryExtInfo_PartNumber(Handle, partNumber, kCountOf(partNumber)));
                        
                        return KToString(partNumber);
                    }
                }

            };
        }
    }
}

#endif

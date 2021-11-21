// 
// GoSensorInfo.h
// 
// Copyright (C) 2016-2021 by LMI Technologies Inc.
// Licensed under the MIT License.
// Redistributed files must retain the above copyright notice.
// 
#ifndef GO_SDK_NET_SENSOR_INFO_H
#define GO_SDK_NET_SENSOR_INFO_H

#include <GoSdk/GoSensorInfo.h>
#include <GoSdkNet/GoSdkDef.h>

namespace Lmi3d
{
    namespace GoSdk
    {
        /// <summary>Represents read-only sensor information.</summary>
        public ref class GoSensorInfo : public KObject
        {
            KDeclareClass(GoSensorInfo, GoSensorInfo)

            /// <summary>Initializes a new instance of the GoSensorInfo class with the specified Zen object handle.</summary>
            /// <param name="handle">Zen object handle.</param>
            GoSensorInfo(IntPtr handle)
                : KObject(handle)
            {}

            /// <summary>Initializes a new instance of the GoSensorInfo class.</summary>
            GoSensorInfo()
            {
                ::GoSensorInfo handle = kNULL;

                KCheck(::GoSensorInfo_Construct(&handle, kNULL));

                Handle = handle;
            }

            /// <inheritdoc cref="GoSensorInfo(GoSensor^ sensor)" />
            /// <param name="allocator">Memory allocator</param>
            GoSensorInfo(KAlloc^ allocator)
            {
                ::GoSensorInfo handle = kNULL;

                KCheck(::GoSensorInfo_Construct(&handle, KToHandle(allocator)));

                Handle = handle;
            }

            /// <summary>The source ID of the sensor information.</summary>
            property k32u Id
            {
                k32u get()           { return ::GoSensorInfo_Id(Handle); }
            }

            /// <summary>The firmware version of the sensor which provided the sensor information.</summary>
            property KVersion Firmware
            {
                KVersion get()           { return (KVersion) ::GoSensorInfo_Firmware(Handle); }
            }

            /// <summary>Gets The model of the sensor which provided the sensor information.
            /// DEPRECATED: this property is deprecated as of 5.3. Use the PartNumber property instead.</summary>
            property String^ Model
            {
                String^ get() 
                {
                    kText64 model;
                    
                    KCheck(::GoSensorInfo_Model(Handle, model, kCountOf(model)));
                    
                    return KToString(model);
                }
            }

            /// <summary>The part number of the sensor which provided the sensor information.</summary>
            property String^ PartNumber
            {
                String^ get() 
                {
                    kText64 partNumber;
                    
                    KCheck(::GoSensorInfo_PartNumber(Handle, partNumber, kCountOf(partNumber)));
                    
                    return KToString(partNumber);
                }
            }

            /// <summary>The model number of the sensor which provided the sensor information.</summary>
            property String^ ModelNumber 
            {
                String^ get() 
                {
                    kText64 modelNumber;
                    
                    KCheck(::GoSensorInfo_ModelNumber(Handle, modelNumber, kCountOf(modelNumber)));
                    
                    return KToString(modelNumber);
                }
            }

            /// <summary>The model display name of the sensor which provided the sensor information (not for parsing).</summary>
            property String^ ModelDisplayName
            {
                String^ get() 
                {
                    kText64 modelDisplayName;
                    
                    KCheck(::GoSensorInfo_ModelDisplayName(Handle, modelDisplayName, kCountOf(modelDisplayName)));
                    
                    return KToString(modelDisplayName);
                }
            }

            /// <summary>The role of the sensor which provided the sensor information.</summary>
            property GoRole Role
            {
                GoRole get()           { return (GoRole) ::GoSensorInfo_Role(Handle); }
            }

            /// <summary>Indicates whether the device providing the sensor information has a buddy device connected.</summary>
            property bool HasBuddy
            {
                bool get()           { return KToBool(::GoSensorInfo_HasBuddy(Handle)); }
            }

            /// <summary>The ID of the buddy device which provided the sensor information.</summary>
            property k32u BuddyId
            {
                k32u get()           { return ::GoSensorInfo_BuddyId(Handle); }
            }

            /// <summary>The state of the device which provided the sensor information.</summary>
            property GoState State
            {
                GoState get()           { return (GoState) ::GoSensorInfo_State(Handle); }
            }
            
            /// <summary>Gets the budddyabel state of the device which provided the sensor information.</summary>
            property GoBuddyState BuddyableStatus
            {
               GoBuddyState get()   { return (GoBuddyState)::GoSensorInfo_BuddyableStatus(Handle); }
            }

            /// <summary>Gets the id of the main device which provided the sensor information.</summary>
            property k32u MainId
            {
                k32u get()   { return(k32u)::GoSensorInfo_MainId(Handle); }
            }
        };
    }
}

#endif

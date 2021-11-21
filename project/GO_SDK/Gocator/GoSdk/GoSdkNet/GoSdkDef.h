//
// GoSdkDef.h
//
// Copyright (C) 2016-2021 by LMI Technologies Inc.
// Licensed under the MIT License.
// Redistributed files must retain the above copyright notice.
//

#ifndef GO_SDK_NET_DEF_H
#define GO_SDK_NET_DEF_H

// Specify kApiNet.dll as a friend assembly
// Required for conversions between native and .NET types
#ifdef WIN32
    #ifdef _DEBUG
        #using "../../bin/win32d/kApiNet.dll" as_friend
    #else
        #using "../../bin/win32/kApiNet.dll" as_friend
    #endif
#elif WIN64
    #ifdef _DEBUG
        #using "../../bin/win64d/kApiNet.dll" as_friend
    #else
        #using "../../bin/win64/kApiNet.dll" as_friend
    #endif
#endif


#include <GoSdk/GoSdkDef.h>
#include <kApiNet/KApiNet.h>

using namespace Lmi3d::Zen;

//
// Forward declarations.
//

namespace Lmi3d
{
   namespace GoSdk
   {
      ref class GoAccelerator;
      ref class GoLayout;
      ref class GoAdvanced;
      ref class GoMaterial;
      ref class GoMultiplexBank;
      ref class GoPartDetection;
      ref class GoPartMatching;
      ref class GoPartModelEdge;
      ref class GoPartModel;
      ref class GoProfileGeneration;
      ref class GoSection;
      ref class GoSections;
      ref class GoSensor;
      ref class GoSensorInfo;
      ref class GoSetup;
      ref class GoSurfaceGeneration;
      ref class GoSystem;
      ref class GoTransform;
      ref class GoGeoCal;

      namespace Messages
      {
         ref class GoDataSet;
         ref class GoStampMsg;
         ref class GoVideoMsg;
         ref class GoRangeMsg;
         ref class GoRangeIntensityMsg;
         ref class GoProfilePointCloudMsg;
         ref class GoUniformProfileMsg;
         ref class GoProfileIntensityMsg;
         ref class GoUniformSurfaceMsg;
         ref class GoSectionIntensityMsg;
         ref class GoMeasurementMsg;
         ref class GoAlignMsg;
         ref class GoExposureCalMsg;
         ref class GoEdgeMatchMsg;
         ref class GoEllipseMatchMsg;
         ref class GoBoundingBoxMatchMsg;
         ref class GoEventMsg;
         ref class GoDiscoveryExtInfo;
         ref class GoHealthMsg;
         ref class GoTracheidMsg;
         ref class GoSurfacePointCloudMsg;
         ref class GoGenericMsg;
         ref class GoCircleFeatureMsg;
         ref class GoLineFeatureMsg;
         ref class GoPlaneFeatureMsg;
         ref class GoPointFeatureMsg;
         ref class GoMeshMsg;
         //Deprecated:
         ref class GoProfileMsg;
         ref class GoResampledProfileMsg;
         ref class GoSurfaceMsg;
         ref class GoUnresampledSurfaceMsg;
      }

      namespace Outputs
      {
         ref class GoAnalog;
         ref class GoDigital;
         ref class GoEthernet;
         ref class GoOutput;
         ref class GoSerial;
      }

      namespace Tools
      {
          ref class GoExtParam;
          ref class GoExtParams;
      }

      namespace AcceleratorMgr
      {
          ref class GoAcceleratorMgr;
      }
   }
}

namespace Lmi3d
{
    namespace GoSdk
    {
        /// <summary>GoSdk core namespace.</summary>
        [System::Runtime::CompilerServices::CompilerGeneratedAttribute()]
        public ref class NamespaceDoc { };

        /// <summary>Represents the sensor operational state.</summary>
        public value struct GoDeviceState
        {
            KDeclareEnum(GoDeviceState, ::GoDeviceState)

            /// <summary>Sensor cannot be used because it is in a conflicting state.</summary>
            literal k32s Conflict = GO_DEVICE_STATE_CONFLICT;

            /// <summary>Sensor is not scanning but is capable of scanning.</summary>
            literal k32s Ready = GO_DEVICE_STATE_READY;

            /// <summary>Sensor is scanning.</summary>
            literal k32s Running = GO_DEVICE_STATE_RUNNING;
        };

        /// <summary>Represents a user id.</summary>
        public value struct GoUser
        {
            KDeclareEnum(GoUser, ::GoUser)

            /// <summary>No user.</summary>
            literal k32s None = GO_USER_NONE;

            /// <summary>Administrator user.</summary>
            literal k32s Admin = GO_USER_ADMIN;

            /// <summary>Technician user.</summary>
            literal k32s Tech = GO_USER_TECH;
        };

        /// <summary>Represents the current state of a sensor object.</summary>
        public value struct GoState
        {
            KDeclareEnum(GoState, ::GoState)

            /// <summary>Sensor disconnected, but detected via discovery.</summary>
            literal k32s Online = GO_STATE_ONLINE;

            /// <summary>Sensor disconnected and no longer detected via discovery (refresh system to eliminate sensor).</summary>
            literal k32s Offline = GO_STATE_OFFLINE;

            /// <summary>Sensor disconnected and currently resetting (wait for completion).</summary>
            literal k32s Resetting = GO_STATE_RESETTING;

            /// <summary>Sensor connected, but state is otherwise unknown. This is an internal state that is normally not returned.
            /// Seeing this state usually indicates a race condition in the user code. Please see the description of GoSystem regarding thread safety.</summary>
            literal k32s Connected = GO_STATE_CONNECTED;

            /// <summary>Sensor connected, but protocol incompatible with client (upgrade required).</summary>
            literal k32s Incompatible = GO_STATE_INCOMPATIBLE;

            /// <summary>Sensor connected, but remote state was changed (refresh sensor).</summary>
            literal k32s Inconsistent = GO_STATE_INCONSISTENT;

            /// <summary>Sensor connected, but no longer detected via health or discovery (disconnect).</summary>
            literal k32s Unresponsive = GO_STATE_UNRESPONSIVE;

            /// <summary>Sensor connected, but communication aborted via kSxSensor_Cancel function (disconnect or refresh sensor).</summary>
            literal k32s Cancelled = GO_STATE_CANCELLED;

            /// <summary>Sensor connected, but a required buddy sensor is not present (wait or remove buddy association).</summary>
            literal k32s Incomplete = GO_STATE_INCOMPLETE;

            /// <summary>Sensor connected, but is currently busy.</summary>
            literal k32s Busy = GO_STATE_BUSY;

            /// <summary>Sensor connected and ready to accept configuration commands.</summary>
            literal k32s Ready = GO_STATE_READY;

            /// <summary>Sensor connected and currently running.</summary>
            literal k32s Running = GO_STATE_RUNNING;

            /// <summary>Sensor is currently upgrading its firmware.</summary>
            literal k32s Upgrading = GO_STATE_UPGRADING;
        };

        public value struct GoBuddyState
        {
            KDeclareEnum(GoBuddyState, ::GoBuddyState)

            /// <summary>General Error.</summary>
            literal k32s Error = GO_BUDDY_STATE_ERROR;
            /// <summary>Buddy is currently connecting.</summary>
            literal k32s Connecting = GO_BUDDY_STATE_CONNECTING;
            /// <summary>Sensor can be buddied to.</summary>
            literal k32s Connectable = GO_BUDDY_STATE_CONNECTABLE;
            /// <summary>Buddy is connected.</summary>
            literal k32s Connected = GO_BUDDY_STATE_CONNECTED;
            /// <summary>Sensor is already buddied to something else.</summary>
            literal k32s Buddied = GO_BUDDY_STATE_ALREADY_BUDDIED;
            /// <summary>Buddy is in an invalid state.</summary>
            literal k32s InvalidState = GO_BUDDY_STATE_INVALID_STATE;
            /// <summary>The sensors are not currently running the same Gocator firmware version.</summary>
            literal k32s VersionMismatch = GO_BUDDY_STATE_VERSION_MISMATCH;
            /// <summary>Sensors are not of the same model number and cannot be buddied.</summary>
            literal k32s ModelMismatch = GO_BUDDY_STATE_MODEL_MISMATCH;
            /// <summary>Sensor cannot be connected to.</summary>
            literal k32s UnreachableAddress = GO_BUDDY_STATE_UNREACHABLE_ADDRESS;
            /// <summary>Buddied sensor cannot be detected.</summary>
            literal k32s DeviceMissing = GO_BUDDY_STATE_DEVICE_MISSING;
            /// <summary>Sensor buddying encountered a connection error.</summary>
            literal k32s ConnectionError = GO_BUDDY_STATE_ERROR_CONNECTION;
            /// <summary>Maximum number of buddies allow has been reached.</summary>
            literal k32s MaxBuddies = GO_BUDDY_STATE_MAX_BUDDIES;
            /// <summary>Sensor is a StandAlone sensor and cannot be buddied.</summary>
            literal k32s StandAloneSensor = GO_BUDDY_STATE_STANDALONE_NOBUDDY;
            /// <summary>Restricted sensor can only be buddied with another restricted sensor of same model.</summary>
            literal k32s RestrictedMismatch = GO_BUDDY_STATE_RESTRICTED_MISMATCH;
        };

        /// <summary>Represents a user role.</summary>
        /// Use GoRole.Main or GoRole.Buddy + buddyidx.
        public value struct GoRole
        {
            KDeclareEnum(GoRole, ::GoRole)

            /// <summary>Sensor is operating as a main sensor.</summary>
            literal k32s Main = GO_ROLE_MAIN;

            /// <summary>Sensor is operating as a buddy sensor.</summary>
            literal k32s Buddy = GO_ROLE_BUDDY;

            //#define GOROLE_BUDDYIDX(buddyidx) ((k32s)Buddy + buddyidx)
        };

        /// <summary>Represents the status of the Accelerator connection.</summary>
        public value struct GoAcceleratorConnectionStatus
        {
            KDeclareEnum(GoAcceleratorConnectionStatus, ::GoAcceleratorConnectionStatus)

            /// <summary>Accelerated sensor has connected.</summary>
            literal k32s Connected = GO_ACCELERATOR_CONNECTION_STATUS_CONNECTED;

            /// <summary>Accelerated sensor has disconnected.</summary>
            literal k32s Disconnected = GO_ACCELERATOR_CONNECTION_STATUS_DISCONNECTED;

            /// <summary>An error occurred with the accelerated sensor connection.</summary>
            literal k32s Error = GO_ACCELERATOR_CONNECTION_STATUS_ERROR;
        };

        /// <summary>Represents an alignment state.</summary>
        public value struct GoAlignmentState
        {
            KDeclareEnum(GoAlignmentState, ::GoAlignmentState)

            /// <summary>Sensor is not aligned.</summary>
            literal k32s NotAligned = GO_ALIGNMENT_STATE_NOT_ALIGNED;

            /// <summary>Sensor is aligned.</summary>
            literal k32s Aligned = GO_ALIGNMENT_STATE_ALIGNED;
        };

        /// <summary>Represents an alignment reference.</summary>
        public value struct GoAlignmentRef
        {
            KDeclareEnum(GoAlignmentRef, ::GoAlignmentRef)

            /// <summary>The alignment used will be specific to the sensor.</summary>
            literal k32s Fixed = GO_ALIGNMENT_REF_FIXED;

            /// <summary>The alignment used will be specific to the current job if saved.</summary>
            literal k32s Dynamic = GO_ALIGNMENT_REF_DYNAMIC;
        };

        /// <summary>Represents a scan mode.</summary>
        public value struct GoMode
        {
            KDeclareEnum(GoMode, ::GoMode)

            /// <summary>Unknown scan mode.</summary>
            literal k32s Unknown = GO_MODE_UNKNOWN;

            /// <summary>Video scan mode.</summary>
            literal k32s Video = GO_MODE_VIDEO;

            /// <summary>Range scan mode.</summary>
            literal k32s Range = GO_MODE_RANGE;

            /// <summary>Profile scan mode.</summary>
            literal k32s Profile = GO_MODE_PROFILE;

            /// <summary>Surface scan mode.</summary>
            literal k32s Surface = GO_MODE_SURFACE;
        };

        /// <summary>Represents a trigger.</summary>
        public value struct GoTrigger
        {
            KDeclareEnum(GoTrigger, ::GoTrigger)

            /// <summary>The sensor will be time triggered.</summary>
            literal k32s Time = GO_TRIGGER_TIME;

            /// <summary>The sensor will be encoder triggered.</summary>
            literal k32s Encoder = GO_TRIGGER_ENCODER;

            /// <summary>The sensor will be digital input triggered.</summary>
            literal k32s DigitalInput = GO_TRIGGER_INPUT;

            /// <summary>The sensor will be software triggered.</summary>
            literal k32s Software = GO_TRIGGER_SOFTWARE;
        };

        /// <summary>Represents an encoder's triggering behavior.</summary>
        public value struct GoEncoderTriggerMode
        {
            KDeclareEnum(GoEncoderTriggerMode, ::GoEncoderTriggerMode)

            /// <summary>Do not reverse trigger. Track reverse motion to prevent repeat forward triggers.</summary>
            literal k32s TrackReverse = GO_ENCODER_TRIGGER_MODE_TRACK_REVERSE;

            /// <summary>Do not reverse trigger. Forward trigger unconditionally.</summary>
            literal k32s IgnoreReverse = GO_ENCODER_TRIGGER_MODE_IGNORE_REVERSE;

            /// <summary>Forward and reverse trigger.</summary>
            literal k32s Bidirectional = GO_ENCODER_TRIGGER_MODE_BIDIRECTIONAL;
        };

        /// <summary>Represents the current maximum frame rate limiting source.</summary>
        public value struct GoFrameRateMaxSource
        {
            KDeclareEnum(GoFrameRateMaxSource, ::GoFrameRateMaxSource)

            /// <summary>Limited by the sensor's camera configuration.</summary>
            literal k32s Camera = GO_FRAME_RATE_MAX_SOURCE_CAMERA;

            /// <summary>Limited by part detection logic.</summary>
            literal k32s PartDetection = GO_FRAME_RATE_MAX_SOURCE_PART_DETECTION;
        };

        /// <summary>Represents the current encoder period limiting source.</summary>
        public value struct GoEncoderSpacingMinSource
        {
            KDeclareEnum(GoEncoderSpacingMinSource, ::GoEncoderSpacingMinSource)

            /// <summary>Limited by encoder resolution.</summary>
            literal k32s Resolution = GO_ENCODER_PERIOD_MAX_SOURCE_RESOLUTION;

            /// <summary>Limited by encoder resolution.</summary>
            literal k32s PartDetection = GO_ENCODER_PERIOD_MAX_SOURCE_PART_DETECTION;
        };

        /// <summary>Represents the system's primary synchronization domain.</summary>
        public value struct GoTriggerUnits
        {
            KDeclareEnum(GoTriggerUnits, ::GoTriggerUnits)

            /// <summary>Base the system on the internal clock.</summary>
            literal k32s Time = GO_TRIGGER_UNIT_TIME;

            /// <summary>Base the system on the encoder.</summary>
            literal k32s Encoder = GO_TRIGGER_UNIT_ENCODER;
        };

        /// <summary>Represents all possible exposure modes.</summary>
        public value struct GoExposureMode
        {
            KDeclareEnum(GoExposureMode, ::GoExposureMode)

            /// <summary>Single exposure mode.</summary>
            literal k32s Single = GO_EXPOSURE_MODE_SINGLE;

            /// <summary>Multiple exposure mode.</summary>
            literal k32s Multiple = GO_EXPOSURE_MODE_MULTIPLE;

            /// <summary>Dynamic exposure mode.</summary>
            literal k32s Dynamic = GO_EXPOSURE_MODE_DYNAMIC;
        };

        /// <summary>Represents a sensor orientation type.</summary>
        public value struct GoOrientation
        {
            KDeclareEnum(GoOrientation, ::GoOrientation)

            /// <summary>Wide sensor orientation.</summary>
            literal k32s Wide = GO_ORIENTATION_WIDE;

            /// <summary>Opposite sensor orientation.</summary>
            literal k32s Opposite = GO_ORIENTATION_OPPOSITE;

            /// <summary>Reverse sensor orientation.</summary>
            literal k32s Reverse = GO_ORIENTATION_REVERSE;
        };

        /// <summary>Represents a data input source.</summary>
        public value struct GoInputSource
        {
            KDeclareEnum(GoInputSource, ::GoInputSource)

            /// <summary>The current data input source is from live sensor data.</summary>
            literal k32s Live = GO_INPUT_SOURCE_LIVE;

            /// <summary>The current data source is from a replay.</summary>
            literal k32s Recording = GO_INPUT_SOURCE_RECORDING;
        };

        /// <summary>Represents the intensity data source.</summary>
        public value struct GoIntensitySource
        {
            KDeclareEnum(GoIntensitySource, ::GoIntensitySource)

            /// <summary>Generate intensity data using both cameras.</summary>
            literal k32s Both = GO_INTENSITY_SOURCE_BOTH;

            /// <summary>Generate intensity data using front camera.</summary>
            literal k32s Front = GO_INTENSITY_SOURCE_FRONT;

            /// <summary>Generate intensity data using front camera.</summary>
            literal k32s Back = GO_INTENSITY_SOURCE_BACK;
        };

        /// <summary>Represents the intensity generation mode.</summary>
        public value struct GoIntensityMode
        {
            KDeclareEnum(GoIntensityMode, ::GoIntensityMode);

            /// <summary>Auto scale.</summary>
            literal k32s AUTO = GO_INTENSITY_MODE_AUTO;

            /// <summary>Preserve original.</summary>
            literal k32s PRESERVE_ORIGINAL = GO_INTENSITY_MODE_PRESERVE_ORIGINAL;
        };

        /// <summary>Represents a playback seek direction.</summary>
        public value struct GoSeekDirection
        {
            KDeclareEnum(GoSeekDirection, ::GoSeekDirection)

            /// <summary>Seek forward in the current replay.</summary>
            literal k32s Forward = GO_SEEK_DIRECTION_FORWARD;

            /// <summary>Seek backward in the current replay.</summary>
            literal k32s Backward = GO_SEEK_DIRECTION_BACKWARD;
        };

        /// <summary>Represents a data source.</summary>
        public value struct GoDataSource
        {
            KDeclareEnum(GoDataSource, ::GoDataSource)

            /// <summary>Used to represent a buddy device when the buddy is not connected</summary>
            literal k32s None = GO_DATA_SOURCE_NONE;

            /// <summary>Represents main device when in a single sensor or opposite orientation buddy setup. Also represents the combined main and buddy in a wide or reverse orientation</summary>
            literal k32s Top = GO_DATA_SOURCE_TOP;

            /// <summary>Represents the buddy device in an opposite orientation buddy configuration</summary>
            literal k32s Bottom = GO_DATA_SOURCE_BOTTOM;

            /// <summary>Represents the main device in a wide or reverse orientation buddy configuration</summary>
            literal k32s Left = GO_DATA_SOURCE_TOP_LEFT;

            /// <summary>Represents the buddy device in a wide or reverse orientation buddy configuration</summary>
            literal k32s Right = GO_DATA_SOURCE_TOP_RIGHT;

            /// <summary>Represents both the main and buddy devices in a opposite orientation</summary>
            literal k32s TopBottom = GO_DATA_SOURCE_TOP_BOTTOM;

            /// <summary>Represents a buddy configuration where data from the two devices are not merged (e.g. buddied 1000 series sensors in a wide layout)</summary>
            literal k32s LeftRight = GO_DATA_SOURCE_LEFT_RIGHT;

            /// <summary>Used to represent a device in a buddy scenario by adding the device's index to this value to retrieve its data.</summary>
            literal k32s GridBase = GO_DATA_SOURCE_GRID_BASE;
        };

        /// <summary>Represents spacing interval types.</summary>
        public value struct GoSpacingIntervalType
        {
            KDeclareEnum(GoSpacingIntervalType, ::GoSpacingIntervalType)

            /// <summary>Maximum resolution spacing interval type.</summary>
            literal k32s MaxRes = GO_SPACING_INTERVAL_TYPE_MAX_RES;

            /// <summary>Balanced spacing interval type.</summary>
            literal k32s Balanced = GO_SPACING_INTERVAL_TYPE_BALANCED;

            /// <summary>Maximum speed spacing interval type.</summary>
            literal k32s MaxSpeed = GO_SPACING_INTERVAL_TYPE_MAX_SPEED;

            /// <summary>The user specified custom interval.</summary>
            literal k32s Custom = GO_SPACING_INTERVAL_TYPE_CUSTOM;
        };

        /// <summary>Represents a trigger source type.</summary>
        public value struct GoTriggerSource
        {
            KDeclareEnum(GoTriggerSource, ::GoTriggerSource)

            /// <summary>Maximum resolution spacing interval type.</summary>
            literal k32s Time = GO_TRIGGER_SOURCE_TIME;

            /// <summary>Balanced spacing interval type.</summary>
            literal k32s Encoder = GO_TRIGGER_SOURCE_ENCODER;

            /// <summary>Maximum speed spacing interval type.</summary>
            literal k32s Input = GO_TRIGGER_SOURCE_INPUT;

            /// <summary>The user specified custom interval.</summary>
            literal k32s Software = GO_TRIGGER_SOURCE_SOFTWARE;
        };

        /// <summary>Represents an alignment type.</summary>
        public value struct GoAlignmentType
        {
            KDeclareEnum(GoAlignmentType, ::GoAlignmentType)

            /// <summary>Stationary target alignment type.</summary>
            literal k32s Stationary = GO_ALIGNMENT_TYPE_STATIONARY;

            /// <summary>Moving target alignment type.</summary>
            literal k32s Moving = GO_ALIGNMENT_TYPE_MOVING;
        };

        /// <summary>Represents an alignment target type.</summary>
        public value struct GoAlignmentTarget
        {
            KDeclareEnum(GoAlignmentTarget, ::GoAlignmentTarget)

            /// <summary>No calibration target.</summary>
            literal k32s None = GO_ALIGNMENT_TARGET_NONE;

            /// <summary>Calibration disk.</summary>
            literal k32s Disk = GO_ALIGNMENT_TARGET_DISK;

            /// <summary>Calibration bar.</summary>
            literal k32s Bar = GO_ALIGNMENT_TARGET_BAR;

            /// <summary>Calibration plate.</summary>
            literal k32s Plate = GO_ALIGNMENT_TARGET_PLATE;

            /// <summary>Calibration polygon.</summary>
            literal k32s Polygon = GO_ALIGNMENT_TARGET_POLYGON;
        };

        /// <summary>Represents an alignment degrees of freedom setting.</summary>
        public value struct GoAlignmentDegreesOfFreedom
        {
            KDeclareEnum(GoAlignmentDegreesOfFreedom, ::GoAlignmentDegreesOfFreedom)

            /// <summary>No degrees of freedom.</summary>
            literal k32s None = GO_ALIGNMENT_DOF_NONE;

            /// <summary>3 degrees of freeedom.</summary>
            literal k32s Disk = GO_ALIGNMENT_3DOF_XZ_Y;

            /// <summary>4 degrees of freedom.</summary>
            literal k32s Bar = GO_ALIGNMENT_4DOF_XYZ_Y;

            /// <summary>5 degrees of freedom.</summary>
            literal k32s Plate = GO_ALIGNMENT_5DOF_XYZ_YZ;

            /// <summary>6 degrees of freedom.</summary>
            literal k32s LBar = GO_ALIGNMENT_6DOF_XYZ_XYZ;
        };

        /// <summary>Represents the replay export source type.</summary>
        public value struct GoReplayExportSourceType
        {
            KDeclareEnum(GoReplayExportSourceType, ::GoReplayExportSourceType)

            /// <summary>Primary data(relevant to the current scan mode) replay export.</summary>
            literal k32s Primary = GO_REPLAY_EXPORT_SOURCE_PRIMARY;

            /// <summary>Export intensity data using the scan data without regards to aspect ratio.</summary>
            literal k32s Intensity = GO_REPLAY_EXPORT_SOURCE_INTENSITY;

            /// <summary>Export intensity data, resizing data to maintain correct aspect ratio of the image.</summary>
            literal k32s IntensityKeepAspectRatio = GO_REPLAY_EXPORT_SOURCE_INTENSITY_KEEP_ASPECT_RATIO;
        };

        /// <summary>Represents the supported Gocator hardware families.</summary>
        public value struct GoFamily
        {
            KDeclareEnum(GoFamily, ::GoFamily)

            /// <summary>Unidentified sensor family.</summary>
            literal k32s Unknown = GO_FAMILY_UNKNOWN;

            /// <summary>1x00 series sensors.</summary>
            literal k32s _1000 = GO_FAMILY_1000;

            /// <summary>2x00 series sensors.</summary>
            literal k32s _2000 = GO_FAMILY_2000;

            /// <summary>3x00 series sensors.</summary>
            literal k32s _3000 = GO_FAMILY_3000;
        };

        /// <summary>Represents the measurement output decision values. Bit 0 represents the decision value, while bits 1 through 7 represent the decision code, outlined by GoDecisionCode.</summary>
        public value struct GoDecision
        {
            KDeclareEnum(GoDecision, ::GoDecision)

            /// <summary>The measurement value is either valid and falls outside the defined passing decision range or is invalid. The failure error code can be used to determine whether the value was valid.</summary>
            literal k32s Fail = GO_DECISION_FAIL;

            /// <summary>The measurement value is valid and it falls within the defined passing decision range.</summary>
            literal k32s Pass = GO_DECISION_PASS;
        };

        /// <summary>Represents the possible measurement decision codes.</summary>
        public value struct GoDecisionCode
        {
            KDeclareEnum(GoDecisionCode, ::GoDecisionCode)

            /// <summary>The measurement value is valid and it falls outside the defined passing decision range.</summary>
            literal k32s OK = GO_DECISION_CODE_OK;

            /// <summary>The measurement value is invalid.</summary>
            literal k32s InvalidValue = GO_DECISION_CODE_INVALID_VALUE;

            /// <summary>The tool associated with the measurement is anchored is has received invalid measurement data from its anchoring source(s).</summary>
            literal k32s InvalidAnchor = GO_DECISION_CODE_INVALID_ANCHOR;
        };

        /// <summary>Sensor state, login, alignment information, recording state, playback source, uptime, playback information, and auto-start setting state.</summary>
        [StructLayout(LayoutKind::Explicit, Size = sizeof(::GoStates))]
        public value struct GoStates
        {
            KDeclareStruct(GoStates, GoStates)

            /// <summary>The state of the sensor.</summary>
            [FieldOffset(offsetof(::GoStates, sensorState))]
            GoDeviceState SensorState;

            /// <summary>The logged in user.</summary>
            [FieldOffset(offsetof(::GoStates, loginType))]
            GoUser LoginType;

            /// <summary>The alignment reference of the sensor.</summary>
            [FieldOffset(offsetof(::GoStates, alignmentReference))]
            GoAlignmentRef AlignmentReference;

            /// <summary>The alignment state of the sensor.</summary>
            [FieldOffset(offsetof(::GoStates, alignmentState))]
            GoAlignmentState AlignmentState;

            /// <summary>The current state of recording on the sensor.</summary>
            [FieldOffset(offsetof(::GoStates, recordingEnabled))]
            bool RecordingEnabled;

            /// <summary>The current playback source of the sensor.</summary>
            [FieldOffset(offsetof(::GoStates, playbackSource))]
            k32s PlaybackSource;

            /// <summary>Sensor uptime in seconds.</summary>
            [FieldOffset(offsetof(::GoStates, uptimeSec))]
            k32u UptimeSec;

            /// <summary>Sensor uptime in microseconds.</summary>
            [FieldOffset(offsetof(::GoStates, uptimeMicrosec))]
            k32u UptimeMicrosec;

            /// <summary>The playback position index.</summary>
            [FieldOffset(offsetof(::GoStates, playbackPos))]
            k32u PlaybackPos;

            /// <summary>The playback count.</summary>
            [FieldOffset(offsetof(::GoStates, playbackCount))]
            k32u PlaybackCount;

            /// <summary>The auto-start enabled state.</summary>
            [FieldOffset(offsetof(::GoStates, autoStartEnabled))]
            bool AutoStartEnabled;

            /// <summary>The accelerated state of the sensor.</summary>
            [FieldOffset(offsetof(::GoStates, isAccelerator))]
            bool IsAccelerator;

            /// <summary>Power Source Voltage: 24 or 48 V.</summary>
            [FieldOffset(offsetof(::GoStates, voltage))]
            GoVoltageSetting Voltage;

            /// <summary>The length of the cable (in millimeters) from the Sensor to the Master.</summary>
            [FieldOffset(offsetof(::GoStates, cableLength))]
            k32u CableLength;

            /// <summary>The current state of editing.</summary>
            [FieldOffset(offsetof(::GoStates, quickEditEnabled))]
            bool QuickEditEnabled;

            /// <summary>The security level setup on the sensor: none/basic; when basic level does not allow anonymous users accessing system.</summary>
            [FieldOffset(offsetof(::GoStates, security))]
            GoSecurityLevel security;

            /// <summary>The branding type of the sensor (for brand customization schemes).</summary>
            [FieldOffset(offsetof(::GoStates, brandingType))]
            GoBrandingType BrandingType;
        };

        /// <summary>Sensor network address settings.</summary>
        [StructLayout(LayoutKind::Explicit, Size = sizeof(::GoAddressInfo))]
        public value struct GoAddressInfo
        {
            KDeclareStruct(GoAddressInfo, GoAddressInfo)

            /// <summary>Sensor uses DHCP?</summary>
            [FieldOffset(offsetof(::GoAddressInfo, useDhcp))]
            bool UseDhcp;

            /// <summary>Sensor IP address.</summary>
            [FieldOffset(offsetof(::GoAddressInfo, address))]
            KIpAddress Address;

            /// <summary>Sensor subnet bit-mask.</summary>
            [FieldOffset(offsetof(::GoAddressInfo, mask))]
            KIpAddress Mask;

            /// <summary>Sensor gateway address.</summary>
            [FieldOffset(offsetof(::GoAddressInfo, gateway))]
            KIpAddress Gateway;
        };

        /// <summary>Ports used from a source device.</summary>
        [StructLayout(LayoutKind::Explicit, Size = sizeof(::GoPortInfo))]
        public value struct GoPortInfo
        {
            KDeclareStruct(GoPortInfo, GoPortInfo)

            /// <summary>Control channel port.</summary>
            [FieldOffset(offsetof(::GoPortInfo, controlPort))]
            k16u ControlPort;

            /// <summary>Upgrade channel port.</summary>
            [FieldOffset(offsetof(::GoPortInfo, upgradePort))]
            k16u UpgradePort;

            /// <summary>Web channel port.</summary>
            [FieldOffset(offsetof(::GoPortInfo, webPort))]
            k16u WebPort;

            /// <summary>Data channel port.</summary>
            [FieldOffset(offsetof(::GoPortInfo, dataPort))]
            k16u DataPort;

            /// <summary>Health channel port.</summary>
            [FieldOffset(offsetof(::GoPortInfo, healthPort))]
            k16u HealthPort;
        };

        /// <summary>Ports used from a source device.</summary>
        [StructLayout(LayoutKind::Explicit, Size = sizeof(::GoBuddyInfo))]
        public value struct GoBuddyInfo
        {
            KDeclareStruct(GoBuddyInfo, GoBuddyInfo)

            /// <summary>Device serial number.</summary>
            [FieldOffset(offsetof(::GoBuddyInfo, id))]
            k32u id;

            /// <summary>Buddy state of device.</summary>
            [FieldOffset(offsetof(::GoBuddyInfo, state))]
            GoBuddyState state;
        };

        /// <summary>Represents a 64-bit floating point configuration element with a range and enabled state.</summary>
        [StructLayout(LayoutKind::Explicit, Size = sizeof(::GoElement64f))]
        public value struct GoElement64f
        {
            KDeclareStruct(GoElement64f, GoElement64f)

            /// <summary>Represents whether the element value is currently used. (not always applicable)</summary>
            [FieldOffset(offsetof(::GoElement64f, enabled))]
            bool Enabled;

            /// <summary>The system value. (not always applicable)</summary>
            [FieldOffset(offsetof(::GoElement64f, systemValue))]
            k64f SystemValue;

            /// <summary>The element's double field value.</summary>
            [FieldOffset(offsetof(::GoElement64f, value))]
            k64f Value;

            /// <summary>The maximum allowable value that can be set for this element.</summary>
            [FieldOffset(offsetof(::GoElement64f, max))]
            k64f Max;

            /// <summary>The minimum allowable value that can be set for this element.</summary>
            [FieldOffset(offsetof(::GoElement64f, min))]
            k64f Min;
        };

        /// <summary>Represents a 32-bit unsigned integer configuration element with a range and enabled state.</summary>
        [StructLayout(LayoutKind::Explicit, Size = sizeof(::GoElement32u))]
        public value struct GoElement32u
        {
            KDeclareStruct(GoElement32u, GoElement32u)

            /// <summary>Represents whether the element value is currently used.</summary>
            [FieldOffset(offsetof(::GoElement32u, enabled))]
            bool Enabled;

            /// <summary>The system value. (not always applicable)</summary>
            [FieldOffset(offsetof(::GoElement32u, systemValue))]
            k32u SystemValue;

            /// <summary>The element's 32-bit unsigned field value.</summary>
            [FieldOffset(offsetof(::GoElement32u, value))]
            k32u Value;

            /// <summary>The maximum allowable value that can be set for this element.</summary>
            [FieldOffset(offsetof(::GoElement32u, max))]
            k32u Max;

            /// <summary>The minimum allowable value that can be set for this element.</summary>
            [FieldOffset(offsetof(::GoElement32u, min))]
            k32u Min;
        };

        /// <summary>Represents a boolean configuration element with an enabled state.</summary>
        [StructLayout(LayoutKind::Explicit, Size = sizeof(::GoElementBool))]
        public value struct GoElementBool
        {
            KDeclareStruct(GoElementBool, GoElementBool)

            /// <summary>Represents whether the element value is currently used.</summary>
            [FieldOffset(offsetof(::GoElementBool, enabled))]
            bool Enabled;

            /// <summary>The system value. (not always applicable)</summary>
            [FieldOffset(offsetof(::GoElementBool, systemValue))]
            bool SystemValue;

            /// <summary>The element's boolean field value.</summary>
            [FieldOffset(offsetof(::GoElementBool, value))]
            bool Value;
        };

        /// <summary>Represents a filter configuration element.</summary>
        [StructLayout(LayoutKind::Explicit, Size = sizeof(::GoFilter))]
        public value struct GoFilter
        {
            KDeclareStruct(GoFilter, GoFilter)

            /// <summary>Represents whether the element value is currently used.</summary>
            [FieldOffset(offsetof(::GoFilter, used))]
            bool Used;

            /// <summary>The filter's configuration properties</summary>
            [FieldOffset(offsetof(::GoFilter, value))]
            GoElement64f Value;
        };

        /// <summary>Represents an active area configuration element.</summary>
        [StructLayout(LayoutKind::Explicit, Size = sizeof(::GoActiveAreaConfig))]
        public value struct GoActiveAreaConfig
        {
            KDeclareStruct(GoActiveAreaConfig, GoActiveAreaConfig)

            /// <summary>The X offset of the active area.</summary>
            [FieldOffset(offsetof(::GoActiveAreaConfig, x))]
            GoElement64f X;

            /// <summary>The Y offset of the active area.</summary>
            [FieldOffset(offsetof(::GoActiveAreaConfig, y))]
            GoElement64f Y;

            /// <summary>The Z offset of the active area.</summary>
            [FieldOffset(offsetof(::GoActiveAreaConfig, z))]
            GoElement64f Z;

            /// <summary>The height of the active area.</summary>
            [FieldOffset(offsetof(::GoActiveAreaConfig, height))]
            GoElement64f Height;

            /// <summary>The length of the active area.</summary>
            [FieldOffset(offsetof(::GoActiveAreaConfig, length))]
            GoElement64f Length;

            /// <summary>The width of the active area.</summary>
            [FieldOffset(offsetof(::GoActiveAreaConfig, width))]
            GoElement64f Width;
        };

        /// <summary>Represents an alignment element.</summary>
        [StructLayout(LayoutKind::Explicit, Size = sizeof(::GoTransformation))]
        public value struct GoTransformation
        {
            KDeclareStruct(GoTransformation, GoTransformation)

            /// <summary>The X offset of the transformation.</summary>
            [FieldOffset(offsetof(::GoTransformation, x))]
            k64f X;

            /// <summary>The Y offset of the transformation.</summary>
            [FieldOffset(offsetof(::GoTransformation, y))]
            k64f Y;

            /// <summary>The Z offset of the transformation.</summary>
            [FieldOffset(offsetof(::GoTransformation, z))]
            k64f Z;

            /// <summary>The X angle of the transformation.</summary>
            [FieldOffset(offsetof(::GoTransformation, xAngle))]
            k64f XAngle;

            /// <summary>The Y angle of the transformation.</summary>
            [FieldOffset(offsetof(::GoTransformation, yAngle))]
            k64f YAngle;

            /// <summary>The Z angle of the transformation.</summary>
            [FieldOffset(offsetof(::GoTransformation, zAngle))]
            k64f ZAngle;
        };

        /// <summary>Represents a transformed data region.</summary>
        [StructLayout(LayoutKind::Explicit, Size = sizeof(::GoTransformedDataRegion))]
        public value struct GoTransformedDataRegion
        {
            KDeclareStruct(GoTransformedDataRegion, GoTransformedDataRegion)

            /// <summary>The X offset of the transformed data region.</summary>
            [FieldOffset(offsetof(::GoTransformedDataRegion, x))]
            k64f X;

            /// <summary>The Y offset of the transformed data region.</summary>
            [FieldOffset(offsetof(::GoTransformedDataRegion, y))]
            k64f Y;

            /// <summary>The Z offset of the transformed data region.</summary>
            [FieldOffset(offsetof(::GoTransformedDataRegion, z))]
            k64f Z;

            /// <summary>The width of the transformed data region.</summary>
            [FieldOffset(offsetof(::GoTransformedDataRegion, width))]
            k64f Width;

            /// <summary>The length of the transformed data region.</summary>
            [FieldOffset(offsetof(::GoTransformedDataRegion, length))]
            k64f Length;

            /// <summary>The height of the transformed data region.</summary>
            [FieldOffset(offsetof(::GoTransformedDataRegion, height))]
            k64f Height;
        };

        /// <summary>Represents a composite data source.</summary>
        [StructLayout(LayoutKind::Explicit, Size = sizeof(::GoOutputCompositeSource))]
        public value struct GoOutputCompositeSource
        {
            KDeclareStruct(GoOutputCompositeSource, GoOutputCompositeSource)

            /// <summary>The X offset of the transformed data region.</summary>
            [FieldOffset(offsetof(::GoOutputCompositeSource, id))]
            k32s Id;

            /// <summary>The Y offset of the transformed data region.</summary>
            [FieldOffset(offsetof(::GoOutputCompositeSource, dataSource))]
            GoDataSource DataSource;
        };

        /// <summary>Represents an ASCII protocol operational type.</summary>
        public value struct GoAsciiOperation
        {
            KDeclareEnum(GoAsciiOperation, ::GoAsciiOperation)

            /// <summary>Selected measurement output will be sent upon sensor start.</summary>
            literal k32s Asynchronous = GO_ASCII_OPERATION_ASYNCHRONOUS;

            /// <summary>Measurement output will only be sent as requested.</summary>
            literal k32s Polling = GO_ASCII_OPERATION_POLLING;
        };

        /// <summary>Represents an ASCII standard format type.</summary>
        public value struct GoAsciiStandardFormatMode
        {
            KDeclareEnum(GoAsciiStandardFormatMode, ::GoAsciiStandardFormatMode)

            /// <summary>Standard format will output with measurement values and decisions.</summary>
            literal k32s Meas = GS_ASCII_FORMAT_MODE_MEAS;

            /// <summary>Standard format will output with Encoder and Frame, then measurement values and decisions.</summary>
            literal k32s EncoderAndFrame = GS_ASCII_FORMAT_MODE_ENCODER_AND_FRAME;
        };

        /// <summary>Represents the selcom format followed on the serial output.</summary>
        public value struct GoSelcomFormat
        {
            KDeclareEnum(GoSelcomFormat, ::GoSelcomFormat)

            /// <summary>Selcom uses the SLS format</summary>
            literal k32s Sls = GO_SELCOM_FORMAT_SLS;

            /// <summary>Selcom uses the 12-Bit Search/Track format</summary>
            literal k32s _12BitSt = GO_SELCOM_FORMAT_12BIT_ST;

            /// <summary>Selcom uses the 14-Bit format</summary>
            literal k32s _14Bit = GO_SELCOM_FORMAT_14BIT;

            /// <summary>Selcom uses the 14-Bit Search/Track format</summary>
            literal k32s _14BitSt = GO_SELCOM_FORMAT_14BIT_ST;
        };

        /// <summary>Represents all serial output protocols.</summary>
        public value struct GoSerialProtocol
        {
            KDeclareEnum(GoSerialProtocol, ::GoSerialProtocol)

            /// <summary>Gocator serial protocol.</summary>
            literal k32s Gocator = GO_SERIAL_PROTOCOL_GOCATOR;

            /// <summary>Selcom serial protocol.</summary>
            literal k32s Selcom = GO_SERIAL_PROTOCOL_SELCOM;
        };

        /// <summary>Represents an analog output trigger.</summary>
        public value struct GoAnalogTrigger
        {
            KDeclareEnum(GoAnalogTrigger, ::GoAnalogTrigger)

            /// <summary>Analog output triggered by measurement data.</summary>
            literal k32s Measurement = GO_ANALOG_TRIGGER_MEASUREMENT;

            /// <summary>Analog output triggered by software.</summary>
            literal k32s Software = GO_ANALOG_TRIGGER_SOFTWARE;
        };

        /// <summary>Represents a digital output condition.</summary>
        public value struct GoDigitalPass
        {
            KDeclareEnum(GoDigitalPass, ::GoDigitalPass)

            /// <summary>Digital output triggers when all selected measurements pass.</summary>
            literal k32s True = GO_DIGITAL_PASS_TRUE;

            /// <summary>Digital output triggers when all selected measurements fail.</summary>
            literal k32s False = GO_DIGITAL_PASS_FALSE;

            /// <summary>Digital output triggers on every scan.</summary>
            literal k32s Always = GO_DIGITAL_PASS_ALWAYS;
        };

        /// <summary>Represents a digital output signal type.</summary>
        public value struct GoDigitalSignal
        {
            KDeclareEnum(GoDigitalSignal, ::GoDigitalSignal)

            /// <summary>Digital output is pulsed when triggered.</summary>
            literal k32s Pulsed = GO_DIGITAL_SIGNAL_PULSED;

            /// <summary>Digital output is continuous when triggered.</summary>
            literal k32s Continuous = GO_DIGITAL_SIGNAL_CONTINUOUS;
        };

        /// <summary>Represents a digital output event.</summary>
        public value struct GoDigitalEvent
        {
            KDeclareEnum(GoDigitalEvent, ::GoDigitalEvent)

            /// <summary>Digital output is triggered by measurement data.</summary>
            literal k32s Measurement = GO_DIGITAL_EVENT_MEASUREMENT;

            /// <summary>Digital output is triggered by software.</summary>
            literal k32s Software = GO_DIGITAL_EVENT_SOFTWARE;

            /// <summary>Digital output represents the alignment status.</summary>
            literal k32s Alignment = GO_DIGITAL_EVENT_ALIGNMENT;

            /// <summary>Digital output is triggered at the start of exposure.</summary>
            literal k32s Begin = GO_DIGITAL_EVENT_EXPOSURE_BEGIN;

            /// <summary>Digital output is triggered at the end of exposure, prior to processing.</summary>
            literal k32s End = GO_DIGITAL_EVENT_EXPOSURE_END;
        };

        /// <summary>Represents a analog output event.</summary>
        public value struct GoAnalogEvent
        {
            KDeclareEnum(GoAnalogEvent, ::GoAnalogEvent)

            /// <summary>Analog output is triggered by measurement data.</summary>
            literal k32s Measurement = GO_ANALOG_EVENT_MEASURMENT;

            /// <summary>Analog output is triggered by software.</summary>
            literal k32s Software = GO_ANALOG_EVENT_SOFTWARE;
        };

        /// <summary>Represents a ethernet output protocol.</summary>
        public value struct GoEthernetProtocol
        {
            KDeclareEnum(GoEthernetProtocol, ::GoEthernetProtocol)

            /// <summary>Gocator ethernet protocol.</summary>
            literal k32s Gocator = GO_ETHERNET_PROTOCOL_GOCATOR;

            /// <summary>Modbus ethernet protocol.</summary>
            literal k32s Modbus = GO_ETHERNET_PROTOCOL_MODBUS;

            /// <summary>EthernetIP ethernet protocol.</summary>
            literal k32s EthernetIp = GO_ETHERNET_PROTOCOL_ETHERNET_IP;

            /// <summary>ASCII ethernet protocol.</summary>
            literal k32s Ascii = GO_ETHERNET_PROTOCOL_ASCII;

            /// <summary>Profinet ethernet protocol.</summary>
            literal k32s Profinet = GO_ETHERNET_PROTOCOL_PROFINET;

            /// <summary>PTP ethernet protocol.</summary>
            literal k32s Ptp = GO_ETHERNET_PROTOCOL_PTP;
        };

        /// <summary>Represents an endian output type.</summary>
        public value struct GoEndianType
        {
            KDeclareEnum(GoEndianType, ::GoEndianType)

            /// <summary>Big Endian output.</summary>
            literal k32s Big = GO_ENDIAN_TYPE_BIG;

            /// <summary>Little Endian output.</summary>
            literal k32s Little = GO_ENDIAN_TYPE_LITTLE;
        };

        /// <summary>Represents output sources.</summary>
        public value struct GoOutputSource
        {
            KDeclareEnum(GoOutputSource, ::GoOutputSource)

            /// <summary>Unknown output source.</summary>
            literal k32s None = GO_OUTPUT_SOURCE_NONE;

            /// <summary>Output video data.</summary>
            literal k32s Video = GO_OUTPUT_SOURCE_VIDEO;

            /// <summary>Output range data.</summary>
            literal k32s Range = GO_OUTPUT_SOURCE_RANGE;

            /// <summary>Output profile data.</summary>
            literal k32s Profile = GO_OUTPUT_SOURCE_PROFILE;

            /// <summary>Output surface data.</summary>
            literal k32s Surface = GO_OUTPUT_SOURCE_SURFACE;

            /// <summary>Output section data.</summary>
            literal k32s Section = GO_OUTPUT_SOURCE_SECTION;

            /// <summary>Output range intensity data.</summary>
            literal k32s RangeIntensity = GO_OUTPUT_SOURCE_RANGE_INTENSITY;

            /// <summary>Output profile intensity data.</summary>
            literal k32s ProfileIntensity = GO_OUTPUT_SOURCE_PROFILE_INTENSITY;

            /// <summary>Output surface intensity data.</summary>
            literal k32s SurfaceIntensity = GO_OUTPUT_SOURCE_SURFACE_INTENSITY;

            /// <summary>Output section intensity data.</summary>
            literal k32s SectionIntensity = GO_OUTPUT_SOURCE_SECTION_INTENSITY;

            /// <summary>Output measurement data.</summary>
            literal k32s Measurement = GO_OUTPUT_SOURCE_MEASUREMENT;

            /// <summary>Output tracheid data.</summary>
            literal k32s Tracheid = GO_OUTPUT_SOURCE_TRACHEID;

            /// <summary>Output event data.</summary>
            literal k32s Event = GO_OUTPUT_SOURCE_EVENT;

            /// <summary>Output feature data.</summary>
            literal k32s Feature = GO_OUTPUT_SOURCE_FEATURE;

            /// <summary>Output tool data output data.</summary>
            literal k32s ToolData = GO_OUTPUT_SOURCE_TOOLDATA;
        };

        /// <summary>Represents possible data streams.</summary>
        public value struct GoDataStep
        {
            KDeclareEnum(GoDataStep, ::GoDataStep)

            /// <summary>Indicates that no specific stream has been specified.</summary>
            literal k32s None = GO_DATA_STEP_NONE;

            /// <summary>Video data stream.</summary>
            literal k32s Video = GO_DATA_STEP_VIDEO;

            /// <summary>Range data stream.</summary>
            literal k32s Range = GO_DATA_STEP_RANGE;

            /// <summary>Profile data stream.</summary>
            literal k32s Profile = GO_DATA_STEP_PROFILE;

            /// <summary>Surface data stream.</summary>
            literal k32s Surface = GO_DATA_STEP_SURFACE;

            /// <summary>Section data stream.</summary>
            literal k32s Section = GO_DATA_STEP_SECTION;

            /// <summary>Tracheid data stream.</summary>
            literal k32s Tracheid = GO_DATA_STEP_TRACHEID;

            /// <summary>Tool Data Outputs data stream.</summary>
            literal k32s ToolDataOutput = GO_DATA_STEP_TOOLDATA_OUTPUTS;

            /// <summary>Unmerged profile data stream.</summary>
            literal k32s ProfileUnmerged = GO_DATA_STEP_PROFILE_UNMERGED_HDR;

            /// <summary>Oriiginal surface data stream.</summary>
            literal k32s SurfaceOriginal = GO_DATA_STEP_SURFACE_ORIGINAL;
        };

        /// <summary>Represents a data stream which consists of a data step and ID.</summary>
        [StructLayout(LayoutKind::Explicit, Size = sizeof(::GoDataStream))]
        public value struct GoDataStream
        {
            KDeclareStruct(GoDataStream, GoDataStream)

            [FieldOffset(offsetof(::GoDataStream, step))]
            GoDataStep Step;

            [FieldOffset(offsetof(::GoDataStream, id))]
            k32s Id;
        };

        /// <summary>Represents a data stream which consists of a data step and ID.</summary>
        [StructLayout(LayoutKind::Explicit, Size = sizeof(::GoDataStreamId))]
        public value struct GoDataStreamId
        {
            KDeclareStruct(GoDataStreamId, GoDataStreamId)

            [FieldOffset(offsetof(::GoDataStreamId, step))]
            k32s Step;

            [FieldOffset(offsetof(::GoDataStreamId, id))]
            k32s Id;

            [FieldOffset(offsetof(::GoDataStreamId, source))]
            k32s source;
        };

        /// <summary>Represents an output delay domain.</summary>
        public value struct GoOutputDelayDomain
        {
            KDeclareEnum(GoOutputDelayDomain, ::GoOutputDelayDomain)

            /// <summary>Time(uS) based delay domain.</summary>
            literal k32s Time = GO_OUTPUT_DELAY_DOMAIN_TIME;

            /// <summary>Encoder tick delay domain.</summary>
            literal k32s Encoder = GO_OUTPUT_DELAY_DOMAIN_ENCODER;
        };

        /// <summary>Represents a video message pixel type.</summary>
        public value struct GoPixelType
        {
            KDeclareEnum(GoPixelType, ::GoPixelType)

            literal k32s Unknown = GO_PIXEL_TYPE_UNKNOWN;

            /// <summary>Each pixel is represented as unsigned 8-bit values.</summary>
            literal k32s _8U = GO_PIXEL_TYPE_8U;

            /// <summary>Each pixel is represented as three unsigned 8-bit values.</summary>
            literal k32s Rgb = GO_PIXEL_TYPE_RGB;
        };

        /// <summary>Lists all tool types.</summary>
        public value struct GoToolType
        {
            KDeclareEnum(GoToolType, ::GoToolType)

            /// <summary>Unknown tool.</summary>
            literal k32s Unknown = GO_TOOL_UNKNOWN;

            /// <summary>Range Position tool.</summary>
            literal k32s RangePosition = GO_TOOL_RANGE_POSITION;

            /// <summary>Range Thickness tool.</summary>
            literal k32s RangeThickness = GO_TOOL_RANGE_THICKNESS;

            /// <summary>Profile Area tool.</summary>
            literal k32s ProfileArea = GO_TOOL_PROFILE_AREA;

            /// <summary>Profile Bounding Box tool.</summary>
            literal k32s ProfileBoundingBox = GO_TOOL_PROFILE_BOUNDING_BOX;

            /// <summary>Profile Bridge Value tool.</summary>
            literal k32s ProfileBridgeValue = GO_TOOL_PROFILE_BRIDGE_VALUE;

            /// <summary>Profile Circle tool.</summary>
            literal k32s ProfileCircle = GO_TOOL_PROFILE_CIRCLE;

            /// <summary> Profile Dimension tool.</summary>
            literal k32s ProfileDimension = GO_TOOL_PROFILE_DIMENSION;

            /// <summary>Profile Groove tool.</summary>
            literal k32s ProfileGroove = GO_TOOL_PROFILE_GROOVE;

            /// <summary>Profile Intersect tool.</summary>
            literal k32s ProfileIntersect = GO_TOOL_PROFILE_INTERSECT;

            /// <summary>Profile Line tool.</summary>
            literal k32s ProfileLine = GO_TOOL_PROFILE_LINE;

            /// <summary>Profile Panel tool.</summary>
            literal k32s ProfilePanel = GO_TOOL_PROFILE_PANEL;

            /// <summary>Profile Position tool.</summary>
            literal k32s ProfilePosition = GO_TOOL_PROFILE_POSITION;

            /// <summary>Profile Strip tool.</summary>
            literal k32s ProfileStrip = GO_TOOL_PROFILE_STRIP;

            /// <summary>Surface Bounding Box tool.</summary>
            literal k32s SurfaceBoundingBox = GO_TOOL_SURFACE_BOUNDING_BOX;

            /// <summary>Surface Countersunk Hole tool.</summary>
            literal k32s SurfaceCountersunkHole = GO_TOOL_SURFACE_COUNTERSUNK_HOLE;

            /// <summary>Surface Dimension tool.</summary>
            literal k32s SurfaceDimension = GO_TOOL_SURFACE_DIMENSION;

            /// <summary>Surface Ellipse tool.</summary>
            literal k32s SurfaceEllipse = GO_TOOL_SURFACE_ELLIPSE;

            /// <summary>Surface Hole tool.</summary>
            literal k32s SurfaceHole = GO_TOOL_SURFACE_HOLE;

            /// <summary>Surface Opening tool.</summary>
            literal k32s SurfaceOpening = GO_TOOL_SURFACE_OPENING;

            /// <summary>Surface Plane tool.</summary>
            literal k32s SurfacePlane = GO_TOOL_SURFACE_PLANE;

            /// <summary>Surface Position tool.</summary>
            literal k32s SurfacePosition = GO_TOOL_SURFACE_POSITION;

            /// <summary>Surface Rivet tool.</summary>
            literal k32s SurfaceRivet = GO_TOOL_SURFACE_RIVET;

            /// <summary>Surface Stud tool.</summary>
            literal k32s SurfaceStud = GO_TOOL_SURFACE_STUD;

            /// <summary>Surface Volume tool.</summary>
            literal k32s SurfaceVolume = GO_TOOL_SURFACE_VOLUME;

            /// <summary>Script tool.</summary>
            literal k32s Script = GO_TOOL_SCRIPT;

            literal k32s Extensible = GO_TOOL_EXTENSIBLE;
        };

        /// <summary>Lists all measurement types.</summary>
        public value struct GoMeasurementType
        {
            KDeclareEnum(GoMeasurementType, ::GoMeasurementType)

            /// <summary>Unknown measurement.</summary>
            literal k32s Unknown = GO_MEASUREMENT_UNKNOWN;

            /// <summary>Range Position tool Z measurement.</summary>
            literal k32s RangePositionZ = GO_MEASUREMENT_RANGE_POSITION_Z;

            /// <summary>Range Thickness tool Thickness measurement.</summary>
            literal k32s RangeThicknessThickness = GO_MEASUREMENT_RANGE_THICKNESS_THICKNESS;

            /// <summary>Profile Area tool Area measurement.</summary>
            literal k32s ProfileAreaArea = GO_MEASUREMENT_PROFILE_AREA_AREA;

            /// <summary>Profile Area tool Centroid X measurement.</summary>
            literal k32s ProfileAreaCentroidX = GO_MEASUREMENT_PROFILE_AREA_CENTROID_X;

            /// <summary>Profile Area tool Centroid Z measurement.</summary>
            literal k32s ProfileAreaCentroidZ = GO_MEASUREMENT_PROFILE_AREA_CENTROID_Z;

            /// <summary>Profile Bounding Box X measurement.</summary>
            literal k32s ProfileBoundingBoxX = GO_MEASUREMENT_PROFILE_BOUNDING_BOX_X;

            /// <summary>Profile Bounding Box Z measurement.</summary>
            literal k32s ProfileBoundingBoxZ = GO_MEASUREMENT_PROFILE_BOUNDING_BOX_Z;

            /// <summary>Profile Bounding Box Height measurement.</summary>
            literal k32s ProfileBoundingBoxHeight = GO_MEASUREMENT_PROFILE_BOUNDING_BOX_HEIGHT;

            /// <summary>Profile Bounding Box Width measurement.</summary>
            literal k32s ProfileBoundingBoxWidth = GO_MEASUREMENT_PROFILE_BOUNDING_BOX_WIDTH;

            /// <summary>Profile Bounding Box Global X measurement.</summary>
            literal k32s ProfileBoundingBoxGlobalX = GO_MEASUREMENT_PROFILE_BOUNDING_BOX_GLOBAL_X;

            /// <summary>Profile Bounding Box Global Y measurement.</summary>
            literal k32s ProfileBoundingBoxGlobalY = GO_MEASUREMENT_PROFILE_BOUNDING_BOX_GLOBAL_Y;

            /// <summary>Profile Bounding Box Global Angle measurement.</summary>
            literal k32s ProfileBoundingBoxGlobalAngle = GO_MEASUREMENT_PROFILE_BOUNDING_BOX_GLOBAL_ANGLE;

            /// <summary>Profile Bridge Value measurement.</summary>
            literal k32s ProfileBridgeValueBridgeValue = GO_MEASUREMENT_PROFILE_BRIDGE_VALUE_BRIDGE_VALUE;

            /// <summary>Profile Bridge Value measurement.</summary>
            literal k32s ProfileBridgeValueAngle = GO_MEASUREMENT_PROFILE_BRIDGE_VALUE_ANGLE;

            /// <summary>Profile Bridge Value measurement.</summary>
            literal k32s ProfileBridgeValueWindow = GO_MEASUREMENT_PROFILE_BRIDGE_VALUE_WINDOW;

            /// <summary>Profile Bridge Value measurement.</summary>
            literal k32s ProfileBridgeValueStdDev = GO_MEASUREMENT_PROFILE_BRIDGE_VALUE_STDDEV;

            /// <summary>Profile Circle tool X measurement.</summary>
            literal k32s ProfileCircleX = GO_MEASUREMENT_PROFILE_CIRCLE_X;

            /// <summary>Profile Circle tool Z measurement.</summary>
            literal k32s ProfileCircleZ = GO_MEASUREMENT_PROFILE_CIRCLE_Z;

            /// <summary>Profile Circle tool Radius measurement.</summary>
            literal k32s ProfileCircleRadius = GO_MEASUREMENT_PROFILE_CIRCLE_RADIUS;

            /// <summary>Profile Circle tool StdDev measurement.</summary>
            literal k32s ProfileCircleStdDev = GO_MEASUREMENT_PROFILE_CIRCLE_STDDEV;

            /// <summary>Profile Circle tool Min Error measurement.</summary>
            literal k32s ProfileCircleMinError = GO_MEASUREMENT_PROFILE_CIRCLE_MIN_ERROR;

            /// <summary>Profile Circle tool Min Error X measurement.</summary>
            literal k32s ProfileCircleMinErrorX = GO_MEASUREMENT_PROFILE_CIRCLE_MIN_ERROR_X;

            /// <summary>Profile Circle tool Min Error Z measurement.</summary>
            literal k32s ProfileCircleMinErrorZ = GO_MEASUREMENT_PROFILE_CIRCLE_MIN_ERROR_Z;

            /// <summary>Profile Circle tool Max Error measurement.</summary>
            literal k32s ProfileCircleMaxError = GO_MEASUREMENT_PROFILE_CIRCLE_MAX_ERROR;

            /// <summary>Profile Circle tool Max Error X measurement.</summary>
            literal k32s ProfileCircleMaxErrorX = GO_MEASUREMENT_PROFILE_CIRCLE_MAX_ERROR_X;

            /// <summary>Profile Circle tool Max Error Z measurement.</summary>
            literal k32s ProfileCircleMaxErrorZ = GO_MEASUREMENT_PROFILE_CIRCLE_MAX_ERROR_Z;

            /// <summary>Profile Dimension tool Width measurement.</summary>
            literal k32s ProfileDimensionWidth = GO_MEASUREMENT_PROFILE_DIMENSION_WIDTH;

            /// <summary>Profile Dimension tool Height measurement.</summary>
            literal k32s ProfileDimensionHeight = GO_MEASUREMENT_PROFILE_DIMENSION_HEIGHT;

            /// <summary>Profile Dimension tool Distance measurement.</summary>
            literal k32s ProfileDimensionDistance = GO_MEASUREMENT_PROFILE_DIMENSION_DISTANCE;

            /// <summary>Profile Dimension tool Center X measurement.</summary>
            literal k32s ProfileDimensionCenterX = GO_MEASUREMENT_PROFILE_DIMENSION_CENTER_X;

            /// <summary>Profile Dimension tool Center Z measurement.</summary>
            literal k32s ProfileDimensionCenterZ = GO_MEASUREMENT_PROFILE_DIMENSION_CENTER_Z;

            /// <summary>Profile Groove tool X measurement.</summary>
            literal k32s ProfileGrooveX = GO_MEASUREMENT_PROFILE_GROOVE_X;

            /// <summary>Profile Groove tool Z measurement.</summary>
            literal k32s ProfileGrooveZ = GO_MEASUREMENT_PROFILE_GROOVE_Z;

            /// <summary>Profile Groove tool Width measurement.</summary>
            literal k32s ProfileGrooveWidth = GO_MEASUREMENT_PROFILE_GROOVE_WIDTH;

            /// <summary>Profile Groove tool Depth measurement.</summary>
            literal k32s ProfileGrooveDepth = GO_MEASUREMENT_PROFILE_GROOVE_DEPTH;

            /// <summary>Profile Intersect tool X measurement.</summary>
            literal k32s ProfileIntersectX = GO_MEASUREMENT_PROFILE_INTERSECT_X;

            /// <summary>Profile Intersect tool Z measurement.</summary>
            literal k32s ProfileIntersectZ = GO_MEASUREMENT_PROFILE_INTERSECT_Z;

            /// <summary>Profile Intersect tool Angle measurement.</summary>
            literal k32s ProfileIntersectAngle = GO_MEASUREMENT_PROFILE_INTERSECT_ANGLE;

            /// <summary>Profile Line tool Standard Deviation measurement.</summary>
            literal k32s ProfileLineStdDev = GO_MEASUREMENT_PROFILE_LINE_STDDEV;

            /// <summary>Profile Line tool Minimum Error measurement.</summary>
            literal k32s ProfileLineErrorMin = GO_MEASUREMENT_PROFILE_LINE_ERROR_MIN;

            /// <summary>Profile Line tool Maximum Error measurement.</summary>
            literal k32s ProfileLineErrorMax = GO_MEASUREMENT_PROFILE_LINE_ERROR_MAX;

            /// <summary>Profile Line tool Maximum Error measurement.</summary>
            literal k32s ProfileLineOffset = GO_MEASUREMENT_PROFILE_LINE_OFFSET;

            /// <summary>Profile Line tool Maximum Error measurement.</summary>
            literal k32s ProfileLineAngle = GO_MEASUREMENT_PROFILE_LINE_ANGLE;

            /// <summary>Profile Line tool Maximum X Error measurement.</summary>
            literal k32s ProfileLineMinErrorX = GO_MEASUREMENT_PROFILE_LINE_ERROR_MIN_X;

            /// <summary>Profile Line tool Maximum Z Error measurement.</summary>
            literal k32s ProfileLineMinErrorZ = GO_MEASUREMENT_PROFILE_LINE_ERROR_MIN_Z;

            /// <summary>Profile Line tool Maximum X Error measurement.</summary>
            literal k32s ProfileLineMaxErrorX = GO_MEASUREMENT_PROFILE_LINE_ERROR_MAX_X;

            /// <summary>Profile Line tool Maximum Z Error measurement.</summary>
            literal k32s ProfileLineMaxErrorZ = GO_MEASUREMENT_PROFILE_LINE_ERROR_MAX_Z;

            /// <summary>Profile Line tool Percentile measurement.</summary>
            literal k32s ProfileLinePercentile = GO_MEASUREMENT_PROFILE_LINE_PERCENTILE;

            /// <summary>Profile Panel tool Gap measurement.</summary>
            literal k32s ProfilePanelGap = GO_MEASUREMENT_PROFILE_PANEL_GAP;

            /// <summary>Profile Panel tool Flush measurement.</summary>
            literal k32s ProfilePanelFlush = GO_MEASUREMENT_PROFILE_PANEL_FLUSH;

            /// <summary>Profile Panel tool Left Gap X measurement.</summary>
            literal k32s ProfilePanelLeftGapX = GO_MEASUREMENT_PROFILE_PANEL_LEFT_GAP_X;

            /// <summary>Profile Panel tool Left Gap Z measurement.</summary>
            literal k32s ProfilePanelLeftGapZ = GO_MEASUREMENT_PROFILE_PANEL_LEFT_GAP_Z;

            /// <summary>Profile Panel tool Left Gap X measurement.</summary>
            literal k32s ProfilePanelLeftFlushX = GO_MEASUREMENT_PROFILE_PANEL_LEFT_FLUSH_X;

            /// <summary>Profile Panel tool Left Gap Z measurement.</summary>
            literal k32s ProfilePanelLeftFlushZ = GO_MEASUREMENT_PROFILE_PANEL_LEFT_FLUSH_Z;

            /// <summary>Profile Panel tool Left Gap Z measurement.</summary>
            literal k32s ProfilePanelLeftSurfaceAngle = GO_MEASUREMENT_PROFILE_PANEL_LEFT_SURFACE_ANGLE;

            /// <summary>Profile Panel tool Right Gap X measurement.</summary>
            literal k32s ProfilePanelRightGapX = GO_MEASUREMENT_PROFILE_PANEL_RIGHT_GAP_X;

            /// <summary>Profile Panel tool Right Gap Z measurement.</summary>
            literal k32s ProfilePanelRightGapZ = GO_MEASUREMENT_PROFILE_PANEL_RIGHT_GAP_Z;

            /// <summary>Profile Panel tool Right Gap X measurement.</summary>
            literal k32s ProfilePanelRightFlushX = GO_MEASUREMENT_PROFILE_PANEL_RIGHT_FLUSH_X;

            /// <summary>Profile Panel tool Right Gap Z measurement.</summary>
            literal k32s ProfilePanelRightFlushZ = GO_MEASUREMENT_PROFILE_PANEL_RIGHT_FLUSH_Z;

            /// <summary>Profile Panel tool Right Gap Z measurement.</summary>
            literal k32s ProfilePanelRightSurfaceAngle = GO_MEASUREMENT_PROFILE_PANEL_RIGHT_SURFACE_ANGLE;

            /// <summary>Profile Position tool X measurement.</summary>
            literal k32s ProfilePositionX = GO_MEASUREMENT_PROFILE_POSITION_X;

            /// <summary>Profile Position tool Z measurement.</summary>
            literal k32s ProfilePositionZ = GO_MEASUREMENT_PROFILE_POSITION_Z;

            /// <summary>Profile Strip tool X Position measurement.</summary>
            literal k32s ProfileStripPositionX = GO_MEASUREMENT_PROFILE_STRIP_POSITION_X;

            /// <summary>Profile Strip tool Z Position measurement.</summary>
            literal k32s ProfileStripPositionZ = GO_MEASUREMENT_PROFILE_STRIP_POSITION_Z;

            /// <summary>Profile Strip tool Width measurement.</summary>
            literal k32s ProfileStripWidth = GO_MEASUREMENT_PROFILE_STRIP_WIDTH;

            /// <summary>Profile Strip tool Height measurement.</summary>
            literal k32s ProfileStripHeight = GO_MEASUREMENT_PROFILE_STRIP_HEIGHT;

            /// <summary>Surface Bounding Box X measurement.</summary>
            literal k32s SurfaceBoundingBoxX = GO_MEASUREMENT_SURFACE_BOUNDING_BOX_X;

            /// <summary>Surface Bounding Box Y measurement.</summary>
            literal k32s SurfaceBoundingBoxY = GO_MEASUREMENT_SURFACE_BOUNDING_BOX_Y;

            /// <summary>Surface Bounding Box Z measurement.</summary>
            literal k32s SurfaceBoundingBoxZ = GO_MEASUREMENT_SURFACE_BOUNDING_BOX_Z;

            /// <summary>Surface Bounding Box Z Angle measurement.</summary>
            literal k32s SurfaceBoundingBoxZAngle = GO_MEASUREMENT_SURFACE_BOUNDING_BOX_ZANGLE;

            /// <summary>Surface Bounding Box Height measurement.</summary>
            literal k32s SurfaceBoundingBoxHeight = GO_MEASUREMENT_SURFACE_BOUNDING_BOX_HEIGHT;

            /// <summary>Surface Bounding Box Width measurement.</summary>
            literal k32s SurfaceBoundingBoxWidth = GO_MEASUREMENT_SURFACE_BOUNDING_BOX_WIDTH;

            /// <summary>Surface Bounding Box Length measurement.</summary>
            literal k32s SurfaceBoundingBoxLength = GO_MEASUREMENT_SURFACE_BOUNDING_BOX_LENGTH;

            /// <summary>Surface Bounding Box Global X measurement.</summary>
            literal k32s SurfaceBoundingBoxGlobalX = GO_MEASUREMENT_SURFACE_BOUNDING_BOX_GLOBAL_X;

            /// <summary>Surface Bounding Box Global Y measurement.</summary>
            literal k32s SurfaceBoundingBoxGlobalY = GO_MEASUREMENT_SURFACE_BOUNDING_BOX_GLOBAL_Y;

            /// <summary>Surface Bounding Box Global Z Angle measurement.</summary>
            literal k32s SurfaceBoundingBoxGlobalZAngle = GO_MEASUREMENT_SURFACE_BOUNDING_BOX_GLOBAL_Z_ANGLE;

            /// <summary>Surface Countersunk Hole tool X position measurement.</summary>
            literal k32s SurfaceCountersunkHoleX = GO_MEASUREMENT_SURFACE_COUNTERSUNK_HOLE_X;

            /// <summary>Surface Countersunk Hole tool Y position measurement.</summary>
            literal k32s SurfaceCountersunkHoleY = GO_MEASUREMENT_SURFACE_COUNTERSUNK_HOLE_Y;

            /// <summary>Surface Countersunk Hole tool Z position measurement.</summary>
            literal k32s SurfaceCountersunkHoleZ = GO_MEASUREMENT_SURFACE_COUNTERSUNK_HOLE_Z;

            /// <summary>Surface Countersunk Hole tool Outer Radius measurement.</summary>
            literal k32s SurfaceCountersunkHoleOuterRadius = GO_MEASUREMENT_SURFACE_COUNTERSUNK_HOLE_OUTER_RADIUS;

            /// <summary>Surface Countersunk Hole tool Depth measurement.</summary>
            literal k32s SurfaceCountersunkHoleDepth = GO_MEASUREMENT_SURFACE_COUNTERSUNK_HOLE_DEPTH;

            /// <summary>Surface Countersunk Hole tool Counterbore Depth measurement.</summary>
            literal k32s SurfaceCountersunkHoleCounterboreDepth = GO_MEASUREMENT_SURFACE_COUNTERSUNK_HOLE_COUNTERBORE_DEPTH;

            /// <summary>Surface Countersunk Hole tool Bevel Radius measurement.</summary>
            literal k32s SurfaceCountersunkHoleBevelRadius = GO_MEASUREMENT_SURFACE_COUNTERSUNK_HOLE_BEVEL_RADIUS;

            /// <summary>Surface Countersunk Hole tool Bevel Angle measurement.</summary>
            literal k32s SurfaceCountersunkHoleBevelAngle = GO_MEASUREMENT_SURFACE_COUNTERSUNK_HOLE_BEVEL_ANGLE;

            /// <summary>Surface Countersunk Hole tool X Angle measurement.</summary>
            literal k32s SurfaceCountersunkHoleXAngle = GO_MEASUREMENT_SURFACE_COUNTERSUNK_HOLE_X_ANGLE;

            /// <summary>Surface Countersunk Hole tool Y Angle measurement.</summary>
            literal k32s SurfaceCountersunkHoleYAngle = GO_MEASUREMENT_SURFACE_COUNTERSUNK_HOLE_Y_ANGLE;

            /// <summary>Surface Countersunk Hole tool axis tilt measurement.</summary>
            literal k32s SurfaceCountersunkHoleAxisTilt = GO_MEASUREMENT_SURFACE_COUNTERSUNK_HOLE_AXIS_TILT;

            /// <summary>Surface Countersunk Hole tool axis orientation measurement.</summary>
            literal k32s SurfaceCountersunkHoleAxisOrientation = GO_MEASUREMENT_SURFACE_COUNTERSUNK_HOLE_AXIS_ORIENTATION;

            /// <summary>Surface Dimension tool Width measurement.</summary>
            literal k32s SurfaceDimensionWidth = GO_MEASUREMENT_SURFACE_DIMENSION_WIDTH;

            /// <summary>Surface Dimension tool Height measurement.</summary>
            literal k32s SurfaceDimensionHeight = GO_MEASUREMENT_SURFACE_DIMENSION_HEIGHT;

            /// <summary>Surface Dimension tool Length measurement.</summary>
            literal k32s SurfaceDimensionLength = GO_MEASUREMENT_SURFACE_DIMENSION_LENGTH;

            /// <summary>Surface Dimension tool Distance measurement.</summary>
            literal k32s SurfaceDimensionDistance = GO_MEASUREMENT_SURFACE_DIMENSION_DISTANCE;

            /// <summary>Surface Dimension tool Plane Distance measurement.</summary>
            literal k32s SurfaceDimensionPlaneDistance = GO_MEASUREMENT_SURFACE_DIMENSION_PLANE_DISTANCE;

            /// <summary>Surface Dimension tool Center X measurement.</summary>
            literal k32s SurfaceDimensionCenterX = GO_MEASUREMENT_SURFACE_DIMENSION_CENTER_X;

            /// <summary>Surface Dimension tool Center Y measurement.</summary>
            literal k32s SurfaceDimensionCenterY = GO_MEASUREMENT_SURFACE_DIMENSION_CENTER_Y;

            /// <summary>Surface Dimension tool Center Z measurement.</summary>
            literal k32s SurfaceDimensionCenterZ = GO_MEASUREMENT_SURFACE_DIMENSION_CENTER_Z;

            /// <summary>Surface Ellipse tool Major measurement.</summary>
            literal k32s SurfaceEllipseMajor = GO_MEASUREMENT_SURFACE_ELLIPSE_MAJOR;

            /// <summary>Surface Ellipse tool Minor measurement.</summary>
            literal k32s SurfaceEllipseMinor = GO_MEASUREMENT_SURFACE_ELLIPSE_MINOR;

            /// <summary>Surface Ellipse tool Ratio measurement.</summary>
            literal k32s SurfaceEllipseRatio = GO_MEASUREMENT_SURFACE_ELLIPSE_RATIO;

            /// <summary>Surface Ellipse tool Z Angle measurement.</summary>
            literal k32s SurfaceEllipseZAngle = GO_MEASUREMENT_SURFACE_ELLIPSE_ZANGLE;

            /// <summary>Surface Hole tool X measurement.</summary>
            literal k32s SurfaceHoleX = GO_MEASUREMENT_SURFACE_HOLE_X;

            /// <summary>Surface Hole tool Y measurement.</summary>
            literal k32s SurfaceHoleY = GO_MEASUREMENT_SURFACE_HOLE_Y;

            /// <summary>Surface Hole tool Z measurement.</summary>
            literal k32s SurfaceHoleZ = GO_MEASUREMENT_SURFACE_HOLE_Z;

            /// <summary>Surface Hole tool Radius measurement.</summary>
            literal k32s SurfaceHoleRadius = GO_MEASUREMENT_SURFACE_HOLE_RADIUS;

            /// <summary>Surface Opening tool X measurement.</summary>
            literal k32s SurfaceOpeningX = GO_MEASUREMENT_SURFACE_OPENING_X;

            /// <summary>Surface Opening tool Y measurement.</summary>
            literal k32s SurfaceOpeningY = GO_MEASUREMENT_SURFACE_OPENING_Y;

            /// <summary>Surface Opening tool Z measurement.</summary>
            literal k32s SurfaceOpeningZ = GO_MEASUREMENT_SURFACE_OPENING_Z;

            /// <summary>Surface Opening tool Width measurement.</summary>
            literal k32s SurfaceOpeningWidth = GO_MEASUREMENT_SURFACE_OPENING_WIDTH;

            /// <summary>Surface Opening tool Length measurement.</summary>
            literal k32s SurfaceOpeningLength = GO_MEASUREMENT_SURFACE_OPENING_LENGTH;

            /// <summary>Surface Opening tool Angle measurement.</summary>
            literal k32s SurfaceOpeningAngle = GO_MEASUREMENT_SURFACE_OPENING_ANGLE;

            /// <summary>Surface Plane tool X Angle measurement.</summary>
            literal k32s SurfacePlaneXAngle = GO_MEASUREMENT_SURFACE_PLANE_X_ANGLE;

            /// <summary>Surface Plane tool Y Angle measurement.</summary>
            literal k32s SurfacePlaneYAngle = GO_MEASUREMENT_SURFACE_PLANE_Y_ANGLE;

            /// <summary>Surface Plane tool Z Offset measurement.</summary>
            literal k32s SurfacePlaneZOffset = GO_MEASUREMENT_SURFACE_PLANE_Z_OFFSET;

            /// <summary>Surface Plane tool Standard Deviation measurement.</summary>
            literal k32s SurfacePlaneStdDev = GO_MEASUREMENT_SURFACE_PLANE_STD_DEV;

            /// <summary>Surface Plane tool Minimum Error measurement.</summary>
            literal k32s SurfacePlaneErrorMin = GO_MEASUREMENT_SURFACE_PLANE_ERROR_MIN;

            /// <summary>Surface Plane tool Maximum Error measurement.</summary>
            literal k32s SurfacePlaneErrorMax = GO_MEASUREMENT_SURFACE_PLANE_ERROR_MAX;

            /// <summary>Surface Plane tool X Normal measurement.</summary>
            literal k32s SurfacePlaneXNormal = GO_MEASUREMENT_SURFACE_PLANE_X_NORMAL;

            /// <summary>Surface Plane tool Y Normal measurement.</summary>
            literal k32s SurfacePlaneYNormal = GO_MEASUREMENT_SURFACE_PLANE_Y_NORMAL;

            /// <summary>Surface Plane tool Z Normal measurement.</summary>
            literal k32s SurfacePlaneZNormal = GO_MEASUREMENT_SURFACE_PLANE_Z_NORMAL;

            /// <summary>Surface Plane tool Distance measurement.</summary>
            literal k32s SurfacePlaneDistance = GO_MEASUREMENT_SURFACE_PLANE_DISTANCE;

            /// <summary>Surface Position tool X measurement.</summary>
            literal k32s SurfacePositionX = GO_MEASUREMENT_SURFACE_POSITION_X;

            /// <summary>Surface Position tool Y measurement.</summary>
            literal k32s SurfacePositionY = GO_MEASUREMENT_SURFACE_POSITION_Y;

            /// <summary>Surface Position tool Z measurement.</summary>
            literal k32s SurfacePositionZ = GO_MEASUREMENT_SURFACE_POSITION_Z;

            /// <summary>Surface Rivet tool X measurement.</summary>
            literal k32s SurfaceRivetX = GO_MEASUREMENT_SURFACE_RIVET_X;

            /// <summary>Surface Rivet tool Y measurement.</summary>
            literal k32s SurfaceRivetY = GO_MEASUREMENT_SURFACE_RIVET_Y;

            /// <summary>Surface Rivet tool Z measurement.</summary>
            literal k32s SurfaceRivetZ = GO_MEASUREMENT_SURFACE_RIVET_Z;

            /// <summary>Surface Rivet tool X Angle measurement.</summary>
            literal k32s SurfaceRivetTiltAngle = GO_MEASUREMENT_SURFACE_RIVET_TILT_ANGLE;

            /// <summary>Surface Rivet tool Y Angle measurement.</summary>
            literal k32s SurfaceRivetTiltDirection = GO_MEASUREMENT_SURFACE_RIVET_TILT_DIRECTION;

            /// <summary>Surface Rivet tool Radius measurement.</summary>
            literal k32s SurfaceRivetRadius = GO_MEASUREMENT_SURFACE_RIVET_RADIUS;

            /// <summary>Surface Rivet tool Top Offset Minimum measurement.</summary>
            literal k32s SurfaceRivetTopOffsetMin = GO_MEASUREMENT_SURFACE_RIVET_TOP_OFFSET_MIN;

            /// <summary>Surface Rivet tool Top Offset Maximum measurement.</summary>
            literal k32s SurfaceRivetTopOffsetMax = GO_MEASUREMENT_SURFACE_RIVET_TOP_OFFSET_MAX;

            /// <summary>Surface Rivet tool Top Offset Mean measurement.</summary>
            literal k32s SurfaceRivetTopOffsetMean = GO_MEASUREMENT_SURFACE_RIVET_TOP_OFFSET_MEAN;

            /// <summary>Surface Rivet tool Top Offset Standard Deviation measurement.</summary>
            literal k32s SurfaceRivetTopOffsetStdDev = GO_MEASUREMENT_SURFACE_RIVET_TOP_OFFSET_STD_DEV;

            /// <summary>Surface Rivet tool Radial Height Minimum measurement.</summary>
            literal k32s SurfaceRivetRadialHeightMin = GO_MEASUREMENT_SURFACE_RIVET_RADIAL_HEIGHT_MIN;

            /// <summary>Surface Rivet tool Radial Height Maximum measurement.</summary>
            literal k32s SurfaceRivetRadialHeightMax = GO_MEASUREMENT_SURFACE_RIVET_RADIAL_HEIGHT_MAX;

            /// <summary>Surface Rivet tool Radial Height Mean measurement.</summary>
            literal k32s SurfaceRivetRadialHeightMean = GO_MEASUREMENT_SURFACE_RIVET_RADIAL_HEIGHT_MEAN;

            /// <summary>Surface Rivet tool Radial Height Standard Deviation measurement.</summary>
            literal k32s SurfaceRivetRadialHeightStdDev = GO_MEASUREMENT_SURFACE_RIVET_RADIAL_HEIGHT_STD_DEV;

            /// <summary>Surface Rivet tool Radial Slope Minimum measurement.</summary>
            literal k32s SurfaceRivetRadialSlopeMin = GO_MEASUREMENT_SURFACE_RIVET_RADIAL_SLOPE_MIN;

            /// <summary>Surface Rivet tool Radial Slope Maximum measurement.</summary>
            literal k32s SurfaceRivetRadialSlopeMax = GO_MEASUREMENT_SURFACE_RIVET_RADIAL_SLOPE_MAX;

            /// <summary>Surface Rivet tool Radial Slope Mean measurement.</summary>
            literal k32s SurfaceRivetRadialSlopeMean = GO_MEASUREMENT_SURFACE_RIVET_RADIAL_SLOPE_MEAN;

            /// <summary>Surface Rivet tool Radial Slope Standard Deviation measurement.</summary>
            literal k32s SurfaceRivetRadialSlopeStdDev = GO_MEASUREMENT_SURFACE_RIVET_RADIAL_SLOPE_STD_DEV;

            /// <summary>Surface Stud tool Base X measurement.</summary>
            literal k32s SurfaceStudBaseX = GO_MEASUREMENT_SURFACE_STUD_BASE_X;

            /// <summary>Surface Stud tool Base Y measurement.</summary>
            literal k32s SurfaceStudBaseY = GO_MEASUREMENT_SURFACE_STUD_BASE_Y;

            /// <summary>Surface Stud tool Base Z measurement.</summary>
            literal k32s SurfaceStudBaseZ = GO_MEASUREMENT_SURFACE_STUD_BASE_Z;

            /// <summary>Surface Stud tool Tip X measurement.</summary>
            literal k32s SurfaceStudTipX = GO_MEASUREMENT_SURFACE_STUD_TIP_X;

            /// <summary>Surface Stud tool Tip Y measurement.</summary>
            literal k32s SurfaceStudTipY = GO_MEASUREMENT_SURFACE_STUD_TIP_Y;

            /// <summary> Surface Stud tool Tip Z measurement.</summary>
            literal k32s SurfaceStudTipZ = GO_MEASUREMENT_SURFACE_STUD_TIP_Z;

            /// <summary>Surface Stud tool Radius measurement.</summary>
            literal k32s SurfaceStudRadius = GO_MEASUREMENT_SURFACE_STUD_RADIUS;

            /// <summary>Surface Volume tool Area measurement.</summary>
            literal k32s SurfaceVolumeArea = GO_MEASUREMENT_SURFACE_VOLUME_AREA;

            /// <summary>Surface Volume tool Volume measurement.</summary>
            literal k32s SurfaceVolumeVolume = GO_MEASUREMENT_SURFACE_VOLUME_VOLUME;

            /// <summary>Surface Volume tool Thickness measurement.</summary>
            literal k32s SurfaceVolumeThickness = GO_MEASUREMENT_SURFACE_VOLUME_THICKNESS;

            /// <summary>Unknown measurement.</summary>
            literal k32s FeatureDimensionWidth = GO_FEATURE_DIMENSION_WIDTH;

            /// <summary>Range Position tool Z measurement.</summary>
            literal k32s FeatureDimensionLenght = GO_FEATURE_DIMENSION_LENGTH;

            /// <summary>Range Thickness tool Thickness measurement.</summary>
            literal k32s FeatureDimensionHeight = GO_FEATURE_DIMENSION_HEIGHT;

            /// <summary>Profile Area tool Area measurement.</summary>
            literal k32s FeatureDimensionDistance = GO_FEATURE_DIMENSION_DISTANCE;

            /// <summary>Profile Area tool Centroid X measurement.</summary>
            literal k32s FeatureDimensionPlaneDistance = GO_FEATURE_DIMENSION_PLANEDISTANCE;

            /// <summary>Profile Area tool Centroid Z measurement.</summary>
            literal k32s FeatureDimensionCenterX = GO_FEATURE_DIMENSION_CENTERX;

            /// <summary>Profile Bounding Box X measurement.</summary>
            literal k32s FeatureDimensionCenterY = GO_FEATURE_DIMENSION_CENTERY;

            /// <summary>Profile Bounding Box Z measurement.</summary>
            literal k32s FeatureDimensionCenterZ = GO_FEATURE_DIMENSION_CENTERZ;

            /// <summary>Script tool Output.</summary>
            literal k32s ScriptOutput = GO_MEASUREMENT_SCRIPT_OUTPUT;

            /// <summary>Extensible tool measurement.</summary>
            literal k32s Extensible = GO_MEASUREMENT_EXTENSIBLE;
        };


        /// <summary>Lists all data message types.</summary>
        public value struct GoDataMessageType
        {
            KDeclareEnum(GoDataMessageType, ::GoDataMessageType)

            /// <summary>Unknown message type.</summary>
            literal k32s Unknown = GO_DATA_MESSAGE_TYPE_UNKNOWN;

            /// <summary>Stamp message type.</summary>
            literal k32s Stamp = GO_DATA_MESSAGE_TYPE_STAMP;

            /// <summary>Health message type.</summary>
            literal k32s Health = GO_DATA_MESSAGE_TYPE_HEALTH;

            /// <summary>Video message type.</summary>
            literal k32s Video = GO_DATA_MESSAGE_TYPE_VIDEO;

            /// <summary>Range message type.</summary>
            literal k32s Range = GO_DATA_MESSAGE_TYPE_RANGE;

            /// <summary>Range Intensity message type.</summary>
            literal k32s RangeIntensity = GO_DATA_MESSAGE_TYPE_RANGE_INTENSITY;

            /// <summary>Profile point cloud message type. </summary>
            literal k32s ProfilePointCloud = GO_DATA_MESSAGE_TYPE_PROFILE_POINT_CLOUD;

            /// <summary>Profile(or Resampled if uniform spacing is enabled) Intensity message type.</summary>
            literal k32s ProfileIntensity = GO_DATA_MESSAGE_TYPE_PROFILE_INTENSITY;

            /// <summary>Uniform Profile message type.</summary>
            literal k32s UniformProfile = GO_DATA_MESSAGE_TYPE_UNIFORM_PROFILE;

            /// <summary>Surface message type.</summary>
            literal k32s UniformSurface = GO_DATA_MESSAGE_TYPE_UNIFORM_SURFACE;

            /// <summary>Surface Point Cloud message type.</summary>
            literal k32s SurfacePointCloud = GO_DATA_MESSAGE_TYPE_SURFACE_POINT_CLOUD;

            /// <summary>Surface Intensity message type.</summary>
            literal k32s SurfaceIntensity = GO_DATA_MESSAGE_TYPE_SURFACE_INTENSITY;

            /// <summary>Measurement message type.</summary>
            literal k32s Measurement = GO_DATA_MESSAGE_TYPE_MEASUREMENT;

            /// <summary>Alignment result message type.</summary>
            literal k32s Alignment = GO_DATA_MESSAGE_TYPE_ALIGNMENT;

            /// <summary>Exposure AutoSet/Calibration result message type.</summary>
            literal k32s ExposureCal = GO_DATA_MESSAGE_TYPE_EXPOSURE_CAL;

            /// <summary>Part matching edge algorithm message type.</summary>
            literal k32s EdgeMatch = GO_DATA_MESSAGE_TYPE_EDGE_MATCH;

            /// <summary>Part matching bounding box algorithm message type.</summary>
            literal k32s BoundingBoxMatch = GO_DATA_MESSAGE_TYPE_BOUNDING_BOX_MATCH;

            /// <summary>Part matching ellipse algorithm message type.</summary>
            literal k32s EllipseMatch = GO_DATA_MESSAGE_TYPE_ELLIPSE_MATCH;

            /// <summary>Section message type.</summary>
            literal k32s Section = GO_DATA_MESSAGE_TYPE_SECTION;

            /// <summary>Section Intensity message type.</summary>
            literal k32s SectionIntensity = GO_DATA_MESSAGE_TYPE_SECTION_INTENSITY;

            /// <summary>Event message type.</summary>
            literal k32s Event = GO_DATA_MESSAGE_TYPE_EVENT;

            /// <summary>Tracheid message type.</summary>
            literal k32s Tracheid = GO_DATA_MESSAGE_TYPE_TRACHEID;

            /// <summary>Generic message type.</summary>
            literal k32s Generic = GO_DATA_MESSAGE_TYPE_GENERIC;

            /// <summary>Mesh message type.</summary>
            literal k32s Mesh = GO_DATA_MESSAGE_TYPE_MESH;

            /// <summary>Profile message type. Deprecated: Use ProfilePointCloud instead</summary>
            literal k32s Profile = ProfilePointCloud;

            /// <summary>Resampled Profile message type. Deprecated: Use UniformProfile instead</summary>
            literal k32s ResampledProfile = UniformProfile;

            /// <summary>Surface message type. Deprecated: Use UniformSurface instead</summary>
            literal k32s Surface = UniformSurface;
        };

        /// <summary>Represents a replay condition type.</summary>
        public value struct GoReplayConditionType
        {
            KDeclareEnum(GoReplayConditionType, ::GoReplayConditionType)

            /// <summary>Any Measurement condition.</summary>
            literal k32s AnyMeasurement = GO_REPLAY_CONDITION_TYPE_ANY_MEASUREMENT;

            /// <summary>Any Data condition.</summary>
            literal k32s AnyData = GO_REPLAY_CONDITION_TYPE_ANY_DATA;

            /// <summary>Measurement condition.</summary>
            literal k32s Measurement = GO_REPLAY_CONDITION_TYPE_MEASUREMENT;
        };

        /// <summary>Represents a replay combine type.</summary>
        public value struct GoReplayCombineType
        {
            KDeclareEnum(GoReplayCombineType, ::GoReplayCombineType)

            /// <summary>Any</summary>
            literal k32s Any = GO_REPLAY_COMBINE_TYPE_ANY;

            /// <summary>All</summary>
            literal k32s All = GO_REPLAY_COMBINE_TYPE_ALL;
        };

        /// <summary>Represents a replay measurement result.</summary>
        public value struct GoReplayMeasurementResult
        {
            KDeclareEnum(GoReplayMeasurementResult, ::GoReplayMeasurementResult)

            /// <summary>Pass</summary>
            literal k32s Pass = GO_REPLAY_MEASUREMENT_RESULT_PASS;

            /// <summary>Fail</summary>
            literal k32s Fail = GO_REPLAY_MEASUREMENT_RESULT_FAIL;

            /// <summary>Valid</summary>
            literal k32s Valid = GO_REPLAY_MEASUREMENT_RESULT_VALID;

            /// <summary>Invalid</summary>
            literal k32s Invalid = GO_REPLAY_MEASUREMENT_RESULT_INVALID;

            /// <summary>Fail or Invalid</summary>
            literal k32s FailOrInvalid = GO_REPLAY_MEASUREMENT_RESULT_FAIL_OR_INVALID;
        };

        /// <summary>Represents a replay range count case.</summary>
        public value struct GoReplayRangeCountCase
        {
            KDeclareEnum(GoReplayRangeCountCase, ::GoReplayRangeCountCase)

            /// <summary>Case at above</summary>
            literal k32s AtAbove = GO_REPLAY_RANGE_COUNT_CASE_AT_ABOVE;

            /// <summary>Case below</summary>
            literal k32s Below = GO_REPLAY_RANGE_COUNT_CASE_BELOW;
        };

        /// <summary>Represents a advanced acquisition type.</summary>
        public value struct GoAdvancedType
        {
            KDeclareEnum(GoAdvancedType, ::GoAdvancedType)

                /// <summary>Custom advanced acquisition type.</summary>
                literal k32s Custom = GO_ADVANCED_TYPE_CUSTOM;

            /// <summary>Diffuse advanced acquisition type.</summary>
            literal k32s Diffuse = GO_ADVANCED_TYPE_DIFFUSE;

            /// <summary>Reflective advanced acquisition type.</summary>
            literal k32s Reflective = GO_ADVANCED_TYPE_REFLECTIVE;
        };

        /// <summary>Represents a material acquisition type.</summary>
        public value struct GoMaterialType
        {
            KDeclareEnum(GoMaterialType, ::GoMaterialType)

            /// <summary>Custom material acquisition type.</summary>
            literal k32s Custom = GO_MATERIAL_TYPE_CUSTOM;

            /// <summary>Diffuse material acquisition type.</summary>
            literal k32s Diffuse = GO_MATERIAL_TYPE_DIFFUSE;
        };

        /// <summary>Represents a spot selection type.</summary>
        public value struct GoSpotSelectionType
        {
            KDeclareEnum(GoSpotSelectionType, ::GoSpotSelectionType)

            /// <summary>Select the spot with the best value.</summary>
            literal k32s Best = GO_SPOT_SELECTION_TYPE_BEST;

            /// <summary>Select the top-most spot.</summary>
            literal k32s Top = GO_SPOT_SELECTION_TYPE_TOP;

            /// <summary>Select the bottom-most spot.</summary>
            literal k32s Bottom = GO_SPOT_SELECTION_TYPE_BOTTOM;

            /// <summary>Disable spot selection.</summary>
            literal k32s None = GO_SPOT_SELECTION_TYPE_NONE;

            /// <summary>Select the spot in a continuous range.</summary>
            literal k32s Continunity = GO_SPOT_SELECTION_TYPE_CONTINUITY;

            /// <summary>Select the translucent spot.</summary>
            literal k32s Translucent = GO_SPOT_SELECTION_TYPE_TRANSLUCENT;
        };

        /// <summary>Represents a translucent spot selection threading mode.</summary>
        public value struct GoTranslucentThreadingMode
        {
            KDeclareEnum(GoTranslucentThreadingMode, ::GoTranslucentThreadingMode)

            /// <summary>Single threaded mode.</summary>
            literal k32s None = GO_TRANSLUCENT_THREADING_MODE_NONE;

            /// <summary>Batching mode.</summary>
            literal k32s Batching = GO_TRANSLUCENT_THREADING_MODE_BATCHING;  
        };

        /// <summary>Represents a profile strip tool base type.</summary>
        public value struct GoProfileStripBaseType
        {
            KDeclareEnum(GoProfileStripBaseType, ::GoProfileStripBaseType)

            /// <summary>No strip base type.</summary>
            literal k32s Custom = GO_PROFILE_STRIP_BASE_TYPE_NONE;

            /// <summary>Flat strip base type.</summary>
            literal k32s Diffuse = GO_PROFILE_STRIP_BASE_TYPE_FLAT;
        };

        /// <summary>Represents a profile strip tool edge type.</summary>
        public value struct GoProfileStripEdgeType
        {
            KDeclareEnum(GoProfileStripEdgeType, ::GoProfileStripEdgeType)

            /// <summary>Rising strip edge type.</summary>
            literal k32s Rising = GO_PROFILE_STRIP_EDGE_TYPE_RISING;

            /// <summary>Falling strip edge type.</summary>
            literal k32s Falling = GO_PROFILE_STRIP_EDGE_TYPE_FALLING;

            /// <summary>Data end strip edge type.</summary>
            literal k32s DataEnd = GO_PROFILE_STRIP_EDGE_TYPE_DATA_END;

            /// <summary>Void strip edge type.</summary>
            literal k32s Void = GO_PROFILE_STRIP_EDGE_TYPE_VOID;
        };

        /// <summary>Represents a profile feature point type.</summary>
        public value struct GoProfileFeatureType
        {
            KDeclareEnum(GoProfileFeatureType, ::GoProfileFeatureType)

            /// <summary>Point with the maximum Z value.</summary>
            literal k32s MaxZ = GO_PROFILE_FEATURE_TYPE_MAX_Z;

            /// <summary>Point with the minimum Z value.</summary>
            literal k32s MinZ = GO_PROFILE_FEATURE_TYPE_MIN_Z;

            /// <summary>Point with the maximum X value.</summary>
            literal k32s MaxX = GO_PROFILE_FEATURE_TYPE_MAX_X;

            /// <summary>Point with the minimum X value.</summary>
            literal k32s MinX = GO_PROFILE_FEATURE_TYPE_MIN_X;

            /// <summary>Dominant corner.</summary>
            literal k32s Corner = GO_PROFILE_FEATURE_TYPE_CORNER;

            /// <summary>Average of points.</summary>
            literal k32s Average = GO_PROFILE_FEATURE_TYPE_AVERAGE;

            /// <summary>Rising edge.</summary>
            literal k32s RisingEdge = GO_PROFILE_FEATURE_TYPE_RISING_EDGE;

            /// <summary>Falling edge.</summary>
            literal k32s FallingEdge = GO_PROFILE_FEATURE_TYPE_FALLING_EDGE;

            /// <summary>Rising or falling edge.</summary>
            literal k32s AnyEdge = GO_PROFILE_FEATURE_TYPE_ANY_EDGE;

            /// <summary>Top-most corner.</summary>
            literal k32s TopCorner = GO_PROFILE_FEATURE_TYPE_TOP_CORNER;

            /// <summary>Bottom-most corner.</summary>
            literal k32s BottomCorner = GO_PROFILE_FEATURE_TYPE_BOTTOM_CORNER;

            /// <summary>Left-most corner.</summary>
            literal k32s LeftCorner = GO_PROFILE_FEATURE_TYPE_LEFT_CORNER;

            /// <summary>Right-most corner.</summary>
            literal k32s RightCorner = GO_PROFILE_FEATURE_TYPE_RIGHT_CORNER;

            /// <summary>Median of points.</summary>
            literal k32s Median = GO_PROFILE_FEATURE_TYPE_MEDIAN;
        };

        /// <summary>Represents a profile gap measurement axis.</summary>
        public value struct GoProfileGapAxis
        {
            KDeclareEnum(GoProfileGapAxis, ::GoProfileGapAxis)

            /// <summary>Measure the gap along the edge normal.</summary>
            literal k32s Edge = GO_PROFILE_GAP_AXIS_EDGE;

            /// <summary>Measure the gap along the surface line.</summary>
            literal k32s Surface = GO_PROFILE_GAP_AXIS_SURFACE;

            /// <summary>Measure the shortest distance between the two edges.</summary>
            literal k32s Distance = GO_PROFILE_GAP_AXIS_DISTANCE;
        };

        /// <summary>Represents a profile edge type.</summary>
        public value struct GoProfileEdgeType
        {
            KDeclareEnum(GoProfileEdgeType, ::GoProfileEdgeType)

            /// <summary>Detect the edge by looking for the tangent.</summary>
            literal k32s Tangent = GO_PROFILE_EDGE_TYPE_TANGENT;

            /// <summary>Detect the edge by looking for the corner</summary>
            literal k32s Corner = GO_PROFILE_EDGE_TYPE_CORNER;
        };

        /// <summary>Determines whether to use a line based on a Profile Line fit, or based on the x-axis.</summary>
        public value struct GoProfileBaseline
        {
            KDeclareEnum(GoProfileBaseline, ::GoProfileBaseline)

            /// <summary>Use the X-Axis.</summary>
            literal k32s XAxis = GO_PROFILE_BASELINE_TYPE_X_AXIS;

            /// <summary>Use the Z-Axis.</summary>
            literal k32s ZAxis = GO_PROFILE_BASELINE_TYPE_Z_AXIS;

            /// <summary>Use the line fit.</summary>
            literal k32s Line = GO_PROFILE_BASELINE_TYPE_LINE;
        };

        /// <summary>Determines how to calculate profile area</summary>
        public value struct GoProfileAreaType
        {
            KDeclareEnum(GoProfileAreaType, ::GoProfileAreaType)

            /// <summary>Sum the profile area that is above the line.</summary>
            literal k32s Object = GO_PROFILE_AREA_TYPE_OBJECT;

            /// <summary>Sum the profile area that is below the line.</summary>
            literal k32s Clearance = GO_PROFILE_AREA_TYPE_CLEARANCE;
        };

        /// <summary>Selects which edge to use as the reference in a panel tool.</summary>
        public value struct GoProfilePanelSide
        {
            KDeclareEnum(GoProfilePanelSide, ::GoProfilePanelSide)

            /// <summary>Use the left edge.</summary>
            literal k32s Left = GO_PROFILE_PANEL_SIDE_LEFT;

            /// <summary>Use the right edge.</summary>
            literal k32s Right = GO_PROFILE_PANEL_SIDE_RIGHT;
        };

        /// <summary>Indicates the reference edge direction for a round corner tool.</summary>
        public value struct GoProfileRoundCornerDirection
        {
            KDeclareEnum(GoProfileRoundCornerDirection, ::GoProfileRoundCornerDirection)

            /// <summary>Use the left edge.</summary>
            literal k32s Left = GO_PROFILE_ROUND_CORNER_DIRECTION_LEFT;

            /// <summary>Use the right edge.</summary>
            literal k32s Right = GO_PROFILE_ROUND_CORNER_DIRECTION_LEFT;
        };

        /// <summary>Represents a profile edge type.</summary>
        public value struct GoProfileGrooveShape
        {
            KDeclareEnum(GoProfileGrooveShape, ::GoProfileGrooveShape)

            /// <summary>Detect grooves that are U shaped.</summary>
            literal k32s U = GO_PROFILE_GROOVE_SHAPE_U;

            /// <summary>Detect grooves that are V shaped.</summary>
            literal k32s V = GO_PROFILE_GROOVE_SHAPE_V;

            /// <summary>Detect grooves that are open.</summary>
            literal k32s Open = GO_PROFILE_GROOVE_SHAPE_OPEN;
        };

        /// <summary>Determines which groove to select when multiple are present.</summary>
        public value struct GoProfileGrooveSelectType
        {
            KDeclareEnum(GoProfileGrooveSelectType, ::GoProfileGrooveSelectType)

            /// <summary>Select the groove with the maximum depth.</summary>
            literal k32s MaxDepth = GO_PROFILE_GROOVE_SELECT_TYPE_MAX_DEPTH;

            /// <summary>Select the groove with the currently selected index starting from the left side.</summary>
            literal k32s LeftIndex = GO_PROFILE_GROOVE_SELECT_TYPE_LEFT_INDEX;

            /// <summary>Select the groove with the currently selected index starting from the right side.</summary>
            literal k32s RightIndex = GO_PROFILE_GROOVE_SELECT_TYPE_RIGHT_INDEX;
        };

        /// <summary>Determines which groove position to return.</summary>
        public value struct GoProfileGrooveLocation
        {
            KDeclareEnum(GoProfileGrooveLocation, ::GoProfileGrooveLocation)

            /// <summary>Return the position of the bottom of the groove.</summary>
            literal k32s Bottom = GO_PROFILE_GROOVE_LOCATION_BOTTOM;

            /// <summary>Return the position of the left corner of the groove.</summary>
            literal k32s Left = GO_PROFILE_GROOVE_LOCATION_LEFT;

            /// <summary>Return the position of the right corner of the groove.</summary>
            literal k32s Right = GO_PROFILE_GROOVE_LOCATION_RIGHT;
        };

        /// <summary>Determines which Strip to select when multiple are present.</summary>
        public value struct GoProfileStripSelectType
        {
            KDeclareEnum(GoProfileStripSelectType, ::GoProfileStripSelectType)

            /// <summary>Select the best strip.</summary>
            literal k32s Best = GO_PROFILE_STRIP_SELECT_TYPE_BEST;

            /// <summary>Select the strip with the currently selected index starting from the left side.</summary>
            literal k32s LeftIndex = GO_PROFILE_STRIP_SELECT_TYPE_LEFT_INDEX;

            /// <summary>Select the strip with the currently selected index starting from the right side.</summary>
            literal k32s RightIndex = GO_PROFILE_STRIP_SELECT_TYPE_RIGHT_INDEX;
        };

        /// <summary>Determines which Strip position to return.</summary>
        public value struct GoProfileStripLocation
        {
            KDeclareEnum(GoProfileStripLocation, ::GoProfileStripLocation)

            /// <summary>Return the position of the left corner of the Strip.</summary>
            literal k32s Best = GO_PROFILE_STRIP_LOCATION_LEFT;

            /// <summary>Return the position of the right corner of the Strip.</summary>
            literal k32s LeftIndex = GO_PROFILE_STRIP_LOCATION_RIGHT;

            /// <summary>Return the position of the center of the Strip.</summary>
            literal k32s RightIndex = GO_PROFILE_STRIP_LOCATION_BOTTOM;
        };

        /// <summary>Represents a profile generation type.</summary>
        public value struct GoProfileGenerationType
        {
            KDeclareEnum(GoProfileGenerationType, ::GoProfileGenerationType)

            /// <summary>Continuous Profile generation.</summary>
            literal k32s Continuous = GO_PROFILE_GENERATION_TYPE_CONTINUOUS;

            /// <summary>Fixed length Profile generation.</summary>
            literal k32s FixedLength = GO_PROFILE_GENERATION_TYPE_FIXED_LENGTH;

            /// <summary>Variable length Profile generation.</summary>
            literal k32s VariableLength = GO_PROFILE_GENERATION_TYPE_VARIABLE_LENGTH;

            /// <summary>Rotational Profile generation.</summary>
            literal k32s Rotational = GO_PROFILE_GENERATION_TYPE_ROTATIONAL;
        };

        /// <summary>Represents a profile generation start trigger.</summary>
        public value struct GoProfileGenerationStartTrigger
        {
            KDeclareEnum(GoProfileGenerationStartTrigger, ::GoProfileGenerationStartTrigger)

            /// <summary>Sequential start trigger.</summary>
            literal k32s Sequential = GO_PROFILE_GENERATION_START_TRIGGER_SEQUENTIAL;

            /// <summary>Digital input start trigger.</summary>
            literal k32s Digital = GO_PROFILE_GENERATION_START_TRIGGER_DIGITAL;
        };

        /// <summary>Represents a part detection frame of reference.</summary>
        public value struct GoPartFrameOfReference
        {
            KDeclareEnum(GoPartFrameOfReference, ::GoPartFrameOfReference)

            /// <summary>Sensor frame of reference. 2x00 only.</summary>
            literal k32s Sensor = GO_PART_FRAME_OF_REFERENCE_TYPE_SENSOR;

            /// <summary>Scan frame of reference. 3x00 only. Value duplication is intentional.</summary>
            literal k32s Scan = GO_PART_FRAME_OF_REFERENCE_TYPE_SCAN;

            /// <summary>Part frame of reference.</summary>
            literal k32s Part = GO_PART_FRAME_OF_REFERENCE_TYPE_PART;
        };

        /// <summary>Represents a part detection height threshold direction.</summary>
        public value struct GoPartHeightThresholdDirection
        {
            KDeclareEnum(GoPartHeightThresholdDirection, ::GoPartHeightThresholdDirection)

            /// <summary>Height threshold direction is above the Z-axis.</summary>
            literal k32s Above = GO_PART_HEIGHT_THRESHOLD_DIRECTION_ABOVE;

            /// <summary>Height threshold direction is below the Z-axis.</summary>
            literal k32s Below = GO_PART_HEIGHT_THRESHOLD_DIRECTION_BELOW;
        };

        /// <summary>Represents a surface generation type.</summary>
        public value struct GoSurfaceGenerationType
        {
            KDeclareEnum(GoSurfaceGenerationType, ::GoSurfaceGenerationType)

            /// <summary>Continuous surface generation.</summary>
            literal k32s Continuous = GO_SURFACE_GENERATION_TYPE_CONTINUOUS;

            /// <summary>Fixed length surface generation.</summary>
            literal k32s FixedLength = GO_SURFACE_GENERATION_TYPE_FIXED_LENGTH;

            /// <summary>Variable length surface generation.</summary>
            literal k32s VariableLength = GO_SURFACE_GENERATION_TYPE_VARIABLE_LENGTH;

            /// <summary>Rotational surface generation.</summary>
            literal k32s Rotational = GO_SURFACE_GENERATION_TYPE_ROTATIONAL;
        };

        /// <summary>Represents a surface generation start trigger.</summary>
        public value struct GoSurfaceGenerationStartTrigger
        {
            KDeclareEnum(GoSurfaceGenerationStartTrigger, ::GoSurfaceGenerationStartTrigger)

            /// <summary>Sequential start trigger.</summary>
            literal k32s Sequential = GO_SURFACE_GENERATION_START_TRIGGER_SEQUENTIAL;

            /// <summary>Digital input start trigger.</summary>
            literal k32s Digital = GO_SURFACE_GENERATION_START_TRIGGER_DIGITAL;

            /// <summary>Software start trigger.</summary>
            literal k32s Software = GO_SURFACE_GENERATION_START_TRIGGER_SOFTWARE;
        };

        /// <summary>Represents a surface location.</summary>
        public value struct GoSurfaceLocation
        {
            KDeclareEnum(GoSurfaceLocation, ::GoSurfaceLocation)

            /// <summary>Location based on the maximum point.</summary>
            literal k32s Max = GO_SURFACE_LOCATION_TYPE_MAX;

            /// <summary>Location based on the minimum point.</summary>
            literal k32s Min = GO_SURFACE_LOCATION_TYPE_MIN;

            /// <summary>Location based on a 2d centroid.</summary>
            literal k32s _2dCentroid = GO_SURFACE_LOCATION_TYPE_2D_CENTROID;

            /// <summary>Location based on a 3d centroid.</summary>
            literal k32s _3dCentroid = GO_SURFACE_LOCATION_TYPE_3D_CENTROID;

            /// <summary>Location based on the average point.</summary>
            literal k32s Avg = GO_SURFACE_LOCATION_TYPE_AVG;

            /// <summary>Location based on the median point.</summary>
            literal k32s Median = GO_SURFACE_LOCATION_TYPE_MEDIAN;
        };

        /// <summary></summary>
        public value struct GoSurfaceFeatureType
        {
            KDeclareEnum(GoSurfaceFeatureType, ::GoSurfaceFeatureType)

            /// <summary>Feature based on the average.</summary>
            literal k32s Average = GO_SURFACE_FEATURE_TYPE_AVERAGE;

            /// <summary>Feature based on the centroid.</summary>
            literal k32s Centroid = GO_SURFACE_FEATURE_TYPE_CENTROID;

            /// <summary>Feature based on the X maximum point.</summary>
            literal k32s XMax = GO_SURFACE_FEATURE_TYPE_X_MAX;

            /// <summary>Feature based on the X minimum point.</summary>
            literal k32s XMin = GO_SURFACE_FEATURE_TYPE_X_MIN;

            /// <summary>Feature based on the Y maximum point.</summary>
            literal k32s YMax = GO_SURFACE_FEATURE_TYPE_Y_MAX;

            /// <summary>Feature based on the Y minimum point.</summary>
            literal k32s YMin = GO_SURFACE_FEATURE_TYPE_Y_MIN;

            /// <summary>Feature based on the Z maximum point.</summary>
            literal k32s ZMax = GO_SURFACE_FEATURE_TYPE_Z_MAX;

            /// <summary>Feature based on the Z minimum point.</summary>
            literal k32s ZMin = GO_SURFACE_FEATURE_TYPE_Z_MIN;

            /// <summary>Feature based on the median.</summary>
            literal k32s Median = GO_SURFACE_FEATURE_TYPE_MEDIAN;
        };

        /// <summary>Represents a surface countersunk hole tool shape.</summary>
        public value struct GoSurfaceCountersunkHoleShape
        {
            KDeclareEnum(GoSurfaceCountersunkHoleShape, ::GoSurfaceCountersunkHoleShape)

            /// <summary>Cone shape.</summary>
            literal k32s Cone = GO_SURFACE_COUNTERSUNK_HOLE_SHAPE_CONE;

            /// <summary>Counterbore shape.</summary>
            literal k32s Counterbore = GO_SURFACE_COUNTERSUNK_HOLE_SHAPE_COUNTERBORE;
        };

        /// <summary>Represents a surface opening tool type.</summary>
        public value struct GoSurfaceOpeningType
        {
            KDeclareEnum(GoSurfaceOpeningType, ::GoSurfaceOpeningType)

            /// <summary>Rounded slot opening type.</summary>
            literal k32s RoundedSlot = GO_SURFACE_OPENING_TYPE_ROUNDED_SLOT;

            /// <summary>Rectangular opening type.</summary>
            literal k32s RoundedRectangle = GO_SURFACE_OPENING_TYPE_ROUNDED_RECTANGLE;
        };

        /// <summary>Represents a surface rivet tool type.</summary>
        public value struct GoSurfaceRivetType
        {
            KDeclareEnum(GoSurfaceRivetType, ::GoSurfaceRivetType)

            /// <summary>Flush rivet type.</summary>
            literal k32s Flush = GO_SURFACE_RIVET_TYPE_FLUSH;

            /// <summary>Raised rivet type.</summary>
            literal k32s Raised = GO_SURFACE_RIVET_TYPE_RAISED;
        };

        /// <summary>Represents a part matching algorithm.</summary>
        public value struct GoPartMatchAlgorithm
        {
            KDeclareEnum(GoPartMatchAlgorithm, ::GoPartMatchAlgorithm)

            /// <summary>Edge based part match algorithm.</summary>
            literal k32s Edge = GO_PART_MATCH_ALGORITHM_EDGE;

            /// <summary>Bounding box based part match algorithm.</summary>
            literal k32s BoundingBox = GO_PART_MATCH_ALGORITHM_BOUNDING_BOX;

            /// <summary>Ellipse based part match algorithm.</summary>
            literal k32s Ellipse = GO_PART_MATCH_ALGORITHM_ELLIPSE;
        };

        /// <summary>Represents the bounding box part matching asymmetry detection type.</summary>
        public value struct GoBoxAsymmetryType
        {
            KDeclareEnum(GoBoxAsymmetryType, ::GoBoxAsymmetryType)

            /// <summary>None</summary>
            literal k32s None = GO_BOX_ASYMMETRY_TYPE_NONE;

            /// <summary>Along Length axis</summary>
            literal k32s AlongLengthAxis = GO_BOX_ASYMMETRY_TYPE_ALONG_LENGTH_AXIS;

            /// <summary>Along Width axis</summary>
            literal k32s AlongWidthAxis = GO_BOX_ASYMMETRY_TYPE_ALONG_WIDTH_AXIS;
        };

        /// <summary>Represents the bounding Ellipse part matching asymmetry detection type.</summary>
        public value struct GoEllipseAsymmetryType
        {
            KDeclareEnum(GoEllipseAsymmetryType, ::GoEllipseAsymmetryType)

            /// <summary>None</summary>
            literal k32s None = GO_ELLIPSE_ASYMMETRY_TYPE_NONE;

            /// <summary>Along Major axis</summary>
            literal k32s AlongMajorAxis = GO_ELLIPSE_ASYMMETRY_TYPE_ALONG_MAJOR_AXIS;

            /// <summary>Along Minor axis</summary>
            literal k32s AlongMinorAxis = GO_ELLIPSE_ASYMMETRY_TYPE_ALONG_MINOR_AXIS;
        };

        /// <summary>Represents an image type.</summary>
        public value struct GoImageType
        {
            KDeclareEnum(GoImageType, ::GoImageType)

            /// <summary>Heightmap image type.</summary>
            literal k32s Heightmap = GO_IMAGE_TYPE_HEIGHTMAP;

            /// <summary>Intensity image type.</summary>
            literal k32s Intensity = GO_IMAGE_TYPE_INTENSITY;
        };

        /// <summary>Represents an advanced gamma type.</summary>
        public value struct GoGammaType
        {
            KDeclareEnum(GoGammaType, ::GoGammaType)

            /// <summary>None. No imager gamma / multi-slope configuration will occur.</summary>
            literal k32s None = GO_GAMMA_TYPE_NONE;

            /// <summary>Low.</summary>
            literal k32s Low = GO_GAMMA_TYPE_LOW;

            /// <summary>Medium.</summary>
            literal k32s Medium = GO_GAMMA_TYPE_MEDIUM;

            /// <summary>High.</summary>
            literal k32s High = GO_GAMMA_TYPE_HIGH;
        };

        /// <summary>Represents a pattern sequence type.</summary>
        public value struct GoPatternSequenceType
        {
            KDeclareEnum(GoPatternSequenceType, ::GoPatternSequenceType)

            /// <summary>Default sequence pattern.</summary>
            literal k32s Default = GO_PATTERN_SEQUENCE_TYPE_DEFAULT;

            /// <summary>Custom sequence pattern.</summary>
            literal k32s Custom = GO_PATTERN_SEQUENCE_TYPE_CUSTOM;

            /// <summary>Focus pattern (G3506 only).</summary>
            literal k32s Focus = GO_PATTERN_SEQUENCE_TYPE_FOCUS;

            /// <summary>Standard sequence pattern (G3 only).</summary>
            literal k32s StandardSequence = GO_PATTERN_SEQUENCE_TYPE_STANDARD_SEQUENCE;

            /// <summary>Pattern with disabled LED light (G3 only).</summary>
            literal k32s ProjectorOff = GO_PATTERN_SEQUENCE_TYPE_PROJECTOR_OFF;
        };

        /// <summary>Represents an EthernetIP implicit messaging trigger override.</summary>
        public value struct GoImplicitTriggerOverride
        {
            KDeclareEnum(GoImplicitTriggerOverride, ::GoImplicitTriggerOverride)

            /// <summary>Use the implicit output trigger specified in the connection header.</summary>
            literal k32s Off = GO_IMPLICIT_TRIGGER_OVERRIDE_OFF;

            /// <summary>Utilize cyclic implicit messaging trigger behavior regardless of what is specified in the connection header.</summary>
            literal k32s Cyclic = GO_IMPLICIT_TRIGGER_OVERRIDE_CYCLIC;

            /// <summary>Utilize change of state implicit messaging trigger behavior regardless of what is specified in the connection header.</summary>
            literal k32s ChangeOfState = GO_IMPLICIT_TRIGGER_OVERRIDE_CHANGE_OF_STATE;
        };

        /// <summary>Represents the operation status of an alignment.</summary>
        public value struct GoAlignmentStatus
        {
            KDeclareEnum(GoAlignmentStatus, ::GoAlignmentStatus)

            /// <summary>Alignment operation succeeded.</summary>
            literal k32s OK = GO_ALIGNMENT_STATUS_OK;

            /// <summary>Alignment operation failed.</summary>
            literal k32s GeneralFailure = GO_ALIGNMENT_STATUS_GENERAL_FAILURE;

            /// <summary>Stationary alignment failed due to no data being received. Please ensure the target is in range.</summary>
            literal k32s NoData = GO_ALIGNMENT_STATUS_STATIONARY_NO_DATA;

            /// <summary>Moving alignment failed due to insufficient data.</summary>
            literal k32s InsufficientData = GO_ALIGNMENT_STATUS_MOVING_INSUFFICIENT_DATA;

            /// <summary>Invalid target detected. Examples include the target dimensions being too small, the target touches both sides of the field of view, or there is insufficient data after some internal filtering.</summary>
            literal k32s InvalidTarget = GO_ALIGNMENT_STATUS_INVALID_TARGET;

            /// <summary>Target detected in an unexpected position. Please ensure the target is stable and there are no obstructions.</summary>
            literal k32s UnexpectedTargetPosition = GO_ALIGNMENT_STATUS_UNEXPECTED_TARGET_POSITION;

            /// <summary>No reference hole was found during bar alignment. Please ensure the holes can be seen and that the target parameters match their physical dimensions.</summary>
            literal k32s BarHoleNotFound = GO_ALIGNMENT_STATUS_BAR_HOLE_NOT_FOUND;

            /// <summary>No change in encoder value occurred during moving alignment. Please ensure the encoder is connected and the target is moving.</summary>
            literal k32s MovingNoEncoderChange = GO_ALIGNMENT_STATUS_MOVING_NO_ENCODER_CHANGE;

            /// <summary>The alignment was aborted by the user.</summary>
            literal k32s Abort = GO_ALIGNMENT_STATUS_ABORT;

            /// <summary>The alignment timed out.</summary>
            literal k32s Timeout = GO_ALIGNMENT_STATUS_TIMEOUT;

            /// <summary>The alignment failed due to incorrected parameters.</summary>
            literal k32s InvalidParameter = GO_ALIGNMENT_STATUS_INVALID_PARAMETER;
        };

        /// <summary>Represents a feature data type.</summary>
        public value struct GoFeatureDataType
        {
            KDeclareEnum(GoFeatureDataType, ::GoFeatureDataType)

            /// <summary>Unknown feature.</summary>
            literal k32s Unknown = GO_FEATURE_DATA_UNKNOWN;

            /// <summary>Point feature.</summary>
            literal k32s Point = GO_FEATURE_DATA_POINT;

            /// <summary>Linear feature.</summary>
            literal k32s Line = GO_FEATURE_DATA_LINE;

            /// <summary>Circular feature.</summary>
            literal k32s Circle = GO_FEATURE_DATA_CIRCLE;

            /// <summary>Planar feature.</summary>
            literal k32s Plane = GO_FEATURE_DATA_PLANE;
        };

        [StructLayout(LayoutKind::Explicit, Size = sizeof(::GoFeatureOption))]
        public value struct GoFeatureOption
        {
            KDeclareStruct(GoFeatureOption, GoFeatureOption)

            [FieldOffset(offsetof(::GoFeatureOption, name))]
            KText64 Name;

            [FieldOffset(offsetof(::GoFeatureOption, minCount))]
            KSize MinCount;

            [FieldOffset(offsetof(::GoFeatureOption, maxCount))]
            KSize MaxCount;

            [FieldOffset(offsetof(::GoFeatureOption, dataType))]
            GoFeatureDataType DataType;

            [FieldOffset(offsetof(::GoFeatureOption, type))]
            KText64 Type;
        };

        [StructLayout(LayoutKind::Explicit, Size = sizeof(::GoMeasurementOption))]
        public value struct GoMeasurementOption
        {
            KDeclareStruct(GoMeasurementOption, GoMeasurementOption)

            [FieldOffset(offsetof(::GoMeasurementOption, name))]
            KText64 Name;

            [FieldOffset(offsetof(::GoMeasurementOption, minCount))]
            KSize MinCount;

            [FieldOffset(offsetof(::GoMeasurementOption, maxCount))]
            KSize MaxCount;
        };

        /// <summary>Represents a video message pixel type.</summary>
        public value struct GoDataType
        {
            KDeclareEnum(GoDataType, ::GoDataType)

            /// <summary>Unknown tool data type.</summary>
            literal k32s None = GO_DATA_TYPE_NONE;

            /// <summary>Range data.</summary>
            literal k32s Range = GO_DATA_TYPE_RANGE;

            /// <summary>Uniformly sampled profile.</summary>
            literal k32s ProfileUniform = GO_DATA_TYPE_UNIFORM_PROFILE;

            /// <summary>Profile Point Cloud.</summary>
            literal k32s ProfilePointCloud = GO_DATA_TYPE_PROFILE_POINT_CLOUD;

            /// <summary>Uniformly sampled surface.</summary>
            literal k32s SurfaceUniform = GO_DATA_TYPE_UNIFORM_SURFACE;

            /// <summary>Surface Point Cloud.</summary>
            literal k32s SurfacePointCloud = GO_DATA_TYPE_SURFACE_POINT_CLOUD;

            /// <summary>Unmerged profile point cloud.</summary>
            literal k32s ProfileUnmergedPointCloud = GO_DATA_TYPE_UNMERGED_PROFILE_POINT_CLOUD;

            /// <summary>Video.</summary>
            literal k32s Video = GO_DATA_TYPE_VIDEO;

            /// <summary>Tracheid.</summary>
            literal k32s Tracheid = GO_DATA_TYPE_TRACHEID;

            /// <summary>Geometric features only. No scan data. </summary>
            literal k32s Features = GO_DATA_TYPE_FEATURES_ONLY;

            /// <summary>Generic data start id value. </summary>
            literal k32s GenericBase = GO_DATA_TYPE_GENERIC_BASE;

            /// <summary>Generic data end id value. </summary>
            literal k32s GenericEnd = GO_DATA_TYPE_GENERIC_END;

            /// <summary>Mesh.</summary>
            literal k32s Mesh = GO_DATA_TYPE_MESH;

            //Deprecated:
            /// <summary>Raw profile. (DEPRECATED: Use ProfilePointCloud instead)</summary>
            literal k32s ProfileRaw = GO_DATA_TYPE_RAW_PROFILE;

            /// <summary>Raw surface. (DEPRECATED: Use SurfacePointCloud instead)</summary>
            literal k32s SurfaceRaw = GO_DATA_TYPE_RAW_SURFACE;

            /// <summary>Unmerged profile. (DEPRECATED: Use ProfileUnmergedPointCloud instead)</summary>
            literal k32s ProfileUnmergedRaw = GO_DATA_TYPE_UNMERGED_RAW_PROFILE;
        };

        [StructLayout(LayoutKind::Explicit, Size = sizeof(::GoToolDataOutputOption))]
        public value struct GoToolDataOutputOption
        {
            KDeclareStruct(GoToolDataOutputOption, GoToolDataOutputOption)

            [FieldOffset(offsetof(::GoToolDataOutputOption, name))]
            KText64 Name;

            [FieldOffset(offsetof(::GoToolDataOutputOption, type))]
            KText64 Type;

            [FieldOffset(offsetof(::GoToolDataOutputOption, dataType))]
            GoDataType DataType;

            [FieldOffset(offsetof(::GoToolDataOutputOption, minCount))]
            KSize MinCount;

            [FieldOffset(offsetof(::GoToolDataOutputOption, maxCount))]
            KSize MaxCount;
        };

        /// <summary>Represents the event type represented by an event message.</summary>
        public value struct GoEventType
        {
            KDeclareEnum(GoEventType, ::GoEventType)

            /// <summary>Exposure end</summary>
            literal k32s ExposureEnd = GO_EVENT_TYPE_EXPOSURE_END;
        };

        /// <summary>Represents the event type represented by an event message.</summary>
        public value struct GoOcclusionReductionAlg
        {
            KDeclareEnum(GoOcclusionReductionAlg, ::GoOcclusionReductionAlg)

            /// <summary>Basic occlusion reduction.</summary>
            literal k32s Normal = GO_OCCLUSION_REDUCTION_NORMAL;

            /// <summary>High quality occlusion reduction.</summary>
            literal k32s HighQuality = GO_OCCLUSION_REDUCTION_HIGH_QUALITY;
        };

        public value struct GoDemosaicStyle
        {
            KDeclareEnum(GoDemosaicStyle, ::GoDemosaicStyle)

            literal k32s Reduce = GO_DEMOSAIC_STYLE_REDUCE;
            literal k32s Bilinear = GO_DEMOSAIC_STYLE_BILINEAR;
            literal k32s Gradient = GO_DEMOSAIC_STYLE_GRADIENT;

        };

        public value struct GoSurfaceEncoding
        {
            KDeclareEnum(GoSurfaceEncoding, ::GoSurfaceEncoding)

            literal k32s Standard = GO_SURFACE_ENCODING_STANDARD;
            literal k32s Interreflection = GO_SURFACE_ENCODING_INTERREFLECTION;
        };

        public value struct GoSurfacePhaseFilter
        {
            KDeclareEnum(GoSurfacePhaseFilter, ::GoSurfacePhaseFilter)

            literal k32s None = GO_SURFACE_PHASE_FILTER_NONE;
            literal k32s Reflective = GO_SURFACE_PHASE_FILTER_REFLECTIVE;
            literal k32s Translucent = GO_SURFACE_PHASE_FILTER_TRANSLUCENT;
        };

        public value struct GoVoltageSetting
        {
            KDeclareEnum(GoVoltageSetting, ::GoVoltageSetting)

            literal k16u VOLTAGE_48 = GO_VOLTAGE_48;
            literal k16u VOLTAGE_24 = GO_VOLTAGE_24;
        };

        public value struct GoDiscoveryOpMode
        {
            KDeclareEnum(GoDiscoveryOpMode, ::GoDiscoveryOpMode)

            literal k32s NOT_AVAILABLE  = GO_DISCOVERY_OP_MODE_NOT_AVAILABLE;
            literal k32s STANDALONE     = GO_DISCOVERY_OP_MODE_STANDALONE;
            literal k32s VIRTUAL        = GO_DISCOVERY_OP_MODE_VIRTUAL;
            literal k32s ACCELERATOR    = GO_DISCOVERY_OP_MODE_ACCELERATOR;
        };

        /// <summary>Represents the supported Gocator extensible parameter type.</summary>
        public value struct GoExtParamType
        {
            KDeclareEnum(GoExtParamType, ::GoExtParamType)

            /// <summary>Unidentified extensible type.</summary>
            literal k32s EXT_PARAM_TYPE_UNKNOWN     = GO_EXT_PARAM_TYPE_UNKNOWN;

            /// <summary>Integer extensible type.</summary>
            literal k32s EXT_PARAM_TYPE_INT         = GO_EXT_PARAM_TYPE_INT;

            /// <summary>Float extensible type.</summary>
            literal k32s EXT_PARAM_TYPE_FLOAT       = GO_EXT_PARAM_TYPE_FLOAT;

            /// <summary>Boolean extensible type.</summary>
            literal k32s EXT_PARAM_TYPE_BOOL        = GO_EXT_PARAM_TYPE_BOOL;

            /// <summary>String extensible type.</summary>
            literal k32s EXT_PARAM_TYPE_STRING      = GO_EXT_PARAM_TYPE_STRING;

            /// <summary>Profile region extensible type.</summary>
            literal k32s EXT_PARAM_TYPE_REGION      = GO_EXT_PARAM_TYPE_PROFILE_REGION;

            /// <summary>3D surface region extensible type.</summary>
            literal k32s EXT_PARAM_TYPE_REGION_3D   = GO_EXT_PARAM_TYPE_SURFACE_REGION_3D;

            /// <summary>2D surface region extensible type.</summary>
            literal k32s EXT_PARAM_TYPE_REGION_2D   = GO_EXT_PARAM_TYPE_SURFACE_REGION_2D;

            /// <summary>Geometric feature extensible type.</summary>
            literal k32s EXT_PARAM_TYPE_FEATURE     = GO_EXT_PARAM_TYPE_GEOMETRIC_FEATURE;

            /// <summary>Measurement extensible type.</summary>
            literal k32s EXT_PARAM_TYPE_MEASUREMENT = GO_EXT_PARAM_TYPE_MEASUREMENT;

            /// <summary>Data input extensible type.</summary>
            literal k32s EXT_PARAM_TYPE_DATA_INPUT  = GO_EXT_PARAM_TYPE_DATA_INPUT;

            /// <summary>Point set region extensible type.</summary>
            literal k32s EXT_PARAM_TYPE_POINT_SET_REGION   = GO_EXT_PARAM_TYPE_POINT_SET_REGION;
        };

        ///<summary>Lists all sensor acceleration states that a sensor can be in.
        /// When a sensor is being accelerated, GoSensorAccelStatus
        /// provides more detail on the status of the acceleration.
        /// These are applicable only when using the GoAcceleratorMgr class.</summary>
        public value struct GoSensorAccelState
        {
            KDeclareEnum(GoSensorAccelState, ::GoSensorAccelState)

            /// <summary>State could not be determined.</summary>
            literal k32s STATE_UNKNOWN = GO_SENSOR_ACCEL_STATE_UNKNOWN;

            /// <summary>Sensor is a candidate for acceleration.</summary>
            literal k32s STATE_AVAILABLE = GO_SENSOR_ACCEL_STATE_AVAILABLE;

            /// <summary>Sensor is accelerated by this host.</summary>
            literal k32s STATE_ACCELERATED = GO_SENSOR_ACCEL_STATE_ACCELERATED;

            /// <summary>Sensor is accelerated by another host.</summary>
            literal k32s STATE_ACCELERATED_BY_OTHER = GO_SENSOR_ACCEL_STATE_ACCELERATED_BY_OTHER;

            /// <summary>Sensor firmware does not match accelerator program version.</summary>
            literal k32s STATE_FW_MISMATCH = GO_SENSOR_ACCEL_STATE_FW_MISMATCH;
        };

        public value struct GoSecurityLevel
        {
            KDeclareEnum(GoSecurityLevel, ::GoSecurityLevel)

            ///<summary> No security, any user type can access system.</summary>
            literal k32u SECURITY_NONE = GO_SECURITY_NONE;

            ///<summary> Basic security level, only authorized user types can access system.</summary>
            literal k32u SECURITY_BASIC = GO_SECURITY_BASIC;
        };
        
        public value struct GoBrandingType
        {
            KDeclareEnum(GoBrandingType, ::GoBrandingType)

            ///<summary>LMI brand displayed.</summary>
            literal k32s LMI            = GO_BRANDING_TYPE_LMI;

            ///<summary>White-label; no brand visible.</summary>
            literal k32s Unbranded      = GO_BRANDING_TYPE_UNBRANDED;

            ///<summary>Custom branding applied.</summary>
            literal k32s Custom         = GO_BRANDING_TYPE_CUSTOM;
        };

        [StructLayout(LayoutKind::Explicit, Size = sizeof(::GoPolygonCornerParameters))]
        public value struct GoPolygonCornerParameters
        {
            KDeclareValue(GoPolygonCornerParameters, GoPolygonCornerParameters)

        internal:

            [EditorBrowsable(EditorBrowsableState::Never)]
            GoPolygonCornerParameters(const ::GoPolygonCornerParameters* native)
            {
                pin_ptr<GoPolygonCornerParameters> addr = this;

                kItemCopy(addr, native, sizeof(GoPolygonCornerParameters));
            }

            [EditorBrowsable(EditorBrowsableState::Never)]
            ::GoPolygonCornerParameters ToNative()
            {
                KAdjustRef(this->deviceIdxs, kTRUE, Nullable<KRefStyle>());
                pin_ptr<GoPolygonCornerParameters> addr = this;

                return kPointer_ReadAs(addr, ::GoPolygonCornerParameters);
            }

            [EditorBrowsable(EditorBrowsableState::Never)]
            static explicit operator GoPolygonCornerParameters(const ::GoPolygonCornerParameters& native)
            {
                return kPointer_ReadAs(&native, GoPolygonCornerParameters);
            }

            [EditorBrowsable(EditorBrowsableState::Never)]
            static explicit operator const ::GoPolygonCornerParameters(GoPolygonCornerParameters clr)
            {
                KAdjustRef(clr.deviceIdxs, kTRUE, Nullable<KRefStyle>());
                return kPointer_ReadAs(&clr, ::GoPolygonCornerParameters);
            }

        public:

            [FieldOffset(offsetof(::GoPolygonCornerParameters, point))]
            KPoint64f Point;

        private:
            [FieldOffset(offsetof(::GoPolygonCornerParameters, deviceIdxs))]
            kArrayList deviceIdxs;

        public:

            property KArrayList^ DeviceIdxs // of type kSize
            {
                KArrayList^ get()
                {
                    KAdjustRef(deviceIdxs, kTRUE, Nullable<KRefStyle>());

                    return KToObject<KArrayList^>(deviceIdxs);
                }
                void set(KArrayList^ value) {
                    deviceIdxs = KToHandle(value);
                }
            }
        };

        /// <summary>Represents the mesh channel id. First 6 channel IDs are reserved for system channel usage</summary>
        public value struct GoMeshMsgChannelId
        {
            KDeclareEnum(GoMeshMsgChannelId, ::GoMeshMsgChannelId)

            /// <summary>System vertex channel.</summary>
            literal k32s SystemVertex = GO_MESH_MSG_CHANNEL_ID_SYSTEM_VERTEX;

            /// <summary>System facet channel.</summary>
            literal k32s SystemFacet = GO_MESH_MSG_CHANNEL_ID_SYSTEM_FACET;

            /// <summary>System facet normal channel.</summary>
            literal k32s SystemFacetNormal = GO_MESH_MSG_CHANNEL_ID_SYSTEM_FACET_NORMAL;

            /// <summary>System vertex normal channel.</summary>
            literal k32s SystemVertexNormal = GO_MESH_MSG_CHANNEL_ID_SYSTEM_VERTEX_NORMAL;

            /// <summary>System vertex texture channel.</summary>
            literal k32s SystemVertexTexture = GO_MESH_MSG_CHANNEL_ID_SYSTEM_VERTEX_TEXTURE;

            /// <summary>System vertex curvature channel.</summary>
            literal k32s SystemVertexCurvature = GO_MESH_MSG_CHANNEL_ID_SYSTEM_VERTEX_CURVATURE;
        };   

        /// <summary>Represents the mesh channel type.</summary>
        public value struct GoMeshMsgChannelType
        {
            KDeclareEnum(GoMeshMsgChannelType, ::GoMeshMsgChannelType)

            /// <summary>Invalid channel type.</summary>
            literal k32u Invalid = GO_MESH_MSG_CHANNEL_TYPE_INVALID;

            /// <summary>Vertex channel type.</summary>
            literal k32u Vertex = GO_MESH_MSG_CHANNEL_TYPE_VERTEX;

            /// <summary>Facet channel type.</summary>
            literal k32u Facet = GO_MESH_MSG_CHANNEL_TYPE_FACET;

            /// <summary>Facet Normal channel type.</summary>
            literal k32u FacetNormal = GO_MESH_MSG_CHANNEL_TYPE_FACET_NORMAL;

            /// <summary>Vertex Normal channel type.</summary>
            literal k32u VertexNormal = GO_MESH_MSG_CHANNEL_TYPE_VERTEX_NORMAL;

            /// <summary>Vertex Texture channel type.</summary>
            literal k32u VertexTexture = GO_MESH_MSG_CHANNEL_TYPE_VERTEX_TEXTURE;

            /// <summary>Vertex Curvature channel.</summary>
            literal k32u VertexCurvature = GO_MESH_MSG_CHANNEL_TYPE_VERTEX_CURVATURE;
        };   

        /// <summary>Represents the mesh channel state.</summary>
        public value struct GoMeshMsgChannelState
        {
            KDeclareEnum(GoMeshMsgChannelState, ::GoMeshMsgChannelState)

            /// <summary>Channel in State.</summary>
            literal k32s Error = GO_MESH_MSG_CHANNEL_STATE_ERROR;

            /// <summary>Channel buffer is not allocated.</summary>
            literal k32s Unallocated = GO_MESH_MSG_CHANNEL_STATE_UNALLOCATED;

            /// <summary>Channel buffer is allocated but is not used.</summary>
            literal k32s Allocated = GO_MESH_MSG_CHANNEL_STATE_ALLOCATED;

            /// <summary>Channel buffer is allocated and is empty.</summary>
            literal k32s Empty = GO_MESH_MSG_CHANNEL_STATE_EMPTY;

            /// <summary>Channel buffer is allocated and is partially used.</summary>
            literal k32s Partial = GO_MESH_MSG_CHANNEL_STATE_PARTIAL;

            /// <summary>Channel buffer is allocated and is full.</summary>
            literal k32s Full = GO_MESH_MSG_CHANNEL_STATE_FULL;
        };   
    }
}

#endif

//
// GoHealth.h
//
// Copyright (C) 2016-2021 by LMI Technologies Inc.
// Licensed under the MIT License.
// Redistributed files must retain the above copyright notice.
//
#ifndef GO_SDK_NET_HEALTH_H
#define GO_SDK_NET_HEALTH_H

#include <GoSdk/Messages/GoHealth.h>
#include <GoSdkNet/GoSdkDef.h>

namespace Lmi3d
{
    namespace GoSdk
    {
        namespace Messages
        {
            /// <summary>Lists all data message types.</summary>
            public value struct GoHealthIndicatorId
            {
                KDeclareEnum(GoHealthIndicatorId, GoHealthIndicatorId)

                /// <summary>Current system encoder tick.</summary>
                literal k32s EncoderValue = GO_HEALTH_ENCODER_VALUE;

                /// <summary>Current system encoder frequency (ticks/s).</summary>
                literal k32s EncoderFrequency = GO_HEALTH_ENCODER_FREQUENCY;

                /// <summary>Laser safety status.</summary>
                literal k32s LaserSafety = GO_HEALTH_LASER_SAFETY;

                /// <summary>Firmware application version.</summary>
                literal k32s FirmwareVersion = GO_HEALTH_FIRMWARE_VERSION;

                /// <summary>FireSync version.</summary>
                literal k32s FireSyncVersion = GO_HEALTH_FIRESYNC_VERSION;

                /// <summary>Time elapsed since boot-up or reset (seconds).</summary>
                literal k32s Uptime = GO_HEALTH_UPTIME;

                /// <summary>Internal temperature (degrees Celsius).</summary>
                literal k32s Temperature = GO_HEALTH_TEMPERATURE;

                /// <summary>Internal temperature extended (supports multiple devices, degrees Celsius).</summary>
                literal k32s TemperatureExtended = GO_HEALTH_TEMPERATURE_EXTENDED;

                /// <summary>Projector temperature (degrees Celsius).</summary>
                literal k32s ProjectorTemperature = GO_HEALTH_PROJECTOR_TEMPERATURE;

                /// <summary>Laser temperature (degrees Celsius).  Available only on 3B-class devices.</summary>
                literal k32s LaserTemperature = GO_HEALTH_LASER_TEMPERATURE;

                /// <summary>Indicates whether the laser is overheating. (DEPRECATED)</summary>
                literal k32s LaserOverheat = GO_HEALTH_LASER_OVERHEAT;

                /// <summary>Indicates how long the laser has been overheating if it is overheating. (DEPRECATED)</summary>
                literal k32s LaserOverheatDuration = GO_HEALTH_LASER_OVERHEAT_DURATION;

                /// <summary>Indicates whether the sensor is overheating.</summary>
                literal k32s Overheat = GO_HEALTH_OVERHEAT;

                /// <summary>Indicates how long the sensor has been overheating if it is overheating.</summary>
                literal k32s OverheatDuration = GO_HEALTH_OVERHEAT_DURATION;

                /// <summary>CPU temperature (degrees Celsius).</summary>
                literal k32s CpuTemperature = GO_HEALTH_CPU_TEMPERATURE;

                /// <summary>Camera 0 temperature (degrees Celsius).</summary>
                literal k32s Camera0Temperature = GO_HEALTH_CAMERA_0_TEMPERATURE;

                /// <summary>Camera 1 temperature (degrees Celsius).</summary>
                literal k32s Camera1Temperature = GO_HEALTH_CAMERA_1_TEMPERATURE;

                /// <summary>Laser driver temperature (degrees Celsius).</summary>
                literal k32s LaserDriverTemperature = GO_HEALTH_LASER_DRIVER_TEMPERATURE;

                /// <summary>Amount of memory currently used (bytes).</summary>
                literal k32s MemoryUsed = GO_HEALTH_MEMORY_USED;

                /// <summary>Total amount of memory available (bytes).</summary>
                literal k32s MemoryCapacity = GO_HEALTH_MEMORY_CAPACITY;

                /// <summary>Amount of non-volatile storage used (bytes).</summary>
                literal k32s StorageUsed = GO_HEALTH_STORAGE_USED;

                /// <summary>Total amount of non-volatile storage available (bytes).</summary>
                literal k32s StorageCapacity = GO_HEALTH_STORAGE_CAPACITY;

                /// <summary>CPU usage (percentage of maximum).</summary>
                literal k32s CpuUsed = GO_HEALTH_CPU_USED;

                /// <summary>Sensor synchronization source. (1 - Master, 2 - Device/::Gocator)</summary>
                literal k32s SyncSource = GO_HEALTH_SYNC_SOURCE;

                /// <summary>Current outbound network count (bytes).</summary>
                literal k32s NetOutUsed = GO_HEALTH_NET_OUT_USED;

                /// <summary>Current outbound network throughput (bytes/second).</summary>
                literal k32s NetOutRate = GO_HEALTH_NET_OUT_RATE;

                /// <summary>Total available outbound network throughput (bytes/s).</summary>
                literal k32s NetOutCapacity = GO_HEALTH_NET_OUT_CAPACITY;

                /// <summary>The Ethernet output's current network link status.</summary>
                literal k32s NetOutLinkStatus = GO_HEALTH_NET_OUT_LINK_STATUS;

                /// <summary>Current digital input status (one bit per input).</summary>
                literal k32s DigitalInputs = GO_HEALTH_DIGITAL_INPUTS;

                /// <summary>Total number of events triggered.</summary>
                literal k32s EventCounts = GO_HEALTH_EVENT_COUNTS;

                /// <summary>Number of search states.</summary>
                literal k32s CameraSearchCount = GO_HEALTH_CAMERA_SEARCH_COUNT;

                /// <summary>Number of dropped triggers.</summary>
                literal k32s CameraTriggerDrops = GO_HEALTH_CAMERA_TRIGGER_DROPS;

                /// <summary>Status of CUDA/GPU support on the sensor (accelerated and non-accelerated) platform.</summary>
                literal k32s CudaStatus = GO_HEALTH_CUDA_STATUS;

                /// <summary>Current system state.</summary>
                literal k32s State = GO_HEALTH_STATE;

                /// <summary>Current speed (Hz).</summary>
                literal k32s Speed = GO_HEALTH_SPEED;

                /// <summary>Maximum speed (Hz).</summary>
                literal k32s MaxSpeed = GO_HEALTH_MAXSPEED;

                /// <summary>Number of found spots in the last profile</summary>
                literal k32s SpotCount = GO_HEALTH_SPOT_COUNT;

                /// <summary>Maximum number of spots that can be found</summary>
                literal k32s MaxSpotCount = GO_HEALTH_MAX_SPOT_COUNT;

                /// <summary>The number of scanned profiles or surfaces.</summary>
                literal k32s ScanCount = GO_HEALTH_SCAN_COUNT;

                /// <summary>The number of found points in the last resampled profile/surface.</summary>
                literal k32s ValidPointCount = GO_HEALTH_VALID_POINT_COUNT;

                /// <summary>Maximum number of points that can be found.</summary>
                literal k32s MaxPointCount = GO_HEALTH_MAX_POINT_COUNT;

                /// <summary>Master connection status: 0=not connected, 1=connected.</summary>
                literal k32s MasterStatus = GO_HEALTH_MASTER_STATUS;

                /// <summary>The state of the second digital input..</summary>
                literal k32s CastStartState = GO_HEALTH_CAST_START_STATE;

                /// <summary>The state of the sensor alignment..</summary>
                literal k32s AlignmentState = GO_HEALTH_ALIGNMENT_STATE;

                /// <summary>Indicates the current replay playback index.</summary>
                literal k32s PlaybackPosition = GO_HEALTH_PLAYBACK_POSITION;

                /// <summary>Indicates the number of frames present in the current replay.</summary>
                literal k32s PlaybackCount = GO_HEALTH_PLAYBACK_COUNT;

                /// <summary>The number scans with high digital output pulses.</summary>
                literal k32s DigitalOutputHighCount = GO_HEALTH_DIGITAL_OUTPUT_HIGH_COUNT;

                /// <summary>The number scans with no digital output pulse.</summary>
                literal k32s DigitalOutputLowCount = GO_HEALTH_DIGITAL_OUTPUT_LOW_COUNT;

                /// <summary>Last reported processing latency value (in uS).</summary>
                literal k32s ProcessingLatencyLast = GO_HEALTH_PROCESSING_LATENCY_LAST;

                /// <summary>Maximum reported processing latency.</summary>
                literal k32s ProcessingLatencyMax = GO_HEALTH_PROCESSING_LATENCY_MAX;

                /// <summary>Current number of processing drops.</summary>
                literal k32s ProcessingDrops = GO_HEALTH_PROCESSING_DROPS;

                /// <summary>Current number of trigger drops.</summary>
                literal k32s TriggerDrops = GO_HEALTH_TRIGGER_DROPS;

                /// <summary>Current number of output drops. Sum of all output drops.</summary>
                literal k32s OutputDrops = GO_HEALTH_OUTPUT_DROPS;

                /// <summary>Current number of analog output drops.</summary>
                literal k32s AnalogDrops = GO_HEALTH_ANALOG_DROPS;

                /// <summary>Current number of digital output drops.</summary>
                literal k32s DigitalDrops = GO_HEALTH_DIGITAL_DROPS;

                /// <summary>Current number of serial output drops.</summary>
                literal k32s SerialDrops = GO_HEALTH_SERIAL_DROPS;

                /// <summary>Trigger drops from the Controlled Triggering System.</summary>
                literal k32s ControlledTriggerDrops = GO_HEALTH_CONTROLLED_TRIGGER_DROPS;

                /// <summary>Processing time of frame on 35XX/32XX (microseconds).</summary>
                literal k32s SurfaceProcessingTime = GO_HEALTH_SURFACE_PROCESSING_TIME;

                /// <summary>Max configurable frame rate (scaled by 0.000001).</summary>
                literal k32s MaxFrameRate = GO_HEALTH_MAX_FRAME_RATE;

                /// <summary>Current number of Ethernet output drops.</summary>
                literal k32s EthernetDrops = GO_HEALTH_ETHERNET_DROPS;

                /// <summary>Current number of frames with valid range data.</summary>
                literal k32s RangeValidCount = GO_HEALTH_RANGE_VALID_COUNT;

                /// <summary>Current number of frames with invalid range data.</summary>
                literal k32s RangeInvalidCount = GO_HEALTH_RANGE_INVALID_COUNT;

                /// <summary>Number of frames with anchor invalid.</summary>
                literal k32s AnchorInvalidCount = GO_HEALTH_ANCHOR_INVALID_COUNT;

                /// <summary>Total amount of time the sensor light source (G2 laser or G3 projector) has been used.</summary>
                literal k32s LightOperationalTimeTotal = GO_HEALTH_LIGHT_OPERATIONAL_TIME_TOTAL;

                /// <summary>ID of first available log entry.</summary>
                literal k32s FirstLogId = GO_HEALTH_FIRST_LOG_ID;

                /// <summary>ID of last available log entry.</summary>
                literal k32s LastLogId = GO_HEALTH_LAST_LOG_ID;

                /// <summary>Encoder z-index pulse drops</summary>
                literal k32s EncoderZIndexPulseDrops = GO_HEALTH_ENCODER_Z_INDEX_PULSE_DROPS;

                /// <summary>Most recent time taken to execute the tool.</summary>
                literal k32s ToolRunTime = GO_HEALTH_TOOL_RUN_TIME;

                /// <summary>Total number of parts emitted.</summary>
                literal k32s PartTotalEmitted = GO_HEALTH_PART_TOTAL_EMITTED;

                /// <summary>Number of parts emitted based on length limit.</summary>
                literal k32s PartLengthLimit = GO_HEALTH_PART_LENGTH_LIMIT;

                /// <summary>Number of parts dropped.</summary>
                literal k32s PartMinAreaDrops = GO_HEALTH_PART_MIN_AREA_DROPS;

                /// <summary>Number of parts dropped due to backtracking.</summary>
                literal k32s PartBacktrackDrops = GO_HEALTH_PART_BACKTRACK_DROPS;

                /// <summary>Number of parts currently being tracked.</summary>
                literal k32s PartCurrentlyActive = GO_HEALTH_PART_CURRENTLY_ACTIVE;

                /// <summary>Length of largest active part.</summary>
                literal k32s PartLength = GO_HEALTH_PART_LENGTH;

                /// <summary>Start Y position of largest active part.</summary>
                literal k32s PartStartY = GO_HEALTH_PART_START_Y;

                /// <summary>Tracking state of largest active part.</summary>
                literal k32s PartTrackingState = GO_HEALTH_PART_TRACKING_STATE;

                /// <summary>Part detection capacity exceeded.</summary>
                literal k32s PartCapacityExceeded = GO_HEALTH_PART_CAPACITY_EXCEEDED;

                /// <summary>Center X position of largets active part.</summary>
                literal k32s PartXPosition = GO_HEALTH_PART_X_POSITION;

                /// <summary>Minimum time for tool to process a sample.</summary>
                literal k32s ToolRunTimeMin = GO_HEALTH_TOOL_RUN_TIME_MIN;

                /// <summary>Maximum time for tool to process a sample.</summary>
                literal k32s ToolRunTimeMax = GO_HEALTH_TOOL_RUN_TIME_MAX;

                /// <summary>Average time for tool to process a sample.</summary>
                literal k32s ToolRunTimeAverage = GO_HEALTH_TOOL_RUN_TIME_AVERAGE;

                /// <summary>Average percentage of total time running the tool.</summary>
                literal k32s ToolRunTimePercent = GO_HEALTH_TOOL_RUN_TIME_PERCENT;

                /// <summary>Measurement value.</summary>
                literal k32s Measurement = GO_HEALTH_MEASUREMENT;

                /// <summary>Number of pass decisions.</summary>
                literal k32s MeasurementPass = GO_HEALTH_MEASUREMENT_PASS;

                /// <summary>Number of fail decisions.</summary>
                literal k32s MeasurementFail = GO_HEALTH_MEASUREMENT_FAIL;

                /// <summary>Minimum measurement value.</summary>
                literal k32s MeasurementMin = GO_HEALTH_MEASUREMENT_MIN;

                /// <summary>Maximum measurement value.</summary>
                literal k32s MeasurementMax = GO_HEALTH_MEASUREMENT_MAX;

                /// <summary>Average measurement value.</summary>
                literal k32s MeasurementAverage = GO_HEALTH_MEASUREMENT_AVERAGE;

                /// <summary>Measurement value standard deviation.</summary>
                literal k32s MeasurementStdDev = GO_HEALTH_MEASUREMENT_STDEV;

                /// <summary>Number of invalid values.</summary>
                literal k32s MeasurementInvalidCount = GO_HEALTH_MEASUREMENT_INVALID_COUNT;

                /// <summary>Number of values which exceed the numerical limit of an output protocol's measurement value field.</summary>
                literal k32s MeasurementOverflowCount = GO_HEALTH_MEASUREMENT_OVERFLOW_COUNT;
            };


            /// <summary>Represents health indicator.</summary>
            [StructLayout(LayoutKind::Explicit, Size = sizeof(::GoIndicator))]
            public value struct GoIndicator
            {
                KDeclareStruct(GoIndicator, GoIndicator)

                /// <summary>Indicator ID (e.g. GoHealthIndicatorId::CpuUsed).</summary>
                [FieldOffset(offsetof(::GoIndicator, id))]
                GoHealthIndicatorId id;

                /// <summary>Indicator instance number.</summary>
                [FieldOffset(offsetof(::GoIndicator, instance))]
                k32u instance;

                /// <summary>Indicator value.</summary>
                [FieldOffset(offsetof(::GoIndicator, value))]
                k64s value;
            };


            /// <summary>Represents a sensor health message.</summary>
            public ref class GoHealthMsg : public KObject
            {
                KDeclareClass(GoHealthMsg, GoHealthMsg)

            public:
                /// <summary>Initializes a new instance of the GoHealthMsg class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoHealthMsg(IntPtr handle)
                    : KObject(handle)
                {}

                /// <summary>Initializes a new instance of the GoHealthMsg class.</summary>
                GoHealthMsg()
                {
                    ::GoHealthMsg handle = kNULL;

                    KCheck(::GoHealthMsg_Construct(&handle, kNULL));

                    Handle = handle;
                }

                /// <inheritdoc cref="GoHealthMsg()" />
                /// <param name="allocator">Memory allocator</param>
                GoHealthMsg(KAlloc^ allocator)
                {
                    ::GoHealthMsg handle = kNULL;

                    KCheck(::GoHealthMsg_Construct(&handle, KToHandle(allocator)));

                    Handle = handle;
                }

                /// <summary>Gets the health source.</summary>
                property GoDataSource Source
                {
                    GoDataSource get() { return (GoDataSource) ::GoHealthMsg_Source(Handle); }
                }

                /// <summary>Count of health indicators in this message.</summary>
                property k32s Count
                {
                    k32s get() { return (k32s) ::GoHealthMsg_Count(Handle); }
                }

                /// <summary>Gets the message at the specified index.</summary>
                /// <param name="index">Indicator index.</param>
                /// <returns>Indicator at given index.</returns>
                GoIndicator Get(k64s index)
                {
                    return GoIndicator(GoHealthMsg_At(Handle, (kSize)index));
                }

                /// <summary>Gets a pointer to the indicator data buffer.</summary>
                property IntPtr Data
                {
                    IntPtr get() { return IntPtr(::GoHealthMsg_At(Handle, 0)); }
                }

                /// <summary>Finds the health indicator with the matching ID. Throws KStatus.ErrorNotFound if not found.</summary>
                /// <remarks>Throws KStatus.ErrorNotFound if not found.</remarks>
                /// <param name="id">Indicator index.</param>
                /// <param name="instance">Indicator instance.</param>
                /// <returns>Indicator for the given identifier.</returns>
                GoIndicator Find(k32s id, k32u instance)
                {
                    ::GoIndicator* indicator = ::GoHealthMsg_Find(Handle, id, instance);

                    if (kIsNull(indicator))
                    {
                        throw gcnew KException(kERROR_NOT_FOUND);
                    }
                    else
                    {
                        return GoIndicator(indicator);
                    }
                }
            };
        }
    }
}

#endif

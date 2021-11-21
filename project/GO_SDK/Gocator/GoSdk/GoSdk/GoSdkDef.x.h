/**
 * @file    GoSdkDef.x.h
 *
 * @internal
 * Copyright (C) 2017-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef GO_SDK_DEF_X_H
#define GO_SDK_DEF_X_H

#include <GoSdk/GoSdkReservedPorts.h>

#define GO_SDK_PROTOCOL_VERSION    "101.11.0.0"

#define GO_ROLE_INVALID                     (-1)       // Internal role: sensor role not yet determined.

#define GO_FEATURE_UNASSIGNED_ID            (-1)
#define GO_MEASUREMENT_UNASSIGNED_ID        (-1)
#define GO_UNASSIGNED_ID                    (-1)

/*
 * Private definitions of Gocator public/private channel protocol message identifiers.
 * These match the definitions GS_COMPACT_MESSAGE_XXX in GsEthernetDef.h,
 * which the sensor code uses.
 * The SDK internal code uses the following definitions to deserialize
 * the messages from the sensor.
 * These definitions must be kept in sync with the GS_COMPACT_MESSAGE_XXX
 * definitions, and can be renumbered at any time.
 *
 * IMPORTANT:
 * In the macro definition of the values, **DO NOT** put brackets around the
 * numbers. There are other macro definitions that depend on the values not
 * having brackets around the numbers.
 *
 * IMPORTANT:
 * Note that the following defines are similar, but not guaranteed to match
 * the SDK message ids GO_DATA_MESSAGE_TYPE_XXX defined in GoSdkDef.h
 * which is the set of definitions that SDK applications must use.
*/
/*
 * *** SDK applications must NEVER use the following set of GO_COMPACT_MESSAGE_xxx definitions. ***
 * *** SDK applications can use GO_DATA_MESSAGE_TYPE_XXX in GoSdkDef.h. ***
 */
#define GO_COMPACT_MESSAGE_HEALTH                       0
#define GO_COMPACT_MESSAGE_STAMP                        1
#define GO_COMPACT_MESSAGE_VIDEO                        2
#define GO_COMPACT_MESSAGE_RANGE                        3
#define GO_COMPACT_MESSAGE_RANGE_INTENSITY              4
#define GO_COMPACT_MESSAGE_PROFILE_POINT_CLOUD          5
#define GO_COMPACT_MESSAGE_UNIFORM_PROFILE              6
#define GO_COMPACT_MESSAGE_PROFILE_INTENSITY            7
#define GO_COMPACT_MESSAGE_UNIFORM_SURFACE              8
#define GO_COMPACT_MESSAGE_SURFACE_INTENSITY            9
#define GO_COMPACT_MESSAGE_MEASUREMENT                  10
#define GO_COMPACT_MESSAGE_OPERATION_RESULT             11
#define GO_COMPACT_MESSAGE_EXPOSURE_CAL_RESULT          12
#define GO_COMPACT_MESSAGE_PARTIAL_RESAMPLED_PROFILE    13
#define GO_COMPACT_MESSAGE_VIDEO_STATS                  14
#define GO_COMPACT_MESSAGE_EDGE_POINTS                  15
#define GO_COMPACT_MESSAGE_EDGE_MATCH                   16
#define GO_COMPACT_MESSAGE_BOX_MATCH                    17
#define GO_COMPACT_MESSAGE_ELLIPSE_MATCH                18
#define GO_COMPACT_MESSAGE_PARTIAL_PROFILE              19
#define GO_COMPACT_MESSAGE_SECTION                      20
#define GO_COMPACT_MESSAGE_SECTION_INTENSITY            21
#define GO_COMPACT_MESSAGE_EVENT                        22
#define GO_COMPACT_MESSAGE_TRACHEID                     23
#define GO_COMPACT_MESSAGE_FEATURE_POINT                24
#define GO_COMPACT_MESSAGE_FEATURE_LINE                 25
#define GO_COMPACT_MESSAGE_FEATURE_PLANE                26
#define GO_COMPACT_MESSAGE_FEATURE_CIRCLE               27
#define GO_COMPACT_MESSAGE_SURFACE_POINT_CLOUD          28
#define GO_COMPACT_MESSAGE_GENERIC                      29
#define GO_COMPACT_MESSAGE_NULL                         30

#define GO_COMPACT_MESSAGE_MEASUREMENT_V2               31
#define GO_COMPACT_MESSAGE_FEATURE_POINT_V2             32
#define GO_COMPACT_MESSAGE_FEATURE_LINE_V2              33
#define GO_COMPACT_MESSAGE_FEATURE_PLANE_V2             34
#define GO_COMPACT_MESSAGE_FEATURE_CIRCLE_V2            35

#define GO_COMPACT_MESSAGE_MESH                         36

#define GO_COMPACT_MESSAGE_PROFILE                      GO_COMPACT_MESSAGE_PROFILE_POINT_CLOUD
#define GO_COMPACT_MESSAGE_RESAMPLED_PROFILE            GO_COMPACT_MESSAGE_UNIFORM_PROFILE
#define GO_COMPACT_MESSAGE_SURFACE                      GO_COMPACT_MESSAGE_UNIFORM_SURFACE

#define GO_CONTROL_PORT_CONTROL                     (GO_SDK_RESERVED_PORT_SENSOR_CONTROL)
#define GO_CONTROL_PORT_UPGRADE                     (GO_SDK_RESERVED_PORT_SENSOR_UPGRADE)
#define GO_ACCELERATOR_PORT_EVENT                   (GO_SDK_RESERVED_PORT_ACCELERATOR_EVENT)

typedef kObject GoData;

/**
 * @struct  GoAsciiConfig
 * @extends kValue
 * @ingroup GoSdk-Output
 * @brief   Represents an ASCII protocol configuration element.
 */
typedef struct GoAsciiConfig
{
    GoAsciiOperation operation;     ///< only used for Ethernet output
    k32u controlPort;               ///< only used for Ethernet output
    k32u dataPort;                  ///< only used for Ethernet output
    k32u healthPort;                ///< only used for Ethernet output
    kString customFormat;
    kBool customFormatEnabled;
    GoAsciiStandardFormatMode standardFormatMode;
    kString delimiter;
    kString terminator;
    kString invalidValue;
} GoAsciiConfig;

/**
 * @struct  GoEipConfig
 * @extends kValue
 * @ingroup GoSdk-Ethernet
 * @brief   Represents an EthernetIP protocol configuration element.
 */
typedef struct GoEipConfig
{
    kBool bufferEnabled;
    kBool implicitOutputEnabled;
    GoEndianType endianOutputType;
    GoImplicitTriggerOverride implicitTriggerOverride;
} GoEipConfig;

/**
 * @struct  GoModbusConfig
 * @extends kValue
 * @ingroup GoSdk-Ethernet
 * @brief   Represents a Modbus protocol configuration element.
 */
typedef struct GoModbusConfig
{
    kBool bufferEnabled;
} GoModbusConfig;

/**
* @struct  GoProfinetConfig
* @extends kValue
* @ingroup GoSdk-Ethernet
* @brief   Represents a Profinet protocol configuration element.
*/
typedef struct GoProfinetConfig
{
    kIpAddress ipAddress;
    kIpAddress subnetMask;
    kIpAddress gateway;
    kString    deviceName;
} GoProfinetConfig;

/**
* @struct  GoPtpConfig
* @extends kValue
* @ingroup GoSdk-Ptp
* @brief   Represents a ptp protocol configuration element.
*/
typedef struct GoPtpConfig
{
    k8u notUsed;
} GoPtpConfig;

/**
 * @struct  GoSelcomConfig
 * @extends kValue
 * @ingroup GoSdk-Serial
 * @brief   Represents a Selcom protocol configuration element.
 */
typedef struct GoSelcomConfig
{
    GoSelcomFormat format;
    kArrayList formatOptions;
    k32u rate;
    kArrayList rateOptions;
    k64f dataScaleMin;
    k64f dataScaleMax;
    k64u delay;
} GoSelcomConfig;

// Keep this list in alphabetical order to make the types easier to find.
kDeclareEnumEx(Go, GoAcceleratorConnectionStatus, kValue)
kDeclareEnumEx(Go, GoAdvancedType, kValue)
kDeclareEnumEx(Go, GoAlignmentDegreesOfFreedom, kValue)
kDeclareEnumEx(Go, GoAlignmentRef, kValue)
kDeclareEnumEx(Go, GoAlignmentState, kValue)
kDeclareEnumEx(Go, GoAlignmentStatus, kValue)
kDeclareEnumEx(Go, GoAlignmentTarget, kValue)
kDeclareEnumEx(Go, GoAlignmentType, kValue)
kDeclareEnumEx(Go, GoAnalogEvent, kValue)
kDeclareEnumEx(Go, GoAnalogTrigger, kValue)
kDeclareEnumEx(Go, GoAsciiOperation, kValue)
kDeclareEnumEx(Go, GoAsciiStandardFormatMode, kValue)
kDeclareEnumEx(Go, GoBoxAsymmetryType, kValue)
kDeclareEnumEx(Go, GoBrandingType, kValue)
kDeclareEnumEx(Go, GoBuddyState, kValue)
kDeclareEnumEx(Go, GoDataMessageType, kValue)
kDeclareEnumEx(Go, GoDataSource, kValue)
kDeclareEnumEx(Go, GoDataStep, kValue)
kDeclareEnumEx(Go, GoDataType, kValue)
kDeclareEnumEx(Go, GoDecision, kValue)
kDeclareEnumEx(Go, GoDecisionCode, kValue)
kDeclareEnumEx(Go, GoDemosaicStyle, kValue)
kDeclareEnumEx(Go, GoDeviceState, kValue)
kDeclareEnumEx(Go, GoDigitalEvent, kValue)
kDeclareEnumEx(Go, GoDigitalPass, kValue)
kDeclareEnumEx(Go, GoDigitalSignal, kValue)
kDeclareEnumEx(Go, GoDiscoveryOpMode, kValue)
kDeclareEnumEx(Go, GoEllipseAsymmetryType, kValue)
kDeclareEnumEx(Go, GoEncoderSpacingMinSource, kValue)
kDeclareEnumEx(Go, GoEncoderTriggerMode, kValue)
kDeclareEnumEx(Go, GoEndianType, kValue)
kDeclareEnumEx(Go, GoEthernetProtocol, kValue)
kDeclareEnumEx(Go, GoEventType, kValue)
kDeclareEnumEx(Go, GoExposureMode, kValue)
kDeclareEnumEx(Go, GoExtMeasurementType, kValue)
kDeclareEnumEx(Go, GoExtParamType, kValue)
kDeclareEnumEx(Go, GoFamily, kValue)
kDeclareEnumEx(Go, GoFeatureDataType, kValue)
kDeclareEnumEx(Go, GoFeatureType, kValue)
kDeclareEnumEx(Go, GoFrameRateMaxSource, kValue)
kDeclareEnumEx(Go, GoState, kValue)
kDeclareEnumEx(Go, GoImageType, kValue)
kDeclareEnumEx(Go, GoGammaType, kValue)
kDeclareEnumEx(Go, GoImplicitTriggerOverride, kValue)
kDeclareEnumEx(Go, GoInputSource, kValue)
kDeclareEnumEx(Go, GoIntensitySource, kValue)
kDeclareEnumEx(Go, GoIntensityMode, kValue)
kDeclareEnumEx(Go, GoMaterialType, kValue)
kDeclareEnumEx(Go, GoMeasurementType, kValue)
kDeclareEnumEx(Go, GoMeshMsgChannelId, kValue)
kDeclareEnumEx(Go, GoMeshMsgChannelState, kValue)
kDeclareEnumEx(Go, GoMeshMsgChannelType, kValue)
kDeclareEnumEx(Go, GoMode, kValue)
kDeclareEnumEx(Go, GoOcclusionReductionAlg, kValue)
kDeclareEnumEx(Go, GoOrientation, kValue)
kDeclareEnumEx(Go, GoOutputDelayDomain, kValue)
kDeclareEnumEx(Go, GoOutputSource, kValue)
kDeclareEnumEx(Go, GoPartFrameOfReference, kValue)
kDeclareEnumEx(Go, GoPartHeightThresholdDirection, kValue)
kDeclareEnumEx(Go, GoPartMatchAlgorithm, kValue)
kDeclareEnumEx(Go, GoPatternSequenceType, kValue)
kDeclareEnumEx(Go, GoProfileAreaType, kValue)
kDeclareEnumEx(Go, GoProfileBaseline, kValue)
kDeclareEnumEx(Go, GoProfileEdgeType, kValue)
kDeclareEnumEx(Go, GoProfileFeatureType, kValue)
kDeclareEnumEx(Go, GoProfileGapAxis, kValue)
kDeclareEnumEx(Go, GoProfileGenerationStartTrigger, kValue)
kDeclareEnumEx(Go, GoProfileGenerationType, kValue)
kDeclareEnumEx(Go, GoProfileGrooveSelectType, kValue)
kDeclareEnumEx(Go, GoProfileGrooveLocation, kValue)
kDeclareEnumEx(Go, GoProfileStripSelectType, kValue)
kDeclareEnumEx(Go, GoProfileStripLocation, kValue)
kDeclareEnumEx(Go, GoProfileGrooveShape, kValue)
kDeclareEnumEx(Go, GoProfilePanelSide, kValue)
kDeclareEnumEx(Go, GoProfileRoundCornerDirection, kValue)
kDeclareEnumEx(Go, GoProfileStripBaseType, kValue)
kDeclareEnumEx(Go, GoProfileStripEdgeType, kValue)
kDeclareEnumEx(Go, GoPixelType, kValue)
kDeclareEnumEx(Go, GoReplayCombineType, kValue)
kDeclareEnumEx(Go, GoReplayConditionType, kValue)
kDeclareEnumEx(Go, GoReplayMeasurementResult, kValue)
kDeclareEnumEx(Go, GoReplayRangeCountCase, kValue)
kDeclareEnumEx(Go, GoReplayExportSourceType, kValue)
kDeclareEnumEx(Go, GoRole, kValue)
kDeclareEnumEx(Go, GoSecurityLevel, kValue)
kDeclareEnumEx(Go, GoSeekDirection, kValue)
kDeclareEnumEx(Go, GoSelcomFormat, kValue)
kDeclareEnumEx(Go, GoSensorAccelState, kValue)
kDeclareEnumEx(Go, GoSensorAccelStatus, kValue)
kDeclareEnumEx(Go, GoSerialProtocol, kValue)
kDeclareEnumEx(Go, GoSpacingIntervalType, kValue)
kDeclareEnumEx(Go, GoSpotSelectionType, kValue)
kDeclareEnumEx(Go, GoTranslucentThreadingMode, kValue)
kDeclareEnumEx(Go, GoSurfacePhaseFilter, kValue)
kDeclareEnumEx(Go, GoSurfaceEncoding, kValue)
kDeclareEnumEx(Go, GoSurfaceGenerationStartTrigger, kValue)
kDeclareEnumEx(Go, GoSurfaceGenerationType, kValue)
kDeclareEnumEx(Go, GoSurfaceLocation, kValue)
kDeclareEnumEx(Go, GoSurfaceFeatureType, kValue)
kDeclareEnumEx(Go, GoSurfaceCountersunkHoleShape, kValue)
kDeclareEnumEx(Go, GoSurfaceOpeningType, kValue)
kDeclareEnumEx(Go, GoSurfaceRivetType, kValue)
kDeclareEnumEx(Go, GoToolType, kValue)
kDeclareEnumEx(Go, GoTrigger, kValue)
kDeclareEnumEx(Go, GoTriggerSource, kValue)
kDeclareEnumEx(Go, GoTriggerUnits, kValue)
kDeclareEnumEx(Go, GoUser, kValue)
kDeclareEnumEx(Go, GoVoltageSetting, kValue)

kDeclareValueEx(Go, Go3dTransform64f, kValue)
kDeclareValueEx(Go, GoActiveAreaConfig, kValue)
kDeclareValueEx(Go, GoAddressInfo, kValue)
kDeclareValueEx(Go, GoAsciiConfig, kValue)
kDeclareValueEx(Go, GoBuddyInfo, kValue)
kDeclareValueEx(Go, GoDataStream, kValue)
kDeclareValueEx(Go, GoDataStreamId, kValue)
kDeclareValueEx(Go, GoEipConfig, kValue)
kDeclareValueEx(Go, GoElementBool, kValue)
kDeclareValueEx(Go, GoElement32u, kValue)
kDeclareValueEx(Go, GoElement64f, kValue)
kDeclareValueEx(Go, GoFacet32u, kValue)
kDeclareValueEx(Go, GoFeatureOption, kValue)
kDeclareValueEx(Go, GoFilter, kValue)
kDeclareValueEx(Go, GoMeshMsgChannel, kValue)
kDeclareValueEx(Go, GoModbusConfig, kValue)
kDeclareValueEx(Go, GoMeasurementOption, kValue)
kDeclareValueEx(Go, GoOutputCompositeSource, kValue)
kDeclareValueEx(Go, GoPolygonCornerParameters, kValue)
kDeclareValueEx(Go, GoPortInfo, kValue)
kDeclareValueEx(Go, GoProfinetConfig, kValue)
kDeclareValueEx(Go, GoSelcomConfig, kValue)
kDeclareValueEx(Go, GoStates, kValue)
kDeclareValueEx(Go, GoToolDataOutputOption, kValue)
kDeclareValueEx(Go, GoTransformation, kValue)
kDeclareValueEx(Go, GoTransformedDataRegion, kValue)
kDeclareValueEx(Go, GoUpgradeFxArgs, kValue)

typedef struct GoTypePair
{
    kType type;
    k32s enumValue;
} GoTypePair;

//@cond Private
/**
* @struct  GoExtMeasurementType
* @extends kValue
* @ingroup GoSdk-DataChannel
* @brief   Lists all extensible tool types.
*
* The following enumerators are defined:
* - #GO_EXT_MEASUREMENT_TYPE_GENERIC
* - #GO_EXT_MEASUREMENT_TYPE_X
* - #GO_EXT_MEASUREMENT_TYPE_Y
* - #GO_EXT_MEASUREMENT_TYPE_Z
* - #GO_EXT_MEASUREMENT_TYPE_X_ANGLE
* - #GO_EXT_MEASUREMENT_TYPE_Y_ANGLE
* - #GO_EXT_MEASUREMENT_TYPE_Z_ANGLE
* -
*/
typedef k32s GoExtMeasurementType;
/** @name    GoExtMeasurementType
*@{*/
#define GO_EXT_MEASUREMENT_TYPE_GENERIC                             (0)     ///< Generic measurement type.
#define GO_EXT_MEASUREMENT_TYPE_X                                   (1)     ///< X Value measurement type.
#define GO_EXT_MEASUREMENT_TYPE_Y                                   (2)     ///< Surface tool type.
#define GO_EXT_MEASUREMENT_TYPE_Z                                   (3)     ///< Surface tool type.
#define GO_EXT_MEASUREMENT_TYPE_X_ANGLE                             (4)     ///< Surface tool type.
#define GO_EXT_MEASUREMENT_TYPE_Y_ANGLE                             (5)     ///< Surface tool type.
#define GO_EXT_MEASUREMENT_TYPE_Z_ANGLE                             (6)     ///< Surface tool type.
//@endcond

//@cond Private
/**
* @struct  GoExtParamType
* @extends kValue
* @ingroup GoSdk-DataChannel
* @brief   Lists all extensible parmaeter types.
*
* The following enumerators are defined:
* - #GO_EXT_PARAM_TYPE_UNKNOWN
* - #GO_EXT_PARAM_TYPE_INT
* - #GO_EXT_PARAM_TYPE_FLOAT
* - #GO_EXT_PARAM_TYPE_BOOL
* - #GO_EXT_PARAM_TYPE_STRING
* - #GO_EXT_PARAM_TYPE_PROFILE_REGION
* - #GO_EXT_PARAM_TYPE_SURFACE_REGION_3D
* - #GO_EXT_PARAM_TYPE_SURFACE_REGION_2D
* - #GO_EXT_PARAM_TYPE_GEOMETRIC_FEATURE
* - #GO_EXT_PARAM_TYPE_MEASUREMENT
* - #GO_EXT_PARAM_TYPE_DATA_INPUT
* - #GO_EXT_PARAM_TYPE_POINT_SET_REGION
*/
typedef k32s GoExtParamType;
/** @name    GoExtParamType
*@{*/
#define GO_EXT_PARAM_TYPE_UNKNOWN              (-1)        ///< Not yet set.
#define GO_EXT_PARAM_TYPE_INT                  (0)         ///< Integer.
#define GO_EXT_PARAM_TYPE_FLOAT                (1)         ///< Float.
#define GO_EXT_PARAM_TYPE_BOOL                 (2)         ///< Boolean.
#define GO_EXT_PARAM_TYPE_STRING               (3)         ///< String.
#define GO_EXT_PARAM_TYPE_PROFILE_REGION       (4)         ///< Profile region.
#define GO_EXT_PARAM_TYPE_SURFACE_REGION_3D    (5)         ///< Surface 3D region.
#define GO_EXT_PARAM_TYPE_SURFACE_REGION_2D    (6)         ///< Surface 2D region (X and Y).
#define GO_EXT_PARAM_TYPE_GEOMETRIC_FEATURE    (7)         ///< Geometric feature
#define GO_EXT_PARAM_TYPE_MEASUREMENT          (8)         ///< Measurement (future)
#define GO_EXT_PARAM_TYPE_DATA_INPUT           (9)         ///< General data input parameter
#define GO_EXT_PARAM_TYPE_POINT_SET_REGION     (10)        ///< Point set region
//@endcond


//@cond Private
/**
* @struct  GoContainerType
* @extends kValue
* @ingroup GoSdk-DataChannel
* @brief   Lists all container types.
*
* The following enumerators are defined:
* - #GO_CONTAINER_TYPE_NONE
* - #GO_CONTAINER_TYPE_CSV
* - #GO_CONTAINER_TYPE_ITEMS
*/
typedef k32s GoContainerType;
/** @name    GoContainerType
*@{*/
#define GO_CONTAINER_TYPE_NONE                              (0)
#define GO_CONTAINER_TYPE_CSV                               (1)
#define GO_CONTAINER_TYPE_ITEMS                             (2)
//@endcond



// Define timeout for read/write event operations. Set to 5 seconds.
#define GO_ACCELERATOR_EVENT_OP_TIMEOUT       5000000

// Define accelerator events. Used by the SDK internally and between SDK and
// the accelerator process code. Not visible to SDK client.
// Choose unlikely values to guard against interpreting garbage values as events.
typedef k16u GoAcceleratorEventType;
#define GO_ACCELERATOR_EVENT_NONE               0x1000  // Dummy event used to terminate event/callback threads.
#define GO_ACCELERATOR_EVENT_TERMINATE          0x1001  // from parent to child - stop/terminate acceleration
#define GO_ACCELERATOR_EVENT_ACCELERATING       0x1002  // from child to parent - acceleration in progress
#define GO_ACCELERATOR_EVENT_SUCCESS            0x1003  // from child to parent - acceleration completed
#define GO_ACCELERATOR_EVENT_DECELERATING       0x1004  // from child to parent - stopping acceleration
#define GO_ACCELERATOR_EVENT_STOPPED            0x1005  // from child to parent - acceleration failed
#define GO_ACCELERATOR_EVENT_DISCONNECT         0x1006  // from child to parent - lost connection with physical device
#define GO_ACCELERATOR_EVENT_DECELERATED        0x1007  // generated internally by parent - sensor is decelerated.
#define GO_ACCELERATOR_EVENT_PROCESS_STOPPED    0x1008  // generated internally by parent - sensor acceleration process stopped unexpectedly.

/**
* @struct  GoAcceleratorEventMsg
* @extends kValue
* @ingroup GoSdk
* @brief   Represents an accelerator event message.
* Fields:
*  sensorId:       Identifier of the accelerated sensor.
*  event:          Specific event regarding the accelerated sensor.
*/
typedef struct GoAcceleratorEventMsg
{
    k32u                    sensorId;
    GoAcceleratorEventType  event;
} GoAcceleratorEventMsg;



#define GO_MEASUREMENT_SURFACE_RIVET_DEFAULT_RADIUS         (1.0)

#define GO_INVALID_ROLE_INDEX                               (-1)

#define GOMEASUREMENTS_NAME_TYPE kText256

// Use this table to map tool+measurement name to measurement type, and vice versa.
typedef struct GoMeasurementsNameTypeMapping
{
    GOMEASUREMENTS_NAME_TYPE    lookupName; //ToolName + MeasurementName
    GOMEASUREMENTS_NAME_TYPE    measurementName;
    kType       measurementType;
} GoMeasurementsNameTypeMapping;

kDeclareValueEx(Go, GoMeasurementsNameTypeMapping, kValue)

#define GOFEATURES_NAME_TYPE kText256

// Use this table to map tool+measurement name to measurement type, and vice versa.
typedef struct GoFeaturesNameTypeMapping
{
    GOFEATURES_NAME_TYPE    lookupName; //ToolName + MeasurementName
    GOFEATURES_NAME_TYPE    featureName;
    kType       featureType;
} GoFeaturesNameTypeMapping;

kDeclareValueEx(Go, GoFeaturesNameTypeMapping, kValue)

GoFx(kBool) GoState_IsConnected(GoState state);
GoFx(kBool) GoState_IsResponsive(GoState state);
GoFx(kBool) GoState_IsReadable(GoState state);
GoFx(kBool) GoState_IsConfigurable(GoState state);
GoFx(kBool) GoState_IsNormal(GoState state);
GoFx(kBool) GoState_IsUpgrading(GoState state);
GoFx(kBool) GoState_ShouldRefresh(GoState state);

GoFx(kBool) GoAddressInfo_VEquals(kType type, const void* value, const void* other);
GoFx(kBool) GoPortInfo_VEquals(kType type, const void* value, const void* other);
GoFx(kBool) GoOutputCompositeSource_VEquals(kType type, const void* value, const void* other);

#endif

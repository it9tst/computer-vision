/**
 * @file    GoSdkDef.h
 * @brief   Essential SDK declarations.
 *
 * For each defined value (ie "#define foo <some value>"), it must be followed
 * on the same line with the three (3)  '/', followed by a '<' followed by a
 * comment about the line, in order
 * for Doxygen to parse the definitions properly. Otherwise you will get
 * Doxygen warnings about explicit link request to 'blah blah' could not be
 * resolved.
 *
 * @internal
 * Copyright (C) 2017 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef GO_SDK_DEF_H
#define GO_SDK_DEF_H

#include <kApi/kApiDef.h>
#include <kApi/Io/kNetwork.h>

#if defined (GO_EMIT)
#    define GoFx(TYPE)    kExportFx(TYPE)            ///< GoSdk function declaration helper.
#    define GoCx(TYPE)    kExportCx(TYPE)            ///< GoSdk dynamic function declaration helper.
#    define GoDx(TYPE)    kExportDx(TYPE)            ///< GoSdk data declaration helper.
#elif defined (GO_STATIC)
#    define GoFx(TYPE)    kInFx(TYPE)
#    define GoCx(TYPE)    kInCx(TYPE)
#    define GoDx(TYPE)    kInDx(TYPE)
#else
#    define GoFx(TYPE)    kImportFx(TYPE)
#    define GoCx(TYPE)    kImportCx(TYPE)
#    define GoDx(TYPE)    kImportDx(TYPE)
#endif

/**
 * Returns the SDK version.
 *
 * @public          @memberof GoSdk
 * @version         Introduced in firmware 4.0.10.27
 * @return          SDK version.
 */
GoFx(kVersion) GoSdk_Version();

/**
 * Returns the protocol version associated with the SDK.
 *
 * @public          @memberof GoSdk
 * @version         Introduced in firmware 4.0.10.27
 * @return          Protocol version.
 */
GoFx(kVersion) GoSdk_ProtocolVersion();

/**
 * Frees the memory associated with a given kObject sourced class handle.
 *
 * @public              @memberof GoSdk
 * @version             Introduced in firmware 4.0.10.27
 * @param   object      A kObject.
 * @return              Operation status.
 */
GoFx(kStatus) GoDestroy(kObject object);

/**
 * @struct  GoUpgradeFxArgs
 * @extends kValue
 * @ingroup GoSdk
 * @brief   Represents arguments provided to an upgrade callback function.
 */
typedef struct GoUpgradeFxArgs
{
    k64f progress;          ///< Upgrade progress (percentage).
} GoUpgradeFxArgs;

typedef kStatus (kCall* GoUpgradeFx) (kPointer receiver, kObject sender, GoUpgradeFxArgs* args);

/**
 * @struct  GoDeviceState
 * @extends kValue
 * @ingroup GoSdk
 * @brief   Represents the sensor operational state. Maps to sensor's GsDeviceState.
 *
 * The following enumerators are defined:
 * - #GO_DEVICE_STATE_CONFLICT
 * - #GO_DEVICE_STATE_READY
 * - #GO_DEVICE_STATE_RUNNING
 */
typedef k32s GoDeviceState;
/** @name    GoDeviceState
 *@{*/
#define GO_DEVICE_STATE_CONFLICT    (-1)        ///< Sensor cannot be used because it is in a conflicting state.
#define GO_DEVICE_STATE_READY       (0)         ///< Sensor is not scanning but is capable of scanning.
#define GO_DEVICE_STATE_RUNNING     (1)         ///< Sensor is scanning.
/**@}*/

/**
 * @struct  GoUser
 * @extends kValue
 * @ingroup GoSdk
 * @brief   Represents a user id.
 *
 * The following enumerators are defined:
 * - #GO_USER_NONE
 * - #GO_USER_ADMIN
 * - #GO_USER_TECH
 */
typedef k32s GoUser;
/** @name    GoUser
 *@{*/
#define GO_USER_NONE        (0)         ///< No user.
#define GO_USER_ADMIN       (1)         ///< Administrator user.
#define GO_USER_TECH        (2)         ///< Technician user.
/**@}*/

typedef k32s Buddyable;
/** @name    Buddyable
*@{*/
#define GO_NOT_BUDDYABLE          (0)         ///< Not Buddyable.
#define GO_BUDDYABLE              (1)         ///< Buddyable.
#define GO_ALREADY_BUDDIED        (-100)         ///< Already Buddied.
#define GO_INVALID_STATE          (-99)         ///< Error: Invalid State.
#define GO_VERSION_MISMATCH       (-98)         ///< Error: Version Mismatch.
#define GO_MODEL_MISMATCH         (-97)         ///< Error: Model Mismatch.
#define GO_UNREACHABLE_ADDRESS    (-96)         ///< Error: Unreachable Address.

/**@}*/


/**
 * @struct  GoState
 * @extends kValue
 * @ingroup GoSdk
 * @brief   Represents the current state of a sensor object.
 *
 * The following enumerators are defined:
 * - #GO_STATE_ONLINE
 * - #GO_STATE_OFFLINE
 * - #GO_STATE_RESETTING
 * - #GO_STATE_CONNECTED
 * - #GO_STATE_INCOMPATIBLE
 * - #GO_STATE_INCONSISTENT
 * - #GO_STATE_UNRESPONSIVE
 * - #GO_STATE_CANCELLED
 * - #GO_STATE_INCOMPLETE
 * - #GO_STATE_BUSY
 * - #GO_STATE_READY
 * - #GO_STATE_RUNNING
 */
typedef k32s GoState;
/** @name    GoState
 *@{*/
#define GO_STATE_ONLINE              (0)        ///< Sensor disconnected, but detected via discovery.
#define GO_STATE_OFFLINE             (1)        ///< Sensor disconnected and no longer detected via discovery (refresh system to eliminate sensor).
#define GO_STATE_RESETTING           (2)        ///< Sensor disconnected and currently resetting (wait for completion).
#define GO_STATE_CONNECTED           (3)        ///< Sensor connected, but state is otherwise unknown. This is an internal state that is normally not returned.
                                                ///< Seeing this state usually indicates a race condition in the user code. Please see the description of GoSystem regarding thread safety.
#define GO_STATE_INCOMPATIBLE        (4)        ///< Sensor connected, but protocol incompatible with client (upgrade required).
#define GO_STATE_INCONSISTENT        (5)        ///< Sensor connected, but remote state was changed (refresh sensor).
#define GO_STATE_UNRESPONSIVE        (6)        ///< Sensor connected, but no longer detected via health or discovery (disconnect).
#define GO_STATE_CANCELLED           (7)        ///< Sensor connected, but communication aborted via GoSensor_Cancel function (disconnect or refresh sensor).
#define GO_STATE_INCOMPLETE          (8)        ///< Sensor connected, but a required buddy sensor is not present (wait or remove buddy association).
#define GO_STATE_BUSY                (9)        ///< Sensor connected, but currently controlled by another sensor (cannot be configured directly).
#define GO_STATE_READY               (10)       ///< Sensor connected and ready to accept configuration commands.
#define GO_STATE_RUNNING             (11)       ///< Sensor connected and currently running.
#define GO_STATE_UPGRADING           (12)       ///< Sensor is currently being upgraded.
/**@}*/

typedef k32s GoBuddyState;
/** @name    GoBuddyState
*@{*/
#define GO_BUDDY_STATE_ERROR                  (0)             ///< General Error.
#define GO_BUDDY_STATE_CONNECTING             (3)             ///< Buddy is currently connecting.
#define GO_BUDDY_STATE_CONNECTABLE            (2)             ///< Sensor can be buddied to.
#define GO_BUDDY_STATE_CONNECTED              (1)             ///< Buddy is connected.
#define GO_BUDDY_STATE_ALREADY_BUDDIED        (-100)          ///< Sensor is already buddied to something else.
#define GO_BUDDY_STATE_INVALID_STATE          (-99)           ///< Buddy is in an invalid state.
#define GO_BUDDY_STATE_VERSION_MISMATCH       (-98)           ///< The sensors are not currently running the same Gocator firmware version.
#define GO_BUDDY_STATE_MODEL_MISMATCH         (-97)           ///< Sensors are not of the same model number and cannot be buddied.
#define GO_BUDDY_STATE_UNREACHABLE_ADDRESS    (-96)           ///< Sensor cannot be connected to.
#define GO_BUDDY_STATE_DEVICE_MISSING         (-95)           ///< Buddied sensor cannot be detected.
#define GO_BUDDY_STATE_ERROR_CONNECTION       (-94)           ///< Buddy connection error encountered.
#define GO_BUDDY_STATE_MAX_BUDDIES            (-93)           ///< Maximum number of buddies allowed reached.
#define GO_BUDDY_STATE_STANDALONE_NOBUDDY     (-92)           ///< StandAlone sensor cannot be buddied
#define GO_BUDDY_STATE_RESTRICTED_MISMATCH    (-91)           ///< Restricted sensor can only be buddied with matching restricted sensors.
/**@}*/


/**
 * @struct  GoRole
 * @extends kValue
 * @ingroup GoSdk
 * @brief   Represents a user role. Use GO_ROLE_MAIN or GOROLE_BUDDYIDX(buddyidx)
 *
 * The following enumerators are defined:
 * - #GO_ROLE_MAIN
 * - #GO_ROLE_BUDDY
 */
typedef k32s GoRole;
/** @name    GoRole
 *@{*/
#define GO_ROLE_MAIN                 (0)        ///< Sensor is operating as a main sensor.
#define GO_ROLE_BUDDY                (1)        ///< Sensor is operating as a buddy sensor.
#define GOROLE_BUDDYIDX(buddyidx) ((GoRole)GO_ROLE_BUDDY + buddyidx) //finds buddy by index, this expects a zero buddy index as the first index to all buddies
/**@}*/

/**
 * @struct  GoAcceleratorConnectionStatus
 * @extends kValue
 * @ingroup GoSdk
 * @brief   Represents the status of the Accelerator connection.
 *          These are applicable only when using the GoAccelerator class.
 *
 * The following enumerators are defined:
 * - #GO_ACCELERATOR_CONNECTION_STATUS_CONNECTED
 * - #GO_ACCELERATOR_CONNECTION_STATUS_DISCONNECTED
 * - #GO_ACCELERATOR_CONNECTION_STATUS_ERROR
 */
typedef k32s GoAcceleratorConnectionStatus;
/** @name    GoAcceleratorConnectionStatus
 *@{*/
#define GO_ACCELERATOR_CONNECTION_STATUS_CONNECTED      (0) ///< Accelerated sensor has connected.
#define GO_ACCELERATOR_CONNECTION_STATUS_DISCONNECTED   (1) ///< Accelerated sensor has disconnected.
#define GO_ACCELERATOR_CONNECTION_STATUS_ERROR          (2) ///< An error occurred with the accelerated sensor connection.
/**@}*/

/**
 * @struct  GoAlignmentState
 * @extends kValue
 * @ingroup GoSdk
 * @brief   Represents an alignment state.
 *
 * The following enumerators are defined:
 * - #GO_ALIGNMENT_STATE_NOT_ALIGNED
 * - #GO_ALIGNMENT_STATE_ALIGNED
 */
typedef k32s GoAlignmentState;
/** @name    GoAlignmentState
 *@{*/
#define GO_ALIGNMENT_STATE_NOT_ALIGNED       (0) ///< Sensor is not aligned.
#define GO_ALIGNMENT_STATE_ALIGNED           (1) ///< Sensor is aligned.
/**@}*/

/**
 * @struct  GoAlignmentRef
 * @extends kValue
 * @ingroup GoSdk
 * @brief   Represents an alignment reference.
 *
 * The following enumerators are defined:
 * - #GO_ALIGNMENT_REF_FIXED
 * - #GO_ALIGNMENT_REF_DYNAMIC
 */
typedef k32s GoAlignmentRef;
/** @name    GoAlignmentRef
 *@{*/
#define GO_ALIGNMENT_REF_FIXED             (0)  ///< The alignment used will be specific to the sensor.
#define GO_ALIGNMENT_REF_DYNAMIC           (1)  ///< The alignment used will be specific to the current job if saved.
/**@}*/

/**
 * @struct  GoMode
 * @extends kValue
 * @ingroup GoSdk
 * @brief   Represents a scan mode.
 *
 * The following enumerators are defined:
 * - #GO_MODE_UNKNOWN
 * - #GO_MODE_VIDEO
 * - #GO_MODE_RANGE
 * - #GO_MODE_PROFILE
 * - #GO_MODE_SURFACE
 */
typedef k32s GoMode;
/** @name    GoMode
 *@{*/
#define GO_MODE_UNKNOWN                     (-1)    ///< Unknown scan mode.
#define GO_MODE_VIDEO                       (0)     ///< Video scan mode.
#define GO_MODE_RANGE                       (1)     ///< Range scan mode.
#define GO_MODE_PROFILE                     (2)     ///< Profile scan mode.
#define GO_MODE_SURFACE                     (3)     ///< Surface scan mode.
/**@}*/


/**
 * @struct  GoTrigger
 * @extends kValue
 * @ingroup GoSdk
 * @brief   Represents a trigger.
 *
 * The following enumerators are defined:
 * - #GO_TRIGGER_TIME
 * - #GO_TRIGGER_ENCODER
 * - #GO_TRIGGER_INPUT
 * - #GO_TRIGGER_SOFTWARE
 */
typedef k32s GoTrigger;
/** @name    GoTrigger
 *@{*/
#define GO_TRIGGER_TIME             (0)     ///< The sensor will be time triggered.
#define GO_TRIGGER_ENCODER          (1)     ///< The sensor will be encoder triggered.
#define GO_TRIGGER_INPUT            (2)     ///< The sensor will be digital input triggered.
#define GO_TRIGGER_SOFTWARE         (3)     ///< The sensor will be software triggered.
/** @} */

/**
 * @struct  GoEncoderTriggerMode
 * @extends kValue
 * @ingroup GoSdk
 * @brief   Represents an encoder's triggering behavior.
 *
 * The following enumerators are defined:
 * - #GO_ENCODER_TRIGGER_MODE_TRACK_REVERSE
 * - #GO_ENCODER_TRIGGER_MODE_IGNORE_REVERSE
 * - #GO_ENCODER_TRIGGER_MODE_BIDIRECTIONAL
 */
typedef k32s GoEncoderTriggerMode;
/** @name    GoEncoderTriggerMode
 *@{*/
#define GO_ENCODER_TRIGGER_MODE_TRACK_REVERSE      (0)        ///< Do not reverse trigger. Track reverse motion to prevent repeat forward triggers.
#define GO_ENCODER_TRIGGER_MODE_IGNORE_REVERSE     (1)        ///< Do not reverse trigger. Forward trigger unconditionally.
#define GO_ENCODER_TRIGGER_MODE_BIDIRECTIONAL      (2)        ///< Forward and reverse trigger.
/** @} */

/**
 * @struct  GoFrameRateMaxSource
 * @extends kValue
 * @ingroup GoSdk
 * @brief   Represents the current maximum frame rate limiting source.
 *
 * The following enumerators are defined:
 * - #GO_FRAME_RATE_MAX_SOURCE_CAMERA
 * - #GO_FRAME_RATE_MAX_SOURCE_PART_DETECTION
 */
typedef k32s GoFrameRateMaxSource;
/** @name    GoFrameRateMaxSource
 *@{*/
#define GO_FRAME_RATE_MAX_SOURCE_CAMERA            (0)        ///< Limited by the sensor's camera configuration.
#define GO_FRAME_RATE_MAX_SOURCE_PART_DETECTION    (1)        ///< Limited by part detection logic.
/** @} */

/**
 * @struct  GoEncoderSpacingMinSource
 * @extends kValue
 * @ingroup GoSdk
 * @brief   Represents the current encoder period limiting source.
 *
 * The following enumerators are defined:
 * - #GO_ENCODER_PERIOD_MAX_SOURCE_RESOLUTION
 * - #GO_ENCODER_PERIOD_MAX_SOURCE_PART_DETECTION
 */
typedef k32s GoEncoderSpacingMinSource;
/** @name    GoEncoderSpacingMinSource
 *@{*/
#define GO_ENCODER_PERIOD_MAX_SOURCE_RESOLUTION         (0)            ///< Limited by encoder resolution.
#define GO_ENCODER_PERIOD_MAX_SOURCE_PART_DETECTION     (1)            ///< Limited by part detection logic.
/**@}*/

/**
 * @struct  GoTriggerUnits
 * @extends kValue
 * @ingroup GoSdk
 * @brief   Represents the system's primary synchronization domain
 *
 * The following enumerators are defined:
 * - #GO_TRIGGER_UNIT_TIME
 * - #GO_TRIGGER_UNIT_ENCODER
 */
typedef k32s GoTriggerUnits;
/** @name    GoTriggerUnits
 *@{*/
#define GO_TRIGGER_UNIT_TIME                      (0)           ///< Base the system on the internal clock.
#define GO_TRIGGER_UNIT_ENCODER                   (1)           ///< Base the system on the encoder.
/**@}*/

/**
 * @struct  GoExposureMode
 * @extends kValue
 * @ingroup GoSdk
 * @brief   Represents all possible exposure modes.
 *
 * The following enumerators are defined:
 * - #GO_EXPOSURE_MODE_SINGLE
 * - #GO_EXPOSURE_MODE_MULTIPLE
 * - #GO_EXPOSURE_MODE_DYNAMIC
 */
typedef k32s GoExposureMode;
/** @name    GoExposureMode
 *@{*/
#define GO_EXPOSURE_MODE_SINGLE         (0)             ///< Single exposure mode.
#define GO_EXPOSURE_MODE_MULTIPLE       (1)             ///< Multiple exposure mode.
#define GO_EXPOSURE_MODE_DYNAMIC        (2)             ///< Dynamic exposure mode.
/**@}*/

/**
 * @struct  GoOrientation
 * @extends kValue
 * @ingroup GoSdk
 * @brief   Represents a sensor orientation type.
 *
 * The following enumerators are defined:
 * - #GO_ORIENTATION_WIDE
 * - #GO_ORIENTATION_OPPOSITE
 * - #GO_ORIENTATION_REVERSE
 */
typedef k32s GoOrientation;
/** @name    GoOrientation
 *@{*/
#define GO_ORIENTATION_WIDE                        (0)   ///< Wide sensor orientation.
#define GO_ORIENTATION_OPPOSITE                    (1)   ///< Opposite sensor orientation.
#define GO_ORIENTATION_REVERSE                     (2)   ///< Reverse sensor orientation.
/**@}*/

/**
 * @struct  GoInputSource
 * @extends kValue
 * @ingroup GoSdk
 * @brief   Represents a data input source.
 *
 * The following enumerators are defined:
 * - #GO_INPUT_SOURCE_LIVE
 * - #GO_INPUT_SOURCE_RECORDING
 */
typedef k32s GoInputSource;
/** @name    GoInputSource
 *@{*/
#define GO_INPUT_SOURCE_LIVE          (0)       ///< The current data input source is from live sensor data.
#define GO_INPUT_SOURCE_RECORDING     (1)       ///< The current data source is from a replay.
/**@}*/

/**
 * @struct  GoSeekDirection
 * @extends kValue
 * @ingroup GoSdk
 * @brief   Represents a playback seek direction.
 *
 * The following enumerators are defined:
 * - #GO_SEEK_DIRECTION_FORWARD
 * - #GO_SEEK_DIRECTION_BACKWARD
 */
typedef k32s GoSeekDirection;
/** @name    GoSeekDirection
 *@{*/
#define GO_SEEK_DIRECTION_FORWARD      (0)      ///< Seek forward in the current replay.
#define GO_SEEK_DIRECTION_BACKWARD     (1)      ///< Seek backward in the current replay.
/**@}*/

/**
 * @struct  GoDataSource
 * @extends kValue
 * @ingroup GoSdk
 * @brief   Represents a data source.
 *
 * The following enumerators are defined:
 *  #GO_DATA_SOURCE_NONE
 *  #GO_DATA_SOURCE_TOP
 *  #GO_DATA_SOURCE_BOTTOM
 *  #GO_DATA_SOURCE_TOP_LEFT
 *  #GO_DATA_SOURCE_TOP_RIGHT
 *  #GO_DATA_SOURCE_TOP_BOTTOM
 *  #GO_DATA_SOURCE_LEFT_RIGHT
 *  #GO_DATA_SOURCE_GRID_BASE
 */
typedef k32s GoDataSource;
/** @name    GoDataSource
 *@{*/
#define GO_DATA_SOURCE_NONE         (-1)    ///< Used to represent a buddy device when the buddy is not connected
#define GO_DATA_SOURCE_TOP          (0)     ///< Represents main device when in a single sensor or opposite orientation buddy setup. Also represents the combined main and buddy in a wide or reverse orientation
#define GO_DATA_SOURCE_BOTTOM       (1)     ///< Represents the buddy device in an opposite orientation buddy configuration
#define GO_DATA_SOURCE_TOP_LEFT     (2)     ///< Represents the main device in a wide or reverse orientation buddy configuration
#define GO_DATA_SOURCE_TOP_RIGHT    (3)     ///< Represents the buddy device in a wide or reverse orientation buddy configuration
#define GO_DATA_SOURCE_TOP_BOTTOM   (4)     ///< Represents both the main and buddy devices in a opposite orientation
#define GO_DATA_SOURCE_LEFT_RIGHT   (5)     ///< Represents a buddy configuration where data from the two devices are not merged (e.g. buddied 1000 series sensors in a wide layout)
#define GO_DATA_SOURCE_GRID_BASE    (100)   ///< Used to represent a device in a buddy scenario by adding the device's index to this value to retrieve its data.
 /**@}*/

/**
 * @struct  GoSpacingIntervalType
 * @extends kValue
 * @ingroup GoSdk
 * @brief   Represents spacing interval types.
 *
 * The following enumerators are defined:
 * - #GO_SPACING_INTERVAL_TYPE_MAX_RES
 * - #GO_SPACING_INTERVAL_TYPE_BALANCED
 * - #GO_SPACING_INTERVAL_TYPE_MAX_SPEED
 */
typedef k32s GoSpacingIntervalType;
/** @name    GoSpacingIntervalType
 *@{*/
#define GO_SPACING_INTERVAL_TYPE_MAX_RES              (0)   ///< Maximum resolution spacing interval type.
#define GO_SPACING_INTERVAL_TYPE_BALANCED             (1)   ///< Balanced spacing interval type.
#define GO_SPACING_INTERVAL_TYPE_MAX_SPEED            (2)   ///< Maximum speed spacing interval type.
#define GO_SPACING_INTERVAL_TYPE_CUSTOM               (3)   ///< The user specified custom interval.
/**@}*/

/**
 * @struct  GoTriggerSource
 * @extends kValue
 * @ingroup GoSdk
 * @brief   Represents a trigger source type.
 *
 * The following enumerators are defined:
 * - #GO_TRIGGER_SOURCE_TIME
 * - #GO_TRIGGER_SOURCE_ENCODER
 * - #GO_TRIGGER_SOURCE_INPUT
 * - #GO_TRIGGER_SOURCE_SOFTWARE
 */
typedef k32s GoTriggerSource;
/** @name    GoTriggerSource
 *@{*/
#define GO_TRIGGER_SOURCE_TIME                     (0)    ///< Trigger on internal clock.
#define GO_TRIGGER_SOURCE_ENCODER                  (1)    ///< Trigger on encoder.
#define GO_TRIGGER_SOURCE_INPUT                    (2)    ///< Trigger on digital input.
#define GO_TRIGGER_SOURCE_SOFTWARE                 (3)    ///< Trigger on software messages.
/**@}*/

/**
 * @struct  GoAlignmentType
 * @extends kValue
 * @ingroup GoSdk
 * @brief   Represents an alignment type.
 *
 * The following enumerators are defined:
 * - #GO_ALIGNMENT_TYPE_STATIONARY
 * - #GO_ALIGNMENT_TYPE_MOVING
 */
typedef k32s GoAlignmentType;
/** @name    GoAlignmentType
 *@{*/
#define GO_ALIGNMENT_TYPE_STATIONARY                 (0)    ///< Stationary target alignment type.
#define GO_ALIGNMENT_TYPE_MOVING                     (1)    ///< Moving target alignment type.
/**@}*/

/**
 * @struct  GoAlignmentTarget
 * @extends kValue
 * @ingroup GoSdk
 * @brief   Represents an alignment target type.
 *
 * The following enumerators are defined:
 * - #GO_ALIGNMENT_TARGET_NONE
 * - #GO_ALIGNMENT_TARGET_DISK
 * - #GO_ALIGNMENT_TARGET_BAR
 * - #GO_ALIGNMENT_TARGET_PLATE
 * - #GO_ALIGNMENT_TARGET_POLYGON
 */

typedef k32s GoAlignmentTarget;
/** @name    GoAlignmentTarget
 *@{*/
#define GO_ALIGNMENT_TARGET_NONE                 (0)                    ///< No calibration target.
#define GO_ALIGNMENT_TARGET_DISK                 (1)                    ///< Calibration disk.
#define GO_ALIGNMENT_TARGET_BAR                  (2)                    ///< Calibration bar.
#define GO_ALIGNMENT_TARGET_PLATE                (3)                    ///< Calibration plate.
#define GO_ALIGNMENT_TARGET_POLYGON              (5)                    ///< Calibration polygon.
/**@}*/


/**
 * @struct  GoAlignmentDegreesOfFreedom
 * @extends kValue
 * @ingroup GoSdk
 * @brief   Represents an alignment degree of freedom setting.
 *
 * The following enumerators are defined:
 * - #GO_ALIGNMENT_DOF_NONE
 * - #GO_ALIGNMENT_3DOF_XZ_Y
 * - #GO_ALIGNMENT_4DOF_XYZ_Y
 * - #GO_ALIGNMENT_5DOF_XYZ_YZ
 * - #GO_ALIGNMENT_6DOF_XYZ_XYZ
 */
typedef k32s GoAlignmentDegreesOfFreedom;
/** @name    GoAlignmentDegreesOfFreedom
 * Enumerations are 6-bit bit masks representing (left to right) axis xyz followed by angles xyz
 *@{*/
#define GO_ALIGNMENT_DOF_NONE                   (0x00)                    ///< No degrees of freedom selected.
#define GO_ALIGNMENT_3DOF_XZ_Y                  (0x2A)                    ///< 3 degrees of freedom: x,z angle y.
#define GO_ALIGNMENT_4DOF_XYZ_Y                 (0x3A)                    ///< 4 degrees of freedom: x,y,z angle y.
#define GO_ALIGNMENT_5DOF_XYZ_YZ                (0x3B)                    ///< 5 degrees of freedom: x,y,z angles y,z.
#define GO_ALIGNMENT_6DOF_XYZ_XYZ               (0x3F)                    ///< 6 degrees of freedom: x,y,z angles x,y,z.
/**@}*/

/**
 * @struct  GoPolygonCornerParameters
 * @extends kValue
 * @ingroup GoSdk
 * @brief   Corner parameters for polygon corner alignment.
 */
typedef struct GoPolygonCornerParameters
{
    kPoint64f point;
    kArrayList deviceIdxs; // of type kSize
} GoPolygonCornerParameters;

/**
 * @struct  GoReplayExportSourceType
 * @extends kValue
 * @ingroup GoSdk
 * @brief   Represents the replay export source type.
 *
 * The following enumerators are defined:
 * - #GO_REPLAY_EXPORT_SOURCE_PRIMARY
 * - #GO_REPLAY_EXPORT_SOURCE_INTENSITY
 */
typedef k32s GoReplayExportSourceType;
/** @name    GoReplayExportSourceType
 *@{*/
#define GO_REPLAY_EXPORT_SOURCE_PRIMARY                     (0) ///< Primary data (relevant to the current scan mode) replay export.
#define GO_REPLAY_EXPORT_SOURCE_INTENSITY                   (1) ///< Export intensity data using the scan data without regards to aspect ratio.
#define GO_REPLAY_EXPORT_SOURCE_INTENSITY_KEEP_ASPECT_RATIO (2) ///< Export intensity data, resizing data to maintain correct aspect ratio of the image.
 /**@}*/

/**
 * @struct  GoFamily
 * @extends kValue
 * @ingroup GoSdk
 * @brief   Represents the supported Gocator hardware families.
 *
 * The following enumerators are defined:
 * - #GO_FAMILY_1000
 * - #GO_FAMILY_2000
 * - #GO_FAMILY_3000
 */
typedef k32s GoFamily;
/** @name    GoFamily
 *@{*/
#define GO_FAMILY_UNKNOWN           (-1)    ///< Unidentified sensor family.
#define GO_FAMILY_1000              (0)     ///< 1x00 series sensors.
#define GO_FAMILY_2000              (1)     ///< 2x00 series sensors.
#define GO_FAMILY_3000              (2)     ///< 3x00 series sensors.
/**@}*/

/**
 * @struct  GoDecision
 * @extends kValue
 * @ingroup GoSdk
 * @brief   Represents the measurement output decision values. Bit 0 represents the decision value, while bits 1 through 7 represent the decision code, outlined by GoDecisionCode.
 * @see     GoDecisionCode
 *
 * The following enumerators are defined:
 * - #GO_DECISION_FAIL
 * - #GO_DECISION_PASS
 */
typedef k8u GoDecision;
/** @name    GoDecision
 *@{*/
#define GO_DECISION_FAIL            (0)     ///< The measurement value is either valid and falls outside the defined passing decision range or is invalid. The failure error code can be used to determine whether the value was valid.
#define GO_DECISION_PASS            (1)     ///< The measurement value is valid and it falls within the defined passing decision range.
/**@}*/


/**
 * @struct  GoDecisionCode
 * @extends kValue
 * @ingroup GoSdk
 * @brief   Represents the possible measurement decision codes.
 *
 * The following enumerators are defined:
 * - #GO_DECISION_CODE_OK
 * - #GO_DECISION_CODE_INVALID_ANCHOR
 * - #GO_DECISION_CODE_INVALID_VALUE
 */
typedef k8u GoDecisionCode;
/** @name    GoDecisionCode
 *@{*/
#define GO_DECISION_CODE_OK              (0)    ///< The measurement value is valid and it falls outside the defined passing decision range.
#define GO_DECISION_CODE_INVALID_VALUE   (1)    ///< The measurement value is invalid.
#define GO_DECISION_CODE_INVALID_ANCHOR  (2)    ///< The tool associated with the measurement is anchored is has received invalid measurement data from its anchoring source(s).
/**@}*/

/**
* @struct  GoIntensitySource
* @extends kValue
* @ingroup GoSdk
* @brief   Represents all possible sources of intensity data.
*
* The following enumerators are defined:
* - #GO_INTENSITY_SOURCE_BOTH
* - #GO_INTENSITY_SOURCE_FRONT
* - #GO_INTENSITY_SOURCE_BACK
*/
typedef k32s GoIntensitySource;
/** @name    GoIntensitySource
*@{*/
#define GO_INTENSITY_SOURCE_BOTH        (0)             ///< Intensity data based on both cameras.
#define GO_INTENSITY_SOURCE_FRONT       (1)             ///< Intensity data based on front camera.
#define GO_INTENSITY_SOURCE_BACK        (2)             ///< Intensity data based on back camera.
/**@}*/

/**
* @struct   GoIntensityMode
* @extends  kValue
* @ingroup  GoSdk
* @brief    Represents all possible intensity generation modes for multiple exposures.
*
* The following enumerators are defined:
* - #GO_INTENSITY_MODE_AUTO
* - #GO_INTENSITY_MODE_PRESERVE_ORIGINAL
*/
typedef k32s GoIntensityMode;

/** @name   GoIntensityMode
*@{*/
#define GO_INTENSITY_MODE_AUTO              (0)             ///< Automatically pick and scale the most reliable intensity data.
#define GO_INTENSITY_MODE_PRESERVE_ORIGINAL (1)             ///< Preserve the original values as much as possible.
/**@}*/


/** @name   GoSecurityLevel
*@{*/
typedef k32u GoSecurityLevel;
#define GO_SECURITY_NONE    (0)                 ///< No security, any user type can access system.
#define GO_SECURITY_BASIC   (1)                 ///< Basic security level, only authorized user types can access system.
/**@}*/

#define GO_ERROR_AUTHENTICATION     (-2001)     ///< logged in user does not have required privileges to performed specific action

/**
* @struct  GoVoltageSetting
* @extends kValue
* @note    Supported with G3
* @ingroup GoSdk
* @brief   Represents either 48V or 24V (with cable length) operation.
*          Only relevant on G3210
*
* The following enumerators are defined:
* - #GO_VOLTAGE_48
* - #GO_VOLTAGE_24
*/
typedef k16u GoVoltageSetting;
/** @name    GoVoltageSetting
*@{*/
#define GO_VOLTAGE_48                  (0)     ///< 48V (No Cable length input required)
#define GO_VOLTAGE_24                  (1)     ///< 24V (Cable Length required + projector dimming)
/**@}*/

/** 
* @struct GoBrandingType
* @extends kValue
* @ingroup GoSdk
* @brief   Represents possible branding types (for brand customization schemes).
*
* The following enumerators are defined:
* - #GO_BRANDING_TYPE_LMI
* - #GO_BRANDING_TYPE_UNBRANDED
* - #GO_BRANDING_TYPE_CUSTOM
*/
typedef k32s GoBrandingType;
#define GO_BRANDING_TYPE_LMI            (0)     ///< LMI brand displayed.
#define GO_BRANDING_TYPE_UNBRANDED      (1)     ///< White-label; no brand visible.
#define GO_BRANDING_TYPE_CUSTOM         (2)     ///< Custom branding applied.
/**@}*/

/**
 * @struct  GoStates
 * @extends kValue
 * @ingroup GoSdk
 * @brief   Sensor state, login, alignment information, recording state, playback source, uptime, playback information, and auto-start setting state.
 */
typedef struct GoStates
{
    GoDeviceState sensorState;              ///< The state of the sensor.
    GoUser loginType;                       ///< The logged in user.
    GoAlignmentRef alignmentReference;      ///< The alignment reference of the sensor.
    GoAlignmentState alignmentState;        ///< The alignment state of the sensor.
    kBool recordingEnabled;                 ///< The current state of recording on the sensor.
    k32s playbackSource;                    ///< The current playback source of the sensor.
    k32u uptimeSec;                         ///< Sensor uptime in seconds.
    k32u uptimeMicrosec;                    ///< Sensor uptime in microseconds.
    k32u playbackPos;                       ///< The playback position index.
    k32u playbackCount;                     ///< The playback count.
    kBool autoStartEnabled;                 ///< The auto-start enabled state.
    kBool isAccelerator;                    ///< The accelerated state of the sensor.
    GoVoltageSetting voltage;               ///< Power Source Voltage: 24 or 48 V
    k32u cableLength;                       ///< The length of the cable (in millimeters) from the Sensor to the Master.
    kBool quickEditEnabled;                 ///< The current state of editing.
    GoSecurityLevel security;               ///< The security level setup on the sensor: none/basic; when basic level does not allow anonymous users accessing system.
    GoBrandingType brandingType;            ///< The branding type of the sensor; (for brand customization schemes). 
} GoStates;

/**
 * @struct  GoAddressInfo
 * @extends kValue
 * @ingroup GoSdk
 * @brief   Sensor network address settings.
 */
typedef struct GoAddressInfo
{
    kBool useDhcp;              ///< Sensor uses DHCP?
    kIpAddress address;         ///< Sensor IP address.
    kIpAddress mask;            ///< Sensor subnet bit-mask.
    kIpAddress gateway;         ///< Sensor gateway address.
} GoAddressInfo;

/**
 * @struct  GoPortInfo
 * @extends kValue
 * @ingroup GoSdk
 * @brief   Ports used from a source device.
 */
typedef struct GoPortInfo
{
    k16u controlPort;           ///< Control channel port.
    k16u upgradePort;           ///< Upgrade channel port.
    k16u webPort;               ///< Web channel port.
    k16u dataPort;              ///< Data channel port.
    k16u healthPort;            ///< Health channel port.
} GoPortInfo;

/**
* @struct  GoBuddyInfo
* @extends kValue
* @ingroup GoSdk
* @brief   Buddy related status of another sensor.
*/
typedef struct GoBuddyInfo
{
    k32u id;           ///< Serial number of the device.
    GoBuddyState state;///< Buddy state of this device.
} GoBuddyInfo;

/**
 * @struct  GoElement64f
 * @extends kValue
 * @ingroup GoSdk
 * @brief   Represents a 64-bit floating point configuration element with a range and enabled state.
 */
typedef struct GoElement64f
{
    kBool enabled;              ///< Represents whether the element value is currently used. (not always applicable)
    k64f systemValue;           ///< The system value. (not always applicable)
    k64f value;                 ///< The element's double field value.
    k64f max;                   ///< The maximum allowable value that can be set for this element.
    k64f min;                   ///< The minimum allowable value that can be set for this element.
} GoElement64f;

/**
 * @struct  GoElement32u
 * @extends kValue
 * @ingroup GoSdk
 * @brief   Represents a 32-bit unsigned integer configuration element with a range and enabled state.
 */
typedef struct GoElement32u
{
    kBool enabled;              ///< Represents whether the element value is currently used.
    k32u systemValue;           ///< The system value. (not always applicable)
    k32u value;                 ///< The element's 32-bit unsigned field value.
    k32u max;                   ///< The maximum allowable value that can be set for this element.
    k32u min;                   ///< The minimum allowable value that can be set for this element.
} GoElement32u;

/**
 * @struct  GoElement32s
 * @extends kValue
 * @ingroup GoSdk
 * @brief   Represents a 32-bit signed integer configuration element with a range and enabled state.
 */
typedef struct GoElement32s
{
    kBool enabled;              ///< Represents whether the element value is currently used.
    k32s systemValue;           ///< The system value. (not always applicable)
    k32s value;                 ///< The element's 32-bit signed field value.
    k32s max;                   ///< The maximum allowable value that can be set for this element.
    k32s min;                   ///< The minimum allowable value that can be set for this element.
} GoElement32s;

/**
 * @struct  GoElementBool
 * @extends kValue
 * @ingroup GoSdk
 * @brief   Represents a boolean configuration element with an enabled state.
 */
typedef struct GoElementBool
{
    kBool enabled;              ///< Represents whether the element value is currently used.
    kBool systemValue;           ///< The system value. (not always applicable)
    kBool value;                 ///< The element's boolean field value.
} GoElementBool;

/**
 * @struct  GoFilter
 * @extends kValue
 * @ingroup GoSdk
 * @brief   Represents a filter configuration element.
 */
typedef struct GoFilter
{
    kBool used;                 ///< Represents whether the filter field is currently used.
    GoElement64f value;         ///< The filter's configuration properties
} GoFilter;

/**
 * @struct  GoActiveAreaConfig
 * @extends kValue
 * @ingroup GoSdk
 * @brief   Represents an active area configuration element.
 */
typedef struct GoActiveAreaConfig
{
    GoElement64f x;             ///< The X offset of the active area. (mm)
    GoElement64f y;             ///< The Y offset of the active area. (mm)
    GoElement64f z;             ///< The Z offset of the active area. (mm)
    GoElement64f height;        ///< The height of the active area. (mm)
    GoElement64f length;        ///< The length of the active area. (mm)
    GoElement64f width;         ///< The width of the active area. (mm)
} GoActiveAreaConfig;

/**
 * @struct  GoTransformation
 * @extends kValue
 * @ingroup GoSdk
 * @brief   Represents an alignment element.
 */
typedef struct GoTransformation
{
    k64f x;                     ///< The X offset of the transformation. (mm)
    k64f y;                     ///< The Y offset of the transformation. (mm)
    k64f z;                     ///< The Z offset of the transformation. (mm)
    k64f xAngle;                ///< The X angle of the transformation. (degrees)
    k64f yAngle;                ///< The Y angle of the transformation. (degrees)
    k64f zAngle;                ///< The Z angle of the transformation. (degrees)
} GoTransformation;

/**
 * @struct  GoTransformedDataRegion
 * @extends kValue
 * @ingroup GoSdk
 * @brief   Represents a transformed data region.
 */
typedef struct GoTransformedDataRegion
{
    k64f x;                     ///< The X offset of the transformed data region. (mm)
    k64f y;                     ///< The Y offset of the transformed data region. (mm)
    k64f z;                     ///< The Z offset of the transformed data region. (mm)
    k64f width;                 ///< The width of the transformed data region. (mm)
    k64f length;                ///< The length of the transformed data region. (mm)
    k64f height;                ///< The height of the transformed data region. (mm)
} GoTransformedDataRegion;

/**
 * @struct  GoOutputCompositeSource
 * @extends kValue
 * @ingroup GoSdk
 * @brief   Represents a composite data source.
 */
typedef struct GoOutputCompositeSource
{
    k32s id;                    ///< The ID of the underlying data source.
    GoDataSource dataSource;    ///< The data source of the composite data source.
} GoOutputCompositeSource;

/**
 * @struct  GoAsciiOperation
 * @extends kValue
 * @ingroup GoSdk-Output
 * @brief   Represents an ASCII protocol operational type.
 *
 * The following enumerators are defined:
 * - #GO_ASCII_OPERATION_ASYNCHRONOUS
 * - #GO_ASCII_OPERATION_POLLING
 */
typedef k32s GoAsciiOperation;
/** @name    GoAsciiOperation
 *@{*/
#define GO_ASCII_OPERATION_ASYNCHRONOUS                      (0)          ///< Selected measurement output will be sent upon sensor start.
#define GO_ASCII_OPERATION_POLLING                           (1)          ///< Measurement output will only be sent as requested.
/**@}*/

/**
 * @struct  GoAsciiStandardFormatMode
 * @extends kValue
 * @ingroup GoSdk-Output
 * @brief   Represents an ASCII standard format type.
 *
 * The following enumerators are defined:
 * - #GS_ASCII_FORMAT_MODE_MEAS
 * - #GS_ASCII_FORMAT_MODE_ENCODER_AND_FRAME
 */
typedef k32s GoAsciiStandardFormatMode;
/** @name    GoAsciiStandardFormatMode
 *@{*/
#define GS_ASCII_FORMAT_MODE_MEAS                   (0)          ///< Standard format will output with measurement values and decisions.
#define GS_ASCII_FORMAT_MODE_ENCODER_AND_FRAME      (1)          ///< Standard format will output with Encoder and Frame, then measurement values and decisions.
/**@}*/

/**
 * @struct  GoSelcomFormat
 * @extends kValue
 * @ingroup GoSdk-Serial
 * @brief   Represents the selcom format followed on the serial output.
 *
 * The following enumerators are defined:
 * - #GO_SELCOM_FORMAT_SLS
 * - #GO_SELCOM_FORMAT_12BIT_ST
 * - #GO_SELCOM_FORMAT_14BIT
 * - #GO_SELCOM_FORMAT_14BIT_ST
 */
typedef k32s GoSelcomFormat;
/** @name    GoSelcomFormat
 *@{*/
#define GO_SELCOM_FORMAT_SLS                       (0)                    ///< Selcom uses the SLS format
#define GO_SELCOM_FORMAT_12BIT_ST                  (1)                    ///< Selcom uses the 12-Bit Search/Track format
#define GO_SELCOM_FORMAT_14BIT                     (2)                    ///< Selcom uses the 14-Bit format
#define GO_SELCOM_FORMAT_14BIT_ST                  (3)                    ///< Selcom uses the 14-Bit Search/Track format
/**@}*/

/**
 * @struct  GoSerialProtocol
 * @extends kValue
 * @ingroup GoSdk-Serial
 * @brief   Represents all serial output protocols.
 *
 * The following enumerators are defined:
 * - #GO_SERIAL_PROTOCOL_GOCATOR
 * - #GO_SERIAL_PROTOCOL_SELCOM
 */
typedef k32s GoSerialProtocol;
/** @name    GoSerialProtocol
 *@{*/
#define GO_SERIAL_PROTOCOL_GOCATOR               (0)            ///< Gocator serial protocol.
#define GO_SERIAL_PROTOCOL_SELCOM                (1)            ///< Selcom serial protocol.
/**@}*/


/**
 * @struct  GoAnalogTrigger
 * @extends kValue
 * @ingroup GoSdk-Analog
 * @brief   Represents an analog output trigger.
 *
 * The following enumerators are defined:
 * - #GO_ANALOG_TRIGGER_MEASUREMENT
 * - #GO_ANALOG_TRIGGER_SOFTWARE
 */
typedef k32s GoAnalogTrigger;
/** @name    GoAnalogTrigger
 *@{*/
#define GO_ANALOG_TRIGGER_MEASUREMENT          (0)  ///< Analog output triggered by measurement data.
#define GO_ANALOG_TRIGGER_SOFTWARE             (1)  ///< Analog output triggered by software.
/**@}*/

/**
 * @struct  GoDigitalPass
 * @extends kValue
 * @ingroup GoSdk-Digital
 * @brief   Represents a digital output condition.
 *
 * The following enumerators are defined:
 * - #GO_DIGITAL_PASS_TRUE
 * - #GO_DIGITAL_PASS_FALSE
 * - #GO_DIGITAL_PASS_ALWAYS
 */
typedef k32s GoDigitalPass;
/** @name    GoDigitalPass
 *@{*/
#define GO_DIGITAL_PASS_TRUE       (0)  ///< Digital output triggers when all selected measurements pass.
#define GO_DIGITAL_PASS_FALSE      (1)  ///< Digital output triggers when all selected measurements fail.
#define GO_DIGITAL_PASS_ALWAYS     (2)  ///< Digital output triggers on every scan.
/**@}*/

/**
 * @struct  GoDigitalSignal
 * @extends kValue
 * @ingroup GoSdk-Digital
 * @brief   Represents a digital output signal type.
 *
 * The following enumerators are defined:
 * - #GO_DIGITAL_SIGNAL_PULSED
 * - #GO_DIGITAL_SIGNAL_CONTINUOUS
 */
typedef k32s GoDigitalSignal;
/** @name    GoDigitalSignal
 *@{*/
#define GO_DIGITAL_SIGNAL_PULSED               (0)  ///< Digital output is pulsed when triggered.
#define GO_DIGITAL_SIGNAL_CONTINUOUS           (1)  ///< Digital output is continuous when triggered.
/**@}*/

/**
 * @struct  GoDigitalEvent
 * @extends kValue
 * @ingroup GoSdk-Digital
 * @brief   Represents a digital output event.
 *
 * The following enumerators are defined:
 * - #GO_DIGITAL_EVENT_MEASUREMENT
 * - #GO_DIGITAL_EVENT_SOFTWARE
 * - #GO_DIGITAL_EVENT_ALIGNMENT
 * - #GO_DIGITAL_EVENT_EXPOSURE_BEGIN
 * - #GO_DIGITAL_EVENT_EXPOSURE_END
 */
typedef k32s GoDigitalEvent;
/** @name    GoDigitalEvent
 *@{*/
#define GO_DIGITAL_EVENT_MEASUREMENT            (1) ///< Digital output is triggered by measurement data.
#define GO_DIGITAL_EVENT_SOFTWARE               (2) ///< Digital output is triggered by software.
#define GO_DIGITAL_EVENT_ALIGNMENT              (3) ///< Digital output represents the alignment status.
#define GO_DIGITAL_EVENT_EXPOSURE_BEGIN         (4) ///< Digital output is triggered at the start of exposure.
#define GO_DIGITAL_EVENT_EXPOSURE_END           (5) ///< Digital output is triggered at the end of exposure, prior to processing.
/**@}*/

/**
 * @struct  GoAnalogEvent
 * @extends kValue
 * @ingroup GoSdk-Analog
 * @brief   Represents a analog output event.
 *
 * The following enumerators are defined:
 * - #GO_ANALOG_EVENT_MEASURMENT
 * - #GO_ANALOG_EVENT_SOFTWARE
 */
typedef k32s GoAnalogEvent;
/** @name    GoAnalogEvent
 *@{*/
#define GO_ANALOG_EVENT_MEASURMENT          (1) ///< Analog output is triggered by measurement data.
#define GO_ANALOG_EVENT_SOFTWARE            (2) ///< Analog output is triggered by software.
/**@}*/

/**
 * @struct  GoEthernetProtocol
 * @extends kValue
 * @ingroup GoSdk-Ethernet
 * @brief   Represents a ethernet output protocol.
 *
 * The following enumerators are defined:
 * - #GO_ETHERNET_PROTOCOL_GOCATOR
 * - #GO_ETHERNET_PROTOCOL_MODBUS
 * - #GO_ETHERNET_PROTOCOL_ETHERNET_IP
 * - #GO_ETHERNET_PROTOCOL_ASCII
 * - #GO_ETHERNET_PROTOCOL_PROFINET
 * - #GO_ETHERNET_PROTOCOL_PTP
 */
typedef k32s GoEthernetProtocol;
/** @name    GoEthernetProtocol
 *@{*/
#define GO_ETHERNET_PROTOCOL_GOCATOR        (0) ///< Gocator ethernet protocol.
#define GO_ETHERNET_PROTOCOL_MODBUS         (1) ///< Modbus ethernet protocol.
#define GO_ETHERNET_PROTOCOL_ETHERNET_IP    (2) ///< EthernetIP ethernet protocol.
#define GO_ETHERNET_PROTOCOL_ASCII          (3) ///< ASCII ethernet protocol.
#define GO_ETHERNET_PROTOCOL_PROFINET       (4) ///< Profinet ethernet protocol.
#define GO_ETHERNET_PROTOCOL_PTP            (5) ///< PTP protocol.

 /**@}*/


/**
 * @struct  GoEndianType
 * @extends kValue
 * @ingroup GoSdk-Ethernet
 * @brief   Represents an endian output type.
 *
 * The following enumerators are defined:
 * - #GO_ENDIAN_TYPE_BIG
 * - #GO_ENDIAN_TYPE_LITTLE
 */
typedef k32s GoEndianType;
/** @name    GoEndianType
 *@{*/
#define GO_ENDIAN_TYPE_BIG                  (0) ///< Big Endian output.
#define GO_ENDIAN_TYPE_LITTLE               (1) ///< Little Endian output.
/**@}*/


/**
 * @struct  GoOutputSource
 * @extends kValue
 * @ingroup GoSdk-Output
 * @brief   Represents output sources.
 *
 * The following enumerators are defined:
 * - #GO_OUTPUT_SOURCE_NONE
 * - #GO_OUTPUT_SOURCE_VIDEO
 * - #GO_OUTPUT_SOURCE_RANGE
 * - #GO_OUTPUT_SOURCE_PROFILE
 * - #GO_OUTPUT_SOURCE_SURFACE
 * - #GO_OUTPUT_SOURCE_SECTION
 * - #GO_OUTPUT_SOURCE_RANGE_INTENSITY
 * - #GO_OUTPUT_SOURCE_PROFILE_INTENSITY
 * - #GO_OUTPUT_SOURCE_SURFACE_INTENSITY
 * - #GO_OUTPUT_SOURCE_SECTION_INTENSITY
 * - #GO_OUTPUT_SOURCE_MEASUREMENT
 * - #GO_OUTPUT_SOURCE_TRACHEID
 */
typedef k32s GoOutputSource;
/** @name    GoOutputSource
 *@{*/
#define GO_OUTPUT_SOURCE_NONE                          (0)  ///< Unknown output source.
#define GO_OUTPUT_SOURCE_VIDEO                         (1)  ///< Output video data.
#define GO_OUTPUT_SOURCE_RANGE                         (2)  ///< Output range data.
#define GO_OUTPUT_SOURCE_PROFILE                       (3)  ///< Output profile data.
#define GO_OUTPUT_SOURCE_SURFACE                       (4)  ///< Output surface data.
#define GO_OUTPUT_SOURCE_RANGE_INTENSITY               (5)  ///< Output range intensity data.
#define GO_OUTPUT_SOURCE_PROFILE_INTENSITY             (6)  ///< Output profile intensity data.
#define GO_OUTPUT_SOURCE_SURFACE_INTENSITY             (7)  ///< Output surface intensity data.
#define GO_OUTPUT_SOURCE_MEASUREMENT                   (8)  ///< Output measurement data.
#define GO_OUTPUT_SOURCE_SECTION                       (9)  ///< Output section data.
#define GO_OUTPUT_SOURCE_SECTION_INTENSITY             (10) ///< Output section intensity data.
#define GO_OUTPUT_SOURCE_TRACHEID                      (11) ///< Output tracheid data.
#define GO_OUTPUT_SOURCE_EVENT                         (12) ///< Output event data.
#define GO_OUTPUT_SOURCE_FEATURE                       (13) ///< Output feature data.
#define GO_OUTPUT_SOURCE_TOOLDATA                      (14) ///< Output tool data.
 /**@}*/

/**
 * @struct  GoDataStep
 * @extends kValue
 * @ingroup GoSdk-Tools
 * @brief   Represents possible data streams.
 *
 * The following enumerators are defined:
 * - #GO_DATA_STEP_NONE
 * - #GO_DATA_STEP_VIDEO
 * - #GO_DATA_STEP_RANGE
 * - #GO_DATA_STEP_PROFILE
 * - #GO_DATA_STEP_SURFACE
 * - #GO_DATA_STEP_SECTION
 * - #GO_DATA_STEP_PROFILE_RAW
 * - #GO_DATA_STEP_SURFACE_RAW
 * - #GO_DATA_STEP_TRACHEID
 * - #GO_DATA_STEP_TOOLDATA_OUTPUTS
 * - #GO_DATA_STEP_PROFILE_UNMERGED_HDR
 * - #GO_DATA_STEP_SURFACE_ORIGINAL
 */
typedef k32s GoDataStep;
/** @name    GoDataStep
 *@{*/
#define GO_DATA_STEP_NONE                    (-1)     ///< Indicates that no specific stream has been specified.
#define GO_DATA_STEP_VIDEO                   (0)      ///< Video data stream.
#define GO_DATA_STEP_RANGE                   (1)      ///< Range data stream.
#define GO_DATA_STEP_PROFILE                 (2)      ///< Profile data stream.
#define GO_DATA_STEP_SURFACE                 (3)      ///< Surface data stream.
#define GO_DATA_STEP_SECTION                 (4)      ///< Section data stream.
#define GO_DATA_STEP_PROFILE_RAW             (5)      ///< Raw profile data stream.
#define GO_DATA_STEP_SURFACE_RAW             (6)      ///< Raw surface data stream.
#define GO_DATA_STEP_TRACHEID                (7)      ///< Tracheid data stream.
#define GO_DATA_STEP_TOOLDATA_OUTPUTS        (8)      ///< Tool Data Output data stream.
#define GO_DATA_STEP_PROFILE_UNMERGED_HDR    (9)      ///< Unmerged profile data stream.
#define GO_DATA_STEP_SURFACE_ORIGINAL        (11)     ///< Original surface data stream.

 /**@}*/


/**
 * @struct  GoDataStream
 * @extends kValue
 * @ingroup GoSdk-Output
 * @brief   Represents a data stream which consists of a data step and ID.
 */
typedef struct GoDataStream
{
    GoDataStep step;
    k32s id;
} GoDataStream;

/**
 * @struct  GoDataStreamId
 * @extends kValue
 * @ingroup GoSdk-ExtTool
 * @brief   Represents a data stream id which consists of a data step, step id and source id.
 */
typedef struct GoDataStreamId
{
    k32s step;
    k32s id;
    k32s source;
} GoDataStreamId;

/**
 * @struct  GoOutputDelayDomain
 * @extends kValue
 * @ingroup GoSdk-Output
 * @brief   Represents an output delay domain.
 *
 * The following enumerators are defined:
 * - #GO_OUTPUT_DELAY_DOMAIN_TIME
 * - #GO_OUTPUT_DELAY_DOMAIN_ENCODER
 */
typedef k32s GoOutputDelayDomain;
/** @name    GoOutputDelayDomain
 *@{*/
#define GO_OUTPUT_DELAY_DOMAIN_TIME                    (0)  ///< Time(uS) based delay domain.
#define GO_OUTPUT_DELAY_DOMAIN_ENCODER                 (1)  ///< Encoder tick delay domain.
/**@}*/

/**
 * @struct  GoPixelType
 * @extends kValue
 * @ingroup GoSdk
 * @brief   Represents a video message pixel type.
 *
 * The following enumerators are defined:
 * - #GO_PIXEL_TYPE_8U
 * - #GO_PIXEL_TYPE_RGB
 */
typedef k32s GoPixelType;
/** @name    GoPixelType
 *@{*/
#define GO_PIXEL_TYPE_UNKNOWN   (-1)
#define GO_PIXEL_TYPE_8U        (0)         ///< Each pixel is represented as unsigned 8-bit values.
#define GO_PIXEL_TYPE_RGB       (1)         ///< Each pixel is represented as three unsigned 8-bit values.
/**@}*/

/**
 * @struct  GoToolType
 * @extends kValue
 * @ingroup GoSdk-Tools
 * @brief   Lists all tool types.
 *
 * The following enumerators are defined:
 * - #GO_TOOL_UNKNOWN
 * - #GO_TOOL_RANGE_POSITION
 * - #GO_TOOL_RANGE_THICKNESS
 * - #GO_TOOL_PROFILE_AREA
 * - #GO_TOOL_PROFILE_BOUNDING_BOX
 * - #GO_TOOL_PROFILE_BRIDGE_VALUE
 * - #GO_TOOL_PROFILE_CIRCLE
 * - #GO_TOOL_PROFILE_DIMENSION
 * - #GO_TOOL_PROFILE_GROOVE
 * - #GO_TOOL_PROFILE_INTERSECT
 * - #GO_TOOL_PROFILE_LINE
 * - #GO_TOOL_PROFILE_PANEL
 * - #GO_TOOL_PROFILE_POSITION
 * - #GO_TOOL_PROFILE_STRIP
 * - #GO_TOOL_PROFILE_X_LINE
 * - #GO_TOOL_SURFACE_BOUNDING_BOX
 * - #GO_TOOL_SURFACE_COUNTERSUNK_HOLE
 * - #GO_TOOL_SURFACE_ELLIPSE
 * - #GO_TOOL_SURFACE_HOLE
 * - #GO_TOOL_SURFACE_OPENING
 * - #GO_TOOL_SURFACE_PLANE
 * - #GO_TOOL_SURFACE_POSITION
 * - #GO_TOOL_SURFACE_STUD
 * - #GO_TOOL_SURFACE_VOLUME
 * - #GO_TOOL_SCRIPT
 */
typedef k32s GoToolType;
/** @name    GoToolType
 *@{*/
#define GO_TOOL_UNKNOWN                             (-1)   ///< Unknown tool.
#define GO_TOOL_RANGE_POSITION                      (0)    ///< Range Position tool.
#define GO_TOOL_RANGE_THICKNESS                     (1)    ///< Range Thickness tool.
#define GO_TOOL_PROFILE_AREA                        (2)    ///< Profile Area tool.
#define GO_TOOL_PROFILE_BOUNDING_BOX                (21)   ///< Profile Bounding Box tool.
#define GO_TOOL_PROFILE_BRIDGE_VALUE                (24)   ///< Profile Bridge Value tool.
#define GO_TOOL_PROFILE_CIRCLE                      (3)    ///< Profile Circle tool.
#define GO_TOOL_PROFILE_DIMENSION                   (4)    ///< Profile Dimension tool.
#define GO_TOOL_PROFILE_GROOVE                      (5)    ///< Profile Groove tool.
#define GO_TOOL_PROFILE_INTERSECT                   (6)    ///< Profile Intersect tool.
#define GO_TOOL_PROFILE_LINE                        (7)    ///< Profile Line tool.
#define GO_TOOL_PROFILE_PANEL                       (8)    ///< Profile Panel tool.
#define GO_TOOL_PROFILE_POSITION                    (9)    ///< Profile Position tool.
#define GO_TOOL_PROFILE_STRIP                       (10)   ///< Profile Strip tool.
#define GO_TOOL_PROFILE_X_LINE                      (23)   ///< Profile X-Line tool.
#define GO_TOOL_SURFACE_BOUNDING_BOX                (11)   ///< Surface Bounding Box tool.
#define GO_TOOL_SURFACE_COUNTERSUNK_HOLE            (20)   ///< Surface Countersunk Hole tool.
#define GO_TOOL_SURFACE_DIMENSION                   (25)   ///< Surface Dimension tool.
#define GO_TOOL_SURFACE_ELLIPSE                     (12)   ///< Surface Ellipse tool.
#define GO_TOOL_SURFACE_HOLE                        (13)   ///< Surface Hole tool.
#define GO_TOOL_SURFACE_OPENING                     (14)   ///< Surface Opening tool.
#define GO_TOOL_SURFACE_PLANE                       (15)   ///< Surface Plane tool.
#define GO_TOOL_SURFACE_POSITION                    (16)   ///< Surface Position tool.
#define GO_TOOL_SURFACE_RIVET                       (22)   ///< Surface Rivet tool.
#define GO_TOOL_SURFACE_STUD                        (17)   ///< Surface Stud tool.
#define GO_TOOL_SURFACE_VOLUME                      (18)   ///< Surface Volume tool.
#define GO_TOOL_SCRIPT                              (19)   ///< Script tool.
#define GO_TOOL_PROFILE_ROUND_CORNER                (26)   ///< Profile Round Corner tool.

#define GO_TOOL_EXTENSIBLE                          (1000)
#define GO_TOOL_TOOL                                (1001)
/**@}*/

/**
* @struct  GoDataType
* @extends kValue
* @ingroup GoSdk-Tools
* @brief   Represents data source selections. Used as a bitmask.
*/
typedef k32s GoDataType;
/** @name    GoDataType
*@{*/
#define GO_DATA_TYPE_NONE                           (0x1)           ///< None.
#define GO_DATA_TYPE_RANGE                          (0x2)           ///< Range data.
#define GO_DATA_TYPE_UNIFORM_PROFILE                (0x3)           ///< Uniformly-spaced (resampled) profile data.
#define GO_DATA_TYPE_PROFILE_POINT_CLOUD            (0x4)           ///< Unresampled profile data.
#define GO_DATA_TYPE_UNIFORM_SURFACE                (0x5)           ///< Uniformly-spaced height map.
#define GO_DATA_TYPE_SURFACE_POINT_CLOUD            (0x6)           ///< Unresampled point cloud.
#define GO_DATA_TYPE_UNMERGED_PROFILE_POINT_CLOUD   (0x7)           ///< Unmerged raw profile data. Reserved for future use.
#define GO_DATA_TYPE_VIDEO                          (0x8)           ///< Video data.
#define GO_DATA_TYPE_TRACHEID                       (0x9)           ///< Tracheid data. Reserved for internal use.
#define GO_DATA_TYPE_MEASUREMENT                    (0xA)           ///< Measurement data.
#define GO_DATA_TYPE_MESH                           (0xB)           ///< Mesh data.

#define GO_DATA_TYPE_FEATURES_ONLY                  (0x200)         ///< Geometric features only. No scan data.
#define GO_DATA_TYPE_GENERIC_BASE                   (0x80000000)    ///< Generic data start id value.
#define GO_DATA_TYPE_GENERIC_END                    (0xFFFFFFFF)    ///< Generic data last id value.

#define GO_DATA_TYPE_RAW_PROFILE          (GO_DATA_TYPE_PROFILE_POINT_CLOUD) ///< Unresampled profile data. (Deprecated)
#define GO_DATA_TYPE_RAW_SURFACE          (GO_DATA_TYPE_SURFACE_POINT_CLOUD) ///< Unresampled point cloud. (Deprecated)
#define GO_DATA_TYPE_UNMERGED_RAW_PROFILE (GO_DATA_TYPE_UNMERGED_PROFILE_POINT_CLOUD) ///< Unmerged raw profile data. (Deprecated)
/**@}*/

/**
* @struct  GoFeatureDataType
* @extends kValue
* @ingroup GoSdk-Tools
* @brief   Lists all feature data types.
*
* The following enumerators are defined:
* - #GO_FEATURE_DATA_UNKNOWN
* - #GO_FEATURE_DATA_POINT
* - #GO_FEATURE_DATA_LINE
* - #GO_FEATURE_DATA_CIRCLE
* - #GO_FEATURE_DATA_PLANE
*/
typedef k32s GoFeatureDataType;
/** @name    GoFeatureDataType
*@{*/
#define GO_FEATURE_DATA_UNKNOWN  (-1)   ///< Unknown feature.
#define GO_FEATURE_DATA_POINT    (0)    ///< Point feature.
#define GO_FEATURE_DATA_LINE     (1)    ///< Linear feature.
#define GO_FEATURE_DATA_CIRCLE   (2)    ///< Circular feature.
#define GO_FEATURE_DATA_PLANE    (3)    ///< Planar feature.
/**@}*/

/**
* @struct  GoFeatureType
* @extends kValue
* @ingroup GoSdk-Tools
* @brief   Lists all feature types.
*
* The following enumerators are defined:
* - #GO_FEATURE_UNKNOWN
* - #GO_FEATURE_EXTENSIBLE
*/
typedef k32s GoFeatureType;
/** @name    GoFeatureType
*@{*/
#define GO_FEATURE_UNKNOWN       (-1)   ///< Unknown feature.
#define GO_FEATURE_EXTENSIBLE    (0)    ///< Extensible feature.
/**@}*/


/**
 * @struct  GoMeasurementType
 * @extends kValue
 * @ingroup GoSdk-Tools
 * @brief   Lists all measurement types.
 *
 * The following enumerators are defined:
 * - #GO_MEASUREMENT_UNKNOWN
 * - #GO_MEASUREMENT_RANGE_POSITION_Z
 * - #GO_MEASUREMENT_RANGE_THICKNESS_THICKNESS
 * - #GO_MEASUREMENT_PROFILE_AREA_AREA
 * - #GO_MEASUREMENT_PROFILE_AREA_CENTROID_X
 * - #GO_MEASUREMENT_PROFILE_AREA_CENTROID_Z
 * - #GO_MEASUREMENT_PROFILE_BOUNDING_BOX_X
 * - #GO_MEASUREMENT_PROFILE_BOUNDING_BOX_Z
 * - #GO_MEASUREMENT_PROFILE_BOUNDING_BOX_HEIGHT
 * - #GO_MEASUREMENT_PROFILE_BOUNDING_BOX_WIDTH
 * - #GO_MEASUREMENT_PROFILE_BOUNDING_BOX_GLOBAL_X
 * - #GO_MEASUREMENT_PROFILE_BRIDGE_VALUE_ANGLE
 * - #GO_MEASUREMENT_PROFILE_BRIDGE_VALUE_BRIDGE_VALUE
 * - #GO_MEASUREMENT_PROFILE_CIRCLE_X
 * - #GO_MEASUREMENT_PROFILE_CIRCLE_Z
 * - #GO_MEASUREMENT_PROFILE_CIRCLE_RADIUS
 * - #GO_MEASUREMENT_PROFILE_CIRCLE_STDDEV
 * - #GO_MEASUREMENT_PROFILE_CIRCLE_MIN_ERROR
 * - #GO_MEASUREMENT_PROFILE_CIRCLE_MIN_ERROR_X
 * - #GO_MEASUREMENT_PROFILE_CIRCLE_MIN_ERROR_Z
 * - #GO_MEASUREMENT_PROFILE_CIRCLE_MAX_ERROR
 * - #GO_MEASUREMENT_PROFILE_CIRCLE_MAX_ERROR_X
 * - #GO_MEASUREMENT_PROFILE_CIRCLE_MAX_ERROR_Z
 * - #GO_MEASUREMENT_PROFILE_DIMENSION_WIDTH
 * - #GO_MEASUREMENT_PROFILE_DIMENSION_HEIGHT
 * - #GO_MEASUREMENT_PROFILE_DIMENSION_DISTANCE
 * - #GO_MEASUREMENT_PROFILE_DIMENSION_CENTER_X
 * - #GO_MEASUREMENT_PROFILE_DIMENSION_CENTER_Z
 * - #GO_MEASUREMENT_PROFILE_GROOVE_X
 * - #GO_MEASUREMENT_PROFILE_GROOVE_Z
 * - #GO_MEASUREMENT_PROFILE_GROOVE_WIDTH
 * - #GO_MEASUREMENT_PROFILE_GROOVE_DEPTH
 * - #GO_MEASUREMENT_PROFILE_INTERSECT_X
 * - #GO_MEASUREMENT_PROFILE_INTERSECT_Z
 * - #GO_MEASUREMENT_PROFILE_INTERSECT_ANGLE
 * - #GO_MEASUREMENT_PROFILE_LINE_STDDEV
 * - #GO_MEASUREMENT_PROFILE_LINE_ERROR_MIN
 * - #GO_MEASUREMENT_PROFILE_LINE_ERROR_MAX
 * - #GO_MEASUREMENT_PROFILE_LINE_PERCENTILE
 * - #GO_MEASUREMENT_PROFILE_PANEL_GAP
 * - #GO_MEASUREMENT_PROFILE_PANEL_FLUSH
 * - #GO_MEASUREMENT_PROFILE_PANEL_LEFT_GAP_X
 * - #GO_MEASUREMENT_PROFILE_PANEL_LEFT_GAP_Z
 * - #GO_MEASUREMENT_PROFILE_PANEL_LEFT_FLUSH_X
 * - #GO_MEASUREMENT_PROFILE_PANEL_LEFT_FLUSH_Z
 * - #GO_MEASUREMENT_PROFILE_PANEL_LEFT_SURFACE_ANGLE
 * - #GO_MEASUREMENT_PROFILE_PANEL_RIGHT_GAP_X
 * - #GO_MEASUREMENT_PROFILE_PANEL_RIGHT_GAP_Z
 * - #GO_MEASUREMENT_PROFILE_PANEL_RIGHT_FLUSH_X
 * - #GO_MEASUREMENT_PROFILE_PANEL_RIGHT_FLUSH_Z
 * - #GO_MEASUREMENT_PROFILE_PANEL_RIGHT_SURFACE_ANGLE
 * - #GO_MEASUREMENT_PROFILE_POSITION_X
 * - #GO_MEASUREMENT_PROFILE_POSITION_Z
 * - #GO_MEASUREMENT_PROFILE_STRIP_POSITION_X
 * - #GO_MEASUREMENT_PROFILE_STRIP_POSITION_Z
 * - #GO_MEASUREMENT_PROFILE_STRIP_WIDTH
 * - #GO_MEASUREMENT_PROFILE_STRIP_HEIGHT
 * - #GO_MEASUREMENT_PROFILE_X_LINE_Z
 * - #GO_MEASUREMENT_PROFILE_X_LINE_VALIDITY
 * - #GO_MEASUREMENT_SURFACE_BOUNDING_BOX_X
 * - #GO_MEASUREMENT_SURFACE_BOUNDING_BOX_Y
 * - #GO_MEASUREMENT_SURFACE_BOUNDING_BOX_Z
 * - #GO_MEASUREMENT_SURFACE_BOUNDING_BOX_ZANGLE
 * - #GO_MEASUREMENT_SURFACE_BOUNDING_BOX_HEIGHT
 * - #GO_MEASUREMENT_SURFACE_BOUNDING_BOX_WIDTH
 * - #GO_MEASUREMENT_SURFACE_BOUNDING_BOX_LENGTH
 * - #GO_MEASUREMENT_SURFACE_BOUNDING_BOX_GLOBAL_X
 * - #GO_MEASUREMENT_SURFACE_BOUNDING_BOX_GLOBAL_Y
 * - #GO_MEASUREMENT_SURFACE_BOUNDING_BOX_GLOBAL_Z_ANGLE
 * - #GO_MEASUREMENT_SURFACE_COUNTERSUNK_HOLE_X
 * - #GO_MEASUREMENT_SURFACE_COUNTERSUNK_HOLE_Y
 * - #GO_MEASUREMENT_SURFACE_COUNTERSUNK_HOLE_Z
 * - #GO_MEASUREMENT_SURFACE_COUNTERSUNK_HOLE_OUTER_RADIUS
 * - #GO_MEASUREMENT_SURFACE_COUNTERSUNK_HOLE_DEPTH
 * - #GO_MEASUREMENT_SURFACE_COUNTERSUNK_HOLE_BEVEL_RADIUS
 * - #GO_MEASUREMENT_SURFACE_COUNTERSUNK_HOLE_BEVEL_ANGLE
 * - #GO_MEASUREMENT_SURFACE_COUNTERSUNK_HOLE_X_ANGLE
 * - #GO_MEASUREMENT_SURFACE_COUNTERSUNK_HOLE_Y_ANGLE
 * - #GO_MEASUREMENT_SURFACE_COUNTERSUNK_HOLE_COUNTERBORE_DEPTH
 * - #GO_MEASUREMENT_SURFACE_DIMENSION_WIDTH
 * - #GO_MEASUREMENT_SURFACE_DIMENSION_HEIGHT
 * - #GO_MEASUREMENT_SURFACE_DIMENSION_LENGTH
 * - #GO_MEASUREMENT_SURFACE_DIMENSION_DISTANCE
 * - #GO_MEASUREMENT_SURFACE_DIMENSION_PLANE_DISTANCE
 * - #GO_MEASUREMENT_SURFACE_DIMENSION_CENTER_X
 * - #GO_MEASUREMENT_SURFACE_DIMENSION_CENTER_Y
 * - #GO_MEASUREMENT_SURFACE_DIMENSION_CENTER_Z
 * - #GO_MEASUREMENT_SURFACE_ELLIPSE_MAJOR
 * - #GO_MEASUREMENT_SURFACE_ELLIPSE_MINOR
 * - #GO_MEASUREMENT_SURFACE_ELLIPSE_RATIO
 * - #GO_MEASUREMENT_SURFACE_ELLIPSE_ZANGLE
 * - #GO_MEASUREMENT_SURFACE_HOLE_X
 * - #GO_MEASUREMENT_SURFACE_HOLE_Y
 * - #GO_MEASUREMENT_SURFACE_HOLE_Z
 * - #GO_MEASUREMENT_SURFACE_HOLE_RADIUS
 * - #GO_MEASUREMENT_SURFACE_OPENING_X
 * - #GO_MEASUREMENT_SURFACE_OPENING_Y
 * - #GO_MEASUREMENT_SURFACE_OPENING_Z
 * - #GO_MEASUREMENT_SURFACE_OPENING_WIDTH
 * - #GO_MEASUREMENT_SURFACE_OPENING_LENGTH
 * - #GO_MEASUREMENT_SURFACE_OPENING_ANGLE
 * - #GO_MEASUREMENT_SURFACE_PLANE_X_ANGLE
 * - #GO_MEASUREMENT_SURFACE_PLANE_Y_ANGLE
 * - #GO_MEASUREMENT_SURFACE_PLANE_Z_OFFSET
 * - #GO_MEASUREMENT_SURFACE_PLANE_STD_DEV
 * - #GO_MEASUREMENT_SURFACE_PLANE_ERROR_MIN
 * - #GO_MEASUREMENT_SURFACE_PLANE_ERROR_MAX
 * - #GO_MEASUREMENT_SURFACE_PLANE_X_NORMAL
 * - #GO_MEASUREMENT_SURFACE_PLANE_Y_NORMAL
 * - #GO_MEASUREMENT_SURFACE_PLANE_Z_NORMAL
 * - #GO_MEASUREMENT_SURFACE_PLANE_DISTANCE
 * - #GO_MEASUREMENT_SURFACE_POSITION_X
 * - #GO_MEASUREMENT_SURFACE_POSITION_Y
 * - #GO_MEASUREMENT_SURFACE_POSITION_Z
 * - #GO_MEASUREMENT_SURFACE_RIVET_X
 * - #GO_MEASUREMENT_SURFACE_RIVET_Y
 * - #GO_MEASUREMENT_SURFACE_RIVET_Z
 * - #GO_MEASUREMENT_SURFACE_RIVET_TILT_ANGLE
 * - #GO_MEASUREMENT_SURFACE_RIVET_TILT_DIRECTION
 * - #GO_MEASUREMENT_SURFACE_RIVET_RADIUS
 * - #GO_MEASUREMENT_SURFACE_RIVET_TOP_OFFSET_MIN
 * - #GO_MEASUREMENT_SURFACE_RIVET_TOP_OFFSET_MAX
 * - #GO_MEASUREMENT_SURFACE_RIVET_TOP_OFFSET_MEAN
 * - #GO_MEASUREMENT_SURFACE_RIVET_TOP_OFFSET_STD_DEV
 * - #GO_MEASUREMENT_SURFACE_RIVET_RADIAL_HEIGHT_MIN
 * - #GO_MEASUREMENT_SURFACE_RIVET_RADIAL_HEIGHT_MAX
 * - #GO_MEASUREMENT_SURFACE_RIVET_RADIAL_HEIGHT_MEAN
 * - #GO_MEASUREMENT_SURFACE_RIVET_RADIAL_HEIGHT_STD_DEV
 * - #GO_MEASUREMENT_SURFACE_RIVET_RADIAL_SLOPE_MIN
 * - #GO_MEASUREMENT_SURFACE_RIVET_RADIAL_SLOPE_MAX
 * - #GO_MEASUREMENT_SURFACE_RIVET_RADIAL_SLOPE_MEAN
 * - #GO_MEASUREMENT_SURFACE_RIVET_RADIAL_SLOPE_STD_DEV
 * - #GO_MEASUREMENT_SURFACE_STUD_BASE_X
 * - #GO_MEASUREMENT_SURFACE_STUD_BASE_Y
 * - #GO_MEASUREMENT_SURFACE_STUD_BASE_Z
 * - #GO_MEASUREMENT_SURFACE_STUD_TIP_X
 * - #GO_MEASUREMENT_SURFACE_STUD_TIP_Y
 * - #GO_MEASUREMENT_SURFACE_STUD_TIP_Z
 * - #GO_MEASUREMENT_SURFACE_STUD_RADIUS
 * - #GO_MEASUREMENT_SURFACE_VOLUME_AREA
 * - #GO_MEASUREMENT_SURFACE_VOLUME_VOLUME
 * - #GO_MEASUREMENT_SURFACE_VOLUME_THICKNESS
 * - #GO_MEASUREMENT_SCRIPT_OUTPUT
 */
typedef k32s GoMeasurementType;
/** @name    GoMeasurementType
 *@{*/
#define GO_MEASUREMENT_UNKNOWN                                      (-1)    ///< Unknown measurement.
#define GO_MEASUREMENT_RANGE_POSITION_Z                             (0)     ///< Range Position tool Z measurement.
#define GO_MEASUREMENT_RANGE_THICKNESS_THICKNESS                    (1)     ///< Range Thickness tool Thickness measurement.
#define GO_MEASUREMENT_PROFILE_AREA_AREA                            (2)     ///< Profile Area tool Area measurement.
#define GO_MEASUREMENT_PROFILE_AREA_CENTROID_X                      (3)     ///< Profile Area tool Centroid X measurement.
#define GO_MEASUREMENT_PROFILE_AREA_CENTROID_Z                      (4)     ///< Profile Area tool Centroid Z measurement.
#define GO_MEASUREMENT_PROFILE_BOUNDING_BOX_X                       (82)    ///< Profile Bounding Box X measurement.
#define GO_MEASUREMENT_PROFILE_BOUNDING_BOX_Z                       (83)    ///< Profile Bounding Box Z measurement.
#define GO_MEASUREMENT_PROFILE_BOUNDING_BOX_HEIGHT                  (84)    ///< Profile Bounding Box Height measurement.
#define GO_MEASUREMENT_PROFILE_BOUNDING_BOX_WIDTH                   (85)    ///< Profile Bounding Box Width measurement.
#define GO_MEASUREMENT_PROFILE_BOUNDING_BOX_GLOBAL_X                (86)    ///< Profile Bounding Box Global X measurement.
#define GO_MEASUREMENT_PROFILE_BOUNDING_BOX_GLOBAL_Y                (112)   ///< Profile Bounding Box Global Y measurement.
#define GO_MEASUREMENT_PROFILE_BOUNDING_BOX_GLOBAL_ANGLE            (113)   ///< Profile Bounding Box Global Angle measurement.
#define GO_MEASUREMENT_PROFILE_BRIDGE_VALUE_BRIDGE_VALUE            (106)   ///< Profile Bridge Value measurement.
#define GO_MEASUREMENT_PROFILE_BRIDGE_VALUE_ANGLE                   (107)   ///< Profile Bridge Value measurement.
#define GO_MEASUREMENT_PROFILE_BRIDGE_VALUE_WINDOW                  (146)   ///< Profile Bridge Value measurement.
#define GO_MEASUREMENT_PROFILE_BRIDGE_VALUE_STDDEV                  (147)   ///< Profile Bridge Value measurement.
#define GO_MEASUREMENT_PROFILE_CIRCLE_X                             (5)     ///< Profile Circle tool X measurement.
#define GO_MEASUREMENT_PROFILE_CIRCLE_Z                             (6)     ///< Profile Circle tool Z measurement.
#define GO_MEASUREMENT_PROFILE_CIRCLE_RADIUS                        (7)     ///< Profile Circle tool Radius measurement.
#define GO_MEASUREMENT_PROFILE_CIRCLE_STDDEV                        (148)     ///< Profile Circle tool StdDev measurement.
#define GO_MEASUREMENT_PROFILE_CIRCLE_MIN_ERROR                     (149)     ///< Profile Circle tool Minimum Error measurement.
#define GO_MEASUREMENT_PROFILE_CIRCLE_MIN_ERROR_X                   (150)     ///< Profile Circle tool Minimum Error X measurement.
#define GO_MEASUREMENT_PROFILE_CIRCLE_MIN_ERROR_Z                   (151)     ///< Profile Circle tool Minimum Error Z measurement.
#define GO_MEASUREMENT_PROFILE_CIRCLE_MAX_ERROR                     (152)     ///< Profile Circle tool Maximum Error measurement.
#define GO_MEASUREMENT_PROFILE_CIRCLE_MAX_ERROR_X                   (153)     ///< Profile Circle tool Maximum Error X measurement.
#define GO_MEASUREMENT_PROFILE_CIRCLE_MAX_ERROR_Z                   (154)     ///< Profile Circle tool Maximum Error Z measurement.
#define GO_MEASUREMENT_PROFILE_DIMENSION_WIDTH                      (8)     ///< Profile Dimension tool Width measurement.
#define GO_MEASUREMENT_PROFILE_DIMENSION_HEIGHT                     (9)     ///< Profile Dimension tool Height measurement.
#define GO_MEASUREMENT_PROFILE_DIMENSION_DISTANCE                   (10)    ///< Profile Dimension tool Distance measurement.
#define GO_MEASUREMENT_PROFILE_DIMENSION_CENTER_X                   (11)    ///< Profile Dimension tool Center X measurement.
#define GO_MEASUREMENT_PROFILE_DIMENSION_CENTER_Z                   (12)    ///< Profile Dimension tool Center Z measurement.
#define GO_MEASUREMENT_PROFILE_GROOVE_X                             (13)    ///< Profile Groove tool X measurement.
#define GO_MEASUREMENT_PROFILE_GROOVE_Z                             (14)    ///< Profile Groove tool Z measurement.
#define GO_MEASUREMENT_PROFILE_GROOVE_WIDTH                         (15)    ///< Profile Groove tool Width measurement.
#define GO_MEASUREMENT_PROFILE_GROOVE_DEPTH                         (16)    ///< Profile Groove tool Depth measurement.
#define GO_MEASUREMENT_PROFILE_INTERSECT_X                          (17)    ///< Profile Intersect tool X measurement.
#define GO_MEASUREMENT_PROFILE_INTERSECT_Z                          (18)    ///< Profile Intersect tool Z measurement.
#define GO_MEASUREMENT_PROFILE_INTERSECT_ANGLE                      (19)    ///< Profile Intersect tool Angle measurement.
#define GO_MEASUREMENT_PROFILE_LINE_STDDEV                          (20)    ///< Profile Line tool Standard Deviation measurement.
#define GO_MEASUREMENT_PROFILE_LINE_ERROR_MIN                       (21)    ///< Profile Line tool Minimum Error measurement.
#define GO_MEASUREMENT_PROFILE_LINE_ERROR_MAX                       (22)    ///< Profile Line tool Maximum Error measurement.
#define GO_MEASUREMENT_PROFILE_LINE_PERCENTILE                      (23)    ///< Profile Line tool Percentile measurement.
#define GO_MEASUREMENT_PROFILE_LINE_OFFSET                          (130)   ///< Profile Line tool Offset measurement.
#define GO_MEASUREMENT_PROFILE_LINE_ANGLE                           (131)   ///< Profile Line tool Angle measurement.
#define GO_MEASUREMENT_PROFILE_LINE_ERROR_MIN_X                     (132)   ///< Profile Line tool Minimum X Error measurement.
#define GO_MEASUREMENT_PROFILE_LINE_ERROR_MIN_Z                     (133)   ///< Profile Line tool Minimum Z Error measurement.
#define GO_MEASUREMENT_PROFILE_LINE_ERROR_MAX_X                     (134)   ///< Profile Line tool Maximum X Error measurement.
#define GO_MEASUREMENT_PROFILE_LINE_ERROR_MAX_Z                     (135)   ///< Profile Line tool Maximum Z Error measurement.
#define GO_MEASUREMENT_PROFILE_PANEL_GAP                            (24)    ///< Profile Panel tool Gap measurement.
#define GO_MEASUREMENT_PROFILE_PANEL_FLUSH                          (25)    ///< Profile Panel tool Flush measurement.
#define GO_MEASUREMENT_PROFILE_PANEL_LEFT_GAP_X                     (136)   ///< Profile Panel tool Left Gap X measurement.
#define GO_MEASUREMENT_PROFILE_PANEL_LEFT_GAP_Z                     (137)   ///< Profile Panel tool Left Gap Z measurement.
#define GO_MEASUREMENT_PROFILE_PANEL_LEFT_FLUSH_X                   (138)   ///< Profile Panel tool Left Flush X measurement.
#define GO_MEASUREMENT_PROFILE_PANEL_LEFT_FLUSH_Z                   (139)   ///< Profile Panel tool Left Flush Z measurement.
#define GO_MEASUREMENT_PROFILE_PANEL_LEFT_SURFACE_ANGLE             (140)   ///< Profile Panel tool Left Surface Angle measurement.
#define GO_MEASUREMENT_PROFILE_PANEL_RIGHT_GAP_X                    (141)   ///< Profile Panel tool Right Gap X measurement.
#define GO_MEASUREMENT_PROFILE_PANEL_RIGHT_GAP_Z                    (142)   ///< Profile Panel tool Right Gap Z measurement.
#define GO_MEASUREMENT_PROFILE_PANEL_RIGHT_FLUSH_X                  (143)   ///< Profile Panel tool Right Flush X measurement.
#define GO_MEASUREMENT_PROFILE_PANEL_RIGHT_FLUSH_Z                  (144)   ///< Profile Panel tool Right Flush Z measurement.
#define GO_MEASUREMENT_PROFILE_PANEL_RIGHT_SURFACE_ANGLE            (145)   ///< Profile Panel tool Right Surface Angle measurement.
#define GO_MEASUREMENT_PROFILE_POSITION_X                           (26)    ///< Profile Position tool X measurement.
#define GO_MEASUREMENT_PROFILE_POSITION_Z                           (27)    ///< Profile Position tool Z measurement.
#define GO_MEASUREMENT_PROFILE_STRIP_POSITION_X                     (28)    ///< Profile Strip tool X Position measurement.
#define GO_MEASUREMENT_PROFILE_STRIP_POSITION_Z                     (29)    ///< Profile Strip tool Z Position measurement.
#define GO_MEASUREMENT_PROFILE_STRIP_WIDTH                          (30)    ///< Profile Strip tool Width measurement.
#define GO_MEASUREMENT_PROFILE_STRIP_HEIGHT                         (31)    ///< Profile Strip tool Height measurement.
#define GO_MEASUREMENT_PROFILE_X_LINE_Z                             (87)    ///< Profile X-Line tool Z measurement.
#define GO_MEASUREMENT_PROFILE_X_LINE_VALIDITY                      (88)    ///< Profile X-Line tool Validity measurement.
#define GO_MEASUREMENT_SURFACE_BOUNDING_BOX_X                       (32)    ///< Surface Bounding Box X measurement.
#define GO_MEASUREMENT_SURFACE_BOUNDING_BOX_Y                       (33)    ///< Surface Bounding Box Y measurement.
#define GO_MEASUREMENT_SURFACE_BOUNDING_BOX_Z                       (34)    ///< Surface Bounding Box Z measurement.
#define GO_MEASUREMENT_SURFACE_BOUNDING_BOX_ZANGLE                  (35)    ///< Surface Bounding Box Z Angle measurement.
#define GO_MEASUREMENT_SURFACE_BOUNDING_BOX_HEIGHT                  (36)    ///< Surface Bounding Box Height measurement.
#define GO_MEASUREMENT_SURFACE_BOUNDING_BOX_WIDTH                   (37)    ///< Surface Bounding Box Width measurement.
#define GO_MEASUREMENT_SURFACE_BOUNDING_BOX_LENGTH                  (38)    ///< Surface Bounding Box Length measurement.
#define GO_MEASUREMENT_SURFACE_BOUNDING_BOX_GLOBAL_X                (39)    ///< Surface Bounding Box Global X measurement.
#define GO_MEASUREMENT_SURFACE_BOUNDING_BOX_GLOBAL_Y                (40)    ///< Surface Bounding Box Global Y measurement.
#define GO_MEASUREMENT_SURFACE_BOUNDING_BOX_GLOBAL_Z_ANGLE          (41)    ///< Surface Bounding Box Global Z Angle measurement.
#define GO_MEASUREMENT_SURFACE_COUNTERSUNK_HOLE_X                   (42)    ///< Surface Countersunk Hole tool X position measurement.
#define GO_MEASUREMENT_SURFACE_COUNTERSUNK_HOLE_Y                   (43)    ///< Surface Countersunk Hole tool Y position measurement.
#define GO_MEASUREMENT_SURFACE_COUNTERSUNK_HOLE_Z                   (44)    ///< Surface Countersunk Hole tool Z position measurement.
#define GO_MEASUREMENT_SURFACE_COUNTERSUNK_HOLE_OUTER_RADIUS        (45)    ///< Surface Countersunk Hole tool Outer Radius measurement.
#define GO_MEASUREMENT_SURFACE_COUNTERSUNK_HOLE_DEPTH               (46)    ///< Surface Countersunk Hole tool Depth measurement.
#define GO_MEASUREMENT_SURFACE_COUNTERSUNK_HOLE_COUNTERBORE_DEPTH   (108)   ///< Surface Countersunk Hole tool Counterbore Depth measurement.
#define GO_MEASUREMENT_SURFACE_COUNTERSUNK_HOLE_BEVEL_RADIUS        (47)    ///< Surface Countersunk Hole tool Bevel Radius measurement.
#define GO_MEASUREMENT_SURFACE_COUNTERSUNK_HOLE_BEVEL_ANGLE         (48)    ///< Surface Countersunk Hole tool Bevel Angle measurement.
#define GO_MEASUREMENT_SURFACE_COUNTERSUNK_HOLE_X_ANGLE             (49)    ///< Surface Countersunk Hole tool X Angle measurement.
#define GO_MEASUREMENT_SURFACE_COUNTERSUNK_HOLE_Y_ANGLE             (50)    ///< Surface Countersunk Hole tool Y Angle measurement.
#define GO_MEASUREMENT_SURFACE_COUNTERSUNK_HOLE_AXIS_TILT           (122)   ///< Surface Countersunk Hole tool axis tilt measurement.
#define GO_MEASUREMENT_SURFACE_COUNTERSUNK_HOLE_AXIS_ORIENTATION    (123)   ///< Surface Countersunk Hole tool axis orientation measurement.
#define GO_MEASUREMENT_SURFACE_DIMENSION_WIDTH                      (114)   ///< Surface Dimension tool Width measurement.
#define GO_MEASUREMENT_SURFACE_DIMENSION_HEIGHT                     (115)   ///< Surface Dimension tool Height measurement.
#define GO_MEASUREMENT_SURFACE_DIMENSION_LENGTH                     (116)   ///< Surface Dimension tool Length measurement.
#define GO_MEASUREMENT_SURFACE_DIMENSION_DISTANCE                   (117)   ///< Surface Dimension tool Distance measurement.
#define GO_MEASUREMENT_SURFACE_DIMENSION_PLANE_DISTANCE             (118)   ///< Surface Dimension tool Plane Distance measurement.
#define GO_MEASUREMENT_SURFACE_DIMENSION_CENTER_X                   (119)   ///< Surface Dimension tool Center X measurement.
#define GO_MEASUREMENT_SURFACE_DIMENSION_CENTER_Y                   (120)   ///< Surface Dimension tool Center Y measurement.
#define GO_MEASUREMENT_SURFACE_DIMENSION_CENTER_Z                   (121)   ///< Surface Dimension tool Center Z measurement.
#define GO_MEASUREMENT_SURFACE_ELLIPSE_MAJOR                        (51)    ///< Surface Ellipse tool Major measurement.
#define GO_MEASUREMENT_SURFACE_ELLIPSE_MINOR                        (52)    ///< Surface Ellipse tool Minor measurement.
#define GO_MEASUREMENT_SURFACE_ELLIPSE_RATIO                        (53)    ///< Surface Ellipse tool Ratio measurement.
#define GO_MEASUREMENT_SURFACE_ELLIPSE_ZANGLE                       (54)    ///< Surface Ellipse tool Z Angle measurement.
#define GO_MEASUREMENT_SURFACE_HOLE_X                               (55)    ///< Surface Hole tool X measurement.
#define GO_MEASUREMENT_SURFACE_HOLE_Y                               (56)    ///< Surface Hole tool Y measurement.
#define GO_MEASUREMENT_SURFACE_HOLE_Z                               (57)    ///< Surface Hole tool Z measurement.
#define GO_MEASUREMENT_SURFACE_HOLE_RADIUS                          (58)    ///< Surface Hole tool Radius measurement.
#define GO_MEASUREMENT_SURFACE_OPENING_X                            (59)    ///< Surface Opening tool X measurement.
#define GO_MEASUREMENT_SURFACE_OPENING_Y                            (60)    ///< Surface Opening tool Y measurement.
#define GO_MEASUREMENT_SURFACE_OPENING_Z                            (61)    ///< Surface Opening tool Z measurement.
#define GO_MEASUREMENT_SURFACE_OPENING_WIDTH                        (62)    ///< Surface Opening tool Width measurement.
#define GO_MEASUREMENT_SURFACE_OPENING_LENGTH                       (63)    ///< Surface Opening tool Length measurement.
#define GO_MEASUREMENT_SURFACE_OPENING_ANGLE                        (64)    ///< Surface Opening tool Angle measurement.
#define GO_MEASUREMENT_SURFACE_PLANE_X_ANGLE                        (65)    ///< Surface Plane tool X Angle measurement.
#define GO_MEASUREMENT_SURFACE_PLANE_Y_ANGLE                        (66)    ///< Surface Plane tool Y Angle measurement.
#define GO_MEASUREMENT_SURFACE_PLANE_Z_OFFSET                       (67)    ///< Surface Plane tool Z Offset measurement.
#define GO_MEASUREMENT_SURFACE_PLANE_STD_DEV                        (109)   ///< Surface Plane tool Standard Deviation measurement.
#define GO_MEASUREMENT_SURFACE_PLANE_ERROR_MIN                      (110)   ///< Surface Plane tool Minimum Error measurement.
#define GO_MEASUREMENT_SURFACE_PLANE_ERROR_MAX                      (111)   ///< Surface Plane tool Maximum Error measurement.
#define GO_MEASUREMENT_SURFACE_PLANE_X_NORMAL                       (126)   ///< Surface Plane tool X Normal measurement.
#define GO_MEASUREMENT_SURFACE_PLANE_Y_NORMAL                       (127)   ///< Surface Plane tool Y Normal measurement.
#define GO_MEASUREMENT_SURFACE_PLANE_Z_NORMAL                       (128)   ///< Surface Plane tool Z Normal measurement.
#define GO_MEASUREMENT_SURFACE_PLANE_DISTANCE                       (129)   ///< Surface Plane tool X Normal measurement.
#define GO_MEASUREMENT_SURFACE_EDGE_X                               (130)   ///< Surface Position edge tool X measurement.
#define GO_MEASUREMENT_SURFACE_EDGE_Y                               (131)   ///< Surface Position edge tool Y measurement.
#define GO_MEASUREMENT_SURFACE_EDGE_Z                               (132)   ///< Surface Position edge tool Z measurement.
#define GO_MEASUREMENT_SURFACE_INTERSECT_X                          (133)   ///< Surface Position intersect tool X measurement.
#define GO_MEASUREMENT_SURFACE_INTERSECT_Y                          (134)   ///< Surface Position intersect tool Y measurement.
#define GO_MEASUREMENT_SURFACE_INTERSECT_Z                          (135)   ///< Surface Position intersect tool Z measurement.
#define GO_MEASUREMENT_SURFACE_INTERSECT_ANGLE                      (136)   ///< Surface Position intersect tool angle.

#define GO_MEASUREMENT_SURFACE_POSITION_X                           (68)    ///< Surface Position tool X measurement.
#define GO_MEASUREMENT_SURFACE_POSITION_Y                           (69)    ///< Surface Position tool Y measurement.
#define GO_MEASUREMENT_SURFACE_POSITION_Z                           (70)    ///< Surface Position tool Z measurement.
#define GO_MEASUREMENT_SURFACE_RIVET_X                              (88)    ///< Surface Rivet tool X measurement.
#define GO_MEASUREMENT_SURFACE_RIVET_Y                              (89)    ///< Surface Rivet tool Y measurement.
#define GO_MEASUREMENT_SURFACE_RIVET_Z                              (90)    ///< Surface Rivet tool Z measurement.
#define GO_MEASUREMENT_SURFACE_RIVET_TILT_ANGLE                     (91)    ///< Surface Rivet tool X Angle measurement.
#define GO_MEASUREMENT_SURFACE_RIVET_TILT_DIRECTION                 (92)    ///< Surface Rivet tool Y Angle measurement.
#define GO_MEASUREMENT_SURFACE_RIVET_RADIUS                         (93)    ///< Surface Rivet tool Radius measurement.
#define GO_MEASUREMENT_SURFACE_RIVET_TOP_OFFSET_MIN                 (94)    ///< Surface Rivet tool Top Offset Minimum measurement.
#define GO_MEASUREMENT_SURFACE_RIVET_TOP_OFFSET_MAX                 (95)    ///< Surface Rivet tool Top Offset Maximum measurement.
#define GO_MEASUREMENT_SURFACE_RIVET_TOP_OFFSET_MEAN                (96)    ///< Surface Rivet tool Top Offset Mean measurement.
#define GO_MEASUREMENT_SURFACE_RIVET_TOP_OFFSET_STD_DEV             (97)    ///< Surface Rivet tool Top Offset Standard Deviation measurement.
#define GO_MEASUREMENT_SURFACE_RIVET_RADIAL_HEIGHT_MIN              (98)    ///< Surface Rivet tool Radial Height Minimum measurement.
#define GO_MEASUREMENT_SURFACE_RIVET_RADIAL_HEIGHT_MAX              (99)    ///< Surface Rivet tool Radial Height Maximum measurement.
#define GO_MEASUREMENT_SURFACE_RIVET_RADIAL_HEIGHT_MEAN             (100)   ///< Surface Rivet tool Radial Height Mean measurement.
#define GO_MEASUREMENT_SURFACE_RIVET_RADIAL_HEIGHT_STD_DEV          (101)   ///< Surface Rivet tool Radial Height Standard Deviation measurement.
#define GO_MEASUREMENT_SURFACE_RIVET_RADIAL_SLOPE_MIN               (102)   ///< Surface Rivet tool Radial Slope Minimum measurement.
#define GO_MEASUREMENT_SURFACE_RIVET_RADIAL_SLOPE_MAX               (103)   ///< Surface Rivet tool Radial Slope Maximum measurement.
#define GO_MEASUREMENT_SURFACE_RIVET_RADIAL_SLOPE_MEAN              (104)   ///< Surface Rivet tool Radial Slope Mean measurement.
#define GO_MEASUREMENT_SURFACE_RIVET_RADIAL_SLOPE_STD_DEV           (105)   ///< Surface Rivet tool Radial Slope Standard Deviation measurement.
#define GO_MEASUREMENT_SURFACE_STUD_BASE_X                          (71)    ///< Surface Stud tool Base X measurement.
#define GO_MEASUREMENT_SURFACE_STUD_BASE_Y                          (72)    ///< Surface Stud tool Base Y measurement.
#define GO_MEASUREMENT_SURFACE_STUD_BASE_Z                          (73)    ///< Surface Stud tool Base Z measurement.
#define GO_MEASUREMENT_SURFACE_STUD_TIP_X                           (74)    ///< Surface Stud tool Tip X measurement.
#define GO_MEASUREMENT_SURFACE_STUD_TIP_Y                           (75)    ///< Surface Stud tool Tip Y measurement.
#define GO_MEASUREMENT_SURFACE_STUD_TIP_Z                           (76)    ///< Surface Stud tool Tip Z measurement.
#define GO_MEASUREMENT_SURFACE_STUD_RADIUS                          (77)    ///< Surface Stud tool Radius measurement.
#define GO_MEASUREMENT_SURFACE_VOLUME_AREA                          (78)    ///< Surface Volume tool Area measurement.
#define GO_MEASUREMENT_SURFACE_VOLUME_VOLUME                        (79)    ///< Surface Volume tool Volume measurement.
#define GO_MEASUREMENT_SURFACE_VOLUME_THICKNESS                     (80)    ///< Surface Volume tool Thickness measurement.
#define GO_MEASUREMENT_SCRIPT_OUTPUT                                (81)    ///< Script tool Output.
#define GO_MEASUREMENT_EXTENSIBLE                                   (87)    ///< Extensible tool measurement.
#define GO_MEASUREMENT_PROFILE_ROUND_CORNER_X                       (123)   ///< Profile Round Corner tool X measurement.
#define GO_MEASUREMENT_PROFILE_ROUND_CORNER_Z                       (124)   ///< Profile Round Corner tool Z measurement.
#define GO_MEASUREMENT_PROFILE_ROUND_CORNER_ANGLE                   (125)   ///< Profile Round Corner tool Angle measurement.



#define GO_FEATURE_DIMENSION_WIDTH                                  (140)   ///< Dimension tool width Intersect angle.
#define GO_FEATURE_DIMENSION_LENGTH                                 (141)   ///< Dimension tool length Intersect angle.
#define GO_FEATURE_DIMENSION_HEIGHT                                 (142)   ///< Dimension tool height Intersect angle.
#define GO_FEATURE_DIMENSION_DISTANCE                               (143)   ///< Dimension tool distance Intersect angle.
#define GO_FEATURE_DIMENSION_PLANEDISTANCE                          (144)   ///< Dimension tool plane distance Intersect angle.
#define GO_FEATURE_DIMENSION_CENTERX                                (145)   ///< Dimension tool center x Intersect angle.
#define GO_FEATURE_DIMENSION_CENTERY                                (146)   ///< Dimension tool center y Intersect angle.
#define GO_FEATURE_DIMENSION_CENTERZ                                (147)   ///< Dimension tool center z Intersect angle.

/**@}*/

// DO NOT CHANGE OR RE-ORDER THESE PUBLICALLY DEFINED ENUMERATOR VALUES. 
// NEW VALUES MUST BE APPENDED, NEVER INSERTED.
/**
* @struct  GoFeatureType
* @extends kValue
* @ingroup GoSdk-Tools
* @brief   Lists all tool feature types.
*
* The following enumerators are defined:
* - #GO_FEATURE_UNKNOWN
* - #GO_FEATURE_EXTENSIBLE
* - #GO_FEATURE_SURFACE_EDGE_EDGE_LINE
* - #GO_FEATURE_SURFACE_CENTER_POINT
* - #GO_FEATURE_SURFACE_BOUNDING_BOX_CENTER_POINT
* - #GO_FEATURE_SURFACE_COUNTERSUNKHOLE_CENTER_POINT
* - #GO_FEATURE_SURFACE_DIMENSION_CENTER_POINT
* - #GO_FEATURE_SURFACE_ELLIPSE_CENTER_POINT
* - #GO_FEATURE_SURFACE_ELLIPSE_MAJOR_AXIS_LINE
* - #GO_FEATURE_SURFACE_ELLIPSE_MINOR_AXIS_LINE
* - #GO_FEATURE_SURFACE_HOLE_CENTER_POINT
* - #GO_FEATURE_SURFACE_OPENING_CENTER_POINT
* - #GO_FEATURE_SURFACE_PLANE_PLANE
* - #GO_FEATURE_SURFACE_POSITION_POINT
* - #GO_FEATURE_SURFACE_STUD_TIP_POINT
* - #GO_FEATURE_SURFACE_STUD_BASE_POINT
* - #GO_FEATURE_SURFACE_BOUNDING_BOX_AXIS_LINE
* - #GO_FEATURE_PROFILE_POSITION_POINT
* - #GO_FEATURE_PROFILE_LINE_LINE
* - #GO_FEATURE_PROFILE_LINE_MIN_ERROR_POINT
* - #GO_FEATURE_PROFILE_LINE_MAX_ERROR_POINT
* - #GO_FEATURE_PROFILE_INTERSECT_INTERSECT_POINT
* - #GO_FEATURE_PROFILE_INTERSECT_LINE
* - #GO_FEATURE_PROFILE_INTERSECT_BASE_LINE
* - #GO_FEATURE_PROFILE_BOUNDING_BOX_CENTER_POINT
* - #GO_FEATURE_PROFILE_BOUNDING_BOX_CORNER_POINT
* - #GO_FEATURE_PROFILE_AREA_CENTER_POINT
* - #GO_FEATURE_PROFILE_CIRCLE_CENTER_POINT
* - #GO_FEATURE_PROFILE_DIMENSION_CENTER_POINT
* - #GO_FEATURE_PROFILE_PANEL_LEFT_GAP_POINT
* - #GO_FEATURE_PROFILE_PANEL_LEFT_FLUSH_POINT
* - #GO_FEATURE_PROFILE_PANEL_RIGHT_GAP_POINT
* - #GO_FEATURE_PROFILE_PANEL_RIGHT_FLUSH_POINT
* - #GO_FEATURE_PROFILE_ROUND_CORNER_POINT
* - #GO_FEATURE_PROFILE_ROUND_CORNER_EDGE_POINT
* - #GO_FEATURE_PROFILE_ROUND_CORNER_CENTER_POINT
*/
typedef k32s GoFeatureType;
/** @name    GoFeatureType
*@{*/
#define GO_FEATURE_UNKNOWN                                          (-1)   ///< Unknown feature.
#define GO_FEATURE_EXTENSIBLE                                       (0)    ///< Extensible feature.
#define GO_FEATURE_SURFACE_EDGE_EDGE_LINE                           (1)     ///< Surface Edge Edge Line feature.
#define GO_FEATURE_SURFACE_CENTER_POINT                             (2)     ///< Surface Center Point feature.
#define GO_FEATURE_SURFACE_BOUNDING_BOX_CENTER_POINT                (3)     ///< Surface Bounding Box Center Point feature.
#define GO_FEATURE_SURFACE_COUNTERSUNKHOLE_CENTER_POINT             (4)     ///< Surface Countersunk Hole Center Point feature.
#define GO_FEATURE_SURFACE_DIMENSION_CENTER_POINT                   (5)     ///< Surface Dimension Center Point feature.
#define GO_FEATURE_SURFACE_ELLIPSE_CENTER_POINT                     (6)     ///< Surface Ellipse Center Point feature.
#define GO_FEATURE_SURFACE_ELLIPSE_MAJOR_AXIS_LINE                  (7)     ///< Surface Ellipse Major Axis feature.
#define GO_FEATURE_SURFACE_ELLIPSE_MINOR_AXIS_LINE                  (8)     ///< Surface Ellipse Minor Axis feature.
#define GO_FEATURE_SURFACE_HOLE_CENTER_POINT                        (9)     ///< Surface Hole Center Point feature.
#define GO_FEATURE_SURFACE_OPENING_CENTER_POINT                     (10)     ///< Surface Opening Center Point feature.
#define GO_FEATURE_SURFACE_PLANE_PLANE                              (11)     ///< Surface Plane Plane feature.
#define GO_FEATURE_SURFACE_POSITION_POINT                           (12)     ///< Surface Position Point feature.
#define GO_FEATURE_SURFACE_STUD_TIP_POINT                           (13)     ///< Surface Stud Tip Point feature.
#define GO_FEATURE_SURFACE_STUD_BASE_POINT                          (14)     ///< Surface Stud Base Point feature.
#define GO_FEATURE_SURFACE_BOUNDING_BOX_AXIS_LINE                   (15)     ///< Surface Bounding Box Axis Line feature.

#define GO_FEATURE_PROFILE_POSITION_POINT                           (50)     ///< Profile Position Point feature.
#define GO_FEATURE_PROFILE_LINE_LINE                                (51)     ///< Profile Line Line feature.
#define GO_FEATURE_PROFILE_LINE_MIN_ERROR_POINT                     (52)     ///< Profile Line Minimum Error Point feature.
#define GO_FEATURE_PROFILE_LINE_MAX_ERROR_POINT                     (53)     ///< Profile Line Maximum Error Point feature.
#define GO_FEATURE_PROFILE_INTERSECT_INTERSECT_POINT                (54)     ///< Profile Intersect Intersect Point feature.
#define GO_FEATURE_PROFILE_INTERSECT_LINE                           (55)     ///< Profile Intersect Line feature.
#define GO_FEATURE_PROFILE_INTERSECT_BASE_LINE                      (56)     ///< Profile Intersect Base Line feature.
#define GO_FEATURE_PROFILE_BOUNDING_BOX_CENTER_POINT                (57)     ///< Profile Bounding Box Center Point feature.
#define GO_FEATURE_PROFILE_BOUNDING_BOX_CORNER_POINT                (58)     ///< Profile Bounding Box Corner Point feature.
#define GO_FEATURE_PROFILE_AREA_CENTER_POINT                        (59)     ///< Profile Area Center Point feature.
#define GO_FEATURE_PROFILE_CIRCLE_CENTER_POINT                      (60)     ///< Profile Circle Center Point feature.
#define GO_FEATURE_PROFILE_DIMENSION_CENTER_POINT                   (61)     ///< Profile Dimension Center Point feature.
#define GO_FEATURE_PROFILE_PANEL_LEFT_GAP_POINT                     (62)     ///< Profile Panel Left Gap Point feature.
#define GO_FEATURE_PROFILE_PANEL_LEFT_FLUSH_POINT                   (63)     ///< Profile Panel Left Flush Point feature.
#define GO_FEATURE_PROFILE_PANEL_RIGHT_GAP_POINT                    (64)     ///< Profile Panel Right Gap Point feature.
#define GO_FEATURE_PROFILE_PANEL_RIGHT_FLUSH_POINT                  (65)     ///< Profile Panel Right Flush Point feature.
#define GO_FEATURE_PROFILE_ROUND_CORNER_POINT                       (66)     ///< Profile Panel Round Corner Point feature.
#define GO_FEATURE_PROFILE_ROUND_CORNER_EDGE_POINT                  (67)     ///< Profile Panel Round Corner Edge Point feature.
#define GO_FEATURE_PROFILE_ROUND_CORNER_CENTER_POINT                (68)     ///< Profile Panel Round Corner Center Point feature.
/**@}*/


#define GO_MEASUREMENT_ID_NONE                                      (-1)
/**
 * @struct  GoDataMessageType
 * @extends kValue
 * @ingroup GoSdk-DataChannel
 * @brief   Lists all data message types.
 *
 * The following enumerators are defined:
 * - #GO_DATA_MESSAGE_TYPE_UNKNOWN
 * - #GO_DATA_MESSAGE_TYPE_STAMP
 * - #GO_DATA_MESSAGE_TYPE_HEALTH
 * - #GO_DATA_MESSAGE_TYPE_VIDEO
 * - #GO_DATA_MESSAGE_TYPE_RANGE
 * - #GO_DATA_MESSAGE_TYPE_RANGE_INTENSITY
 * - #GO_DATA_MESSAGE_TYPE_PROFILE_POINT_CLOUD
 * - #GO_DATA_MESSAGE_TYPE_PROFILE_INTENSITY
 * - #GO_DATA_MESSAGE_TYPE_UNIFORM_PROFILE
 * - #GO_DATA_MESSAGE_TYPE_UNIFORM_SURFACE
 * - #GO_DATA_MESSAGE_TYPE_SURFACE_INTENSITY
 * - #GO_DATA_MESSAGE_TYPE_MEASUREMENT
 * - #GO_DATA_MESSAGE_TYPE_ALIGNMENT
 * - #GO_DATA_MESSAGE_TYPE_EXPOSURE_CAL
 * - #GO_DATA_MESSAGE_TYPE_EDGE_MATCH
 * - #GO_DATA_MESSAGE_TYPE_BOUNDING_BOX_MATCH
 * - #GO_DATA_MESSAGE_TYPE_ELLIPSE_MATCH
 * - #GO_DATA_MESSAGE_TYPE_SECTION
 * - #GO_DATA_MESSAGE_TYPE_SECTION_INTENSITY
 * - #GO_DATA_MESSAGE_TYPE_EVENT
 * - #GO_DATA_MESSAGE_TYPE_TRACHEID
 * - #GO_DATA_MESSAGE_TYPE_FEATURE_POINT
 * - #GO_DATA_MESSAGE_TYPE_FEATURE_LINE
 * - #GO_DATA_MESSAGE_TYPE_FEATURE_PLANE
 * - #GO_DATA_MESSAGE_TYPE_FEATURE_CIRCLE
 * - #GO_DATA_MESSAGE_TYPE_SURFACE_POINT_CLOUD
 * - #GO_DATA_MESSAGE_TYPE_GENERIC
 * - #GO_DATA_MESSAGE_TYPE_MESH
 * - #GO_DATA_MESSAGE_TYPE_PROFILE //Deprecated use GO_DATA_MESSAGE_TYPE_PROFILE_POINT_CLOUD instead
 * - #GO_DATA_MESSAGE_TYPE_RESAMPLED_PROFILE //Deprecated use GO_DATA_MESSAGE_TYPE_UNIFORM_PROFILE instead
 * - #GO_DATA_MESSAGE_TYPE_SURFACE //Deprecated use GO_DATA_MESSAGE_TYPE_UNIFORM_SURFACE
 */
/* These definitions are similar to the GO_COMPACT_MESSAGE_XXX but are not
 * guaranteed to be the same.
 *
 * This set of GO_DATA_MESSAGE_TYPE_xxx definitions are external (customer)
 * facing and must NEVER be renumbered. Some SDK applications hardcode these values.
 *
 */
typedef k32s GoDataMessageType;
/** @name    GoDataMessageType
 *@{*/
#define GO_DATA_MESSAGE_TYPE_UNKNOWN                                 -1   ///< Unknown message type.
#define GO_DATA_MESSAGE_TYPE_STAMP                                   0    ///< Stamp message type.
#define GO_DATA_MESSAGE_TYPE_HEALTH                                  1    ///< Health message type.
#define GO_DATA_MESSAGE_TYPE_VIDEO                                   2    ///< Video message type.
#define GO_DATA_MESSAGE_TYPE_RANGE                                   3    ///< Range message type.
#define GO_DATA_MESSAGE_TYPE_RANGE_INTENSITY                         4    ///< Range Intensity message type.
#define GO_DATA_MESSAGE_TYPE_PROFILE_POINT_CLOUD                     5    ///< Unresampled Profile message type.
#define GO_DATA_MESSAGE_TYPE_PROFILE_INTENSITY                       6    ///< Profile Point Cloud (or Uniform Profile) Intensity message type.
#define GO_DATA_MESSAGE_TYPE_UNIFORM_PROFILE                         7    ///< Uniform (resampled) Profile message type.
#define GO_DATA_MESSAGE_TYPE_UNIFORM_SURFACE                         8    ///< Uniform (resampled) Surface message type.
#define GO_DATA_MESSAGE_TYPE_SURFACE_INTENSITY                       9    ///< Surface Point Cloud (or Uniform Surface) Intensity message type.
#define GO_DATA_MESSAGE_TYPE_MEASUREMENT                             10   ///< Measurement message type.
#define GO_DATA_MESSAGE_TYPE_ALIGNMENT                               11   ///< Alignment result message type.
#define GO_DATA_MESSAGE_TYPE_EXPOSURE_CAL                            12   ///< Exposure AutoSet/Calibration result message type.
#define GO_DATA_MESSAGE_TYPE_EDGE_MATCH                              16   ///< Part matching edge algorithm message type.
#define GO_DATA_MESSAGE_TYPE_BOUNDING_BOX_MATCH                      17   ///< Part matching bounding box algorithm message type.
#define GO_DATA_MESSAGE_TYPE_ELLIPSE_MATCH                           18   ///< Part matching ellipse algorithm message type.
#define GO_DATA_MESSAGE_TYPE_SECTION                                 20   ///< Section message type.
#define GO_DATA_MESSAGE_TYPE_SECTION_INTENSITY                       21   ///< Section Intensity message type.
#define GO_DATA_MESSAGE_TYPE_EVENT                                   22   ///< Event message type.
#define GO_DATA_MESSAGE_TYPE_TRACHEID                                23   ///< Tracheid message type.
#define GO_DATA_MESSAGE_TYPE_FEATURE_POINT                           24   ///< Point Feature message type.
#define GO_DATA_MESSAGE_TYPE_FEATURE_LINE                            25   ///< Line Feature message type.
#define GO_DATA_MESSAGE_TYPE_FEATURE_PLANE                           26   ///< Plane Feature message type.
#define GO_DATA_MESSAGE_TYPE_FEATURE_CIRCLE                          27   ///< Circle Feature message type.
#define GO_DATA_MESSAGE_TYPE_SURFACE_POINT_CLOUD                     28   ///< Surface Point Cloud (Un-Resampled surface) message type.
#define GO_DATA_MESSAGE_TYPE_GENERIC                                 29   ///< Generic message type.
#define GO_DATA_MESSAGE_TYPE_NULL                                    30   ///< Null message type.

#define GO_DATA_MESSAGE_TYPE_MESH                                    36   ///< Mesh message type.


// Refinements to measurement and feature messages were required to make them work with array support.
// The below GoDataMessageType ids are used INTERNALLY to distinguish the arrayed versions of measurements and features
// but they automatically still resolve to the same respective Go<>Msg object, albeit with a different id.
///
// eg. For measurements:
//     Unary GoMeasureMsg::id = GO_DATA_MESSAGE_TYPE_MEASUREMENT
//     (internal Arrayed GvMeasureMsgs use GO_COMPACT_MESSAGE_MEASUREMENT_V2)
//     Arrayed GoMeasureMsg::id = GO_DATA_MESSAGE_TYPE_MEASUREMENT_V2  (but still a GoMeasureMsg object).
//
#define GO_DATA_MESSAGE_TYPE_MEASUREMENT_V2                         (31)  ///< Measurement message type V2.
#define GO_DATA_MESSAGE_TYPE_FEATURE_POINT_V2                       (32)  ///< Point Feature message type V2. 
#define GO_DATA_MESSAGE_TYPE_FEATURE_LINE_V2                        (33)  ///< Line Feature message type V2.
#define GO_DATA_MESSAGE_TYPE_FEATURE_PLANE_V2                       (34)  ///< Plane Feature message type V2.
#define GO_DATA_MESSAGE_TYPE_FEATURE_CIRCLE_V2                      (35)  ///< Circle Feature message type V2.


#define GO_DATA_MESSAGE_TYPE_PROFILE GO_DATA_MESSAGE_TYPE_PROFILE_POINT_CLOUD ///< Deprecated Unresampled Profile message type.
#define GO_DATA_MESSAGE_TYPE_RESAMPLED_PROFILE GO_DATA_MESSAGE_TYPE_UNIFORM_PROFILE ///< Deprecated Uniform (resampled) Profile message type.
#define GO_DATA_MESSAGE_TYPE_SURFACE GO_DATA_MESSAGE_TYPE_UNIFORM_SURFACE ///< Deprecated Surface message type.
/**@}*/

/**
* @struct  GoReplayConditionType
* @extends kValue
* @ingroup GoSdk-Replay
* @brief   Represents a replay condition type.
*
* The following enumerators are defined:
* - #GO_REPLAY_CONDITION_TYPE_ANY_MEASUREMENT
* - #GO_REPLAY_CONDITION_TYPE_ANY_DATA
* - #GO_REPLAY_CONDITION_TYPE_MEASUREMENT
*/
typedef k32s GoReplayConditionType;
/** @name    GoReplayConditionType
*@{*/
#define GO_REPLAY_CONDITION_TYPE_ANY_MEASUREMENT        (0) ///< Any Measurement condition.
#define GO_REPLAY_CONDITION_TYPE_ANY_DATA               (1) ///< Any Data condition.
#define GO_REPLAY_CONDITION_TYPE_MEASUREMENT            (2) ///< Measurement condition.

/**@}*/

/**
* @struct  GoReplayCombineType
* @extends kValue
* @ingroup GoSdk-Replay
* @brief   Represents a replay combine type.
*
* The following enumerators are defined:
* - #GO_REPLAY_COMBINE_TYPE_ANY
* - #GO_REPLAY_COMBINE_TYPE_ALL
*/
typedef k32s GoReplayCombineType;
/** @name    GoReplayCombineType
*@{*/
#define GO_REPLAY_COMBINE_TYPE_ANY               (0)         ///< Any
#define GO_REPLAY_COMBINE_TYPE_ALL               (1)         ///< All
/**@}*/

/**
* @struct  GoReplayMeasurementResult
* @extends kValue
* @ingroup GoSdk-Replay
* @brief   Represents a replay measurement result.
*
* The following enumerators are defined:
* - #GO_REPLAY_MEASUREMENT_RESULT_PASS
* - #GO_REPLAY_MEASUREMENT_RESULT_FAIL
* - #GO_REPLAY_MEASUREMENT_RESULT_VALID
* - #GO_REPLAY_MEASUREMENT_RESULT_INVALID
* - #GO_REPLAY_MEASUREMENT_RESULT_FAIL_OR_INVALID
*/
typedef k32s GoReplayMeasurementResult;
/** @name    GoReplayMeasurementResult
*@{*/
#define GO_REPLAY_MEASUREMENT_RESULT_PASS               (0)         ///< Pass
#define GO_REPLAY_MEASUREMENT_RESULT_FAIL               (1)         ///< Fail
#define GO_REPLAY_MEASUREMENT_RESULT_VALID              (2)         ///< Valid
#define GO_REPLAY_MEASUREMENT_RESULT_INVALID            (3)         ///< Invalid
#define GO_REPLAY_MEASUREMENT_RESULT_FAIL_OR_INVALID    (4)         ///< Fail or Invalid
/**@}*/

/**
* @struct  GoReplayRangeCountCase
* @extends kValue
* @ingroup GoSdk-Replay
* @brief   Represents a replay range count case.
*
* The following enumerators are defined:
* - #GO_REPLAY_RANGE_COUNT_CASE_AT_ABOVE
* - #GO_REPLAY_RANGE_COUNT_CASE_BELOW
*/
typedef k32s GoReplayRangeCountCase;
/** @name    GoReplayRangeCountCase
*@{*/
#define GO_REPLAY_RANGE_COUNT_CASE_AT_ABOVE             (0)         ///< Case at above
#define GO_REPLAY_RANGE_COUNT_CASE_BELOW                (1)         ///< Case below
/**@}*/

/**
* @struct  GoSensorAccelState
* @extends kValue
* @ingroup GoSdk
* @brief   Lists all sensor acceleration states that a sensor can be in.
*          When a sensor is being accelerated, GoSensorAccelStatus
*          provides more detail on the status of the acceleration.
*          These are applicable only when using the GoAcceleratorMgr class.
*
* The following enumerators are defined:
* - #GO_SENSOR_ACCEL_STATE_UNKNOWN
* - #GO_SENSOR_ACCEL_STATE_AVAILABLE
* - #GO_SENSOR_ACCEL_STATE_ACCELERATED
* - #GO_SENSOR_ACCEL_STATE_ACCELERATED_BY_OTHER
* - #GO_SENSOR_ACCEL_STATE_FW_MISMATCH
*/
typedef k32s GoSensorAccelState;
/** @name    GoSensorAccelState
*@{*/
#define GO_SENSOR_ACCEL_STATE_UNKNOWN               (0)     ///< State could not be determined.
#define GO_SENSOR_ACCEL_STATE_AVAILABLE             (1)     ///< Sensor is a candidate for acceleration.
#define GO_SENSOR_ACCEL_STATE_ACCELERATED           (2)     ///< Sensor is accelerated by this host.
#define GO_SENSOR_ACCEL_STATE_ACCELERATED_BY_OTHER  (3)     ///< Sensor is accelerated by another host.
#define GO_SENSOR_ACCEL_STATE_FW_MISMATCH           (4)     ///< Sensor firmware does not match accelerator program version.
/**@}*/

/**
* @struct  GoSensorAccelStatus
* @extends kValue
* @ingroup GoSdk
* @brief   Represents the acceleration status of a sensor that is available or
*          being accelerated by the local host. The corresponding acceleration state
*          can be GO_SENSOR_ACCEL_STATE_AVAILABLE (while acceleration has not
*          yet) or GO_SENSOR_ACCEL_STATE_ACCELERATED (acceleration completed).
*          These status values are not applicable for a sensor
*          accelerated by another host (ie. state is GO_SENSOR_ACCEL_STATE_ACCELERATED_BY_OTHER)).
*          These statuses are applicable only when using the GoAcceleratorMgr class.
*
* The following enumerators are defined:
* - #GO_SENSOR_ACCEL_STATUS_SUCCESS
* - #GO_SENSOR_ACCEL_STATUS_ACCELERATING
* - #GO_SENSOR_ACCEL_STATUS_DECELERATING
* - #GO_SENSOR_ACCEL_STATUS_MISSING
* - #GO_SENSOR_ACCEL_STATUS_STOPPED
* - #GO_SENSOR_ACCEL_STATUS_FAIL_TO_ACCEL
* - #GO_SENSOR_ACCEL_STATUS_STOPPED_AVAILABLE
* - #GO_SENSOR_ACCEL_STATUS_STOPPED_ACCELERATED_BY_OTHER
* - #GO_SENSOR_ACCEL_STATUS_STOPPED_FW_MISMATCH
* - #GO_SENSOR_ACCEL_STATUS_STOPPED_PORT_IN_USE
* - #GO_SENSOR_ACCEL_STATUS_STOPPED_UNREACHABLE
*/
typedef k32s GoSensorAccelStatus;
/** @name    GoSensorAccelStatus
*@{*/
#define GO_SENSOR_ACCEL_STATUS_SUCCESS               (0)      ///< Sensor accelerated successfully.
#define GO_SENSOR_ACCEL_STATUS_ACCELERATING          (-1)     ///< Sensor is in the process of being accelerated.
#define GO_SENSOR_ACCEL_STATUS_DECELERATING          (-2)     ///< Sensor is in the process of being unaccelerated.
#define GO_SENSOR_ACCEL_STATUS_MISSING               (-3)     ///< Sensor is accelerated, but has disappeared from network.
#define GO_SENSOR_ACCEL_STATUS_STOPPED               (-4)     ///< Sensor was accelerated but unexpectedly stopped (eg. crashed).
#define GO_SENSOR_ACCEL_STATUS_FAIL_TO_ACCEL         (-5)     ///< Generic failure to accelerate sensor.
#define GO_SENSOR_ACCEL_STATUS_STOPPED_AVAILABLE     (-6)     ///< Acceleration stopped and sensor is unaccelerated. Status is only for client use to elaborate on a STOPPED status.
#define GO_SENSOR_ACCEL_STATUS_STOPPED_ACCELERATED_BY_OTHER (-7)     ///< Acceleration stopped and sensor is now accelerated by another host. Status is only for client use to elaborate on a STOPPED status.
#define GO_SENSOR_ACCEL_STATUS_STOPPED_FW_MISMATCH   (-8)     ///< Acceleration stopped and sensor now has an incompatible firmware version. Status is only for client use to elaborate on a STOPPED status.
#define GO_SENSOR_ACCEL_STATUS_STOPPED_PORT_IN_USE   (-9)     ///< Acceleration stopped because sensor ports are in use by another application. Status is only for client use to elaborate on a STOPPED status.
#define GO_SENSOR_ACCEL_STATUS_STOPPED_UNREACHABLE   (-10)     ///< Acceleration stopped because sensor in on an unreachable network. Status is only for client use to elaborate on a STOPPED status.

/**
* @struct  GoAdvancedType
* @extends kValue
* @note    Supported with G1, G2
* @ingroup GoSdk
* @brief   Represents advanced acquisition type.
*
* The following enumerators are defined:
* - #GO_ADVANCED_TYPE_CUSTOM
* - #GO_ADVANCED_TYPE_DIFFUSE
* - #GO_ADVANCED_TYPE_REFLECTIVE
*/
typedef k32s GoAdvancedType;
/** @name    GoAdvancedType
*@{*/
#define GO_ADVANCED_TYPE_CUSTOM              (0)    ///< Custom advanced acquisition type.
#define GO_ADVANCED_TYPE_DIFFUSE             (1)    ///< Diffuse advanced acquisition type.
#define GO_ADVANCED_TYPE_REFLECTIVE          (3)    ///< Reflective advanced acquisition type.
/**@}*/

/**
 * @struct  GoMaterialType
 * @deprecated
 * @extends kValue
 * @note    Supported with G1, G2
 * @ingroup GoSdk
 * @brief   Represents a material acquisition type.
 *
 * The following enumerators are defined:
 * - #GO_MATERIAL_TYPE_CUSTOM
 * - #GO_MATERIAL_TYPE_DIFFUSE
 */
typedef k32s GoMaterialType;
/** @name    GoMaterialType
 *@{*/
#define GO_MATERIAL_TYPE_CUSTOM              (0)    ///< Custom material acquisition type.
#define GO_MATERIAL_TYPE_DIFFUSE             (1)    ///< Diffuse material acquisition type.
/**@}*/

/**
 * @struct  GoSpotSelectionType
 * @extends kValue
 * @note    Supported with G1, G2
 * @ingroup GoSdk
 * @brief   Represents a spot selection type.
 *
 * The following enumerators are defined:
 * - #GO_SPOT_SELECTION_TYPE_BEST
 * - #GO_SPOT_SELECTION_TYPE_TOP
 * - #GO_SPOT_SELECTION_TYPE_BOTTOM
 * - #GO_SPOT_SELECTION_TYPE_NONE
 * - #GO_SPOT_SELECTION_TYPE_CONTINUITY
 * - #GO_SPOT_SELECTION_TYPE_TRANSLUCENT
 */
typedef k32s GoSpotSelectionType;
/** @name    GoSpotSelectionType
 *@{*/
#define GO_SPOT_SELECTION_TYPE_BEST         (0)    ///< Select the spot with the best value.
#define GO_SPOT_SELECTION_TYPE_TOP          (1)    ///< Select the top-most spot.
#define GO_SPOT_SELECTION_TYPE_BOTTOM       (2)    ///< Select the bottom-most spot.
#define GO_SPOT_SELECTION_TYPE_NONE         (3)    ///< Disable spot selection.
#define GO_SPOT_SELECTION_TYPE_CONTINUITY   (4)    ///< Select most continuous spot
#define GO_SPOT_SELECTION_TYPE_TRANSLUCENT  (5)    ///< Select translucent spot.
 /**@}*/

/**
 * @struct  GoTranslucentThreadingMode
 * @extends kValue
 * @note    Supported with G2
 * @ingroup GoSdk
 * @brief   Represents a translucent spot threading mode.
 *
 * The following enumerators are defined:
 * - #GO_TRANSLUCENT_THREADING_MODE_NONE
 * - #GO_TRANSLUCENT_THREADING_MODE_BATCHING
 */
typedef k32s GoTranslucentThreadingMode;
/** @name    GoTranslucentThreadingMode
 *@{*/
#define GO_TRANSLUCENT_THREADING_MODE_NONE      (0)    ///< Single thread mode.
#define GO_TRANSLUCENT_THREADING_MODE_BATCHING  (1)    ///< Batching mode.
 /**@}*/

/**
 * @struct  GoProfileStripBaseType
 * @extends kValue
 * @note    Supported with G1, G2
 * @ingroup GoSdk-ProfileTools
 * @brief   Represents a profile strip tool base type.
 *
 * The following enumerators are defined:
 * - #GO_PROFILE_STRIP_BASE_TYPE_NONE
 * - #GO_PROFILE_STRIP_BASE_TYPE_FLAT
 */
typedef k32s GoProfileStripBaseType;
/** @name    GoProfileStripBaseType
 *@{*/
#define GO_PROFILE_STRIP_BASE_TYPE_NONE     (0)     ///< No strip base type.
#define GO_PROFILE_STRIP_BASE_TYPE_FLAT     (1)     ///< Flat strip base type.
/**@}*/

/**
 * @struct  GoProfileStripEdgeType
 * @extends kValue
 * @note    Supported with G1, G2
 * @ingroup GoSdk-ProfileTools
 * @brief   Represents a profile strip tool edge type.
 *
 * The following enumerators are defined:
 * - #GO_PROFILE_STRIP_EDGE_TYPE_RISING
 * - #GO_PROFILE_STRIP_EDGE_TYPE_FALLING
 * - #GO_PROFILE_STRIP_EDGE_TYPE_DATA_END
 * - #GO_PROFILE_STRIP_EDGE_TYPE_VOID
 */
typedef k32s GoProfileStripEdgeType;
/** @name    GoProfileStripEdgeType
 *@{*/
#define GO_PROFILE_STRIP_EDGE_TYPE_RISING           (1)     ///< Rising strip edge type.
#define GO_PROFILE_STRIP_EDGE_TYPE_FALLING          (2)     ///< Falling strip edge type.
#define GO_PROFILE_STRIP_EDGE_TYPE_DATA_END         (4)     ///< Data end strip edge type.
#define GO_PROFILE_STRIP_EDGE_TYPE_VOID             (8)     ///< Void strip edge type.
/**@}*/


/**
 * @struct  GoProfileFeatureType
 * @note    Supported with G1, G2
 * @ingroup GoSdk-ProfileTools
 * @brief   Represents a profile feature point type.
 *
 * The following enumerators are defined:
 * - #GO_PROFILE_FEATURE_TYPE_MAX_Z
 * - #GO_PROFILE_FEATURE_TYPE_MIN_Z
 * - #GO_PROFILE_FEATURE_TYPE_MAX_X
 * - #GO_PROFILE_FEATURE_TYPE_MIN_X
 * - #GO_PROFILE_FEATURE_TYPE_CORNER
 * - #GO_PROFILE_FEATURE_TYPE_AVERAGE
 * - #GO_PROFILE_FEATURE_TYPE_RISING_EDGE
 * - #GO_PROFILE_FEATURE_TYPE_FALLING_EDGE
 * - #GO_PROFILE_FEATURE_TYPE_ANY_EDGE
 * - #GO_PROFILE_FEATURE_TYPE_TOP_CORNER
 * - #GO_PROFILE_FEATURE_TYPE_BOTTOM_CORNER
 * - #GO_PROFILE_FEATURE_TYPE_LEFT_CORNER
 * - #GO_PROFILE_FEATURE_TYPE_RIGHT_CORNER
 * - #GO_PROFILE_FEATURE_TYPE_MEDIAN
 */
typedef k32s GoProfileFeatureType;
/** @name    GoProfileFeatureType
 *@{*/
#define GO_PROFILE_FEATURE_TYPE_MAX_Z               (0)                     ///< Point with the maximum Z value.
#define GO_PROFILE_FEATURE_TYPE_MIN_Z               (1)                     ///< Point with the minimum Z value.
#define GO_PROFILE_FEATURE_TYPE_MAX_X               (2)                     ///< Point with the maximum X value.
#define GO_PROFILE_FEATURE_TYPE_MIN_X               (3)                     ///< Point with the minimum X value.
#define GO_PROFILE_FEATURE_TYPE_CORNER              (4)                     ///< Dominant corner.
#define GO_PROFILE_FEATURE_TYPE_AVERAGE             (5)                     ///< Average of points.
#define GO_PROFILE_FEATURE_TYPE_RISING_EDGE         (6)                     ///< Rising edge.
#define GO_PROFILE_FEATURE_TYPE_FALLING_EDGE        (7)                     ///< Falling edge.
#define GO_PROFILE_FEATURE_TYPE_ANY_EDGE            (8)                     ///< Rising or falling edge.
#define GO_PROFILE_FEATURE_TYPE_TOP_CORNER          (9)                     ///< Top-most corner.
#define GO_PROFILE_FEATURE_TYPE_BOTTOM_CORNER       (10)                    ///< Bottom-most corner.
#define GO_PROFILE_FEATURE_TYPE_LEFT_CORNER         (11)                    ///< Left-most corner.
#define GO_PROFILE_FEATURE_TYPE_RIGHT_CORNER        (12)                    ///< Right-most corner.
#define GO_PROFILE_FEATURE_TYPE_MEDIAN              (13)                    ///< Median of points.
/**@}*/

/**
 * @struct  GoProfileGapAxis
 * @note    Supported with G1, G2
 * @ingroup GoSdk-ProfileTools
 * @brief   Represents a profile gap measurement axis.
 *
 * The following enumerators are defined:
 * - #GO_PROFILE_GAP_AXIS_EDGE
 * - #GO_PROFILE_GAP_AXIS_SURFACE
 * - #GO_PROFILE_GAP_AXIS_DISTANCE
 */
typedef k32s GoProfileGapAxis;
/** @name    GoProfileGapAxis
 *@{*/
#define GO_PROFILE_GAP_AXIS_EDGE                    (0)                     ///< Measure the gap along the edge normal.
#define GO_PROFILE_GAP_AXIS_SURFACE                 (1)                     ///< Measure the gap along the surface line.
#define GO_PROFILE_GAP_AXIS_DISTANCE                (2)                     ///< Measure the shortest distance between the two edges.
/**@}*/

/**
 * @struct  GoProfileEdgeType
 * @note    Supported with G1, G2
 * @ingroup GoSdk-ProfileTools
 * @brief   Represents a profile edge type.
 *
 * The following enumerators are defined:
 * - #GO_PROFILE_EDGE_TYPE_TANGENT
 * - #GO_PROFILE_EDGE_TYPE_CORNER
 */
typedef k32s GoProfileEdgeType;
/** @name    GoProfileEdgeType
 *@{*/
#define GO_PROFILE_EDGE_TYPE_TANGENT                (0)                     ///< Detect the edge by looking for the tangent.
#define GO_PROFILE_EDGE_TYPE_CORNER                 (1)                     ///< Detect the edge by looking for the corner.
/**@}*/

/**
 * @struct  GoProfileBaseline
 * @note    Supported with G1, G2
 * @ingroup GoSdk-ProfileTools
 * @brief   Determines whether to use a line based on a Profile Line fit, or based on the x-axis.
 *
 * The following enumerators are defined:
 * - #GO_PROFILE_BASELINE_TYPE_X_AXIS
 * - #GO_PROFILE_BASELINE_TYPE_Z_AXIS
 * - #GO_PROFILE_BASELINE_TYPE_LINE
 */
typedef k32s GoProfileBaseline;
/** @name    GoProfileBaseline
 *@{*/
#define GO_PROFILE_BASELINE_TYPE_X_AXIS                 (0)             ///< Use the X-Axis.
#define GO_PROFILE_BASELINE_TYPE_Z_AXIS                 (1)             ///< Use the Z-Axis.
#define GO_PROFILE_BASELINE_TYPE_LINE                   (2)             ///< Use the line fit.
/**@}*/

/**
 * @struct  GoProfileAreaType
 * @note    Supported with G1, G2
 * @ingroup GoSdk-ProfileTools
 * @brief   Determines how to calculate profile area
 *
 * The following enumerators are defined:
 * - #GO_PROFILE_AREA_TYPE_OBJECT
 * - #GO_PROFILE_AREA_TYPE_CLEARANCE
 */
typedef k32s GoProfileAreaType;
/** @name    GoProfileAreaType
 *@{*/
#define GO_PROFILE_AREA_TYPE_OBJECT                 (0)                     ///< Sum the profile area that is above the line.
#define GO_PROFILE_AREA_TYPE_CLEARANCE              (1)                     ///< Sum the profile area that is below the line.
/**@}*/

/**
 * @struct  GoProfileGapSide
 * @note    Supported with G1, G2
 * @ingroup GoSdk-ProfileTools
 * @brief   Selects which edge to use as the reference in a panel tool.
 *
 * The following enumerators are defined:
 * - #GO_PROFILE_PANEL_SIDE_LEFT
 * - #GO_PROFILE_PANEL_SIDE_RIGHT
 */
typedef k32s GoProfilePanelSide;
/** @name    GoProfilePanelSide
 *@{*/
#define GO_PROFILE_PANEL_SIDE_LEFT                  (0)                     ///< Use the left edge.
#define GO_PROFILE_PANEL_SIDE_RIGHT                 (1)                     ///< Use the right edge.
/**@}*/

/**
 * @struct  GoProfileRoundCornerDirection
 * @note    Supported with G1, G2
 * @ingroup GoSdk-ProfileTools
 * @brief   Selects which reference direction to use for the round corner tool.
 *
 * The following enumerators are defined:
 * - #GO_PROFILE_ROUND_CORNER_DIRECTION_LEFT
 * - #GO_PROFILE_ROUND_CORNER_DIRECTION_RIGHT
 */

typedef k32s GoProfileRoundCornerDirection;
/** @name    GoProfileRoundCornerDirection
 *@{*/
#define GO_PROFILE_ROUND_CORNER_DIRECTION_LEFT              (0)                     ///< Use the left edge.
#define GO_PROFILE_ROUND_CORNER_DIRECTION_RIGHT             (1)                     ///< Use the right edge.
/**@}*/

/**
 * @struct  GoProfileGrooveShape
 * @note    Supported with G1, G2
 * @ingroup GoSdk-ProfileTools
 * @brief   Represents a profile edge type.
 *
 * The following enumerators are defined:
 * - #GO_PROFILE_GROOVE_SHAPE_U
 * - #GO_PROFILE_GROOVE_SHAPE_V
 * - #GO_PROFILE_GROOVE_SHAPE_OPEN
 */
typedef k32s GoProfileGrooveShape;
/** @name    GoProfileGrooveShape
 *@{*/
#define GO_PROFILE_GROOVE_SHAPE_U                   (0)                     ///< Detect grooves that are U shaped.
#define GO_PROFILE_GROOVE_SHAPE_V                   (1)                     ///< Detect grooves that are V shaped.
#define GO_PROFILE_GROOVE_SHAPE_OPEN                (2)                     ///< Detect grooves that are open.
/**@}*/

/**
 * @struct  GoProfileGrooveSelectType
 * @note    Supported with G1, G2
 * @ingroup GoSdk-ProfileTools
 * @brief   Determines which groove to select when multiple are present.
 *
 * The following enumerators are defined:
 * - #GO_PROFILE_GROOVE_SELECT_TYPE_MAX_DEPTH
 * - #GO_PROFILE_GROOVE_SELECT_TYPE_LEFT_INDEX
 * - #GO_PROFILE_GROOVE_SELECT_TYPE_RIGHT_INDEX
 */
typedef k32s GoProfileGrooveSelectType;
/** @name    GoProfileGrooveSelectType
 *@{*/
#define GO_PROFILE_GROOVE_SELECT_TYPE_MAX_DEPTH           (0)             ///< Select the groove with the maximum depth.
#define GO_PROFILE_GROOVE_SELECT_TYPE_LEFT_INDEX          (1)             ///< Select the groove with the currently selected index starting from the left side.
#define GO_PROFILE_GROOVE_SELECT_TYPE_RIGHT_INDEX         (2)             ///< Select the groove with the currently selected index starting from the right side.
/**@}*/

/**
 * @struct  GoProfileGrooveLocation
 * @note    Supported with G1, G2
 * @ingroup GoSdk-ProfileTools
 * @brief   Determines which groove position to return.
 *
 * The following enumerators are defined:
 * - #GO_PROFILE_GROOVE_LOCATION_BOTTOM
 * - #GO_PROFILE_GROOVE_LOCATION_LEFT
 * - #GO_PROFILE_GROOVE_LOCATION_RIGHT
 */
typedef k32s GoProfileGrooveLocation;
/** @name    GoProfileGrooveLocation
 *@{*/
#define GO_PROFILE_GROOVE_LOCATION_BOTTOM           (0)                     ///< Return the position of the bottom of the groove.
#define GO_PROFILE_GROOVE_LOCATION_LEFT             (1)                     ///< Return the position of the left corner of the groove.
#define GO_PROFILE_GROOVE_LOCATION_RIGHT            (2)                     ///< Return the position of the right corner of the groove.
/**@}*/

/**
 * @struct  GoProfileStripSelectType
 * @note    Supported with G1, G2
 * @ingroup GoSdk-ProfileTools
 * @brief   Determines which Strip to select when multiple are present.
 *
 * The following enumerators are defined:
 * - #GO_PROFILE_STRIP_SELECT_TYPE_BEST
 * - #GO_PROFILE_STRIP_SELECT_TYPE_LEFT_INDEX
 * - #GO_PROFILE_STRIP_SELECT_TYPE_RIGHT_INDEX
 */
typedef k32s GoProfileStripSelectType;
/** @name    GoProfileStripSelectType
 *@{*/
#define GO_PROFILE_STRIP_SELECT_TYPE_BEST                (0)             ///< Select the best strip.
#define GO_PROFILE_STRIP_SELECT_TYPE_LEFT_INDEX          (1)             ///< Select the strip with the currently selected index starting from the left side.
#define GO_PROFILE_STRIP_SELECT_TYPE_RIGHT_INDEX         (2)             ///< Select the strip with the currently selected index starting from the right side.
/**@}*/

/**
 * @struct  GoProfileStripLocation
 * @note    Supported with G1, G2
 * @ingroup GoSdk-ProfileTools
 * @brief   Determines which Strip position to return.
 *
 * The following enumerators are defined:
 * - #GO_PROFILE_STRIP_LOCATION_LEFT
 * - #GO_PROFILE_STRIP_LOCATION_RIGHT
 * - #GO_PROFILE_STRIP_LOCATION_BOTTOM
 */
typedef k32s GoProfileStripLocation;
/** @name    GoProfileStripLocation
 *@{*/
#define GO_PROFILE_STRIP_LOCATION_LEFT             (0)         ///< Return the position of the left corner of the Strip.
#define GO_PROFILE_STRIP_LOCATION_RIGHT            (1)         ///< Return the position of the right corner of the Strip.
#define GO_PROFILE_STRIP_LOCATION_BOTTOM           (2)         ///< Return the position of the center of the Strip.
/**@}*/

/**
* @struct  GoProfileGenerationType
* @extends kValue
* @note    Supported with G1, G2
* @brief   Represents a profile generation type.
*
* The following enumerators are defined:
* - #GO_PROFILE_GENERATION_TYPE_CONTINUOUS
* - #GO_PROFILE_GENERATION_TYPE_FIXED_LENGTH
* - #GO_PROFILE_GENERATION_TYPE_VARIABLE_LENGTH
* - #GO_PROFILE_GENERATION_TYPE_ROTATIONAL
*/
typedef k32s GoProfileGenerationType;
/** @name    GoProfileGenerationType
*@{*/
#define GO_PROFILE_GENERATION_TYPE_CONTINUOUS           (0) ///< Continuous Profile generation.
#define GO_PROFILE_GENERATION_TYPE_FIXED_LENGTH         (1) ///< Fixed length Profile generation.
#define GO_PROFILE_GENERATION_TYPE_VARIABLE_LENGTH      (2) ///< Variable length Profile generation.
#define GO_PROFILE_GENERATION_TYPE_ROTATIONAL           (3) ///< Rotational Profile generation.
/**@}*/

/**
* @struct  GoProfileGenerationStartTrigger
* @extends kValue
* @note    Supported with G1, G2
* @ingroup GoSdk-Profile
* @brief   Represents a profile generation start trigger.
*
* The following enumerators are defined:
* - #GO_PROFILE_GENERATION_START_TRIGGER_SEQUENTIAL
* - #GO_PROFILE_GENERATION_START_TRIGGER_DIGITAL
*/
typedef k32s GoProfileGenerationStartTrigger;
/** @name    GoProfileGenerationStartTrigger
*@{*/
#define GO_PROFILE_GENERATION_START_TRIGGER_SEQUENTIAL  (0) ///< Sequential start trigger.
#define GO_PROFILE_GENERATION_START_TRIGGER_DIGITAL     (1) ///< Digital input start trigger.
/**@}*/

/**
 * @struct  GoPartFrameOfReference
 * @extends kValue
 * @note    Supported with G2, G3
 * @ingroup GoSdk
 * @brief   Represents a part detection frame of reference.
 *
 * The following enumerators are defined:
 * - #GO_PART_FRAME_OF_REFERENCE_TYPE_SENSOR
 * - #GO_PART_FRAME_OF_REFERENCE_TYPE_SCAN
 * - #GO_PART_FRAME_OF_REFERENCE_TYPE_PART
 */
typedef k32s GoPartFrameOfReference;
/** @name    GoPartFrameOfReference
 *@{*/
#define GO_PART_FRAME_OF_REFERENCE_TYPE_SENSOR                (0)   ///< Sensor frame of reference. 2x00 only.
#define GO_PART_FRAME_OF_REFERENCE_TYPE_SCAN                  (0)   ///< Scan frame of reference. 3x00 only. Value duplication is intentional.
#define GO_PART_FRAME_OF_REFERENCE_TYPE_PART                  (1)   ///< Part frame of reference.
/**@}*/

/**
 * @struct  GoPartHeightThresholdDirection
 * @extends kValue
 * @note    Supported with G2, G3
 * @ingroup GoSdk-Surface
 * @brief   Represents a part detection height threshold direction.
 *
 * The following enumerators are defined:
 * - #GO_PART_HEIGHT_THRESHOLD_DIRECTION_ABOVE
 * - #GO_PART_HEIGHT_THRESHOLD_DIRECTION_BELOW
 */
typedef k32s GoPartHeightThresholdDirection;
/** @name    GoPartHeightThresholdDirection
 *@{*/
#define GO_PART_HEIGHT_THRESHOLD_DIRECTION_ABOVE                 (0)    ///< Height threshold direction is above the Z-axis.
#define GO_PART_HEIGHT_THRESHOLD_DIRECTION_BELOW                 (1)    ///< Height threshold direction is below the Z-axis.
/**@}*/

/**
 * @struct  GoSurfaceGenerationType
 * @extends kValue
 * @note    Supported with G2, G3
 * @ingroup GoSdk-Surface
 * @brief   Represents a surface generation type.
 *
 * The following enumerators are defined:
 * - #GO_SURFACE_GENERATION_TYPE_CONTINUOUS
 * - #GO_SURFACE_GENERATION_TYPE_FIXED_LENGTH
 * - #GO_SURFACE_GENERATION_TYPE_VARIABLE_LENGTH
 * - #GO_SURFACE_GENERATION_TYPE_ROTATIONAL
 */
typedef k32s GoSurfaceGenerationType;
/** @name    GoSurfaceGenerationType
 *@{*/
#define GO_SURFACE_GENERATION_TYPE_CONTINUOUS           (0) ///< Continuous surface generation.
#define GO_SURFACE_GENERATION_TYPE_FIXED_LENGTH         (1) ///< Fixed length surface generation.
#define GO_SURFACE_GENERATION_TYPE_VARIABLE_LENGTH      (2) ///< Variable length surface generation.
#define GO_SURFACE_GENERATION_TYPE_ROTATIONAL           (3) ///< Rotational surface generation.
/**@}*/

/**
 * @struct  GoSurfaceGenerationStartTrigger
 * @extends kValue
 * @note    Supported with G2, G3
 * @ingroup GoSdk-Surface
 * @brief   Represents a surface generation start trigger.
 *
 * The following enumerators are defined:
 * - #GO_SURFACE_GENERATION_START_TRIGGER_SEQUENTIAL
 * - #GO_SURFACE_GENERATION_START_TRIGGER_DIGITAL
 * - #GO_SURFACE_GENERATION_START_TRIGGER_SOFTWARE
 */
typedef k32s GoSurfaceGenerationStartTrigger;
/** @name    GoSurfaceGenerationStartTrigger
 *@{*/
#define GO_SURFACE_GENERATION_START_TRIGGER_SEQUENTIAL  (0) ///< Sequential start trigger.
#define GO_SURFACE_GENERATION_START_TRIGGER_DIGITAL     (1) ///< Digital input start trigger.
#define GO_SURFACE_GENERATION_START_TRIGGER_SOFTWARE    (2) ///< Software start trigger.
 /**@}*/

/**
 * @struct  GoSurfaceLocation
 * @note    Supported with G2, G3
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents a surface location.
 *
 * The following enumerators are defined:
 * - #GO_SURFACE_LOCATION_TYPE_MAX
 * - #GO_SURFACE_LOCATION_TYPE_MIN
 * - #GO_SURFACE_LOCATION_TYPE_2D_CENTROID
 * - #GO_SURFACE_LOCATION_TYPE_3D_CENTROID
 * - #GO_SURFACE_LOCATION_TYPE_AVG
 * - #GO_SURFACE_LOCATION_TYPE_MEDIAN
 */
typedef k32s GoSurfaceLocation;
/** @name    GoSurfaceLocation
 *@{*/
#define GO_SURFACE_LOCATION_TYPE_MAX                (0)         ///< Location based on the maximum point.
#define GO_SURFACE_LOCATION_TYPE_MIN                (1)         ///< Location based on the minimum point.
#define GO_SURFACE_LOCATION_TYPE_2D_CENTROID        (2)         ///< Location based on a 2d centroid.
#define GO_SURFACE_LOCATION_TYPE_3D_CENTROID        (3)         ///< Location based on a 3d centroid.
#define GO_SURFACE_LOCATION_TYPE_AVG                (4)         ///< Location based on the average point.
#define GO_SURFACE_LOCATION_TYPE_MEDIAN             (5)         ///< Location based on the median point.
/**@}*/

/**
 * @struct  GoSurfaceFeatureType
 * @note    Supported with G2, G3
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents a surface feature type.
 *
 * The following enumerators are defined:
 * - #GO_SURFACE_FEATURE_TYPE_AVERAGE
 * - #GO_SURFACE_FEATURE_TYPE_CENTROID
 * - #GO_SURFACE_FEATURE_TYPE_X_MAX
 * - #GO_SURFACE_FEATURE_TYPE_X_MIN
 * - #GO_SURFACE_FEATURE_TYPE_Y_MAX
 * - #GO_SURFACE_FEATURE_TYPE_Y_MIN
 * - #GO_SURFACE_FEATURE_TYPE_Z_MAX
 * - #GO_SURFACE_FEATURE_TYPE_Z_MIN
 * - #GO_SURFACE_FEATURE_TYPE_MEDIAN
 */
typedef k32s GoSurfaceFeatureType;
/** @name    GoSurfaceFeatureType
 *@{*/
#define GO_SURFACE_FEATURE_TYPE_AVERAGE            (0)          ///< Feature based on the average.
#define GO_SURFACE_FEATURE_TYPE_CENTROID           (1)          ///< Feature based on the centroid.
#define GO_SURFACE_FEATURE_TYPE_X_MAX              (2)          ///< Feature based on the X maximum point.
#define GO_SURFACE_FEATURE_TYPE_X_MIN              (3)          ///< Feature based on the X minimum point.
#define GO_SURFACE_FEATURE_TYPE_Y_MAX              (4)          ///< Feature based on the Y maximum point.
#define GO_SURFACE_FEATURE_TYPE_Y_MIN              (5)          ///< Feature based on the Y minimum point.
#define GO_SURFACE_FEATURE_TYPE_Z_MAX              (6)          ///< Feature based on the Z maximum point.
#define GO_SURFACE_FEATURE_TYPE_Z_MIN              (7)          ///< Feature based on the Z minimum point.
#define GO_SURFACE_FEATURE_TYPE_MEDIAN             (8)          ///< Feature based on the median.
/**@}*/

/**
 * @struct  GoSurfaceCountersunkHoleShape
 * @extends kValue
 * @note    Supported with G2, G3
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents a surface countersunk hole tool shape.
 *
 * The following enumerators are defined:
 * - #GO_SURFACE_COUNTERSUNK_HOLE_SHAPE_CONE
 * - #GO_SURFACE_COUNTERSUNK_HOLE_SHAPE_COUNTERBORE
 */
typedef k32s GoSurfaceCountersunkHoleShape;
/** @name    GoSurfaceCountersunkHoleShape
 *@{*/
#define GO_SURFACE_COUNTERSUNK_HOLE_SHAPE_CONE           (0) ///< Cone shape.
#define GO_SURFACE_COUNTERSUNK_HOLE_SHAPE_COUNTERBORE    (1) ///< Counterbore shape.
/**@}*/


/**
 * @struct  GoSurfaceOpeningType
 * @extends kValue
 * @note    Supported with G2, G3
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents a surface opening tool type.
 *
 * The following enumerators are defined:
 * - #GO_SURFACE_OPENING_TYPE_ROUNDED_SLOT
 * - #GO_SURFACE_OPENING_TYPE_ROUNDED_RECTANGLE
 */
typedef k32s GoSurfaceOpeningType;
/** @name    GoSurfaceOpeningType
 *@{*/
#define GO_SURFACE_OPENING_TYPE_ROUNDED_SLOT            (0) ///< Rounded slot opening type.
#define GO_SURFACE_OPENING_TYPE_ROUNDED_RECTANGLE       (1) ///< Rectangular opening type.
/**@}*/

/**
* @struct  GoSurfaceRivetType
* @extends kValue
* @ingroup GoSdk-SurfaceTools
* @brief   Represents a surface rivet tool type.
*
* The following enumerators are defined:
* - #GO_SURFACE_RIVET_TYPE_FLUSH
* - #GO_SURFACE_RIVET_TYPE_RAISED
*/
typedef k32s GoSurfaceRivetType;
/** @name    GoSurfaceRivetType
*@{*/
#define GO_SURFACE_RIVET_TYPE_FLUSH                   (0) ///< Flush rivet type.
#define GO_SURFACE_RIVET_TYPE_RAISED                  (1) ///< Raised rivet type.
/**@}*/

/**
 * @struct  GoPartMatchAlgorithm
 * @extends kValue
 * @note    Supported with G2, G3
 * @ingroup GoSdk-Surface
 * @brief   Represents a part matching algorithm.
 *
 * The following enumerators are defined:
 * - #GO_PART_MATCH_ALGORITHM_EDGE
 * - #GO_PART_MATCH_ALGORITHM_BOUNDING_BOX
 * - #GO_PART_MATCH_ALGORITHM_ELLIPSE
 */
typedef k32s GoPartMatchAlgorithm;
/** @name    GoPartMatchAlgorithm
 *@{*/
#define GO_PART_MATCH_ALGORITHM_EDGE                      (0)   ///< Edge based part match algorithm.
#define GO_PART_MATCH_ALGORITHM_BOUNDING_BOX              (1)   ///< Bounding box based part match algorithm.
#define GO_PART_MATCH_ALGORITHM_ELLIPSE                   (2)   ///< Ellipse based part match algorithm.
/**@}*/


/**
 * @struct  GoBoxAsymmetryType
 * @extends kValue
 * @note    Supported with G2, G3
 * @ingroup GoSdk-Surface
 * @brief   Represents the bounding box part matching asymmetry detection type.
 *
 * The following enumerators are defined:
 * - #GO_BOX_ASYMMETRY_TYPE_NONE
 * - #GO_BOX_ASYMMETRY_TYPE_ALONG_LENGTH_AXIS
 * - #GO_BOX_ASYMMETRY_TYPE_ALONG_WIDTH_AXIS
 */
typedef k32s GoBoxAsymmetryType;
/** @name    GoBoxAsymmetryType
 *@{*/
#define GO_BOX_ASYMMETRY_TYPE_NONE                  (0)   ///< None
#define GO_BOX_ASYMMETRY_TYPE_ALONG_LENGTH_AXIS     (1)   ///< Along Length axis
#define GO_BOX_ASYMMETRY_TYPE_ALONG_WIDTH_AXIS      (2)   ///< Along Width axis
/**@}*/

/**
 * @struct  GoEllipseAsymmetryType
 * @extends kValue
 * @note    Supported with G2, G3
 * @ingroup GoSdk-Surface
 * @brief   Represents the bounding Ellipse part matching asymmetry detection type.
 *
 * The following enumerators are defined:
 * - #GO_ELLIPSE_ASYMMETRY_TYPE_NONE
 * - #GO_ELLIPSE_ASYMMETRY_TYPE_ALONG_MAJOR_AXIS
 * - #GO_ELLIPSE_ASYMMETRY_TYPE_ALONG_MINOR_AXIS
 */
typedef k32s GoEllipseAsymmetryType;
/** @name    GoEllipseAsymmetryType
 *@{*/
#define GO_ELLIPSE_ASYMMETRY_TYPE_NONE                  (0)   ///< None
#define GO_ELLIPSE_ASYMMETRY_TYPE_ALONG_MAJOR_AXIS      (1)   ///< Along Major axis
#define GO_ELLIPSE_ASYMMETRY_TYPE_ALONG_MINOR_AXIS      (2)   ///< Along Minor axis
/**@}*/


#define GO_SURFACE_COUNTERSUNK_HOLE_MAX_REF_REGIONS     (2)     ///< The maximum number of reference regions permitted for the Surface Counter Sunk Hole Tool.
#define GO_SURFACE_HOLE_MAX_REF_REGIONS                 (2)     ///< The maximum number of reference regions permitted for the Surface Hole Tool.
#define GO_SURFACE_OPENING_MAX_REF_REGIONS              (2)     ///< The maximum number of reference regions permitted for the Surface Opening Tool.
#define GO_SURFACE_PLANE_MAX_REGIONS                    (4)     ///< The maximum number of reference regions permitted for the Surface Plane Tool.
#define GO_SURFACE_RIVET_MAX_REF_REGIONS                (2)     ///< The maximum number of reference regions permitted for the Surface Rivet Tool.
#define GO_SURFACE_STUD_MAX_REF_REGIONS                 (2)     ///< The maximum number of reference regions permitted for the Surface Stud Tool.

/**
 * @struct  GoImageType
 * @extends kValue
 * @note    Supported with G2, G3
 * @ingroup GoSdk
 * @brief   Represents an image type.
 *
 * The following enumerators are defined:
 * - #GO_IMAGE_TYPE_HEIGHTMAP
 * - #GO_IMAGE_TYPE_INTENSITY
 */
typedef k32s GoImageType;
/** @name    GoImageType
 *@{*/
#define GO_IMAGE_TYPE_HEIGHTMAP                         (0)     ///< Heightmap image type.
#define GO_IMAGE_TYPE_INTENSITY                         (1)     ///< Intensity image type.
/**@}*/

/**
 * @struct  GoSurfaceEncoding
 * @extends kValue
 * @note    Supported with G2, G3
 * @ingroup GoSdk
 * @brief   Represents a surface scanning engine encoding type
 *
 * The following enumerators are defined:
 * - #GO_SURFACE_ENCODING_STANDARD
 * - #GO_SURFACE_ENCODING_INTERREFLECTION
 */
typedef k32s GoSurfaceEncoding;
/** @name    GoSurfaceEncoding
 *@{*/
#define GO_SURFACE_ENCODING_STANDARD          (0) ///< Standard Phase Encoding
#define GO_SURFACE_ENCODING_INTERREFLECTION   (1) ///< Interreflection Encoding (Advanced Users Only)
/**@}*/

/**
 * @struct  GoSurfacePhaseFilter
 * @extends kValue
 * @note    Supported with G2, G3
 * @ingroup GoSdk
 * @brief   Represents a surface phase filter type
 *
 * The following enumerators are defined:
 * - #GO_SURFACE_PHASE_FILTER_NONE
 * - #GO_SURFACE_PHASE_FILTER_REFLECTIVE
 * - #GO_SURFACE_PHASE_FILTER_TRANSLUCENT
 */
typedef k32s GoSurfacePhaseFilter;
/** @name    GoSurfacePhaseFilter
 *@{*/
#define GO_SURFACE_PHASE_FILTER_NONE        (0) ///< Standard
#define GO_SURFACE_PHASE_FILTER_REFLECTIVE  (1) ///< Reflective Phase Filters
#define GO_SURFACE_PHASE_FILTER_TRANSLUCENT (2) ///< Translucent Phase Filters
/**@}*/

/**
 * @struct  GoGammaType
 * @extends kValue
 * @ingroup GoSdk
 * @brief   Represents an advanced gamma type.
 *
 * The following enumerators are defined:
 * - #GO_GAMMA_TYPE_NONE
 * - #GO_GAMMA_TYPE_LOW
 * - #GO_GAMMA_TYPE_MEDIUM
 * - #GO_GAMMA_TYPE_HIGH
 */
typedef k32s GoGammaType;
/** @name    GoGammaType
 *@{*/
#define GO_GAMMA_TYPE_NONE                              (0)     ///< None. No imager gamma / multi-slope configuration will occur.
#define GO_GAMMA_TYPE_LOW                               (1)     ///< Low.
#define GO_GAMMA_TYPE_MEDIUM                            (2)     ///< Medium.
#define GO_GAMMA_TYPE_HIGH                              (3)     ///< High.
/**@}*/

/**
 * @struct  GoPatternSequenceType
 * @extends kValue
 * @ingroup GoSdk
 * @brief   Represents a pattern sequence type.
 *
 * The following enumerators are defined:
 * - #GO_PATTERN_SEQUENCE_TYPE_DEFAULT
 * - #GO_PATTERN_SEQUENCE_TYPE_CUSTOM
 *
 */
typedef k32s GoPatternSequenceType;
/** @name    GoPatternSequenceType
 *@{*/

#define GO_PATTERN_SEQUENCE_TYPE_DEFAULT            (0)     ///< Default sequence pattern.
#define GO_PATTERN_SEQUENCE_TYPE_CUSTOM             (100)   ///< Custom sequence pattern.
#define GO_PATTERN_SEQUENCE_TYPE_FOCUS              (101)   ///< Focus pattern (G3506 only).
#define GO_PATTERN_SEQUENCE_TYPE_STANDARD_SEQUENCE  (102)   ///< Standard sequence pattern (G3 only).
#define GO_PATTERN_SEQUENCE_TYPE_PROJECTOR_OFF      (103)   ///< Pattern with disabled LED light (G3 only).
/**@}*/

#define GO_PATTERN_SEQUENCE_TYPE_FOCUS_AID              (101)   ///< @deprecated use GO_PATTERN_SEQUENCE_TYPE_FOCUS instead

/**
 * @struct  GoImplicitTriggerOverride
 * @extends kValue
 * @ingroup GoSdk
 * @brief   Represents an EthernetIP implicit messaging trigger override.
 *
 * The following enumerators are defined:
 * - #GO_IMPLICIT_TRIGGER_OVERRIDE_OFF
 * - #GO_IMPLICIT_TRIGGER_OVERRIDE_CYCLIC
 * - #GO_IMPLICIT_TRIGGER_OVERRIDE_CHANGE_OF_STATE
 *
 */
typedef k32s GoImplicitTriggerOverride;
/** @name    GoImplicitTriggerOverride
 *@{*/

#define GO_IMPLICIT_TRIGGER_OVERRIDE_OFF                (0)     ///< Use the implicit output trigger specified in the connection header.
#define GO_IMPLICIT_TRIGGER_OVERRIDE_CYCLIC             (1)     ///< Utilize cyclic implicit messaging trigger behavior regardless of what is specified in the connection header.
#define GO_IMPLICIT_TRIGGER_OVERRIDE_CHANGE_OF_STATE    (2)     ///< Utilize change of state implicit messaging trigger behavior regardless of what is specified in the connection header.
/**@}*/

/**
 * @struct  GoAlignmentStatus
 * @extends kValue
 * @ingroup GoSdk
 * @brief   Represents the operation status of an alignment.
 *
 * The following enumerators are defined:
 * - #GO_ALIGNMENT_STATUS_OK
 * - #GO_ALIGNMENT_STATUS_GENERAL_FAILURE
 * - #GO_ALIGNMENT_STATUS_STATIONARY_NO_DATA
 * - #GO_ALIGNMENT_STATUS_MOVING_INSUFFICIENT_DATA
 * - #GO_ALIGNMENT_STATUS_INVALID_TARGET
 * - #GO_ALIGNMENT_STATUS_UNEXPECTED_TARGET_POSITION
 * - #GO_ALIGNMENT_STATUS_BAR_HOLE_NOT_FOUND
 * - #GO_ALIGNMENT_STATUS_MOVING_NO_ENCODER_CHANGE
 * - #GO_ALIGNMENT_STATUS_ABORT
 * - #GO_ALIGNMENT_STATUS_TIMEOUT
 * - #GO_ALIGNMENT_STATUS_INVALID_PARAMETER
 *
 */
typedef k32s GoAlignmentStatus;
/** @name    GoAlignmentStatus
 *@{*/

#define GO_ALIGNMENT_STATUS_OK                              (1)                 ///< Alignment operation succeeded.
#define GO_ALIGNMENT_STATUS_GENERAL_FAILURE                 (0)                 ///< Alignment operation failed.
#define GO_ALIGNMENT_STATUS_STATIONARY_NO_DATA              (-1)                ///< Stationary alignment failed due to no data being received. Please ensure the target is in range.
#define GO_ALIGNMENT_STATUS_MOVING_INSUFFICIENT_DATA        (-2)                ///< Moving alignment failed due to insufficient data.
#define GO_ALIGNMENT_STATUS_INVALID_TARGET                  (-3)                ///< Invalid target detected. Examples include the target dimensions being too small, the target touches both sides of the field of view, or there is insufficient data after some internal filtering.
#define GO_ALIGNMENT_STATUS_UNEXPECTED_TARGET_POSITION      (-4)                ///< Target detected in an unexpected position. Please ensure the target is stable and there are no obstructions.
#define GO_ALIGNMENT_STATUS_BAR_HOLE_NOT_FOUND              (-5)                ///< No reference hole was found during bar alignment. Please ensure the holes can be seen and that the target parameters match their physical dimensions.
#define GO_ALIGNMENT_STATUS_MOVING_NO_ENCODER_CHANGE        (-6)                ///< No change in encoder value occurred during moving alignment. Please ensure the encoder is connected and the target is moving.
#define GO_ALIGNMENT_STATUS_ABORT                           (kERROR_ABORT)      ///< The alignment was aborted by the user.
#define GO_ALIGNMENT_STATUS_TIMEOUT                         (kERROR_TIMEOUT)    ///< The alignment timed out.
#define GO_ALIGNMENT_STATUS_INVALID_PARAMETER               (kERROR_PARAMETER)  ///< The alignment failed due to incorrected parameters.
/**@}*/

typedef struct GoFeatureOption
{
    kText64 name;
    kSize minCount;
    kSize maxCount;
    GoFeatureDataType dataType;
    kText64 type;
} GoFeatureOption;

typedef struct GoMeasurementOption
{
    kText64 name;
    kSize minCount;
    kSize maxCount;
} GoMeasurementOption;

typedef struct GoToolDataOutputOption
{
    kText64 name;
    kText64 type;
    GoDataType dataType;
    kSize minCount;
    kSize maxCount;
} GoToolDataOutputOption;

/**
 * @struct  GoEventType
 * @extends kValue
 * @ingroup GoSdk
 * @brief   Represents the event type represented by an event message.
 *
 * The following enumerator is defined:
 * - GO_EVENT_TYPE_EXPOSURE_END
 *
 */
typedef k32s GoEventType;
/** @name    GoEventType
 *@{*/

#define GO_EVENT_TYPE_EXPOSURE_END                  (1)
/**@}*/

/**
* @struct  GoOcclusionReductionAlg
* @extends kValue
* @ingroup GoSdk
* @brief   Represents an occlusion reduction algorithm.
*
* The following enumerators are defined:
* - #GO_OCCLUSION_REDUCTION_NORMAL
* - #GO_OCCLUSION_REDUCTION_HIGH_QUALITY
*/
typedef k32s GoOcclusionReductionAlg;
/** @name    GoOcclusionReductionAlg
*@{*/
#define GO_OCCLUSION_REDUCTION_NORMAL               (0)         ///< Basic occlusion reduction.
#define GO_OCCLUSION_REDUCTION_HIGH_QUALITY         (1)         ///< High quality occlusion reduction.
/**@}*/

/**
* @struct  GoDemosaicStyle
* @extends kValue
* @ingroup GoSdk
* @brief   Represents a Bayer demosaic algorithm style.
*
* The following enumerators are defined:
* - #GO_DEMOSAIC_STYLE_REDUCE
* - #GO_DEMOSAIC_STYLE_BILINEAR
* - #GO_DEMOSAIC_STYLE_GRADIENT
*/
typedef k32s GoDemosaicStyle;
/** @name    GoDemosaicStyle
*@{*/
#define GO_DEMOSAIC_STYLE_REDUCE                    (0)     ///< Simple Reduce (Shrinks image width and height by a factor of 2)
#define GO_DEMOSAIC_STYLE_BILINEAR                  (1)     ///< Bilinear demosaic (Same size output)
#define GO_DEMOSAIC_STYLE_GRADIENT                  (2)     ///< Gradient demosaic (Same size output)
/**@}*/

/**
* @struct  GoDiscoveryOpMode
* @extends kValue
* @ingroup GoSdk-Discovery
* @brief   Represents operational mode of the main controller responding
*          to the discovery protocol.
*
* The following enumerators are defined:
* - #GO_DISCOVERY_OP_MODE_NOT_AVAILABLE
* - #GO_DISCOVERY_OP_MODE_STANDALONE
* - #GO_DISCOVERY_OP_MODE_VIRTUAL
* - #GO_DISCOVERY_OP_MODE_ACCELERATOR
*/
typedef k8u GoDiscoveryOpMode;
/** @name    GoDiscoveryOpMode
*@{*/
#define GO_DISCOVERY_OP_MODE_NOT_AVAILABLE  (0)     ///< Not provided by sensor
#define GO_DISCOVERY_OP_MODE_STANDALONE     (1)     ///< Sensor is running standalone
#define GO_DISCOVERY_OP_MODE_VIRTUAL        (2)     ///< Sensor is a virtual sensor
#define GO_DISCOVERY_OP_MODE_ACCELERATOR    (3)     ///< Sensor is accelerated
/**@}*/

#define GO_MESH_MSG_NUM_OF_SYSTEM_CHANNEL 6
#define GO_MESH_MSG_NUM_OF_MAX_USER_CHANNEL 5

typedef k32s GoMeshMsgChannelId;

#define GO_MESH_MSG_CHANNEL_ID_SYSTEM_VERTEX             (0)
#define GO_MESH_MSG_CHANNEL_ID_SYSTEM_FACET              (1)
#define GO_MESH_MSG_CHANNEL_ID_SYSTEM_FACET_NORMAL       (2)
#define GO_MESH_MSG_CHANNEL_ID_SYSTEM_VERTEX_NORMAL      (3)
#define GO_MESH_MSG_CHANNEL_ID_SYSTEM_VERTEX_TEXTURE     (4)
#define GO_MESH_MSG_CHANNEL_ID_SYSTEM_VERTEX_CURVATURE   (5)

typedef k32u GoMeshMsgChannelType;

#define GO_MESH_MSG_CHANNEL_TYPE_INVALID            (0)
#define GO_MESH_MSG_CHANNEL_TYPE_VERTEX             (1)
#define GO_MESH_MSG_CHANNEL_TYPE_FACET              (2)
#define GO_MESH_MSG_CHANNEL_TYPE_FACET_NORMAL       (3)
#define GO_MESH_MSG_CHANNEL_TYPE_VERTEX_NORMAL      (4)
#define GO_MESH_MSG_CHANNEL_TYPE_VERTEX_TEXTURE     (5)
#define GO_MESH_MSG_CHANNEL_TYPE_VERTEX_CURVATURE   (6)

typedef k32s GoMeshMsgChannelState;

#define GO_MESH_MSG_CHANNEL_STATE_ERROR         (-1)
#define GO_MESH_MSG_CHANNEL_STATE_UNALLOCATED   (0)
#define GO_MESH_MSG_CHANNEL_STATE_ALLOCATED     (1)
#define GO_MESH_MSG_CHANNEL_STATE_EMPTY         (2)
#define GO_MESH_MSG_CHANNEL_STATE_PARTIAL       (3)
#define GO_MESH_MSG_CHANNEL_STATE_FULL          (4)

typedef struct GoFacet32u
{
    k32u vertex1;    //  Index of first vertex
    k32u vertex2;    //  Index of second vertex
    k32u vertex3;    //  Index of third vertex
} GoFacet32u; 

typedef struct Go3dTransform64f
{
    k64f xx;
    k64f xy;
    k64f xz;
    k64f xt;

    k64f yx;
    k64f yy;
    k64f yz;
    k64f yt;

    k64f zx;
    k64f zy;
    k64f zz;
    k64f zt;
} Go3dTransform64f;

typedef struct GoMeshMsgChannel
{
    GoMeshMsgChannelId id;          //  channel id
    GoMeshMsgChannelType type;      //  Type of channel
    GoMeshMsgChannelState state;    //  State of channel
    k32u flag;                      //  User specified channel flag
    kSize allocatedCount;           //  Allocated buffer size
    kSize dataCount;                //  Actual used buffer size
    kType dataType;                 //  buffer data type
    kArray1 buffer;                 //  buffer, type of dataType
} GoMeshMsgChannel;

#include <GoSdk/GoSdkDef.x.h>

#endif

/**
 * @file    GoSdkDef.c
 *
 * @internal
 * Copyright (C) 2017-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <GoSdk/GoSdkDef.h>
#include <GoSdk/GoSdkLib.x.h>
#include <stdio.h>

kBeginEnumEx(Go, GoDeviceState)
    kAddEnumerator(GoDeviceState, GO_DEVICE_STATE_CONFLICT)
    kAddEnumerator(GoDeviceState, GO_DEVICE_STATE_READY)
    kAddEnumerator(GoDeviceState, GO_DEVICE_STATE_RUNNING)
kEndEnumEx()

kBeginEnumEx(Go, GoUser)
    kAddEnumerator(GoUser, GO_USER_NONE)
    kAddEnumerator(GoUser, GO_USER_ADMIN)
    kAddEnumerator(GoUser, GO_USER_TECH)
kEndEnumEx()

kBeginEnumEx(Go, GoState)
    kAddEnumerator(GoState, GO_STATE_ONLINE)
    kAddEnumerator(GoState, GO_STATE_OFFLINE)
    kAddEnumerator(GoState, GO_STATE_RESETTING)
    kAddEnumerator(GoState, GO_STATE_INCOMPATIBLE)
    kAddEnumerator(GoState, GO_STATE_INCONSISTENT)
    kAddEnumerator(GoState, GO_STATE_UNRESPONSIVE)
    kAddEnumerator(GoState, GO_STATE_CANCELLED)
    kAddEnumerator(GoState, GO_STATE_INCOMPLETE)
    kAddEnumerator(GoState, GO_STATE_BUSY)
    kAddEnumerator(GoState, GO_STATE_READY)
    kAddEnumerator(GoState, GO_STATE_RUNNING)
    kAddEnumerator(GoState, GO_STATE_UPGRADING)
kEndEnumEx()

kBeginEnumEx(Go, GoBuddyState)
    kAddEnumerator(GoBuddyState, GO_BUDDY_STATE_ERROR)
    kAddEnumerator(GoBuddyState, GO_BUDDY_STATE_CONNECTING)
    kAddEnumerator(GoBuddyState, GO_BUDDY_STATE_CONNECTABLE)
    kAddEnumerator(GoBuddyState, GO_BUDDY_STATE_CONNECTED)
    kAddEnumerator(GoBuddyState, GO_BUDDY_STATE_ALREADY_BUDDIED)
    kAddEnumerator(GoBuddyState, GO_BUDDY_STATE_INVALID_STATE)
    kAddEnumerator(GoBuddyState, GO_BUDDY_STATE_VERSION_MISMATCH)
    kAddEnumerator(GoBuddyState, GO_BUDDY_STATE_MODEL_MISMATCH)
    kAddEnumerator(GoBuddyState, GO_BUDDY_STATE_UNREACHABLE_ADDRESS)
    kAddEnumerator(GoBuddyState, GO_BUDDY_STATE_DEVICE_MISSING)
    kAddEnumerator(GoBuddyState, GO_BUDDY_STATE_ERROR_CONNECTION)
    kAddEnumerator(GoBuddyState, GO_BUDDY_STATE_MAX_BUDDIES)
    kAddEnumerator(GoBuddyState, GO_BUDDY_STATE_STANDALONE_NOBUDDY)
    kAddEnumerator(GoBuddyState, GO_BUDDY_STATE_RESTRICTED_MISMATCH)
    kEndEnumEx()


GoFx(kBool) GoState_IsConnected(GoState state)
{
    return (state != GO_STATE_OFFLINE) &&
           (state != GO_STATE_ONLINE) &&
           (state != GO_STATE_RESETTING);
}

GoFx(kBool) GoState_IsResponsive(GoState state)
{
    return (state != GO_STATE_OFFLINE)      &&
           (state != GO_STATE_ONLINE)       &&
           (state != GO_STATE_RESETTING)    &&
           (state != GO_STATE_CANCELLED)    &&
           (state != GO_STATE_UNRESPONSIVE);
}

GoFx(kBool) GoState_IsReadable(GoState state)
{
    return (state == GO_STATE_INCOMPLETE)   ||
           (state == GO_STATE_READY)        ||
           (state == GO_STATE_RUNNING);
}

GoFx(kBool) GoState_IsConfigurable(GoState state)
{
    return (state == GO_STATE_READY);
}

GoFx(kBool) GoState_IsNormal(GoState state)
{
    return (state == GO_STATE_READY)        ||
           (state == GO_STATE_RUNNING);
}

GoFx(kBool) GoState_IsUpgrading(GoState state)
{
    return (state == GO_STATE_UPGRADING);
}

GoFx(kBool) GoState_ShouldRefresh(GoState state)
{
    return (state == GO_STATE_OFFLINE)      ||
           (state == GO_STATE_CANCELLED)    ||
           (state == GO_STATE_UNRESPONSIVE) ||
           (state == GO_STATE_INCONSISTENT);
}

kBeginEnumEx(Go, GoRole)
    kAddEnumerator(GoRole, GO_ROLE_MAIN)
    kAddEnumerator(GoRole, GO_ROLE_BUDDY)
kEndEnumEx()

kBeginEnumEx(Go, GoAcceleratorConnectionStatus)
    kAddEnumerator(GoAcceleratorConnectionStatus, GO_ACCELERATOR_CONNECTION_STATUS_CONNECTED)
    kAddEnumerator(GoAcceleratorConnectionStatus, GO_ACCELERATOR_CONNECTION_STATUS_DISCONNECTED)
    kAddEnumerator(GoAcceleratorConnectionStatus, GO_ACCELERATOR_CONNECTION_STATUS_ERROR)
kEndEnumEx()

kBeginEnumEx(Go, GoAlignmentStatus)
    kAddEnumerator(GoAlignmentStatus, GO_ALIGNMENT_STATUS_OK)
    kAddEnumerator(GoAlignmentStatus, GO_ALIGNMENT_STATUS_GENERAL_FAILURE)
    kAddEnumerator(GoAlignmentStatus, GO_ALIGNMENT_STATUS_STATIONARY_NO_DATA)
    kAddEnumerator(GoAlignmentStatus, GO_ALIGNMENT_STATUS_MOVING_INSUFFICIENT_DATA)
    kAddEnumerator(GoAlignmentStatus, GO_ALIGNMENT_STATUS_INVALID_TARGET)
    kAddEnumerator(GoAlignmentStatus, GO_ALIGNMENT_STATUS_UNEXPECTED_TARGET_POSITION)
    kAddEnumerator(GoAlignmentStatus, GO_ALIGNMENT_STATUS_BAR_HOLE_NOT_FOUND)
    kAddEnumerator(GoAlignmentStatus, GO_ALIGNMENT_STATUS_MOVING_NO_ENCODER_CHANGE)
    kAddEnumerator(GoAlignmentStatus, GO_ALIGNMENT_STATUS_ABORT)
    kAddEnumerator(GoAlignmentStatus, GO_ALIGNMENT_STATUS_TIMEOUT)
    kAddEnumerator(GoAlignmentStatus, GO_ALIGNMENT_STATUS_INVALID_PARAMETER)
kEndEnumEx()


kBeginEnumEx(Go, GoAlignmentRef)
    kAddEnumerator(GoAlignmentRef, GO_ALIGNMENT_REF_FIXED)
    kAddEnumerator(GoAlignmentRef, GO_ALIGNMENT_REF_DYNAMIC)
kEndEnumEx()

kBeginEnumEx(Go, GoAlignmentState)
    kAddEnumerator(GoAlignmentState, GO_ALIGNMENT_STATE_NOT_ALIGNED)
    kAddEnumerator(GoAlignmentState, GO_ALIGNMENT_STATE_ALIGNED)
kEndEnumEx()

kBeginEnumEx(Go, GoBoxAsymmetryType)
    kAddEnumerator(GoBoxAsymmetryType, GO_BOX_ASYMMETRY_TYPE_NONE)
    kAddEnumerator(GoBoxAsymmetryType, GO_BOX_ASYMMETRY_TYPE_ALONG_LENGTH_AXIS)
    kAddEnumerator(GoBoxAsymmetryType, GO_BOX_ASYMMETRY_TYPE_ALONG_WIDTH_AXIS)
kEndEnumEx()

kBeginEnumEx(Go, GoDecision)
    kAddEnumerator(GoDecision, GO_DECISION_FAIL)
    kAddEnumerator(GoDecision, GO_DECISION_PASS)
kEndEnumEx()

kBeginEnumEx(Go, GoDecisionCode)
    kAddEnumerator(GoDecisionCode, GO_DECISION_CODE_OK)
    kAddEnumerator(GoDecisionCode, GO_DECISION_CODE_INVALID_VALUE)
    kAddEnumerator(GoDecisionCode, GO_DECISION_CODE_INVALID_ANCHOR)
kEndEnumEx()

kBeginEnumEx(Go, GoEllipseAsymmetryType)
    kAddEnumerator(GoEllipseAsymmetryType, GO_ELLIPSE_ASYMMETRY_TYPE_NONE)
    kAddEnumerator(GoEllipseAsymmetryType, GO_ELLIPSE_ASYMMETRY_TYPE_ALONG_MAJOR_AXIS)
    kAddEnumerator(GoEllipseAsymmetryType, GO_ELLIPSE_ASYMMETRY_TYPE_ALONG_MINOR_AXIS)
kEndEnumEx()

kBeginEnumEx(Go, GoImageType)
    kAddEnumerator(GoImageType, GO_IMAGE_TYPE_HEIGHTMAP)
    kAddEnumerator(GoImageType, GO_IMAGE_TYPE_INTENSITY)
kEndEnumEx()

kBeginEnumEx(Go, GoGammaType)
    kAddEnumerator(GoGammaType, GO_GAMMA_TYPE_NONE)
    kAddEnumerator(GoGammaType, GO_GAMMA_TYPE_LOW)
    kAddEnumerator(GoGammaType, GO_GAMMA_TYPE_MEDIUM)
    kAddEnumerator(GoGammaType, GO_GAMMA_TYPE_HIGH)
kEndEnumEx()

kBeginEnumEx(Go, GoMode)
    kAddEnumerator(GoMode, GO_MODE_UNKNOWN)
    kAddEnumerator(GoMode, GO_MODE_VIDEO)
    kAddEnumerator(GoMode, GO_MODE_RANGE)
    kAddEnumerator(GoMode, GO_MODE_PROFILE)
    kAddEnumerator(GoMode, GO_MODE_SURFACE)
kEndEnumEx()

kBeginEnumEx(Go, GoOrientation)
    kAddEnumerator(GoOrientation, GO_ORIENTATION_WIDE)
    kAddEnumerator(GoOrientation, GO_ORIENTATION_OPPOSITE)
    kAddEnumerator(GoOrientation, GO_ORIENTATION_REVERSE)
kEndEnumEx()

kBeginEnumEx(Go, GoExposureMode)
    kAddEnumerator(GoExposureMode, GO_EXPOSURE_MODE_SINGLE)
    kAddEnumerator(GoExposureMode, GO_EXPOSURE_MODE_MULTIPLE)
    kAddEnumerator(GoExposureMode, GO_EXPOSURE_MODE_DYNAMIC)
kEndEnumEx()

kBeginEnumEx(Go, GoAlignmentTarget)
    kAddEnumerator(GoAlignmentTarget, GO_ALIGNMENT_TARGET_NONE)
    kAddEnumerator(GoAlignmentTarget, GO_ALIGNMENT_TARGET_BAR)
    kAddEnumerator(GoAlignmentTarget, GO_ALIGNMENT_TARGET_DISK)
    kAddEnumerator(GoAlignmentTarget, GO_ALIGNMENT_TARGET_PLATE)
    kAddEnumerator(GoAlignmentTarget, GO_ALIGNMENT_TARGET_POLYGON)
kEndEnumEx()

kBeginEnumEx(Go, GoAlignmentDegreesOfFreedom)
    kAddEnumerator(GoAlignmentDegreesOfFreedom, GO_ALIGNMENT_DOF_NONE)
    kAddEnumerator(GoAlignmentDegreesOfFreedom, GO_ALIGNMENT_3DOF_XZ_Y)
    kAddEnumerator(GoAlignmentDegreesOfFreedom, GO_ALIGNMENT_4DOF_XYZ_Y)
    kAddEnumerator(GoAlignmentDegreesOfFreedom, GO_ALIGNMENT_5DOF_XYZ_YZ)
    kAddEnumerator(GoAlignmentDegreesOfFreedom, GO_ALIGNMENT_6DOF_XYZ_XYZ)
kEndEnumEx()

kBeginEnumEx(Go, GoImplicitTriggerOverride)
    kAddEnumerator(GoImplicitTriggerOverride, GO_IMPLICIT_TRIGGER_OVERRIDE_OFF)
    kAddEnumerator(GoImplicitTriggerOverride, GO_IMPLICIT_TRIGGER_OVERRIDE_CYCLIC)
    kAddEnumerator(GoImplicitTriggerOverride, GO_IMPLICIT_TRIGGER_OVERRIDE_CHANGE_OF_STATE)
kEndEnumEx()

kBeginEnumEx(Go, GoIntensitySource)
kAddEnumerator(GoIntensitySource, GO_INTENSITY_SOURCE_BOTH)
kAddEnumerator(GoIntensitySource, GO_INTENSITY_SOURCE_FRONT)
kAddEnumerator(GoIntensitySource, GO_INTENSITY_SOURCE_BACK)
kEndEnumEx()

kBeginEnumEx(Go, GoIntensityMode)
kAddEnumerator(GoIntensityMode, GO_INTENSITY_MODE_AUTO)
kAddEnumerator(GoIntensityMode, GO_INTENSITY_MODE_PRESERVE_ORIGINAL)
kEndEnumEx()

kBeginEnumEx(Go, GoPatternSequenceType)
    kAddEnumerator(GoPatternSequenceType, GO_PATTERN_SEQUENCE_TYPE_DEFAULT)
    kAddEnumerator(GoPatternSequenceType, GO_PATTERN_SEQUENCE_TYPE_CUSTOM)
kEndEnumEx()


kBeginEnumEx(Go, GoDataSource)
    kAddEnumerator(GoDataSource, GO_DATA_SOURCE_NONE)
    kAddEnumerator(GoDataSource, GO_DATA_SOURCE_TOP)
    kAddEnumerator(GoDataSource, GO_DATA_SOURCE_TOP_LEFT)
    kAddEnumerator(GoDataSource, GO_DATA_SOURCE_TOP_RIGHT)
    kAddEnumerator(GoDataSource, GO_DATA_SOURCE_TOP_BOTTOM)
kEndEnumEx()

kBeginEnumEx(Go, GoDataStep)
    kAddEnumerator(GoDataStep, GO_DATA_STEP_NONE)
    kAddEnumerator(GoDataStep, GO_DATA_STEP_VIDEO)
    kAddEnumerator(GoDataStep, GO_DATA_STEP_RANGE)
    kAddEnumerator(GoDataStep, GO_DATA_STEP_PROFILE)
    kAddEnumerator(GoDataStep, GO_DATA_STEP_SURFACE)
    kAddEnumerator(GoDataStep, GO_DATA_STEP_SECTION)
kEndEnumEx()


kBeginEnumEx(Go, GoDigitalPass)
    kAddEnumerator(GoDigitalPass, GO_DIGITAL_PASS_TRUE)
    kAddEnumerator(GoDigitalPass, GO_DIGITAL_PASS_FALSE)
    kAddEnumerator(GoDigitalPass, GO_DIGITAL_PASS_ALWAYS)
kEndEnumEx()

kBeginEnumEx(Go, GoTrigger)
    kAddEnumerator(GoTrigger, GO_TRIGGER_TIME)
    kAddEnumerator(GoTrigger, GO_TRIGGER_ENCODER)
    kAddEnumerator(GoTrigger, GO_TRIGGER_INPUT)
    kAddEnumerator(GoTrigger, GO_TRIGGER_SOFTWARE)
kEndEnumEx()

kBeginEnumEx(Go, GoTriggerUnits)
    kAddEnumerator(GoTriggerUnits, GO_TRIGGER_UNIT_TIME)
    kAddEnumerator(GoTriggerUnits, GO_TRIGGER_UNIT_ENCODER)
kEndEnumEx()

kBeginEnumEx(Go, GoEncoderTriggerMode)
    kAddEnumerator(GoEncoderTriggerMode, GO_ENCODER_TRIGGER_MODE_TRACK_REVERSE)
    kAddEnumerator(GoEncoderTriggerMode, GO_ENCODER_TRIGGER_MODE_IGNORE_REVERSE)
    kAddEnumerator(GoEncoderTriggerMode, GO_ENCODER_TRIGGER_MODE_BIDIRECTIONAL)
kEndEnumEx()

kBeginEnumEx(Go, GoFrameRateMaxSource)
    kAddEnumerator(GoFrameRateMaxSource, GO_FRAME_RATE_MAX_SOURCE_CAMERA)
    kAddEnumerator(GoFrameRateMaxSource, GO_FRAME_RATE_MAX_SOURCE_PART_DETECTION)
kEndEnumEx()

kBeginEnumEx(Go, GoEncoderSpacingMinSource)
    kAddEnumerator(GoEncoderSpacingMinSource, GO_ENCODER_PERIOD_MAX_SOURCE_RESOLUTION)
    kAddEnumerator(GoEncoderSpacingMinSource, GO_ENCODER_PERIOD_MAX_SOURCE_PART_DETECTION)
kEndEnumEx()

kBeginEnumEx(Go, GoAnalogTrigger)
    kAddEnumerator(GoAnalogTrigger, GO_ANALOG_TRIGGER_MEASUREMENT)
    kAddEnumerator(GoAnalogTrigger, GO_ANALOG_TRIGGER_SOFTWARE)
kEndEnumEx()

kBeginEnumEx(Go, GoDigitalSignal)
    kAddEnumerator(GoDigitalSignal, GO_DIGITAL_SIGNAL_PULSED)
    kAddEnumerator(GoDigitalSignal, GO_DIGITAL_SIGNAL_CONTINUOUS)
kEndEnumEx()

kBeginEnumEx(Go, GoDigitalEvent)
    kAddEnumerator(GoDigitalEvent, GO_DIGITAL_EVENT_MEASUREMENT)
    kAddEnumerator(GoDigitalEvent, GO_DIGITAL_EVENT_SOFTWARE)
kEndEnumEx()

kBeginEnumEx(Go, GoAnalogEvent)
    kAddEnumerator(GoAnalogEvent, GO_ANALOG_EVENT_MEASURMENT)
    kAddEnumerator(GoAnalogEvent, GO_ANALOG_EVENT_SOFTWARE)
kEndEnumEx()

kBeginEnumEx(Go, GoEthernetProtocol)
    kAddEnumerator(GoEthernetProtocol, GO_ETHERNET_PROTOCOL_GOCATOR)
    kAddEnumerator(GoEthernetProtocol, GO_ETHERNET_PROTOCOL_MODBUS)
    kAddEnumerator(GoEthernetProtocol, GO_ETHERNET_PROTOCOL_ETHERNET_IP)
    kAddEnumerator(GoEthernetProtocol, GO_ETHERNET_PROTOCOL_ASCII)
    kAddEnumerator(GoEthernetProtocol, GO_ETHERNET_PROTOCOL_PROFINET)
    kAddEnumerator(GoEthernetProtocol, GO_ETHERNET_PROTOCOL_PTP)
    kEndEnumEx()

kBeginValueEx(Go, GoOutputCompositeSource)
    kAddField(GoOutputCompositeSource, k32s, id)
    kAddField(GoOutputCompositeSource, GoDataSource, dataSource)
    kAddVMethod(GoOutputCompositeSource, kValue, VEquals)
kEndValueEx()

kBeginValueEx(Go, GoDataStream)
    kAddField(GoDataStream, GoDataStep, step)
    kAddField(GoDataStream, k32s, id)
kEndValueEx()

kBeginValueEx(Go, GoDataStreamId)
    kAddField(GoDataStreamId, k32s, step)
    kAddField(GoDataStreamId, k32s, id)
    kAddField(GoDataStreamId, k32s, source)
kEndValueEx()

GoFx(kBool) GoOutputCompositeSource_VEquals(kType type, const void* value, const void* other)
{
    const GoOutputCompositeSource* a = value;
    const GoOutputCompositeSource* b = value;

    if (a->id == b->id && a->dataSource == b->dataSource)
    {
        return kTRUE;
    }

    return kFALSE;
}

kBeginEnumEx(Go, GoOutputSource)
    kAddEnumerator(GoOutputSource, GO_OUTPUT_SOURCE_NONE)
    kAddEnumerator(GoOutputSource, GO_OUTPUT_SOURCE_VIDEO)
    kAddEnumerator(GoOutputSource, GO_OUTPUT_SOURCE_RANGE)
    kAddEnumerator(GoOutputSource, GO_OUTPUT_SOURCE_PROFILE)
    kAddEnumerator(GoOutputSource, GO_OUTPUT_SOURCE_SURFACE)
    kAddEnumerator(GoOutputSource, GO_OUTPUT_SOURCE_SECTION)
    kAddEnumerator(GoOutputSource, GO_OUTPUT_SOURCE_RANGE_INTENSITY)
    kAddEnumerator(GoOutputSource, GO_OUTPUT_SOURCE_PROFILE_INTENSITY)
    kAddEnumerator(GoOutputSource, GO_OUTPUT_SOURCE_SURFACE_INTENSITY)
    kAddEnumerator(GoOutputSource, GO_OUTPUT_SOURCE_SECTION_INTENSITY)
    kAddEnumerator(GoOutputSource, GO_OUTPUT_SOURCE_MEASUREMENT)
    kAddEnumerator(GoOutputSource, GO_OUTPUT_SOURCE_TRACHEID)
    kAddEnumerator(GoOutputSource, GO_OUTPUT_SOURCE_EVENT)
    kAddEnumerator(GoOutputSource, GO_OUTPUT_SOURCE_FEATURE)
    kAddEnumerator(GoOutputSource, GO_OUTPUT_SOURCE_TOOLDATA)
    kEndEnumEx()

kBeginEnumEx(Go, GoOutputDelayDomain)
    kAddEnumerator(GoOutputDelayDomain, GO_OUTPUT_DELAY_DOMAIN_TIME)
    kAddEnumerator(GoOutputDelayDomain, GO_OUTPUT_DELAY_DOMAIN_ENCODER)
kEndEnumEx()

kBeginEnumEx(Go, GoPartFrameOfReference)
    kAddEnumerator(GoPartFrameOfReference, GO_PART_FRAME_OF_REFERENCE_TYPE_SENSOR)
    kAddEnumerator(GoPartFrameOfReference, GO_PART_FRAME_OF_REFERENCE_TYPE_SCAN)
    kAddEnumerator(GoPartFrameOfReference, GO_PART_FRAME_OF_REFERENCE_TYPE_PART)
kEndEnumEx()

kBeginEnumEx(Go, GoSpacingIntervalType)
    kAddEnumerator(GoSpacingIntervalType, GO_SPACING_INTERVAL_TYPE_MAX_RES)
    kAddEnumerator(GoSpacingIntervalType, GO_SPACING_INTERVAL_TYPE_BALANCED)
    kAddEnumerator(GoSpacingIntervalType, GO_SPACING_INTERVAL_TYPE_MAX_SPEED)
kEndEnumEx()

kBeginEnumEx(Go, GoTriggerSource)
    kAddEnumerator(GoTriggerSource, GO_TRIGGER_SOURCE_TIME)
    kAddEnumerator(GoTriggerSource, GO_TRIGGER_SOURCE_ENCODER)
    kAddEnumerator(GoTriggerSource, GO_TRIGGER_SOURCE_INPUT)
    kAddEnumerator(GoTriggerSource, GO_TRIGGER_SOURCE_SOFTWARE)
kEndEnumEx()

kBeginValueEx(Go, GoFeatureOption)
    kAddField(GoFeatureOption, kText64, name)
    kAddField(GoFeatureOption, kSize, minCount)
    kAddField(GoFeatureOption, kSize, maxCount)
    kAddField(GoFeatureOption, GoFeatureDataType, dataType)
    kAddField(GoFeatureOption, kText64, type)
kEndValueEx()

kBeginValueEx(Go, GoToolDataOutputOption)
    kAddField(GoToolDataOutputOption, kText64, name)
    kAddField(GoToolDataOutputOption, kText64, type)
    kAddField(GoToolDataOutputOption, GoDataType, dataType)
    kAddField(GoToolDataOutputOption, kSize, minCount)
    kAddField(GoToolDataOutputOption, kSize, maxCount)
kEndValueEx()

kBeginValueEx(Go, GoMeasurementOption)
    kAddField(GoMeasurementOption, kText64, name)
    kAddField(GoMeasurementOption, kSize, minCount)
    kAddField(GoMeasurementOption, kSize, maxCount)
kEndValueEx()

kBeginValueEx(Go, GoStates)
    kAddField(GoStates, GoState, sensorState)
    kAddField(GoStates, GoUser, loginType)
    kAddField(GoStates, GoAlignmentRef, alignmentReference)
    kAddField(GoStates, GoAlignmentState, alignmentState)
    kAddField(GoStates, kBool, recordingEnabled)
    kAddField(GoStates, k32s, playbackSource)
    kAddField(GoStates, k32u, uptimeSec)
    kAddField(GoStates, k32u, uptimeMicrosec)
    kAddField(GoStates, k32u, playbackPos)
    kAddField(GoStates, k32u, playbackCount)
    kAddField(GoStates, kBool, autoStartEnabled)
    kAddField(GoStates, kBool, isAccelerator)
    kAddField(GoStates, GoVoltageSetting, voltage)
    kAddField(GoStates, k32u, cableLength)
    kAddField(GoStates, kBool, quickEditEnabled)
    kAddField(GoStates, GoBrandingType, brandingType)
kEndValueEx()

kBeginValueEx(Go, GoAddressInfo)
    kAddField(GoAddressInfo, kBool, useDhcp)
    kAddField(GoAddressInfo, kIpAddress, address)
    kAddField(GoAddressInfo, kIpAddress, mask)
    kAddField(GoAddressInfo, kIpAddress, gateway)

    kAddVMethod(GoAddressInfo, kValue, VEquals)
kEndValueEx()

GoFx(kBool) GoAddressInfo_VEquals(kType type, const void* value, const void* other)
{
    const GoAddressInfo* a = value;
    const GoAddressInfo* b = other;
    kSize i;

    if (a->useDhcp == b->useDhcp)
    {
        for (i = 0; i < 16; i++)
        {
            if (a->address.address[i] != b->address.address[i]
                || a->mask.address[i] != b->mask.address[i]
                || a->gateway.address[i] != b->gateway.address[i])
            {
                return kFALSE;
            }
        }

        return kTRUE;
    }

    return kFALSE;
}

kBeginValueEx(Go, GoPortInfo)
    kAddField(GoPortInfo, k16u, controlPort)
    kAddField(GoPortInfo, k16u, upgradePort)
    kAddField(GoPortInfo, k16u, webPort)
    kAddField(GoPortInfo, k16u, dataPort)
    kAddField(GoPortInfo, k16u, healthPort)
kEndValueEx()

GoFx(kBool) GoPortInfo_VEquals(kType type, const void* value, const void* other)
{
    const GoPortInfo* a = value;
    const GoPortInfo* b = other;

    if (a->controlPort == b->controlPort &&
        a->dataPort == b->dataPort &&
        a->healthPort == b->healthPort &&
        a->upgradePort == b->upgradePort &&
        a->webPort == b->webPort)
    {
        return kTRUE;
    }

    return kFALSE;
}

kBeginValueEx(Go, GoBuddyInfo)
    kAddField(GoBuddyInfo, k32u, id)
    kAddField(GoBuddyInfo, GoBuddyState, state)
kEndValueEx()

kBeginValueEx(Go, GoElement64f)
    kAddField(GoElement64f, kBool, enabled)
    kAddField(GoElement64f, k64f, systemValue)
    kAddField(GoElement64f, k64f, value)
    kAddField(GoElement64f, k64f, max)
    kAddField(GoElement64f, k64f, min)
kEndValueEx()

kBeginValueEx(Go, GoElement32u)
    kAddField(GoElement32u, kBool, enabled)
    kAddField(GoElement32u, k32u, systemValue)
    kAddField(GoElement32u, k32u, value)
    kAddField(GoElement32u, k32u, max)
    kAddField(GoElement32u, k32u, min)
kEndValueEx()

kBeginValueEx(Go, GoElementBool)
    kAddField(GoElementBool, kBool, enabled)
    kAddField(GoElementBool, kBool, systemValue)
    kAddField(GoElementBool, kBool, value)
kEndValueEx()

kBeginValueEx(Go, GoFilter)
    kAddField(GoFilter, kBool, used)
    kAddField(GoFilter, GoElement64f, value)
kEndValueEx()

kBeginValueEx(Go, GoActiveAreaConfig)
    kAddField(GoActiveAreaConfig, GoElement64f, x)
    kAddField(GoActiveAreaConfig, GoElement64f, y)
    kAddField(GoActiveAreaConfig, GoElement64f, z)
    kAddField(GoActiveAreaConfig, GoElement64f, height)
    kAddField(GoActiveAreaConfig, GoElement64f, length)
    kAddField(GoActiveAreaConfig, GoElement64f, width)
kEndValueEx()

kBeginValueEx(Go, GoTransformation)
    kAddField(GoTransformation, k64f, x)
    kAddField(GoTransformation, k64f, y)
    kAddField(GoTransformation, k64f, z)
    kAddField(GoTransformation, k64f, xAngle)
    kAddField(GoTransformation, k64f, yAngle)
    kAddField(GoTransformation, k64f, zAngle)
kEndValueEx()

kBeginValueEx(Go, GoTransformedDataRegion)
    kAddField(GoTransformedDataRegion, k64f, x)
    kAddField(GoTransformedDataRegion, k64f, y)
    kAddField(GoTransformedDataRegion, k64f, z)
    kAddField(GoTransformedDataRegion, k64f, width)
    kAddField(GoTransformedDataRegion, k64f, length)
    kAddField(GoTransformedDataRegion, k64f, height)
kEndValueEx()

kBeginValueEx(Go, GoPolygonCornerParameters)
    kAddField(GoPolygonCornerParameters, kPoint64f, point)
    kAddField(GoPolygonCornerParameters, kArrayList, deviceIdxs)
kEndValueEx()

kBeginEnumEx(Go, GoPartHeightThresholdDirection)
    kAddEnumerator(GoPartHeightThresholdDirection, GO_PART_HEIGHT_THRESHOLD_DIRECTION_ABOVE)
    kAddEnumerator(GoPartHeightThresholdDirection, GO_PART_HEIGHT_THRESHOLD_DIRECTION_BELOW)
kEndEnumEx()

kBeginEnumEx(Go, GoSelcomFormat)
    kAddEnumerator(GoSelcomFormat, GO_SELCOM_FORMAT_SLS)
    kAddEnumerator(GoSelcomFormat, GO_SELCOM_FORMAT_12BIT_ST)
    kAddEnumerator(GoSelcomFormat, GO_SELCOM_FORMAT_14BIT)
    kAddEnumerator(GoSelcomFormat, GO_SELCOM_FORMAT_14BIT_ST)
kEndEnumEx()

kBeginEnumEx(Go, GoProfileGrooveLocation)
    kAddEnumerator(GoProfileGrooveLocation, GO_PROFILE_GROOVE_LOCATION_BOTTOM)
    kAddEnumerator(GoProfileGrooveLocation, GO_PROFILE_GROOVE_LOCATION_LEFT)
    kAddEnumerator(GoProfileGrooveLocation, GO_PROFILE_GROOVE_LOCATION_RIGHT)
kEndEnumEx()

kBeginEnumEx(Go, GoProfileStripSelectType)
    kAddEnumerator(GoProfileStripSelectType, GO_PROFILE_STRIP_SELECT_TYPE_BEST)
    kAddEnumerator(GoProfileStripSelectType, GO_PROFILE_STRIP_SELECT_TYPE_LEFT_INDEX)
    kAddEnumerator(GoProfileStripSelectType, GO_PROFILE_STRIP_SELECT_TYPE_RIGHT_INDEX)
kEndEnumEx()

kBeginEnumEx(Go, GoProfileStripLocation)
    kAddEnumerator(GoProfileStripLocation, GO_PROFILE_STRIP_LOCATION_LEFT)
    kAddEnumerator(GoProfileStripLocation, GO_PROFILE_STRIP_LOCATION_RIGHT)
    kAddEnumerator(GoProfileStripLocation, GO_PROFILE_STRIP_LOCATION_BOTTOM)
kEndEnumEx()

kBeginEnumEx(Go, GoProfileGenerationType)
    kAddEnumerator(GoProfileGenerationType, GO_PROFILE_GENERATION_TYPE_CONTINUOUS)
    kAddEnumerator(GoProfileGenerationType, GO_PROFILE_GENERATION_TYPE_FIXED_LENGTH)
    kAddEnumerator(GoProfileGenerationType, GO_PROFILE_GENERATION_TYPE_VARIABLE_LENGTH)
    kAddEnumerator(GoProfileGenerationType, GO_PROFILE_GENERATION_TYPE_ROTATIONAL)
kEndEnumEx()

kBeginEnumEx(Go, GoReplayCombineType)
    kAddEnumerator(GoReplayCombineType, GO_REPLAY_COMBINE_TYPE_ANY)
    kAddEnumerator(GoReplayCombineType, GO_REPLAY_COMBINE_TYPE_ALL)
kEndEnumEx()

kBeginEnumEx(Go, GoReplayConditionType)
    kAddEnumerator(GoReplayConditionType, GO_REPLAY_CONDITION_TYPE_ANY_MEASUREMENT)
    kAddEnumerator(GoReplayConditionType, GO_REPLAY_CONDITION_TYPE_ANY_DATA)
    kAddEnumerator(GoReplayConditionType, GO_REPLAY_CONDITION_TYPE_MEASUREMENT)
kEndEnumEx()

kBeginEnumEx(Go, GoReplayMeasurementResult)
    kAddEnumerator(GoReplayMeasurementResult, GO_REPLAY_MEASUREMENT_RESULT_PASS)
    kAddEnumerator(GoReplayMeasurementResult, GO_REPLAY_MEASUREMENT_RESULT_FAIL)
    kAddEnumerator(GoReplayMeasurementResult, GO_REPLAY_MEASUREMENT_RESULT_VALID)
    kAddEnumerator(GoReplayMeasurementResult, GO_REPLAY_MEASUREMENT_RESULT_INVALID)
    kAddEnumerator(GoReplayMeasurementResult, GO_REPLAY_MEASUREMENT_RESULT_FAIL_OR_INVALID)
kEndEnumEx()

kBeginEnumEx(Go, GoReplayRangeCountCase)
    kAddEnumerator(GoReplayRangeCountCase, GO_REPLAY_RANGE_COUNT_CASE_AT_ABOVE)
    kAddEnumerator(GoReplayRangeCountCase, GO_REPLAY_RANGE_COUNT_CASE_BELOW)
kEndEnumEx()

kBeginEnumEx(Go, GoSurfaceGenerationType)
    kAddEnumerator(GoSurfaceGenerationType, GO_SURFACE_GENERATION_TYPE_CONTINUOUS)
    kAddEnumerator(GoSurfaceGenerationType, GO_SURFACE_GENERATION_TYPE_FIXED_LENGTH)
    kAddEnumerator(GoSurfaceGenerationType, GO_SURFACE_GENERATION_TYPE_VARIABLE_LENGTH)
    kAddEnumerator(GoSurfaceGenerationType, GO_SURFACE_GENERATION_TYPE_ROTATIONAL)
kEndEnumEx()

kBeginEnumEx(Go, GoProfileStripBaseType)
    kAddEnumerator(GoProfileStripBaseType, GO_PROFILE_STRIP_BASE_TYPE_NONE)
    kAddEnumerator(GoProfileStripBaseType, GO_PROFILE_STRIP_BASE_TYPE_FLAT)
kEndEnumEx()

kBeginEnumEx(Go, GoSerialProtocol)
    kAddEnumerator(GoSerialProtocol, GO_SERIAL_PROTOCOL_GOCATOR)
    kAddEnumerator(GoSerialProtocol, GO_SERIAL_PROTOCOL_SELCOM)
kEndEnumEx()

kBeginEnumEx(Go, GoEndianType)
    kAddEnumerator(GoEndianType, GO_ENDIAN_TYPE_BIG)
    kAddEnumerator(GoEndianType, GO_ENDIAN_TYPE_LITTLE)
kEndEnumEx()

kBeginEnumEx(Go, GoAlignmentType)
    kAddEnumerator(GoAlignmentType, GO_ALIGNMENT_TYPE_STATIONARY)
    kAddEnumerator(GoAlignmentType, GO_ALIGNMENT_TYPE_MOVING)
kEndEnumEx()

kBeginEnumEx(Go, GoAsciiOperation)
    kAddEnumerator(GoAsciiOperation, GO_ASCII_OPERATION_ASYNCHRONOUS)
    kAddEnumerator(GoAsciiOperation, GO_ASCII_OPERATION_POLLING)
kEndEnumEx()

kBeginEnumEx(Go, GoAsciiStandardFormatMode)
    kAddEnumerator(GoAsciiStandardFormatMode, GS_ASCII_FORMAT_MODE_MEAS)
    kAddEnumerator(GoAsciiStandardFormatMode, GS_ASCII_FORMAT_MODE_ENCODER_AND_FRAME)
kEndEnumEx()

kBeginEnumEx(Go, GoProfileGenerationStartTrigger)
    kAddEnumerator(GoProfileGenerationStartTrigger, GO_PROFILE_GENERATION_START_TRIGGER_SEQUENTIAL)
    kAddEnumerator(GoProfileGenerationStartTrigger, GO_PROFILE_GENERATION_START_TRIGGER_DIGITAL)
kEndEnumEx()

kBeginEnumEx(Go, GoSurfaceGenerationStartTrigger)
    kAddEnumerator(GoSurfaceGenerationStartTrigger, GO_SURFACE_GENERATION_START_TRIGGER_SEQUENTIAL)
    kAddEnumerator(GoSurfaceGenerationStartTrigger, GO_SURFACE_GENERATION_START_TRIGGER_DIGITAL)
    kAddEnumerator(GoSurfaceGenerationStartTrigger, GO_SURFACE_GENERATION_START_TRIGGER_SOFTWARE)
kEndEnumEx()

kBeginEnumEx(Go, GoInputSource)
    kAddEnumerator(GoInputSource, GO_INPUT_SOURCE_LIVE)
    kAddEnumerator(GoInputSource, GO_INPUT_SOURCE_RECORDING)
kEndEnumEx()

kBeginEnumEx(Go, GoExtMeasurementType)
    kAddEnumerator(GoExtMeasurementType, GO_EXT_MEASUREMENT_TYPE_GENERIC)
    kAddEnumerator(GoExtMeasurementType, GO_EXT_MEASUREMENT_TYPE_X)
    kAddEnumerator(GoExtMeasurementType, GO_EXT_MEASUREMENT_TYPE_Y)
    kAddEnumerator(GoExtMeasurementType, GO_EXT_MEASUREMENT_TYPE_Z)
    kAddEnumerator(GoExtMeasurementType, GO_EXT_MEASUREMENT_TYPE_X_ANGLE)
    kAddEnumerator(GoExtMeasurementType, GO_EXT_MEASUREMENT_TYPE_Y_ANGLE)
    kAddEnumerator(GoExtMeasurementType, GO_EXT_MEASUREMENT_TYPE_Z_ANGLE)
kEndEnumEx()


kBeginEnumEx(Go, GoExtParamType)
    kAddEnumerator(GoExtParamType, GO_EXT_PARAM_TYPE_UNKNOWN)
    kAddEnumerator(GoExtParamType, GO_EXT_PARAM_TYPE_INT)
    kAddEnumerator(GoExtParamType, GO_EXT_PARAM_TYPE_FLOAT)
    kAddEnumerator(GoExtParamType, GO_EXT_PARAM_TYPE_BOOL)
    kAddEnumerator(GoExtParamType, GO_EXT_PARAM_TYPE_STRING)
    kAddEnumerator(GoExtParamType, GO_EXT_PARAM_TYPE_PROFILE_REGION)
    kAddEnumerator(GoExtParamType, GO_EXT_PARAM_TYPE_SURFACE_REGION_3D)
    kAddEnumerator(GoExtParamType, GO_EXT_PARAM_TYPE_SURFACE_REGION_2D)
    kAddEnumerator(GoExtParamType, GO_EXT_PARAM_TYPE_GEOMETRIC_FEATURE)
    kAddEnumerator(GoExtParamType, GO_EXT_PARAM_TYPE_MEASUREMENT)
    kAddEnumerator(GoExtParamType, GO_EXT_PARAM_TYPE_DATA_INPUT)
    kAddEnumerator(GoExtParamType, GO_EXT_PARAM_TYPE_POINT_SET_REGION)
kEndEnumEx()

kBeginEnumEx(Go, GoFamily)
    kAddEnumerator(GoFamily, GO_FAMILY_1000)
    kAddEnumerator(GoFamily, GO_FAMILY_2000)
    kAddEnumerator(GoFamily, GO_FAMILY_3000)
kEndEnumEx()

kBeginEnumEx(Go, GoSurfaceOpeningType)
    kAddEnumerator(GoSurfaceOpeningType, GO_SURFACE_OPENING_TYPE_ROUNDED_SLOT)
    kAddEnumerator(GoSurfaceOpeningType, GO_SURFACE_OPENING_TYPE_ROUNDED_RECTANGLE)
kEndEnumEx()

kBeginEnumEx(Go, GoSurfaceRivetType)
    kAddEnumerator(GoSurfaceRivetType, GO_SURFACE_RIVET_TYPE_FLUSH)
    kAddEnumerator(GoSurfaceRivetType, GO_SURFACE_RIVET_TYPE_RAISED)
kEndEnumEx()

kBeginEnumEx(Go, GoPartMatchAlgorithm)
    kAddEnumerator(GoPartMatchAlgorithm, GO_PART_MATCH_ALGORITHM_EDGE)
    kAddEnumerator(GoPartMatchAlgorithm, GO_PART_MATCH_ALGORITHM_BOUNDING_BOX)
    kAddEnumerator(GoPartMatchAlgorithm, GO_PART_MATCH_ALGORITHM_ELLIPSE)
kEndEnumEx()

kBeginEnumEx(Go, GoDataMessageType)
    kAddEnumerator(GoDataMessageType, GO_DATA_MESSAGE_TYPE_UNKNOWN)
    kAddEnumerator(GoDataMessageType, GO_DATA_MESSAGE_TYPE_STAMP)
    kAddEnumerator(GoDataMessageType, GO_DATA_MESSAGE_TYPE_HEALTH)
    kAddEnumerator(GoDataMessageType, GO_DATA_MESSAGE_TYPE_VIDEO)
    kAddEnumerator(GoDataMessageType, GO_DATA_MESSAGE_TYPE_RANGE)
    kAddEnumerator(GoDataMessageType, GO_DATA_MESSAGE_TYPE_RANGE_INTENSITY)
    kAddEnumerator(GoDataMessageType, GO_DATA_MESSAGE_TYPE_PROFILE)
    kAddEnumerator(GoDataMessageType, GO_DATA_MESSAGE_TYPE_PROFILE_INTENSITY)
    kAddEnumerator(GoDataMessageType, GO_DATA_MESSAGE_TYPE_RESAMPLED_PROFILE)
    kAddEnumerator(GoDataMessageType, GO_DATA_MESSAGE_TYPE_SURFACE)
    kAddEnumerator(GoDataMessageType, GO_DATA_MESSAGE_TYPE_SURFACE_POINT_CLOUD)
    kAddEnumerator(GoDataMessageType, GO_DATA_MESSAGE_TYPE_SURFACE_INTENSITY)
    kAddEnumerator(GoDataMessageType, GO_DATA_MESSAGE_TYPE_MEASUREMENT)
    kAddEnumerator(GoDataMessageType, GO_DATA_MESSAGE_TYPE_ALIGNMENT)
    kAddEnumerator(GoDataMessageType, GO_DATA_MESSAGE_TYPE_EXPOSURE_CAL)
    kAddEnumerator(GoDataMessageType, GO_DATA_MESSAGE_TYPE_EDGE_MATCH)
    kAddEnumerator(GoDataMessageType, GO_DATA_MESSAGE_TYPE_BOUNDING_BOX_MATCH)
    kAddEnumerator(GoDataMessageType, GO_DATA_MESSAGE_TYPE_ELLIPSE_MATCH)
    kAddEnumerator(GoDataMessageType, GO_DATA_MESSAGE_TYPE_SECTION)
    kAddEnumerator(GoDataMessageType, GO_DATA_MESSAGE_TYPE_SECTION_INTENSITY)
    kAddEnumerator(GoDataMessageType, GO_DATA_MESSAGE_TYPE_EVENT)
    kAddEnumerator(GoDataMessageType, GO_DATA_MESSAGE_TYPE_TRACHEID)
    kAddEnumerator(GoDataMessageType, GO_DATA_MESSAGE_TYPE_GENERIC)
    kAddEnumerator(GoDataMessageType, GO_DATA_MESSAGE_TYPE_MESH)
kEndEnumEx()

kBeginEnumEx(Go, GoAdvancedType)
kAddEnumerator(GoAdvancedType, GO_ADVANCED_TYPE_CUSTOM)
kAddEnumerator(GoAdvancedType, GO_ADVANCED_TYPE_DIFFUSE)
kAddEnumerator(GoAdvancedType, GO_ADVANCED_TYPE_REFLECTIVE)
kEndEnumEx()

kBeginEnumEx(Go, GoMaterialType)
    kAddEnumerator(GoMaterialType, GO_MATERIAL_TYPE_CUSTOM)
    kAddEnumerator(GoMaterialType, GO_MATERIAL_TYPE_DIFFUSE)
kEndEnumEx()

kBeginEnumEx(Go, GoSpotSelectionType)
    kAddEnumerator(GoSpotSelectionType, GO_SPOT_SELECTION_TYPE_BEST)
    kAddEnumerator(GoSpotSelectionType, GO_SPOT_SELECTION_TYPE_TOP)
    kAddEnumerator(GoSpotSelectionType, GO_SPOT_SELECTION_TYPE_BOTTOM)
    kAddEnumerator(GoSpotSelectionType, GO_SPOT_SELECTION_TYPE_NONE)
kEndEnumEx()

kBeginEnumEx(Go, GoTranslucentThreadingMode)
    kAddEnumerator(GoTranslucentThreadingMode, GO_TRANSLUCENT_THREADING_MODE_NONE)
    kAddEnumerator(GoTranslucentThreadingMode, GO_TRANSLUCENT_THREADING_MODE_BATCHING)
kEndEnumEx()

kBeginEnumEx(Go, GoPixelType)
    kAddEnumerator(GoPixelType, GO_PIXEL_TYPE_8U)
    kAddEnumerator(GoPixelType, GO_PIXEL_TYPE_RGB)
kEndEnumEx()

kBeginEnumEx(Go, GoToolType)
    kAddEnumerator(GoToolType, GO_TOOL_UNKNOWN)
    kAddEnumerator(GoToolType, GO_TOOL_RANGE_POSITION)
    kAddEnumerator(GoToolType, GO_TOOL_RANGE_THICKNESS)
    kAddEnumerator(GoToolType, GO_TOOL_PROFILE_AREA)
    kAddEnumerator(GoToolType, GO_TOOL_PROFILE_BOUNDING_BOX)
    kAddEnumerator(GoToolType, GO_TOOL_PROFILE_BRIDGE_VALUE)
    kAddEnumerator(GoToolType, GO_TOOL_PROFILE_CIRCLE)
    kAddEnumerator(GoToolType, GO_TOOL_PROFILE_DIMENSION)
    kAddEnumerator(GoToolType, GO_TOOL_PROFILE_GROOVE)
    kAddEnumerator(GoToolType, GO_TOOL_PROFILE_INTERSECT)
    kAddEnumerator(GoToolType, GO_TOOL_PROFILE_LINE)
    kAddEnumerator(GoToolType, GO_TOOL_PROFILE_PANEL)
    kAddEnumerator(GoToolType, GO_TOOL_PROFILE_POSITION)
    kAddEnumerator(GoToolType, GO_TOOL_PROFILE_STRIP)
    kAddEnumerator(GoToolType, GO_TOOL_PROFILE_X_LINE)
    kAddEnumerator(GoToolType, GO_TOOL_SURFACE_BOUNDING_BOX)
    kAddEnumerator(GoToolType, GO_TOOL_SURFACE_COUNTERSUNK_HOLE)
    kAddEnumerator(GoToolType, GO_TOOL_SURFACE_DIMENSION)
    kAddEnumerator(GoToolType, GO_TOOL_SURFACE_ELLIPSE)
    kAddEnumerator(GoToolType, GO_TOOL_SURFACE_HOLE)
    kAddEnumerator(GoToolType, GO_TOOL_SURFACE_OPENING)
    kAddEnumerator(GoToolType, GO_TOOL_SURFACE_PLANE)
    kAddEnumerator(GoToolType, GO_TOOL_SURFACE_POSITION)
    kAddEnumerator(GoToolType, GO_TOOL_SURFACE_RIVET)
    kAddEnumerator(GoToolType, GO_TOOL_SURFACE_STUD)
    kAddEnumerator(GoToolType, GO_TOOL_SURFACE_VOLUME)
    kAddEnumerator(GoToolType, GO_TOOL_SCRIPT)
    kAddEnumerator(GoToolType, GO_TOOL_EXTENSIBLE)
kEndEnumEx()

kBeginEnumEx(Go, GoDataType)
    kAddEnumerator(GoDataType, GO_DATA_TYPE_NONE)
    kAddEnumerator(GoDataType, GO_DATA_TYPE_RANGE)
    kAddEnumerator(GoDataType, GO_DATA_TYPE_UNIFORM_PROFILE)
    kAddEnumerator(GoDataType, GO_DATA_TYPE_PROFILE_POINT_CLOUD)
    kAddEnumerator(GoDataType, GO_DATA_TYPE_UNIFORM_SURFACE)
    kAddEnumerator(GoDataType, GO_DATA_TYPE_SURFACE_POINT_CLOUD)
    kAddEnumerator(GoDataType, GO_DATA_TYPE_UNMERGED_PROFILE_POINT_CLOUD)
    kAddEnumerator(GoDataType, GO_DATA_TYPE_VIDEO)
    kAddEnumerator(GoDataType, GO_DATA_TYPE_TRACHEID)
    kAddEnumerator(GoDataType, GO_DATA_TYPE_MESH)
    kAddEnumerator(GoDataType, GO_DATA_TYPE_FEATURES_ONLY)
    kAddEnumerator(GoDataType, GO_DATA_TYPE_GENERIC_BASE)
    kAddEnumerator(GoDataType, GO_DATA_TYPE_GENERIC_END)
    //Deprecated:
    kAddEnumerator(GoDataType, GO_DATA_TYPE_RAW_PROFILE)
    kAddEnumerator(GoDataType, GO_DATA_TYPE_RAW_SURFACE)
    kAddEnumerator(GoDataType, GO_DATA_TYPE_UNMERGED_RAW_PROFILE)
kEndEnumEx()

kBeginEnumEx(Go, GoFeatureDataType)
    kAddEnumerator(GoFeatureDataType, GO_FEATURE_UNKNOWN)
    kAddEnumerator(GoFeatureDataType, GO_FEATURE_DATA_POINT)
    kAddEnumerator(GoFeatureDataType, GO_FEATURE_DATA_LINE)
    kAddEnumerator(GoFeatureDataType, GO_FEATURE_DATA_CIRCLE)
    kAddEnumerator(GoFeatureDataType, GO_FEATURE_DATA_PLANE)
kEndEnumEx()

kBeginValueEx(Go, GoFacet32u)
    kAddField(GoFacet32u, k32u, vertex1)
    kAddField(GoFacet32u, k32u, vertex2)
    kAddField(GoFacet32u, k32u, vertex3)
kEndValueEx()

kBeginValueEx(Go, Go3dTransform64f)
    kAddField(Go3dTransform64f, k64f, xx)
    kAddField(Go3dTransform64f, k64f, xy)
    kAddField(Go3dTransform64f, k64f, xz)
    kAddField(Go3dTransform64f, k64f, xt)
    kAddField(Go3dTransform64f, k64f, yx)
    kAddField(Go3dTransform64f, k64f, yy)
    kAddField(Go3dTransform64f, k64f, yz)
    kAddField(Go3dTransform64f, k64f, yt)
    kAddField(Go3dTransform64f, k64f, zx)
    kAddField(Go3dTransform64f, k64f, zy)
    kAddField(Go3dTransform64f, k64f, zz)
    kAddField(Go3dTransform64f, k64f, zt)
kEndValueEx()

kBeginValueEx(Go, GoMeshMsgChannel)
    kAddField(GoMeshMsgChannel, k32s, id)
    kAddField(GoMeshMsgChannel, k32s, type)
    kAddField(GoMeshMsgChannel, k32s, state)
    kAddField(GoMeshMsgChannel, k32u, flag)
    kAddField(GoMeshMsgChannel, kSize, allocatedCount)
    kAddField(GoMeshMsgChannel, kSize, dataCount)
    kAddField(GoMeshMsgChannel, kType, dataType)
    kAddField(GoMeshMsgChannel, kArray1, buffer)
kEndValueEx()

kBeginEnumEx(Go, GoFeatureType)
kAddEnumerator(GoFeatureType, GO_FEATURE_UNKNOWN)
kAddEnumerator(GoFeatureType, GO_FEATURE_EXTENSIBLE)
kEndEnumEx()

kBeginValueEx(Go, GoMeasurementsNameTypeMapping)
    kAddField(GoMeasurementsNameTypeMapping, GOMEASUREMENTS_NAME_TYPE, lookupName)
    kAddField(GoMeasurementsNameTypeMapping, GOMEASUREMENTS_NAME_TYPE, measurementName)
    kAddField(GoMeasurementsNameTypeMapping, kType, measurementType)
kEndValueEx()

kBeginEnumEx(Go, GoMeasurementType)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_UNKNOWN)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_RANGE_POSITION_Z)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_RANGE_THICKNESS_THICKNESS)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_PROFILE_AREA_AREA)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_PROFILE_AREA_CENTROID_X)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_PROFILE_AREA_CENTROID_Z)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_PROFILE_BOUNDING_BOX_X)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_PROFILE_BOUNDING_BOX_Z)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_PROFILE_BOUNDING_BOX_HEIGHT)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_PROFILE_BOUNDING_BOX_WIDTH)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_PROFILE_BOUNDING_BOX_GLOBAL_X)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_PROFILE_BOUNDING_BOX_GLOBAL_Y)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_PROFILE_BOUNDING_BOX_GLOBAL_ANGLE)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_PROFILE_BRIDGE_VALUE_BRIDGE_VALUE)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_PROFILE_BRIDGE_VALUE_ANGLE)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_PROFILE_CIRCLE_X)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_PROFILE_CIRCLE_Z)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_PROFILE_CIRCLE_RADIUS)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_PROFILE_CIRCLE_STDDEV)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_PROFILE_CIRCLE_MIN_ERROR)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_PROFILE_CIRCLE_MIN_ERROR_X)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_PROFILE_CIRCLE_MIN_ERROR_Z)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_PROFILE_CIRCLE_MAX_ERROR)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_PROFILE_CIRCLE_MAX_ERROR_X)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_PROFILE_CIRCLE_MAX_ERROR_Z)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_PROFILE_DIMENSION_WIDTH)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_PROFILE_DIMENSION_HEIGHT)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_PROFILE_DIMENSION_DISTANCE)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_PROFILE_DIMENSION_CENTER_X)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_PROFILE_DIMENSION_CENTER_Z)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_PROFILE_GROOVE_X)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_PROFILE_GROOVE_Z)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_PROFILE_GROOVE_WIDTH)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_PROFILE_GROOVE_DEPTH)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_PROFILE_INTERSECT_X)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_PROFILE_INTERSECT_Z)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_PROFILE_INTERSECT_ANGLE)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_PROFILE_LINE_STDDEV)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_PROFILE_LINE_ERROR_MIN)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_PROFILE_LINE_ERROR_MAX)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_PROFILE_LINE_OFFSET)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_PROFILE_LINE_ANGLE)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_PROFILE_LINE_ERROR_MIN_X)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_PROFILE_LINE_ERROR_MIN_Z)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_PROFILE_LINE_ERROR_MAX_X)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_PROFILE_LINE_ERROR_MAX_Z)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_PROFILE_LINE_PERCENTILE)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_PROFILE_PANEL_GAP)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_PROFILE_PANEL_FLUSH)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_PROFILE_POSITION_X)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_PROFILE_POSITION_Z)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_PROFILE_STRIP_POSITION_X)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_PROFILE_STRIP_POSITION_Z)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_PROFILE_STRIP_WIDTH)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_PROFILE_STRIP_HEIGHT)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_PROFILE_X_LINE_Z)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_PROFILE_X_LINE_VALIDITY)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_SURFACE_BOUNDING_BOX_X)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_SURFACE_BOUNDING_BOX_Y)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_SURFACE_BOUNDING_BOX_Z)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_SURFACE_BOUNDING_BOX_ZANGLE)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_SURFACE_BOUNDING_BOX_HEIGHT)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_SURFACE_BOUNDING_BOX_WIDTH)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_SURFACE_BOUNDING_BOX_LENGTH)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_SURFACE_BOUNDING_BOX_GLOBAL_X)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_SURFACE_BOUNDING_BOX_GLOBAL_Y)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_SURFACE_BOUNDING_BOX_GLOBAL_Z_ANGLE)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_SURFACE_COUNTERSUNK_HOLE_X)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_SURFACE_COUNTERSUNK_HOLE_Y)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_SURFACE_COUNTERSUNK_HOLE_Z)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_SURFACE_COUNTERSUNK_HOLE_OUTER_RADIUS)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_SURFACE_COUNTERSUNK_HOLE_DEPTH)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_SURFACE_COUNTERSUNK_HOLE_COUNTERBORE_DEPTH)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_SURFACE_COUNTERSUNK_HOLE_BEVEL_RADIUS)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_SURFACE_COUNTERSUNK_HOLE_BEVEL_ANGLE)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_SURFACE_COUNTERSUNK_HOLE_X_ANGLE)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_SURFACE_COUNTERSUNK_HOLE_Y_ANGLE)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_SURFACE_COUNTERSUNK_HOLE_AXIS_TILT)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_SURFACE_COUNTERSUNK_HOLE_AXIS_ORIENTATION)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_SURFACE_DIMENSION_WIDTH)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_SURFACE_DIMENSION_HEIGHT)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_SURFACE_DIMENSION_LENGTH)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_SURFACE_DIMENSION_DISTANCE)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_SURFACE_DIMENSION_PLANE_DISTANCE)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_SURFACE_DIMENSION_CENTER_X)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_SURFACE_DIMENSION_CENTER_Y)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_SURFACE_DIMENSION_CENTER_Z)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_SURFACE_ELLIPSE_MAJOR)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_SURFACE_ELLIPSE_MINOR)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_SURFACE_ELLIPSE_RATIO)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_SURFACE_ELLIPSE_ZANGLE)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_SURFACE_HOLE_X)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_SURFACE_HOLE_Y)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_SURFACE_HOLE_Z)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_SURFACE_HOLE_RADIUS)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_SURFACE_OPENING_X)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_SURFACE_OPENING_Y)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_SURFACE_OPENING_Z)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_SURFACE_OPENING_WIDTH)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_SURFACE_OPENING_LENGTH)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_SURFACE_OPENING_ANGLE)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_SURFACE_PLANE_X_ANGLE)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_SURFACE_PLANE_Y_ANGLE)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_SURFACE_PLANE_Z_OFFSET)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_SURFACE_PLANE_STD_DEV)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_SURFACE_PLANE_ERROR_MIN)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_SURFACE_PLANE_ERROR_MAX)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_SURFACE_PLANE_X_NORMAL)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_SURFACE_PLANE_Y_NORMAL)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_SURFACE_PLANE_Z_NORMAL)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_SURFACE_PLANE_DISTANCE)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_SURFACE_POSITION_X)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_SURFACE_POSITION_Y)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_SURFACE_POSITION_Z)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_SURFACE_INTERSECT_X)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_SURFACE_INTERSECT_Y)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_SURFACE_INTERSECT_Z)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_SURFACE_EDGE_X)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_SURFACE_EDGE_Y)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_SURFACE_EDGE_Z)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_SURFACE_RIVET_X)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_SURFACE_RIVET_Y)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_SURFACE_RIVET_Z)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_SURFACE_RIVET_TILT_ANGLE)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_SURFACE_RIVET_TILT_DIRECTION)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_SURFACE_RIVET_RADIUS)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_SURFACE_RIVET_TOP_OFFSET_MIN)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_SURFACE_RIVET_TOP_OFFSET_MAX)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_SURFACE_RIVET_TOP_OFFSET_MEAN)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_SURFACE_RIVET_TOP_OFFSET_STD_DEV)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_SURFACE_RIVET_RADIAL_HEIGHT_MIN)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_SURFACE_RIVET_RADIAL_HEIGHT_MAX)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_SURFACE_RIVET_RADIAL_HEIGHT_MEAN)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_SURFACE_RIVET_RADIAL_HEIGHT_STD_DEV)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_SURFACE_RIVET_RADIAL_SLOPE_MIN)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_SURFACE_RIVET_RADIAL_SLOPE_MAX)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_SURFACE_RIVET_RADIAL_SLOPE_MEAN)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_SURFACE_RIVET_RADIAL_SLOPE_STD_DEV)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_SURFACE_STUD_BASE_X)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_SURFACE_STUD_BASE_Y)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_SURFACE_STUD_BASE_Z)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_SURFACE_STUD_TIP_X)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_SURFACE_STUD_TIP_Y)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_SURFACE_STUD_TIP_Z)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_SURFACE_STUD_RADIUS)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_SURFACE_VOLUME_AREA)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_SURFACE_VOLUME_VOLUME)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_SURFACE_VOLUME_THICKNESS)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_SCRIPT_OUTPUT)
    kAddEnumerator(GoMeasurementType, GO_MEASUREMENT_EXTENSIBLE)
kEndEnumEx()

kBeginEnumEx(Go, GoReplayExportSourceType)
    kAddEnumerator(GoReplayExportSourceType, GO_REPLAY_EXPORT_SOURCE_PRIMARY)
    kAddEnumerator(GoReplayExportSourceType, GO_REPLAY_EXPORT_SOURCE_INTENSITY)
kEndEnumEx()

kBeginEnumEx(Go, GoSeekDirection)
    kAddEnumerator(GoSeekDirection, GO_SEEK_DIRECTION_FORWARD)
    kAddEnumerator(GoSeekDirection, GO_SEEK_DIRECTION_BACKWARD)
kEndEnumEx()

kBeginEnumEx(Go, GoProfileStripEdgeType)
    kAddEnumerator(GoProfileStripEdgeType, GO_PROFILE_STRIP_EDGE_TYPE_RISING)
    kAddEnumerator(GoProfileStripEdgeType, GO_PROFILE_STRIP_EDGE_TYPE_FALLING)
    kAddEnumerator(GoProfileStripEdgeType, GO_PROFILE_STRIP_EDGE_TYPE_DATA_END)
    kAddEnumerator(GoProfileStripEdgeType, GO_PROFILE_STRIP_EDGE_TYPE_VOID)
kEndEnumEx()

kBeginValueEx(Go, GoFeaturesNameTypeMapping)
    kAddField(GoFeaturesNameTypeMapping, GOFEATURES_NAME_TYPE, lookupName)
    kAddField(GoFeaturesNameTypeMapping, GOFEATURES_NAME_TYPE, featureName)
    kAddField(GoFeaturesNameTypeMapping, kType, featureType)
kEndValueEx()

kBeginEnumEx(Go, GoProfileFeatureType)
    kAddEnumerator(GoProfileFeatureType, GO_PROFILE_FEATURE_TYPE_MAX_Z)
    kAddEnumerator(GoProfileFeatureType, GO_PROFILE_FEATURE_TYPE_MIN_Z)
    kAddEnumerator(GoProfileFeatureType, GO_PROFILE_FEATURE_TYPE_MAX_X)
    kAddEnumerator(GoProfileFeatureType, GO_PROFILE_FEATURE_TYPE_MIN_X)
    kAddEnumerator(GoProfileFeatureType, GO_PROFILE_FEATURE_TYPE_CORNER)
    kAddEnumerator(GoProfileFeatureType, GO_PROFILE_FEATURE_TYPE_AVERAGE)
    kAddEnumerator(GoProfileFeatureType, GO_PROFILE_FEATURE_TYPE_RISING_EDGE)
    kAddEnumerator(GoProfileFeatureType, GO_PROFILE_FEATURE_TYPE_FALLING_EDGE)
    kAddEnumerator(GoProfileFeatureType, GO_PROFILE_FEATURE_TYPE_ANY_EDGE)
    kAddEnumerator(GoProfileFeatureType, GO_PROFILE_FEATURE_TYPE_TOP_CORNER)
    kAddEnumerator(GoProfileFeatureType, GO_PROFILE_FEATURE_TYPE_BOTTOM_CORNER)
    kAddEnumerator(GoProfileFeatureType, GO_PROFILE_FEATURE_TYPE_LEFT_CORNER)
    kAddEnumerator(GoProfileFeatureType, GO_PROFILE_FEATURE_TYPE_RIGHT_CORNER)
    kAddEnumerator(GoProfileFeatureType, GO_PROFILE_FEATURE_TYPE_MEDIAN)
kEndEnumEx()

kBeginEnumEx(Go, GoProfileGapAxis)
    kAddEnumerator(GoProfileGapAxis, GO_PROFILE_GAP_AXIS_EDGE)
    kAddEnumerator(GoProfileGapAxis, GO_PROFILE_GAP_AXIS_SURFACE)
    kAddEnumerator(GoProfileGapAxis, GO_PROFILE_GAP_AXIS_DISTANCE)
kEndEnumEx()

kBeginEnumEx(Go, GoProfileEdgeType)
    kAddEnumerator(GoProfileEdgeType, GO_PROFILE_EDGE_TYPE_TANGENT)
    kAddEnumerator(GoProfileEdgeType, GO_PROFILE_EDGE_TYPE_CORNER)
kEndEnumEx()

kBeginEnumEx(Go, GoProfileGrooveShape)
    kAddEnumerator(GoProfileGrooveShape, GO_PROFILE_GROOVE_SHAPE_U)
    kAddEnumerator(GoProfileGrooveShape, GO_PROFILE_GROOVE_SHAPE_V)
    kAddEnumerator(GoProfileGrooveShape, GO_PROFILE_GROOVE_SHAPE_OPEN)
kEndEnumEx()

kBeginEnumEx(Go, GoProfileGrooveSelectType)
    kAddEnumerator(GoProfileGrooveSelectType, GO_PROFILE_GROOVE_SELECT_TYPE_MAX_DEPTH)
    kAddEnumerator(GoProfileGrooveSelectType, GO_PROFILE_GROOVE_SELECT_TYPE_LEFT_INDEX)
    kAddEnumerator(GoProfileGrooveSelectType, GO_PROFILE_GROOVE_SELECT_TYPE_RIGHT_INDEX)
kEndEnumEx()

kBeginEnumEx(Go, GoProfileBaseline)
    kAddEnumerator(GoProfileBaseline, GO_PROFILE_BASELINE_TYPE_LINE)
    kAddEnumerator(GoProfileBaseline, GO_PROFILE_BASELINE_TYPE_X_AXIS)
kEndEnumEx()

kBeginEnumEx(Go, GoProfileAreaType)
    kAddEnumerator(GoProfileAreaType, GO_PROFILE_AREA_TYPE_OBJECT)
    kAddEnumerator(GoProfileAreaType, GO_PROFILE_AREA_TYPE_CLEARANCE)
kEndEnumEx()

kBeginEnumEx(Go, GoProfilePanelSide)
    kAddEnumerator(GoProfilePanelSide, GO_PROFILE_PANEL_SIDE_LEFT)
    kAddEnumerator(GoProfilePanelSide, GO_PROFILE_PANEL_SIDE_RIGHT)
kEndEnumEx()

kBeginEnumEx(Go, GoProfileRoundCornerDirection)
    kAddEnumerator(GoProfileRoundCornerDirection, GO_PROFILE_ROUND_CORNER_DIRECTION_LEFT)
    kAddEnumerator(GoProfileRoundCornerDirection, GO_PROFILE_ROUND_CORNER_DIRECTION_RIGHT)
kEndEnumEx()

kBeginEnumEx(Go, GoSurfaceLocation)
    kAddEnumerator(GoSurfaceLocation, GO_SURFACE_LOCATION_TYPE_MAX)
    kAddEnumerator(GoSurfaceLocation, GO_SURFACE_LOCATION_TYPE_MIN)
    kAddEnumerator(GoSurfaceLocation, GO_SURFACE_LOCATION_TYPE_2D_CENTROID)
    kAddEnumerator(GoSurfaceLocation, GO_SURFACE_LOCATION_TYPE_3D_CENTROID)
    kAddEnumerator(GoSurfaceLocation, GO_SURFACE_LOCATION_TYPE_AVG)
    kAddEnumerator(GoSurfaceLocation, GO_SURFACE_LOCATION_TYPE_MEDIAN)
kEndEnumEx()

kBeginEnumEx(Go, GoEventType)
    kAddEnumerator(GoEventType, GO_EVENT_TYPE_EXPOSURE_END)
kEndEnumEx()

kBeginEnumEx(Go, GoSurfaceFeatureType)
    kAddEnumerator(GoSurfaceFeatureType, GO_SURFACE_FEATURE_TYPE_AVERAGE)
    kAddEnumerator(GoSurfaceFeatureType, GO_SURFACE_FEATURE_TYPE_CENTROID)
    kAddEnumerator(GoSurfaceFeatureType, GO_SURFACE_FEATURE_TYPE_X_MAX)
    kAddEnumerator(GoSurfaceFeatureType, GO_SURFACE_FEATURE_TYPE_X_MIN)
    kAddEnumerator(GoSurfaceFeatureType, GO_SURFACE_FEATURE_TYPE_Y_MAX)
    kAddEnumerator(GoSurfaceFeatureType, GO_SURFACE_FEATURE_TYPE_Y_MIN)
    kAddEnumerator(GoSurfaceFeatureType, GO_SURFACE_FEATURE_TYPE_Z_MAX)
    kAddEnumerator(GoSurfaceFeatureType, GO_SURFACE_FEATURE_TYPE_Z_MIN)
    kAddEnumerator(GoSurfaceFeatureType, GO_SURFACE_FEATURE_TYPE_MEDIAN)
kEndEnumEx()

kBeginEnumEx(Go, GoSurfaceCountersunkHoleShape)
    kAddEnumerator(GoSurfaceCountersunkHoleShape, GO_SURFACE_COUNTERSUNK_HOLE_SHAPE_CONE)
    kAddEnumerator(GoSurfaceCountersunkHoleShape, GO_SURFACE_COUNTERSUNK_HOLE_SHAPE_COUNTERBORE)
kEndEnumEx()

kBeginEnumEx(Go, GoOcclusionReductionAlg)
    kAddEnumerator(GoOcclusionReductionAlg, GO_OCCLUSION_REDUCTION_NORMAL)
    kAddEnumerator(GoOcclusionReductionAlg, GO_OCCLUSION_REDUCTION_HIGH_QUALITY)
kEndEnumEx()

kBeginEnumEx(Go, GoDemosaicStyle)
    kAddEnumerator(GoDemosaicStyle, GO_DEMOSAIC_STYLE_REDUCE)
    kAddEnumerator(GoDemosaicStyle, GO_DEMOSAIC_STYLE_BILINEAR)
    kAddEnumerator(GoDemosaicStyle, GO_DEMOSAIC_STYLE_GRADIENT)
kEndEnumEx()

kBeginEnumEx(Go, GoSurfaceEncoding)
    kAddEnumerator(GoSurfaceEncoding, GO_SURFACE_ENCODING_STANDARD)
    kAddEnumerator(GoSurfaceEncoding, GO_SURFACE_ENCODING_INTERREFLECTION)
kEndEnumEx()

kBeginEnumEx(Go, GoSurfacePhaseFilter)
    kAddEnumerator(GoSurfacePhaseFilter, GO_SURFACE_PHASE_FILTER_NONE)
    kAddEnumerator(GoSurfacePhaseFilter, GO_SURFACE_PHASE_FILTER_REFLECTIVE)
    kAddEnumerator(GoSurfacePhaseFilter, GO_SURFACE_PHASE_FILTER_TRANSLUCENT)
kEndEnumEx()

kBeginEnumEx(Go, GoVoltageSetting)
    kAddEnumerator(GoVoltageSetting, GO_VOLTAGE_48)
    kAddEnumerator(GoVoltageSetting, GO_VOLTAGE_24)
kEndEnumEx()

kBeginEnumEx(Go, GoDiscoveryOpMode)
    kAddEnumerator(GoDiscoveryOpMode, GO_DISCOVERY_OP_MODE_NOT_AVAILABLE)
    kAddEnumerator(GoDiscoveryOpMode, GO_DISCOVERY_OP_MODE_STANDALONE)
    kAddEnumerator(GoDiscoveryOpMode, GO_DISCOVERY_OP_MODE_VIRTUAL)
    kAddEnumerator(GoDiscoveryOpMode, GO_DISCOVERY_OP_MODE_ACCELERATOR)
kEndEnumEx()

kBeginEnumEx(Go, GoSensorAccelState)
    kAddEnumerator(GoSensorAccelState, GO_SENSOR_ACCEL_STATE_UNKNOWN)
    kAddEnumerator(GoSensorAccelState, GO_SENSOR_ACCEL_STATE_AVAILABLE)
    kAddEnumerator(GoSensorAccelState, GO_SENSOR_ACCEL_STATE_ACCELERATED)
    kAddEnumerator(GoSensorAccelState, GO_SENSOR_ACCEL_STATE_ACCELERATED_BY_OTHER)
    kAddEnumerator(GoSensorAccelState, GO_SENSOR_ACCEL_STATE_FW_MISMATCH)
kEndEnumEx()

kBeginEnumEx(Go, GoSensorAccelStatus)
    kAddEnumerator(GoSensorAccelStatus, GO_SENSOR_ACCEL_STATUS_SUCCESS)
    kAddEnumerator(GoSensorAccelStatus, GO_SENSOR_ACCEL_STATUS_ACCELERATING)
    kAddEnumerator(GoSensorAccelStatus, GO_SENSOR_ACCEL_STATUS_DECELERATING)
    kAddEnumerator(GoSensorAccelStatus, GO_SENSOR_ACCEL_STATUS_MISSING)
    kAddEnumerator(GoSensorAccelStatus, GO_SENSOR_ACCEL_STATUS_STOPPED)
    kAddEnumerator(GoSensorAccelStatus, GO_SENSOR_ACCEL_STATUS_FAIL_TO_ACCEL)
    kAddEnumerator(GoSensorAccelStatus, GO_SENSOR_ACCEL_STATUS_STOPPED_AVAILABLE)
    kAddEnumerator(GoSensorAccelStatus, GO_SENSOR_ACCEL_STATUS_STOPPED_ACCELERATED_BY_OTHER)
    kAddEnumerator(GoSensorAccelStatus, GO_SENSOR_ACCEL_STATUS_STOPPED_FW_MISMATCH)
kEndEnumEx()

kBeginEnumEx(Go, GoSecurityLevel)
    kAddEnumerator(GoSecurityLevel, GO_SECURITY_NONE)
    kAddEnumerator(GoSecurityLevel, GO_SECURITY_BASIC)
kEndEnumEx()

kBeginEnumEx(Go, GoBrandingType)
    kAddEnumerator(GoBrandingType, GO_BRANDING_TYPE_LMI)
    kAddEnumerator(GoBrandingType, GO_BRANDING_TYPE_UNBRANDED)
    kAddEnumerator(GoBrandingType, GO_BRANDING_TYPE_CUSTOM)
kEndEnumEx()

kBeginEnumEx(Go, GoMeshMsgChannelId)
    kAddEnumerator(GoMeshMsgChannelId, GO_MESH_MSG_CHANNEL_ID_SYSTEM_VERTEX)
    kAddEnumerator(GoMeshMsgChannelId, GO_MESH_MSG_CHANNEL_ID_SYSTEM_FACET)
    kAddEnumerator(GoMeshMsgChannelId, GO_MESH_MSG_CHANNEL_ID_SYSTEM_FACET_NORMAL)
    kAddEnumerator(GoMeshMsgChannelId, GO_MESH_MSG_CHANNEL_ID_SYSTEM_VERTEX_NORMAL)
    kAddEnumerator(GoMeshMsgChannelId, GO_MESH_MSG_CHANNEL_ID_SYSTEM_VERTEX_TEXTURE)
    kAddEnumerator(GoMeshMsgChannelId, GO_MESH_MSG_CHANNEL_ID_SYSTEM_VERTEX_CURVATURE)
kEndEnumEx()

kBeginEnumEx(Go, GoMeshMsgChannelType)
    kAddEnumerator(GoMeshMsgChannelType, GO_MESH_MSG_CHANNEL_TYPE_INVALID)
    kAddEnumerator(GoMeshMsgChannelType, GO_MESH_MSG_CHANNEL_TYPE_VERTEX)
    kAddEnumerator(GoMeshMsgChannelType, GO_MESH_MSG_CHANNEL_TYPE_FACET)
    kAddEnumerator(GoMeshMsgChannelType, GO_MESH_MSG_CHANNEL_TYPE_FACET_NORMAL)
    kAddEnumerator(GoMeshMsgChannelType, GO_MESH_MSG_CHANNEL_TYPE_VERTEX_NORMAL)
    kAddEnumerator(GoMeshMsgChannelType, GO_MESH_MSG_CHANNEL_TYPE_VERTEX_TEXTURE)
    kAddEnumerator(GoMeshMsgChannelType, GO_MESH_MSG_CHANNEL_TYPE_VERTEX_CURVATURE)
kEndEnumEx()

kBeginEnumEx(Go, GoMeshMsgChannelState)
    kAddEnumerator(GoMeshMsgChannelState, GO_MESH_MSG_CHANNEL_STATE_ERROR)
    kAddEnumerator(GoMeshMsgChannelState, GO_MESH_MSG_CHANNEL_STATE_UNALLOCATED)
    kAddEnumerator(GoMeshMsgChannelState, GO_MESH_MSG_CHANNEL_STATE_ALLOCATED)
    kAddEnumerator(GoMeshMsgChannelState, GO_MESH_MSG_CHANNEL_STATE_EMPTY)
    kAddEnumerator(GoMeshMsgChannelState, GO_MESH_MSG_CHANNEL_STATE_PARTIAL)
    kAddEnumerator(GoMeshMsgChannelState, GO_MESH_MSG_CHANNEL_STATE_FULL)
kEndEnumEx()

GoFx(kVersion) GoSdk_ProtocolVersion()
{
    kVersion version = 0;

    if (kSuccess(kVersion_Parse(&version, GO_SDK_PROTOCOL_VERSION)))
    {
        return version;
    }
    else
    {
        return 0;
    }
}

GoFx(kVersion) GoSdk_Version()
{
    return kVersion_Create(GO_SDK_VERSION_MAJOR, GO_SDK_VERSION_MINOR, GO_SDK_VERSION_RELEASE, GO_SDK_VERSION_BUILD);
}

GoFx(kStatus) GoDestroy(kObject object)
{
    return kObject_Destroy(object);
}

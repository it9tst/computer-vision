/**
 * @file    GoMeasurements.x.h
 *
 * @internal
 * Copyright (C) 2016-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef GO_SDK_MEASUREMENTS_X_H
#define GO_SDK_MEASUREMENTS_X_H

#include <kApi/Data/kXml.h>
#include <GoSdk/Tools/GoExtMeasurement.h>

// Define measurement names. Definitions are ordered alphabetically.
#define GO_MEASUREMENTS_NAME_ANGLE                  "Angle"
#define GO_MEASUREMENTS_NAME_AREA                   "Area"
#define GO_MEASUREMENTS_NAME_AXIS_TILT              "AxisTilt"
#define GO_MEASUREMENTS_NAME_AXIS_ORIENTATION       "AxisOrientation"
#define GO_MEASUREMENTS_NAME_BEVEL_ANGLE            "BevelAngle"
#define GO_MEASUREMENTS_NAME_BEVEL_RADIUS           "BevelRadius"
#define GO_MEASUREMENTS_NAME_BRIDGE_VALUE           "BridgeValue"
#define GO_MEASUREMENTS_NAME_BASE_X                 "BaseX"
#define GO_MEASUREMENTS_NAME_BASE_Y                 "BaseY"
#define GO_MEASUREMENTS_NAME_BASE_Z                 "BaseZ"
#define GO_MEASUREMENTS_NAME_CENTER_X               "CenterX"
#define GO_MEASUREMENTS_NAME_CENTER_Y               "CenterY"
#define GO_MEASUREMENTS_NAME_CENTER_Z               "CenterZ"
#define GO_MEASUREMENTS_NAME_CENTROID_X             "CentroidX"
#define GO_MEASUREMENTS_NAME_CENTROID_Z             "CentroidZ"
#define GO_MEASUREMENTS_NAME_COUNTERBORE_DEPTH      "CounterboreDepth"
#define GO_MEASUREMENTS_NAME_CUSTOM                 "Custom"
#define GO_MEASUREMENTS_NAME_DEPTH                  "Depth"
#define GO_MEASUREMENTS_NAME_DISTANCE               "Distance"
#define GO_MEASUREMENTS_NAME_FLUSH                  "Flush"
#define GO_MEASUREMENTS_NAME_GAP                    "Gap"
#define GO_MEASUREMENTS_NAME_GLOBAL_X               "GlobalX"
#define GO_MEASUREMENTS_NAME_GLOBAL_Y               "GlobalY"
#define GO_MEASUREMENTS_NAME_GLOBAL_ANGLE           "GlobalAngle"
#define GO_MEASUREMENTS_NAME_GLOBAL_Z_ANGLE         "GlobalZAngle"
#define GO_MEASUREMENTS_NAME_HEIGHT                 "Height"
#define GO_MEASUREMENTS_NAME_LEFT_FLUSH_X           "LeftFlushX"
#define GO_MEASUREMENTS_NAME_LEFT_FLUSH_Z           "LeftFlushZ"
#define GO_MEASUREMENTS_NAME_LEFT_GAP_X             "LeftGapX"
#define GO_MEASUREMENTS_NAME_LEFT_GAP_Z             "LeftGapZ"
#define GO_MEASUREMENTS_NAME_LEFT_SURFACE_ANGLE     "LeftSurfaceAngle"
#define GO_MEASUREMENTS_NAME_LENGTH                 "Length"
#define GO_MEASUREMENTS_NAME_MAJOR                  "Major"
#define GO_MEASUREMENTS_NAME_MAX_ERROR              "MaxError"
#define GO_MEASUREMENTS_NAME_MAX_ERROR_X            "MaxErrorX"
#define GO_MEASUREMENTS_NAME_MAX_ERROR_Z            "MaxErrorZ"
#define GO_MEASUREMENTS_NAME_MEASUREMENT            "Measurement"
#define GO_MEASUREMENTS_NAME_MIN_ERROR              "MinError"
#define GO_MEASUREMENTS_NAME_MIN_ERROR_X            "MinErrorX"
#define GO_MEASUREMENTS_NAME_MIN_ERROR_Z            "MinErrorZ"
#define GO_MEASUREMENTS_NAME_MINOR                  "Minor"
#define GO_MEASUREMENTS_NAME_OFFSET                 "Offset"
#define GO_MEASUREMENTS_NAME_OUTER_RADIUS           "OuterRadius"
#define GO_MEASUREMENTS_NAME_OUTPUT                 "Output"
#define GO_MEASUREMENTS_NAME_PLANE_DISTANCE         "PlaneDistance"
#define GO_MEASUREMENTS_NAME_PERCENTILE             "Percentile"
#define GO_MEASUREMENTS_NAME_RADIAL_HEIGHT_MAX      "RadialHeightMax"
#define GO_MEASUREMENTS_NAME_RADIAL_HEIGHT_MEAN     "RadialHeightMean"
#define GO_MEASUREMENTS_NAME_RADIAL_HEIGHT_MIN      "RadialHeightMin"
#define GO_MEASUREMENTS_NAME_RADIAL_HEIGHT_STD_DEV  "RadialHeightStdDev"
#define GO_MEASUREMENTS_NAME_RADIAL_SLOPE_MAX       "RadialSlopeMax"
#define GO_MEASUREMENTS_NAME_RADIAL_SLOPE_MEAN      "RadialSlopeMean"
#define GO_MEASUREMENTS_NAME_RADIAL_SLOPE_MIN       "RadialSlopeMin"
#define GO_MEASUREMENTS_NAME_RADIAL_SLOPE_STD_DEV   "RadialSlopeStdDev"
#define GO_MEASUREMENTS_NAME_RADIUS                 "Radius"
#define GO_MEASUREMENTS_NAME_RATIO                  "Ratio"
#define GO_MEASUREMENTS_NAME_RIGHT_FLUSH_X          "RightFlushX"
#define GO_MEASUREMENTS_NAME_RIGHT_FLUSH_Z          "RightFlushZ"
#define GO_MEASUREMENTS_NAME_RIGHT_GAP_X            "RightGapX"
#define GO_MEASUREMENTS_NAME_RIGHT_GAP_Z            "RightGapZ"
#define GO_MEASUREMENTS_NAME_RIGHT_SURFACE_ANGLE    "RightSurfaceAngle"
#define GO_MEASUREMENTS_NAME_STD_DEV                "StdDev"
#define GO_MEASUREMENTS_NAME_THICKNESS              "Thickness"
#define GO_MEASUREMENTS_NAME_TILT_ANGLE             "TiltAngle"
#define GO_MEASUREMENTS_NAME_TILT_DIRECTION         "TiltDirection"
#define GO_MEASUREMENTS_NAME_TIP_X                  "TipX"
#define GO_MEASUREMENTS_NAME_TIP_Y                  "TipY"
#define GO_MEASUREMENTS_NAME_TIP_Z                  "TipZ"
#define GO_MEASUREMENTS_NAME_TOP_OFFSET_MIN         "TopOffsetMin"
#define GO_MEASUREMENTS_NAME_TOP_OFFSET_MAX         "TopOffsetMax"
#define GO_MEASUREMENTS_NAME_TOP_OFFSET_MEAN        "TopOffsetMean"
#define GO_MEASUREMENTS_NAME_TOP_OFFSET_STD_DEV     "TopOffsetStdDev"
#define GO_MEASUREMENTS_NAME_VALIDITY               "Validity"
#define GO_MEASUREMENTS_NAME_VOLUME                 "Volume"
#define GO_MEASUREMENTS_NAME_WIDTH                  "Width"
#define GO_MEASUREMENTS_NAME_WINDOW                 "Window"
#define GO_MEASUREMENTS_NAME_X                      "X"
#define GO_MEASUREMENTS_NAME_X_ANGLE                "XAngle"
#define GO_MEASUREMENTS_NAME_X_NORMAL               "XNormal"
#define GO_MEASUREMENTS_NAME_Y                      "Y"
#define GO_MEASUREMENTS_NAME_Y_ANGLE                "YAngle"
#define GO_MEASUREMENTS_NAME_Y_NORMAL               "YNormal"
#define GO_MEASUREMENTS_NAME_Z                      "Z"
#define GO_MEASUREMENTS_NAME_Z_ANGLE                "ZAngle"
#define GO_MEASUREMENTS_NAME_Z_NORMAL               "ZNormal"
#define GO_MEASUREMENTS_NAME_Z_OFFSET               "ZOffset"

typedef kObject GoMeasurements;

typedef struct GoMeasurementsStatic
{
    kArrayList nameTypeMapList;
} GoMeasurementsStatic;

kDeclareStaticClassEx(Go, GoMeasurements)

GoFx(kStatus) xGoMeasurements_InitStatic();
GoFx(kStatus) xGoMeasurements_ReleaseStatic();

GoFx(kStatus) GoMeasurements_NameTypeMapAdd(const char* name, const char *measurementName, kType measurementType);
GoFx(kStatus) GoMeasurements_NameTypeMapInit(void);
GoFx(kSize) GoMeasurements_NameTypeMapEntryCount(void);
GoFx(kStatus) GoMeasurements_CreateNameTypeMap(void);
GoFx(kStatus) GoMeasurements_ParseType(const kChar* toolName, const kChar* measurementName, kType* type);
GoFx(kStatus) GoMeasurements_ParseTypeHelper(const kChar* name, kType* type);
GoFx(kStatus) GoMeasurements_FormatType(kType measurement, kChar* measurementName, kSize capacity);
GoFx(kStatus) GoMeasurements_FormatTypeExtToolFormat(kType measurement, kBool internalTool, kChar* measurementName, kSize capacity);
GoFx(kStatus) GoMeasurements_FormatTypeHelper(kType measurement, const kChar** outputPtr);

typedef struct GoRangePositionZClass
{
    GoMeasurementClass base;
} GoRangePositionZClass;

kDeclareClassEx(Go, GoRangePositionZ, GoMeasurement)

GoFx(kStatus) GoRangePositionZ_VInit(GoRangePositionZ measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);


typedef struct GoRangeThicknessThicknessClass
{
    GoMeasurementClass base;
} GoRangeThicknessThicknessClass;

kDeclareClassEx(Go, GoRangeThicknessThickness, GoMeasurement)

GoFx(kStatus) GoRangeThicknessThickness_VInit(GoRangeThicknessThickness measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);


typedef struct GoProfileAreaAreaClass
{
    GoMeasurementClass base;
} GoProfileAreaAreaClass;

kDeclareClassEx(Go, GoProfileAreaArea, GoMeasurement)

GoFx(kStatus) GoProfileAreaArea_VInit(GoProfileAreaArea measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);


typedef struct GoProfileAreaCentroidXClass
{
    GoMeasurementClass base;
} GoProfileAreaCentroidXClass;

kDeclareClassEx(Go, GoProfileAreaCentroidX, GoMeasurement)

GoFx(kStatus) GoProfileAreaCentroidX_VInit(GoProfileAreaCentroidX measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);


typedef struct GoProfileAreaCentroidZClass
{
    GoMeasurementClass base;
} GoProfileAreaCentroidZClass;

kDeclareClassEx(Go, GoProfileAreaCentroidZ, GoMeasurement)

GoFx(kStatus) GoProfileAreaCentroidZ_VInit(GoProfileAreaCentroidZ measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);


typedef struct GoProfileBoxXClass
{
    GoMeasurementClass base;
} GoProfileBoxXClass;

kDeclareClassEx(Go, GoProfileBoxX, GoMeasurement)

GoFx(kStatus) GoProfileBoxX_VInit(GoProfileBoxX measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);


typedef struct GoProfileBoxZClass
{
    GoMeasurementClass base;
} GoProfileBoxZClass;

kDeclareClassEx(Go, GoProfileBoxZ, GoMeasurement)

GoFx(kStatus) GoProfileBoxZ_VInit(GoProfileBoxZ measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);


typedef struct GoProfileBoxWidthClass
{
    GoMeasurementClass base;
} GoProfileBoxWidthClass;

kDeclareClassEx(Go, GoProfileBoxWidth, GoMeasurement)

GoFx(kStatus) GoProfileBoxWidth_VInit(GoProfileBoxWidth measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);


typedef struct GoProfileBoxHeightClass
{
    GoMeasurementClass base;
} GoProfileBoxHeightClass;

kDeclareClassEx(Go, GoProfileBoxHeight, GoMeasurement)

GoFx(kStatus) GoProfileBoxHeight_VInit(GoProfileBoxHeight measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);


typedef struct GoProfileBoxGlobalXClass
{
    GoMeasurementClass base;
} GoProfileBoxGlobalXClass;

kDeclareClassEx(Go, GoProfileBoxGlobalX, GoMeasurement)

GoFx(kStatus) GoProfileBoxGlobalX_VInit(GoProfileBoxGlobalX measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);


typedef struct GoProfileBoxGlobalYClass
{
    GoMeasurementClass base;
} GoProfileBoxGlobalYClass;

kDeclareClassEx(Go, GoProfileBoxGlobalY, GoMeasurement)

GoFx(kStatus) GoProfileBoxGlobalY_VInit(GoProfileBoxGlobalY measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);


typedef struct GoProfileBoxGlobalAngleClass
{
    GoMeasurementClass base;
} GoProfileBoxGlobalAngleClass;

kDeclareClassEx(Go, GoProfileBoxGlobalAngle, GoMeasurement)

GoFx(kStatus) GoProfileBoxGlobalAngle_VInit(GoProfileBoxGlobalAngle measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);


typedef struct GoProfileBridgeValueBridgeValueClass
{
    GoMeasurementClass base;
} GoProfileBridgeValueBridgeValueClass;

kDeclareClassEx(Go, GoProfileBridgeValueBridgeValue, GoMeasurement)

GoFx(kStatus) GoProfileBridgeValueBridgeValue_VInit(GoProfileBridgeValueBridgeValue measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);


typedef struct GoProfileBridgeValueAngleClass
{
    GoMeasurementClass base;
} GoProfileBridgeValueAngleClass;

kDeclareClassEx(Go, GoProfileBridgeValueAngle, GoMeasurement)

GoFx(kStatus) GoProfileBridgeValueAngle_VInit(GoProfileBridgeValueAngle measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);

typedef struct GoProfileBridgeValueWindowClass
{
    GoMeasurementClass base;
} GoProfileBridgeValueWindowClass;

kDeclareClassEx(Go, GoProfileBridgeValueWindow, GoMeasurement)

GoFx(kStatus) GoProfileBridgeValueWindow_VInit(GoProfileBridgeValueWindow measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);

typedef struct GoProfileBridgeValueStdDevClass
{
    GoMeasurementClass base;
} GoProfileBridgeValueStdDevClass;

kDeclareClassEx(Go, GoProfileBridgeValueStdDev, GoMeasurement)

GoFx(kStatus) GoProfileBridgeValueStdDev_VInit(GoProfileBridgeValueStdDev measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);


typedef struct GoProfileCircleXClass
{
    GoMeasurementClass base;
} GoProfileCircleXClass;

kDeclareClassEx(Go, GoProfileCircleX, GoMeasurement)

GoFx(kStatus) GoProfileCircleX_VInit(GoProfileCircleX measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);


typedef struct GoProfileCircleZClass
{
    GoMeasurementClass base;
} GoProfileCircleZClass;

kDeclareClassEx(Go, GoProfileCircleZ, GoMeasurement)

GoFx(kStatus) GoProfileCircleZ_VInit(GoProfileCircleZ measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);


typedef struct GoProfileCircleRadiusClass
{
    GoMeasurementClass base;
} GoProfileCircleRadiusClass;

kDeclareClassEx(Go, GoProfileCircleRadius, GoMeasurement)

GoFx(kStatus) GoProfileCircleRadius_VInit(GoProfileCircleRadius measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);


typedef struct GoProfileCircleStdDevClass
{
    GoMeasurementClass base;
} GoProfileCircleStdDevClass;

kDeclareClassEx(Go, GoProfileCircleStdDev, GoMeasurement)

GoFx(kStatus) GoProfileCircleStdDev_VInit(GoProfileCircleStdDev measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);


typedef struct GoProfileCircleMinErrorClass
{
    GoMeasurementClass base;
} GoProfileCircleMinErrorClass;

kDeclareClassEx(Go, GoProfileCircleMinError, GoMeasurement)

GoFx(kStatus) GoProfileCircleMinError_VInit(GoProfileCircleMinError measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);


typedef struct GoProfileCircleMinErrorXClass
{
    GoMeasurementClass base;
} GoProfileCircleMinErrorXClass;

kDeclareClassEx(Go, GoProfileCircleMinErrorX, GoMeasurement)

GoFx(kStatus) GoProfileCircleMinErrorX_VInit(GoProfileCircleMinErrorX measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);


typedef struct GoProfileCircleMinErrorZClass
{
    GoMeasurementClass base;
} GoProfileCircleMinErrorZClass;

kDeclareClassEx(Go, GoProfileCircleMinErrorZ, GoMeasurement)

GoFx(kStatus) GoProfileCircleMinErrorZ_VInit(GoProfileCircleMinErrorZ measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);


typedef struct GoProfileCircleMaxErrorClass
{
    GoMeasurementClass base;
} GoProfileCircleMaxErrorClass;

kDeclareClassEx(Go, GoProfileCircleMaxError, GoMeasurement)

GoFx(kStatus) GoProfileCircleMaxError_VInit(GoProfileCircleMaxError measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);


typedef struct GoProfileCircleMaxErrorXClass
{
    GoMeasurementClass base;
} GoProfileCircleMaxErrorXClass;

kDeclareClassEx(Go, GoProfileCircleMaxErrorX, GoMeasurement)

GoFx(kStatus) GoProfileCircleMaxErrorX_VInit(GoProfileCircleMaxErrorX measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);


typedef struct GoProfileCircleMaxErrorZClass
{
    GoMeasurementClass base;
} GoProfileCircleMaxErrorZClass;

kDeclareClassEx(Go, GoProfileCircleMaxErrorZ, GoMeasurement)

GoFx(kStatus) GoProfileCircleMaxErrorZ_VInit(GoProfileCircleMaxErrorZ measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);


typedef struct GoProfileDimWidthClass
{
    GoMeasurementClass base;
    kBool absolute;
} GoProfileDimWidthClass;

kDeclareClassEx(Go, GoProfileDimWidth, GoMeasurement)

GoFx(kStatus) GoProfileDimWidth_VInit(GoProfileDimWidth measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);
GoFx(kStatus) GoProfileDimWidth_VRead(GoProfileDimWidth measurement, kXml xml, kXmlItem item);
GoFx(kStatus) GoProfileDimWidth_VWrite(GoProfileDimWidth measurement, kXml xml, kXmlItem item);


typedef struct GoProfileDimHeightClass
{
    GoMeasurementClass base;
    kBool absolute;
} GoProfileDimHeightClass;

kDeclareClassEx(Go, GoProfileDimHeight, GoMeasurement)

GoFx(kStatus) GoProfileDimHeight_VInit(GoProfileDimHeight measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);
GoFx(kStatus) GoProfileDimHeight_VRead(GoProfileDimHeight measurement, kXml xml, kXmlItem item);
GoFx(kStatus) GoProfileDimHeight_VWrite(GoProfileDimHeight measurement, kXml xml, kXmlItem item);


typedef struct GoProfileDimDistanceClass
{
    GoMeasurementClass base;
} GoProfileDimDistanceClass;

kDeclareClassEx(Go, GoProfileDimDistance, GoMeasurement)

GoFx(kStatus) GoProfileDimDistance_VInit(GoProfileDimDistance measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);


typedef struct GoProfileDimCenterXClass
{
    GoMeasurementClass base;
} GoProfileDimCenterXClass;

kDeclareClassEx(Go, GoProfileDimCenterX, GoMeasurement)

GoFx(kStatus) GoProfileDimCenterX_VInit(GoProfileDimCenterX measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);


typedef struct GoProfileDimCenterZClass
{
    GoMeasurementClass base;
} GoProfileDimCenterZClass;

kDeclareClassEx(Go, GoProfileDimCenterZ, GoMeasurement)

GoFx(kStatus) GoProfileDimCenterZ_VInit(GoProfileDimCenterZ measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);

typedef struct GoProfileRoundCornerXClass
{
    GoMeasurementClass base;
} GoProfileRoundCornerXClass;

kDeclareClassEx(Go, GoProfileRoundCornerX, GoMeasurement)

GoFx(kStatus) GoProfileRoundCornerX_VInit(GoProfileRoundCornerX measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);

typedef struct GoProfileRoundCornerZClass
{
    GoMeasurementClass base;
} GoProfileRoundCornerZClass;

kDeclareClassEx(Go, GoProfileRoundCornerZ, GoMeasurement)

GoFx(kStatus) GoProfileRoundCornerZ_VInit(GoProfileRoundCornerZ measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);

typedef struct GoProfileRoundCornerAngleClass
{
    GoMeasurementClass base;
} GoProfileRoundCornerAngleClass;

kDeclareClassEx(Go, GoProfileRoundCornerAngle, GoMeasurement)

GoFx(kStatus) GoProfileRoundCornerAngle_VInit(GoProfileRoundCornerAngle measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);

typedef struct GoProfilePositionXClass
{
    GoMeasurementClass base;
} GoProfilePositionXClass;

kDeclareClassEx(Go, GoProfilePositionX, GoMeasurement)

GoFx(kStatus) GoProfilePositionX_VInit(GoProfilePositionX measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);


typedef struct GoProfilePositionZClass
{
    GoMeasurementClass base;
} GoProfilePositionZClass;

kDeclareClassEx(Go, GoProfilePositionZ, GoMeasurement)

GoFx(kStatus) GoProfilePositionZ_VInit(GoProfilePositionZ measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);


typedef struct GoProfileIntersectXClass
{
    GoMeasurementClass base;
} GoProfileIntersectXClass;

kDeclareClassEx(Go, GoProfileIntersectX, GoMeasurement)

GoFx(kStatus) GoProfileIntersectX_VInit(GoProfileIntersectX measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);


typedef struct GoProfileIntersectZClass
{
    GoMeasurementClass base;
} GoProfileIntersectZClass;

kDeclareClassEx(Go, GoProfileIntersectZ, GoMeasurement)

GoFx(kStatus) GoProfileIntersectZ_VInit(GoProfileIntersectZ measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);


typedef struct GoProfileIntersectAngleClass
{
    GoMeasurementClass base;
    kBool range0to180Enabled;
} GoProfileIntersectAngleClass;

kDeclareClassEx(Go, GoProfileIntersectAngle, GoMeasurement)

GoFx(kStatus) GoProfileIntersectAngle_VInit(GoProfileIntersectAngle measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);
GoFx(kStatus) GoProfileIntersectAngle_VRead(GoProfileIntersectAngle measurement, kXml xml, kXmlItem item);
GoFx(kStatus) GoProfileIntersectAngle_VWrite(GoProfileIntersectAngle measurement, kXml xml, kXmlItem item);

/**
 * @deprecated Returns a boolean value representing whether an absolute measurement value will be returned.
 *
 * @public                  @memberof GoProfileIntersectAngle
 * @version                 Introduced in firmware 4.0.10.27
 * @param    measurement    GoProfileIntersectAngle object.
 * @return                  kTRUE if an absolute value will be returned. kFALSE otherwise.
 * @see                     GoProfileIntersectAngle_Range0to180Enabled
 */
GoFx(kBool) GoProfileIntersectAngle_AbsoluteEnabled(GoProfileIntersectAngle measurement);

/**
 * @deprecated Enables or disables an absolute value result for the given measurement.
 *
 * @public                  @memberof GoProfileIntersectAngle
 * @version                 Introduced in firmware 4.0.10.27
 * @param    measurement    GoProfileIntersectAngle object.
 * @param    absolute       kTRUE to enable absolute value and kFALSE to disable it.
 * @return                  Operation status.
 * @see                     GoProfileIntersectAngle_EnableRange0to180
 */
GoFx(kStatus) GoProfileIntersectAngle_EnableAbsolute(GoProfileIntersectAngle measurement, kBool absolute);


typedef struct GoProfileLineStdDevClass
{
    GoMeasurementClass base;
} GoProfileLineStdDevClass;

kDeclareClassEx(Go, GoProfileLineStdDev, GoMeasurement)

GoFx(kStatus) GoProfileLineStdDev_VInit(GoProfileLineStdDev measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);


typedef struct GoProfileLineMinErrorClass
{
    GoMeasurementClass base;
} GoProfileLineMinErrorClass;

kDeclareClassEx(Go, GoProfileLineMinError, GoMeasurement)

GoFx(kStatus) GoProfileLineMinError_VInit(GoProfileLineMinError measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);


typedef struct GoProfileLineMaxErrorClass
{
    GoMeasurementClass base;
} GoProfileLineMaxErrorClass;

kDeclareClassEx(Go, GoProfileLineMaxError, GoMeasurement)

GoFx(kStatus) GoProfileLineMaxError_VInit(GoProfileLineMaxError measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);

typedef struct GoProfileLineOffsetClass
{
    GoMeasurementClass base;
} GoProfileLineOffsetClass;

kDeclareClassEx(Go, GoProfileLineOffset, GoMeasurement)

GoFx(kStatus) GoProfileLineOffset_VInit(GoProfileLineOffset measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);

typedef struct GoProfileLineAngleClass
{
    GoMeasurementClass base;
} GoProfileLineAngleClass;

kDeclareClassEx(Go, GoProfileLineAngle, GoMeasurement)

GoFx(kStatus) GoProfileLineAngle_VInit(GoProfileLineAngle measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);

typedef struct GoProfileLineMinErrorXClass
{
    GoMeasurementClass base;
} GoProfileLineMinErrorXClass;

kDeclareClassEx(Go, GoProfileLineMinErrorX, GoMeasurement)

GoFx(kStatus) GoProfileLineMinErrorX_VInit(GoProfileLineMinErrorX measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);

typedef struct GoProfileLineMinErrorZClass
{
    GoMeasurementClass base;
} GoProfileLineMinErrorZClass;

kDeclareClassEx(Go, GoProfileLineMinErrorZ, GoMeasurement)

GoFx(kStatus) GoProfileLineMinErrorZ_VInit(GoProfileLineMinErrorZ measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);

typedef struct GoProfileLineMaxErrorXClass
{
    GoMeasurementClass base;
} GoProfileLineMaxErrorXClass;

kDeclareClassEx(Go, GoProfileLineMaxErrorX, GoMeasurement)

GoFx(kStatus) GoProfileLineMaxErrorX_VInit(GoProfileLineMaxErrorX measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);

typedef struct GoProfileLineMaxErrorZClass
{
    GoMeasurementClass base;
} GoProfileLineMaxErrorZClass;

kDeclareClassEx(Go, GoProfileLineMaxErrorZ, GoMeasurement)

GoFx(kStatus) GoProfileLineMaxErrorZ_VInit(GoProfileLineMaxErrorZ measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);

typedef struct GoProfileLinePercentileClass
{
    GoMeasurementClass base;
    k64f percent;
} GoProfileLinePercentileClass;

kDeclareClassEx(Go, GoProfileLinePercentile, GoMeasurement)

GoFx(kStatus) GoProfileLinePercentile_VInit(GoProfileLinePercentile measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);
GoFx(kStatus) GoProfileLinePercentile_VRead(GoProfileLinePercentile measurement, kXml xml, kXmlItem item);
GoFx(kStatus) GoProfileLinePercentile_VWrite(GoProfileLinePercentile measurement, kXml xml, kXmlItem item);


typedef struct GoProfilePanelGapClass
{
    GoMeasurementClass base;
    GoProfileGapAxis axis;
} GoProfilePanelGapClass;

kDeclareClassEx(Go, GoProfilePanelGap, GoMeasurement)

GoFx(kStatus) GoProfilePanelGap_VInit(GoProfilePanelGap measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);
GoFx(kStatus) GoProfilePanelGap_VRead(GoProfilePanelGap measurement, kXml xml, kXmlItem item);
GoFx(kStatus) GoProfilePanelGap_VWrite(GoProfilePanelGap measurement, kXml xml, kXmlItem item);

typedef struct GoProfilePanelFlushClass
{
    GoMeasurementClass base;
    kBool absolute;
} GoProfilePanelFlushClass;

kDeclareClassEx(Go, GoProfilePanelFlush, GoMeasurement)

GoFx(kStatus) GoProfilePanelFlush_VInit(GoProfilePanelFlush measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);
GoFx(kStatus) GoProfilePanelFlush_VRead(GoProfilePanelFlush measurement, kXml xml, kXmlItem item);
GoFx(kStatus) GoProfilePanelFlush_VWrite(GoProfilePanelFlush measurement, kXml xml, kXmlItem item);

typedef struct GoProfilePanelLeftGapXClass {
    GoMeasurementClass base;
} GoProfilePanelLeftGapXClass;

kDeclareClassEx(Go, GoProfilePanelLeftGapX, GoMeasurement)

GoFx(kStatus) GoProfilePanelLeftGapX_VInit(GoProfilePanelLeftGapX measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);
GoFx(kStatus) GoProfilePanelLeftGapX_VRead(GoProfilePanelLeftGapX measurement, kXml xml, kXmlItem item);
GoFx(kStatus) GoProfilePanelLeftGapX_VWrite(GoProfilePanelLeftGapX measurement, kXml xml, kXmlItem item);

typedef struct GoProfilePanelLeftGapZClass {
    GoMeasurementClass base;
} GoProfilePanelLeftGapZClass;

kDeclareClassEx(Go, GoProfilePanelLeftGapZ, GoMeasurement)

GoFx(kStatus) GoProfilePanelLeftGapZ_VInit(GoProfilePanelLeftGapZ measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);
GoFx(kStatus) GoProfilePanelLeftGapZ_VRead(GoProfilePanelLeftGapZ measurement, kXml xml, kXmlItem item);
GoFx(kStatus) GoProfilePanelLeftGapZ_VWrite(GoProfilePanelLeftGapZ measurement, kXml xml, kXmlItem item);

typedef struct GoProfilePanelLeftFlushXClass {
    GoMeasurementClass base;
} GoProfilePanelLeftFlushXClass;

kDeclareClassEx(Go, GoProfilePanelLeftFlushX, GoMeasurement)

GoFx(kStatus) GoProfilePanelLeftFlushX_VInit(GoProfilePanelLeftFlushX measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);
GoFx(kStatus) GoProfilePanelLeftFlushX_VRead(GoProfilePanelLeftFlushX measurement, kXml xml, kXmlItem item);
GoFx(kStatus) GoProfilePanelLeftFlushX_VWrite(GoProfilePanelLeftFlushX measurement, kXml xml, kXmlItem item);

typedef struct GoProfilePanelLeftFlushZClass {
    GoMeasurementClass base;
} GoProfilePanelLeftFlushZClass;

kDeclareClassEx(Go, GoProfilePanelLeftFlushZ, GoMeasurement)

GoFx(kStatus) GoProfilePanelLeftFlushZ_VInit(GoProfilePanelLeftFlushZ measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);
GoFx(kStatus) GoProfilePanelLeftFlushZ_VRead(GoProfilePanelLeftFlushZ measurement, kXml xml, kXmlItem item);
GoFx(kStatus) GoProfilePanelLeftFlushZ_VWrite(GoProfilePanelLeftFlushZ measurement, kXml xml, kXmlItem item);

typedef struct GoProfilePanelLeftSurfaceAngleClass {
    GoMeasurementClass base;
} GoProfilePanelLeftSurfaceAngleClass;

kDeclareClassEx(Go, GoProfilePanelLeftSurfaceAngle, GoMeasurement)

GoFx(kStatus) GoProfilePanelLeftSurfaceAngle_VInit(GoProfilePanelLeftSurfaceAngle measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);
GoFx(kStatus) GoProfilePanelLeftSurfaceAngle_VRead(GoProfilePanelLeftSurfaceAngle measurement, kXml xml, kXmlItem item);
GoFx(kStatus) GoProfilePanelLeftSurfaceAngle_VWrite(GoProfilePanelLeftSurfaceAngle measurement, kXml xml, kXmlItem item);

typedef struct GoProfilePanelRightGapXClass {
    GoMeasurementClass base;
} GoProfilePanelRightGapXClass;


kDeclareClassEx(Go, GoProfilePanelRightGapX, GoMeasurement)

GoFx(kStatus) GoProfilePanelRightGapX_VInit(GoProfilePanelRightGapX measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);
GoFx(kStatus) GoProfilePanelRightGapX_VRead(GoProfilePanelRightGapX measurement, kXml xml, kXmlItem item);
GoFx(kStatus) GoProfilePanelRightGapX_VWrite(GoProfilePanelRightGapX measurement, kXml xml, kXmlItem item);

typedef struct GoProfilePanelRightGapZClass {
    GoMeasurementClass base;
} GoProfilePanelRightGapZClass;

kDeclareClassEx(Go, GoProfilePanelRightGapZ, GoMeasurement)

GoFx(kStatus) GoProfilePanelRightGapZ_VInit(GoProfilePanelRightGapZ measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);
GoFx(kStatus) GoProfilePanelRightGapZ_VRead(GoProfilePanelRightGapZ measurement, kXml xml, kXmlItem item);
GoFx(kStatus) GoProfilePanelRightGapZ_VWrite(GoProfilePanelRightGapZ measurement, kXml xml, kXmlItem item);

typedef struct GoProfilePanelRightFlushXClass {
    GoMeasurementClass base;
} GoProfilePanelRightFlushXClass;

kDeclareClassEx(Go, GoProfilePanelRightFlushX, GoMeasurement)

GoFx(kStatus) GoProfilePanelRightFlushX_VInit(GoProfilePanelRightFlushX measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);
GoFx(kStatus) GoProfilePanelRightFlushX_VRead(GoProfilePanelRightFlushX measurement, kXml xml, kXmlItem item);
GoFx(kStatus) GoProfilePanelRightFlushX_VWrite(GoProfilePanelRightFlushX measurement, kXml xml, kXmlItem item);

typedef struct GoProfilePanelRightFlushZClass {
    GoMeasurementClass base;
} GoProfilePanelRightFlushZClass;

kDeclareClassEx(Go, GoProfilePanelRightFlushZ, GoMeasurement)

GoFx(kStatus) GoProfilePanelRightFlushZ_VInit(GoProfilePanelRightFlushZ measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);
GoFx(kStatus) GoProfilePanelRightFlushZ_VRead(GoProfilePanelRightFlushZ measurement, kXml xml, kXmlItem item);
GoFx(kStatus) GoProfilePanelRightFlushZ_VWrite(GoProfilePanelRightFlushZ measurement, kXml xml, kXmlItem item);

typedef struct GoProfilePanelRightSurfaceAngleClass {
    GoMeasurementClass base;
} GoProfilePanelRightSurfaceAngleClass;

kDeclareClassEx(Go, GoProfilePanelRightSurfaceAngle, GoMeasurement)

GoFx(kStatus) GoProfilePanelRightSurfaceAngle_VInit(GoProfilePanelRightSurfaceAngle measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);
GoFx(kStatus) GoProfilePanelRightSurfaceAngle_VRead(GoProfilePanelRightSurfaceAngle measurement, kXml xml, kXmlItem item);
GoFx(kStatus) GoProfilePanelRightSurfaceAngle_VWrite(GoProfilePanelRightSurfaceAngle measurement, kXml xml, kXmlItem item);

typedef struct GoProfileGrooveXClass
{
    GoMeasurementClass base;
    GoProfileGrooveLocation location;
    GoProfileGrooveSelectType selectType;
    k32u selectIndex;
} GoProfileGrooveXClass;

kDeclareClassEx(Go, GoProfileGrooveX, GoMeasurement)

GoFx(kStatus) GoProfileGrooveX_VInit(GoProfileGrooveX measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);
GoFx(kStatus) GoProfileGrooveX_VRead(GoProfileGrooveX measurement, kXml xml, kXmlItem item);
GoFx(kStatus) GoProfileGrooveX_VWrite(GoProfileGrooveX measurement, kXml xml, kXmlItem item);


typedef struct GoProfileGrooveZClass
{
    GoMeasurementClass base;
    GoProfileGrooveLocation location;
    GoProfileGrooveSelectType selectType;
    k32u selectIndex;
} GoProfileGrooveZClass;

kDeclareClassEx(Go, GoProfileGrooveZ, GoMeasurement)

GoFx(kStatus) GoProfileGrooveZ_VInit(GoProfileGrooveZ measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);
GoFx(kStatus) GoProfileGrooveZ_VRead(GoProfileGrooveZ measurement, kXml xml, kXmlItem item);
GoFx(kStatus) GoProfileGrooveZ_VWrite(GoProfileGrooveZ measurement, kXml xml, kXmlItem item);


typedef struct GoProfileGrooveWidthClass
{
    GoMeasurementClass base;
    GoProfileGrooveSelectType selectType;
    k32u selectIndex;
} GoProfileGrooveWidthClass;

kDeclareClassEx(Go, GoProfileGrooveWidth, GoMeasurement)

GoFx(kStatus) GoProfileGrooveWidth_VInit(GoProfileGrooveWidth measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);
GoFx(kStatus) GoProfileGrooveWidth_VRead(GoProfileGrooveWidth measurement, kXml xml, kXmlItem item);
GoFx(kStatus) GoProfileGrooveWidth_VWrite(GoProfileGrooveWidth measurement, kXml xml, kXmlItem item);


typedef struct GoProfileGrooveDepthClass
{
    GoMeasurementClass base;
    GoProfileGrooveSelectType selectType;
    k32u selectIndex;
} GoProfileGrooveDepthClass;

kDeclareClassEx(Go, GoProfileGrooveDepth, GoMeasurement)

GoFx(kStatus) GoProfileGrooveDepth_VInit(GoProfileGrooveDepth measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);
GoFx(kStatus) GoProfileGrooveDepth_VRead(GoProfileGrooveDepth measurement, kXml xml, kXmlItem item);
GoFx(kStatus) GoProfileGrooveDepth_VWrite(GoProfileGrooveDepth measurement, kXml xml, kXmlItem item);


typedef struct GoProfileStripXClass
{
    GoMeasurementClass base;
    GoProfileStripLocation location;
    GoProfileStripSelectType selectType;
    k32u selectIndex;
} GoProfileStripXClass;

kDeclareClassEx(Go, GoProfileStripX, GoMeasurement)

GoFx(kStatus) GoProfileStripX_VInit(GoProfileStripX measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);
GoFx(kStatus) GoProfileStripX_VRead(GoProfileStripX measurement, kXml xml, kXmlItem item);
GoFx(kStatus) GoProfileStripX_VWrite(GoProfileStripX measurement, kXml xml, kXmlItem item);

typedef struct GoProfileStripZClass
{
    GoMeasurementClass base;
    GoProfileStripLocation location;
    GoProfileStripSelectType selectType;
    k32u selectIndex;
} GoProfileStripZClass;

kDeclareClassEx(Go, GoProfileStripZ, GoMeasurement)

GoFx(kStatus) GoProfileStripZ_VInit(GoProfileStripZ measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);
GoFx(kStatus) GoProfileStripZ_VRead(GoProfileStripZ measurement, kXml xml, kXmlItem item);
GoFx(kStatus) GoProfileStripZ_VWrite(GoProfileStripZ measurement, kXml xml, kXmlItem item);

typedef struct GoProfileStripWidthClass
{
    GoMeasurementClass base;
    GoProfileStripSelectType selectType;
    k32u selectIndex;
} GoProfileStripWidthClass;

kDeclareClassEx(Go, GoProfileStripWidth, GoMeasurement)

GoFx(kStatus) GoProfileStripWidth_VInit(GoProfileStripWidth measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);
GoFx(kStatus) GoProfileStripWidth_VRead(GoProfileStripWidth measurement, kXml xml, kXmlItem item);
GoFx(kStatus) GoProfileStripWidth_VWrite(GoProfileStripWidth measurement, kXml xml, kXmlItem item);

typedef struct GoProfileStripHeightClass
{
    GoMeasurementClass base;
    GoProfileStripLocation location;
    GoProfileStripSelectType selectType;
    k32u selectIndex;
} GoProfileStripHeightClass;

kDeclareClassEx(Go, GoProfileStripHeight, GoMeasurement)

GoFx(kStatus) GoProfileStripHeight_VInit(GoProfileStripHeight measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);
GoFx(kStatus) GoProfileStripHeight_VRead(GoProfileStripHeight measurement, kXml xml, kXmlItem item);
GoFx(kStatus) GoProfileStripHeight_VWrite(GoProfileStripHeight measurement, kXml xml, kXmlItem item);

typedef struct GoSurfaceBoxXClass
{
    GoMeasurementClass base;
} GoSurfaceBoxXClass;

kDeclareClassEx(Go, GoSurfaceBoxX, GoMeasurement)

GoFx(kStatus) GoSurfaceBoxX_VInit(GoSurfaceBoxX measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);


typedef struct GoSurfaceBoxYClass
{
    GoMeasurementClass base;
} GoSurfaceBoxYClass;

kDeclareClassEx(Go, GoSurfaceBoxY, GoMeasurement)

GoFx(kStatus) GoSurfaceBoxY_VInit(GoSurfaceBoxY measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);


typedef struct GoSurfaceBoxZClass
{
    GoMeasurementClass base;
} GoSurfaceBoxZClass;

kDeclareClassEx(Go, GoSurfaceBoxZ, GoMeasurement)

GoFx(kStatus) GoSurfaceBoxZ_VInit(GoSurfaceBoxZ measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);

typedef struct GoSurfaceBoxZAngleClass
{
    GoMeasurementClass base;
} GoSurfaceBoxZAngleClass;

kDeclareClassEx(Go, GoSurfaceBoxZAngle, GoMeasurement)

GoFx(kStatus) GoSurfaceBoxZAngle_VInit(GoSurfaceBoxZAngle measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);


typedef struct GoSurfaceBoxWidthClass
{
    GoMeasurementClass base;
} GoSurfaceBoxWidthClass;

kDeclareClassEx(Go, GoSurfaceBoxWidth, GoMeasurement)

GoFx(kStatus) GoSurfaceBoxWidth_VInit(GoSurfaceBoxWidth measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);


typedef struct GoSurfaceBoxLengthClass
{
    GoMeasurementClass base;
} GoSurfaceBoxLengthClass;

kDeclareClassEx(Go, GoSurfaceBoxLength, GoMeasurement)

GoFx(kStatus) GoSurfaceBoxLength_VInit(GoSurfaceBoxLength measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);


typedef struct GoSurfaceBoxHeightClass
{
    GoMeasurementClass base;
} GoSurfaceBoxHeightClass;

kDeclareClassEx(Go, GoSurfaceBoxHeight, GoMeasurement)

GoFx(kStatus) GoSurfaceBoxHeight_VInit(GoSurfaceBoxHeight measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);


typedef struct GoSurfaceBoxGlobalXClass
{
    GoMeasurementClass base;
} GoSurfaceBoxGlobalXClass;

kDeclareClassEx(Go, GoSurfaceBoxGlobalX, GoMeasurement)

GoFx(kStatus) GoSurfaceBoxGlobalX_VInit(GoSurfaceBoxGlobalX measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);


typedef struct GoSurfaceBoxGlobalYClass
{
    GoMeasurementClass base;
} GoSurfaceBoxGlobalYClass;

kDeclareClassEx(Go, GoSurfaceBoxGlobalY, GoMeasurement)

GoFx(kStatus) GoSurfaceBoxGlobalY_VInit(GoSurfaceBoxGlobalY measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);


typedef struct GoSurfaceBoxGlobalZAngleClass
{
    GoMeasurementClass base;
} GoSurfaceBoxGlobalZAngleClass;

kDeclareClassEx(Go, GoSurfaceBoxGlobalZAngle, GoMeasurement)

GoFx(kStatus) GoSurfaceBoxGlobalZAngle_VInit(GoSurfaceBoxGlobalZAngle measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);


typedef struct GoSurfaceCountersunkHoleXClass
{
    GoMeasurementClass base;
} GoSurfaceCountersunkHoleXClass;

kDeclareClassEx(Go, GoSurfaceCountersunkHoleX, GoMeasurement)

GoFx(kStatus) GoSurfaceCountersunkHoleX_VInit(GoSurfaceCountersunkHoleX measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);


typedef struct GoSurfaceCountersunkHoleXAngleClass
{
    GoMeasurementClass base;
} GoSurfaceCountersunkHoleXAngleClass;

kDeclareClassEx(Go, GoSurfaceCountersunkHoleXAngle, GoMeasurement)

GoFx(kStatus) GoSurfaceCountersunkHoleXAngle_VInit(GoSurfaceCountersunkHoleXAngle measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);


typedef struct GoSurfaceCountersunkHoleYClass
{
    GoMeasurementClass base;
} GoSurfaceCountersunkHoleYClass;

kDeclareClassEx(Go, GoSurfaceCountersunkHoleY, GoMeasurement)

GoFx(kStatus) GoSurfaceCountersunkHoleY_VInit(GoSurfaceCountersunkHoleY measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);


typedef struct GoSurfaceCountersunkHoleYAngleClass
{
    GoMeasurementClass base;
} GoSurfaceCountersunkHoleYAngleClass;

kDeclareClassEx(Go, GoSurfaceCountersunkHoleYAngle, GoMeasurement)

GoFx(kStatus) GoSurfaceCountersunkHoleYAngle_VInit(GoSurfaceCountersunkHoleYAngle measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);


typedef struct GoSurfaceCountersunkHoleZClass
{
    GoMeasurementClass base;
} GoSurfaceCountersunkHoleZClass;

kDeclareClassEx(Go, GoSurfaceCountersunkHoleZ, GoMeasurement)

GoFx(kStatus) GoSurfaceCountersunkHoleZ_VInit(GoSurfaceCountersunkHoleZ measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);


typedef struct GoSurfaceCountersunkHoleDepthClass
{
    GoMeasurementClass base;
} GoSurfaceCountersunkHoleDepthClass;

kDeclareClassEx(Go, GoSurfaceCountersunkHoleDepth, GoMeasurement)

GoFx(kStatus) GoSurfaceCountersunkHoleDepth_VInit(GoSurfaceCountersunkHoleDepth measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);


typedef struct GoSurfaceCountersunkHoleCounterboreDepthClass
{
    GoMeasurementClass base;
} GoSurfaceCountersunkHoleCounterboreDepthClass;

kDeclareClassEx(Go, GoSurfaceCountersunkHoleCounterboreDepth, GoMeasurement)

GoFx(kStatus) GoSurfaceCountersunkHoleCounterboreDepth_VInit(GoSurfaceCountersunkHoleCounterboreDepth measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);


typedef struct GoSurfaceCountersunkHoleOuterRadiusClass
{
    GoMeasurementClass base;
} GoSurfaceCountersunkHoleOuterRadiusClass;

kDeclareClassEx(Go, GoSurfaceCountersunkHoleOuterRadius, GoMeasurement)

GoFx(kStatus) GoSurfaceCountersunkHoleOuterRadius_VInit(GoSurfaceCountersunkHoleOuterRadius measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);


typedef struct GoSurfaceCountersunkHoleBevelAngleClass
{
    GoMeasurementClass base;
} GoSurfaceCountersunkHoleBevelAngleClass;

kDeclareClassEx(Go, GoSurfaceCountersunkHoleBevelAngle, GoMeasurement)

GoFx(kStatus) GoSurfaceCountersunkHoleBevelAngle_VInit(GoSurfaceCountersunkHoleBevelAngle measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);


typedef struct GoSurfaceCountersunkHoleBevelRadiusClass
{
    GoMeasurementClass base;
} GoSurfaceCountersunkHoleBevelRadiusClass;

kDeclareClassEx(Go, GoSurfaceCountersunkHoleBevelRadius, GoMeasurement)

GoFx(kStatus) GoSurfaceCountersunkHoleBevelRadius_VInit(GoSurfaceCountersunkHoleBevelRadius measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);


typedef struct GoSurfaceCountersunkHoleAxisTiltClass
{
    GoMeasurementClass base;
} GoSurfaceCountersunkHoleAxisTiltClass;

kDeclareClassEx(Go, GoSurfaceCountersunkHoleAxisTilt, GoMeasurement)

GoFx(kStatus) GoSurfaceCountersunkHoleAxisTilt_VInit(GoSurfaceCountersunkHoleAxisTilt measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);


typedef struct GoSurfaceCountersunkHoleAxisOrientationClass
{
    GoMeasurementClass base;
} GoSurfaceCountersunkHoleAxisOrientationClass;

kDeclareClassEx(Go, GoSurfaceCountersunkHoleAxisOrientation, GoMeasurement)

GoFx(kStatus) GoSurfaceCountersunkHoleAxisOrientation_VInit(GoSurfaceCountersunkHoleAxisOrientation measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);


typedef struct GoSurfaceDimWidthClass
{
    GoMeasurementClass base;
    kBool absolute;
} GoSurfaceDimWidthClass;

kDeclareClassEx(Go, GoSurfaceDimWidth, GoMeasurement)

GoFx(kStatus) GoSurfaceDimWidth_VInit(GoSurfaceDimWidth measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);
GoFx(kStatus) GoSurfaceDimWidth_VRead(GoSurfaceDimWidth measurement, kXml xml, kXmlItem item);
GoFx(kStatus) GoSurfaceDimWidth_VWrite(GoSurfaceDimWidth measurement, kXml xml, kXmlItem item);


typedef struct GoSurfaceDimHeightClass
{
    GoMeasurementClass base;
    kBool absolute;
} GoSurfaceDimHeightClass;

kDeclareClassEx(Go, GoSurfaceDimHeight, GoMeasurement)

GoFx(kStatus) GoSurfaceDimHeight_VInit(GoSurfaceDimHeight measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);
GoFx(kStatus) GoSurfaceDimHeight_VRead(GoSurfaceDimHeight measurement, kXml xml, kXmlItem item);
GoFx(kStatus) GoSurfaceDimHeight_VWrite(GoSurfaceDimHeight measurement, kXml xml, kXmlItem item);


typedef struct GoSurfaceDimLengthClass
{
    GoMeasurementClass base;
    kBool absolute;
} GoSurfaceDimLengthClass;

kDeclareClassEx(Go, GoSurfaceDimLength, GoMeasurement)

GoFx(kStatus) GoSurfaceDimLength_VInit(GoSurfaceDimLength measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);
GoFx(kStatus) GoSurfaceDimLength_VRead(GoSurfaceDimLength measurement, kXml xml, kXmlItem item);
GoFx(kStatus) GoSurfaceDimLength_VWrite(GoSurfaceDimLength measurement, kXml xml, kXmlItem item);


typedef struct GoSurfaceDimDistanceClass
{
    GoMeasurementClass base;
} GoSurfaceDimDistanceClass;

kDeclareClassEx(Go, GoSurfaceDimDistance, GoMeasurement)

GoFx(kStatus) GoSurfaceDimDistance_VInit(GoSurfaceDimDistance measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);

typedef struct GoSurfaceDimPlaneDistanceClass
{
    GoMeasurementClass base;
} GoSurfaceDimPlaneDistanceClass;

kDeclareClassEx(Go, GoSurfaceDimPlaneDistance, GoMeasurement)

GoFx(kStatus) GoSurfaceDimPlaneDistance_VInit(GoSurfaceDimPlaneDistance measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);


typedef struct GoSurfaceDimCenterXClass
{
    GoMeasurementClass base;
} GoSurfaceDimCenterXClass;

kDeclareClassEx(Go, GoSurfaceDimCenterX, GoMeasurement)

GoFx(kStatus) GoSurfaceDimCenterX_VInit(GoSurfaceDimCenterX measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);


typedef struct GoSurfaceDimCenterYClass
{
    GoMeasurementClass base;
} GoSurfaceDimCenterYClass;

kDeclareClassEx(Go, GoSurfaceDimCenterY, GoMeasurement)

GoFx(kStatus) GoSurfaceDimCenterY_VInit(GoSurfaceDimCenterY measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);


typedef struct GoSurfaceDimCenterZClass
{
    GoMeasurementClass base;
} GoSurfaceDimCenterZClass;

kDeclareClassEx(Go, GoSurfaceDimCenterZ, GoMeasurement)

GoFx(kStatus) GoSurfaceDimCenterZ_VInit(GoSurfaceDimCenterZ measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);


typedef struct GoSurfaceEllipseMajorClass
{
    GoMeasurementClass base;
} GoSurfaceEllipseMajorClass;

kDeclareClassEx(Go, GoSurfaceEllipseMajor, GoMeasurement)

GoFx(kStatus) GoSurfaceEllipseMajor_VInit(GoSurfaceEllipseMajor measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);


typedef struct GoSurfaceEllipseMinorClass
{
    GoMeasurementClass base;
} GoSurfaceEllipseMinorClass;

kDeclareClassEx(Go, GoSurfaceEllipseMinor, GoMeasurement)

GoFx(kStatus) GoSurfaceEllipseMinor_VInit(GoSurfaceEllipseMinor measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);


typedef struct GoSurfaceEllipseRatioClass
{
    GoMeasurementClass base;
} GoSurfaceEllipseRatioClass;

kDeclareClassEx(Go, GoSurfaceEllipseRatio, GoMeasurement)

GoFx(kStatus) GoSurfaceEllipseRatio_VInit(GoSurfaceEllipseRatio measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);


typedef struct GoSurfaceEllipseZAngleClass
{
    GoMeasurementClass base;
} GoSurfaceEllipseZAngleClass;

kDeclareClassEx(Go, GoSurfaceEllipseZAngle, GoMeasurement)

GoFx(kStatus) GoSurfaceEllipseZAngle_VInit(GoSurfaceEllipseZAngle measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);


typedef struct GoSurfaceHoleXClass
{
    GoMeasurementClass base;
} GoSurfaceHoleXClass;

kDeclareClassEx(Go, GoSurfaceHoleX, GoMeasurement)

GoFx(kStatus) GoSurfaceHoleX_VInit(GoSurfaceHoleX measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);


typedef struct GoSurfaceHoleYClass
{
    GoMeasurementClass base;
} GoSurfaceHoleYClass;

kDeclareClassEx(Go, GoSurfaceHoleY, GoMeasurement)

GoFx(kStatus) GoSurfaceHoleY_VInit(GoSurfaceHoleY measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);


typedef struct GoSurfaceHoleZClass
{
    GoMeasurementClass base;
} GoSurfaceHoleZClass;

kDeclareClassEx(Go, GoSurfaceHoleZ, GoMeasurement)

GoFx(kStatus) GoSurfaceHoleZ_VInit(GoSurfaceHoleZ measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);


typedef struct GoSurfaceHoleRadiusClass
{
    GoMeasurementClass base;
} GoSurfaceHoleRadiusClass;

kDeclareClassEx(Go, GoSurfaceHoleRadius, GoMeasurement)

GoFx(kStatus) GoSurfaceHoleRadius_VInit(GoSurfaceHoleRadius measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);


typedef struct GoSurfaceOpeningXClass
{
    GoMeasurementClass base;
} GoSurfaceOpeningXClass;

kDeclareClassEx(Go, GoSurfaceOpeningX, GoMeasurement)

GoFx(kStatus) GoSurfaceOpeningX_VInit(GoSurfaceOpeningX measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);


typedef struct GoSurfaceOpeningYClass
{
    GoMeasurementClass base;
} GoSurfaceOpeningYClass;

kDeclareClassEx(Go, GoSurfaceOpeningY, GoMeasurement)

GoFx(kStatus) GoSurfaceOpeningY_VInit(GoSurfaceOpeningY measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);


typedef struct GoSurfaceOpeningZClass
{
    GoMeasurementClass base;
} GoSurfaceOpeningZClass;

kDeclareClassEx(Go, GoSurfaceOpeningZ, GoMeasurement)

GoFx(kStatus) GoSurfaceOpeningZ_VInit(GoSurfaceOpeningZ measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);


typedef struct GoSurfaceOpeningWidthClass
{
    GoMeasurementClass base;
} GoSurfaceOpeningWidthClass;

kDeclareClassEx(Go, GoSurfaceOpeningWidth, GoMeasurement)

GoFx(kStatus) GoSurfaceOpeningWidth_VInit(GoSurfaceOpeningWidth measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);


typedef struct GoSurfaceOpeningLengthClass
{
    GoMeasurementClass base;
} GoSurfaceOpeningLengthClass;

kDeclareClassEx(Go, GoSurfaceOpeningLength, GoMeasurement)

GoFx(kStatus) GoSurfaceOpeningLength_VInit(GoSurfaceOpeningLength measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);


typedef struct GoSurfaceOpeningAngleClass
{
    GoMeasurementClass base;
} GoSurfaceOpeningAngleClass;

kDeclareClassEx(Go, GoSurfaceOpeningAngle, GoMeasurement)

GoFx(kStatus) GoSurfaceOpeningAngle_VInit(GoSurfaceOpeningAngle measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);


typedef struct GoSurfacePlaneXAngleClass
{
    GoMeasurementClass base;
} GoSurfacePlaneXAngleClass;

kDeclareClassEx(Go, GoSurfacePlaneXAngle, GoMeasurement)

GoFx(kStatus) GoSurfacePlaneXAngle_VInit(GoSurfacePlaneXAngle measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);


typedef struct GoSurfacePlaneYAngleClass
{
    GoMeasurementClass base;
} GoSurfacePlaneYAngleClass;

kDeclareClassEx(Go, GoSurfacePlaneYAngle, GoMeasurement)

GoFx(kStatus) GoSurfacePlaneYAngle_VInit(GoSurfacePlaneYAngle measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);


typedef struct GoSurfacePlaneZOffsetClass
{
    GoMeasurementClass base;
} GoSurfacePlaneZOffsetClass;

kDeclareClassEx(Go, GoSurfacePlaneZOffset, GoMeasurement)

GoFx(kStatus) GoSurfacePlaneZOffset_VInit(GoSurfacePlaneZOffset measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);


typedef struct GoSurfacePlaneStdDevClass
{
    GoMeasurementClass base;
} GoSurfacePlaneStdDevClass;

kDeclareClassEx(Go, GoSurfacePlaneStdDev, GoMeasurement)

GoFx(kStatus) GoSurfacePlaneStdDev_VInit(GoSurfacePlaneStdDev measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);


typedef struct GoSurfacePlaneMinErrorClass
{
    GoMeasurementClass base;
} GoSurfacePlaneMinErrorClass;

kDeclareClassEx(Go, GoSurfacePlaneMinError, GoMeasurement)

GoFx(kStatus) GoSurfacePlaneMinError_VInit(GoSurfacePlaneMinError measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);


typedef struct GoSurfacePlaneMaxErrorClass
{
    GoMeasurementClass base;
} GoSurfacePlaneMaxErrorClass;

kDeclareClassEx(Go, GoSurfacePlaneMaxError, GoMeasurement)

GoFx(kStatus) GoSurfacePlaneMaxError_VInit(GoSurfacePlaneMaxError measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);


typedef struct GoSurfacePlaneXNormalClass
{
    GoMeasurementClass base;
} GoSurfacePlaneXNormalClass;

kDeclareClassEx(Go, GoSurfacePlaneXNormal, GoMeasurement)

GoFx(kStatus) GoSurfacePlaneXNormal_VInit(GoSurfacePlaneXNormal measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);


typedef struct GoSurfacePlaneYNormalClass
{
    GoMeasurementClass base;
} GoSurfacePlaneYNormalClass;

kDeclareClassEx(Go, GoSurfacePlaneYNormal, GoMeasurement)

GoFx(kStatus) GoSurfacePlaneYNormal_VInit(GoSurfacePlaneYNormal measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);


typedef struct GoSurfacePlaneZNormalClass
{
    GoMeasurementClass base;
} GoSurfacePlaneZNormalClass;

kDeclareClassEx(Go, GoSurfacePlaneZNormal, GoMeasurement)

GoFx(kStatus) GoSurfacePlaneZNormal_VInit(GoSurfacePlaneZNormal measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);


typedef struct GoSurfacePlaneDistanceClass
{
    GoMeasurementClass base;
} GoSurfacePlaneDistanceClass;

kDeclareClassEx(Go, GoSurfacePlaneDistance, GoMeasurement)

GoFx(kStatus) GoSurfacePlaneDistance_VInit(GoSurfacePlaneDistance measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);


typedef struct GoSurfacePositionXClass
{
    GoMeasurementClass base;
} GoSurfacePositionXClass;

kDeclareClassEx(Go, GoSurfacePositionX, GoMeasurement)

GoFx(kStatus) GoSurfacePositionX_VInit(GoSurfacePositionX measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);


typedef struct GoSurfacePositionYClass
{
    GoMeasurementClass base;
} GoSurfacePositionYClass;

kDeclareClassEx(Go, GoSurfacePositionY, GoMeasurement)

GoFx(kStatus) GoSurfacePositionY_VInit(GoSurfacePositionY measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);


typedef struct GoSurfacePositionZClass
{
    GoMeasurementClass base;
} GoSurfacePositionZClass;

kDeclareClassEx(Go, GoSurfacePositionZ, GoMeasurement)

GoFx(kStatus) GoSurfacePositionZ_VInit(GoSurfacePositionZ measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);



/*
#define  GoDimesion_Cast_(MEASUREMENT,CONTEXT)    kCastClass_(MEASUREMENT, CONTEXT) \
#define DECLARE_MEASUREMENT(MEASUREMENT)                 \
typedef struct MEASUREMENT##Class                   \
{                                                   \
GoMeasurementClass base;                        \
} MEASUREMENT##Class;                               \
kDeclareClassEx(Go,  MEASUREMENT, GoMeasurement)      \
GoDimesion_Cast_(MEASUREMENT)                       \
GoFx(kStatus)  MEASUREMENT##ClassVint(MEASUREMENT measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);

DECLARE_MEASUREMENT(GoDimesionWidth)
*/

/**
 * @class   GoSurfaceRivetX
 * @extends GoMeasurement
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents an X measurement for a Surface Rivet Tool.
 */
typedef GoMeasurement GoSurfaceRivetX;

/**
 * @class   GoSurfaceRivetY
 * @extends GoMeasurement
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents a Y measurement for a Surface Rivet Tool.
 */
typedef GoMeasurement GoSurfaceRivetY;

/**
 * @class   GoSurfaceRivetZ
 * @extends GoMeasurement
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents a Z measurement for a Surface Rivet Tool.
 */
typedef GoMeasurement GoSurfaceRivetZ;

/**
 * @class   GoSurfaceRivetTiltAngle
 * @extends GoMeasurement
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents an X angle measurement for a Surface Rivet Tool.
 */
typedef GoMeasurement GoSurfaceRivetTiltAngle;

/**
 * @class   GoSurfaceRivetTiltDirection
 * @extends GoMeasurement
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents an Y angle measurement for a Surface Rivet Tool.
 */
typedef GoMeasurement GoSurfaceRivetTiltDirection;

/**
 * @class   GoSurfaceRivetRadius
 * @extends GoMeasurement
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents a radius measurement for a Surface Rivet Tool.
 */
typedef GoMeasurement GoSurfaceRivetRadius;

/**
 * @class   GoSurfaceRivetTopOffsetMin
 * @extends GoMeasurement
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents a top offset minimum measurement for a Surface Rivet Tool.
 */
typedef GoMeasurement GoSurfaceRivetTopOffsetMin;

/**
 * @class   GoSurfaceRivetTopOffsetMax
 * @extends GoMeasurement
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents a top offset maximum measurement for a Surface Rivet Tool.
 */
typedef GoMeasurement GoSurfaceRivetTopOffsetMax;

/**
 * @class   GoSurfaceRivetTopOffsetMean
 * @extends GoMeasurement
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents a top offset mean measurement for a Surface Rivet Tool.
 */
typedef GoMeasurement GoSurfaceRivetTopOffsetMean;

/**
 * @class   GoSurfaceRivetTopOffsetStdDev
 * @extends GoMeasurement
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents a top offset standard deviation measurement for a Surface Rivet Tool.
 */
typedef GoMeasurement GoSurfaceRivetTopOffsetStdDev;

/**
 * @class   GoSurfaceRivetRadialHeightMin
 * @extends GoMeasurement
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents a radial height minimum measurement for a Surface Rivet Tool.
 */
typedef GoMeasurement GoSurfaceRivetRadialHeightMin;

/**
 * Gets the radius.
 *
 * @public                      @memberof GoSurfaceRivet
 * @param    measurement        GoSurfaceRivetRadialHeightMin object.
 * @return                      The surface location.
 */
GoFx(k64f) GoSurfaceRivetRadialHeightMin_Radius(GoSurfaceRivetRadialHeightMin measurement);

/**
 * Sets the radius.
 *
 * @public                      @memberof GoSurfaceRivetRadialHeightMin
 * @param    measurement        GoSurfaceRivetRadialHeightMin object.
 * @param    value              The offset value to set.
 * @return                      Operation status.
 */
GoFx(kStatus) GoSurfaceRivetRadialHeightMin_SetRadius(GoSurfaceRivetRadialHeightMin measurement, k64f value);


/**
 * @class   GoSurfaceRivetRadialHeightMax
 * @extends GoMeasurement
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents a radial height maximum measurement for a Surface Rivet Tool.
 */
typedef GoMeasurement GoSurfaceRivetRadialHeightMax;

/**
 * Gets the radius.
 *
 * @public                      @memberof GoSurfaceRivet
 * @param    measurement        GoSurfaceRivetRadialHeightMax object.
 * @return                      The surface location.
 */
GoFx(k64f) GoSurfaceRivetRadialHeightMax_Radius(GoSurfaceRivetRadialHeightMax measurement);

/**
 * Sets the radius.
 *
 * @public                      @memberof GoSurfaceRivetRadialHeightMax
 * @param    measurement        GoSurfaceRivetRadialHeightMax object.
 * @param    value              The offset value to set.
 * @return                      Operation status.
 */
GoFx(kStatus) GoSurfaceRivetRadialHeightMax_SetRadius(GoSurfaceRivetRadialHeightMax measurement, k64f value);


/**
 * @class   GoSurfaceRivetRadialHeightMean
 * @extends GoMeasurement
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents a radial height mean measurement for a Surface Rivet Tool.
 */
typedef GoMeasurement GoSurfaceRivetRadialHeightMean;

/**
 * Gets the radius.
 *
 * @public                      @memberof GoSurfaceRivet
 * @param    measurement        GoSurfaceRivetRadialHeightMean object.
 * @return                      The surface location.
 */
GoFx(k64f) GoSurfaceRivetRadialHeightMean_Radius(GoSurfaceRivetRadialHeightMean measurement);

/**
 * Sets the radius.
 *
 * @public                      @memberof GoSurfaceRivetRadialHeightMean
 * @param    measurement        GoSurfaceRivetRadialHeightMean object.
 * @param    value              The offset value to set.
 * @return                      Operation status.
 */
GoFx(kStatus) GoSurfaceRivetRadialHeightMean_SetRadius(GoSurfaceRivetRadialHeightMean measurement, k64f value);

/**
 * @class   GoSurfaceRivetRadialHeightStdDev
 * @extends GoMeasurement
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents a radial height standard deviation measurement for a Surface Rivet Tool.
 */
typedef GoMeasurement GoSurfaceRivetRadialHeightStdDev;

/**
 * Gets the radius.
 *
 * @public                      @memberof GoSurfaceRivet
 * @param    measurement        GoSurfaceRivetRadialHeightStdDev object.
 * @return                      The surface location.
 */
GoFx(k64f) GoSurfaceRivetRadialHeightStdDev_Radius(GoSurfaceRivetRadialHeightStdDev measurement);

/**
 * Sets the radius.
 *
 * @public                      @memberof GoSurfaceRivetRadialHeightStdDev
 * @param    measurement        GoSurfaceRivetRadialHeightStdDev object.
 * @param    value              The offset value to set.
 * @return                      Operation status.
 */
GoFx(kStatus) GoSurfaceRivetRadialHeightStdDev_SetRadius(GoSurfaceRivetRadialHeightStdDev measurement, k64f value);


/**
 * @class   GoSurfaceRivetRadialSlopeMin
 * @extends GoMeasurement
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents a radial slope minimum measurement for a Surface Rivet Tool.
 */
typedef GoMeasurement GoSurfaceRivetRadialSlopeMin;

/**
 * Gets the radius.
 *
 * @public                      @memberof GoSurfaceRivet
 * @param    measurement        GoSurfaceRivetRadialSlopeMin object.
 * @return                      The surface location.
 */
GoFx(k64f) GoSurfaceRivetRadialSlopeMin_Radius(GoSurfaceRivetRadialSlopeMin measurement);

/**
 * Sets the radius.
 *
 * @public                      @memberof GoSurfaceRivetRadialSlopeMin
 * @param    measurement        GoSurfaceRivetRadialSlopeMin object.
 * @param    value              The offset value to set.
 * @return                      Operation status.
 */
GoFx(kStatus) GoSurfaceRivetRadialSlopeMin_SetRadius(GoSurfaceRivetRadialSlopeMin measurement, k64f value);


/**
 * @class   GoSurfaceRivetRadialSlopeMax
 * @extends GoMeasurement
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents a radial slope maximum measurement for a Surface Rivet Tool.
 */
typedef GoMeasurement GoSurfaceRivetRadialSlopeMax;

/**
 * Gets the radius.
 *
 * @public                      @memberof GoSurfaceRivet
 * @param    measurement        GoSurfaceRivetRadialSlopeMax object.
 * @return                      The surface location.
 */
GoFx(k64f) GoSurfaceRivetRadialSlopeMax_Radius(GoSurfaceRivetRadialSlopeMax measurement);

/**
 * Sets the radius.
 *
 * @public                      @memberof GoSurfaceRivetRadialSlopeMax
 * @param    measurement        GoSurfaceRivetRadialSlopeMax object.
 * @param    value              The offset value to set.
 * @return                      Operation status.
 */
GoFx(kStatus) GoSurfaceRivetRadialSlopeMax_SetRadius(GoSurfaceRivetRadialSlopeMax measurement, k64f value);

/**
 * @class   GoSurfaceRivetRadialSlopeMean
 * @extends GoMeasurement
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents a radial slope mean measurement for a Surface Rivet Tool.
 */
typedef GoMeasurement GoSurfaceRivetRadialSlopeMean;

/**
 * Gets the radius.
 *
 * @public                      @memberof GoSurfaceRivet
 * @param    measurement        GoSurfaceRivetRadialSlopeMean object.
 * @return                      The surface location.
 */
GoFx(k64f) GoSurfaceRivetRadialSlopeMean_Radius(GoSurfaceRivetRadialSlopeMean measurement);

/**
 * Sets the radius.
 *
 * @public                      @memberof GoSurfaceRivetRadialSlopeMean
 * @param    measurement        GoSurfaceRivetRadialSlopeMean object.
 * @param    value              The offset value to set.
 * @return                      Operation status.
 */
GoFx(kStatus) GoSurfaceRivetRadialSlopeMean_SetRadius(GoSurfaceRivetRadialSlopeMean measurement, k64f value);


/**
 * @class   GoSurfaceRivetRadialSlopeStdDev
 * @extends GoMeasurement
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents a radial slope standard deviation measurement for a Surface Rivet Tool.
 */
typedef GoMeasurement GoSurfaceRivetRadialSlopeStdDev;

/**
 * Gets the radius.
 *
 * @public                      @memberof GoSurfaceRivet
 * @param    measurement        GoSurfaceRivetRadialSlopeStdDev object.
 * @return                      The surface location.
 */
GoFx(k64f) GoSurfaceRivetRadialSlopeStdDev_Radius(GoSurfaceRivetRadialSlopeStdDev measurement);

/**
 * Sets the radius.
 *
 * @public                      @memberof GoSurfaceRivetRadialSlopeStdDev
 * @param    measurement        GoSurfaceRivetRadialSlopeStdDev object.
 * @param    value              The offset value to set.
 * @return                      Operation status.
 */
GoFx(kStatus) GoSurfaceRivetRadialSlopeStdDev_SetRadius(GoSurfaceRivetRadialSlopeStdDev measurement, k64f value);

typedef struct GoSurfaceRivetXClass
{
    GoMeasurementClass base;
} GoSurfaceRivetXClass;

kDeclareClassEx(Go, GoSurfaceRivetX, GoMeasurement)

GoFx(kStatus) GoSurfaceRivetX_VInit(GoSurfaceRivetX measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);


typedef struct GoSurfaceRivetYClass
{
    GoMeasurementClass base;
} GoSurfaceRivetYClass;

kDeclareClassEx(Go, GoSurfaceRivetY, GoMeasurement)

GoFx(kStatus) GoSurfaceRivetY_VInit(GoSurfaceRivetY measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);


typedef struct GoSurfaceRivetZClass
{
    GoMeasurementClass base;
} GoSurfaceRivetZClass;

kDeclareClassEx(Go, GoSurfaceRivetZ, GoMeasurement)

GoFx(kStatus) GoSurfaceRivetZ_VInit(GoSurfaceRivetZ measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);


typedef struct GoSurfaceRivetTiltAngleClass
{
    GoMeasurementClass base;
} GoSurfaceRivetTiltAngleClass;

kDeclareClassEx(Go, GoSurfaceRivetTiltAngle, GoMeasurement)

GoFx(kStatus) GoSurfaceRivetTiltAngle_VInit(GoSurfaceRivetTiltAngle measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);


typedef struct GoSurfaceRivetTiltDirectionClass
{
    GoMeasurementClass base;
} GoSurfaceRivetTiltDirectionClass;

kDeclareClassEx(Go, GoSurfaceRivetTiltDirection, GoMeasurement)

GoFx(kStatus) GoSurfaceRivetTiltDirection_VInit(GoSurfaceRivetTiltDirection measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);


typedef struct GoSurfaceRivetRadiusClass
{
    GoMeasurementClass base;
} GoSurfaceRivetRadiusClass;

kDeclareClassEx(Go, GoSurfaceRivetRadius, GoMeasurement)

GoFx(kStatus) GoSurfaceRivetRadius_VInit(GoSurfaceRivetRadius measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);

typedef struct GoSurfaceRivetTopOffsetMinClass
{
    GoMeasurementClass base;
} GoSurfaceRivetTopOffsetMinClass;

kDeclareClassEx(Go, GoSurfaceRivetTopOffsetMin, GoMeasurement)

GoFx(kStatus) GoSurfaceRivetTopOffsetMin_VInit(GoSurfaceRivetTopOffsetMin measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);

typedef struct GoSurfaceRivetTopOffsetMaxClass
{
    GoMeasurementClass base;
} GoSurfaceRivetTopOffsetMaxClass;

kDeclareClassEx(Go, GoSurfaceRivetTopOffsetMax, GoMeasurement)

GoFx(kStatus) GoSurfaceRivetTopOffsetMax_VInit(GoSurfaceRivetTopOffsetMax measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);

typedef struct GoSurfaceRivetTopOffsetMeanClass
{
    GoMeasurementClass base;
} GoSurfaceRivetTopOffsetMeanClass;

kDeclareClassEx(Go, GoSurfaceRivetTopOffsetMean, GoMeasurement)

GoFx(kStatus) GoSurfaceRivetTopOffsetMean_VInit(GoSurfaceRivetTopOffsetMean measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);

typedef struct GoSurfaceRivetTopOffsetStdDevClass
{
    GoMeasurementClass base;
} GoSurfaceRivetTopOffsetStdDevClass;

kDeclareClassEx(Go, GoSurfaceRivetTopOffsetStdDev, GoMeasurement)

GoFx(kStatus) GoSurfaceRivetTopOffsetStdDev_VInit(GoSurfaceRivetTopOffsetStdDev measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);

typedef struct GoSurfaceRivetRadialHeightMinClass
{
    GoMeasurementClass base;
    k64f radius;
} GoSurfaceRivetRadialHeightMinClass;

kDeclareClassEx(Go, GoSurfaceRivetRadialHeightMin, GoMeasurement)

GoFx(kStatus) GoSurfaceRivetRadialHeightMin_VInit(GoSurfaceRivetRadialHeightMin measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);
GoFx(kStatus) GoSurfaceRivetRadialHeightMin_VRead(GoSurfaceRivetRadialHeightMin measurement, kXml xml, kXmlItem item);
GoFx(kStatus) GoSurfaceRivetRadialHeightMin_VWrite(GoSurfaceRivetRadialHeightMin measurement, kXml xml, kXmlItem item);

typedef struct GoSurfaceRivetRadialHeightMaxClass
{
    GoMeasurementClass base;
    k64f radius;
} GoSurfaceRivetRadialHeightMaxClass;

kDeclareClassEx(Go, GoSurfaceRivetRadialHeightMax, GoMeasurement)

GoFx(kStatus) GoSurfaceRivetRadialHeightMax_VInit(GoSurfaceRivetRadialHeightMax measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);
GoFx(kStatus) GoSurfaceRivetRadialHeightMax_VRead(GoSurfaceRivetRadialHeightMax measurement, kXml xml, kXmlItem item);
GoFx(kStatus) GoSurfaceRivetRadialHeightMax_VWrite(GoSurfaceRivetRadialHeightMax measurement, kXml xml, kXmlItem item);

typedef struct GoSurfaceRivetRadialHeightMeanClass
{
    GoMeasurementClass base;
    k64f radius;
} GoSurfaceRivetRadialHeightMeanClass;

kDeclareClassEx(Go, GoSurfaceRivetRadialHeightMean, GoMeasurement)

GoFx(kStatus) GoSurfaceRivetRadialHeightMean_VInit(GoSurfaceRivetRadialHeightMean measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);
GoFx(kStatus) GoSurfaceRivetRadialHeightMean_VRead(GoSurfaceRivetRadialHeightMean measurement, kXml xml, kXmlItem item);
GoFx(kStatus) GoSurfaceRivetRadialHeightMean_VWrite(GoSurfaceRivetRadialHeightMean measurement, kXml xml, kXmlItem item);

typedef struct GoSurfaceRivetRadialHeightStdDevClass
{
    GoMeasurementClass base;
    k64f radius;
} GoSurfaceRivetRadialHeightStdDevClass;

kDeclareClassEx(Go, GoSurfaceRivetRadialHeightStdDev, GoMeasurement)

GoFx(kStatus) GoSurfaceRivetRadialHeightStdDev_VInit(GoSurfaceRivetRadialHeightStdDev measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);
GoFx(kStatus) GoSurfaceRivetRadialHeightStdDev_VRead(GoSurfaceRivetRadialHeightStdDev measurement, kXml xml, kXmlItem item);
GoFx(kStatus) GoSurfaceRivetRadialHeightStdDev_VWrite(GoSurfaceRivetRadialHeightStdDev measurement, kXml xml, kXmlItem item);

typedef struct GoSurfaceRivetRadialSlopeMinClass
{
    GoMeasurementClass base;
    k64f radius;
} GoSurfaceRivetRadialSlopeMinClass;

kDeclareClassEx(Go, GoSurfaceRivetRadialSlopeMin, GoMeasurement)

GoFx(kStatus) GoSurfaceRivetRadialSlopeMin_VInit(GoSurfaceRivetRadialSlopeMin measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);
GoFx(kStatus) GoSurfaceRivetRadialSlopeMin_VRead(GoSurfaceRivetRadialSlopeMin measurement, kXml xml, kXmlItem item);
GoFx(kStatus) GoSurfaceRivetRadialSlopeMin_VWrite(GoSurfaceRivetRadialSlopeMin measurement, kXml xml, kXmlItem item);


typedef struct GoSurfaceRivetRadialSlopeMaxClass
{
    GoMeasurementClass base;
    k64f radius;
} GoSurfaceRivetRadialSlopeMaxClass;

kDeclareClassEx(Go, GoSurfaceRivetRadialSlopeMax, GoMeasurement)

GoFx(kStatus) GoSurfaceRivetRadialSlopeMax_VInit(GoSurfaceRivetRadialSlopeMax measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);
GoFx(kStatus) GoSurfaceRivetRadialSlopeMax_VRead(GoSurfaceRivetRadialSlopeMax measurement, kXml xml, kXmlItem item);
GoFx(kStatus) GoSurfaceRivetRadialSlopeMax_VWrite(GoSurfaceRivetRadialSlopeMax measurement, kXml xml, kXmlItem item);

typedef struct GoSurfaceRivetRadialSlopeMeanClass
{
    GoMeasurementClass base;
    k64f radius;
} GoSurfaceRivetRadialSlopeMeanClass;

kDeclareClassEx(Go, GoSurfaceRivetRadialSlopeMean, GoMeasurement)

GoFx(kStatus) GoSurfaceRivetRadialSlopeMean_VInit(GoSurfaceRivetRadialSlopeMean measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);
GoFx(kStatus) GoSurfaceRivetRadialSlopeMean_VRead(GoSurfaceRivetRadialSlopeMean measurement, kXml xml, kXmlItem item);
GoFx(kStatus) GoSurfaceRivetRadialSlopeMean_VWrite(GoSurfaceRivetRadialSlopeMean measurement, kXml xml, kXmlItem item);

typedef struct GoSurfaceRivetRadialSlopeStdDevClass
{
    GoMeasurementClass base;
    k64f radius;
} GoSurfaceRivetRadialSlopeStdDevClass;

kDeclareClassEx(Go, GoSurfaceRivetRadialSlopeStdDev, GoMeasurement)

GoFx(kStatus) GoSurfaceRivetRadialSlopeStdDev_VInit(GoSurfaceRivetRadialSlopeStdDev measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);
GoFx(kStatus) GoSurfaceRivetRadialSlopeStdDev_VRead(GoSurfaceRivetRadialSlopeStdDev measurement, kXml xml, kXmlItem item);
GoFx(kStatus) GoSurfaceRivetRadialSlopeStdDev_VWrite(GoSurfaceRivetRadialSlopeStdDev measurement, kXml xml, kXmlItem item);


typedef struct GoSurfaceStudBaseXClass
{
    GoMeasurementClass base;
} GoSurfaceStudBaseXClass;

kDeclareClassEx(Go, GoSurfaceStudBaseX, GoMeasurement)

GoFx(kStatus) GoSurfaceStudBaseX_VInit(GoSurfaceStudBaseX measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);


typedef struct GoSurfaceStudBaseYClass
{
    GoMeasurementClass base;
} GoSurfaceStudBaseYClass;

kDeclareClassEx(Go, GoSurfaceStudBaseY, GoMeasurement)

GoFx(kStatus) GoSurfaceStudBaseY_VInit(GoSurfaceStudBaseY measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);


typedef struct GoSurfaceStudBaseZClass
{
    GoMeasurementClass base;
} GoSurfaceStudBaseZClass;

kDeclareClassEx(Go, GoSurfaceStudBaseZ, GoMeasurement)

GoFx(kStatus) GoSurfaceStudBaseZ_VInit(GoSurfaceStudBaseZ measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);


typedef struct GoSurfaceStudTipXClass
{
    GoMeasurementClass base;
} GoSurfaceStudTipXClass;

kDeclareClassEx(Go, GoSurfaceStudTipX, GoMeasurement)

GoFx(kStatus) GoSurfaceStudTipX_VInit(GoSurfaceStudTipX measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);


typedef struct GoSurfaceStudTipYClass
{
    GoMeasurementClass base;
} GoSurfaceStudTipYClass;

kDeclareClassEx(Go, GoSurfaceStudTipY, GoMeasurement)

GoFx(kStatus) GoSurfaceStudTipY_VInit(GoSurfaceStudTipY measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);


typedef struct GoSurfaceStudTipZClass
{
    GoMeasurementClass base;
} GoSurfaceStudTipZClass;

kDeclareClassEx(Go, GoSurfaceStudTipZ, GoMeasurement)

GoFx(kStatus) GoSurfaceStudTipZ_VInit(GoSurfaceStudTipZ measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);


typedef struct GoSurfaceStudRadiusClass
{
    GoMeasurementClass base;
    k64f radiusOffset;
} GoSurfaceStudRadiusClass;

kDeclareClassEx(Go, GoSurfaceStudRadius, GoMeasurement)

GoFx(kStatus) GoSurfaceStudRadius_VInit(GoSurfaceStudRadius measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);
GoFx(kStatus) GoSurfaceStudRadius_VRead(GoSurfaceStudRadius measurement, kXml xml, kXmlItem item);
GoFx(kStatus) GoSurfaceStudRadius_VWrite(GoSurfaceStudRadius measurement, kXml xml, kXmlItem item);

typedef struct GoSurfaceVolumeVolumeClass
{
    GoMeasurementClass base;
} GoSurfaceVolumeVolumeClass;

kDeclareClassEx(Go, GoSurfaceVolumeVolume, GoMeasurement)

GoFx(kStatus) GoSurfaceVolumeVolume_VInit(GoSurfaceVolumeVolume measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);


typedef struct GoSurfaceVolumeAreaClass
{
    GoMeasurementClass base;
} GoSurfaceVolumeAreaClass;

kDeclareClassEx(Go, GoSurfaceVolumeArea, GoMeasurement)

GoFx(kStatus) GoSurfaceVolumeArea_VInit(GoSurfaceVolumeArea measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);


typedef struct GoSurfaceVolumeThicknessClass
{
    GoMeasurementClass base;
    GoSurfaceLocation location;
} GoSurfaceVolumeThicknessClass;

kDeclareClassEx(Go, GoSurfaceVolumeThickness, GoMeasurement)

GoFx(kStatus) GoSurfaceVolumeThickness_VInit(GoSurfaceVolumeThickness measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);
GoFx(kStatus) GoSurfaceVolumeThickness_VRead(GoSurfaceVolumeThickness measurement, kXml xml, kXmlItem item);
GoFx(kStatus) GoSurfaceVolumeThickness_VWrite(GoSurfaceVolumeThickness measurement, kXml xml, kXmlItem item);


typedef struct GoScriptOutputClass
{
    GoMeasurementClass base;
} GoScriptOutputClass;

kDeclareClassEx(Go, GoScriptOutput, GoMeasurement)

GoFx(kStatus) GoScriptOutput_VInit(GoScriptOutput measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);
GoFx(kStatus) GoScriptOutput_VRead(GoScriptOutput measurement, kXml xml, kXmlItem item);
GoFx(kStatus) GoScriptOutput_VWrite(GoScriptOutput measurement, kXml xml, kXmlItem item);

#endif

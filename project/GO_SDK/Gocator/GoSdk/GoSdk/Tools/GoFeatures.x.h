/**
 * @file    GoFeatures.x.h
 *
 * @internal
 * Copyright (C) 2017-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef GO_SDK_FEATURES_X_H
#define GO_SDK_FEATURES_X_H

#include <kApi/Data/kXml.h>
#include <GoSdk/GoSdkDef.h>

// Define the feature names, ordered alphabetically.
#define GO_FEATURES_NAME_AXIS_LINE          "AxisLine"
#define GO_FEATURES_NAME_BASE_POINT         "BasePoint"
#define GO_FEATURES_NAME_BASELINE           "BaseLine"
#define GO_FEATURES_NAME_CENTER_POINT       "CenterPoint"
#define GO_FEATURES_NAME_CORNER_POINT       "CornerPoint"
#define GO_FEATURES_NAME_CUSTOM             "Custom"
#define GO_FEATURES_NAME_EDGE_LINE          "EdgeLine"
#define GO_FEATURES_NAME_EDGE_POINT         "EdgePoint"
#define GO_FEATURES_NAME_ERROR_MAX_POINT    "ErrorMaxPoint"
#define GO_FEATURES_NAME_ERROR_MIN_POINT    "ErrorMinPoint"
#define GO_FEATURES_NAME_FEATURE            "Feature"
#define GO_FEATURES_NAME_INTERSECT_POINT    "IntersectPoint"
#define GO_FEATURES_NAME_LEFT_FLUSH_POINT   "LeftFlushPoint"
#define GO_FEATURES_NAME_LEFT_GAP_POINT     "LeftGapPoint"
#define GO_FEATURES_NAME_LINE               "Line"
#define GO_FEATURES_NAME_MAJOR_AXIS_LINE    "MajorAxisLine"
#define GO_FEATURES_NAME_MINOR_AXIS_LINE    "MinorAxisLine"
#define GO_FEATURES_NAME_PLANE              "Plane"
#define GO_FEATURES_NAME_POINT              "Point"
#define GO_FEATURES_NAME_RIGHT_GAP_POINT    "RightGapPoint"
#define GO_FEATURES_NAME_RIGHT_FLUSH_POINT  "RightFlushPoint"
#define GO_FEATURES_NAME_TIP_POINT          "TipPoint"

typedef kObject GoFeatures;

#define GOFEATURES_NAME_TYPE kText256

typedef struct GoFeaturesStatic
{
    kArrayList nameTypeMapList;
} GoFeaturesStatic;

kDeclareStaticClassEx(Go, GoFeatures)

GoFx(kStatus) xGoFeatures_InitStatic();
GoFx(kStatus) xGoFeatures_ReleaseStatic();

GoFx(kStatus) GoFeatures_NameTypeMapAdd(const char* name, const char *featureName, kType featureType);
GoFx(kStatus) GoFeatures_NameTypeMapInit(void);
GoFx(kSize) GoFeatures_NameTypeMapEntryCount(void);
GoFx(kStatus) GoFeatures_CreateNameTypeMap(void);
GoFx(kStatus) GoFeatures_ParseType(const kChar* toolName, const kChar* featureName, kType* type);
GoFx(kStatus) GoFeatures_ParseTypeHelper(const kChar* name, kType* type);
GoFx(kStatus) GoFeatures_FormatType(kType feature, kChar* featureName, kSize capacity);
GoFx(kStatus) GoFeatures_FormatTypeExtToolFormat(kType feature, kBool internalTool, kChar* featureTypeName, kSize capacity);
GoFx(kStatus) GoFeatures_FormatTypeHelper(kType feature, const kChar** outputPtr);

GoFx(kStatus) GoFeatures_FormatDataType(GoFeatureDataType type, kChar* featureDataType, kSize capacity);
GoFx(kStatus) GoFeatures_ParseDataType(const kChar* featureDataType, GoFeatureType* type);


typedef struct GoExtFeatureClass
{
    GoFeatureClass base;
    GoExtParams params;
} GoExtFeatureClass;

kDeclareClassEx(Go, GoExtFeature, GoFeature)

GoFx(kStatus) GoExtFeature_VInit(GoExtFeature feature, kType type, kObject sensor, kObject srcTool, kAlloc alloc);
GoFx(kStatus) GoExtFeature_VRead(GoExtFeature feature, kXml xml, kXmlItem item);
GoFx(kStatus) GoExtFeature_VWrite(GoExtFeature feature, kXml xml, kXmlItem item);
GoFx(kStatus) GoExtFeature_VRelease(GoExtFeature feature);


typedef struct GoSurfaceEdgeEdgeLineClass
{
    GoFeatureClass base;
}GoSurfaceEdgeEdgeLineClass;

kDeclareClassEx(Go, GoSurfaceEdgeEdgeLine, GoFeature)

GoFx(kStatus) GoSurfaceEdgeEdgeLine_VInit(GoSurfaceEdgeEdgeLine feature, kType type, kObject sensor, kObject srcTool, kAlloc alloc);
GoFx(kStatus) GoSurfaceEdgeEdgeLine_VRead(GoFeature feature, kXml xml, kXmlItem item);
GoFx(kStatus) GoSurfaceEdgeEdgeLine_VWrite(GoFeature feature, kXml xml, kXmlItem item);

typedef struct GoSurfaceEdgeCenterPointClass
{
    GoFeatureClass base;
}GoSurfaceEdgeCenterPointClass;

kDeclareClassEx(Go, GoSurfaceEdgeCenterPoint, GoFeature)

GoFx(kStatus) GoSurfaceEdgeCenterPoint_VInit(GoSurfaceEdgeCenterPoint feature, kType type, kObject sensor, kObject srcTool, kAlloc alloc);
GoFx(kStatus) GoSurfaceEdgeCenterPoint_VRead(GoFeature feature, kXml xml, kXmlItem item);
GoFx(kStatus) GoSurfaceEdgeCenterPoint_VWrite(GoFeature feature, kXml xml, kXmlItem item);

typedef struct GoSurfaceBoundingBoxCenterPointClass
{
    GoFeatureClass base;
}GoSurfaceBoundingBoxCenterPointClass;

kDeclareClassEx(Go, GoSurfaceBoundingBoxCenterPoint, GoFeature)

GoFx(kStatus) GoSurfaceBoundingBoxCenterPoint_VInit(GoSurfaceBoundingBoxCenterPoint feature, kType type, kObject sensor, kObject srcTool, kAlloc alloc);
GoFx(kStatus) GoSurfaceBoundingBoxCenterPoint_VRead(GoFeature feature, kXml xml, kXmlItem item);
GoFx(kStatus) GoSurfaceBoundingBoxCenterPoint_VWrite(GoFeature feature, kXml xml, kXmlItem item);

typedef struct GoSurfaceBoundingBoxAxisLineClass
{
    GoFeatureClass base;
}GoSurfaceBoundingBoxAxisLineClass;

kDeclareClassEx(Go, GoSurfaceBoundingBoxAxisLine, GoFeature)

GoFx(kStatus) GoSurfaceBoundingBoxAxisLine_VInit(GoSurfaceBoundingBoxAxisLine feature, kType type, kObject sensor, kObject srcTool, kAlloc alloc);
GoFx(kStatus) GoSurfaceBoundingBoxAxisLine_VRead(GoFeature feature, kXml xml, kXmlItem item);
GoFx(kStatus) GoSurfaceBoundingBoxAxisLine_VWrite(GoFeature feature, kXml xml, kXmlItem item);

typedef struct GoSurfaceCountersunkHoleCenterPointClass
{
    GoFeatureClass base;
}GoSurfaceCountersunkHoleCenterPointClass;

kDeclareClassEx(Go, GoSurfaceCountersunkHoleCenterPoint, GoFeature)

GoFx(kStatus) GoSurfaceCountersunkHoleCenterPoint_VInit(GoSurfaceCountersunkHoleCenterPoint feature, kType type, kObject sensor, kObject srcTool, kAlloc alloc);
GoFx(kStatus) GoSurfaceCountersunkHoleCenterPoint_VRead(GoFeature feature, kXml xml, kXmlItem item);
GoFx(kStatus) GoSurfaceCountersunkHoleCenterPoint_VWrite(GoFeature feature, kXml xml, kXmlItem item);


typedef struct     GoSurfaceDimensionCenterPointClass
{
    GoFeatureClass base;
}GoSurfaceDimensionCenterPointClass;

kDeclareClassEx(Go, GoSurfaceDimensionCenterPoint, GoFeature)

GoFx(kStatus) GoSurfaceDimensionCenterPoint_VInit(GoSurfaceDimensionCenterPoint feature, kType type, kObject sensor, kObject srcTool, kAlloc alloc);
GoFx(kStatus) GoSurfaceDimensionCenterPoint_VRead(GoFeature feature, kXml xml, kXmlItem item);
GoFx(kStatus) GoSurfaceDimensionCenterPoint_VWrite(GoFeature feature, kXml xml, kXmlItem item);

typedef struct     GoSurfaceEllipseCenterPointClass
{
    GoFeatureClass base;
}GoSurfaceEllipseCenterPointClass;

kDeclareClassEx(Go, GoSurfaceEllipseCenterPoint, GoFeature)

GoFx(kStatus) GoSurfaceEllipseCenterPoint_VInit(GoSurfaceEllipseCenterPoint feature, kType type, kObject sensor, kObject srcTool, kAlloc alloc);
GoFx(kStatus) GoSurfaceEllipseCenterPoint_VRead(GoFeature feature, kXml xml, kXmlItem item);
GoFx(kStatus) GoSurfaceEllipseCenterPoint_VWrite(GoFeature feature, kXml xml, kXmlItem item);

typedef struct     GoSurfaceEllipseMajorAxisLineClass
{
    GoFeatureClass base;
}GoSurfaceEllipseMajorAxisLineClass;

kDeclareClassEx(Go, GoSurfaceEllipseMajorAxisLine, GoFeature)

GoFx(kStatus) GoSurfaceEllipseMajorAxisLine_VInit(GoSurfaceEllipseMajorAxisLine feature, kType type, kObject sensor, kObject srcTool, kAlloc alloc);
GoFx(kStatus) GoSurfaceEllipseMajorAxisLine_VRead(GoFeature feature, kXml xml, kXmlItem item);
GoFx(kStatus) GoSurfaceEllipseMajorAxisLine_VWrite(GoFeature feature, kXml xml, kXmlItem item);

typedef struct     GoSurfaceEllipseMinorAxisLineClass
{
    GoFeatureClass base;
}GoSurfaceEllipseMinorAxisLineClass;

kDeclareClassEx(Go, GoSurfaceEllipseMinorAxisLine, GoFeature)

GoFx(kStatus) GoSurfaceEllipseMinorAxisLine_VInit(GoSurfaceEllipseMinorAxisLine feature, kType type, kObject sensor, kObject srcTool, kAlloc alloc);
GoFx(kStatus) GoSurfaceEllipseMinorAxisLine_VRead(GoFeature feature, kXml xml, kXmlItem item);
GoFx(kStatus) GoSurfaceEllipseMinorAxisLine_VWrite(GoFeature feature, kXml xml, kXmlItem item);


typedef struct     GoSurfaceHoleCenterPointClass
{
    GoFeatureClass base;
}GoSurfaceHoleCenterPointClass;

kDeclareClassEx(Go, GoSurfaceHoleCenterPoint, GoFeature)

GoFx(kStatus) GoSurfaceHoleCenterPoint_VInit(GoSurfaceHoleCenterPoint feature, kType type, kObject sensor, kObject srcTool, kAlloc alloc);
GoFx(kStatus) GoSurfaceHoleCenterPoint_VRead(GoFeature feature, kXml xml, kXmlItem item);
GoFx(kStatus) GoSurfaceHoleCenterPoint_VWrite(GoFeature feature, kXml xml, kXmlItem item);

typedef struct     GoSurfaceOpeningCenterPointClass
{
    GoFeatureClass base;
}GoSurfaceOpeningCenterPointClass;

kDeclareClassEx(Go, GoSurfaceOpeningCenterPoint, GoFeature)

GoFx(kStatus) GoSurfaceOpeningCenterPoint_VInit(GoSurfaceOpeningCenterPoint feature, kType type, kObject sensor, kObject srcTool, kAlloc alloc);
GoFx(kStatus) GoSurfaceOpeningCenterPoint_VRead(GoFeature feature, kXml xml, kXmlItem item);
GoFx(kStatus) GoSurfaceOpeningCenterPoint_VWrite(GoFeature feature, kXml xml, kXmlItem item);

typedef struct     GoSurfacePlanePlaneClass
{
    GoFeatureClass base;
}GoSurfacePlanePlaneClass;

kDeclareClassEx(Go, GoSurfacePlanePlane, GoFeature)

GoFx(kStatus) GoSurfacePlanePlane_VInit(GoSurfacePlanePlane feature, kType type, kObject sensor, kObject srcTool, kAlloc alloc);
GoFx(kStatus) GoSurfacePlanePlane_VRead(GoFeature feature, kXml xml, kXmlItem item);
GoFx(kStatus) GoSurfacePlanePlane_VWrite(GoFeature feature, kXml xml, kXmlItem item);

typedef struct     GoSurfacePositionPointClass
{
    GoFeatureClass base;
}GoSurfacePositionPointClass;

kDeclareClassEx(Go, GoSurfacePositionPoint, GoFeature)

GoFx(kStatus) GoSurfacePositionPoint_VInit(GoSurfacePositionPoint feature, kType type, kObject sensor, kObject srcTool, kAlloc alloc);
GoFx(kStatus) GoSurfacePositionPoint_VRead(GoFeature feature, kXml xml, kXmlItem item);
GoFx(kStatus) GoSurfacePositionPoint_VWrite(GoFeature feature, kXml xml, kXmlItem item);

typedef struct     GoSurfaceStudTipPointClass
{
    GoFeatureClass base;
}GoSurfaceStudTipPointClass;

kDeclareClassEx(Go, GoSurfaceStudTipPoint, GoFeature)

GoFx(kStatus) GoSurfaceStudTipPoint_VInit(GoSurfaceStudTipPoint feature, kType type, kObject sensor, kObject srcTool, kAlloc alloc);
GoFx(kStatus) GoSurfaceStudTipPoint_VRead(GoFeature feature, kXml xml, kXmlItem item);
GoFx(kStatus) GoSurfaceStudTipPoint_VWrite(GoFeature feature, kXml xml, kXmlItem item);

typedef struct     GoSurfaceStudBasePointClass
{
    GoFeatureClass base;
}GoSurfaceStudBasePointClass;

kDeclareClassEx(Go, GoSurfaceStudBasePoint, GoFeature)

GoFx(kStatus) GoSurfaceStudBasePoint_VInit(GoSurfaceStudBasePoint feature, kType type, kObject sensor, kObject srcTool, kAlloc alloc);
GoFx(kStatus) GoSurfaceStudBasePoint_VRead(GoFeature feature, kXml xml, kXmlItem item);
GoFx(kStatus) GoSurfaceStudBasePoint_VWrite(GoFeature feature, kXml xml, kXmlItem item);


typedef struct     GoProfilePositionPointClass
{
    GoFeatureClass base;
}GoProfilePositionPointClass;

kDeclareClassEx(Go, GoProfilePositionPoint, GoFeature)

GoFx(kStatus) GoProfilePositionPoint_VInit(GoProfilePositionPoint feature, kType type, kObject sensor, kObject srcTool, kAlloc alloc);
GoFx(kStatus) GoProfilePositionPoint_VRead(GoFeature feature, kXml xml, kXmlItem item);
GoFx(kStatus) GoProfilePositionPoint_VWrite(GoFeature feature, kXml xml, kXmlItem item);

typedef struct     GoProfileLineLineClass
{
    GoFeatureClass base;
}GoProfileLineLineClass;

kDeclareClassEx(Go, GoProfileLineLine, GoFeature)

GoFx(kStatus) GoProfileLineLine_VInit(GoProfileLineLine feature, kType type, kObject sensor, kObject srcTool, kAlloc alloc);
GoFx(kStatus) GoProfileLineLine_VRead(GoFeature feature, kXml xml, kXmlItem item);
GoFx(kStatus) GoProfileLineLine_VWrite(GoFeature feature, kXml xml, kXmlItem item);

typedef struct     GoProfileLineMinErrorPointClass
{
    GoFeatureClass base;
}GoProfileLineMinErrorPointClass;

kDeclareClassEx(Go, GoProfileLineMinErrorPoint, GoFeature)

GoFx(kStatus) GoProfileLineMinErrorPoint_VInit(GoProfileLineMinErrorPoint feature, kType type, kObject sensor, kObject srcTool, kAlloc alloc);
GoFx(kStatus) GoProfileLineMinErrorPoint_VRead(GoFeature feature, kXml xml, kXmlItem item);
GoFx(kStatus) GoProfileLineMinErrorPoint_VWrite(GoFeature feature, kXml xml, kXmlItem item);

typedef struct    GoProfileLineMaxErrorPointClass
{
    GoFeatureClass base;
}GoProfileLineMaxErrorPointClass;

kDeclareClassEx(Go, GoProfileLineMaxErrorPoint, GoFeature)

GoFx(kStatus) GoProfileLineMaxErrorPoint_VInit(GoProfileLineMaxErrorPoint feature, kType type, kObject sensor, kObject srcTool, kAlloc alloc);
GoFx(kStatus) GoProfileLineMaxErrorPoint_VRead(GoFeature feature, kXml xml, kXmlItem item);
GoFx(kStatus) GoProfileLineMaxErrorPoint_VWrite(GoFeature feature, kXml xml, kXmlItem item);

typedef struct    GoProfileIntersectIntersectPointClass
{
    GoFeatureClass base;
}GoProfileIntersectIntersectPointClass;

kDeclareClassEx(Go, GoProfileIntersectIntersectPoint, GoFeature)

GoFx(kStatus) GoProfileIntersectIntersectPoint_VInit(GoProfileIntersectIntersectPoint feature, kType type, kObject sensor, kObject srcTool, kAlloc alloc);
GoFx(kStatus) GoProfileIntersectIntersectPoint_VRead(GoFeature feature, kXml xml, kXmlItem item);
GoFx(kStatus) GoProfileIntersectIntersectPoint_VWrite(GoFeature feature, kXml xml, kXmlItem item);

typedef struct    GoProfileIntersectLineClass
{
    GoFeatureClass base;
}GoProfileIntersectLineClass;

kDeclareClassEx(Go, GoProfileIntersectLine, GoFeature)

GoFx(kStatus) GoProfileIntersectLine_VInit(GoProfileIntersectLine feature, kType type, kObject sensor, kObject srcTool, kAlloc alloc);
GoFx(kStatus) GoProfileIntersectLine_VRead(GoFeature feature, kXml xml, kXmlItem item);
GoFx(kStatus) GoProfileIntersectLine_VWrite(GoFeature feature, kXml xml, kXmlItem item);

typedef struct    GoProfileIntersectBaseLineClass
{
    GoFeatureClass base;
}GoProfileIntersectBaseLineClass;

kDeclareClassEx(Go, GoProfileIntersectBaseLine, GoFeature)

GoFx(kStatus) GoProfileIntersectBaseLine_VInit(GoProfileIntersectBaseLine feature, kType type, kObject sensor, kObject srcTool, kAlloc alloc);
GoFx(kStatus) GoProfileIntersectBaseLine_VRead(GoFeature feature, kXml xml, kXmlItem item);
GoFx(kStatus) GoProfileIntersectBaseLine_VWrite(GoFeature feature, kXml xml, kXmlItem item);

typedef struct    GoProfileBoundingBoxCornerPointClass
{
    GoFeatureClass base;
}GoProfileBoundingBoxCornerPointClass;

kDeclareClassEx(Go, GoProfileBoundingBoxCornerPoint, GoFeature)

GoFx(kStatus) GoProfileBoundingBoxCornerPoint_VInit(GoProfileBoundingBoxCornerPoint feature, kType type, kObject sensor, kObject srcTool, kAlloc alloc);
GoFx(kStatus) GoProfileBoundingBoxCornerPoint_VRead(GoFeature feature, kXml xml, kXmlItem item);
GoFx(kStatus) GoProfileBoundingBoxCornerPoint_VWrite(GoFeature feature, kXml xml, kXmlItem item);

typedef struct    GoProfileBoundingBoxCenterPointClass
{
    GoFeatureClass base;
}GoProfileBoundingBoxCenterPointClass;

kDeclareClassEx(Go, GoProfileBoundingBoxCenterPoint, GoFeature)

GoFx(kStatus) GoProfileBoundingBoxCenterPoint_VInit(GoProfileBoundingBoxCenterPoint feature, kType type, kObject sensor, kObject srcTool, kAlloc alloc);
GoFx(kStatus) GoProfileBoundingBoxCenterPoint_VRead(GoFeature feature, kXml xml, kXmlItem item);
GoFx(kStatus) GoProfileBoundingBoxCenterPoint_VWrite(GoFeature feature, kXml xml, kXmlItem item);

typedef struct    GoProfileAreaCenterPointClass
{
    GoFeatureClass base;
}GoProfileAreaCenterPointClass;

kDeclareClassEx(Go, GoProfileAreaCenterPoint, GoFeature)

#define  GoProfileAreaCenterPointCast_(CONTEXT)    kCastClass_(GoProfileAreaCenterPoint, CONTEXT)

GoFx(kStatus) GoProfileAreaCenterPoint_VInit(GoProfileAreaCenterPoint feature, kType type, kObject sensor, kObject srcTool, kAlloc alloc);
GoFx(kStatus) GoProfileAreaCenterPoint_VRead(GoFeature feature, kXml xml, kXmlItem item);
GoFx(kStatus) GoProfileAreaCenterPoint_VWrite(GoFeature feature, kXml xml, kXmlItem item);

typedef struct    GoProfileCircleCenterPointClass
{
    GoFeatureClass base;
}GoProfileCircleCenterPointClass;

kDeclareClassEx(Go, GoProfileCircleCenterPoint, GoFeature)

GoFx(kStatus) GoProfileCircleCenterPoint_VInit(GoProfileCircleCenterPoint feature, kType type, kObject sensor, kObject srcTool, kAlloc alloc);
GoFx(kStatus) GoProfileCircleCenterPoint_VRead(GoFeature feature, kXml xml, kXmlItem item);
GoFx(kStatus) GoProfileCircleCenterPoint_VWrite(GoFeature feature, kXml xml, kXmlItem item);

typedef struct    GoProfileDimensionCenterPointClass
{
    GoFeatureClass base;
}GoProfileDimensionCenterPointClass;

kDeclareClassEx(Go, GoProfileDimensionCenterPoint, GoFeature)

GoFx(kStatus) GoProfileDimensionCenterPoint_VInit(GoProfileDimensionCenterPoint feature, kType type, kObject sensor, kObject srcTool, kAlloc alloc);
GoFx(kStatus) GoProfileDimensionCenterPoint_VRead(GoFeature feature, kXml xml, kXmlItem item);
GoFx(kStatus) GoProfileDimensionCenterPoint_VWrite(GoFeature feature, kXml xml, kXmlItem item);

typedef struct    GoProfilePanelLeftGapPointClass
{
    GoFeatureClass base;
}GoProfilePanelLeftGapPointClass;

kDeclareClassEx(Go, GoProfilePanelLeftGapPoint, GoFeature)

GoFx(kStatus) GoProfilePanelLeftGapPoint_VInit(GoProfilePanelLeftGapPoint feature, kType type, kObject sensor, kObject srcTool, kAlloc alloc);
GoFx(kStatus) GoProfilePanelLeftGapPoint_VRead(GoFeature feature, kXml xml, kXmlItem item);
GoFx(kStatus) GoProfilePanelLeftGapPoint_VWrite(GoFeature feature, kXml xml, kXmlItem item);

typedef struct    GoProfilePanelLeftFlushPointClass
{
    GoFeatureClass base;
}GoProfilePanelLeftFlushPointClass;

kDeclareClassEx(Go, GoProfilePanelLeftFlushPoint, GoFeature)

GoFx(kStatus) GoProfilePanelLeftFlushPoint_VInit(GoProfilePanelLeftFlushPoint feature, kType type, kObject sensor, kObject srcTool, kAlloc alloc);
GoFx(kStatus) GoProfilePanelLeftFlushPoint_VRead(GoFeature feature, kXml xml, kXmlItem item);
GoFx(kStatus) GoProfilePanelLeftFlushPoint_VWrite(GoFeature feature, kXml xml, kXmlItem item);

typedef struct    GoProfilePanelRightGapPointClass
{
    GoFeatureClass base;
}GoProfilePanelRightGapPointClass;

kDeclareClassEx(Go, GoProfilePanelRightGapPoint, GoFeature)

GoFx(kStatus) GoProfilePanelRightGapPoint_VInit(GoProfilePanelRightGapPoint feature, kType type, kObject sensor, kObject srcTool, kAlloc alloc);
GoFx(kStatus) GoProfilePanelRightGapPoint_VRead(GoFeature feature, kXml xml, kXmlItem item);
GoFx(kStatus) GoProfilePanelRightGapPoint_VWrite(GoFeature feature, kXml xml, kXmlItem item);

typedef struct    GoProfilePanelRightFlushPointClass
{
    GoFeatureClass base;
}GoProfilePanelRightFlushPointClass;

kDeclareClassEx(Go, GoProfilePanelRightFlushPoint, GoFeature)

GoFx(kStatus) GoProfilePanelRightFlushPoint_VInit(GoProfilePanelRightFlushPoint feature, kType type, kObject sensor, kObject srcTool, kAlloc alloc);
GoFx(kStatus) GoProfilePanelRightFlushPoint_VRead(GoFeature feature, kXml xml, kXmlItem item);
GoFx(kStatus) GoProfilePanelRightFlushPoint_VWrite(GoFeature feature, kXml xml, kXmlItem item);

typedef struct    GoProfileRoundCornerPointClass
{
    GoFeatureClass base;
}GoProfileRoundCornerPointClass;

kDeclareClassEx(Go, GoProfileRoundCornerPoint, GoFeature)

GoFx(kStatus) GoProfileRoundCornerPoint_VInit(GoProfilePanelRightGapPoint feature, kType type, kObject sensor, kObject srcTool, kAlloc alloc);
GoFx(kStatus) GoProfileRoundCornerPoint_VRead(GoFeature feature, kXml xml, kXmlItem item);
GoFx(kStatus) GoProfileRoundCornerPoint_VWrite(GoFeature feature, kXml xml, kXmlItem item);

typedef struct    GoProfileRoundCornerEdgePointClass
{
    GoFeatureClass base;
}GoProfileRoundCornerEdgePointClass;

kDeclareClassEx(Go, GoProfileRoundCornerEdgePoint, GoFeature)

GoFx(kStatus) GoProfileRoundCornerEdgePoint_VInit(GoProfileRoundCornerEdgePoint feature, kType type, kObject sensor, kObject srcTool, kAlloc alloc);
GoFx(kStatus) GoProfileRoundCornerEdgePoint_VRead(GoFeature feature, kXml xml, kXmlItem item);
GoFx(kStatus) GoProfileRoundCornerEdgePoint_VWrite(GoFeature feature, kXml xml, kXmlItem item);

typedef struct    GoProfileRoundCornerCenterPointClass
{
    GoFeatureClass base;
}GoProfileRoundCornerCenterPointClass;

kDeclareClassEx(Go, GoProfileRoundCornerCenterPoint, GoFeature)

GoFx(kStatus) GoProfileRoundCornerCenterPoint_VInit(GoProfileRoundCornerCenterPoint feature, kType type, kObject sensor, kObject srcTool, kAlloc alloc);
GoFx(kStatus) GoProfileRoundCornerCenterPoint_VRead(GoFeature feature, kXml xml, kXmlItem item);
GoFx(kStatus) GoProfileRoundCornerCenterPoint_VWrite(GoFeature feature, kXml xml, kXmlItem item);

#endif

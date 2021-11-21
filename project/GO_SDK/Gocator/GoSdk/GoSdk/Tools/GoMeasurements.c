/**
 * @file    GoMeasurements.c
 *
 * @internal
 * Copyright (C) 2016-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <GoSdk/Tools/GoMeasurements.h>
#include <GoSdk/Tools/GoMeasurements.x.h>
#include <GoSdk/Tools/GoExtParam.x.h>
#include <GoSdk/GoSensor.h>
#include <GoSdk/GoUtils.h>

kBeginStaticClassEx(Go, GoMeasurements)
kEndStaticClassEx()

GoFx(kStatus) xGoMeasurements_InitStatic()
{
    kStaticObj(GoMeasurements);

    kCheck(kArrayList_Construct(&sobj->nameTypeMapList, kTypeOf(GoMeasurementsNameTypeMapping), 0, kNULL));

    kCheck(GoMeasurements_CreateNameTypeMap());

    return kOK;

}
GoFx(kStatus) xGoMeasurements_ReleaseStatic()
{
    kStaticObj(GoMeasurements);
    
    kObject_Destroy(sobj->nameTypeMapList);

    return kOK;
}

GoFx(kSize) GoMeasurements_NameTypeMapEntryCount(void)
{
    kStaticObj(GoMeasurements);

    return kArrayList_Count(sobj->nameTypeMapList);
}

GoFx(kStatus) GoMeasurements_NameTypeMapAdd(const char* name, const char *measurementName, kType measurementType)
{
    kStaticObj(GoMeasurements);

    GoMeasurementsNameTypeMapping map;

    kCheck(kStrCopy(map.lookupName, kCountOf(map.lookupName), name));
    kCheck(kStrCopy(map.measurementName, kCountOf(map.measurementName), measurementName));
    map.measurementType = measurementType;

    kCheck(kArrayList_AddT(sobj->nameTypeMapList, &map));

    return kOK;
}

// This function is the runtime equivalent of defining a static mapping table.
GoFx(kStatus) GoMeasurements_NameTypeMapInit(void)
{
    kStatus exception;

    kTry
    {
        // NOTE: first parameter is a concatenation of tool name and measurement name.
        kTest(GoMeasurements_NameTypeMapAdd(GO_RANGE_TOOL_NAME_POSITION GO_MEASUREMENTS_NAME_Z, GO_MEASUREMENTS_NAME_Z, kTypeOf(GoRangePositionZ)));
        kTest(GoMeasurements_NameTypeMapAdd(GO_RANGE_TOOL_NAME_THICKNESS GO_MEASUREMENTS_NAME_THICKNESS, GO_MEASUREMENTS_NAME_THICKNESS, kTypeOf(GoRangeThicknessThickness)));

        kTest(GoMeasurements_NameTypeMapAdd(GO_PROFILE_TOOL_NAME_AREA GO_MEASUREMENTS_NAME_AREA, GO_MEASUREMENTS_NAME_AREA, kTypeOf(GoProfileAreaArea)));
        kTest(GoMeasurements_NameTypeMapAdd(GO_PROFILE_TOOL_NAME_AREA GO_MEASUREMENTS_NAME_CENTROID_X, GO_MEASUREMENTS_NAME_CENTROID_X, kTypeOf(GoProfileAreaCentroidX)));
        kTest(GoMeasurements_NameTypeMapAdd(GO_PROFILE_TOOL_NAME_AREA GO_MEASUREMENTS_NAME_CENTROID_Z, GO_MEASUREMENTS_NAME_CENTROID_Z, kTypeOf(GoProfileAreaCentroidZ)));

        // TODO: confirm if these are still needed since there is no bridge value tool.
        kTest(GoMeasurements_NameTypeMapAdd(GO_PROFILE_TOOL_NAME_BRIDGE_VALUE GO_MEASUREMENTS_NAME_BRIDGE_VALUE, GO_MEASUREMENTS_NAME_BRIDGE_VALUE, kTypeOf(GoProfileBridgeValueBridgeValue)));
        kTest(GoMeasurements_NameTypeMapAdd(GO_PROFILE_TOOL_NAME_BRIDGE_VALUE GO_MEASUREMENTS_NAME_ANGLE, GO_MEASUREMENTS_NAME_ANGLE, kTypeOf(GoProfileBridgeValueAngle)));
        kTest(GoMeasurements_NameTypeMapAdd(GO_PROFILE_TOOL_NAME_BRIDGE_VALUE GO_MEASUREMENTS_NAME_WINDOW, GO_MEASUREMENTS_NAME_WINDOW, kTypeOf(GoProfileBridgeValueWindow)));
        kTest(GoMeasurements_NameTypeMapAdd(GO_PROFILE_TOOL_NAME_BRIDGE_VALUE GO_MEASUREMENTS_NAME_STD_DEV, GO_MEASUREMENTS_NAME_STD_DEV, kTypeOf(GoProfileBridgeValueStdDev)));

        kTest(GoMeasurements_NameTypeMapAdd(GO_PROFILE_TOOL_NAME_BOUNDING_BOX GO_MEASUREMENTS_NAME_X, GO_MEASUREMENTS_NAME_X, kTypeOf(GoProfileBoxX)));
        kTest(GoMeasurements_NameTypeMapAdd(GO_PROFILE_TOOL_NAME_BOUNDING_BOX GO_MEASUREMENTS_NAME_Z, GO_MEASUREMENTS_NAME_Z, kTypeOf(GoProfileBoxZ)));
        kTest(GoMeasurements_NameTypeMapAdd(GO_PROFILE_TOOL_NAME_BOUNDING_BOX GO_MEASUREMENTS_NAME_WIDTH, GO_MEASUREMENTS_NAME_WIDTH, kTypeOf(GoProfileBoxWidth)));
        kTest(GoMeasurements_NameTypeMapAdd(GO_PROFILE_TOOL_NAME_BOUNDING_BOX GO_MEASUREMENTS_NAME_HEIGHT, GO_MEASUREMENTS_NAME_HEIGHT, kTypeOf(GoProfileBoxHeight)));
        kTest(GoMeasurements_NameTypeMapAdd(GO_PROFILE_TOOL_NAME_BOUNDING_BOX GO_MEASUREMENTS_NAME_GLOBAL_X, GO_MEASUREMENTS_NAME_GLOBAL_X, kTypeOf(GoProfileBoxGlobalX)));
        kTest(GoMeasurements_NameTypeMapAdd(GO_PROFILE_TOOL_NAME_BOUNDING_BOX GO_MEASUREMENTS_NAME_GLOBAL_Y, GO_MEASUREMENTS_NAME_GLOBAL_Y, kTypeOf(GoProfileBoxGlobalY)));
        kTest(GoMeasurements_NameTypeMapAdd(GO_PROFILE_TOOL_NAME_BOUNDING_BOX GO_MEASUREMENTS_NAME_GLOBAL_ANGLE, GO_MEASUREMENTS_NAME_GLOBAL_ANGLE, kTypeOf(GoProfileBoxGlobalAngle)));

        kTest(GoMeasurements_NameTypeMapAdd(GO_PROFILE_TOOL_NAME_CIRCLE GO_MEASUREMENTS_NAME_X, GO_MEASUREMENTS_NAME_X, kTypeOf(GoProfileCircleX)));
        kTest(GoMeasurements_NameTypeMapAdd(GO_PROFILE_TOOL_NAME_CIRCLE GO_MEASUREMENTS_NAME_Z, GO_MEASUREMENTS_NAME_Z, kTypeOf(GoProfileCircleZ)));
        kTest(GoMeasurements_NameTypeMapAdd(GO_PROFILE_TOOL_NAME_CIRCLE GO_MEASUREMENTS_NAME_RADIUS, GO_MEASUREMENTS_NAME_RADIUS, kTypeOf(GoProfileCircleRadius)));
        kTest(GoMeasurements_NameTypeMapAdd(GO_PROFILE_TOOL_NAME_CIRCLE GO_MEASUREMENTS_NAME_STD_DEV, GO_MEASUREMENTS_NAME_STD_DEV, kTypeOf(GoProfileCircleStdDev)));        
        kTest(GoMeasurements_NameTypeMapAdd(GO_PROFILE_TOOL_NAME_CIRCLE GO_MEASUREMENTS_NAME_MIN_ERROR, GO_MEASUREMENTS_NAME_MIN_ERROR, kTypeOf(GoProfileCircleMinError)));
        kTest(GoMeasurements_NameTypeMapAdd(GO_PROFILE_TOOL_NAME_CIRCLE GO_MEASUREMENTS_NAME_MIN_ERROR_X, GO_MEASUREMENTS_NAME_MIN_ERROR_X, kTypeOf(GoProfileCircleMinErrorX)));
        kTest(GoMeasurements_NameTypeMapAdd(GO_PROFILE_TOOL_NAME_CIRCLE GO_MEASUREMENTS_NAME_MIN_ERROR_Z, GO_MEASUREMENTS_NAME_MIN_ERROR_Z, kTypeOf(GoProfileCircleMinErrorZ)));        
        kTest(GoMeasurements_NameTypeMapAdd(GO_PROFILE_TOOL_NAME_CIRCLE GO_MEASUREMENTS_NAME_MAX_ERROR, GO_MEASUREMENTS_NAME_MAX_ERROR, kTypeOf(GoProfileCircleMaxError)));
        kTest(GoMeasurements_NameTypeMapAdd(GO_PROFILE_TOOL_NAME_CIRCLE GO_MEASUREMENTS_NAME_MAX_ERROR_X, GO_MEASUREMENTS_NAME_MAX_ERROR_X, kTypeOf(GoProfileCircleMaxErrorX)));
        kTest(GoMeasurements_NameTypeMapAdd(GO_PROFILE_TOOL_NAME_CIRCLE GO_MEASUREMENTS_NAME_MAX_ERROR_Z, GO_MEASUREMENTS_NAME_MAX_ERROR_Z, kTypeOf(GoProfileCircleMaxErrorZ)));

        kTest(GoMeasurements_NameTypeMapAdd(GO_PROFILE_TOOL_NAME_DIMENSION GO_MEASUREMENTS_NAME_WIDTH, GO_MEASUREMENTS_NAME_WIDTH, kTypeOf(GoProfileDimWidth)));
        kTest(GoMeasurements_NameTypeMapAdd(GO_PROFILE_TOOL_NAME_DIMENSION GO_MEASUREMENTS_NAME_HEIGHT, GO_MEASUREMENTS_NAME_HEIGHT, kTypeOf(GoProfileDimHeight)));
        kTest(GoMeasurements_NameTypeMapAdd(GO_PROFILE_TOOL_NAME_DIMENSION GO_MEASUREMENTS_NAME_DISTANCE, GO_MEASUREMENTS_NAME_DISTANCE, kTypeOf(GoProfileDimDistance)));
        kTest(GoMeasurements_NameTypeMapAdd(GO_PROFILE_TOOL_NAME_DIMENSION GO_MEASUREMENTS_NAME_CENTER_X, GO_MEASUREMENTS_NAME_CENTER_X, kTypeOf(GoProfileDimCenterX)));
        kTest(GoMeasurements_NameTypeMapAdd(GO_PROFILE_TOOL_NAME_DIMENSION GO_MEASUREMENTS_NAME_CENTER_Z, GO_MEASUREMENTS_NAME_CENTER_Z, kTypeOf(GoProfileDimCenterZ)));

        kTest(GoMeasurements_NameTypeMapAdd(GO_PROFILE_TOOL_NAME_GROOVE GO_MEASUREMENTS_NAME_X, GO_MEASUREMENTS_NAME_X, kTypeOf(GoProfileGrooveX)));
        kTest(GoMeasurements_NameTypeMapAdd(GO_PROFILE_TOOL_NAME_GROOVE GO_MEASUREMENTS_NAME_Z, GO_MEASUREMENTS_NAME_Z, kTypeOf(GoProfileGrooveZ)));
        kTest(GoMeasurements_NameTypeMapAdd(GO_PROFILE_TOOL_NAME_GROOVE GO_MEASUREMENTS_NAME_WIDTH, GO_MEASUREMENTS_NAME_WIDTH, kTypeOf(GoProfileGrooveWidth)));
        kTest(GoMeasurements_NameTypeMapAdd(GO_PROFILE_TOOL_NAME_GROOVE GO_MEASUREMENTS_NAME_DEPTH, GO_MEASUREMENTS_NAME_DEPTH, kTypeOf(GoProfileGrooveDepth)));

        kTest(GoMeasurements_NameTypeMapAdd(GO_PROFILE_TOOL_NAME_INTERSECT GO_MEASUREMENTS_NAME_X, GO_MEASUREMENTS_NAME_X, kTypeOf(GoProfileIntersectX)));
        kTest(GoMeasurements_NameTypeMapAdd(GO_PROFILE_TOOL_NAME_INTERSECT GO_MEASUREMENTS_NAME_Z, GO_MEASUREMENTS_NAME_Z, kTypeOf(GoProfileIntersectZ)));
        kTest(GoMeasurements_NameTypeMapAdd(GO_PROFILE_TOOL_NAME_INTERSECT GO_MEASUREMENTS_NAME_ANGLE, GO_MEASUREMENTS_NAME_ANGLE, kTypeOf(GoProfileIntersectAngle)));

        kTest(GoMeasurements_NameTypeMapAdd(GO_PROFILE_TOOL_NAME_LINE GO_MEASUREMENTS_NAME_STD_DEV, GO_MEASUREMENTS_NAME_STD_DEV, kTypeOf(GoProfileLineStdDev)));
        kTest(GoMeasurements_NameTypeMapAdd(GO_PROFILE_TOOL_NAME_LINE GO_MEASUREMENTS_NAME_MAX_ERROR, GO_MEASUREMENTS_NAME_MAX_ERROR, kTypeOf(GoProfileLineMaxError)));
        kTest(GoMeasurements_NameTypeMapAdd(GO_PROFILE_TOOL_NAME_LINE GO_MEASUREMENTS_NAME_MIN_ERROR, GO_MEASUREMENTS_NAME_MIN_ERROR, kTypeOf(GoProfileLineMinError)));
        kTest(GoMeasurements_NameTypeMapAdd(GO_PROFILE_TOOL_NAME_LINE GO_MEASUREMENTS_NAME_PERCENTILE, GO_MEASUREMENTS_NAME_PERCENTILE, kTypeOf(GoProfileLinePercentile)));
        kTest(GoMeasurements_NameTypeMapAdd(GO_PROFILE_TOOL_NAME_LINE GO_MEASUREMENTS_NAME_OFFSET, GO_MEASUREMENTS_NAME_OFFSET, kTypeOf(GoProfileLineOffset)));
        kTest(GoMeasurements_NameTypeMapAdd(GO_PROFILE_TOOL_NAME_LINE GO_MEASUREMENTS_NAME_ANGLE, GO_MEASUREMENTS_NAME_ANGLE, kTypeOf(GoProfileLineAngle)));
        kTest(GoMeasurements_NameTypeMapAdd(GO_PROFILE_TOOL_NAME_LINE GO_MEASUREMENTS_NAME_MIN_ERROR_X, GO_MEASUREMENTS_NAME_MIN_ERROR_X, kTypeOf(GoProfileLineMinErrorX)));
        kTest(GoMeasurements_NameTypeMapAdd(GO_PROFILE_TOOL_NAME_LINE GO_MEASUREMENTS_NAME_MIN_ERROR_Z, GO_MEASUREMENTS_NAME_MIN_ERROR_Z, kTypeOf(GoProfileLineMinErrorZ)));
        kTest(GoMeasurements_NameTypeMapAdd(GO_PROFILE_TOOL_NAME_LINE GO_MEASUREMENTS_NAME_MAX_ERROR_X, GO_MEASUREMENTS_NAME_MAX_ERROR_X, kTypeOf(GoProfileLineMaxErrorX)));
        kTest(GoMeasurements_NameTypeMapAdd(GO_PROFILE_TOOL_NAME_LINE GO_MEASUREMENTS_NAME_MAX_ERROR_Z, GO_MEASUREMENTS_NAME_MAX_ERROR_Z, kTypeOf(GoProfileLineMaxErrorZ)));

        kTest(GoMeasurements_NameTypeMapAdd(GO_PROFILE_TOOL_NAME_PANEL GO_MEASUREMENTS_NAME_GAP, GO_MEASUREMENTS_NAME_GAP, kTypeOf(GoProfilePanelGap)));
        kTest(GoMeasurements_NameTypeMapAdd(GO_PROFILE_TOOL_NAME_PANEL GO_MEASUREMENTS_NAME_FLUSH, GO_MEASUREMENTS_NAME_FLUSH, kTypeOf(GoProfilePanelFlush)));
        kTest(GoMeasurements_NameTypeMapAdd(GO_PROFILE_TOOL_NAME_PANEL GO_MEASUREMENTS_NAME_LEFT_FLUSH_X, GO_MEASUREMENTS_NAME_LEFT_FLUSH_X, kTypeOf(GoProfilePanelLeftFlushX)));
        kTest(GoMeasurements_NameTypeMapAdd(GO_PROFILE_TOOL_NAME_PANEL GO_MEASUREMENTS_NAME_LEFT_FLUSH_Z, GO_MEASUREMENTS_NAME_LEFT_FLUSH_Z, kTypeOf(GoProfilePanelLeftFlushZ)));
        kTest(GoMeasurements_NameTypeMapAdd(GO_PROFILE_TOOL_NAME_PANEL GO_MEASUREMENTS_NAME_LEFT_GAP_X, GO_MEASUREMENTS_NAME_LEFT_GAP_X, kTypeOf(GoProfilePanelLeftGapX)));
        kTest(GoMeasurements_NameTypeMapAdd(GO_PROFILE_TOOL_NAME_PANEL GO_MEASUREMENTS_NAME_LEFT_GAP_Z, GO_MEASUREMENTS_NAME_LEFT_GAP_Z, kTypeOf(GoProfilePanelLeftGapZ)));
        kTest(GoMeasurements_NameTypeMapAdd(GO_PROFILE_TOOL_NAME_PANEL GO_MEASUREMENTS_NAME_LEFT_SURFACE_ANGLE, GO_MEASUREMENTS_NAME_LEFT_SURFACE_ANGLE, kTypeOf(GoProfilePanelLeftSurfaceAngle)));
        kTest(GoMeasurements_NameTypeMapAdd(GO_PROFILE_TOOL_NAME_PANEL GO_MEASUREMENTS_NAME_RIGHT_FLUSH_X, GO_MEASUREMENTS_NAME_RIGHT_FLUSH_X, kTypeOf(GoProfilePanelRightFlushX)));
        kTest(GoMeasurements_NameTypeMapAdd(GO_PROFILE_TOOL_NAME_PANEL GO_MEASUREMENTS_NAME_RIGHT_FLUSH_Z, GO_MEASUREMENTS_NAME_RIGHT_FLUSH_Z, kTypeOf(GoProfilePanelRightFlushZ)));
        kTest(GoMeasurements_NameTypeMapAdd(GO_PROFILE_TOOL_NAME_PANEL GO_MEASUREMENTS_NAME_RIGHT_GAP_X, GO_MEASUREMENTS_NAME_RIGHT_GAP_X, kTypeOf(GoProfilePanelRightGapX)));
        kTest(GoMeasurements_NameTypeMapAdd(GO_PROFILE_TOOL_NAME_PANEL GO_MEASUREMENTS_NAME_RIGHT_GAP_Z, GO_MEASUREMENTS_NAME_RIGHT_GAP_Z, kTypeOf(GoProfilePanelRightGapZ)));
        kTest(GoMeasurements_NameTypeMapAdd(GO_PROFILE_TOOL_NAME_PANEL GO_MEASUREMENTS_NAME_RIGHT_SURFACE_ANGLE, GO_MEASUREMENTS_NAME_RIGHT_SURFACE_ANGLE, kTypeOf(GoProfilePanelRightSurfaceAngle)));

        kTest(GoMeasurements_NameTypeMapAdd(GO_PROFILE_TOOL_NAME_ROUND_CORNER GO_MEASUREMENTS_NAME_X, GO_MEASUREMENTS_NAME_X, kTypeOf(GoProfileRoundCornerX)));
        kTest(GoMeasurements_NameTypeMapAdd(GO_PROFILE_TOOL_NAME_ROUND_CORNER GO_MEASUREMENTS_NAME_Z, GO_MEASUREMENTS_NAME_Z, kTypeOf(GoProfileRoundCornerZ)));
        kTest(GoMeasurements_NameTypeMapAdd(GO_PROFILE_TOOL_NAME_ROUND_CORNER GO_MEASUREMENTS_NAME_ANGLE, GO_MEASUREMENTS_NAME_ANGLE, kTypeOf(GoProfileRoundCornerAngle)));

        kTest(GoMeasurements_NameTypeMapAdd(GO_PROFILE_TOOL_NAME_POSITION GO_MEASUREMENTS_NAME_X, GO_MEASUREMENTS_NAME_X, kTypeOf(GoProfilePositionX)));
        kTest(GoMeasurements_NameTypeMapAdd(GO_PROFILE_TOOL_NAME_POSITION GO_MEASUREMENTS_NAME_Z, GO_MEASUREMENTS_NAME_Z, kTypeOf(GoProfilePositionZ)));

        kTest(GoMeasurements_NameTypeMapAdd(GO_PROFILE_TOOL_NAME_STRIP GO_MEASUREMENTS_NAME_X, GO_MEASUREMENTS_NAME_X, kTypeOf(GoProfileStripX)));
        kTest(GoMeasurements_NameTypeMapAdd(GO_PROFILE_TOOL_NAME_STRIP GO_MEASUREMENTS_NAME_Z, GO_MEASUREMENTS_NAME_Z, kTypeOf(GoProfileStripZ)));
        kTest(GoMeasurements_NameTypeMapAdd(GO_PROFILE_TOOL_NAME_STRIP GO_MEASUREMENTS_NAME_WIDTH, GO_MEASUREMENTS_NAME_WIDTH, kTypeOf(GoProfileStripWidth)));
        kTest(GoMeasurements_NameTypeMapAdd(GO_PROFILE_TOOL_NAME_STRIP GO_MEASUREMENTS_NAME_HEIGHT, GO_MEASUREMENTS_NAME_HEIGHT, kTypeOf(GoProfileStripHeight)));

        kTest(GoMeasurements_NameTypeMapAdd(GO_SURFACE_TOOL_NAME_BOUNDING_BOX GO_MEASUREMENTS_NAME_X, GO_MEASUREMENTS_NAME_X, kTypeOf(GoSurfaceBoxX)));
        kTest(GoMeasurements_NameTypeMapAdd(GO_SURFACE_TOOL_NAME_BOUNDING_BOX GO_MEASUREMENTS_NAME_Y, GO_MEASUREMENTS_NAME_Y, kTypeOf(GoSurfaceBoxY)));
        kTest(GoMeasurements_NameTypeMapAdd(GO_SURFACE_TOOL_NAME_BOUNDING_BOX GO_MEASUREMENTS_NAME_Z, GO_MEASUREMENTS_NAME_Z, kTypeOf(GoSurfaceBoxZ)));
        kTest(GoMeasurements_NameTypeMapAdd(GO_SURFACE_TOOL_NAME_BOUNDING_BOX GO_MEASUREMENTS_NAME_WIDTH, GO_MEASUREMENTS_NAME_WIDTH, kTypeOf(GoSurfaceBoxWidth)));
        kTest(GoMeasurements_NameTypeMapAdd(GO_SURFACE_TOOL_NAME_BOUNDING_BOX GO_MEASUREMENTS_NAME_LENGTH, GO_MEASUREMENTS_NAME_LENGTH, kTypeOf(GoSurfaceBoxLength)));
        kTest(GoMeasurements_NameTypeMapAdd(GO_SURFACE_TOOL_NAME_BOUNDING_BOX GO_MEASUREMENTS_NAME_HEIGHT, GO_MEASUREMENTS_NAME_HEIGHT, kTypeOf(GoSurfaceBoxHeight)));
        kTest(GoMeasurements_NameTypeMapAdd(GO_SURFACE_TOOL_NAME_BOUNDING_BOX GO_MEASUREMENTS_NAME_Z_ANGLE, GO_MEASUREMENTS_NAME_Z_ANGLE, kTypeOf(GoSurfaceBoxZAngle)));
        kTest(GoMeasurements_NameTypeMapAdd(GO_SURFACE_TOOL_NAME_BOUNDING_BOX GO_MEASUREMENTS_NAME_GLOBAL_X, GO_MEASUREMENTS_NAME_GLOBAL_X, kTypeOf(GoSurfaceBoxGlobalX)));
        kTest(GoMeasurements_NameTypeMapAdd(GO_SURFACE_TOOL_NAME_BOUNDING_BOX GO_MEASUREMENTS_NAME_GLOBAL_Y, GO_MEASUREMENTS_NAME_GLOBAL_Y, kTypeOf(GoSurfaceBoxGlobalY)));
        kTest(GoMeasurements_NameTypeMapAdd(GO_SURFACE_TOOL_NAME_BOUNDING_BOX GO_MEASUREMENTS_NAME_GLOBAL_Z_ANGLE, GO_MEASUREMENTS_NAME_GLOBAL_Z_ANGLE, kTypeOf(GoSurfaceBoxGlobalZAngle)));

        kTest(GoMeasurements_NameTypeMapAdd(GO_SURFACE_TOOL_NAME_COUNTERSUNK_HOLE GO_MEASUREMENTS_NAME_X, GO_MEASUREMENTS_NAME_X, kTypeOf(GoSurfaceCountersunkHoleX)));
        kTest(GoMeasurements_NameTypeMapAdd(GO_SURFACE_TOOL_NAME_COUNTERSUNK_HOLE GO_MEASUREMENTS_NAME_Y, GO_MEASUREMENTS_NAME_Y, kTypeOf(GoSurfaceCountersunkHoleY)));
        kTest(GoMeasurements_NameTypeMapAdd(GO_SURFACE_TOOL_NAME_COUNTERSUNK_HOLE GO_MEASUREMENTS_NAME_Z, GO_MEASUREMENTS_NAME_Z, kTypeOf(GoSurfaceCountersunkHoleZ)));
        kTest(GoMeasurements_NameTypeMapAdd(GO_SURFACE_TOOL_NAME_COUNTERSUNK_HOLE GO_MEASUREMENTS_NAME_OUTER_RADIUS, GO_MEASUREMENTS_NAME_OUTER_RADIUS, kTypeOf(GoSurfaceCountersunkHoleOuterRadius)));
        kTest(GoMeasurements_NameTypeMapAdd(GO_SURFACE_TOOL_NAME_COUNTERSUNK_HOLE GO_MEASUREMENTS_NAME_DEPTH, GO_MEASUREMENTS_NAME_DEPTH, kTypeOf(GoSurfaceCountersunkHoleDepth)));
        kTest(GoMeasurements_NameTypeMapAdd(GO_SURFACE_TOOL_NAME_COUNTERSUNK_HOLE GO_MEASUREMENTS_NAME_BEVEL_ANGLE, GO_MEASUREMENTS_NAME_BEVEL_ANGLE, kTypeOf(GoSurfaceCountersunkHoleBevelAngle)));
        kTest(GoMeasurements_NameTypeMapAdd(GO_SURFACE_TOOL_NAME_COUNTERSUNK_HOLE GO_MEASUREMENTS_NAME_BEVEL_RADIUS, GO_MEASUREMENTS_NAME_BEVEL_RADIUS, kTypeOf(GoSurfaceCountersunkHoleBevelRadius)));
        kTest(GoMeasurements_NameTypeMapAdd(GO_SURFACE_TOOL_NAME_COUNTERSUNK_HOLE GO_MEASUREMENTS_NAME_X_ANGLE, GO_MEASUREMENTS_NAME_X_ANGLE, kTypeOf(GoSurfaceCountersunkHoleXAngle)));
        kTest(GoMeasurements_NameTypeMapAdd(GO_SURFACE_TOOL_NAME_COUNTERSUNK_HOLE GO_MEASUREMENTS_NAME_Y_ANGLE, GO_MEASUREMENTS_NAME_Y_ANGLE, kTypeOf(GoSurfaceCountersunkHoleYAngle)));
        kTest(GoMeasurements_NameTypeMapAdd(GO_SURFACE_TOOL_NAME_COUNTERSUNK_HOLE GO_MEASUREMENTS_NAME_COUNTERBORE_DEPTH, GO_MEASUREMENTS_NAME_COUNTERBORE_DEPTH, kTypeOf(GoSurfaceCountersunkHoleCounterboreDepth)));
        kTest(GoMeasurements_NameTypeMapAdd(GO_SURFACE_TOOL_NAME_COUNTERSUNK_HOLE GO_MEASUREMENTS_NAME_AXIS_TILT, GO_MEASUREMENTS_NAME_AXIS_TILT, kTypeOf(GoSurfaceCountersunkHoleAxisTilt)));
        kTest(GoMeasurements_NameTypeMapAdd(GO_SURFACE_TOOL_NAME_COUNTERSUNK_HOLE GO_MEASUREMENTS_NAME_AXIS_ORIENTATION, GO_MEASUREMENTS_NAME_AXIS_ORIENTATION, kTypeOf(GoSurfaceCountersunkHoleAxisOrientation)));

        kTest(GoMeasurements_NameTypeMapAdd(GO_SURFACE_TOOL_NAME_DIMENSION GO_MEASUREMENTS_NAME_WIDTH, GO_MEASUREMENTS_NAME_WIDTH, kTypeOf(GoSurfaceDimWidth)));
        kTest(GoMeasurements_NameTypeMapAdd(GO_SURFACE_TOOL_NAME_DIMENSION GO_MEASUREMENTS_NAME_HEIGHT, GO_MEASUREMENTS_NAME_HEIGHT, kTypeOf(GoSurfaceDimHeight)));
        kTest(GoMeasurements_NameTypeMapAdd(GO_SURFACE_TOOL_NAME_DIMENSION GO_MEASUREMENTS_NAME_LENGTH, GO_MEASUREMENTS_NAME_LENGTH, kTypeOf(GoSurfaceDimLength)));
        kTest(GoMeasurements_NameTypeMapAdd(GO_SURFACE_TOOL_NAME_DIMENSION GO_MEASUREMENTS_NAME_DISTANCE, GO_MEASUREMENTS_NAME_DISTANCE, kTypeOf(GoSurfaceDimDistance)));
        kTest(GoMeasurements_NameTypeMapAdd(GO_SURFACE_TOOL_NAME_DIMENSION GO_MEASUREMENTS_NAME_PLANE_DISTANCE, GO_MEASUREMENTS_NAME_PLANE_DISTANCE, kTypeOf(GoSurfaceDimPlaneDistance)));
        kTest(GoMeasurements_NameTypeMapAdd(GO_SURFACE_TOOL_NAME_DIMENSION GO_MEASUREMENTS_NAME_CENTER_X, GO_MEASUREMENTS_NAME_CENTER_X, kTypeOf(GoSurfaceDimCenterX)));
        kTest(GoMeasurements_NameTypeMapAdd(GO_SURFACE_TOOL_NAME_DIMENSION GO_MEASUREMENTS_NAME_CENTER_Y, GO_MEASUREMENTS_NAME_CENTER_Y, kTypeOf(GoSurfaceDimCenterY)));
        kTest(GoMeasurements_NameTypeMapAdd(GO_SURFACE_TOOL_NAME_DIMENSION GO_MEASUREMENTS_NAME_CENTER_Z, GO_MEASUREMENTS_NAME_CENTER_Z, kTypeOf(GoSurfaceDimCenterZ)));

        // NOTE: Surface Edge tool is special. Its tool+measurement name is ToolMeasurement so its type is GoExtMeasurement.

        kTest(GoMeasurements_NameTypeMapAdd(GO_SURFACE_TOOL_NAME_ELLIPSE GO_MEASUREMENTS_NAME_MAJOR, GO_MEASUREMENTS_NAME_MAJOR, kTypeOf(GoSurfaceEllipseMajor)));
        kTest(GoMeasurements_NameTypeMapAdd(GO_SURFACE_TOOL_NAME_ELLIPSE GO_MEASUREMENTS_NAME_MINOR, GO_MEASUREMENTS_NAME_MINOR, kTypeOf(GoSurfaceEllipseMinor)));
        kTest(GoMeasurements_NameTypeMapAdd(GO_SURFACE_TOOL_NAME_ELLIPSE GO_MEASUREMENTS_NAME_RATIO, GO_MEASUREMENTS_NAME_RATIO, kTypeOf(GoSurfaceEllipseRatio)));
        kTest(GoMeasurements_NameTypeMapAdd(GO_SURFACE_TOOL_NAME_ELLIPSE GO_MEASUREMENTS_NAME_Z_ANGLE, GO_MEASUREMENTS_NAME_Z_ANGLE, kTypeOf(GoSurfaceEllipseZAngle)));

        kTest(GoMeasurements_NameTypeMapAdd(GO_SURFACE_TOOL_NAME_HOLE GO_MEASUREMENTS_NAME_X, GO_MEASUREMENTS_NAME_X, kTypeOf(GoSurfaceHoleX)));
        kTest(GoMeasurements_NameTypeMapAdd(GO_SURFACE_TOOL_NAME_HOLE GO_MEASUREMENTS_NAME_Y, GO_MEASUREMENTS_NAME_Y, kTypeOf(GoSurfaceHoleY)));
        kTest(GoMeasurements_NameTypeMapAdd(GO_SURFACE_TOOL_NAME_HOLE GO_MEASUREMENTS_NAME_Z, GO_MEASUREMENTS_NAME_Z, kTypeOf(GoSurfaceHoleZ)));
        kTest(GoMeasurements_NameTypeMapAdd(GO_SURFACE_TOOL_NAME_HOLE GO_MEASUREMENTS_NAME_RADIUS, GO_MEASUREMENTS_NAME_RADIUS, kTypeOf(GoSurfaceHoleRadius)));

        kTest(GoMeasurements_NameTypeMapAdd(GO_SURFACE_TOOL_NAME_OPENING GO_MEASUREMENTS_NAME_X, GO_MEASUREMENTS_NAME_X, kTypeOf(GoSurfaceOpeningX)));
        kTest(GoMeasurements_NameTypeMapAdd(GO_SURFACE_TOOL_NAME_OPENING GO_MEASUREMENTS_NAME_Y, GO_MEASUREMENTS_NAME_Y, kTypeOf(GoSurfaceOpeningY)));
        kTest(GoMeasurements_NameTypeMapAdd(GO_SURFACE_TOOL_NAME_OPENING GO_MEASUREMENTS_NAME_Z, GO_MEASUREMENTS_NAME_Z, kTypeOf(GoSurfaceOpeningZ)));
        kTest(GoMeasurements_NameTypeMapAdd(GO_SURFACE_TOOL_NAME_OPENING GO_MEASUREMENTS_NAME_WIDTH, GO_MEASUREMENTS_NAME_WIDTH, kTypeOf(GoSurfaceOpeningWidth)));
        kTest(GoMeasurements_NameTypeMapAdd(GO_SURFACE_TOOL_NAME_OPENING GO_MEASUREMENTS_NAME_LENGTH, GO_MEASUREMENTS_NAME_LENGTH, kTypeOf(GoSurfaceOpeningLength)));
        kTest(GoMeasurements_NameTypeMapAdd(GO_SURFACE_TOOL_NAME_OPENING GO_MEASUREMENTS_NAME_ANGLE, GO_MEASUREMENTS_NAME_ANGLE, kTypeOf(GoSurfaceOpeningAngle)));

        kTest(GoMeasurements_NameTypeMapAdd(GO_SURFACE_TOOL_NAME_PLANE GO_MEASUREMENTS_NAME_X_ANGLE, GO_MEASUREMENTS_NAME_X_ANGLE, kTypeOf(GoSurfacePlaneXAngle)));
        kTest(GoMeasurements_NameTypeMapAdd(GO_SURFACE_TOOL_NAME_PLANE GO_MEASUREMENTS_NAME_Y_ANGLE, GO_MEASUREMENTS_NAME_Y_ANGLE, kTypeOf(GoSurfacePlaneYAngle)));
        kTest(GoMeasurements_NameTypeMapAdd(GO_SURFACE_TOOL_NAME_PLANE GO_MEASUREMENTS_NAME_Z_OFFSET, GO_MEASUREMENTS_NAME_Z_OFFSET, kTypeOf(GoSurfacePlaneZOffset)));
        kTest(GoMeasurements_NameTypeMapAdd(GO_SURFACE_TOOL_NAME_PLANE GO_MEASUREMENTS_NAME_STD_DEV, GO_MEASUREMENTS_NAME_STD_DEV, kTypeOf(GoSurfacePlaneStdDev)));
        kTest(GoMeasurements_NameTypeMapAdd(GO_SURFACE_TOOL_NAME_PLANE GO_MEASUREMENTS_NAME_MIN_ERROR, GO_MEASUREMENTS_NAME_MIN_ERROR, kTypeOf(GoSurfacePlaneMinError)));
        kTest(GoMeasurements_NameTypeMapAdd(GO_SURFACE_TOOL_NAME_PLANE GO_MEASUREMENTS_NAME_MAX_ERROR, GO_MEASUREMENTS_NAME_MAX_ERROR, kTypeOf(GoSurfacePlaneMaxError)));
        kTest(GoMeasurements_NameTypeMapAdd(GO_SURFACE_TOOL_NAME_PLANE GO_MEASUREMENTS_NAME_X_NORMAL, GO_MEASUREMENTS_NAME_X_NORMAL, kTypeOf(GoSurfacePlaneXNormal)));
        kTest(GoMeasurements_NameTypeMapAdd(GO_SURFACE_TOOL_NAME_PLANE GO_MEASUREMENTS_NAME_Y_NORMAL, GO_MEASUREMENTS_NAME_Y_NORMAL, kTypeOf(GoSurfacePlaneYNormal)));
        kTest(GoMeasurements_NameTypeMapAdd(GO_SURFACE_TOOL_NAME_PLANE GO_MEASUREMENTS_NAME_Z_NORMAL, GO_MEASUREMENTS_NAME_Z_NORMAL, kTypeOf(GoSurfacePlaneZNormal)));
        kTest(GoMeasurements_NameTypeMapAdd(GO_SURFACE_TOOL_NAME_PLANE GO_MEASUREMENTS_NAME_DISTANCE, GO_MEASUREMENTS_NAME_DISTANCE, kTypeOf(GoSurfacePlaneDistance)));

        kTest(GoMeasurements_NameTypeMapAdd(GO_SURFACE_TOOL_NAME_POSITION GO_MEASUREMENTS_NAME_X, GO_MEASUREMENTS_NAME_X, kTypeOf(GoSurfacePositionX)));
        kTest(GoMeasurements_NameTypeMapAdd(GO_SURFACE_TOOL_NAME_POSITION GO_MEASUREMENTS_NAME_Y, GO_MEASUREMENTS_NAME_Y, kTypeOf(GoSurfacePositionY)));
        kTest(GoMeasurements_NameTypeMapAdd(GO_SURFACE_TOOL_NAME_POSITION GO_MEASUREMENTS_NAME_Z, GO_MEASUREMENTS_NAME_Z, kTypeOf(GoSurfacePositionZ)));

        // TODO: confirm if surface rivet tool needs support in SDK. Original code commented out support in the ParseType() function,
        //       but provided support in FormatType() function.

        /*kTest(GoMeasurements_NameTypeMapAdd(GO_SURFACE_TOOL_NAME_RIVET GO_MEASUREMENTS_NAME_X, GO_MEASUREMENTS_NAME_X, kTypeOf(GoSurfaceRivetX)));
        kTest(GoMeasurements_NameTypeMapAdd(GO_SURFACE_TOOL_NAME_RIVET GO_MEASUREMENTS_NAME_Y, GO_MEASUREMENTS_NAME_Y, kTypeOf(GoSurfaceRivetY)));
        kTest(GoMeasurements_NameTypeMapAdd(GO_SURFACE_TOOL_NAME_RIVET GO_MEASUREMENTS_NAME_Z, GO_MEASUREMENTS_NAME_Z, kTypeOf(GoSurfaceRivetZ)));
        kTest(GoMeasurements_NameTypeMapAdd(GO_SURFACE_TOOL_NAME_RIVET GO_MEASUREMENTS_NAME_TILT_ANGLE, GO_MEASUREMENTS_NAME_TILT_ANGLE, kTypeOf(GoSurfaceRivetTiltAngle)));
        kTest(GoMeasurements_NameTypeMapAdd(GO_SURFACE_TOOL_NAME_RIVET GO_MEASUREMENTS_NAME_TILT_DIRECTION, GO_MEASUREMENTS_NAME_TILT_DIRECTION, kTypeOf(GoSurfaceRivetTiltDirection)));
        kTest(GoMeasurements_NameTypeMapAdd(GO_SURFACE_TOOL_NAME_RIVET GO_MEASUREMENTS_NAME_RADIUS, GO_MEASUREMENTS_NAME_RADIUS, kTypeOf(GoSurfaceRivetRadius)));
        kTest(GoMeasurements_NameTypeMapAdd(GO_SURFACE_TOOL_NAME_RIVET GO_MEASUREMENTS_NAME_TOP_OFFSET_MIN, GO_MEASUREMENTS_NAME_TOP_OFFSET_MIN, kTypeOf(GoSurfaceRivetTopOffsetMin)));
        kTest(GoMeasurements_NameTypeMapAdd(GO_SURFACE_TOOL_NAME_RIVET GO_MEASUREMENTS_NAME_TOP_OFFSET_MAX, GO_MEASUREMENTS_NAME_TOP_OFFSET_MAX, kTypeOf(GoSurfaceRivetTopOffsetMax)));
        kTest(GoMeasurements_NameTypeMapAdd(GO_SURFACE_TOOL_NAME_RIVET GO_MEASUREMENTS_NAME_TOP_OFFSET_MEAN, GO_MEASUREMENTS_NAME_TOP_OFFSET_MEAN, kTypeOf(GoSurfaceRivetTopOffsetMean)));
        kTest(GoMeasurements_NameTypeMapAdd(GO_SURFACE_TOOL_NAME_RIVET GO_MEASUREMENTS_NAME_TOP_OFFSET_STD_DEV, GO_MEASUREMENTS_NAME_TOP_OFFSET_STD_DEV, kTypeOf(GoSurfaceRivetTopOffsetStdDev)));
        kTest(GoMeasurements_NameTypeMapAdd(GO_SURFACE_TOOL_NAME_RIVET GO_MEASUREMENTS_NAME_RADIAL_HEIGHT_MIN, GO_MEASUREMENTS_NAME_RADIAL_HEIGHT_MIN, kTypeOf(GoSurfaceRivetRadialHeightMin)));
        kTest(GoMeasurements_NameTypeMapAdd(GO_SURFACE_TOOL_NAME_RIVET GO_MEASUREMENTS_NAME_RADIAL_HEIGHT_MAX, GO_MEASUREMENTS_NAME_RADIAL_HEIGHT_MAX, kTypeOf(GoSurfaceRivetRadialHeightMax)));
        kTest(GoMeasurements_NameTypeMapAdd(GO_SURFACE_TOOL_NAME_RIVET GO_MEASUREMENTS_NAME_RADIAL_HEIGHT_MEAN, GO_MEASUREMENTS_NAME_RADIAL_HEIGHT_MEAN, kTypeOf(GoSurfaceRivetRadialHeightMean)));
        kTest(GoMeasurements_NameTypeMapAdd(GO_SURFACE_TOOL_NAME_RIVET GO_MEASUREMENTS_NAME_RADIAL_HEIGHT_STD_DEV, GO_MEASUREMENTS_NAME_RADIAL_HEIGHT_STD_DEV, kTypeOf(GoSurfaceRivetRadialHeightStdDev)));
        kTest(GoMeasurements_NameTypeMapAdd(GO_SURFACE_TOOL_NAME_RIVET GO_MEASUREMENTS_NAME_RADIAL_SLOPE_MIN, GO_MEASUREMENTS_NAME_RADIAL_SLOPE_MIN, kTypeOf(GoSurfaceRivetRadialSlopeMin)));
        kTest(GoMeasurements_NameTypeMapAdd(GO_SURFACE_TOOL_NAME_RIVET GO_MEASUREMENTS_NAME_RADIAL_SLOPE_MAX, GO_MEASUREMENTS_NAME_RADIAL_SLOPE_MAX, kTypeOf(GoSurfaceRivetRadialSlopeMax)));
        kTest(GoMeasurements_NameTypeMapAdd(GO_SURFACE_TOOL_NAME_RIVET GO_MEASUREMENTS_NAME_RADIAL_SLOPE_MEAN, GO_MEASUREMENTS_NAME_RADIAL_SLOPE_MEAN, kTypeOf(GoSurfaceRivetRadialSlopeMean)));
        kTest(GoMeasurements_NameTypeMapAdd(GO_SURFACE_TOOL_NAME_RIVET GO_MEASUREMENTS_NAME_RADIAL_SLOPE_STD_DEV, GO_MEASUREMENTS_NAME_RADIAL_SLOPE_STD_DEV, kTypeOf(GoSurfaceRivetRadialSlopeStdDev)));*/

        kTest(GoMeasurements_NameTypeMapAdd(GO_SURFACE_TOOL_NAME_STUD GO_MEASUREMENTS_NAME_BASE_X, GO_MEASUREMENTS_NAME_BASE_X, kTypeOf(GoSurfaceStudBaseX)));
        kTest(GoMeasurements_NameTypeMapAdd(GO_SURFACE_TOOL_NAME_STUD GO_MEASUREMENTS_NAME_BASE_Y, GO_MEASUREMENTS_NAME_BASE_Y, kTypeOf(GoSurfaceStudBaseY)));
        kTest(GoMeasurements_NameTypeMapAdd(GO_SURFACE_TOOL_NAME_STUD GO_MEASUREMENTS_NAME_BASE_Z, GO_MEASUREMENTS_NAME_BASE_Z, kTypeOf(GoSurfaceStudBaseZ)));
        kTest(GoMeasurements_NameTypeMapAdd(GO_SURFACE_TOOL_NAME_STUD GO_MEASUREMENTS_NAME_TIP_X, GO_MEASUREMENTS_NAME_TIP_X, kTypeOf(GoSurfaceStudTipX)));
        kTest(GoMeasurements_NameTypeMapAdd(GO_SURFACE_TOOL_NAME_STUD GO_MEASUREMENTS_NAME_TIP_Y, GO_MEASUREMENTS_NAME_TIP_Y, kTypeOf(GoSurfaceStudTipY)));
        kTest(GoMeasurements_NameTypeMapAdd(GO_SURFACE_TOOL_NAME_STUD GO_MEASUREMENTS_NAME_TIP_Z, GO_MEASUREMENTS_NAME_TIP_Z, kTypeOf(GoSurfaceStudTipZ)));
        kTest(GoMeasurements_NameTypeMapAdd(GO_SURFACE_TOOL_NAME_STUD GO_MEASUREMENTS_NAME_RADIUS, GO_MEASUREMENTS_NAME_RADIUS, kTypeOf(GoSurfaceStudRadius)));

        kTest(GoMeasurements_NameTypeMapAdd(GO_SURFACE_TOOL_NAME_VOLUME GO_MEASUREMENTS_NAME_VOLUME, GO_MEASUREMENTS_NAME_VOLUME, kTypeOf(GoSurfaceVolumeVolume)));
        kTest(GoMeasurements_NameTypeMapAdd(GO_SURFACE_TOOL_NAME_VOLUME GO_MEASUREMENTS_NAME_AREA, GO_MEASUREMENTS_NAME_AREA, kTypeOf(GoSurfaceVolumeArea)));
        kTest(GoMeasurements_NameTypeMapAdd(GO_SURFACE_TOOL_NAME_VOLUME GO_MEASUREMENTS_NAME_THICKNESS, GO_MEASUREMENTS_NAME_THICKNESS, kTypeOf(GoSurfaceVolumeThickness)));

        kTest(GoMeasurements_NameTypeMapAdd(GO_TOOLS_NAME_SCRIPT GO_MEASUREMENTS_NAME_OUTPUT, GO_MEASUREMENTS_NAME_OUTPUT, kTypeOf(GoScriptOutput)));

        // GOC-14127: The measurements mapping table has the same issue as the features mapping table.
        //
        // GDK measurement name -> GDK measurement type mapping:
        // Normal combinations of measurement names are:
        //  1. GO_EXT_TOOL_NAME_CUSTOM GO_MEASUREMENTS_NAME_CUSTOM
        //  2. GO_EXT_TOOL_NAME_TOOL GO_MEASUREMENTS_NAME_MEASUREMENT
        // Abnormal combinations are:
        //  3. GO_EXT_TOOL_NAME_CUSTOM GO_MEASUREMENTS_NAME_MEASUREMENT
        //  4. GO_EXT_TOOL_NAME_TOOL GO_MEASUREMENTS_NAME_CUSTOM
        //
        // In general, be lenient on what is read in when trying to map to the GDK measurement type so support even the
        // abnormal combinations.
        //
        // GDK measurement type -> GDK measurement name mapping
        // This table has multiple entries for GoExtMeasurement. When doing this mapping, the first entry found for
        // GoExtMeasurement is what is returned so this table always returns GO_MEASUREMENTS_NAME_CUSTOM 
        // for GoExtMeasurement, which is correct for USER GDK tools.
        // To support INTERNAL GDK tools, need extra code to expicitly check for INTERNAL GDK tool and bypass use
        // of this table. See GoMeasurements_FormatTypeExtToolFormat().
        kTest(GoMeasurements_NameTypeMapAdd(GO_EXT_TOOL_NAME_CUSTOM GO_MEASUREMENTS_NAME_CUSTOM, GO_MEASUREMENTS_NAME_CUSTOM, kTypeOf(GoExtMeasurement)));

        // TODO: Should these be obsoleted and deleted?? There is no unique
        //       mapping of type to measurement for GoExtMeasurement.
        //       For example, GoExtMeasurement, in legacy code,
        //       mapped to measurement "Custom" and "Measurement",
        //       but since "Custom" appeared first, that is what would have
        //       been returned every time. That is why the measurement
        //       name is set to "Custom" rather than "Measurement" in
        //       this mapping table code.
        kTest(GoMeasurements_NameTypeMapAdd(GO_EXT_TOOL_NAME_CUSTOM GO_MEASUREMENTS_NAME_MEASUREMENT, GO_MEASUREMENTS_NAME_CUSTOM, kTypeOf(GoExtMeasurement)));
        kTest(GoMeasurements_NameTypeMapAdd(GO_EXT_TOOL_NAME_TOOL GO_MEASUREMENTS_NAME_MEASUREMENT, GO_MEASUREMENTS_NAME_CUSTOM, kTypeOf(GoExtMeasurement)));

        // GOC-14127: Add support for this abnormal measurement name combination, for leniency reasons.
        kTest(GoMeasurements_NameTypeMapAdd(GO_EXT_TOOL_NAME_TOOL GO_MEASUREMENTS_NAME_CUSTOM, GO_MEASUREMENTS_NAME_CUSTOM, kTypeOf(GoExtMeasurement)));
    }
    kCatch(&exception)
    {
        // During development, fail hard if there are map table issues
        // such as the table being too small for all the entries.
        kAssert(0);
        kEndCatch(exception);
    }

    return kOK;
}

GoFx(kStatus) GoMeasurements_CreateNameTypeMap(void)
{
    kCheck(GoMeasurements_NameTypeMapInit());

    // The table should have at least one entry.
    kCheckTrue((GoMeasurements_NameTypeMapEntryCount() > 0), kERROR);

    // Table is now available.
    return kOK;
}

GoFx(kStatus) GoMeasurements_ParseTypeHelper(const kChar* name, kType* type)
{
    kStaticObj(GoMeasurements);
    kSize i;
    GoMeasurementsNameTypeMapping *map;
    kStatus status = kERROR_PARAMETER;

    for (i = 0; i < kArrayList_Count(sobj->nameTypeMapList); ++i)
    {
        map = kArrayList_AtT(sobj->nameTypeMapList, i, GoMeasurementsNameTypeMapping);
        if (kStrCompare(map->lookupName, name) == 0)
        {
            *type = map->measurementType;
            status = kOK;
            break;
        }
    }

    return status;
}

GoFx(kStatus) GoMeasurements_ParseType(const kChar* toolName, const kChar* measurementName, kType* type)
{
    kChar* name = kNULL;
    kSize capacity = kStrLength(toolName) + kStrLength(measurementName) + 1;

    kTry
    {
        // Concatenate the tool name and measurement name to form
        // the lookup name.
        kTest(kMemAllocZero(sizeof(kChar) * capacity, &name));
        kTest(kStrCopy(name, capacity, toolName));
        kTest(kStrCat(name, capacity, measurementName));

        kTest(GoMeasurements_ParseTypeHelper(name, type));
    }
    kFinally
    {
        if (name)
        {
            // Want to propagate the original failure in the try block so
            // don't check this memory free operation.
            kMemFree(name);
        }
        kEndFinally();
    }

    return kOK;
}

GoFx(kStatus) GoMeasurements_FormatTypeHelper(kType measurement, const kChar** outputPtr)
{
    kStaticObj(GoMeasurements);
    kSize i;
    GoMeasurementsNameTypeMapping map;

    for (i = 0; i < kArrayList_Count(sobj->nameTypeMapList); ++i)
    {
        map = *kArrayList_AtT(sobj->nameTypeMapList, i, GoMeasurementsNameTypeMapping);
        if (measurement == map.measurementType)
        {
            *outputPtr = map.measurementName;
            break;
        }
    }

    return kOK;
}

// Pre-existing call before GOC-14127 that only uses the mapping
// table to map measurement type to measurement name.
// Does not handle INTERNAL GDK tools, only USER GDK tools.
// GoMeasurements_FormatTypeExtToolFormat() adds support for INTERNAL
// GDK tool mapping of measurement type to measurement name.
GoFx(kStatus) GoMeasurements_FormatType(kType measurement, kChar* measurementName, kSize capacity)
{
    const kChar* output = kNULL;

    kCheck(GoMeasurements_FormatTypeHelper(measurement, &output));

    kCheck(kStrCopy(measurementName, capacity, output));

    return kOK;
}

// GOC-14127: This function is called internally by the SDK to get the measurement type name.
// The mapping table entries cannot differentiate between a USER or INTERNAL
// GDK GoExtMeasurement, so the table entries all return GO_MEASUREMENTS_NAME_CUSTOM.
// This API accepts a flag indicating if the measurement is from
// a INTERNAL tool or not, and return GO_MEASUREMENTS_NAME_MEASUREMENT for
// INTERNAL tool measurements.
GoFx(kStatus) GoMeasurements_FormatTypeExtToolFormat(kType measurement, kBool internalTool, kChar* measurementName, kSize capacity)
{
    const kChar* output = kNULL;

    if (internalTool)
    {
        // Internal GDK format tool XML node for measurements should be called 
        // GO_MEASUREMENTS_NAME_MEASUREMENT.
        // Older SDKs might have labelled the measurement as GO_MEASUREMENTS_NAME_CUSTOM, 
        // so ignore what is read and set the measurement name to the correct 
        // internal GDK measurement name.
        output = GO_MEASUREMENTS_NAME_MEASUREMENT;
    }
    else
    {
        // Format is standard tool format or user GDK format or not set (unknown),
        // so use the name from the mapping table.
        kCheck(GoMeasurements_FormatTypeHelper(measurement, &output));
    }

    kCheck(kStrCopy(measurementName, capacity, output));

    return kOK;
}

/* Range tool measurements */

kBeginClassEx(Go, GoRangePositionZ)
    kAddVMethod(GoRangePositionZ, GoMeasurement, VInit)
kEndClassEx()

GoFx(kStatus) GoRangePositionZ_VInit(GoRangePositionZ measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_RANGE_POSITION_Z, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


kBeginClassEx(Go, GoRangeThicknessThickness)
    kAddVMethod(GoRangeThicknessThickness, GoMeasurement, VInit)
kEndClassEx()

GoFx(kStatus) GoRangeThicknessThickness_VInit(GoRangeThicknessThickness measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_RANGE_THICKNESS_THICKNESS, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


/* Profile tool measurements */

kBeginClassEx(Go, GoProfileAreaArea)
    kAddVMethod(GoProfileAreaArea, GoMeasurement, VInit)
kEndClassEx()

GoFx(kStatus) GoProfileAreaArea_VInit(GoProfileAreaArea measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_PROFILE_AREA_AREA, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


kBeginClassEx(Go, GoProfileAreaCentroidX)
    kAddVMethod(GoProfileAreaCentroidX, GoMeasurement, VInit)
kEndClassEx()

GoFx(kStatus) GoProfileAreaCentroidX_VInit(GoProfileAreaCentroidX measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_PROFILE_AREA_CENTROID_X, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


kBeginClassEx(Go, GoProfileAreaCentroidZ)
    kAddVMethod(GoProfileAreaCentroidZ, GoMeasurement, VInit)
kEndClassEx()

GoFx(kStatus) GoProfileAreaCentroidZ_VInit(GoProfileAreaCentroidZ measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_PROFILE_AREA_CENTROID_Z, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


kBeginClassEx(Go, GoProfileBoxX)
kAddVMethod(GoProfileBoxX, GoMeasurement, VInit)
kEndClassEx()

GoFx(kStatus) GoProfileBoxX_VInit(GoProfileBoxX measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_PROFILE_BOUNDING_BOX_X, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


kBeginClassEx(Go, GoProfileBoxZ)
kAddVMethod(GoProfileBoxZ, GoMeasurement, VInit)
kEndClassEx()

GoFx(kStatus) GoProfileBoxZ_VInit(GoProfileBoxZ measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_PROFILE_BOUNDING_BOX_Z, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


kBeginClassEx(Go, GoProfileBoxWidth)
kAddVMethod(GoProfileBoxWidth, GoMeasurement, VInit)
kEndClassEx()


GoFx(kStatus) GoProfileBoxWidth_VInit(GoProfileBoxWidth measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_PROFILE_BOUNDING_BOX_WIDTH, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


kBeginClassEx(Go, GoProfileBoxHeight)
kAddVMethod(GoProfileBoxHeight, GoMeasurement, VInit)
kEndClassEx()

GoFx(kStatus) GoProfileBoxHeight_VInit(GoProfileBoxHeight measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_PROFILE_BOUNDING_BOX_HEIGHT, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


kBeginClassEx(Go, GoProfileBoxGlobalX)
kAddVMethod(GoProfileBoxGlobalX, GoMeasurement, VInit)
kEndClassEx()

GoFx(kStatus) GoProfileBoxGlobalX_VInit(GoProfileBoxGlobalX measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_PROFILE_BOUNDING_BOX_GLOBAL_X, sensor, srcTool, kTRUE, alloc));

    return kOK;
}

kBeginClassEx(Go, GoProfileBoxGlobalY)
kAddVMethod(GoProfileBoxGlobalY, GoMeasurement, VInit)
kEndClassEx()

GoFx(kStatus) GoProfileBoxGlobalY_VInit(GoProfileBoxGlobalX measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_PROFILE_BOUNDING_BOX_GLOBAL_Y, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


kBeginClassEx(Go, GoProfileBoxGlobalAngle)
kAddVMethod(GoProfileBoxGlobalAngle, GoMeasurement, VInit)
kEndClassEx()

GoFx(kStatus) GoProfileBoxGlobalAngle_VInit(GoProfileBoxGlobalAngle measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_PROFILE_BOUNDING_BOX_GLOBAL_ANGLE, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


kBeginClassEx(Go, GoProfileBridgeValueBridgeValue)
kAddVMethod(GoProfileBridgeValueBridgeValue, GoMeasurement, VInit)
kEndClassEx()

GoFx(kStatus) GoProfileBridgeValueBridgeValue_VInit(GoProfileBridgeValueBridgeValue measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_PROFILE_BRIDGE_VALUE_BRIDGE_VALUE, sensor, srcTool, kTRUE, alloc));

    return kOK;
}

kBeginClassEx(Go, GoProfileBridgeValueAngle)
kAddVMethod(GoProfileBridgeValueAngle, GoMeasurement, VInit)
kEndClassEx()

GoFx(kStatus) GoProfileBridgeValueAngle_VInit(GoProfileBridgeValueAngle measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_PROFILE_BRIDGE_VALUE_ANGLE, sensor, srcTool, kTRUE, alloc));

    return kOK;
}

kBeginClassEx(Go, GoProfileBridgeValueWindow)
kAddVMethod(GoProfileBridgeValueWindow, GoMeasurement, VInit)
kEndClassEx()

GoFx(kStatus) GoProfileBridgeValueWindow_VInit(GoProfileBridgeValueWindow measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_PROFILE_BRIDGE_VALUE_WINDOW, sensor, srcTool, kTRUE, alloc));

    return kOK;
}

kBeginClassEx(Go, GoProfileBridgeValueStdDev)
kAddVMethod(GoProfileBridgeValueStdDev, GoMeasurement, VInit)
kEndClassEx()

GoFx(kStatus) GoProfileBridgeValueStdDev_VInit(GoProfileBridgeValueStdDev measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_PROFILE_BRIDGE_VALUE_STDDEV, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


kBeginClassEx(Go, GoProfileCircleX)
    kAddVMethod(GoProfileCircleX, GoMeasurement, VInit)
kEndClassEx()

GoFx(kStatus) GoProfileCircleX_VInit(GoProfileCircleX measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_PROFILE_CIRCLE_X, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


kBeginClassEx(Go, GoProfileCircleZ)
    kAddVMethod(GoProfileCircleZ, GoMeasurement, VInit)
kEndClassEx()

GoFx(kStatus) GoProfileCircleZ_VInit(GoProfileCircleZ measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_PROFILE_CIRCLE_Z, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


kBeginClassEx(Go, GoProfileCircleRadius)
    kAddVMethod(GoProfileCircleRadius, GoMeasurement, VInit)
kEndClassEx()

GoFx(kStatus) GoProfileCircleRadius_VInit(GoProfileCircleRadius measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_PROFILE_CIRCLE_RADIUS, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


kBeginClassEx(Go, GoProfileCircleStdDev)
    kAddVMethod(GoProfileCircleStdDev, GoMeasurement, VInit)
kEndClassEx()

GoFx(kStatus) GoProfileCircleStdDev_VInit(GoProfileCircleStdDev measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_PROFILE_CIRCLE_STDDEV, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


kBeginClassEx(Go, GoProfileCircleMinError)
    kAddVMethod(GoProfileCircleMinError, GoMeasurement, VInit)
kEndClassEx()

GoFx(kStatus) GoProfileCircleMinError_VInit(GoProfileCircleMinError measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_PROFILE_CIRCLE_MIN_ERROR, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


kBeginClassEx(Go, GoProfileCircleMinErrorX)
    kAddVMethod(GoProfileCircleMinErrorX, GoMeasurement, VInit)
kEndClassEx()

GoFx(kStatus) GoProfileCircleMinErrorX_VInit(GoProfileCircleMinErrorX measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_PROFILE_CIRCLE_MIN_ERROR_X, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


kBeginClassEx(Go, GoProfileCircleMinErrorZ)
    kAddVMethod(GoProfileCircleMinErrorZ, GoMeasurement, VInit)
kEndClassEx()

GoFx(kStatus) GoProfileCircleMinErrorZ_VInit(GoProfileCircleMinErrorZ measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_PROFILE_CIRCLE_MIN_ERROR_Z, sensor, srcTool, kTRUE, alloc));

    return kOK;
}

kBeginClassEx(Go, GoProfileCircleMaxError)
    kAddVMethod(GoProfileCircleMaxError, GoMeasurement, VInit)
kEndClassEx()

GoFx(kStatus) GoProfileCircleMaxError_VInit(GoProfileCircleMaxError measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_PROFILE_CIRCLE_MAX_ERROR, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


kBeginClassEx(Go, GoProfileCircleMaxErrorX)
    kAddVMethod(GoProfileCircleMaxErrorX, GoMeasurement, VInit)
kEndClassEx()

GoFx(kStatus) GoProfileCircleMaxErrorX_VInit(GoProfileCircleMaxErrorX measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_PROFILE_CIRCLE_MAX_ERROR_X, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


kBeginClassEx(Go, GoProfileCircleMaxErrorZ)
    kAddVMethod(GoProfileCircleMaxErrorZ, GoMeasurement, VInit)
kEndClassEx()

GoFx(kStatus) GoProfileCircleMaxErrorZ_VInit(GoProfileCircleMaxErrorZ measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_PROFILE_CIRCLE_MAX_ERROR_Z, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


kBeginClassEx(Go, GoProfileDimWidth)
    kAddVMethod(GoProfileDimWidth, GoMeasurement, VInit)
    kAddVMethod(GoProfileDimWidth, GoMeasurement, VRead)
    kAddVMethod(GoProfileDimWidth, GoMeasurement, VWrite)
kEndClassEx()

GoFx(kStatus) GoProfileDimWidth_VInit(GoProfileDimWidth measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kObjR(GoProfileDimWidth, measurement);

    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_PROFILE_DIMENSION_WIDTH, sensor, srcTool, kTRUE, alloc));

    obj->absolute = kFALSE;

    return kOK;
}

GoFx(kStatus) GoProfileDimWidth_VRead(GoProfileDimWidth measurement, kXml xml, kXmlItem item)
{
    kObj(GoProfileDimWidth, measurement);

    kCheck(GoMeasurement_VRead(measurement, xml, item));
    kCheck(kXml_Child32s(xml, item, "Absolute", &obj->absolute));

    return kOK;
}

GoFx(kStatus) GoProfileDimWidth_VWrite(GoProfileDimWidth measurement, kXml xml, kXmlItem item)
{
    kObj(GoProfileDimWidth, measurement);

    kCheck(kXml_SetChild32s(xml, item, "Absolute", obj->absolute));
    kCheck(GoMeasurement_VWrite(measurement, xml, item));

    return kOK;
}

GoFx(kBool) GoProfileDimWidth_AbsoluteEnabled(GoProfileDimWidth measurement)
{
    kObj(GoProfileDimWidth, measurement);

    GoSensor_SyncConfig(GoMeasurement_Sensor(measurement));

    return obj->absolute;
}

GoFx(kStatus) GoProfileDimWidth_EnableAbsolute(GoProfileDimWidth measurement, kBool absolute)
{
    kObj(GoProfileDimWidth, measurement);

    kCheckState(GoSensor_IsConfigurable(GoMeasurement_Sensor(measurement)));
    kCheck(GoSensor_CacheConfig(GoMeasurement_Sensor(measurement)));
    obj->absolute = absolute;
    kCheck(GoSensor_SetConfigModified(GoMeasurement_Sensor(measurement)));

    return kOK;
}


kBeginClassEx(Go, GoProfileDimHeight)
    kAddVMethod(GoProfileDimHeight, GoMeasurement, VInit)
    kAddVMethod(GoProfileDimHeight, GoMeasurement, VRead)
    kAddVMethod(GoProfileDimHeight, GoMeasurement, VWrite)
kEndClassEx()

GoFx(kStatus) GoProfileDimHeight_VInit(GoProfileDimHeight measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kObjR(GoProfileDimHeight, measurement);

    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_PROFILE_DIMENSION_HEIGHT, sensor, srcTool, kTRUE, alloc));

    obj->absolute = kFALSE;

    return kOK;
}

GoFx(kStatus) GoProfileDimHeight_VRead(GoProfileDimHeight measurement, kXml xml, kXmlItem item)
{
    kObj(GoProfileDimHeight, measurement);

    kCheck(GoMeasurement_VRead(measurement, xml, item));
    kCheck(kXml_Child32s(xml, item, "Absolute", &obj->absolute));

    return kOK;
}

GoFx(kStatus) GoProfileDimHeight_VWrite(GoProfileDimHeight measurement, kXml xml, kXmlItem item)
{
    kObj(GoProfileDimHeight, measurement);

    kCheck(kXml_SetChild32s(xml, item, "Absolute", obj->absolute));
    kCheck(GoMeasurement_VWrite(measurement, xml, item));

    return kOK;
}

GoFx(kBool) GoProfileDimHeight_AbsoluteEnabled(GoProfileDimHeight measurement)
{
    kObj(GoProfileDimHeight, measurement);

    GoSensor_SyncConfig(GoMeasurement_Sensor(measurement));

    return obj->absolute;
}

GoFx(kStatus) GoProfileDimHeight_EnableAbsolute(GoProfileDimHeight measurement, kBool absolute)
{
    kObj(GoProfileDimHeight, measurement);

    kCheckState(GoSensor_IsConfigurable(GoMeasurement_Sensor(measurement)));
    kCheck(GoSensor_CacheConfig(GoMeasurement_Sensor(measurement)));
    obj->absolute = absolute;
    kCheck(GoSensor_SetConfigModified(GoMeasurement_Sensor(measurement)));

    return kOK;
}


kBeginClassEx(Go, GoProfileDimDistance)
    kAddVMethod(GoProfileDimDistance, GoMeasurement, VInit)
kEndClassEx()

GoFx(kStatus) GoProfileDimDistance_VInit(GoProfileDimDistance measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_PROFILE_DIMENSION_DISTANCE, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


kBeginClassEx(Go, GoProfileDimCenterX)
    kAddVMethod(GoProfileDimCenterX, GoMeasurement, VInit)
kEndClassEx()

GoFx(kStatus) GoProfileDimCenterX_VInit(GoProfileDimCenterX measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_PROFILE_DIMENSION_CENTER_X, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


kBeginClassEx(Go, GoProfileDimCenterZ)
    kAddVMethod(GoProfileDimCenterZ, GoMeasurement, VInit)
kEndClassEx()

GoFx(kStatus) GoProfileDimCenterZ_VInit(GoProfileDimCenterZ measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_PROFILE_DIMENSION_CENTER_Z, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


kBeginClassEx(Go, GoProfileGrooveX)
    kAddVMethod(GoProfileGrooveX, GoMeasurement, VInit)
    kAddVMethod(GoProfileGrooveX, GoMeasurement, VRead)
    kAddVMethod(GoProfileGrooveX, GoMeasurement, VWrite)
kEndClassEx()


GoFx(kStatus) GoProfileGrooveX_VInit(GoProfileGrooveX measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kObjR(GoProfileGrooveX, measurement);

    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_PROFILE_GROOVE_X, sensor, srcTool, kTRUE, alloc));

    obj->location = GO_PROFILE_GROOVE_LOCATION_BOTTOM;
    obj->selectType = GO_PROFILE_GROOVE_SELECT_TYPE_MAX_DEPTH;
    obj->selectIndex = 0;

    return kOK;
}

GoFx(kStatus) GoProfileGrooveX_VRead(GoProfileGrooveX measurement, kXml xml, kXmlItem item)
{
    kObj(GoProfileGrooveX, measurement);

    kCheck(GoMeasurement_VRead(measurement, xml, item));
    kCheck(kXml_Child32s(xml, item, "SelectType", &obj->selectType));
    kCheck(kXml_Child32u(xml, item, "SelectIndex", &obj->selectIndex));
    kCheck(kXml_Child32s(xml, item, "Location", &obj->location));

    return kOK;
}

GoFx(kStatus) GoProfileGrooveX_VWrite(GoProfileGrooveX measurement, kXml xml, kXmlItem item)
{
    kObj(GoProfileGrooveX, measurement);

    kCheck(kXml_SetChild32s(xml, item, "SelectType", obj->selectType));
    kCheck(kXml_SetChild32u(xml, item, "SelectIndex", obj->selectIndex));
    kCheck(kXml_SetChild32s(xml, item, "Location", obj->location));

    kCheck(GoMeasurement_VWrite(measurement, xml, item));

    return kOK;
}

GoFx(GoProfileGrooveLocation) GoProfileGrooveX_Location(GoProfileGrooveX measurement)
{
    kObj(GoProfileGrooveX, measurement);

    GoSensor_SyncConfig(GoMeasurement_Sensor(measurement));

    return obj->location;
}

GoFx(kStatus) GoProfileGrooveX_SetLocation(GoProfileGrooveX measurement, GoProfileGrooveLocation location)
{
    kObj(GoProfileGrooveX, measurement);

    kCheckState(GoSensor_IsConfigurable(GoMeasurement_Sensor(measurement)));
    kCheck(GoSensor_CacheConfig(GoMeasurement_Sensor(measurement)));
    obj->location = location;
    kCheck(GoSensor_SetConfigModified(GoMeasurement_Sensor(measurement)));

    return kOK;
}

GoFx(GoProfileGrooveSelectType) GoProfileGrooveX_SelectType(GoProfileGrooveX measurement)
{
    kObj(GoProfileGrooveX, measurement);

    GoSensor_SyncConfig(GoMeasurement_Sensor(measurement));

    return obj->selectType;
}

GoFx(kStatus) GoProfileGrooveX_SetSelectType(GoProfileGrooveX measurement, GoProfileGrooveSelectType selectType)
{
    kObj(GoProfileGrooveX, measurement);

    kCheckState(GoSensor_IsConfigurable(GoMeasurement_Sensor(measurement)));
    kCheck(GoSensor_CacheConfig(GoMeasurement_Sensor(measurement)));
    obj->selectType = selectType;
    kCheck(GoSensor_SetConfigModified(GoMeasurement_Sensor(measurement)));

    return kOK;
}

GoFx(k32u) GoProfileGrooveX_SelectIndex(GoProfileGrooveX measurement)
{
    kObj(GoProfileGrooveX, measurement);
    return obj->selectIndex;
}

GoFx(kStatus) GoProfileGrooveX_SetSelectIndex(GoProfileGrooveX measurement, k32u selectIndex)
{
    kObj(GoProfileGrooveX, measurement);

    kCheckState(GoSensor_IsConfigurable(GoMeasurement_Sensor(measurement)));
    kCheck(GoSensor_CacheConfig(GoMeasurement_Sensor(measurement)));
    obj->selectIndex = selectIndex;
    kCheck(GoSensor_SetConfigModified(GoMeasurement_Sensor(measurement)));

    return kOK;
}


kBeginClassEx(Go, GoProfileGrooveZ)
    kAddVMethod(GoProfileGrooveZ, GoMeasurement, VInit)
    kAddVMethod(GoProfileGrooveZ, GoMeasurement, VRead)
    kAddVMethod(GoProfileGrooveZ, GoMeasurement, VWrite)
kEndClassEx()


GoFx(kStatus) GoProfileGrooveZ_VInit(GoProfileGrooveZ measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kObjR(GoProfileGrooveZ, measurement);

    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_PROFILE_GROOVE_Z, sensor, srcTool, kTRUE, alloc));

    obj->location = GO_PROFILE_GROOVE_LOCATION_BOTTOM;
    obj->selectType = GO_PROFILE_GROOVE_SELECT_TYPE_MAX_DEPTH;
    obj->selectIndex = 0;

    return kOK;
}

GoFx(kStatus) GoProfileGrooveZ_VRead(GoProfileGrooveZ measurement, kXml xml, kXmlItem item)
{
    kObj(GoProfileGrooveZ, measurement);

    kCheck(GoMeasurement_VRead(measurement, xml, item));
    kCheck(kXml_Child32s(xml, item, "SelectType", &obj->selectType));
    kCheck(kXml_Child32u(xml, item, "SelectIndex", &obj->selectIndex));
    kCheck(kXml_Child32s(xml, item, "Location", &obj->location));

    return kOK;
}

GoFx(kStatus) GoProfileGrooveZ_VWrite(GoProfileGrooveZ measurement, kXml xml, kXmlItem item)
{
    kObj(GoProfileGrooveZ, measurement);

    kCheck(kXml_SetChild32s(xml, item, "SelectType", obj->selectType));
    kCheck(kXml_SetChild32u(xml, item, "SelectIndex", obj->selectIndex));
    kCheck(kXml_SetChild32s(xml, item, "Location", obj->location));
    kCheck(GoMeasurement_VWrite(measurement, xml, item));

    return kOK;
}

GoFx(GoProfileGrooveLocation) GoProfileGrooveZ_Location(GoProfileGrooveZ measurement)
{
    kObj(GoProfileGrooveZ, measurement);

    GoSensor_SyncConfig(GoMeasurement_Sensor(measurement));

    return obj->location;
}

GoFx(kStatus) GoProfileGrooveZ_SetLocation(GoProfileGrooveZ measurement, GoProfileGrooveLocation location)
{
    kObj(GoProfileGrooveZ, measurement);

    kCheckState(GoSensor_IsConfigurable(GoMeasurement_Sensor(measurement)));
    kCheck(GoSensor_CacheConfig(GoMeasurement_Sensor(measurement)));
    obj->location = location;
    kCheck(GoSensor_SetConfigModified(GoMeasurement_Sensor(measurement)));

    return kOK;
}


GoFx(GoProfileGrooveSelectType) GoProfileGrooveZ_SelectType(GoProfileGrooveZ measurement)
{
    kObj(GoProfileGrooveZ, measurement);
    GoSensor_SyncConfig(GoMeasurement_Sensor(measurement));

    return obj->selectType;
}

GoFx(kStatus) GoProfileGrooveZ_SetSelectType(GoProfileGrooveZ measurement, GoProfileGrooveSelectType selectType)
{
    kObj(GoProfileGrooveZ, measurement);

    kCheckState(GoSensor_IsConfigurable(GoMeasurement_Sensor(measurement)));
    kCheck(GoSensor_CacheConfig(GoMeasurement_Sensor(measurement)));
    obj->selectType = selectType;
    kCheck(GoSensor_SetConfigModified(GoMeasurement_Sensor(measurement)));

    return kOK;
}

GoFx(k32u) GoProfileGrooveZ_SelectIndex(GoProfileGrooveZ measurement)
{
    kObj(GoProfileGrooveZ, measurement);
    GoSensor_SyncConfig(GoMeasurement_Sensor(measurement));

    return obj->selectIndex;
}

GoFx(kStatus) GoProfileGrooveZ_SetSelectIndex(GoProfileGrooveZ measurement, k32u selectIndex)
{
    kObj(GoProfileGrooveZ, measurement);

    kCheckState(GoSensor_IsConfigurable(GoMeasurement_Sensor(measurement)));
    kCheck(GoSensor_CacheConfig(GoMeasurement_Sensor(measurement)));
    obj->selectIndex = selectIndex;
    kCheck(GoSensor_SetConfigModified(GoMeasurement_Sensor(measurement)));

    return kOK;
}


kBeginClassEx(Go, GoProfileGrooveWidth)
    kAddVMethod(GoProfileGrooveWidth, GoMeasurement, VInit)
    kAddVMethod(GoProfileGrooveWidth, GoMeasurement, VRead)
    kAddVMethod(GoProfileGrooveWidth, GoMeasurement, VWrite)
kEndClassEx()

GoFx(kStatus) GoProfileGrooveWidth_VInit(GoProfileGrooveWidth measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kObjR(GoProfileGrooveWidth, measurement);

    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_PROFILE_GROOVE_WIDTH, sensor, srcTool, kTRUE, alloc));

    obj->selectType = GO_PROFILE_GROOVE_SELECT_TYPE_MAX_DEPTH;
    obj->selectIndex = 0;

    return kOK;
}

GoFx(kStatus) GoProfileGrooveWidth_VRead(GoProfileGrooveWidth measurement, kXml xml, kXmlItem item)
{
    kObj(GoProfileGrooveWidth, measurement);

    kCheck(GoMeasurement_VRead(measurement, xml, item));
    kCheck(kXml_Child32s(xml, item, "SelectType", &obj->selectType));
    kCheck(kXml_Child32u(xml, item, "SelectIndex", &obj->selectIndex));

    return kOK;
}

GoFx(kStatus) GoProfileGrooveWidth_VWrite(GoProfileGrooveWidth measurement, kXml xml, kXmlItem item)
{
    kObj(GoProfileGrooveWidth, measurement);

    kCheck(kXml_SetChild32s(xml, item, "SelectType", obj->selectType));
    kCheck(kXml_SetChild32u(xml, item, "SelectIndex", obj->selectIndex));
    kCheck(GoMeasurement_VWrite(measurement, xml, item));

    return kOK;
}

GoFx(GoProfileGrooveSelectType) GoProfileGrooveWidth_SelectType(GoProfileGrooveWidth measurement)
{
    kObj(GoProfileGrooveWidth, measurement);
    GoSensor_SyncConfig(GoMeasurement_Sensor(measurement));

    return obj->selectType;
}

GoFx(kStatus) GoProfileGrooveWidth_SetSelectType(GoProfileGrooveWidth measurement, GoProfileGrooveSelectType selectType)
{
    kObj(GoProfileGrooveWidth, measurement);

    kCheckState(GoSensor_IsConfigurable(GoMeasurement_Sensor(measurement)));
    kCheck(GoSensor_CacheConfig(GoMeasurement_Sensor(measurement)));
    obj->selectType = selectType;
    kCheck(GoSensor_SetConfigModified(GoMeasurement_Sensor(measurement)));

    return kOK;
}

GoFx(k32u) GoProfileGrooveWidth_SelectIndex(GoProfileGrooveWidth measurement)
{
    kObj(GoProfileGrooveWidth, measurement);    return obj->selectIndex;
}

GoFx(kStatus) GoProfileGrooveWidth_SetSelectIndex(GoProfileGrooveWidth measurement, k32u selectIndex)
{
    kObj(GoProfileGrooveWidth, measurement);

    kCheckState(GoSensor_IsConfigurable(GoMeasurement_Sensor(measurement)));
    kCheck(GoSensor_CacheConfig(GoMeasurement_Sensor(measurement)));
    obj->selectIndex = selectIndex;
    kCheck(GoSensor_SetConfigModified(GoMeasurement_Sensor(measurement)));

    return kOK;
}


kBeginClassEx(Go, GoProfileGrooveDepth)
    kAddVMethod(GoProfileGrooveDepth, GoMeasurement, VInit)
    kAddVMethod(GoProfileGrooveDepth, GoMeasurement, VRead)
    kAddVMethod(GoProfileGrooveDepth, GoMeasurement, VWrite)
kEndClassEx()

GoFx(kStatus) GoProfileGrooveDepth_VInit(GoProfileGrooveDepth measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kObjR(GoProfileGrooveDepth, measurement);

    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_PROFILE_GROOVE_DEPTH, sensor, srcTool, kTRUE, alloc));

    obj->selectType = GO_PROFILE_GROOVE_SELECT_TYPE_MAX_DEPTH;
    obj->selectIndex = 0;

    return kOK;
}

GoFx(kStatus) GoProfileGrooveDepth_VRead(GoProfileGrooveDepth measurement, kXml xml, kXmlItem item)
{
    kObj(GoProfileGrooveDepth, measurement);

    kCheck(GoMeasurement_VRead(measurement, xml, item));
    kCheck(kXml_Child32s(xml, item, "SelectType", &obj->selectType));
    kCheck(kXml_Child32u(xml, item, "SelectIndex", &obj->selectIndex));

    return kOK;
}

GoFx(kStatus) GoProfileGrooveDepth_VWrite(GoProfileGrooveDepth measurement, kXml xml, kXmlItem item)
{
    kObj(GoProfileGrooveDepth, measurement);

    kCheck(kXml_SetChild32s(xml, item, "SelectType", obj->selectType));
    kCheck(kXml_SetChild32u(xml, item, "SelectIndex", obj->selectIndex));
    kCheck(GoMeasurement_VWrite(measurement, xml, item));

    return kOK;
}

GoFx(GoProfileGrooveSelectType) GoProfileGrooveDepth_SelectType(GoProfileGrooveDepth measurement)
{
    kObj(GoProfileGrooveDepth, measurement);

    GoSensor_SyncConfig(GoMeasurement_Sensor(measurement));

    return obj->selectType;
}

GoFx(kStatus) GoProfileGrooveDepth_SetSelectType(GoProfileGrooveDepth measurement, GoProfileGrooveSelectType selectType)
{
    kObj(GoProfileGrooveDepth, measurement);

    kCheckState(GoSensor_IsConfigurable(GoMeasurement_Sensor(measurement)));
    kCheck(GoSensor_CacheConfig(GoMeasurement_Sensor(measurement)));
    obj->selectType = selectType;
    kCheck(GoSensor_SetConfigModified(GoMeasurement_Sensor(measurement)));

    return kOK;
}

GoFx(k32u) GoProfileGrooveDepth_SelectIndex(GoProfileGrooveDepth measurement)
{
    kObj(GoProfileGrooveDepth, measurement);

    GoSensor_SyncConfig(GoMeasurement_Sensor(measurement));

    return obj->selectIndex;
}

GoFx(kStatus) GoProfileGrooveDepth_SetSelectIndex(GoProfileGrooveDepth measurement, k32u selectIndex)
{
    kObj(GoProfileGrooveDepth, measurement);

    kCheckState(GoSensor_IsConfigurable(GoMeasurement_Sensor(measurement)));
    kCheck(GoSensor_CacheConfig(GoMeasurement_Sensor(measurement)));
    obj->selectIndex = selectIndex;
    kCheck(GoSensor_SetConfigModified(GoMeasurement_Sensor(measurement)));

    return kOK;
}


kBeginClassEx(Go, GoProfileIntersectX)
    kAddVMethod(GoProfileIntersectX, GoMeasurement, VInit)
kEndClassEx()

GoFx(kStatus) GoProfileIntersectX_VInit(GoProfileIntersectX measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_PROFILE_INTERSECT_X, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


kBeginClassEx(Go, GoProfileIntersectZ)
    kAddVMethod(GoProfileIntersectZ, GoMeasurement, VInit)
kEndClassEx()

GoFx(kStatus) GoProfileIntersectZ_VInit(GoProfileIntersectZ measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_PROFILE_INTERSECT_Z, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


kBeginClassEx(Go, GoProfileIntersectAngle)
    kAddVMethod(GoProfileIntersectAngle, GoMeasurement, VInit)
    kAddVMethod(GoProfileIntersectAngle, GoMeasurement, VRead)
    kAddVMethod(GoProfileIntersectAngle, GoMeasurement, VWrite)
kEndClassEx()

GoFx(kStatus) GoProfileIntersectAngle_VInit(GoProfileIntersectAngle measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kObjR(GoProfileIntersectAngle, measurement);

    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_PROFILE_INTERSECT_ANGLE, sensor, srcTool, kTRUE, alloc));

    obj->range0to180Enabled = kFALSE;

    return kOK;
}

GoFx(kStatus) GoProfileIntersectAngle_VRead(GoProfileIntersectAngle measurement, kXml xml, kXmlItem item)
{
    kObj(GoProfileIntersectAngle, measurement);

    kCheck(GoMeasurement_VRead(measurement, xml, item));
    kCheck(kXml_Child32s(xml, item, "Absolute", &obj->range0to180Enabled));

    return kOK;
}

GoFx(kStatus) GoProfileIntersectAngle_VWrite(GoProfileIntersectAngle measurement, kXml xml, kXmlItem item)
{
    kObj(GoProfileIntersectAngle, measurement);

    kCheck(kXml_SetChild32s(xml, item, "Absolute", obj->range0to180Enabled));
    kCheck(GoMeasurement_VWrite(measurement, xml, item));

    return kOK;
}

GoFx(kBool) GoProfileIntersectAngle_AbsoluteEnabled(GoProfileIntersectAngle measurement)
{
    return GoProfileIntersectAngle_Range0to180Enabled(measurement);
}

GoFx(kStatus) GoProfileIntersectAngle_EnableAbsolute(GoProfileIntersectAngle measurement, kBool absolute)
{
    kCheck(GoProfileIntersectAngle_EnableRange0to180(measurement, absolute));

    return kOK;
}

GoFx(kBool) GoProfileIntersectAngle_Range0to180Enabled(GoProfileIntersectAngle measurement)
{
    kObj(GoProfileIntersectAngle, measurement);

    GoSensor_SyncConfig(GoMeasurement_Sensor(measurement));

    return obj->range0to180Enabled;
}

GoFx(kStatus) GoProfileIntersectAngle_EnableRange0to180(GoProfileIntersectAngle measurement, kBool enable)
{
    kObj(GoProfileIntersectAngle, measurement);

    kCheckState(GoSensor_IsConfigurable(GoMeasurement_Sensor(measurement)));
    kCheck(GoSensor_CacheConfig(GoMeasurement_Sensor(measurement)));
    obj->range0to180Enabled = enable;
    kCheck(GoSensor_SetConfigModified(GoMeasurement_Sensor(measurement)));

    return kOK;
}

kBeginClassEx(Go, GoProfileLineStdDev)
    kAddVMethod(GoProfileLineStdDev, GoMeasurement, VInit)
kEndClassEx()

GoFx(kStatus) GoProfileLineStdDev_VInit(GoProfileLineStdDev measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_PROFILE_LINE_STDDEV, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


kBeginClassEx(Go, GoProfileLineMinError)
    kAddVMethod(GoProfileLineMinError, GoMeasurement, VInit)
kEndClassEx()

GoFx(kStatus) GoProfileLineMinError_VInit(GoProfileLineMinError measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_PROFILE_LINE_ERROR_MIN, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


kBeginClassEx(Go, GoProfileLineMaxError)
    kAddVMethod(GoProfileLineMaxError, GoMeasurement, VInit)
kEndClassEx()

GoFx(kStatus) GoProfileLineMaxError_VInit(GoProfileLineMaxError measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_PROFILE_LINE_ERROR_MAX, sensor, srcTool, kTRUE, alloc));

    return kOK;
}

kBeginClassEx(Go, GoProfileLineOffset)
kAddVMethod(GoProfileLineOffset, GoMeasurement, VInit)
kEndClassEx()

GoFx(kStatus) GoProfileLineOffset_VInit(GoProfileLineOffset measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_PROFILE_LINE_OFFSET     , sensor, srcTool, kTRUE, alloc));

    return kOK;
}
kBeginClassEx(Go, GoProfileLineAngle)
kAddVMethod(GoProfileLineAngle, GoMeasurement, VInit)
kEndClassEx()

GoFx(kStatus) GoProfileLineAngle_VInit(GoProfileLineAngle measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_PROFILE_LINE_ANGLE      , sensor, srcTool, kTRUE, alloc));

    return kOK;
}

kBeginClassEx(Go, GoProfileLineMinErrorX)
kAddVMethod(GoProfileLineMinErrorX, GoMeasurement, VInit)
kEndClassEx()

GoFx(kStatus) GoProfileLineMinErrorX_VInit(GoProfileLineMinErrorX measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_PROFILE_LINE_ERROR_MIN_X, sensor, srcTool, kTRUE, alloc));

    return kOK;
}

kBeginClassEx(Go, GoProfileLineMinErrorZ)
kAddVMethod(GoProfileLineMinErrorZ, GoMeasurement, VInit)
kEndClassEx()

GoFx(kStatus) GoProfileLineMinErrorZ_VInit(GoProfileLineMinErrorZ measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_PROFILE_LINE_ERROR_MIN_Z, sensor, srcTool, kTRUE, alloc));

    return kOK;
}

kBeginClassEx(Go, GoProfileLineMaxErrorX)
kAddVMethod(GoProfileLineMaxErrorX, GoMeasurement, VInit)
kEndClassEx()

GoFx(kStatus) GoProfileLineMaxErrorX_VInit(GoProfileLineMaxErrorX measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_PROFILE_LINE_ERROR_MAX_X, sensor, srcTool, kTRUE, alloc));

    return kOK;
}

kBeginClassEx(Go, GoProfileLineMaxErrorZ)
kAddVMethod(GoProfileLineMaxErrorZ, GoMeasurement, VInit)
kEndClassEx()

GoFx(kStatus) GoProfileLineMaxErrorZ_VInit(GoProfileLineMaxErrorZ measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_PROFILE_LINE_ERROR_MAX_Z, sensor, srcTool, kTRUE, alloc));

    return kOK;
}

kBeginClassEx(Go, GoProfileLinePercentile)
    kAddVMethod(GoProfileLinePercentile, GoMeasurement, VInit)
    kAddVMethod(GoProfileLinePercentile, GoMeasurement, VRead)
    kAddVMethod(GoProfileLinePercentile, GoMeasurement, VWrite)
kEndClassEx()

GoFx(kStatus) GoProfileLinePercentile_VInit(GoProfileLinePercentile measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kObjR(GoProfileLinePercentile, measurement);

    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_PROFILE_LINE_PERCENTILE, sensor, srcTool, kTRUE, alloc));

    obj->percent = 95.0;

    return kOK;
}

GoFx(kStatus) GoProfileLinePercentile_VRead(GoProfileLinePercentile measurement, kXml xml, kXmlItem item)
{
    kObj(GoProfileLinePercentile, measurement);

    kCheck(GoMeasurement_VRead(measurement, xml, item));
    kCheck(kXml_Child64f(xml, item, "Percent", &obj->percent));

    return kOK;
}

GoFx(kStatus) GoProfileLinePercentile_VWrite(GoProfileLinePercentile measurement, kXml xml, kXmlItem item)
{
    kObj(GoProfileLinePercentile, measurement);

    kCheck(kXml_SetChild64f(xml, item, "Percent", obj->percent));
    kCheck(GoMeasurement_VWrite(measurement, xml, item));

    return kOK;
}

GoFx(k64f) GoProfileLinePercentile_Percent(GoProfileLinePercentile measurement)
{
    kObj(GoProfileLinePercentile, measurement);

    GoSensor_SyncConfig(GoMeasurement_Sensor(measurement));

    return obj->percent;
}

GoFx(kStatus) GoProfileLinePercentile_SetPercent(GoProfileLinePercentile measurement, k64f percent)
{
    kObj(GoProfileLinePercentile, measurement);

    kCheckState(GoSensor_IsConfigurable(GoMeasurement_Sensor(measurement)));
    kCheck(GoSensor_CacheConfig(GoMeasurement_Sensor(measurement)));
    obj->percent = percent;
    kCheck(GoSensor_SetConfigModified(GoMeasurement_Sensor(measurement)));

    return kOK;
}


kBeginClassEx(Go, GoProfilePanelGap)
    kAddVMethod(GoProfilePanelGap, GoMeasurement, VInit)
    kAddVMethod(GoProfilePanelGap, GoMeasurement, VRead)
    kAddVMethod(GoProfilePanelGap, GoMeasurement, VWrite)
kEndClassEx()

GoFx(kStatus) GoProfilePanelGap_VInit(GoProfilePanelGap measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kObjR(GoProfilePanelGap, measurement);

    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_PROFILE_PANEL_GAP, sensor, srcTool, kTRUE, alloc));

    obj->axis = GO_PROFILE_GAP_AXIS_EDGE;

    return kOK;
}

GoFx(kStatus) GoProfilePanelGap_VRead(GoProfilePanelGap measurement, kXml xml, kXmlItem item)
{
    kObj(GoProfilePanelGap, measurement);

    kCheck(GoMeasurement_VRead(measurement, xml, item));
    kCheck(kXml_Child32s(xml, item, "Axis", &obj->axis));

    return kOK;
}

GoFx(kStatus) GoProfilePanelGap_VWrite(GoProfilePanelGap measurement, kXml xml, kXmlItem item)
{
    kObj(GoProfilePanelGap, measurement);

    kCheck(kXml_SetChild32s(xml, item, "Axis", obj->axis));
    kCheck(GoMeasurement_VWrite(measurement, xml, item));

    return kOK;
}

GoFx(GoProfileGapAxis) GoProfilePanelGap_Axis(GoProfilePanelGap measurement)
{
    kObj(GoProfilePanelGap, measurement);

    GoSensor_SyncConfig(GoMeasurement_Sensor(measurement));

    return obj->axis;
}

GoFx(kStatus) GoProfilePanelGap_SetAxis(GoProfilePanelGap measurement, GoProfileGapAxis axis)
{
    kObj(GoProfilePanelGap, measurement);

    kCheckState(GoSensor_IsConfigurable(GoMeasurement_Sensor(measurement)));
    kCheck(GoSensor_CacheConfig(GoMeasurement_Sensor(measurement)));
    obj->axis = axis;
    kCheck(GoSensor_SetConfigModified(GoMeasurement_Sensor(measurement)));

    return kOK;
}


kBeginClassEx(Go, GoProfilePanelFlush)
    kAddVMethod(GoProfilePanelFlush, GoMeasurement, VInit)
    kAddVMethod(GoProfilePanelFlush, GoMeasurement, VRead)
    kAddVMethod(GoProfilePanelFlush, GoMeasurement, VWrite)
kEndClassEx()

GoFx(kStatus) GoProfilePanelFlush_VInit(GoProfilePanelFlush measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kObjR(GoProfilePanelFlush, measurement);

    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_PROFILE_PANEL_FLUSH, sensor, srcTool, kTRUE, alloc));

    obj->absolute = kFALSE;

    return kOK;
}

GoFx(kStatus) GoProfilePanelFlush_VRead(GoProfilePanelFlush measurement, kXml xml, kXmlItem item)
{
    kObj(GoProfilePanelFlush, measurement);
    kCheck(GoMeasurement_VRead(measurement, xml, item));
    kCheck(kXml_Child32s(xml, item, "Absolute", &obj->absolute));

    return kOK;
}

GoFx(kStatus) GoProfilePanelFlush_VWrite(GoProfilePanelFlush measurement, kXml xml, kXmlItem item)
{
    kObj(GoProfilePanelFlush, measurement);
    kCheck(kXml_SetChild32s(xml, item, "Absolute", obj->absolute));
    kCheck(GoMeasurement_VWrite(measurement, xml, item));

    return kOK;
}

GoFx(kBool) GoProfilePanelFlush_AbsoluteEnabled(GoProfilePanelFlush measurement)
{
    kObj(GoProfilePanelFlush, measurement);
    GoSensor_SyncConfig(GoMeasurement_Sensor(measurement));

    return obj->absolute;
}

GoFx(kStatus) GoProfilePanelFlush_EnableAbsolute(GoProfilePanelFlush measurement, kBool absolute)
{
    kObj(GoProfilePanelFlush, measurement);
    kCheckState(GoSensor_IsConfigurable(GoMeasurement_Sensor(measurement)));
    kCheck(GoSensor_CacheConfig(GoMeasurement_Sensor(measurement)));
    obj->absolute = absolute;
    kCheck(GoSensor_SetConfigModified(GoMeasurement_Sensor(measurement)));

    return kOK;
}

kBeginClassEx(Go, GoProfilePanelLeftFlushX)
kAddVMethod(GoProfilePanelLeftFlushX, GoMeasurement, VInit)
kAddVMethod(GoProfilePanelLeftFlushX, GoMeasurement, VRead)
kAddVMethod(GoProfilePanelLeftFlushX, GoMeasurement, VWrite)
kEndClassEx()

GoFx(kStatus) GoProfilePanelLeftFlushX_VInit(GoProfilePanelLeftFlushX measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_PROFILE_PANEL_LEFT_FLUSH_X, sensor, srcTool, kTRUE, alloc));
    return kOK;
}

GoFx(kStatus) GoProfilePanelLeftFlushX_VRead(GoProfilePanelLeftFlushX measurement, kXml xml, kXmlItem item)
{
    kObj(GoProfilePanelLeftFlushX, measurement);
    kCheck(GoMeasurement_VRead(measurement, xml, item));
    return kOK;
}

GoFx(kStatus) GoProfilePanelLeftFlushX_VWrite(GoProfilePanelLeftFlushX measurement, kXml xml, kXmlItem item)
{
    kObj(GoProfilePanelLeftFlushX, measurement);
    kCheck(GoMeasurement_VWrite(measurement, xml, item));

    return kOK;
}

kBeginClassEx(Go, GoProfilePanelLeftFlushZ)
kAddVMethod(GoProfilePanelLeftFlushZ, GoMeasurement, VInit)
kAddVMethod(GoProfilePanelLeftFlushZ, GoMeasurement, VRead)
kAddVMethod(GoProfilePanelLeftFlushZ, GoMeasurement, VWrite)
kEndClassEx()

GoFx(kStatus) GoProfilePanelLeftFlushZ_VInit(GoProfilePanelLeftFlushZ measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_PROFILE_PANEL_LEFT_FLUSH_Z, sensor, srcTool, kTRUE, alloc));
    return kOK;
}

GoFx(kStatus) GoProfilePanelLeftFlushZ_VRead(GoProfilePanelLeftFlushZ measurement, kXml xml, kXmlItem item)
{
    kObj(GoProfilePanelLeftFlushZ, measurement);
    kCheck(GoMeasurement_VRead(measurement, xml, item));
    return kOK;
}

GoFx(kStatus) GoProfilePanelLeftFlushZ_VWrite(GoProfilePanelLeftFlushZ measurement, kXml xml, kXmlItem item)
{
    kObj(GoProfilePanelLeftFlushZ, measurement);
    kCheck(GoMeasurement_VWrite(measurement, xml, item));

    return kOK;
}

kBeginClassEx(Go, GoProfilePanelLeftGapX)
kAddVMethod(GoProfilePanelLeftGapX, GoMeasurement, VInit)
kAddVMethod(GoProfilePanelLeftGapX, GoMeasurement, VRead)
kAddVMethod(GoProfilePanelLeftGapX, GoMeasurement, VWrite)
kEndClassEx()

GoFx(kStatus) GoProfilePanelLeftGapX_VInit(GoProfilePanelLeftGapX measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_PROFILE_PANEL_LEFT_GAP_X, sensor, srcTool, kTRUE, alloc));
    return kOK;
}

GoFx(kStatus) GoProfilePanelLeftGapX_VRead(GoProfilePanelLeftGapX measurement, kXml xml, kXmlItem item)
{
    kObj(GoProfilePanelLeftGapX, measurement);
    kCheck(GoMeasurement_VRead(measurement, xml, item));
    return kOK;
}

GoFx(kStatus) GoProfilePanelLeftGapX_VWrite(GoProfilePanelLeftGapX measurement, kXml xml, kXmlItem item)
{
    kObj(GoProfilePanelLeftGapX, measurement);
    kCheck(GoMeasurement_VWrite(measurement, xml, item));

    return kOK;
}

kBeginClassEx(Go, GoProfilePanelLeftGapZ)
kAddVMethod(GoProfilePanelLeftGapZ, GoMeasurement, VInit)
kAddVMethod(GoProfilePanelLeftGapZ, GoMeasurement, VRead)
kAddVMethod(GoProfilePanelLeftGapZ, GoMeasurement, VWrite)
kEndClassEx()

GoFx(kStatus) GoProfilePanelLeftGapZ_VInit(GoProfilePanelLeftGapZ measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_PROFILE_PANEL_LEFT_GAP_Z, sensor, srcTool, kTRUE, alloc));
    return kOK;
}

GoFx(kStatus) GoProfilePanelLeftGapZ_VRead(GoProfilePanelLeftGapZ measurement, kXml xml, kXmlItem item)
{
    kObj(GoProfilePanelLeftGapZ, measurement);
    kCheck(GoMeasurement_VRead(measurement, xml, item));
    return kOK;
}

GoFx(kStatus) GoProfilePanelLeftGapZ_VWrite(GoProfilePanelLeftGapZ measurement, kXml xml, kXmlItem item)
{
    kObj(GoProfilePanelLeftGapZ, measurement);
    kCheck(GoMeasurement_VWrite(measurement, xml, item));

    return kOK;
}

kBeginClassEx(Go, GoProfilePanelLeftSurfaceAngle)
kAddVMethod(GoProfilePanelLeftSurfaceAngle, GoMeasurement, VInit)
kAddVMethod(GoProfilePanelLeftSurfaceAngle, GoMeasurement, VRead)
kAddVMethod(GoProfilePanelLeftSurfaceAngle, GoMeasurement, VWrite)
kEndClassEx()

GoFx(kStatus) GoProfilePanelLeftSurfaceAngle_VInit(GoProfilePanelLeftSurfaceAngle measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_PROFILE_PANEL_LEFT_SURFACE_ANGLE, sensor, srcTool, kTRUE, alloc));
    return kOK;
}

GoFx(kStatus) GoProfilePanelLeftSurfaceAngle_VRead(GoProfilePanelLeftSurfaceAngle measurement, kXml xml, kXmlItem item)
{
    kObj(GoProfilePanelLeftSurfaceAngle, measurement);
    kCheck(GoMeasurement_VRead(measurement, xml, item));
    return kOK;
}

GoFx(kStatus) GoProfilePanelLeftSurfaceAngle_VWrite(GoProfilePanelLeftSurfaceAngle measurement, kXml xml, kXmlItem item)
{
    kObj(GoProfilePanelLeftSurfaceAngle, measurement);
    kCheck(GoMeasurement_VWrite(measurement, xml, item));

    return kOK;
}

kBeginClassEx(Go, GoProfilePanelRightFlushX)
kAddVMethod(GoProfilePanelRightFlushX, GoMeasurement, VInit)
kAddVMethod(GoProfilePanelRightFlushX, GoMeasurement, VRead)
kAddVMethod(GoProfilePanelRightFlushX, GoMeasurement, VWrite)
kEndClassEx()

GoFx(kStatus) GoProfilePanelRightFlushX_VInit(GoProfilePanelRightFlushX measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_PROFILE_PANEL_RIGHT_FLUSH_X, sensor, srcTool, kTRUE, alloc));
    return kOK;
}

GoFx(kStatus) GoProfilePanelRightFlushX_VRead(GoProfilePanelRightFlushX measurement, kXml xml, kXmlItem item)
{
    kObj(GoProfilePanelRightFlushX, measurement);
    kCheck(GoMeasurement_VRead(measurement, xml, item));
    return kOK;
}

GoFx(kStatus) GoProfilePanelRightFlushX_VWrite(GoProfilePanelRightFlushX measurement, kXml xml, kXmlItem item)
{
    kObj(GoProfilePanelRightFlushX, measurement);
    kCheck(GoMeasurement_VWrite(measurement, xml, item));

    return kOK;
}

kBeginClassEx(Go, GoProfilePanelRightFlushZ)
kAddVMethod(GoProfilePanelRightFlushZ, GoMeasurement, VInit)
kAddVMethod(GoProfilePanelRightFlushZ, GoMeasurement, VRead)
kAddVMethod(GoProfilePanelRightFlushZ, GoMeasurement, VWrite)
kEndClassEx()

GoFx(kStatus) GoProfilePanelRightFlushZ_VInit(GoProfilePanelRightFlushZ measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_PROFILE_PANEL_RIGHT_FLUSH_Z, sensor, srcTool, kTRUE, alloc));
    return kOK;
}

GoFx(kStatus) GoProfilePanelRightFlushZ_VRead(GoProfilePanelRightFlushZ measurement, kXml xml, kXmlItem item)
{
    kObj(GoProfilePanelRightFlushZ, measurement);
    kCheck(GoMeasurement_VRead(measurement, xml, item));
    return kOK;
}

GoFx(kStatus) GoProfilePanelRightFlushZ_VWrite(GoProfilePanelRightFlushZ measurement, kXml xml, kXmlItem item)
{
    kObj(GoProfilePanelRightFlushZ, measurement);
    kCheck(GoMeasurement_VWrite(measurement, xml, item));

    return kOK;
}

kBeginClassEx(Go, GoProfilePanelRightGapX)
kAddVMethod(GoProfilePanelRightGapX, GoMeasurement, VInit)
kAddVMethod(GoProfilePanelRightGapX, GoMeasurement, VRead)
kAddVMethod(GoProfilePanelRightGapX, GoMeasurement, VWrite)
kEndClassEx()

GoFx(kStatus) GoProfilePanelRightGapX_VInit(GoProfilePanelRightGapX measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_PROFILE_PANEL_RIGHT_GAP_X, sensor, srcTool, kTRUE, alloc));
    return kOK;
}

GoFx(kStatus) GoProfilePanelRightGapX_VRead(GoProfilePanelRightGapX measurement, kXml xml, kXmlItem item)
{
    kObj(GoProfilePanelRightGapX, measurement);
    kCheck(GoMeasurement_VRead(measurement, xml, item));
    return kOK;
}

GoFx(kStatus) GoProfilePanelRightGapX_VWrite(GoProfilePanelRightGapX measurement, kXml xml, kXmlItem item)
{
    kObj(GoProfilePanelRightGapX, measurement);
    kCheck(GoMeasurement_VWrite(measurement, xml, item));

    return kOK;
}

kBeginClassEx(Go, GoProfilePanelRightGapZ)
kAddVMethod(GoProfilePanelRightGapZ, GoMeasurement, VInit)
kAddVMethod(GoProfilePanelRightGapZ, GoMeasurement, VRead)
kAddVMethod(GoProfilePanelRightGapZ, GoMeasurement, VWrite)
kEndClassEx()

GoFx(kStatus) GoProfilePanelRightGapZ_VInit(GoProfilePanelRightGapZ measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_PROFILE_PANEL_RIGHT_GAP_Z, sensor, srcTool, kTRUE, alloc));
    return kOK;
}

GoFx(kStatus) GoProfilePanelRightGapZ_VRead(GoProfilePanelRightGapZ measurement, kXml xml, kXmlItem item)
{
    kObj(GoProfilePanelRightGapZ, measurement);
    kCheck(GoMeasurement_VRead(measurement, xml, item));
    return kOK;
}

GoFx(kStatus) GoProfilePanelRightGapZ_VWrite(GoProfilePanelRightGapZ measurement, kXml xml, kXmlItem item)
{
    kObj(GoProfilePanelRightGapZ, measurement);
    kCheck(GoMeasurement_VWrite(measurement, xml, item));

    return kOK;
}

kBeginClassEx(Go, GoProfilePanelRightSurfaceAngle)
kAddVMethod(GoProfilePanelRightSurfaceAngle, GoMeasurement, VInit)
kAddVMethod(GoProfilePanelRightSurfaceAngle, GoMeasurement, VRead)
kAddVMethod(GoProfilePanelRightSurfaceAngle, GoMeasurement, VWrite)
kEndClassEx()

GoFx(kStatus) GoProfilePanelRightSurfaceAngle_VInit(GoProfilePanelRightSurfaceAngle measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_PROFILE_PANEL_RIGHT_SURFACE_ANGLE, sensor, srcTool, kTRUE, alloc));
    return kOK;
}

GoFx(kStatus) GoProfilePanelRightSurfaceAngle_VRead(GoProfilePanelRightSurfaceAngle measurement, kXml xml, kXmlItem item)
{
    kObj(GoProfilePanelRightSurfaceAngle, measurement);
    kCheck(GoMeasurement_VRead(measurement, xml, item));
    return kOK;
}

GoFx(kStatus) GoProfilePanelRightSurfaceAngle_VWrite(GoProfilePanelRightSurfaceAngle measurement, kXml xml, kXmlItem item)
{
    kObj(GoProfilePanelRightSurfaceAngle, measurement);
    kCheck(GoMeasurement_VWrite(measurement, xml, item));

    return kOK;
}

kBeginClassEx(Go, GoProfileRoundCornerX)
    kAddVMethod(GoProfileRoundCornerX, GoMeasurement, VInit)
kEndClassEx()

GoFx(kStatus) GoProfileRoundCornerX_VInit(GoProfileRoundCornerX measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_PROFILE_ROUND_CORNER_X, sensor, srcTool, kTRUE, alloc));

    return kOK;
}

kBeginClassEx(Go, GoProfileRoundCornerZ)
    kAddVMethod(GoProfileRoundCornerZ, GoMeasurement, VInit)
kEndClassEx()

GoFx(kStatus) GoProfileRoundCornerZ_VInit(GoProfileRoundCornerZ measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_PROFILE_ROUND_CORNER_Z, sensor, srcTool, kTRUE, alloc));

    return kOK;
}

kBeginClassEx(Go, GoProfileRoundCornerAngle)
    kAddVMethod(GoProfileRoundCornerAngle, GoMeasurement, VInit)
kEndClassEx()

GoFx(kStatus) GoProfileRoundCornerAngle_VInit(GoProfileRoundCornerAngle measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_PROFILE_ROUND_CORNER_ANGLE, sensor, srcTool, kTRUE, alloc));

    return kOK;
}

kBeginClassEx(Go, GoProfilePositionX)
    kAddVMethod(GoProfilePositionX, GoMeasurement, VInit)
kEndClassEx()

GoFx(kStatus) GoProfilePositionX_VInit(GoProfilePositionX measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_PROFILE_POSITION_X, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


kBeginClassEx(Go, GoProfilePositionZ)
    kAddVMethod(GoProfilePositionZ, GoMeasurement, VInit)
kEndClassEx()

GoFx(kStatus) GoProfilePositionZ_VInit(GoProfilePositionZ measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_PROFILE_POSITION_Z, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


kBeginClassEx(Go, GoProfileStripX)
    kAddVMethod(GoProfileStripX, GoMeasurement, VInit)
    kAddVMethod(GoProfileStripX, GoMeasurement, VRead)
    kAddVMethod(GoProfileStripX, GoMeasurement, VWrite)
kEndClassEx()

GoFx(kStatus) GoProfileStripX_VInit(GoProfileStripX measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kObjR(GoProfileStripX, measurement);

    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_PROFILE_STRIP_POSITION_X, sensor, srcTool, kTRUE, alloc));

    obj->location = GO_PROFILE_STRIP_LOCATION_LEFT;
    obj->selectType = GO_PROFILE_STRIP_SELECT_TYPE_BEST;
    obj->selectIndex = 0;

    return kOK;
}

GoFx(kStatus) GoProfileStripX_VRead(GoProfileStripX measurement, kXml xml, kXmlItem item)
{
    kObj(GoProfileStripX, measurement);
    kCheck(GoMeasurement_VRead(measurement, xml, item));
    kCheck(kXml_Child32s(xml, item, "Location", &obj->location));
    kCheck(kXml_Child32s(xml, item, "SelectType", &obj->selectType));
    kCheck(kXml_Child32u(xml, item, "SelectIndex", &obj->selectIndex));

    return kOK;
}

GoFx(kStatus) GoProfileStripX_VWrite(GoProfileStripX measurement, kXml xml, kXmlItem item)
{
    kObj(GoProfileStripX, measurement);
    kCheck(kXml_SetChild32s(xml, item, "SelectType", obj->selectType));
    kCheck(kXml_SetChild32u(xml, item, "SelectIndex", obj->selectIndex));
    kCheck(kXml_SetChild32s(xml, item, "Location", obj->location));
    kCheck(GoMeasurement_VWrite(measurement, xml, item));

    return kOK;
}

GoFx(GoProfileGrooveLocation) GoProfileStripX_Location(GoProfileStripX measurement)
{
    kObj(GoProfileStripX, measurement);
    GoSensor_SyncConfig(GoMeasurement_Sensor(measurement));

    return obj->location;
}

GoFx(kStatus) GoProfileStripX_SetLocation(GoProfileStripX measurement, GoProfileGrooveLocation location)
{
    kObj(GoProfileStripX, measurement);
    kCheckState(GoSensor_IsConfigurable(GoMeasurement_Sensor(measurement)));
    kCheck(GoSensor_CacheConfig(GoMeasurement_Sensor(measurement)));
    obj->location = location;
    kCheck(GoSensor_SetConfigModified(GoMeasurement_Sensor(measurement)));

    return kOK;
}

GoFx(GoProfileGrooveSelectType) GoProfileStripX_SelectType(GoProfileStripX measurement)
{
    kObj(GoProfileStripX, measurement);
    GoSensor_SyncConfig(GoMeasurement_Sensor(measurement));

    return obj->selectType;
}

GoFx(kStatus) GoProfileStripX_SetSelectType(GoProfileStripX measurement, GoProfileGrooveSelectType selectType)
{
    kObj(GoProfileStripX, measurement);
    kCheckState(GoSensor_IsConfigurable(GoMeasurement_Sensor(measurement)));
    kCheck(GoSensor_CacheConfig(GoMeasurement_Sensor(measurement)));
    obj->selectType = selectType;
    kCheck(GoSensor_SetConfigModified(GoMeasurement_Sensor(measurement)));

    return kOK;
}

GoFx(k32u) GoProfileStripX_SelectIndex(GoProfileStripX measurement)
{
    kObj(GoProfileStripX, measurement);
    GoSensor_SyncConfig(GoMeasurement_Sensor(measurement));

    return obj->selectIndex;
}

GoFx(kStatus) GoProfileStripX_SetSelectIndex(GoProfileStripX measurement, k32u selectIndex)
{
    kObj(GoProfileStripX, measurement);
    kCheckState(GoSensor_IsConfigurable(GoMeasurement_Sensor(measurement)));
    kCheck(GoSensor_CacheConfig(GoMeasurement_Sensor(measurement)));
    obj->selectIndex = selectIndex;
    kCheck(GoSensor_SetConfigModified(GoMeasurement_Sensor(measurement)));

    return kOK;
}


kBeginClassEx(Go, GoProfileStripZ)
    kAddVMethod(GoProfileStripZ, GoMeasurement, VInit)
    kAddVMethod(GoProfileStripZ, GoMeasurement, VRead)
    kAddVMethod(GoProfileStripZ, GoMeasurement, VWrite)
kEndClassEx()

GoFx(kStatus) GoProfileStripZ_VInit(GoProfileStripZ measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kObjR(GoProfileStripZ, measurement);

    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_PROFILE_STRIP_POSITION_Z, sensor, srcTool, kTRUE, alloc));

    obj->location = GO_PROFILE_STRIP_LOCATION_LEFT;
    obj->selectType = GO_PROFILE_STRIP_SELECT_TYPE_BEST;
    obj->selectIndex = 0;

    return kOK;
}

GoFx(kStatus) GoProfileStripZ_VRead(GoProfileStripZ measurement, kXml xml, kXmlItem item)
{
    kObj(GoProfileStripZ, measurement);
    kCheck(GoMeasurement_VRead(measurement, xml, item));
    kCheck(kXml_Child32s(xml, item, "Location", &obj->location));
    kCheck(kXml_Child32s(xml, item, "SelectType", &obj->selectType));
    kCheck(kXml_Child32u(xml, item, "SelectIndex", &obj->selectIndex));

    return kOK;
}

GoFx(kStatus) GoProfileStripZ_VWrite(GoProfileStripZ measurement, kXml xml, kXmlItem item)
{
    kObj(GoProfileStripZ, measurement);
    kCheck(kXml_SetChild32s(xml, item, "SelectType", obj->selectType));
    kCheck(kXml_SetChild32u(xml, item, "SelectIndex", obj->selectIndex));
    kCheck(kXml_SetChild32s(xml, item, "Location", obj->location));
    kCheck(GoMeasurement_VWrite(measurement, xml, item));

    return kOK;
}

GoFx(GoProfileGrooveLocation) GoProfileStripZ_Location(GoProfileStripZ measurement)
{
    kObj(GoProfileStripZ, measurement);
    GoSensor_SyncConfig(GoMeasurement_Sensor(measurement));

    return obj->location;
}

GoFx(kStatus) GoProfileStripZ_SetLocation(GoProfileStripZ measurement, GoProfileGrooveLocation location)
{
    kObj(GoProfileStripZ, measurement);
    kCheckState(GoSensor_IsConfigurable(GoMeasurement_Sensor(measurement)));
    kCheck(GoSensor_CacheConfig(GoMeasurement_Sensor(measurement)));
    obj->location = location;
    kCheck(GoSensor_SetConfigModified(GoMeasurement_Sensor(measurement)));

    return kOK;
}

GoFx(GoProfileGrooveSelectType) GoProfileStripZ_SelectType(GoProfileStripZ measurement)
{
    kObj(GoProfileStripZ, measurement);
    GoSensor_SyncConfig(GoMeasurement_Sensor(measurement));

    return obj->selectType;
}

GoFx(kStatus) GoProfileStripZ_SetSelectType(GoProfileStripZ measurement, GoProfileGrooveSelectType selectType)
{
    kObj(GoProfileStripZ, measurement);
    kCheckState(GoSensor_IsConfigurable(GoMeasurement_Sensor(measurement)));
    kCheck(GoSensor_CacheConfig(GoMeasurement_Sensor(measurement)));
    obj->selectType = selectType;
    kCheck(GoSensor_SetConfigModified(GoMeasurement_Sensor(measurement)));

    return kOK;
}

GoFx(k32u) GoProfileStripZ_SelectIndex(GoProfileStripZ measurement)
{
    kObj(GoProfileStripZ, measurement);
    GoSensor_SyncConfig(GoMeasurement_Sensor(measurement));

    return obj->selectIndex;
}

GoFx(kStatus) GoProfileStripZ_SetSelectIndex(GoProfileStripZ measurement, k32u selectIndex)
{
    kObj(GoProfileStripZ, measurement);
    kCheckState(GoSensor_IsConfigurable(GoMeasurement_Sensor(measurement)));
    kCheck(GoSensor_CacheConfig(GoMeasurement_Sensor(measurement)));
    obj->selectIndex = selectIndex;
    kCheck(GoSensor_SetConfigModified(GoMeasurement_Sensor(measurement)));

    return kOK;
}


kBeginClassEx(Go, GoProfileStripWidth)
    kAddVMethod(GoProfileStripWidth, GoMeasurement, VInit)
    kAddVMethod(GoProfileStripWidth, GoMeasurement, VRead)
    kAddVMethod(GoProfileStripWidth, GoMeasurement, VWrite)
kEndClassEx()

GoFx(kStatus) GoProfileStripWidth_VInit(GoProfileStripWidth measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kObjR(GoProfileStripWidth, measurement);

    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_PROFILE_STRIP_WIDTH, sensor, srcTool, kTRUE, alloc));

    obj->selectType = GO_PROFILE_STRIP_SELECT_TYPE_BEST;
    obj->selectIndex = 0;

    return kOK;
}

GoFx(kStatus) GoProfileStripWidth_VRead(GoProfileStripWidth measurement, kXml xml, kXmlItem item)
{
    kObj(GoProfileStripWidth, measurement);
    kCheck(GoMeasurement_VRead(measurement, xml, item));
    kCheck(kXml_Child32s(xml, item, "SelectType", &obj->selectType));
    kCheck(kXml_Child32u(xml, item, "SelectIndex", &obj->selectIndex));

    return kOK;
}

GoFx(kStatus) GoProfileStripWidth_VWrite(GoProfileStripWidth measurement, kXml xml, kXmlItem item)
{
    kObj(GoProfileStripWidth, measurement);
    kCheck(kXml_SetChild32s(xml, item, "SelectType", obj->selectType));
    kCheck(kXml_SetChild32u(xml, item, "SelectIndex", obj->selectIndex));
    kCheck(GoMeasurement_VWrite(measurement, xml, item));

    return kOK;
}

GoFx(GoProfileGrooveSelectType) GoProfileStripWidth_SelectType(GoProfileStripWidth measurement)
{
    kObj(GoProfileStripWidth, measurement);
    GoSensor_SyncConfig(GoMeasurement_Sensor(measurement));

    return obj->selectType;
}

GoFx(kStatus) GoProfileStripWidth_SetSelectType(GoProfileStripWidth measurement, GoProfileGrooveSelectType selectType)
{
    kObj(GoProfileStripWidth, measurement);
    kCheckState(GoSensor_IsConfigurable(GoMeasurement_Sensor(measurement)));
    kCheck(GoSensor_CacheConfig(GoMeasurement_Sensor(measurement)));
    obj->selectType = selectType;
    kCheck(GoSensor_SetConfigModified(GoMeasurement_Sensor(measurement)));

    return kOK;
}

GoFx(k32u) GoProfileStripWidth_SelectIndex(GoProfileStripWidth measurement)
{
    kObj(GoProfileStripWidth, measurement);
    GoSensor_SyncConfig(GoMeasurement_Sensor(measurement));

    return obj->selectIndex;
}

GoFx(kStatus) GoProfileStripWidth_SetSelectIndex(GoProfileStripWidth measurement, k32u selectIndex)
{
    kObj(GoProfileStripWidth, measurement);
    kCheckState(GoSensor_IsConfigurable(GoMeasurement_Sensor(measurement)));
    kCheck(GoSensor_CacheConfig(GoMeasurement_Sensor(measurement)));
    obj->selectIndex = selectIndex;
    kCheck(GoSensor_SetConfigModified(GoMeasurement_Sensor(measurement)));

    return kOK;
}


kBeginClassEx(Go, GoProfileStripHeight)
    kAddVMethod(GoProfileStripHeight, GoMeasurement, VInit)
    kAddVMethod(GoProfileStripHeight, GoMeasurement, VRead)
    kAddVMethod(GoProfileStripHeight, GoMeasurement, VWrite)
kEndClassEx()

GoFx(kStatus) GoProfileStripHeight_VInit(GoProfileStripHeight measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kObjR(GoProfileStripHeight, measurement);

    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_PROFILE_STRIP_HEIGHT, sensor, srcTool, kTRUE, alloc));

    obj->location = GO_PROFILE_STRIP_LOCATION_LEFT;
    obj->selectType = GO_PROFILE_STRIP_SELECT_TYPE_BEST;
    obj->selectIndex = 0;

    return kOK;
}

GoFx(kStatus) GoProfileStripHeight_VRead(GoProfileStripHeight measurement, kXml xml, kXmlItem item)
{
    kObj(GoProfileStripHeight, measurement);
    kCheck(GoMeasurement_VRead(measurement, xml, item));
    kCheck(kXml_Child32s(xml, item, "Location", &obj->location));
    kCheck(kXml_Child32s(xml, item, "SelectType", &obj->selectType));
    kCheck(kXml_Child32u(xml, item, "SelectIndex", &obj->selectIndex));

    return kOK;
}

GoFx(kStatus) GoProfileStripHeight_VWrite(GoProfileStripHeight measurement, kXml xml, kXmlItem item)
{
    kObj(GoProfileStripHeight, measurement);
    kCheck(kXml_SetChild32s(xml, item, "SelectType", obj->selectType));
    kCheck(kXml_SetChild32u(xml, item, "SelectIndex", obj->selectIndex));
    kCheck(kXml_SetChild32s(xml, item, "Location", obj->location));
    kCheck(GoMeasurement_VWrite(measurement, xml, item));

    return kOK;
}

GoFx(GoProfileGrooveLocation) GoProfileStripHeight_Location(GoProfileStripHeight measurement)
{
    kObj(GoProfileStripHeight, measurement);
    GoSensor_SyncConfig(GoMeasurement_Sensor(measurement));

    return obj->location;
}

GoFx(kStatus) GoProfileStripHeight_SetLocation(GoProfileStripHeight measurement, GoProfileGrooveLocation location)
{
    kObj(GoProfileStripHeight, measurement);
    kCheckState(GoSensor_IsConfigurable(GoMeasurement_Sensor(measurement)));
    kCheck(GoSensor_CacheConfig(GoMeasurement_Sensor(measurement)));
    obj->location = location;
    kCheck(GoSensor_SetConfigModified(GoMeasurement_Sensor(measurement)));

    return kOK;
}

GoFx(GoProfileGrooveSelectType) GoProfileStripHeight_SelectType(GoProfileStripHeight measurement)
{
    kObj(GoProfileStripHeight, measurement);
    GoSensor_SyncConfig(GoMeasurement_Sensor(measurement));

    return obj->selectType;
}

GoFx(kStatus) GoProfileStripHeight_SetSelectType(GoProfileStripHeight measurement, GoProfileGrooveSelectType selectType)
{
    kObj(GoProfileStripHeight, measurement);
    kCheckState(GoSensor_IsConfigurable(GoMeasurement_Sensor(measurement)));
    kCheck(GoSensor_CacheConfig(GoMeasurement_Sensor(measurement)));
    obj->selectType = selectType;
    kCheck(GoSensor_SetConfigModified(GoMeasurement_Sensor(measurement)));

    return kOK;
}

GoFx(k32u) GoProfileStripHeight_SelectIndex(GoProfileStripHeight measurement)
{
    kObj(GoProfileStripHeight, measurement);
    GoSensor_SyncConfig(GoMeasurement_Sensor(measurement));

    return obj->selectIndex;
}

GoFx(kStatus) GoProfileStripHeight_SetSelectIndex(GoProfileStripHeight measurement, k32u selectIndex)
{
    kObj(GoProfileStripHeight, measurement);
    kCheckState(GoSensor_IsConfigurable(GoMeasurement_Sensor(measurement)));
    kCheck(GoSensor_CacheConfig(GoMeasurement_Sensor(measurement)));
    obj->selectIndex = selectIndex;
    kCheck(GoSensor_SetConfigModified(GoMeasurement_Sensor(measurement)));

    return kOK;
}

/* Surface tool measurements */

kBeginClassEx(Go, GoSurfaceBoxX)
    kAddVMethod(GoSurfaceBoxX, GoMeasurement, VInit)
kEndClassEx()

GoFx(kStatus) GoSurfaceBoxX_VInit(GoSurfaceBoxX measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_SURFACE_BOUNDING_BOX_X, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


kBeginClassEx(Go, GoSurfaceBoxY)
    kAddVMethod(GoSurfaceBoxY, GoMeasurement, VInit)
kEndClassEx()

GoFx(kStatus) GoSurfaceBoxY_VInit(GoSurfaceBoxY measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_SURFACE_BOUNDING_BOX_Y, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


kBeginClassEx(Go, GoSurfaceBoxZ)
    kAddVMethod(GoSurfaceBoxZ, GoMeasurement, VInit)
kEndClassEx()

GoFx(kStatus) GoSurfaceBoxZ_VInit(GoSurfaceBoxZ measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_SURFACE_BOUNDING_BOX_Z, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


kBeginClassEx(Go, GoSurfaceBoxWidth)
    kAddVMethod(GoSurfaceBoxWidth, GoMeasurement, VInit)
kEndClassEx()


GoFx(kStatus) GoSurfaceBoxWidth_VInit(GoSurfaceBoxWidth measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_SURFACE_BOUNDING_BOX_WIDTH, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


kBeginClassEx(Go, GoSurfaceBoxLength)
    kAddVMethod(GoSurfaceBoxLength, GoMeasurement, VInit)
kEndClassEx()

GoFx(kStatus) GoSurfaceBoxLength_VInit(GoSurfaceBoxLength measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_SURFACE_BOUNDING_BOX_LENGTH, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


kBeginClassEx(Go, GoSurfaceBoxHeight)
    kAddVMethod(GoSurfaceBoxHeight, GoMeasurement, VInit)
kEndClassEx()

GoFx(kStatus) GoSurfaceBoxHeight_VInit(GoSurfaceBoxHeight measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_SURFACE_BOUNDING_BOX_HEIGHT, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


kBeginClassEx(Go, GoSurfaceBoxZAngle)
    kAddVMethod(GoSurfaceBoxZAngle, GoMeasurement, VInit)
kEndClassEx()

GoFx(kStatus) GoSurfaceBoxZAngle_VInit(GoSurfaceBoxZAngle measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_SURFACE_BOUNDING_BOX_ZANGLE, sensor, srcTool, kTRUE, alloc));

    return kOK;
}

kBeginClassEx(Go, GoSurfaceBoxGlobalX)
    kAddVMethod(GoSurfaceBoxGlobalX, GoMeasurement, VInit)
kEndClassEx()

GoFx(kStatus) GoSurfaceBoxGlobalX_VInit(GoSurfaceBoxGlobalX measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_SURFACE_BOUNDING_BOX_GLOBAL_X, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


kBeginClassEx(Go, GoSurfaceBoxGlobalY)
    kAddVMethod(GoSurfaceBoxGlobalY, GoMeasurement, VInit)
kEndClassEx()

GoFx(kStatus) GoSurfaceBoxGlobalY_VInit(GoSurfaceBoxGlobalY measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_SURFACE_BOUNDING_BOX_GLOBAL_Y, sensor, srcTool, kTRUE, alloc));

    return kOK;
}

kBeginClassEx(Go, GoSurfaceBoxGlobalZAngle)
    kAddVMethod(GoSurfaceBoxGlobalZAngle, GoMeasurement, VInit)
kEndClassEx()

GoFx(kStatus) GoSurfaceBoxGlobalZAngle_VInit(GoSurfaceBoxGlobalY measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_SURFACE_BOUNDING_BOX_GLOBAL_Z_ANGLE, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


kBeginClassEx(Go, GoSurfaceCountersunkHoleX)
kAddVMethod(GoSurfaceCountersunkHoleX, GoMeasurement, VInit)
kEndClassEx()

GoFx(kStatus) GoSurfaceCountersunkHoleX_VInit(GoSurfaceCountersunkHoleX measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_SURFACE_COUNTERSUNK_HOLE_X, sensor, srcTool, kTRUE, alloc));

    return kOK;
}

kBeginClassEx(Go, GoSurfaceCountersunkHoleXAngle)
kAddVMethod(GoSurfaceCountersunkHoleXAngle, GoMeasurement, VInit)
kEndClassEx()

GoFx(kStatus) GoSurfaceCountersunkHoleXAngle_VInit(GoSurfaceCountersunkHoleXAngle measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_SURFACE_COUNTERSUNK_HOLE_X_ANGLE, sensor, srcTool, kTRUE, alloc));

    return kOK;
}

kBeginClassEx(Go, GoSurfaceCountersunkHoleY)
kAddVMethod(GoSurfaceCountersunkHoleY, GoMeasurement, VInit)
kEndClassEx()

GoFx(kStatus) GoSurfaceCountersunkHoleY_VInit(GoSurfaceCountersunkHoleY measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_SURFACE_COUNTERSUNK_HOLE_Y, sensor, srcTool, kTRUE, alloc));

    return kOK;
}

kBeginClassEx(Go, GoSurfaceCountersunkHoleYAngle)
kAddVMethod(GoSurfaceCountersunkHoleYAngle, GoMeasurement, VInit)
kEndClassEx()

GoFx(kStatus) GoSurfaceCountersunkHoleYAngle_VInit(GoSurfaceCountersunkHoleYAngle measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_SURFACE_COUNTERSUNK_HOLE_Y_ANGLE, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


kBeginClassEx(Go, GoSurfaceCountersunkHoleZ)
kAddVMethod(GoSurfaceCountersunkHoleZ, GoMeasurement, VInit)
kEndClassEx()

GoFx(kStatus) GoSurfaceCountersunkHoleZ_VInit(GoSurfaceCountersunkHoleZ measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_SURFACE_COUNTERSUNK_HOLE_Z, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


kBeginClassEx(Go, GoSurfaceCountersunkHoleCounterboreDepth)
kAddVMethod(GoSurfaceCountersunkHoleCounterboreDepth, GoMeasurement, VInit)
kEndClassEx()

GoFx(kStatus) GoSurfaceCountersunkHoleCounterboreDepth_VInit(GoSurfaceCountersunkHoleCounterboreDepth measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_SURFACE_COUNTERSUNK_HOLE_COUNTERBORE_DEPTH, sensor, srcTool, kTRUE, alloc));

    return kOK;
}

kBeginClassEx(Go, GoSurfaceCountersunkHoleBevelAngle)
kAddVMethod(GoSurfaceCountersunkHoleBevelAngle, GoMeasurement, VInit)
kEndClassEx()

GoFx(kStatus) GoSurfaceCountersunkHoleBevelAngle_VInit(GoSurfaceCountersunkHoleBevelAngle measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_SURFACE_COUNTERSUNK_HOLE_BEVEL_ANGLE, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


kBeginClassEx(Go, GoSurfaceCountersunkHoleBevelRadius)
kAddVMethod(GoSurfaceCountersunkHoleBevelRadius, GoMeasurement, VInit)
kEndClassEx()

GoFx(kStatus) GoSurfaceCountersunkHoleBevelRadius_VInit(GoSurfaceCountersunkHoleBevelRadius measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_SURFACE_COUNTERSUNK_HOLE_BEVEL_RADIUS, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


kBeginClassEx(Go, GoSurfaceCountersunkHoleDepth)
kAddVMethod(GoSurfaceCountersunkHoleDepth, GoMeasurement, VInit)
kEndClassEx()

GoFx(kStatus) GoSurfaceCountersunkHoleDepth_VInit(GoSurfaceCountersunkHoleDepth measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_SURFACE_COUNTERSUNK_HOLE_DEPTH, sensor, srcTool, kTRUE, alloc));

    return kOK;
}

kBeginClassEx(Go, GoSurfaceCountersunkHoleOuterRadius)
kAddVMethod(GoSurfaceCountersunkHoleOuterRadius, GoMeasurement, VInit)
kEndClassEx()

GoFx(kStatus) GoSurfaceCountersunkHoleOuterRadius_VInit(GoSurfaceCountersunkHoleOuterRadius measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_SURFACE_COUNTERSUNK_HOLE_OUTER_RADIUS, sensor, srcTool, kTRUE, alloc));

    return kOK;
}

kBeginClassEx(Go, GoSurfaceCountersunkHoleAxisTilt)
kAddVMethod(GoSurfaceCountersunkHoleAxisTilt, GoMeasurement, VInit)
kEndClassEx()

GoFx(kStatus) GoSurfaceCountersunkHoleAxisTilt_VInit(GoSurfaceCountersunkHoleAxisTilt measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_SURFACE_COUNTERSUNK_HOLE_AXIS_TILT, sensor, srcTool, kTRUE, alloc));

    return kOK;
}

kBeginClassEx(Go, GoSurfaceCountersunkHoleAxisOrientation)
kAddVMethod(GoSurfaceCountersunkHoleAxisOrientation, GoMeasurement, VInit)
kEndClassEx()

GoFx(kStatus) GoSurfaceCountersunkHoleAxisOrientation_VInit(GoSurfaceCountersunkHoleAxisOrientation measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_SURFACE_COUNTERSUNK_HOLE_AXIS_ORIENTATION, sensor, srcTool, kTRUE, alloc));

    return kOK;
}

kBeginClassEx(Go, GoSurfaceDimWidth)
kAddVMethod(GoSurfaceDimWidth, GoMeasurement, VInit)
kAddVMethod(GoSurfaceDimWidth, GoMeasurement, VRead)
kAddVMethod(GoSurfaceDimWidth, GoMeasurement, VWrite)
kEndClassEx()

GoFx(kStatus) GoSurfaceDimWidth_VInit(GoSurfaceDimWidth measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kObjR(GoSurfaceDimWidth, measurement);

    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_SURFACE_DIMENSION_WIDTH, sensor, srcTool, kTRUE, alloc));

    obj->absolute = kFALSE;

    return kOK;
}

GoFx(kStatus) GoSurfaceDimWidth_VRead(GoSurfaceDimWidth measurement, kXml xml, kXmlItem item)
{
    kObj(GoSurfaceDimWidth, measurement);

    kCheck(GoMeasurement_VRead(measurement, xml, item));
    kCheck(kXml_Child32s(xml, item, "Absolute", &obj->absolute));

    return kOK;
}

GoFx(kStatus) GoSurfaceDimWidth_VWrite(GoSurfaceDimWidth measurement, kXml xml, kXmlItem item)
{
    kObj(GoSurfaceDimWidth, measurement);

    kCheck(kXml_SetChild32s(xml, item, "Absolute", obj->absolute));
    kCheck(GoMeasurement_VWrite(measurement, xml, item));

    return kOK;
}

GoFx(kBool) GoSurfaceDimWidth_AbsoluteEnabled(GoSurfaceDimWidth measurement)
{
    kObj(GoSurfaceDimWidth, measurement);

    GoSensor_SyncConfig(GoMeasurement_Sensor(measurement));

    return obj->absolute;
}

GoFx(kStatus) GoSurfaceDimWidth_EnableAbsolute(GoSurfaceDimWidth measurement, kBool absolute)
{
    kObj(GoSurfaceDimWidth, measurement);

    kCheckState(GoSensor_IsConfigurable(GoMeasurement_Sensor(measurement)));
    kCheck(GoSensor_CacheConfig(GoMeasurement_Sensor(measurement)));
    obj->absolute = absolute;
    kCheck(GoSensor_SetConfigModified(GoMeasurement_Sensor(measurement)));

    return kOK;
}


kBeginClassEx(Go, GoSurfaceDimHeight)
kAddVMethod(GoSurfaceDimHeight, GoMeasurement, VInit)
kAddVMethod(GoSurfaceDimHeight, GoMeasurement, VRead)
kAddVMethod(GoSurfaceDimHeight, GoMeasurement, VWrite)
kEndClassEx()

GoFx(kStatus) GoSurfaceDimHeight_VInit(GoSurfaceDimHeight measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kObjR(GoSurfaceDimHeight, measurement);

    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_SURFACE_DIMENSION_HEIGHT, sensor, srcTool, kTRUE, alloc));

    obj->absolute = kFALSE;

    return kOK;
}

GoFx(kStatus) GoSurfaceDimHeight_VRead(GoSurfaceDimHeight measurement, kXml xml, kXmlItem item)
{
    kObj(GoSurfaceDimHeight, measurement);

    kCheck(GoMeasurement_VRead(measurement, xml, item));
    kCheck(kXml_Child32s(xml, item, "Absolute", &obj->absolute));

    return kOK;
}

GoFx(kStatus) GoSurfaceDimHeight_VWrite(GoSurfaceDimHeight measurement, kXml xml, kXmlItem item)
{
    kObj(GoSurfaceDimHeight, measurement);

    kCheck(kXml_SetChild32s(xml, item, "Absolute", obj->absolute));
    kCheck(GoMeasurement_VWrite(measurement, xml, item));

    return kOK;
}

GoFx(kBool) GoSurfaceDimHeight_AbsoluteEnabled(GoSurfaceDimHeight measurement)
{
    kObj(GoSurfaceDimHeight, measurement);

    GoSensor_SyncConfig(GoMeasurement_Sensor(measurement));

    return obj->absolute;
}

GoFx(kStatus) GoSurfaceDimHeight_EnableAbsolute(GoSurfaceDimHeight measurement, kBool absolute)
{
    kObj(GoSurfaceDimHeight, measurement);

    kCheckState(GoSensor_IsConfigurable(GoMeasurement_Sensor(measurement)));
    kCheck(GoSensor_CacheConfig(GoMeasurement_Sensor(measurement)));
    obj->absolute = absolute;
    kCheck(GoSensor_SetConfigModified(GoMeasurement_Sensor(measurement)));

    return kOK;
}


kBeginClassEx(Go, GoSurfaceDimLength)
kAddVMethod(GoSurfaceDimLength, GoMeasurement, VInit)
kAddVMethod(GoSurfaceDimLength, GoMeasurement, VRead)
kAddVMethod(GoSurfaceDimLength, GoMeasurement, VWrite)
kEndClassEx()

GoFx(kStatus) GoSurfaceDimLength_VInit(GoSurfaceDimLength measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kObjR(GoSurfaceDimLength, measurement);

    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_SURFACE_DIMENSION_LENGTH, sensor, srcTool, kTRUE, alloc));

    obj->absolute = kFALSE;

    return kOK;
}

GoFx(kStatus) GoSurfaceDimLength_VRead(GoSurfaceDimLength measurement, kXml xml, kXmlItem item)
{
    kObj(GoSurfaceDimLength, measurement);

    kCheck(GoMeasurement_VRead(measurement, xml, item));
    kCheck(kXml_Child32s(xml, item, "Absolute", &obj->absolute));

    return kOK;
}

GoFx(kStatus) GoSurfaceDimLength_VWrite(GoSurfaceDimLength measurement, kXml xml, kXmlItem item)
{
    kObj(GoSurfaceDimLength, measurement);

    kCheck(kXml_SetChild32s(xml, item, "Absolute", obj->absolute));
    kCheck(GoMeasurement_VWrite(measurement, xml, item));

    return kOK;
}

GoFx(kBool) GoSurfaceDimLength_AbsoluteEnabled(GoSurfaceDimLength measurement)
{
    kObj(GoSurfaceDimLength, measurement);

    GoSensor_SyncConfig(GoMeasurement_Sensor(measurement));

    return obj->absolute;
}

GoFx(kStatus) GoSurfaceDimLength_EnableAbsolute(GoSurfaceDimLength measurement, kBool absolute)
{
    kObj(GoSurfaceDimLength, measurement);

    kCheckState(GoSensor_IsConfigurable(GoMeasurement_Sensor(measurement)));
    kCheck(GoSensor_CacheConfig(GoMeasurement_Sensor(measurement)));
    obj->absolute = absolute;
    kCheck(GoSensor_SetConfigModified(GoMeasurement_Sensor(measurement)));

    return kOK;
}


kBeginClassEx(Go, GoSurfaceDimDistance)
kAddVMethod(GoSurfaceDimDistance, GoMeasurement, VInit)
kEndClassEx()

GoFx(kStatus) GoSurfaceDimDistance_VInit(GoSurfaceDimDistance measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_SURFACE_DIMENSION_DISTANCE, sensor, srcTool, kTRUE, alloc));

    return kOK;
}

kBeginClassEx(Go, GoSurfaceDimPlaneDistance)
kAddVMethod(GoSurfaceDimPlaneDistance, GoMeasurement, VInit)
kEndClassEx()

GoFx(kStatus) GoSurfaceDimPlaneDistance_VInit(GoSurfaceDimPlaneDistance measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_SURFACE_DIMENSION_PLANE_DISTANCE, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


kBeginClassEx(Go, GoSurfaceDimCenterX)
kAddVMethod(GoSurfaceDimCenterX, GoMeasurement, VInit)
kEndClassEx()

GoFx(kStatus) GoSurfaceDimCenterX_VInit(GoSurfaceDimCenterX measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_SURFACE_DIMENSION_CENTER_X, sensor, srcTool, kTRUE, alloc));

    return kOK;
}

kBeginClassEx(Go, GoSurfaceDimCenterY)
kAddVMethod(GoSurfaceDimCenterY, GoMeasurement, VInit)
kEndClassEx()

GoFx(kStatus) GoSurfaceDimCenterY_VInit(GoSurfaceDimCenterY measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_SURFACE_DIMENSION_CENTER_Y, sensor, srcTool, kTRUE, alloc));

    return kOK;
}

kBeginClassEx(Go, GoSurfaceDimCenterZ)
kAddVMethod(GoSurfaceDimCenterZ, GoMeasurement, VInit)
kEndClassEx()

GoFx(kStatus) GoSurfaceDimCenterZ_VInit(GoSurfaceDimCenterZ measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_SURFACE_DIMENSION_CENTER_Z, sensor, srcTool, kTRUE, alloc));

    return kOK;
}

kBeginClassEx(Go, GoSurfaceEllipseMajor)
    kAddVMethod(GoSurfaceEllipseMajor, GoMeasurement, VInit)
kEndClassEx()

GoFx(kStatus) GoSurfaceEllipseMajor_VInit(GoSurfaceEllipseMajor measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_SURFACE_ELLIPSE_MAJOR, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


kBeginClassEx(Go, GoSurfaceEllipseMinor)
    kAddVMethod(GoSurfaceEllipseMinor, GoMeasurement, VInit)
kEndClassEx()

GoFx(kStatus) GoSurfaceEllipseMinor_VInit(GoSurfaceEllipseMinor measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_SURFACE_ELLIPSE_MINOR, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


kBeginClassEx(Go, GoSurfaceEllipseRatio)
    kAddVMethod(GoSurfaceEllipseRatio, GoMeasurement, VInit)
kEndClassEx()

GoFx(kStatus) GoSurfaceEllipseRatio_VInit(GoSurfaceEllipseRatio measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_SURFACE_ELLIPSE_RATIO, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


kBeginClassEx(Go, GoSurfaceEllipseZAngle)
    kAddVMethod(GoSurfaceEllipseZAngle, GoMeasurement, VInit)
kEndClassEx()

GoFx(kStatus) GoSurfaceEllipseZAngle_VInit(GoSurfaceEllipseZAngle measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_SURFACE_ELLIPSE_ZANGLE, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


kBeginClassEx(Go, GoSurfaceHoleX)
    kAddVMethod(GoSurfaceHoleX, GoMeasurement, VInit)
kEndClassEx()

GoFx(kStatus) GoSurfaceHoleX_VInit(GoSurfaceHoleX measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_SURFACE_HOLE_X, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


kBeginClassEx(Go, GoSurfaceHoleY)
    kAddVMethod(GoSurfaceHoleY, GoMeasurement, VInit)
kEndClassEx()

GoFx(kStatus) GoSurfaceHoleY_VInit(GoSurfaceHoleY measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_SURFACE_HOLE_Y, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


kBeginClassEx(Go, GoSurfaceHoleZ)
    kAddVMethod(GoSurfaceHoleZ, GoMeasurement, VInit)
kEndClassEx()

GoFx(kStatus) GoSurfaceHoleZ_VInit(GoSurfaceHoleZ measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_SURFACE_HOLE_Z, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


kBeginClassEx(Go, GoSurfaceHoleRadius)
    kAddVMethod(GoSurfaceHoleRadius, GoMeasurement, VInit)
kEndClassEx()

GoFx(kStatus) GoSurfaceHoleRadius_VInit(GoSurfaceHoleRadius measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_SURFACE_HOLE_RADIUS, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


kBeginClassEx(Go, GoSurfaceOpeningX)
    kAddVMethod(GoSurfaceOpeningX, GoMeasurement, VInit)
kEndClassEx()

GoFx(kStatus) GoSurfaceOpeningX_VInit(GoSurfaceOpeningX measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_SURFACE_OPENING_X, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


kBeginClassEx(Go, GoSurfaceOpeningY)
    kAddVMethod(GoSurfaceOpeningY, GoMeasurement, VInit)
kEndClassEx()

GoFx(kStatus) GoSurfaceOpeningY_VInit(GoSurfaceOpeningY measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_SURFACE_OPENING_Y, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


kBeginClassEx(Go, GoSurfaceOpeningZ)
    kAddVMethod(GoSurfaceOpeningZ, GoMeasurement, VInit)
kEndClassEx()

GoFx(kStatus) GoSurfaceOpeningZ_VInit(GoSurfaceOpeningZ measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_SURFACE_OPENING_Z, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


kBeginClassEx(Go, GoSurfaceOpeningWidth)
    kAddVMethod(GoSurfaceOpeningWidth, GoMeasurement, VInit)
kEndClassEx()

GoFx(kStatus) GoSurfaceOpeningWidth_VInit(GoSurfaceOpeningWidth measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_SURFACE_OPENING_WIDTH, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


kBeginClassEx(Go, GoSurfaceOpeningLength)
    kAddVMethod(GoSurfaceOpeningLength, GoMeasurement, VInit)
kEndClassEx()

GoFx(kStatus) GoSurfaceOpeningLength_VInit(GoSurfaceOpeningLength measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_SURFACE_OPENING_LENGTH, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


kBeginClassEx(Go, GoSurfaceOpeningAngle)
    kAddVMethod(GoSurfaceOpeningAngle, GoMeasurement, VInit)
kEndClassEx()

GoFx(kStatus) GoSurfaceOpeningAngle_VInit(GoSurfaceOpeningAngle measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_SURFACE_OPENING_ANGLE, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


kBeginClassEx(Go, GoSurfacePlaneXAngle)
    kAddVMethod(GoSurfacePlaneXAngle, GoMeasurement, VInit)
kEndClassEx()

GoFx(kStatus) GoSurfacePlaneXAngle_VInit(GoSurfacePlaneXAngle measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_SURFACE_PLANE_X_ANGLE, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


kBeginClassEx(Go, GoSurfacePlaneYAngle)
    kAddVMethod(GoSurfacePlaneYAngle, GoMeasurement, VInit)
kEndClassEx()

GoFx(kStatus) GoSurfacePlaneYAngle_VInit(GoSurfacePlaneYAngle measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_SURFACE_PLANE_Y_ANGLE, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


kBeginClassEx(Go, GoSurfacePlaneZOffset)
    kAddVMethod(GoSurfacePlaneZOffset, GoMeasurement, VInit)
kEndClassEx()

GoFx(kStatus) GoSurfacePlaneZOffset_VInit(GoSurfacePlaneZOffset measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_SURFACE_PLANE_Z_OFFSET, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


kBeginClassEx(Go, GoSurfacePlaneStdDev)
    kAddVMethod(GoSurfacePlaneStdDev, GoMeasurement, VInit)
kEndClassEx()

GoFx(kStatus) GoSurfacePlaneStdDev_VInit(GoSurfacePlaneStdDev measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_SURFACE_PLANE_STD_DEV, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


kBeginClassEx(Go, GoSurfacePlaneMinError)
    kAddVMethod(GoSurfacePlaneMinError, GoMeasurement, VInit)
kEndClassEx()

GoFx(kStatus) GoSurfacePlaneMinError_VInit(GoSurfacePlaneMinError measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_SURFACE_PLANE_ERROR_MIN, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


kBeginClassEx(Go, GoSurfacePlaneMaxError)
    kAddVMethod(GoSurfacePlaneMaxError, GoMeasurement, VInit)
kEndClassEx()

GoFx(kStatus) GoSurfacePlaneMaxError_VInit(GoSurfacePlaneMaxError measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_SURFACE_PLANE_ERROR_MAX, sensor, srcTool, kTRUE, alloc));

    return kOK;
}

kBeginClassEx(Go, GoSurfacePlaneXNormal)
kAddVMethod(GoSurfacePlaneXNormal, GoMeasurement, VInit)
kEndClassEx()

GoFx(kStatus) GoSurfacePlaneXNormal_VInit(GoSurfacePlaneXNormal measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_SURFACE_PLANE_X_NORMAL, sensor, srcTool, kTRUE, alloc));

    return kOK;
}

kBeginClassEx(Go, GoSurfacePlaneYNormal)
kAddVMethod(GoSurfacePlaneYNormal, GoMeasurement, VInit)
kEndClassEx()

GoFx(kStatus) GoSurfacePlaneYNormal_VInit(GoSurfacePlaneYNormal measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_SURFACE_PLANE_Y_NORMAL, sensor, srcTool, kTRUE, alloc));

    return kOK;
}

kBeginClassEx(Go, GoSurfacePlaneZNormal)
kAddVMethod(GoSurfacePlaneZNormal, GoMeasurement, VInit)
kEndClassEx()

GoFx(kStatus) GoSurfacePlaneZNormal_VInit(GoSurfacePlaneZNormal measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_SURFACE_PLANE_Z_NORMAL, sensor, srcTool, kTRUE, alloc));

    return kOK;
}

kBeginClassEx(Go, GoSurfacePlaneDistance)
kAddVMethod(GoSurfacePlaneDistance, GoMeasurement, VInit)
kEndClassEx()

GoFx(kStatus) GoSurfacePlaneDistance_VInit(GoSurfacePlaneDistance measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_SURFACE_PLANE_DISTANCE, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


kBeginClassEx(Go, GoSurfacePositionX)
kAddVMethod(GoSurfacePositionX, GoMeasurement, VInit)
kEndClassEx()

GoFx(kStatus) GoSurfacePositionX_VInit(GoSurfacePositionX measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_SURFACE_POSITION_X, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


kBeginClassEx(Go, GoSurfacePositionY)
kAddVMethod(GoSurfacePositionY, GoMeasurement, VInit)
kEndClassEx()

GoFx(kStatus) GoSurfacePositionY_VInit(GoSurfacePositionY measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_SURFACE_POSITION_Y, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


kBeginClassEx(Go, GoSurfacePositionZ)
kAddVMethod(GoSurfacePositionZ, GoMeasurement, VInit)
kEndClassEx()

GoFx(kStatus) GoSurfacePositionZ_VInit(GoSurfacePositionZ measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_SURFACE_POSITION_Z, sensor, srcTool, kTRUE, alloc));

    return kOK;
}

kBeginClassEx(Go, GoSurfaceRivetX)
kAddVMethod(GoSurfaceRivetX, GoMeasurement, VInit)
kEndClassEx()

GoFx(kStatus) GoSurfaceRivetX_VInit(GoSurfaceRivetX measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_SURFACE_RIVET_X, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


kBeginClassEx(Go, GoSurfaceRivetY)
kAddVMethod(GoSurfaceRivetY, GoMeasurement, VInit)
kEndClassEx()

GoFx(kStatus) GoSurfaceRivetY_VInit(GoSurfaceRivetY measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_SURFACE_RIVET_Y, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


kBeginClassEx(Go, GoSurfaceRivetZ)
kAddVMethod(GoSurfaceRivetZ, GoMeasurement, VInit)
kEndClassEx()

GoFx(kStatus) GoSurfaceRivetZ_VInit(GoSurfaceRivetZ measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_SURFACE_RIVET_Z, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


kBeginClassEx(Go, GoSurfaceRivetTiltAngle)
kAddVMethod(GoSurfaceRivetTiltAngle, GoMeasurement, VInit)
kEndClassEx()

GoFx(kStatus) GoSurfaceRivetTiltAngle_VInit(GoSurfaceRivetTiltAngle measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_SURFACE_RIVET_TILT_ANGLE, sensor, srcTool, kTRUE, alloc));

    return kOK;
}

kBeginClassEx(Go, GoSurfaceRivetTiltDirection)
kAddVMethod(GoSurfaceRivetTiltDirection, GoMeasurement, VInit)
kEndClassEx()

GoFx(kStatus) GoSurfaceRivetTiltDirection_VInit(GoSurfaceRivetTiltDirection measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_SURFACE_RIVET_TILT_DIRECTION, sensor, srcTool, kTRUE, alloc));

    return kOK;
}

kBeginClassEx(Go, GoSurfaceRivetRadius)
kAddVMethod(GoSurfaceRivetRadius, GoMeasurement, VInit)
kEndClassEx()

GoFx(kStatus) GoSurfaceRivetRadius_VInit(GoSurfaceRivetRadius measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_SURFACE_RIVET_RADIUS, sensor, srcTool, kTRUE, alloc));

    return kOK;
}

kBeginClassEx(Go, GoSurfaceRivetTopOffsetMin)
kAddVMethod(GoSurfaceRivetTopOffsetMin, GoMeasurement, VInit)
kEndClassEx()

GoFx(kStatus) GoSurfaceRivetTopOffsetMin_VInit(GoSurfaceRivetTopOffsetMin measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_SURFACE_RIVET_TOP_OFFSET_MIN, sensor, srcTool, kTRUE, alloc));

    return kOK;
}

kBeginClassEx(Go, GoSurfaceRivetTopOffsetMax)
kAddVMethod(GoSurfaceRivetTopOffsetMax, GoMeasurement, VInit)
kEndClassEx()

GoFx(kStatus) GoSurfaceRivetTopOffsetMax_VInit(GoSurfaceRivetTopOffsetMax measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_SURFACE_RIVET_TOP_OFFSET_MAX, sensor, srcTool, kTRUE, alloc));

    return kOK;
}

kBeginClassEx(Go, GoSurfaceRivetTopOffsetMean)
kAddVMethod(GoSurfaceRivetTopOffsetMean, GoMeasurement, VInit)
kEndClassEx()

GoFx(kStatus) GoSurfaceRivetTopOffsetMean_VInit(GoSurfaceRivetTopOffsetMean measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_SURFACE_RIVET_TOP_OFFSET_MEAN, sensor, srcTool, kTRUE, alloc));

    return kOK;
}

kBeginClassEx(Go, GoSurfaceRivetTopOffsetStdDev)
kAddVMethod(GoSurfaceRivetTopOffsetStdDev, GoMeasurement, VInit)
kEndClassEx()

GoFx(kStatus) GoSurfaceRivetTopOffsetStdDev_VInit(GoSurfaceRivetTopOffsetStdDev measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_SURFACE_RIVET_TOP_OFFSET_STD_DEV, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


kBeginClassEx(Go, GoSurfaceRivetRadialHeightMin)
kAddVMethod(GoSurfaceRivetRadialHeightMin, GoMeasurement, VInit)
kAddVMethod(GoSurfaceRivetRadialHeightMin, GoMeasurement, VRead)
kAddVMethod(GoSurfaceRivetRadialHeightMin, GoMeasurement, VWrite)
kEndClassEx()

GoFx(kStatus) GoSurfaceRivetRadialHeightMin_VInit(GoSurfaceRivetRadialHeightMin measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kObjR(GoSurfaceRivetRadialHeightMin, measurement);

    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_SURFACE_RIVET_RADIAL_HEIGHT_MIN, sensor, srcTool, kTRUE, alloc));
    obj->radius = GO_MEASUREMENT_SURFACE_RIVET_DEFAULT_RADIUS;

    return kOK;
}

GoFx(kStatus) GoSurfaceRivetRadialHeightMin_VRead(GoSurfaceRivetRadialHeightMin measurement, kXml xml, kXmlItem item)
{
    kObj(GoSurfaceRivetRadialHeightMin, measurement);
    kCheck(GoMeasurement_VRead(measurement, xml, item));
    kCheck(kXml_Child64f(xml, item, "Radius", &obj->radius));

    return kOK;
}

GoFx(kStatus) GoSurfaceRivetRadialHeightMin_VWrite(GoSurfaceRivetRadialHeightMin measurement, kXml xml, kXmlItem item)
{
    kObj(GoSurfaceRivetRadialHeightMin, measurement);
    kCheck(kXml_SetChild64f(xml, item, "Radius", obj->radius));
    kCheck(GoMeasurement_VWrite(measurement, xml, item));

    return kOK;
}

GoFx(k64f) GoSurfaceRivetRadialHeightMin_Radius(GoSurfaceRivetRadialHeightMin measurement)
{
    kObj(GoSurfaceRivetRadialHeightMin, measurement);
    GoSensor_SyncConfig(GoMeasurement_Sensor(measurement));

    return obj->radius;
}

GoFx(kStatus) GoSurfaceRivetRadialHeightMin_SetRadius(GoSurfaceRivetRadialHeightMin measurement, k64f value)
{
    kObj(GoSurfaceRivetRadialHeightMin, measurement);
    kCheckState(GoSensor_IsConfigurable(GoMeasurement_Sensor(measurement)));
    kCheck(GoSensor_CacheConfig(GoMeasurement_Sensor(measurement)));
    obj->radius = value;
    kCheck(GoSensor_SetConfigModified(GoMeasurement_Sensor(measurement)));

    return kOK;
}

kBeginClassEx(Go, GoSurfaceRivetRadialHeightMax)
kAddVMethod(GoSurfaceRivetRadialHeightMax, GoMeasurement, VInit)
kAddVMethod(GoSurfaceRivetRadialHeightMax, GoMeasurement, VRead)
kAddVMethod(GoSurfaceRivetRadialHeightMax, GoMeasurement, VWrite)
kEndClassEx()

GoFx(kStatus) GoSurfaceRivetRadialHeightMax_VInit(GoSurfaceRivetRadialHeightMax measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kObjR(GoSurfaceRivetRadialHeightMax, measurement);

    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_SURFACE_RIVET_RADIAL_HEIGHT_MAX, sensor, srcTool, kTRUE, alloc));
    obj->radius = GO_MEASUREMENT_SURFACE_RIVET_DEFAULT_RADIUS;

    return kOK;
}

GoFx(kStatus) GoSurfaceRivetRadialHeightMax_VRead(GoSurfaceRivetRadialHeightMax measurement, kXml xml, kXmlItem item)
{
    kObj(GoSurfaceRivetRadialHeightMax, measurement);
    kCheck(GoMeasurement_VRead(measurement, xml, item));
    kCheck(kXml_Child64f(xml, item, "Radius", &obj->radius));

    return kOK;
}

GoFx(kStatus) GoSurfaceRivetRadialHeightMax_VWrite(GoSurfaceRivetRadialHeightMax measurement, kXml xml, kXmlItem item)
{
    kObj(GoSurfaceRivetRadialHeightMax, measurement);
    kCheck(kXml_SetChild64f(xml, item, "Radius", obj->radius));
    kCheck(GoMeasurement_VWrite(measurement, xml, item));

    return kOK;
}

GoFx(k64f) GoSurfaceRivetRadialHeightMax_Radius(GoSurfaceRivetRadialHeightMax measurement)
{
    kObj(GoSurfaceRivetRadialHeightMax, measurement);
    GoSensor_SyncConfig(GoMeasurement_Sensor(measurement));

    return obj->radius;
}

GoFx(kStatus) GoSurfaceRivetRadialHeightMax_SetRadius(GoSurfaceRivetRadialHeightMax measurement, k64f value)
{
    kObj(GoSurfaceRivetRadialHeightMax, measurement);
    kCheckState(GoSensor_IsConfigurable(GoMeasurement_Sensor(measurement)));
    kCheck(GoSensor_CacheConfig(GoMeasurement_Sensor(measurement)));
    obj->radius = value;
    kCheck(GoSensor_SetConfigModified(GoMeasurement_Sensor(measurement)));

    return kOK;
}


kBeginClassEx(Go, GoSurfaceRivetRadialHeightMean)
kAddVMethod(GoSurfaceRivetRadialHeightMean, GoMeasurement, VInit)
kAddVMethod(GoSurfaceRivetRadialHeightMean, GoMeasurement, VRead)
kAddVMethod(GoSurfaceRivetRadialHeightMean, GoMeasurement, VWrite)
kEndClassEx()

GoFx(kStatus) GoSurfaceRivetRadialHeightMean_VInit(GoSurfaceRivetRadialHeightMean measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kObjR(GoSurfaceRivetRadialHeightMean, measurement);

    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_SURFACE_RIVET_RADIAL_HEIGHT_MEAN, sensor, srcTool, kTRUE, alloc));
    obj->radius = GO_MEASUREMENT_SURFACE_RIVET_DEFAULT_RADIUS;

    return kOK;
}

GoFx(kStatus) GoSurfaceRivetRadialHeightMean_VRead(GoSurfaceRivetRadialHeightMean measurement, kXml xml, kXmlItem item)
{
    kObj(GoSurfaceRivetRadialHeightMean, measurement);
    kCheck(GoMeasurement_VRead(measurement, xml, item));
    kCheck(kXml_Child64f(xml, item, "Radius", &obj->radius));

    return kOK;
}

GoFx(kStatus) GoSurfaceRivetRadialHeightMean_VWrite(GoSurfaceRivetRadialHeightMean measurement, kXml xml, kXmlItem item)
{
    kObj(GoSurfaceRivetRadialHeightMean, measurement);
    kCheck(kXml_SetChild64f(xml, item, "Radius", obj->radius));
    kCheck(GoMeasurement_VWrite(measurement, xml, item));

    return kOK;
}

GoFx(k64f) GoSurfaceRivetRadialHeightMean_Radius(GoSurfaceRivetRadialHeightMean measurement)
{
    kObj(GoSurfaceRivetRadialHeightMean, measurement);
    GoSensor_SyncConfig(GoMeasurement_Sensor(measurement));

    return obj->radius;
}

GoFx(kStatus) GoSurfaceRivetRadialHeightMean_SetRadius(GoSurfaceRivetRadialHeightMean measurement, k64f value)
{
    kObj(GoSurfaceRivetRadialHeightMean, measurement);
    kCheckState(GoSensor_IsConfigurable(GoMeasurement_Sensor(measurement)));
    kCheck(GoSensor_CacheConfig(GoMeasurement_Sensor(measurement)));
    obj->radius = value;
    kCheck(GoSensor_SetConfigModified(GoMeasurement_Sensor(measurement)));

    return kOK;
}

kBeginClassEx(Go, GoSurfaceRivetRadialHeightStdDev)
kAddVMethod(GoSurfaceRivetRadialHeightStdDev, GoMeasurement, VInit)
kAddVMethod(GoSurfaceRivetRadialHeightStdDev, GoMeasurement, VRead)
kAddVMethod(GoSurfaceRivetRadialHeightStdDev, GoMeasurement, VWrite)
kEndClassEx()

GoFx(kStatus) GoSurfaceRivetRadialHeightStdDev_VInit(GoSurfaceRivetRadialHeightStdDev measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kObjR(GoSurfaceRivetRadialHeightStdDev, measurement);

    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_SURFACE_RIVET_RADIAL_HEIGHT_STD_DEV, sensor, srcTool, kTRUE, alloc));
    obj->radius = GO_MEASUREMENT_SURFACE_RIVET_DEFAULT_RADIUS;

    return kOK;
}

GoFx(kStatus) GoSurfaceRivetRadialHeightStdDev_VRead(GoSurfaceRivetRadialHeightStdDev measurement, kXml xml, kXmlItem item)
{
    kObj(GoSurfaceRivetRadialHeightStdDev, measurement);
    kCheck(GoMeasurement_VRead(measurement, xml, item));
    kCheck(kXml_Child64f(xml, item, "Radius", &obj->radius));

    return kOK;
}

GoFx(kStatus) GoSurfaceRivetRadialHeightStdDev_VWrite(GoSurfaceRivetRadialHeightStdDev measurement, kXml xml, kXmlItem item)
{
    kObj(GoSurfaceRivetRadialHeightStdDev, measurement);
    kCheck(kXml_SetChild64f(xml, item, "Radius", obj->radius));
    kCheck(GoMeasurement_VWrite(measurement, xml, item));

    return kOK;
}

GoFx(k64f) GoSurfaceRivetRadialHeightStdDev_Radius(GoSurfaceRivetRadialHeightStdDev measurement)
{
    kObj(GoSurfaceRivetRadialHeightStdDev, measurement);
    GoSensor_SyncConfig(GoMeasurement_Sensor(measurement));

    return obj->radius;
}

GoFx(kStatus) GoSurfaceRivetRadialHeightStdDev_SetRadius(GoSurfaceRivetRadialHeightStdDev measurement, k64f value)
{
    kObj(GoSurfaceRivetRadialHeightStdDev, measurement);
    kCheckState(GoSensor_IsConfigurable(GoMeasurement_Sensor(measurement)));
    kCheck(GoSensor_CacheConfig(GoMeasurement_Sensor(measurement)));
    obj->radius = value;
    kCheck(GoSensor_SetConfigModified(GoMeasurement_Sensor(measurement)));

    return kOK;
}


kBeginClassEx(Go, GoSurfaceRivetRadialSlopeMin)
kAddVMethod(GoSurfaceRivetRadialSlopeMin, GoMeasurement, VInit)
kAddVMethod(GoSurfaceRivetRadialSlopeMin, GoMeasurement, VRead)
kAddVMethod(GoSurfaceRivetRadialSlopeMin, GoMeasurement, VWrite)
kEndClassEx()

GoFx(kStatus) GoSurfaceRivetRadialSlopeMin_VInit(GoSurfaceRivetRadialSlopeMin measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kObjR(GoSurfaceRivetRadialSlopeMin, measurement);

    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_SURFACE_RIVET_RADIAL_SLOPE_MIN, sensor, srcTool, kTRUE, alloc));
    obj->radius = GO_MEASUREMENT_SURFACE_RIVET_DEFAULT_RADIUS;

    return kOK;
}

GoFx(kStatus) GoSurfaceRivetRadialSlopeMin_VRead(GoSurfaceRivetRadialSlopeMin measurement, kXml xml, kXmlItem item)
{
    kObj(GoSurfaceRivetRadialSlopeMin, measurement);
    kCheck(GoMeasurement_VRead(measurement, xml, item));
    kCheck(kXml_Child64f(xml, item, "Radius", &obj->radius));

    return kOK;
}

GoFx(kStatus) GoSurfaceRivetRadialSlopeMin_VWrite(GoSurfaceRivetRadialSlopeMin measurement, kXml xml, kXmlItem item)
{
    kObj(GoSurfaceRivetRadialSlopeMin, measurement);
    kCheck(kXml_SetChild64f(xml, item, "Radius", obj->radius));
    kCheck(GoMeasurement_VWrite(measurement, xml, item));

    return kOK;
}

GoFx(k64f) GoSurfaceRivetRadialSlopeMin_Radius(GoSurfaceRivetRadialSlopeMin measurement)
{
    kObj(GoSurfaceRivetRadialSlopeMin, measurement);
    GoSensor_SyncConfig(GoMeasurement_Sensor(measurement));

    return obj->radius;
}

GoFx(kStatus) GoSurfaceRivetRadialSlopeMin_SetRadius(GoSurfaceRivetRadialSlopeMin measurement, k64f value)
{
    kObj(GoSurfaceRivetRadialSlopeMin, measurement);
    kCheckState(GoSensor_IsConfigurable(GoMeasurement_Sensor(measurement)));
    kCheck(GoSensor_CacheConfig(GoMeasurement_Sensor(measurement)));
    obj->radius = value;
    kCheck(GoSensor_SetConfigModified(GoMeasurement_Sensor(measurement)));

    return kOK;
}

kBeginClassEx(Go, GoSurfaceRivetRadialSlopeMax)
kAddVMethod(GoSurfaceRivetRadialSlopeMax, GoMeasurement, VInit)
kAddVMethod(GoSurfaceRivetRadialSlopeMax, GoMeasurement, VRead)
kAddVMethod(GoSurfaceRivetRadialSlopeMax, GoMeasurement, VWrite)
kEndClassEx()

GoFx(kStatus) GoSurfaceRivetRadialSlopeMax_VInit(GoSurfaceRivetRadialSlopeMax measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kObjR(GoSurfaceRivetRadialSlopeMax, measurement);

    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_SURFACE_RIVET_RADIAL_SLOPE_MAX, sensor, srcTool, kTRUE, alloc));
    obj->radius = GO_MEASUREMENT_SURFACE_RIVET_DEFAULT_RADIUS;

    return kOK;
}

GoFx(kStatus) GoSurfaceRivetRadialSlopeMax_VRead(GoSurfaceRivetRadialSlopeMax measurement, kXml xml, kXmlItem item)
{
    kObj(GoSurfaceRivetRadialSlopeMax, measurement);
    kCheck(GoMeasurement_VRead(measurement, xml, item));
    kCheck(kXml_Child64f(xml, item, "Radius", &obj->radius));

    return kOK;
}

GoFx(kStatus) GoSurfaceRivetRadialSlopeMax_VWrite(GoSurfaceRivetRadialSlopeMax measurement, kXml xml, kXmlItem item)
{
    kObj(GoSurfaceRivetRadialSlopeMax, measurement);
    kCheck(kXml_SetChild64f(xml, item, "Radius", obj->radius));
    kCheck(GoMeasurement_VWrite(measurement, xml, item));

    return kOK;
}

GoFx(k64f) GoSurfaceRivetRadialSlopeMax_Radius(GoSurfaceRivetRadialSlopeMax measurement)
{
    kObj(GoSurfaceRivetRadialSlopeMax, measurement);
    GoSensor_SyncConfig(GoMeasurement_Sensor(measurement));

    return obj->radius;
}

GoFx(kStatus) GoSurfaceRivetRadialSlopeMax_SetRadius(GoSurfaceRivetRadialSlopeMax measurement, k64f value)
{
    kObj(GoSurfaceRivetRadialSlopeMax, measurement);
    kCheckState(GoSensor_IsConfigurable(GoMeasurement_Sensor(measurement)));
    kCheck(GoSensor_CacheConfig(GoMeasurement_Sensor(measurement)));
    obj->radius = value;
    kCheck(GoSensor_SetConfigModified(GoMeasurement_Sensor(measurement)));

    return kOK;
}


kBeginClassEx(Go, GoSurfaceRivetRadialSlopeMean)
kAddVMethod(GoSurfaceRivetRadialSlopeMean, GoMeasurement, VInit)
kAddVMethod(GoSurfaceRivetRadialSlopeMean, GoMeasurement, VRead)
kAddVMethod(GoSurfaceRivetRadialSlopeMean, GoMeasurement, VWrite)
kEndClassEx()

GoFx(kStatus) GoSurfaceRivetRadialSlopeMean_VInit(GoSurfaceRivetRadialSlopeMean measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kObjR(GoSurfaceRivetRadialSlopeMean, measurement);

    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_SURFACE_RIVET_RADIAL_SLOPE_MEAN, sensor, srcTool, kTRUE, alloc));
    obj->radius = GO_MEASUREMENT_SURFACE_RIVET_DEFAULT_RADIUS;

    return kOK;
}

GoFx(kStatus) GoSurfaceRivetRadialSlopeMean_VRead(GoSurfaceRivetRadialSlopeMean measurement, kXml xml, kXmlItem item)
{
    kObj(GoSurfaceRivetRadialSlopeMean, measurement);
    kCheck(GoMeasurement_VRead(measurement, xml, item));
    kCheck(kXml_Child64f(xml, item, "Radius", &obj->radius));

    return kOK;
}

GoFx(kStatus) GoSurfaceRivetRadialSlopeMean_VWrite(GoSurfaceRivetRadialSlopeMean measurement, kXml xml, kXmlItem item)
{
    kObj(GoSurfaceRivetRadialSlopeMean, measurement);
    kCheck(kXml_SetChild64f(xml, item, "Radius", obj->radius));
    kCheck(GoMeasurement_VWrite(measurement, xml, item));

    return kOK;
}

GoFx(k64f) GoSurfaceRivetRadialSlopeMean_Radius(GoSurfaceRivetRadialSlopeMean measurement)
{
    kObj(GoSurfaceRivetRadialSlopeMean, measurement);
    GoSensor_SyncConfig(GoMeasurement_Sensor(measurement));

    return obj->radius;
}

GoFx(kStatus) GoSurfaceRivetRadialSlopeMean_SetRadius(GoSurfaceRivetRadialSlopeMean measurement, k64f value)
{
    kObj(GoSurfaceRivetRadialSlopeMean, measurement);
    kCheckState(GoSensor_IsConfigurable(GoMeasurement_Sensor(measurement)));
    kCheck(GoSensor_CacheConfig(GoMeasurement_Sensor(measurement)));
    obj->radius = value;
    kCheck(GoSensor_SetConfigModified(GoMeasurement_Sensor(measurement)));

    return kOK;
}

kBeginClassEx(Go, GoSurfaceRivetRadialSlopeStdDev)
kAddVMethod(GoSurfaceRivetRadialSlopeStdDev, GoMeasurement, VInit)
kAddVMethod(GoSurfaceRivetRadialSlopeStdDev, GoMeasurement, VRead)
kAddVMethod(GoSurfaceRivetRadialSlopeStdDev, GoMeasurement, VWrite)
kEndClassEx()

GoFx(kStatus) GoSurfaceRivetRadialSlopeStdDev_VInit(GoSurfaceRivetRadialSlopeStdDev measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kObjR(GoSurfaceRivetRadialSlopeStdDev, measurement);

    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_SURFACE_RIVET_RADIAL_SLOPE_STD_DEV, sensor, srcTool, kTRUE, alloc));
    obj->radius = GO_MEASUREMENT_SURFACE_RIVET_DEFAULT_RADIUS;

    return kOK;
}

GoFx(kStatus) GoSurfaceRivetRadialSlopeStdDev_VRead(GoSurfaceRivetRadialSlopeStdDev measurement, kXml xml, kXmlItem item)
{
    kObj(GoSurfaceRivetRadialSlopeStdDev, measurement);
    kCheck(GoMeasurement_VRead(measurement, xml, item));
    kCheck(kXml_Child64f(xml, item, "Radius", &obj->radius));

    return kOK;
}

GoFx(kStatus) GoSurfaceRivetRadialSlopeStdDev_VWrite(GoSurfaceRivetRadialSlopeStdDev measurement, kXml xml, kXmlItem item)
{
    kObj(GoSurfaceRivetRadialSlopeStdDev, measurement);
    kCheck(kXml_SetChild64f(xml, item, "Radius", obj->radius));
    kCheck(GoMeasurement_VWrite(measurement, xml, item));

    return kOK;
}

GoFx(k64f) GoSurfaceRivetRadialSlopeStdDev_Radius(GoSurfaceRivetRadialSlopeStdDev measurement)
{
    kObj(GoSurfaceRivetRadialSlopeStdDev, measurement);
    GoSensor_SyncConfig(GoMeasurement_Sensor(measurement));

    return obj->radius;
}

GoFx(kStatus) GoSurfaceRivetRadialSlopeStdDev_SetRadius(GoSurfaceRivetRadialSlopeStdDev measurement, k64f value)
{
    kObj(GoSurfaceRivetRadialSlopeStdDev, measurement);
    kCheckState(GoSensor_IsConfigurable(GoMeasurement_Sensor(measurement)));
    kCheck(GoSensor_CacheConfig(GoMeasurement_Sensor(measurement)));
    obj->radius = value;
    kCheck(GoSensor_SetConfigModified(GoMeasurement_Sensor(measurement)));

    return kOK;
}

kBeginClassEx(Go, GoSurfaceStudBaseX)
    kAddVMethod(GoSurfaceStudBaseX, GoMeasurement, VInit)
kEndClassEx()

GoFx(kStatus) GoSurfaceStudBaseX_VInit(GoSurfaceStudBaseX measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_SURFACE_STUD_BASE_X, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


kBeginClassEx(Go, GoSurfaceStudBaseY)
    kAddVMethod(GoSurfaceStudBaseY, GoMeasurement, VInit)
kEndClassEx()

GoFx(kStatus) GoSurfaceStudBaseY_VInit(GoSurfaceStudBaseY measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_SURFACE_STUD_BASE_Y, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


kBeginClassEx(Go, GoSurfaceStudBaseZ)
    kAddVMethod(GoSurfaceStudBaseZ, GoMeasurement, VInit)
kEndClassEx()

GoFx(kStatus) GoSurfaceStudBaseZ_VInit(GoSurfaceStudBaseZ measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_SURFACE_STUD_BASE_Z, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


kBeginClassEx(Go, GoSurfaceStudTipX)
    kAddVMethod(GoSurfaceStudTipX, GoMeasurement, VInit)
kEndClassEx()


GoFx(kStatus) GoSurfaceStudTipX_VInit(GoSurfaceStudTipX measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_SURFACE_STUD_TIP_X, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


kBeginClassEx(Go, GoSurfaceStudTipY)
    kAddVMethod(GoSurfaceStudTipY, GoMeasurement, VInit)
kEndClassEx()

GoFx(kStatus) GoSurfaceStudTipY_VInit(GoSurfaceStudTipY measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_SURFACE_STUD_TIP_Y, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


kBeginClassEx(Go, GoSurfaceStudTipZ)
    kAddVMethod(GoSurfaceStudTipZ, GoMeasurement, VInit)
kEndClassEx()

GoFx(kStatus) GoSurfaceStudTipZ_VInit(GoSurfaceStudTipZ measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_SURFACE_STUD_TIP_Z, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


kBeginClassEx(Go, GoSurfaceStudRadius)
    kAddVMethod(GoSurfaceStudRadius, GoMeasurement, VInit)
    kAddVMethod(GoSurfaceStudRadius, GoMeasurement, VRead)
    kAddVMethod(GoSurfaceStudRadius, GoMeasurement, VWrite)
kEndClassEx()


GoFx(kStatus) GoSurfaceStudRadius_VInit(GoSurfaceStudRadius measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kObjR(GoSurfaceStudRadius, measurement);

    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_SURFACE_STUD_RADIUS, sensor, srcTool, kTRUE, alloc));

    obj->radiusOffset = 0.0;

    return kOK;
}

GoFx(kStatus) GoSurfaceStudRadius_VRead(GoSurfaceStudRadius measurement, kXml xml, kXmlItem item)
{
    kObj(GoSurfaceStudRadius, measurement);
    kCheck(GoMeasurement_VRead(measurement, xml, item));
    kCheck(kXml_Child64f(xml, item, "RadiusOffset", &obj->radiusOffset));

    return kOK;
}

GoFx(kStatus) GoSurfaceStudRadius_VWrite(GoSurfaceStudRadius measurement, kXml xml, kXmlItem item)
{
    kObj(GoSurfaceStudRadius, measurement);
    kCheck(kXml_SetChild64f(xml, item, "RadiusOffset", obj->radiusOffset));
    kCheck(GoMeasurement_VWrite(measurement, xml, item));

    return kOK;
}

GoFx(k64f) GoSurfaceStudRadius_RadiusOffset(GoSurfaceStudRadius measurement)
{
    kObj(GoSurfaceStudRadius, measurement);
    GoSensor_SyncConfig(GoMeasurement_Sensor(measurement));

    return obj->radiusOffset;
}

GoFx(kStatus) GoSurfaceStudRadius_SetRadiusOffset(GoSurfaceStudRadius measurement, k64f offset)
{
    kObj(GoSurfaceStudRadius, measurement);
    kCheckState(GoSensor_IsConfigurable(GoMeasurement_Sensor(measurement)));
    kCheck(GoSensor_CacheConfig(GoMeasurement_Sensor(measurement)));
    obj->radiusOffset = offset;
    kCheck(GoSensor_SetConfigModified(GoMeasurement_Sensor(measurement)));

    return kOK;
}

kBeginClassEx(Go, GoSurfaceVolumeVolume)
    kAddVMethod(GoSurfaceVolumeVolume, GoMeasurement, VInit)
kEndClassEx()

GoFx(kStatus) GoSurfaceVolumeVolume_VInit(GoSurfaceVolumeVolume measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kCheck(GoMeasurement_Init(measurement, type,  GO_MEASUREMENT_SURFACE_VOLUME_VOLUME, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


kBeginClassEx(Go, GoSurfaceVolumeArea)
    kAddVMethod(GoSurfaceVolumeArea, GoMeasurement, VInit)
kEndClassEx()

GoFx(kStatus) GoSurfaceVolumeArea_VInit(GoSurfaceVolumeArea measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_SURFACE_VOLUME_AREA, sensor, srcTool, kTRUE, alloc));

    return kOK;
}


kBeginClassEx(Go, GoSurfaceVolumeThickness)
    kAddVMethod(GoSurfaceVolumeThickness, GoMeasurement, VInit)
    kAddVMethod(GoSurfaceVolumeThickness, GoMeasurement, VRead)
    kAddVMethod(GoSurfaceVolumeThickness, GoMeasurement, VWrite)
kEndClassEx()

GoFx(kStatus) GoSurfaceVolumeThickness_VInit(GoSurfaceVolumeThickness measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kObjR(GoSurfaceVolumeThickness, measurement);

    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_SURFACE_VOLUME_THICKNESS, sensor, srcTool, kTRUE, alloc));

    obj->location = GO_SURFACE_LOCATION_TYPE_MAX;

    return kOK;
}

GoFx(kStatus) GoSurfaceVolumeThickness_VRead(GoSurfaceVolumeThickness measurement, kXml xml, kXmlItem item)
{
    kObj(GoSurfaceVolumeThickness, measurement);
    kCheck(GoMeasurement_VRead(measurement, xml, item));
    kCheck(kXml_Child32s(xml, item, "Location", &obj->location));

    return kOK;
}

GoFx(kStatus) GoSurfaceVolumeThickness_VWrite(GoSurfaceVolumeThickness measurement, kXml xml, kXmlItem item)
{
    kObj(GoSurfaceVolumeThickness, measurement);

    kCheck(GoMeasurement_VWrite(measurement, xml, item));
    kCheck(kXml_SetChild32s(xml, item, "Location", obj->location));

    return kOK;
}

GoFx(GoSurfaceLocation) GoSurfaceVolumeThickness_Location(GoSurfaceVolumeThickness measurement)
{
    kObj(GoSurfaceVolumeThickness, measurement);
    GoSensor_SyncConfig(GoMeasurement_Sensor(measurement));

    return obj->location;
}

GoFx(kStatus) GoSurfaceVolumeThickness_SetLocation(GoSurfaceVolumeThickness measurement, GoSurfaceLocation location)
{
    kObj(GoSurfaceVolumeThickness, measurement);
    kCheckState(GoSensor_IsConfigurable(GoMeasurement_Sensor(measurement)));
    kCheck(GoSensor_CacheConfig(GoMeasurement_Sensor(measurement)));
    obj->location = location;
    kCheck(GoSensor_SetConfigModified(GoMeasurement_Sensor(measurement)));

    return kOK;
}


/* Script tool measurement */

kBeginClassEx(Go, GoScriptOutput)
    kAddVMethod(GoScriptOutput, GoMeasurement, VInit)
    kAddVMethod(GoScriptOutput, GoMeasurement, VRead)
    kAddVMethod(GoScriptOutput, GoMeasurement, VWrite)
kEndClassEx()

GoFx(kStatus) GoScriptOutput_VInit(GoScriptOutput measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kCheck(GoMeasurement_Init(measurement, type, GO_MEASUREMENT_SCRIPT_OUTPUT, sensor, srcTool, kFALSE, alloc));

    return kOK;
}

GoFx(kStatus) GoScriptOutput_VRead(GoScriptOutput measurement, kXml xml, kXmlItem item)
{
    kCheck(GoMeasurement_VRead(measurement, xml, item));

    return kOK;
}

GoFx(kStatus) GoScriptOutput_VWrite(GoScriptOutput measurement, kXml xml, kXmlItem item)
{
    kCheck(GoMeasurement_VWrite(measurement, xml, item));

    return kOK;
}

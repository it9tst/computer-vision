/**
 * @file    GoSurfaceTools.x.h
 *
 * @internal
 * Copyright (C) 2016-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef GO_SURFACE_TOOLS_X_H
#define GO_SURFACE_TOOLS_X_H

#include <kApi/Data/kXml.h>
#include <GoSdk/Tools/GoTool.h>

// Define Surface tool names, ordered alphabetically.
#define GO_SURFACE_TOOL_NAME_BOUNDING_BOX       "SurfaceBoundingBox"
#define GO_SURFACE_TOOL_NAME_COUNTERSUNK_HOLE   "SurfaceCsHole"
#define GO_SURFACE_TOOL_NAME_DIMENSION          "SurfaceDimension"
#define GO_SURFACE_TOOL_NAME_ELLIPSE            "SurfaceEllipse"
#define GO_SURFACE_TOOL_NAME_HOLE               "SurfaceHole"
#define GO_SURFACE_TOOL_NAME_OPENING            "SurfaceOpening"
#define GO_SURFACE_TOOL_NAME_PLANE              "SurfacePlane"
#define GO_SURFACE_TOOL_NAME_POSITION           "SurfacePosition"
#define GO_SURFACE_TOOL_NAME_RIVET              "SurfaceRivet"
#define GO_SURFACE_TOOL_NAME_STUD               "SurfaceStud"
#define GO_SURFACE_TOOL_NAME_VOLUME             "SurfaceVolume"

typedef struct GoSurfaceToolClass
{
    GoToolClass base;

    kArrayList streamOptions;
    GoDataStream stream;

    kArrayList sourceOptions;
    GoDataSource source;

    kArrayList xAnchorOptions;
    k32s xAnchor;
    kArrayList yAnchorOptions;
    k32s yAnchor;
    kArrayList zAnchorOptions;
    k32s zAnchor;
    kArrayList zAngleAnchorOptions;
    k32s zAngleAnchor;
} GoSurfaceToolClass;

kDeclareClassEx(Go, GoSurfaceTool, GoTool)

GoFx(kStatus) GoSurfaceTool_Init(GoSurfaceTool tool, kType type, GoToolType typeId, kObject sensor, kAlloc alloc);
GoFx(kStatus) GoSurfaceTool_VRelease(GoSurfaceTool tool);
GoFx(kStatus) GoSurfaceTool_Read(GoSurfaceTool tool, kXml xml, kXmlItem item);
GoFx(kStatus) GoSurfaceTool_Write(GoSurfaceTool tool, kXml xml, kXmlItem item);

typedef struct GoSurfaceBoxClass
{
    GoSurfaceToolClass base;
    kBool zRotationEnabled;
    kBool regionEnabled;
    GoRegion3d region;
    GoBoxAsymmetryType asymDetectType;
} GoSurfaceBoxClass;

kDeclareClassEx(Go, GoSurfaceBox, GoSurfaceTool)

GoFx(kStatus) GoSurfaceBox_Construct(GoSurfaceBox* tool, kObject sensor, kAlloc allocator);
GoFx(kStatus) GoSurfaceBox_VInit(GoSurfaceBox tool, kType type, kObject sensor, kAlloc alloc);
GoFx(kStatus) GoSurfaceBox_VRelease(GoSurfaceBox tool);
GoFx(kStatus) GoSurfaceBox_VRead(GoSurfaceBox tool, kXml xml, kXmlItem item);
GoFx(kStatus) GoSurfaceBox_VWrite(GoSurfaceBox tool, kXml xml, kXmlItem item);

typedef struct GoSurfaceCountersunkHoleClass
{
    GoSurfaceToolClass base;

    GoSurfaceCountersunkHoleShape shape;
    k64f nominalBevelAngle;
    k64f bevelAngleTolerance;
    k64f nominalOuterRadius;
    k64f outerRadiusTolerance;
    k64f nominalInnerRadius;
    k64f innerRadiusTolerance;
    k64f bevelRadiusOffset;

    kBool partialDetectionEnabled;
    kBool regionEnabled;
    GoRegion3d region;
    kBool refRegionsEnabled;
    kSize refRegionCount;
    GoSurfaceRegion2d refRegions[GO_SURFACE_COUNTERSUNK_HOLE_MAX_REF_REGIONS];

    kBool autoTiltEnabled;
    k64f tiltXAngle;
    k64f tiltYAngle;

    kBool curveFitEnabled;
    k64f curveOrientation;

    kBool planeFitRangeEnabled;
    k64f planeFitRange;
} GoSurfaceCountersunkHoleClass;

kDeclareClassEx(Go, GoSurfaceCountersunkHole, GoSurfaceTool)

GoFx(kStatus) GoSurfaceCountersunkHole_Construct(GoSurfaceCountersunkHole* tool, kObject sensor, kAlloc allocator);
GoFx(kStatus) GoSurfaceCountersunkHole_VInit(GoSurfaceCountersunkHole tool, kType type, kObject sensor, kAlloc alloc);
GoFx(kStatus) GoSurfaceCountersunkHole_VRelease(GoSurfaceCountersunkHole tool);
GoFx(kStatus) GoSurfaceCountersunkHole_VRead(GoSurfaceCountersunkHole tool, kXml xml, kXmlItem item);
GoFx(kStatus) GoSurfaceCountersunkHole_VWrite(GoSurfaceCountersunkHole tool, kXml xml, kXmlItem item);


/**
 * @deprecated Sets the bevel angle tolerance.
 *
 * @public           @memberof GoSurfaceCountersunkHole
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoSurfaceCountersunkHole object.
 * @param    value   The value to set.
 * @return           Operation status.
 */
GoFx(kStatus) GoSurfaceCountersunkHole_SetBevelAngleTolerance(GoSurfaceCountersunkHole tool, k64f value);

/**
 * @deprecated Returns the bevel angle tolerance.
 *
 * @public           @memberof GoSurfaceCountersunkHole
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoSurfaceCountersunkHole object.
 * @return           The bevel angle tolerance.
 */
GoFx(k64f) GoSurfaceCountersunkHole_BevelAngleTolerance(GoSurfaceCountersunkHole tool);


/**
 * @deprecated Sets the inner radius tolerance.
 *
 * @public           @memberof GoSurfaceCountersunkHole
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoSurfaceCountersunkHole object.
 * @param    value   The value to set.
 * @return           Operation status.
 */
GoFx(kStatus) GoSurfaceCountersunkHole_SetInnerRadiusTolerance(GoSurfaceCountersunkHole tool, k64f value);

/**
 * @deprecated Returns the inner radius tolerance.
 *
 * @public           @memberof GoSurfaceCountersunkHole
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoSurfaceCountersunkHole object.
 * @return           The inner radius tolerance.
 */
GoFx(k64f) GoSurfaceCountersunkHole_InnerRadiusTolerance(GoSurfaceCountersunkHole tool);

/**
 * @deprecated Sets the outer radius tolerance.
 *
 * @public           @memberof GoSurfaceCountersunkHole
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoSurfaceCountersunkHole object.
 * @param    value   The value to set.
 * @return           Operation status.
 */
GoFx(kStatus) GoSurfaceCountersunkHole_SetOuterRadiusTolerance(GoSurfaceCountersunkHole tool, k64f value);

/**
 * @deprecated Returns the outer radius tolerance.
 *
 * @public           @memberof GoSurfaceCountersunkHole
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoSurfaceCountersunkHole object.
 * @return           The outer radius tolerance.
 */
GoFx(k64f) GoSurfaceCountersunkHole_OuterRadiusTolerance(GoSurfaceCountersunkHole tool);


typedef struct GoSurfaceDimClass
{
    GoSurfaceToolClass base;
    GoSurfaceFeature refFeature;
    GoSurfaceFeature feature;
} GoSurfaceDimClass;

kDeclareClassEx(Go, GoSurfaceDim, GoSurfaceTool)

GoFx(kStatus) GoSurfaceDim_Construct(GoSurfaceDim* tool, kObject sensor, kAlloc allocator);
GoFx(kStatus) GoSurfaceDim_VInit(GoSurfaceDim tool, kType type, kObject sensor, kAlloc alloc);
GoFx(kStatus) GoSurfaceDim_VRelease(GoSurfaceDim tool);
GoFx(kStatus) GoSurfaceDim_VRead(GoSurfaceDim tool, kXml xml, kXmlItem item);
GoFx(kStatus) GoSurfaceDim_VWrite(GoSurfaceDim tool, kXml xml, kXmlItem item);


typedef struct GoSurfaceEllipseClass
{
    GoSurfaceToolClass base;
    kBool regionEnabled;
    GoRegion3d region;
    GoEllipseAsymmetryType asymDetectType;
} GoSurfaceEllipseClass;

kDeclareClassEx(Go, GoSurfaceEllipse, GoSurfaceTool)

GoFx(kStatus) GoSurfaceEllipse_Construct(GoSurfaceEllipse* tool, kObject sensor, kAlloc allocator);
GoFx(kStatus) GoSurfaceEllipse_VInit(GoSurfaceEllipse tool, kType type, kObject sensor, kAlloc alloc);
GoFx(kStatus) GoSurfaceEllipse_VRelease(GoSurfaceEllipse tool);
GoFx(kStatus) GoSurfaceEllipse_VRead(GoSurfaceEllipse tool, kXml xml, kXmlItem item);
GoFx(kStatus) GoSurfaceEllipse_VWrite(GoSurfaceEllipse tool, kXml xml, kXmlItem item);


typedef struct GoSurfaceHoleClass
{
    GoSurfaceToolClass base;

    k64f nominalRadius;
    k64f radiusTolerance;
    kBool partialDetectionEnabled;
    kBool regionEnabled;
    GoRegion3d region;
    kBool refRegionsEnabled;
    kSize refRegionCount;
    GoSurfaceRegion2d refRegions[GO_SURFACE_HOLE_MAX_REF_REGIONS];
    k64f tiltXAngle;
    k64f tiltYAngle;
    kBool autoTiltEnabled;

    kBool depthLimitEnabled;
    k64f depthLimit;
} GoSurfaceHoleClass;

kDeclareClassEx(Go, GoSurfaceHole, GoSurfaceTool)

GoFx(kStatus) GoSurfaceHole_Construct(GoSurfaceHole* tool, kObject sensor, kAlloc allocator);
GoFx(kStatus) GoSurfaceHole_VInit(GoSurfaceHole tool, kType type, kObject sensor, kAlloc alloc);
GoFx(kStatus) GoSurfaceHole_VRelease(GoSurfaceHole tool);
GoFx(kStatus) GoSurfaceHole_VRead(GoSurfaceHole tool, kXml xml, kXmlItem item);
GoFx(kStatus) GoSurfaceHole_VWrite(GoSurfaceHole tool, kXml xml, kXmlItem item);


typedef struct GoSurfaceOpeningClass
{
    GoSurfaceToolClass base;

    GoSurfaceOpeningType type;
    k64f nominalWidth;
    k64f nominalLength;
    k64f nominalAngle;
    k64f nominalRadius;
    k64f widthTolerance;
    k64f lengthTolerance;
    k64f angleTolerance;
    kBool partialDetectionEnabled;
    kBool regionEnabled;
    GoRegion3d region;
    kBool refRegionsEnabled;
    kSize refRegionCount;
    GoSurfaceRegion2d refRegions[GO_SURFACE_OPENING_MAX_REF_REGIONS];
    kBool autoTiltEnabled;
    k64f tiltXAngle;
    k64f tiltYAngle;

    kBool depthLimitEnabled;
    k64f depthLimit;
} GoSurfaceOpeningClass;

kDeclareClassEx(Go, GoSurfaceOpening, GoSurfaceTool)

GoFx(kStatus) GoSurfaceOpening_Construct(GoSurfaceOpening* tool, kObject sensor, kAlloc allocator);
GoFx(kStatus) GoSurfaceOpening_VInit(GoSurfaceOpening tool, kType type, kObject sensor, kAlloc alloc);
GoFx(kStatus) GoSurfaceOpening_VRelease(GoSurfaceOpening tool);
GoFx(kStatus) GoSurfaceOpening_VRead(GoSurfaceOpening tool, kXml xml, kXmlItem item);
GoFx(kStatus) GoSurfaceOpening_VWrite(GoSurfaceOpening tool, kXml xml, kXmlItem item);


typedef struct GoSurfacePlaneClass
{
    GoSurfaceToolClass base;

    kBool regionsEnabled;
    kSize regionCount;
    GoRegion3d regions[GO_SURFACE_PLANE_MAX_REGIONS];
} GoSurfacePlaneClass;

kDeclareClassEx(Go, GoSurfacePlane, GoSurfaceTool)

GoFx(kStatus) GoSurfacePlane_Construct(GoSurfacePlane* tool, kObject sensor, kAlloc allocator);
GoFx(kStatus) GoSurfacePlane_VInit(GoSurfacePlane tool, kType type, kObject sensor, kAlloc alloc);
GoFx(kStatus) GoSurfacePlane_VRelease(GoSurfacePlane tool);
GoFx(kStatus) GoSurfacePlane_VRead(GoSurfacePlane tool, kXml xml, kXmlItem item);
GoFx(kStatus) GoSurfacePlane_VWrite(GoSurfacePlane tool, kXml xml, kXmlItem item);

typedef struct GoSurfacePositionClass
{
    GoSurfaceToolClass base;
    GoSurfaceFeature feature;
    kBool regionEnabled;
    GoRegion3d region;
} GoSurfacePositionClass;

kDeclareClassEx(Go, GoSurfacePosition, GoSurfaceTool)

GoFx(kStatus) GoSurfacePosition_Construct(GoSurfacePosition* tool, kObject sensor, kAlloc allocator);
GoFx(kStatus) GoSurfacePosition_VInit(GoSurfacePosition tool, kType type, kObject sensor, kAlloc alloc);
GoFx(kStatus) GoSurfacePosition_VRelease(GoSurfacePosition tool);
GoFx(kStatus) GoSurfacePosition_VRead(GoSurfacePosition tool, kXml xml, kXmlItem item);
GoFx(kStatus) GoSurfacePosition_VWrite(GoSurfacePosition tool, kXml xml, kXmlItem item);


typedef struct GoSurfaceStudClass
{
    GoSurfaceToolClass base;
    k64f studRadius;
    k64f studHeight;
    k64f baseHeight;
    k64f tipHeight;

    kBool regionEnabled;
    GoRegion3d region;

    kBool refRegionsEnabled;
    kSize refRegionCount;
    GoSurfaceRegion2d refRegions[GO_SURFACE_STUD_MAX_REF_REGIONS];
    kBool autoTiltEnabled;
    k64f tiltXAngle;
    k64f tiltYAngle;
} GoSurfaceStudClass;

kDeclareClassEx(Go, GoSurfaceStud, GoSurfaceTool)

GoFx(kStatus) GoSurfaceStud_Construct(GoSurfaceStud* tool, kObject sensor, kAlloc allocator);
GoFx(kStatus) GoSurfaceStud_VInit(GoSurfaceStud tool, kType type, kObject sensor, kAlloc alloc);
GoFx(kStatus) GoSurfaceStud_VRelease(GoSurfaceStud tool);
GoFx(kStatus) GoSurfaceStud_VRead(GoSurfaceStud tool, kXml xml, kXmlItem item);
GoFx(kStatus) GoSurfaceStud_VWrite(GoSurfaceStud tool, kXml xml, kXmlItem item);


typedef struct GoSurfaceVolumeClass
{
    GoSurfaceToolClass base;

    kBool regionEnabled;
    GoRegion3d region;
} GoSurfaceVolumeClass;

kDeclareClassEx(Go, GoSurfaceVolume, GoSurfaceTool)

GoFx(kStatus) GoSurfaceVolume_Construct(GoSurfaceVolume* tool, kObject sensor, kAlloc allocator);
GoFx(kStatus) GoSurfaceVolume_VInit(GoSurfaceVolume tool, kType type, kObject sensor, kAlloc alloc);
GoFx(kStatus) GoSurfaceVolume_VRelease(GoSurfaceVolume tool);
GoFx(kStatus) GoSurfaceVolume_VRead(GoSurfaceVolume tool, kXml xml, kXmlItem item);
GoFx(kStatus) GoSurfaceVolume_VWrite(GoSurfaceVolume tool, kXml xml, kXmlItem item);


/**
 * Gets the current nominal height value.
 *
 * @public                  @memberof GoSurfaceRivet
 * @param    tool           GoSurfaceRivet object.
 * @return                  The nominal height value.
 */
GoFx(k64f) GoSurfaceRivet_NominalHeight(GoSurfaceRivet tool);

/**
 * Sets the nominal height value.
 *
 * @public                  @memberof GoSurfaceRivet
 * @param    tool           GoSurfaceRivet object.
 * @param    nominalHeight  Nominal height value to set.
 * @return                  Operation status.
 */
GoFx(kStatus) GoSurfaceRivet_SetNominalHeight(GoSurfaceRivet tool, k64f nominalHeight);


/**
 * Gets the current nominal radius value.
 *
 * @public                  @memberof GoSurfaceRivet
 * @param    tool           GoSurfaceRivet object.
 * @return                  The nominal radius value.
 */
GoFx(k64f) GoSurfaceRivet_NominalRadius(GoSurfaceRivet tool);

/**
 * Sets the nominal radius value.
 *
 * @public                  @memberof GoSurfaceRivet
 * @param    tool           GoSurfaceRivet object.
 * @param    nominalRadius  Nominal radius value to set.
 * @return                  Operation status.
 */
GoFx(kStatus) GoSurfaceRivet_SetNominalRadius(GoSurfaceRivet tool, k64f nominalRadius);

/**
 * Gets the current radius tolerance value.
 *
 * @public                  @memberof GoSurfaceRivet
 * @param    tool           GoSurfaceRivet object.
 * @return                  The radius tolerance value.
 */
GoFx(k64f) GoSurfaceRivet_RadiusTolerance(GoSurfaceRivet tool);

/**
 * Sets the radius tolerance value.
 *
 * @public                      @memberof GoSurfaceRivet
 * @param    tool               GoSurfaceRivet object.
 * @param    radiusTolerance    The radius tolerance value to set.
 * @return                      Operation status.
 */
GoFx(kStatus) GoSurfaceRivet_SetRadiusTolerance(GoSurfaceRivet tool, k64f radiusTolerance);

/**
 * Gets the enabled state of partial detection.
 *
 * @public                  @memberof GoSurfaceRivet
 * @param    tool           GoSurfaceRivet object.
 * @return                  kTRUE if enabled and kFALSE if disabled.
 */
GoFx(kBool) GoSurfaceRivet_PartialDetectionEnabled(GoSurfaceRivet tool);

/**
 * Sets the enabled state of partial detection.
 *
 * @public                  @memberof GoSurfaceRivet
 * @param    tool           GoSurfaceRivet object.
 * @param    enable         kTRUE to enable partial detection and kFALSE to disable it.
 * @return                  Operation status.
 */
GoFx(kStatus) GoSurfaceRivet_EnablePartialDetection(GoSurfaceRivet tool, kBool enable);

/**
 * Gets the enabled state of the tool region.
 *
 * @public                  @memberof GoSurfaceRivet
 * @param    tool           GoSurfaceRivet object.
 * @return                  kTRUE if enabled and kFALSE if disabled.
 */
GoFx(kBool) GoSurfaceRivet_RegionEnabled(GoSurfaceRivet tool);

/**
 * Sets the enabled state of the tool region.
 *
 * @public                  @memberof GoSurfaceRivet
 * @param    tool           GoSurfaceRivet object.
 * @param    enable         kTRUE to enable the tool region and kFALSE to disable it.
 * @return                  Operation status.
 */
GoFx(kStatus) GoSurfaceRivet_EnableRegion(GoSurfaceRivet tool, kBool enable);

/**
 * Returns the tool's region object.
 *
 * @public                  @memberof GoSurfaceRivet
 * @param    tool           GoSurfaceRivet object.
 * @return                  A GoRegion3d object.
 */
GoFx(GoRegion3d) GoSurfaceRivet_Region(GoSurfaceRivet tool);

/**
 * Gets the enabled state of reference regions.
 *
 * @public                  @memberof GoSurfaceRivet
 * @param    tool           GoSurfaceRivet object.
 * @return                  kTRUE if enabled and kFALSE if disabled.
 */
GoFx(kBool) GoSurfaceRivet_RefRegionsEnabled(GoSurfaceRivet tool);

/**
 * Sets the enabled state of reference regions.
 *
 * @public                  @memberof GoSurfaceRivet
 * @param    tool           GoSurfaceRivet object.
 * @param    enable         kTRUE to enable reference regions and kFALSE to disable it.
 * @return                  Operation status.
 */
GoFx(kStatus) GoSurfaceRivet_EnableRefRegions(GoSurfaceRivet tool, kBool enable);

/**
 * Gets the reference region count.
 *
 * @public                  @memberof GoSurfaceRivet
 * @param    tool           GoSurfaceRivet object.
 * @return                  The reference region count.
 */
GoFx(kSize) GoSurfaceRivet_RefRegionCount(GoSurfaceRivet tool);

/**
 * Sets the reference region count.
 *
 * @public                  @memberof GoSurfaceRivet
 * @param    tool           GoSurfaceRivet object.
 * @param    count          The reference region count.
 * @return                  Operation status.
 * @see                     GO_SURFACE_RIVET_MAX_REF_REGIONS
 */
GoFx(kStatus) GoSurfaceRivet_SetRefRegionCount(GoSurfaceRivet tool, kSize count);

/**
 * Gets a reference region object at the given index.
 *
 * @public                  @memberof GoSurfaceRivet
 * @param    tool           GoSurfaceRivet object.
 * @param    index          The index with which to retrieve a reference region.
 * @return                  A GoSurfaceRegion2d object.
 * @see                     GoSurfaceRivet_RefRegionCount
 */
GoFx(GoSurfaceRegion2d) GoSurfaceRivet_RefRegionAt(GoSurfaceRivet tool, kSize index);

/**
 * Gets the enabled state of auto-tilt.
 *
 * @public                  @memberof GoSurfaceRivet
 * @param    tool           GoSurfaceRivet object.
 * @return                  kTRUE if enabled and kFALSE if disabled.
 */
GoFx(kBool) GoSurfaceRivet_AutoTiltEnabled(GoSurfaceRivet tool);

/**
 * Sets the enabled state of auto-tilt.
 *
 * @public                  @memberof GoSurfaceRivet
 * @param    tool           GoSurfaceRivet object.
 * @param    enable         kTRUE to enable it and kFALSE to disable it.
 * @return                  Operation status.
 */
GoFx(kStatus) GoSurfaceRivet_EnableAutoTilt(GoSurfaceRivet tool, kBool enable);

/**
 * Gets the tilt X-angle value.
 *
 * @public                  @memberof GoSurfaceRivet
 * @param    tool           GoSurfaceRivet object.
 * @return                  Tilt X-angle value.
 */
GoFx(k64f) GoSurfaceRivet_TiltXAngle(GoSurfaceRivet tool);

/**
 * Sets the tilt X-angle value.
 *
 * @public                  @memberof GoSurfaceRivet
 * @param    tool           GoSurfaceRivet object.
 * @param    value          The tilt X-angle value to set.
 * @return                  Operation status.
 */
GoFx(kStatus) GoSurfaceRivet_SetTiltXAngle(GoSurfaceRivet tool, k64f value);

/**
 * Gets the tilt Y-angle value.
 *
 * @public                  @memberof GoSurfaceRivet
 * @param    tool           GoSurfaceRivet object.
 * @return                  Tilt Y-angle value.
 */
GoFx(k64f) GoSurfaceRivet_TiltYAngle(GoSurfaceRivet tool);

/**
 * Sets the tilt Y-angle value.
 *
 * @public                  @memberof GoSurfaceRivet
 * @param    tool           GoSurfaceRivet object.
 * @param    value          The tilt Y-angle value to set.
 * @return                  Operation status.
 */
GoFx(kStatus) GoSurfaceRivet_SetTiltYAngle(GoSurfaceRivet tool, k64f value);

/**
 * Gets the rivet type.
 *
 * @public                  @memberof GoSurfaceRivet
 * @param    tool           GoSurfaceRivet object.
 * @return                  Rivet type value.
 */
GoFx(GoSurfaceRivetType) GoSurfaceRivet_Type(GoSurfaceRivet tool);

/**
 * Sets the rivet type.
 *
 * @public                  @memberof GoSurfaceRivet
 * @param    tool           GoSurfaceRivet object.
 * @param    type           The rivet type to set.
 * @return                  Operation status.
 */
GoFx(kStatus) GoSurfaceRivet_SetType(GoSurfaceRivet tool, GoSurfaceRivetType type);

/**
 * Gets the edge sensitivity.
 *
 * @public                  @memberof GoSurfaceRivet
 * @param    tool           GoSurfaceRivet object.
 * @return                  Sensitivity value.
 */
GoFx(k64f) GoSurfaceRivet_EdgeSensitivity(GoSurfaceRivet tool);

/**
 * Sets the edge sensitivity.
 *
 * @public                  @memberof GoSurfaceRivet
 * @param    tool           GoSurfaceRivet object.
 * @param    value          Value to set.
 * @return                  Operation status.
 */
GoFx(kStatus) GoSurfaceRivet_SetEdgeSensitivity(GoSurfaceRivet tool, k64f value);

/**
 * Gets the inner padding.
 *
 * @public                  @memberof GoSurfaceRivet
 * @param    tool           GoSurfaceRivet object.
 * @return                  Inner padding value.
 */
GoFx(k64f) GoSurfaceRivet_InnerPadding(GoSurfaceRivet tool);

/**
 * Sets the inner padding.
 *
 * @public                  @memberof GoSurfaceRivet
 * @param    tool           GoSurfaceRivet object.
 * @param    value          Value to set.
 * @return                  Operation status.
 */
GoFx(kStatus) GoSurfaceRivet_SetInnerPadding(GoSurfaceRivet tool, k64f value);

/**
 * Adds a new rivet tool measurement.
 *
 * @public                  @memberof GoSurfaceRivet
 * @param    tool           GoSurfaceRivet object
 * @param    type           The measurement type that is being added.
 * @param    measurement    A handle to the newly added measurement. (can be kNULL)
 * @return                  Operation status.
 */
GoFx(kStatus) GoSurfaceRivet_AddMeasurement(GoSurfaceRivet tool, GoMeasurementType type, GoMeasurement* measurement);

/**
 * Removes a measurement at the given index.
 *
 * @public                  @memberof GoSurfaceRivet
 * @param    tool           GoSurfaceRivet object.
 * @param    index          The index of the measurement to remove.
 * @return                  Operation status.
 */
GoFx(kStatus) GoSurfaceRivet_RemoveMeasurement(GoSurfaceRivet tool, kSize index);

/**
 * Gets the measurement count.
 *
 * @public                  @memberof GoSurfaceRivet
 * @param    tool           GoSurfaceRivet object.
 * @return                  Measurement count.
 */
GoFx(kSize) GoSurfaceRivet_MeasurementCount(GoSurfaceRivet tool);

/**
 * Gets the measurement at the given index.
 *
 * @public                  @memberof GoSurfaceRivet
 * @param    tool           GoSurfaceRivet object.
 * @param    index          The index of the measurement to return.
 * @return                  A rivet tool measurement handle.
 */
GoFx(GoMeasurement) GoSurfaceRivet_MeasurementAt(GoSurfaceRivet tool, kSize index);

typedef struct GoSurfaceRivetClass
{
    GoSurfaceToolClass base;

    k64f nominalHeight;
    k64f nominalRadius;
    k64f radiusTolerance;
    kBool partialDetectionEnabled;
    kBool regionEnabled;
    GoRegion3d region;
    kBool refRegionsEnabled;
    kSize refRegionCount;
    GoSurfaceRegion2d refRegions[GO_SURFACE_RIVET_MAX_REF_REGIONS];
    k64f tiltXAngle;
    k64f tiltYAngle;
    kBool autoTiltEnabled;
    GoSurfaceRivetType type;
    k64f edgeSensitivity;
    k64f innerPadding;
} GoSurfaceRivetClass;

kDeclareClassEx(Go, GoSurfaceRivet, GoSurfaceTool)

GoFx(kStatus) GoSurfaceRivet_Construct(GoSurfaceRivet* tool, kObject sensor, kAlloc allocator);
GoFx(kStatus) GoSurfaceRivet_VInit(GoSurfaceRivet tool, kType type, kObject sensor, kAlloc alloc);
GoFx(kStatus) GoSurfaceRivet_VRelease(GoSurfaceRivet tool);
GoFx(kStatus) GoSurfaceRivet_VRead(GoSurfaceRivet tool, kXml xml, kXmlItem item);
GoFx(kStatus) GoSurfaceRivet_VWrite(GoSurfaceRivet tool, kXml xml, kXmlItem item);

#endif

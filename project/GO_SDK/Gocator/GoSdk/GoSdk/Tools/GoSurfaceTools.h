/** 
 * @file    GoSurfaceTools.h
 * @brief   Declares all surface tools and their related classes. 
 *
 * @internal
 * Copyright (C) 2016-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */

#ifndef GO_SURFACE_TOOLS_H
#define GO_SURFACE_TOOLS_H

#include <GoSdk/GoSdkDef.h>
#include <GoSdk/Tools/GoTool.h>
#include <GoSdk/Tools/GoSurfaceToolUtils.h>
#include <GoSdk/GoUtils.h>

/**
 * @class   GoSurfaceTool
 * @extends GoTool
 * @note    Supported with G2, G3
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents a base surface tool.
 */
typedef GoTool GoSurfaceTool; 

/**
 * Sets the data stream. Note that stream validation will only occur if tool
 * is in the tool options list.
 *
 * @public              @memberof GoSurfaceTool
 * @note                Supported with G2, G3
 * @version             Introduced in firmware 5.3.19.50
 * @param    tool       GoSurfaceTool object.
 * @param    stream     GoDataStream value.
 * @return              Operation status.
 * @see                 GoSurfaceTool_StreamOptionCount, GoSurfaceTool_StreamOptionAt
 */
GoFx(kStatus) GoSurfaceTool_SetStream(GoSurfaceTool tool, GoDataStream stream);

/**
 * Gets the data stream.
 *
 * @public              @memberof GoSurfaceTool
 * @note                Supported with G2, G3
 * @version             Introduced in firmware 5.3.19.50
 * @param    tool       GoSurfaceTool object.
 * @return              The current Surface tool data stream value.
 */
GoFx(GoDataStream) GoSurfaceTool_Stream(GoSurfaceTool tool);

/**
 * Gets the data stream option list count.
 *
 * @public              @memberof GoSurfaceTool
 * @note                Supported with G2, G3
 * @version             Introduced in firmware 5.3.19.50
 * @param    tool       GoSurfaceTool object.
 * @return              The current Surface tool data stream option list count.
 */
GoFx(kSize) GoSurfaceTool_StreamOptionCount(GoSurfaceTool tool);

/**
 * Gets the data stream option at the given index.
 *
 * @public              @memberof GoSurfaceTool
 * @note                Supported with G2, G3
 * @version             Introduced in firmware 5.3.19.50
 * @param    tool       GoSurfaceTool object.
 * @param    index      The index of the option list to access.
 * @return              The Surface tool data stream option at the given index, or k32U_MAX if an invalid index is given.
 */
GoFx(GoDataStream) GoSurfaceTool_StreamOptionAt(GoSurfaceTool tool, kSize index);

/** 
 * Sets the data source. Note that source validation will only occur if tool 
 * is in the tool options list.
 *
 * @public               @memberof GoSurfaceTool
 * @note                 Supported with G2, G3
 * @version              Introduced in firmware 4.0.10.27
 * @param    tool        GoSurfaceTool object.
 * @param    source      GoDataSource object.
 * @return               Operation status.            
 * @see                  GoTools_ToolOptionCount, GoTools_ToolOptionAt
 */
GoFx(kStatus) GoSurfaceTool_SetSource(GoSurfaceTool tool, GoDataSource source);

/** 
 * Gets the data source.
 *
 * @public               @memberof GoSurfaceTool
 * @note                 Supported with G2, G3
 * @version              Introduced in firmware 4.0.10.27
 * @param    tool        GoSurfaceTool object.
 * @return   The data source.            
 */
GoFx(GoDataSource) GoSurfaceTool_Source(GoSurfaceTool tool);

/** 
 * Gets the data source option list count.
 *
 * @public              @memberof GoSurfaceTool
 * @note                Supported with G2, G3
 * @version             Introduced in firmware 4.0.10.27
 * @param    tool       GoSurfaceTool object.
 * @return              The current tool data source option list count.            
 */
GoFx(kSize) GoSurfaceTool_SourceOptionCount(GoSurfaceTool tool);

/** 
 * Gets the data source option at the given index.
 *
 * @public              @memberof GoSurfaceTool
 * @note                Supported with G2, G3
 * @version             Introduced in firmware 4.0.10.27
 * @param    tool       GoSurfaceTool object.
 * @param    index      The index of the option list to access.
 * @return              The tool data source option at the given index, or k32U_MAX if an invalid index is given.            
 */

GoFx(k32u) GoSurfaceTool_SourceOptionAt(GoSurfaceTool tool, kSize index);

/** 
 * Gets the X-anchoring option list count.
 *
 * @public              @memberof GoSurfaceTool
 * @note                Supported with G2, G3
 * @version             Introduced in firmware 4.0.10.27
 * @param    tool       GoSurfaceTool object.
 * @return              The X-anchoring option list count.
 */
GoFx(kSize) GoSurfaceTool_XAnchorOptionCount(GoSurfaceTool tool);

/** 
 * Gets the X-anchoring option at the given index.
 *
 * @public              @memberof GoSurfaceTool
 * @note                Supported with G2, G3
 * @version             Introduced in firmware 4.0.10.27
 * @param    tool       GoSurfaceTool object.
 * @param    index      The index of the option list to access.
 * @return              The X-anchoring option at the given index or k32U_MAX if invalid.
 */
GoFx(k32u) GoSurfaceTool_XAnchorOptionAt(GoSurfaceTool tool, kSize index);

/** 
 * Gets the current X-anchoring source.
 *
 * @public              @memberof GoSurfaceTool
 * @note                Supported with G2, G3
 * @version             Introduced in firmware 4.0.10.27
 * @param    tool       GoSurfaceTool object.
 * @return              The X-anchoring source or -1 if no source is currently set.
 */
GoFx(k32s) GoSurfaceTool_XAnchor(GoSurfaceTool tool);

/** 
 * Sets the X-anchoring source.
 *
 * @public              @memberof GoSurfaceTool
 * @note                Supported with G2, G3
 * @version             Introduced in firmware 4.0.10.27
 * @param    tool       GoSurfaceTool object.
 * @param    id         The measurement ID of a valid X-anchoring source.
 * @return              Operation status.
 */
GoFx(kStatus) GoSurfaceTool_SetXAnchor(GoSurfaceTool tool, k32s id);

/** 
 * Returns a boolean value representing whether or not a valid X-anchoring source has been set for X-anchoring.
 *
 * @public              @memberof GoSurfaceTool
 * @note                Supported with G2, G3
 * @version             Introduced in firmware 4.0.10.27
 * @param    tool       GoSurfaceTool object.
 * @return              kTRUE if a valid anchoring source is currently set and kFALSE otherwise.
 */
GoFx(kBool) GoSurfaceTool_XAnchorEnabled(GoSurfaceTool tool);

/** 
 * Gets the Y-anchoring option list count.
 *
 * @public              @memberof GoSurfaceTool
 * @note                Supported with G2, G3
 * @version             Introduced in firmware 4.0.10.27
 * @param    tool       GoSurfaceTool object.
 * @return              The Y-anchoring option list count.
 */
GoFx(kSize) GoSurfaceTool_YAnchorOptionCount(GoSurfaceTool tool);

/** 
 * Gets the Y-anchoring option at the given index.
 *
 * @public              @memberof GoSurfaceTool
 * @note                Supported with G2, G3
 * @version             Introduced in firmware 4.0.10.27
 * @param    tool       GoSurfaceTool object.
 * @param    index      The index of the option list to access.
 * @return              The Y-anchoring option at the given index or k32U_MAX if invalid.
 */
GoFx(k32u) GoSurfaceTool_YAnchorOptionAt(GoSurfaceTool tool, kSize index);

/** 
 * Gets the current Y-anchoring source.
 *
 * @public              @memberof GoSurfaceTool
 * @note                Supported with G2, G3
 * @version             Introduced in firmware 4.0.10.27
 * @param    tool       GoSurfaceTool object.
 * @return              The Y-anchoring source or -1 if no source is currently set.
 */
GoFx(k32s) GoSurfaceTool_YAnchor(GoSurfaceTool tool);

/** 
 * Sets the Y-anchoring source.
 *
 * @public              @memberof GoSurfaceTool
 * @note                Supported with G2, G3
 * @version             Introduced in firmware 4.0.10.27
 * @param    tool       GoSurfaceTool object.
 * @param    id         The measurement ID of a valid Y-anchoring source.
 * @return              Operation status.
 */
GoFx(kStatus) GoSurfaceTool_SetYAnchor(GoSurfaceTool tool, k32s id);

/** 
 * Returns a boolean value representing whether or not a valid Y-anchoring source has been set for Y-anchoring.
 *
 * @public              @memberof GoSurfaceTool
 * @note                Supported with G2, G3
 * @version             Introduced in firmware 4.0.10.27
 * @param    tool       GoSurfaceTool object.
 * @return              kTRUE if a valid anchoring source is currently set and kFALSE otherwise.
 */
GoFx(kBool) GoSurfaceTool_YAnchorEnabled(GoSurfaceTool tool);

/** 
 * Gets the Z-anchoring option list count.
 *
 * @public              @memberof GoSurfaceTool
 * @note                Supported with G2, G3
 * @version             Introduced in firmware 4.0.10.27
 * @param    tool       GoSurfaceTool object.
 * @return              The X-anchoring option list count.
 */
GoFx(kSize) GoSurfaceTool_ZAnchorOptionCount(GoSurfaceTool tool);

/** 
 * Gets the Z-anchoring option at the given index.
 *
 * @public              @memberof GoSurfaceTool
 * @note                Supported with G2, G3
 * @version             Introduced in firmware 4.0.10.27
 * @param    tool       GoSurfaceTool object.
 * @param    index      The index of the option list to access.
 * @return              The Z-anchoring option at the given index or k32U_MAX if invalid.
 */
GoFx(k32u) GoSurfaceTool_ZAnchorOptionAt(GoSurfaceTool tool, kSize index);

/** 
 * Gets the current Z-anchoring source.
 *
 * @public              @memberof GoSurfaceTool
 * @note                Supported with G2, G3
 * @version             Introduced in firmware 4.0.10.27
 * @param    tool       GoSurfaceTool object.
 * @return              The Z-anchoring source or -1 if no source is currently set.
 */
GoFx(k32s) GoSurfaceTool_ZAnchor(GoSurfaceTool tool);

/** 
 * Sets the Z-anchoring source.
 *
 * @public              @memberof GoSurfaceTool
 * @note                Supported with G2, G3
 * @version             Introduced in firmware 4.0.10.27
 * @param    tool       GoSurfaceTool object.
 * @param    id         The measurement ID of a valid Z-anchoring source.
 * @return              Operation status.
 */
GoFx(kStatus) GoSurfaceTool_SetZAnchor(GoSurfaceTool tool, k32s id);

/** 
 * Returns a boolean value representing whether or not a valid Z-anchoring source has been set for Z-anchoring.
 *
 * @public              @memberof GoSurfaceTool
 * @note                Supported with G2, G3
 * @version             Introduced in firmware 4.0.10.27 
 * @param    tool       GoSurfaceTool object.
 * @return              kTRUE if a valid anchoring source is currently set and kFALSE otherwise.
 */
GoFx(kBool) GoSurfaceTool_ZAnchorEnabled(GoSurfaceTool tool);

/**
* Gets the Z-anchoring option list count.
*
* @public              @memberof GoSurfaceTool
* @note                Supported with G2, G3
* @version             Introduced in firmware 4.6.1.123
* @param    tool       GoSurfaceTool object.
* @return              The X-anchoring option list count.
*/
GoFx(kSize) GoSurfaceTool_ZAngleAnchorOptionCount(GoSurfaceTool tool);

/**
* Gets the ZAngle-anchoring option at the given index.
*
* @public              @memberof GoSurfaceTool
* @note                Supported with G2, G3
* @version             Introduced in firmware 4.6.1.123
* @param    tool       GoSurfaceTool object.
* @param    index      The index of the option list to access.
* @return              The Z-anchoring option at the given index or k32U_MAX if invalid.
*/
GoFx(k32u) GoSurfaceTool_ZAngleAnchorOptionAt(GoSurfaceTool tool, kSize index);

/**
* Sets the ZAngle-anchoring source.
*
* @public              @memberof GoSurfaceTool
* @note                Supported with G2, G3
* @version             Introduced in firmware 4.6.1.123
* @param    tool       GoSurfaceTool object.
* @param    id         The measurement ID of a valid Z-anchoring source.
* @return              Operation status.
*/
GoFx(kStatus) GoSurfaceTool_SetZAngleAnchor(GoSurfaceTool tool, k32s id);

/**
* Returns a boolean value representing whether or not a valid ZAngle - anchoring source has been set for ZAngle - anchoring.
*
* @public              @memberof GoSurfaceTool
* @note                Supported with G2, G3
* @version             Introduced in firmware 4.6.1.123
* @param    tool       GoSurfaceTool object.
* @return              kTRUE if a valid anchoring source is currently set and kFALSE otherwise.
*/
GoFx(kBool) GoSurfaceTool_ZAngleAnchorEnabled(GoSurfaceTool tool);

/**
* Gets the ZAngle-anchoring source.
*
* @public              @memberof GoSurfaceTool
* @note                Supported with G2, G3
* @version             Introduced in firmware 4.6.1.123
* @param    tool       GoSurfaceTool object.
* @return              Operation status.
*/
GoFx(k32s) GoSurfaceTool_ZAngleAnchor(GoSurfaceTool tool);

/**
 * @class   GoSurfaceBox
 * @extends GoSurfaceTool
 * @note    Supported with G2, G3
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents a surface bounding box tool.\n all width/height/length/radius and x/y/z units are in mm, angles in degrees
 */
typedef GoSurfaceTool GoSurfaceBox; 

/** 
 * Returns the enabled state of Z-rotation.
 *
 * @public                  @memberof GoSurfaceBox
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceBox object.
 * @return                  kTRUE if enabled and kFALSE if disabled.
 */
GoFx(kBool) GoSurfaceBox_ZRotationEnabled(GoSurfaceBox tool);

/** 
 * Enables or disables Z-rotation.
 *
 * @public                  @memberof GoSurfaceBox
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceBox object.
 * @param    enable         kTRUE to enable Z-rotation, kFALSE to disable it.
 * @return                  Operation status.
 */
GoFx(kStatus) GoSurfaceBox_EnableZRotation(GoSurfaceBox tool, kBool enable);

/** 
 * Returns the enabled state of the tool region.
 *
 * @public                  @memberof GoSurfaceBox
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceBox object.
 * @return                  kTRUE if enabled and kFALSE if disabled.
 */
GoFx(kBool) GoSurfaceBox_RegionEnabled(GoSurfaceBox tool);

/** 
 * Enables or disables the tool region.
 *
 * @public                  @memberof GoSurfaceBox
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceBox object.
 * @param    enable         kTRUE to enable the tool region, kFALSE to disable it.
 * @return                  Operation status.
 */
GoFx(kStatus) GoSurfaceBox_EnableRegion(GoSurfaceBox tool, kBool enable);

/** 
 * Gets the surface bounding box region.
 *
 * @public                  @memberof GoSurfaceBox
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceBox object.
 * @return                  A GoRegion3d object.
 */
GoFx(GoRegion3d) GoSurfaceBox_Region(GoSurfaceBox tool);

/** 
 * Sets the asymmetry detection type.
 *
 * @public                  @memberof GoSurfaceBox
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.4.4.14
 * @param    tool           GoSurfaceBox object.
 * @param    value          The asymmetry detection type to set.
 * @return                  Operation status.
 */
GoFx(kStatus) GoSurfaceBox_SetAsymmetryDetectionType(GoSurfaceBox tool, GoBoxAsymmetryType value);

/** 
 * Gets the asymmetry detection type.
 *
 * @public                  @memberof GoSurfaceBox
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.4.4.14
 * @param    tool           GoSurfaceBox object.
 * @return                  The asymmetry detection type.
 */
GoFx(GoBoxAsymmetryType) GoSurfaceBox_AsymmetryDetectionType(GoSurfaceBox tool);

/**
 * Returns a GoSurfaceBox X measurement object.
 *
 * @public              @memberof GoSurfaceBox
 * @note                Supported with G2, G3
 * @version             Introduced in firmware 4.0.10.27
 * @param    tool       GoSurfaceBox object.
 * @return              A GoSurfaceBox X measurement.
 */
GoFx(GoSurfaceBoxX) GoSurfaceBox_XMeasurement(GoSurfaceBox tool);

/**
 * Returns a GoSurfaceBox Y measurement object.
 *
 * @public              @memberof GoSurfaceBox
 * @note                Supported with G2, G3
 * @version             Introduced in firmware 4.0.10.27
 * @param    tool       GoSurfaceBox object.
 * @return              A GoSurfaceBox Y measurement.
 */
GoFx(GoSurfaceBoxY) GoSurfaceBox_YMeasurement(GoSurfaceBox tool);

/**
 * Returns a GoSurfaceBox Z measurement object.
 *
 * @public              @memberof GoSurfaceBox
 * @note                Supported with G2, G3
 * @version             Introduced in firmware 4.0.10.27
 * @param    tool       GoSurfaceBox object.
 * @return              A GoSurfaceBox Z measurement.
 */
GoFx(GoSurfaceBoxZ) GoSurfaceBox_ZMeasurement(GoSurfaceBox tool);

/**
 * Returns a GoSurfaceBox Width measurement object.
 *
 * @public              @memberof GoSurfaceBox
 * @note                Supported with G2, G3
 * @version             Introduced in firmware 4.0.10.27
 * @param    tool       GoSurfaceBox object.
 * @return              A GoSurfaceBox Width measurement.
 */
GoFx(GoSurfaceBoxWidth) GoSurfaceBox_WidthMeasurement(GoSurfaceBox tool);

/**
 * Returns a GoSurfaceBox Length measurement object.
 *
 * @public              @memberof GoSurfaceBox
 * @note                Supported with G2, G3
 * @version             Introduced in firmware 4.0.10.27
 * @param    tool       GoSurfaceBox object.
 * @return              A GoSurfaceBox Length measurement.
 */
GoFx(GoSurfaceBoxLength) GoSurfaceBox_LengthMeasurement(GoSurfaceBox tool);

/**
 * Returns a GoSurfaceBox Height measurement object.
 *
 * @public              @memberof GoSurfaceBox
 * @note                Supported with G2, G3
 * @version             Introduced in firmware 4.0.10.27
 * @param    tool       GoSurfaceBox object.
 * @return              A GoSurfaceBox Height measurement.
 */
GoFx(GoSurfaceBoxHeight) GoSurfaceBox_HeightMeasurement(GoSurfaceBox tool);

/**
 * Returns a GoSurfaceBox Z Angle measurement object.
 *
 * @public              @memberof GoSurfaceBox
 * @note                Supported with G2, G3
 * @version             Introduced in firmware 4.0.10.27
 * @param    tool       GoSurfaceBox object.
 * @return              A GoSurfaceBox Z Angle measurement.
 */
GoFx(GoSurfaceBoxZAngle) GoSurfaceBox_ZAngleMeasurement(GoSurfaceBox tool);

/**
 * Returns a GoSurfaceBox global X measurement object.
 *
 * @public              @memberof GoSurfaceBox
 * @note                Supported with G2, G3
 * @version             Introduced in firmware 4.0.10.27
 * @param    tool       GoSurfaceBox object.
 * @return              A GoSurfaceBox global X measurement.
 */
GoFx(GoSurfaceBoxGlobalX) GoSurfaceBox_GlobalXMeasurement(GoSurfaceBox tool);

/**
 * Returns a GoSurfaceBox global Y measurement object.
 *
 * @public              @memberof GoSurfaceBox
 * @note                Supported with G2, G3
 * @version             Introduced in firmware 4.0.10.27
 * @param    tool       GoSurfaceBox object.
 * @return              A GoSurfaceBox global Y measurement.
 */
GoFx(GoSurfaceBoxGlobalY) GoSurfaceBox_GlobalYMeasurement(GoSurfaceBox tool);

/**
 * Returns a GoSurfaceBox global Z Angle measurement object.
 *
 * @public              @memberof GoSurfaceBox
 * @note                Supported with G2, G3
 * @version             Introduced in firmware 4.2.4.7
 * @param    tool       GoSurfaceBox object.
 * @return              A GoSurfaceBox global Z angle measurement.
 */
GoFx(GoSurfaceBoxGlobalZAngle) GoSurfaceBox_GlobalZAngleMeasurement(GoSurfaceBox tool);

/**
* Returns a GoSurfaceBoundingBoxCenterPoint center point feature object.
*
* @public              @memberof GoSurfaceBox
* @note                Supported with G2, G3
* @version             Introduced in firmware 4.2.4.7
* @param    tool       GoSurfaceBox object.
* @return              A GoSurfaceBoundingBoxCenterPoint center point feature.
*/
GoFx(GoSurfaceBoundingBoxCenterPoint) GoSurfaceBox_CenterPoint(GoSurfaceBox tool);

/**
* Returns a GoSurfaceBoundingBoxAxisLine axis line feature object.
*
* @public              @memberof GoSurfaceBox
* @version             Introduced in firmware 6.1.18.16
* @note                Supported with G2, G3
* @param    tool       GoSurfaceBox object.
* @return              A GoSurfaceBoundingBoxAxisLine axis line feature.
*/
GoFx(GoSurfaceBoundingBoxAxisLine) GoSurfaceBox_AxisLine(GoSurfaceBox tool);

/**
* @class   GoSurfaceCountersunkHole
* @extends GoSurfaceTool
* @note    Supported with G2, G3
* @ingroup GoSdk-SurfaceTools
* @brief   Represents a Surface Counter Sunk Hole tool. all width/height/length/radius and x/y/z units are in mm, angles in degrees
*/
typedef GoSurfaceTool GoSurfaceCountersunkHole; 

/**
 * Sets the shape.
 *
 * @public           @memberof GoSurfaceCountersunkHole
 * @note             Supported with G2, G3
 * @version          Introduced in firmware 4.3.3.124
 * @param    tool    GoSurfaceCountersunkHole object.
 * @param    value   The value to set.
 * @return           Operation status.
 */
GoFx(kStatus) GoSurfaceCountersunkHole_SetShape(GoSurfaceCountersunkHole tool, GoSurfaceCountersunkHoleShape value);

/**
 * Returns the shape.
 *
 * @public           @memberof GoSurfaceCountersunkHole
 * @note             Supported with G2, G3
 * @version          Introduced in firmware 4.3.3.124
 * @param    tool    GoSurfaceCountersunkHole object.
 * @return           The nominal bevel angle.
 */
GoFx(GoSurfaceCountersunkHoleShape) GoSurfaceCountersunkHole_Shape(GoSurfaceCountersunkHole tool);

/**
 * Sets the nominal bevel angle.
 *
 * @public           @memberof GoSurfaceCountersunkHole
 * @note             Supported with G2, G3
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoSurfaceCountersunkHole object.
 * @param    value   The value to set.
 * @return           Operation status.
 */
GoFx(kStatus) GoSurfaceCountersunkHole_SetNominalBevelAngle(GoSurfaceCountersunkHole tool, k64f value);

/**
 * Returns the nominal bevel angle.
 *
 * @public           @memberof GoSurfaceCountersunkHole
 * @note             Supported with G2, G3
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoSurfaceCountersunkHole object.
 * @return           The nominal bevel angle.
 */
GoFx(k64f) GoSurfaceCountersunkHole_NominalBevelAngle(GoSurfaceCountersunkHole tool);

/**
 * Sets the nominal outer radius.
 *
 * @public           @memberof GoSurfaceCountersunkHole
 * @note             Supported with G2, G3
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoSurfaceCountersunkHole object.
 * @param    value   The value to set.
 * @return           Operation status.
 */
GoFx(kStatus) GoSurfaceCountersunkHole_SetNominalOuterRadius(GoSurfaceCountersunkHole tool, k64f value);

/**
 * Returns the nominal outer radius.
 *
 * @public           @memberof GoSurfaceCountersunkHole
 * @note             Supported with G2, G3
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoSurfaceCountersunkHole object.
 * @return           The nominal outer radius.
 */
GoFx(k64f) GoSurfaceCountersunkHole_NominalOuterRadius(GoSurfaceCountersunkHole tool);

/**
 * Sets the nominal inner radius.
 *
 * @public           @memberof GoSurfaceCountersunkHole
 * @note             Supported with G2, G3
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoSurfaceCountersunkHole object.
 * @param    value   The value to set.
 * @return           Operation status.
 */
GoFx(kStatus) GoSurfaceCountersunkHole_SetNominalInnerRadius(GoSurfaceCountersunkHole tool, k64f value);

/**
 * Returns the nominal inner radius.
 *
 * @public           @memberof GoSurfaceCountersunkHole
 * @note             Supported with G2, G3
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoSurfaceCountersunkHole object.
 * @return           The nominal inner radius.
 */
GoFx(k64f) GoSurfaceCountersunkHole_NominalInnerRadius(GoSurfaceCountersunkHole tool);

/**
 * Sets the bevel radius offset.
 *
 * @public           @memberof GoSurfaceCountersunkHole
 * @note             Supported with G2, G3
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoSurfaceCountersunkHole object.
 * @param    value   The value to set.
 * @return           Operation status.
 */
GoFx(kStatus) GoSurfaceCountersunkHole_SetBevelRadiusOffset(GoSurfaceCountersunkHole tool, k64f value);

/**
 * Returns the bevel radius offset.
 *
 * @public           @memberof GoSurfaceCountersunkHole
 * @note             Supported with G2, G3
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoSurfaceCountersunkHole object.
 * @return           The bevel radius offset.
 */
GoFx(k64f) GoSurfaceCountersunkHole_BevelRadiusOffset(GoSurfaceCountersunkHole tool);

/**
 * Enables or disables partial counter sunk hole detection.
 *
 * @public           @memberof GoSurfaceCountersunkHole
 * @note             Supported with G2, G3
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoSurfaceCountersunkHole object.
 * @param    enable  kTRUE to enable partial detection and kFALSE to disable it.
 * @return           Operation status.
 */
GoFx(kStatus) GoSurfaceCountersunkHole_EnablePartialDetection(GoSurfaceCountersunkHole tool, kBool enable);

/**
 * Returns the state of partial detection.
 *
 * @public           @memberof GoSurfaceCountersunkHole
 * @note             Supported with G2, G3
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoSurfaceCountersunkHole object.
 * @return           kTRUE if partial detection is enabled. kFALSE otherwise.
 */
GoFx(kBool) GoSurfaceCountersunkHole_PartialDetectionEnabled(GoSurfaceCountersunkHole tool);

/**
 * Enables or disables the tool region.
 *
 * @public           @memberof GoSurfaceCountersunkHole
 * @note             Supported with G2, G3
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoSurfaceCountersunkHole object.
 * @param    enable  kTRUE to enable the tool region and kFALSE to disable it.
 * @return           Operation status.
 */
GoFx(kStatus) GoSurfaceCountersunkHole_EnableRegion(GoSurfaceCountersunkHole tool, kBool enable);

/**
 * Returns the state of the tool region.
 *
 * @public           @memberof GoSurfaceCountersunkHole
 * @note             Supported with G2, G3
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoSurfaceCountersunkHole object.
 * @return           kTRUE if the tool region is enabled. kFALSE otherwise.
 */
GoFx(kBool) GoSurfaceCountersunkHole_RegionEnabled(GoSurfaceCountersunkHole tool);

/**
 * Returns the tool region.
 *
 * @public           @memberof GoSurfaceCountersunkHole
 * @note             Supported with G2, G3
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoSurfaceCountersunkHole object.
 * @return           The tool region.
 */
GoFx(GoRegion3d) GoSurfaceCountersunkHole_Region(GoSurfaceCountersunkHole tool);

/**
 * Enables or disables reference regions.
 *
 * @public           @memberof GoSurfaceCountersunkHole
 * @note             Supported with G2, G3
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoSurfaceCountersunkHole object.
 * @param    enable  kTRUE to enable reference regions and kFALSE to disable them.
 * @return           Operation status.
 */
GoFx(kStatus) GoSurfaceCountersunkHole_EnableRefRegions(GoSurfaceCountersunkHole tool, kBool enable);

/**
 * Returns the state of the tool reference regions.
 *
 * @public           @memberof GoSurfaceCountersunkHole
 * @note             Supported with G2, G3
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoSurfaceCountersunkHole object.
 * @return           kTRUE if refrence regions are enabled. kFALSE otherwise.
 */
GoFx(kBool) GoSurfaceCountersunkHole_RefRegionsEnabled(GoSurfaceCountersunkHole tool);

/**
 * Sets the reference region count.
 *
 * @public           @memberof GoSurfaceCountersunkHole
 * @note             Supported with G2, G3
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoSurfaceCountersunkHole object.
 * @param    count   The number of references regions to use when enabled.
 * @return           Reference region count.
 * @see              GO_SURFACE_COUNTERSUNK_HOLE_MAX_REF_REGIONS

 */
GoFx(kStatus) GoSurfaceCountersunkHole_SetRefRegionCount(GoSurfaceCountersunkHole tool, kSize count);

/**
 * Returns the reference region count.
 *
 * @public           @memberof GoSurfaceCountersunkHole
 * @note             Supported with G2, G3
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoSurfaceCountersunkHole object.
 * @return           Reference region count.
 */
GoFx(kSize) GoSurfaceCountersunkHole_RefRegionCount(GoSurfaceCountersunkHole tool);

/**
 * Returns the reference region at the given index.
 *
 * @public           @memberof GoSurfaceCountersunkHole
 * @note             Supported with G2, G3
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoSurfaceCountersunkHole object.
 * @param    index   The index of the reference region to retrieve.
 * @return           A reference region.
 */
GoFx(GoSurfaceRegion2d) GoSurfaceCountersunkHole_RefRegionAt(GoSurfaceCountersunkHole tool, kSize index);

/**
 * Enables or disables automatic tilt.
 *
 * @public           @memberof GoSurfaceCountersunkHole
 * @note             Supported with G2, G3
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoSurfaceCountersunkHole object.
 * @param    enable  kTRUE to enable auto tilt and kFALSE to disable it.
 * @return           Operation status.
 */
GoFx(kStatus) GoSurfaceCountersunkHole_EnableAutoTilt(GoSurfaceCountersunkHole tool, kBool enable);

/**
 * Returns the state of auto tilt.
 *
 * @public           @memberof GoSurfaceCountersunkHole
 * @note             Supported with G2, G3
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoSurfaceCountersunkHole object.
 * @return           kTRUE if auto tilt is enabled. kFALSE otherwise.
 */
GoFx(kBool) GoSurfaceCountersunkHole_AutoTiltEnabled(GoSurfaceCountersunkHole tool);

/**
 * Sets the tilt X angle.
 *
 * @public           @memberof GoSurfaceCountersunkHole
 * @note             Supported with G2, G3
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoSurfaceCountersunkHole object.
 * @param    value   The value to set.
 * @return           Operation status.
 */
GoFx(kStatus) GoSurfaceCountersunkHole_SetTiltXAngle(GoSurfaceCountersunkHole tool, k64f value);

/**
 * Returns the tilt X angle value.
 *
 * @public           @memberof GoSurfaceCountersunkHole
 * @note             Supported with G2, G3
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoSurfaceCountersunkHole object.
 * @return           Tilt X angle value.
 */
GoFx(k64f) GoSurfaceCountersunkHole_TiltXAngle(GoSurfaceCountersunkHole tool);

/**
 * Sets the tilt Y angle.
 *
 * @public           @memberof GoSurfaceCountersunkHole
 * @note             Supported with G2, G3
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoSurfaceCountersunkHole object.
 * @param    value   The value to set.
 * @return           Operation status.
 */
GoFx(kStatus) GoSurfaceCountersunkHole_SetTiltYAngle(GoSurfaceCountersunkHole tool, k64f value);

/**
 * Returns the tilt Y angle value.
 *
 * @public           @memberof GoSurfaceCountersunkHole
 * @note             Supported with G2, G3
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoSurfaceCountersunkHole object.
 * @return           Tilt Y angle value.
 */
GoFx(k64f) GoSurfaceCountersunkHole_TiltYAngle(GoSurfaceCountersunkHole tool);

/**
 * Enables or disables curve fitting.
 *
 * @public           @memberof GoSurfaceCountersunkHole
 * @note             Supported with G2, G3
 * @version          Introduced in firmware 4.1.3.106
 * @param    tool    GoSurfaceCountersunkHole object.
 * @param    enable  kTRUE to enable curve fitting and kFALSE to disable it.
 * @return           Operation status.
 */
GoFx(kStatus) GoSurfaceCountersunkHole_EnableCurveFit(GoSurfaceCountersunkHole tool, kBool enable);


/**
 * Returns the state of auto tilt.
 *
 * @public           @memberof GoSurfaceCountersunkHole
 * @note             Supported with G2, G3
 * @version          Introduced in firmware 4.1.3.106
 * @param    tool    GoSurfaceCountersunkHole object.
 * @return           kTRUE if curve fitting is enabled. kFALSE otherwise.
 */
GoFx(kBool) GoSurfaceCountersunkHole_CurveFitEnabled(GoSurfaceCountersunkHole tool);

/**
 * Sets the curve orientation angle.
 *
 * @public           @memberof GoSurfaceCountersunkHole
 * @note             Supported with G2, G3
 * @version          Introduced in firmware 4.1.3.106
 * @param    tool    GoSurfaceCountersunkHole object.
 * @param    value   The value to set.
 * @return           Operation status.
 */
GoFx(kStatus) GoSurfaceCountersunkHole_SetCurveOrientation(GoSurfaceCountersunkHole tool, k64f value);

/**
 * Returns the curve orientation angle value.
 *
 * @public           @memberof GoSurfaceCountersunkHole
 * @note             Supported with G2, G3
 * @version          Introduced in firmware 4.1.3.106
 * @param    tool    GoSurfaceCountersunkHole object.
 * @return           Curve orientation angle value.
 */
GoFx(k64f) GoSurfaceCountersunkHole_CurveOrientation(GoSurfaceCountersunkHole tool);

/**
 * Sets the plane fit range.
 *
 * @public           @memberof GoSurfaceCountersunkHole
 * @note             Supported with G2, G3
 * @version          Introduced in firmware 4.5.3.57
 * @param    tool    GoSurfaceCountersunkHole object.
 * @param    value   The value to set.
 * @return           Operation status.
 */
GoFx(kStatus) GoSurfaceCountersunkHole_SetPlaneFitRange(GoSurfaceCountersunkHole tool, k64f value);

/** 
 * Gets the enabled state of the plane fit range.
 *
 * @public                  @memberof GoSurfaceCountersunkHole
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.5.3.57
 * @param    tool           GoSurfaceCountersunkHole object.
 * @return                  kTRUE if enabled and kFALSE if disabled.
 */
GoFx(kBool) GoSurfaceCountersunkHole_PlaneFitRangeEnabled(GoSurfaceCountersunkHole tool);

/** 
 * Sets the enabled state of the plane fit range.
 *
 * @public                  @memberof GoSurfaceCountersunkHole
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.5.3.57
 * @param    tool           GoSurfaceCountersunkHole object.
 * @param    enable         kTRUE to enable it and kFALSE to disable it.
 * @return                  Operation status.
 */
GoFx(kStatus) GoSurfaceCountersunkHole_EnablePlaneFitRange(GoSurfaceCountersunkHole tool, kBool enable);

/**
 * Returns the plane fit range.
 *
 * @public           @memberof GoSurfaceCountersunkHole
 * @note             Supported with G2, G3
 * @version          Introduced in firmware 4.5.3.57
 * @param    tool    GoSurfaceCountersunkHole object.
 * @return           Plane fit range value.
 */
GoFx(k64f) GoSurfaceCountersunkHole_PlaneFitRange(GoSurfaceCountersunkHole tool);

/**
 * Returns a GoSurfaceCountersunkHole tool X position measurement object.
 *
 * @public           @memberof GoSurfaceCountersunkHole
 * @note             Supported with G2, G3
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoSurfaceCountersunkHole object.
 * @return           A GoSurfaceCountersunkHole X position measurement.
 */
GoFx(GoSurfaceCountersunkHoleX) GoSurfaceCountersunkHole_XMeasurement(GoSurfaceCountersunkHole tool);

/**
 * Returns a GoSurfaceCountersunkHole tool Y position measurement object.
 *
 * @public           @memberof GoSurfaceCountersunkHole
 * @note             Supported with G2, G3
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoSurfaceCountersunkHole object.
 * @return           A GoSurfaceCountersunkHole Y position measurement.
 */
GoFx(GoSurfaceCountersunkHoleY) GoSurfaceCountersunkHole_YMeasurement(GoSurfaceCountersunkHole tool);

/**
 * Returns a GoSurfaceCountersunkHole tool Z position measurement object.
 *
 * @public           @memberof GoSurfaceCountersunkHole
 * @note             Supported with G2, G3
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoSurfaceCountersunkHole object.
 * @return           A GoSurfaceCountersunkHole Z position measurement.
 */
GoFx(GoSurfaceCountersunkHoleZ) GoSurfaceCountersunkHole_ZMeasurement(GoSurfaceCountersunkHole tool);

/**
 * Returns a GoSurfaceCountersunkHole tool Outer Radius measurement object.
 *
 * @public           @memberof GoSurfaceCountersunkHole
 * @note             Supported with G2, G3
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoSurfaceCountersunkHole object.
 * @return           A GoSurfaceCountersunkHoleOuterRadius Outer Radius measurement.
 */
GoFx(GoSurfaceCountersunkHoleOuterRadius) GoSurfaceCountersunkHole_OuterRadiusMeasurement(GoSurfaceCountersunkHole tool);

/**
 * Returns a GoSurfaceCountersunkHole tool Depth measurement object.
 *
 * @public           @memberof GoSurfaceCountersunkHole
 * @note             Supported with G2, G3
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoSurfaceCountersunkHole object.
 * @return           A GoSurfaceCoGoSurfaceCountersunkHoleDepthuntersunkHole Depth measurement.
 */
GoFx(GoSurfaceCountersunkHoleDepth) GoSurfaceCountersunkHole_DepthMeasurement(GoSurfaceCountersunkHole tool);

/**
 * Returns a GoSurfaceCountersunkHole tool counterbore depth measurement object.
 *
 * @public           @memberof GoSurfaceCountersunkHole
 * @note             Supported with G2, G3
 * @version          Introduced in firmware 4.3.3.124
 * @param    tool    GoSurfaceCountersunkHole object.
 * @return           A GoSurfaceCountersunkHoleCounterboreDepth counterbore depth measurement.
 */
GoFx(GoSurfaceCountersunkHoleCounterboreDepth) GoSurfaceCountersunkHole_CounterboreDepth(GoSurfaceCountersunkHole tool);

/**
 * Returns a GoSurfaceCountersunkHole tool Bevel Radius measurement object.
 *
 * @public           @memberof GoSurfaceCountersunkHole
 * @note             Supported with G2, G3
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoSurfaceCountersunkHole object.
 * @return           A GoSurfaceCountersunkHoleBevelRadius Bevel Radius measurement.
 */
GoFx(GoSurfaceCountersunkHoleBevelRadius) GoSurfaceCountersunkHole_BevelRadiusMeasurement(GoSurfaceCountersunkHole tool);

/**
 * Returns a GoSurfaceCountersunkHole tool Bevel Angle measurement object.
 *
 * @public           @memberof GoSurfaceCountersunkHole
 * @note             Supported with G2, G3
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoSurfaceCountersunkHole object.
 * @return           A GoSurfaceCountersunkHoleBevelAngle Bevel Angle measurement.
 */
GoFx(GoSurfaceCountersunkHoleBevelAngle) GoSurfaceCountersunkHole_BevelAngleMeasurement(GoSurfaceCountersunkHole tool);

/**
 * Returns a GoSurfaceCountersunkHole tool X Angle measurement object.
 *
 * @public           @memberof GoSurfaceCountersunkHole
 * @note             Supported with G2, G3
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoSurfaceCountersunkHole object.
 * @return           A GoSurfaceCountersunkHoleXAngle X angle measurement.
 */
GoFx(GoSurfaceCountersunkHoleXAngle) GoSurfaceCountersunkHole_XAngleMeasurement(GoSurfaceCountersunkHole tool);

/**
 * Returns a GoSurfaceCountersunkHole tool Y Angle measurement object.
 *
 * @public           @memberof GoSurfaceCountersunkHole
 * @note             Supported with G2, G3
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoSurfaceCountersunkHole object.
 * @return           A GoSurfaceCountersunkHoleYAngle Y angle measurement.
 */
GoFx(GoSurfaceCountersunkHoleYAngle) GoSurfaceCountersunkHole_YAngleMeasurement(GoSurfaceCountersunkHole tool);

/**
 * Returns a GoSurfaceCountersunkHole tool axis tilt measurement object.
 *
 * @public           @memberof GoSurfaceCountersunkHole
 * @note             Supported with G2, G3
 * @version          Introduced in firmware 4.5.3.57
 * @param    tool    GoSurfaceCountersunkHole object.
 * @return           A GoSurfaceCountersunkHoleAxisTilt axis tilt measurement.
 */
GoFx(GoSurfaceCountersunkHoleAxisTilt) GoSurfaceCountersunkHole_AxisTilt(GoSurfaceCountersunkHole tool);

/**
 * Returns a GoSurfaceCountersunkHole tool axis orientation measurement object.
 *
 * @public           @memberof GoSurfaceCountersunkHole
 * @note             Supported with G2, G3
 * @version          Introduced in firmware 4.5.3.57
 * @param    tool    GoSurfaceCountersunkHole object.
 * @return           A GoSurfaceCountersunkHoleAxisOrientation axis orientation measurement.
 */
GoFx(GoSurfaceCountersunkHoleAxisOrientation) GoSurfaceCountersunkHole_AxisOrientation(GoSurfaceCountersunkHole tool);

/**
* Returns a GoSurfaceCountersunkHoleCenterPoint tool center point feature.
*
* @public           @memberof GoSurfaceCountersunkHoleCenterPoint
* @note             Supported with G2, G3
* @version          Introduced in firmware 4.5.3.57
* @param    tool    GoSurfaceCountersunkHole object.
* @return           A GoSurfaceCountersunkHoleCenterPoint center point feature.
*/
GoFx(GoSurfaceCountersunkHoleCenterPoint) GoSurfaceCountersunkHole_CenterPoint(GoSurfaceCountersunkHoleCenterPoint tool);

/**
 * @class   GoSurfaceDim
 * @extends GoSurfaceTool
 * @note    Supported with G2, G3
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents a Surface dimension tool. all width/height/length/radius and x/y/z units are in mm, angles in degrees
 */
typedef GoSurfaceTool GoSurfaceDim; 

/** 
 * Gets the reference Surface feature.
 *
 * @public               @memberof GoSurfaceDim
 * @note                 Supported with G2, G3
 * @version              Introduced in firmware 4.4.4.14
 * @param    tool        GoSurfaceDim object.
 * @return               The reference Surface feature object.
 */
GoFx(GoSurfaceFeature) GoSurfaceDim_RefFeature(GoSurfaceDim tool);


/** 
 * Gets the non-reference Surface feature.
 *
 * @public               @memberof GoSurfaceDim
 * @note                 Supported with G2, G3
 * @version              Introduced in firmware 4.4.4.14
 * @param    tool        GoSurfaceDim object.
 * @return               The non-reference Surface feature object.
 */
GoFx(GoSurfaceFeature) GoSurfaceDim_Feature(GoSurfaceDim tool);

/**
 * Returns a GoSurfaceDim Width measurement object.
 *
 * @public              @memberof GoSurfaceDim
 * @note                Supported with G2, G3
 * @version             Introduced in firmware 4.4.4.14
 * @param    tool       GoSurfaceDim object.
 * @return              A GoSurfaceDimWidth measurement.
 */
GoFx(GoSurfaceDimWidth) GoSurfaceDim_WidthMeasurement(GoSurfaceDim tool);

/**
 * Returns a GoSurfaceDim Height measurement object.
 *
 * @public              @memberof GoSurfaceDim
 * @note                Supported with G2, G3
 * @version             Introduced in firmware 4.4.4.14
 * @param    tool       GoSurfaceDim object.
 * @return              A GoSurfaceDimHeight measurement.
 */
GoFx(GoSurfaceDimHeight) GoSurfaceDim_HeightMeasurement(GoSurfaceDim tool);

/**
 * Returns a GoSurfaceDim Length measurement object.
 *
 * @public              @memberof GoSurfaceDim
 * @note                Supported with G2, G3
 * @version             Introduced in firmware 4.4.4.14
 * @param    tool       GoSurfaceDim object.
 * @return              A GoSurfaceDimLength measurement.
 */
GoFx(GoSurfaceDimLength) GoSurfaceDim_LengthMeasurement(GoSurfaceDim tool);

/**
 * Returns a GoSurfaceDim Distance measurement object.
 *
 * @public              @memberof GoSurfaceDim
 * @note                Supported with G2, G3
 * @version             Introduced in firmware 4.4.4.14
 * @param    tool       GoSurfaceDim object.
 * @return              A GoSurfaceDimDistance measurement.
 */
GoFx(GoSurfaceDimDistance) GoSurfaceDim_DistanceMeasurement(GoSurfaceDim tool);

/**
 * Returns a GoSurfaceDim Plane Distance measurement object.
 *
 * @public              @memberof GoSurfaceDim
 * @note                Supported with G2, G3
 * @version             Introduced in firmware 4.4.4.14
 * @param    tool       GoSurfaceDim object.
 * @return              A GoSurfaceDimPlaneDistance measurement.
 */
GoFx(GoSurfaceDimPlaneDistance) GoSurfaceDim_PlaneDistanceMeasurement(GoSurfaceDim tool);

/**
 * Returns a GoSurfaceDim Center X measurement object.
 *
 * @public              @memberof GoSurfaceDim
 * @note                Supported with G2, G3
 * @version             Introduced in firmware 4.4.4.14
 * @param    tool       GoSurfaceDim object.
 * @return              A GoSurfaceDimCenterX measurement.
 */
GoFx(GoSurfaceDimCenterX) GoSurfaceDim_CenterXMeasurement(GoSurfaceDim tool);

/**
 * Returns a GoSurfaceDim Center Y measurement object.
 *
 * @public              @memberof GoSurfaceDim
 * @note                Supported with G2, G3
 * @version             Introduced in firmware 4.4.4.14
 * @param    tool       GoSurfaceDim object.
 * @return              A GoSurfaceDimCenterY measurement.
 */
GoFx(GoSurfaceDimCenterY) GoSurfaceDim_CenterYMeasurement(GoSurfaceDim tool);

/**
* Returns a GoSurfaceDim Center Z measurement object.
*
* @public              @memberof GoSurfaceDim
* @note                Supported with G2, G3
* @version             Introduced in firmware 4.4.4.14
* @param    tool       GoSurfaceDim object.
* @return              A GoSurfaceDimCenterZ measurement.
*/
GoFx(GoSurfaceDimCenterZ) GoSurfaceDim_CenterZMeasurement(GoSurfaceDim tool);

/**
* Returns a GoSurfaceDim center point feature object.
*
* @public              @memberof GoSurfaceDim
* @note                Supported with G2, G3
* @version             Introduced in firmware 4.4.4.14
* @param    tool       GoSurfaceDim object.
* @return              A GoSurfaceDimensionCenterPoint measurement.
*/
GoFx(GoSurfaceDimensionCenterPoint) GoSurfaceDim_CenterPoint(GoSurfaceDimensionCenterPoint tool);

/**
 * @class   GoSurfaceEllipse
 * @extends GoSurfaceTool
 * @note    Supported with G2, G3
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents a surface ellipse tool. all width/height/length/radius and x/y/z units are in mm, angles in degrees
 */
typedef GoSurfaceTool GoSurfaceEllipse; 

/** 
 * Enables or disables the tool region.
 *
 * @public                  @memberof GoSurfaceEllipse
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceEllipse object.
 * @param    enable         kTRUE to enable the tool region, kFALSE to disable it.
 * @return                  Operation status.
 */
GoFx(kStatus) GoSurfaceEllipse_EnableRegion(GoSurfaceEllipse tool, kBool enable);

/** 
 * Returns the enabled state of the tool region.
 *
 * @public                  @memberof GoSurfaceEllipse
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceEllipse object.
 * @return                  kTRUE if enabled and kFALSE if disabled.
 */
GoFx(kBool) GoSurfaceEllipse_RegionEnabled(GoSurfaceEllipse tool);

/** 
 * Gets the tool region.
 *
 * @public                  @memberof GoSurfaceEllipse
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceEllipse object.
 * @return                  A GoRegion3d object.
 */
GoFx(GoRegion3d) GoSurfaceEllipse_Region(GoSurfaceEllipse tool);

/** 
 * Sets the asymmetry detection type.
 *
 * @public                  @memberof GoSurfaceEllipse
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.4.4.14
 * @param    tool           GoSurfaceEllipse object.
 * @param    value          The asymmetry detection type to set.
 * @return                  Operation status.
 */
GoFx(kStatus) GoSurfaceEllipse_SetAsymmetryDetectionType(GoSurfaceEllipse tool, GoEllipseAsymmetryType value);

/** 
 * Gets the asymmetry detection type.
 *
 * @public                  @memberof GoSurfaceEllipse
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.4.4.14
 * @param    tool           GoSurfaceEllipse object.
 * @return                  The asymmetry detection type.
 */
GoFx(GoEllipseAsymmetryType) GoSurfaceEllipse_AsymmetryDetectionType(GoSurfaceEllipse tool);

/**
 * Returns a GoSurfaceEllipse Major measurement object.
 *
 * @public              @memberof GoSurfaceEllipse
 * @note                Supported with G2, G3
 * @version             Introduced in firmware 4.0.10.27
 * @param    tool       GoSurfaceEllipse object.
 * @return              A GoSurfaceEllipse Major measurement.
 */
GoFx(GoSurfaceEllipseMajor) GoSurfaceEllipse_MajorMeasurement(GoSurfaceEllipse tool);

/**
 * Returns a GoSurfaceEllipse Minor measurement object.
 *
 * @public              @memberof GoSurfaceEllipse
 * @note                Supported with G2, G3
 * @version             Introduced in firmware 4.0.10.27
 * @param    tool       GoSurfaceEllipse object.
 * @return              A GoSurfaceEllipse Minor measurement.
 */
GoFx(GoSurfaceEllipseMinor) GoSurfaceEllipse_MinorMeasurement(GoSurfaceEllipse tool);

/**
 * Returns a GoSurfaceEllipse Ratio measurement object.
 *
 * @public              @memberof GoSurfaceEllipse
 * @note                Supported with G2, G3
 * @version             Introduced in firmware 4.0.10.27
 * @param    tool       GoSurfaceEllipse object.
 * @return              A GoSurfaceEllipse Ratio measurement.
 */
GoFx(GoSurfaceEllipseRatio) GoSurfaceEllipse_RatioMeasurement(GoSurfaceEllipse tool);

/**
 * Returns a GoSurfaceEllipse Z Angle measurement object.
 *
 * @public              @memberof GoSurfaceEllipse
 * @note                Supported with G2, G3
 * @version             Introduced in firmware 4.0.10.27
 * @param    tool       GoSurfaceEllipse object.
 * @return              A GoSurfaceEllipse Z Angle measurement.
 */
GoFx(GoSurfaceEllipseZAngle) GoSurfaceEllipse_ZAngleMeasurement(GoSurfaceEllipse tool);

/**
* Returns a GoSurfaceEllipse point feature.
*
* @public              @memberof GoSurfaceEllipse
* @note                Supported with G2, G3
* @version             Introduced in firmware 4.0.10.27
* @param    tool       GoSurfaceEllipse object.
* @return              A GoSurfaceEllipseCenterPoint point feature
*/
GoFx(GoSurfaceEllipseCenterPoint) GoSurfaceEllipse_CenterPoint(GoSurfaceEllipse tool);

/**
* Returns a GoSurfaceEllipse major axis line feature.
*
* @public              @memberof GoSurfaceEllipse
* @note                Supported with G2, G3
* @version             Introduced in firmware 4.0.10.27
* @param    tool       GoSurfaceEllipse object.
* @return              A GoSurfaceEllipseMajorAxisLine axis line feature..
*/
GoFx(GoSurfaceEllipseMajorAxisLine) GoSurfaceEllipse_MajorAxisLine(GoSurfaceEllipse tool);

/**
* Returns a GoSurfaceEllipse major axis line feature.
*
* @public              @memberof GoSurfaceEllipse
* @note                Supported with G2, G3
* @version             Introduced in firmware 4.0.10.27
* @param    tool       GoSurfaceEllipse object.
* @return              A GoSurfaceEllipseMinorAxisLine major axis line feature.
*/
GoFx(GoSurfaceEllipseMinorAxisLine) GoSurfaceEllipse_MinorAxisLine(GoSurfaceEllipse tool);


/**
 * @class   GoSurfaceHole
 * @extends GoSurfaceTool
 * @note    Supported with G2, G3
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents a surface hole tool. all width/height/length/radius and x/y/z units are in mm, angles in degrees
 */
typedef GoSurfaceTool GoSurfaceHole; 

/** 
 * Gets the current nominal radius value.
 *
 * @public                  @memberof GoSurfaceHole
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceHole object.
 * @return                  The nominal radius value. 
 */
GoFx(k64f) GoSurfaceHole_NominalRadius(GoSurfaceHole tool);

/** 
 * Sets the nominal radius value.
 *
 * @public                  @memberof GoSurfaceHole
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceHole object.
 * @param    nominalRadius  Nominal radius value to set.
 * @return                  Operation status.
 */
GoFx(kStatus) GoSurfaceHole_SetNominalRadius(GoSurfaceHole tool, k64f nominalRadius);

/** 
 * Gets the current radius tolerance value.
 *
 * @public                  @memberof GoSurfaceHole
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceHole object.
 * @return                  The radius tolerance value.
 */
GoFx(k64f) GoSurfaceHole_RadiusTolerance(GoSurfaceHole tool);

/** 
 * Sets the radius tolerance value.
 *
 * @public                      @memberof GoSurfaceHole
 * @note                        Supported with G2, G3
 * @version                     Introduced in firmware 4.0.10.27
 * @param    tool               GoSurfaceHole object.
 * @param    radiusTolerance    The radius tolerance value to set.
 * @return                      Operation status.
 */
GoFx(kStatus) GoSurfaceHole_SetRadiusTolerance(GoSurfaceHole tool, k64f radiusTolerance);

/** 
 * Gets the enabled state of partial detection.
 *
 * @public                  @memberof GoSurfaceHole
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceHole object.
 * @return                  kTRUE if enabled and kFALSE if disabled.
 */
GoFx(kBool) GoSurfaceHole_PartialDetectionEnabled(GoSurfaceHole tool);

/** 
 * Sets the enabled state of partial detection.
 *
 * @public                  @memberof GoSurfaceHole
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceHole object.
 * @param    enable         kTRUE to enable partial detection and kFALSE to disable it.
 * @return                  Operation status.
 */
GoFx(kStatus) GoSurfaceHole_EnablePartialDetection(GoSurfaceHole tool, kBool enable);

/** 
 * Gets the enabled state of the tool region.
 *
 * @public                  @memberof GoSurfaceHole
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceHole object.
 * @return                  kTRUE if enabled and kFALSE if disabled.
 */
GoFx(kBool) GoSurfaceHole_RegionEnabled(GoSurfaceHole tool);

/** 
 * Sets the enabled state of the tool region.
 *
 * @public                  @memberof GoSurfaceHole
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceHole object.
 * @param    enable         kTRUE to enable the tool region and kFALSE to disable it.
 * @return                  Operation status.
 */
GoFx(kStatus) GoSurfaceHole_EnableRegion(GoSurfaceHole tool, kBool enable);

/** 
 * Returns the tool's region object.
 *
 * @public                  @memberof GoSurfaceHole
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceHole object.
 * @return                  A GoRegion3d object.
 */
GoFx(GoRegion3d) GoSurfaceHole_Region(GoSurfaceHole tool);

/** 
 * Gets the enabled state of reference regions.
 *
 * @public                  @memberof GoSurfaceHole
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceHole object.
 * @return                  kTRUE if enabled and kFALSE if disabled.
 */
GoFx(kBool) GoSurfaceHole_RefRegionsEnabled(GoSurfaceHole tool);

/** 
 * Sets the enabled state of reference regions.
 *
 * @public                  @memberof GoSurfaceHole
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceHole object.
 * @param    enable         kTRUE to enable reference regions and kFALSE to disable it.
 * @return                  Operation status.
 */
GoFx(kStatus) GoSurfaceHole_EnableRefRegions(GoSurfaceHole tool, kBool enable);

/** 
 * Gets the reference region count.
 *
 * @public                  @memberof GoSurfaceHole
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceHole object.
 * @return                  The reference region count.
 */
GoFx(kSize) GoSurfaceHole_RefRegionCount(GoSurfaceHole tool);

/** 
 * Sets the reference region count.
 *
 * @public                  @memberof GoSurfaceHole
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceHole object.
 * @param    count          The reference region count.
 * @return                  Operation status.
 * @see                     GO_SURFACE_HOLE_MAX_REF_REGIONS
 */
GoFx(kStatus) GoSurfaceHole_SetRefRegionCount(GoSurfaceHole tool, kSize count);

/** 
 * Gets a reference region object at the given index.
 *
 * @public                  @memberof GoSurfaceHole
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceHole object.
 * @param    index          The index with which to retrieve a reference region.
 * @return                  A GoSurfaceRegion2d object.
 * @see                     GoSurfaceHole_RefRegionCount
 */
GoFx(GoSurfaceRegion2d) GoSurfaceHole_RefRegionAt(GoSurfaceHole tool, kSize index);

/** 
 * Gets the enabled state of auto-tilt.
 *
 * @public                  @memberof GoSurfaceHole
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceHole object.
 * @return                  kTRUE if enabled and kFALSE if disabled.
 */
GoFx(kBool) GoSurfaceHole_AutoTiltEnabled(GoSurfaceHole tool);

/** 
 * Sets the enabled state of auto-tilt.
 *
 * @public                  @memberof GoSurfaceHole
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceHole object.
 * @param    enable         kTRUE to enable it and kFALSE to disable it.
 * @return                  Operation status.
 */
GoFx(kStatus) GoSurfaceHole_EnableAutoTilt(GoSurfaceHole tool, kBool enable);

/** 
 * Gets the tilt X-angle value.
 *
 * @public                  @memberof GoSurfaceHole
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceHole object.
 * @return                  Tilt X-angle value.
 */
GoFx(k64f) GoSurfaceHole_TiltXAngle(GoSurfaceHole tool);

/** 
 * Sets the tilt X-angle value.
 *
 * @public                  @memberof GoSurfaceHole
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceHole object.
 * @param    value          The tilt X-angle value to set.
 * @return                  Operation status.
 */
GoFx(kStatus) GoSurfaceHole_SetTiltXAngle(GoSurfaceHole tool, k64f value);

/** 
 * Gets the tilt Y-angle value.
 *
 * @public                  @memberof GoSurfaceHole
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceHole object.
 * @return                  Tilt Y-angle value.
 */
GoFx(k64f) GoSurfaceHole_TiltYAngle(GoSurfaceHole tool);

/** 
 * Sets the tilt Y-angle value.
 *
 * @public                  @memberof GoSurfaceHole
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceHole object.
 * @param    value          The tilt Y-angle value to set.
 * @return                  Operation status.
 */
GoFx(kStatus) GoSurfaceHole_SetTiltYAngle(GoSurfaceHole tool, k64f value);

/** 
 * Gets the enabled state of the depth limit.
 *
 * @public                  @memberof GoSurfaceHole
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.3.3.124
 * @param    tool           GoSurfaceHole object.
 * @return                  kTRUE if enabled and kFALSE if disabled.
 */
GoFx(kBool) GoSurfaceHole_DepthLimitEnabled(GoSurfaceHole tool);

/** 
 * Sets the enabled state of the depth limit.
 *
 * @public                  @memberof GoSurfaceHole
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.3.3.124
 * @param    tool           GoSurfaceHole object.
 * @param    enable         kTRUE to enable it and kFALSE to disable it.
 * @return                  Operation status.
 */
GoFx(kStatus) GoSurfaceHole_EnableDepthLimit(GoSurfaceHole tool, kBool enable);

/** 
 * Gets the depth limit value.
 *
 * @public                  @memberof GoSurfaceHole
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.3.3.124
 * @param    tool           GoSurfaceHole object.
 * @return                  Depth limit value.

 */
GoFx(k64f) GoSurfaceHole_DepthLimit(GoSurfaceHole tool);

/** 
 * Sets the depth limit value.
 *
 * @public                  @memberof GoSurfaceHole
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.3.3.124
 * @param    tool           GoSurfaceHole object.
 * @param    value          The depth limit value to set.
 * @return                  Operation status.
 * @see                     GoSurfaceHole_DepthLimitEnabled, GoSurfaceHole_EnableDepthLimit
 */
GoFx(kStatus) GoSurfaceHole_SetDepthLimit(GoSurfaceHole tool, k64f value);

/**
 * Returns a GoSurfaceHole X measurement object.
 *
 * @public           @memberof GoSurfaceHole
 * @note             Supported with G2, G3
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoSurfaceHole object.
 * @return           A GoSurfaceHoleX X measurement.
 */
GoFx(GoSurfaceHoleX) GoSurfaceHole_XMeasurement(GoSurfaceHole tool);

/**
 * Returns a GoSurfaceHole Y measurement object.
 *
 * @public           @memberof GoSurfaceHole
 * @note             Supported with G2, G3
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoSurfaceHole object.
 * @return           A GoSurfaceHoleY Y measurement.
 */
GoFx(GoSurfaceHoleY) GoSurfaceHole_YMeasurement(GoSurfaceHole tool);

/**
 * Returns a GoSurfaceHole Z measurement object.
 *
 * @public           @memberof GoSurfaceHole
 * @note             Supported with G2, G3
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoSurfaceHole object.
 * @return           A GoSurfaceHoleZ Z measurement.
 */
GoFx(GoSurfaceHoleZ) GoSurfaceHole_ZMeasurement(GoSurfaceHole tool);

/**
 * Returns a GoSurfaceHole Radius measurement object.
 *
 * @public           @memberof GoSurfaceHole
 * @note             Supported with G2, G3
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoSurfaceHole object.
 * @return           A GoSurfaceHoleRadius Radius measurement.
 */
GoFx(GoSurfaceHoleRadius) GoSurfaceHole_RadiusMeasurement(GoSurfaceHole tool);

/**
* Returns a GoSurfaceHole point feature object.
*
* @public           @memberof GoSurfaceHole
* @note             Supported with G2, G3
* @version          Introduced in firmware 4.0.10.27
* @param    tool    GoSurfaceHole object.
* @return           A GoSurfaceHoleCenterPoint point feature.
*/
GoFx(GoSurfaceHoleCenterPoint) GoSurfaceHole_Point(GoSurfaceHole tool);


/**
 * @class   GoSurfaceOpening
 * @extends GoSurfaceTool
 * @note    Supported with G2, G3
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents a surface opening tool. all width/height/length/radius and x/y/z units are in mm, angles in degrees
 */
typedef GoSurfaceTool GoSurfaceOpening; 

GoFx(kStatus) GoSurfaceOpening_SetType(GoSurfaceOpening tool, GoSurfaceOpeningType type);

/** 
 * Gets the surface opening type.
 *
 * @public                  @memberof GoSurfaceOpening
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceOpening object.
 * @return                  The surface opening type.
 */
GoFx(GoSurfaceOpeningType) GoSurfaceOpening_Type(GoSurfaceOpening tool);

/** 
 * Gets the nominal width.
 *
 * @public                  @memberof GoSurfaceOpening
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceOpening object.
 * @return                  The nominal width (in mm).
 */
GoFx(k64f) GoSurfaceOpening_NominalWidth(GoSurfaceOpening tool);

/** 
 * Sets the nominal width.
 *
 * @public                  @memberof GoSurfaceOpening
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceOpening object.
 * @param    value          The nominal width to set (in mm).
 * @return                  Operation status.
 */
GoFx(kStatus) GoSurfaceOpening_SetNominalWidth(GoSurfaceOpening tool, k64f value);

/** 
 * Gets the nominal length.
 *
 * @public                  @memberof GoSurfaceOpening
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceOpening object.
 * @return                  The nominal length (in mm).
 */
GoFx(k64f) GoSurfaceOpening_NominalLength(GoSurfaceOpening tool);

/** 
 * Sets the nominal length.
 *
 * @public                  @memberof GoSurfaceOpening
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceOpening object.
 * @param    value          The nominal length to set (in mm).
 * @return                  Operation status.
 */
GoFx(kStatus) GoSurfaceOpening_SetNominalLength(GoSurfaceOpening tool, k64f value);

/** 
 * Gets the nominal angle.
 *
 * @public                  @memberof GoSurfaceOpening
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceOpening object.
 * @return                  The nominal angle (in degrees).
 */
GoFx(k64f) GoSurfaceOpening_NominalAngle(GoSurfaceOpening tool);

/** 
 * Sets the nominal angle.
 *
 * @public                  @memberof GoSurfaceOpening
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceOpening object.
 * @param    value          The nominal angle to set (in degrees).
 * @return                  Operation status.
 */
GoFx(kStatus) GoSurfaceOpening_SetNominalAngle(GoSurfaceOpening tool, k64f value);

/** 
 * Gets the nominal radius.
 *
 * @public                  @memberof GoSurfaceOpening
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceOpening object.
 * @return                  The nominal radius (in mm).
 */
GoFx(k64f) GoSurfaceOpening_NominalRadius(GoSurfaceOpening tool);

/** 
 * Sets the nominal radius.
 *
 * @public                  @memberof GoSurfaceOpening
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceOpening object.
 * @param    value          The nominal radius to set (in mm).
 * @return                  Operation status.
 */
GoFx(kStatus) GoSurfaceOpening_SetNominalRadius(GoSurfaceOpening tool, k64f value);

/** 
 * Gets the width tolerance.
 *
 * @public                  @memberof GoSurfaceOpening
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceOpening object.
 * @return                  The width tolerance (in mm).
 */
GoFx(k64f) GoSurfaceOpening_WidthTolerance(GoSurfaceOpening tool);

/** 
 * Sets the width tolerance.
 *
 * @public                  @memberof GoSurfaceOpening
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceOpening object.
 * @param    value          The width tolerance to set (in mm).
 * @return                  Operation status.
 */
GoFx(kStatus) GoSurfaceOpening_SetWidthTolerance(GoSurfaceOpening tool, k64f value);

/** 
 * Gets the length tolerance.
 *
 * @public                  @memberof GoSurfaceOpening
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceOpening object.
 * @return                  The length tolerance (in mm).
 */
GoFx(k64f) GoSurfaceOpening_LengthTolerance(GoSurfaceOpening tool);

/** 
 * Sets the length tolerance.
 *
 * @public                  @memberof GoSurfaceOpening
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceOpening object.
 * @param    value          The length tolerance to set (in mm).
 * @return                  Operation status.
 */
GoFx(kStatus) GoSurfaceOpening_SetLengthTolerance(GoSurfaceOpening tool, k64f value);

/** 
 * Gets the angle tolerance.
 *
 * @public                  @memberof GoSurfaceOpening
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceOpening object.
 * @return                  The angle tolerance (in degrees).
 */
GoFx(k64f) GoSurfaceOpening_AngleTolerance(GoSurfaceOpening tool);

/** 
 * Sets the angle tolerance.
 *
 * @public                  @memberof GoSurfaceOpening
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceOpening object.
 * @param    value          The angle tolerance to set (in degrees).
 * @return                  Operation status.
 */
GoFx(kStatus) GoSurfaceOpening_SetAngleTolerance(GoSurfaceOpening tool, k64f value);

/** 
 * Gets the enabled state of partial detection.
 *
 * @public                  @memberof GoSurfaceOpening
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceOpening object.
 * @return                  kTRUE if partial detection is enabled and kFALSE otherwise.
 */
GoFx(kBool) GoSurfaceOpening_PartialDetectionEnabled(GoSurfaceOpening tool);

/** 
 * Sets the enabled state of partial detection.
 *
 * @public                  @memberof GoSurfaceOpening
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceOpening object.
 * @param    enable         kTRUE to enable partial detection and kFALSE to disable it.
 * @return                  Operation status.
 */
GoFx(kStatus) GoSurfaceOpening_EnablePartialDetection(GoSurfaceOpening tool, kBool enable);

/** 
 * Gets the enabled state of the tool region.
 *
 * @public                  @memberof GoSurfaceOpening
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceOpening object.
 * @return                  kTRUE if the tool region is enabled and kFALSE otherwise.
 */
GoFx(kBool) GoSurfaceOpening_RegionEnabled(GoSurfaceOpening tool);

/** 
 * Sets the enabled state of the tool region.
 *
 * @public                  @memberof GoSurfaceOpening
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceOpening object.
 * @param    enable         kTRUE to enable the tool region and kFALSE to disable it.
 * @return                  Operation status.
 */
GoFx(kStatus) GoSurfaceOpening_EnableRegion(GoSurfaceOpening tool, kBool enable);

/** 
 * Returns the region object for the tool.
 *
 * @public                  @memberof GoSurfaceOpening
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceOpening object.
 * @return                  A GoRegion3d object.
 */
GoFx(GoRegion3d) GoSurfaceOpening_Region(GoSurfaceOpening tool);

/** 
 * Gets the enabled state of reference regions.
 *
 * @public                  @memberof GoSurfaceOpening
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceOpening object.
 * @return                  kTRUE if reference regions are enabled and kFALSE otherwise.
 */
GoFx(kBool) GoSurfaceOpening_RefRegionsEnabled(GoSurfaceOpening tool);

/** 
 * Sets the enabled state of reference regions.
 *
 * @public                  @memberof GoSurfaceOpening
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceOpening object.
 * @param    enable         kTRUE to enable reference regions and kFALSE to disable it.
 * @return                  Operation status.
 */
GoFx(kStatus) GoSurfaceOpening_EnableRefRegions(GoSurfaceOpening tool, kBool enable);

/** 
 * Gets the reference region count.
 *
 * @public                  @memberof GoSurfaceOpening
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceOpening object.
 * @return                  The count of reference regions.
 */
GoFx(kSize) GoSurfaceOpening_RefRegionCount(GoSurfaceOpening tool);

/** 
 * Sets the reference region count.
 *
 * @public                  @memberof GoSurfaceOpening
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceOpening object.
 * @param    count          The reference region count to set.
 * @return                  Operation status.
 * @see                     GO_SURFACE_OPENING_MAX_REF_REGIONS
 */
GoFx(kStatus) GoSurfaceOpening_SetRefRegionCount(GoSurfaceOpening tool, kSize count);

/** 
 * Gets the reference region object at the specified index.
 *
 * @public                  @memberof GoSurfaceOpening
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceOpening object.
 * @param    index          The index with which to retrieve a reference region.
 * @return                  A GoSurfaceRegion2d object or kNULL if the index is invalid.
 * @see                     GoSurfaceOpening_RefRegionCount
 */
GoFx(GoSurfaceRegion2d) GoSurfaceOpening_RefRegionAt(GoSurfaceOpening tool, kSize index);

/** 
 * Gets the enabled state of auto-tilt.
 *
 * @public                  @memberof GoSurfaceOpening
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceOpening object.
 * @return                  kTRUE if auto-tilt is enabled and kFALSE otherwise.
 */
GoFx(kBool) GoSurfaceOpening_AutoTiltEnabled(GoSurfaceOpening tool);

/** 
 * Sets the enabled state of auto-tilt.
 *
 * @public                  @memberof GoSurfaceOpening
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceOpening object.
 * @param    enable         kTRUE to enable auto-tilt and kFALSE to disable it.
 * @return                  Operation status.
 */
GoFx(kStatus) GoSurfaceOpening_EnableAutoTilt(GoSurfaceOpening tool, kBool enable);

/** 
 * Gets the tilt X-angle.
 *
 * @public                  @memberof GoSurfaceOpening
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceOpening object.
 * @return                  The tilt X-angle.
 */
GoFx(k64f) GoSurfaceOpening_TiltXAngle(GoSurfaceOpening tool);

/** 
 * Sets the tilt X-angle.
 *
 * @public                  @memberof GoSurfaceOpening
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceOpening object.
 * @param    value          The tilt X-angle to set.
 * @return                  Operation status.
 */
GoFx(kStatus) GoSurfaceOpening_SetTiltXAngle(GoSurfaceOpening tool, k64f value);

/** 
 * Gets the tilt Y-angle.
 *
 * @public                  @memberof GoSurfaceOpening
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceOpening object.
 * @return                  The tilt Y-angle.
 */
GoFx(k64f) GoSurfaceOpening_TiltYAngle(GoSurfaceOpening tool);

/** 
 * Sets the tilt Y-angle.
 *
 * @public                  @memberof GoSurfaceOpening
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceOpening object.
 * @param    value          The tilt Y-angle to set.
 * @return                  Operation status.
 */
GoFx(kStatus) GoSurfaceOpening_SetTiltYAngle(GoSurfaceOpening tool, k64f value);

/** 
 * Gets the enabled state of the depth limit.
 *
 * @public                  @memberof GoSurfaceOpening
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.3.3.124
 * @param    tool           GoSurfaceOpening object.
 * @return                  kTRUE if enabled and kFALSE if disabled.
 */
GoFx(kBool) GoSurfaceOpening_DepthLimitEnabled(GoSurfaceOpening tool);

/** 
 * Sets the enabled state of the depth limit.
 *
 * @public                  @memberof GoSurfaceOpening
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.3.3.124
 * @param    tool           GoSurfaceOpening object.
 * @param    enable         kTRUE to enable it and kFALSE to disable it.
 * @return                  Operation status.
 */
GoFx(kStatus) GoSurfaceOpening_EnableDepthLimit(GoSurfaceOpening tool, kBool enable);

/** 
 * Gets the depth limit value.
 *
 * @public                  @memberof GoSurfaceOpening
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.3.3.124
 * @param    tool           GoSurfaceOpening object.
 * @return                  Depth limit value.

 */
GoFx(k64f) GoSurfaceOpening_DepthLimit(GoSurfaceOpening tool);

/** 
 * Sets the depth limit value.
 *
 * @public                  @memberof GoSurfaceOpening
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.3.3.124
 * @param    tool           GoSurfaceOpening object.
 * @param    value          The depth limit value to set.
 * @return                  Operation status.
 * @see                     GoSurfaceOpening_DepthLimitEnabled, GoSurfaceOpening_EnableDepthLimit
 */
GoFx(kStatus) GoSurfaceOpening_SetDepthLimit(GoSurfaceOpening tool, k64f value);


/**
 * Returns a GoSurfaceOpening X measurement object.
 *
 * @public           @memberof GoSurfaceOpening
 * @note             Supported with G2, G3
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoSurfaceOpening object.
 * @return           A GoSurfaceOpeningX X measurement.
 */
GoFx(GoSurfaceOpeningX) GoSurfaceOpening_XMeasurement(GoSurfaceOpening tool);

/**
 * Returns a GoSurfaceOpening Y measurement object.
 *
 * @public           @memberof GoSurfaceOpening
 * @note             Supported with G2, G3
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoSurfaceOpening object.
 * @return           A GoSurfaceOpeningY Y measurement.
 */
GoFx(GoSurfaceOpeningY) GoSurfaceOpening_YMeasurement(GoSurfaceOpening tool);

/**
 * Returns a GoSurfaceOpening Z measurement object.
 *
 * @public           @memberof GoSurfaceOpening
 * @note             Supported with G2, G3
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoSurfaceOpening object.
 * @return           A GoSurfaceOpening Z measurement.
 */
GoFx(GoSurfaceOpeningZ) GoSurfaceOpening_ZMeasurement(GoSurfaceOpening tool);

/**
 * Returns a GoSurfaceOpening Width measurement object.
 *
 * @public           @memberof GoSurfaceOpening
 * @note             Supported with G2, G3
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoSurfaceOpening object.
 * @return           A GoSurfaceOpeningWidth Width measurement.
 */
GoFx(GoSurfaceOpeningWidth) GoSurfaceOpening_WidthMeasurement(GoSurfaceOpening tool);

/**
 * Returns a GoSurfaceOpening Length measurement object.
 *
 * @public           @memberof GoSurfaceOpening
 * @note             Supported with G2, G3
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoSurfaceOpening object.
 * @return           A GoSurfaceOpeningLength Length measurement.
 */
GoFx(GoSurfaceOpeningLength) GoSurfaceOpening_LengthMeasurement(GoSurfaceOpening tool);

/**
 * Returns a GoSurfaceOpening Angle measurement object.
 *
 * @public           @memberof GoSurfaceOpening
 * @note             Supported with G2, G3
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoSurfaceOpening object.
 * @return           A GoSurfaceOpeningAngle Angle measurement.
 */
GoFx(GoSurfaceOpeningAngle) GoSurfaceOpening_AngleMeasurement(GoSurfaceOpening tool);

/**
* Returns a GoSurfaceOpening center point feature object.
*
* @public           @memberof GoSurfaceOpening
* @note             Supported with G2, G3
* @version          Introduced in firmware 4.0.10.27
* @param    tool    GoSurfaceOpening object.
* @return           A GoSurfaceOpeningCenterPoint center point feature.
*/
GoFx(GoSurfaceOpeningCenterPoint) GoSurfaceOpening_CenterPoint(GoSurfaceHole tool);

/**
 * @class   GoSurfacePlane
 * @extends GoSurfaceTool
 * @note    Supported with G2, G3
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents a surface plane tool.all width/height/length/radius and x/y/z units are in mm, angles in degrees
 */
typedef GoSurfaceTool GoSurfacePlane; 

/** 
 * Gets the enabled state of the reference regions.
 *
 * @public                  @memberof GoSurfacePlane
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfacePlane object.
 * @return                  kTRUE if the tool region is enabled and kFALSE otherwise.
 */
GoFx(kBool) GoSurfacePlane_RegionsEnabled(GoSurfacePlane tool);

/** 
 * Sets the enabled state of the reference regions.
 *
 * @public                  @memberof GoSurfacePlane
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfacePlane object.
 * @param    enable         kTRUE to enable regions and kFALSE to disable them.
 * @return                  Operation status.
 */
GoFx(kStatus) GoSurfacePlane_EnableRegions(GoSurfacePlane tool, kBool enable);

/** 
 * Gets the tool's region count.
 *
 * @public                  @memberof GoSurfacePlane
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfacePlane object.
 * @return                  The number of regions in the tool.
 */
GoFx(kSize) GoSurfacePlane_RegionCount(GoSurfacePlane tool);

/** 
 * Sets the tool region count.
 *
 * @public                  @memberof GoSurfacePlane
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfacePlane object.
 * @param    count          The region count to set.
 * @return                  Operation status.
 */
GoFx(kStatus) GoSurfacePlane_SetRegionCount(GoSurfacePlane tool, kSize count);

/** 
 * Gets a region at the specified index.
 *
 * @public                  @memberof GoSurfacePlane
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfacePlane object.
 * @param    index          The index with which to return a tool region.
 * @return                  A GoRegion3d object or kNULL if the index is in invalid.
 * @see                     GoSurfacePlane_RegionCount
 */
GoFx(GoRegion3d) GoSurfacePlane_RegionAt(GoSurfacePlane tool, kSize index);

/**
 * Returns a GoSurfacePlane X Angle measurement object.
 *
 * @public           @memberof GoSurfacePlane
 * @note             Supported with G2, G3
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoSurfacePlane object.
 * @return           A GoSurfacePlaneXAngle X Angle measurement.
 */
GoFx(GoSurfacePlaneXAngle) GoSurfacePlane_XAngleMeasurement(GoSurfacePlane tool);

/**
 * Returns a GoSurfacePlane Y Angle measurement object.
 *
 * @public           @memberof GoSurfacePlane
 * @note             Supported with G2, G3
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoSurfacePlane object.
 * @return           A GoSurfacePlaneYAngle Y Angle measurement.
 */
GoFx(GoSurfacePlaneYAngle) GoSurfacePlane_YAngleMeasurement(GoSurfacePlane tool);

/**
 * Returns a GoSurfacePlane Z Offset measurement object.
 *
 * @public           @memberof GoSurfacePlane
 * @note             Supported with G2, G3
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoSurfacePlane object.
 * @return           A GoSurfacePlaneZOffset Z Offset measurement.
 */
GoFx(GoSurfacePlaneZOffset) GoSurfacePlane_ZOffsetMeasurement(GoSurfacePlane tool);

/**
 * Returns a GoSurfacePlane Standard Deviation measurement object.
 *
 * @public           @memberof GoSurfacePlane
 * @note             Supported with G2, G3
 * @version          Introduced in firmware 4.4.4.14
 * @param    tool    GoSurfacePlane object.
 * @return           A GoSurfacePlane Standard Deviation measurement.
 */
GoFx(GoSurfacePlaneStdDev) GoSurfacePlane_StdDevMeasurement(GoSurfacePlane tool);

/**
 * Returns a GoSurfacePlane Minimum Error measurement object.
 *
 * @public           @memberof GoSurfacePlane
 * @note             Supported with G2, G3
 * @version          Introduced in firmware 4.4.4.14
 * @param    tool    GoSurfacePlane object.
 * @return           A GoSurfacePlaneMinError Minimum Error measurement.
 */
GoFx(GoSurfacePlaneMinError) GoSurfacePlane_MinErrorMeasurement(GoSurfacePlane tool);

/**
 * Returns a GoSurfacePlane Maximum Error measurement object.
 *
 * @public           @memberof GoSurfacePlane
 * @note             Supported with G2, G3
 * @version          Introduced in firmware 4.4.4.14
 * @param    tool    GoSurfacePlane object.
 * @return           A GoSurfacePlaneMaxError Maximum Error measurement.
 */
GoFx(GoSurfacePlaneMaxError) GoSurfacePlane_MaxErrorMeasurement(GoSurfacePlane tool);

/**
* Returns a GoSurfacePlane X Normal measurement object.
*
* @public           @memberof GoSurfacePlane
* @note             Supported with G2, G3
* @version          Introduced in firmware 4.6.0.49
* @param    tool    GoSurfacePlane object.
* @return           A GoSurfacePlaneXNormal X Normal measurement.
*/
GoFx(GoSurfacePlaneXNormal) GoSurfacePlane_XNormalMeasurement(GoSurfacePlane tool);
/**
* Returns a GoSurfacePlane Y Normal measurement object.
*
* @public           @memberof GoSurfacePlane
* @note             Supported with G2, G3
* @version          Introduced in firmware 4.6.0.49
* @param    tool    GoSurfacePlane object.
* @return           A GoSurfacePlane Y Normal measurement.
*/
GoFx(GoSurfacePlaneYNormal) GoSurfacePlane_YNormalMeasurement(GoSurfacePlane tool);
/**
* Returns a GoSurfacePlane X Normal measurement object.
*
* @public           @memberof GoSurfacePlane
* @note             Supported with G2, G3
* @version          Introduced in firmware 4.6.0.49
* @param    tool    GoSurfacePlane object.
* @return           A GoSurfacePlane X Normal measurement.
*/
GoFx(GoSurfacePlaneZNormal) GoSurfacePlane_ZNormalMeasurement(GoSurfacePlane tool);
/**
* Returns a GoSurfacePlane Distance measurement object.
*
* @public           @memberof GoSurfacePlane
* @note             Supported with G2, G3
* @version          Introduced in firmware 4.6.0.49
* @param    tool    GoSurfacePlane object.
* @return           A GoSurfacePlaneDistance Distance measurement.
*/
GoFx(GoSurfacePlaneDistance) GoSurfacePlane_DistanceMeasurement(GoSurfacePlane tool);

/**
* Returns a GoSurfacePlane plane feature object.
*
* @public           @memberof GoSurfacePlane
* @note             Supported with G2, G3
* @version          Introduced in firmware 4.7.2.x
* @param    tool    GoSurfacePlane object.
* @return           A GoSurfacePlanePlane plane feature.
*/
GoFx(GoSurfacePlanePlane) GoSurfacePlane_Plane(GoSurfacePlane tool);

/**
 * @class   GoSurfacePosition
 * @extends GoSurfaceTool
 * @note    Supported with G2, G3
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents a surface position tool. all width/height/length/radius and x/y/z units are in mm, angles in degrees
 */
typedef GoSurfaceTool GoSurfacePosition; 

GoFx(GoSurfaceFeature) GoSurfacePosition_Feature(GoSurfacePosition tool);

/**
 * Returns a GoSurfacePosition X measurement object.
 *
 * @public           @memberof GoSurfacePosition
 * @note             Supported with G2, G3
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoSurfacePosition object.
 * @return           A GoSurfacePositionX X measurement.
 */
GoFx(GoSurfacePositionX) GoSurfacePosition_XMeasurement(GoSurfacePosition tool);

/**
 * Returns a GoSurfacePosition Y measurement object.
 *
 * @public           @memberof GoSurfacePosition
 * @note             Supported with G2, G3
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoSurfacePosition object.
 * @return           A GoSurfacePosition Y measurement.
 */
GoFx(GoSurfacePositionY) GoSurfacePosition_YMeasurement(GoSurfacePosition tool);

/**
 * Returns a GoSurfacePosition Z measurement object.
 *
 * @public           @memberof GoSurfacePosition
 * @note             Supported with G2, G3
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoSurfacePosition object.
 * @return           A GoSurfacePosition Z measurement.
 */
GoFx(GoSurfacePositionZ) GoSurfacePosition_ZMeasurement(GoSurfacePosition tool);

/**
* Returns a GoSurfacePosition point feature object.
*
* @public           @memberof GoSurfacePosition
* @note             Supported with G2, G3
* @version          Introduced in firmware 4.0.10.27
* @param    tool    GoSurfacePosition object.
* @return           A GoSurfacePositionPoint point feature.
*/
GoFx(GoSurfacePositionPoint) GoSurfacePosition_Point(GoSurfaceHole tool);

/**
* @class   GoSurfaceRivet
* @extends GoSurfaceTool
* @note    Supported with G2, G3
* @ingroup GoSdk-SurfaceTools
* @brief   Represents a surface rivet tool.
*/
typedef GoSurfaceTool GoSurfaceRivet;

/**
 * @class   GoSurfaceStud
 * @extends GoSurfaceTool
 * @note    Supported with G2, G3
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents a surface stud tool. all width/height/length/radius and x/y/z units are in mm, angles in degrees
 */
typedef GoSurfaceTool GoSurfaceStud; 

/** 
 * Returns the stud radius value.
 *
 * @public                  @memberof GoSurfaceStud
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceStud object.
 * @return                  The stud radius value.
 */
GoFx(k64f) GoSurfaceStud_StudRadius(GoSurfaceStud tool);

/** 
 * Sets the stud radius value.
 *
 * @public                  @memberof GoSurfaceStud
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceStud object.
 * @param    value          The stud radius value to set.
 * @return                  Operation status.
 */
GoFx(kStatus) GoSurfaceStud_SetStudRadius(GoSurfaceStud tool, k64f value);

/** 
 * Returns the stud height value.
 *
 * @public                  @memberof GoSurfaceStud
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceStud object.
 * @return                  The stud height value.
 */
GoFx(k64f) GoSurfaceStud_StudHeight(GoSurfaceStud tool);

/** 
 * Sets the stud height value.
 *
 * @public                  @memberof GoSurfaceStud
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceStud object.
 * @param    value          The stud height value to set.
 * @return                  Operation status.
 */
GoFx(kStatus) GoSurfaceStud_SetStudHeight(GoSurfaceStud tool, k64f value);

/** 
 * Returns the stud base height value.
 *
 * @public                  @memberof GoSurfaceStud
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceStud object.
 * @return                  The stud base height value.
 */
GoFx(k64f) GoSurfaceStud_BaseHeight(GoSurfaceStud tool);

/** 
 * Sets the base height value.
 *
 * @public                  @memberof GoSurfaceStud
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceStud object.
 * @param    value          The base height value to set.
 * @return                  Operation status.
 */
GoFx(kStatus) GoSurfaceStud_SetBaseHeight(GoSurfaceStud tool, k64f value);

/** 
 * Returns the stud tip height value.
 *
 * @public                  @memberof GoSurfaceStud
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceStud object.
 * @return                  The stud tip height value.
 */
GoFx(k64f) GoSurfaceStud_TipHeight(GoSurfaceStud tool);

/** 
 * Sets the tip height value.
 *
 * @public                  @memberof GoSurfaceStud
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceStud object.
 * @param    value          The tip height value to set.
 * @return                  Operation status.
 */
GoFx(kStatus) GoSurfaceStud_SetTipHeight(GoSurfaceStud tool, k64f value);

/** 
 * Gets the enabled state of the tool region.
 *
 * @public                  @memberof GoSurfaceStud
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceStud object.
 * @return                  kTRUE if enabled and kFALSE if disabled.
 */
GoFx(kBool) GoSurfaceStud_RegionEnabled(GoSurfaceStud tool);

/** 
 * Sets the enabled state of the tool region.
 *
 * @public                  @memberof GoSurfaceStud
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceStud object.
 * @param    enable         kTRUE to enable the region and kFALSE to disable it.
 * @return                  Operation status.
 */
GoFx(kStatus) GoSurfaceStud_EnableRegion(GoSurfaceStud tool, kBool enable);

/** 
 * Returns the tool region object.
 *
 * @public                  @memberof GoSurfaceStud
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceStud object.
 * @return                  A GoRegion3d object.
 */
GoFx(GoRegion3d) GoSurfaceStud_Region(GoSurfaceStud tool);

/** 
 * Gets the enabled state of the reference regions.
 *
 * @public                  @memberof GoSurfaceStud
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceStud object.
 * @return   enable         kTRUE if the reference regions are enabled and kFALSE otherwise.
 */
GoFx(kBool) GoSurfaceStud_RefRegionsEnabled(GoSurfaceStud tool);

/** 
 * Sets the enabled state of the reference regions.
 *
 * @public                  @memberof GoSurfaceStud
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceStud object.
 * @param    enable         kTRUE to enable the region and kFALSE to disable it.
 * @return                  Operation status.
 */
GoFx(kStatus) GoSurfaceStud_EnableRefRegions(GoSurfaceStud tool, kBool enable);

/** 
 * Sets the reference region count.
 *
 * @public                  @memberof GoSurfaceStud
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceStud object.
 * @param    count          The number of reference regions to use.
 * @return                  Operation status.
 * @see                     GO_SURFACE_STUD_MAX_REF_REGIONS
 */
GoFx(kStatus) GoSurfaceStud_SetRefRegionCount(GoSurfaceStud tool, kSize count);

/** 
 * Returns the reference region count.
 *
 * @public                  @memberof GoSurfaceStud
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceStud object.
 * @return                  The reference region count.
 */
GoFx(kSize) GoSurfaceStud_RefRegionCount(GoSurfaceStud tool);

/** 
 * Returns the reference region object at the given index.
 *
 * @public                  @memberof GoSurfaceStud
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceStud object.
 * @param    index          The index with which to return a reference region.
 * @return                  A GoSurfaceRegion2d object.
 * @see                     GoSurfaceStud_RefRegionCount
 */
GoFx(GoSurfaceRegion2d) GoSurfaceStud_RefRegionAt(GoSurfaceStud tool, kSize index);

/** 
 * Gets the enabled state of auto-tilt.
 *
 * @public                  @memberof GoSurfaceStud
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceStud object.
 * @return                  kTRUE if enabled and kFALSE if disabled.
 */
GoFx(kBool) GoSurfaceStud_AutoTiltEnabled(GoSurfaceStud tool);

/** 
 * Sets the enabled state of auto-tilt.
 *
 * @public                  @memberof GoSurfaceStud
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceStud object.
 * @param    enable         kTRUE to enable the region and kFALSE to disable it.
 * @return                  Operation status.
 */
GoFx(kStatus) GoSurfaceStud_EnableAutoTilt(GoSurfaceStud tool, kBool enable);

/** 
 * Gets the tilt X-angle value.
 *
 * @public                  @memberof GoSurfaceStud
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceStud object.
 * @return                  The tilt X-angle value.
 */
GoFx(k64f) GoSurfaceStud_TiltXAngle(GoSurfaceStud tool);

/** 
 * Sets the tilt X-angle value.
 *
 * @public                  @memberof GoSurfaceStud
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceStud object.
 * @param    value          The tilt X-angle value to set.
 * @return                  Operation status.
 */
GoFx(kStatus) GoSurfaceStud_SetTiltXAngle(GoSurfaceStud tool, k64f value);

/** 
 * Returns the tilt Y-angle value.
 *
 * @public                  @memberof GoSurfaceStud
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceStud object.
 * @return                  The tilt Y-angle value.
 */
GoFx(k64f) GoSurfaceStud_TiltYAngle(GoSurfaceStud tool);

/** 
 * Sets the tilt Y-angle value.
 *
 * @public                  @memberof GoSurfaceStud
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceStud object.
 * @param    value          The tilt Y-angle value to set.
 * @return                  Operation status.
 */
GoFx(kStatus) GoSurfaceStud_SetTiltYAngle(GoSurfaceStud tool, k64f value);

/**
 * Returns a GoSurfaceStud Base X measurement object.
 *
 * @public           @memberof GoSurfaceStud
 * @note             Supported with G2, G3
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoSurfaceStud object.
 * @return           A GoSurfaceStudBaseX Base X measurement.
 */
GoFx(GoSurfaceStudBaseX) GoSurfaceStud_BaseXMeasurement(GoSurfaceStud tool);

/**
 * Returns a GoSurfaceStud Base Y measurement object.
 *
 * @public           @memberof GoSurfaceStud
 * @note             Supported with G2, G3
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoSurfaceStud object.
 * @return           A GoSurfaceStudBaseY Base Y measurement.
 */
GoFx(GoSurfaceStudBaseY) GoSurfaceStud_BaseYMeasurement(GoSurfaceStud tool);

/**
 * Returns a GoSurfaceStud Base Z measurement object.
 *
 * @public           @memberof GoSurfaceStud
 * @note             Supported with G2, G3
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoSurfaceStud object.
 * @return           A GoSurfaceStud Base Z measurement.
 */
GoFx(GoSurfaceStudBaseZ) GoSurfaceStud_BaseZMeasurement(GoSurfaceStud tool);

/**
 * Returns a GoSurfaceStud Tip X measurement object.
 *
 * @public           @memberof GoSurfaceStud
 * @note             Supported with G2, G3
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoSurfaceStud object.
 * @return           A GoSurfaceStudTipX Tip X measurement.
 */
GoFx(GoSurfaceStudTipX) GoSurfaceStud_TipXMeasurement(GoSurfaceStud tool);

/**
 * Returns a GoSurfaceStud Tip Y measurement object.
 *
 * @public           @memberof GoSurfaceStud
 * @note             Supported with G2, G3
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoSurfaceStud object.
 * @return           A GoSurfaceStudTipY Tip Y measurement.
 */
GoFx(GoSurfaceStudTipY) GoSurfaceStud_TipYMeasurement(GoSurfaceStud tool);

/**
 * Returns a GoSurfaceStud Tip Z measurement object.
 *
 * @public           @memberof GoSurfaceStud
 * @note             Supported with G2, G3
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoSurfaceStud object.
 * @return           A GoSurfaceStudTipZ Tip Z measurement.
 */
GoFx(GoSurfaceStudTipZ) GoSurfaceStud_TipZMeasurement(GoSurfaceStud tool);

/**
 * Returns a GoSurfaceStud Radius measurement object.
 *
 * @public           @memberof GoSurfaceStud
 * @note             Supported with G2, G3
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoSurfaceStud object.
 * @return           A GoSurfaceStudRadius Radius measurement.
 */
GoFx(GoSurfaceStudRadius) GoSurfaceStud_RadiusMeasurement(GoSurfaceStud tool);

/**
* Returns a GoSurfaceStud tip point feature object.
*
* @public           @memberof GoSurfaceStud
* @note             Supported with G2, G3
* @version          Introduced in firmware 4.0.10.27
* @param    tool    GoSurfaceStud object.
* @return           A GoSurfaceStudTipPoint tip point feature .
*/
GoFx(GoSurfaceStudTipPoint) GoSurfaceStud_TipPoint(GoSurfaceHole tool);

/**
* Returns a GoSurfaceStud base point feature object.
*
* @public           @memberof GoSurfaceStud
* @note             Supported with G2, G3
* @version          Introduced in firmware 4.0.10.27
* @param    tool    GoSurfaceStud object.
* @return           A GoSurfaceStudBasePoint base point feature.
*/
GoFx(GoSurfaceStudBasePoint) GoSurfaceStud_BasePoint(GoSurfaceHole tool);

/**
 * @class   GoSurfaceVolume
 * @extends GoSurfaceTool
 * @note    Supported with G2, G3
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents a surface volume tool. measurements in mm, mm^2 for area and mm^3 for volume
 */
typedef GoSurfaceTool GoSurfaceVolume; 

/** 
 * Gets the enabled state of the tool region.
 *
 * @public                  @memberof GoSurfaceVolume
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceVolume object.
 * @return                  kTRUE if enabled and kFALSE if disabled.
 */
GoFx(kBool) GoSurfaceVolume_RegionEnabled(GoSurfaceVolume tool);

/** 
 * Sets the enabled state of the tool region.
 *
 * @public                  @memberof GoSurfaceVolume
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceVolume object.
 * @param    enable         kTRUE to enable the tool region and kFALSE to disable it.
 */
GoFx(kStatus) GoSurfaceVolume_EnableRegion(GoSurfaceVolume tool, kBool enable);

/** 
 * Returns the tool region object.
 *
 * @public                  @memberof GoSurfaceVolume
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoSurfaceVolume object.
 * @return                  A GoRegion3d object.
 */
GoFx(GoRegion3d) GoSurfaceVolume_Region(GoSurfaceVolume tool);

/**
 * Returns a GoSurfaceVolume Volume measurement object.
 *
 * @public           @memberof GoSurfaceVolume
 * @note             Supported with G2, G3
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoSurfaceVolume object.
 * @return           A GoSurfaceVolumeVolume Volume measurement.
 */
GoFx(GoSurfaceVolumeVolume) GoSurfaceVolume_VolumeMeasurement(GoSurfaceVolume tool);

/**
 * Returns a GoSurfaceVolume Area measurement object.
 *
 * @public           @memberof GoSurfaceVolume
 * @note             Supported with G2, G3
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoSurfaceVolume object.
 * @return           A GoSurfaceVolumeArea Area measurement.
 */
GoFx(GoSurfaceVolumeArea) GoSurfaceVolume_AreaMeasurement(GoSurfaceVolume tool);

/**
 * Returns a GoSurfaceVolume Thickness measurement object.
 *
 * @public           @memberof GoSurfaceVolume
 * @note             Supported with G2, G3
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoSurfaceVolume object.
 * @return           A GoSurfaceVolumeThickness Thickness measurement.
 */
GoFx(GoSurfaceVolumeThickness) GoSurfaceVolume_ThicknessMeasurement(GoSurfaceVolume tool);

#include <GoSdk/Tools/GoSurfaceTools.x.h>

#endif

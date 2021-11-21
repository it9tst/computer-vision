/** 
 * @file    GoProfileTools.h
 * @brief   Declares all profile tools and their related classes. 
 *
 * @internal
 * Copyright (C) 2016-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef GO_PROFILE_TOOLS_H
#define GO_PROFILE_TOOLS_H

#include <GoSdk/GoSdkDef.h>
#include <GoSdk/Tools/GoTool.h>
#include <GoSdk/Tools/GoProfileToolUtils.h>
#include <GoSdk/GoUtils.h>

/**
 * @class   GoProfileTool
 * @extends GoTool
 * @ingroup GoSdk-ProfileTools
 * @brief   Represents a base profile tool.
 */
typedef GoTool GoProfileTool; 


/** 
 * Sets the data stream. Note that stream validation will only occur if tool 
 * is in the tool options list.
 *
 * @public              @memberof GoProfileTool
 * @version             Introduced in firmware 4.4.4.14
 * @param    tool       GoProfileTool object.
 * @param    stream     GoDataStream value.
 * @return              Operation status.            
 * @see                 GoProfileTool_StreamOptionCount, GoProfileTool_StreamOptionAt
 */
GoFx(kStatus) GoProfileTool_SetStream(GoProfileTool tool, GoDataStream stream);

/** 
 * Gets the data stream.
 *
 * @public              @memberof GoProfileTool
 * @version             Introduced in firmware 4.4.4.14
 * @param    tool       GoProfileTool object.
 * @return              The current profile tool data stream value.            
 */
GoFx(GoDataStream) GoProfileTool_Stream(GoProfileTool tool);

/** 
 * Gets the data stream option list count.
 *
 * @public              @memberof GoProfileTool
 * @version             Introduced in firmware 4.4.4.14
 * @param    tool       GoProfileTool object.
 * @return              The current profile tool data stream option list count.            
 */
GoFx(kSize) GoProfileTool_StreamOptionCount(GoProfileTool tool);

/** 
 * Gets the data stream option at the given index.
 *
 * @public              @memberof GoProfileTool
 * @version             Introduced in firmware 4.4.4.14
 * @param    tool       GoProfileTool object.
 * @param    index      The index of the option list to access.
 * @return              The profile tool data stream option at the given index, or k32U_MAX if an invalid index is given.            
 */
GoFx(GoDataStream) GoProfileTool_StreamOptionAt(GoProfileTool tool, kSize index);

/** 
 * Sets the data source. Note that source validation will only occur if tool 
 * is in the tool options list.
 *
 * @public              @memberof GoProfileTool
 * @version             Introduced in firmware 4.0.10.27
 * @param    tool       GoProfileTool object.
 * @param    source     GoDataSource object.
 * @return              Operation status.            
 * @see                 GoTools_ToolOptionCount, GoTools_ToolOptionAt
 */
GoFx(kStatus) GoProfileTool_SetSource(GoProfileTool tool, GoDataSource source);

/** 
 * Gets the data source.
 *
 * @public              @memberof GoProfileTool
 * @version             Introduced in firmware 4.0.10.27
 * @param    tool       GoProfileTool object.
 * @return              The current profile tool data source.            
 */
GoFx(GoDataSource) GoProfileTool_Source(GoProfileTool tool);

/** 
 * Gets the data source option list count.
 *
 * @public              @memberof GoProfileTool
 * @version             Introduced in firmware 4.0.10.27
 * @param    tool       GoProfileTool object.
 * @return              The current profile tool data source option list count.            
 */
GoFx(kSize) GoProfileTool_SourceOptionCount(GoProfileTool tool);

/** 
 * Gets the data source option at the given index.
 *
 * @public              @memberof GoProfileTool
 * @version             Introduced in firmware 4.0.10.27
 * @param    tool       GoProfileTool object.
 * @param    index      The index of the option list to access.
 * @return              The profile tool data source option at the given index, or k32U_MAX if an invalid index is given.            
 */
GoFx(GoDataSource) GoProfileTool_SourceOptionAt(GoProfileTool tool, kSize index);

/** 
 * Gets the X-anchoring option list count.
 *
 * @public              @memberof GoProfileTool
 * @version             Introduced in firmware 4.0.10.27
 * @param    tool       GoProfileTool object.
 * @return              The X-anchoring option list count.
 */
GoFx(kSize) GoProfileTool_XAnchorOptionCount(GoProfileTool tool);

/** 
 * Gets the X-anchoring option at the given index.
 *
 * @public              @memberof GoProfileTool
 * @version             Introduced in firmware 4.0.10.27
 * @param    tool       GoProfileTool object.
 * @param    index      The index of the option list to access.
 * @return              The X-anchoring option at the given index or k32U_MAX if invalid.
 */
GoFx(k32u) GoProfileTool_XAnchorOptionAt(GoProfileTool tool, kSize index);

/** 
 * Gets the current X-anchoring source.
 *
 * @public              @memberof GoProfileTool
 * @version             Introduced in firmware 4.0.10.27
 * @param    tool       GoProfileTool object.
 * @return              The X-anchoring source or -1 if no source is currently set.
 */
GoFx(k32s) GoProfileTool_XAnchor(GoProfileTool tool);

/** 
 * Sets the X-anchoring source.
 *
 * @public              @memberof GoProfileTool
 * @version             Introduced in firmware 4.0.10.27
 * @param    tool       GoProfileTool object.
 * @param    id         The measurement ID of a valid X-anchoring source.
 * @return              Operation status.
 */
GoFx(kStatus) GoProfileTool_SetXAnchor(GoProfileTool tool, k32s id);

/** 
 * Returns a boolean value representing whether or not a valid X-anchoring source has been set for X-anchoring.
 *
 * @public              @memberof GoProfileTool
 * @version             Introduced in firmware 4.0.10.27
 * @param    tool       GoProfileTool object.
 * @return              kTRUE if a valid anchoring source is currently set and kFALSE otherwise.
 */
GoFx(kBool) GoProfileTool_XAnchorEnabled(GoProfileTool tool);

/** 
 * Gets the Z-anchoring option list count.
 *
 * @public              @memberof GoProfileTool
 * @version             Introduced in firmware 4.0.10.27
 * @param    tool       GoProfileTool object.
 * @return              The X-anchoring option list count.
 */
GoFx(kSize) GoProfileTool_ZAnchorOptionCount(GoProfileTool tool);

/** 
 * Gets the Z-anchoring option at the given index.
 *
 * @public              @memberof GoProfileTool
 * @version             Introduced in firmware 4.0.10.27
 * @param    tool       GoProfileTool object.
 * @param    index      The index of the option list to access.
 * @return              The Z-anchoring option at the given index or k32U_MAX if invalid.
 */
GoFx(k32u) GoProfileTool_ZAnchorOptionAt(GoProfileTool tool, kSize index);

/** 
 * Gets the current Z-anchoring source.
 *
 * @public              @memberof GoProfileTool
 * @version             Introduced in firmware 4.0.10.27
 * @param    tool       GoProfileTool object.
 * @return              The Z-anchoring source or -1 if no source is currently set.
 */
GoFx(k32s) GoProfileTool_ZAnchor(GoProfileTool tool);

/** 
 * Sets the Z-anchoring source.
 *
 * @public              @memberof GoProfileTool
 * @version             Introduced in firmware 4.0.10.27
 * @param    tool       GoProfileTool object.
 * @param    id         The measurement ID of a valid Z-anchoring source.
 * @return              Operation status.
 */
GoFx(kStatus) GoProfileTool_SetZAnchor(GoProfileTool tool, k32s id);

/** 
 * Returns a boolean value representing whether or not a valid Z-anchoring source has been set for Z-anchoring.
 *
 * @public              @memberof GoProfileTool
 * @version             Introduced in firmware 4.0.10.27
 * @param    tool       GoProfileTool object.
 * @return              kTRUE if a valid anchoring source is currently set and kFALSE otherwise.
 */
GoFx(kBool) GoProfileTool_ZAnchorEnabled(GoProfileTool tool);

/**
 * @class   GoProfileArea
 * @extends GoProfileTool
 * @ingroup GoSdk-ProfileTools
 * @brief   Represents a profile area tool.
 */
typedef GoProfileTool GoProfileArea; 

/** 
 * Gets the profile area baseline.
 *
 * @public               @memberof GoProfileArea
 * @version              Introduced in firmware 4.0.10.27
 * @param    tool        GoProfileArea object.
 * @return               The profile area baseline.            
 */
GoFx(GoProfileBaseline) GoProfileArea_Baseline(GoProfileArea tool);

/** 
 * Sets the profile area type.
 *
 * @public               @memberof GoProfileArea
 * @version              Introduced in firmware 4.0.10.27
 * @param    tool        GoProfileArea object.
 * @param    type        The baseline type to set.
 * @return               Operation status.
 */
GoFx(kStatus) GoProfileArea_SetBaseline(GoProfileArea tool, GoProfileBaseline type);

/** 
 * Returns a boolean representing whether the profile area baseline is used.
 *
 * @public               @memberof GoProfileArea
 * @version              Introduced in firmware 4.0.10.27
 * @param    tool        GoProfileArea object.
 * @return               kTRUE if the baseline is used; kFALSE otherwise.
 */
GoFx(kBool) GoProfileArea_BaselineUsed(GoProfileArea tool);

/** 
 * Gets the reference profile line.
 *
 * @public               @memberof GoProfileArea
 * @version              Introduced in firmware 4.0.10.27
 * @param    tool        GoProfileArea object.
 * @return               The reference profile line.            
 */
GoFx(GoProfileLineRegion) GoProfileArea_LineRegion(GoProfileArea tool);

/** 
 * Gets the profile area type.
 *
 * @public               @memberof GoProfileArea
 * @version              Introduced in firmware 4.0.10.27
 * @param    tool        GoProfileArea object.
 * @return               The profile area type.            
 */
GoFx(GoProfileAreaType) GoProfileArea_Type(GoProfileArea tool);

/** 
 * Gets the boolean representing whether the area type is used.
 *
 * @public               @memberof GoProfileArea
 * @version              Introduced in firmware 4.0.10.27
 * @param    tool        GoProfileArea object.
 * @return               kTRUE if profile area type is used; kFALSE otherwise.
 */
GoFx(kBool) GoProfileArea_TypeUsed(GoProfileArea tool);

/** 
 * Sets the profile area type.
 *
 * @public               @memberof GoProfileArea
 * @version              Introduced in firmware 4.0.10.27
 * @param    tool        GoProfileArea object.
 * @param    type        GoProfileAreaType object.
 * @return               Operation status.            
 */
GoFx(kStatus) GoProfileArea_SetType(GoProfileArea tool, GoProfileAreaType type);

/** 
 * Gets the profile region.
 *
 * @public               @memberof GoProfileArea
 * @version              Introduced in firmware 4.0.10.27
 * @param    tool        GoProfileArea object.
 * @return               The profile region.            
 */
GoFx(GoProfileRegion) GoProfileArea_Region(GoProfileArea tool);

/** 
 * Indicates whether the region is enabled.
 *
 * @public               @memberof GoProfileArea
 * @version              Introduced in firmware 4.4.4.14
 * @param    tool        GoProfileArea object.
 * @return               kTRUE if enabled and kFALSE otherwise.
 */
GoFx(kBool) GoProfileArea_RegionEnabled(GoProfileArea tool);

/** 
 * Enables or disables the region.
 *
 * @public               @memberof GoProfileArea
 * @version              Introduced in firmware 4.4.4.14
 * @param    tool        GoProfileArea object.
 * @param    enable      kTRUE to enable the region and kFALSE to disable it.
 * @return               Operation status.            
 */
GoFx(kStatus) GoProfileArea_EnableRegion(GoProfileArea tool, kBool enable);

/**
 * Returns a GoProfileArea Area measurement object.
 *
 * @public           @memberof GoProfileArea
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoProfileArea object.
 * @return           A GoProfileAreaArea measurement.
 */
GoFx(GoProfileAreaArea) GoProfileArea_AreaMeasurement(GoProfileArea tool);

/**
 * Returns a GoProfileArea Centroid X measurement object.
 *
 * @public           @memberof GoProfileArea
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoProfileArea object.
 * @return           A GoProfileAreaCentroidX measurement.
 */
GoFx(GoProfileAreaCentroidX) GoProfileArea_CentroidXMeasurement(GoProfileArea tool);

/**
 * Returns a GoProfileArea Centroid Z measurement object.
 *
 * @public           @memberof GoProfileArea
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoProfileArea object.
 * @return           A GoProfileAreaCentroidZ measurement.
 */
GoFx(GoProfileAreaCentroidZ) GoProfileArea_CentroidZMeasurement(GoProfileArea tool);

/**
* Returns a GoProfileArea Center point feature object.
*
* @public           @memberof GoProfileArea
* @version          Introduced in firmware 4.6.4.9
* @param    tool    GoProfileArea object.
* @return           A GoProfileAreaCenterPoint Center point feature.
*/
GoFx(GoProfileAreaCenterPoint) GoProfileArea_CenterPoint(GoProfileArea tool);

/**
 * @class   GoProfileBox
 * @extends GoProfileTool
 * @ingroup GoSdk-ProfileTools
 * @brief   Represents a profile bounding box tool.
 */
typedef GoProfileTool GoProfileBox; 

/** 
 * Returns the enabled state of the tool region.
 *
 * @public                  @memberof GoProfileBox
 * @version                 Introduced in firmware 4.2.4.7
 * @param    tool           GoProfileBox object.
 * @return                  kTRUE if enabled and kFALSE if disabled.
 */
GoFx(kBool) GoProfileBox_RegionEnabled(GoProfileBox tool);

/** 
 * Enables or disables the tool region.
 *
 * @public                  @memberof GoProfileBox
 * @version                 Introduced in firmware 4.2.4.7
 * @param    tool           GoProfileBox object.
 * @param    enable         kTRUE to enable the tool region, kFALSE to disable it.
 * @return                  Operation status.
 */
GoFx(kStatus) GoProfileBox_EnableRegion(GoProfileBox tool, kBool enable);

/** 
 * Gets the profile bounding box region.
 *
 * @public                  @memberof GoProfileBox
 * @version                 Introduced in firmware 4.2.4.7
 * @param    tool           GoProfileBox object.
 * @return                  A GoRegion3d object.
 */
GoFx(GoProfileRegion) GoProfileBox_Region(GoProfileBox tool);

/**
 * Returns a GoProfileBox X measurement object.
 *
 * @public              @memberof GoProfileBox
 * @version             Introduced in firmware 4.2.4.7
 * @param    tool       GoProfileBox object.
 * @return              A GoProfileBox X measurement. (mm)
 */
GoFx(GoProfileBoxX) GoProfileBox_XMeasurement(GoProfileBox tool);

/**
 * Returns a GoProfileBox Z measurement object.
 *
 * @public              @memberof GoProfileBox
 * @version             Introduced in firmware 4.2.4.7
 * @param    tool       GoProfileBox object.
 * @return              A GoProfileBox Z measurement. (mm)
 */
GoFx(GoProfileBoxZ) GoProfileBox_ZMeasurement(GoProfileBox tool);

/**
 * Returns a GoProfileBox Width measurement object.
 *
 * @public              @memberof GoProfileBox
 * @version             Introduced in firmware 4.2.4.7
 * @param    tool       GoProfileBox object.
 * @return              A GoProfileBox Width measurement. (mm)
 */
GoFx(GoProfileBoxWidth) GoProfileBox_WidthMeasurement(GoProfileBox tool);

/**
 * Returns a GoProfileBox Height measurement object.
 *
 * @public              @memberof GoProfileBox
 * @version             Introduced in firmware 4.2.4.7
 * @param    tool       GoProfileBox object.
 * @return              A GoProfileBox Height measurement. (mm)
 */
GoFx(GoProfileBoxHeight) GoProfileBox_HeightMeasurement(GoProfileBox tool);

/**
 * Returns a GoProfileBox global X measurement object.
 *
 * @public              @memberof GoProfileBox
 * @version             Introduced in firmware 4.2.4.7
 * @param    tool       GoProfileBox object.
 * @return              A GoProfileBox global X measurement. (mm)
 */
GoFx(GoProfileBoxGlobalX) GoProfileBox_GlobalXMeasurement(GoProfileBox tool);

/**
 * Returns a GoProfileBox global Y measurement object.
 *
 * @public              @memberof GoProfileBox
 * @version             Introduced in firmware 4.4.4.14
 * @param    tool       GoProfileBox object.
 * @return              A GoProfileBox global Y measurement. (mm)
 */
GoFx(GoProfileBoxGlobalY) GoProfileBox_GlobalYMeasurement(GoProfileBox tool);

/**
 * Returns a GoProfileBox global Angle measurement object.
 *
 * @public              @memberof GoProfileBox
 * @version             Introduced in firmware 4.4.4.14
 * @param    tool       GoProfileBox object.
 * @return              A GoProfileBox global angle measurement. (degrees)
 */
GoFx(GoProfileBoxGlobalAngle) GoProfileBox_GlobalAngleMeasurement(GoProfileBox tool);

/**
* Returns a GoProfileBox corner point feature object.
*
* @public              @memberof GoProfileBox
* @version             Introduced in firmware 4.4.4.14
* @param    tool       GoProfileBox object.
* @return              A GoProfileBoundingBoxCornerPoint global angle measurement. (degrees)
*/
GoFx(GoProfileBoundingBoxCornerPoint) GoProfileBox_CornerPoint(GoProfileBox tool);

/**
* Returns a GoProfileBox center point feature object.
*
* @public              @memberof GoProfileBox
* @version             Introduced in firmware 4.4.4.14
* @param    tool       GoProfileBox object.
* @return              A GoProfileBoundingBoxCenterPoint center point feature.
*/
GoFx(GoProfileBoundingBoxCenterPoint) GoProfileBox_CenterPoint(GoProfileBox tool);

/**
 * @class   GoProfileBridgeValue
 * @extends GoProfileTool
 * @ingroup GoSdk-ProfileTools
 * @brief   Represents a profile bridge value tool.
*/
typedef GoProfileTool GoProfileBridgeValue;

/** 
 * Returns the enabled state of the tool region.
 *
 * @public                  @memberof GoProfileBridgeValue
 * @version                 Introduced in firmware 4.3.3.124
 * @param    tool           GoProfileBridgeValue object.
 * @return                  kTRUE if enabled and kFALSE if disabled.
 */
GoFx(kBool) GoProfileBridgeValue_RegionEnabled(GoProfileBridgeValue tool);

/** 
 * Enables or disables the tool region.
 *
 * @public                  @memberof GoProfileBridgeValue
 * @version                 Introduced in firmware 4.3.3.124
 * @param    tool           GoProfileBridgeValue object.
 * @param    enable         kTRUE to enable the tool region, kFALSE to disable it.
 * @return                  Operation status.
 */
GoFx(kStatus) GoProfileBridgeValue_EnableRegion(GoProfileBridgeValue tool, kBool enable);

/** 
 * Gets the bridge value profile region.
 *
 * @public               @memberof GoProfileBridgeValue
 * @version              Introduced in firmware 4.3.3.124
 * @param    tool        GoProfileArea object.
 * @return               The profile region.            
 */
GoFx(GoProfileRegion) GoProfileBridgeValue_Region(GoProfileBridgeValue tool);


/**
 * Returns the enabled state of normalization.
 *
 * @public                  @memberof GoProfileBridgeValue
 * @version                 Introduced in firmware 4.3.3.124
 * @param    tool           GoProfileBridgeValue object.
 * @return                  kTRUE if enabled and kFALSE if disabled.
 */
GoFx(kBool) GoProfileBridgeValue_NormalizeEnabled(GoProfileBridgeValue tool);

/**
 * Enables or disables normalization.
 *
 * @public                  @memberof GoProfileBridgeValue
 * @version                 Introduced in firmware 4.3.3.124
 * @param    tool           GoProfileBridgeValue object.
 * @param    enable         kTRUE to enable the tool region, kFALSE to disable it.
 * @return                  Operation status.
 */
GoFx(kStatus) GoProfileBridgeValue_EnableNormalize(GoProfileBridgeValue tool, kBool enable);

/**
 * Sets the profile X-Line tool window size percentage.
 *
 * @public                  @memberof GoProfileBridgeValue
 * @version                 Introduced in firmware 4.3.3.124
 * @param    tool           GoProfileBridgeValue object.
 * @param    value          The value to set.
 * @return                  Operation status.
 */
GoFx(kStatus) GoProfileBridgeValue_SetWindowSize(GoProfileBridgeValue tool, k64f value);

/**
 * Gets the profile X-Line tool window size percentage.
 *
 * @public                  @memberof GoProfileBridgeValue
 * @version                 Introduced in firmware 4.3.3.124
 * @param    tool           GoProfileBridgeValue object.
 * @return                  The window size.
 */
GoFx(k64f) GoProfileBridgeValue_WindowSize(GoProfileBridgeValue tool);

/**
 * Sets the profile X-Line tool window skip percentage.
 *
 * @public                  @memberof GoProfileBridgeValue
 * @version                 Introduced in firmware 4.3.3.124
 * @param    tool           GoProfileBridgeValue object.
 * @param    value          The value to set.
 * @return                  Operation status.
 */
GoFx(kStatus) GoProfileBridgeValue_SetWindowSkip(GoProfileBridgeValue tool, k64f value);

/**
 * Gets the profile X-Line tool window skip percentage.
 *
 * @public                  @memberof GoProfileBridgeValue
 * @version                 Introduced in firmware 4.3.3.124
 * @param    tool           GoProfileBridgeValue object.
 * @return                  The window size.
 */
GoFx(k64f) GoProfileBridgeValue_WindowSkip(GoProfileBridgeValue tool);

/**
 * Sets the profile X-Line tool max invalid percentage.
 *
 * @public                  @memberof GoProfileBridgeValue
 * @version                 Introduced in firmware 4.3.3.124
 * @param    tool           GoProfileBridgeValue object.
 * @param    value          The value to set.
 * @return                  Operation status.
 */
GoFx(kStatus) GoProfileBridgeValue_SetMaxInvalid(GoProfileBridgeValue tool, k64f value);

/**
 * Gets the profile X-Line tool max invalid percentage.
 *
 * @public                  @memberof GoProfileBridgeValue
 * @version                 Introduced in firmware 4.3.3.124
 * @param    tool           GoProfileBridgeValue object.
 * @return                  The window size.
 */
GoFx(k64f) GoProfileBridgeValue_MaxInvalid(GoProfileBridgeValue tool);

/**
 * Sets the profile X-Line tool max differential.
 *
 * @public                  @memberof GoProfileBridgeValue
 * @version                 Introduced in firmware 4.3.3.124
 * @param    tool           GoProfileBridgeValue object.
 * @param    value          The value to set.
 * @return                  Operation status.
 */
GoFx(kStatus) GoProfileBridgeValue_SetMaxDifferential(GoProfileBridgeValue tool, k64f value);

/**
 * Gets the profile X-Line tool max differential.
 *
 * @public                  @memberof GoProfileBridgeValue
 * @version                 Introduced in firmware 4.3.3.124
 * @param    tool           GoProfileBridgeValue object.
 * @return                  The max differential.
 */
GoFx(k64f) GoProfileBridgeValue_MaxDifferential(GoProfileBridgeValue tool);

/**
 * Gets the profile X-Line tool max differential maximum value limit.
 *
 * @public                  @memberof GoProfileBridgeValue
 * @version                 Introduced in firmware 4.3.3.124
 * @param    tool           GoProfileBridgeValue object.
 * @return                  The max differential maximum value limit.
 */
GoFx(k64f) GoProfileBridgeValue_MaxDifferentialLimitMax(GoProfileBridgeValue tool);

/**
 * Gets the profile X-Line tool max differential minimum value limit.
 * NOTE: A value of 0 will result in an automated differential determination.
 *
 * @public                  @memberof GoProfileBridgeValue
 * @version                 Introduced in firmware 4.3.3.124
 * @param    tool           GoProfileBridgeValue object.
 * @return                  The max differential minimum value limit.
 */
GoFx(k64f) GoProfileBridgeValue_MaxDifferentialLimitMin(GoProfileBridgeValue tool);

/**
 * Returns a GoProfileBridgeValue bridge value measurement object.
 *
 * @public              @memberof GoProfileBridgeValue
 * @version             Introduced in firmware 4.3.3.124
 * @param    tool       GoProfileBridgeValue object.
 * @return              A GoProfileBridgeValue bridge value measurement.
 */
GoFx(GoProfileBridgeValueBridgeValue) GoProfileBridgeValue_BridgeValueMeasurement(GoProfileBridgeValue tool);

/**
 * Returns a GoProfileBridgeValue angle measurement object.
 *
 * @public              @memberof GoProfileBridgeValue
 * @version             Introduced in firmware 4.3.3.124
 * @param    tool       GoProfileBridgeValue object.
 * @return              A GoProfileBridgeValue angle measurement. (degrees)
 */
GoFx(GoProfileBridgeValueAngle) GoProfileBridgeValue_AngleMeasurement(GoProfileBridgeValue tool);

/**
* Returns a GoProfileBridgeValue window measurement object.
*
* @public              @memberof GoProfileBridgeValue
* @version             Introduced in firmware 4.3.3.124
* @param    tool       GoProfileBridgeValue object.
* @return              A GoProfileBridgeValue window measurement.
*/
GoFx(GoProfileBridgeValueWindow) GoProfileBridgeValue_WindowMeasurement(GoProfileBridgeValue tool);

/**
* Returns a GoProfileBridgeValue standard deviation measurement object.
*
* @public              @memberof GoProfileBridgeValue
* @version             Introduced in firmware 4.3.3.124
* @param    tool       GoProfileBridgeValue object.
* @return              A GoProfileBridgeValue standard deviation measurement.
*/
GoFx(GoProfileBridgeValueStdDev) GoProfileBridgeValue_StdDevMeasurement(GoProfileBridgeValue tool);


/**
 * @class   GoProfileCircle
 * @extends GoProfileTool
 * @ingroup GoSdk-ProfileTools
 * @brief   Represents a profile circle tool.
 */
typedef GoProfileTool GoProfileCircle; 

/** 
 * Gets the profile region.
 *
 * @public               @memberof GoProfileCircle
 * @version              Introduced in firmware 4.0.10.27
 * @param    tool        GoProfileCircle object.
 * @return               The profile region.            
 */
GoFx(GoProfileRegion) GoProfileCircle_Region(GoProfileCircle tool);

/** 
 * Indicates whether the region is enabled.
 *
 * @public               @memberof GoProfileCircle
 * @version              Introduced in firmware 4.4.4.14
 * @param    tool        GoProfileCircle object.
 * @return               kTRUE if enabled and kFALSE otherwise.
 */
GoFx(kBool) GoProfileCircle_RegionEnabled(GoProfileCircle tool);

/** 
 * Enables or disables the region.
 *
 * @public               @memberof GoProfileCircle
 * @version              Introduced in firmware 4.4.4.14
 * @param    tool        GoProfileCircle object.
 * @param    enable      kTRUE to enable the region and kFALSE to disable it.
 * @return               Operation status.            
 */
GoFx(kStatus) GoProfileCircle_EnableRegion(GoProfileCircle tool, kBool enable);

/**
 * Returns a GoProfileCircle X measurement object.
 *
 * @public               @memberof GoProfileCircle
 * @version              Introduced in firmware 4.0.10.27
 * @param    tool        GoProfileCircle object.
 * @return               A GoProfileCircleX measurement. (mm)
 */
GoFx(GoProfileCircleX) GoProfileCircle_XMeasurement(GoProfileCircle tool);

/**
 * Returns a GoProfileCircle Z measurement object.
 *
 * @public               @memberof GoProfileCircle
 * @version              Introduced in firmware 4.0.10.27
 * @param    tool        GoProfileCircle object.
 * @return               A GoProfileCircleZ measurement. (mm)
 */
GoFx(GoProfileCircleZ) GoProfileCircle_ZMeasurement(GoProfileCircle tool);

/**
 * Returns a GoProfileCircle Radius measurement object.
 *
 * @public               @memberof GoProfileCircle
 * @version              Introduced in firmware 4.0.10.27
 * @param    tool        GoProfileCircle object.
 * @return               A GoProfileCircleRadius Radius measurement.(mm)
 */
GoFx(GoProfileCircleRadius) GoProfileCircle_RadiusMeasurement(GoProfileCircle tool);

/**
 * Returns a GoProfileCircle StdDev measurement object.
 *
 * @public               @memberof GoProfileCircle
 * @version             Introduced in firmware 5.2.18.3
 * @param    tool        GoProfileCircle object.
 * @return               A GoProfileCircleRadius StdDev measurement (mm).
 */
GoFx(GoProfileCircleRadius) GoProfileCircle_StdDevMeasurement(GoProfileCircle tool);

/**
 * Returns a GoProfileCircle Min Error measurement object.
 *
 * @public               @memberof GoProfileCircle
 * @version             Introduced in firmware 5.2.18.3
 * @param    tool        GoProfileCircle object.
 * @return               A GoProfileCircleRadius Min Error measurement (mm).
 */
GoFx(GoProfileCircleRadius) GoProfileCircle_MinErrorMeasurement(GoProfileCircle tool);

/**
 * Returns a GoProfileCircle Min Error X measurement object.
 *
 * @public               @memberof GoProfileCircle
 * @version             Introduced in firmware 5.2.18.3
 * @param    tool        GoProfileCircle object.
 * @return               A GoProfileCircleRadius Min Error X measurement (mm).
 */
GoFx(GoProfileCircleRadius) GoProfileCircle_MinErrorXMeasurement(GoProfileCircle tool);

/**
 * Returns a GoProfileCircle Min Error Z measurement object.
 *
 * @public               @memberof GoProfileCircle
 * @version             Introduced in firmware 5.2.18.3
 * @param    tool        GoProfileCircle object.
 * @return               A GoProfileCircleRadius Min Error Z measurement (mm).
 */
GoFx(GoProfileCircleRadius) GoProfileCircle_MinErrorZMeasurement(GoProfileCircle tool);

/**
 * Returns a GoProfileCircle Max Error measurement object.
 *
 * @public               @memberof GoProfileCircle
 * @version             Introduced in firmware 5.2.18.3
 * @param    tool        GoProfileCircle object.
 * @return               A GoProfileCircleRadius Max Error measurement (mm).
 */
GoFx(GoProfileCircleRadius) GoProfileCircle_MaxErrorMeasurement(GoProfileCircle tool);

/**
 * Returns a GoProfileCircle Max Error X measurement object.
 *
 * @public               @memberof GoProfileCircle
 * @version             Introduced in firmware 5.2.18.3
 * @param    tool        GoProfileCircle object.
 * @return               A GoProfileCircleRadius Max Error X measurement (mm).
 */
GoFx(GoProfileCircleRadius) GoProfileCircle_MaxErrorXMeasurement(GoProfileCircle tool);

/**
 * Returns a GoProfileCircle Max Error Z measurement object.
 *
 * @public               @memberof GoProfileCircle
 * @version             Introduced in firmware 5.2.18.3
 * @param    tool        GoProfileCircle object.
 * @return               A GoProfileCircleRadius Max Error Z measurement (mm).
 */
GoFx(GoProfileCircleRadius) GoProfileCircle_MaxErrorZMeasurement(GoProfileCircle tool);


/**
* Returns a GoProfileCircle center point feature object.
*
* @public               @memberof GoProfileCircle
* @version              Introduced in firmware 4.0.10.27
* @param    tool        GoProfileCircle object.
* @return               A GoProfileCircleCenterPoint center point feature .
*/
GoFx(GoProfileCircleCenterPoint) GoProfileCircle_CenterPoint(GoProfileCircle tool);

/**
 * @class   GoProfileDim
 * @extends GoProfileTool
 * @ingroup GoSdk-ProfileTools
 * @brief   Represents a profile dimension tool.
 */
typedef GoProfileTool GoProfileDim; 


/** 
 * Gets the reference profile feature.
 *
 * @public               @memberof GoProfileDim
 * @version              Introduced in firmware 4.0.10.27
 * @param    tool        GoProfileDim object.
 * @return               The reference profile feature object.
 */
GoFx(GoProfileFeature) GoProfileDim_RefFeature(GoProfileDim tool);

/** 
 * Gets the non-reference profile feature.
 *
 * @public               @memberof GoProfileDim
 * @version              Introduced in firmware 4.0.10.27
 * @param    tool        GoProfileDim object.
 * @return               The profile feature object.
 */
GoFx(GoProfileFeature) GoProfileDim_Feature(GoProfileDim tool);

/**
 * Returns a GoProfileDim Width measurement object.
 *
 * @public              @memberof GoProfileDim
 * @version             Introduced in firmware 4.0.10.27
 * @param    tool       GoProfileDim object.
 * @return              A GoProfileDimWidth measurement.(mm)
 */
GoFx(GoProfileDimWidth) GoProfileDim_WidthMeasurement(GoProfileDim tool);

/**
 * Returns a GoProfileDim Height measurement object.
 *
 * @public              @memberof GoProfileDim
 * @version             Introduced in firmware 4.0.10.27
 * @param    tool       GoProfileDim object.
 * @return              A GoProfileDimHeight measurement.(mm)
 */
GoFx(GoProfileDimHeight) GoProfileDim_HeightMeasurement(GoProfileDim tool);

/**
 * Returns a GoProfileDim Distance measurement object.
 *
 * @public              @memberof GoProfileDim
 * @version             Introduced in firmware 4.0.10.27
 * @param    tool       GoProfileDim object.
 * @return              A GoProfileDimDistance measurement.(mm)
 */
GoFx(GoProfileDimDistance) GoProfileDim_DistanceMeasurement(GoProfileDim tool);

/**
 * Returns a GoProfileDim Center X measurement object.
 *
 * @public              @memberof GoProfileDim
 * @version             Introduced in firmware 4.0.10.27
 * @param    tool       GoProfileDim object.
 * @return              A GoProfileDimCenterX measurement.
 */
GoFx(GoProfileDimCenterX) GoProfileDim_CenterXMeasurement(GoProfileDim tool);

/**
 * Returns a GoProfileDim Center Z measurement object.
 *
 * @public              @memberof GoProfileDim
 * @version             Introduced in firmware 4.0.10.27
 * @param    tool       GoProfileDim object.
 * @return              A GoProfileDimCenterZ measurement.
 */
GoFx(GoProfileDimCenterZ) GoProfileDim_CenterZMeasurement(GoProfileDim tool);

/**
* Returns a GoProfileDim Center point feature object.
*
* @public              @memberof GoProfileDim
* @version             Introduced in firmware 4.0.10.27
* @param    tool       GoProfileDim object.
* @return              A GoProfileDimensionCenterPoint Center point feature.
*/
GoFx(GoProfileDimensionCenterPoint) GoProfileDim_CenterPoint(GoProfileDim tool);

/**
 * @class   GoProfileGroove
 * @extends GoProfileTool
 * @ingroup GoSdk-ProfileTools
 * @brief   Represents a profile groove tool.
 */
typedef GoProfileTool GoProfileGroove; 

/** 
 * Adds an additional profile groove tool measurement.
 *
 * @public                  @memberof GoProfileGroove
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoProfileGroove object.
 * @param    type           The measurement type to add. It must be a valid profile groove tool type.
 * @param    measurement    A reference to the new GoMeasurement handle. Can be kNULL if you do not wish to do anything immediate with the new measurement.
 * @return                  Operation status.
 */
GoFx(kStatus) GoProfileGroove_AddMeasurement(GoProfileGroove tool, GoMeasurementType type, GoMeasurement* measurement);

/** 
 * Removes a measurement from the tool at the given index.
 *
 * @public                  @memberof GoProfileGroove
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoProfileGroove object.
 * @param    index          The index with which to remove a measurement.
 * @return                  Operation status.
 */
GoFx(kStatus) GoProfileGroove_RemoveMeasurement(GoProfileGroove tool, kSize index);

/** 
 * Returns the measurement count for the given tool.
 *
 * @public                  @memberof GoProfileGroove
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoProfileGroove object.
 * @return                  Tool measurement count.
 */
GoFx(kSize) GoProfileGroove_MeasurementCount(GoProfileGroove tool);

/** 
 * Returns a measurement object at the given index.
 *
 * @public                  @memberof GoProfileGroove
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoProfileGroove object.
 * @param    index          The index with which to return a measurement object.
 * @return                  A profile groove tool measurement or kNULL if the index is invalid.
 */
GoFx(GoMeasurement) GoProfileGroove_MeasurementAt(GoProfileGroove tool, kSize index);

/** 
 * Gets the current groove determination shape.
 *
 * @public               @memberof GoProfileGroove
 * @version              Introduced in firmware 4.0.10.27
 * @param    tool        GoProfileGroove object.
 * @return               The profile groove shape.            
 */
GoFx(GoProfileGrooveShape) GoProfileGroove_Shape(GoProfileGroove tool);

/** 
 * Sets the groove determination shape.
 *
 * @public               @memberof GoProfileGroove
 * @version              Introduced in firmware 4.0.10.27
 * @param    tool        GoProfileGroove object.
 * @param    shape       The intended profile groove shape.
 * @return               Operation status.            
 */
GoFx(kStatus) GoProfileGroove_SetShape(GoProfileGroove tool, GoProfileGrooveShape shape);

/** 
 * Gets the groove depth minimum.
 *
 * @public               @memberof GoProfileGroove
 * @version              Introduced in firmware 4.0.10.27
 * @param    tool        GoProfileGroove object.
 * @return               The groove depth minimum value.(mm)            
 */
GoFx(k64f) GoProfileGroove_MinDepth(GoProfileGroove tool);

/** 
 * Sets the groove depth minimum.
 *
 * @public               @memberof GoProfileGroove
 * @version              Introduced in firmware 4.0.10.27
 * @param    tool        GoProfileGroove object.
 * @param    depth       The minimum groove depth value to set.(mm)
 * @return               Operation status.            
 */
GoFx(kStatus) GoProfileGroove_SetMinDepth(GoProfileGroove tool, k64f depth);

/** 
 * Gets the groove width maximum.
 *
 * @public               @memberof GoProfileGroove
 * @version              Introduced in firmware 4.0.10.27
 * @param    tool        GoProfileGroove object.
 * @return               The groove width maximum value.(mm)            
 */
GoFx(k64f) GoProfileGroove_MaxWidth(GoProfileGroove tool);

/** 
 * Sets the groove width maximum.
 *
 * @public               @memberof GoProfileGroove
 * @version              Introduced in firmware 4.0.10.27
 * @param    tool        GoProfileGroove object.
 * @param    width       The maximum groove width value to set.(mm)
 * @return               Operation status.            
 */
GoFx(kStatus) GoProfileGroove_SetMaxWidth(GoProfileGroove tool, k64f width);

/** 
 * Gets the groove width minimum value.
 *
 * @public               @memberof GoProfileGroove
 * @version              Introduced in firmware 4.0.10.27
 * @param    tool        GoProfileGroove object.
 * @return               The groove width minimum.(mm)            
 */
GoFx(k64f) GoProfileGroove_MinWidth(GoProfileGroove tool);

/** 
 * Sets the groove width minimum.
 *
 * @public               @memberof GoProfileGroove
 * @version              Introduced in firmware 4.0.10.27
 * @param    tool        GoProfileGroove object.
 * @param    width       The minimum groove width value to set.(mm)
 * @return               Operation status.            
 */
GoFx(kStatus) GoProfileGroove_SetMinWidth(GoProfileGroove tool, k64f width);

/** 
 * Gets the profile region.
 *
 * @public               @memberof GoProfileGroove
 * @version              Introduced in firmware 4.0.10.27
 * @param    tool        GoProfileGroove object.
 * @return               The profile region.            
 */
GoFx(GoProfileRegion) GoProfileGroove_Region(GoProfileGroove tool);

/** 
 * Indicates whether the region is enabled.
 *
 * @public               @memberof GoProfileGroove
 * @version              Introduced in firmware 4.4.4.14
 * @param    tool        GoProfileGroove object.
 * @return               kTRUE if enabled and kFALSE otherwise.
 */
GoFx(kBool) GoProfileGroove_RegionEnabled(GoProfileGroove tool);

/** 
 * Enables or disables the region.
 *
 * @public               @memberof GoProfileGroove
 * @version              Introduced in firmware 4.4.4.14
 * @param    tool        GoProfileGroove object.
 * @param    enable      kTRUE to enable the region and kFALSE to disable it.
 * @return               Operation status.            
 */
GoFx(kStatus) GoProfileGroove_EnableRegion(GoProfileGroove tool, kBool enable);

/**
 * @class   GoProfileIntersect
 * @extends GoProfileTool
 * @ingroup GoSdk-ProfileTools
 * @brief   Represents a profile intersect tool.
 */
typedef GoProfileTool GoProfileIntersect; 

/** 
 * Gets the reference profile line type.
 *
 * @public               @memberof GoProfileIntersect
 * @version              Introduced in firmware 4.0.10.27
 * @param    tool        GoProfileIntersect object.
 * @return   The profile line type.            
 */
GoFx(GoProfileBaseline) GoProfileIntersect_RefLineType(GoProfileIntersect tool);

/** 
 * Sets the reference line type.
 *
 * @public               @memberof GoProfileIntersect
 * @version              Introduced in firmware 4.0.10.27
 * @param    tool        GoProfileIntersect object.
 * @param    type        The line type to set.
 * @return               
 */
GoFx(kStatus) GoProfileIntersect_SetRefLineType(GoProfileIntersect tool, GoProfileBaseline type);

/** 
 * Gets the reference profile line.
 *
 * @public               @memberof GoProfileIntersect
 * @version              Introduced in firmware 4.0.10.27
 * @param    tool        GoProfileIntersect object.
 * @return               The reference profile line.
 */
GoFx(GoProfileLineRegion) GoProfileIntersect_RefLine(GoProfileIntersect tool);

/** 
 * Gets the non-reference profile line.
 *
 * @public               @memberof GoProfileIntersect
 * @version              Introduced in firmware 4.0.10.27
 * @param    tool        GoProfileIntersect object.
 * @return               The non-reference profile line.            
 */
GoFx(GoProfileLineRegion) GoProfileIntersect_Line(GoProfileIntersect tool);

/**
 * Returns a GoProfileIntersect X measurement object.
 *
 * @public               @memberof GoProfileIntersect
 * @version              Introduced in firmware 4.0.10.27
 * @param    tool        GoProfileIntersect object.
 * @return               A GoProfileIntersect X measurement. (mm)
 */
GoFx(GoProfileIntersectX) GoProfileIntersect_XMeasurement(GoProfileIntersect tool);

/**
 * Returns a GoProfileIntersect Z measurement object.
 *
 * @public              @memberof GoProfileIntersect
 * @version             Introduced in firmware 4.0.10.27
 * @param    tool       GoProfileIntersect object.
 * @return              A GoProfileIntersect Z measurement.(mm)
 */
GoFx(GoProfileIntersectZ) GoProfileIntersect_ZMeasurement(GoProfileIntersect tool);

/**
 * Returns a GoProfileIntersect Angle measurement object.
 *
 * @public              @memberof GoProfileIntersect
 * @version             Introduced in firmware 4.0.10.27
 * @param    tool       GoProfileIntersect object.
 * @return              A GoProfileIntersect Angle measurement.(degrees)
 */
GoFx(GoProfileIntersectAngle) GoProfileIntersect_AngleMeasurement(GoProfileIntersect tool);

/**
* Returns a GoProfileIntersect Angle measurement object.
*
* @public              @memberof GoProfileIntersect
* @version             Introduced in firmware 4.0.10.27
* @param    tool       GoProfileIntersect object.
* @return              A GoProfileIntersect Angle measurement.(degrees)
*/
GoFx(GoProfileIntersectIntersectPoint) GoProfileIntersect_PointFeature(GoProfileIntersect tool);

/**
* Returns a GoProfileIntersectLine Feature object.
*
* @public              @memberof GoProfileIntersect
* @version             Introduced in firmware 4.6.4.10
* @param    tool       GoProfileIntersect object.
* @return              A GoProfileIntersectLine feature object.
*/
GoFx(GoProfileIntersectLine) GoProfileIntersect_LineFeature(GoProfileIntersect tool);

/**
* Returns a GoProfileIntersectBaseLine Feature object.
*
* @public              @memberof GoProfileIntersect
* @version             Introduced in firmware 4.6.4.10
* @param    tool       GoProfileIntersect object.
* @return              A GoProfileIntersectBaseLine feature object.
*/
GoFx(GoProfileIntersectBaseLine) GoProfileIntersect_BaseLineFeature(GoProfileIntersect tool);

/**
 * @class   GoProfileLine
 * @extends GoProfileTool
 * @ingroup GoSdk-ProfileTools
 * @brief   Represents a profile line tool.
 */
typedef GoProfileTool GoProfileLine; 

/** 
 * Gets the measurement region.
 *
 * @public               @memberof GoProfileDev
 * @version              Introduced in firmware 4.0.10.27
 * @param    tool        GoProfileDev object.
 * @return               The profile Line Measurement region.            
 */
GoFx(GoProfileRegion) GoProfileLine_Region(GoProfileLine tool);

/** 
 * Indicates whether the region is enabled.
 *
 * @public               @memberof GoProfileLine
 * @version              Introduced in firmware 4.4.4.14
 * @param    tool        GoProfileLine object.
 * @return               kTRUE if enabled and kFALSE otherwise.
 */
GoFx(kBool) GoProfileLine_RegionEnabled(GoProfileLine tool);

/** 
 * Enables or disables the region.
 *
 * @public               @memberof GoProfileLine
 * @version              Introduced in firmware 4.4.4.14
 * @param    tool        GoProfileLine object.
 * @param    enable      kTRUE to enable the region and kFALSE to disable it.
 * @return               Operation status.            
 */
GoFx(kStatus) GoProfileLine_EnableRegion(GoProfileLine tool, kBool enable);

/**
* Gets the fitting regions.
*
* @public               @memberof GoProfileDev
* @version              Introduced in firmware 4.6.0.95
* @param    tool        GoProfileLine object.
* @return               The profile Line fitting regions.
*/
GoFx(GoProfileLineRegion) GoProfileLine_FittingRegions(GoProfileLine tool);

/**
* Indicates whether the fitting regions are enabled.
*
* @public               @memberof GoProfileLine
* @version              Introduced in firmware 4.6.0.147
* @param    tool        GoProfileLine object.
* @return               kTRUE if enabled and kFALSE otherwise.
*/
GoFx(kBool) GoProfileLine_FittingRegionsEnabled(GoProfileLine tool);

/**
* Enables or disables the fitting regions.
*
* @public               @memberof GoProfileLine
* @version              Introduced in firmware 4.6.0.147
* @param    tool        GoProfileLine object.
* @param    enable      Boolean enable or disable fitting regions.
* @return               kTRUE if enabled and kFALSE otherwise.
*/
GoFx(kStatus) GoProfileLine_EnableFittingRegions(GoProfileLine tool, kBool enable);

/**
 * Returns a GoProfileLine Standard Deviation measurement object.
 *
 * @public           @memberof GoProfileLine
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoProfileLine object.
 * @return           A GoProfileLine Standard Deviation measurement.
 */
GoFx(GoProfileLineStdDev) GoProfileLine_StdDevMeasurement(GoProfileLine tool);

/**
 * Returns a GoProfileLine Maximum Error measurement object.
 *
 * @public           @memberof GoProfileLine
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoProfileLine object.
 * @return           A GoProfileLine Maximum Error measurement.
 */
GoFx(GoProfileLineMaxError) GoProfileLine_MaxErrorMeasurement(GoProfileLine tool);

/**
 * Returns a GoProfileLine Minimum Error measurement object.
 *
 * @public           @memberof GoProfileLine
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoProfileLine object.
 * @return           A GoProfileLine Minimum Error measurement.
 */
GoFx(GoProfileLineMinError) GoProfileLine_MinErrorMeasurement(GoProfileLine tool);

/**
 * Returns a GoProfileLine Percentile measurement object.
 *
 * @public           @memberof GoProfileLine
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoProfileLine object.
 * @return           A GoProfileLine Percentile measurement.
 */
GoFx(GoProfileLinePercentile) GoProfileLine_PercentileMeasurement(GoProfileLine tool);

/**
 * Returns a GoProfileLine Offset measurement object.
 *
 * @public           @memberof GoProfileLine
 * @version          Introduced in firmware 4.6.0.49
 * @param    tool    GoProfileLine object.
 * @return           A GoProfileLine Offset measurement.
 */
GoFx(GoProfileLineOffset) GoProfileLine_OffsetMeasurement(GoProfileLine tool);

/**
 * Returns a GoProfileLine Angle measurement object.
 *
 * @public           @memberof GoProfileLine
 * @version          Introduced in firmware 4.6.0.49
 * @param    tool    GoProfileLine object.
 * @return           A GoProfileLine Angle measurement.
 */
GoFx(GoProfileLineAngle) GoProfileLine_AngleMeasurement(GoProfileLine tool);

/**
 * Returns a GoProfileLine Minimum X Error measurement object.
 *
 * @public           @memberof GoProfileLine
 * @version          Introduced in firmware 4.6.0.49
 * @param    tool    GoProfileLine object.
 * @return           A GoProfileLine Minimum X Error measurement.
 */
GoFx(GoProfileLineMinErrorX) GoProfileLine_MinErrorXMeasurement(GoProfileLine tool);

/**
 * Returns a GoProfileLine Minimum Z Error measurement object.
 *
 * @public           @memberof GoProfileLine
 * @version          Introduced in firmware 4.6.0.49
 * @param    tool    GoProfileLine object.
 * @return           A GoProfileLine Minimum Z Error measurement.
 */
GoFx(GoProfileLineMinErrorZ) GoProfileLine_MinErrorZMeasurement(GoProfileLine tool);

/**
 * Returns a GoProfileLine Maximum X Error measurement object.
 *
 * @public           @memberof GoProfileLine
 * @version          Introduced in firmware 4.6.0.49
 * @param    tool    GoProfileLine object.
 * @return           A GoProfileLine Maximum X Error measurement.
 */
GoFx(GoProfileLineMaxErrorX) GoProfileLine_MaxErrorXMeasurement(GoProfileLine tool);

/**
 * Returns a GoProfileLine Maximum Z Error measurement object.
 *
 * @public           @memberof GoProfileLine
 * @version          Introduced in firmware 4.6.0.49
 * @param    tool    GoProfileLine object.
 * @return           A GoProfileLine Maximum Z Error measurement.
 */
GoFx(GoProfileLineMaxErrorZ) GoProfileLine_MaxErrorZMeasurement(GoProfileLine tool);

/**
* Returns a GoProfileLine line feature object.
*
* @public           @memberof GoProfileLine
* @version          Introduced in firmware 4.6.0.49
* @param    tool    GoProfileLine object.
* @return           A GoProfileLineLine line feature.
*/
GoFx(GoProfileLineLine) GoProfileLine_Line(GoProfileLine tool);

/**
* Returns a GoProfileLine Maximum Z Error measurement object.
*
* @public           @memberof GoProfileLine
* @version          Introduced in firmware 4.6.0.49
* @param    tool    GoProfileLineMinErrorPoint object.
* @return           A GoProfileLineMinErrorPoint min error point feature.
*/
GoFx(GoProfileLineMinErrorPoint) GoProfileLine_MinErrorPoint(GoProfileLine tool);

/**
* Returns a GoProfileLine min error point feature object.
*
* @public           @memberof GoProfileLine
* @version          Introduced in firmware 4.6.0.49
* @param    tool    GoProfileLineMaxErrorPoint object.
* @return           A GoProfileLineMaxErrorPoint min error point feature.
*/
GoFx(GoProfileLineMaxErrorPoint) GoProfileLine_MaxErrorPoint(GoProfileLine tool);

/**
 * @class   GoProfilePanel
 * @extends GoProfileTool
 * @ingroup GoSdk-ProfileTools
 * @brief   Represents a profile panel tool.
 */
typedef GoProfileTool GoProfilePanel; 

/** 
 * Gets the maximum gap width.
 *
 * @public               @memberof GoProfilePanel
 * @version              Introduced in firmware 4.0.10.27
 * @param    tool        GoProfilePanel object.
 * @return               The maximum gap width.(mm)             
 */
GoFx(k64f) GoProfilePanel_MaxGapWidth(GoProfilePanel tool);

/** 
 * Sets the maximum gap width.
 *
 * @public               @memberof GoProfilePanel
 * @version              Introduced in firmware 4.0.10.27
 * @param    tool        GoProfilePanel object.
 * @param    width       The maximum gap width value to set.(mm)
 * @return               Operation status.            
 */
GoFx(kStatus) GoProfilePanel_SetMaxGapWidth(GoProfilePanel tool, k64f width);

/** 
 * Gets the reference edge side.
 *
 * @public               @memberof GoProfilePanel
 * @version              Introduced in firmware 4.0.10.27
 * @param    tool        GoProfilePanel object.
 * @return               The reference edge side.            
 */
GoFx(GoProfilePanelSide) GoProfilePanel_RefEdgeSide(GoProfilePanel tool);

/** 
 * Sets the reference edge side.
 *
 * @public               @memberof GoProfilePanel
 * @version              Introduced in firmware 4.0.10.27
 * @param    tool        GoProfilePanel object.
 * @param    side        The reference edge side.
 * @return               Operation status.            
 */
GoFx(kStatus) GoProfilePanel_SetRefEdgeSide(GoProfilePanel tool, GoProfilePanelSide side);

/** 
 * Gets the left profile edge.
 *
 * @public               @memberof GoProfilePanel
 * @version              Introduced in firmware 4.0.10.27
 * @param    tool        GoProfilePanel object.
 * @return               The left profile edge.            
 */
GoFx(GoProfileEdge) GoProfilePanel_LeftEdge(GoProfilePanel tool);

/** 
 * Gets the right profile edge.
 *
 * @public               @memberof GoProfilePanel
 * @version              Introduced in firmware 4.0.10.27
 * @param    tool        GoProfilePanel object.
 * @return               The right profile edge.            
 */
GoFx(GoProfileEdge) GoProfilePanel_RightEdge(GoProfilePanel tool);

/**
 * Returns a GoProfilePanel Gap measurement object.
 *
 * @public           @memberof GoProfilePanel
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoProfilePanel object.
 * @return           A GoProfilePanel Gap measurement.
 */
GoFx(GoProfilePanelGap) GoProfilePanel_GapMeasurement(GoProfilePanel tool);

/**
 * Returns a GoProfilePanel Flush measurement object.
 *
 * @public           @memberof GoProfilePanel
 * @version          Introduced in firmware 4.0.10.27
 * @param    tool    GoProfilePanel object.
 * @return           A GoProfilePanel Flush measurement.
 */
GoFx(GoProfilePanelFlush) GoProfilePanel_FlushMeasurement(GoProfilePanel tool);

/**
* Returns a GoProfilePanel Left Gap X measurement object.
*
* @public           @memberof GoProfilePanel
* @version          Introduced in firmware 4.0.10.27
* @param    tool    GoProfilePanel object.
* @return           A GoProfilePanel Left Gap X measurement.
*/
GoFx(GoProfilePanelLeftGapX) GoProfilePanel_LeftGapXMeasurement(GoProfilePanel tool);

/**
* Returns a GoProfilePanel Left Gap Z measurement object.
*
* @public           @memberof GoProfilePanel
* @version          Introduced in firmware 4.0.10.27
* @param    tool    GoProfilePanel object.
* @return           A GoProfilePanel Left Gap Z measurement.
*/
GoFx(GoProfilePanelLeftGapZ) GoProfilePanel_LeftGapZMeasurement(GoProfilePanel tool);

/**
* Returns a GoProfilePanel Left Flush X measurement object.
*
* @public           @memberof GoProfilePanel
* @version          Introduced in firmware 4.0.10.27
* @param    tool    GoProfilePanel object.
* @return           A GoProfilePanel Left Flush X measurement.
*/
GoFx(GoProfilePanelLeftFlushX) GoProfilePanel_LeftFlushXMeasurement(GoProfilePanel tool);

/**
* Returns a GoProfilePanel Left Flush Z measurement object.
*
* @public           @memberof GoProfilePanel
* @version          Introduced in firmware 4.0.10.27
* @param    tool    GoProfilePanel object.
* @return           A GoProfilePanel Left Flush Z measurement.
*/
GoFx(GoProfilePanelLeftFlushZ) GoProfilePanel_LeftFlushZMeasurement(GoProfilePanel tool);

/**
* Returns a GoProfilePanel Left Surface Angle measurement object.
*
* @public           @memberof GoProfilePanel
* @version          Introduced in firmware 4.0.10.27
* @param    tool    GoProfilePanel object.
* @return           A GoProfilePanel Left Surface Angle measurement.
*/
GoFx(GoProfilePanelLeftSurfaceAngle) GoProfilePanel_LeftSurfaceAngleMeasurement(GoProfilePanel tool);

/**
* Returns a GoProfilePanel Right Gap X measurement object.
*
* @public           @memberof GoProfilePanel
* @version          Introduced in firmware 4.0.10.27
* @param    tool    GoProfilePanel object.
* @return           A GoProfilePanel Right Gap X measurement.
*/
GoFx(GoProfilePanelRightGapX) GoProfilePanel_RightGapXMeasurement(GoProfilePanel tool);

/**
* Returns a GoProfilePanel Right Gap Z measurement object.
*
* @public           @memberof GoProfilePanel
* @version          Introduced in firmware 4.0.10.27
* @param    tool    GoProfilePanel object.
* @return           A GoProfilePanel Right Gap Z measurement.
*/
GoFx(GoProfilePanelRightGapZ) GoProfilePanel_RightGapZMeasurement(GoProfilePanel tool);

/**
* Returns a GoProfilePanel Right Flush Z measurement object.
*
* @public           @memberof GoProfilePanel
* @version          Introduced in firmware 4.0.10.27
* @param    tool    GoProfilePanel object.
* @return           A GoProfilePanel Right Flush Z measurement.
*/
GoFx(GoProfilePanelRightFlushZ) GoProfilePanel_RightFlushZMeasurement(GoProfilePanel tool);

/**
* Returns a GoProfilePanel Right Surface Angle measurement object.
*
* @public           @memberof GoProfilePanel
* @version          Introduced in firmware 4.0.10.27
* @param    tool    GoProfilePanel object.
* @return           A GoProfilePanel Right Surface Angle measurement.
*/
GoFx(GoProfilePanelRightSurfaceAngle) GoProfilePanel_RightSurfaceAngleMeasurement(GoProfilePanel tool);

/**
 * @class   GoProfilePosition
 * @extends GoProfileTool
 * @ingroup GoSdk-ProfileTools
 * @brief   Represents a profile position tool.
 */
typedef GoProfileTool GoProfilePosition; 

/**
* Indicates whether the region is enabled.
*
* @public               @memberof GoProfilePosition
 * @version             Introduced in firmware 4.8.2.76
* @param    tool        GoProfilePosition object.
* @return               kTRUE if enabled and kFALSE otherwise.
*/
GoFx(kBool) GoProfilePosition_RegionEnabled(GoProfilePosition tool);

/**
* Enables or disables the region.
*
* @public               @memberof GoProfilePosition
 * @version             Introduced in firmware 4.8.2.76
* @param    tool        GoProfilePosition object.
* @param    enable      kTRUE to enable the region and kFALSE to disable it.
* @return               Operation status.
*/
GoFx(kStatus) GoProfilePosition_EnableRegion(GoProfilePosition tool, kBool enable);
/** 
 * Gets the profile feature.
 *
 * @public               @memberof GoProfilePosition
 * @version              Introduced in firmware 4.0.10.27
 * @param    tool        GoProfilePos object.
 * @return               The profile feature.            
 */
GoFx(GoProfileFeature) GoProfilePosition_Feature(GoProfilePosition tool);

/**
 * Returns a GoProfilePosition X measurement object.
 *
 * @public              @memberof GoProfilePosition
 * @version             Introduced in firmware 4.0.10.27
 * @param    tool       GoProfilePosition object.
 * @return              A GoProfilePosition X measurement.(mm)
 */
GoFx(GoProfilePositionX) GoProfilePosition_XMeasurement(GoProfilePosition tool);

/**
 * Returns a GoProfilePosition Z measurement object.
 *
 * @public              @memberof GoProfilePosition
 * @version             Introduced in firmware 4.0.10.27
 * @param    tool       GoProfilePosition object.
 * @return              A GoProfilePosition Z measurement.(mm)
 */
GoFx(GoProfilePositionZ) GoProfilePosition_ZMeasurement(GoProfilePosition tool);

/**
* Returns a GoProfilePosition point feature object.
*
* @public              @memberof GoProfilePosition
* @version             Introduced in firmware 4.0.10.27
* @param    tool       GoProfilePosition object.
* @return              A GoProfilePosition point feature.
*/
GoFx(GoProfilePositionPoint) GoProfilePosition_Point(GoProfilePosition tool);

/**
 * @class   GoProfileStrip
 * @extends GoProfileTool
 * @ingroup GoSdk-ProfileTools
 * @brief   Represents a profile strip tool.
 */
typedef GoProfileTool GoProfileStrip; 

/** 
 * Sets the strip base type.
 *
 * @public                  @memberof GoProfileStrip
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoProfileStrip object.
 * @param    type           The strip base type.
 * @return                  Operation status.
 */
GoFx(kStatus) GoProfileStrip_SetBaseType(GoProfileStrip tool, GoProfileStripBaseType type);

/** 
 * Gets the strip base type.
 *
 * @public                  @memberof GoProfileStrip
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoProfileStrip object.
 * @return                  The strip base type.
 */
GoFx(GoProfileStripBaseType) GoProfileStrip_BaseType(GoProfileStrip tool);

/** 
 * Gets the left edge value.
 *
 * @public                  @memberof GoProfileStrip
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoProfileStrip object.
 * @return                  The left edge value.
 */
GoFx(GoProfileStripEdgeType) GoProfileStrip_LeftEdge(GoProfileStrip tool);

/** 
 * Sets the left edge.
 *
 * @public                  @memberof GoProfileStrip
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoProfileStrip object.
 * @param    leftEdge       Left edge value.
 * @return                  Operation status.
 */
GoFx(kStatus) GoProfileStrip_SetLeftEdge(GoProfileStrip tool, GoProfileStripEdgeType leftEdge);

/** 
 * Gets the right edge value.
 *
 * @public                  @memberof GoProfileStrip
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoProfileStrip object.
 * @return                  The right edge value.
 */
GoFx(GoProfileStripEdgeType) GoProfileStrip_RightEdge(GoProfileStrip tool);

/** 
 * Sets the right edge.
 *
 * @public                  @memberof GoProfileStrip
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoProfileStrip object.
 * @param    rightEdge      Right edge value.
 * @return                  Operation status.
 */
GoFx(kStatus) GoProfileStrip_SetRightEdge(GoProfileStrip tool, GoProfileStripEdgeType rightEdge);

/** 
 * Gets the tilt enabled state.
 *
 * @public                  @memberof GoProfileStrip
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoProfileStrip object.
 * @return                  kTRUE if tilt is enabled, kFALSE otherwise.
 */
GoFx(kBool) GoProfileStrip_TiltEnabled(GoProfileStrip tool);

/** 
 * Enables or disables tilt.
 *
 * @public                  @memberof GoProfileStrip
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoProfileStrip object.
 * @param    enable         kTRUE to enable tilt, kFALSE to disable it.
 * @return                  Operation status.
 */
GoFx(kStatus) GoProfileStrip_EnableTilt(GoProfileStrip tool, kBool enable);


/** 
 * Gets the support width.
 *
 * @public                  @memberof GoProfileStrip
 * @version                 Introduced in firmware 4.5.3.57
 * @param    tool           GoProfileStrip object.
 * @return                  The support width (in mm).
 */
GoFx(k64f) GoProfileStrip_SupportWidth(GoProfileStrip tool);

/** 
 * Sets the support width.
 *
 * @public                  @memberof GoProfileStrip
 * @version                 Introduced in firmware 4.5.3.57
 * @param    tool           GoProfileStrip object.
 * @param    value          The support width (in mm).
 * @return                  Operation status.
 */
GoFx(kStatus) GoProfileStrip_SetSupportWidth(GoProfileStrip tool, k64f value);


/** 
 * Gets the transition width limit minimum.
 *
 * @public                  @memberof GoProfileStrip
 * @version                 Introduced in firmware 4.5.3.57
 * @param    tool           GoProfileStrip object.
 * @return                  The minimum permissible transition width (in mm).
 */
GoFx(k64f) GoProfileStrip_TransitionWidthLimitMin(GoProfileStrip tool);

/** 
 * Gets the transition width limit maximum.
 *
 * @public                  @memberof GoProfileStrip
 * @version                 Introduced in firmware 4.5.3.57
 * @param    tool           GoProfileStrip object.
 * @return                  The maximum permissible transition width (in mm).
 */
GoFx(k64f) GoProfileStrip_TransitionWidthLimitMax(GoProfileStrip tool);

/** 
 * Gets the transition width.
 *
 * @public                  @memberof GoProfileStrip
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoProfileStrip object.
 * @return                  The transition width (in mm).
 */
GoFx(k64f) GoProfileStrip_TransitionWidth(GoProfileStrip tool);

/** 
 * Sets the transition width.
 *
 * @public                  @memberof GoProfileStrip
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoProfileStrip object.
 * @param    value          The transition width (in mm).
 * @return                  Operation status.
 */
GoFx(kStatus) GoProfileStrip_SetTransitionWidth(GoProfileStrip tool, k64f value);

/** 
 * Gets the minimum width.
 *
 * @public                  @memberof GoProfileStrip
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoProfileStrip object.
 * @return                  The minimum width (in mm).
 */
GoFx(k64f) GoProfileStrip_MinWidth(GoProfileStrip tool);

/** 
 * Sets the minimum width.
 *
 * @public                  @memberof GoProfileStrip
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoProfileStrip object.
 * @param    value          The minimum width (in mm).
 * @return                  Operation status.
 */
GoFx(kStatus) GoProfileStrip_SetMinWidth(GoProfileStrip tool, k64f value);

/** 
 * Gets the minimum height.
 *
 * @public                  @memberof GoProfileStrip
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoProfileStrip object.
 * @return                  The minimum height (in mm).
 */
GoFx(k64f) GoProfileStrip_MinHeight(GoProfileStrip tool);

/** 
 * Sets the transition height.
 *
 * @public                  @memberof GoProfileStrip
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoProfileStrip object.
 * @param    value          The minimum height (in mm).
 * @return                  Operation status.
 */
GoFx(kStatus) GoProfileStrip_SetMinHeight(GoProfileStrip tool, k64f value);

/** 
 * Gets the maximum void width.
 *
 * @public                  @memberof GoProfileStrip
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoProfileStrip object.
 * @return                  The maximum void width (in mm).
 */
GoFx(k64f) GoProfileStrip_MaxVoidWidth(GoProfileStrip tool);

/** 
 * Sets the maximum void width.
 *
 * @public                  @memberof GoProfileStrip
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoProfileStrip object.
 * @param    value          The maximum void width (in mm).
 * @return                  Operation status.
 */
GoFx(kStatus) GoProfileStrip_SetMaxVoidWidth(GoProfileStrip tool, k64f value);

/** 
 * Gets the profile strip tool region.
 *
 * @public                  @memberof GoProfileStrip
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoProfileStrip object.
 * @return                  A GoProfileRegion object.
 */
GoFx(GoProfileRegion) GoProfileStrip_Region(GoProfileStrip tool);

/** 
 * Indicates whether the region is enabled.
 *
 * @public               @memberof GoProfileStrip
 * @version              Introduced in firmware 4.4.4.14
 * @param    tool        GoProfileStrip object.
 * @return               kTRUE if enabled and kFALSE otherwise.
 */
GoFx(kBool) GoProfileStrip_RegionEnabled(GoProfileStrip tool);

/** 
 * Enables or disables the region.
 *
 * @public               @memberof GoProfileStrip
 * @version              Introduced in firmware 4.4.4.14
 * @param    tool        GoProfileStrip object.
 * @param    enable      kTRUE to enable the region and kFALSE to disable it.
 * @return               Operation status.            
 */
GoFx(kStatus) GoProfileStrip_EnableRegion(GoProfileStrip tool, kBool enable);

/** 
 * Adds an additional profile strip tool measurement.
 *
 * @public                  @memberof GoProfileStrip
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoProfileStrip object.
 * @param    type           The measurement type to add. It must be a valid profile strip tool type.
 * @param    measurement    A reference to the new GoMeasurement handle. Can be kNULL if you do not wish to do anything immediate with the new measurement.
 * @return                  Operation status.
 */
GoFx(kStatus) GoProfileStrip_AddMeasurement(GoProfileStrip tool, GoMeasurementType type, GoMeasurement* measurement);

/** 
 * Removes a measurement from the tool at the given index.
 *
 * @public                  @memberof GoProfileStrip
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoProfileStrip object.
 * @param    index          The index with which to remove a measurement.
 * @return                  Operation status.
 */
GoFx(kStatus) GoProfileStrip_RemoveMeasurement(GoProfileStrip tool, kSize index);

/** 
 * Returns the measurement count for the given tool.
 *
 * @public                  @memberof GoProfileStrip
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoProfileStrip object.
 * @return                  Tool measurement count.
 */
GoFx(kSize) GoProfileStrip_MeasurementCount(GoProfileStrip tool);

/** 
 * Returns a measurement object at the given index.
 *
 * @public                  @memberof GoProfileStrip
 * @version                 Introduced in firmware 4.0.10.27
 * @param    tool           GoProfileStrip object.
 * @param    index          The index with which to return a measurement object.
 * @return                  A profile strip tool measurement or kNULL if the index is invalid.
 */
GoFx(GoMeasurement) GoProfileStrip_MeasurementAt(GoProfileStrip tool, kSize index);

/**
 * @class   GoProfileRoundCorner
 * @extends GoProfileTool
 * @ingroup GoSdk-ProfileTools
 * @brief   Represents a profile round corner tool.
 */
typedef GoProfileTool GoProfileRoundCorner; 

/**
 * Returns a GoProfileRoundCorner X measurement object.
 *
 * @public              @memberof GoProfileRoundCorner
 * @version             Introduced in firmware 4.0.10.27
 * @param    tool       GoProfileRoundCorner object.
 * @return              A GoProfileRoundCorner X measurement.(mm)
 */
GoFx(GoProfileRoundCornerX) GoProfileRoundCorner_XMeasurement(GoProfileRoundCorner tool);

/**
 * Returns a GoProfileRoundCorner Z measurement object.
 *
 * @public              @memberof GoProfileRoundCorner
 * @version             Introduced in firmware 4.0.10.27
 * @param    tool       GoProfileRoundCorner object.
 * @return              A GoProfileRoundCorner Z measurement.(mm)
 */
GoFx(GoProfileRoundCornerZ) GoProfileRoundCorner_ZMeasurement(GoProfileRoundCorner tool);

/**
 * Returns a GoProfileRoundCorner Angle measurement object.
 *
 * @public              @memberof GoProfileRoundCorner
 * @version             Introduced in firmware 4.0.10.27
 * @param    tool       GoProfileRoundCorner object.
 * @return              A GoProfileRoundCorner Angle measurement.(degrees)
 */
GoFx(GoProfileRoundCornerAngle) GoProfileRoundCorner_AngleMeasurement(GoProfileRoundCorner tool);

/** 
 * Gets the edge.
 *
 * @public               @memberof GoProfileRoundCorner
 * @version              Introduced in firmware 4.0.10.27
 * @param    tool        GoProfileRoundCorner object.
 * @return               The edge.            
 */
GoFx(GoProfileEdge) GoProfileRoundCorner_Edge(GoProfileRoundCorner tool);

/** 
 * Sets the edge.
 *
 * @public               @memberof GoProfileRoundCorner
 * @version              Introduced in firmware 4.0.10.27
 * @param    tool        GoProfileRoundCorner object.
 * @param    edge        The edge.
 * @return               Operation status.            
 */
GoFx(kStatus) GoProfileRoundCorner_SetEdge(GoProfileRoundCorner tool, GoProfileEdge edge);

/** 
 * Gets the reference direction.
 *
 * @public               @memberof GoProfileRoundCorner
 * @version              Introduced in firmware 4.0.10.27
 * @param    tool        GoProfileRoundCorner object.
 * @return               The reference direction.            
 */
GoFx(GoProfileRoundCornerDirection) GoProfileRoundCorner_RefDirection(GoProfileRoundCorner tool);

/** 
 * Sets the reference direction.
 *
 * @public               @memberof GoProfileRoundCorner
 * @version              Introduced in firmware 4.0.10.27
 * @param    tool        GoProfileRoundCorner object.
 * @param    direction   The reference direction.
 * @return               Operation status.            
 */
GoFx(kStatus) GoProfileRoundCorner_SetRefDirection(GoProfileRoundCorner tool, GoProfileRoundCornerDirection direction);

/**
* 
*
* @public               @memberof GoProfileRoundCorner
* @version              Introduced in firmware 4.0.10.27
* @param    tool        GoProfileRoundCorner object.
* @return               A GoProfileRoundCornerPoint point feature
*/
GoFx(GoProfileRoundCornerPoint) GoProfileRoundCorner_Point(GoProfileRoundCorner tool);

/**
* gets the round corner edge point
*
* @public               @memberof GoProfileRoundCorner
* @version              Introduced in firmware 4.8.1.80
* @param    tool        GoProfileRoundCorner object.
* @return               A GoProfileRoundCornerEdgePoint point feature
*/
GoFx(GoProfileRoundCornerPoint) GoProfileRoundCorner_EdgePoint(GoProfileRoundCorner tool);

/**
* gets the round corner center point
*
* @public               @memberof GoProfileRoundCorner
* @version              Introduced in firmware 4.8.1.80
* @param    tool        GoProfileRoundCorner object.
* @return               A GoProfileRoundCornerCenterPoint point feature
*/
GoFx(GoProfileRoundCornerCenterPoint) GoProfileRoundCorner_CenterPoint(GoProfileRoundCorner tool);

#include <GoSdk/Tools/GoProfileTools.x.h>

#endif

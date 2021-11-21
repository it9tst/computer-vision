/** 
 * @file    GoTool.h
 * @brief   Declares the base GoTool class.
 *
 * @internal
 * Copyright (C) 2016-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef GO_TOOL_H
#define GO_TOOL_H

#include <GoSdk/GoSdkDef.h>
#include <GoSdk/Tools/GoMeasurements.h>
#include <GoSdk/Tools/GoMeasurement.h>
#include <GoSdk/Tools/GoFeatures.h>
#include <GoSdk/Tools/GoFeature.h>

#include <kApi/Data/kXml.h>

/**
 * @class   GoTool
 * @extends kObject
 * @ingroup GoSdk-Tools
 * @brief   Represents the base tool class.
 */
typedef kObject GoTool; 

/** 
 * Returns the measurement count.
 *
 * @public              @memberof GoTool
 * @version             Introduced in firmware 4.0.10.27
 * @param    tool       GoTool object.
 * @return              The measurement count.            
 */
GoFx(kSize) GoTool_MeasurementCount(GoTool tool);

/** 
 * Retrieves the measurement at the given index.
 *
 * @public              @memberof GoTool
 * @version             Introduced in firmware 4.0.10.27
 * @param    tool       GoTool object.
 * @param    index      The index of the measurement.
 * @return              The measurement at the given index or kNULL if an invalid index is provided.            
 */
GoFx(GoMeasurement) GoTool_MeasurementAt(GoTool tool, kSize index);

/**
* Returns the feature output count.
*
* @public              @memberof GoTool
* @version             Introduced in firmware 4.6.3.27
* @param    tool       GoTool object.
* @return              The feature output count.
*/
GoFx(kSize) GoTool_FeatureOutputCount(GoTool tool);

/**
* Retrieves the feature output at the given index.
*
* @public              @memberof GoTool
* @version             Introduced in firmware 4.6.3.27
* @param    tool       GoTool object.
* @param    index      The index of the feature output.
* @return              The feature output at the given index or kNULL if an invalid index is provided.
*/
GoFx(GoFeature) GoTool_FeatureOutputAt(GoTool tool, kSize index);

/** 
 * Sets the name of the tool.
 *
 * @public              @memberof GoTool
 * @version             Introduced in firmware 4.0.10.27
 * @param    tool       GoTool object.
 * @param    name       The name to be set for the tool.
 * @return              Operation status.            
 */
GoFx(kStatus) GoTool_SetName(GoTool tool, const kChar* name);

/** 
 * Retrieves the name of the tool.
 *
 * @public              @memberof GoTool
 * @version             Introduced in firmware 4.0.10.27
 * @param    tool       GoTool object.
 * @param    name       Receives the name of the tool.
 * @param    capacity   The maximum capacity of the name array.
 * @return              Operation status.            
 */
GoFx(kStatus) GoTool_Name(GoTool tool, kChar* name, kSize capacity);

/** 
 * Retrieves the id of the instance of the tool.
 *
 * @public              @memberof GoTool
 * @version             Introduced in firmware 5.1.1.28
 * @param    tool       GoTool object.
 * @return              The instance id of the tool.            
 */
GoFx(k32s) GoTool_Id(GoTool tool);

/** 
 * Retrieves the tool type enumeration value of the tool.
 *
 * @public              @memberof GoTool
 * @version             Introduced in firmware 4.0.10.27
 * @param    tool       GoTool object.
 * @return              The tool type enumeration value.            
 */
GoFx(GoToolType) GoTool_Type(GoTool tool);

/** 
 * Retrieves the first found instance of a measurement for a given enumeration type.
 *
 * @public              @memberof GoTool
 * @version             Introduced in firmware 4.0.10.27
 * @param    tool       GoTool object.
 * @param    type       A GoMeasurementType representing the measurement type to find in the given tool.
 * @return              A measurement object if one is found, otherwise kNULL.
 */
GoFx(GoMeasurement) GoTool_FindMeasurementByType(GoTool tool, GoMeasurementType type);

/**
* Retrieves the instance of a feature output for a given enumeration type.
*
* @public              @memberof GoTool
* @version             Introduced in firmware 4.6.3.27
* @param    tool       GoTool object.
* @param    type       A GoFeatureType representing the feature output type to find in the given tool.
* @return              A feature object if one is found, otherwise kNULL. Returns the first found.
*/
GoFx(GoFeature) GoTool_FindFeatureOutputByType(GoTool tool, GoFeatureType type);

/**
* Adds the given measurement to the tool set.
*
* @public                  @memberof GoTool
* @version                 Introduced in firmware 4.8.2.76
* @param    tool           GoTool object.
* @param    type           The type of the measurement.
* @param    isFilterable   Indicates whether the measurement can be toggled and filtered;
* @param    measurement    A pointer to hold a reference to the created measurement (can be null).
* @return   Operation status.
*/
GoFx(kStatus) GoTool_AddMeasurement(GoTool tool, kType type, kBool isFilterable, GoMeasurement* measurement);

/**
* [Deprecated] 
* 
* Adds the given custom measurement to the tool set.
*
* @deprecated
* @public                  @memberof GoTool
* @version                 Introduced in firmware 4.8.2.76
* @param    tool           GoTool object.
* @param    type           The type of the measurement.
* @param    isFilterable   Indicates whether the measurement can be toggled and filtered;
* @param    measurement    A pointer to hold a reference to the created measurement (can be null).
* @return   Operation status.
*/
GoFx(kStatus) GoTool_AddExtMeasurement(GoTool tool, kType type, kBool isFilterable, GoExtMeasurement* measurement);

/**
* Removes a measurement at a given index.
*
* @public               @memberof GoTool
* @version             Introduced in firmware 4.8.2.76
* @param    tool        GoTool object.
* @param    index       The index of the measurement to remove.
* @return   Operation status.
*/
GoFx(kStatus) GoTool_RemoveMeasurement(GoTool tool, kSize index);

/**
* Removes all measurements for the given tool.
*
* @public               @memberof GoTool
* @version             Introduced in firmware 4.8.2.76
* @param    tool        GoTool object.
* @return   Operation status.
*/
GoFx(kStatus) GoTool_ClearMeasurements(GoTool tool);

/**
* Adds the given feature output to the tool set.
*
* @public                  @memberof GoTool
* @version                 Introduced in firmware 5.1.4.38
* @param    tool           GoTool object.
* @param    type           The type of the feature.
* @param    featureOutput  A pointer to hold a reference to the created feature output (can be null).
* @return   Operation status.
*/
GoFx(kStatus) GoTool_AddFeatureOutput(GoTool tool, kType type, GoFeature* featureOutput);

/**
* Removes a feature output at a given index.
*
* @public               @memberof GoTool
 * @version             Introduced in firmware 5.1.4.38
* @param    tool        GoTool object.
* @param    index       The index of the feature output to remove.
* @return   Operation status.
*/
GoFx(kStatus) GoTool_RemoveFeatureOutput(GoTool tool, kSize index);

/**
* Removes all feature outputs for the given tool.
*
* @public               @memberof GoTool
 * @version             Introduced in firmware 5.1.4.38
* @param    tool        GoTool object.
* @return   Operation status.
*/
GoFx(kStatus) GoTool_ClearFeatureOutputs(GoTool tool);

#include <GoSdk/Tools/GoTool.x.h>

#endif

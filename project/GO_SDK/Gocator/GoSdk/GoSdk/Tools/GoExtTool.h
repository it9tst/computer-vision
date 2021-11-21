/**
 * @file    GoExtTool.h
 * @brief   Declares the base GoExtTool class.
 *
 * @internal
 * Copyright (C) 2016-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef GO_EXT_TOOL_H
#define GO_EXT_TOOL_H

#include <GoSdk/GoSdkDef.h>
#include <GoSdk/Tools/GoTool.h>
#include <GoSdk/Tools/GoExtParams.h>
#include <GoSdk/Tools/GoExtToolDataOutput.h>
#include <kApi/Data/kXml.h>

/**
 * @class   GoExtTool
 * @extends GoTool
 * @ingroup GoSdk-Tools
 * @brief   Represents an extensible tool.
 */
typedef GoTool GoExtTool;

/**
 * Returns the tool configuration version string.
 *
 * @public              @memberof GoExtTool
 * @version             Introduced in firmware 4.4.4.14
 * @param    tool       GoExtTool object.
 * @return              The tool configuration version string.
 */
GoFx(const kChar*) GoExtTool_Version(GoExtTool tool);

/**
 * Sets the data stream. Note that stream validation will only occur if tool
 * is in the tool options list.
 *
 * @public              @memberof GoExtTool
 * @version             Introduced in firmware 5.3.19.50
 * @param    tool       GoExtTool object.
 * @param    stream     GoDataStream value.
 * @return              Operation status.
 * @see                 GoExtTool_StreamOptionCount, GoExtTool_StreamOptionAt
 */
GoFx(kStatus) GoExtTool_SetStream(GoExtTool tool, GoDataStream stream);

/**
 * Gets the data stream.
 *
 * @public              @memberof GoExtTool
 * @version             Introduced in firmware 5.3.19.50
 * @param    tool       GoExtTool object.
 * @return              The current Ext tool data stream value.
 */
GoFx(GoDataStream) GoExtTool_Stream(GoExtTool tool);

/**
 * Gets the data stream option list count.
 *
 * @public              @memberof GoExtTool
 * @version             Introduced in firmware 5.3.19.50
 * @param    tool       GoExtTool object.
 * @return              The current Ext tool data stream option list count.
 */
GoFx(kSize) GoExtTool_StreamOptionCount(GoExtTool tool);

/**
 * Gets the data stream option at the given index.
 *
 * @public              @memberof GoExtTool
 * @version             Introduced in firmware 5.3.19.50
 * @param    tool       GoExtTool object.
 * @param    index      The index of the option list to access.
 * @return              The Ext tool data stream option at the given index, or k32U_MAX if an invalid index is given.
 */
GoFx(GoDataStream) GoExtTool_StreamOptionAt(GoExtTool tool, kSize index);

/**
 * Returns the measurement count.
 *
 * @public              @memberof GoExtTool
 * @version             Introduced in firmware 4.4.4.14
 * @param    tool       GoExtTool object.
 * @return              The measurement count.
 */
GoFx(kSize) GoExtTool_MeasurementCount(GoExtTool tool);

/**
 * Retrieves the measurement at the given index.
 *
 * @public              @memberof GoExtTool
 * @version             Introduced in firmware 4.4.4.14
 * @param    tool       GoExtTool object.
 * @param    index      The index of the measurement.
 * @return              The measurement at the given index or kNULL if an invalid index is provided.
 */
GoFx(GoMeasurement) GoExtTool_MeasurementAt(GoExtTool tool, kSize index);

/**
* Sets the name of the tool.
*
* @public              @memberof GoExtTool
 * @version            Introduced in firmware 4.4.4.14
* @param    tool       GoExtTool object.
* @param    name       The name to be set for the tool.
* @return              Operation status.
*/
GoFx(kStatus) GoExtTool_SetDisplayName(GoExtTool tool, const kChar* name);

/**
 * Retrieves the display name of the tool.
 *
 * @public              @memberof GoExtTool
 * @version             Introduced in firmware 4.4.4.14
 * @param    tool       GoExtTool object.
 * @return              The tool display name.
 */
GoFx(const kChar*) GoExtTool_DisplayName(GoExtTool tool);

/**
 * Retrieves the type of the tool.
 *
 * @public              @memberof GoExtTool
 * @version             Introduced in firmware 4.4.4.14
 * @param    tool       GoExtTool object.
 * @return              The tool type as a character array.
 */
GoFx(const kChar*) GoExtTool_Type(GoExtTool tool);

/**
 * Sets the data source.
 *
 * @public               @memberof GoExtTool
 * @version              Introduced in firmware 4.4.4.14
 * @param    tool        GoExtTool object.
 * @param    source      GoDataSource object.
 * @return               Operation status.
 * @see                  GoExtTool_IntensitySupportEnabled
 */
GoFx(kStatus) GoExtTool_SetSource(GoExtTool tool, GoDataSource source);

/**
 * Gets the data source.
 *
 * @public               @memberof GoExtTool
 * @version              Introduced in firmware 4.4.4.14
 * @param    tool        GoExtTool object.
 * @return   The data source.
 */
GoFx(GoDataSource) GoExtTool_Source(GoExtTool tool);

/**
 * Gets the data source option list count.
 *
 * @public              @memberof GoExtTool
 * @version             Introduced in firmware 4.4.4.14
 * @param    tool       GoExtTool object.
 * @return              The current tool data source option list count.
 */
GoFx(kSize) GoExtTool_SourceOptionCount(GoExtTool tool);

/**
 * Gets the data source option at the given index.
 *
 * @public              @memberof GoExtTool
 * @version             Introduced in firmware 4.4.4.14
 * @param    tool       GoExtTool object.
 * @param    index      The index of the option list to access.
 * @return              The tool data source option at the given index, or k32U_MAX if an invalid index is given.
 */
GoFx(k32u) GoExtTool_SourceOptionAt(GoExtTool tool, kSize index);

/**
 * Returns a boolean value representing whether the tool supports x anchoring.
 *
 * @public              @memberof GoExtTool
 * @version             Introduced in firmware 4.4.4.14
 * @param    tool       GoExtTool object.
 * @return              kTRUE if supported, kFALSE if not.
 */
GoFx(kBool) GoExtTool_XAnchorSupportEnabled(GoExtTool tool);

/**
 * Gets the X-anchoring option list count.
 *
 * @public              @memberof GoExtTool
 * @version             Introduced in firmware 4.4.4.14
 *
 * @param    tool       GoExtTool object.
 * @return              The X-anchoring option list count.
 */
GoFx(kSize) GoExtTool_XAnchorOptionCount(GoExtTool tool);

/**
 * Gets the X-anchoring option at the given index.
 *
 * @public              @memberof GoExtTool
 * @version             Introduced in firmware 4.4.4.14
 *
 * @param    tool       GoExtTool object.
 * @param    index      The index of the option list to access.
 * @return              The X-anchoring option at the given index or k32U_MAX if invalid.
 */
GoFx(k32u) GoExtTool_XAnchorOptionAt(GoExtTool tool, kSize index);

/**
 * Gets the current X-anchoring source.
 *
 * @public              @memberof GoExtTool
 * @version             Introduced in firmware 4.4.4.14
 *
 * @param    tool       GoExtTool object.
 * @return              The X-anchoring source or -1 if no source is currently set.
 */
GoFx(k32s) GoExtTool_XAnchor(GoExtTool tool);

/**
 * Sets the X-anchoring source.
 *
 * @public              @memberof GoExtTool
 * @version             Introduced in firmware 4.4.4.14
 *
 * @param    tool       GoExtTool object.
 * @param    id         The measurement ID of a valid X-anchoring source.
 * @return              Operation status.
 * @see                 GoExtTool_AnchorSupportEnabled
 */
GoFx(kStatus) GoExtTool_SetXAnchor(GoExtTool tool, k32s id);

/**
 * Returns a boolean value representing whether or not a valid X-anchoring source has been set for X-anchoring.
 *
 * @public              @memberof GoExtTool
 * @version             Introduced in firmware 4.4.4.14
 *
 * @param    tool       GoExtTool object.
 * @return              kTRUE if a valid anchoring source is currently set and kFALSE otherwise.
 * @see                 GoExtTool_AnchorSupportEnabled
 */
GoFx(kBool) GoExtTool_XAnchorEnabled(GoExtTool tool);

/**
* Returns a boolean value representing whether the tool supports y anchoring.
*
* @public              @memberof GoExtTool
* @note                Supported with G3
* @version             Introduced in firmware 4.6.3.95
* @param    tool       GoExtTool object.
* @return              kTRUE if supported, kFALSE if not.
*/
GoFx(kBool) GoExtTool_YAnchorSupportEnabled(GoExtTool tool);

/**
 * Gets the Y-anchoring option list count.
 *
 * @public              @memberof GoExtTool
 * @note                Supported with G3
 * @version             Introduced in firmware 4.4.4.14
 *
 * @param    tool       GoExtTool object.
 * @return              The Y-anchoring option list count.
 */
GoFx(kSize) GoExtTool_YAnchorOptionCount(GoExtTool tool);

/**
 * Gets the Y-anchoring option at the given index.
 *
 * @public              @memberof GoExtTool
 * @note                Supported with G3
 * @version             Introduced in firmware 4.4.4.14
 *
 * @param    tool       GoExtTool object.
 * @param    index      The index of the option list to access.
 * @return              The Y-anchoring option at the given index or k32U_MAX if invalid.
 */
GoFx(k32u) GoExtTool_YAnchorOptionAt(GoExtTool tool, kSize index);

/**
 * Gets the current Y-anchoring source.
 *
 * @public              @memberof GoExtTool
 * @note                Supported with G3
 * @version             Introduced in firmware 4.4.4.14
 *
 * @param    tool       GoExtTool object.
 * @return              The Y-anchoring source or -1 if no source is currently set.
 */
GoFx(k32s) GoExtTool_YAnchor(GoExtTool tool);

/**
 * Sets the Y-anchoring source.
 *
 * @public              @memberof GoExtTool
 * @note                Supported with G3
 * @version             Introduced in firmware 4.4.4.14
 * @param    tool       GoExtTool object.
 * @param    id         The measurement ID of a valid Y-anchoring source.
 * @return              Operation status.
 * @see                 GoExtTool_AnchorSupportEnabled, GoExtTool_Type
 */
GoFx(kStatus) GoExtTool_SetYAnchor(GoExtTool tool, k32s id);

/**
 * Returns a boolean value representing whether or not a valid Y-anchoring source has been set for Y-anchoring.
 *
 * @public              @memberof GoExtTool
 * @note                Supported with G3
 * @version             Introduced in firmware 4.4.4.14
 * @param    tool       GoExtTool object.
 * @return              kTRUE if a valid anchoring source is currently set and kFALSE otherwise.
 * @see                 GoExtTool_AnchorSupportEnabled, GoExtTool_Type
 */
GoFx(kBool) GoExtTool_YAnchorEnabled(GoExtTool tool);

/**
* Returns a boolean value representing whether the tool supports z anchoring.
*
* @public              @memberof GoExtTool
* @note                Supported with G2, G3
* @version             Introduced in firmware 4.6.3.95
* @param    tool       GoExtTool object.
* @return              kTRUE if supported, kFALSE if not.
*/
GoFx(kBool) GoExtTool_ZAnchorSupportEnabled(GoExtTool tool);

/**
 * Gets the Z-anchoring option list count.
 *
 * @public              @memberof GoExtTool
 * @note                Supported with G2, G3
 * @version             Introduced in firmware 4.4.4.14
 * @param    tool       GoExtTool object.
 * @return              The X-anchoring option list count.
 */
GoFx(kSize) GoExtTool_ZAnchorOptionCount(GoExtTool tool);

/**
 * Gets the Z-anchoring option at the given index.
 *
 * @public              @memberof GoExtTool
 * @note                Supported with G2, G3
 * @version             Introduced in firmware 4.4.4.14
 * @param    tool       GoExtTool object.
 * @param    index      The index of the option list to access.
 * @return              The Z-anchoring option at the given index or k32U_MAX if invalid.
 */
GoFx(k32u) GoExtTool_ZAnchorOptionAt(GoExtTool tool, kSize index);

/**
 * Gets the current Z-anchoring source.
 *
 * @public              @memberof GoExtTool
 * @note                Supported with G2, G3
 * @version             Introduced in firmware 4.4.4.14
 * @param    tool       GoExtTool object.
 * @return              The Z-anchoring source or -1 if no source is currently set.
 */
GoFx(k32s) GoExtTool_ZAnchor(GoExtTool tool);

/**
 * Sets the Z-anchoring source.
 *
 * @public              @memberof GoExtTool
 * @note                Supported with G2, G3
 * @version             Introduced in firmware 4.4.4.14
 * @param    tool       GoExtTool object.
 * @param    id         The measurement ID of a valid Z-anchoring source.
 * @return              Operation status.
 * @see                 GoExtTool_AnchorSupportEnabled, GoExtTool_Type
 */
GoFx(kStatus) GoExtTool_SetZAnchor(GoExtTool tool, k32s id);

/**
 * Returns a boolean value representing whether or not a valid Z-anchoring source has been set for Z-anchoring.
 *
 * @public              @memberof GoExtTool
 * @note                Supported with G2, G3
 * @version             Introduced in firmware 4.4.4.14
 * @param    tool       GoExtTool object.
 * @return              kTRUE if a valid anchoring source is currently set and kFALSE otherwise.
 * @see                 GoExtTool_AnchorSupportEnabled, GoExtTool_Type
 */
GoFx(kBool) GoExtTool_ZAnchorEnabled(GoExtTool tool);

/**
* Returns a boolean value representing whether the tool supports z angle anchoring.
*
* @public              @memberof GoExtTool
* @note                Supported with G2, G3
* @version             Introduced in firmware 4.6.3.95
* @param    tool       GoExtTool object.
* @return              kTRUE if supported, kFALSE if not.
*/
GoFx(kBool) GoExtTool_ZAngleAnchorSupportEnabled(GoExtTool tool);

/**
* Gets the ZAngle-anchoring option list count.
*
* @public              @memberof GoExtTool
* @note                Supported with G2, G3
* @version             Introduced in firmware 4.6.2.133
* @param    tool       GoExtTool object.
* @return              The ZAngle-anchoring option list count.
*/
GoFx(kSize) GoExtTool_ZAngleAnchorOptionCount(GoExtTool tool);

/**
* Gets the ZAngle-anchoring option at the given index.
*
* @public              @memberof GoExtTool
* @note                Supported with G2, G3
* @version             Introduced in firmware 4.6.2.133
* @param    tool       GoExtTool object.
* @param    index      The index of the option list to access.
* @return              The ZAngle-anchoring option at the given index or k32U_MAX if invalid.
*/
GoFx(k32u) GoExtTool_ZAngleAnchorOptionAt(GoExtTool tool, kSize index);

/**
* Gets the current Z-anchoring source.
*
* @public              @memberof GoExtTool
* @note                Supported with G2, G3
* @version             Introduced in firmware 4.6.2.133
* @param    tool       GoExtTool object.
* @return              The ZAngle-anchoring source or -1 if no source is currently set.
*/
GoFx(k32s) GoExtTool_ZAngleAnchor(GoExtTool tool);

/**
* Sets the Z-anchoring source.
*
* @public              @memberof GoExtTool
* @note                Supported with G2, G3
* @version             Introduced in firmware 4.6.2.133
* @param    tool       GoExtTool object.
* @param    id         The measurement ID of a valid Z-anchoring source.
* @return              Operation status.
* @see                 GoExtTool_AngleAnchorSupportEnabled, GoExtTool_Type
*/
GoFx(kStatus) GoExtTool_SetZAngleAnchor(GoExtTool tool, k32s id);

/**
* Returns a boolean value representing whether or not a valid Z-anchoring source has been set for Z-anchoring.
*
* @public              @memberof GoExtTool
* @note                Supported with G2, G3
* @version             Introduced in firmware 4.6.2.133
* @param    tool       GoExtTool object.
* @return              kTRUE if a valid anchoring source is currently set and kFALSE otherwise.
* @see                 GoExtTool_AngleAnchorSupportEnabled, GoExtTool_Type
*/
GoFx(kBool) GoExtTool_ZAngleAnchorEnabled(GoExtTool tool);

/**
 * Retrieves the first found instance of a measurement for a given enumeration type.
 *
 * @public              @memberof GoExtTool
 * @version             Introduced in firmware 4.4.4.14
 * @param    tool       GoExtTool object.
 * @param    name       The name of the measurement to retrieve.
 * @return              A measurement object if one is found, otherwise kNULL.
 */
GoFx(GoMeasurement) GoExtTool_FindMeasurementByName(GoExtTool tool, const kChar* name);

/**
 * Returns the number of parameters available for the given tool.
 *
 * @public              @memberof GoExtTool
 * @version             Introduced in firmware 4.4.4.14
 * @param    tool       GoExtTool object.
 * @return              Parameter count.
 */
GoFx(kSize) GoExtTool_ParameterCount(GoExtTool tool);

/**
 * Returns the parameter at the given index.
 *
 * @public              @memberof GoExtTool
 * @version             Introduced in firmware 4.4.4.14
 * @param    tool       GoExtTool object.
 * @param    index      The index of the parameter to retrieve.
 * @return              The custom tool parameter object.
 */
GoFx(GoExtParam) GoExtTool_ParameterAt(GoExtTool tool, kSize index);

/**
 * Returns the parameter which matches the given label.
 *
 * @public              @memberof GoExtTool
 * @version             Introduced in firmware 4.4.4.14
 * @param    tool       GoExtTool object.
 * @param    label      The label of the parameter to retrieve.
 * @return              The parameter matching the label or kNULL if no match is found.
 * @remark              The search is case sensitive.
 */
GoFx(GoExtParam) GoExtTool_FindParameterById(GoExtTool tool, const kChar* label);

/**
* Returns the number of tool data outputs available for the given tool.
*
* @public              @memberof GoExtTool
* @version             Introduced in firmware 4.7.3.97
* @param    tool       GoExtTool object.
* @return              Tool data output count.
*/
GoFx(kSize) GoExtTool_ToolDataOutputCount(GoExtTool tool);

/**
* Returns the tool data output item at the given index.
*
* @public              @memberof GoExtTool
* @version             Introduced in firmware 4.7.3.97
* @param    tool       GoExtTool object.
* @param    index      The index of the tool data output item to retrieve.
* @return              The custom tool data output object or kNULL if no such entry.
*/
GoFx(GoExtToolDataOutput) GoExtTool_ToolDataOutputAt(GoExtTool tool, kSize index);

/**
* Compares the tool with a specific tool type.
*
* @public              @memberof GoExtTool
* @version             Introduced in firmware 5.2.13.12
* @param    tool       GoExtTool object.
* @param    toolType   tool type to compare the tool object to.
* @return              kTrue if tool object eaquals to toolType, otherwise kFalse.
*/
GoFx(kBool) GoExtTool_Equals(GoExtTool tool, kString toolType);

#include <GoSdk/Tools/GoExtTool.x.h>

#endif

/** 
 * @file    GoRangeTools.h
 * @brief   Declares all range tools and their related classes.
 *
 * @internal
 * Copyright (C) 2016-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef GO_RANGE_TOOLS_H
#define GO_RANGE_TOOLS_H

#include <GoSdk/GoSdkDef.h>
#include <GoSdk/GoUtils.h>
#include <GoSdk/Tools/GoTool.h>

/**
 * @class   GoRangeTool
 * @extends GoTool
 * @note    Supported with G1
 * @ingroup GoSdk-RangeTools
 * @brief   Represents a base range tool.
 */
typedef GoTool GoRangeTool; 

/**
 * Sets the data stream. Note that stream validation will only occur if tool
 * is in the tool options list.
 *
 * @public              @memberof GoRangeTool
 * @note                Supported with G1
 * @version             Introduced in firmware 5.3.19.50
 * @param    tool       GoRangeTool object.
 * @param    stream     GoDataStream value.
 * @return              Operation status.
 * @see                 GoRangeTool_StreamOptionCount, GoRangeTool_StreamOptionAt
 */
GoFx(kStatus) GoRangeTool_SetStream(GoRangeTool tool, GoDataStream stream);

/**
 * Gets the data stream.
 *
 * @public              @memberof GoRangeTool
 * @note                Supported with G1
 * @version             Introduced in firmware 5.3.19.50
 * @param    tool       GoRangeTool object.
 * @return              The current Range tool data stream value.
 */
GoFx(GoDataStream) GoRangeTool_Stream(GoRangeTool tool);

/**
 * Gets the data stream option list count.
 *
 * @public              @memberof GoRangeTool
 * @note                Supported with G1
 * @version             Introduced in firmware 5.3.19.50
 * @param    tool       GoRangeTool object.
 * @return              The current Range tool data stream option list count.
 */
GoFx(kSize) GoRangeTool_StreamOptionCount(GoRangeTool tool);

/**
 * Gets the data stream option at the given index.
 *
 * @public              @memberof GoRangeTool
 * @note                Supported with G1
 * @version             Introduced in firmware 5.3.19.50
 * @param    tool       GoRangeTool object.
 * @param    index      The index of the option list to access.
 * @return              The Range tool data stream option at the given index, or k32U_MAX if an invalid index is given.
 */
GoFx(GoDataStream) GoRangeTool_StreamOptionAt(GoRangeTool tool, kSize index);

/** 
 * Sets the data source. Note that source validation will only occur if tool
 * is in the tool options list.
 *
 * @public              @memberof GoRangeTool
 * @note                Supported with G1
 * @version             Introduced in firmware 4.0.10.27
 * @param    tool       GoRangeTool object.
 * @param    source     GoDataSource object.
 * @return              Operation status.            
 * @see                 GoTools_ToolOptionCount, GoTools_ToolOptionAt
 */
GoFx(kStatus) GoRangeTool_SetSource(GoRangeTool tool, GoDataSource source);

/** 
 * Gets the data source.
 *
 * @public              @memberof GoRangeTool
 * @note                Supported with G1
 * @version             Introduced in firmware 4.0.10.27
 * @param    tool       GoRangeTool object.
 * @return              The data source.            
 */
GoFx(GoDataSource) GoRangeTool_Source(GoRangeTool tool);

/** 
 * Gets the data source option list count.
 *
 * @public             @memberof GoRangeTool
 * @note               Supported with G1
 * @version             Introduced in firmware 4.0.10.27
 * @param    tool      GoRangeTool object.
 * @return             The data source option list count.            
 */
GoFx(kSize) GoRangeTool_SourceOptionCount(GoRangeTool tool);

/** 
 * Gets the data source option at the given index.
 *
 * @public             @memberof GoRangeTool
 * @note               Supported with G1
 * @version            Introduced in firmware 4.0.10.27
 * @param    tool      GoRangeTool object.
 * @param    index     The index with which to retrieve a data source option.
 * @return             The data source option at the given index or k32U_MAX if an invalid index is given.            
 */
GoFx(GoDataSource) GoRangeTool_SourceOptionAt(GoRangeTool tool, kSize index);

/**
 * @class   GoRangePosition
 * @extends GoRangeTool
 * @note    Supported with G1
 * @ingroup GoSdk-RangeTools
 * @brief   Represents a range position tool.
 */
typedef GoRangeTool GoRangePosition; 

/**
 * Returns a range position tool Z measurement object.
 *
 * @public              @memberof GoRangePosition
 * @note                Supported with G1
 * @version             Introduced in firmware 4.0.10.27
 * @param    tool       GoRangePosition object.
 * @return              A GoRangePositionZ measurement.
 */
GoFx(GoRangePositionZ) GoRangePosition_ZMeasurement(GoRangePosition tool);

/**
 * @class   GoRangeThickness
 * @extends GoRangeTool
 * @note    Supported with G1
 * @ingroup GoSdk-RangeTools
 * @brief   Represents a range thickness tool.
 */
typedef GoRangeTool GoRangeThickness; 

/**
 * Returns a boolean value representing whether or not the measurement is set to return an absolute value.
 *
 * @public              @memberof GoRangePosition
 * @note                Supported with G1
 * @version             Introduced in firmware 4.0.10.27
 * @param    tool       GoRangePosition object.
 * @return              kTRUE if enabled and kFALSE otherwise.
 */
GoFx(kBool) GoRangeThickness_AbsoluteEnabled(GoRangeThickness tool);

/**
 * Enable or disable absolute value measurement output.
 *
 * @public              @memberof GoRangePosition
 * @note                Supported with G1
 * @version             Introduced in firmware 4.0.10.27
 * @param    tool       GoRangePosition object.
 * @param    enable     kTRUE to enable absolute value and kFALSE to disable it.
 * @return              Operation status.
 */
GoFx(kStatus) GoRangeThickness_EnableAbsolute(GoRangeThickness tool, kBool enable);

/**
 * Returns a range thickness tool Thickness measurement object.
 *
 * @public               @memberof GoRangeThickness
 * @note                 Supported with G1
 * @version              Introduced in firmware 4.0.10.27
 * @param    tool        GoRangeThickness object.
 * @return               A GoRangeThicknessThickness measurement.
 */
GoFx(GoRangeThickness) GoRangeThickness_ThicknessMeasurement(GoRangeThickness tool);

#include <GoSdk/Tools/GoRangeTools.x.h>

#endif

/**
 * @file    GoExtToolDataOutput.h
 * @brief   Declares the GoExtToolDataOutput class.
 *
 * @internal
 * Copyright (C) 2017-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef GO_EXT_TOOL_DATA_OUTPUT_H
#define GO_EXT_TOOL_DATA_OUTPUT_H

#include <GoSdk/GoSdkDef.h>

/**
 * @class   GoExtToolDataOutput
 * @extends kObject
 * @ingroup GoSdk
 * @brief   Represents the base class for a tool data output.
 */
typedef kObject GoExtToolDataOutput;


/**
 * Sets an ID number for the given toolDataOutput.
 *
 * @public                  @memberof GoExtToolDataOutput
 * @version                 Introduced in firmware 4.7.3.97
 * @param    toolDataOutput GoExtToolDataOutput object.
 * @param    id             The ID value to set for the toolDataOutput.
 * @return                  Operation status.
 */
GoFx(kStatus) GoExtToolDataOutput_SetId(GoExtToolDataOutput toolDataOutput, k32s id);

/**
 * Gets the ID for the given toolDataOutput.
 *
 * @public                  @memberof GoExtToolDataOutput
 * @version                 Introduced in firmware 4.7.3.97
 * @param    toolDataOutput GoExtToolDataOutput object.
 * @return                  The ID value if there is one assigned.  Otherwise, -1 is returned.
 */
GoFx(k32s) GoExtToolDataOutput_Id(GoExtToolDataOutput toolDataOutput);

/**
* Sets the name for the given toolDataOutput.
*
* @public                  @memberof GoExtToolDataOutput
* @version                 Introduced in firmware 4.7.3.97
* @param    toolDataOutput GoExtToolDataOutput object.
* @param    name           The name to assign to the toolDataOutput.
* @return                  Operation status.
*/
GoFx(kStatus) GoExtToolDataOutput_SetName(GoExtToolDataOutput toolDataOutput, const kChar* name);

/**
 * Gets the name for the given toolDataOutput.
 *
 * @public                  @memberof GoExtToolDataOutput
 * @version                 Introduced in firmware 4.7.3.97
 * @param    toolDataOutput GoExtToolDataOutput object.
 * @return                  A character array pointer for the toolDataOutput name.
 */
GoFx(const kChar*) GoExtToolDataOutput_Name(GoExtToolDataOutput toolDataOutput);

/**
 * Enables the given toolDataOutput for output.
 *
 * @public                  @memberof GoExtToolDataOutput
 * @version                 Introduced in firmware 4.7.3.97
 * @param    toolDataOutput GoExtToolDataOutput object.
 * @param    enable         Set to kTRUE to enable the toolDataOutput, kFALSE to disable it.
 * @return                  Operation status.
 */
GoFx(kStatus) GoExtToolDataOutput_SetEnable(GoExtToolDataOutput toolDataOutput, kBool enable);

/**
 * Returns a boolean value representing whether the given toolDataOutput is enabled.
 *
 * @public                  @memberof GoExtToolDataOutput
 * @version                 Introduced in firmware 4.7.3.97
 * @param    toolDataOutput GoExtToolDataOutput object.
 * @return                  kTRUE if enabled; kFALSE otherwise.
 */
GoFx(kBool) GoExtToolDataOutput_Enabled(GoExtToolDataOutput toolDataOutput);

/**
 * Gets the data type for the given toolDataOutput.
 *
 * @public                  @memberof GoExtToolDataOutput
 * @version                 Introduced in firmware 4.7.3.97
 * @param    toolDataOutput GoExtToolDataOutput object.
 * @return                  The data type value.
 */
GoFx(GoDataType) GoExtToolDataOutput_DataType(GoExtToolDataOutput toolDataOutput);

/**
 * Gets the toolDataOutput type name for the given toolDataOutput.
 *
 * @public                  @memberof GoExtToolDataOutput
 * @version                 Introduced in firmware 4.7.3.97
 * @param    toolDataOutput GoExtToolDataOutput object.
 * @return                  The toolDataOutput type name.
 */
GoFx(const kChar*) GoExtToolDataOutput_Type(GoExtToolDataOutput toolDataOutput);

#include <GoSdk/Tools/GoExtToolDataOutput.x.h>

#endif // GO_EXT_TOOL_DATA_OUTPUT_H

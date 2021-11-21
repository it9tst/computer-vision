/** 
 * @file    GoSection.h
 * @brief   Declares the GoSection class. 
 *
 * @internal
 * Copyright (C) 2016-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef GO_SDK_SECTION_H
#define GO_SDK_SECTION_H

#include <GoSdk/GoSdkDef.h>

/**
 * @class   GoSection
 * @extends kObject
 * @ingroup GoSdk
 * @brief   Represents sections to be applied on surface data.
 */
typedef kObject GoSection; 

/** 
 * Gets the ID of the section.
 *
 * @public              @memberof GoSection
 * @version             Introduced in firmware 4.4.4.14
 * @param   section     GoSection object.
 * @return              The section ID.
 */
GoFx(k16s) GoSection_Id(GoSection section);

/** 
 * Sets the name of the section.
 *
 * @public              @memberof GoSection
 * @version             Introduced in firmware 4.4.4.14
 * @param   section     GoSection object.
 * @param    name       The name to be set for the section.
 * @return              Operation status.            
 */
GoFx(kStatus) GoSection_SetName(GoSection section, const kChar* name);

/** 
 * Retrieves the name of the section.
 *
 * @public              @memberof GoSection
 * @version             Introduced in firmware 4.4.4.14
 * @param    section    GoSection object.
 * @param    name       Receives the name of the section.
 * @param    capacity   The maximum capacity of the name array.
 * @return              Operation status.            
 */
GoFx(kStatus) GoSection_Name(GoSection section, kChar* name, kSize capacity);

/** 
 * Gets the start point of the section.
 *
 * @public              @memberof GoSection
 * @version             Introduced in firmware 4.4.4.14
 * @param   section     GoSection object.
 * @return              The firmware version.
 */
GoFx(kPoint64f) GoSection_StartPoint(GoSection section);

/** 
 * Sets the start point of the section.
 *
 * @public              @memberof GoSection
 * @version             Introduced in firmware 4.4.4.14
 * @param   section     GoSection object.
 * @param   point       The new start point coordinates to set.
 * @return              Operation status.
 */
GoFx(kStatus) GoSection_SetStartPoint(GoSection section, kPoint64f point);

/** 
 * Gets the end point of the section.
 *
 * @public              @memberof GoSection
 * @version             Introduced in firmware 4.4.4.14
 * @param   section     GoSection object.
 * @return              The firmware version.
 */
GoFx(kPoint64f) GoSection_EndPoint(GoSection section);

/** 
 * Sets the end point of the section.
 *
 * @public              @memberof GoSection
 * @version             Introduced in firmware 4.4.4.14
 * @param   section     GoSection object.
 * @param   point       The new end point coordinates to set.
 * @return              Operation status.
 */
GoFx(kStatus) GoSection_SetEndPoint(GoSection section, kPoint64f point);

/** 
 * Indicates whether the custom spacing interval is enabled.
 *
 * @public              @memberof GoSection
 * @version             Introduced in firmware 4.4.4.14
 * @param   section     GoSection object.
 * @return              kTRUE if enabled and kFALSE if disabled.
 */
GoFx(kBool) GoSection_CustomSpacingIntervalEnabled(GoSection section);

/** 
 * Enables or disables the custom spacing interval.
 *
 * @public              @memberof GoSection
 * @version             Introduced in firmware 4.4.4.14
 * @param   section     GoSection object.
 * @param   enable      kTRUE to enable the custom spacing interval and kFALSE to disable it.
 * @return              Operation status.
 */
GoFx(kStatus) GoSection_EnableCustomSpacingInterval(GoSection section, kBool enable);

/** 
 * Gets the current user defined spacing interval. (mm)
 *
 * @public              @memberof GoSection
 * @version             Introduced in firmware 4.4.4.14
 * @param   section     GoSection object.
 * @return              User defined spacing interval.
 */
GoFx(k64f) GoSection_SpacingInterval(GoSection section);

/** 
 * Gets the current spacing interval minimum limit. (mm)
 *
 * @public              @memberof GoSection
 * @version             Introduced in firmware 4.4.4.14
 * @param   section     GoSection object.
 * @return              Spacing interval minimum limit.
 */
GoFx(k64f) GoSection_SpacingIntervalLimitMin(GoSection section);

/** 
 * Gets the current spacing interval maximum limit. (mm)
 *
 * @public              @memberof GoSection
 * @version             Introduced in firmware 4.4.4.14
 * @param   section     GoSection object.
 * @return              Spacing interval maximum limit.
 */
GoFx(k64f) GoSection_SpacingIntervalLimitMax(GoSection section);

/** 
 * Gets the spacing interval system value. (mm)
 *
 * @public              @memberof GoSection
 * @version             Introduced in firmware 4.4.4.14
 * @param   section     GoSection object.
 * @return              Spacing interval system value.
 */
GoFx(k64f) GoSection_SpacingIntervalSystemValue(GoSection section);

/** 
 * Sets the spacing interval of the section.
 *
 * @public              @memberof GoSection
 * @version             Introduced in firmware 4.4.4.14
 * @param   section     GoSection object.
 * @param   value       The spacing interval to set.
 * @return              Operation status.
 */
GoFx(kStatus) GoSection_SetSpacingInterval(GoSection section, k64f value);

#include <GoSdk/GoSection.x.h>

#endif

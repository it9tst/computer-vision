/** 
 * @file    GoLayout.h
 * @brief   Declares the GoLayout class. 
 *
 * @internal
 * Copyright (C) 2016-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef GO_LAYOUT_H
#define GO_LAYOUT_H

#include <GoSdk/GoSdkDef.h>

/**
 * @class   GoLayout
 * @extends kObject
 * @ingroup GoSdk
 * @brief   Represents a layout related sensor configuration.
 */
typedef kObject GoLayout; 

/** 
 * Sets the buddied sensor configuration orientation.
 *
 * @public                  @memberof GoLayout
 * @version                Introduced in firmware 4.0.10.27
 * @param   layout          GoLayout object.
 * @param   orientation     The orientation to set.
 * @return                  Operation status.            
 */
GoFx(kStatus) GoLayout_SetOrientation(GoLayout layout, GoOrientation orientation);

/** 
 * Returns the current orientation used when buddied.
 *
 * @public             @memberof GoLayout
 * @version            Introduced in firmware 4.0.10.27
 * @param   layout     GoLayout object.
 * @return             Buddied device orientation.
 */
GoFx(GoOrientation) GoLayout_Orientation(GoLayout layout);

/** 
 * Returns the transformed data region X-component value.
 *
 * @public             @memberof GoLayout
 * @version            Introduced in firmware 4.0.10.27
 * @param   layout     GoLayout object.
 * @return             Transformed data region value.
 */
GoFx(k64f) GoLayout_TransformedDataRegionX(GoLayout layout);

/** 
 * Returns the transformed data region Y-component value.
 *
 * @public             @memberof GoLayout
 * @version            Introduced in firmware 4.0.10.27
 * @param   layout     GoLayout object.
 * @return             Transformed data region value.
 */
GoFx(k64f) GoLayout_TransformedDataRegionY(GoLayout layout);

/** 
 * Returns the transformed data region Z-component value.
 *
 * @public             @memberof GoLayout
 * @version            Introduced in firmware 4.0.10.27
 * @param   layout     GoLayout object.
 * @return             Transformed data region value.
 */
GoFx(k64f) GoLayout_TransformedDataRegionZ(GoLayout layout);

/** 
 * Returns the transformed data region width value. (mm)
 *
 * @public             @memberof GoLayout
 * @version            Introduced in firmware 4.0.10.27
 * @param   layout     GoLayout object.
 * @return             Transformed data region value.
 */
GoFx(k64f) GoLayout_TransformedDataRegionWidth(GoLayout layout);

/** 
 * Returns the transformed data region length value. (mm)
 *
 * @public             @memberof GoLayout
 * @version            Introduced in firmware 4.0.10.27
 * @param   layout     GoLayout object.
 * @return             Transformed data region value.
 */
GoFx(k64f) GoLayout_TransformedDataRegionLength(GoLayout layout);

/** 
 * Returns the transformed data region height value. (mm)
 *
 * @public             @memberof GoLayout
 * @version            Introduced in firmware 4.0.10.27
 * @param   layout     GoLayout object.
 * @return             Transformed data region value.
 */
GoFx(k64f) GoLayout_TransformedDataRegionHeight(GoLayout layout);

/** 
 * Returns the layout specific X spacing count value.
 * This is the number of resampled profile points in the layout 
 * for the configured X-axis range and X-axis increment/step within the range.
 *
 * @public             @memberof GoLayout
 * @version            Introduced in firmware 4.1.3.106
 * @param   layout     GoLayout object.
 * @return             X spacing count value.
 * @see                GoSetup_XSpacingCount
 */
GoFx(k64f) GoLayout_XSpacingCount(GoLayout layout);

/** 
 * Returns the layout specific Y spacing count value.
 *
 * @public             @memberof GoLayout
 * @version            Introduced in firmware 4.1.3.106
 * @param   layout     GoLayout object.
 * @return             Y spacing count value.
 * @see                GoSetup_YSpacingCount
 */
GoFx(k64f) GoLayout_YSpacingCount(GoLayout layout);


/** 
 * Returns a boolean value representing whether or not multiplexing is 
 * enabled in a buddied configuration.
 *
 * @public             @memberof GoLayout
 * @note               Supported with G1, G2
 * @version            Introduced in firmware 4.0.10.27
 * @param   layout     GoLayout object.
 * @return             kTRUE if multiplexing is enabled for a buddied configuration. kFALSE otherwise.
 */
GoFx(kBool) GoLayout_MultiplexBuddyEnabled(GoLayout layout);

/** 
 * Enables buddied sensor multiplexing with automated parameter calculation.
 *
 * @public                  @memberof GoLayout
 * @note                    Supported with G1, G2
 * @version                 Introduced in firmware 4.0.10.27
 * @param   layout          GoLayout object.
 * @param   enable          kTRUE to enable, or kFALSE to disable.
 * @return                  Operation status.            
 */
GoFx(kStatus) GoLayout_EnableMultiplexBuddy(GoLayout layout, kBool enable);

/** 
 * Returns a boolean value representing whether or not multiplexing is 
 * enabled in a single device configuration.
 *
 * @public             @memberof GoLayout
 * @note               Supported with G1, G2
 * @version            Introduced in firmware 4.0.10.27
 * @param   layout     GoLayout object.
 * @return             kTRUE if multiplexing is enabled. kFALSE otherwise.
 */
GoFx(kBool) GoLayout_MultiplexSingleEnabled(GoLayout layout);

/** 
 * Enables single sensor(not buddied) configuration multiplexing.
 *
 * @public                  @memberof GoLayout
 * @note                    Supported with G1, G2
 * @version                 Introduced in firmware 4.0.10.27
 * @param   layout          GoLayout object.
 * @param   enabled         kTRUE to enable, or kFALSE to disable.
 * @return                  Operation status.            
 */
GoFx(kStatus) GoLayout_EnableMultiplexSingle(GoLayout layout, kBool enabled);

/** 
 * Returns a value representing the multiplexing delay in a single device sensor configuration. (microseconds)
 *
 * @public             @memberof GoLayout
 * @note               Supported with G1, G2
 * @version            Introduced in firmware 4.0.10.27
 * @param   layout     GoLayout object.
 * @return             Single device multiplexing delay.
 */
GoFx(k64f) GoLayout_MultiplexSingleDelay(GoLayout layout);

/** 
 * Sets the single sensor multiplexing delay. (microseconds)
 *
 * @public                  @memberof GoLayout
 * @note                    Supported with G1, G2
 * @version                 Introduced in firmware 4.0.10.27
 * @param   layout          GoLayout object.
 * @param   value           The delay (in uS) to set.
 * @return                  Operation status.            
 */
GoFx(kStatus) GoLayout_SetMultiplexSingleDelay(GoLayout layout, k64f value);

/** 
 * Returns a value representing the multiplexing period in a single device sensor configuration. (microseconds)
 *
 * @public             @memberof GoLayout
 * @note               Supported with G1, G2
 * @version            Introduced in firmware 4.0.10.27
 * @param   layout     GoLayout object.
 * @return             Single device multiplexing period.
 */
GoFx(k64f) GoLayout_MultiplexSinglePeriod(GoLayout layout); 

/**
* Returns a value representing the minimum multiplexing period in a single device sensor configuration. (microseconds)
*
* @public             @memberof GoLayout
* @note               Supported with G1, G2
* @version            Introduced in firmware 4.0.10.27
* @param   layout     GoLayout object.
* @return             Single device multiplexing period minimum value.
*/
GoFx(k64f) GoLayout_MultiplexSinglePeriodMin(GoLayout layout);

/** 
 * Sets the single sensor multiplexing period. (microseconds)
 *
 * @public                  @memberof GoLayout
 * @note                    Supported with G1, G2
 * @version                 Introduced in firmware 4.0.10.27
 * @param   layout          GoLayout object.
 * @param   value           The multiplexing period (in uS) to set.
 * @return                  Operation status.            
 */
GoFx(kStatus) GoLayout_SetMultiplexSinglePeriod(GoLayout layout, k64f value);

/** 
 * Returns a value representing the multiplexing exposure duration in a single device sensor configuration. (microseconds)
 *
 * @public             @memberof GoLayout
 * @note               Supported with G1, G2
 * @version            Introduced in firmware 4.0.10.27
 * @param   layout     GoLayout object.
 * @return             Single device multiplexing exposure duration.
 */
GoFx(k64f) GoLayout_MultiplexSingleExposureDuration(GoLayout layout);

/** 
 * Sets number of columns being configured (may not be applicable or used) on device in n-buddy system
 *
 * @public                  @memberof GoLayout
 * @version                 Introduced in firmware 6.0.18.7
 * @note                    Supported with G1, G2
 * @param   layout          GoLayout object.
 * @param   value           The number of column
 * @return                  Operation status.            
 */
GoFx(kStatus) GoLayout_SetGridColumnCount(GoLayout layout, k32u value);

/** 
 * Returns number of columns being configured (may not be applicable or used) on device in n-buddy system.
 *
 * @public             @memberof GoLayout
 * @version            Introduced in firmware 6.0.18.7
 * @note               Supported with G1, G2
 * @param   layout     GoLayout object.
 * @return             The number of column
 */
GoFx(k32u) GoLayout_GridColumnCount(GoLayout layout);

/** 
 * Returns number of columns system value being used on device in n-buddy system
 *
 * @public             @memberof GoLayout
 * @version            Introduced in firmware 6.0.18.7
 * @note               Supported with G1, G2
 * @param   layout     GoLayout object.
 * @return             The number of column system value
 */
GoFx(k32u) GoLayout_GridColumnCountSystemValue(GoLayout layout);

#include <GoSdk/GoLayout.x.h>

#endif

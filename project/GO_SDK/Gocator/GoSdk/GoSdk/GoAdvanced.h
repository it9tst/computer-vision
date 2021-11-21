/** 
 * @file    GoAdvanced.h
 * @brief   Declares the GoAdvanced class. 
 *
 * @internal
 * Copyright (C) 2016-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef GO_ADVANCED_H
#define GO_ADVANCED_H

#include <GoSdk/GoSdkDef.h>

/**
 * @class   GoAdvanced
 * @extends kObject
 * @ingroup GoSdk
 * @brief   Represents configurable advanced acquisition settings.
 */
typedef kObject GoAdvanced; 

/** 
 * Sets the advanced acquisition type.
 *
 * @public             @memberof GoAdvanced
 * @version            Introduced in firmware 4.6.1.140
 * @param   advanced   GoAdvanced object.
 * @param   type       The advanced type to set.
 * @return             Operation status.
 */
GoFx(kStatus) GoAdvanced_SetType(GoAdvanced advanced, GoAdvancedType type);

/** 
 * Returns the user defined advanced acquisition type.
 *
 * @public             @memberof GoAdvanced
 * @version            Introduced in firmware 4.6.1.140
 * @param   advanced   GoAdvanced object.
 * @return             The advanced type.
 */
GoFx(GoAdvancedType) GoAdvanced_Type(GoAdvanced advanced);

/** 
 * Returns a boolean relating to whether the user defined advanced acquisition type value will be used by the system.
 *
 * @public             @memberof GoAdvanced
 * @version            Introduced in firmware 4.6.1.140
 * @param   advanced   GoAdvanced object.
 * @return             kTRUE if the user defined advanced type will be used and kFALSE otherwise.
 */
GoFx(kBool) GoAdvanced_IsTypeUsed(GoAdvanced advanced);

/** 
 * Returns the advanced acquisition type to be used by the system.
 *
 * @public             @memberof GoAdvanced
 * @version            Introduced in firmware 4.6.1.140
 * @param   advanced   GoAdvanced object.
 * @return             The system value advanced type.
 */
GoFx(GoAdvancedType) GoAdvanced_TypeSystemValue(GoAdvanced advanced);

/** 
 * Sets the spot threshold.
 *
 * @public             @memberof GoAdvanced
 * @note               Supported with G1, G2
 * @version            Introduced in firmware 4.6.1.140
 * @param   advanced   GoAdvanced object.
 * @param   value      Spot threshold.
 * @return             Operation status.
 */
GoFx(kStatus) GoAdvanced_SetSpotThreshold(GoAdvanced advanced, k32u value);

/** 
 * Returns the user defined spot threshold.
 *
 * @public             @memberof GoAdvanced
 * @note               Supported with G1, G2
 * @version            Introduced in firmware 4.6.1.140
 * @param   advanced   GoAdvanced object.
 * @return             The spot threshold.
 */
GoFx(k32u) GoAdvanced_SpotThreshold(GoAdvanced advanced);

/** 
 * Returns the minimum spot threshold limit.
 *
 * @public             @memberof GoAdvanced
 * @note               Supported with G1, G2
 * @version            Introduced in firmware 4.6.1.140
 * @param   advanced   GoAdvanced object.
 * @return             The minimum spot threshold.
 */
GoFx(k32u) GoAdvanced_SpotThresholdLimitMin(GoAdvanced advanced);

/** 
 * Returns the maximum spot threshold limit.
 *
 * @public             @memberof GoAdvanced
 * @note               Supported with G1, G2
 * @version            Introduced in firmware 4.6.1.140
 * @param   advanced   GoAdvanced object.
 * @return             The maximum spot threshold.
 */
GoFx(k32u) GoAdvanced_SpotThresholdLimitMax(GoAdvanced advanced);

/** 
 * Returns a boolean value representing whether the user specified spot threshold value is used.
 *
 * @public             @memberof GoAdvanced
 * @note               Supported with G1, G2
 * @version            Introduced in firmware 4.6.1.140
 * @param   advanced   GoAdvanced object.
 * @return             kTRUE if it is used and kFALSE otherwise.
 */
GoFx(kBool) GoAdvanced_IsSpotThresholdUsed(GoAdvanced advanced);

/** 
 * Returns the system spot threshold value.
 *
 * @public             @memberof GoAdvanced
 * @note               Supported with G1, G2
 * @version            Introduced in firmware 4.6.1.140
 * @param   advanced   GoAdvanced object.
 * @return             The system spot threshold.
 */
GoFx(k32u) GoAdvanced_SpotThresholdSystemValue(GoAdvanced advanced);

/** 
 * Sets the maximum spot width.
 *
 * @public             @memberof GoAdvanced
 * @version            Introduced in firmware 4.6.1.140
 * @param   advanced   GoAdvanced object.
 * @param   value      Maximum spot width.
 * @return             Operation status.
 */
GoFx(kStatus) GoAdvanced_SetSpotWidthMax(GoAdvanced advanced, k32u value);

/** 
 * Returns the user defined maximum spot width.
 *
 * @public             @memberof GoAdvanced
 * @version            Introduced in firmware 4.6.1.140
 * @param   advanced   GoAdvanced object.
 * @return             The maximum spot width.
 */
GoFx(k32u) GoAdvanced_SpotWidthMax(GoAdvanced advanced);

/** 
 * Returns the maximum spot width minimum limit.
 *
 * @public             @memberof GoAdvanced
 * @version            Introduced in firmware 4.6.1.140
 * @param   advanced   GoAdvanced object.
 * @return             The maximum spot width minimum limit.
 */
GoFx(k32u) GoAdvanced_SpotWidthMaxLimitMin(GoAdvanced advanced);

/** 
 * Returns the maximum spot width maximum limit.
 *
 * @public             @memberof GoAdvanced
 * @version            Introduced in firmware 4.6.1.140
 * @param   advanced   GoAdvanced object.
 * @return             The maximum spot width maximum limit.
 */
GoFx(k32u) GoAdvanced_SpotWidthMaxLimitMax(GoAdvanced advanced);

/** 
 * Returns a boolean relating to whether the user defined spot width max value will be used by the system.
 *
 * @public             @memberof GoAdvanced
 * @version            Introduced in firmware 4.6.1.140
 * @param   advanced   GoAdvanced object.
 * @return             kTRUE if the user value will be used and kFALSE otherwise.
 */
GoFx(kBool) GoAdvanced_IsSpotWidthMaxUsed(GoAdvanced advanced);

/** 
 * Returns the maximum spot width system value.
 *
 * @public             @memberof GoAdvanced
 * @version            Introduced in firmware 4.6.1.140
 * @param   advanced   GoAdvanced object.
 * @return             The maximum spot width system value.
 */
GoFx(k32u) GoAdvanced_SpotWidthMaxSystemValue(GoAdvanced advanced);

/** 
 * Returns the number of spot selection type options.
 *
 * @public             @memberof GoAdvanced
 * @version            Introduced in firmware 4.6.1.140
 * @param   advanced   GoAdvanced object.
 * @return             The spot selection type option count.
 */
GoFx(kSize) GoAdvanced_SpotSelectionTypeOptionCount(GoAdvanced advanced);

/** 
 * Returns the spot selection type option at the given index.
 *
 * @public             @memberof GoAdvanced
 * @version            Introduced in firmware 4.6.1.140
 * @param   advanced   GoAdvanced object.
 * @param   index      The option list index to access.
 * @return             The spot selection type option at the given index.
 */
GoFx(GoSpotSelectionType) GoAdvanced_SpotSelectionTypeOptionAt(GoAdvanced advanced, kSize index);

/** 
 * Sets the spot selection type.
 *
 * @public             @memberof GoAdvanced
 * @version            Introduced in firmware 4.6.1.140
 * @param   advanced   GoAdvanced object.
 * @param   type       Spot selection type.
 * @return             Operation status.
 */
GoFx(kStatus) GoAdvanced_SetSpotSelectionType(GoAdvanced advanced, GoSpotSelectionType type);

/** 
 * Returns the user defined spot selection type.
 *
 * @public             @memberof GoAdvanced
 * @version            Introduced in firmware 4.6.1.140
 * @param   advanced   GoAdvanced object.
 * @return             The maximum spot width.
 */
GoFx(GoSpotSelectionType) GoAdvanced_SpotSelectionType(GoAdvanced advanced);

/** 
 * Returns a boolean relating to whether the user defined spot selection type will be used by the system.
 *
 * @public             @memberof GoAdvanced
 * @version            Introduced in firmware 4.6.1.140
 * @param   advanced   GoAdvanced object.
 * @return             kTRUE if the user value will be used and kFALSE otherwise.
 */
GoFx(kBool) GoAdvanced_IsSpotSelectionTypeUsed(GoAdvanced advanced);

/** 
 * Returns the system spot selection type.
 *
 * @public             @memberof GoAdvanced
 * @version            Introduced in firmware 4.6.1.140
 * @param   advanced   GoAdvanced object.
 * @return             System spot selection type.
 */
GoFx(GoSpotSelectionType) GoAdvanced_SpotSelectionTypeSystemValue(GoAdvanced advanced);

/**
* Returns the minimum segment size used in continuity sorting spot detection.
*
* @public             @memberof GoAdvanced
* @version            Introduced in firmware 4.6.2.79
* @param   advanced   GoAdvanced object.
* @return             Minimum segment size
*/
GoFx(k32u) GoAdvanced_SpotContinuityMinimumSegmentSize(GoAdvanced advanced);

/**
* Sets the minimum segment size used in continuity sorting spot detection.
*
* @public             @memberof GoAdvanced
* @version            Introduced in firmware 4.6.2.79
* @param   advanced   GoAdvanced object.
* @param   value      Minimum Segment Size.
* @return             Operation status.
*/
GoFx(kStatus) GoAdvanced_SetSpotContinuityMinimumSegmentSize(GoAdvanced advanced, k32u value);

/**
* Returns the search window size in the X direction used in continuity sorting spot detection.
*
* @public             @memberof GoAdvanced
* @version            Introduced in firmware 4.6.2.79
* @param   advanced   GoAdvanced object.
* @return             Search window X dimension
*/
GoFx(k32u) GoAdvanced_SpotContinuitySearchWindowX(GoAdvanced advanced);

/**
* Sets the search window size in the X direction used in continuity sorting spot detection.
*
* @public             @memberof GoAdvanced
* @version            Introduced in firmware 4.6.2.79
* @param   advanced   GoAdvanced object.
* @param   value      Search window X dimension.
* @return             Operation status.
*/
GoFx(kStatus) GoAdvanced_SetSpotContinuitySearchWindowX(GoAdvanced advanced, k32u value);

/**
* Returns the search window size in the Y direction used in continuity sorting spot detection
*
* @public             @memberof GoAdvanced
* @version            Introduced in firmware 4.6.2.79
* @param   advanced   GoAdvanced object.
* @return             Search window Y dimension
*/
GoFx(k32u) GoAdvanced_SpotContinuitySearchWindowY(GoAdvanced advanced);

/**
* Sets the search window size in the Y direction used in continuity sorting spot detection
*
* @public             @memberof GoAdvanced
* @version            Introduced in firmware 4.6.2.79
* @param   advanced   GoAdvanced object.
* @param   value      Search window Y dimension.
* @return             Operation status.
*/
GoFx(kStatus) GoAdvanced_SetSpotContinuitySearchWindowY(GoAdvanced advanced, k32u value);

/**
* Returns the spot width threshold below which spots are considered to be in an opaque section of the profile. This is in number of pixels in the video image Y direction.
*
* @public             @memberof GoAdvanced
* @version            Introduced in firmware 6.1.18.16
* @param   advanced   GoAdvanced object.
* @return             The spot width threshold below which spots are considered to be in an opaque section of the profile.
*/
GoFx(k32u) GoAdvanced_SpotTranslucentOpaqueWidth(GoAdvanced advanced);

/**
* Sets the spot width threshold below which spots are considered to be in an opaque section of the profile. This is in number of pixels in the video image Y direction.
*
* @public             @memberof GoAdvanced
* @version            Introduced in firmware 6.1.18.16
* @param   advanced   GoAdvanced object.
* @param   value      The spot width threshold below which spots are considered to be in an opaque section of the profile.
* @return             Operation status.
*/
GoFx(kStatus) GoAdvanced_SetSpotTranslucentOpaqueWidth(GoAdvanced advanced, k32u value);

/** 
 * Returns the minimum limit of the spot width threshold below which spots are considered to be in an opaque section of the profile.
*
 *
 * @public             @memberof GoAdvanced
 * @version            Introduced in firmware 6.1.18.16
 * @param   advanced   GoAdvanced object.
 * @return             The minimum limit of the spot width threshold below which spots are considered to be in an opaque section of the profile.
 */
GoFx(k32u) GoAdvanced_SpotTranslucentOpaqueWidthLimitMin(GoAdvanced advanced);

/** 
 * Returns the maximum limit of the spot width threshold below which spots are considered to be in an opaque section of the profile.
 *
 * @public             @memberof GoAdvanced
 * @version            Introduced in firmware 6.1.18.16
 * @param   advanced   GoAdvanced object.
 * @return             The maximum limit of the spot width threshold below which spots are considered to be in an opaque section of the profile.
 */
GoFx(k32u) GoAdvanced_SpotTranslucentOpaqueWidthLimitMax(GoAdvanced advanced);

/** 
 * Returns a boolean value representing whether the spot width threshold is used in translucent sorting algorithm.
 *
 * @public             @memberof GoAdvanced
 * @version            Introduced in firmware 6.1.18.16
 * @param   advanced   GoAdvanced object.
 * @return             kTRUE if it is used and kFALSE otherwise.
 */
GoFx(kBool) GoAdvanced_IsSpotTranslucentOpaqueWidthUsed(GoAdvanced advanced);

/** 
 * Returns the system value of the spot width threshold below which spots are considered to be in an opaque section of the profile.
 *
 * @public             @memberof GoAdvanced
 * @version            Introduced in firmware 6.1.18.16
 * @param   advanced   GoAdvanced object.
 * @return             The system value of the spot width threshold below which spots are considered to be in an opaque section of the profile.
 */
GoFx(k32u) GoAdvanced_SpotTranslucentOpaqueWidthSystemValue(GoAdvanced advanced);

/**
* Returns the spot width required to activate a translucent section in the profile. This is in number of pixels in the video image Y direction.
*
* @public             @memberof GoAdvanced
* @version            Introduced in firmware 6.1.18.16
* @param   advanced   GoAdvanced object.
* @return             The spot width required to activate a translucent section in the profile. This is in number of pixels in the video image Y direction.
*/
GoFx(k32u) GoAdvanced_SpotTranslucentTranslucentWidth(GoAdvanced advanced);

/**
* Sets the spot width required to activate a translucent section in the profile. This is in number of pixels in the video image Y direction.
*
* @public             @memberof GoAdvanced
* @version            Introduced in firmware 6.1.18.16
* @param   advanced   GoAdvanced object.
* @param   value      The spot width required to activate a translucent section in the profile.
* @return             Operation status.
*/
GoFx(kStatus) GoAdvanced_SetSpotTranslucentTranslucentWidth(GoAdvanced advanced, k32u value);

/** 
 * Returns the minimum limit of the spot width required to activate a translucent section in the profile. This is in number of pixels in the video image Y direction.
*
 *
 * @public             @memberof GoAdvanced
 * @version            Introduced in firmware 6.1.18.16
 * @param   advanced   GoAdvanced object.
 * @return             The minimum limit of the spot width required to activate a translucent section in the profile. 
 */
GoFx(k32u) GoAdvanced_SpotTranslucentTranslucentWidthLimitMin(GoAdvanced advanced);

/** 
 * Returns the maximum limit of the spot width required to activate a translucent section in the profile.
 *
 * @public             @memberof GoAdvanced
 * @version            Introduced in firmware 6.1.18.16
 * @param   advanced   GoAdvanced object.
 * @return             The maximum limit of the spot width required to activate a translucent section in the profile.
 */
GoFx(k32u) GoAdvanced_SpotTranslucentTranslucentWidthLimitMax(GoAdvanced advanced);

/** 
 * Returns a boolean value representing whether the spot width required to activate a translucent section is used in translucent sorting algorithm.
 *
 * @public             @memberof GoAdvanced
 * @version            Introduced in firmware 6.1.18.16
 * @param   advanced   GoAdvanced object.
 * @return             kTRUE if it is used and kFALSE otherwise.
 */
GoFx(kBool) GoAdvanced_IsSpotTranslucentTranslucentWidthUsed(GoAdvanced advanced);

/** 
 * Returns the system value of the spot width required to activate a translucent section in the profile.
 *
 * @public             @memberof GoAdvanced
 * @version            Introduced in firmware 6.1.18.16
 * @param   advanced   GoAdvanced object.
 * @return             The system value of the spot width required to activate a translucent section in the profile.
 */
GoFx(k32u) GoAdvanced_SpotTranslucentTranslucentWidthSystemValue(GoAdvanced advanced);

/**
 * Returns the minimum length of a translucent section. This is in number of pixels in the video image X direction.
 *
 * @public             @memberof GoAdvanced
 * @version            Introduced in firmware 6.1.18.16
 * @param   advanced   GoAdvanced object.
 * @return             The minimum length of a translucent section.
 */
GoFx(k32u) GoAdvanced_SpotTranslucentMinimumLength(GoAdvanced advanced);

/**
* Sets the minimum length of a translucent section. This is in number of pixels in the video image X direction.
*
* @public             @memberof GoAdvanced
* @version            Introduced in firmware 6.1.18.16
* @param   advanced   GoAdvanced object.
* @param   value      The minimum length of a translucent section.
* @return             Operation status.
*/
GoFx(kStatus) GoAdvanced_SetSpotTranslucentMinimumLength(GoAdvanced advanced, k32u value);

/** 
 * Returns the minimum limit of minimum length of a translucent section.
 *
 *
 * @public             @memberof GoAdvanced
 * @version            Introduced in firmware 6.1.18.16
 * @param   advanced   GoAdvanced object.
 * @return             The minimum limit of the minimum length of a translucent section.
 */
GoFx(k32u) GoAdvanced_SpotTranslucentMinimumLengthLimitMin(GoAdvanced advanced);

/** 
 * Returns the maximum limit of the minimum length of a translucent section.
 *
 * @public             @memberof GoAdvanced
 * @version            Introduced in firmware 6.1.18.16
 * @param   advanced   GoAdvanced object.
 * @return             The maximum limit of the minimum length of a translucent section.
 */
GoFx(k32u) GoAdvanced_SpotTranslucentMinimumLengthLimitMax(GoAdvanced advanced);

/** 
 * Returns a boolean value representing whether the minimum length of a translucent section is used in translucent sorting algorithm.
 *
 * @public             @memberof GoAdvanced
 * @version            Introduced in firmware 6.1.18.16
 * @param   advanced   GoAdvanced object.
 * @return             kTRUE if it is used and kFALSE otherwise.
 */
GoFx(kBool) GoAdvanced_IsSpotTranslucentMinimumLengthUsed(GoAdvanced advanced);

/** 
 * Returns the system value of the minimum length of a translucent section.
 *
 * @public             @memberof GoAdvanced
 * @version            Introduced in firmware 6.1.18.16
 * @param   advanced   GoAdvanced object.
 * @return             The system value of the minimum length of a translucent section.
 */
GoFx(k32u) GoAdvanced_SpotTranslucentMinimumLengthSystemValue(GoAdvanced advanced);

/** 
 * Returns the threading mode options count used in translucent sorting algorithm.
 *
 * @public             @memberof GoAdvanced
 * @version            Introduced in firmware 6.1.18.16
 * @param   advanced   GoAdvanced object.
 * @return             The threading mode options count used in translucent sorthing algorithm.
 */
GoFx(kSize) GoAdvanced_SpotTranslucentThreadingModeOptionCount(GoAdvanced advanced);

/** 
 * Returns the threading mode option at the given index used in translucent sorting algorithm.
 *
 * @public             @memberof GoAdvanced
 * @version            Introduced in firmware 6.1.18.16
 * @param   advanced   GoAdvanced object.
 * @param   index      The option list index to access.
 * @return             The threading mode option at the given index used in translucent sorthing algorithm.
 */
GoFx(GoTranslucentThreadingMode) GoAdvanced_SpotTranslucentThreadingModeOptionAt(GoAdvanced advanced, kSize index);

/** 
 * Sets the threading mode used in translucent sorting algorithm.
 *
 * @public             @memberof GoAdvanced
 * @version            Introduced in firmware 6.1.18.16
 * @param   advanced   GoAdvanced object.
 * @param   mode       Threading mode.
 * @return             Operation status.
 */
GoFx(kStatus) GoAdvanced_SetSpotTranslucentThreadingMode(GoAdvanced advanced, GoTranslucentThreadingMode mode);

/** 
 * Returns the threading mode used in translucent sorting algorithm.
 *
 * @public             @memberof GoAdvanced
 * @version            Introduced in firmware 6.1.18.16
 * @param   advanced   GoAdvanced object.
 * @return             The threading mode used in translucent sorting algorithm.
 */
GoFx(GoTranslucentThreadingMode) GoAdvanced_SpotTranslucentThreadingMode(GoAdvanced advanced);

/** 
 * Returns a boolean value representing whether threading mode is used in translucent sorting algorithm.
 *
 * @public             @memberof GoAdvanced
 * @version            Introduced in firmware 6.1.18.16
 * @param   advanced   GoAdvanced object.
 * @return             kTRUE if it is used and kFALSE otherwise.
 */
GoFx(kBool) GoAdvanced_IsSpotTranslucentThreadingModeUsed(GoAdvanced advanced);

/** 
 * Returns the system value of threading mode used in translucent sorting algorithm.
 *
 * @public             @memberof GoAdvanced
 * @version            Introduced in firmware 6.1.18.16
 * @param   advanced   GoAdvanced object.
 * @return             The system value of threading mode used in translucent sorting algorithm.
 */
GoFx(GoTranslucentThreadingMode) GoAdvanced_SpotTranslucentThreadingModeSystemValue(GoAdvanced advanced);

/** 
 * Sets the analog camera gain.
 *
 * @public             @memberof GoAdvanced
 * @version            Introduced in firmware 4.6.1.140
 * @param   advanced   GoAdvanced object.
 * @param   value      Analog camera gain.
 * @return             Operation status.
 */
GoFx(kStatus) GoAdvanced_SetCameraGainAnalog(GoAdvanced advanced, k64f value);

/** 
 * Returns the user defined analog camera gain value.
 *
 * @public             @memberof GoAdvanced
 * @version            Introduced in firmware 4.6.1.140
 * @param   advanced   GoAdvanced object.
 * @return             Analog camera gain value.
 */
GoFx(k64f) GoAdvanced_CameraGainAnalog(GoAdvanced advanced);

/** 
 * Returns the analog camera gain minimum value limit.
 *
 * @public             @memberof GoAdvanced
 * @version            Introduced in firmware 4.6.1.140
 * @param   advanced   GoAdvanced object.
 * @return             Analog camera gain minimum value limit.
 */
GoFx(k64f) GoAdvanced_CameraGainAnalogLimitMin(GoAdvanced advanced);

/** 
 * Returns the analog camera gain maximum value limit.
 *
 * @public             @memberof GoAdvanced
 * @version            Introduced in firmware 4.6.1.140
 * @param   advanced   GoAdvanced object.
 * @return             Analog camera gain maximum value limit.
 */
GoFx(k64f) GoAdvanced_CameraGainAnalogLimitMax(GoAdvanced advanced);

/** 
 * Returns a boolean value representing whether the user defined analog camera gain is used.
 *
 * @public             @memberof GoAdvanced
 * @version            Introduced in firmware 4.6.1.140
 * @param   advanced   GoAdvanced object.
 * @return             kTRUE if the user defined analog camera gain is used and kFALSE otherwise.
 */
GoFx(kBool) GoAdvanced_IsCameraGainAnalogUsed(GoAdvanced advanced);

/** 
 * Returns the analog camera gain system value.
 *
 * @public             @memberof GoAdvanced
 * @version            Introduced in firmware 4.6.1.140
 * @param   advanced   GoAdvanced object.
 * @return             The analog camera gain system value.
 */
GoFx(k64f) GoAdvanced_CameraGainAnalogSystemValue(GoAdvanced advanced);

/** 
 * Sets the digital camera gain
 *
 * @public             @memberof GoAdvanced
 * @version            Introduced in firmware 4.6.1.140
 * @param   advanced   GoAdvanced object.
 * @param   value      Digital camera gain.
 * @return             Operation status.
 */
GoFx(kStatus) GoAdvanced_SetCameraGainDigital(GoAdvanced advanced, k64f value);

/** 
 * Returns the user defined digital camera gain value.
 *
 * @public             @memberof GoAdvanced
 * @version            Introduced in firmware 4.6.1.140
 * @param   advanced   GoAdvanced object.
 * @return             The digital camera gain system value.
 */
GoFx(k64f) GoAdvanced_CameraGainDigital(GoAdvanced advanced);

/** 
 * Returns the digital camera gain minimum value limit.
 *
 * @public             @memberof GoAdvanced
 * @version            Introduced in firmware 4.6.1.140
 * @param   advanced   GoAdvanced object.
 * @return             Digital camera gain minimum value limit.
 */
GoFx(k64f) GoAdvanced_CameraGainDigitalLimitMin(GoAdvanced advanced);

/** 
 * Returns the digital camera gain maximum value limit.
 *
 * @public             @memberof GoAdvanced
 * @version            Introduced in firmware 4.6.1.140
 * @param   advanced   GoAdvanced object.
 * @return             Digital camera gain maximum value limit.
 */
GoFx(k64f) GoAdvanced_CameraGainDigitalLimitMax(GoAdvanced advanced);

/** 
 * Returns a boolean value representing whether the user's digital camera gain value is used by the system.
 *
 * @public             @memberof GoAdvanced
 * @version            Introduced in firmware 4.6.1.140
 * @param   advanced   GoAdvanced object.
 * @return             kTRUE if used and kFALSE otherwise.
 */
GoFx(kBool) GoAdvanced_IsCameraGainDigitalUsed(GoAdvanced advanced);

/** 
 * Returns the system's digital camera gain value.
 *
 * @public             @memberof GoAdvanced
 * @version            Introduced in firmware 4.6.1.140
 * @param   advanced   GoAdvanced object.
 * @return             Digital camera gain system value.
 */
GoFx(k64f) GoAdvanced_CameraGainDigitalSystemValue(GoAdvanced advanced);

/** 
 * Sets the dynamic sensitivity.
 *
 * @public             @memberof GoAdvanced
 * @version            Introduced in firmware 4.6.1.140
 * @param   advanced   GoAdvanced object.
 * @param   value      Dynamic sensitivity.
 * @return             Operation status.
 */
GoFx(kStatus) GoAdvanced_SetDynamicSensitivity(GoAdvanced advanced, k64f value);

/** 
 * Returns the user defined dynamic sensitivity value.
 *
 * @public             @memberof GoAdvanced
 * @version            Introduced in firmware 4.6.1.140
 * @param   advanced   GoAdvanced object.
 * @return             User defined dynamic sensitivity value.
 */
GoFx(k64f) GoAdvanced_DynamicSensitivity(GoAdvanced advanced);

/** 
 * Returns the dynamic sensitivity minimum value limit.
 *
 * @public             @memberof GoAdvanced
 * @version            Introduced in firmware 4.6.1.140
 * @param   advanced   GoAdvanced object.
 * @return             Dynamic sensitivity minimum value limit.
 */
GoFx(k64f) GoAdvanced_DynamicSensitivityLimitMin(GoAdvanced advanced);

/** 
 * Returns the dynamic sensitivity maximum value limit.
 *
 * @public             @memberof GoAdvanced
 * @version            Introduced in firmware 4.6.1.140
 * @param   advanced   GoAdvanced object.
 * @return             Dynamic sensitivity maximum value limit.
 */
GoFx(k64f) GoAdvanced_DynamicSensitivityLimitMax(GoAdvanced advanced);

/** 
 * Returns a boolean representing whether the user defined dynamic sensitivity value is used.
 *
 * @public             @memberof GoAdvanced
 * @version            Introduced in firmware 4.6.1.140
 * @param   advanced   GoAdvanced object.
 * @return             kTRUE if used and kFALSE otherwise.
 */
GoFx(kBool) GoAdvanced_IsDynamicSensitivityUsed(GoAdvanced advanced);

/** 
 * Returns the dynamic sensitivity system value.
 *
 * @public             @memberof GoAdvanced
 * @version            Introduced in firmware 4.6.1.140
 * @param   advanced   GoAdvanced object.
 * @return             Dynamic sensitivity system value.
 */
GoFx(k64f) GoAdvanced_DynamicSensitivitySystemValue(GoAdvanced advanced);

/** 
 * Sets the dynamic threshold.
 *
 * @public             @memberof GoAdvanced
 * @version            Introduced in firmware 4.6.1.140
 * @param   advanced   GoAdvanced object.
 * @param   value      Dynamic threshold.
 * @return             Operation status.
 */
GoFx(kStatus) GoAdvanced_SetDynamicThreshold(GoAdvanced advanced, k32u value);

/** 
 * Returns the dynamic threshold minimum value limit.
 *
 * @public             @memberof GoAdvanced
 * @version            Introduced in firmware 4.6.1.140
 * @param   advanced   GoAdvanced object.
 * @return             Dynamic threshold minimum value limit.
 */
GoFx(k32u) GoAdvanced_DynamicThresholdLimitMin(GoAdvanced advanced);

/** 
 * Returns the dynamic threshold maximum value limit.
 *
 * @public             @memberof GoAdvanced
 * @version            Introduced in firmware 4.6.1.140
 * @param   advanced   GoAdvanced object.
 * @return             Dynamic threshold maximum value limit.
 */
GoFx(k32u) GoAdvanced_DynamicThresholdLimitMax(GoAdvanced advanced);

/** 
 * Returns the user defined dynamic threshold value.
 *
 * @public             @memberof GoAdvanced
 * @version            Introduced in firmware 4.6.1.140
 * @param   advanced   GoAdvanced object.
 * @return             The user defined dynamic threshold value.
 */
GoFx(k32u) GoAdvanced_DynamicThreshold(GoAdvanced advanced);

/** 
 * Returns a boolean representing whether or not the user defined dynamic threshold is used by the system.
 *
 * @public             @memberof GoAdvanced
 * @version            Introduced in firmware 4.6.1.140
 * @param   advanced   GoAdvanced object.
 * @return             Dynamic threshold minimum value limit.
 */
GoFx(kBool) GoAdvanced_IsDynamicThresholdUsed(GoAdvanced advanced);

/** 
 * Returns the dynamic threshold system value.
 *
 * @public             @memberof GoAdvanced
 * @version            Introduced in firmware 4.6.1.140
 * @param   advanced   GoAdvanced object.
 * @return             Dynamic threshold system value.
 */
GoFx(k32u) GoAdvanced_DynamicThresholdSystemValue(GoAdvanced advanced);

/** 
 * Sets the gamma type.
 *
 * @public             @memberof GoAdvanced
 * @version            Introduced in firmware 4.6.1.140
 * @param   advanced   GoAdvanced object.
 * @param   value      Gamma type.
 * @return             Operation status.
 */
GoFx(kStatus) GoAdvanced_SetGammaType(GoAdvanced advanced, GoGammaType value);

/** 
 * Returns the user defined gamma type.
 *
 * @public             @memberof GoAdvanced
 * @version            Introduced in firmware 4.6.1.140
 * @param   advanced   GoAdvanced object.
 * @return             User defined gamma type.
 */
GoFx(GoGammaType) GoAdvanced_GammaType(GoAdvanced advanced);

/** 
 * Returns a boolean representing whether the user defined gamma type is used by the system.
 *
 * @public             @memberof GoAdvanced
 * @version            Introduced in firmware 4.6.1.140
 * @param   advanced   GoAdvanced object.
 * @return             kTRUE if used and kFALSE otherwise.
 */
GoFx(kBool) GoAdvanced_IsGammaTypeUsed(GoAdvanced advanced);

/** 
 * Returns the system's gamma type value.
 *
 * @public             @memberof GoAdvanced
 * @version            Introduced in firmware 4.6.1.140
 * @param   advanced   GoAdvanced object.
 * @return             The system gamma type value.
 */
GoFx(GoGammaType) GoAdvanced_GammaTypeSystemValue(GoAdvanced advanced);

/** 
 * Enables or disables senstivity compensation. NOTE: This is only applicable to 
 * 2300 B series sensors.
 *
 * @public             @memberof GoAdvanced
 * @version            Introduced in firmware 4.6.1.140
 * @param   advanced   GoAdvanced object.
 * @param   value      kTRUE to enable and kFALSE to disable.
 * @return             Operation status.
 */
GoFx(kStatus) GoAdvanced_EnableSensitivityCompensation(GoAdvanced advanced, kBool value);

/** 
 * Returns the user defined sensitivity compensation value.
 *
 * @public             @memberof GoAdvanced
 * @version            Introduced in firmware 4.6.1.140
 * @param   advanced   GoAdvanced object.
 * @return             User defined sensitivity compensation.
 */
GoFx(kBool) GoAdvanced_SensitivityCompensationEnabled(GoAdvanced advanced);

/** 
 * Returns a boolean representing whether the user defined sensitivity compensation is used by the system.
 *
 * @public             @memberof GoAdvanced
 * @version            Introduced in firmware 4.6.1.140
 * @param   advanced   GoAdvanced object.
 * @return             kTRUE if used and kFALSE otherwise.
 */
GoFx(kBool) GoAdvanced_IsSensitivityCompensationEnabledUsed(GoAdvanced advanced);

/** 
 * Returns the system's sensitivity compensation value.
 *
 * @public             @memberof GoAdvanced
 * @version            Introduced in firmware 4.6.1.140
 * @param   advanced   GoAdvanced object.
 * @return             kTRUE if enabled and kFALSE otherwise.
 */
GoFx(kBool) GoAdvanced_SensitivityCompensationEnabledSystemValue(GoAdvanced advanced);

/**
* Sets surface engine encoding type (Default Standard)
*
* @public             @memberof GoAdvanced
* @note               Supported with G3
* @version            Introduced in firmware 4.7.3.22
* @param   advanced   GoAdvanced object.
* @param   encoding   Surface Engine Encoding type
* @return             Operation status.
*/
GoFx(kStatus) GoAdvanced_SetSurfaceEncoding(GoAdvanced advanced, GoSurfaceEncoding encoding);

/**
* Returns the surface engine encoding type.
*
* @public             @memberof GoAdvanced
* @note               Supported with G3
* @version            Introduced in firmware 4.7.3.22
* @param   advanced   GoAdvanced object.
* @return             The Surface Engine Encoding type.
*/
GoFx(GoSurfaceEncoding) GoAdvanced_SurfaceEncoding(GoAdvanced advanced);

/**
* Returns the surface engine encoding type.
*
* @public             @memberof GoAdvanced
* @note               Supported with G3
* @version            Introduced in firmware 5.3.19.50
* @param   advanced   GoAdvanced object.
* @return             kTRUE if used and kFALSE otherwise.
*/
GoFx(kBool) GoAdvanced_IsSurfaceEncodingUsed(GoAdvanced advanced);

/**
* Returns the system's surface engine encoding type.
*
* @public             @memberof GoAdvanced
* @note               Supported with G3
* @version            Introduced in firmware 5.3.19.50
* @param   advanced   GoAdvanced object.
* @return             The Surface Engine Encoding type.
*/
GoFx(GoSurfaceEncoding) GoAdvanced_SurfaceEncodingSystemValue(GoAdvanced advanced);

/**
* Sets the phase filter type. (Default None)
*
* @public               @memberof GoAdvanced
* @note                 Supported with G3
* @version              Introduced in firmware 4.7.3.22
* @param   advanced     GoAdvanced object.
* @param   phaseFilter  Phase Filter type.
* @return               Operation status.
*/
GoFx(kStatus) GoAdvanced_SetSurfacePhaseFilter(GoAdvanced advanced, GoSurfacePhaseFilter phaseFilter);

/**
* Returns the phase filter type.
*
* @public             @memberof GoAdvanced
* @note               Supported with G3
* @version            Introduced in firmware 4.7.3.22
* @param   advanced   GoAdvanced object.
* @return             The phase filter type.
*/
GoFx(GoSurfacePhaseFilter) GoAdvanced_SurfacePhaseFilter(GoAdvanced advanced);

/** 
 * Sets the contrast threshold.
 *
 * @public             @memberof GoAdvanced
 * @note               Supported with G3
 * @version            Introduced in firmware 5.3.12.7
 * @param   advanced   GoAdvanced object.
 * @param   value      Contrast threshold.
 * @return             Operation status.
 */
GoFx(kStatus) GoAdvanced_SetContrastThreshold(GoAdvanced advanced, k32u value);

/** 
 * Returns the user defined contrast threshold.
 *
 * @public             @memberof GoAdvanced
 * @note               Supported with G3
 * @version            Introduced in firmware 5.3.12.7
 * @param   advanced   GoAdvanced object.
 * @return             The contrast threshold.
 */
GoFx(k32u) GoAdvanced_ContrastThreshold(GoAdvanced advanced);

/** 
 * Returns the minimum contrast threshold limit.
 *
 * @public             @memberof GoAdvanced
 * @note               Supported with G3
 * @version            Introduced in firmware 5.3.12.7
 * @param   advanced   GoAdvanced object.
 * @return             The minimum contrast threshold.
 */
GoFx(k32u) GoAdvanced_ContrastThresholdLimitMin(GoAdvanced advanced);

/** 
 * Returns the maximum contrast threshold limit.
 *
 * @public             @memberof GoAdvanced
 * @note               Supported with G3
 * @version            Introduced in firmware 5.3.12.7
 * @param   advanced   GoAdvanced object.
 * @return             The maximum contrast threshold.
 */
GoFx(k32u) GoAdvanced_ContrastThresholdLimitMax(GoAdvanced advanced);

/** 
 * Returns a boolean value representing whether the user specified contrast threshold value is used.
 *
 * @public             @memberof GoAdvanced
 * @note               Supported with G3
 * @version            Introduced in firmware 5.3.12.7
 * @param   advanced   GoAdvanced object.
 * @return             kTRUE if it is used and kFALSE otherwise.
 */
GoFx(kBool) GoAdvanced_IsContrastThresholdUsed(GoAdvanced advanced);

/** 
 * Returns the system contrast threshold value.
 *
 * @public             @memberof GoAdvanced
 * @note               Supported with G3
 * @version            Introduced in firmware 5.3.12.7
 * @param   advanced   GoAdvanced object.
 * @return             The system contrast threshold.
 */
GoFx(k32u) GoAdvanced_ContrastThresholdSystemValue(GoAdvanced advanced);

#include <GoSdk/GoAdvanced.x.h>

#endif

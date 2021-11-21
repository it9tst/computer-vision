/** 
 * @file    GoMaterial.h
 * @brief   Declares the GoMaterial class. 
 * @deprecated
 * @internal
 * Copyright (C) 2016-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef GO_MATERIAL_H
#define GO_MATERIAL_H

#include <GoSdk/GoSdkDef.h>
#include <GoSdk/GoAdvanced.h>

//// Deprecated Class

/**
 * [Deprecated] Use GoAdvanced class instead.
 *
 * @deprecated
 * @class   GoMaterial
 * @extends kObject
 * @note    Supported with G1, G2
 * @ingroup GoSdk
 * @brief   Represents configurable material acquisition settings.
 */
typedef GoAdvanced GoMaterial;

/** 
 * [Deprecated] Use GoAdvanced_SetType() instead.
 *
 * Sets the material acquisition type.
 *
 * @deprecated
 * @public             @memberof GoMaterial
 * @note               Supported with G1, G2
 * @version            Introduced in firmware 4.1.3.106
 * @param   material   GoMaterial object.
 * @param   type       The material type to set.
 * @return             Operation status.
 */
GoFx(kStatus) GoMaterial_SetType(GoMaterial material, GoMaterialType type);

/** 
 * [Deprecated] Use GoAdvanced_Type() instead.
 *
 * Returns the user defined material acquisition type.
 *
 * @deprecated
 * @public             @memberof GoMaterial
 * @note               Supported with G1, G2
 * @version            Introduced in firmware 4.1.3.106
 * @param   material   GoMaterial object.
 * @return             The material type.
 */
GoFx(GoMaterialType) GoMaterial_Type(GoMaterial material);

/** 
 * [Deprecated] Use GoAdvanced_IsTypeUsed() instead.
 *
 * Returns a boolean relating to whether the user defined material acquisition type value will be used by the system.
 *
 * @deprecated
 * @public             @memberof GoMaterial
 * @note               Supported with G1, G2
 * @version            Introduced in firmware 4.1.3.106
 * @param   material   GoMaterial object.
 * @return             kTRUE if the user defined material type will be used and kFALSE otherwise.
 */
GoFx(kBool) GoMaterial_IsTypeUsed(GoMaterial material);

/** 
 * [Deprecated] Use GoAdvanced_TypeSystemValue() instead.
 *
 * Returns the material acquisition type to be used by the system.
 *
 * @deprecated
 * @public             @memberof GoMaterial
 * @note               Supported with G1, G2
 * @version            Introduced in firmware 4.1.3.106
 * @param   material   GoMaterial object.
 * @return             The system value material type.
 */
GoFx(GoMaterialType) GoMaterial_TypeSystemValue(GoMaterial material);

/** 
 * [Deprecated] Use GoAdvanced_SetSpotThreshold() instead.
 *
 * Sets the spot threshold.
 *
 * @deprecated
 * @public             @memberof GoMaterial
 * @note               Supported with G1, G2
 * @version            Introduced in firmware 4.1.3.106
 * @param   material   GoMaterial object.
 * @param   value      Spot threshold.
 * @return             Operation status.
 */
GoFx(kStatus) GoMaterial_SetSpotThreshold(GoMaterial material, k32u value);

/** 
 * [Deprecated] Use GoAdvanced_SpotThreshold() instead.
 *
 * Returns the user defined spot threshold.
 *
 * @deprecated
 * @public             @memberof GoMaterial
 * @note               Supported with G1, G2
 * @version            Introduced in firmware 4.1.3.106
 * @param   material   GoMaterial object.
 * @return             The spot threshold.
 */
GoFx(k32u) GoMaterial_SpotThreshold(GoMaterial material);

/** 
 * [Deprecated] Use GoAdvanced_SpotThresholdLimitMin() instead.
 *
 * Returns the minimum spot threshold limit.
 *
 * @deprecated
 * @public             @memberof GoMaterial
 * @note               Supported with G1, G2
 * @version            Introduced in firmware 4.1.3.106
 * @param   material   GoMaterial object.
 * @return             The minimum spot threshold.
 */
GoFx(k32u) GoMaterial_SpotThresholdLimitMin(GoMaterial material);

/** 
 * [Deprecated] Use GoAdvanced_SpotThresholdLimitMax() instead.
 *
 * Returns the maximum spot threshold limit.
 *
 * @deprecated
 * @public             @memberof GoMaterial
 * @note               Supported with G1, G2
 * @version            Introduced in firmware 4.1.3.106
 * @param   material   GoMaterial object.
 * @return             The maximum spot threshold.
 */
GoFx(k32u) GoMaterial_SpotThresholdLimitMax(GoMaterial material);

/** 
 * [Deprecated] Use GoAdvanced_IsSpotThresholdUsed() instead.
 *
 * Returns a boolean value representing whether the user specified spot threshold value is used.
 *
 * @deprecated
 * @public             @memberof GoMaterial
 * @note               Supported with G1, G2
 * @version            Introduced in firmware 4.1.3.106
 * @param   material   GoMaterial object.
 * @return             kTRUE if it is used and kFALSE otherwise.
 */
GoFx(kBool) GoMaterial_IsSpotThresholdUsed(GoMaterial material);

/** 
 * [Deprecated] Use GoAdvanced_SpotThresholdSystemValue() instead.
 *
 * Returns the system spot threshold value.
 *
 * @deprecated
 * @public             @memberof GoMaterial
 * @note               Supported with G1, G2
 * @version            Introduced in firmware 4.1.3.106
 * @param   material   GoMaterial object.
 * @return             The system spot threshold.
 */
GoFx(k32u) GoMaterial_SpotThresholdSystemValue(GoMaterial material);

/** 
 * [Deprecated] Use GoAdvanced_SetSpotWidthMax() instead.
 *
 * Sets the maximum spot width.
 *
 * @deprecated
 * @public             @memberof GoMaterial
 * @note               Supported with G1, G2
 * @version            Introduced in firmware 4.1.3.106
 * @param   material   GoMaterial object.
 * @param   value      Maximum spot width.
 * @return             Operation status.
 */
GoFx(kStatus) GoMaterial_SetSpotWidthMax(GoMaterial material, k32u value);

/** 
 * [Deprecated] Use GoAdvanced_SpotWidthMax() instead.
 *
 * Returns the user defined maximum spot width.
 *
 * @deprecated
 * @public             @memberof GoMaterial
 * @note               Supported with G1, G2
 * @version            Introduced in firmware 4.1.3.106
 * @param   material   GoMaterial object.
 * @return             The maximum spot width.
 */
GoFx(k32u) GoMaterial_SpotWidthMax(GoMaterial material);

/** 
 * [Deprecated] Use GoAdvanced_SpotWidthMaxLimitMin() instead.
 *
 * Returns the maximum spot width minimum limit.
 *
 * @deprecated
 * @public             @memberof GoMaterial
 * @note               Supported with G1, G2
 * @version            Introduced in firmware 4.1.3.106
 * @param   material   GoMaterial object.
 * @return             The maximum spot width minimum limit.
 */
GoFx(k32u) GoMaterial_SpotWidthMaxLimitMin(GoMaterial material);

/** 
 * [Deprecated] Use GoAdvanced_SpotWidthMaxLimitMax() instead.
 *
 * Returns the maximum spot width maximum limit.
 *
 * @deprecated
 * @public             @memberof GoMaterial
 * @note               Supported with G1, G2
 * @version            Introduced in firmware 4.1.3.106
 * @param   material   GoMaterial object.
 * @return             The maximum spot width maximum limit.
 */
GoFx(k32u) GoMaterial_SpotWidthMaxLimitMax(GoMaterial material);

/** 
 * [Deprecated] Use GoAdvanced_IsSpotWidthMaxUsed() instead.
 *
 * Returns a boolean relating to whether the user defined spot width max value will be used by the system.
 *
 * @deprecated
 * @public             @memberof GoMaterial
 * @note               Supported with G1, G2
 * @version            Introduced in firmware 4.1.3.106
 * @param   material   GoMaterial object.
 * @return             kTRUE if the user value will be used and kFALSE otherwise.
 */
GoFx(kBool) GoMaterial_IsSpotWidthMaxUsed(GoMaterial material);

/** 
 * [Deprecated] Use GoAdvanced_SpotWidthMaxSystemValue() instead.
 *
 * Returns the maximum spot width system value.
 *
 * @deprecated
 * @public             @memberof GoMaterial
 * @note               Supported with G1, G2
 * @version            Introduced in firmware 4.1.3.106
 * @param   material   GoMaterial object.
 * @return             The maximum spot width system value.
 */
GoFx(k32u) GoMaterial_SpotWidthMaxSystemValue(GoMaterial material);

/** 
 * [Deprecated] Use GoAdvanced_SpotSelectionTypeOptionCount() instead.
 *
 * Returns the number of spot selection type options.
 *
 * @deprecated
 * @public             @memberof GoMaterial
 * @note               Supported with G1, G2
 * @version            Introduced in firmware 4.3.3.124
 * @param   material   GoMaterial object.
 * @return             The spot selection type option count.
 */
GoFx(kSize) GoMaterial_SpotSelectionTypeOptionCount(GoMaterial material);

/** 
 * [Deprecated] Use GoAdvanced_SpotSelectionTypeOptionAt() instead.
 *
 * Returns the spot selection type option at the given index.
 *
 * @deprecated
 * @public             @memberof GoMaterial
 * @note               Supported with G1, G2
 * @version            Introduced in firmware 4.3.3.124
 * @param   material   GoMaterial object.
 * @param   index      The option list index to access.
 * @return             The spot selection type option at the given index.
 */
GoFx(GoSpotSelectionType) GoMaterial_SpotSelectionTypeOptionAt(GoMaterial material, kSize index);

/** 
 * [Deprecated] Use GoAdvanced_SetSpotSelectionType() instead.
 *
 * Sets the spot selection type.
 *
 * @deprecated
 * @public             @memberof GoMaterial
 * @note               Supported with G1, G2
 * @version            Introduced in firmware 4.1.3.106
 * @param   material   GoMaterial object.
 * @param   type       Spot selection type.
 * @return             Operation status.
 */
GoFx(kStatus) GoMaterial_SetSpotSelectionType(GoMaterial material, GoSpotSelectionType type);

/** 
 * [Deprecated] Use GoAdvanced_SpotSelectionType() instead.
 *
 * Returns the user defined spot selection type.
 *
 * @deprecated
 * @public             @memberof GoMaterial
 * @note               Supported with G1, G2
 * @version            Introduced in firmware 4.1.3.106
 * @param   material   GoMaterial object.
 * @return             The maximum spot width.
 */
GoFx(GoSpotSelectionType) GoMaterial_SpotSelectionType(GoMaterial material);

/** 
 * [Deprecated] Use GoAdvanced_IsSpotSelectionTypeUsed() instead.
 *
 * Returns a boolean relating to whether the user defined spot selection type will be used by the system.
 *
 * @deprecated
 * @public             @memberof GoMaterial
 * @note               Supported with G1, G2
 * @version            Introduced in firmware 4.1.3.106
 * @param   material   GoMaterial object.
 * @return             kTRUE if the user value will be used and kFALSE otherwise.
 */
GoFx(kBool) GoMaterial_IsSpotSelectionTypeUsed(GoMaterial material);

/** 
 * [Deprecated] Use GoAdvanced_SpotSelectionTypeSystemValue() instead.
 *
 * Returns the system spot selection type.
 *
 * @deprecated
 * @public             @memberof GoMaterial
 * @note               Supported with G1, G2
 * @version            Introduced in firmware 4.1.3.106
 * @param   material   GoMaterial object.
 * @return             System spot selection type.
 */
GoFx(GoSpotSelectionType) GoMaterial_SpotSelectionTypeSystemValue(GoMaterial material);

/** 
 * [Deprecated] Use GoAdvanced_SetCameraGainAnalog() instead.
 *
 * Sets the analog camera gain.
 *
 * @deprecated
 * @public             @memberof GoMaterial
 * @note               Supported with G1, G2
 * @version            Introduced in firmware 4.1.3.106
 * @param   material   GoMaterial object.
 * @param   value      Analog camera gain.
 * @return             Operation status.
 */
GoFx(kStatus) GoMaterial_SetCameraGainAnalog(GoMaterial material, k64f value);

/** 
 * [Deprecated] Use GoAdvanced_CameraGainAnalog() instead.
 *
 * Returns the user defined analog camera gain value.
 *
 * @deprecated
 * @public             @memberof GoMaterial
 * @note               Supported with G1, G2
 * @version            Introduced in firmware 4.1.3.106
 * @param   material   GoMaterial object.
 * @return             Analog camera gain value.
 */
GoFx(k64f) GoMaterial_CameraGainAnalog(GoMaterial material);

/** 
 *
 * [Deprecated] Use GoAdvanced_CameraGainAnalogLimitMin() instead.
 *
 * Returns the analog camera gain minimum value limit.
 *
 * @deprecated
 * @public             @memberof GoMaterial
 * @note               Supported with G1, G2
 * @version            Introduced in firmware 4.1.3.106
 * @param   material   GoMaterial object.
 * @return             Analog camera gain minimum value limit.
 */
GoFx(k64f) GoMaterial_CameraGainAnalogLimitMin(GoMaterial material);

/** 
 * [Deprecated] Use GoAdvanced_CameraGainAnalogLimitMax() instead.
 *
 * Returns the analog camera gain maximum value limit.
 *
 * @deprecated
 * @public             @memberof GoMaterial
 * @note               Supported with G1, G2
 * @version            Introduced in firmware 4.1.3.106
 * @param   material   GoMaterial object.
 * @return             Analog camera gain maximum value limit.
 */
GoFx(k64f) GoMaterial_CameraGainAnalogLimitMax(GoMaterial material);

/** 
 * [Deprecated] Use GoAdvanced_IsCameraGainAnalogUsed() instead.
 *
 * Returns a boolean value representing whether the user defined analog camera gain is used.
 *
 * @deprecated
 * @public             @memberof GoMaterial
 * @note               Supported with G1, G2
 * @version            Introduced in firmware 4.1.3.106
 * @param   material   GoMaterial object.
 * @return             kTRUE if the user defined analog camera gain is used and kFALSE otherwise.
 */
GoFx(kBool) GoMaterial_IsCameraGainAnalogUsed(GoMaterial material);

/** 
 * [Deprecated] Use GoAdvanced_CameraGainAnalogSystemValue() instead.
 *
 * Returns the analog camera gain system value.
 *
 * @deprecated
 * @public             @memberof GoMaterial
 * @note               Supported with G1, G2
 * @version            Introduced in firmware 4.1.3.106
 * @param   material   GoMaterial object.
 * @return             The analog camera gain system value.
 */
GoFx(k64f) GoMaterial_CameraGainAnalogSystemValue(GoMaterial material);

/** 
 * [Deprecated] Use GoAdvanced_SetCameraGainDigital() instead.
 *
 * Sets the digital camera gain.
 *
 * @deprecated
 * @public             @memberof GoMaterial
 * @note               Supported with G1, G2
 * @version            Introduced in firmware 4.1.3.106
 * @param   material   GoMaterial object.
 * @param   value      Digital camera gain.
 * @return             Operation status.
 */
GoFx(kStatus) GoMaterial_SetCameraGainDigital(GoMaterial material, k64f value);

/** 
 * [Deprecated] Use GoAdvanced_CameraGainDigital() instead.
 *
 * Returns the user defined digital camera gain value.
 *
 * @deprecated
 * @public             @memberof GoMaterial
 * @note               Supported with G1, G2
 * @version            Introduced in firmware 4.1.3.106
 * @param   material   GoMaterial object.
 * @return             The digital camera gain system value.
 */
GoFx(k64f) GoMaterial_CameraGainDigital(GoMaterial material);

/** 
 * [Deprecated] Use GoAdvanced_CameraGainDigitalLimitMin() instead.
 *
 * Returns the digital camera gain minimum value limit.
 *
 * @deprecated
 * @public             @memberof GoMaterial
 * @note               Supported with G1, G2
 * @version            Introduced in firmware 4.1.3.106
 * @param   material   GoMaterial object.
 * @return             Digital camera gain minimum value limit.
 */
GoFx(k64f) GoMaterial_CameraGainDigitalLimitMin(GoMaterial material);

/** 
 * [Deprecated] Use GoAdvanced_CameraGainDigitalLimitMax() instead.
 *
 * Returns the digital camera gain maximum value limit.
 *
 * @deprecated
 * @public             @memberof GoMaterial
 * @note               Supported with G1, G2
 * @version            Introduced in firmware 4.1.3.106
 * @param   material   GoMaterial object.
 * @return             Digital camera gain maximum value limit.
 */
GoFx(k64f) GoMaterial_CameraGainDigitalLimitMax(GoMaterial material);

/** 
 * [Deprecated] Use GoAdvanced_IsCameraGainDigitalUsed() instead.
 *
 * Returns a boolean value representing whether the user's digital camera gain value is used by the system.
 *
 * @deprecated
 * @public             @memberof GoMaterial
 * @note               Supported with G1, G2
 * @version            Introduced in firmware 4.1.3.106
 * @param   material   GoMaterial object.
 * @return             kTRUE if used and kFALSE otherwise.
 */
GoFx(kBool) GoMaterial_IsCameraGainDigitalUsed(GoMaterial material);

/** 
 * [Deprecated] Use GoAdvanced_CameraGainDigitalSystemValue() instead.
 *
 * Returns the system's digital camera gain value.
 *
 * @deprecated
 * @public             @memberof GoMaterial
 * @note               Supported with G1, G2
 * @version            Introduced in firmware 4.1.3.106
 * @param   material   GoMaterial object.
 * @return             Digital camera gain system value.
 */
GoFx(k64f) GoMaterial_CameraGainDigitalSystemValue(GoMaterial material);

/** 
 * [Deprecated] Use GoAdvanced_SetDynamicSensitivity() instead.
 *
 * Sets the dynamic sensitivity.
 *
 * @deprecated
 * @public             @memberof GoMaterial
 * @note               Supported with G1, G2
 * @version            Introduced in firmware 4.1.3.106
 * @param   material   GoMaterial object.
 * @param   value      Dynamic sensitivity.
 * @return             Operation status.
 */
GoFx(kStatus) GoMaterial_SetDynamicSensitivity(GoMaterial material, k64f value);

/** 
 * [Deprecated] Use GoAdvanced_DynamicSensitivity() instead.
 *
 * Returns the user defined dynamic sensitivity value.
 *
 * @deprecated
 * @public             @memberof GoMaterial
 * @note               Supported with G1, G2
 * @version            Introduced in firmware 4.1.3.106
 * @param   material   GoMaterial object.
 * @return             User defined dynamic sensitivity value.
 */
GoFx(k64f) GoMaterial_DynamicSensitivity(GoMaterial material);

/** 
 * [Deprecated] Use GoAdvanced_DynamicSensitivityLimitMin() instead.
 *
 * Returns the dynamic sensitivity minimum value limit.
 *
 * @deprecated
 * @public             @memberof GoMaterial
 * @note               Supported with G1, G2
 * @version            Introduced in firmware 4.1.3.106
 * @param   material   GoMaterial object.
 * @return             Dynamic sensitivity minimum value limit.
 */
GoFx(k64f) GoMaterial_DynamicSensitivityLimitMin(GoMaterial material);

/** 
 * [Deprecated] Use GoAdvanced_DynamicSensitivityLimitMax() instead.
 *
 * Returns the dynamic sensitivity maximum value limit.
 *
 * @deprecated
 * @public             @memberof GoMaterial
 * @note               Supported with G1, G2
 * @version            Introduced in firmware 4.1.3.106
 * @param   material   GoMaterial object.
 * @return             Dynamic sensitivity maximum value limit.
 */
GoFx(k64f) GoMaterial_DynamicSensitivityLimitMax(GoMaterial material);

/** 
 * [Deprecated] Use GoAdvanced_IsDynamicSensitivityUsed() instead.
 *
 * Returns a boolean representing whether the user defined dynamic sensitivity value is used.
 *
 * @deprecated
 * @public             @memberof GoMaterial
 * @note               Supported with G1, G2
 * @version            Introduced in firmware 4.1.3.106
 * @param   material   GoMaterial object.
 * @return             kTRUE if used and kFALSE otherwise.
 */
GoFx(kBool) GoMaterial_IsDynamicSensitivityUsed(GoMaterial material);

/** 
 * [Deprecated] Use GoAdvanced_DynamicSensitivitySystemValue() instead.
 *
 * Returns the dynamic sensitivity system value.
 *
 * @deprecated
 * @public             @memberof GoMaterial
 * @note               Supported with G1, G2
 * @version            Introduced in firmware 4.1.3.106
 * @param   material   GoMaterial object.
 * @return             Dynamic sensitivity system value.
 */
GoFx(k64f) GoMaterial_DynamicSensitivitySystemValue(GoMaterial material);

/** 
 * [Deprecated] Use GoAdvanced_SetDynamicThreshold() instead.
 *
 * Sets the dynamic threshold.
 *
 * @deprecated
 * @public             @memberof GoMaterial
 * @note               Supported with G1, G2
 * @version            Introduced in firmware 4.1.3.106
 * @param   material   GoMaterial object.
 * @param   value      Dynamic threshold.
 * @return             Operation status.
 */
GoFx(kStatus) GoMaterial_SetDynamicThreshold(GoMaterial material, k32u value);

/** 
 * [Deprecated] Use GoAdvanced_DynamicThresholdLimitMin() instead.
 *
 * Returns the dynamic threshold minimum value limit.
 *
 * @deprecated
 * @public             @memberof GoMaterial
 * @note               Supported with G1, G2
 * @version            Introduced in firmware 4.1.3.106
 * @param   material   GoMaterial object.
 * @return             Dynamic threshold minimum value limit.
 */
GoFx(k32u) GoMaterial_DynamicThresholdLimitMin(GoMaterial material);

/** 
 * [Deprecated] Use GoAdvanced_DynamicThresholdLimitMax() instead.
 *
 * Returns the dynamic threshold maximum value limit.
 *
 * @deprecated
 * @public             @memberof GoMaterial
 * @note               Supported with G1, G2
 * @version            Introduced in firmware 4.1.3.106
 * @param   material   GoMaterial object.
 * @return             Dynamic threshold maximum value limit.
 */
GoFx(k32u) GoMaterial_DynamicThresholdLimitMax(GoMaterial material);

/** 
 * [Deprecated] Use GoAdvanced_DynamicThreshold() instead.
 *
 * Returns the user defined dynamic threshold value.
 *
 * @deprecated
 * @public             @memberof GoMaterial
 * @note               Supported with G1, G2
 * @version            Introduced in firmware 4.1.3.106
 * @param   material   GoMaterial object.
 * @return             The user defined dynamic threshold value.
 */
GoFx(k32u) GoMaterial_DynamicThreshold(GoMaterial material);

/** 
 * [Deprecated] Use GoAdvanced_IsDynamicThresholdUsed() instead.
 *
 * Returns a boolean representing whether or not the user defined dynamic threshold is used by the system.
 *
 * @deprecated
 * @public             @memberof GoMaterial
 * @note               Supported with G1, G2
 * @version            Introduced in firmware 4.1.3.106
 * @param   material   GoMaterial object.
 * @return             Dynamic threshold minimum value limit.
 */
GoFx(kBool) GoMaterial_IsDynamicThresholdUsed(GoMaterial material);

/** 
 * [Deprecated] Use GoAdvanced_DynamicThresholdSystemValue() instead.
 *
 * Returns the dynamic threshold system value.
 *
 * @deprecated
 * @public             @memberof GoMaterial
 * @note               Supported with G1, G2
 * @version            Introduced in firmware 4.1.3.106
 * @param   material   GoMaterial object.
 * @return             Dynamic threshold system value.
 */
GoFx(k32u) GoMaterial_DynamicThresholdSystemValue(GoMaterial material);

/** 
 * [Deprecated] Use GoAdvanced_SetGammaType() instead.
 *
 * Sets the gamma type.
 *
 * @deprecated
 * @public             @memberof GoMaterial
 * @note               Supported with G1, G2
 * @version            Introduced in firmware 4.1.3.106
 * @param   material   GoMaterial object.
 * @param   value      Gamma type.
 * @return             Operation status.
 */
GoFx(kStatus) GoMaterial_SetGammaType(GoMaterial material, GoGammaType value);

/** 
 * [Deprecated] Use GoAdvanced_GammaType() instead.
 *
 * Returns the user defined gamma type.
 *
 * @deprecated
 * @public             @memberof GoMaterial
 * @note               Supported with G1, G2
 * @version            Introduced in firmware 4.1.3.106
 * @param   material   GoMaterial object.
 * @return             User defined gamma type.
 */
GoFx(GoGammaType) GoMaterial_GammaType(GoMaterial material);

/** 
 * [Deprecated] Use GoAdvanced_IsGammaTypeUsed() instead.
 *
 * Returns a boolean representing whether the user defined gamma type is used by the system.
 *
 * @deprecated
 * @public             @memberof GoMaterial
 * @note               Supported with G1, G2
 * @version            Introduced in firmware 4.1.3.106
 * @param   material   GoMaterial object.
 * @return             kTRUE if used and kFALSE otherwise.
 */
GoFx(kBool) GoMaterial_IsGammaTypeUsed(GoMaterial material);

/** 
 * [Deprecated] Use GoAdvanced_GammaTypeSystemValue() instead.
 *
 * Returns the system's gamma type value.
 *
 * @deprecated
 * @public             @memberof GoMaterial
 * @note               Supported with G1, G2
 * @version            Introduced in firmware 4.1.3.106
 * @param   material   GoMaterial object.
 * @return             The system gamma type value.
 */
GoFx(GoGammaType) GoMaterial_GammaTypeSystemValue(GoMaterial material);

/** 
 * [Deprecated] Use GoAdvanced_EnableSensitivityCompensation() instead.
 *
 * Enables or disables senstivity compensation. NOTE: This is only applicable to 
 * 2300 B series sensors.
 *
 * @deprecated
 * @public             @memberof GoMaterial
 * @note               Supported with G1, G2
 * @version            Introduced in firmware 4.3.3.124
 * @param   material   GoMaterial object.
 * @param   value      kTRUE to enable and kFALSE to disable.
 * @return             Operation status.
 */
GoFx(kStatus) GoMaterial_EnableSensitivityCompensation(GoMaterial material, kBool value);

/** 
 * [Deprecated] Use GoAdvanced_SensitivityCompensationEnabled() instead.
 *
 * Returns the user defined sensitivity compensation value.
 *
 * @deprecated
 * @public             @memberof GoMaterial
 * @note               Supported with G1, G2
 * @version            Introduced in firmware 4.3.3.124
 * @param   material   GoMaterial object.
 * @return             User defined sensitivity compensation.
 */
GoFx(kBool) GoMaterial_SensitivityCompensationEnabled(GoMaterial material);

/** 
 * [Deprecated] Use GoAdvanced_IsSensitivityCompensationEnabledUsed() instead.
 *
 * Returns a boolean representing whether the user defined sensitivity compensation is used by the system.
 *
 * @deprecated
 * @public             @memberof GoMaterial
 * @note               Supported with G1, G2
 * @version            Introduced in firmware 4.3.3.124
 * @param   material   GoMaterial object.
 * @return             kTRUE if used and kFALSE otherwise.
 */
GoFx(kBool) GoMaterial_IsSensitivityCompensationEnabledUsed(GoMaterial material);

/** 
 * [Deprecated] Use GoAdvanced_SensitivityCompensationEnabledSystemValue() instead.
 *
 * Returns the system's sensitivity compensation value.
 *
 * @deprecated
 * @public             @memberof GoMaterial
 * @note               Supported with G1, G2
 * @version            Introduced in firmware 4.3.3.124
 * @param   material   GoMaterial object.
 * @return             kTRUE if enabled and kFALSE otherwise.
 */
GoFx(kBool) GoMaterial_SensitivityCompensationEnabledSystemValue(GoMaterial material);

#endif

/** 
 * @file    GoMaterial.c
 * @deprecated
 * @internal
 * Copyright (C) 2016-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <GoSdk/GoMaterial.h>
#include <GoSdk/GoSensor.h>
#include <GoSdk/GoUtils.h>

GoFx(kStatus) GoMaterial_SetType(GoMaterial material, GoMaterialType type)
{
    return GoAdvanced_SetType(material, type);
}

GoFx(GoMaterialType) GoMaterial_Type(GoMaterial material)
{
    return GoAdvanced_Type(material);
}

GoFx(GoMaterialType) GoMaterial_TypeSystemValue(GoMaterial material)
{
    return GoAdvanced_TypeSystemValue(material);
}

GoFx(kBool) GoMaterial_IsTypeUsed(GoMaterial material)
{
    return GoAdvanced_IsTypeUsed(material);
}

GoFx(kStatus) GoMaterial_SetSpotThreshold(GoMaterial material, k32u value)
{
    return GoAdvanced_SetSpotThreshold(material, value);
}

GoFx(k32u) GoMaterial_SpotThreshold(GoMaterial material)
{
    return GoAdvanced_SpotThreshold(material);
}

GoFx(k32u) GoMaterial_SpotThresholdLimitMin(GoMaterial material)
{
    return GoAdvanced_SpotThresholdLimitMin(material);
}

GoFx(k32u) GoMaterial_SpotThresholdLimitMax(GoMaterial material)
{
    return GoAdvanced_SpotThresholdLimitMax(material);
}

GoFx(kBool) GoMaterial_IsSpotThresholdUsed(GoMaterial material)
{
    return GoAdvanced_IsSpotThresholdUsed(material);
}

GoFx(k32u) GoMaterial_SpotThresholdSystemValue(GoMaterial material)
{
    return GoAdvanced_SpotThresholdSystemValue(material);
}

GoFx(kStatus) GoMaterial_SetSpotWidthMax(GoMaterial material, k32u value)
{
    return GoAdvanced_SetSpotWidthMax(material, value);
}

GoFx(k32u) GoMaterial_SpotWidthMax(GoMaterial material)
{
    return GoAdvanced_SpotWidthMax(material);
}

GoFx(k32u) GoMaterial_SpotWidthMaxLimitMin(GoMaterial material)
{
    return GoAdvanced_SpotWidthMaxLimitMin(material);
}

GoFx(k32u) GoMaterial_SpotWidthMaxLimitMax(GoMaterial material)
{
    return GoAdvanced_SpotWidthMaxLimitMax(material);
}

GoFx(kBool) GoMaterial_IsSpotWidthMaxUsed(GoMaterial material)
{
    return GoAdvanced_IsSpotWidthMaxUsed(material);
}

GoFx(k32u) GoMaterial_SpotWidthMaxSystemValue(GoMaterial material)
{
    return GoAdvanced_SpotWidthMaxSystemValue(material);
}

GoFx(kStatus) GoMaterial_SetSpotSelectionType(GoMaterial material, GoSpotSelectionType type)
{
    return GoAdvanced_SetSpotSelectionType(material, type);
}

GoFx(GoSpotSelectionType) GoMaterial_SpotSelectionType(GoMaterial material)
{
    return GoAdvanced_SpotSelectionType(material);
}

GoFx(kBool) GoMaterial_IsSpotSelectionTypeUsed(GoMaterial material)
{
    return GoAdvanced_IsSpotSelectionTypeUsed(material);
}

GoFx(GoSpotSelectionType) GoMaterial_SpotSelectionTypeSystemValue(GoMaterial material)
{
    return GoAdvanced_SpotSelectionTypeSystemValue(material);
}


GoFx(kStatus) GoMaterial_SetCameraGainAnalog(GoMaterial material, k64f value)
{
    return GoAdvanced_SetCameraGainAnalog(material, value);
}

GoFx(k64f) GoMaterial_CameraGainAnalog(GoMaterial material)
{
    return GoAdvanced_CameraGainAnalog(material);
}

GoFx(k64f) GoMaterial_CameraGainAnalogLimitMin(GoMaterial material)
{
    return GoAdvanced_CameraGainAnalogLimitMin(material);
}

GoFx(k64f) GoMaterial_CameraGainAnalogLimitMax(GoMaterial material)
{
    return GoAdvanced_CameraGainAnalogLimitMax(material);
}

GoFx(kBool) GoMaterial_IsCameraGainAnalogUsed(GoMaterial material)
{
    return GoAdvanced_IsCameraGainAnalogUsed(material);
}

GoFx(k64f) GoMaterial_CameraGainAnalogSystemValue(GoMaterial material)
{
    return GoAdvanced_CameraGainAnalogSystemValue(material);
}

GoFx(kStatus) GoMaterial_SetCameraGainDigital(GoMaterial material, k64f value)
{
    return GoAdvanced_SetCameraGainDigital(material, value);
}

GoFx(k64f) GoMaterial_CameraGainDigital(GoMaterial material)
{
    return GoAdvanced_CameraGainDigital(material);
}

GoFx(k64f) GoMaterial_CameraGainDigitalLimitMin(GoMaterial material)
{
    return GoAdvanced_CameraGainDigitalLimitMin(material);
}

GoFx(k64f) GoMaterial_CameraGainDigitalLimitMax(GoMaterial material)
{
    return GoAdvanced_CameraGainDigitalLimitMax(material);
}

GoFx(kBool) GoMaterial_IsCameraGainDigitalUsed(GoMaterial material)
{
    return GoAdvanced_IsCameraGainDigitalUsed(material);
}

GoFx(k64f) GoMaterial_CameraGainDigitalSystemValue(GoMaterial material)
{
    return GoAdvanced_CameraGainDigitalSystemValue(material);
}

GoFx(kStatus) GoMaterial_SetDynamicSensitivity(GoMaterial material, k64f value)
{
    return GoAdvanced_SetDynamicSensitivity(material, value);
}

GoFx(k64f) GoMaterial_DynamicSensitivity(GoMaterial material)
{
    return GoAdvanced_DynamicSensitivity(material);
}

GoFx(k64f) GoMaterial_DynamicSensitivityLimitMin(GoMaterial material)
{
    return GoAdvanced_DynamicSensitivityLimitMin(material);
}

GoFx(k64f) GoMaterial_DynamicSensitivityLimitMax(GoMaterial material)
{
    return GoAdvanced_DynamicSensitivityLimitMax(material);
}

GoFx(kBool) GoMaterial_IsDynamicSensitivityUsed(GoMaterial material)
{
    return GoAdvanced_IsDynamicSensitivityUsed(material);
}

GoFx(k64f) GoMaterial_DynamicSensitivitySystemValue(GoMaterial material)
{
    return GoAdvanced_DynamicSensitivitySystemValue(material);
}

GoFx(kStatus) GoMaterial_SetDynamicThreshold(GoMaterial material, k32u value)
{
    return GoAdvanced_SetDynamicThreshold(material, value);
}

GoFx(k32u) GoMaterial_DynamicThreshold(GoMaterial material)
{
    return GoAdvanced_DynamicThreshold(material);
}

GoFx(k32u) GoMaterial_DynamicThresholdLimitMin(GoMaterial material)
{
    return GoAdvanced_DynamicThresholdLimitMin(material);
}

GoFx(k32u) GoMaterial_DynamicThresholdLimitMax(GoMaterial material)
{
    return GoAdvanced_DynamicThresholdLimitMax(material);
}

GoFx(kBool) GoMaterial_IsDynamicThresholdUsed(GoMaterial material)
{
    return GoAdvanced_IsDynamicThresholdUsed(material);
}

GoFx(k32u) GoMaterial_DynamicThresholdSystemValue(GoMaterial material)
{
    return GoAdvanced_DynamicThresholdSystemValue(material);
}

GoFx(kStatus) GoMaterial_SetGammaType(GoMaterial material, GoGammaType value)
{
    return GoAdvanced_SetGammaType(material, value);
}

GoFx(GoGammaType) GoMaterial_GammaType(GoMaterial material)
{
    return GoAdvanced_GammaType(material);
}

GoFx(kBool) GoMaterial_IsGammaTypeUsed(GoMaterial material)
{
    return GoAdvanced_IsGammaTypeUsed(material);
}

GoFx(GoGammaType) GoMaterial_GammaTypeSystemValue(GoMaterial material)
{
    return GoAdvanced_GammaTypeSystemValue(material);
}

GoFx(kStatus) GoMaterial_EnableSensitivityCompensation(GoMaterial material, kBool value)
{
    return GoAdvanced_EnableSensitivityCompensation(material, value);
}

GoFx(kBool) GoMaterial_SensitivityCompensationEnabled(GoMaterial material)
{
    return GoAdvanced_SensitivityCompensationEnabled(material);
}

GoFx(kBool) GoMaterial_IsSensitivityCompensationEnabledUsed(GoMaterial material)
{
    return GoAdvanced_IsSensitivityCompensationEnabledUsed(material);
}

GoFx(kBool) GoMaterial_SensitivityCompensationEnabledSystemValue(GoMaterial material)
{
    return GoAdvanced_SensitivityCompensationEnabledSystemValue(material);
}

GoFx(kSize) GoMaterial_SpotSelectionTypeOptionCount(GoMaterial material)
{
    return GoAdvanced_SpotSelectionTypeOptionCount(material);
}

GoFx(GoSpotSelectionType) GoMaterial_SpotSelectionTypeOptionAt(GoMaterial material, kSize index)
{
    return GoAdvanced_SpotSelectionTypeOptionAt(material, index);
}

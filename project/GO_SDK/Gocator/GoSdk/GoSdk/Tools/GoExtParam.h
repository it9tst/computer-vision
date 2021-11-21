///@cond private

/**
 * @file    GoExtParam.h
 * @brief   Declares the GoExtParam class.
 *
 * @internal
 * Copyright (C) 2016-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef GO_EXT_VALUE_H
#define GO_EXT_VALUE_H

#include <GoSdk/GoSdkDef.h>
#include <GoSdk/Tools/GoProfileToolUtils.h>
#include <GoSdk/Tools/GoSurfaceToolUtils.h>
#include <kApi/Data/kXml.h>

typedef struct GoExtParamIntOption
{
    k32s value;
    kText64 description;
} GoExtParamIntOption;

typedef struct GoExtParamFloatOption
{
    k64f value;
    kText64 description;
} GoExtParamFloatOption;

typedef kObject GoExtParam;
GoFx(const kChar*) GoExtParam_Label(GoExtParam param);
GoFx(const kChar*) GoExtParam_Id(GoExtParam param);
GoFx(GoExtParamType) GoExtParam_Type(GoExtParam param);
GoFx(kBool) GoExtParam_Used(GoExtParam param);
GoFx(const kChar*) GoExtParam_UnitType(GoExtParam param);

typedef GoExtParam GoExtParamBool;
GoFx(kBool) GoExtParamBool_Value(GoExtParamBool param);
GoFx(kStatus) GoExtParamBool_SetValue(GoExtParamBool param, kBool newVal);

typedef GoExtParam GoExtParamInt;
GoFx(k32s) GoExtParamInt_Value(GoExtParamInt param);
GoFx(kStatus) GoExtParamInt_SetValue(GoExtParamInt param, k32s newVal);
GoFx(kBool) GoExtParamInt_IsValueLimitUsed(GoExtParamInt param);
GoFx(k32s) GoExtParamInt_ValueMin(GoExtParamInt param);
GoFx(k32s) GoExtParamInt_ValueMax(GoExtParamInt param);
GoFx(kSize) GoExtParamInt_OptionCount(GoExtParamInt param);
GoFx(k32s) GoExtParamInt_OptionValueAt(GoExtParamInt param, kSize index);
GoFx(const kChar*) GoExtParamInt_OptionDescriptionAt(GoExtParamInt param, kSize index);

typedef GoExtParam GoExtParamFloat;
GoFx(k64f) GoExtParamFloat_Value(GoExtParamFloat param);
GoFx(kStatus) GoExtParamFloat_SetValue(GoExtParamFloat param, k64f newVal);
GoFx(kBool) GoExtParamFloat_IsValueLimitUsed(GoExtParamFloat param);
GoFx(k64f) GoExtParamFloat_ValueMin(GoExtParamFloat param);
GoFx(k64f) GoExtParamFloat_ValueMax(GoExtParamFloat param);
GoFx(kSize) GoExtParamFloat_OptionCount(GoExtParamFloat param);
GoFx(k64f) GoExtParamFloat_OptionValueAt(GoExtParamFloat param, kSize index);
GoFx(const kChar*) GoExtParamFloat_OptionDescriptionAt(GoExtParamFloat param, kSize index);

typedef GoExtParam GoExtParamFeature;
GoFx(kStatus) GoExtParamFeature_SetFeatureId(GoExtParamFeature param, k32s newVal);
GoFx(k32s) GoExtParamFeature_FeatureId(GoExtParamFeature param);
GoFx(k32s) GoExtParamFeature_ValueMin(GoExtParamFeature param);
GoFx(k32s) GoExtParamFeature_ValueMax(GoExtParamFeature param);
GoFx(kSize) GoExtParamFeature_OptionCount(GoExtParamFeature param);
GoFx(k32s) GoExtParamFeature_OptionValueAt(GoExtParamFeature param, kSize index);
GoFx(const kChar*) GoExtParamFeature_OptionDescriptionAt(GoExtParamFeature param, kSize index);

typedef GoExtParam GoExtParamString;
GoFx(kString) GoExtParamString_Value(GoExtParamString param);

typedef GoExtParam GoExtParamProfileRegion;
GoFx(GoProfileRegion) GoExtParamProfileRegion_Value(GoExtParamProfileRegion param);

typedef GoExtParam GoExtParamSurfaceRegion2d;
GoFx(GoSurfaceRegion2d) GoExtParamSurfaceRegion2d_Value(GoExtParamSurfaceRegion2d param);

typedef GoExtParam GoExtParamSurfaceRegion3d;
GoFx(GoRegion3d) GoExtParamSurfaceRegion3d_Value(GoExtParamSurfaceRegion3d param);

typedef GoExtParam GoExtParamDataInput;
GoFx(GoDataStreamId) GoExtParamDataInput_Value(GoExtParamDataInput param);
GoFx(kStatus) GoExtParamDataInput_SetValue(GoExtParamDataInput param, GoDataStreamId newVal);
GoFx(kSize) GoExtParamDataInput_OptionCount(GoExtParamDataInput param);
GoFx(GoDataStreamId) GoExtParamDataInput_OptionValueAt(GoExtParamDataInput param, kSize index);

typedef GoExtParam GoExtParamPointSetRegion;
GoFx(GoPointSetRegion) GoExtParamPointSetRegion_Value(GoExtParamPointSetRegion param);

#include <GoSdk/Tools/GoExtParam.x.h>

#endif
///@endcond

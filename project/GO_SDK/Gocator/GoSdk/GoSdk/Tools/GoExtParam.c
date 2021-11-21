/**
 * @file    GoExtParam.c
 *
 * @internal
 * Copyright (C) 2016-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <GoSdk/Tools/GoExtParam.h>
#include <GoSdk/GoUtils.h>
#include <GoSdk/GoSensor.h>

// GOC-14479 - Increase the parameter text capacity from 2560 to 16384
// to prevent kERROR_MEMORY, with large drop down menu option.
#define GO_EXT_PARAM_TEXT_CAPACITY (16384)

kBeginValueEx(Go, GoExtParamIntOption)
    kAddField(GoExtParamIntOption, k32s, value)
    kAddField(GoExtParamIntOption, kText64, description)
kEndValueEx()

kBeginValueEx(Go, GoExtParamFloatOption)
    kAddField(GoExtParamFloatOption, k64f, value)
    kAddField(GoExtParamFloatOption, kText64, description)
kEndValueEx()

kBeginVirtualClassEx(Go, GoExtParam)
    kAddFlags(GoExtParam, kTYPE_FLAGS_ABSTRACT)

    kAddVMethod(GoExtParam, kObject, VRelease)
    kAddVMethod(GoExtParam, GoExtParam, VInit)
    kAddVMethod(GoExtParam, GoExtParam, VRead)
    kAddVMethod(GoExtParam, GoExtParam, VWrite)
kEndVirtualClassEx()

GoFx(kStatus) GoExtParam_Construct(GoExtParam* param, kType type, kObject sensor, kAlloc allocator)
{
    kStatus status;

    kCheck(kAlloc_GetObject(allocator, type, param));

    if (!kSuccess(status = kType_VTableT(type, GoExtParam)->VInit(*param, type, sensor, allocator)))
    {
        kAlloc_FreeRef(allocator, param);
    }

    return status;
}

GoFx(kStatus) GoExtParam_VInit(GoTool tool, kType type, kObject sensor, kAlloc alloc)
{
    return kERROR_UNIMPLEMENTED; //this function must be overriden by every tool
}

GoFx(kStatus) GoExtParam_Init(GoExtParam param, kType type, kObject sensor, kAlloc alloc)
{
    kObjR(GoExtParam, param);

    kCheck(kObject_Init(param, type, alloc));
    kZero(obj->sensor);
    obj->paramType[0] = 0;
    obj->label[0] = 0;
    obj->id[0] = 0;
    obj->used = kFALSE;
    kZero(obj->units);

    kCheck(kString_Construct(&obj->units, kNULL, alloc));

    obj->sensor = sensor;

    return kOK;
}

GoFx(kStatus) GoExtParam_VRelease(GoExtParam param)
{
    kObj(GoExtParam, param);

    kCheck(kDestroyRef(&obj->units));

    return kObject_VRelease(param);
}

GoFx(kStatus) GoExtParam_VRead(GoExtParam param, kXml xml, kXmlItem item)
{
    kObj(GoExtParam, param);
    kXml paramXml = kNULL;
    kXmlItem paramItem = kNULL;

    kCheck(kStrCopy(obj->id, kCountOf(obj->id), kXml_ItemName(xml, item)));
    kCheck(kXml_AttrText(xml, item, "label", obj->label, kCountOf(obj->label)));
    kCheck(kXml_AttrText(xml, item, "type", obj->paramType, kCountOf(obj->paramType)));

    //optional reads
    if (kXml_AttrExists(xml, item, "used"))
    {
        kCheck(kXml_AttrBool(xml, item, "used", &obj->used));
    }
    else  // parameters without "used" attribute are interpretted as used = true
    {
        obj->used = kTRUE;
    }

    if (kXml_AttrExists(xml, item, "units"))
    {
        kCheck(kXml_AttrString(xml, item, "units", obj->units));
    }

    return kOK;
}

GoFx(kStatus) GoExtParam_VWrite(GoExtParam param, kXml xml, kXmlItem item)
{
    kObj(GoExtParam, param);

    kCheck(kXml_SetAttrText(xml, item, "label", obj->label));
    kCheck(kXml_SetAttrText(xml, item, "type", obj->paramType));

    return kOK;
}

GoFx(kStatus) GoExtParam_Read(GoExtParam param, kXml xml, kXmlItem item)
{
    kObj(GoExtParam, param);
    return kCast(GoExtParamVTable*, xkObject_VTable(param))->VRead(param, xml, item);
}

GoFx(kStatus) GoExtParam_Write(GoExtParam param, kXml xml, kXmlItem item)
{
    kObj(GoExtParam, param);
    return kCast(GoExtParamVTable*, xkObject_VTable(param))->VWrite(param, xml, item);
}

GoFx(const kChar*) GoExtParam_Label(GoExtParam param)
{
    kObj(GoExtParam, param);

    GoSensor_SyncConfig(obj->sensor);

    return obj->label;
}

GoFx(const kChar*) GoExtParam_Id(GoExtParam param)
{
    kObj(GoExtParam, param);

    GoSensor_SyncConfig(obj->sensor);

    return obj->id;
}

GoFx(GoExtParamType) GoExtParam_Type(GoExtParam param)
{
    kObj(GoExtParam, param);

    GoSensor_SyncConfig(obj->sensor);

    if (kStrCompare(obj->paramType, "Int") == 0)
    {
        return GO_EXT_PARAM_TYPE_INT;
    }
    else if (kStrCompare(obj->paramType, "Float") == 0)
    {
        return GO_EXT_PARAM_TYPE_FLOAT;
    }
    else if (kStrCompare(obj->paramType, "Bool") == 0)
    {
        return GO_EXT_PARAM_TYPE_BOOL;
    }
    else if (kStrCompare(obj->paramType, "String") == 0)
    {
        return GO_EXT_PARAM_TYPE_STRING;
    }
    else if (kStrCompare(obj->paramType, "ProfileRegion") == 0)
    {
        return GO_EXT_PARAM_TYPE_PROFILE_REGION;
    }
    else if (kStrCompare(obj->paramType, "SurfaceRegion2d") == 0)
    {
        return GO_EXT_PARAM_TYPE_SURFACE_REGION_2D;
    }
    else if (kStrCompare(obj->paramType, "SurfaceRegion3d") == 0)
    {
        return GO_EXT_PARAM_TYPE_SURFACE_REGION_3D;
    }
    else if (kStrCompare(obj->paramType, "GeometricFeature") == 0)
    {
        return GO_EXT_PARAM_TYPE_GEOMETRIC_FEATURE;
    }
    else if (kStrCompare(obj->paramType, "Measurement") == 0)
    {
        return GO_EXT_PARAM_TYPE_MEASUREMENT;
    }
    else if (kStrCompare(obj->paramType, "DataInput") == 0)
    {
        return GO_EXT_PARAM_TYPE_DATA_INPUT;
    }
    else if (kStrCompare(obj->paramType, "PointSetRegion") == 0)
    {
        return GO_EXT_PARAM_TYPE_POINT_SET_REGION;
    }

    return GO_EXT_PARAM_TYPE_UNKNOWN;
}

GoFx(kBool) GoExtParam_Used(GoExtParam param)
{
    kObj(GoExtParam, param);

    GoSensor_SyncConfig(obj->sensor);

    return obj->used;
}

GoFx(const kChar*) GoExtParam_UnitType(GoExtParam param)
{
    kObj(GoExtParam, param);

    GoSensor_SyncConfig(obj->sensor);

    return kString_Chars(obj->units);
}

kBeginClassEx(Go, GoExtParamInt)
kAddVMethod(GoExtParamInt, kObject, VRelease)
kAddVMethod(GoExtParamInt, GoExtParam, VInit)
kAddVMethod(GoExtParamInt, GoExtParam, VRead)
kAddVMethod(GoExtParamInt, GoExtParam, VWrite)
kEndClassEx()

GoFx(kStatus) GoExtParamInt_Construct(GoExtParamInt* param, kObject sensor, kAlloc allocator)
{
    kStatus status;

    kCheck(kAlloc_GetObject(allocator, kTypeOf(GoExtParamInt), param));

    if (!kSuccess(status = GoExtParamInt_VInit(*param, kTypeOf(GoExtParamInt), sensor, allocator)))
    {
        kAlloc_FreeRef(allocator, param);
    }

    return status;
}

GoFx(kStatus) GoExtParamInt_VInit(GoExtParamInt param, kType type, kObject sensor, kAlloc alloc)
{
    kObjR(GoExtParamInt, param);
    kStatus exception;

    kCheck(GoExtParam_Init(param, type, sensor, alloc));
    obj->value = 0;
    kZero(obj->options);

    obj->valLimitsUsed = kFALSE;
    obj->valMin = k32S_MIN;
    obj->valMax = k32S_MAX;

    kTry
    {
        kTest(kArrayList_Construct(&obj->options, kTypeOf(GoExtParamIntOption), 0, alloc));
    }
    kCatch(&exception)
    {
        GoExtParamInt_VRelease(param);
        kEndCatch(exception);
    }

    return kOK;
}

GoFx(kStatus) GoExtParamInt_VRelease(GoExtParamInt param)
{
    kObj(GoExtParamInt, param);

    kDisposeRef(&obj->options);

    return GoExtParam_VRelease(param);
}

GoFx(kStatus) GoExtParamInt_VRead(GoExtParamInt param, kXml xml, kXmlItem item)
{
    kObj(GoExtParamInt, param);

    kCheck(GoExtParam_VRead(param, xml, item));
    kCheck(kXml_Item32s(xml, item, &obj->value));

    if (kXml_AttrExists(xml, item, "options"))
    {
        kCheck(GoExtParamInt_ParseList(xml, item, obj->options));
    }

    if (kXml_AttrExists(xml, item, "min"))
    {
        kCheck(kXml_Attr32s(xml, item, "min", &obj->valMin));
        obj->valLimitsUsed = kTRUE;
    }

    if (kXml_AttrExists(xml, item, "max"))
    {
        kCheck(kXml_Attr32s(xml, item, "max", &obj->valMax));
        obj->valLimitsUsed = kTRUE;
    }

    return kOK;
}

GoFx(kStatus) GoExtParamInt_VWrite(GoExtParamInt param, kXml xml, kXmlItem item)
{
    kObj(GoExtParamInt, param);

    kCheck(GoExtParam_VWrite(param, xml, item));
    kCheck(kXml_SetItem32s(xml, item, obj->value));

    return kOK;
}

GoFx(k32s) GoExtParamInt_Value(GoExtParamInt param)
{
    kObj(GoExtParamInt, param);

    GoSensor_SyncConfig(obj->base.sensor);

    return obj->value;
}

GoFx(kStatus) GoExtParamInt_SetValue(GoExtParamInt param, k32s newVal)
{
    kObj(GoExtParamInt, param);

    kCheckState(GoSensor_IsConfigurable(obj->base.sensor));
    kCheck(GoSensor_CacheConfig(obj->base.sensor));

    if (GoExtParamInt_IsValueLimitUsed(param))
    {
        kCheckArgs(GoUtils_MinMax_(newVal, GoExtParamInt_ValueMin(param), GoExtParamInt_ValueMax(param)));
    }

    if (GoExtParamInt_OptionCount(param) > 0)
    {
        kCheckArgs(kTRUE == GoExtUtils_InParamOptionList32s(obj->options, newVal));
    }

    obj->value = newVal;
    kCheck(GoSensor_SetConfigModified(obj->base.sensor));

    return kOK;
}

GoFx(kBool) GoExtParamInt_IsValueLimitUsed(GoExtParamInt param)
{
    kObj(GoExtParamInt, param);

    GoSensor_SyncConfig(obj->base.sensor);

    return obj->valLimitsUsed;
}

GoFx(k32s) GoExtParamInt_ValueMin(GoExtParamInt param)
{
    kObj(GoExtParamInt, param);

    GoSensor_SyncConfig(obj->base.sensor);

    return obj->valMin;
}

GoFx(k32s) GoExtParamInt_ValueMax(GoExtParamInt param)
{
    kObj(GoExtParamInt, param);

    GoSensor_SyncConfig(obj->base.sensor);

    return obj->valMax;
}

GoFx(kSize) GoExtParamInt_OptionCount(GoExtParamInt param)
{
    kObj(GoExtParamInt, param);

    GoSensor_SyncConfig(obj->base.sensor);

    return kArrayList_Count(obj->options);
}

GoFx(k32s) GoExtParamInt_OptionValueAt(GoExtParamInt param, kSize index)
{
    kObj(GoExtParamInt, param);
    GoExtParamIntOption* option;

    kAssert(index < kArrayList_Count(obj->options));

    GoSensor_SyncConfig(obj->base.sensor);

    option = kArrayList_AtT(obj->options, index, GoExtParamIntOption);

    return option->value;
}

GoFx(const kChar*) GoExtParamInt_OptionDescriptionAt(GoExtParamInt param, kSize index)
{
    kObj(GoExtParamInt, param);
    GoExtParamIntOption* option;

    kAssert(index < kArrayList_Count(obj->options));

    GoSensor_SyncConfig(obj->base.sensor);

    option = kArrayList_AtT(obj->options, index, GoExtParamIntOption);

    return option->description;
}

kBeginClassEx(Go, GoExtParamFeature)
kAddVMethod(GoExtParamFeature, kObject, VRelease)
kAddVMethod(GoExtParamFeature, GoExtParam, VInit)
kAddVMethod(GoExtParamFeature, GoExtParam, VRead)
kAddVMethod(GoExtParamFeature, GoExtParam, VWrite)
kEndClassEx()

GoFx(kStatus) GoExtParamFeature_Construct(GoExtParamFeature* param, kObject sensor, kAlloc allocator)
{
    kStatus status;

    kCheck(kAlloc_GetObject(allocator, kTypeOf(GoExtParamInt), param));

    if (!kSuccess(status = GoExtParamFeature_VInit(*param, kTypeOf(GoExtParamFeature), sensor, allocator)))
    {
        kAlloc_FreeRef(allocator, param);
    }

    return status;
}

GoFx(kStatus) GoExtParamFeature_VInit(GoExtParamFeature param, kType type, kObject sensor, kAlloc alloc)
{
    kObjR(GoExtParamFeature, param);
    kStatus exception;
    kCheck(GoExtParam_Init(param, type, sensor, alloc));
    obj->featureId = 0;
    kZero(obj->options);
    obj->valMin = 0;
    obj->valMax = 0;
    obj->valLimitsUsed = kFALSE;

    kTry
    {
        kTest(kArrayList_Construct(&obj->options, kTypeOf(GoExtParamIntOption), 0, alloc));
    }
    kCatch(&exception)
    {
        GoExtParamFeature_VRelease(param);
        //GoExtParamSurfaceRegion2d_VRelease(param);
        kEndCatch(exception);
    }


    obj->valLimitsUsed = kFALSE;
    obj->valMin = k32S_MIN;
    obj->valMax = k32S_MAX;


    return kOK;
}

GoFx(kStatus) GoExtParamFeature_VRelease(GoExtParamFeature param)
{
    kObj(GoExtParamFeature, param);

    kDisposeRef(&obj->options);

    return GoExtParam_VRelease(param);
}


GoFx(kStatus) GoExtParamFeature_VRead(GoExtParamFeature param, kXml xml, kXmlItem item)
{

    kObj(GoExtParamFeature, param);

    kCheck(GoExtParam_VRead(param, xml, item));

    kCheck(kXml_Item32s(xml, item, &obj->featureId));

    if (kXml_AttrExists(xml, item, "options"))
    {
        kCheck(GoExtParamInt_ParseList(xml, item, obj->options));
    }

    if (kXml_AttrExists(xml, item, "min"))
    {
        kCheck(kXml_Attr32s(xml, item, "min", &obj->valMin));
        obj->valLimitsUsed = kTRUE;
    }

    if (kXml_AttrExists(xml, item, "max"))
    {
        kCheck(kXml_Attr32s(xml, item, "max", &obj->valMax));
        obj->valLimitsUsed = kTRUE;
    }

    return kOK;
}

GoFx(kStatus) GoExtParamFeature_VWrite(GoExtParamFeature param, kXml xml, kXmlItem item)
{
    kObj(GoExtParamFeature, param);

    kCheck(GoExtParam_VWrite(param, xml, item));

    kCheck(kXml_SetItem32s(xml, item, obj->featureId));

    return kOK;
}

GoFx(k32s) GoExtParamFeature_FeatureId(GoExtParamFeature param)
{
    kObj(GoExtParamFeature, param);

    GoSensor_SyncConfig(obj->base.sensor);

    return obj->featureId;
}

GoFx(kStatus) GoExtParamFeature_SetFeatureId(GoExtParamFeature param, k32s newVal)
{
    kObj(GoExtParamFeature, param);

    kCheckState(GoSensor_IsConfigurable(obj->base.sensor));
    kCheck(GoSensor_CacheConfig(obj->base.sensor));
    obj->featureId = newVal;
    kCheck(GoSensor_SetConfigModified(obj->base.sensor));

    return kOK;
}

GoFx(k32s) GoExtParamFeature_ValueMin(GoExtParamFeature param)
{
    kObj(GoExtParamFeature, param);

    GoSensor_SyncConfig(obj->base.sensor);

    return obj->valMin;
}

GoFx(k32s) GoExtParamFeature_ValueMax(GoExtParamFeature param)
{
    kObj(GoExtParamFeature, param);

    GoSensor_SyncConfig(obj->base.sensor);

    return obj->valMax;
}

GoFx(kSize) GoExtParamFeature_OptionCount(GoExtParamFeature param)
{
    kObj(GoExtParamFeature, param);

    GoSensor_SyncConfig(obj->base.sensor);

    return kArrayList_Count(obj->options);
}

GoFx(k32s) GoExtParamFeature_OptionValueAt(GoExtParamFeature param, kSize index)
{
    kObj(GoExtParamFeature, param);
    GoExtParamIntOption* option;

    kAssert(index < kArrayList_Count(obj->options));

    GoSensor_SyncConfig(obj->base.sensor);

    option = kArrayList_AtT(obj->options, index, GoExtParamIntOption);

    return option->value;
}

GoFx(const kChar*) GoExtParamFeature_OptionDescriptionAt(GoExtParamFeature param, kSize index)
{
    kObj(GoExtParamFeature, param);
    GoExtParamIntOption* option;

    kAssert(index < kArrayList_Count(obj->options));

    GoSensor_SyncConfig(obj->base.sensor);

    option = kArrayList_AtT(obj->options, index, GoExtParamIntOption);

    return option->description;
}

kBeginClassEx(Go, GoExtParamDataInput)
kAddVMethod(GoExtParamDataInput, kObject, VRelease)
kAddVMethod(GoExtParamDataInput, GoExtParam, VInit)
kAddVMethod(GoExtParamDataInput, GoExtParam, VRead)
kAddVMethod(GoExtParamDataInput, GoExtParam, VWrite)
kEndClassEx()

GoFx(kStatus) GoExtParamDataInput_Construct(GoExtParamDataInput* param, kObject sensor, kAlloc allocator)
{
    kStatus status;

    kCheck(kAlloc_GetObject(allocator, kTypeOf(GoExtParamDataInput), param));

    if (!kSuccess(status = GoExtParamDataInput_VInit(*param, kTypeOf(GoExtParamDataInput), sensor, allocator)))
    {
        kAlloc_FreeRef(allocator, param);
    }

    return status;
}

GoFx(kStatus) GoExtParamDataInput_VInit(GoExtParamDataInput param, kType type, kObject sensor, kAlloc alloc)
{
    kObjR(GoExtParamDataInput, param);
    kStatus exception;
    kCheck(GoExtParam_Init(param, type, sensor, alloc));
    kZero(obj->streamId);
    kZero(obj->options);

    kTry
    {
        kTest(kArrayList_Construct(&obj->options, kTypeOf(GoDataStreamId), 0, alloc));
    }
    kCatch(&exception)
    {
        GoExtParamDataInput_VRelease(param);
        kEndCatch(exception);
    }

    return kOK;
}

GoFx(kStatus) GoExtParamDataInput_VRelease(GoExtParamDataInput param)
{
    kObj(GoExtParamDataInput, param);

    kDisposeRef(&obj->options);

    return GoExtParam_VRelease(param);
}


GoFx(kStatus) GoExtParamDataInput_VRead(GoExtParamDataInput param, kXml xml, kXmlItem item)
{

    kObj(GoExtParamDataInput, param);
    kText128 streamIdStr;

    kCheck(GoExtParam_VRead(param, xml, item));

    kCheck(kXml_ItemText(xml, item, streamIdStr, kCountOf(streamIdStr)));

    kCheck(GoUtils_ParseHelperDataStreamId(streamIdStr, strlen(streamIdStr), &obj->streamId));

    if (kXml_AttrExists(xml, item, "options"))
    {
        kCheck(GoExtParamDataInput_ParseList(xml, item, obj->options));
    }

    return kOK;
}

GoFx(kStatus) GoExtParamDataInput_VWrite(GoExtParamDataInput param, kXml xml, kXmlItem item)
{
    kObj(GoExtParamDataInput, param);
    kText128 streamId;

    kCheck(GoExtParam_VWrite(param, xml, item));

    if (obj->streamId.step == GO_DATA_STEP_NONE)    // special case if NONE, write out single "-1"
    {
        kStrPrintf(streamId, kCountOf(streamId), "-1");
    }
    else
    {
        kStrPrintf(streamId, kCountOf(streamId), "%d%c%d%c%d", obj->streamId.step, GO_UTILS_STREAM_ID_SEPARATOR, obj->streamId.id, GO_UTILS_STREAM_ID_SEPARATOR, obj->streamId.source);
    }

    kCheck(kXml_SetItemText(xml, item, streamId));

    return kOK;
}

GoFx(GoDataStreamId) GoExtParamDataInput_Value(GoExtParamDataInput param)
{
    kObj(GoExtParamDataInput, param);

    GoSensor_SyncConfig(obj->base.sensor);

    return obj->streamId;
}

GoFx(kStatus) GoExtParamDataInput_SetValue(GoExtParamDataInput param, GoDataStreamId newVal)
{
    kObj(GoExtParamDataInput, param);

    kCheckState(GoSensor_IsConfigurable(obj->base.sensor));
    kCheck(GoSensor_CacheConfig(obj->base.sensor));
    obj->streamId = newVal;
    kCheck(GoSensor_SetConfigModified(obj->base.sensor));

    return kOK;
}

GoFx(kSize) GoExtParamDataInput_OptionCount(GoExtParamDataInput param)
{
    kObj(GoExtParamDataInput, param);

    GoSensor_SyncConfig(obj->base.sensor);

    return kArrayList_Count(obj->options);
}

GoFx(GoDataStreamId) GoExtParamDataInput_OptionValueAt(GoExtParamDataInput param, kSize index)
{
    kObj(GoExtParamDataInput, param);
    GoDataStreamId* option;

    kAssert(index < kArrayList_Count(obj->options));

    GoSensor_SyncConfig(obj->base.sensor);

    option = kArrayList_AtT(obj->options, index, GoDataStreamId);

    return *option;
}

kBeginClassEx(Go, GoExtParamFloat)
kAddVMethod(GoExtParamFloat, kObject, VRelease)
kAddVMethod(GoExtParamFloat, GoExtParam, VInit)
kAddVMethod(GoExtParamFloat, GoExtParam, VRead)
kAddVMethod(GoExtParamFloat, GoExtParam, VWrite)
kEndClassEx()

GoFx(kStatus) GoExtParamFloat_Construct(GoExtParamFloat* param, kObject sensor, kAlloc allocator)
{
    kStatus status;

    kCheck(kAlloc_GetObject(allocator, kTypeOf(GoExtParamFloat), param));

    if (!kSuccess(status = GoExtParamFloat_VInit(*param, kTypeOf(GoExtParamFloat), sensor, allocator)))
    {
        kAlloc_FreeRef(allocator, param);
    }

    return status;
}

GoFx(kStatus) GoExtParamFloat_VInit(GoExtParamFloat param, kType type, kObject sensor, kAlloc alloc)
{
    kObjR(GoExtParamFloat, param);
    kStatus exception;

    kCheck(GoExtParam_Init(param, type, sensor, alloc));
    obj->value = 0.0;
    kZero(obj->options);

    obj->valLimitsUsed = kFALSE;
    obj->valMin = k64F_MIN;
    obj->valMax = k64F_MAX;

    kTry
    {
        kTest(kArrayList_Construct(&obj->options, kTypeOf(GoExtParamFloatOption), 0, alloc));
    }
    kCatch(&exception)
    {
        GoExtParamFloat_VRelease(param);
        kEndCatch(exception);
    }

    return kOK;
}

GoFx(kStatus) GoExtParamFloat_VRelease(GoExtParamFloat param)
{
    kObj(GoExtParamFloat, param);

    kCheck(kDisposeRef(&obj->options));

    return GoExtParam_VRelease(param);
}

GoFx(kStatus) GoExtParamFloat_VRead(GoExtParamFloat param, kXml xml, kXmlItem item)
{
    kObj(GoExtParamFloat, param);

    kCheck(GoExtParam_VRead(param, xml, item));
    kCheck(kXml_Item64f(xml, item, &obj->value));

    if (kXml_AttrExists(xml, item, "options"))
    {
        kCheck(GoExtParamFloat_ParseList(xml, item, obj->options));
    }

    if (kXml_AttrExists(xml, item, "min"))
    {
        kCheck(kXml_Attr64f(xml, item, "min", &obj->valMin));
        obj->valLimitsUsed = kTRUE;
    }

    if (kXml_AttrExists(xml, item, "max"))
    {
        kCheck(kXml_Attr64f(xml, item, "max", &obj->valMax));
        obj->valLimitsUsed = kTRUE;
    }

    return kOK;
}

GoFx(kStatus) GoExtParamFloat_VWrite(GoExtParamFloat param, kXml xml, kXmlItem item)
{
    kObj(GoExtParamFloat, param);

    kCheck(GoExtParam_VWrite(param, xml, item));
    kCheck(kXml_SetItem64f(xml, item, obj->value));

    return kOK;
}

GoFx(k64f) GoExtParamFloat_Value(GoExtParamFloat param)
{
    kObj(GoExtParamFloat, param);

    GoSensor_SyncConfig(obj->base.sensor);

    return obj->value;
}

GoFx(kStatus) GoExtParamFloat_SetValue(GoExtParamFloat param, k64f newVal)
{
    kObj(GoExtParamFloat, param);

    kCheckState(GoSensor_IsConfigurable(obj->base.sensor));
    kCheck(GoSensor_CacheConfig(obj->base.sensor));

    if (GoExtParamFloat_IsValueLimitUsed(param))
    {
        kCheckArgs(GoUtils_MinMax_(newVal, GoExtParamFloat_ValueMin(param), GoExtParamFloat_ValueMax(param)));
    }

    if (GoExtParamFloat_OptionCount(param) > 0)
    {
        kCheckArgs(kTRUE == GoExtUtils_InParamOptionList64f(obj->options, newVal));
    }

    obj->value = newVal;
    kCheck(GoSensor_SetConfigModified(obj->base.sensor));

    return kOK;
}

GoFx(kBool) GoExtParamFloat_IsValueLimitUsed(GoExtParamFloat param)
{
    kObj(GoExtParamFloat, param);

    GoSensor_SyncConfig(obj->base.sensor);

    return obj->valLimitsUsed;
}

GoFx(k64f) GoExtParamFloat_ValueMin(GoExtParamFloat param)
{
    kObj(GoExtParamFloat, param);

    GoSensor_SyncConfig(obj->base.sensor);

    return obj->valMin;
}

GoFx(k64f) GoExtParamFloat_ValueMax(GoExtParamFloat param)
{
    kObj(GoExtParamFloat, param);

    GoSensor_SyncConfig(obj->base.sensor);

    return obj->valMax;
}

GoFx(kSize) GoExtParamFloat_OptionCount(GoExtParamFloat param)
{
    kObj(GoExtParamFloat, param);

    GoSensor_SyncConfig(obj->base.sensor);

    return kArrayList_Count(obj->options);
}

GoFx(k64f) GoExtParamFloat_OptionValueAt(GoExtParamFloat param, kSize index)
{
    kObj(GoExtParamFloat, param);

    kAssert(index < kArrayList_Count(obj->options));

    GoSensor_SyncConfig(obj->base.sensor);

    return kArrayList_AtT(obj->options, index, GoExtParamFloatOption)->value;
}

GoFx(const kChar*) GoExtParamFloat_OptionDescriptionAt(GoExtParamFloat param, kSize index)
{
    kObj(GoExtParamFloat, param);

    kAssert(index < kArrayList_Count(obj->options));

    GoSensor_SyncConfig(obj->base.sensor);

    return (kArrayList_AtT(obj->options, index, GoExtParamFloatOption))->description;
}

kBeginClassEx(Go, GoExtParamBool)
kAddVMethod(GoExtParamBool, kObject, VRelease)
kAddVMethod(GoExtParamBool, GoExtParam, VInit)
kAddVMethod(GoExtParamBool, GoExtParam, VRead)
kAddVMethod(GoExtParamBool, GoExtParam, VWrite)
kEndClassEx()

GoFx(kStatus) GoExtParamBool_Construct(GoExtParamBool* param, kObject sensor, kAlloc allocator)
{
    kStatus status;

    kCheck(kAlloc_GetObject(allocator, kTypeOf(GoExtParamBool), param));

    if (!kSuccess(status = GoExtParamBool_VInit(*param, kTypeOf(GoExtParamBool), sensor, allocator)))
    {
        kAlloc_FreeRef(allocator, param);
    }

    return status;
}

GoFx(kStatus) GoExtParamBool_VInit(GoExtParamBool param, kType type, kObject sensor, kAlloc alloc)
{
    kObjR(GoExtParamBool, param);

    kCheck(GoExtParam_Init(param, type, sensor, alloc));
    obj->value = kFALSE;

    return kOK;
}

GoFx(kStatus) GoExtParamBool_VRelease(GoExtParamBool param)
{
    kObj(GoExtParamBool, param);

    return GoExtParam_VRelease(param);
}

GoFx(kStatus) GoExtParamBool_VRead(GoExtParamBool param, kXml xml, kXmlItem item)
{
    kObj(GoExtParamBool, param);

    kCheck(GoExtParam_VRead(param, xml, item));
    kCheck(kXml_ItemBool(xml, item, &obj->value));

    return kOK;
}

GoFx(kStatus) GoExtParamBool_VWrite(GoExtParamBool param, kXml xml, kXmlItem item)
{
    kObj(GoExtParamBool, param);

    kCheck(GoExtParam_VWrite(param, xml, item));
    kCheck(kXml_SetItemBool(xml, item, obj->value));

    return kOK;
}

GoFx(kBool) GoExtParamBool_Value(GoExtParamBool param)
{
    kObj(GoExtParamBool, param);

    GoSensor_SyncConfig(obj->base.sensor);

    return obj->value;
}

GoFx(kStatus) GoExtParamBool_SetValue(GoExtParamBool param, kBool newVal)
{
    kObj(GoExtParamBool, param);

    kCheckState(GoSensor_IsConfigurable(obj->base.sensor));
    kCheck(GoSensor_CacheConfig(obj->base.sensor));
    obj->value = newVal;
    kCheck(GoSensor_SetConfigModified(obj->base.sensor));

    return kOK;
}

kBeginClassEx(Go, GoExtParamString)
kAddVMethod(GoExtParamString, kObject, VRelease)
kAddVMethod(GoExtParamString, GoExtParam, VInit)
kAddVMethod(GoExtParamString, GoExtParam, VRead)
kAddVMethod(GoExtParamString, GoExtParam, VWrite)
kEndClassEx()

GoFx(kStatus) GoExtParamString_Construct(GoExtParamString* param, kObject sensor, kAlloc allocator)
{
    kStatus status;

    kCheck(kAlloc_GetObject(allocator, kTypeOf(GoExtParamString), param));

    if (!kSuccess(status = GoExtParamString_VInit(*param, kTypeOf(GoExtParamString), sensor, allocator)))
    {
        kAlloc_FreeRef(allocator, param);
    }

    return status;
}

GoFx(kStatus) GoExtParamString_VInit(GoExtParamString param, kType type, kObject sensor, kAlloc alloc)
{
    kObjR(GoExtParamString, param);
    kStatus exception;

    kCheck(GoExtParam_Init(param, type, sensor, alloc));
    kZero(obj->value);

    kTry
    {
        kTest(kString_Construct(&obj->value, "", alloc));
    }
    kCatch(&exception)
    {
        GoExtParamString_VRelease(param);
        kEndCatch(exception);
    }

    return kOK;
}

GoFx(kStatus) GoExtParamString_VRelease(GoExtParamString param)
{
    kObj(GoExtParamString, param);

    kCheck(kDestroyRef(&obj->value));

    return GoExtParam_VRelease(param);
}

GoFx(kStatus) GoExtParamString_VRead(GoExtParamString param, kXml xml, kXmlItem item)
{
    kObj(GoExtParamString, param);

    kCheck(GoExtParam_VRead(param, xml, item));
    kCheck(kXml_ItemString(xml, item, obj->value));

    return kOK;
}

GoFx(kStatus) GoExtParamString_VWrite(GoExtParamString param, kXml xml, kXmlItem item)
{
    kObj(GoExtParamString, param);

    kCheck(GoExtParam_VWrite(param, xml, item));
    kCheck(kXml_SetItemText(xml, item, kString_Chars(obj->value)));

    return kOK;
}

GoFx(kString) GoExtParamString_Value(GoExtParamString param)
{
    kObj(GoExtParamString, param);

    GoSensor_SyncConfig(obj->base.sensor);

    return obj->value;
}

kBeginClassEx(Go, GoExtParamProfileRegion)
kAddVMethod(GoExtParamProfileRegion, kObject, VRelease)
kAddVMethod(GoExtParamProfileRegion, GoExtParam, VInit)
kAddVMethod(GoExtParamProfileRegion, GoExtParam, VRead)
kAddVMethod(GoExtParamProfileRegion, GoExtParam, VWrite)
kEndClassEx()

GoFx(kStatus) GoExtParamProfileRegion_Construct(GoExtParamProfileRegion* param, kObject sensor, kAlloc allocator)
{
    kStatus status;

    kCheck(kAlloc_GetObject(allocator, kTypeOf(GoExtParamProfileRegion), param));

    if (!kSuccess(status = GoExtParamProfileRegion_VInit(*param, kTypeOf(GoExtParamProfileRegion), sensor, allocator)))
    {
        kAlloc_FreeRef(allocator, param);
    }

    return status;
}

GoFx(kStatus) GoExtParamProfileRegion_VInit(GoExtParamProfileRegion param, kType type, kObject sensor, kAlloc alloc)
{
    kObjR(GoExtParamProfileRegion, param);
    kStatus exception;

    kCheck(GoExtParam_Init(param, type, sensor, alloc));
    obj->value = kNULL;

    kTry
    {
        kTest(GoProfileRegion_Construct(&obj->value, sensor, alloc));
    }
    kCatch(&exception)
    {
        GoExtParamProfileRegion_VRelease(param);
        kEndCatch(exception);
    }

    return kOK;
}

GoFx(kStatus) GoExtParamProfileRegion_VRelease(GoExtParamProfileRegion param)
{
    kObj(GoExtParamProfileRegion, param);

    kCheck(kDestroyRef(&obj->value));

    return GoExtParam_VRelease(param);
}

GoFx(kStatus) GoExtParamProfileRegion_VRead(GoExtParamProfileRegion param, kXml xml, kXmlItem item)
{
    kObj(GoExtParamProfileRegion, param);

    kCheck(GoExtParam_VRead(param, xml, item));
    kCheck(GoProfileRegion_Read(obj->value, xml, item));

    return kOK;
}

GoFx(kStatus) GoExtParamProfileRegion_VWrite(GoExtParamProfileRegion param, kXml xml, kXmlItem item)
{
    kObj(GoExtParamProfileRegion, param);

    kCheck(GoExtParam_VWrite(param, xml, item));
    kCheck(GoProfileRegion_Write(obj->value, xml, item));

    return kOK;
}

GoFx(GoProfileRegion) GoExtParamProfileRegion_Value(GoExtParamProfileRegion param)
{
    kObj(GoExtParamProfileRegion, param);

    GoSensor_SyncConfig(obj->base.sensor);

    return obj->value;
}

kBeginClassEx(Go, GoExtParamSurfaceRegion2d)
kAddVMethod(GoExtParamSurfaceRegion2d, kObject, VRelease)
kAddVMethod(GoExtParamSurfaceRegion2d, GoExtParam, VInit)
kAddVMethod(GoExtParamSurfaceRegion2d, GoExtParam, VRead)
kAddVMethod(GoExtParamSurfaceRegion2d, GoExtParam, VWrite)
kEndClassEx()

GoFx(kStatus) GoExtParamSurfaceRegion2d_Construct(GoExtParamSurfaceRegion2d* param, kObject sensor, kAlloc allocator)
{
    kStatus status;

    kCheck(kAlloc_GetObject(allocator, kTypeOf(GoExtParamSurfaceRegion2d), param));

    if (!kSuccess(status = GoExtParamSurfaceRegion2d_VInit(*param, kTypeOf(GoExtParamSurfaceRegion2d), sensor, allocator)))
    {
        kAlloc_FreeRef(allocator, param);
    }

    return status;
}

GoFx(kStatus) GoExtParamSurfaceRegion2d_VInit(GoExtParamSurfaceRegion2d param, kType type, kObject sensor, kAlloc alloc)
{
    kObjR(GoExtParamSurfaceRegion2d, param);
    kStatus exception;

    kCheck(GoExtParam_Init(param, type, sensor, alloc));
    obj->value = kNULL;

    kTry
    {
        kTest(GoSurfaceRegion2d_Construct(&obj->value, sensor, alloc));
    }
    kCatch(&exception)
    {
        GoExtParamSurfaceRegion2d_VRelease(param);
        kEndCatch(exception);
    }

    return kOK;
}

GoFx(kStatus) GoExtParamSurfaceRegion2d_VRelease(GoExtParamSurfaceRegion2d param)
{
    kObj(GoExtParamSurfaceRegion2d, param);

    kCheck(kDestroyRef(&obj->value));

    return GoExtParam_VRelease(param);
}

GoFx(kStatus) GoExtParamSurfaceRegion2d_VRead(GoExtParamSurfaceRegion2d param, kXml xml, kXmlItem item)
{
    kObj(GoExtParamSurfaceRegion2d, param);

    kCheck(GoExtParam_VRead(param, xml, item));
    kCheck(GoSurfaceRegion2d_Read(obj->value, xml, item));

    return kOK;
}

GoFx(kStatus) GoExtParamSurfaceRegion2d_VWrite(GoExtParamSurfaceRegion2d param, kXml xml, kXmlItem item)
{
    kObj(GoExtParamSurfaceRegion2d, param);

    kCheck(GoExtParam_VWrite(param, xml, item));
    kCheck(GoSurfaceRegion2d_Write(obj->value, xml, item));

    return kOK;
}

GoFx(GoSurfaceRegion2d) GoExtParamSurfaceRegion2d_Value(GoExtParamSurfaceRegion2d param)
{
    kObj(GoExtParamSurfaceRegion2d, param);

    GoSensor_SyncConfig(obj->base.sensor);

    return obj->value;
}

kBeginClassEx(Go, GoExtParamSurfaceRegion3d)
kAddVMethod(GoExtParamSurfaceRegion3d, kObject, VRelease)
kAddVMethod(GoExtParamSurfaceRegion3d, GoExtParam, VInit)
kAddVMethod(GoExtParamSurfaceRegion3d, GoExtParam, VRead)
kAddVMethod(GoExtParamSurfaceRegion3d, GoExtParam, VWrite)
kEndClassEx()

GoFx(kStatus) GoExtParamSurfaceRegion3d_Construct(GoExtParamSurfaceRegion3d* param, kObject sensor, kAlloc allocator)
{
    kStatus status;

    kCheck(kAlloc_GetObject(allocator, kTypeOf(GoExtParamSurfaceRegion3d), param));

    if (!kSuccess(status = GoExtParamSurfaceRegion3d_VInit(*param, kTypeOf(GoExtParamSurfaceRegion3d), sensor, allocator)))
    {
        kAlloc_FreeRef(allocator, param);
    }

    return status;
}

GoFx(kStatus) GoExtParamSurfaceRegion3d_VInit(GoExtParamSurfaceRegion3d param, kType type, kObject sensor, kAlloc alloc)
{
    kObjR(GoExtParamSurfaceRegion3d, param);
    kStatus exception;

    kCheck(GoExtParam_Init(param, type, sensor, alloc));
    obj->value = kNULL;

    kTry
    {
        kTest(GoRegion3d_Construct(&obj->value, sensor, alloc));
    }
    kCatch(&exception)
    {
        GoExtParamSurfaceRegion3d_VRelease(param);
        kEndCatch(exception);
    }

    return kOK;
}

GoFx(kStatus) GoExtParamSurfaceRegion3d_VRelease(GoExtParamSurfaceRegion3d param)
{
    kObj(GoExtParamSurfaceRegion3d, param);

    kCheck(kDestroyRef(&obj->value));

    return GoExtParam_VRelease(param);
}

GoFx(kStatus) GoExtParamSurfaceRegion3d_VRead(GoExtParamSurfaceRegion3d param, kXml xml, kXmlItem item)
{
    kObj(GoExtParamSurfaceRegion3d, param);

    kCheck(GoExtParam_VRead(param, xml, item));
    kCheck(GoRegion3d_Read(obj->value, xml, item));

    return kOK;
}

GoFx(kStatus) GoExtParamSurfaceRegion3d_VWrite(GoExtParamSurfaceRegion3d param, kXml xml, kXmlItem item)
{
    kObj(GoExtParamSurfaceRegion3d, param);

    kCheck(GoExtParam_VWrite(param, xml, item));
    kCheck(GoRegion3d_Write(obj->value, xml, item));

    return kOK;
}

GoFx(GoRegion3d) GoExtParamSurfaceRegion3d_Value(GoExtParamSurfaceRegion3d param)
{
    kObj(GoExtParamSurfaceRegion3d, param);

    GoSensor_SyncConfig(obj->base.sensor);

    return obj->value;
}

GoFx(kType) GoExtUtils_GetKType(const kChar* paramType)
{
    if (kStrCompare(paramType, "Bool") == 0)
    {
        return kTypeOf(GoExtParamBool);
    }
    else if (kStrCompare(paramType, "Int") == 0)
    {
        return kTypeOf(GoExtParamInt);
    }
    else if (kStrCompare(paramType, "Float") == 0)
    {
        return kTypeOf(GoExtParamFloat);
    }
    else if (kStrCompare(paramType, "String") == 0)
    {
        return kTypeOf(GoExtParamString);
    }
    else if (kStrCompare(paramType, "ProfileRegion") == 0)
    {
        return kTypeOf(GoExtParamProfileRegion);
    }
    else if (kStrCompare(paramType, "SurfaceRegion2d") == 0)
    {
        return kTypeOf(GoExtParamSurfaceRegion2d);
    }
    else if (kStrCompare(paramType, "SurfaceRegion3d") == 0)
    {
        return kTypeOf(GoExtParamSurfaceRegion3d);
    }
    else if (kStrCompare(paramType, "GeometricFeature") == 0)
    {
        return kTypeOf(GoExtParamFeature);
    }
    else if (kStrCompare(paramType, "DataInput") == 0)
    {
        return kTypeOf(GoExtParamDataInput);
    }
    else if (kStrCompare(paramType, "PointSetRegion") == 0)
    {
        return kTypeOf(GoExtParamPointSetRegion);
    }

    return kNULL;
}

GoFx(kBool) GoExtUtils_InParamOptionList64f(kArrayList list, k64f value)
{
    kSize i;

    for (i = 0; i < kArrayList_Count(list); i++)
    {
        GoExtParamFloatOption* option = kArrayList_AtT(list, i, GoExtParamFloatOption);

        if (option->value == value)
        {
            return kTRUE;
        }
    }

    return kFALSE;
}

GoFx(kBool) GoExtUtils_InParamOptionList32s(kArrayList list, k32s value)
{
    kSize i;

    for (i = 0; i < kArrayList_Count(list); i++)
    {
        GoExtParamIntOption* option = kArrayList_AtT(list, i, GoExtParamIntOption);

        if (option->value == value)
        {
            return kTRUE;
        }
    }

    return kFALSE;
}

GoFx(kStatus) GoOptionList_ParseHelperText(const kChar* text, kSize length, kText64 value)
{
    if(length == 0)
    {
        return kERROR_PARAMETER;
    }
    kCheck(length < 64);
    strncpy(value, text, length);
    value[length] = 0;

    return kOK;
}

GoFx(kStatus) GoExtParamInt_ParseList(kXml xml, kXmlItem item, kArrayList list)
{
    kChar valuesText[GO_EXT_PARAM_TEXT_CAPACITY];
    kChar descriptionsText[GO_EXT_PARAM_TEXT_CAPACITY];
    const kChar* valReadIt = "";
    const kChar* descReadIt = "";
    const kChar* valSeparator = "";
    const kChar* descSeparator = "";
    GoExtParamIntOption option;

    kCheck(kMemSet(&option, 0, sizeof(option)));

    kCheck(kXml_AttrText(xml, item, "options", valuesText, kCountOf(valuesText)));
    valReadIt = valuesText;
    if (kXml_AttrExists(xml, item, "optionNames"))
    {
        kCheckArgs(kXml_AttrExists(xml, item, "optionNames"));
        kCheck(kXml_AttrText(xml, item, "optionNames", descriptionsText, kCountOf(descriptionsText)));
        descReadIt = descriptionsText;
    }

    kCheck(kArrayList_Purge(list));
    kCheck(kArrayList_Allocate(list, kTypeOf(GoExtParamIntOption), 0));

    while (!kIsNull(valSeparator = strchr(valReadIt, ','))
        && !kIsNull(descSeparator = strchr(descReadIt, ',')))
    {
        GoExtParamIntOption loopOption;

        if (kSuccess(GoOptionList_ParseHelper32s(valReadIt, valSeparator - valReadIt, &loopOption.value))
            && kSuccess(GoOptionList_ParseHelperText(descReadIt, descSeparator - descReadIt, loopOption.description)))
        {
             kCheck(kArrayList_AddT(list, &loopOption));
        }
        valReadIt = valSeparator + 1;
        descReadIt = descSeparator + 1;
    }

    if (kSuccess(GoOptionList_ParseHelper32s(valReadIt, strlen(valReadIt), &option.value)))
    {
        if(strlen(descReadIt) > 0)
        {
            kSuccess(GoOptionList_ParseHelperText(descReadIt, strlen(descReadIt), option.description));
        }

        kCheck(kArrayList_AddT(list, &option));
    }


    return kOK;
}

GoFx(kStatus) GoExtParamFloat_ParseList(kXml xml, kXmlItem item, kArrayList list)
{
    kChar valuesText[GO_EXT_PARAM_TEXT_CAPACITY];
    kChar descriptionsText[GO_EXT_PARAM_TEXT_CAPACITY];
    const kChar* valReadIt = kNULL;
    const kChar* descReadIt = kNULL;
    const kChar* valSeparator = kNULL;
    const kChar* descSeparator = kNULL;
    GoExtParamFloatOption option;

    kCheck(kMemSet(&option, 0, sizeof(option)));

    kCheck(kXml_AttrText(xml, item, "options", valuesText, kCountOf(valuesText)));
    valReadIt = valuesText;

    kCheckArgs(kXml_AttrExists(xml, item, "optionNames"));
    kCheck(kXml_AttrText(xml, item, "optionNames", descriptionsText, kCountOf(descriptionsText)));
    descReadIt = descriptionsText;

    kCheck(kArrayList_Purge(list));
    kCheck(kArrayList_Allocate(list, kTypeOf(GoExtParamFloatOption), 0));

    while (!kIsNull(valSeparator = strchr(valReadIt, ','))
        && !kIsNull(descSeparator = strchr(descReadIt, ',')))
    {
        if (kSuccess(GoOptionList_ParseHelper64f(valReadIt, valSeparator - valReadIt, &option.value))
            && kSuccess(GoOptionList_ParseHelperText(descReadIt, descSeparator - descReadIt, option.description)))
        {
            kCheck(kArrayList_AddT(list, &option));
        }
        valReadIt = valSeparator + 1;
        descReadIt = descSeparator + 1;
    }

    if (kSuccess(GoOptionList_ParseHelper64f(valReadIt, strlen(valReadIt), &option.value))
        && kSuccess(GoOptionList_ParseHelperText(descReadIt, strlen(descReadIt), option.description)))
    {
        kCheck(kArrayList_AddT(list, &option));
    }

    return kOK;
}

GoFx(kStatus) GoExtParamDataInput_ParseList(kXml xml, kXmlItem item, kArrayList list)
{
    kChar valuesText[GO_EXT_PARAM_TEXT_CAPACITY];
    const kChar* valReadIt = kNULL;
    const kChar* valSeparator = kNULL;
    GoDataStreamId option;

    kCheck(kXml_AttrText(xml, item, "options", valuesText, kCountOf(valuesText)));
    valReadIt = valuesText;

    kCheck(kArrayList_Purge(list));
    kCheck(kArrayList_Allocate(list, kTypeOf(GoDataStreamId), 0));

    while (!kIsNull(valSeparator = strchr(valReadIt, ',')))
    {
        if (kSuccess(GoUtils_ParseHelperDataStreamId(valReadIt, valSeparator - valReadIt, &option)))
        {
            kCheck(kArrayList_AddT(list, &option));
        }
        valReadIt = valSeparator + 1;
    }

    if (kSuccess(GoUtils_ParseHelperDataStreamId(valReadIt, strlen(valReadIt), &option)))
    {
        kCheck(kArrayList_AddT(list, &option));
    }

    return kOK;
}

kBeginClassEx(Go, GoExtParamPointSetRegion)
kAddVMethod(GoExtParamPointSetRegion, kObject, VRelease)
kAddVMethod(GoExtParamPointSetRegion, GoExtParam, VInit)
kAddVMethod(GoExtParamPointSetRegion, GoExtParam, VRead)
kAddVMethod(GoExtParamPointSetRegion, GoExtParam, VWrite)
kEndClassEx()

GoFx(kStatus) GoExtParamPointSetRegion_Construct(GoExtParamPointSetRegion* param, kObject sensor, kAlloc allocator)
{
    kStatus status;

    kCheck(kAlloc_GetObject(allocator, kTypeOf(GoExtParamPointSetRegion), param));

    if (!kSuccess(status = GoExtParamPointSetRegion_VInit(*param, kTypeOf(GoExtParamPointSetRegion), sensor, allocator)))
    {
        kAlloc_FreeRef(allocator, param);
    }

    return status;
}

GoFx(kStatus) GoExtParamPointSetRegion_VInit(GoExtParamPointSetRegion param, kType type, kObject sensor, kAlloc alloc)
{
    kObjR(GoExtParamPointSetRegion, param);
    kStatus exception;

    kCheck(GoExtParam_Init(param, type, sensor, alloc));
    obj->value = kNULL;

    kTry
    {
        kTest(GoPointSetRegion_Construct(&obj->value, sensor, alloc));
    }
    kCatch(&exception)
    {
        GoExtParamPointSetRegion_VRelease(param);
        kEndCatch(exception);
    }

    return kOK;
}

GoFx(kStatus) GoExtParamPointSetRegion_VRelease(GoExtParamPointSetRegion param)
{
    kObj(GoExtParamPointSetRegion, param);

    kCheck(kDestroyRef(&obj->value));

    return GoExtParam_VRelease(param);
}

GoFx(kStatus) GoExtParamPointSetRegion_VRead(GoExtParamPointSetRegion param, kXml xml, kXmlItem item)
{
    kObj(GoExtParamPointSetRegion, param);

    kCheck(GoExtParam_VRead(param, xml, item));
    kCheck(GoPointSetRegion_Read(obj->value, xml, item));

    return kOK;
}

GoFx(kStatus) GoExtParamPointSetRegion_VWrite(GoExtParamPointSetRegion param, kXml xml, kXmlItem item)
{
    kObj(GoExtParamPointSetRegion, param);

    kCheck(GoExtParam_VWrite(param, xml, item));
    kCheck(GoPointSetRegion_Write(obj->value, xml, item));

    return kOK;
}

GoFx(GoPointSetRegion) GoExtParamPointSetRegion_Value(GoExtParamPointSetRegion param)
{
    kObj(GoExtParamPointSetRegion, param);

    GoSensor_SyncConfig(obj->base.sensor);

    return obj->value;
}

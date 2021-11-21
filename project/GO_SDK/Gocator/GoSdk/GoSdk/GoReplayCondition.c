/**
* @file    GoReplayCondition.c
*
* @internal
* Copyright (C) 2016-2021 by LMI Technologies Inc.
* Licensed under the MIT License.
* Redistributed files must retain the above copyright notice.
*/
#include <GoSdk/GoReplayCondition.h>
#include <GoSdk/GoSensor.h>
#include <GoSdk/GoUtils.h>

kBeginVirtualClassEx(Go, GoReplayCondition)
    kAddFlags(GoReplayCondition, kTYPE_FLAGS_ABSTRACT)
    kAddVMethod(GoReplayCondition, kObject, VRelease)
    kAddVMethod(GoReplayCondition, GoReplayCondition, VInit)
    kAddVMethod(GoReplayCondition, GoReplayCondition, VName)
    kAddVMethod(GoReplayCondition, GoReplayCondition, VRead)
    kAddVMethod(GoReplayCondition, GoReplayCondition, VWrite)
kEndVirtualClassEx()

GoFx(kStatus) GoReplayCondition_VInit(GoReplayCondition condition, kType type, kObject sensor, kAlloc alloc)
{
    kObjR(GoReplayCondition, condition);

    kCheck(kObject_Init(condition, type, alloc));
    obj->enabled = kFALSE;

    obj->sensor = sensor;

    return kOK;
}

GoFx(kStatus) GoReplayCondition_VRelease(GoReplayCondition condition)
{
    return kObject_VRelease(condition);
}

GoFx(const kChar*) GoReplayCondition_VName()
{
    return "";
}

GoFx(kStatus) GoReplayCondition_Init(GoReplayCondition condition, kType type, kObject sensor, kAlloc alloc)
{
    GoReplayConditionVTable* vtable = kType_VTableT(type, GoReplayCondition);

    return vtable->VInit(condition, type, sensor, alloc);
}

GoFx(const kChar*) GoReplayCondition_Name(kType type)
{
    GoReplayConditionVTable* vtable = kType_VTableT(type, GoReplayCondition);

    return vtable->VName();
}

GoFx(kStatus) GoReplayCondition_VRead(GoReplayCondition condition, kXml xml, kXmlItem root)
{
    kObj(GoReplayCondition, condition);

    kCheck(kXml_ChildBool(xml, root, "Enabled", &obj->enabled));

    return kOK;
}

GoFx(kStatus) GoReplayCondition_VWrite(GoReplayCondition condition, kXml xml, kXmlItem root)
{
    kObj(GoReplayCondition, condition);

    kCheck(kXml_SetChildBool(xml, root, "Enabled", obj->enabled));
    
    return kOK;
}

GoFx(kBool) GoReplayCondition_Enabled(GoReplayCondition condition)
{
    kObj(GoReplayCondition, condition);

    GoSensor_SyncConfig(obj->sensor);

    return obj->enabled;
}

GoFx(kStatus) GoReplayCondition_Enable(GoReplayCondition condition, kBool enable)
{
    kObj(GoReplayCondition, condition);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));
    obj->enabled = enable;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

kBeginClassEx(Go, GoReplayAnyMeasurement)
    kAddVMethod(GoReplayAnyMeasurement, kObject, VRelease)
    kAddVMethod(GoReplayAnyMeasurement, GoReplayCondition, VInit)
    kAddVMethod(GoReplayAnyMeasurement, GoReplayCondition, VName)
    kAddVMethod(GoReplayAnyMeasurement, GoReplayCondition, VRead)
    kAddVMethod(GoReplayAnyMeasurement, GoReplayCondition, VWrite)
kEndClassEx()

GoFx(kStatus) GoReplayAnyMeasurement_Construct(GoReplayAnyMeasurement* condition, kObject sensor, kAlloc alloc)
{
    kStatus status;
    kType type = kTypeOf(GoReplayAnyMeasurement);

    alloc = kAlloc_Fallback(alloc);
    kCheck(kAlloc_GetObject(alloc, type, condition));

    if (!kSuccess(status = GoReplayCondition_Init(*condition, type, sensor, alloc)))
    {
        kAlloc_FreeRef(alloc, condition);
    }

    return status;
}

GoFx(kStatus) GoReplayAnyMeasurement_VInit(GoReplayAnyMeasurement condition, kType type, kObject sensor, kAlloc alloc)
{
    kObjR(GoReplayAnyMeasurement, condition);
    kStatus exception = kOK;

    kCheck(GoReplayCondition_VInit(condition, type, sensor, alloc));
    obj->result = GO_REPLAY_MEASUREMENT_RESULT_PASS;

    kTry
    {
    }
    kCatch(&exception)
    {
        kObject_VRelease(condition);
        kEndCatch(exception);
    }

    return kOK;
}

GoFx(kStatus) GoReplayAnyMeasurement_VRelease(GoReplayAnyMeasurement condition)
{
    return GoReplayCondition_VRelease(condition);
}

GoFx(const kChar*) GoReplayAnyMeasurement_VName()
{
    return "AnyMeasurement";
}

GoFx(kStatus) GoReplayAnyMeasurement_VRead(GoReplayAnyMeasurement condition, kXml xml, kXmlItem root)
{
    kObj(GoReplayAnyMeasurement, condition);

    kCheck(GoReplayCondition_VRead(condition, xml, root));

    kCheck(kXml_Child32s(xml, root, "Result", &obj->result));

    return kOK;
}

GoFx(kStatus) GoReplayAnyMeasurement_VWrite(GoReplayAnyMeasurement condition, kXml xml, kXmlItem root)
{
    kObj(GoReplayAnyMeasurement, condition);

    kCheck(GoReplayCondition_VWrite(condition, xml, root));

    kCheck(kXml_SetChild32s(xml, root, "Result", obj->result));

    return kOK;
}

GoFx(kStatus) GoReplayAnyMeasurement_SetResult(GoReplayAnyMeasurement condition, GoReplayMeasurementResult result)
{
    kObj(GoReplayAnyMeasurement, condition);

    kCheckState(GoSensor_IsConfigurable(obj->base.sensor));
    kCheck(GoSensor_CacheConfig(obj->base.sensor));
    obj->result = result;
    kCheck(GoSensor_SetConfigModified(obj->base.sensor));

    return kOK;
}

GoFx(GoReplayMeasurementResult) GoReplayAnyMeasurement_Result(GoReplayAnyMeasurement condition)
{
    kObj(GoReplayAnyMeasurement, condition);

    GoSensor_SyncConfig(obj->base.sensor);

    return obj->result;
}

kBeginClassEx(Go, GoReplayAnyData)
    kAddVMethod(GoReplayAnyData, kObject, VRelease)
    kAddVMethod(GoReplayAnyData, GoReplayCondition, VInit)
    kAddVMethod(GoReplayAnyData, GoReplayCondition, VName)
    kAddVMethod(GoReplayAnyData, GoReplayCondition, VRead)
    kAddVMethod(GoReplayAnyData, GoReplayCondition, VWrite)
kEndClassEx()


GoFx(kStatus) GoReplayAnyData_Construct(GoReplayAnyData* condition, kObject sensor, kAlloc alloc)
{
    kStatus status;
    kType type = kTypeOf(GoReplayAnyData);

    alloc = kAlloc_Fallback(alloc);
    kCheck(kAlloc_GetObject(alloc, type, condition));

    if (!kSuccess(status = GoReplayCondition_Init(*condition, type, sensor, alloc)))
    {
        kAlloc_FreeRef(alloc, condition);
    }

    return status;
}

GoFx(kStatus) GoReplayAnyData_VInit(GoReplayAnyData condition, kType type, kObject sensor, kAlloc alloc)
{
    kObjR(GoReplayAnyData, condition);
    kStatus exception = kOK;

    kCheck(GoReplayCondition_VInit(condition, type, sensor, alloc));

    kTry
    {
        obj->rangeCountCase = GO_REPLAY_RANGE_COUNT_CASE_AT_ABOVE;
        obj->rangeCountThreshold = 1;
    }
    kCatch(&exception)
    {
        kObject_VRelease(condition);
        kEndCatch(exception);
    }

    return kOK;
}

GoFx(kStatus) GoReplayAnyData_VRelease(GoReplayAnyData condition)
{
    return GoReplayCondition_VRelease(condition);
}

GoFx(const kChar*) GoReplayAnyData_VName()
{
    return "AnyData";
}

GoFx(kStatus) GoReplayAnyData_VRead(GoReplayAnyData condition, kXml xml, kXmlItem root)
{
    kObj(GoReplayAnyData, condition);

    kCheck(GoReplayCondition_VRead(condition, xml, root));

    kCheck(kXml_Child32s(xml, root, "RangeCountCase", &obj->rangeCountCase));
    kCheck(kXml_Child32u(xml, root, "RangeCountThreshold", &obj->rangeCountThreshold));

    return kOK;
}

GoFx(kStatus) GoReplayAnyData_VWrite(GoReplayAnyData condition, kXml xml, kXmlItem root)
{
    kObj(GoReplayAnyData, condition);

    kCheck(GoReplayCondition_VWrite(condition, xml, root));

    kCheck(kXml_SetChild32s(xml, root, "RangeCountCase", obj->rangeCountCase));
    kCheck(kXml_SetChild32u(xml, root, "RangeCountThreshold", obj->rangeCountThreshold));

    return kOK;
}

GoFx(kStatus) GoReplayAnyData_SetRangeCountCase(GoReplayAnyData condition, GoReplayRangeCountCase rangeCase)
{
    kObj(GoReplayAnyData, condition);

    kCheckState(GoSensor_IsConfigurable(obj->base.sensor));
    kCheck(GoSensor_CacheConfig(obj->base.sensor));
    obj->rangeCountCase = rangeCase;
    kCheck(GoSensor_SetConfigModified(obj->base.sensor));
    
    return kOK;
}

GoFx(GoReplayRangeCountCase) GoReplayAnyData_RangeCountCase(GoReplayAnyData condition)
{
    kObj(GoReplayAnyData, condition);

    GoSensor_SyncConfig(obj->base.sensor);

    return obj->rangeCountCase;
}

GoFx(kStatus) GoReplayAnyData_SetRangeCountThreshold(GoReplayAnyData condition, k32u threshold)
{
    kObj(GoReplayAnyData, condition);

    kCheckState(GoSensor_IsConfigurable(obj->base.sensor));
    kCheck(GoSensor_CacheConfig(obj->base.sensor));
    obj->rangeCountThreshold = threshold;
    kCheck(GoSensor_SetConfigModified(obj->base.sensor));

    return kOK;
}

GoFx(k32u) GoReplayAnyData_RangeCountThreshold(GoReplayAnyData condition)
{
    kObj(GoReplayAnyData, condition);

    GoSensor_SyncConfig(obj->base.sensor);

    return obj->rangeCountThreshold;
}

kBeginClassEx(Go, GoReplayMeasurement)
    kAddVMethod(GoReplayMeasurement, kObject, VRelease)
    kAddVMethod(GoReplayMeasurement, GoReplayCondition, VInit)
    kAddVMethod(GoReplayMeasurement, GoReplayCondition, VName)
    kAddVMethod(GoReplayMeasurement, GoReplayCondition, VRead)
    kAddVMethod(GoReplayMeasurement, GoReplayCondition, VWrite)
kEndClassEx()

GoFx(kStatus) GoReplayMeasurement_Construct(GoReplayMeasurement* condition, kObject sensor, kAlloc alloc)
{
    kStatus status;
    kType type = kTypeOf(GoReplayMeasurement);

    alloc = kAlloc_Fallback(alloc);
    kCheck(kAlloc_GetObject(alloc, type, condition));

    if (!kSuccess(status = GoReplayCondition_Init(*condition, type, sensor, alloc)))
    {
        kAlloc_FreeRef(alloc, condition);
    }

    return status;
}

GoFx(kStatus) GoReplayMeasurement_VInit(GoReplayMeasurement condition, kType type, kObject sensor, kAlloc alloc)
{
    kObjR(GoReplayMeasurement, condition);
    kStatus exception = kOK;

    kCheck(GoReplayCondition_VInit(condition, type, sensor, alloc));
    obj->result = GO_REPLAY_MEASUREMENT_RESULT_PASS;
    obj->id = 0;

    kTry
    {
    }
    kCatch(&exception)
    {
        kObject_VRelease(condition);
        kEndCatch(exception);
    }

    return kOK;
}

GoFx(kStatus) GoReplayMeasurement_VRelease(GoReplayMeasurement condition)
{
    return GoReplayCondition_VRelease(condition);
}

GoFx(const kChar*) GoReplayMeasurement_VName()
{
    return "Measurement";
}

GoFx(kStatus) GoReplayMeasurement_VRead(GoReplayMeasurement condition, kXml xml, kXmlItem root)
{
    kObj(GoReplayMeasurement, condition);

    kCheck(GoReplayCondition_VRead(condition, xml, root));

    kCheck(kXml_Child32s(xml, root, "Result", &obj->result));

    // This is for backward compatibility
    if (kXml_ChildExists(xml, root, "Selection"))
    {
        kCheck(kXml_Child32u(xml, root, "Selection", &obj->id));
    }

    if (kXml_ChildExists(xml, root, "Ids"))
    {
        kCheck(kXml_Child32u(xml, root, "Ids", &obj->id));
    }

    return kOK;
}

GoFx(kStatus) GoReplayMeasurement_VWrite(GoReplayMeasurement condition, kXml xml, kXmlItem root)
{
    kObj(GoReplayMeasurement, condition);

    kCheck(GoReplayCondition_VWrite(condition, xml, root));

    kCheck(kXml_SetChild32s(xml, root, "Result", obj->result));
    kCheck(kXml_SetChild32u(xml, root, "Ids", obj->id));

    return kOK;
}

GoFx(kStatus) GoReplayMeasurement_SetResult(GoReplayMeasurement condition, GoReplayMeasurementResult result)
{
    kObj(GoReplayMeasurement, condition);

    kCheckState(GoSensor_IsConfigurable(obj->base.sensor));
    kCheck(GoSensor_CacheConfig(obj->base.sensor));
    obj->result = result;
    kCheck(GoSensor_SetConfigModified(obj->base.sensor));

    return kOK;
}

GoFx(GoReplayMeasurementResult) GoReplayMeasurement_Result(GoReplayMeasurement condition)
{
    kObj(GoReplayMeasurement, condition);

    GoSensor_SyncConfig(obj->base.sensor);

    return obj->result;
}

GoFx(kSize) GoReplayMeasurement_IdCount(GoReplayMeasurement condition)
{
    kObj(GoReplayMeasurement, condition);

    return 1;
}

GoFx(k32u) GoReplayMeasurement_IdAt(GoReplayMeasurement condition, kSize index)
{
    kObj(GoReplayMeasurement, condition);

    if (index != 0)
        return 0;

    GoSensor_SyncConfig(obj->base.sensor);

    return obj->id;
}

GoFx(kStatus) GoReplayMeasurement_SetIdAt(GoReplayMeasurement condition, kSize index, k32u id)
{
    kObj(GoReplayMeasurement, condition);

    kCheckArgs(index == 0);

    kCheckState(GoSensor_IsConfigurable(obj->base.sensor));
    kCheck(GoSensor_CacheConfig(obj->base.sensor));
    obj->id = id;
    kCheck(GoSensor_SetConfigModified(obj->base.sensor));

    return kOK;
}

/**
* @file    GoRecordingFilter.c
*
* @internal
* Copyright (C) 2016-2021 by LMI Technologies Inc.
* Licensed under the MIT License.
* Redistributed files must retain the above copyright notice.
*/
#include <GoSdk/GoRecordingFilter.h>
#include <GoSdk/GoSensor.h>
#include <GoSdk/GoUtils.h>

kBeginClassEx(Go, GoRecordingFilter)
    kAddVMethod(GoRecordingFilter, kObject, VRelease)
kEndClassEx()

GoFx(kStatus) GoRecordingFilter_Construct(GoRecordingFilter* filter, kObject sensor, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback(allocator);
    kStatus status;

    kCheck(kAlloc_GetObject(alloc, kTypeOf(GoRecordingFilter), filter));

    if (!kSuccess(status = GoRecordingFilter_Init(*filter, kTypeOf(GoRecordingFilter), sensor, alloc)))
    {
        kAlloc_FreeRef(alloc, filter);
    }

    return status;
}

GoFx(kStatus) GoRecordingFilter_Init(GoRecordingFilter filter, kType type, kObject sensor, kAlloc alloc)
{
    kObjR(GoRecordingFilter, filter);
    kStatus status = kOK;

    kCheck(kObject_Init(filter, type, alloc));
    kZero(obj->sensor);
    obj->conditionCombineType = GO_REPLAY_COMBINE_TYPE_ANY;
    obj->anyMeasurement = kNULL;
    obj->anyData = kNULL;
    obj->measurement = kNULL;

    kTry
    {
        kTest(GoReplayAnyMeasurement_Construct(&obj->anyMeasurement, sensor, alloc));
        kTest(GoReplayAnyData_Construct(&obj->anyData, sensor, alloc));
        kTest(GoReplayMeasurement_Construct(&obj->measurement, sensor, alloc));
    }
    kCatchEx(&status)
    {
        kDisposeRef(&obj->anyMeasurement);
        kDisposeRef(&obj->anyData);
        kDisposeRef(&obj->measurement);

        kEndCatchEx(status);
    }
    kFinallyEx
    {
        kEndFinallyEx();
    }

    obj->sensor = sensor;

    return kOK;
}

GoFx(kStatus) GoRecordingFilter_VRelease(GoRecordingFilter filter)
{
    kObj(GoRecordingFilter, filter);

    kDisposeRef(&obj->anyMeasurement);
    kDisposeRef(&obj->anyData);
    kDisposeRef(&obj->measurement);

    kCheck(kObject_VRelease(filter));

    return kOK;
}

GoFx(kStatus) GoRecordingFilter_Read(GoRecordingFilter filter, kXml xml, kXmlItem item)
{
    kObj(GoRecordingFilter, filter);
    kXmlItem conditionsItem = kNULL;
    kXmlItem conditionItem = kNULL;

    kCheck(kXml_Child32s(xml, item, "ConditionCombineType", &obj->conditionCombineType));
        
    kCheck(!kIsNull(conditionsItem = kXml_Child(xml, item, "Conditions")));

    kCheck(!kIsNull(conditionItem = kXml_Child(xml, conditionsItem, "AnyMeasurement")));
    kCheck(GoReplayAnyMeasurement_VRead(obj->anyMeasurement, xml, conditionItem));

    kCheck(!kIsNull(conditionItem = kXml_Child(xml, conditionsItem, "AnyData")));
    kCheck(GoReplayAnyData_VRead(obj->anyData, xml, conditionItem));

    kCheck(!kIsNull(conditionItem = kXml_Child(xml, conditionsItem, "Measurement")));
    kCheck(GoReplayMeasurement_VRead(obj->measurement, xml, conditionItem));
    
    return kOK;
}

GoFx(kStatus) GoRecordingFilter_Write(GoRecordingFilter filter, kXml xml, kXmlItem item)
{
    kObj(GoRecordingFilter, filter);
    kXmlItem conditionsItem = kNULL;
    kXmlItem conditionItem = kNULL;

    kCheck(kXml_SetChild32s(xml, item, "ConditionCombineType", obj->conditionCombineType));

    kCheck(kXml_AddItem(xml, item, "Conditions", &conditionsItem));

    kCheck(kXml_AddItem(xml, conditionsItem, "AnyMeasurement", &conditionItem));
    kCheck(GoReplayAnyMeasurement_VWrite(obj->anyMeasurement, xml, conditionItem));

    kCheck(kXml_AddItem(xml, conditionsItem, "AnyData", &conditionItem));
    kCheck(GoReplayAnyData_VWrite(obj->anyData, xml, conditionItem));

    kCheck(kXml_AddItem(xml, conditionsItem, "Measurement", &conditionItem));
    kCheck(GoReplayMeasurement_VWrite(obj->measurement, xml, conditionItem));

    return kOK;
}

GoFx(GoReplayCombineType) GoRecordingFilter_ConditionCombineType(GoRecordingFilter filter)
{
    kObj(GoRecordingFilter, filter);

    GoSensor_SyncConfig(obj->sensor);

    return obj->conditionCombineType;
}

GoFx(kStatus) GoRecordingFilter_SetConditionCombineType(GoRecordingFilter filter, GoReplayCombineType conditionCombineType)
{
    kObj(GoRecordingFilter, filter);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));
    obj->conditionCombineType = conditionCombineType;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(GoReplayAnyMeasurement) GoRecordingFilter_AnyMeasurement(GoRecordingFilter filter)
{
    kObj(GoRecordingFilter, filter);

    return obj->anyMeasurement;
}

GoFx(GoReplayAnyData) GoRecordingFilter_AnyData(GoRecordingFilter filter)
{
    kObj(GoRecordingFilter, filter);

    return obj->anyData;
}

GoFx(GoReplayMeasurement) GoRecordingFilter_Measurement(GoRecordingFilter filter)
{
    kObj(GoRecordingFilter, filter);

    return obj->measurement;
}

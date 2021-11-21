/**
* @file    GoReplayCondition.x.h
*
* @internal
* Copyright (C) 2016-2021 by LMI Technologies Inc.
* Licensed under the MIT License.
* Redistributed files must retain the above copyright notice.
*/
#ifndef GO_REPLAY_CONDITION_X_H
#define GO_REPLAY_CONDITION_X_H

#include <kApi/Data/kXml.h>

typedef struct GoReplayConditionClass
{
    kObjectClass base;
    kObject sensor;

    kBool enabled;

} GoReplayConditionClass;

typedef struct GoReplayConditionVTable 
{
    kObjectVTable base;

    kStatus(kCall *VInit)(GoReplayCondition condition, kType type, kObject sensor, kAlloc alloc);
    const kChar* (kCall *VName)();

    kStatus(kCall *VRead)(GoReplayCondition condition, kXml xml, kXmlItem root);
    kStatus(kCall *VWrite)(GoReplayCondition condition, kXml xml, kXmlItem root);

} GoReplayConditionVTable;

kDeclareVirtualClassEx(Go, GoReplayCondition, kObject)

GoFx(kStatus) GoReplayCondition_VInit(GoReplayCondition condition, kType type, kObject sensor, kAlloc alloc);
GoFx(kStatus) GoReplayCondition_VRelease(GoReplayCondition condition);
GoFx(const kChar*) GoReplayCondition_VName();
GoFx(kStatus) GoReplayCondition_VRead(GoReplayCondition condition, kXml xml, kXmlItem root);
GoFx(kStatus) GoReplayCondition_VWrite(GoReplayCondition condition, kXml xml, kXmlItem root);

GoFx(kStatus) GoReplayCondition_Init(GoReplayCondition condition, kType type, kObject sensor, kAlloc alloc);
GoFx(const kChar*) GoReplayCondition_Name(kType type);

typedef struct GoReplayAnyMeasurementClass
{
    GoReplayConditionClass base;

    GoReplayMeasurementResult result;

} GoReplayAnyMeasurementClass;

kDeclareClassEx(Go, GoReplayAnyMeasurement, GoReplayCondition)

GoFx(kStatus) GoReplayAnyMeasurement_Construct(GoReplayAnyMeasurement* condition, kObject sensor, kAlloc alloc);
GoFx(kStatus) GoReplayAnyMeasurement_VInit(GoReplayAnyMeasurement condition, kType type, kObject sensor, kAlloc alloc);
GoFx(kStatus) GoReplayAnyMeasurement_VRelease(GoReplayAnyMeasurement condition);
GoFx(const kChar*) GoReplayAnyMeasurement_VName();

GoFx(kStatus) GoReplayAnyMeasurement_VRead(GoReplayAnyMeasurement condition, kXml xml, kXmlItem root);
GoFx(kStatus) GoReplayAnyMeasurement_VWrite(GoReplayAnyMeasurement condition, kXml xml, kXmlItem root);

typedef struct GoReplayAnyDataClass
{
    GoReplayConditionClass base;

    GoReplayRangeCountCase rangeCountCase;
    k32u rangeCountThreshold;

} GoReplayAnyDataClass;

kDeclareClassEx(Go, GoReplayAnyData, GoReplayCondition)

GoFx(kStatus) GoReplayAnyData_Construct(GoReplayAnyData* condition, kObject sensor, kAlloc alloc);
GoFx(kStatus) GoReplayAnyData_VInit(GoReplayAnyData condition, kType type, kObject sensor, kAlloc alloc);
GoFx(kStatus) GoReplayAnyData_VRelease(GoReplayAnyData condition);
GoFx(const kChar*) GoReplayAnyData_VName();

GoFx(kStatus) GoReplayAnyData_VRead(GoReplayAnyData condition, kXml xml, kXmlItem root);
GoFx(kStatus) GoReplayAnyData_VWrite(GoReplayAnyData condition, kXml xml, kXmlItem root);

typedef struct GoReplayMeasurementClass
{
    GoReplayConditionClass base;

    GoReplayMeasurementResult result;
    k32u id;

} GoReplayMeasurementClass;

kDeclareClassEx(Go, GoReplayMeasurement, GoReplayCondition)

GoFx(kStatus) GoReplayMeasurement_Construct(GoReplayMeasurement* condition, kObject sensor, kAlloc alloc);
GoFx(kStatus) GoReplayMeasurement_VInit(GoReplayMeasurement condition, kType type, kObject sensor, kAlloc alloc);
GoFx(kStatus) GoReplayMeasurement_VRelease(GoReplayMeasurement condition);
GoFx(const kChar*) GoReplayMeasurement_VName();

GoFx(kStatus) GoReplayMeasurement_VRead(GoReplayMeasurement condition, kXml xml, kXmlItem root);
GoFx(kStatus) GoReplayMeasurement_VWrite(GoReplayMeasurement condition, kXml xml, kXmlItem root);

#endif

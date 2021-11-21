/**
* @file    GoExtMeasurement.c
*
* @internal
* Copyright (C) 2016-2021 by LMI Technologies Inc.
* Licensed under the MIT License.
* Redistributed files must retain the above copyright notice.
*/
#include <GoSdk/Tools/GoMeasurement.h>
#include <GoSdk/Tools/GoExtMeasurement.h>
#include <GoSdk/GoSensor.h>
#include <GoSdk/GoUtils.h>


kBeginClassEx(Go, GoExtMeasurement)
kAddVMethod(GoExtMeasurement, GoMeasurement, VInit)
kAddVMethod(GoExtMeasurement, GoMeasurement, VRead)
kAddVMethod(GoExtMeasurement, GoMeasurement, VWrite)
kAddVMethod(GoExtMeasurement, kObject, VRelease)
kEndClassEx()

GoFx(kStatus) GoExtMeasurement_Construct(GoExtMeasurement* measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback(allocator);
    kStatus status;

    kCheck(kAlloc_GetObject(alloc, type, measurement));

    if (!kSuccess(status = kType_VTableT(type, GoMeasurement)->VInit(*measurement, type, sensor, srcTool, isFilterable, alloc)))
    {
        kAlloc_FreeRef(alloc, measurement);
    }

    return status;
}

GoFx(kStatus) GoExtMeasurement_VInit(GoExtMeasurement measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    return GoExtMeasurement_Init(measurement, type, GO_MEASUREMENT_EXTENSIBLE, sensor, srcTool, isFilterable, alloc);
}
GoFx(kStatus) GoExtMeasurement_Init(GoExtMeasurement measurement, kType type, GoMeasurementType typeId, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc)
{
    kObjR(GoExtMeasurement, measurement);
    kStatus exception;

    kCheck(GoMeasurement_Init(measurement, type, typeId, sensor, srcTool, kTRUE, alloc));
    obj->params = kNULL;
    obj->type[0] = 0;

    kTry
    {
        kTest(GoExtParams_Construct(&obj->params, sensor, alloc));
    }
    kCatch(&exception)
    {
        GoExtMeasurement_VRelease(measurement);
        kEndCatch(exception);
    }

    return kOK;
}

GoFx(kStatus) GoExtMeasurement_VRead(GoExtMeasurement measurement, kXml xml, kXmlItem item)
{
    kObj(GoExtMeasurement, measurement);
    kXmlItem paramsItem = kNULL;

    kCheck(GoMeasurement_VRead(measurement, xml, item));

    kCheck(kXml_AttrText(xml, item, "type", obj->type, 64));
    
    kCheck(!kIsNull(paramsItem = kXml_Child(xml, item, "Parameters")));
    kCheck(GoExtParams_Read(obj->params, xml, paramsItem));

    return kOK;
}

GoFx(kStatus) GoExtMeasurement_VWrite(GoExtMeasurement measurement, kXml xml, kXmlItem item)
{
    kObj(GoExtMeasurement, measurement);
    kXmlItem paramsItem = kNULL;

    kCheck(GoMeasurement_VWrite(measurement, xml, item));

    kCheck(kXml_SetAttrText(xml, item, "type", obj->type));

    kCheck(kXml_AddItem(xml, item, "Parameters", &paramsItem));
    kCheck(GoExtParams_Write(obj->params, xml, paramsItem));

    return kOK;
}

GoFx(kStatus) GoExtMeasurement_VRelease(GoExtMeasurement measurement)
{
    kObj(GoExtMeasurement, measurement);

    kCheck(kDisposeRef(&obj->params));

    return GoMeasurement_VRelease(measurement);
}

GoFx(const kChar*) GoExtMeasurement_Type(GoExtMeasurement measurement)
{
    kObj(GoExtMeasurement, measurement);
    GoSensor_SyncConfig(obj->base.sensor);
    return obj->type;
}

GoFx(kSize) GoExtMeasurement_CustomParameterCount(GoExtMeasurement measurement)
{
    kObj(GoExtMeasurement, measurement);
    GoSensor_SyncConfig(obj->base.sensor);

    return GoExtParams_ParameterCount(obj->params);
}

GoFx(GoExtParam) GoExtMeasurement_CustomParameterAt(GoExtMeasurement measurement, kSize index)
{
    kObj(GoExtMeasurement, measurement);
    GoSensor_SyncConfig(obj->base.sensor);
    return GoExtParams_ParameterAt(obj->params, index);
}

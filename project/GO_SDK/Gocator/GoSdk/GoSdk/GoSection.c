/** 
 * @file    GoSection.c
 *
 * @internal
 * Copyright (C) 2016-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <GoSdk/GoSection.h>
#include <kApi/Data/kXml.h>
#include <kApi/Utils/kUtils.h>
#include <GoSdk/GoSensor.h>
#include <GoSdk/GoUtils.h>

kBeginClassEx(Go, GoSection)
    kAddVMethod(GoSection, kObject, VRelease)
kEndClassEx()

GoFx(kStatus) GoSection_Construct(GoSection* section, kObject sensor, k16s id, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback(allocator);
    kStatus status; 

    kCheck(kAlloc_GetObject(alloc, kTypeOf(GoSection), section)); 

    if (!kSuccess(status = GoSection_Init(*section, kTypeOf(GoSection), sensor, id, alloc)))
    {
        kAlloc_FreeRef(alloc, section); 
    }

    return status; 
} 

GoFx(kStatus) GoSection_Init(GoSection section, kType type, kObject sensor, k16s id, kAlloc alloc)
{
    kObjR(GoSection, section); 

    kCheck(kObject_Init(section, type, alloc)); 
    obj->name[0] = 0;
    kZero(obj->startPoint);
    kZero(obj->endPoint);
    obj->customSpacingIntervalEnabled = kFALSE;
    kZero(obj->spacingInterval);

    obj->id = id;
    obj->sensor = sensor;
    
    return kOK; 
}

GoFx(kStatus) GoSection_VRelease(GoSection section)
{
    kObj(GoSection, section);

    return kObject_VRelease(section);
}

GoFx(kStatus) GoSection_Read(GoSection section, kXml xml, kXmlItem item)
{
    kObj(GoSection, section); 
    kXmlItem pointItem = kNULL;

    kCheck(kXml_Attr16s(xml, item, "id", &obj->id));
    kCheck(kXml_ChildText(xml, item, "Name", obj->name, 128));
    pointItem = kXml_Child(xml, item, "StartPoint");
    if (!kIsNull(pointItem))
    {
        kCheck(kXml_Child64f(xml, pointItem, "X", &obj->startPoint.x));
        kCheck(kXml_Child64f(xml, pointItem, "Y", &obj->startPoint.y));
    }

    pointItem = kXml_Child(xml, item, "EndPoint");
    if (!kIsNull(pointItem))
    {
        kCheck(kXml_Child64f(xml, pointItem, "X", &obj->endPoint.x));
        kCheck(kXml_Child64f(xml, pointItem, "Y", &obj->endPoint.y));
    }

    kCheck(kXml_ChildBool(xml, item, "CustomSpacingIntervalEnabled", &obj->customSpacingIntervalEnabled));
    kCheck(GoConfig_ReadRangeElement64f(xml, item, "SpacingInterval", &obj->spacingInterval));
    
    return kOK; 
}

GoFx(kStatus) GoSection_Write(GoSection section, kXml xml, kXmlItem item)
{
    kObj(GoSection, section);
    kXmlItem pointItem = kNULL;

    kCheck(kXml_SetAttr16s(xml, item, "id", obj->id));
    kCheck(kXml_SetChildText(xml, item, "Name", obj->name));
    kCheck(kXml_AddItem(xml, item, "StartPoint", &pointItem));
    kCheck(kXml_SetChild64f(xml, pointItem, "X", obj->startPoint.x));
    kCheck(kXml_SetChild64f(xml, pointItem, "Y", obj->startPoint.y));

    kCheck(kXml_AddItem(xml, item, "EndPoint", &pointItem));
    kCheck(kXml_SetChild64f(xml, pointItem, "X", obj->endPoint.x));
    kCheck(kXml_SetChild64f(xml, pointItem, "Y", obj->endPoint.y));

    kCheck(kXml_SetChildBool(xml, item, "CustomSpacingIntervalEnabled", obj->customSpacingIntervalEnabled));
    kCheck(kXml_SetChild64f(xml, item, "SpacingInterval", obj->spacingInterval.value));

    return kOK;
}

GoFx(k16s) GoSection_Id(GoSection section)
{
    kObj(GoSection, section);  
    return obj->id; 
}

GoFx(kStatus) GoSection_SetId(GoSection section, k16s id)
{
    kObj(GoSection, section);
    
    obj->id = id;

    return kOK;
}

GoFx(kStatus) GoSection_SetName(GoSection section, const kChar* name)
{
    kObj(GoSection, section);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));
    kStrCopy(obj->name, kCountOf(obj->name), name);
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(kStatus) GoSection_Name(GoSection section, kChar* name, kSize capacity)
{
    kObj(GoSection, section);

    GoSensor_SyncConfig(obj->sensor);
    kStrCopy(name, capacity, obj->name);

    return kOK;
}

GoFx(kPoint64f) GoSection_StartPoint(GoSection section)
{
    kObj(GoSection, section);

    GoSensor_SyncConfig(obj->sensor);

    return obj->startPoint;
}

GoFx(kStatus) GoSection_SetStartPoint(GoSection section, kPoint64f point)
{
    kObj(GoSection, section);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));
    kCheck(GoSensor_SyncConfig(obj->sensor));

    obj->startPoint = point;

    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(kPoint64f) GoSection_EndPoint(GoSection section)
{
    kObj(GoSection, section);

    GoSensor_SyncConfig(obj->sensor);

    return obj->endPoint;
}

GoFx(kStatus) GoSection_SetEndPoint(GoSection section, kPoint64f point)
{
    kObj(GoSection, section);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));
    kCheck(GoSensor_SyncConfig(obj->sensor));

    obj->endPoint = point;

    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(k64f) GoSection_SpacingInterval(GoSection section)
{
    kObj(GoSection, section);

    return obj->spacingInterval.value;
}

GoFx(kStatus) GoSection_SetSpacingInterval(GoSection section, k64f value)
{
    kObj(GoSection, section);
    
    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));
    kCheck(GoSensor_SyncConfig(obj->sensor));

    kCheckArgs(value >= obj->spacingInterval.min && value <= obj->spacingInterval.max);

    obj->spacingInterval.value = value;

    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(k64f) GoSection_SpacingIntervalLimitMin(GoSection section)
{
    kObj(GoSection, section);

    GoSensor_SyncConfig(obj->sensor);

    return obj->spacingInterval.min;
}

GoFx(k64f) GoSection_SpacingIntervalLimitMax(GoSection section)
{
    kObj(GoSection, section);

    GoSensor_SyncConfig(obj->sensor);

    return obj->spacingInterval.max;
}

GoFx(k64f) GoSection_SpacingIntervalSystemValue(GoSection section)
{
    kObj(GoSection, section);

    GoSensor_SyncConfig(obj->sensor);

    return obj->spacingInterval.systemValue;
}

GoFx(kBool) GoSection_CustomSpacingIntervalEnabled(GoSection section)
{
    kObj(GoSection, section);

    GoSensor_SyncConfig(obj->sensor);

    return obj->customSpacingIntervalEnabled;
}

GoFx(kStatus) GoSection_EnableCustomSpacingInterval(GoSection section, kBool enable)
{
    kObj(GoSection, section);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));
    kCheck(GoSensor_SyncConfig(obj->sensor));

    obj->customSpacingIntervalEnabled = enable;

    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

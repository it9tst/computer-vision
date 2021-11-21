/** 
 * @file    GoTracheid.c
 *
 * @internal
 * Copyright (C) 2016-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <GoSdk/GoTracheid.h>
#include <GoSdk/GoSensor.h>
#include <GoSdk/GoUtils.h>

kBeginClassEx(Go, GoTracheid)
    kAddVMethod(GoTracheid, kObject, VRelease)
kEndClassEx()

GoFx(kStatus) GoTracheid_Construct(GoTracheid* tracheid, kObject sensor, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback(allocator);
    kStatus status; 

    kCheck(kAlloc_GetObject(alloc, kTypeOf(GoTracheid), tracheid));

    if (!kSuccess(status = GoTracheid_Init(*tracheid, kTypeOf(GoTracheid), sensor, alloc)))
    {
        kAlloc_FreeRef(alloc, tracheid); 
    }

    return status; 
} 

GoFx(kStatus) GoTracheid_Init(GoTracheid tracheid, kType type, kObject sensor, kAlloc alloc)
{
    kObjR(GoTracheid, tracheid); 

    kCheck(kObject_Init(tracheid, type, alloc)); 
    kZero(obj->xml);
    kZero(obj->xmlItem);
    obj->used = kFALSE;
    kZero(obj->exposure);
    obj->camera0Threshold = 0;
    obj->camera1Threshold = 0;

    obj->sensor = sensor; 

    return kOK; 
}

GoFx(kStatus) GoTracheid_VRelease(GoTracheid tracheid)
{
    kObj(GoTracheid, tracheid); 

    return kObject_VRelease(tracheid);
}

GoFx(kStatus) GoTracheid_Read(GoTracheid tracheid, kXml xml, kXmlItem item)
{
    kObj(GoTracheid, tracheid);
    kXmlItem tempItem = kNULL;

    obj->xml = xml;
    obj->xmlItem = item;

    kCheck(kXml_AttrBool(xml, item, "used", &obj->used));

    kCheck(kXml_ChildBool(xml, item, "TracheidExposureEnabled", &obj->exposure.enabled));

    kCheck(kXml_Child64f(xml, item, "TracheidExposure", &obj->exposure.value));
    tempItem = kXml_Child(xml, item, "TracheidExposure");
    if (!kIsNull(tempItem))
    {
        kCheck(kXml_Attr64f(xml, tempItem, "min", &obj->exposure.min));
        kCheck(kXml_Attr64f(xml, tempItem, "max", &obj->exposure.min));
    }

    kCheck(kXml_Child32s(xml, item, "Camera0Threshold", &obj->camera0Threshold));
    kCheck(kXml_Child32s(xml, item, "Camera1Threshold", &obj->camera1Threshold));

    return kOK; 
}

GoFx(kStatus) GoTracheid_Write(GoTracheid tracheid, kXml xml, kXmlItem item)
{
    kObj(GoTracheid, tracheid);
    kXmlItem tempItem = kNULL;

    kCheck(kXml_SetAttrBool(xml, item, "used", obj->used));

    kCheck(kXml_SetChildBool(xml, item, "TracheidExposureEnabled", obj->exposure.enabled));

    kCheck(kXml_AddItem(xml, item, "TracheidExposure", &tempItem));
    kCheck(kXml_SetItem64f(xml, tempItem, obj->exposure.value));
    kCheck(kXml_SetAttr64f(xml, tempItem, "min", obj->exposure.min));
    kCheck(kXml_SetAttr64f(xml, tempItem, "max", obj->exposure.max));

    kCheck(kXml_SetChild32s(xml, item, "Camera0Threshold", obj->camera0Threshold));
    kCheck(kXml_SetChild32s(xml, item, "Camera1Threshold", obj->camera1Threshold));

    //Forwards Compatibility
    kCheck(GoUtils_XmlMerge(obj->xml, obj->xmlItem, xml, item));

    return kOK; 
}

GoFx(kBool) GoTracheid_Used(GoTracheid tracheid)
{
    kObj(GoTracheid, tracheid);

    return obj->used;
}

GoFx(kStatus) GoTracheid_SetExposure(GoTracheid tracheid, k64f exposure)
{
    kObj(GoTracheid, tracheid);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->exposure.value = exposure;

    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(k64f) GoTracheid_Exposure(GoTracheid tracheid)
{
    kObj(GoTracheid, tracheid);

    GoSensor_SyncConfig(obj->sensor);

    return obj->exposure.value;
}

GoFx(kStatus) GoTracheid_EnableExposure(GoTracheid tracheid, kBool enable)
{
    kObj(GoTracheid, tracheid);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->exposure.enabled = enable;

    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(kBool) GoTracheid_ExposureEnabled(GoTracheid tracheid)
{
    kObj(GoTracheid, tracheid);

    GoSensor_SyncConfig(obj->sensor);

    return obj->exposure.enabled;
}

GoFx(kStatus) GoTracheid_SetCameraThresholdAt(GoTracheid tracheid, kSize cameraIndex, k32s threshold)
{
    kObj(GoTracheid, tracheid);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    switch (cameraIndex)
    {
    case 0: obj->camera0Threshold = threshold; 
        break;

    case 1: obj->camera1Threshold = threshold; 
        break;

    default: return kERROR_PARAMETER;
    }

    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(kStatus) GoTracheid_GetCameraThresholdAt(GoTracheid tracheid, kSize cameraIndex, k32s* threshold)
{
    kObj(GoTracheid, tracheid);

    GoSensor_SyncConfig(obj->sensor);

    kCheckArgs(!kIsNull(threshold));

    switch (cameraIndex)
    {
    case 0: *threshold = obj->camera0Threshold;
        return kOK;

    case 1: *threshold = obj->camera1Threshold;
        return kOK;

    default: return kERROR_PARAMETER;
    }
}

/** 
 * @file    GoTransform.c
 *
 * @internal
 * Copyright (C) 2016-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <GoSdk/GoTransform.h>
#include <GoSdk/GoSensor.h>
#include <GoSdk/GoUtils.h>

kBeginClassEx(Go, GoTransform)
    kAddVMethod(GoTransform, kObject, VRelease)
kEndClassEx()

GoFx(kStatus) GoTransform_Construct(GoTransform* transform, kObject sensor, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback(allocator);
    kStatus status; 

    kCheck(kAlloc_GetObject(alloc, kTypeOf(GoTransform), transform)); 

    if (!kSuccess(status = GoTransform_Init(*transform, kTypeOf(GoTransform), sensor, alloc)))
    {
        kAlloc_FreeRef(alloc, transform); 
    }

    return status; 
} 

GoFx(kStatus) GoTransform_Init(GoTransform transform, kType type, kObject sensor, kAlloc alloc)
{
    kObjR(GoTransform, transform); 

    kCheck(kObject_Init(transform, type, alloc)); 
    kZero(obj->xml);
    kZero(obj->xmlItem);
    obj->encoderSpeed = 0.0;
    obj->encoderResolution = 0.0;
    kZero(obj->transformationList);

    obj->sensor = sensor; 

    kCheck(kArrayList_Construct(&obj->transformationList, kTypeOf(GoTransformation), 0, alloc));

    return kOK; 
}

GoFx(kStatus) GoTransform_VRelease(GoTransform transform)
{
    kObj(GoTransform, transform); 
    
    kDestroyRef(&obj->xml);
    kDestroyRef(&obj->transformationList);

    return kObject_VRelease(transform);
}

GoFx(kStatus) GoTransform_Read(GoTransform transform, kXml xml, kXmlItem item)
{
    kObj(GoTransform, transform); 
    kXmlItem devicesItem = kNULL;
    kXmlItem tempItem = kNULL;
    kSize i;
    kSize deviceCount;

    kDestroyRef(&obj->xml);
    obj->xml = xml;
    obj->xmlItem = item;

    kCheck(kXml_Child64f(xml, item, "EncoderResolution", &obj->encoderResolution));
    kCheck(kXml_Child64f(xml, item, "Speed", &obj->encoderSpeed));

    kCheckArgs(!kIsNull(devicesItem = kXml_Child(xml, item, "Devices")));
    deviceCount = kXml_ChildCount(xml, devicesItem);

    kCheck(kArrayList_Purge(obj->transformationList));

    for (i = 0; i < deviceCount; i++)
    {
        GoTransformation transformation;

        tempItem = kXml_ChildAt(xml, devicesItem, i);

        kCheck(kXml_Child64f(xml, tempItem, "X", &transformation.x));
        kCheck(kXml_Child64f(xml, tempItem, "Y", &transformation.y));
        kCheck(kXml_Child64f(xml, tempItem, "Z", &transformation.z));
        kCheck(kXml_Child64f(xml, tempItem, "XAngle", &transformation.xAngle));
        kCheck(kXml_Child64f(xml, tempItem, "YAngle", &transformation.yAngle));
        kCheck(kXml_Child64f(xml, tempItem, "ZAngle", &transformation.zAngle));

        kCheck(kArrayList_AddT(obj->transformationList, &transformation));
    }

    return kOK; 
}

GoFx(kStatus) GoTransform_Write(GoTransform transform, kXml xml, kXmlItem item)
{
    kObj(GoTransform, transform); 
    kXmlItem devicesItem = kNULL;
    kXmlItem tempItem = kNULL;
    kSize i;

    kCheck(kXml_SetChild64f(xml, item, "EncoderResolution", obj->encoderResolution));
    kCheck(kXml_SetChild64f(xml, item, "Speed", obj->encoderSpeed));

    kCheck(kXml_AddItem(xml, item, "Devices", &devicesItem));

    for (i = 0; i < kArrayList_Count(obj->transformationList); i++)
    {
        GoTransformation transformation = kPointer_ReadAs(kArrayList_AtT(obj->transformationList, i, GoTransformation), GoTransformation);

        kCheck(kXml_AddItem(xml, devicesItem, "Device", &tempItem));
        kCheck(kXml_SetAttrSize(xml, tempItem, "role", i));
        kCheck(kXml_SetChild64f(xml, tempItem, "X", transformation.x));
        kCheck(kXml_SetChild64f(xml, tempItem, "Y", transformation.y));
        kCheck(kXml_SetChild64f(xml, tempItem, "Z", transformation.z));
        kCheck(kXml_SetChild64f(xml, tempItem, "XAngle", transformation.xAngle));
        kCheck(kXml_SetChild64f(xml, tempItem, "YAngle", transformation.yAngle));
        kCheck(kXml_SetChild64f(xml, tempItem, "ZAngle", transformation.zAngle));
    }

    //Forwards Compatibility
    kCheck(GoUtils_XmlMerge(obj->xml, obj->xmlItem, xml, item));

    return kOK; 
}

GoFx(k64f) GoTransform_EncoderResolution(GoTransform transform)
{
    kObj(GoTransform, transform);

    GoSensor_SyncTransform(obj->sensor);

    return obj->encoderResolution;
}

GoFx(kStatus) GoTransform_SetEncoderResolution(GoTransform transform, k64f value)
{
    kObj(GoTransform, transform);

    obj->encoderResolution = value;

    kCheck(GoSensor_SetTransformModified(obj->sensor));
    kCheck(GoSensor_SyncTransform(obj->sensor));

    return kOK;
}

GoFx(k64f) GoTransform_Speed(GoTransform transform)
{
    kObj(GoTransform, transform);

    GoSensor_SyncTransform(obj->sensor);

    return obj->encoderSpeed;
}

GoFx(kStatus) GoTransform_SetSpeed(GoTransform transform, k64f value)
{
    kObj(GoTransform, transform);

    obj->encoderSpeed = value;

    kCheck(GoSensor_SetTransformModified(obj->sensor));
    kCheck(GoSensor_SyncTransform(obj->sensor));

    return kOK;
}

GoFx(k64f) GoTransform_X(GoTransform transform, GoRole role)
{
    kObj(GoTransform, transform);

    GoSensor_SyncTransform(obj->sensor);

    return kPointer_ReadAs(kArrayList_AtT(obj->transformationList, (kSize)role, GoTransformation), GoTransformation).x;
}

GoFx(kStatus) GoTransform_SetX(GoTransform transform, GoRole role, k64f offset)
{
    kObj(GoTransform, transform);

    kPointer_ReadAs(kArrayList_AtT(obj->transformationList, (kSize) role, GoTransformation), GoTransformation).x = offset;

    kCheck(GoSensor_SetTransformModified(obj->sensor));
    kCheck(GoSensor_SyncTransform(obj->sensor));

    return kOK;
}

GoFx(k64f) GoTransform_Y(GoTransform transform, GoRole role)
{
    kObj(GoTransform, transform);

    GoSensor_SyncTransform(obj->sensor);

    return kPointer_ReadAs(kArrayList_AtT(obj->transformationList, (kSize)role, GoTransformation), GoTransformation).y;
}

GoFx(kStatus) GoTransform_SetY(GoTransform transform, GoRole role, k64f offset)
{
    kObj(GoTransform, transform);

    kPointer_ReadAs(kArrayList_AtT(obj->transformationList, (kSize)role, GoTransformation), GoTransformation).y = offset;

    kCheck(GoSensor_SetTransformModified(obj->sensor));
    kCheck(GoSensor_SyncTransform(obj->sensor));

    return kOK;
}

GoFx(k64f) GoTransform_Z(GoTransform transform, GoRole role)
{
    kObj(GoTransform, transform);

    GoSensor_SyncTransform(obj->sensor);

    return kPointer_ReadAs(kArrayList_AtT(obj->transformationList, (kSize)role, GoTransformation), GoTransformation).z;
}

GoFx(kStatus) GoTransform_SetZ(GoTransform transform, GoRole role, k64f offset)
{
    kObj(GoTransform, transform);

    kPointer_ReadAs(kArrayList_AtT(obj->transformationList, (kSize)role, GoTransformation), GoTransformation).z = offset;

    kCheck(GoSensor_SetTransformModified(obj->sensor));
    kCheck(GoSensor_SyncTransform(obj->sensor));

    return kOK;
}


GoFx(k64f) GoTransform_XAngle(GoTransform transform, GoRole role)
{
    kObj(GoTransform, transform);

    GoSensor_SyncTransform(obj->sensor);

    return kPointer_ReadAs(kArrayList_AtT(obj->transformationList, (kSize)role, GoTransformation), GoTransformation).xAngle;
}

GoFx(kStatus) GoTransform_SetXAngle(GoTransform transform, GoRole role, k64f angle)
{
    kObj(GoTransform, transform);

    kPointer_ReadAs(kArrayList_AtT(obj->transformationList, (kSize)role, GoTransformation), GoTransformation).xAngle = angle;

    kCheck(GoSensor_SetTransformModified(obj->sensor));
    kCheck(GoSensor_SyncTransform(obj->sensor));

    return kOK;
}

GoFx(k64f) GoTransform_YAngle(GoTransform transform, GoRole role)
{
    kObj(GoTransform, transform);

    GoSensor_SyncTransform(obj->sensor);

    return kPointer_ReadAs(kArrayList_AtT(obj->transformationList, (kSize)role, GoTransformation), GoTransformation).yAngle;
}

GoFx(kStatus) GoTransform_SetYAngle(GoTransform transform, GoRole role, k64f angle)
{
    kObj(GoTransform, transform);

    kPointer_ReadAs(kArrayList_AtT(obj->transformationList, (kSize)role, GoTransformation), GoTransformation).yAngle = angle;

    kCheck(GoSensor_SetTransformModified(obj->sensor));
    kCheck(GoSensor_SyncTransform(obj->sensor));

    return kOK;
}

GoFx(k64f) GoTransform_ZAngle(GoTransform transform, GoRole role)
{
    kObj(GoTransform, transform);

    GoSensor_SyncTransform(obj->sensor);

    return kPointer_ReadAs(kArrayList_AtT(obj->transformationList, (kSize)role, GoTransformation), GoTransformation).zAngle;
}

GoFx(kStatus) GoTransform_SetZAngle(GoTransform transform, GoRole role, k64f angle)
{
    kObj(GoTransform, transform);

    kPointer_ReadAs(kArrayList_AtT(obj->transformationList, (kSize)role, GoTransformation), GoTransformation).zAngle = angle;

    kCheck(GoSensor_SetTransformModified(obj->sensor));
    kCheck(GoSensor_SyncTransform(obj->sensor));

    return kOK;
}

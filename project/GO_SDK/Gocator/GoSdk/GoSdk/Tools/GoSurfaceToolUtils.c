/**
 * @file    GoSurfaceToolUtils.c
 *
 * @internal
 * Copyright (C) 2016-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <GoSdk/Tools/GoSurfaceToolUtils.h>
#include <GoSdk/GoSensor.h>


kBeginClassEx(Go, GoCylinderRegion)
kEndClassEx()

GoFx(kStatus) GoCylinderRegion_Construct(GoCylinderRegion* region, kObject sensor, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback(allocator);
    kStatus status;

    kCheck(kAlloc_GetObject(alloc, kTypeOf(GoCylinderRegion), region));

    if (!kSuccess(status = GoCylinderRegion_Init(*region, kTypeOf(GoCylinderRegion), sensor, alloc)))
    {
        kAlloc_FreeRef(alloc, region);
    }

    return status;
}

GoFx(kStatus) GoCylinderRegion_Init(GoCylinderRegion region, kType type, kObject sensor, kAlloc alloc)
{
    kObjR(GoCylinderRegion, region);

    kCheck(kObject_Init(region, type, alloc));
    kZero(obj->xml);
    kZero(obj->xmlItem);
    obj->x = 0.0;
    obj->y = 0.0;
    obj->z = 0.0;

    obj->sensor = sensor;

    obj->radius = 0.5;
    obj->height = 1.0;

    return kOK;
}

GoFx(kStatus) GoCylinderRegion_Read(GoCylinderRegion region, kXml xml, kXmlItem item)
{
    kObj(GoCylinderRegion, region);

    kCheck(kXml_Child64f(xml, item, "X", &obj->x));
    kCheck(kXml_Child64f(xml, item, "Y", &obj->y));
    kCheck(kXml_Child64f(xml, item, "Z", &obj->z));
    kCheck(kXml_Child64f(xml, item, "Radius", &obj->radius));
    kCheck(kXml_Child64f(xml, item, "Height", &obj->height));

    return kOK;
}

GoFx(kStatus) GoCylinderRegion_Write(GoCylinderRegion region, kXml xml, kXmlItem item)
{
    kObj(GoCylinderRegion, region);

    kCheck(kXml_SetChild64f(xml, item, "X", obj->x));
    kCheck(kXml_SetChild64f(xml, item, "Y", obj->y));
    kCheck(kXml_SetChild64f(xml, item, "Z", obj->z));
    kCheck(kXml_SetChild64f(xml, item, "Radius", obj->radius));
    kCheck(kXml_SetChild64f(xml, item, "Height", obj->height));

    return kOK;
}


GoFx(kStatus) GoCylinderRegion_SetX(GoCylinderRegion region, k64f x)
{
    kObj(GoCylinderRegion, region);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->x = x;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(k64f) GoCylinderRegion_X(GoCylinderRegion region)
{
    kObj(GoCylinderRegion, region);

    GoSensor_SyncConfig(obj->sensor);

    return obj->x;
}

GoFx(kStatus) GoCylinderRegion_SetY(GoCylinderRegion region, k64f y)
{
    kObj(GoCylinderRegion, region);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->y = y;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(k64f) GoCylinderRegion_Y(GoCylinderRegion region)
{
    kObj(GoCylinderRegion, region);

    GoSensor_SyncConfig(obj->sensor);

    return obj->y;
}

GoFx(kStatus) GoCylinderRegion_SetZ(GoCylinderRegion region, k64f z)
{
    kObj(GoCylinderRegion, region);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->z = z;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(k64f) GoCylinderRegion_Z(GoCylinderRegion region)
{
    kObj(GoCylinderRegion, region);

    GoSensor_SyncConfig(obj->sensor);

    return obj->z;
}

GoFx(kStatus) GoCylinderRegion_SetRadius(GoCylinderRegion region, k64f radius)
{
    kObj(GoCylinderRegion, region);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->radius = radius;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(k64f) GoCylinderRegion_Radius(GoCylinderRegion region)
{
    kObj(GoCylinderRegion, region);

    GoSensor_SyncConfig(obj->sensor);

    return obj->radius;
}

GoFx(kStatus) GoCylinderRegion_SetHeight(GoCylinderRegion region, k64f height)
{
    kObj(GoCylinderRegion, region);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->height = height;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(k64f) GoCylinderRegion_Height(GoCylinderRegion region)
{
    kObj(GoCylinderRegion, region);

    GoSensor_SyncConfig(obj->sensor);

    return obj->height;
}


kBeginClassEx(Go, GoSurfaceRegion2d)
kEndClassEx()

GoFx(kStatus) GoSurfaceRegion2d_Construct(GoSurfaceRegion2d* region, kObject sensor, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback(allocator);
    kStatus status;

    kCheck(kAlloc_GetObject(alloc, kTypeOf(GoSurfaceRegion2d), region));

    if (!kSuccess(status = GoSurfaceRegion2d_Init(*region, kTypeOf(GoSurfaceRegion2d), sensor, alloc)))
    {
        kAlloc_FreeRef(alloc, region);
    }

    return status;
}

GoFx(kStatus) GoSurfaceRegion2d_Init(GoSurfaceRegion2d region, kType type, kObject sensor, kAlloc alloc)
{
    kObjR(GoSurfaceRegion2d, region);

    kCheck(kObject_Init(region, type, alloc));
    kZero(obj->xml);
    kZero(obj->xmlItem);

    obj->sensor = sensor;

    obj->x = 0;
    obj->y = 0;
    obj->width = 1.0;
    obj->length = 1.0;
    obj->zAngle = 0;

    return kOK;
}

GoFx(kStatus) GoSurfaceRegion2d_Read(GoSurfaceRegion2d region, kXml xml, kXmlItem item)
{
    kObj(GoSurfaceRegion2d, region);

    kCheck(kXml_Child64f(xml, item, "X", &obj->x));
    kCheck(kXml_Child64f(xml, item, "Y", &obj->y));
    kCheck(kXml_Child64f(xml, item, "Width", &obj->width));
    kCheck(kXml_Child64f(xml, item, "Length", &obj->length));
    if (kXml_ChildExists(xml, item, "ZAngle") == kTRUE)
    {
        kCheck(kXml_Child64f(xml, item, "ZAngle", &obj->zAngle));
    }

    return kOK;
}

GoFx(kStatus) GoSurfaceRegion2d_Write(GoSurfaceRegion2d region, kXml xml, kXmlItem item)
{
    kObj(GoSurfaceRegion2d, region);

    kCheck(kXml_SetChild64f(xml, item, "X", obj->x));
    kCheck(kXml_SetChild64f(xml, item, "Y", obj->y));
    kCheck(kXml_SetChild64f(xml, item, "Width", obj->width));
    kCheck(kXml_SetChild64f(xml, item, "Length", obj->length));
    kCheck(kXml_SetChild64f(xml, item, "ZAngle", obj->zAngle));

    return kOK;
}


GoFx(kStatus) GoSurfaceRegion2d_SetX(GoSurfaceRegion2d region, k64f x)
{
    kObj(GoSurfaceRegion2d, region);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->x = x;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(k64f) GoSurfaceRegion2d_X(GoSurfaceRegion2d region)
{
    kObj(GoSurfaceRegion2d, region);

    GoSensor_SyncConfig(obj->sensor);

    return obj->x;
}

GoFx(kStatus) GoSurfaceRegion2d_SetY(GoSurfaceRegion2d region, k64f y)
{
    kObj(GoSurfaceRegion2d, region);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->y = y;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(k64f) GoSurfaceRegion2d_Y(GoSurfaceRegion2d region)
{
    kObj(GoSurfaceRegion2d, region);

    GoSensor_SyncConfig(obj->sensor);

    return obj->y;
}

GoFx(kStatus) GoSurfaceRegion2d_SetWidth(GoSurfaceRegion2d region, k64f width)
{
    kObj(GoSurfaceRegion2d, region);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->width = width;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(k64f) GoSurfaceRegion2d_Width(GoSurfaceRegion2d region)
{
    kObj(GoSurfaceRegion2d, region);

    GoSensor_SyncConfig(obj->sensor);

    return obj->width;
}

GoFx(kStatus) GoSurfaceRegion2d_SetLength(GoSurfaceRegion2d region, k64f length)
{
    kObj(GoSurfaceRegion2d, region);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->length = length;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(k64f) GoSurfaceRegion2d_Length(GoSurfaceRegion2d region)
{
    kObj(GoSurfaceRegion2d, region);

    GoSensor_SyncConfig(obj->sensor);

    return obj->length;
}
//Added H.Ware
GoFx(kStatus) GoSurfaceRegion2d_SetZAngle(GoSurfaceRegion2d region, k64f zAngle)
{
    kObj(GoSurfaceRegion2d, region);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->zAngle = zAngle;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

   return kOK;
}

GoFx(k64f) GoSurfaceRegion2d_ZAngle(GoSurfaceRegion2d region)
{
    kObj(GoSurfaceRegion2d, region);

    GoSensor_SyncConfig(obj->sensor);

    return obj->zAngle;
}

kBeginClassEx(Go, GoRegion3d)
kEndClassEx()

GoFx(kStatus) GoRegion3d_Construct(GoRegion3d* region, kObject sensor, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback(allocator);
    kStatus status;

    kCheck(kAlloc_GetObject(alloc, kTypeOf(GoRegion3d), region));

    if (!kSuccess(status = GoRegion3d_Init(*region, kTypeOf(GoRegion3d), sensor, alloc)))
    {
        kAlloc_FreeRef(alloc, region);
    }

    return status;
}

GoFx(kStatus) GoRegion3d_Init(GoRegion3d region, kType type, kObject sensor, kAlloc alloc)
{
    kObjR(GoRegion3d, region);

    kCheck(kObject_Init(region, type, alloc));
    kZero(obj->xml);
    kZero(obj->xmlItem);
    obj->x = 0.0;
    obj->y = 0.0;
    obj->z = 0.0;

    obj->sensor = sensor;

    obj->width = 1.0;
    obj->height = 1.0;
    obj->length = 1.0;

    obj->zAngle = 0;
    return kOK;
}

GoFx(kStatus) GoRegion3d_Read(GoRegion3d region, kXml xml, kXmlItem item)
{
    kObj(GoRegion3d, region);
    kXmlItem tempItem = kNULL;

    kCheck(kXml_Child64f(xml, item, "X", &obj->x));
    kCheck(kXml_Child64f(xml, item, "Y", &obj->y));
    kCheck(kXml_Child64f(xml, item, "Z", &obj->z));
    kCheck(kXml_Child64f(xml, item, "Width", &obj->width));
    kCheck(kXml_Child64f(xml, item, "Length", &obj->length));
    kCheck(kXml_Child64f(xml, item, "Height", &obj->height));

    tempItem = kXml_Child(xml, item, "ZAngle");
    if (!kIsNull(tempItem))
    {
        if (kXml_AttrExists(xml, tempItem, "used"))
        {
            kBool bUsed = kFALSE;
            kCheck(kXml_AttrBool(xml, tempItem, "used", &bUsed));
            if (bUsed == kTRUE)
            {
                kCheck(kXml_Child64f(xml, item, "ZAngle", &obj->zAngle));
            }
        }
    }

    return kOK;
}

GoFx(kStatus) GoRegion3d_Write(GoRegion3d region, kXml xml, kXmlItem item)
{
    kObj(GoRegion3d, region);

    kCheck(kXml_SetChild64f(xml, item, "X", obj->x));
    kCheck(kXml_SetChild64f(xml, item, "Y", obj->y));
    kCheck(kXml_SetChild64f(xml, item, "Z", obj->z));
    kCheck(kXml_SetChild64f(xml, item, "Width", obj->width));
    kCheck(kXml_SetChild64f(xml, item, "Length", obj->length));
    kCheck(kXml_SetChild64f(xml, item, "Height", obj->height));
    kCheck(kXml_SetChild64f(xml, item, "ZAngle", obj->zAngle));

    return kOK;
}


GoFx(kStatus) GoRegion3d_SetX(GoRegion3d region, k64f x)
{
    kObj(GoRegion3d, region);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->x = x;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(k64f) GoRegion3d_X(GoRegion3d region)
{
    kObj(GoRegion3d, region);

    GoSensor_SyncConfig(obj->sensor);

    return obj->x;
}

GoFx(kStatus) GoRegion3d_SetY(GoRegion3d region, k64f y)
{
    kObj(GoRegion3d, region);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->y = y;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(k64f) GoRegion3d_Y(GoRegion3d region)
{
    kObj(GoRegion3d, region);

    GoSensor_SyncConfig(obj->sensor);

    return obj->y;
}

GoFx(kStatus) GoRegion3d_SetZ(GoRegion3d region, k64f z)
{
    kObj(GoRegion3d, region);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->z = z;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(k64f) GoRegion3d_Z(GoRegion3d region)
{
    kObj(GoRegion3d, region);

    GoSensor_SyncConfig(obj->sensor);

    return obj->z;
}

GoFx(kStatus) GoRegion3d_SetWidth(GoRegion3d region, k64f width)
{
    kObj(GoRegion3d, region);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->width = width;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(k64f) GoRegion3d_Width(GoRegion3d region)
{
    kObj(GoRegion3d, region);

    GoSensor_SyncConfig(obj->sensor);

    return obj->width;
}

GoFx(kStatus) GoRegion3d_SetLength(GoRegion3d region, k64f length)
{
    kObj(GoRegion3d, region);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->length = length;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(k64f) GoRegion3d_Length(GoRegion3d region)
{
    kObj(GoRegion3d, region);

    GoSensor_SyncConfig(obj->sensor);

    return obj->length;
}

GoFx(kStatus) GoRegion3d_SetHeight(GoRegion3d region, k64f height)
{
    kObj(GoRegion3d, region);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->height = height;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(k64f) GoRegion3d_Height(GoRegion3d region)
{
    kObj(GoRegion3d, region);

    GoSensor_SyncConfig(obj->sensor);

    return obj->height;
}
GoFx(kStatus) GoRegion3d_SetZAngle(GoRegion3d region, k64f zAngle)
{
    kObj(GoRegion3d, region);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->zAngle = zAngle;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(k64f) GoRegion3d_ZAngle(GoRegion3d region)
{
    kObj(GoRegion3d, region);

    GoSensor_SyncConfig(obj->sensor);

    return obj->zAngle;
}

kBeginClassEx(Go, GoSurfaceFeature)
    kAddVMethod(GoSurfaceFeature, kObject, VRelease)
kEndClassEx()

GoFx(kStatus) GoSurfaceFeature_Construct(GoSurfaceFeature* feature, kObject sensor, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback(allocator);
    kStatus status;

    kCheck(kAlloc_GetObject(alloc, kTypeOf(GoSurfaceFeature), feature));

    if (!kSuccess(status = GoSurfaceFeature_Init(*feature, kTypeOf(GoSurfaceFeature), sensor, alloc)))
    {
        kAlloc_FreeRef(alloc, feature);
    }

    return status;
}

GoFx(kStatus) GoSurfaceFeature_Init(GoSurfaceFeature feature, kType type, kObject sensor, kAlloc alloc)
{
    kObjR(GoSurfaceFeature, feature);
    kStatus exception;

    kCheck(kObject_Init(feature, type, alloc));
    kZero(obj->xml);
    kZero(obj->xmlItem);
    obj->type = GO_SURFACE_FEATURE_TYPE_AVERAGE;
    obj->region = kNULL;

    obj->sensor = sensor;
    obj->regionEnabled = kTRUE;

    kTry
    {
        kTest(GoRegion3d_Construct(&obj->region, sensor, alloc));
    }
    kCatch(&exception)
    {
        GoSurfaceFeature_VRelease(feature);
        kEndCatch(exception);
    }

    return kOK;
}

GoFx(kStatus) GoSurfaceFeature_VRelease(GoSurfaceFeature feature)
{
    kObj(GoSurfaceFeature, feature);

    kCheck(kDestroyRef(&obj->region));

    return kObject_VRelease(feature);
}

GoFx(kStatus) GoSurfaceFeature_Read(GoSurfaceFeature feature, kXml xml, kXmlItem item)
{
    kObj(GoSurfaceFeature, feature);
    kXmlItem regionItem = kNULL;

    kCheck(kXml_Child32s(xml, item, "Type", &obj->type));
    kCheck(kXml_ChildBool(xml, item, "RegionEnabled", &obj->regionEnabled));

    kCheck(!kIsNull(regionItem = kXml_Child(xml, item, "Region")));
    kCheck(GoRegion3d_Read(obj->region, xml, regionItem));

    return kOK;
}

GoFx(kStatus) GoSurfaceFeature_Write(GoSurfaceFeature feature, kXml xml, kXmlItem item)
{
    kObj(GoSurfaceFeature, feature);
    kXmlItem regionItem = kNULL;

    kCheck(kXml_SetChild32s(xml, item, "Type", obj->type));
    kCheck(kXml_SetChildBool(xml, item, "RegionEnabled", obj->regionEnabled));

    kCheck(kXml_AddItem(xml, item, "Region", &regionItem));
    kCheck(GoRegion3d_Write(obj->region, xml, regionItem));

    return kOK;
}

GoFx(GoSurfaceFeatureType) GoSurfaceFeature_Type(GoSurfaceFeature feature)
{
    kObj(GoSurfaceFeature, feature);

    GoSensor_SyncConfig(obj->sensor);

    return obj->type;
}

GoFx(kStatus) GoSurfaceFeature_SetType(GoSurfaceFeature feature, GoSurfaceFeatureType type)
{
    kObj(GoSurfaceFeature, feature);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->type = type;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(kBool) GoSurfaceFeature_RegionEnabled(GoSurfaceFeature feature)
{
    kObj(GoSurfaceFeature, feature);

    GoSensor_SyncConfig(obj->sensor);

    return obj->regionEnabled;
}

GoFx(kStatus) GoSurfaceFeature_EnableRegion(GoSurfaceFeature feature, kBool enable)
{
    kObj(GoSurfaceFeature, feature);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->regionEnabled = enable;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(GoRegion3d) GoSurfaceFeature_Region(GoSurfaceFeature feature)
{
    kObj(GoSurfaceFeature, feature);

    GoSensor_SyncConfig(obj->sensor);

    return obj->region;
}

kBeginClassEx(Go, GoPointSetRegion)
kAddVMethod(GoPointSetRegion, kObject, VRelease)
kEndClassEx()

GoFx(kStatus) GoPointSetRegion_Construct(GoPointSetRegion* region, kObject sensor, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback(allocator);
    kStatus status;

    kCheck(kAlloc_GetObject(alloc, kTypeOf(GoPointSetRegion), region));

    if (!kSuccess(status = GoPointSetRegion_Init(*region, kTypeOf(GoPointSetRegion), sensor, alloc)))
    {
        kAlloc_FreeRef(alloc, region);
    }

    return status;
}

GoFx(kStatus) GoPointSetRegion_Init(GoPointSetRegion region, kType type, kObject sensor, kAlloc alloc)
{
    kObjR(GoPointSetRegion, region);

    kCheck(kObject_Init(region, type, alloc));
    kZero(obj->xml);
    kZero(obj->xmlItem);
    kZero(obj->points);

    obj->sensor = sensor;

    obj->constantZ = k64F_NULL;
    kCheck(kArrayList_Construct(&obj->points, kTypeOf(kPoint3d64f), 0, alloc));

    return kOK;
}

GoFx(kStatus) GoPointSetRegion_VRelease(GoPointSetRegion region)
{
    kObj(GoPointSetRegion, region);

    kCheck(kDestroyRef(&obj->points));

    return kObject_VRelease(region);
}


GoFx(kStatus) GoPointSetRegion_Read(GoPointSetRegion region, kXml xml, kXmlItem item)
{
    kObj(GoPointSetRegion, region);
    kArrayList points = obj->points;
    kXmlItem nextPoint;
    kXmlItem tempItem = kNULL;

    kCheck(kArrayList_Clear(points));

    for(nextPoint = kXml_FirstChild(xml, item); nextPoint != kNULL; nextPoint = kXml_NextSibling(xml, nextPoint))
    {
        kPoint3d64f point;

        kCheck(kXml_Child64f(xml, nextPoint, "X", &point.x));
        kCheck(kXml_Child64f(xml, nextPoint, "Y", &point.y));
        kCheck(kXml_Child64f(xml, nextPoint, "Z", &point.z));

        kCheck(kArrayList_AddT(points, &point));
    }

    return kOK;
}

GoFx(kStatus) GoPointSetRegion_Write(GoPointSetRegion region, kXml xml, kXmlItem item)
{
    kObj(GoPointSetRegion, region);
    kArrayList points = obj->points;
    kSize pointNum;
    kXmlItem pointItem;

    if (points != kNULL)
    {
        for (pointNum = 0; pointNum < kArrayList_Count(points); ++pointNum)
        {
            kPoint3d64f point = *kArrayList_AtT(points, pointNum, kPoint3d64f);
            kCheck(kXml_AddItem(xml, item, "Point", &pointItem));

            kCheck(GoPointSetRegion_WriteCoord(xml, pointItem, "X", point.x, kFALSE));
            kCheck(GoPointSetRegion_WriteCoord(xml, pointItem, "Y", point.y, kFALSE));

            k64f zValue = obj->constantZ;
            kBool readOnly;
            if (zValue == k64F_NULL)
            {
                readOnly = kFALSE;
                zValue = point.z;
            }
            else
            {
                readOnly = kTRUE;
                zValue = obj->constantZ;
            }

            kCheck(GoPointSetRegion_WriteCoord(xml, pointItem, "Z", zValue, readOnly));
        }
    }

    return kOK;
}

GoFx(kStatus) GoPointSetRegion_WriteCoord(kXml xml, kXmlItem item, kChar *name, k64f value, kBool readOnly)
{
    kXmlItem coordItem;

    kCheck(kXml_AddItem(xml, item, name, &coordItem));

    kCheck(kXml_SetItem64f(xml, coordItem, value));
    kCheck(kXml_SetAttr64f(xml, coordItem, "value", value));
    kCheck(kXml_SetAttrBool(xml, coordItem, "readonly", readOnly));

    return kOK;
}


GoFx(kStatus) GoPointSetRegion_DisableConstantZ(GoPointSetRegion region)
{
    kObj(GoPointSetRegion, region);
    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->constantZ = k64F_NULL;

    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(kStatus) GoPointSetRegion_SetConstantZ(GoPointSetRegion region, k64f constantZ)
{
    kObj(GoPointSetRegion, region);
    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->constantZ = constantZ;

    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(const kPoint3d64f*) GoPointSetRegion_PointAt(GoPointSetRegion region, kSize index)
{
    kObj(GoPointSetRegion, region);

    GoSensor_SyncConfig(obj->sensor);

    return kArrayList_AtT(obj->points, index, kPoint3d64f);
}

GoFx(kStatus) GoPointSetRegion_SetPointAt(GoPointSetRegion region, kSize index, const kPoint3d64f* point)
{
    kObj(GoPointSetRegion, region);
    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    *kArrayList_AtT(obj->points, index, kPoint3d64f) = *point;

    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(kSize) GoPointSetRegion_PointCount(GoPointSetRegion region)
{
    kObj(GoPointSetRegion, region);

    GoSensor_SyncConfig(obj->sensor);

    return kArrayList_Count(obj->points);
}

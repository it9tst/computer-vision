/**
 * @file    GoProfileToolUtils.c
 *
 * @internal
 * Copyright (C) 2016-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <GoSdk/Tools/GoProfileToolUtils.h>
#include <GoSdk/GoSensor.h>

kBeginClassEx(Go, GoProfileRegion)
kEndClassEx()

GoFx(kStatus) GoProfileRegion_Construct(GoProfileRegion* region, kObject sensor, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback(allocator);
    kStatus status;

    kCheck(kAlloc_GetObject(alloc, kTypeOf(GoProfileRegion), region));

    if (!kSuccess(status = GoProfileRegion_Init(*region, kTypeOf(GoProfileRegion), sensor, alloc)))
    {
        kAlloc_FreeRef(alloc, region);
    }

    return status;
}

GoFx(kStatus) GoProfileRegion_Init(GoProfileRegion region, kType type, kObject sensor, kAlloc alloc)
{
    kObjR(GoProfileRegion, region);

    kCheck(kObject_Init(region, type, alloc));
    kZero(obj->xml);
    kZero(obj->xmlItem);

    obj->sensor = sensor;

    obj->x = 0;
    obj->z = 0;
    obj->width = 1.0;
    obj->height = 1.0;

    return kOK;
}

GoFx(kStatus) GoProfileRegion_Read(GoProfileRegion region, kXml xml, kXmlItem item)
{
    kObj(GoProfileRegion, region);

    kCheck(kXml_Child64f(xml, item, "X", &obj->x));
    kCheck(kXml_Child64f(xml, item, "Z", &obj->z));
    kCheck(kXml_Child64f(xml, item, "Width", &obj->width));
    kCheck(kXml_Child64f(xml, item, "Height", &obj->height));

    return kOK;
}

GoFx(kStatus) GoProfileRegion_Write(GoProfileRegion region, kXml xml, kXmlItem item)
{
    kObj(GoProfileRegion, region);

    kCheck(kXml_SetChild64f(xml, item, "X", obj->x));
    kCheck(kXml_SetChild64f(xml, item, "Z", obj->z));
    kCheck(kXml_SetChild64f(xml, item, "Width", obj->width));
    kCheck(kXml_SetChild64f(xml, item, "Height", obj->height));

    return kOK;
}


GoFx(kStatus) GoProfileRegion_SetX(GoProfileRegion region, k64f x)
{
    kObj(GoProfileRegion, region);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->x = x;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(k64f) GoProfileRegion_X(GoProfileRegion region)
{
    kObj(GoProfileRegion, region);

    GoSensor_SyncConfig(obj->sensor);

    return obj->x;
}

GoFx(kStatus) GoProfileRegion_SetZ(GoProfileRegion region, k64f z)
{
    kObj(GoProfileRegion, region);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->z = z;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(k64f) GoProfileRegion_Z(GoProfileRegion region)
{
    kObj(GoProfileRegion, region);

    GoSensor_SyncConfig(obj->sensor);

    return obj->z;
}

GoFx(kStatus) GoProfileRegion_SetWidth(GoProfileRegion region, k64f width)
{
    kObj(GoProfileRegion, region);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->width = width;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(k64f) GoProfileRegion_Width(GoProfileRegion region)
{
    kObj(GoProfileRegion, region);

    GoSensor_SyncConfig(obj->sensor);

    return obj->width;
}

GoFx(kStatus) GoProfileRegion_SetHeight(GoProfileRegion region, k64f height)
{
    kObj(GoProfileRegion, region);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->height = height;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(k64f) GoProfileRegion_Height(GoProfileRegion region)
{
    kObj(GoProfileRegion, region);

    GoSensor_SyncConfig(obj->sensor);

    return obj->height;
}

kBeginClassEx(Go, GoProfileFeature)
    kAddVMethod(GoProfileFeature, kObject, VRelease)
kEndClassEx()

GoFx(kStatus) GoProfileFeature_Construct(GoProfileFeature* feature, kObject sensor, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback(allocator);
    kStatus status;

    kCheck(kAlloc_GetObject(alloc, kTypeOf(GoProfileFeature), feature));

    if (!kSuccess(status = GoProfileFeature_Init(*feature, kTypeOf(GoProfileFeature), sensor, alloc)))
    {
        kAlloc_FreeRef(alloc, feature);
    }

    return status;
}

GoFx(kStatus) GoProfileFeature_Init(GoProfileFeature feature, kType type, kObject sensor, kAlloc alloc)
{
    kObjR(GoProfileFeature, feature);
    kStatus exception;

    kCheck(kObject_Init(feature, type, alloc));
    kZero(obj->xml);
    kZero(obj->xmlItem);
    obj->regionEnabled = kFALSE;
    obj->region = kNULL;

    obj->sensor = sensor;
    obj->type = GO_PROFILE_FEATURE_TYPE_MAX_Z;

    kTry
    {
        kTest(GoProfileRegion_Construct(&obj->region, sensor, alloc));
    }
    kCatch(&exception)
    {
        GoProfileFeature_VRelease(feature);
        kEndCatch(exception);
    }

    return kOK;
}

GoFx(kStatus) GoProfileFeature_VRelease(GoProfileFeature feature)
{
    kObj(GoProfileFeature, feature);

    kCheck(kDestroyRef(&obj->region));

    return kObject_VRelease(feature);
}

GoFx(kStatus) GoProfileFeature_Read(GoProfileFeature feature, kXml xml, kXmlItem item)
{
    kObj(GoProfileFeature, feature);
    kXmlItem regionItem = kNULL;

    kCheck(kXml_Child32s(xml, item, "Type", (k32s*) &obj->type));

    if (kXml_ChildExists(xml, item, "RegionEnabled"))
    {
        kCheck(kXml_ChildBool(xml, item, "RegionEnabled", &obj->regionEnabled));
    }

    kCheck(!kIsNull(regionItem = kXml_Child(xml, item, "Region")));
    kCheck(GoProfileRegion_Read(obj->region, xml, regionItem));

    return kOK;
}

GoFx(kStatus) GoProfileFeature_Write(GoProfileFeature feature, kXml xml, kXmlItem item)
{
    kObj(GoProfileFeature, feature);
    kXmlItem regionItem = kNULL;

    kCheck(kXml_SetChild32s(xml, item, "Type", obj->type));

    kCheck(kXml_SetChildBool(xml, item, "RegionEnabled", obj->regionEnabled));
    kCheck(kXml_AddItem(xml, item, "Region", &regionItem));
    kCheck(GoProfileRegion_Write(obj->region, xml, regionItem));

    return kOK;
}


GoFx(kStatus) GoProfileFeature_SetType(GoProfileFeature feature, GoProfileFeatureType type)
{
    kObj(GoProfileFeature, feature);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->type = type;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(GoProfileFeatureType) GoProfileFeature_Type(GoProfileFeature feature)
{
    kObj(GoProfileFeature, feature);

    GoSensor_SyncConfig(obj->sensor);

    return obj->type;
}

GoFx(GoProfileRegion) GoProfileFeature_Region(GoProfileFeature feature)
{
    kObj(GoProfileFeature, feature);

    GoSensor_SyncConfig(obj->sensor);

    return obj->region;
}

GoFx(kStatus) GoProfileFeature_EnableRegion(GoProfileFeature feature, kBool enable)
{
    kObj(GoProfileFeature, feature);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->regionEnabled = enable;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(kBool) GoProfileFeature_RegionEnabled(GoProfileFeature feature)
{
    kObj(GoProfileFeature, feature);

    GoSensor_SyncConfig(obj->sensor);

    return obj->regionEnabled;
}


kBeginClassEx(Go, GoProfileLineRegion)
    kAddVMethod(GoProfileLineFittingRegion, kObject, VRelease)
kEndClassEx()

GoFx(kStatus) GoProfileLineFittingRegion_Construct(GoProfileLineFittingRegion* lineRegion, kObject sensor, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback(allocator);
    kStatus status;

    kCheck(kAlloc_GetObject(alloc, kTypeOf(GoProfileLineFittingRegion), lineRegion));

    if (!kSuccess(status = GoProfileLineFittingRegion_Init(*lineRegion, kTypeOf(GoProfileLineFittingRegion), sensor, alloc)))
    {
        kAlloc_FreeRef(alloc, lineRegion);
    }

    return status;
}

GoFx(kStatus) GoProfileLineFittingRegion_Init(GoProfileLineFittingRegion lineRegion, kType type, kObject sensor, kAlloc alloc)
{
    kObjR(GoProfileLineFittingRegion, lineRegion);
    k32u i;
    kStatus exception;

    kCheck(kObject_Init(lineRegion, type, alloc));
    kZero(obj->xml);
    kZero(obj->xmlItem);
    kZero(obj->regionEnabled);
    kZero(obj->regions);

    obj->sensor = sensor;
    obj->regionCount = 1;

    kTry
    {
        for (i = 0; i < kCountOf(obj->regions); ++i)
        {
            kTest(GoProfileRegion_Construct(&obj->regions[i], sensor, alloc));     //max number of regions constructed regardless of region count
        }
    }
    kCatch(&exception)
    {
        GoProfileLineFittingRegion_VRelease(lineRegion);
        kEndCatch(exception);
    }

    return kOK;
}

GoFx(kStatus) GoProfileLineFittingRegion_VRelease(GoProfileLineFittingRegion lineRegion)
{
    kObj(GoProfileLineFittingRegion, lineRegion);
    k32u i;

    for (i = 0; i < kCountOf(obj->regions); ++i)
    {
        kCheck(kDestroyRef(&obj->regions[i]));
    }

    return kObject_VRelease(lineRegion);
}

GoFx(kStatus) GoProfileLineFittingRegion_Read(GoProfileLineFittingRegion lineRegion, kXml xml, kXmlItem item)
{
    kObj(GoProfileLineFittingRegion, lineRegion);
    kXmlItem regionItem = kNULL;
    kXmlItem regionsItem = kNULL;

    kCheck(!kIsNull(regionsItem = kXml_Child(xml, item, "Regions")));

    // Read first line region parameters.
    kCheck(!kIsNull(regionItem = kXml_Child(xml, regionsItem, "Region")));

    if (kXml_ChildExists(xml, regionItem, "RegionEnabled"))
    {
        kCheck(kXml_ChildBool(xml, regionItem, "RegionEnabled", &obj->regionEnabled[0]));
    }

    // TODO: confirm if this code is for backwards compatibility where older
    // sensors do not send the region count, but always sends all the regions
    // supported, thereby forcing SDK client to assume region count is the
    // same as the number of regions supported.
    kCheck(GoProfileRegion_Read(obj->regions[0], xml, regionItem));
    obj->regionCount = 1;

    // Read second line region parameters.
    kCheck(!kIsNull(regionItem = kXml_NextSibling(xml, regionItem)) &&
            (strcmp(kXml_ItemName(xml, regionItem), "Region") == 0));

    kCheck(kXml_ChildBool(xml, regionItem, "RegionEnabled", &obj->regionEnabled[1]));

    kCheck(GoProfileRegion_Read(obj->regions[1], xml, regionItem));
    obj->regionCount++;

    // If configuration file has the region count tag, get the region count
    // from the configuration file, rather than relying on counting how many
    // regions were sent.
    if (kXml_ChildExists(xml, item, "RegionCount"))
    {
        kCheck(kXml_Child32u(xml, item, "RegionCount", &obj->regionCount));
    }

    return kOK;
}

GoFx(kStatus) GoProfileLineFittingRegion_Write(GoProfileLineFittingRegion lineRegion, kXml xml, kXmlItem item)
{
    kObj(GoProfileLineFittingRegion, lineRegion);
    kXmlItem regionItem = kNULL;
    kXmlItem regionsItem = kNULL;
    k32u i;

    // This version of SDK supports the region count tag, so write it into
    // the XML configuration file.
    kCheck(kXml_SetChild32u(xml, item, "RegionCount", obj->regionCount));
    kCheck(kXml_AddItem(xml, item, "Regions", &regionsItem));

    // For backwards compatibility with older sensor firmware that expected
    // the maximum number of supported regions to be in the XML configuration
    // file, and therefore read that number of regions, always write the
    // maximum supported regions into the configuration file.
    for (i = 0; i < GO_PROFILE_LINE_REGION_MAX_FITTING_REGIONS; ++i)
    {
        kCheck(kXml_AddItem(xml, regionsItem, "Region", &regionItem));
        kCheck(kXml_SetChildBool(xml, regionItem, "RegionEnabled", obj->regionEnabled[i]));
        kCheck(GoProfileRegion_Write(obj->regions[i], xml, regionItem));
    }

    return kOK;
}

GoFx(k32u) GoProfileLineFittingRegion_RegionCount(GoProfileLineFittingRegion lineRegion)
{
    kObj(GoProfileLineFittingRegion, lineRegion);

    GoSensor_SyncConfig(obj->sensor);

    return obj->regionCount;
}

GoFx(kStatus) GoProfileLineFittingRegion_SetRegionCount(GoProfileLineFittingRegion lineRegion, kSize count)
{
    kObj(GoProfileLineFittingRegion, lineRegion);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    kCheckArgs(count <= GO_PROFILE_LINE_REGION_MAX_FITTING_REGIONS);
    obj->regionCount = (k32u) count;

    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(GoProfileRegion) GoProfileLineFittingRegion_RegionAt(GoProfileLineFittingRegion lineRegion, kSize index)
{
    kObj(GoProfileLineFittingRegion, lineRegion);

    GoSensor_SyncConfig(obj->sensor);

    kAssert(index < obj->regionCount);

    return obj->regions[index];
}

GoFx(kStatus) GoProfileLineFittingRegion_EnableRegionAt(GoProfileLineFittingRegion lineRegion, kSize index, kBool enable)
{
    kObj(GoProfileLineFittingRegion, lineRegion);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    kAssert(index < obj->regionCount);

    obj->regionEnabled[index] = enable;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(kBool) GoProfileLineFittingRegion_RegionEnabledAt(GoProfileLineFittingRegion lineRegion, kSize index)
{
    kObj(GoProfileLineFittingRegion, lineRegion);

    GoSensor_SyncConfig(obj->sensor);
    kAssert(index < obj->regionCount);

    return obj->regionEnabled[index];
}

kBeginClassEx(Go, GoProfileEdge)
    kAddVMethod(GoProfileEdge, kObject, VRelease)
kEndClassEx()

GoFx(kStatus) GoProfileEdge_Construct(GoProfileEdge* edge, kObject sensor, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback(allocator);
    kStatus status;

    kCheck(kAlloc_GetObject(alloc, kTypeOf(GoProfileEdge), edge));

    if (!kSuccess(status = GoProfileEdge_Init(*edge, kTypeOf(GoProfileEdge), sensor, alloc)))
    {
        kAlloc_FreeRef(alloc, edge);
    }

    return status;
}

GoFx(kStatus) GoProfileEdge_Init(GoProfileEdge edge, kType type, kObject sensor, kAlloc alloc)
{
    kObjR(GoProfileEdge, edge);
    kStatus exception;

    kCheck(kObject_Init(edge, type, alloc));
    kZero(obj->xml);
    kZero(obj->xmlItem);
    obj->edgeType = GO_PROFILE_EDGE_TYPE_TANGENT;
    obj->regionEnabled = kFALSE;
    obj->region = kNULL;
    obj->maxVoidWidth = 0.0;
    obj->minDepth = 0.0;

    obj->sensor = sensor;

    obj->surfaceWidth = 5.0;
    obj->surfaceOffset = 2.0;
    obj->nominalRadius = 2.0;
    obj->edgeAngle = 90.0;

    kTry
    {
        kTest(GoProfileRegion_Construct(&obj->region, sensor, alloc));
    }
    kCatch(&exception)
    {
        GoProfileEdge_VRelease(edge);
        kEndCatch(exception);
    }

    return kOK;
}

GoFx(kStatus) GoProfileEdge_VRelease(GoProfileEdge edge)
{
    kObj(GoProfileEdge, edge);

    kCheck(kDestroyRef(&obj->region));

    return kObject_VRelease(edge);
}

GoFx(kStatus) GoProfileEdge_Read(GoProfileEdge edge, kXml xml, kXmlItem item)
{
    kObj(GoProfileEdge, edge);
    kXmlItem regionItem = kNULL;

    kCheck(kXml_Child64f(xml, item, "MaxVoidWidth", &obj->maxVoidWidth));
    kCheck(kXml_Child64f(xml, item, "MinDepth", &obj->minDepth));
    kCheck(kXml_Child64f(xml, item, "SurfaceWidth", &obj->surfaceWidth));
    kCheck(kXml_Child64f(xml, item, "SurfaceOffset", &obj->surfaceOffset));
    kCheck(kXml_Child64f(xml, item, "NominalRadius", &obj->nominalRadius));
    kCheck(kXml_Child64f(xml, item, "EdgeAngle", &obj->edgeAngle));
    kCheck(kXml_Child32s(xml, item, "EdgeType", (k32s*) &obj->edgeType));

    if (kXml_ChildExists(xml, item, "RegionEnabled"))
    {
        kCheck(kXml_ChildBool(xml, item, "RegionEnabled", &obj->regionEnabled));
    }

    kCheck(!kIsNull(regionItem = kXml_Child(xml, item, "Region")));
    kCheck(GoProfileRegion_Read(obj->region, xml, regionItem));

    return kOK;
}

GoFx(kStatus) GoProfileEdge_Write(GoProfileEdge edge, kXml xml, kXmlItem item)
{
    kObj(GoProfileEdge, edge);
    kXmlItem regionItem = kNULL;

    kCheck(kXml_SetChild64f(xml, item, "MaxVoidWidth", obj->maxVoidWidth));
    kCheck(kXml_SetChild64f(xml, item, "MinDepth", obj->minDepth));
    kCheck(kXml_SetChild64f(xml, item, "SurfaceWidth", obj->surfaceWidth));
    kCheck(kXml_SetChild64f(xml, item, "SurfaceOffset", obj->surfaceOffset));
    kCheck(kXml_SetChild64f(xml, item, "NominalRadius", obj->nominalRadius));
    kCheck(kXml_SetChild64f(xml, item, "EdgeAngle", obj->edgeAngle));
    kCheck(kXml_SetChild32s(xml, item, "EdgeType", obj->edgeType));

    kCheck(kXml_SetChildBool(xml, item, "RegionEnabled", obj->regionEnabled));
    kCheck(kXml_AddItem(xml, item, "Region", &regionItem));
    kCheck(GoProfileRegion_Write(obj->region, xml, regionItem));

    return kOK;
}

GoFx(kStatus) GoProfileEdge_SetType(GoProfileEdge edge, GoProfileEdgeType type)
{
    kObj(GoProfileEdge, edge);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->edgeType = type;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(GoProfileEdgeType) GoProfileEdge_Type(GoProfileEdge edge)
{
    kObj(GoProfileEdge, edge);

    GoSensor_SyncConfig(obj->sensor);

    return obj->edgeType;
}

GoFx(GoProfileRegion) GoProfileEdge_Region(GoProfileEdge edge)
{
    kObj(GoProfileEdge, edge);

    GoSensor_SyncConfig(obj->sensor);

    return obj->region;
}

GoFx(kStatus) GoProfileEdge_EnableRegion(GoProfileEdge edge, kBool enable)
{
    kObj(GoProfileEdge, edge);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->regionEnabled = enable;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(kBool) GoProfileEdge_RegionEnabled(GoProfileEdge edge)
{
    kObj(GoProfileEdge, edge);

    GoSensor_SyncConfig(obj->sensor);

    return obj->regionEnabled;
}

GoFx(kStatus) GoProfileEdge_SetVoidWidthMax(GoProfileEdge edge, k64f width)
{
    kObj(GoProfileEdge, edge);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->maxVoidWidth = width;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(k64f) GoProfileEdge_VoidWidthMax(GoProfileEdge edge)
{
    kObj(GoProfileEdge, edge);

    GoSensor_SyncConfig(obj->sensor);

    return obj->maxVoidWidth;
}

GoFx(kStatus) GoProfileEdge_SetDepthMin(GoProfileEdge edge, k64f depth)
{
    kObj(GoProfileEdge, edge);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->minDepth = depth;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(k64f) GoProfileEdge_DepthMin(GoProfileEdge edge)
{
    kObj(GoProfileEdge, edge);

    GoSensor_SyncConfig(obj->sensor);

    return obj->minDepth;
}

GoFx(kStatus) GoProfileEdge_SetSurfaceWidth(GoProfileEdge edge, k64f width)
{
    kObj(GoProfileEdge, edge);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->surfaceWidth = width;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(k64f) GoProfileEdge_SurfaceWidth(GoProfileEdge edge)
{
    kObj(GoProfileEdge, edge);

    GoSensor_SyncConfig(obj->sensor);

    return obj->surfaceWidth;
}

GoFx(kStatus) GoProfileEdge_SetSurfaceOffset(GoProfileEdge edge, k64f offset)
{
    kObj(GoProfileEdge, edge);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->surfaceOffset = offset;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(k64f) GoProfileEdge_SurfaceOffset(GoProfileEdge edge)
{
    kObj(GoProfileEdge, edge);

    GoSensor_SyncConfig(obj->sensor);

    return obj->surfaceOffset;
}

GoFx(kStatus) GoProfileEdge_SetNominalRadius(GoProfileEdge edge, k64f radius)
{
    kObj(GoProfileEdge, edge);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->nominalRadius = radius;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(k64f) GoProfileEdge_NominalRadius(GoProfileEdge edge)
{
    kObj(GoProfileEdge, edge);

    GoSensor_SyncConfig(obj->sensor);

    return obj->nominalRadius;
}

GoFx(kStatus) GoProfileEdge_SetEdgeAngle(GoProfileEdge edge, k64f angle)
{
    kObj(GoProfileEdge, edge);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->edgeAngle = angle;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(k64f) GoProfileEdge_EdgeAngle(GoProfileEdge edge)
{
    kObj(GoProfileEdge, edge);

    GoSensor_SyncConfig(obj->sensor);

    return obj->edgeAngle;
}

GoFx(kStatus) GoProfileLineRegion_Construct(GoProfileLineRegion* lineRegion, kObject sensor, kAlloc allocator)
{
    return GoProfileLineFittingRegion_Construct(lineRegion, sensor, allocator);
}

GoFx(kStatus) GoProfileLineRegion_Init(GoProfileLineRegion lineRegion, kType type, kObject sensor, kAlloc alloc)
{
    return GoProfileLineFittingRegion_Init(lineRegion, type, sensor, alloc);
}

GoFx(kStatus) GoProfileLineRegion_VRelease(GoProfileLineRegion lineRegion)
{
    return GoProfileLineFittingRegion_VRelease(lineRegion);
}

GoFx(kStatus) GoProfileLineRegion_Read(GoProfileLineRegion lineRegion, kXml xml, kXmlItem item)
{
    return GoProfileLineFittingRegion_Read(lineRegion, xml, item);
}

GoFx(kStatus) GoProfileLineRegion_Write(GoProfileLineRegion lineRegion, kXml xml, kXmlItem item)
{
    return GoProfileLineFittingRegion_Write(lineRegion, xml, item);
}

GoFx(k32u) GoProfileLineRegion_RegionCount(GoProfileLineRegion lineRegion)
{
    return GoProfileLineFittingRegion_RegionCount(lineRegion);
}

GoFx(kStatus) GoProfileLineRegion_SetRegionCount(GoProfileLineRegion lineRegion, kSize count)
{
    return GoProfileLineFittingRegion_SetRegionCount(lineRegion, count); 
}

GoFx(GoProfileRegion) GoProfileLineRegion_RegionAt(GoProfileLineRegion lineRegion, kSize index)
{
    return GoProfileLineFittingRegion_RegionAt(lineRegion, index);
}

GoFx(kStatus) GoProfileLineRegion_EnableRegionAt(GoProfileLineRegion lineRegion, kSize index, kBool enable)
{
    return GoProfileLineFittingRegion_EnableRegionAt(lineRegion, index, enable);
}

GoFx(kBool) GoProfileLineRegion_RegionEnabledAt(GoProfileLineRegion lineRegion, kSize index)
{
    return GoProfileLineFittingRegion_RegionEnabledAt(lineRegion, index);
}

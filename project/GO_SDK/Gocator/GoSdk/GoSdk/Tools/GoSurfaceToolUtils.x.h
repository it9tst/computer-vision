/** 
 * @file    GoSurfaceToolUtils.x.h
 *
 * @internal
 * Copyright (C) 2016-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef GO_SURFACE_TOOL_UTILS_X_H
#define GO_SURFACE_TOOL_UTILS_X_H

#include <kApi/Data/kXml.h>

typedef struct GoSurfaceRegion2dClass
{
    kObjectClass base; 
    kObject sensor; 

    kXml xml;
    kXmlItem xmlItem;

    k64f x;
    k64f y;
    k64f width;
    k64f length;

    //Added H.Ware
    k64f zAngle;
} GoSurfaceRegion2dClass; 

kDeclareClassEx(Go, GoSurfaceRegion2d, kObject)

GoFx(kStatus) GoSurfaceRegion2d_Construct(GoSurfaceRegion2d* region, kObject sensor, kAlloc allocator);
GoFx(kStatus) GoSurfaceRegion2d_Init(GoSurfaceRegion2d region, kType type, kObject sensor, kAlloc alloc);
GoFx(kStatus) GoSurfaceRegion2d_VRelease(GoSurfaceRegion2d region);

GoFx(kStatus) GoSurfaceRegion2d_Read(GoSurfaceRegion2d region, kXml xml, kXmlItem item);
GoFx(kStatus) GoSurfaceRegion2d_Write(GoSurfaceRegion2d region, kXml xml, kXmlItem item); 


typedef struct GoRegion3dClass
{
    kObjectClass base; 
    kObject sensor; 

    kXml xml;
    kXmlItem xmlItem;

    k64f x;
    k64f y;
    k64f z;
    k64f width;
    k64f length;
    k64f height;

    //Added H.Ware
    k64f zAngle;
} GoRegion3dClass; 

kDeclareClassEx(Go, GoRegion3d, kObject)

GoFx(kStatus) GoRegion3d_Construct(GoRegion3d* region, kObject sensor, kAlloc allocator);
GoFx(kStatus) GoRegion3d_Init(GoRegion3d region, kType type, kObject sensor, kAlloc alloc);
GoFx(kStatus) GoRegion3d_VRelease(GoRegion3d region);

GoFx(kStatus) GoRegion3d_Read(GoRegion3d region, kXml xml, kXmlItem item);
GoFx(kStatus) GoRegion3d_Write(GoRegion3d region, kXml xml, kXmlItem item); 

typedef struct GoPointSetRegionClass
{
    kObjectClass base; 
    kObject sensor; 

    kXml xml;
    kXmlItem xmlItem;

    k64f constantZ;
    kArrayList points;

    //Added H.Ware
    //k64f zAngle;
} GoPointSetRegionClass; 

kDeclareClassEx(Go, GoPointSetRegion, kObject)

GoFx(kStatus) GoPointSetRegion_Construct(GoPointSetRegion* region, kObject sensor, kAlloc allocator);
GoFx(kStatus) GoPointSetRegion_Init(GoPointSetRegion region, kType type, kObject sensor, kAlloc alloc);
GoFx(kStatus) GoPointSetRegion_VRelease(GoPointSetRegion region);

GoFx(kStatus) GoPointSetRegion_Read(GoPointSetRegion region, kXml xml, kXmlItem item);
GoFx(kStatus) GoPointSetRegion_Write(GoPointSetRegion region, kXml xml, kXmlItem item); 
GoFx(kStatus) GoPointSetRegion_WriteCoord(kXml xml, kXmlItem item, kChar *name, k64f value, kBool readOnly);


typedef struct GoCylinderRegionClass
{
    kObjectClass base; 
    kObject sensor; 

    kXml xml;
    kXmlItem xmlItem;

    k64f x;
    k64f y;
    k64f z;
    k64f radius;
    k64f height;
} GoCylinderRegionClass; 

kDeclareClassEx(Go, GoCylinderRegion, kObject)

GoFx(kStatus) GoCylinderRegion_Construct(GoCylinderRegion* region, kObject sensor, kAlloc allocator);
GoFx(kStatus) GoCylinderRegion_Init(GoCylinderRegion region, kType type, kObject sensor, kAlloc alloc);
GoFx(kStatus) GoCylinderRegion_VRelease(GoCylinderRegion region);

GoFx(kStatus) GoCylinderRegion_Read(GoCylinderRegion region, kXml xml, kXmlItem item);
GoFx(kStatus) GoCylinderRegion_Write(GoCylinderRegion region, kXml xml, kXmlItem item); 



typedef struct GoSurfaceFeatureClass
{
    kObjectClass base; 
    kObject sensor; 

    kXml xml;
    kXmlItem xmlItem;

    GoSurfaceFeatureType type;
    kBool regionEnabled;
    GoRegion3d region;
} GoSurfaceFeatureClass; 

kDeclareClassEx(Go, GoSurfaceFeature, kObject)

GoFx(kStatus) GoSurfaceFeature_Construct(GoSurfaceFeature* feature, kObject sensor, kAlloc allocator);
GoFx(kStatus) GoSurfaceFeature_Init(GoSurfaceFeature feature, kType type, kObject sensor, kAlloc alloc);
GoFx(kStatus) GoSurfaceFeature_VRelease(GoSurfaceFeature feature);

GoFx(kStatus) GoSurfaceFeature_Read(GoSurfaceFeature feature, kXml xml, kXmlItem item);
GoFx(kStatus) GoSurfaceFeature_Write(GoSurfaceFeature feature, kXml xml, kXmlItem item); 

#endif

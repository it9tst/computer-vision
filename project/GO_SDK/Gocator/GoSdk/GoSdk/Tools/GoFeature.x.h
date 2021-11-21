/** 
 * @file    GoFeature.x.h
 *
 * @internal
 * Copyright (C) 2017-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef GO_FEATURE_X_H
#define GO_FEATURE_X_H

#include <kApi/Data/kXml.h>

typedef struct GoFeatureVTable
{    
    kObjectVTable base; 
    kStatus (kCall* VInit)(GoFeature feature, kType type, kObject sensor, kObject srcTool, kAlloc allocator); 
    kStatus (kCall* VWrite)(GoFeature feature, kXml xml, kXmlItem item); 
    kStatus (kCall* VRead)(GoFeature feature, kXml xml, kXmlItem item); 
} GoFeatureVTable; 

typedef struct GoFeatureClass
{
    kObjectClass base;
    
    kObject sensor;
    kObject srcTool;

    kXml xml;
    kXmlItem xmlItem;

    kText128 name;
    k32s id;

    GoFeatureType typeId;

    kBool enabled;
    kText64 type;
    GoFeatureDataType dataType;
} GoFeatureClass; 

kDeclareVirtualClassEx(Go, GoFeature, kObject)

GoFx(kStatus) GoFeature_Construct(GoFeature* feature, 
                                        kType type, 
                                        kObject sensor, 
                                        kObject srcTool,
                                        kAlloc allocator);
GoFx(kStatus) GoFeature_VInit(GoFeature feature, 
                                 kType type, 
                                 kObject sensor, 
                                 kObject srcTool,
                                 kAlloc alloc); //note the lack of typeId compared to the Init below
GoFx(kStatus) GoFeature_Init(GoFeature feature, 
                                kType type, 
                                GoFeatureType typeId, 
                                kObject sensor, 
                                kObject srcTool,
                                GoFeatureDataType dataType,
                                kAlloc alloc);
GoFx(kStatus) GoFeature_VRelease(GoFeature feature);

GoFx(kStatus) GoFeature_VRead(GoFeature feature, kXml xml, kXmlItem item);
GoFx(kStatus) GoFeature_VWrite(GoFeature feature, kXml xml, kXmlItem item); 
GoFx(kStatus) GoFeature_Read(GoFeature feature, kXml xml, kXmlItem item);
GoFx(kStatus) GoFeature_Write(GoFeature feature, kXml xml, kXmlItem item); 

GoFx(kObject) GoFeature_Sensor(GoFeature feature);

#endif

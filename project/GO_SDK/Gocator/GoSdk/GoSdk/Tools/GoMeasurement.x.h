/** 
 * @file    GoMeasurement.x.h
 *
 * @internal
 * Copyright (C) 2016-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef GO_MEASUREMENT_X_H
#define GO_MEASUREMENT_X_H

#include <kApi/Data/kXml.h>

typedef struct GoMeasurementVTable
{    
    kObjectVTable base; 
    kStatus (kCall* VInit)(GoMeasurement measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc allocator); 
    kStatus (kCall* VWrite)(GoMeasurement measurement, kXml xml, kXmlItem item); 
    kStatus (kCall* VRead)(GoMeasurement measurement, kXml xml, kXmlItem item); 
} GoMeasurementVTable; 

typedef struct GoMeasurementClass
{
    kObjectClass base;
    
    kObject sensor;
    kObject srcTool;

    kXml xml;
    kXmlItem xmlItem;

    kText128 name;
    k32s id;

    kBool isFilterable; 
    kBool enabled;
    kBool holdEnabled;
    kBool smoothingEnabled;
    k64s smoothingWindow;
    kBool preserveInvalidsEnabled;
    k64f scale;
    k64f offset;
    k64f decisionMin;
    k64f decisionMax;
    GoMeasurementType typeId;
} GoMeasurementClass; 

kDeclareVirtualClassEx(Go, GoMeasurement, kObject)

GoFx(kStatus) GoMeasurement_Construct(GoMeasurement* measurement, 
                                        kType type, 
                                        kObject sensor, 
                                        kObject srcTool, 
                                        kBool isFilterable, 
                                        kAlloc allocator);
GoFx(kStatus) GoMeasurement_VInit(GoMeasurement measurement, 
                                 kType type, 
                                 kObject sensor, 
                                 kObject srcTool, 
                                 kBool isFilterable, 
                                 kAlloc alloc); //note the lack of typeId compared to the Init below
GoFx(kStatus) GoMeasurement_Init(GoMeasurement measurement, 
                                kType type, 
                                GoMeasurementType typeId, 
                                kObject sensor, 
                                kObject srcTool, 
                                kBool isFilterable, 
                                kAlloc alloc);
GoFx(kStatus) GoMeasurement_VRelease(GoMeasurement measurement);

GoFx(kStatus) GoMeasurement_VRead(GoMeasurement measurement, kXml xml, kXmlItem item);
GoFx(kStatus) GoMeasurement_VWrite(GoMeasurement measurement, kXml xml, kXmlItem item); 
GoFx(kStatus) GoMeasurement_Read(GoMeasurement measurement, kXml xml, kXmlItem item);
GoFx(kStatus) GoMeasurement_Write(GoMeasurement measurement, kXml xml, kXmlItem item); 

GoFx(kObject) GoMeasurement_Sensor(GoMeasurement measurement);

#endif

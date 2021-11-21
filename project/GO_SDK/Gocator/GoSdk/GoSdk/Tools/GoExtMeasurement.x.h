/**
* @file    GoExtMeasurements.x.h
*
* @internal
* Copyright (C) 2016-2021 by LMI Technologies Inc.
* Licensed under the MIT License.
* Redistributed files must retain the above copyright notice.
*/
#ifndef GO_SDK_EXTMEASUREMENTS_X_H
#define GO_SDK_EXTMEASUREMENTS_X_H

#include <kApi/Data/kXml.h>
#include <GoSdk/Tools/GoMeasurement.h>
#include <GoSdk/Tools/GoExtParam.h>

typedef struct GoExtMeasurementVTable
{
    GoMeasurementVTable base;
    kStatus(kCall* VInit)(GoExtMeasurement measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc allocator);
    kStatus(kCall* VWrite)(GoExtMeasurement measurement, kXml xml, kXmlItem item);
    kStatus(kCall* VRead)(GoExtMeasurement measurement, kXml xml, kXmlItem item);
} GoExtMeasurementVTable;

typedef struct GoExtMeasurementClass
{
    GoMeasurementClass base;
    GoExtParams params;
    kText64 type;
} GoExtMeasurementClass;

kDeclareVirtualClassEx(Go, GoExtMeasurement, GoMeasurement)

GoFx(kStatus) GoExtMeasurement_Init(GoExtMeasurement measurement, kType type, GoMeasurementType typeId, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);
GoFx(kStatus) GoExtMeasurement_Construct(GoExtMeasurement* measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc allocator);
GoFx(kStatus) GoExtMeasurement_VInit(GoExtMeasurement measurement, kType type, kObject sensor, kObject srcTool, kBool isFilterable, kAlloc alloc);
GoFx(kStatus) GoExtMeasurement_VRead(GoExtMeasurement measurement, kXml xml, kXmlItem item);
GoFx(kStatus) GoExtMeasurement_VWrite(GoExtMeasurement measurement, kXml xml, kXmlItem item);
GoFx(kStatus) GoExtMeasurement_VRelease(GoExtMeasurement measurement);

#endif

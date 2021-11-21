/** 
 * @file    GoSections.x.h
 *
 * @internal
 * Copyright (C) 2016-2021 by LMI Technologies Inc.  
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef GO_SECTIONS_X_H
#define GO_SECTIONS_X_H

#include <kApi/Data/kXml.h>
#include <kApi/Data/kArrayList.h>
#include <GoSdk/GoUtils.h>
#include <GoSdk/GoSection.x.h>

typedef struct GoSectionsClass
{
    kObjectClass base; 
    kObject sensor; 

    kXml xml;
    kXmlItem xmlItem;

    k64f xMin;
    k64f xMax;
    k64f yMin;
    k64f yMax;

    kArrayList sections; //of type GoSection
} GoSectionsClass;

kDeclareClassEx(Go, GoSections, kObject)

GoFx(kStatus) GoSections_Construct(GoSections* part, kObject sensor, kAlloc allocator);

GoFx(kStatus) GoSections_Init(GoSections sections, kType type, kObject sensor, kAlloc alloc);
GoFx(kStatus) GoSections_VRelease(GoSections sections);

GoFx(kStatus) GoSections_Read(GoSections sections, kXml xml, kXmlItem item);
GoFx(kStatus) GoSections_Write(GoSections sections, kXml xml, kXmlItem item);

#endif

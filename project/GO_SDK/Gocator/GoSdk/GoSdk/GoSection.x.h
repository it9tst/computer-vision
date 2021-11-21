/** 
 * @file    GoSection.x.h
 *
 * @internal
 * Copyright (C) 2016-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef GO_SDK_SECTION_X_H
#define GO_SDK_SECTION_X_H

#include <GoSdk/GoSection.h>

typedef struct GoSectionClass
{
    kObjectClass base; 
    kObject sensor;

    k16s id; 
    kText128 name;
    kPoint64f startPoint;
    kPoint64f endPoint;
    kBool customSpacingIntervalEnabled;
    GoElement64f spacingInterval;
} GoSectionClass; 

kDeclareClassEx(Go, GoSection, kObject)

GoFx(kStatus) GoSection_Construct(GoSection* section, kObject sensor, k16s id, kAlloc allocator);
GoFx(kStatus) GoSection_Init(GoSection section, kType type, kObject sensor, k16s id, kAlloc alloc);
GoFx(kStatus) GoSection_VRelease(GoSection info);
GoFx(kStatus) GoSection_Read(GoSection info, kXml xml, kXmlItem item);
GoFx(kStatus) GoSection_Write(GoSection info, kXml xml, kXmlItem item);

GoFx(kStatus) GoSection_SetId(GoSection section, k16s id); //users add sections via the AddSection command, which assigns an ID

#endif

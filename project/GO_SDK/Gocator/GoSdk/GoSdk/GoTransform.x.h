/** 
 * @file    GoTransform.x.h
 *
 * @internal
 * Copyright (C) 2016-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef GO_TRANSFORM_X_H
#define GO_TRANSFORM_X_H

#include <kApi/Data/kXml.h>

typedef struct GoTransformClass
{
    kObjectClass base;
    kObject sensor;

    kXml xml;
    kXmlItem xmlItem;

    k64f encoderSpeed;
    k64f encoderResolution;

    kArrayList transformationList;
} GoTransformClass;

kDeclareClassEx(Go, GoTransform, kObject)

GoFx(kStatus) GoTransform_Construct(GoTransform* transform, kObject sensor, kAlloc allocator);

GoFx(kStatus) GoTransform_Init(GoTransform transform, kType type, kObject sensor, kAlloc alloc);
GoFx(kStatus) GoTransform_VRelease(GoTransform transform);

GoFx(kStatus) GoTransform_Read(GoTransform transform, kXml xml, kXmlItem item);
GoFx(kStatus) GoTransform_Write(GoTransform transform, kXml xml, kXmlItem item); 

#endif

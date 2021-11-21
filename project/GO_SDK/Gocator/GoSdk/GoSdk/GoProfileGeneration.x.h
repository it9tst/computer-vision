/** 
 * @file    GoProfileGeneration.x.h
 *
 * @internal
 * Copyright (C) 2016-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef GO_PROFILEGENERATION_X_H
#define GO_PROFILEGENERATION_X_H

#include <kApi/Data/kXml.h>

typedef struct GoProfileGenerationFixedLength
{
    GoProfileGenerationStartTrigger startTrigger;
    k32s externalInputIndex;
    kBool externalInputIndexUsed;
    kArrayList externalInputIndexOptions;
    GoElement64f length;
} GoProfileGenerationFixedLength;

typedef struct GoProfileGenerationVariableLength
{
    GoElement64f maxLength;
} GoProfileGenerationVariableLength;

typedef struct GoProfileGenerationRotational
{
    GoElement64f circumference;
} GoProfileGenerationRotational;


typedef struct GoProfileGenerationClass
{
    kObjectClass base; 
    kObject sensor; 

    kXml xml;
    kXmlItem xmlItem;

    GoProfileGenerationType type;
    GoProfileGenerationFixedLength fixedLength;
    GoProfileGenerationVariableLength variableLength;
    GoProfileGenerationRotational rotational;
} GoProfileGenerationClass; 

kDeclareClassEx(Go, GoProfileGeneration, kObject)

GoFx(kStatus) GoProfileGeneration_Construct(GoProfileGeneration* surface, kObject sensor, kAlloc allocator);

GoFx(kStatus) GoProfileGeneration_Init(GoProfileGeneration surface, kType type, kObject sensor, kAlloc alloc);
GoFx(kStatus) GoProfileGeneration_VRelease(GoProfileGeneration surface);

GoFx(kStatus) GoProfileGeneration_Read(GoProfileGeneration surface, kXml xml, kXmlItem item);
GoFx(kStatus) GoProfileGeneration_Write(GoProfileGeneration surface, kXml xml, kXmlItem item); 

#endif

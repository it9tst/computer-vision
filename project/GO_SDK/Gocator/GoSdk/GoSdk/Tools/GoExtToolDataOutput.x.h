/**
 * @file    GoExtToolDataOutput.x.h
 *
 * @internal
 * Copyright (C) 2017-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef GO_EXT_TOOL_DATA_OUTPUT_X_H
#define GO_EXT_TOOL_DATA_OUTPUT_X_H

#include <kApi/Data/kXml.h>

// Ideally these XML tag name definitions should go in a single place.
#define GO_XML_TAG_NAME_CUSTOM              "Custom"
#define GO_XML_TAG_NAME_ATTR_TYPE           "type"
#define GO_XML_TAG_NAME_ATTR_ID             "id"
#define GO_XML_TAG_NAME_ATTR_DATA_TYPE      "dataType"
#define GO_XML_TAG_NAME_NAME                "Name"
#define GO_XML_TAG_NAME_ENABLED             "Enabled"
#define GO_XML_TAG_NAME_TOOLDATAOUTPUT      "ToolDataOutput"

typedef struct GoExtToolDataOutputClass
{
    kObjectClass base;
    kObject      sensor;

    k32s        id;
    kText128    type;
    GoDataType  dataType;
    kText128    name;
    kBool       enabled;
} GoExtToolDataOutputClass;

kDeclareClassEx(Go, GoExtToolDataOutput, kObject)

GoFx(kStatus) GoExtToolDataOutput_Construct(GoExtToolDataOutput* toolDataOutput, kType type, kObject sensor, kAlloc allocator);
GoFx(kStatus) GoExtToolDataOutput_Init(GoExtToolDataOutput toolDataOutput, kType type, kObject sensor, kAlloc alloc);
GoFx(kStatus) GoExtToolDataOutput_VRelease(GoExtToolDataOutput toolDataOutput);

GoFx(kStatus) GoExtToolDataOutput_Read(GoExtToolDataOutput toolDataOutput, kXml xml, kXmlItem item, kBool isGdkUserTool);
GoFx(kStatus) GoExtToolDataOutput_Write(GoExtToolDataOutput toolDataOutput, kXml xml, kXmlItem item, kBool isGdkUserTool);
GoFx(kBool) GoExtToolDataOutput_IsValidDataType(GoDataType dataType);

#endif // GO_EXT_TOOL_DATA_OUTPUT_X_H

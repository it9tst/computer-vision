/**
 * @file    GoExtTool.x.h
 *
 * @internal
 * Copyright (C) 2016-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef GO_EXT_TOOL_X_H
#define GO_EXT_TOOL_X_H

#include <kApi/Data/kXml.h>
#include <GoSdk/Tools/GoMeasurement.h>

// Define Extensible tool names, ordered alphabetically.
#define GO_EXT_TOOL_NAME_CUSTOM         "Custom"
#define GO_EXT_TOOL_NAME_TOOL           "Tool"
#define GO_EXT_TOOL_NAME_SURFACE_EDGE   "SurfaceEdge"

#define GO_SDK_ANCHOR_X 0
#define GO_SDK_ANCHOR_Y 1
#define GO_SDK_ANCHOR_Z 2

#define GO_SDK_ANGLE_ANCHOR_Z 0

// Ideally, these should be defined in one place that defines all XML
// configuration tag names.
#define GO_XML_TAG_NAME_ATTR_TOOL_ID       "id"
#define GO_XML_TAG_NAME_ATTR_TYPE          "type"
#define GO_XML_TAG_NAME_ATTR_VERSION       "version"
#define GO_XML_TAG_NAME_ATTR_OPTIONS       "options"
#define GO_XML_TAG_NAME_ATTR_USED          "used"
#define GO_XML_TAG_NAME_ATTR_FORMAT        "format"
#define GO_XML_TAG_NAME_SOURCE             "Source"
#define GO_XML_TAG_NAME_ANCHOR             "Anchor"
#define GO_XML_TAG_NAME_ANCHOR_X           "X"
#define GO_XML_TAG_NAME_ANCHOR_Y           "Y"
#define GO_XML_TAG_NAME_ANCHOR_Z           "Z"
#define GO_XML_TAG_NAME_ANCHOR_ZANGLE      "ZAngle"
#define GO_XML_TAG_NAME_PARAMETERS         "Parameters"
#define GO_XML_TAG_NAME_STREAM             "Stream"
#define GO_XML_TAG_NAME_STREAM_STEP        "Step"
#define GO_XML_TAG_NAME_STREAM_ID          "Id"
#define GO_XML_TAG_NAME_TOOL_DATA_OUTPUTS  "ToolDataOutputs"


typedef struct GoExtToolAnchorCfg
{
    kBool used;
    kArrayList options;
    k32s anchor;
} GoExtToolAnchorCfg;

typedef struct GoExtToolAnchoringCfg
{
    GoExtToolAnchorCfg x;
    GoExtToolAnchorCfg y;
    GoExtToolAnchorCfg z;
    GoExtToolAnchorCfg zAngle;
} GoExtToolAnchoringCfg;

typedef struct GoExtToolClass
{
    GoToolClass base;

    kText32 version;
    kText128 toolType;

    kArrayList streamOptions;
    GoDataStream stream;

    kArrayList sourceOptions;
    GoDataSource source;

    GoExtToolAnchoringCfg anchoring;    // 4 for X, Y, Z, and ZAngle

    GoExtParams params;

    kArrayList toolDataOutputs;    // Of GoExtToolDataOutput.
} GoExtToolClass;

kDeclareClassEx(Go, GoExtTool, GoTool)

GoFx(kStatus) GoExtTool_Construct(GoExtTool* tool, kObject sensor, kAlloc allocator);
GoFx(kStatus) GoExtTool_VInit(GoExtTool tool, kType type, kObject sensor, kAlloc alloc);
GoFx(kStatus) GoExtTool_VRelease(GoExtTool tool);
GoFx(kStatus) GoExtTool_VRead(GoExtTool tool, kXml xml, kXmlItem item);
GoFx(kStatus) GoExtTool_VWrite(GoExtTool tool, kXml xml, kXmlItem item);
GoFx(kStatus) GoExtTool_Init(GoTool tool, kType type, GoToolType typeId, kObject sensor, kAlloc alloc);

GoFx(kStatus) GoExtTool_ParseAnchor(kXml xml, kXmlItem anchorItem, GoExtToolAnchoringCfg* anchoring);
GoFx(kStatus) GoExtTool_ReadToolDataOutputs(GoExtTool tool, kXml xml, kXmlItem item);
GoFx(kStatus) GoExtTool_WriteToolDataOutputs(GoExtTool tool, kXml xml, kXmlItem item);

#endif

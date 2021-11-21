/**
 * @file    GoRangeTools.x.h
 *
 * @internal
 * Copyright (C) 2016-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef GO_RANGE_TOOLS_X_H
#define GO_RANGE_TOOLS_X_H

#include <kApi/Data/kXml.h>

// Define Range tool names.
#define GO_RANGE_TOOL_NAME_POSITION     "RangePosition"
#define GO_RANGE_TOOL_NAME_THICKNESS    "RangeThickness"

typedef struct GoRangeToolClass
{
    GoToolClass base;

    kArrayList streamOptions;
    GoDataStream stream;

    kArrayList sourceOptions;
    GoDataSource source;

    kArrayList zAnchorOptions;   //of k32u - no api at the moment since Range tool anchoring isn't supported at the moment
    k32s zAnchor;
} GoRangeToolClass;

kDeclareClassEx(Go, GoRangeTool, GoTool)

GoFx(kStatus) GoRangeTool_Init(GoRangeTool tool, kType type, GoToolType typeId, kObject sensor, kAlloc alloc);
GoFx(kStatus) GoRangeTool_VRelease(GoRangeTool tool);
GoFx(kStatus) GoRangeTool_Read(GoRangeTool tool, kXml xml, kXmlItem item);
GoFx(kStatus) GoRangeTool_Write(GoRangeTool tool, kXml xml, kXmlItem item);

typedef struct GoRangePositionClass
{
    GoRangeToolClass base;
} GoRangePositionClass;

kDeclareClassEx(Go, GoRangePosition, GoRangeTool)

GoFx(kStatus) GoRangePosition_Construct(GoRangePosition* tool, kObject sensor, kAlloc allocator);
GoFx(kStatus) GoRangePosition_VInit(GoRangePosition tool, kType type, kObject sensor, kAlloc alloc);
GoFx(kStatus) GoRangePosition_VRelease(GoRangePosition tool);
GoFx(kStatus) GoRangePosition_VRead(GoRangePosition tool, kXml xml, kXmlItem item);
GoFx(kStatus) GoRangePosition_VWrite(GoRangePosition tool, kXml xml, kXmlItem item);


typedef struct GoRangeThicknessClass
{
    GoRangeToolClass base;
    kBool absoluteEnabled;
} GoRangeThicknessClass;

kDeclareClassEx(Go, GoRangeThickness, GoRangeTool)

GoFx(kStatus) GoRangeThickness_Construct(GoRangeThickness* tool, kObject sensor, kAlloc allocator);
GoFx(kStatus) GoRangeThickness_VInit(GoRangeThickness tool, kType type, kObject sensor, kAlloc alloc);
GoFx(kStatus) GoRangeThickness_VRelease(GoRangeThickness tool);
GoFx(kStatus) GoRangeThickness_VRead(GoRangeThickness tool, kXml xml, kXmlItem item);
GoFx(kStatus) GoRangeThickness_VWrite(GoRangeThickness tool, kXml xml, kXmlItem item);

#endif

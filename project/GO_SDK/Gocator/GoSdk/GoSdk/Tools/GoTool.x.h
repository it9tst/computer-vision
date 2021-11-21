/** 
 * @file    GoTool.x.h
 *
 * @internal
 * Copyright (C) 2016-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef GO_TOOL_X_H
#define GO_TOOL_X_H

#include <kApi/Data/kXml.h>

// The GoToolFormat replaces the use of the XML tool attribute "isCustom" (with the attribute "format")
// to identify a tool that is a GDK tool instead of a built-in Gocator tool. '
// The "isCustom" could not differentiate between a USER developed GDK tool or
// a LMI INTERNAL GDK tool, so that is why it has been replaced with the attribute "format".
// The "isCustom" attribute is redundant now that the "format" is available. 
typedef k32s GoToolFormat;
#define GO_TOOL_FORMAT_UNKNOWN                      (-1)
#define GO_TOOL_FORMAT_STANDARD                     (0) // Non-GDK tools (legacy/builtin tools part of the Gocator software base).
#define GO_TOOL_FORMAT_GDK_USER                     (1) // GDK tool created by LMI customers.
#define GO_TOOL_FORMAT_GDK_INTERNAL                 (2) // GDK tool created by LMI internally by the Vision Engineerng team.

typedef struct GoToolVTable
{    
    kObjectVTable base; 
    kStatus (kCall* VInit)(GoTool tool, kType type, kObject sensor, kAlloc allocator); 
    kStatus (kCall* VWrite)(GoTool tool, kXml xml, kXmlItem item); 
    kStatus (kCall* VRead)(GoTool tool, kXml xml, kXmlItem item);
} GoToolVTable; 

typedef struct GoToolClass
{
    kObjectClass base; 
    kObject sensor; 

    kSize index;   //represents position index in the Tools element
    kXml xml;
    kXmlItem xmlItem;

    kText128 name;
    k32s id;
    GoToolFormat format;
    
    kArrayList featureOutputs;
    kArrayList featureOptions;
    kArrayList featureNodesToMerge;

    kArrayList measurements;
    kArrayList measurementOptions;
    kArrayList measurementNodesToMerge;
    
    GoToolType typeId;
} GoToolClass; 

kDeclareVirtualClassEx(Go, GoTool, kObject)

#define GoTool_Class_(MODE)                     (kCastClass_(GoTool, MODE))

GoFx(kStatus) GoTool_Construct(GoTool* tool, kType type, kObject sensor, kAlloc allocator);

GoFx(kStatus) GoTool_VInit(GoTool tool, kType type, kObject sensor, kAlloc alloc);
GoFx(kStatus) GoTool_VRelease(GoTool tool);
GoFx(kStatus) GoTool_VRead(GoTool tool, kXml xml, kXmlItem item);
GoFx(kStatus) GoTool_VWrite(GoTool tool, kXml xml, kXmlItem item); 
GoFx(kStatus) GoTool_Init(GoTool tool, kType type, GoToolType typeId, kObject sensor, kAlloc alloc);
GoFx(kStatus) GoTool_Read(GoTool tool, kXml xml, kXmlItem item);
GoFx(kStatus) GoTool_Write(GoTool tool, kXml xml, kXmlItem item); 
GoFx(kStatus) GoTool_ReadMeasurements(GoTool tool, kXml xml, kXmlItem measurements, const kText64 toolType);
GoFx(kStatus) GoTool_ReadFeatures(GoTool tool,kXml xml, kXmlItem features, const kText64 toolType);

GoFx(kObject) GoTool_Sensor(GoTool tool);
GoFx(kStatus) GoTool_SetIndex(GoTool tool, kSize index);

GoFx(kStatus) GoTool_ReadToolName(GoTool tool, kXml xml, kXmlItem item);
GoFx(kBool) GoTool_IsGdkInternal(GoTool tool);

/**
* Parses stream options for the given tool.
*
* @public               @memberof GoTool
 * @version             Introduced in firmware 4.8.2.76
* @param    tool        GoTool object.
* @param    xml         kXml object.
* @param    item        kXmlItem object.
* @param    list        kArrayList object.
* @return   Operation status.
*/
GoFx(kStatus) GoToolUtil_ParseStreamOptions(GoTool tool, kXml xml, kXmlItem item, kArrayList list);

#endif

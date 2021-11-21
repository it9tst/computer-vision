/**
 * @file    GoTools.x.h
 *
 * @internal
 * Copyright (C) 2016-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef GO_TOOLS_X_H
#define GO_TOOLS_X_H

#include <kApi/Data/kXml.h>

// Define the Script tool name.
#define GO_TOOLS_NAME_SCRIPT "Script"

typedef struct GoToolOptionClass
{
    kObjectClass base;

    kText64 name;
    kBool isExtensibleTool;
    kArrayList measurementOptions;      // Of type GoMeasurementOption
    kArrayList featureOptions;          // Of type GoFeatureOption
    kArrayList toolDataOutputOptions;   // Of type GoToolDataOutputOption
} GoToolOptionClass;

kDeclareClassEx(Go, GoToolOption, kObject)

GoFx(kStatus) GoToolOption_Construct(GoToolOption* option, kAlloc allocator);
GoFx(kStatus) GoToolOption_Init(GoToolOption option, kType type, kAlloc alloc);
GoFx(kStatus) GoToolOption_VRelease(GoToolOption option);

GoFx(kBool) GoToolOption_IsCustom(GoToolOption option);

typedef struct GoToolsClass
{
    kObjectClass base;

    kObject sensor;

    kXml xml;
    //kXmlItem xmlItem; //there will be no forwards compatibility for collections

    kArrayList tools; //of type GoTool
    kArrayList toolOptions; //of type GoExtToolOption
    kBool isToolOptionsLegacy;

    kArrayList nodesToMerge;    //of type kXmlItem
} GoToolsClass;

typedef struct GoToolsVTable
{
    kObjectVTable base;                                 //base virtual table
} GoToolsVTable;

#define GOTOOLS_NAME_TYPE kText256

typedef struct GoToolsStatic
{
    kMap namekTypeMap; //GOTOOLS_NAME_TYPE - kType
    kMap nameToolIdMap; //GOTOOLS_NAME_TYPE - GoToolType
} GoToolsStatic;

kDeclareFullClassEx(Go, GoTools, kObject)

GoFx(kStatus) GoTools_Construct(GoTools* tools, kObject sensor, kAlloc allocator);
GoFx(kStatus) GoTools_Init(GoTools tools, kType type, kObject sensor, kAlloc alloc);
GoFx(kStatus) GoTools_VRelease(GoTools tools);
GoFx(kStatus) GoTools_Read(GoTools tools, kXml xml, kXmlItem item, kXmlItem toolOptionsItem);
GoFx(kStatus) GoTools_Write(GoTools tools, kXml xml, kXmlItem item);

GoFx(const kChar*) GoTools_BuiltInToolDefineToString(GoToolType type);
GoFx(GoToolType) GoTools_StringToBuiltInDefine(const kChar* name);

GoFx(kBool) GoTools_IsValidDataTypeOption(k32s dataTypeValue);
GoFx(kStatus) GoTools_ReadTools(GoTools tools, kXml xml, kXmlItem ToolListItem);
GoFx(kStatus) GoTools_ReadToolsGetToolType(kXml xml, kXmlItem toolItem, kType* type);
GoFx(kStatus) GoTools_ReadOptions(GoTools tools, kXml xml, kXmlItem optionsItem);
GoFx(kStatus) GoTools_ReadToolDataOutputOptions(kXml xml, kXmlItem toolOptionNode, GoToolOption option);
GoFx(kStatus) GoTools_ReadFeaturesOptions(kXml xml, kXmlItem toolOptionNode, GoToolOption option);
GoFx(kStatus) GoTools_ReadMeasurementsOptions(kXml xml, kXmlItem toolOptionNode, GoToolOption option);

GoFx(kBool) GoTools_IsToolInOptionsList(GoTools tools, kType toolType);

GoFx(const GoMeasurementOption*) GoTools_FindMeasurementOption(GoTools tools, kType toolType, kType measurementType);
GoFx(const GoFeatureOption*) GoTools_FindFeatureOption(GoTools tools, kType toolType, kType featureType);

GoFx(kSize) GoTools_NamekTypeMapEntryCount(void);
GoFx(kStatus) GoTools_NamekTypeMapAdd(const char* name, kType toolType);
GoFx(kStatus) GoTools_NamekTypeMapInit(void);
GoFx(kStatus) GoTools_CreateNamekTypeMap(void);
GoFx(kStatus) GoTools_CreateNameToolIdMap(void);
GoFx(kStatus) GoTools_ParseToolType(const kChar* toolName, kType* type);
GoFx(kStatus) GoTools_ParseToolTypeHelper(const kChar* toolName, kType* toolType);
GoFx(kStatus) GoTools_FormatToolType(kType type, kChar* toolName, kSize capacity);
GoFx(kStatus) GoTools_FormatToolTypeExtToolFormat(kType toolType, kBool isGdkInternal, kChar* toolName, kSize capacity);
GoFx(kStatus) GoTools_FormatToolTypeGetName(kType toolType, const kChar** outputPtr);
GoFx(kStatus) GoTools_FormatToolTypeHelper(kType toolType, const kChar** outputPtr);

typedef struct GoScriptClass
{
    GoToolClass base;
    kString code;
} GoScriptClass;

kDeclareClassEx(Go, GoScript, GoTool)

GoFx(kStatus) GoScript_Construct(GoScript* tool, kObject sensor, kAlloc allocator);
GoFx(kStatus) GoScript_VInit(GoScript tool, kType type, kObject sensor, kAlloc alloc);
GoFx(kStatus) GoScript_VRelease(GoScript tool);
GoFx(kStatus) xGoTools_InitStatic();
GoFx(kStatus) xGoTools_ReleaseStatic();
GoFx(kStatus) GoScript_VRead(GoScript tool, kXml xml, kXmlItem item);
GoFx(kStatus) GoScript_VWrite(GoScript tool, kXml xml, kXmlItem item);

#endif

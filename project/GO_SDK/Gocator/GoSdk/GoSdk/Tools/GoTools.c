/**
 * @file    GoTools.c
 *
 * @internal
 * Copyright (C) 2016-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <GoSdk/Tools/GoTools.h>
#include <GoSdk/GoSensor.h>
#include <GoSdk/Tools/GoMeasurements.h>
#include <GoSdk/Tools/GoTools.h>

kBeginClassEx(Go, GoToolOption)
    kAddVMethod(GoToolOption, kObject, VRelease)
kEndClassEx()


GoFx(kStatus) GoToolOption_Construct(GoToolOption* option, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback(allocator);
    kStatus status;

    kCheck(kAlloc_GetObject(alloc, kTypeOf(GoToolOption), option));

    if (!kSuccess(status = GoToolOption_Init(*option, kTypeOf(GoToolOption), alloc)))
    {
        kAlloc_FreeRef(alloc, option);
    }

    return status;
}

GoFx(kStatus) GoToolOption_Init(GoToolOption option, kType type, kAlloc alloc)
{
    kObjR(GoToolOption, option);
    kStatus exception;

    kCheck(kObject_Init(option, type, alloc));
    obj->name[0] = 0;
    obj->isExtensibleTool = kFALSE;
    kZero(obj->measurementOptions);
    kZero(obj->featureOptions);
    kZero(obj->toolDataOutputOptions);

    kTry
    {
        kTest(kArrayList_Construct(&obj->measurementOptions, kTypeOf(GoMeasurementOption), 0, alloc));
        kTest(kArrayList_Construct(&obj->featureOptions, kTypeOf(GoFeatureOption), 0, alloc));
        kTest(kArrayList_Construct(&obj->toolDataOutputOptions, kTypeOf(GoToolDataOutputOption), 0, alloc));
    }
    kCatch(&exception)
    {
        GoToolOption_VRelease(option);
        kEndCatch(exception);
    }

    return kOK;
}

GoFx(kStatus) GoToolOption_VRelease(GoToolOption option)
{
    kObj(GoToolOption, option);

    kCheck(kDisposeRef(&obj->measurementOptions));
    kCheck(kDisposeRef(&obj->featureOptions));
    kCheck(kDisposeRef(&obj->toolDataOutputOptions));

    return kObject_VRelease(option);
}

GoFx(const kChar*) GoToolOption_Name(GoToolOption option)
{
    kObj(GoToolOption, option);

    return obj->name;
}

GoFx(kBool) GoToolOption_IsCustom(GoToolOption option)
{
    kObj(GoToolOption, option);

    return obj->isExtensibleTool;
}

GoFx(kSize) GoToolOption_MeasurementOptionCount(GoToolOption option)
{
    kObj(GoToolOption, option);

    return kArrayList_Count(obj->measurementOptions);
}

GoFx(const GoMeasurementOption*) GoToolOption_MeasurementOptionAt(GoToolOption option, kSize index)
{
    kObj(GoToolOption, option);

    kAssert(index < kArrayList_Count(obj->measurementOptions));

    return kArrayList_AtT(obj->measurementOptions, index, GoMeasurementOption);
}

GoFx(kSize) GoToolOption_FeatureOptionCount(GoToolOption option)
{
    kObj(GoToolOption, option);

    return kArrayList_Count(obj->featureOptions);
}

GoFx(const GoFeatureOption*) GoToolOption_FeatureOptionAt(GoToolOption option, kSize index)
{
    kObj(GoToolOption, option);

    kAssert(index < kArrayList_Count(obj->featureOptions));

    return kArrayList_AtT(obj->featureOptions, index, GoFeatureOption);
}

GoFx(kSize) GoToolOption_ToolDataOutputOptionCount(GoToolOption option)
{
    kObj(GoToolOption, option);

    return kArrayList_Count(obj->toolDataOutputOptions);
}

GoFx(const GoToolDataOutputOption*) GoToolOption_ToolDataOutputOptionAt(GoToolOption option, kSize index)
{
    kObj(GoToolOption, option);

    kAssert(index < kArrayList_Count(obj->toolDataOutputOptions));

    return kArrayList_AtT(obj->toolDataOutputOptions, index, GoToolDataOutputOption);
}

kBeginFullClassEx(Go, GoTools)
    kAddVMethod(GoTools, kObject, VRelease)
kEndFullClassEx()

GoFx(kStatus) GoTools_Construct(GoTools* tools, kObject sensor, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback(allocator);
    kStatus status;

    kCheck(kAlloc_GetObject(alloc, kTypeOf(GoTools), tools));

    if (!kSuccess(status = GoTools_Init(*tools, kTypeOf(GoTools), sensor, alloc)))
    {
        kAlloc_FreeRef(alloc, tools);
    }

    return status;
}

GoFx(kStatus) GoTools_Init(GoTools tools, kType type, kObject sensor, kAlloc alloc)
{
    kObjR(GoTools, tools);
    kStatus exception;

    kCheck(kObject_Init(tools, type, alloc));
    kZero(obj->xml);
    kZero(obj->tools);
    kZero(obj->toolOptions);
    obj->isToolOptionsLegacy = kFALSE;
    kZero(obj->nodesToMerge);

    obj->sensor = sensor;

    kTry
    {
        kTest(kArrayList_Construct(&obj->tools, kTypeOf(GoTool), 0, alloc));
        kTest(kArrayList_Construct(&obj->toolOptions, kTypeOf(GoToolOption), 0, alloc));
        kTest(kArrayList_Construct(&obj->nodesToMerge, kTypeOf(kXml), 0, alloc));
    }
    kCatch(&exception)
    {
        GoTools_VRelease(tools);
        kEndCatch(exception);
    }

    return kOK;
}

GoFx(kStatus) GoTools_VRelease(GoTools tools)
{
    kObj(GoTools, tools);

    kCheck(kDestroyRef(&obj->nodesToMerge));
    kCheck(kDisposeRef(&obj->tools));
    kCheck(kDisposeRef(&obj->toolOptions));

    return kObject_VRelease(tools);
}

GoFx(kStatus) xGoTools_InitStatic()
{
    kStaticObj(GoTools);

    kCheck(kMap_Construct(&sobj->namekTypeMap, kTypeOf(GOTOOLS_NAME_TYPE), kTypeOf(kType), 0, kNULL));
    kCheck(kMap_Construct(&sobj->nameToolIdMap, kTypeOf(GOTOOLS_NAME_TYPE), kTypeOf(GoToolType), 0, kNULL));

    kCheck(GoTools_CreateNamekTypeMap());
    kCheck(GoTools_CreateNameToolIdMap());

    return kOK;
}

GoFx(kStatus) xGoTools_ReleaseStatic()
{
    kStaticObj(GoTools);
    
    kObject_Destroy(sobj->namekTypeMap);
    kObject_Destroy(sobj->nameToolIdMap);
    
    return kOK;
}

GoFx(kStatus) GoTools_Read(GoTools tools, kXml xml, kXmlItem ToolListItem, kXmlItem toolOptionsItem)
{
    kObj(GoTools, tools);

    kCheck(!kIsNull(ToolListItem));
    obj->xml = xml;
    kCheck(kArrayList_Clear(obj->nodesToMerge));

    //xml forward compatibility is not supported for the tools element, as it is dynamic

    if (!kIsNull(toolOptionsItem))
    {
        kCheck(GoTools_ReadOptions(tools, xml, toolOptionsItem));
    }

    kCheck(GoTools_ReadTools(tools, xml, ToolListItem));

    return kOK;
}

GoFx(kStatus) GoTools_Write(GoTools tools, kXml xml, kXmlItem item)
{
    kObj(GoTools, tools);
    kXmlItem toolItem = kNULL;
    kText256 toolName;
    kSize i;

    for (i = 0; i < kArrayList_Count(obj->tools); i++)
    {
        GoTool currentTool = kArrayList_AsT(obj->tools, i, GoTool);

        // GOC-14127: On writes, choose the correct GDK tool name based on whether the
        // tool is a USER GDK tool or INTERNAL GDK tool.
        kCheck(GoTools_FormatToolTypeExtToolFormat(kObject_Type(currentTool), GoTool_IsGdkInternal(currentTool), toolName, kCountOf(toolName)));

        kCheck(kXml_AddItem(xml, item, toolName, &toolItem));
        kCheck(GoTool_Write(currentTool, xml, toolItem));
    }

    // XML merge intentionally omitted due to the dynamic nature of the Tools element. Replaced
    //  with unrecognized node insertion
    for (i = 0; i < kArrayList_Count(obj->nodesToMerge); i++)
    {
        kXmlItem mergeItem = *kArrayList_AtT(obj->nodesToMerge, i, kXmlItem);

        kCheck(kXml_CopyItem(xml, item, kNULL, obj->xml, mergeItem, kNULL));
    }

    return kOK;
}

GoFx(kStatus) GoTools_ReadTools(GoTools tools, kXml xml, kXmlItem ToolListItem)
{
    kObj(GoTools, tools);
    kXmlItem        currentToolItem = kNULL;
    kType           type;
    kStatus         exception;
    kSize           i;
    GoTool          tool = kNULL;
    kBool           insertTool = kFALSE;
    kSize           toolCount;

    toolCount = kXml_ChildCount(xml, ToolListItem);
    for (i = 0; i < toolCount; i++)
    {
        insertTool = kFALSE;
        tool = kNULL;

        currentToolItem = kXml_ChildAt(xml, ToolListItem, i);

        if (kSuccess(GoTools_ReadToolsGetToolType(xml, currentToolItem, &type)))
        {
            kString custToolType = kNULL;

            kTry
            {
                if (kTypeOf(GoExtTool) == type && kXml_AttrExists(xml, currentToolItem, GO_XML_TAG_NAME_ATTR_TYPE))
                {
                    kTest(kString_Construct(&custToolType, kNULL, kObject_Alloc(tools)));
                    kTest(kXml_AttrString(xml, currentToolItem, GO_XML_TAG_NAME_ATTR_TYPE, custToolType));
                }
                if (i >= kArrayList_Count(obj->tools)                       ||
                    kIsNull(tool = kArrayList_AsT(obj->tools, i, GoTool))   ||
                    !kObject_Is(tool, type)                                 ||
                    (!kIsNull(custToolType) && !GoExtTool_Equals(tool, custToolType)))
                {
                    if (!kIsNull(tool))
                    {
                        kTest(kArrayList_RemoveT(obj->tools, i, &tool));
                        kDisposeRef(&tool);
                    }

                    kTest(GoTool_Construct(&tool, type, obj->sensor, kObject_Alloc(tools)));
                    insertTool = kTRUE;
                }

                if (kTypeOf(GoExtTool) != type || !kIsNull(custToolType))
                {
                    kTest(GoTool_Read(tool, xml, currentToolItem));
                    kTest(GoTool_SetIndex(tool, i));

                    if (insertTool)
                    {
                        if (i >= kArrayList_Count(obj->tools))
                        {
                            kCheck(kArrayList_AddT(obj->tools, &tool));
                        }
                        else
                        {
                            kCheck(kArrayList_InsertT(obj->tools, i, &tool));
                        }
                    }
                }
            }
            kCatchEx(&exception)
            {
                kDisposeRef(&tool);
                kEndCatchEx(exception);
            }
            kFinallyEx
            {
                kDestroyRef(&custToolType);
                kEndFinallyEx();
            }
        }
        else
        {
            kCheck(kArrayList_AddT(obj->nodesToMerge, &currentToolItem));
        }

    }

    while (toolCount < kArrayList_Count(obj->tools))
    {
        kCheck(kArrayList_RemoveT(obj->tools, kArrayList_Count(obj->tools) - 1, &tool));
        kDisposeRef(&tool);
    }

    return kOK;
}

GoFx(kStatus) GoTools_LegacyOptionsRead(GoTools tools, kXml xml, kXmlItem item)
{
    kObj(GoTools, tools);
    kString csvList = kNULL;
    kArrayList tempArrayList = kNULL;
    kBool addOptionToList = kFALSE;
    kSize i;
    kXmlItem toolOptionNode = kNULL;
    GoToolOption option = kNULL;

    kTry
    {
        kTest(kString_Construct(&csvList, "", kObject_Alloc(tools)));
        kTest(kXml_AttrString(xml, item, "options", csvList));
        kTest(kString_Split(csvList, ",", &tempArrayList, kObject_Alloc(tools)));

        for (i = 0; i < kArrayList_Count(tempArrayList); i++)
        {
            kString token = *kArrayList_AtT(tempArrayList, i, kString);
            kObjNR(GoToolOption, opt, kNULL);

            option = kNULL;
            addOptionToList = kFALSE;

            if (i >= kArrayList_Count(obj->toolOptions)
                || kIsNull(option = kArrayList_AsT(obj->toolOptions, i, GoToolOption))
                || !kStrEquals(GoToolOption_Name(option), kXml_ItemName(xml, toolOptionNode)))
            {
                if (!kIsNull(option))
                {
                    kTest(kArrayList_RemoveT(obj->toolOptions, i, &option));
                    kDestroyRef(&option);
                }

                kTest(GoToolOption_Construct(&option, kObject_Alloc(tools)));
                addOptionToList = kTRUE;
            }

            opt = xGoToolOption_Cast(option);

            if (!kStrEquals(opt->name, kXml_ItemName(xml, toolOptionNode)))
            {
                kTest(kStrCopy(opt->name, kCountOf(opt->name), kString_Chars(token)));
            }

            if (addOptionToList)
            {
                kCheck(kArrayList_InsertT(obj->toolOptions, i, &option));
            }
        }

        while (kArrayList_Count(tempArrayList) < kArrayList_Count(obj->toolOptions))
        {
            kCheck(kArrayList_RemoveT(obj->toolOptions, kArrayList_Count(obj->toolOptions) - 1, &option));
            kDisposeRef(&option);
        }

    }
    kFinally
    {
        kDestroyRef(&csvList);
        kDisposeRef(&tempArrayList);
        kEndFinally();
    }

    obj->isToolOptionsLegacy = kTRUE;

    return kOK;
}

GoFx(kStatus) GoTools_ReadMeasurementsOptions(kXml xml, kXmlItem toolOptionNode, GoToolOption option)
{
    kObjN(GoToolOption, opt, option);
    kXmlItem            measurementOptions;
    kSize               j;
    kSize               childCount;

    if (!kIsNull(measurementOptions = kXml_Child(xml, toolOptionNode, "MeasurementOptions")))
    {
        childCount = kXml_ChildCount(xml, measurementOptions);
        kCheck(kArrayList_Resize(opt->measurementOptions, childCount));

        for (j = 0; j < childCount; j++)
        {
            kXmlItem                measurementOption = kXml_ChildAt(xml, measurementOptions, j);
            GoMeasurementOption*    optionToAdd = kNULL;

            optionToAdd = kArrayList_AtT(opt->measurementOptions, j, GoMeasurementOption);

            kCheck(kStrCopy(optionToAdd->name, kCountOf(optionToAdd->name), kXml_ItemName(xml, measurementOption)));
            kCheck(kXml_AttrSize(xml, measurementOption, "minCount", &optionToAdd->minCount));
            kCheck(kXml_AttrSize(xml, measurementOption, "maxCount", &optionToAdd->maxCount));
        }
    }

    return kOK;
}

GoFx(kStatus) GoTools_ReadFeaturesOptions(kXml xml, kXmlItem toolOptionNode, GoToolOption option)
{
    kObjN(GoToolOption, opt, option);
    kXmlItem            featureOptions;
    kSize               j;
    kSize               childCount;

    if (!kIsNull(featureOptions = kXml_Child(xml, toolOptionNode, "FeatureOptions")))
    {
        childCount = kXml_ChildCount(xml, featureOptions);
        kCheck(kArrayList_Resize(opt->featureOptions, childCount));

        for (j = 0; j < childCount; j++)
        {
            kXmlItem            featureOption = kXml_ChildAt(xml, featureOptions, j);
            GoFeatureOption*    optionToAdd = kNULL;
            kText64             tempStr;

            optionToAdd = kArrayList_AtT(opt->featureOptions, j, GoFeatureOption);

            kCheck(kXml_AttrText(xml, featureOption, "dataType", tempStr, kCountOf(tempStr)));
            kCheck(GoFeatures_ParseDataType(tempStr, &optionToAdd->dataType));

            kCheck(kStrCopy(optionToAdd->type, kCountOf(optionToAdd->type), kXml_ItemName(xml, featureOption)));
            kCheck(kXml_AttrText(xml, featureOption, "displayName", optionToAdd->name, kCountOf(optionToAdd->name)));
            kCheck(kXml_AttrSize(xml, featureOption, "minCount", &optionToAdd->minCount));
            kCheck(kXml_AttrSize(xml, featureOption, "maxCount", &optionToAdd->maxCount));
        }
    }

    return kOK;
}

GoFx(kBool) GoTools_IsValidDataTypeOption(k32s dataTypeValue)
{
    kBool valid = kFALSE;

    switch (dataTypeValue)
    {
    case GO_DATA_TYPE_RANGE:
    case GO_DATA_TYPE_UNIFORM_PROFILE:
    case GO_DATA_TYPE_PROFILE_POINT_CLOUD:
    case GO_DATA_TYPE_UNIFORM_SURFACE:
    case GO_DATA_TYPE_SURFACE_POINT_CLOUD:
    case GO_DATA_TYPE_UNMERGED_PROFILE_POINT_CLOUD:
    case GO_DATA_TYPE_VIDEO:
    case GO_DATA_TYPE_TRACHEID:       // Will SDK ever get this?
    case GO_DATA_TYPE_MESH:
    case GO_DATA_TYPE_FEATURES_ONLY:  // Will SDK ever get this?
        valid = kTRUE;
        break;
    default:
        if (dataTypeValue >= GO_DATA_TYPE_GENERIC_BASE && dataTypeValue <= GO_DATA_TYPE_GENERIC_END)
            valid = kTRUE;
        break;
    }

    return valid;
}

GoFx(kStatus) GoTools_ReadToolDataOutputOptions(kXml xml, kXmlItem toolOptionNode, GoToolOption option)
{
    kObjN(GoToolOption, opt, option);
    kXmlItem            toolDataOutputOptions;
    kSize               j;
    kSize               childCount;

    if (!kIsNull(toolDataOutputOptions = kXml_Child(xml, toolOptionNode, "ToolDataOutputOptions")))
    {
        childCount = kXml_ChildCount(xml, toolDataOutputOptions);
        kCheck(kArrayList_Resize(opt->toolDataOutputOptions, childCount));

        for (j = 0; j < childCount; j++)
        {
            kXmlItem                toolDataOutputOption = kXml_ChildAt(xml, toolDataOutputOptions, j);
            GoToolDataOutputOption* optionToAdd = kNULL;

            optionToAdd = kArrayList_AtT(opt->toolDataOutputOptions, j, GoToolDataOutputOption);

            kCheck(kXml_Attr32s(xml, toolDataOutputOption, "dataType", &optionToAdd->dataType));
            if (! GoTools_IsValidDataTypeOption(optionToAdd->dataType))
            {
                optionToAdd->dataType = GO_DATA_TYPE_NONE;
            }
            kCheck(kStrCopy(optionToAdd->type, kCountOf(optionToAdd->type), kXml_ItemName(xml, toolDataOutputOption)));
            kCheck(kXml_AttrText(xml, toolDataOutputOption, "displayName", optionToAdd->name, kCountOf(optionToAdd->name)));
            kCheck(kXml_AttrSize(xml, toolDataOutputOption, "minCount", &optionToAdd->minCount));
            kCheck(kXml_AttrSize(xml, toolDataOutputOption, "maxCount", &optionToAdd->maxCount));
        }
    }

    return kOK;
}

GoFx(kStatus) GoTools_ReadOptions(GoTools tools, kXml xml, kXmlItem optionsItem)
{
    kObj(GoTools, tools);
    kXmlItem        toolOptionNode = kNULL;
    kSize           i;
    kBool           addOptionToList = kFALSE;
    GoToolOption    option;
    kStatus         exception = kOK;
    kSize           toolOptionCount;

    if (kXml_AttrExists(xml, optionsItem, "options"))
    {
        kCheck(GoTools_LegacyOptionsRead(tools, xml, optionsItem));
    }
    else
    {
        toolOptionCount = kXml_ChildCount(xml, optionsItem);
        kTry
        {
            for (i = 0; i < toolOptionCount; i++)
            {
                kObjNR(GoToolOption, opt, kNULL);

                addOptionToList = kFALSE;
                option = kNULL;
                toolOptionNode = kXml_ChildAt(xml, optionsItem, i);

                if (i >= kArrayList_Count(obj->toolOptions)
                    || kIsNull(option = kArrayList_AsT(obj->toolOptions, i, GoToolOption))
                    || !kStrEquals(GoToolOption_Name(option), kXml_ItemName(xml, toolOptionNode)))
                {
                    if (!kIsNull(option))
                    {
                        kTest(kArrayList_RemoveT(obj->toolOptions, i, &option));
                        kDestroyRef(&option);
                    }

                    kTest(GoToolOption_Construct(&option, kObject_Alloc(tools)));
                    opt = xGoToolOption_Cast(option);

                    kCheck(kXml_AttrBool(xml, toolOptionNode, "isCustom", &opt->isExtensibleTool));
                    kCheck(kStrCopy(opt->name, kCountOf(opt->name), kXml_ItemName(xml, toolOptionNode)));
                    kCheck(GoTools_ReadMeasurementsOptions(xml, toolOptionNode, option));
                    kCheck(GoTools_ReadFeaturesOptions(xml, toolOptionNode, option));
                    kCheck(GoTools_ReadToolDataOutputOptions(xml, toolOptionNode, option));

                    addOptionToList = kTRUE;
                }

                if (addOptionToList)
                {
                    kCheck(kArrayList_InsertT(obj->toolOptions, i, &option));
                }
            }
        }
        kCatch(&exception)
        {
            if (addOptionToList)
            {
                kDestroyRef(&option);
            }
            kEndCatch(exception);
        }

        while (toolOptionCount < kArrayList_Count(obj->toolOptions))
        {
            kCheck(kArrayList_RemoveT(obj->toolOptions, kArrayList_Count(obj->toolOptions) - 1, &option));
            kDisposeRef(&option);
        }

        obj->isToolOptionsLegacy = kFALSE;
    }

    return kOK;
}

GoFx(kSize) GoTools_ToolOptionCount(GoTools tools)
{
    kObj(GoTools, tools);

    GoSensor_SyncConfig(obj->sensor);

    return kArrayList_Count(obj->toolOptions);
}

GoFx(GoToolOption) GoTools_ToolOptionAt(GoTools tools, kSize index)
{
    kObj(GoTools, tools);

    GoSensor_SyncConfig(obj->sensor);
    kAssert(index < kArrayList_Count(obj->toolOptions));

    return *kArrayList_AtT(obj->toolOptions, index, GoToolOption);
}

GoFx(kStatus) GoTools_AddToolByName(GoTools tools, const kChar* optionName, GoTool* tool)
{
    kObj(GoTools, tools);

    kCheck(GoSensor_AddTool(obj->sensor, optionName, optionName));

    if (!kIsNull(tool))
    {
        *tool = kArrayList_AsT(obj->tools, GoTools_ToolCount(tools) - 1, GoTool);
    }

    return kOK;
}

GoFx(kSize) GoTools_ToolCount(GoTools tools)
{
    kObj(GoTools, tools);

    GoSensor_SyncConfig(obj->sensor);

    return kArrayList_Count(obj->tools);
}

GoFx(GoTool) GoTools_ToolAt(GoTools tools, kSize index)
{
    kObj(GoTools, tools);

    GoSensor_SyncConfig(obj->sensor);

    kAssert(index < kArrayList_Count(obj->tools));

    return kArrayList_AsT(obj->tools, index, GoTool);
}

GoFx(kStatus) GoTools_MoveTool(GoTools tools, kSize index, kSize newIndex)
{
    GoToolsClass* K_ATTRIBUTE_UNUSED obj = xGoTools_Cast(tools);
    GoTool tool = kNULL;

    kCheckArgs(newIndex < kArrayList_Count(obj->tools));

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    kCheck(kArrayList_RemoveT(obj->tools, index, &tool));

    if (newIndex == kArrayList_Count(obj->tools))
    {
        kCheck(kArrayList_AddT(obj->tools, &tool));
    }
    else
    {
        kCheck(kArrayList_InsertT(obj->tools, newIndex, &tool));
    }

    kCheck(GoSensor_SetConfigModified(obj->sensor));
    kCheck(GoSensor_SyncConfig(obj->sensor));

    return kOK;
}

GoFx(kStatus) GoTools_AddTool(GoTools tools, GoToolType type, GoTool* tool)
{
    kObj(GoTools, tools);
    GoTool newTool = kNULL;
    kStatus exception;
    kType mapType;
    kSize toolListLength = kArrayList_Count(obj->tools);

    switch(type)
    {
    case GO_TOOL_RANGE_POSITION: mapType = kTypeOf(GoRangePosition); break;
    case GO_TOOL_RANGE_THICKNESS: mapType = kTypeOf(GoRangeThickness); break;
    case GO_TOOL_PROFILE_AREA: mapType = kTypeOf(GoProfileArea); break;
    case GO_TOOL_PROFILE_BOUNDING_BOX: mapType = kTypeOf(GoProfileBox); break;
    case GO_TOOL_PROFILE_BRIDGE_VALUE: mapType = kTypeOf(GoProfileBridgeValue); break;
    case GO_TOOL_PROFILE_CIRCLE: mapType = kTypeOf(GoProfileCircle); break;
    case GO_TOOL_PROFILE_DIMENSION: mapType = kTypeOf(GoProfileDim); break;
    case GO_TOOL_PROFILE_GROOVE: mapType = kTypeOf(GoProfileGroove);break;
    case GO_TOOL_PROFILE_INTERSECT: mapType = kTypeOf(GoProfileIntersect); break;
    case GO_TOOL_PROFILE_LINE: mapType = kTypeOf(GoProfileLine); break;
    case GO_TOOL_PROFILE_PANEL: mapType = kTypeOf(GoProfilePanel); break;
    case GO_TOOL_PROFILE_ROUND_CORNER: mapType = kTypeOf(GoProfileRoundCorner); break;
    case GO_TOOL_PROFILE_POSITION: mapType = kTypeOf(GoProfilePosition); break;
    case GO_TOOL_PROFILE_STRIP: mapType = kTypeOf(GoProfileStrip); break;
    case GO_TOOL_SURFACE_BOUNDING_BOX: mapType = kTypeOf(GoSurfaceBox); break;
    case GO_TOOL_SURFACE_COUNTERSUNK_HOLE: mapType = kTypeOf(GoSurfaceCountersunkHole); break;
    case GO_TOOL_SURFACE_DIMENSION: mapType = kTypeOf(GoSurfaceDim); break;
    case GO_TOOL_SURFACE_ELLIPSE: mapType = kTypeOf(GoSurfaceEllipse); break;
    case GO_TOOL_SURFACE_HOLE: mapType = kTypeOf(GoSurfaceHole); break;
    case GO_TOOL_SURFACE_OPENING: mapType = kTypeOf(GoSurfaceOpening); break;
    case GO_TOOL_SURFACE_PLANE: mapType = kTypeOf(GoSurfacePlane); break;
    case GO_TOOL_SURFACE_POSITION: mapType = kTypeOf(GoSurfacePosition); break;
    //case GO_TOOL_SURFACE_RIVET: mapType = kTypeOf(GoSurfaceRivet); break;
    case GO_TOOL_SURFACE_STUD: mapType = kTypeOf(GoSurfaceStud); break;
    case GO_TOOL_SURFACE_VOLUME: mapType = kTypeOf(GoSurfaceVolume); break;
    case GO_TOOL_SCRIPT: mapType = kTypeOf(GoScript); break;
    case GO_TOOL_EXTENSIBLE: mapType = kTypeOf(GoExtTool); break;
    default: return kERROR_PARAMETER;
    }

    if (kSuccess(GoSensor_ReadInfo(obj->sensor)) && GoSensor_IsConfigurable(obj->sensor))
    {
        GoSensor_CacheConfig(obj->sensor);
    }

    kTry
    {
        kTest(GoTool_Construct(&newTool, mapType, obj->sensor, kObject_Alloc(tools)));

        kTest(kArrayList_AddT(obj->tools, &newTool));

        kTest(GoSensor_SetConfigModified(obj->sensor));
        kTest(GoSensor_SyncConfig(obj->sensor));
    }
    kCatch(&exception)
    {
        if (toolListLength < kArrayList_Count(obj->tools))
        {
            kArrayList_Discard(obj->tools, toolListLength);
        }

        kDestroyRef(&newTool);

        kEndCatch(exception);
    }

    if (!kIsNull(tool))
    {
        *tool = newTool;
    }

    return kOK;
}

GoFx(kStatus) GoTools_RemoveTool(GoTools tools, kSize index)
{
    kObj(GoTools, tools);
    GoTool tool = kNULL;

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    kCheck(kArrayList_RemoveT(obj->tools, index, &tool));
    kDestroyRef(&tool);

    kCheck(GoSensor_SetConfigModified(obj->sensor));
    kCheck(GoSensor_SyncConfig(obj->sensor));

    return kOK;
}

GoFx(kStatus) GoTools_ClearTools(GoTools tools)
{
    kObj(GoTools, tools);

    kCheck(kArrayList_Purge(obj->tools));

    kCheck(GoSensor_SetConfigModified(obj->sensor));
    kCheck(GoSensor_SyncConfig(obj->sensor));

    return kOK;
}


kBeginClassEx(Go, GoScript)
    kAddVMethod(GoScript, kObject, VRelease)
    kAddVMethod(GoScript, GoTool, VInit)
    kAddVMethod(GoScript, GoTool, VRead)
    kAddVMethod(GoScript, GoTool, VWrite)
kEndClassEx()

GoFx(kStatus) GoScript_Construct(GoScript* tool, kObject sensor, kAlloc allocator)
{
    return GoTool_Construct(tool, kTypeOf(GoScript), sensor, allocator);
}

GoFx(kStatus) GoScript_VInit(GoScript tool, kType type, kObject sensor, kAlloc alloc)
{
    kObjR(GoScript, tool);

    kCheck(GoTool_Init(tool, type, GO_TOOL_SCRIPT, sensor, alloc));
    kZero(obj->code);

    kCheck(kString_Construct(&obj->code, "", alloc));

    return kOK;
}

GoFx(kStatus) GoScript_VRelease(GoScript tool)
{
    kObj(GoScript, tool);

    kCheck(kDestroyRef(&obj->code));

    return GoTool_VRelease(tool);
}

GoFx(kStatus) GoScript_VRead(GoScript tool, kXml xml, kXmlItem item)
{
    kObj(GoScript, tool);

    kCheck(GoTool_VRead(tool, xml, item));

    kCheck(kXml_ChildString(xml, item, "Code", obj->code));

    return kOK;
}

GoFx(kStatus) GoScript_VWrite(GoScript tool, kXml xml, kXmlItem item)
{
    kObj(GoScript, tool);

    kCheck(kXml_SetChildText(xml, item, "Name", obj->base.name));
    kCheck(kXml_SetChildText(xml, item, "Code", kString_Chars(obj->code)));
    kCheck(GoTool_VWrite(tool, xml, item));

    return kOK;
}

GoFx(kStatus) GoScript_Code(GoScript tool, kChar** code)
{
    kObj(GoScript, tool);
    kSize length = kString_Length(obj->code) + 1;
    kChar* output = kNULL;

    kCheck(kMemAllocZero(sizeof(kChar) * length , &output));
    kCheck(kStrCopy(output, length, kString_Chars(obj->code)));

    *code = output;

    return kOK;
}

GoFx(kStatus) GoScript_SetCode(GoScript tool, kChar* code)
{
    kObj(GoScript, tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));

    if (!kIsNull(code))
    {
        kCheck(kString_Set(obj->code, code));
    }
    else
    {
        kCheck(kString_Set(obj->code, ""));
    }

    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(kStatus) GoScript_AddOutput(GoScript tool, k32u id)
{
    GoMeasurement output = kNULL;
    kStatus exception = kOK;
    kSize i;

    for (i = 0; i < GoTool_MeasurementCount(tool); i++)
    {
        if ((k32s)id == GoMeasurement_Id(GoTool_MeasurementAt(tool, i)))
        {
            return kERROR_PARAMETER;
        }
    }

    kTry
    {
        kTest(GoTool_AddMeasurement(tool, kTypeOf(GoScriptOutput), kFALSE, &output));
        kTest(GoMeasurement_SetId(output, id));
        kTest(GoSensor_SetConfigModified(GoTool_Sensor(tool)));
        kTest(GoSensor_SyncConfig(GoTool_Sensor(tool)));
    }
    kCatch(&exception);
    {
        kDestroyRef(&output);
        kEndCatch(exception);
    }


    return kOK;
}

GoFx(kStatus) GoScript_RemoveOutput(GoScript tool, k32u id)
{
    kSize i;
    GoMeasurement output = kNULL;

    for (i = 0; i < GoTool_MeasurementCount(tool); i++)
    {
        output = GoTool_MeasurementAt(tool, i);
        //confirm that id is present and remove from map

        if (GoMeasurement_Id(output) == (k32s)id)
        {
            kCheck(GoTool_RemoveMeasurement(tool, i));
            kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));
            kCheck(GoSensor_SyncConfig(GoTool_Sensor(tool)));

            return kOK;
        }
    }

    return kERROR_NOT_FOUND;
}

GoFx(kSize) GoScript_OutputCount(GoScript tool)
{
    return GoTool_MeasurementCount(tool);
}

GoFx(GoScriptOutput) GoScript_OutputAt(GoScript tool, kSize index)
{
    kAssert(index < GoTool_MeasurementCount(tool));

    return GoTool_MeasurementAt(tool, index);
}

GoFx(kSize) GoTools_NamekTypeMapEntryCount(void)
{
    kStaticObj(GoTools);

    return kMap_Count(sobj->namekTypeMap);
}

GoFx(kStatus) GoTools_NamekTypeMapAdd(const char* name, kType toolType)
{
    kStaticObj(GoTools);

    GOTOOLS_NAME_TYPE storage = { 0 };

    if (kStrLength(name) > kCountOf(storage))
    {
        kAssert(kFALSE);
        return kERROR_PARAMETER;
    }

    kCheck(kStrCopy(storage, kCountOf(storage), name));

    kCheck(kMap_AddT(sobj->namekTypeMap, &storage, &toolType));

    return kOK;
}

// This function is the runtime equivalent of defining a static mapping table.
GoFx(kStatus) GoTools_NamekTypeMapInit(void)
{
    kStaticObj(GoTools);

    kStatus exception;

    kTry
    {
        kTest(GoTools_NamekTypeMapAdd(GO_RANGE_TOOL_NAME_POSITION, kTypeOf(GoRangePosition)));
        kTest(GoTools_NamekTypeMapAdd(GO_RANGE_TOOL_NAME_THICKNESS, kTypeOf(GoRangeThickness)));

        kTest(GoTools_NamekTypeMapAdd(GO_PROFILE_TOOL_NAME_AREA, kTypeOf(GoProfileArea)));
        kTest(GoTools_NamekTypeMapAdd(GO_PROFILE_TOOL_NAME_BOUNDING_BOX, kTypeOf(GoProfileBox)));
        kTest(GoTools_NamekTypeMapAdd(GO_PROFILE_TOOL_NAME_BRIDGE_VALUE, kTypeOf(GoProfileBridgeValue)));
        kTest(GoTools_NamekTypeMapAdd(GO_PROFILE_TOOL_NAME_CIRCLE, kTypeOf(GoProfileCircle)));
        kTest(GoTools_NamekTypeMapAdd(GO_PROFILE_TOOL_NAME_DIMENSION, kTypeOf(GoProfileDim)));
        kTest(GoTools_NamekTypeMapAdd(GO_PROFILE_TOOL_NAME_GROOVE, kTypeOf(GoProfileGroove)));
        kTest(GoTools_NamekTypeMapAdd(GO_PROFILE_TOOL_NAME_INTERSECT, kTypeOf(GoProfileIntersect)));
        kTest(GoTools_NamekTypeMapAdd(GO_PROFILE_TOOL_NAME_LINE, kTypeOf(GoProfileLine)));
        kTest(GoTools_NamekTypeMapAdd(GO_PROFILE_TOOL_NAME_PANEL, kTypeOf(GoProfilePanel)));
        kTest(GoTools_NamekTypeMapAdd(GO_PROFILE_TOOL_NAME_POSITION, kTypeOf(GoProfilePosition)));
        kTest(GoTools_NamekTypeMapAdd(GO_PROFILE_TOOL_NAME_ROUND_CORNER, kTypeOf(GoProfileRoundCorner)));
        kTest(GoTools_NamekTypeMapAdd(GO_PROFILE_TOOL_NAME_STRIP, kTypeOf(GoProfileStrip)));

        kTest(GoTools_NamekTypeMapAdd(GO_SURFACE_TOOL_NAME_BOUNDING_BOX, kTypeOf(GoSurfaceBox)));
        kTest(GoTools_NamekTypeMapAdd(GO_SURFACE_TOOL_NAME_COUNTERSUNK_HOLE, kTypeOf(GoSurfaceCountersunkHole)));
        kTest(GoTools_NamekTypeMapAdd(GO_SURFACE_TOOL_NAME_DIMENSION, kTypeOf(GoSurfaceDim)));
        kTest(GoTools_NamekTypeMapAdd(GO_SURFACE_TOOL_NAME_ELLIPSE, kTypeOf(GoSurfaceEllipse)));
        kTest(GoTools_NamekTypeMapAdd(GO_SURFACE_TOOL_NAME_HOLE, kTypeOf(GoSurfaceHole)));
        kTest(GoTools_NamekTypeMapAdd(GO_SURFACE_TOOL_NAME_OPENING, kTypeOf(GoSurfaceOpening)));
        kTest(GoTools_NamekTypeMapAdd(GO_SURFACE_TOOL_NAME_PLANE, kTypeOf(GoSurfacePlane)));
        kTest(GoTools_NamekTypeMapAdd(GO_SURFACE_TOOL_NAME_POSITION, kTypeOf(GoSurfacePosition)));
        //kTest(GoTools_NamekTypeMapAdd(GO_SURFACE_TOOL_NAME_RIVET, kTypeOf(GoSurfaceRivet)));
        kTest(GoTools_NamekTypeMapAdd(GO_SURFACE_TOOL_NAME_STUD, kTypeOf(GoSurfaceStud)));
        kTest(GoTools_NamekTypeMapAdd(GO_SURFACE_TOOL_NAME_VOLUME, kTypeOf(GoSurfaceVolume)));

        kTest(GoTools_NamekTypeMapAdd(GO_TOOLS_NAME_SCRIPT, kTypeOf(GoScript)));
        kTest(GoTools_NamekTypeMapAdd(GO_EXT_TOOL_NAME_CUSTOM, kTypeOf(GoExtTool)));
        kTest(GoTools_NamekTypeMapAdd(GO_EXT_TOOL_NAME_TOOL, kTypeOf(GoExtTool)));
    }
    kCatch(&exception)
    {
        // During development, fail hard if there are map table issues
        // such as the table being too small for all the entries.
        kAssert(0);
        kEndCatch(exception);
    }

    return kOK;
}

GoFx(kStatus) GoTools_CreateNamekTypeMap(void)
{
    kStaticObj(GoTools);

    kCheck(GoTools_NamekTypeMapInit());

    // The table should have at least one entry.
    kCheckTrue((GoTools_NamekTypeMapEntryCount() > 0), kERROR);

    // Table is now available.
    return kOK;
}

GoFx(kStatus) GoTools_ParseToolTypeHelper(const kChar* toolName, kType* toolType)
{
    kStaticObj(GoTools);
    kMapItem item = kNULL;

    GOTOOLS_NAME_TYPE storage = { 0 };

    if (kStrLength(toolName) > kCountOf(storage))
    {
        kAssert(kFALSE);
        return kERROR_PARAMETER;
    }

    kStrCopy(storage, kCountOf(storage), toolName);

    kCheck(kMap_FindItemT(sobj->namekTypeMap, storage, &item));

    if (!kIsNull(item))
    {
        *toolType = *kMap_ValueT(sobj->namekTypeMap, item, kType);
        return kOK;
    }    

    kAssert(0);
    return kERROR_NOT_FOUND;
}

GoFx(kStatus) GoTools_ParseToolType(const kChar* toolName, kType* toolType)
{
    kCheck(GoTools_ParseToolTypeHelper(toolName, toolType));

    return kOK;
}

// This function looks up the tool type in the mapping table and returns
// the name associated with that type.
GoFx(kStatus) GoTools_FormatToolTypeHelper(kType toolType, const kChar** outputPtr)
{
    kStaticObj(GoTools);

    kMapItem item = kNULL;

    item = kMap_First(sobj->namekTypeMap);

    while (!kIsNull(item))
    {
        const kChar* key = kMap_KeyT(sobj->namekTypeMap, item, kChar);
        kType value = *kMap_ValueT(sobj->namekTypeMap, item, kType);

        if (value == toolType)
        {
            *outputPtr = key;
            break;
        }

        item = kMap_Next(sobj->namekTypeMap, item);
    }

    return kOK;
}

// This is an internal helper function added to support the different flavours of
// the FormatToolType() API.
GoFx(kStatus) GoTools_FormatToolTypeGetName(kType toolType, const kChar** outputPtr)
{
    kCheck(GoTools_FormatToolTypeHelper(toolType, outputPtr));

    return kOK;
}

// Pre-existing call before GOC-14127 that only uses the mapping
// table to map tool type to tool name.
// Does not handle INTERNAL GDK tools, only USER GDK tools.
// GoTools_FormatToolTypeExtToolFormat() adds support for INTERNAL
// GDK tool mapping of tool type to tool name.
GoFx(kStatus) GoTools_FormatToolType(kType toolType, kChar* toolName, kSize capacity)
{
    const kChar* output = kNULL;

    kCheck(GoTools_FormatToolTypeGetName(toolType, &output));

    kCheck(kStrCopy(toolName, capacity, output));

    return kOK;
}

// GOC-14127: This function is called internally by the SDK to get the tool type name.
// The mapping table has two entries for tool type GoExtTool: 
//    GO_EXT_TOOL_NAME_CUSTOM (USER) and
//    GO_EXT_TOOL_NAME_TOOL (INTERNAL)
// and cannot differentiate the two so Custom is always 
// returned by the mapping table for both USER and INTERNAL GDK tools.
// This function differentiates between whether the tool is a USER or INTERNAL GDK tool 
// based on the format argument passed in, and returns the correct name.
GoFx(kStatus) GoTools_FormatToolTypeExtToolFormat(kType toolType, kBool isGdkInternal, kChar* toolName, kSize capacity)
{
    const kChar* output = kNULL;

    if (isGdkInternal)
    {
        // GDK internal format tool XML node should be called "Tool".
        // Older SDKs might have labelled the tool as "Custom", so ignore what
        // is read and set the tool name to the correct internal tool name.
        output = GO_EXT_TOOL_NAME_TOOL;
    }
    else
    {
        // Format is standard tool format or user GDK format or not set (unknown),
        // so use the name from the mapping table.
        kCheck(GoTools_FormatToolTypeGetName(toolType, &output));
    }

    kCheck(kStrCopy(toolName, capacity, output));

    return kOK;
}

GoFx(kStatus) GoTools_AssignMeasurementId(GoTools tools, GoMeasurement measurement)
{
    k32s currentId;
    kSize i, j;
    GoTool currentTool;
    GoMeasurement currentMeasurement;
    kMap positiveIdMap = kNULL;

    kTry
    {
        kTest(kMap_Construct(&positiveIdMap, kTypeOf(k32s), kTypeOf(kPointer), 0, kObject_Alloc(tools)));

        for (i = 0; i < GoTools_ToolCount(tools); i++)
        {
            currentTool = GoTools_ToolAt(tools, i);

            for (j = 0; j < GoTool_MeasurementCount(currentTool); j++)
            {
                currentMeasurement = GoTool_MeasurementAt(currentTool, j);

                //enabled state isn't checked due to possibility of valid IDs in measurements to be enabled later
                if (GoMeasurement_Id(currentMeasurement) > -1)
                {
                    k32s measurementId = GoMeasurement_Id(currentMeasurement);
                    kPointer measurementIdPointer = &measurementId;

                    kMap_AddT(positiveIdMap, &measurementId, &measurementIdPointer); //no test/check is done here due to the possibility of duplicate Ids
                }
            }
        }

        for (currentId = 0; currentId < k32S_MAX; currentId++)
        {
            //if currentID not used, set the measurement with it
            if (!kMap_HasT(positiveIdMap, &currentId))
            {                
                kTest(GoMeasurement_SetId(measurement, currentId));
                break;
            }
        }
    }
    kFinally
    {
        kDestroyRef(&positiveIdMap);
        kEndFinally();
    }

    return kOK;
}

GoFx(GoMeasurement) GoTools_FindMeasurementById(GoTools tools, k32u id)
{
    kSize i, j;
    GoTool tool = kNULL;
    GoMeasurement measurement = kNULL;

    for (i = 0; i < GoTools_ToolCount(tools); i++)
    {
        tool = GoTools_ToolAt(tools, i);

        for (j = 0; j < GoTool_MeasurementCount(tool); j++)
        {
            measurement = GoTool_MeasurementAt(tool, j);

            if (GoMeasurement_Id(measurement) == (k32s)id)
                return measurement;
        }
    }

    return kNULL;
}

GoFx(kStatus) GoTools_NameToolIdMapAdd(const char* name, GoToolType toolId)
{
    kStaticObj(GoTools);

    GOTOOLS_NAME_TYPE storage = { 0 };

    if (kStrLength(name) > kCountOf(storage))
    {
        kAssert(kFALSE);
        return kERROR_PARAMETER;
    }

    kStrCopy(storage, kCountOf(storage), name);

    kCheck(kMap_AddT(sobj->nameToolIdMap, &storage, &toolId));

    return kOK;
}

GoFx(kSize) GoTools_NameToolIdEntryCount(void)
{
    kStaticObj(GoTools);

    return kMap_Count(sobj->nameToolIdMap);
}

GoFx(kStatus) GoTools_CreateNameToolIdMap(void)
{
    kStaticObj(GoTools);

    kCheck(GoTools_NameToolIdMapAdd(GO_RANGE_TOOL_NAME_POSITION, GO_TOOL_RANGE_POSITION));
    kCheck(GoTools_NameToolIdMapAdd(GO_RANGE_TOOL_NAME_THICKNESS, GO_TOOL_RANGE_THICKNESS ));

    kCheck(GoTools_NameToolIdMapAdd(GO_PROFILE_TOOL_NAME_AREA, GO_TOOL_PROFILE_AREA ));
    kCheck(GoTools_NameToolIdMapAdd(GO_PROFILE_TOOL_NAME_BOUNDING_BOX, GO_TOOL_PROFILE_BOUNDING_BOX ));
    kCheck(GoTools_NameToolIdMapAdd(GO_PROFILE_TOOL_NAME_BRIDGE_VALUE, GO_TOOL_PROFILE_BRIDGE_VALUE ));
    kCheck(GoTools_NameToolIdMapAdd(GO_PROFILE_TOOL_NAME_CIRCLE, GO_TOOL_PROFILE_CIRCLE ));
    kCheck(GoTools_NameToolIdMapAdd(GO_PROFILE_TOOL_NAME_DIMENSION, GO_TOOL_PROFILE_DIMENSION ));
    kCheck(GoTools_NameToolIdMapAdd(GO_PROFILE_TOOL_NAME_GROOVE, GO_TOOL_PROFILE_GROOVE ));
    kCheck(GoTools_NameToolIdMapAdd(GO_PROFILE_TOOL_NAME_INTERSECT, GO_TOOL_PROFILE_INTERSECT ));
    kCheck(GoTools_NameToolIdMapAdd(GO_PROFILE_TOOL_NAME_LINE, GO_TOOL_PROFILE_LINE ));
    kCheck(GoTools_NameToolIdMapAdd(GO_PROFILE_TOOL_NAME_PANEL, GO_TOOL_PROFILE_PANEL ));
    kCheck(GoTools_NameToolIdMapAdd(GO_PROFILE_TOOL_NAME_POSITION, GO_TOOL_PROFILE_POSITION ));
    kCheck(GoTools_NameToolIdMapAdd(GO_PROFILE_TOOL_NAME_ROUND_CORNER, GO_TOOL_PROFILE_ROUND_CORNER ));
    kCheck(GoTools_NameToolIdMapAdd(GO_PROFILE_TOOL_NAME_STRIP, GO_TOOL_PROFILE_STRIP ));

    kCheck(GoTools_NameToolIdMapAdd(GO_SURFACE_TOOL_NAME_BOUNDING_BOX, GO_TOOL_SURFACE_BOUNDING_BOX ));
    kCheck(GoTools_NameToolIdMapAdd(GO_SURFACE_TOOL_NAME_COUNTERSUNK_HOLE, GO_TOOL_SURFACE_COUNTERSUNK_HOLE ));
    kCheck(GoTools_NameToolIdMapAdd(GO_SURFACE_TOOL_NAME_DIMENSION, GO_TOOL_SURFACE_DIMENSION ));
    kCheck(GoTools_NameToolIdMapAdd(GO_SURFACE_TOOL_NAME_ELLIPSE, GO_TOOL_SURFACE_ELLIPSE ));
    kCheck(GoTools_NameToolIdMapAdd(GO_SURFACE_TOOL_NAME_HOLE, GO_TOOL_SURFACE_HOLE ));
    kCheck(GoTools_NameToolIdMapAdd(GO_SURFACE_TOOL_NAME_OPENING, GO_TOOL_SURFACE_OPENING ));
    kCheck(GoTools_NameToolIdMapAdd(GO_SURFACE_TOOL_NAME_PLANE, GO_TOOL_SURFACE_PLANE ));
    kCheck(GoTools_NameToolIdMapAdd(GO_SURFACE_TOOL_NAME_POSITION, GO_TOOL_SURFACE_POSITION ));
    //kCheck(GoTools_NameToolIdMapAdd(GO_SURFACE_TOOL_NAME_RIVET, GO_TOOL_SURFACE_RIVET));
    kCheck(GoTools_NameToolIdMapAdd(GO_SURFACE_TOOL_NAME_STUD, GO_TOOL_SURFACE_STUD ));
    kCheck(GoTools_NameToolIdMapAdd(GO_SURFACE_TOOL_NAME_VOLUME, GO_TOOL_SURFACE_VOLUME ));
    kCheck(GoTools_NameToolIdMapAdd(GO_TOOLS_NAME_SCRIPT, GO_TOOL_SCRIPT ));
    kCheck(GoTools_NameToolIdMapAdd(GO_EXT_TOOL_NAME_CUSTOM, GO_TOOL_EXTENSIBLE ));
    kCheck(GoTools_NameToolIdMapAdd(GO_EXT_TOOL_NAME_TOOL, GO_TOOL_TOOL ));

    // Table should have at least one entry.
    kCheckTrue((GoTools_NameToolIdEntryCount() > 0), kERROR);

    // Table is now available.
    return kOK;
}

GoFx(const kChar*) GoTools_BuiltInToolDefineToString(GoToolType type)
{
    kStaticObj(GoTools);
    kMapItem item = kNULL;

    item = kMap_First(sobj->nameToolIdMap);

    while (!kIsNull(item))
    {
        const kChar* key = kMap_KeyT(sobj->nameToolIdMap, item, kChar);
        GoToolType value = *kMap_ValueT(sobj->nameToolIdMap, item, GoToolType);

        if (value == type)
        {
            return key;
        }

        item = kMap_Next(sobj->nameToolIdMap, item);
    }

    kAssert(kFALSE);
    return "TOOL_UNKNOWN";
}

GoFx(GoToolType) GoTools_StringToBuiltInDefine(const kChar* name)
{
    kStaticObj(GoTools);
    kMapItem item = kNULL;
    GoToolType  retType = GO_TOOL_UNKNOWN;

    GOTOOLS_NAME_TYPE storage = { 0 };

    if (kStrLength(name) > kCountOf(storage))
    {
        kAssert(kFALSE);
        return kERROR_PARAMETER;
    }

    kStrCopy(storage, kCountOf(storage), name);

    kCheck(kMap_FindItemT(sobj->nameToolIdMap, &storage, &item));

    if (!kIsNull(item))
    {
        retType = *kMap_ValueT(sobj->nameToolIdMap, item, GoToolType);
    }    

    return retType;
}


GoFx(kStatus) GoTools_AddMeasurementByName(GoTools tools, GoTool tool,
                                        const kChar* type, const kChar* name,
                                        GoMeasurement* measurement)
{
    kObj(GoTools, tools);
    kObjN(GoTool, toolObj, tool);
    kSize i;
    kBool measurementFound = kFALSE;
    GoToolOption option = kNULL;
    kType optionType = kNULL;

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));

    for (i = 0; i < kArrayList_Count(obj->toolOptions); i++)
    {
        optionType = kNULL;

        option = *kArrayList_AtT(obj->toolOptions, i, GoToolOption);

        if (!kSuccess(GoTools_ParseToolType(GoToolOption_Name(option), &optionType))
            && GoToolOption_IsCustom(option) && kObject_Is(tool, kTypeOf(GoExtTool))
            && kStrEquals(GoToolOption_Name(option), GoExtTool_Type(tool)))
        {
            optionType = kTypeOf(GoExtTool);
        }

        if (optionType == kObject_Type(tool))
        {
            break;
        }
    }

    kCheckArgs(!kIsNull(option) && optionType == kObject_Type(tool));

    for (i = 0; i < GoToolOption_MeasurementOptionCount(option); i++)
    {
        const GoMeasurementOption* measurementOption = GoToolOption_MeasurementOptionAt(option, i);

        if (kStrEquals(measurementOption->name, type))
        {
            kCheck(GoSensor_AddMeasurement(obj->sensor, toolObj->id, type, name));
            measurementFound = kTRUE;
            break;
        }
    }

    kCheckArgs(measurementFound);

    if (!kIsNull(measurement))
    {
        *measurement = GoTool_MeasurementAt(tool, GoTool_MeasurementCount(tool) - 1);
    }

    return kOK;
}

GoFx(kBool) GoTools_IsToolInOptionsList(GoTools tools, kType toolType)
{
    kObj(GoTools, tools);
    kSize i;
    kText64 toolName;

    kCheck(GoTools_FormatToolType(toolType, toolName, kCountOf(toolName)));

    for (i = 0; i < kArrayList_Count(obj->toolOptions); i++)
    {
        GoToolOption* option = *kArrayList_AtT(obj->toolOptions, i, GoToolOption);

        if (strcmp(GoToolOption_Name(option), toolName) == 0)
        {
            return kTRUE;
        }
    }

    return kFALSE;
}

GoFx(const GoMeasurementOption*) GoTools_FindMeasurementOption(GoTools tools, kType toolType, kType measurementType)
{
    kObj(GoTools, tools);
    kSize i, j;
    kText64 toolName;
    kText64 measurementName;

    if (kSuccess(GoTools_FormatToolType(toolType, toolName, kCountOf(toolName)))
        && kSuccess(GoMeasurements_FormatType(measurementType, measurementName, kCountOf(measurementName))))
    {
        for (i = 0; i < kArrayList_Count(obj->toolOptions); i++)
        {
            GoToolOption* option = *kArrayList_AtT(obj->toolOptions, i, GoToolOption);

            if (strcmp(GoToolOption_Name(option), toolName) == 0)
            {
                for (j = 0; j < GoToolOption_MeasurementOptionCount(option); j++)
                {
                    const GoMeasurementOption* measurement = GoToolOption_MeasurementOptionAt(option, j);

                    if (strcmp(measurement->name, measurementName) == 0)
                    {
                        return measurement;
                    }
                }
            }
        }
    }

    return kNULL;
}

GoFx(const GoFeatureOption*) GoTools_FindFeatureOption(GoTools tools, kType toolType, kType featureType)
{
    kObj(GoTools, tools);
    kSize i, j;
    kText64 toolName;
    kText64 featureName;

    if (kSuccess(GoTools_FormatToolType(toolType, toolName, kCountOf(toolName)))
        && kSuccess(GoFeatures_FormatType(featureType, featureName, kCountOf(featureName))))
    {
        for (i = 0; i < kArrayList_Count(obj->toolOptions); i++)
        {
            GoToolOption* option = *kArrayList_AtT(obj->toolOptions, i, GoToolOption);

            if (strcmp(GoToolOption_Name(option), toolName) == 0)
            {
                for (j = 0; j < GoToolOption_FeatureOptionCount(option); j++)
                {
                    const GoFeatureOption* feature = GoToolOption_FeatureOptionAt(option, j);

                    if (strcmp(feature->type, featureName) == 0)
                    {
                        return feature;
                    }
                }
            }
        }
    }

    return kNULL;
}

// GOC-14127: helper function to return the kType of the tool, given the tool name.
GoFx(kStatus) GoTools_ReadToolsGetToolType(kXml xml, kXmlItem toolItem, kType* type)
{
    const kChar*    toolName;

    toolName = kXml_ItemName(xml, toolItem);

    return (GoTools_ParseToolType(toolName, type));
}
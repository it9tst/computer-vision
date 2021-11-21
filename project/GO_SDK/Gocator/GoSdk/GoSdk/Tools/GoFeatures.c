/**
* @file    GoFeatures.c
*
* @internal
* Copyright (C) 2017-2021 by LMI Technologies Inc.
* Licensed under the MIT License.
* Redistributed files must retain the above copyright notice.
*/
#include <GoSdk/Tools/GoFeatures.h>
#include <GoSdk/GoSensor.h>
#include <GoSdk/Tools/GoExtParams.h>
#include <GoSdk/GoUtils.h>

kBeginStaticClassEx(Go, GoFeatures)
kEndStaticClassEx()

GoFx(kStatus) xGoFeatures_InitStatic()
{
    kStaticObj(GoFeatures);

    kCheck(kArrayList_Construct(&sobj->nameTypeMapList, kTypeOf(GoFeaturesNameTypeMapping), 0, kNULL));

    kCheck(GoFeatures_CreateNameTypeMap());

    return kOK;

}
GoFx(kStatus) xGoFeatures_ReleaseStatic()
{
    kStaticObj(GoFeatures);

    kObject_Destroy(sobj->nameTypeMapList);

    return kOK;
}

GoFx(kSize) GoFeatures_NameTypeMapEntryCount(void)
{
    kStaticObj(GoFeatures);
    return kArrayList_Count(sobj->nameTypeMapList);
}

GoFx(kStatus) GoFeatures_NameTypeMapAdd(const char* name, const char* featureName, kType featureType)
{
    kStaticObj(GoFeatures);
    GoFeaturesNameTypeMapping map;

    kCheck(kStrCopy(map.lookupName, kCountOf(map.lookupName), name));
    kCheck(kStrCopy(map.featureName, kCountOf(map.featureName), featureName));
    map.featureType = featureType;

    kCheck(kArrayList_AddT(sobj->nameTypeMapList, &map));


    return kOK;
}

// This function is the runtime equivalent of defining a static mapping table.
GoFx(kStatus) GoFeatures_NameTypeMapInit(void)
{
    kStatus exception;

    kTry
    {
        // NOTE: In the table, the first parameter is a concatenation of tool name and feature name.
        //
        // GOC-14127: older (pre-6.1) SDK messes up writing the XML config by always marking GDK tools as "Custom"
        // regardless of whether the GDK tool is a USER GDK tool or INTERNAL GDK tool.
        // Users can sometimes end up messing up the XML config if user uses both SDK and Gocator GUI to configure a sensor.
        // One example of a messed up XML config is where the tool name is GO_EXT_TOOL_NAME_TOOL 
        // but feature is GO_FEATURES_NAME_CUSTOM.
        //
        // GDK tool name -> GDK tool type mapping:
        // In general, mapping of GDK tool name to GDK tool type to GoExtFeature should be lenient and allow even 
        // invalid combinations of GDK tool/feature combinations.
        // So put all the possible combinations for GDK tool, even if most of the time they do not
        // or should not happen, that map to GoExtFeature into the mapping table.
        // 
        // NOTE: 
        // For USER GDK tools, tool name is GO_EXT_TOOL_NAME_CUSTOM and feature name should be GO_FEATURES_NAME_CUSTOM.
        // For INTERNAL GDK tools, tool name is GO_EXT_TOOL_NAME_TOOL and feature name should be GO_FEATURES_NAME_FEATURE.
        // So the normal combinations for mapping to type GoExtFeature are:
        //  1. GO_EXT_TOOL_NAME_CUSTOM GO_FEATURES_NAME_CUSTOM
        //  2. GO_EXT_TOOL_NAME_TOOL GO_FEATURES_NAME_FEATURE
        // The abnormal combinations are:
        //  3. GO_EXT_TOOL_NAME_TOOL GO_FEATURES_NAME_CUSTOM
        //  4. GO_EXT_TOOL_NAME_CUSTOM GO_FEATURES_NAME_FEATURE
        //
        // GDK tool type -> GDK tool name mapping.
        // This table has multiple entries for GoExtFeature type.
        // For mapping GoExtFeature to a feature name, search stops when the first entry with type GoExtFeature is found.
        // In this case, the first entry added to the map above is the one and therefore will always return
        // GO_FEATURES_NAME_CUSTOM which is correct only for USER GDK tool.
        // To support INTERNAL GDK tool, this table cannot be used. Instead, extra code is needed to explicitly handle
        // INTERNAL GDK tool by bypassing this table. See GoFeatures_FormatTypeExtToolFormat().
        kTest(GoFeatures_NameTypeMapAdd(GO_EXT_TOOL_NAME_CUSTOM GO_FEATURES_NAME_CUSTOM, GO_FEATURES_NAME_CUSTOM, kTypeOf(GoExtFeature)));
        kTest(GoFeatures_NameTypeMapAdd(GO_EXT_TOOL_NAME_TOOL GO_FEATURES_NAME_FEATURE, GO_FEATURES_NAME_FEATURE, kTypeOf(GoExtFeature)));

        // GOC-14127: Add support for these abnormal combinations, for leniency reasons.
        kTest(GoFeatures_NameTypeMapAdd(GO_EXT_TOOL_NAME_TOOL GO_FEATURES_NAME_CUSTOM, GO_FEATURES_NAME_FEATURE, kTypeOf(GoExtFeature)));
        kTest(GoFeatures_NameTypeMapAdd(GO_EXT_TOOL_NAME_CUSTOM GO_FEATURES_NAME_FEATURE, GO_FEATURES_NAME_CUSTOM, kTypeOf(GoExtFeature)));

        kTest(GoFeatures_NameTypeMapAdd(GO_PROFILE_TOOL_NAME_POSITION GO_FEATURES_NAME_POINT, GO_FEATURES_NAME_POINT, kTypeOf(GoProfilePositionPoint)));

        kTest(GoFeatures_NameTypeMapAdd(GO_PROFILE_TOOL_NAME_LINE GO_FEATURES_NAME_LINE, GO_FEATURES_NAME_LINE, kTypeOf(GoProfileLineLine)));
        kTest(GoFeatures_NameTypeMapAdd(GO_PROFILE_TOOL_NAME_LINE GO_FEATURES_NAME_ERROR_MIN_POINT, GO_FEATURES_NAME_ERROR_MIN_POINT, kTypeOf(GoProfileLineMinErrorPoint)));
        kTest(GoFeatures_NameTypeMapAdd(GO_PROFILE_TOOL_NAME_LINE GO_FEATURES_NAME_ERROR_MAX_POINT, GO_FEATURES_NAME_ERROR_MAX_POINT, kTypeOf(GoProfileLineMaxErrorPoint)));

        kTest(GoFeatures_NameTypeMapAdd(GO_PROFILE_TOOL_NAME_INTERSECT GO_FEATURES_NAME_INTERSECT_POINT, GO_FEATURES_NAME_INTERSECT_POINT, kTypeOf(GoProfileIntersectIntersectPoint)));
        kTest(GoFeatures_NameTypeMapAdd(GO_PROFILE_TOOL_NAME_INTERSECT GO_FEATURES_NAME_LINE, GO_FEATURES_NAME_LINE, kTypeOf(GoProfileIntersectLine)));
        kTest(GoFeatures_NameTypeMapAdd(GO_PROFILE_TOOL_NAME_INTERSECT GO_FEATURES_NAME_BASELINE, GO_FEATURES_NAME_BASELINE, kTypeOf(GoProfileIntersectBaseLine)));

        kTest(GoFeatures_NameTypeMapAdd(GO_PROFILE_TOOL_NAME_BOUNDING_BOX GO_FEATURES_NAME_CORNER_POINT, GO_FEATURES_NAME_CORNER_POINT, kTypeOf(GoProfileBoundingBoxCornerPoint)));
        kTest(GoFeatures_NameTypeMapAdd(GO_PROFILE_TOOL_NAME_BOUNDING_BOX GO_FEATURES_NAME_CENTER_POINT, GO_FEATURES_NAME_CENTER_POINT, kTypeOf(GoProfileBoundingBoxCenterPoint)));

        kTest(GoFeatures_NameTypeMapAdd(GO_PROFILE_TOOL_NAME_AREA GO_FEATURES_NAME_CENTER_POINT, GO_FEATURES_NAME_CENTER_POINT, kTypeOf(GoProfileAreaCenterPoint)));
        kTest(GoFeatures_NameTypeMapAdd(GO_PROFILE_TOOL_NAME_CIRCLE GO_FEATURES_NAME_CENTER_POINT, GO_FEATURES_NAME_CENTER_POINT, kTypeOf(GoProfileCircleCenterPoint)));

        // TODO: confirm profile dimension has a feature or not.
        kTest(GoFeatures_NameTypeMapAdd(GO_PROFILE_TOOL_NAME_DIMENSION GO_FEATURES_NAME_CENTER_POINT, GO_FEATURES_NAME_CENTER_POINT, kTypeOf(GoProfileDimensionCenterPoint)));

        // TODO: confirm if profile panel has features or not.
        kTest(GoFeatures_NameTypeMapAdd(GO_PROFILE_TOOL_NAME_PANEL GO_FEATURES_NAME_LEFT_GAP_POINT, GO_FEATURES_NAME_LEFT_GAP_POINT, kTypeOf(GoProfilePanelLeftGapPoint)));
        kTest(GoFeatures_NameTypeMapAdd(GO_PROFILE_TOOL_NAME_PANEL GO_FEATURES_NAME_LEFT_FLUSH_POINT, GO_FEATURES_NAME_LEFT_FLUSH_POINT, kTypeOf(GoProfilePanelLeftFlushPoint)));
        kTest(GoFeatures_NameTypeMapAdd(GO_PROFILE_TOOL_NAME_PANEL GO_FEATURES_NAME_RIGHT_GAP_POINT, GO_FEATURES_NAME_RIGHT_GAP_POINT, kTypeOf(GoProfilePanelRightGapPoint)));
        kTest(GoFeatures_NameTypeMapAdd(GO_PROFILE_TOOL_NAME_PANEL GO_FEATURES_NAME_RIGHT_FLUSH_POINT, GO_FEATURES_NAME_RIGHT_FLUSH_POINT, kTypeOf(GoProfilePanelRightFlushPoint)));

        // TODO: confirm if round corner has a feature or not.
        kTest(GoFeatures_NameTypeMapAdd(GO_PROFILE_TOOL_NAME_ROUND_CORNER GO_FEATURES_NAME_POINT, GO_FEATURES_NAME_POINT, kTypeOf(GoProfileRoundCornerPoint)));
        kTest(GoFeatures_NameTypeMapAdd(GO_PROFILE_TOOL_NAME_ROUND_CORNER GO_FEATURES_NAME_EDGE_POINT, GO_FEATURES_NAME_EDGE_POINT, kTypeOf(GoProfileRoundCornerEdgePoint)));
        kTest(GoFeatures_NameTypeMapAdd(GO_PROFILE_TOOL_NAME_ROUND_CORNER GO_FEATURES_NAME_CENTER_POINT, GO_FEATURES_NAME_CENTER_POINT, kTypeOf(GoProfileRoundCornerCenterPoint)));

        kTest(GoFeatures_NameTypeMapAdd(GO_SURFACE_TOOL_NAME_BOUNDING_BOX GO_FEATURES_NAME_CENTER_POINT, GO_FEATURES_NAME_CENTER_POINT, kTypeOf(GoSurfaceBoundingBoxCenterPoint)));
        kTest(GoFeatures_NameTypeMapAdd(GO_SURFACE_TOOL_NAME_BOUNDING_BOX GO_FEATURES_NAME_AXIS_LINE, GO_FEATURES_NAME_AXIS_LINE, kTypeOf(GoSurfaceBoundingBoxAxisLine)));

        kTest(GoFeatures_NameTypeMapAdd(GO_SURFACE_TOOL_NAME_COUNTERSUNK_HOLE GO_FEATURES_NAME_CENTER_POINT, GO_FEATURES_NAME_CENTER_POINT, kTypeOf(GoSurfaceCountersunkHoleCenterPoint)));
        // TODO: confirm if surface dimension tool has a feature or not.
        kTest(GoFeatures_NameTypeMapAdd(GO_SURFACE_TOOL_NAME_DIMENSION GO_FEATURES_NAME_CENTER_POINT, GO_FEATURES_NAME_CENTER_POINT, kTypeOf(GoSurfaceDimensionCenterPoint)));

        kTest(GoFeatures_NameTypeMapAdd(GO_EXT_TOOL_NAME_SURFACE_EDGE GO_FEATURES_NAME_EDGE_LINE, GO_FEATURES_NAME_EDGE_LINE, kTypeOf(GoSurfaceEdgeEdgeLine)));
        kTest(GoFeatures_NameTypeMapAdd(GO_EXT_TOOL_NAME_SURFACE_EDGE GO_FEATURES_NAME_CENTER_POINT, GO_FEATURES_NAME_CENTER_POINT, kTypeOf(GoSurfaceEdgeCenterPoint)));

        kTest(GoFeatures_NameTypeMapAdd(GO_SURFACE_TOOL_NAME_ELLIPSE GO_FEATURES_NAME_CENTER_POINT, GO_FEATURES_NAME_CENTER_POINT, kTypeOf(GoSurfaceEllipseCenterPoint)));
        kTest(GoFeatures_NameTypeMapAdd(GO_SURFACE_TOOL_NAME_ELLIPSE GO_FEATURES_NAME_MAJOR_AXIS_LINE, GO_FEATURES_NAME_MAJOR_AXIS_LINE, kTypeOf(GoSurfaceEllipseMajorAxisLine)));
        kTest(GoFeatures_NameTypeMapAdd(GO_SURFACE_TOOL_NAME_ELLIPSE GO_FEATURES_NAME_MINOR_AXIS_LINE, GO_FEATURES_NAME_MINOR_AXIS_LINE, kTypeOf(GoSurfaceEllipseMinorAxisLine)));

        kTest(GoFeatures_NameTypeMapAdd(GO_SURFACE_TOOL_NAME_OPENING GO_FEATURES_NAME_CENTER_POINT, GO_FEATURES_NAME_CENTER_POINT, kTypeOf(GoSurfaceOpeningCenterPoint)));
        kTest(GoFeatures_NameTypeMapAdd(GO_SURFACE_TOOL_NAME_PLANE GO_FEATURES_NAME_PLANE, GO_FEATURES_NAME_PLANE, kTypeOf(GoSurfacePlanePlane)));
        kTest(GoFeatures_NameTypeMapAdd(GO_SURFACE_TOOL_NAME_POSITION GO_FEATURES_NAME_POINT, GO_FEATURES_NAME_POINT, kTypeOf(GoSurfacePositionPoint)));
        // TODO: check if original code was missing GoSurfaceHoleCenterPoint because tool has feature.
        kTest(GoFeatures_NameTypeMapAdd(GO_SURFACE_TOOL_NAME_HOLE GO_FEATURES_NAME_CENTER_POINT, GO_FEATURES_NAME_CENTER_POINT, kTypeOf(GoSurfaceHoleCenterPoint)));

        kTest(GoFeatures_NameTypeMapAdd(GO_SURFACE_TOOL_NAME_STUD GO_FEATURES_NAME_TIP_POINT, GO_FEATURES_NAME_TIP_POINT, kTypeOf(GoSurfaceStudTipPoint)));
        kTest(GoFeatures_NameTypeMapAdd(GO_SURFACE_TOOL_NAME_STUD GO_FEATURES_NAME_BASE_POINT, GO_FEATURES_NAME_BASE_POINT, kTypeOf(GoSurfaceStudBasePoint)));
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

GoFx(kStatus) GoFeatures_CreateNameTypeMap(void)
{
    kCheck(GoFeatures_NameTypeMapInit());

    // The table should have at least one entry.
    kCheckTrue((GoFeatures_NameTypeMapEntryCount() > 0), kERROR);

    // Table is now available.
    return kOK;
}

GoFx(kStatus) GoFeatures_ParseTypeHelper(const kChar* name, kType* type)
{
    kStaticObj(GoFeatures);
    kSize i;
    GoFeaturesNameTypeMapping *map;

    for (i = 0; i < kArrayList_Count(sobj->nameTypeMapList); ++i)
    {
        map = kArrayList_AtT(sobj->nameTypeMapList, i, GoFeaturesNameTypeMapping);
        if (kStrCompare(map->lookupName, name) == 0)
        {
            *type = map->featureType;
            return kOK;
        }
    }

    return kERROR_NOT_FOUND;
}

GoFx(kStatus) GoFeatures_ParseType(const kChar* toolName, const kChar* featureName, kType* type)
{
    kChar* featureType = kNULL;
    kSize capacity = kStrLength(toolName) + kStrLength(featureName) + 1;

    kTry
    {
        // Concatenate the tool name and feature name to form
        // the lookup name.
        kTest(kMemAllocZero(sizeof(kChar) * capacity, &featureType));
        kTest(kStrCopy(featureType, capacity, toolName));
        kTest(kStrCat(featureType, capacity, featureName));

        kTest(GoFeatures_ParseTypeHelper(featureType, type));
    }
    kFinally
    {
        if (featureType)
        {
            // Want to propagate the original failure in the try block so
            // don't check this memory free operation.
            kMemFree(featureType);
        }
        kEndFinally();
    }

    return kOK;
}

GoFx(kStatus) GoFeatures_FormatTypeHelper(kType feature, const kChar** outputPtr)
{
    kStaticObj(GoFeatures);
    kSize i;
    GoFeaturesNameTypeMapping map;

    for (i = 0; i < kArrayList_Count(sobj->nameTypeMapList); ++i)
    {
        map = *kArrayList_AtT(sobj->nameTypeMapList, i, GoFeaturesNameTypeMapping);
        if (feature == map.featureType)
        {
            *outputPtr = map.featureName;
            break;
        }
    }

    return kOK;
}

// Pre-existing call before GOC-14127 that only uses the mapping
// table to map feature type to feature name.
// Does not handle INTERNAL GDK tools, only USER GDK tools.
// GoFeatures_FormatTypeExtToolFormat() adds support for INTERNAL
// GDK tool mapping of feature type to feature name.
GoFx(kStatus) GoFeatures_FormatType(kType feature, kChar* featureTypeName, kSize capacity)
{
    const kChar* output = kNULL;

    kCheck(GoFeatures_FormatTypeHelper(feature, &output));

    kCheck(kStrCopy(featureTypeName, capacity, output));

    return kOK;
}

// GOC-14127: This function is called internally by the SDK to get the feature type name.
// The mapping table always maps GoExtFeature to GO_FEATURES_NAME_CUSTOM (ie. USER GDK
// tool) and cannot support returning GO_FEATURES_NAME_FEATURE for INTERNAL
// GDK tool GoExtFeature.
// This API accepts a flag indicating if the feature is from
// a INTERNAL tool or not, and return GO_FEATURES_NAME_FEATURE for
// INTERNAL tool features.
GoFx(kStatus) GoFeatures_FormatTypeExtToolFormat(kType feature, kBool internalTool, kChar* featureTypeName, kSize capacity)
{
    const kChar* output = kNULL;

    if (internalTool)
    {
        // Internal GDK format tool XML node feature should be called GO_FEATURES_NAME_FEATURE.
        // Older SDKs might have labelled the feature as GO_FEATURES_NAME_CUSTOM, so ignore what
        // is read and set the feature name to the correct internal GDK tool feature name.
        output = GO_FEATURES_NAME_FEATURE;
    }
    else
    {
        // Format is standard tool format, or USER GDK tool or not set (unknown),
        // so use the name from the mapping table.
        kCheck(GoFeatures_FormatTypeHelper(feature, &output));
    }

    kCheck(kStrCopy(featureTypeName, capacity, output));

    return kOK;
}

kBeginClassEx(Go, GoExtFeature)
    kAddVMethod(GoExtFeature, GoFeature, VInit)
    kAddVMethod(GoExtFeature, GoFeature, VRead)
    kAddVMethod(GoExtFeature, GoFeature, VWrite)
    kAddVMethod(GoExtFeature, kObject, VRelease)
kEndClassEx()

GoFx(kStatus) GoExtFeature_Construct(GoExtFeature* feature, kType type, kObject sensor, kObject srcTool, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback(allocator);
    kStatus status;

    kCheck(kAlloc_GetObject(alloc, type, feature));

    if (!kSuccess(status = kType_VTableT(type, GoFeature)->VInit(*feature, type, sensor, srcTool, alloc)))
    {
        kAlloc_FreeRef(alloc, feature);
    }

    return status;
}

GoFx(kStatus) GoExtFeature_VInit(GoExtFeature feature, kType type, kObject sensor, kObject srcTool, kAlloc alloc)
{
    kObjR(GoExtFeature, feature);
    kStatus exception;

    kCheck(GoFeature_Init(feature, type, GO_FEATURE_EXTENSIBLE, sensor, srcTool, GO_FEATURE_DATA_UNKNOWN, alloc));
    obj->params = kNULL;

    kTry
    {
        kTest(GoExtParams_Construct(&obj->params, sensor, alloc));
    }
    kCatch(&exception)
    {
        GoExtFeature_VRelease(feature);
        kEndCatch(exception);
    }

    return kOK;
}

GoFx(kStatus) GoExtFeature_VRead(GoExtFeature feature, kXml xml, kXmlItem item)
{
    kObj(GoExtFeature, feature);
    kXmlItem paramsItem = kNULL;
    kText64 tempStr;

    obj->base.xml = xml;
    obj->base.xmlItem = item;

    if (!kSuccess(kXml_Attr32s(xml, item, "id", &obj->base.id)))
    {
        obj->base.id = GO_FEATURE_UNASSIGNED_ID;
    }

    kCheck(kXml_ChildText(xml, item, "Name", obj->base.name, kCountOf(obj->base.name)));

    kCheck(kXml_ChildBool(xml, item, "Enabled", &obj->base.enabled));

    kCheck(kXml_AttrText(xml, item, "dataType", tempStr, kCountOf(tempStr)));
    kCheck(GoFeatures_ParseDataType(tempStr, &obj->base.dataType));

    kCheck(kXml_AttrText(xml, item, "type", obj->base.type, kCountOf(obj->base.type)));
    if (kXml_ChildExists(xml, item, "Parameters"))
    {
        kCheck(!kIsNull(paramsItem = kXml_Child(xml, item, "Parameters")));
        kCheck(GoExtParams_Read(obj->params, xml, paramsItem));
    }
    return kOK;
}

GoFx(kStatus) GoExtFeature_VWrite(GoExtFeature feature, kXml xml, kXmlItem item)
{
    kObj(GoExtFeature, feature);
    kXmlItem paramsItem = kNULL;
    kText64 tempStr;

    kCheck(kXml_SetAttr32s(xml, item, "id", obj->base.id));
    kCheck(kXml_SetChildText(xml, item, "Name", obj->base.name));

    kCheck(kXml_SetChildBool(xml, item, "Enabled", obj->base.enabled));

    if (obj->base.dataType != GO_FEATURE_DATA_UNKNOWN)
    {
        kCheck(GoFeatures_FormatDataType(obj->base.dataType, tempStr, kCountOf(tempStr)));
        kCheck(kXml_SetAttrText(xml, item, "dataType", tempStr));
    }

    //kCheck(GoFeatures_FormatType(kObject_Type(feature), tempStr, kCountOf(tempStr)));
    kCheck(kXml_SetAttrText(xml, item, "type", obj->base.type));

    kCheck(GoUtils_XmlMerge(obj->base.xml, obj->base.xmlItem, xml, item));

    kCheck(kXml_AddItem(xml, item, "Parameters", &paramsItem));
    kCheck(GoExtParams_Write(obj->params, xml, paramsItem));

    return kOK;
}

GoFx(kStatus) GoExtFeature_VRelease(GoExtFeature feature)
{
    kObj(GoExtFeature, feature);

    kCheck(kDisposeRef(&obj->params));

    return GoFeature_VRelease(feature);
}

GoFx(kSize) GoExtFeature_CustomParameterCount(GoExtFeature feature)
{
    kObj(GoExtFeature, feature);
    kObjN(GoFeature, baseobj, feature);

    GoSensor_SyncConfig(baseobj->sensor);

    return GoExtParams_ParameterCount(obj->params);
}

GoFx(GoExtParam) GoExtFeature_CustomParameterAt(GoExtFeature feature, kSize index)
{
    kObj(GoExtFeature, feature);
    kObjN(GoFeature, baseobj, feature);

    GoSensor_SyncConfig(baseobj->sensor);

    return GoExtParams_ParameterAt(obj->params, index);
}


GoFx(kStatus) GoFeatures_ParseDataType(const kChar* featureDataType, GoFeatureType* type)
{
    if      (strcmp(featureDataType, "PointFeature") == 0)       *type = GO_FEATURE_DATA_POINT;
    else if (strcmp(featureDataType, "LineFeature") == 0)        *type = GO_FEATURE_DATA_LINE;
    else if (strcmp(featureDataType, "CircleFeature") == 0)      *type = GO_FEATURE_DATA_CIRCLE;
    else if (strcmp(featureDataType, "PlaneFeature") == 0)       *type = GO_FEATURE_DATA_PLANE;
    else
    {
        *type = GO_FEATURE_DATA_UNKNOWN;
    }

    return kOK;
}

GoFx(kStatus) GoFeatures_FormatDataType(GoFeatureDataType type, kChar* featureDataType, kSize capacity)
{
    kStatus status = kOK;
    const kChar* output = kNULL;

    switch (type)
    {
    case GO_FEATURE_DATA_POINT  : output = "PointFeature"; break;
    case GO_FEATURE_DATA_LINE   : output = "LineFeature"; break;
    case GO_FEATURE_DATA_CIRCLE : output = "CircleFeature"; break;
    case GO_FEATURE_DATA_PLANE  : output = "PlaneFeature"; break;
    case GO_FEATURE_DATA_UNKNOWN: output = "UnknownFeature"; break;
    default: return kERROR_PARAMETER;
    }

    strncpy(featureDataType, output, capacity - 1);
    featureDataType[capacity - 1] = 0;

    return kOK;
}

kBeginClassEx(Go, GoSurfaceEdgeEdgeLine)
    kAddVMethod(GoSurfaceEdgeEdgeLine, GoFeature, VInit)
    //kAddVMethod(GoSurfaceEdgeEdgeLine, GoFeature, VRead)
    //kAddVMethod(GoSurfaceEdgeEdgeLine, GoFeature, VWrite)
kEndClassEx()

GoFx(kStatus) GoSurfaceEdgeEdgeLine_VInit(GoSurfaceEdgeEdgeLine feature, kType type, kObject sensor, kObject srcTool, kAlloc alloc)
{
    kCheck(GoFeature_Init(feature, type, GO_FEATURE_SURFACE_EDGE_EDGE_LINE, sensor, srcTool, GO_FEATURE_DATA_LINE, alloc));

    return kOK;
}
/*
GoFx(kStatus) GoSurfaceEdgeEdgeLine_VRead(GoFeature feature, kXml xml, kXmlItem item)
{
    return GoFeature_VRead(feature, xml,item);
}

GoFx(kStatus) GoSurfaceEdgeEdgeLine_VWrite(GoFeature feature, kXml xml, kXmlItem item)
{
    return GoFeature_VWrite(feature, xml, item);
}
*/
kBeginClassEx(Go, GoSurfaceEdgeCenterPoint)
    kAddVMethod(GoSurfaceEdgeCenterPoint, GoFeature, VInit)
    kAddVMethod(GoSurfaceEdgeCenterPoint, GoFeature, VRead)
    kAddVMethod(GoSurfaceEdgeCenterPoint, GoFeature, VWrite)
kEndClassEx()

GoFx(kStatus) GoSurfaceEdgeCenterPoint_VInit(GoSurfaceEdgeCenterPoint feature, kType type, kObject sensor, kObject srcTool, kAlloc alloc)
{
    kCheck(GoFeature_Init(feature, type, GO_FEATURE_SURFACE_EDGE_EDGE_LINE, sensor, srcTool, GO_FEATURE_DATA_LINE, alloc));

    return kOK;
}

GoFx(kStatus) GoSurfaceEdgeCenterPoint_VRead(GoFeature feature, kXml xml, kXmlItem item)
{
    return GoFeature_VRead(feature, xml, item);
}

GoFx(kStatus) GoSurfaceEdgeCenterPoint_VWrite(GoFeature feature, kXml xml, kXmlItem item)
{
    return GoFeature_VWrite(feature, xml, item);
}

kBeginClassEx(Go, GoSurfaceBoundingBoxCenterPoint)
    kAddVMethod(GoSurfaceBoundingBoxCenterPoint, GoFeature, VInit)
    kAddVMethod(GoSurfaceBoundingBoxCenterPoint, GoFeature, VRead)
    kAddVMethod(GoSurfaceBoundingBoxCenterPoint, GoFeature, VWrite)
kEndClassEx()

GoFx(kStatus) GoSurfaceBoundingBoxCenterPoint_VInit(GoSurfaceBoundingBoxCenterPoint feature, kType type, kObject sensor, kObject srcTool, kAlloc alloc)
{
    kCheck(GoFeature_Init(feature, type, GO_FEATURE_SURFACE_BOUNDING_BOX_CENTER_POINT, sensor, srcTool, GO_FEATURE_DATA_POINT, alloc));

    return kOK;
}

GoFx(kStatus) GoSurfaceBoundingBoxCenterPoint_VRead(GoFeature feature, kXml xml, kXmlItem item)
{
    return GoFeature_VRead(feature, xml, item);
}

GoFx(kStatus) GoSurfaceBoundingBoxCenterPoint_VWrite(GoFeature feature, kXml xml, kXmlItem item)
{
    return GoFeature_VWrite(feature, xml, item);
}

kBeginClassEx(Go, GoSurfaceBoundingBoxAxisLine)
kAddVMethod(GoSurfaceBoundingBoxAxisLine, GoFeature, VInit)
kAddVMethod(GoSurfaceBoundingBoxAxisLine, GoFeature, VRead)
kAddVMethod(GoSurfaceBoundingBoxAxisLine, GoFeature, VWrite)
kEndClassEx()

GoFx(kStatus) GoSurfaceBoundingBoxAxisLine_VInit(GoSurfaceBoundingBoxAxisLine feature, kType type, kObject sensor, kObject srcTool, kAlloc alloc)
{
    kCheck(GoFeature_Init(feature, type, GO_FEATURE_SURFACE_BOUNDING_BOX_AXIS_LINE, sensor, srcTool, GO_FEATURE_DATA_POINT, alloc));

    return kOK;
}

GoFx(kStatus) GoSurfaceBoundingBoxAxisLine_VRead(GoFeature feature, kXml xml, kXmlItem item)
{
    return GoFeature_VRead(feature, xml, item);
}

GoFx(kStatus) GoSurfaceBoundingBoxAxisLine_VWrite(GoFeature feature, kXml xml, kXmlItem item)
{
    return GoFeature_VWrite(feature, xml, item);
}

kBeginClassEx(Go, GoSurfaceCountersunkHoleCenterPoint)
    kAddVMethod(GoSurfaceCountersunkHoleCenterPoint, GoFeature, VInit)
    kAddVMethod(GoSurfaceCountersunkHoleCenterPoint, GoFeature, VRead)
    kAddVMethod(GoSurfaceCountersunkHoleCenterPoint, GoFeature, VWrite)
kEndClassEx()

GoFx(kStatus) GoSurfaceCountersunkHoleCenterPoint_VInit(GoSurfaceCountersunkHoleCenterPoint feature, kType type, kObject sensor, kObject srcTool, kAlloc alloc)
{
    kCheck(GoFeature_Init(feature, type, GO_FEATURE_SURFACE_COUNTERSUNKHOLE_CENTER_POINT, sensor, srcTool, GO_FEATURE_DATA_POINT, alloc));

    return kOK;
}

GoFx(kStatus) GoSurfaceCountersunkHoleCenterPoint_VRead(GoFeature feature, kXml xml, kXmlItem item)
{
    return GoFeature_VRead(feature, xml, item);
}

GoFx(kStatus) GoSurfaceCountersunkHoleCenterPoint_VWrite(GoFeature feature, kXml xml, kXmlItem item)
{
    return GoFeature_VWrite(feature, xml, item);
}

kBeginClassEx(Go, GoSurfaceDimensionCenterPoint)
    kAddVMethod(GoSurfaceDimensionCenterPoint, GoFeature, VInit)
    kAddVMethod(GoSurfaceDimensionCenterPoint, GoFeature, VRead)
    kAddVMethod(GoSurfaceDimensionCenterPoint, GoFeature, VWrite)
kEndClassEx()

GoFx(kStatus) GoSurfaceDimensionCenterPoint_VInit(GoSurfaceDimensionCenterPoint feature, kType type, kObject sensor, kObject srcTool, kAlloc alloc)
{
    kCheck(GoFeature_Init(feature, type, GO_FEATURE_SURFACE_DIMENSION_CENTER_POINT, sensor, srcTool, GO_FEATURE_DATA_POINT, alloc));

    return kOK;
}

GoFx(kStatus) GoSurfaceDimensionCenterPoint_VRead(GoFeature feature, kXml xml, kXmlItem item)
{
    return GoFeature_VRead(feature, xml, item);
}

GoFx(kStatus) GoSurfaceDimensionCenterPoint_VWrite(GoFeature feature, kXml xml, kXmlItem item)
{
    return GoFeature_VWrite(feature, xml, item);
}

kBeginClassEx(Go, GoSurfaceEllipseCenterPoint)
    kAddVMethod(GoSurfaceEllipseCenterPoint, GoFeature, VInit)
    kAddVMethod(GoSurfaceEllipseCenterPoint, GoFeature, VRead)
    kAddVMethod(GoSurfaceEllipseCenterPoint, GoFeature, VWrite)
kEndClassEx()

GoFx(kStatus)GoSurfaceEllipseCenterPoint_VInit(GoSurfaceEllipseCenterPoint feature, kType type, kObject sensor, kObject srcTool, kAlloc alloc)
{
    kCheck(GoFeature_Init(feature, type, GO_FEATURE_SURFACE_ELLIPSE_CENTER_POINT, sensor, srcTool, GO_FEATURE_DATA_LINE, alloc));

    return kOK;
}

GoFx(kStatus) GoSurfaceEllipseCenterPoint_VRead(GoFeature feature, kXml xml, kXmlItem item)
{
    return GoFeature_VRead(feature, xml, item);
}

GoFx(kStatus) GoSurfaceEllipseCenterPoint_VWrite(GoFeature feature, kXml xml, kXmlItem item)
{
    return GoFeature_VWrite(feature, xml, item);
}

kBeginClassEx(Go, GoSurfaceEllipseMajorAxisLine)
    kAddVMethod(GoSurfaceEllipseMajorAxisLine, GoFeature, VInit)
    kAddVMethod(GoSurfaceEllipseMajorAxisLine, GoFeature, VRead)
    kAddVMethod(GoSurfaceEllipseMajorAxisLine, GoFeature, VWrite)
kEndClassEx()

GoFx(kStatus) GoSurfaceEllipseMajorAxisLine_VInit(GoSurfaceEllipseMajorAxisLine feature, kType type, kObject sensor, kObject srcTool, kAlloc alloc)
{
    kCheck(GoFeature_Init(feature, type, GO_FEATURE_SURFACE_ELLIPSE_MAJOR_AXIS_LINE, sensor, srcTool, GO_FEATURE_DATA_LINE, alloc));

    return kOK;
}

GoFx(kStatus) GoSurfaceEllipseMajorAxisLine_VRead(GoFeature feature, kXml xml, kXmlItem item)
{
    return GoFeature_VRead(feature, xml, item);
}

GoFx(kStatus) GoSurfaceEllipseMajorAxisLine_VWrite(GoFeature feature, kXml xml, kXmlItem item)
{
    return GoFeature_VWrite(feature, xml, item);
}

kBeginClassEx(Go, GoSurfaceEllipseMinorAxisLine)
    kAddVMethod(GoSurfaceEllipseMinorAxisLine, GoFeature, VInit)
    kAddVMethod(GoSurfaceEllipseMinorAxisLine, GoFeature, VRead)
    kAddVMethod(GoSurfaceEllipseMinorAxisLine, GoFeature, VWrite)
kEndClassEx()

GoFx(kStatus) GoSurfaceEllipseMinorAxisLine_VInit(GoSurfaceEllipseMinorAxisLine feature, kType type, kObject sensor, kObject srcTool, kAlloc alloc)
{
    kCheck(GoFeature_Init(feature, type, GO_FEATURE_SURFACE_ELLIPSE_MINOR_AXIS_LINE, sensor, srcTool, GO_FEATURE_DATA_LINE, alloc));

    return kOK;
}

GoFx(kStatus) GoSurfaceEllipseMinorAxisLine_VRead(GoFeature feature, kXml xml, kXmlItem item)
{
    return GoFeature_VRead(feature, xml, item);
}

GoFx(kStatus) GoSurfaceEllipseMinorAxisLine_VWrite(GoFeature feature, kXml xml, kXmlItem item)
{
    return GoFeature_VWrite(feature, xml, item);
}

kBeginClassEx(Go, GoSurfaceHoleCenterPoint)
    kAddVMethod(GoSurfaceHoleCenterPoint, GoFeature, VInit)
    kAddVMethod(GoSurfaceHoleCenterPoint, GoFeature, VRead)
    kAddVMethod(GoSurfaceHoleCenterPoint, GoFeature, VWrite)
kEndClassEx()

GoFx(kStatus) GoSurfaceHoleCenterPoint_VInit(GoSurfaceHoleCenterPoint feature, kType type, kObject sensor, kObject srcTool, kAlloc alloc)
{
    kCheck(GoFeature_Init(feature, type, GO_FEATURE_SURFACE_HOLE_CENTER_POINT, sensor, srcTool, GO_FEATURE_DATA_POINT, alloc));

    return kOK;
}

GoFx(kStatus) GoSurfaceHoleCenterPoint_VRead(GoFeature feature, kXml xml, kXmlItem item)
{
    return GoFeature_VRead(feature, xml, item);
}

GoFx(kStatus) GoSurfaceHoleCenterPoint_VWrite(GoFeature feature, kXml xml, kXmlItem item)
{
    return GoFeature_VWrite(feature, xml, item);
}

kBeginClassEx(Go, GoSurfaceOpeningCenterPoint)
    kAddVMethod(GoSurfaceOpeningCenterPoint, GoFeature, VInit)
    kAddVMethod(GoSurfaceOpeningCenterPoint, GoFeature, VRead)
    kAddVMethod(GoSurfaceOpeningCenterPoint, GoFeature, VWrite)
kEndClassEx()

GoFx(kStatus) GoSurfaceOpeningCenterPoint_VInit(GoSurfaceOpeningCenterPoint feature, kType type, kObject sensor, kObject srcTool, kAlloc alloc)
{
    kCheck(GoFeature_Init(feature, type, GO_FEATURE_SURFACE_OPENING_CENTER_POINT, sensor, srcTool, GO_FEATURE_DATA_POINT, alloc));

    return kOK;
}

GoFx(kStatus) GoSurfaceOpeningCenterPoint_VRead(GoFeature feature, kXml xml, kXmlItem item)
{
    return GoFeature_VRead(feature, xml, item);
}

GoFx(kStatus) GoSurfaceOpeningCenterPoint_VWrite(GoFeature feature, kXml xml, kXmlItem item)
{
    return GoFeature_VWrite(feature, xml, item);
}


kBeginClassEx(Go, GoSurfacePlanePlane)
    kAddVMethod(GoSurfacePlanePlane, GoFeature, VInit)
    kAddVMethod(GoSurfacePlanePlane, GoFeature, VRead)
    kAddVMethod(GoSurfacePlanePlane, GoFeature, VWrite)
kEndClassEx()

GoFx(kStatus) GoSurfacePlanePlane_VInit(GoSurfacePlanePlane feature, kType type, kObject sensor, kObject srcTool, kAlloc alloc)
{
    kCheck(GoFeature_Init(feature, type, GO_FEATURE_SURFACE_PLANE_PLANE, sensor, srcTool, GO_FEATURE_DATA_PLANE, alloc));

    return kOK;
}

GoFx(kStatus) GoSurfacePlanePlane_VRead(GoFeature feature, kXml xml, kXmlItem item)
{
    return GoFeature_VRead(feature, xml, item);
}

GoFx(kStatus) GoSurfacePlanePlane_VWrite(GoFeature feature, kXml xml, kXmlItem item)
{
    return GoFeature_VWrite(feature, xml, item);
}


kBeginClassEx(Go, GoSurfacePositionPoint)
    kAddVMethod(GoSurfacePositionPoint, GoFeature, VInit)
    kAddVMethod(GoSurfacePositionPoint, GoFeature, VRead)
    kAddVMethod(GoSurfacePositionPoint, GoFeature, VWrite)
kEndClassEx()

GoFx(kStatus) GoSurfacePositionPoint_VInit(GoSurfacePositionPoint feature, kType type, kObject sensor, kObject srcTool, kAlloc alloc)
{
    kCheck(GoFeature_Init(feature, type, GO_FEATURE_SURFACE_POSITION_POINT, sensor, srcTool, GO_FEATURE_DATA_POINT, alloc));

    return kOK;
}

GoFx(kStatus) GoSurfacePositionPoint_VRead(GoFeature feature, kXml xml, kXmlItem item)
{
    return GoFeature_VRead(feature, xml, item);
}

GoFx(kStatus) GoSurfacePositionPoint_VWrite(GoFeature feature, kXml xml, kXmlItem item)
{
    return GoFeature_VWrite(feature, xml, item);
}

kBeginClassEx(Go, GoSurfaceStudTipPoint)
    kAddVMethod(GoSurfaceStudTipPoint, GoFeature, VInit)
    kAddVMethod(GoSurfaceStudTipPoint, GoFeature, VRead)
    kAddVMethod(GoSurfaceStudTipPoint, GoFeature, VWrite)
kEndClassEx()

GoFx(kStatus) GoSurfaceStudTipPoint_VInit(GoSurfaceStudTipPoint feature, kType type, kObject sensor, kObject srcTool, kAlloc alloc)
{
    kCheck(GoFeature_Init(feature, type, GO_FEATURE_SURFACE_STUD_TIP_POINT, sensor, srcTool, GO_FEATURE_DATA_POINT, alloc));

    return kOK;
}

GoFx(kStatus) GoSurfaceStudTipPoint_VRead(GoFeature feature, kXml xml, kXmlItem item)
{
    return GoFeature_VRead(feature, xml, item);
}

GoFx(kStatus) GoSurfaceStudTipPoint_VWrite(GoFeature feature, kXml xml, kXmlItem item)
{
    return GoFeature_VWrite(feature, xml, item);
}


kBeginClassEx(Go, GoSurfaceStudBasePoint)
    kAddVMethod(GoSurfaceStudBasePoint, GoFeature, VInit)
    kAddVMethod(GoSurfaceStudBasePoint, GoFeature, VRead)
    kAddVMethod(GoSurfaceStudBasePoint, GoFeature, VWrite)
kEndClassEx()

GoFx(kStatus) GoSurfaceStudBasePoint_VInit(GoSurfaceStudBasePoint feature, kType type, kObject sensor, kObject srcTool, kAlloc alloc)
{
    kCheck(GoFeature_Init(feature, type, GO_FEATURE_SURFACE_STUD_BASE_POINT, sensor, srcTool, GO_FEATURE_DATA_POINT, alloc));

    return kOK;
}

GoFx(kStatus) GoSurfaceStudBasePoint_VRead(GoFeature feature, kXml xml, kXmlItem item)
{
    return GoFeature_VRead(feature, xml, item);
}

GoFx(kStatus) GoSurfaceStudBasePoint_VWrite(GoFeature feature, kXml xml, kXmlItem item)
{
    return GoFeature_VWrite(feature, xml, item);
}

kBeginClassEx(Go, GoProfilePositionPoint)
    kAddVMethod(GoProfilePositionPoint, GoFeature, VInit)
    kAddVMethod(GoProfilePositionPoint, GoFeature, VRead)
    kAddVMethod(GoProfilePositionPoint, GoFeature, VWrite)
kEndClassEx()

GoFx(kStatus) GoProfilePositionPoint_VInit(GoProfilePositionPoint feature, kType type, kObject sensor, kObject srcTool, kAlloc alloc)
{
    kCheck(GoFeature_Init(feature, type, GO_FEATURE_PROFILE_POSITION_POINT, sensor, srcTool, GO_FEATURE_DATA_POINT, alloc));

    return kOK;
}

GoFx(kStatus) GoProfilePositionPoint_VRead(GoFeature feature, kXml xml, kXmlItem item)
{
    return GoFeature_VRead(feature, xml, item);
}

GoFx(kStatus) GoProfilePositionPoint_VWrite(GoFeature feature, kXml xml, kXmlItem item)
{
    return GoFeature_VWrite(feature, xml, item);
}

kBeginClassEx(Go, GoProfileLineLine)
    kAddVMethod(GoProfileLineLine, GoFeature, VInit)
    kAddVMethod(GoProfileLineLine, GoFeature, VRead)
    kAddVMethod(GoProfileLineLine, GoFeature, VWrite)
kEndClassEx()

GoFx(kStatus) GoProfileLineLine_VInit(GoProfileLineLine feature, kType type, kObject sensor, kObject srcTool, kAlloc alloc)
{
    kCheck(GoFeature_Init(feature, type, GO_FEATURE_PROFILE_LINE_LINE, sensor, srcTool, GO_FEATURE_DATA_LINE, alloc));

    return kOK;
}

GoFx(kStatus) GoProfileLineLine_VRead(GoFeature feature, kXml xml, kXmlItem item)
{
    return GoFeature_VRead(feature, xml, item);
}

GoFx(kStatus) GoProfileLineLine_VWrite(GoFeature feature, kXml xml, kXmlItem item)
{
    return GoFeature_VWrite(feature, xml, item);
}

kBeginClassEx(Go, GoProfileLineMinErrorPoint)
    kAddVMethod(GoProfileLineMinErrorPoint, GoFeature, VInit)
    kAddVMethod(GoProfileLineMinErrorPoint, GoFeature, VRead)
    kAddVMethod(GoProfileLineMinErrorPoint, GoFeature, VWrite)
kEndClassEx()

GoFx(kStatus) GoProfileLineMinErrorPoint_VInit(GoProfileLineMinErrorPoint feature, kType type, kObject sensor, kObject srcTool, kAlloc alloc)
{
    kCheck(GoFeature_Init(feature, type, GO_FEATURE_PROFILE_LINE_MIN_ERROR_POINT, sensor, srcTool, GO_FEATURE_DATA_POINT, alloc));

    return kOK;
}

GoFx(kStatus) GoProfileLineMinErrorPoint_VRead(GoFeature feature, kXml xml, kXmlItem item)
{
    return GoFeature_VRead(feature, xml, item);
}

GoFx(kStatus) GoProfileLineMinErrorPoint_VWrite(GoFeature feature, kXml xml, kXmlItem item)
{
    return GoFeature_VWrite(feature, xml, item);
}


kBeginClassEx(Go, GoProfileLineMaxErrorPoint)
    kAddVMethod(GoProfileLineMaxErrorPoint, GoFeature, VInit)
    kAddVMethod(GoProfileLineMaxErrorPoint, GoFeature, VRead)
    kAddVMethod(GoProfileLineMaxErrorPoint, GoFeature, VWrite)
kEndClassEx()

GoFx(kStatus) GoProfileLineMaxErrorPoint_VInit(GoProfileLineMaxErrorPoint feature, kType type, kObject sensor, kObject srcTool, kAlloc alloc)
{
    kCheck(GoFeature_Init(feature, type, GO_FEATURE_PROFILE_LINE_MAX_ERROR_POINT, sensor, srcTool, GO_FEATURE_DATA_POINT, alloc));

    return kOK;
}

GoFx(kStatus) GoProfileLineMaxErrorPoint_VRead(GoFeature feature, kXml xml, kXmlItem item)
{
    return GoFeature_VRead(feature, xml, item);
}

GoFx(kStatus) GoProfileLineMaxErrorPoint_VWrite(GoFeature feature, kXml xml, kXmlItem item)
{
    return GoFeature_VWrite(feature, xml, item);
}


kBeginClassEx(Go, GoProfileIntersectIntersectPoint)
    kAddVMethod(GoProfileIntersectIntersectPoint, GoFeature, VInit)
    kAddVMethod(GoProfileIntersectIntersectPoint, GoFeature, VRead)
    kAddVMethod(GoProfileIntersectIntersectPoint, GoFeature, VWrite)
kEndClassEx()

GoFx(kStatus) GoProfileIntersectIntersectPoint_VInit(GoProfileIntersectIntersectPoint feature, kType type, kObject sensor, kObject srcTool, kAlloc alloc)
{
    kCheck(GoFeature_Init(feature, type, GO_FEATURE_PROFILE_INTERSECT_INTERSECT_POINT, sensor, srcTool, GO_FEATURE_DATA_POINT, alloc));

    return kOK;
}

GoFx(kStatus) GoProfileIntersectIntersectPoint_VRead(GoFeature feature, kXml xml, kXmlItem item)
{
    return GoFeature_VRead(feature, xml, item);
}

GoFx(kStatus) GoProfileIntersectIntersectPoint_VWrite(GoFeature feature, kXml xml, kXmlItem item)
{
    return GoFeature_VWrite(feature, xml, item);
}

kBeginClassEx(Go, GoProfileIntersectLine)
    kAddVMethod(GoProfileIntersectLine, GoFeature, VInit)
    kAddVMethod(GoProfileIntersectLine, GoFeature, VRead)
    kAddVMethod(GoProfileIntersectLine, GoFeature, VWrite)
kEndClassEx()

GoFx(kStatus) GoProfileIntersectLine_VInit(GoProfileIntersectLine feature, kType type, kObject sensor, kObject srcTool, kAlloc alloc)
{
    kCheck(GoFeature_Init(feature, type, GO_FEATURE_PROFILE_INTERSECT_LINE, sensor, srcTool, GO_FEATURE_DATA_LINE, alloc));

    return kOK;
}

GoFx(kStatus) GoProfileIntersectLine_VRead(GoFeature feature, kXml xml, kXmlItem item)
{
    return GoFeature_VRead(feature, xml, item);
}

GoFx(kStatus) GoProfileIntersectLine_VWrite(GoFeature feature, kXml xml, kXmlItem item)
{
    return GoFeature_VWrite(feature, xml, item);
}

kBeginClassEx(Go, GoProfileIntersectBaseLine)
    kAddVMethod(GoProfileIntersectBaseLine, GoFeature, VInit)
    kAddVMethod(GoProfileIntersectBaseLine, GoFeature, VRead)
    kAddVMethod(GoProfileIntersectBaseLine, GoFeature, VWrite)
kEndClassEx()

GoFx(kStatus) GoProfileIntersectBaseLine_VInit(GoProfileIntersectBaseLine feature, kType type, kObject sensor, kObject srcTool, kAlloc alloc)
{
    kCheck(GoFeature_Init(feature, type, GO_FEATURE_PROFILE_INTERSECT_BASE_LINE, sensor, srcTool, GO_FEATURE_DATA_LINE, alloc));

    return kOK;
}

GoFx(kStatus) GoProfileIntersectBaseLine_VRead(GoFeature feature, kXml xml, kXmlItem item)
{
    return GoFeature_VRead(feature, xml, item);
}

GoFx(kStatus) GoProfileIntersectBaseLine_VWrite(GoFeature feature, kXml xml, kXmlItem item)
{
    return GoFeature_VWrite(feature, xml, item);
}

kBeginClassEx(Go, GoProfileBoundingBoxCornerPoint)
    kAddVMethod(GoProfileBoundingBoxCornerPoint, GoFeature, VInit)
    kAddVMethod(GoProfileBoundingBoxCornerPoint, GoFeature, VRead)
    kAddVMethod(GoProfileBoundingBoxCornerPoint, GoFeature, VWrite)
kEndClassEx()

GoFx(kStatus) GoProfileBoundingBoxCornerPoint_VInit(GoProfileBoundingBoxCornerPoint feature, kType type, kObject sensor, kObject srcTool, kAlloc alloc)
{
    kCheck(GoFeature_Init(feature, type, GO_FEATURE_PROFILE_BOUNDING_BOX_CORNER_POINT, sensor, srcTool, GO_FEATURE_DATA_POINT, alloc));

    return kOK;
}

GoFx(kStatus) GoProfileBoundingBoxCornerPoint_VRead(GoFeature feature, kXml xml, kXmlItem item)
{
    return GoFeature_VRead(feature, xml, item);
}

GoFx(kStatus) GoProfileBoundingBoxCornerPoint_VWrite(GoFeature feature, kXml xml, kXmlItem item)
{
    return GoFeature_VWrite(feature, xml, item);
}

kBeginClassEx(Go, GoProfileBoundingBoxCenterPoint)
    kAddVMethod(GoProfileBoundingBoxCenterPoint, GoFeature, VInit)
    kAddVMethod(GoProfileBoundingBoxCenterPoint, GoFeature, VRead)
    kAddVMethod(GoProfileBoundingBoxCenterPoint, GoFeature, VWrite)
kEndClassEx()

GoFx(kStatus) GoProfileBoundingBoxCenterPoint_VInit(GoProfileBoundingBoxCenterPoint feature, kType type, kObject sensor, kObject srcTool, kAlloc alloc)
{
    kCheck(GoFeature_Init(feature, type, GO_FEATURE_PROFILE_BOUNDING_BOX_CENTER_POINT, sensor, srcTool, GO_FEATURE_DATA_POINT, alloc));

    return kOK;
}

GoFx(kStatus) GoProfileBoundingBoxCenterPoint_VRead(GoFeature feature, kXml xml, kXmlItem item)
{
    return GoFeature_VRead(feature, xml, item);
}

GoFx(kStatus) GoProfileBoundingBoxCenterPoint_VWrite(GoFeature feature, kXml xml, kXmlItem item)
{
    return GoFeature_VWrite(feature, xml, item);
}

kBeginClassEx(Go, GoProfileAreaCenterPoint)
    kAddVMethod(GoProfileAreaCenterPoint, GoFeature, VInit)
    kAddVMethod(GoProfileAreaCenterPoint, GoFeature, VRead)
    kAddVMethod(GoProfileAreaCenterPoint, GoFeature, VWrite)
kEndClassEx()

GoFx(kStatus) GoProfileAreaCenterPoint_VInit(GoProfileAreaCenterPoint feature, kType type, kObject sensor, kObject srcTool, kAlloc alloc)
{
    kCheck(GoFeature_Init(feature, type, GO_FEATURE_PROFILE_AREA_CENTER_POINT, sensor, srcTool, GO_FEATURE_DATA_POINT, alloc));

    return kOK;
}

GoFx(kStatus) GoProfileAreaCenterPoint_VRead(GoFeature feature, kXml xml, kXmlItem item)
{
    return GoFeature_VRead(feature, xml, item);
}

GoFx(kStatus) GoProfileAreaCenterPoint_VWrite(GoFeature feature, kXml xml, kXmlItem item)
{
    return GoFeature_VWrite(feature, xml, item);
}

kBeginClassEx(Go, GoProfileCircleCenterPoint)
    kAddVMethod(GoProfileCircleCenterPoint, GoFeature, VInit)
    kAddVMethod(GoProfileCircleCenterPoint, GoFeature, VRead)
    kAddVMethod(GoProfileCircleCenterPoint, GoFeature, VWrite)
kEndClassEx()

GoFx(kStatus) GoProfileCircleCenterPoint_VInit(GoProfileCircleCenterPoint feature, kType type, kObject sensor, kObject srcTool, kAlloc alloc)
{
    kCheck(GoFeature_Init(feature, type, GO_FEATURE_PROFILE_CIRCLE_CENTER_POINT, sensor, srcTool, GO_FEATURE_DATA_POINT, alloc));

    return kOK;
}

GoFx(kStatus) GoProfileCircleCenterPoint_VRead(GoFeature feature, kXml xml, kXmlItem item)
{
    return GoFeature_VRead(feature, xml, item);
}

GoFx(kStatus) GoProfileCircleCenterPoint_VWrite(GoFeature feature, kXml xml, kXmlItem item)
{
    return GoFeature_VWrite(feature, xml, item);
}

kBeginClassEx(Go, GoProfileDimensionCenterPoint)
    kAddVMethod(GoProfileDimensionCenterPoint, GoFeature, VInit)
    kAddVMethod(GoProfileDimensionCenterPoint, GoFeature, VRead)
    kAddVMethod(GoProfileDimensionCenterPoint, GoFeature, VWrite)
kEndClassEx()

GoFx(kStatus) GoProfileDimensionCenterPoint_VInit(GoProfileDimensionCenterPoint feature, kType type, kObject sensor, kObject srcTool, kAlloc alloc)
{
    kCheck(GoFeature_Init(feature, type, GO_FEATURE_PROFILE_DIMENSION_CENTER_POINT, sensor, srcTool, GO_FEATURE_DATA_POINT, alloc));

    return kOK;
}

GoFx(kStatus) GoProfileDimensionCenterPoint_VRead(GoFeature feature, kXml xml, kXmlItem item)
{
    return GoFeature_VRead(feature, xml, item);
}

GoFx(kStatus) GoProfileDimensionCenterPoint_VWrite(GoFeature feature, kXml xml, kXmlItem item)
{
    return GoFeature_VWrite(feature, xml, item);
}


kBeginClassEx(Go, GoProfilePanelLeftGapPoint)
    kAddVMethod(GoProfilePanelLeftGapPoint, GoFeature, VInit)
    kAddVMethod(GoProfilePanelLeftGapPoint, GoFeature, VRead)
    kAddVMethod(GoProfilePanelLeftGapPoint, GoFeature, VWrite)
kEndClassEx()

GoFx(kStatus) GoProfilePanelLeftGapPoint_VInit(GoProfilePanelLeftGapPoint feature, kType type, kObject sensor, kObject srcTool, kAlloc alloc)
{
    kCheck(GoFeature_Init(feature, type, GO_FEATURE_PROFILE_PANEL_LEFT_GAP_POINT, sensor, srcTool, GO_FEATURE_DATA_POINT, alloc));

    return kOK;
}

GoFx(kStatus) GoProfilePanelLeftGapPoint_VRead(GoFeature feature, kXml xml, kXmlItem item)
{
    return GoFeature_VRead(feature, xml, item);
}

GoFx(kStatus) GoProfilePanelLeftGapPoint_VWrite(GoFeature feature, kXml xml, kXmlItem item)
{
    return GoFeature_VWrite(feature, xml, item);
}


kBeginClassEx(Go, GoProfilePanelLeftFlushPoint)
    kAddVMethod(GoProfilePanelLeftFlushPoint, GoFeature, VInit)
    kAddVMethod(GoProfilePanelLeftFlushPoint, GoFeature, VRead)
    kAddVMethod(GoProfilePanelLeftFlushPoint, GoFeature, VWrite)
kEndClassEx()

GoFx(kStatus) GoProfilePanelLeftFlushPoint_VInit(GoProfilePanelLeftFlushPoint feature, kType type, kObject sensor, kObject srcTool, kAlloc alloc)
{
    kCheck(GoFeature_Init(feature, type, GO_FEATURE_PROFILE_PANEL_LEFT_FLUSH_POINT, sensor, srcTool, GO_FEATURE_DATA_POINT, alloc));

    return kOK;
}

GoFx(kStatus) GoProfilePanelLeftFlushPoint_VRead(GoFeature feature, kXml xml, kXmlItem item)
{
    return GoFeature_VRead(feature, xml, item);
}

GoFx(kStatus) GoProfilePanelLeftFlushPoint_VWrite(GoFeature feature, kXml xml, kXmlItem item)
{
    return GoFeature_VWrite(feature, xml, item);
}


kBeginClassEx(Go, GoProfilePanelRightGapPoint)
    kAddVMethod(GoProfilePanelRightGapPoint, GoFeature, VInit)
    kAddVMethod(GoProfilePanelRightGapPoint, GoFeature, VRead)
    kAddVMethod(GoProfilePanelRightGapPoint, GoFeature, VWrite)
kEndClassEx()

GoFx(kStatus) GoProfilePanelRightGapPoint_VInit(GoProfilePanelRightGapPoint feature, kType type, kObject sensor, kObject srcTool, kAlloc alloc)
{
    kCheck(GoFeature_Init(feature, type, GO_FEATURE_PROFILE_PANEL_RIGHT_GAP_POINT, sensor, srcTool, GO_FEATURE_DATA_POINT, alloc));

    return kOK;
}

GoFx(kStatus) GoProfilePanelRightGapPoint_VRead(GoFeature feature, kXml xml, kXmlItem item)
{
    return GoFeature_VRead(feature, xml, item);
}

GoFx(kStatus) GoProfilePanelRightGapPoint_VWrite(GoFeature feature, kXml xml, kXmlItem item)
{
    return GoFeature_VWrite(feature, xml, item);
}

kBeginClassEx(Go, GoProfilePanelRightFlushPoint)
    kAddVMethod(GoProfilePanelRightFlushPoint, GoFeature, VInit)
    kAddVMethod(GoProfilePanelRightFlushPoint, GoFeature, VRead)
    kAddVMethod(GoProfilePanelRightFlushPoint, GoFeature, VWrite)
kEndClassEx()

GoFx(kStatus) GoProfilePanelRightFlushPoint_VInit(GoProfilePanelRightFlushPoint feature, kType type, kObject sensor, kObject srcTool, kAlloc alloc)
{
    kCheck(GoFeature_Init(feature, type, GO_FEATURE_PROFILE_PANEL_RIGHT_FLUSH_POINT, sensor, srcTool, GO_FEATURE_DATA_POINT, alloc));

    return kOK;
}

GoFx(kStatus) GoProfilePanelRightFlushPoint_VRead(GoFeature feature, kXml xml, kXmlItem item)
{
    return GoFeature_VRead(feature, xml, item);
}

GoFx(kStatus) GoProfilePanelRightFlushPoint_VWrite(GoFeature feature, kXml xml, kXmlItem item)
{
    return GoFeature_VWrite(feature, xml, item);
}

kBeginClassEx(Go, GoProfileRoundCornerPoint)
    kAddVMethod(GoProfileRoundCornerPoint, GoFeature, VInit)
    kAddVMethod(GoProfileRoundCornerPoint, GoFeature, VRead)
    kAddVMethod(GoProfileRoundCornerPoint, GoFeature, VWrite)
kEndClassEx()

GoFx(kStatus) GoProfileRoundCornerPoint_VInit(GoProfileRoundCornerPoint feature, kType type, kObject sensor, kObject srcTool, kAlloc alloc)
{
    kCheck(GoFeature_Init(feature, type, GO_FEATURE_PROFILE_ROUND_CORNER_POINT, sensor, srcTool, GO_FEATURE_DATA_POINT, alloc));

    return kOK;
}

GoFx(kStatus) GoProfileRoundCornerPoint_VRead(GoFeature feature, kXml xml, kXmlItem item)
{
    return GoFeature_VRead(feature, xml, item);
}

GoFx(kStatus) GoProfileRoundCornerPoint_VWrite(GoFeature feature, kXml xml, kXmlItem item)
{
    return GoFeature_VWrite(feature, xml, item);
}

kBeginClassEx(Go, GoProfileRoundCornerEdgePoint)
    kAddVMethod(GoProfileRoundCornerEdgePoint, GoFeature, VInit)
    kAddVMethod(GoProfileRoundCornerEdgePoint, GoFeature, VRead)
    kAddVMethod(GoProfileRoundCornerEdgePoint, GoFeature, VWrite)
kEndClassEx()

GoFx(kStatus) GoProfileRoundCornerEdgePoint_VInit(GoProfileRoundCornerEdgePoint feature, kType type, kObject sensor, kObject srcTool, kAlloc alloc)
{
    kCheck(GoFeature_Init(feature, type, GO_FEATURE_PROFILE_ROUND_CORNER_EDGE_POINT, sensor, srcTool, GO_FEATURE_DATA_POINT, alloc));

    return kOK;
}

GoFx(kStatus) GoProfileRoundCornerEdgePoint_VRead(GoFeature feature, kXml xml, kXmlItem item)
{
    return GoFeature_VRead(feature, xml, item);
}

GoFx(kStatus) GoProfileRoundCornerEdgePoint_VWrite(GoFeature feature, kXml xml, kXmlItem item)
{
    return GoFeature_VWrite(feature, xml, item);
}


kBeginClassEx(Go, GoProfileRoundCornerCenterPoint)
    kAddVMethod(GoProfileRoundCornerCenterPoint, GoFeature, VInit)
    kAddVMethod(GoProfileRoundCornerCenterPoint, GoFeature, VRead)
    kAddVMethod(GoProfileRoundCornerCenterPoint, GoFeature, VWrite)
kEndClassEx()

GoFx(kStatus) GoProfileRoundCornerCenterPoint_VInit(GoProfileRoundCornerCenterPoint feature, kType type, kObject sensor, kObject srcTool, kAlloc alloc)
{
    kCheck(GoFeature_Init(feature, type, GO_FEATURE_PROFILE_ROUND_CORNER_CENTER_POINT, sensor, srcTool, GO_FEATURE_DATA_POINT, alloc));

    return kOK;
}

GoFx(kStatus) GoProfileRoundCornerCenterPoint_VRead(GoFeature feature, kXml xml, kXmlItem item)
{
    return GoFeature_VRead(feature, xml, item);
}

GoFx(kStatus) GoProfileRoundCornerCenterPoint_VWrite(GoFeature feature, kXml xml, kXmlItem item)
{
    return GoFeature_VWrite(feature, xml, item);
}

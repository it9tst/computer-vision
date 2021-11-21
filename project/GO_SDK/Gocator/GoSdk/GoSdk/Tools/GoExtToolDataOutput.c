/**
 * @file    GoExtToolDataOutput.c
 *
 * @internal
 * Copyright (C) 2017-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
/*
 * This file contains the SDK implementation for a Tool Data Output of
 * a tool. Only extensible tools currently support having tool data as an
 * output.
 */
#include <GoSdk/Tools/GoExtToolDataOutput.h>
#include <GoSdk/GoSensor.h>

kBeginClassEx(Go, GoExtToolDataOutput)
    kAddVMethod(GoExtToolDataOutput, kObject, VRelease)
kEndClassEx()

GoFx(kStatus) GoExtToolDataOutput_Construct(GoExtToolDataOutput* toolDataOutput, kType type, kObject sensor, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback(allocator);
    kStatus status;

    kCheck(kAlloc_GetObject(alloc, type, toolDataOutput));

    if (!kSuccess(status = GoExtToolDataOutput_Init(*toolDataOutput, type, sensor, alloc)))
    {
        kAlloc_FreeRef(alloc, toolDataOutput);
    }

    return status;
}

GoFx(kStatus) GoExtToolDataOutput_Init(GoExtToolDataOutput toolDataOutput, kType type, kObject sensor, kAlloc alloc)
{
    kObjR(GoExtToolDataOutput, toolDataOutput);

    kCheck(kObject_Init(toolDataOutput, type, alloc));
    obj->type[0] = 0;
    obj->dataType = 0;
    obj->name[0] = 0;
    obj->enabled = kFALSE;

    obj->id = GO_UNASSIGNED_ID;
    obj->sensor = sensor;

    return kOK;
}

GoFx(kStatus) GoExtToolDataOutput_VRelease(GoExtToolDataOutput toolDataOutput)
{
    kCheck(kObject_VRelease(toolDataOutput));

    return kOK;
}

GoFx(kBool) GoExtToolDataOutput_IsValidDataType(GoDataType dataType)
{
    kBool valid = kFALSE;

    switch (dataType)
    {
    case GO_DATA_TYPE_NONE:
    case GO_DATA_TYPE_RANGE:
    case GO_DATA_TYPE_UNIFORM_PROFILE:
    case GO_DATA_TYPE_PROFILE_POINT_CLOUD:
    case GO_DATA_TYPE_UNIFORM_SURFACE:
    case GO_DATA_TYPE_SURFACE_POINT_CLOUD:
    case GO_DATA_TYPE_UNMERGED_PROFILE_POINT_CLOUD:
    case GO_DATA_TYPE_VIDEO:
    case GO_DATA_TYPE_TRACHEID:
    case GO_DATA_TYPE_MESH:
    case GO_DATA_TYPE_FEATURES_ONLY:
        valid = kTRUE;
        break;
    default:
        if (dataType >= GO_DATA_TYPE_GENERIC_BASE && dataType <= GO_DATA_TYPE_GENERIC_END)
            valid = kTRUE;
        break;
    }

    return valid;
}

GoFx(kStatus) GoExtToolDataOutput_Read(GoExtToolDataOutput toolDataOutput, kXml xml, kXmlItem toolDataOutputItem, kBool isGdkUserTool)
{
    kObj(GoExtToolDataOutput, toolDataOutput);

    if (isGdkUserTool)
        kCheckState(kStrEquals(kXml_ItemName(xml, toolDataOutputItem), GO_XML_TAG_NAME_CUSTOM));
    else
        kCheckState(kStrEquals(kXml_ItemName(xml, toolDataOutputItem), GO_XML_TAG_NAME_TOOLDATAOUTPUT));

    kCheck(kXml_Attr32s(xml, toolDataOutputItem, GO_XML_TAG_NAME_ATTR_ID, &obj->id));
    kCheck(kXml_AttrText(xml, toolDataOutputItem, GO_XML_TAG_NAME_ATTR_TYPE, obj->type, kCountOf(obj->type)));
    kCheck(kXml_Attr32s(xml, toolDataOutputItem, GO_XML_TAG_NAME_ATTR_DATA_TYPE, &obj->dataType));
    if (!GoExtToolDataOutput_IsValidDataType(obj->dataType))
    {
        obj->dataType = GO_DATA_TYPE_NONE;
    }
    kCheck(kXml_ChildText(xml, toolDataOutputItem, GO_XML_TAG_NAME_NAME, obj->name, kCountOf(obj->name)));
    kCheck(kXml_ChildBool(xml, toolDataOutputItem, GO_XML_TAG_NAME_ENABLED, &obj->enabled));

    return kOK;
}

GoFx(kStatus) GoExtToolDataOutput_Write(GoExtToolDataOutput toolDataOutput, kXml xml, kXmlItem toolDataOutputItem, kBool isGdkUserTool)
{
    kObj(GoExtToolDataOutput, toolDataOutput);
    kXmlItem temp = kNULL;
    kXmlItem child = kNULL;

    // Put correct name in the tool data output item node.
    if (isGdkUserTool)
        kCheck(kXml_SetItemName(xml, toolDataOutputItem, GO_XML_TAG_NAME_CUSTOM));
    else
        kCheck(kXml_SetItemName(xml, toolDataOutputItem, GO_XML_TAG_NAME_TOOLDATAOUTPUT));

    kCheck(kXml_SetAttr32s(xml, toolDataOutputItem, GO_XML_TAG_NAME_ATTR_ID, obj->id));
    kCheck(kXml_SetAttrText(xml, toolDataOutputItem, GO_XML_TAG_NAME_ATTR_TYPE, obj->type));
    kCheck(kXml_SetAttr32s(xml, toolDataOutputItem, GO_XML_TAG_NAME_ATTR_DATA_TYPE, obj->dataType));

    kCheck(kXml_SetChildText(xml, toolDataOutputItem, GO_XML_TAG_NAME_NAME, obj->name));
    kCheck(kXml_SetChildBool(xml, toolDataOutputItem, GO_XML_TAG_NAME_ENABLED, obj->enabled));

    return kOK;
}


GoFx(kStatus) GoExtToolDataOutput_SetId(GoExtToolDataOutput toolDataOutput, k32s id)
{
    kObj(GoExtToolDataOutput, toolDataOutput);

    obj->id = id;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(k32s) GoExtToolDataOutput_Id(GoExtToolDataOutput toolDataOutput)
{
    kObj(GoExtToolDataOutput, toolDataOutput);

    GoSensor_SyncConfig(obj->sensor);

    return obj->id;
}

GoFx(kStatus) GoExtToolDataOutput_SetName(GoExtToolDataOutput toolDataOutput, const kChar* name)
{
    kObj(GoExtToolDataOutput, toolDataOutput);

    kCheck(kStrCopy(obj->name, kCountOf(obj->name), name));

    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(const kChar*) GoExtToolDataOutput_Name(GoExtToolDataOutput toolDataOutput)
{
    kObj(GoExtToolDataOutput, toolDataOutput);

    GoSensor_SyncConfig(obj->sensor);

    return obj->name;
}

GoFx(kStatus) GoExtToolDataOutput_SetEnable(GoExtToolDataOutput toolDataOutput, kBool enable)
{
    kObj(GoExtToolDataOutput, toolDataOutput);

    obj->enabled = enable;

    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(kBool) GoExtToolDataOutput_Enabled(GoExtToolDataOutput toolDataOutput)
{
    kObj(GoExtToolDataOutput, toolDataOutput);

    GoSensor_SyncConfig(obj->sensor);

    return obj->enabled;
}

GoFx(GoDataType) GoExtToolDataOutput_DataType(GoExtToolDataOutput toolDataOutput)
{
    kObj(GoExtToolDataOutput, toolDataOutput);

    GoSensor_SyncConfig(obj->sensor);

    return obj->dataType;
}

GoFx(const kChar*) GoExtToolDataOutput_Type(GoExtToolDataOutput toolDataOutput)
{
    kObj(GoExtToolDataOutput, toolDataOutput);

    GoSensor_SyncConfig(obj->sensor);

    return obj->type;
}

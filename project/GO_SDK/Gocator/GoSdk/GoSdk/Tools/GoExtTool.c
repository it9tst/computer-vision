/**
 * @file    GoExtTool.c
 *
 * @internal
 * Copyright (C) 2016-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <GoSdk/Tools/GoExtTool.h>
#include <GoSdk/GoUtils.h>
#include <GoSdk/GoSensor.h>

kBeginClassEx(Go, GoExtTool)
    kAddVMethod(GoExtTool, kObject, VRelease)
    kAddVMethod(GoExtTool, GoTool, VInit)
    kAddVMethod(GoExtTool, GoTool, VRead)
    kAddVMethod(GoExtTool, GoTool, VWrite)
kEndClassEx()

GoFx(kStatus) GoExtTool_Construct(GoExtTool* tool, kObject sensor, kAlloc allocator)
{
    return kERROR_UNIMPLEMENTED;
}

GoFx(kStatus) GoExtTool_Init(GoTool tool, kType type, GoToolType typeId, kObject sensor, kAlloc alloc)
{
    kObjR(GoExtTool, tool);

    kCheck(GoTool_Init(tool, type, typeId, sensor, alloc));
    obj->version[0] = 0;
    obj->toolType[0] = 0;
    kZero(obj->streamOptions);
    kZero(obj->stream);
    kZero(obj->sourceOptions);
    kZero(obj->anchoring);
    obj->params = kNULL;
    kZero(obj->toolDataOutputs);

    obj->source = GO_DATA_SOURCE_TOP;

    obj->anchoring.x.anchor = GO_MEASUREMENT_ID_NONE;
    obj->anchoring.y.anchor = GO_MEASUREMENT_ID_NONE;
    obj->anchoring.z.anchor = GO_MEASUREMENT_ID_NONE;
    obj->anchoring.zAngle.anchor = GO_MEASUREMENT_ID_NONE;
    obj->anchoring.x.used = kFALSE;
    obj->anchoring.y.used = kFALSE;
    obj->anchoring.z.used = kFALSE;
    obj->anchoring.zAngle.used = kFALSE;

    kCheck(kArrayList_Construct(&obj->anchoring.x.options, kTypeOf(k32u), 0, alloc));
    kCheck(kArrayList_Construct(&obj->anchoring.y.options, kTypeOf(k32u), 0, alloc));
    kCheck(kArrayList_Construct(&obj->anchoring.z.options, kTypeOf(k32u), 0, alloc));
    kCheck(kArrayList_Construct(&obj->anchoring.zAngle.options, kTypeOf(k32u), 0, alloc));

    //obj->stream.step = GO_DATA_STEP_SURFACE;
    //obj->stream.id = 0;

    kCheck(kArrayList_Construct(&obj->streamOptions, kTypeOf(GoDataStream), 0, alloc));
    kCheck(kArrayList_Construct(&obj->sourceOptions, kTypeOf(GoDataSource), 0, alloc));

    kCheck(GoExtParams_Construct(&obj->params, sensor, alloc));
    kCheck(kArrayList_Construct(&obj->toolDataOutputs, kTypeOf(GoExtToolDataOutput), 0, alloc));

    return kOK;
}

GoFx(kStatus) GoExtTool_VInit(GoExtTool tool, kType type, kObject sensor, kAlloc alloc)
{
    return GoExtTool_Init(tool, type, GO_TOOL_EXTENSIBLE, sensor, alloc);
}

GoFx(kStatus) GoExtTool_VRelease(GoExtTool tool)
{
    kObj(GoExtTool, tool);

    kCheck(kDisposeRef(&obj->streamOptions));
    kCheck(kDisposeRef(&obj->sourceOptions));

    kCheck(kDisposeRef(&obj->anchoring.x.options));
    kCheck(kDisposeRef(&obj->anchoring.y.options));
    kCheck(kDisposeRef(&obj->anchoring.z.options));
    kCheck(kDisposeRef(&obj->anchoring.zAngle.options));

    kCheck(kDisposeRef(&obj->params));
    kCheck(kDisposeRef(&obj->toolDataOutputs));

    return GoTool_VRelease(tool);
}

GoFx(kStatus) GoExtTool_VRead(GoExtTool tool, kXml xml, kXmlItem item)
{
    kObj(GoExtTool, tool);
    kString tempText = kNULL;
    kXmlItem childItem;

    kCheck(GoTool_VRead(tool, xml, item));
    kCheck(GoToolUtil_ParseStreamOptions(tool, xml, item, obj->streamOptions));

    kTry
    {
        kTest(kString_Construct(&tempText, kNULL, kObject_Alloc(tool)));
        if (kXml_AttrExists(xml, item, GO_XML_TAG_NAME_ATTR_VERSION))
        {
            kTest(kXml_AttrText(xml, item, GO_XML_TAG_NAME_ATTR_VERSION, obj->version, kCountOf(obj->version)));
        }
        if (kXml_AttrExists(xml, item, GO_XML_TAG_NAME_ATTR_TYPE))
        {
            kTest(kXml_AttrText(xml, item, GO_XML_TAG_NAME_ATTR_TYPE, obj->toolType, kCountOf(obj->toolType)));
        }

        childItem = kXml_Child(xml, item, GO_XML_TAG_NAME_STREAM);
        if (!kIsNull(childItem))
        {
            kTest(kXml_Child32s(xml, childItem, GO_XML_TAG_NAME_STREAM_STEP, &obj->stream.step));
            kTest(kXml_Child32s(xml, childItem, GO_XML_TAG_NAME_STREAM_ID, &obj->stream.id));
        }

        kTest(!kIsNull(childItem = kXml_Child(xml, item, GO_XML_TAG_NAME_ANCHOR)));
        kTest(GoExtTool_ParseAnchor(xml, childItem, &obj->anchoring));

        kTest(kXml_Child32s(xml, item, GO_XML_TAG_NAME_SOURCE, &obj->source));
        kTest(!kIsNull(childItem = kXml_Child(xml, item, GO_XML_TAG_NAME_SOURCE)));
        kTest(kXml_AttrString(xml, childItem, GO_XML_TAG_NAME_ATTR_OPTIONS, tempText));
        kTest(GoOptionList_ParseList32u(kString_Chars(tempText), obj->sourceOptions));

        // May not exist in non GDK internal tools
        childItem = kXml_Child(xml, item, GO_XML_TAG_NAME_PARAMETERS);
        if (!kIsNull(childItem))
        {
            kTest(GoExtParams_Read(obj->params, xml, childItem));
        }

        // Only extensible tools have tool data outputs section.
        kTest(GoExtTool_ReadToolDataOutputs(tool, xml, item));
    }
    kFinally
    {
        kObject_Dispose(tempText);
        kEndFinally();
    }

    return kOK;
}

GoFx(kStatus) GoExtTool_VWrite(GoExtTool tool, kXml xml, kXmlItem item)
{
    kObj(GoExtTool, tool);
    kXmlItem tempItem;
    kXmlItem streamItem = kNULL;

    // Custom configuration nodes.
    kCheck(kXml_SetAttrText(xml, item, GO_XML_TAG_NAME_ATTR_TYPE, obj->toolType));
    kCheck(kXml_SetAttrText(xml, item, GO_XML_TAG_NAME_ATTR_VERSION, obj->version));
    kCheck(kXml_SetChild32s(xml, item, GO_XML_TAG_NAME_SOURCE, obj->source));

    kCheck(kXml_AddItem(xml, item, GO_XML_TAG_NAME_ANCHOR, &tempItem));
    if (obj->anchoring.x.used)
    {
        kCheck(kXml_SetChild32s(xml, tempItem, GO_XML_TAG_NAME_ANCHOR_X, obj->anchoring.x.anchor));
    }
    if (obj->anchoring.y.used)
    {
        kCheck(kXml_SetChild32s(xml, tempItem, GO_XML_TAG_NAME_ANCHOR_Y, obj->anchoring.y.anchor));
    }
    if (obj->anchoring.z.used)
    {
        kCheck(kXml_SetChild32s(xml, tempItem, GO_XML_TAG_NAME_ANCHOR_Z, obj->anchoring.z.anchor));
    }
    if (obj->anchoring.zAngle.used)
    {
        kCheck(kXml_SetChild32s(xml, tempItem, GO_XML_TAG_NAME_ANCHOR_ZANGLE, obj->anchoring.zAngle.anchor));
    }

    kCheck(kXml_AddItem(xml, item, GO_XML_TAG_NAME_PARAMETERS, &tempItem));
    kCheck(GoExtParams_Write(obj->params, xml, tempItem));

    kCheck(kXml_AddItem(xml, item, GO_XML_TAG_NAME_STREAM, &streamItem));
    kCheck(kXml_SetChild32s(xml, streamItem, GO_XML_TAG_NAME_STREAM_STEP, obj->stream.step));
    kCheck(kXml_SetChild32s(xml, streamItem, GO_XML_TAG_NAME_STREAM_ID, obj->stream.id));

    kCheck(GoExtTool_WriteToolDataOutputs(tool, xml, item));

    return GoTool_VWrite(tool, xml, item);
}

GoFx(kStatus) GoExtTool_ReadToolDataOutputs(GoExtTool tool, kXml xml, kXmlItem item)
{
    kObj(GoExtTool, tool);
    kXmlItem            toolDataListItem;
    kXmlItem            toolDataItem;
    kSize               itemCount = 0;
    kSize               i;
    GoExtToolDataOutput toolDataOutput;
    kBool               insertOutput;
    kStatus             exception;

    toolDataListItem = kXml_Child(xml, item, GO_XML_TAG_NAME_TOOL_DATA_OUTPUTS);
    if (!kIsNull(toolDataListItem))
    {
        itemCount = kXml_ChildCount(xml, toolDataListItem);
        for (i = 0; i < itemCount; i++)
        {
            insertOutput = kFALSE;
            toolDataItem = kXml_ChildAt(xml, toolDataListItem, i);

            kTry
            {
                if ((i >= kArrayList_Count(obj->toolDataOutputs)) ||
                (kIsNull(toolDataOutput = kArrayList_AsT(obj->toolDataOutputs, i, GoExtToolDataOutput))))
                {
                    // No existing entry to overwrite, so allocate new entry.
                    kTest(GoExtToolDataOutput_Construct(&toolDataOutput, kTypeOf(GoExtToolDataOutput), obj->base.sensor, kObject_Alloc(tool)));
                    insertOutput = kTRUE;
                }

                kTest(GoExtToolDataOutput_Read(toolDataOutput, xml, toolDataItem, obj->base.format == GO_TOOL_FORMAT_GDK_USER));

                if (insertOutput)
                {
                    if (i >= kArrayList_Count(obj->toolDataOutputs))
                    {
                        kTest(kArrayList_AddT(obj->toolDataOutputs, &toolDataOutput));
                    }
                    else
                    {
                        kTest(kArrayList_InsertT(obj->toolDataOutputs, i, &toolDataOutput));
                    }
                }
            }
            kCatch(&exception)
            {
                if (insertOutput)
                {
                    // An object was allocated
                    kDisposeRef(&toolDataOutput);
                }
                kEndCatch(exception);
            }
        }
    }

    // If no exceptions raised, remove extraneous entries from the list
    // so that correct information is sent back to the sensor.
    // If exception occurs, leave the list as-is.
    while (itemCount < kArrayList_Count(obj->toolDataOutputs))
    {
        kCheck(kArrayList_RemoveT(obj->toolDataOutputs, kArrayList_Count(obj->toolDataOutputs) - 1, &toolDataOutput));
        kDisposeRef(&toolDataOutput);
    }

    return kOK;
}

GoFx(kStatus) GoExtTool_WriteToolDataOutputs(GoExtTool tool, kXml xml, kXmlItem item)
{
    kObj(GoExtTool, tool);
    kXmlItem            toolDataListItem;
    kXmlItem            toolDataItem;
    kSize               i;
    GoExtToolDataOutput toolDataOutput;

    kCheck(kXml_AddItem(xml, item, GO_XML_TAG_NAME_TOOL_DATA_OUTPUTS, &toolDataListItem));

    for (i = 0; i < kArrayList_Count(obj->toolDataOutputs); i++)
    {
        toolDataOutput = *kArrayList_AtT(obj->toolDataOutputs, i, GoExtToolDataOutput);

        // Add a place holder for child node, and give it a dummy name so that
        // the node can be added to the XML structure.
        // The real name will be filled in by the class code implementation.
        kCheck(kXml_AddItem(xml, toolDataListItem, GO_XML_TAG_NAME_TOOL_DATA_OUTPUTS, &toolDataItem));

        kCheck(GoExtToolDataOutput_Write(toolDataOutput, xml, toolDataItem, obj->base.format == GO_TOOL_FORMAT_GDK_USER));
    }

    return kOK;
}

GoFx(kSize) GoExtTool_StreamOptionCount(GoExtTool tool)
{
    kObj(GoExtTool, tool);

    GoSensor_SyncConfig(obj->base.sensor);

    return kArrayList_Count(obj->streamOptions);
}

GoFx(GoDataStream) GoExtTool_StreamOptionAt(GoExtTool tool, kSize index)
{
    kObj(GoExtTool, tool);

    GoSensor_SyncConfig(obj->base.sensor);

    kAssert(index < kArrayList_Count(obj->streamOptions));

    return kArrayList_AsT(obj->streamOptions, index, GoDataStream);
}

GoFx(kStatus) GoExtTool_SetStream(GoExtTool tool, GoDataStream stream)
{
    kObj(GoExtTool, tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    kCheck(GoSensor_SyncConfig(GoTool_Sensor(tool)));

    if (GoTools_IsToolInOptionsList(GoSensor_Tools(GoTool_Sensor(tool)), kObject_Type(tool)))
    {
        kCheck(GoOptionList_CheckDataStream(kArrayList_DataT(obj->streamOptions, GoDataStream), kArrayList_Count(obj->streamOptions), stream));
    }

    obj->stream = stream;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(GoDataStream) GoExtTool_Stream(GoExtTool tool)
{
    kObj(GoExtTool, tool);

    GoSensor_SyncConfig(obj->base.sensor);

    return obj->stream;
}

GoFx(kStatus) GoExtTool_SetDisplayName(GoExtTool tool, const kChar* value)
{
    kObj(GoExtTool, tool);

    return GoTool_SetName(tool, value);
}

GoFx(const kChar*) GoExtTool_DisplayName(GoExtTool tool)
{
    kObj(GoExtTool, tool);

    GoSensor_SyncConfig(obj->base.sensor);

    return obj->base.name;
}

GoFx(const kChar*) GoExtTool_Type(GoExtTool tool)
{
    kObj(GoExtTool, tool);

    GoSensor_SyncConfig(obj->base.sensor);


    return obj->toolType;
}

GoFx(kBool) GoExtTool_XAnchorSupportEnabled(GoExtTool tool)
{
    kObj(GoExtTool, tool);

    GoSensor_SyncConfig(obj->base.sensor);

    return obj->anchoring.x.used;
}

GoFx(kBool) GoExtTool_YAnchorSupportEnabled(GoExtTool tool)
{
    kObj(GoExtTool, tool);

    GoSensor_SyncConfig(obj->base.sensor);

    return obj->anchoring.y.used;
}

GoFx(kBool) GoExtTool_ZAnchorSupportEnabled(GoExtTool tool)
{
    kObj(GoExtTool, tool);

    GoSensor_SyncConfig(obj->base.sensor);

    return obj->anchoring.z.used;
}

GoFx(kBool) GoExtTool_ZAngleAnchorSupportEnabled(GoExtTool tool)
{
    kObj(GoExtTool, tool);

    GoSensor_SyncConfig(obj->base.sensor);

    return obj->anchoring.zAngle.used;
}

GoFx(kSize) GoExtTool_MeasurementCount(GoExtTool tool)
{
    kObj(GoExtTool, tool);

    GoSensor_SyncConfig(obj->base.sensor);

    return kArrayList_Count(obj->base.measurements);
}

GoFx(GoMeasurement) GoExtTool_MeasurementAt(GoExtTool tool, kSize index)
{
    kObj(GoExtTool, tool);

    GoSensor_SyncConfig(obj->base.sensor);

    if (index > kArrayList_Count(obj->base.measurements))
    {
        return kNULL;
    }

    return *kArrayList_AtT(obj->base.measurements, index, GoMeasurement);
}

GoFx(GoMeasurement) GoExtTool_FindMeasurementByName(GoExtTool tool, const kChar* name)
{
    kObj(GoExtTool, tool);
    kSize i;

    GoSensor_SyncConfig(obj->base.sensor);

    for (i = 0; i < kArrayList_Count(obj->base.measurements); i++)
    {
        GoMeasurement extMeasurement = *kArrayList_AtT(obj->base.measurements, i, GoMeasurement);
        const kChar* name2 = GoMeasurement_Name(extMeasurement);
        //not quite right - this checks the measurement name, but not the extensible tool's measurement type name identifier
        if (strcmp(name,name2) == 0)
        {
            return extMeasurement;
        }
    }

    return kNULL;
}

GoFx(kStatus) GoExtTool_SetSource(GoExtTool tool, GoDataSource source)
{
    kObj(GoExtTool, tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    kCheck(GoSensor_SyncConfig(GoTool_Sensor(tool)));
    kCheck(GoOptionList_Check32u((k32u*)kArrayList_DataT(obj->sourceOptions, GoDataSource), kArrayList_Count(obj->sourceOptions), source));
    obj->source = source;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(kSize) GoExtTool_SourceOptionCount(GoExtTool tool)
{
    kObj(GoExtTool, tool);

    GoSensor_SyncConfig(obj->base.sensor);

    return kArrayList_Count(obj->sourceOptions);
}

GoFx(k32u) GoExtTool_SourceOptionAt(GoExtTool tool, kSize index)
{
    kObj(GoExtTool, tool);

    GoSensor_SyncConfig(obj->base.sensor);

    kAssert(index < kArrayList_Count(obj->sourceOptions));

    return kArrayList_AsT(obj->sourceOptions, index, k32u);
}

GoFx(GoDataSource) GoExtTool_Source(GoExtTool tool)
{
    kObj(GoExtTool, tool);

    GoSensor_SyncConfig(obj->base.sensor);

    return obj->source;
}

GoFx(kSize) GoExtTool_XAnchorOptionCount(GoExtTool tool)
{
    kObj(GoExtTool, tool);

    GoSensor_SyncConfig(obj->base.sensor);

    return kArrayList_Count(obj->anchoring.x.options);
}

GoFx(k32u) GoExtTool_XAnchorOptionAt(GoExtTool tool, kSize index)
{
    kObj(GoExtTool, tool);

    GoSensor_SyncConfig(obj->base.sensor);

    kAssert(index < kArrayList_Count(obj->anchoring.x.options));

    return kArrayList_AsT(obj->anchoring.x.options, index, k32u);
}

GoFx(k32s) GoExtTool_XAnchor(GoExtTool tool)
{
    kObj(GoExtTool, tool);

    GoSensor_SyncConfig(obj->base.sensor);

    return obj->anchoring.x.anchor;
}

GoFx(kStatus) GoExtTool_SetXAnchor(GoExtTool tool, k32s id)
{
    kObj(GoExtTool, tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    kCheck(GoSensor_SyncConfig(GoTool_Sensor(tool)));

    if (id != GO_MEASUREMENT_ID_NONE)
    {
        obj->anchoring.x.used = kTRUE;
        kCheck(GoOptionList_Check32u(kArrayList_DataT(obj->anchoring.x.options, k32u), kArrayList_Count(obj->anchoring.x.options), id));
    }

    obj->anchoring.x.anchor = id;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(kBool) GoExtTool_XAnchorEnabled(GoExtTool tool)
{
    kObj(GoExtTool, tool);

    GoSensor_SyncConfig(obj->base.sensor);

    if (obj->anchoring.x.anchor != GO_MEASUREMENT_ID_NONE)
    {
        return kTRUE;
    }

    return kFALSE;
}

GoFx(kSize) GoExtTool_YAnchorOptionCount(GoExtTool tool)
{
    kObj(GoExtTool, tool);

    GoSensor_SyncConfig(obj->base.sensor);

    return kArrayList_Count(obj->anchoring.y.options);
}

GoFx(k32u) GoExtTool_YAnchorOptionAt(GoExtTool tool, kSize index)
{
    kObj(GoExtTool, tool);

    GoSensor_SyncConfig(obj->base.sensor);

    kAssert(index < kArrayList_Count(obj->anchoring.y.options));

    return kArrayList_AsT(obj->anchoring.y.options, index, k32u);
}

GoFx(k32s) GoExtTool_YAnchor(GoExtTool tool)
{
    kObj(GoExtTool, tool);

    GoSensor_SyncConfig(obj->base.sensor);

    return obj->anchoring.y.anchor;
}

GoFx(kStatus) GoExtTool_SetYAnchor(GoExtTool tool, k32s id)
{
    kObj(GoExtTool, tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    kCheck(GoSensor_SyncConfig(GoTool_Sensor(tool)));

    if (id != GO_MEASUREMENT_ID_NONE)
    {
        obj->anchoring.y.used = kTRUE;
        kCheck(GoOptionList_Check32u(kArrayList_DataT(obj->anchoring.y.options, k32u), kArrayList_Count(obj->anchoring.y.options), id));
    }

    obj->anchoring.y.anchor = id;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(kBool) GoExtTool_YAnchorEnabled(GoExtTool tool)
{
    kObj(GoExtTool, tool);

    GoSensor_SyncConfig(obj->base.sensor);

    if (obj->anchoring.y.anchor != GO_MEASUREMENT_ID_NONE)
    {
        return kTRUE;
    }

    return kFALSE;
}

GoFx(kSize) GoExtTool_ZAnchorOptionCount(GoExtTool tool)
{
    kObj(GoExtTool, tool);

    GoSensor_SyncConfig(obj->base.sensor);

    return kArrayList_Count(obj->anchoring.z.options);
}

GoFx(k32u) GoExtTool_ZAnchorOptionAt(GoExtTool tool, kSize index)
{
    kObj(GoExtTool, tool);

    GoSensor_SyncConfig(obj->base.sensor);

    kAssert(index < kArrayList_Count(obj->anchoring.z.options));

    return kArrayList_AsT(obj->anchoring.z.options, index, k32u);
}

GoFx(kStatus) GoExtTool_SetZAnchor(GoExtTool tool, k32s id)
{
    kObj(GoExtTool, tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    kCheck(GoSensor_SyncConfig(GoTool_Sensor(tool)));

    if (id != GO_MEASUREMENT_ID_NONE)
    {
        obj->anchoring.z.used = kTRUE;
        kCheck(GoOptionList_Check32u(kArrayList_DataT(obj->anchoring.z.options, k32u), kArrayList_Count(obj->anchoring.z.options), id));
    }

    obj->anchoring.z.anchor = id;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}


GoFx(kBool) GoExtTool_ZAnchorEnabled(GoExtTool tool)
{
    kObj(GoExtTool, tool);

    GoSensor_SyncConfig(obj->base.sensor);

    if (obj->anchoring.z.anchor != GO_MEASUREMENT_ID_NONE)
    {
        return kTRUE;
    }

    return kFALSE;
}

GoFx(k32s) GoExtTool_ZAnchor(GoExtTool tool)
{
    kObj(GoExtTool, tool);

    GoSensor_SyncConfig(obj->base.sensor);

    return obj->anchoring.z.anchor;
}

GoFx(kSize) GoExtTool_ZAngleAnchorOptionCount(GoExtTool tool)
{
    kObj(GoExtTool, tool);

    GoSensor_SyncConfig(obj->base.sensor);

    return kArrayList_Count(obj->anchoring.zAngle.options);
}

GoFx(k32u) GoExtTool_ZAngleAnchorOptionAt(GoExtTool tool, kSize index)
{
    kObj(GoExtTool, tool);

    GoSensor_SyncConfig(obj->base.sensor);

    kAssert(index < kArrayList_Count(obj->anchoring.zAngle.options));

    return kArrayList_AsT(obj->anchoring.zAngle.options, index, k32u);
}

GoFx(kStatus) GoExtTool_SetZAngleAnchor(GoExtTool tool, k32s id)
{
    kObj(GoExtTool, tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    kCheck(GoSensor_SyncConfig(GoTool_Sensor(tool)));

    if (id != GO_MEASUREMENT_ID_NONE)
    {
        obj->anchoring.zAngle.used = kTRUE;
        kCheck(GoOptionList_Check32u(kArrayList_DataT(obj->anchoring.zAngle.options, k32u), kArrayList_Count(obj->anchoring.zAngle.options), id));
    }

    obj->anchoring.zAngle.anchor = id;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}


GoFx(kBool) GoExtTool_ZAngleAnchorEnabled(GoExtTool tool)
{
    kObj(GoExtTool, tool);

    GoSensor_SyncConfig(obj->base.sensor);

    if (obj->anchoring.zAngle.anchor != GO_MEASUREMENT_ID_NONE)
    {
        return kTRUE;
    }

    return kFALSE;
}

GoFx(k32s) GoExtTool_ZAngleAnchor(GoExtTool tool)
{
    kObj(GoExtTool, tool);

    GoSensor_SyncConfig(obj->base.sensor);

    return obj->anchoring.zAngle.anchor;
}

GoFx(kSize) GoExtTool_ParameterCount(GoExtTool tool)
{
    kObj(GoExtTool, tool);

    GoSensor_SyncConfig(obj->base.sensor);

    return GoExtParams_ParameterCount(obj->params);
}

GoFx(GoExtParam) GoExtTool_ParameterAt(GoExtTool tool, kSize index)
{
    kObj(GoExtTool, tool);

    GoSensor_SyncConfig(obj->base.sensor);

    return GoExtParams_ParameterAt(obj->params, index);
}

GoFx(GoExtParam) GoExtTool_FindParameterById(GoExtTool tool, const kChar* id)
{
    kObj(GoExtTool, tool);

    GoSensor_SyncConfig(obj->base.sensor);

    return GoExtParams_FindParameterById(obj->params, id);
}

GoFx(const kChar*) GoExtTool_Version(GoExtTool tool)
{
    kObj(GoExtTool, tool);

    GoSensor_SyncConfig(obj->base.sensor);

    return obj->version;
}

GoFx(kStatus) GoExtTool_ParseAnchor(kXml xml, kXmlItem anchorItem, GoExtToolAnchoringCfg* anchoring)
{
    kXmlItem tempItem = kNULL;
    kString tempText = kNULL;

    kTry
    {
        kTest(kString_Construct(&tempText, kNULL, kObject_Alloc(anchoring->x.options)));

        if (kXml_ChildExists(xml, anchorItem, GO_XML_TAG_NAME_ANCHOR_X))
        {
            kTestArgs(!kIsNull(tempItem = kXml_Child(xml, anchorItem, GO_XML_TAG_NAME_ANCHOR_X)));
            kTest(kXml_Item32s(xml, tempItem, &anchoring->x.anchor));

            if (kXml_AttrExists(xml, tempItem, GO_XML_TAG_NAME_ATTR_USED))
            {
                kTest(kXml_AttrBool(xml, tempItem, GO_XML_TAG_NAME_ATTR_USED, &anchoring->x.used));
            }
            else
            {
                anchoring->x.used = kTRUE;
            }
            kTest(kXml_AttrString(xml, tempItem, GO_XML_TAG_NAME_ATTR_OPTIONS, tempText));
            kTest(GoOptionList_ParseList32u(kString_Chars(tempText), anchoring->x.options));
        }
        else
        {
            anchoring->x.used = kFALSE;
        }
        
        if (kXml_ChildExists(xml, anchorItem, GO_XML_TAG_NAME_ANCHOR_Y))
        {
            kTestArgs(!kIsNull(tempItem = kXml_Child(xml, anchorItem, GO_XML_TAG_NAME_ANCHOR_Y)));
            kTest(kXml_Item32s(xml, tempItem, &anchoring->y.anchor));
            if (kXml_AttrExists(xml, tempItem, GO_XML_TAG_NAME_ATTR_USED))
            {
                kTest(kXml_AttrBool(xml, tempItem, GO_XML_TAG_NAME_ATTR_USED, &anchoring->y.used));
            }
            else
            {
                anchoring->y.used = kTRUE;
            }
            kTest(kXml_AttrString(xml, tempItem, GO_XML_TAG_NAME_ATTR_OPTIONS, tempText));
            kTest(GoOptionList_ParseList32u(kString_Chars(tempText), anchoring->y.options));
        }
        else
        {
            anchoring->y.used = kFALSE;
        }

        if (kXml_ChildExists(xml, anchorItem, GO_XML_TAG_NAME_ANCHOR_Z))
        {
            kTestArgs(!kIsNull(tempItem = kXml_Child(xml, anchorItem, GO_XML_TAG_NAME_ANCHOR_Z)));
            kTest(kXml_Item32s(xml, tempItem, &anchoring->z.anchor));
            if (kXml_AttrExists(xml, tempItem, GO_XML_TAG_NAME_ATTR_USED))
            {
                kTest(kXml_AttrBool(xml, tempItem, GO_XML_TAG_NAME_ATTR_USED, &anchoring->z.used));
            }
            else
            {
                anchoring->z.used = kTRUE;
            }
            kTest(kXml_AttrString(xml, tempItem, GO_XML_TAG_NAME_ATTR_OPTIONS, tempText));
            kTest(GoOptionList_ParseList32u(kString_Chars(tempText), anchoring->z.options));
        }
        else
        {
            anchoring->z.used = kFALSE;
        }

        if (kXml_ChildExists(xml, anchorItem, GO_XML_TAG_NAME_ANCHOR_ZANGLE))
        {
            kTestArgs(!kIsNull(tempItem = kXml_Child(xml, anchorItem, GO_XML_TAG_NAME_ANCHOR_ZANGLE)));
            kTest(kXml_Item32s(xml, tempItem, &anchoring->zAngle.anchor));
            if (kXml_AttrExists(xml, tempItem, GO_XML_TAG_NAME_ATTR_USED))
            {
                kTest(kXml_AttrBool(xml, tempItem, GO_XML_TAG_NAME_ATTR_USED, &anchoring->zAngle.used));
            }
            else
            {
                anchoring->zAngle.used = kTRUE;
            }
            kTest(kXml_AttrString(xml, tempItem, GO_XML_TAG_NAME_ATTR_OPTIONS, tempText));
            kTest(GoOptionList_ParseList32u(kString_Chars(tempText), anchoring->zAngle.options));
        }
        else
        {
            anchoring->zAngle.used = kFALSE;
        }
    }
    kFinally
    {
        kObject_Dispose(tempText);
        kEndFinally();
    }

    return kOK;
}

GoFx(kSize) GoExtTool_ToolDataOutputCount(GoExtTool tool)
{
    kObj(GoExtTool, tool);

    GoSensor_SyncConfig(obj->base.sensor);

    return kArrayList_Count(obj->toolDataOutputs);
}

GoFx(GoExtToolDataOutput) GoExtTool_ToolDataOutputAt(GoExtTool tool, kSize index)
{
    kObj(GoExtTool, tool);

    GoSensor_SyncConfig(obj->base.sensor);

    kAssert(index < kArrayList_Count(obj->toolDataOutputs));
    if (index >= kArrayList_Count(obj->toolDataOutputs))
    {
        return kNULL;
    }

    return *kArrayList_AtT(obj->toolDataOutputs, index, GoExtToolDataOutput);
}

GoFx(kBool) GoExtTool_Equals(GoExtTool tool, kString toolType)
{
    kObj(GoExtTool, tool);

    if (kIsNull(toolType) || 0 >= kString_Length(toolType))
    {
        return kFALSE;
    }
    else if (0 < kStrLength(obj->toolType))
    {
        return kString_Equals(toolType, obj->toolType);
    }
    
    return kString_Equals(toolType, kType_Name(kObject_Type(tool)));
}

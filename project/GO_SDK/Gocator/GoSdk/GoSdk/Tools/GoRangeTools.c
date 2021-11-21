#include <GoSdk/Tools/GoRangeTools.h>
#include <GoSdk/GoSensor.h>

kBeginClassEx(Go, GoRangeTool)
kAddVMethod(GoRangeTool, kObject, VRelease)
kEndClassEx()

GoFx(kStatus) GoRangeTool_Init(GoRangeTool tool, kType type, GoToolType typeId, kObject sensor, kAlloc alloc)
{
    kObjR(GoRangeTool, tool);

    kCheck(GoTool_Init(tool, type, typeId, sensor, alloc));
    kZero(obj->streamOptions);
    kZero(obj->stream);
    kZero(obj->sourceOptions);
    kZero(obj->zAnchorOptions);

    obj->source = GO_DATA_SOURCE_TOP;
    obj->zAnchor = -1;
    obj->stream.step = GO_DATA_STEP_RANGE;
    obj->stream.id = 0;

    kCheck(kArrayList_Construct(&obj->streamOptions, kTypeOf(GoDataStream), 0, alloc));
    kCheck(kArrayList_Construct(&obj->sourceOptions, kTypeOf(GoDataSource), 0, alloc));
    kCheck(kArrayList_Construct(&obj->zAnchorOptions, kTypeOf(k32u), 0, alloc));

    return kOK;
}

GoFx(kStatus) GoRangeTool_VRelease(GoRangeTool tool)
{
    kObj(GoRangeTool, tool);

    kCheck(kDisposeRef(&obj->streamOptions));
    kCheck(kDisposeRef(&obj->zAnchorOptions));
    kCheck(kDisposeRef(&obj->sourceOptions));
    kCheck(GoTool_VRelease(tool));

    return kOK;
}

GoFx(kStatus) GoRangeTool_Read(GoRangeTool tool, kXml xml, kXmlItem item)
{
    kObj(GoRangeTool, tool);
    kString text = kNULL;
    kXmlItem tempItem = kNULL;
    kXmlItem streamItem;

    kCheck(GoTool_VRead(tool, xml, item));
    kCheck(GoToolUtil_ParseStreamOptions(tool, xml, item, obj->streamOptions));

    kTry
    {
        kTest(kString_Construct(&text, kNULL, kObject_Alloc(tool)));

        if (kXml_ChildExists(xml, item, "Stream"))
        {
            streamItem = kXml_Child(xml, item, "Stream");

            kTest(kXml_Child32s(xml, streamItem, "Step", &obj->stream.step));
            kTest(kXml_Child32s(xml, streamItem, "Id", &obj->stream.id));
        }

        kTest(kXml_Child32s(xml, item, "Source", &obj->source));
        kTest(!kIsNull(tempItem = kXml_Child(xml, item, "Source")));
        kTest(kXml_AttrString(xml, tempItem, "options", text));
        kTest(GoOptionList_ParseList32u(kString_Chars(text), obj->sourceOptions));

        kTest(!kIsNull(tempItem = kXml_Child(xml, item, "Anchor")));
        kTest(kXml_Child32s(xml, tempItem, "Z", &obj->zAnchor));
        kTest(!kIsNull(tempItem = kXml_Child(xml, tempItem, "Z")));
        kTest(kXml_AttrString(xml, tempItem, "options", text));
        kTest(GoOptionList_ParseList32u(kString_Chars(text), obj->zAnchorOptions));
    }
    kFinally
    {
        kObject_Dispose(text);
        kEndFinally();
    }

    return kOK;
}

GoFx(kStatus) GoRangeTool_Write(GoRangeTool tool, kXml xml, kXmlItem item)
{
    kObj(GoRangeTool, tool);
    kXmlItem streamItem = kNULL;
    kXmlItem anchorItem = kNULL;

    kCheck(kXml_AddItem(xml, item, "Stream", &streamItem));
    kCheck(kXml_SetChild32s(xml, streamItem, "Step", obj->stream.step));
    kCheck(kXml_SetChild32s(xml, streamItem, "Id", obj->stream.id));

    kCheck(kXml_SetChild32s(xml, item, "Source", obj->source));

    kCheck(kXml_AddItem(xml, item, "Anchor", &anchorItem));
    kCheck(kXml_SetChild32s(xml, anchorItem, "Z", obj->zAnchor));

    kCheck(GoTool_VWrite(tool, xml, item));

    return kOK;
}

GoFx(kSize) GoRangeTool_StreamOptionCount(GoRangeTool tool)
{
    kObj(GoRangeTool, tool);

    GoSensor_SyncConfig(obj->base.sensor);

    return kArrayList_Count(obj->streamOptions);
}

GoFx(GoDataStream) GoRangeTool_StreamOptionAt(GoRangeTool tool, kSize index)
{
    kObj(GoRangeTool, tool);

    GoSensor_SyncConfig(obj->base.sensor);

    kAssert(index < kArrayList_Count(obj->streamOptions));

    return kArrayList_AsT(obj->streamOptions, index, GoDataStream);
}

GoFx(kStatus) GoRangeTool_SetStream(GoRangeTool tool, GoDataStream stream)
{
    kObj(GoRangeTool, tool);

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

GoFx(GoDataStream) GoRangeTool_Stream(GoRangeTool tool)
{
    kObj(GoRangeTool, tool);

    GoSensor_SyncConfig(obj->base.sensor);

    return obj->stream;
}

GoFx(kSize) GoRangeTool_SourceOptionCount(GoRangeTool tool)
{
    kObj(GoRangeTool, tool);

    GoSensor_SyncConfig(obj->base.sensor);

    return kArrayList_Count(obj->sourceOptions);
}

GoFx(GoDataSource) GoRangeTool_SourceOptionAt(GoRangeTool tool, kSize index)
{
    kObj(GoRangeTool, tool);

    GoSensor_SyncConfig(obj->base.sensor);

    kAssert(index < kArrayList_Count(obj->sourceOptions));

    return kArrayList_AsT(obj->sourceOptions, index, GoDataSource);
}

GoFx(kStatus) GoRangeTool_SetSource(GoRangeTool tool, GoDataSource source)
{
    kObj(GoRangeTool, tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    kCheck(GoSensor_SyncConfig(GoTool_Sensor(tool)));

    if (GoTools_IsToolInOptionsList(GoSensor_Tools(GoTool_Sensor(tool)), kObject_Type(tool)))
    {
        kCheck(GoOptionList_Check32u((k32u*)kArrayList_DataT(obj->sourceOptions, GoDataSource), kArrayList_Count(obj->sourceOptions), source));
    }

    obj->source = source;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(GoDataSource) GoRangeTool_Source(GoRangeTool tool)
{
    kObj(GoRangeTool, tool);

    GoSensor_SyncConfig(obj->base.sensor);

    return obj->source;
}

kBeginClassEx(Go, GoRangePosition)
kAddVMethod(GoRangePosition, kObject, VRelease)
kAddVMethod(GoRangePosition, GoTool, VInit)
kAddVMethod(GoRangePosition, GoTool, VRead)
kAddVMethod(GoRangePosition, GoTool, VWrite)
kEndClassEx()

GoFx(kStatus) GoRangePosition_Construct(GoRangePosition* tool, kObject sensor, kAlloc allocator)
{
    return GoTool_Construct(tool, kTypeOf(GoRangePosition), sensor, allocator);
}

GoFx(kStatus) GoRangePosition_VInit(GoRangePosition tool, kType type, kObject sensor, kAlloc alloc)
{
    kCheck(GoRangeTool_Init(tool, type, GO_TOOL_RANGE_POSITION, sensor, alloc));

    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoRangePositionZ), kTRUE, kNULL));

    return kOK;
}

GoFx(kStatus) GoRangePosition_VRelease(GoRangePosition tool)
{
    return GoRangeTool_VRelease(tool);
}

GoFx(kStatus) GoRangePosition_VRead(GoRangePosition tool, kXml xml, kXmlItem item)
{
    kCheck(GoRangeTool_Read(tool, xml, item));

    return kOK;
}

GoFx(kStatus) GoRangePosition_VWrite(GoRangePosition tool, kXml xml, kXmlItem item)
{
    kCheck(GoRangeTool_Write(tool, xml, item));

    return kOK;
}

GoFx(GoRangePositionZ) GoRangePosition_ZMeasurement(GoRangePosition tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_RANGE_POSITION_Z);
}


kBeginClassEx(Go, GoRangeThickness)
kAddVMethod(GoRangeThickness, kObject, VRelease)
kAddVMethod(GoRangeThickness, GoTool, VInit)
kAddVMethod(GoRangeThickness, GoTool, VRead)
kAddVMethod(GoRangeThickness, GoTool, VWrite)
kEndClassEx()

GoFx(kStatus) GoRangeThickness_Construct(GoRangeThickness* tool, kObject sensor, kAlloc allocator)
{
    return GoTool_Construct(tool, kTypeOf(GoRangeThickness), sensor, allocator);
}

GoFx(kStatus) GoRangeThickness_VInit(GoRangeThickness tool, kType type, kObject sensor, kAlloc alloc)
{
    kObjR(GoRangeThickness, tool);

    kCheck(GoRangeTool_Init(tool, type, GO_TOOL_RANGE_THICKNESS, sensor, alloc));
    obj->absoluteEnabled = kFALSE;

    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoRangeThicknessThickness), kTRUE, kNULL));

    return kOK;
}

GoFx(kStatus) GoRangeThickness_VRelease(GoRangeThickness tool)
{
    return GoRangeTool_VRelease(tool);
}

GoFx(kStatus) GoRangeThickness_VRead(GoRangeThickness tool, kXml xml, kXmlItem item)
{
    kObj(GoRangeThickness, tool);

    kCheck(kXml_ChildBool(xml, item, "Absolute", &obj->absoluteEnabled));
    kCheck(GoRangeTool_Read(tool, xml, item));

    return kOK;
}

GoFx(kStatus) GoRangeThickness_VWrite(GoRangeThickness tool, kXml xml, kXmlItem item)
{
    kObj(GoRangeThickness, tool);

    kCheck(kXml_SetChildBool(xml, item, "Absolute", obj->absoluteEnabled));
    kCheck(GoRangeTool_Write(tool, xml, item));

    return kOK;
}

GoFx(kBool) GoRangeThickness_AbsoluteEnabled(GoRangeThickness tool)
{
    kObj(GoRangeThickness, tool);

    return obj->absoluteEnabled;
}

GoFx(kStatus) GoRangeThickness_EnableAbsolute(GoRangeThickness tool, kBool enable)
{
    kObj(GoRangeThickness, tool);

    obj->absoluteEnabled = enable;

    return kOK;
}

GoFx(GoRangeThickness) GoRangeThickness_ThicknessMeasurement(GoRangeThickness tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_RANGE_THICKNESS_THICKNESS);
}

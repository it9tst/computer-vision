/**
 * @file    GoSurfaceTools.c
 *
 * @internal
 * Copyright (C) 2016-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <GoSdk/Tools/GoSurfaceTools.h>
#include <GoSdk/Tools/GoExtTool.h>
#include <GoSdk/GoSensor.h>

kBeginClassEx(Go, GoSurfaceTool)
kAddVMethod(GoSurfaceTool, kObject, VRelease)
kEndClassEx()

GoFx(kStatus) GoSurfaceTool_Init(GoSurfaceTool tool, kType type, GoToolType typeId, kObject sensor, kAlloc alloc)
{
    kObjR(GoSurfaceTool, tool);

    kCheck(GoTool_Init(tool, type, typeId, sensor, alloc));

    obj->streamOptions = kNULL;
    obj->sourceOptions = kNULL;
    obj->xAnchorOptions = kNULL;
    obj->yAnchorOptions = kNULL;
    obj->zAnchorOptions = kNULL;
    obj->zAngleAnchorOptions = kNULL;

    obj->source = GO_DATA_SOURCE_TOP; 
    obj->stream.step = GO_DATA_STEP_SURFACE;
    obj->stream.id = 0;
    obj->xAnchor = GO_MEASUREMENT_ID_NONE;
    obj->yAnchor = GO_MEASUREMENT_ID_NONE;
    obj->zAnchor = GO_MEASUREMENT_ID_NONE;
    obj->zAngleAnchor = GO_MEASUREMENT_ID_NONE;

    kCheck(kArrayList_Construct(&obj->streamOptions, kTypeOf(GoDataStream), 0, alloc)); 
    kCheck(kArrayList_Construct(&obj->sourceOptions, kTypeOf(GoDataSource), 0, alloc)); 

    kCheck(kArrayList_Construct(&obj->xAnchorOptions, kTypeOf(k32u), 0, alloc));
    kCheck(kArrayList_Construct(&obj->yAnchorOptions, kTypeOf(k32u), 0, alloc));
    kCheck(kArrayList_Construct(&obj->zAnchorOptions, kTypeOf(k32u), 0, alloc));
    kCheck(kArrayList_Construct(&obj->zAngleAnchorOptions, kTypeOf(k32u), 0, alloc));


    return kOK;
}

GoFx(kStatus) GoSurfaceTool_VRelease(GoSurfaceTool tool)
{
    kObj(GoSurfaceTool, tool);

    kCheck(kDisposeRef(&obj->streamOptions)); 
    kCheck(kDisposeRef(&obj->sourceOptions)); 

    kCheck(kDisposeRef(&obj->xAnchorOptions));
    kCheck(kDisposeRef(&obj->yAnchorOptions));
    kCheck(kDisposeRef(&obj->zAnchorOptions));
    kCheck(kDisposeRef(&obj->zAngleAnchorOptions));

    return GoTool_VRelease(tool);
}

GoFx(kStatus) GoSurfaceTool_Read(GoSurfaceTool tool, kXml xml, kXmlItem item)
{
    kObj(GoSurfaceTool, tool);
    kString text = kNULL;
    kXmlItem anchorItem = kNULL;
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

        kTestArgs(!kIsNull(anchorItem = kXml_Child(xml, item, "Anchor")));
        if (kXml_ChildExists(xml, anchorItem, "X"))
        {
            kTest(kXml_Child32s(xml, anchorItem, "X", &obj->xAnchor));
            kTestArgs(!kIsNull(tempItem = kXml_Child(xml, anchorItem, "X")));
            kTest(kXml_AttrString(xml, tempItem, "options", text)); 
            kTest(GoOptionList_ParseList32u(kString_Chars(text), obj->xAnchorOptions));
        }

        if (kXml_ChildExists(xml, anchorItem, "Y"))
        {
            kTest(kXml_Child32s(xml, anchorItem, "Y", &obj->yAnchor));
            kTestArgs(!kIsNull(tempItem = kXml_Child(xml, anchorItem, "Y")));
            kTest(kXml_AttrString(xml, tempItem, "options", text)); 
            kTest(GoOptionList_ParseList32u(kString_Chars(text), obj->yAnchorOptions));
        }

        if (kXml_ChildExists(xml, anchorItem, "Z"))
        {
            kTest(kXml_Child32s(xml, anchorItem, "Z", &obj->zAnchor));
            kTestArgs(!kIsNull(tempItem = kXml_Child(xml, anchorItem, "Z")));
            kTest(kXml_AttrString(xml, tempItem, "options", text)); 
            kTest(GoOptionList_ParseList32u(kString_Chars(text), obj->zAnchorOptions));
        }

        if (kXml_ChildExists(xml, anchorItem, "ZAngle"))
        {
            kTest(kXml_Child32s(xml, anchorItem, "ZAngle", &obj->zAngleAnchor));
            kTestArgs(!kIsNull(tempItem = kXml_Child(xml, anchorItem, "ZAngle")));
            kTest(kXml_AttrString(xml, tempItem, "options", text)); 
            kTest(GoOptionList_ParseList32u(kString_Chars(text), obj->zAngleAnchorOptions));
        }
    }
    kFinally
    {
        kObject_Dispose(text);
        kEndFinally();
    }

    return kOK; }

GoFx(kStatus) GoSurfaceTool_Write(GoSurfaceTool tool, kXml xml, kXmlItem item)
{
    kObj(GoSurfaceTool, tool);
    kXmlItem streamItem = kNULL;
    kXmlItem anchorItem = kNULL;

    kCheck(kXml_AddItem(xml, item, "Stream", &streamItem));
    kCheck(kXml_SetChild32s(xml, streamItem, "Step", obj->stream.step));
    kCheck(kXml_SetChild32s(xml, streamItem, "Id", obj->stream.id));

    kCheck(kXml_SetChild32s(xml, item, "Source", obj->source));

    kCheck(kXml_AddItem(xml, item, "Anchor", &anchorItem));
    kCheck(kXml_SetChild32s(xml, anchorItem, "X", obj->xAnchor));
    kCheck(kXml_SetChild32s(xml, anchorItem, "Y", obj->yAnchor));
    kCheck(kXml_SetChild32s(xml, anchorItem, "Z", obj->zAnchor));
    kCheck(kXml_SetChild32s(xml, anchorItem, "ZAngle", obj->zAngleAnchor));

    kCheck(GoTool_VWrite(tool, xml, item));

    return kOK;
}


GoFx(kSize) GoSurfaceTool_StreamOptionCount(GoSurfaceTool tool)
{
    kObj(GoSurfaceTool, tool);

    GoSensor_SyncConfig(obj->base.sensor);

    return kArrayList_Count(obj->streamOptions);
}

GoFx(GoDataStream) GoSurfaceTool_StreamOptionAt(GoSurfaceTool tool, kSize index)
{
    kObj(GoSurfaceTool, tool);

    GoSensor_SyncConfig(obj->base.sensor);

    kAssert(index < kArrayList_Count(obj->streamOptions));

    return kArrayList_AsT(obj->streamOptions, index, GoDataStream);
}

GoFx(kStatus) GoSurfaceTool_SetStream(GoSurfaceTool tool, GoDataStream stream)
{
    kObj(GoSurfaceTool, tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    kCheck(GoSensor_SyncConfig(GoTool_Sensor(tool)));

    if (GoTools_IsToolInOptionsList(GoSensor_Tools(GoTool_Sensor(tool)), kObject_Type(tool)))
    {
        kCheck(GoOptionList_CheckDataStream(kArrayList_DataT(obj->streamOptions, const GoDataStream), kArrayList_Count(obj->streamOptions), stream));
    }

    obj->stream = stream;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(GoDataStream) GoSurfaceTool_Stream(GoSurfaceTool tool)
{
    kObj(GoSurfaceTool, tool);

    GoSensor_SyncConfig(obj->base.sensor);

    return obj->stream;
}

GoFx(kStatus) GoSurfaceTool_SetSource(GoSurfaceTool tool, GoDataSource source)
{
    kObj(GoSurfaceTool, tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    kCheck(GoSensor_SyncConfig(GoTool_Sensor(tool)));

    if (GoTools_IsToolInOptionsList(GoSensor_Tools(GoTool_Sensor(tool)), kObject_Type(tool)))
    {
        kCheck(GoOptionList_Check32u(kArrayList_DataT(obj->sourceOptions, const k32u), kArrayList_Count(obj->sourceOptions), source));
    }

    obj->source = source;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(kSize) GoSurfaceTool_SourceOptionCount(GoSurfaceTool tool)
{
    kObj(GoSurfaceTool, tool);

    GoSensor_SyncConfig(obj->base.sensor);

    return kArrayList_Count(obj->sourceOptions);
}

GoFx(k32u) GoSurfaceTool_SourceOptionAt(GoSurfaceTool tool, kSize index)
{
    kObj(GoSurfaceTool, tool);

    GoSensor_SyncConfig(obj->base.sensor);

    kAssert(index < kArrayList_Count(obj->sourceOptions));

    return kArrayList_AsT(obj->sourceOptions, index, k32u);
}

GoFx(GoDataSource) GoSurfaceTool_Source(GoSurfaceTool tool)
{
    kObj(GoSurfaceTool, tool);

    GoSensor_SyncConfig(obj->base.sensor);

    return obj->source;
}

GoFx(kSize) GoSurfaceTool_XAnchorOptionCount(GoSurfaceTool tool)
{
    kObj(GoSurfaceTool, tool);

    GoSensor_SyncConfig(obj->base.sensor);

    return kArrayList_Count(obj->xAnchorOptions);
}

GoFx(k32u) GoSurfaceTool_XAnchorOptionAt(GoSurfaceTool tool, kSize index)
{
    kObj(GoSurfaceTool, tool);

    GoSensor_SyncConfig(obj->base.sensor);

    kAssert(index < kArrayList_Count(obj->xAnchorOptions));

    return kArrayList_AsT(obj->xAnchorOptions, index, k32u);
}

GoFx(k32s) GoSurfaceTool_XAnchor(GoSurfaceTool tool)
{
    kObj(GoSurfaceTool, tool);

    GoSensor_SyncConfig(obj->base.sensor);

    return obj->xAnchor;
}

GoFx(kStatus) GoSurfaceTool_SetXAnchor(GoSurfaceTool tool, k32s id)
{
    kObj(GoSurfaceTool, tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    kCheck(GoSensor_SyncConfig(GoTool_Sensor(tool)));

    if (id != GO_MEASUREMENT_ID_NONE)
    {
        kCheck(GoOptionList_Check32u(kArrayList_DataT(obj->xAnchorOptions, const k32u), kArrayList_Count(obj->xAnchorOptions), id));
    }

    obj->xAnchor = id;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(kBool) GoSurfaceTool_XAnchorEnabled(GoSurfaceTool tool)
{
    kObj(GoSurfaceTool, tool);

    GoSensor_SyncConfig(obj->base.sensor);

    if (obj->xAnchor != GO_MEASUREMENT_ID_NONE)
    {
        return kTRUE;
    }

    return kFALSE;
}

GoFx(kSize) GoSurfaceTool_YAnchorOptionCount(GoSurfaceTool tool)
{
    kObj(GoSurfaceTool, tool);

    GoSensor_SyncConfig(obj->base.sensor);

    return kArrayList_Count(obj->yAnchorOptions);
}

GoFx(k32u) GoSurfaceTool_YAnchorOptionAt(GoSurfaceTool tool, kSize index)
{
    kObj(GoSurfaceTool, tool);

    GoSensor_SyncConfig(obj->base.sensor);

    kAssert(index < kArrayList_Count(obj->yAnchorOptions));

    return kArrayList_AsT(obj->yAnchorOptions, index, k32u);
}

GoFx(k32s) GoSurfaceTool_YAnchor(GoSurfaceTool tool)
{
    kObj(GoSurfaceTool, tool);

    GoSensor_SyncConfig(obj->base.sensor);

    return obj->yAnchor;
}

GoFx(kStatus) GoSurfaceTool_SetYAnchor(GoSurfaceTool tool, k32s id)
{
    kObj(GoSurfaceTool, tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    kCheck(GoSensor_SyncConfig(GoTool_Sensor(tool)));

    if (id != GO_MEASUREMENT_ID_NONE)
    {
        kCheck(GoOptionList_Check32u(kArrayList_DataT(obj->yAnchorOptions, const k32u), kArrayList_Count(obj->yAnchorOptions), id));
    }

    obj->yAnchor = id;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(kBool) GoSurfaceTool_YAnchorEnabled(GoSurfaceTool tool)
{
    kObj(GoSurfaceTool, tool);

    GoSensor_SyncConfig(obj->base.sensor);

    if (obj->yAnchor != GO_MEASUREMENT_ID_NONE)
    {
        return kTRUE;
    }

    return kFALSE;
}

GoFx(kSize) GoSurfaceTool_ZAnchorOptionCount(GoSurfaceTool tool)
{
    kObj(GoSurfaceTool, tool);

    GoSensor_SyncConfig(obj->base.sensor);

    return kArrayList_Count(obj->zAnchorOptions);
}

GoFx(k32u) GoSurfaceTool_ZAnchorOptionAt(GoSurfaceTool tool, kSize index)
{
    kObj(GoSurfaceTool, tool);

    GoSensor_SyncConfig(obj->base.sensor);

    kAssert(index < kArrayList_Count(obj->zAnchorOptions));

    return kArrayList_AsT(obj->zAnchorOptions, index, k32u);
}

GoFx(kStatus) GoSurfaceTool_SetZAnchor(GoSurfaceTool tool, k32s id)
{
    kObj(GoSurfaceTool, tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    kCheck(GoSensor_SyncConfig(GoTool_Sensor(tool)));

    if (id != GO_MEASUREMENT_ID_NONE)
    {
        kCheck(GoOptionList_Check32u(kArrayList_DataT(obj->zAnchorOptions, const k32u), kArrayList_Count(obj->zAnchorOptions), id));
    }

    obj->zAnchor = id;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}


GoFx(kBool) GoSurfaceTool_ZAnchorEnabled(GoSurfaceTool tool)
{
    kObj(GoSurfaceTool, tool);

    GoSensor_SyncConfig(obj->base.sensor);

    if (obj->zAnchor != GO_MEASUREMENT_ID_NONE)
    {
        return kTRUE;
    }

    return kFALSE;
}

GoFx(k32s) GoSurfaceTool_ZAnchor(GoSurfaceTool tool)
{
    kObj(GoSurfaceTool, tool);

    GoSensor_SyncConfig(obj->base.sensor);

    return obj->zAnchor;
}

GoFx(kSize) GoSurfaceTool_ZAngleAnchorOptionCount(GoSurfaceTool tool)
{
    kObj(GoSurfaceTool, tool);

    GoSensor_SyncConfig(obj->base.sensor);

    return kArrayList_Count(obj->zAngleAnchorOptions);
}

GoFx(k32u) GoSurfaceTool_ZAngleAnchorOptionAt(GoSurfaceTool tool, kSize index)
{
    kObj(GoSurfaceTool, tool);

    GoSensor_SyncConfig(obj->base.sensor);

    kAssert(index < kArrayList_Count(obj->zAngleAnchorOptions));

    return kArrayList_AsT(obj->zAngleAnchorOptions, index, k32u);
}

GoFx(kStatus) GoSurfaceTool_SetZAngleAnchor(GoSurfaceTool tool, k32s id)
{
    kObj(GoSurfaceTool, tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    kCheck(GoSensor_SyncConfig(GoTool_Sensor(tool)));

    if (id != GO_MEASUREMENT_ID_NONE)
    {
        kCheck(GoOptionList_Check32u(kArrayList_DataT(obj->zAngleAnchorOptions, const k32u), kArrayList_Count(obj->zAngleAnchorOptions), id));
    }

    obj->zAngleAnchor = id;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(kBool) GoSurfaceTool_ZAngleAnchorEnabled(GoSurfaceTool tool)
{
    kObj(GoSurfaceTool, tool);

    GoSensor_SyncConfig(obj->base.sensor);

    if (obj->zAngleAnchor != GO_MEASUREMENT_ID_NONE)
    {
        return kTRUE;
    }

    return kFALSE;
}

GoFx(k32s) GoSurfaceTool_ZAngleAnchor(GoSurfaceTool tool)
{
    kObj(GoSurfaceTool, tool);

    GoSensor_SyncConfig(obj->base.sensor);

    return obj->zAngleAnchor;
}

kBeginClassEx(Go, GoSurfaceBox)
kAddVMethod(GoSurfaceBox, kObject, VRelease)
kAddVMethod(GoSurfaceBox, GoTool, VInit)
kAddVMethod(GoSurfaceBox, GoTool, VRead)
kAddVMethod(GoSurfaceBox, GoTool, VWrite)
kEndClassEx()

GoFx(kStatus) GoSurfaceBox_Construct(GoSurfaceBox* tool, kObject sensor, kAlloc allocator)
{
    return GoTool_Construct(tool, kTypeOf(GoSurfaceBox), sensor, allocator);
}

GoFx(kStatus) GoSurfaceBox_VInit(GoSurfaceBox tool, kType type, kObject sensor, kAlloc alloc)
{
    kObjR(GoSurfaceBox, tool);

    kCheck(GoSurfaceTool_Init(tool, type, GO_TOOL_SURFACE_BOUNDING_BOX, sensor, alloc));
    obj->zRotationEnabled = kFALSE;
    obj->regionEnabled = kFALSE;
    obj->region = kNULL;
    obj->asymDetectType = GO_BOX_ASYMMETRY_TYPE_NONE;

    kCheck(GoRegion3d_Construct(&obj->region, sensor, alloc));

    obj->regionEnabled = kTRUE;

    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoSurfaceBoxX), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoSurfaceBoxY), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoSurfaceBoxZ), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoSurfaceBoxWidth), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoSurfaceBoxLength), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoSurfaceBoxHeight), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoSurfaceBoxGlobalX), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoSurfaceBoxGlobalY), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoSurfaceBoxGlobalZAngle), kTRUE, kNULL));

    return kOK;
}

GoFx(kStatus) GoSurfaceBox_VRelease(GoSurfaceBox tool)
{
    kObj(GoSurfaceBox, tool);

    kDestroyRef(&obj->region);

    return GoSurfaceTool_VRelease(tool);
}

GoFx(kStatus) GoSurfaceBox_VRead(GoSurfaceBox tool, kXml xml, kXmlItem item)
{
    kObj(GoSurfaceBox, tool);
    kXmlItem regionItem = kNULL;

    kCheck(kXml_ChildBool(xml, item, "ZRotationEnabled", &obj->zRotationEnabled));
    kCheck(kXml_ChildBool(xml, item, "RegionEnabled", &obj->regionEnabled));

    kCheck(!kIsNull(regionItem = kXml_Child(xml, item, "Region")));
    kCheck(GoRegion3d_Read(obj->region, xml, regionItem));

    kCheck(kXml_Child32s(xml, item, "AsymmetryDetectionType", &obj->asymDetectType));

    kCheck(GoSurfaceTool_Read(tool, xml, item));

    return kOK;
}

GoFx(kStatus) GoSurfaceBox_VWrite(GoSurfaceBox tool, kXml xml, kXmlItem item)
{
    kObj(GoSurfaceBox, tool);
    kXmlItem regionItem = kNULL;

    kCheck(kXml_SetChildBool(xml, item, "ZRotationEnabled", obj->zRotationEnabled));
    kCheck(kXml_SetChildBool(xml, item, "RegionEnabled", obj->regionEnabled));

    kCheck(kXml_AddItem(xml, item, "Region", &regionItem));
    kCheck(GoRegion3d_Write(obj->region, xml, regionItem));

    kCheck(kXml_SetChild32s(xml, item, "AsymmetryDetectionType", obj->asymDetectType));

    kCheck(GoSurfaceTool_Write(tool, xml, item));

    return kOK;
}

GoFx(kBool) GoSurfaceBox_ZRotationEnabled(GoSurfaceBox tool)
{
    kObj(GoSurfaceBox, tool);

    return obj->zRotationEnabled;
}

GoFx(kStatus) GoSurfaceBox_EnableZRotation(GoSurfaceBox tool, kBool enable)
{
    kObj(GoSurfaceBox, tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    obj->zRotationEnabled = enable;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(kBool) GoSurfaceBox_RegionEnabled(GoSurfaceBox tool)
{
    kObj(GoSurfaceBox, tool);

    return obj->regionEnabled;
}

GoFx(kStatus) GoSurfaceBox_EnableRegion(GoSurfaceBox tool, kBool enable)
{
    kObj(GoSurfaceBox, tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    obj->regionEnabled = enable;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(GoRegion3d) GoSurfaceBox_Region(GoSurfaceBox tool)
{
    kObj(GoSurfaceBox, tool);

    return obj->region;
}

GoFx(kStatus) GoSurfaceBox_SetAsymmetryDetectionType(GoSurfaceBox tool, GoBoxAsymmetryType value)
{
    kObj(GoSurfaceBox, tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    obj->asymDetectType = value;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(GoBoxAsymmetryType) GoSurfaceBox_AsymmetryDetectionType(GoSurfaceBox tool)
{
    kObj(GoSurfaceBox, tool);

    return obj->asymDetectType;
}


GoFx(GoSurfaceBoxX) GoSurfaceBox_XMeasurement(GoSurfaceBox tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_SURFACE_BOUNDING_BOX_X);
}

GoFx(GoSurfaceBoxY) GoSurfaceBox_YMeasurement(GoSurfaceBox tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_SURFACE_BOUNDING_BOX_Y);
}

GoFx(GoSurfaceBoxZ) GoSurfaceBox_ZMeasurement(GoSurfaceBox tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_SURFACE_BOUNDING_BOX_Z);
}

GoFx(GoSurfaceBoxWidth) GoSurfaceBox_WidthMeasurement(GoSurfaceBox tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_SURFACE_BOUNDING_BOX_WIDTH);
}

GoFx(GoSurfaceBoxLength) GoSurfaceBox_LengthMeasurement(GoSurfaceBox tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_SURFACE_BOUNDING_BOX_LENGTH);
}

GoFx(GoSurfaceBoxHeight) GoSurfaceBox_HeightMeasurement(GoSurfaceBox tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_SURFACE_BOUNDING_BOX_HEIGHT);
}

GoFx(GoSurfaceBoxZAngle) GoSurfaceBox_ZAngleMeasurement(GoSurfaceBox tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_SURFACE_BOUNDING_BOX_ZANGLE);
}

GoFx(GoSurfaceBoxGlobalX) GoSurfaceBox_GlobalXMeasurement(GoSurfaceBox tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_SURFACE_BOUNDING_BOX_GLOBAL_X);
}

GoFx(GoSurfaceBoxGlobalY) GoSurfaceBox_GlobalYMeasurement(GoSurfaceBox tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_SURFACE_BOUNDING_BOX_GLOBAL_Y);
}

GoFx(GoSurfaceBoxGlobalZAngle) GoSurfaceBox_GlobalZAngleMeasurement(GoSurfaceBox tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_SURFACE_BOUNDING_BOX_GLOBAL_Z_ANGLE);
}

GoFx(GoSurfaceBoundingBoxCenterPoint) GoSurfaceBox_CenterPoint(GoSurfaceBox tool)
{
    return GoTool_FindFeatureOutputByType(tool, GO_FEATURE_SURFACE_BOUNDING_BOX_CENTER_POINT);
}

GoFx(GoSurfaceBoundingBoxAxisLine) GoSurfaceBox_AxisLine(GoSurfaceBox tool)
{
    return GoTool_FindFeatureOutputByType(tool, GO_FEATURE_SURFACE_BOUNDING_BOX_AXIS_LINE);
}

kBeginClassEx(Go, GoSurfaceCountersunkHole)
kAddVMethod(GoSurfaceCountersunkHole, kObject, VRelease)
kAddVMethod(GoSurfaceCountersunkHole, GoTool, VInit)
kAddVMethod(GoSurfaceCountersunkHole, GoTool, VRead)
kAddVMethod(GoSurfaceCountersunkHole, GoTool, VWrite)
kEndClassEx()

GoFx(kStatus) GoSurfaceCountersunkHole_Construct(GoSurfaceCountersunkHole* tool, kObject sensor, kAlloc allocator)
{
    return GoTool_Construct(tool, kTypeOf(GoSurfaceCountersunkHole), sensor, allocator);
}

GoFx(kStatus) GoSurfaceCountersunkHole_VInit(GoSurfaceCountersunkHole tool, kType type, kObject sensor, kAlloc alloc)
{
    kObjR(GoSurfaceCountersunkHole, tool);
    kSize i;

    kCheck(GoSurfaceTool_Init(tool, type, GO_TOOL_SURFACE_COUNTERSUNK_HOLE, sensor, alloc));
    obj->shape = GO_SURFACE_COUNTERSUNK_HOLE_SHAPE_CONE;
    obj->region = kNULL;
    obj->refRegionsEnabled = kFALSE;
    obj->refRegionCount = 0;
    kZero(obj->refRegions);
    obj->curveFitEnabled = kFALSE;
    obj->curveOrientation = 0.0;
    obj->planeFitRangeEnabled = kFALSE;
    obj->planeFitRange = 0.0;

    obj->nominalBevelAngle = 100.0;
    obj->bevelAngleTolerance = 5.0;
    obj->nominalOuterRadius = 10.0;
    obj->outerRadiusTolerance = 1.0;
    obj->nominalInnerRadius = 4.0;
    obj->innerRadiusTolerance = 1.0;
    obj->bevelRadiusOffset = 4.0;
    obj->partialDetectionEnabled = kFALSE;
    obj->autoTiltEnabled = kTRUE;
    obj->tiltXAngle = 0.0;
    obj->tiltYAngle = 0.0;

    obj->regionEnabled = kTRUE;
    kCheck(GoRegion3d_Construct(&obj->region, sensor, alloc));

    obj->refRegionsEnabled = kFALSE;
    obj->refRegionCount = 0;
    for (i = 0; i < GO_SURFACE_COUNTERSUNK_HOLE_MAX_REF_REGIONS; i++)
    {
        kCheck(GoSurfaceRegion2d_Construct(&obj->refRegions[i], sensor, alloc));
    }

    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoSurfaceCountersunkHoleX), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoSurfaceCountersunkHoleY), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoSurfaceCountersunkHoleZ), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoSurfaceCountersunkHoleOuterRadius), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoSurfaceCountersunkHoleDepth), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoSurfaceCountersunkHoleCounterboreDepth), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoSurfaceCountersunkHoleBevelAngle), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoSurfaceCountersunkHoleBevelRadius), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoSurfaceCountersunkHoleXAngle), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoSurfaceCountersunkHoleYAngle), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoSurfaceCountersunkHoleAxisTilt), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoSurfaceCountersunkHoleAxisOrientation), kTRUE, kNULL));

    return kOK;
}

GoFx(kStatus) GoSurfaceCountersunkHole_VRelease(GoSurfaceCountersunkHole tool)
{
    kObj(GoSurfaceCountersunkHole, tool);
    kSize i;

    kDestroyRef(&obj->region);

    for (i = 0; i < GO_SURFACE_HOLE_MAX_REF_REGIONS; i++)
    {
        kDestroyRef(&obj->refRegions[i]);
    }

    return GoSurfaceTool_VRelease(tool);
}

GoFx(kStatus) GoSurfaceCountersunkHole_VRead(GoSurfaceCountersunkHole tool, kXml xml, kXmlItem item)
{
    kObj(GoSurfaceCountersunkHole, tool);
    kSize i;
    kXmlItem regionItem = kNULL;
    kXmlItem refRegionsItem = kNULL;

    if (kXml_ChildExists(xml, item, "Shape"))
    {
        kCheck(kXml_Child32s(xml, item, "Shape", &obj->shape));
    }

    kCheck(kXml_Child64f(xml, item, "NominalBevelAngle", &obj->nominalBevelAngle));

    if (kXml_ChildExists(xml, item, "BevelAngleTolerance"))
    {
        kCheck(kXml_Child64f(xml, item, "BevelAngleTolerance", &obj->bevelAngleTolerance));
    }

    kCheck(kXml_Child64f(xml, item, "NominalOuterRadius", &obj->nominalOuterRadius));

    if (kXml_ChildExists(xml, item, "OuterRadiusTolerance"))
    {
        kCheck(kXml_Child64f(xml, item, "OuterRadiusTolerance", &obj->outerRadiusTolerance));
    }

    kCheck(kXml_Child64f(xml, item, "NominalInnerRadius", &obj->nominalInnerRadius));

    if (kXml_ChildExists(xml, item, "InnerRadiusTolerance"))
    {
        kCheck(kXml_Child64f(xml, item, "InnerRadiusTolerance", &obj->innerRadiusTolerance));
    }

    kCheck(kXml_Child64f(xml, item, "BevelRadiusOffset", &obj->bevelRadiusOffset));

    kCheck(kXml_ChildBool(xml, item, "PartialDetectionEnabled", &obj->partialDetectionEnabled));

    kCheck(kXml_ChildBool(xml, item, "RegionEnabled", &obj->regionEnabled));
    kCheck(!kIsNull(regionItem = kXml_Child(xml, item, "Region")));
    kCheck(GoRegion3d_Read(obj->region, xml, regionItem));

    kCheck(kXml_ChildBool(xml, item, "RefRegionsEnabled", &obj->refRegionsEnabled));
    kCheck(kXml_ChildSize(xml, item, "RefRegionCount", &obj->refRegionCount));

    kCheck(!kIsNull(refRegionsItem = kXml_Child(xml, item, "RefRegions")));
    for (i = 0; i < GO_SURFACE_COUNTERSUNK_HOLE_MAX_REF_REGIONS; i++)
    {
        kCheck(!kIsNull(regionItem = kXml_ChildAt(xml, refRegionsItem, i)));
        kCheck(GoSurfaceRegion2d_Read(obj->refRegions[i], xml, regionItem));
    }

    kCheck(kXml_ChildBool(xml, item, "AutoTiltEnabled", &obj->autoTiltEnabled));
    kCheck(kXml_Child64f(xml, item, "TiltXAngle", &obj->tiltXAngle));
    kCheck(kXml_Child64f(xml, item, "TiltYAngle", &obj->tiltYAngle));

    kCheck(GoConfig_ReadBoolOptional(xml, item, "CurveFitEnabled", kFALSE, &obj->curveFitEnabled));
    kCheck(GoConfig_Read64fOptional(xml, item, "CurveOrientation", 0.0, &obj->curveOrientation));

    kCheck(GoConfig_ReadBoolOptional(xml, item, "PlaneFitRangeEnabled", kFALSE, &obj->planeFitRangeEnabled));
    kCheck(GoConfig_Read64fOptional(xml, item, "PlaneFitRange", 0.0, &obj->planeFitRange));

    kCheck(GoSurfaceTool_Read(tool, xml, item));

    return kOK;
}

GoFx(kStatus) GoSurfaceCountersunkHole_VWrite(GoSurfaceCountersunkHole tool, kXml xml, kXmlItem item)
{
    kObj(GoSurfaceCountersunkHole, tool);
    kXmlItem regionItem = kNULL;
    kXmlItem refRegionsItem = kNULL;
    kSize i;

    kCheck(kXml_SetChild32s(xml, item, "Shape", obj->shape));

    kCheck(kXml_SetChild64f(xml, item, "NominalBevelAngle", obj->nominalBevelAngle));
    kCheck(kXml_SetChild64f(xml, item, "BevelAngleTolerance", obj->bevelAngleTolerance));

    kCheck(kXml_SetChild64f(xml, item, "NominalOuterRadius", obj->nominalOuterRadius));
    kCheck(kXml_SetChild64f(xml, item, "OuterRadiusTolerance", obj->outerRadiusTolerance));

    kCheck(kXml_SetChild64f(xml, item, "NominalInnerRadius", obj->nominalInnerRadius));
    kCheck(kXml_SetChild64f(xml, item, "InnerRadiusTolerance", obj->innerRadiusTolerance));

    kCheck(kXml_SetChild64f(xml, item, "BevelRadiusOffset", obj->bevelRadiusOffset));

    kCheck(kXml_SetChildBool(xml, item, "PartialDetectionEnabled", obj->partialDetectionEnabled));

    kCheck(kXml_SetChildBool(xml, item, "RegionEnabled", obj->regionEnabled));
    kCheck(kXml_AddItem(xml, item, "Region", &regionItem));
    kCheck(GoRegion3d_Write(obj->region, xml, regionItem));

    kCheck(kXml_SetChildBool(xml, item, "RefRegionsEnabled", obj->refRegionsEnabled));
    kCheck(kXml_SetChildSize(xml, item, "RefRegionCount", obj->refRegionCount));

    kCheck(kXml_AddItem(xml, item, "RefRegions", &refRegionsItem));
    for (i = 0; i < GO_SURFACE_HOLE_MAX_REF_REGIONS; i++)
    {
        kCheck(kXml_AddItem(xml, refRegionsItem, "RefRegion", &regionItem));
        kCheck(GoSurfaceRegion2d_Write(obj->refRegions[i], xml, regionItem));
    }

    kCheck(kXml_SetChildBool(xml, item, "AutoTiltEnabled", obj->autoTiltEnabled));
    kCheck(kXml_SetChild64f(xml, item, "TiltXAngle", obj->tiltXAngle));
    kCheck(kXml_SetChild64f(xml, item, "TiltYAngle", obj->tiltYAngle));

    kCheck(kXml_SetChildBool(xml, item, "CurveFitEnabled", obj->curveFitEnabled));
    kCheck(kXml_SetChild64f(xml, item, "CurveOrientation", obj->curveOrientation));

    kCheck(kXml_SetChildBool(xml, item, "PlaneFitRangeEnabled", obj->planeFitRangeEnabled));
    kCheck(kXml_SetChild64f(xml, item, "PlaneFitRange", obj->planeFitRange));

    kCheck(GoSurfaceTool_Write(tool, xml, item));

    return kOK;
}

GoFx(kStatus) GoSurfaceCountersunkHole_SetShape(GoSurfaceCountersunkHole tool, GoSurfaceCountersunkHoleShape value)
{
    kObj(GoSurfaceCountersunkHole, tool);

    obj->shape = value;

    return kOK;
}

GoFx(GoSurfaceCountersunkHoleShape) GoSurfaceCountersunkHole_Shape(GoSurfaceCountersunkHole tool)
{
    kObj(GoSurfaceCountersunkHole, tool);

    return obj->shape;
}

GoFx(kStatus) GoSurfaceCountersunkHole_SetNominalBevelAngle(GoSurfaceCountersunkHole tool, k64f value)
{
    kObj(GoSurfaceCountersunkHole, tool);

    obj->nominalBevelAngle = value;

    return kOK;
}

GoFx(k64f) GoSurfaceCountersunkHole_NominalBevelAngle(GoSurfaceCountersunkHole tool)
{
    kObj(GoSurfaceCountersunkHole, tool);

    return obj->nominalBevelAngle;
}

GoFx(kStatus) GoSurfaceCountersunkHole_SetBevelAngleTolerance(GoSurfaceCountersunkHole tool, k64f value)
{
    kObj(GoSurfaceCountersunkHole, tool);

    obj->bevelAngleTolerance = value;

    return kOK;
}

GoFx(k64f) GoSurfaceCountersunkHole_BevelAngleTolerance(GoSurfaceCountersunkHole tool)
{
    kObj(GoSurfaceCountersunkHole, tool);

    return obj->bevelAngleTolerance;
}

GoFx(kStatus) GoSurfaceCountersunkHole_SetNominalOuterRadius(GoSurfaceCountersunkHole tool, k64f value)
{
    kObj(GoSurfaceCountersunkHole, tool);

    obj->nominalOuterRadius = value;

    return kOK;
}

GoFx(k64f) GoSurfaceCountersunkHole_NominalOuterRadius(GoSurfaceCountersunkHole tool)
{
    kObj(GoSurfaceCountersunkHole, tool);

    return obj->nominalOuterRadius;
}

GoFx(kStatus) GoSurfaceCountersunkHole_SetOuterRadiusTolerance(GoSurfaceCountersunkHole tool, k64f value)
{
    kObj(GoSurfaceCountersunkHole, tool);

    obj->outerRadiusTolerance = value;

    return kOK;
}

GoFx(k64f) GoSurfaceCountersunkHole_OuterRadiusTolerance(GoSurfaceCountersunkHole tool)
{
    kObj(GoSurfaceCountersunkHole, tool);

    return obj->outerRadiusTolerance;
}

GoFx(kStatus) GoSurfaceCountersunkHole_SetNominalInnerRadius(GoSurfaceCountersunkHole tool, k64f value)
{
    kObj(GoSurfaceCountersunkHole, tool);

    obj->nominalInnerRadius = value;

    return kOK;
}

GoFx(k64f) GoSurfaceCountersunkHole_NominalInnerRadius(GoSurfaceCountersunkHole tool)
{
    kObj(GoSurfaceCountersunkHole, tool);

    return obj->nominalInnerRadius;
}

GoFx(kStatus) GoSurfaceCountersunkHole_SetInnerRadiusTolerance(GoSurfaceCountersunkHole tool, k64f value)
{
    kObj(GoSurfaceCountersunkHole, tool);

    obj->innerRadiusTolerance = value;

    return kOK;
}

GoFx(k64f) GoSurfaceCountersunkHole_InnerRadiusTolerance(GoSurfaceCountersunkHole tool)
{
    kObj(GoSurfaceCountersunkHole, tool);

    return obj->innerRadiusTolerance;
}

GoFx(kStatus) GoSurfaceCountersunkHole_SetBevelRadiusOffset(GoSurfaceCountersunkHole tool, k64f value)
{
    kObj(GoSurfaceCountersunkHole, tool);

    obj->bevelRadiusOffset = value;

    return kOK;
}

GoFx(k64f) GoSurfaceCountersunkHole_BevelRadiusOffset(GoSurfaceCountersunkHole tool)
{
    kObj(GoSurfaceCountersunkHole, tool);

    return obj->bevelRadiusOffset;
}

GoFx(kStatus) GoSurfaceCountersunkHole_EnablePartialDetection(GoSurfaceCountersunkHole tool, kBool enable)
{
    kObj(GoSurfaceCountersunkHole, tool);

    obj->partialDetectionEnabled = enable;

    return kOK;
}

GoFx(kBool) GoSurfaceCountersunkHole_PartialDetectionEnabled(GoSurfaceCountersunkHole tool)
{
    kObj(GoSurfaceCountersunkHole, tool);

    return obj->partialDetectionEnabled;
}

GoFx(kStatus) GoSurfaceCountersunkHole_EnableRegion(GoSurfaceCountersunkHole tool, kBool enable)
{
    kObj(GoSurfaceCountersunkHole, tool);

    obj->regionEnabled = enable;

    return kOK;
}

GoFx(kBool) GoSurfaceCountersunkHole_RegionEnabled(GoSurfaceCountersunkHole tool)
{
    kObj(GoSurfaceCountersunkHole, tool);

    return obj->regionEnabled;
}

GoFx(GoRegion3d) GoSurfaceCountersunkHole_Region(GoSurfaceCountersunkHole tool)
{
    kObj(GoSurfaceCountersunkHole, tool);

    return obj->region;
}

GoFx(kStatus) GoSurfaceCountersunkHole_EnableRefRegions(GoSurfaceCountersunkHole tool, kBool enable)
{
    kObj(GoSurfaceCountersunkHole, tool);

    obj->refRegionsEnabled = enable;

    return kOK;
}

GoFx(kBool) GoSurfaceCountersunkHole_RefRegionsEnabled(GoSurfaceCountersunkHole tool)
{
    kObj(GoSurfaceCountersunkHole, tool);

    return obj->refRegionsEnabled;
}

GoFx(kStatus) GoSurfaceCountersunkHole_SetRefRegionCount(GoSurfaceCountersunkHole tool, kSize count)
{
    kObj(GoSurfaceCountersunkHole, tool);

    kCheckArgs(count <= GO_SURFACE_COUNTERSUNK_HOLE_MAX_REF_REGIONS);

    obj->refRegionCount = count;

    return kOK;
}

GoFx(kSize) GoSurfaceCountersunkHole_RefRegionCount(GoSurfaceCountersunkHole tool)
{
    kObj(GoSurfaceCountersunkHole, tool);

    return obj->refRegionCount;
}

GoFx(GoSurfaceRegion2d) GoSurfaceCountersunkHole_RefRegionAt(GoSurfaceCountersunkHole tool, kSize index)
{
    kObj(GoSurfaceCountersunkHole, tool);

    kAssert(index < GO_SURFACE_COUNTERSUNK_HOLE_MAX_REF_REGIONS);

    return obj->refRegions[index];
}

GoFx(kStatus) GoSurfaceCountersunkHole_EnableAutoTilt(GoSurfaceCountersunkHole tool, kBool enable)
{
    kObj(GoSurfaceCountersunkHole, tool);

    obj->autoTiltEnabled = enable;

    return kOK;
}

GoFx(kBool) GoSurfaceCountersunkHole_AutoTiltEnabled(GoSurfaceCountersunkHole tool)
{
    kObj(GoSurfaceCountersunkHole, tool);

    return obj->autoTiltEnabled;
}

GoFx(kStatus) GoSurfaceCountersunkHole_SetTiltXAngle(GoSurfaceCountersunkHole tool, k64f value)
{
    kObj(GoSurfaceCountersunkHole, tool);

    obj->tiltXAngle = value;

    return kOK;
}

GoFx(k64f) GoSurfaceCountersunkHole_TiltXAngle(GoSurfaceCountersunkHole tool)
{
    kObj(GoSurfaceCountersunkHole, tool);

    return obj->tiltXAngle;
}

GoFx(kStatus) GoSurfaceCountersunkHole_SetTiltYAngle(GoSurfaceCountersunkHole tool, k64f value)
{
    kObj(GoSurfaceCountersunkHole, tool);

    obj->tiltYAngle = value;

    return kOK;
}

GoFx(k64f) GoSurfaceCountersunkHole_TiltYAngle(GoSurfaceCountersunkHole tool)
{
    kObj(GoSurfaceCountersunkHole, tool);

    return obj->tiltYAngle;
}


GoFx(kStatus) GoSurfaceCountersunkHole_EnableCurveFit(GoSurfaceCountersunkHole tool, kBool enable)
{
    kObj(GoSurfaceCountersunkHole, tool);

    obj->curveFitEnabled = enable;

    return kOK;
}

GoFx(kBool) GoSurfaceCountersunkHole_CurveFitEnabled(GoSurfaceCountersunkHole tool)
{
    kObj(GoSurfaceCountersunkHole, tool);

    return obj->curveFitEnabled;
}

GoFx(kStatus) GoSurfaceCountersunkHole_SetCurveOrientation(GoSurfaceCountersunkHole tool, k64f value)
{
    kObj(GoSurfaceCountersunkHole, tool);

    obj->curveOrientation = value;

    return kOK;
}

GoFx(k64f) GoSurfaceCountersunkHole_CurveOrientation(GoSurfaceCountersunkHole tool)
{
    kObj(GoSurfaceCountersunkHole, tool);

    return obj->curveOrientation;
}

GoFx(kBool) GoSurfaceCountersunkHole_PlaneFitRangeEnabled(GoSurfaceCountersunkHole tool)
{
    kObj(GoSurfaceCountersunkHole, tool);

    return obj->planeFitRangeEnabled;
}

GoFx(kStatus) GoSurfaceCountersunkHole_EnablePlaneFitRange(GoSurfaceCountersunkHole tool, kBool enable)
{
    kObj(GoSurfaceCountersunkHole, tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    obj->planeFitRangeEnabled = enable;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(kStatus) GoSurfaceCountersunkHole_SetPlaneFitRange(GoSurfaceCountersunkHole tool, k64f value)
{
    kObj(GoSurfaceCountersunkHole, tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    obj->planeFitRange = value;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(k64f) GoSurfaceCountersunkHole_PlaneFitRange(GoSurfaceCountersunkHole tool)
{
    kObj(GoSurfaceCountersunkHole, tool);

    return obj->planeFitRange;
}

GoFx(GoSurfaceCountersunkHoleX) GoSurfaceCountersunkHole_XMeasurement(GoSurfaceCountersunkHole tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_SURFACE_COUNTERSUNK_HOLE_X);
}

GoFx(GoSurfaceCountersunkHoleY) GoSurfaceCountersunkHole_YMeasurement(GoSurfaceCountersunkHole tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_SURFACE_COUNTERSUNK_HOLE_Y);
}

GoFx(GoSurfaceCountersunkHoleZ) GoSurfaceCountersunkHole_ZMeasurement(GoSurfaceCountersunkHole tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_SURFACE_COUNTERSUNK_HOLE_Z);
}

GoFx(GoSurfaceCountersunkHoleOuterRadius) GoSurfaceCountersunkHole_OuterRadiusMeasurement(GoSurfaceCountersunkHole tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_SURFACE_COUNTERSUNK_HOLE_OUTER_RADIUS);
}

GoFx(GoSurfaceCountersunkHoleDepth) GoSurfaceCountersunkHole_DepthMeasurement(GoSurfaceCountersunkHole tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_SURFACE_COUNTERSUNK_HOLE_DEPTH);
}

GoFx(GoSurfaceCountersunkHoleCounterboreDepth) GoSurfaceCountersunkHole_CounterboreDepth(GoSurfaceCountersunkHole tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_SURFACE_COUNTERSUNK_HOLE_COUNTERBORE_DEPTH);
}

GoFx(GoSurfaceCountersunkHoleBevelRadius) GoSurfaceCountersunkHole_BevelRadiusMeasurement(GoSurfaceCountersunkHole tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_SURFACE_COUNTERSUNK_HOLE_BEVEL_RADIUS);
}

GoFx(GoSurfaceCountersunkHoleBevelAngle) GoSurfaceCountersunkHole_BevelAngleMeasurement(GoSurfaceCountersunkHole tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_SURFACE_COUNTERSUNK_HOLE_BEVEL_ANGLE);
}

GoFx(GoSurfaceCountersunkHoleXAngle) GoSurfaceCountersunkHole_XAngleMeasurement(GoSurfaceCountersunkHole tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_SURFACE_COUNTERSUNK_HOLE_X_ANGLE);
}

GoFx(GoSurfaceCountersunkHoleYAngle) GoSurfaceCountersunkHole_YAngleMeasurement(GoSurfaceCountersunkHole tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_SURFACE_COUNTERSUNK_HOLE_Y_ANGLE);
}

GoFx(GoSurfaceCountersunkHoleAxisTilt) GoSurfaceCountersunkHole_AxisTilt(GoSurfaceCountersunkHole tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_SURFACE_COUNTERSUNK_HOLE_AXIS_TILT);
}

GoFx(GoSurfaceCountersunkHoleAxisOrientation) GoSurfaceCountersunkHole_AxisOrientation(GoSurfaceCountersunkHole tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_SURFACE_COUNTERSUNK_HOLE_AXIS_ORIENTATION);
}

GoFx(GoSurfaceCountersunkHoleCenterPoint) GoSurfaceCountersunkHole_CenterPoint(GoSurfaceCountersunkHole tool)
{
    return GoTool_FindFeatureOutputByType(tool, GO_FEATURE_SURFACE_COUNTERSUNKHOLE_CENTER_POINT);
}

kBeginClassEx(Go, GoSurfaceDim)
kAddVMethod(GoSurfaceDim, kObject, VRelease)
kAddVMethod(GoSurfaceDim, GoTool, VInit)
kAddVMethod(GoSurfaceDim, GoTool, VRead)
kAddVMethod(GoSurfaceDim, GoTool, VWrite)
kEndClassEx()

GoFx(kStatus) GoSurfaceDim_Construct(GoSurfaceDim* tool, kObject sensor, kAlloc allocator)
{
    return GoTool_Construct(tool, kTypeOf(GoSurfaceDim), sensor, allocator);
}

GoFx(kStatus) GoSurfaceDim_VInit(GoSurfaceDim tool, kType type, kObject sensor, kAlloc alloc)
{
    kObjR(GoSurfaceDim, tool);

    kCheck(GoSurfaceTool_Init(tool, type, GO_TOOL_SURFACE_DIMENSION, sensor, alloc));
    obj->refFeature = kNULL;
    obj->feature = kNULL;

    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoSurfaceDimWidth), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoSurfaceDimHeight), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoSurfaceDimLength), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoSurfaceDimDistance), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoSurfaceDimPlaneDistance), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoSurfaceDimCenterX), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoSurfaceDimCenterY), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoSurfaceDimCenterZ), kTRUE, kNULL));

    kCheck(GoSurfaceFeature_Construct(&obj->refFeature, sensor, alloc));
    kCheck(GoSurfaceFeature_Construct(&obj->feature, sensor, alloc));

    return kOK;
}

GoFx(kStatus) GoSurfaceDim_VRelease(GoSurfaceDim tool)
{
    kObj(GoSurfaceDim, tool);

    kCheck(kDestroyRef(&obj->refFeature));
    kCheck(kDestroyRef(&obj->feature));

    return GoSurfaceTool_VRelease(tool);
}

GoFx(kStatus) GoSurfaceDim_VRead(GoSurfaceDim tool, kXml xml, kXmlItem item)
{
    kObj(GoSurfaceDim, tool);
    kXmlItem featureItem;

    kCheck(GoSurfaceTool_Read(tool, xml, item));

    kCheck(!kIsNull(featureItem = kXml_Child(xml, item, "RefFeature")));
    kCheck(GoSurfaceFeature_Read(obj->refFeature, xml, featureItem));

    kCheck(!kIsNull(featureItem = kXml_Child(xml, item, "Feature")));
    kCheck(GoSurfaceFeature_Read(obj->feature, xml, featureItem));

    return kOK;
}

GoFx(kStatus) GoSurfaceDim_VWrite(GoSurfaceDim tool, kXml xml, kXmlItem item)
{
    kObj(GoSurfaceDim, tool);
    kXmlItem featureItem;

    kCheck(kXml_AddItem(xml, item, "RefFeature", &featureItem));
    kCheck(GoSurfaceFeature_Write(obj->refFeature, xml, featureItem));

    kCheck(kXml_AddItem(xml, item, "Feature", &featureItem));
    kCheck(GoSurfaceFeature_Write(obj->feature, xml, featureItem));

    kCheck(GoSurfaceTool_Write(tool, xml, item));

    return kOK;
}

GoFx(GoSurfaceFeature) GoSurfaceDim_RefFeature(GoSurfaceDim tool)
{
    kObj(GoSurfaceDim, tool);

    return obj->refFeature;
}

GoFx(GoSurfaceFeature) GoSurfaceDim_Feature(GoSurfaceDim tool)
{
    kObj(GoSurfaceDim, tool);

    return obj->feature;
}

GoFx(GoMeasurement) GoSurfaceDim_WidthMeasurement(GoSurfaceDim tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_SURFACE_DIMENSION_WIDTH);
}

GoFx(GoMeasurement) GoSurfaceDim_HeightMeasurement(GoSurfaceDim tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_SURFACE_DIMENSION_HEIGHT);
}

GoFx(GoMeasurement) GoSurfaceDim_LengthMeasurement(GoSurfaceDim tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_SURFACE_DIMENSION_LENGTH);
}

GoFx(GoMeasurement) GoSurfaceDim_DistanceMeasurement(GoSurfaceDim tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_SURFACE_DIMENSION_DISTANCE);
}

GoFx(GoMeasurement) GoSurfaceDim_PlaneDistanceMeasurement(GoSurfaceDim tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_SURFACE_DIMENSION_PLANE_DISTANCE);
}

GoFx(GoMeasurement) GoSurfaceDim_CenterXMeasurement(GoSurfaceDim tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_SURFACE_DIMENSION_CENTER_X);
}

GoFx(GoMeasurement) GoSurfaceDim_CenterYMeasurement(GoSurfaceDim tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_SURFACE_DIMENSION_CENTER_Y);
}

GoFx(GoMeasurement) GoSurfaceDim_CenterZMeasurement(GoSurfaceDim tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_SURFACE_DIMENSION_CENTER_Z);
}

GoFx(GoSurfaceDimensionCenterPoint) GoSurfaceDim_CenterPoint(GoSurfaceDim tool)
{
    return GoTool_FindFeatureOutputByType(tool, GO_FEATURE_SURFACE_DIMENSION_CENTER_POINT);
}

kBeginClassEx(Go, GoSurfaceEllipse)
kAddVMethod(GoSurfaceEllipse, kObject, VRelease)
kAddVMethod(GoSurfaceEllipse, GoTool, VInit)
kAddVMethod(GoSurfaceEllipse, GoTool, VRead)
kAddVMethod(GoSurfaceEllipse, GoTool, VWrite)
kEndClassEx()

GoFx(kStatus) GoSurfaceEllipse_Construct(GoSurfaceEllipse* tool, kObject sensor, kAlloc allocator)
{
    return GoTool_Construct(tool, kTypeOf(GoSurfaceEllipse), sensor, allocator);
}

GoFx(kStatus) GoSurfaceEllipse_VInit(GoSurfaceEllipse tool, kType type, kObject sensor, kAlloc alloc)
{
    kObjR(GoSurfaceEllipse, tool);

    kCheck(GoSurfaceTool_Init(tool, type, GO_TOOL_SURFACE_ELLIPSE, sensor, alloc));
    obj->regionEnabled = kFALSE;
    obj->region = kNULL;
    obj->asymDetectType = GO_ELLIPSE_ASYMMETRY_TYPE_NONE;

    kCheck(GoRegion3d_Construct(&obj->region, sensor, alloc));

    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoSurfaceEllipseMajor), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoSurfaceEllipseMinor), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoSurfaceEllipseRatio), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoSurfaceEllipseZAngle), kTRUE, kNULL));

    return kOK;
}

GoFx(kStatus) GoSurfaceEllipse_VRelease(GoSurfaceEllipse tool)
{
    kObj(GoSurfaceEllipse, tool);

    kCheck(kDestroyRef(&obj->region));

    return GoSurfaceTool_VRelease(tool);
}

GoFx(kStatus) GoSurfaceEllipse_VRead(GoSurfaceEllipse tool, kXml xml, kXmlItem item)
{
    kObj(GoSurfaceEllipse, tool);
    kXmlItem regionItem = kNULL;

    kCheck(kXml_ChildBool(xml, item, "RegionEnabled", &obj->regionEnabled));

    kCheck(!kIsNull(regionItem = kXml_Child(xml, item, "Region")));
    kCheck(GoRegion3d_Read(obj->region, xml, regionItem));

    kCheck(kXml_Child32s(xml, item, "AsymmetryDetectionType", &obj->asymDetectType));

    kCheck(GoSurfaceTool_Read(tool, xml, item));

    return kOK;
}

GoFx(kStatus) GoSurfaceEllipse_VWrite(GoSurfaceEllipse tool, kXml xml, kXmlItem item)
{
    kObj(GoSurfaceEllipse, tool);
    kXmlItem regionItem = kNULL;

    kCheck(kXml_SetChildBool(xml, item, "RegionEnabled", obj->regionEnabled));

    kCheck(kXml_AddItem(xml, item, "Region", &regionItem));
    kCheck(GoRegion3d_Write(obj->region, xml, regionItem));

    kCheck(kXml_SetChild32s(xml, item, "AsymmetryDetectionType", obj->asymDetectType));

    kCheck(GoSurfaceTool_Write(tool, xml, item));

    return kOK;
}

GoFx(kStatus) GoSurfaceEllipse_EnableRegion(GoSurfaceEllipse tool, kBool enable)
{
    kObj(GoSurfaceEllipse, tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    obj->regionEnabled = enable;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(kBool) GoSurfaceEllipse_RegionEnabled(GoSurfaceEllipse tool)
{
    kObj(GoSurfaceEllipse, tool);

    return obj->regionEnabled;
}

GoFx(GoRegion3d) GoSurfaceEllipse_Region(GoSurfaceEllipse tool)
{
    kObj(GoSurfaceEllipse, tool);

    return obj->region;
}

GoFx(kStatus) GoSurfaceEllipse_SetAsymmetryDetectionType(GoSurfaceEllipse tool, GoEllipseAsymmetryType value)
{
    kObj(GoSurfaceEllipse, tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    obj->asymDetectType = value;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(GoEllipseAsymmetryType) GoSurfaceEllipse_AsymmetryDetectionType(GoSurfaceEllipse tool)
{
    kObj(GoSurfaceEllipse, tool);

    return obj->asymDetectType;
}

GoFx(GoSurfaceEllipseMajor) GoSurfaceEllipse_MajorMeasurement(GoSurfaceEllipse tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_SURFACE_ELLIPSE_MAJOR);
}

GoFx(GoSurfaceEllipseMinor) GoSurfaceEllipse_MinorMeasurement(GoSurfaceEllipse tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_SURFACE_ELLIPSE_MINOR);
}

GoFx(GoSurfaceEllipseRatio) GoSurfaceEllipse_RatioMeasurement(GoSurfaceEllipse tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_SURFACE_ELLIPSE_RATIO);
}

GoFx(GoSurfaceEllipseZAngle) GoSurfaceEllipse_ZAngleMeasurement(GoSurfaceEllipse tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_SURFACE_ELLIPSE_ZANGLE);
}

GoFx(GoSurfaceEllipseCenterPoint) GoSurfaceEllipse_CenterPoint(GoSurfaceEllipse tool)
{
    return GoTool_FindFeatureOutputByType(tool, GO_FEATURE_SURFACE_ELLIPSE_CENTER_POINT);
}

GoFx(GoSurfaceEllipseMajorAxisLine) GoSurfaceEllipse_MajorAxisLine(GoSurfaceEllipse tool)
{
    return GoTool_FindFeatureOutputByType(tool, GO_FEATURE_SURFACE_ELLIPSE_MAJOR_AXIS_LINE);
}

GoFx(GoSurfaceEllipseMinorAxisLine) GoSurfaceEllipse_MinorAxisLine(GoSurfaceEllipse tool)
{
    return GoTool_FindFeatureOutputByType(tool, GO_FEATURE_SURFACE_ELLIPSE_MINOR_AXIS_LINE);
}


kBeginClassEx(Go, GoSurfaceHole)
kAddVMethod(GoSurfaceHole, kObject, VRelease)
kAddVMethod(GoSurfaceHole, GoTool, VInit)
kAddVMethod(GoSurfaceHole, GoTool, VRead)
kAddVMethod(GoSurfaceHole, GoTool, VWrite)
kEndClassEx()

GoFx(kStatus) GoSurfaceHole_Construct(GoSurfaceHole* tool, kObject sensor, kAlloc allocator)
{
    return GoTool_Construct(tool, kTypeOf(GoSurfaceHole), sensor, allocator);
}

GoFx(kStatus) GoSurfaceHole_VInit(GoSurfaceHole tool, kType type, kObject sensor, kAlloc alloc)
{
    kObjR(GoSurfaceHole, tool);
    kSize i;

    kCheck(GoSurfaceTool_Init(tool, type, GO_TOOL_SURFACE_HOLE, sensor, alloc));
    obj->partialDetectionEnabled = kFALSE;
    obj->region = kNULL;
    obj->refRegionsEnabled = kFALSE;
    obj->refRegionCount = 0;
    kZero(obj->refRegions);
    obj->tiltXAngle = 0.0;
    obj->tiltYAngle = 0.0;

    obj->nominalRadius = 10.0;
    obj->radiusTolerance = 1.0;
    obj->regionEnabled = kTRUE;
    obj->autoTiltEnabled = kTRUE;
    obj->depthLimitEnabled = kFALSE;
    obj->depthLimit = 5.0;

    kCheck(GoRegion3d_Construct(&obj->region, sensor, alloc));

    for (i = 0; i < GO_SURFACE_HOLE_MAX_REF_REGIONS; i++)
    {
        kCheck(GoSurfaceRegion2d_Construct(&obj->refRegions[i], sensor, alloc));
    }

    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoSurfaceHoleX), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoSurfaceHoleY), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoSurfaceHoleZ), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoSurfaceHoleRadius), kTRUE, kNULL));

    return kOK;
}

GoFx(kStatus) GoSurfaceHole_VRelease(GoSurfaceHole tool)
{
    kObj(GoSurfaceHole, tool);
    kSize i;

    kDestroyRef(&obj->region);

    for (i = 0; i < GO_SURFACE_HOLE_MAX_REF_REGIONS; i++)
    {
        kDestroyRef(&obj->refRegions[i]);
    }

    return GoSurfaceTool_VRelease(tool);
}

GoFx(kStatus) GoSurfaceHole_VRead(GoSurfaceHole tool, kXml xml, kXmlItem item)
{
    kObj(GoSurfaceHole, tool);
    kSize i;
    kXmlItem regionItem = kNULL;
    kXmlItem refRegionsItem = kNULL;

    kCheck(kXml_Child64f(xml, item, "NominalRadius", &obj->nominalRadius));
    kCheck(kXml_Child64f(xml, item, "RadiusTolerance", &obj->radiusTolerance));
    kCheck(kXml_ChildBool(xml, item, "PartialDetectionEnabled", &obj->partialDetectionEnabled));

    kCheck(kXml_ChildBool(xml, item, "RegionEnabled", &obj->regionEnabled));
    kCheck(!kIsNull(regionItem = kXml_Child(xml, item, "Region")));
    kCheck(GoRegion3d_Read(obj->region, xml, regionItem));

    kCheck(kXml_ChildBool(xml, item, "RefRegionsEnabled", &obj->refRegionsEnabled));
    kCheck(kXml_ChildSize(xml, item, "RefRegionCount", &obj->refRegionCount));

    kCheck(!kIsNull(refRegionsItem = kXml_Child(xml, item, "RefRegions")));
    for (i = 0; i < GO_SURFACE_HOLE_MAX_REF_REGIONS; i++)
    {
        kCheck(!kIsNull(regionItem = kXml_ChildAt(xml, refRegionsItem, i)));
        kCheck(GoSurfaceRegion2d_Read(obj->refRegions[i], xml, regionItem));
    }

    kCheck(kXml_ChildBool(xml, item, "AutoTiltEnabled", &obj->autoTiltEnabled));
    kCheck(kXml_Child64f(xml, item, "TiltXAngle", &obj->tiltXAngle));
    kCheck(kXml_Child64f(xml, item, "TiltYAngle", &obj->tiltYAngle));

    kCheck(GoConfig_ReadBoolOptional(xml, item, "DepthLimitEnabled", kFALSE, &obj->depthLimitEnabled));
    kCheck(GoConfig_Read64fOptional(xml, item, "DepthLimit", 0.0, &obj->depthLimit));

    kCheck(GoSurfaceTool_Read(tool, xml, item));

    return kOK;
}

GoFx(kStatus) GoSurfaceHole_VWrite(GoSurfaceHole tool, kXml xml, kXmlItem item)
{
    kObj(GoSurfaceHole, tool);
    kXmlItem regionItem = kNULL;
    kXmlItem refRegionsItem = kNULL;
    kSize i;

    kCheck(kXml_SetChild64f(xml, item, "NominalRadius", obj->nominalRadius));
    kCheck(kXml_SetChild64f(xml, item, "RadiusTolerance", obj->radiusTolerance));
    kCheck(kXml_SetChildBool(xml, item, "PartialDetectionEnabled", obj->partialDetectionEnabled));

    kCheck(kXml_SetChildBool(xml, item, "RegionEnabled", obj->regionEnabled));
    kCheck(kXml_AddItem(xml, item, "Region", &regionItem));
    kCheck(GoRegion3d_Write(obj->region, xml, regionItem));

    kCheck(kXml_SetChildBool(xml, item, "RefRegionsEnabled", obj->refRegionsEnabled));
    kCheck(kXml_SetChildSize(xml, item, "RefRegionCount", obj->refRegionCount));

    kCheck(kXml_AddItem(xml, item, "RefRegions", &refRegionsItem));
    for (i = 0; i < GO_SURFACE_HOLE_MAX_REF_REGIONS; i++)
    {
        kCheck(kXml_AddItem(xml, refRegionsItem, "RefRegion", &regionItem));
        kCheck(GoSurfaceRegion2d_Write(obj->refRegions[i], xml, regionItem));
    }

    kCheck(kXml_SetChildBool(xml, item, "AutoTiltEnabled", obj->autoTiltEnabled));
    kCheck(kXml_SetChild64f(xml, item, "TiltXAngle", obj->tiltXAngle));
    kCheck(kXml_SetChild64f(xml, item, "TiltYAngle", obj->tiltYAngle));

    kCheck(kXml_SetChildBool(xml, item, "DepthLimitEnabled", obj->depthLimitEnabled));
    kCheck(kXml_SetChild64f(xml, item, "DepthLimit", obj->depthLimit));

    kCheck(GoSurfaceTool_Write(tool, xml, item));

    return kOK;
}

GoFx(k64f) GoSurfaceHole_NominalRadius(GoSurfaceHole tool)
{
    kObj(GoSurfaceHole, tool);

    return obj->nominalRadius;
}

GoFx(kStatus) GoSurfaceHole_SetNominalRadius(GoSurfaceHole tool, k64f nominalRadius)
{
    kObj(GoSurfaceHole, tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    obj->nominalRadius = nominalRadius;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}
GoFx(k64f) GoSurfaceHole_RadiusTolerance(GoSurfaceHole tool)
{
    kObj(GoSurfaceHole, tool);

    return obj->radiusTolerance;
}

GoFx(kStatus) GoSurfaceHole_SetRadiusTolerance(GoSurfaceHole tool, k64f radiusTolerance)
{
    kObj(GoSurfaceHole, tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    obj->radiusTolerance = radiusTolerance;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(kBool) GoSurfaceHole_PartialDetectionEnabled(GoSurfaceHole tool)
{
    kObj(GoSurfaceHole, tool);

    return obj->partialDetectionEnabled;
}

GoFx(kStatus) GoSurfaceHole_EnablePartialDetection(GoSurfaceHole tool, kBool enable)
{
    kObj(GoSurfaceHole, tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    obj->partialDetectionEnabled = enable;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(kBool) GoSurfaceHole_RegionEnabled(GoSurfaceHole tool)
{
    kObj(GoSurfaceHole, tool);

    return obj->regionEnabled;
}

GoFx(kStatus) GoSurfaceHole_EnableRegion(GoSurfaceHole tool, kBool enable)
{
    kObj(GoSurfaceHole, tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    obj->regionEnabled = enable;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(GoRegion3d) GoSurfaceHole_Region(GoSurfaceHole tool)
{
    kObj(GoSurfaceHole, tool);

    return obj->region;
}

GoFx(kBool) GoSurfaceHole_RefRegionsEnabled(GoSurfaceHole tool)
{
    kObj(GoSurfaceHole, tool);

    return obj->refRegionsEnabled;
}

GoFx(kStatus) GoSurfaceHole_EnableRefRegions(GoSurfaceHole tool, kBool enable)
{
    kObj(GoSurfaceHole, tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    obj->refRegionsEnabled = enable;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(kSize) GoSurfaceHole_RefRegionCount(GoSurfaceHole tool)
{
    kObj(GoSurfaceHole, tool);

    return obj->refRegionCount;
}

GoFx(kStatus) GoSurfaceHole_SetRefRegionCount(GoSurfaceHole tool, kSize count)
{
    kObj(GoSurfaceHole, tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    kCheckArgs(count <= GO_SURFACE_HOLE_MAX_REF_REGIONS);
    obj->refRegionCount = count;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(GoSurfaceRegion2d) GoSurfaceHole_RefRegionAt(GoSurfaceHole tool, kSize index)
{
    kObj(GoSurfaceHole, tool);

    kAssert(index < GO_SURFACE_HOLE_MAX_REF_REGIONS);

    return obj->refRegions[index];
}

GoFx(kBool) GoSurfaceHole_AutoTiltEnabled(GoSurfaceHole tool)
{
    kObj(GoSurfaceHole, tool);

    return obj->autoTiltEnabled;
}

GoFx(kStatus) GoSurfaceHole_EnableAutoTilt(GoSurfaceHole tool, kBool enable)
{
    kObj(GoSurfaceHole, tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    obj->autoTiltEnabled = enable;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(k64f) GoSurfaceHole_TiltXAngle(GoSurfaceHole tool)
{
    kObj(GoSurfaceHole, tool);

    return obj->tiltXAngle;
}

GoFx(kStatus) GoSurfaceHole_SetTiltXAngle(GoSurfaceHole tool, k64f value)
{
    kObj(GoSurfaceHole, tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    obj->tiltXAngle = value;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(k64f) GoSurfaceHole_TiltYAngle(GoSurfaceHole tool)
{
    kObj(GoSurfaceHole, tool);

    return obj->tiltYAngle;
}

GoFx(kStatus) GoSurfaceHole_SetTiltYAngle(GoSurfaceHole tool, k64f value)
{
    kObj(GoSurfaceHole, tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    obj->tiltYAngle = value;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(kBool) GoSurfaceHole_DepthLimitEnabled(GoSurfaceHole tool)
{
    kObj(GoSurfaceHole, tool);

    return obj->depthLimitEnabled;
}

GoFx(kStatus) GoSurfaceHole_EnableDepthLimit(GoSurfaceHole tool, kBool enable)
{
    kObj(GoSurfaceHole, tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    obj->depthLimitEnabled = enable;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(k64f) GoSurfaceHole_DepthLimit(GoSurfaceHole tool)
{
    kObj(GoSurfaceHole, tool);

    return obj->depthLimit;
}

GoFx(kStatus) GoSurfaceHole_SetDepthLimit(GoSurfaceHole tool, k64f value)
{
    kObj(GoSurfaceHole, tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    obj->depthLimit = value;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(GoSurfaceHoleX) GoSurfaceHole_XMeasurement(GoSurfaceHole tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_SURFACE_HOLE_X);
}

GoFx(GoSurfaceHoleY) GoSurfaceHole_YMeasurement(GoSurfaceHole tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_SURFACE_HOLE_Y);
}

GoFx(GoSurfaceHoleZ) GoSurfaceHole_ZMeasurement(GoSurfaceHole tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_SURFACE_HOLE_Z);
}

GoFx(GoSurfaceHoleRadius) GoSurfaceHole_RadiusMeasurement(GoSurfaceHole tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_SURFACE_HOLE_RADIUS);
}

GoFx(GoSurfaceHoleCenterPoint) GoSurfaceHole_Point(GoSurfaceHole tool)
{
    return GoTool_FindFeatureOutputByType(tool, GO_FEATURE_SURFACE_HOLE_CENTER_POINT);
}



kBeginClassEx(Go, GoSurfaceOpening)
kAddVMethod(GoSurfaceOpening, kObject, VRelease)
kAddVMethod(GoSurfaceOpening, GoTool, VInit)
kAddVMethod(GoSurfaceOpening, GoTool, VRead)
kAddVMethod(GoSurfaceOpening, GoTool, VWrite)
kEndClassEx()

GoFx(kStatus) GoSurfaceOpening_Construct(GoSurfaceOpening* tool, kObject sensor, kAlloc allocator)
{
    return GoTool_Construct(tool, kTypeOf(GoSurfaceOpening), sensor, allocator);
}

GoFx(kStatus) GoSurfaceOpening_VInit(GoSurfaceOpening tool, kType type, kObject sensor, kAlloc alloc)
{
    kObjR(GoSurfaceOpening, tool);
    kSize i;

    kCheck(GoSurfaceTool_Init(tool, type, GO_TOOL_SURFACE_OPENING, sensor, alloc));
    obj->nominalAngle = 0.0;
    obj->partialDetectionEnabled = kFALSE;
    obj->region = kNULL;
    obj->refRegionsEnabled = kFALSE;
    obj->refRegionCount = 0;
    kZero(obj->refRegions);
    obj->tiltXAngle = 0.0;
    obj->tiltYAngle = 0.0;

    obj->nominalWidth = 10.0;
    obj->nominalLength = 20.0;
    obj->nominalRadius = 5.0;
    obj->widthTolerance = 2.0;
    obj->lengthTolerance = 4.0;
    obj->angleTolerance = 5.0;
    obj->regionEnabled = kTRUE;
    obj->autoTiltEnabled = kTRUE;
    obj->type = GO_SURFACE_OPENING_TYPE_ROUNDED_SLOT;
    obj->depthLimitEnabled = kFALSE;
    obj->depthLimit = 5.0;

    kCheck(GoRegion3d_Construct(&obj->region, sensor, alloc));

    for (i = 0; i < GO_SURFACE_OPENING_MAX_REF_REGIONS; i++)
    {
        kCheck(GoSurfaceRegion2d_Construct(&obj->refRegions[i], sensor, alloc));
    }

    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoSurfaceOpeningX), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoSurfaceOpeningY), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoSurfaceOpeningZ), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoSurfaceOpeningWidth), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoSurfaceOpeningLength), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoSurfaceOpeningAngle), kTRUE, kNULL));

    return kOK;
}

GoFx(kStatus) GoSurfaceOpening_VRelease(GoSurfaceOpening tool)
{
    kObj(GoSurfaceOpening, tool);
    kSize i;

    kDestroyRef(&obj->region);

    for (i = 0; i < GO_SURFACE_OPENING_MAX_REF_REGIONS; i++)
    {
        kDestroyRef(&obj->refRegions[i]);
    }

    return GoSurfaceTool_VRelease(tool);
}

GoFx(kStatus) GoSurfaceOpening_VRead(GoSurfaceOpening tool, kXml xml, kXmlItem item)
{
    kObj(GoSurfaceOpening, tool);
    kSize i;
    kXmlItem regionItem = kNULL;
    kXmlItem refRegionsItem = kNULL;

    kCheck(kXml_Child32s(xml, item, "Type", &obj->type));

    kCheck(kXml_Child64f(xml, item, "NominalWidth", &obj->nominalWidth));
    kCheck(kXml_Child64f(xml, item, "NominalLength", &obj->nominalLength));
    kCheck(kXml_Child64f(xml, item, "NominalAngle", &obj->nominalAngle));
    kCheck(kXml_Child64f(xml, item, "NominalRadius", &obj->nominalRadius));

    kCheck(kXml_Child64f(xml, item, "WidthTolerance", &obj->widthTolerance));
    kCheck(kXml_Child64f(xml, item, "LengthTolerance", &obj->lengthTolerance));
    kCheck(kXml_Child64f(xml, item, "AngleTolerance", &obj->angleTolerance));

    kCheck(kXml_ChildBool(xml, item, "PartialDetectionEnabled", &obj->partialDetectionEnabled));

    kCheck(kXml_ChildBool(xml, item, "RegionEnabled", &obj->regionEnabled));

    kCheck(!kIsNull(regionItem = kXml_Child(xml, item, "Region")));
    kCheck(GoRegion3d_Read(obj->region, xml, regionItem));

    kCheck(kXml_ChildBool(xml, item, "RefRegionsEnabled", &obj->refRegionsEnabled));
    kCheck(kXml_ChildSize(xml, item, "RefRegionCount", &obj->refRegionCount));

    kCheck(!kIsNull(refRegionsItem = kXml_Child(xml, item, "RefRegions")));
    for (i = 0; i < GO_SURFACE_HOLE_MAX_REF_REGIONS; i++)
    {
        kCheck(!kIsNull(regionItem = kXml_ChildAt(xml, refRegionsItem, i)));
        kCheck(GoSurfaceRegion2d_Read(obj->refRegions[i], xml, regionItem));
    }

    kCheck(kXml_ChildBool(xml, item, "AutoTiltEnabled", &obj->autoTiltEnabled));
    kCheck(kXml_Child64f(xml, item, "TiltXAngle", &obj->tiltXAngle));
    kCheck(kXml_Child64f(xml, item, "TiltYAngle", &obj->tiltYAngle));

    kCheck(GoConfig_ReadBoolOptional(xml, item, "DepthLimitEnabled", kFALSE, &obj->depthLimitEnabled));
    kCheck(GoConfig_Read64fOptional(xml, item, "DepthLimit", 0.0, &obj->depthLimit));

    kCheck(GoSurfaceTool_Read(tool, xml, item));

    return kOK;
}

GoFx(kStatus) GoSurfaceOpening_VWrite(GoSurfaceOpening tool, kXml xml, kXmlItem item)
{
    kObj(GoSurfaceOpening, tool);
    kXmlItem regionItem = kNULL;
    kXmlItem refRegionsItem = kNULL;
    kSize i;

    kCheck(kXml_SetChild32s(xml, item, "Type", obj->type));

    kCheck(kXml_SetChild64f(xml, item, "NominalWidth", obj->nominalWidth));
    kCheck(kXml_SetChild64f(xml, item, "NominalLength", obj->nominalLength));
    kCheck(kXml_SetChild64f(xml, item, "NominalAngle", obj->nominalAngle));
    kCheck(kXml_SetChild64f(xml, item, "NominalRadius", obj->nominalRadius));

    kCheck(kXml_SetChild64f(xml, item, "WidthTolerance", obj->widthTolerance));
    kCheck(kXml_SetChild64f(xml, item, "LengthTolerance", obj->lengthTolerance));
    kCheck(kXml_SetChild64f(xml, item, "AngleTolerance", obj->angleTolerance));

    kCheck(kXml_SetChildBool(xml, item, "PartialDetectionEnabled", obj->partialDetectionEnabled));

    kCheck(kXml_SetChildBool(xml, item, "RegionEnabled", obj->regionEnabled));
    kCheck(kXml_AddItem(xml, item, "Region", &regionItem));
    kCheck(GoRegion3d_Write(obj->region, xml, regionItem));

    kCheck(kXml_SetChildBool(xml, item, "RefRegionsEnabled", obj->refRegionsEnabled));
    kCheck(kXml_SetChildSize(xml, item, "RefRegionCount", obj->refRegionCount));

    kCheck(kXml_AddItem(xml, item, "RefRegions", &refRegionsItem));
    for (i = 0; i < GO_SURFACE_HOLE_MAX_REF_REGIONS; i++)
    {
        kCheck(kXml_AddItem(xml, refRegionsItem, "RefRegion", &regionItem));
        kCheck(GoSurfaceRegion2d_Write(obj->refRegions[i], xml, regionItem));
    }

    kCheck(kXml_SetChildBool(xml, item, "AutoTiltEnabled", obj->autoTiltEnabled));
    kCheck(kXml_SetChild64f(xml, item, "TiltXAngle", obj->tiltXAngle));
    kCheck(kXml_SetChild64f(xml, item, "TiltYAngle", obj->tiltYAngle));

    kCheck(kXml_SetChildBool(xml, item, "DepthLimitEnabled", obj->depthLimitEnabled));
    kCheck(kXml_SetChild64f(xml, item, "DepthLimit", obj->depthLimit));

    kCheck(GoSurfaceTool_Write(tool, xml, item));

    return kOK;
}

GoFx(kStatus) GoSurfaceOpening_SetType(GoSurfaceOpening tool, GoSurfaceOpeningType type)
{
    kObj(GoSurfaceOpening, tool);

    obj->type = type;

    return kOK;
}

GoFx(GoSurfaceOpeningType) GoSurfaceOpening_Type(GoSurfaceOpening tool)
{
    kObj(GoSurfaceOpening, tool);

    return obj->type;
}

GoFx(k64f) GoSurfaceOpening_NominalWidth(GoSurfaceOpening tool)
{
    kObj(GoSurfaceOpening, tool);

    return obj->nominalWidth;
}

GoFx(kStatus) GoSurfaceOpening_SetNominalWidth(GoSurfaceOpening tool, k64f value)
{
    kObj(GoSurfaceOpening, tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    obj->nominalWidth = value;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(k64f) GoSurfaceOpening_NominalLength(GoSurfaceOpening tool)
{
    kObj(GoSurfaceOpening, tool);

    return obj->nominalLength;
}

GoFx(kStatus) GoSurfaceOpening_SetNominalLength(GoSurfaceOpening tool, k64f value)
{
    kObj(GoSurfaceOpening, tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    obj->nominalLength = value;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(k64f) GoSurfaceOpening_NominalAngle(GoSurfaceOpening tool)
{
    kObj(GoSurfaceOpening, tool);

    return obj->nominalAngle;
}

GoFx(kStatus) GoSurfaceOpening_SetNominalAngle(GoSurfaceOpening tool, k64f value)
{
    kObj(GoSurfaceOpening, tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    obj->nominalAngle = value;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(k64f) GoSurfaceOpening_NominalRadius(GoSurfaceOpening tool)
{
    kObj(GoSurfaceOpening, tool);

    return obj->nominalRadius;
}

GoFx(kStatus) GoSurfaceOpening_SetNominalRadius(GoSurfaceOpening tool, k64f value)
{
    kObj(GoSurfaceOpening, tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    obj->nominalRadius = value;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(k64f) GoSurfaceOpening_WidthTolerance(GoSurfaceOpening tool)
{
    kObj(GoSurfaceOpening, tool);

    return obj->widthTolerance;
}

GoFx(kStatus) GoSurfaceOpening_SetWidthTolerance(GoSurfaceOpening tool, k64f value)
{
    kObj(GoSurfaceOpening, tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    obj->widthTolerance = value;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(k64f) GoSurfaceOpening_LengthTolerance(GoSurfaceOpening tool)
{
    kObj(GoSurfaceOpening, tool);

    return obj->lengthTolerance;
}

GoFx(kStatus) GoSurfaceOpening_SetLengthTolerance(GoSurfaceOpening tool, k64f value)
{
    kObj(GoSurfaceOpening, tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    obj->lengthTolerance = value;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(k64f) GoSurfaceOpening_AngleTolerance(GoSurfaceOpening tool)
{
    kObj(GoSurfaceOpening, tool);

    return obj->angleTolerance;
}

GoFx(kStatus) GoSurfaceOpening_SetAngleTolerance(GoSurfaceOpening tool, k64f value)
{
    kObj(GoSurfaceOpening, tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    obj->angleTolerance = value;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(kBool) GoSurfaceOpening_PartialDetectionEnabled(GoSurfaceOpening tool)
{
    kObj(GoSurfaceOpening, tool);

    return obj->partialDetectionEnabled;
}

GoFx(kStatus) GoSurfaceOpening_EnablePartialDetection(GoSurfaceOpening tool, kBool enable)
{
    kObj(GoSurfaceOpening, tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    obj->partialDetectionEnabled = enable;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(kBool) GoSurfaceOpening_RegionEnabled(GoSurfaceOpening tool)
{
    kObj(GoSurfaceOpening, tool);

    return obj->regionEnabled;
}

GoFx(kStatus) GoSurfaceOpening_EnableRegion(GoSurfaceOpening tool, kBool enable)
{
    kObj(GoSurfaceOpening, tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    obj->regionEnabled = enable;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(GoRegion3d) GoSurfaceOpening_Region(GoSurfaceOpening tool)
{
    kObj(GoSurfaceOpening, tool);

    return obj->region;
}


GoFx(kBool) GoSurfaceOpening_RefRegionsEnabled(GoSurfaceOpening tool)
{
    kObj(GoSurfaceOpening, tool);

    return obj->refRegionsEnabled;
}

GoFx(kStatus) GoSurfaceOpening_EnableRefRegions(GoSurfaceOpening tool, kBool enable)
{
    kObj(GoSurfaceOpening, tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    obj->refRegionsEnabled = enable;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(kSize) GoSurfaceOpening_RefRegionCount(GoSurfaceOpening tool)
{
    kObj(GoSurfaceOpening, tool);

    return obj->refRegionCount;
}

GoFx(kStatus) GoSurfaceOpening_SetRefRegionCount(GoSurfaceOpening tool, kSize count)
{
    kObj(GoSurfaceOpening, tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    kCheckArgs(count <= GO_SURFACE_OPENING_MAX_REF_REGIONS);
    obj->refRegionCount = count;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(GoSurfaceRegion2d) GoSurfaceOpening_RefRegionAt(GoSurfaceOpening tool, kSize index)
{
    kObj(GoSurfaceOpening, tool);

    kAssert(index < GO_SURFACE_OPENING_MAX_REF_REGIONS);

    return obj->refRegions[index];
}

GoFx(kBool) GoSurfaceOpening_AutoTiltEnabled(GoSurfaceOpening tool)
{
    kObj(GoSurfaceOpening, tool);

    return obj->autoTiltEnabled;
}

GoFx(kStatus) GoSurfaceOpening_EnableAutoTilt(GoSurfaceOpening tool, kBool enable)
{
    kObj(GoSurfaceOpening, tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    obj->autoTiltEnabled = enable;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(k64f) GoSurfaceOpening_TiltXAngle(GoSurfaceOpening tool)
{
    kObj(GoSurfaceOpening, tool);

    return obj->tiltXAngle;
}

GoFx(kStatus) GoSurfaceOpening_SetTiltXAngle(GoSurfaceOpening tool, k64f value)
{
    kObj(GoSurfaceOpening, tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    obj->tiltXAngle = value;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(k64f) GoSurfaceOpening_TiltYAngle(GoSurfaceOpening tool)
{
    kObj(GoSurfaceOpening, tool);

    return obj->tiltYAngle;
}

GoFx(kStatus) GoSurfaceOpening_SetTiltYAngle(GoSurfaceOpening tool, k64f value)
{
    kObj(GoSurfaceOpening, tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    obj->tiltYAngle = value;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(kBool) GoSurfaceOpening_DepthLimitEnabled(GoSurfaceOpening tool)
{
    kObj(GoSurfaceOpening, tool);

    return obj->depthLimitEnabled;
}

GoFx(kStatus) GoSurfaceOpening_EnableDepthLimit(GoSurfaceOpening tool, kBool enable)
{
    kObj(GoSurfaceOpening, tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    obj->depthLimitEnabled = enable;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(k64f) GoSurfaceOpening_DepthLimit(GoSurfaceOpening tool)
{
    kObj(GoSurfaceOpening, tool);

    return obj->depthLimit;
}

GoFx(kStatus) GoSurfaceOpening_SetDepthLimit(GoSurfaceOpening tool, k64f value)
{
    kObj(GoSurfaceOpening, tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    obj->depthLimit = value;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(GoSurfaceOpeningX) GoSurfaceOpening_XMeasurement(GoSurfaceOpening tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_SURFACE_OPENING_X);
}

GoFx(GoSurfaceOpeningY) GoSurfaceOpening_YMeasurement(GoSurfaceOpening tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_SURFACE_OPENING_Y);
}

GoFx(GoSurfaceOpeningZ) GoSurfaceOpening_ZMeasurement(GoSurfaceOpening tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_SURFACE_OPENING_Z);
}

GoFx(GoSurfaceOpeningWidth) GoSurfaceOpening_WidthMeasurement(GoSurfaceOpening tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_SURFACE_OPENING_WIDTH);
}

GoFx(GoSurfaceOpeningLength) GoSurfaceOpening_LengthMeasurement(GoSurfaceOpening tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_SURFACE_OPENING_LENGTH);
}

GoFx(GoSurfaceOpeningAngle) GoSurfaceOpening_AngleMeasurement(GoSurfaceOpening tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_SURFACE_OPENING_ANGLE);
}

GoFx(GoSurfaceOpeningCenterPoint) GoSurfaceOpening_CenterPoint(GoSurfaceHole tool)
{
    return GoTool_FindFeatureOutputByType(tool, GO_FEATURE_SURFACE_OPENING_CENTER_POINT);
}



kBeginClassEx(Go, GoSurfacePlane)
kAddVMethod(GoSurfacePlane, kObject, VRelease)
kAddVMethod(GoSurfacePlane, GoTool, VInit)
kAddVMethod(GoSurfacePlane, GoTool, VRead)
kAddVMethod(GoSurfacePlane, GoTool, VWrite)
kEndClassEx()

GoFx(kStatus) GoSurfacePlane_Construct(GoSurfacePlane* tool, kObject sensor, kAlloc allocator)
{
    return GoTool_Construct(tool, kTypeOf(GoSurfacePlane), sensor, allocator);
}

GoFx(kStatus) GoSurfacePlane_VInit(GoSurfacePlane tool, kType type, kObject sensor, kAlloc alloc)
{
    kObjR(GoSurfacePlane, tool);
    kSize i;

    kCheck(GoSurfaceTool_Init(tool, type, GO_TOOL_SURFACE_PLANE, sensor, alloc));
    kZero(obj->regions);

    obj->regionsEnabled = kTRUE;
    obj->regionCount = 1;

    for (i = 0; i < GO_SURFACE_PLANE_MAX_REGIONS; i++)
    {
        kCheck(GoRegion3d_Construct(&obj->regions[i], sensor, alloc));
    }

    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoSurfacePlaneXAngle), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoSurfacePlaneYAngle), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoSurfacePlaneZOffset), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoSurfacePlaneStdDev), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoSurfacePlaneMinError), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoSurfacePlaneMaxError), kTRUE, kNULL));

    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoSurfacePlaneXNormal), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoSurfacePlaneYNormal), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoSurfacePlaneZNormal), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoSurfacePlaneDistance), kTRUE, kNULL));



    return kOK;
}

GoFx(kStatus) GoSurfacePlane_VRelease(GoSurfacePlane tool)
{
    kObj(GoSurfacePlane, tool);
    kSize i;

    for (i = 0; i < GO_SURFACE_PLANE_MAX_REGIONS; i++)
    {
        kDestroyRef(&obj->regions[i]);
    }

    return GoSurfaceTool_VRelease(tool);
}

GoFx(kStatus) GoSurfacePlane_VRead(GoSurfacePlane tool, kXml xml, kXmlItem item)
{
    kObj(GoSurfacePlane, tool);
    kSize i;
    kXmlItem regionsItem = kNULL;
    kXmlItem regionItem = kNULL;

    kCheck(kXml_ChildBool(xml, item, "RegionsEnabled", &obj->regionsEnabled));
    kCheck(kXml_ChildSize(xml, item, "RegionCount", &obj->regionCount));

    // For compatibility with different sensor software versions which may
    // support more regions than the SDK can support, limit
    // max number of regions to the number supported by the SDK code.
    if (obj->regionCount > GO_SURFACE_PLANE_MAX_REGIONS)
    {
        obj->regionCount = GO_SURFACE_PLANE_MAX_REGIONS;
    }

    kCheck(!kIsNull(regionsItem = kXml_Child(xml, item, "Regions")));

    // The XML file contains a persistent record of region parameters, even
    // if the region is not selected. These parameters are expected to be
    // available again when the region is selected, so it is important
    // to store _all_ the region parameters in the tool object, up to the
    // maximum number of regions supported by the SDK software.
    // The excess region data from the XML file are discarded.
    // NOTE: the region count is the number of regions in use, and is not
    //       the total number of regions supported.
    for (i = 0; i < GO_SURFACE_PLANE_MAX_REGIONS; i++)
    {
        // It is an error if there are less regions than the region count.
        if (kIsNull(regionItem = kXml_ChildAt(xml, regionsItem, i)))
        {
            // There are fewer regions in the XML file than is supported by
            // the sensor. This is a supported scenario. The extra regions
            // in the SDK software will be all set to zeroes.
            break;
        }
        kCheck(GoRegion3d_Read(obj->regions[i], xml, regionItem));
    }

    kCheck(GoSurfaceTool_Read(tool, xml, item));

    return kOK;
}

GoFx(kStatus) GoSurfacePlane_VWrite(GoSurfacePlane tool, kXml xml, kXmlItem item)
{
    kObj(GoSurfacePlane, tool);
    kSize i;
    kXmlItem regionsItem = kNULL;
    kXmlItem regionItem = kNULL;

    kCheck(kXml_SetChildBool(xml, item, "RegionsEnabled", obj->regionsEnabled));
    kCheck(kXml_SetChildSize(xml, item, "RegionCount", obj->regionCount));

    kCheck(kXml_AddItem(xml, item, "Regions", &regionsItem));

    // For backwards compatibility with older sensor firmware that expected
    // the maximum number of supported regions to be in the XML configuration
    // file, and therefore read that number of regions, always write the
    // maximum supported regions into the configuration file.
    for (i = 0; i < GO_SURFACE_PLANE_MAX_REGIONS; i++)
    {
        kCheck(kXml_AddItem(xml, regionsItem, "Region", &regionItem));
        kCheck(GoRegion3d_Write(obj->regions[i], xml, regionItem));
    }

    kCheck(GoSurfaceTool_Write(tool, xml, item));

    return kOK;
}

GoFx(kBool) GoSurfacePlane_RegionsEnabled(GoSurfacePlane tool)
{
    kObj(GoSurfacePlane, tool);

    return obj->regionsEnabled;
}

GoFx(kStatus) GoSurfacePlane_EnableRegions(GoSurfacePlane tool, kBool enable)
{
    kObj(GoSurfacePlane, tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    obj->regionsEnabled = enable;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(kSize) GoSurfacePlane_RegionCount(GoSurfacePlane tool)
{
    kObj(GoSurfacePlane, tool);

    return obj->regionCount;
}

GoFx(kStatus) GoSurfacePlane_SetRegionCount(GoSurfacePlane tool, kSize count)
{
    kObj(GoSurfacePlane, tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));

    kCheckArgs(count <= GO_SURFACE_PLANE_MAX_REGIONS);
    obj->regionCount = count;

    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(GoRegion3d) GoSurfacePlane_RegionAt(GoSurfacePlane tool, kSize index)
{
    kObj(GoSurfacePlane, tool);

    kAssert(index < GO_SURFACE_PLANE_MAX_REGIONS);

    return obj->regions[index];
}

GoFx(GoSurfacePlaneXAngle) GoSurfacePlane_XAngleMeasurement(GoSurfacePlane tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_SURFACE_PLANE_X_ANGLE);
}

GoFx(GoSurfacePlaneYAngle) GoSurfacePlane_YAngleMeasurement(GoSurfacePlane tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_SURFACE_PLANE_Y_ANGLE);
}

GoFx(GoSurfacePlaneZOffset) GoSurfacePlane_ZOffsetMeasurement(GoSurfacePlane tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_SURFACE_PLANE_Z_OFFSET);
}

GoFx(GoSurfacePlaneStdDev) GoSurfacePlane_StdDevMeasurement(GoSurfacePlane tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_SURFACE_PLANE_STD_DEV);
}

GoFx(GoSurfacePlaneMinError) GoSurfacePlane_MinErrorMeasurement(GoSurfacePlane tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_SURFACE_PLANE_ERROR_MIN);
}

GoFx(GoSurfacePlaneMaxError) GoSurfacePlane_MaxErrorMeasurement(GoSurfacePlane tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_SURFACE_PLANE_ERROR_MAX);
}

GoFx(GoSurfacePlaneXNormal) GoSurfacePlane_XNormalMeasurement(GoSurfacePlane tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_SURFACE_PLANE_X_NORMAL);
}

GoFx(GoSurfacePlaneYNormal) GoSurfacePlane_YNormalMeasurement(GoSurfacePlane tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_SURFACE_PLANE_Y_NORMAL);
}

GoFx(GoSurfacePlaneZNormal) GoSurfacePlane_ZNormalMeasurement(GoSurfacePlane tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_SURFACE_PLANE_Z_NORMAL);
}

GoFx(GoSurfacePlaneDistance) GoSurfacePlane_DistanceMeasurement(GoSurfacePlane tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_SURFACE_PLANE_DISTANCE);
}

GoFx(GoSurfacePlanePlane) GoSurfacePlane_Plane(GoSurfacePlane tool)
{
    return GoTool_FindFeatureOutputByType(tool, GO_FEATURE_SURFACE_PLANE_PLANE);
}

kBeginClassEx(Go, GoSurfacePosition)
kAddVMethod(GoSurfacePosition, kObject, VRelease)
kAddVMethod(GoSurfacePosition, GoTool, VInit)
kAddVMethod(GoSurfacePosition, GoTool, VRead)
kAddVMethod(GoSurfacePosition, GoTool, VWrite)
kEndClassEx()

GoFx(kStatus) GoSurfacePosition_Construct(GoSurfacePosition* tool, kObject sensor, kAlloc allocator)
{
    return GoTool_Construct(tool, kTypeOf(GoSurfacePosition), sensor, allocator);
}

GoFx(kStatus) GoSurfacePosition_VInit(GoSurfacePosition tool, kType type, kObject sensor, kAlloc alloc)
{
    kObjR(GoSurfacePosition, tool);

    kCheck(GoSurfaceTool_Init(tool, type, GO_TOOL_SURFACE_POSITION, sensor, alloc));
    obj->feature = kNULL;
    obj->regionEnabled = kFALSE;
    obj->region = kNULL;

    kCheck(GoSurfaceFeature_Construct(&obj->feature, sensor, alloc));

    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoSurfacePositionX), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoSurfacePositionY), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoSurfacePositionZ), kTRUE, kNULL));

    return kOK;
}

GoFx(kStatus) GoSurfacePosition_VRelease(GoSurfacePosition tool)
{
    kObj(GoSurfacePosition, tool);

    kCheck(kDestroyRef(&obj->feature));

    return GoSurfaceTool_VRelease(tool);
}

GoFx(kStatus) GoSurfacePosition_VRead(GoSurfacePosition tool, kXml xml, kXml item)
{
    kObj(GoSurfacePosition, tool);
    kXmlItem featureItem = kNULL;

    kCheck(!kIsNull(featureItem = kXml_Child(xml, item, "Feature")));
    kCheck(GoSurfaceFeature_Read(obj->feature, xml, featureItem));

    kCheck(GoSurfaceTool_Read(tool, xml, item));

    return kOK;
}

GoFx(kStatus) GoSurfacePosition_VWrite(GoSurfacePosition tool, kXml xml, kXml item)
{
    kObj(GoSurfacePosition, tool);
    kXmlItem featureItem = kNULL;

    kCheck(kXml_AddItem(xml, item, "Feature", &featureItem));
    kCheck(GoSurfaceFeature_Write(obj->feature, xml, featureItem));

    kCheck(GoSurfaceTool_Write(tool, xml, item));

    return kOK;
}

GoFx(GoSurfaceFeature) GoSurfacePosition_Feature(GoSurfacePosition tool)
{
    kObj(GoSurfacePosition, tool);

    return obj->feature;
}

GoFx(GoSurfacePositionX) GoSurfacePosition_XMeasurement(GoSurfacePosition tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_SURFACE_POSITION_X);
}

GoFx(GoSurfacePositionY) GoSurfacePosition_YMeasurement(GoSurfacePosition tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_SURFACE_POSITION_Y);
}

GoFx(GoSurfacePositionZ) GoSurfacePosition_ZMeasurement(GoSurfacePosition tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_SURFACE_POSITION_Z);
}

GoFx(GoSurfacePositionPoint) GoSurfacePosition_Point(GoSurfacePosition tool)
{
    return GoTool_FindFeatureOutputByType(tool, GO_FEATURE_SURFACE_POSITION_POINT);
}

kBeginClassEx(Go, GoSurfaceStud)
kAddVMethod(GoSurfaceStud, kObject, VRelease)
kAddVMethod(GoSurfaceStud, GoTool, VInit)
kAddVMethod(GoSurfaceStud, GoTool, VRead)
kAddVMethod(GoSurfaceStud, GoTool, VWrite)
kEndClassEx()

GoFx(kStatus) GoSurfaceStud_Construct(GoSurfaceStud* tool, kObject sensor, kAlloc allocator)
{
    return GoTool_Construct(tool, kTypeOf(GoSurfaceStud), sensor, allocator);
}

GoFx(kStatus) GoSurfaceStud_VInit(GoSurfaceStud tool, kType type, kObject sensor, kAlloc alloc)
{
    kObjR(GoSurfaceStud, tool);
    kSize i;

    kCheck(GoSurfaceTool_Init(tool, type, GO_TOOL_SURFACE_STUD, sensor, alloc));
    obj->baseHeight = 0.0;
    obj->tipHeight = 0.0;
    obj->region = kNULL;
    obj->refRegionsEnabled = kFALSE;
    obj->refRegionCount = 0;
    kZero(obj->refRegions);
    obj->tiltXAngle = 0.0;
    obj->tiltYAngle = 0.0;

    obj->studRadius = 5.0;
    obj->studHeight = 20.0;
    obj->regionEnabled = kTRUE;
    obj->autoTiltEnabled = kTRUE;

    kCheck(GoRegion3d_Construct(&obj->region, sensor, alloc));

    for (i = 0; i < GO_SURFACE_STUD_MAX_REF_REGIONS; i++)
    {
        kCheck(GoSurfaceRegion2d_Construct(&obj->refRegions[i], sensor, alloc));
    }

    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoSurfaceStudBaseX), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoSurfaceStudBaseY), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoSurfaceStudBaseZ), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoSurfaceStudTipX), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoSurfaceStudTipY), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoSurfaceStudTipZ), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoSurfaceStudRadius), kTRUE, kNULL));

    return kOK;
}

GoFx(kStatus) GoSurfaceStud_VRelease(GoSurfaceStud tool)
{
    kObj(GoSurfaceStud, tool);
    kSize i;

    kDestroyRef(&obj->region);

    for (i = 0; i < GO_SURFACE_STUD_MAX_REF_REGIONS; i++)
    {
        kDestroyRef(&obj->refRegions[i]);
    }

    return GoSurfaceTool_VRelease(tool);
}

GoFx(kStatus) GoSurfaceStud_VRead(GoSurfaceStud tool, kXml xml, kXmlItem item)
{
    kObj(GoSurfaceStud, tool);
    kSize i;
    kXmlItem refRegionsItem = kNULL;
    kXmlItem regionItem = kNULL;

    kCheck(kXml_Child64f(xml, item, "StudRadius", &obj->studRadius));
    kCheck(kXml_Child64f(xml, item, "StudHeight", &obj->studHeight));
    kCheck(kXml_Child64f(xml, item, "BaseHeight", &obj->baseHeight));
    kCheck(kXml_Child64f(xml, item, "TipHeight", &obj->tipHeight));

    kCheck(kXml_ChildBool(xml, item, "RegionEnabled", &obj->regionEnabled));
    kCheck(!kIsNull(regionItem = kXml_Child(xml, item, "Region")));
    kCheck(GoRegion3d_Read(obj->region, xml, regionItem));

    kCheck(kXml_ChildBool(xml, item, "RefRegionsEnabled", &obj->refRegionsEnabled));
    kCheck(kXml_ChildSize(xml, item, "RefRegionCount", &obj->refRegionCount));

    kCheck(!kIsNull(refRegionsItem = kXml_Child(xml, item, "RefRegions")));
    kCheck(kXml_ChildCount(xml, refRegionsItem) == GO_SURFACE_STUD_MAX_REF_REGIONS);

    for (i = 0; i < GO_SURFACE_STUD_MAX_REF_REGIONS; i++)
    {
        kCheck(!kIsNull(regionItem = kXml_Child(xml, refRegionsItem, "RefRegion")));
        kCheck(GoSurfaceRegion2d_Read(obj->refRegions[i], xml, regionItem));
    }

    kCheck(kXml_ChildBool(xml, item, "AutoTiltEnabled", &obj->autoTiltEnabled));
    kCheck(kXml_Child64f(xml, item, "TiltXAngle", &obj->tiltXAngle));
    kCheck(kXml_Child64f(xml, item, "TiltYAngle", &obj->tiltYAngle));

    kCheck(GoSurfaceTool_Read(tool, xml, item));

    return kOK;
}

GoFx(kStatus) GoSurfaceStud_VWrite(GoSurfaceStud tool, kXml xml, kXmlItem item)
{
    kObj(GoSurfaceStud, tool);
    kSize i;
    kXmlItem refRegionsItem = kNULL;
    kXmlItem regionItem = kNULL;

    kCheck(kXml_SetChild64f(xml, item, "StudRadius", obj->studRadius));
    kCheck(kXml_SetChild64f(xml, item, "StudHeight", obj->studHeight));
    kCheck(kXml_SetChild64f(xml, item, "BaseHeight", obj->baseHeight));
    kCheck(kXml_SetChild64f(xml, item, "TipHeight", obj->tipHeight));

    kCheck(kXml_SetChildBool(xml, item, "RegionEnabled", obj->regionEnabled));
    kCheck(kXml_AddItem(xml, item, "Region", &regionItem));
    kCheck(GoRegion3d_Write(obj->region, xml, regionItem));

    kCheck(kXml_SetChildBool(xml, item, "RefRegionsEnabled", obj->refRegionsEnabled));
    kCheck(kXml_SetChildSize(xml, item, "RefRegionCount", obj->refRegionCount));

    kCheck(kXml_AddItem(xml, item, "RefRegions", &refRegionsItem));

    for (i = 0; i < GO_SURFACE_STUD_MAX_REF_REGIONS; i++)
    {
        kCheck(kXml_AddItem(xml, refRegionsItem, "RefRegion", &regionItem));
        kCheck(GoSurfaceRegion2d_Write(obj->refRegions[i], xml, regionItem));
    }
    kCheck(kXml_SetChildBool(xml, item, "AutoTiltEnabled", obj->autoTiltEnabled));
    kCheck(kXml_SetChild64f(xml, item, "TiltXAngle", obj->tiltXAngle));
    kCheck(kXml_SetChild64f(xml, item, "TiltYAngle", obj->tiltYAngle));

    kCheck(GoSurfaceTool_Write(tool, xml, item));

    return kOK;
}


GoFx(k64f) GoSurfaceStud_StudRadius(GoSurfaceStud tool)
{
    kObj(GoSurfaceStud, tool);

    return obj->studRadius;
}

GoFx(kStatus) GoSurfaceStud_SetStudRadius(GoSurfaceStud tool, k64f value)
{
    kObj(GoSurfaceStud, tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    obj->studRadius = value;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(k64f) GoSurfaceStud_StudHeight(GoSurfaceStud tool)
{
    kObj(GoSurfaceStud, tool);

    return obj->studHeight;
}

GoFx(kStatus) GoSurfaceStud_SetStudHeight(GoSurfaceStud tool, k64f value)

{
    kObj(GoSurfaceStud, tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    obj->studHeight = value;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(k64f) GoSurfaceStud_BaseHeight(GoSurfaceStud tool)
{
    kObj(GoSurfaceStud, tool);

    return obj->baseHeight;
}

GoFx(kStatus) GoSurfaceStud_SetBaseHeight(GoSurfaceStud tool, k64f value)
{
    kObj(GoSurfaceStud, tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    obj->baseHeight = value;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(k64f) GoSurfaceStud_TipHeight(GoSurfaceStud tool)
{
    kObj(GoSurfaceStud, tool);

    return obj->tipHeight;
}

GoFx(kStatus) GoSurfaceStud_SetTipHeight(GoSurfaceStud tool, k64f value)
{
    kObj(GoSurfaceStud, tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    obj->tipHeight = value;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(kBool) GoSurfaceStud_RegionEnabled(GoSurfaceStud tool)
{
    kObj(GoSurfaceStud, tool);

    return obj->regionEnabled;
}

GoFx(kStatus) GoSurfaceStud_EnableRegion(GoSurfaceStud tool, kBool enable)
{
    kObj(GoSurfaceStud, tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    obj->regionEnabled = enable;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(GoRegion3d) GoSurfaceStud_Region(GoSurfaceStud tool)
{
    kObj(GoSurfaceStud, tool);

    return obj->region;
}

GoFx(kBool) GoSurfaceStud_RefRegionsEnabled(GoSurfaceStud tool)
{
    kObj(GoSurfaceStud, tool);

    return obj->refRegionsEnabled;
}

GoFx(kStatus) GoSurfaceStud_EnableRefRegions(GoSurfaceStud tool, kBool enable)
{
    kObj(GoSurfaceStud, tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    obj->refRegionsEnabled = enable;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(kStatus) GoSurfaceStud_SetRefRegionCount(GoSurfaceStud tool, kSize count)
{
    kObj(GoSurfaceStud, tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    kCheckArgs(count <= GO_SURFACE_STUD_MAX_REF_REGIONS);
    obj->refRegionCount = count;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(kSize) GoSurfaceStud_RefRegionCount(GoSurfaceStud tool)
{
    kObj(GoSurfaceStud, tool);

    return obj->refRegionCount;
}

GoFx(GoSurfaceRegion2d) GoSurfaceStud_RefRegionAt(GoSurfaceStud tool, kSize index)
{
    kObj(GoSurfaceStud, tool);

    kAssert(index < GO_SURFACE_STUD_MAX_REF_REGIONS);

    return obj->refRegions[index];
}

GoFx(kBool) GoSurfaceStud_AutoTiltEnabled(GoSurfaceStud tool)
{
    kObj(GoSurfaceStud, tool);

    return obj->autoTiltEnabled;
}

GoFx(kStatus) GoSurfaceStud_EnableAutoTilt(GoSurfaceStud tool, kBool enable)
{
    kObj(GoSurfaceStud, tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    obj->autoTiltEnabled = enable;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(k64f) GoSurfaceStud_TiltXAngle(GoSurfaceStud tool)
{
    kObj(GoSurfaceStud, tool);

    return obj->tiltXAngle;
}

GoFx(kStatus) GoSurfaceStud_SetTiltXAngle(GoSurfaceStud tool, k64f value)
{
    kObj(GoSurfaceStud, tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    obj->tiltXAngle = value;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(k64f) GoSurfaceStud_TiltYAngle(GoSurfaceStud tool)
{
    kObj(GoSurfaceStud, tool);

    return obj->tiltYAngle;
}

GoFx(kStatus) GoSurfaceStud_SetTiltYAngle(GoSurfaceStud tool, k64f value)
{
    kObj(GoSurfaceStud, tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    obj->tiltYAngle = value;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(GoSurfaceStudBaseX) GoSurfaceStud_BaseXMeasurement(GoSurfaceStud tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_SURFACE_STUD_BASE_X);
}

GoFx(GoSurfaceStudBaseY) GoSurfaceStud_BaseYMeasurement(GoSurfaceStud tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_SURFACE_STUD_BASE_Y);
}

GoFx(GoSurfaceStudBaseZ) GoSurfaceStud_BaseZMeasurement(GoSurfaceStud tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_SURFACE_STUD_BASE_Z);
}

GoFx(GoSurfaceStudTipX) GoSurfaceStud_TipXMeasurement(GoSurfaceStud tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_SURFACE_STUD_TIP_X);
}

GoFx(GoSurfaceStudTipY) GoSurfaceStud_TipYMeasurement(GoSurfaceStud tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_SURFACE_STUD_TIP_Y);
}

GoFx(GoSurfaceStudTipZ) GoSurfaceStud_TipZMeasurement(GoSurfaceStud tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_SURFACE_STUD_TIP_Z);
}

GoFx(GoSurfaceStudRadius) GoSurfaceStud_RadiusMeasurement(GoSurfaceStud tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_SURFACE_STUD_RADIUS);
}

GoFx(GoSurfaceStudTipPoint) GoSurfaceStud_TipPoint(GoSurfaceHole tool)
{
    return GoTool_FindFeatureOutputByType(tool, GO_FEATURE_SURFACE_STUD_TIP_POINT);
}

GoFx(GoSurfaceStudBasePoint) GoSurfaceStud_BasePoint(GoSurfaceHole tool)
{
    return GoTool_FindFeatureOutputByType(tool, GO_FEATURE_SURFACE_STUD_BASE_POINT);
}

kBeginClassEx(Go, GoSurfaceVolume)
kAddVMethod(GoSurfaceVolume, kObject, VRelease)
kAddVMethod(GoSurfaceVolume, GoTool, VInit)
kAddVMethod(GoSurfaceVolume, GoTool, VRead)
kAddVMethod(GoSurfaceVolume, GoTool, VWrite)
kEndClassEx()

GoFx(kStatus) GoSurfaceVolume_Construct(GoSurfaceVolume* tool, kObject sensor, kAlloc allocator)
{
    return GoTool_Construct(tool, kTypeOf(GoSurfaceVolume), sensor, allocator);
}

GoFx(kStatus) GoSurfaceVolume_VInit(GoSurfaceVolume tool, kType type, kObject sensor, kAlloc alloc)
{
    kObjR(GoSurfaceVolume, tool);

    kCheck(GoSurfaceTool_Init(tool, type, GO_TOOL_SURFACE_VOLUME, sensor, alloc));
    obj->region = kNULL;

    obj->regionEnabled = kTRUE;

    kCheck(GoRegion3d_Construct(&obj->region, sensor, alloc));

    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoSurfaceVolumeVolume), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoSurfaceVolumeArea), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoSurfaceVolumeThickness), kTRUE, kNULL));

    return kOK;
}

GoFx(kStatus) GoSurfaceVolume_VRelease(GoSurfaceVolume tool)
{
    kObj(GoSurfaceVolume, tool);

    kCheck(kDestroyRef(&obj->region));

    return GoSurfaceTool_VRelease(tool);
}

GoFx(kStatus) GoSurfaceVolume_VRead(GoSurfaceVolume tool, kXml xml, kXmlItem item)
{
    kObj(GoSurfaceVolume, tool);
    kXmlItem regionItem = kNULL;

    kCheck(kXml_ChildBool(xml, item, "RegionEnabled", &obj->regionEnabled));

    kCheck(!kIsNull(regionItem = kXml_Child(xml, item, "Region")));
    kCheck(GoRegion3d_Read(obj->region, xml, regionItem));

    kCheck(GoSurfaceTool_Read(tool, xml, item));

    return kOK;
}

GoFx(kStatus) GoSurfaceVolume_VWrite(GoSurfaceVolume tool, kXml xml, kXmlItem item)
{
    kObj(GoSurfaceVolume, tool);
    kXmlItem regionItem = kNULL;

    kCheck(kXml_SetChildBool(xml, item, "RegionEnabled", obj->regionEnabled));

    kCheck(kXml_AddItem(xml, item, "Region", &regionItem));
    kCheck(GoRegion3d_Write(obj->region, xml, regionItem));

    kCheck(GoSurfaceTool_Write(tool, xml, item));

    return kOK;
}


GoFx(kBool) GoSurfaceVolume_RegionEnabled(GoSurfaceVolume tool)
{
    kObj(GoSurfaceVolume, tool);

    return obj->regionEnabled;
}

GoFx(kStatus) GoSurfaceVolume_EnableRegion(GoSurfaceVolume tool, kBool enable)
{
    kObj(GoSurfaceVolume, tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));

    obj->regionEnabled = enable;

    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(GoRegion3d) GoSurfaceVolume_Region(GoSurfaceVolume tool)
{
    kObj(GoSurfaceVolume, tool);

    return obj->region;
}

GoFx(GoSurfaceVolumeVolume) GoSurfaceVolume_VolumeMeasurement(GoSurfaceVolume tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_SURFACE_VOLUME_VOLUME);
}

GoFx(GoSurfaceVolumeArea) GoSurfaceVolume_AreaMeasurement(GoSurfaceVolume tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_SURFACE_VOLUME_AREA);
}

GoFx(GoSurfaceVolumeThickness) GoSurfaceVolume_ThicknessMeasurement(GoSurfaceVolume tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_SURFACE_VOLUME_THICKNESS);
}

kBeginClassEx(Go, GoSurfaceRivet)
kAddVMethod(GoSurfaceRivet, kObject, VRelease)
kAddVMethod(GoSurfaceRivet, GoTool, VInit)
kAddVMethod(GoSurfaceRivet, GoTool, VRead)
kAddVMethod(GoSurfaceRivet, GoTool, VWrite)
kEndClassEx()

GoFx(kStatus) GoSurfaceRivet_Construct(GoSurfaceRivet* tool, kObject sensor, kAlloc allocator)
{
    return GoTool_Construct(tool, kTypeOf(GoSurfaceRivet), sensor, allocator);
}

GoFx(kStatus) GoSurfaceRivet_VInit(GoSurfaceRivet tool, kType type, kObject sensor, kAlloc alloc)
{
    kObjR(GoSurfaceRivet, tool);
    kSize i;

    kCheck(GoSurfaceTool_Init(tool, type, GO_TOOL_SURFACE_RIVET, sensor, alloc));
    obj->nominalHeight = 0.0;
    obj->partialDetectionEnabled = kFALSE;
    obj->region = kNULL;
    obj->refRegionsEnabled = kFALSE;
    obj->refRegionCount = 0;
    kZero(obj->refRegions);
    obj->tiltXAngle = 0.0;
    obj->tiltYAngle = 0.0;
    obj->type = GO_SURFACE_RIVET_TYPE_FLUSH;
    obj->edgeSensitivity = 0.0;
    obj->innerPadding = 0.0;

    obj->nominalRadius = 10.0;
    obj->radiusTolerance = 1.0;
    obj->regionEnabled = kTRUE;
    obj->autoTiltEnabled = kTRUE;

    kCheck(GoRegion3d_Construct(&obj->region, sensor, alloc));

    for (i = 0; i < GO_SURFACE_RIVET_MAX_REF_REGIONS; i++)
    {
        kCheck(GoSurfaceRegion2d_Construct(&obj->refRegions[i], sensor, alloc));
    }

    return kOK;
}

GoFx(kStatus) GoSurfaceRivet_VRelease(GoSurfaceRivet tool)
{
    kObj(GoSurfaceRivet, tool);
    kSize i;

    kDestroyRef(&obj->region);

    for (i = 0; i < GO_SURFACE_HOLE_MAX_REF_REGIONS; i++)
    {
        kDestroyRef(&obj->refRegions[i]);
    }

    return GoSurfaceTool_VRelease(tool);
}

GoFx(kStatus) GoSurfaceRivet_VRead(GoSurfaceRivet tool, kXml xml, kXmlItem item)
{
    kObj(GoSurfaceRivet, tool);
    kSize i;
    kXmlItem regionItem = kNULL;
    kXmlItem refRegionsItem = kNULL;

    kCheck(kXml_Child64f(xml, item, "NominalHeight", &obj->nominalHeight));
    kCheck(kXml_Child64f(xml, item, "NominalRadius", &obj->nominalRadius));
    kCheck(kXml_Child64f(xml, item, "RadiusTolerance", &obj->radiusTolerance));
    kCheck(kXml_ChildBool(xml, item, "PartialDetectionEnabled", &obj->partialDetectionEnabled));

    kCheck(kXml_ChildBool(xml, item, "RegionEnabled", &obj->regionEnabled));
    kCheck(!kIsNull(regionItem = kXml_Child(xml, item, "Region")));
    kCheck(GoRegion3d_Read(obj->region, xml, regionItem));

    kCheck(kXml_ChildBool(xml, item, "RefRegionsEnabled", &obj->refRegionsEnabled));
    kCheck(kXml_ChildSize(xml, item, "RefRegionCount", &obj->refRegionCount));

    kCheck(!kIsNull(refRegionsItem = kXml_Child(xml, item, "RefRegions")));
    for (i = 0; i < GO_SURFACE_HOLE_MAX_REF_REGIONS; i++)
    {
        kCheck(!kIsNull(regionItem = kXml_ChildAt(xml, refRegionsItem, i)));
        kCheck(GoSurfaceRegion2d_Read(obj->refRegions[i], xml, regionItem));
    }

    kCheck(kXml_ChildBool(xml, item, "AutoTiltEnabled", &obj->autoTiltEnabled));
    kCheck(kXml_Child64f(xml, item, "TiltXAngle", &obj->tiltXAngle));
    kCheck(kXml_Child64f(xml, item, "TiltYAngle", &obj->tiltYAngle));

    kCheck(kXml_Child32s(xml, item, "Type", &obj->type));
    kCheck(kXml_Child64f(xml, item, "EdgeSensitivity", &obj->edgeSensitivity));
    kCheck(kXml_Child64f(xml, item, "InnerPadding", &obj->innerPadding));

    kCheck(GoSurfaceTool_Read(tool, xml, item));

    return kOK;
}

GoFx(kStatus) GoSurfaceRivet_VWrite(GoSurfaceRivet tool, kXml xml, kXmlItem item)
{
    kObj(GoSurfaceRivet, tool);
    kXmlItem regionItem = kNULL;
    kXmlItem refRegionsItem = kNULL;
    kSize i;

    kCheck(kXml_SetChild64f(xml, item, "NominalHeight", obj->nominalHeight));
    kCheck(kXml_SetChild64f(xml, item, "NominalRadius", obj->nominalRadius));
    kCheck(kXml_SetChild64f(xml, item, "RadiusTolerance", obj->radiusTolerance));
    kCheck(kXml_SetChildBool(xml, item, "PartialDetectionEnabled", obj->partialDetectionEnabled));

    kCheck(kXml_SetChildBool(xml, item, "RegionEnabled", obj->regionEnabled));
    kCheck(kXml_AddItem(xml, item, "Region", &regionItem));
    kCheck(GoRegion3d_Write(obj->region, xml, regionItem));

    kCheck(kXml_SetChildBool(xml, item, "RefRegionsEnabled", obj->refRegionsEnabled));
    kCheck(kXml_SetChildSize(xml, item, "RefRegionCount", obj->refRegionCount));

    kCheck(kXml_AddItem(xml, item, "RefRegions", &refRegionsItem));
    for (i = 0; i < GO_SURFACE_HOLE_MAX_REF_REGIONS; i++)
    {
        kCheck(kXml_AddItem(xml, refRegionsItem, "RefRegion", &regionItem));
        kCheck(GoSurfaceRegion2d_Write(obj->refRegions[i], xml, regionItem));
    }

    kCheck(kXml_SetChildBool(xml, item, "AutoTiltEnabled", obj->autoTiltEnabled));
    kCheck(kXml_SetChild64f(xml, item, "TiltXAngle", obj->tiltXAngle));
    kCheck(kXml_SetChild64f(xml, item, "TiltYAngle", obj->tiltYAngle));

    kCheck(kXml_SetChild32s(xml, item, "Type", obj->type));
    kCheck(kXml_SetChild64f(xml, item, "EdgeSensitivity", obj->edgeSensitivity));
    kCheck(kXml_SetChild64f(xml, item, "InnerPadding", obj->innerPadding));

    kCheck(GoSurfaceTool_Write(tool, xml, item));

    return kOK;
}

GoFx(k64f) GoSurfaceRivet_NominalHeight(GoSurfaceRivet tool)
{
    kObj(GoSurfaceRivet, tool);

    return obj->nominalHeight;
}

GoFx(kStatus) GoSurfaceRivet_SetNominalHeight(GoSurfaceRivet tool, k64f nominalHeight)
{
    kObj(GoSurfaceRivet, tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    obj->nominalHeight = nominalHeight;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(k64f) GoSurfaceRivet_NominalRadius(GoSurfaceRivet tool)
{
    kObj(GoSurfaceRivet, tool);

    return obj->nominalRadius;
}

GoFx(kStatus) GoSurfaceRivet_SetNominalRadius(GoSurfaceRivet tool, k64f nominalRadius)
{
    kObj(GoSurfaceRivet, tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    obj->nominalRadius = nominalRadius;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(k64f) GoSurfaceRivet_RadiusTolerance(GoSurfaceRivet tool)
{
    kObj(GoSurfaceRivet, tool);

    return obj->radiusTolerance;
}

GoFx(kStatus) GoSurfaceRivet_SetRadiusTolerance(GoSurfaceRivet tool, k64f radiusTolerance)
{
    kObj(GoSurfaceRivet, tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    obj->radiusTolerance = radiusTolerance;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(kBool) GoSurfaceRivet_PartialDetectionEnabled(GoSurfaceRivet tool)
{
    kObj(GoSurfaceRivet, tool);

    return obj->partialDetectionEnabled;
}

GoFx(kStatus) GoSurfaceRivet_EnablePartialDetection(GoSurfaceRivet tool, kBool enable)
{
    kObj(GoSurfaceRivet, tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    obj->partialDetectionEnabled = enable;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(kBool) GoSurfaceRivet_RegionEnabled(GoSurfaceRivet tool)
{
    kObj(GoSurfaceRivet, tool);

    return obj->regionEnabled;
}

GoFx(kStatus) GoSurfaceRivet_EnableRegion(GoSurfaceRivet tool, kBool enable)
{
    kObj(GoSurfaceRivet, tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    obj->regionEnabled = enable;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(GoRegion3d) GoSurfaceRivet_Region(GoSurfaceRivet tool)
{
    kObj(GoSurfaceRivet, tool);

    return obj->region;
}

GoFx(kBool) GoSurfaceRivet_RefRegionsEnabled(GoSurfaceRivet tool)
{
    kObj(GoSurfaceRivet, tool);

    return obj->refRegionsEnabled;
}

GoFx(kStatus) GoSurfaceRivet_EnableRefRegions(GoSurfaceRivet tool, kBool enable)
{
    kObj(GoSurfaceRivet, tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    obj->refRegionsEnabled = enable;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(kSize) GoSurfaceRivet_RefRegionCount(GoSurfaceRivet tool)
{
    kObj(GoSurfaceRivet, tool);

    return obj->refRegionCount;
}

GoFx(kStatus) GoSurfaceRivet_SetRefRegionCount(GoSurfaceRivet tool, kSize count)
{
    kObj(GoSurfaceRivet, tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    kCheckArgs(count <= GO_SURFACE_RIVET_MAX_REF_REGIONS);
    obj->refRegionCount = count;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(GoSurfaceRegion2d) GoSurfaceRivet_RefRegionAt(GoSurfaceRivet tool, kSize index)
{
    kObj(GoSurfaceRivet, tool);

    kAssert(index < GO_SURFACE_RIVET_MAX_REF_REGIONS);

    return obj->refRegions[index];
}

GoFx(kBool) GoSurfaceRivet_AutoTiltEnabled(GoSurfaceRivet tool)
{
    kObj(GoSurfaceRivet, tool);

    return obj->autoTiltEnabled;
}

GoFx(kStatus) GoSurfaceRivet_EnableAutoTilt(GoSurfaceRivet tool, kBool enable)
{
    kObj(GoSurfaceRivet, tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    obj->autoTiltEnabled = enable;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(k64f) GoSurfaceRivet_TiltXAngle(GoSurfaceRivet tool)
{
    kObj(GoSurfaceRivet, tool);

    return obj->tiltXAngle;
}

GoFx(kStatus) GoSurfaceRivet_SetTiltXAngle(GoSurfaceRivet tool, k64f value)
{
    kObj(GoSurfaceRivet, tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    obj->tiltXAngle = value;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(k64f) GoSurfaceRivet_TiltYAngle(GoSurfaceRivet tool)
{
    kObj(GoSurfaceRivet, tool);

    return obj->tiltYAngle;
}

GoFx(kStatus) GoSurfaceRivet_SetTiltYAngle(GoSurfaceRivet tool, k64f value)
{
    kObj(GoSurfaceRivet, tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    obj->tiltYAngle = value;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(GoSurfaceRivetType) GoSurfaceRivet_Type(GoSurfaceRivet tool)
{
    kObj(GoSurfaceRivet, tool);

    return obj->type;
}

GoFx(kStatus) GoSurfaceRivet_SetType(GoSurfaceRivet tool, GoSurfaceRivetType type)
{
    kObj(GoSurfaceRivet, tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    obj->type = type;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(k64f) GoSurfaceRivet_EdgeSensitivity(GoSurfaceRivet tool)
{
    kObj(GoSurfaceRivet, tool);

    return obj->edgeSensitivity;
}

GoFx(kStatus) GoSurfaceRivet_SetEdgeSensitivity(GoSurfaceRivet tool, k64f value)
{
    kObj(GoSurfaceRivet, tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    obj->edgeSensitivity = value;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(k64f) GoSurfaceRivet_InnerPadding(GoSurfaceRivet tool)
{
    kObj(GoSurfaceRivet, tool);

    return obj->innerPadding;
}

GoFx(kStatus) GoSurfaceRivet_SetInnerPadding(GoSurfaceRivet tool, k64f value)
{
    kObj(GoSurfaceRivet, tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    obj->innerPadding = value;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(kSize) GoSurfaceRivet_MeasurementMax(GoSurfaceRivet tool, GoMeasurementType measurementType)
{
    switch (measurementType)
    {
        case GO_MEASUREMENT_SURFACE_RIVET_X:
        case GO_MEASUREMENT_SURFACE_RIVET_Y:
        case GO_MEASUREMENT_SURFACE_RIVET_Z:
        case GO_MEASUREMENT_SURFACE_RIVET_TILT_ANGLE:
        case GO_MEASUREMENT_SURFACE_RIVET_TILT_DIRECTION:
        case GO_MEASUREMENT_SURFACE_RIVET_RADIUS:
        case GO_MEASUREMENT_SURFACE_RIVET_TOP_OFFSET_MIN:
        case GO_MEASUREMENT_SURFACE_RIVET_TOP_OFFSET_MAX:
        case GO_MEASUREMENT_SURFACE_RIVET_TOP_OFFSET_MEAN:
        case GO_MEASUREMENT_SURFACE_RIVET_TOP_OFFSET_STD_DEV: return 1;
        case GO_MEASUREMENT_SURFACE_RIVET_RADIAL_HEIGHT_MIN:
        case GO_MEASUREMENT_SURFACE_RIVET_RADIAL_HEIGHT_MAX:
        case GO_MEASUREMENT_SURFACE_RIVET_RADIAL_HEIGHT_MEAN:
        case GO_MEASUREMENT_SURFACE_RIVET_RADIAL_HEIGHT_STD_DEV:
        case GO_MEASUREMENT_SURFACE_RIVET_RADIAL_SLOPE_MIN:
        case GO_MEASUREMENT_SURFACE_RIVET_RADIAL_SLOPE_MAX:
        case GO_MEASUREMENT_SURFACE_RIVET_RADIAL_SLOPE_MEAN:
        case GO_MEASUREMENT_SURFACE_RIVET_RADIAL_SLOPE_STD_DEV: return 100;
    }

    return 0;
}

GoFx(kStatus) GoSurfaceRivet_AddMeasurement(GoSurfaceRivet tool, GoMeasurementType type, GoMeasurement* measurement)
{
    kType classType = kNULL;
    kStatus status;
    GoMeasurementOption* measurementOption = kNULL;
    kSize instanceCount = 0;
    kSize i;

    if (type != GO_MEASUREMENT_SURFACE_RIVET_X
        && type != GO_MEASUREMENT_SURFACE_RIVET_Y
        && type != GO_MEASUREMENT_SURFACE_RIVET_Z
        && type != GO_MEASUREMENT_SURFACE_RIVET_TILT_ANGLE
        && type != GO_MEASUREMENT_SURFACE_RIVET_TILT_DIRECTION
        && type != GO_MEASUREMENT_SURFACE_RIVET_RADIUS
        && type != GO_MEASUREMENT_SURFACE_RIVET_TOP_OFFSET_MIN
        && type != GO_MEASUREMENT_SURFACE_RIVET_TOP_OFFSET_MAX
        && type != GO_MEASUREMENT_SURFACE_RIVET_TOP_OFFSET_MEAN
        && type != GO_MEASUREMENT_SURFACE_RIVET_TOP_OFFSET_STD_DEV
        && type != GO_MEASUREMENT_SURFACE_RIVET_RADIAL_HEIGHT_MIN
        && type != GO_MEASUREMENT_SURFACE_RIVET_RADIAL_HEIGHT_MAX
        && type != GO_MEASUREMENT_SURFACE_RIVET_RADIAL_HEIGHT_MEAN
        && type != GO_MEASUREMENT_SURFACE_RIVET_RADIAL_HEIGHT_STD_DEV
        && type != GO_MEASUREMENT_SURFACE_RIVET_RADIAL_SLOPE_MIN
        && type != GO_MEASUREMENT_SURFACE_RIVET_RADIAL_SLOPE_MAX
        && type != GO_MEASUREMENT_SURFACE_RIVET_RADIAL_SLOPE_MEAN
        && type != GO_MEASUREMENT_SURFACE_RIVET_RADIAL_SLOPE_STD_DEV)
    {
        return kERROR_PARAMETER;
    }
    else
    {
        switch (type)
        {
            case GO_MEASUREMENT_SURFACE_RIVET_X: classType = kTypeOf(GoSurfaceRivetX); break;
            case GO_MEASUREMENT_SURFACE_RIVET_Y: classType = kTypeOf(GoSurfaceRivetY); break;
            case GO_MEASUREMENT_SURFACE_RIVET_Z: classType = kTypeOf(GoSurfaceRivetZ); break;
            case GO_MEASUREMENT_SURFACE_RIVET_TILT_ANGLE: classType = kTypeOf(GoSurfaceRivetTiltAngle); break;
            case GO_MEASUREMENT_SURFACE_RIVET_TILT_DIRECTION: classType = kTypeOf(GoSurfaceRivetTiltDirection); break;
            case GO_MEASUREMENT_SURFACE_RIVET_RADIUS: classType = kTypeOf(GoSurfaceRivetRadius); break;
            case GO_MEASUREMENT_SURFACE_RIVET_TOP_OFFSET_MIN: classType = kTypeOf(GoSurfaceRivetTopOffsetMin); break;
            case GO_MEASUREMENT_SURFACE_RIVET_TOP_OFFSET_MAX: classType = kTypeOf(GoSurfaceRivetTopOffsetMax); break;
            case GO_MEASUREMENT_SURFACE_RIVET_TOP_OFFSET_MEAN: classType = kTypeOf(GoSurfaceRivetTopOffsetMean); break;
            case GO_MEASUREMENT_SURFACE_RIVET_TOP_OFFSET_STD_DEV: classType = kTypeOf(GoSurfaceRivetTopOffsetStdDev); break;
            case GO_MEASUREMENT_SURFACE_RIVET_RADIAL_HEIGHT_MIN: classType = kTypeOf(GoSurfaceRivetRadialHeightMin); break;
            case GO_MEASUREMENT_SURFACE_RIVET_RADIAL_HEIGHT_MAX: classType = kTypeOf(GoSurfaceRivetRadialHeightMax); break;
            case GO_MEASUREMENT_SURFACE_RIVET_RADIAL_HEIGHT_MEAN: classType = kTypeOf(GoSurfaceRivetRadialHeightMean); break;
            case GO_MEASUREMENT_SURFACE_RIVET_RADIAL_HEIGHT_STD_DEV: classType = kTypeOf(GoSurfaceRivetRadialHeightStdDev); break;
            case GO_MEASUREMENT_SURFACE_RIVET_RADIAL_SLOPE_MIN: classType = kTypeOf(GoSurfaceRivetRadialSlopeMin); break;
            case GO_MEASUREMENT_SURFACE_RIVET_RADIAL_SLOPE_MAX: classType = kTypeOf(GoSurfaceRivetRadialSlopeMax); break;
            case GO_MEASUREMENT_SURFACE_RIVET_RADIAL_SLOPE_MEAN: classType = kTypeOf(GoSurfaceRivetRadialSlopeMean); break;
            case GO_MEASUREMENT_SURFACE_RIVET_RADIAL_SLOPE_STD_DEV: classType = kTypeOf(GoSurfaceRivetRadialSlopeStdDev); break;
        }
    }

    for (i = 0; i < GoTool_MeasurementCount(tool); i++)
    {
        if (kObject_Type(GoTool_MeasurementAt(tool, i)) == classType)
        {
            instanceCount++;
        }
    }

    if (instanceCount + 1 > GoSurfaceRivet_MeasurementMax(tool, type))
    {
        return kERROR;
    }

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    kCheck(GoTool_AddMeasurement(tool, classType, kTRUE, measurement));

    kTry
    {
        kTest(GoSensor_SetConfigModified(GoTool_Sensor(tool)));
        kTest(GoSensor_SyncConfig(GoTool_Sensor(tool)));
    }
    kCatch(&status)
    {
        kCheck(GoTool_RemoveMeasurement(tool, GoTool_MeasurementCount(tool) - 1));
        kEndCatch(status);
    }

    return kOK;
}

GoFx(kStatus) GoSurfaceRivet_RemoveMeasurement(GoSurfaceRivet tool, kSize index)
{
    if (index > GoTool_MeasurementCount(tool))
    {
        return kERROR_PARAMETER;
    }

    kCheck(GoTool_RemoveMeasurement(tool, index));

    return kOK;
}

GoFx(kSize) GoSurfaceRivet_MeasurementCount(GoSurfaceRivet tool)
{
    return GoTool_MeasurementCount(tool);
}

GoFx(GoMeasurement) GoSurfaceRivet_MeasurementAt(GoSurfaceRivet tool, kSize index)
{
    kAssert(index < GoTool_MeasurementCount(tool));

    return GoTool_MeasurementAt(tool, index);
}

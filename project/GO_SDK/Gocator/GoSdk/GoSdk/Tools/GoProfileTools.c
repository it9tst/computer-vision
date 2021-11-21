/** 
 * @file    GoProfileTools.c
 *
 * @internal
 * Copyright (C) 2016-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <GoSdk/Tools/GoProfileTools.h>
#include <GoSdk/GoSensor.h>

kBeginClassEx(Go, GoProfileTool)
kAddVMethod(GoProfileTool, kObject, VRelease)
kEndClassEx()

GoFx(kStatus) GoProfileTool_Init(GoProfileTool tool, kType type, GoToolType typeId, kObject sensor, kAlloc alloc)
{
    kObjR(GoProfileTool, tool);

    kCheck(GoTool_Init(tool, type, typeId, sensor, alloc)); 
    kZero(obj->streamOptions);
    kZero(obj->stream);
    kZero(obj->sourceOptions);
    kZero(obj->xAnchorOptions);
    kZero(obj->zAnchorOptions);

    obj->source = GO_DATA_SOURCE_TOP; 
    obj->xAnchor = -1;
    obj->zAnchor = -1;
    obj->stream.step = GO_DATA_STEP_PROFILE;
    obj->stream.id = 0;

    kCheck(kArrayList_Construct(&obj->streamOptions, kTypeOf(GoDataStream), 0, alloc)); 
    kCheck(kArrayList_Construct(&obj->sourceOptions, kTypeOf(GoDataSource), 0, alloc)); 

    kCheck(kArrayList_Construct(&obj->xAnchorOptions, kTypeOf(k32u), 0, alloc));
    kCheck(kArrayList_Construct(&obj->zAnchorOptions, kTypeOf(k32u), 0, alloc));

    return kOK; 
}

GoFx(kStatus) GoProfileTool_VRelease(GoProfileTool tool)
{
    kObj(GoProfileTool, tool);

    kCheck(kDisposeRef(&obj->streamOptions)); 
    kCheck(kDisposeRef(&obj->sourceOptions)); 
    kCheck(kDisposeRef(&obj->xAnchorOptions));
    kCheck(kDisposeRef(&obj->zAnchorOptions));

    return GoTool_VRelease(tool);
}

GoFx(kStatus) GoProfileTool_Read(GoProfileTool tool, kXml xml, kXmlItem item)
{
    kObj(GoProfileTool, tool);
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
        kTest(kXml_Child32s(xml, anchorItem, "X", &obj->xAnchor));
        kTestArgs(!kIsNull(tempItem = kXml_Child(xml, anchorItem, "X")));
        kTest(kXml_AttrString(xml, tempItem, "options", text)); 
        kTest(GoOptionList_ParseList32u(kString_Chars(text), obj->xAnchorOptions));

        kTestArgs(!kIsNull(anchorItem = kXml_Child(xml, item, "Anchor")));
        kTest(kXml_Child32s(xml, anchorItem, "Z", &obj->zAnchor));
        kTestArgs(!kIsNull(tempItem = kXml_Child(xml, anchorItem, "Z")));
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

GoFx(kStatus) GoProfileTool_Write(GoProfileTool tool, kXml xml, kXmlItem item)
{
    kObj(GoProfileTool, tool);
    kXmlItem streamItem = kNULL;
    kXmlItem anchorItem = kNULL;

    kCheck(kXml_AddItem(xml, item, "Stream", &streamItem));
    kCheck(kXml_SetChild32s(xml, streamItem, "Step", obj->stream.step));
    kCheck(kXml_SetChild32s(xml, streamItem, "Id", obj->stream.id));

    kCheck(kXml_SetChild32s(xml, item, "Source", obj->source));

    kCheck(kXml_AddItem(xml, item, "Anchor", &anchorItem));
    kCheck(kXml_SetChild32s(xml, anchorItem, "X", obj->xAnchor));
    kCheck(kXml_SetChild32s(xml, anchorItem, "Z", obj->zAnchor));

    kCheck(GoTool_VWrite(tool, xml, item));

    return kOK; 
}

GoFx(kSize) GoProfileTool_StreamOptionCount(GoProfileTool tool)
{
    kObj(GoProfileTool, tool);

    GoSensor_SyncConfig(obj->base.sensor);

    return kArrayList_Count(obj->streamOptions);
}

GoFx(GoDataStream) GoProfileTool_StreamOptionAt(GoProfileTool tool, kSize index)
{
    kObj(GoProfileTool, tool);

    GoSensor_SyncConfig(obj->base.sensor);

    kAssert(index < kArrayList_Count(obj->streamOptions));

    return kArrayList_AsT(obj->streamOptions, index, GoDataStream);
}

GoFx(kStatus) GoProfileTool_SetStream(GoProfileTool tool, GoDataStream stream)
{
    kObj(GoProfileTool, tool);

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

GoFx(GoDataStream) GoProfileTool_Stream(GoProfileTool tool)
{
    kObj(GoProfileTool, tool);

    GoSensor_SyncConfig(obj->base.sensor);

    return obj->stream;
}

GoFx(kSize) GoProfileTool_SourceOptionCount(GoProfileTool tool)
{
    kObj(GoProfileTool, tool);

    GoSensor_SyncConfig(obj->base.sensor);

    return kArrayList_Count(obj->sourceOptions);
}

GoFx(GoDataSource) GoProfileTool_SourceOptionAt(GoProfileTool tool, kSize index)
{
    kObj(GoProfileTool, tool);

    GoSensor_SyncConfig(obj->base.sensor);

    kAssert(index < kArrayList_Count(obj->sourceOptions));
    
    return kArrayList_AsT(obj->sourceOptions, index, GoDataSource);
}

GoFx(kStatus) GoProfileTool_SetSource(GoProfileTool tool, GoDataSource source)
{
    kObj(GoProfileTool, tool);

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

GoFx(GoDataSource) GoProfileTool_Source(GoProfileTool tool)
{
    kObj(GoProfileTool, tool);

    GoSensor_SyncConfig(obj->base.sensor);

    return obj->source;
}

GoFx(kSize) GoProfileTool_XAnchorOptionCount(GoProfileTool tool)
{
    kObj(GoProfileTool, tool);

    GoSensor_SyncConfig(obj->base.sensor);

    return kArrayList_Count(obj->xAnchorOptions);
}

GoFx(k32u) GoProfileTool_XAnchorOptionAt(GoProfileTool tool, kSize index)
{
    kObj(GoProfileTool, tool);

    GoSensor_SyncConfig(obj->base.sensor);

    kAssert(index < kArrayList_Count(obj->xAnchorOptions));
    
    return kArrayList_AsT(obj->xAnchorOptions, index, k32u);
}

GoFx(k32s) GoProfileTool_XAnchor(GoProfileTool tool)
{
    kObj(GoProfileTool, tool);

    GoSensor_SyncConfig(obj->base.sensor);

    return obj->xAnchor;
}

GoFx(kStatus) GoProfileTool_SetXAnchor(GoProfileTool tool, k32s id)
{
    kObj(GoProfileTool, tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    kCheck(GoSensor_SyncConfig(GoTool_Sensor(tool)));

    if (id >= 0)
    {
        kCheck(GoOptionList_Check32u(kArrayList_DataT(obj->xAnchorOptions, k32u), kArrayList_Count(obj->xAnchorOptions), id));
    }

    obj->xAnchor = id;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(kBool) GoProfileTool_XAnchorEnabled(GoProfileTool tool)
{
    kObj(GoProfileTool, tool);

    GoSensor_SyncConfig(obj->base.sensor);

    if (obj->xAnchor >= 0)
    {
        return kTRUE;
    }

    return kFALSE;
}

GoFx(kSize) GoProfileTool_ZAnchorOptionCount(GoProfileTool tool)
{
    kObj(GoProfileTool, tool);

    GoSensor_SyncConfig(obj->base.sensor);

    return kArrayList_Count(obj->zAnchorOptions);
}

GoFx(k32u) GoProfileTool_ZAnchorOptionAt(GoProfileTool tool, kSize index)
{
    kObj(GoProfileTool, tool);

    GoSensor_SyncConfig(obj->base.sensor);

    kAssert(index < kArrayList_Count(obj->zAnchorOptions));
    
    return kArrayList_AsT(obj->zAnchorOptions, index, k32u);
}

GoFx(kStatus) GoProfileTool_SetZAnchor(GoProfileTool tool, k32s id)
{
    kObj(GoProfileTool, tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    kCheck(GoSensor_SyncConfig(GoTool_Sensor(tool)));

    if (id >= 0)
    {
        kCheck(GoOptionList_Check32u(kArrayList_DataT(obj->zAnchorOptions, k32u), kArrayList_Count(obj->zAnchorOptions), id));
    }

    obj->zAnchor = id;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}


GoFx(kBool) GoProfileTool_ZAnchorEnabled(GoProfileTool tool)
{
    kObj(GoProfileTool, tool);

    GoSensor_SyncConfig(obj->base.sensor);

    if (obj->zAnchor >= 0)
    {
        return kTRUE;
    }

    return kFALSE;
}

GoFx(k32s) GoProfileTool_ZAnchor(GoProfileTool tool)
{
    kObj(GoProfileTool, tool);

    GoSensor_SyncConfig(obj->base.sensor);

    return obj->zAnchor;
}

kBeginClassEx(Go, GoProfileArea)
kAddVMethod(GoProfileArea, kObject, VRelease)
kAddVMethod(GoProfileArea, GoTool, VInit)
kAddVMethod(GoProfileArea, GoTool, VRead)
kAddVMethod(GoProfileArea, GoTool, VWrite)
kEndClassEx()

GoFx(kStatus) GoProfileArea_Construct(GoProfileArea* tool, kObject sensor, kAlloc allocator)
{
    return GoTool_Construct(tool, kTypeOf(GoProfileArea), sensor, allocator);
}

GoFx(kStatus) GoProfileArea_VInit(GoProfileArea tool, kType type, kObject sensor, kAlloc alloc)
{
    kObjR(GoProfileArea, tool); 

    kCheck(GoProfileTool_Init(tool, type, GO_TOOL_PROFILE_AREA, sensor, alloc)); 
    obj->type = GO_PROFILE_AREA_TYPE_OBJECT;
    obj->typeUsed = kFALSE;
    obj->baseline = GO_PROFILE_BASELINE_TYPE_X_AXIS;
    obj->baselineUsed = kFALSE;
    obj->region = kNULL;
    obj->regionEnabled = kFALSE;
    obj->line = kNULL;

    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoProfileAreaArea), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoProfileAreaCentroidX), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoProfileAreaCentroidZ), kTRUE, kNULL));

    kCheck(GoTool_AddFeatureOutput(tool, kTypeOf(GoProfileAreaCenterPoint), kNULL));

    obj->type = GO_PROFILE_AREA_TYPE_OBJECT;
    obj->baseline = GO_PROFILE_BASELINE_TYPE_X_AXIS;

    kCheck(GoProfileLineFittingRegion_Construct(&obj->line, sensor, alloc)); 
    kCheck(GoProfileRegion_Construct(&obj->region, sensor, alloc)); 

    return kOK; 
}

GoFx(kStatus) GoProfileArea_VRelease(GoProfileArea tool)
{
    kObj(GoProfileArea, tool);

    kCheck(kDestroyRef(&obj->line)); 
    kCheck(kDestroyRef(&obj->region)); 

    return GoProfileTool_VRelease(tool);
}

GoFx(kStatus) GoProfileArea_VRead(GoProfileArea tool, kXml xml, kXmlItem item)
{
    kObj(GoProfileArea, tool);
    kXmlItem lineItem, regionItem;
    kXmlItem tempItem = kNULL;

    kCheck(GoProfileTool_Read(tool, xml, item)); 

    tempItem = kXml_Child(xml, item, "Type");
    if (!kIsNull(tempItem))
    {
        kCheck(kXml_Item32s(xml, tempItem, &obj->type));
        kCheck(kXml_AttrBool(xml, tempItem, "used", &obj->typeUsed));
    }

    tempItem = kXml_Child(xml, item, "Baseline");
    if (!kIsNull(tempItem))
    {
        kCheck(kXml_Item32s(xml, tempItem, &obj->baseline));
        kCheck(kXml_AttrBool(xml, tempItem, "used", &obj->baselineUsed));
    }

    kCheck(!kIsNull(lineItem = kXml_Child(xml, item, "Line"))); 
    kCheck(GoProfileLineFittingRegion_Read(obj->line, xml, lineItem));
    
    if (kXml_ChildExists(xml, item, "RegionEnabled"))
    {
        kCheck(kXml_ChildBool(xml, item, "RegionEnabled", &obj->regionEnabled));
    }
    
    kCheck(!kIsNull(regionItem = kXml_Child(xml, item, "Region"))); 
    kCheck(GoProfileRegion_Read(obj->region, xml, regionItem));

    return kOK;
}

GoFx(kStatus) GoProfileArea_VWrite(GoProfileArea tool, kXml xml, kXmlItem item)
{
    kObj(GoProfileArea, tool); 
    kXmlItem lineItem, regionItem;

    kCheck(kXml_SetChild32s(xml, item, "Type", obj->type));
    kCheck(kXml_SetChild32s(xml, item, "Baseline", obj->baseline));

    kCheck(kXml_AddItem(xml, item, "Line", &lineItem)); 
    kCheck(GoProfileLineFittingRegion_Write(obj->line, xml, lineItem));
    kCheck(kXml_SetChildBool(xml, item, "RegionEnabled", obj->regionEnabled));
    kCheck(kXml_AddItem(xml, item, "Region", &regionItem)); 
    kCheck(GoProfileRegion_Write(obj->region, xml, regionItem));

    kCheck(GoProfileTool_Write(tool, xml, item)); 

    return kOK;
}

GoFx(GoProfileBaseline) GoProfileArea_Baseline(GoProfileArea tool)
{
    kObj(GoProfileArea, tool); 
    return obj->baseline;
}

GoFx(kStatus) GoProfileArea_SetBaseline(GoProfileArea tool, GoProfileBaseline type)
{
    kObj(GoProfileArea, tool); 

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    obj->baseline = type;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(kBool) GoProfileArea_BaselineUsed(GoProfileArea tool)
{
    kObj(GoProfileArea, tool); 

    return obj->baselineUsed;
}

GoFx(GoProfileLineFittingRegion) GoProfileArea_LineRegion(GoProfileArea tool)
{
    kObj(GoProfileArea, tool); 
    return obj->line;
}

GoFx(GoProfileAreaType) GoProfileArea_Type(GoProfileArea tool)
{
    kObj(GoProfileArea, tool); 
    return obj->type;
}

GoFx(kStatus) GoProfileArea_SetType(GoProfileArea tool, GoProfileAreaType type)
{
    kObj(GoProfileArea, tool); 

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    obj->type = type;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(kBool) GoProfileArea_TypeUsed(GoProfileArea tool)
{
    kObj(GoProfileArea, tool); 

    return obj->typeUsed;
}

GoFx(GoProfileRegion) GoProfileArea_Region(GoProfileArea tool)
{
    kObj(GoProfileArea, tool); 
    return obj->region;
}

GoFx(kBool) GoProfileArea_RegionEnabled(GoProfileArea tool)
{
    kObj(GoProfileArea, tool);

    return obj->regionEnabled;
}

GoFx(kStatus) GoProfileArea_EnableRegion(GoProfileArea tool, kBool enable)
{
    kObj(GoProfileArea, tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    obj->regionEnabled = enable;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(GoProfileAreaArea) GoProfileArea_AreaMeasurement(GoProfileArea tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_PROFILE_AREA_AREA);
}

GoFx(GoProfileAreaCentroidX) GoProfileArea_CentroidXMeasurement(GoProfileArea tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_PROFILE_AREA_CENTROID_X);
}

GoFx(GoProfileAreaCentroidX) GoProfileArea_CentroidZMeasurement(GoProfileArea tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_PROFILE_AREA_CENTROID_Z);
}

GoFx(GoProfileAreaCenterPoint) GoProfileArea_CenterPoint(GoProfileArea tool)
{
    return GoTool_FindFeatureOutputByType(tool, GO_FEATURE_PROFILE_AREA_CENTER_POINT);
}


kBeginClassEx(Go, GoProfileBox)
kAddVMethod(GoProfileBox, kObject, VRelease)
kAddVMethod(GoProfileBox, GoTool, VInit)
kAddVMethod(GoProfileBox, GoTool, VRead)
kAddVMethod(GoProfileBox, GoTool, VWrite)
kEndClassEx()

GoFx(kStatus) GoProfileBox_Construct(GoProfileBox* tool, kObject sensor, kAlloc allocator)
{
    return GoTool_Construct(tool, kTypeOf(GoProfileBox), sensor, allocator);
}

GoFx(kStatus) GoProfileBox_VInit(GoProfileBox tool, kType type, kObject sensor, kAlloc alloc)
{
    kObjR(GoProfileBox, tool);

    kCheck(GoProfileTool_Init(tool, type, GO_TOOL_PROFILE_BOUNDING_BOX, sensor, alloc)); 
    obj->regionEnabled = kFALSE;
    obj->region = kNULL;

    kCheck(GoProfileRegion_Construct(&obj->region, sensor, alloc));

    obj->regionEnabled = kTRUE;

    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoProfileBoxX), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoProfileBoxZ), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoProfileBoxWidth), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoProfileBoxHeight), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoProfileBoxGlobalX), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoProfileBoxGlobalY), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoProfileBoxGlobalAngle), kTRUE, kNULL));

    kCheck(GoTool_AddFeatureOutput(tool, kTypeOf(GoProfileBoundingBoxCenterPoint), kNULL));
    kCheck(GoTool_AddFeatureOutput(tool, kTypeOf(GoProfileBoundingBoxCornerPoint), kNULL));

    return kOK; 
}

GoFx(kStatus) GoProfileBox_VRelease(GoProfileBox tool)
{
    kObj(GoProfileBox, tool);

    kDestroyRef(&obj->region);

    return GoProfileTool_VRelease(tool);
}

GoFx(kStatus) GoProfileBox_VRead(GoProfileBox tool, kXml xml, kXmlItem item)
{
    kObj(GoProfileBox, tool);
    kXmlItem regionItem = kNULL;

    kCheck(kXml_ChildBool(xml, item, "RegionEnabled", &obj->regionEnabled));
    
    kCheck(!kIsNull(regionItem = kXml_Child(xml, item, "Region")));
    kCheck(GoProfileRegion_Read(obj->region, xml, regionItem));

    kCheck(GoProfileTool_Read(tool, xml, item)); 

    return kOK;
}

GoFx(kStatus) GoProfileBox_VWrite(GoProfileBox tool, kXml xml, kXmlItem item)
{
    kObj(GoProfileBox, tool);
    kXmlItem regionItem = kNULL;

    kCheck(kXml_SetChildBool(xml, item, "RegionEnabled", obj->regionEnabled));

    kCheck(kXml_AddItem(xml, item, "Region", &regionItem));
    kCheck(GoProfileRegion_Write(obj->region, xml, regionItem));

    kCheck(GoProfileTool_Write(tool, xml, item)); 

    return kOK;
}

GoFx(kBool) GoProfileBox_RegionEnabled(GoProfileBox tool)
{
    kObj(GoProfileBox, tool);

    return obj->regionEnabled;
}

GoFx(kStatus) GoProfileBox_EnableRegion(GoProfileBox tool, kBool enable)
{
    kObj(GoProfileBox, tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    obj->regionEnabled = enable;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(GoProfileRegion) GoProfileBox_Region(GoProfileBox tool)
{
    kObj(GoProfileBox, tool);

    return obj->region;
}

GoFx(GoProfileBoxX) GoProfileBox_XMeasurement(GoProfileBox tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_PROFILE_BOUNDING_BOX_X);
}

GoFx(GoProfileBoxZ) GoProfileBox_ZMeasurement(GoProfileBox tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_PROFILE_BOUNDING_BOX_Z);
}

GoFx(GoProfileBoxWidth) GoProfileBox_WidthMeasurement(GoProfileBox tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_PROFILE_BOUNDING_BOX_WIDTH);
}

GoFx(GoProfileBoxHeight) GoProfileBox_HeightMeasurement(GoProfileBox tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_PROFILE_BOUNDING_BOX_HEIGHT);
}

GoFx(GoProfileBoxGlobalX) GoProfileBox_GlobalXMeasurement(GoProfileBox tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_PROFILE_BOUNDING_BOX_GLOBAL_X);
}

GoFx(GoProfileBoxGlobalY) GoProfileBox_GlobalYMeasurement(GoProfileBox tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_PROFILE_BOUNDING_BOX_GLOBAL_Y);
}

GoFx(GoProfileBoxGlobalAngle) GoProfileBox_GlobalAngleMeasurement(GoProfileBox tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_PROFILE_BOUNDING_BOX_GLOBAL_ANGLE);
}

GoFx(GoProfileBoundingBoxCornerPoint) GoProfileBox_CornerPoint(GoProfileBox tool)
{
    return GoTool_FindFeatureOutputByType(tool, GO_FEATURE_PROFILE_BOUNDING_BOX_CORNER_POINT);
}

GoFx(GoProfileBoundingBoxCenterPoint) GoProfileBox_CenterPoint(GoProfilePosition tool)
{
    return GoTool_FindFeatureOutputByType(tool, GO_FEATURE_PROFILE_BOUNDING_BOX_CENTER_POINT);
}

kBeginClassEx(Go, GoProfileBridgeValue)
    kAddVMethod(GoProfileBridgeValue, kObject, VRelease)
    kAddVMethod(GoProfileBridgeValue, GoTool, VInit)
    kAddVMethod(GoProfileBridgeValue, GoTool, VRead)
    kAddVMethod(GoProfileBridgeValue, GoTool, VWrite)
kEndClassEx()

GoFx(kStatus) GoProfileBridgeValue_Construct(GoProfileBridgeValue* tool, kObject sensor, kAlloc allocator)
{
    return GoTool_Construct(tool, kTypeOf(GoProfileBridgeValue), sensor, allocator);
}

GoFx(kStatus) GoProfileBridgeValue_VInit(GoProfileBridgeValue tool, kType type, kObject sensor, kAlloc alloc)
{
    kObjR(GoProfileBridgeValue, tool);

    kCheck(GoProfileTool_Init(tool, type, GO_TOOL_PROFILE_BRIDGE_VALUE, sensor, alloc));
    obj->regionEnabled = kFALSE;
    obj->region = kNULL;
    obj->normalizeEnabled = kFALSE;
    obj->windowSize = 0.0;
    obj->windowSkip = 0.0;
    obj->maxInvalid = 0.0;
    kZero(obj->maxDifferential);

    kCheck(GoProfileRegion_Construct(&obj->region, sensor, alloc));

    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoProfileBridgeValueBridgeValue), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoProfileBridgeValueAngle), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoProfileBridgeValueWindow), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoProfileBridgeValueStdDev), kTRUE, kNULL));

    return kOK;
}

GoFx(kStatus) GoProfileBridgeValue_VRelease(GoProfileBridgeValue tool)
{
    kObj(GoProfileBridgeValue, tool);

    kCheck(kDestroyRef(&obj->region)); 

    return GoProfileTool_VRelease(tool);
}

GoFx(kStatus) GoProfileBridgeValue_VRead(GoProfileBridgeValue tool, kXml xml, kXmlItem item)
{
    kObj(GoProfileBridgeValue, tool);
    kXmlItem regionItem = kNULL;
    kXmlItem diffItem = kNULL;

    kCheck(kXml_Child64f(xml, item, "WindowSize", &obj->windowSize));
    kCheck(kXml_Child64f(xml, item, "WindowSkip", &obj->windowSkip));
    kCheck(kXml_Child64f(xml, item, "MaxInvalid", &obj->maxInvalid));
    kCheck(kXml_ChildBool(xml, item, "NormalizeEnabled", &obj->normalizeEnabled));

    kCheck(kXml_ChildBool(xml, item, "RegionEnabled", &obj->regionEnabled));

    kCheck(GoConfig_ReadRangeElement64f(xml, item, "MaxDifferential", &obj->maxDifferential));

    kCheck(!kIsNull(regionItem = kXml_Child(xml, item, "Region")));
    kCheck(GoProfileRegion_Read(obj->region, xml, regionItem));
    
    kCheck(GoProfileTool_Read(tool, xml, item));

    return kOK;
}

GoFx(kStatus) GoProfileBridgeValue_VWrite(GoProfileBridgeValue tool, kXml xml, kXmlItem item)
{
    kObj(GoProfileBridgeValue, tool);
    kXmlItem regionItem = kNULL;

    kCheck(kXml_SetChild64f(xml, item, "WindowSize", obj->windowSize));
    kCheck(kXml_SetChild64f(xml, item, "WindowSkip", obj->windowSkip));
    kCheck(kXml_SetChild64f(xml, item, "MaxInvalid", obj->maxInvalid));
    kCheck(GoConfig_WriteRangeElement64f(xml, item, "MaxDifferential", obj->maxDifferential));
    kCheck(kXml_SetChildBool(xml, item, "NormalizeEnabled", obj->normalizeEnabled));

    kCheck(kXml_SetChildBool(xml, item, "RegionEnabled", obj->regionEnabled));

    kCheck(kXml_AddItem(xml, item, "Region", &regionItem));
    kCheck(GoProfileRegion_Write(obj->region, xml, regionItem));

    kCheck(GoProfileTool_Write(tool, xml, item));

    return kOK;
}

GoFx(kBool) GoProfileBridgeValue_RegionEnabled(GoProfileBridgeValue tool)
{
    kObj(GoProfileBridgeValue, tool);

    return obj->regionEnabled;
}

GoFx(kStatus) GoProfileBridgeValue_EnableRegion(GoProfileBridgeValue tool, kBool enable)
{
    kObj(GoProfileBridgeValue, tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    obj->regionEnabled = enable;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(GoProfileRegion) GoProfileBridgeValue_Region(GoProfileBridgeValue tool)
{
    kObj(GoProfileBridgeValue, tool);
    return obj->region;
}

GoFx(kBool) GoProfileBridgeValue_NormalizeEnabled(GoProfileBridgeValue tool)
{
    kObj(GoProfileBridgeValue, tool);

    return obj->normalizeEnabled;
}

GoFx(kStatus) GoProfileBridgeValue_EnableNormalize(GoProfileBridgeValue tool, kBool enable)
{
    kObj(GoProfileBridgeValue, tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    obj->normalizeEnabled = enable;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(kStatus) GoProfileBridgeValue_SetWindowSize(GoProfileBridgeValue tool, k64f value)
{
    kObj(GoProfileBridgeValue, tool);

    obj->windowSize = value;

    return kOK;
}

GoFx(k64f) GoProfileBridgeValue_WindowSize(GoProfileBridgeValue tool)
{
    kObj(GoProfileBridgeValue, tool);

    return obj->windowSize;
}

GoFx(kStatus) GoProfileBridgeValue_SetWindowSkip(GoProfileBridgeValue tool, k64f value)
{
    kObj(GoProfileBridgeValue, tool);

    obj->windowSkip = value;

    return kOK;
}

GoFx(k64f) GoProfileBridgeValue_WindowSkip(GoProfileBridgeValue tool)
{
    kObj(GoProfileBridgeValue, tool);

    return obj->windowSkip;
}

GoFx(kStatus) GoProfileBridgeValue_SetMaxInvalid(GoProfileBridgeValue tool, k64f value)
{
    kObj(GoProfileBridgeValue, tool);

    obj->maxInvalid = value;

    return kOK;
}

GoFx(k64f) GoProfileBridgeValue_MaxInvalid(GoProfileBridgeValue tool)
{
    kObj(GoProfileBridgeValue, tool);

    return obj->maxInvalid;
}


GoFx(kStatus) GoProfileBridgeValue_SetMaxDifferential(GoProfileBridgeValue tool, k64f value)
{
    kObj(GoProfileBridgeValue, tool);

    kCheckArgs(value >= obj->maxDifferential.min && value <= obj->maxDifferential.max);

    obj->maxDifferential.value = value;

    return kOK;
}

GoFx(k64f) GoProfileBridgeValue_MaxDifferential(GoProfileBridgeValue tool)
{
    kObj(GoProfileBridgeValue, tool);

    return obj->maxDifferential.value;
}

GoFx(k64f) GoProfileBridgeValue_MaxDifferentialLimitMax(GoProfileBridgeValue tool)
{
    kObj(GoProfileBridgeValue, tool);

    return obj->maxDifferential.max;
}

GoFx(k64f) GoProfileBridgeValue_MaxDifferentialLimitMin(GoProfileBridgeValue tool)
{
    kObj(GoProfileBridgeValue, tool);

    return obj->maxDifferential.min;
}

GoFx(GoProfileBridgeValueBridgeValue) GoProfileBridgeValue_BridgeValueMeasurement(GoProfileBridgeValue tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_PROFILE_BRIDGE_VALUE_BRIDGE_VALUE);
}

GoFx(GoProfileBridgeValueAngle) GoProfileBridgeValue_AngleMeasurement(GoProfileBridgeValue tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_PROFILE_BRIDGE_VALUE_ANGLE);
}

GoFx(GoProfileBridgeValueWindow) GoProfileBridgeValue_WindowMeasurement(GoProfileBridgeValue tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_PROFILE_BRIDGE_VALUE_WINDOW);
}

GoFx(GoProfileBridgeValueStdDev) GoProfileBridgeValue_StdDevMeasurement(GoProfileBridgeValue tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_PROFILE_BRIDGE_VALUE_STDDEV);
}

kBeginClassEx(Go, GoProfileCircle)
kAddVMethod(GoProfileCircle, kObject, VRelease)
kAddVMethod(GoProfileCircle, GoTool, VInit)
kAddVMethod(GoProfileCircle, GoTool, VRead)
kAddVMethod(GoProfileCircle, GoTool, VWrite)
kEndClassEx()

GoFx(kStatus) GoProfileCircle_Construct(GoProfileCircle* tool, kObject sensor, kAlloc allocator)
{
    return GoTool_Construct(tool, kTypeOf(GoProfileCircle), sensor, allocator);
}

GoFx(kStatus) GoProfileCircle_VInit(GoProfileCircle tool, kType type, kObject sensor, kAlloc alloc)
{
    kObjR(GoProfileCircle, tool); 

    kCheck(GoProfileTool_Init(tool, type, GO_TOOL_PROFILE_CIRCLE, sensor, alloc)); 
    obj->regionEnabled = kFALSE;
    obj->region = kNULL;

    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoProfileCircleX), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoProfileCircleZ), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoProfileCircleRadius), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoProfileCircleStdDev), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoProfileCircleMinError), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoProfileCircleMinErrorX), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoProfileCircleMinErrorZ), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoProfileCircleMaxError), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoProfileCircleMaxErrorX), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoProfileCircleMaxErrorZ), kTRUE, kNULL));

    kCheck(GoTool_AddFeatureOutput(tool, kTypeOf(GoProfileCircleCenterPoint), kNULL));

    kCheck(GoProfileRegion_Construct(&obj->region, sensor, alloc)); 

    return kOK; 
}

GoFx(kStatus) GoProfileCircle_VRelease(GoProfileCircle tool)
{
    kObj(GoProfileCircle, tool); 

    kCheck(kDestroyRef(&obj->region));  


    return GoProfileTool_VRelease(tool); 
}

GoFx(kStatus) GoProfileCircle_VRead(GoProfileCircle tool, kXml xml, kXmlItem item)
{
    kObj(GoProfileCircle, tool); 
    kXmlItem regionItem;

    kCheck(GoProfileTool_Read(tool, xml, item)); 

    if (kXml_ChildExists(xml, item, "RegionEnabled"))
    {
        kCheck(kXml_ChildBool(xml, item, "RegionEnabled", &obj->regionEnabled));
    }

    kCheck(!kIsNull(regionItem = kXml_Child(xml, item, "Region"))); 
    kCheck(GoProfileRegion_Read(obj->region, xml, regionItem));

    return kOK;
}

GoFx(kStatus) GoProfileCircle_VWrite(GoProfileCircle tool, kXml xml, kXmlItem item)
{
    kObj(GoProfileCircle, tool); 
    kXmlItem regionItem;

    kCheck(kXml_SetChildBool(xml, item, "RegionEnabled", obj->regionEnabled));
    kCheck(kXml_AddItem(xml, item, "Region", &regionItem)); 
    kCheck(GoProfileRegion_Write(obj->region, xml, regionItem));

    kCheck(GoProfileTool_Write(tool, xml, item)); 

    return kOK;
}

GoFx(GoProfileRegion) GoProfileCircle_Region(GoProfileCircle tool)
{
    kObj(GoProfileCircle, tool); 
    return obj->region;
}

GoFx(kBool) GoProfileCircle_RegionEnabled(GoProfileCircle tool)
{
    kObj(GoProfileCircle, tool);

    return obj->regionEnabled;
}

GoFx(kStatus) GoProfileCircle_EnableRegion(GoProfileCircle tool, kBool enable)
{
    kObj(GoProfileCircle, tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    obj->regionEnabled = enable;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(GoProfileCircleX) GoProfileCircle_XMeasurement(GoProfileCircle tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_PROFILE_CIRCLE_X);
}

GoFx(GoProfileCircleZ) GoProfileCircle_ZMeasurement(GoProfileCircle tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_PROFILE_CIRCLE_Z);
}

GoFx(GoProfileCircleRadius) GoProfileCircle_RadiusMeasurement(GoProfileCircle tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_PROFILE_CIRCLE_RADIUS);
}

GoFx(GoProfileCircleStdDev) GoProfileCircle_StdDevMeasurement(GoProfileCircle tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_PROFILE_CIRCLE_STDDEV);
}

GoFx(GoProfileCircleMinError) GoProfileCircle_MinErrorMeasurement(GoProfileCircle tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_PROFILE_CIRCLE_MIN_ERROR);
}

GoFx(GoProfileCircleMinErrorX) GoProfileCircle_MinErrorXMeasurement(GoProfileCircle tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_PROFILE_CIRCLE_MIN_ERROR_X);
}

GoFx(GoProfileCircleMinErrorZ) GoProfileCircle_MinErrorZMeasurement(GoProfileCircle tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_PROFILE_CIRCLE_MIN_ERROR_Z);
}

GoFx(GoProfileCircleMaxError) GoProfileCircle_MaxErrorMeasurement(GoProfileCircle tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_PROFILE_CIRCLE_MAX_ERROR);
}

GoFx(GoProfileCircleMaxErrorX) GoProfileCircle_MaxErrorXMeasurement(GoProfileCircle tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_PROFILE_CIRCLE_MAX_ERROR_X);
}

GoFx(GoProfileCircleMaxErrorZ) GoProfileCircle_MaxErrorZMeasurement(GoProfileCircle tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_PROFILE_CIRCLE_MAX_ERROR_Z);
}

GoFx(GoProfileCircleCenterPoint) GoProfileCircle_CenterPoint(GoProfileCircle tool)
{
    return GoTool_FindFeatureOutputByType(tool, GO_FEATURE_PROFILE_CIRCLE_CENTER_POINT);
}

kBeginClassEx(Go, GoProfileDim)
kAddVMethod(GoProfileDim, kObject, VRelease)
kAddVMethod(GoProfileDim, GoTool, VInit)
kAddVMethod(GoProfileDim, GoTool, VRead)
kAddVMethod(GoProfileDim, GoTool, VWrite)
kEndClassEx()

GoFx(kStatus) GoProfileDim_Construct(GoProfileDim* tool, kObject sensor, kAlloc allocator)
{
    return GoTool_Construct(tool, kTypeOf(GoProfileDim), sensor, allocator);
}

GoFx(kStatus) GoProfileDim_VInit(GoProfileDim tool, kType type, kObject sensor, kAlloc alloc)
{
    kObjR(GoProfileDim, tool); 

    kCheck(GoProfileTool_Init(tool, type, GO_TOOL_PROFILE_DIMENSION, sensor, alloc)); 
    obj->refFeature = kNULL;
    obj->feature = kNULL;

    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoProfileDimWidth), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoProfileDimHeight), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoProfileDimDistance), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoProfileDimCenterX), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoProfileDimCenterZ), kTRUE, kNULL));

    kCheck(GoProfileFeature_Construct(&obj->refFeature, sensor, alloc));
    kCheck(GoProfileFeature_Construct(&obj->feature, sensor, alloc));

    return kOK; 
}

GoFx(kStatus) GoProfileDim_VRelease(GoProfileDim tool)
{
    kObj(GoProfileDim, tool);

    kCheck(kDestroyRef(&obj->refFeature));
    kCheck(kDestroyRef(&obj->feature));

    return GoProfileTool_VRelease(tool); 
}

GoFx(kStatus) GoProfileDim_VRead(GoProfileDim tool, kXml xml, kXmlItem item)
{
    kObj(GoProfileDim, tool);
    kXmlItem refFeatureItem;
    kXmlItem featureItem;

    kCheck(GoProfileTool_Read(tool, xml, item)); 

    kCheck(!kIsNull(refFeatureItem  = kXml_Child(xml, item, "RefFeature"))); 
    kCheck(GoProfileFeature_Read(obj->refFeature, xml, refFeatureItem));
    kCheck(!kIsNull(featureItem = kXml_Child(xml, item, "Feature"))); 
    kCheck(GoProfileFeature_Read(obj->feature, xml, featureItem));

    return kOK;
}

GoFx(kStatus) GoProfileDim_VWrite(GoProfileDim tool, kXml xml, kXmlItem item)
{
    kObj(GoProfileDim, tool);
    kXmlItem refFeatureItem;
    kXmlItem featureItem;

    kCheck(kXml_AddItem(xml, item, "RefFeature", &refFeatureItem)); 
    kCheck(GoProfileFeature_Write(obj->refFeature, xml, refFeatureItem));
    kCheck(kXml_AddItem(xml, item, "Feature", &featureItem)); 
    kCheck(GoProfileFeature_Write(obj->feature, xml, featureItem));

    kCheck(GoProfileTool_Write(tool, xml, item)); 

    return kOK;
}

GoFx(GoProfileFeature) GoProfileDim_RefFeature(GoProfileDim tool)
{
    kObj(GoProfileDim, tool);
    return obj->refFeature;
}

GoFx(GoProfileFeature) GoProfileDim_Feature(GoProfileDim tool)
{
    kObj(GoProfileDim, tool);
    return obj->feature;
}

GoFx(GoMeasurement) GoProfileDim_WidthMeasurement(GoProfileDim tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_PROFILE_DIMENSION_WIDTH);
}

GoFx(GoMeasurement) GoProfileDim_HeightMeasurement(GoProfileDim tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_PROFILE_DIMENSION_HEIGHT);
}

GoFx(GoMeasurement) GoProfileDim_DistanceMeasurement(GoProfileDim tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_PROFILE_DIMENSION_DISTANCE);
}

GoFx(GoMeasurement) GoProfileDim_CenterXMeasurement(GoProfileDim tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_PROFILE_DIMENSION_CENTER_X);
}

GoFx(GoMeasurement) GoProfileDim_CenterZMeasurement(GoProfileDim tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_PROFILE_DIMENSION_CENTER_Z);
}

GoFx(GoProfileDimensionCenterPoint) GoProfileDim_CenterPoint(GoProfileDim tool)
{
    return GoTool_FindFeatureOutputByType(tool, GO_FEATURE_PROFILE_DIMENSION_CENTER_POINT);
}

kBeginClassEx(Go, GoProfileGroove)
kAddVMethod(GoProfileGroove, kObject, VRelease)
kAddVMethod(GoProfileGroove, GoTool, VInit)
kAddVMethod(GoProfileGroove, GoTool, VRead)
kAddVMethod(GoProfileGroove, GoTool, VWrite)
kEndClassEx()

GoFx(kStatus) GoProfileGroove_Construct(GoProfileGroove* tool, kObject sensor, kAlloc allocator)
{
    return GoTool_Construct(tool, kTypeOf(GoProfileGroove), sensor, allocator);
}

GoFx(kStatus) GoProfileGroove_VInit(GoProfileGroove tool, kType type, kObject sensor, kAlloc alloc)
{
    kObjR(GoProfileGroove, tool); 

    kCheck(GoProfileTool_Init(tool, type, GO_TOOL_PROFILE_GROOVE, sensor, alloc)); 
    obj->regionEnabled = kFALSE;
    obj->region = kNULL;

    obj->shape = GO_PROFILE_GROOVE_SHAPE_U;
    obj->minWidth = 0;
    obj->maxWidth = 0;
    obj->minDepth = 0;

    kCheck(GoProfileRegion_Construct(&obj->region, sensor, alloc)); 

    return kOK; 
}

GoFx(kStatus) GoProfileGroove_VRelease(GoProfileGroove tool)
{
    kObjR(GoProfileGroove, tool);

    kCheck(kDestroyRef(&obj->region)); 

    return GoProfileTool_VRelease(tool); 
}

GoFx(kStatus) GoProfileGroove_VRead(GoProfileGroove tool, kXml xml, kXmlItem item)
{
    kObj(GoProfileGroove, tool);
    kXmlItem regionItem;

    kCheck(GoProfileTool_Read(tool, xml, item)); 

    if (kXml_ChildExists(xml, item, "RegionEnabled"))
    {
        kCheck(kXml_ChildBool(xml, item, "RegionEnabled", &obj->regionEnabled));
    }

    kCheck(!kIsNull(regionItem = kXml_Child(xml, item, "Region"))); 
    kCheck(GoProfileRegion_Read(obj->region, xml, regionItem));

    kCheck(kXml_Child32s(xml, item, "Shape", &obj->shape));    
    kCheck(kXml_Child64f(xml, item, "MinWidth", &obj->minWidth));
    kCheck(kXml_Child64f(xml, item, "MaxWidth", &obj->maxWidth));
    kCheck(kXml_Child64f(xml, item, "MinDepth", &obj->minDepth));    

    return kOK;
}

GoFx(kStatus) GoProfileGroove_VWrite(GoProfileGroove tool, kXml xml, kXmlItem item)
{
    kObj(GoProfileGroove, tool);
    kXmlItem regionItem;

    kCheck(GoProfileTool_Write(tool, xml, item)); 
    kCheck(kXml_SetChildBool(xml, item, "RegionEnabled", obj->regionEnabled));
    kCheck(kXml_AddItem(xml, item, "Region", &regionItem)); 
    kCheck(GoProfileRegion_Write(obj->region, xml, regionItem));    

    kCheck(kXml_SetChild32s(xml, item, "Shape", obj->shape));    
    kCheck(kXml_SetChild64f(xml, item, "MinWidth", obj->minWidth));
    kCheck(kXml_SetChild64f(xml, item, "MaxWidth", obj->maxWidth));       
    kCheck(kXml_SetChild64f(xml, item, "MinDepth", obj->minDepth));       

    return kOK;
}

GoFx(GoProfileGrooveShape) GoProfileGroove_Shape(GoProfileGroove tool)
{
    kObj(GoProfileGroove, tool);
    return obj->shape;
}

GoFx(kStatus) GoProfileGroove_SetShape(GoProfileGroove tool, GoProfileGrooveShape shape)
{
    kObj(GoProfileGroove, tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    obj->shape = shape;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(k64f) GoProfileGroove_MinDepth(GoProfileGroove tool)
{
    kObjR(GoProfileGroove, tool);
    return obj->maxWidth;
}

GoFx(kStatus) GoProfileGroove_SetMinDepth(GoProfileGroove tool, k64f width)
{
    kObj(GoProfileGroove, tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    obj->maxWidth = width;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(k64f) GoProfileGroove_MaxWidth(GoProfileGroove tool)
{
    kObjR(GoProfileGroove, tool);
    return obj->maxWidth;
}

GoFx(kStatus) GoProfileGroove_SetMaxWidth(GoProfileGroove tool, k64f width)
{
    kObj(GoProfileGroove, tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    obj->maxWidth = width;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(k64f) GoProfileGroove_MinWidth(GoProfileGroove tool)
{
    kObjR(GoProfileGroove, tool);
    return obj->minWidth;
}

GoFx(kStatus) GoProfileGroove_SetMinWidth(GoProfileGroove tool, k64f width)
{
    kObj(GoProfileGroove, tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    obj->minWidth = width;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(GoProfileRegion) GoProfileGroove_Region(GoProfileGroove tool)
{
    kObjR(GoProfileGroove, tool);
    return obj->region;
}

GoFx(kBool) GoProfileGroove_RegionEnabled(GoProfileGroove tool)
{
    kObj(GoProfileGroove, tool);

    return obj->regionEnabled;
}

GoFx(kStatus) GoProfileGroove_EnableRegion(GoProfileGroove tool, kBool enable)
{
    kObj(GoProfileGroove, tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    obj->regionEnabled = enable;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(kStatus) GoProfileGroove_AddMeasurement(GoProfileGroove tool, GoMeasurementType type, GoMeasurement* measurement)
{
    kObj(GoProfileGroove, tool);
    kType classType = kNULL;
    kStatus status = kOK;

    if (type != GO_MEASUREMENT_PROFILE_GROOVE_X
        && type != GO_MEASUREMENT_PROFILE_GROOVE_Z
        && type != GO_MEASUREMENT_PROFILE_GROOVE_WIDTH
        && type != GO_MEASUREMENT_PROFILE_GROOVE_DEPTH)
    {
        return kERROR_PARAMETER;
    }
    else
    {
        switch(type)
        {
        case GO_MEASUREMENT_PROFILE_GROOVE_X: classType = kTypeOf(GoProfileGrooveX); break;
        case GO_MEASUREMENT_PROFILE_GROOVE_Z: classType = kTypeOf(GoProfileGrooveZ); break;
        case GO_MEASUREMENT_PROFILE_GROOVE_WIDTH: classType = kTypeOf(GoProfileGrooveWidth); break;
        case GO_MEASUREMENT_PROFILE_GROOVE_DEPTH: classType = kTypeOf(GoProfileGrooveDepth); break;
        }
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
        GoTool_RemoveMeasurement(tool, GoTool_MeasurementCount(tool) - 1);  
        kEndCatch(status);
    }

    return kOK;
}

GoFx(kStatus) GoProfileGroove_RemoveMeasurement(GoProfileGroove tool, kSize index)
{
    if (index > GoTool_MeasurementCount(tool))
    {
        return kERROR_PARAMETER;
    }

    kCheck(GoTool_RemoveMeasurement(tool, index));

    return kOK;
}

GoFx(kSize) GoProfileGroove_MeasurementCount(GoProfileGroove tool)
{
    return GoTool_MeasurementCount(tool);
}

GoFx(GoMeasurement) GoProfileGroove_MeasurementAt(GoProfileGroove tool, kSize index)
{
    kAssert(index < GoTool_MeasurementCount(tool));
    
    return GoTool_MeasurementAt(tool, index);
}


kBeginClassEx(Go, GoProfileIntersect)
kAddVMethod(GoProfileIntersect, kObject, VRelease)
kAddVMethod(GoProfileIntersect, GoTool, VInit)
kAddVMethod(GoProfileIntersect, GoTool, VRead)
kAddVMethod(GoProfileIntersect, GoTool, VWrite)
kEndClassEx()

GoFx(kStatus) GoProfileIntersect_Construct(GoProfileIntersect* tool, kObject sensor, kAlloc allocator)
{
    return GoTool_Construct(tool, kTypeOf(GoProfileIntersect), sensor, allocator);
}

GoFx(kStatus) GoProfileIntersect_VInit(GoProfileIntersect tool, kType type, kObject sensor, kAlloc alloc)
{
    kObjR(GoProfileIntersect, tool); 

    kCheck(GoProfileTool_Init(tool, type, GO_TOOL_PROFILE_INTERSECT, sensor, alloc)); 
    obj->refLineType = GO_PROFILE_BASELINE_TYPE_X_AXIS;
    obj->refLineRegion = kNULL;
    obj->lineRegion = kNULL;

    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoProfileIntersectX), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoProfileIntersectZ), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoProfileIntersectAngle), kTRUE, kNULL));

    kCheck(GoTool_AddFeatureOutput(tool, kTypeOf(GoProfileIntersectIntersectPoint), kNULL));
    kCheck(GoTool_AddFeatureOutput(tool, kTypeOf(GoProfileIntersectLine), kNULL));
    kCheck(GoTool_AddFeatureOutput(tool, kTypeOf(GoProfileIntersectBaseLine), kNULL));

    obj->refLineType = GO_PROFILE_BASELINE_TYPE_LINE;

    kCheck(GoProfileLineFittingRegion_Construct(&obj->refLineRegion, sensor, alloc)); 
    kCheck(GoProfileLineFittingRegion_Construct(&obj->lineRegion, sensor, alloc)); 

    return kOK; 
}

GoFx(kStatus) GoProfileIntersect_VRelease(GoProfileIntersect tool)
{
    kObj(GoProfileIntersect, tool);

    kCheck(kDestroyRef(&obj->refLineRegion)); 
    kCheck(kDestroyRef(&obj->lineRegion)); 

    return GoProfileTool_VRelease(tool); 
}

GoFx(kStatus) GoProfileIntersect_VRead(GoProfileIntersect tool, kXml xml, kXmlItem item)
{
    kObj(GoProfileIntersect, tool); 
    kXmlItem lineItem;

    kCheck(GoProfileTool_Read(tool, xml, item)); 

    kCheck(kXml_Child32s(xml, item, "RefType", &obj->refLineType));

    kCheck(!kIsNull(lineItem = kXml_Child(xml, item, "RefLine"))); 
    kCheck(GoProfileLineFittingRegion_Read(obj->refLineRegion, xml, lineItem));
    kCheck(!kIsNull(lineItem = kXml_Child(xml, item, "Line"))); 
    kCheck(GoProfileLineFittingRegion_Read(obj->lineRegion, xml, lineItem));

    return kOK;
}

GoFx(kStatus) GoProfileIntersect_VWrite(GoProfileIntersect tool, kXml xml, kXmlItem item)
{
    kObj(GoProfileIntersect, tool); 
    kXmlItem lineItem;

    kCheck(kXml_SetChild32s(xml, item, "RefType", obj->refLineType));

    kCheck(kXml_AddItem(xml, item, "RefLine", &lineItem)); 
    kCheck(GoProfileLineFittingRegion_Write(obj->refLineRegion, xml, lineItem));
    kCheck(kXml_AddItem(xml, item, "Line", &lineItem)); 
    kCheck(GoProfileLineFittingRegion_Write(obj->lineRegion, xml, lineItem));

    kCheck(GoProfileTool_Write(tool, xml, item)); 

    return kOK;
}

GoFx(GoProfileBaseline) GoProfileIntersect_RefLineType(GoProfileIntersect tool)
{
    kObj(GoProfileIntersect, tool); 
    return obj->refLineType;
}

GoFx(kStatus) GoProfileIntersect_SetRefLineType(GoProfileIntersect tool, GoProfileBaseline type)
{
    kObj(GoProfileIntersect, tool); 

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    obj->refLineType = type;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(GoProfileLineFittingRegion) GoProfileIntersect_RefLine(GoProfileIntersect tool)
{
    kObj(GoProfileIntersect, tool); 
    return obj->refLineRegion;
}

GoFx(GoProfileLineFittingRegion) GoProfileIntersect_Line(GoProfileIntersect tool)
{
    kObj(GoProfileIntersect, tool); 
    return obj->lineRegion;
}

GoFx(GoProfileIntersectX) GoProfileIntersect_XMeasurement(GoProfileIntersect tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_PROFILE_INTERSECT_X);
}

GoFx(GoProfileIntersectZ) GoProfileIntersect_ZMeasurement(GoProfileIntersect tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_PROFILE_INTERSECT_Z);
}

GoFx(GoProfileIntersectAngle) GoProfileIntersect_AngleMeasurement(GoProfileIntersect tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_PROFILE_INTERSECT_ANGLE);
}

GoFx(GoProfileIntersectIntersectPoint) GoProfileIntersect_PointFeature(GoProfileIntersect tool)
{
    return GoTool_FindFeatureOutputByType(tool, GO_FEATURE_PROFILE_INTERSECT_INTERSECT_POINT);
}

GoFx(GoProfileIntersectLine) GoProfileIntersect_LineFeature(GoProfileIntersect tool)
{
    return GoTool_FindFeatureOutputByType(tool, GO_FEATURE_PROFILE_INTERSECT_LINE);
}

GoFx(GoProfileIntersectBaseLine) GoProfileIntersect_BaseLineFeature(GoProfileIntersect tool)
{
    return GoTool_FindFeatureOutputByType(tool, GO_FEATURE_PROFILE_INTERSECT_BASE_LINE);
}

kBeginClassEx(Go, GoProfileLine)
    kAddVMethod(GoProfileLine, kObject, VRelease)
    kAddVMethod(GoProfileLine, GoTool, VInit)
    kAddVMethod(GoProfileLine, GoTool, VRead)
    kAddVMethod(GoProfileLine, GoTool, VWrite)
kEndClassEx()

GoFx(kStatus) GoProfileLine_Construct(GoProfileLine* tool, kObject sensor, kAlloc allocator)
{
    return GoTool_Construct(tool, kTypeOf(GoProfileLine), sensor, allocator);
}

GoFx(kStatus) GoProfileLine_VInit(GoProfileLine tool, kType type, kObject sensor, kAlloc alloc)
{
    kObjR(GoProfileLine, tool); 

    kCheck(GoProfileTool_Init(tool, type, GO_TOOL_PROFILE_LINE, sensor, alloc)); 
    obj->regionEnabled = kFALSE;
    obj->region = kNULL;
    obj->fittingRegionsEnabled = kFALSE;
    obj->fittingRegions = kNULL;

    kCheck(GoProfileRegion_Construct(&obj->region, sensor, alloc));
    obj->regionEnabled = kTRUE;
    kCheck(GoProfileLineFittingRegion_Construct(&obj->fittingRegions, sensor, alloc));
    obj->fittingRegionsEnabled = kFALSE;

    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoProfileLineStdDev), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoProfileLineMaxError), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoProfileLineMinError), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoProfileLinePercentile), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoProfileLineOffset), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoProfileLineAngle), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoProfileLineMinErrorX), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoProfileLineMinErrorZ), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoProfileLineMaxErrorX), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoProfileLineMaxErrorZ), kTRUE, kNULL));

    kCheck(GoTool_AddFeatureOutput(tool, kTypeOf(GoProfileLineLine), kNULL));
    kCheck(GoTool_AddFeatureOutput(tool, kTypeOf(GoProfileLineMinErrorPoint), kNULL));
    kCheck(GoTool_AddFeatureOutput(tool, kTypeOf(GoProfileLineMaxErrorPoint), kNULL));

    return kOK; 
}

GoFx(kStatus) GoProfileLine_VRelease(GoProfileLine tool)
{
    kObj(GoProfileLine, tool);

    kCheck(kDestroyRef(&obj->region));
    kCheck(kDestroyRef(&obj->fittingRegions));

    return GoProfileTool_VRelease(tool); 
}

GoFx(kStatus) GoProfileLine_VRead(GoProfileLine tool, kXml xml, kXmlItem item)
{
    kObj(GoProfileLine, tool);
    kXmlItem regionItem;

    if (kXml_ChildExists(xml, item, "RegionEnabled"))
    {
        kCheck(kXml_ChildBool(xml, item, "RegionEnabled", &obj->regionEnabled));
    }

    kCheck(!kIsNull(regionItem = kXml_Child(xml, item, "Region"))); 
    kCheck(GoProfileRegion_Read(obj->region, xml, regionItem));


    if (kXml_ChildExists(xml, item, "FittingRegionsEnabled"))
    {
        kCheck(kXml_ChildBool(xml, item, "FittingRegionsEnabled", &obj->fittingRegionsEnabled));
    }
    if (!kIsNull(regionItem = kXml_Child(xml, item, "FittingRegions")))
    {
        kCheck(GoProfileLineFittingRegion_Read(obj->fittingRegions, xml, regionItem));
    }

    kCheck(GoProfileTool_Read(tool, xml, item));     

    return kOK;
}

GoFx(kStatus) GoProfileLine_VWrite(GoProfileLine tool, kXml xml, kXmlItem item)
{
    kObj(GoProfileLine, tool);
    kXmlItem regionItem;

    kCheck(kXml_SetChildBool(xml, item, "RegionEnabled", obj->regionEnabled));
    kCheck(kXml_AddItem(xml, item, "Region", &regionItem)); 
    kCheck(GoProfileRegion_Write(obj->region, xml, regionItem));
    kCheck(kXml_SetChildBool(xml, item, "FittingRegionsEnabled", obj->fittingRegionsEnabled));
    kCheck(kXml_AddItem(xml, item, "FittingRegions", &regionItem));
    kCheck(GoProfileLineFittingRegion_Write(obj->fittingRegions, xml, regionItem));

    kCheck(GoProfileTool_Write(tool, xml, item));        

    return kOK;
}

GoFx(GoProfileRegion) GoProfileLine_Region(GoProfileLine tool)
{
    kObj(GoProfileLine, tool);
    return obj->region;
}

GoFx(kBool) GoProfileLine_RegionEnabled(GoProfileLine tool)
{
    kObj(GoProfileLine, tool);

    return obj->regionEnabled;
}

GoFx(kStatus) GoProfileLine_EnableRegion(GoProfileLine tool, kBool enable)
{
    kObj(GoProfileLine, tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    obj->regionEnabled = enable;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(kBool) GoProfileLine_FittingRegionsEnabled(GoProfileLine tool)
{
    kObj(GoProfileLine, tool);

    return obj->fittingRegionsEnabled;
}

GoFx(kStatus) GoProfileLine_EnableFittingRegions(GoProfileLine tool, kBool enable)
{
    kObj(GoProfileLine, tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    obj->fittingRegionsEnabled = enable;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(GoProfileLineFittingRegion) GoProfileLine_FittingRegions(GoProfileLine tool)
{
    kObj(GoProfileLine, tool);
    return obj->fittingRegions;
}

GoFx(GoProfileLineStdDev) GoProfileLine_StdDevMeasurement(GoProfileLine tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_PROFILE_LINE_STDDEV);
}

GoFx(GoProfileLineMaxError) GoProfileLine_MaxErrorMeasurement(GoProfileLine tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_PROFILE_LINE_ERROR_MAX);
}

GoFx(GoProfileLineMinError) GoProfileLine_MinErrorMeasurement(GoProfileLine tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_PROFILE_LINE_ERROR_MIN);
}

GoFx(GoProfileLinePercentile) GoProfileLine_PercentileMeasurement(GoProfileLine tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_PROFILE_LINE_PERCENTILE);
}

GoFx(GoProfileLineOffset) GoProfileLine_OffsetMeasurement(GoProfileLine tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_PROFILE_LINE_OFFSET);
}
GoFx(GoProfileLineAngle) GoProfileLine_AngleMeasurement(GoProfileLine tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_PROFILE_LINE_ANGLE);
}
GoFx(GoProfileLineMinErrorX) GoProfileLine_MinErrorXMeasurement(GoProfileLine tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_PROFILE_LINE_ERROR_MIN_X);
}
GoFx(GoProfileLineMinErrorZ) GoProfileLine_MinErrorZMeasurement(GoProfileLine tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_PROFILE_LINE_ERROR_MIN_Z);
}
GoFx(GoProfileLineMaxErrorX) GoProfileLine_MaxErrorXMeasurement(GoProfileLine tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_PROFILE_LINE_ERROR_MAX_X);
}
GoFx(GoProfileLineMaxErrorZ) GoProfileLine_MaxErrorZMeasurement(GoProfileLine tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_PROFILE_LINE_ERROR_MAX_Z);
}
    
GoFx(GoProfileLineLine) GoProfileLine_Line(GoProfilePosition tool)
{
    return GoTool_FindFeatureOutputByType(tool, GO_FEATURE_PROFILE_LINE_LINE);
}

GoFx(GoProfileLineMinErrorPoint) GoProfileLine_MinErrorPoint(GoProfilePosition tool)
{
    return GoTool_FindFeatureOutputByType(tool, GO_FEATURE_PROFILE_LINE_MIN_ERROR_POINT);
}

GoFx(GoProfileLineMaxErrorPoint) GoProfileLine_MaxErrorPoint(GoProfilePosition tool)
{
    return GoTool_FindFeatureOutputByType(tool, GO_FEATURE_PROFILE_LINE_MAX_ERROR_POINT);
}


kBeginClassEx(Go, GoProfilePanel)
kAddVMethod(GoProfilePanel, kObject, VRelease)
kAddVMethod(GoProfilePanel, GoTool, VInit)
kAddVMethod(GoProfilePanel, GoTool, VRead)
kAddVMethod(GoProfilePanel, GoTool, VWrite)
kEndClassEx()

GoFx(kStatus) GoProfilePanel_Construct(GoProfilePanel* tool, kObject sensor, kAlloc allocator)
{
    return GoTool_Construct(tool, kTypeOf(GoProfilePanel), sensor, allocator);
}

GoFx(kStatus) GoProfilePanel_VInit(GoProfilePanel tool, kType type, kObject sensor, kAlloc alloc)
{
    kObjR(GoProfilePanel, tool); 

    kCheck(GoProfileTool_Init(tool, type, GO_TOOL_PROFILE_PANEL, sensor, alloc)); 
    obj->refEdgeSide = GO_PROFILE_PANEL_SIDE_LEFT;
    obj->maxGapWidth = 0.0;
    obj->leftEdge = kNULL;
    obj->rightEdge = kNULL;

    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoProfilePanelGap), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoProfilePanelFlush), kTRUE, kNULL));

    obj->maxGapWidth = 0;
    obj->refEdgeSide = GO_PROFILE_PANEL_SIDE_LEFT;

    kCheck(GoProfileEdge_Construct(&obj->leftEdge, sensor, alloc)); 
    kCheck(GoProfileEdge_Construct(&obj->rightEdge, sensor, alloc)); 

    return kOK; 
}

GoFx(kStatus) GoProfilePanel_VRelease(GoProfilePanel tool)
{
    kObj(GoProfilePanel, tool);

    kCheck(kDestroyRef(&obj->leftEdge)); 
    kCheck(kDestroyRef(&obj->rightEdge)); 

    return GoProfileTool_VRelease(tool); 
}

GoFx(kStatus) GoProfilePanel_VRead(GoProfilePanel tool, kXml xml, kXmlItem item)
{
    kObj(GoProfilePanel, tool);
    kXmlItem edgeItem;

    kCheck(GoProfileTool_Read(tool, xml, item)); 

    kCheck(kXml_Child32s(xml, item, "RefSide", &obj->refEdgeSide));
    kCheck(kXml_Child64f(xml, item, "MaxGapWidth", &obj->maxGapWidth));

    kCheck(!kIsNull(edgeItem = kXml_Child(xml, item, "LeftEdge"))); 
    kCheck(GoProfileEdge_Read(obj->leftEdge, xml, edgeItem));
    kCheck(!kIsNull(edgeItem = kXml_Child(xml, item, "RightEdge"))); 
    kCheck(GoProfileEdge_Read(obj->rightEdge, xml, edgeItem));

    return kOK;
}

GoFx(kStatus) GoProfilePanel_VWrite(GoProfilePanel tool, kXml xml, kXmlItem item)
{
    kObj(GoProfilePanel, tool);
    kXmlItem edgeItem;

    kCheck(kXml_SetChild32s(xml, item, "RefSide", obj->refEdgeSide));
    kCheck(kXml_SetChild64f(xml, item, "MaxGapWidth", obj->maxGapWidth));

    kCheck(kXml_AddItem(xml, item, "LeftEdge", &edgeItem)); 
    kCheck(GoProfileEdge_Write(obj->leftEdge, xml, edgeItem));
    kCheck(kXml_AddItem(xml, item, "RightEdge", &edgeItem)); 
    kCheck(GoProfileEdge_Write(obj->rightEdge, xml, edgeItem));

    kCheck(GoProfileTool_Write(tool, xml, item)); 

    return kOK;
}

GoFx(k64f) GoProfilePanel_MaxGapWidth(GoProfilePanel tool)
{
    kObj(GoProfilePanel, tool);
    return obj->maxGapWidth;
}

GoFx(kStatus) GoProfilePanel_SetMaxGapWidth(GoProfilePanel tool, k64f width)
{
    kObj(GoProfilePanel, tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    obj->maxGapWidth = width;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(GoProfilePanelSide) GoProfilePanel_RefEdgeSide(GoProfilePanel tool)
{
    kObjR(GoProfilePanel, tool);
    return obj->refEdgeSide;
}

GoFx(kStatus) GoProfilePanel_SetRefEdgeSide(GoProfilePanel tool, GoProfilePanelSide side)
{
    kObj(GoProfilePanel, tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    obj->refEdgeSide = side;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(GoProfileEdge) GoProfilePanel_LeftEdge(GoProfilePanel tool)
{
    kObj(GoProfilePanel, tool);
    return obj->leftEdge;
}

GoFx(GoProfileEdge) GoProfilePanel_RightEdge(GoProfilePanel tool)
{
    kObj(GoProfilePanel, tool);
    return obj->rightEdge;
}

GoFx(GoProfilePanelGap) GoProfilePanel_GapMeasurement(GoProfilePanel tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_PROFILE_PANEL_GAP);
}

GoFx(GoProfilePanelFlush) GoProfilePanel_FlushMeasurement(GoProfilePanel tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_PROFILE_PANEL_FLUSH);
}


kBeginClassEx(Go, GoProfilePosition)
kAddVMethod(GoProfilePosition, kObject, VRelease)
kAddVMethod(GoProfilePosition, GoTool, VInit)
kAddVMethod(GoProfilePosition, GoTool, VRead)
kAddVMethod(GoProfilePosition, GoTool, VWrite)
kEndClassEx()

GoFx(kStatus) GoProfilePosition_Construct(GoProfilePosition* tool, kObject sensor, kAlloc allocator)
{
    return GoTool_Construct(tool, kTypeOf(GoProfilePosition), sensor, allocator);
}

GoFx(kStatus) GoProfilePosition_VInit(GoProfilePosition tool, kType type, kObject sensor, kAlloc alloc)
{
    kObjR(GoProfilePosition, tool); 
    GoFeature featureOutput = kNULL;

    kCheck(GoProfileTool_Init(tool, type, GO_TOOL_PROFILE_POSITION, sensor, alloc)); 
    obj->feature = kNULL;

    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoProfilePositionX), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoProfilePositionZ), kTRUE, kNULL));

    kCheck(GoTool_AddFeatureOutput(tool, kTypeOf(GoProfilePositionPoint), &featureOutput));

    kCheck(GoProfileFeature_Construct(&obj->feature, sensor, alloc)); 

    return kOK; 
}

GoFx(kStatus) GoProfilePosition_VRelease(GoProfilePosition tool)
{
    kObj(GoProfilePosition, tool);

    kCheck(kDestroyRef(&obj->feature)); 

    return GoProfileTool_VRelease(tool); 
}

GoFx(kStatus) GoProfilePosition_VRead(GoProfilePosition tool, kXml xml, kXmlItem item)
{
    kObj(GoProfilePosition, tool);
    kXmlItem featureItem;

    kCheck(GoProfileTool_Read(tool, xml, item)); 
    
    kCheck(!kIsNull(featureItem = kXml_Child(xml, item, "Feature"))); 
    kCheck(GoProfileFeature_Read(obj->feature, xml, featureItem));

    return kOK;
}

GoFx(kStatus) GoProfilePosition_VWrite(GoProfilePosition tool, kXml xml, kXmlItem item)
{
    kObj(GoProfilePosition, tool);
    kXmlItem featureItem;

    kCheck(kXml_AddItem(xml, item, "Feature", &featureItem)); 
    kCheck(GoProfileFeature_Write(obj->feature, xml, featureItem));


    kCheck(GoProfileTool_Write(tool, xml, item)); 

    return kOK;
}

GoFx(GoProfileFeature) GoProfilePosition_Feature(GoProfilePosition tool)
{
    kObj(GoProfilePosition, tool);
    return obj->feature;
}

GoFx(GoProfilePositionX) GoProfilePosition_XMeasurement(GoProfilePosition tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_PROFILE_POSITION_X);
}

GoFx(kBool) GoProfilePosition_RegionEnabled(GoProfilePosition tool)
{
    kObj(GoProfilePosition, tool);
    kObjN(GoProfileFeature, featureObj, obj->feature);
    return featureObj->regionEnabled;
}

GoFx(kStatus) GoProfilePosition_EnableRegion(GoProfilePosition tool, kBool enable)
{
    kObj(GoProfilePosition, tool);
    kObjN(GoProfileFeature, featureObj, obj->feature);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    
    featureObj->regionEnabled = enable;

    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(GoProfilePositionZ) GoProfilePosition_ZMeasurement(GoProfilePosition tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_PROFILE_POSITION_Z);
}

GoFx(GoProfilePositionPoint) GoProfilePosition_Point(GoProfilePosition tool)
{
    return GoTool_FindFeatureOutputByType(tool, GO_FEATURE_PROFILE_POSITION_POINT);
}


kBeginClassEx(Go, GoProfileStrip)
kAddVMethod(GoProfileStrip, kObject, VRelease)
kAddVMethod(GoProfileStrip, GoTool, VInit)
kAddVMethod(GoProfileStrip, GoTool, VRead)
kAddVMethod(GoProfileStrip, GoTool, VWrite)
kEndClassEx()

GoFx(kStatus) GoProfileStrip_Construct(GoProfileStrip* tool, kObject sensor, kAlloc allocator)
{
    return GoTool_Construct(tool, kTypeOf(GoProfileStrip), sensor, allocator);
}

GoFx(kStatus) GoProfileStrip_VInit(GoProfileStrip tool, kType type, kObject sensor, kAlloc alloc)
{
    kObjR(GoProfileStrip, tool); 

    kCheck(GoProfileTool_Init(tool, type, GO_TOOL_PROFILE_STRIP, sensor, alloc)); 
    obj->baseType = GO_PROFILE_STRIP_BASE_TYPE_NONE;
    kZero(obj->transitionWidth);
    obj->minWidth = 0.0;
    obj->maxVoidWidth = 0.0;
    obj->regionEnabled = kFALSE;
    obj->region = kNULL;

    obj->leftEdge = GO_PROFILE_STRIP_EDGE_TYPE_RISING 
        | GO_PROFILE_STRIP_EDGE_TYPE_FALLING 
        | GO_PROFILE_STRIP_EDGE_TYPE_DATA_END 
        | GO_PROFILE_STRIP_EDGE_TYPE_VOID;

    obj->rightEdge = GO_PROFILE_STRIP_EDGE_TYPE_RISING 
        | GO_PROFILE_STRIP_EDGE_TYPE_FALLING 
        | GO_PROFILE_STRIP_EDGE_TYPE_DATA_END 
        | GO_PROFILE_STRIP_EDGE_TYPE_VOID;

    obj->tiltEnabled = kTRUE;
    obj->supportWidth = 5.0;
    obj->minHeight = 2.0;

    kCheck(GoProfileRegion_Construct(&obj->region, sensor, alloc));

    return kOK; 
}

GoFx(kStatus) GoProfileStrip_VRelease(GoProfileStrip tool)
{
    kObj(GoProfileStrip, tool);

    kDestroyRef(&obj->region);

    return GoProfileTool_VRelease(tool);
}

GoFx(kStatus) GoProfileStrip_VRead(GoProfileStrip tool, kXml xml, kXmlItem item)
{
    kObj(GoProfileStrip, tool);
    kXmlItem regionItem = kNULL, transitionItem = kNULL;;
    
    kCheck(kXml_Child32s(xml, item, "BaseType", &obj->baseType));
    kCheck(kXml_Child32s(xml, item, "LeftEdge", &obj->leftEdge));
    kCheck(kXml_Child32s(xml, item, "RightEdge", &obj->rightEdge));
    kCheck(kXml_ChildBool(xml, item, "TiltEnabled", &obj->tiltEnabled));

    kCheck(kXml_Child64f(xml, item, "SupportWidth", &obj->supportWidth));

    kCheckArgs(!kIsNull(transitionItem = kXml_Child(xml, item, "TransitionWidth")));
    kCheck(kXml_Item64f(xml, transitionItem, &obj->transitionWidth.value));

    if (kXml_AttrExists(xml, transitionItem, "min"))
    {
        kCheck(kXml_Attr64f(xml, transitionItem, "min", &obj->transitionWidth.min));
        kCheck(kXml_Attr64f(xml, transitionItem, "max", &obj->transitionWidth.max));
    }
    else
    {
        obj->transitionWidth.min = obj->transitionWidth.max = k64F_NULL;
    }

    kCheck(kXml_Child64f(xml, item, "MinWidth", &obj->minWidth));
    kCheck(kXml_Child64f(xml, item, "MinHeight", &obj->minHeight));
    kCheck(kXml_Child64f(xml, item, "MaxVoidWidth", &obj->maxVoidWidth));

    if (kXml_ChildExists(xml, item, "RegionEnabled"))
    {
        kCheck(kXml_ChildBool(xml, item, "RegionEnabled", &obj->regionEnabled));
    }

    kCheck(!kIsNull(regionItem = kXml_Child(xml, item, "Region")));
    kCheck(GoProfileRegion_Read(obj->region, xml, regionItem));

    kCheck(GoProfileTool_Read(tool, xml, item)); 

    return kOK;
}

GoFx(kStatus) GoProfileStrip_VWrite(GoProfileStrip tool, kXml xml, kXmlItem item)
{
    kObj(GoProfileStrip, tool);
    kXmlItem regionItem = kNULL;

    kCheck(kXml_SetChild32s(xml, item, "BaseType", obj->baseType));
    kCheck(kXml_SetChild32s(xml, item, "LeftEdge", obj->leftEdge));
    kCheck(kXml_SetChild32s(xml, item, "RightEdge", obj->rightEdge));
    kCheck(kXml_SetChildBool(xml, item, "TiltEnabled", obj->tiltEnabled));

    kCheck(kXml_SetChild64f(xml, item, "SupportWidth", obj->supportWidth));
    kCheck(GoConfig_WriteRangeElement64f(xml, item, "TransitionWidth", obj->transitionWidth));
    
    kCheck(kXml_SetChild64f(xml, item, "MinWidth", obj->minWidth));
    kCheck(kXml_SetChild64f(xml, item, "MinHeight", obj->minHeight));
    kCheck(kXml_SetChild64f(xml, item, "MaxVoidWidth", obj->maxVoidWidth));

    kCheck(kXml_SetChildBool(xml, item, "RegionEnabled", obj->regionEnabled));
    kCheck(kXml_AddItem(xml, item, "Region", &regionItem));
    kCheck(GoProfileRegion_Write(obj->region, xml, regionItem));

    kCheck(GoProfileTool_Write(tool, xml, item)); 

    return kOK;
}

GoFx(kStatus) GoProfileStrip_SetBaseType(GoProfileStrip tool, GoProfileStripBaseType type)
{
    kObj(GoProfileStrip, tool); 

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    obj->baseType = type;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(GoProfileStripBaseType) GoProfileStrip_BaseType(GoProfileStrip tool)
{
    kObj(GoProfileStrip, tool);

    return obj->baseType;
}

GoFx(GoProfileStripEdgeType) GoProfileStrip_LeftEdge(GoProfileStrip tool)
{
    kObj(GoProfileStrip, tool);

    return obj->leftEdge;
}

GoFx(kStatus) GoProfileStrip_SetLeftEdge(GoProfileStrip tool, GoProfileStripEdgeType leftEdge)
{
    kObj(GoProfileStrip, tool); 

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    obj->leftEdge = leftEdge;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(GoProfileStripEdgeType) GoProfileStrip_RightEdge(GoProfileStrip tool)
{
    kObj(GoProfileStrip, tool);

    return obj->rightEdge;
}

GoFx(kStatus) GoProfileStrip_SetRightEdge(GoProfileStrip tool, GoProfileStripEdgeType rightEdge)
{
    kObj(GoProfileStrip, tool); 

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    obj->rightEdge = rightEdge;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(kBool) GoProfileStrip_TiltEnabled(GoProfileStrip tool)
{
    kObj(GoProfileStrip, tool);

    return obj->tiltEnabled;
}
GoFx(kStatus) GoProfileStrip_EnableTilt(GoProfileStrip tool, kBool enable)
{
    kObj(GoProfileStrip, tool); 

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    obj->tiltEnabled = enable;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(k64f) GoProfileStrip_SupportWidth(GoProfileStrip tool)
{
    kObj(GoProfileStrip, tool);

    return obj->supportWidth;
}

GoFx(kStatus) GoProfileStrip_SetSupportWidth(GoProfileStrip tool, k64f value)
{
    kObj(GoProfileStrip, tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));

    obj->supportWidth = value;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));
    GoSensor_SyncConfig(GoTool_Sensor(tool));

    return kOK;
}

GoFx(k64f) GoProfileStrip_TransitionWidthLimitMin(GoProfileStrip tool)
{
    kObj(GoProfileStrip, tool);

    return obj->transitionWidth.min;
}

GoFx(kStatus) GoProfileStrip_SetTransitionWidthMin(GoProfileStrip tool, k64f value)
{
    kObj(GoProfileStrip, tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    obj->transitionWidth.min = value;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(k64f) GoProfileStrip_TransitionWidthLimitMax(GoProfileStrip tool)
{
    kObj(GoProfileStrip, tool);

    return obj->transitionWidth.max;
}

GoFx(kStatus) GoProfileStrip_SetTransitionWidthMax(GoProfileStrip tool, k64f value)
{
    kObj(GoProfileStrip, tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    obj->transitionWidth.max = value;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(k64f) GoProfileStrip_TransitionWidth(GoProfileStrip tool)
{
    kObj(GoProfileStrip, tool);

    return obj->transitionWidth.value;
}

GoFx(kStatus) GoProfileStrip_SetTransitionWidth(GoProfileStrip tool, k64f value)
{
    kObj(GoProfileStrip, tool); 

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));

    if (obj->transitionWidth.min != k64F_NULL)
    {
        kCheckArgs(value >= obj->transitionWidth.min && value <= obj->transitionWidth.max);
    }

    obj->transitionWidth.value = value;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(k64f) GoProfileStrip_MinWidth(GoProfileStrip tool)
{
    kObj(GoProfileStrip, tool);

    return obj->minWidth;
}

GoFx(kStatus) GoProfileStrip_SetMinWidth(GoProfileStrip tool, k64f value)
{
    kObj(GoProfileStrip, tool); 

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    obj->minWidth = value;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(k64f) GoProfileStrip_MinHeight(GoProfileStrip tool)
{
    kObj(GoProfileStrip, tool);

    return obj->minHeight;
}

GoFx(kStatus) GoProfileStrip_SetMinHeight(GoProfileStrip tool, k64f value)
{
    kObj(GoProfileStrip, tool); 

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    obj->minHeight = value;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(k64f) GoProfileStrip_MaxVoidWidth(GoProfileStrip tool)
{
    kObj(GoProfileStrip, tool);

    return obj->maxVoidWidth;
}

GoFx(kStatus) GoProfileStrip_SetMaxVoidWidth(GoProfileStrip tool, k64f value)
{
    kObj(GoProfileStrip, tool); 

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    obj->maxVoidWidth = value;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(GoProfileRegion) GoProfileStrip_Region(GoProfileStrip tool)
{
    kObj(GoProfileStrip, tool);

    return obj->region;
}

GoFx(kBool) GoProfileStrip_RegionEnabled(GoProfileStrip tool)
{
    kObj(GoProfileStrip, tool);

    return obj->regionEnabled;
}

GoFx(kStatus) GoProfileStrip_EnableRegion(GoProfileStrip tool, kBool enable)
{
    kObj(GoProfileStrip, tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    obj->regionEnabled = enable;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(kStatus) GoProfileStrip_AddMeasurement(GoProfileStrip tool, GoMeasurementType type, GoMeasurement* measurement)
{
    kObj(GoProfileStrip, tool);
    kType classType = kNULL;
    kStatus status = kOK;

    if (type != GO_MEASUREMENT_PROFILE_STRIP_POSITION_X
        && type != GO_MEASUREMENT_PROFILE_STRIP_POSITION_Z
        && type != GO_MEASUREMENT_PROFILE_STRIP_HEIGHT
        && type != GO_MEASUREMENT_PROFILE_STRIP_WIDTH)
    {
        return kERROR_PARAMETER;
    }
    else
    {
        switch(type)
        {
        case GO_MEASUREMENT_PROFILE_STRIP_POSITION_X: classType = kTypeOf(GoProfileStripX); break;
        case GO_MEASUREMENT_PROFILE_STRIP_POSITION_Z: classType = kTypeOf(GoProfileStripZ); break;
        case GO_MEASUREMENT_PROFILE_STRIP_HEIGHT: classType = kTypeOf(GoProfileStripHeight); break;
        case GO_MEASUREMENT_PROFILE_STRIP_WIDTH: classType = kTypeOf(GoProfileStripWidth); break;
        }
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
        GoTool_RemoveMeasurement(tool, GoTool_MeasurementCount(tool) - 1);
        kEndCatch(status);
    }

    return kOK;
}

GoFx(kStatus) GoProfileStrip_RemoveMeasurement(GoProfileStrip tool, kSize index)
{
    kCheckArgs(index < GoTool_MeasurementCount(tool));
    kCheck(GoTool_RemoveMeasurement(tool, index));

    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));
    kCheck(GoSensor_SyncConfig(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(kSize) GoProfileStrip_MeasurementCount(GoProfileStrip tool)
{
    return GoTool_MeasurementCount(tool);
}

GoFx(GoMeasurement) GoProfileStrip_MeasurementAt(GoProfileStrip tool, kSize index)
{
    kAssert(index < GoTool_MeasurementCount(tool));
    
    return GoTool_MeasurementAt(tool, index);
}

kBeginClassEx(Go, GoProfileRoundCorner)
kAddVMethod(GoProfileRoundCorner, kObject, VRelease)
kAddVMethod(GoProfileRoundCorner, GoTool, VInit)
kAddVMethod(GoProfileRoundCorner, GoTool, VRead)
kAddVMethod(GoProfileRoundCorner, GoTool, VWrite)
kEndClassEx()

GoFx(kStatus) GoProfileRoundCorner_Construct(GoProfileRoundCorner* tool, kObject sensor, kAlloc allocator)
{
    return GoTool_Construct(tool, kTypeOf(GoProfileRoundCorner), sensor, allocator);
}

GoFx(kStatus) GoProfileRoundCorner_VInit(GoProfileRoundCorner tool, kType type, kObject sensor, kAlloc alloc)
{
    kObjR(GoProfileRoundCorner, tool); 

    kCheck(GoProfileTool_Init(tool, type, GO_TOOL_PROFILE_ROUND_CORNER, sensor, alloc)); 
    obj->refDirection = GO_PROFILE_ROUND_CORNER_DIRECTION_LEFT;
    obj->edge = kNULL;

    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoProfileRoundCornerX), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoProfileRoundCornerZ), kTRUE, kNULL));
    kCheck(GoTool_AddMeasurement(tool, kTypeOf(GoProfileRoundCornerAngle), kTRUE, kNULL));

    kCheck(GoTool_AddFeatureOutput(tool, kTypeOf(GoProfileRoundCornerEdgePoint), kNULL));
    kCheck(GoTool_AddFeatureOutput(tool, kTypeOf(GoProfileRoundCornerCenterPoint), kNULL));

    obj->refDirection = GO_PROFILE_ROUND_CORNER_DIRECTION_LEFT;

    kCheck(GoProfileEdge_Construct(&obj->edge, sensor, alloc)); 

    return kOK; 
}

GoFx(kStatus) GoProfileRoundCorner_VRelease(GoProfileRoundCorner tool)
{
    kObj(GoProfileRoundCorner, tool);

    kCheck(kDestroyRef(&obj->edge)); 

    return GoProfileTool_VRelease(tool); 
}

GoFx(kStatus) GoProfileRoundCorner_VRead(GoProfileRoundCorner tool, kXml xml, kXmlItem item)
{
    kObj(GoProfileRoundCorner, tool);
    kXmlItem edgeItem;

    kCheck(GoProfileTool_Read(tool, xml, item)); 

    kCheck(kXml_Child32s(xml, item, "RefDirection", &obj->refDirection));

    kCheck(!kIsNull(edgeItem = kXml_Child(xml, item, "Edge"))); 
    kCheck(GoProfileEdge_Read(obj->edge, xml, edgeItem));

    return kOK;
}

GoFx(kStatus) GoProfileRoundCorner_VWrite(GoProfileRoundCorner tool, kXml xml, kXmlItem item)
{
    kObj(GoProfileRoundCorner, tool);
    kXmlItem edgeItem;

    kCheck(kXml_SetChild32s(xml, item, "RefDirection", obj->refDirection));

    kCheck(kXml_AddItem(xml, item, "Edge", &edgeItem)); 
    kCheck(GoProfileEdge_Write(obj->edge, xml, edgeItem));

    kCheck(GoProfileTool_Write(tool, xml, item)); 

    return kOK;
}

GoFx(GoProfileRoundCornerX) GoProfileRoundCorner_XMeasurement(GoProfileRoundCorner tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_PROFILE_ROUND_CORNER_X);
}

GoFx(GoProfileRoundCornerZ) GoProfileRoundCorner_ZMeasurement(GoProfileRoundCorner tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_PROFILE_ROUND_CORNER_Z);
}

GoFx(GoProfileRoundCornerAngle) GoProfileRoundCorner_AngleMeasurement(GoProfileRoundCorner tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_PROFILE_ROUND_CORNER_ANGLE);
}

GoFx(GoProfileEdge) GoProfileRoundCorner_Edge(GoProfileRoundCorner tool)
{
    kObjR(GoProfileRoundCorner, tool);
    return obj->edge;
}

GoFx(kStatus) GoProfileRoundCorner_SetEdge(GoProfileRoundCorner tool, GoProfileEdge edge)
{
    kObjR(GoProfileRoundCorner, tool);
    
    obj->edge = edge;

    return kOK;
}

GoFx(GoProfileRoundCornerDirection) GoProfileRoundCorner_RefDirection(GoProfileRoundCorner tool)
{
    kObjR(GoProfileRoundCorner, tool);
    return obj->refDirection;
}

GoFx(kStatus) GoProfileRoundCorner_SetRefDirection(GoProfileRoundCorner tool, GoProfileRoundCornerDirection direction)
{
    kObj(GoProfileRoundCorner, tool);

    kCheckState(GoSensor_IsConfigurable(GoTool_Sensor(tool)));
    kCheck(GoSensor_CacheConfig(GoTool_Sensor(tool)));
    obj->refDirection = direction;
    kCheck(GoSensor_SetConfigModified(GoTool_Sensor(tool)));

    return kOK;
}

GoFx(GoProfileRoundCornerPoint) GoProfileRoundCorner_Point(GoProfileRoundCorner tool)
{
    return GoTool_FindFeatureOutputByType(tool, GO_FEATURE_PROFILE_ROUND_CORNER_POINT);
}

GoFx(GoProfileRoundCornerEdgePoint) GoProfileRoundCorner_EdgePoint(GoProfileRoundCorner tool)
{
    return GoTool_FindFeatureOutputByType(tool, GO_FEATURE_PROFILE_ROUND_CORNER_EDGE_POINT);
}

GoFx(GoProfileRoundCornerCenterPoint) GoProfileRoundCorner_CenterPoint(GoProfileRoundCorner tool)
{
    return GoTool_FindFeatureOutputByType(tool, GO_FEATURE_PROFILE_ROUND_CORNER_CENTER_POINT);
}

GoFx(GoProfilePanelLeftFlushX) GoProfilePanel_LeftFlushXMeasurement(GoProfilePanel tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_PROFILE_PANEL_LEFT_FLUSH_X);
}

GoFx(GoProfilePanelLeftFlushZ) GoProfilePanel_LeftFlushZMeasurement(GoProfilePanel tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_PROFILE_PANEL_LEFT_FLUSH_Z);
}

GoFx(GoProfilePanelLeftGapX) GoProfilePanel_LeftGapXMeasurement(GoProfilePanel tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_PROFILE_PANEL_LEFT_GAP_X);
}

GoFx(GoProfilePanelLeftGapZ) GoProfilePanel_LeftGapZMeasurement(GoProfilePanel tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_PROFILE_PANEL_LEFT_GAP_Z);
}

GoFx(GoProfilePanelLeftSurfaceAngle) GoProfilePanel_LeftSurfaceAngleMeasurement(GoProfilePanel tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_PROFILE_PANEL_LEFT_SURFACE_ANGLE);
}

GoFx(GoProfilePanelRightFlushX) GoProfilePanel_RightFlushXMeasurement(GoProfilePanel tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_PROFILE_PANEL_RIGHT_FLUSH_X);
}

GoFx(GoProfilePanelRightFlushZ) GoProfilePanel_RightFlushZMeasurement(GoProfilePanel tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_PROFILE_PANEL_RIGHT_FLUSH_Z);
}

GoFx(GoProfilePanelRightGapX) GoProfilePanel_RightGapXMeasurement(GoProfilePanel tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_PROFILE_PANEL_RIGHT_GAP_X);
}

GoFx(GoProfilePanelRightGapZ) GoProfilePanel_RightGapZMeasurement(GoProfilePanel tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_PROFILE_PANEL_RIGHT_GAP_Z);
}

GoFx(GoProfilePanelRightSurfaceAngle) GoProfilePanel_RightSurfaceAngleMeasurement(GoProfilePanel tool)
{
    return GoTool_FindMeasurementByType(tool, GO_MEASUREMENT_PROFILE_PANEL_RIGHT_SURFACE_ANGLE);
}

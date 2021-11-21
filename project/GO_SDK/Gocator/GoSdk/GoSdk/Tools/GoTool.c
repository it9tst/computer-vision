/**
 * @file    GoTool.c
 *
 * @internal
 * Copyright (C) 2016-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <GoSdk/Tools/GoTool.h>
#include <GoSdk/GoUtils.h>
#include <GoSdk/GoSensor.h>

kBeginClassEx(Go, GoTool)
    kAddVMethod(GoTool, kObject, VRelease)
    kAddVMethod(GoTool, GoTool, VInit)
    kAddVMethod(GoTool, GoTool, VRead)
    kAddVMethod(GoTool, GoTool, VWrite)
kEndClassEx()

GoFx(kStatus) GoTool_Construct(GoTool* tool, kType type, kObject sensor, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback(allocator);
    kStatus status;

    kCheck(kAlloc_GetObject(alloc, type, tool));

    if (!kSuccess(status = kType_VTableT(type, GoTool)->VInit(*tool, type, sensor, alloc)))
    {
        kAlloc_FreeRef(alloc, tool);
    }

    return status;
}

GoFx(kStatus) GoTool_VInit(GoTool tool, kType type, kObject sensor, kAlloc alloc)
{
    return kERROR_UNIMPLEMENTED; //this function must be overriden by every tool
}

GoFx(kStatus) GoTool_Init(GoTool tool, kType type, GoToolType typeId, kObject sensor, kAlloc alloc)
{
    kObjR(GoTool, tool);
    kStatus exception;

    kCheck(kObject_Init(tool, type, alloc));
    obj->index = 0;
    kZero(obj->xml);
    kZero(obj->xmlItem);
    obj->name[0] = 0;
    obj->format = GO_TOOL_FORMAT_STANDARD;
    kZero(obj->featureOutputs);
    kZero(obj->featureOptions);
    kZero(obj->featureNodesToMerge);
    kZero(obj->measurements);
    kZero(obj->measurementOptions);
    kZero(obj->measurementNodesToMerge);

    obj->id = GO_UNASSIGNED_ID;

    obj->sensor = sensor;
    obj->typeId = typeId;

    kTry
    {
        kTest(kArrayList_Construct(&obj->featureNodesToMerge, kTypeOf(kPointer), 0, alloc));
        kTest(kArrayList_Construct(&obj->measurementNodesToMerge, kTypeOf(kPointer), 0, alloc));
        kTest(kArrayList_Construct(&obj->measurements, kTypeOf(GoMeasurement), 0, alloc));
        kTest(kArrayList_Construct(&obj->featureOutputs, kTypeOf(GoFeature), 0, alloc));

        //kTest(kArrayList_Construct(&obj->measurementOptions, kTypeOf(kPointer), 0, alloc));
        //kTest(kArrayList_Construct(&obj->featureOptions, kTypeOf(kPointer), 0, alloc));
    }
    kCatch(&exception)
    {
        GoTool_VRelease(tool);
        kEndCatch(exception);
    }

    return kOK;
}

GoFx(kStatus) GoTool_VRelease(GoTool tool)
{
    kObj(GoTool, tool);

    kCheck(kDestroyRef(&obj->featureNodesToMerge));
    kCheck(kDestroyRef(&obj->measurementNodesToMerge));
    kCheck(kDisposeRef(&obj->measurements));
    kCheck(kDisposeRef(&obj->featureOutputs));

    //kCheck(kDisposeRef(&obj->measurementOptions));
    //kCheck(kDisposeRef(&obj->featureOptions));

    return kObject_VRelease(tool);
}

// Reads the tool XML node specified by "item".
GoFx(kStatus) GoTool_VRead(GoTool tool, kXml xml, kXmlItem item)
{
    kObj(GoTool, tool);
    kXmlItem toolNameItem = kNULL;
    kXmlItem measurements = kNULL;
    kXmlItem features = kNULL;
    kText64 toolType;

    obj->xml = xml;
    obj->xmlItem = item;

    if (kXml_AttrExists(xml, item, GO_XML_TAG_NAME_ATTR_TOOL_ID))
    {
        kCheck(kXml_Attr32s(xml, item, GO_XML_TAG_NAME_ATTR_TOOL_ID, &obj->id));
    }
    else
    {
        obj->id = GO_UNASSIGNED_ID;
    }

    // GOC-14127: The format is used to validate/correct the tool name and
    // Feature/Measurement/ToolDataOutput name so this must be set up
    // before reading Features/Measurements/ToolDataOutputs.
    if (kXml_AttrExists(xml, item, GO_XML_TAG_NAME_ATTR_FORMAT))
    {
        kCheck(kXml_Attr32s(xml, item, GO_XML_TAG_NAME_ATTR_FORMAT, &obj->format));
    }
    else
    {
        obj->format = GO_TOOL_FORMAT_UNKNOWN;
    }

    kCheck(kArrayList_Clear(obj->measurementNodesToMerge));
    kCheck(kArrayList_Clear(obj->featureNodesToMerge));

    kCheck(GoTool_ReadToolName(tool, xml, item));

    // GOC-14127: Get the tool name, making use of the tool format version to determine
    // if tool is an internal GDK tool or not, which requires special handling
    // to map tool type to a tool name.
    // NOTE: the GoTools_FormatToolType() call would also work by providing
    // GoTool_ReadMeasurements() and GoTool_ReadFeatures() with a tool name
    // they can use in their lookup tables, but the
    // new API is used instead because the returned result is more correct.
    kCheck(GoTools_FormatToolTypeExtToolFormat(kObject_Type(tool), GoTool_IsGdkInternal(tool), toolType, kCountOf(toolType)));

    kCheck(!kIsNull(measurements = kXml_Child(xml, item, "Measurements")));
    kCheck(GoTool_ReadMeasurements(tool, xml, measurements, toolType));

    if (!kIsNull(features = kXml_Child(xml, item, "Features")))
    {
        kCheck(GoTool_ReadFeatures(tool, xml, features, toolType));
    }

    return kOK;
}

GoFx(kStatus) GoTool_VWrite(GoTool tool, kXml xml, kXmlItem item)
{
    kObj(GoTool, tool);
    kXmlItem temp = kNULL;
    kXmlItem child = kNULL;

    kXmlItem toolItem = kNULL;
    kXmlItem measurementsItem = kNULL;
    kXmlItem measurementItem = kNULL;
    GoMeasurement measurement = kNULL;
    kText64 measurementName;

    kXmlItem featuresItem = kNULL;
    kXmlItem featureItem = kNULL;
    GoFeature feature = kNULL;
    kText64 featureName;

    kSize i;
    kXml xmlCopy = kNULL;

    if (obj->id != GO_UNASSIGNED_ID)
    {
        kCheck(kXml_SetAttr32s(xml, item, "id", obj->id));
    }
        
    if (obj->format != GO_TOOL_FORMAT_UNKNOWN)
    {
        kCheck(kXml_SetAttr32s(xml, item, "format", obj->format));
    }

    kCheck(kXml_SetChildText(xml, item, "Name", obj->name));

    kCheck(kXml_AddItem(xml, item, "Measurements", &measurementsItem));

    for (i = 0; i < kArrayList_Count(obj->measurements); i++)
    {
        measurement = kArrayList_AsT(obj->measurements, i, GoMeasurement);

        // GOC-14127: on write, choose the correct measurement name
        // for GDK tools based on whether the tool is a USER GDK or
        // INTERNAL GDK tool.
        kCheck(GoMeasurements_FormatTypeExtToolFormat(kObject_Type(measurement), GoTool_IsGdkInternal(tool), measurementName, kCountOf(measurementName)));
        kCheck(kXml_AddItem(xml, measurementsItem, measurementName, &measurementItem));
        kCheck(GoMeasurement_Write(measurement, xml, measurementItem));
    }

    for (i = 0; i < kArrayList_Count(obj->measurementNodesToMerge); i++)
    {
        kXmlItem itemToMerge = *kArrayList_AtT(obj->measurementNodesToMerge, i, kXmlItem);
        kCheck(kXml_CopyItem(xml, measurementsItem, kNULL, obj->xml, itemToMerge, kNULL));
    }


    kCheck(kXml_AddItem(xml, item, "Features", &featuresItem));

    for (i = 0; i < kArrayList_Count(obj->featureOutputs); i++)
    {
        feature = kArrayList_AsT(obj->featureOutputs, i, GoFeature);

        // GOC-14127: on write, choose the correct feature name
        // for GDK tools based on whether the tool is a USER GDK or
        // INTERNAL GDK tool.
        kCheck(GoFeatures_FormatTypeExtToolFormat(kObject_Type(feature), GoTool_IsGdkInternal(tool), featureName, kCountOf(featureName)));
        kCheck(kXml_AddItem(xml, featuresItem, featureName, &featureItem));
        kCheck(GoFeature_Write(feature, xml, featureItem));
    }

    for (i = 0; i < kArrayList_Count(obj->featureNodesToMerge); i++)
    {
        kXmlItem itemToMerge = *kArrayList_AtT(obj->featureNodesToMerge, i, kXmlItem);
        kCheck(kXml_CopyItem(xml, featuresItem, kNULL, obj->xml, itemToMerge, kNULL));
    }

    kCheck(GoUtils_XmlMerge(obj->xml, obj->xmlItem, xml, item));

    return kOK;
}

GoFx(kStatus) GoTool_Read(GoTool tool, kXml xml, kXmlItem item)
{
    kObj(GoTool, tool);
    return kCast(GoToolVTable*, xkObject_VTable(tool))->VRead(tool, xml, item);
}

GoFx(kStatus) GoTool_Write(GoTool tool, kXml xml, kXmlItem item)
{
    kObj(GoTool, tool);
    return kCast(GoToolVTable*, xkObject_VTable(tool))->VWrite(tool, xml, item);
}

GoFx(kStatus) GoTool_ReadMeasurements(GoTool tool, kXml xml, kXmlItem measurements, const kText64 toolType)
{
    kObj(GoTool, tool);
    kXmlItem measurementItem = kNULL;
    GoMeasurement measurement;
    kType measurementType;
    kBool insertMeasurement;
    kBool isFilterable;
    kSize i;

    for (i = 0; i < kXml_ChildCount(xml, measurements); i++)
    {
        measurementItem = kXml_ChildAt(xml, measurements, i);
        measurement = kNULL;
        insertMeasurement = kFALSE;
        isFilterable = kTRUE;

        if (kSuccess(GoMeasurements_ParseType(toolType, kXml_ItemName(xml, measurementItem), &measurementType)))
        {
            if (i >= kArrayList_Count(obj->measurements)
                || kIsNull(measurement = kArrayList_AsT(obj->measurements, i, GoMeasurement))
                || !kObject_Is(measurement, measurementType))
            {
                if (measurementType == kTypeOf(GoScriptOutput))
                {
                    isFilterable = kFALSE;
                }

                if (!kIsNull(measurement))
                {
                    kCheck(kArrayList_Discard(obj->measurements, i));
                    kDestroyRef(&measurement);
                }

                kCheck(GoMeasurement_Construct(&measurement, measurementType, obj->sensor, tool, isFilterable, kObject_Alloc(tool)));
                insertMeasurement = kTRUE;
            }

            kCheck(GoMeasurement_Read(measurement, xml, measurementItem));

            if (insertMeasurement)
            {
                if (i >= kArrayList_Count(obj->measurements))
                    kCheck(kArrayList_AddT(obj->measurements, &measurement));
                else
                    kCheck(kArrayList_InsertT(obj->measurements, i, &measurement));
            }
        }
        else
        {
            kCheck(kArrayList_AddT(obj->measurementNodesToMerge, &measurementItem));
        }
    }

    while (kXml_ChildCount(xml, measurements) < kArrayList_Count(obj->measurements))    //arraylist is accessed directly due to refresh behavior when the remove measurement function is used
    {
        kCheck(kArrayList_RemoveT(obj->measurements, kArrayList_Count(obj->measurements) - 1, &measurement));
        kDestroyRef(&measurement);
    }

    return kOK;
}

GoFx(kStatus) GoTool_ReadFeatures(GoTool tool, kXml xml, kXmlItem features, const kText64 toolType)
{
    kObj(GoTool, tool);
    kXmlItem featureItem = kNULL;
    GoFeature feature;
    kType featureType;
    kBool insertFeature;
    kSize i;
    kSize numFeatures;

    numFeatures = kXml_ChildCount(xml, features);

    for (i = 0; i < numFeatures; i++)
    {
        featureItem = kXml_ChildAt(xml, features, i);
        feature = kNULL;
        insertFeature = kFALSE;

        if (kSuccess(GoFeatures_ParseType(toolType, kXml_ItemName(xml, featureItem), &featureType)))
        {
            if (i >= kArrayList_Count(obj->featureOutputs)
                || kIsNull(feature = kArrayList_AsT(obj->featureOutputs, i, GoFeature))
                || !kObject_Is(feature, featureType))
            {
                if (!kIsNull(feature))
                {
                    kCheck(kArrayList_Discard(obj->featureOutputs, i));
                    kDestroyRef(&feature);
                }

                kCheck(GoFeature_Construct(&feature, featureType, obj->sensor, tool, kObject_Alloc(tool)));
                insertFeature = kTRUE;
            }

            kCheck(GoFeature_Read(feature, xml, featureItem));

            if (insertFeature)
            {
                kCheck(kArrayList_InsertT(obj->featureOutputs, i, &feature));
            }
        }
        else
        {
            if (i >= kArrayList_Count(obj->featureNodesToMerge))
                kCheck(kArrayList_AddT(obj->featureNodesToMerge, &featureItem));
            else
                kCheck(kArrayList_InsertT(obj->featureNodesToMerge, i, &featureItem));
        }
    }

    // Arraylist is accessed directly due to refresh behavior when the remove measurement function is used
    while (kXml_ChildCount(xml, features) < kArrayList_Count(obj->featureOutputs))    
    {
        kCheck(kArrayList_RemoveT(obj->featureOutputs, kArrayList_Count(obj->featureOutputs) - 1, &feature));
        kDestroyRef(&feature);
    }

    return kOK;
}

GoFx(kSize) GoTool_MeasurementCount(GoTool tool)
{
    kObj(GoTool, tool);

    GoSensor_SyncConfig(obj->sensor);

    return kArrayList_Count(obj->measurements);
}

GoFx(GoMeasurement) GoTool_MeasurementAt(GoTool tool, kSize index)
{
    kObj(GoTool, tool);

    GoSensor_SyncConfig(obj->sensor);

    kAssert(index < kArrayList_Count(obj->measurements));

    return kArrayList_AsT(obj->measurements, index, GoMeasurement);
}

GoFx(kStatus) GoTool_AddMeasurement(GoTool tool, kType type, kBool isFilterable, GoMeasurement* measurement)
{
    kObj(GoTool, tool);
    GoMeasurement output = kNULL;
    kStatus exception;

    //private method, no need to check configurable or set modified
    kTry
    {

        kTest(GoMeasurement_Construct(&output, type, obj->sensor, tool, isFilterable, kObject_Alloc(tool)));

        kTest(kArrayList_AddT(obj->measurements, &output));

        if(!kIsNull(measurement))
        {
            *measurement = output;
        }
    }
    kCatch(&exception)
    {
        kDestroyRef(&output);
        kEndCatch(exception);
    }

    return kOK;
}

GoFx(kStatus) GoTool_AddExtMeasurement(GoTool tool, kType type, kBool isFilterable, GoExtMeasurement* measurement)
{
    kObj(GoTool, tool);
    GoMeasurement output = kNULL;
    kStatus exception;

    //private method, no need to check configurable or set modified
    kTry
    {

        kTest(GoExtMeasurement_Construct(&output, type, obj->sensor, tool, isFilterable, kObject_Alloc(tool)));

        kTest(GoSensor_IsConfigurable(obj->sensor));
        kTest(GoSensor_CacheConfig(obj->sensor));
        kTest(kArrayList_AddT(obj->measurements, &output));
        kTest(GoSensor_SetConfigModified(obj->sensor));

        if (!kIsNull(measurement))
        {
            *measurement = output;
        }

    }
    kCatch(&exception)
    {
        kDestroyRef(&output);
        kEndCatch(exception);
    }

    return kOK;
}

GoFx(kStatus) GoTool_RemoveMeasurement(GoTool tool, kSize index)
{
    kObj(GoTool, tool);
    GoMeasurement measurement = kNULL;

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    //private method, no need to check configurable or set modified
    kCheck(kArrayList_RemoveT(obj->measurements, index, &measurement));
    kCheck(kDestroyRef(&measurement));

    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(kStatus) GoTool_ClearMeasurements(GoTool tool)
{
    kObj(GoTool, tool);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));
    //private method, no need to check configurable or set modified
    kCheck(kArrayList_Purge(obj->measurements));
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(kSize) GoTool_FeatureOutputCount(GoTool tool)
{
    kObj(GoTool, tool);

    GoSensor_SyncConfig(obj->sensor);

    return kArrayList_Count(obj->featureOutputs);
}

GoFx(GoFeature) GoTool_FeatureOutputAt(GoTool tool, kSize index)
{
    kObj(GoTool, tool);

    GoSensor_SyncConfig(obj->sensor);

    kAssert(index < kArrayList_Count(obj->featureOutputs));

    return kArrayList_AsT(obj->featureOutputs, index, GoFeature);
}

GoFx(kStatus) GoTool_AddFeatureOutput(GoTool tool, kType type, GoFeature* featureOutput)
{
    kObj(GoTool, tool);
    GoFeature output = kNULL;
    kStatus exception;

    //private method, no need to check configurable or set modified
    kTry 
    {

        kTest(GoFeature_Construct(&output, type, obj->sensor, tool, kObject_Alloc(tool)));

        kTest(kArrayList_AddT(obj->featureOutputs, &output));

        if(!kIsNull(featureOutput))
        {
            *featureOutput = output;
        }
    }
    kCatch(&exception)
    {
        kDestroyRef(&output);
        kEndCatch(exception);
    }

    return kOK;
}

GoFx(kStatus) GoTool_RemoveFeatureOutput(GoTool tool, kSize index)
{
    kObj(GoTool, tool);
    GoFeature featureOutput = kNULL;

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    //private method, no need to check configurable or set modified
    kCheck(kArrayList_RemoveT(obj->featureOutputs, index, &featureOutput));
    kCheck(kDestroyRef(&featureOutput));

    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(kStatus) GoTool_ClearFeatureOutputs(GoTool tool)
{
    kObj(GoTool, tool);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));
    //private method, no need to check configurable or set modified
    kCheck(kArrayList_Purge(obj->featureOutputs));
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(kStatus) GoTool_SetName(GoTool tool, const kChar* name)
{
    kObj(GoTool, tool);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));
    kStrCopy(obj->name, kCountOf(obj->name), name);
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(kStatus) GoTool_Name(GoTool tool, kChar* name, kSize capacity)
{
    kObj(GoTool, tool);

    GoSensor_SyncConfig(obj->sensor);

    kStrCopy(name, capacity, obj->name);

    return kOK;
}

GoFx(kObject) GoTool_Sensor(GoTool tool)
{
    kObj(GoTool, tool);

    return obj->sensor;
}

GoFx(GoMeasurement) GoTool_FindMeasurementByType(GoTool tool, GoMeasurementType type)
{
    kSize i;
    GoMeasurement measurement = kNULL;
    kBool found = kFALSE;

    for (i = 0; i < GoTool_MeasurementCount(tool); i++)
    {
        measurement = GoTool_MeasurementAt(tool, i);

        if (GoMeasurement_Type(measurement) == type)
        {
            found = kTRUE;
            break;
        }
    }

    if (!found)
    {
        return kNULL;
    }

    return measurement;
}

GoFx(GoFeature) GoTool_FindFeatureOutputByType(GoTool tool, GoFeatureType type)
{
    kObj(GoTool, tool);

    kSize i;
    GoFeature feature = kNULL;
    kBool found = kFALSE;

    GoSensor_SyncConfig(obj->sensor);

    for (i = 0; i < GoTool_FeatureOutputCount(tool); i++)
    {
        feature = GoTool_FeatureOutputAt(tool, i);

        if (GoFeature_TypeId(feature) == type)
        {
            found = kTRUE;
            break;
        }
    }

    if (!found)
    {
        return kNULL;
    }

    return feature;
}

GoFx(GoToolType) GoTool_Type(GoTool tool)
{
    kObj(GoTool, tool);

    GoSensor_SyncConfig(obj->sensor);

    return obj->typeId;
}

GoFx(k32s) GoTool_Id(GoTool tool)
{
    kObj(GoTool, tool);

    GoSensor_SyncConfig(obj->sensor);

    return obj->id;
}

GoFx(kStatus) GoTool_SetIndex(GoTool tool, kSize index)
{
    kObj(GoTool, tool);

    obj->index = index;

    return kOK;
}

// Helper function to set the tool name member variable.
GoFx(kStatus) GoTool_ReadToolName(GoTool tool, kXml xml, kXmlItem item)
{
    kObj(GoTool, tool);
    kXmlItem toolNameItem = kNULL;

    kCheck(!kIsNull(toolNameItem = kXml_Child(xml, item, "Name")));
    kCheck(kXml_ItemText(xml, toolNameItem, obj->name, kCountOf(obj->name)));

    return kOK;
}

// GOC-14127: This function checks the GDK tool format to determine if it is
// an INTERNAL GDK tool or not. 
// It is useful when formatting the XML configuration file for
// a GDK tool to know if the tool is a USER GDK tool or an
// INTERNAL GDK tool.
GoFx(kBool) GoTool_IsGdkInternal(GoTool tool)
{
    kObj(GoTool, tool);

    return (obj->format == GO_TOOL_FORMAT_GDK_INTERNAL);
}

GoFx(kStatus) GoToolUtil_ParseStreamOptions(GoTool tool, kXml xml, kXmlItem item, kArrayList list)
{
    kObj(GoTool, tool);

    kXml tempItem = kNULL;
    kString csvList = kNULL;
    kArrayList tempArrayList = kNULL;

    kTry
    {
        kTest(kString_Construct(&csvList, "", kObject_Alloc(tool)));

        if (kXml_ChildExists(xml, item, "StreamOptions"))
        {
            kXmlItem streamItem = kXml_Child(xml, item, "StreamOptions");
            kSize i,j;

            kTest(kArrayList_Clear(list));

            for (i = 0; i < kXml_ChildCount(xml, streamItem); i++)
            {
                GoDataStream option;

                tempItem = kXml_ChildAt(xml, streamItem, i);
                if (!kIsNull(tempItem))
                {
                    kTest(kXml_Attr32s(xml, tempItem, "step", &option.step));
                    kTest(kXml_AttrString(xml, tempItem, "ids", csvList));
                    kTest(kString_Split(csvList, ",", &tempArrayList, kObject_Alloc(tool)));

                    for (j = 0; j < kArrayList_Count(tempArrayList); j++)
                    {
                        kString token = *kArrayList_AtT(tempArrayList, j, kString);
                        kTest(k32s_Parse(&option.id, kString_Chars(token)));
                        kTest(kArrayList_AddT(list, &option));
                    }
                    // GOC-14516 Dispose memory allocation per iteration to avoid memory leak.
                    kDisposeRef(&tempArrayList);
                }
            }
        }
    }
    kFinally
    {
        kDestroyRef(&csvList);
        // GOC-14516 Dispose memory allocation here in case of last iteration fails. 
        kDisposeRef(&tempArrayList);
        kEndFinally();
    }

    return kOK;
}


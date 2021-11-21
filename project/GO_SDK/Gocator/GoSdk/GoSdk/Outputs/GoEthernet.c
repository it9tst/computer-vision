/**
 * @file    GoEthernet.c
 *
 * @internal
 * Copyright (C) 2016-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <GoSdk/Outputs/GoEthernet.h>
#include <GoSdk/GoUtils.h>
#include <GoSdk/GoSensor.h>
#include <stdio.h>

kBeginClassEx(Go, GoEthernet)
    kAddVMethod(GoEthernet, kObject, VRelease)
kEndClassEx()

// Table of the Ethernet Output sources and their configuration XML tag names.
GoValueNamePair g_EthernetOutputSourceTypeMap[] =
{
    { GO_OUTPUT_SOURCE_VIDEO,               "Videos" },
    { GO_OUTPUT_SOURCE_RANGE,               "Ranges" },
    { GO_OUTPUT_SOURCE_PROFILE,             "Profiles" },
    { GO_OUTPUT_SOURCE_SURFACE,             "Surfaces" },
    { GO_OUTPUT_SOURCE_SECTION,             "SurfaceSections" },
    { GO_OUTPUT_SOURCE_RANGE_INTENSITY,     "RangeIntensities" },
    { GO_OUTPUT_SOURCE_PROFILE_INTENSITY,   "ProfileIntensities" },
    { GO_OUTPUT_SOURCE_SURFACE_INTENSITY,   "SurfaceIntensities" },
    { GO_OUTPUT_SOURCE_SECTION_INTENSITY,   "SurfaceSectionsIntensities" },
    { GO_OUTPUT_SOURCE_MEASUREMENT,         "Measurements" },
    { GO_OUTPUT_SOURCE_TRACHEID,            "Tracheids" },
    { GO_OUTPUT_SOURCE_EVENT,               "Events" },
    { GO_OUTPUT_SOURCE_FEATURE,             "Features" },
    { GO_OUTPUT_SOURCE_TOOLDATA,            "ToolDataOutputs" }
};

GoFx(kStatus) GoEthernet_Construct(GoEthernet* ethernet, kObject sensor, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback(allocator);
    kStatus status;

    kCheck(kAlloc_GetObject(alloc, kTypeOf(GoEthernet), ethernet));

    if (!kSuccess(status = GoEthernet_Init(*ethernet, kTypeOf(GoEthernet), sensor, alloc)))
    {
        kAlloc_FreeRef(alloc, ethernet);
    }

    return status;
}

GoFx(kStatus) GoEthernet_Init(GoEthernet ethernet, kType type, kObject sensor, kAlloc alloc)
{
    kObjR(GoEthernet, ethernet);
    kStatus             exception;

    kCheck(kObject_Init(ethernet, type, alloc));
    kZero(obj->xml);
    kZero(obj->xmlItem);
    kZero(obj->videoOptions);
    kZero(obj->videoSources);
    kZero(obj->rangeOptions);
    kZero(obj->rangeSources);
    kZero(obj->rangeIntensityOptions);
    kZero(obj->rangeIntensitySources);
    kZero(obj->profileOptions);
    kZero(obj->profileSources);
    kZero(obj->profileIntensityOptions);
    kZero(obj->profileIntensitySources);
    kZero(obj->surfaceIntensityOptions);
    kZero(obj->surfaceIntensitySources);
    kZero(obj->surfaceOptions);
    kZero(obj->surfaceSources);
    kZero(obj->sectionIntensityOptions);
    kZero(obj->sectionIntensitySources);
    kZero(obj->sectionOptions);
    kZero(obj->sectionSources);
    kZero(obj->measurementOptions);
    kZero(obj->measurementSources);
    kZero(obj->eventOptions);
    kZero(obj->events);
    kZero(obj->tracheidOptions);
    kZero(obj->tracheidSources);
    kZero(obj->featureOptions);
    kZero(obj->featureSources);
    kZero(obj->toolDataOptions);
    kZero(obj->toolDataSources);
    obj->outputUsed = kFALSE;
    obj->protocol = GO_ETHERNET_PROTOCOL_GOCATOR;
    kZero(obj->eip);
    kZero(obj->modbus);
    kZero(obj->ascii);
    kZero(obj->profinet);
    kZero(obj->ptp);

    obj->sensor = sensor;

    // Auto-disconnect is available and enabled by default.
    obj->timeoutEnabled = kTRUE;
    obj->timeoutEnabledAvail = kTRUE;

    obj->timeout = 10.0;

    kTry
    {
        kTest(kArrayList_Construct(&obj->videoOptions, kTypeOf(k32s), 0, alloc));
        kTest(kArrayList_Construct(&obj->videoSources, kTypeOf(k32s), 0, alloc));

        kTest(kArrayList_Construct(&obj->rangeOptions, kTypeOf(k32s), 0, alloc));
        kTest(kArrayList_Construct(&obj->rangeSources, kTypeOf(k32s), 0, alloc));

        kTest(kArrayList_Construct(&obj->rangeIntensityOptions, kTypeOf(k32s), 0, alloc));
        kTest(kArrayList_Construct(&obj->rangeIntensitySources, kTypeOf(k32s), 0, alloc));

        kTest(kArrayList_Construct(&obj->profileOptions, kTypeOf(k32s), 0, alloc));
        kTest(kArrayList_Construct(&obj->profileSources, kTypeOf(k32s), 0, alloc));

        kTest(kArrayList_Construct(&obj->profileIntensityOptions, kTypeOf(k32s), 0, alloc));
        kTest(kArrayList_Construct(&obj->profileIntensitySources, kTypeOf(k32s), 0, alloc));

        kTest(kArrayList_Construct(&obj->surfaceOptions, kTypeOf(k32s), 0, alloc));
        kTest(kArrayList_Construct(&obj->surfaceSources, kTypeOf(k32s), 0, alloc));

        kTest(kArrayList_Construct(&obj->surfaceIntensityOptions, kTypeOf(k32s), 0, alloc));
        kTest(kArrayList_Construct(&obj->surfaceIntensitySources, kTypeOf(k32s), 0, alloc));

        kTest(kArrayList_Construct(&obj->sectionOptions, kTypeOf(GoOutputCompositeSource), 0, alloc));
        kTest(kArrayList_Construct(&obj->sectionSources, kTypeOf(GoOutputCompositeSource), 0, alloc));

        kTest(kArrayList_Construct(&obj->sectionIntensityOptions, kTypeOf(GoOutputCompositeSource), 0, alloc));
        kTest(kArrayList_Construct(&obj->sectionIntensitySources, kTypeOf(GoOutputCompositeSource), 0, alloc));

        kTest(kArrayList_Construct(&obj->measurementOptions, kTypeOf(k32s), 0, alloc));
        kTest(kArrayList_Construct(&obj->measurementSources, kTypeOf(k32s), 0, alloc));

        kTest(kArrayList_Construct(&obj->eventOptions, kTypeOf(k32s), 0, alloc));
        kTest(kArrayList_Construct(&obj->events, kTypeOf(k32s), 0, alloc));

        kTest(kArrayList_Construct(&obj->tracheidOptions, kTypeOf(k32s), 0, alloc));
        kTest(kArrayList_Construct(&obj->tracheidSources, kTypeOf(k32s), 0, alloc));

        kTest(kArrayList_Construct(&obj->featureOptions, kTypeOf(k32s), 0, alloc));
        kTest(kArrayList_Construct(&obj->featureSources, kTypeOf(k32s), 0, alloc));

        kTest(kArrayList_Construct(&obj->toolDataOptions, kTypeOf(k32s), 0, alloc));
        kTest(kArrayList_Construct(&obj->toolDataSources, kTypeOf(k32s), 0, alloc));

        kTest(kString_Construct(&obj->ascii.customFormat, "", alloc));
        kTest(kString_Construct(&obj->ascii.delimiter, "", alloc));
        kTest(kString_Construct(&obj->ascii.invalidValue, "", alloc));
        kTest(kString_Construct(&obj->ascii.terminator, "", alloc));

        kTest(kString_Construct(&obj->profinet.deviceName, "", alloc));
    }
    kCatch(&exception)
    {
        GoEthernet_VRelease(ethernet);
        kEndCatch(exception);
    }

    return kOK;
}

GoFx(kStatus) GoEthernet_VRelease(GoEthernet ethernet)
{
    kObj(GoEthernet, ethernet);

    kCheck(kDisposeRef(&obj->videoOptions));
    kCheck(kDisposeRef(&obj->videoSources));

    kCheck(kDisposeRef(&obj->rangeOptions));
    kCheck(kDisposeRef(&obj->rangeSources));

    kCheck(kDisposeRef(&obj->rangeIntensityOptions));
    kCheck(kDisposeRef(&obj->rangeIntensitySources));

    kCheck(kDisposeRef(&obj->profileOptions));
    kCheck(kDisposeRef(&obj->profileSources));

    kCheck(kDisposeRef(&obj->profileIntensityOptions));
    kCheck(kDisposeRef(&obj->profileIntensitySources));

    kCheck(kDisposeRef(&obj->surfaceOptions));
    kCheck(kDisposeRef(&obj->surfaceSources));

    kCheck(kDisposeRef(&obj->surfaceIntensityOptions));
    kCheck(kDisposeRef(&obj->surfaceIntensitySources));

    kCheck(kDisposeRef(&obj->sectionOptions));
    kCheck(kDisposeRef(&obj->sectionSources));

    kCheck(kDisposeRef(&obj->sectionIntensityOptions));
    kCheck(kDisposeRef(&obj->sectionIntensitySources));

    kCheck(kDisposeRef(&obj->measurementOptions));
    kCheck(kDisposeRef(&obj->measurementSources));

    kCheck(kDisposeRef(&obj->eventOptions));
    kCheck(kDisposeRef(&obj->events));

    kCheck(kDisposeRef(&obj->tracheidOptions));
    kCheck(kDisposeRef(&obj->tracheidSources));

    kCheck(kDisposeRef(&obj->featureOptions));
    kCheck(kDisposeRef(&obj->featureSources));

    kCheck(kDisposeRef(&obj->toolDataOptions));
    kCheck(kDisposeRef(&obj->toolDataSources));

    kCheck(kDestroyRef(&obj->ascii.customFormat));
    kCheck(kDestroyRef(&obj->ascii.delimiter));
    kCheck(kDestroyRef(&obj->ascii.invalidValue));
    kCheck(kDestroyRef(&obj->ascii.terminator));

    kCheck(kDestroyRef(&obj->profinet.deviceName));

    return kObject_VRelease(ethernet);
}

GoFx(kStatus) GoEthernet_ParseOutputList(GoOutputSource outputSource, kChar* text, kArrayList outputList)
{
    if (outputSource == GO_OUTPUT_SOURCE_EVENT)
    {
        // Events are of type GoEventType which is a signed integer.
        kCheck(GoOptionList_ParseList32s(text, outputList));
    }
    else if ((outputSource == GO_OUTPUT_SOURCE_SECTION) ||
             (outputSource == GO_OUTPUT_SOURCE_SECTION_INTENSITY))
    {
        // Not typical ints -> actually of the form "ID-DataSource".
        kCheck(GoEthernet_ParseCompositeOptionList(text, outputList));
    }
    else
    {
        kCheck(GoOptionList_ParseList32u(text, outputList));
    }

    return kOK;
}

GoFx(kStatus) GoEthernet_ParseOutputOptionList(GoEthernet ethernet, GoOutputSource outputSource, kChar* text)
{
    kArrayList outputList;

    outputList = GoEthernet_OptionList(ethernet, outputSource);
    kCheckState(!kIsNull(outputList));

    kCheck(GoEthernet_ParseOutputList(outputSource, text, outputList));

    return kOK;
}

GoFx(kStatus) GoEthernet_ParseOutputSourceList(GoEthernet ethernet, GoOutputSource outputSource, kChar* text)
{
    kArrayList outputList;

    outputList = GoEthernet_SourceList(ethernet, outputSource);
    kCheckState(!kIsNull(outputList));

    kCheck(GoEthernet_ParseOutputList(outputSource, text, outputList));

    return kOK;
}

GoFx(kStatus) GoEthernet_ReadAllOutputConfig(GoEthernet ethernet)
{
    kObj(GoEthernet, ethernet);
    kChar*              text = kNULL;
    kSize               textCapacity = GO_OUTPUT_SOURCE_TEXT_CAPACITY;
    kXmlItem            tempItem = kNULL;
    kSize               i;
    kSize               numMapEntry = kCountOf(g_EthernetOutputSourceTypeMap);
    GoValueNamePair*    tableEntry;

    kTry
    {
        kTest(kObject_GetMemZero(ethernet, sizeof(kChar) * textCapacity, &text));

        for (i = 0; i < numMapEntry; i++)
        {
            tableEntry = &g_EthernetOutputSourceTypeMap[i];

            tempItem = kXml_Child(obj->xml, obj->xmlItem, tableEntry->name);
            if (!kIsNull(tempItem))
            {
                kTest(kXml_AttrText(obj->xml, tempItem, "options", text, textCapacity));
                kTest(GoEthernet_ParseOutputOptionList(ethernet, (GoOutputSource) tableEntry->value, text));

                kTest(kXml_ItemText(obj->xml, tempItem, text, textCapacity));
                kTest(GoEthernet_ParseOutputSourceList(ethernet, (GoOutputSource) tableEntry->value, text));
            }
        }
    }
    kFinally
    {
        kObject_FreeMem(ethernet, text);
        kEndFinally();
    }

    return kOK;
}

GoFx(kStatus) GoEthernet_Read(GoEthernet ethernet, kXml xml, kXmlItem item)
{
    kObj(GoEthernet, ethernet);
    kXmlItem tempItem = kNULL;

    obj->xml = xml;
    obj->xmlItem = item;

    kTry
    {
        tempItem = kXml_Child(xml, item, "TimeoutEnabled");
        if (!kIsNull(tempItem))
        {
            kTest(kXml_ItemBool(xml, tempItem, &obj->timeoutEnabled));
            // Set default value to true to match legacy behaviour, if
            // the configuration file does not have this attribute.
            kTest(GoConfig_ReadAttrBoolOptional(xml, tempItem, "used", kTRUE, &obj->timeoutEnabledAvail));

            kTest(kXml_Child64f(xml, item, "Timeout", &obj->timeout));
        }

        kTest(GoConfig_ReadAttrBoolOptional(xml, item, "used", kTRUE, &obj->outputUsed));;
        kTest(kXml_Child32s(xml, item, "Protocol", (k32s*)&obj->protocol));

        kTest(GoEthernet_ReadAscii(ethernet, xml, item));
        kTest(GoEthernet_ReadEip(ethernet, xml, item));
        kTest(GoEthernet_ReadModbus(ethernet, xml, item));
        kTest(GoEthernet_ReadProfinet(ethernet, xml, item));
        kTest(GoEthernet_ReadPtp(ethernet, xml, item));
        kTest(GoEthernet_ReadAllOutputConfig(ethernet));
    }
    kFinally
    {
        kEndFinally();
    }

    return kOK;
}

GoFx(kStatus) GoEthernet_WriteAllOutputConfig(GoEthernet ethernet, kXml xml, kXmlItem item)
{
    kChar*              text = kNULL;
    kSize               textCapacity = GO_OUTPUT_SOURCE_TEXT_CAPACITY;
    GoValueNamePair*    tableEntry;
    kSize               i;
    kSize               numMapEntry = kCountOf(g_EthernetOutputSourceTypeMap);
    kArrayList          sourceList;

    kTry
    {
        kTest(kObject_GetMemZero(ethernet, sizeof(kChar) * textCapacity, &text));

        for (i = 0; i < numMapEntry; i++)
        {
            tableEntry = &g_EthernetOutputSourceTypeMap[i];
            sourceList = GoEthernet_SourceList(ethernet, (GoOutputSource) tableEntry->value);
            kTestState(!kIsNull(sourceList));

            if (((GoOutputSource) tableEntry->value == GO_OUTPUT_SOURCE_SECTION) ||
                ((GoOutputSource) tableEntry->value == GO_OUTPUT_SOURCE_SECTION_INTENSITY))
            {
                kTest(GoOptionList_FormatOutputCompositeSource(kArrayList_DataT(sourceList, const GoOutputCompositeSource), kArrayList_Count(sourceList), text, textCapacity));
            }
            else
            {
                kTest(GoOptionList_Format32u(kArrayList_DataT(sourceList, const k32u), kArrayList_Count(sourceList), text, textCapacity));
            }
            kTest(kXml_SetChildText(xml, item, tableEntry->name, text));
        }
    }
    kFinally
    {
        kObject_FreeMem(ethernet, text);
        kEndFinally();
    }

    return kOK;
}

GoFx(kStatus) GoEthernet_Write(GoEthernet ethernet, kXml xml, kXmlItem item)
{
    kObj(GoEthernet, ethernet);
    kXmlItem            tempItem = kNULL;

    kTry
    {
        kTest(kXml_SetChild32s(xml, item, "Protocol", obj->protocol));

        // Only need to send the TimeoutEnabled value to the sensor.
        // The "used" attribute is only sent from sensor to client, never
        // from client to sensor.
        kTest(kXml_SetChildBool(xml, item, "TimeoutEnabled", obj->timeoutEnabled));

        kTest(kXml_SetChild64f(xml, item, "Timeout", obj->timeout));

        kTest(GoEthernet_WriteAscii(ethernet, xml, item));
        kTest(GoEthernet_WriteEip(ethernet, xml, item));
        kTest(GoEthernet_WriteModbus(ethernet, xml, item));

        // Client never changes the Profinet parameters since it is
        // read-only for the client. Therefore don't send the
        // Profinet parameters back to the sensor.

        kTest(GoEthernet_WriteAllOutputConfig(ethernet, xml, item));
    }
    kFinally
    {
        kEndFinally();
    }

    // Forwards Compatibility.
    kCheck(GoUtils_XmlMerge(obj->xml, obj->xmlItem, xml, item));

    return kOK;
}

GoFx(kStatus) GoEthernet_ReadAscii(GoEthernet ethernet, kXml xml, kXmlItem item)
{
    kObj(GoEthernet, ethernet);
    kXmlItem tempItem = kNULL;

    //ASCII config
    tempItem = kXml_Child(xml, item, "Ascii");
    if (!kIsNull(tempItem))
    {
        kCheck(kXml_Child32u(xml, tempItem, "Operation", (k32u*) &obj->ascii.operation));
        kCheck(kXml_Child32u(xml, tempItem, "ControlPort", &obj->ascii.controlPort));
        kCheck(kXml_Child32u(xml, tempItem, "DataPort", &obj->ascii.dataPort));
        kCheck(kXml_Child32u(xml, tempItem, "HealthPort", &obj->ascii.healthPort));
        kCheck(kXml_ChildString(xml, tempItem, "Delimiter", obj->ascii.delimiter));
        kCheck(kXml_ChildString(xml, tempItem, "Terminator", obj->ascii.terminator));
        kCheck(kXml_ChildString(xml, tempItem, "InvalidValue", obj->ascii.invalidValue));
        kCheck(kXml_ChildString(xml, tempItem, "CustomDataFormat", obj->ascii.customFormat));
        kCheck(kXml_ChildBool(xml, tempItem, "CustomFormatEnabled", &obj->ascii.customFormatEnabled));

        if (kXml_ChildExists(xml, tempItem, "StandardFormatMode"))
        {
            kCheck(kXml_Child32s(xml, tempItem, "StandardFormatMode", &obj->ascii.standardFormatMode));
        }
    }
    return kOK;
}

GoFx(kStatus) GoEthernet_WriteAscii(GoEthernet ethernet, kXml xml, kXmlItem item)
{
    kObj(GoEthernet, ethernet);
    kXmlItem tempItem = kNULL;

    //ASCII config
    kCheck(kXml_AddItem(xml, item, "Ascii", &tempItem));
    kCheck(kXml_SetChild32s(xml, tempItem, "Operation", obj->ascii.operation));
    kCheck(kXml_SetChild32u(xml, tempItem, "ControlPort", obj->ascii.controlPort));
    kCheck(kXml_SetChild32u(xml, tempItem, "DataPort", obj->ascii.dataPort));
    kCheck(kXml_SetChild32u(xml, tempItem, "HealthPort", obj->ascii.healthPort));
    kCheck(kXml_SetChildText(xml, tempItem, "Delimiter", kString_Chars(obj->ascii.delimiter)));
    kCheck(kXml_SetChildText(xml, tempItem, "Terminator", kString_Chars(obj->ascii.terminator)));
    kCheck(kXml_SetChildText(xml, tempItem, "InvalidValue", kString_Chars(obj->ascii.invalidValue)));
    kCheck(kXml_SetChildText(xml, tempItem, "CustomDataFormat", kString_Chars(obj->ascii.customFormat)));
    kCheck(kXml_SetChildBool(xml, tempItem, "CustomFormatEnabled", obj->ascii.customFormatEnabled));
    kCheck(kXml_SetChild32s(xml, tempItem, "StandardFormatMode", obj->ascii.standardFormatMode));

    return kOK;
}

GoFx(kStatus) GoEthernet_ReadEip(GoEthernet ethernet, kXml xml, kXmlItem item)
{
    kObj(GoEthernet, ethernet);
    kXmlItem tempItem = kNULL;

    tempItem = kXml_Child(xml, item, "EIP");
    if (!kIsNull(tempItem))
    {
        kCheck(kXml_ChildBool(xml, tempItem, "BufferEnabled", &obj->eip.bufferEnabled));
        kCheck(GoConfig_Read32sOptional(xml, tempItem, "EndianOutputType", kFALSE, &obj->eip.endianOutputType));
        kCheck(GoConfig_ReadBoolOptional(xml, tempItem, "ImplicitOutputEnabled", kFALSE, &obj->eip.implicitOutputEnabled));
        kCheck(GoConfig_Read32sOptional(xml, tempItem, "ImplicitTriggerOverride", kFALSE, &obj->eip.implicitTriggerOverride));
    }
    return kOK;
}

GoFx(kStatus) GoEthernet_WriteEip(GoEthernet ethernet, kXml xml, kXmlItem item)
{
    kObj(GoEthernet, ethernet);
    kXmlItem tempItem = kNULL;

    kCheck(kXml_AddItem(xml, item, "EIP", &tempItem));
    kCheck(kXml_SetChildBool(xml, tempItem, "BufferEnabled", obj->eip.bufferEnabled));
    kCheck(kXml_SetChild32s(xml, tempItem, "EndianOutputType", obj->eip.endianOutputType));
    kCheck(kXml_SetChildBool(xml, tempItem, "ImplicitOutputEnabled", obj->eip.implicitOutputEnabled));
    kCheck(kXml_SetChild32s(xml, tempItem, "ImplicitTriggerOverride", obj->eip.implicitTriggerOverride));

    return kOK;
}

GoFx(kStatus) GoEthernet_ReadModbus(GoEthernet ethernet, kXml xml, kXmlItem item)
{
    kObj(GoEthernet, ethernet);
    kXmlItem tempItem = kNULL;

    tempItem = kXml_Child(xml, item, "Modbus");
    if (!kIsNull(tempItem))
    {
        kCheck(kXml_ChildBool(xml, tempItem, "BufferEnabled", &obj->modbus.bufferEnabled));
    }

    return kOK;
}

GoFx(kStatus) GoEthernet_WriteModbus(GoEthernet ethernet, kXml xml, kXmlItem item)
{
    kObj(GoEthernet, ethernet);
    kXmlItem tempItem = kNULL;

    kCheck(kXml_AddItem(xml, item, "Modbus", &tempItem));
    kCheck(kXml_SetChildBool(xml, tempItem, "BufferEnabled", obj->modbus.bufferEnabled));

    return kOK;
}

GoFx(kStatus) GoEthernet_ReadProfinet(GoEthernet ethernet, kXml xml, kXmlItem item)
{
    kObj(GoEthernet, ethernet);
    kXmlItem tempItem = kNULL;
    kString  tempString = kNULL;
    kStatus  exception;

    tempItem = kXml_Child(xml, item, "Profinet");
    if (!kIsNull(tempItem))
    {
        kTry
        {
            kTest(kString_Construct(&tempString, "", kObject_Alloc(ethernet)));

            kTest(kXml_ChildString(xml, tempItem, "IpAddress", tempString));
            kTest(kIpAddress_Parse(&obj->profinet.ipAddress, kString_Chars(tempString)));

            kTest(kXml_ChildString(xml, tempItem, "SubnetMask", tempString));
            kTest(kIpAddress_Parse(&obj->profinet.subnetMask, kString_Chars(tempString)));

            kTest(kXml_ChildString(xml, tempItem, "Gateway", tempString));
            kTest(kIpAddress_Parse(&obj->profinet.gateway, kString_Chars(tempString)));

            kTest(kXml_ChildString(xml, tempItem, "DeviceName", obj->profinet.deviceName));
        }
        kCatchEx(&exception)
        {
            kLogf("%s: error %d reading Profinet configuration", __FUNCTION__, exception);

            kEndCatchEx(exception);
        }
        kFinallyEx
        {
            kCheck(kObject_Destroy(tempString));

            kEndFinallyEx();
        }
    }

    return kOK;
}

GoFx(kStatus) GoEthernet_ReadPtp(GoEthernet ethernet, kXml xml, kXmlItem item)
{
    kObj(GoEthernet, ethernet);
    kXmlItem tempItem = kNULL;
    kString  tempString = kNULL;
    kStatus  exception;

    tempItem = kXml_Child(xml, item, "Ptp");
    if (!kIsNull(tempItem))
    {
        kTry
        {
            kTest(kString_Construct(&tempString, "", kObject_Alloc(ethernet)));
        }
        kCatchEx(&exception)
        {
            kLogf("%s: error %d reading Ptp configuration", __FUNCTION__, exception);

            kEndCatchEx(exception);
        }
        kFinallyEx
        {
            kCheck(kObject_Destroy(tempString));

            kEndFinallyEx();
        }
    }

    return kOK;
}

GoFx(kStatus) GoEthernet_ParseHelperComposite(const kChar* text, kSize length, GoOutputCompositeSource* value)
{
    const kChar* readIt = text;
    const kChar* separator = kNULL;

    if (length == 0)
    {
        return kERROR_PARAMETER;
    }

    if (!kIsNull(separator = strchr(readIt, GO_UTILS_STREAM_ID_SEPARATOR)))
    {
        if (kSuccess(GoOptionList_ParseHelper32s(readIt, separator - readIt, &value->id)))
        {
            readIt = separator + 1;
            if (kSuccess(GoOptionList_ParseHelper32s(readIt, separator - readIt, &value->dataSource)))
            {
                return kOK;
            }
        }
    }

    return kERROR_CONFLICT;
}

GoFx(kStatus) GoEthernet_ParseCompositeOptionList(const kChar* text, kArrayList list)
{
    const kChar* readIt = text;
    const kChar* separator = kNULL;
    GoOutputCompositeSource option;

    kCheck(kArrayList_Clear(list));
    kCheck(kArrayList_Allocate(list, kTypeOf(GoOutputCompositeSource), 0));

    while (!kIsNull(separator = strchr(readIt, ',')))
    {
        if (kSuccess(GoEthernet_ParseHelperComposite(readIt, separator - readIt, &option)))
        {
            kCheck(kArrayList_AddT(list, &option));
        }
        readIt = separator + 1;
    }

    if (kSuccess(GoEthernet_ParseHelperComposite(readIt, strlen(readIt), &option)))
    {
        kCheck(kArrayList_AddT(list, &option));
    }

    return kOK;
}

GoFx(kBool) GoEthernet_CheckCompositeSource(const GoOutputCompositeSource* optionList, kSize count, GoOutputCompositeSource value)
{
    kSize i;
    for (i = 0; i < count; ++i)
    {
        if (optionList[i].id == value.id && optionList[i].dataSource == value.dataSource)
        {
            return kTRUE;
        }
    }

    return kFALSE;
}

GoFx(kArrayList) GoEthernet_OptionList(GoEthernet ethernet, GoOutputSource type)
{
    kObj(GoEthernet, ethernet);
    kArrayList list = kNULL;

    switch (type)
    {
        case GO_OUTPUT_SOURCE_VIDEO:                           list = obj->videoOptions;            break;
        case GO_OUTPUT_SOURCE_RANGE:                           list = obj->rangeOptions;            break;
        case GO_OUTPUT_SOURCE_RANGE_INTENSITY:                 list = obj->rangeIntensityOptions;   break;
        case GO_OUTPUT_SOURCE_PROFILE:                         list = obj->profileOptions;          break;
        case GO_OUTPUT_SOURCE_PROFILE_INTENSITY:               list = obj->profileIntensityOptions; break;
        case GO_OUTPUT_SOURCE_SURFACE:                         list = obj->surfaceOptions;          break;
        case GO_OUTPUT_SOURCE_SURFACE_INTENSITY:               list = obj->surfaceIntensityOptions; break;
        case GO_OUTPUT_SOURCE_SECTION:                         list = obj->sectionOptions;          break;
        case GO_OUTPUT_SOURCE_SECTION_INTENSITY:               list = obj->sectionIntensityOptions; break;
        case GO_OUTPUT_SOURCE_MEASUREMENT:                     list = obj->measurementOptions;      break;
        case GO_OUTPUT_SOURCE_TRACHEID:                        list = obj->tracheidOptions;         break;
        case GO_OUTPUT_SOURCE_EVENT:                           list = obj->eventOptions;            break;
        case GO_OUTPUT_SOURCE_FEATURE:                         list = obj->featureOptions;          break;
        case GO_OUTPUT_SOURCE_TOOLDATA:                        list = obj->toolDataOptions;         break;
    }

    return list;
}

GoFx(kArrayList) GoEthernet_SourceList(GoEthernet ethernet, GoOutputSource type)
{
    kObj(GoEthernet, ethernet);
    kArrayList list = kNULL;

    switch (type)
    {
        case GO_OUTPUT_SOURCE_VIDEO:                         list = obj->videoSources;              break;
        case GO_OUTPUT_SOURCE_RANGE:                         list = obj->rangeSources;              break;
        case GO_OUTPUT_SOURCE_RANGE_INTENSITY:               list = obj->rangeIntensitySources;     break;
        case GO_OUTPUT_SOURCE_PROFILE:                       list = obj->profileSources;            break;
        case GO_OUTPUT_SOURCE_PROFILE_INTENSITY:             list = obj->profileIntensitySources;   break;
        case GO_OUTPUT_SOURCE_SURFACE:                       list = obj->surfaceSources;            break;
        case GO_OUTPUT_SOURCE_SURFACE_INTENSITY:             list = obj->surfaceIntensitySources;   break;
        case GO_OUTPUT_SOURCE_SECTION:                       list = obj->sectionSources;            break;
        case GO_OUTPUT_SOURCE_SECTION_INTENSITY:             list = obj->sectionIntensitySources;   break;
        case GO_OUTPUT_SOURCE_MEASUREMENT:                   list = obj->measurementSources;        break;
        case GO_OUTPUT_SOURCE_TRACHEID:                      list = obj->tracheidSources;           break;
        case GO_OUTPUT_SOURCE_EVENT:                         list = obj->events;                    break;
        case GO_OUTPUT_SOURCE_FEATURE:                       list = obj->featureSources;            break;
        case GO_OUTPUT_SOURCE_TOOLDATA:                      list = obj->toolDataSources;           break;
    }

    return list;
}

GoFx(kSize) GoEthernet_OptionCount(GoEthernet ethernet, GoOutputSource type)
{
    kObj(GoEthernet, ethernet);
    kArrayList list;

    GoSensor_SyncConfig(obj->sensor);

    list = GoEthernet_OptionList(ethernet, type);

    if (kIsNull(list))
    {
        return 0;
    }

    return kArrayList_Count(list);
}

GoFx(k32u) GoEthernet_OptionAt(GoEthernet ethernet, GoOutputSource type, kSize index)
{
    kObj(GoEthernet, ethernet);
    kArrayList list;

    GoSensor_SyncConfig(obj->sensor);

    list = GoEthernet_OptionList(ethernet, type);

    if (kIsNull(list))
    {
        return 0;
    }

    kAssert(index < kArrayList_Count(list));

    return kArrayList_AsT(list, index, k32u);
}

GoFx(kSize) GoEthernet_SourceCount(GoEthernet ethernet, GoOutputSource type)
{
    kObj(GoEthernet, ethernet);
    kArrayList list;

    GoSensor_SyncConfig(obj->sensor);

    list = GoEthernet_SourceList(ethernet, type);

    if (kIsNull(list))
    {
        return 0;
    }

    return kArrayList_Count(list);
}

GoFx(k32u) GoEthernet_SourceAt(GoEthernet ethernet, GoOutputSource type, kSize index)
{
    kObj(GoEthernet, ethernet);
    kArrayList list;

    GoSensor_SyncConfig(obj->sensor);

    list = GoEthernet_SourceList(ethernet, type);

    if (kIsNull(list))
    {
        return 0;
    }

    kAssert(index < kArrayList_Count(list));

    return kArrayList_AsT(list, index, k32u);
}

GoFx(kStatus) GoEthernet_AddSource(GoEthernet ethernet, GoOutputSource type, k32s sourceId)
{
    kObj(GoEthernet, ethernet);
    kArrayList list = kNULL;
    kArrayList optionList = kNULL;

    kCheck(GoSensor_SyncConfig(obj->sensor));


    // Section and Section Intensity:
    //  - GoEthernet_RemoveCompositeSource must be used for sections.
     if ((type != GO_OUTPUT_SOURCE_SECTION) &&
            (type != GO_OUTPUT_SOURCE_SECTION_INTENSITY))
    {
        list = GoEthernet_SourceList(ethernet, type);
        optionList = GoEthernet_OptionList(ethernet, type);
    }

    if (kIsNull(list) || kIsNull(optionList))
    {
        return kERROR_PARAMETER;
    }

    kCheckArgs(GoOptionList_Check32s(kArrayList_DataT(optionList, const k32s), kArrayList_Count(optionList), sourceId));
    kCheck(kArrayList_AddT(list, &sourceId));
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(kStatus) GoEthernet_RemoveSource(GoEthernet ethernet, GoOutputSource type, kSize index)
{
    kObj(GoEthernet, ethernet);
    kArrayList list = kNULL;

    kCheck(GoSensor_SyncConfig(obj->sensor));

    // Section and Section Intensity:
    //  - GoEthernet_RemoveCompositeSource must be used for sections.
    // Event:
    //  - Use the event specific APIs to add and remove events.
    if ((type != GO_OUTPUT_SOURCE_EVENT)   &&
        (type != GO_OUTPUT_SOURCE_SECTION) &&
        (type != GO_OUTPUT_SOURCE_SECTION_INTENSITY))
    {
        list = GoEthernet_SourceList(ethernet, type);
    }

    if (kIsNull(list))
    {
        return kERROR_PARAMETER;
    }

    kCheck(kArrayList_Discard(list, index));
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(kSize) GoEthernet_CompositeOptionCount(GoEthernet ethernet, GoOutputSource type)
{
    kObj(GoEthernet, ethernet);
    kArrayList list = kNULL;

    GoSensor_SyncConfig(obj->sensor);

    if (type == GO_OUTPUT_SOURCE_SECTION ||
        type == GO_OUTPUT_SOURCE_SECTION_INTENSITY)
    {
        list = GoEthernet_OptionList(ethernet, type);
    }

    if (kIsNull(list))
    {
        return 0;
    }

    return kArrayList_Count(list);
}

GoFx(GoOutputCompositeSource) GoEthernet_CompositeOptionAt(GoEthernet ethernet, GoOutputSource type, kSize index)
{
    kObj(GoEthernet, ethernet);
    kArrayList list = kNULL;
    GoOutputCompositeSource failedSearch;

    failedSearch.id = -1;
    failedSearch.dataSource = GO_DATA_SOURCE_NONE;

    GoSensor_SyncConfig(obj->sensor);

    if (type == GO_OUTPUT_SOURCE_SECTION ||
        type == GO_OUTPUT_SOURCE_SECTION_INTENSITY)
    {
        list = GoEthernet_OptionList(ethernet, type);
    }

    if (kIsNull(list))
    {
        return failedSearch;
    }

    kAssert(index < kArrayList_Count(list));

    return kArrayList_AsT(list, index, GoOutputCompositeSource);
}

GoFx(kSize) GoEthernet_CompositeSourceCount(GoEthernet ethernet, GoOutputSource type)
{
    kObj(GoEthernet, ethernet);
    kArrayList list = kNULL;

    GoSensor_SyncConfig(obj->sensor);

    if (type == GO_OUTPUT_SOURCE_SECTION ||
        type == GO_OUTPUT_SOURCE_SECTION_INTENSITY)
    {
        list = GoEthernet_OptionList(ethernet, type);
    }

    if (kIsNull(list))
    {
        return 0;
    }

    return kArrayList_Count(list);
}

GoFx(GoOutputCompositeSource) GoEthernet_CompositeSourceAt(GoEthernet ethernet, GoOutputSource type, kSize index)
{
    kObj(GoEthernet, ethernet);
    kArrayList list = kNULL;
    GoOutputCompositeSource failedSearch;

    failedSearch.id = -1;
    failedSearch.dataSource = GO_DATA_SOURCE_NONE;

    GoSensor_SyncConfig(obj->sensor);

    if (type == GO_OUTPUT_SOURCE_SECTION ||
        type == GO_OUTPUT_SOURCE_SECTION_INTENSITY)
    {
        list = GoEthernet_OptionList(ethernet, type);
    }

    if (kIsNull(list))
    {
        return failedSearch;
    }

    kAssert(index < kArrayList_Count(list));

    return kArrayList_AsT(list, index, GoOutputCompositeSource);
}

GoFx(kStatus) GoEthernet_AddCompositeSource(GoEthernet ethernet, GoOutputSource type, GoOutputCompositeSource option)
{
    kObj(GoEthernet, ethernet);
    kArrayList list = kNULL;
    kArrayList optionList = kNULL;

    kCheck(GoSensor_SyncConfig(obj->sensor));

    if (type == GO_OUTPUT_SOURCE_SECTION ||
        type == GO_OUTPUT_SOURCE_SECTION_INTENSITY)
    {
        list = GoEthernet_SourceList(ethernet, type);
        optionList = GoEthernet_OptionList(ethernet, type);
    }

    if (kIsNull(list) || kIsNull(optionList))
    {
        return kERROR_PARAMETER;
    }

    kCheck(GoEthernet_CheckCompositeSource(kArrayList_DataT(optionList, const GoOutputCompositeSource), kArrayList_Count(optionList), option));
    kCheck(kArrayList_AddT(list, &option));
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(kStatus) GoEthernet_RemoveCompositeSource(GoEthernet ethernet, GoOutputSource type, kSize index)
{
    kObj(GoEthernet, ethernet);
    kArrayList list = kNULL;

    kCheck(GoSensor_SyncConfig(obj->sensor));

    if (type == GO_OUTPUT_SOURCE_SECTION ||
        type == GO_OUTPUT_SOURCE_SECTION_INTENSITY)
    {
        list = GoEthernet_SourceList(ethernet, type);
    }

    if (kIsNull(list))
    {
        return kERROR_PARAMETER;
    }

    kCheck(kArrayList_Discard(list, index));
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}


GoFx(kStatus) GoEthernet_ClearAllSources(GoEthernet ethernet)
{
    kObj(GoEthernet, ethernet);
    kSize               i;
    kSize               numMapEntry = kCountOf(g_EthernetOutputSourceTypeMap);

    kCheck(GoSensor_SyncConfig(obj->sensor));

    for (i = 0; i < numMapEntry; i++)
    {
        kCheck(GoEthernet_ClearSources(ethernet, (GoOutputSource) g_EthernetOutputSourceTypeMap[i].value));
    }

    return kOK;
}

GoFx(kStatus) GoEthernet_ClearSources(GoEthernet ethernet, GoOutputSource type)
{
    kObj(GoEthernet, ethernet);
    kArrayList list;

    list = GoEthernet_SourceList(ethernet, type);
    if (kIsNull(list))
    {
        return kERROR_PARAMETER;
    }

    kCheck(kArrayList_Clear(list));
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}


GoFx(k64f) GoEthernet_Timeout(GoEthernet ethernet)
{
    kObj(GoEthernet, ethernet);

    GoSensor_SyncConfig(obj->sensor);

    return obj->timeout;
}

GoFx(kStatus) GoEthernet_SetTimeout(GoEthernet ethernet, k64f value)
{
    kObj(GoEthernet, ethernet);

    kCheckArgs(value >= 0.0);

    obj->timeout = value;

    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(kBool) GoEthernet_TimeoutEnabled(GoEthernet ethernet)
{
    kObj(GoEthernet, ethernet);

    GoSensor_SyncConfig(obj->sensor);

    return obj->timeoutEnabled;
}

GoFx(kStatus) GoEthernet_EnableTimeout(GoEthernet ethernet, kBool value)
{
    kObj(GoEthernet, ethernet);

    obj->timeoutEnabled = value;

    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(kBool) GoEthernet_TimeoutEnabledIsAvailable(GoEthernet ethernet)
{
    kObj(GoEthernet, ethernet);

    GoSensor_SyncConfig(obj->sensor);

    return obj->timeoutEnabledAvail;
}

GoFx(GoEthernetProtocol) GoEthernet_Protocol(GoEthernet ethernet)
{
    kObj(GoEthernet, ethernet);

    GoSensor_SyncConfig(obj->sensor);

    return obj->protocol;
}

GoFx(kStatus) GoEthernet_SetProtocol(GoEthernet ethernet, GoEthernetProtocol protocol)
{
    kObj(GoEthernet, ethernet);

    obj->protocol = protocol;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(GoAsciiOperation) GoEthernet_AsciiOperation(GoEthernet ethernet)
{
    kObj(GoEthernet, ethernet);

    GoSensor_SyncConfig(obj->sensor);

    return obj->ascii.operation;
}

GoFx(kStatus) GoEthernet_SetAsciiOperation(GoEthernet ethernet, GoAsciiOperation mode)
{
    kObj(GoEthernet, ethernet);

    obj->ascii.operation = mode;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(k32u) GoEthernet_AsciiControlPort(GoEthernet ethernet)
{
    kObj(GoEthernet, ethernet);

    GoSensor_SyncConfig(obj->sensor);

    return obj->ascii.controlPort;
}

GoFx(kStatus) GoEthernet_SetAsciiControlPort(GoEthernet ethernet, k32u port)
{
    kObj(GoEthernet, ethernet);

    obj->ascii.controlPort = port;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(k32u) GoEthernet_AsciiHealthPort(GoEthernet ethernet)
{
    kObj(GoEthernet, ethernet);

    GoSensor_SyncConfig(obj->sensor);

    return obj->ascii.healthPort;
}

GoFx(kStatus) GoEthernet_SetAsciiHealthPort(GoEthernet ethernet, k32u port)
{
    kObj(GoEthernet, ethernet);

    obj->ascii.healthPort = port;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(k32u) GoEthernet_AsciiDataPort(GoEthernet ethernet)
{
    kObj(GoEthernet, ethernet);

    GoSensor_SyncConfig(obj->sensor);

    return obj->ascii.dataPort;
}

GoFx(kStatus) GoEthernet_SetAsciiDataPort(GoEthernet ethernet, k32u port)
{
    kObj(GoEthernet, ethernet);

    obj->ascii.dataPort = port;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(kChar*) GoEthernet_AsciiDelimiter(GoEthernet ethernet)
{
    kObj(GoEthernet, ethernet);

    GoSensor_SyncConfig(obj->sensor);

    return kString_Chars(obj->ascii.delimiter);
}

GoFx(kStatus) GoEthernet_SetAsciiDelimiter(GoEthernet ethernet, const kChar* string)
{
    kObj(GoEthernet, ethernet);

    kCheck(kString_Set(obj->ascii.delimiter, string));
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(kChar*) GoEthernet_AsciiTerminator(GoEthernet ethernet)
{
    kObj(GoEthernet, ethernet);

    GoSensor_SyncConfig(obj->sensor);

    return kString_Chars(obj->ascii.terminator);
}

GoFx(kStatus) GoEthernet_SetAsciiTerminator(GoEthernet ethernet, const kChar* string)
{
    kObj(GoEthernet, ethernet);

    kCheck(kString_Set(obj->ascii.terminator, string));
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(kChar*) GoEthernet_AsciiInvalidValue(GoEthernet ethernet)
{
    kObj(GoEthernet, ethernet);

    GoSensor_SyncConfig(obj->sensor);

    return kString_Chars(obj->ascii.invalidValue);
}

GoFx(kStatus) GoEthernet_SetAsciiInvalidValue(GoEthernet ethernet, const kChar* string)
{
    kObj(GoEthernet, ethernet);

    kCheck(kString_Set(obj->ascii.invalidValue, string));
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(kChar*) GoEthernet_AsciiCustomDataFormat(GoEthernet ethernet)
{
    kObj(GoEthernet, ethernet);

    GoSensor_SyncConfig(obj->sensor);

    return kString_Chars(obj->ascii.customFormat);
}

GoFx(kStatus) GoEthernet_SetAsciiCustomDataFormat(GoEthernet ethernet, const kChar* string)
{
    kObj(GoEthernet, ethernet);

    kCheck(kString_Set(obj->ascii.customFormat, string));
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(kStatus) GoEthernet_EnableAsciiCustomFormat(GoEthernet ethernet, kBool enabled)
{
    kObj(GoEthernet, ethernet);

    obj->ascii.customFormatEnabled = enabled;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(kBool) GoEthernet_AsciiCustomFormatEnabled(GoEthernet ethernet)
{
    kObj(GoEthernet, ethernet);

    GoSensor_SyncConfig(obj->sensor);

    return obj->ascii.customFormatEnabled;
}

GoFx(kStatus) GoEthernet_SetAsciiStandardFormat(GoEthernet ethernet, GoAsciiStandardFormatMode mode)
{
    kObj(GoEthernet, ethernet);

    obj->ascii.standardFormatMode = mode;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(GoAsciiStandardFormatMode) GoEthernet_AsciiStandardFormat(GoEthernet ethernet)
{
    kObj(GoEthernet, ethernet);

    GoSensor_SyncConfig(obj->sensor);

    return obj->ascii.standardFormatMode;
}

GoFx(kStatus) GoEthernet_SetEIPBufferingEnabled(GoEthernet ethernet, kBool enabled)
{
    kObj(GoEthernet, ethernet);

    obj->eip.bufferEnabled = enabled;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(kBool) GoEthernet_EIPBufferingEnabled(GoEthernet ethernet)
{
    kObj(GoEthernet, ethernet);

    GoSensor_SyncConfig(obj->sensor);

    return obj->eip.bufferEnabled;
}

GoFx(kStatus) GoEthernet_SetEIPImplicitOutputEnabled(GoEthernet ethernet, kBool enabled)
{
    kObj(GoEthernet, ethernet);

    obj->eip.implicitOutputEnabled = enabled;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(kBool) GoEthernet_EIPImplicitOutputEnabled(GoEthernet ethernet)
{
    kObj(GoEthernet, ethernet);

    GoSensor_SyncConfig(obj->sensor);

    return obj->eip.implicitOutputEnabled;
}

GoFx(kStatus) GoEthernet_SetEIPEndianOutputType(GoEthernet ethernet, GoEndianType type)
{
    kObj(GoEthernet, ethernet);

    obj->eip.endianOutputType = type;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(GoEndianType) GoEthernet_EIPEndianOutputType(GoEthernet ethernet)
{
    kObj(GoEthernet, ethernet);

    GoSensor_SyncConfig(obj->sensor);

    return obj->eip.endianOutputType;
}

GoFx(kStatus) GoEthernet_SetEIPImplicitTriggerOverride(GoEthernet ethernet, GoImplicitTriggerOverride value)
{
    kObj(GoEthernet, ethernet);

    obj->eip.implicitTriggerOverride = value;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(GoImplicitTriggerOverride) GoEthernet_EIPImplicitTriggerOverride(GoEthernet ethernet)
{
    kObj(GoEthernet, ethernet);

    GoSensor_SyncConfig(obj->sensor);

    return obj->eip.implicitTriggerOverride;
}


GoFx(kStatus) GoEthernet_SetModbusBufferingEnabled(GoEthernet ethernet, kBool enabled)
{
    kObj(GoEthernet, ethernet);

    obj->modbus.bufferEnabled = enabled;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(kBool) GoEthernet_ModbusBufferingEnabled(GoEthernet ethernet)
{
    kObj(GoEthernet, ethernet);

    GoSensor_SyncConfig(obj->sensor);

    return obj->modbus.bufferEnabled;
}

GoFx(const kIpAddress*) GoEthernet_ProfinetIpAddress(GoEthernet ethernet)
{
    kObj(GoEthernet, ethernet);

    GoSensor_SyncConfig(obj->sensor);

    return (const kIpAddress*) &obj->profinet.ipAddress;
}

GoFx(const kIpAddress*) GoEthernet_ProfinetSubnetMask(GoEthernet ethernet)
{
    kObj(GoEthernet, ethernet);

    GoSensor_SyncConfig(obj->sensor);

    return (const kIpAddress*) &obj->profinet.subnetMask;
}

GoFx(const kIpAddress*) GoEthernet_ProfinetGateway(GoEthernet ethernet)
{
    kObj(GoEthernet, ethernet);

    GoSensor_SyncConfig(obj->sensor);

    return (const kIpAddress*) &obj->profinet.gateway;
}

GoFx(const kChar*) GoEthernet_ProfinetDeviceName(GoEthernet ethernet)
{
    kObj(GoEthernet, ethernet);

    GoSensor_SyncConfig(obj->sensor);

    return (const kChar*) kString_Chars(obj->profinet.deviceName);
}

GoFx(kSize) GoEthernet_EventOptionCount(GoEthernet ethernet)
{
    kObj(GoEthernet, ethernet);

    return kArrayList_Count(obj->eventOptions);
}

GoFx(GoEventType) GoEthernet_EventOptionAt(GoEthernet ethernet, kSize index)
{
    kObj(GoEthernet, ethernet);

    kAssert(index < kArrayList_Count(obj->eventOptions));

    return *kArrayList_AtT(obj->eventOptions, index, GoEventType);
}

GoFx(kSize) GoEthernet_EventCount(GoEthernet ethernet)
{
    kObj(GoEthernet, ethernet);

    return kArrayList_Count(obj->events);
}

GoFx(GoEventType) GoEthernet_EventAt(GoEthernet ethernet, kSize index)
{
    kObj(GoEthernet, ethernet);

    kAssert(index < kArrayList_Count(obj->events));

    return *kArrayList_AtT(obj->events, index, GoEventType);
}

GoFx(kStatus) GoEthernet_AddEvent(GoEthernet ethernet, GoEventType type)
{
    kObj(GoEthernet, ethernet);

    kCheck(kArrayList_AddT(obj->events, &type));
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(kStatus) GoEthernet_RemoveEvent(GoEthernet ethernet, kSize index)
{
    kObj(GoEthernet, ethernet);

    kAssert(index < kArrayList_Count(obj->eventOptions));
    kCheck(kArrayList_Discard(obj->eventOptions, index));
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(kStatus) GoEthernet_ClearEvents(GoEthernet ethernet)
{
    kObj(GoEthernet, ethernet);

    kCheck(kArrayList_Clear(obj->events));
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

/**
 * @file    GoDigital.c
 *
 * @internal
 * Copyright (C) 2016-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <GoSdk/Outputs/GoDigital.h>
#include <GoSdk/GoUtils.h>
#include <GoSdk/GoSensor.h>

kBeginClassEx(Go, GoDigital)
kAddVMethod(GoDigital, kObject, VRelease)
kEndClassEx()

GoFx(kStatus) GoDigital_Construct(GoDigital* digital, kObject sensor, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback(allocator);
    kStatus status;

    kCheck(kAlloc_GetObject(alloc, kTypeOf(GoDigital), digital));

    if (!kSuccess(status = GoDigital_Init(*digital, kTypeOf(GoDigital), sensor, alloc)))
    {
        kAlloc_FreeRef(alloc, digital);
    }

    return status;
}

GoFx(kStatus) GoDigital_Init(GoDigital digital, kType type, kObject sensor, kAlloc alloc)
{
    kObjR(GoDigital, digital);
    kStatus exception;

    kCheck(kObject_Init(digital, type, alloc));
    kZero(obj->xml);
    kZero(obj->xmlItem);
    kZero(obj->decisionOptions);
    kZero(obj->decisionSources);
    obj->passMode = GO_DIGITAL_PASS_TRUE;
    obj->scheduleEnabled = kFALSE;
    obj->signalType = GO_DIGITAL_SIGNAL_PULSED;
    obj->delay = 0;
    obj->delayDomain = GO_OUTPUT_DELAY_DOMAIN_TIME;
    obj->event = 0;
    kZero(obj->pulseWidth);
    obj->invertOutput = kFALSE;
    obj->outputUsed = kFALSE;

    obj->sensor = sensor;

    kTry
    {
        kTest(kArrayList_Construct(&obj->decisionOptions, kTypeOf(k32u), 0, alloc));
        kTest(kArrayList_Construct(&obj->decisionSources, kTypeOf(k32u), 0, alloc));
    }
    kCatch(&exception)
    {
        GoDigital_VRelease(digital);
        kEndCatch(exception);
    }


    return kOK;
}

GoFx(kStatus) GoDigital_VRelease(GoDigital digital)
{
    kObj(GoDigital, digital);

    kCheck(kDestroyRef(&obj->decisionOptions));
    kCheck(kDestroyRef(&obj->decisionSources));

    kCheck(kObject_VRelease(digital));

    return kOK;
}

GoFx(kStatus) GoDigital_Read(GoDigital digital, kXml xml, kXmlItem item)
{
    kObj(GoDigital, digital);
    kChar* text = kNULL;
    kXmlItem tempItem = kNULL;
    kSize textCapacity = GO_OUTPUT_SOURCE_TEXT_CAPACITY;
    k32u temp32u;

    obj->xml = xml;
    obj->xmlItem = item;

    kTry
    {
        kTest(kObject_GetMemZero(digital, sizeof(kChar) * textCapacity, &text));

        kTest(GoConfig_ReadAttrBoolOptional(xml, item, "used", kTRUE, &obj->outputUsed));;

        tempItem = kXml_Child(xml, item, "Measurements");
        if (!kIsNull(tempItem))
        {
            kTest(kXml_AttrText(xml, tempItem, "options", text, textCapacity));
            kTest(GoOptionList_ParseList32u(text, obj->decisionOptions));
            kTest(kXml_ItemText(xml, tempItem, text, textCapacity));
            kTest(GoOptionList_ParseList32u(text, obj->decisionSources));
        }

        kTest(kXml_Child32u(xml, item, "Event", &temp32u));
        obj->event = (k32s)temp32u;
        kTest(kXml_Child32u(xml, item, "SignalType", &temp32u));
        obj->signalType = (k32s)temp32u;
        kTest(kXml_Child32s(xml, item, "ScheduleEnabled", &obj->scheduleEnabled));

        tempItem = kXml_Child(xml, item, "PulseWidth");
        if (!kIsNull(tempItem))
        {
            kTest(kXml_Item32u(xml, tempItem, &obj->pulseWidth.value));
            kTest(kXml_Attr32u(xml, tempItem, "min", &obj->pulseWidth.min));
            kTest(kXml_Attr32u(xml, tempItem, "max", &obj->pulseWidth.max));
        }

        kTest(kXml_Child32s(xml, item, "PassMode", &obj->passMode));
        kTest(kXml_Child64s(xml, item, "Delay", &obj->delay));
        kTest(kXml_Child32s(xml, item, "DelayDomain", &obj->delayDomain));

        if (kXml_ChildExists(xml, item, "Inverted"))
        {
            kTest(kXml_ChildBool(xml, item, "Inverted", &obj->invertOutput));
        }
    }
    kFinally
    {
        kObject_FreeMem(digital, text);
        kEndFinally();
    }

    return kOK;
}

GoFx(kStatus) GoDigital_Write(GoDigital digital, kXml xml, kXmlItem item)
{
    kObj(GoDigital, digital);
    kChar* text = kNULL;
    kSize textCapacity = GO_OUTPUT_SOURCE_TEXT_CAPACITY;

    kTry
    {
        kTest(kObject_GetMemZero(digital, sizeof(kChar) * textCapacity, &text));

        kTest(GoOptionList_Format32u(kArrayList_DataT(obj->decisionSources, k32u), kArrayList_Count(obj->decisionSources), text, textCapacity));
        kTest(kXml_SetChildText(xml, item, "Measurements", text));

        kTest(kXml_SetChild32u(xml, item, "Event", obj->event));
        kTest(kXml_SetChild32u(xml, item, "SignalType", obj->signalType));
        kTest(kXml_SetChild32u(xml, item, "ScheduleEnabled", obj->scheduleEnabled));
        kTest(kXml_SetChild32u(xml, item, "PulseWidth", obj->pulseWidth.value));
        kTest(kXml_SetChild32s(xml, item, "PassMode", obj->passMode));
        kTest(kXml_SetChild64s(xml, item, "Delay", obj->delay));
        kTest(kXml_SetChild32s(xml, item, "DelayDomain", obj->delayDomain));
        kTest(kXml_SetChildBool(xml, item, "Inverted", obj->invertOutput));
    }
    kFinally
    {
        kObject_FreeMem(digital, text);
        kEndFinally();
    }

    //Forwards Compatibility
    kCheck(GoUtils_XmlMerge(obj->xml, obj->xmlItem, xml, item));

    return kOK;
}

GoFx(kStatus) GoDigital_SetEvent(GoDigital digital, GoDigitalEvent event)
{
    kObj(GoDigital, digital);

    obj->event = event;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(GoDigitalEvent) GoDigital_Event(GoDigital digital)
{
    kObj(GoDigital, digital);

    GoSensor_SyncConfig(obj->sensor);

    return obj->event;
}

GoFx(kArrayList) GoDigital_OptionList(GoDigital digital)
{
    kObj(GoDigital, digital);

    GoSensor_SyncConfig(obj->sensor);

    return obj->decisionOptions;
}

GoFx(kSize) GoDigital_OptionCount(GoDigital digital)
{
    kObj(GoDigital, digital);

    GoSensor_SyncConfig(obj->sensor);

    return kArrayList_Count(obj->decisionOptions);
}

GoFx(k32u) GoDigital_OptionAt(GoDigital digital, kSize index)
{
    kObj(GoDigital, digital);

    GoSensor_SyncConfig(obj->sensor);

    kAssert(index < GoDigital_OptionCount(digital));

    return kArrayList_AsT(obj->decisionOptions, index, k32u);
}

GoFx(kSize) GoDigital_SourceCount(GoDigital digital)
{
    kObj(GoDigital, digital);

    GoSensor_SyncConfig(obj->sensor);

    return kArrayList_Count(obj->decisionSources);
}

GoFx(k32u) GoDigital_SourceAt(GoDigital digital, kSize index)
{
    kObj(GoDigital, digital);

    GoSensor_SyncConfig(obj->sensor);

    kAssert(index < GoDigital_SourceCount(digital));

    return kArrayList_AsT(obj->decisionSources, index, k32u);
}

GoFx(kStatus) GoDigital_AddSource(GoDigital digital, k32u sourceId)
{
    kObj(GoDigital, digital);
    kArrayList options = GoDigital_OptionList(digital);

    kCheck(GoOptionList_Check32u(kArrayList_DataT(options, const k32u), kArrayList_Count(options), sourceId));
    kCheck(kArrayList_AddT(obj->decisionSources, &sourceId));
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(kStatus) GoDigital_RemoveSource(GoDigital digital, kSize index)
{
    kObj(GoDigital, digital);

    kCheck(kArrayList_Discard(obj->decisionSources, index));
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(kStatus) GoDigital_ClearSources(GoDigital digital)
{
    kObj(GoDigital, digital);
    kCheck(kArrayList_Clear(obj->decisionSources));
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(kStatus) GoDigital_SetPassMode(GoDigital digital, GoDigitalPass pass)
{
    kObj(GoDigital, digital);

    obj->passMode = pass;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(GoDigitalPass) GoDigital_PassMode(GoDigital digital)
{
    kObj(GoDigital, digital);

    GoSensor_SyncConfig(obj->sensor);

    return obj->passMode;
}

GoFx(k32u) GoDigital_PulseWidthLimitMin(GoDigital digital)
{
    kObj(GoDigital, digital);

    GoSensor_SyncConfig(obj->sensor);

    return obj->pulseWidth.min;
}

GoFx(k32u) GoDigital_PulseWidthLimitMax(GoDigital digital)
{
    kObj(GoDigital, digital);

    GoSensor_SyncConfig(obj->sensor);

    return obj->pulseWidth.max;
}

GoFx(kStatus) GoDigital_SetPulseWidth(GoDigital digital, k32u width)
{
    kObj(GoDigital, digital);

    kCheckArgs(GoUtils_MinMax_(width, GoDigital_PulseWidthLimitMin(digital), GoDigital_PulseWidthLimitMax(digital)));

    obj->pulseWidth.value = width;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(k32u) GoDigital_PulseWidth(GoDigital digital)
{
    kObj(GoDigital, digital);

    GoSensor_SyncConfig(obj->sensor);

    return obj->pulseWidth.value;
}

GoFx(kStatus) GoDigital_SetSignalType(GoDigital digital, GoDigitalSignal signal)
{
    kObj(GoDigital, digital);

    obj->signalType = signal;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(GoDigitalSignal) GoDigital_SignalType(GoDigital digital)
{
    kObj(GoDigital, digital);

    GoSensor_SyncConfig(obj->sensor);

    return obj->signalType;
}

GoFx(kStatus) GoDigital_SetDelay(GoDigital digital, k64s delay)
{
    kObj(GoDigital, digital);

    obj->delay = delay;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(k64s) GoDigital_Delay(GoDigital digital)
{
    kObj(GoDigital, digital);

    GoSensor_SyncConfig(obj->sensor);

    return obj->delay;
}

GoFx(kStatus) GoDigital_SetDelayDomain(GoDigital digital, GoOutputDelayDomain delayDomain)
{
    kObj(GoDigital, digital);

    obj->delayDomain = delayDomain;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(GoOutputDelayDomain) GoDigital_DelayDomain(GoDigital digital)
{
    kObj(GoDigital, digital);

    GoSensor_SyncConfig(obj->sensor);

    return obj->delayDomain;
}

GoFx(kStatus) GoDigital_EnableSchedule(GoDigital digital, kBool enabled )
{
    kObj(GoDigital, digital);

    obj->scheduleEnabled = enabled;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(kBool) GoDigital_ScheduleEnabled(GoDigital digital)
{
    kObj(GoDigital, digital);

    GoSensor_SyncConfig(obj->sensor);

    return obj->scheduleEnabled;
}

GoFx(kBool) GoDigital_IsOutputInverted(GoDigital digital)
{
    kObj(GoDigital, digital);

    GoSensor_SyncConfig(obj->sensor);

    return obj->invertOutput;
}

GoFx(kStatus) GoDigital_SetOutputInverted(GoDigital digital, kBool invert)
{
    kObj(GoDigital, digital);

    obj->invertOutput = invert;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

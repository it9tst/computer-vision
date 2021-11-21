/** 
 * @file    GoAnalog.c
 *
 * @internal
 * Copyright (C) 2016-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <GoSdk/Outputs/GoAnalog.h>
#include <GoSdk/GoUtils.h>
#include <GoSdk/GoSensor.h>

kBeginClassEx(Go, GoAnalog)
kAddVMethod(GoAnalog, kObject, VRelease)
kEndClassEx()

GoFx(kStatus) GoAnalog_Construct(GoAnalog* analog, kObject sensor, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback(allocator);
    kStatus status; 

    kCheck(kAlloc_GetObject(alloc, kTypeOf(GoAnalog), analog)); 

    if (!kSuccess(status = GoAnalog_Init(*analog, kTypeOf(GoAnalog), sensor, alloc)))
    {
        kAlloc_FreeRef(alloc, analog); 
    }

    return status; 
} 

GoFx(kStatus) GoAnalog_Init(GoAnalog analog, kType type, kObject sensor, kAlloc alloc)
{
    kObjR(GoAnalog, analog); 
    kStatus exception;

    kCheck(kObject_Init(analog, type, alloc)); 
    kZero(obj->xml);
    kZero(obj->xmlItem);
    kZero(obj->measurementOptions);
    kZero(obj->measurementSource);
    kZero(obj->currentMin);
    kZero(obj->currentMax);
    kZero(obj->currentInvalid);
    obj->scheduleEnabled = kFALSE;
    obj->delay = 0;
    obj->delayDomain = GO_OUTPUT_DELAY_DOMAIN_TIME;
    obj->event = 0;
    obj->dataScaleMin = 0.0;
    obj->dataScaleMax = 0.0;
    obj->outputUsed = kFALSE;

    obj->sensor = sensor;
    
    kTry
    {
        kTest(kArrayList_Construct(&obj->measurementSource, kTypeOf(k32u), 0, alloc)); 
        kTest(kArrayList_Construct(&obj->measurementOptions, kTypeOf(k32u), 0, alloc)); 
    }
    kCatch(&exception)
    {
        GoAnalog_VRelease(analog);
        kEndCatch(exception);
    }

    return kOK; 
}

GoFx(kStatus) GoAnalog_VRelease(GoAnalog analog)
{
    kObj(GoAnalog, analog); 

    kCheck(kDestroyRef(&obj->measurementSource)); 
    kCheck(kDestroyRef(&obj->measurementOptions)); 

    kCheck(kObject_VRelease(analog)); 

    return kOK; 
}

GoFx(kStatus) GoAnalog_Read(GoAnalog analog, kXml xml, kXmlItem item)
{
    kObj(GoAnalog, analog);
    kChar* text = kNULL; 
    kXmlItem tempItem = kNULL;
    kSize textCapacity = GO_OUTPUT_SOURCE_TEXT_CAPACITY; 
    k32u temp32u;

    obj->xml = xml;
    obj->xmlItem = item;

    kTry
    {
        kTest(GoConfig_ReadAttrBoolOptional(xml, item, "used", kTRUE, &obj->outputUsed));;
        
        kTest(kObject_GetMemZero(analog, sizeof(kChar) * textCapacity, &text)); 

        tempItem = kXml_Child(xml, item, "Measurement");
        if (!kIsNull(tempItem))
        {
            kTest(kXml_AttrText(xml, tempItem, "options", text, textCapacity));
            kTest(GoOptionList_ParseList32u(text, obj->measurementOptions));
        }

        kTest(kXml_ChildText(xml, item, "Measurement", text, textCapacity)); 
        kTest(GoOptionList_ParseList32u(text, obj->measurementSource));  
        if (kArrayList_Count(obj->measurementSource) > 1)
        {
            return kERROR_PARAMETER;
        }

        kTest(kXml_Child32u(xml, item, "Event", &temp32u)); 
        obj->event = (k32s)temp32u;
        kTest(kXml_ChildBool(xml, item, "ScheduleEnabled", &obj->scheduleEnabled)); 

        tempItem = kXml_Child(xml, item, "CurrentMin");
        if (!kIsNull(tempItem))
        {
            kTest(kXml_Item64f(xml, tempItem, &obj->currentMin.value));
            kTest(kXml_Attr64f(xml, tempItem, "min", &obj->currentMin.min));
            kTest(kXml_Attr64f(xml, tempItem, "max", &obj->currentMin.max));
        }

        tempItem = kXml_Child(xml, item, "CurrentMax");
        if (!kIsNull(tempItem))
        {
            kTest(kXml_Item64f(xml, tempItem, &obj->currentMax.value));
            kTest(kXml_Attr64f(xml, tempItem, "min", &obj->currentMax.min));
            kTest(kXml_Attr64f(xml, tempItem, "max", &obj->currentMax.max));
        }

        kTest(kXml_ChildBool(xml, item, "CurrentInvalidEnabled", &obj->currentInvalid.enabled)); 
        tempItem = kXml_Child(xml, item, "CurrentInvalid");
        if (!kIsNull(tempItem))
        {
            kTest(kXml_Item64f(xml, tempItem, &obj->currentInvalid.value));
            kTest(kXml_Attr64f(xml, tempItem, "min", &obj->currentInvalid.min));
            kTest(kXml_Attr64f(xml, tempItem, "max", &obj->currentInvalid.max));
        }

        kTest(kXml_Child64f(xml, item, "DataScaleMin", &obj->dataScaleMin)); 
        kTest(kXml_Child64f(xml, item, "DataScaleMax", &obj->dataScaleMax)); 

        kTest(kXml_Child64s(xml, item, "Delay", &obj->delay));
        kTest(kXml_Child32s(xml, item, "DelayDomain", &obj->delayDomain));
    }
    kFinally
    {
        kObject_FreeMem(analog, text); 
        kEndFinally(); 
    }

    return kOK;
}

GoFx(kStatus) GoAnalog_Write(GoAnalog analog, kXml xml, kXmlItem item)
{
    kObj(GoAnalog, analog);
    kChar* text = kNULL; 
    kSize textCapacity = GO_OUTPUT_SOURCE_TEXT_CAPACITY; 

    kTry
    {
        kTest(kObject_GetMemZero(analog, sizeof(kChar) * textCapacity, &text)); 

        kTest(GoOptionList_Format32u(kArrayList_DataT(obj->measurementSource, k32u), kArrayList_Count(obj->measurementSource), text, textCapacity)); 
        kTest(kXml_SetChildText(xml, item, "Measurement", text)); 

        kTest(kXml_SetChild32s(xml, item, "Event", obj->event)); 
        kTest(kXml_SetChild32s(xml, item, "ScheduleEnabled", obj->scheduleEnabled)); 

        kTest(kXml_SetChild64f(xml, item, "CurrentMin", obj->currentMin.value)); 
        kTest(kXml_SetChild64f(xml, item, "CurrentMax", obj->currentMax.value)); 
        kTest(kXml_SetChild64f(xml, item, "CurrentInvalidEnabled", obj->currentInvalid.enabled)); 
        kTest(kXml_SetChild64f(xml, item, "CurrentInvalid", obj->currentInvalid.value)); 

        kTest(kXml_SetChild64f(xml, item, "DataScaleMin", obj->dataScaleMin)); 
        kTest(kXml_SetChild64f(xml, item, "DataScaleMax", obj->dataScaleMax)); 
        kTest(kXml_SetChild64s(xml, item, "Delay", obj->delay)); 
        kTest(kXml_SetChild32s(xml, item, "DelayDomain", obj->delayDomain)); 
    }
    kFinally
    {
        kObject_FreeMem(analog, text); 
        kEndFinally(); 
    }

    //Forwards Compatibility
    kCheck(GoUtils_XmlMerge(obj->xml, obj->xmlItem, xml, item));

    return kOK; 
}

GoFx(kStatus) GoAnalog_SetEvent(GoAnalog analog, GoAnalogEvent event)
{
    kObj(GoAnalog, analog);

    obj->event = event; 
    kCheck(GoSensor_SetConfigModified(obj->sensor)); 

    return kOK; 
}

GoFx(GoAnalogEvent) GoAnalog_Event(GoAnalog analog)
{
    kObj(GoAnalog, analog);

    GoSensor_SyncConfig(obj->sensor);

    return obj->event; 
} 

GoFx(kArrayList) GoAnalog_OptionList(GoAnalog analog)
{
    kObj(GoAnalog, analog);

    GoSensor_SyncConfig(obj->sensor);

    return obj->measurementOptions;
}

GoFx(kSize) GoAnalog_OptionCount( GoAnalog analog )
{
    kObj(GoAnalog, analog);
    kArrayList list;

    GoSensor_SyncConfig(obj->sensor);
    list = obj->measurementOptions;

    return kArrayList_Count(list);
}

GoFx(k32u) GoAnalog_OptionAt(GoAnalog analog, kSize index)
{
    kObj(GoAnalog, analog);
    k32u* option = kNULL; 
    kArrayList list;
    
    GoSensor_SyncConfig(obj->sensor);

    list = obj->measurementOptions;
    kAssert(index < kArrayList_Count(list));

    return kArrayList_AsT(list, index, k32u);
} 

GoFx(kStatus) GoAnalog_SetSource( GoAnalog analog, k32u sourceId )
{
    kObj(GoAnalog, analog);
    kArrayList optionList = GoAnalog_OptionList(analog);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));

    kCheck(GoOptionList_Check32u(kArrayList_DataT(optionList, const k32u), kArrayList_Count(optionList), sourceId));
    kCheck(kArrayList_Clear(obj->measurementSource));
    kCheck(kArrayList_AddT(obj->measurementSource, &sourceId));
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(k32u) GoAnalog_Source(GoAnalog analog)
{
    kObj(GoAnalog, analog);

    GoSensor_SyncConfig(obj->sensor);

    return kArrayList_AsT(obj->measurementSource, 0, k32u);
}

GoFx(kStatus) GoAnalog_ClearSource(GoAnalog analog)
{
    kObj(GoAnalog, analog);
    kCheck(kArrayList_Clear(obj->measurementSource));  
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}   

GoFx(k64f) GoAnalog_CurrentLimitMin(GoAnalog analog)
{
    kObj(GoAnalog, analog);

    GoSensor_SyncConfig(obj->sensor);

    return obj->currentMin.value; 
}  

GoFx(k64f) GoAnalog_CurrentLimitMax(GoAnalog analog)
{
    kObj(GoAnalog, analog);

    GoSensor_SyncConfig(obj->sensor);

    return obj->currentMax.value; 
} 

GoFx(kStatus) GoAnalog_SetCurrentMin(GoAnalog analog, k64f min)
{
    kObj(GoAnalog, analog);

    kCheckArgs(GoUtils_MinMax_(min, GoAnalog_CurrentMinLimitMin(analog), GoAnalog_CurrentMinLimitMax(analog)));

    obj->currentMin.value = min; 
    kCheck(GoSensor_SetConfigModified(obj->sensor)); 

    return kOK; 
} 

GoFx(k64f) GoAnalog_CurrentMin(GoAnalog analog)
{
    kObj(GoAnalog, analog);

    GoSensor_SyncConfig(obj->sensor);

    return obj->currentMin.value;  
} 

GoFx(k64f) GoAnalog_CurrentMinLimitMin(GoAnalog analog)
{
    kObj(GoAnalog, analog);

    GoSensor_SyncConfig(obj->sensor);

    return obj->currentMin.min;  
}  

GoFx(k64f) GoAnalog_CurrentMinLimitMax(GoAnalog analog)
{
    kObj(GoAnalog, analog);

    GoSensor_SyncConfig(obj->sensor);

    return obj->currentMin.max;  
}  

GoFx(kStatus) GoAnalog_SetCurrentMax(GoAnalog analog, k64f max)
{
    kObj(GoAnalog, analog);

    kCheckArgs(GoUtils_MinMax_(max, GoAnalog_CurrentMaxLimitMin(analog), GoAnalog_CurrentMaxLimitMax(analog)));

    obj->currentMax.value = max; 
    kCheck(GoSensor_SetConfigModified(obj->sensor)); 

    return kOK; 
} 

GoFx(k64f) GoAnalog_CurrentMax(GoAnalog analog)
{
    kObj(GoAnalog, analog);

    GoSensor_SyncConfig(obj->sensor);

    return obj->currentMax.value;  
}  

GoFx(k64f) GoAnalog_CurrentMaxLimitMin(GoAnalog analog)
{
    kObj(GoAnalog, analog);

    GoSensor_SyncConfig(obj->sensor);

    return obj->currentMax.min;  
}  

GoFx(k64f) GoAnalog_CurrentMaxLimitMax(GoAnalog analog)
{
    kObj(GoAnalog, analog);

    GoSensor_SyncConfig(obj->sensor);

    return obj->currentMax.max;  
}  

GoFx(kStatus) GoAnalog_EnableCurrentInvalid(GoAnalog analog, kBool enable)
{
    kObj(GoAnalog, analog);

    obj->currentInvalid.enabled = enable; 
    kCheck(GoSensor_SetConfigModified(obj->sensor)); 

    return kOK; 
} 

GoFx(kBool) GoAnalog_CurrentInvalidEnabled(GoAnalog analog)
{
    kObj(GoAnalog, analog);

    GoSensor_SyncConfig(obj->sensor);

    return obj->currentInvalid.enabled;  
}  

GoFx(kStatus) GoAnalog_SetCurrentInvalid(GoAnalog analog, k64f invalid)
{
    kObj(GoAnalog, analog);

    kCheckArgs(GoUtils_MinMax_(invalid, GoAnalog_CurrentInvalidLimitMin(analog), GoAnalog_CurrentInvalidLimitMax(analog)));

    obj->currentInvalid.value = invalid; 
    kCheck(GoSensor_SetConfigModified(obj->sensor)); 

    return kOK; 
} 

GoFx(k64f) GoAnalog_CurrentInvalid(GoAnalog analog)
{
    kObj(GoAnalog, analog);

    GoSensor_SyncConfig(obj->sensor);

    return obj->currentInvalid.value;  
}  

GoFx(k64f) GoAnalog_CurrentInvalidLimitMin(GoAnalog analog)
{
    kObj(GoAnalog, analog);

    GoSensor_SyncConfig(obj->sensor);

    return obj->currentInvalid.min;  
}  

GoFx(k64f) GoAnalog_CurrentInvalidLimitMax(GoAnalog analog)
{
    kObj(GoAnalog, analog);

    GoSensor_SyncConfig(obj->sensor);

    return obj->currentInvalid.max;  
}  

GoFx(kStatus) GoAnalog_SetDataScaleMin(GoAnalog analog, k64f min)
{
    kObj(GoAnalog, analog);

    obj->dataScaleMin = min; 
    kCheck(GoSensor_SetConfigModified(obj->sensor)); 

    return kOK; 
} 

GoFx(k64f) GoAnalog_DataScaleMin(GoAnalog analog)
{
    kObj(GoAnalog, analog);

    GoSensor_SyncConfig(obj->sensor);

    return obj->dataScaleMin;  
}  

GoFx(kStatus) GoAnalog_SetDataScaleMax(GoAnalog analog, k64f max)
{
    kObj(GoAnalog, analog);

    obj->dataScaleMax = max; 
    kCheck(GoSensor_SetConfigModified(obj->sensor)); 

    return kOK; 
} 

GoFx(k64f) GoAnalog_DataScaleMax(GoAnalog analog)
{
    kObj(GoAnalog, analog);

    GoSensor_SyncConfig(obj->sensor);

    return obj->dataScaleMax;  
}  

GoFx(kStatus) GoAnalog_SetDelay(GoAnalog analog, k64s delay)
{
    kObj(GoAnalog, analog);

    obj->delay = delay; 
    kCheck(GoSensor_SetConfigModified(obj->sensor)); 

    return kOK; 
} 

GoFx(k64s) GoAnalog_Delay(GoAnalog analog)
{
    kObj(GoAnalog, analog);

    GoSensor_SyncConfig(obj->sensor);

    return obj->delay;  
} 

GoFx(kStatus) GoAnalog_SetDelayDomain(GoAnalog analog, GoOutputDelayDomain delayDomain)
{
    kObj(GoAnalog, analog);

    obj->delayDomain = delayDomain; 
    kCheck(GoSensor_SetConfigModified(obj->sensor)); 

    return kOK; 
} 

GoFx(GoOutputDelayDomain) GoAnalog_DelayDomain(GoAnalog analog)
{
    kObj(GoAnalog, analog);

    GoSensor_SyncConfig(obj->sensor);

    return obj->delayDomain;  
} 

GoFx(kStatus) GoAnalog_EnableSchedule(GoAnalog analog, kBool enabled )
{
    kObj(GoAnalog, analog);

    obj->scheduleEnabled = enabled; 
    kCheck(GoSensor_SetConfigModified(obj->sensor)); 

    return kOK; 
} 

GoFx(kBool) GoAnalog_ScheduleEnabled(GoAnalog analog)
{
    kObj(GoAnalog, analog);

    GoSensor_SyncConfig(obj->sensor);

    return obj->scheduleEnabled;  
} 


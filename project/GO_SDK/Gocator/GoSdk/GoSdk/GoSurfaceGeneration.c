/** 
 * @file    GoSurfaceGeneration.c
 *
 * @internal
 * Copyright (C) 2016-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <GoSdk/GoSurfaceGeneration.h>
#include <GoSdk/GoSensor.h>
#include <GoSdk/GoUtils.h>

kBeginClassEx(Go, GoSurfaceGeneration)
    kAddVMethod(GoSurfaceGeneration, kObject, VRelease)
kEndClassEx()

GoFx(kStatus) GoSurfaceGeneration_Construct(GoSurfaceGeneration* surface, kObject sensor, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback(allocator);
    kStatus status; 

    kCheck(kAlloc_GetObject(alloc, kTypeOf(GoSurfaceGeneration), surface)); 

    if (!kSuccess(status = GoSurfaceGeneration_Init(*surface, kTypeOf(GoSurfaceGeneration), sensor, alloc)))
    {
        kAlloc_FreeRef(alloc, surface); 
    }

    return status; 
} 

GoFx(kStatus) GoSurfaceGeneration_Init(GoSurfaceGeneration surface, kType type, kObject sensor, kAlloc alloc)
{
    kObjR(GoSurfaceGeneration, surface); 

    kCheck(kObject_Init(surface, type, alloc)); 
    kZero(obj->xml);
    kZero(obj->xmlItem);
    obj->type = GO_SURFACE_GENERATION_TYPE_CONTINUOUS;
    kZero(obj->fixedLength);
    kZero(obj->variableLength);
    kZero(obj->rotational);

    obj->sensor = sensor; 
    kCheck(kArrayList_Construct(&obj->fixedLength.externalInputIndexOptions, kTypeOf(k32s), 0, alloc));

    return kOK; 
}

GoFx(kStatus) GoSurfaceGeneration_VRelease(GoSurfaceGeneration surface)
{
    kObj(GoSurfaceGeneration, surface); 

    kCheck(kDisposeRef(&obj->fixedLength.externalInputIndexOptions));

    kCheck(kObject_VRelease(surface)); 

    return kOK; 
}

GoFx(kStatus) GoSurfaceGeneration_Read(GoSurfaceGeneration surface, kXml xml, kXmlItem item)
{
    kObj(GoSurfaceGeneration, surface); 
    kXmlItem tempItem = kNULL;
    kXmlItem tempItem2 = kNULL;
    kText128 tempText;

    obj->xml = xml;
    obj->xmlItem = item;
    
    kCheck(kXml_Child32s(xml, item, "Type", &obj->type));

    kCheck(!kIsNull(tempItem = kXml_Child(xml, item, "FixedLength")));
    kCheck(kXml_Child32s(xml, tempItem, "StartTrigger", &obj->fixedLength.startTrigger));
    tempItem2 = kXml_Child(xml, tempItem, "ExternalInputIndex");
    if (!kIsNull(tempItem2))
    {
        kCheck(kXml_Item32s(xml, tempItem2, (k32s*)&obj->fixedLength.externalInputIndex));
        kCheck(kXml_AttrText(xml, tempItem2, "options", tempText, kCountOf(tempText)));
        kCheck(kXml_AttrBool(xml, tempItem2, "used", &obj->fixedLength.externalInputIndexUsed));
        kCheck(GoOptionList_ParseList32s(tempText, obj->fixedLength.externalInputIndexOptions));
    }

    kCheck(GoConfig_ReadRangeElement64f(xml, tempItem, "Length", &obj->fixedLength.length));

    kCheck(!kIsNull(tempItem = kXml_Child(xml, item, "VariableLength")));
    kCheck(GoConfig_ReadRangeElement64f(xml, tempItem, "MaxLength", &obj->variableLength.maxLength));

    kCheck(!kIsNull(tempItem = kXml_Child(xml, item, "Rotational")));
    kCheck(GoConfig_ReadRangeElement64f(xml, tempItem, "Circumference", &obj->rotational.circumference));

    return kOK; 
}

GoFx(kStatus) GoSurfaceGeneration_Write(GoSurfaceGeneration surface, kXml xml, kXmlItem item)
{
    kObj(GoSurfaceGeneration, surface); 
    kXmlItem tempItem = kNULL;
    kXmlItem tempItem2 = kNULL;
    kText128 tempText;

    kCheck(kXml_SetChild32s(xml, item, "Type", obj->type));

    kCheck(kXml_AddItem(xml, item, "FixedLength", &tempItem));
    kCheck(kXml_SetChild32s(xml, tempItem, "StartTrigger", obj->fixedLength.startTrigger));
    kCheck(kXml_AddItem(xml, tempItem, "ExternalInputIndex", &tempItem2));
    kCheck(GoOptionList_Format32s(kArrayList_DataT(obj->fixedLength.externalInputIndexOptions, k32s), kArrayList_Count(obj->fixedLength.externalInputIndexOptions), tempText, kCountOf(tempText)));
    kCheck(kXml_SetAttrText(xml, tempItem2, "options", tempText));
    kCheck(kXml_SetItem32s(xml, tempItem2, obj->fixedLength.externalInputIndex));
    kCheck(kXml_SetAttrBool(xml, tempItem2, "used", obj->fixedLength.externalInputIndexUsed));
    kCheck(GoConfig_WriteRangeElement64f(xml, tempItem, "Length", obj->fixedLength.length));

    kCheck(kXml_AddItem(xml, item, "VariableLength", &tempItem));
    kCheck(GoConfig_WriteRangeElement64f(xml, tempItem, "MaxLength", obj->variableLength.maxLength));

    kCheck(kXml_AddItem(xml, item, "Rotational", &tempItem));
    kCheck(GoConfig_WriteRangeElement64f(xml, tempItem, "Circumference", obj->rotational.circumference));

    //Forwards Compatibility
    kCheck(GoUtils_XmlMerge(obj->xml, obj->xmlItem, xml, item));

    return kOK; 
}


GoFx(kStatus) GoSurfaceGeneration_SetGenerationType(GoSurfaceGeneration surface, GoSurfaceGenerationType type)
{
    kObj(GoSurfaceGeneration, surface); 

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));
    obj->type = type;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(GoSurfaceGenerationType) GoSurfaceGeneration_GenerationType(GoSurfaceGeneration surface)
{
    kObj(GoSurfaceGeneration, surface); 

    GoSensor_SyncConfig(obj->sensor);

    return obj->type;
}

GoFx(kStatus) GoSurfaceGenerationFixedLength_SetStartTrigger(GoSurfaceGeneration surface, GoSurfaceGenerationStartTrigger trigger)
{
    kObj(GoSurfaceGeneration, surface); 

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));
    obj->fixedLength.startTrigger = trigger;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(GoSurfaceGenerationStartTrigger) GoSurfaceGenerationFixedLength_StartTrigger(GoSurfaceGeneration surface)
{
    kObj(GoSurfaceGeneration, surface); 

    GoSensor_SyncConfig(obj->sensor);

    return obj->fixedLength.startTrigger;
}

GoFx(kStatus) GoSurfaceGenerationFixedLength_SetTriggerExternalInputIndex(GoSurfaceGeneration surface, k32s index)
{
    kObj(GoSurfaceGeneration, surface);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->fixedLength.externalInputIndex = index;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(k32s) GoSurfaceGenerationFixedLength_TriggerExternalInputIndex(GoSurfaceGeneration surface)
{
    kObj(GoSurfaceGeneration, surface);

    GoSensor_SyncConfig(obj->sensor);

    return obj->fixedLength.externalInputIndex;
}

GoFx(kBool) GoSurfaceGenerationFixedLength_TriggerExternalInputIndexUsed(GoSurfaceGeneration surface)
{
    kObj(GoSurfaceGeneration, surface);

    GoSensor_SyncConfig(obj->sensor);

    return obj->fixedLength.externalInputIndexUsed;
}

GoFx(kSize) GoSurfaceGenerationFixedLength_TriggerExternalInputIndexOptionCount(GoSurfaceGeneration surface)
{
    kObj(GoSurfaceGeneration, surface);

    GoSensor_SyncConfig(obj->sensor);

    return kArrayList_Count(obj->fixedLength.externalInputIndexOptions);
}

GoFx(kStatus) GoSurfaceGenerationFixedLength_SetLength(GoSurfaceGeneration surface, k64f length)
{
    kObj(GoSurfaceGeneration, surface); 

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));
    kCheck(GoSensor_SyncConfig(obj->sensor));
    kCheckArgs(GoUtils_MinMax_(length, obj->fixedLength.length.min, obj->fixedLength.length.max));

    obj->fixedLength.length.value = length;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(k64f) GoSurfaceGenerationFixedLength_Length(GoSurfaceGeneration surface)
{
    kObj(GoSurfaceGeneration, surface); 

    GoSensor_SyncConfig(obj->sensor);

    return obj->fixedLength.length.value;
}

GoFx(k64f) GoSurfaceGenerationFixedLength_LengthLimitMax(GoSurfaceGeneration surface)
{
    kObj(GoSurfaceGeneration, surface); 

    GoSensor_SyncConfig(obj->sensor);

    return obj->fixedLength.length.max;
}

GoFx(k64f) GoSurfaceGenerationFixedLength_LengthLimitMin(GoSurfaceGeneration surface)
{
    kObj(GoSurfaceGeneration, surface); 

    GoSensor_SyncConfig(obj->sensor);

    return obj->fixedLength.length.min;
}

GoFx(kStatus) GoSurfaceGenerationVariableLength_SetMaxLength(GoSurfaceGeneration surface, k64f length)
{
    kObj(GoSurfaceGeneration, surface); 

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));
    kCheck(GoSensor_SyncConfig(obj->sensor));
    kCheckArgs(GoUtils_MinMax_(length, obj->variableLength.maxLength.min, obj->variableLength.maxLength.max));

    obj->variableLength.maxLength.value = length;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(k64f) GoSurfaceGenerationVariableLength_MaxLength(GoSurfaceGeneration surface)
{
    kObj(GoSurfaceGeneration, surface);

    GoSensor_SyncConfig(obj->sensor);

    return obj->variableLength.maxLength.value;
}

GoFx(k64f) GoSurfaceGenerationVariableLength_MaxLengthLimitMax(GoSurfaceGeneration surface)
{
    kObj(GoSurfaceGeneration, surface);

    GoSensor_SyncConfig(obj->sensor);

    return obj->variableLength.maxLength.max;
}

GoFx(k64f) GoSurfaceGenerationVariableLength_MaxLengthLimitMin(GoSurfaceGeneration surface)
{
    kObj(GoSurfaceGeneration, surface); 

    GoSensor_SyncConfig(obj->sensor);

    return obj->variableLength.maxLength.min;
}

GoFx(kStatus) GoSurfaceGenerationRotational_SetCircumference(GoSurfaceGeneration surface, k64f value)
{
    kObj(GoSurfaceGeneration, surface); 

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));
    kCheckArgs(GoUtils_MinMax_(value, obj->rotational.circumference.min, obj->rotational.circumference.max));
    obj->rotational.circumference.value = value;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(k64f) GoSurfaceGenerationRotational_Circumference(GoSurfaceGeneration surface)
{
    kObj(GoSurfaceGeneration, surface); 

    GoSensor_SyncConfig(obj->sensor);

    return obj->rotational.circumference.value;
}

GoFx(k64f) GoSurfaceGenerationRotational_CircumferenceLimitMax(GoSurfaceGeneration surface)
{
    kObj(GoSurfaceGeneration, surface); 

    GoSensor_SyncConfig(obj->sensor);

    return obj->rotational.circumference.max;
}

GoFx(k64f) GoSurfaceGenerationRotational_CircumferenceLimitMin(GoSurfaceGeneration surface)
{
    kObj(GoSurfaceGeneration, surface); 

    GoSensor_SyncConfig(obj->sensor);

    return obj->rotational.circumference.min;
}

GoFx(kStatus) GoSurfaceGenerationRotational_SetEncoderResolution(GoSurfaceGeneration surface, k64f value)
{
    kObj(GoSurfaceGeneration, surface); 

    // converting value from ticks/rev to deg/tick as it's stored in live.tfm
    GoTransform_SetEncoderResolution((GoTransform)GoSensor_Transform(obj->sensor), 360.0/value);

    return kOK;
}

GoFx(k64f) GoSurfaceGenerationRotational_EncoderResolution(GoSurfaceGeneration surface)
{
    kObj(GoSurfaceGeneration, surface); 

    // converting value from deg/tick as it's stored in live.tfm to ticks/rev
    return (360.0 / GoTransform_EncoderResolution((GoTransform)GoSensor_Transform(obj->sensor)));
}
/** 
 * @file    GoProfileGeneration.c
 *
 * @internal
 * Copyright (C) 2016-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <GoSdk/GoProfileGeneration.h>
#include <GoSdk/GoSensor.h>
#include <GoSdk/GoUtils.h>

kBeginClassEx(Go, GoProfileGeneration)
    kAddVMethod(GoProfileGeneration, kObject, VRelease)
kEndClassEx()

GoFx(kStatus) GoProfileGeneration_Construct(GoProfileGeneration* profile, kObject sensor, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback(allocator);
    kStatus status; 

    kCheck(kAlloc_GetObject(alloc, kTypeOf(GoProfileGeneration), profile)); 

    if (!kSuccess(status = GoProfileGeneration_Init(*profile, kTypeOf(GoProfileGeneration), sensor, alloc)))
    {
        kAlloc_FreeRef(alloc, profile); 
    }

    return status; 
} 

GoFx(kStatus) GoProfileGeneration_Init(GoProfileGeneration profile, kType type, kObject sensor, kAlloc alloc)
{
    kObjR(GoProfileGeneration, profile); 

    kCheck(kObject_Init(profile, type, alloc)); 
    kZero(obj->xml);
    kZero(obj->xmlItem);
    obj->type = GO_PROFILE_GENERATION_TYPE_CONTINUOUS;
    kZero(obj->fixedLength);
    kZero(obj->variableLength);
    kZero(obj->rotational);

    obj->sensor = sensor; 
    kCheck(kArrayList_Construct(&obj->fixedLength.externalInputIndexOptions, kTypeOf(k32s), 0, alloc));

    return kOK; 
}

GoFx(kStatus) GoProfileGeneration_VRelease(GoProfileGeneration profile)
{
    kObj(GoProfileGeneration, profile); 

    kCheck(kDisposeRef(&obj->fixedLength.externalInputIndexOptions));

    kCheck(kObject_VRelease(profile)); 

    return kOK; 
}

GoFx(kStatus) GoProfileGeneration_Read(GoProfileGeneration profile, kXml xml, kXmlItem item)
{
    kObj(GoProfileGeneration, profile); 
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

GoFx(kStatus) GoProfileGeneration_Write(GoProfileGeneration profile, kXml xml, kXmlItem item)
{
    kObj(GoProfileGeneration, profile); 
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


GoFx(kStatus) GoProfileGeneration_SetGenerationType(GoProfileGeneration profile, GoProfileGenerationType type)
{
    kObj(GoProfileGeneration, profile); 

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));
    obj->type = type;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(GoProfileGenerationType) GoProfileGeneration_GenerationType(GoProfileGeneration profile)
{
    kObj(GoProfileGeneration, profile); 

    return obj->type;
}

GoFx(kStatus) GoProfileGenerationFixedLength_SetStartTrigger(GoProfileGeneration profile, GoProfileGenerationStartTrigger trigger)
{
    kObj(GoProfileGeneration, profile); 

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));
    obj->fixedLength.startTrigger = trigger;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(GoProfileGenerationStartTrigger) GoProfileGenerationFixedLength_StartTrigger(GoProfileGeneration profile)
{
    kObj(GoProfileGeneration, profile); 

    GoSensor_SyncConfig(obj->sensor);

    return obj->fixedLength.startTrigger;
}

GoFx(kStatus) GoProfileGenerationFixedLength_SetTriggerExternalInputIndex(GoProfileGeneration surface, k32s index)
{
    kObj(GoProfileGeneration, surface);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->fixedLength.externalInputIndex = index;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(k32s) GoProfileGenerationFixedLength_TriggerExternalInputIndex(GoProfileGeneration surface)
{
    kObj(GoProfileGeneration, surface);

    GoSensor_SyncConfig(obj->sensor);

    return obj->fixedLength.externalInputIndex;
}

GoFx(kBool) GoProfileGenerationFixedLength_TriggerExternalInputIndexUsed(GoProfileGeneration surface)
{
    kObj(GoProfileGeneration, surface);

    GoSensor_SyncConfig(obj->sensor);

    return obj->fixedLength.externalInputIndexUsed;
}

GoFx(kSize) GoProfileGenerationFixedLength_TriggerExternalInputIndexOptionCount(GoProfileGeneration surface)
{
    kObj(GoProfileGeneration, surface);

    GoSensor_SyncConfig(obj->sensor);

    return kArrayList_Count(obj->fixedLength.externalInputIndexOptions);
}

GoFx(kStatus) GoProfileGenerationFixedLength_SetLength(GoProfileGeneration profile, k64f length)
{
    kObj(GoProfileGeneration, profile); 

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));
    kCheck(GoSensor_SyncConfig(obj->sensor));
    kCheckArgs(GoUtils_MinMax_(length, obj->fixedLength.length.min, obj->fixedLength.length.max));

    obj->fixedLength.length.value = length;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(k64f) GoProfileGenerationFixedLength_Length(GoProfileGeneration profile)
{
    kObj(GoProfileGeneration, profile); 

    GoSensor_SyncConfig(obj->sensor);

    return obj->fixedLength.length.value;
}

GoFx(k64f) GoProfileGenerationFixedLength_LengthLimitMax(GoProfileGeneration profile)
{
    kObj(GoProfileGeneration, profile); 

    GoSensor_SyncConfig(obj->sensor);

    return obj->fixedLength.length.max;
}

GoFx(k64f) GoProfileGenerationFixedLength_LengthLimitMin(GoProfileGeneration profile)
{
    kObj(GoProfileGeneration, profile); 

    GoSensor_SyncConfig(obj->sensor);

    return obj->fixedLength.length.min;
}

GoFx(kStatus) GoProfileGenerationVariableLength_SetMaxLength(GoProfileGeneration profile, k64f length)
{
    kObj(GoProfileGeneration, profile); 

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));
    kCheck(GoSensor_SyncConfig(obj->sensor));
    kCheckArgs(GoUtils_MinMax_(length, obj->variableLength.maxLength.min, obj->variableLength.maxLength.max));

    obj->variableLength.maxLength.value = length;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(k64f) GoProfileGenerationVariableLength_MaxLength(GoProfileGeneration profile)
{
    kObj(GoProfileGeneration, profile);

    GoSensor_SyncConfig(obj->sensor);

    return obj->variableLength.maxLength.value;
}

GoFx(k64f) GoProfileGenerationVariableLength_MaxLengthLimitMax(GoProfileGeneration profile)
{
    kObj(GoProfileGeneration, profile);

    GoSensor_SyncConfig(obj->sensor);

    return obj->variableLength.maxLength.max;
}

GoFx(k64f) GoProfileGenerationVariableLength_MaxLengthLimitMin(GoProfileGeneration profile)
{
    kObj(GoProfileGeneration, profile); 

    GoSensor_SyncConfig(obj->sensor);

    return obj->variableLength.maxLength.min;
}

GoFx(kStatus) GoProfileGenerationRotational_SetCircumference(GoProfileGeneration profile, k64f value)
{
    kObj(GoProfileGeneration, profile); 

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));
    kCheckArgs(GoUtils_MinMax_(value, obj->rotational.circumference.min, obj->rotational.circumference.max));
    obj->rotational.circumference.value = value;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(k64f) GoProfileGenerationRotational_Circumference(GoProfileGeneration profile)
{
    kObj(GoProfileGeneration, profile); 

    GoSensor_SyncConfig(obj->sensor);

    return obj->rotational.circumference.value;
}

GoFx(k64f) GoProfileGenerationRotational_CircumferenceLimitMax(GoProfileGeneration profile)
{
    kObj(GoProfileGeneration, profile); 

    GoSensor_SyncConfig(obj->sensor);

    return obj->rotational.circumference.max;
}

GoFx(k64f) GoProfileGenerationRotational_CircumferenceLimitMin(GoProfileGeneration profile)
{
    kObj(GoProfileGeneration, profile); 

    GoSensor_SyncConfig(obj->sensor);

    return obj->rotational.circumference.min;
}

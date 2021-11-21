/** 
 * @file    GoUtils.c
 *
 * @internal
 * Copyright (C) 2016-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <GoSdk/GoUtils.h>
#include <stdio.h>

GoFx(kStatus) GoOptionList_ParseHelper32s(const kChar* text, kSize length, k32s* value)
{
    if (length == 0)
    {
        return kERROR_PARAMETER;
    }

    kCheck(k32s_Parse(value, text));

    return kOK;
}

GoFx(kStatus) GoOptionList_ParseList32s(const kChar* text, kArrayList list)
{
    const kChar* readIt = text;
    const kChar* separator = kNULL;
    k32s option = 0;

    kCheck(kArrayList_Purge(list));
    kCheck(kArrayList_Allocate(list, kTypeOf(k32s), 0));

    while (!kIsNull(separator = strchr(readIt, ',')))
    {
        if (kSuccess(GoOptionList_ParseHelper32s(readIt, separator - readIt, &option)))
        {
            kCheck(kArrayList_AddT(list, &option));
        }
        readIt = separator + 1;
    }

    if (kSuccess(GoOptionList_ParseHelper32s(readIt, strlen(readIt), &option)))
    {
        kCheck(kArrayList_AddT(list, &option));
    }

    return kOK;
}

GoFx(kStatus) GoOptionList_Parse32s(const kChar* text, k32s* optionList, kSize capacity, kSize* count)
{
    kArrayList list = kNULL; 

    kTry
    {
        kTest(kArrayList_Construct(&list, kTypeOf(k32s), capacity, kNULL)); 
        kTest(GoOptionList_ParseList32s(text, list)); 

        *count = kMin_(capacity, kArrayList_Count(list)); 
        kMemCopy(optionList, kArrayList_DataT(list, k32s), (*count) * sizeof(k32s));
    }
    kFinally
    {
        kDestroyRef(&list); 
        kEndFinally();  
    }

    return kOK; 
}

GoFx(kStatus) GoOptionList_Format32s(const k32s* optionList, kSize count, kChar* text, kSize capacity)
{
    kText64 buffer; 
    kSize totalLength = 0; 
    kSize length = 0; 
    kSize i; 
    
    text[0] = 0; 

    for (i = 0; i < count; ++i)
    {
        kStrPrintf(buffer, kCountOf(buffer), "%s%d", (i != 0) ? "," : "", optionList[i]);
        length = kStrLength(buffer);
        if ((totalLength + length) < capacity)
        {
            kStrCopy(&text[totalLength], length + 1, buffer);
            totalLength += length; 
        }
        else
        {
            return kERROR_PARAMETER; 
        }
    }
    
    return kOK; 
}

GoFx(kStatus) GoOptionList_ParseHelper32u(const kChar* text, kSize length, k32u* value)
{
    if(length == 0)
    {
        return kERROR_PARAMETER;
    }
    
    kCheck(k32u_Parse(value, text));

    return kOK; 
}

GoFx(kStatus) GoOptionList_ParseList32u(const kChar* text, kArrayList list)
{
    const kChar* readIt = text; 
    const kChar* separator = kNULL; 
    k32u option = 0; 

    kCheck(kArrayList_Purge(list));
    kCheck(kArrayList_Allocate(list, kTypeOf(k32u), 0)); 

    while (!kIsNull(separator = strchr(readIt, ',')))
    {
        if (kSuccess(GoOptionList_ParseHelper32u(readIt, separator - readIt, &option)))
        {
            kCheck(kArrayList_AddT(list, &option)); 
        }
        readIt = separator + 1; 
    }

    if (kSuccess(GoOptionList_ParseHelper32u(readIt, strlen(readIt), &option)))
    {
        kCheck(kArrayList_AddT(list, &option)); 
    }

    return kOK;    
}

GoFx(kStatus) GoOptionList_Parse32u(const kChar* text, k32u* optionList, kSize capacity, kSize* count)
{
    kArrayList list = kNULL; 

    kTry
    {
        kTest(kArrayList_Construct(&list, kTypeOf(k32u), capacity, kNULL)); 
        kTest(GoOptionList_ParseList32u(text, list)); 

        *count = kMin_(capacity, kArrayList_Count(list)); 
        kMemCopy(optionList, kArrayList_DataT(list, k32u), (*count) * sizeof(k32u));
    }
    kFinally
    {
        kDestroyRef(&list); 
        kEndFinally();  
    }

    return kOK; 
}

GoFx(kStatus) GoOptionList_Format32u(const k32u* optionList, kSize count, kChar* text, kSize capacity)
{
    kText64 buffer; 
    kSize totalLength = 0; 
    kSize length = 0; 
    kSize i; 
    
    text[0] = 0; 

    for (i = 0; i < count; ++i)
    {
        kStrPrintf(buffer, kCountOf(buffer), "%s%u", (i != 0) ? "," : "", optionList[i]);
        length = kStrLength(buffer);
        if ((totalLength + length) < capacity)
        {
            kStrCopy(&text[totalLength], length + 1, buffer);
            totalLength += length; 
        }
        else
        {
            return kERROR_PARAMETER; 
        }
    }
    
    return kOK; 
}

GoFx(kStatus) GoOptionList_Format32uString(const k32u* optionList, kSize count, kString str)
{
    kSize i; 
    
    kCheck(kString_Clear(str));

    for (i = 0; i < count; ++i)
    {
        kCheck(kString_Addf(str, "%s%u", (i != 0) ? "," : "", optionList[i]));
    }
    
    return kOK; 
}

GoFx(kStatus) GoOptionList_FormatOutputCompositeSource(const GoOutputCompositeSource* optionList, kSize count, kChar* text, kSize capacity)
{
    kText64 buffer;
    kSize totalLength = 0;
    kSize length = 0;
    kSize i;

    text[0] = 0;

    for (i = 0; i < count; ++i)
    {
        kStrPrintf(buffer, kCountOf(buffer), "%s%u-%u", (i != 0) ? "," : "", optionList[i].id, optionList[i].dataSource);
        length = kStrLength(buffer);
        if ((totalLength + length) < capacity)
        {
            kStrCopy(&text[totalLength], length + 1, buffer);
            totalLength += length;
        }
        else
        {
            return kERROR_PARAMETER;
        }
    }

    return kOK;
}

GoFx(kBool) GoOptionList_Check32u(const k32u* optionList, kSize count, k32u value)
{
    kSize i; 
    for (i = 0; i < count; ++i)
    {
        if (optionList[i] == value)
        {
            return kTRUE;
        }
    }
    
    return kFALSE; 
}


GoFx(kBool) GoOptionList_Check32s(const k32s* optionList, kSize count, k32s value)
{
    kSize i;
    for (i = 0; i < count; ++i)
    {
        if (optionList[i] == value)
        {
            return kTRUE;
        }
    }

    return kFALSE;
}

GoFx(kBool) GoOptionList_CheckSize(const k32u* optionList, kSize count, kSize value)
{
    kSize i; 
    for (i = 0; i < count; ++i)
    {
        if (optionList[i] == value)
        {
            return kTRUE;
        }
    }

    return kFALSE; 
}

GoFx(kStatus) GoOptionList_ParseHelper64f(const kChar* text, kSize length, k64f* value)
{
    if (length == 0)
    {
        return kERROR_PARAMETER;
    }

    kCheck(k64f_Parse(value, text));

    return kOK; 
}

GoFx(kStatus) GoOptionList_ParseList64f(const kChar* text, kArrayList list)
{
    const kChar* readIt = text; 
    const kChar* separator = kNULL; 
    k64f option = 0; 

    kCheck(kArrayList_Purge(list));
    kCheck(kArrayList_Allocate(list, kTypeOf(k64f), 0)); 

    while (!kIsNull(separator = strchr(readIt, ',')))
    {
        if (kSuccess(GoOptionList_ParseHelper64f(readIt, separator - readIt, &option)))
        {
            kCheck(kArrayList_AddT(list, &option)); 
        }
        readIt = separator + 1; 
    }

    if (kSuccess(GoOptionList_ParseHelper64f(readIt, kStrLength(readIt), &option)))
    {
        kCheck(kArrayList_AddT(list, &option)); 
    }

    return kOK;    
}

GoFx(kStatus) GoOptionList_Parse64f(const kChar* text, k64f* optionList, kSize capacity, kSize* count)
{
    kArrayList list = kNULL; 

    kTry
    {
        kTest(kArrayList_Construct(&list, kTypeOf(k64f), capacity, kNULL)); 
        kTest(GoOptionList_ParseList64f(text, list)); 

        *count = kMin_(capacity, kArrayList_Count(list)); 
        kMemCopy(optionList, kArrayList_DataT(list, k64f), (*count) * sizeof(k64f));
    }
    kFinally
    {
        kDestroyRef(&list); 
        kEndFinally();  
    }

    return kOK; 
}

GoFx(kStatus) GoOptionList_Format64f(const k64f* optionList, kSize count, kChar* text, kSize capacity)
{
    kText64 buffer; 
    kSize totalLength = 0; 
    kSize length = 0; 
    kSize i; 

    text[0] = 0; 

    for (i = 0; i < count; ++i)
    {
        kStrPrintf(buffer, kCountOf(buffer), "%s%lf", (i != 0) ? "," : "", optionList[i]);
        length = kStrLength(buffer);
        if ((totalLength + length) < capacity)
        {
            kStrCopy(&text[totalLength], length + 1, buffer);
            totalLength += length; 
        }
        else
        {
            return kERROR_PARAMETER; 
        }
    }

    return kOK; 
}

GoFx(kBool) GoOptionList_Check64f(const k64f* optionList, kSize count, k64f value)
{
    kSize i; 
    for (i = 0; i < count; ++i)
    {
        if (optionList[i] == value)
        {
            return kTRUE;
        }
    }

    return kFALSE; 
}

GoFx(kStatus) GoUtils_ParseHelperDataStreamId(const kChar* text, kSize length, GoDataStreamId* value)
{
    const kChar* readIt = text;
    const kChar* separator = kNULL;
    k32s val[3];
    k32u count = 0;

    if (length == 0)
    {
        return kERROR_PARAMETER;
    }

    if (length == 2)    // check if special case of single "-1"
    {
        if (kSuccess(GoOptionList_ParseHelper32s(text, length, &val[0])))
        {
            if (val[0] == GO_DATA_STEP_NONE)
            {
                value->step = value->id = value->source = GO_DATA_STEP_NONE;
                return kOK;
            }
        }
            
        return kERROR_CONFLICT;
    }

    while (!kIsNull(separator = strchr(readIt, GO_UTILS_STREAM_ID_SEPARATOR)) && count < 2)
    {
        if (kSuccess(GoOptionList_ParseHelper32s(readIt, separator - readIt, &val[count])))
        {
            count++;
        }
            
        readIt = separator + 1;
    }

    if (kSuccess(GoOptionList_ParseHelper32s(readIt, separator - readIt, &val[count])))
    {
        count++;
    }

    if (count != 3)
    {
        return kERROR_CONFLICT;
    }
    else
    {
        value->step = val[0];
        value->id = val[1];
        value->source = val[2];
    }

    return kOK; 
}

GoFx(kBool) GoUtils_FuzzyEquivalence(k64f first, k64f second, k8u decimalPrecision)
{    
    k64f precision = pow(0.1, decimalPrecision);

    if (fabs(first - second) < precision)
    {
        return kTRUE;
    }

    return kFALSE;
}

GoFx(kStatus) GoUtils_XmlMerge(kXml srcXml, kXmlItem srcItem, kXml dstXml, kXmlItem dstItem)
{
    kXmlItem temp = kNULL;
    kXmlItem child = kNULL;
    kSize i;
    kText256 val;

    if(kIsNull(srcXml) || kIsNull(srcItem) || kIsNull(dstItem))
    {
        return kOK;
    }

    //merge attributes
    for(i = 0; i < kXml_AttrCount(srcXml, srcItem); i++)
    {
        const kChar* name = kXml_AttrNameAt(srcXml, srcItem, i);

        if(!kXml_AttrExists(dstXml, dstItem, name))
        {
            kCheck(kXml_AttrText(srcXml, srcItem, name, val, kCountOf(val)));
            kCheck(kXml_SetAttrText(dstXml, dstItem, name, val));
        }
    }
    
    //merge children
    child = kXml_FirstChild(srcXml, srcItem);
    while(!kIsNull(child))
    {
        if(kIsNull(kXml_Child(dstXml, dstItem, kXml_ItemName(srcXml, child))))
        {
            kCheck(kXml_CopyItem(dstXml, dstItem, kNULL, srcXml, child, &temp));
        }
        child = kXml_NextSibling(srcXml, child);
    }
    
    return kOK;
}


GoFx(kStatus) GoConfig_WriteRangeElement64f( kXml xml, kXmlItem item, const kChar* elementName, GoElement64f element )
{
    kXmlItem newItem = kNULL;

    kCheck(kXml_AddItem(xml, item, elementName, &newItem));
    kCheck(kXml_SetItem64f(xml, newItem, element.value));
    kCheck(kXml_SetAttr64f(xml, newItem, "min", element.min));
    kCheck(kXml_SetAttr64f(xml, newItem, "max", element.max));

    if (element.systemValue != k64F_NULL)
    {
        kCheck(kXml_SetAttr64f(xml, newItem, "value", element.systemValue));
    }

    return kOK;
}

GoFx(kStatus) GoConfig_WriteRangeElement32u(kXml xml, kXmlItem item, const kChar* elementName, GoElement32u element)
{
    kXmlItem newItem = kNULL;

    kCheck(kXml_AddItem(xml, item, elementName, &newItem));
    kCheck(kXml_SetItem32u(xml, newItem, element.value));
    kCheck(kXml_SetAttr32u(xml, newItem, "min", element.min));
    kCheck(kXml_SetAttr32u(xml, newItem, "max", element.max));

    if (element.systemValue != k32U_NULL)
    {
        kCheck(kXml_SetAttr32u(xml, newItem, "value", element.systemValue));
    }

    return kOK;
}

GoFx(kStatus) GoConfig_ReadRangeElement64f( kXml xml, kXmlItem item, const kChar* elementName, GoElement64f* element )
{
    kXmlItem newItem = kNULL;

    newItem = kXml_Child(xml, item, elementName);
    if (!kIsNull(newItem))
    {
        kCheck(kXml_Item64f(xml, newItem, &element->value));
        kCheck(kXml_Attr64f(xml, newItem, "min", &element->min));
        kCheck(kXml_Attr64f(xml, newItem, "max", &element->max));
    }

    // Set the system value even if the XML node does not exist.
    if (!kIsNull(newItem) && kXml_AttrExists(xml, newItem, "value"))
    {
        kCheck(kXml_Attr64f(xml, newItem, "value", &element->systemValue));
    }
    else
    {
        element->systemValue = k64F_NULL;
    }

    return kOK;
}

GoFx(kStatus) GoConfig_ReadRangeElement32u(kXml xml, kXmlItem item, const kChar* elementName, GoElement32u* element)
{
    kXmlItem newItem = kNULL;

    newItem = kXml_Child(xml, item, elementName);
    if (!kIsNull(newItem))
    {
        kCheck(kXml_Item32u(xml, newItem, &element->value));
        kCheck(kXml_Attr32u(xml, newItem, "min", &element->min));
        kCheck(kXml_Attr32u(xml, newItem, "max", &element->max));
    }

    // Set the system value even if the XML node does not exist.
    if (!kIsNull(newItem) && kXml_AttrExists(xml, newItem, "value"))
    {
        kCheck(kXml_Attr32u(xml, newItem, "value", &element->systemValue));
    }
    else
    {
        element->systemValue = k32U_NULL;
    }

    return kOK;
}

GoFx(kStatus) GoConfig_ReadChildOptional(kXml xml, kXmlItem item, const kChar* name, kObject object, GoReadFunction func)
{
    if (kXml_ChildExists(xml, item, name))
    {
        kXmlItem tempItem = kXml_Child(xml, item, name);

        kCheck(func(object, xml, tempItem));
    }
    
    return kOK;
}

GoFx(kStatus) GoConfig_Read16sOptional(kXml xml, kXmlItem item, const kChar* name, k16s defaultVal, k16s* value)
{
    if (kXml_ChildExists(xml, item, name))
    {
        kCheck(kXml_Child16s(xml, item, name, value));
    }
    else
    {
        *value = defaultVal;
    }

    return kOK;
}

GoFx(kStatus) GoConfig_Read16uOptional(kXml xml, kXmlItem item, const kChar* name, k16u defaultVal, k16u* value)
{
    if (kXml_ChildExists(xml, item, name))
    {
        kCheck(kXml_Child16u(xml, item, name, value));
    }
    else
    {
        *value = defaultVal;
    }

    return kOK;
}


GoFx(kStatus) GoConfig_Read32sOptional(kXml xml, kXmlItem item, const kChar* name, k32s defaultVal, k32s* value)
{
    if (kXml_ChildExists(xml, item, name))
    {
        kCheck(kXml_Child32s(xml, item, name, value));
    }
    else
    {
        *value = defaultVal;
    }

    return kOK;
}

GoFx(kStatus) GoConfig_Read32uOptional(kXml xml, kXmlItem item, const kChar* name, k32u defaultVal, k32u* value)
{
    if (kXml_ChildExists(xml, item, name))
    {
        kCheck(kXml_Child32u(xml, item, name, value));
    }
    else
    {
        *value = defaultVal;
    }

    return kOK;
}

GoFx(kStatus) GoConfig_ReadRangeElement32uOptional(kXml xml, kXmlItem item, const kChar* elementName, GoElement32u* element)
{
    if (kXml_ChildExists(xml, item, elementName))
    {
        kCheck(GoConfig_ReadRangeElement32u(xml, item, elementName, element));
    }

    return kOK;
}

GoFx(kStatus) GoConfig_Read32fOptional(kXml xml, kXmlItem item, const kChar* name, k32f defaultVal, k32f* value)
{
    if (kXml_ChildExists(xml, item, name))
    {
        kCheck(kXml_Child32f(xml, item, name, value));
    }
    else
    {
        *value = defaultVal;
    }

    return kOK;
}

GoFx(kStatus) GoConfig_Read64sOptional(kXml xml, kXmlItem item, const kChar* name, k64s defaultVal, k64s* value)
{
    if (kXml_ChildExists(xml, item, name))
    {
        kCheck(kXml_Child64s(xml, item, name, value));
    }
    else
    {
        *value = defaultVal;
    }

    return kOK;
}

GoFx(kStatus) GoConfig_Read64uOptional(kXml xml, kXmlItem item, const kChar* name, k64u defaultVal, k64u* value)
{
    if (kXml_ChildExists(xml, item, name))
    {
        kCheck(kXml_Child64u(xml, item, name, value));
    }
    else
    {
        *value = defaultVal;
    }

    return kOK;
}

GoFx(kStatus) GoConfig_Read64fOptional(kXml xml, kXmlItem item, const kChar* name, k64f defaultVal, k64f* value)
{
    if (kXml_ChildExists(xml, item, name))
    {
        kCheck(kXml_Child64f(xml, item, name, value));
    }
    else
    {
        *value = defaultVal;
    }

    return kOK;
}

GoFx(kStatus) GoConfig_ReadRangeElement64fOptional(kXml xml, kXmlItem item, const kChar* elementName, GoElement64f* element)
{
    if (kXml_ChildExists(xml, item, elementName))
    {
        kCheck(GoConfig_ReadRangeElement64f(xml, item, elementName, element));
    }
    
    return kOK;
}

GoFx(kStatus) GoConfig_ReadAttrBoolOptional(kXml xml, kXmlItem item, const kChar* name, kBool defaultVal, kBool* value)
{
    if (kXml_AttrExists(xml, item, name))
    {
        kCheck(kXml_AttrBool(xml, item, name, value));
    }
    else
    {
        *value = defaultVal;
    }

    return kOK;
}

GoFx(kStatus) GoConfig_ReadBoolOptional(kXml xml, kXmlItem item, const kChar* name, kBool defaultVal, kBool* value)
{
    if (kXml_ChildExists(xml, item, name))
    {
        kCheck(kXml_ChildBool(xml, item, name, value));
    }
    else
    {
        *value = defaultVal;
    }

    return kOK;
}

GoFx(kStatus) GoConfig_ReadStringOptional(kXml xml, kXmlItem item, const kChar* name, const kChar* defaultVal, kString value)
{
    if (kXml_ChildExists(xml, item, name))
    {
        kCheck(kXml_ChildString(xml, item, name, value));
    }
    else
    {
        kCheck(kString_Set(value, defaultVal));
    }

    return kOK;
}

GoFx(kStatus) GoConfig_ReadSizeOptional(kXml xml, kXmlItem item, const kChar* name, kSize defaultVal, kSize* value)
{
    if (kXml_ChildExists(xml, item, name))
    {
        kCheck(kXml_ChildSize(xml, item, name, value));
    }
    else
    {
        *value = defaultVal;
    }

    return kOK;
}

GoFx(kBool) GoOptionList_CheckDataStream(const GoDataStream* optionList, kSize count, GoDataStream value)
{
    kSize i;
    for (i = 0; i < count; ++i)
    {
        if (optionList[i].step == value.step && optionList[i].id == value.id)
        {
            return kTRUE;
        }
    }

    return kFALSE;
}

/** 
 * @file    GoSections.c
 *
 * @internal
 * Copyright (C) 2016-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <GoSdk/GoSections.h>
#include <GoSdk/GoSensor.h>

kBeginClassEx(Go, GoSections)
    kAddVMethod(GoSections, kObject, VRelease)
kEndClassEx()

GoFx(kStatus) GoSections_Construct(GoSections* detection, kObject sensor, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback(allocator);
    kStatus status; 

    kCheck(kAlloc_GetObject(alloc, kTypeOf(GoSections), detection)); 

    if (!kSuccess(status = GoSections_Init(*detection, kTypeOf(GoSections), sensor, alloc)))
    {
        kAlloc_FreeRef(alloc, detection); 
    }

    return status; 
} 

GoFx(kStatus) GoSections_Init(GoSections detection, kType type, kObject sensor, kAlloc alloc)
{
    kObjR(GoSections, detection); 
    kStatus exception = kOK;

    kCheck(kObject_Init(detection, type, alloc)); 
    kZero(obj->xml);
    kZero(obj->xmlItem);
    obj->xMin = 0.0;
    obj->xMax = 0.0;
    obj->yMin = 0.0;
    obj->yMax = 0.0;
    kZero(obj->sections);

    obj->sensor = sensor;
    kCheck(kArrayList_Construct(&obj->sections, kTypeOf(GoSection), 0, alloc));

    return kOK; 
}

GoFx(kStatus) GoSections_VRelease(GoSections detection)
{
    kObj(GoSections, detection); 
    
    kCheck(kDisposeRef(&obj->sections));

    return kObject_VRelease(detection); 
}

GoFx(kStatus) GoSections_Read(GoSections detection, kXml xml, kXmlItem item)
{
    kObj(GoSections, detection);
    kXmlItem tempItem = kNULL;
    kSize i;
    GoSection section = kNULL;

    obj->xml = xml;
    obj->xmlItem = item;

    kCheck(kXml_Attr64f(xml, item, "xMin", &obj->xMin));
    kCheck(kXml_Attr64f(xml, item, "xMax", &obj->xMax));
    kCheck(kXml_Attr64f(xml, item, "yMin", &obj->yMin));
    kCheck(kXml_Attr64f(xml, item, "yMax", &obj->yMax));

    for (i = 0; i < kXml_ChildCount(xml, item); i++)
    {
        kBool addToList = kFALSE;

        section = kNULL;
        tempItem = kXml_ChildAt(xml, item, i);

        if (i >= kArrayList_Count(obj->sections))
        {
            k16s temp16s;

            kCheck(kXml_Attr16s(xml, tempItem, "id", &temp16s));
            kCheck(GoSection_Construct(&section, obj->sensor, temp16s, kObject_Alloc(detection)));
            addToList = kTRUE;
        }
        else
        {
            section = *kArrayList_AtT(obj->sections, i, GoSection);
        }

        kCheck(GoSection_Read(section, xml, tempItem));

        if (addToList)
        {
            kCheck(kArrayList_AddT(obj->sections, &section));
        }
    }

    while (kXml_ChildCount(xml, item) < kArrayList_Count(obj->sections))
    {
        kCheck(kArrayList_RemoveT(obj->sections, kArrayList_Count(obj->sections) - 1, &section));
        kDisposeRef(&section);
    }
    
    return kOK;
}

GoFx(kStatus) GoSections_Write(GoSections matching, kXml xml, kXmlItem item)
{
    kObj(GoSections, matching); 
    kXmlItem tempItem = kNULL;
    kSize i;

    for (i = 0; i < kArrayList_Count(obj->sections); i++)
    {
        GoSection section = *kArrayList_AtT(obj->sections, i, GoSection);

        kCheck(kXml_AddItem(xml, item, "Section", &tempItem));
        kCheck(GoSection_Write(section, xml, tempItem));
    }
    
    //No forwards compatibility due to dynamic nature of the section list

    return kOK; 
}

GoFx(k64f) GoSections_XLimitMin(GoSections sections)
{
    kObj(GoSections, sections);

    GoSensor_SyncConfig(obj->sensor);

    return obj->xMin;
}

GoFx(k64f) GoSections_XLimitMax(GoSections sections)
{
    kObj(GoSections, sections);

    GoSensor_SyncConfig(obj->sensor);

    return obj->xMax;
}

GoFx(k64f) GoSections_YLimitMin(GoSections sections)
{
    kObj(GoSections, sections);

    GoSensor_SyncConfig(obj->sensor);

    return obj->yMin;
}

GoFx(k64f) GoSections_YLimitMax(GoSections sections)
{
    kObj(GoSections, sections);

    GoSensor_SyncConfig(obj->sensor);

    return obj->yMax;
}

GoFx(kSize) GoSections_SectionCount(GoSections sections)
{
    kObj(GoSections, sections);

    GoSensor_SyncConfig(obj->sensor);

    return kArrayList_Count(obj->sections);
}

GoFx(GoSection) GoSections_SectionAt(GoSections sections, kSize index)
{
    kObj(GoSections, sections);

    GoSensor_SyncConfig(obj->sensor);
    kAssert(index < kArrayList_Count(obj->sections));

    return *kArrayList_AtT(obj->sections, index, GoSection);
}

GoFx(kStatus) GoSections_AddSection(GoSections sections, GoSection* addedSection)
{
    kObj(GoSections, sections);
    GoSection newSection = kNULL;
    kSize i;
    k16s largestId = 0;
    kStatus exception = kOK;

    if (kSuccess(GoSensor_ReadInfo(obj->sensor)) && GoSensor_IsConfigurable(obj->sensor))
    {
        GoSensor_CacheConfig(obj->sensor);
    }

    for (i = 0; i < kArrayList_Count(obj->sections); i++)
    {
        if (GoSection_Id(GoSections_SectionAt(sections, i)) > largestId)
        {
            largestId = GoSection_Id(GoSections_SectionAt(sections, i));
        }
        //add one to get the next id
        largestId++;
    }

    kTry
    {
        kTest(GoSection_Construct(&newSection, obj->sensor, largestId , kObject_Alloc(sections)));
        kTest(kArrayList_AddT(obj->sections, &newSection));

        kTest(GoSensor_SetConfigModified(obj->sensor));
        kTest(GoSensor_SyncConfig(obj->sensor));
    }
    kCatch(&exception)
    {
        kDestroyRef(&newSection);
        kEndCatch(exception);
    }

    if (!kIsNull(addedSection))
    {
        *addedSection = newSection;
    }    
    
    return kOK;
}

GoFx(kStatus) GoSections_RemoveSection(GoSections sections, kSize index)
{
    kObj(GoSections, sections);
    GoSection sectionToRemove = kNULL;

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    kCheckArgs(index < kArrayList_Count(obj->sections));

    kCheck(kArrayList_RemoveT(obj->sections, index, &sectionToRemove));
    kCheck(kDestroyRef(&sectionToRemove));

    kCheck(GoSensor_SetConfigModified(obj->sensor));
    kCheck(GoSensor_SyncConfig(obj->sensor));

    return kOK;
}

GoFx(kStatus) GoSections_Clear(GoSections sections)
{
    kObj(GoSections, sections);

    kCheck(kArrayList_Purge(obj->sections));

    kCheck(GoSensor_SetConfigModified(obj->sensor));
    kCheck(GoSensor_SyncConfig(obj->sensor));

    return kOK;
}

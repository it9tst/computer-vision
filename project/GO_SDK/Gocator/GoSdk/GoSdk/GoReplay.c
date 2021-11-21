/**
* @file    GoReplay.c
*
* @internal
* Copyright (C) 2016-2021 by LMI Technologies Inc.
* Licensed under the MIT License.
* Redistributed files must retain the above copyright notice.
*/
#include <GoSdk/GoReplay.h>
#include <GoSdk/GoSensor.h>
#include <GoSdk/GoUtils.h>

kBeginClassEx(Go, GoReplay)
    kAddVMethod(GoReplay, kObject, VRelease)
kEndClassEx()

GoFx(kStatus) GoReplay_Construct(GoReplay* replay, kObject sensor, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback(allocator);
    kStatus status;

    kCheck(kAlloc_GetObject(alloc, kTypeOf(GoReplay), replay));

    if (!kSuccess(status = GoReplay_Init(*replay, kTypeOf(GoReplay), sensor, alloc)))
    {
        kAlloc_FreeRef(alloc, replay);
    }

    return status;
}

GoFx(kStatus) GoReplay_Init(GoReplay replay, kType type, kObject sensor, kAlloc alloc)
{
    kObjR(GoReplay, replay);

    kCheck(kObject_Init(replay, type, alloc));

    obj->sensor = sensor;
    obj->xml = obj->xmlItem = obj->recordingFilter = kNULL;

    kCheck(GoRecordingFilter_Construct(&obj->recordingFilter, sensor, alloc));

    return kOK;
}

GoFx(kStatus) GoReplay_VRelease(GoReplay replay)
{
    kObj(GoReplay, replay);

    kCheck(kDisposeRef(&obj->recordingFilter));
    
    kCheck(kObject_VRelease(replay));

    return kOK;
}

GoFx(kStatus) GoReplay_Read(GoReplay replay, kXml xml, kXmlItem item)
{
    kObj(GoReplay, replay);

    obj->xml = xml;
    obj->xmlItem = item;

    if (kXml_ChildExists(xml, item, "RecordingFilter"))
    {
        kCheck(GoRecordingFilter_Read(obj->recordingFilter, xml, kXml_Child(xml, item, "RecordingFilter")));
    }
    
    return kOK;
}

GoFx(kStatus) GoReplay_Write(GoReplay replay, kXml xml, kXmlItem item)
{
    kObj(GoReplay, replay);
    kXmlItem filterItem = kNULL;

    if (kIsNull(obj->xml) || kIsNull(obj->xmlItem)) return kERROR_STATE;

    kCheck(kXml_AddItem(xml, item, "RecordingFilter", &filterItem));
    kCheck(GoRecordingFilter_Write(obj->recordingFilter, xml, filterItem));

    //Forwards Compatibility
    kCheck(GoUtils_XmlMerge(obj->xml, obj->xmlItem, xml, item));

    return kOK;
}

GoFx(GoRecordingFilter) GoReplay_RecordingFilter(GoReplay replay)
{
    kObj(GoReplay, replay);

    return obj->recordingFilter;
}

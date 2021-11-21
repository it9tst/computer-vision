/**
* @file    GoRecordingFilter.x.h
*
* @internal
* Copyright (C) 2016-2021 by LMI Technologies Inc.
* Licensed under the MIT License.
* Redistributed files must retain the above copyright notice.
*/
#ifndef GO_RECORDING_FILTER_X_H
#define GO_RECORDING_FILTER_X_H

#include <GoSdk/GoReplayCondition.h>

typedef struct GoRecordingFilterClass
{
    kObjectClass base;
    kObject sensor;

    GoReplayCombineType conditionCombineType;

    // Conditions
    GoReplayAnyMeasurement anyMeasurement;
    GoReplayAnyData anyData;
    GoReplayMeasurement measurement;

} GoRecordingFilterClass;

kDeclareClassEx(Go, GoRecordingFilter, kObject)

GoFx(kStatus) GoRecordingFilter_Construct(GoRecordingFilter* filter, kObject sensor, kAlloc allocator);

GoFx(kStatus) GoRecordingFilter_Init(GoRecordingFilter filter, kType type, kObject sensor, kAlloc alloc);
GoFx(kStatus) GoRecordingFilter_VRelease(GoRecordingFilter filter);

GoFx(kStatus) GoRecordingFilter_Read(GoRecordingFilter filter, kXml xml, kXmlItem item);
GoFx(kStatus) GoRecordingFilter_Write(GoRecordingFilter filter, kXml xml, kXmlItem item);

#endif

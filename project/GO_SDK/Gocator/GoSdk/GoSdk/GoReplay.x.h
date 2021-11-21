/**
* @file    GoReplay.x.h
*
* @internal
* Copyright (C) 2016-2021 by LMI Technologies Inc.
* Licensed under the MIT License.
* Redistributed files must retain the above copyright notice.
*/
#ifndef GO_REPLAY_X_H
#define GO_REPLAY_X_H

#include <GoSdk/GoRecordingFilter.h>

typedef struct GoReplayClass
{
    kObjectClass base;
    kObject sensor;

    kXml xml;
    kXmlItem xmlItem;

    GoRecordingFilter recordingFilter;

} GoReplayClass;

kDeclareClassEx(Go, GoReplay, kObject)

GoFx(kStatus) GoReplay_Construct(GoReplay* replay, kObject sensor, kAlloc allocator);

GoFx(kStatus) GoReplay_Init(GoReplay replay, kType type, kObject sensor, kAlloc alloc);
GoFx(kStatus) GoReplay_VRelease(GoReplay replay);

GoFx(kStatus) GoReplay_Read(GoReplay replay, kXml xml, kXmlItem item);
GoFx(kStatus) GoReplay_Write(GoReplay replay, kXml xml, kXmlItem item);

#endif

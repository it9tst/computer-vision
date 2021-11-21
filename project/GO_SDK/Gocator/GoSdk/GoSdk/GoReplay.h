
/**
* @file    GoReplay.h
* @brief   Declares the GoReplay class.
*
* @internal
* Copyright (C) 2016-2021 by LMI Technologies Inc.
* Licensed under the MIT License.
* Redistributed files must retain the above copyright notice.
*/
#ifndef GO_REPLAY_H
#define GO_REPLAY_H

#include <GoSdk/GoRecordingFilter.h>

/**
* @class   GoReplay
* @extends kObject
* @ingroup GoSdk-Replay
* @brief   Represents a replay configuration.
*/
typedef kObject GoReplay;

/**
* Gets the recording filter for the replay.
*
* @public                  @memberof GoReplay
 * @version                Introduced in firmware 4.5.3.57
* @param   replay          GoReplay object.
* @return                  The recording filter.
*/
GoFx(GoRecordingFilter) GoReplay_RecordingFilter(GoReplay replay);

#include <GoSdk/GoReplay.x.h>

#endif

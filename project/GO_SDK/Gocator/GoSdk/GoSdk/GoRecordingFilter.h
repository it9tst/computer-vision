
/**
* @file    GoRecordingFilter.h
* @brief   Declares the GoRecordingFilter class.
*
* @internal
* Copyright (C) 2016-2021 by LMI Technologies Inc.
* Licensed under the MIT License.
* Redistributed files must retain the above copyright notice.
*/
#ifndef GO_RECORDING_FILTER_H
#define GO_RECORDING_FILTER_H

#include <GoSdk/GoReplayCondition.h>

/**
* @class   GoRecordingFilter
* @extends kObject
* @ingroup GoSdk-Replay
* @brief   Represents a recording filter configuration.
*/
typedef kObject GoRecordingFilter;

/**
* Gets the condition combine type.
*
* @public                  @memberof GoRecordingFilter
* @version                 Introduced in firmware 4.5.3.57
* @param   filter          GoRecordingFilter object.
* @return                  The condition combine type.
*/
GoFx(GoReplayCombineType) GoRecordingFilter_ConditionCombineType(GoRecordingFilter filter);

/**
* Sets the condition combine type.
*
* @public                           @memberof GoRecordingFilter
* @version                          Introduced in firmware 4.5.3.57
* @param   filter                   GoRecordingFilter object.
* @param   conditionCombineType     GoReplayCombineType object.
* @return                           Operation status.
*/
GoFx(kStatus) GoRecordingFilter_SetConditionCombineType(GoRecordingFilter filter, GoReplayCombineType conditionCombineType);

/**
* Gets the Any Measurement condition.
*
* @public                  @memberof GoRecordingFilter
* @version                 Introduced in firmware 4.5.3.57
* @param   filter          GoRecordingFilter object.
* @return                  The Any Measurement condition.
*/
GoFx(GoReplayAnyMeasurement) GoRecordingFilter_AnyMeasurement(GoRecordingFilter filter);

/**
* Gets the Any Data condition.
*
* @public                  @memberof GoRecordingFilter
* @version                 Introduced in firmware 4.5.3.57
* @param   filter          GoRecordingFilter object.
* @return                  The Any Data condition.
*/
GoFx(GoReplayAnyData) GoRecordingFilter_AnyData(GoRecordingFilter filter);

/**
* Gets the Measurement condition.
*
* @public                  @memberof GoRecordingFilter
* @version                 Introduced in firmware 4.5.3.57
* @param   filter          GoRecordingFilter object.
* @return                  The Measurement condition.
*/
GoFx(GoReplayMeasurement) GoRecordingFilter_Measurement(GoRecordingFilter filter);

#include <GoSdk/GoRecordingFilter.x.h>

#endif

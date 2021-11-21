
/**
* @file    GoReplayCondition.h
* @brief   Declares the GoReplayCondition classes.
*
* @internal
* Copyright (C) 2016-2021 by LMI Technologies Inc.
* Licensed under the MIT License.
* Redistributed files must retain the above copyright notice.
*/
#ifndef GO_REPLAY_CONDITION_H
#define GO_REPLAY_CONDITION_H

#include <GoSdk/GoSdkDef.h>

/**
* @class   GoReplayCondition
* @extends kObject
* @ingroup GoSdk-Replay
* @brief   Represents the base replay condition class.
*/
typedef kObject GoReplayCondition;

/**
* Gets the enabled state of a replay condition.
*
* @public                   @memberof GoReplayCondition
* @version                  Introduced in firmware 4.5.3.57
* @param   condition        GoReplayCondition object.
* @return                   kTRUE if enabled and kFALSE if disabled.
*/
GoFx(kBool) GoReplayCondition_Enabled(GoReplayCondition condition);

/**
* Sets the enabled state of a replay condition.
*
* @public                   @memberof GoReplayCondition
* @version                  Introduced in firmware 4.5.3.57
* @param   condition        GoReplayCondition object.
* @param   enable           kTRUE to enable the replay condition and kFALSE to disable it.
* @return                   Operation status.
*/
GoFx(kStatus) GoReplayCondition_Enable(GoReplayCondition condition, kBool enable);

/**
* @class   GoReplayAnyMeasurement
* @extends GoReplayCondition
* @ingroup GoSdk-Replay
* @brief   Represents the Any Measurement condition.
*/
typedef GoReplayCondition GoReplayAnyMeasurement;

/**
* Sets the required measurement result.
*
* @public                  @memberof GoReplayAnyMeasurement
* @version                 Introduced in firmware 4.5.3.57
* @param   condition       GoReplayAnyMeasurement object.
* @param   result          The required measurement result.
* @return                  Operation status.
*/
GoFx(kStatus) GoReplayAnyMeasurement_SetResult(GoReplayAnyMeasurement condition, GoReplayMeasurementResult result);

/**
* Gets the required measurement result.
*
* @public                  @memberof GoReplayAnyMeasurement
* @version                 Introduced in firmware 4.5.3.57
* @param   condition       GoReplayAnyMeasurement object.
* @return                  The required measurement result.
*/
GoFx(GoReplayMeasurementResult) GoReplayAnyMeasurement_Result(GoReplayAnyMeasurement condition);

/**
* @class   GoReplayAnyData
* @extends GoReplayCondition
* @ingroup GoSdk-Replay
* @brief   Represents the Any Data condition.
*/
typedef GoReplayCondition GoReplayAnyData;

/**
* Sets the required range count case.
*
* @public                  @memberof GoReplayAnyData
* @version                 Introduced in firmware 4.5.3.57
* @param   condition       GoReplayAnyData object.
* @param   rangeCase       The required range count case.
* @return                  Operation status.
*/
GoFx(kStatus) GoReplayAnyData_SetRangeCountCase(GoReplayAnyData condition, GoReplayRangeCountCase rangeCase);

/**
* Gets the required range count case.
*
* @public                  @memberof GoReplayAnyData
* @version                 Introduced in firmware 4.5.3.57
* @param   condition       GoReplayAnyData object.
* @return                  The required range count case.
*/
GoFx(GoReplayRangeCountCase) GoReplayAnyData_RangeCountCase(GoReplayAnyData condition);

/**
* Sets the required range count threshold.
*
* @public                  @memberof GoReplayAnyData
* @version                 Introduced in firmware 4.5.3.57
* @param   condition       GoReplayAnyData object.
* @param   threshold       The required range count threshold.
* @return                  Operation status.
*/
GoFx(kStatus) GoReplayAnyData_SetRangeCountThreshold(GoReplayAnyData condition, k32u threshold);

/**
* Gets the required range count threshold.
*
* @public                  @memberof GoReplayAnyData
* @version                 Introduced in firmware 4.5.3.57
* @param   condition       GoReplayAnyData object.
* @return                  The required range count threshold.
*/
GoFx(k32u) GoReplayAnyData_RangeCountThreshold(GoReplayAnyData condition);

/**
* @class   GoReplayMeasurement
* @extends GoReplayCondition
* @ingroup GoSdk-Replay
* @brief   Represents the Measurement condition.
*/
typedef GoReplayCondition GoReplayMeasurement;

/**
* Sets the required measurement result.
*
* @public                  @memberof GoReplayAnyData
* @version                 Introduced in firmware 4.5.3.57
* @param   condition       GoReplayAnyData object.
* @param   result          The required measurement result.
* @return                  Operation status.
*/
GoFx(kStatus) GoReplayMeasurement_SetResult(GoReplayAnyData condition, GoReplayMeasurementResult result);

/**
* Gets the required measurement result.
*
* @public                  @memberof GoReplayMeasurement
* @version                 Introduced in firmware 4.5.3.57
* @param   condition       GoReplayMeasurement object.
* @return                  The required measurement result.
*/
GoFx(GoReplayMeasurementResult) GoReplayMeasurement_Result(GoReplayMeasurement condition);

/**
* Gets the ID count.
*
* @public                  @memberof GoReplayMeasurement
* @version                 Introduced in firmware 4.5.3.57
* @param   condition       GoReplayMeasurement object.
* @return                  The ID count.
*/
GoFx(kSize) GoReplayMeasurement_IdCount(GoReplayMeasurement condition);

/**
* Gets the ID at the specified index.
*
* @public                  @memberof GoReplayMeasurement
* @version                 Introduced in firmware 4.5.3.57
* @param   condition       GoReplayMeasurement object.
* @param   index           The index of the ID.
* @return                  The ID at the specified index.
*/
GoFx(k32u) GoReplayMeasurement_IdAt(GoReplayMeasurement condition, kSize index);

/**
* Sets the ID at the specified index.
*
* @public                  @memberof GoReplayAnyData
* @version                 Introduced in firmware 4.5.3.57
* @param   condition       GoReplayAnyData object.
* @param   index           The index of the ID.
* @param   id              The ID.
* @return                  Operation status.
*/
GoFx(kStatus) GoReplayMeasurement_SetIdAt(GoReplayMeasurement condition, kSize index, k32u id);

#include <GoSdk/GoReplayCondition.x.h>

#endif

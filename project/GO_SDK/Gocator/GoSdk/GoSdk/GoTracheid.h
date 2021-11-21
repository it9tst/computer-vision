/** 
 * @file    GoTracheid.h
 * @brief   Declares the GoTransform class. 
 *
 * @internal
 * Copyright (C) 2016-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef GO_TRACHEID_H
#define GO_TRACHEID_H

#include <GoSdk/GoSdkDef.h>

/**
 * @class   GoTracheid
 * @extends kObject
 * @ingroup GoSdk
 * @brief   Represents tracheid data.
 */
typedef kObject GoTracheid; 

/** 
 * Returns a boolean representing whether tracheid is used.
 *
 * @public              @memberof GoTracheid
 * @version             Introduced in firmware 4.5.3.57
 * @param   tracheid    GoTracheid object.
 * @return              kTRUE if used, or kFALSE if not used.
 */
GoFx(kBool) GoTracheid_Used(GoTracheid tracheid);

/** 
 * Sets the exposure to use for tracheids.
 *
 * @public              @memberof GoTracheid
 * @version             Introduced in firmware 4.5.3.57
 * @param   tracheid    GoTracheid object.
 * @param   exposure    Exposure value (microseconds).
 * @return              Operation status.
 */
GoFx(kStatus) GoTracheid_SetExposure(GoTracheid tracheid, k64f exposure);

/**
 * Gets the exposure used for tracheids.
 *
 * @public              @memberof GoTracheid
 * @version             Introduced in firmware 4.5.3.57
 * @param   tracheid    GoTracheid object.
 * @return              Exposure value (microseconds).
 */
GoFx(k64f) GoTracheid_Exposure(GoTracheid tracheid);

/** 
 * Enables or disables tracheid exposure.
 *
 * @public              @memberof GoTracheid
 * @version             Introduced in firmware 4.5.3.57
 * @param   tracheid    GoTracheid object.
 * @param   enable      kTRUE to enable it and kFALSE to disable it.
 * @return              Operation status.
 */
GoFx(kStatus) GoTracheid_EnableExposure(GoTracheid tracheid, kBool enable);

/**
 * Determines if tracheid exposure is enabled.
 *
 * @public              @memberof GoTracheid
 * @version             Introduced in firmware 4.5.3.57
 * @param   tracheid    GoTracheid object.
 * @return              kTRUE if enabled and kFALSE otherwise.
 */
GoFx(kBool) GoTracheid_ExposureEnabled(GoTracheid tracheid);

/** 
 * Sets the camera threshold.
 *
 * @public              @memberof GoTracheid
 * @version             Introduced in firmware 4.5.3.57
 * @param   tracheid    GoTracheid object.
 * @param   cameraIndex Camera index (0 or 1).
 * @param   threshold   Threshold value.
 * @return              Operation status.
 */
GoFx(kStatus) GoTracheid_SetCameraThresholdAt(GoTracheid tracheid, kSize cameraIndex, k32s threshold);

/**
 * Gets the camera threshold.
 *
 * @public              @memberof GoTracheid
 * @version             Introduced in firmware 4.5.3.57
 * @param   tracheid    GoTracheid object.
 * @param   cameraIndex Camera index (0 or 1).
 * @param   threshold   A reference to be updated with the threshold value.
 * @return              Operation status.
 */
GoFx(kStatus) GoTracheid_GetCameraThresholdAt(GoTracheid tracheid, kSize cameraIndex, k32s* threshold);

#include <GoSdk/GoTracheid.x.h>

#endif

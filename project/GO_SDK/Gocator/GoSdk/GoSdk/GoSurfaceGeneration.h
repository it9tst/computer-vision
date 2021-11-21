/** 
 * @file    GoSurfaceGeneration.h
 * @brief   Declares the GoSurfaceGeneration class. 
 *
 * @internal
 * Copyright (C) 2016-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef GO_SURFACEGENERATION_H
#define GO_SURFACEGENERATION_H

#include <GoSdk/GoSdkDef.h>

/**
 * @class   GoSurfaceGeneration
 * @extends kObject
 * @note    Supported with G2, G3
 * @ingroup GoSdk-Surface
 * @brief   Represents a surface generation configuration.
 */
typedef kObject GoSurfaceGeneration; 

/** 
 * Sets the surface generation type.
 *
 * @public                  @memberof GoSurfaceGeneration
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.0.10.27
 * @param   surface         GoSurfaceGeneration object.
 * @param   type            The surface generation type to set.
 * @return                  Operation status.
 */
GoFx(kStatus) GoSurfaceGeneration_SetGenerationType(GoSurfaceGeneration surface, GoSurfaceGenerationType type);

/** 
 * Gets the current surface generation type.
 *
 * @public                  @memberof GoSurfaceGeneration
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.0.10.27
 * @param   surface         GoSurfaceGeneration object.
 * @return                  The surface generation type.
 */
GoFx(GoSurfaceGenerationType) GoSurfaceGeneration_GenerationType(GoSurfaceGeneration surface);

/** 
 * Sets the fixed length surface generation start trigger.
 *
 * @public                  @memberof GoSurfaceGeneration
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.0.10.27
 * @param   surface         GoSurfaceGeneration object.
 * @param   trigger         The surface generation start trigger value to set.
 * @return                  Operation status.
 */
GoFx(kStatus) GoSurfaceGenerationFixedLength_SetStartTrigger(GoSurfaceGeneration surface, GoSurfaceGenerationStartTrigger trigger);

/** 
 * Gets the fixed length surface generation start trigger.
 *
 * @public                  @memberof GoSurfaceGeneration
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.0.10.27
 * @param   surface         GoSurfaceGeneration object.
 * @return                  The fixed length surface generation start trigger.
 */
GoFx(GoSurfaceGenerationStartTrigger) GoSurfaceGenerationFixedLength_StartTrigger(GoSurfaceGeneration surface);

/** 
 * Sets the fixed length surface generation surface length.
 *
 * @public                  @memberof GoSurfaceGeneration
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.0.10.27
 * @param   surface         GoSurfaceGeneration object.
 * @param   length          The fixed length surface generation surface length to set.
 * @return                  Operation status.
 */
GoFx(kStatus) GoSurfaceGenerationFixedLength_SetLength(GoSurfaceGeneration surface, k64f length);

/** 
 * Gets the fixed length surface generation surface length.
 *
 * @public                  @memberof GoSurfaceGeneration
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.0.10.27
 * @param   surface         GoSurfaceGeneration object.
 * @return                  The fixed length surface generation surface length.
 */
GoFx(k64f) GoSurfaceGenerationFixedLength_Length(GoSurfaceGeneration surface);

/** 
 * Gets the fixed length surface generation circumference limit maximum value.
 *
 * @public                  @memberof GoSurfaceGeneration
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.0.10.27
 * @param   surface         GoSurfaceGeneration object.
 * @return                  The fixed length surface generation circumference limit maximum value.
 */
GoFx(k64f) GoSurfaceGenerationFixedLength_LengthLimitMax(GoSurfaceGeneration surface);

/** 
 * Gets the fixed length surface generation circumference limit minimum value.
 *
 * @public                  @memberof GoSurfaceGeneration
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.0.10.27
 * @param   surface         GoSurfaceGeneration object.
 * @return                  The fixed length surface generation circumference limit minimum value.
 */
GoFx(k64f) GoSurfaceGenerationFixedLength_LengthLimitMin(GoSurfaceGeneration surface);

/** 
 * Gets the flag indicating whether the external input trigger is being used.
 *
 * @public              @memberof GoSurfaceGeneration
 * @note                Supported with G2, G3
 * @version             Introduced in firmware 5.2.1.110
 * @param   surface     GoSurfaceGeneration object.
 * @return              TRUE if being used, FALSE otherwise.
 */
GoFx(kBool)GoSurfaceGenerationFixedLength_TriggerExternalInputIndexUsed(GoSurfaceGeneration surface);

/** 
 * Sets the external input index to trigger surface capturing.
 *
 * @public              @memberof GoSurfaceGeneration
 * @note                Supported with G2, G3
 * @version             Introduced in firmware 5.2.1.110
 * @param   surface     GoSurfaceGeneration object.
 * @param   index       External digital input index.
 * @return              Operation status.
 */
GoFx(kStatus) GoSurfaceGenerationFixedLength_SetTriggerExternalInputIndex(GoSurfaceGeneration surface, k32s index);

/** 
 * Gets the external input index to trigger surface capturing.
 *
 * @public              @memberof GoSurfaceGeneration
 * @note                Supported with G2, G3
 * @version             Introduced in firmware 5.2.1.110
 * @param   surface     GoSurfaceGeneration object.
 * @return              External input index to trigger capturing.
 */
GoFx(k32s) GoSurfaceGenerationFixedLength_TriggerExternalInputIndex(GoSurfaceGeneration surface);

/** 
 * Gets the count of available external input trigger index options.
 *
 * @public              @memberof GoSurfaceGeneration
 * @note                Supported with G2, G3
 * @version             Introduced in firmware 5.2.1.110
 * @param   surface     GoSurfaceGeneration object.
 * @return              Count of external input index options.
 */
GoFx(kSize) GoSurfaceGenerationFixedLength_TriggerExternalInputIndexOptionCount(GoSurfaceGeneration surface);

/** 
 * Sets the variable length surface generation maximum length.
 *
 * @public                  @memberof GoSurfaceGeneration
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.0.10.27
 * @param   surface         GoSurfaceGeneration object.
 * @param   length          The variable length surface generation maximum length to set.
 * @return                  Operation status.
 */
GoFx(kStatus) GoSurfaceGenerationVariableLength_SetMaxLength(GoSurfaceGeneration surface, k64f length);

/** 
 * Gets the variable length surface generation maximum length.
 *
 * @public                  @memberof GoSurfaceGeneration
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.0.10.27
 * @param   surface         GoSurfaceGeneration object.
 * @return                  The variable length surface generation maximum length.
 */
GoFx(k64f) GoSurfaceGenerationVariableLength_MaxLength(GoSurfaceGeneration surface);

/** 
 * Gets the variable length surface generation circumference limit maximum value.
 *
 * @public                  @memberof GoSurfaceGeneration
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.0.10.27
 * @param   surface         GoSurfaceGeneration object.
 * @return                  The variable length surface generation circumference limit maximum value.
 */
GoFx(k64f) GoSurfaceGenerationVariableLength_MaxLengthLimitMax(GoSurfaceGeneration surface);

/** 
 * Gets the variable length surface generation circumference limit minimum value.
 *
 * @public                  @memberof GoSurfaceGeneration
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.0.10.27
 * @param   surface         GoSurfaceGeneration object.
 * @return                  The variable length surface generation circumference limit minimum value.
 */
GoFx(k64f) GoSurfaceGenerationVariableLength_MaxLengthLimitMin(GoSurfaceGeneration surface);

/**
 * Sets the rotational surface generation surface encoder resolution.
 *
 * @public                  @memberof GoSurfaceGeneration
 * @note                    Supported with G2, G3
 * @version
 * @param   surface         GoSurfaceGeneration object.
 * @param   value           The rotational surface generation encoder resolution value [ticks/rev] to set.
 * @return                  Operation status.
 */
GoFx(kStatus) GoSurfaceGenerationRotational_SetEncoderResolution(GoSurfaceGeneration surface, k64f value);

/**
 * Gets the rotational surface generation encoder resolution in ticks/rev.
 *
 * @public                  @memberof GoSurfaceGeneration
 * @note                    Supported with G2, G3
 * @version
 * @param   surface         GoSurfaceGeneration object.
 * @return                  The rotational surface generation encoder resolution.
 */
GoFx(k64f) GoSurfaceGenerationRotational_EncoderResolution(GoSurfaceGeneration surface);

/**
* [Deprecated]
*
* Sets the rotational surface generation surface circumference.
*
* @deprecated
* @public                  @memberof GoSurfaceGeneration
* @note                    Supported with G2, G3
* @version                 Introduced in firmware 4.0.10.27
* @param   surface         GoSurfaceGeneration object.
* @param   value           The rotational surface generation circumference value to set.
* @return                  Operation status.
*/
GoFx(kStatus) GoSurfaceGenerationRotational_SetCircumference(GoSurfaceGeneration surface, k64f value);

/** 
* [Deprecated]
*
* Gets the rotational surface generation circumference.
*
* @deprecated
* @public                  @memberof GoSurfaceGeneration
* @note                    Supported with G2, G3
* @version                 Introduced in firmware 4.0.10.27
* @param   surface         GoSurfaceGeneration object.
* @return                  The rotational surface generation circumference.
*/
GoFx(k64f) GoSurfaceGenerationRotational_Circumference(GoSurfaceGeneration surface);

/** 
* [Deprecated]
*
* Gets the rotational surface generation circumference limit maximum value.
*
* @deprecated
* @public                  @memberof GoSurfaceGeneration
* @note                    Supported with G2, G3
* @version                 Introduced in firmware 4.0.10.27
* @param   surface         GoSurfaceGeneration object.
* @return                  The rotational surface generation circumference limit maximum value.
*/
GoFx(k64f) GoSurfaceGenerationRotational_CircumferenceLimitMax(GoSurfaceGeneration surface);

/** 
* [Deprecated] 
*
* Gets the rotational surface generation circumference limit minimum value.
*
* @deprecated
* @public                  @memberof GoSurfaceGeneration
* @note                    Supported with G2, G3
* @version                 Introduced in firmware 4.0.10.27
* @param   surface         GoSurfaceGeneration object.
* @return                  The rotational surface generation circumference limit minimum value.
*/
GoFx(k64f) GoSurfaceGenerationRotational_CircumferenceLimitMin(GoSurfaceGeneration surface);

#include <GoSdk/GoSurfaceGeneration.x.h>

#endif

/** 
 * @file    GoProfileGeneration.h
 * @brief   Declares the GoProfileGeneration class. 
 *
 * @internal
 * Copyright (C) 2016-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef GO_PROFILEGENERATION_H
#define GO_PROFILEGENERATION_H

#include <GoSdk/GoSdkDef.h>

/**
 * @class   GoProfileGeneration
 * @extends kObject
 * @note    Supported with G1
 * @ingroup GoSdk-Profile
 * @brief   Represents a profile generation configuration.
 */
typedef kObject GoProfileGeneration; 

/** 
 * Sets the profile generation type.
 *
 * @public                  @memberof GoProfileGeneration
 * @note                    Supported with G1
 * @version                 Introduced in firmware 4.2.4.7
 * @param   profile         GoProfileGeneration object.
 * @param   type            The profile generation type to set.
 * @return                  Operation status.
 */
GoFx(kStatus) GoProfileGeneration_SetGenerationType(GoProfileGeneration profile, GoProfileGenerationType type);

/** 
 * Gets the current profile generation type.
 *
 * @public                  @memberof GoProfileGeneration
 * @note                    Supported with G1
 * @version                 Introduced in firmware 4.2.4.7
 * @param   profile         GoProfileGeneration object.
 * @return                  The profile generation type.
 */
GoFx(GoProfileGenerationType) GoProfileGeneration_GenerationType(GoProfileGeneration profile);

/** 
 * Sets the fixed length profile generation start trigger.
 *
 * @public                  @memberof GoProfileGeneration
 * @note                    Supported with G1
 * @version                 Introduced in firmware 4.2.4.7
 * @param   profile         GoProfileGeneration object.
 * @param   trigger         The profile generation start trigger value to set.
 * @return                  Operation status.
 */
GoFx(kStatus) GoProfileGenerationFixedLength_SetStartTrigger(GoProfileGeneration profile, GoProfileGenerationStartTrigger trigger);

/** 
 * Gets the fixed length profile generation start trigger.
 *
 * @public                  @memberof GoProfileGeneration
 * @note                    Supported with G1
 * @version                 Introduced in firmware 4.2.4.7
 * @param   profile         GoProfileGeneration object.
 * @return                  The fixed length profile generation start trigger.
 */
GoFx(GoProfileGenerationStartTrigger) GoProfileGenerationFixedLength_StartTrigger(GoProfileGeneration profile);

/** 
 * Gets the flag indicating whether the external input trigger is being used.
 *
 * @public              @memberof GoProfileGeneration
 * @note                Supported with G1
 * @version             Introduced in firmware 5.2.1.110
 * @param   surface     GoProfileGeneration object.
 * @return              TRUE if being used, FALSE otherwise.
 */
GoFx(kBool)GoProfileGenerationFixedLength_TriggerExternalInputIndexUsed(GoProfileGeneration surface);

/** 
 * Sets the external input index to trigger surface capturing.
 *
 * @public              @memberof GoProfileGeneration
 * @note                Supported with G1
 * @version             Introduced in firmware 5.2.1.110
 * @param   surface     GoProfileGeneration object.
 * @param   index       External digital input index.
 * @return              Operation status.
 */
GoFx(kStatus) GoProfileGenerationFixedLength_SetTriggerExternalInputIndex(GoProfileGeneration surface, k32s index);

/** 
 * Gets the external input index to trigger surface capturing.
 *
 * @public              @memberof GoProfileGeneration
 * @note                Supported with G1
 * @version             Introduced in firmware 5.2.1.110
 * @param   surface     GoProfileGeneration object.
 * @return              External input index to trigger capturing.
 */
GoFx(k32s) GoProfileGenerationFixedLength_TriggerExternalInputIndex(GoProfileGeneration surface);

/** 
 * Gets the count of available external input trigger index options.
 *
 * @public              @memberof GoProfileGeneration
 * @note                Supported with G1
 * @version             Introduced in firmware 5.2.1.110
 * @param   surface     GoProfileGeneration object.
 * @return              Count of external input index options.
 */
GoFx(kSize) GoProfileGenerationFixedLength_TriggerExternalInputIndexOptionCount(GoProfileGeneration surface);

/** 
 * Sets the fixed length profile generation profile length.
 *
 * @public                  @memberof GoProfileGeneration
 * @note                    Supported with G1
 * @version                 Introduced in firmware 4.2.4.7
 * @param   profile         GoProfileGeneration object.
 * @param   length          The fixed length profile generation profile length to set.
 * @return                  Operation status.
 */
GoFx(kStatus) GoProfileGenerationFixedLength_SetLength(GoProfileGeneration profile, k64f length);

/** 
 * Gets the fixed length profile generation profile length.
 *
 * @public                  @memberof GoProfileGeneration
 * @note                    Supported with G1
 * @version                 Introduced in firmware 4.2.4.7
 * @param   profile         GoProfileGeneration object.
 * @return                  The fixed length profile generation profile length.
 */
GoFx(k64f) GoProfileGenerationFixedLength_Length(GoProfileGeneration profile);

/** 
 * Gets the fixed length profile generation circumference limit maximum value.
 *
 * @public                  @memberof GoProfileGeneration
 * @note                    Supported with G1
 * @version                 Introduced in firmware 4.2.4.7
 * @param   profile         GoProfileGeneration object.
 * @return                  The fixed length profile generation circumference limit maximum value.
 */
GoFx(k64f) GoProfileGenerationFixedLength_LengthLimitMax(GoProfileGeneration profile);

/** 
 * Gets the fixed length profile generation circumference limit minimum value.
 *
 * @public                  @memberof GoProfileGeneration
 * @note                    Supported with G1
 * @version                 Introduced in firmware 4.2.4.7
 * @param   profile         GoProfileGeneration object.
 * @return                  The fixed length profile generation circumference limit minimum value.
 */
GoFx(k64f) GoProfileGenerationFixedLength_LengthLimitMin(GoProfileGeneration profile);

/** 
 * Sets the variable length profile generation maximum length.
 *
 * @public                  @memberof GoProfileGeneration
 * @note                    Supported with G1
 * @version                 Introduced in firmware 4.2.4.7
 * @param   profile         GoProfileGeneration object.
 * @param   length          The variable length profile generation maximum length to set.
 * @return                  Operation status.
 */
GoFx(kStatus) GoProfileGenerationVariableLength_SetMaxLength(GoProfileGeneration profile, k64f length);

/** 
 * Gets the variable length profile generation maximum length.
 *
 * @public                  @memberof GoProfileGeneration
 * @note                    Supported with G1
 * @version                 Introduced in firmware 4.2.4.7
 * @param   profile         GoProfileGeneration object.
 * @return                  The variable length profile generation maximum length.
 */
GoFx(k64f) GoProfileGenerationVariableLength_MaxLength(GoProfileGeneration profile);

/** 
 * Gets the variable length profile generation circumference limit maximum value.
 *
 * @public                  @memberof GoProfileGeneration
 * @note                    Supported with G1
 * @version                 Introduced in firmware 4.2.4.7
 * @param   profile         GoProfileGeneration object.
 * @return                  The variable length profile generation circumference limit maximum value.
 */
GoFx(k64f) GoProfileGenerationVariableLength_MaxLengthLimitMax(GoProfileGeneration profile);

/** 
 * Gets the variable length profile generation circumference limit minimum value.
 *
 * @public                  @memberof GoProfileGeneration
 * @note                    Supported with G1
 * @version                 Introduced in firmware 4.2.4.7
 * @param   profile         GoProfileGeneration object.
 * @return                  The variable length profile generation circumference limit minimum value.
 */
GoFx(k64f) GoProfileGenerationVariableLength_MaxLengthLimitMin(GoProfileGeneration profile);

/** 
 * Sets the rotational profile generation profile circumference.
 *
 * @public                  @memberof GoProfileGeneration
 * @note                    Supported with G1
 * @version                 Introduced in firmware 4.2.4.7
 * @param   profile         GoProfileGeneration object.
 * @param   value           The rotational profile generation circumference value to set.
 * @return                  Operation status.
 */
GoFx(kStatus) GoProfileGenerationRotational_SetCircumference(GoProfileGeneration profile, k64f value);

/** 
 * Gets the rotational profile generation circumference.
 *
 * @public                  @memberof GoProfileGeneration
 * @note                    Supported with G1
 * @version                 Introduced in firmware 4.2.4.7
 * @param   profile         GoProfileGeneration object.
 * @return                  The rotational profile generation circumference.
 */
GoFx(k64f) GoProfileGenerationRotational_Circumference(GoProfileGeneration profile);

/** 
 * Gets the rotational profile generation circumference limit maximum value.
 *
 * @public                  @memberof GoProfileGeneration
 * @note                    Supported with G1
 * @version                 Introduced in firmware 4.2.4.7
 * @param   profile         GoProfileGeneration object.
 * @return                  The rotational profile generation circumference limit maximum value.
 */
GoFx(k64f) GoProfileGenerationRotational_CircumferenceLimitMax(GoProfileGeneration profile);

/** 
 * Gets the rotational profile generation circumference limit minimum value.
 *
 * @public                  @memberof GoProfileGeneration
 * @note                    Supported with G1
 * @version                 Introduced in firmware 4.2.4.7
 * @param   profile         GoProfileGeneration object.
 * @return                  The rotational profile generation circumference limit minimum value.
 */
GoFx(k64f) GoProfileGenerationRotational_CircumferenceLimitMin(GoProfileGeneration profile);

#include <GoSdk/GoProfileGeneration.x.h>

#endif

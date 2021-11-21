/** 
 * @file    GoTransform.h
 * @brief   Declares the GoTransform class. 
 *
 * @internal
 * Copyright (C) 2016-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef GO_TRANSFORM_H
#define GO_TRANSFORM_H

#include <GoSdk/GoSdkDef.h>

/**
 * @class   GoTransform
 * @extends kObject
 * @ingroup GoSdk
 * @brief   Represents a sensor transformation.
 */
typedef kObject GoTransform; 

/** 
 * Gets the encoder resolution.
 *
 * @public                  @memberof GoTransform
 * @version                 Introduced in firmware 4.0.10.27
 * @param    transform      GoTransform object.
 * @return                  The encoder resolution in mm/tick.
 */
GoFx(k64f) GoTransform_EncoderResolution(GoTransform transform);

/** 
 * Sets the encoder resolution.
 *
 * WARNING! This operation writes to flash storage.
 * Review the user manual for implications.
 *
 * @public               @memberof GoTransform
 * @version              Introduced in firmware 4.0.10.27
 * @param    transform   GoTransform object.
 * @param    value       Encoder resolution in mm/tick.
 * @return               Operation status.            
 */
GoFx(kStatus) GoTransform_SetEncoderResolution(GoTransform transform, k64f value);

/** 
 * Gets the travel speed.
 *
 * @public                  @memberof GoTransform
 * @version                 Introduced in firmware 4.0.10.27
 * @param    transform      GoTransform object.
 * @return                  The travel speed in mm/sec.
 */
GoFx(k64f) GoTransform_Speed(GoTransform transform);

/** 
 * Sets the travel speed.
 *
 * WARNING! This operation writes to flash storage.
 * Review the user manual for implications.
 *
 * @public               @memberof GoTransform
 * @version              Introduced in firmware 4.0.10.27
 * @param    transform   GoTransform object.
 * @param    value       travel speed in mm/sec.
 * @return               Operation status.            
 */
GoFx(kStatus) GoTransform_SetSpeed(GoTransform transform, k64f value);

/** 
 * Gets the X component of the transformation.
 *
 * @public                  @memberof GoTransform
 * @version                 Introduced in firmware 4.0.10.27
 * @param    transform      GoTransform object.
 * @param    role           Determines which device to retrieve the value from. Use GO_ROLE_MAIN or GOROLE_BUDDYIDX(buddyidx). See GoRole for more details.
 * @return                  The transformation X component.
 */
GoFx(k64f) GoTransform_X(GoTransform transform, GoRole role);

/** 
 * Sets the transformation X component.
 *
 * WARNING! This operation writes to flash storage.
 * Review the user manual for implications.
 *
 * @public               @memberof GoTransform
 * @version              Introduced in firmware 4.0.10.27
 * @param    transform   GoTransform object.
 * @param    role        Determines which device to apply changes to. Use GO_ROLE_MAIN or GOROLE_BUDDYIDX(buddyidx). See GoRole for more details.
 * @param    offset      The transformation X component to set.
 * @return               Operation status.            
 */
GoFx(kStatus) GoTransform_SetX(GoTransform transform, GoRole role, k64f offset);

/** 
 * Gets the Y component of the transformation.
 *
 * @public                  @memberof GoTransform
 * @version                 Introduced in firmware 4.0.10.27
 * @param    transform      GoTransform object.
 * @param    role           Determines which device to retrieve the value from. Use GO_ROLE_MAIN or GOROLE_BUDDYIDX(buddyidx). See GoRole for more details.
 * @return                  The transformation Y component.
 */
GoFx(k64f) GoTransform_Y(GoTransform transform, GoRole role);

/** 
 * Sets the transformation Y component.
 *
 * WARNING! This operation writes to flash storage.
 * Review the user manual for implications.
 *
 * @public               @memberof GoTransform
 * @version              Introduced in firmware 4.0.10.27
 * @param    transform   GoTransform object.
 * @param    role        Determines which device to apply changes to. Use GO_ROLE_MAIN or GOROLE_BUDDYIDX(buddyidx). See GoRole for more details.
 * @param    offset      The transformation Y component to set.
 * @return               Operation status.            
 */
GoFx(kStatus) GoTransform_SetY(GoTransform transform, GoRole role, k64f offset);

/** 
 * Gets the Z component of the transformation.
 *
 * @public                  @memberof GoTransform
 * @version                 Introduced in firmware 4.0.10.27
 * @param    transform      GoTransform object.
 * @param    role           Determines which device to retrieve the value from. Use GO_ROLE_MAIN or GOROLE_BUDDYIDX(buddyidx). See GoRole for more details.
 * @return                  The transformation Z component.
 */
GoFx(k64f) GoTransform_Z(GoTransform transform, GoRole role);

/** 
 * Sets the transformation Z component.
 *
 * WARNING! This operation writes to flash storage.
 * Review the user manual for implications.
 *
 * @public               @memberof GoTransform
 * @version              Introduced in firmware 4.0.10.27
 * @param    transform   GoTransform object.
 * @param    role        Determines which device to apply changes to. Use GO_ROLE_MAIN or GOROLE_BUDDYIDX(buddyidx). See GoRole for more details.
 * @param    offset      The transformation Z component to set.
 * @return               Operation status.            
 */
GoFx(kStatus) GoTransform_SetZ(GoTransform transform, GoRole role, k64f offset);

/** 
 * Gets the X-angle of the transformation.
 *
 * @public                  @memberof GoTransform
 * @version                 Introduced in firmware 4.0.10.27
 * @param    transform      GoTransform object.
 * @param    role           Determines which device to retrieve the value from. Use GO_ROLE_MAIN or GOROLE_BUDDYIDX(buddyidx). See GoRole for more details.
 * @return                  The transformation X-angle in degrees
 */
GoFx(k64f) GoTransform_XAngle(GoTransform transform, GoRole role);

/** 
 * Sets the transformation X-angle.
 *
 * WARNING! This operation writes to flash storage.
 * Review the user manual for implications.
 *
 * @public               @memberof GoTransform
 * @version              Introduced in firmware 4.0.10.27
 * @param    transform   GoTransform object.
 * @param    role        Determines which device to apply changes to. Use GO_ROLE_MAIN or GOROLE_BUDDYIDX(buddyidx). See GoRole for more details.
 * @param    offset      The transformation X-angle to set in degrees
 * @return               Operation status.            
 */
GoFx(kStatus) GoTransform_SetXAngle(GoTransform transform, GoRole role, k64f offset);

/** 
 * Gets the Y-angle of the transformation.
 *
 * @public                  @memberof GoTransform
 * @version                 Introduced in firmware 4.0.10.27
 * @param    transform      GoTransform object.
 * @param    role           Determines which device to retrieve the value from. Use GO_ROLE_MAIN or GOROLE_BUDDYIDX(buddyidx). See GoRole for more details.
 * @return                  The transformation Y-angle component in degrees
 */
GoFx(k64f) GoTransform_YAngle(GoTransform transform, GoRole role);

/** 
 * Sets the transformation Y-angle.
 *
 * WARNING! This operation writes to flash storage.
 * Review the user manual for implications.
 *
 * @public               @memberof GoTransform
 * @version              Introduced in firmware 4.0.10.27
 * @param    transform   GoTransform object.
 * @param    role        Determines which device to apply changes to. Use GO_ROLE_MAIN or GOROLE_BUDDYIDX(buddyidx). See GoRole for more details.
 * @param    offset      The transformation Y-angle to set in degrees
 * @return               Operation status.            
 */
GoFx(kStatus) GoTransform_SetYAngle(GoTransform transform, GoRole role, k64f offset);

/** 
 * Gets the Z-angle of the transformation.
 *
 * @public                  @memberof GoTransform
 * @version                 Introduced in firmware 4.0.10.27
 * @param    transform      GoTransform object.
 * @param    role           Determines which device to retrieve the value from. Use GO_ROLE_MAIN or GOROLE_BUDDYIDX(buddyidx). See GoRole for more details.
 * @return                  The transformation Z-angle component in degrees
 */
GoFx(k64f) GoTransform_ZAngle(GoTransform transform, GoRole role);

/** 
 * Sets the transformation Z-angle.
 *
 * WARNING! This operation writes to flash storage.
 * Review the user manual for implications.
 *
 * @public               @memberof GoTransform
 * @version              Introduced in firmware 4.0.10.27
 * @param    transform   GoTransform object.
 * @param    role        Determines which device to apply changes to. Use GO_ROLE_MAIN or GOROLE_BUDDYIDX(buddyidx). See GoRole for more details.
 * @param    offset      The transformation Z-angle to set in degrees
 * @return               Operation status.            
 */
GoFx(kStatus) GoTransform_SetZAngle(GoTransform transform, GoRole role, k64f offset);

#include <GoSdk/GoTransform.x.h>

#endif

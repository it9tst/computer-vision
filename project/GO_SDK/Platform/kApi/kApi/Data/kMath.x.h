/** 
 * @file    kMath.x.h
 * @brief   Utilities. 
 *
 * @internal
 * Copyright (C) 2005-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_MATH_X_H
#define K_API_MATH_X_H

#include <float.h>

typedef struct kMathStatic
{
    k32u placeholder;       //unused
} kMathStatic;

kDeclareStaticClassEx(k, kMath)

kFx(kStatus) xkMath_InitStatic(); 
kFx(kStatus) xkMath_ReleaseStatic();

/* 
* Actually public methods that should be made, well, actually public. 
*/

kFx(kStatus) kMath_Average8u(const k8u* v, kSize count, k64f* average);
kFx(kStatus) kMath_Stdev8u(const k8u* v, kSize count, k64f* stdev);

kFx(kStatus) kMath_Min8u(const k8u* v, kSize count, k8u* minValue);
kFx(kStatus) kMath_Max8u(const k8u* v, kSize count, k8u* maxValue);

kFx(kStatus) kMath_Min64u(const k64u* v, kSize count, k64u* minValue);
kFx(kStatus) kMath_Max64u(const k64u* v, kSize count, k64u* minValue);

kFx(kStatus) kMath_Sum64u(const k64u* v, kSize count, k64u* sum);

#endif

/**
* @file    GoExtMeasurement.h
* @brief   Declares the GoExtMeasurement class.
*
* @internal
* Copyright (C) 2017-2021 by LMI Technologies Inc.
* Licensed under the MIT License.
* Redistributed files must retain the above copyright notice.
*/
#ifndef GO_EXTMEASUREMENT_H
#define GO_EXTMEASUREMENT_H

#include <GoSdk/Tools/GoMeasurement.h>
#include <GoSdk/Tools/GoExtParams.h>

typedef GoMeasurement GoExtMeasurement;

/**
* Returns the type name of the extensible measurement
* @public                  @memberof GoExtMeasurement
* @version                 Introduced in firmware 4.6.3.142
* @param    measurement    GoExtMeasurement object.
* @return                  Type of extensible measurement..
*/
GoFx(const kChar*) GoExtMeasurement_Type(GoExtMeasurement measurement);

/**
* Returns the number of custom parameters available for this extensible measurement.
*
* @public                  @memberof GoExtMeasurement
* @version                 Introduced in firmware 4.6.3.142
* @param    measurement    GoExtMeasurement object.
* @return                  Number of extensible parameters.
*/
GoFx(kSize) GoExtMeasurement_CustomParameterCount(GoExtMeasurement measurement);

/**
* Returns the custom parameter at the specified index.
*
* @public                  @memberof GoExtMeasurement
* @version                 Introduced in firmware 4.6.3.142
* @param    measurement    GoExtMeasurement object.
* @param    index          Index of parameter to retrieve.
* @return                  The custom measurement parameter object.
*/
GoFx(GoExtParam) GoExtMeasurement_CustomParameterAt(GoExtMeasurement measurement, kSize index);

#include <GoSdk/Tools/GoExtMeasurement.x.h>

#endif

/**
* @file    GoGeoCal.h
* @brief   Declares the GoGeoCal class.
*
* @internal
* Copyright (C) 2017-2021 by LMI Technologies Inc.
* Licensed under the MIT License.
* Redistributed files must retain the above copyright notice.
*/
#ifndef GO_GEOCAL_H
#define GO_GEOCAL_H

#include <GoSdk/GoSdkDef.h>

/**
* @class   GoGeoCal
* @extends kObject
* @ingroup GoSdk
* @brief   Contains the GeoCal data for a sensor.
*          Do not read the GeoCal file directly, use this class to parse it.
*/
typedef kObject GoGeoCal;

/**
* Gets sensor ID
*
* @public                     @memberof GoGeoCal
* @version                    Introduced in firmware 4.7.0.130
* @param  cal                 GoGeoCal object.
* @return                     Sensor ID.
*/
GoFx(k32u) GoGeoCal_Id(GoGeoCal cal);

/**
* Gets timestamp
*
* @public                     @memberof GoGeoCal
* @version                    Introduced in firmware 4.7.0.130
* @param  cal                 GoGeoCal object.
* @return                     Timestamp.
*/
GoFx(const kChar*) GoGeoCal_Timestamp(GoGeoCal cal);

/**
* Gets count of cameras.
*
* @public                     @memberof GoGeoCal
* @version                    Introduced in firmware 4.7.0.130
* @param  cal                 GoGeoCal object.
* @return                     Count of views.
*/
GoFx(kSize) GoGeoCal_CameraCount(GoGeoCal cal);

/**
* Gets calibration window.
*
* @public                     @memberof GoGeoCal
* @version                    Introduced in firmware 4.7.0.130
* @param  cal                 GoGeoCal object.
* @param  cameraIndex         Camera index. kERROR_PARAMETER if invalid.
* @param  x                   Output parameter for window x. Provide with a pointer to a k32u. Ignored if input is kNULL.
* @param  y                   Output parameter for window y. Provide with a pointer to a k32u. Ignored if input is kNULL.
* @param  width               Output parameter for window width. Provide with a pointer to a k32u. Ignored if input is kNULL.
* @param  height              Output parameter for window height. Provide with a pointer to a k32u. Ignored if input is kNULL.
* @param  xSubsampling        Output parameter for window xSubsampling. Provide with a pointer to a k32u. Ignored if input is kNULL.
* @param  ySubsampling        Output parameter for window ySubsampling. Provide with a pointer to a k32u. Ignored if input is kNULL.
* @return                     Operation status.
*/
GoFx(kStatus) GoGeoCal_CalWindow(GoGeoCal cal, kSize cameraIndex, k32u* x, k32u* y, k32u* width, k32u* height, k32u* xSubsampling, k32u* ySubsampling);

/**
* Gets X resolution to Z fit.
* The X and Y resolution (mm/pixel) of an imager's pixels varies with the range. When resampling images,
* it is necessary to know the native resolution of each pixel. Typically, this is only needed for color/texture output.
* Profile cameras would have a fit set to kNULL. The fit is stored as polynomial coefficients. The X resolution at a
* given Z range (mm) can be determined by:
*
*       X resolution = coefficients[0] + coefficients[1]*Z + ... + coefficients[N]*Z^N
*
* @public                     @memberof GoGeoCal
* @version                    Introduced in firmware 4.7.0.130
* @param  cal                 GoGeoCal object.
* @param  cameraIndex         Camera index.
* @return                     Array of X resolution to Z fit coefficients. Type: k64f. kNULL is returned if the
*                             camera does not have a X resolution to Z fit.
*/
GoFx(kArray1) GoGeoCal_XResolutionFit(GoGeoCal cal, kSize cameraIndex);

/**
* Gets Y resolution to Z fit.
* The X and Y resolution (mm/pixel) of an imager's pixels varies with the range. When resampling images,
* it is necessary to know the native resolution of each pixel. Typically, this is only needed for color/texture output.
* Profile cameras would have a fit set to kNULL. The fit is stored as polynomial coefficients. The Y resolution at a
* given Z range (mm) can be determined by:
*
*       Y resolution = coefficients[0] + coefficients[1]*Z + ... + coefficients[N]*Z^N
*
* @public                     @memberof GoGeoCal
* @version                    Introduced in firmware 4.7.0.130
* @param  cal                 GoGeoCal object.
* @param  cameraIndex         Camera index.
* @return                     Array of Y resolution to Z fit coefficients. Type: k64f. kNULL is returned if the
*                             camera does not have a X resolution to Z fit.
*/
GoFx(kArray1) GoGeoCal_YResolutionFit(GoGeoCal cal, kSize cameraIndex);

/**
* Gets X center to Z fit.
* The world X center (mm) of an imager varies with the range if any yaw is present. It also represents the distance
* from the sensor reference 0 plane (usually mid-sensor). The fit is stored as polynomial coefficients. The X center
* at a given Z range (mm) can be determined by:
*
*       X center = coefficients[0] + coefficients[1]*Z + ... + coefficients[N]*Z^N
*
* @public                     @memberof GoGeoCal
* @version                    Introduced in firmware 4.7.0.130
* @param  cal                 GoGeoCal object.
* @param  cameraIndex         Camera index.
* @return                     Array of X center to Z fit coefficients. Type: k64f.
*/
GoFx(kArray1) GoGeoCal_XCenterFit(GoGeoCal cal, kSize cameraIndex);

/**
* Gets Y center to Z fit.
* The world Y center (mm) of an imager varies with the range if any pitch is present. It also represents the distance
* from the sensor reference 0 plane (usually mid-sensor). The fit is stored as polynomial coefficients. The Y center
* at a given Z range (mm) can be determined by:
*
*       Y center = coefficients[0] + coefficients[1]*Z + ... + coefficients[N]*Z^N
*
* @public                     @memberof GoGeoCal
* @version                    Introduced in firmware 4.7.0.130
* @param  cal                 GoGeoCal object.
* @param  cameraIndex         Camera index. kERROR_PARAMETER if invalid.
* @return                     Array of Y center to Z fit coefficients. Type: k64f.
*/
GoFx(kArray1) GoGeoCal_YCenterFit(GoGeoCal cal, kSize cameraIndex);

/**
* Gets camera roll angle. This can be used to correct for camera roll.
*
* @public                     @memberof GoGeoCal
* @version                    Introduced in firmware 4.7.0.130
* @param  cal                 GoGeoCal object.
* @param  cameraIndex         Camera index
* @return                     Roll angle (in radians)
*/
GoFx(k64f) GoGeoCal_Roll(GoGeoCal cal, kSize cameraIndex);

/**
* Applies a new camera window to the camera orientation information and updates coefficients.
* 
* @public                  @memberof GoGeoCal
* @version                 Introduced in firmware 5.2.18.3
* @param   cal             GoGeoCal object.
* @param   x               X of new camera window.
* @param   y               Y of new camera window.
* @param   width           Width of new camera window.
* @param   height          Height of new camera window.
* @param   xSubsampling    X subsampling of new camera window.
* @param   ySubsampling    Y subsampling of new camera window.
* @param   cameraIndex     Camera index.
* @return                  Operation status.
*/
GoFx(kStatus) GoGeoCal_ApplyActiveArea(GoGeoCal cal, k32u x, k32u y, k32u width, k32u height, k32u xSubsampling, k32u ySubsampling, kSize cameraIndex);

#include <GoSdk/GoGeoCal.x.h>

#endif

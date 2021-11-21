// 
// GoGeoCal.h
// 
// Copyright (C) 2017-2021 by LMI Technologies Inc.
// Licensed under the MIT License.
// Redistributed files must retain the above copyright notice.
// 
#ifndef GO_SDK_NET_GEOCAL_H
#define GO_SDK_NET_GEOCAL_H

#include <GoSdk/GoGeoCal.h>
#include <GoSdk/GoSensor.h>
#include <GoSdkNet/GoSdkDef.h>

namespace Lmi3d
{
    namespace GoSdk
    {
        /// <summary>Represents a sensor GeoCal file.</summary>
        public ref class GoGeoCal : public KObject
        {
            KDeclareClass(GoGeoCal, GoGeoCal)

                /// <summary>Initializes a new instance of the GoGeoCal class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoGeoCal(IntPtr handle)
                : KObject(handle)
            {}

            /// <summary>Initializes a new instance of the GoGeoCal class.</summary>
            /// <param name="sensor">The connected sensor to grab the GeoCal from</param>
            GoGeoCal(GoSensor^ sensor)
            {
                ::GoGeoCal handle = kNULL;

                KCheck(::GoSensor_GeoCal(KToHandle(sensor), &handle));

                Handle = handle;
            }

            /// <summary>Gets sensor ID</summary>
            k32u Id()
            {
                return ::GoGeoCal_Id(Handle);
            }

            /// <summary>Gets the timestamp</summary>
            property String^ Timestamp
            {
                String^ get()
                {
                    return KToString(::GoGeoCal_Timestamp(Handle));
                }
            }

            /// <summary>Gets the camera count</summary>
            property kSize CameraCount
            {
                kSize get()
                {
                    return ::GoGeoCal_CameraCount(Handle);
                }
            }

            /// <summary>Gets a calibration window</summary>
            /// <param name="cameraIndex">Camera index.</param>
            /// <param name="x">Output parameter for window x. Provide with a pointer to a k32u. Ignored if input is kNULL.</param>
            /// <param name="y">Output parameter for window y. Provide with a pointer to a k32u. Ignored if input is kNULL.</param>
            /// <param name="width">Output parameter for window width. Provide with a pointer to a k32u. Ignored if input is kNULL.</param>
            /// <param name="height">Output parameter for window height. Provide with a pointer to a k32u. Ignored if input is kNULL.</param>
            /// <param name="xSubsampling">Output parameter for window xSubsampling. Provide with a pointer to a k32u. Ignored if input is kNULL.</param>
            /// <param name="ySubsampling">Output parameter for window ySubsampling. Provide with a pointer to a k32u. Ignored if input is kNULL.</param>
            void CalWindow(kSize cameraIndex, k32u* x, k32u* y, k32u* width, k32u* height, k32u* xSubsampling, k32u* ySubsampling)
            {
                KCheck(::GoGeoCal_CalWindow(Handle, cameraIndex, x, y, width, height, xSubsampling, ySubsampling));
            }

            /// <summary>
            /// Gets X resolution to Z fit.
            /// The X and Y resolution (mm/pixel) of an imager's pixels varies with the range. When resampling images,
            /// it is necessary to know the native resolution of each pixel. Typically, this is only needed for color/texture output.
            /// Profile cameras would have a fit set to kNULL. The fit is stored as polynomial coefficients. The X resolution at a
            /// given Z range (mm) can be determined by:
            ///
            ///       X resolution = coefficients[0] + coefficients[1]*Z + ... + coefficients[N]*Z^N
            ///
            /// returns an Array of X resolution to Z fit coefficients. Type: k64f. kNULL is returned if the camera does not
            /// have a X resolution to Z fit.
            ///</summary>
            ///<param name="cameraIndex">Camera index.</param>
            KArray1^ XResolutionFit(kSize cameraIndex)
            {
                kArray1 arr = ::GoGeoCal_XResolutionFit(Handle, cameraIndex);
                return KToObject<KArray1^>(arr);
            }

            /// <summary>
            /// Gets Y resolution to Z fit.
            /// The Y and Y resolution (mm/pixel) of an imager's pixels varies with the range. When resampling images,
            /// it is necessary to know the native resolution of each pixel. Typically, this is only needed for color/texture output.
            /// Profile cameras would have a fit set to kNULL. The fit is stored as polynomial coefficients. The Y resolution at a
            /// given Z range (mm) can be determined by:
            ///
            ///       Y resolution = coefficients[0] + coefficients[1]*Z + ... + coefficients[N]*Z^N
            ///
            /// returns an Array of Y resolution to Z fit coefficients. Type: k64f. kNULL is returned if the camera does not
            /// have a Y resolution to Z fit.
            ///</summary>
            ///<param name="cameraIndex">Camera index.</param>
            KArray1^ YResolutionFit(kSize cameraIndex)
            {
                kArray1 arr = ::GoGeoCal_YResolutionFit(Handle, cameraIndex);
                return KToObject<KArray1^>(arr);
            }

            /// <summary>
            /// Gets X Center to Z fit.
            /// The world X center(mm) of an imager varies with the range if any yaw is present.It also represents the distance
            /// from the sensor reference 0 plane(usually mid - sensor).The fit is stored as polynomial coefficients.The X center
            /// at a given Z range(mm) can be determined by :
            ///
            ///       X Center = coefficients[0] + coefficients[1]*Z + ... + coefficients[N]*Z^N
            ///
            /// returns an Array of X Center to Z fit coefficients. Type: k64f
            ///</summary>
            ///<param name="cameraIndex">Camera index.</param>
            KArray1^ XCenterFit(kSize cameraIndex)
            {
                kArray1 arr = ::GoGeoCal_XCenterFit(Handle, cameraIndex);
                return KToObject<KArray1^>(arr);
            }

            /// <summary>
            /// Gets Y Center to Z fit.
            /// The world Y center(mm) of an imager varies with the range if any yaw is present.It also represents the distance
            /// from the sensor reference 0 plane(usually mid - sensor).The fit is stored as polynomial coefficients.The Y center
            /// at a given Z range(mm) can be determined by :
            ///
            ///       Y Center = coefficients[0] + coefficients[1]*Z + ... + coefficients[N]*Z^N
            ///
            /// returns an Array of Y Center to Z fit coefficients. Type: k64f
            ///</summary>
            ///<param name="cameraIndex">Camera index.</param>
            KArray1^ YCenterFit(kSize cameraIndex)
            {
                kArray1 arr = ::GoGeoCal_YCenterFit(Handle, cameraIndex);
                return KToObject<KArray1^>(arr);
            }

            ///<summary> Gets camera roll angle. This can be used to correct for camera roll.</summary>
            ///<param name="cameraIndex">Camera index.</param>
            k64f Roll(kSize cameraIndex)
            {
                return ::GoGeoCal_Roll(Handle, cameraIndex);
            }

            ///<summary> Applies a new camera window to the camera orientation information and updates coefficients.</summary> 
            ///<param name="x">X of new camera window.</param>
            ///<param name="y">Y of new camera window.</param>
            ///<param name="width">Width of new camera window.</param>
            ///<param name="height">Height of new camera window.</param>
            ///<param name="xSubsampling">X subsampling of new camera window.</param>
            ///<param name="ySubsampling">Y subsampling of new camera window.</param>
            ///<param name="cameraIndex">Camera index.</param>
            void ApplyActiveArea(k32u x, k32u y, k32u width, k32u height, k32u xSubsampling, k32u ySubsampling, kSize cameraIndex)
            {
                KCheck(::GoGeoCal_ApplyActiveArea(Handle, x, y, width, height, xSubsampling, ySubsampling, cameraIndex));
            }

        };
    }
}

#endif

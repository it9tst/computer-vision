// 
// GoTracheid.h
// 
// Copyright (C) 2016-2021 by LMI Technologies Inc.
// Licensed under the MIT License.
// Redistributed files must retain the above copyright notice.
// 
#ifndef GO_SDK_NET_TRACHEID_H
#define GO_SDK_NET_TRACHEID_H

#include <GoSdk/GoTracheid.h>
#include <GoSdkNet/GoSdkDef.h>

namespace Lmi3d
{
    namespace GoSdk
    {
        /// <summary>Represents a sensor transformation.</summary>
        public ref class GoTracheid : public KObject
        {
            KDeclareClass(GoTracheid, GoTracheid)

            /// <summary>Initializes a new instance of the GoTracheid class with the specified Zen object handle.</summary>
            /// <param name="handle">Zen object handle.</param>
            GoTracheid(IntPtr handle)
                : KObject(handle)
            {}

            /// <summary>Initializes a new instance of the GoTracheid class.</summary>
            /// <param name="sensor">A GoSensor instance.</param>
            GoTracheid(GoSensor^ sensor)
            {
                ::GoTracheid handle = kNULL;

                KCheck(::GoTracheid_Construct(&handle, KToHandle(sensor), kNULL));

                Handle = handle;
            }

            /// <inheritdoc cref="GoTracheid(GoSensor^ sensor)" />
            /// <param name="allocator">Memory allocator</param>
            GoTracheid(GoSensor^ sensor, KAlloc^ allocator)
            {
                ::GoTracheid handle = kNULL;

                KCheck(::GoTracheid_Construct(&handle, KToHandle(sensor), KToHandle(allocator)));

                Handle = handle;
            }

            /// <summary>Determines whether tracheid is used.</summary>
            property bool Used
            {
                bool get()           { return KToBool(::GoTracheid_Used(Handle)); }
            }

            /// <summary>The exposure (microseconds).</summary>
            property k64f Exposure
            {
                k64f get()           { return (k64f)::GoTracheid_Exposure(Handle); }
                void set(k64f value)  { KCheck(::GoTracheid_SetExposure(Handle, value)); }
            }

            /// <summary>The exposure enabled flag.</summary>
            property bool ExposureEnabled
            {
                bool get()           { return KToBool(::GoTracheid_ExposureEnabled(Handle)); }
                void set(bool value)  { KCheck(::GoTracheid_EnableExposure(Handle, value)); }
            }

            /// <summary>Gets the camera threshold.</summary>
            /// <param name="cameraIndex">Camera index (0 or 1).</param>
            k32s GetCameraThresholdAt(kSize cameraIndex)
            {
                k32s threshold;
                KCheck(::GoTracheid_GetCameraThresholdAt(Handle, cameraIndex, &threshold));

                return threshold;
            }

            /// <summary>Sets the camera threshold.</summary>
            /// <param name="cameraIndex">Camera index (0 or 1).</param>
            /// <param name="threshold">Threshold value.</param>
            void SetCameraThreshold(kSize cameraIndex, k32s threshold)
            {
                KCheck(::GoTracheid_SetCameraThresholdAt(Handle, cameraIndex, threshold));
            }
        };
    }
}

#endif

























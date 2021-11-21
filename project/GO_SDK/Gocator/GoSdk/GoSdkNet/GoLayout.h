// 
// GoLayout.h
// 
// Copyright (C) 2016-2021 by LMI Technologies Inc.
// Licensed under the MIT License.
// Redistributed files must retain the above copyright notice.
// 
#ifndef GO_SDK_NET_LAYOUT_H
#define GO_SDK_NET_LAYOUT_H

#include <GoSdk/GoLayout.h>
#include <GoSdkNet/GoSdkDef.h>

namespace Lmi3d
{
    namespace GoSdk
    {
        /// <summary>Represents a layout related sensor configuration.</summary>
        public ref class GoLayout : public KObject
        {
            KDeclareClass(GoLayout, GoLayout)

            /// <summary>Initializes a new instance of the GoLayout class with the specified Zen object handle.</summary>
            /// <param name="handle">Zen object handle.</param>
            GoLayout(IntPtr handle)
                : KObject(handle)
            {}

            /// <summary>Initializes a new instance of the GoLayout class.</summary>
            /// <param name="sensor">A GoSensor instance.</param>
            GoLayout(GoSensor^ sensor)
            {
                ::GoLayout handle = kNULL;

                KCheck(::GoLayout_Construct(&handle, KToHandle(sensor), kNULL));

                Handle = handle;
            }

            /// <inheritdoc cref="GoLayout(GoSensor^ sensor)" />
            /// <param name="allocator">Memory allocator</param>
            GoLayout(GoSensor^ sensor, KAlloc^ allocator)
            {
                ::GoLayout handle = kNULL;

                KCheck(::GoLayout_Construct(&handle, KToHandle(sensor), KToHandle(allocator)));

                Handle = handle;
            }

            /// <summary>The sensor configuration orientation used when buddied.</summary>
            property GoOrientation Orientation
            {
                GoOrientation get()           { return (GoOrientation) ::GoLayout_Orientation(Handle); }
                void set(GoOrientation orientation)  { KCheck(::GoLayout_SetOrientation(Handle, orientation)); }
            }

            /// <summary>The transformed data region X-component value.</summary>
            property k64f TransformedDataRegionX
            {
                k64f get()           { return ::GoLayout_TransformedDataRegionX(Handle); }
            }

            /// <summary>The transformed data region Y-component value.</summary>
            property k64f TransformedDataRegionY
            {
                k64f get()           { return ::GoLayout_TransformedDataRegionY(Handle); }
            }

            /// <summary>The transformed data region Z-component value.</summary>
            property k64f TransformedDataRegionZ
            {
                k64f get()           { return ::GoLayout_TransformedDataRegionZ(Handle); }
            }

            /// <summary>The transformed data region width value.</summary>
            property k64f TransformedDataRegionWidth
            {
                k64f get()           { return ::GoLayout_TransformedDataRegionWidth(Handle); }
            }

            /// <summary>The transformed data region length value.</summary>
            property k64f TransformedDataRegionLength
            {
                k64f get()           { return ::GoLayout_TransformedDataRegionLength(Handle); }
            }

            /// <summary>The transformed data region height value.</summary>
            property k64f TransformedDataRegionHeight
            {
                k64f get()           { return ::GoLayout_TransformedDataRegionHeight(Handle); }
            }

            /// <summary>The layout specific X spacing count value.</summary>
            property k64f XSpacingCount
            {
                k64f get()           { return ::GoLayout_XSpacingCount(Handle); }
            }

            /// <summary>The layout specific Y spacing count value.</summary>
            property k64f YSpacingCount
            {
                k64f get()           { return ::GoLayout_YSpacingCount(Handle); }
            }

            /// <summary>A boolean value representing whether or not multiplexing is enabled in a buddied configuration.</summary>
            property bool MultiplexBuddyEnabled
            {
                bool get()           { return KToBool(::GoLayout_MultiplexBuddyEnabled(Handle)); }
                void set(bool enable)  { KCheck(::GoLayout_EnableMultiplexBuddy(Handle, enable)); }
            }

            /// <summary>A boolean value representing whether or not multiplexing is enabled in a single device configuration.</summary>
            property bool MultiplexSingleEnabled
            {
                bool get()           { return KToBool(::GoLayout_MultiplexSingleEnabled(Handle)); }
                void set(bool enable)  { KCheck(::GoLayout_EnableMultiplexSingle(Handle, enable)); }
            }

            /// <summary>A value representing the multiplexing delay in a single device sensor configuration.</summary>
            property k64f MultiplexSingleDelay
            {
                k64f get()           { return ::GoLayout_MultiplexSingleDelay(Handle); }
                void set(k64f value)  { KCheck(::GoLayout_SetMultiplexSingleDelay(Handle, value)); }
            }

            /// <summary>A value representing the multiplexing period in a single device sensor configuration.</summary>
            property k64f MultiplexSinglePeriod
            {
                k64f get()           { return ::GoLayout_MultiplexSinglePeriod(Handle); }
                void set(k64f value)  { KCheck(::GoLayout_SetMultiplexSinglePeriod(Handle, value)); }
            }

            /// <summary>A value representing the minimum multiplexing period in a single device sensor configuration.</summary>
            property k64f MultiplexSinglePeriodMin
            {
                k64f get()           { return ::GoLayout_MultiplexSinglePeriodMin(Handle); }
            }

            /// <summary>A value representing the multiplexing exposure duration in a single device sensor configuration.</summary>
            property k64f MultiplexSingleExposureDuration
            {
                k64f get()           { return ::GoLayout_MultiplexSingleExposureDuration(Handle); }
            }

            /// <summary>number of columns being configured (may not be applicable or used) on device in n-buddy system.</summary>
            property k32u GridColumnCount
            {
                k32u get()           { return ::GoLayout_GridColumnCount(Handle); }
                void set(k32u value)  { KCheck(::GoLayout_SetGridColumnCount(Handle, value)); }
            }


            /// <summary>number of columns system value being used on device in n-buddy system.</summary>
            property k32u GridColumnCountSystemValue
            {
                k32u get()           { return ::GoLayout_GridColumnCountSystemValue(Handle); }
            }
       };
    }
}

#endif

// 
// GoRecordingFilter.h
// 
// Copyright (C) 2016-2021 by LMI Technologies Inc.
// Licensed under the MIT License.
// Redistributed files must retain the above copyright notice.
// 
#ifndef GO_SDK_NET_RECORDING_FILTER_H
#define GO_SDK_NET_RECORDING_FILTER_H

#include <GoSdk/GoRecordingFilter.h>
#include <GoSdkNet/GoSdkDef.h>
#include <GoSdkNet/GoReplayCondition.h>

namespace Lmi3d
{
    namespace GoSdk
    {
        /// <summary>Represents a recording filter instance.</summary>
        public ref class GoRecordingFilter : public KObject
        {
            KDeclareClass(GoRecordingFilter, GoRecordingFilter)

            /// <summary>Initializes a new instance of the GoRecordingFilter class with the specified Zen object handle.</summary>
            /// <param name="handle">Zen object handle.</param>
            GoRecordingFilter(IntPtr handle)
                : KObject(handle)
            {}

            /// <summary>Initializes a new instance of the GoRecordingFilter class.</summary>
            /// <param name="sensor">A GoSensor instance.</param>
            GoRecordingFilter(GoSensor^ sensor)
            {
                ::GoRecordingFilter handle = kNULL;

                KCheck(::GoRecordingFilter_Construct(&handle, KToHandle(sensor), kNULL));

                Handle = handle;
            }

            /// <inheritdoc cref="GoAccelerator()" />
            /// <param name="allocator">Memory allocator</param>
            GoRecordingFilter(GoSensor^ sensor, KAlloc^ allocator)
            {
                ::GoRecordingFilter handle = kNULL;

                KCheck(::GoRecordingFilter_Construct(&handle, KToHandle(allocator), kNULL));

                Handle = handle;
            }

            /// <summary>The condition combine type.</summary>
            property GoReplayCombineType ConditionCombineType
            {
                GoReplayCombineType get()           { return (GoReplayCombineType) ::GoRecordingFilter_ConditionCombineType(Handle); }
                void set(GoReplayCombineType conditionCombineType)  { KCheck(::GoRecordingFilter_SetConditionCombineType(Handle, conditionCombineType)); }
            }

            /// <summary>The Any Measurement condition.</summary>
            property GoReplayAnyMeasurement^ AnyMeasurement
            {
                GoReplayAnyMeasurement^ get()           { return KToObject<GoReplayAnyMeasurement^>(::GoRecordingFilter_AnyMeasurement(Handle)); }
            }

            /// <summary>The Any Data condition.</summary>
            property GoReplayAnyData^ AnyData
            {
                GoReplayAnyData^ get()           { return KToObject<GoReplayAnyData^>(::GoRecordingFilter_AnyData(Handle)); }
            }

            /// <summary>The Measurement condition.</summary>
            property GoReplayMeasurement^ Measurement
            {
                GoReplayMeasurement^ get()           { return KToObject<GoReplayMeasurement^>(::GoRecordingFilter_Measurement(Handle)); }
            }
        };
    }
}

#endif

// 
// GoReplay.h
// 
// Copyright (C) 2016-2021 by LMI Technologies Inc.
// Licensed under the MIT License.
// Redistributed files must retain the above copyright notice.
// 
#ifndef GO_SDK_NET_REPLAY_H
#define GO_SDK_NET_REPLAY_H

#include <GoSdk/GoReplay.h>
#include <GoSdkNet/GoSdkDef.h>
#include <GoSdkNet/GoRecordingFilter.h>

namespace Lmi3d
{
    namespace GoSdk
    {
        /// <summary>Represents a replay instance.</summary>
        public ref class GoReplay : public KObject
        {
            KDeclareClass(GoReplay, GoReplay)

            /// <summary>Initializes a new instance of the GoReplay class with the specified Zen object handle.</summary>
            /// <param name="handle">Zen object handle.</param>
            GoReplay(IntPtr handle)
                : KObject(handle)
            {}

            /// <summary>Initializes a new instance of the GoReplay class.</summary>
            /// <param name="sensor">A GoSensor instance.</param>
            GoReplay(GoSensor^ sensor)
            {
                ::GoReplay handle = kNULL;

                KCheck(::GoReplay_Construct(&handle, KToHandle(sensor), kNULL));

                Handle = handle;
            }

            /// <inheritdoc cref="GoAccelerator()" />
            /// <param name="allocator">Memory allocator</param>
            GoReplay(GoSensor^ sensor, KAlloc^ allocator)
            {
                ::GoReplay handle = kNULL;

                KCheck(::GoReplay_Construct(&handle, KToHandle(sensor), KToHandle(allocator)));

                Handle = handle;
            }

            /// <summary>The recording filter for the replay.</summary>
            property GoRecordingFilter^ RecordingFilter
            {
                GoRecordingFilter^ get()    { return KToObject<GoRecordingFilter^>(::GoReplay_RecordingFilter(Handle)); }
            }
        };
    }
}

#endif

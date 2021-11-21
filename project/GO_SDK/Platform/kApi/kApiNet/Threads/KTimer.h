// 
// KTimer.h
// 
// Copyright (C) 2014-2021 by LMI Technologies Inc.
// Licensed under the MIT License.
// Redistributed files must retain the above copyright notice.
// 
#ifndef K_API_NET_TIMER_H
#define K_API_NET_TIMER_H

#include <kApi/Threads/kTimer.h>
#include "kApiNet/KAlloc.h"

namespace Lmi3d
{
    namespace Zen 
    {
        namespace Threads
        {
            /// <summary>Represents an interval timer. Requires manual disposal.</summary>
            ///
            /// <remarks>
            /// <para>Default KRefStyle: Auto</para>
            /// </remarks>
            public ref class KTimer : public KObject
            {
                KDeclareAutoClass(KTimer, kTimer)

            public:
                /// <summary>Initializes a new instance of the KTimer class with the specified Zen object handle.</summary>           
                /// <param name="handle">Zen object handle.</param>
                KTimer(IntPtr handle)
                    : KObject(handle, DefaultRefStyle)
                {}  

                /// <inheritdoc cref="KTimer()" />
                ///
                /// <param name="refStyle">Ref style.</param>
                KTimer(IntPtr handle, KRefStyle refStyle)
                    : KObject(handle, refStyle)
                {}

                /// <summary>Initializes a new instance of the KTimer class.</summary>   
                KTimer()
                    : KObject(DefaultRefStyle)
                {
                    kTimer handle = kNULL;

                    KCheck(kTimer_Construct(&handle, kNULL));

                    Handle = handle;
                }

                /// <inheritdoc cref="KTimer()" />
                /// <param name="allocator">Memory allocator.</param>
                KTimer(KAlloc^ allocator)
                    : KObject(DefaultRefStyle)
                {
                    kTimer handle = kNULL;

                    KCheck(kTimer_Construct(&handle, KToHandle(allocator)));

                    Handle = handle;
                }

                /// <inheritdoc cref="KTimer(KAlloc^)" />
                ///
                /// <param name="refStyle">Ref style.</param>
                KTimer(KAlloc^ allocator, KRefStyle refStyle)
                    : KObject(refStyle)
                {
                    kTimer handle = kNULL;

                    KCheck(kTimer_Construct(&handle, KToHandle(allocator)));

                    Handle = handle;
                }

                /// <summary>Provides the current time in microseconds.</summary>
                /// 
                /// <remarks>
                /// This property provides a time value in microseconds; successive calls to this property can be
                /// used to measure a time interval.
                /// </remarks>
                static property k64s Now
                {
                    k64s get() { return (k64s)kTimer_Now(); }
                }

                /// <summary>Starts the timer with a countdown amount.</summary>
                /// 
                /// <param name="totalTime">Total time to count down, in microseconds.</param>
                void Start(k64s totalTime)
                {
                    KCheck(kTimer_Start(Handle, (k64u)totalTime));
                }

                /// <summary>Starts the timer.</summary>
                void Start()
                {
                    KCheck(kTimer_Start(Handle, 0)); 
                }

                /// <summary>Stops the timer.</summary>
                /// 
                /// <remarks>
                /// KTimer.Elapsed can be used to report the time between start and stop.
                /// </remarks>
                /// 
                /// The use of KTimer.Stop is strictly optional; it is not necessary to call KTimer.Stop for each
                /// invocation of KTimer.Start.
                void Stop()
                {
                    KCheck(kTimer_Stop(Handle)); 
                }

                /// <summary>Reports whether a timer has been started.</summary>
                property bool IsStarted
                {
                    bool get() { return KToBool(kTimer_IsStarted(Handle)); }
                }

                /// <summary>Reports whether a count-down timer has expired.</summary>
                property bool IsExpired
                {
                    bool get() { return KToBool(kTimer_IsExpired(Handle)); }
                }

                /// <summary>Gets the duration, in microseconds, for which the timer has been running.</summary>
                property k64s Elapsed
                {
                    k64s get() { return (k64s)kTimer_Elapsed(Handle); }
                }

                /// <summary>Gets the remaining time, in microseconds, for a countdown timer.</summary>
                property k64s Remaining
                {
                    k64s get() { return (k64s)kTimer_Remaining(Handle); }
                }
            };

        }
    }
}

#endif

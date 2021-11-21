// 
// KPeriodic.h
// 
// Copyright (C) 2014-2021 by LMI Technologies Inc.
// Licensed under the MIT License.
// Redistributed files must retain the above copyright notice.
// 
#ifndef K_API_NET_PERIODIC_H
#define K_API_NET_PERIODIC_H

#include <kApi/Threads/kPeriodic.h>
#include "kApiNet/KAlloc.h"

namespace Lmi3d
{
    namespace Zen 
    {
        namespace Threads
        {
            /// <summary>A delegate that matches the signature of kCallbackFx from the underlying Zen library.</summary>
            private delegate kStatus KPeriodicElapsedFx(kPointer receiver, kPointer timer);

            /// <summary>Provides a periodic method call. <para/> Requires manual disposal.</summary>
            public ref class KPeriodic : public KObject
            {
                KDeclareClass(KPeriodic, kPeriodic)

            public:
                /// <summary>Initializes a new instance of the KPeriodic class with the specified Zen object handle.</summary>           
                /// <param name="handle">Zen object handle.</param>
                KPeriodic(IntPtr handle)
                    : KObject(handle, DefaultRefStyle)
                {}

                /// <summary>Initializes a new instance of the KPeriodic class.</summary>           
                KPeriodic()
                    : KObject(DefaultRefStyle)
                {
                    kPeriodic handle = kNULL;

                    KCheck(kPeriodic_Construct(&handle,  kNULL));

                    Handle = handle;
                }

                /// <inheritdoc cref="KPeriodic()" />
                /// <param name="allocator">Memory allocator.</param>
                KPeriodic(KAlloc^ allocator)
                    : KObject(DefaultRefStyle)
                {
                    kPeriodic handle = kNULL;

                    KCheck(kPeriodic_Construct(&handle, KToHandle(allocator)));

                    Handle = handle;
                }

                /// <summary>Starts callbacks at the specified period.</summary>
                /// 
                /// <remarks>
                /// <para>It is safe to call this method from within a timer callback. It is valid to call Start multiple
                /// times without first calling Stop, in order to change the timer period or callback method.</para>
                /// 
                /// <para>Each subsequent timer period after the first callback is measured relative to the end
                /// of the previous timer callback.</para>
                /// 
                /// <para>The registered handler must be unregistered when no longer needed, else the underlying timer object
                /// will retain a reference to it, possibly resulting in a CLR object leak. The handler will be unregistered 
                /// when KPeriodic.Stop is called, or if KObject.Dispose is used to destroy the timer.</para>
                /// </remarks>
                ///
                /// <param name="period">Callback period, in microseconds.</param>
                /// <param name="handler">Callback handler.</param>
                void Start(k64s period, Action^ handler)
                {
                    kStatus status;

                    KCheckArgs(handler != nullptr);

                    Stop(); 

                    KPeriodicElapsedFx^ thunk = gcnew KPeriodicElapsedFx(this, &KPeriodic::OnTimer);
                    KCallbackState^ context = gcnew KCallbackState(thunk, handler);

                    if (!kSuccess(status = kPeriodic_Start(Handle, (k64u)period, (kPeriodicElapsedFx)context->NativeFunction, context->NativeContext)))
                    {
                        delete context; 
                        throw gcnew KException(status);
                    }
                }

                /// <summary>Stops timer callbacks.</summary>
                /// 
                /// <remarks>
                /// It is guaranteed that callbacks are stopped when this method returns. Deadlock can occur
                /// if a timer callback is in progress and is blocked indefinitely. It is safe to call this method
                /// from within a timer callback.
                /// </remarks>
                void Stop()
                {
                    if (kPeriodic_Enabled(Handle))
                    {
                        kPointer nativeContext = xkPeriodic_HandlerContext(Handle);

                        KCheck(kPeriodic_Stop(Handle));

                        KCallbackState::Dispose(nativeContext);
                    }
                }

                /// <summary>Reports whether periodic timer callbacks are currently enabled.</summary>
                /// 
                /// <remarks>
                /// Deadlock can occur if a timer callback is in progress and is blocked indefinitely when this
                /// property is accessed. It is safe to access this property from within a timer callback.
                /// </remarks>
                property bool Enabled
                {
                    bool get() { return KToBool(kPeriodic_Enabled(Handle)); }
                }

            protected:

                //ensures that callbacks are unhooked
                virtual void OnDisposing() override
                {
                    Stop();
                }

            private:

                kStatus OnTimer(kPointer receiver, kPointer timer)
                {
                    kStatus status = kOK;

                    try
                    {
                        KCallbackState^ context = KCallbackState::FromNativeContext(receiver);
                        Action^ handler = (Action^) context->Handler;

                        handler();
                    }
                    catch (KException^ e)
                    {
                        status = e->Status;
                    }
                    catch (...)
                    {
                        status = kERROR;
                    }

                    return status;
                }
            };

        }
    }
}

#endif

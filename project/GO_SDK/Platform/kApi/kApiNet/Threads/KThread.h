// 
// KThread.h
// 
// Copyright (C) 2014-2021 by LMI Technologies Inc.
// Licensed under the MIT License.
// Redistributed files must retain the above copyright notice.
// 
#ifndef K_API_NET_THREAD_H
#define K_API_NET_THREAD_H

#include <kApi/Threads/kThread.h>
#include "kApiNet/KAlloc.h"

namespace Lmi3d
{
    namespace Zen 
    {
        namespace Threads
        {
            /// <summary>A delegate that matches the signature of kThreadFx from the underlying Zen library.</summary>
            
            private delegate kStatus KThreadFx(kPointer receiver);

            /// <summary>Represents a thread. <para/> Requires manual disposal.</summary>
            public ref class KThread : public KObject
            {
                KDeclareClass(KThread, kThread)

            public:
                /// <summary>Initializes a new instance of the KThread class with the specified Zen object handle.</summary>           
                /// <param name="handle">Zen object handle.</param>
                KThread(IntPtr handle)
                    : KObject(handle, DefaultRefStyle)
                {}             

                /// <summary>Initializes a new instance of the KThread class.</summary>   
                KThread()
                    : KObject(DefaultRefStyle)
                {
                    kThread handle = kNULL;

                    KCheck(kThread_Construct(&handle, kNULL));

                    Handle = handle;
                }

                /// <inheritdoc cref="KThread()" />
                /// <param name="allocator">Memory allocator.</param>
                KThread(KAlloc^ allocator)
                    : KObject(DefaultRefStyle)
                {
                    kThread handle = kNULL;

                    KCheck(kThread_Construct(&handle, KToHandle(allocator)));

                    Handle = handle;
                }

                /// <summary>Causes the current thread to yield control for approximately the specified duration.</summary>
                /// 
                /// <remarks>
                /// <para>The duration specified in this method is approximate. Sleep duration is governed by the underlying
                /// operating system, and may be affected by kernel timer resolution or thread scheduling policies. In general,
                /// if CPU utilization is low (quiet system), then sleep duration is most often +/- 1 kernel timer tick. At the
                /// time of this writing, most desktop operating systems have a 10 ms kernel timer resolution, while most
                /// embedded systems have a 1-10 ms kernel timer resolution.</para>
                /// 
                /// <para>If sleep duration must be constrained to be <em>at least</em> the specified duration, consider using
                /// kThread_SleepAtLeast.</para>
                /// </remarks>
                /// 
                /// <param name="duration">Approximate time to sleep, in microseconds.</param>
                static void Sleep(k64s duration)
                {
                    KCheck(kThread_Sleep((k64u)duration));
                }

                /// <summary>Causes the current thread to yield control for at least the specified duration.</summary>
                /// 
                /// <remarks>
                /// This method sleeps until at least the specified duration has elapsed. Consider using this function
                /// when a sleep operation must provide a guaranteed minimum hold time.
                /// </remarks>
                /// 
                /// <param name="duration">Minimum time to sleep, in microseconds.</param>
                static void SleepAtLeast(k64s duration)
                {
                    KCheck(kThread_SleepAtLeast((k64u)duration));
                }

                /// <summary>Begins executing a thread using the specified callback method.</summary>
                /// 
                /// <param name="handler">The thread entry handler.</param>
                void Start(Action^ handler)
                {
                    kStatus status;

                    KCheckArgs(handler != nullptr);
                    
                    KThreadFx^ thunk = gcnew KThreadFx(this, &KThread::Entry);
                    KCallbackState^ session = gcnew KCallbackState(thunk, handler);

                    if (!kSuccess(status = kThread_Start(Handle, (kThreadFx)session->NativeFunction, session->NativeContext)))
                    {
                        delete session;
                        throw gcnew KException(status);
                    }
                }

                /// <summary>Blocks until the thread exits, or until a timeout occurs.</summary>
                /// 
                /// <param name="timeout">Timeout in microseconds.</param>
                /// <returns>Status code representing the result from thread execution.</returns>
                /// <exception cref="KException">Thrown if a timeout occurrs.</exception>
                KStatus Join(k64s timeout)
                {
                    KStatus exitCode; 

                    KCheck(JoinEx(timeout, exitCode));

                    return exitCode; 
                }

                /// <summary>Blocks until the thread exits, or until a timeout occurs.</summary>
                /// 
                /// <param name="timeout">Timeout in microseconds.</param>
                /// <returns>true if the thread was joined; otherwise false.</returns>
                bool TryJoin(k64s timeout)
                {
                    KStatus exitCode;

                    return KToBool(kSuccess(JoinEx(timeout, exitCode)));
                }

                /// <summary>Blocks until the thread exits, or until a timeout occurs.</summary>
                /// 
                /// <param name="timeout">Timeout in microseconds.</param>
                /// <param name="exitCode">Status code representing the result from thread execution.</param>
                /// <returns>true if the thread was joined; otherwise false.</returns>
                bool TryJoin(k64s timeout, [Out] KStatus% exitCode)
                {
                    return KToBool(kSuccess(JoinEx(timeout, exitCode))); 
                }

            protected:

                //ensures that callbacks are unhooked
                virtual void OnDisposing() override
                {
                    Join(KTimeout::Infinite); 
                }

            private:

                kStatus Entry(kPointer receiver)
                {
                    kStatus status = kOK;

                    try
                    {
                        KCallbackState^ context = KCallbackState::FromNativeContext(receiver);
                        Action^ handler = (Action^)context->Handler;
                        
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

                KStatus JoinEx(k64s timeout, [Out] KStatus% exitCode)
                {
                    kStatus code = kOK; 

                    if (xkThread_Handler(Handle))
                    {
                        kPointer nativeContext = xkThread_HandlerContext(Handle);

                        kCheck(kThread_Join(Handle, (k64u)timeout, &code));

                        KCallbackState::Dispose(nativeContext); 
                    }

                    exitCode = code; 

                    return kOK; 
                }
            };

        }
    }
}

#endif

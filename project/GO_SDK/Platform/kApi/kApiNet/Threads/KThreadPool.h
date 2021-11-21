// 
// KThreadPool.h
// 
// Copyright (C) 2017-2021 by LMI Technologies Inc.
// Licensed under the MIT License.
// Redistributed files must retain the above copyright notice.
//  
#ifndef K_API_NET_THREAD_POOL_H
#define K_API_NET_THREAD_POOL_H

#include <kApi/Threads/kThreadPool.h>

namespace Lmi3d
{
    namespace Zen
    {
        namespace Threads
        {
            /// <summary>kThreadPool class allows to operate with a fixed numbers of threads.</summary>
            public ref class KThreadPool : public KObject
            {
                KDeclareAutoClass(KThreadPool, kThreadPool)

            public:
                /// <summary>Initializes a new instance of the KThreadPool class with the specified Zen object handle.</summary>           
                /// <param name="handle">Zen object handle.</param>
                KThreadPool(IntPtr handle)
                    : KObject(handle, DefaultRefStyle)
                {}

                /// <inheritdoc cref="KThreadPool(IntPtr)" />
                ///
                /// <param name="refStyle">RefStyle for this object.</param>
                KThreadPool(IntPtr handle, KRefStyle refStyle)
                    : KObject(handle, refStyle)
                {}

                /// <summary>Constructs a kThreadPool object.</summary>           
                /// <param name="threadCount">Number of threads in the pool.</param>
                KThreadPool(kSize threadCount)
                {
                    kThreadPool handle = kNULL;

                    KCheck(kThreadPool_Construct(&handle, threadCount, kNULL));

                    Handle = handle;
                }

                property k64s Count
                {
                    k64s get()
                    {
                        return kThreadPool_Count(Handle);
                    }
                }

                void BeginExecute(Action^ handler, [Out] KPointer% transaction)
                {
                    kStatus status;
                    kPointer t = kNULL;

                    KCheckArgs(handler != nullptr);

                    KThreadFx^ thunk = gcnew KThreadFx(this, &KThreadPool::Entry);
                    KCallbackState^ session = gcnew KCallbackState(thunk, handler);
                    
                    status = kThreadPool_BeginExecute(Handle, (kThreadFx)session->NativeFunction, session->NativeContext, &t);

                    if (!kSuccess(status))
                    {
                        delete session;
                        throw gcnew KException(status);
                    }

                    transaction = KPointer(t);
                }

                kStatus EndExecute(KPointer transaction, k64s timeout)
                {
                    kStatus status;

                    KCheck(kThreadPool_EndExecute(Handle, (kPointer)transaction, (k64u)timeout, &status));

                    return status;
                }

                void BeginExecute(Action^ handler)
                {
                    kStatus status;

                    KCheckArgs(handler != nullptr);

                    KThreadFx^ thunk = gcnew KThreadFx(this, &KThreadPool::Entry);
                    KCallbackState^ session = gcnew KCallbackState(thunk, handler);

                    status = kThreadPool_BeginExecute(Handle, (kThreadFx)session->NativeFunction, session->NativeContext, nullptr);

                    if (!kSuccess(status))
                    {
                        delete session;
                        throw gcnew KException(status);
                    }
                }

                static property KThreadPool^ Default
                {
                    KThreadPool^ get()
                    {
                        kThreadPool pool = kThreadPool_Default();
                        return KToObject<KThreadPool^>(pool, KRefStyle::None);
                    }
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
            };
        }
    }
}

#endif

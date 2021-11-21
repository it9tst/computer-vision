// 
// KParallel.h
// 
// Copyright (C) 2017-2021 by LMI Technologies Inc.
// Licensed under the MIT License.
// Redistributed files must retain the above copyright notice.
//  
#ifndef K_API_NET_PARALLEL_H
#define K_API_NET_PARALLEL_H

#include <kApi/Threads/kParallel.h>
#include "kApiNet/KAlloc.h"

namespace Lmi3d
{
    namespace Zen
    {
        namespace Threads
        {
            /// <summary>KParallel class allows to operate on some data using a fixed numbers of threads.</summary>
            public ref class KParallel : public KObject
            {
                KDeclareAutoClass(KParallel, kParallel)

            public:
                /// <summary>Initializes a new instance of the KParallel class with the specified Zen object handle.</summary>           
                /// <param name="handle">Zen object handle.</param>
                KParallel(IntPtr handle)
                    : KObject(handle, DefaultRefStyle)
                {}

                /// <inheritdoc cref="KParallel(IntPtr)" />
                ///
                /// <param name="refStyle">RefStyle for this object.</param>
                KParallel(IntPtr handle, KRefStyle refStyle)
                    : KObject(handle, refStyle)
                {}

                /// <summary> Helper method that can be used to calculate the data start index within the callback.</summary> 
                ///
                /// <remarks>
                /// <para>The kParallel class enables data processing to be distributed across multiple callbacks. This
                /// arrangement requires the dataset to be partitioned, with each callback performing a portion of the
                /// total required work.</para>
                ///
                /// <para>Assuming that the dataset can be represented by a linear range(e.g., elements in an array), the
                /// BeginIndex method can optionally be used to calculate the lower bound(start index) of the
                /// data to be processed in the current callback invocation.</para>
                /// </remarks>
                /// 
                /// <param name="threadCount">The total thread count.</param>
                /// <param name="index">Index of current processing callback (0..threadCount-1).</param>
                /// <param name="start">First index of the overall range to be collectively processed.</param>
                /// <param name="elementCount">Total count of elements to be collectively processed</param>
                /// <returns>Index of first element to be processed in this callback.</returns>
                static KSize BeginIndex(KSize threadCount, KSize index, KSize start, KSize elementCount)
                {
                    kParallelArgs args;
                    args.count = threadCount;
                    args.index = index;

                    return kParallelArgs_Begin(&args, start, elementCount);
                }

                /// <summary> Helper method that can be used to calculate the data end index within the callback. </summary>
                ///
                /// <remarks>
                /// <para>The kParallel class enables data processing to be distributed across multiple callbacks.This
                /// arrangement requires the dataset to be partitioned, with each callback performing a portion of the
                /// total required work.</para>
                /// 
                /// <para>Assuming that the dataset can be represented by a linear range(e.g., elements in an array), the
                /// EndIndex method can optionally be used to calculate the upper bound(end index) of the
                /// data to be processed in the current kParallel callback invocation.</para>
                /// </remarks>
                /// 
                /// <param name="threadCount">The total thread count.</param>
                /// <param name="index">Index of current processing callback (0..threadCount-1).</param>
                /// <param name="start">First index of the overall range to be collectively processed.</param>
                /// <param name="elementCount">Total count of elements to be collectively processed</param>
                /// <returns>One greater than the index of the last element to be processed in this callback.</returns>
                static KSize EndIndex(KSize threadCount, KSize index, KSize start, KSize elementCount)
                {
                    kParallelArgs args;
                    args.count = threadCount;
                    args.index = index;

                    return kParallelArgs_End(&args, start, elementCount);
                }
                
                /// <summary> Begins asynchronously processing a dataset. </summary>
                ///
                /// <remarks>
                /// <para>This function returns immediately, providing a transaction handle to represent the in-progress data processing. 
                /// The EndExecute function must be used to wait for execution to complete. Failure to call
                /// EndExecute will result in leaks.</para>
                /// 
                /// <para>This method is thread-safe.</para>
                /// </remarks>
                /// 
                /// <param name="callbackFx">Data processing callback function.</param>
                /// <param name="content">Content to be provided to callback.</param>
                /// <param name="transaction">Receives transaction handle, which must be passed to EndExecute.</param>
                static void BeginExecute(Action<KSize, KSize, KPointer>^callbackFx, IntPtr content, [Out] KPointer% transaction)
                {
                    kPointer t = kNULL;
                    KCallbackFx^ thunk = gcnew KCallbackFx(&KParallel::CallbackHandler);
                    KCallbackState^ context = gcnew KCallbackState(thunk, callbackFx);

                    KCheck(kParallel_BeginExecute((kParallelFx)context->NativeFunction, context->NativeContext, content.ToPointer(), &t));

                    transaction = KPointer(t);
                }

                /// <summary>Blocks until execution of the specified kParallel transaction is complete. </summary>
                ///
                /// <remarks>
                /// This method is thread-safe.
                /// </remarks>
                /// 
                /// <param name="transaction">Transaction handle emitted by BeginExecute.</param>
                static kStatus EndExecute(KPointer transaction)
                {
                    return kParallel_EndExecute((kPointer)transaction);
                }
                

            private:
                static kStatus CallbackHandler(kPointer receiver, kPointer sender, kPointer parallelArgs)
                {
                    kStatus status = kOK;

                    try
                    {
                        KCallbackState^ context = KCallbackState::FromNativeContext(receiver);
                        Action<KSize, KSize, KPointer>^ handler = (Action<KSize, KSize, KPointer>^) context->Handler;
                        kParallelArgs* args = (kParallelArgs*)parallelArgs;

                        handler(args->index, args->count, KPointer(args->content));
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

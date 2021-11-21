// 
// KSemaphore.h
// 
// Copyright (C) 2014-2021 by LMI Technologies Inc.
// Licensed under the MIT License.
// Redistributed files must retain the above copyright notice.
// 
#ifndef K_API_NET_SEMAPHORE_H
#define K_API_NET_SEMAPHORE_H

#include <kApi/Threads/kSemaphore.h>
#include "kApiNet/KAlloc.h"

namespace Lmi3d
{
    namespace Zen 
    {
        namespace Threads
        {
            /// <summary>Represents a semaphore. <para/> Requires manual disposal.</summary>
            public ref class KSemaphore : public KObject
            {
                KDeclareClass(KSemaphore, kSemaphore)

            public:
                /// <summary>Initializes a new instance of the KSemaphore class with the specified Zen object handle.</summary>           
                /// <param name="handle">Zen object handle.</param>
                KSemaphore(IntPtr handle)
                    : KObject(handle, DefaultRefStyle)
                {}
              
                /// <summary>Initializes a new instance of the KSemaphore class.</summary>         
                KSemaphore()
                    : KObject(DefaultRefStyle)
                {
                    kSemaphore handle = kNULL;

                    KCheck(kSemaphore_Construct(&handle, 0, kNULL));

                    Handle = handle;
                }

                /// <inheritdoc cref="KSemaphore()" />
                /// <param name="initialCount">Initial value of the semaphore.</param>
                KSemaphore(k64s initialCount)
                    : KObject(DefaultRefStyle)
                {
                    kSemaphore handle = kNULL;

                    KCheck(kSemaphore_Construct(&handle, (kSize)initialCount, kNULL));

                    Handle = handle;
                }

                /// <inheritdoc cref="KSemaphore(k64s)" />
                /// <param name="allocator">Memory allocator.</param>
                KSemaphore(k64s initialCount, KAlloc^ allocator)
                    : KObject(DefaultRefStyle)
                {
                    kSemaphore handle = kNULL;

                    KCheck(kSemaphore_Construct(&handle, (kSize)initialCount, KToHandle(allocator)));

                    Handle = handle;
                }

                /// <summary>Increments the semaphore.</summary>
                void Post()
                {
                    KCheck(kSemaphore_Post(Handle)); 
                }


                /// <summary>Waits until the semaphore can be decremented or the timeout interval has elapsed.</summary>
                /// 
                /// <param name="timeout">Timeout in microseconds.</param>
                /// <returns>true if the semaphore was decremented; otherwise false.</returns>
                bool Wait(k64s timeout)
                {
                    kStatus result = kSemaphore_Wait(Handle, (k64u)timeout); 

                    switch (result)
                    {
                    case kOK:               return true; 
                    case kERROR_TIMEOUT:    return false; 
                    default:                throw gcnew KException(result); 
                    }
                }
            };

        }
    }
}

#endif

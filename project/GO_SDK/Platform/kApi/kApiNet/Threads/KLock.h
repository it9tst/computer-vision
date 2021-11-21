// 
// KLock.h
// 
// Copyright (C) 2014-2021 by LMI Technologies Inc.
// Licensed under the MIT License.
// Redistributed files must retain the above copyright notice.
// 
#ifndef K_API_NET_LOCK_H
#define K_API_NET_LOCK_H

#include <kApi/Threads/kLock.h>
#include "kApiNet/KAlloc.h"

namespace Lmi3d
{
    namespace Zen 
    {
        namespace Threads
        {
            /// <summary> Represents a recursive, mutual exclusion lock. <para/> Requires manual disposal.</summary>
            public ref class KLock : public KObject
            {
                KDeclareClass(KLock, kLock)

            public:
                /// <summary>Initializes a new instance of the KLock class with the specified Zen object handle.</summary>           
                /// <param name="handle">Zen object handle.</param>
                KLock(IntPtr handle)
                    : KObject(handle, DefaultRefStyle)
                {}

                /// <summary>Initializes a new instance of the KLock class.</summary>        
                KLock()
                    : KObject(DefaultRefStyle)
                {
                    kLock handle = kNULL;

                    KCheck(kLock_Construct(&handle, kNULL));

                    Handle = handle;
                }

                /// <inheritdoc cref="KLock()" />
                /// <param name="allocator">Memory allocator.</param>
                KLock(KAlloc^ allocator)
                    : KObject(DefaultRefStyle)
                {
                    kLock handle = kNULL;

                    KCheck(kLock_Construct(&handle, KToHandle(allocator)));

                    Handle = handle;
                }

                /// <summary>Initializes a new instance of the KLock class.</summary>
                ///
                ///<remarks>
                /// If isWaitable is true, KLock.Enter can be used with a timeout 
                /// to wait for the lock to become available.
                ///</remarks>
                /// <param name="isWaitable">Should lock support waiting?</param>
                KLock(KBool isWaitable)
                    : KObject(DefaultRefStyle)
                {
                    kLock handle = kNULL;

                    KCheck(kLock_ConstructEx(&handle, (isWaitable) ? kLOCK_OPTION_TIMEOUT : kLOCK_OPTION_NONE, kNULL));

                    Handle = handle;
                }

                /// <inheritdoc cref="KLock()" />
                /// <param name="allocator">Memory allocator.</param>
                KLock(KBool isWaitable, KAlloc^ allocator)
                    : KObject(DefaultRefStyle)
                {
                    kLock handle = kNULL;

                    KCheck(kLock_ConstructEx(&handle, (isWaitable) ? kLOCK_OPTION_TIMEOUT : kLOCK_OPTION_NONE, KToHandle(allocator)));

                    Handle = handle;
                }

                /// <summary>Blocks until exclusive ownership of the lock is acquired.</summary>
                /// 
                /// <remarks>
                /// KLock ownership is recursive - the lock is relinquished when the number of calls
                /// to KLock.Exit equals the number of calls to KLock.Enter.
                /// </remarks>
                void Enter()
                {
                    KCheck(kLock_Enter(Handle)); 
                }

                /// <summary>Blocks until exclusive ownership of the lock is acquired or the timeout interval has elapsed.</summary>
                /// 
                /// <remarks>
                /// KLock ownership is recursive - the lock is relinquished when the number of calls
                /// to KLock.Exit equals the number of calls to KLock.Enter.
                /// </remarks>
                /// <param name="timeout">Timeout in microseconds.</param>
                void Enter(K64u timeout)
                {
                    KCheck(kLock_EnterEx(Handle, timeout));
                }

                /// <summary>Relinquishes ownership of the lock.</summary>
                /// 
                /// <remarks>
                /// KLock ownership is recursive - the lock is relinquished when the number of calls
                /// to KLock.Exit equals the number of calls to KLock.Enter.
                /// </remarks>
                void Exit()
                {
                    KCheck(kLock_Exit(Handle));
                }

            };
        }
    }
}

#endif

// 
// KAtomic.h
// 
// Copyright (C) 2014-2021 by LMI Technologies Inc.
// Licensed under the MIT License.
// Redistributed files must retain the above copyright notice.
// 
#ifndef K_API_NET_ATOMIC_H
#define K_API_NET_ATOMIC_H

#include <kApi/Threads/kAtomic.h>
#include "kApiNet/KApiDef.h"

namespace Lmi3d
{
    namespace Zen 
    {
        namespace Threads
        {
            /// <summary>Represents a 32-bit, atomically-accessed, signed integer.</summary>
            public value struct KAtomic32s
            {
            public:

                /// <summary>Initializes a new KAtomic32s instance with the specified value.</summary>
                KAtomic32s(int value)
                    : m_atomic(value)
                { }

                /// <summary>Increments the atomic variable.</summary>
                /// 
                /// <remarks>
                /// This method is thread-safe. Implements a full memory barrier.
                /// </remarks>
                /// 
                /// <returns>New atomic value.</returns>
                k32s Increment()
                {
                    pin_ptr<k32s> pinnedValue = &m_atomic;

                    return kAtomic32s_Increment((kAtomic32s*)pinnedValue);
                }

                /// <summary>Decrements the atomic variable.</summary>
                /// 
                /// <remarks>
                /// This method is thread-safe. Implements a full memory barrier.
                /// </remarks>
                /// 
                /// <returns>New atomic value.</returns>
                k32s Decrement()
                {
                    pin_ptr<k32s> pinnedValue = &m_atomic;

                    return kAtomic32s_Decrement((kAtomic32s*)pinnedValue);
                }

                /// <summary>Exchanges the value of the atomic variable.</summary>
                /// 
                /// <remarks>
                /// This method is thread-safe. Implements a full memory barrier.
                /// </remarks>
                /// 
                /// <param name="value">New atomic value.</param>
                /// <returns>Previous atomic value.</returns>
                k32s Exchange(k32s value)
                {
                    pin_ptr<k32s> pinnedValue = &m_atomic;

                    return kAtomic32s_Exchange((kAtomic32s*)pinnedValue, value);
                }

                /// <summary>Conditionally exchanges the value of the atomic variable.</summary>
                /// 
                /// <remarks>
                /// If the atomic value is equal to the oldValue argument, then it is replaced by the value argument.
                /// 
                /// This method is thread-safe. Implements a full memory barrier.
                /// </remarks>
                /// 
                /// <param name="oldValue">Previous atomic value.</param>
                /// <param name="value">New atomic value.</param>
                /// <returns>true if the exchange succeeded.</returns>
                bool CompareExchange(k32s oldValue, k32s value)
                {
                    pin_ptr<k32s> pinnedValue = &m_atomic;

                    return KToBool(kAtomic32s_CompareExchange((kAtomic32s*)pinnedValue, oldValue, value)); 
                }

                /// <summary>Gets the current value of the atomic variable.</summary>
                /// 
                /// <remarks>
                /// This method is thread-safe. Implements a full memory barrier.
                /// </remarks>
                /// 
                /// <returns>Atomic value.</returns>
                k32s Get()
                {
                    pin_ptr<k32s> pinnedValue = &m_atomic;

                    return kAtomic32s_Get((kAtomic32s*)pinnedValue);
                }

            private:
                k32s m_atomic;
            };

            /// <summary>Represents an atomically-accessed pointer.</summary>
            public value struct KAtomicPointer
            {
            public:

                /// <summary>Initializes a new KAtomicPointer instance with the specified value.</summary>
                KAtomicPointer(IntPtr value)
                    : m_atomic(value.ToPointer())
                { }

                /// <summary>Exchanges the value of the atomic pointer.</summary>
                /// 
                /// <remarks>
                /// This method is thread-safe. Implements a full memory barrier.
                /// </remarks>
                /// 
                /// <param name="value">New atomic value.</param>
                /// <returns>Previous atomic value.</returns>
                IntPtr Exchange(IntPtr value)
                {
                    pin_ptr<kPointer> pinnedValue = &m_atomic;

                    return IntPtr(kAtomicPointer_Exchange((kAtomicPointer*)pinnedValue, value.ToPointer()));
                }

                /// <summary>Conditionally exchanges the value of the atomic pointer.</summary>
                /// 
                /// <remarks>
                /// <para>If the atomic value is equal to the oldValue argument, then it is replaced by the value argument.</para>
                /// 
                /// <para>This method is thread-safe. Implements a full memory barrier.</para>
                /// </remarks>
                /// 
                /// <param name="oldValue">Previous atomic value.</param>
                /// <param name="value">New atomic value.</param>
                /// <returns>Previous atomic value.</returns>
                bool CompareExchange(IntPtr oldValue, IntPtr value)
                {
                    pin_ptr<kPointer> pinnedValue = &m_atomic;

                    return KToBool(kAtomicPointer_CompareExchange((kAtomicPointer*)pinnedValue, oldValue.ToPointer(), value.ToPointer()));
                }              

                /// <summary>Gets the current value of the atomic pointer.</summary>
                /// 
                /// <remarks>
                /// This method is thread-safe. Implements a full memory barrier.
                /// </remarks>
                /// 
                /// <returns>Atomic value.</returns>
                IntPtr Get()
                {
                    pin_ptr<kPointer> pinnedValue = &m_atomic;

                    return IntPtr(kAtomicPointer_Get((kAtomicPointer*)pinnedValue)); 
                }
            private:
                kPointer m_atomic;
            };
        }
    }
}

#endif

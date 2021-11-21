/** 
 * @file    kAtomic.cpp
 *
 * @internal
 * Copyright (C) 2012-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#define K_PLATFORM
#include <kApi/Threads/kAtomic.h>

kBeginStaticClassEx(k, kAtomic)
kEndStaticClassEx()

kFx(kStatus) xkAtomic_InitStatic()
{
    return kOK;
}

kFx(kStatus) xkAtomic_ReleaseStatic()
{
    return kOK; 
}

//The GCC 4.3.3 compiler shipped with VxWorks 6.9 doesn't support GCC built-in atomics.
//(e.g. undefined reference to __sync_add_and_fetch_N).
//Avoiding the built-in atomics seems no longer necessary. Presumably, the code below 
//exists merely for historic reasons.
#if defined (K_VX_KERNEL)

kFx(k32s) xkAtomic32s_IncrementImpl(kAtomic32s* atomic) 
{
    VX_MEM_BARRIER_RW(); 

    return vxAtomic32Inc(atomic) + 1; 
}

kFx(k32s) xkAtomic32s_DecrementImpl(kAtomic32s* atomic)
{
    VX_MEM_BARRIER_RW(); 

    return vxAtomic32Dec(atomic) - 1; 
}

kFx(k32s) xkAtomic32s_ExchangeImpl(kAtomic32s* atomic, k32s value) 
{
    VX_MEM_BARRIER_RW(); 

    return vxAtomic32Set(atomic, value);
}

kFx(kBool) xkAtomic32s_CompareExchangeImpl(kAtomic32s* atomic, k32s oldValue, k32s value)
{
    VX_MEM_BARRIER_RW(); 

    return vxAtomic32Cas(atomic, oldValue, value);
}

kFx(k32s) xkAtomic32s_GetImpl(kAtomic32s* atomic)
{
    VX_MEM_BARRIER_RW(); 

    return vxAtomic32Get(atomic);
}

kFx(kPointer) xkAtomicPointer_ExchangeImpl(kAtomicPointer* atomic, kPointer value)
{
    VX_MEM_BARRIER_RW(); 

    return (kPointer)vxAtomicSet(atomic, (kSize)value);
}

kFx(kBool) xkAtomicPointer_CompareExchangeImpl(kAtomicPointer* atomic, kPointer oldValue, kPointer value)
{
    VX_MEM_BARRIER_RW(); 

    return vxAtomicCas(atomic, (kSize)oldValue, (kSize)value);
}

kFx(kPointer) xkAtomicPointer_GetImpl(kAtomicPointer* atomic)
{
    VX_MEM_BARRIER_RW(); 

    return (kPointer)vxAtomicGet(atomic);
}

#endif
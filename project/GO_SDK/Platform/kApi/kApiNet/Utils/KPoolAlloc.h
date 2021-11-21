// 
// KPoolAlloc.h
// 
// Copyright (C) 2014-2021 by LMI Technologies Inc.
// Licensed under the MIT License.
// Redistributed files must retain the above copyright notice.
// 
#ifndef K_API_NET_POOL_ALLOC_H
#define K_API_NET_POOL_ALLOC_H

#include <kApi/Utils/kPoolAlloc.h>
#include "kApiNet/KAlloc.h"

namespace Lmi3d
{
    namespace Zen 
    {
        namespace Utils
        {
            /// <summary>Allocates small buffers from larger blocks and/or caches deallocated buffers for later reuse.
            /// <para/> Requires manual disposal.</summary>
            /// 
            /// <remarks>
            /// <para>This memory allocator can be used to improve performance in some circumstances. It can reduce the number
            /// of individual requests to an underlying allocator by allocating multiple small memory buffers from larger
            /// blocks. It can also cache deallocated buffers for later reuse, reducing the frequency of allocation requests
            /// made to the underlying allocator.</para>
            /// 
            /// <para>For each memory request, a 'rank' is determined by calculating the base-2 logarithm of the requested size and then
            /// rounding up. The rank determines the true size of the buffer that will be allocated (requests are rounded up to
            /// the nearest power of two). Rank-based buffer management provides simple organization and fast reallocation; the
            /// cost is increased memory space.</para>
            /// 
            /// <para>Parameters are provided to control which ranks should be allocated from larger blocks, which ranks should
            /// be cached upon deallocation, memory capacities, etc. Memory can be reserved using the KPoolAlloc.Reserve
            /// and KPoolAlloc.ReserveAt methods, and/or dynamically allocated from the underlying allocator as needed.</para>
            /// 
            /// <para>The operations provided in this class should be used in the following order:
            /// <list type="bullet">
            /// <item><description>Construct a KPoolAlloc instance.</description></item>
            /// <item><description>Perform configuration.</description></item>
            /// <item><description>Use the KPoolAlloc.Start method to prepare the allocator for use.</description></item>
            /// <item><description>Use the KPoolAlloc.Reserve or KPoolAlloc.ReserveAt methods to make pre-emptive allocations.</description></item>
            /// <item><description>Use the KAlloc.Get/KAlloc.Free methods to perform allocations/deallocations.</description></item>
            /// <item><description>Destroy the KPoolAlloc instance when no longer needed.</description></item>
            /// </list>
            /// </para>
            /// <para>All outstanding memory allocations must be freed before destroying the allocator.</para>
            /// </remarks>
            /// 
            public ref class KPoolAlloc : public KAlloc
            {
                KDeclareClass(KPoolAlloc, kPoolAlloc)

            public:
                /// <summary>Initializes a new instance of the KPoolAlloc class with the specified Zen object handle.</summary>           
                /// <param name="handle">Zen object handle.</param>
                KPoolAlloc(IntPtr handle)
                    : KAlloc(handle, DefaultRefStyle)
                {}

                /// <summary>Initializes a new instance of the KPoolAlloc class.</summary>           
                KPoolAlloc()
                    : KAlloc(DefaultRefStyle)
                {
                    kPoolAlloc handle = kNULL;

                    KCheck(kPoolAlloc_Construct(&handle, kNULL));

                    Handle = handle;
                }

                /// <summary>Initializes a new instance of the KPoolAlloc class with the specified underlying allocator.</summary>           
                /// 
                /// <remarks>
                /// The 'allocator' argument specifies the underlying memory allocator used by this kPoolAlloc
                /// instance to satisfy memory requests.
                /// </remarks>
                /// 
                /// <param name="allocator">Memory allocator.</param>
                KPoolAlloc(KAlloc^ allocator)
                    : KAlloc(DefaultRefStyle)
                {
                    kPoolAlloc handle = kNULL;

                    KCheck(kPoolAlloc_Construct(&handle, KToHandle(allocator)));

                    Handle = handle;
                }

                /// <summary>Sets or gets the approximate size (bytes) of large memory blocks used to satisfy small memory requests.</summary>
                /// 
                /// <remarks>
                /// <para>Blocks are assigned to one rank at a time. Multiple small allocations are typically performed
                /// from each larger block. The number of allocations that can be performed from a single block
                /// depends on the rank.</para>
                /// 
                /// <para>A common block size is used for all ranks to allow free blocks to be transferred between ranks
                /// (if reuse is enabled).</para>
                /// </remarks>
                property k64s BlockSize
                {
                    k64s get() { return KSizeTo64s(kPoolAlloc_BlockSize(Handle)); }
                    
                    void set(k64s value) { KCheck(kPoolAlloc_SetBlockSize(Handle, K64sToSize(value))); }
                }               

                /// <summary>Sets or gets the size limit (bytes) for memory requests that can be allocated from larger blocks.</summary>
                /// 
                /// <remarks>
                /// This property must be smaller than the BlockSize property. By default, this property
                /// is zero (block-based allocation is disabled).
                /// </remarks>
                property k64s MaxBlockBufferSize
                {
                    k64s get() { return KSizeTo64s(kPoolAlloc_MaxBlockBufferSize(Handle)); }

                    void set(k64s value) { KCheck(kPoolAlloc_SetMaxBlockBufferSize(Handle, K64sToSize(value))); }
                }

                /// <summary>Gets or sets the maximum total amount of memory (bytes) that can be used for block-based allocations.</summary>
                /// 
                /// <remarks>
                /// Blocks are dynamically allocated when needed; this property controls the maximum amount
                /// of memory that can be used for blocks. This property is equal to K64s.Max (Int64.MaxValue) by default.
                /// </remarks>
                property k64s BlockCapacity
                {
                    k64s get() { return KSizeTo64s(kPoolAlloc_BlockCapacity(Handle)); }

                    void set(k64s value) { KCheck(kPoolAlloc_SetBlockCapacity(Handle, K64sToSize(value))); }
                }

                /// <summary>Determines whether blocks can be reused between ranks.</summary>
                /// 
                /// <remarks>
                /// <para>When a memory request qualifies for block-based allocation, and no free buffers
                /// are available at the required rank, a new block must be provided. If block
                /// reuse is enabled, then free blocks from other ranks can be reassigned as needed.
                /// If block reuse is disabled, then blocks remain at the rank to which they are first
                /// assigned.</para>
                /// 
                /// <para>Block reuse can increase allocation time slightly, due to the need to search
                /// through all ranks for a free block. Reuse is enabled by default.</para>
                /// </remarks>
                property bool BlockReuseEnabled
                {
                    bool get() { return KToBool(kPoolAlloc_BlockReuseEnabled(Handle)); }

                    void set(bool value) { KCheck(kPoolAlloc_EnableBlockReuse(Handle, value)); }
                }


                /// <summary>Sets the size limit for memory requests that can be cached upon deallocation.</summary>
                /// 
                /// <remarks>
                /// <para>Small allocations are typically configured to be provided by block-based allocation, which
                /// automatically caches deallocated buffers for reuse. But larger (individually allocated)
                /// buffers can also be cached upon deallocation for later use. This property controls the
                /// maximum buffer size than can be cached upon deallocation.</para>
                /// 
                /// <para>This property is zero by default (caching of individual allocations is disabled).</para>
                /// </remarks>
                property k64s MaxCachedBufferSize
                {
                    k64s get() { return KSizeTo64s(kPoolAlloc_MaxCachedBufferSize(Handle)); }

                    void set(k64s value) { KCheck(kPoolAlloc_SetMaxCachedBufferSize(Handle, K64sToSize(value))); }
                }


                /// <summary>Sets the maximum total amount of memory that can be used to cache buffers upon deallocation.</summary>
                /// 
                /// <remarks>
                /// <para>This property does not affect block-based allocations, which can be limited using the BlockCapacity property.</para>
                /// 
                /// <para>This property is equal to K64s.Max (Int64.MaxValue) by default.</para>
                /// </remarks>
                property k64s CacheCapacity
                {
                    k64s get() { return KSizeTo64s(kPoolAlloc_CacheCapacity(Handle)); }

                    void set(k64s value) { KCheck(kPoolAlloc_SetCacheCapacity(Handle, K64sToSize(value))); }
                }

                /// <summary>Sets the total amount of memory that can be requested from the underlying allocator.</summary>
                /// 
                /// <remarks>
                /// <para>This property is equal to K64s.Max (Int64.MaxValue) by default.</para>
                /// </remarks>
                property k64s TotalCapacity
                {
                    k64s get() { return KSizeTo64s(kPoolAlloc_TotalCapacity(Handle)); }

                    void set(k64s value) { KCheck(kPoolAlloc_SetTotalCapacity(Handle, K64sToSize(value))); }
                }
             
                /// <summary>Prepares the allocator for first use.</summary>
                /// 
                /// <remarks>
                /// <para>This methods should be called after configuration properties have been provided, and before using
                /// memory reservation or allocation methods.</para>
                /// 
                /// <para>Configuration properties cannot be used after calling KPoolAlloc.Start.</para>
                /// </remarks>
                void Start()
                {
                    KCheck(kPoolAlloc_Start(Handle)); 
                }


                /// <summary>Specifies the minimum amount of memory (bytes) that should be set aside for blocks.</summary>
                /// 
                /// <remarks>
                /// <para>This method can be used to ensure that the specified amount of memory is set aside
                /// for block-based allocations. The amount of memory specified is inclusive of any
                /// existing block memory (e.g., individual rank reservations).</para>
                /// 
                /// <para>This method cannot be used before calling KPoolAlloc.Start.</para>
                /// </remarks>
                /// 
                /// <param name="size">Amount of memory to reserve, in bytes.</param>
                void Reserve(k64s size)
                {
                    KCheck(kPoolAlloc_Reserve(Handle, K64sToSize(size))); 
                }


                /// <summary>Specifies the minimum amount of memory (bytes) that should be set aside at a particular rank.</summary>
                /// 
                /// <remarks>
                /// <para>This method can be used to ensure that the specified amount of memory is set aside
                /// for block-based allocations at a specific rank.</para>
                /// 
                /// <para>This method can also be used to pre-cache individual buffers at a specific rank.
                /// However, these buffers may later be deallocated if the CacheCapacity limit is reached.</para>
                /// 
                /// <para>This method cannot be used before calling KPoolAlloc.Start.</para>
                /// </remarks>
                /// 
                /// <param name="rank">Memory rank (base-2 logarithm of memory size).</param>
                /// <param name="size">Amount of memory to reserve, in bytes.</param>
                void ReserveAt(k32s rank, k64s size)
                {
                    KCheck(kPoolAlloc_ReserveAt(Handle, (kSize)rank, K64sToSize(size))); 
                }

                /// <summary>Returns surplus memory to the underlying allocator.</summary>
                /// 
                /// <remarks>
                /// <para>For block-based ranks, any unused blocks will be returned to the underlying allocator.
                /// Reservations made using KPoolAlloc.Reserve/KPoolAlloc.ReserveAt are honored; those
                /// blocks will be preserved for later use.</para>
                /// 
                /// <para>For ranks configured to cache buffers upon deallocation, any unused buffers will be
                /// returned to the underlying allocator. Reservations made using KPoolAlloc.ReserveAt are
                /// honored; those buffers will be preserved for later use.</para>
                /// </remarks>
                void Clear()
                {
                    KCheck(kPoolAlloc_Clear(Handle)); 
                }

                /// <summary>Removes any existing memory reservations and returns surplus memory to the underlying allocator.</summary>
                void ClearAll()
                {
                    KCheck(kPoolAlloc_ClearAll(Handle));
                }

                /// <summary>Reports the total number of memory buffers at the given rank.</summary>
                /// 
                /// <remarks>
                /// <para>This method reports the total number of buffers, including buffers currently in use
                /// (allocated via KAlloc.Get and not yet freed) and buffers cached for later use.</para>
                /// </remarks>
                /// 
                /// <param name="rank">Memory rank (base-2 logarithm of memory size).</param>
                /// <returns>Buffer count.</returns>
                k64s GetBufferCount(k32s rank)
                {
                    return (k64s) kPoolAlloc_BufferCountAt(Handle, (kSize)rank); 
                }

                /// <summary>Reports the current amount of memory drawn from the underlying allocator.</summary>
                /// 
                /// <remarks>
                /// Total does not include the KPoolAlloc object header; all other memory is included.
                /// </remarks>
                property k64s TotalSize
                {
                    k64s get() { return (k64s) kPoolAlloc_TotalSize(Handle); }
                }

            protected:
                virtual void OnDisposing() override
                {
                    GC::Collect();
                    GC::WaitForPendingFinalizers();
                }
            };

        }
    }
}

#endif

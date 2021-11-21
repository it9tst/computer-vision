/** 
 * @file    kPoolAlloc.x.h
 *
 * @internal
 * Copyright (C) 2013-2021 by LMI Technologies Inc.  All rights reserved.
 */
#ifndef K_API_POOL_ALLOC_X_H
#define K_API_POOL_ALLOC_X_H

#define xkPOOL_ALLOC_DEFAULT_BLOCK_SIZE                  (1 << 14)               ///< Default value for BlockSize property, in bytes.
#define xkPOOL_ALLOC_DEFAULT_MAX_BLOCK_BUFFER_SIZE       (0)                     ///< Default value for MaxBlockBufferSize property, in bytes.
#define xkPOOL_ALLOC_DEFAULT_BLOCK_CAPACITY              (kSIZE_MAX)             ///< Default value for BlockCapacity property, in bytes.
#define xkPOOL_ALLOC_DEFAULT_BLOCK_REUSE_ENABLED         (kTRUE)                 ///< Default value for BlockReuseEnabled property.
#define xkPOOL_ALLOC_DEFAULT_MAX_CACHED_BUFFER_SIZE      (0)                     ///< Default value for MaxCachedBufferSize property, in bytes.
#define xkPOOL_ALLOC_DEFAULT_CACHE_CAPACITY              (kSIZE_MAX)             ///< Default value for CacheCapacity property, in bytes.
#define xkPOOL_ALLOC_DEFAULT_TOTAL_CAPACITY              (kSIZE_MAX)             ///< Default value for TotalCapacity property, in bytes.

#define xkPOOL_ALLOC_RANK_CAPACITY                       (32)                    ///< kPoolAlloc can optimize 32-bit or smaller allocations (limited by use of kMath_Log2Ceil32u to compute rank).
#define xkPOOL_ALLOC_MAX_MANAGED_BUFFER_SIZE             (0x80000000)            ///< Based on xkPOOL_ALLOC_RANK_CAPACITY, hard interal limit on optimized buffer size.

#define xkPOOL_ALLOC_MIN_HEADERS_PER_BLOCK               (8)                     ///< Used in block allocation to ensure at least N buffer headers will fit (helps reduce fragementation).

#define xkPOOL_ALLOC_ALLOC_TYPE_MASK                     (1)                     ///< Used to distinguish between type (bit set) and aligned address (bit clear)

/**
 * @internal
 * @struct  xkPoolAllocAllocType
 * @ingroup kApi-Utils
 * @brief   Represents a kPoolAlloc allocation type GUID.
 * 
 * Allocation type identifiers are dynamically-generated GUIDs (use of GUIDs improves ability to detect 
 * invalid deallocations). Must have xkPOOL_ALLOC_BUFFER_HEADER_TYPE_MASK bit set.
 */
typedef kSize xkPoolAllocAllocType; 

/**
 * @internal
 * @struct  xkPoolAllocBlockHeader
 * @ingroup kApi-Utils
 * @brief   Header for a kPoolAlloc buffer allocation block. 
 */
typedef struct xkPoolAllocBlockHeader
{
    kSize allocCount;                               ///< Count of allocations made from this block.
    kSize refCount;                                 ///< Count of allocations from this block that are currently in use (not free).
    kSize rank;                                     ///< Base-2 logarithm of block buffer size (index into kPoolAlloc rank array; kSIZE_NULL if block unassigned).
    struct xkPoolAllocBlockHeader* next;             ///< Pointer to next block (used in xkPoolAllocRank.newBlockList, xkPoolAllocRank.freeBlockList, kPoolAlloc.unassignedBlockList).
    struct xkPoolAllocBlockHeader* previous;         ///< Pointer to previous block (used in xkPoolAllocRank.newBlockList, xkPoolAllocRank.freeBlockList, kPoolAlloc.unassignedBlockList).
    kSize unused;                                   ///< Structure allocation padding. 
} xkPoolAllocBlockHeader; 

/**
 * @internal
 * @struct  xkPoolAllocBufferHeader
 * @ingroup kApi-Utils
 * @brief   Header for a kPoolAlloc memory buffer. 
 */
typedef struct xkPoolAllocBufferHeader
{
    union
    {
        xkPoolAllocBlockHeader* block;               ///< For block-based allocations, the block from which this allocation was created.
        kSize rank;                                 ///< For potentially-cached allocations, the rank associated with the allocation.
        kSize size;                                 ///< For unmanaged allocations, the raw size of the allocation (excluding header).         
    } info;                                         ///< Allocation-type-specific information.
    struct xkPoolAllocBufferHeader* next;            ///< Pointer to next buffer (used in xkPoolAllocRank.freeBufferList).
    struct xkPoolAllocBufferHeader* previous;        ///< Pointer to previous buffer (used in xkPoolAllocRank.freeBufferList).
    xkPoolAllocAllocType type;                       ///< Type of kPoolAlloc allocation. 
} xkPoolAllocBufferHeader;

/**
 * @internal
 * @struct  xkPoolAllocBlockList
 * @ingroup kApi-Utils
 * @brief   List of kPoolAlloc memory blocks.
 */
typedef struct xkPoolAllocBlockList
{
    xkPoolAllocBlockHeader* first;                   ///< Head of block list. 
    xkPoolAllocBlockHeader* last;                    ///< Tail of block list. 
    kSize count;                                    ///< Count of items in block list. 
} xkPoolAllocBlockList; 

/**
 * @internal
 * @struct  xkPoolAllocBufferList
 * @ingroup kApi-Utils
 * @brief   List of kPoolAlloc memory buffers.
 */
typedef struct xkPoolAllocBufferList
{
    xkPoolAllocBufferHeader* first;                  ///< Head of buffer list. 
    xkPoolAllocBufferHeader* last;                   ///< Tail of buffer list. 
    kSize count;                                    ///< Count of items in buffer list. 
} xkPoolAllocBufferList; 

/**
 * @internal
 * @struct  xkPoolAllocRank
 * @ingroup kApi-Utils
 * @brief   Represents kPoolAlloc information related to a specific memory rank.
 */
typedef struct xkPoolAllocRank
{
    kSize rank;                                     ///< Rank (base-2 logarithm of buffer size).  
    kSize size;                                     ///< Size of buffer associate with this rank. 
    kSize bufferStride;                             ///< Memory footprint of one block-based buffer, including buffer header and alignment padding.
    kBool isBlockBased;                             ///< Use blocks to allocate buffers?
    kBool isCached;                                 ///< Cache deallocated buffers for later use?
    kSize buffersPerBlock;                          ///< Count of buffers that can be allocated per block.
    kSize bufferCount;                              ///< Count of buffers in use for this rank (allocated or cached). 
    kSize minBufferCount;                           ///< Count of buffers reserved for this rank.
    xkPoolAllocBufferList freeBufferList;            ///< List of free buffers. 
    xkPoolAllocBlockList newBlockList;               ///< List of blocks whose allocCount is less than buffersPerBlock.
    xkPoolAllocBlockList freeBlockList;              ///< List of blocks with a reference count of zero (not currently in use).
} xkPoolAllocRank; 

typedef struct kPoolAllocClass
{
    kAllocClass base;     

    kLock lock;                                             //Provides mutual exclusion.

    kBool inheritPriority;                                  //Support priority inheritance?
    kSize nominalBlockSize;                                 //Nominal block size for block-based allocations (exluding header), in bytes.
    kSize maxBlockBufferSize;                               //Maximum buffer size for an individual block-based allocation, in bytes.
    kSize blockCapacity;                                    //Maximum memory used to allocate blocks, in bytes.
    kBool blockReuseEnabled;                                //Can blocks be automatically reassigned between ranks as needed?
    kSize maxCachedBufferSize;                              //Maximum buffer size for an individual cached allocation, in bytes.
    kSize cacheCapacity;                                    //Maximum amount of non-block memory to retain upon deallocation, in bytes.
    kSize totalCapacity;                                    //Maximum amount of memory that can be requested from underlying allocator.

    kSize blockBasedRankCount;                              //Count of ranks potentially eligible for block-based allocation.
    kSize cacheBasedRankCount;                              //Count of ranks potentially eligible for cache-based allocation.
    kSize managedRankCount;                                 //Maximum of blockBasedRankCount and cacheBasedRankCount. 
    kSize maxManagedBufferSize;                             //Maximum size of managed buffer.

    kSize blockAllocId;                                     //GUID for block allocations from this allocator. 
    kSize cacheAllocId;                                     //GUID for cached allocations from this allocator. 
    kSize unmanagedAllocId;                                 //GUID for unmanaged allocations from this allocator. 

    kBool isStarted;                                        //Has pool-alloc been started?

    xkPoolAllocRank rankInfo[xkPOOL_ALLOC_RANK_CAPACITY];   //Information associated with each rank. 

    xkPoolAllocBlockList unassignedBlockList;                //List of blocks blocks not yet assigned to any rank.

    kSize blockCount;                                       //Total number of blocks currently allocated from underlying allocator.
    kSize minBlockCount;                                    //Minimum number of blocks that should be allocated from underlying allocator (reservation).
    kSize cacheSize;                                        //Total size of free, cached (non-block) allocations, excluding headers, in bytes.
    kSize totalSize;                                        //Total amount of memory currently drawn from underlying allocator.
   
} kPoolAllocClass; 

kDeclareClassEx(k, kPoolAlloc, kAlloc)
        
/* 
* Private methods. 
*/

kFx(kStatus) xkPoolAlloc_Init(kPoolAlloc object, kType type, kAlloc alloc);
kFx(kStatus) xkPoolAlloc_VRelease(kPoolAlloc object);

kFx(kStatus) xkPoolAlloc_VGet(kPoolAlloc object, kSize size, void* mem, kMemoryAlignment alignment);
kFx(kStatus) xkPoolAlloc_VFree(kPoolAlloc object, void* mem); 

kFx(kStatus) xkPoolAlloc_EnablePriorityInheritance(kPoolAlloc object, kBool enable);

kFx(kStatus) xkPoolAlloc_ReserveBlocksAt(kPoolAlloc object, kSize rank, kSize size);
kFx(kStatus) xkPoolAlloc_ReserveCachedBuffersAt(kPoolAlloc object, kSize rank, kSize size);

kFx(void) xkPoolAlloc_RemoveFromBlockList(kPoolAlloc object, xkPoolAllocBlockList* list, xkPoolAllocBlockHeader* item); 
kFx(void) xkPoolAlloc_InsertIntoBlockList(kPoolAlloc object, xkPoolAllocBlockList* list, xkPoolAllocBlockHeader* before, xkPoolAllocBlockHeader* item); 

kFx(void) xkPoolAlloc_RemoveFromBufferList(kPoolAlloc object, xkPoolAllocBufferList* list, xkPoolAllocBufferHeader* item); 
kFx(void) xkPoolAlloc_InsertIntoBufferList(kPoolAlloc object, xkPoolAllocBufferList* list, xkPoolAllocBufferHeader* before, xkPoolAllocBufferHeader* item); 

kFx(kStatus) xkPoolAlloc_AddBlockRef(kPoolAlloc object, xkPoolAllocBlockHeader* block); 
kFx(kStatus) xkPoolAlloc_RemoveBlockRef(kPoolAlloc object, xkPoolAllocBlockHeader* block); 

kFx(kStatus) xkPoolAlloc_AllocBlock(kPoolAlloc object); 
kFx(kStatus) xkPoolAlloc_AllocCachedBuffers(kPoolAlloc object, kSize rank, kSize count); 
kFx(kStatus) xkPoolAlloc_AllocUnmanagedBuffer(kPoolAlloc object, kSize size, void* mem, kMemoryAlignment alignment);

kFx(kStatus) xkPoolAlloc_PrepareFreeBlockBasedBuffer(kPoolAlloc object, kSize rank); 
kFx(kStatus) xkPoolAlloc_RecycleBlocks(kPoolAlloc object, kSize maxCount); 
kFx(kStatus) xkPoolAlloc_ReturnBlockToUnassigned(kPoolAlloc object, xkPoolAllocBlockList* list, xkPoolAllocBlockHeader* block);

kFx(kStatus) xkPoolAlloc_ClearEx(kPoolAlloc object, kBool all);
kFx(kStatus) xkPoolAlloc_FreeUnassignedBlocks(kPoolAlloc object); 
kFx(kStatus) xkPoolAlloc_FreeCachedBuffers(kPoolAlloc object); 

/** Reports size of a single block, excluding block header, in bytes. */
kInlineFx(kSize) xkPoolAlloc_BlockContentSize(kPoolAlloc object)
{
    kObj(kPoolAlloc, object); 

    return obj->nominalBlockSize + xkPOOL_ALLOC_MIN_HEADERS_PER_BLOCK*sizeof(xkPoolAllocBufferHeader) + kALIGN_ANY_SIZE;
}

kInlineFx(kSize) xkPoolAlloc_AllocBlockSize(kPoolAlloc object)
{
    kObj(kPoolAlloc, object); 

    return sizeof(xkPoolAllocBlockHeader) + xkPoolAlloc_BlockContentSize(object);
}


#endif

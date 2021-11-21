/** 
 * @file    kPoolAlloc.cpp
 *
 * @internal
 * Copyright (C) 2013-2021 by LMI Technologies Inc.  All rights reserved.
 */
#include <kApi/Utils/kPoolAlloc.h>
#include <kApi/Data/kMath.h>
#include <kApi/Threads/kLock.h>

kBeginClassEx(k, kPoolAlloc)
    kAddPrivateVMethod(kPoolAlloc, kObject, VRelease)
    kAddPrivateVMethod(kPoolAlloc, kAlloc, VGet)
    kAddPrivateVMethod(kPoolAlloc, kAlloc, VFree)
kEndClassEx()

kFx(kStatus) kPoolAlloc_Construct(kPoolAlloc* object, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback(allocator);
    kStatus status; 

    kAssert((sizeof(xkPoolAllocBlockHeader) % kALIGN_ANY_SIZE) == 0); 
    kAssert((sizeof(xkPoolAllocBufferHeader) % kALIGN_ANY_SIZE) == 0); 

    kCheck(kAlloc_GetObject(alloc, kTypeOf(kPoolAlloc), object)); 

    if (!kSuccess(status = xkPoolAlloc_Init(*object, kTypeOf(kPoolAlloc), alloc)))
    {
        kAlloc_FreeRef(alloc, object); 
    }

    return status; 
} 

kFx(kStatus) xkPoolAlloc_Init(kPoolAlloc object, kType type, kAlloc alloc)
{
    kObjR(kPoolAlloc, object); 

    kCheck(kAlloc_Init(object, type, alloc)); 

    obj->base.traits = kAlloc_Traits(alloc);

    obj->lock = kNULL;
    obj->inheritPriority = kFALSE;
    obj->nominalBlockSize = xkPOOL_ALLOC_DEFAULT_BLOCK_SIZE; 
    obj->maxBlockBufferSize = xkPOOL_ALLOC_DEFAULT_MAX_BLOCK_BUFFER_SIZE; 
    obj->blockCapacity = xkPOOL_ALLOC_DEFAULT_BLOCK_CAPACITY; 
    obj->blockReuseEnabled = xkPOOL_ALLOC_DEFAULT_BLOCK_REUSE_ENABLED; 
    obj->maxCachedBufferSize = xkPOOL_ALLOC_DEFAULT_MAX_CACHED_BUFFER_SIZE; 
    obj->cacheCapacity = xkPOOL_ALLOC_DEFAULT_CACHE_CAPACITY; 
    obj->totalCapacity = xkPOOL_ALLOC_DEFAULT_TOTAL_CAPACITY; 
    obj->blockBasedRankCount = 0;
    obj->cacheBasedRankCount = 0;
    obj->managedRankCount = 0;
    obj->maxManagedBufferSize = 0;
    obj->blockAllocId = 0;
    obj->cacheAllocId = 0;
    obj->unmanagedAllocId = 0;
    obj->isStarted = kFALSE;
    kZero(obj->rankInfo);
    kZero(obj->unassignedBlockList);
    obj->blockCount = 0;
    obj->minBlockCount = 0;
    obj->cacheSize = 0;
    obj->totalSize = 0;

    return kOK; 
}

kFx(kStatus) xkPoolAlloc_VRelease(kPoolAlloc object)
{
    kObj(kPoolAlloc, object); 

    if (obj->isStarted)
    {
        kCheck(kPoolAlloc_ClearAll(object)); 
    }

    kCheck(kObject_Destroy(obj->lock)); 

    kCheck(kAlloc_VRelease(object)); 

    return kOK; 
}

kFx(kStatus) xkPoolAlloc_EnablePriorityInheritance(kPoolAlloc object, kBool enable)
{
    kObj(kPoolAlloc, object); 

    kCheckState(!obj->isStarted); 

    obj->inheritPriority = enable;

    return kOK; 
}

kFx(kStatus) kPoolAlloc_SetBlockSize(kPoolAlloc object, kSize size)
{
    kObj(kPoolAlloc, object); 

    kCheckState(!obj->isStarted); 

    obj->nominalBlockSize = size; 

    return kOK; 
}

kFx(kSize) kPoolAlloc_BlockSize(kPoolAlloc object)
{
    kObj(kPoolAlloc, object); 
   
    return obj->nominalBlockSize; 
}

kFx(kStatus) kPoolAlloc_SetMaxBlockBufferSize(kPoolAlloc object, kSize size)
{
    kObj(kPoolAlloc, object); 

    kCheckState(!obj->isStarted); 

    obj->maxBlockBufferSize = size; 

    return kOK; 
}

kFx(kSize) kPoolAlloc_MaxBlockBufferSize(kPoolAlloc object)
{
    kObj(kPoolAlloc, object); 
   
    return obj->maxBlockBufferSize; 
}

kFx(kStatus) kPoolAlloc_SetBlockCapacity(kPoolAlloc object, kSize size)
{
    kObj(kPoolAlloc, object); 

    kCheckState(!obj->isStarted); 

    obj->blockCapacity = size; 

    return kOK; 
}

kFx(kSize) kPoolAlloc_BlockCapacity(kPoolAlloc object)
{
    kObj(kPoolAlloc, object); 
   
    return obj->blockCapacity; 
}

kFx(kStatus) kPoolAlloc_EnableBlockReuse(kPoolAlloc object, kBool enabled)
{
    kObj(kPoolAlloc, object); 

    kCheckState(!obj->isStarted); 

    obj->blockReuseEnabled = enabled; 

    return kOK; 
}

kFx(kBool) kPoolAlloc_BlockReuseEnabled(kPoolAlloc object)
{
    kObj(kPoolAlloc, object); 
   
    return obj->blockReuseEnabled; 
}

kFx(kStatus) kPoolAlloc_SetMaxCachedBufferSize(kPoolAlloc object, kSize size)
{
    kObj(kPoolAlloc, object); 

    kCheckState(!obj->isStarted); 

    obj->maxCachedBufferSize = size; 

    return kOK; 
}

kFx(kSize) kPoolAlloc_MaxCachedBufferSize(kPoolAlloc object)
{
    kObj(kPoolAlloc, object); 
   
    return obj->maxCachedBufferSize; 
}

kFx(kStatus) kPoolAlloc_SetCacheCapacity(kPoolAlloc object, kSize size)
{
    kObj(kPoolAlloc, object); 

    kCheckState(!obj->isStarted); 

    obj->cacheCapacity = size; 

    return kOK; 
}

kFx(kSize) kPoolAlloc_CacheCapacity(kPoolAlloc object)
{
    kObj(kPoolAlloc, object); 
   
    return obj->cacheCapacity; 
}

kFx(kStatus) kPoolAlloc_SetTotalCapacity(kPoolAlloc object, kSize size)
{
    kObj(kPoolAlloc, object); 

    kCheckState(!obj->isStarted); 

    obj->totalCapacity = size; 

    return kOK; 
}

kFx(kSize) kPoolAlloc_TotalCapacity(kPoolAlloc object)
{
    kObj(kPoolAlloc, object); 
   
    return obj->totalCapacity; 
}

kFx(kStatus) kPoolAlloc_Start(kPoolAlloc object)
{
    kObj(kPoolAlloc, object); 
    kSize hardMaxBufferSize = xkPOOL_ALLOC_MAX_MANAGED_BUFFER_SIZE; 
    kSize i; 

    kCheckState(!obj->isStarted); 
    kCheckState(obj->maxBlockBufferSize <= obj->nominalBlockSize); 

    kCheck(kLock_ConstructEx(&obj->lock, (obj->inheritPriority) ? xkLOCK_OPTION_PRIORITY_INHERITANCE : kLOCK_OPTION_NONE, kObject_Alloc(object))); 

    //create randomized GUIDs for allocation type identifiers
    do 
    {
        // xkPOOL_ALLOC_ALLOC_TYPE_MASK must be set to all ids.
        obj->blockAllocId = kRandomSize() | xkPOOL_ALLOC_ALLOC_TYPE_MASK;
        obj->cacheAllocId = kRandomSize() | xkPOOL_ALLOC_ALLOC_TYPE_MASK;
        obj->unmanagedAllocId = kRandomSize() | xkPOOL_ALLOC_ALLOC_TYPE_MASK;
    } 
    while ((obj->blockAllocId == obj->cacheAllocId) || (obj->cacheAllocId == obj->unmanagedAllocId)); 

    //pre-calculate various parameters
    if (obj->maxBlockBufferSize == 0)                       obj->blockBasedRankCount = 0; 
    else if (obj->maxBlockBufferSize >= hardMaxBufferSize)  obj->blockBasedRankCount = xkPOOL_ALLOC_RANK_CAPACITY; 
    else                                                    obj->blockBasedRankCount = (kSize) kMath_Log2Ceil32u((k32u)obj->maxBlockBufferSize) + 1; 

    if (obj->maxCachedBufferSize == 0)                      obj->cacheBasedRankCount = 0; 
    else if (obj->maxCachedBufferSize >= hardMaxBufferSize) obj->cacheBasedRankCount = xkPOOL_ALLOC_RANK_CAPACITY; 
    else                                                    obj->cacheBasedRankCount = (kSize) kMath_Log2Ceil32u((k32u)obj->maxCachedBufferSize) + 1; 

    obj->managedRankCount = kMax_(obj->blockBasedRankCount, obj->cacheBasedRankCount); 
    obj->maxManagedBufferSize = (obj->managedRankCount > 0) ? ((kSize)1 << (obj->managedRankCount-1)) : 0; 

    for (i = 0; i < xkPOOL_ALLOC_RANK_CAPACITY; ++i)
    {
        xkPoolAllocRank* rankInfo = &obj->rankInfo[i]; 

        rankInfo->rank = i; 
        rankInfo->size = (kSize)1 << i;

        rankInfo->bufferStride = sizeof(xkPoolAllocBufferHeader) + rankInfo->size; 
        rankInfo->bufferStride = kSize_Align(rankInfo->bufferStride, kALIGN_ANY); 

        if (i < obj->blockBasedRankCount)
        {
            rankInfo->isBlockBased = kTRUE;  
            rankInfo->buffersPerBlock = xkPoolAlloc_BlockContentSize(object) / rankInfo->bufferStride;  
        }
        else if (i < obj->cacheBasedRankCount)
        {
            rankInfo->isCached = kTRUE;  
        }
    }

    obj->isStarted = kTRUE; 

    return kOK; 
}

static kPointer xkPoolAlloc_AlignAllocation(kPoolAlloc object, kPointer buffer, kMemoryAlignment alignment)
{
    auto* const userAllocation = kPointer_ByteOffset(buffer, sizeof(xkPoolAllocBufferHeader));
    auto* const alignedAllocation = (kPointer)kSize_Align((kSize)userAllocation, alignment);

    // Check whether we need to adjust memory address due to the alignment request
    if (alignedAllocation != userAllocation)
    {
        // Write address of the buffer just before user allocation that VFree()-function would be able to find the correct header location
        auto* allocationPointerPosition = kPointer_ByteOffset(alignedAllocation, -1 * (kSSize)sizeof(kPointer));
        kPointer_WriteAs(allocationPointerPosition, buffer, kPointer);

        return alignedAllocation;
    }

    return userAllocation;
}

kFx(kStatus) xkPoolAlloc_VGet(kPoolAlloc object, kSize size, void* mem, kMemoryAlignment alignment)
{
    kObj(kPoolAlloc, object); 
    kBool isAllocated = kFALSE; 

    if (alignment > kALIGN_ANY && size > 0)
    {
        // Size must be multiple of alignments
        size = kSize_Align(size, alignment);

        // Reserve extra space for possible padding bytes
        size += kMemoryAlignment_Size(alignment);
    }

    kLock_Enter(obj->lock); 

    kTry
    {   
        if (size == 0)
        {
            *(void**)mem = kNULL; 
            isAllocated = kTRUE; 
        }
        else if (size <= obj->maxManagedBufferSize)
        {
            kSize rank = (kSize) kMath_Log2Ceil32u((k32u)size);       
            xkPoolAllocRank* rankInfo = &obj->rankInfo[rank]; 

            if (rankInfo->isBlockBased)
            {
                if (rankInfo->freeBufferList.count == 0)
                {                 
                    xkPoolAlloc_PrepareFreeBlockBasedBuffer(object, rank); 
                }

                if (rankInfo->freeBufferList.count > 0)
                {
                    xkPoolAllocBufferHeader* buffer = rankInfo->freeBufferList.first;

                    kAssert(buffer->info.block->rank == rank); 

                    xkPoolAlloc_RemoveFromBufferList(object, &rankInfo->freeBufferList, buffer); 
                    xkPoolAlloc_AddBlockRef(object, buffer->info.block); 

                    *(void**)mem = xkPoolAlloc_AlignAllocation(object, buffer, alignment);
                    isAllocated = kTRUE;
                }
            }
            else if (rankInfo->isCached)
            {
                if (rankInfo->freeBufferList.count == 0)
                {                 
                    xkPoolAlloc_AllocCachedBuffers(object, rank, 1); 
                }

                if (rankInfo->freeBufferList.count > 0)
                {
                    xkPoolAllocBufferHeader* buffer = rankInfo->freeBufferList.first; 

                    xkPoolAlloc_RemoveFromBufferList(object, &rankInfo->freeBufferList, buffer); 
                    obj->cacheSize -= rankInfo->size; 

                    *(void**)mem = xkPoolAlloc_AlignAllocation(object, buffer, alignment);
                    isAllocated = kTRUE; 
                }
            }
        }

        if (!isAllocated)
        {
            kTest(xkPoolAlloc_AllocUnmanagedBuffer(object, size, mem, alignment));
        }
    }
    kFinally
    {
        kLock_Exit(obj->lock); 
        kEndFinally(); 
    }

    return kOK; 
}

kFx(kStatus) xkPoolAlloc_VFree(kPoolAlloc object, void* mem)
{
    kObj(kPoolAlloc, object); 
    kAlloc innerAlloc = kObject_Alloc(object); 

    kLock_Enter(obj->lock); 

    kTry
    {   
        if (!kIsNull(mem))
        {
            xkPoolAllocBufferHeader* buffer = kNULL;

            const auto bufferHeaderCandidateAddress = *(kSize*)kPointer_ByteOffset(mem, -1 * (kSSize)sizeof(kPointer));
            if (bufferHeaderCandidateAddress & xkPOOL_ALLOC_ALLOC_TYPE_MASK)
            {
                // xkPOOL_ALLOC_ALLOC_TYPE_MASK is set which means that 
                // the buffer header is found just before user allocation without padding.
                buffer = (xkPoolAllocBufferHeader*)kPointer_ByteOffset(mem, -1 * (kSSize)sizeof(xkPoolAllocBufferHeader));
            }
            else            
            {
                // xkPOOL_ALLOC_ALLOC_TYPE_MASK is not set which means that
                // padding is used and the buffer header is found from the candidate address.
                buffer = (xkPoolAllocBufferHeader*)bufferHeaderCandidateAddress;
            }
             
            if (buffer->type == obj->blockAllocId)
            {
                xkPoolAllocBlockHeader* block = buffer->info.block; 
                xkPoolAllocRank* rankInfo = &obj->rankInfo[block->rank]; 

                //CPU cache optimization: insert free buffer at front of list, so that most-recently-used buffer will be reused first
                xkPoolAlloc_InsertIntoBufferList(object, &rankInfo->freeBufferList, rankInfo->freeBufferList.first, buffer); 
                xkPoolAlloc_RemoveBlockRef(object, block); 
            }
            else if (buffer->type == obj->cacheAllocId)
            {
                xkPoolAllocRank* rankInfo = &obj->rankInfo[buffer->info.rank]; 

                if ((obj->cacheSize + rankInfo->size) <= obj->cacheCapacity)
                {
                    //CPU cache optimization: insert free buffer at front of list, so that most-recently-used buffer will be reused first
                    xkPoolAlloc_InsertIntoBufferList(object, &rankInfo->freeBufferList, rankInfo->freeBufferList.first, buffer); 
                    obj->cacheSize += rankInfo->size; 
                }
                else
                {
                    obj->totalSize -= sizeof(xkPoolAllocBufferHeader) + rankInfo->size; 
                    rankInfo->bufferCount--; 

                    kTest(kAlloc_Free(innerAlloc, buffer));             
                }
            }
            else if (buffer->type == obj->unmanagedAllocId)
            {
                obj->totalSize -= sizeof(xkPoolAllocBufferHeader) + buffer->info.size; 

                kTest(kAlloc_Free(innerAlloc, buffer));             
            }
            else
            {
                //if this assertion is reached, an invalid buffer was passed to this function (corrupted, or not from this allocator)
                kAssert(kFALSE); 
                kThrow(kERROR_PARAMETER); 
            }
        }
    }
    kFinally
    {
        kLock_Exit(obj->lock); 
        kEndFinally(); 
    }

    return kOK; 
}

kFx(kStatus) kPoolAlloc_Reserve(kPoolAlloc object, kSize size)
{
    kObj(kPoolAlloc, object); 

    kLock_Enter(obj->lock); 

    kTry
    {   
        obj->minBlockCount = kDivideCeilUInt_(size, obj->nominalBlockSize); 

        while (obj->blockCount < obj->minBlockCount)
        {
            kTest(xkPoolAlloc_AllocBlock(object)); 
        }
    }
    kFinally
    {
        kLock_Exit(obj->lock); 
        kEndFinally(); 
    }

    return kOK;
}

kFx(kStatus) kPoolAlloc_ReserveAt(kPoolAlloc object, kSize rank, kSize size)
{
    kObj(kPoolAlloc, object); 

    kLock_Enter(obj->lock); 

    kTry
    {   
        if (rank < obj->blockBasedRankCount)
        {
            kTest(xkPoolAlloc_ReserveBlocksAt(object, rank, size)); 
        }
        else if (rank < obj->cacheBasedRankCount)
        {
            kTest(xkPoolAlloc_ReserveCachedBuffersAt(object, rank, size)); 
        }
        else
        {
            kThrow(kERROR_PARAMETER); 
        }
    }
    kFinally
    {
        kLock_Exit(obj->lock); 
        kEndFinally(); 
    }

    return kOK;
}

kFx(kStatus) xkPoolAlloc_ReserveBlocksAt(kPoolAlloc object, kSize rank, kSize size)
{
    kObj(kPoolAlloc, object); 
    xkPoolAllocRank* rankInfo = &obj->rankInfo[rank]; 

    rankInfo->minBufferCount = kDivideCeilUInt_(size, rankInfo->size); 

    //reuse existing blocks, if possible
    if (obj->blockReuseEnabled && (rankInfo->minBufferCount > rankInfo->bufferCount))
    {
        xkPoolAlloc_RecycleBlocks(object, kDivideCeilUInt_(rankInfo->minBufferCount - rankInfo->bufferCount, rankInfo->buffersPerBlock)); 
    }

    //allocate the rest
    while ((rankInfo->bufferCount + obj->unassignedBlockList.count*rankInfo->buffersPerBlock) < rankInfo->minBufferCount)
    {
        kCheck(xkPoolAlloc_AllocBlock(object)); 
    }

    //assign allocated buffers to the requested rank
    while (rankInfo->bufferCount < rankInfo->minBufferCount)
    {       
        xkPoolAllocBlockHeader* block = obj->unassignedBlockList.first; 

        kAssert(obj->unassignedBlockList.count > 0); 

        xkPoolAlloc_RemoveFromBlockList(object, &obj->unassignedBlockList, block); 
        xkPoolAlloc_InsertIntoBlockList(object, &rankInfo->newBlockList, kNULL, block);

        block->rank = rankInfo->rank; 
        rankInfo->bufferCount += rankInfo->buffersPerBlock;            
    }

    return kOK;
}

kFx(kStatus) xkPoolAlloc_ReserveCachedBuffersAt(kPoolAlloc object, kSize rank, kSize size)
{
    kObj(kPoolAlloc, object); 
    xkPoolAllocRank* rankInfo = &obj->rankInfo[rank]; 

    rankInfo->minBufferCount = kDivideCeilUInt_(size, rankInfo->size); 

    if (rankInfo->bufferCount < rankInfo->minBufferCount)
    {
        kCheck(xkPoolAlloc_AllocCachedBuffers(object, rank, rankInfo->minBufferCount - rankInfo->bufferCount)); 
    }

    return kOK;
}

kFx(kStatus) kPoolAlloc_Clear(kPoolAlloc object)
{
    return xkPoolAlloc_ClearEx(object, kFALSE); 
}

kFx(kStatus) kPoolAlloc_ClearAll(kPoolAlloc object)
{
    return xkPoolAlloc_ClearEx(object, kTRUE); 
}

kFx(kStatus) xkPoolAlloc_PrepareFreeBlockBasedBuffer(kPoolAlloc object, kSize rank)
{
    kObj(kPoolAlloc, object); 
    xkPoolAllocRank* rankInfo = &obj->rankInfo[rank]; 

    if (rankInfo->freeBufferList.count == 0)
    {
        //if we don't have a "new" block to draw from, attempt to reuse or allocate one
        if (rankInfo->newBlockList.count == 0)
        {
            if ((obj->unassignedBlockList.count == 0) && obj->blockReuseEnabled)
            {
                xkPoolAlloc_RecycleBlocks(object, 1); 
            }
           
            if ((obj->unassignedBlockList.count == 0))
            {
                xkPoolAlloc_AllocBlock(object); 
            }

            if (obj->unassignedBlockList.count > 0)
            {
                xkPoolAllocBlockHeader* block = obj->unassignedBlockList.first; 

                xkPoolAlloc_RemoveFromBlockList(object, &obj->unassignedBlockList, block); 
                xkPoolAlloc_InsertIntoBlockList(object, &rankInfo->newBlockList, kNULL, block); 

                block->rank = rank; 
                rankInfo->bufferCount += rankInfo->buffersPerBlock; 
            }
        }

        //if we now have a "new" block to draw from, get a free buffer from the block
        if (rankInfo->newBlockList.count > 0)
        {
            xkPoolAllocBlockHeader* block = rankInfo->newBlockList.first;
            xkPoolAllocBufferHeader* buffer = (xkPoolAllocBufferHeader*) kPointer_ByteOffset(block, (kSSize)(sizeof(xkPoolAllocBlockHeader) + rankInfo->bufferStride*block->allocCount)); 

            kAssert(block->rank == rank); 

            buffer->type = obj->blockAllocId; 
            buffer->info.block = block; 
            buffer->next = kNULL; 
            buffer->previous = kNULL; 

            xkPoolAlloc_InsertIntoBufferList(object, &rankInfo->freeBufferList, kNULL, buffer); 

            block->allocCount++; 

            if (block->allocCount == rankInfo->buffersPerBlock)
            {
                xkPoolAlloc_RemoveFromBlockList(object, &rankInfo->newBlockList, block); 
                xkPoolAlloc_RemoveBlockRef(object, block); 
            }
        }
    }

    return (rankInfo->freeBufferList.count > 0) ? kOK : kERROR_NOT_FOUND; 
}

kFx(void) xkPoolAlloc_RemoveFromBlockList(kPoolAlloc object, xkPoolAllocBlockList* list, xkPoolAllocBlockHeader* item)
{
    if (!kIsNull(item->previous))
    {
        item->previous->next = item->next; 
    }
    
    if (!kIsNull(item->next))
    {
        item->next->previous = item->previous; 
    }

    if (list->first == item)
    {
        list->first = item->next; 
    }

    if (list->last == item)
    {
        list->last = item->previous; 
    }

    item->next = kNULL; 
    item->previous = kNULL;

    list->count--; 
} 

kFx(void) xkPoolAlloc_InsertIntoBlockList(kPoolAlloc object, xkPoolAllocBlockList* list, xkPoolAllocBlockHeader* before, xkPoolAllocBlockHeader* item)
{
    item->previous = kNULL; 
    item->next = kNULL; 

    if (kIsNull(list->first))
    {
        list->first = item; 
        list->last = item; 
    }
    else if (kIsNull(before))
    {
        item->previous = list->last; 
        list->last->next = item; 
        list->last = item;         
    }
    else if (before == list->first)
    {
        item->next = list->first; 
        list->first->previous = item; 
        list->first = item;         
    }
    else
    {
        item->next = before; 
        item->previous = before->previous; 
        before->previous->next = item; 
        before->previous = item;         
    }    

    list->count++; 
} 

kFx(void) xkPoolAlloc_RemoveFromBufferList(kPoolAlloc object, xkPoolAllocBufferList* list, xkPoolAllocBufferHeader* item)
{
    if (!kIsNull(item->previous))
    {
        item->previous->next = item->next; 
    }
    
    if (!kIsNull(item->next))
    {
        item->next->previous = item->previous; 
    }

    if (list->first == item)
    {
        list->first = item->next; 
    }

    if (list->last == item)
    {
        list->last = item->previous; 
    }

    item->next = kNULL; 
    item->previous = kNULL;

    list->count--; 
} 

kFx(void) xkPoolAlloc_InsertIntoBufferList(kPoolAlloc object, xkPoolAllocBufferList* list, xkPoolAllocBufferHeader* before, xkPoolAllocBufferHeader* item)
{
    item->previous = kNULL; 
    item->next = kNULL; 

    if (kIsNull(list->first))
    {
        list->first = item; 
        list->last = item; 
    }
    else if (kIsNull(before))
    {
        item->previous = list->last; 
        list->last->next = item; 
        list->last = item;         
    }
    else if (before == list->first)
    {
        item->next = list->first; 
        list->first->previous = item; 
        list->first = item;         
    }
    else
    {
        item->next = before; 
        item->previous = before->previous; 
        before->previous->next = item; 
        before->previous = item;         
    }    

    list->count++; 
} 

kFx(kStatus) xkPoolAlloc_AddBlockRef(kPoolAlloc object, xkPoolAllocBlockHeader* block)
{
    kObj(kPoolAlloc, object); 

    if (block->refCount == 0)
    {
        xkPoolAlloc_RemoveFromBlockList(object, &obj->rankInfo[block->rank].freeBlockList, block); 
    }

    block->refCount++; 

    return kOK; 
}

kFx(kStatus) xkPoolAlloc_RemoveBlockRef(kPoolAlloc object, xkPoolAllocBlockHeader* block)
{
    kObj(kPoolAlloc, object); 

    block->refCount--; 

    if (block->refCount == 0)
    {
        xkPoolAlloc_InsertIntoBlockList(object, &obj->rankInfo[block->rank].freeBlockList, kNULL, block); 
    }

    return kOK; 
}

kFx(kStatus) xkPoolAlloc_RecycleBlocks(kPoolAlloc object, kSize maxCount)
{
    kObj(kPoolAlloc, object); 
    kSize count = 0; 
    kSize i; 
    
    for (i = 0; i < obj->blockBasedRankCount; ++i)
    {
        kSize rankIndex = obj->blockBasedRankCount - i - 1;     //largest rank first (fewer buffers to unhook)
        xkPoolAllocRank* rankInfo = &obj->rankInfo[rankIndex]; 
            
        //try to reuse surplus "free" blocks
        while ((rankInfo->freeBlockList.count > 0) && 
               (rankInfo->bufferCount >= (rankInfo->minBufferCount + rankInfo->buffersPerBlock)))
        {
            kCheck(xkPoolAlloc_ReturnBlockToUnassigned(object, &rankInfo->freeBlockList, rankInfo->freeBlockList.last)); 

            if (++count == maxCount)
            {
                return kOK;
            }
        }

        //try to reuse surplus "new" blocks 
        while ((rankInfo->newBlockList.count > 0) && (rankInfo->newBlockList.last->refCount == 1) && 
               (rankInfo->bufferCount >= (rankInfo->minBufferCount + rankInfo->buffersPerBlock)))
        {
            kCheck(xkPoolAlloc_ReturnBlockToUnassigned(object, &rankInfo->newBlockList, rankInfo->newBlockList.last)); 

            if (++count == maxCount)
            {
                return kOK;
            }
        }      
    }

    return kOK;
}

kFx(kStatus) xkPoolAlloc_ReturnBlockToUnassigned(kPoolAlloc object, xkPoolAllocBlockList* list, xkPoolAllocBlockHeader* block)
{
    kObj(kPoolAlloc, object); 
    xkPoolAllocRank* rankInfo = &obj->rankInfo[block->rank]; 
    kSize i; 

    for (i = 0; i < block->allocCount; ++i)
    {
        xkPoolAllocBufferHeader* buffer = (xkPoolAllocBufferHeader*) kPointer_ByteOffset(block, (kSSize)(sizeof(xkPoolAllocBlockHeader) + i*rankInfo->bufferStride)); 

        xkPoolAlloc_RemoveFromBufferList(object, &rankInfo->freeBufferList, buffer); 
    }

    rankInfo->bufferCount -= rankInfo->buffersPerBlock; 

    block->allocCount = 0; 
    block->rank = kSIZE_NULL; 
    block->refCount = 1; 

    xkPoolAlloc_RemoveFromBlockList(object, list, block); 
    xkPoolAlloc_InsertIntoBlockList(object, &obj->unassignedBlockList, kNULL, block);

    return kOK; 
}

kFx(kStatus) xkPoolAlloc_AllocBlock(kPoolAlloc object)
{
    kObj(kPoolAlloc, object); 
    kAlloc innerAlloc = kObject_Alloc(object); 
    kSize allocSize = xkPoolAlloc_AllocBlockSize(object); 
    xkPoolAllocBlockHeader* block = kNULL; 

    if (((obj->blockCount + 1)*obj->nominalBlockSize > obj->blockCapacity) || 
        ((obj->totalSize + allocSize) > obj->totalCapacity))
    {
        return kERROR_MEMORY; 
    }

    kCheck(kAlloc_Get(innerAlloc, allocSize, &block)); 

    block->allocCount = 0; 
    block->refCount = 1; 
    block->rank = kSIZE_NULL; 
    block->next = kNULL; 
    block->previous = kNULL; 

    obj->blockCount++; 
    obj->totalSize += allocSize; 

    xkPoolAlloc_InsertIntoBlockList(object, &obj->unassignedBlockList, kNULL, block); 

    return kOK; 
}

kFx(kStatus) xkPoolAlloc_ClearEx(kPoolAlloc object, kBool all)
{
    kObj(kPoolAlloc, object); 
    kSize i; 

    kLock_Enter(obj->lock); 

    kTry
    {
        //clear reservations, if requested
        if (all)
        {            
            kPoolAlloc_Reserve(object, 0); 

            for (i = 0; i < obj->managedRankCount; ++i)
            {
                kPoolAlloc_ReserveAt(object, i, 0); 
            }
        }

        //blocks
        kTest(xkPoolAlloc_RecycleBlocks(object, kSIZE_MAX)); 
        kTest(xkPoolAlloc_FreeUnassignedBlocks(object)); 

        //cached buffers
        kTest(xkPoolAlloc_FreeCachedBuffers(object)); 
    }
    kFinally
    {
        kLock_Exit(obj->lock); 
        kEndFinally(); 
    }

    return kOK;
}

kFx(kStatus) xkPoolAlloc_AllocCachedBuffers(kPoolAlloc object, kSize rank, kSize count)
{
    kObj(kPoolAlloc, object); 
    xkPoolAllocRank* rankInfo = &obj->rankInfo[rank]; 
    kAlloc innerAlloc = kObject_Alloc(object); 
    kSize allocSize = sizeof(xkPoolAllocBufferHeader) + rankInfo->size; 
    kSize i; 

    for (i = 0; i < count; ++i)
    {
        xkPoolAllocBufferHeader* buffer = kNULL; 

        if ((obj->totalSize + allocSize) > obj->totalCapacity)
        {
            return kERROR_MEMORY; 
        }

        kCheck(kAlloc_Get(innerAlloc, allocSize, &buffer)); 

        buffer->type = obj->cacheAllocId; 
        buffer->info.rank = rank; 
        buffer->next = kNULL; 
        buffer->previous = kNULL; 

        xkPoolAlloc_InsertIntoBufferList(object, &rankInfo->freeBufferList, kNULL, buffer); 

        rankInfo->bufferCount++; 
        obj->cacheSize += rankInfo->size;          
        obj->totalSize += allocSize; 
    }

    return kOK; 
}

kFx(kStatus) xkPoolAlloc_AllocUnmanagedBuffer(kPoolAlloc object, kSize size, void* mem, kMemoryAlignment alignment)
{
    kObj(kPoolAlloc, object); 
    kAlloc innerAlloc = kObject_Alloc(object); 
    kSize allocSize = sizeof(xkPoolAllocBufferHeader) + size; 
    xkPoolAllocBufferHeader* buffer = kNULL; 

    if ((obj->totalSize + allocSize) > obj->totalCapacity)
    {
        return kERROR_MEMORY; 
    }

    kCheck(kAlloc_Get(innerAlloc, allocSize, &buffer)); 

    buffer->type = obj->unmanagedAllocId;
    buffer->info.size = size; 
    buffer->next = kNULL; 
    buffer->previous = kNULL; 

    obj->totalSize += allocSize; 

    *(void**)mem = xkPoolAlloc_AlignAllocation(object, buffer, alignment);

    return kOK; 
}

kFx(kStatus) xkPoolAlloc_FreeUnassignedBlocks(kPoolAlloc object)
{
    kObj(kPoolAlloc, object); 
    kAlloc internalAlloc = kObject_Alloc(object); 

    while ((obj->blockCount > obj->minBlockCount) && (obj->unassignedBlockList.count > 0))
    {
        xkPoolAllocBlockHeader* block = obj->unassignedBlockList.first; 

        xkPoolAlloc_RemoveFromBlockList(object, &obj->unassignedBlockList, block); 
        
        kCheck(kAlloc_Free(internalAlloc, block)); 

        obj->totalSize -= xkPoolAlloc_AllocBlockSize(object); 
        obj->blockCount--; 
    }

    return kOK; 
}

kFx(kStatus) xkPoolAlloc_FreeCachedBuffers(kPoolAlloc object)
{
    kObj(kPoolAlloc, object); 
    kAlloc internalAlloc = kObject_Alloc(object); 
    kSize i;

    for (i = obj->blockBasedRankCount; i < obj->cacheBasedRankCount; ++i)
    {
        xkPoolAllocRank* rankInfo = &obj->rankInfo[i]; 

        kAssert(rankInfo->isCached); 

        while ((rankInfo->bufferCount > rankInfo->minBufferCount) && (rankInfo->freeBufferList.count > 0))
        {
            xkPoolAllocBufferHeader* buffer = rankInfo->freeBufferList.first;

            xkPoolAlloc_RemoveFromBufferList(object, &rankInfo->freeBufferList, buffer); 

            kCheck(kAlloc_Free(internalAlloc, buffer)); 

            obj->cacheSize -= rankInfo->size; 
            obj->totalSize -= sizeof(xkPoolAllocBufferHeader) + rankInfo->size; 

            rankInfo->bufferCount--; 
        }
    }

    return kOK; 
}

kFx(kSize) kPoolAlloc_TotalSize(kPoolAlloc object)
{
    kObj(kPoolAlloc, object); 
    kSize size; 

    kLock_Enter(obj->lock); 
    {
        size = obj->totalSize; 
    }
    kLock_Exit(obj->lock); 

    return size; 
}

kFx(kSize) kPoolAlloc_BufferCountAt(kPoolAlloc object, kSize rank)
{
    kObj(kPoolAlloc, object); 
    kSize count = 0; 

    kLock_Enter(obj->lock); 
    {
        if (rank < obj->managedRankCount)
        {
            count += obj->rankInfo[rank].bufferCount; 
        }
    }
    kLock_Exit(obj->lock); 

    return count; 
}

/** 
 * @file    kMsgQueue.cpp
 *
 * @internal
 * Copyright (C) 2011-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <kApi/Threads/kMsgQueue.h>
#include <kApi/Threads/kLock.h>
#include <kApi/Threads/kSemaphore.h>

kBeginEnumEx(k, kMsgQueueItemOption)
    kAddEnumerator(kMsgQueueItemOption, kMSG_QUEUE_ITEM_OPTION_CRITICAL)
kEndEnumEx()

kBeginEnumEx(k, kMsgQueuePurgeOption)
    kAddEnumerator(kMsgQueuePurgeOption, kMSG_QUEUE_PURGE_OPTION_PRESERVE_CRITICAL)
    kAddEnumerator(kMsgQueuePurgeOption, kMSG_QUEUE_PURGE_OPTION_USE_HANDLER)
    kAddEnumerator(kMsgQueuePurgeOption, kMSG_QUEUE_PURGE_OPTION_DISPOSE_ITEMS)
    kAddEnumerator(kMsgQueuePurgeOption, kMSG_QUEUE_PURGE_OPTION_COUNT_DROPS)
kEndEnumEx()

kBeginClassEx(k, kMsgQueue)
    kAddPrivateVMethod(kMsgQueue, kObject, VRelease)
    kAddPrivateVMethod(kMsgQueue, kObject, VDisposeItems)
    kAddPrivateVMethod(kMsgQueue, kObject, VSize)
kEndClassEx()

kFx(kStatus) kMsgQueue_Construct(kMsgQueue* queue, kType itemType, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback(allocator);
    kStatus status; 

    kCheck(kAlloc_GetObject(alloc, kTypeOf(kMsgQueue), queue)); 

    if (!kSuccess(status = xkMsgQueue_Init(*queue, kTypeOf(kMsgQueue), itemType, alloc)))
    {
        kAlloc_FreeRef(alloc, queue); 
    }

    return status; 
} 

kFx(kStatus) xkMsgQueue_Init(kMsgQueue queue, kType type, kType itemType, kAlloc allocator)
{
    kObjR(kMsgQueue, queue); 
    kStatus status; 

    kCheck(kObject_Init(queue, type, allocator));     

    obj->entries = kNULL;
    obj->entrySize = 0;
    kZero(obj->itemField);
    obj->entryCapacity = 0;
    obj->entryDivisor = 1; 
    obj->count = 0;
    obj->pruneCount = 0;
    obj->size = 0;
    obj->maxCount = kSIZE_MAX; 
    obj->maxSize = kSIZE_MAX; 
    obj->head = 0;
    obj->canRemove = kNULL;
    obj->lock = kNULL;
    obj->onDrop.function = kNULL; 
    obj->onDrop.receiver = kNULL;
    obj->dropCount = 0;

    kTry
    {
        kTest(xkMsgQueue_Layout(queue, itemType)); 
        kTest(kSemaphore_Construct(&obj->canRemove, 0, allocator)); 
        kTest(kLock_Construct(&obj->lock, allocator)); 
    }
    kCatch(&status)
    {
        xkMsgQueue_VRelease(queue); 
        kEndCatch(status); 
    }    
    
    return kOK; 
}

kFx(kStatus) xkMsgQueue_Layout(kMsgQueue queue, kType itemType)
{
    kObj(kMsgQueue, queue); 
    xkStructField sizeField; 
    xkStructField optionsField; 
    xkStructField* fields[3];     

    sizeField.type = kTypeOf(kSize); 
    sizeField.offset = offsetof(xkMsgQueueEntryStruct, size); 
    sizeField.count = 1; 
    fields[0] = &sizeField; 

    optionsField.type = kTypeOf(kMsgQueueItemOption); 
    optionsField.offset = offsetof(xkMsgQueueEntryStruct, options); 
    optionsField.count = 1; 
    fields[1] = &optionsField; 

    obj->itemField.type = itemType; 
    obj->itemField.offset = kSIZE_NULL; 
    obj->itemField.count = 1; 
    fields[2] = &obj->itemField; 

    kCheck(xkType_LayoutStruct(fields, kCountOf(fields), &obj->entrySize)); 
 
    return kOK; 
}

kFx(kStatus) xkMsgQueue_VRelease(kMsgQueue queue)
{
    kObj(kMsgQueue, queue); 

    kCheck(kObject_FreeMem(queue, obj->entries)); 

    kCheck(kObject_Destroy(obj->canRemove)); 
    kCheck(kObject_Destroy(obj->lock));   

    kCheck(kObject_VRelease(queue)); 

    return kOK; 
}

kFx(kStatus) xkMsgQueue_VDisposeItems(kMsgQueue queue)
{
    kObj(kMsgQueue, queue); 
   
    return xkMsgQueue_Prune(queue, kMSG_QUEUE_PURGE_OPTION_DISPOSE_ITEMS, 0, 0); 
}

kFx(kSize) xkMsgQueue_VSize(kMsgQueue queue)
{
    kObj(kMsgQueue, queue); 
    kSize size = 0; 

    kLock_Enter(obj->lock);
    {
        size += sizeof(kMsgQueueClass) + obj->entryCapacity*obj->entrySize + obj->size; 
    }
    kLock_Exit(obj->lock); 

    return size; 
}

kFx(kStatus) kMsgQueue_SetDropHandler(kMsgQueue queue, kMsgQueueDropFx onDrop, kPointer receiver)
{
    kObj(kMsgQueue, queue); 

    obj->onDrop.function = (kCallbackFx) onDrop; 
    obj->onDrop.receiver = receiver; 

    return kOK; 
}

kFx(kStatus) kMsgQueue_SetMaxSize(kMsgQueue queue, kSize size)
{
    kObj(kMsgQueue, queue); 

    kLock_Enter(obj->lock); 

    kTry
    {
        obj->maxSize = size; 

        kTest(xkMsgQueue_Prune(queue, xkMSG_QUEUE_NORMAL_PRUNE_OPTIONS, obj->maxCount, obj->maxSize)); 
    }
    kFinally
    {
        kLock_Exit(obj->lock);        
        kEndFinally(); 
    }

    return kOK; 
}

kFx(kStatus) kMsgQueue_SetMaxCount(kMsgQueue queue, kSize count)
{
    kObj(kMsgQueue, queue); 
  
    kLock_Enter(obj->lock); 

    kTry
    {
        obj->maxCount = count; 

        kTest(xkMsgQueue_Prune(queue, xkMSG_QUEUE_NORMAL_PRUNE_OPTIONS, obj->maxCount, obj->maxSize)); 
    }
    kFinally
    {
        kLock_Exit(obj->lock);        
        kEndFinally(); 
    }

    return kOK; 
}

kFx(kStatus) kMsgQueue_Add(kMsgQueue queue, void* item)
{
    return kMsgQueue_AddEx(queue, item, kMSG_QUEUE_ITEM_OPTION_NULL); 
}

kFx(kStatus) kMsgQueue_AddEx(kMsgQueue queue, void* item, kMsgQueueItemOption options)
{
    kObj(kMsgQueue, queue); 
    xkMsgQueueEntryStruct* entryObj = kNULL; 
    kSize itemSize = 0; 
    kBool shouldPost = kFALSE; 
    
    kLock_Enter(obj->lock); 

    kTry
    {
        if (xkMsgQueue_ItemIsRef(queue))
        {
            kObject objectItem = kPointer_ReadAs(item, kObject); 
            itemSize = kIsNull(objectItem) ? 0 : kObject_Size(objectItem); 
        }
        else
        {
            itemSize = obj->itemField.fieldSize; 
        }

        if (obj->count == obj->entryCapacity)
        {
            kTest(kMsgQueue_Reserve(queue, obj->entryCapacity + 1)); 
        }
        
        entryObj = xkMsgQueue_EntryAt(queue, obj->count); 
        entryObj->size = itemSize; 
        entryObj->options = options;  
        kValue_Import(obj->itemField.type, xkMsgQueueEntry_Item(queue, entryObj), item); 

        obj->size += itemSize; 
        obj->count++;        
        shouldPost = kTRUE; 

        if ((obj->count > obj->maxCount) || (obj->size > obj->maxSize))
        {
            xkMsgQueue_Prune(queue, xkMSG_QUEUE_NORMAL_PRUNE_OPTIONS, obj->maxCount, obj->maxSize); 
        }
        
        if (obj->pruneCount > 0)
        {       
            shouldPost = kFALSE; 
            obj->pruneCount--; 
        }
    }
    kFinally
    {
        kLock_Exit(obj->lock);      

        if (shouldPost)
        {
            kCheck(kSemaphore_Post(obj->canRemove)); 
        }

        kEndFinally(); 
    }
   
    return kOK; 
}

kFx(kStatus) kMsgQueue_Remove(kMsgQueue queue, void* item, k64u timeout)
{
    kObj(kMsgQueue, queue); 
    kBool shouldContinue = kTRUE;     

    while (shouldContinue)
    {
        kCheck(kSemaphore_Wait(obj->canRemove, timeout)); 

        kLock_Enter(obj->lock); 
        {
            if (obj->pruneCount > 0)
            {
                obj->pruneCount--; 
            }
            else
            {
                xkMsgQueueEntryStruct* entryObj = xkMsgQueue_EntryAt(queue, 0); 
                kSize itemSize = entryObj->size; 

                shouldContinue = kFALSE; 

                if (item)
                {
                    kItemCopy(item, xkMsgQueueEntry_Item(queue, entryObj), obj->itemField.fieldSize); 
                }

                obj->size -= itemSize; 
                obj->head = (obj->head + 1) % obj->entryDivisor; 
                obj->count--; 
            }
        }
        kLock_Exit(obj->lock); 
    }  

    return kOK; 
} 

kFx(kStatus) kMsgQueue_Clear(kMsgQueue queue)
{
    return kMsgQueue_PurgeEx(queue, kMSG_QUEUE_PURGE_OPTION_NULL);
}

kFx(kStatus) kMsgQueue_Purge(kMsgQueue queue)
{
    return kMsgQueue_PurgeEx(queue, kMSG_QUEUE_PURGE_OPTION_DISPOSE_ITEMS); 
}

kFx(kStatus) kMsgQueue_PurgeEx(kMsgQueue queue, kMsgQueuePurgeOption options)
{
    kObj(kMsgQueue, queue); 
    
    kLock_Enter(obj->lock); 

    kTry
    {    
        kTest(xkMsgQueue_Prune(queue, options, 0, 0)); 
    }
    kFinally
    {
        kLock_Exit(obj->lock);        
        kEndFinally(); 
    }

    return kOK; 
}

kFx(kStatus) kMsgQueue_Reserve(kMsgQueue queue, kSize count)
{
    kObj(kMsgQueue, queue); 
    kSize mimumCapacity = count + 1;    //reserved one extra; allows add before prune
   
    kLock_Enter(obj->lock); 

    kTry
    {
        if (obj->entryCapacity < mimumCapacity)
        {
            kSize newCapacity = 0; 
            void* newItems = kNULL;  

            newCapacity = kMax_(mimumCapacity, xkMSG_QUEUE_GROWTH_FACTOR*obj->entryCapacity);
            newCapacity = kMax_(newCapacity, xkMSG_QUEUE_MIN_CAPACITY); 

            kTest(kObject_GetMem(queue, newCapacity*obj->entrySize, &newItems)); 

            if (obj->count > 0)
            {
                kSize headCount = xkMsgQueue_HeadCount(queue); 
                kSize tailCount = xkMsgQueue_TailCount(queue); 
                kSize entrySize = obj->entrySize; 

                kMemCopy(kPointer_ItemOffset(newItems, 0, entrySize), xkMsgQueue_EntryAt(queue, 0), headCount*entrySize); 
                kMemCopy(kPointer_ItemOffset(newItems, (kSSize)headCount, entrySize), xkMsgQueue_EntryAt(queue, headCount), tailCount*entrySize); 
            }

            kObject_FreeMem(queue, obj->entries); 

            obj->entries = newItems; 
            obj->entryCapacity = newCapacity; 
            obj->entryDivisor = kMax_(1, obj->entryCapacity); 
            obj->head = 0; 
        }
    }
    kFinally
    {
        kLock_Exit(obj->lock); 
        kEndFinally(); 
    }
    
    return kOK; 
}

kFx(kStatus) xkMsgQueue_Prune(kMsgQueue queue, kMsgQueuePurgeOption options, kSize maxCount, kSize maxSize)
{
    kObj(kMsgQueue, queue); 
    kBool preserveCritical = ((options & kMSG_QUEUE_PURGE_OPTION_PRESERVE_CRITICAL) != 0);
    kBool itemIsRef = xkMsgQueue_ItemIsRef(queue); 
    kSize criticalCount = 0; 
    kSize itEnd = obj->count; 
    kSize it = 0; 
    kSize i = 0; 
    kSize dropCount = 0; 
    kMsgQueueDropArgs args; 

    while (((obj->count > maxCount) || (obj->size > maxSize)) && (it != itEnd))
    {
        xkMsgQueueEntryStruct* entryObj = xkMsgQueue_EntryAt(queue, it); 
        void* entryItem = xkMsgQueueEntry_Item(queue, entryObj); 

        if (preserveCritical && ((entryObj->options & kMSG_QUEUE_ITEM_OPTION_CRITICAL) != 0))
        {
            xkMsgQueueEntryStruct* dst = xkMsgQueue_EntryAt(queue, criticalCount); 

            if (dst != entryObj)
            {
                kItemCopy(dst, entryObj, obj->entrySize); 
            }
            criticalCount++; 
        }
        else
        {
            if (!kIsNull(obj->onDrop.function) && ((options & kMSG_QUEUE_PURGE_OPTION_USE_HANDLER) != 0))
            {
                args.item = entryItem;
                obj->onDrop.function(obj->onDrop.receiver, queue, &args);
            }
            else if (itemIsRef && ((options & kMSG_QUEUE_PURGE_OPTION_DISPOSE_ITEMS) != 0))
            {
                kDisposeRef((kObject*) entryItem);
            }

            obj->size -= entryObj->size;
            obj->count--;
            obj->pruneCount++;
            dropCount++; 
        }

        it++; 
    }

    for (i = 0; i < criticalCount; ++i)
    {
        xkMsgQueueEntryStruct* src = xkMsgQueue_EntryAt(queue, criticalCount - i - 1); 
        xkMsgQueueEntryStruct* dst = xkMsgQueue_EntryAt(queue, it - i - 1); 

        kItemCopy(dst, src, obj->entrySize); 
    }

    obj->head = (obj->head + dropCount) % obj->entryDivisor; 

    if ((options & kMSG_QUEUE_PURGE_OPTION_COUNT_DROPS) != 0)
    {
        obj->dropCount += dropCount; 
    }

    return kOK; 
}

kFx(kSize) kMsgQueue_Count(kMsgQueue queue)
{
    kObj(kMsgQueue, queue); 
    kSize count = 0; 

    kLock_Enter(obj->lock); 
    {
        count = obj->count; 
    }
    kLock_Exit(obj->lock); 
    
    return count; 
}

kFx(kSize) kMsgQueue_MaxSize(kMsgQueue queue)
{
    kObj(kMsgQueue, queue); 
    kSize size = 0; 

    kLock_Enter(obj->lock); 
    {
        size = obj->maxSize; 
    }
    kLock_Exit(obj->lock); 
    
    return size; 
}

kFx(kSize) kMsgQueue_MaxCount(kMsgQueue queue)
{
    kObj(kMsgQueue, queue); 
    kSize count = 0; 

    kLock_Enter(obj->lock); 
    {
        count = obj->maxCount; 
    }
    kLock_Exit(obj->lock); 
    
    return count; 
}

kFx(kType) kMsgQueue_ItemType(kMsgQueue queue)
{
    kObj(kMsgQueue, queue); 
    
    return obj->itemField.type;
}

kFx(kSize) kMsgQueue_ItemSize(kQueue queue)
{
    kObj(kMsgQueue, queue);

    return obj->itemField.typeSize;
}

kFx(kSize) kMsgQueue_DataSize(kMsgQueue queue)
{
    kObj(kMsgQueue, queue); 
    kSize size = 0; 

    kLock_Enter(obj->lock); 
    {
        size = obj->size; 
    }
    kLock_Exit(obj->lock); 
    
    return size; 
}

kFx(k64u) kMsgQueue_DropCount(kMsgQueue queue)
{
    kObj(kMsgQueue, queue); 
    k64u dropCount = 0; 

    kLock_Enter(obj->lock); 
    {
        dropCount = obj->dropCount; 
    }
    kLock_Exit(obj->lock); 
    
    return dropCount; 
}

/** 
 * @file    kMsgQueue.x.h
 *
 * @internal
 * Copyright (C) 2011-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_MSG_QUEUE_X_H
#define K_API_MSG_QUEUE_X_H

#include <kApi/Threads/kLock.h>

#define xkMSG_QUEUE_GROWTH_FACTOR                (2)
#define xkMSG_QUEUE_MIN_CAPACITY                 (16)

#define xkMSG_QUEUE_NORMAL_PRUNE_OPTIONS         (kMSG_QUEUE_PURGE_OPTION_PRESERVE_CRITICAL |    \
                                                  kMSG_QUEUE_PURGE_OPTION_USE_HANDLER |          \
                                                  kMSG_QUEUE_PURGE_OPTION_DISPOSE_ITEMS |        \
                                                  kMSG_QUEUE_PURGE_OPTION_COUNT_DROPS)

kDeclareEnumEx(k, kMsgQueueItemOption, kValue)
kDeclareEnumEx(k, kMsgQueuePurgeOption, kValue)

typedef void* xkMsgQueueEntry; 

typedef struct xkMsgQueueEntryStruct//<T>
{
    kSize size;                       //cached item size (for ref types, kObject_Size result)
    kMsgQueueItemOption options;      //item flags
    //T item;                         //queue item
} xkMsgQueueEntryStruct; 

typedef struct kMsgQueueClass
{
    kObjectClass base; 
    void* entries;                      //data buffer
    kSize entrySize;                    //size of xkMsgQueueEntry<T> instance
    xkStructField itemField;             //information about generic item field
    kSize entryCapacity;                //maximum count of items before reallocation
    kSize entryDivisor;                 //use for index calculation 
    kSize count;                        //current count of items 
    kSize pruneCount;                   //current count of pruned items 
    kSize size;                         //total size of current items 
    kSize maxCount;                     //maximum count of items 
    kSize maxSize;                      //maximum total size of items 
    kSize head;                         //current read index 
    kSemaphore canRemove;               //counts available items 
    kLock lock;                         //mutual exclusion lock 
    kCallback onDrop;                   //drop handler
    k64u dropCount;                     //number of items dropped 
} kMsgQueueClass;

kDeclareClassEx(k, kMsgQueue, kObject)

/* 
* Forward declarations. 
*/

kFx(kStatus) kMsgQueue_Add(kMsgQueue queue, void* item);
kFx(kStatus) kMsgQueue_AddEx(kMsgQueue queue, void* item, kMsgQueueItemOption options);
kFx(kStatus) kMsgQueue_Remove(kMsgQueue queue, void* item, k64u timeout);
kFx(kType) kMsgQueue_ItemType(kMsgQueue queue);

/* 
* Private methods. 
*/

kFx(kStatus) xkMsgQueue_Init(kMsgQueue queue, kType type, kType itemType, kAlloc allocator); 
kFx(kStatus) xkMsgQueue_VRelease(kMsgQueue queue); 
kFx(kStatus) xkMsgQueue_VDisposeItems(kMsgQueue queue); 
kFx(kSize) xkMsgQueue_VSize(kMsgQueue queue); 

kFx(kStatus) xkMsgQueue_Layout(kMsgQueue queue, kType itemType); 

kFx(kStatus) xkMsgQueue_Prune(kMsgQueue queue, kMsgQueuePurgeOption options, kSize maxCount, kSize maxSize); 

kInlineFx(kBool) xkMsgQueue_IsSplit(kMsgQueue queue)
{
    kObj(kMsgQueue, queue); 

    return (obj->head + obj->count) > obj->entryCapacity; 
}

kInlineFx(kSize) xkMsgQueue_HeadCount(kMsgQueue queue)
{
    kObj(kMsgQueue, queue); 

    if (xkMsgQueue_IsSplit(queue))      return obj->entryCapacity - obj->head; 
    else                                return obj->count;
}

kInlineFx(kSize) xkMsgQueue_TailCount(kMsgQueue queue)
{
    kObj(kMsgQueue, queue); 

    if (xkMsgQueue_IsSplit(queue))      return obj->head + obj->count - obj->entryCapacity;  
    else                                return 0; 
}

kInlineFx(kBool) xkMsgQueue_ItemIsRef(kMsgQueue queue)
{
    kObj(kMsgQueue, queue); 

    return kType_IsReference(obj->itemField.type);
}

kInlineFx(xkMsgQueueEntryStruct*) xkMsgQueue_EntryAt(kMsgQueue queue, kSize index)
{
    kObj(kMsgQueue, queue); 
    kSize entryIndex = (obj->head + index) % obj->entryDivisor;

    return (xkMsgQueueEntryStruct*) kPointer_ItemOffset(obj->entries, (kSSize)entryIndex, obj->entrySize);
}

kInlineFx(kPointer) xkMsgQueueEntry_Item(kMsgQueue queue, xkMsgQueueEntryStruct* entry)
{
    kObj(kMsgQueue, queue); 
   
    return kPointer_ByteOffset(entry, (kSSize)obj->itemField.offset);
}

kInlineFx(kCallback) xkMsgQueue_DropHandler(kMsgQueue queue)
{
    kObj(kMsgQueue, queue); 

    return obj->onDrop;
}

kInlineFx(kStatus) xkMsgQueue_AddT(kMsgQueue queue, void* item, kSize itemSize)
{
    kAssert(xkType_IsPointerCompatible(kMsgQueue_ItemType(queue), itemSize)); 

    return kMsgQueue_Add(queue, item);
} 

kInlineFx(kStatus) xkMsgQueue_AddExT(kMsgQueue queue, void* item, kMsgQueueItemOption options, kSize itemSize)
{
    kAssert(xkType_IsPointerCompatible(kMsgQueue_ItemType(queue), itemSize)); 

    return kMsgQueue_AddEx(queue, item, options);
} 

kInlineFx(kStatus) xkMsgQueue_RemoveT(kMsgQueue queue, void* item, k64u timeout, kSize itemSize)
{
    kAssert(xkType_IsPointerCompatible(kMsgQueue_ItemType(queue), itemSize)); 

    return kMsgQueue_Remove(queue, item, timeout);
} 

#endif

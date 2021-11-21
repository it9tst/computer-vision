/** 
 * @file    kQueue.x.h
 *
 * @internal
 * Copyright (C) 2005-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_QUEUE_X_H
#define K_API_QUEUE_X_H

#define xkQUEUE_MIN_CAPACITY                (16)
#define xkQUEUE_GROWTH_FACTOR               (2)

typedef struct kQueueClass
{
    kObjectClass base; 
    kType itemType;             //item type
    kSize itemSize;             //item size, in bytes
    kSize allocSize;            //size of allocated array memory, in bytes
    void* items;                //item array
    kSize count;                //current number of elements
    kSize head;                 //array index of first item
    kSize capacity;             //maximum elements before reallocation
    kBool isAttached;           //is array memory externally owned?
} kQueueClass;

kDeclareClassEx(k, kQueue, kObject) 

/* 
* Forward declarations. 
*/

kFx(kStatus) kQueue_Add(kQueue queue, const void* item); 
kFx(kStatus) kQueue_SetItem(kQueue queue, kSize index, const void* item); 
kFx(kStatus) kQueue_Item(kQueue queue, kSize index, void* item);  
kFx(kStatus) kQueue_Purge(kQueue queue); 

kInlineFx(kType) kQueue_ItemType(kQueue queue);
kInlineFx(kSize) kQueue_Count(kQueue queue);
kInlineFx(kSize) kQueue_Capacity(kQueue queue);
kInlineFx(kSize) kQueue_ItemSize(kQueue queue);
kInlineFx(void*) kQueue_DataAt(kQueue queue, kSize index);
kInlineFx(void*) kQueue_At(kQueue queue, kSize index); 

/* 
* Private methods. 
*/

kFx(kStatus) xkQueue_ConstructFramework(kQueue* queue, kAlloc allocator); 

kFx(kStatus) xkQueue_Init(kQueue queue, kType classType, kType itemType, kSize initialCapacity, kAlloc alloc); 

kFx(kStatus) xkQueue_VClone(kQueue queue, kQueue source, kAlloc valueAlloc, kObject context); 

kFx(kStatus) xkQueue_VRelease(kQueue queue); 

kInlineFx(kStatus) xkQueue_VDisposeItems(kQueue queue)
{
    return kQueue_Purge(queue);
}

kFx(kBool) xkQueue_VHasShared(kQueue queue); 
kFx(kSize) xkQueue_VSize(kQueue queue); 
kFx(kAllocTrait) xkQueue_VAllocTraits(kQueue queue);    

kFx(kStatus) xkQueue_WriteDat5V1(kQueue queue, kSerializer serializer); 
kFx(kStatus) xkQueue_ReadDat5V1(kQueue queue, kSerializer serializer); 

kFx(kStatus) xkQueue_WriteDat6V0(kQueue queue, kSerializer serializer); 
kFx(kStatus) xkQueue_ReadDat6V0(kQueue queue, kSerializer serializer); 

kFx(kStatus) xkQueue_Resize(kQueue queue, kSize count); 

kFx(kIterator) xkQueue_GetIterator(kQueue queue); 
kFx(kBool) xkQueue_HasNext(kQueue queue, kIterator iterator); 
kFx(void*) xkQueue_Next(kQueue queue, kIterator* iterator); 

kInlineFx(kBool) xkQueue_IsSplit(kQueue queue)
{
    kObj(kQueue, queue); 

    return (obj->head + obj->count) > obj->capacity;
}

kInlineFx(kSize) xkQueue_HeadCount(kQueue queue)
{
    kObj(kQueue, queue); 

    if (xkQueue_IsSplit(queue))
    {
        return obj->capacity - obj->head; 
    }
    else
    {
        return obj->count;
    }
}

kInlineFx(kSize) xkQueue_TailCount(kQueue queue)
{
    kObj(kQueue, queue); 

    if (xkQueue_IsSplit(queue))
    {
        return obj->head + obj->count - obj->capacity; 
    }
    else
    {
        return 0;
    }
}

kInlineFx(kSize) xkQueue_DataSize(kQueue queue)
{
    return kQueue_Count(queue) * kQueue_ItemSize(queue);
}

kInlineFx(kStatus) xkQueue_AddT(kQueue queue, const void* item, kSize itemSize)
{
    kAssert(xkType_IsPointerCompatible(kQueue_ItemType(queue), itemSize)); 

    return kQueue_Add(queue, item);
} 

kInlineFx(kStatus) xkQueue_SetItemT(kQueue queue, kSize index, const void* item, kSize itemSize)
{
    kAssert(xkType_IsPointerCompatible(kQueue_ItemType(queue), itemSize)); 

    return kQueue_SetItem(queue, index, item);
} 

kInlineFx(kStatus) xkQueue_ItemT(kQueue queue, kSize index, void* item, kSize itemSize)
{
    kAssert(xkType_IsPointerCompatible(kQueue_ItemType(queue), itemSize)); 

    return kQueue_Item(queue, index, item);
} 

kInlineFx(void*) xkQueue_DataAtT(kQueue queue, kSize index, kSize itemSize)
{
    kAssert(xkType_IsPointerCompatible(kQueue_ItemType(queue), itemSize)); 

    return kQueue_DataAt(queue, index);
} 

kInlineFx(void*) xkQueue_AtT(kQueue queue, kSize index, kSize itemSize)
{
    kAssert(xkType_IsPointerCompatible(kQueue_ItemType(queue), itemSize)); 

    return kQueue_At(queue, index);
} 

kInlineFx(void*) xkQueue_AsT(kQueue queue, kSize index, kSize itemSize)
{
    kAssert(itemSize == kQueue_ItemSize(queue)); 

    return kQueue_At(queue, index);
}

/* 
* Experimental (FSS-993)
*/

kFx(kStatus) kQueue_ConstructEx(kQueue* queue, kType itemType, kSize initialCapacity, kAlloc allocator, kAlloc valueAllocator); 

#endif

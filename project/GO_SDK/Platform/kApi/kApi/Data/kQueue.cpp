/** 
 * @file    kQueue.cpp
 *
 * @internal
 * Copyright (C) 2005-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <kApi/Data/kQueue.h>
#include <kApi/Data/kCollection.h>
#include <kApi/Io/kSerializer.h>

kBeginClassEx(k, kQueue) 
    
    //serialization versions
    kAddPrivateVersionEx(kQueue, "kdat5", "5.0.0.0", "42-1", WriteDat5V1, ReadDat5V1)
    kAddPrivateVersionEx(kQueue, "kdat6", "5.7.1.0", "kQueue-0", WriteDat6V0, ReadDat6V0)

    //special constructors
    kAddPrivateFrameworkConstructor(kQueue, ConstructFramework)

    //virtual methods
    kAddPrivateVMethod(kQueue, kObject, VRelease)
    kAddPrivateVMethod(kQueue, kObject, VDisposeItems)
    kAddPrivateVMethod(kQueue, kObject, VClone)
    kAddPrivateVMethod(kQueue, kObject, VHasShared)
    kAddPrivateVMethod(kQueue, kObject, VSize)
    kAddPrivateVMethod(kQueue, kObject, VAllocTraits)

    //collection interface 
    kAddInterface(kQueue, kCollection)
    kAddPrivateIVMethod(kQueue, kCollection, VGetIterator, GetIterator)
    kAddIVMethod(kQueue, kCollection, VItemType, ItemType)
    kAddIVMethod(kQueue, kCollection, VCount, Count)
    kAddPrivateIVMethod(kQueue, kCollection, VHasNext, HasNext)
    kAddPrivateIVMethod(kQueue, kCollection, VNext, Next)

kEndClassEx() 

kFx(kStatus) kQueue_Construct(kQueue* queue, kType itemType, kSize initialCapacity, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback(allocator);
    kStatus status; 

    kCheck(kAlloc_GetObject(alloc, kTypeOf(kQueue), queue)); 

    if (!kSuccess(status = xkQueue_Init(*queue, kTypeOf(kQueue), itemType, initialCapacity, alloc)))
    {
        kAlloc_FreeRef(alloc, queue); 
    }

    return status; 
} 

kFx(kStatus) xkQueue_ConstructFramework(kQueue* queue, kAlloc allocator)
{
    return kQueue_Construct(queue, kNULL, 0, allocator);
}

kFx(kStatus) xkQueue_Init(kQueue queue, kType classType, kType itemType, kSize initialCapacity, kAlloc alloc)
{
    kObjR(kQueue, queue);  
    kStatus status = kOK; 

    kCheck(kObject_Init(queue, classType, alloc)); 

    obj->itemType = kIsNull(itemType) ? kTypeOf(kVoid) : itemType; 
    obj->itemSize = kType_Size(obj->itemType); 
    obj->allocSize = 0; 
    obj->items = kNULL; 
    obj->count = 0; 
    obj->head = 0; 
    obj->capacity = 0; 
    obj->isAttached = kFALSE; 

    kTry
    {
        kTest(kQueue_Reserve(queue, initialCapacity)); 
    }
    kCatch(&status)
    {
        xkQueue_VRelease(queue); 
        kEndCatch(status); 
    }

    return kOK; 
}

kFx(kStatus) xkQueue_VClone(kQueue queue, kQueue source, kAlloc valueAlloc, kObject context)
{
    kObj(kQueue, queue);  
    kAlloc objectAlloc = kObject_Alloc(queue);
    kSize headCount = xkQueue_HeadCount(source); 
    kSize tailCount = xkQueue_TailCount(source); 
    kSize count = headCount + tailCount;

    kCheck(kQueue_Allocate(queue, kQueue_ItemType(source), count));     
    kCheck(xkQueue_Resize(queue, count)); 

    kCheck(kCloneItems(obj->itemType, kObject_Alloc(queue), kQueue_DataAt(queue, 0), kObject_Alloc(source), kQueue_DataAt(source, 0), headCount, context, objectAlloc, valueAlloc)); 
    kCheck(kCloneItems(obj->itemType, kObject_Alloc(queue), kQueue_DataAt(queue, headCount), kObject_Alloc(source), kQueue_DataAt(source, headCount), tailCount, context, objectAlloc, valueAlloc)); 

    return kOK; 
}

kFx(kStatus) xkQueue_VRelease(kQueue queue)
{
    kObj(kQueue, queue); 

    kCheck(kAlloc_Free(kObject_Alloc(queue), obj->items)); 

    kCheck(kObject_VRelease(queue)); 

    return kOK; 
}

kFx(kBool) xkQueue_VHasShared(kQueue queue)
{
    kObj(kQueue, queue); 
    kSize headCount = xkQueue_HeadCount(queue);
    kSize tailCount = xkQueue_TailCount(queue); 

    return kObject_IsShared(queue) || 
           kHasSharedItems(obj->itemType, kQueue_DataAt(queue, 0), headCount) || 
           kHasSharedItems(obj->itemType, kQueue_DataAt(queue, headCount), tailCount);  
}

kFx(kSize) xkQueue_VSize(kQueue queue)
{
    kObj(kQueue, queue); 
    kSize dataSize = (!obj->isAttached) ? obj->allocSize : xkQueue_DataSize(queue); 
    kSize size = sizeof(kQueueClass) + dataSize; 
    kSize headCount = xkQueue_HeadCount(queue); 
    kSize tailCount = xkQueue_TailCount(queue); 

    size += kMeasureItems(obj->itemType, kQueue_DataAt(queue, 0), headCount); 
    size += kMeasureItems(obj->itemType, kQueue_DataAt(queue, headCount), tailCount); 

    return size; 
}

kFx(kAllocTrait) xkQueue_VAllocTraits(kQueue queue)
{
    kObj(kQueue, queue); 
    kSize headCount = xkQueue_HeadCount(queue); 
    kSize tailCount = xkQueue_TailCount(queue); 

    return kAlloc_Traits(kObject_Alloc(queue)) | 
           kEnumerateAllocTraits(obj->itemType, kQueue_DataAt(queue, 0), headCount) | 
           kEnumerateAllocTraits(obj->itemType, kQueue_DataAt(queue, headCount), tailCount); 
}

kFx(kStatus) xkQueue_WriteDat5V1(kQueue queue, kSerializer serializer)
{
    kObj(kQueue, queue); 
    kSize headCount = xkQueue_HeadCount(queue); 
    kSize tailCount = xkQueue_TailCount(queue); 
    kTypeVersion itemVersion; 

    kCheck(kSerializer_WriteSize(serializer, obj->count)); 
    kCheck(kSerializer_WriteSize(serializer, obj->head)); 
    kCheck(kSerializer_WriteSize(serializer, obj->capacity)); 
    kCheck(kSerializer_Write32s(serializer, kTRUE));  //isDynamic

    kCheck(kSerializer_WriteSize(serializer, headCount)); 
    kCheck(kSerializer_WriteType(serializer, obj->itemType, &itemVersion));
    kCheck(kSerializer_WriteItems(serializer, obj->itemType, itemVersion, kQueue_DataAt(queue, 0), headCount)); 

    kCheck(kSerializer_WriteSize(serializer, tailCount)); 
    kCheck(kSerializer_WriteType(serializer, obj->itemType, &itemVersion));
    kCheck(kSerializer_WriteItems(serializer, obj->itemType, itemVersion, kQueue_DataAt(queue, headCount), tailCount)); 

    return kOK; 
}

kFx(kStatus) xkQueue_ReadDat5V1(kQueue queue, kSerializer serializer)
{
    kSize count = 0, head = 0, headCount = 0, tailCount = 0, capacity = 0; 
    kTypeVersion itemVersion;
    k32s isDynamic; 
    kType itemType = kNULL;            
 
    kCheck(kSerializer_ReadSize(serializer, &count));
    kCheck(kSerializer_ReadSize(serializer, &head));
    kCheck(kSerializer_ReadSize(serializer, &capacity));      
    kCheck(kSerializer_Read32s(serializer, &isDynamic));

    kCheck(kSerializer_ReadSize(serializer, &headCount));
    kCheck(kSerializer_ReadType(serializer, &itemType, &itemVersion)); 

    kCheck(kQueue_Allocate(queue, itemType, count)); 

    kCheck(xkQueue_Resize(queue, count)); 

    kCheck(kSerializer_ReadItems(serializer, itemType, itemVersion, kQueue_DataAt(queue, 0), headCount)); 

    kCheck(kSerializer_ReadSize(serializer, &tailCount));
    kCheck(kSerializer_ReadType(serializer, &itemType, &itemVersion)); 

    kCheck(kSerializer_ReadItems(serializer, itemType, itemVersion, kQueue_DataAt(queue, headCount), tailCount)); 

    return kOK; 
}

kFx(kStatus) xkQueue_WriteDat6V0(kQueue queue, kSerializer serializer)
{
    kObj(kQueue, queue); 
    kSize headCount = xkQueue_HeadCount(queue); 
    kSize tailCount = xkQueue_TailCount(queue); 
    kTypeVersion itemVersion; 

    kCheck(kSerializer_WriteType(serializer, obj->itemType, &itemVersion));
    kCheck(kSerializer_WriteSize(serializer, obj->count)); 

    kCheck(kSerializer_WriteItems(serializer, obj->itemType, itemVersion, kQueue_DataAt(queue, 0), headCount)); 
    kCheck(kSerializer_WriteItems(serializer, obj->itemType, itemVersion, kQueue_DataAt(queue, headCount), tailCount)); 

    return kOK; 
}

kFx(kStatus) xkQueue_ReadDat6V0(kQueue queue, kSerializer serializer)
{
    kObj(kQueue, queue); 
    kType itemType = kNULL;            
    kTypeVersion itemVersion; 
    kSize count = 0; 
 
    kCheck(kSerializer_ReadType(serializer, &itemType, &itemVersion));
    kCheck(kSerializer_ReadSize(serializer, &count)); 

    kCheck(kQueue_Allocate(queue, itemType, count)); 

    kCheck(xkQueue_Resize(queue, count)); 

    kCheck(kSerializer_ReadItems(serializer, itemType, itemVersion, obj->items, count)); 

    return kOK; 
}

kFx(kStatus) kQueue_Allocate(kQueue queue, kType itemType, kSize initialCapacity)
{
    kObj(kQueue, queue); 

    if (obj->isAttached)
    {
        obj->items = kNULL; 
        obj->isAttached = kFALSE; 
    }
    
    obj->itemType = kIsNull(itemType) ? kTypeOf(kVoid) : itemType; 
    obj->itemSize = kType_Size(obj->itemType); 
    obj->count = 0; 
    obj->head = 0; 
    obj->capacity = 0; 

    kCheck(kQueue_Reserve(queue, initialCapacity)); 

    return kOK; 
}

kFx(kStatus) kQueue_Assign(kQueue queue, kQueue source)
{
    kObj(kQueue, queue); 
    kObjN(kQueue, sourceObj, source);  
    kSize headCount = xkQueue_HeadCount(source); 
    kSize tailCount = xkQueue_TailCount(source); 

    kCheck(kQueue_Allocate(queue, sourceObj->itemType, sourceObj->count)); 

    kCheck(xkQueue_Resize(queue, sourceObj->count)); 

    kCheck(kCopyItems(obj->itemType, kQueue_DataAt(queue, 0), kQueue_DataAt(source, 0), headCount)); 
    kCheck(kCopyItems(obj->itemType, kQueue_DataAt(queue, headCount), kQueue_DataAt(source, headCount), tailCount)); 

    return kOK;   
}

kFx(kStatus) kQueue_Purge(kQueue queue)
{
    kObj(kQueue, queue); 
    kSize headCount = xkQueue_HeadCount(queue); 
    kSize tailCount = xkQueue_TailCount(queue); 

    kCheck(kDisposeItems(obj->itemType, kQueue_DataAt(queue, 0), headCount)); 
    kCheck(kDisposeItems(obj->itemType, kQueue_DataAt(queue, headCount), tailCount)); 

    kCheck(kQueue_Clear(queue)); 

    return kOK; 
}

kFx(kStatus) kQueue_Zero(kQueue queue)
{
    kObj(kQueue, queue); 
    kSize headCount = xkQueue_HeadCount(queue); 
    kSize tailCount = xkQueue_TailCount(queue); 

    kCheck(kZeroItems(obj->itemType, kQueue_DataAt(queue, 0), headCount));
    kCheck(kZeroItems(obj->itemType, kQueue_DataAt(queue, headCount), tailCount));

    return kOK; 
}

kFx(kStatus) kQueue_Reserve(kQueue queue, kSize minimumCapacity)
{
    kObj(kQueue, queue); 

    if (obj->capacity < minimumCapacity)
    {
        kSize grownCapacity = kMax_(minimumCapacity, xkQUEUE_GROWTH_FACTOR*obj->capacity);
        kSize newCapacity = kMax_(grownCapacity, xkQUEUE_MIN_CAPACITY); 
        kSize newSize = kMax_(obj->allocSize, newCapacity*obj->itemSize); 

        if (newSize > obj->allocSize)
        {
            void* oldItems = obj->items; 
            void* newItems = kNULL; 

            kCheck(kAlloc_Get(kObject_Alloc(queue), newSize, &newItems));

            if (obj->count > 0)
            {
                kSize headCount = xkQueue_HeadCount(queue); 
                kSize tailCount = xkQueue_TailCount(queue); 

                kMemCopy(kPointer_ItemOffset(newItems, 0, obj->itemSize), kQueue_DataAt(queue, 0), headCount*obj->itemSize); 
                kMemCopy(kPointer_ItemOffset(newItems, (kSSize)headCount, obj->itemSize), kQueue_DataAt(queue, headCount), tailCount*obj->itemSize); 
            }

            obj->items = newItems; 
            obj->allocSize = newSize; 
            obj->head = 0; 

            kCheck(kAlloc_Free(kObject_Alloc(queue), oldItems));
        }

        obj->capacity = newCapacity;
    }
    
    return kOK; 
}

kFx(kStatus) xkQueue_Resize(kQueue queue, kSize count)
{
    kObj(kQueue, queue); 
    kSize i; 

    if (count > obj->capacity)
    {
        kCheck(kQueue_Reserve(queue, count)); 
    }

    if (kType_IsReference(obj->itemType) && (count > obj->count))
    {
        for (i = obj->count; i < count; ++i)
        {
            kItemZero(kQueue_DataAt(queue, i), obj->itemSize); 
        }
    }

    obj->count = count; 
    
    return kOK; 
}

kFx(kStatus) kQueue_Add(kQueue queue, const void* item)
{
    kObj(kQueue, queue); 

    if (obj->count == obj->capacity)
    {
        kCheck(kQueue_Reserve(queue, obj->capacity + 1)); 
    }

    kValue_Import(obj->itemType, kQueue_DataAt(queue, obj->count), item); 

    obj->count++; 

    return kOK; 
}

kFx(kStatus) kQueue_SetItem(kQueue queue, kSize index, const void* item)
{
    kObj(kQueue, queue); 

    kCheckArgs(index < obj->count); 

    kValue_Import(obj->itemType, kQueue_DataAt(queue, index), item); 

    return kOK; 
}

kFx(kStatus) kQueue_Item(kQueue queue, kSize index, void* item)
{
    kObj(kQueue, queue); 

    kCheckArgs(index < obj->count); 

    kItemCopy(item, kQueue_DataAt(queue, index), obj->itemSize); 

    return kOK; 
}

kFx(kStatus) kQueue_Remove(kQueue queue, void* item)
{
    kObj(kQueue, queue); 

    kCheckArgs(obj->count > 0); 

    if (!kIsNull(item))
    {
        kItemCopy(item, kQueue_DataAt(queue, 0), obj->itemSize); 
    }

    obj->count--; 
    obj->head = (obj->head + 1) % obj->capacity; 

    return kOK; 
}


kFx(kIterator) xkQueue_GetIterator(kQueue queue)
{
    return (kSize) 0; 
}

kFx(kBool) xkQueue_HasNext(kQueue queue, kIterator iterator)
{
    kObj(kQueue, queue); 
    kSize it = (kSize) iterator; 
    kSize end = (kSize) obj->count; 
    
    return (it != end); 
}

kFx(void*) xkQueue_Next(kQueue queue, kIterator* iterator)
{
    kObj(kQueue, queue); 
    kSize it = (kSize) iterator; 
    void* next = kQueue_DataAt(obj, it); 
       
    *iterator = (kPointer)(kSize)(it + 1); 

    return next; 
}


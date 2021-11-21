/** 
 * @file    kArrayList.cpp
 *
 * @internal
 * Copyright (C) 2005-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <kApi/Data/kArrayList.h>
#include <kApi/Data/kCollection.h>
#include <kApi/Io/kSerializer.h>

kBeginClassEx(k, kArrayList) 
    
    //serialization versions
    kAddPrivateVersionEx(kArrayList, "kdat5", "5.0.0.0", "27-1", WriteDat5V1, ReadDat5V1)
    kAddPrivateVersionEx(kArrayList, "kdat6", "5.7.1.0", "kArrayList-0", WriteDat6V0, ReadDat6V0)

    //special constructors
    kAddPrivateFrameworkConstructor(kArrayList, ConstructFramework)

    //virtual methods
    kAddPrivateVMethod(kArrayList, kObject, VRelease)
    kAddPrivateVMethod(kArrayList, kObject, VDisposeItems)
    kAddPrivateVMethod(kArrayList, kObject, VClone)
    kAddPrivateVMethod(kArrayList, kObject, VHasShared)
    kAddPrivateVMethod(kArrayList, kObject, VSize)
    kAddPrivateVMethod(kArrayList, kObject, VAllocTraits)

    //collection interface 
    kAddInterface(kArrayList, kCollection)
    kAddPrivateIVMethod(kArrayList, kCollection, VGetIterator, GetIterator)
    kAddIVMethod(kArrayList, kCollection, VItemType, ItemType)
    kAddIVMethod(kArrayList, kCollection, VCount, Count)
    kAddPrivateIVMethod(kArrayList, kCollection, VHasNext, HasNext)
    kAddPrivateIVMethod(kArrayList, kCollection, VNext, Next)

kEndClassEx() 

kFx(kStatus) kArrayList_Construct(kArrayList* list, kType itemType, kSize initialCapacity, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback(allocator);
    kStatus status; 

    kCheck(kAlloc_GetObject(alloc, kTypeOf(kArrayList), list)); 

    if (!kSuccess(status = xkArrayList_Init(*list, kTypeOf(kArrayList), itemType, initialCapacity, alloc)))
    {
        kAlloc_FreeRef(alloc, list); 
    }

    return status; 
} 

kFx(kStatus) xkArrayList_ConstructFramework(kArrayList* list, kAlloc allocator)
{
    return kArrayList_Construct(list, kNULL, 0, allocator);
}

kFx(kStatus) xkArrayList_Init(kArrayList list, kType classType, kType itemType, kSize initialCapacity, kAlloc alloc)
{
    kObjR(kArrayList, list);
    kStatus status = kOK; 

    kCheck(kObject_Init(list, classType, alloc)); 

    obj->itemType = kIsNull(itemType) ? kTypeOf(kVoid) : itemType; 
    obj->itemSize = kType_Size(obj->itemType); 
    obj->allocSize = 0; 
    obj->items = kNULL; 
    obj->count = 0; 
    obj->capacity = 0; 
    obj->isAttached = kFALSE; 

    kTry
    {
        kTest(kArrayList_Reserve(list, initialCapacity)); 
    }
    kCatch(&status)
    {
        xkArrayList_VRelease(list); 
        kEndCatch(status); 
    }

    return kOK; 
}

kFx(kStatus) xkArrayList_VClone(kArrayList list, kArrayList source, kAlloc valueAlloc, kObject context)
{
    kObj(kArrayList, list);
    
    obj->itemType = kArrayList_ItemType(source); 
    obj->itemSize = kArrayList_ItemSize(source); 

    kCheck(kArrayList_Resize(list, kArrayList_Count(source))); 
    
    kCheck(kCloneItems(obj->itemType, kObject_Alloc(list), obj->items, kObject_Alloc(source), kArrayList_Data(source), obj->count, context, kObject_Alloc(list), valueAlloc));

    return kOK; 
}

kFx(kStatus) xkArrayList_VRelease(kArrayList list)
{
    kObj(kArrayList, list); 

    if (!obj->isAttached)
    {
        kCheck(kAlloc_Free(kObject_Alloc(list), obj->items)); 
    }

    kCheck(kObject_VRelease(list)); 

    return kOK; 
}

kFx(kBool) xkArrayList_VHasShared(kArrayList list)
{
    kObj(kArrayList, list);

    return kObject_IsShared(list) || kHasSharedItems(obj->itemType, obj->items, obj->count); 
}

kFx(kSize) xkArrayList_VSize(kArrayList list)
{
    kObj(kArrayList, list); 
    kSize dataSize = (!obj->isAttached) ? obj->allocSize : kArrayList_DataSize(list); 
    kSize size = sizeof(kArrayListClass) + dataSize; 

    size += kMeasureItems(obj->itemType, obj->items, obj->count); 

    return size; 
}

kFx(kAllocTrait) xkArrayList_VAllocTraits(kArrayList list)
{
    kObj(kArrayList, list);

    return kAlloc_Traits(kObject_Alloc(list)) | kEnumerateAllocTraits(obj->itemType, obj->items, obj->count);
}

kFx(kStatus) xkArrayList_WriteDat5V1(kArrayList list, kSerializer serializer)
{
    kObj(kArrayList, list); 
    kTypeVersion itemVersion; 

    kCheck(kSerializer_WriteSize(serializer, obj->capacity)); 
    kCheck(kSerializer_Write32s(serializer, kTRUE));  //isDynamic

    kCheck(kSerializer_WriteSize(serializer, obj->count)); 
    kCheck(kSerializer_WriteType(serializer, obj->itemType, &itemVersion));
    kCheck(kSerializer_WriteItems(serializer, obj->itemType, itemVersion, obj->items, obj->count)); 

    return kOK; 
}

kFx(kStatus) xkArrayList_ReadDat5V1(kArrayList list, kSerializer serializer)
{
    kObj(kArrayList, list);
    kSize capacity = 0; 
    kSize count = 0; 
    kTypeVersion itemVersion;
    k32s isDynamic; 
    kType itemType = kNULL;            

    kCheck(kSerializer_ReadSize(serializer, &capacity));      
    kCheck(kSerializer_Read32s(serializer, &isDynamic));

    kCheck(kSerializer_ReadSize(serializer, &count));
    kCheck(kSerializer_ReadType(serializer, &itemType, &itemVersion)); 

    kCheck(kArrayList_Allocate(list, itemType, count)); 
    kCheck(kArrayList_Resize(list, count)); 

    kCheck(kSerializer_ReadItems(serializer, itemType, itemVersion, obj->items, count)); 

    return kOK; 
}

kFx(kStatus) xkArrayList_WriteDat6V0(kArrayList list, kSerializer serializer)
{
    kObj(kArrayList, list); 
    kTypeVersion itemVersion; 

    kCheck(kSerializer_WriteType(serializer, obj->itemType, &itemVersion));
    kCheck(kSerializer_WriteSize(serializer, obj->count)); 
    kCheck(kSerializer_WriteItems(serializer, obj->itemType, itemVersion, obj->items, obj->count)); 

    return kOK; 
}

kFx(kStatus) xkArrayList_ReadDat6V0(kArrayList list, kSerializer serializer)
{
    kObj(kArrayList, list);
    kType itemType = kNULL;            
    kTypeVersion itemVersion; 
    kSize count = 0; 
 
    kCheck(kSerializer_ReadType(serializer, &itemType, &itemVersion));
    kCheck(kSerializer_ReadSize(serializer, &count)); 

    kCheck(kArrayList_Allocate(list, itemType, count)); 
    kCheck(kArrayList_Resize(list, count)); 

    kCheck(kSerializer_ReadItems(serializer, itemType, itemVersion, obj->items, count)); 

    return kOK; 
}

kFx(kStatus) kArrayList_Allocate(kArrayList list, kType itemType, kSize initialCapacity)
{
    kObj(kArrayList, list); 

    if (obj->isAttached)
    {
        obj->items = kNULL; 
        obj->isAttached = kFALSE; 
    }
    
    obj->itemType = kIsNull(itemType) ? kTypeOf(kVoid) : itemType; 
    obj->itemSize = kType_Size(obj->itemType); 
    obj->count = 0; 
    obj->capacity = 0; 

    kCheck(kArrayList_Reserve(list, initialCapacity)); 

    return kOK; 
}

kFx(kStatus) kArrayList_Attach(kArrayList list, void* items, kType itemType, kSize capacity)
{
    kObj(kArrayList, list); 

    if (!obj->isAttached)
    {
        kCheck(kAlloc_FreeRef(kObject_Alloc(list), &obj->items));
    }
    
    obj->itemType = kIsNull(itemType) ? kTypeOf(kVoid) : itemType; 
    obj->itemSize = kType_Size(obj->itemType); 
    obj->allocSize = 0; 
    obj->items = items; 
    obj->count = capacity; 
    obj->capacity = capacity; 
    obj->isAttached = kTRUE; 

    return kOK; 
}

kFx(kStatus) kArrayList_Import(kArrayList list, const void* items, kType itemType, kSize count)
{
    kObj(kArrayList, list); 

    kCheck(kArrayList_Allocate(list, itemType, count)); 
    
    kCheck(kMemCopy(obj->items, items, count*obj->itemSize));     
    obj->count = count; 

    return kOK; 
}

kFx(kStatus) kArrayList_Append(kArrayList list, const void* items, kSize count)
{
    kObj(kArrayList, list); 

    kCheck(kArrayList_Reserve(list, obj->count + count)); 

    kCheck(kMemCopy(kArrayList_DataAt(list, (kSSize)obj->count), items, count*obj->itemSize)); 
    obj->count += count; 

    return kOK; 
}

kFx(kStatus) kArrayList_Assign(kArrayList list, kArrayList source)
{
    kObj(kArrayList, list); 
    kObjN(kArrayList, sourceObj, source);
    
    kCheck(kArrayList_Allocate(list, sourceObj->itemType, sourceObj->count)); 

    kCheck(kArrayList_Resize(list, sourceObj->count)); 

    kCheck(kCopyItems(obj->itemType, obj->items, sourceObj->items, obj->count)); 

    return kOK;   
}

kFx(kStatus) kArrayList_Purge(kArrayList list)
{    
    kObj(kArrayList, list); 

    kCheck(kDisposeItems(obj->itemType, obj->items, obj->count)); 

    kCheck(kArrayList_Clear(list)); 

    return kOK; 
}

kFx(kStatus) kArrayList_Zero(kArrayList list)
{
    kObj(kArrayList, list); 

    kCheck(kZeroItems(obj->itemType, obj->items, obj->count));

    return kOK; 
}

kFx(kStatus) kArrayList_Reserve(kArrayList list, kSize minimumCapacity)
{
    kObj(kArrayList, list); 

    if (obj->capacity < minimumCapacity)
    {
        kSize grownCapacity = kMax_(minimumCapacity, xkARRAY_LIST_GROWTH_FACTOR*obj->capacity);
        kSize newCapacity = kMax_(grownCapacity, xkARRAY_LIST_MIN_CAPACITY); 
        kSize newSize = kMax_(obj->allocSize, newCapacity*obj->itemSize); 

        if (newSize > obj->allocSize)
        {
            void* oldItems = obj->items; 
            void* newItems = kNULL; 

            kCheck(kAlloc_Get(kObject_Alloc(list), newSize, &newItems));

            if (obj->count > 0)
            {
                kMemCopy(newItems, oldItems, obj->count*obj->itemSize); 
            }

            obj->items = newItems; 
            obj->allocSize = newSize; 

            kCheck(kAlloc_Free(kObject_Alloc(list), oldItems));
        }

        obj->capacity = newCapacity;
    }
    
    return kOK; 
}

kFx(kStatus) kArrayList_Resize(kArrayList list, kSize count)
{
    kObj(kArrayList, list); 

    if (count > obj->capacity)
    {
        kCheck(kArrayList_Reserve(list, count)); 
    }

    if (kType_IsReference(obj->itemType) && (count > obj->count))
    {
        kMemSet(kArrayList_DataAt(list, (kSSize)obj->count), 0, obj->itemSize*(count - obj->count)); 
    }

    obj->count = count; 
    
    return kOK; 
}

kFx(kStatus) kArrayList_Add(kArrayList list, const void* item)
{
    kObj(kArrayList, list); 

    if (obj->count == obj->capacity)
    {
        kCheck(kArrayList_Reserve(list, obj->capacity + 1)); 
    }

    kValue_Import(obj->itemType, kArrayList_DataAt(list, (kSSize)obj->count), item); 

    obj->count++; 

    return kOK; 
}

kFx(kStatus) kArrayList_SetItem(kArrayList list, kSize index, const void* item)
{
    kObj(kArrayList, list); 

    kCheckArgs(index < obj->count); 

    kValue_Import(obj->itemType, kArrayList_DataAt(list, (kSSize)index), item); 

    return kOK; 
}

kFx(kStatus) kArrayList_Item(kArrayList list, kSize index, void* item)
{
    kObj(kArrayList, list); 

    kCheckArgs(index < obj->count); 

    kItemCopy(item, kArrayList_DataAt(list, (kSSize)index), obj->itemSize); 

    return kOK; 
}

kFx(kStatus) kArrayList_Insert(kArrayList list, kSize before, const void* item)
{
    kObj(kArrayList, list); 

    kCheckArgs(before <= obj->count); 

    if (obj->count == obj->capacity)
    {
        kCheck(kArrayList_Reserve(list, obj->capacity + 1)); 
    }

    if (before != obj->count)
    {
        kMemMove(kArrayList_DataAt(list, (kSSize)before + 1), kArrayList_DataAt(list, (kSSize)before), obj->itemSize*(obj->count - before)); 
    }

    kValue_Import(obj->itemType, kArrayList_DataAt(list, (kSSize)before), item); 

    obj->count++; 

    return kOK; 
}

kFx(kStatus) kArrayList_Remove(kArrayList list, kSize index, void* item)
{
    kObj(kArrayList, list); 

    kCheckArgs(index < obj->count); 

    if (item)
    {
        kItemCopy(item, kArrayList_DataAt(list, (kSSize)index), obj->itemSize); 
    }

    if (index != (obj->count-1))
    {
        kMemMove(kArrayList_DataAt(list, (kSSize)index), kArrayList_DataAt(list, (kSSize)index + 1), obj->itemSize*(obj->count - index - 1)); 
    }

    obj->count--; 

    return kOK; 
}

kFx(kIterator) xkArrayList_GetIterator(kArrayList list)
{
    kObj(kArrayList, list); 

    return obj->items; 
}

kFx(kBool) xkArrayList_HasNext(kArrayList list, kIterator iterator)
{
    kObj(kArrayList, list); 
    void* end = (kByte*)obj->items + obj->count*obj->itemSize;

    return (iterator != end); 
}

kFx(void*) xkArrayList_Next(kArrayList list, kIterator* iterator)
{
    kObj(kArrayList, list); 
    void* next = *iterator; 
   
    *iterator = (kByte*)*iterator + obj->itemSize; 

    return next; 
}








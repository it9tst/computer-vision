/** 
 * @file    kArray1.cpp
 *
 * @internal
 * Copyright (C) 2005-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <kApi/Data/kArray1.h>
#include <kApi/Data/kArrayProvider.h>
#include <kApi/Data/kCollection.h>
#include <kApi/Io/kSerializer.h>

kBeginClassEx(k, kArray1) 
    
    //serialization versions
    kAddPrivateVersionEx(kArray1, "kdat5", "5.0.0.0", "24-1", WriteDat5V1, ReadDat5V1)
    kAddPrivateVersionEx(kArray1, "kdat6", "5.7.1.0", "kArray1-0", WriteDat6V0, ReadDat6V0)

    //special constructors
    kAddPrivateFrameworkConstructor(kArray1, ConstructFramework)

    //virtual methods
    kAddPrivateVMethod(kArray1, kObject, VRelease)
    kAddPrivateVMethod(kArray1, kObject, VDisposeItems)
    kAddPrivateVMethod(kArray1, kObject, VClone)
    kAddPrivateVMethod(kArray1, kObject, VHasShared)
    kAddPrivateVMethod(kArray1, kObject, VSize)
    kAddPrivateVMethod(kArray1, kObject, VAllocTraits)

    //collection interface 
    kAddInterface(kArray1, kCollection)
    kAddPrivateIVMethod(kArray1, kCollection, VGetIterator, GetIterator)
    kAddIVMethod(kArray1, kCollection, VItemType, ItemType)
    kAddIVMethod(kArray1, kCollection, VCount, Count)
    kAddPrivateIVMethod(kArray1, kCollection, VHasNext, HasNext)
    kAddPrivateIVMethod(kArray1, kCollection, VNext, Next)

    //array provider interface 
    kAddInterface(kArray1, kArrayProvider)
    kAddPrivateIVMethod(kArray1, kArrayProvider, VConstructDefault, ConstructDefaultEx)
    kAddPrivateIVMethod(kArray1, kArrayProvider, VImitate, Imitate)
    kAddPrivateIVMethod(kArray1, kArrayProvider, VAssign, Assign)
    kAddIVMethod(kArray1, kArrayProvider, VItemType, ItemType)
    kAddIVMethod(kArray1, kArrayProvider, VCount, Count)
    kAddIVMethod(kArray1, kArrayProvider, VData, Data)
    kAddIVMethod(kArray1, kArrayProvider, VDataSize, DataSize)
    kAddIVMethod(kArray1, kArrayProvider, VDataAlloc, DataAlloc)

kEndClassEx() 

kFx(kStatus) kArray1_ConstructEx(kArray1* array, kType itemType, kSize length, kAlloc allocator, kAlloc valueAllocator, kMemoryAlignment valueAlignment)
{
    kAlloc alloc = kAlloc_Fallback(allocator);
    kAlloc valueAlloc = kAlloc_Fallback(valueAllocator);
    kStatus status; 

    kCheck(kAlloc_GetObject(alloc, kTypeOf(kArray1), array)); 

    if (!kSuccess(status = xkArray1_Init(*array, kTypeOf(kArray1), itemType, length, alloc, valueAlloc, valueAlignment)))
    {
        kAlloc_FreeRef(alloc, array); 
    }

    return status; 
}

kFx(kStatus) kArray1_Construct(kArray1* array, kType itemType, kSize length, kAlloc allocator)
{
    return kArray1_ConstructEx(array, itemType, length, allocator, allocator);
}

kFx(kStatus) xkArray1_ConstructFramework(kArray1* array, kAlloc allocator)
{
    return kArray1_ConstructEx(array, kNULL, 0, allocator, allocator);
}

kFx(kStatus) xkArray1_ConstructDefaultEx(kArray1* array, kAlloc objectAlloc, kAlloc valueAlloc)
{
    return kArray1_ConstructEx(array, kNULL, 0, objectAlloc, valueAlloc);
}

kFx(kStatus) xkArray1_Init(kArray1 array, kType classType, kType itemType, kSize length, kAlloc alloc, kAlloc valueAlloc, kMemoryAlignment alignment)
{
    kObjR(kArray1, array);
    kType resolvedItemType = kIsNull(itemType) ? kTypeOf(kVoid) : itemType; 
    kStatus status = kOK; 

    kCheck(kObject_Init(array, classType, alloc)); 

    obj->valueAlloc = valueAlloc; 
    obj->itemType = resolvedItemType;
    obj->itemSize = kType_Size(obj->itemType); 
    obj->allocSize = 0; 
    obj->items = kNULL;
    obj->length = 0; 
    obj->isAttached = kFALSE;
    obj->alignment = alignment;

    kTry
    {
        kTest(xkArray1_Realloc(array, length)); 
    }
    kCatch(&status)
    {
        xkArray1_VRelease(array); 
        kEndCatch(status); 
    }

    return kOK; 
}

kFx(kStatus) xkArray1_VClone(kArray1 array, kArray1 source, kAlloc valueAlloc, kObject context)
{
    kObj(kArray1, array);

    obj->valueAlloc = valueAlloc;

    kCheck(xkArray1_Imitate(array, source));     

    kCheck(kCloneItems(obj->itemType, kArray1_DataAlloc(array), obj->items, kArray1_DataAlloc(source), kArray1_Data(source), obj->length, context, kObject_Alloc(array), valueAlloc)); 

    return kOK; 
}

kFx(kStatus) xkArray1_VRelease(kArray1 array)
{
    kObj(kArray1, array);

    if (!obj->isAttached)
    {
        kCheck(kAlloc_Free(kArray1_DataAlloc(array), obj->items)); 
    }

    kCheck(kObject_VRelease(array)); 

    return kOK; 
}

kFx(kStatus) xkArray1_VDisposeItems(kArray1 array)
{
    kObj(kArray1, array);

    kCheck(kDisposeItems(obj->itemType, obj->items, obj->length)); 

    return kOK; 
}

kFx(kBool) xkArray1_VHasShared(kArray1 array)
{
    kObj(kArray1, array);

    return kObject_IsShared(array) || kHasSharedItems(obj->itemType, obj->items, obj->length); 
}

kFx(kSize) xkArray1_VSize(kArray1 array)
{
    kObj(kArray1, array);
    kSize dataSize = (!obj->isAttached) ? obj->allocSize : kArray1_DataSize(array); 
    kSize size = sizeof(kArray1Class) + dataSize; 

    size += kMeasureItems(obj->itemType, obj->items, obj->length); 

    return size; 
}

kFx(kAllocTrait) xkArray1_VAllocTraits(kArray1 array)
{
    kObj(kArray1, array);

    return kAlloc_Traits(kObject_Alloc(array)) | kAlloc_Traits(kArray1_DataAlloc(array)) | kEnumerateAllocTraits(obj->itemType, obj->items, obj->length);
}

kFx(kStatus) xkArray1_WriteDat5V1(kArray1 array, kSerializer serializer)
{
    kObj(kArray1, array);
    kTypeVersion itemVersion; 

    kCheck(kSerializer_WriteSize(serializer, obj->length)); 
    kCheck(kSerializer_WriteType(serializer, obj->itemType, &itemVersion));
    kCheck(kSerializer_WriteItems(serializer, obj->itemType, itemVersion, obj->items, obj->length)); 

    return kOK; 
}

kFx(kStatus) xkArray1_ReadDat5V1(kArray1 array, kSerializer serializer)
{
    kObj(kArray1, array);
    kSize length = 0; 
    kTypeVersion itemVersion;
    kType itemType = kNULL;            

    kCheck(kSerializer_ReadSize(serializer, &length));
    kCheck(kSerializer_ReadType(serializer, &itemType, &itemVersion)); 

    kCheck(kArray1_Allocate(array, itemType, length)); 
  
    kCheck(kSerializer_ReadItems(serializer, itemType, itemVersion, obj->items, length));

    return kOK; 
}

kFx(kStatus) xkArray1_WriteDat6V0(kArray1 array, kSerializer serializer)
{
    kObj(kArray1, array);
    kTypeVersion itemVersion; 

    kCheck(kSerializer_WriteType(serializer, obj->itemType, &itemVersion));
    kCheck(kSerializer_WriteSize(serializer, obj->length)); 
    kCheck(kSerializer_WriteItems(serializer, obj->itemType, itemVersion, obj->items, obj->length)); 

    return kOK; 
}

kFx(kStatus) xkArray1_ReadDat6V0(kArray1 array, kSerializer serializer)
{
    kObj(kArray1, array);
    kType itemType = kNULL;            
    kTypeVersion itemVersion; 
    kSize length = 0; 

    kCheck(kSerializer_ReadType(serializer, &itemType, &itemVersion));
    kCheck(kSerializer_ReadSize(serializer, &length)); 

    kCheck(kArray1_Allocate(array, itemType, length)); 

    kCheck(kSerializer_ReadItems(serializer, itemType, itemVersion, obj->items, length)); 

    return kOK; 
}

kFx(kStatus) xkArray1_Realloc(kArray1 array, kSize length)
{
    kObj(kArray1, array);
    kSize newSize = kMax_(obj->allocSize, length*obj->itemSize); 

    if (newSize > obj->allocSize)
    {
        void* oldItems = obj->items; 
        void* newItems = kNULL; 

        kCheck(kAlloc_Get(kArray1_DataAlloc(array), newSize, &newItems, obj->alignment));
        
        obj->items = newItems; 
        obj->allocSize = newSize; 

        kCheck(kAlloc_Free(kArray1_DataAlloc(array), oldItems));
    }
        
    if (kType_IsReference(obj->itemType))
    {
        kMemSet(obj->items, 0, obj->itemSize*length);  
    }

    obj->length = length; 
    
    return kOK; 
}

kFx(kStatus) kArray1_Allocate(kArray1 array, kType itemType, kSize length)
{
    kObj(kArray1, array);

    if (obj->isAttached)
    {
        obj->items = kNULL; 
        obj->isAttached = kFALSE; 
    }

    //if the value allocator is specialized, and we are changing from value type to ref type (or vice versa), reallocate
    if ((kObject_Alloc(array) != obj->valueAlloc) && (kType_IsValue(itemType) != kType_IsValue(obj->itemType)))
    {
        kCheck(kAlloc_FreeRef(kArray1_DataAlloc(array), &obj->items)); 
        obj->allocSize = 0;
    }
    
    obj->itemType = kIsNull(itemType) ? kTypeOf(kVoid) : itemType; 
    obj->itemSize = kType_Size(obj->itemType); 
    obj->length = 0; 

    kCheck(xkArray1_Realloc(array, length)); 

    return kOK; 
}

kFx(kStatus) kArray1_Attach(kArray1 array, void* items, kType itemType, kSize length)
{
    kObj(kArray1, array);

    if (!obj->isAttached)
    {
        kCheck(kAlloc_FreeRef(kArray1_DataAlloc(array), &obj->items));
    }
    
    obj->itemType = kIsNull(itemType) ? kTypeOf(kVoid) : itemType; 
    obj->itemSize = kType_Size(obj->itemType); 
    obj->allocSize = 0; 
    obj->items = items; 
    obj->length = length; 
    obj->isAttached = kTRUE; 

    return kOK; 
}

kFx(kStatus) xkArray1_Imitate(kArray1 array, kArray1 source)
{
    return kArray1_Allocate(array, kArray1_ItemType(source), kArray1_Length(source));
}

kFx(kStatus) xkArray1_Assign(kArray1 array, kArray1 source, kObject context)
{
    kObj(kArray1, array);
    kObjN(kArray1, sourceObj, source);

    kCheck(xkArray1_Imitate(array, source));
  
    kCheck(kCopyItems(obj->itemType, kArray1_DataAlloc(array), obj->items, kArray1_DataAlloc(source), sourceObj->items, obj->length, context)); 

    return kOK;   
}

kFx(kStatus) kArray1_Zero(kArray1 array)
{
    kObj(kArray1, array);

    kCheckArgs(!kAlloc_IsForeign(kArray1_DataAlloc(array)));

    kCheck(kZeroItems(obj->itemType, obj->items, obj->length));

    return kOK; 
}

kFx(kStatus) kArray1_SetItem(kArray1 array, kSize index, const void* item)
{
    kObj(kArray1, array);

    kAssert(!kAlloc_IsForeign(kArray1_DataAlloc(array)));
    kCheckArgs(index < obj->length);    

    kValue_Import(obj->itemType, kArray1_DataAt(array, (kSSize)index), item); 

    return kOK; 
}

kFx(kStatus) kArray1_Item(kArray1 array, kSize index, void* item)
{
    kObj(kArray1, array);

    kAssert(!kAlloc_IsForeign(kArray1_DataAlloc(array)));
    kCheckArgs(index < obj->length); 

    kItemCopy(item, kArray1_DataAt(array, (kSSize)index), obj->itemSize); 

    return kOK; 
}

kFx(kIterator) xkArray1_GetIterator(kArray1 array)
{
    kObj(kArray1, array);
    return obj->items; 
}

kFx(kBool) xkArray1_HasNext(kArray1 array, kIterator iterator)
{
    kObj(kArray1, array);
    void* end = (kByte*)obj->items + obj->length*obj->itemSize;

    return (iterator != end); 
}

kFx(void*) xkArray1_Next(kArray1 array, kIterator* iterator)
{
    kObj(kArray1, array);
    void* next = *iterator; 
   
    *iterator = (kByte*)*iterator + obj->itemSize; 

    return next; 
}

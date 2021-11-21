/** 
 * @file    kArray2.cpp
 *
 * @internal
 * Copyright (C) 2005-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <kApi/Data/kArray2.h>
#include <kApi/Data/kArrayProvider.h>
#include <kApi/Data/kCollection.h>
#include <kApi/Io/kSerializer.h>

kBeginClassEx(k, kArray2) 
    
    //serialization versions
    kAddPrivateVersionEx(kArray2, "kdat5", "5.0.0.0", "25-1", WriteDat5V1, ReadDat5V1)
    kAddPrivateVersionEx(kArray2, "kdat6", "5.7.1.0", "kArray2-0", WriteDat6V0, ReadDat6V0)

    //special constructors
    kAddPrivateFrameworkConstructor(kArray2, ConstructFramework)

    //virtual methods
    kAddPrivateVMethod(kArray2, kObject, VRelease)
    kAddPrivateVMethod(kArray2, kObject, VDisposeItems)
    kAddPrivateVMethod(kArray2, kObject, VClone)
    kAddPrivateVMethod(kArray2, kObject, VHasShared)
    kAddPrivateVMethod(kArray2, kObject, VSize)
    kAddPrivateVMethod(kArray2, kObject, VAllocTraits)

    //collection interface 
    kAddInterface(kArray2, kCollection)
    kAddPrivateIVMethod(kArray2, kCollection, VGetIterator, GetIterator)
    kAddIVMethod(kArray2, kCollection, VItemType, ItemType)
    kAddIVMethod(kArray2, kCollection, VCount, Count)
    kAddPrivateIVMethod(kArray2, kCollection, VHasNext, HasNext)
    kAddPrivateIVMethod(kArray2, kCollection, VNext, Next)

    //array provider interface 
    kAddInterface(kArray2, kArrayProvider)
    kAddPrivateIVMethod(kArray2, kArrayProvider, VConstructDefault, ConstructDefaultEx)
    kAddPrivateIVMethod(kArray2, kArrayProvider, VImitate, Imitate)
    kAddPrivateIVMethod(kArray2, kArrayProvider, VAssign, Assign)
    kAddIVMethod(kArray2, kArrayProvider, VItemType, ItemType)
    kAddIVMethod(kArray2, kArrayProvider, VCount, Count)
    kAddIVMethod(kArray2, kArrayProvider, VData, Data)
    kAddIVMethod(kArray2, kArrayProvider, VDataSize, DataSize)
    kAddIVMethod(kArray2, kArrayProvider, VDataAlloc, DataAlloc)

kEndClassEx() 

kFx(kStatus) kArray2_ConstructEx(kArray2* array, kType itemType, kSize length0, kSize length1, kAlloc allocator, kAlloc valueAllocator, kMemoryAlignment valueAlignment)
{
    kAlloc alloc = kAlloc_Fallback(allocator);
    kAlloc valueAlloc = kAlloc_Fallback(valueAllocator);
    kStatus status; 

    kCheck(kAlloc_GetObject(alloc, kTypeOf(kArray2), array)); 

    if (!kSuccess(status = xkArray2_Init(*array, kTypeOf(kArray2), itemType, length0, length1, alloc, valueAlloc, valueAlignment)))
    {
        kAlloc_FreeRef(alloc, array); 
    }

    return status; 
}

kFx(kStatus) kArray2_Construct(kArray2* array, kType itemType, kSize length0, kSize length1, kAlloc allocator)
{
    return kArray2_ConstructEx(array, itemType, length0, length1, allocator, allocator);
} 

kFx(kStatus) xkArray2_ConstructFramework(kArray2* array, kAlloc allocator)
{
    return kArray2_ConstructEx(array, kNULL, 0, 0, allocator, allocator);
}

kFx(kStatus) xkArray2_ConstructDefaultEx(kArray2* array, kAlloc objectAlloc, kAlloc valueAlloc)
{
    return kArray2_ConstructEx(array, kNULL, 0, 0, objectAlloc, valueAlloc);
}

kFx(kStatus) xkArray2_Init(kArray2 array, kType classType, kType itemType, kSize length0, kSize length1, kAlloc alloc, kAlloc valueAlloc, kMemoryAlignment alignment)
{
    kObjR(kArray2, array);
    kType resolvedItemType = kIsNull(itemType) ? kTypeOf(kVoid) : itemType; 
    kStatus status = kOK; 

    kCheck(kObject_Init(array, classType, alloc)); 

    obj->valueAlloc = valueAlloc;
    obj->itemType = resolvedItemType;
    obj->itemSize = kType_Size(obj->itemType); 
    obj->allocSize = 0; 
    obj->items = kNULL;
    obj->length[0] = 0; 
    obj->length[1] = 0; 
    obj->isAttached = kFALSE; 
    obj->alignment = alignment;

    kTry
    {
        kTest(xkArray2_Realloc(array, length0, length1)); 
    }
    kCatch(&status)
    {
        xkArray2_VRelease(array); 
        kEndCatch(status); 
    }

    return kOK; 
}

kFx(kStatus) xkArray2_VClone(kArray2 array, kArray2 source, kAlloc valueAlloc, kObject context)
{
    kObj(kArray2, array);

    obj->valueAlloc = valueAlloc; 

    kCheck(xkArray2_Imitate(array, source));     

    kCheck(kCloneItems(obj->itemType, kArray2_DataAlloc(array), obj->items, kArray2_DataAlloc(source), kArray2_Data(source), kArray2_Count(source), context, kObject_Alloc(array), valueAlloc));  

    return kOK; 
}

kFx(kStatus) xkArray2_VRelease(kArray2 array)
{
    kObj(kArray2, array);

    if (!obj->isAttached)
    {
        kCheck(kAlloc_Free(kArray2_DataAlloc(array), obj->items)); 
    }

    kCheck(kObject_VRelease(array)); 

    return kOK; 
}

kFx(kStatus) xkArray2_VDisposeItems(kArray2 array)
{
    kObj(kArray2, array);

    kCheck(kDisposeItems(obj->itemType, obj->items, kArray2_Count(array))); 

    return kOK; 
}

kFx(kBool) xkArray2_VHasShared(kArray2 array)
{
    kObj(kArray2, array);

    return kObject_IsShared(array) || kHasSharedItems(obj->itemType, obj->items, kArray2_Count(array)); 
}

kFx(kSize) xkArray2_VSize(kArray2 array)
{
    kObj(kArray2, array);
    kSize dataSize = (!obj->isAttached) ? obj->allocSize : kArray2_DataSize(array); 
    kSize size = sizeof(kArray2Class) + dataSize; 

    size += kMeasureItems(obj->itemType, obj->items, kArray2_Count(array)); 

    return size; 
}

kFx(kAllocTrait) xkArray2_VAllocTraits(kArray2 array)
{
    kObj(kArray2, array);

    return kAlloc_Traits(kObject_Alloc(array)) | kAlloc_Traits(kArray2_DataAlloc(array)) | kEnumerateAllocTraits(obj->itemType, obj->items, kArray2_Count(array));
}

kFx(kStatus) xkArray2_WriteDat5V1(kArray2 array, kSerializer serializer)
{
    kObj(kArray2, array);
    kTypeVersion itemVersion; 
    kSize count = kArray2_Count(array); 

    kCheckState(!kAlloc_IsForeign(kArray2_DataAlloc(array)));

    kCheck(kSerializer_WriteSize(serializer, obj->length[0])); 
    kCheck(kSerializer_WriteSize(serializer, obj->length[1])); 
    kCheck(kSerializer_WriteSize(serializer, count)); 
    kCheck(kSerializer_WriteType(serializer, obj->itemType, &itemVersion));
    kCheck(kSerializer_WriteItems(serializer, obj->itemType, itemVersion, obj->items, count)); 

    return kOK; 
}

kFx(kStatus) xkArray2_ReadDat5V1(kArray2 array, kSerializer serializer)
{
    kObj(kArray2, array);
    kSize length0 = 0, length1 = 0, count = 0; 
    kTypeVersion itemVersion;
    kType itemType = kNULL;            
  
    kCheck(kSerializer_ReadSize(serializer, &length0)); 
    kCheck(kSerializer_ReadSize(serializer, &length1)); 

    kCheck(kSerializer_ReadSize(serializer, &count));
    kCheck(kSerializer_ReadType(serializer, &itemType, &itemVersion)); 

    kCheck(kArray2_Allocate(array, itemType, length0, length1)); 
  
    kCheck(kSerializer_ReadItems(serializer, itemType, itemVersion, obj->items, count));

    return kOK; 
}

kFx(kStatus) xkArray2_WriteDat6V0(kArray2 array, kSerializer serializer)
{
    kObj(kArray2, array);
    kTypeVersion itemVersion; 

    kCheckState(!kAlloc_IsForeign(kArray2_DataAlloc(array)));

    kCheck(kSerializer_WriteType(serializer, obj->itemType, &itemVersion));
    kCheck(kSerializer_WriteSize(serializer, obj->length[0])); 
    kCheck(kSerializer_WriteSize(serializer, obj->length[1])); 
    kCheck(kSerializer_WriteItems(serializer, obj->itemType, itemVersion, obj->items, kArray2_Count(array))); 

    return kOK; 
}

kFx(kStatus) xkArray2_ReadDat6V0(kArray2 array, kSerializer serializer)
{
    kObj(kArray2, array);
    kType itemType = kNULL;            
    kTypeVersion itemVersion; 
    kSize length0 = 0, length1 = 0; 

    kCheck(kSerializer_ReadType(serializer, &itemType, &itemVersion));
    kCheck(kSerializer_ReadSize(serializer, &length0)); 
    kCheck(kSerializer_ReadSize(serializer, &length1)); 

    kCheck(kArray2_Allocate(array, itemType, length0, length1)); 

    kCheck(kSerializer_ReadItems(serializer, itemType, itemVersion, obj->items, kArray2_Count(array)));

    return kOK; 
}

kFx(kStatus) xkArray2_Realloc(kArray2 array, kSize length0, kSize length1)
{
    kObj(kArray2, array);
    kSize newSize = kMax_(obj->allocSize, length0*length1*obj->itemSize); 

    if (newSize > obj->allocSize)
    {
        void* oldItems = obj->items; 
        void* newItems = kNULL; 

        kCheck(kAlloc_Get(kArray2_DataAlloc(array), newSize, &newItems, obj->alignment));
        
        obj->items = newItems; 
        obj->allocSize = newSize; 

        kCheck(kAlloc_Free(kArray2_DataAlloc(array), oldItems));
    }
        
    if (kType_IsReference(obj->itemType))
    {
        kMemSet(obj->items, 0, length0*length1*obj->itemSize);  
    }

    obj->length[0] = length0; 
    obj->length[1] = length1; 
    
    return kOK; 
}

kFx(kStatus) kArray2_Allocate(kArray2 array, kType itemType, kSize length0, kSize length1)
{
    kObj(kArray2, array);

    if (obj->isAttached)
    {
        obj->items = kNULL; 
        obj->isAttached = kFALSE; 
    }
    
    //if the value allocator is specialized, and we are changing from value type to ref type (or vice versa), reallocate
    if ((kObject_Alloc(array) != obj->valueAlloc) && (kType_IsValue(itemType) != kType_IsValue(obj->itemType)))
    {
        kCheck(kAlloc_FreeRef(kArray2_DataAlloc(array), &obj->items)); 
        obj->allocSize = 0;
    }

    obj->itemType = kIsNull(itemType) ? kTypeOf(kVoid) : itemType; 
    obj->itemSize = kType_Size(obj->itemType); 
    obj->length[0] = 0; 
    obj->length[1] = 0; 

    kCheck(xkArray2_Realloc(array, length0, length1)); 

    return kOK; 
}

kFx(kStatus) kArray2_Attach(kArray2 array, void* items, kType itemType, kSize length0, kSize length1)
{
    kObj(kArray2, array);

    if (!obj->isAttached)
    {
        kCheck(kAlloc_FreeRef(kArray2_DataAlloc(array), &obj->items));
    }
    
    obj->itemType = kIsNull(itemType) ? kTypeOf(kVoid) : itemType; 
    obj->itemSize = kType_Size(obj->itemType); 
    obj->allocSize = 0; 
    obj->items = items; 
    obj->length[0] = length0; 
    obj->length[1] = length1; 
    obj->isAttached = kTRUE; 

    return kOK; 
}

kFx(kStatus) xkArray2_Imitate(kArray2 array, kArray2 source)
{
    return kArray2_Allocate(array, kArray2_ItemType(source), kArray2_Length(source, 0), kArray2_Length(source, 1));
}

kFx(kStatus) xkArray2_Assign(kArray2 array, kArray2 source, kObject context)
{
    kObj(kArray2, array);
    kObjN(kArray2, sourceObj, source);

    kCheck(xkArray2_Imitate(array, source));

    kCheck(kCopyItems(obj->itemType, kArray2_DataAlloc(array), obj->items, kArray2_DataAlloc(source), sourceObj->items, kArray2_Count(array), context));

    return kOK;   
}

kFx(kStatus) kArray2_Zero(kArray2 array)
{
    kObj(kArray2, array);

    kCheckArgs(!kAlloc_IsForeign(kArray2_DataAlloc(array)));

    kCheck(kZeroItems(obj->itemType, obj->items, kArray2_Count(array)));

    return kOK; 
}

kFx(kStatus) kArray2_SetItem(kArray2 array, kSize index0, kSize index1, const void* item)
{
    kObj(kArray2, array);

    kAssert(!kAlloc_IsForeign(kArray2_DataAlloc(array)));
    kCheckArgs((index0 < obj->length[0]) && (index1 < obj->length[1])); 

    kValue_Import(obj->itemType, kArray2_DataAt(array, (kSSize)index0, (kSSize)index1), item); 

    return kOK; 
}

kFx(kStatus) kArray2_Item(kArray2 array, kSize index0, kSize index1, void* item)
{
    kObj(kArray2, array);

    kAssert(!kAlloc_IsForeign(kArray2_DataAlloc(array)));
    kCheckArgs((index0 < obj->length[0]) && (index1 < obj->length[1])); 

    kItemCopy(item, kArray2_DataAt(array, (kSSize)index0, (kSSize)index1), obj->itemSize); 

    return kOK; 
}

kFx(kIterator) xkArray2_GetIterator(kArray2 array)
{
    kObj(kArray2, array);
    return obj->items; 
}

kFx(kBool) xkArray2_HasNext(kArray2 array, kIterator iterator)
{
    kObj(kArray2, array);
    void* end = (kByte*)obj->items + kArray2_DataSize(array); 

    return (iterator != end); 
}

kFx(void*) xkArray2_Next(kArray2 array, kIterator* iterator)
{
    kObj(kArray2, array);
    void* next = *iterator; 
   
    *iterator = (kByte*)*iterator + obj->itemSize; 

    return next; 
}


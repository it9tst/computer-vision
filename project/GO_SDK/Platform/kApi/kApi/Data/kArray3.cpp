/** 
 * @file    kArray3.cpp
 *
 * @internal
 * Copyright (C) 2006-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <kApi/Data/kArray3.h>
#include <kApi/Data/kArrayProvider.h>
#include <kApi/Data/kCollection.h>
#include <kApi/Io/kSerializer.h>

kBeginClassEx(k, kArray3) 
    
    //serialization versions
    kAddPrivateVersionEx(kArray3, "kdat5", "5.0.0.0", "26-0", WriteDat5V0, ReadDat5V0)
    kAddPrivateVersionEx(kArray3, "kdat6", "5.7.1.0", "kArray3-0", WriteDat6V0, ReadDat6V0)

    //special constructors
    kAddPrivateFrameworkConstructor(kArray3, ConstructFramework)

    //virtual methods
    kAddPrivateVMethod(kArray3, kObject, VRelease)
    kAddPrivateVMethod(kArray3, kObject, VDisposeItems)
    kAddPrivateVMethod(kArray3, kObject, VClone)
    kAddPrivateVMethod(kArray3, kObject, VHasShared)
    kAddPrivateVMethod(kArray3, kObject, VSize)
    kAddPrivateVMethod(kArray3, kObject, VAllocTraits)

    //collection interface 
    kAddInterface(kArray3, kCollection)
    kAddPrivateIVMethod(kArray3, kCollection, VGetIterator, GetIterator)
    kAddIVMethod(kArray3, kCollection, VItemType, ItemType)
    kAddIVMethod(kArray3, kCollection, VCount, Count)
    kAddPrivateIVMethod(kArray3, kCollection, VHasNext, HasNext)
    kAddPrivateIVMethod(kArray3, kCollection, VNext, Next)

    //array provider interface 
    kAddInterface(kArray3, kArrayProvider)
    kAddPrivateIVMethod(kArray3, kArrayProvider, VConstructDefault, ConstructDefaultEx)
    kAddPrivateIVMethod(kArray3, kArrayProvider, VImitate, Imitate)
    kAddPrivateIVMethod(kArray3, kArrayProvider, VAssign, Assign)
    kAddIVMethod(kArray3, kArrayProvider, VItemType, ItemType)
    kAddIVMethod(kArray3, kArrayProvider, VCount, Count)
    kAddIVMethod(kArray3, kArrayProvider, VData, Data)
    kAddIVMethod(kArray3, kArrayProvider, VDataSize, DataSize)
    kAddIVMethod(kArray3, kArrayProvider, VDataAlloc, DataAlloc)

kEndClassEx() 

kFx(kStatus) kArray3_ConstructEx(kArray3* array, kType itemType, kSize length0, kSize length1, kSize length2, kAlloc allocator, kAlloc valueAllocator, kMemoryAlignment valueAlignment)
{
    kAlloc alloc = kAlloc_Fallback(allocator);
    kAlloc valueAlloc = kAlloc_Fallback(valueAllocator);
    kStatus status; 

    kCheck(kAlloc_GetObject(alloc, kTypeOf(kArray3), array)); 

    if (!kSuccess(status = xkArray3_Init(*array, kTypeOf(kArray3), itemType, length0, length1, length2, alloc, valueAlloc, valueAlignment)))
    {
        kAlloc_FreeRef(alloc, array); 
    }

    return status; 
} 

kFx(kStatus) kArray3_Construct(kArray3* array, kType itemType, kSize length0, kSize length1, kSize length2, kAlloc allocator)
{
    return kArray3_ConstructEx(array, itemType, length0, length1, length2, allocator, allocator);
} 

kFx(kStatus) xkArray3_ConstructFramework(kArray3* array, kAlloc allocator)
{
    return kArray3_ConstructEx(array, kNULL, 0, 0, 0, allocator, allocator);
}

kFx(kStatus) xkArray3_ConstructDefaultEx(kArray3* array, kAlloc objectAlloc, kAlloc valueAlloc)
{
    return kArray3_ConstructEx(array, kNULL, 0, 0, 0, objectAlloc, valueAlloc);
}

kFx(kStatus) xkArray3_Init(kArray3 array, kType classType, kType itemType, kSize length0, kSize length1, kSize length2, kAlloc alloc, kAlloc valueAlloc, kMemoryAlignment alignment)
{
    kObjR(kArray3, array);
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
    obj->length[2] = 0; 
    obj->isAttached = kFALSE; 
    obj->alignment = alignment;

    kTry
    {
        kTest(xkArray3_Realloc(array, length0, length1, length2)); 
    }
    kCatch(&status)
    {
        xkArray3_VRelease(array); 
        kEndCatch(status); 
    }

    return kOK; 
}

kFx(kStatus) xkArray3_VClone(kArray3 array, kArray3 source, kAlloc valueAlloc, kObject context)
{
    kObj(kArray3, array);

    obj->valueAlloc = valueAlloc; 

    kCheck(xkArray3_Imitate(array, source));     

    kCheck(kCloneItems(obj->itemType, kArray3_DataAlloc(array), obj->items, kArray3_DataAlloc(source), kArray3_Data(source), kArray3_Count(source), context, kObject_Alloc(array), valueAlloc));  

    return kOK; 
}

kFx(kStatus) xkArray3_VRelease(kArray3 array)
{
    kObj(kArray3, array);

    if (!obj->isAttached)
    {
        kCheck(kAlloc_Free(kArray3_DataAlloc(array), obj->items)); 
    }

    kCheck(kObject_VRelease(array)); 

    return kOK; 
}

kFx(kStatus) xkArray3_VDisposeItems(kArray3 array)
{
    kObj(kArray3, array);

    kCheck(kDisposeItems(obj->itemType, obj->items, kArray3_Count(array))); 

    return kOK; 
}

kFx(kBool) xkArray3_VHasShared(kArray3 array)
{
    kObj(kArray3, array);

    return kObject_IsShared(array) || kHasSharedItems(obj->itemType, obj->items, kArray3_Count(array)); 
}

kFx(kSize) xkArray3_VSize(kArray3 array)
{
    kObj(kArray3, array);
    kSize dataSize = (!obj->isAttached) ? obj->allocSize : kArray3_DataSize(array); 
    kSize size = sizeof(kArray3Class) + dataSize; 

    size += kMeasureItems(obj->itemType, obj->items, kArray3_Count(array)); 

    return size; 
}

kFx(kAllocTrait) xkArray3_VAllocTraits(kArray3 array)
{
    kObj(kArray3, array);

    return kAlloc_Traits(kObject_Alloc(array)) | kAlloc_Traits(kArray3_DataAlloc(array)) | kEnumerateAllocTraits(obj->itemType, obj->items, kArray3_Count(array));
}

kFx(kStatus) xkArray3_WriteDat5V0(kArray3 array, kSerializer serializer)
{
    kObj(kArray3, array);
    kTypeVersion itemVersion; 
    kSize count = kArray3_Count(array); 

    kCheckState(!kAlloc_IsForeign(kArray3_DataAlloc(array)));

    kCheck(kSerializer_WriteSize(serializer, obj->length[0])); 
    kCheck(kSerializer_WriteSize(serializer, obj->length[1])); 
    kCheck(kSerializer_WriteSize(serializer, obj->length[2])); 
    kCheck(kSerializer_WriteSize(serializer, count)); 
    kCheck(kSerializer_WriteType(serializer, obj->itemType, &itemVersion));
    kCheck(kSerializer_WriteItems(serializer, obj->itemType, itemVersion, obj->items, count)); 

    return kOK; 
}

kFx(kStatus) xkArray3_ReadDat5V0(kArray3 array, kSerializer serializer)
{
    kObj(kArray3, array);
    kSize length0 = 0, length1 = 0, length2 = 0; 
    k32u count = 0; 
    kTypeVersion itemVersion;
    kType itemType = kNULL;            

    kCheck(kSerializer_ReadSize(serializer, &length0)); 
    kCheck(kSerializer_ReadSize(serializer, &length1)); 
    kCheck(kSerializer_ReadSize(serializer, &length2)); 

    kCheck(kSerializer_Read32u(serializer, &count));
    kCheck(kSerializer_ReadType(serializer, &itemType, &itemVersion)); 

    kCheck(kArray3_Allocate(array, itemType, length0, length1, length2)); 

    kCheck(kSerializer_ReadItems(serializer, itemType, itemVersion, obj->items, count));

    return kOK; 
}

kFx(kStatus) xkArray3_WriteDat6V0(kArray3 array, kSerializer serializer)
{
    kObj(kArray3, array);
    kTypeVersion itemVersion; 

    kCheckState(!kAlloc_IsForeign(kArray3_DataAlloc(array)));

    kCheck(kSerializer_WriteType(serializer, obj->itemType, &itemVersion));
    kCheck(kSerializer_WriteSize(serializer, obj->length[0])); 
    kCheck(kSerializer_WriteSize(serializer, obj->length[1])); 
    kCheck(kSerializer_WriteSize(serializer, obj->length[2])); 
    kCheck(kSerializer_WriteItems(serializer, obj->itemType, itemVersion, obj->items, kArray3_Count(array))); 

    return kOK; 
}

kFx(kStatus) xkArray3_ReadDat6V0(kArray3 array, kSerializer serializer)
{
    kObj(kArray3, array);
    kType itemType = kNULL;            
    kTypeVersion itemVersion; 
    kSize length0 = 0, length1 = 0, length2 = 0; 

    kCheck(kSerializer_ReadType(serializer, &itemType, &itemVersion));
    kCheck(kSerializer_ReadSize(serializer, &length0)); 
    kCheck(kSerializer_ReadSize(serializer, &length1)); 
    kCheck(kSerializer_ReadSize(serializer, &length2)); 

    kCheck(kArray3_Allocate(array, itemType, length0, length1, length2)); 

    kCheck(kSerializer_ReadItems(serializer, itemType, itemVersion, obj->items, kArray3_Count(array))); 
  
    return kOK; 
}

kFx(kStatus) xkArray3_Realloc(kArray3 array, kSize length0, kSize length1, kSize length2)
{
    kObj(kArray3, array);
    kSize newSize = kMax_(obj->allocSize, length0*length1*length2*obj->itemSize); 

    if (newSize > obj->allocSize)
    {
        void* oldItems = obj->items; 
        void* newItems = kNULL; 

        kCheck(kAlloc_Get(kArray3_DataAlloc(array), newSize, &newItems, obj->alignment));
        
        obj->items = newItems; 
        obj->allocSize = newSize; 

        kCheck(kAlloc_Free(kArray3_DataAlloc(array), oldItems));
    }
        
    if (kType_IsReference(obj->itemType))
    {
        kMemSet(obj->items, 0, length0*length1*length2*obj->itemSize);  
    }

    obj->length[0] = length0; 
    obj->length[1] = length1; 
    obj->length[2] = length2; 
    
    return kOK; 
}

kFx(kStatus) kArray3_Allocate(kArray3 array, kType itemType, kSize length0, kSize length1, kSize length2)
{
    kObj(kArray3, array);

    if (obj->isAttached)
    {
        obj->items = kNULL; 
        obj->isAttached = kFALSE; 
    }

    //if the value allocator is specialized, and we are changing from value type to ref type (or vice versa), reallocate
    if ((kObject_Alloc(array) != obj->valueAlloc) && (kType_IsValue(itemType) != kType_IsValue(obj->itemType)))
    {
        kCheck(kAlloc_FreeRef(kArray3_DataAlloc(array), &obj->items)); 
        obj->allocSize = 0;
    }

    obj->itemType = kIsNull(itemType) ? kTypeOf(kVoid) : itemType; 
    obj->itemSize = kType_Size(obj->itemType); 
    obj->length[0] = 0; 
    obj->length[1] = 0; 
    obj->length[2] = 0; 

    kCheck(xkArray3_Realloc(array, length0, length1, length2)); 

    return kOK; 
}

kFx(kStatus) kArray3_Attach(kArray3 array, void* items, kType itemType, kSize length0, kSize length1, kSize length2)
{
    kObj(kArray3, array);

    if (!obj->isAttached)
    {
        kCheck(kAlloc_FreeRef(kArray3_DataAlloc(array), &obj->items));
    }
    
    obj->itemType = kIsNull(itemType) ? kTypeOf(kVoid) : itemType; 
    obj->itemSize = kType_Size(obj->itemType); 
    obj->allocSize = 0; 
    obj->items = items; 
    obj->length[0] = length0; 
    obj->length[1] = length1; 
    obj->length[2] = length2; 
    obj->isAttached = kTRUE; 

    return kOK; 
}

kFx(kStatus) xkArray3_Imitate(kArray3 array, kArray3 source)
{
    return kArray3_Allocate(array, kArray3_ItemType(source), kArray3_Length(source, 0), kArray3_Length(source, 1), kArray3_Length(source, 2));
}

kFx(kStatus) xkArray3_Assign(kArray3 array, kArray3 source, kObject context)
{
    kObj(kArray3, array);
    kObjN(kArray3, sourceObj, source);

    kCheck(xkArray3_Imitate(array, source));

    kCheck(kCopyItems(obj->itemType, kArray3_DataAlloc(array), obj->items, kArray3_DataAlloc(source), sourceObj->items, kArray3_Count(array), context));

    return kOK;   
}

kFx(kStatus) kArray3_Zero(kArray3 array)
{
    kObj(kArray3, array);

    kCheckArgs(!kAlloc_IsForeign(kArray3_DataAlloc(array)));

    kCheck(kZeroItems(obj->itemType, obj->items, kArray3_Count(array)));

    return kOK; 
}

kFx(kStatus) kArray3_SetItem(kArray3 array, kSize index0, kSize index1, kSize index2, const void* item)
{
    kObj(kArray3, array);

    kAssert(!kAlloc_IsForeign(kArray3_DataAlloc(array)));
    kCheckArgs((index0 < obj->length[0]) && (index1 < obj->length[1]) && (index2 < obj->length[2])); 

    kValue_Import(obj->itemType, kArray3_DataAt(array, (kSSize)index0, (kSSize)index1, (kSSize)index2), item); 

    return kOK; 
}

kFx(kStatus) kArray3_Item(kArray3 array, kSize index0, kSize index1, kSize index2, void* item)
{
    kObj(kArray3, array);

    kAssert(!kAlloc_IsForeign(kArray3_DataAlloc(array)));
    kCheckArgs((index0 < obj->length[0]) && (index1 < obj->length[1]) && (index2 < obj->length[2])); 

    kItemCopy(item, kArray3_DataAt(array, (kSSize)index0, (kSSize)index1, (kSSize)index2), obj->itemSize); 

    return kOK; 
}

kFx(kIterator) xkArray3_GetIterator(kArray3 array)
{
    kObj(kArray3, array);

    return obj->items; 
}

kFx(kBool) xkArray3_HasNext(kArray3 array, kIterator iterator)
{
    kObj(kArray3, array);
    void* end = (kByte*)obj->items + kArray3_DataSize(array); 

    return (iterator != end); 
}

kFx(void*) xkArray3_Next(kArray3 array, kIterator* iterator)
{
    kObj(kArray3, array);
    void* next = *iterator; 
   
    *iterator = (kByte*)*iterator + obj->itemSize; 

    return next; 
}

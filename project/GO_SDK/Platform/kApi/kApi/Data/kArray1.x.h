/** 
 * @file    kArray1.x.h
 *
 * @internal
 * Copyright (C) 2005-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_ARRAY_1_X_H
#define K_API_ARRAY_1_X_H

typedef struct kArray1Class
{
    kObjectClass base; 
    kAlloc valueAlloc;          //allocator to be used for value allocations
    kType itemType;             //item type
    kSize itemSize;             //item size, in bytes
    kSize allocSize;            //size of allocated array memory, in bytes
    void* items;                //array memory 
    kSize length;               //array length, in items
    kBool isAttached;           //is array memory externally owned?
    kMemoryAlignment alignment; //alignment for the value array
} kArray1Class;
    
kDeclareClassEx(k, kArray1, kObject) 

/* 
* Forward declarations. 
*/

kFx(kStatus) kArray1_Attach(kArray1 array, void* items, kType itemType, kSize length); 
kFx(kStatus) kArray1_SetItem(kArray1 array, kSize index, const void* item); 
kFx(kStatus) kArray1_Item(kArray1 array, kSize index, void* item); 
kInlineFx(kSize) kArray1_Length(kArray1 array); 
kInlineFx(kType) kArray1_ItemType(kArray1 array);
kInlineFx(void*) kArray1_Data(kArray1 array); 
kInlineFx(void*) kArray1_DataAt(kArray1 array, kSSize index);
kInlineFx(void*) kArray1_At(kArray1 array, kSize index); 
kInlineFx(kSize) kArray1_ItemSize(kArray1 array);

/* 
* Private methods. 
*/
kFx(kStatus) xkArray1_ConstructFramework(kArray1* array, kAlloc objectAlloc);

kFx(kStatus) xkArray1_Init(kArray1 array, kType classType, kType itemType, kSize length, kAlloc alloc, kAlloc valueAlloc, kMemoryAlignment alignment);
kFx(kStatus) xkArray1_VClone(kArray1 array, kArray1 source, kAlloc valueAlloc, kObject context); 

kFx(kStatus) xkArray1_VRelease(kArray1 array); 
kFx(kStatus) xkArray1_VDisposeItems(kArray1 array); 

kFx(kBool) xkArray1_VHasShared(kArray1 array); 
kFx(kSize) xkArray1_VSize(kArray1 array); 
kFx(kAllocTrait) xkArray1_VAllocTraits(kArray1 array);

kFx(kStatus) xkArray1_WriteDat5V1(kArray1 array, kSerializer serializer); 
kFx(kStatus) xkArray1_ReadDat5V1(kArray1 array, kSerializer serializer); 
kFx(kStatus) xkArray1_WriteDat6V0(kArray1 array, kSerializer serializer); 
kFx(kStatus) xkArray1_ReadDat6V0(kArray1 array, kSerializer serializer); 

kFx(kStatus) xkArray1_Assign(kArray1 array, kArray1 source, kObject context);
kFx(kStatus) xkArray1_Realloc(kArray1 array, kSize length); 

//kCollection 
kFx(kIterator) xkArray1_GetIterator(kArray1 array); 
kFx(kBool) xkArray1_HasNext(kArray1 array, kIterator iterator); 
kFx(void*) xkArray1_Next(kArray1 array, kIterator* iterator); 

//kArrayProvider
kFx(kStatus) xkArray1_ConstructDefaultEx(kArray1* array, kAlloc objectAlloc, kAlloc valueAlloc);
kFx(kStatus) xkArray1_Imitate(kArray1 array, kArray1 source);

kInlineFx(kStatus) xkArray1_AttachT(kArray1 array, void* items, kType itemType, kSize length, kSize itemSize)
{
    kAssert(xkType_IsPointerCompatible(itemType, itemSize)); 

    return kArray1_Attach(array, items, itemType, length);
} 

kInlineFx(kStatus) xkArray1_SetItemT(kArray1 array, kSize index, const void* item, kSize itemSize)
{
    kAssert(xkType_IsPointerCompatible(kArray1_ItemType(array), itemSize)); 

    return kArray1_SetItem(array, index, item); 
} 

kInlineFx(kStatus) xkArray1_ItemT(kArray1 array, kSize index, void* item, kSize itemSize)
{
    kAssert(xkType_IsPointerCompatible(kArray1_ItemType(array), itemSize)); 

    return kArray1_Item(array, index, item); 
} 

kInlineFx(void*) xkArray1_DataT(kArray1 array, kSize itemSize)
{
    kAssert(xkType_IsPointerCompatible(kArray1_ItemType(array), itemSize)); 

    return kArray1_Data(array);
} 

kInlineFx(void*) xkArray1_DataAtT(kArray1 array, kSSize index, kSize itemSize)
{
    kAssert(xkType_IsPointerCompatible(kArray1_ItemType(array), itemSize)); 

    return kArray1_DataAt(array, index);
} 

kInlineFx(void*) xkArray1_AtT(kArray1 array, kSize index, kSize itemSize)
{
    kAssert(xkType_IsPointerCompatible(kArray1_ItemType(array), itemSize)); 

    return kArray1_At(array, index);
} 

kInlineFx(void*) xkArray1_AsT(kArray1 array, kSize index, kSize itemSize)
{
    kAssert(itemSize == kArray1_ItemSize(array)); 

    return kArray1_At(array, index);
}

#endif

/** 
 * @file    kArray3.x.h
 *
 * @internal
 * Copyright (C) 2006-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_ARRAY_3_X_H
#define K_API_ARRAY_3_X_H

typedef struct kArray3Class
{
    kObjectClass base; 
    kAlloc valueAlloc;          //allocator to be used for value allocations
    kType itemType;             //item type
    kSize itemSize;             //item size, in bytes
    kSize allocSize;            //size of allocated array memory, in bytes
    void* items;                //array memory 
    kSize length[3];            //array length per dimension, in items
    kBool isAttached;           //is array memory externally owned?
    kMemoryAlignment alignment; //memory alignment for data.
} kArray3Class;
    
kDeclareClassEx(k, kArray3, kObject) 

/* 
* Forward declarations. 
*/
kFx(kStatus) kArray3_Attach(kArray3 array, void* items, kType itemType, kSize length0, kSize length1, kSize length2); 
kFx(kStatus) kArray3_SetItem(kArray3 array, kSize index0, kSize index1, kSize index2, const void* item); 
kFx(kStatus) kArray3_Item(kArray3 array, kSize index0, kSize index1, kSize index2, void* item); 
kInlineFx(kSize) kArray3_Count(kArray3 array);
kInlineFx(kSize) kArray3_Length(kArray3 array, kSize dimension);
kInlineFx(kType) kArray3_ItemType(kArray3 array); 
kInlineFx(kSize) kArray3_ItemSize(kArray3 array);
kInlineFx(void*) kArray3_Data(kArray3 array);
kInlineFx(void*) kArray3_DataAt(kArray3 array, kSSize index0, kSSize index1, kSSize index2);
kInlineFx(void*) kArray3_At(kArray3 array, kSize index0, kSize index1, kSize index2);

/* 
* Private methods. 
*/

kFx(kStatus) xkArray3_ConstructFramework(kArray3* array, kAlloc objectAlloc);

kFx(kStatus) xkArray3_Init(kArray3 array, kType classType, kType itemType, kSize length0, kSize length1, kSize length2, kAlloc alloc, kAlloc valueAlloc, kMemoryAlignment alignment);
kFx(kStatus) xkArray3_VClone(kArray3 array, kArray3 source, kAlloc valueAlloc, kObject context); 

kFx(kStatus) xkArray3_VRelease(kArray3 array); 
kFx(kStatus) xkArray3_VDisposeItems(kArray3 array); 
kFx(kBool) xkArray3_VHasShared(kArray3 array); 
kFx(kSize) xkArray3_VSize(kArray3 array); 
kFx(kAllocTrait) xkArray3_VAllocTraits(kArray3 array);

kFx(kStatus) xkArray3_WriteDat5V0(kArray3 array, kSerializer serializer); 
kFx(kStatus) xkArray3_ReadDat5V0(kArray3 array, kSerializer serializer); 
kFx(kStatus) xkArray3_WriteDat6V0(kArray3 array, kSerializer serializer); 
kFx(kStatus) xkArray3_ReadDat6V0(kArray3 array, kSerializer serializer); 

kFx(kStatus) xkArray3_Assign(kArray3 array, kArray3 source, kObject context);
kFx(kStatus) xkArray3_Realloc(kArray3 array, kSize length0, kSize length1, kSize length2);

//kCollection
kFx(kIterator) xkArray3_GetIterator(kArray3 array); 
kFx(kBool) xkArray3_HasNext(kArray3 array, kIterator iterator); 
kFx(void*) xkArray3_Next(kArray3 array, kIterator* iterator); 

//kArrayProvider
kFx(kStatus) xkArray3_ConstructDefaultEx(kArray3* array, kAlloc objectAlloc, kAlloc valueAlloc);
kFx(kStatus) xkArray3_Imitate(kArray3 array, kArray3 source);

kInlineFx(kStatus) xkArray3_AttachT(kArray3 array, void* items, kType itemType, kSize length0, kSize length1, kSize length2, kSize itemSize)
{
    kAssert(xkType_IsPointerCompatible(itemType, itemSize)); 

    return kArray3_Attach(array, items, itemType, length0, length1, length2);
} 

kInlineFx(kStatus) xkArray3_SetItemT(kArray3 array, kSize index0, kSize index1, kSize index2, const void* item, kSize itemSize)
{
    kAssert(xkType_IsPointerCompatible(kArray3_ItemType(array), itemSize)); 

    return kArray3_SetItem(array, index0, index1, index2, item);
} 

kInlineFx(kStatus) xkArray3_ItemT(kArray3 array, kSize index0, kSize index1, kSize index2, void* item, kSize itemSize)
{
    kAssert(xkType_IsPointerCompatible(kArray3_ItemType(array), itemSize)); 

    return kArray3_Item(array, index0, index1, index2, item);
} 

kInlineFx(void*) xkArray3_DataT(kArray3 array, kSize itemSize)
{
    kAssert(xkType_IsPointerCompatible(kArray3_ItemType(array), itemSize)); 

    return kArray3_Data(array);
} 

kInlineFx(void*) xkArray3_DataAtT(kArray3 array, kSSize index0, kSSize index1, kSize index2, kSize itemSize)
{
    kAssert(xkType_IsPointerCompatible(kArray3_ItemType(array), itemSize)); 

    return kArray3_DataAt(array, (kSSize)index0, (kSSize)index1, (kSSize)index2);
} 

kInlineFx(void*) xkArray3_AtT(kArray3 array, kSize index0, kSize index1, kSize index2, kSize itemSize)
{
    kAssert(xkType_IsPointerCompatible(kArray3_ItemType(array), itemSize)); 

    return kArray3_At(array, index0, index1, index2);
} 

kInlineFx(void*) xkArray3_AsT(kArray3 array, kSize index0, kSize index1, kSize index2, kSize itemSize)
{
    kAssert(itemSize == kArray3_ItemSize(array)); 

    return kArray3_At(array, index0, index1, index2);
}


#endif

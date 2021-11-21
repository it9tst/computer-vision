/** 
 * @file    kArrayList.x.h
 *
 * @internal
 * Copyright (C) 2005-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_ARRAY_LIST_X_H
#define K_API_ARRAY_LIST_X_H

#define xkARRAY_LIST_MIN_CAPACITY                (16)        ///< Minimum number of allocated elements.
#define xkARRAY_LIST_GROWTH_FACTOR               (2)         ///< Factor by which capacity is increased upon exhaustion.

typedef struct kArrayListClass
{
    kObjectClass base; 
    kType itemType;             //item type
    kSize itemSize;             //item size, in bytes
    kSize allocSize;            //size of allocated array memory, in bytes
    void* items;                //item array
    kSize count;                //current number of elements
    kSize capacity;             //maximum elements before reallocation
    kBool isAttached;           //is array memory externally owned?
} kArrayListClass;

kDeclareClassEx(k, kArrayList, kObject) 

/* 
* Forward declarations. 
*/

kFx(kStatus) kArrayList_Attach(kArrayList list, void* items, kType itemType, kSize capacity); 
kFx(kStatus) kArrayList_Import(kArrayList list, const void* items, kType itemType, kSize count); 
kFx(kStatus) kArrayList_Append(kArrayList list, const void* items, kSize count); 
kFx(kStatus) kArrayList_Reserve(kArrayList list, kSize capacity); 
kFx(kStatus) kArrayList_Remove(kArrayList list, kSize index, void* item);
kFx(kStatus) kArrayList_Add(kArrayList list, const void* item);
kFx(kStatus) kArrayList_Insert(kArrayList list, kSize before, const void* item); 
kFx(kStatus) kArrayList_Item(kArrayList list, kSize index, void* item);
kFx(kStatus) kArrayList_SetItem(kArrayList list, kSize index, const void* item);
kInlineFx(kSize) kArrayList_Count(kArrayList list);
kInlineFx(void*) kArrayList_Begin(kArrayList list);
kInlineFx(void*) kArrayList_RBegin(kArrayList list);
kFx(kStatus) kArrayList_Purge(kArrayList list); 
kInlineFx(kType) kArrayList_ItemType(kArrayList list); 
kInlineFx(kSize) kArrayList_ItemSize(kArrayList list);
kInlineFx(void*) kArrayList_Data(kArrayList list);
kInlineFx(void*) kArrayList_DataAt(kArrayList list, kSSize index);
kInlineFx(void*) kArrayList_At(kArrayList list, kSize index);
kInlineFx(void*) kArrayList_First(kArrayList list); 
kInlineFx(void*) kArrayList_Last(kArrayList list);
kInlineFx(void*) kArrayList_Begin(kArrayList list);
kInlineFx(void*) kArrayList_End(kArrayList list);
kInlineFx(void*) kArrayList_RBegin(kArrayList list);
kInlineFx(void*) kArrayList_REnd(kArrayList list);

/* 
* Private methods. 
*/

kFx(kStatus) xkArrayList_ConstructFramework(kArrayList* list, kAlloc allocator); 

kFx(kStatus) xkArrayList_Init(kArrayList list, kType classType, kType itemType, kSize initialCapacity, kAlloc alloc); 
kFx(kStatus) xkArrayList_VClone(kArrayList list, kArrayList source, kAlloc valueAlloc, kObject context); 

kFx(kStatus) xkArrayList_VRelease(kArrayList list); 

kInlineFx(kStatus) xkArrayList_VDisposeItems(kArrayList list)
{
    return kArrayList_Purge(list);
}

kFx(kBool) xkArrayList_VHasShared(kArrayList list); 
kFx(kSize) xkArrayList_VSize(kArrayList list); 
kFx(kAllocTrait) xkArrayList_VAllocTraits(kArrayList list);

kFx(kStatus) xkArrayList_WriteDat5V1(kArrayList list, kSerializer serializer); 
kFx(kStatus) xkArrayList_ReadDat5V1(kArrayList list, kSerializer serializer); 
kFx(kStatus) xkArrayList_WriteDat6V0(kArrayList list, kSerializer serializer); 
kFx(kStatus) xkArrayList_ReadDat6V0(kArrayList list, kSerializer serializer); 

kFx(kIterator) xkArrayList_GetIterator(kArrayList list); 
kFx(kBool) xkArrayList_HasNext(kArrayList list, kIterator iterator); 
kFx(void*) xkArrayList_Next(kArrayList list, kIterator* iterator); 

kInlineFx(kStatus) xkArrayList_AttachT(kArrayList list, void* items, kType itemType, kSize capacity, kSize itemSize)
{
    kAssert(xkType_IsPointerCompatible(itemType, itemSize)); 

    return kArrayList_Attach(list, items, itemType, capacity); 
} 

kInlineFx(kStatus) xkArrayList_ImportT(kArrayList list, const void* items, kType itemType, kSize count, kSize itemSize)
{
    kAssert(xkType_IsPointerCompatible(itemType, itemSize)); 

    return kArrayList_Import(list, items, itemType, count); 
} 

kInlineFx(kStatus) xkArrayList_AppendT(kArrayList list, const void* items, kSize count, kSize itemSize)
{
    kAssert(xkType_IsPointerCompatible(kArrayList_ItemType(list), itemSize)); 

    return kArrayList_Append(list, items, count); 
} 

kInlineFx(kStatus) xkArrayList_AddT(kArrayList list, const void* item, kSize itemSize)
{
    kAssert(xkType_IsPointerCompatible(kArrayList_ItemType(list), itemSize)); 

    return kArrayList_Add(list, item); 
} 

kInlineFx(kStatus) xkArrayList_InsertT(kArrayList list, kSize before, const void* item, kSize itemSize)
{
    kAssert(xkType_IsPointerCompatible(kArrayList_ItemType(list), itemSize)); 

    return kArrayList_Insert(list, before, item); 
} 

kInlineFx(kStatus) xkArrayList_RemoveT(kArrayList list, kSize index, void* item, kSize itemSize)
{
    kAssert(xkType_IsPointerCompatible(kArrayList_ItemType(list), itemSize)); 

    return kArrayList_Remove(list, index, item); 
} 

kInlineFx(kStatus) xkArrayList_ItemT(kArrayList list, kSize index, void* item, kSize itemSize)
{
    kAssert(xkType_IsPointerCompatible(kArrayList_ItemType(list), itemSize)); 

    return kArrayList_Item(list, index, item); 
} 

kInlineFx(kStatus) xkArrayList_SetItemT(kArrayList list, kSize index, const void* item, kSize itemSize)
{
    kAssert(xkType_IsPointerCompatible(kArrayList_ItemType(list), itemSize)); 

    return kArrayList_SetItem(list, index, item); 
} 

kInlineFx(void*) xkArrayList_DataT(kArrayList list, kSize itemSize)
{
    kAssert(xkType_IsPointerCompatible(kArrayList_ItemType(list), itemSize)); 

    return kArrayList_Data(list); 
} 

kInlineFx(void*) xkArrayList_DataAtT(kArrayList list, kSSize index, kSize itemSize)
{
    kAssert(xkType_IsPointerCompatible(kArrayList_ItemType(list), itemSize)); 

    return kArrayList_DataAt(list, index); 
} 

kInlineFx(void*) xkArrayList_AtT(kArrayList list, kSize index, kSize itemSize)
{
    kAssert(xkType_IsPointerCompatible(kArrayList_ItemType(list), itemSize)); 

    return kArrayList_At(list, index); 
} 

kInlineFx(void*) xkArrayList_FirstT(kArrayList list, kSize itemSize)
{
    kAssert(xkType_IsPointerCompatible(kArrayList_ItemType(list), itemSize)); 

    return kArrayList_First(list); 
} 

kInlineFx(void*) xkArrayList_LastT(kArrayList list, kSize itemSize)
{
    kAssert(xkType_IsPointerCompatible(kArrayList_ItemType(list), itemSize)); 

    return kArrayList_Last(list); 
} 

kInlineFx(void*) xkArrayList_BeginT(kArrayList list, kSize itemSize)
{
    kAssert(xkType_IsPointerCompatible(kArrayList_ItemType(list), itemSize)); 

    return kArrayList_Begin(list); 
} 

kInlineFx(void*) xkArrayList_EndT(kArrayList list, kSize itemSize)
{
    kAssert(xkType_IsPointerCompatible(kArrayList_ItemType(list), itemSize)); 

    return kArrayList_End(list); 
} 

kInlineFx(void*) xkArrayList_RBeginT(kArrayList list, kSize itemSize)
{
    kAssert(xkType_IsPointerCompatible(kArrayList_ItemType(list), itemSize)); 

    return kArrayList_RBegin(list); 
} 

kInlineFx(void*) xkArrayList_REndT(kArrayList list, kSize itemSize)
{
    kAssert(xkType_IsPointerCompatible(kArrayList_ItemType(list), itemSize)); 

    return kArrayList_REnd(list); 
} 

kInlineFx(void*) xkArrayList_AsT(kArrayList list, kSize index, kSize itemSize)
{
    kAssert(itemSize == kArrayList_ItemSize(list)); 

    return kArrayList_At(list, index);
}

#endif

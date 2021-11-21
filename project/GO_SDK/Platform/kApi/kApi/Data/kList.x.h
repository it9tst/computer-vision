/** 
 * @file    kList.x.h
 *
 * @internal
 * Copyright (C) 2013-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_LIST_X_H
#define K_API_LIST_X_H

#define xkLIST_MIN_ITEMS_PER_BLOCK            (16)            //minimum number of list items per allocated memory block

//represents a single list entry
typedef struct kListItemStruct //<T>
{
    struct kListItemStruct* next;               //linked list pointer to next item
    struct kListItemStruct* previous;           //linked list pointer to previous item (ignored/unused on free list)
    //T content;                                //content; size dynamically calculated
} kListItemStruct;

//represents a block of allocated list items; multiple items
//are allocated at the same time for efficiency
typedef struct kListItemBlock //<T, N>
{
    struct kListItemBlock* next;             //linked list pointer to next memory block 
    kSize itemCount;                         //count of items in block (N)
    kListItemStruct* items;                  //pointer to first item in block
    //kListItemStruct<T> item[N];            //array of list items
} kListItemBlock; 

typedef struct kListClass
{
    kObjectClass base;                          // base fields
    kListItemBlock* blocks;                     // allocated memory blocks, where block contains N items
    kListItemStruct* free;                      // singly-linked list of free items
    kListItemStruct* first;                     // pointer to first list item
    kListItemStruct* last;                      // pointer to last list item
    kSize count;                                // current count of items
    kSize capacity;                             // maximum number of items before allocation required
    xkStructField contentField;                  // content field info
    kSize itemSize;                             // size of list item, including content fields
} kListClass;

kDeclareClassEx(k, kList, kObject) 

/* 
* Forward declarations. 
*/

kFx(kStatus) kList_Add(kList list, const void* itemContent, kListItem* item); 
kFx(kStatus) kList_Insert(kList list, kListItem before, const void* itemContent, kListItem* item); 
kFx(kStatus) kList_SetItem(kList list, kListItem item, const void* content); 
kFx(kStatus) kList_Item(kList list, kListItem item, void* content); 
kFx(kStatus) kList_Purge(kList list); 

kInlineFx(kListItem) kList_First(kList list);
kInlineFx(kListItem) kList_Next(kList list, kListItem item);
kInlineFx(kType) kList_ItemType(kList list); 
kInlineFx(void*) kList_At(kList list, kListItem item); 
kFx(void*) kList_AtIndex(kList list, kSize index); 

/* 
* Private methods. 
*/

kFx(kStatus) xkList_ConstructFramework(kList* list, kAlloc allocator); 

kFx(kStatus) xkList_Init(kList list, kType classType, kType itemType, kSize initialCapacity, kAlloc alloc); 

kFx(kStatus) xkList_VClone(kList list, kList source, kAlloc valueAlloc, kObject context); 

kFx(kStatus) xkList_VRelease(kList list); 

kInlineFx(kStatus) xkList_VDisposeItems(kList list)
{
    return kList_Purge(list);
}

kFx(kBool) xkList_VHasShared(kList list); 
kFx(kSize) xkList_VSize(kList list); 
kFx(kAllocTrait) xkList_VAllocTraits(kList list); 

kFx(kStatus) xkList_WriteDat6V0(kList list, kSerializer serializer); 
kFx(kStatus) xkList_ReadDat6V0(kList list, kSerializer serializer); 

kFx(kStatus) xkList_Deallocate(kList list); 

kFx(kStatus) xkList_Layout(kList list, kType itemType); 

kInlineFx(kListItemStruct*) xkList_ItemCast(kList list, kListItem item)
{
    kAssertType(list, kList);

    return kCast(kListItemStruct*, item);
}

kInlineFx(void*) xkList_ItemContent(kList list, kListItem item)
{
    kObj(kList, list);   

    return kPointer_ByteOffset(item, (kSSize)obj->contentField.offset);
}

kInlineFx(kBool) xkList_ItemIsRef(kList list)
{
    return kType_IsReference(kList_ItemType(list));
}
  
kInlineFx(kIterator) xkList_CollectionGetIterator(kList list)
{
    return kList_First(list); 
}

kInlineFx(kType) xkList_CollectionItemType(kList list)
{
    return kList_ItemType(list); 
}

kInlineFx(kBool) xkList_CollectionHasNext(kList list, kIterator iterator)
{
    return !kIsNull(iterator); 
}

kInlineFx(void*) xkList_CollectionNext(kList list, kIterator* iterator)
{
    void* content = xkList_ItemContent(list, *iterator); 

    *iterator = kList_Next(list, *iterator); 

    return content; 
}

kInlineFx(kStatus) xkList_AddT(kList list, const void* itemContent, kListItem* item, kSize itemSize)
{
    kAssert(xkType_IsPointerCompatible(kList_ItemType(list), itemSize)); 

    return kList_Add(list, itemContent, item);
} 

kInlineFx(kStatus) xkList_InsertT(kList list, kListItem before, const void* itemContent, kListItem* item, kSize itemSize)
{
    kAssert(xkType_IsPointerCompatible(kList_ItemType(list), itemSize)); 

    return kList_Insert(list, before, itemContent, item);
} 

kInlineFx(kStatus) xkList_SetItemT(kList list, kListItem item, const void* content, kSize itemSize)
{
    kAssert(xkType_IsPointerCompatible(kList_ItemType(list), itemSize)); 

    return kList_SetItem(list, item, content);
} 

kInlineFx(kStatus) xkList_ItemT(kList list, kListItem item, void* content, kSize itemSize)
{
    kAssert(xkType_IsPointerCompatible(kList_ItemType(list), itemSize)); 

    return kList_Item(list, item, content);
} 

kInlineFx(void*) xkList_AtT(kList list, kListItem item, kSize itemSize)
{
    kAssert(xkType_IsPointerCompatible(kList_ItemType(list), itemSize)); 

    return kList_At(list, item);
} 

kInlineFx(void*) xkList_AtIndexT(kList list, kSize index, kSize itemSize)
{
    kAssert(xkType_IsPointerCompatible(kList_ItemType(list), itemSize)); 

    return kList_AtIndex(list, index);
} 

kInlineFx(void*) xkList_AsT(kList list, kListItem item, kSize itemSize)
{
    kAssert(itemSize == kType_Size(kList_ItemType(list)));  

    return kList_At(list, item);
}

#endif

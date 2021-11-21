/** 
 * @file    kList.h
 * @brief   Declares the kList class. 
 *
 * @internal
 * Copyright (C) 2013-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_LIST_H
#define K_API_LIST_H

#include <kApi/kApiDef.h>

/**
 * @class       kList
 * @extends     kObject
 * @implements  kCollection
 * @ingroup     kApi-Data
 * @brief       Represents a doubly-linked list. 
 * 
 * The kList class represents a doubly-linked list of objects or values. The kList constructor accepts a kType 
 * value that determines the type of items that will be stored in the list. The list will automatically 
 * grow as new items are added or inserted. 
 * 
 * @code {.c}
 * kStatus ListExample()
 * {
 *     kList list = kNULL; 
 *     k32s values[] = { 1, 2, 3, 5, 7, 9 }; 
 *     kSize i; 
 *     kListItem it = kNULL;     //list iterator
 *     
 *     kTry
 *     {
 *          //create a list that can store 32-bit integers
 *          kTest(kList_Construct(&list, kTypeOf(k32s), 0, kNULL)); 
 *          
 *          //add some initial items to the list
 *          for (i = 0; i < kCountOf(values); ++i)
 *          {
 *              kTest(kList_AddT(list, &values[i], kNULL); 
 *          }
 *          
 *          //print some information about the list and its items
 *          printf("Item type: %s\n", kType_Name(kList_ItemType(list))); 
 *          printf("Count: %u\n", (k32u) kList_Count(list)); 
 *  
 *          it = kList_First(list); 
 *         
 *          while (!kIsNull(it))
 *          {
 *              //get an item from the list
 *              k32s value = kList_AsT(list, it, k32s); 
 *
 *              it = kList_Next(list, it); 
 *          }
 *     }
 *     kFinally
 *     {
 *          kObject_Destroy(list); 
 * 
 *          kEndFinally(); 
 *     }
 *    
 *     return kOK; 
 * }
 * 
 * @endcode
 *
 * For lists that contain <em>objects</em> (e.g. kImage) as opposed to <em>values</em> (e.g. k32s), the objects 
 * are not automatically destroyed when the list is destroyed. To recursively destroy both the list and the 
 * list items, use kObject_Dispose. 
 * 
 * kList supports the kObject_Clone, kObject_Dispose, and kObject_Size methods.
 *
 * kList supports the kdat6 serialization protocol. 
 */
//typedef kObject kList;        --forward-declared in kApiDef.x.h 

/**
 * Represents a node within a kList object. 
 * 
 * @typedef kPointer kListItem
 * @relates kList
 */
typedef kPointer kListItem; 

#include <kApi/Data/kList.x.h>

/** 
 * Constructs a kList object.
 *
 * @public                      @memberof kList
 * @param   list                List object. 
 * @param   itemType            Type of list item.
 * @param   initialCapacity     Capacity initially reserved for list items. 
 * @param   allocator           Memory allocator. 
 * @return                      Operation status. 
 */
kFx(kStatus) kList_Construct(kList* list, kType itemType, kSize initialCapacity, kAlloc allocator);

/** 
 * Reallocates the list. 
 * 
 * Existing items are discarded. 
 *
 * @public                      @memberof kList
 * @param   list                List object. 
 * @param   itemType            Type of list item.
 * @param   initialCapacity     Capacity initially reserved for list items. 
 * @return                      Operation status. 
 */
kFx(kStatus) kList_Allocate(kList list, kType itemType, kSize initialCapacity);

/** 
 * Performs a shallow copy of the source list.  
 *
 * Source items are copied by value; if the source list contains objects, the object 
 * handles are copied but the objects are not cloned. 
 *
 * @public              @memberof kList
 * @param   list        List object. 
 * @param   source      List to be copied.
 * @return              Operation status. 
 */
kFx(kStatus) kList_Assign(kList list, kList source);

/** 
 * Returns the item type. 
 *
 * @public              @memberof kList
 * @param   list        List object. 
 * @return              Item type. 
 */
kInlineFx(kType) kList_ItemType(kList list)
{
    kObj(kList, list);   

    return obj->contentField.type; 
}

/** 
 * Returns the count of list elements.  
 *
 * @public              @memberof kList
 * @param   list        List object. 
 * @return              Count of elements. 
 */
kInlineFx(kSize) kList_Count(kList list)
{
    kObj(kList, list);   

    return obj->count; 
}

/** 
 * Returns the number of elements for which space has been allocated.  
 *
 * @public              @memberof kList
 * @param   list        List object. 
 * @return              List capacity, in elements. 
 */
kInlineFx(kSize) kList_Capacity(kList list)
{
    kObj(kList, list);   

    return obj->capacity; 
}

/** 
 * Adds a new item to the end of the list.
 * 
 * Increases list capacity, if necessary.
 * 
 * @public                  @memberof kList
 * @param   list            List object. 
 * @param   itemContent     Optional pointer to item content that will be copied (by value) into the list.
 * @param   item            Optionally receives pointer to newly-inserted item.
 * @return                  Operation status. 
 */
kFx(kStatus) kList_Add(kList list, const void* itemContent, kListItem* item); 

/** 
 * Adds a new item to the end of the list.
 * 
 * Increases list capacity, if necessary.
 * 
 * A debug assertion will be raised if the size of the specified item type is not equal to the 
 * size of the collection item type.
 * 
 * @relates                     kList
 * @param   kList_list          List object. 
 * @param   TPtr_itemContent    Strongly-typed pointer to item content that will be copied (by value) into the list.
 * @param   kListItemPtr_item   Optionally receives pointer to newly-inserted item.
 * @return                      Operation status. 
 */
#define kList_AddT(kList_list, TPtr_itemContent, kListItemPtr_item) \
    xkList_AddT(kList_list, TPtr_itemContent, kListItemPtr_item, sizeof(*(TPtr_itemContent)))

/** 
 * Inserts an item into the list before the specified list item.
 *
 * Increases list capacity, if necessary.
 *
 * @public                  @memberof kList
 * @param   list            List object. 
 * @param   before          Item will be inserted before this list node (if null, inserts at tail). 
 * @param   itemContent     Optional pointer to item content that will be copied (by value) into the list.
 * @param   item            Optionally receives pointer to newly-inserted item.
 * @return                  Operation status. 
 */
kFx(kStatus) kList_Insert(kList list, kListItem before, const void* itemContent, kListItem* item); 

/** 
 * Inserts an item into the list before the specified list item.
 *
 * Increases list capacity, if necessary.
 *
 * A debug assertion will be raised if the size of the specified item type is not equal to the 
 * size of the collection item type.
 * 
 * @relates                     kList
 * @param   kList_list          List object. 
 * @param   kListItem_before    Item will be inserted before this list node (if null, inserts at tail). 
 * @param   TPtr_itemContent    Strongly-typed pointer to item content that will be copied (by value) into the list.
 * @param   kListItemPtr_item   Optionally receives pointer to newly-inserted item.
 * @return                      Operation status. 
 */
#define kList_InsertT(kList_list, kListItem_before, TPtr_itemContent, kListItemPtr_item) \
    xkList_InsertT(kList_list, kListItem_before, TPtr_itemContent, kListItemPtr_item, sizeof(*(TPtr_itemContent)))

/** 
 * Removes the specified item from the list. 
 *
 * @public              @memberof kList
 * @param   list        List object. 
 * @param   item        Node to be removed from the list.
 * @return              Operation status. 
 */
kFx(kStatus) kList_Remove(kList list, kListItem item); 

/** 
 * Sets the content associated with a list item. 
 *
 * @public              @memberof kList
 * @param   list        List object. 
 * @param   item        List item. 
 * @param   content     Pointer to content to be copied into the list item.
 * @return              Operation status. 
 */
kFx(kStatus) kList_SetItem(kList list, kListItem item, const void* content); 

/** 
 * Sets the content associated with a list item. 
 *
 * A debug assertion will be raised if the size of the specified item type is not equal to the 
 * size of the collection item type.
 * 
 * @relates                 kList
 * @param   kList_list      List object. 
 * @param   kListItem_item  List item. 
 * @param   TPtr_content    Strongly-typed pointer to content to be copied into the list item.
 * @return                  Operation status. 
 */
#define kList_SetItemT(kList_list, kListItem_item, TPtr_content) \
    xkList_SetItemT(kList_list, kListItem_item, TPtr_content, sizeof(*(TPtr_content)))

/** 
 * Gets the content associated with a list item. 
 *
 * @public              @memberof kList
 * @param   list        List object. 
 * @param   item        List item. 
 * @param   content     Destination for content that will be copied (by value) from the list item. 
 * @return              Operation status. 
 */
kFx(kStatus) kList_Item(kList list, kListItem item, void* content); 

/** 
 * Gets the content associated with a list item. 
 *
 * A debug assertion will be raised if the size of the specified item type is not equal to the 
 * size of the collection item type.
 * 
 * @relates                 kList
 * @param   kList_list      List object. 
 * @param   kListItem_item  List item. 
 * @param   TPtr_content    Strongly-typed destination pointer for content that will be copied (by value) from the list item. 
 * @return                  Operation status. 
 */
#define kList_ItemT(kList_list, kListItem_item, TPtr_content) \
    xkList_ItemT(kList_list, kListItem_item, TPtr_content, sizeof(*(TPtr_content)))

/** 
* Sets the content associated with a list item.
* 
* A debug assertion will be raised if the size of the specified data type does not match the size of the collection type.
*
* @relates                      kList
* @param   kList_list           List object. 
* @param   kListItem_item       List item. 
* @param   T_value              Item value.  
* @param   T                    Item type identifier (e.g., k32s).
* @return                       Item value.
*/
#define kList_SetAsT(kList_list, kListItem_item, T_value, T) \
    (kPointer_WriteAs(xkList_AsT(kList_list, kListItem_item, sizeof(T)), T_value, T), (void)0)

/** 
* Gets the content associated with a list item.
* 
* A debug assertion will be raised if the size of the specified data type does not match the size of the collection type.
*
* @relates                      kList
* @param   kList_list           List object. 
* @param   kListItem_item       List item. 
* @param   T                    Item type identifier (e.g., k32s).
* @return                       Item value.
*/
#define kList_AsT(kList_list, kListItem_item, T) \
    kPointer_ReadAs(xkList_AsT(kList_list, kListItem_item, sizeof(T)), T)

/** 
 * Ensures that capacity is reserved for at least the specified number of list items. 
 * 
 * @public              @memberof kList
 * @param   list        List object. 
 * @param   capacity    List capacity, in items.
 * @return              kOK if removed; kERROR_NOT_FOUND if key not found. 
 */
kFx(kStatus) kList_Reserve(kList list, kSize capacity); 

/** 
 * Sets the count of list items to zero. 
 *
 * @public              @memberof kList
 * @param   list        List object. 
 * @return              Operation status. 
 */
kFx(kStatus) kList_Clear(kList list); 

/** 
 * Disposes any elements in the list and sets the count of list items to zero.
 *
 * @public              @memberof kList
 * @param   list        List object. 
 * @return              Operation status. 
 */
kFx(kStatus) kList_Purge(kList list); 

/** 
 * Gets a reference to the first list item. 
 *
 * @public              @memberof kList
 * @param   list        List object.
 * @return              First list item, or kNULL.
 */
kInlineFx(kListItem) kList_First(kList list)
{
    kObj(kList, list);   

    return obj->first; 
}

/** 
 * Gets a reference to the last list item. 
 *
 * @public              @memberof kList
 * @param   list        List object.
 * @return              Last list item, or kNULL.
 */
kInlineFx(kListItem) kList_Last(kList list)
{
    kObj(kList, list);   

    return obj->last; 
}

/** 
 * Given a list item, gets a reference to the next list item. 
 *
 * @public              @memberof kList
 * @param   list        List object.
 * @param   item        List item.
 * @return              Next list item, or kNULL.
 */
kInlineFx(kListItem) kList_Next(kList list, kListItem item)
{
    return xkList_ItemCast(list, item)->next;
}

/** 
 * Given a list item, gets a reference to the previous list item. 
 *
 * @public              @memberof kList
 * @param   list        List object.
 * @param   item        List item.
 * @return              Previous list item, or kNULL.
 */
kInlineFx(kListItem) kList_Previous(kList list, kListItem item)
{
    return xkList_ItemCast(list, item)->previous;
}

/** 
 * Finds a reference to the list item at the specified index.
 * 
 * This method requires a linear search through the list. 
 *
 * @public              @memberof kList
 * @param   list        List object. 
 * @param   index       List item index. 
 * @return              List item at index (or kNULL if index is beyond Count).
 */
kFx(kListItem) kList_FindIndex(kList list, kSize index); 

/** 
 * Returns a pointer to the content associated with a list item. 
 *
 * @public              @memberof kList
 * @param   list        List object. 
 * @param   item        List item. 
 * @return              Pointer to content.
 */
kInlineFx(void*) kList_At(kList list, kListItem item)
{
    return xkList_ItemContent(list, item); 
}

/** 
 * Returns a strongly-typed pointer to the content associated with a list item. 
 *
 * A debug assertion will be raised if the size of the specified item type is not equal to the 
 * size of the collection item type.
 * 
 * @relates                 kList
 * @param   kList_list      List object. 
 * @param   kListItem_item  List item. 
 * @param   T               Item type identifier (e.g., k32s).
 * @return                  Strongly-typed pointer to content.
 */
#define kList_AtT(kList_list, kListItem_item, T) \
    kCast(T*, xkList_AtT(kList_list, kListItem_item, sizeof(T)))

/** 
 * Returns a pointer to the content associated with a list item at the specified index.
 *
 * This method requires a linear search through the list. 
 *
 * @public              @memberof kList
 * @param   list        List object. 
 * @param   index       Item index.  
 * @return              Pointer to content (or kNULL if index is beyond Count).
 */
kFx(void*) kList_AtIndex(kList list, kSize index); 

/** 
 * Returns a strongly-typed pointer to the content associated with a list item at the specified index.
 *
 * This method requires a linear search through the list. 
 *
 * A debug assertion will be raised if the size of the specified item type is not equal to the 
 * size of the collection item type.
 * 
 * @relates                 kList
 * @param   kList_list      List object. 
 * @param   kSize_index     Item index.  
 * @param   T               Item type identifier (e.g., k32s).
 * @return                  Strongly-typed pointer to content (or kNULL if index is beyond Count).
 */
#define kList_AtIndexT(kList_list, kSize_index, T) \
    kCast(T*, xkList_AtIndexT(kList_list, kSize_index, sizeof(T)))

#endif

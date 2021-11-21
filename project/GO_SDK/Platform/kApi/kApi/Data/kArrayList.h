/** 
 * @file    kArrayList.h
 * @brief   Declares the kArrayList class. 
 *
 * @internal
 * Copyright (C) 2005-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_ARRAY_LIST_H
#define K_API_ARRAY_LIST_H

#include <kApi/kApiDef.h> 
#include <kApi/Data/kArrayList.x.h>

/**
 * @class       kArrayList
 * @extends     kObject
 * @implements  kCollection
 * @ingroup     kApi-Data
 * @brief       Represents a list implemented with a dynamic array.
 * 
 * The kArrayList class represents a dynamic, array-based list of objects or values. The kArrayList constructor 
 * accepts a kType value that determines the type of items that will be stored in the list. The list will 
 * automatically grow as new items are added. 
 * 
 * @code {.c}
 * kStatus ArrayListExample()
 * {
 *     kArrayList list = kNULL; 
 *     k32s values[] = { 1, 2, 3, 5, 7, 9 }; 
 *     kSize i; 
 *     
 *     kTry
 *     {
 *          //create a list that can store 32-bit integers
 *          kTest(kArrayList_Construct(&list, kTypeOf(k32s), 0, kNULL)); 
 *          
 *          //add some initial items to the list
 *          for (i = 0; i < kCountOf(values); ++i)
 *          {
 *              kTest(kArrayList_AddT(list, &values[i]); 
 *          }
 *          
 *          //print some information about the list and its items
 *          printf("Item type: %s\n", kType_Name(kArrayList_ItemType(list))); 
 *          printf("Count: %u\n", (k32u) kArrayList_Count(list)); 
 *  
 *          for (i = 0; i < kArrayList_Count(list); ++i)
 *          {
 *              //get an item from the list
 *              k32s value = kArrayList_AsT(list, i, k32s); 
 *
 *              printf("Item %u: %d\n", (k32u)i, value);  
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
 * kArrayList supports the kObject_Clone, kObject_Dispose, and kObject_Size methods.
 * 
 * kArrayList supports the kdat5 and kdat6 serialization protocols. 
 */
//typedef kObject kArrayList;   --forward-declared in kApiDef.x.h

/** 
 * Constructs a kArrayList object.
 *
 * @public                      @memberof kArrayList
 * @param   list                Receives constructed list object. 
 * @param   itemType            Type of list element.
 * @param   initialCapacity     Capacity initially reserved for list items. 
 * @param   allocator           Memory allocator (or kNULL for default). 
 * @return                      Operation status. 
 */
kFx(kStatus) kArrayList_Construct(kArrayList* list, kType itemType, kSize initialCapacity, kAlloc allocator);

/** 
 * Reallocates the list item buffer.
 *
 * @public                      @memberof kArrayList
 * @param   list                List object. 
 * @param   itemType            Type of list element.
 * @param   initialCapacity     Capacity initially reserved for list items. 
 * @return                      Operation status. 
 */
kFx(kStatus) kArrayList_Allocate(kArrayList list, kType itemType, kSize initialCapacity);

/** 
 * Attaches the list object to an external buffer. 
 * 
 * The list count is set to the same value as the list capacity argument. 
 *
 * @public                      @memberof kArrayList
 * @param   list                List object. 
 * @param   items               Item buffer.
 * @param   itemType            Type of list element.
 * @param   capacity            List capacity. 
 * @return                      Operation status. 
 */
kFx(kStatus) kArrayList_Attach(kArrayList list, void* items, kType itemType, kSize capacity); 

/** 
 * Attaches the list object to an external buffer. 
 * 
 * The list count is set to the same value as the list capacity argument. 
 *
 * A debug assertion will be raised if the size of the specified items is not equal to the 
 * size of the specified item type.
 * 
 * @relates                     kArrayList
 * @param   kArrayList_list     List object. 
 * @param   TPtr_items          Strongly-typed pointer to external item buffer.
 * @param   kType_itemType      Type of list element.
 * @param   kSize_capacity      List capacity. 
 * @return                      Operation status. 
 */
#define kArrayList_AttachT(kArrayList_list, TPtr_items, kType_itemType, kSize_capacity) \
    xkArrayList_AttachT(kArrayList_list, TPtr_items, kType_itemType, kSize_capacity, sizeof(*(TPtr_items)))

/** 
 * Copies the specified items into the list, replacing existing contents. 
 *
 * The list count is set to the value of the count argument. 
 *
 * @public                      @memberof kArrayList
 * @param   list                List object. 
 * @param   items               Item buffer. 
 * @param   itemType            Type of list element.
 * @param   count               Count of list items. 
 * @return                      Operation status. 
 */
kFx(kStatus) kArrayList_Import(kArrayList list, const void* items, kType itemType, kSize count); 

/** 
 * Copies the specified items into the list, replacing existing contents. 
 *
 * The list count is set to the value of the count argument. 
 *
 * A debug assertion will be raised if the size of the specified items is not equal to the 
 * size of the specified item type.
 * 
 * @relates                     kArrayList
 * @param   kArrayList_list     List object. 
 * @param   TPtr_items          Strongly-typed pointer to items that should be imported.
 * @param   kType_itemType      Type of list element.
 * @param   kSize_count         Count of list items. 
 * @return                      Operation status. 
 */
#define kArrayList_ImportT(kArrayList_list, TPtr_items, kType_itemType, kSize_count) \
    xkArrayList_ImportT(kArrayList_list, TPtr_items, kType_itemType, kSize_count, sizeof(*(TPtr_items)))

/** 
 * Appends the specified items to the list. 
 *
 * @public                      @memberof kArrayList
 * @param   list                List object. 
 * @param   items               Item buffer. 
 * @param   count               Count of list items. 
 * @return                      Operation status. 
 */
kFx(kStatus) kArrayList_Append(kArrayList list, const void* items, kSize count); 

/** 
 * Appends the specified items to the list. 
 *
 * A debug assertion will be raised if the size of the specified item type is not equal to the 
 * size of the collection item type.
 * 
 * @relates                     kArrayList
 * @param   kArrayList_list     List object. 
 * @param   TPtr_items          Strongly-typed pointer to items that should be appended. 
 * @param   kSize_count         Count of list items. 
 * @return                      Operation status. 
 */
#define kArrayList_AppendT(kArrayList_list, TPtr_items, kSize_count) \
    xkArrayList_AppendT(kArrayList_list, TPtr_items, kSize_count, sizeof(*(TPtr_items)))

/** 
 * Performs a shallow copy of the source list.  
 *
 * Source items are copied by value; if the source list contains objects, the object 
 * handles are copied but the objects are not cloned. 
 *
 * @public              @memberof kArrayList
 * @param   list        List object. 
 * @param   source      Source list to be copied.
 * @return              Operation status. 
 */
kFx(kStatus) kArrayList_Assign(kArrayList list, kArrayList source);

/** 
 * Sets the count of list items to zero. 
 *
 * @public              @memberof kArrayList
 * @param   list        List object. 
 * @return              Operation status. 
 */
kInlineFx(kStatus) kArrayList_Clear(kArrayList list)
{
    kObj(kArrayList, list); 

    obj->count = 0; 

    return kOK; 
}

/** 
 * Disposes any elements in the list and sets the count of list items to zero.
 *
 * @public              @memberof kArrayList
 * @param   list        List object. 
 * @return              Operation status. 
 */
kFx(kStatus) kArrayList_Purge(kArrayList list); 

/** 
 * Sets the memory for all list elements to zero. 
 *
 * @public              @memberof kArrayList
 * @param   list        List object. 
 * @return              Operation status. 
 */
kFx(kStatus) kArrayList_Zero(kArrayList list); 

/** 
 * Adds the specified item to the end of the list.
 *
 * @public              @memberof kArrayList
 * @param   list        List object. 
 * @param   item        Pointer to item that will be copied (by value) into the list.
 * @return              Operation status. 
 */
kFx(kStatus) kArrayList_Add(kArrayList list, const void* item);

/** 
 * Adds the specified item to the end of the list.
 *
 * A debug assertion will be raised if the size of the specified item type is not equal to the 
 * size of the collection item type.
 * 
 * @relates                     kArrayList
 * @param   kArrayList_list     List object. 
 * @param   TPtr_item           Strongly-typed pointer to item that will be copied (by value) into the list.
 * @return                      Operation status. 
 */
#define kArrayList_AddT(kArrayList_list, TPtr_item) \
    xkArrayList_AddT(kArrayList_list, TPtr_item, sizeof(*(TPtr_item)))

/** 
 * Inserts an item into the list at the specified position.
 *
 * Increases list capacity, if necessary.
 *
 * @public              @memberof kArrayList
 * @param   list        List object. 
 * @param   before      Item will be inserted before the item at this index.
 * @param   item        Pointer to item that will be copied (by value) into the list.
 * @return              Operation status. 
 */
kFx(kStatus) kArrayList_Insert(kArrayList list, kSize before, const void* item); 

/** 
 * Inserts an item into the list at the specified position.
 *
 * Increases list capacity, if necessary.
 *
 * A debug assertion will be raised if the size of the specified item type is not equal to the 
 * size of the collection item type.
 * 
 * @relates                     kArrayList
 * @param   kArrayList_list     List object. 
 * @param   kSize_before        Item will be inserted before the item at this index.
 * @param   TPtr_item           Strongly-typed pointer to item that will be copied (by value) into the list.
 * @return                      Operation status. 
 */
#define kArrayList_InsertT(kArrayList_list, kSize_before, TPtr_item) \
    xkArrayList_InsertT(kArrayList_list, kSize_before, TPtr_item, sizeof(*(TPtr_item)))

/** 
 * Removes an item from the list at the specified index, optionally returning the value.
 *
 * @public              @memberof kArrayList
 * @param   list        List object. 
 * @param   index       Item at this index will be removed from the list.
 * @param   item        Destination for the removed item (copied by value, can be null).
 * @return              Operation status. 
 */
kFx(kStatus) kArrayList_Remove(kArrayList list, kSize index, void* item); 

/** 
 * Removes an item from the list at the specified index. 
 *
 * A debug assertion will be raised if the size of the specified item type is not equal to the 
 * size of the collection item type. 
 * 
 * @relates                     kArrayList
 * @param   kArrayList_list     List object. 
 * @param   kSize_index         Item at this index will be removed from the list.
 * @param   TPtr_item           Strongly-typed destination pointer for the removed item (copied by value).
 * @return                      Operation status. 
 */
#define kArrayList_RemoveT(kArrayList_list, kSize_index, TPtr_item) \
    xkArrayList_RemoveT(kArrayList_list, kSize_index, TPtr_item, sizeof(*(TPtr_item)))

/** 
 * Removes an item from the list at the specified index. 
 *
 * Discared objects are not automatically disposed. If the object must be destroyed or disposed, 
 * used kArrayList_Remove instead to receive the removed object.  
 * 
 * @public              @memberof kArrayList
 * @param   list        List object. 
 * @param   index       Item at this index will be removed from the list.
 * @return              Operation status. 
 */
kInlineFx(kStatus) kArrayList_Discard(kArrayList list, kSize index)
{
    return kArrayList_Remove(list, index, kNULL);
}

/** 
 * Sets the value of an item.
 *
 * @public              @memberof kArrayList
 * @param   list        List object. 
 * @param   index       Item index. 
 * @param   item        Pointer to item that will be copied into the list.
 * @return              Operation status. 
 */
kFx(kStatus) kArrayList_SetItem(kArrayList list, kSize index, const void* item);

/** 
 * Sets the value of an item.
 *
 * A debug assertion will be raised if the size of the specified item type is not equal to the 
 * size of the collection item type.
 * 
 * @relates                     kArrayList
 * @param   kArrayList_list     List object. 
 * @param   kSize_index         Item at this index will be removed from the list.
 * @param   TPtr_item           Strongly-typed pointer to item that will be copied into the list.
 * @return                      Operation status. 
 */
#define kArrayList_SetItemT(kArrayList_list, kSize_index, TPtr_item) \
    xkArrayList_SetItemT(kArrayList_list, kSize_index, TPtr_item, sizeof(*(TPtr_item)))

/** 
 * Gets the value of an item. 
 *
 * @public              @memberof kArrayList
 * @param   list        List object. 
 * @param   index       Item index.  
 * @param   item        Destination for item that will be copied (by value) from the list. 
 * @return              Operation status. 
 */
kFx(kStatus) kArrayList_Item(kArrayList list, kSize index, void* item); 

/** 
 * Gets the value of an item. 
 *
 * A debug assertion will be raised if the size of the specified item type is not equal to the 
 * size of the collection item type.
 * 
 * @relates                     kArrayList
 * @param   kArrayList_list     List object. 
 * @param   kSize_index         Item index. 
 * @param   TPtr_item           Strongly-typed destination pointer for received item.
 * @return                      Operation status. 
 */
#define kArrayList_ItemT(kArrayList_list, kSize_index, TPtr_item) \
    xkArrayList_ItemT(kArrayList_list, kSize_index, TPtr_item, sizeof(*(TPtr_item)))

/** 
* Sets the value of an item. 
* 
* A debug assertion will be raised if the index argument is greater than or equal to 
* the current list length, or if the size of the specified item type is not equal to the 
* size of the collection item type.
* 
* @relates                      kArrayList 
* @param   kArrayList_list      List object. 
* @param   kSize_index          Item index.
* @param   T_value              Item value.  
* @param   T                    Item type identifier (e.g., k32s).
* @return                       Item value. 
*/
#define kArrayList_SetAsT(kArrayList_list, kSize_index, T_value, T) \
    (kPointer_WriteAs(xkArrayList_AsT(kArrayList_list, kSize_index, sizeof(T)), T_value, T), (void)0)

/** 
* Gets the value of an item. 
* 
* A debug assertion will be raised if the index argument is greater than or equal to 
* the current list length, or if the size of the specified item type is not equal to the 
* size of the collection item type.
* 
* @relates                      kArrayList 
* @param   kArrayList_list      List object. 
* @param   kSize_index          Item index.
* @param   T                    Item type identifier (e.g., k32s).
* @return                       Item value. 
*/
#define kArrayList_AsT(kArrayList_list, kSize_index, T) \
    kPointer_ReadAs(xkArrayList_AsT(kArrayList_list, kSize_index, sizeof(T)), T)

/** 
 * Sets the current count of list items to the specified value.
 *
 * Increases list capacity if necessary; existing list items are preserved. 
 *
 * @public              @memberof kArrayList
 * @param   list        List object. 
 * @param   count       List size, in items.
 * @return              Operation status. 
 */
kFx(kStatus) kArrayList_Resize(kArrayList list, kSize count); 

/** 
 * Increases the list count by the specified amount.
 *
 * Increases list capacity if necessary; existing list items are preserved. 
 * New items are not initialized. 
 *
 * @public              @memberof kArrayList
 * @param   list        List object. 
 * @param   count       Amount to add to the existing count.
 * @return              Operation status. 
 */
kInlineFx(kStatus) kArrayList_AddCount(kArrayList list, kSize count)
{
    kObj(kArrayList, list); 

    if ((obj->count + count) > obj->capacity)
    {
        kCheck(kArrayList_Reserve(list, obj->count + count)); 
    }

    obj->count += count; 

    return kOK; 
}

/** 
 * Decreases the list count by the specified amount.
 *
 * @public              @memberof kArrayList
 * @param   list        List object. 
 * @param   count       Amount to remove from the existing count.
 * @return              Operation status. 
 */
kInlineFx(kStatus) kArrayList_RemoveCount(kArrayList list, kSize count)
{
    kObj(kArrayList, list); 

    if (count > obj->count)
    {
        return kERROR_STATE; 
    }

    obj->count -= count; 

    return kOK; 
}

/** 
 * Ensures that capacity is reserved for at least the specified number of list items. 
 *
 * Existing list items are preserved. 
 *
 * @public              @memberof kArrayList
 * @param   list        List object. 
 * @param   capacity    List capacity, in items.
 * @return              Operation status. 
 */
kFx(kStatus) kArrayList_Reserve(kArrayList list, kSize capacity); 

/** 
 * Returns a pointer to the list item buffer. 
 *
 * @public              @memberof kArrayList
 * @param   list        List object. 
 * @return              Pointer to list items. 
 */
kInlineFx(void*) kArrayList_Data(kArrayList list)
{
    kObj(kArrayList, list); 

    return obj->items;
}

/** 
 * Returns a strongly-typed pointer to the list item buffer. 
 *
 * A debug assertion will be raised if the size of the specified item type is not equal to the 
 * size of the collection item type.
 * 
 * @relates                     kArrayList
 * @param   kArrayList_list     List object. 
 * @param   T                   Item type identifier (e.g., k32s).
 * @return                      Strongly-typed pointer to list items. 
 */
#define kArrayList_DataT(kArrayList_list, T) \
    kCast(T*, xkArrayList_DataT(kArrayList_list, sizeof(T)))

/** 
 * Calculates an address relative to the start of the buffer.
 *
 * This method is similar to the At method, but does not bounds-check the index argument. 
 * 
 * @public              @memberof kArrayList
 * @param   list        List object. 
 * @param   index       Item index.
 * @return              Calculated address.
 */
kInlineFx(void*) kArrayList_DataAt(kArrayList list, kSSize index)
{
    kObj(kArrayList, list); 
    
    return kPointer_ItemOffset(obj->items, index, obj->itemSize);
}

/** 
 * Returns a strongly-typed pointer address relative to the start of the buffer.
 *
 * A debug assertion will be raised if the size of the specified item type is not equal to the 
 * size of the collection item type.
 * 
 * @relates                     kArrayList
 * @param   kArrayList_list     List object. 
 * @param   kSSize_index        Item index.
 * @param   T                   Item type identifier (e.g., k32s).
 * @return                      Strongly-typed pointer to calculated address.
 */
#define kArrayList_DataAtT(kArrayList_list, kSSize_index, T) \
    kCast(T*, xkArrayList_DataAtT(kArrayList_list, kSSize_index, sizeof(T)))

/** 
 * Returns the total size of list data (Count x ItemSize), in bytes. 
 *
 * @public              @memberof kArrayList
 * @param   list        List object. 
 * @return              Data size, in bytes.  
 */
kInlineFx(kSize) kArrayList_DataSize(kArrayList list)
{
    kObj(kArrayList, list); 

    return obj->count * obj->itemSize; 
}

/** 
 * Returns a pointer to the specified item in the list buffer. 
 *
 * A debug assertion will be raised if the index argument is greater than or equal to 
 * the current list count (note: count, not capacity).  
 * 
 * @public              @memberof kArrayList
 * @param   list        List object. 
 * @param   index       Item index.
 * @return              Pointer to list element.
 */
kInlineFx(void*) kArrayList_At(kArrayList list, kSize index) 
{    
#   if !defined(K_FSS_912_DISABLE_BOUNDS_CHECK)
    {
        kAssert(index < kArrayList_Count(list));
    }
#   endif

    return kArrayList_DataAt(list, (kSSize)index);
}

/** 
 * Returns a strongly-typed pointer to the specified item in the list buffer. 
 *
 * A debug assertion will be raised if the index argument is greater than or equal to 
 * the current list count (note: count, not capacity), or if the size of the specified 
 * item type is not equal to the size of the collection item type.
 * 
 * @relates                     kArrayList
 * @param   kArrayList_list     List object. 
 * @param   kSize_index         Item index.
 * @param   T                   Item type identifier (e.g., k32s).
 * @return                      Pointer to list element.
 */
#define kArrayList_AtT(kArrayList_list, kSize_index, T) \
    kCast(T*, xkArrayList_AtT(kArrayList_list, kSize_index, sizeof(T)))

/** 
 * Returns the list element type. 
 *
 * @public              @memberof kArrayList
 * @param   list        List object. 
 * @return              Item type. 
 */
kInlineFx(kType) kArrayList_ItemType(kArrayList list)
{
    kObj(kArrayList, list); 

    return obj->itemType;
}

/** 
 * Returns the list element size.
 *
 * @public              @memberof kArrayList
 * @param   list        List object. 
 * @return              Item size, in bytes. 
 */
kInlineFx(kSize) kArrayList_ItemSize(kArrayList list)
{
    kObj(kArrayList, list); 

    return obj->itemSize;
}

/** 
 * Returns the current count of items in the list. 
 *
 * @public              @memberof kArrayList
 * @param   list        List object. 
 * @return              Current count of items. 
 */
kInlineFx(kSize) kArrayList_Count(kArrayList list)
{
    kObj(kArrayList, list); 

    return obj->count;
}

/** 
 * Returns the number of elements for which space has been allocated.  
 *
 * @public              @memberof kArrayList
 * @param   list        List object. 
 * @return              List capacity. 
 */
kInlineFx(kSize) kArrayList_Capacity(kArrayList list)
{
    kObj(kArrayList, list); 

    return obj->capacity;
}

/** 
 * Returns a pointer to the first item in the list.  
 *
 * @public              @memberof kArrayList
 * @param   list        List object. 
 * @return              Pointer to first item.
 */
kInlineFx(void*) kArrayList_First(kArrayList list)
{
    return kArrayList_Begin(list);
}

/** 
 * Returns a strongly-typed pointer to the first item in the list.  
 *
 * A debug assertion will be raised if the size of the specified item type is not equal to the 
 * size of the collection item type.
 * 
 * @relates                     kArrayList
 * @param   kArrayList_list     List object. 
 * @param   T                   Item type identifier (e.g., k32s).
 * @return                      Strongly-typed pointer to first item.
 */
#define kArrayList_FirstT(kArrayList_list, T) \
    kCast(T*, xkArrayList_FirstT(kArrayList_list, sizeof(T)))

/** 
 * Returns a pointer to the last item in the list.  
 *
 * @public              @memberof kArrayList
 * @param   list        List object. 
 * @return              Pointer to last item.
 */
kInlineFx(void*) kArrayList_Last(kArrayList list)
{    
    return kArrayList_RBegin(list);
}

/** 
 * Returns a strongly-typed pointer to the last item in the list.  
 *
 * A debug assertion will be raised if the size of the specified item type is not equal to the 
 * size of the collection item type.
 * 
 * @relates                     kArrayList
 * @param   kArrayList_list     List object. 
 * @param   T                   Item type identifier (e.g., k32s).
 * @return                      Strongly-typed pointer to last item.
 */
#define kArrayList_LastT(kArrayList_list, T) \
    kCast(T*, xkArrayList_LastT(kArrayList_list, sizeof(T)))

/** 
 * Returns a pointer to the first item for forward iteration.  
 *
 * @public              @memberof kArrayList
 * @param   list        List object. 
 * @return              Pointer to first item, in forward order.
 */
kInlineFx(void*) kArrayList_Begin(kArrayList list)
{
    kObj(kArrayList, list); 

    return obj->items;
}

/** 
 * Returns a strongly-typed pointer to the first item for forward iteration.  
 *
 * A debug assertion will be raised if the size of the specified item type is not equal to the 
 * size of the collection item type.
 * 
 * @relates                     kArrayList
 * @param   kArrayList_list     List object. 
 * @param   T                   Item type identifier (e.g., k32s).
 * @return                      Strongly-typed pointer to first item, in forward order.
 */
#define kArrayList_BeginT(kArrayList_list, T) \
    kCast(T*, xkArrayList_BeginT(kArrayList_list, sizeof(T)))

/** 
 * Returns a pointer to one past the last item for forward iteration.  
 *
 * @public              @memberof kArrayList
 * @param   list        List object. 
 * @return              Pointer to one past the last element, in forward order.
 */
kInlineFx(void*) kArrayList_End(kArrayList list)
{
    kObj(kArrayList, list); 

    return kPointer_ItemOffset(obj->items, (kSSize)obj->count, obj->itemSize);
}

/** 
 * Returns a strongly-typed pointer to one past the last item for forward iteration.  
 *
 * A debug assertion will be raised if the size of the specified item type is not equal to the 
 * size of the collection item type.
 * 
 * @relates                     kArrayList
 * @param   kArrayList_list     List object. 
 * @param   T                   Item type identifier (e.g., k32s).
 * @return                      Strongly-typed pointer to one past the last element, in forward order.
 */
#define kArrayList_EndT(kArrayList_list, T) \
    kCast(T*, xkArrayList_EndT(kArrayList_list, sizeof(T)))

/** 
 * Returns a pointer to the first item for reverse iteration.  
 *
 * @public              @memberof kArrayList
 * @param   list        List object. 
 * @return              Pointer to first item, in reverse order.
 */
kInlineFx(void*) kArrayList_RBegin(kArrayList list)
{
    kObj(kArrayList, list); 

    return kPointer_ItemOffset(obj->items, (kSSize)obj->count-1, obj->itemSize);
}

/** 
 * Returns a strongly-typed pointer to the first item for reverse iteration.  
 *
 * A debug assertion will be raised if the size of the specified item type is not equal to the 
 * size of the collection item type.
 * 
 * @relates                     kArrayList
 * @param   kArrayList_list     List object. 
 * @param   T                   Item type identifier (e.g., k32s).
 * @return                      Strongly-typed pointer to first item, in reverse order.
 */
#define kArrayList_RBeginT(kArrayList_list, T) \
    kCast(T*, xkArrayList_RBeginT(kArrayList_list, sizeof(T)))

/** 
 * Returns a pointer to one past the last item for reverse iteration.  
 *
 * @public              @memberof kArrayList
 * @param   list        List object. 
 * @return              Pointer to one past the last element, in reverse order.
 */
kInlineFx(void*) kArrayList_REnd(kArrayList list)
{
    kObj(kArrayList, list); 

    return kPointer_ItemOffset(obj->items, (kSSize)-1, obj->itemSize);
}

/** 
 * Returns a strongly-typed pointer to one past the last item for reverse iteration.  
 *
 * A debug assertion will be raised if the size of the specified item type is not equal to the 
 * size of the collection item type.
 * 
 * @relates                     kArrayList
 * @param   kArrayList_list     List object. 
 * @param   T                   Item type identifier (e.g., k32s).
 * @return                      Strongly-typed pointer to one past the last element, in reverse order.
 */
#define kArrayList_REndT(kArrayList_list, T) \
    kCast(T*, xkArrayList_REndT(kArrayList_list, sizeof(T)))

#endif

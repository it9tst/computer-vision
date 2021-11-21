/** 
 * @file    kQueue.h
 * @brief   Declares the kQueue class. 
 *
 * @internal
 * Copyright (C) 2005-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_QUEUE_H
#define K_API_QUEUE_H

#include <kApi/kApiDef.h>
#include <kApi/Data/kQueue.x.h>

/**
 * @class       kQueue
 * @extends     kObject
 * @implements  kCollection
 * @ingroup     kApi-Data
 * @brief       Represents a FIFO queue implemented with a dynamic array.
 * 
 * The kQueue class represents a dynamic, array-based queue of objects or values. The kQueue constructor 
 * accepts a kType value that determines the type of items that will be stored in the queue. The queue will 
 * automatically grow as new items are added. 
 * 
 * @code {.c}
 * kStatus QueueExample()
 * {
 *     kQueue queue = kNULL; 
 *     k32s values[] = { 1, 2, 3, 5, 7, 9 }; 
 *     kSize i; 
 *     
 *     kTry
 *     {
 *          //create a queue that can store 32-bit integers
 *          kTest(kQueue_Construct(&queue, kTypeOf(k32s), 0, kNULL)); 
 *          
 *          //add some initial items to the queue
 *          for (i = 0; i < kCountOf(values); ++i)
 *          {
 *              kTest(kQueue_Add(queue, &values[i]); 
 *          }
 *          
 *          //print some information about the queue and its items
 *          printf("Item type: %s\n", kType_Name(kQueue_ItemType(queue))); 
 *          printf("Count: %u\n", (k32u) kQueue_Count(queue)); 
 *  
 *          for (i = 0; i < kQueue_Count(queue); ++i)
 *          {
 *              //the kQueue_As_ macro can be used to get a queue item and cast it to the desired type; 
 *              //this is equivalent to *(k32s*)kQueue_At(queue, i);
 *              k32s value = kQueue_As_(queue, i, k32s); 
 *
 *              printf("Item %u: %d\n", (k32u)i, value);  
 *          }
 *     }
 *     kFinally
 *     {
 *          kObject_Destroy(queue); 
 * 
 *          kEndFinally(); 
 *     }
 *    
 *     return kOK; 
 * }
 * 
 * @endcode
 *
 * For queues that contain <em>objects</em> (e.g. kImage) as opposed to <em>values</em> (e.g. k32s), the objects 
 * are not automatically destroyed when the queue is destroyed. To recursively destroy both the queue and the 
 * queue items, use kObject_Dispose.  
 *
 * kQueue supports the kObject_Clone, kObject_Dispose, and kObject_Size methods.
 * 
 * kQueue supports the kdat5 and kdat6 serialization protocols.
 */
//typedef kObject kQueue;       --forward-declared in kApiDef.x.h 

/** 
 * Constructs a kQueue object.
 *
 * @public                      @memberof kQueue
 * @param   queue               Receives constructed queue object. 
 * @param   itemType            Type of queue element.
 * @param   initialCapacity     Capacity initially reserved for queue items. 
 * @param   allocator           Memory allocator (or kNULL for default). 
 * @return                      Operation status. 
 */
kFx(kStatus) kQueue_Construct(kQueue* queue, kType itemType, kSize initialCapacity, kAlloc allocator); 

/** 
 * Reallocates the queue item buffer.
 *
 * @public                      @memberof kQueue
 * @param   queue               Queue object. 
 * @param   itemType            Type of queue element.
 * @param   initialCapacity     Capacity initially reserved for queue items. 
 * @return                      Operation status. 
 */
kFx(kStatus) kQueue_Allocate(kQueue queue, kType itemType, kSize initialCapacity); 

/** 
 * Performs a shallow copy of the source queue.  
 *
 * Source items are copied by value; if the source queue contains objects, the object 
 * handles are copied but the objects are not cloned. 
 *
 * @public              @memberof kQueue
 * @param   queue       Queue object. 
 * @param   source      Source queue to be copied.
 * @return              Operation status. 
 */
kFx(kStatus) kQueue_Assign(kQueue queue, kQueue source); 

/** 
 * Sets the count of queue items to zero. 
 *
 * @public              @memberof kQueue
 * @param   queue       Queue object. 
 * @return              Operation status. 
 */
kInlineFx(kStatus) kQueue_Clear(kQueue queue)
{
    kObj(kQueue, queue); 

    obj->count = 0; 
    obj->head = 0; 

    return kOK; 
}

/** 
 * Disposes any elements in the queue and sets the count of queue items to zero.
 *
 * @public              @memberof kQueue
 * @param   queue       Queue object. 
 * @return              Operation status. 
 */
kFx(kStatus) kQueue_Purge(kQueue queue); 

/** 
 * Ensures that capacity is reserved for at least the specified number of queue items. 
 *
 * Existing queue items are preserved. 
 *
 * @public              @memberof kQueue
 * @param   queue       Queue object. 
 * @param   capacity    Queue capacity, in items.
 * @return              Operation status. 
 */
kFx(kStatus) kQueue_Reserve(kQueue queue, kSize capacity); 

/** 
 * Adds the specified item to the end of the queue.
 *
 * @public              @memberof kQueue
 * @param   queue       Queue object. 
 * @param   item        Pointer to item that will be copied (by value) into the queue.
 * @return              Operation status. 
 */
kFx(kStatus) kQueue_Add(kQueue queue, const void* item); 

/** 
 * Adds the specified item to the end of the queue.
 *
 * A debug assertion will be raised if the size of the specified item type is not equal to the 
 * size of the collection item type.
 * 
 * @relates                 kQueue
 * @param   kQueue_queue    Queue object. 
 * @param   TPtr_item       Strongly-typed pointer to item that will be copied (by value) into the queue.
 * @return                  Operation status. 
 */
#define kQueue_AddT(kQueue_queue, TPtr_item) \
    xkQueue_AddT(kQueue_queue, TPtr_item, sizeof(*(TPtr_item)))

/** 
 * Removes the item at the head of the queue. 
 *
 * @public              @memberof kQueue
 * @param   queue       Queue object. 
 * @param   item        Destination for the removed item (copied by value, can be null).
 * @return              Operation status. 
 */
kFx(kStatus) kQueue_Remove(kQueue queue, void* item);

/** 
 * Sets the value of an item.
 *
 * @public              @memberof kQueue
 * @param   queue       Queue object. 
 * @param   index       Item index. 
 * @param   item        Pointer to item that will be copied into the queue.
 * @return              Operation status. 
 */
kFx(kStatus) kQueue_SetItem(kQueue queue, kSize index, const void* item); 

/** 
 * Sets the value of an item.
 *
 * A debug assertion will be raised if the size of the specified item type is not equal to the 
 * size of the collection item type.
 * 
 * @relates                 kQueue
 * @param   kQueue_queue    Queue object. 
 * @param   kSize_index     Item index. 
 * @param   TPtr_item       Strongly-typed pointer to item that will be copied into the queue.
 * @return                  Operation status. 
 */
#define kQueue_SetItemT(kQueue_queue, kSize_index, TPtr_item) \
    xkQueue_SetItemT(kQueue_queue, kSize_index, TPtr_item, sizeof(*(TPtr_item)))

/** 
 * Gets the value of an item. 
 *
 * @public              @memberof kQueue
 * @param   queue       Queue object. 
 * @param   index       Item index.  
 * @param   item        Destination for item that will be copied (by value) from the queue. 
 * @return              Operation status. 
 */
kFx(kStatus) kQueue_Item(kQueue queue, kSize index, void* item);  

/** 
 * Gets the value of an item. 
 *
 * A debug assertion will be raised if the size of the specified item type is not equal to the 
 * size of the collection item type.
 * 
 * @relates                 kQueue
 * @param   kQueue_queue    Queue object. 
 * @param   kSize_index     Item index.  
 * @param   TPtr_item       Strongly-typed destination pointer for item that will be copied (by value) from the queue. 
 * @return                  Operation status. 
 */
#define kQueue_ItemT(kQueue_queue, kSize_index, TPtr_item) \
     xkQueue_ItemT(kQueue_queue, kSize_index, TPtr_item, sizeof(*(TPtr_item)))

/** 
* Sets the value of an item. 
* 
* A debug assertion will be raised if the size of the specified item type is not equal to the 
* size of the collection item type.
*
* @relates                  kQueue
* @param   kQueue_queue     Queue object. 
* @param   kSize_index      Item index.  
* @param   T_value          Item value.  
* @param   T                Item type identifier (e.g., k32s).
* @return                   Item value. 
*/
#define kQueue_SetAsT(kQueue_queue, kSize_index, T_value, T) \
    (kPointer_WriteAs(xkQueue_AsT(kQueue_queue, kSize_index, sizeof(T)), T_value, T), (void)0)

/** 
* Gets the value of an item. 
* 
* A debug assertion will be raised if the size of the specified item type is not equal to the 
* size of the collection item type.
*
* @relates                  kQueue
* @param   kQueue_queue     Queue object. 
* @param   kSize_index      Item index.  
* @param   T                Item type identifier (e.g., k32s).
* @return                   Item value. 
*/
#define kQueue_AsT(kQueue_queue, kSize_index, T) \
    kPointer_ReadAs(xkQueue_AsT(kQueue_queue, kSize_index, sizeof(T)), T)

/** 
 * Increases the queue count by the specified amount.
 *
 * Increases queue capacity if necessary; existing queue items are preserved. 
 * New items are not initialized. 
 *
 * @public              @memberof kQueue
 * @param   queue       Queue object. 
 * @param   count       Amount to add to the existing count.
 * @return              Operation status. 
 */
kInlineFx(kStatus) kQueue_AddCount(kQueue queue, kSize count)
{
    kObj(kQueue, queue); 

    if ((obj->count + count) > obj->capacity)
    {
        kCheck(kQueue_Reserve(queue, obj->count + count)); 
    }

    obj->count += count; 

    return kOK; 
}

/** 
 * Decreases the queue count by the specified amount.
 *
 * @public              @memberof kQueue
 * @param   queue       Queue object. 
 * @param   count       Amount to remove from the existing count.
 * @return              Operation status. 
 */
kInlineFx(kStatus) kQueue_RemoveCount(kQueue queue, kSize count)
{
    kObj(kQueue, queue); 

    if (count > obj->count)
    {
        return kERROR_STATE; 
    }

    obj->count -= count; 
    obj->head += count; 

    if (obj->head >= obj->capacity)
    {
        obj->head -= obj->capacity; 
    }

    return kOK; 
}

/** 
 * Calculates an element address.
 *
 * This method is similar to the At method, but does not bounds-check the index argument. 
 * 
 * @public              @memberof kQueue
 * @param   queue       Queue object. 
 * @param   index       Item index.  
 * @return              Calculated pointer.
 */
kInlineFx(void*) kQueue_DataAt(kQueue queue, kSize index)
{
    kObj(kQueue, queue); 
    
    if ((obj->head + index) >= obj->capacity)
    {
        return kPointer_ItemOffset(obj->items, (kSSize)(obj->head + index - obj->capacity), obj->itemSize); 
    }
    else
    {
        return kPointer_ItemOffset(obj->items, (kSSize)(obj->head + index), obj->itemSize);
    }
}

/** 
 * Calculates an element address.
 *
 * This method is similar to the At method, but does not bounds-check the index argument. 
 * 
 * A debug assertion will be raised if the size of the specified item type is not equal to the 
 * size of the collection item type.
 * 
 * @relates                 kQueue
 * @param   kQueue_queue    Queue object. 
 * @param   kSize_index     Item index.  
 * @param   T               Item type identifier (e.g., k32s).
 * @return                  Calculated pointer.
 */
#define kQueue_DataAtT(kQueue_queue, kSize_index, T) \
    kCast(T*, xkQueue_DataAtT(kQueue_queue, kSize_index, sizeof(T)))

/** 
 * Returns a pointer to the specified item in the queue buffer. 
 *
 * A debug assertion will be raised if the index argument is greater than or equal to 
 * the current queue count (note: count, not capacity).  
 * 
 * @public              @memberof kQueue
 * @param   queue       Queue object. 
 * @param   index       Item index.  
 * @return              Pointer to queue element.
 */
kInlineFx(void*) kQueue_At(kQueue queue, kSize index)
{    
#   if !defined(K_FSS_912_DISABLE_BOUNDS_CHECK)
    {
        kAssert(index < kQueue_Count(queue));
    }
#   endif

    return kQueue_DataAt(queue, index);
}

/** 
 * Returns a strongly-typed pointer to the specified item in the queue buffer. 
 *
 * A debug assertion will be raised if the index argument is greater than or equal to 
 * the current queue count (note: count, not capacity), or if the size of the specified 
 * item type is not equal to the size of the collection item type.
 * 
 * @relates                 kQueue
 * @param   kQueue_queue    Queue object. 
 * @param   kSize_index     Item index.  
 * @param   T               Item type identifier (e.g., k32s).
 * @return                  Strongly-typed pointer to queue element.
 */
#define kQueue_AtT(kQueue_queue, kSize_index, T) \
    kCast(T*, xkQueue_AtT(kQueue_queue, kSize_index, sizeof(T)))

/** 
 * Returns the queue element type. 
 *
 * @public              @memberof kQueue
 * @param   queue       Queue object. 
 * @return              Item type. 
 */
kInlineFx(kType) kQueue_ItemType(kQueue queue)
{
    kObj(kQueue, queue); 

    return obj->itemType;
}

/** 
 * Returns the queue element size.
 *
 * @public              @memberof kQueue
 * @param   queue       Queue object. 
 * @return              Item size, in bytes. 
 */
kInlineFx(kSize) kQueue_ItemSize(kQueue queue)
{
    kObj(kQueue, queue); 

    return obj->itemSize;
}

/** 
 * Returns the current count of items in the queue. 
 *
 * @public              @memberof kQueue
 * @param   queue       Queue object. 
 * @return              Current count of items. 
 */
kInlineFx(kSize) kQueue_Count(kQueue queue)
{
    kObj(kQueue, queue); 

    return obj->count; 
}

/** 
 * Returns the number of elements for which space has been allocated.  
 *
 * @public              @memberof kQueue
 * @param   queue       Queue object. 
 * @return              Queue capacity. 
 */
kInlineFx(kSize) kQueue_Capacity(kQueue queue)
{
    kObj(kQueue, queue); 

    return obj->capacity;
} 

#endif

/** 
 * @file    kMsgQueue.h
 * @brief   Declares the kMsgQueue class. 
 *
 * @internal
 * Copyright (C) 2011-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_MSG_QUEUE_H
#define K_API_MSG_QUEUE_H

#include <kApi/kApiDef.h>

/**
 * @struct  kMsgQueueDropArgs
 * @ingroup kApi-Threads
 * @brief   Represents arguments passed in a kMsgQueue drop callback. 
 */
typedef struct kMsgQueueDropArgs
{
    void* item;       ///< Pointer to the item to be dropped. 
} kMsgQueueDropArgs; 

/**
 * @struct  kMsgQueuePurgeOption
 * @extends kValue
 * @ingroup kApi-Threads  
 * @brief   Represents a set of kMsgQueue purge options.
 */
typedef k32s kMsgQueuePurgeOption; 

/** @relates kMsgQueuePurgeOption @{ */
#define kMSG_QUEUE_PURGE_OPTION_NULL                        (0x0)       ///< No options.
#define kMSG_QUEUE_PURGE_OPTION_PRESERVE_CRITICAL           (0x1)       ///< Critical items should be preserved. 
#define kMSG_QUEUE_PURGE_OPTION_USE_HANDLER                 (0x2)       ///< Drop handler should be called to process dropped items.
#define kMSG_QUEUE_PURGE_OPTION_DISPOSE_ITEMS               (0x4)       ///< Reference items should be automatically disposed. 
#define kMSG_QUEUE_PURGE_OPTION_COUNT_DROPS                 (0x8)       ///< Dropped items should add to drop counter. 
/** @} */

/**
 * @struct  kMsgQueueItemOption
 * @extends kValue
 * @ingroup kApi-Threads  
 * @brief   Represents a set of options describing a kMsgQueue item.
 */
typedef k32s kMsgQueueItemOption; 

/** @relates kMsgQueueItemOption @{ */
#define kMSG_QUEUE_ITEM_OPTION_NULL                         (0x0)       ///< No options.
#define kMSG_QUEUE_ITEM_OPTION_CRITICAL                     (0x1)       ///< Item should not be subject to capacity constraints.
/** @} */

/** Defines the signature of a callback function to handle dropped items. */
typedef kStatus (kCall *kMsgQueueDropFx) (kPointer receiver, kMsgQueue queue, kMsgQueueDropArgs* args); 

#include <kApi/Threads/kMsgQueue.x.h>

/**
 * @class   kMsgQueue
 * @extends kObject
 * @ingroup kApi-Threads
 * @brief   Represents a synchronized FIFO queue with an optional maximum content size and/or item capacity. 
 * 
 * kMsgQueue is typically used to exchange data between threads. kMsgQueue supports multiple producers and 
 * multiple consumers, but is not strictly fair. Most kMsgQueue methods are thread-safe.
 * 
 * Limits can be placed on the maximum number of items (kMsgQueue_SetMaxCount) and/or maximum total
 * data capacity (kMsgQueue_SetMaxSize). Data capacity limits rely on the type system. For reference types, the 
 * size of each item is calculated by calling kObject_Size. For value types, the size of each item is determined 
 * by calling kType_Size on the 'itemType' constructor argument. 
 * 
 * kMsgQueue supports blocking with a timeout when removing an item from the queue. This enables consumers
 * to efficiently wait for an item to become available. 
 * 
 * When adding an item, if capacity limits are set and queue capacity is exceeded, the oldest item in 
 * the queue will be dropped. By default, kObject_Dispose will be called for dropped reference items. A custom 
 * drop handler can be installed using the kMsgQueue_SetDropHandler method. A count of dropped items can 
 * be determined using the kMsgQueue_DropCount method. 
 * 
 * With the kMsgQueue_AddEx method, items can be designated as "critical". Critical items are not 
 * subject to queue capacity constraints. This provides support for two different kinds of traffic within
 * a single queue: normal items that can be dropped, and critical items that cannot be dropped. This feature 
 * is most commonly used to embed special control messages within a stream of non-essential messages.  
 * 
 * @code {.c}
kStatus MsgQueueExample()
{
    kMsgQueue queue = kNULL; 
    kThread thread = kNULL; 
    kObject quitMsg = kNULL; 
    kSize i; 
    
    kTry
    {
         //create a queue that can store images
         kTest(kMsgQueue_Construct(&queue, kTypeOf(kImage), kNULL)); 

         //create a consumer thread and pass the queue as the thread entry argument
         kTest(kThread_Construct(&thread, kNULL)); 
         kTest(kThread_Start(thread, MsgProcessingThreadEntry, queue)); 

         //create some work for the thread
         for (i = 0; i < 5; ++i)
         {
             kImage image = kNULL; 
             
             kTest(kImage_Construct(&image, kTypeOf(k8u), 1024, 1024, kNULL)); 
             kTest(kMsgQueue_Add(queue, &image)); 
         }

         //request the thread to terminate
         kTest(kMsgQueue_Add(queue, &quitMsg)); 
    }
    kFinally
    {
         //destroying the thread will join automatically
         kObject_Destroy(thread); 

         //disposing the queue will also destroy any images remaining in the queue
         kObject_Dispose(queue);  

         kEndFinally(); 
    }
   
    return kOK; 
}

//process messages until a 'quit' (null) message is received
kStatus MsgProcessingThreadEntry(kMsgQueue queue)
{
    kImage image = kNULL; 

    while (kSuccess(kMsgQueue_Remove(queue, &image, kINFINITE)) && !kIsNull(image))
    {
        //process the image
        //...
        
        //clean up
        kObject_Destroy(image); 
    }

    return kOK; 
}

 * @endcode
 * 
 * kMsgQueue supports the kObject_Dispose and kObject_Size methods.
 * 
 */
//typedef kObject kMsgQueue;          // --forward-declared in kApiDef.x.h 

/** 
 * Constructs a kMsgQueue object. 
 *
 * @public              @memberof kMsgQueue
 * @param   queue       Receives a handle to the constructed object. 
 * @param   itemType    Type of queue element (must be a reference type). 
 * @param   allocator   Memory allocator (or kNULL for default). 
 * @return              Operation status. 
 */
kFx(kStatus) kMsgQueue_Construct(kMsgQueue* queue, kType itemType, kAlloc allocator); 

/** 
 * Sets the maximum amount of data retained by the queue. 
 *
 * By default, the maximum total amount of data is unlimited. 
 * 
 * This method is thread-safe. 
 *
 * @public                 @memberof kMsgQueue
 * @param   queue          Queue object. 
 * @param   size           Maximum total size of all data items in queue, in bytes.
 * @return                 Operation status. 
 */
kFx(kStatus) kMsgQueue_SetMaxSize(kMsgQueue queue, kSize size);

/** 
 * Sets the maximum count of items retained by the queue. 
 *
 * By default, the maximum count of items is unlimited. 
 * 
 * This method is thread-safe. 
 *
 * @public                 @memberof kMsgQueue
 * @param   queue          Queue object. 
 * @param   count          Maximum total items in queue.
 * @return                 Operation status. 
 */
kFx(kStatus) kMsgQueue_SetMaxCount(kMsgQueue queue, kSize count);

/** 
 * Reserves memory for the specified number of items.
 *
 * This method reserves internal memory for queue items, in order to reduce occurences of internal memory 
 * allocation as the size of the queue grows. It is unrelated to the queue capacity constraints specified 
 * with the kMsgQueue_SetMaxSize or kMsgQueue_SetMaxCount methods.
 * 
 * This method is thread-safe. 
 *
 * @public                 @memberof kMsgQueue
 * @param   queue          Queue object. 
 * @param   count          Count of items for which to reserve capacity.
 * @return                 Operation status. 
 */
kFx(kStatus) kMsgQueue_Reserve(kMsgQueue queue, kSize count);

/** 
 * Sets the callback used when dropping an item.
 *
 * If a handler is not set and the queue contains objects, dropped objects will be processed 
 * with kObject_Dispose.
 * 
 * This method is not thread-safe. 
 *
 * @public              @memberof kMsgQueue
 * @param   queue       Queue object. 
 * @param   onDrop      Callback function.
 * @param   receiver    Callback context. 
 * @return              Operation status. 
 */
kFx(kStatus) kMsgQueue_SetDropHandler(kMsgQueue queue, kMsgQueueDropFx onDrop, kPointer receiver);

/** 
 * Adds an item to the queue. 
 * 
 * If queue capacity is exceeded, the oldest non-critical item in the queue will be dropped to accomodate the 
 * new item. If a drop handler is installed, the handler will be called. If no drop handler is installed and the 
 * queue is an object container, then kObject_Dispose will be called to dispose the dropped item. 
 * 
 * If the operation status returned by this function indicates an error, it is the responsibility of the caller 
 * to dispose the item (if appropriate). Because items can be automatically dropped (and this is not considered 
 * an error), failure is unlikely unless the underlying memory allocator is exhausted. 
 *
 * This method is thread-safe. 
 *
 * @public              @memberof kMsgQueue
 * @param   queue       Queue object. 
 * @param   item        Item to be added. 
 * @return              Operation status. 
 */
kFx(kStatus) kMsgQueue_Add(kMsgQueue queue, void* item);

/** 
 * Adds an item to the queue. 
 * 
 * This method is a type-checked variant of kMsgQueue_Add. A debug assertion will be raised if the size 
 * of the specified item type is not equal to the size of the collection item type.
 * 
 * @relates                     kMsgQueue
 * @param   kMsgQueue_queue     Queue object. 
 * @param   TPtr_item           Strongly-typed pointer to item to be added. 
 * @return                      Operation status. 
 * @see                         kMsgQueue_Add
 */
#define kMsgQueue_AddT(kMsgQueue_queue, TPtr_item) \
    xkMsgQueue_AddT(kMsgQueue_queue, TPtr_item, sizeof(*(TPtr_item)))

/** 
 * Adds an item to the queue with the specified options. 
 * 
 * This method extends the behaviour of kMsgQueue_Add, with the addition of special option flags. 
 *
 * This method is thread-safe. 
 *
 * @public              @memberof kMsgQueue
 * @param   queue       Queue object. 
 * @param   item        Item to be added. 
 * @param   options     Bitset of item options.
 * @return              Operation status. 
 * @see                 kMsgQueue_Add
 */
kFx(kStatus) kMsgQueue_AddEx(kMsgQueue queue, void* item, kMsgQueueItemOption options);

/** 
 * Adds an item to the queue with the specified options. 
 * 
 * This method is a type-checked variant of kMsgQueue_AddEx. A debug assertion will be raised if the size 
 * of the specified item type is not equal to the size of the collection item type.
 * 
 * @relates                             kMsgQueue
 * @param   kMsgQueue_queue             Queue object. 
 * @param   TPtr_item                   Strongly-typed pointer to item to be added. 
 * @param   kMsgQueueItemOption_options Bitset of item options.
 * @return                              Operation status. 
 * @see                                 kMsgQueue_Add
 */
#define kMsgQueue_AddExT(kMsgQueue_queue, TPtr_item, kMsgQueueItemOption_options) \
    xkMsgQueue_AddExT(kMsgQueue_queue, TPtr_item, kMsgQueueItemOption_options, sizeof(*(TPtr_item)))

/** 
 * Removes an item from the queue. 
 * 
 * The timeout provided to this function can be used to efficiently wait for an item to 
 * become available. A zero timeout can be used to return immediately, while a kINFINITE timeout
 * can be used to wait indefinitely. 
 *
 * This method is thread-safe. 
 *
 * @public              @memberof kMsgQueue
 * @param   queue       Queue object. 
 * @param   item        Optionally receives removed item (if kNULL, item is not returned). 
 * @param   timeout     Timeout (microseconds).  
 * @return              Operation status. 
 */
kFx(kStatus) kMsgQueue_Remove(kMsgQueue queue, void* item, k64u timeout);

/** 
 * Removes an item from the queue. 
 * 
 * This method is a type-checked variant of kMsgQueue_Remove. A debug assertion will be raised if the size 
 * of the specified item type is not equal to the size of the collection item type.
 * 
 * @relates                     kMsgQueue
 * @param   kMsgQueue_queue     Queue object. 
 * @param   TPtr_item           Strongly-typed destionation pointer to receive removed item. 
 * @param   k64u_timeout        Timeout (microseconds).  
 * @return                      Operation status. 
 */
#define kMsgQueue_RemoveT(kMsgQueue_queue, TPtr_item, k64u_timeout) \
     xkMsgQueue_RemoveT(kMsgQueue_queue, TPtr_item, k64u_timeout, sizeof(*(TPtr_item)))

/** 
 * Removes all items from the queue.  
 *
 * This method does not call the drop handler when removing items and does not automatically 
 * dispose queue elements. Use kMsgQueue_Purge to both remove and dispose items. 
 *
 * This method is thread-safe. 
 *
 * @public              @memberof kMsgQueue
 * @param   queue       Queue object. 
 * @return              Operation status. 
 * @see                 kMsgQueue_Purge, kMsgQueue_PurgeEx
 */
kFx(kStatus) kMsgQueue_Clear(kMsgQueue queue);

/** 
 * Removes and disposes all items from the queue.  
 * 
 * This method does not call the drop handler when removing items. Reference (i.e., kObject-derived) items 
 * are automatically disposed. 
 *
 * This method is thread-safe. 
 *
 * @public              @memberof kMsgQueue
 * @param   queue       Queue object. 
 * @return              Operation status. 
 * @see                 kMsgQueue_Clear, kMsgQueue_PurgeEx
 */
kFx(kStatus) kMsgQueue_Purge(kMsgQueue queue);

/** 
 * Removes items from the queue using the specified options.
 * 
 * This method provides a flexible set of options for removing items from the queue. The options 
 * argument can be used to control the way in which item are disposed and counted. 
 *
 * This method is thread-safe. 
 *
 * @public              @memberof kMsgQueue
 * @param   queue       Queue object. 
 * @param   options     Bitset of options. 
 * @return              Operation status. 
 * @see                 kMsgQueue_Purge, kMsgQueue_Clear
 */
kFx(kStatus) kMsgQueue_PurgeEx(kMsgQueue queue, kMsgQueuePurgeOption options);

/** 
 * Reports the current count of queue items. 
 *
 * This method is thread-safe. 
 *
 * @public              @memberof kMsgQueue
 * @param   queue       Queue object. 
 * @return              Count of queue items. 
 */
kFx(kSize) kMsgQueue_Count(kMsgQueue queue);

/** 
 * Reports the maximum total data size of all items in the queue. 
 *
 * This method is thread-safe. 
 *
 * @public              @memberof kMsgQueue
 * @param   queue       Queue object. 
 * @return              Maximum total recursive data size, in bytes.
 */
kFx(kSize) kMsgQueue_MaxSize(kMsgQueue queue);

/** 
 * Reports the maximum count of items in the queue. 
 *
 * This method is thread-safe. 
 *
 * @public              @memberof kMsgQueue
 * @param   queue       Queue object. 
 * @return              Maximum count of items.
 */
kFx(kSize) kMsgQueue_MaxCount(kMsgQueue queue);

/** 
 * Reports the type of element stored in the queue. 
 *
 * This method is thread-safe. 
 *
 * @public              @memberof kMsgQueue
 * @param   queue       Queue object. 
 * @return              Capacity of queue (bytes). 
 */
kFx(kType) kMsgQueue_ItemType(kMsgQueue queue);

/**
 * Returns the queue element size.
 *
 * This method is thread-safe. 
 *
 * @public              @memberof kMsgQueue
 * @param   queue       Queue object.
 * @return              Item size, in bytes.
 */
kFx(kSize) kMsgQueue_ItemSize(kQueue queue);

/** 
 * Reports the current amount of data stored in the queue (in bytes).
 *
 * This method is thread-safe. 
 *
 * @public              @memberof kMsgQueue
 * @param   queue       Queue object. 
 * @return              Size of queue (bytes). 
 */
kFx(kSize) kMsgQueue_DataSize(kMsgQueue queue);

/** 
 * Reports the count of dropped items. 
 *
 * This method is thread-safe. 
 *
 * @public              @memberof kMsgQueue
 * @param   queue       Queue object. 
 * @return              Count of dropped items. 
 */
kFx(k64u) kMsgQueue_DropCount(kMsgQueue queue);

#endif

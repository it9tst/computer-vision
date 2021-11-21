// 
// KMsgQueue.h
// 
// Copyright (C) 2014-2021 by LMI Technologies Inc.
// Licensed under the MIT License.
// Redistributed files must retain the above copyright notice.
// 
#ifndef K_API_NET_MSG_QUEUE_H
#define K_API_NET_MSG_QUEUE_H

#include <kApi/Threads/kMsgQueue.h>
#include "kApiNet/KAlloc.h"

namespace Lmi3d
{
    namespace Zen 
    {
        namespace Threads
        {
            /// <summary>Represents arguments passed to a KMsgQueue drop handler.</summary>
            /// 
            public ref class KMsgQueueDropArgs : EventArgs
            {
            public:

                /// <summary>Gets the dropped item.</summary>
                /// 
                /// <typeparam name="T">Type of item to be accessed.</typeparam>
                /// <returns>Dropped item.</returns>
                generic <typename T>
                T GetItem()
                {
                    if (kType_IsValue(m_type))
                    {
                        T value;

                        KCheckArgs(sizeof(T) == kType_Size(m_type));

                        kItemCopy(&value, m_item, sizeof(T));

                        return value;
                    }
                    else
                    {
                        kObject object = *(kObject*)m_item; 

                        return KToObject<T>(object);
                    }
                }

            internal:
                
                KMsgQueueDropArgs(kType type, kPointer item)
                    : m_item(item), m_type(type)
                {}

            private:
                kType m_type;       //Item type.
                kPointer m_item;       //Pointer to the item to be dropped. 
            };

            /// <summary>Represents a synchronized FIFO queue with an optional maximum size or count capacity.  
            /// <para/> Requires manual disposal.</summary>
            public ref class KMsgQueue : public KObject
            {
                KDeclareClass(KMsgQueue, kMsgQueue)

            public:
                /// <summary>Initializes a new instance of the KMsgQueue class with the specified Zen object handle.</summary>           
                /// <param name="handle">Zen object handle.</param>
                KMsgQueue(IntPtr handle)
                    : KObject(handle, DefaultRefStyle)
                {}

                KMsgQueue(IntPtr handle, KRefStyle refStyle)
                    : KObject(handle, refStyle)
                {}

                /// <summary>Constructs a kMsgQueue object.</summary>
                /// 
                /// <param name="itemType">Type of list element (must be a reference type).</param>
                KMsgQueue(KType^ itemType)
                    : KObject(DefaultRefStyle)
                {
                    kMsgQueue handle = kNULL;

                    KCheck(kMsgQueue_Construct(&handle, KToHandle(itemType), kNULL));

                    Handle = handle;
                }

                /// <inheritdoc cref="KMsgQueue(KType^)" />
                /// <param name="allocator">Memory allocator.</param>
                KMsgQueue(KType^ itemType, KAlloc^ allocator)
                    : KObject(DefaultRefStyle)
                {
                    kMsgQueue handle = kNULL;

                    KCheck(kMsgQueue_Construct(&handle, KToHandle(itemType), KToHandle(allocator)));

                    Handle = handle;
                }

                KMsgQueue(KType^ itemType, KAlloc^ allocator, KRefStyle refStyle)
                    : KObject(refStyle)
                {
                    kMsgQueue handle = kNULL;

                    KCheck(kMsgQueue_Construct(&handle, KToHandle(itemType), KToHandle(allocator)));

                    Handle = handle;
                }

                /// <summary>Gets or sets the maximum amount of data (byte) retained by the queue.</summary>
                /// 
                /// <remarks>
                /// This property determines the maximum total recursive size of all data items in queue, in bytes.
                /// </remarks>
                property k64s MaxSize
                {
                    k64s get() { return (k64s)kMsgQueue_MaxSize(Handle);  }

                    void set(k64s value) { KCheck(kMsgQueue_SetMaxSize(Handle, (kSize) value)); }
                }

                /// <summary>Gets or sets the maximum count of items retained by the queue..</summary>
                property k64s MaxCount
                {
                    k64s get() { return (k64s)kMsgQueue_MaxCount(Handle);  }

                    void set(k64s value) { KCheck(kMsgQueue_SetMaxCount(Handle, (kSize) value)); }
                }

                /// <summary>Reserves capacity for the specified number of items.</summary>
                /// 
                /// <param name="count">Count of items for which to reserve capacity.</param>
                void Reserve(k64s count)
                {
                    KCheck(kMsgQueue_Reserve(Handle, (kSize)count)); 
                }

                /// <summary>Sets a handler for dropped items.</summary>
                /// 
                /// <remarks>
                /// <para>The registered handler must be unregistered when no longer needed, else the underlying message queue 
                /// will retain a reference to it, possibly resulting in a CLR object leak. The handler will be automatically 
                /// unregistered if KObject.Dispose is used to destroy the queue.</para>
                ///
                /// <para>If a handler is not set, and the queue contains objects, then dropped objects are passed
                /// to KObject.Dispose.</para>
                /// </remarks>
                /// 
                /// <param name="dropHandler">Item drop handler (or null to clear the drop handler).</param>
                void SetDropHandler(EventHandler<KMsgQueueDropArgs^>^ dropHandler)
                {
                    ClearDropHandler();

                    if (dropHandler)
                    {
                        KCallbackFx^ thunk = gcnew KCallbackFx(this, &KMsgQueue::OnDrop);
                        KCallbackState^ context = gcnew KCallbackState(thunk, dropHandler);

                        KCheck(kMsgQueue_SetDropHandler(Handle, (kMsgQueueDropFx)context->NativeFunction, context->NativeContext));
                    }
                }

                /// <summary>Adds an item to the queue.</summary>
                /// 
                /// <remarks>
                /// <para>If queue capacity is exceeded, the oldest item in the queue will be removed. If a drop handler is
                /// installed, the handler will be called; otherwise, if the queue is an object container, then
                /// KObject.Dispose will be used to dispose the item.</para>
                /// </remarks>
                /// 
                /// <typeparam name="T">Type of item to be added to the queue.</typeparam>
                /// <param name="item">Item to be added.</param>
                generic <typename T>
                void Add(T item)
                {
                    Add(item, Nullable<KRefStyle>());
                }

                generic <typename T>
                void Add(T item, Nullable<KRefStyle> refStyle)
                {
                    kType itemType = kMsgQueue_ItemType(Handle); 

                    if (kType_IsValue(itemType))
                    {
                        KCheckArgs(sizeof(T) == kType_Size(itemType)); 

                        KCheck(kMsgQueue_Add(Handle, &item));
                    }
                    else
                    {
                        kObject object = KToHandle(item);

                        KCheck(kMsgQueue_Add(Handle, &object));
                        KAdjustRef(object, kTRUE, refStyle);
                    }
                }
 
                /// <summary>Removes an item from the queue.</summary>
                /// 
                /// <typeparam name="T">Type of item to be removed from the queue.</typeparam>
                /// <param name="timeout">Timeout (microseconds).</param>
                /// <returns>The removed item.</returns>
                /// <exception cref="KException">Thrown if a timeout occurs.</exception>
                generic <typename T>
                T Remove(k64s timeout)
                {
                    return Remove<T>(timeout, Nullable<KRefStyle>());
                }

                generic <typename T>
                T Remove(k64s timeout, Nullable<KRefStyle> refStyle)
                {
                    kType itemType = kMsgQueue_ItemType(Handle);

                    if (kType_IsValue(itemType))
                    {
                        T item;

                        KCheckArgs(sizeof(T) == kType_Size(itemType)); 

                        KCheck(kMsgQueue_Remove(Handle, &item, (k64u)timeout));

                        return item;
                    }
                    else
                    {
                        kObject object = kNULL;

                        KCheck(kMsgQueue_Remove(Handle, &object, (k64u)timeout));

                        return KToObject<T>(object, refStyle);
                    }
                }

                /// <summary>Attempst to remove an item from the queue.</summary>
                /// 
                /// <typeparam name="T">Type of item to be removed from the queue.</typeparam>
                /// <param name="timeout">Timeout (microseconds).</param>
                /// <param name="removedItem">The removed item, if successful.</param>
                /// <returns>true if an item was successfully removed; otherwise false.</returns>
                generic <typename T>
                bool TryRemove([Out] T% removedItem, k64s timeout)
                {
                    kType itemType = kMsgQueue_ItemType(Handle);

                    if (kType_IsValue(itemType))
                    {
                        T item;

                        KCheckArgs(sizeof(T) == kType_Size(itemType)); 

                        if (kSuccess(kMsgQueue_Remove(Handle, &item, (k64u)timeout)))
                        {
                            removedItem = item; 
                            return true; 
                        }
                    }
                    else
                    {
                        kObject object = kNULL;

                        if (kSuccess(kMsgQueue_Remove(Handle, &object, (k64u)timeout)))
                        {
                            removedItem = KToObject<T>(object);
                            return true; 
                        }
                    }

                    return false;
                }


                /// <summary>Removes all items from the queue.</summary>
                /// 
                /// <remarks>
                /// This method does not call the drop handler when removing items.
                /// </remarks>
                void Clear()
                {
                    Clear(Nullable<KRefStyle>());
                }

                void Clear(Nullable<KRefStyle> refStyle)
                {
                    if (!kType_IsValue(kMsgQueue_ItemType(Handle))){
                        while (Count > 0)
                        {
                            kObject object = kNULL;
                            KCheck(kMsgQueue_Remove(Handle, &object, (k64u)1000 * 10));

                            KAdjustRef(object, kFALSE, refStyle);
                        }
                    }

                    //KCheck(kMsgQueue_Clear(Handle)); 
                }

                /// <summary>Gets the current count of queue items.</summary>
                property k64s Count
                {
                    k64s get() { return kMsgQueue_Count(Handle); }
                }

                /// <summary>Gets the type of element stored in the queue.</summary>
                property KType^ ItemType
                {
                    KType^ get() { return gcnew KType(kMsgQueue_ItemType(Handle));  }
                }

                /// <summary>Gets the current amount of data stored in the queue (in bytes).</summary>
                property k64s DataSize
                {
                    k64s get() { return kMsgQueue_DataSize(Handle); }
                }

                /// <summary>Gets the count of dropped items.</summary>
                property k64s DropCount
                {
                    k64s get() { return kMsgQueue_DropCount(Handle); }
                }

            protected:
                KMsgQueue() : KObject(DefaultRefStyle) {}

                //ensures that callbacks are unhooked
                virtual void OnDisposing() override
                {
                    ClearDropHandler();
                }

            private:
                void ClearDropHandler()
                {
                    kCallback nativeHandler = xkMsgQueue_DropHandler(Handle);

                    KCheck(kMsgQueue_SetDropHandler(Handle, kNULL, kNULL));

                    KCallbackState::Dispose(nativeHandler.receiver);
                }

                kStatus OnDrop(kPointer receiver, kPointer sender, kPointer args)
                {
                    kStatus status = kOK;

                    try
                    {
                        KCallbackState^ context = KCallbackState::FromNativeContext(receiver);
                        EventHandler<KMsgQueueDropArgs^>^ handler = (EventHandler<KMsgQueueDropArgs^>^) context->Handler;
                        kMsgQueueDropArgs* nativeArgs = (kMsgQueueDropArgs*)args;
                        KMsgQueue^ queue = (KMsgQueue^)context->Thunk->Target;
                        
                        handler(queue, gcnew KMsgQueueDropArgs(kMsgQueue_ItemType(sender), nativeArgs->item));
                    }
                    catch (KException^ e)
                    {
                        status = e->Status;
                    }
                    catch (...)
                    {
                        status = kERROR;
                    }

                    return status;
                }
            };

        }
    }
}

#endif

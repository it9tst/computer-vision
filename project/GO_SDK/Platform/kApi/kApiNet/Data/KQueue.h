//
// KQueue.h
// 
// Copyright (C) 2014-2021 by LMI Technologies Inc.
// Licensed under the MIT License.
// Redistributed files must retain the above copyright notice.
// 
#ifndef K_API_NET_QUEUE_H
#define K_API_NET_QUEUE_H

#include <kApi/Data/kQueue.h>
#include "kApiNet/KAlloc.h"

namespace Lmi3d
{
    namespace Zen 
    {       
        namespace Data
        {
            /// <summary>Represents a FIFO queue implemented with a dynamic array.</summary>
            ///
            /// <remarks>
            /// <para>The KQueue class represents a dynamic, array-based queue of objects or values. The KQueue constructor
            /// accepts a KType value that determines the type of items that will be stored in the queue. The queue will
            /// automatically grow as new items are added.</para>
            ///
            /// <para>KQueue supports the KObject.Clone and KObject.Size methods.</para>
            ///
            /// <para>KQueue supports the kdat5 and kdat6 serialization protocols.</para>
            ///
            /// <para>Default KRefStyle: Auto</para>
            /// </remarks>
            ///
            /// <example>This example demonstrates the creation and use of a KQueue instance with item type K32s.
            /// <code language="C#" source="examples/Data/KQueue_Add_Get_Values.cs" />
            /// </example>
            public ref class KQueue : public KObject
            {
                KDeclareAutoClass(KQueue, kQueue)

            public:
                /// <summary>Initializes a new instance of the KQueue class with the specified Zen object handle.</summary>           
                /// <param name="handle">Zen object handle.</param>
                KQueue(IntPtr handle)
                    : KObject(handle, DefaultRefStyle)
                {}

                /// <inheritdoc cref="KQueue(IntPtr)" />
                ///
                /// <param name="refStyle">RefStyle for this object.</param>
                KQueue(IntPtr handle, KRefStyle refStyle)
                    : KObject(handle, refStyle)
                {}

                /// <summary>Initializes a new instance of the KQueue class without specifying an item type.</summary>      
                KQueue()
                    : KObject(DefaultRefStyle)
                {
                    kQueue handle = kNULL;

                    KCheck(kQueue_Construct(&handle, kTypeOf(kVoid), 0, kNULL));

                    Handle = handle;
                }

                /// <summary>Initializes a new instance of the KQueue class with the specified item type.</summary>
                /// <param name="itemType">Type of queue element.</param>
                KQueue(KType^ itemType)
                    : KObject(DefaultRefStyle)
                {
                    kQueue handle = kNULL;

                    KCheck(kQueue_Construct(&handle, KToHandle(itemType), 0, kNULL));

                    Handle = handle;
                }

                /// <summary>Initializes a new instance of the KQueue class with the specified item type and initial cacpacity.</summary>      
                /// 
                /// <param name="itemType">Type of queue element.</param>
                /// <param name="initialCapacity">Capacity initially reserved for queue items.</param>
                KQueue(KType^ itemType, k64s initialCapacity)
                    : KObject(DefaultRefStyle)
                {
                    kQueue handle = kNULL;

                    KCheck(kQueue_Construct(&handle, KToHandle(itemType), (kSize)initialCapacity, kNULL));

                    Handle = handle;
                }

                /// <inheritdoc cref="KQueue(KType^, k64s)" />
                /// <param name="allocator"> Memory allocator.</param>
                KQueue(KType^ itemType, k64s initialCapacity, KAlloc^ allocator)
                    : KObject(DefaultRefStyle)
                {
                    kQueue handle = kNULL;

                    KCheck(kQueue_Construct(&handle, KToHandle(itemType), (kSize)initialCapacity, KToHandle(allocator)));

                    Handle = handle;
                }

                /// <inheritdoc cref="KQueue(KType^, k64s, KAlloc^)" />
                ///
                /// <param name="refStyle">RefStyle for this object.</param>
                KQueue(KType^ itemType, k64s initialCapacity, KAlloc^ allocator, KRefStyle refStyle)
                    : KObject(refStyle)
                {
                    kQueue handle = kNULL;

                    KCheck(kQueue_Construct(&handle, KToHandle(itemType), (kSize)initialCapacity, KToHandle(allocator)));

                    Handle = handle;
                }

                /// <summary>Reallocates the queue item buffer.</summary> 
                /// 
                /// <param name="itemType">Type of queue element.</param>
                /// <param name="initialCapacity">Capacity initially reserved for queue items.</param>
                void Allocate(KType^ itemType, k64s initialCapacity)
                {
                    Allocate(itemType, initialCapacity, Nullable<KRefStyle>());
                }

                /// <inheritdoc cref="Allocate(KType^, k64s)" />
                ///
                /// <param name="refStyle">RefStyle for this object.</param>
                void Allocate(KType^ itemType, k64s initialCapacity, Nullable<KRefStyle> refStyle)
                {
                    RemoveRefRange(0, Count, refStyle);

                    KCheck(kQueue_Allocate(Handle, KToHandle(itemType), (kSize)initialCapacity)); 
                }

                /// <summary>Performs a shallow copy of the source queue.</summary> 
                /// 
                /// <remarks>Source items are copied by value; if the source queue contains objects, the object
                /// handles are copied but the objects are not cloned.</remarks>
                /// 
                /// <param name="source">Source queue to be copied.</param>
                void Assign(KQueue^ source)
                {
                    Assign(source, Nullable<KRefStyle>());
                }

                /// <inheritdoc cref="Assign(KQueue^)" />
                ///
                /// <param name="refStyle">RefStyle for this object.</param>
                void Assign(KQueue^ source, Nullable<KRefStyle> refStyle)
                {
                    RemoveRefRange(0, Count, refStyle);

                    KCheck(kQueue_Assign(Handle, KToHandle(source)));

                    AddRefRange(0, Count, refStyle);
                }

                /// <summary>Sets the count of queue items to zero.</summary> 
                void Clear()
                {
                    Clear(Nullable<KRefStyle>());
                }

                /// <inheritdoc cref="Clear()" />
                ///
                /// <param name="refStyle">RefStyle for this object.</param>
                void Clear(Nullable<KRefStyle> refStyle)
                {
                    RemoveRefRange(0, Count, refStyle);

                    KCheck(kQueue_Clear(Handle)); 
                }

                /// <summary>Ensures that capacity is reserved for at least the specified number of queue items.</summary> 
                /// 
                /// <remarks>Existing queue items are preserved.</remarks>
                /// 
                /// <param name="capacity">Queue capacity, in items.</param>
                void Reserve(k64s capacity)
                {
                    KCheck(kQueue_Reserve(Handle, (kSize)capacity)); 
                }

                /// <summary>Adds the specified item to the end of the queue.</summary> 
                /// 
                /// <typeparam name="T">Type of item to be added to the queue.</typeparam>
                /// <param name="item">Item to be copied into the queue.</param>
                generic <typename T>
                void Add(T item)
                {
                    Add(item, Nullable<KRefStyle>());
                }
                
                /// <summary>Adds the specified item to the end of the queue.</summary> 
                /// 
                /// <typeparam name="T">Type of item to be added to the queue.</typeparam>
                /// <param name="item">Item to be copied into the queue.</param>
                /// <param name="refStyle">RefStyle for this object.</param>
                generic <typename T>
                void Add(T item, Nullable<KRefStyle> refStyle)
                {
                    if (kType_IsValue(kQueue_ItemType(Handle)))
                    {
                        KCheckArgs(sizeof(T) == kQueue_ItemSize(Handle));

                        KCheck(kQueue_Add(Handle, &item));
                    }
                    else
                    {
                        kObject object = KToHandle(item);

                        KCheck(kQueue_Add(Handle, &object));

                        AddRefRange((Count - 1), 1, refStyle);
                    }
                }

                /// <summary>Removes the item at the head of the queue.</summary> 
                /// 
                /// <typeparam name="T">Type of item to be removed from the queue.</typeparam>
                /// <returns>The removed item.</returns>
                generic <typename T>
                T Remove()
                {
                    return Remove<T>(Nullable<KRefStyle>());
                }
                
                /// <inheritdoc cref="Remove()" />
                ///
                /// <param name="refStyle">RefStyle for this object.</param>
                generic <typename T>
                T Remove(Nullable<KRefStyle> refStyle)
                {
                    if (kType_IsValue(kQueue_ItemType(Handle)))
                    {
                        T item;

                        KCheckArgs(sizeof(T) == kQueue_ItemSize(Handle));

                        KCheck(kQueue_Remove(Handle, &item));

                        return item;
                    }
                    else
                    {
                        kObject object = kNULL;

                        KCheck(kQueue_Remove(Handle, &object));

                        return KToObject<T>(object, refStyle);
                    }
                }

                /// <summary>Sets the value of an item.</summary> 
                /// 
                /// <typeparam name="T">Type of item to be modified.</typeparam>
                /// <param name="index">Item index.</param>
                /// <param name="item">Item to be copied into the queue.</param>
                generic <typename T>
                void Set(k64s index, T item)
                {
                    Set(index, item, Nullable<KRefStyle>());
                }

                /// <summary>Sets the value of an item.</summary> 
                /// 
                /// <typeparam name="T">Type of item to be modified.</typeparam>
                /// <param name="index">Item index.</param>
                /// <param name="item">Item to be copied into the queue.</param>
                /// <param name="refStyle">RefStyle for this object.</param>
                generic <typename T>
                void Set(k64s index, T item, Nullable<KRefStyle> refStyle)
                {
                    if (kType_IsValue(kQueue_ItemType(Handle)))
                    {
                        KCheckArgs(sizeof(T) == kQueue_ItemSize(Handle));

                        KCheck(kQueue_SetItem(Handle, (kSize)index, &item));
                    }
                    else
                    {
                        kObject object = KToHandle(item);

                        RemoveRefRange(index, 1, refStyle);

                        KCheck(kQueue_SetItem(Handle, (kSize)index, &object));

                        AddRefRange(index, 1, refStyle);
                    }
                }

                /// <summary>Gets the item at the specified index.</summary>
                /// 
                /// <typeparam name="T">Type of item to be accessed.</typeparam>
                /// <param name="index">Item index.</param>
                /// <returns>Queue item.</returns>
                generic <typename T>
                T Get(k64s index)
                {
                    return Get<T>(index, Nullable<KRefStyle>());
                }

                /// <inheritdoc cref="Get(k64s)" />
                ///
                /// <param name="refStyle">RefStyle for this object.</param>
                generic <typename T>
                T Get(k64s index, Nullable<KRefStyle> refStyle)
                {
                    KCheckArgs((kSize)index < kQueue_Count(Handle));

                    if (kType_IsValue(kQueue_ItemType(Handle)))
                    {
                        void* item = kQueue_At(Handle, (kSize)index);
                        T value;

                        KCheckArgs(sizeof(T) == kQueue_ItemSize(Handle));

                        kItemCopy(&value, item, sizeof(T));

                        return value;
                    }
                    else
                    {
                        kObject object = *(kObject*)kQueue_At(Handle, (kSize)index);

                        AddRefRange(index, 1, refStyle);

                        return KToObject<T>(object, refStyle);
                    }
                }

                /// <summary>Gets a pointer to the specified item in the queue buffer.</summary> 
                /// 
                /// <param name="index">Item index.</param>
                /// <returns>Pointer to queue element.</returns>
                IntPtr GetDataAt(k64s index)
                {
                    return IntPtr(kQueue_At(Handle, (kSize)index)); 
                }

                /// <summary>Gets the queue element type.</summary> 
                property KType^ ItemType
                {
                    KType^ get() { return gcnew KType(kQueue_ItemType(Handle));  }
                }

                /// <summary>Gets the queue element size.</summary> 
                property k64s ItemSize
                {
                    k64s get() { return (k64s)kQueue_ItemSize(Handle); }
                }

                /// <summary>Gets the current count of items in the queue.</summary> 
                property k64s Count
                {
                    k64s get() { return (k64s)kQueue_Count(Handle); }
                }

                /// <summary>Gets the number of elements for which space has been allocated.</summary> 
                property k64s Capacity
                {
                    k64s get() { return (k64s)kQueue_Capacity(Handle); }
                }

            private:
                void AddRefRange(k64s start, k64s count, Nullable<KRefStyle> refStyle)
                {
                    AdjustRefRange(start, count, kTRUE, refStyle); 
                }

                void RemoveRefRange(k64s start, k64s count, Nullable<KRefStyle> refStyle)
                { 
                    AdjustRefRange(start, count, kFALSE, refStyle);
                }

                /// <summary>Adjusts the ref count of a range of objects, based on the refStyle parameter or the objects' default ref style.</summary>
                void AdjustRefRange(k64s start, k64s count, kBool add, Nullable<KRefStyle> refStyle)
                {
                    if (!kType_IsValue(kQueue_ItemType(Handle)))
                    {
                        KCheckArgs(start + count <= Count);

                        for (k64s i = start; i < (start + count); ++i)
                        {
                            kObject object = kNULL;

                            KCheck(kQueue_Item(Handle, (kSize)i, &object));

                            KAdjustRef(object, add, refStyle);
                        }
                    }
                }
            };
        }
    }
}

#endif

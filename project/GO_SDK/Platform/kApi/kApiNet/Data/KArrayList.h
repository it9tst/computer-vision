//
// KArrayList.h
// 
// Copyright (C) 2014-2021 by LMI Technologies Inc.
// Licensed under the MIT License.
// Redistributed files must retain the above copyright notice.
// 
#ifndef K_API_NET_ARRAYLIST_H
#define K_API_NET_ARRAYLIST_H

#include <kApi/Data/kArrayList.h>
#include "kApiNet/KAlloc.h"

namespace Lmi3d
{
    namespace Zen 
    {       
        namespace Data
        {
            /// <summary>Represents a list implemented with a dynamic array.</summary>
            ///
            /// <remarks>
            /// <para>The KArrayList class represents a dynamic, array-based list of objects or values. The KArrayList constructor 
            /// accepts a KType value that determines the type of items that will be stored in the list. The list will 
            /// automatically grow as new items are added.</para>
            /// 
            /// <para>KArrayList supports the KObject.Clone and KObject.Size methods.</para>
            /// 
            /// <para>KArrayList supports the kdat5 and kdat6 serialization protocols.</para>
            ///
            /// <para>Default KRefStyle: Auto</para>
            /// </remarks>
            /// 
            /// <example>This example demonstrates the creation and use of a KArrayList instance with item type K32s.
            /// <code language="C#" source="examples/Data/KArrayList_Add_At_Value.cs" />
            /// </example>
            public ref class KArrayList : public KObject
            {
                KDeclareAutoClass(KArrayList, kArrayList)

            public:
                /// <summary>Initializes a new instance of the KArrayList class with the specified Zen object handle.</summary>           
                /// <param name="handle">Zen object handle.</param>
                KArrayList(IntPtr handle)
                    : KObject(handle, DefaultRefStyle)
                {}

                /// <inheritdoc cref="KArrayList(IntPtr)" />
                ///
                /// <param name="refStyle">Ref style.</param>
                KArrayList(IntPtr handle, KRefStyle refStyle)
                    : KObject(handle, refStyle)
                {}

                /// <summary>Initializes a new instance of the KArrayList class without specifying an item type.</summary>
                KArrayList()
                    : KObject(DefaultRefStyle)
                {
                    kArrayList handle = kNULL;

                    KCheck(kArrayList_Construct(&handle, kTypeOf(kVoid), 0, kNULL));

                    Handle = handle;
                }

                /// <summary>Initializes a new instance of the KArrayList class with the specified item type.</summary>
                /// <param name="itemType">Type of list element.</param>
                KArrayList(KType^ itemType)
                    : KObject(DefaultRefStyle)
                {
                    kArrayList handle = kNULL;

                    KCheck(kArrayList_Construct(&handle, KToHandle(itemType), 0, kNULL));

                    Handle = handle;
                }

                /// <summary>Initializes a new instance of the KArrayList class with the specified initial capacity.</summary>
                /// <param name="itemType">Type of list element.</param>
                /// <param name="initialCapacity">Capacity initially reserved for list items.</param>
                KArrayList(KType^ itemType, k64s initialCapacity)
                    : KObject(DefaultRefStyle)
                {
                    kArrayList handle = kNULL;

                    KCheck(kArrayList_Construct(&handle, KToHandle(itemType), (kSize)initialCapacity, kNULL));

                    Handle = handle;
                }

                /// <inheritdoc cref="KArrayList(KType^, k64s)" />
                /// <param name="allocator">Memory allocator.</param>
                KArrayList(KType^ itemType, k64s initialCapacity, KAlloc^ allocator)
                    : KObject(DefaultRefStyle)
                {
                    kArrayList handle = kNULL;

                    KCheck(kArrayList_Construct(&handle, KToHandle(itemType), (kSize)initialCapacity, KToHandle(allocator)));

                    Handle = handle;
                }

                /// <inheritdoc cref="KArrayList(KType^, k64s, KAlloc^)" />
                ///
                /// <param name="refStyle">RefStyle for this object.</param>
                KArrayList(KType^ itemType, k64s initialCapacity, KAlloc^ allocator, KRefStyle refStyle)
                    : KObject(refStyle)
                {
                    kArrayList handle = kNULL;

                    KCheck(kArrayList_Construct(&handle, KToHandle(itemType), (kSize)initialCapacity, KToHandle(allocator)));

                    Handle = handle;
                }

                /// <summary>Reallocates the list item buffer.</summary>
                /// 
                /// <param name="itemType">Type of list element.</param>
                /// <param name="initialCapacity">Capacity initially reserved for list items.</param>
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

                    KCheck(kArrayList_Allocate(Handle, KToHandle(itemType), (kSize)initialCapacity));
                }

                /// <summary>Attaches the list object to an external buffer.</summary>
                /// 
                /// <remarks>The list count is set to the same value as the list capacity argument.</remarks>
                /// 
                /// <param name="items">Item buffer.</param>
                /// <param name="itemType">Type of list element.</param>
                /// <param name="capacity">List capacity.</param>
                void Attach(IntPtr items, KType^ itemType, k64s capacity)
                {
                    Attach(items, itemType, capacity, Nullable<KRefStyle>()); 
                }

                /// <inheritdoc cref="Attach(IntPtr, KType^, k64s)" />
                ///
                /// <param name="refStyle">RefStyle for this object.</param>
                void Attach(IntPtr items, KType^ itemType, k64s capacity, Nullable<KRefStyle> refStyle)
                {
                    RemoveRefRange(0, Count, refStyle);

                    KCheck(kArrayList_Attach(Handle, items.ToPointer(), KToHandle(itemType), (kSize)capacity));
                }

                /// <summary>Copies the specified items into the list, replacing existing contents.</summary>
                /// 
                /// <remarks>The list count is set to the value of the count argument.</remarks>
                /// 
                /// <param name="items">Item buffer.</param>
                /// <param name="itemType">Type of list element.</param>
                /// <param name="count">Count of list items.</param>
                void Import(IntPtr items, KType^ itemType, k64s count)
                {
                    Import(items, itemType, count, Nullable<KRefStyle>());
                }

                /// <inheritdoc cref="Import(IntPtr, KType^, k64s)" />
                ///
                /// <param name="refStyle">RefStyle for this object.</param>
                void Import(IntPtr items, KType^ itemType, k64s count, Nullable<KRefStyle> refStyle)
                {
                    RemoveRefRange(0, Count, refStyle);

                    KCheck(kArrayList_Import(Handle, items.ToPointer(), KToHandle(itemType), (kSize)count));

                    AddRefRange(0, count, refStyle); 
                }

                /// <summary>Appends the specified items to the list.</summary>
                /// 
                /// <param name="items">Item buffer.</param>
                /// <param name="count">Count of list items.</param>
                void Append(IntPtr items, k64s count)
                {
                    Append(items, count, Nullable<KRefStyle>()); 
                }

                /// <inheritdoc cref="Append(IntPtr, k64s)" />
                ///
                /// <param name="refStyle">RefStyle for this object.</param>
                void Append(IntPtr items, k64s count, Nullable<KRefStyle> refStyle)
                {
                    KCheck(kArrayList_Append(Handle, items.ToPointer(), (kSize)count));

                    AddRefRange(Count-count, count, refStyle);
                }

                /// <summary>Performs a shallow copy of the source list.</summary>
                /// 
                /// <remarks>Source items are copied by value; if the source list contains objects, the object
                /// handles are copied but the objects are not cloned.</remarks>
                /// 
                /// <param name="source">Source list to be copied.</param>
                void Assign(KArrayList^ source)
                {
                    Assign(source, Nullable<KRefStyle>()); 
                }

                /// <inheritdoc cref="Assign(KArrayList^)" />
                ///
                /// <param name="refStyle">RefStyle for this object.</param>
                void Assign(KArrayList^ source, Nullable<KRefStyle> refStyle)
                {
                    RemoveRefRange(0, Count, refStyle);

                    KCheck(kArrayList_Assign(Handle, KToHandle(source)));

                    AddRefRange(0, Count, refStyle);
                }

                /// <summary>Sets the count of list items to zero.</summary>
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

                    KCheck(kArrayList_Clear(Handle));
                }

                /// <summary>Sets the memory for all list elements to zero.</summary>
                void Zero()
                {
                    Zero(Nullable<KRefStyle>()); 
                }

                /// <inheritdoc cref="Zero()" />
                ///
                /// <param name="refStyle">RefStyle for this object.</param>
                void Zero(Nullable<KRefStyle> refStyle)
                {
                    RemoveRefRange(0, Count, refStyle);

                    KCheck(kArrayList_Zero(Handle)); 
                }

                /// <summary>Adds the specified item to the end of the list.</summary>
                /// 
                /// <typeparam name="T">Type of item to be added to the list.</typeparam>
                /// <param name="item">Item to be copied into the list.</param>
                generic <typename T>
                void Add(T item)
                {
                    Add(item, Nullable<KRefStyle>()); 
                }

                /// <summary>Adds the specified item to the end of the list.</summary>
                /// 
                /// <typeparam name="T">Type of item to be added to the list.</typeparam>
                /// <param name="item">Item to be copied into the list.</param>
                /// <param name="refStyle">RefStyle for this object.</param>
                generic <typename T>
                void Add(T item, Nullable<KRefStyle> refStyle)
                {
                    if (kType_IsValue(kArrayList_ItemType(Handle)))
                    {
                        KCheckArgs(sizeof(T) == kArrayList_ItemSize(Handle));

                        KCheck(kArrayList_Add(Handle, &item));
                    }
                    else
                    {
                        kObject object = KToHandle(item);
  
                        KCheck(kArrayList_Add(Handle, &object));

                        AddRefRange(Count - 1, 1, refStyle); 
                    }
                }

                /// <summary>Inserts an item into the list at the specified position.</summary>
                /// 
                /// <remarks>Increases list capacity, if necessary.</remarks>
                /// 
                /// <typeparam name="T">Type of item to be inserted into the list.</typeparam>
                /// <param name="before">Item will be inserted before the item at this index.</param>
                /// <param name="item">Item to be copied into the list.</param>
                generic <typename T>
                void Insert(k64s before, T item)
                {
                    Insert(before, item, Nullable<KRefStyle>());
                }

                /// <summary>Inserts an item into the list at the specified position.</summary>
                /// 
                /// <remarks>Increases list capacity, if necessary.</remarks>
                /// 
                /// <typeparam name="T">Type of item to be inserted into the list.</typeparam>
                /// <param name="before">Item will be inserted before the item at this index.</param>
                /// <param name="item">Item to be copied into the list.</param>
                generic <typename T>
                void Insert(k64s before, T item, Nullable<KRefStyle> refStyle)
                {
                    if (kType_IsValue(kArrayList_ItemType(Handle)))
                    {
                        KCheckArgs(sizeof(T) == kArrayList_ItemSize(Handle));

                        KCheck(kArrayList_Insert(Handle, (kSize)before, &item));
                    }
                    else
                    {
                        kObject object = KToHandle(item);

                        KCheck(kArrayList_Insert(Handle, (kSize)before, &object));

                        AddRefRange(before, 1, refStyle);

                    }
                }

                /// <summary>Removes an item from the list at the specified index.</summary>
                /// 
                /// <typeparam name="T">Type of item to be removed from the list.</typeparam>
                /// <param name="index">Item at this index will be removed from the list.</param>
                /// <returns>Item removed from list.</returns> 
                generic <typename T>
                T Remove(k64s index)
                {
                    return Remove<T>(index, Nullable<KRefStyle>());
                }

                /// <inheritdoc cref="Remove(k64s)" />
                ///
                /// <param name="refStyle">RefStyle for this object.</param>
                generic <typename T>
                T Remove(k64s index, Nullable<KRefStyle> refStyle)
                {
                    if (kType_IsValue(kArrayList_ItemType(Handle)))
                    {
                        T item;

                        KCheckArgs(sizeof(T) == kArrayList_ItemSize(Handle));

                        KCheck(kArrayList_Remove(Handle, (kSize)index, &item));

                        return item;
                    }
                    else
                    {
                        kObject object = kNULL;

                        KCheck(kArrayList_Remove(Handle, (kSize)index, &object));
                        
                        return KToObject<T>(object, refStyle);                        
                    }
                }

                /// <summary>Sets the value of an item.</summary>
                /// 
                /// <typeparam name="T">Type of item to be modified.</typeparam>
                /// <param name="index">Item index.</param>
                /// <param name="item">Item to be copied into the list.</param>
                generic <typename T>
                void Set(k64s index, T item)
                {
                    Set(index, item, Nullable<KRefStyle>()); 
                }

                /// <summary>Sets the value of an item.</summary>
                /// 
                /// <typeparam name="T">Type of item to be modified.</typeparam>
                /// <param name="index">Item index.</param>
                /// <param name="item">Item to be copied into the list.</param>
                /// <param name="refStyle">RefStyle for this object.</param>
                generic <typename T>
                void Set(k64s index, T item, Nullable<KRefStyle> refStyle)
                {
                    if (kType_IsValue(kArrayList_ItemType(Handle)))
                    {
                        KCheckArgs(sizeof(T) == kArrayList_ItemSize(Handle));

                        KCheck(kArrayList_SetItem(Handle, (kSize)index, &item));
                    }
                    else
                    {
                        kObject object = KToHandle(item);

                        RemoveRefRange(index, 1, refStyle); 

                        KCheck(kArrayList_SetItem(Handle, (kSize)index, &object));

                        AddRefRange(index, 1, refStyle);
                    }
                }

                /// <summary>Gets the item at the specified index.</summary>
                /// 
                /// <typeparam name="T">Type of item to be accessed.</typeparam>
                /// <param name="index">Item index.</param>
                /// <returns>List item.</returns>
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
                    KCheckArgs((kSize)index < kArrayList_Count(Handle));

                    if (kType_IsValue(kArrayList_ItemType(Handle)))
                    {
                        void* item = kArrayList_At(Handle, (kSize)index);
                        T value;

                        KCheckArgs(sizeof(T) == kArrayList_ItemSize(Handle));

                        kItemCopy(&value, item, sizeof(T));

                        return value;
                    }
                    else
                    {
                        kObject object = *(kObject*)kArrayList_At(Handle, (kSize)index);

                        AddRefRange(index, 1, refStyle);

                        return KToObject<T>(object, refStyle);
                    }
                }

                /// <summary>Sets the current count of list items to the specified value.</summary>
                /// 
                /// <remarks>Increases list capacity if necessary; existing list items are preserved.</remarks>
                /// 
                /// <param name="count">List size, in items.</param>
                void Resize(k64s count)
                {
                    Resize(count, Nullable<KRefStyle>()); 
                }

                /// <inheritdoc cref="Resize(k64s)" />
                ///
                /// <param name="refStyle">RefStyle for this object.</param>
                void Resize(k64s count, Nullable<KRefStyle> refStyle)
                {
                    if (count < Count)
                    {
                        RemoveRefRange(Count - count, count, refStyle); 
                    }

                    KCheck(kArrayList_Resize(Handle, (kSize)count));
                }

                /// <summary>Ensures that capacity is reserved for at least the specified number of list items.</summary>
                /// 
                /// <remarks>Existing list items are preserved.</remarks>
                /// 
                /// <param name="capacity">List capacity, in items.</param>
                void Reserve(k64s capacity)
                {
                    KCheck(kArrayList_Reserve(Handle, (kSize)capacity)); 
                }

                /// <summary>Gets a pointer to the list item buffer.</summary>
                property IntPtr Data
                {
                    IntPtr get() { return IntPtr(kArrayList_Data(Handle)); }
                }

                /// <summary>Gets the total size of list data (Count x ItemSize), in bytes.</summary>
                property k64s DataSize
                {
                    k64s get() { return kArrayList_DataSize(Handle); }
                }

                /// <summary>Gets the list element type.</summary>
                property KType^ ItemType
                {
                    KType^ get() { return gcnew KType(kArrayList_ItemType(Handle)); }
                }

                /// <summary>Gets the list element size, in bytes.</summary>
                property k64s ItemSize
                {
                    k64s get() { return (k64s)kArrayList_ItemSize(Handle); }
                }

                /// <summary>Gets the current count of items in the list.</summary>
                property k64s Count
                {
                    k64s get() { return (k64s)kArrayList_Count(Handle); }
                }

                /// <summary>Gets the number of elements for which space has been allocated.</summary>
                property k64s Capacity
                {
                    k64s get() { return (k64s)kArrayList_Capacity(Handle); }
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
                    if (!kType_IsValue(kArrayList_ItemType(Handle)))
                    {
                        KCheckArgs(start + count <= Count);

                        for (k64s i = start; i < (start + count); ++i)
                        { 
                            kObject object = kNULL;

                            KCheck(kArrayList_Item(Handle, (kSize)i, &object));

                            KAdjustRef(object, add, refStyle);
                        }
                    }
                }
            };
        }
    }
}

#endif

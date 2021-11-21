//
// KList.h
// 
// Copyright (C) 2014-2021 by LMI Technologies Inc.
// Licensed under the MIT License.
// Redistributed files must retain the above copyright notice.
// 
#ifndef K_API_NET_LIST_H
#define K_API_NET_LIST_H

#include <kApi/Data/kList.h>
#include "kApiNet/KAlloc.h"

namespace Lmi3d
{
    namespace Zen 
    {       
        namespace Data
        {
            ref class KList; 

            /// <summary>Represents a node within a KList object.</summary>           
            public ref struct KListItem
            {          
            public:

                /// <summary>Sets the content associated with a list item.</summary>
                /// 
                /// <typeparam name="T">Type of item to be modified.</typeparam>
                /// <param name="content">Content to be copied into the list item.</param>
                generic <typename T>
                void Set(T content)
                {
                    Set(content, Nullable<KRefStyle>());
                }

                /// <summary>Sets the content associated with a list item.</summary>
                /// 
                /// <typeparam name="T">Type of item to be modified.</typeparam>
                /// <param name="content">Content to be copied into the list item.</param>
                /// <param name="refStyle">RefStyle for this object.</param>
                generic <typename T>
                void Set(T content, Nullable<KRefStyle> refStyle)
                {
                    if (kType_IsValue(kList_ItemType(m_list)))
                    {
                        KCheckArgs(sizeof(T) == kType_Size(kList_ItemType(m_list)));

                        KCheck(kList_SetItem(m_list, m_item, &content));
                    }
                    else
                    {
                        kObject object = KToHandle(content);
                        kObject oldObject = *(kObject*)kList_At(m_list, m_item);

                        AdjustRef(oldObject, kFALSE, refStyle);
                        KCheck(kList_SetItem(m_list, m_item, &object));
                        AdjustRef(object, kTRUE, refStyle);
                    }
                }

                /// <summary>Gets the content associated with a list item.</summary>
                /// 
                /// <typeparam name="T">Type of item to be accessed.</typeparam>
                /// <returns>List item.</returns>
                generic <typename T>
                T Get()
                {
                    return Get<T>(Nullable<KRefStyle>());
                }

                /// <inheritdoc cref="Get()" />
                ///
                /// <param name="refStyle">RefStyle for this object.</param>
                generic <typename T>
                T Get(Nullable<KRefStyle> refStyle)
                {
                    if (kType_IsValue(kList_ItemType(m_list)))
                    {
                        void* content = kList_At(m_list, m_item);
                        T value;

                        KCheckArgs(sizeof(T) == kType_Size(kList_ItemType(m_list)));

                        kItemCopy(&value, content, sizeof(T));

                        return value;
                    }
                    else
                    {
                        kObject object = *(kObject*)kList_At(m_list, m_item);

                        AdjustRef(object, kTRUE, refStyle);
                        return KToObject<T>(object, refStyle);
                    }
                }

                /// <summary> Gets a reference to the next list item.</summary>
                property KListItem^ Next
                {
                    KListItem^ get() { return KListItem::ToObject(m_root, m_list, kList_Next(m_list, m_item)); }
                }

                /// <summary> Gets a reference to the previous list item.</summary>
                property KListItem^ Previous
                {
                    KListItem^ get() { return KListItem::ToObject(m_root, m_list, kList_Previous(m_list, m_item)); }
                }

                virtual bool Equals(Object^ other) override
                {
                    KListItem^ it = dynamic_cast<KListItem^>(other);

                    if (it != nullptr)
                    {
                        return (it->m_list == this->m_list) && (it->m_item == this->m_item);
                    }

                    return false;
                }

            internal:

                KListItem(Object^ root, kList list, kPointer item)
                    : m_root(root), m_list(list), m_item(item) {}

                static kPointer ToHandle(KListItem^ it)
                {
                    return (it == nullptr) ? kNULL : it->m_item;
                }

                static KListItem^ ToObject(Object^ root, kList list, kPointer it)
                {
                    return (it == kNULL) ? nullptr : gcnew KListItem(root, list, it);
                }

            private:
                Object^ m_root;               //prevents premature finalization of parent object
                kList m_list; 
                kListItem m_item; 

                void AdjustRef(kObject object, kBool add, Nullable<KRefStyle> refStyle)
                {
                    if (!kType_IsValue(kList_ItemType(m_list)))
                    {
                        KAdjustRef(object, add, refStyle);
                    }
                }
            };      

            /// <summary>Represents a doubly-linked list.</summary>
            /// 
            /// <remarks>
            /// <para>The KList class represents a doubly-linked list of objects or values. The KList constructor accepts a KType
            /// value that determines the type of items that will be stored in the list. The list will automatically
            /// grow as new items are added or inserted.</para>
            /// 
            /// <para>KList supports the KObject.Clone and KObject.Size methods.</para>
            /// 
            /// <para>KList supports the kdat6 serialization protocol.</para>
            ///
            /// <para>Default KRefStyle: Auto</para>
            /// </remarks>
            ////
            /// <example> This example illustrates adding a few values to a KList instance and then iterating over the resulting list.
            /// <code language="C#" source="examples/Data/KList_Add_Iterate_Value.cs" />
            /// </example>
            public ref class KList : public KObject
            {
                KDeclareAutoClass(KList, kList)

            public:
                /// <summary>Initializes a new instance of the KList class with the specified Zen object handle.</summary>           
                /// <param name="handle">Zen object handle.</param>
                KList(IntPtr handle)
                    : KObject(handle, DefaultRefStyle)
                {}

                /// <inheritdoc cref="KList(IntPtr)" />
                ///
                /// <param name="refStyle">RefStyle for this object.</param>
                KList(IntPtr handle, KRefStyle refStyle)
                    : KObject(handle, refStyle)
                {}

                /// <summary>Initializes a new instance of the KList class without specifying an item type.</summary>           
                KList()
                    : KObject(DefaultRefStyle)
                {
                    kList handle = kNULL;

                    KCheck(kList_Construct(&handle, kTypeOf(kVoid), 0, kNULL));

                    Handle = handle;
                }

                /// <summary>Initializes a new instance of the KList class with the specified item type and initial capacity.</summary>           
                /// 
                /// <param name="itemType">Type of list item.</param>
                /// <param name="initialCapacity">Capacity initially reserved for list items.</param>
                KList(KType^ itemType, k64s initialCapacity)
                    : KObject(DefaultRefStyle)
                {
                    kList handle = kNULL;

                    KCheck(kList_Construct(&handle, KToHandle(itemType), (kSize)initialCapacity, kNULL));

                    Handle = handle;
                }

                /// <inheritdoc cref="KList(KType^, k64s)" />
                /// <param name="allocator">Memory allocator.</param>
                KList(KType^ itemType, k64s initialCapacity, KAlloc^ allocator)
                    : KObject(DefaultRefStyle)
                {
                    kList handle = kNULL;

                    KCheck(kList_Construct(&handle, KToHandle(itemType), (kSize)initialCapacity, KToHandle(allocator)));

                    Handle = handle;
                }

                /// <inheritdoc cref="KList(KType^, k64s, KAlloc^)" />
                ///
                /// <param name="refStyle">RefStyle for this object.</param>
                KList(KType^ itemType, k64s initialCapacity, KAlloc^ allocator, KRefStyle refStyle)
                    : KObject(refStyle)
                {
                    kList handle = kNULL;

                    KCheck(kList_Construct(&handle, KToHandle(itemType), (kSize)initialCapacity, KToHandle(allocator)));

                    Handle = handle;
                }

                /// <summary>Reallocates the list.</summary>
                /// 
                /// <remarks>Existing items are discarded.</remarks>
                /// 
                /// <param name="itemType">Type of list item.</param>
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
                    RemoveRefAll(refStyle);

                    KCheck(kList_Allocate(Handle, KToHandle(itemType), (kSize)initialCapacity)); 
                }

                /// <summary>Performs a shallow copy of the source list.</summary>
                /// 
                /// <remarks>Source items are copied by value; if the source list contains objects, the object
                /// handles are copied but the objects are not cloned.</remarks>
                /// 
                /// <param name="source">List to be copied.</param>
                void Assign(KList^ source)
                {
                    Assign(source, Nullable<KRefStyle>());
                }

                /// <inheritdoc cref="Assign(KList^)" />
                ///
                /// <param name="refStyle">RefStyle for this object.</param>
                void Assign(KList^ source, Nullable<KRefStyle> refStyle)
                {
                    RemoveRefAll(refStyle);

                    KCheck(kList_Assign(Handle, KToHandle(source))); 

                    AddRefAll(refStyle);
                }

                /// <summary>Gets the item type.</summary>
                property KType^ ItemType
                {
                    KType^ get() { return gcnew KType(kList_ItemType(Handle)); }
                }

                /// <summary>Gets the count of list elements.</summary>
                property k64s Count
                {
                    k64s get() { return (k64s)kList_Count(Handle); }
                }

                /// <summary>Gets the number of elements for which space has been allocated.</summary>
                property k64s Capacity
                {
                    k64s get() { return (k64s)kList_Capacity(Handle); }
                }

                /// <summary>Adds a new item to the end of the list.</summary>
                /// 
                /// <remarks>Increases list capacity, if necessary.</remarks>
                /// 
                /// <typeparam name="T">Type of item to be added to the list.</typeparam>
                /// <param name="content">Item to be added to the list.</param>
                /// <returns>Iterator referring to newly-inserted item.</returns>
                generic <typename T>
                KListItem^ Add(T content)
                {
                    return Add(content, Nullable<KRefStyle>());
                }

                /// <summary>Adds a new item to the end of the list.</summary>
                /// 
                /// <remarks>Increases list capacity, if necessary.</remarks>
                /// 
                /// <typeparam name="T">Type of item to be added to the list.</typeparam>
                /// <param name="content">Item to be added to the list.</param>
                /// <param name="refStyle">RefStyle for this object.</param>
                /// <returns>Iterator referring to newly-inserted item.</returns>
                generic <typename T>
                KListItem^ Add(T content, Nullable<KRefStyle> refStyle)
                {
                    kListItem it = kNULL; 

                    if (kType_IsValue(kList_ItemType(Handle)))
                    {
                        KCheckArgs(sizeof(T) == kType_Size(kList_ItemType(Handle))); 

                        KCheck(kList_Add(Handle, &content, &it));
                    }
                    else
                    {
                        kObject object = KToHandle(content);

                        KCheck(kList_Add(Handle, &object, &it));

                        KAdjustRef(object, kTRUE, refStyle);
                    }

                    return gcnew KListItem(this, Handle, it); 
                }

                /// <summary>Inserts an item into the list before the specified list item.</summary>
                /// 
                /// <remarks>Increases list capacity, if necessary.</remarks>
                /// 
                /// <typeparam name="T">Type of item to be added to the list.</typeparam>
                /// <param name="before">Item will be inserted before this list node (if null, inserts at tail).</param>
                /// <param name="content">Item to be inserted into the list.</param>
                /// <returns>Iterator referring to newly-inserted item.</returns>
                generic <typename T>
                KListItem^ Insert(KListItem^ before, T content)
                {
                    return Insert(before, content, Nullable<KRefStyle>());
                }

                 /// <summary>Inserts an item into the list before the specified list item.</summary>
                /// 
                /// <remarks>Increases list capacity, if necessary.</remarks>
                /// 
                /// <typeparam name="T">Type of item to be added to the list.</typeparam>
                /// <param name="before">Item will be inserted before this list node (if null, inserts at tail).</param>
                /// <param name="content">Item to be inserted into the list.</param>
                /// <param name="refStyle">RefStyle for this object.</param>
                /// <returns>Iterator referring to newly-inserted item.</returns>
                generic <typename T>
                KListItem^ Insert(KListItem^ before, T content, Nullable<KRefStyle> refStyle)
                {
                    kListItem it = kNULL;

                    if (kType_IsValue(kList_ItemType(Handle)))
                    {
                        KCheckArgs(sizeof(T) == kType_Size(kList_ItemType(Handle)));

                        KCheck(kList_Insert(Handle, KListItem::ToHandle(before), &content, &it));
                    }
                    else
                    {
                        kObject object = KToHandle(content);

                        KCheck(kList_Insert(Handle, KListItem::ToHandle(before), &object, &it));

                        KAdjustRef(object, kTRUE, refStyle);
                    }

                    return gcnew KListItem(this, Handle, it);
                }

                /// <summary>Removes the specified item from the list.</summary>
                /// 
                /// <param name="item">Node to be removed from the list.</param>
                void Remove(KListItem^ item)
                {
                    Remove(item, Nullable<KRefStyle>());
                }

                /// <inheritdoc cref="Remove(KListItem^)" />
                ///
                /// <param name="refStyle">RefStyle for this object.</param>
                void Remove(KListItem^ item, Nullable<KRefStyle> refStyle)
                {
                    if (!kType_IsValue(kList_ItemType(Handle)))
                    {
                        kObject object = KToHandle(item->Get<KObject^>());
                        KAdjustRef(object, kFALSE, refStyle);
                    }

                    KCheck(kList_Remove(Handle, KListItem::ToHandle(item)));
                }

                /// <summary>Ensures that capacity is reserved for at least the specified number of list items.</summary>
                /// 
                /// <param name="capacity">List capacity, in items.</param>
                void Reserve(k64s capacity)
                {
                    KCheck(kList_Reserve(Handle, (kSize)capacity)); 
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
                    RemoveRefAll(refStyle);

                    KCheck(kList_Clear(Handle)); 
                }

                /// <summary>Gets a reference to the first list item.</summary>
                property KListItem^ First
                {
                    KListItem^ get() { return KListItem::ToObject(this, Handle, kList_First(Handle)); }
                }

                /// <summary>Gets a reference to the last list item.</summary>
                property KListItem^ Last
                {
                    KListItem^ get() { return KListItem::ToObject(this, Handle, kList_Last(Handle)); }
                }

                /// <summary>Gets a reference to the list item at the specified index.</summary>
                /// 
                /// <remarks>This method requires a linear search through the list.</remarks>
                /// 
                /// <param name="index">List item index.</param>
                /// <returns>List item at index.</returns>
                KListItem^ FindIndex(k64s index)
                {
                    return KListItem::ToObject(this, Handle, kList_FindIndex(Handle, (kSize)index)); 
                }

            private:
                void AddRefAll(Nullable<KRefStyle> refStyle)
                {
                    AdjustRefAll(kTRUE, refStyle); 
                }

                void RemoveRefAll(Nullable<KRefStyle> refStyle)
                { 
                    AdjustRefAll(kFALSE, refStyle);
                }

                /// <summary>Adjusts all objects ref count based on the refStyle parameter, or the objects default ref style.</summary>
                void AdjustRefAll(kBool add, Nullable<KRefStyle> refStyle)
                {
                    if (!kType_IsValue(kList_ItemType(Handle)))
                    {
                        KListItem^ it = this->First;
                        while (it != nullptr)
                        {
                            KObject^ obj = it->Get<KObject^>();
                            kObject object = obj->ToHandle().ToPointer();
                            
                            KAdjustRef(object, add, refStyle);

                            it = it->Next;
                        }
                    }
                }

                /// <summary>Adjusts a single object's ref count based on the refStyle parameter, or the object's default ref style.</summary>
                void AdjustRef(kObject object, kBool add, Nullable<KRefStyle> refStyle)
                {
                    if (object != kNULL && !kType_IsValue(kList_ItemType(Handle)))
                    {
                        KAdjustRef(object, add, refStyle);
                    }
                }
            };
        }
    }
}

#endif

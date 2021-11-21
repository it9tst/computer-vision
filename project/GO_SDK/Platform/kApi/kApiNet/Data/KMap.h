//
// KMap.h
// 
// Copyright (C) 2014-2021 by LMI Technologies Inc.
// Licensed under the MIT License.
// Redistributed files must retain the above copyright notice.
// 
#ifndef K_API_NET_MAP_H
#define K_API_NET_MAP_H

#include <kApi/Data/kMap.h>
#include "kApiNet/KAlloc.h"

namespace Lmi3d
{
    namespace Zen 
    {       
        namespace Data
        {
            /// <summary>Represents a node within a KMap object.</summary>           
            public ref struct KMapItem
            {          
            public:
                /// <summary>Gets the key associated with a map item.</summary>
                /// <typeparam name="TKey">Key type.</typeparam>
                /// <typeparam name="TValue">Value type.</typeparam>
                /// <returns>Map key.</returns>
                generic <typename TKey, typename TValue>
                TKey GetKey()
                {
                    KGenericArg<TKey> genericKey(kMap_KeyType(m_map), kMap_Key(m_map, m_item));

                    return genericKey.ClrData;
                }

                /// <summary>Gets the value associated with a map item.</summary>
                /// <typeparam name="TKey">Key type.</typeparam>
                /// <typeparam name="TValue">Value type.</typeparam>
                /// <returns>Map value.</returns>
                generic <typename TKey, typename TValue>
                TValue GetValue()
                {
                    KGenericArg<TValue> genericValue(kMap_ValueType(m_map), kMap_Value(m_map, m_item));

                    return genericValue.ClrData;
                }

                /// <summary>Set the value associated with a map item.</summary>
                /// <typeparam name="TKey">Key type.</typeparam>
                /// <typeparam name="TValue">Value type.</typeparam>
                /// <param name="value">Map value</param>
                generic <typename TKey, typename TValue>
                void SetValue(TValue value)
                {
                    KGenericArg<TValue> genericValue(kMap_ValueType(m_map), value);

                    KCheck(kMap_SetValue(m_map, m_item, genericValue.ZenData));
                }

                /// <summary> Gets a reference to the next map item.</summary>
                property KMapItem^ Next
                {
                    KMapItem^ get() { return KMapItem::ToObject(m_root, m_map, kMap_Next(m_map, m_item)); }
                }

                virtual bool Equals(Object^ other) override
                {
                    KMapItem^ it = dynamic_cast<KMapItem^>(other);

                    if (it != nullptr)
                    {
                        return (it->m_map == this->m_map) && (it->m_item == this->m_item);
                    }

                    return false;
                }
           
            internal:

                KMapItem(Object^ root, kMap map, kPointer item)
                    : m_root(root), m_map(map), m_item(item) {}

                static kPointer ToHandle(KMapItem^ it)
                {
                    return (it == nullptr) ? kNULL : it->m_item;
                }

                static KMapItem^ ToObject(Object^ root, kMap map, kPointer it)
                {
                    return (it == kNULL) ? nullptr : gcnew KMapItem(root, map, it);
                }

            private:
                Object^ m_root;               //prevents premature finalization of parent object
                kMap m_map; 
                kMapItem m_item; 
            };      

            /// <summary>Represents a collection of key-value pairs stored in a hash table.</summary>
            /// 
            /// <remarks>
            /// <para>The KMap class represents a hash-table of key-value pairs. The KMap constructor accepts KType arguments that determine
            /// the key type and value type. The map will automatically grow as new items are inserted.</para>
            /// 
            /// <para>KMap supports the KObject.Clone and KObject.Size methods.</para>
            /// 
            /// <para>KMap supports the kdat6 serialization protocol.</para>
            ///
            /// <para>Default KRefStyle: Auto</para>
            /// </remarks>
            public ref class KMap : public KObject
            {
                KDeclareAutoClass(KMap, kMap)

            public:
                /// <summary>Initializes a new instance of the KMap class with the specified Zen object handle.</summary>           
                /// <param name="handle">Zen object handle.</param>
                KMap(IntPtr handle)
                    : KObject(handle, DefaultRefStyle)
                {}

                /// <inheritdoc cref="KMap(IntPtr)" />
                ///
                /// <param name="refStyle">RefStyle for this object.</param>
                KMap(IntPtr handle, KRefStyle refStyle)
                    : KObject(handle, refStyle)
                {}

                /// <summary>Initializes a new instance of the KMap class without specifying key/value types.</summary>
                KMap()
                    : KObject(DefaultRefStyle)
                {
                    kMap handle = kNULL;

                    KCheck(kMap_Construct(&handle, kTypeOf(kVoid), kTypeOf(kVoid), 0, kNULL));

                    Handle = handle;
                }

                /// <summary>Initializes a new instance of the KMap class with the specified key/value types and initial capacity.</summary>
                /// 
                /// <param name="keyType">Type of map key.</param>
                /// <param name="valueType">Type of map value.</param>
                /// <param name="initialCapacity">Capacity initially reserved for map items.</param>
                KMap(KType^ keyType, KType^ valueType, k64s initialCapacity)
                    : KObject(DefaultRefStyle)
                {
                    kMap handle = kNULL;

                    KCheck(kMap_Construct(&handle, KToHandle(keyType), KToHandle(valueType), (kSize)initialCapacity, kNULL));

                    Handle = handle;
                }

                /// <inheritdoc cref="KMap(KType^, KType^, k64s)" />
                /// <param name="allocator">Memory allocator.</param>
                KMap(KType^ keyType, KType^ valueType, k64s initialCapacity, KAlloc^ allocator)
                    : KObject(DefaultRefStyle)
                {
                    kMap handle = kNULL;

                    KCheck(kMap_Construct(&handle, KToHandle(keyType), KToHandle(valueType), (kSize)initialCapacity, KToHandle(allocator)));

                    Handle = handle;
                }

                /// <inheritdoc cref="KMap(KType^, KType^, k64s, KAlloc^)" />
                ///
                /// <param name="refStyle">RefStyle for this object.</param>
                KMap(KType^ keyType, KType^ valueType, k64s initialCapacity, KAlloc^ allocator, KRefStyle refStyle)
                    : KObject(refStyle)
                {
                    kMap handle = kNULL;

                    KCheck(kMap_Construct(&handle, KToHandle(keyType), KToHandle(valueType), (kSize)initialCapacity, KToHandle(allocator)));

                    Handle = handle;
                }

                /// <summary>Reallocates the map.</summary>
                /// 
                /// <remarks>Existing items are discarded.</remarks>
                /// 
                /// <param name="keyType">Type of map key.</param>
                /// <param name="valueType">Type of map value.</param>
                /// <param name="initialCapacity">Capacity initially reserved for map items.</param>
                void Allocate(KType^ keyType, KType^ valueType, k64s initialCapacity)
                {
                    Allocate(keyType, valueType, initialCapacity, Nullable<KRefStyle>());
                }

                /// <inheritdoc cref="Allocate(KType^, KType^, k64s)" />
                ///
                /// <param name="refStyle">RefStyle for this object.</param>
                void Allocate(KType^ keyType, KType^ valueType, k64s initialCapacity, Nullable<KRefStyle> refStyle)
                {
                    RemoveRefAll(refStyle);

                    KCheck(kMap_Allocate(Handle, KToHandle(keyType), KToHandle(valueType), (kSize)initialCapacity)); 
                }

                /// <summary>Performs a shallow copy of the source map.</summary>
                /// 
                /// <remarks>Source key-value pairs are copied by value; if the source map contains objects, the object
                /// handles are copied but the objects are not cloned.</remarks>
                /// 
                /// <param name="source">Map to be copied.</param>
                void Assign(KMap^ source)
                {
                    Assign(source, Nullable<KRefStyle>());
                }

                /// <inheritdoc cref="Assign(KMap^)" />
                ///
                /// <param name="refStyle">RefStyle for this object.</param>
                void Assign(KMap^ source, Nullable<KRefStyle> refStyle)
                {
                    RemoveRefAll(refStyle);

                    KCheck(kMap_Assign(Handle, KToHandle(source))); 

                    AddRefAll(refStyle);
                }

                /// <summary>Returns the key type.</summary>
                property KType^ KeyType
                {
                    KType^ get() { return gcnew KType(kMap_KeyType(Handle)); }
                }

                /// <summary>Returns the value type.</summary>
                property KType^ ValueType
                {
                    KType^ get() { return gcnew KType(kMap_ValueType(Handle)); }
                }

                /// <summary>Returns the count of map elements.</summary>
                property k64s Count
                {
                    k64s get() { return (k64s) kMap_Count(Handle); }
                }

                /// <summary>Returns the number of elements for which space has been allocated.</summary>
                property k64s Capacity
                {
                    k64s get() { return (k64s)kMap_Capacity(Handle); }
                }

                /// <summary>Finds the value associated with the given key.</summary>
                /// 
                /// <typeparam name="TKey">Key type.</typeparam>
                /// <typeparam name="TValue">Value type.</typeparam>
                /// <param name="key">Map key.</param>
                /// <returns>Map value.</returns>
                /// <exception cref="KException">Thrown if not found.</exception>
                generic <typename TKey, typename TValue>
                TValue Find(TKey key)
                {
                    return Find<TKey, TValue>(key, Nullable<KRefStyle>());
                }

                /// <summary>Finds the value associated with the given key.</summary>
                /// 
                /// <typeparam name="TKey">Key type.</typeparam>
                /// <typeparam name="TValue">Value type.</typeparam>
                /// <param name="key">Map key.</param>
                /// <param name="refStyle">RefStyle for this object.</param>
                /// <returns>Map value.</returns>
                /// <exception cref="KException">Thrown if not found.</exception>
                generic <typename TKey, typename TValue>
                TValue Find(TKey key, Nullable<KRefStyle> refStyle)
                {
                    KGenericArg<TKey> genericKey(kMap_KeyType(Handle), key);
                    KGenericArg<TValue> genericValue(kMap_ValueType(Handle));

                    KCheck(kMap_Find(Handle, genericKey.ZenData, genericValue.ZenData));

                    AdjustGenericRef(genericValue, kTRUE, refStyle);

                    return genericValue.ClrData;
                }

                /// <summary>Adds a new key-value pair.</summary>
                /// 
                /// <typeparam name="TKey">Key type.</typeparam>
                /// <typeparam name="TValue">Value type.</typeparam>
                /// <param name="key">Map key.</param>
                /// <param name="value">Map value.</param>
                /// <exception cref="KException">Thrown if already present.</exception>
                generic <typename TKey, typename TValue>
                void Add(TKey key, TValue value)
                {
                    Add(key, value, Nullable<KRefStyle>());
                }
                
                /// <summary>Adds a new key-value pair.</summary>
                /// 
                /// <typeparam name="TKey">Key type.</typeparam>
                /// <typeparam name="TValue">Value type.</typeparam>
                /// <param name="key">Map key.</param>
                /// <param name="value">Map value.</param>
                /// <param name="refStyle">RefStyle for this object.</param>
                /// <exception cref="KException">Thrown if already present.</exception>
                generic <typename TKey, typename TValue>
                void Add(TKey key, TValue value, Nullable<KRefStyle> refStyle)
                {
                    KGenericArg<TKey> genericKey(kMap_KeyType(Handle), key);
                    KGenericArg<TValue> genericValue(kMap_ValueType(Handle), value);

                    KCheck(kMap_Add(Handle, genericKey.ZenData, genericValue.ZenData));

                    AdjustGenericRef(genericKey, kTRUE, refStyle);
                    AdjustGenericRef(genericValue, kTRUE, refStyle);
                }

                /// <summary>Adds or replaces a key-value pair.</summary>
                /// 
                /// <typeparam name="TKey">Key type.</typeparam>
                /// <typeparam name="TValue">Value type.</typeparam>
                /// <param name="key">Map key.</param>
                /// <param name="value">map value.</param>
                generic <typename TKey, typename TValue>
                void Replace(TKey key, TValue value)
                {
                    KGenericArg<TKey> genericKey(kMap_KeyType(Handle), key);
                    KGenericArg<TValue> genericValue(kMap_ValueType(Handle), value);

                    // Don't need to dispose of old value; handled for us by the native kMap_Replace
                    KCheck(kMap_Replace(Handle, genericKey.ZenData, genericValue.ZenData));
                }

                /// <summary>Removes a key-value pair from the map.</summary>
                /// 
                /// <typeparam name="TKey">Key type.</typeparam>
                /// <typeparam name="TValue">Value type.</typeparam>
                /// <param name="key">Map key.</param>
                /// <exception cref="KException">Thrown if not found.</exception>
                generic <typename TKey, typename TValue>
                void Remove(TKey key)
                {
                    Remove<TKey, TValue>(key, Nullable<KRefStyle>());
                }

                /// <summary>Removes a key-value pair from the map.</summary>
                /// 
                /// <typeparam name="TKey">Key type.</typeparam>
                /// <typeparam name="TValue">Value type.</typeparam>
                /// <param name="key">Map key.</param>
                /// <param name="refStyle">RefStyle for this object.</param>
                /// <exception cref="KException">Thrown if not found.</exception>
                generic <typename TKey, typename TValue>
                void Remove(TKey key, Nullable<KRefStyle> refStyle)
                {
                    KGenericArg<TKey> genericKey(kMap_KeyType(Handle), key);                    
                    KGenericArg<TKey> genericOldKey(kMap_KeyType(Handle));
                    KGenericArg<TValue> genericOldValue(kMap_ValueType(Handle));

                    KCheck(kMap_Remove(Handle, genericKey.ZenData, genericOldKey.ZenData, genericOldValue.ZenData));

                    AdjustGenericRef(genericOldKey, kFALSE, refStyle);
                    AdjustGenericRef(genericOldValue, kFALSE, refStyle);
                }

                /// <summary>Removes a key-value pair from the map.</summary>
                /// 
                /// <remarks>This method does not automatically dispose the key or the value.</remarks>
                ///
                /// <typeparam name="TKey">Key type.</typeparam>
                /// <typeparam name="TValue">Value type.</typeparam>
                /// <param name="key">Map key.</param>
                /// <param name="oldKey">Receives old key.</param>
                /// <param name="oldValue">Receives old value.</param>
                /// <exception cref="KException">Thrown if not found.</exception>
                generic <typename TKey, typename TValue>
                void Remove(TKey key, [Out] TKey% oldKey, [Out] TValue% oldValue)
                {
                    KGenericArg<TKey> genericKey(kMap_KeyType(Handle), key);
                    KGenericArg<TKey> genericOldKey(kMap_KeyType(Handle));
                    KGenericArg<TValue> genericOldValue(kMap_ValueType(Handle));

                    KCheck(kMap_Remove(Handle, genericKey.ZenData, genericOldKey.ZenData, genericOldValue.ZenData));

                    oldKey = genericOldKey.ClrData; 
                    oldValue = genericOldValue.ClrData; 
                }

                /// <summary>Ensures that capacity is reserved for at least the specified number of map items.</summary>
                /// 
                /// <param name="capacity">Map capacity, in items.</param>
                void Reserve(k64s capacity)
                {
                    // Doesn't decrease capacity, so don't need to remove refs.
                    KCheck(kMap_Reserve(Handle, (kSize)capacity));
                }

                /// <summary>Sets the count of map items to zero.</summary>
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
                    KCheck(kMap_Clear(Handle));
                }

                /// <summary>Gets a reference to the first map item (key-value pair).</summary>
                property KMapItem^ First
                {
                    KMapItem^ get() { return KMapItem::ToObject(this, Handle, kMap_First(Handle)); }
                }

                /// <summary>Attempts to find the map item associated with the given key.</summary>
                /// 
                /// <typeparam name="TKey">Key type.</typeparam>
                /// <typeparam name="TValue">Value type.</typeparam>
                /// <param name="key">Pointer to key.</param>
                /// <returns>Map item, or null if not found</returns>
                generic <typename TKey, typename TValue>
                KMapItem^ TryFindItem(TKey key)
                {
                    KGenericArg<TKey> genericKey(kMap_KeyType(Handle), key);
                    kMapItem item = kNULL; 

                    if (kSuccess(kMap_FindItem(Handle, genericKey.ZenData, &item)))
                    {
                        return gcnew KMapItem(this, Handle, item); 
                    }
                    else
                    {
                        return nullptr;
                    }
                }

                /// <summary>Finds the map item associated with the given key.</summary>
                /// 
                /// <typeparam name="TKey">Key type.</typeparam>
                /// <typeparam name="TValue">Value type.</typeparam>
                /// <param name="key">Pointer to key.</param>
                /// <returns>Map item</returns>
                /// <exception cref="KException">Thrown if not found.</exception>
                generic <typename TKey, typename TValue>
                KMapItem^ FindItem(TKey key)
                {
                    KGenericArg<TKey> genericKey(kMap_KeyType(Handle), key);
                    kMapItem item = kNULL; 

                    KCheck(kMap_FindItem(Handle, genericKey.ZenData, &item)); 

                    return gcnew KMapItem(this, Handle, item);
                }

                /// <summary>Removes an item from the map.</summary>
                /// 
                /// <param name="item">Map item.</param>
                void RemoveItem(KMapItem^ item)
                {
                    RemoveItem(item, Nullable<KRefStyle>());
                }

                /// <inheritdoc cref="RemoveItem(KMapItem^)" />
                ///
                /// <param name="refStyle">RefStyle for this object.</param>
                void RemoveItem(KMapItem^ item, Nullable<KRefStyle> refStyle)
                {
                    kObject itemHandle = KMapItem::ToHandle(item);

                    AdjustRefMapItem(itemHandle, kFALSE, refStyle);
                    KCheck(kMap_RemoveItem(Handle, KMapItem::ToHandle(item))); 
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

                /// <summary>Adjusts all mapItems KVP ref count based on the refStyle parameter, or the KVP default ref style.</summary>
                void AdjustRefAll(kBool add, Nullable<KRefStyle> refStyle)
                {
                    if (kType_IsValue(kMap_ValueType(Handle)) && kType_IsValue(kMap_KeyType(Handle)))
                    {
                        return; // Quick return if both are value types
                    }

                    kMapItem it = kMap_First(Handle);
                    while (!kIsNull(it))
                    {
                        AdjustRefMapItem(it, add, refStyle);
                        it = kMap_Next(Handle, it);
                    }
                }

                /// <summary>Adjusts a single KVP ref count based on the refStyle parameter, or the KVP's default ref style.</summary>
                void AdjustRefMapItem(kMapItem mapItem, kBool add, Nullable<KRefStyle> refStyle)
                {
                    if (!kType_IsValue(kMap_KeyType(Handle))) 
                    { 
                        kObject key = kMap_KeyAsT(Handle, mapItem, kObject);
                        KAdjustRef(key, add, refStyle);
                    }

                    if (!kType_IsValue(kMap_ValueType(Handle))) 
                    { 
                        kObject value = kMap_ValueAsT(Handle, mapItem, kObject);
                        KAdjustRef(value, add, refStyle);
                    }
                }

                generic <typename T>
                void AdjustGenericRef(KGenericArg<T> arg, kBool add, Nullable<KRefStyle> refStyle)
                {
                    if (kType_IsValue(arg.ZenType))
                    {
                        return;
                    }
                        
                    kObject object = *(kObject*)arg.ZenData;
                    KAdjustRef(object, add, refStyle);
                }
            };
        }
    }
}

#endif

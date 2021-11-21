/** 
 * @file    kMap.h
 * @brief   Declares the kMap class. 
 *
 * @internal
 * Copyright (C) 2012-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_MAP_H
#define K_API_MAP_H

#include <kApi/kApiDef.h>

/**
 * @class       kMap
 * @extends     kObject
 * @ingroup     kApi-Data
 * @brief       Represents a collection of key-value pairs stored in a hash table.
 * 
 * The kMap class represents a hash table of key-value pairs. The kMap constructor accepts kType arguments that determine 
 * the key type and value type. 
 * 
 * The map will automatically grow as new items are inserted. The size of the internal bucket array is always 
 * a power of two and secondary hashing is not performed on keys. This places the burden on the caller to ensure 
 * that keys have roughly uniform distribution, else clustering may occur. If in doubt, consider installing 
 * a custom hash function (via kMap_SetHashFx) to ensure good key distribution. 
 * 
 * The following example illustrates how to construct a kMap instance, populate it, and access elements. 
 * 
@code {.c}
kStatus MapExample()
{
    kMap map = kNULL; 
    kText32 keys[] = { "one", "two", "three", "four", "five" }; 
    k32s values[] = { 1, 2, 3, 4, 5 }; 
    k32s result = 0; 
    kMapItem resultIt = kNULL;     //map iterator
    kSize i; 
    kMapItem it = kNULL;     //map iterator
    
    kTry
    {
        //create a map that can store small char-array keys and 32-bit integer values
        kTest(kMap_Construct(&map, kTypeOf(kText32), kTypeOf(k32s), 0, kNULL)); 
         
        //add some initial items to the map
        for (i = 0; i < kCountOf(keys); ++i)
        {
            kTest(kMap_Add(map, &keys[i], &values[i])); 
        }

        //perform some queries;
        //kMap_Find looks up a key and returns the value
        if (kSuccess(kMap_Find(map, "three", &result)))
        {
            printf("three: %d\n", result); 
        }

        //kMap_FindItem looks up a key and returns an iterator to the key-value pair
        if (kSuccess(kMap_FindItem(map, "four", &resultIt)))
        {
            printf("four: %d\n", kMap_ValueAs_(map, resultIt, k32s)); 
        }
         
        //print some information about the map 
        printf("Key type: %s\n", kType_Name(kMap_KeyType(map))); 
        printf("Value type: %s\n", kType_Name(kMap_ValueType(map))); 
        printf("Count: %u\n", (k32u) kMap_Count(map)); 

        //print the map items; note, the items will not be printed in sorted order 
        //because kMap is not a sorted collection
        it = kMap_First(map); 
        
        while (!kIsNull(it))
        {
            const kChar* key = kMap_Key(map, it); 
            k32s value = *(k32s*) kMap_Value(map, it); 

            printf("key: %s, value: %d\n", key, value); 

            it = kMap_Next(map, it); 
        }
    }
    kFinally
    {
         kObject_Destroy(map); 

         kEndFinally(); 
    }
   
    return kOK; 
} 
@endcode
 * 
 * For maps that contain key or value <em>objects</em> (e.g. kString) as opposed to <em>values</em> (e.g. kText32), 
 * the objects are not automatically destroyed when the map is destroyed. To recursively destroy both the map and its
 * keys/values, use kObject_Dispose. Similarly, kMap does not automatically dispose key/value objects when they are 
 * replaced or cleared from the map. 
 *
 @code {.c}
kStatus MapOfObjectsExample()
{
    kMap map = kNULL;    
    kString keyIn = kNULL;   
    kImage valueIn = kNULL; 
    kString keyOut = kNULL;   
    kImage valueOut = kNULL; 
    kImage result = kNULL; 
    kMapItem resultIt = kNULL;        //map iterator
    kMapItem it = kNULL;        //map iterator
    
    kTry
    {
        //create a map that can store kString keys and kImage values
        kTest(kMap_Construct(&map, kTypeOf(kString), kTypeOf(kImage), 0, kNULL)); 
         
        //add some initial items to the map
        kTest(kString_Construct(&keyIn, "one", kNULL)); 
        kTest(kImage_Construct(&valueIn, kTypeOf(k8u), 100, 100, kNULL));         
        kTest(kMap_AddT(map, &keyIn, &valueIn)); 
        keyIn = valueIn = kNULL; 

        kTest(kString_Construct(&keyIn, "two", kNULL)); 
        kTest(kImage_Construct(&valueIn, kTypeOf(k8u), 200, 200, kNULL));         
        kTest(kMap_AddT(map, &keyIn, &valueIn)); 
        keyIn = valueIn = kNULL; 

        kTest(kString_Construct(&keyIn, "three", kNULL)); 
        kTest(kImage_Construct(&valueIn, kTypeOf(k8u), 300, 300, kNULL));         
        kTest(kMap_AddT(map, &keyIn, &valueIn)); 
        keyIn = valueIn = kNULL; 

        //perform some queries;
        //kMap_FindT looks up a key and returns the value
        if (kSuccess(kMap_FindT(map, "two", &result)))
        {
            printf("two: width(%u), height(%u)\n", (k32u)kImage_Width(result), (k32u)kImage_Height(result)); 
        }

        //kMap_FindItemT looks up a key and returns an iterator to the key-value pair
        if (kSuccess(kMap_FindItemT(map, "one", &resultIt)))
        {
            kImage image = kMap_ValueAsT(map, resultIt, kImage); 

            printf("one: width(%u), height(%u)\n", (k32u)kImage_Width(image), (k32u)kImage_Height(image)); 
        }

        //there are several ways to replace a key/value pair, depending on whether it is already known 
        //that the key is in the collection, and whether the key/value need to be destroyed/disposed;         
        //the approach below first removes and destroys the key/value if they exist, then adds replacements
        if (kSuccess(kMap_RemoveT(map, "one", &keyOut, &valueOut)))
        {
            kObject_Destroy(keyOut);          
            kObject_Destroy(valueOut);          
        }
        
        kTest(kString_Construct(&keyIn, "one", kNULL)); 
        kTest(kImage_Construct(&valueIn, kTypeOf(k8u), 1000, 1000, kNULL));         
        kTest(kMap_AddT(map, &keyIn, &valueIn)); 
        keyIn = valueIn = kNULL; 
    }
    kFinally
    {
         //clean up temp variables, in case of error above
         kObject_Destroy(keyIn);          
         kObject_Destroy(valueIn);          

         //dispose map; will destroy internal key/value objects
         kObject_Dispose(map); 

         kEndFinally(); 
    }
   
    return kOK; 
} 
@endcode
 *
 * kMap supports the kObject_Clone, kObject_Dispose, and kObject_Size methods.
 *
 * kMap supports the kdat6 serialization protocol. 
 * 
 */
//typedef kObject kMap;   --forward-declared in kApiDef.x.h

/**
 * Represents a key-value pair within a map. 
 * @typedef kPointer kMapItem 
 * @relates kMap
*/
typedef kPointer kMapItem; 

#include <kApi/Data/kMap.x.h>

/** 
 * Constructs a kMap object.
 *
 * @public                      @memberof kMap
 * @param   map                 Map object. 
 * @param   keyType             Type of map key.
 * @param   valueType           Type of map value.
 * @param   initialCapacity     Capacity initially reserved for map items. 
 * @param   allocator           Memory allocator. 
 * @return                      Operation status. 
 */
kFx(kStatus) kMap_Construct(kMap* map, kType keyType, kType valueType, kSize initialCapacity, kAlloc allocator);

/** 
 * Reallocates the map. 
 * 
 * Existing items are discarded. 
 *
 * @public                      @memberof kMap
 * @param   map                 Map object. 
 * @param   keyType             Type of map key.
 * @param   valueType           Type of map value.
 * @param   initialCapacity     Capacity initially reserved for map items. 
 * @return                      Operation status. 
 */
kFx(kStatus) kMap_Allocate(kMap map, kType keyType, kType valueType, kSize initialCapacity);

/** 
 * Performs a shallow copy of the source map.  
 *
 * Source key-value pairs are copied by value; if the source map contains objects, the object 
 * handles are copied but the objects are not cloned. 
 *
 * @public              @memberof kMap
 * @param   map         Map object. 
 * @param   source      Map to be copied.
 * @return              Operation status. 
 */
kFx(kStatus) kMap_Assign(kMap map, kMap source);

/** 
 * Sets a custom key equality comparator. 
 *
 * @public              @memberof kMap
 * @param   map         Map object. 
 * @param   function    Key equality function (or kNULL to unset). 
 * @return              Operation status. 
 */
kInlineFx(kStatus) kMap_SetEqualsFx(kMap map, kEqualsFx function)
{
    kObj(kMap, map);   

    obj->equalsFx = function; 

    return kOK; 
}
/** 
 * Sets a custom hash code generator. 
 *
 * Calling this method will cause the existing map keys to be rehashed using 
 * using the new hash function. If changing both the equals function and the hash function, 
 * change equals first, then hash. 
 *
 * @public              @memberof kMap
 * @param   map         Map object. 
 * @param   function    Hash code function (or kNULL to unset). 
 * @return              Operation status. 
 */
kInlineFx(kStatus) kMap_SetHashFx(kMap map, kHashFx function)
{
    kObj(kMap, map);   
   
    obj->hashFx = function; 

    kCheck(xkMap_Rehash(map)); 

    return kOK; 
}

/** 
 * Returns the key type. 
 *
 * @public              @memberof kMap
 * @param   map         Map object. 
 * @return              Key type. 
 */
kInlineFx(kType) kMap_KeyType(kMap map)
{
    kObj(kMap, map);   

    return obj->keyField.type; 
}

/** 
 * Returns the value type. 
 *
 * @public              @memberof kMap
 * @param   map         Map object. 
 * @return              Key type. 
 */
kInlineFx(kType) kMap_ValueType(kMap map)
{
    kObj(kMap, map);   

    return obj->valueField.type; 
}

/** 
 * Returns the count of map elements.  
 *
 * @public              @memberof kMap
 * @param   map         Map object. 
 * @return              Count of elements. 
 */
kInlineFx(kSize) kMap_Count(kMap map)
{
    kObj(kMap, map);   

    return obj->count; 
}

/** 
 * Returns the number of elements for which space has been allocated.  
 *
 * @public              @memberof kMap
 * @param   map         Map object. 
 * @return              Map capacity, in elements. 
 */
kInlineFx(kSize) kMap_Capacity(kMap map)
{
    kObj(kMap, map);   

    return obj->capacity; 
}

/** 
 * Finds the value associated with the given key. 
 * 
 * @public              @memberof kMap
 * @param   map         Map object. 
 * @param   key         Pointer to key to be found. 
 * @param   value       Optionally receives value (can be kNULL). 
 * @return              kOK if found; kERROR_NOT_FOUND if not found. 
 */
kFx(kStatus) kMap_Find(kMap map, const void* key, void* value);

/** 
 * Finds the value associated with the given key. 
 * 
 * A debug assertion will be raised if the size of the specified key or value type is not equal to the 
 * size of the collection key or value type, respectively.
 * 
 * @relates                 kMap
 * @param   kMap_map        Map object. 
 * @param   KPtr_key        Strongly-typed pointer to key that should be found. 
 * @param   VPtr_value      Strongly-typed pointer to destination for found value. 
 * @return                  kOK if found; kERROR_NOT_FOUND if not found. 
 */
#define kMap_FindT(kMap_map, KPtr_key, VPtr_value) \
    xkMap_FindT(kMap_map, KPtr_key, VPtr_value, sizeof(*(KPtr_key)), sizeof(*(VPtr_value)))

/** 
 * Reports whether the specified key is present in the map.
 * 
 * @public              @memberof kMap
 * @param   map         Map object. 
 * @param   key         Pointer to key. 
 * @return              kTRUE if found. 
 */
kInlineFx(kBool) kMap_Has(kMap map, const void* key)
{
    return kSuccess(kMap_Find(map, key, kNULL));
}

/** 
 * Reports whether the specified key is present in the map.
 * 
 * A debug assertion will be raised if the size of the specified key type is not equal to the 
 * size of the collection key type.
 * 
 * @relates             kMap
 * @param   kMap_map    Map object. 
 * @param   KPtr_key    Strongly-typed pointer to key. 
 * @return              kTRUE if found. 
 */
#define kMap_HasT(kMap_map, KPtr_key) \
     xkMap_HasT(kMap_map, KPtr_key, sizeof(*(KPtr_key)))

/** 
 * Adds a new key-value pair. 
 * 
 * @public              @memberof kMap
 * @param   map         Map object. 
 * @param   key         Pointer to key. 
 * @param   value       Pointer to value.
 * @return              kOK if added; kERROR_ALREADY_EXISTS if key already present. 
 */
kFx(kStatus) kMap_Add(kMap map, const void* key, const void* value); 

/** 
 * Adds a new key-value pair. 
 * 
 * A debug assertion will be raised if the size of the specified key or value type is not equal to the 
 * size of the collection key or value type, respectively.
 * 
 * @relates                 kMap
 * @param   kMap_map        Map object. 
 * @param   KPtr_key        Strongly-typed pointer to key. 
 * @param   VPtr_value      Strongly-typed pointer to value.
 * @return                  kOK if added; kERROR_ALREADY_EXISTS if key already present. 
 */
#define kMap_AddT(kMap_map, KPtr_key, VPtr_value) \
    xkMap_AddT(kMap_map, KPtr_key, VPtr_value, sizeof(*(KPtr_key)), sizeof(*(VPtr_value)))

/** 
 * Adds or replaces a key-value pair. 
 * 
 * For object-based keys or values, the old objects are not automatically disposed. In this case, 
 * consider using kMap_Remove to first remove and dispose the existing item, then kMap_Add
 * to add the new item.
 * 
 * @public              @memberof kMap
 * @param   map         Map object. 
 * @param   key         Pointer to key. 
 * @param   value       Pointer to value.
 * @return              Operation status.  
 */
kFx(kStatus) kMap_Replace(kMap map, const void* key, const void* value); 

/** 
 * Adds or replaces a key-value pair. 
 * 
 * For object-based keys or values, the old objects are not automatically disposed. In this case, 
 * consider using kMap_Remove to first remove and dispose the existing item, then kMap_Add
 * to add the new item.
 * 
 * A debug assertion will be raised if the size of the specified key or value type is not equal to the 
 * size of the collection key or value type, respectively.
 * 
 * @relates             kMap
 * @param   kMap_map    Map object. 
 * @param   KPtr_key    Strongly-typed pointer to key. 
 * @param   VPtr_value  Strongly-typed pointer to value.
 * @return              Operation status.  
 */
#define kMap_ReplaceT(kMap_map, KPtr_key, VPtr_value) \
    xkMap_ReplaceT(kMap_map, KPtr_key, VPtr_value, sizeof(*(KPtr_key)), sizeof(*(VPtr_value)))

/** 
 * Removes a key-value pair from the map, optionally returning the old key and/or value.
 * 
 * For object-based keys or values, the old objects are not automatically disposed. In this case, 
 * use the oldKey and/or oldValue parameters to receive the previous object(s) and then dispose 
 * them. 
 * 
 * @public              @memberof kMap
 * @param   map         Map object. 
 * @param   key         Pointer to key. 
 * @param   oldKey      Optionally receives key (can be kNULL). 
 * @param   oldValue    Optionally receives value (can be kNULL). 
 * @return              kOK if removed; kERROR_NOT_FOUND if key not found. 
 */
kFx(kStatus) kMap_Remove(kMap map, const void* key, void* oldKey, void* oldValue); 

/** 
 * Removes a key-value pair from the map, returning the old key and/or value.
 * 
 * For object-based keys or values, the old objects are not automatically disposed. In this case, 
 * use the oldKey and/or oldValue parameters to receive the previous object(s) and then dispose 
 * them. 
 * 
 * A debug assertion will be raised if the size of the specified key or value type is not equal to the 
 * size of the collection key or value type, respectively.
 * 
 * @relates                 kMap
 * @param   kMap_map        Map object. 
 * @param   KPtr_key        Pointer to key. 
 * @param   KPtr_oldKey     Strongly-typed destination pointer to receive old key. 
 * @param   VPtr_oldValue   Strongly-typed destination pointer to receive old value.
 * @return                  kOK if removed; kERROR_NOT_FOUND if key not found. 
 */
#define kMap_RemoveT(kMap_map, KPtr_key, KPtr_oldKey, VPtr_oldValue) \
    xkMap_RemoveT(kMap_map, KPtr_key, KPtr_oldKey, VPtr_oldValue, sizeof(*(KPtr_key)), sizeof(*(KPtr_oldKey)), sizeof(*(VPtr_oldValue)))

/** 
 * Removes a key-value pair from the map.  
 * 
 * For object-based keys or values, the old objects are not automatically disposed. In this case, 
 * kMap_Remove can be used instead to receive the old keys/values.  
 * 
 * @public              @memberof kMap
 * @param   map         Map object. 
 * @param   key         Pointer to key. 
 * @return              kOK if removed; kERROR_NOT_FOUND if key not found. 
 */
kInlineFx(kStatus) kMap_Discard(kMap map, const void* key)
{
    return kMap_Remove(map, key, kNULL, kNULL);
}

/** 
 * Removes a key-value pair from the map.  
 * 
 * For object-based keys or values, the old objects are not automatically disposed. In this case, 
 * use kMap_Remove instead to access the old keys/values.  
 * 
 * A debug assertion will be raised if the size of the specified key type is not equal to the 
 * size of the collection key type.
 * 
 * @relates             @memberof kMap
 * @param   kMap_map    Map object. 
 * @param   KPtr_key    Strongly-typed pointer to key. 
 * @return              kOK if removed; kERROR_NOT_FOUND if key not found. 
 */
#define kMap_DiscardT(kMap_map, KPtr_key) \
    xkMap_DiscardT(kMap_map, KPtr_key, sizeof(*(KPtr_key)))

/** 
 * Ensures that capacity is reserved for at least the specified number of map items. 
 * 
 * @public              @memberof kMap
 * @param   map         Map object. 
 * @param   capacity    Map capacity, in items.
 * @return              kOK if removed; kERROR_NOT_FOUND if key not found. 
 */
kFx(kStatus) kMap_Reserve(kMap map, kSize capacity); 

/** 
 * Sets the count of map items to zero. 
 *
 * @public              @memberof kMap
 * @param   map         Map object. 
 * @return              Operation status. 
 */
kFx(kStatus) kMap_Clear(kMap map); 

/** 
 * Disposes any elements in the map and sets the count of map items to zero.
 *
 * @public              @memberof kMap
 * @param   map         Map object. 
 * @return              Operation status. 
 */
kFx(kStatus) kMap_Purge(kMap map); 

/** 
 * Gets a reference to the first map item (key-value pair). 
 *
 * @public              @memberof kMap
 * @param   map         Map object.
 * @return              First map item, or kNULL.
 */
kFx(kMapItem) kMap_First(kMap map); 

/** 
 * Given a map item, gets a reference to the next map item. 
 *
 * @public              @memberof kMap
 * @param   map         Map object.
 * @param   item        Map item.
 * @return              Next map item, or kNULL.
 */
kFx(kMapItem) kMap_Next(kMap map, kMapItem item); 

/** 
 * Finds the map item associated with the given key. 
 * 
 * @public              @memberof kMap
 * @param   map         Map object. 
 * @param   key         Pointer to key. 
 * @param   item        Optionally receives map item (can be kNULL). 
 * @return              kOK if found; kERROR_NOT_FOUND if not found. 
 */
kFx(kStatus) kMap_FindItem(kMap map, const void* key, kMapItem* item); 

/** 
 * Finds the map item associated with the given key. 
 * 
 * A debug assertion will be raised if the size of the specified key type is not equal to the 
 * size of the collection key type.
 * 
 * @relates                     kMap
 * @param   kMap_map            Map object. 
 * @param   KPtr_key            Strongly-typed pointer to key. 
 * @param   kMapItemPtr_item    Optionally receives map item (can be kNULL). 
 * @return                      kOK if found; kERROR_NOT_FOUND if not found. 
 */
#define kMap_FindItemT(kMap_map, KPtr_key, kMapItemPtr_item) \
    xkMap_FindItemT(kMap_map, KPtr_key, kMapItemPtr_item, sizeof(*(KPtr_key)))

/** 
 * Removes an item from the map. 
 * 
 * @public              @memberof kMap
 * @param   map         Map object. 
 * @param   item        Map item. 
 * @return              Operation status. 
 */
kFx(kStatus) kMap_RemoveItem(kMap map, kMapItem item); 

/** 
 * Returns a pointer to the key associated with a map item. 
 *
 * @public              @memberof kMap
 * @param   map         Map object. 
 * @param   item        Map item. 
 * @return              Pointer to key.
 */
kInlineFx(const void*) kMap_Key(kMap map, kMapItem item)
{   
    kObj(kMap, map);   

    return kPointer_ByteOffset(item, (kSSize)obj->keyField.offset);
}

/** 
 * Returns a strongly-typed pointer to the key associated with a map item. 
 *
 * A debug assertion will be raised if the size of the specified key type is not equal to the 
 * size of the collection key type.
 * 
 * @relates                 kMap
 * @param   kMap_map        Map object. 
 * @param   kMapItem_item   Map item. 
 * @param   K               Key type identifier (e.g., k32s).
 * @return                  Strongly-typed pointer to key.
 */
#define kMap_KeyT(kMap_map, kMapItem_item, K) \
    kCast(K*, xkMap_KeyT(kMap_map, kMapItem_item, sizeof(K)))

/** 
* Gets the key associated with a map item. 
* 
* A debug assertion will be raised if the size of the specified key type is not equal to the 
* size of the collection key type.
*
* @relates                  kMap
* @param   kMap_map         Map object. 
* @param   kMapItem_item    Map item. 
* @param   K                Key type identifier (e.g., k32s).
* @return                   Key content.
*/
#define kMap_KeyAsT(kMap_map, kMapItem_item, K) \
    kPointer_ReadAs(xkMap_KeyAsT(kMap_map, kMapItem_item, sizeof(K)), K)

/** 
 * Returns a pointer to the value associated with a map item. 
 *
 * @public              @memberof kMap
 * @param   map         Map object. 
 * @param   item        Map item. 
 * @return              Pointer to value.
 */
kInlineFx(void*) kMap_Value(kMap map, kMapItem item)
{   
    kObj(kMap, map);   

    return kPointer_ByteOffset(item, (kSSize)obj->valueField.offset);
}

/** 
 * Returns a strongly-typed pointer to the value associated with a map item. 
 *
 * A debug assertion will be raised if the size of the specified value type is not equal to the 
 * size of the collection value type.
 * 
 * @relates                 kMap
 * @param   kMap_map        Map object. 
 * @param   kMapItem_item   Map item. 
 * @param   V               Value type identifier (e.g., k32s).
 * @return                  Strongly-typed pointer to value.
 */
#define kMap_ValueT(kMap_map, kMapItem_item, V) \
    kCast(V*, xkMap_ValueT(kMap_map, kMapItem_item, sizeof(V)))

/** 
 * Sets the value associated with a map item. 
 *
 * @public              @memberof kMap
 * @param   map         Map object. 
 * @param   item        Map item. 
 * @param   value       Pointer to value to be copied into the map item.
 * @return              Operation status. 
 */
kInlineFx(kStatus) kMap_SetValue(kMap map, kMapItem item, const void* value)
{
    kObj(kMap, map);   
    void* valueSlot = kMap_Value(map, item); 

    kValue_Import(obj->valueField.type, valueSlot, value); 

    return kOK; 
} 

/** 
 * Sets the value associated with a map item. 
 *
 * A debug assertion will be raised if the size of the specified value type is not equal to the 
 * size of the collection value type.
 * 
 * @relates                 kMap
 * @param   kMap_map        Map object. 
 * @param   kMapItem_item   Map item. 
 * @param   VPtr_value      Strongly-typed pointer to value to be copied into the map item.
 * @return                  Operation status. 
 */
#define kMap_SetValueT(kMap_map, kMapItem_item, VPtr_value) \
    xkMap_SetValueT(kMap_map, kMapItem_item, VPtr_value, sizeof(*(VPtr_value)))

/** 
* Sets the value associated with a map item. 
* 
* A debug assertion will be raised if the size of the specified value type is not equal to the 
* size of the collection value type.
*
* @relates                  kMap
* @param   kMap_map         Map object. 
* @param   kMapItem_item    Map item. 
* @param   V_value          Item value.  
* @param   V                Value type identifier (e.g., k32s).
* @return                   Value content.
*/
#define kMap_SetValueAsT(kMap_map, kMapItem_item, V_value, V) \
    (kPointer_WriteAs(xkMap_ValueAsT(kMap_map, kMapItem_item, sizeof(V)), V_value, V), (void)0) 

/** 
* Gets the value associated with a map item. 
* 
* A debug assertion will be raised if the size of the specified value type is not equal to the 
* size of the collection value type.
*
* @relates                  kMap
* @param   kMap_map         Map object. 
* @param   kMapItem_item    Map item. 
* @param   V                Value type identifier (e.g., k32s).
* @return                   Value content.
*/
#define kMap_ValueAsT(kMap_map, kMapItem_item, V) \
    kPointer_ReadAs(xkMap_ValueAsT(kMap_map, kMapItem_item, sizeof(V)), V) 

#endif

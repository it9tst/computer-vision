/** 
 * @file    kMap.x.h
 *
 * @internal
 * Copyright (C) 2012-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_MAP_X_H
#define K_API_MAP_X_H

#define xkMAP_CAPACITY_TO_BUCKETS(C)         (5*(C)/4)       //determines minimum buckets, assuming max load of 0.75
#define xkMAP_MIN_ITEMS_PER_BLOCK            (16)            //minimum number of map items per allocated memory block

//represents a single key/value map entry
typedef struct kMapItemStruct //<K, V>
{
    struct kMapItemStruct* next;                //linked list pointer to next item
    kSize hashCode;                             //cached hash code
    //K key;                                    //key; offset/size dynamically calculated
    //V value;                                  //value; offset/size dynamically calculated
} kMapItemStruct;

//represents a block of allocated map items; multiple items
//are allocated at the same time for efficiency
typedef struct kMapItemBlock //<N>
{
    struct kMapItemBlock* next;              //linked list pointer to next memory block 
    kSize itemCount;                         //count of items in block (N)
    kMapItemStruct* items;                   //pointer to first item in block
    //kMapItemStruct item[N];                //array of map items
} kMapItemBlock; 

typedef struct kMapClass
{
    kObjectClass base;                          // base fields
    kMapItemBlock* blocks;                      // allocated memory blocks, where block contains N items
    kMapItemStruct* free;                       // linked list of free items
    kMapItemStruct** buckets;                   // array of buckets, where each bucket is a linked list of items
    kSize bucketCount;                          // count of buckets 
    kSize count;                                // current count of items
    kSize capacity;                             // maximum number of items before allocation required
    xkStructField keyField;                      // key field info
    xkStructField valueField;                    // value field info
    kSize itemSize;                             // size of map item, including key and value fields
    kEqualsFx equalsFx;                         // callback function for equality comparison
    kHashFx hashFx;                             // callback function for hash code generation
    void* collectionItem;                       // temp variable, required to implement kCollection interface
} kMapClass;

kDeclareClassEx(k, kMap, kObject) 

/* 
* Forward declarations. 
*/
kFx(kStatus) kMap_Find(kMap map, const void* key, void* value);
kInlineFx(kBool) kMap_Has(kMap map, const void* key);
kFx(kStatus) kMap_Add(kMap map, const void* key, const void* value); 
kFx(kStatus) kMap_Replace(kMap map, const void* key, const void* value); 
kFx(kStatus) kMap_Remove(kMap map, const void* key, void* oldKey, void* oldValue); 
kInlineFx(kStatus) kMap_Discard(kMap map, const void* key); 
kFx(kStatus) kMap_FindItem(kMap map, const void* key, kMapItem* item); 
kFx(kStatus) kMap_RemoveItem(kMap map, kMapItem item); 
kInlineFx(kStatus) kMap_SetValue(kMap map, kMapItem item, const void* value);

kFx(kStatus) kMap_Purge(kMap map); 
kInlineFx(const void*) kMap_Key(kMap map, kMapItem item); 
kInlineFx(void*) kMap_Value(kMap map, kMapItem item); 
kInlineFx(kType) kMap_KeyType(kMap map); 
kInlineFx(kType) kMap_ValueType(kMap map);

/* 
* Private methods. 
*/

kFx(kStatus) xkMap_ConstructFramework(kMap* map, kAlloc allocator); 

kFx(kStatus) xkMap_Init(kMap map, kType classType, kType keyType, kType valueType, kSize initialCapacity, kAlloc alloc); 

kFx(kStatus) xkMap_VClone(kMap map, kMap source, kAlloc valueAlloc, kObject context); 

kFx(kStatus) xkMap_VRelease(kMap map); 

kInlineFx(kStatus) xkMap_VDisposeItems(kMap map)
{
    return kMap_Purge(map); 
}

kFx(kBool) xkMap_VHasShared(kMap map); 
kFx(kSize) xkMap_VSize(kMap map); 
kFx(kAllocTrait) xkMap_VAllocTraits(kMap map);   

kFx(kStatus) xkMap_WriteDat6V0(kMap map, kSerializer serializer); 
kFx(kStatus) xkMap_ReadDat6V0(kMap map, kSerializer serializer); 

kFx(kStatus) xkMap_Deallocate(kMap map); 

kFx(kStatus) xkMap_Layout(kMap map, kType keyType, kType valueType); 

kFx(kStatus) xkMap_ReserveItems(kMap map, kSize capacity);
kFx(kStatus) xkMap_ReserveBuckets(kMap map);

kFx(kStatus) xkMap_Rehash(kMap map);

kInlineFx(kBool) xkMap_KeyIsRef(kMap map)
{
    return kType_IsReference(kMap_KeyType(map)); 
}

kInlineFx(kBool) xkMap_ValueIsRef(kMap map)
{
    return kType_IsReference(kMap_ValueType(map)); 
}

kInlineFx(kSize) xkMap_HashToIndex(kSize code, kSize count)
{
    return code & (count - 1);
}

kInlineFx(kSize) xkMap_BucketIndex(kMap map, kSize code)
{
    kObj(kMap, map);

    return xkMap_HashToIndex(code, obj->bucketCount); 
}

kInlineFx(kSize) xkMap_HashKey(kMap map, const void* key)
{
    kObj(kMap, map);

    if (obj->hashFx)
    {
        return obj->hashFx(key);
    }
    else
    {
        if (xkMap_KeyIsRef(map))
        {
            return kObject_HashCode(kPointer_ReadAs(key, kObject));  
        }
        else
        {
            return kValue_HashCode(kMap_KeyType(map), key); 
        }
    }
}

kInlineFx(kBool) xkMap_KeyEquals(kMap map, const void* key1, const void* key2)
{
    kObj(kMap, map);

    if (obj->equalsFx)
    {
        return obj->equalsFx(key1, key2);
    }
    else
    {
        if (xkMap_KeyIsRef(map))
        {
            return kObject_Equals(kPointer_ReadAs(key1, kObject), kPointer_ReadAs(key2, kObject));  
        }
        else
        {
            return kValue_Equals(kMap_KeyType(map), key1, key2);  
        }
    }
}

kInlineFx(kSize) xkMapItem_HashCode(kMap map, kMapItem item)
{
    return kCast(kMapItemStruct*, item)->hashCode;
}

kInlineFx(kStatus) xkMap_FindT(kMap map, const void* key, void* value, kSize keySize, kSize valueSize)
{
    kAssert(xkType_IsPointerCompatible(kMap_KeyType(map), keySize)); 
    kAssert(xkType_IsPointerCompatible(kMap_ValueType(map), valueSize)); 

    return kMap_Find(map, key, value);
} 

kInlineFx(kBool) xkMap_HasT(kMap map, const void* key, kSize keySize)
{
    kAssert(xkType_IsPointerCompatible(kMap_KeyType(map), keySize)); 
 
    return kMap_Has(map, key);
} 

kInlineFx(kStatus) xkMap_AddT(kMap map, const void* key, const void* value, kSize keySize, kSize valueSize)
{
    kAssert(xkType_IsPointerCompatible(kMap_KeyType(map), keySize)); 
    kAssert(xkType_IsPointerCompatible(kMap_ValueType(map), valueSize)); 

    return kMap_Add(map, key, value);
} 
 
kInlineFx(kStatus) xkMap_ReplaceT(kMap map, const void* key, const void* value, kSize keySize, kSize valueSize)
{
    kAssert(xkType_IsPointerCompatible(kMap_KeyType(map), keySize)); 
    kAssert(xkType_IsPointerCompatible(kMap_ValueType(map), valueSize)); 

    return kMap_Replace(map, key, value);
} 
 
kInlineFx(kStatus) xkMap_RemoveT(kMap map, const void* key, void* oldKey, void* oldValue, kSize keySize, kSize oldKeySize, kSize oldValueSize)
{
    kAssert(xkType_IsPointerCompatible(kMap_KeyType(map), keySize)); 
    kAssert(xkType_IsPointerCompatible(kMap_KeyType(map), oldKeySize)); 
    kAssert(xkType_IsPointerCompatible(kMap_ValueType(map), oldValueSize)); 

    return kMap_Remove(map, key, oldKey, oldValue);
} 
 
kInlineFx(kStatus) xkMap_DiscardT(kMap map, const void* key, kSize keySize)
{
    kAssert(xkType_IsPointerCompatible(kMap_KeyType(map), keySize)); 
 
    return kMap_Discard(map, key);
}

kInlineFx(kStatus) xkMap_FindItemT(kMap map, const void* key, kMapItem* item, kSize keySize)
{
    kAssert(xkType_IsPointerCompatible(kMap_KeyType(map), keySize)); 
 
    return kMap_FindItem(map, key, item);
}

kInlineFx(kStatus) xkMap_SetValueT(kMap map, kMapItem item, const void* value, kSize valueSize)
{
    kAssert(xkType_IsPointerCompatible(kMap_ValueType(map), valueSize)); 
 
    return kMap_SetValue(map, item, value);
}

kInlineFx(const void*) xkMap_KeyT(kMap map, kMapItem item, kSize keySize)
{
    kAssert(xkType_IsPointerCompatible(kMap_KeyType(map), keySize)); 
 
    return kMap_Key(map, item);
}

kInlineFx(void*) xkMap_ValueT(kMap map, kMapItem item, kSize valueSize)
{
    kAssert(xkType_IsPointerCompatible(kMap_ValueType(map), valueSize)); 
 
    return kMap_Value(map, item);
}

kInlineFx(const void*) xkMap_KeyAsT(kMap map, kMapItem item, kSize keySize)
{
    kAssert(keySize == kType_Size(kMap_KeyType(map)));  

    return kMap_Key(map, item);
}

kInlineFx(void*) xkMap_ValueAsT(kMap map, kMapItem item, kSize valueSize)
{
    kAssert(valueSize == kType_Size(kMap_ValueType(map)));  

    return kMap_Value(map, item);
}

#endif

/** 
 * @file    kMap.cpp
 *
 * @internal
 * Copyright (C) 2012-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <kApi/Data/kMap.h>
#include <kApi/Data/kCollection.h>
#include <kApi/Io/kSerializer.h>

kBeginClassEx(k, kMap) 

    //serialization versions
    kAddPrivateVersionEx(kMap, "kdat6", "5.7.1.0", "kMap-0", WriteDat6V0, ReadDat6V0)

    //special constructors
    kAddPrivateFrameworkConstructor(kMap, ConstructFramework)

    //virtual methods
    kAddPrivateVMethod(kMap, kObject, VRelease)
    kAddPrivateVMethod(kMap, kObject, VDisposeItems)
    kAddPrivateVMethod(kMap, kObject, VClone)
    kAddPrivateVMethod(kMap, kObject, VHasShared)
    kAddPrivateVMethod(kMap, kObject, VSize)
    kAddPrivateVMethod(kMap, kObject, VAllocTraits)

kEndClassEx() 

kFx(kStatus) kMap_Construct(kMap* map, kType keyType, kType valueType, kSize initialCapacity, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback(allocator);
    kStatus status; 

    kCheck(kAlloc_GetObject(alloc, kTypeOf(kMap), map)); 

    if (!kSuccess(status = xkMap_Init(*map, kTypeOf(kMap), keyType, valueType, initialCapacity, alloc)))
    {
        kAlloc_FreeRef(alloc, map); 
    }

    return status; 
} 

kFx(kStatus) xkMap_ConstructFramework(kMap* map, kAlloc allocator)
{
    return kMap_Construct(map, kNULL, kNULL, 0, allocator);
}

kFx(kStatus) xkMap_Init(kMap map, kType classType, kType keyType, kType valueType, kSize initialCapacity, kAlloc alloc)
{
    kObjR(kMap, map); 
    kStatus status = kOK; 

    kCheck(kObject_Init(map, classType, alloc)); 

    obj->blocks = kNULL; 
    obj->free = kNULL; 
    obj->buckets = kNULL; 
    obj->bucketCount = 0; 
    obj->count = 0; 
    obj->capacity = 0; 
    kZero(obj->keyField); 
    kZero(obj->valueField); 
    obj->collectionItem = kNULL; 
    obj->hashFx = kNULL; 
    obj->equalsFx = kNULL; 
    
    kTry
    {
        kTest(xkMap_Layout(map, keyType, valueType)); 
        kTest(kMap_Reserve(map, initialCapacity));       
    }
    kCatch(&status)
    {
        xkMap_VRelease(map); 
        kEndCatch(status); 
    }

    return kOK; 
}

kFx(kStatus) xkMap_VClone(kMap map, kMap source, kAlloc valueAlloc, kObject context)
{ 
    kObj(kMap, map); 
    kObjN(kMap, sourceObj, source); 
    kAlloc objectAlloc = kObject_Alloc(map);
    kObject keyClone = kNULL; 
    kObject valueClone = kNULL; 
    kStatus status = kOK; 

    kCheck(kMap_Allocate(map, sourceObj->keyField.type, sourceObj->valueField.type, sourceObj->count)); 

    obj->hashFx = sourceObj->hashFx; 
    obj->equalsFx = sourceObj->equalsFx; 

    kTry
    {
        kMapItem item = kMap_First(source); 
        
        while (!kIsNull(item))
        {
            const void* key = kMap_Key(source, item); 
            void* value = kMap_Value(source, item); 
            
            if (xkMap_KeyIsRef(source))
            {
                kTest(kObject_Clone(&keyClone, *(kObject*)key, objectAlloc, valueAlloc, context)); 
                key = &keyClone; 
            }
            if (xkMap_ValueIsRef(source))
            {
                kTest(kObject_Clone(&valueClone, *(kObject*)value, objectAlloc, valueAlloc, context)); 
                value = &valueClone; 
            }

            kTest(kMap_Add(map, key, value)); 
            keyClone = valueClone = kNULL; 

            item = kMap_Next(source, item); 
        }
    }
    kCatch(&status)
    {
        kObject_Dispose(keyClone); 
        kObject_Dispose(valueClone); 
        kEndCatch(status); 
    }

    return kOK; 
}

kFx(kStatus) xkMap_VRelease(kMap map)
{
    kCheck(xkMap_Deallocate(map)); 

    kCheck(kObject_VRelease(map)); 

    return kOK; 
}

kFx(kStatus) kMap_Allocate(kMap map, kType keyType, kType valueType, kSize initialCapacity)
{
    kObj(kMap, map);   
    kHashFx hashFx = obj->hashFx; 
    kEqualsFx equalsFx = obj->equalsFx; 

    kCheck(xkMap_Deallocate(map)); 

    obj->free = kNULL; 
    obj->bucketCount = 0; 
    obj->count = 0; 
    obj->capacity = 0; 
    obj->keyField.type = kNULL; 
    obj->valueField.type = kNULL; 
    obj->collectionItem = kNULL; 
    obj->hashFx = kNULL; 
    obj->equalsFx = kNULL; 

    kCheck(xkMap_Layout(map, keyType, valueType)); 
    kCheck(kMap_Reserve(map, initialCapacity));   

    if (obj->keyField.type == keyType)
    {
        obj->hashFx = hashFx; 
        obj->equalsFx = equalsFx; 
    }

    return kOK; 
}

kFx(kStatus) kMap_Assign(kMap map, kMap source)
{
    kObj(kMap, map);   
    kObjN(kMap, sourceObj, source);
    kMapItem item  = kNULL; 

    kCheck(kMap_Allocate(map, sourceObj->keyField.type, sourceObj->valueField.type, sourceObj->count)); 

    obj->hashFx = sourceObj->hashFx; 
    obj->equalsFx = sourceObj->equalsFx; 

    item = kMap_First(source); 

    while (!kIsNull(item))
    {
        kCheck(kMap_Add(map, kMap_Key(source, item), kMap_Value(source, item))); 
        
        item = kMap_Next(source, item); 
    }

    return kOK; 
}

kFx(kStatus) xkMap_Layout(kMap map, kType keyType, kType valueType)
{
    kObj(kMap, map);   
    xkStructField nextField, hashCodeField; 
    xkStructField* fields[4];  

    nextField.type = kTypeOf(kPointer); 
    nextField.offset = offsetof(kMapItemStruct, next); 
    nextField.count = 1; 
    fields[0] = &nextField; 

    hashCodeField.type = kTypeOf(kSize); 
    hashCodeField.offset = offsetof(kMapItemStruct, hashCode); 
    hashCodeField.count = 1; 
    fields[1] = &hashCodeField; 

    obj->keyField.type = kIsNull(keyType) ? kTypeOf(kVoid) : keyType;
    obj->keyField.offset = kSIZE_NULL; 
    obj->keyField.count = 1; 
    fields[2] = &obj->keyField; 

    obj->valueField.type = kIsNull(valueType) ? kTypeOf(kVoid) : valueType;
    obj->valueField.offset = kSIZE_NULL; 
    obj->valueField.count = 1; 
    fields[3] = &obj->valueField;

    kCheck(xkType_LayoutStruct(fields, kCountOf(fields), &obj->itemSize)); 
              
    return kOK; 
}

kFx(kStatus) xkMap_Deallocate(kMap map)
{
    kObj(kMap, map);   

    while (!kIsNull(obj->blocks))
    {
        kMapItemBlock* next = obj->blocks->next; 

        kCheck(kAlloc_Free(kObject_Alloc(map), obj->blocks)); 

        obj->blocks = next; 
    }

    //note: buckets are considered to be part of "control" rather than "data" memory
    kCheck(kObject_FreeMemRef(map, &obj->buckets)); 

    return kOK; 
}

kFx(kBool) xkMap_VHasShared(kMap map)
{
    kObj(kMap, map);   

    if (kObject_IsShared(map))
    {
        return kTRUE; 
    }
    else 
    {
        kBool keyIsRef = kType_IsReference(obj->keyField.type); 
        kBool valIsRef = kType_IsReference(obj->valueField.type); 

        if (keyIsRef || valIsRef)
        {
            kMapItem item = kMap_First(map); 
            
            while (!kIsNull(item))
            {
                if (keyIsRef)   
                {
                    kObject key = kMap_KeyAsT(map, item, kObject); 
                    
                    if (!kIsNull(key) && kObject_HasShared(key))
                    {
                        return kTRUE; 
                    }
                }

                if (valIsRef)   
                {
                    kObject value = kMap_ValueAsT(map, item, kObject); 
                    
                    if (!kIsNull(value) && kObject_HasShared(value))
                    {
                        return kTRUE;
                    }
                }

                item = kMap_Next(map, item); 
            }
        }
    }

    return kFALSE;
}

kFx(kSize) xkMap_VSize(kMap map)
{
    kObj(kMap, map);   
    kSize size = sizeof(kMapClass);
    kMapItemBlock* blockObj = obj->blocks; 
    kBool keyIsRef = kType_IsReference(obj->keyField.type); 
    kBool valIsRef = kType_IsReference(obj->valueField.type); 

    while (!kIsNull(blockObj))
    {
        size += (kByte*)blockObj->items - (kByte*)blockObj; 
        size += obj->itemSize*blockObj->itemCount;

        blockObj = blockObj->next; 
    }

    size += sizeof(kPointer) * obj->bucketCount; 

    if (keyIsRef || valIsRef)
    {
        kMapItem item = kMap_First(map); 
        
        while (!kIsNull(item))
        {
            if (keyIsRef)   
            {
                kObject key = kMap_KeyAsT(map, item, kObject); 
                
                if (!kIsNull(key))
                {
                    size += kObject_Size(key); 
                }
            }

            if (valIsRef)   
            {
                kObject value = kMap_ValueAsT(map, item, kObject); 
                
                if (!kIsNull(value))
                {
                    size += kObject_Size(value); 
                }
            }

            item = kMap_Next(map, item); 
        }
    }

    return size; 
}

kFx(kAllocTrait) xkMap_VAllocTraits(kMap map)
{
    kObj(kMap, map);   
    kAllocTrait traits = kAlloc_Traits(kObject_Alloc(map));
    kBool keyIsRef = kType_IsReference(obj->keyField.type); 
    kBool valIsRef = kType_IsReference(obj->valueField.type); 

    if (keyIsRef || valIsRef)
    {
        kMapItem item = kMap_First(map); 
        
        while (!kIsNull(item))
        {
            if (keyIsRef)   
            {
                kObject key = kMap_KeyAsT(map, item, kObject); 
                
                if (!kIsNull(key))
                {
                    traits |= kObject_AllocTraits(key);
                }
            }

            if (valIsRef)   
            {
                kObject value = kMap_ValueAsT(map, item, kObject); 
                
                if (!kIsNull(value))
                {
                    traits |= kObject_AllocTraits(value);
                }
            }

            item = kMap_Next(map, item); 
        }
    }

    return traits;
}

kFx(kStatus) xkMap_WriteDat6V0(kMap map, kSerializer serializer)
{
    kObj(kMap, map);   
    kMapItem item = kMap_First(map); 
    kTypeVersion keyVersion, valueVersion; 

    kCheck(kSerializer_WriteType(serializer, obj->keyField.type, &keyVersion)); 
    kCheck(kSerializer_WriteType(serializer, obj->valueField.type, &valueVersion)); 
    kCheck(kSerializer_WriteSize(serializer, obj->count)); 

    while (!kIsNull(item))
    {
        kCheck(kSerializer_WriteItems(serializer, obj->keyField.type, keyVersion, kMap_Key(map, item), 1)); 
        kCheck(kSerializer_WriteItems(serializer, obj->valueField.type, valueVersion, kMap_Value(map, item), 1)); 

        item = kMap_Next(map, item); 
    }

    return kOK; 
}

kFx(kStatus) xkMap_ReadDat6V0(kMap map, kSerializer serializer)
{
    kObj(kMap, map);
    kMapItemStruct* itemObj = kNULL; 
    kType keyType, valueType; 
    kTypeVersion keyVersion, valueVersion; 
    kSize count = 0;  

    kCheck(kSerializer_ReadType(serializer, &keyType, &keyVersion)); 
    kCheck(kSerializer_ReadType(serializer, &valueType, &valueVersion)); 
    kCheck(kSerializer_ReadSize(serializer, &count)); 

    kCheck(kMap_Allocate(map, keyType, valueType, count)); 

    for (kSize i = 0; i < count; ++i)
    {
        itemObj = obj->free;
        obj->free = itemObj->next;
        itemObj->next = obj->buckets[0];
        obj->buckets[0] = itemObj;

        itemObj->hashCode = 0;
        kItemZero((void*)kMap_Key(map, itemObj), obj->keyField.fieldSize);
        kItemZero(kMap_Value(map, itemObj), obj->valueField.fieldSize);

        kCheck(kSerializer_ReadItems(serializer, obj->keyField.type, keyVersion, (void*)kMap_Key(map, itemObj), 1));
        kCheck(kSerializer_ReadItems(serializer, obj->valueField.type, valueVersion, kMap_Value(map, itemObj), 1));
    }

    obj->count = count;

    kCheck(xkMap_Rehash(map));

    return kOK; 
}

kFx(kStatus) kMap_Reserve(kMap map, kSize capacity)
{
    kObj(kMap, map);   
    
    if (capacity > obj->capacity)
    {
        kCheck(xkMap_ReserveItems(map, capacity)); 
        kCheck(xkMap_ReserveBuckets(map));  
    }

    return kOK; 
}

kFx(kStatus) xkMap_ReserveItems(kMap map, kSize capacity)
{
    kObj(kMap, map);   
    kSize itemCount = kMax_(capacity - obj->capacity, xkMAP_MIN_ITEMS_PER_BLOCK); 
    kSize itemOffset = kSize_Align(sizeof(kMapItemBlock), kALIGN_ANY); 
    kSize blockSize = kSize_Align(itemOffset + itemCount*obj->itemSize, kALIGN_ANY); 
    kMapItemBlock* block = kNULL; 
    kSize i; 
      
    kCheck(kAlloc_Get(kObject_Alloc(map), blockSize, &block));

    block->itemCount = itemCount; 
    block->items = (kMapItemStruct*) kPointer_ByteOffset(block, (kSSize)itemOffset); 

    block->next = obj->blocks; 
    obj->blocks = block; 

    for (i = 0; i < itemCount; ++i)
    {
        kMapItemStruct* item = (kMapItemStruct*) kPointer_ItemOffset(&block->items[0], (kSSize)i, obj->itemSize); 

        item->next = obj->free; 
        obj->free = item; 
    }

    obj->capacity += itemCount; 

    return kOK; 
}

kFx(kStatus) xkMap_ReserveBuckets(kMap map)
{
    kObj(kMap, map);   
    kSize minBucketCount = xkMAP_CAPACITY_TO_BUCKETS(obj->capacity); 
    kSize bucketCount = 1; 
    kMapItemStruct** buckets = kNULL; 
    kMapItemStruct** tempBuckets = kNULL; 
    kSize i; 

    kCheckArgs(minBucketCount < (1U << 31)); 

    while (bucketCount < minBucketCount)
    {
        bucketCount *= 2; 
    }

    //note: buckets are considered to be part of "control" rather than "data" memory
    kCheck(kObject_GetMemZero(map, bucketCount*sizeof(kPointer), &buckets)); 

    for (i = 0; i < obj->bucketCount; ++i)
    {
        kMapItemStruct* itemObj = obj->buckets[i]; 

        while (!kIsNull(itemObj))
        {
            kMapItemStruct* nextObj = itemObj->next; 
            kSize newBucketIndex = xkMap_HashToIndex(itemObj->hashCode, bucketCount); 
            
            itemObj->next = buckets[newBucketIndex]; 
            buckets[newBucketIndex] = itemObj; 
            
            itemObj = nextObj; 
        }  
    }

    tempBuckets = obj->buckets; 
    obj->buckets = buckets; 
    obj->bucketCount = bucketCount; 

    kCheck(kObject_FreeMem(map, tempBuckets)); 

    return kOK; 
}

kFx(kStatus) xkMap_Rehash(kMap map)
{
    kObj(kMap, map);   
    kMapItemStruct* items = kNULL; 
    kMapItemStruct* itemObj = kNULL; 
    kSize i; 

    for (i = 0; i < obj->bucketCount; ++i)
    {
        itemObj = obj->buckets[i]; 

        while (!kIsNull(itemObj))
        {
            kMapItemStruct* nextObj = itemObj->next; 

            itemObj->hashCode = xkMap_HashKey(map, kMap_Key(map, itemObj)); 
            itemObj->next = items; 
            items = itemObj; 
            
            itemObj = nextObj; 
        }  

        obj->buckets[i] = kNULL; 
    }

    itemObj = items; 

    while (!kIsNull(itemObj))
    {
        kMapItemStruct* nextObj = itemObj->next; 
        kSize bucketIndex = xkMap_BucketIndex(map, itemObj->hashCode); 

        itemObj->next = obj->buckets[bucketIndex]; 
        obj->buckets[bucketIndex] = itemObj; 

        itemObj = nextObj; 
    }  

    return kOK; 
}

kFx(kStatus) kMap_Find(kMap map, const void* key, void* value)
{
    kObj(kMap, map);   

    if (obj->bucketCount > 0)
    {
        kSize hashCode = xkMap_HashKey(map, key);
        kMapItemStruct* itemObj = obj->buckets[xkMap_BucketIndex(map, hashCode)]; 

        while (!kIsNull(itemObj))
        {
            if (xkMap_KeyEquals(map, key, kMap_Key(map, itemObj)))
            {
                //if the assertion below is encountered, then the map item has been corrupted
                kAssert(hashCode == itemObj->hashCode); 

                if (!kIsNull(value))
                {
                    kItemCopy(value, kMap_Value(map, itemObj), obj->valueField.fieldSize); 
                }

                return kOK; 
            }
            itemObj = itemObj->next; 
        }
    }

    return kERROR_NOT_FOUND;    
}

kFx(kStatus) kMap_Add(kMap map, const void* key, const void* value)
{
    kObj(kMap, map);   
    kMapItemStruct* itemObj = kNULL; 
    kSize hashCode = xkMap_HashKey(map, key); 
    kSize bucketIndex = 0; 
    void* keySlot = kNULL; 
    void* valueSlot = kNULL; 

    if (obj->bucketCount > 0)
    {
        bucketIndex = xkMap_BucketIndex(map, hashCode); 
        itemObj = obj->buckets[bucketIndex]; 

        while (!kIsNull(itemObj))
        {
            if (xkMap_KeyEquals(map, key, kMap_Key(map, itemObj)))
            {
                return kERROR_ALREADY_EXISTS; 
            }
            itemObj = itemObj->next; 
        }
    }

    if (obj->count == obj->capacity)
    {
        kCheck(kMap_Reserve(map, obj->capacity+1)); 
    }

    itemObj = obj->free; 
    obj->free = itemObj->next; 
    itemObj->next = kNULL; 

    bucketIndex = xkMap_BucketIndex(map, hashCode);

    itemObj->next = obj->buckets[bucketIndex]; 
    obj->buckets[bucketIndex] = itemObj; 

    itemObj->hashCode = hashCode; 

    obj->count++; 

    keySlot = (void*)kMap_Key(map, itemObj); 
    valueSlot = kMap_Value(map, itemObj); 

    kValue_Import(obj->keyField.type, keySlot, key); 
    kValue_Import(obj->valueField.type, valueSlot, value); 
    
    return kOK; 
}

kFx(kStatus) kMap_Replace(kMap map, const void* key, const void* value)
{
    kObj(kMap, map);   
    kMapItemStruct* itemObj = kNULL; 
    kSize hashCode = xkMap_HashKey(map, key); 
    kSize bucketIndex = 0; 
    void* keySlot = kNULL; 
    void* valueSlot = kNULL; 
    kBool found = kFALSE; 

    if (obj->bucketCount > 0)
    {
        bucketIndex = xkMap_BucketIndex(map, hashCode);
        itemObj = obj->buckets[bucketIndex]; 

        while (!kIsNull(itemObj))
        {
            if (xkMap_KeyEquals(map, key, kMap_Key(map, itemObj)))
            {
                found = kTRUE; 
                break;
            }
            itemObj = itemObj->next; 
        }
    }

    if (!found)
    {
        if (obj->count >= obj->capacity)
        {
            kCheck(kMap_Reserve(map, obj->capacity+1)); 
        }

        itemObj = obj->free; 
        obj->free = itemObj->next; 
        itemObj->next = kNULL; 

        bucketIndex = xkMap_BucketIndex(map, hashCode);

        itemObj->next = obj->buckets[bucketIndex]; 
        obj->buckets[bucketIndex] = itemObj; 

        itemObj->hashCode = hashCode; 

        obj->count++; 
    }

    keySlot = (void*)kMap_Key(map, itemObj); 
    valueSlot = kMap_Value(map, itemObj); 

    kValue_Import(obj->keyField.type, keySlot, key); 
    kValue_Import(obj->valueField.type, valueSlot, value); 
    
    return kOK; 
}

kFx(kStatus) kMap_Remove(kMap map, const void* key, void* oldKey, void* oldValue)
{
    kObj(kMap, map);   
    kMapItemStruct* previousObj = kNULL; 
    kMapItemStruct* itemObj = kNULL; 
    kSize hashCode = xkMap_HashKey(map, key);  
    kSize bucketIndex = 0; 

    if (obj->bucketCount > 0)
    {
        bucketIndex = xkMap_BucketIndex(map, hashCode); 
        itemObj = obj->buckets[bucketIndex]; 

        while (!kIsNull(itemObj))
        {
            if (xkMap_KeyEquals(map, key, kMap_Key(map, itemObj)))
            {            
                //if the assertion below is encountered, then the map item has been corrupted
                kAssert(hashCode == itemObj->hashCode); 

                if (oldKey)     kItemCopy(oldKey, kMap_Key(map, itemObj), obj->keyField.fieldSize); 
                if (oldValue)   kItemCopy(oldValue, kMap_Value(map, itemObj), obj->valueField.fieldSize); 

                if (kIsNull(previousObj))
                {
                    obj->buckets[bucketIndex] = itemObj->next; 
                }
                else
                {
                    previousObj->next = itemObj->next; 
                }

                itemObj->next = obj->free;              
                obj->free = itemObj; 

                obj->count--; 

                return kOK; 
            }

            previousObj = itemObj; 
            itemObj = itemObj->next; 
        }
    }

    return kERROR_NOT_FOUND; 
}

kFx(kStatus) kMap_Clear(kMap map)
{
    kObj(kMap, map);   
    kSize i; 

    if (obj->count > 0)
    {
        for (i = 0; i < obj->bucketCount; ++i)
        {
            kMapItemStruct* itemObj = obj->buckets[i]; 

            while (!kIsNull(itemObj))
            {
                kMapItemStruct* nextObj = itemObj->next; 

                itemObj->next = obj->free; 
                obj->free = itemObj; 

                itemObj = nextObj; 
            }

            obj->buckets[i] = kNULL; 
        }

        obj->count = 0; 
    }

    return kOK; 
}

kFx(kStatus) kMap_Purge(kMap map)
{
    kObj(kMap, map);   

    if (!kIsNull(obj->keyField.type) && !kIsNull(obj->valueField.type))
    {
        kBool keyIsRef = kType_IsReference(obj->keyField.type); 
        kBool valIsRef = kType_IsReference(obj->valueField.type); 

        if (keyIsRef || valIsRef)
        {
            kMapItem item = kMap_First(map); 

            while (!kIsNull(item))
            {
                if (keyIsRef)   kCheck(kObject_Dispose(kMap_KeyAsT(map, item, kObject))); 
                if (valIsRef)   kCheck(kObject_Dispose(kMap_ValueAsT(map, item, kObject))); 

                item = kMap_Next(map, item); 
            }
        }
    }

    kCheck(kMap_Clear(map)); 

    return kOK; 
}

kFx(kMapItem) kMap_First(kMap map)
{
    kObj(kMap, map);   
    kSize i; 

    for (i = 0; i < obj->bucketCount; ++i)
    {
        if (!kIsNull(obj->buckets[i]))
        {
            return obj->buckets[i]; 
        }
    }

    return kNULL; 
}

kFx(kMapItem) kMap_Next(kMap map, kMapItem item)
{
    kObj(kMap, map);   
    kMapItemStruct* itemObj = (kMapItemStruct*) item; 

    if (!kIsNull(itemObj->next))
    {
        return itemObj->next; 
    }
    else
    {
        kSize hashCode = xkMapItem_HashCode(map, item); 
        kSize bucketIndex = xkMap_BucketIndex(map, hashCode); 
        kSize i; 

        for (i = bucketIndex+1; i < obj->bucketCount; ++i)
        {
            if (!kIsNull(obj->buckets[i]))
            {
                return obj->buckets[i]; 
            }
        }
    }
    
    return kNULL; 
}

kFx(kStatus) kMap_FindItem(kMap map, const void* key, kMapItem* item)
{
    kObj(kMap, map);   
    kSize hashCode = xkMap_HashKey(map, key); 

    if (obj->bucketCount > 0)
    {
        kMapItemStruct* itemObj = obj->buckets[xkMap_BucketIndex(map, hashCode)]; 

        while (!kIsNull(itemObj))
        {
            if (xkMap_KeyEquals(map, key, kMap_Key(map, itemObj)))
            {
                //if the assertion below is encountered, then the map item has been corrupted
                kAssert(hashCode == itemObj->hashCode); 

                *item = itemObj; 
                return kOK; 
            }
            itemObj = itemObj->next; 
        }
    }

    return kERROR_NOT_FOUND;    
}

kFx(kStatus) kMap_RemoveItem(kMap map, kMapItem item)
{
    kObj(kMap, map);   
    kMapItemStruct* previousObj = kNULL; 
    kMapItemStruct* itemObj = kNULL; 
    kSize hashCode = xkMapItem_HashCode(map, item); 
    kSize bucketIndex = 0; 
    kBool found = kFALSE; 

    if (obj->bucketCount > 0)
    {
        bucketIndex = xkMap_BucketIndex(map, hashCode); 
        itemObj = obj->buckets[bucketIndex]; 

        while (!kIsNull(itemObj))
        {
            if (itemObj == item)
            {
                found = kTRUE;                        

                if (kIsNull(previousObj))
                {
                    obj->buckets[bucketIndex] = itemObj->next; 
                }
                else
                {
                    previousObj->next = itemObj->next; 
                }

                itemObj->next = obj->free;              
                obj->free = itemObj; 

                obj->count--; 

                break;
            }

            previousObj = itemObj; 
            itemObj = itemObj->next; 
        }
    }

    return (found) ? kOK : kERROR_NOT_FOUND; 
}

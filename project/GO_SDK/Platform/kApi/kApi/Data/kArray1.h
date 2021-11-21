/** 
 * @file    kArray1.h
 * @brief   Declares the kArray1 class. 
 *
 * @internal
 * Copyright (C) 2005-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_ARRAY_1_H
#define K_API_ARRAY_1_H

#include <kApi/kApiDef.h>
#include <kApi/Data/kArray1.x.h>

/**
 * @class       kArray1
 * @extends     kObject
 * @implements  kCollection
 * @implements  kArrayProvider
 * @ingroup     kApi-Data
 * @brief       Represents a 1D array.
 * 
 * The kArray1 class represents a 1D array of objects or values. The kArray1 constructor accepts arguments
 * that determine the array item type (kType) and array length. 
 * 
 * For arrays that contain <em>objects</em> (e.g. kImage) as opposed to <em>values</em> (e.g. k32s), the objects 
 * are not automatically destroyed when the array is destroyed. To recursively destroy both the array and the 
 * array items, use kObject_Dispose. 
 * 
 * kArray1 supports the kObject_Clone, kObject_Dispose, and kObject_Size methods.
 * 
 * kArray1 supports the kdat5 and kdat6 serialization protocols. 
 */
//typedef kObject kArray1;   --forward-declared in kApiDef.x.h 

/** 
 * Constructs a kArray1 object.
 *
 * @public              @memberof kArray1
 * @param   array       Receives the constructed object.  
 * @param   itemType    Type of array element.
 * @param   length      Length of array.
 * @param   allocator   Memory allocator (or kNULL for default). 
 * @return              Operation status. 
 */
kFx(kStatus) kArray1_Construct(kArray1* array, kType itemType, kSize length, kAlloc allocator);

#if defined(K_CPP)
/**
 * Constructs a kArray1 object using a separate allocator for data array memory.
 *
 * @public                      @memberof kArray1
 * @param   array               Receives the constructed object.
 * @param   itemType            Type of array element.
 * @param   length              Length of array.
 * @param   allocator           Primary memory allocator (or kNULL for default).
 * @param   valueAllocator      Data array allocator (or kNULL for default).
 * @param   valueAlignment      Memory alignment.
 * @return                      Operation status.
 */
kFx(kStatus) kArray1_ConstructEx(kArray1* array, kType itemType, kSize length, kAlloc allocator, kAlloc valueAllocator, kMemoryAlignment valueAlignment = kALIGN_ANY);
#endif

/** 
 * Reallocates the internal array item buffer. 
 *
 * @public              @memberof kArray1
 * @param   array       Array object. 
 * @param   itemType    Type of array element.
 * @param   length      Length of array.
 * @return              Operation status. 
 */
kFx(kStatus) kArray1_Allocate(kArray1 array, kType itemType, kSize length); 

/** 
 * Resizes the internal array item buffer. 
 *
 * Note: at present, this method does not preserve existing array elements. Element preservation 
 * may be added in a future release.
 * 
 * @public              @memberof kArray1
 * @param   array       Array object. 
 * @param   length      Length of array.
 * @return              Operation status. 
 */
kInlineFx(kStatus) kArray1_Resize(kArray1 array, kSize length)
{
    return kArray1_Allocate(array, kArray1_ItemType(array), length); 
}

/** 
 * Attaches the array to an external item buffer. 
 *
 * Attached item buffers are not freed when the array is destroyed. 
 *
 * @public              @memberof kArray1
 * @param   array       Array object. 
 * @param   items       External item buffer. 
 * @param   itemType    Type of array element.
 * @param   length      Length of array.
 * @return              Operation status. 
 */
kFx(kStatus) kArray1_Attach(kArray1 array, void* items, kType itemType, kSize length); 

/** 
 * Attaches the array to an external item buffer. 
 *
 * Attached item buffers are not freed when the array is destroyed. 
 *
 * A debug assertion will be raised if the size of the specified items is not equal to the 
 * size of the specified item type.
 * 
 * @relates                 kArray1
 * @param   kArray1_array   Array object. 
 * @param   TPtr_items      Strongly-typed pointer to external item buffer. 
 * @param   kType_itemType  Type of array element.
 * @param   kSize_length    Length of array.
 * @return                  Operation status. 
 */
#define kArray1_AttachT(kArray1_array, TPtr_items, kType_itemType, kSize_length) \
    xkArray1_AttachT(kArray1_array, TPtr_items, kType_itemType, kSize_length, sizeof(*(TPtr_items)))

/** 
 * Performs a shallow copy of the source array.  
 *
 * Source items are copied by value; if the source array contains objects, the object 
 * handles are copied but the objects are not cloned. 
 *
 * @public              @memberof kArray1
 * @param   array       Array object. 
 * @param   source      Source array to be copied. 
 * @return              Operation status. 
 */
kInlineFx(kStatus) kArray1_Assign(kArray1 array, kArray1 source)
{
    return xkArray1_Assign(array, source, kNULL); 
}

/** 
 * Performs a shallow copy of the source array.  
 *
 * Source items are copied by value; if the source array contains objects, the object 
 * handles are copied but the objects are not cloned. 
 *
 * @public              @memberof kArray1
 * @param   array       Array object. 
 * @param   source      Source array to be copied. 
 * @param   context     Context for copy operation (allocator specific; not usually required).
 * @return              Operation status. 
 */
#if defined (K_CPP)
kInlineFx(kStatus) kArray1_Assign(kArray1 array, kArray1 source, kObject context)
{
    return xkArray1_Assign(array, source, context); 
}
#endif 

/** 
 * Sets all array element bits to zero.
 *
 * @public              @memberof kArray1
 * @param   array       Array object. 
 * @return              Operation status. 
 */
kFx(kStatus) kArray1_Zero(kArray1 array); 

/** 
 * Sets the value of an item. 
 *
 * @public              @memberof kArray1
 * @param   array       Array object. 
 * @param   index       Array item index. 
 * @param   item        Pointer to item that will be copied (by value) into the array.
 * @return              Operation status. 
 */
kFx(kStatus) kArray1_SetItem(kArray1 array, kSize index, const void* item); 

/** 
 * Sets the value of an item. 
 *
 * A debug assertion will be raised if the size of the specified item type is not equal to the 
 * size of the collection item type.
 * 
 * @relates                 kArray1
 * @param   kArray1_array   Array object. 
 * @param   kSize_index     Array item index. 
 * @param   TPtr_item       Strongly-typed pointer to item that will be copied (by value) into the array.
 * @return                  Operation status. 
 */
#define kArray1_SetItemT(kArray1_array, kSize_index, TPtr_item) \
    xkArray1_SetItemT(kArray1_array, kSize_index, TPtr_item, sizeof(*(TPtr_item)))

/** 
 * Gets the value of an item. 
 *
 * @public              @memberof kArray1
 * @param   array       Array object. 
 * @param   index       Array item index. 
 * @param   item        Destination for item that will be copied (by value) from the array. 
 * @return              Operation status. 
 */
kFx(kStatus) kArray1_Item(kArray1 array, kSize index, void* item); 

/** 
 * Gets the value of an item. 
 *
 * A debug assertion will be raised if the size of the specified item type is not equal to the 
 * size of the collection item type.
 * 
 * @relates                 kArray1
 * @param   kArray1_array   Array object. 
 * @param   kSize_index     Array item index. 
 * @param   TPtr_item       Strongly-typed destination pointer for item that will be copied (by value) from the array. 
 * @return                  Operation status. 
 */
#define kArray1_ItemT(kArray1_array, kSize_index, TPtr_item) \
    xkArray1_ItemT(kArray1_array, kSize_index, TPtr_item, sizeof(*(TPtr_item)))

/** 
* Sets the value of an item. 
*
* A debug assertion will be raised if the index argument is greater than or equal to 
* the current array length, or if the size of the specified item type is not equal to the 
* size of the collection item type.
* 
* @relates                      kArray1 @
* @param   kArray1_array        Array object. 
* @param   kSize_index          Item index. 
* @param   T_value              Item value.  
* @param   T                    Item type identifier (e.g., k32s).
*/
#define kArray1_SetAsT(kArray1_array, kSize_index, T_value, T) \
    (kPointer_WriteAs(xkArray1_AsT(kArray1_array, kSize_index, sizeof(T)), T_value, T), (void)0)

/** 
* Gets the value of an item. 
*
* A debug assertion will be raised if the index argument is greater than or equal to 
* the current array length, or if the size of the specified item type is not equal to the 
* size of the collection item type.
* 
* @relates                      kArray1 @
* @param   kArray1_array        Array object. 
* @param   kSize_index          Item index. 
* @param   T                    Item type identifier (e.g., k32s).
* @return                       Item value. 
*/
#define kArray1_AsT(kArray1_array, kSize_index, T) \
    kPointer_ReadAs(xkArray1_AsT(kArray1_array, kSize_index, sizeof(T)), T)

/** 
 * Returns a pointer to the array item buffer.  
 *
 * @public              @memberof kArray1
 * @param   array       Array object. 
 * @return              Pointer to array item buffer. 
 */
kInlineFx(void*) kArray1_Data(kArray1 array)
{
    kObj(kArray1, array);
    
    return obj->items;
}

/** 
 * Returns a strongly-typed pointer to the array item buffer.  
 *
 * A debug assertion will be raised if the size of the specified item type is not equal to the 
 * size of the collection item type.
 * 
 * @relates                 kArray1
 * @param   kArray1_array   Array object. 
 * @param   T               Item type identifier (e.g., k32s).
 * @return                  Strongly-typed pointer to array item buffer. 
 */
#define kArray1_DataT(kArray1_array, T) \
    kCast(T*, xkArray1_DataT(kArray1_array, sizeof(T)))

/** 
 * Calculates an address relative to the start of the buffer.
 *
 * This method is similar to the At method, but does not bounds-check the index argument. 
 * 
 * @public              @memberof kArray1
 * @param   array       Array object. 
 * @param   index       Item index. 
 * @return              Calculated pointer. 
 */
kInlineFx(void*) kArray1_DataAt(kArray1 array, kSSize index)
{
    kObj(kArray1, array);
   
    return kPointer_ItemOffset(obj->items, index, obj->itemSize);
}

/** 
 * Calculates an address relative to the start of the buffer.
 *
 * This method is similar to the At method, but does not bounds-check the index argument. 
 * 
 * A debug assertion will be raised if the size of the specified item type is not equal to the 
 * size of the collection item type.
 * 
 * @relates                 kArray1
 * @param   kArray1_array   Array object. 
 * @param   kSSize_index    Item index. 
 * @param   T               Item type identifier (e.g., k32s).
 * @return                  Calculated pointer. 
 */
#define kArray1_DataAtT(kArray1_array, kSSize_index, T) \
    kCast(T*, xkArray1_DataAtT(kArray1_array, kSSize_index, sizeof(T)))

/** 
 * Reports the size, in bytes, of the array item buffer. 
 *
 * @public              @memberof kArray1
 * @param   array       Array object. 
 * @return              Size of array item buffer (bytes). 
 */
kInlineFx(kSize) kArray1_DataSize(kArray1 array)
{
    kObj(kArray1, array);

    return obj->length * obj->itemSize;
}

/** 
 * Returns a pointer to the specified item in the array. 
 *
 * A debug assertion will be raised if the index argument is greater than or equal to 
 * the current array Length.  
 * 
 * @public              @memberof kArray1
 * @param   array       Array object. 
 * @param   index       Item index. 
 * @return              Pointer to item. 
 */
kInlineFx(void*) kArray1_At(kArray1 array, kSize index)
{    
#   if !defined(K_FSS_912_DISABLE_BOUNDS_CHECK)
    {
        kAssert(index < kArray1_Length(array));
    }
#   endif

    return kArray1_DataAt(array, (kSSize)index);
}

/** 
 * Returns a strongly-typed pointer to the specified item in the array. 
 *
 * A debug assertion will be raised if the index argument is greater than or equal to 
 * the current array length, or if the size of the specified item type is not equal to the 
 * size of the collection item type.  
 * 
 * @relates                 kArray1
 * @param   kArray1_array   Array object. 
 * @param   kSize_index     Item index. 
 * @param   T               Item type identifier (e.g., k32s).
 * @return                  Strongly-typed pointer to item. 
 */
#define kArray1_AtT(kArray1_array, kSize_index, T) \
    kCast(T*, xkArray1_AtT(kArray1_array, kSize_index, sizeof(T)))

/** 
 * Returns the array item type. 
 *
 * @public              @memberof kArray1
 * @param   array       Array object. 
 * @return              Array item type. 
 */
kInlineFx(kType) kArray1_ItemType(kArray1 array)
{
    kObj(kArray1, array);

    return obj->itemType;
}

/** 
 * Returns the array item size. 
 *
 * @public              @memberof kArray1
 * @param   array       Array object. 
 * @return              Array item size. 
 */
kInlineFx(kSize) kArray1_ItemSize(kArray1 array)
{
    kObj(kArray1, array);

    return obj->itemSize;
}

/** 
 * Returns the array length, in elements. 
 *
 * @public              @memberof kArray1
 * @param   array       Array object. 
 * @return              Array length (in elements). 
 */
kInlineFx(kSize) kArray1_Length(kArray1 array)
{
    kObj(kArray1, array);

    return obj->length;
}

/** 
 * Returns the array item count, in elements. 
 *
 * This method is provided for symmetry with kArray2/kArray3.  In practice, 
 * the item count for a 1D array is always equal to the length. 
 * 
 * @public              @memberof kArray1
 * @param   array       Array object. 
 * @return              Array item count (in elements). 
 */
kInlineFx(kSize) kArray1_Count(kArray1 array)
{
    kObj(kArray1, array);

    return obj->length;
}

/** 
 * Reports the allocator used for the internal data array.
 *
 * @public          @memberof kArray1
 * @param   array   Array object. 
 * @return          Data array allocator.
 */
kInlineFx(kAlloc) kArray1_DataAlloc(kArray1 array)
{
    kObj(kArray1, array);

    return kType_IsValue(obj->itemType) ? obj->valueAlloc : kObject_Alloc(array); 
}

#endif

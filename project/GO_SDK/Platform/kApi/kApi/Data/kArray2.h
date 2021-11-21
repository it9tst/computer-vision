/** 
 * @file    kArray2.h
 * @brief   Declares the kArray2 class. 
 *
 * @internal
 * Copyright (C) 2005-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_ARRAY_2_H
#define K_API_ARRAY_2_H

#include <kApi/kApiDef.h>
#include <kApi/Data/kArray2.x.h>

/**
 * @class       kArray2
 * @extends     kObject
 * @implements  kCollection
 * @implements  kArrayProvider
 * @ingroup     kApi-Data
 * @brief       Represents a 2D array.
 * 
 * The kArray2 class represents a 2D array of objects or values. The kArray2 constructor accepts arguments
 * that determine the array item type (kType) and array dimension lengths. 
 * 
 * For arrays that contain <em>objects</em> (e.g. kImage) as opposed to <em>values</em> (e.g. k32s), the objects 
 * are not automatically destroyed when the array is destroyed. To recursively destroy both the array and the 
 * array items, use kObject_Dispose. 
 * 
 * kArray2 supports the kObject_Clone, kObject_Dispose, and kObject_Size methods.
 *
 * kArray2 supports the kdat5 and kdat6 serialization protocols.
 */
//typedef kObject kArray2;   --forward-declared in kApiDef.x.h  

/**
 * Constructs a kArray2 object.
 *
 * @public                  @memberof kArray2
 * @param   array           Receives the constructed object.  
 * @param   itemType        Type of array element.
 * @param   length0         Length of first array dimension (outermost). 
 * @param   length1         Length of second array dimension (innermost). 
 * @param   allocator       Memory allocator (or kNULL for default). 
 * @return                  Operation status.
 */
kFx(kStatus) kArray2_Construct(kArray2* array, kType itemType, kSize length0, kSize length1, kAlloc allocator);

#if defined(K_CPP)
/**
 * Constructs a kArray2 object using a separate allocator for data array memory.
 *
 * @public                  @memberof kArray2
 * @param   array           Receives the constructed object.  
 * @param   itemType        Type of array element.
 * @param   length0         Length of first array dimension (outermost). 
 * @param   length1         Length of second array dimension (innermost). 
 * @param   allocator       Primary memory allocator (or kNULL for default). 
 * @param   valueAllocator  Data array allocator (or kNULL for default).
 * @param   valueAlignment  Memory alignment for the start address of values. 
 * @return                  Operation status. 
 */
kFx(kStatus) kArray2_ConstructEx(kArray2* array, kType itemType, kSize length0, kSize length1, kAlloc allocator, kAlloc valueAllocator, kMemoryAlignment valueAlignment = kALIGN_ANY);
#endif

/** 
 * Reallocates the internal array item buffer. 
 *
 * @public              @memberof kArray2
 * @param   array       Array object. 
 * @param   itemType    Type of array element.
 * @param   length0     Length of first array dimension (outermost). 
 * @param   length1     Length of second array dimension (innermost). 
 * @return              Operation status. 
 */
kFx(kStatus) kArray2_Allocate(kArray2 array, kType itemType, kSize length0, kSize length1); 

/** 
 * Resizes the internal array item buffer. 
 *
 * Note: at present, this method does not preserve existing array elements. Element preservation 
 * may be added in a future release.
 * 
 * @public              @memberof kArray2
 * @param   array       Array object. 
 * @param   length0     Length of first array dimension (outermost). 
 * @param   length1     Length of second array dimension (innermost). 
 * @return              Operation status. 
 */
kInlineFx(kStatus) kArray2_Resize(kArray2 array, kSize length0, kSize length1)
{
    return kArray2_Allocate(array, kArray2_ItemType(array), length0, length1); 
}

/** 
 * Attaches the array to an external item buffer. 
 *
 * Attached item buffers are not freed when the array is destroyed. 
 *
 * @public              @memberof kArray2
 * @param   array       Array object. 
 * @param   items       External item buffer. 
 * @param   itemType    Type of array element.
 * @param   length0     Length of first array dimension (outermost). 
 * @param   length1     Length of second array dimension (innermost). 
 * @return              Operation status. 
 */
kFx(kStatus) kArray2_Attach(kArray2 array, void* items, kType itemType, kSize length0, kSize length1); 

/** 
 * Attaches the array to an external item buffer. 
 *
 * Attached item buffers are not freed when the array is destroyed. 
 *
 * A debug assertion will be raised if the size of the specified items is not equal to the 
 * size of the specified item type.
 * 
 * @relates                     kArray2
 * @param   kArray2_array       Array object. 
 * @param   TPtr_items          Strongly-typed pointer to external item buffer. 
 * @param   kType_itemType      Type of array element.
 * @param   kSize_length0       Length of first array dimension (outermost). 
 * @param   kSize_length1       Length of second array dimension (innermost). 
 * @return                      Operation status. 
 */
#define kArray2_AttachT(kArray2_array, TPtr_items, kType_itemType, kSize_length0, kSize_length1) \
    xkArray2_AttachT(kArray2_array, TPtr_items, kType_itemType, kSize_length0, kSize_length1, sizeof(*(TPtr_items)))

/** 
 * Performs a shallow copy of the source array.  
 *
 * Source items are copied by value; if the source array contains objects, the object 
 * handles are copied but the objects are not cloned. 
 *
 * @public              @memberof kArray2
 * @param   array       Array object. 
 * @param   source      Source array to be copied.
 * @return              Operation status. 
 */
kInlineFx(kStatus) kArray2_Assign(kArray2 array, kArray2 source)
{
    return xkArray2_Assign(array, source, kNULL);
}

/** 
 * Performs a shallow copy of the source array.  
 *
 * Source items are copied by value; if the source array contains objects, the object 
 * handles are copied but the objects are not cloned. 
 *
 * @public              @memberof kArray2
 * @param   array       Array object. 
 * @param   source      Source array to be copied.
 * @param   context     Context for copy operation (allocator specific; not usually required).
 * @return              Operation status. 
 */
#if defined (K_CPP)
kInlineFx(kStatus) kArray2_Assign(kArray2 array, kArray2 source, kObject context)
{
    return xkArray2_Assign(array, source, context);
}
#endif

/** 
 * Sets all array element bits to zero.
 *
 * @public              @memberof kArray2
 * @param   array       Array object. 
 * @return              Operation status. 
 */
kFx(kStatus) kArray2_Zero(kArray2 array); 

/** 
 * Sets the value of an item. 
 *
 * @public              @memberof kArray2
 * @param   array       Array object. 
 * @param   index0      First dimension index. 
 * @param   index1      Second dimension index. 
 * @param   item        Pointer to item that will be copied (by value) into the array.
 * @return              Operation status. 
 */
kFx(kStatus) kArray2_SetItem(kArray2 array, kSize index0, kSize index1, const void* item); 

/** 
 * Sets the value of an item. 
 *
 * A debug assertion will be raised if the size of the specified item type is not equal to the 
 * size of the collection item type.
 * 
 * @relates                 kArray2
 * @param   kArray2_array   Array object. 
 * @param   kSize_index0    First dimension index. 
 * @param   kSize_index1    Second dimension index. 
 * @param   TPtr_item       Strongly-typed pointer to item that will be copied (by value) into the array.
 * @return                  Operation status. 
 */
#define kArray2_SetItemT(kArray2_array, kSize_index0, kSize_index1, TPtr_item) \
    xkArray2_SetItemT(kArray2_array, kSize_index0, kSize_index1, TPtr_item, sizeof(*(TPtr_item)))

/** 
 * Gets the value of an item. 
 *
 * @public              @memberof kArray2
 * @param   array       Array object. 
 * @param   index0      First dimension index. 
 * @param   index1      Second dimension index. 
 * @param   item        Destination for item that will be copied (by value) from the array. 
 * @return              Operation status. 
 */
kFx(kStatus) kArray2_Item(kArray2 array, kSize index0, kSize index1, void* item); 

/** 
 * Gets the value of an item. 
 *
 * A debug assertion will be raised if the size of the specified item type is not equal to the 
 * size of the collection item type.
 * 
 * @relates                 kArray2
 * @param   kArray2_array   Array object. 
 * @param   kSize_index0    First dimension index. 
 * @param   kSize_index1    Second dimension index. 
 * @param   TPtr_item       Strongly-typed destination pointer for item that will be copied (by value) from the array. 
 * @return                  Operation status. 
 */
#define kArray2_ItemT(kArray2_array, kSize_index0, kSize_index1, TPtr_item) \
    xkArray2_ItemT(kArray2_array, kSize_index0, kSize_index1, TPtr_item, sizeof(*(TPtr_item)))

/** 
* Sets the value of an item. 
* 
* A debug assertion will be raised if the index arguments are greater than or equal to 
* the current array lengths, or if the size of the specified item type is not equal to the 
* size of the collection item type.
* 
* @relates                  kArray2
* @param   kArray2_array    Array object. 
* @param   kSize_index0     First dimension index. 
* @param   kSize_index1     Second dimension index. 
* @param   T_value          Item value.  
* @param   T                Item type identifier (e.g., k32s).  
*/
#define kArray2_SetAsT(kArray2_array, kSize_index0, kSize_index1, T_value, T) \
    (kPointer_WriteAs(xkArray2_AsT(kArray2_array, kSize_index0, kSize_index1, sizeof(T)), T_value, T), (void)0)

/** 
* Gets the value of an item. 
* 
* A debug assertion will be raised if the index arguments are greater than or equal to 
* the current array lengths, or if the size of the specified item type is not equal to the 
* size of the collection item type.
* 
* @relates                  kArray2
* @param   kArray2_array    Array object. 
* @param   kSize_index0     First dimension index. 
* @param   kSize_index1     Second dimension index. 
* @param   T                Item type identifier (e.g., k32s).  
* @return                   Item value. 
*/
#define kArray2_AsT(kArray2_array, kSize_index0, kSize_index1, T) \
    kPointer_ReadAs(xkArray2_AsT(kArray2_array, kSize_index0, kSize_index1, sizeof(T)), T)

/** 
 * Returns a pointer to the array item buffer.  
 *
 * @public              @memberof kArray2
 * @param   array       Array object. 
 * @return              Pointer to array item buffer. 
 */
kInlineFx(void*) kArray2_Data(kArray2 array)
{
    kObj(kArray2, array);

    return obj->items; 
}

/** 
 * Returns a strongly-typed pointer to the array item buffer.  
 *
 * A debug assertion will be raised if the size of the specified item type is not equal to the 
 * size of the collection item type.
 * 
 * @relates                 kArray2
 * @param   kArray2_array   Array object. 
 * @param   T               Item type identifier (e.g., k32s).
 * @return                  Strongly-typed pointer to array item buffer. 
 */
#define kArray2_DataT(kArray2_array, T) \
    kCast(T*, xkArray2_DataT(kArray2_array, sizeof(T)))

/** 
 * Calculates an address relative to the start of the buffer.
 *
 * This method is similar to the At method, but does not bounds-check the index arguments. 
 *
 * @public              @memberof kArray2
 * @param   array       Array object. 
 * @param   index0      First dimension index. 
 * @param   index1      Second dimension index. 
 * @return              Calculated pointer.
 */
kInlineFx(void*) kArray2_DataAt(kArray2 array, kSSize index0, kSSize index1)
{
    kObj(kArray2, array);
    kSSize index = index0*(kSSize)obj->length[1] + index1;

    return kPointer_ItemOffset(obj->items, index, obj->itemSize);
}

/** 
 * Calculates an address relative to the start of the buffer.
 *
 * This method is similar to the At method, but does not bounds-check the index arguments. 
 *
 * A debug assertion will be raised if the size of the specified item type is not equal to the 
 * size of the collection item type.
 * 
 * @relates                 kArray2
 * @param   kArray2_array   Array object. 
 * @param   kSSize_index0   First dimension index. 
 * @param   kSSize_index1   Second dimension index. 
 * @param   T               Item type identifier (e.g., k32s).
 * @return                  Calculated pointer. 
 */
#define kArray2_DataAtT(kArray2_array, kSSize_index0, kSSize_index1, T) \
    kCast(T*, xkArray2_DataAtT(kArray2_array, kSSize_index0, kSSize_index1, sizeof(T)))

/** 
 * Reports the size, in bytes, of the array item buffer. 
 *
 * @public              @memberof kArray2
 * @param   array       Array object. 
 * @return              Size of array item buffer (bytes). 
 */
kInlineFx(kSize) kArray2_DataSize(kArray2 array)
{
    kObj(kArray2, array);

    return kArray2_Count(array) * obj->itemSize;
}

/** 
 * Returns a pointer to the specified item in the array. 
 *
 * A debug assertion will be raised if the index arguments are greater than or equal to 
 * the current array dimensions. 
 * 
 * @public              @memberof kArray2
 * @param   array       Array object. 
 * @param   index0      First dimension index. 
 * @param   index1      Second dimension index. 
 * @return              Pointer to item. 
 */
kInlineFx(void*) kArray2_At(kArray2 array, kSize index0, kSize index1)
{    
#   if !defined(K_FSS_912_DISABLE_BOUNDS_CHECK)
    {
        kAssert(index0 < kArray2_Length(array, 0));
        kAssert(index1 < kArray2_Length(array, 1));
    }
#   endif

    return kArray2_DataAt(array, (kSSize)index0, (kSSize)index1);
}

/** 
 * Returns a pointer to the specified item in the array. 
 *
 * A debug assertion will be raised if the index arguments are greater than or equal to 
 * the current array dimensions, or if the size of the specified item type is not equal to the 
 * size of the collection item type.
 * 
 * @relates                kArray2
 * @param   kArray2_array  Array object. 
 * @param   kSize_index0   First dimension index. 
 * @param   kSize_index1   Second dimension index. 
 * @param   T              Item type identifier (e.g., k32s).
 * @return                 Strongly-typed pointer to item. 
 */
#define kArray2_AtT(kArray2_array, kSize_index0, kSize_index1, T) \
    kCast(T*, xkArray2_AtT(kArray2_array, kSize_index0, kSize_index1, sizeof(T)))

/** 
 * Returns the array item type. 
 *
 * @public              @memberof kArray2
 * @param   array       Array object. 
 * @return              Array item type. 
 */
kInlineFx(kType) kArray2_ItemType(kArray2 array)
{
    kObj(kArray2, array);

    return obj->itemType;
}

/** 
 * Returns the array item size. 
 *
 * @public              @memberof kArray2
 * @param   array       Array object. 
 * @return              Array item size. 
 */
kInlineFx(kSize) kArray2_ItemSize(kArray2 array)
{
    kObj(kArray2, array);

    return obj->itemSize; 
}

/** 
 * Returns the length of the specified array dimension, in elements. 
 *
 * @public              @memberof kArray2
 * @param   array       Array object. 
 * @param   dimension   Array dimension index. 
 * @return              Array dimension length (in elements). 
 */
kInlineFx(kSize) kArray2_Length(kArray2 array, kSize dimension)
{
    kObj(kArray2, array);

    kAssert(dimension < 2); 

    return obj->length[dimension];
}

/** 
 * Returns the array item count, in elements. 
 *
 * @public              @memberof kArray2
 * @param   array       Array object. 
 * @return              Array item count (in elements). 
 */
kInlineFx(kSize) kArray2_Count(kArray2 array)
{
    kObj(kArray2, array);

    return obj->length[0] * obj->length[1];
}

/** 
 * Reports the allocator used for the internal data array.
 *
 * @public          @memberof kArray2
 * @param   array   Array object. 
 * @return          Data array allocator.
 */
kInlineFx(kAlloc) kArray2_DataAlloc(kArray2 array)
{
    kObj(kArray2, array);

    return kType_IsValue(obj->itemType) ? obj->valueAlloc : kObject_Alloc(array); 
}

#endif

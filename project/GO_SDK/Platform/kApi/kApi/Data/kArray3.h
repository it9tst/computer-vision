/** 
 * @file    kArray3.h
 * @brief   Declares the kArray3 class. 
 *
 * @internal
 * Copyright (C) 2006-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_ARRAY_3_H
#define K_API_ARRAY_3_H

#include <kApi/kApiDef.h>
#include <kApi/Data/kArray3.x.h>

/**
 * @class       kArray3
 * @extends     kObject
 * @implements  kCollection
 * @implements  kArrayProvider
 * @ingroup     kApi-Data
 * @brief       Represents a 3D array.
 * 
 * The kArray3 class represents a 3D array of objects or values. The kArray3 constructor accepts arguments
 * that determine the array item type (kType) and array dimension lengths. 
 * 
 * For arrays that contain <em>objects</em> (e.g. kImage) as opposed to <em>values</em> (e.g. k32s), the objects 
 * are not automatically destroyed when the array is destroyed. To recursively destroy both the array and the 
 * array items, use kObject_Dispose. 
 * 
 * kArray3 supports the kObject_Clone, kObject_Dispose, and kObject_Size methods.
 * 
 * kArray3 supports the kdat5 and kdat6 serialization protocols.
 */
//typedef kObject kArray3;    --forward-declared in kApiDef.x.h 

/**
 * Constructs a kArray3 object.
 *
 * @public                  @memberof kArray3
 * @param   array           Receives the constructed kArray3 object.  
 * @param   itemType        Type of array element.
 * @param   length0         Length of first array dimension (outermost). 
 * @param   length1         Length of second array dimension. 
 * @param   length2         Length of third array dimension (innermost). 
 * @param   allocator       Memory allocator (or kNULL for default). 
 * @return                  Operation status.
 */
kFx(kStatus) kArray3_Construct(kArray3* array, kType itemType, kSize length0, kSize length1, kSize length2, kAlloc allocator);

#if defined(K_CPP)
/**
 * Constructs a kArray3 object using a separate allocator for data array memory.
 *
 * @public                  @memberof kArray3
 * @param   array           Receives the constructed kArray3 object.  
 * @param   itemType        Type of array element.
 * @param   length0         Length of first array dimension (outermost). 
 * @param   length1         Length of second array dimension. 
 * @param   length2         Length of third array dimension (innermost). 
 * @param   allocator       Primary memory allocator (or kNULL for default). 
 * @param   valueAllocator  Data array allocator (or kNULL for default). 
 * @param   valueAlignment  Memory alignment for the start address of values.
 * @return                  Operation status.
 */
kFx(kStatus) kArray3_ConstructEx(kArray3* array, kType itemType, kSize length0, kSize length1, kSize length2,  kAlloc allocator, kAlloc valueAllocator, kMemoryAlignment valueAlignment = kALIGN_ANY);
#endif

/** 
 * Reallocates the internal array item buffer. 
 *
 * @public              @memberof kArray3
 * @param   array       Array3 object. 
 * @param   itemType    Type of array element.
 * @param   length0     Length of first array dimension (outermost). 
 * @param   length1     Length of second array dimension. 
 * @param   length2     Length of third array dimension (innermost). 
 * @return              Operation status. 
 */
kFx(kStatus) kArray3_Allocate(kArray3 array, kType itemType, kSize length0, kSize length1, kSize length2); 

/** 
 * Resizes the internal array item buffer. 
 *
 * Note: at present, this method does not preserve existing array elements. Element preservation 
 * may be added in a future release.
 * 
 * @public              @memberof kArray3
 * @param   array       Array3 object. 
 * @param   length0     Length of first array dimension (outermost). 
 * @param   length1     Length of second array dimension. 
 * @param   length2     Length of third array dimension (innermost). 
 * @return              Operation status. 
 */
kInlineFx(kStatus) kArray3_Resize(kArray3 array, kSize length0, kSize length1, kSize length2)
{
    return kArray3_Allocate(array, kArray3_ItemType(array), length0, length1, length2); 
}

/** 
 * Attaches the array to an external item buffer. 
 *
 * Attached item buffers are not freed when the array is destroyed. 
 *
 * @public              @memberof kArray3
 * @param   array       Array object. 
 * @param   items       External item buffer. 
 * @param   itemType    Type of array element.
 * @param   length0     Length of first array dimension (outermost). 
 * @param   length1     Length of second array dimension. 
 * @param   length2     Length of third array dimension (innermost). 
 * @return              Operation status. 
 */
kFx(kStatus) kArray3_Attach(kArray3 array, void* items, kType itemType, kSize length0, kSize length1, kSize length2); 

/** 
 * Attaches the array to an external item buffer. 
 *
 * Attached item buffers are not freed when the array is destroyed. 
 *
 * A debug assertion will be raised if the size of the specified items is not equal to the 
 * size of the specified item type.
 * 
 * @relates                     @kArray3
 * @param   kArray3_array       Array object. 
 * @param   TPtr_items          Strongly-typed pointer to external item buffer. 
 * @param   kType_itemType      Type of array element.
 * @param   kSize_length0       Length of first array dimension (outermost). 
 * @param   kSize_length1       Length of second array dimension. 
 * @param   kSize_length2       Length of third array dimension (innermost). 
 * @return                      Operation status. 
 */
#define kArray3_AttachT(kArray3_array, TPtr_items, kType_itemType, kSize_length0, kSize_length1, kSize_length2) \
    xkArray3_AttachT(kArray3_array, TPtr_items, kType_itemType, kSize_length0, kSize_length1, kSize_length2, sizeof(*(TPtr_items)))

/** 
 * Performs a shallow copy of the source array.  
 *
 * Source items are copied by value; if the source array contains objects, the object 
 * handles are copied but the objects are not cloned. 
 *
 * @public              @memberof kArray3
 * @param   array       Array object. 
 * @param   source      Source array to be copied. 
 * @return              Operation status. 
 */
kInlineFx(kStatus) kArray3_Assign(kArray3 array, kArray3 source)
{
    return xkArray3_Assign(array, source, kNULL);
}

/** 
 * Performs a shallow copy of the source array.  
 *
 * Source items are copied by value; if the source array contains objects, the object 
 * handles are copied but the objects are not cloned. 
 *
 * @public              @memberof kArray3
 * @param   array       Array object. 
 * @param   source      Source array to be copied. 
 * @param   context     Context for copy operation (allocator specific; not usually required).
 * @return              Operation status. 
 */
#if defined (K_CPP)
kInlineFx(kStatus) kArray3_Assign(kArray3 array, kArray3 source, kObject context)
{
    return xkArray3_Assign(array, source, context);
}
#endif

/** 
 * Sets all array element bits to zero.
 *
 * @public              @memberof kArray3
 * @param   array       Array object. 
 * @return              Operation status. 
 */
kFx(kStatus) kArray3_Zero(kArray3 array); 

/** 
 * Sets the value of an item. 
 *
 * @public              @memberof kArray3
 * @param   array       Array object. 
 * @param   index0      First dimension index. 
 * @param   index1      Second dimension index. 
 * @param   index2      Third dimension index. 
 * @param   item        Pointer to item that will be copied (by value) into the array.
 * @return              Operation status. 
 */
kFx(kStatus) kArray3_SetItem(kArray3 array, kSize index0, kSize index1, kSize index2, const void* item); 

/** 
 * Sets the value of an item. 
 *
 * A debug assertion will be raised if the size of the specified item type is not equal to the 
 * size of the collection item type.
 * 
 * @relates                 kArray3
 * @param   kArray3_array   Array object. 
 * @param   kSize_index0    First dimension index. 
 * @param   kSize_index1    Second dimension index. 
 * @param   kSize_index2    Third dimension index. 
 * @param   TPtr_item       Strongly-typed pointer to item that will be copied (by value) into the array.
 * @return                  Operation status. 
 */
#define kArray3_SetItemT(kArray3_array, kSize_index0, kSize_index1, kSize_index2, TPtr_item) \
    xkArray3_SetItemT(kArray3_array, kSize_index0, kSize_index1, kSize_index2, TPtr_item, sizeof(*(TPtr_item)))

/** 
 * Gets the value of an item. 
 *
 * @public              @memberof kArray3
 * @param   array       Array object. 
 * @param   index0      First dimension index. 
 * @param   index1      Second dimension index. 
 * @param   index2      Third dimension index. 
 * @param   item        Destination for item that will be copied (by value) from the array. 
 * @return              Operation status. 
 */
kFx(kStatus) kArray3_Item(kArray3 array, kSize index0, kSize index1, kSize index2, void* item); 

/** 
 * Gets the value of an item. 
 *
 * A debug assertion will be raised if the size of the specified item type is not equal to the 
 * size of the collection item type.
 * 
 * @relates                 kArray3
 * @param   kArray3_array   Array object. 
 * @param   kSize_index0    First dimension index. 
 * @param   kSize_index1    Second dimension index. 
 * @param   kSize_index2    Third dimension index. 
 * @param   TPtr_item       Strongly-typed destination pointer for item that will be copied (by value) from the array. 
 * @return                  Operation status. 
 */
#define kArray3_ItemT(kArray3_array, kSize_index0, kSize_index1, kSize_index2, TPtr_item) \
    xkArray3_ItemT(kArray3_array, kSize_index0, kSize_index1, kSize_index2, TPtr_item, sizeof(*(TPtr_item)))


/** 
* Sets the value of an item. 
*
* A debug assertion will be raised if the index arguments are greater than or equal to 
* the current array lengths, or if the size of the specified item type is not equal to the 
* size of the collection item type.
* 
* @relates                      @Array3
* @param   kArray3_array        Array object. 
* @param   kSize_index0         First dimension index. 
* @param   kSize_index1         Second dimension index. 
* @param   kSize_index2         Third dimension index. 
* @param   T_value              Item value.  
* @param   T                    Item type identifier (e.g., k32s).
* @return                       Item value. 
*/
#define kArray3_SetAsT(kArray3_array, kSize_index0, kSize_index1, kSize_index2, T_value, T) \
    (kPointer_WriteAs(xkArray3_AsT(kArray3_array, kSize_index0, kSize_index1, kSize_index2, sizeof(T)), T_value, T), (void)0)

/** 
* Gets the value of an item. 
*
* A debug assertion will be raised if the index arguments are greater than or equal to 
* the current array lengths, or if the size of the specified item type is not equal to the 
* size of the collection item type.
* 
* @relates                      @Array3
* @param   kArray3_array        Array object. 
* @param   kSize_index0         First dimension index. 
* @param   kSize_index1         Second dimension index. 
* @param   kSize_index2         Third dimension index. 
* @param   T                    Item type identifier (e.g., k32s).
* @return                       Item value. 
*/
#define kArray3_AsT(kArray3_array, kSize_index0, kSize_index1, kSize_index2, T) \
    kPointer_ReadAs(xkArray3_AsT(kArray3_array, kSize_index0, kSize_index1, kSize_index2, sizeof(T)), T)

/** 
 * Returns a pointer to the array item buffer.  
 *
 * @public              @memberof kArray3
 * @param   array       Array object. 
 * @return              Pointer to array item buffer. 
 */
kInlineFx(void*) kArray3_Data(kArray3 array)
{
    kObj(kArray3, array);

    return obj->items; 
}

/** 
 * Returns a strongly-typed pointer to the array item buffer.  
 *
 * A debug assertion will be raised if the size of the specified item type is not equal to the 
 * size of the collection item type.
 * 
 * @relates                 kArray3
 * @param   kArray3_array   Array object. 
 * @param   T               Item type identifier (e.g., k32s).
 * @return                  Strongly-typed Ppointer to array item buffer. 
 */
#define kArray3_DataT(kArray3_array, T) \
    kCast(T*, xkArray3_DataT(kArray3_array, sizeof(T)))

/** 
 * Calculates an address relative to the start of the buffer.
 *
 * This method is similar to the At method, but does not bounds-check the index arguments. 
 *
 * @public              @memberof kArray3
 * @param   array       Array object. 
 * @param   index0      First dimension index. 
 * @param   index1      Second dimension index. 
 * @param   index2      Third dimension index. 
 * @return              Calculated pointer. 
 */
kInlineFx(void*) kArray3_DataAt(kArray3 array, kSSize index0, kSSize index1, kSSize index2)
{
    kObj(kArray3, array); 
    kSSize index = (index0*(kSSize)obj->length[1] + index1)*(kSSize)obj->length[2] + index2; 

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
 * @relates                 kArray3
 * @param   kArray3_array   Array object. 
 * @param   kSize_index0    First dimension index. 
 * @param   kSize_index1    Second dimension index. 
 * @param   kSize_index2    Third dimension index. 
 * @param   T               Item type identifier (e.g., k32s).
 * @return                  Calculated pointer. 
 */
#define kArray3_DataAtT(kArray3_array, kSize_index0, kSize_index1, kSize_index2, T) \
    kCast(T*, xkArray3_DataAtT(kArray3_array, kSize_index0, kSize_index1, kSize_index2, sizeof(T)))

/** 
 * Reports the size, in bytes, of the array item buffer. 
 *
 * @public              @memberof kArray3
 * @param   array       Array object. 
 * @return              Size of array item buffer (bytes). 
 */
kInlineFx(kSize) kArray3_DataSize(kArray3 array)
{
    kObj(kArray3, array); 

    return kArray3_Count(array) * obj->itemSize;
}

/** 
 * Returns a pointer to the specified item in the array. 
 *
 * A debug assertion will be raised if the index arguments are greater than or equal to 
 * the current array dimensions. 
 * 
 * @public              @memberof kArray3
 * @param   array       Array object. 
 * @param   index0      First dimension index. 
 * @param   index1      Second dimension index. 
 * @param   index2      Third dimension index. 
 * @return              Pointer to item. 
 */
kInlineFx(void*) kArray3_At(kArray3 array, kSize index0, kSize index1, kSize index2)
{    
#   if !defined(K_FSS_912_DISABLE_BOUNDS_CHECK)
    {
        kAssert(index0 < kArray3_Length(array, 0));
        kAssert(index1 < kArray3_Length(array, 1));
        kAssert(index2 < kArray3_Length(array, 2));
    }
#   endif

    return kArray3_DataAt(array, (kSSize)index0, (kSSize)index1, (kSSize)index2);
}

/** 
 * Returns a strongly-typed pointer to the specified item in the array. 
 *
 * A debug assertion will be raised if the index arguments are greater than or equal to 
 * the current array dimensions, or if the size of the specified item type is not equal to the 
 * size of the collection item type.
 * 
 * @relates                 kArray3
 * @param   kArray3_array   Array object. 
 * @param   kSize_index0    First dimension index. 
 * @param   kSize_index1    Second dimension index. 
 * @param   kSize_index2    Third dimension index. 
 * @param   T               Item type identifier (e.g., k32s).
 * @return                  Strongly-typed pointer to item. 
 */
#define kArray3_AtT(kArray3_array, kSize_index0, kSize_index1, kSize_index2, T) \
    kCast(T*, xkArray3_AtT(kArray3_array, kSize_index0, kSize_index1, kSize_index2, sizeof(T)))

/** 
 * Returns the array item type. 
 *
 * @public              @memberof kArray3
 * @param   array       Array object. 
 * @return              Array item type. 
 */
kInlineFx(kType) kArray3_ItemType(kArray3 array)
{
    kObj(kArray3, array); 

    return obj->itemType;  
}

/** 
 * Returns the array item size. 
 *
 * @public              @memberof kArray3
 * @param   array       Array object. 
 * @return              Array item size. 
 */
kInlineFx(kSize) kArray3_ItemSize(kArray3 array)
{
    kObj(kArray3, array); 

    return obj->itemSize;
}

/** 
 * Returns the length of the specified array dimension, in elements. 
 *
 * @public              @memberof kArray3
 * @param   array       Array object. 
 * @param   dimension   Array dimension index. 
 * @return              Array dimension length (in elements). 
 */
kInlineFx(kSize) kArray3_Length(kArray3 array, kSize dimension)
{
    kObj(kArray3, array); 

    kAssert(dimension < 3); 

    return obj->length[dimension];
}

/** 
 * Returns the array item count, in elements. 
 *
 * @public              @memberof kArray3
 * @param   array       Array object. 
 * @return              Array item count (in elements). 
 */
kInlineFx(kSize) kArray3_Count(kArray3 array)
{
    kObj(kArray3, array); 

    return obj->length[0] * obj->length[1] * obj->length[2];
}

/** 
 * Reports the allocator used for the internal data array.
 *
 * @public          @memberof kArray3
 * @param   array   Array object. 
 * @return          Data array allocator.
 */
kInlineFx(kAlloc) kArray3_DataAlloc(kArray3 array)
{
    kObj(kArray3, array);

    return kType_IsValue(obj->itemType) ? obj->valueAlloc : kObject_Alloc(array); 
}

#endif

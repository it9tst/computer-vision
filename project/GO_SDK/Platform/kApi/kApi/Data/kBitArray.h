/** 
 * @file    kBitArray.h
 * @brief   Declares the kBitArray class. 
 *
 * @internal
 * Copyright (C) 2019-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_BIT_ARRAY_H
#define K_API_BIT_ARRAY_H

#include <kApi/kApiDef.h>
#include <kApi/Data/kBitArray.x.h>

/**
 * @class       kBitArray
 * @extends     kObject
 * @ingroup     kApi-Data
 * @brief       Represents a 1D array of bits.
 * 
 * The kBitArray class represents a 1D array of bits. kBitArray is similar in spirit to 
 * kArray1<kBool>, but is optimized for space and provides a set of convenient bitwise 
 * operations.
 * 
 * kBitArray supports the kObject_Clone, kObject_Dispose, kObject_Equals, and kObject_Size methods.
 * 
 * kBitArray supports the kdat6 serialization protocol. 
 */
//typedef kObject kBitArray;   --forward-declared in kApiDef.x.h 

/** 
 * Constructs a kBitArray object.
 *
 * @public              @memberof kBitArray
 * @param   array       Receives the constructed object.  
 * @param   length      Length of array.
 * @param   allocator   Memory allocator (or kNULL for default). 
 * @return              Operation status. 
 */
kFx(kStatus) kBitArray_Construct(kBitArray* array, kSize length, kAlloc allocator);

/** 
 * Reallocates the array item buffer. 
 *
 * @public              @memberof kBitArray
 * @param   array       Array object. 
 * @param   length      Length of array, in bits.
 * @return              Operation status. 
 */
kInlineFx(kStatus) kBitArray_Allocate(kBitArray array, kSize length)
{
    return xkBitArray_Reallocate(array, length, kFALSE);
}

/** 
 * Resizes the internal array item buffer. 
 * 
 * Existing elements are preserved; new elements are set to zero.
 *
 * @public              @memberof kBitArray
 * @param   array       Array object. 
 * @param   length      Length of array, in bits.
 * @return              Operation status. 
 */
kInlineFx(kStatus) kBitArray_Resize(kBitArray array, kSize length)
{
    return xkBitArray_Reallocate(array, length, kTRUE);
}

/** 
 * Copies the source array.  
 *
 * @public              @memberof kBitArray
 * @param   array       Array object. 
 * @param   source      Source array to be copied. 
 * @return              Operation status. 
 */
kFx(kStatus) kBitArray_Assign(kBitArray array, kBitArray source);

/** 
 * Sets all array elements to the specified value.
 *
 * @public              @memberof kBitArray
 * @param   array       Array object. 
 * @param   value       Value to be set. 
 * @return              Operation status. 
 */
kInlineFx(kStatus) kBitArray_SetAll(kBitArray array, kBool value)
{
    kObj(kBitArray, array);
    kSize wordCount = xkBitArray_WordCount(array);
    k64u wordValue = (value) ? k64U_MAX : 0;
    
    for (kSize i = 0; i < wordCount; ++i)
    {
        obj->buffer[i] = wordValue;
    }

    return kOK;
}

/** 
 * Sets the value of the specified item. 
 *
 * @public              @memberof kBitArray
 * @param   array       Array object. 
 * @param   index       Array item index. 
 * @param   value       Value to set. 
 * @return              Operation status. 
 */
kInlineFx(kStatus) kBitArray_SetItem(kBitArray array, kSize index, kBool value)
{
    kObj(kBitArray, array);

    kCheckArgs(index < obj->bitLength);    

    kSize wordIndex = index >> kBIT_ARRAY_WORD_WIDTH_SHIFT; 
    kSize bitOffset = index & kBIT_ARRAY_BIT_OFFSET_MASK;

    if (value)  obj->buffer[wordIndex] |=  (1llu << bitOffset); 
    else        obj->buffer[wordIndex] &= ~(1llu << bitOffset);

    return kOK;
}

/** 
 * Gets the value of an item. 
 *
 * @public              @memberof kBitArray
 * @param   array       Array object. 
 * @param   index       Array item index. 
 * @param   value       Destination for item that will be copied from the array. 
 * @return              Operation status. 
 */
kInlineFx(kStatus) kBitArray_Item(kBitArray array, kSize index, kBool* value)
{
    kObj(kBitArray, array);

    kCheckArgs(index < obj->bitLength);    

    *value = kBitArray_At(array, index);

    return kOK;
}

/** 
 * Inverts all bits.
 *
 * @public              @memberof kBitArray
 * @param   array       Array object. 
 * @return              Operation status.
 */
kFx(kStatus) kBitArray_Not(kBitArray array);

/** 
 * Performs bitwise-and over all bits from the specified arrays.
 * 
 * Arrays must be the same length.
 *
 * @public              @memberof kBitArray
 * @param   array       First source array; receives results. 
 * @param   other       Second source array.
 * @return              Operation status.
 */
kFx(kStatus) kBitArray_And(kBitArray array, kBitArray other);

/** 
 * Performs bitwise-or over all bits from the specified arrays.
 *
 * Arrays must be the same length.
 *
 * @public              @memberof kBitArray
 * @param   array       First source array; receives results. 
 * @param   other       Second source array.
 * @return              Operation status.
 */
kFx(kStatus) kBitArray_Or(kBitArray array, kBitArray other);

/** 
 * Performs bitwise-xor over all bits from the specified arrays.
 *
 * Arrays must be the same length.
 *
 * @public              @memberof kBitArray
 * @param   array       First source array; receives results. 
 * @param   other       Second source array.
 * @return              Operation status.
 */
kFx(kStatus) kBitArray_Xor(kBitArray array, kBitArray other);

/** 
 * Returns the specified item in the array. 
 *
 * A debug assertion will be raised if the index argument is greater than or equal to 
 * the current array Length.  
 * 
 * @public              @memberof kBitArray
 * @param   array       Array object. 
 * @param   index       Item index. 
 * @return              Item value. 
 */
kInlineFx(kBool) kBitArray_At(kBitArray array, kSize index)
{
    kObj(kBitArray, array);

    kAssert(index < obj->bitLength);

    kSize wordIndex = index >> kBIT_ARRAY_WORD_WIDTH_SHIFT; 
    kSize bitOffset = index & kBIT_ARRAY_BIT_OFFSET_MASK;

    return (obj->buffer[wordIndex] >> bitOffset) & 0x1;
}

/** 
 * Returns the array length, in bits. 
 *
 * @public              @memberof kBitArray
 * @param   array       Array object. 
 * @return              Array length (in bits). 
 */
kInlineFx(kSize) kBitArray_Length(kBitArray array)
{
    kObj(kBitArray, array);

    return obj->bitLength;
}

/** 
 * Counts the number of bits in the array that are set to kTRUE.
 *
 * @public              @memberof kBitArray
 * @param   array       Array object. 
 * @return              Count of bits set to kTRUE.
 */
kFx(k64u) kBitArray_TrueCount(kBitArray array);

/** 
 * Counts the number of bits in the array that are set to kFALSE.
 *
 * @public              @memberof kBitArray
 * @param   array       Array object. 
 * @return              Count of bits set to kFALSE.
 */
kInlineFx(k64u) kBitArray_FalseCount(kBitArray array)
{
    return kBitArray_Length(array) - kBitArray_TrueCount(array);
}

#endif

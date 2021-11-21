/** 
 * @file    kBitArray.x.h
 *
 * @internal
 * Copyright (C) 2019-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_BIT_ARRAY_X_H
#define K_API_BIT_ARRAY_X_H

#define kBIT_ARRAY_WORD_WIDTH_SHIFT         (6)                                 //word width, expressed as bit shift
#define kBIT_ARRAY_BITS_PER_WORD            (1 << kBIT_ARRAY_WORD_WIDTH_SHIFT)  //word width, expressed as bit count
#define kBIT_ARRAY_BYTES_PER_WORD           (kBIT_ARRAY_BITS_PER_WORD >> 3)     //bytes per word
#define kBIT_ARRAY_BIT_OFFSET_MASK          (kBIT_ARRAY_BITS_PER_WORD - 1)      //mask for bit offset within one word

typedef struct kBitArrayClass
{
    kObjectClass base; 
    kSize allocSize;            //size of allocated array memory, in bytes
    k64u* buffer;               //array memory 
    kSize bufferCount;          //count of allocated buffer elements; 
    kSize bitLength;            //logical bit array length, in bits
} kBitArrayClass;
    
kDeclareClassEx(k, kBitArray, kObject) 

/* 
* Forward declarations. 
*/

kInlineFx(kBool) kBitArray_At(kBitArray array, kSize index);

/* 
* Private methods. 
*/

kFx(kStatus) xkBitArray_ConstructFramework(kBitArray* array, kAlloc allocator);

kInlineFx(kSize) xkBitArray_WordCount(kBitArray array)
{
    kObj(kBitArray, array);
    
    return (obj->bitLength + kBIT_ARRAY_BITS_PER_WORD - 1) >> kBIT_ARRAY_WORD_WIDTH_SHIFT;
}

kFx(kStatus) xkBitArray_Init(kBitArray array, kType classType, kSize length, kAlloc alloc);
kFx(kStatus) xkBitArray_VClone(kBitArray array, kBitArray source, kAlloc valueAlloc, kObject context); 

kFx(kStatus) xkBitArray_VRelease(kBitArray array); 

kFx(kSize) xkBitArray_VSize(kBitArray array); 
kFx(kBool) xkBitArray_VEquals(kBitArray array, kBitArray other); 

kFx(kStatus) xkBitArray_WriteDat6V0(kBitArray array, kSerializer serializer); 
kFx(kStatus) xkBitArray_ReadDat6V0(kBitArray array, kSerializer serializer); 

kFx(kStatus) xkBitArray_Reallocate(kBitArray array, kSize length, kBool preserveAndZero); 

#endif

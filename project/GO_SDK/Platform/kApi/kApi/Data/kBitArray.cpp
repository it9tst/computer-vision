/** 
 * @file    kBitArray.cpp
 *
 * @internal
 * Copyright (C) 2019-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <kApi/Data/kBitArray.h>
#include <kApi/Io/kSerializer.h>

kBeginClassEx(k, kBitArray) 
    
    //serialization versions
    kAddPrivateVersionEx(kBitArray, "kdat6", "7.0.0.0", "kBitArray-0", WriteDat6V0, ReadDat6V0)

    //special constructors
    kAddPrivateFrameworkConstructor(kBitArray, ConstructFramework)

    //virtual methods
    kAddPrivateVMethod(kBitArray, kObject, VRelease)
    kAddPrivateVMethod(kBitArray, kObject, VClone)
    kAddPrivateVMethod(kBitArray, kObject, VSize)
    kAddPrivateVMethod(kBitArray, kObject, VEquals)

kEndClassEx() 

kFx(kStatus) kBitArray_Construct(kBitArray* array, kSize length, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback(allocator);
    kStatus status; 

    kCheck(kAlloc_GetObject(alloc, kTypeOf(kBitArray), array)); 

    if (!kSuccess(status = xkBitArray_Init(*array, kTypeOf(kBitArray), length, alloc)))
    {
        kAlloc_FreeRef(alloc, array); 
    }

    return status; 
} 

kFx(kStatus) xkBitArray_ConstructFramework(kBitArray* array, kAlloc allocator)
{
    return kBitArray_Construct(array, 0, allocator);
}

kFx(kStatus) xkBitArray_Init(kBitArray array, kType classType, kSize length, kAlloc alloc)
{
    kObjR(kBitArray, array);
    kStatus status = kOK; 

    kCheck(kObject_Init(array, classType, alloc)); 

    obj->allocSize = 0;
    obj->buffer = kNULL;
    obj->bufferCount = 0; 
    obj->bitLength = 0;

    kTry
    {
        kTest(kBitArray_Allocate(array, length)); 
    }
    kCatch(&status)
    {
        xkBitArray_VRelease(array); 
        kEndCatch(status); 
    }

    return kOK; 
}

kFx(kStatus) xkBitArray_VClone(kBitArray array, kBitArray source, kAlloc valueAlloc, kObject context)
{
    return kBitArray_Assign(array, source);
}

kFx(kStatus) xkBitArray_VRelease(kBitArray array)
{
    kObj(kBitArray, array);

    kCheck(kAlloc_Free(kObject_Alloc(array), obj->buffer)); 

    kCheck(kObject_VRelease(array)); 

    return kOK; 
}

kFx(kSize) xkBitArray_VSize(kBitArray array)
{
    kObj(kBitArray, array);

    return sizeof(kBitArrayClass) + obj->bufferCount*sizeof(obj->buffer[0]); 
}

kFx(kBool) xkBitArray_VEquals(kBitArray array, kBitArray other)
{
    if (!kObject_Is(other, kTypeOf(kBitArray)) || (kBitArray_Length(array) != kBitArray_Length(other)))
    {
        return kFALSE; 
    }
   
    kObj(kBitArray, array);
    kObjN(kBitArray, otherObj, other);
    kSize completeWordCount = obj->bitLength >> kBIT_ARRAY_WORD_WIDTH_SHIFT;
    
    for (kSize i = 0; i < completeWordCount; ++i)
    {
        if (obj->buffer[i] != otherObj->buffer[i])
        {
            return kFALSE;
        }
    }
    
    kSize incompleteWordBitCount = obj->bitLength & kBIT_ARRAY_BIT_OFFSET_MASK;             
    
    if (incompleteWordBitCount > 0)
    {
        k64u incompleteWord = obj->buffer[completeWordCount] & ((1llu << incompleteWordBitCount) - 1);
        k64u otherIncompleteWord = otherObj->buffer[completeWordCount] & ((1llu << incompleteWordBitCount) - 1);
        
        if (incompleteWord != otherIncompleteWord)
        {
            return kFALSE;
        } 
    }
   
    return kTRUE;
}

kFx(kStatus) xkBitArray_WriteDat6V0(kBitArray array, kSerializer serializer)
{
    kObj(kBitArray, array);
  
    kCheck(kSerializer_WriteSize(serializer, obj->bitLength)); 

    kCheck(kSerializer_Write64uArray(serializer, obj->buffer, xkBitArray_WordCount(array))); 

    return kOK; 
}

kFx(kStatus) xkBitArray_ReadDat6V0(kBitArray array, kSerializer serializer)
{
    kObj(kBitArray, array);
    kSize bitLength = 0; 

    kCheck(kSerializer_ReadSize(serializer, &bitLength)); 

    kCheck(kBitArray_Allocate(array, bitLength)); 

    kCheck(kSerializer_Read64uArray(serializer, obj->buffer,  xkBitArray_WordCount(array))); 

    return kOK; 
}

kFx(kStatus) kBitArray_Assign(kBitArray array, kBitArray source)
{
    kObj(kBitArray, array);
    kObjN(kBitArray, sourceObj, source);
  
    kCheck(kBitArray_Allocate(array, sourceObj->bitLength)); 

    kCheck(kMemCopy(obj->buffer, sourceObj->buffer, xkBitArray_WordCount(array)*kBIT_ARRAY_BYTES_PER_WORD));

    return kOK;   
}

kFx(kStatus) xkBitArray_Reallocate(kBitArray array, kSize length, kBool preserveAndZero)
{
    kObj(kBitArray, array);

    if (length > obj->bitLength)
    {
        kSize requiredWordCount = kDivideCeilUInt_(length, kBIT_ARRAY_BITS_PER_WORD);
        kSize newWordCount = kMax_(obj->bufferCount, requiredWordCount); 
  
        if (newWordCount > obj->bufferCount)
        {
            k64u* oldBuffer = obj->buffer; 
            kSize oldWordCount = obj->bufferCount;
            kSize wordSize = sizeof(obj->buffer[0]);

            kCheck(kAlloc_Get(kObject_Alloc(array), newWordCount*wordSize, &obj->buffer));
            obj->bufferCount = newWordCount; 

            //copy old words, zero new words
            if (preserveAndZero)
            {
                kCheck(kMemCopy(obj->buffer, oldBuffer, oldWordCount*wordSize));               
                kCheck(kMemZero(kPointer_ByteOffset(obj->buffer, oldWordCount*wordSize), (newWordCount-oldWordCount)*wordSize));
            }

            kCheck(kAlloc_Free(kObject_Alloc(array), oldBuffer));
        }

        //zero any previously-unused bits in previous final word
        if (preserveAndZero)
        { 
            kSize incompleteWordBitCount = obj->bitLength & kBIT_ARRAY_BIT_OFFSET_MASK; 
            
            if (incompleteWordBitCount > 0)
            {
                kSize incompleteWordIndex = obj->bitLength >> kBIT_ARRAY_WORD_WIDTH_SHIFT; 
            
                 obj->buffer[incompleteWordIndex] &= (1llu << incompleteWordBitCount) - 1;  
            }
        }
    }

    obj->bitLength = length; 
    
    return kOK; 
}

kFx(k64u) kBitArray_TrueCount(kBitArray array)
{
   kObj(kBitArray, array);
   k64u bitCount = 0;

   if (obj->bitLength > 0)
   {
        kSize completeWordCount = obj->bitLength >> kBIT_ARRAY_WORD_WIDTH_SHIFT;
   
        for (kSize i = 0; i < completeWordCount; ++i)
        {
            bitCount += xkCountBits64(obj->buffer[i]);
        }

        kSize incompleteWordBitCount = obj->bitLength & kBIT_ARRAY_BIT_OFFSET_MASK;             

        if (incompleteWordBitCount > 0)
        {
            k64u incompleteWord = obj->buffer[completeWordCount]; 
            
            incompleteWord &= (1llu << incompleteWordBitCount) - 1;  

            bitCount += xkCountBits64(incompleteWord);
        }
    }

    return bitCount;
}

kFx(kStatus) kBitArray_Not(kBitArray array)
{
    kObj(kBitArray, array);
    kSize wordCount = xkBitArray_WordCount(array);
    
    for (kSize i = 0; i < wordCount; ++i)
    {
        obj->buffer[i] = ~obj->buffer[i];
    }

    return kOK;
}

kFx(kStatus) kBitArray_And(kBitArray array, kBitArray other)
{
    kObj(kBitArray, array);
    kObjN(kBitArray, otherObj, other);

    kCheckArgs(kBitArray_Length(array) == kBitArray_Length(other));

    kSize wordCount = xkBitArray_WordCount(array);
    
    for (kSize i = 0; i < wordCount; ++i)
    {
        obj->buffer[i] = obj->buffer[i] & otherObj->buffer[i];
    }

    return kOK;
}

kFx(kStatus) kBitArray_Or(kBitArray array, kBitArray other)
{
    kObj(kBitArray, array);
    kObjN(kBitArray, otherObj, other);

    kCheckArgs(kBitArray_Length(array) == kBitArray_Length(other));

    kSize wordCount = xkBitArray_WordCount(array);
    
    for (kSize i = 0; i < wordCount; ++i)
    {
        obj->buffer[i] = obj->buffer[i] | otherObj->buffer[i];
    }

    return kOK;
}

kFx(kStatus) kBitArray_Xor(kBitArray array, kBitArray other)
{
    kObj(kBitArray, array);
    kObjN(kBitArray, otherObj, other);

    kCheckArgs(kBitArray_Length(array) == kBitArray_Length(other));

    kSize wordCount = xkBitArray_WordCount(array);
    
    for (kSize i = 0; i < wordCount; ++i)
    {
        obj->buffer[i] = obj->buffer[i] ^ otherObj->buffer[i];
    }

    return kOK;
}

/** 
 * @file    kString.cpp
 *
 * @internal
 * Copyright (C) 2005-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <kApi/Data/kString.h>
#include <kApi/Data/kArrayList.h>
#include <kApi/Io/kSerializer.h>
#include <stdio.h>

kBeginClassEx(k, kString) 
    
    //serialization versions
    kAddPrivateVersionEx(kString, "kdat5", "5.0.0.0", "43-1", WriteDat5V1, ReadDat5V1)
    kAddPrivateVersionEx(kString, "kdat6", "5.7.1.0", "kString-0", WriteDat6V0, ReadDat6V0)

    //special constructors
    kAddPrivateFrameworkConstructor(kString, ConstructFramework)

    //virtual methods
    kAddPrivateVMethod(kString, kObject, VRelease)
    kAddPrivateVMethod(kString, kObject, VClone)
    kAddPrivateVMethod(kString, kObject, VHashCode)
    kAddPrivateVMethod(kString, kObject, VEquals)
    kAddPrivateVMethod(kString, kObject, VSize)

kEndClassEx() 

kFx(kStatus) kString_Construct(kString* str, const kChar* content, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback(allocator);
    kStatus status; 

    kCheck(kAlloc_GetObject(alloc, kTypeOf(kString), str)); 

    if (!kSuccess(status = xkString_Init(*str, kTypeOf(kString), content, alloc)))
    {
        kAlloc_FreeRef(alloc, str); 
    }

    return status; 
} 

kFx(kStatus) xkString_ConstructFramework(kString* string, kAlloc allocator)
{
    return kString_Construct(string, kNULL, allocator);
}

kFx(kStatus) xkString_Init(kString str, kType classType, const kChar* content, kAlloc allocator)
{
    kObjR(kString, str);  
    kStatus status = kOK; 

    kCheck(kObject_Init(str, classType, allocator)); 

    obj->allocSize = 0; 
    obj->chars = &obj->nullStr; 
    obj->length = 0; 
    obj->capacity = 0; 
    obj->nullStr = 0; 

    kTry
    {
        kTest(kString_Import(str, content, kFALSE)); 
    }
    kCatch(&status)
    {
        xkString_VRelease(str); 
        kEndCatch(status); 
    }

    return kOK; 
}

kFx(kStatus) xkString_VClone(kString str, kString source, kAlloc valueAlloc, kObject context)
{
    return kString_Import(str, kString_Chars(source), kFALSE);
}

kFx(kStatus) xkString_VRelease(kString str)
{
    kObj(kString, str); 

    if (obj->allocSize > 0)
    {
        kCheck(kObject_FreeMem(str, obj->chars)); 
    }

    kCheck(kObject_VRelease(str)); 

    return kOK; 
}

kFx(kSize) xkString_VSize(kString str)
{
    kObj(kString, str); 
    kSize size = sizeof(kStringClass) + obj->allocSize; 

    return size; 
}

kFx(kSize) xkString_VHashCode(kString str)
{
    kObj(kString, str); 
    const kChar* it = kString_Chars(obj); 
    kSize hash = 0;
    
    while (!kIsNull(*it))
    {
        hash = hash * 31 + *it++; 
    }

    return hash; 
}

kFx(kBool) xkString_VEquals(kString str, kObject other)
{
    kAssertType(str, kString); 

    if (kObject_Type(other) == kTypeOf(kString))
    {
        return kStrEquals(kString_Chars(str), kString_Chars(other)); 
    }

    return kFALSE; 
}

kFx(kStatus) xkString_WriteDat5V1(kString str, kSerializer serializer)
{
    kObj(kString, str); 
    kTypeVersion itemVersion; 

    kCheck(kSerializer_WriteSize(serializer, obj->capacity)); 
    kCheck(kSerializer_Write32s(serializer, kTRUE));  //isDynamic

    kCheck(kSerializer_WriteSize(serializer, obj->length)); 
    kCheck(kSerializer_WriteType(serializer, kTypeOf(kChar), &itemVersion));
    kCheck(kSerializer_WriteItems(serializer, kTypeOf(kChar), itemVersion, obj->chars, obj->length)); 

    return kOK; 
}

kFx(kStatus) xkString_ReadDat5V1(kString str, kSerializer serializer)
{
    kObj(kString, str); 
    kSize capacity = 0, length = 0; 
    kTypeVersion itemVersion;
    k32s isDynamic; 
    kType itemType = kNULL;            

    kCheck(kSerializer_ReadSize(serializer, &capacity));      
    kCheck(kSerializer_Read32s(serializer, &isDynamic));

    kCheck(kSerializer_ReadSize(serializer, &length));
    kCheck(kSerializer_ReadType(serializer, &itemType, &itemVersion)); 

    kCheck(kString_Reserve(str, length)); 

    kCheck(kSerializer_ReadItems(serializer, itemType, itemVersion, obj->chars, length)); 
    obj->length = length; 
    obj->chars[length] = 0; 

    return kOK; 
}

kFx(kStatus) xkString_WriteDat6V0(kString str, kSerializer serializer)
{
    kObj(kString, str); 
    kTypeVersion itemVersion; 

    kCheck(kSerializer_WriteType(serializer, kTypeOf(kChar), &itemVersion));
    kCheck(kSerializer_WriteSize(serializer, obj->length)); 
    kCheck(kSerializer_WriteItems(serializer, kTypeOf(kChar), itemVersion, obj->chars, obj->length)); 

    return kOK; 
}

kFx(kStatus) xkString_ReadDat6V0(kString str, kSerializer serializer)
{
    kObj(kString, str); 
    kType itemType = kNULL;            
    kTypeVersion itemVersion; 
    kSize length = 0; 

    kCheck(kSerializer_ReadType(serializer, &itemType, &itemVersion));
    kCheck(kSerializer_ReadSize(serializer, &length)); 

    kCheck(kString_Reserve(str, length)); 

    kCheck(kSerializer_ReadItems(serializer, itemType, itemVersion, obj->chars, length)); 

    obj->length = length; 
    obj->chars[length] = 0; 

    return kOK; 
}

kFx(kStatus) kString_Assign(kString str, kString source)
{
    return kString_Import(str, kString_Chars(source), kFALSE); 
}

kFx(kStatus) kString_Clear(kString str)
{
    kObj(kString, str); 
    
    obj->chars[0] = 0; 
    obj->length = 0; 

    return kOK; 
}

kFx(kStatus) kString_Reserve(kString str, kSize minimumCapacity)
{
    kObj(kString, str); 
    kType itemType = kTypeOf(kChar); 
    kSize itemSize = kType_Size(itemType); 

    if (obj->capacity < minimumCapacity)
    {
        kSize grownCapacity = kMax_(minimumCapacity, xkSTRING_GROWTH_FACTOR*obj->capacity);
        kSize newCapacity = kMax_(grownCapacity, xkSTRING_MIN_CAPACITY); 
        kSize newSize = kMax_(obj->allocSize, (newCapacity+1)*itemSize);    //+1 for null terminator

        if (newSize > obj->allocSize)
        {
            kChar* oldChars = obj->chars; 
            kChar* newChars = kNULL; 

            kCheck(kObject_GetMem(str, newSize, &newChars)); 

            kMemCopy(newChars, oldChars, (obj->capacity+1)*itemSize);    //+1 for null terminator

            obj->chars = newChars; 
            obj->allocSize = newSize; 

            if (oldChars != &obj->nullStr)
            {
                kCheck(kObject_FreeMem(str, oldChars)); 
            }
        }

        obj->capacity = newCapacity;
    }
    
    return kOK; 
}

kFx(kStatus) kString_SetLength(kString str, kSize length)
{
    kObj(kString, str);

    if (length > obj->capacity)
    {
        kCheck(kString_Reserve(str, length));
    }

    obj->length = length;
    obj->chars[length] = 0;

    return kOK;
}

kFx(k32s) kString_Compare(kString str, const kChar* content)
{
    kObj(kString, str); 

    return kStrCompare(kString_Chars(obj), content); 
}

kFx(kBool) kString_Equals(kString str, const kChar* content)
{
    kObj(kString, str); 

    return kStrEquals(kString_Chars(obj), content); 
}

kFx(kStatus) kString_Trim(kString str)
{
    kObj(kString, str); 
    kSize it = 0; 

    //remove leading
    while (kChar_IsSpace(obj->chars[it]))
    {
        it++; 
    }

    kCheck(kMemMove(obj->chars, &obj->chars[it], obj->length + 1 - it)); 
    obj->length -= it; 

    //remove trailing
    if (obj->length > 0)
    {
        it = obj->length - 1; 

        while ((it > 0) && kChar_IsSpace(obj->chars[it]))
        {
            it--; 
        }

        obj->length = it + 1; 
        obj->chars[obj->length] = 0; 
    }

    return kOK; 
}

kFx(kStatus) kString_Setf(kString str, const kChar* format, ...)
{
    kVarArgList argList; 
    kStatus status; 
   
    kVarArgList_Start(argList, format);
    {
        status = kString_Setvf(str, format, argList); 
    }
    kVarArgList_End(argList);
    
    return status; 
}

kFx(kStatus) kString_Setvf(kString str, const kChar* format, kVarArgList argList)
{
    kObj(kString, str); 
    kVarArgList argListCopy; 
    k32s charCount; 

    kVarArgList_Copy(argListCopy, argList);
    {
        charCount = xkStrMeasuref(format, argListCopy);  
    }
    kVarArgList_End(argListCopy);

    kCheckArgs(charCount >= 0); 

    kCheck(kString_Reserve(str, (kSize)charCount)); 
            
    if (vsprintf(obj->chars, format, argList) < 0)
    {
        return kERROR; 
    }

    obj->length = (kSize) charCount; 

    return kOK; 
}

kFx(kStatus) kString_Set(kString str, const kChar* content)
{
    kObj(kString, str); 

    kCheck(kString_Clear(str));  
    kCheck(kString_Add(str, content));  
    
    return kOK; 
}

kFx(kStatus) kString_Addf(kString str, const kChar* format, ...)
{
    kVarArgList argList;  
    kStatus status; 
   
    kVarArgList_Start(argList, format);
    {
        status = kString_Addvf(str, format, argList); 
    }
    kVarArgList_End(argList);
 
    return status;   
}

kFx(kStatus) kString_Addvf(kString str, const kChar* format, kVarArgList argList)
{
    kObj(kString, str); 
    kSize length = kString_Length(str);    
    kVarArgList argListCopy; 
    k32s charCount; 

    kVarArgList_Copy(argListCopy, argList);
    {
        charCount = xkStrMeasuref(format, argListCopy);  
    }
    kVarArgList_End(argListCopy);

    kCheckArgs(charCount >= 0); 

    kCheck(kString_Reserve(str, length + (kSize)charCount)); 
        
    if (vsprintf(obj->chars + length, format, argList) < 0)
    {
        return kERROR; 
    }

    obj->length = length + (kSize)charCount; 

    return kOK; 
}

kFx(kStatus) kString_Add(kString str, const kChar* content)
{
    kObj(kString, str); 

    if (!kIsNull(content))
    {  
        kSize contentLength = kStrLength(content); 
        kSize combinedLength = contentLength + obj->length;

        kCheck(kString_Reserve(str, combinedLength)); 
    
        kMemCopy(obj->chars + obj->length, content, contentLength*sizeof(kChar));

        obj->length = combinedLength;
        obj->chars[combinedLength] = 0; 
    }
    
    return kOK;
}

kFx(kStatus) kString_AddSubstring(kString str, const kChar* content, kSize start, kSize count)
{
    kObj(kString, str); 
    kSize newLength = obj->length + count;

    kCheck(kString_Reserve(str, newLength));
    
    kMemCopy(obj->chars + obj->length, content + start, count * sizeof(kChar));

    obj->chars[newLength] = 0; 
    obj->length = newLength;

    return kOK;
}

kFx(kStatus) xkString_SplitAdd(const kChar* start, kSize length, kArrayList tokens, kAlloc allocator, kBool discardEmpty)
{
    kString token = kNULL; 
    kStatus exception;

    kTry
    {
        if (!discardEmpty || (length > 0))
        {
            kTest(kString_Construct(&token, kNULL, allocator));
            kTest(kString_AddSubstring(token, start, 0, length));

            kTest(kArrayList_AddT(tokens, &token));
        }
    }
    kCatch(&exception)
    {
        kObject_Destroy(token); 
        kEndCatch(exception); 
    }

    return kOK; 
}

kFx(kStatus) kString_SplitEx(kString str, const kChar* delimiters, kArrayList* tokens, kBool discardEmpty, kAlloc allocator)
{
    kObj(kString, str); 
    const kChar* start = kString_Chars(obj); 
    const kChar* it = start;
    kArrayList output = kNULL; 
    kStatus exception;
    
    kTry
    {
        kTest(kArrayList_Construct(&output, kTypeOf(kString), 0, allocator)); 
        
        while (*it != 0)
        {
            const kChar* delimIt = delimiters; 

            while (*delimIt != 0)
            {
                if (*it == *delimIt++)
                {
                    kTest(xkString_SplitAdd(start, (kSize)(it-start), output, allocator, discardEmpty)); 
                    start = it+1; 
                    break;
                }
            }
            ++it; 
        }    
        
        kTest(xkString_SplitAdd(start, (kSize)(it-start), output, allocator, discardEmpty)); 

        *tokens = output; 
    }
    kCatch(&exception)
    {
        kObject_Dispose(output); 
        kEndCatch(exception); 
    }

    return kOK; 
}



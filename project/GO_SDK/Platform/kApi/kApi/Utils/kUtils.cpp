/*
 * @file    kUtils.cpp
 * 
 * The Zen API 
 * Copyright (C) 2008-2021 by LMI Technologies Inc.
 * 
 * Licensed under The MIT License.
 * Redistributions of files must retain the above copyright notice.
 */
#define K_PLATFORM
#include <kApi/Utils/kUtils.h>
#include <kApi/Data/kArray1.h>
#include <kApi/Data/kArray2.h>
#include <kApi/Data/kArray3.h>
#include <kApi/Data/kArrayList.h>
#include <kApi/Data/kBox.h>
#include <kApi/Data/kMath.h>
#include <kApi/Data/kImage.h>
#include <kApi/Data/kString.h>
#include <kApi/Io/kDat5Serializer.h>
#include <kApi/Io/kDat6Serializer.h>
#include <kApi/Io/kFile.h>
#include <kApi/Io/kMemory.h>
#include <kApi/Io/kSerializer.h>
#include <kApi/Threads/kThread.h>
#include <kApi/Threads/kTimer.h>
#include <kApi/Utils/kBackTrace.h>
#include <kApi/Utils/kDateTime.h>
#include <kApi/Utils/kUserAlloc.h>
#include <kApi/Data/kCollection.h>
#include <float.h>
#include <math.h>
#include <stdio.h>
#include <time.h>

kBeginStaticClassEx(k, kUtils)    
kEndStaticClassEx()

kBeginValueEx(k, kLogArgs)
kEndValueEx()

kFx(kStatus) xkUtils_InitStatic()
{
    //if a custom random number generator has not be registered, initialize the standard 
    //generator and set a default handler
    if (kIsNull(kApiLib_RandomHandler()))
    {
        k32u seed = (k32u)time(kNULL) ^ (k32u)xkGetCurrentProcessId(); 

        srand(seed); 

        kCheck(kApiLib_SetRandomHandler(xkDefaultRandom)); 
    }

    return kOK; 
}

kFx(kStatus) xkUtils_ReleaseStatic()
{    
    return kOK; 
}

kFx(kStatus) kMemAlloc(kSize size, void* mem)
{
    kPointer_WriteAs(mem, kNULL, kPointer); 

    if (size > 0)
    {
        kAlloc alloc = kAlloc_App(); 
        void* allocation = kNULL; 
        kSize allocationSize = kALIGN_ANY_SIZE + size; 

        kCheck(kAlloc_Get(alloc, allocationSize, &allocation)); 

        kPointer_WriteAs(allocation, alloc, kObject); 
        kPointer_WriteAs(mem, kPointer_ByteOffset(allocation, kALIGN_ANY_SIZE), kPointer); 
    }

    return kOK; 
}

kFx(kStatus) kMemAllocZero(kSize size, void* mem)
{
    kCheck(kMemAlloc(size, mem)); 
    kCheck(kMemSet(kPointer_ReadAs(mem, kPointer), 0, size)); 

    return kOK; 
}

kFx(kStatus) kMemFree(void* mem)
{
    if (!kIsNull(mem))
    {
        void* allocation = kPointer_ByteOffset(mem, -kALIGN_ANY_SIZE); 
        kAlloc alloc = kPointer_ReadAs(allocation, kObject); 

        kCheck(kAlloc_Free(alloc, allocation)); 
    }
    
    return kOK; 
}

kFx(kStatus) kMemFreeRef(void* mem)
{
    kCheck(kMemFree(kPointer_ReadAs(mem, kPointer))); 

    kPointer_WriteAs(mem, kNULL, kPointer); 

    return kOK; 
}

kFx(kStatus) kLoad5(kObject* object, const kChar* fileName, kAlloc allocator)
{
    return kSerializer_LoadObject(object, kTypeOf(kDat5Serializer), fileName, allocator); 
}

kFx(kStatus) kSave5(kObject object, const kChar* fileName)
{
    return kSerializer_SaveObject(object, kTypeOf(kDat5Serializer), fileName); 
}

kFx(kStatus) kSaveCompressed5(kObject object, const kChar* fileName, kCompressionType algorithm, k32s level)
{
    return kDat5Serializer_SaveCompressed(object, fileName, algorithm, level); 
}

kFx(kStatus) kLoad6(kObject* object, const kChar* fileName, kAlloc allocator)
{
    return kSerializer_LoadObject(object, kTypeOf(kDat6Serializer), fileName, allocator); 
}

kFx(kStatus) kSave6(kObject object, const kChar* fileName)
{
    return kSerializer_SaveObject(object, kTypeOf(kDat6Serializer), fileName); 
}

kFx(kStatus) kSaveCompressed6(kObject object, const kChar* fileName, kCompressionType algorithm, k32s level)
{
    return kDat6Serializer_SaveCompressed(object, fileName, algorithm, level); 
}

kFx(kStatus) kMemReverseCopy(void* dest, const void* src, kSize size)
{
    kByte* writer = (kByte*) dest;
    kByte* writerEnd = writer + size; 
    const kByte* reader = kCast(kByte*, src) + (size - 1); 

    kAssert((size == 0) || (!kIsNull(dest) && !kIsNull(src))); 

    while (writer != writerEnd)
    {
        *writer++ = *reader--; 
    }

    return kOK; 
}

kFx(kStatus) kMemReverse(void* buffer, kSize size)
{
    kByte* head = (kByte*) buffer;
    kByte* headEnd = head + (size / 2); 
    kByte* tail = head + (size - 1); 

    kAssert((size == 0) || !kIsNull(buffer)); 

    while (head != headEnd)
    {
        kByte temp = *head; 

        *head++ = *tail; 
        *tail-- = temp; 
    }

    return kOK; 
}

kFx(kStatus) kMemSet(void* dest, kByte fill, kSize size)
{
    kAssert((size == 0) || !kIsNull(dest)); 

    if (size >= xkMEM_COPY_THRESHOLD)
    {
        kApiMemSetFx handler = kApiLib_MemSetHandler();

        if (!kIsNull(handler))
        {
            return handler(dest, fill, size);
        }
    }

    memset(dest, fill, size);

    return kOK;
}

kFx(kStatus) kMemCopy(void* dest, const void* src, kSize size)
{
    kAssert((size == 0) || (!kIsNull(dest) && !kIsNull(src))); 

    if (size >= xkMEM_COPY_THRESHOLD)
    {
        kApiMemCopyFx handler = kApiLib_MemCopyHandler(); 

        if (!kIsNull(handler))
        {
            return handler(dest, src, size); 
        }
    }
    
    memcpy(dest, src, size); 

    return kOK; 
}

kFx(kStatus) kMemMove(void* dest, const void* src, kSize size)
{
    kAssert((size == 0) || (!kIsNull(dest) && !kIsNull(src))); 

    if (size >= xkMEM_COPY_THRESHOLD)
    {
        kApiMemCopyFx handler = kApiLib_MemMoveHandler();

        if (!kIsNull(handler))
        {
            return handler(dest, src, size);
        }
    }

    memmove(dest, src, size);
   
    return kOK;
}

kFx(kStatus) xkStrCopyEx(kChar* dest, kSize capacity, const kChar* src, kChar** endPos)
{
    kChar* destLast = dest + capacity - 1;  

    kCheckArgs(!kIsNull(dest) && (capacity > 0)); 

    if (kIsNull(src))
    {
        *dest = 0; 
        return kERROR_PARAMETER; 
    }

    while ((*src != 0) && (dest != destLast))
    {
        *dest++ = *src++; 
    }

    if (!kIsNull(endPos))
    {
        *endPos = dest;
    }

    *dest = 0; 

    return (*src == 0) ? kOK : kERROR_INCOMPLETE; 
}

kFx(kStatus) kStrCopy(kChar* dest, kSize capacity, const kChar* src)
{
    return xkStrCopyEx(dest, capacity, src, kNULL); 
}

kFx(kStatus) kStrCat(kChar* dest, kSize capacity, const kChar* src)
{
    kSize length = kStrLength(dest); 

    return kStrCopy(&dest[length], capacity-length, src); 
}

kFx(kStatus) kStrToLower(kChar* str)
{
    while (*str != 0)
    {
        *str = kChar_ToLower(*str); 
        str++; 
    }

    return kOK; 
}

kFx(kBool) kStrEquals(const kChar* a, const kChar* b)
{
    return (strcmp(a, b) == 0); 
}

kFx(kBool) kStrnEquals(const kChar* a, const kChar* b, kSize maxCount)
{
    return (strncmp(a, b, maxCount) == 0); 
}

kFx(k32s) kStrCompare(const kChar* a, const kChar* b)
{
    return strcmp(a, b); 
}

kFx(k32s) kStrCompareN(const kChar* a, const kChar* b, kSize maxCount)
{
    return strncmp(a, b, maxCount); 
}

kFx(k32s) kStrCompareLower(const kChar* a, const kChar* b)
{
    while (kChar_ToLower(*a) == kChar_ToLower(*b))
    {
        if (*a == 0)
        {
            return 0; 
        }

        a++; 
        b++; 
    }

    return kChar_ToLower(*a) - kChar_ToLower(*b);  
}

kFx(kSize) kStrLength(const kChar* str)
{
    return strlen(str); 
}

kFx(const kChar*) kStrFindFirst(const kChar* str, const kChar* subStr)
{
    return strstr(str, subStr); 
}

kFx(const kChar*) kStrFindLast(const kChar* str, const kChar* subStr)
{
    kSize strSize = strlen(str); 
    kSize subSize = strlen(subStr); 
    const kChar* it = str + (strSize - subSize); 
    kSize i; 

    while (it >= str)
    {
        for (i = 0; i < subSize; ++i)
        {
            if (it[i] != subStr[i])
            {
                break; 
            }
        }

        if (i == subSize)
        {
            return it; 
        }

        it--; 
    }

    return kNULL; 
}

kFx(kStatus) kStrPrintf(kChar* dest, kSize capacity, const kChar* format, ...)
{
    kVarArgList argList; 
    kStatus status; 
   
    kVarArgList_Start(argList, format);
    {
        status = kStrPrintvf(dest, capacity, format, argList); 
    }
    kVarArgList_End(argList); 

    return status; 
}

kFx(kStatus) kBase64Encode(const void* buffer, kSize size, kString base64String)
{
    static const kChar base64EncodingTable[] = {
        'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H',
        'I', 'J', 'K', 'L', 'M', 'N', 'O', 'P',
        'Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X',
        'Y', 'Z', 'a', 'b', 'c', 'd', 'e', 'f',
        'g', 'h', 'i', 'j', 'k', 'l', 'm', 'n',
        'o', 'p', 'q', 'r', 's', 't', 'u', 'v',
        'w', 'x', 'y', 'z', '0', '1', '2', '3',
        '4', '5', '6', '7', '8', '9', '+', '/' };

    const kByte* s = (const kByte*)buffer;
    kChar* p = kNULL;
    k32s pos = 2;
    k32u val = 0;
    kSize i;

    kCheck(kString_SetLength(base64String, 4 * kDivideCeilUInt_(size, 3))); 

    p = kString_Chars(base64String);

    for (i = 0; i < size; i++)
    {
        //build up a 24-bit value
        val |= s[i] << (pos * 8);   
        pos--;

        //if we have 24 bits, emit  
        if (pos < 0)
        {
            *p++ = base64EncodingTable[(val >> 18) & 0x3f];
            *p++ = base64EncodingTable[(val >> 12) & 0x3f];
            *p++ = base64EncodingTable[(val >> 6) & 0x3f];
            *p++ = base64EncodingTable[val & 0x3f];

            pos = 2;
            val = 0;
        }
    }

    //handle any trailing bytes, adding padding as needed
    if (pos != 2)
    {
        *p++ = base64EncodingTable[(val >> 18) & 0x3f];
        *p++ = base64EncodingTable[(val >> 12) & 0x3f];
        *p++ = (pos == 0) ? base64EncodingTable[(val >> 6) & 0x3f] : '=';
        *p++ = '=';
    }
    
    kAssert((kSize)(p - kString_Chars(base64String)) == kString_Length(base64String));   

    return kOK;
}

#if defined(K_MSVC)

kFx(kStatus) kStrPrintvf(kChar* dest, kSize capacity, const kChar* format, kVarArgList argList)
{
    kSSize written; 
   
    kCheckArgs(capacity > 0); 
        
    written = _vsnprintf(dest, capacity, format, argList); 
    
    if (written < 0 || written >= (kSSize) capacity)
    {
        dest[capacity-1] = 0; 
    }

    return (written < 0 || written >= (kSSize) capacity) ? kERROR_INCOMPLETE : kOK; 
}

#else

kFx(kStatus) kStrPrintvf(kChar* dest, kSize capacity, const kChar* format, kVarArgList argList)
{
    kSSize written; 
   
    kCheckArgs(capacity > 0); 
        
    written = vsnprintf(dest, capacity, format, argList); 

    if (written >= (kSSize) capacity)
    {
        dest[capacity-1] = 0; 
    }

    return (written >= (kSSize)capacity) ? kERROR_INCOMPLETE : kOK; 
}

#endif

#if defined(K_MSVC)

kFx(k32s) xkStrMeasuref(const kChar* format, kVarArgList argList)
{
    return _vscprintf(format, argList); 
}

#else

kFx(k32s) xkStrMeasuref(const kChar* format, kVarArgList argList)
{
    return vsnprintf(kNULL, 0, format, argList);
}

#endif

kFx(kStatus) kLogf(const kChar* format, ...)
{
    kVarArgList argList; 
    kStatus status; 

    kVarArgList_Start(argList, format);
    {
        status = kLogvf(format, argList); 
    }
    kVarArgList_End(argList);
    
    return status; 
}

kFx(kStatus) kLog(const kChar* message)
{
    return kLogf("%s", message);
}

kFx(kStatus) kLogvf(const kChar* format, kVarArgList argList)
{
    return xkLogvf(0, "", format, argList);
}

kFx(kStatus) xkLogvf(kLogOption options, const kChar* source, const kChar* format, kVarArgList argList)
{
    kText256 buffer = { 0 };
    kLogArgs args = { 0 };
    
    //flatten to buffer
    kStrPrintvf(buffer, sizeof(buffer), format, argList);

    args.message = buffer;
    args.options = options;
    args.source = kIsNull(source) ? "" : source;
    args.dateTime = kDateTime_Now();
    args.upTime = kTimer_Now();

    return xkApiLib_Log(&args);
}

kFx(kStatus) kLogBackTrace(kSize skip)
{
    kBackTrace trace = kNULL; 
    kArrayList lines = kNULL; 
    kSize i; 

    kTry
    {
        kTest(kBackTrace_Construct(&trace, kNULL)); 

        kTest(kBackTrace_Capture(trace, skip)); 
        kTest(kBackTrace_Describe(trace, &lines, kNULL));
        
        for (i = 0; i < kArrayList_Count(lines); ++i)
        {
            kLogf("%s", kString_Chars(kArrayList_AsT(lines, i, kString))); 
        }
    }
    kFinally
    {
        kObject_Dispose(lines); 
        kObject_Destroy(trace); 

        kEndFinally();
    }

    return kOK; 
}

kFx(kStatus) xkLogBackTrace(kLogOption options, const kChar* source, kSize skip)
{
    kBackTrace trace = kNULL; 
    kArrayList lines = kNULL; 
    kSize i; 

    kTry
    {
        kTest(kBackTrace_Construct(&trace, kNULL)); 

        kTest(kBackTrace_Capture(trace, skip)); 
        kTest(kBackTrace_Describe(trace, &lines, kNULL));
        
        for (i = 0; i < kArrayList_Count(lines); ++i)
        {
            kLogf(options, source, "%s", kString_Chars(kArrayList_AsT(lines, i, kString))); 
        }
    }
    kFinally
    {
        kObject_Dispose(lines); 
        kObject_Destroy(trace); 

        kEndFinally();
    }

    return kOK;
}

kFx(kStatus) xkLog(const kChar* source, const kChar* message)
{
    return kLogf(kLOG_OPTION_PLATFORM, source, "%s", message);
}

kFx(kStatus) xkLogf(const kChar* source, const kChar* format, ...)
{
    kVarArgList argList;
    kStatus status;

    kVarArgList_Start(argList, format);
    {
        status = kLogvf(kLOG_OPTION_PLATFORM, source, format, argList); 
    }
    kVarArgList_End(argList);

    return status;
}

kFx(kStatus) xkLogWarning(const kChar* source, const kChar* message)
{
    return kLog(kLOG_OPTION_PLATFORM | kLOG_OPTION_WARNING, source, message);
}

kFx(kStatus) xkLogfWarning(const kChar* source, const kChar* format, ...)
{
    kVarArgList argList;
    kStatus status;

    kVarArgList_Start(argList, format);
    {
        status = kLogvf(kLOG_OPTION_PLATFORM | kLOG_OPTION_WARNING, source, format, argList); 
    }
    kVarArgList_End(argList);

    return status;
}

kFx(kStatus) xkLogError(const kChar* source, const kChar* message)
{
    return kLog(kLOG_OPTION_PLATFORM | kLOG_OPTION_ERROR, source, message);
}

kFx(kStatus) xkLogfError(const kChar* source, const kChar* format, ...)
{
    kVarArgList argList;
    kStatus status;

    kVarArgList_Start(argList, format);
    {
        status = kLogvf(kLOG_OPTION_PLATFORM | kLOG_OPTION_ERROR, source, format, argList); 
    }
    kVarArgList_End(argList);

    return status;
}

kFx(kStatus) kReallocContent(kType type, void* itemsRef, kSize* allocSize, kSize newCount, kAlloc allocator)
{
    kSize itemSize = kType_Size(type); 
    kAlloc alloc = kAlloc_Fallback(allocator); 
    kSize oldSize = *allocSize; 
    kSize newSize = kMax_(oldSize, newCount*itemSize); 
    kByte* oldItems = *(kByte**)itemsRef; 
    kByte* newItems = oldItems; 
   
    if (newSize > oldSize)
    {
        kCheck(kAlloc_Get(alloc, newSize, &newItems));         
        kCheck(kAlloc_Free(alloc, oldItems)); 
    }
        
    if (kType_IsReference(type))
    {
        kMemSet(newItems, 0, itemSize*newCount);  
    }

    *(void**)itemsRef = newItems; 
    *allocSize = newSize; 
    
    return kOK; 
}

kFx(kStatus) xkOverrideFunctions(void* base, kSize baseSize, void* overrides)
{
    kSize count = baseSize / sizeof(kFunction); 
    kFunction* baseFx = (kFunction*) base; 
    kFunction* overrideFx = (kFunction*) overrides; 
    kSize i; 

    for (i = 0; i < count; ++i)
    {
        if (!kIsNull(overrideFx[i]))
        {
            baseFx[i] = overrideFx[i]; 
        }
    }

    return kOK; 
}

kFx(kStatus) kZeroItems(kType type, void* items, kSize count)
{
    return kMemSet(items, 0, count*kType_Size(type)); 
}

kFx(kStatus) xkCopyItems(kType type, void* dest, const void* src, kSize count)
{
    return kMemCopy(dest, src, count*kType_Size(type)); 
}

kFx(kStatus) xkCopyItemsEx(kType type, kAlloc destAlloc, void* dest, kAlloc srcAlloc, const void* src, kSize count, kObject context)
{
    return kAlloc_Copy(destAlloc, dest, srcAlloc, src, count*kType_Size(type), context); 
}

kFx(kStatus) xkCloneItems(kType type, void* dest, const void* src, kSize count, kAlloc allocator)
{
    if (kType_IsValue(type))
    {
        kCheck(kMemCopy(dest, src, count*kType_Size(type))); 
    }
    else
    {
        kAlloc alloc = kAlloc_Fallback(allocator); 
        const kObject* in = (const kObject*) src; 
        kObject* out = (kObject*) dest;  
        kStatus status; 
        kSize i = 0; 

        kTry
        {
            for (i = 0; i < count; ++i)
            {
                kTest(kObject_Clone(&out[i], in[i], alloc));                  
            }
        }
        kCatch(&status)
        {
            kDisposeItems(type, dest, i); 
            kZeroItems(type, dest, count); 
            kEndCatch(status); 
        }
    }       

    return kOK; 
}

kFx(kStatus) xkCloneItemsEx(kType type, kAlloc dstAlloc, void* dest, kAlloc srcAlloc, const void* src, kSize count, kObject context, kAlloc destObjectAlloc, kAlloc destValueAlloc)
{
    if (kType_IsValue(type))
    {
        kCheck(kAlloc_Copy(dstAlloc, dest, srcAlloc, src, count*kType_Size(type), context)); 
    }
    else
    {
        const kObject* in = (const kObject*) src; 
        kObject* out = (kObject*) dest;  
        kStatus status; 
        kSize i = 0; 

        kTry
        {
            for (i = 0; i < count; ++i)
            {
                kTest(kObject_Clone(&out[i], in[i], destObjectAlloc, destValueAlloc, context));                  
            }
        }
        kCatch(&status)
        {
            kDisposeItems(type, dest, i); 
            kZeroItems(type, dest, count); 
            kEndCatch(status); 
        }
    }       

    return kOK; 
}

kFx(kStatus) kDisposeItems(kType type, void* items, kSize count)
{
    if (kType_IsReference(type))
    {
        kObject* elements = (kObject*) items; 
        kSize i; 

        for (i = 0; i < count; ++i)
        {
            if (!kIsNull(elements[i]))
            {
                kCheck(kObject_Dispose(elements[i]));             
                elements[i] = kNULL; 
            }
        }
    }

    return kOK; 
}

kFx(kStatus) kShareItems(kType type, void* items, kSize count)
{
    if (kType_IsReference(type))
    {
        kObject* elements = (kObject*) items; 
        kSize i; 

        for (i = 0; i < count; ++i)
        {
            if (!kIsNull(elements[i]))
            {
                kCheck(kObject_Share(elements[i]));             
            }
        }
    }

    return kOK; 
}

kFx(kSize) kMeasureItems(kType type, const void* items, kSize count)
{
    kSize size = 0; 

    if (kType_IsReference(type))
    {
        const kObject* elements = (const kObject*) items; 
        kSize i; 

        for (i = 0; i < count; ++i)
        {
            if (!kIsNull(elements[i]))
            {
                size += kObject_Size(elements[i]); 
            }
        }
    }

    return size; 
}

kFx(kBool) kHasSharedItems(kType type, const void* items, kSize count)
{
    if (kType_IsReference(type))
    {
        auto elements = (const kObject*) items; 

        for (kSize i = 0; i < count; ++i)
        {
            if (!kIsNull(elements[i]))
            {
                if (kObject_HasShared(elements[i]))
                {
                    return kTRUE;
                } 
            }
        }
    }

    return kFALSE; 
}

kFx(kAllocTrait) kEnumerateAllocTraits(kType type, const void* items, kSize count)
{
    kAllocTrait traits = 0; 

    if (kType_IsReference(type))
    {
        const kObject* elements = (const kObject*) items; 
        kSize i; 

        for (i = 0; i < count; ++i)
        {
            if (!kIsNull(elements[i]))
            {
                traits |= kObject_AllocTraits(elements[i]);
            }
        }
    }

    return traits;
}

/* http://en.wikipedia.org/wiki/Jenkins_hash_function -- one-at-a-time */
kFx(kSize) xkHashBytes(const void* key, kSize length)
{
    const kByte* input = (const kByte*) key; 
    kSize hash = 0; 
    kSize i = 0; 
    
    for (i = 0; i < length; ++i)
    {
        hash += input[i];
        hash += (hash << 10);
        hash ^= (hash >> 6);
    }

    hash += (hash << 3);
    hash ^= (hash >> 11);
    hash += (hash << 15);
    
    return hash;
}

kFx(k32u) xkReverseBits32(k32u input, k32u bitCount)
{
    k32u output = 0; 
    k32u i; 

    for (i = 1; i <= bitCount; ++i)
    {
        output |= (input & 1) << (bitCount - i); 
        input = input >> 1; 
    }

    return output; 
}

kFx(kStatus) xkDefaultMemAlloc(kPointer receiver, kSize size, void* mem, kMemoryAlignment alignment)
{
    *(void**)mem = kNULL; 
    
    if (size > 0)
    {
        kPointer allocation = kNULL;

        kCheck(xkUtils_AlignedAlloc(&allocation, size, alignment));
        
        kAssert(((kSize)allocation & (kMemoryAlignment_Size(alignment) - 1)) == 0);

        *(void**)mem = allocation; 
    }

    return kOK; 
}

kFx(kStatus) xkDefaultMemFree(kPointer receiver, void* mem)
{
    xkUtils_AlignedFree(mem);

    return kOK; 
}

kFx(kStatus) xkDefaultAssert(const kChar* file, k32u line)
{
    kLogf("\n\nAssertion: %s, line %u\n", file, line);    
     
    kLogBackTrace(2); 

    kLog("");

    //give a second to let log output flush
    kThread_Sleep(1000000); 

    assert(0);

    return kOK;
}

kFx(kStatus) xkSysMemAlloc(kSize size, void* mem)
{
    kApiMemAllocFx handler = kApiLib_MemAllocHandler(); 
    kPointer provider = kApiLib_MemAllocProvider(); 

    kCheck(handler(provider, size, mem, kALIGN_ANY)); 

    kCheck(kMemSet(*(void**)mem, 0, size)); 

    return kOK; 
}

kFx(kStatus) xkSysMemFree(void* mem)
{
    kApiMemFreeFx handler = kApiLib_MemFreeHandler(); 
    kPointer provider = kApiLib_MemAllocProvider(); 

    return handler(provider, mem); 
}

kFx(kStatus) xkSysMemReallocList(void* list, kSize count, kSize itemSize, kSize* capacity, kSize initialCapacity, kSize requiredCapacity)
{
    const kSize GROWTH_FACTOR = 2; 
    kSize oldCapacity = *capacity; 

    if (oldCapacity < requiredCapacity)
    {
        kSize newCapacity = 0; 
        void* newList = kNULL;  

        newCapacity = kMax_(requiredCapacity, GROWTH_FACTOR*oldCapacity);
        newCapacity = kMax_(newCapacity, initialCapacity); 

        kCheck(xkSysMemAlloc(newCapacity*itemSize, &newList)); 

        kMemCopy(newList, *(void**)list, itemSize*count); 
        
        xkSysMemFree(*(void**)list); 

        *(void**)list = newList; 
        *capacity = newCapacity; 
    }

    return kOK; 
}

kFx(k32u) xkDefaultRandom()
{
    //standard guarantees that rand will generate at least 15 bits
    return ((k32u)rand() << 17) ^ ((k32u)rand() << 8) ^ ((k32u)rand()); 
}

kFx(k32u) kRandom32u()
{
    return kApiLib_RandomHandler()(); 
}

kFx(k64u) kRandom64u()
{
    k64u lower = kRandom32u(); 
    k64u upper = kRandom32u(); 

    return (upper << 32) | lower; 
}

kFx(kSize) kRandomSize()
{
    return (K_POINTER_SIZE == 8) ? (kSize)kRandom64u() : (kSize)kRandom32u(); 
}

kFx(kStatus) kRandomBytes(void* data, kSize length)
{
    kApiRandomFx generator = kApiLib_RandomHandler(); 
    kByte* it = (kByte*) data; 
    kByte* end = it + length; 
    kByte* optEnd = it + ((length >> 2) << 2);  
    
    //process as much as possible in 32-bit increments; 
    //uses memcpy (rather than casting to k3u*) to avoid violating strict aliasing rules 
    while (it != optEnd)
    {
        k32u random = generator(); 

        kItemCopy(it, &random, 4); 

        it += 4; 
    }

    //handle any remaining bytes
    while (it != end)
    {
        *it++ = (k8u) generator(); 
    }

    return kOK; 
}

#if defined (K_DARWIN)

kFx(kStatus) xkFormatTimeout(k64u timeout, struct timespec* tm)
{
    struct timeval tv;
    k64u absTime = 0; 
    
    if (gettimeofday(&tv, kNULL) != 0)
    {
        return kERROR; 
    }
    
    absTime = k64U(1000000)*tv.tv_sec + tv.tv_usec;
    absTime += timeout; 
    
    tm->tv_sec = (time_t) (absTime / 1000000); 
    tm->tv_nsec =  (long)(absTime % 1000000) * 1000; 
    
    return kOK; 
}

#elif defined (K_POSIX)

kFx(kStatus) xkFormatTimeout(k64u timeout, struct timespec* tm)
{
    k64u absTime; 
    
    if (clock_gettime(CLOCK_REALTIME, tm) != 0)
    {
        return kERROR; 
    }

    absTime = k64U(1000000)*tm->tv_sec + tm->tv_nsec/k64U(1000);
    absTime += timeout; 

    tm->tv_sec = (time_t) (absTime / 1000000); 
    tm->tv_nsec =  (long)(absTime % 1000000) * 1000; 

    return kOK;  
}

#endif

kFx(void) xkUpdateProgress(kCallbackFx progress, kPointer receiver, kPointer sender, k64u* updateTime, k32u progressValue)
{
    if (!kIsNull(progress))
    {
        const k64u updateIntervalUs = 1000000;
        k64u now = kTimer_Now(); 
        kBool shouldUpdate = kIsNull(updateTime) || (*updateTime == k64U_NULL) || ((now - *updateTime) >= updateIntervalUs); 

        if (shouldUpdate)
        {
            progress(receiver, sender, &progressValue); 

            if (!kIsNull(updateTime))
            {
                *updateTime = now; 
            }
        }       
    }
}

kFx(kStatus) xkStrFormat8u(k8u value, kChar* buffer, kSize capacity, kChar** endPos)
{
    return xkStrFormat64u(value, buffer, capacity, endPos);
}

kFx(kStatus) xkStrParse8u(k8u* value, const kChar* str)
{
    k64u temp;

    kCheck(k64u_Parse(&temp, str));

    *value = (k8u) temp;
    return kOK;
}

kFx(kStatus) xkStrFormat8s(k8s value, kChar* buffer, kSize capacity, kChar** endPos)
{
    return xkStrFormat64s(value, buffer, capacity, endPos);
}

kFx(kStatus) xkStrParse8s(k8s* value, const kChar* str)
{
    k64s temp;

    kCheck(k64s_Parse(&temp, str));

    *value = (k8s) temp;
    return kOK;
}

kFx(kStatus) xkStrFormat16u(k16u value, kChar* buffer, kSize capacity, kChar** endPos)
{
    return xkStrFormat64u(value, buffer, capacity, endPos);
}

kFx(kStatus) xkStrParse16u(k16u* value, const kChar* str)
{
    k64u temp;

    kCheck(k64u_Parse(&temp, str));

    *value = (k16u) temp;
    return kOK;
}

kFx(kStatus) xkStrFormat16s(k16s value, kChar* buffer, kSize capacity, kChar** endPos)
{
    return xkStrFormat64s(value, buffer, capacity, endPos);
}

kFx(kStatus) xkStrParse16s(k16s* value, const kChar* str)
{
    k64s temp;

    kCheck(k64s_Parse(&temp, str));

    *value = (k16s) temp;
    return kOK;
}

kFx(kStatus) xkStrFormat32u(k32u value, kChar* buffer, kSize capacity, kChar** endPos)
{
    return xkStrFormat64u(value, buffer, capacity, endPos);
}

kFx(kStatus) xkStrParse32u(k32u* value, const kChar* str)
{
    k64u temp;

    kCheck(k64u_Parse(&temp, str));

    *value = (k32u) temp;
    return kOK;
}

kFx(kStatus) xkStrFormat32s(k32s value, kChar* buffer, kSize capacity, kChar** endPos)
{
    return xkStrFormat64s(value, buffer, capacity, endPos);
}

kFx(kStatus) xkStrParse32s(k32s* value, const kChar* str)
{
    k64s temp;

    kCheck(k64s_Parse(&temp, str));

    *value = (k32s) temp;
    return kOK;
}

kFx(kStatus) xkStrFormat64u(k64u value, kChar* buffer, kSize capacity, kChar** endPos)
{
    kChar tmpBuf[21];
    kChar* writePos = &tmpBuf[sizeof(tmpBuf) - 1];

    *writePos = 0;

    do
    {
        *--writePos = (kChar)((value % 10) + '0');

        value /= 10; 
    }
    while (value != 0);

    return xkStrCopyEx(buffer, capacity, writePos, endPos);
}

kFx(kStatus) xkStrParse64u(k64u* value, const kChar* str)
{
    kSize len = kStrLength(str);
    k64u v = 0;

    if (len == 0)
    {
        return kERROR_FORMAT;
    }

    while (len && kChar_IsSpace(*str))
    {
        str++;
        len--;
    }

    while (len && *str >= '0' && *str <= '9')
    {
        v = v * 10 + *str - '0';
        len--;
        str++;
    }

    *value = v;
    return kOK;
}

kFx(kStatus) xkStrFormat64s(k64s value, kChar* buffer, kSize capacity, kChar** endPos)
{
    kChar tmpBuf[21];
    kChar* writePos = &tmpBuf[sizeof(tmpBuf) - 1];
    k64s sign = kMath_Sign_(value);

    *writePos = 0;

    do
    {
        *--writePos = (kChar)(sign * (value % 10) + '0');

        value /= 10;
    }
    while (value != 0);

    if (sign < 0)
    {
        *--writePos = '-';
    }

    return xkStrCopyEx(buffer, capacity, writePos, endPos);
}

kFx(kStatus) xkStrParse64s(k64s* value, const kChar* str)
{
    kSize len = kStrLength(str);
    kBool isNegative = kFALSE;
    k64u v = 0;
    kChar tmp;

    if (len == 0)
    {
        return kERROR_FORMAT;
    }

    while (len && kChar_IsSpace(*str))
    {
        str++;
        len--;
    }

    if (len)
    {
        if (*str == '-' || *str == '+')
        {
            isNegative = (*str == '-');
            str++;
            len--;
        }
    }

    while (len && (tmp = *str - '0') >= 0 && tmp <= 9)
    {
        v = v * 10 + tmp;
        len--;
        str++;
    }

    if (isNegative)
    {
        *value = - (k64s) v;
    }
    else
    {
        *value = (k64s) v;
    }

    return kOK;
}

kFx(kStatus) xkStrFormatBool(kBool value, kChar* buffer, kSize capacity, kChar** endPos)
{
    return xkStrFormat32s(!!value, buffer, capacity, endPos);
}

kFx(kStatus) xkStrParseBool(kBool* value, const kChar* str)
{
    k32s temp;

    kCheck(k32s_Parse(&temp, str));

    *value = (kBool) !!temp;
    return kOK;
}

kFx(kStatus) xkStrFormatSize(kSize value, kChar* buffer, kSize capacity, kChar** endPos)
{
    return xkStrFormat64u(value, buffer, capacity, endPos);
}

kFx(kStatus) xkStrParseSize(kSize* value, const kChar* str)
{
    k64u temp;

    kCheck(k64u_Parse(&temp, str));

    *value = (kSize) temp;
    return kOK;
}

kFx(kStatus) xkStrFormatSSize(kSSize value, kChar* buffer, kSize capacity, kChar** endPos)
{
    return xkStrFormat64s(value, buffer, capacity, endPos);
}

kFx(kStatus) xkStrParseSSize(kSSize* value, const kChar* str)
{
    k64s temp;

    kCheck(k64s_Parse(&temp, str));

    *value = (kSSize) temp;
    return kOK;
}

kFx(kStatus) xkStrFormat32f(k32f value, kChar* buffer, kSize capacity, k32s digitCount, kChar** endPos)
{
    return xkStrFormat64f(value, buffer, capacity, digitCount, endPos);
}

kFx(kStatus) xkStrParse32f(k32f* value, const kChar* str)
{
    k64f temp;

    kCheck(k64f_Parse(&temp, str));

    *value = (k32f) temp;
    return kOK;
}

kFx(kStatus) xkStrFormat64f(k64f value, kChar* buffer, kSize capacity, k32s digitCount, kChar** endPos)
{
    kChar valueBuffer[100];
    kChar formatBuffer[510];
    kChar* pos = formatBuffer + sizeof(formatBuffer) - 1;
    k32s exp = 0;
    k32s i;
    kBool isNegative;
    kBool digitFound = kFALSE;
    k32s decPt;
    kChar* result = kNULL;

    kCheckTrue(capacity > 1, kERROR_PARAMETER);

    if (digitCount == 0) digitCount = 1;
    if (digitCount < 0)  digitCount = 6;

    *(pos--) = '\0'; 

    if (k64f_IsNanOrInf(value))
    {
        return xkStrCopyEx(buffer, capacity, "NAN", endPos);
    }

    kCheck(xkStrEcvt(value, valueBuffer, kCountOf(valueBuffer), digitCount, &decPt, &isNegative, &result));

    if (decPt < -3 || decPt > digitCount)
    {
        k64u expValue;

        for (; decPt > 1; decPt--, exp++);
        for (; decPt < 1; decPt++, exp--);

        expValue = (k64u) ((exp > 0) ? exp : -exp);

        do
        {
            *(pos--) = (kChar)(expValue % 10 + '0');
        }
        while (expValue /= 10);

        if (exp < 10 && exp > -10)
        {
            *(pos--) = '0';
        }

        *(pos--) = (exp < 0) ? '-' : '+';
        *(pos--) = 'e';
    }

    for (i = (k32s) kStrLength(result) - 1; i >= decPt; --i)
    {
        kChar digit = (i >= 0) ? result[i] : '0'; 

        if (digit != '0' || digitFound)
        {
            *(pos--) = digit;

            digitFound = kTRUE;
        }
    }

    if (digitFound)
    {
        *(pos--) = '.';
    }

    if (decPt > 0)
    {
        kChar* tmpPtr = result + decPt - 1;

        for (; tmpPtr >= result; tmpPtr--)
        {
            *(pos--) = *tmpPtr;
        }
    }
    else
    {
        *(pos--) = '0';
    }

    if (isNegative)
    {
        *(pos--) = '-'; 
    }

    return xkStrCopyEx(buffer, capacity, pos+1, endPos);
}

kFx(kStatus) xkStrParse64f(k64f* value, const kChar* str)
{
    kStatus status = kOK;
    k64f result = 0;
    kChar tmp;
    const kChar* readPos = str;
    k32s exp = 0;
    k32s count = 0;
    kBool isValid = kFALSE;
    kBool isNegative;

    if (kStrLength(str) == 0)
    {
        return kERROR_FORMAT;
    }

    while (kChar_IsSpace(*readPos))
    {
        ++readPos;
    }

    if ((isNegative = ((tmp = *readPos) == '-')) || (tmp == '+'))
    {
        ++readPos;
        isValid = kTRUE;
    }

    for (; (tmp = *readPos) >= '0' && tmp <= '9'; ++readPos)
    {
        result = result * 10 + tmp - '0';
        isValid = kTRUE;
    }

    if (tmp == '.')
    {
        while ((tmp = *++readPos) >= '0' && tmp <= '9')
        {
            result = result * 10 + tmp - '0'; 
            isValid = kTRUE;
            --exp;
        }
    }

    if (isNegative)
    {
        result = -result;
    }

    if (isValid && kChar_ToLower(*readPos) == 'e')
    {
        if ((isNegative = ((tmp = *++readPos) == '-')) || (tmp == '+'))
        {
            tmp = *++readPos;
        }
 
        for (count = 0; tmp >= '0' && tmp <= '9'; tmp = *++readPos)
        {
            if ((k32S_MAX - abs(exp) - (tmp - '0')) / 10 > count)
            {
                count *= 10; 
                count += tmp - '0';
            }
            else
            {
                count = k32S_MAX - exp;
                break;
            }
        }

        if (isNegative)  exp -= count;
        else             exp += count;
    }

    if (result != 0.0)
    {
        if (exp > DBL_MAX_10_EXP)
        {
            status = kERROR_FORMAT;

            result = (result < 0) ? -HUGE_VAL : HUGE_VAL;
        }
        else if (exp < DBL_MIN_10_EXP)
        {
            status = kERROR_FORMAT;

            result = 0.0;
        }
        else if (exp < 0)
        {
            result /= pow(10.0, -exp);
        }
        else
        {
            result *= pow(10.0, exp);
        }
    }

    *value = result;
    return status;
}

kFx(kStatus) xkStrEcvt(k64f value, kChar* buffer, kSize capacity, k32s digitCount, k32s* decPt, kBool* isNegative, kChar** endPos)
{
    kChar* pos = kNULL;
    k32s digits = 0;
    k32s temp;

    buffer[0] = '0';
    digitCount++;

    if ((*isNegative = (value < 0)))
    {
        value = -value;
    }

    while (value > k32S_MAX) { value /= 10; digits++; }
    while (value && value < 1) { value *= 10; digits--; }

    kCheck(xkStrFormat32s((k32s) value, buffer+1, capacity-1, &pos));

    temp = (k32s) (pos - buffer) - 1;
    *decPt  = digits + temp;

    if (temp >= (kSSize) digitCount)
    {
        pos = buffer + digitCount + 1;
    }
    else if ((digitCount -= temp) > 0) 
    {
        do
        {
            value -= (k64s)value;
            *pos++ = (k8s)(value *= 10.0) + '0';
        } 
        while (--digitCount);
    }

    if (*--pos >= '5')
    {
        kChar* ptr = pos;
        while ((*--ptr += 1) > '9') *ptr = '0';

        if (ptr == buffer)
        {
            *--pos = 0;
            *decPt += 1;
            *endPos = buffer;
            return kOK;
        }
    }

    *pos = 0;
    *endPos = buffer + 1;
    return kOK;
}

#if defined(K_WINDOWS)

kFx(k64u) xkGetCurrentProcessId()
{
    return (k64u) GetCurrentProcessId();
}

#elif defined(K_POSIX)

kFx(k64u) xkGetCurrentProcessId()
{
    return (k64u) getpid();
}

#else

kFx(k64u) xkGetCurrentProcessId()
{
    return 0; 
}

#endif

#if defined(K_VX_KERNEL)

kFx(kStatus) xkUtils_ErrnoToStatus(k32s err)
{
    switch (err)
    {
        case S_memLib_NOT_ENOUGH_MEMORY:     return kERROR_MEMORY;
        default:                             return kERROR_OS;
    }
}

#else

kFx(kStatus) xkUtils_ErrnoToStatus(k32s err)
{
    return kERROR_OS;
}

#endif


//FSS-1181: DllExport datatype accessors
//The following methods are provided as a short-term solution for users that require non-inline equivalents of specific 
//functions.  

kFx(kType) xkObject_Type_NonInline(kObject object)
{
    return kObject_Type(object);
}

kFx(const kChar*) xkType_Name_NonInline(kType type)
{
    return kType_Name(type);
}

kFx(void*) xkArray1_At_NonInline(kArray1 array, kSize index)
{
    return kArray1_At(array, index);
}

kFx(kSize) xkArray1_Count_NonInline(kArray1 array)
{
    return kArray1_Count(array);
}

kFx(kType) xkArray1_ItemType_NonInline(kArray1 array)
{
    return kArray1_ItemType(array);
}

kFx(void*) xkArray2_At_NonInline(kArray2 array, kSize index0, kSize index1)
{
    return kArray2_At(array, index0, index1);
}

kFx(kSize) xkArray2_Length_NonInline(kArray2 array, kSize dimension)
{
    return kArray2_Length(array, dimension);
}

kFx(kSize) xkArray2_Count_NonInline(kArray2 array)
{
    return kArray2_Count(array);
}

kFx(kType) xkArray2_ItemType_NonInline(kArray2 array)
{
    return kArray2_ItemType(array); 
}

kFx(void*) xkArray3_At_NonInline(kArray3 array, kSize index0, kSize index1, kSize index2)
{
    return kArray3_At(array, index0, index1, index2);
}

kFx(kSize) xkArray3_Length_NonInline(kArray3 array, kSize dimension)
{
    return kArray3_Length(array, dimension);
}

kFx(kSize) xkArray3_Count_NonInline(kArray3 array)
{
    return kArray3_Count(array);
}

kFx(kType) xkArray3_ItemType_NonInline(kArray3 array)
{
    return kArray3_ItemType(array);
}

kFx(void*) xkArrayList_At_NonInline(kArrayList list, kSize index)
{
    return kArrayList_At(list, index);
}

kFx(kSize) xkArrayList_Count_NonInline(kArrayList list)
{
    return kArrayList_Count(list);
}

kFx(kType) xkArrayList_ItemType_NonInline(kArrayList list)
{
    return kArrayList_ItemType(list);
}

kFx(kType) xkBox_ItemType_NonInline(kBox box)
{
    return kBox_ItemType(box);
}

kFx(kChar*) xkString_Chars_NonInline(kString str)
{
    return kString_Chars(str);
}

kFx(kSize) xkImage_Height_NonInline(kImage image)
{
    return kImage_Height(image);
}

kFx(kSize) xkImage_Width_NonInline(kImage image)
{
    return kImage_Width(image);
}

kFx(kType) xkImage_PixelType_NonInline(kImage image)
{
    return kImage_PixelType(image);
}

kFx(void*) xkImage_At_NonInline(kImage image, kSize x, kSize y)
{
    return kImage_At(image, x, y);
}

kFx(kBool) xkCollection_HasNext_NonInline(kCollection collection, kIterator iterator)
{
    return kCollection_HasNext(collection, iterator);
}

kFx(void*) xkCollection_Next_NonInline(kCollection collection, kIterator* iterator)
{
    return kCollection_Next(collection, iterator);
}

#ifdef K_POSIX

kFx(kStatus) xkUtils_AlignedAlloc(kPointer* allocation, kSize size, kMemoryAlignment alignment)
{
    if (alignment <= kALIGN_ANY)
    {
        *allocation = malloc(size);

        kCheckTrue(!kIsNull(*allocation) || (size == 0), kERROR_MEMORY);
    }
    else
    {
        void *mem = kNULL;

        kCheckTrue(posix_memalign(&mem, kMemoryAlignment_Size(alignment), size) == 0, kERROR_MEMORY);
        *allocation = mem;
    }

    return kOK;
}

kFx(void) xkUtils_AlignedFree(void* mem)
{
    free(mem);
}

#elif defined(K_MSVC)

kFx(kStatus) xkUtils_AlignedAlloc(kPointer* allocation, kSize size, kMemoryAlignment alignment)
{
    *allocation = _aligned_malloc(size, kMemoryAlignment_Size(alignment));

    kCheckTrue(!kIsNull(*allocation) || (size == 0), kERROR_MEMORY);
    
    return kOK;
}

kFx(void) xkUtils_AlignedFree(void* mem)
{
    return _aligned_free(mem);
}

#else

kFx(kStatus) xkUtils_AlignedAlloc(kPointer* allocation, kSize size, kMemoryAlignment alignment)
{
    //attempt allocation; it's unclear whether the requested alignment will be supported by the underlying system
    *allocation = malloc(size);

    kCheckTrue(!kIsNull(*allocation) || (size == 0), kERROR_MEMORY);

    //verify that the requested alignment is respected
    if ((((kSize)*allocation) & (kMemoryAlignment_Size(alignment) - 1)) != 0)
    {
        xkUtils_AlignedFree(*allocation);
        *allocation = kNULL;

        return kERROR_UNIMPLEMENTED;
    }

    return kOK;
}

kFx(void) xkUtils_AlignedFree(void* mem)
{
    free(mem);
}
#endif
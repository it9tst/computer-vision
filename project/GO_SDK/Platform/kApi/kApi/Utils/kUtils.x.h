/** 
 * @file    kUtils.x.h
 *
 * @internal
 * Copyright (C) 2008-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_UTILS_X_H
#define K_API_UTILS_X_H

#define xkMEM_COPY_THRESHOLD                (2048)

typedef struct kUtilsStatic
{
    k32u placeHolder;       //unused
} kUtilsStatic; 

kDeclareStaticClassEx(k, kUtils)

kFx(kStatus) xkUtils_InitStatic(); 
kFx(kStatus) xkUtils_ReleaseStatic(); 

kFx(kStatus) xkOverrideFunctions(void* base, kSize baseSize, void* overrides); 

kFx(k32s) xkStrMeasuref(const kChar* format, kVarArgList argList); 

kFx(kSize) xkHashBytes(const void* key, kSize length); 

kInlineFx(kSize) xkHashPointer(kPointer pointer)
{   
    return (((kSize)pointer) >> kALIGN_ANY) | 
            ((kSize)pointer) << ((8*K_POINTER_SIZE)-kALIGN_ANY);
}

kFx(k32u) xkReverseBits32(k32u input, k32u bitCount); 

kFx(kStatus) xkDefaultMemAlloc(kPointer receiver, kSize size, void* mem, kMemoryAlignment alignment);
kFx(kStatus) xkDefaultMemFree(kPointer receiver, void* mem); 
kFx(kStatus) xkDefaultAssert(const kChar* file, k32u line); 

kFx(kStatus) xkStrCopyEx(kChar* dest, kSize capacity, const kChar* src, kChar** endPos);

/** 
 * Utility function, used internally by kApi for memory allocation. 
 *
 * This function calls the memory allocation callback provided via kApiLib_SetMemAllocHandlers. 
 * 
 * Most memory allocations in the kApi library are performed using the kAlloc_App allocator, 
 * including allocations performed via the kMemAlloc function.  But any allocations that are 
 * required before the kApi type system is initialized should use xkSysMemAlloc instead.
 * 
 * This function ensures that allocated memory is initialized to zero.
 * 
 * @param   size    Size of memory to allocate, in bytes.
 * @param   mem     Receives a pointer to the memory block.
 * @return          Operation status. 
 */
kFx(kStatus) xkSysMemAlloc(kSize size, void* mem);

/** 
 * Utility function, used internally by kApi for memory deallocation. 
 *
 * This function calls the memory deallocation callback provided via kApiLib_SetMemAllocHandlers.
 * 
 * This function should be used to free any memory allocated with the xkSysMemAlloc function.
 *
 * @param   mem    Pointer to memory to free (or kNULL). 
 * @return         Operation status. 
 */
kFx(kStatus) xkSysMemFree(void* mem);

/** 
 * Utility function, used internally by kApi to resize an array-based list of elements.
 *
 * Most logic in the kApi library that requires managing a dynamically-resizing list 
 * should use the kArrayList class. But if a dynamic list is required before the 
 * kApi type system is initialized, the xkSysMemReallocList function can be used instead.
 * 
 * Use xkSysMemFree to deallocate any memory allocated with this function.
 *
 * @param   list                Pointer to pointer to first list element (or pointer to kNULL). 
 * @param   count               Count of existing list elements that should be preserved.
 * @param   itemSize            Size of each list element, in bytes.
 * @param   capacity            On input, the current list capacity, in elements; on output, the new list capacity. 
 * @param   initialCapacity     If the list has not yet been allocated, the initial capacity that should be used, in elements.
 * @param   requiredCapacity    The minimum list capacity after reallocation, in elements.
 * @return                      Operation status. 
 */
kFx(kStatus) xkSysMemReallocList(void* list, kSize count, kSize itemSize, kSize* capacity, kSize initialCapacity, kSize requiredCapacity); 

/** 
 * Default random number generator. 
 * 
 * @return      Returns a random 32-bit number. 
 */
kFx(k32u) xkDefaultRandom(); 

/** 
 * Converts from microseconds to kernel time units. 
 * 
 * This function isn't used on POSIX-based systems. 
 * 
 * @param   time    Time, in microseconds (can be kINFINITE).  
 * @return          Time, in kernel time units. 
 */
kInlineFx(k32u) xkTimeToKernelTime(k64u time)
{
    return xkApiLib_timerScaleFx(time);
}

/** 
 * Utility function to dipatch progress udpates at a limited time interval. 
 * 
 * Regardless of the frequency at which this function is called, updates are dispatched at no more 
 * than one second intervals.
 * 
 * @public                  @memberof kUtils
 * @param   progress        Optional progress callback function.
 * @param   receiver        Progress callback receiver. 
 * @param   sender          Progress callback sender. 
 * @param   updateTime      Time of most recent update (in/out); if null, update is not time-limited.
 * @param   progressValue   Current progress percentage.
 */
kFx(void) xkUpdateProgress(kCallbackFx progress, kPointer receiver, kPointer sender, k64u* updateTime, k32u progressValue);

kFx(kStatus) xkStrFormat8u(k8u value, kChar* buffer, kSize capacity, kChar** endPos);
kFx(kStatus) xkStrFormat8s(k8s value, kChar* buffer, kSize capacity, kChar** endPos);
kFx(kStatus) xkStrFormat16u(k16u value, kChar* buffer, kSize capacity, kChar** endPos);
kFx(kStatus) xkStrFormat16s(k16s value, kChar* buffer, kSize capacity, kChar** endPos);
kFx(kStatus) xkStrFormat32u(k32u value, kChar* buffer, kSize capacity, kChar** endPos);
kFx(kStatus) xkStrFormat32s(k32s value, kChar* buffer, kSize capacity, kChar** endPos);
kFx(kStatus) xkStrFormat64u(k64u value, kChar* buffer, kSize capacity, kChar** endPos);
kFx(kStatus) xkStrFormat64s(k64s value, kChar* buffer, kSize capacity, kChar** endPos);
kFx(kStatus) xkStrFormatBool(kBool value, kChar* buffer, kSize capacity, kChar** endPos);
kFx(kStatus) xkStrFormatSize(kSize value, kChar* buffer, kSize capacity, kChar** endPos);
kFx(kStatus) xkStrFormatSSize(kSSize value, kChar* buffer, kSize capacity, kChar** endPos);
kFx(kStatus) xkStrFormat32f(k32f value, kChar* buffer, kSize capacity, k32s digitCount, kChar** endPos);
kFx(kStatus) xkStrFormat64f(k64f value, kChar* buffer, kSize capacity, k32s digitCount, kChar** endPos);

kFx(kStatus) xkStrParse8u(k8u* value, const kChar* str);
kFx(kStatus) xkStrParse8s(k8s* value, const kChar* str);
kFx(kStatus) xkStrParse16u(k16u* value, const kChar* str);
kFx(kStatus) xkStrParse16s(k16s* value, const kChar* str);
kFx(kStatus) xkStrParse32u(k32u* value, const kChar* str);
kFx(kStatus) xkStrParse32s(k32s* value, const kChar* str);
kFx(kStatus) xkStrParse64u(k64u* value, const kChar* str);
kFx(kStatus) xkStrParse64s(k64s* value, const kChar* str);
kFx(kStatus) xkStrParseBool(kBool* value, const kChar* str);
kFx(kStatus) xkStrParseSize(kSize* value, const kChar* str);
kFx(kStatus) xkStrParseSSize(kSSize* value, const kChar* str);
kFx(kStatus) xkStrParse64f(k64f* value, const kChar* str);
kFx(kStatus) xkStrParse32f(k32f* value, const kChar* str);

kFx(kStatus) xkStrEcvt(k64f value, kChar* buffer, kSize capacity, k32s digitCount, k32s* decPt, kBool* isNegative, kChar** endPos);

kFx(kStatus) xkLogvf(kLogOption options, const kChar* source, const kChar* format, kVarArgList argList);
kFx(kStatus) xkLogBackTrace(kLogOption options, const kChar* source, kSize skip);

//adds log entry with kLOG_OPTION_PLATFORM option
kFx(kStatus) xkLog(const kChar* source, const kChar* message);

//adds formatted log entry with kLOG_OPTION_PLATFORM option
kFx(kStatus) xkLogf(const kChar* source, const kChar* format, ...);

//adds log entry with kLOG_OPTION_PLATFORM | kLOG_OPTION_WARNING options
kFx(kStatus) xkLogWarning(const kChar* source, const kChar* message);

//adds formatted log entry with kLOG_OPTION_PLATFORM | kLOG_OPTION_WARNING options
kFx(kStatus) xkLogfWarning(const kChar* source, const kChar* format, ...);

//adds log entry with kLOG_OPTION_PLATFORM | kLOG_OPTION_ERROR options
kFx(kStatus) xkLogError(const kChar* source, const kChar* message);

//adds formatted log entry with kLOG_OPTION_PLATFORM | kLOG_OPTION_ERROR options
kFx(kStatus) xkLogfError(const kChar* source, const kChar* format, ...);

#if defined(K_POSIX) && defined(K_PLATFORM)
kFx(kStatus) xkFormatTimeout(k64u timeout, struct timespec* ts); 
#endif

kInlineFx(kByte) xkReverseBits8(kByte byte)
{
    return (kByte) ((((byte) & 0x80) >> 7) |
                    (((byte) & 0x40) >> 5) |
                    (((byte) & 0x20) >> 3) |
                    (((byte) & 0x10) >> 1) |
                    (((byte) & 0x08) << 1) |
                    (((byte) & 0x04) << 3) |
                    (((byte) & 0x02) << 5) |
                    (((byte) & 0x01) << 7)); 
}

kFx(k64u) xkGetCurrentProcessId();

kFx(kStatus) xkUtils_ErrnoToStatus(k32s err);

kFx(kStatus) xkCopyItems(kType type, void* dest, const void* src, kSize count);
kFx(kStatus) xkCopyItemsEx(kType type, kAlloc destAlloc, void* dest, kAlloc srcAlloc, const void* src, kSize count, kObject context);

kFx(kStatus) xkCloneItems(kType type, void* dest, const void* src, kSize count, kAlloc allocator);
kFx(kStatus) xkCloneItemsEx(kType type, kAlloc dstAlloc, void* dest, kAlloc srcAlloc, const void* src, kSize count, kObject context, kAlloc destObjectAlloc, kAlloc destValueAlloc);

//FSS-1181: DllExport datatype accessors
//The following methods are provided as a short-term solution for users that require non-inline equivalents of specific 
//functions.  

kFx(kType) xkObject_Type_NonInline(kObject object); 
kFx(const kChar*) xkType_Name_NonInline(kType type);
kFx(void*) xkArray1_At_NonInline(kArray1 array, kSize index);
kFx(kSize) xkArray1_Count_NonInline(kArray1 array);
kFx(kType) xkArray1_ItemType_NonInline(kArray1 array);
kFx(void*) xkArray2_At_NonInline(kArray2 array, kSize index0, kSize index1);
kFx(kSize) xkArray2_Length_NonInline(kArray2 array, kSize dimension);
kFx(kSize) xkArray2_Count_NonInline(kArray2 array);
kFx(kType) xkArray2_ItemType_NonInline(kArray2 array);
kFx(void*) xkArray3_At_NonInline(kArray3 array, kSize index0, kSize index1, kSize index2);
kFx(kSize) xkArray3_Length_NonInline(kArray3 array, kSize dimension);
kFx(kSize) xkArray3_Count_NonInline(kArray3 array);
kFx(kType) xkArray3_ItemType_NonInline(kArray3 array); 
kFx(void*) xkArrayList_At_NonInline(kArrayList list, kSize index);
kFx(kSize) xkArrayList_Count_NonInline(kArrayList list);
kFx(kType) xkArrayList_ItemType_NonInline(kArrayList list); 
kFx(kType) xkBox_ItemType_NonInline(kBox box);
kFx(kChar*) xkString_Chars_NonInline(kString str);
kFx(kSize) xkImage_Height_NonInline(kImage image);
kFx(kSize) xkImage_Width_NonInline(kImage image);
kFx(kType) xkImage_PixelType_NonInline(kImage image);
kFx(void*) xkImage_At_NonInline(kImage image, kSize x, kSize y);
kFx(kBool) xkCollection_HasNext_NonInline(kCollection collection, kIterator iterator);
kFx(void*) xkCollection_Next_NonInline(kCollection collection, kIterator* iterator);

//counts the number of bits in a 64-bit word
kInlineFx(k32u) xkCountBits64(k64u v);

#if defined(K_MSVC) && (K_POINTER_SIZE == 8)

kInlineFx(k32u) xkCountBits64(k64u v)
{
    return (k32u) __popcnt64(v); 
}

#elif defined (K_GCC)

kInlineFx(k32u) xkCountBits64(k64u v)
{
    return (k32u) __builtin_popcountll(v); 
}

#else

//https://en.wikipedia.org/wiki/Hamming_weight
kInlineFx(k32u) xkCountBits64(k64u v)
{
    const k64u m1  = 0x5555555555555555; //binary: 0101...
    const k64u m2  = 0x3333333333333333; //binary: 00110011..
    const k64u m4  = 0x0f0f0f0f0f0f0f0f; //binary:  4 zeros,  4 ones ...
    const k64u h01 = 0x0101010101010101; //the sum of 256 to the power of 0,1,2,3...

    v -= (v >> 1) & m1;             //put count of each 2 bits into those 2 bits
    v = (v & m2) + ((v >> 2) & m2); //put count of each 4 bits into those 4 bits 
    v = (v + (v >> 4)) & m4;        //put count of each 8 bits into those 8 bits 
    return (v * h01) >> 56;         //returns left 8 bits of v + (v<<8) + (v<<16) + (v<<24) + ... 
}

#endif

kFx(kStatus) xkUtils_AlignedAlloc(kPointer* allocation, kSize size, kMemoryAlignment alignment);
kFx(void) xkUtils_AlignedFree(void* mem);

#endif

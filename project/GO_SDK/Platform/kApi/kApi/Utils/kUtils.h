/** 
 * @file    kUtils.h
 * @brief   Utility functions.
 *
 * @internal
 * Copyright (C) 2008-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <kApi/kApiDef.h>         //--inclusion order controlled by kApiDef

#ifndef K_API_UTILS_H
#define K_API_UTILS_H

#include <kApi/Utils/kUtils.x.h>

/**
 * @class   kUtils
 * @extends kObject
 * @ingroup kApi-Utils
 * @brief   Collection of utility functions. 
 */

/** 
 * Destroys an object and resets the object handle to kNULL. 
 *
 * @public              @memberof kUtils
 * @param   object      Pointer to object, or pointer to kNULL. 
 * @return              Operation status.
 */
kInlineFx(kStatus) kDestroyRef(kObject* object)
{
    kStatus result = kObject_Destroy(*object);     

    *object = kNULL; 

    return result; 
}

/** 
 * Disposes an object and resets the object handle to kNULL. 
 *
 * @public              @memberof kUtils
 * @param   object      Pointer to object, or pointer to kNULL. 
 * @return              Operation status.
 */
kInlineFx(kStatus) kDisposeRef(kObject* object)
{
    kStatus result = kObject_Dispose(*object);     

    *object = kNULL; 

    return result; 
}

/** 
 * Shares an object and sets a handle to refer to the shared object. 
 *
 * @public              @memberof kUtils
 * @param   object      Receives shared object handle. 
 * @param   source      Object to be shared (or kNULL). 
 * @return              Operation status.
 */
kInlineFx(kStatus) kShareRef(kObject* object, kObject source)
{
    if (!kIsNull(source))
    {
        kCheck(kObject_Share(source)); 

        if (!kIsNull(object))
        {
            *object = source; 
        }
    }
    else
    {
        *object = kNULL; 
    }

    return kOK; 
}

/** 
 * Replaces a reference to existing object with a new object. 
 * 
 * The existing object is disposed, the new object is shared, and the existing object reference
 * is replaced with a reference to the new object.
 *
 * @public              @memberof kUtils
 * @param   object      Pointer to object handle to be replaced.
 * @param   source      Replacement object. 
 * @return              Operation status.
 */
kInlineFx(kStatus) kReplaceRef(kObject* object, kObject source)
{
    kCheck(kDisposeRef(object)); 

    kCheck(kShareRef(object, source)); 

    return kOK; 
}

/** 
 * Zeros the memory associated with a generic array of items. 
 * 
 * Values and/or object handles are set to zero.
 * 
 * @public              @memberof kUtils
 * @param   type        Item type.
 * @param   items       Array of items. 
 * @param   count       Count of items. 
 * @return              Operation status.
 */
kFx(kStatus) kZeroItems(kType type, void* items, kSize count); 

/** 
 * Performs a shallow copy of a generic array of items. 
 * 
 * Values and/or object handles are copied.
 * 
 * @public              @memberof kUtils
 * @param   type        Item type.
 * @param   dest        Destination array of items. 
 * @param   src         Source array of items. 
 * @param   count       Count of items. 
 * @return              Operation status.
 */
kInlineFx(kStatus) kCopyItems(kType type, void* dest, const void* src, kSize count)
{
    return xkCopyItems(type, dest, src, count);
}

/** 
 * Performs a shallow copy of a generic array of items. 
 * 
 * Values and/or object handles are copied.
 * 
 * @public              @memberof kUtils
 * @param   type        Item type.
 * @param   destAlloc   Destination memory allocator. 
 * @param   dest        Destination array of items. 
 * @param   srcAlloc    Source memory allocator. 
 * @param   src         Source array of items. 
 * @param   count       Count of items. 
 * @param   context     Context for copy operation (allocator specific; may be required by some foreign domain allocators).  
 * @return              Operation status.
 */
#if defined (K_CPP)
kInlineFx(kStatus) kCopyItems(kType type, kAlloc destAlloc, void* dest, kAlloc srcAlloc, const void* src, kSize count, kObject context)
{
    return xkCopyItemsEx(type, destAlloc, dest, srcAlloc, src, count, context);
}
#endif

/** 
 * Performs a deep copy of a generic array of items. 
 * 
 * Value types are copied; reference types are cloned. 
 * 
 * @public              @memberof kUtils
 * @param   type        Item type.
 * @param   dest        Destination array of items. 
 * @param   src         Source array of items. 
 * @param   count       Count of items. 
 * @param   allocator   Memory allocator. 
 * @return              Operation status.
 */
kInlineFx(kStatus) kCloneItems(kType type, void* dest, const void* src, kSize count, kAlloc allocator)
{
    return xkCloneItems(type, dest, src, count, allocator);
}

/** 
 * Performs a deep copy of a generic array of items. 
 * 
 * Value types are copied; reference types are cloned. If source value types were allocated in 
 * foreign memory, will attempt to export values from foreign memory to local memory using the 
 * source allocator's kAlloc_Export implementation.
 * 
 * @public                      @memberof kUtils
 * @param   type                Item type.
 * @param   destAlloc           Allocator associated with destination memory. 
 * @param   dest                Destination for the memory copy.
 * @param   srcAlloc            Allocator associated with source memory. 
 * @param   src                 Source for the memory copy.
 * @param   count               Count of items. 
 * @param   context             Context for copy operation (allocator specific; may be required by some foreign domain allocators).  
 * @param   destObjectAlloc     Object memory allocator (passed to kObject_Clone for newly-cloned objects).
 * @param   destValueAlloc      Value memory allocator (passed to kObject_Clone for newly-cloned objects). 
 * @return              Operation status.
 */
#if defined (K_CPP)
kInlineFx(kStatus) kCloneItems(kType type, kAlloc destAlloc, void* dest, kAlloc srcAlloc, const void* src, kSize count, kObject context, kAlloc destObjectAlloc, kAlloc destValueAlloc)
{
    return xkCloneItemsEx(type, destAlloc, dest, srcAlloc, src, count, context, destObjectAlloc, destValueAlloc);
}
#endif

/** 
 * Disposes a generic array of items. 
 * 
 * Reference types are disposed; value types are ignored.
 * 
 * @public              @memberof kUtils
 * @param   type        Item type.
 * @param   items       Array of items. 
 * @param   count       Count of items. 
 * @return              Operation status.
 */
kFx(kStatus) kDisposeItems(kType type, void* items, kSize count); 

/** 
 * Increments the reference count of a generic array of items. 
 * 
 * Value types are ignored.
 * 
 * @public              @memberof kUtils
 * @param   type        Item type.
 * @param   items       Array of items. 
 * @param   count       Count of items. 
 * @return              Operation status.
 */
kFx(kStatus) kShareItems(kType type, void* items, kSize count); 

/** 
 * Calculates the total size, in bytes, of a generic array of items.
 * 
 * Calls kObject_Size for each object. Value types are ignored. Does 
 * not include the size of the array itself. 
 * 
 * @public              @memberof kUtils
 * @param   type        Item type.
 * @param   items       Array of items. 
 * @param   count       Count of items. 
 * @return              Operation status.
 */
kFx(kSize) kMeasureItems(kType type, const void* items, kSize count); 

/** 
 * Determines whether a list of items contains any shared objects.
 * 
 * Calls kObject_HasShared for each object. Value types are ignored. 
 * 
 * @public              @memberof kUtils
 * @param   type        Item type.
 * @param   items       Array of items. 
 * @param   count       Count of items. 
 * @return              Operation status.
 */
kFx(kBool) kHasSharedItems(kType type, const void* items, kSize count); 

/** 
 * Gets the bitset of allocator traits associated with objects in the specified array.
 * 
 * Calls kObject_AllocTraits for each object. Value types are ignored.
 * 
 * @public              @memberof kUtils
 * @param   type        Item type.
 * @param   items       Array of items. 
 * @param   count       Count of items. 
 * @return              kTRUE if any object references foreign memory.
 */
kFx(kBool) kEnumerateAllocTraits(kType type, const void* items, kSize count);

/** 
 * Loads an object from file using kDat-5 serialization.
 *
 * @public              @memberof kUtils
 * @param   object      Receives deserialized object.
 * @param   fileName    Path of the file to load. 
 * @param   allocator   Memory allocator to use for loaded object (or kNULL for default). 
 * @return              Operation status.
 */
kFx(kStatus) kLoad5(kObject* object, const kChar* fileName, kAlloc allocator); 

/** 
 * Saves an object to file using kDat-5 serialization.
 *
 * @public              @memberof kUtils
 * @param   object      Object to be serialized.
 * @param   fileName    Path of the file to save. 
 * @return              Operation status.
 */
kFx(kStatus) kSave5(kObject object, const kChar* fileName); 

/** 
 * Saves an object to file using kDat-5 serialization and compression.
 *
 * @public              @memberof kUtils
 * @param   object      Object to be serialized.
 * @param   fileName    Path of the file to save. 
 * @param   algorithm   Compression algorithm to use. Example: kCOMPRESSION_TYPE_ZSTD. 
 * @param   level       Compression level to use. Example: kCOMPRESSION_PRESET_DEFAULT. 
 * @return              Operation status.
 */
kFx(kStatus) kSaveCompressed5(kObject object, const kChar* fileName, kCompressionType algorithm, k32s level); 

/** 
 * Loads an object from file using kDat-6 serialization.
 *
 * @public              @memberof kUtils
 * @param   object      Receives deserialized object.
 * @param   fileName    Path of the file to load. 
 * @param   allocator   Memory allocator to use for loaded object (or kNULL for default). 
 * @return              Operation status.
 */
kFx(kStatus) kLoad6(kObject* object, const kChar* fileName, kAlloc allocator); 

/** 
 * Saves an object to file using kDat-6 serialization.
 *
 * @public              @memberof kUtils
 * @param   object      Object to be serialized.
 * @param   fileName    Path of the file to save. 
 * @return              Operation status.
 */
kFx(kStatus) kSave6(kObject object, const kChar* fileName); 

/** 
 * Saves an object to file using kDat-6 serialization and compression.
 *
 * @public              @memberof kUtils
 * @param   object      Object to be serialized.
 * @param   fileName    Path of the file to save.
 * @param   algorithm   Compression algorithm to use. Example: kCOMPRESSION_TYPE_ZSTD. 
 * @param   level       Compression level to use. Example: kCOMPRESSION_PRESET_DEFAULT. 
 * @return              Operation status.
 */
kFx(kStatus) kSaveCompressed6(kObject object, const kChar* fileName, kCompressionType algorithm, k32s level); 

/** 
 * Allocates a block of memory from the application heap.
 *
 * Memory allocated with this function should be freed with the kMemFree function.
 *
 * @public              @memberof kUtils
 * @param   size        Size of memory to allocate, in bytes.
 * @param   mem         Receives pointer to allocated memory (pointer to a pointer).
 * @return              Operation status.
 */
kFx(kStatus) kMemAlloc(kSize size, void* mem);

/** 
 * Allocates and zero-initializes block of memory from the application heap.
 *
 * Memory allocated with this function should be freed with the kMemFree function.
 *
 * @public              @memberof kUtils
 * @param   size        Size of memory to allocate, in bytes.
 * @param   mem         Receives pointer to allocated memory (pointer to a pointer).
 * @return              Operation status.
 */
kFx(kStatus) kMemAllocZero(kSize size, void* mem);

/** 
 * Frees a block of memory that was allocated using kMemAlloc or kMemAllocZero. 
 *
 * @public              @memberof kUtils
 * @param   mem         Pointer to memory to free (or kNULL). 
 * @return              Operation status.
 */
kFx(kStatus) kMemFree(void* mem); 

/** 
 * Frees a block of memory that was allocated using kMemAlloc or kMemAllocZero and resets the memory pointer to kNULL. 
 *
 * @public              @memberof kUtils
 * @param   mem         Pointer to pointer to memory to free (or pointer to kNULL). 
 * @return              Operation status.
 */
kFx(kStatus) kMemFreeRef(void* mem); 

/** 
 * Sets a block of memory to the given byte value. 
 *
 * @public              @memberof kUtils
 * @param   dest        Destination for the memory set operation.
 * @param   fill        Value to be set.
 * @param   size        Size of memory block to be set, in bytes.
 * @return              Operation status (generally safe to assume successful).
 */
kFx(kStatus) kMemSet(void* dest, kByte fill, kSize size);

/** 
 * Sets a block of memory to zero. 
 *
 * @public              @memberof kUtils
 * @param   dest        Destination for the memory set operation.
 * @param   size        Size of memory block to be zeroed, in bytes.
 * @return              Operation status.
 */
kInlineFx(kStatus) kMemZero(void* dest, kSize size)
{
    return kMemSet(dest, 0, size);
}

/** 
 * Copies memory from a source buffer to a non-overlapping destination.  
 *
 * @public              @memberof kUtils
 * @param   dest        Destination for the memory copy.
 * @param   src         Source for the memory copy.
 * @param   size        Size of memory block to be copied, in bytes.
 * @return              Operation status. 
 */
kFx(kStatus) kMemCopy(void* dest, const void* src, kSize size);

/** 
 * Copies memory from a source buffer to a potentially-overlapping destination.  
 *
 * @public              @memberof kUtils
 * @param   dest        Destination for the memory copy.
 * @param   src         Source for the memory copy.
 * @param   size        Size of memory block to be copied, in bytes.
 * @return              Operation status.
 */
kFx(kStatus) kMemMove(void* dest, const void* src, kSize size);

/** 
 * Copies memory in reverse from a source buffer to a non-overlapping destination.  
 *
 * @public              @memberof kUtils
 * @param   dest        Destination for the memory copy.
 * @param   src         Source for the memory copy.
 * @param   size        Size of memory block to be reverse-copied, in bytes.
 * @return              Operation status. 
 */
kFx(kStatus) kMemReverseCopy(void* dest, const void* src, kSize size);

/** 
 * Reverse the order of bytes in a buffer.  
 *
 * @public              @memberof kUtils
 * @param   buffer      Buffer to reverse.
 * @param   size        Size of memory block to be reversed, in bytes.
 * @return              Operation status. 
 */
kFx(kStatus) kMemReverse(void* buffer, kSize size);

/** 
 * Compares one memory buffer with another. 
 *
 * @public              @memberof kUtils
 * @param   a           First buffer. 
 * @param   b           Second buffer. 
 * @param   size        Size of memory buffers to be compared, in bytes.
 * @return              kTRUE if the memory buffers are equal; otherwise, kFALSE. 
 */
kInlineFx(kBool) kMemEquals(const void* a, const void* b, kSize size)
{
    return (memcmp(a, b, size) == 0); 
}

/** 
 * Copies characters from source to destination. 
 * 
 * If the buffer is insufficient, the copy will transfer as many characters
 * as possible, null-terminate the resulting string, and return kERROR_INCOMPLETE. 
 * 
 * If the destination capacity is zero, kERROR_PARAMETER will be returned. 
 *
 * @public              @memberof kUtils
 * @param   dest        Destination for the string copy.
 * @param   capacity    Capacity of destination buffer, in characters.
 * @param   src         Source for the string copy.
 * @return              Operation status. 
 */
kFx(kStatus) kStrCopy(kChar* dest, kSize capacity, const kChar* src); 

/** 
 * Appends characters from source to destination. 
 * 
 * If the buffer is insufficient, the concatenation will transfer as many characters
 * as possible, null-terminate the resulting string, and return kERROR_INCOMPLETE. 
 * 
 * If the destination capacity is zero, kERROR_PARAMETER will be returned. 
 *
 * @public              @memberof kUtils
 * @param   dest        Destination string to append to.
 * @param   capacity    Capacity of destination buffer, in characters.
 * @param   src         Source string to append.
 * @return              Operation status. 
 */
kFx(kStatus) kStrCat(kChar* dest, kSize capacity, const kChar* src); 

/** 
 * Converts characters in the given sequence to lower case. 
 *
 * This function currently supports conversion of characters only within the ASCII character range. 
 *
 * @public          @memberof kUtils
 * @param   str     Character sequence to convert.
 * @return          Operation status. 
 */
kFx(kStatus) kStrToLower(kChar* str); 

/** 
 * Tests a pair of character sequences for equality. 
 * 
 * @public           @memberof kUtils
 * @param   a        First string. 
 * @param   b        Second string. 
 * @return           kTRUE if the character sequences are equal; otherwise, kFALSE. 
 */
kFx(kBool) kStrEquals(const kChar* a, const kChar* b); 

/** 
 * Tests a pair of character sequences for equality, up to a maximum number of characters. 
 * 
 * @public           @memberof kUtils
 * @param   a        First string. 
 * @param   b        Second string. 
 * @param   maxCount Maximum number of characters to compare. 
 * @return           kTRUE if the character sequences are equal; otherwise, kFALSE. 
 */
kFx(kBool) kStrnEquals(const kChar* a, const kChar* b, kSize maxCount);

/** 
 * Compares one string to another. 
 * 
 * The result is negative if the string a is lexically less than string b, positive 
 * if string a is lexically greater than string b, and zero if they are equal. 
 *
 * This function performs comparison of UTF-8 encoded characters by Unicode code point.
 *
 * @public           @memberof kUtils
 * @param   a        First string. 
 * @param   b        Second string. 
 * @return           Positive if a is greater than b, negative if b is greater than a; otherwise zero.
 */
kFx(k32s) kStrCompare(const kChar* a, const kChar* b); 

/** 
 * Compares one string to another, only up to a maximum number of characters. 
 * 
 * The result is negative if the string a is lexically less than string b, positive 
 * if string a is lexically greater than string b, and zero if they are equal. 
 *
 * This function performs comparison of UTF-8 encoded characters by Unicode code point.
 *
 * @public           @memberof kUtils
 * @param   a        First string. 
 * @param   b        Second string. 
 * @param   maxCount Maximum number of characters to compare. 
 * @return           Positive if a is greater than b, negative if b is greater than a; otherwise zero.
 */
kFx(k32s) kStrCompareN(const kChar* a, const kChar* b, kSize maxCount);

/** 
 * Performs a case-insenstive comparison of two strings. 
 *
 * This function currently supports comparison of characters only within the ASCII character range.
 *
 * @public           @memberof kUtils
 * @param   a        First string. 
 * @param   b        Second string. 
 * @return           Positive if a is greater than b, negative if b is greater than a; otherwise zero.
 */
kFx(k32s) kStrCompareLower(const kChar* a, const kChar* b); 

/** 
 * Determines the number of kChar units in a characater sequence. 
 * 
 * @public           @memberof kUtils
 * @param   str      Input string. 
 * @return           Number of kChar units in sequence.
 */
kFx(kSize) kStrLength(const kChar* str); 

/** 
 * Finds the first occurrence of a character sequence. 
 * 
 * @public           @memberof kUtils
 * @param   str      Input string to be searched. 
 * @param   subStr   Substring to find. 
 * @return           Pointer to first occurrence, or kNULL. 
 */
kFx(const kChar*) kStrFindFirst(const kChar* str, const kChar* subStr); 

/** 
 * Finds the last occurrence of a character sequence. 
 * 
 * @public           @memberof kUtils
 * @param   str      Input string to be searched. 
 * @param   subStr   Substring to find. 
 * @return           Pointer to last occurrence, or kNULL. 
 */
kFx(const kChar*) kStrFindLast(const kChar* str, const kChar* subStr); 

/** 
 * Formats a string using printf-style arguments.  
 *
 * This function relies on formatting support from underlying system libraries; results can vary.  
 *
 * If the output buffer is insufficient, kERROR_INCOMPLETE will be returned. In this case, the 
 * destination buffer will contain truncated, null-terminated output. 
 *
 * @public           @memberof kUtils
 * @param   dest     Destination for formatted output. 
 * @param   capacity Capacity of output buffer. 
 * @param   format   Print format string. 
 * @return           Operation status. 
 */
kFx(kStatus) kStrPrintf(kChar* dest, kSize capacity, const kChar* format, ...); 

/** 
 * Variable-argument version of kStrPrintf. 
 *
 * @public           @memberof kUtils
 * @param   dest     Destination for formatted output. 
 * @param   capacity Capacity of output buffer. 
 * @param   format   Print format string. 
 * @param   argList  Variable argument list.
 * @return           Operation status. 
 * @see              kStrPrintf
 */
kFx(kStatus) kStrPrintvf(kChar* dest, kSize capacity, const kChar* format, kVarArgList argList); 

/** 
 * Variable-argument version of kLogf. 
 *
 * @public              @memberof kUtils
 * @param    format     Print format string. 
 * @param    argList    Variable argument list.
 * @return              Operation status. 
 * @see                 kLog, kLogf, kApiLib_AddLogHandler
 */
kFx(kStatus) kLogvf(const kChar* format, kVarArgList argList);

/** 
 * Variable-argument version of kLogf. 
 *
 * @public              @memberof kUtils
 * @param    options    Log options.
 * @param    source     Message source to print.
 * @param    format     Print format string. 
 * @param    argList    Variable argument list.
 * @return              Operation status. 
 * @see                 kLog, kLogf, kApiLib_AddLogHandler
 */
#if defined(K_CPP)
kInlineFx(kStatus) kLogvf(kLogOption options, const kChar* source, const kChar* format, kVarArgList argList)
{
    return xkLogvf(options, source, format, argList);
}
#endif

/** 
 * Formats and writes a message to log handlers (if registered). 
 * 
 * This method makes use of underlying platform support to format string output. Accordingly, 
 * minor formatting differences may occur on different platforms. 
 * 
 * The format buffer used to implement this method limits the size of one output message to 256 bytes.
 * 
 * @public              @memberof kUtils
 * @param    format     Print format string. 
 * @return              Operation status. 
 * @see                 kLog, kLogvf, kApiLib_AddLogHandler
 */
kFx(kStatus) kLogf(const kChar* format, ...);

/** 
 * Formats and writes a message to log handlers (if registered). 
 * 
 * This method makes use of underlying platform support to format string output. Accordingly, 
 * minor formatting differences may occur on different platforms. 
 * 
 * The format buffer used to implement this method limits the size of one output message to 256 bytes.
 * 
 * @public              @memberof kUtils
 * @param    options    Log options.
 * @param    source     Message source to print.
 * @param    format     Print format string. 
 * @return              Operation status. 
 * @see                 kLog, kLogvf, kApiLib_AddLogHandler
 */
#if defined(K_CPP)
kInlineFx(kStatus) kLogf(kLogOption options, const kChar* source, const kChar* format, ...)
{
    kVarArgList argList;
    kStatus status;

    kVarArgList_Start(argList, format);
    {
        status = kLogvf(options, source, format, argList); 
    }
    kVarArgList_End(argList);

    return status;
}
#endif

/** 
 * Writes a message to log handlers (if registered). 
 * 
 * @public              @memberof kUtils
 * @param    message    Message to print. 
 * @return              Operation status. 
 * @see                 kLogf, kLogvf, kApiLib_AddLogHandler
 */
kFx(kStatus) kLog(const kChar* message);

/** 
 * Writes a message to log handlers (if registered). 
 * 
 * @public              @memberof kUtils
 * @param    options    Log options.
 * @param    source     Message source to print.
 * @param    message    Message to print. 
 * @return              Operation status. 
 * @see                 kLogf, kLogvf, kApiLib_AddLogHandler
 */
#if defined(K_CPP)
kInlineFx(kStatus) kLog(kLogOption options, const kChar* source, const kChar* message)
{
    return kLogf(options, source, "%s", message);
}
#endif

/** 
 * Writes a stack backtrace to the logging handler (if registered). The output of this 
 * method is system-specific. 
 *
 * @public              @memberof kUtils
 * @param    skip       Count of recent functions to omit from trace. 
 * @return              Operation status. 
 */
kFx(kStatus) kLogBackTrace(kSize skip);

/** 
 * Writes a stack backtrace to the logging handler (if registered). The output of this 
 * method is system-specific. 
 *
 * @public              @memberof kUtils
 * @param    options    Log options.
 * @param    source     Message source to print.
 * @param    skip       Count of recent functions to omit from trace. 
 * @return              Operation status. 
 */
#if defined(K_CPP)
kInlineFx(kStatus) kLogBackTrace(kLogOption options, const kChar* source, kSize skip)
{
    return xkLogBackTrace(options, source, skip);
}
#endif

/** 
 * Generates a random 32-bit number.
 *
 * @public  @memberof kUtils
 * @return  Random 32-bit number.
 */
kFx(k32u) kRandom32u();

/** 
 * Generates a random 64-bit number.
 *
 * @public  @memberof kUtils
 * @return  Random 64-bit number.
 */
kFx(k64u) kRandom64u();

/** 
 * Generates a random number of type kSize. 
 *
 * @public  @memberof kUtils
 * @return  Random number.
 */
kFx(kSize) kRandomSize();

/** 
 * Generates a random array of bytes.
 *
 * @public  @memberof kUtils
 * @param   data     Buffer to receives random bytes. 
 * @param   length   Length  of buffer.
 * @return  Operation status.
 */
kFx(kStatus) kRandomBytes(void* data, kSize length);

/**
 * Encodes an array of bytes as a base-64 string.
 * 
 * Adds padding, if needed.  
 *
 * @public                  @memberof kFsUtils
 * @param   buffer          Buffer containing the data to encode.
 * @param   size            Size of the buffer in bytes.
 * @param   base64String    Receives the base64-encoded string.
 * @return                  Operation status.
 */
kFx(kStatus) kBase64Encode(const void* buffer, kSize size, kString base64String);

#endif

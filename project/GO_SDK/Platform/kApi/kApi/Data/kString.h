/** 
 * @file    kString.h
 * @brief   Declares the kString class. 
 *
 * @internal
 * Copyright (C) 2005-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_STRING_H
#define K_API_STRING_H

#include <kApi/kApiDef.h>
#include <kApi/Data/kString.x.h>

/**
 * @class       kString
 * @extends     kObject
 * @ingroup     kApi-Data
 * @brief       Represents a character string. 
 * 
 * The kString class represents a variable-length, null-terminated sequence of kChar elements. 
 *
 * kString supports the kObject_Clone, kObject_Size, kObject_Equals, and kObject_HashCode methods.
 *
 * kString supports the kdat5 and kdat6 serialization protocols.
 */
//typedef kObject kString;           --forward-declared in kApiDef.x.h 

/** 
 * Constructs a kString object.
 *
 * @public              @memberof kString
 * @param   str         Receives constructed string object. 
 * @param   content     Initial string content (or kNULL).
 * @param   allocator   Memory allocator. 
 * @return              Operation status. 
 */
kFx(kStatus) kString_Construct(kString* str, const kChar* content, kAlloc allocator); 

/** 
 * Copies the source string content. 
 *
 * @public              @memberof kString
 * @param   str         String object. 
 * @param   source      Source string to be copied.
 * @return              Operation status. 
 */
kFx(kStatus) kString_Assign(kString str, kString source); 

/** 
 * Sets the length of the string to zero. 
 *
 * @public              @memberof kString
 * @param   str         String object. 
 * @return              Operation status. 
 */
kFx(kStatus) kString_Clear(kString str); 

/** 
 * Ensures that capacity is reserved for at least the specified number of character units (excluding null terminator). 
 *
 * Any string content within the original capacity is preserved.
 *
 * @public                      @memberof kString
 * @param   str                 String object. 
 * @param   minimumCapacity     Minimum string capacity, in character units. 
 * @return                      Operation status. 
 */
kFx(kStatus) kString_Reserve(kString str, kSize minimumCapacity); 

/**
* Explicitly sets the length of the string.
*
* This function can be used to specify the string length if the string buffer has been directly
* manipulated. 
*
* If the current capacity is less than the specified length, the capacity will be automatically increased. A null
* terminator will be written at the end of the string buffer.
* 
* Use with caution; no error-checking is performed to ensure that the string does not contain extra null
* terminators prior to the specified length.
*
* @public              @memberof kString
* @param   str         String object.
* @param   length      New string length, in character units (excluding null-terminator).
* @return              Operation status.
*/
kFx(kStatus) kString_SetLength(kString str, kSize length);

/** 
 * Sets the content of the string.
 *
 * @public              @memberof kString
 * @param   str         String object. 
 * @param   content     String content to copy. 
 * @return              Operation status. 
 */
kFx(kStatus) kString_Set(kString str, const kChar* content);

/** 
 * Sets the content of the string using a printf-like format string and arguments.
 * 
 * This function relies on formatting support from underlying system libraries; results can vary.  
 *
 * @public              @memberof kString
 * @param   str         String object. 
 * @param   format      Print format string. 
 * @return              Operation status. 
 */
kFx(kStatus) kString_Setf(kString str, const kChar* format, ...); 

/** 
 * Variable-argument version of kString_Setf.
 * 
 * @public              @memberof kString
 * @param   str         String object. 
 * @param   format      Print format string. 
 * @param   argList     Variable argument list.
 * @return              Operation status. 
 */
kFx(kStatus) kString_Setvf(kString str, const kChar* format, kVarArgList argList);

/** 
 * Appends content to the string.
 *
 * @public              @memberof kString
 * @param   str         String object. 
 * @param   content     String content to append. 
 * @return              Operation status. 
 */
kFx(kStatus) kString_Add(kString str, const kChar* content); 

/** 
 * Appends content to the string using a printf-like format string and arguments.
 * 
 * This function relies on formatting support from underlying system libraries; results can vary.  
 *
 * @public              @memberof kString
 * @param   str         String object. 
 * @param   format      Print format string. 
 * @return              Operation status. 
 */
kFx(kStatus) kString_Addf(kString str, const kChar* format, ...); 

/** 
 * Variable-argument version of kString_Addf.
 * 
 * @public              @memberof kString
 * @param   str         String object. 
 * @param   format      Print format string. 
 * @param   argList     Variable argument list.
 * @return              Operation status. 
 */
kFx(kStatus) kString_Addvf(kString str, const kChar* format, kVarArgList argList); 

/** 
 * Appends a portion of another string to this string.
 *
 * @public              @memberof kString
 * @param   str         String object. 
 * @param   content     String content to append. 
 * @param   start       Starting offset with content argument. 
 * @param   count       Count of characters to append. 
 * @return              Operation status. 
 */
kFx(kStatus) kString_AddSubstring(kString str, const kChar* content, kSize start, kSize count); 

/** 
 * Compares this string to another string. 
 * 
 * The result is negative if the string object is lexically less than the input, positive 
 * if the string object is lexically greater than the input, and zero if they are equal. 
 * 
 * This function performs comparison of UTF-8 encoded characters by Unicode code point.
 *
 * @public              @memberof kString
 * @param   str         String object. 
 * @param   content     String content to be compared. 
 * @return              Comparison result. 
 */
kFx(k32s) kString_Compare(kString str, const kChar* content);

/** 
 * Compares this string to another character sequence to determine equality.
 * 
 * @public              @memberof kString
 * @param   str         String object. 
 * @param   content     String content to be compared. 
 * @return              kTRUE if the character sequences are equal; otherwise, kFALSE. 
 */
kFx(kBool) kString_Equals(kString str, const kChar* content);

/** 
 * Removes leading and trailing whitespace. 
 * 
 * @public              @memberof kString
 * @param   str         String object. 
 * @return              Operation status. 
 */
kFx(kStatus) kString_Trim(kString str); 

/** 
 * Splits this string into substrings using the supplied delimiters.  
 * 
 * When using this method, all delimiter instances will result in the creation of 
 * additional output tokens, even if the output tokens are empty (e.g., multiple, 
 * successive delimiters). Use the kString_SplitEx method if control over this 
 * behaviour is required.
 * 
 * This function currently supports delimiters only within the ASCII character range. 
 *
 * @public              @memberof kString
 * @param   str         String object. 
 * @param   delimiters  Null-terminated string containing delimiter characters.  
 * @param   tokens      Receives a list substrings. 
 * @param   allocator   Memory allocator (or kNULL for default). 
 * @return              Operation status.
 */
kInlineFx(kStatus) kString_Split(kString str, const kChar* delimiters, kArrayList* tokens, kAlloc allocator)
{
    return kString_SplitEx(str, delimiters, tokens, kFALSE, allocator);
}

/** 
 * Splits this string into substrings using the supplied delimiters.  
 * 
 * This function currently supports delimiters only within the ASCII character range. 
 *
 * @public                  @memberof kString
 * @param   str             String object. 
 * @param   delimiters      Null-terminated string containing delimiter characters.  
 * @param   tokens          Receives a list substrings. 
 * @param   discardEmpty    kTRUE to omit empty tokens. 
 * @param   allocator       Memory allocator (or kNULL for default). 
 * @return                  Operation status.
 */
kFx(kStatus) kString_SplitEx(kString str, const kChar* delimiters, kArrayList* tokens, kBool discardEmpty, kAlloc allocator);

/** 
 * Returns a pointer to the internal character buffer.
 * 
 * @public              @memberof kString
 * @param   str         String object. 
 * @return              Pointer to null-terminated character buffer. 
 */
kInlineFx(kChar*) kString_Chars(kString str)
{
    kObj(kString, str); 

    return obj->chars;
}

/** 
 * Returns the number of character units in the string buffer (excluding null-terminator). 
 * 
 * kString assumes UTF8-encoded data; the string length refers to the number of encoded 
 * character units (bytes), rather than the number of characters. 
 * 
 * @public              @memberof kString
 * @param   str         String object. 
 * @return              Count of character units (excluding null-terminator). 
 */
kInlineFx(kSize) kString_Length(kString str)
{
    kObj(kString, str); 

    return obj->length;
}

/** 
 * Returns the number of character units that can be stored without reallocation. 
 * 
 * @public              @memberof kString
 * @param   str         String object. 
 * @return              String capacity. 
 */
kInlineFx(kSize) kString_Capacity(kString str)
{
    kObj(kString, str); 

    return obj->capacity;
}

#endif

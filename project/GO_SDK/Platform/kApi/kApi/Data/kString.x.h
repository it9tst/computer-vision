/** 
 * @file    kString.x.h
 *
 * @internal
 * Copyright (C) 2005-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_STRING_X_H
#define K_API_STRING_X_H

#define xkSTRING_MIN_CAPACITY                (16)
#define xkSTRING_GROWTH_FACTOR               (2)

typedef struct kStringClass
{
    kObjectClass base; 
    kSize allocSize;            //size of allocated array memory, in bytes
    kChar* chars;               //null-terminated character array
    kSize length;               //current number of elements (excluding null-terminator)
    kSize capacity;             //maximum elements before reallocation
    kChar nullStr;              //null string (avoids need for dynamic allocation when null content pointer passed)
} kStringClass;

kDeclareClassEx(k, kString, kObject) 

/* 
* Forward declarations. 
*/

kFx(kStatus) kString_Set(kString str, const kChar* content);
kFx(kStatus) kString_Add(kString str, const kChar* content); 
kFx(kStatus) kString_SplitEx(kString str, const kChar* delimiters, kArrayList* tokens, kBool discardEmpty, kAlloc allocator);

/* 
* Private methods. 
*/

kFx(kStatus) xkString_ConstructFramework(kString* string, kAlloc allocator); 

kFx(kStatus) xkString_Init(kString str, kType classType, const kChar* content, kAlloc allocator); 

kFx(kStatus) xkString_VClone(kString str, kString source, kAlloc valueAlloc, kObject context); 

kFx(kStatus) xkString_VRelease(kString str); 

kFx(kSize) xkString_VHashCode(kString str); 
kFx(kBool) xkString_VEquals(kString str, kObject other); 
kFx(kSize) xkString_VSize(kString str); 

kFx(kStatus) xkString_WriteDat5V1(kString str, kSerializer serializer); 
kFx(kStatus) xkString_ReadDat5V1(kString str, kSerializer serializer); 

kFx(kStatus) xkString_WriteDat6V0(kString str, kSerializer serializer); 
kFx(kStatus) xkString_ReadDat6V0(kString str, kSerializer serializer); 

kFx(kStatus) xkString_SplitAdd(const kChar* start, kSize length, kArrayList tokens, kAlloc allocator, kBool discardEmpty);

/* 
* Deprecated (Stage 1): not recommended for further use, but not yet announced via kDeprecate
*/

//[Deprecated] Use either kString_Add (append == kTRUE) or kString_Set (append == kFALSE)
kInlineFx(kStatus) kString_Import(kString str, const kChar* cstr, kBool append)
{
    if (append)     return kString_Add(str, cstr); 
    else            return kString_Set(str, cstr);
}

#endif

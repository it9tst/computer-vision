/** 
 * @file    kApiDef.cpp
 *
 * @internal
 * Copyright (C) 2005-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#define K_PLATFORM
#include <kApi/kApiDef.h>
#include <kApi/Io/kSerializer.h>
#include <kApi/Io/kStream.h>
#include <kApi/Threads/kThread.h>
#include <stdio.h>

kDx(kType) xkTypeVar(kNull);

xkBeginVoidValueEx(k, kVoid)
    kAddFlags(kVoid, kTYPE_FLAGS_PRIMITIVE)

    kAddPrivateVersionEx(kVoid, "kdat6", "5.7.1.0", "kVoid-0", Write, Read)
xkEndVoidValueEx()

kFx(kStatus) xkVoid_Write(kType type, void* values, kSize count, kSerializer serializer)
{
    return kOK; 
}

kFx(kStatus) xkVoid_Read(kType type, void* values, kSize count, kSerializer serializer)
{
    return kOK; 
}

kBeginValueEx(k, k8u)
    kAddFlags(k8u, kTYPE_FLAGS_PRIMITIVE)

    kAddPrivateVersionEx(k8u, "kdat5", "5.0.0.0", "146-0", Write, Read)
    kAddPrivateVersionEx(k8u, "kdat6", "5.7.1.0", "k8u-0", Write, Read)

    kAddPrivateVMethod(k8u, kValue, VEquals)
    kAddPrivateVMethod(k8u, kValue, VHashCode)
kEndValueEx()

kFx(kBool) xk8u_VEquals(kType type, const void* value, const void* other)
{
    return *(k8u*)value == *(k8u*)other; 
}

kFx(kSize) xk8u_VHashCode(kType type, const void* value)
{
    return (kSize) *(k8u*)value; 
}

kFx(kStatus) xk8u_Write(kType type, const void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Write8uArray(serializer, (const k8u*)values, count); 
}

kFx(kStatus) xk8u_Read(kType type, void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Read8uArray(serializer, (k8u*)values, count); 
}

kFx(kStatus) k8u_Format(k8u value, kChar* buffer, kSize capacity)
{
    return xkStrFormat8u(value, buffer, capacity, kNULL);
}

kFx(kStatus) k8u_Parse(k8u* value, const kChar* str)
{
    return xkStrParse8u(value, str);
}

kBeginValueEx(k, k16u)
    kAddFlags(k16u, kTYPE_FLAGS_PRIMITIVE)

    kAddPrivateVersionEx(k16u, "kdat5", "5.0.0.0", "148-0", Write, Read)
    kAddPrivateVersionEx(k16u, "kdat6", "5.7.1.0", "k16u-0", Write, Read)

    kAddPrivateVMethod(k16u, kValue, VEquals)
    kAddPrivateVMethod(k16u, kValue, VHashCode)
kEndValueEx()

kFx(kBool) xk16u_VEquals(kType type, const void* value, const void* other)
{
    return *(k16u*)value == *(k16u*)other; 
}

kFx(kSize) xk16u_VHashCode(kType type, const void* value)
{
    return (kSize) *(k16u*)value; 
}

kFx(kStatus) xk16u_Write(kType type, const void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Write16uArray(serializer, (const k16u*) values, count); 
}

kFx(kStatus) xk16u_Read(kType type, void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Read16uArray(serializer, (k16u*)values, count); 
}

kFx(kStatus) k16u_Format(k16u value, kChar* buffer, kSize capacity)
{
    return xkStrFormat16u(value, buffer, capacity, kNULL);
}

kFx(kStatus) k16u_Parse(k16u* value, const kChar* str)
{
    return xkStrParse16u(value, str);
}

kBeginValueEx(k, k32u)
    kAddFlags(k32u, kTYPE_FLAGS_PRIMITIVE)

    kAddPrivateVersionEx(k32u, "kdat5", "5.0.0.0", "150-0", Write, Read)
    kAddPrivateVersionEx(k32u, "kdat6", "5.7.1.0", "k32u-0", Write, Read)

    kAddPrivateVMethod(k32u, kValue, VEquals)
    kAddPrivateVMethod(k32u, kValue, VHashCode)
kEndValueEx()

kFx(kBool) xk32u_VEquals(kType type, const void* value, const void* other)
{
    return *(k32u*)value == *(k32u*)other; 
}

kFx(kSize) xk32u_VHashCode(kType type, const void* value)
{
    return (kSize) *(k32u*)value; 
}

kFx(kStatus) xk32u_Write(kType type, const void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Write32uArray(serializer, (const k32u*) values, count); 
}

kFx(kStatus) xk32u_Read(kType type, void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Read32uArray(serializer, (k32u*)values, count); 
}

kFx(kStatus) k32u_Format(k32u value, kChar* buffer, kSize capacity)
{
    return xkStrFormat32u(value, buffer, capacity, kNULL);
}

kFx(kStatus) k32u_Parse(k32u* value, const kChar* str)
{
    return xkStrParse32u(value, str);
}

kBeginValueEx(k, k64u)
    kAddFlags(k64u, kTYPE_FLAGS_PRIMITIVE)

    kAddPrivateVersionEx(k64u, "kdat5", "5.0.0.0", "153-0", Write, Read)
    kAddPrivateVersionEx(k64u, "kdat6", "5.7.1.0", "k64u-0", Write, Read)

    kAddPrivateVMethod(k64u, kValue, VEquals)
    kAddPrivateVMethod(k64u, kValue, VHashCode)
kEndValueEx()

kFx(kBool) xk64u_VEquals(kType type, const void* value, const void* other)
{
    return *(k64u*)value == *(k64u*)other; 
}

#if (K_POINTER_SIZE == 4)

kFx(kSize) xk64u_VHashCode(kType type, const void* value)
{
    const k32u* v = (const k32u*) value;
    return v[0] ^ v[1];
}

#elif (K_POINTER_SIZE == 8)

kFx(kSize) xk64u_VHashCode(kType type, const void* value)
{
    return *(kSize*)value;
}

#endif

kFx(kStatus) xk64u_Write(kType type, const void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Write64uArray(serializer, (const k64u*) values, count); 
}

kFx(kStatus) xk64u_Read(kType type, void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Read64uArray(serializer, (k64u*) values, count); 
}

kFx(kStatus) k64u_Format(k64u value, kChar* buffer, kSize capacity)
{
    return xkStrFormat64u(value, buffer, capacity, kNULL);
}

kFx(kStatus) k64u_Parse(k64u* value, const kChar* str)
{
    return xkStrParse64u(value, str);
}

kBeginValueEx(k, k8s)
    kAddFlags(k8s, kTYPE_FLAGS_PRIMITIVE)

    kAddPrivateVersionEx(k8s, "kdat5", "5.0.0.0", "147-0", Write, Read)
    kAddPrivateVersionEx(k8s, "kdat6", "5.7.1.0", "k8s-0", Write, Read)

    kAddPrivateVMethod(k8s, kValue, VEquals)
    kAddPrivateVMethod(k8s, kValue, VHashCode)
kEndValueEx()

kFx(kBool) xk8s_VEquals(kType type, const void* value, const void* other)
{
    return *(k8s*)value == *(k8s*)other; 
}

kFx(kSize) xk8s_VHashCode(kType type, const void* value)
{
    return (kSize) *(k8s*)value; 
}

kFx(kStatus) xk8s_Write(kType type, const void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Write8sArray(serializer, (const k8s*) values, count); 
}

kFx(kStatus) xk8s_Read(kType type, void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Read8sArray(serializer, (k8s*) values, count); 
}

kFx(kStatus) k8s_Format(k8s value, kChar* buffer, kSize capacity)
{
    return xkStrFormat8s(value, buffer, capacity, kNULL);
}

kFx(kStatus) k8s_Parse(k8s* value, const kChar* str)
{
    return xkStrParse8s(value, str);
}

kBeginValueEx(k, k16s)
    kAddFlags(k16s, kTYPE_FLAGS_PRIMITIVE)

    kAddPrivateVersionEx(k16s, "kdat5", "5.0.0.0", "149-0", Write, Read)
    kAddPrivateVersionEx(k16s, "kdat6", "5.7.1.0", "k16s-0", Write, Read)

    kAddPrivateVMethod(k16s, kValue, VEquals)
    kAddPrivateVMethod(k16s, kValue, VHashCode)
kEndValueEx()

kFx(kBool) xk16s_VEquals(kType type, const void* value, const void* other)
{
    return *(k16s*)value == *(k16s*)other; 
}

kFx(kSize) xk16s_VHashCode(kType type, const void* value)
{
    return (kSize) *(k16s*)value; 
}

kFx(kStatus) xk16s_Write(kType type, const void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Write16sArray(serializer, (const k16s*) values, count); 
}

kFx(kStatus) xk16s_Read(kType type, void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Read16sArray(serializer, (k16s*) values, count); 
}

kFx(kStatus) k16s_Format(k16s value, kChar* buffer, kSize capacity)
{
    return xkStrFormat16s(value, buffer, capacity, kNULL);
}

kFx(kStatus) k16s_Parse(k16s* value, const kChar* str)
{
    return xkStrParse16s(value, str);
}

kBeginValueEx(k, k32s)
    kAddFlags(k32s, kTYPE_FLAGS_PRIMITIVE)

    kAddPrivateVersionEx(k32s, "kdat5", "5.0.0.0", "151-0", Write, Read)
    kAddPrivateVersionEx(k32s, "kdat6", "5.7.1.0", "k32s-0", Write, Read)

    kAddPrivateVMethod(k32s, kValue, VEquals)
    kAddPrivateVMethod(k32s, kValue, VHashCode)
kEndValueEx()

kFx(kBool) xk32s_VEquals(kType type, const void* value, const void* other)
{
    return *(k32s*)value == *(k32s*)other; 
}

kFx(kSize) xk32s_VHashCode(kType type, const void* value)
{
    return (kSize) *(k32s*)value; 
}

kFx(kStatus) xk32s_Write(kType type, const void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Write32sArray(serializer, (const k32s*) values, count); 
}

kFx(kStatus) xk32s_Read(kType type, void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Read32sArray(serializer, (k32s*) values, count); 
}

kFx(kStatus) k32s_Format(k32s value, kChar* buffer, kSize capacity)
{
    return xkStrFormat32s(value, buffer, capacity, kNULL);
}

kFx(kStatus) k32s_Parse(k32s* value, const kChar* str)
{
    return xkStrParse32s(value, str);
}

kBeginValueEx(k, k64s)
    kAddFlags(k64s, kTYPE_FLAGS_PRIMITIVE)

    kAddPrivateVersionEx(k64s, "kdat5", "5.0.0.0", "154-0", Write, Read)
    kAddPrivateVersionEx(k64s, "kdat6", "5.7.1.0", "k64s-0", Write, Read)

    kAddPrivateVMethod(k64s, kValue, VEquals)
    kAddPrivateVMethod(k64s, kValue, VHashCode)
kEndValueEx()

kFx(kBool) xk64s_VEquals(kType type, const void* value, const void* other)
{
    return *(k64s*)value == *(k64s*)other; 
}

#if (K_POINTER_SIZE == 4)

kFx(kSize) xk64s_VHashCode(kType type, const void* value)
{
    const k32u* v = (const k32u*) value; 
    return v[0] ^ v[1]; 
}

#elif (K_POINTER_SIZE == 8)

kFx(kSize) xk64s_VHashCode(kType type, const void* value)
{
    return *(kSize*)value;
}

#endif

kFx(kStatus) xk64s_Write(kType type, const void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Write64sArray(serializer, (const k64s*) values, count); 
}

kFx(kStatus) xk64s_Read(kType type, void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Read64sArray(serializer, (k64s*) values, count); 
}

kFx(kStatus) k64s_Format(k64s value, kChar* buffer, kSize capacity)
{
    return xkStrFormat64s(value, buffer, capacity, kNULL);
}

kFx(kStatus) k64s_Parse(k64s* value, const kChar* str)
{
    return xkStrParse64s(value, str);
}

kBeginValueEx(k, k32f)
    kAddFlags(k32f, kTYPE_FLAGS_PRIMITIVE)

    kAddPrivateVersionEx(k32f, "kdat5", "5.0.0.0", "152-0", Write, Read)
    kAddPrivateVersionEx(k32f, "kdat6", "5.7.1.0", "k32f-0", Write, Read)

    kAddPrivateVMethod(k32f, kValue, VEquals)
    kAddPrivateVMethod(k32f, kValue, VHashCode)
kEndValueEx()

kFx(kBool) xk32f_VEquals(kType type, const void* value, const void* other)
{
    return *(k32f*)value == *(k32f*)other; 
}

kFx(kSize) xk32f_VHashCode(kType type, const void* value)
{
    const k32u* v = (k32u*) value; 
    return v[0]; 
}

kFx(kStatus) xk32f_Write(kType type, const void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Write32fArray(serializer, (const k32f*) values, count); 
}

kFx(kStatus) xk32f_Read(kType type, void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Read32fArray(serializer, (k32f*) values, count); 
}

kFx(kStatus) k32f_Format(k32f value, kChar* buffer, kSize capacity)
{
    return xkStrFormat32f(value, buffer, capacity, -1, kNULL);
}

kFx(kStatus) k32f_Parse(k32f* value, const kChar* str)
{
    return xkStrParse32f(value, str);
}

kBeginValueEx(k, k64f)
    kAddFlags(k64f, kTYPE_FLAGS_PRIMITIVE)

    kAddPrivateVersionEx(k64f, "kdat5", "5.0.0.0", "155-0", Write, Read)
    kAddPrivateVersionEx(k64f, "kdat6", "5.7.1.0", "k64f-0", Write, Read)

    kAddPrivateVMethod(k64f, kValue, VEquals)
    kAddPrivateVMethod(k64f, kValue, VHashCode)
kEndValueEx()

kFx(kBool) xk64f_VEquals(kType type, const void* value, const void* other)
{
    return *(k64f*)value == *(k64f*)other; 
}

#if (K_POINTER_SIZE == 4)

kFx(kSize) xk64f_VHashCode(kType type, const void* value)
{
    const k32u* v = (k32u*) value; 
    return (kSize) (v[0] ^ v[1]); 
}

#elif (K_POINTER_SIZE == 8)

kFx(kSize) xk64f_VHashCode(kType type, const void* value)
{
    return *(kSize*)value;
}

#endif

kFx(kStatus) xk64f_Write(kType type, const void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Write64fArray(serializer, (const k64f*) values, count); 
}

kFx(kStatus) xk64f_Read(kType type, void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Read64fArray(serializer, (k64f*)values, count); 
}

kFx(kStatus) k64f_Format(k64f value, kChar* buffer, kSize capacity)
{
    return xkStrFormat64f(value, buffer, capacity, -1, kNULL);
}

kFx(kStatus) k64f_Parse(k64f* value, const kChar* str)
{
    return xkStrParse64f(value, str);
}

kFx(k32s) k64f_IsNanOrInf(k64f value)
{
    k64u temp;
    
    kItemCopy(&temp, &value, sizeof(temp)); 
    
    return (temp & k64U(0x7FF0000000000000)) == k64U(0x7FF0000000000000);
}

kBeginValueEx(k, kByte)
    kAddFlags(kByte, kTYPE_FLAGS_PRIMITIVE)

    kAddPrivateVersionEx(kByte, "kdat5", "5.0.0.0", "157-0", Write, Read)
    kAddPrivateVersionEx(kByte, "kdat6", "5.7.1.0", "kByte-0", Write, Read)

    kAddPrivateVMethod(kByte, kValue, VEquals)
    kAddPrivateVMethod(kByte, kValue, VHashCode)
kEndValueEx()

kFx(kBool) xkByte_VEquals(kType type, const void* value, const void* other)
{
    return *(kByte*)value == *(kByte*)other; 
}

kFx(kSize) xkByte_VHashCode(kType type, const void* value)
{
    return (kSize) *(kByte*)value; 
}

kFx(kStatus) xkByte_Write(kType type, const void* values, kSize count, kSerializer serializer)
{
    return kSerializer_WriteByteArray(serializer, (const kByte*) values, count); 
}

kFx(kStatus) xkByte_Read(kType type, void* values, kSize count, kSerializer serializer)
{
    return kSerializer_ReadByteArray(serializer, (kByte*) values, count); 
}

kBeginValueEx(k, kChar)
    kAddFlags(kChar, kTYPE_FLAGS_PRIMITIVE)

    kAddPrivateVersionEx(kChar, "kdat5", "5.0.0.0", "158-0", Write, Read)
    kAddPrivateVersionEx(kChar, "kdat6", "5.7.1.0", "kChar-0", Write, Read)

    kAddPrivateVMethod(kChar, kValue, VEquals)
    kAddPrivateVMethod(kChar, kValue, VHashCode)
kEndValueEx()

kFx(kBool) xkChar_VEquals(kType type, const void* value, const void* other)
{
    return *(kChar*)value == *(kChar*)other; 
}

kFx(kSize) xkChar_VHashCode(kType type, const void* value)
{
    return (kSize) *(kChar*)value; 
}

kFx(kStatus) xkChar_Write(kType type, const void* values, kSize count, kSerializer serializer)
{
    return kSerializer_WriteCharArray(serializer, (const kChar*) values, count); 
}

kFx(kStatus) xkChar_Read(kType type, void* values, kSize count, kSerializer serializer)
{
    return kSerializer_ReadCharArray(serializer, (kChar*) values, count); 
}

kBeginValueEx(k, kBool)
    kAddFlags(kBool, kTYPE_FLAGS_PRIMITIVE)

    kAddPrivateVersionEx(kBool, "kdat5", "5.0.0.0", "156-0", Write, Read)
    kAddPrivateVersionEx(kBool, "kdat6", "5.7.1.0", "kBool-0", Write, Read)

    kAddPrivateVMethod(kBool, kValue, VEquals)
    kAddPrivateVMethod(kBool, kValue, VHashCode)
kEndValueEx()

kFx(kBool) xkBool_VEquals(kType type, const void* value, const void* other)
{
    return *(kBool*)value == *(kBool*)other; 
}

kFx(kSize) xkBool_VHashCode(kType type, const void* value)
{
    return (kSize) *(kBool*)value; 
}

kFx(kStatus) xkBool_Write(kType type, const void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Write32sArray(serializer, (const kBool*) values, count); 
}

kFx(kStatus) xkBool_Read(kType type, void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Read32sArray(serializer, (kBool*) values, count); 
}

kFx(kStatus) kBool_Format(kBool value, kChar* buffer, kSize capacity)
{
    return xkStrFormatBool(value, buffer, capacity, kNULL);
}

kFx(kStatus) kBool_Parse(kBool* value, const kChar* str)
{
    return xkStrParseBool(value, str);
}

kBeginValueEx(k, kSize)
    kAddFlags(kSize, kTYPE_FLAGS_PRIMITIVE)

    kAddPrivateVersionEx(kSize, "kdat6", "5.7.1.0", "kSize-0", Write, Read)

    kAddPrivateVMethod(kSize, kValue, VEquals)
    kAddPrivateVMethod(kSize, kValue, VHashCode)
kEndValueEx()

kFx(kBool) xkSize_VEquals(kType type, const void* value, const void* other)
{
    return *(kSize*)value == *(kSize*)other; 
}

kFx(kSize) xkSize_VHashCode(kType type, const void* value)
{
    return *(kSize*)value; 
}
kFx(kStatus) xkSize_Write(kType type, const void* values, kSize count, kSerializer serializer)
{
    return kSerializer_WriteSizeArray(serializer, (const kSize*) values, count); 
}

kFx(kStatus) xkSize_Read(kType type, void* values, kSize count, kSerializer serializer)
{
    return kSerializer_ReadSizeArray(serializer, (kSize*) values, count); 
}

kFx(kStatus) kSize_Format(kSize value, kChar* buffer, kSize capacity)
{
    return xkStrFormatSize(value, buffer, capacity, kNULL);
}

kFx(kStatus) kSize_Parse(kSize* value, const kChar* str)
{
    return xkStrParseSize(value, str);
}

kBeginValueEx(k, kSSize)
    kAddFlags(kSSize, kTYPE_FLAGS_PRIMITIVE)

    kAddPrivateVersionEx(kSSize, "kdat6", "5.7.1.0", "kSSize-0", Write, Read)

    kAddPrivateVMethod(kSSize, kValue, VEquals)
    kAddPrivateVMethod(kSSize, kValue, VHashCode)
kEndValueEx()

kFx(kBool) xkSSize_VEquals(kType type, const void* value, const void* other)
{
    return *(kSSize*)value == *(kSSize*)other; }

kFx(kSize) xkSSize_VHashCode(kType type, const void* value)
{
    return (kSize) *(kSSize*)value; 
}

kFx(kStatus) xkSSize_Write(kType type, const void* values, kSize count, kSerializer serializer)
{
    return kSerializer_WriteSSizeArray(serializer, (const kSSize*) values, count); 
}

kFx(kStatus) xkSSize_Read(kType type, void* values, kSize count, kSerializer serializer)
{
    return kSerializer_ReadSSizeArray(serializer, (kSSize*) values, count); 
}

kFx(kStatus) kSSize_Format(kSSize value, kChar* buffer, kSize capacity)
{
    return xkStrFormatSSize(value, buffer, capacity, kNULL);
}

kFx(kStatus) kSSize_Parse(kSSize* value, const kChar* str)
{
    return xkStrParseSSize(value, str);
}

kBeginValueEx(k, kPointer)
    kAddFlags(kPointer, kTYPE_FLAGS_PRIMITIVE)

    kAddPrivateVMethod(kPointer, kValue, VEquals)
    kAddPrivateVMethod(kPointer, kValue, VHashCode)
kEndValueEx()

kFx(kBool) xkPointer_VEquals(kType type, const void* value, const void* other)
{
    return *(kPointer*)value == *(kPointer*)other; 
}

kFx(kSize) xkPointer_VHashCode(kType type, const void* value)
{
    return xkHashPointer(*(kPointer*)value); 
}

kBeginValueEx(k, kFunction)
    kAddFlags(kFunction, kTYPE_FLAGS_PRIMITIVE)

    kAddPrivateVMethod(kFunction, kValue, VEquals)
    kAddPrivateVMethod(kFunction, kValue, VHashCode)
kEndValueEx()

kFx(kBool) xkFunction_VEquals(kType type, const void* value, const void* other)
{
    return *(kFunction*)value == *(kFunction*)other; 
}

kFx(kSize) xkFunction_VHashCode(kType type, const void* value)
{
    return xkHashBytes(value, sizeof(kFunction)); 
}

kBeginArrayValueEx(k, kText16, kChar)
    kAddPrivateVersionEx(kText16, "kdat5", "5.0.0.0", "171-0", Write, Read)
    kAddPrivateVersionEx(kText16, "kdat6", "5.7.1.0", "kText16-0", Write, Read)

    kAddPrivateVMethod(kText16, kValue, VEquals)
    kAddPrivateVMethod(kText16, kValue, VHashCode)
    kAddPrivateVMethod(kText16, kValue, VImport)
kEndArrayValueEx()

kFx(kBool) xkText16_VEquals(kType type, const void* value, const void* other)
{
    return kStrEquals((const kChar*) value, (const kChar*) other); 
}

kFx(kSize) xkText16_VHashCode(kType type, const void* value)
{
    const kChar* it = (const kChar*) value; 
    kSize hash = 0;
    
    while (!kIsNull(*it))
    {
        hash = hash * 31 + *it++; 
    }

    return hash; 
}

kFx(void) xkText16_VImport(kType type, void* value, const void* source)
{
    kStrCopy((kChar*)value, sizeof(kText16), (const kChar*) source); 
}

kFx(kStatus) xkText16_Write(kType type, const void* values, kSize count, kSerializer serializer)
{
    return kSerializer_WriteCharArray(serializer, (const kChar*) values, 16*count); 
}

kFx(kStatus) xkText16_Read(kType type, void* values, kSize count, kSerializer serializer)
{
    return kSerializer_ReadCharArray(serializer, (kChar*) values, 16*count); 
}

kBeginArrayValueEx(k, kText32, kChar)
    kAddPrivateVersionEx(kText32, "kdat5", "5.0.0.0", "172-0", Write, Read)
    kAddPrivateVersionEx(kText32, "kdat6", "5.7.1.0", "kText32-0", Write, Read)

    kAddPrivateVMethod(kText32, kValue, VEquals)
    kAddPrivateVMethod(kText32, kValue, VHashCode)
    kAddPrivateVMethod(kText32, kValue, VImport)
kEndArrayValueEx()

kFx(kBool) xkText32_VEquals(kType type, const void* value, const void* other)
{
    return kStrEquals((const kChar*) value, (const kChar*) other); 
}

kFx(kSize) xkText32_VHashCode(kType type, const void* value)
{
    const kChar* it = (const kChar*) value; 
    kSize hash = 0;
    
    while (!kIsNull(*it))
    {
        hash = hash * 31 + *it++; 
    }

    return hash; 
}

kFx(void) xkText32_VImport(kType type, void* value, const void* source)
{
    kStrCopy((kChar*)value, sizeof(kText32), (const kChar*)source); 
}

kFx(kStatus) xkText32_Write(kType type, const void* values, kSize count, kSerializer serializer)
{
    return kSerializer_WriteCharArray(serializer, (const kChar*) values, 32*count); 
}

kFx(kStatus) xkText32_Read(kType type, void* values, kSize count, kSerializer serializer)
{
    return kSerializer_ReadCharArray(serializer, (kChar*) values, 32*count); 
}

kBeginArrayValueEx(k, kText64, kChar)
    kAddPrivateVersionEx(kText64, "kdat5", "5.0.0.0", "173-0", Write, Read)
    kAddPrivateVersionEx(kText64, "kdat6", "5.7.1.0", "kText64-0", Write, Read)

    kAddPrivateVMethod(kText64, kValue, VEquals)
    kAddPrivateVMethod(kText64, kValue, VHashCode)
    kAddPrivateVMethod(kText64, kValue, VImport)
kEndArrayValueEx()

kFx(kBool) xkText64_VEquals(kType type, const void* value, const void* other)
{
    return kStrEquals((const kChar*) value, (const kChar*) other); 
}

kFx(kSize) xkText64_VHashCode(kType type, const void* value)
{
    const kChar* it = (const kChar*) value; 
    kSize hash = 0;
    
    while (!kIsNull(*it))
    {
        hash = hash * 31 + *it++; 
    }

    return hash; 
}

kFx(void) xkText64_VImport(kType type, void* value, const void* source)
{
    kStrCopy((kChar*) value, sizeof(kText64), (const kChar*) source); 
}

kFx(kStatus) xkText64_Write(kType type, const void* values, kSize count, kSerializer serializer)
{
    return kSerializer_WriteCharArray(serializer, (const kChar*) values, 64*count); 
}

kFx(kStatus) xkText64_Read(kType type, void* values, kSize count, kSerializer serializer)
{
    return kSerializer_ReadCharArray(serializer, (kChar*) values, 64*count); 
}

kBeginArrayValueEx(k, kText128, kChar)
    kAddPrivateVersionEx(kText128, "kdat5", "5.0.0.0", "174-0", Write, Read)
    kAddPrivateVersionEx(kText128, "kdat6", "5.7.1.0", "kText128-0", Write, Read)

    kAddPrivateVMethod(kText128, kValue, VEquals)
    kAddPrivateVMethod(kText128, kValue, VHashCode)
    kAddPrivateVMethod(kText128, kValue, VImport)
kEndArrayValueEx()

kFx(kBool) xkText128_VEquals(kType type, const void* value, const void* other)
{
    return kStrEquals((const kChar*) value, (const kChar*) other); 
}

kFx(kSize) xkText128_VHashCode(kType type, const void* value)
{
    const kChar* it = (const kChar*) value; 
    kSize hash = 0;
    
    while (!kIsNull(*it))
    {
        hash = hash * 31 + *it++; 
    }

    return hash; 
}

kFx(void) xkText128_VImport(kType type, void* value, const void* source)
{
    kStrCopy((kChar*) value, sizeof(kText128), (const kChar*) source); 
}

kFx(kStatus) xkText128_Write(kType type, const void* values, kSize count, kSerializer serializer)
{
    return kSerializer_WriteCharArray(serializer, (const kChar*) values, 128*count); 
}

kFx(kStatus) xkText128_Read(kType type, void* values, kSize count, kSerializer serializer)
{
    return kSerializer_ReadCharArray(serializer, (kChar*) values, 128*count); 
}

kBeginArrayValueEx(k, kText256, kChar)
    kAddPrivateVersionEx(kText256, "kdat5", "5.0.0.0", "175-0", Write, Read)
    kAddPrivateVersionEx(kText256, "kdat6", "5.7.1.0", "kText256-0", Write, Read)

    kAddPrivateVMethod(kText256, kValue, VEquals)
    kAddPrivateVMethod(kText256, kValue, VHashCode)
    kAddPrivateVMethod(kText256, kValue, VImport)
kEndArrayValueEx()

kFx(kBool) xkText256_VEquals(kType type, const void* value, const void* other)
{
    return kStrEquals((const kChar*) value, (const kChar*) other); 
}

kFx(kSize) xkText256_VHashCode(kType type, const void* value)
{
    const kChar* it = (const kChar*) value; 
    kSize hash = 0;
    
    while (!kIsNull(*it))
    {
        hash = hash * 31 + *it++; 
    }

    return hash; 
}

kFx(void) xkText256_VImport(kType type, void* value, const void* source)
{
    kStrCopy((kChar*) value, sizeof(kText256), (const kChar*) source); 
}

kFx(kStatus) xkText256_Write(kType type, const void* values, kSize count, kSerializer serializer)
{
    return kSerializer_WriteCharArray(serializer, (const kChar*) values, 256*count); 
}

kFx(kStatus) xkText256_Read(kType type, void* values, kSize count, kSerializer serializer)
{
    return kSerializer_ReadCharArray(serializer, (kChar*) values, 256*count); 
}

kBeginEnumEx(k, kStatus)

    kAddPrivateVersionEx(kStatus, "kdat6", "5.7.1.0", "kStatus-0", Write, Read)

    kAddEnumerator(kStatus, kERROR_STATE)
    kAddEnumerator(kStatus, kERROR_NOT_FOUND)
    kAddEnumerator(kStatus, kERROR_COMMAND)
    kAddEnumerator(kStatus, kERROR_PARAMETER)
    kAddEnumerator(kStatus, kERROR_UNIMPLEMENTED)
    kAddEnumerator(kStatus, kERROR_MEMORY)
    kAddEnumerator(kStatus, kERROR_TIMEOUT)
    kAddEnumerator(kStatus, kERROR_INCOMPLETE)
    kAddEnumerator(kStatus, kERROR_STREAM)
    kAddEnumerator(kStatus, kERROR_CLOSED)
    kAddEnumerator(kStatus, kERROR_ABORT)
    kAddEnumerator(kStatus, kERROR_ALREADY_EXISTS)
    kAddEnumerator(kStatus, kERROR_NETWORK)
    kAddEnumerator(kStatus, kERROR_HEAP)
    kAddEnumerator(kStatus, kERROR_FORMAT)
    kAddEnumerator(kStatus, kERROR_READ_ONLY)
    kAddEnumerator(kStatus, kERROR_WRITE_ONLY)
    kAddEnumerator(kStatus, kERROR_BUSY)
    kAddEnumerator(kStatus, kERROR_CONFLICT)
    kAddEnumerator(kStatus, kERROR_OS)
    kAddEnumerator(kStatus, kERROR_DEVICE)
    kAddEnumerator(kStatus, kERROR_FULL)
    kAddEnumerator(kStatus, kERROR_IN_PROGRESS)
    kAddEnumerator(kStatus, kERROR)
    kAddEnumerator(kStatus, kOK)

    kAddPrivateVMethod(kStatus, kValue, VEquals)
    kAddPrivateVMethod(kStatus, kValue, VHashCode)

kEndEnumEx()

kFx(const kChar*) kStatus_Name(kStatus status)
{
    switch(status)
    {
    case kERROR_STATE:              return "kERROR_STATE";      
    case kERROR_NOT_FOUND:          return "kERROR_NOT_FOUND"; 
    case kERROR_COMMAND:            return "kERROR_COMMAND"; 
    case kERROR_PARAMETER:          return "kERROR_PARAMETER"; 
    case kERROR_UNIMPLEMENTED:      return "kERROR_UNIMPLEMENTED"; 
    case kERROR_MEMORY:             return "kERROR_MEMORY"; 
    case kERROR_TIMEOUT:            return "kERROR_TIMEOUT"; 
    case kERROR_INCOMPLETE:         return "kERROR_INCOMPLETE"; 
    case kERROR_STREAM:             return "kERROR_STREAM"; 
    case kERROR_CLOSED:             return "kERROR_CLOSED"; 
    case kERROR_VERSION:            return "kERROR_VERSION"; 
    case kERROR_ABORT:              return "kERROR_ABORT"; 
    case kERROR_ALREADY_EXISTS:     return "kERROR_ALREADY_EXISTS"; 
    case kERROR_NETWORK:            return "kERROR_NETWORK"; 
    case kERROR_HEAP:               return "kERROR_HEAP"; 
    case kERROR_FORMAT:             return "kERROR_FORMAT"; 
    case kERROR_READ_ONLY:          return "kERROR_READ_ONLY"; 
    case kERROR_WRITE_ONLY:         return "kERROR_WRITE_ONLY"; 
    case kERROR_BUSY:               return "kERROR_BUSY"; 
    case kERROR_CONFLICT:           return "kERROR_CONFLICT"; 
    case kERROR_OS:                 return "kERROR_OS"; 
    case kERROR_DEVICE:             return "kERROR_DEVICE"; 
    case kERROR_FULL:               return "kERROR_FULL"; 
    case kERROR_IN_PROGRESS:        return "kERROR_IN_PROGRESS"; 
    case kERROR:                    return "kERROR"; 
    case kOK:                       return "kOK"; 
    default:                        return "Unknown";        
    }
}

kFx(kBool) xkStatus_VEquals(kType type, const void* value, const void* other)
{
    return *(kStatus*)value == *(kStatus*)other; 
}

kFx(kSize) xkStatus_VHashCode(kType type, const void* value)
{
    return (kSize) *(kStatus*)value; 
}

kFx(kStatus) xkStatus_Write(kType type, const void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Write32sArray(serializer, (const k32s*) values, count); 
}

kFx(kStatus) xkStatus_Read(kType type, void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Read32sArray(serializer, (k32s*) values, count); 
}

kBeginValueEx(k, kVersion)
    kAddFlags(kVersion, kTYPE_FLAGS_PRIMITIVE)

    kAddPrivateVersionEx(kVersion, "kdat6", "5.7.1.0", "kVersion-0", Write, Read)

    kAddPrivateVMethod(kVersion, kValue, VEquals)
    kAddPrivateVMethod(kVersion, kValue, VHashCode)
kEndValueEx()

kFx(kStatus) kVersion_Parse(kVersion* version, const kChar* buffer)
{
    k32u major, minor, release, build; 

    kCheck(sscanf(buffer, "%u.%u.%u.%u", &major, &minor, &release, &build) == 4); 

    *version = kVersion_Create(major, minor, release, build); 

    return kOK; 
}

kFx(kStatus) kVersion_Format(kVersion version, kChar* buffer, kSize capacity)
{
    return kStrPrintf(buffer, capacity, "%u.%u.%u.%u", (k32u)kVersion_Major(version), 
        (k32u)kVersion_Minor(version), (k32u)kVersion_Release(version), (k32u)kVersion_Build(version)); 
}

kFx(kBool) xkVersion_VEquals(kType type, const void* value, const void* other)
{
    return *(kVersion*)value == *(kVersion*)other; 
}

kFx(kSize) xkVersion_VHashCode(kType type, const void* value)
{
    return xkHashBytes(value, sizeof(kVersion)); 
}

kFx(kStatus) xkVersion_Write(kType type, const void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Write32uArray(serializer, (const k32u*) values, count); 
}

kFx(kStatus) xkVersion_Read(kType type, void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Read32uArray(serializer, (k32u*) values, count); 
}

kBeginEnumEx(k, kEndianness)
    kAddEnumerator(kEndianness, kENDIANNESS_LITTLE)
    kAddEnumerator(kEndianness, kENDIANNESS_BIG)
kEndEnumEx()

kBeginValueEx(k, kPoint16s)
    kAddPrivateVersionEx(kPoint16s, "kdat5", "5.0.0.0", "161-0", Write, Read)
    kAddPrivateVersionEx(kPoint16s, "kdat6", "5.7.1.0", "kPoint16s-0", Write, Read)

    kAddField(kPoint16s, k16s, x)
    kAddField(kPoint16s, k16s, y)
kEndValueEx()

kFx(kBool) xkPoint16s_VEquals(kType type, const void* value, const void* other)
{
    const kPoint16s* a = (const kPoint16s*) value; 
    const kPoint16s* b = (const kPoint16s*) other; 

    return (a->x == b->x) && (a->y == b->y); 
}

kFx(kSize) xkPoint16s_VHashCode(kType type, const void* value)
{
    const kPoint16s* v = (const kPoint16s*) value; 
    
    return (kSize) (v->x ^ v->y); 
}

kFx(kStatus) xkPoint16s_Write(kType type, const void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Write16sArray(serializer, (const k16s*) values, 2*count); 
}

kFx(kStatus) xkPoint16s_Read(kType type, void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Read16sArray(serializer, (k16s*)values, 2*count); 
}

kBeginValueEx(k, kPoint32s)
    kAddPrivateVersionEx(kPoint32s, "kdat5", "5.0.0.0", "162-0", Write, Read)
    kAddPrivateVersionEx(kPoint32s, "kdat6", "5.7.1.0", "kPoint32s-0", Write, Read)

    kAddField(kPoint32s, k32s, x)
    kAddField(kPoint32s, k32s, y)
kEndValueEx()

kFx(kBool) xkPoint32s_VEquals(kType type, const void* value, const void* other)
{
    const kPoint32s* a = (const kPoint32s*) value; 
    const kPoint32s* b = (const kPoint32s*) other; 

    return (a->x == b->x) && (a->y == b->y); 
}

kFx(kSize) xkPoint32s_VHashCode(kType type, const void* value)
{
    const kPoint32s* v = (const kPoint32s*) value; 
    return (kSize) (v->x ^ v->y); 
}

kFx(kStatus) xkPoint32s_Write(kType type, const void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Write32sArray(serializer, (const k32s*) values, 2*count); 
}

kFx(kStatus) xkPoint32s_Read(kType type, void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Read32sArray(serializer, (k32s*) values, 2*count); 
}

kBeginValueEx(k, kPoint32f)
    kAddPrivateVersionEx(kPoint32f, "kdat5", "5.0.0.0", "163-0", Write, Read)
    kAddPrivateVersionEx(kPoint32f, "kdat6", "5.7.1.0", "kPoint32f-0", Write, Read)

    kAddField(kPoint32f, k32f, x)
    kAddField(kPoint32f, k32f, y)
kEndValueEx()

kFx(kBool) xkPoint32f_VEquals(kType type, const void* value, const void* other)
{
    const kPoint32f* a = (const kPoint32f*) value; 
    const kPoint32f* b = (const kPoint32f*) other; 

    return (a->x == b->x) && (a->y == b->y); 
}

kFx(kSize) xkPoint32f_VHashCode(kType type, const void* value)
{
    const kPoint32f* v = (const kPoint32f*) value;   
    const k32u* xu = (const k32u*) (void*) &v->x; 
    const k32u* yu = (const k32u*) (void*) &v->y; 
    
    return (kSize) (xu[0] ^ yu[0]); 
}

kFx(kStatus) xkPoint32f_Write(kType type, const void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Write32fArray(serializer, (const k32f*) values, 2*count); 
}

kFx(kStatus) xkPoint32f_Read(kType type, void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Read32fArray(serializer, (k32f*) values, 2*count); 
}

kBeginValueEx(k, kPoint64f)
    kAddPrivateVersionEx(kPoint64f, "kdat5", "5.0.0.0", "165-0", Write, Read)
    kAddPrivateVersionEx(kPoint64f, "kdat6", "5.7.1.0", "kPoint64f-0", Write, Read)

    kAddField(kPoint64f, k64f, x)
    kAddField(kPoint64f, k64f, y)
kEndValueEx()

kFx(kBool) xkPoint64f_VEquals(kType type, const void* value, const void* other)
{
    const kPoint64f* a = (const kPoint64f*) value; 
    const kPoint64f* b = (const kPoint64f*) other; 

    return (a->x == b->x) && (a->y == b->y); 
}

kFx(kSize) xkPoint64f_VHashCode(kType type, const void* value)
{
    const kPoint64f* v = (const kPoint64f*) value;    
    const k32u* xu = (const k32u*) (void*) &v->x; 
    const k32u* yu = (const k32u*) (void*) &v->y; 
    
    return (kSize) (xu[0] ^ xu[1] ^ yu[0] ^ yu[1]); 
}

kFx(kStatus) xkPoint64f_Write(kType type, const void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Write64fArray(serializer, (const k64f*) values, 2*count); 
}

kFx(kStatus) xkPoint64f_Read(kType type, void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Read64fArray(serializer, (k64f*) values, 2*count); 
}

kBeginValueEx(k, kPoint3d16s)
    kAddPrivateVersionEx(kPoint3d16s, "kdat5", "5.0.0.0", "194-0", Write, Read)
    kAddPrivateVersionEx(kPoint3d16s, "kdat6", "5.7.1.0", "kPoint3d16s-0", Write, Read)

    kAddField(kPoint3d16s, k16s, x)
    kAddField(kPoint3d16s, k16s, y) 
    kAddField(kPoint3d16s, k16s, z) 
kEndValueEx()

kFx(kBool) xkPoint3d16s_VEquals(kType type, const void* value, const void* other)
{
    const kPoint3d16s* a = (const kPoint3d16s*) value; 
    const kPoint3d16s* b = (const kPoint3d16s*) other; 

    return (a->x == b->x) && (a->y == b->y) && (a->z == b->z); 
}

kFx(kSize) xkPoint3d16s_VHashCode(kType type, const void* value)
{
    const kPoint3d16s* v = (const kPoint3d16s*) value; 
    
    return (kSize) (v->x ^ v->y ^ v->z); 
}

kFx(kStatus) xkPoint3d16s_Write(kType type, const void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Write16sArray(serializer, (const k16s*) values, 3*count); 
}

kFx(kStatus) xkPoint3d16s_Read(kType type, void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Read16sArray(serializer, (k16s*) values, 3*count); 
}

kBeginValueEx(k, kPoint3d32s)
    kAddPrivateVersionEx(kPoint3d32s, "kdat5", "5.0.0.0", "164-0", Write, Read)
    kAddPrivateVersionEx(kPoint3d32s, "kdat6", "5.7.1.0", "kPoint3d32s-0", Write, Read)

    kAddField(kPoint3d32s, k32s, x)
    kAddField(kPoint3d32s, k32s, y)
    kAddField(kPoint3d32s, k32s, z) 
kEndValueEx()

kFx(kBool) xkPoint3d32s_VEquals(kType type, const void* value, const void* other)
{
    const kPoint3d32s* a = (const kPoint3d32s*) value; 
    const kPoint3d32s* b = (const kPoint3d32s*) other; 

    return (a->x == b->x) && (a->y == b->y) && (a->z == b->z); 
}

kFx(kSize) xkPoint3d32s_VHashCode(kType type, const void* value)
{
    const kPoint3d32s* v = (const kPoint3d32s*) value; 
    return (kSize) (v->x ^ v->y ^ v->z); 
}

kFx(kStatus) xkPoint3d32s_Write(kType type, const void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Write32sArray(serializer, (const k32s*) values, 3*count); 
}

kFx(kStatus) xkPoint3d32s_Read(kType type, void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Read32sArray(serializer, (k32s*) values, 3*count); 
}

kBeginValueEx(k, kPoint3d32f)
    kAddPrivateVersionEx(kPoint3d32f, "kdat5", "5.0.0.0", "195-0", Write, Read)
    kAddPrivateVersionEx(kPoint3d32f, "kdat6", "5.7.1.0", "kPoint3d32f-0", Write, Read)

    kAddField(kPoint3d32f, k32f, x)
    kAddField(kPoint3d32f, k32f, y)
    kAddField(kPoint3d32f, k32f, z) 
kEndValueEx()

kFx(kBool) xkPoint3d32f_VEquals(kType type, const void* value, const void* other)
{
    const kPoint3d32f* a = (const kPoint3d32f*) value; 
    const kPoint3d32f* b = (const kPoint3d32f*) other; 

    return (a->x == b->x) && (a->y == b->y) && (a->z == b->z); 
}

kFx(kSize) xkPoint3d32f_VHashCode(kType type, const void* value)
{
    const kPoint3d32f* v = (const kPoint3d32f*) value;    
    const k32u* xu = (const k32u*) (void*) &v->x; 
    const k32u* yu = (const k32u*) (void*) &v->y; 
    const k32u* zu = (const k32u*) (void*) &v->z; 
       
    return (kSize) (xu[0] ^ yu[0] ^ zu[1]); 
}

kFx(kStatus) xkPoint3d32f_Write(kType type, const void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Write32fArray(serializer, (const k32f*) values, 3*count); 
}

kFx(kStatus) xkPoint3d32f_Read(kType type, void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Read32fArray(serializer, (k32f*) values, 3*count); 
}

kBeginValueEx(k, kPoint3d64f)
    kAddPrivateVersionEx(kPoint3d64f, "kdat5", "5.0.0.0", "196-0", Write, Read)
    kAddPrivateVersionEx(kPoint3d64f, "kdat6", "5.7.1.0", "kPoint3d64f-0", Write, Read)

    kAddField(kPoint3d64f, k64f, x)
    kAddField(kPoint3d64f, k64f, y)
    kAddField(kPoint3d64f, k64f, z) 
kEndValueEx()

kFx(kBool) xkPoint3d64f_VEquals(kType type, const void* value, const void* other)
{
    const kPoint3d64f* a = (const kPoint3d64f*) value; 
    const kPoint3d64f* b = (const kPoint3d64f*) other; 

    return (a->x == b->x) && (a->y == b->y) && (a->z == b->z); 
}

kFx(kSize) xkPoint3d64f_VHashCode(kType type, const void* value)
{
    const kPoint3d64f* v = (const kPoint3d64f*) value;    
    const k32u* xu = (const k32u*) (void*) &v->x; 
    const k32u* yu = (const k32u*) (void*) &v->y; 
    const k32u* zu = (const k32u*) (void*) &v->z; 
    
    return (kSize) (xu[0] ^ xu[1] ^ yu[0] ^ yu[1] ^ zu[0] ^ zu[1]); 
}

kFx(kStatus) xkPoint3d64f_Write(kType type, const void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Write64fArray(serializer, (const k64f*) values, 3*count); 
}

kFx(kStatus) xkPoint3d64f_Read(kType type, void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Read64fArray(serializer, (k64f*) values, 3*count); 
}

kBeginValueEx(k, kPoint4d16s)
    kAddPrivateVersionEx(kPoint4d16s, "kdat6", "5.7.1.0", "kPoint4d16s-0", Write, Read)

    kAddField(kPoint4d16s, k16s, x)
    kAddField(kPoint4d16s, k16s, y)
    kAddField(kPoint4d16s, k16s, z)
    kAddField(kPoint4d16s, k16s, w)
kEndValueEx()

kFx(kBool) xkPoint4d16s_VEquals(kType type, const void* value, const void* other)
{
    const kPoint4d16s* a = (const kPoint4d16s*) value; 
    const kPoint4d16s* b = (const kPoint4d16s*) other; 

    return (a->x == b->x) && (a->y == b->y) && (a->z == b->z) && (a->w == b->w); 
}

kFx(kSize) xkPoint4d16s_VHashCode(kType type, const void* value)
{
    const kPoint4d16s* v = (const kPoint4d16s*) value; 
    
    return (kSize) (v->x ^ v->y ^ v->z ^ v->w); 
}

kFx(kStatus) xkPoint4d16s_Write(kType type, const void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Write16sArray(serializer, (const k16s*) values, 4*count); 
}

kFx(kStatus) xkPoint4d16s_Read(kType type, void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Read16sArray(serializer, (k16s*) values, 4*count); 
}

kBeginValueEx(k, kRect16s)
    kAddPrivateVersionEx(kRect16s, "kdat6", "5.7.1.0", "kRect16s-0", Write, Read)

    kAddField(kRect16s, k16s, x)
    kAddField(kRect16s, k16s, y)
    kAddField(kRect16s, k16s, width)
    kAddField(kRect16s, k16s, height)
kEndValueEx()

kFx(kBool) xkRect16s_VEquals(kType type, const void* value, const void* other)
{
    const kRect16s* a = (const kRect16s*) value; 
    const kRect16s* b = (const kRect16s*) other; 

    return (a->x == b->x) && (a->y == b->y) && (a->width == b->width) && (a->height == b->height); 
}

kFx(kSize) xkRect16s_VHashCode(kType type, const void* value)
{
    const kRect16s* v = (const kRect16s*) value; 

    return (kSize) (v->x ^ v->y ^ v->width ^ v->height); 
}

kFx(kStatus) xkRect16s_Write(kType type, const void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Write16sArray(serializer, (const k16s*) values, 4*count); 
}

kFx(kStatus) xkRect16s_Read(kType type, void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Read16sArray(serializer, (k16s*) values, 4*count); 
}

kBeginValueEx(k, kRect32s)
    kAddPrivateVersionEx(kRect32s, "kdat5", "5.0.0.0", "166-0", Write, Read)
    kAddPrivateVersionEx(kRect32s, "kdat6", "5.7.1.0", "kRect32s-0", Write, Read)

    kAddField(kRect32s, k32s, x)
    kAddField(kRect32s, k32s, y)
    kAddField(kRect32s, k32s, width)
    kAddField(kRect32s, k32s, height)
kEndValueEx()

kFx(kBool) xkRect32s_VEquals(kType type, const void* value, const void* other)
{
    const kRect32s* a = (const kRect32s*) value; 
    const kRect32s* b = (const kRect32s*) other; 

    return (a->x == b->x) && (a->y == b->y) && (a->width == b->width) && (a->height == b->height); 
}
 
kFx(kSize) xkRect32s_VHashCode(kType type, const void* value)
{
    const kRect32s* v = (const kRect32s*) value; 

    return (kSize) (v->x ^ v->y ^ v->width ^ v->height); 
}

kFx(kStatus) xkRect32s_Write(kType type, const void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Write32sArray(serializer, (const k32s*) values, 4*count); 
}

kFx(kStatus) xkRect32s_Read(kType type, void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Read32sArray(serializer, (k32s*) values, 4*count); 
}

kBeginValueEx(k, kRect32f)
    kAddPrivateVersionEx(kRect32f, "kdat5", "5.0.0.0", "167-0", Write, Read)
    kAddPrivateVersionEx(kRect32f, "kdat6", "5.7.1.0", "kRect32f-0", Write, Read)

    kAddField(kRect32f, k32f, x)
    kAddField(kRect32f, k32f, y)
    kAddField(kRect32f, k32f, width)
    kAddField(kRect32f, k32f, height)
kEndValueEx()

kFx(kBool) xkRect32f_VEquals(kType type, const void* value, const void* other)
{
    const kRect32f* a = (const kRect32f*) value; 
    const kRect32f* b = (const kRect32f*) other; 

    return (a->x == b->x) && (a->y == b->y) && (a->width == b->width) && (a->height == b->height); 
}
 
kFx(kSize) xkRect32f_VHashCode(kType type, const void* value)
{
    return xkHashBytes(value, kType_Size(type)); 
}

kFx(kStatus) xkRect32f_Write(kType type, const void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Write32fArray(serializer, (const k32f*) values, 4*count); 
}

kFx(kStatus) xkRect32f_Read(kType type, void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Read32fArray(serializer, (k32f*) values, 4*count); 
}

kBeginValueEx(k, kRect64f)
    kAddPrivateVersionEx(kRect64f, "kdat5", "5.0.0.0", "168-0", Write, Read)
    kAddPrivateVersionEx(kRect64f, "kdat6", "5.7.1.0", "kRect64f-0", Write, Read)

    kAddField(kRect64f, k64f, x)
    kAddField(kRect64f, k64f, y)
    kAddField(kRect64f, k64f, width)
    kAddField(kRect64f, k64f, height)
kEndValueEx()

kFx(kBool) xkRect64f_VEquals(kType type, const void* value, const void* other)
{
    const kRect64f* a = (const kRect64f*) value; 
    const kRect64f* b = (const kRect64f*) other; 

    return (a->x == b->x) && (a->y == b->y) && (a->width == b->width) && (a->height == b->height); 
}
 
kFx(kSize) xkRect64f_VHashCode(kType type, const void* value)
{
    return xkHashBytes(value, kType_Size(type)); 
}

kFx(kStatus) xkRect64f_Write(kType type, const void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Write64fArray(serializer, (const k64f*) values, 4*count); 
}

kFx(kStatus) xkRect64f_Read(kType type, void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Read64fArray(serializer, (k64f*) values, 4*count); 
}

kBeginValueEx(k, kRect3d64f)
    kAddPrivateVersionEx(kRect3d64f, "kdat6", "6.0.14.0", "kRect3d64f-0", Write, Read)

    kAddField(kRect3d64f, k64f, x)
    kAddField(kRect3d64f, k64f, y)
    kAddField(kRect3d64f, k64f, z)
    kAddField(kRect3d64f, k64f, width)
    kAddField(kRect3d64f, k64f, height)
    kAddField(kRect3d64f, k64f, depth)
kEndValueEx()

kFx(kBool) xkRect3d64f_VEquals(kType type, const void* value, const void* other)
{
    const kRect3d64f* a = (const kRect3d64f*) value; 
    const kRect3d64f* b = (const kRect3d64f*) other; 

    return (a->x == b->x) && (a->y == b->y) && (a->z == b->z) &&
           (a->width == b->width) && (a->height == b->height) && (a->depth == b->depth); 
}
 
kFx(kSize) xkRect3d64f_VHashCode(kType type, const void* value)
{
    return xkHashBytes(value, kType_Size(type)); 
}

kFx(kStatus) xkRect3d64f_Write(kType type, const void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Write64fArray(serializer, (const k64f*) values, 6*count); 
}

kFx(kStatus) xkRect3d64f_Read(kType type, void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Read64fArray(serializer, (k64f*) values, 6*count); 
}

kBeginValueEx(k, kRotatedRect32s)
    kAddPrivateVersionEx(kRotatedRect32s, "kdat5", "5.0.0.0", "169-0", Write, Read)
    kAddPrivateVersionEx(kRotatedRect32s, "kdat6", "5.7.1.0", "kRotatedRect32s-0", Write, Read)

    kAddField(kRotatedRect32s, k32s, xc)
    kAddField(kRotatedRect32s, k32s, yc)
    kAddField(kRotatedRect32s, k32s, width)
    kAddField(kRotatedRect32s, k32s, height)
    kAddField(kRotatedRect32s, k32s, angle)
kEndValueEx()

kFx(kBool) xkRotatedRect32s_VEquals(kType type, const void* value, const void* other)
{
    const kRotatedRect32s* a = (const kRotatedRect32s*) value; 
    const kRotatedRect32s* b = (const kRotatedRect32s*) other; 

    return (a->xc == b->xc) && (a->yc == b->yc) && (a->width == b->width) && (a->height == b->height) && (a->angle == b->angle); 
}
 
kFx(kSize) xkRotatedRect32s_VHashCode(kType type, const void* value)
{
    return xkHashBytes(value, kType_Size(type)); 
}

kFx(kStatus) xkRotatedRect32s_Write(kType type, const void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Write32sArray(serializer, (const k32s*) values, 5*count); 
}

kFx(kStatus) xkRotatedRect32s_Read(kType type, void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Read32sArray(serializer, (k32s*) values, 5*count); 
}

kBeginValueEx(k, kRotatedRect32f)
    kAddPrivateVersionEx(kRotatedRect32f, "kdat5", "5.0.0.0", "170-0", Write, Read)
    kAddPrivateVersionEx(kRotatedRect32f, "kdat6", "5.7.1.0", "kRotatedRect32f-0", Write, Read)

    kAddField(kRotatedRect32f, k32f, xc)
    kAddField(kRotatedRect32f, k32f, yc)
    kAddField(kRotatedRect32f, k32f, width)
    kAddField(kRotatedRect32f, k32f, height)
    kAddField(kRotatedRect32f, k32f, angle)
kEndValueEx()

kFx(kBool) xkRotatedRect32f_VEquals(kType type, const void* value, const void* other)
{
    const kRotatedRect32f* a = (const kRotatedRect32f*) value; 
    const kRotatedRect32f* b = (const kRotatedRect32f*) other; 

    return (a->xc == b->xc) && (a->yc == b->yc) && (a->width == b->width) && (a->height == b->height) && (a->angle == b->angle); 
}
 
kFx(kSize) xkRotatedRect32f_VHashCode(kType type, const void* value)
{
    return xkHashBytes(value, kType_Size(type)); 
}

kFx(kStatus) xkRotatedRect32f_Write(kType type, const void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Write32fArray(serializer, (const k32f*) values, 5*count); 
}

kFx(kStatus) xkRotatedRect32f_Read(kType type, void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Read32fArray(serializer, (k32f*) values, 5*count); 
}

kBeginEnumEx(k, kPixelFormat)
    kAddPrivateVersionEx(kPixelFormat, "kdat6", "5.7.1.0", "kPixelFormat-0", Write, Read)

    kAddEnumerator(kPixelFormat, kPIXEL_FORMAT_NULL)
    kAddEnumerator(kPixelFormat, kPIXEL_FORMAT_8BPP_GREYSCALE)
    kAddEnumerator(kPixelFormat, kPIXEL_FORMAT_8BPP_CFA)
    kAddEnumerator(kPixelFormat, kPIXEL_FORMAT_8BPC_BGRX)
    kAddEnumerator(kPixelFormat, kPIXEL_FORMAT_1BPP_GREYSCALE)
    kAddEnumerator(kPixelFormat, kPIXEL_FORMAT_16BPP_GREYSCALE)

kEndEnumEx()

kFx(kBool) xkPixelFormat_VEquals(kType type, const void* value, const void* other)
{
    return *(kPixelFormat*)value == *(kPixelFormat*)other; 
}

kFx(kSize) xkPixelFormat_VHashCode(kType type, const void* value)
{
    return (kSize) *(kPixelFormat*)value; 
}

kFx(kStatus) xkPixelFormat_Write(kType type, const void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Write32sArray(serializer, (const k32s*) values, count); 
}

kFx(kStatus) xkPixelFormat_Read(kType type, void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Read32sArray(serializer, (k32s*) values, count); 
}

kBeginEnumEx(k, kCfa)
    kAddPrivateVersionEx(kCfa, "kdat6", "5.7.1.0", "kCfa-0", Write, Read)

    kAddEnumerator(kCfa, kCFA_NONE)
    kAddEnumerator(kCfa, kCFA_BAYER_BGGR)
    kAddEnumerator(kCfa, kCFA_BAYER_GBRG)
    kAddEnumerator(kCfa, kCFA_BAYER_RGGB)
    kAddEnumerator(kCfa, kCFA_BAYER_GRBG)
kEndEnumEx()

kFx(kBool) xkCfa_VEquals(kType type, const void* value, const void* other)
{
    return *(kCfa*)value == *(kCfa*)other; 
}

kFx(kSize) xkCfa_VHashCode(kType type, const void* value)
{
    return (kSize) *(kCfa*)value; 
}

kFx(kStatus) xkCfa_Write(kType type, const void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Write32sArray(serializer, (const k32s*) values, count); 
}

kFx(kStatus) xkCfa_Read(kType type, void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Read32sArray(serializer, (k32s*) values, count); 
}

kBeginValueEx(k, kRgb)
    kAddPrivateVersionEx(kRgb, "kdat5", "5.0.0.0", "179-0", Write, Read)
    kAddPrivateVersionEx(kRgb, "kdat6", "5.7.1.0", "kRgb-0", Write, Read)

    kAddField(kRgb, k8u, b)
    kAddField(kRgb, k8u, g)
    kAddField(kRgb, k8u, r)
    kAddField(kRgb, k8u, x)
kEndValueEx()

kFx(kBool) xkRgb_VEquals(kType type, const void* value, const void* other)
{
    const kRgb* a = (const kRgb*) value; 
    const kRgb* b = (const kRgb*) other; 

    return (a->b == b->b) && (a->g == b->g) && (a->r == b->r); 
}

kFx(kSize) xkRgb_VHashCode(kType type, const void* value)
{
    return xkHashBytes(value, 3); 
}

kFx(kStatus) xkRgb_Write(kType type, const void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Write8uArray(serializer, (const k8u*) values, 4*count); 
}

kFx(kStatus) xkRgb_Read(kType type, void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Read8uArray(serializer, (k8u*) values, 4*count); 
}

kBeginValueEx(k, kArgb)
    kAddPrivateVersionEx(kArgb, "kdat5", "5.0.0.0", "180-0", Write, Read)
    kAddPrivateVersionEx(kArgb, "kdat6", "5.7.1.0", "kArgb-0", Write, Read)

    kAddField(kArgb, k8u, b)
    kAddField(kArgb, k8u, g)
    kAddField(kArgb, k8u, r)
    kAddField(kArgb, k8u, a)
kEndValueEx()

kFx(kBool) xkArgb_VEquals(kType type, const void* value, const void* other)
{
    const kArgb* a = (const kArgb*) value; 
    const kArgb* b = (const kArgb*) other; 

    return (a->b == b->b) && (a->g == b->g) && (a->r == b->r) && (a->a == b->a); 
}

kFx(kSize) xkArgb_VHashCode(kType type, const void* value)
{
    return xkHashBytes(value, kType_Size(type)); 
}

kFx(kStatus) xkArgb_Write(kType type, const void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Write8uArray(serializer, (const k8u*) values, 4*count); 
}

kFx(kStatus) xkArgb_Read(kType type, void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Read8uArray(serializer, (k8u*) values, 4*count); 
}

kBeginValueEx(k, kMacAddress)
    kAddField(kMacAddress, kByte, address)

    kAddPrivateVMethod(kMacAddress, kValue, VEquals)
kEndValueEx()

kFx(kStatus) kMacAddress_Parse(kMacAddress* address, const kChar* text)
{
    k32u parts[6];
    kSize i;

    kItemZero(address, sizeof(kMacAddress));

    if (sscanf(text, "%X-%X-%X-%X-%X-%X", &parts[0], &parts[1], &parts[2], &parts[3], &parts[4], &parts[5]) != 6)
    {
        return kERROR_PARAMETER;
    }

    for (i = 0; i < kCountOf(address->address); ++i)
    {
        address->address[i] = (k8u)parts[i];
    }

    return kOK;
}

kFx(kStatus) kMacAddress_Format(kMacAddress address, kChar* text, kSize capacity)
{
    return kStrPrintf(text, capacity, "%02X-%02X-%02X-%02X-%02X-%02X",
        address.address[0], address.address[1], address.address[2],
        address.address[3], address.address[4], address.address[5]);
}

kFx(kBool) xkMacAddress_VEquals(kType type, const void* value, const void* other)
{
    const kMacAddress* a = (const kMacAddress*) value; 
    const kMacAddress* b = (const kMacAddress*) other; 

    return (a->address[0] == b->address[0] && 
        a->address[1] == b->address[1] && 
        a->address[2] == b->address[2] && 
        a->address[3] == b->address[3] && 
        a->address[4] == b->address[4] && 
        a->address[5] == b->address[5]);
}

kBeginEnumEx(k, kComparison)
    kAddEnumerator(kComparison, kCOMPARISON_EQ)
    kAddEnumerator(kComparison, kCOMPARISON_NEQ)
    kAddEnumerator(kComparison, kCOMPARISON_LT)
    kAddEnumerator(kComparison, kCOMPARISON_LTE)
    kAddEnumerator(kComparison, kCOMPARISON_GT)
    kAddEnumerator(kComparison, kCOMPARISON_GTE)
kEndEnumEx()

kFx(kBool) xkComparison_VEquals(kType type, const void* value, const void* other)
{
    return *(kComparison*)value == *(kComparison*)other; 
}

kFx(kSize) xkComparison_VHashCode(kType type, const void* value)
{
    return (kSize)*(kComparison*)value; 
}

kFx(kStatus) xkComparison_Write(kType type, const void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Write32sArray(serializer, (const k32s*) values, count); 
}

kFx(kStatus) xkComparison_Read(kType type, void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Read32sArray(serializer, (k32s*) values, count); 
}

kBeginValueEx(k, kCallbackFx)
    kAddPrivateVMethod(kCallbackFx, kValue, VEquals)
    kAddPrivateVMethod(kCallbackFx, kValue, VHashCode)
kEndValueEx()

kFx(kBool) xkCallbackFx_VEquals(kType type, const void* value, const void* other)
{
    return *(kCallbackFx*)value == *(kCallbackFx*)other; 
}

kFx(kSize) xkCallbackFx_VHashCode(kType type, const void* value)
{
    return xkHashBytes(value, sizeof(kCallbackFx)); 
}

kBeginValueEx(k, kCallback)
    kAddField(kCallback, kCallbackFx, function);  
    kAddField(kCallback, kPointer, receiver);  

    kAddPrivateVMethod(kCallback, kValue, VEquals)
    kAddPrivateVMethod(kCallback, kValue, VHashCode)
kEndValueEx()

kFx(kBool) xkCallback_VEquals(kType type, const void* value, const void* other)
{
    const kCallback* a = (const kCallback*) value; 
    const kCallback* b = (const kCallback*) other; 
    
    return (a->function == b->function) && (a->receiver == b->receiver); 
}

kFx(kSize) xkCallback_VHashCode(kType type, const void* value)
{
    const kCallback* v = (const kCallback*) value; 

    return kValue_HashCode(kTypeOf(kCallbackFx), &v->function) ^ 
           kValue_HashCode(kTypeOf(kPointer), &v->receiver); 
}

kBeginEnumEx(k, kFileMode)
    kAddEnumerator(kFileMode, kFILE_MODE_READ)
    kAddEnumerator(kFileMode, kFILE_MODE_WRITE)
    kAddEnumerator(kFileMode, kFILE_MODE_UPDATE)
kEndEnumEx()

kBeginEnumEx(k, kSeekOrigin)
    kAddEnumerator(kSeekOrigin, kSEEK_ORIGIN_BEGIN)
    kAddEnumerator(kSeekOrigin, kSEEK_ORIGIN_CURRENT)
    kAddEnumerator(kSeekOrigin, kSEEK_ORIGIN_END)
kEndEnumEx()

kBeginEnumEx(k, kCompressionType)
    kAddEnumerator(kCompressionType, kCOMPRESSION_TYPE_ZSTD)
kEndEnumEx()

kBeginEnumEx(k, kCompressionPreset)
    kAddEnumerator(kCompressionPreset, kCOMPRESSION_PRESET_MIN);
    kAddEnumerator(kCompressionPreset, kCOMPRESSION_PRESET_FAST);
    kAddEnumerator(kCompressionPreset, kCOMPRESSION_PRESET_DEFAULT);
    kAddEnumerator(kCompressionPreset, kCOMPRESSION_PRESET_DENSE);
    kAddEnumerator(kCompressionPreset, kCOMPRESSION_PRESET_MAX);
kEndEnumEx()

kBeginEnumEx(k, kMemoryAlignment)
    kAddEnumerator(kMemoryAlignment, kMEMORY_ALIGNMENT_8);
    kAddEnumerator(kMemoryAlignment, kMEMORY_ALIGNMENT_16);
    kAddEnumerator(kMemoryAlignment, kMEMORY_ALIGNMENT_32);
    kAddEnumerator(kMemoryAlignment, kMEMORY_ALIGNMENT_64);
    kAddEnumerator(kMemoryAlignment, kMEMORY_ALIGNMENT_128);
    kAddEnumerator(kMemoryAlignment, kMEMORY_ALIGNMENT_256);
    kAddEnumerator(kMemoryAlignment, kMEMORY_ALIGNMENT_512);
    kAddEnumerator(kMemoryAlignment, kMEMORY_ALIGNMENT_1024);
    kAddEnumerator(kMemoryAlignment, kMEMORY_ALIGNMENT_2048);
    kAddEnumerator(kMemoryAlignment, kMEMORY_ALIGNMENT_4096);
kEndEnumEx()

kBeginValueEx(k, kLogOption)
    kAddEnumerator(kLogOption, kLOG_OPTION_WARNING);
    kAddEnumerator(kLogOption, kLOG_OPTION_ERROR);
    kAddEnumerator(kLogOption, kLOG_OPTION_PLATFORM);
kEndValueEx()

kBeginEnumEx(k, kAllocTrait)
    kAddEnumerator(kAllocTrait, kALLOC_TRAIT_FOREIGN)
    kAddEnumerator(kAllocTrait, kALLOC_TRAIT_SERIAL)
    kAddEnumerator(kAllocTrait, kALLOC_TRAIT_NON_ATOMIC)
    kAddEnumerator(kAllocTrait, kALLOC_TRAIT_CONTEXT)
    kAddEnumerator(kAllocTrait, kALLOC_TRAIT_CUDA_PINNED)
    kAddEnumerator(kAllocTrait, kALLOC_TRAIT_CUDA_PINNED)
    kAddEnumerator(kAllocTrait, kALLOC_TRAIT_CUDA_MANAGED)
    kAddEnumerator(kAllocTrait, kALLOC_TRAIT_CUDA_DEVICE)
kEndEnumEx()

kBeginValueEx(k, kThreadId)
kEndValueEx()

kFx(kBool) kThreadId_Compare(kThreadId a, kThreadId b)
{
    return xkThread_CompareId((xkThreadId)a, (xkThreadId)b);
}

kBeginValueEx(k, kThreadPriorityClass)
    kAddEnumerator(kThreadPriorityClass, kTHREAD_PRIORITY_CLASS_LOW)
    kAddEnumerator(kThreadPriorityClass, kTHREAD_PRIORITY_CLASS_NORMAL)
    kAddEnumerator(kThreadPriorityClass, kTHREAD_PRIORITY_CLASS_HIGH)
kEndValueEx()

//this method is intentionally not inlined, to reduce bloat in debug builds
kFx(kBool) xkCastClass_ObjectIs(kPointer object, kPointer type)
{
    return kObject_Is(object, type);
}

//this method is intentionally not inlined, to reduce bloat in debug builds
kFx(kBool) xkCastClass_ObjectIsValid(kPointer object, kPointer type)
{
    return !kIsNull(object) && 
            xkObject_RawVerifyTag(object) && 
            (kIsNull(xkObject_RawType(object)) || (xkObject_RawType(object) == type)); 
}

kFx(void) xkCheckBase(const kChar* prefix, const kChar* typeName, const kChar* definitionBase, const kChar* declarationBase)
{
    if (!kStrEquals(definitionBase, declarationBase))
    {
        kLogf("The base specified for type %s in its declaration (e.g., \n"
              "    kDeclareClass(%s, %s, %s)\n" 
              "does not match the base specified in its definition (e.g., \n"
              "    kBeginClass(%s, %s, %s)", 
              typeName, prefix, typeName, declarationBase, prefix, typeName, definitionBase); 
        kAssert(kFALSE); 
    }
}

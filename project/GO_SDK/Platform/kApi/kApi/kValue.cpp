/** 
 * @file    kValue.cpp
 *
 * @internal
 * Copyright (C) 2012-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <kApi/kValue.h>

xkBeginVirtualValueEx(k, kValue)
    kAddVMethod(kValue, kValue, VEquals)
    kAddVMethod(kValue, kValue, VHashCode)
    kAddVMethod(kValue, kValue, VImport)
xkEndVirtualValueEx()

kFx(kBool) kValue_VEquals(kType type, const void* value, const void* other)
{
    if (!kType_IsArrayValue(type))
    {
        kSize fieldCount = kType_FieldCount(type);
        kSize i;

        for (i = 0; i < fieldCount; ++i)
        {
            const kFieldInfo* field = kType_FieldInfoAt(type, i);

            if (!kType_IsValue(field->type) || !kValue_Equals(field->type, kPointer_ByteOffset(value, (kSSize)field->offset), kPointer_ByteOffset(other, (kSSize)field->offset)))
            {
                return kFALSE;
            }
        }
    }
    else
    {
        const kFieldInfo* field = kType_FieldInfoAt(type, 0); 
        kSize i;

        for (i = 0; i < field->count; ++i)
        {
            kSize offset = i * kType_Size(field->type);

            if (!kType_IsValue(field->type) || !kValue_Equals(field->type, kPointer_ByteOffset(value, (kSSize)offset), kPointer_ByteOffset(other, (kSSize)offset)))
            {
                return kFALSE;
            }
        }
    }

    return kTRUE; 
}

kFx(kSize) kValue_VHashCode(kType type, const void* value)
{
    kSize hash = 1; 

    if (!kType_IsArrayValue(type))
    {
        kSize fieldCount = kType_FieldCount(type);
        kSize i;

        for (i = 0; i < fieldCount; ++i)
        {
            const kFieldInfo* field = kType_FieldInfoAt(type, i);

            if (kType_IsValue(field->type))
            {
                hash = hash * 31 + kValue_HashCode(field->type, kPointer_ByteOffset(value, (kSSize)field->offset)); 
            }
        }
    }
    else
    {
        const kFieldInfo* field = kType_FieldInfoAt(type, 0); 
        kSize i;

        if (kType_IsValue(field->type))
        {
            for (i = 0; i < field->count; ++i)
            {
                kSize offset = i * kType_Size(field->type);

                hash = hash * 31 + kValue_HashCode(field->type, kPointer_ByteOffset(value, (kSSize)offset)); 
            }
        }
    }

    return hash; 
}

/** 
 * @file    kValue.x.h
 *
 * @internal
 * Copyright (C) 2012-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_VALUE_X_H
#define K_API_VALUE_X_H

typedef struct kValueVTable
{
    kBool (kCall* VEquals)(kType type, const void* value, const void* other); 
    kSize (kCall* VHashCode)(kType type, const void* value); 
    void (kCall* VImport)(kType type, void* value, const void* source); 
} kValueVTable; 

xkDeclareVirtualValue(k, kValue, kNull)

/* 
* Forward declarations. 
*/

kInlineFx(kBool) kValue_Equals(kType type, const void* value, const void* other); 
kInlineFx(kSize) kValue_HashCode(kType type, const void* value); 

/* 
* Private methods. 
*/

kInlineFx(kBool) xkValue_EqualsT(kType type, const void* value, const void* other, kSize valueSize, kSize otherSize)
{
    kAssert(xkType_IsPointerCompatible(type, valueSize)); 
    kAssert(xkType_IsPointerCompatible(type, otherSize)); 

    return kValue_Equals(type, value, other);
}

kInlineFx(kSize) xkValue_HashCodeT(kType type, const void* value, kSize valueSize) 
{
    kAssert(xkType_IsPointerCompatible(type, valueSize)); 
   
    return kValue_HashCode(type, value); 
}

#endif

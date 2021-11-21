/** 
 * @file    kBytes.cpp
 *
 * @internal
 * Copyright (C) 2012-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <kApi/Data/kBytes.h>
#include <kApi/kApiLib.h>
#include <kApi/Io/kSerializer.h>
#include <stdio.h>

static kType kBytes_types[xkBYTES_CAPACITY] = { 0 }; 

kFx(kStatus) xkBytes_AddTypes(kAssembly assembly)
{
    kChar name[64];
    kSize i; 
    
    kCheck(kMemSet(&kBytes_types[0], 0, sizeof(kBytes_types))); 
    
    //add each kByteN type
    for (i = 1; i < xkBYTES_CAPACITY; ++i)
    {
        kCheck(kStrPrintf(name, kCountOf(name), "kByte%u", i)); 
        kCheck(xkAssembly_AddType(assembly, xkBytes_Register, name)); 
    }

    return kOK; 
}

kFx(kStatus) xkBytes_Register(kAssembly assembly, const kChar* name)
{    
    kSize guid5Base = 0x40000000; 
    kChar guid5[64];
    kChar guid6[64];
    k32u typeIndex; 

    kCheckArgs(sscanf(name, "kByte%u", &typeIndex) == 1); 

    kCheck(kStrPrintf(guid5, kCountOf(guid5), "%u-0", guid5Base + typeIndex)); 
    kCheck(kStrPrintf(guid6, kCountOf(guid6), "kBytes%u-0", typeIndex)); 

    kCheck(xkAssembly_AddValue(assembly, &kBytes_types[typeIndex], name, kTypeOf(kValue), "kValue", typeIndex, kTYPE_FLAGS_VALUE)); 

    kCheck(xkType_AddVMethod(kBytes_types[typeIndex], offsetof(kValueVTable, VEquals)/sizeof(kPointer), (kFunction)xkBytes_VEquals, "VEquals")); 
    kCheck(xkType_AddVMethod(kBytes_types[typeIndex], offsetof(kValueVTable, VHashCode)/sizeof(kPointer), (kFunction)xkBytes_VHashCode, "VHashCode")); 

    kCheck(xkType_AddVersion(kBytes_types[typeIndex], "kdat5", "5.0.0.0", guid5, (kFunction)xkBytes_Write, (kFunction)xkBytes_Read, kTRUE));  
    kCheck(xkType_AddVersion(kBytes_types[typeIndex], "kdat6", "5.7.1.0", guid6, (kFunction)xkBytes_Write, (kFunction)xkBytes_Read, kTRUE));  

    return kOK; 
}

kFx(kBool) xkBytes_VEquals(kType type, const void* value, const void* other)                    
{                                                                                              
    return kMemEquals(value, other, kType_Size(type));                                                      
}           

kFx(kSize) xkBytes_VHashCode(kType type, const void* value)                                     
{                                                                                              
    return xkHashBytes(value, kType_Size(type));                                                             
}

kFx(kStatus) xkBytes_Write(kType type, const void* values, kSize count, kSerializer serializer)       
{                                                                                              
    return kSerializer_WriteByteArray(serializer, (const kByte*) values, kType_Size(type)*count);                         
}

kFx(kStatus) xkBytes_Read(kType type, void* values, kSize count, kSerializer serializer)        
{                                                                                              
    return kSerializer_ReadByteArray(serializer, (kByte*) values, kType_Size(type)*count);                          
}

kFx(kType) kBytes_GetType(kSize size)
{
    if (size < xkBYTES_CAPACITY)
    {
        return kBytes_types[size]; 
    }
    else
    {
        //If this assertion fails, the requested size is larger than the maximum size 
        //supported by the kTYPE_BYTES macro. Consider defining a custom value type instead.
        kAssert(size < xkBYTES_CAPACITY); 

        return kNULL; 
    }
}

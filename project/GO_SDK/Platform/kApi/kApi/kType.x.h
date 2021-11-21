/** 
 * @file  kType.x.h
 *
 * @internal
 * Copyright (C) 2005-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_TYPE_X_H
#define K_API_TYPE_X_H

#define xkTYPE_MAX_BASES                     (4)
#define xkTYPE_MAX_INTERFACES                (4)

#define xkTYPE_GROWTH_FACTOR                 (2)
#define xkTYPE_INITIAL_CMETHOD_CAPACITY      (16)
#define xkTYPE_INITIAL_FIELD_CAPACITY        (16)
#define xkTYPE_INITIAL_ENUMERATOR_CAPACITY   (16)
#define xkTYPE_INITIAL_VERSION_CAPACITY      (16)

//private flags
#define xkTYPE_FLAGS_PACKED               (0x00010000)       ///< Value type has no gaps between fields. 


typedef kStatus (kCall* xkStaticInitFx)(); 
typedef kStatus (kCall* xkStaticReleaseFx)(); 

//used by individual container types, not the type system itself. 
typedef struct xkStructField
{
    kType type;             //field type
    kSize offset;           //field offset, in bytes
    kSize count;            //field element count, in items
    kSize typeSize;         //field element size, in bytes
    kSize fieldSize;        //field size, in bytes
} xkStructField;

/**
* @struct  kTypeVersionInfo
* @ingroup kApi
* @brief   Represents serialization version information.
*/
typedef struct kTypeVersionInfo
{
    kText16 format;                             ///< Serialization format name (e.g. "kdat6").
    kVersion formatVersion;                     ///< Serialization format version (e.g. "6.0.0.0").
    kText64 guid;                               ///< Unique id (e.g. "kArrayList-0").
    kFunction serialize;                        ///< Serialization method.
    kFunction deserialize;                      ///< Deserialization method.
    kBool hasLegacyDeserializer;                ///< True if version has legacy, self-initializing deserializer method. 
} kTypeVersionInfo;

typedef struct xkInterfaceInfo
{
    kType type; 
    kFunction* iTable; 
    kMethodInfo* iMethodInfo; 
    kSize iMethodCount; 
} xkInterfaceInfo; 

typedef struct kTypeClass
{
    kObjectClass base; 
    kAssembly assembly; 
    kTypeName name; 
    kType* selfReference; 
    kTypeFlags flags; 
    kSize size;
    kSize innerSize;
    kType bases[xkTYPE_MAX_BASES]; 
    kSize baseCount;
    kFrameworkConstructorFx frameworkConstructor;
    kFunction* vTable;
    kMethodInfo* vMethodInfo; 
    kSize vMethodCount;
    kMethodInfo* cMethodInfo; 
    kSize cMethodCount;
    kSize cMethodCapacity;
    kFieldInfo* fieldInfo; 
    kSize fieldCount; 
    kSize fieldCapacity; 
    kEnumeratorInfo* enumeratorInfo; 
    kSize enumeratorCount; 
    kSize enumeratorCapacity; 
    kPointer staticData; 
    kSize staticSize; 
    xkStaticInitFx staticInit; 
    xkStaticReleaseFx staticRelease; 
    kBool staticInitialized; 
    k32u staticPriority; 
    xkInterfaceInfo interfaces[xkTYPE_MAX_INTERFACES]; 
    kSize interfaceCount;
    kTypeVersionInfo* versionInfo;
    kSize versionCount; 
    kSize versionCapacity; 
} kTypeClass;

kDeclareClassEx(k, kType, kObject)

/* 
* Forward declarations.
*/

kInlineFx(kFunction*) kType_VTable(kType type);
kInlineFx(kBool) kType_Is(kType type, kType other);

/* 
* Private methods. 
*/

//performs less type-checking than usual; required due to early/late uses in type-system lifecycle
kInlineFx(kTypeClass*) xkType_CastMin(kType type) 
{ 
    return xkTypeCastClass_(kType, type); 
}

#define xkTypeObj(T_object) \
    kTypeClass* obj = xkType_CastMin(T_object)

#define xkTypeObjN(T_object) \
    kTypeClass* T_object##Obj = xkType_CastMin(T_object)

kFx(kStatus) xkType_LayoutStruct(xkStructField** fields, kSize fieldCount, kSize* structureSize); 

kFx(kStatus) xkType_Construct(kType* type, kType* reference, const kChar* name, kAssembly assembly, kTypeFlags flags, kSize size, kSize innerSize); 
kFx(kStatus) xkType_Init(kType type, kType typeType, kAlloc allocator, kType* reference, const kChar* name, kAssembly assembly, kTypeFlags flags, kSize size, kSize innerSize); 
kFx(kStatus) xkType_VRelease(kType type);

kFx(kStatus) xkType_SetBase(kType type, kType base);
kFx(kStatus) xkType_InitMethods(kType type);
kFx(kStatus) xkType_InitVTable(kType type);
kFx(kStatus) xkType_InitInterfaces(kType type);

kFx(kStatus) xkType_ReserveVTable(kType type, kSize vMethodCount);

kFx(kStatus) xkType_ImplementInterface(kType type, kType interfaceType, const kChar* interfaceName, kSize iTableSize); 

kFx(kStatus) xkType_AddStatic(kType type, kSize staticSize, xkStaticInitFx init, xkStaticReleaseFx release); 
kFx(kStatus) xkType_AddVersion(kType type, const kChar* format, const kChar* formatVersion, const kChar* guid, kFunction serialize, kFunction deserialize, kBool hasLegacyDeserializer); 
kFx(kStatus) xkType_AddFlags(kType type, kTypeFlags flags); 
kFx(kStatus) xkType_AddFrameworkConstructor(kType type, kFunction constructor); 
kFx(kStatus) xkType_AddMethod(kType type, kFunction function, const kChar* methodName); 
kFx(kStatus) xkType_AddVMethod(kType type, kSize index, kFunction function, const kChar* methodName); 
kFx(kStatus) xkType_AddIVMethod(kType type, kType interfaceType, kSize index, kFunction function, const kChar* iMethodName, const kChar* cMethodName); 
kFx(kStatus) xkType_AddField(kType type, kType fieldType, const kChar* fieldTypeName, kSize size, kSize offset, kSize count, const kChar* fieldName); 
kFx(kStatus) xkType_AddEnumerator(kType type, k32s value, const kChar* name); 

kFx(kStatus) xkType_FormatEnumeratorDisplayName(kType type, const kChar* enumeratorName, kChar* buffer, kSize capacity); 
kFx(kStatus) xkType_FindEnumeratorInfoByName(kType type, const kChar* displayName, const kEnumeratorInfo** info); 

kFx(kStatus) xkType_SetInitPriority(kType type, k32u priority);

kFx(kStatus) xkType_Prepare(kType type);

kFx(kBool) xkType_DetermineIsPacked(kType type); 

kFx(kStatus) xkType_RunStaticInit(kType type);
kFx(kStatus) xkType_RunStaticRelease(kType type); 

kFx(kStatus) xkType_FrameworkConstructorStub(kObject* object, kAlloc allocator); 

kFx(kStatus) xkType_ObjectSerializationStub(kObject object, kSerializer serializer); 
kFx(kStatus) xkType_ObjectDeserializationStub(kObject object, kSerializer serializer, kAlloc allocator);

kFx(kStatus) xkType_ValueSerializationStub(kType type, const void* values, kSize count, kSerializer serializer); 
kFx(kStatus) xkType_ValueDeserializationStub(kType type, void* values, kSize count, kSerializer serializer); 

kFx(k32u) xkType_Priority(kType type); 

k32s xkType_LayoutStructOffsetComparator(const void* field1, const void* field2); 
k32s xkType_LayoutStructSizeComparator(const void* field1, const void* field2);

kFx(kSize) xkType_MaxPrimitiveSize(kType type); 

kInlineFx(xkInterfaceInfo*) xkType_InterfaceInfo(kType type, kType iface)
{
    xkTypeObj(type); 

    if      (obj->interfaces[0].type == iface)      return &obj->interfaces[0]; 
    else if (obj->interfaces[1].type == iface)      return &obj->interfaces[1]; 
    else if (obj->interfaces[2].type == iface)      return &obj->interfaces[2]; 
    else if (obj->interfaces[3].type == iface)      return &obj->interfaces[3]; 
    else                                            return kNULL;  
}

kInlineFx(kBool) xkType_IsPointerCompatible(kType type, kSize size)
{
    xkTypeObj(type); 

    if ((obj->flags & kTYPE_FLAGS_ARRAY_VALUE) == 0)
    {
        return obj->size == size; 
    }
    else
    {
        return (obj->size == size) || (xkType_CastMin(obj->fieldInfo[0].type)->size == size);        
    } 
}

kInlineFx(kFunction*) xkType_VTableT(kType type, kType other)
{ 
    kAssert(kType_Is(type, other));  

    return kType_VTable(type);
}

kInlineFx(kFunction*) xkType_IVTableT(kType type, kType interfaceType)
{ 
    kFunction* vTable = kType_IVTable(type, interfaceType);

    kAssert(!kIsNull(vTable)); 
    
    return vTable; 
}

/* 
* Deprecated (Stage 1): not recommended for further use, but not yet announced via kDeprecate
*/

#define kTYPE_FLAGS_BIT_ENUM              (0x10)             ///< Type is a bitset enumeration. 

kInlineFx(kBool) kType_IsBitEnum(kType type)
{ 
    xkTypeObj(type); 

    return (obj->flags & kTYPE_FLAGS_BIT_ENUM) != 0; 
}

#endif

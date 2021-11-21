/** 
 * @file    kType.h
 * @brief   Declares the kType class. 
 *
 * @internal
 * Copyright (C) 2005-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <kApi/kApiDef.h>   //--inclusion order controlled by kApiDef

#ifndef K_API_TYPE_H
#define K_API_TYPE_H

#include <kApi/kType.x.h>

/**
 * @class   kType
 * @extends kObject
 * @ingroup kApi
 * @brief   Represents metadata about a type (class, interface, or value). 
 * 
 * When an assembly is constructed, one kType instance is created for every type defined in the assembly. 
 * kAssembly methods can be used to discover the types in the assembly, and kType methods can be used to 
 * learn about individual types (e.g., type name, base class). 
 * 
 * The kObject_Type method can be used to obtain type information for any object derived from kObject. 
 * The kTypeOf macro can be used to obtain type information for any class, interface, or value by 
 * compile-time type symbol. 
 * 
 * @code {.c}
 * 
 * void DescribeObjectType(kObject object)
 * {
 *     kType type = kObject_Type(object);
 *     kType base = kType_Base(type);
 *
 *     printf("Type name: %s\n", kType_Name(type));
 *     printf("Base type: %s\n", (base == 0) ? "(None)" : kType_Name(base));
 *     printf("Type assembly: %s\n", kAssembly_Name(kType_Assembly(type))); 
 *     printf("Virtual method count: %u\n", (k32u) kType_VMethodCount(type));
 *     printf("Extends kStream?: %s\n", kType_Extends(type, kTypeOf(kStream)) ? "Yes" : "No"); 
 *     printf("Implements kCollection?: %s\n", kType_Implements(type, kTypeOf(kCollection)) ? "Yes" : "No"); 
 *
 * }
 * 
 * @endcode
 * 
 */
//typedef kObject kType;   --forward-declared in kApiDef.x.h

/** 
 * Gets the assembly to which the type belongs. 
 *
 * @public              @memberof kType
 * @param   type        Type. 
 * @return              Assembly. 
 */
kInlineFx(kAssembly) kType_Assembly(kType type)
{
    xkTypeObj(type); 

    return obj->assembly; 
}
/** 
 * Gets the name of the type. 
 *
 * @public              @memberof kType
 * @param   type        Type. 
 * @return              Type name. 
 */
kInlineFx(const kChar*) kType_Name(kType type)
{
    xkTypeObj(type); 
    
    return obj->name; 
}

/** 
 * Determines whether a type is equivalent to another type. 
 *
 * Checks type equality, inheritance, and interfaces. 
 *
 * @public              @memberof kType
 * @param   type        Type to be compared. 
 * @param   other       Type to which type is compared. 
 * @return              kTRUE if type is equivalent. 
 */
kInlineFx(kBool) kType_Is(kType type, kType other)
{ 
    return (type == other) || 
           kType_Extends(type, other) || 
           kType_Implements(type, other); 
}

/** 
 * Determines whether a type represents a value (primitive, struct, enum). 
 *
 * @public              @memberof kType
 * @param   type        Type. 
 * @return              kTRUE if type represents a value.  
 */
kInlineFx(kBool) kType_IsValue(kType type)
{ 
    xkTypeObj(type); 

    return (obj->flags & kTYPE_FLAGS_VALUE) != 0; 
}

/** 
 * Determines whether a type represents a class. 
 *
 * @public              @memberof kType
 * @param   type        Type. 
 * @return              kTRUE if type represents a class.  
 */
kInlineFx(kBool) kType_IsClass(kType type) 
{ 
    xkTypeObj(type); 

    return (obj->flags & kTYPE_FLAGS_CLASS) != 0; 
}

/** 
 * Determines whether a type represents an interface.  
 *
 * @public              @memberof kType
 * @param   type        Type. 
 * @return              kTRUE if type represents an interface. 
 */
kInlineFx(kBool) kType_IsInterface(kType type)
{ 
    xkTypeObj(type); 

    return (obj->flags & kTYPE_FLAGS_INTERFACE) != 0; 
}

/** 
 * Determines whether a type represents a class or interface. 
 *
 * @public              @memberof kType
 * @param   type        Type. 
 * @return              kTRUE if type represents a reference. 
 */
kInlineFx(kBool) kType_IsReference(kType type)
{ 
    xkTypeObj(type); 

    return (obj->flags & (kTYPE_FLAGS_INTERFACE | kTYPE_FLAGS_CLASS)) != 0; 
}

/** 
 * Determines whether a type represents an abstract class. 
 *
 * @public              @memberof kType
 * @param   type        Class type. 
 * @return              kTRUE if type is abstract. 
 */
kInlineFx(kBool) kType_IsAbstract(kType type)
{ 
    xkTypeObj(type); 

    return (obj->flags & kTYPE_FLAGS_ABSTRACT) != 0; 
}

/** 
 * Reports whether the type is an enumeration. 
 *
 * @public          @memberof kType
 * @return          kTRUE if the type is an enumeration; otherwise, kFALSE. 
 */
kInlineFx(kBool) kType_IsEnum(kType type)
{
    xkTypeObj(type); 

    return (obj->flags & kTYPE_FLAGS_ENUM) != 0; 
}

/** 
 * Reports whether the type is an 'array-value' type (e.g., kText32)
 *
 * @public          @memberof kType
 * @return          kTRUE if the type is an 'array-value' type; otherwise, kFALSE. 
 */
kInlineFx(kBool) kType_IsArrayValue(kType type)
{ 
    xkTypeObj(type); 

    return (obj->flags & kTYPE_FLAGS_ARRAY_VALUE) != 0; 
}

/**
* Reports whether the type is a primitive value (single-valued; lacks fields). 
*
* @public          @memberof kType
* @return          kTRUE if the type is a primitive value.
*/
kInlineFx(kBool) kType_IsPrimitive(kType type)
{ 
    xkTypeObj(type); 

    return (obj->flags & kTYPE_FLAGS_PRIMITIVE) != 0;
}

/**
* Reports whether the fields of a value type are tightly packed (free from structure padding).
*
* @public          @memberof kType
* @return          kTRUE if the type is tightly packed.
*/
kInlineFx(kBool) kType_IsPacked(kType type) 
{ 
    xkTypeObj(type); 

    return (obj->flags & xkTYPE_FLAGS_PACKED) != 0; 
}

/** 
 * Determines whether a type implements a specific interface.  
 *
 * @public                  @memberof kType
 * @param   type            Type. 
 * @param   interfaceType   Interface type. 
 * @return                  kTRUE if type implements interface. 
 */
kInlineFx(kBool) kType_Implements(kType type, kType interfaceType)
{ 
    return kType_IVTable(type, interfaceType) != kNULL; 
}

/** 
 * Determines whether a type extends another type. 
 *
 * @public                  @memberof kType
 * @param   type            Type. 
 * @param   baseType        Base type. 
 * @return                  kTRUE if type extends base type.
 */
kInlineFx(kBool) kType_Extends(kType type, kType baseType)
{
    xkTypeObj(type); 

    return obj->bases[0] == baseType || 
           obj->bases[1] == baseType ||
           obj->bases[2] == baseType ||
           obj->bases[3] == baseType; 
}

/** 
 * Gets the base of a class or interface. 
 *
 * @public                  @memberof kType
 * @param   type            Type.
 * @return                  Base type (or kNULL if none). 
 */
kInlineFx(kType) kType_Base(kType type)
{ 
    xkTypeObj(type); 

    return obj->bases[0]; 
}

/** 
 * Reports count of implemented interfaces. 
 * 
 * @public          @memberof kType
 * @param   type    Type.
 * @return          Count of implemented interface. 
 */
kInlineFx(kSize) kType_InterfaceCount(kType type)
{
    xkTypeObj(type); 

    return obj->interfaceCount; 
}

/** 
 * Gets the implemented interface at the specified index. 
 * 
 * @public          @memberof kType
 * @param   type    Type.
 * @param   index   Interface index.
 * @return          Interface type.
 */
kInlineFx(kType) kType_InterfaceAt(kType type, kSize index)
{
    xkTypeObj(type); 

    kAssert(index < obj->interfaceCount);

    return obj->interfaces[index].type;
}

/** 
 * Gets the external size of a type. 
 *
 * The external size of a value type reflects the size of the struct, value or enum. The external size of a reference 
 * type is equal to the size of a pointer. 
 *
 * @public                  @memberof kType
 * @param   type            Type.
 * @return                  Type size. 
 */
kInlineFx(kSize) kType_Size(kType type)
{ 
    xkTypeObj(type); 

    return obj->size; 
}

/** 
 * Gets the internal size of a type. 
 *
 * The internal size of a value type reflects the size of the struct, value or enum. The internal size of a reference 
 * type is equal to the size of its class structure. 
 *
 * @public                  @memberof kType
 * @param   type            Type.
 * @return                  Type size. 
 */
kInlineFx(kSize) kType_InnerSize(kType type) 
{ 
    xkTypeObj(type); 

    return obj->innerSize; 
}

/** 
 * Gets the size of a type's static data. 
 *
 * @public                  @memberof kType
 * @param   type            Type.
 * @return                  Static data size. 
 */
kInlineFx(kSize) kType_StaticSize(kType type)
{
    xkTypeObj(type);

    return obj->staticSize;
}

/** 
 * Gets a pointer to the type's primary virtual method table. 
 *
 * @public          @memberof kType
 * @param   type    Type.
 * @return          Virtual table pointer.
 */
kInlineFx(kFunction*) kType_VTable(kType type)
{ 
    xkTypeObj(type); 

    return obj->vTable; 
}

/** 
 * Gets a strongly-typed pointer to the type's primary virtual method table. 
 *
 * This method raises a debug assertion if the specified type object is not an instance
 * of the specified type identifier.
 * 
 * @relates             kType
 * @param   kType_type  Type object. 
 * @param   T           Type identifier (e.g., kObject). 
 * @return              Strongly-typed virtual table pointer.
 */
#define kType_VTableT(kType_type, T) \
    kCast(T##VTable*, xkType_VTableT(kType_type, kTypeOf(T)))

/** 
 * Gets a pointer to the type's virtual method table corresponding to the specified interface type. 
 *
 * @public                   @memberof kType
 * @param   type             Type.
 * @param   interfaceType    Interface type.
 * @return  Interface virtual table pointer.
 */
kInlineFx(kFunction*) kType_IVTable(kType type, kType interfaceType)
{
    xkTypeObj(type); 

    if      (obj->interfaces[0].type == interfaceType)      return obj->interfaces[0].iTable; 
    else if (obj->interfaces[1].type == interfaceType)      return obj->interfaces[1].iTable; 
    else if (obj->interfaces[2].type == interfaceType)      return obj->interfaces[2].iTable; 
    else if (obj->interfaces[3].type == interfaceType)      return obj->interfaces[3].iTable; 
    else                                                    return kNULL;  
}

/** 
 * Gets a strongly-typed pointer to the type's virtual method table corresponding to the specified interface type. 
 * 
 * This method raises a debug assertion if the specified type object does not implement 
 * of the specified interface type.
 * 
 * @relates                 kType
 * @param   kType_type      Type object. 
 * @param   T               Interface type identifier (e.g., kCollection). 
 * @return                  Strongly-typed interface virtual table pointer.
 */
#define kType_IVTableT(kType_type, T) \
    kCast(T##VTable*, xkType_IVTableT(kType_type, kTypeOf(T)))

/** 
 * Gets a pointer to the type's static data structure. 
 *
 * @public          @memberof kType
 * @param   type    Type.
 * @return          Pointer to static data. 
 */
kInlineFx(void*) kType_Static(kType type)
{ 
    xkTypeObj(type); 

    return obj->staticData; 
}

/** 
 * Reports whether the type's static data structure has been successfully initialized. 
 * 
 * This function can be helpful during startup, to determine whether the static data for a particular type 
 * has been initialized. This function is not thread-safe.
 *
 * @public          @memberof kType
 * @param   type    Type.
 * @return          kTRUE if static data has been initialized; otherwise, kFALSE. 
 */
kInlineFx(kBool) kType_StaticInitialized(kType type)
{
    kObjR(kType, type); 

    return obj->staticInitialized; 
}

/** 
 * Gets the framework constructor for this type. 
 * 
 * Framework constructors can optionally be registered using the kAddFrameworkConstructor macro. Framework 
 * constructor registration may be required to support specific features (e.g., serialization). 
 * 
 * If a framework constructor has not been registered for the specified type, this method will return 
 * a stub constructor that yields kERROR_UNIMPLEMENTED.
 *
 * @public          @memberof kType
 * @param   type    Type.
 * @return          Framework constructor.
 */
kInlineFx(kFrameworkConstructorFx) kType_FrameworkConstructor(kType type)
{
    xkTypeObj(type); 

    return obj->frameworkConstructor; 
}

/** 
 * Reports count of non-virtual methods. 
 * 
 * Most types do not register non-virtual methods. However, non-virtual methods can optionally be registered 
 * using the kAddMethod macro. This can be useful in specific scenarios requiring non-virtual method reflection.  
 *
 * @public          @memberof kType
 * @param   type    Type.
 * @return          Count of non-virtual methods. 
 */
kInlineFx(kSize) kType_MethodCount(kType type)
{
    xkTypeObj(type); 

    return obj->cMethodCount; 
}

/** 
 * Gets metadata for the non-virtual method at the specified index.
 * 
 * @public          @memberof kType
 * @param   type    Type.
 * @param   index   Method index.
 * @return          Method metadata.
 */
kInlineFx(const kMethodInfo*) kType_MethodInfoAt(kType type, kSize index)
{
    xkTypeObj(type); 

    kAssert(index < obj->cMethodCount);

    return &obj->cMethodInfo[index]; 
}

/** 
 * Finds metadata for the non-virtual method with the specified name.
 * 
 * @public          @memberof kType
 * @param   type    Type.
 * @param   name    Method name. 
 * @param   info    Receives pointer to metadata. 
 * @return          Operation status (kERROR_NOT_FOUND if method info not located).
 */
kFx(kStatus) kType_FindMethodInfo(kType type, const kChar* name, const kMethodInfo** info);

/** 
 * Reports count of virtual methods. 
 * 
 * @public          @memberof kType
 * @param   type    Type.
 * @return          Count of virtual methods. 
 */
kInlineFx(kSize) kType_VMethodCount(kType type)
{
    xkTypeObj(type); 

    return obj->vMethodCount; 
}

/** 
 * Gets metadata for the virtual method at the specified index.
 * 
 * @public          @memberof kType
 * @param   type    Type.
 * @param   index   Method index.
 * @return          Method metadata.
 */
kInlineFx(const kMethodInfo*) kType_VMethodInfoAt(kType type, kSize index)
{
    xkTypeObj(type); 

    kAssert(index < obj->vMethodCount); 
    
    return &obj->vMethodInfo[index]; 
}

/** 
 * Reports count of interface methods for the given interface.
 * 
 * @public                  @memberof kType
 * @param   type            Type.
 * @param   interfaceType   Interface type.
 * @return                  Count of interface methods. 
 */
kInlineFx(kSize) kType_IMethodCount(kType type, kType interfaceType)
{
    xkTypeObj(type); 
    xkInterfaceInfo* info = xkType_InterfaceInfo(obj, interfaceType);   

    return kIsNull(info) ? 0 : info->iMethodCount; 
}

/** 
 * Gets metadata for the interface method at the specified index.
 * 
 * @public                  @memberof kType
 * @param   type            Type.
 * @param   interfaceType   Interface type.
 * @param   index           Method index.
 * @return                  Method metadata.
 */
kInlineFx(const kMethodInfo*) kType_IMethodInfoAt(kType type, kType interfaceType, kSize index)
{
    xkTypeObj(type); 
    xkInterfaceInfo* info = xkType_InterfaceInfo(obj, interfaceType);   

    kAssert(index < info->iMethodCount);

    return kIsNull(info) ? kNULL : &info->iMethodInfo[index]; 
}

/** 
 * Reports count of registered fields for the given type.
 * 
 * Value fields can optionally be registered using the kAddField macro. 
 *
 * @public                  @memberof kType
 * @param   type            Type.
 * @return                  Count of fields. 
 */
kInlineFx(kSize) kType_FieldCount(kType type)
{
    xkTypeObj(type); 

    return obj->fieldCount; 
}

/** 
 * Gets metadata for the field at the specified index.
 * 
 * @public                  @memberof kType
 * @param   type            Type.
 * @param   index           Field index.
 * @return                  Field metadata.
 */
kInlineFx(const kFieldInfo*) kType_FieldInfoAt(kType type, kSize index)
{
    xkTypeObj(type); 

    kAssert(index < obj->fieldCount);

    return &obj->fieldInfo[index];
}

/** 
 * Reports count of registered enumerators for the given enumeration type.
 * 
 * Enumerators can optionally be registered using the kAddEnumerator macro. 
 *
 * @public                  @memberof kType
 * @param   type            Type.
 * @return                  Count of enumerators.
 */
kInlineFx(kSize) kType_EnumeratorCount(kType type)
{
    xkTypeObj(type); 

    return obj->enumeratorCount; 
}

/** 
 * Gets metadata for the enumerator at the specified index.
 * 
 * @public                  @memberof kType
 * @param   type            Type.
 * @param   index           Enumerator index.
 * @return                  Enumerator metadata.
 */
kInlineFx(const kEnumeratorInfo*) kType_EnumeratorInfoAt(kType type, kSize index)
{
    xkTypeObj(type); 

    kAssert(index < obj->enumeratorCount);

    return &obj->enumeratorInfo[index]; 
}

/** 
 * Finds enumerator metadata for the enumerator with the specified value. 
 * 
 * @public          @memberof kType
 * @param   type    Type.
 * @param   value   Enumerator value. 
 * @param   info    Receives pointer to enumerator metadata.
 * @return          Operation status (kERROR_NOT_FOUND if metadata not located). 
 */
kFx(kStatus) kType_FindEnumeratorInfo(kType type, k32s value, const kEnumeratorInfo** info);

/** 
 * Formats an enumerator value to a text buffer using the enumerator display name.
 *
 * If the enumerator value isn't known, the buffer will receive [D], where D is decimal representation 
 * of the enumerator value, and the return value will be kERROR_NOT_FOUND.
 * 
 * Exercise caution when using formatted enumerator names in file formats or communication protocols; 
 * enumerator text names may change if the enumerator source code is modified. Consider using explicit and 
 * stable format/parse functions, or using numeric values instead.
 * 
 * @public              @memberof kType
 * @param   type        Type.
 * @param   value       Enumerator value. 
 * @param   displayName Receives formatted enumerator name. 
 * @param   capacity    Buffer capacity. 
 * @return              Operation status (see notes). 
 */
kFx(kStatus) kType_FormatEnumerator(kType type, k32s value, kChar* displayName, kSize capacity);

/** 
 * Parses an enumerator value from a text buffer using the enumerator display name.
 *
 * If a matching enumerator display name cannot be found, the value field will receive 0 and the return 
 * value will be kERROR_NOT_FOUND. 
 * 
 * @public              @memberof kType
 * @param   type        Type.
 * @param   value       Receives parsed enumerator value. 
 * @param   displayName Enumerator display name. 
 * @return              Operation status (see notes). 
 */
kFx(kStatus) kType_ParseEnumerator(kType type, k32s* value, const kChar* displayName);

/**
 * Reports count of registered type versions.
 *
 * Type versions are used in serialization. Type versions can optionally 
 * be registered using the kAddVersionEx or kAddPrivateVersionEx macro.
 *
 * @public                  @memberof kType
 * @param   type            Type.
 * @return                  Count of type versions.
 */
kInlineFx(kSize) kType_VersionCount(kType type)
{
    xkTypeObj(type); 

    return obj->versionCount; 
}

/** 
 * Gets the type version handle at the specified index.
 * 
 * @public                  @memberof kType
 * @param   type            Type.
 * @param   index           Type version index.
 * @return                  Type version handle.
 */
kInlineFx(kTypeVersion) kType_VersionAt(kType type, kSize index)
{
    xkTypeObj(type); 

    kAssert(index < obj->versionCount);

    return &obj->versionInfo[index]; 
}

/** 
 * Gets the GUID associated with the specified type version.
 * 
 * @public              @memberof kType
 * @param   type        Type.
 * @param   version     Type version handle.    
 * @return              GUID.
 */
kInlineFx(const kChar*) kType_VersionGuid(kType type, kPointer version) 
{ 
    return kCast(kTypeVersionInfo*, version)->guid; 
}

/** 
 * Gets the serialization method for the specified type version.
 * 
 * @public              @memberof kType
 * @param   type        Type.
 * @param   version     Type version handle.    
 * @return              Serialization method.
 */
kInlineFx(kFunction) kType_VersionSerializeFx(kType type, kTypeVersion version)
{
    return kCast(kTypeVersionInfo*, version)->serialize;
}

/** 
 * Gets the deserialization method for the specified type version.
 * 
 * Legacy and modern object deserialization methods have different responsibilities 
 * and expected signatures. Legacy object deserialization methods are responsible for performing 
 * object initialization (like "Init" methods). Modern object deserialization methods assume 
 * that initialization has already been performed via calling the registered framework constructor 
 * for the type.
 * 
 * Legacy: kStatus (kCall*)(kObject object, kSerializer serializer, kAlloc allocator)
 * 
 * Modern: kStatus (kCall*)(kObject object, kSerializer serializer)
 * 
 * There are no differences in serialization responsibilities or signatures for legacy vs modern 
 * <em>value</em> deserialization methods. This distinction applies only to object deserialization methods.
 * 
 * The @ref kType_VersionHasLegacyDeserializer method can be used to determine whether a type 
 * version has a legacy or modern deserialization method.
 * 
 * @public              @memberof kType
 * @param   type        Type.
 * @param   version     Type version handle.    
 * @return              Deserialization method.
 */
kInlineFx(kFunction) kType_VersionDeserializeFx(kType type, kTypeVersion version)
{
    return kCast(kTypeVersionInfo*, version)->deserialize;
}

/** 
 * Gets the serialization format name associated with the type version.
 * 
 * @public              @memberof kType
 * @param   type        Type.
 * @param   version     Type version handle.    
 * @return              Serialization format name (e.g., "kdat6"). 
 */
kInlineFx(const kChar*) kType_VersionFormat(kType type, kTypeVersion version)
{
    return kCast(kTypeVersionInfo*, version)->format;
}

/** 
 * Gets the serialization format version associated with the type version.
 * 
 * @public              @memberof kType
 * @param   type        Type.
 * @param   version     Type version handle.    
 * @return              Serialization format version (e.g., 6.0.0.0). 
 */
kInlineFx(kVersion) kType_VersionFormatVersion(kType type, kTypeVersion version)
{
    return kCast(kTypeVersionInfo*, version)->formatVersion;
}

/** 
 * Reports whether the type version has a legacy deserialization method. 
 * 
 * Refer to @ref kType_VersionDeserializeFx for more information on the differences between legacy 
 * and modern deserialization methods. 
 * 
 * @public              @memberof kType
 * @param   type        Type.
 * @param   version     Type version handle.    
 * @return              kTRUE if the type version has a legacy deserializer. 
 */
kInlineFx(kBool) kType_VersionHasLegacyDeserializer(kType type, kTypeVersion version)
{
    return kCast(kTypeVersionInfo*, version)->hasLegacyDeserializer;
}

#endif

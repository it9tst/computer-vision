/** 
 * @file  kType.cpp
 *
 * @internal
 * Copyright (C) 2005-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <kApi/kType.h>
#include <kApi/kApiLib.h>
#include <stdio.h>

kBeginClassEx(k, kType)
    kAddPrivateVMethod(kType, kObject, VRelease)
kEndClassEx()

kFx(kStatus) xkType_Construct(kType* type, kType* reference, const kChar* name, kAssembly assembly, kTypeFlags flags, kSize size, kSize innerSize)
{
    kType output = kNULL; 
    kStatus status; 

    kCheck(xkSysMemAlloc(sizeof(kTypeClass), &output)); 

    if (!kSuccess(status = xkType_Init(output, kNULL, kNULL, reference, name, assembly, flags, size, innerSize)))
    {
        xkSysMemFree(output); 
        return status; 
    }

    *type = output; 
          
    return kOK; 
} 

kFx(kStatus) xkType_Init(kType type, kType typeType, kAlloc allocator, kType* reference, const kChar* name, kAssembly assembly, kTypeFlags flags, kSize size, kSize innerSize)
{
    kObjR(kType, type);
    kStatus status = kOK;

    kCheck(kObject_Init(type, typeType, allocator)); 
    
    /* kType instances are zero-initialized on allocation; most fields do not require explicit initialization. */
    obj->assembly = assembly; 
    obj->selfReference = reference; 
    obj->flags = flags; 
    obj->size = size; 
    obj->innerSize = innerSize; 
    obj->frameworkConstructor = xkType_FrameworkConstructorStub; 
    obj->staticPriority = k16S_MAX; 

    kTry
    {
        if (!kSuccess(status = kStrCopy(obj->name, kCountOf(obj->name), name)))
        {
            kLogf("kType: Type name (%s) too long (capacity=%u).", name, (k32u)kCountOf(obj->name));
            kThrow(status);
        }
    }
    kCatch(&status)
    {
        xkType_VRelease(type);
        kEndCatch(status);
    }
 
    return kOK; 
}

kFx(kStatus) xkType_VRelease(kType type)
{
    kObjR(kType, type);
    kType* reference = obj->selfReference; 
    kSize i; 

    for (i = 0; i < obj->interfaceCount; ++i)
    {            
        kCheck(xkSysMemFree(obj->interfaces[i].iTable)); 
        kCheck(xkSysMemFree(obj->interfaces[i].iMethodInfo)); 
    }

    kCheck(xkSysMemFree(obj->vTable)); 
    kCheck(xkSysMemFree(obj->vMethodInfo)); 
    kCheck(xkSysMemFree(obj->cMethodInfo)); 
    kCheck(xkSysMemFree(obj->fieldInfo)); 
    kCheck(xkSysMemFree(obj->enumeratorInfo)); 
    kCheck(xkSysMemFree(obj->versionInfo)); 
    kCheck(xkSysMemFree(obj->staticData)); 

    kCheck(kObject_VRelease(type)); 

    *reference = kNULL; 
    
    return kOK; 
}

kFx(kStatus) xkType_AddStatic(kType type, kSize staticSize, xkStaticInitFx init, xkStaticReleaseFx release)
{
    kObjR(kType, type);

    kAssert(kIsNull(obj->staticData));

    kCheck(xkSysMemAlloc(staticSize, &obj->staticData));

    obj->staticSize = staticSize; 
    obj->staticInit = init; 
    obj->staticRelease = release; 
    obj->staticInitialized = kFALSE; 
    
    return kOK; 
}

kFx(kStatus) xkType_SetBase(kType type, kType base)
{
    kObjR(kType, type);
    kType baseIt = base; 

    obj->baseCount = 0; 

    while (!kIsNull(baseIt) && (obj->baseCount < xkTYPE_MAX_BASES))
    {        
        obj->bases[obj->baseCount++] = baseIt; 
        baseIt = kType_Base(baseIt);  
    }
   
    return !kIsNull(baseIt) ? kERROR_PARAMETER : kOK; 
}

kFx(kStatus) xkType_InitMethods(kType type)
{
    kObjR(kType, type);
    kType base = kType_Base(type); 
    
    if (!kIsNull(base) && (kType_MethodCount(base) > 0))
    {
        kObjN(kType, baseObj, base); 

        kCheck(xkSysMemReallocList(&obj->cMethodInfo, 0, sizeof(kMethodInfo), &obj->cMethodCapacity, 
            xkTYPE_INITIAL_CMETHOD_CAPACITY, baseObj->cMethodCapacity)); 

        kCheck(kMemCopy(obj->cMethodInfo, baseObj->cMethodInfo, obj->cMethodCount * sizeof(kMethodInfo))); 
    }
    
    return kOK; 
}

kFx(kStatus) xkType_InitVTable(kType type)
{
    kObjR(kType, type);
    kType base = kType_Base(type); 

    if (!kIsNull(base))
    {
        kObjNR(kType, baseObj, base);  

        kSize vMethodCount = baseObj->vMethodCount; 

        kCheck(xkSysMemAlloc(vMethodCount * sizeof(kPointer), &obj->vTable)); 
        kCheck(xkSysMemAlloc(vMethodCount * sizeof(kMethodInfo), &obj->vMethodInfo)); 

        obj->vMethodCount = vMethodCount; 

        kMemCopy(obj->vTable, baseObj->vTable, baseObj->vMethodCount * sizeof(kPointer)); 
        kMemCopy(obj->vMethodInfo, baseObj->vMethodInfo, baseObj->vMethodCount * sizeof(kMethodInfo)); 
    }

    return kOK; 
}

kFx(kStatus) xkType_ReserveVTable(kType type, kSize vMethodCount)
{
    kObjR(kType, type);

    if (vMethodCount > obj->vMethodCount)
    {
        kFunction* vTable = kNULL; 
        kMethodInfo* vMethodInfo = kNULL;
       
        kCheck(xkSysMemAlloc(vMethodCount * sizeof(kPointer), &vTable)); 
        kCheck(xkSysMemAlloc(vMethodCount * sizeof(kMethodInfo), &vMethodInfo)); 

        kMemCopy(vTable, obj->vTable, obj->vMethodCount * sizeof(kPointer)); 
        kMemCopy(vMethodInfo, obj->vMethodInfo, obj->vMethodCount * sizeof(kMethodInfo)); 

        xkSysMemFree(obj->vTable);
        xkSysMemFree(obj->vMethodInfo);

        obj->vTable = vTable; 
        obj->vMethodInfo = vMethodInfo;
        obj->vMethodCount = vMethodCount; 
    }
    
    return kOK; 
}

kFx(kStatus) xkType_InitInterfaces(kType type)
{
    kObjR(kType, type);
    kType base = kType_Base(type); 
    kSize i; 

    if (!kIsNull(base))
    {
        kObjNR(kType, baseObj, base);  

        for (i = 0; i < baseObj->interfaceCount; ++i)
        {   
            kSize iMethodCount = baseObj->interfaces[i].iMethodCount; 

            kCheck(xkSysMemAlloc(iMethodCount * sizeof(kPointer), &obj->interfaces[i].iTable)); 
            kCheck(xkSysMemAlloc(iMethodCount * sizeof(kMethodInfo), &obj->interfaces[i].iMethodInfo)); 

            kMemCopy(obj->interfaces[i].iTable, baseObj->interfaces[i].iTable, iMethodCount * sizeof(kPointer)); 
            kMemCopy(obj->interfaces[i].iMethodInfo, baseObj->interfaces[i].iMethodInfo, iMethodCount * sizeof(kMethodInfo)); 

            obj->interfaces[i].type = baseObj->interfaces[i].type; 
            obj->interfaces[i].iMethodCount = iMethodCount; 

            obj->interfaceCount++; 
        }
    }

    return kOK; 
}

kFx(kStatus) xkType_ImplementInterface(kType type, kType interfaceType, const kChar* interfaceName, kSize iTableSize)
{ 
    kObjR(kType, type);
    kType interfaceIt = interfaceType; 

    if (kIsNull(interfaceType))
    {
        if (kSuccess(xkAssembly_ReorderType(obj->assembly, interfaceName)))
        {
            return kERROR_NOT_FOUND; 
        }
        else
        {
            //Forgot to register the interface type with kAddType?
            kLogf("kType: Forgot to register the interface type (%s; required by %s) with kAddType?", interfaceName, kType_Name(type));
            kCheck(kERROR_STATE); 
        }
    }

    kCheckArgs(!kType_IsInterface(type) && kType_IsInterface(interfaceType) && (iTableSize == kType_VMethodCount(interfaceType) * sizeof(kPointer))); 

    while (interfaceIt != kNULL)
    {        
        kObjNR(kType, interfaceItObj, interfaceIt);  

        //add an interface entry, if one does not already exist
        if (!kType_Implements(type, interfaceIt))
        {
            kSize nextIndex = obj->interfaceCount; 

            kCheckState(nextIndex < xkTYPE_MAX_INTERFACES); 
           
            kCheck(xkSysMemAlloc(interfaceItObj->vMethodCount * sizeof(kPointer), &obj->interfaces[nextIndex].iTable)); 
            kCheck(xkSysMemAlloc(interfaceItObj->vMethodCount * sizeof(kMethodInfo), &obj->interfaces[nextIndex].iMethodInfo)); 

            kCheck(kMemCopy(obj->interfaces[nextIndex].iTable, interfaceItObj->vTable, interfaceItObj->vMethodCount * sizeof(kPointer))); 
            kCheck(kMemCopy(obj->interfaces[nextIndex].iMethodInfo, interfaceItObj->vMethodInfo, interfaceItObj->vMethodCount * sizeof(kMethodInfo))); 

            obj->interfaces[nextIndex].type = interfaceIt; 
            obj->interfaces[nextIndex].iMethodCount = interfaceItObj->vMethodCount;  
            obj->interfaceCount++; 
        }
                    
        interfaceIt = interfaceItObj->bases[0]; 
    }

    return kOK; 
}

kFx(kStatus) xkType_AddVersion(kType type, const kChar* format, const kChar* formatVersion, const kChar* guid, kFunction serialize, kFunction deserialize, kBool hasLegacyDeserializer)
{
    kObjR(kType, type);
    kVersion fmtVer; 
    kStatus status;
    kSize i; 

    kCheck(kVersion_Parse(&fmtVer, formatVersion)); 

    kCheck(xkSysMemReallocList(&obj->versionInfo, obj->versionCount, sizeof(kTypeVersionInfo), 
            &obj->versionCapacity, xkTYPE_INITIAL_VERSION_CAPACITY, obj->versionCapacity+1)); 
    
    //insert sorted by format version
    for (i = 0; i < obj->versionCount; ++i)
    {
        if (kStrEquals(format, obj->versionInfo[i].format) && (fmtVer < obj->versionInfo[i].formatVersion))
        {
            break; 
        }
    }

    kCheck(kMemMove(&obj->versionInfo[i+1], &obj->versionInfo[i], sizeof(kTypeVersionInfo)*(obj->versionCount - i))); 
    
    if (!kSuccess(status = kStrCopy(obj->versionInfo[i].format, kCountOf(obj->versionInfo[i].format), format)))
    {
        kLogf("kType: Version format name (%s) too long (capacity=%u).", format, (k32u)kCountOf(obj->versionInfo[i].format));
        kCheck(status);
    }

    if (!kSuccess(status = kStrCopy(obj->versionInfo[i].guid, kCountOf(obj->versionInfo[i].guid), guid)))
    {
        kLogf("kType: Version format GUID (%s) too long (capacity=%u).", guid, (k32u)kCountOf(obj->versionInfo[i].guid));
        kCheck(status);
    }

    obj->versionInfo[i].formatVersion = fmtVer; 
    obj->versionInfo[i].hasLegacyDeserializer = hasLegacyDeserializer;

    if (kType_IsValue(type))
    {
        obj->versionInfo[i].serialize = kIsNull(serialize) ? (kFunction)xkType_ValueSerializationStub : serialize; 
        obj->versionInfo[i].deserialize = kIsNull(deserialize) ? (kFunction)xkType_ValueDeserializationStub : deserialize; 
    }
    else
    {
        obj->versionInfo[i].serialize = kIsNull(serialize) ? (kFunction)xkType_ObjectSerializationStub : serialize; 
        obj->versionInfo[i].deserialize = kIsNull(deserialize) ? (kFunction)xkType_ObjectDeserializationStub : deserialize; 
    }

    obj->versionCount++; 

    return kOK; 
}

kFx(kStatus) xkType_AddFlags(kType type, kTypeFlags flags)
{
    kObjR(kType, type);

    obj->flags |= flags; 

    return kOK; 
}

kFx(kStatus) xkType_AddFrameworkConstructor(kType type, kFunction constructor)
{
    kObjR(kType, type);
    
    obj->frameworkConstructor = (kFrameworkConstructorFx) constructor; 

    return kOK;
}

kFx(kStatus) xkType_AddMethod(kType type, kFunction function, const kChar* methodName)
{
    kObjR(kType, type);
    kMethodInfo* info = kNULL;
    kStatus status = kOK;

    kCheck(xkSysMemReallocList(&obj->cMethodInfo, obj->cMethodCount, sizeof(kMethodInfo), 
        &obj->cMethodCapacity, xkTYPE_INITIAL_CMETHOD_CAPACITY, obj->cMethodCount+1)); 
    
    info = &obj->cMethodInfo[obj->cMethodCount]; 

    if (!kSuccess(status = kStrPrintf(info->functionName, kCountOf(info->functionName), "%s_%s", obj->name, methodName)))
    {
        kLogf("kType: Method name (%s_%s) too long (capacity=%u).", obj->name, methodName, (k32u)kCountOf(info->functionName));
        kCheck(status);
    }

    //if qualified function name not too long, unqualified function name will be fine
    kCheck(kStrCopy(info->methodName, kCountOf(info->methodName), methodName)); 
    info->function = function; 

    obj->cMethodCount++; 

    return kOK; 
}

kFx(kStatus) xkType_AddVMethod(kType type, kSize index, kFunction function, const kChar* methodName)
{
    kObjR(kType, type);
    kMethodInfo* info = kNULL; 
    kStatus status = kOK;

    kCheck(xkType_ReserveVTable(type, index+1));

    info = &obj->vMethodInfo[index]; 

    obj->vTable[index] = function; 

    if (!kSuccess(status = kStrPrintf(info->functionName, kCountOf(info->functionName), "%s_%s", obj->name, methodName)))
    {
        kLogf("kType: Virtual method name (%s_%s) too long (capacity=%u).", obj->name, methodName, (k32u)kCountOf(info->functionName));
        kCheck(status);
    }
    
    //if qualified function name not too long, unqualified function name will be fine
    kCheck(kStrCopy(info->methodName, kCountOf(info->methodName), &methodName[1]));   //removes the "V" prefix
    info->function = function; 

    return kOK; 
}

kFx(kStatus) xkType_AddIVMethod(kType type, kType interfaceType, kSize index, kFunction function, const kChar* iMethodName, const kChar* cMethodName)
{
    kObjR(kType, type);
    kSize i; 
    kStatus status;

    for (i = 0; i < obj->interfaceCount; ++i)
    {
        if (kType_Is(obj->interfaces[i].type, interfaceType))
        {
            kMethodInfo* info = &obj->interfaces[i].iMethodInfo[index]; 

            kCheckArgs(index < obj->interfaces[i].iMethodCount); 

            obj->interfaces[i].iTable[index] = function; 

            if (!kSuccess(status = kStrPrintf(info->functionName, kCountOf(info->functionName), "%s_%s", obj->name, cMethodName)))
            {
                kLogf("kType: Interface method name (%s_%s) too long (capacity=%u).", obj->name, cMethodName, (k32u)kCountOf(info->functionName));
                kCheck(status);
            }

            if (!kSuccess(status = kStrCopy(info->methodName, kCountOf(info->methodName), iMethodName)))
            {
                kLogf("kType: Interface method name (%s) too long (capacity=%u).", iMethodName, (k32u)kCountOf(info->methodName));
                kCheck(status);
            }
                   
            info->function = function; 

            return kOK; 
        }
    }

    return kERROR_NOT_FOUND; 
}

kFx(kStatus) xkType_AddField(kType type, kType fieldType, const kChar* fieldTypeName, kSize size, kSize offset, kSize count, const kChar* fieldName)
{
    kObjR(kType, type);
    kFieldInfo* info = kNULL; 
    kStatus status = kOK;

    if (kIsNull(fieldType))
    {
        if (kSuccess(xkAssembly_ReorderType(obj->assembly, fieldTypeName)))
        {
            return kERROR_NOT_FOUND; 
        }
        else
        {
            //Forgot to register the field type with kAddType?
            kLogf("kType: Forgot to register field type (%s; required by %s) with kAddType?", fieldTypeName, kType_Name(type));
            kCheck(kERROR_STATE); 
        }
    }

    //This assertion may be raised during type registration if the field type is incorrect. 
    if (kType_Size(fieldType)*count != size)
    {
        kLogf("kType: Size (%u x %u) of registered field %s.%s does not match actual field size (%u).", (k32u)count, (k32u)kType_Size(fieldType), kType_Name(type), fieldName, (k32u)size);
        kCheck(kERROR_STATE); 
    }

    kCheck(xkSysMemReallocList(&obj->fieldInfo, obj->fieldCount, sizeof(kFieldInfo), 
        &obj->fieldCapacity, xkTYPE_INITIAL_FIELD_CAPACITY, obj->fieldCount+1)); 

    info = &obj->fieldInfo[obj->fieldCount]; 

    if (!kSuccess(status = kStrCopy(&info->name[0], kCountOf(info->name), fieldName)))
    {
        kLogf("kType: Field name (%s) too long (capacity=%u).", fieldName, (k32u)kCountOf(info->name));
        kCheck(status);
    }

    info->type = fieldType; 
    info->offset = offset; 
    info->count = count; 

    obj->fieldCount++; 

    return kOK; 
}

kFx(kStatus) xkType_AddEnumerator(kType type, k32s value, const kChar* name)
{
    kObjR(kType, type);
    kEnumeratorInfo* info = kNULL; 
    kStatus status = kOK;
    
    kCheck(xkSysMemReallocList(&obj->enumeratorInfo, obj->enumeratorCount, sizeof(kEnumeratorInfo), 
        &obj->enumeratorCapacity, xkTYPE_INITIAL_ENUMERATOR_CAPACITY, obj->enumeratorCount+1)); 

    info = &obj->enumeratorInfo[obj->enumeratorCount]; 

    if (!kSuccess(status = kStrCopy(&info->name[0], kCountOf(info->name), name)))
    {
        kLogf("kType: Enumerator name (%s) too long (capacity=%u).", name, (k32u)kCountOf(info->name));
        kCheck(status);
    }

    info->value = value; 

    kCheck(xkType_FormatEnumeratorDisplayName(type, info->name, info->displayName, kCountOf(info->displayName))); 

    obj->enumeratorCount++; 

    return kOK; 
}

kFx(kStatus) xkType_FormatEnumeratorDisplayName(kType type, const kChar* enumeratorName, kChar* buffer, kSize capacity)
{
    const kChar* typeNameIt = kType_Name(type); 
    const kChar* enumNameIt = enumeratorName; 
    kChar* bufferIt = buffer; 
    kBool firstLetter = kTRUE; 

    //skip type name prefix
    while ((*typeNameIt != 0) && (*enumNameIt != 0))
    {
        if (*enumNameIt == '_')
        {
            enumNameIt++; 
        }
        else if (kChar_ToLower(*typeNameIt) == kChar_ToLower(*enumNameIt))
        {
            enumNameIt++; 
            typeNameIt++; 
        }
        else
        {
            break;
        }
    }

    //skip leading underscore
    if (*enumNameIt == '_')
    {
        enumNameIt++; 
    }

    //copy out remaining characters
    kCheck(kStrCopy(buffer, capacity, enumNameIt)); 

    //convert from all-caps-with-underscores to first-letter-caps-with-spaces
    while (*bufferIt != 0)
    {
        if (*bufferIt == '_')
        {
            *bufferIt = ' '; 
            firstLetter = kTRUE; 
        }
        else if (!firstLetter)
        {
            *bufferIt = kChar_ToLower(*bufferIt); 
        }
        else
        {
            firstLetter = kFALSE; 
        }

        bufferIt++; 
    }

    return kOK; 
}

kFx(kStatus) xkType_SetInitPriority(kType type, k32u priority)
{
    kObjR(kType, type);

    obj->staticPriority = priority; 

    return kOK; 
}

kFx(k32u) xkType_Priority(kType type)
{
    kObjR(kType, type);
    return obj->staticPriority; 
}

kFx(kStatus) xkType_Prepare(kType type)
{
    kObjR(kType, type);

    if (xkType_DetermineIsPacked(type))
    {
        xkType_AddFlags(type, xkTYPE_FLAGS_PACKED); 
    }

    if (!kIsNull(obj->staticInit))
    {
        kCheck(kMemSet(obj->staticData, 0, obj->staticSize));
    }

    return kOK;
}

kFx(kBool) xkType_DetermineIsPacked(kType type)
{
    kObjR(kType, type);
    kSize totalFieldSize = 0;
    kSize i;

    if (kType_IsPrimitive(type))
    {
        return kTRUE; 
    }
    else if (kType_IsValue(type))
    {
        for (i = 0; i < obj->fieldCount; ++i)
        {
            if (!xkType_DetermineIsPacked(obj->fieldInfo[i].type))
            {
                return kFALSE; 
            }

            totalFieldSize += obj->fieldInfo[i].count * kType_Size(obj->fieldInfo[i].type);
        }

        return (totalFieldSize == kType_Size(type)); 
    }
    else
    {
        return kFALSE; 
    }
}

kFx(kStatus) xkType_RunStaticInit(kType type)
{
    kObjR(kType, type);
    
    if (!kIsNull(obj->staticInit))
    {
        obj->staticInitialized = kSuccess(obj->staticInit()); 

        if (!(obj->staticInitialized))
        {
            kLogf("kType: static initialization failed for type %s.", obj->name); 
        }        
    }

    return kOK; 
}

kFx(kStatus) xkType_RunStaticRelease(kType type)
{
    kObjR(kType, type);

    if (kType_StaticInitialized(type))
    {
        obj->staticInitialized = kFALSE; 

        if (!kSuccess(obj->staticRelease()))
        {
            kLogf("kType: static release failed for type %s.", obj->name); 
        }
    }

    return kOK; 
}


kFx(kStatus) kType_FindMethodInfo(kType type, const kChar* name, const kMethodInfo** info)
{
    xkTypeObj(type); 
    kSize i; 
    
    for (i = 0; i < obj->cMethodCount; ++i)
    {
        if (kStrEquals(obj->cMethodInfo[i].methodName, name))
        {
            *info = &obj->cMethodInfo[i]; 
            return kOK; 
        }
    }

    return kERROR_NOT_FOUND; 
}

kFx(kStatus) kType_FindEnumeratorInfo(kType type, k32s value, const kEnumeratorInfo** info)
{
    xkTypeObj(type); 
    kSize i; 
    
    for (i = 0; i < obj->enumeratorCount; ++i)
    {
        if (obj->enumeratorInfo[i].value == value)
        {
            *info = &obj->enumeratorInfo[i];  
            return kOK; 
        }
    }

    return kERROR_NOT_FOUND; 
}

kFx(kStatus) xkType_FindEnumeratorInfoByName(kType type, const kChar* displayName, const kEnumeratorInfo** info)
{
    xkTypeObj(type); 
    kSize i; 
    
    for (i = 0; i < obj->enumeratorCount; ++i)
    {
        if (kStrEquals(obj->enumeratorInfo[i].displayName, displayName))
        {
            *info = &obj->enumeratorInfo[i];  
            return kOK; 
        }
    }

    return kERROR_NOT_FOUND; 
}

kFx(kStatus) kType_FormatEnumerator(kType type, k32s value, kChar* displayName, kSize capacity)
{  
    const kEnumeratorInfo* info = kNULL; 

    if (!kSuccess(kType_FindEnumeratorInfo(type, value, &info)))
    {
        kCheck(kStrPrintf(displayName, capacity, "[%d]", value)); 
        return kERROR_NOT_FOUND; 
    }

    return kStrCopy(displayName, capacity, info->displayName); 
}

kFx(kStatus) kType_ParseEnumerator(kType type, k32s* value, const kChar* displayName)
{  
    const kEnumeratorInfo* info = kNULL; 

    if (!kSuccess(xkType_FindEnumeratorInfoByName(type, displayName, &info)))
    {
        if (sscanf(displayName, "[%d]", value) != 1)
        {
            *value = 0;
            return kERROR_NOT_FOUND;
        }
    }
    else
    {    
        *value = info->value; 
    }

    return kOK; 
}

kFx(kStatus) xkType_FrameworkConstructorStub(kObject* object, kAlloc allocator)
{
    //Constructor for types that have not registered a framework constructor. 
    return kERROR_UNIMPLEMENTED; 
}

kFx(kStatus) xkType_ObjectSerializationStub(kObject object, kSerializer serializer)
{
    //This assertion is raised when a serializer attempts to write an object of a type for which 
    //a null serialization method was registered.
    kAssert(kFALSE); 
    return kERROR_UNIMPLEMENTED; 
}

kFx(kStatus) xkType_ObjectDeserializationStub(kObject object, kSerializer serializer, kAlloc allocator)
{
    //This assertion is raised when a serializer attempts to read an object of a type for which 
    //a null serialization method was registered.
    kAssert(kFALSE); 
    return kERROR_UNIMPLEMENTED; 
}

kFx(kStatus) xkType_ValueSerializationStub(kType type, const void* values, kSize count, kSerializer serializer)
{
    //This assertion is raised when a serializer attempts to write a value of a type for which 
    //a null serialization method was registered.
    kAssert(kFALSE); 
    return kERROR_UNIMPLEMENTED; 
}

kFx(kStatus) xkType_ValueDeserializationStub(kType type, void* values, kSize count, kSerializer serializer)
{
    //This assertion is raised when a serializer attempts to write a value of a type for which 
    //a null serialization method was registered.
    kAssert(kFALSE); 
    return kERROR_UNIMPLEMENTED; 
}

kFx(kStatus) xkType_LayoutStruct(xkStructField** fields, kSize fieldCount, kSize* structureSize)
{
    kSize fieldMaxPrimitive = 0; 
    kSize maxPrimitive = 0; 
    kSize totalSize = 0; 
    kSize knownFields = 0; 
    kSize i; 

    //sort fields based on increasing offset; dynamic fields (unknown offset) are sorted last
    qsort(fields, fieldCount, sizeof(kPointer), xkType_LayoutStructOffsetComparator); 

    //calculate fields sizes and count the pre-determined fields
    for (i = 0; i < fieldCount; ++i)
    {
        kCheckArgs(!kIsNull(fields[i]->type)); 

        fields[i]->typeSize = kType_Size(fields[i]->type); 
        fields[i]->fieldSize = fields[i]->typeSize * fields[i]->count; 

        if (fields[i]->offset != kSIZE_NULL)
        {
            knownFields++; 
        }
    }

    //sort dynamic fields such that larger fields come first
    qsort(&fields[knownFields], fieldCount-knownFields, sizeof(kPointer), xkType_LayoutStructSizeComparator); 

    //calculate dynamic field offsets and maximum primitive size
    for (i = 0; i < fieldCount; ++i)
    {
        fieldMaxPrimitive = xkType_MaxPrimitiveSize(fields[i]->type); 

        if (fields[i]->offset == kSIZE_NULL)
        {
            fields[i]->offset = kDivideCeilUInt_(totalSize, fieldMaxPrimitive) * fieldMaxPrimitive; 
        }

        totalSize = fields[i]->offset + fields[i]->fieldSize; 
        maxPrimitive = kMax_(maxPrimitive, fieldMaxPrimitive); 
    }
    
    //pad structure for maximum primitive alignment
    if (maxPrimitive > 0)
    {
        totalSize = kDivideCeilUInt_(totalSize, maxPrimitive) * maxPrimitive; 
    }
    
    *structureSize = totalSize; 

    return kOK; 
}

k32s xkType_LayoutStructOffsetComparator(const void* field1, const void* field2)
{
    const xkStructField* f1 = kPointer_ReadAs(field1, xkStructField*); 
    const xkStructField* f2 = kPointer_ReadAs(field2, xkStructField*); 

    if      ((f1->offset == kSIZE_NULL) && (f2->offset != kSIZE_NULL))      return 1;
    else if ((f2->offset == kSIZE_NULL) && (f1->offset != kSIZE_NULL))      return -1; 
    else if ((f1->offset == kSIZE_NULL) && (f2->offset == kSIZE_NULL))      return 0; 
    else                                                                    return (k32s) ((kSSize)f1->offset - (kSSize)f2->offset); 
}

k32s xkType_LayoutStructSizeComparator(const void* field1, const void* field2)
{
    const xkStructField* f1 = kPointer_ReadAs(field1, xkStructField*); 
    const xkStructField* f2 = kPointer_ReadAs(field2, xkStructField*); 

    return (k32s) ((kSSize)f2->fieldSize - (kSSize)f1->fieldSize); 
}

kFx(kSize) xkType_MaxPrimitiveSize(kType type)
{
    xkTypeObj(type); 
    kSize max = 0; 
    kSize count = kType_FieldCount(obj); 
    kSize i; 

    if (count == 0)
    {
        kSize size = kType_Size(type); 
        
        if ((size == 1) || (size == 2) || (size == 4) || (size == 8))
        {
            return size; 
        }
        else
        {
            //struct likely defined without describing its fields; assume the worst. 
            return (1 << kALIGN_ANY); 
        }
    }
    else
    {
        for (i = 0; i < count; ++i)
        {
            kSize fieldMax = xkType_MaxPrimitiveSize(kType_FieldInfoAt(type, i)->type); 

            max = kMax_(max, fieldMax); 
        }
    }

    return max; 
}

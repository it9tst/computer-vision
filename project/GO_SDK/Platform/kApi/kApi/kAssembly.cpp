/** 
 * @file    kAssembly.cpp
 *
 * @internal
 * Copyright (C) 2005-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <kApi/kAssembly.h>
#include <kApi/kApiLib.h>
#include <kApi/Data/kArrayList.h>
#include <kApi/Data/kMap.h>
#include <kApi/Threads/kLock.h>
#include <kApi/Utils/kEvent.h>

kBeginFullClassEx(k, kAssembly)
    kAddPrivateVMethod(kAssembly, kObject, VRelease)
kEndFullClassEx()

kFx(kStatus) xkAssembly_InitStatic()
{
    kStaticObj(kAssembly);

    kCheck(kLock_ConstructEx(&sobj->lock, xkLOCK_OPTION_PRIORITY_INHERITANCE, kNULL)); 

    kCheck(kEvent_Construct(&sobj->onLoad, kNULL)); 
    kCheck(kEvent_Construct(&sobj->onUnload, kNULL)); 
    kCheck(kEvent_Construct(&sobj->onUnloaded, kNULL)); 

    //kAlloc and kAssembly have a mutual static dependency; this line breaks the dependency, 
    //allowing kAlloc to perform part of its initialization before kAssembly and part after
    kCheck(xkAlloc_EndInitStatic()); 

    return kOK;
}

kFx(kStatus) xkAssembly_ReleaseStatic()
{
    kStaticObj(kAssembly);

    kCheck(kDestroyRef(&sobj->lock)); 
   
    kCheck(kDestroyRef(&sobj->onLoad)); 
    kCheck(kDestroyRef(&sobj->onUnload)); 
    kCheck(kDestroyRef(&sobj->onUnloaded)); 

    return kOK;
}

kFx(kStatus) xkAssembly_AddAssembly(kAssembly assembly)
{
    kObj(kAssembly, assembly);
    kStaticObj(kAssembly);

    kCheck(kLock_Enter(sobj->lock));

    kTry
    {
        if (kSuccess(kAssembly_Find(obj->name, kNULL)))
        {
            kLogf("kAssembly: assembly %s already exsists", obj->name);
            kTest(kERROR_ALREADY_EXISTS);
        }

        kTest(xkSysMemReallocList(&sobj->assemblies, sobj->assemblyCount, sizeof(kAssembly), 
            &sobj->assemblyCapacity, xkASSEMBLY_INITIAL_ASSEMBLY_CAPACITY, sobj->assemblyCount+1)); 
                
        sobj->assemblies[sobj->assemblyCount++] = assembly; 

        kTest(kEvent_Notify(sobj->onLoad, assembly, kNULL)); 
    }
    kFinally
    {
        kLock_Exit(sobj->lock); 
        kEndFinally(); 
    }

    return kOK; 
}

kFx(kStatus) xkAssembly_RemoveAssembly(kAssembly assembly)
{
    kStaticObj(kAssembly);
    kSize i; 

    kCheck(kLock_Enter(sobj->lock)); 
    
    kTry
    {
        kTest(kEvent_Notify(sobj->onUnload, assembly, kNULL)); 

        for (i = 0; i < sobj->assemblyCount; ++i)
        {
            if (sobj->assemblies[i] == assembly)
            {
                kTest(kMemMove(&sobj->assemblies[i], &sobj->assemblies[i+1], sizeof(kAssembly)*(sobj->assemblyCount-i-1))); 
                sobj->assemblyCount--; 
                break;
            }
        }

        if (sobj->assemblyCount == 0)
        {
            kTest(xkSysMemFree(sobj->assemblies)); 
            sobj->assemblyCapacity = 0; 
        }
    }
    kFinally
    {
        kLock_Exit(sobj->lock); 
        kEndFinally(); 
    }

    return kOK; 
}

kFx(kStatus) kAssembly_AddLoadHandler(kCallbackFx function, kPointer receiver)
{
    kStaticObj(kAssembly);
    
    kCheck(kLock_Enter(sobj->lock)); 

    kTry
    {
        kTest(kEvent_Add(sobj->onLoad, function, receiver)); 
    }
    kFinally
    {
        kLock_Exit(sobj->lock); 
        kEndFinally(); 
    }

    return kOK; 
}

kFx(kStatus) kAssembly_RemoveLoadHandler(kCallbackFx function, kPointer receiver)
{
    kStaticObj(kAssembly);
    
    kCheck(kLock_Enter(sobj->lock)); 

    kTry
    {
        kTest(kEvent_Remove(sobj->onLoad, function, receiver)); 
    }
    kFinally
    {
        kLock_Exit(sobj->lock); 
        kEndFinally(); 
    }

    return kOK; 
}

kFx(kStatus) kAssembly_AddUnloadHandler(kCallbackFx function, kPointer receiver)
{
    kStaticObj(kAssembly);
    
    kCheck(kLock_Enter(sobj->lock)); 

    kTry
    {
        kTest(kEvent_Add(sobj->onUnload, function, receiver)); 
    }
    kFinally
    {
        kLock_Exit(sobj->lock); 
        kEndFinally(); 
    }

    return kOK; 
}

kFx(kStatus) kAssembly_RemoveUnloadHandler(kCallbackFx function, kPointer receiver)
{
    kStaticObj(kAssembly); 
    
    kCheck(kLock_Enter(sobj->lock)); 

    kTry
    {
        kTest(kEvent_Remove(sobj->onUnload, function, receiver)); 
    }
    kFinally
    {
        kLock_Exit(sobj->lock); 
        kEndFinally(); 
    }

    return kOK; 
}

kFx(kStatus) kAssembly_AddUnloadedHandler(kCallbackFx function, kPointer receiver)
{
    kStaticObj(kAssembly); 
    
    kCheck(kLock_Enter(sobj->lock)); 

    kTry
    {
        kTest(kEvent_Add(sobj->onUnloaded, function, receiver)); 
    }
    kFinally
    {
        kLock_Exit(sobj->lock); 
        kEndFinally(); 
    }

    return kOK; 
}

kFx(kStatus) kAssembly_RemoveUnloadedHandler(kCallbackFx function, kPointer receiver)
{
    kStaticObj(kAssembly); 
    
    kCheck(kLock_Enter(sobj->lock)); 

    kTry
    {
        kTest(kEvent_Remove(sobj->onUnloaded, function, receiver)); 
    }
    kFinally
    {
        kLock_Exit(sobj->lock); 
        kEndFinally(); 
    }

    return kOK; 
}


kFx(kStatus) kAssembly_Enumerate(kArrayList assemblies)
{
    kStaticObj(kAssembly); 
    
    kCheck(kLock_Enter(sobj->lock)); 

    kTry
    {
        kTest(kArrayList_Import(assemblies, sobj->assemblies, kTypeOf(kAssembly), sobj->assemblyCount)); 
        kTest(kShareItems(kTypeOf(kAssembly), kArrayList_DataT(assemblies, kAssembly), kArrayList_Count(assemblies))); 
    }
    kFinally
    {
        kLock_Exit(sobj->lock); 
        kEndFinally(); 
    }

    return kOK; 
}

kFx(kStatus) kAssembly_Find(const kChar* name, kAssembly* assembly)
{
    kStaticObj(kAssembly);
    kBool found = kFALSE;

    kCheck(kLock_Enter(sobj->lock)); 

    kTry
    {
        for (kSize i = 0; i < sobj->assemblyCount; ++i)
        {
            kAssembly assemblyAt = sobj->assemblies[i];

            if (kStrEquals(kAssembly_Name(assemblyAt), name))
            {
                if (!kIsNull(assembly))
                {
                    kShareRef(assembly, assemblyAt);
                }

                found = kTRUE; 
                break;
            }
        }
    }
    kFinally
    {
        kLock_Exit(sobj->lock); 
        kEndFinally(); 
    }

    return (found) ? kOK : kERROR_NOT_FOUND; 
}

kFx(kStatus) xkAssembly_ConstructInternal(kAssembly* assembly, kAssembly* reference, const kChar* name, kVersion version, kVersion platformVersion)
{
    kAssembly output = kNULL;
    kStatus status; 

    kCheck(xkSysMemAlloc(sizeof(kAssemblyClass), &output)); 

    if (!kSuccess(status = xkAssembly_Init(output, kNULL, kNULL, reference, name, version, platformVersion)))
    {
        kCheck(xkSysMemFree(output)); 
        return status; 
    }

    *assembly = output; 
    *reference = output; 

    return kOK; 
}

kFx(kStatus) xkAssembly_Init(kAssembly assembly, kType type, kAlloc allocator, kAssembly* reference, const kChar* name, kVersion version, kVersion platformVersion)
{
    kObjR(kAssembly, assembly); 
    kStatus status; 

    kCheck(kObject_Init(assembly, type, allocator)); 

    kTry
    {
        if (!kSuccess(status = kStrCopy(obj->name, kCountOf(obj->name), name)))
        {
            kLogf("kAssembly: name (%s) too long (capacity=%u).", name, kCountOf(obj->name)); 
            kThrow(status); 
        }

        obj->version = version; 
        obj->platformVersion = platformVersion;
        obj->selfReference = reference;      
    }
    kCatch(&status)
    {
        kObject_VRelease(assembly); 
        kEndCatch(status); 
    }
    
    return kOK; 
}

kFx(kStatus) xkAssembly_VRelease(kAssembly assembly)
{
    kObjR(kAssembly, assembly); 
    kAssembly* reference = obj->selfReference; 
    
    if (obj->isRegistered)
    {
        kCheck(xkAssembly_RemoveAssembly(assembly)); 
    }

    kCheck(kObject_Destroy(obj->typeMap)); 

    kCheck(xkAssembly_ReleaseTypes(assembly)); 
    kCheck(xkAssembly_ReleaseDependencies(assembly));         

    kCheck(kObject_VRelease(assembly)); 

    *reference = kNULL; 
    
    return kOK; 
}

kFx(kStatus) xkAssembly_AddDependency(kAssembly assembly, xkAssemblyConstructFx depend)
{
    kObjR(kAssembly, assembly); 

    kCheck(xkSysMemReallocList(&obj->dependCalls, obj->dependCallCount, sizeof(xkAssemblyConstructFx), 
        &obj->dependCallCapacity, xkASSEMBLY_INITIAL_DEPEND_CAPACITY, obj->dependCallCount+1)); 

    obj->dependCalls[obj->dependCallCount++] = depend; 

    return kOK; 
}

kFx(kStatus) xkAssembly_AddType(kAssembly assembly, xkAssemblyRegisterFx call, const kChar* name)
{
    kObjR(kAssembly, assembly); 

    kCheck(xkSysMemReallocList(&obj->regCalls, obj->regCallCount, sizeof(xkAssemblyRegCallInfo), 
        &obj->regCallCapacity, xkASSEMBLY_INITIAL_TYPE_CAPACITY, obj->regCallCount+1)); 

    obj->regCalls[obj->regCallCount].call = call; 
    kStrCopy(obj->regCalls[obj->regCallCount].name, kCountOf(obj->regCalls[obj->regCallCount].name), name); 
    obj->regCalls[obj->regCallCount].priority = k16S_MAX; 

    obj->regCallCount++; 

    return kOK; 
}

kFx(kStatus) xkAssembly_AddPriority(kAssembly assembly, xkAssemblyRegisterFx call, k32u priority)
{
    kObjR(kAssembly, assembly); 
    kSize i; 

    for (i = 0; i < obj->regCallCount; ++i)
    {
        if (obj->regCalls[i].call == call)
        {
            obj->regCalls[i].priority = priority; 
            return kOK; 
        }
    }

    return kERROR_NOT_FOUND; 
}

kFx(kStatus) xkAssembly_Finalize(kAssembly assembly)
{
    kObjR(kAssembly, assembly); 

    kCheck(xkAssembly_InitDependencies(assembly)); 
    kCheck(xkAssembly_InitTypes(assembly)); 
    kCheck(xkAssembly_MapTypes(assembly)); 

    kCheck(xkAssembly_AddAssembly(assembly)); 

    obj->isRegistered = kTRUE; 

    return kOK; 
}

kFx(kStatus) xkAssembly_InitDependencies(kAssembly assembly)
{
    kObjR(kAssembly, assembly); 
    kSize callCount = obj->dependCallCount; 
    kSize i; 

    kCheck(xkSysMemAlloc(callCount*sizeof(kAssembly), &obj->dependencies));     

    for (i = 0; i < callCount; ++i)
    {
        kCheck(obj->dependCalls[i](&obj->dependencies[i])); 
        obj->dependencyCount++; 
    }

    return kOK; 
}

kFx(kStatus) xkAssembly_ReleaseDependencies(kAssembly assembly)
{
    kObjR(kAssembly, assembly); 
    kSSize count = (kSSize)obj->dependencyCount; 
    kSSize i; 

    for (i = count-1; i >= 0; --i)
    {
        kCheck(kDestroyRef(&obj->dependencies[i])); 
    }

    kCheck(xkSysMemFree(obj->dependencies)); 
    kCheck(xkSysMemFree(obj->dependCalls)); 

    return kOK; 
}

kFx(kStatus) xkAssembly_ReorderType(kAssembly assembly, const kChar* dependentOn)
{
    kObjR(kAssembly, assembly); 
    kSize dependentIndex = 0; 
    
    kCheck(xkAssembly_FindRegCall(assembly, dependentOn, &dependentIndex)); 
    
    if (dependentIndex > obj->nowRegistering)
    {
        xkAssemblyRegCallInfo callInfo = obj->regCalls[obj->nowRegistering]; 

        kMemMove(&obj->regCalls[obj->nowRegistering], &obj->regCalls[obj->nowRegistering+1], 
            (dependentIndex-obj->nowRegistering)*sizeof(callInfo)); 

        obj->regCalls[dependentIndex] = callInfo; 

        obj->reordered = kTRUE; 
    }

    return kOK; 
}

kFx(kStatus) xkAssembly_FindRegCall(kAssembly assembly, const kChar* name, kSize* index)
{
    kObjR(kAssembly, assembly); 
    kSize i; 
    
    for (i = 0; i < obj->regCallCount; ++i)
    {
        if (kStrEquals(obj->regCalls[i].name, name))
        {
            *index = i; 
            return kOK; 
        }
    }

    return kERROR_NOT_FOUND; 
}

kFx(kStatus) xkAssembly_InitTypes(kAssembly assembly)
{
    kObjR(kAssembly, assembly); 
    xkAssemblyRegCallInfo* regCalls = obj->regCalls;
    kSize count = obj->regCallCount; 
    kSize typeCount; 
    kStatus status; 
    kSize i; 

    kCheck(xkSysMemAlloc(count*sizeof(kType), &obj->types));     

    i = 0; 
    while (i < count)
    {
        typeCount = obj->typeCount; 
        obj->nowRegistering = i; 
        obj->reordered = kFALSE; 

        status = regCalls[i].call(assembly, regCalls[i].name);         

        if (!kSuccess(status) && (obj->typeCount > typeCount))
        {
            kCheck(xkType_VRelease(obj->types[typeCount])); 
            kCheck(xkSysMemFree(obj->types[typeCount])); 

            obj->typeCount--; 
        }

        if (!obj->reordered)
        {
            kCheck(status);    
            kCheck(xkType_SetInitPriority(obj->types[typeCount], regCalls[typeCount].priority)); 
            i++; 
        }              
    }

    for (i = 0; i < obj->typeCount; ++i)
    {
        xkObject_CastRaw(obj->types[i])->type = kTypeOf(kType); 
    }

    xkObject_CastRaw(assembly)->type = kTypeOf(kAssembly); 

    kCheck(xkAssembly_SortTypes(assembly));  

    for (i = 0; i < obj->typeCount; ++i)
    {
        kCheck(xkType_Prepare(obj->types[i]));
    }

    for (i = 0; i < obj->typeCount; ++i)
    {
        kCheck(xkType_RunStaticInit(obj->types[i])); 
    }
    
    return kOK; 
}

kFx(kStatus) xkAssembly_ReleaseTypes(kAssembly assembly)
{
    kObjR(kAssembly, assembly); 
    kSSize count = (k32s) obj->typeCount; 
    kSSize i; 

    for (i = count-1; i >= 0; --i)
    {
        kCheck(xkType_RunStaticRelease(obj->types[i])); 
    }
    
    if (!kIsNull(kAssemblyOf(kApiLib)) && !kIsNull(xkTypeVar(kAssembly)) && kType_StaticInitialized(kTypeOf(kAssembly)) && obj->isRegistered)
    {
        kStaticObj(kAssembly); 

        kEvent_Notify(sobj->onUnloaded, assembly, kNULL); 
    }

    for (i = count-1; i >= 0; --i)
    {
        kCheck(xkType_VRelease(obj->types[i])); 
        kCheck(xkSysMemFree(obj->types[i])); 
    }

    kCheck(xkSysMemFree(obj->types)); 
    kCheck(xkSysMemFree(obj->regCalls)); 

    return kOK; 
}

int kAssembly_TypeSortComparator(const void* a, const void* b)
{
    return (k32s)xkType_Priority(*(kType*)a) - (k32s)xkType_Priority(*(kType*)b); 
}

kFx(kStatus) xkAssembly_SortTypes(kAssembly assembly)
{
    kObjR(kAssembly, assembly); 
    
    qsort(obj->types, obj->typeCount, sizeof(kType), kAssembly_TypeSortComparator); 

    return kOK; 
}

kFx(kStatus) xkAssembly_MapTypes(kAssembly assembly)
{
    kObjR(kAssembly, assembly); 
    kStatus status = kOK;

    kCheck(kMap_Construct(&obj->typeMap, kTypeOf(kTypeName), kTypeOf(kType), 0, kNULL)); 

    for (kSize i = 0; i < obj->typeCount; ++i)
    {
        if (!kSuccess(status = kMap_AddT(obj->typeMap, kType_Name(obj->types[i]), &obj->types[i])))
        {
            kLogf("kAssembly: Same type name (%s) registered multiple times?", kType_Name(obj->types[i]));
            kCheck(status); 
        }
    }
    
    return kOK; 
}

kFx(kStatus) xkAssembly_AddValue(kAssembly assembly, kType* type, const kChar* name, kType base, const kChar* baseName, kSize size, kTypeFlags flags)
{      
    kObjR(kAssembly, assembly); 
    kType output = kNULL; 
    kStatus status; 

    if (kIsNull(base) && !kStrEquals(baseName, "kNull"))
    {
        if (kSuccess(xkAssembly_ReorderType(assembly, baseName)))
        {
            return kERROR_NOT_FOUND; 
        }
        else
        {
            kLogf("kAssembly: Forgot to register base (%s) for type (%s) with kAddType?", baseName, name);
            kCheck(kERROR_STATE); 
        }
    }

    kCheck(xkType_Construct(&output, type, name, assembly, flags, size, 0));     

    kTry
    {
        kTest(xkType_SetBase(output, base)); 
        kTest(xkType_InitMethods(output)); 
        kTest(xkType_InitVTable(output)); 
        kTest(xkType_InitInterfaces(output)); 

        obj->types[obj->typeCount++] = output; 
        *type = output; 
    }
    kCatch(&status) 
    {
        xkType_VRelease(output);   
        xkSysMemFree(output); 
        kEndCatch(status); 
    }

    return kOK; 
}

kFx(kStatus) xkAssembly_AddClass(kAssembly assembly, kType* type, const kChar* name, kType base, const kChar* baseName, kSize innerSize)
{
    kObjR(kAssembly, assembly); 
    kType output = kNULL; 
    kStatus status;    

    if (kIsNull(base) && !kStrEquals(baseName, "kNull"))
    {
        if (kSuccess(xkAssembly_ReorderType(assembly, baseName)))
        {
            return kERROR_NOT_FOUND; 
        }
        else
        {
            kLogf("kAssembly: Forgot to register base (%s) for type (%s) with kAddType?", baseName, name);
            kCheck(kERROR_STATE); 
        }
    }

    kCheck(xkType_Construct(&output, type, name, assembly, kTYPE_FLAGS_CLASS, sizeof(kPointer), innerSize));             

    kTry
    {
        kTest(xkType_SetBase(output, base)); 
        kTest(xkType_InitMethods(output)); 
        kTest(xkType_InitVTable(output)); 
        kTest(xkType_InitInterfaces(output)); 
    
        obj->types[obj->typeCount++] = output; 
        *type = output; 
    }
    kCatch(&status); 
    {
        xkType_VRelease(output);   
        xkSysMemFree(output); 
        kEndCatch(status); 
    }

    return kOK; 
}

kFx(kStatus) xkAssembly_AddInterface(kAssembly assembly, kType* type, const kChar* name, kType base, const kChar* baseName)
{      
    kObjR(kAssembly, assembly); 
    kType output = kNULL; 
    kStatus status; 

    if (kIsNull(base) && !kStrEquals(baseName, "kNull"))
    {
        if (kSuccess(xkAssembly_ReorderType(assembly, baseName)))
        {
            return kERROR_NOT_FOUND; 
        }
        else
        {
            kLogf("kAssembly: Forgot to register base interface (%s) for type (%s) with kAddType?", baseName, name);
            kCheck(kERROR_STATE); 
        }
    }

    kCheck(xkType_Construct(&output, type, name, assembly, kTYPE_FLAGS_INTERFACE, sizeof(kPointer), 0)); 

    kTry
    {
        kTest(xkType_SetBase(output, base)); 
        kTest(xkType_InitVTable(output)); 

        obj->types[obj->typeCount++] = output; 
        *type = output; 
    }
    kCatch(&status) 
    {
        xkType_VRelease(output);   
        xkSysMemFree(output); 
        kEndCatch(status); 
    }

    return kOK; 
}

kFx(kStatus) kAssembly_FindType(kAssembly assembly, const kChar* name, kType* type)
{
    if (!kIsNull(assembly))
    {
        kObj(kAssembly, assembly);
    
        return kMap_FindT(obj->typeMap, name, type); 
    }
    else
    {
        kArrayList assemblies = kNULL; 
        kBool found = kFALSE;
        kSize i = 0;

        kTry
        {
            kTest(kArrayList_Construct(&assemblies, kTypeOf(kAssembly), 0, kAlloc_App()));

            kTest(kAssembly_Enumerate(assemblies)); 

            while (!found && (i < kArrayList_Count(assemblies)))
            {
                kAssembly assemblyAt = kArrayList_AsT(assemblies, i++, kAssembly); 

                found = kSuccess(kAssembly_FindType(assemblyAt, name, type)); 
            }
        }
        kFinally
        {
            kDisposeRef(&assemblies);

            kEndFinally();
        }        

        return (found) ? kOK : kERROR_NOT_FOUND;
    }
}

kFx(kStatus) xkAssembly_FindDerivedTypesInAssembly(kAssembly assembly, kType base, kArrayList types)
{
    kObj(kAssembly, assembly);
    kSize i; 

    for (i = 0; i < kAssembly_TypeCount(assembly); ++i)
    {
        kType type = kAssembly_TypeAt(assembly, i); 

        if (kType_Is(type, base))
        {
            kCheck(kArrayList_Add(types, &type));
        }
    }

    return kOK; 
}

kFx(kStatus) kAssembly_FindDerivedTypes(kAssembly assembly, kType base, kArrayList types)
{
    kCheck(kArrayList_Allocate(types, kTypeOf(kType), 0)); 

    if (!kIsNull(assembly))
    {
        return xkAssembly_FindDerivedTypesInAssembly(assembly, base, types); 
    }
    else
    {
        kArrayList assemblies = kNULL; 
        kSize i; 

        kTry
        {
            kTest(kArrayList_Construct(&assemblies, kTypeOf(kAssembly), 0, kObject_Alloc(types)));

            kTest(kAssembly_Enumerate(assemblies)); 

            for (i = 0; i < kArrayList_Count(assemblies); ++i)
            {
                kAssembly assembly = kArrayList_AsT(assemblies, i, kAssembly); 

                kTest(xkAssembly_FindDerivedTypesInAssembly(assembly, base, types)); 
            }
        }
        kFinally
        {
            kDisposeRef(&assemblies);

            kEndFinally();
        }
    }

    return kOK;
}

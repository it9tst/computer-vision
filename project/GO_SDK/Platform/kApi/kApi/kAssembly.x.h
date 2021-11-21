/** 
 * @file    kAssembly.x.h
 *
 * @internal
 * Copyright (C) 2005-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_ASSEMBLY_X_H
#define K_API_ASSEMBLY_X_H

#define xkASSEMBLY_GROWTH_FACTOR                     (2)
#define xkASSEMBLY_INITIAL_ASSEMBLY_CAPACITY         (32)
#define xkASSEMBLY_INITIAL_DEPEND_CAPACITY           (32)
#define xkASSEMBLY_INITIAL_TYPE_CAPACITY             (256)

typedef kStatus (kCall* xkAssemblyConstructFx)(kAssembly* assembly); 
typedef kStatus (kCall* xkAssemblyRegisterFx)(kAssembly assembly, const kChar* typeName); 

typedef struct xkAssemblyRegCallInfo
{
    xkAssemblyRegisterFx call; 
    kTypeName name; 
    k32u priority; 
} xkAssemblyRegCallInfo; 

typedef struct kAssemblyClass
{
    kObjectClass base; 
    kText64 name; 
    kVersion version; 
    kVersion platformVersion; 
    kAssembly* selfReference;
    kBool isRegistered; 
    
    xkAssemblyConstructFx* dependCalls;
    kSize dependCallCount; 
    kSize dependCallCapacity; 

    xkAssemblyRegCallInfo* regCalls;
    kSize regCallCount; 
    kSize regCallCapacity;
    kBool reordered; 
    kSize nowRegistering; 

    kAssembly* dependencies; 
    kSize dependencyCount; 
    kType* types; 
    kSize typeCount; 
    kMap typeMap; 
} kAssemblyClass;

typedef struct kAssemblyVTable
{
    kObjectVTable base; 
} kAssemblyVTable; 

typedef struct kAssemblyStatic
{
    kLock lock; 
    kAssembly* assemblies; 
    kSize assemblyCount; 
    kSize assemblyCapacity; 
    kEvent onLoad; 
    kEvent onUnload; 
    kEvent onUnloaded; 
} kAssemblyStatic; 

kDeclareFullClassEx(k, kAssembly, kObject) 

/* 
* Private methods. 
*/

kFx(kStatus) xkAssembly_InitStatic(); 
kFx(kStatus) xkAssembly_ReleaseStatic(); 

kFx(kStatus) xkAssembly_Init(kAssembly assembly, kType type, kAlloc allocator, kAssembly* reference, const kChar* name, kVersion version, kVersion platformVersion); 
kFx(kStatus) xkAssembly_VRelease(kAssembly assembly); 

kFx(kStatus) xkAssembly_AddAssembly(kAssembly assembly); 
kFx(kStatus) xkAssembly_RemoveAssembly(kAssembly assembly); 
kFx(kStatus) xkAssembly_ConstructInternal(kAssembly* assembly, kAssembly* reference, const kChar* name, kVersion version, kVersion platformVersion);

kFx(kStatus) xkAssembly_AddDependency(kAssembly assembly, xkAssemblyConstructFx depend); 
kFx(kStatus) xkAssembly_AddType(kAssembly assembly, xkAssemblyRegisterFx call, const kChar* name); 
kFx(kStatus) xkAssembly_AddPriority(kAssembly assembly, xkAssemblyRegisterFx call, k32u priority); 

kFx(kStatus) xkAssembly_Finalize(kAssembly assembly); 

kFx(kStatus) xkAssembly_InitDependencies(kAssembly assembly); 
kFx(kStatus) xkAssembly_ReleaseDependencies(kAssembly assembly); 

kFx(kStatus) xkAssembly_InitTypes(kAssembly assembly); 
kFx(kStatus) xkAssembly_ReleaseTypes(kAssembly assembly); 

kFx(kStatus) xkAssembly_SortTypes(kAssembly assembly); 
kFx(kStatus) xkAssembly_MapTypes(kAssembly assembly); 

kFx(kStatus) xkAssembly_AddValue(kAssembly assembly, kType* type, const kChar* name, kType base, const kChar* baseName, kSize size, kTypeFlags flags); 
kFx(kStatus) xkAssembly_AddClass(kAssembly assembly, kType* type, const kChar* name, kType base, const kChar* baseName, kSize innerSize); 
kFx(kStatus) xkAssembly_AddInterface(kAssembly assembly, kType* type, const kChar* name, kType base, const kChar* baseName); 

kFx(kStatus) xkAssembly_ReorderType(kAssembly assembly, const kChar* dependentOn); 
kFx(kStatus) xkAssembly_FindRegCall(kAssembly assembly, const kChar* name, kSize* index); 

kFx(kStatus) xkAssembly_FindDerivedTypesInAssembly(kAssembly assembly, kType base, kArrayList types);

#endif

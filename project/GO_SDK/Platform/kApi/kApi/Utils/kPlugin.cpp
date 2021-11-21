/** 
 * @file    kPlugin.cpp
 *
 * @internal
 * Copyright (C) 2014-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <kApi/Utils/kPlugin.h>
#include <kApi/Utils/kDynamicLib.h>
#include <kApi/Io/kPath.h>

kBeginClassEx(k, kPlugin)
    kAddPrivateVMethod(kPlugin, kObject, VRelease)
kEndClassEx()

kFx(kStatus) kPlugin_Construct(kPlugin* plugin, const kChar* path, kAlloc allocator)
{
    return kPlugin_ConstructEx(plugin, path, 0, kNULL, 0, kNULL, allocator);
}

kFx(kStatus) kPlugin_ConstructEx(kPlugin* plugin, const kChar* path, kVersion requiredPlatformVersion, kVersion* platformVersion, 
                                    kVersion requiredAssemblyVersion, kVersion* assemblyVersion, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback(allocator);
    kStatus status; 

    kCheck(kAlloc_GetObject(alloc, kTypeOf(kPlugin), plugin));

    if (!kSuccess(status = xkPlugin_Init(*plugin, kTypeOf(kPlugin), path, requiredPlatformVersion, platformVersion, requiredAssemblyVersion, assemblyVersion, alloc)))
    {
        kAlloc_FreeRef(alloc, plugin); 
    }

    return status; 
} 

kFx(kStatus) xkPlugin_CheckVersion(kPlugin plugin, const kChar* path, const kChar* kind, kVersion required, const kChar* versionFxName, kVersion* actual)
{
    kObjR(kPlugin, plugin);  
    xkPluginVersionFx versionFx = kNULL;
    kVersion parsed = 0; 
    kSize i; 

    if ((required != k32U_MIN) && (required != k32U_MAX))
    {
        if (!kSuccess(kDynamicLib_FindFunction(obj->library, versionFxName, (kFunction*)&versionFx)))
        {
            kLogf("kPlugin: Could not find %s version function in dynamic library (%s).", kind, path); 
            return kERROR_FORMAT; 
        }

        if (!kSuccess(kVersion_Parse(&parsed, versionFx())))
        {
            kLogf("kPlugin: Could not parse %s version in dynamic library (%s).", kind, path); 
            return kERROR_VERSION; 
        }
        
        for (i = 0; i < 4; ++i)  //major, minor, release, build
        {
            k32u requiredDigit = (required >> 8*i) & 0xFF;  
            k32u parsedDigit = (parsed >> 8*i) & 0xFF; 
            
            if (requiredDigit != k8U_MAX)
            {
                if (requiredDigit != parsedDigit)
                {
                    kText32 requiredStr; 
                    kText32 parsedStr; 

                    kCheck(kVersion_Format(required, requiredStr, sizeof(requiredStr))); 
                    kCheck(kVersion_Format(parsed, parsedStr, sizeof(parsedStr))); 

                    kLogf("kPlugin: %s version mismatch (required=%s, actual=%s) in dynamic library (%s).", kind, requiredStr, parsedStr, path); 
                    return kERROR_VERSION; 
                }
            }
        }
    }

    if (!kIsNull(actual))
    {
        *actual = parsed;
    }

    return kOK; 
}

kFx(kStatus) xkPlugin_Init(kPlugin plugin, kType type, const kChar* path, kVersion requiredPlatformVersion, kVersion* platformVersion, 
                                    kVersion requiredAssemblyVersion, kVersion* assemblyVersion, kAlloc allocator)
{
    kObjR(kPlugin, plugin);  
    xkPluginConstructFx constructFx = kNULL;
    kStatus exception = kOK;
    kStatus opStatus = kOK;

    if (!kIsNull(platformVersion))  *platformVersion = 0; 
    if (!kIsNull(assemblyVersion))  *assemblyVersion = 0; 

    kCheck(kObject_Init(plugin, type, allocator)); 

    obj->library = kNULL;
    obj->assembly = kNULL;
    obj->handle = kNULL;

    kTry
    {
        if (!kSuccess(opStatus = kDynamicLib_Construct(&obj->library, path, allocator)))
        {
            kLogf("kPlugin: Load failed (%s) for dynamic library (%s).", kStatus_Name(opStatus), path); 
            kThrow(opStatus);
        }

        kTest(xkPlugin_CheckVersion(plugin, path, "Platform", requiredPlatformVersion, xkPLUGIN_PLATFORM_VERSION_FX, platformVersion));
        kTest(xkPlugin_CheckVersion(plugin, path, "Assembly", requiredAssemblyVersion, xkPLUGIN_ASSEMBLY_VERSION_FX, assemblyVersion));

        if (!kSuccess(kDynamicLib_FindFunction(obj->library, xkPLUGIN_CONSTRUCT_FX, (kFunction*)&constructFx)))
        {
            kLogf("kPlugin: Assembly constructor could not be located in dynamic library (%s).", path); 
            kThrow(kERROR_FORMAT);
        }

        if (!kSuccess(opStatus = constructFx(&obj->assembly)))
        {
            kLogf("kPlugin: Assembly construction failed (code=%s) for plugin (%s).", kStatus_Name(opStatus), path); 
            kThrow(kERROR_CONFLICT);
        }
    }
    kCatch(&exception)
    {
        xkPlugin_VRelease(plugin); 
        kEndCatch(exception);
    }

    return kOK; 
}

kFx(kStatus) xkPlugin_VRelease(kPlugin plugin)
{
    kObj(kPlugin, plugin); 

    kCheck(kObject_Destroy(obj->assembly)); 
    
    kCheck(kObject_Destroy(obj->library)); 
   
    kCheck(kObject_VRelease(plugin)); 

    return kOK;
}

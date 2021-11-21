/** 
 * @file    kPlugin.x.h
 *
 * @internal
 * Copyright (C) 2014-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_PLUGIN_X_H
#define K_API_PLUGIN_X_H

kDeclareClassEx(k, kPlugin, kObject)

typedef kStatus (kDlCall* xkPluginConstructFx)(kAssembly* assembly);
typedef const kChar* (kDlCall* xkPluginVersionFx)();

#define xkPLUGIN_CONSTRUCT_FX            "kPlugin_ConstructAssembly"
#define xkPLUGIN_PLATFORM_VERSION_FX     "kPlugin_PlatformVersion"
#define xkPLUGIN_ASSEMBLY_VERSION_FX     "kPlugin_AssemblyVersion"

typedef struct kPluginClass
{
    kObjectClass base;
    kDynamicLib library;        // Library handle.
    kAssembly assembly;         // Assembly handle.
    kPointer handle;
} kPluginClass;

/* 
* Private methods. 
*/

kFx(kStatus) xkPlugin_Init(kPlugin plugin, kType type, const kChar* path, kVersion requiredPlatformVersion, kVersion* platformVersion, 
                                    kVersion requiredAssemblyVersion, kVersion* assemblyVersion, kAlloc allocator);
kFx(kStatus) xkPlugin_VRelease(kPlugin plugin); 

kFx(kStatus) xkPlugin_CheckVersion(kPlugin plugin, const kChar* path, const kChar* kind, kVersion required, const kChar* versionFxName, kVersion* actual);

#endif

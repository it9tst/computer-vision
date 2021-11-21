/** 
 * @file    kPlugin.h
 * @brief   Declares the kPlugin class. 
 *
 * @internal
 * Copyright (C) 2014-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_PLUGIN_H
#define K_API_PLUGIN_H

#include <kApi/kApiDef.h>
#include <kApi/Utils/kPlugin.x.h>

/**
 * @class   kPlugin
 * @extends kObject
 * @ingroup kApi-Utils
 * @brief   Represents a dynamically loaded assembly. 
 * 
 * This class relies on OS support for dynamic loading, and is only available on platforms that 
 * provide underlying support. A module version protection system is provided to prevent incompatible
 * modules from being loaded and executed.
 */
//typedef kObject kPlugin;             --forward-declared in kApiDef.x.h 

/** 
 * Constructs a kPlugin object.
 *
 * This constructor ignores the assembly platform version when loading the plugin.
 * 
 * @public              @memberof kPlugin
 * @param   plugin      Destination for the constructed object handle. 
 * @param   path        Path to the plugin library. 
 * @param   allocator   Memory allocator (or kNULL for default). 
 * @return              Operation status. 
 */
kFx(kStatus) kPlugin_Construct(kPlugin* plugin, const kChar* path, kAlloc allocator);

/** 
 * Constructs a kPlugin object, optionally performing version checks to ensure compatibility.
 * 
 * The requiredPlatformVersion argument is checked against the platform version that the 
 * plugin was built against. Any digits of the requiredPlatformVersion field that are set 
 * to 255 will not be checked. If all digits are 255 (or all zero), no check will be 
 * performed. Assuming the library can be loaded and queried for its version, the platformVersion
 * argument will receive the actual platform version that the plugin was built against. This 
 * value can be used in error reporting logic if kERROR_VERSION is returned by this function.
 * Note that platform builds are only expected to be binary-compatible after they 
 * have achieved release status, and only if the major, minor, and release digits all match. 
 * Accordingly, when performing platform version checks, it is strongly recommended to check at 
 * least the major, minor, and release version digits. 
 * 
 * Similarly, the requiredAssemblyVersion argument is checked against the version of the 
 * plugin assembly. Any digits of the requiredAssemblyVersion field that are set 
 * to 255 will not be checked. If all digits are 255 (or all zero), no check will be 
 * performed. Assuming the library can be loaded and queried for its version, the assemblyVersion
 * argument will receive the actual plugin assembly version. This value can be used in error 
 * reporting logic if kERROR_VERSION is returned by this function. Assembly version checking 
 * requires that plugins are built with at least platform version 6.2.6.28; if a required assembly 
 * version is specified and the plugin was built against an older platform version, loading 
 * will fail.
 *
 * The following error codes will be returned in the event of specific failures: 
 * -kERROR_NOT_FOUND: Dynamic library missing or otherwise unloadable.
 * -kERROR_INCOMPLETE: Dynamic load failed due to unresolved symbols.
 * -kERROR_FORMAT: Required plugin method(s) not available. 
 * -kERROR_VERSION: Version mismatch (platform or assembly).
 * -kERROR_CONFLICT: Assembly construction failure.
 * 
 * @public                          @memberof kPlugin
 * @param   plugin                  Destination for the constructed object handle. 
 * @param   path                    Path to the plugin library. 
 * @param   requiredPlatformVersion Required platform version (all-digits-zero to fully disable, or 255 per digit to disable individual digits). 
 * @param   platformVersion         Actual platform version (optional, can be kNULL; receives 0.0.0.0 if unknown). 
 * @param   requiredAssemblyVersion Required plugin assembly version (all-digits-zero to fully disable, or 255 per digit to disable individual digits). 
 * @param   assemblyVersion         Actual assembly version (optional, can be kNULL; receives 0.0.0.0 if unknown).  
 * @param   allocator               Memory allocator (or kNULL for default). 
 * @return                          Operation status (see description of error codes).  
 */
kFx(kStatus) kPlugin_ConstructEx(kPlugin* plugin, const kChar* path, kVersion requiredPlatformVersion, kVersion* platformVersion, 
                                    kVersion requiredAssemblyVersion, kVersion* assemblyVersion, kAlloc allocator); 

/** 
 * Gets the assembly associated with this plugin.
 *
 * @public              @memberof kPlugin
 * @param   plugin      Plugin object.
 * @return              Assembly object. 
 */
kInlineFx(kAssembly) kPlugin_Assembly(kPlugin plugin)
{
    kObj(kPlugin, plugin);

    return obj->assembly; 
}

/** 
 * Gets the library associated with this plugin.
 *
 * @public              @memberof kPlugin
 * @param   plugin      Plugin object.
 * @return              Library object. 
 */
kInlineFx(kDynamicLib) kPlugin_Library(kPlugin plugin)
{
    kObj(kPlugin, plugin);

    return obj->library; 
}

#endif

/** 
 * @file    kPath.x.h
 *
 * @internal
 * Copyright (C) 2008-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_PATH_X_H
#define K_API_PATH_X_H

#define xkPATH_MAX                  (512)             
#define xkPATH_SEPARATOR             '/'                ///< Normal path separator. 
#define xkPATH_ALT_SEPARATOR         '\\'               ///< Alternative path separator. 

#if defined(K_WINDOWS)
#   define xkPATH_DEFAULT_NATIVE_SEPARATOR     '\\'      ///< Default path separator used by underlying OS.
#   define xkPATH_VOLUME_SEPARATOR             ':'       ///< Volume separator (Windows only, e.g. "c:"). 
#   define xkPATH_LIBRARY_EXTENSION            "dll"     ///< Assumed file extension for dynamic libraries.
#   define xkPATH_EXECUTABLE_EXTENSION         ".exe"    ///< Assumed file extension for executables.
#elif defined(K_VX_KERNEL)
#   define xkPATH_DEFAULT_NATIVE_SEPARATOR     '/'
#   define xkPATH_VOLUME_SEPARATOR              0
#   define xkPATH_LIBRARY_EXTENSION            "kext"
#   define xkPATH_EXECUTABLE_EXTENSION         ""
#else
#   define xkPATH_DEFAULT_NATIVE_SEPARATOR     '/' 
#   define xkPATH_VOLUME_SEPARATOR              0
#   define xkPATH_LIBRARY_EXTENSION            "so"  
#   define xkPATH_EXECUTABLE_EXTENSION         "" 
#endif

#if defined(K_WINDOWS)
    kInlineFx(kBool) xkPath_CompareChar(kChar a, kChar b)  { return kChar_ToLower(a) == kChar_ToLower(b); }
#else
    kInlineFx(kBool) xkPath_CompareChar(kChar a, kChar b)  { return a == b; }
#endif

typedef struct kPathStatic
{
    k32u placeholder;       //unused
} kPathStatic; 

kDeclareStaticClassEx(k, kPath)


/* 
* Private methods. 
*/

kFx(kStatus) xkPath_SetDefaultHandlers();
kFx(kStatus) xkPath_InitStatic();
kFx(kStatus) xkPath_ReleaseStatic();

/** 
 * Returns the alternative path separator character.
 *
 * @public              @memberof kPath
 * @return              Alternative path separator character.
 */
kFx(kChar) xkPath_AltSeparator();

/** 
 * Returns the native path separator character.
 *
 * @public              @memberof kPath
 * @return              Native path separator character.
 */
kFx(kChar) xkPath_NativeSeparator();

/** 
 * Returns the volume separator character, if supported. 
 * 
 * Volume separators are used on Windows (i.e., the ":" in "c:\"). 
 *
 * @public              @memberof kPath
 * @return              Volume separator character. 
 */
kFx(kChar) xkPath_VolumeSeparator();

/** 
 * Utility method that finds common length between two paths. 
 *
 * @public              @memberof kPath
 * @param  pathA        First path. 
 * @param  pathB        Second path. 
 * @return              Common path length, in characters.
 */
kFx(kSize) xkPath_CommonLength(const kChar* pathA, const kChar* pathB); 

/** 
 * Utility method that removes extraneous trailing separators. 
 *
 * @public              @memberof kPath
 * @param  path         Path to be modified.  
 * @return              Common path length, in characters.
 */
kFx(kStatus) xkPath_RemoveTrailingSeparator(kChar* path); 

/** 
 * Converts a virtual path to a real path. 
 *
 * Virtual paths are used to simulate a file system structure that is different from the 
 * structure of the actual underlying file system. This function uses the toVirtual member of 
 * kApiLib_DirectoryHandlers() to convert a virtual path to a real path.
 *
 * @public              @memberof kPath
 * @param  vPath        Virtual path. 
 * @param  path         Receives real path.
 * @param  capacity     Capacity of path argument. 
 * @return              Operation status. 
 */
kFx(kStatus) xkPath_FromVirtual(const kChar* vPath, kChar* path, kSize capacity); 

/** 
 * Converts a real path to a virtual path. 
 *
 * Virtual paths are used to simulate a file system structure that is different from the 
 * structure of the actual underlying file system. This function uses the fromVirtual member of 
 * kApiLib_DirectoryHandlers() to convert a real path to a virtual path.
 *
 * @public              @memberof kPath
 * @param  path         Real path.
 * @param  vPath        Receives virtual path. 
 * @param  capacity     Capacity of path argument. 
 * @return              Operation status. 
 */
kFx(kStatus) xkPath_ToVirtual(const kChar* path, kChar* vPath, kSize capacity); 

#if defined(K_PLATFORM)
#   if defined(K_WINDOWS)

        kFx(kStatus) xkPath_NormalizedToNativeWideWin(const kChar* path, WCHAR* wpath, kSize capacity); 
        kFx(kStatus) xkPath_NativeWideToNormalizedWin(const WCHAR* wpath, kChar* path, kSize capacity); 

#   endif
#endif

#endif

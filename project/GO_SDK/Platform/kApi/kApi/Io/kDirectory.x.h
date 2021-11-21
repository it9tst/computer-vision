/** 
 * @file    kDirectory.x.h
 *
 * @internal
 * Copyright (C) 2008-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_DIRECTORY_X_H
#define K_API_DIRECTORY_X_H

#include <kApi/Data/kArrayList.h>
#include <kApi/Data/kString.h>

#define xkDIRECTORY_DEFAULT_DATA_FOLDER          "data"
#define xkDIRECTORY_DEFAULT_CONFIG_FOLDER        "res"

#define xkDIRECTORY_WINDOWS_IO_ATTEMPTS    (50)              // Let virus scanner or indexer do whatever they do and try up to this time again
#define xkDIRECTORY_WINDOWS_ACCESS_DENIED  (0x5)

typedef struct kDirectoryStatic
{
    k32u placeholder;       //unused
} kDirectoryStatic; 

kDeclareStaticClassEx(k, kDirectory)

/* 
* Forward declarations. 
*/

kFx(kStatus) kDirectory_ListFiles(const kChar* directory, kArrayList files); 
kFx(kStatus) kDirectory_ListDirectories(const kChar* directory, kArrayList directories); 
kFx(kStatus) kDirectory_ListEntries(const kChar* directory, kArrayList entries); 

/* 
* Private methods. 
*/

kFx(kStatus) xkDirectory_SetDefaultHandlers();
kFx(kStatus) xkDirectory_InitStatic();
kFx(kStatus) xkDirectory_ReleaseStatic();

kFx(kStatus) xkDirectory_CreateImpl(const kChar* directory); 
kFx(kBool) xkDirectory_ExistsImpl(const kChar* directory);
kFx(kStatus) xkDirectory_MoveImpl(const kChar* source, const kChar* destination); 
kFx(kStatus) xkDirectory_DeleteImpl(const kChar* directory);
kFx(kStatus) xkDirectory_EnumerateImpl(const kChar* directory, kBool includeFile, kBool includeDirectories, kArrayList entries); 
kFx(kStatus) xkDirectory_SetCurrentImpl(const kChar* directory);
kFx(kStatus) xkDirectory_CurrentImpl(kChar* directory, kSize capacity); 
kFx(kStatus) xkDirectory_ApplicationImpl(kChar* directory, kSize capacity);
kFx(kStatus) xkDirectory_ApiImpl(kChar* directory, kSize capacity);
kFx(kStatus) xkDirectory_TempImpl(kChar* directory, kSize capacity);
kFx(kStatus) xkDirectory_AppConfigImpl(const kChar* appName, kChar* directory, kSize capacity);
kFx(kStatus) xkDirectory_AppDataImpl(const kChar* appName, kChar* directory, kSize capacity);
kFx(kStatus) xkDirectory_FromVirtualImpl(const kChar* vPath, kChar* path, kSize capacity);
kFx(kStatus) xkDirectory_ToVirtualImpl(const kChar* path, kChar* vPath, kSize capacity);

kFx(kStatus) kDirectory_Api(kChar* directory, kSize capacity);

/* 
* Deprecated (Stage 1): not recommended for further use, but not yet announced via kDeprecate
*/

//[Deprecated] Convert to kDirectory_ListFiles, kDirectory_ListDirectories, or kDirectory_ListEntries
kFx(kStatus) kDirectory_Enumerate(const kChar* directory, kBool includeFiles, kBool includeDirectories, kArrayList entries); 

#endif

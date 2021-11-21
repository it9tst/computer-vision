/** 
 * @file    kDirectory.cpp
 *
 * @internal
 * Copyright (C) 2008-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#define K_PLATFORM
#include <kApi/Io/kDirectory.h>
#include <kApi/Io/kFile.h>
#include <kApi/Io/kPath.h>
#include <kApi/Data/kArrayList.h>
#include <kApi/Data/kString.h>
#include <kApi/Threads/kThread.h>
#include <stdio.h>

kBeginStaticClassEx(k, kDirectory)
kEndStaticClassEx()

kFx(kStatus) xkDirectory_SetDefaultHandlers()
{
    kApiDirectoryFx handlers = { kNULL }; 

    handlers.create = xkDirectory_CreateImpl; 
    handlers.exists = xkDirectory_ExistsImpl; 
    handlers.move= xkDirectory_MoveImpl; 
    handlers.del= xkDirectory_DeleteImpl; 
    handlers.enumerate = xkDirectory_EnumerateImpl; 
    handlers.setCurrent = xkDirectory_SetCurrentImpl; 
    handlers.current = xkDirectory_CurrentImpl; 
    handlers.appDirectory = xkDirectory_ApplicationImpl; 
    handlers.apiDirectory = xkDirectory_ApiImpl; 
    handlers.tempDirectory = xkDirectory_TempImpl; 
    handlers.appConfigDirectory = xkDirectory_AppConfigImpl; 
    handlers.appDataDirectory = xkDirectory_AppDataImpl; 
    handlers.toVirtual = xkDirectory_ToVirtualImpl; 
    handlers.fromVirtual = xkDirectory_FromVirtualImpl; 

    //respect handlers that have already been installed    
    if (kApiLib_HasDirectoryHandlers())
    {
        kCheck(xkOverrideFunctions(&handlers, sizeof(handlers), kApiLib_DirectoryHandlers())); 
    }

    kCheck(kApiLib_SetDirectoryHandlers(&handlers)); 

    return kOK;
}

kFx(kStatus) xkDirectory_InitStatic()
{
    return xkDirectory_SetDefaultHandlers();
}

kFx(kStatus) xkDirectory_ReleaseStatic()
{
    return kOK; 
}

kFx(kStatus) kDirectory_Create(const kChar* directory)
{
    kChar parent[kPATH_MAX]; 
    
    //create any missing parent directories
    while (kSuccess(kPath_Directory(directory, parent, kCountOf(parent))) && !kDirectory_Exists(parent))
    {
        kCheck(kDirectory_Create(parent)); 
    }

    //create the requested directory        
    return kApiLib_DirectoryHandlers()->create(directory); 
}

kFx(kBool) kDirectory_Exists(const kChar* directory)
{    
    return kApiLib_DirectoryHandlers()->exists(directory); 
}

kFx(kStatus) kDirectory_Copy(const kChar* source, const kChar* destination)
{
    kSize i;
    kArrayList files = kNULL; 
    kArrayList directories = kNULL; 
    kString srcItem = kNULL; 
    kChar srcPath[kPATH_MAX]; 
    kChar destPath[kPATH_MAX]; 

    if (kStrEquals(source, destination))
    {
        return kOK; 
    }

    kCheckArgs(kDirectory_Exists(source)); 

    //create destination root folder
    kCheck(kDirectory_Create(destination)); 

    kTry
    {
        //copy files
        kTest(kArrayList_Construct(&files, kNULL, 0, kNULL)); 
        kTest(kDirectory_ListFiles(source, files)); 

        for (i = 0; i < kArrayList_Count(files); ++i)
        {
            kTest(kArrayList_ItemT(files, i, &srcItem)); 
            kTest(kPath_Combine(source, kString_Chars(srcItem), srcPath, kCountOf(srcPath))); 
            kTest(kPath_Combine(destination, kString_Chars(srcItem), destPath, kCountOf(destPath))); 
            kTest(kFile_Copy(srcPath, destPath));  
        }

        //copy directories
        kTest(kArrayList_Construct(&directories, kNULL, 0, kNULL)); 
        kTest(kDirectory_ListDirectories(source, directories)); 

        for (i = 0; i < kArrayList_Count(directories); ++i)
        {
            kTest(kArrayList_ItemT(directories, i, &srcItem)); 
            kTest(kPath_Combine(source, kString_Chars(srcItem), srcPath, kCountOf(srcPath))); 
            kTest(kPath_Combine(destination, kString_Chars(srcItem), destPath, kCountOf(destPath))); 
            kTest(kDirectory_Copy(srcPath, destPath));  
        }
    }
    kFinally
    {
        kCheck(kObject_Dispose(files)); 
        kCheck(kObject_Dispose(directories)); 
        
        kEndFinally(); 
    }

    return kOK; 
}

kFx(kStatus) kDirectory_Move(const kChar* source, const kChar* destination)
{
    return kApiLib_DirectoryHandlers()->move(source, destination); 
}

kFx(kStatus) kDirectory_Delete(const kChar* directory)
{
    kCheckTrue(kDirectory_Exists(directory), kERROR_NOT_FOUND);

    kCheck(kDirectory_DeleteEntries(directory)); 

    //delete the target directory
    kCheck(kApiLib_DirectoryHandlers()->del(directory)); 

    return kOK; 
}

kFx(kStatus) kDirectory_DeleteEntries(const kChar* directory)
{
    kSize i;
    kArrayList files = kNULL; 
    kArrayList directories = kNULL; 
    kString srcItem = kNULL; 
    kChar srcPath[kPATH_MAX]; 

    kCheckTrue(kDirectory_Exists(directory), kERROR_NOT_FOUND);

    kTry
    {
        //delete files
        kTest(kArrayList_Construct(&files, kNULL, 0, kNULL)); 
        kTest(kDirectory_ListFiles(directory, files)); 

        for (i = 0; i < kArrayList_Count(files); ++i)
        {
            kTest(kArrayList_ItemT(files, i, &srcItem)); 
            kTest(kPath_Combine(directory, kString_Chars(srcItem), srcPath, kCountOf(srcPath))); 
            kTest(kFile_Delete(srcPath));  
        }

        //delete sub-directories
        kTest(kArrayList_Construct(&directories, kNULL, 0, kNULL)); 
        kTest(kDirectory_ListDirectories(directory, directories)); 

        for (i = 0; i < kArrayList_Count(directories); ++i)
        {
            kTest(kArrayList_ItemT(directories, i, &srcItem)); 
            kTest(kPath_Combine(directory, kString_Chars(srcItem), srcPath, kCountOf(srcPath))); 
            kTest(kDirectory_Delete(srcPath));  
        }    
    }
    kFinally
    {
        kCheck(kObject_Dispose(files)); 
        kCheck(kObject_Dispose(directories)); 
        
        kEndFinally(); 
    }

    return kOK; 
}


kFx(kStatus) kDirectory_Enumerate(const kChar* directory, kBool includeFiles, kBool includeDirectories, kArrayList entries)
{    
    kCheck(kArrayList_Purge(entries)); 
    
    return kApiLib_DirectoryHandlers()->enumerate(directory, includeFiles, includeDirectories, entries); 
}

kFx(kStatus) kDirectory_ListFiles(const kChar* directory, kArrayList files)
{    
    kCheck(kArrayList_Purge(files)); 

    return kApiLib_DirectoryHandlers()->enumerate(directory, kTRUE, kFALSE, files); 
}

kFx(kStatus) kDirectory_ListDirectories(const kChar* directory, kArrayList directories)
{        
    kCheck(kArrayList_Purge(directories)); 

    return kApiLib_DirectoryHandlers()->enumerate(directory, kFALSE, kTRUE, directories); 
}

kFx(kStatus) kDirectory_ListEntries(const kChar* directory, kArrayList entries)
{    
    kCheck(kArrayList_Purge(entries)); 

    return kApiLib_DirectoryHandlers()->enumerate(directory, kTRUE, kTRUE, entries); 
}

kFx(kStatus) kDirectory_SetCurrent(const kChar* directory)
{
    return kApiLib_DirectoryHandlers()->setCurrent(directory); 
}

kFx(kStatus) kDirectory_Current(kChar* directory, kSize capacity)
{
    return kApiLib_DirectoryHandlers()->current(directory, capacity); 
}

kFx(kStatus) kDirectory_Application(kChar* directory, kSize capacity)
{
    return kApiLib_DirectoryHandlers()->appDirectory(directory, capacity); 
}

kFx(kStatus) kDirectory_Api(kChar* directory, kSize capacity)
{
    return kApiLib_DirectoryHandlers()->apiDirectory(directory, capacity); 
}

kFx(kStatus) kDirectory_Temp(kChar* directory, kSize capacity)
{
    return kApiLib_DirectoryHandlers()->tempDirectory(directory, capacity); 
}

kFx(kStatus) kDirectory_AppConfig(const kChar* appName, kChar* directory, kSize capacity)
{
    return kApiLib_DirectoryHandlers()->appConfigDirectory(appName, directory, capacity); 
}

kFx(kStatus) kDirectory_AppData(const kChar* appName, kChar* directory, kSize capacity)
{
    return kApiLib_DirectoryHandlers()->appDataDirectory(appName, directory, capacity); 
}

kFx(kStatus) xkDirectory_AppConfigImpl(const kChar* appName, kChar* directory, kSize capacity)
{    
    kCheck(kDirectory_Api(directory, capacity)); 
    kCheck(kPath_Directory(directory, directory, capacity)); 
    kCheck(kPath_Directory(directory, directory, capacity)); 
    kCheck(kPath_Combine(directory, xkDIRECTORY_DEFAULT_CONFIG_FOLDER, directory, capacity)); 

    if (!kIsNull(appName))
    {
        kCheck(kPath_Combine(directory, appName, directory, capacity)); 
    }

    return kOK; 
}

kFx(kStatus) xkDirectory_AppDataImpl(const kChar* appName, kChar* directory, kSize capacity)
{    
    kCheck(kDirectory_Application(directory, capacity)); 
    kCheck(kPath_Directory(directory, directory, capacity)); 
    kCheck(kPath_Directory(directory, directory, capacity)); 
    kCheck(kPath_Combine(directory, xkDIRECTORY_DEFAULT_DATA_FOLDER, directory, capacity)); 

    if (!kIsNull(appName))
    {
        kCheck(kPath_Combine(directory, appName, directory, capacity)); 
    }

    return kOK; 
}

kFx(kStatus) xkDirectory_FromVirtualImpl(const kChar* vPath, kChar* path, kSize capacity)
{
    return kStrCopy(path, capacity, vPath); 
}

kFx(kStatus) xkDirectory_ToVirtualImpl(const kChar* path, kChar* vPath, kSize capacity)
{
    return kStrCopy(vPath, capacity, path); 
}

kFx(kStatus) kDirectory_Plugin(kChar* directory, kSize capacity)
{
    return kDirectory_Api(directory, capacity);
}

#if defined(K_WINDOWS)

kFx(kStatus) xkDirectory_CreateImpl(const kChar* path)
{
    WCHAR wpath[MAX_PATH]; 

    kCheck(xkPath_NormalizedToNativeWideWin(path, wpath, kCountOf(wpath))); 

    return CreateDirectory(wpath, NULL) ? kOK : kERROR_OS;
}

kFx(kBool) xkDirectory_ExistsImpl(const kChar* path)
{
    WCHAR wpath[MAX_PATH]; 
    DWORD attr = 0; 

    kCheck(xkPath_NormalizedToNativeWideWin(path, wpath, kCountOf(wpath))); 
    
    attr = GetFileAttributes(wpath);

    return (attr != INVALID_FILE_ATTRIBUTES) && (attr & FILE_ATTRIBUTE_DIRECTORY);
}

kFx(kStatus) xkDirectory_MoveImpl(const kChar* source, const kChar* destination)
{
    WCHAR wpathSrc[MAX_PATH];
    WCHAR wpathDst[MAX_PATH];  
    kSize i;

    kCheck(xkPath_NormalizedToNativeWideWin(source, wpathSrc, kCountOf(wpathSrc))); 
    kCheck(xkPath_NormalizedToNativeWideWin(destination, wpathDst, kCountOf(wpathDst))); 
    
    for (i = 0; i < xkDIRECTORY_WINDOWS_IO_ATTEMPTS; i++)
    {
        if (MoveFile(wpathSrc, wpathDst))
        {
            return kOK;
        }
        else if (GetLastError() == xkDIRECTORY_WINDOWS_ACCESS_DENIED)
        {
            kCheck(kThread_Sleep(100000));
        }
        else
        {
            return kERROR_OS;
        }
    }

    return kERROR_OS;
}

kFx(kStatus) xkDirectory_DeleteImpl(const kChar* path)
{
    WCHAR tempWidePath[MAX_PATH]; 
    kChar absPath[kPATH_MAX];
    kChar parent[kPATH_MAX];
    kChar tempName[kPATH_MAX];
    kBool directoryAlreadyExists = kFALSE; 
    kBool fileAlreadyExists = kFALSE; 
    kBool moveFailure = kFALSE; 
    kSize retryCount = 0; 

    //we want to get the parent directory, but the given path may be relative;
    //if so, we convert to an absolute path that is relative to the current working directory
    kCheck(kPath_ToAbsolute(kNULL, path, absPath, sizeof(absPath)));
    kCheck(kPath_Directory(absPath, parent, kCountOf(parent)));

    do
    {
        kCheckTrue(retryCount++ < xkFILE_WINDOWS_IO_ATTEMPTS, kERROR_OS); 

        kCheck(kFile_TempName(tempName, kCountOf(tempName)));
        kCheck(kPath_Combine(parent, tempName, tempName, kCountOf(tempName)));
        kCheck(xkPath_NormalizedToNativeWideWin(tempName, tempWidePath, kCountOf(tempWidePath))); 

        directoryAlreadyExists = kDirectory_Exists(tempName); 

        if (directoryAlreadyExists)
        {
            //should never get here; but if we do, clean up and select a new name
            RemoveDirectory(tempWidePath); 
        }
        else
        {
            fileAlreadyExists = kFile_Exists(tempName); 

            if (fileAlreadyExists)
            {
                //should never get here; but if we do, clean up and select a new name
                kFile_Delete(tempName);
            }
            else 
            {
                moveFailure = kIsError(kDirectory_Move(absPath, tempName));
            }
        }
    }
    while (directoryAlreadyExists || fileAlreadyExists || moveFailure); 

    return RemoveDirectory(tempWidePath) ? kOK : kERROR_OS; 
}

kFx(kStatus) xkDirectory_EnumerateImpl(const kChar* path, kBool includeFile, kBool includeDirectories, kArrayList entries)
{
    HANDLE findHandle = INVALID_HANDLE_VALUE;
    WCHAR searchFilter[MAX_PATH];
    kChar entry[MAX_PATH];
    WIN32_FIND_DATA findData;
    kString item = kNULL;

    kTry
    {
        kTest(xkPath_NormalizedToNativeWideWin(path, searchFilter, kCountOf(searchFilter))); 
        wcscat(searchFilter, L"\\*"); 

        if ((findHandle = FindFirstFile(searchFilter, &findData)) == INVALID_HANDLE_VALUE)
        {
            kThrow(kERROR_OS);
        }

        kTest(kArrayList_Allocate(entries, kTypeOf(kString), 0)); 
        
        do
        {
            kBool isDirectory = (findData.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) != 0; 
            kBool isSpecial = (wcscmp(findData.cFileName, L".") == 0) || (wcscmp(findData.cFileName, L"..") == 0); 
            kBool filter = !isSpecial && ((includeFile && !isDirectory) || (includeDirectories && isDirectory)); 

            if (filter)
            {
                kTest(xkPath_NativeWideToNormalizedWin(findData.cFileName, entry, kCountOf(entry))); 

                kTest(kString_Construct(&item, entry, kNULL)); 
                kTest(kArrayList_AddT(entries, &item));
                item = kNULL;
            }
        } 
        while (FindNextFile(findHandle, &findData));
    } 
    kFinally
    {
        kCheck(kObject_Destroy(item)); 

        if (findHandle != INVALID_HANDLE_VALUE)
        {
            FindClose(findHandle);
        }

        kEndFinally(); 
    }

    return kOK;
}

kFx(kStatus) xkDirectory_SetCurrentImpl(const kChar* path)
{
    WCHAR wpath[MAX_PATH]; 

    kCheck(xkPath_NormalizedToNativeWideWin(path, wpath, kCountOf(wpath))); 

    return SetCurrentDirectory(wpath) ? kOK : kERROR_OS;
}

kFx(kStatus) xkDirectory_CurrentImpl(kChar* directory, kSize capacity)
{
    WCHAR wpath[MAX_PATH]; 

    if (GetCurrentDirectory(kCountOf(wpath), wpath) == 0)
    {
        return kERROR_OS;
    }
  
    kCheck(xkPath_NativeWideToNormalizedWin(wpath, directory, capacity)); 

    return kOK; 
}

kFx(kStatus) xkDirectory_ApplicationImpl(kChar* directory, kSize capacity)
{
    WCHAR wmodulePath[kPATH_MAX]; 
    HANDLE module;
    
    if (kIsNull(module = GetModuleHandle(NULL)))
    {
        return kERROR; 
    }

    if (GetModuleFileName((HMODULE)module, wmodulePath, kCountOf(wmodulePath)) == 0)
    {
        return kERROR; 
    }

    kCheck(xkPath_NativeWideToNormalizedWin(wmodulePath, directory, capacity)); 
    kCheck(kPath_Directory(directory, directory, capacity)); 

    return kOK;
}

kFx(kStatus) xkDirectory_ApiImpl(kChar* directory, kSize capacity)
{
    WCHAR wmodulePath[kPATH_MAX]; 
    HMODULE module;

    if (!GetModuleHandleExA(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | 
        GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
        (LPCSTR) &xkDirectory_ApiImpl, &module))
    {
        return kERROR; 
    }

    if (GetModuleFileName(module, wmodulePath, kCountOf(wmodulePath)) == 0)
    {
        return kERROR; 
    }

    kCheck(xkPath_NativeWideToNormalizedWin(wmodulePath, directory, capacity)); 
    kCheck(kPath_Directory(directory, directory, capacity)); 

    return kOK;
}

kFx(kStatus) xkDirectory_TempImpl(kChar* path, kSize capacity)
{
    WCHAR wpath[kPATH_MAX]; 

    if (GetTempPath(kCountOf(wpath), wpath) == 0)
    {
        return kERROR_OS;
    }
    
    kCheck(xkPath_NativeWideToNormalizedWin(wpath, path, capacity)); 
    
    return kOK; 
}

#elif defined(K_DARWIN)

kFx(kStatus) xkDirectory_CreateImpl(const kChar* directory)
{
    kChar nativeDirectory[kPATH_MAX];

    kCheck(xkPath_FromVirtual(directory, nativeDirectory, kCountOf(nativeDirectory)));

    return (mkdir(nativeDirectory, S_IRWXU) == -1) ? kERROR_OS : kOK; 
}

kFx(kBool) xkDirectory_ExistsImpl(const kChar* directory)
{   
    struct stat st; 
    kChar nativeDirectory[kPATH_MAX];

    kCheck(xkPath_FromVirtual(directory, nativeDirectory, kCountOf(nativeDirectory)));

    return (stat(nativeDirectory, &st) != -1) && S_ISDIR(st.st_mode); 
}

kFx(kStatus) xkDirectory_MoveImpl(const kChar* source, const kChar* destination)
{
    kChar nativeSource[kPATH_MAX];
    kChar nativeDestination[kPATH_MAX];

    kCheck(xkPath_FromVirtual(source, nativeSource, kCountOf(nativeSource)));
    kCheck(xkPath_FromVirtual(destination, nativeDestination, kCountOf(nativeDestination)));

    return (rename(nativeSource, nativeDestination) == -1) ? kERROR_OS : kOK; 
}

kFx(kStatus) xkDirectory_DeleteImpl(const kChar* directory)
{
    kChar nativeDirectory[kPATH_MAX];

    kCheck(xkPath_FromVirtual(directory, nativeDirectory, kCountOf(nativeDirectory)));

    return (rmdir(nativeDirectory) == -1) ? kERROR_OS : kOK; 
}

kFx(kStatus) xkDirectory_EnumerateImpl(const kChar* directory, kBool includeFile, kBool includeDirectories, kArrayList entries)
{
    kChar nativeDirectory[kPATH_MAX];
    DIR* dirIt = kNULL; 
    struct dirent* dirEntry = kNULL; 
    kString item = kNULL; 

    kCheck(xkPath_FromVirtual(directory, nativeDirectory, kCountOf(nativeDirectory)));

    kCheckTrue((dirIt = opendir(nativeDirectory)) != kNULL, kERROR_NOT_FOUND); 

    kTry
    {
        kTest(kArrayList_Allocate(entries, kTypeOf(kString), 0)); 
  
        while ((dirEntry = readdir(dirIt)) != kNULL)
        {
            kBool isDirectory = (dirEntry->d_type == DT_DIR); 
            kBool isSpecial = (kStrEquals(dirEntry->d_name, ".") || kStrEquals(dirEntry->d_name, "..")); 
            kBool filter = !isSpecial && ((includeFile && !isDirectory) || (includeDirectories && isDirectory)); 

            if (filter)
            {               
                kTest(kString_Construct(&item, dirEntry->d_name, kNULL)); 
                kTest(kArrayList_AddT(entries, &item));
                item = kNULL;
            }
        } 
    }
    kFinally
    {
        closedir(dirIt);
        kObject_Destroy(item); 

        kEndFinally(); 
    }

    return kOK; 
}

kFx(kStatus) xkDirectory_SetCurrentImpl(const kChar* directory)
{
    kChar nativeDirectory[kPATH_MAX];

    kCheck(xkPath_FromVirtual(directory, nativeDirectory, kCountOf(nativeDirectory)));

    return (chdir(nativeDirectory) == -1) ? kERROR_OS : kOK; 
}

kFx(kStatus) xkDirectory_CurrentImpl(kChar* directory, kSize capacity)
{
    kChar nativePath[kPATH_MAX]; 

    if (getcwd(nativePath, kCountOf(nativePath)) == kNULL)
    {
        return kERROR_OS;
    }

    kCheck(xkPath_ToVirtual(nativePath, directory, capacity)); 

    return kOK;
}

kFx(kStatus) xkDirectory_ApplicationImpl(kChar* directory, kSize capacity)
{
    kChar modulePath[kPATH_MAX];
    uint32_t size = kCountOf(modulePath); 

    if (_NSGetExecutablePath(modulePath, &size) != 0)
    {
        return kERROR; 
    }

    kCheck(kPath_ToNormal(modulePath, modulePath, kCountOf(modulePath)));  
    kCheck(xkPath_ToVirtual(modulePath, modulePath, kCountOf(modulePath)));  

    kCheck(kPath_Directory(modulePath, directory, capacity));     
    kCheck(kPath_Directory(directory, directory, capacity));         

    return kOK;
}

kFx(kStatus) xkDirectory_ApiImpl(kChar* directory, kSize capacity)
{
    kChar baseName[kPATH_MAX];

    //resolving the plugin directory via the application directory is not necessarily correct, but good enough in most cases
    kCheck(xkDirectory_ApplicationImpl(directory, capacity)); 
    kCheck(kPath_BaseName(directory, baseName, kCountOf(baseName))); 

    kCheck(kPath_Directory(directory, directory, capacity)); 
    kCheck(kPath_Directory(directory, directory, capacity)); 

    kCheck(kPath_Combine(directory, "lib", directory, capacity)); 
    kCheck(kPath_Combine(directory, baseName, directory, capacity)); 

    return kOK; 
}

kFx(kStatus) xkDirectory_TempImpl(kChar* directory, kSize capacity)
{
    //TODO: system-specific; could probably be improved (?)
    return xkPath_ToVirtual("/tmp", directory, capacity); 
}

#elif defined(K_LINUX)

kFx(kStatus) xkDirectory_CreateImpl(const kChar* directory)
{
    kChar nativeDirectory[kPATH_MAX];

    kCheck(xkPath_FromVirtual(directory, nativeDirectory, kCountOf(nativeDirectory)));

    return (mkdir(nativeDirectory, S_IRWXU) == -1) ? kERROR_OS : kOK; 
}

kFx(kBool) xkDirectory_ExistsImpl(const kChar* directory)
{   
    struct stat st; 
    kChar nativeDirectory[kPATH_MAX];

    kCheck(xkPath_FromVirtual(directory, nativeDirectory, kCountOf(nativeDirectory)));

    return (stat(nativeDirectory, &st) != -1) && S_ISDIR(st.st_mode); 
}

kFx(kStatus) xkDirectory_MoveImpl(const kChar* source, const kChar* destination)
{
    kChar nativeSource[kPATH_MAX];
    kChar nativeDestination[kPATH_MAX];

    kCheck(xkPath_FromVirtual(source, nativeSource, kCountOf(nativeSource)));
    kCheck(xkPath_FromVirtual(destination, nativeDestination, kCountOf(nativeDestination)));

    return (rename(nativeSource, nativeDestination) == -1) ? kERROR_OS : kOK; 
}

kFx(kStatus) xkDirectory_DeleteImpl(const kChar* directory)
{
    kChar nativeDirectory[kPATH_MAX];

    kCheck(xkPath_FromVirtual(directory, nativeDirectory, kCountOf(nativeDirectory)));

    return (rmdir(nativeDirectory) == -1) ? kERROR_OS : kOK; 
}

kFx(kStatus) xkDirectory_EnumerateImpl(const kChar* directory, kBool includeFile, kBool includeDirectories, kArrayList entries)
{
    kChar nativeDirectory[kPATH_MAX];
    DIR* dirIt = kNULL; 
    struct dirent* dirEntry = kNULL; 
    kString item = kNULL; 

    kCheck(xkPath_FromVirtual(directory, nativeDirectory, kCountOf(nativeDirectory)));

    kCheckTrue((dirIt = opendir(nativeDirectory)) != kNULL, kERROR_NOT_FOUND); 

    kTry
    {
        kTest(kArrayList_Allocate(entries, kTypeOf(kString), 0)); 
  
        while ((dirEntry = readdir(dirIt)) != kNULL)
        {
            kBool isDirectory = (dirEntry->d_type == DT_DIR); 
            kBool isSpecial = (kStrEquals(dirEntry->d_name, ".") || kStrEquals(dirEntry->d_name, "..")); 
            kBool filter = !isSpecial && ((includeFile && !isDirectory) || (includeDirectories && isDirectory)); 

            if (filter)
            {               
                kTest(kString_Construct(&item, dirEntry->d_name, kNULL)); 
                kTest(kArrayList_AddT(entries, &item));
                item = kNULL;
            }
        } 
    }
    kFinally
    {
        closedir(dirIt);
        kObject_Destroy(item); 

        kEndFinally(); 
    }

    return kOK; 
}

kFx(kStatus) xkDirectory_SetCurrentImpl(const kChar* directory)
{
    kChar nativeDirectory[kPATH_MAX];

    kCheck(xkPath_FromVirtual(directory, nativeDirectory, kCountOf(nativeDirectory)));

    return (chdir(nativeDirectory) == -1) ? kERROR_OS : kOK; 
}

kFx(kStatus) xkDirectory_CurrentImpl(kChar* directory, kSize capacity)
{
    kChar nativePath[kPATH_MAX]; 

    if (getcwd(nativePath, kCountOf(nativePath)) == kNULL)
    {
        return kERROR_OS;
    }

    kCheck(xkPath_ToVirtual(nativePath, directory, capacity)); 

    return kOK;
}

kFx(kStatus) xkDirectory_ApplicationImpl(kChar* directory, kSize capacity)
{
    kChar modulePath[kPATH_MAX];
    ssize_t length; 
    
    if ((length = readlink("/proc/self/exe", modulePath, kCountOf(modulePath)-1)) == -1)
    {
        return kERROR; 
    }

    modulePath[length] = 0;

    kCheck(kPath_ToNormal(modulePath, modulePath, kCountOf(modulePath))); 
    kCheck(xkPath_ToVirtual(modulePath, modulePath, kCountOf(modulePath)));  

    kCheck(kPath_Directory(modulePath, directory, capacity)); 

    return kOK;
}

kFx(kStatus) xkDirectory_ApiImpl(kChar* directory, kSize capacity)
{
    Dl_info info;

    kCheck(dladdr((void*)xkDirectory_ApiImpl, &info) != 0);

    kCheck(kPath_Directory(info.dli_fname, directory, capacity)); 

    return kOK; 
}

kFx(kStatus) xkDirectory_TempImpl(kChar* directory, kSize capacity)
{
    //TODO: system-specific; could probably be improved (?)
    return xkPath_ToVirtual("/tmp", directory, capacity); 
}

#else

kFx(kStatus) xkDirectory_CreateImpl(const kChar* directory)
{   
    kAssert(kFALSE); 
    return kERROR_UNIMPLEMENTED;
}

kFx(kBool) xkDirectory_ExistsImpl(const kChar* directory)
{   
    kAssert(kFALSE); 
    return kFALSE; 
}

kFx(kStatus) xkDirectory_MoveImpl(const kChar* source, const kChar* destination)
{   
    kAssert(kFALSE); 
    return kERROR_UNIMPLEMENTED;
}

kFx(kStatus) xkDirectory_DeleteImpl(const kChar* directory)
{   
    kAssert(kFALSE); 
    return kERROR_UNIMPLEMENTED;
}

kFx(kStatus) xkDirectory_EnumerateImpl(const kChar* directory, kBool includeFile, kBool includeDirectories, kArrayList entries)
{   
    kAssert(kFALSE); 
    return kERROR_UNIMPLEMENTED;
}

kFx(kStatus) xkDirectory_SetCurrentImpl(const kChar* directory)
{   
    kAssert(kFALSE); 
    return kERROR_UNIMPLEMENTED;
}

kFx(kStatus) xkDirectory_CurrentImpl(kChar* directory, kSize capacity)
{   
    kAssert(kFALSE); 
    return kERROR_UNIMPLEMENTED;
}

kFx(kStatus) xkDirectory_ApplicationImpl(kChar* directory, kSize capacity)
{   
    kAssert(kFALSE); 
    return kERROR_UNIMPLEMENTED;
}

kFx(kStatus) xkDirectory_ApiImpl(kChar* directory, kSize capacity)
{   
    kAssert(kFALSE); 
    return kERROR_UNIMPLEMENTED;
}

kFx(kStatus) xkDirectory_TempImpl(kChar* directory, kSize capacity)
{   
    kAssert(kFALSE); 
    return kERROR_UNIMPLEMENTED;
}

#endif


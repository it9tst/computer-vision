// 
// KDirectory.h
// 
// Copyright (C) 2014-2021 by LMI Technologies Inc.
// Licensed under the MIT License.
// Redistributed files must retain the above copyright notice.
// 
#ifndef K_API_NET_DIRECTORY_H
#define K_API_NET_DIRECTORY_H

#include <kApi/Io/kDirectory.h>
#include <kApi/Io/kPath.h>
#include "kApiNet/KAlloc.h"
#include "kApiNet/Data/KString.h"
#include "kApiNet/Utils/KUtils.h"

namespace Lmi3d
{
    namespace Zen 
    {
        namespace Io
        {
            /// <summary>Collection of directory-related utility methods.</summary>
            ///
            /// <remarks>
            /// <para>Default KRefStyle: None</para>
            /// </remarks>
            public ref class KDirectory : public KObject
            {
                KDeclareNoneClass(KDirectory, kDirectory)

            public:

                /// <summary>Creates a directory at the specified location.</summary>
                /// 
                /// <remarks>This method will fail if the directory already exists. Missing parent folders will 
                /// be created automatically.</remarks>
                /// 
                /// <param name="directory">Full path of the directory.</param>
                static void Create(String^ directory)
                {
                    KString dir(directory); 

                    KCheck(kDirectory_Create(dir.CharPtr)); 
                }

                /// <summary>Reports whether the specified directory exists.</summary>
                /// 
                /// <param name="directory">Full path of the directory.</param>
                /// <returns>true if the directory exists, kFALSE otherwise.</returns>
                static bool Exists(String^ directory)
                {
                    KString dir(directory);

                    return KToBool(kDirectory_Exists(dir.CharPtr));
                }

                /// <summary>Copies the specified directory, including all of its contents.</summary>
                /// 
                /// <param name="source">Full path of the source directory.</param>
                /// <param name="destination">Full path of the destination directory.</param>
                static void Copy(String^ source, String^ destination)
                {
                    KString src(source); 
                    KString dest(destination); 

                    KCheck(kDirectory_Copy(src.CharPtr, dest.CharPtr)); 
                }

                /// <summary>Moves the specified directory, including all of its contents.</summary>
                /// 
                /// <param name="source">Full path of the source directory.</param>
                /// <param name="destination">Full path of the destination directory.</param>
                static void Move(String^ source, String^ destination)
                {
                    KString src(source);
                    KString dest(destination);

                    KCheck(kDirectory_Move(src.CharPtr, dest.CharPtr));
                }

                /// <summary>Deletes the specified directory, including all of its contents.</summary>
                /// 
                /// <param name="directory">Full path of the directory.</param>
                static void Delete(String^ directory)
                {
                    KString dir(directory);
                    
                    KCheck(kDirectory_Delete(dir.CharPtr)); 
                }

                /// <summary>List the files in the specified directory.</summary>
                /// 
                /// <remarks>Use KObject.Dispose to destroy the list returned by this method.</remarks>
                /// 
                /// <param name="directory">Full path of the directory.</param>
                /// <returns>List of file names.</returns>
                static KArrayList^ ListFiles(String^ directory)
                {
                    KString dir(directory);
                    KArrayList^ list = gcnew KArrayList();

                    try
                    {
                        KCheck(kDirectory_ListFiles(dir.CharPtr, KToHandle(list)));

                        return list; 
                    }
                    catch (...)
                    {
                        KUtils::Dispose(list); 
                        throw; 
                    }
                }

                /// <summary>Creates a list of the sub-directories in the specified directory.</summary>
                /// 
                /// <remarks>Use KObject.Dispose to destroy the list returned by this method.</remarks>
                /// 
                /// <param name="directory">Full path of the directory.</param>
                /// <returns>List of directory names.</returns>
                static KArrayList^ ListDirectories(String^ directory)
                {
                    KString dir(directory);
                    KArrayList^ list = gcnew KArrayList();

                    try
                    {
                        KCheck(kDirectory_ListDirectories(dir.CharPtr, KToHandle(list)));

                        return list;
                    }
                    catch (...)
                    {
                        KUtils::Dispose(list);
                        throw;
                    }
                }

                /// <summary>Creates a list of the file system entries in the specified directory.</summary>
                /// 
                /// <remarks>Use KObject.Dispose to destroy the list returned by this method.</remarks>
                /// 
                /// <param name="directory">Full path of the directory.</param>
                /// <returns>List of file system entries (directories and files).</returns>
                static KArrayList^ ListEntries(String^ directory)
                {
                    KString dir(directory);
                    KArrayList^ list = gcnew KArrayList();

                    try
                    {
                        KCheck(kDirectory_ListEntries(dir.CharPtr, KToHandle(list)));

                        return list;
                    }
                    catch (...)
                    {
                        KUtils::Dispose(list);
                        throw;
                    }
                }

                /// <summary>Sets the current working directory.</summary>
                /// 
                /// <param name="directory">Full path of the desired working directory.</param>
                static void SetCurrent(String^ directory)
                {
                    KString dir(directory);

                    KCheck(kDirectory_SetCurrent(dir.CharPtr)); 
                }

                /// <summary>Gets the current working directory.</summary>
                static property String^ Current
                {
                    String^ get()
                    {
                        kChar path[kPATH_MAX];

                        KCheck(kDirectory_Current(path, kCountOf(path)));

                        return KToString(path);
                    }
                }

                /// <summary>Gets the directory in which the application executable file resides.</summary>
                static property String^ Application
                {
                    String^ get()
                    {
                        kChar path[kPATH_MAX];

                        KCheck(kDirectory_Application(path, kCountOf(path)));

                        return KToString(path);
                    }
                }

                /// <summary>Gets the path of a directory suitable for temporary files.</summary>
                static property String^ Temp
                {
                    String^ get()
                    {
                        kChar path[kPATH_MAX];

                        KCheck(kDirectory_Temp(path, kCountOf(path)));

                        return KToString(path);
                    }
                }

                /// <summary>Gets the shared directory from which applications should load configuration/resource files.</summary>
                /// 
                /// <remarks>This property assumes the standard folder organization of a zen-based application.</remarks>
                static property String^ AppConfig
                {
                    String^ get()
                    {
                        kChar path[kPATH_MAX];

                        KCheck(kDirectory_AppConfig(kNULL, path, kCountOf(path)));

                        return KToString(path);
                    }
                }

                /// <summary>Gets the directory from which an application should load its configuration/resource files.</summary>
                /// 
                /// <remarks>This method assumes the standard folder organization of a zen-based application.</remarks>
                /// 
                /// <param name="appName">The name of the application.</param>
                /// <returns>Application configuration directory.</returns>
                static String^ GetAppConfig(String^ appName)
                {
                    KString appStr(appName); 
                    kChar path[kPATH_MAX];

                    KCheck(kDirectory_AppConfig(appStr.CharPtr, path, kCountOf(path))); 

                    return KToString(path);
                }

                /// <summary>Gets the shared directory from which applications should load/save data files.</summary>
                /// 
                /// <remarks>This property assumes the standard folder organization of a zen-based application.</remarks>
                static property String^ AppData
                {
                    String^ get()
                    {
                        kChar path[kPATH_MAX];

                        KCheck(kDirectory_AppData(kNULL, path, kCountOf(path)));

                        return KToString(path);
                    }
                }

                /// <summary>Gets the directory from which an application should load/save data files.</summary>
                /// 
                /// <remarks>This method assumes the standard folder organization of a zen-based application.</remarks>
                /// 
                /// <param name="appName">The name of the application.</param>
                /// <returns>Application data directory.</returns>
                static String^ GetAppData(String^ appName)
                {
                    KString appStr(appName);
                    kChar path[kPATH_MAX];

                    KCheck(kDirectory_AppData(appStr.CharPtr, path, kCountOf(path)));

                    return KToString(path);
                }

                /// <summary>Gets the directory where plug-ins are located.</summary>
                /// 
                /// <remarks>This property assumes the standard folder organization of a zen-based application.</remarks>
                static property String^ Plugin
                {
                    String^ get()
                    {
                        kChar path[kPATH_MAX];

                        KCheck(kDirectory_Plugin(path, kCountOf(path)));

                        return KToString(path);
                    }
                }

                private:
                    KDirectory() : KObject(DefaultRefStyle) {};
            };
        }
    }
}

#endif

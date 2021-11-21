// 
// KPath.h
// 
// Copyright (C) 2014-2021 by LMI Technologies Inc.
// Licensed under the MIT License.
// Redistributed files must retain the above copyright notice.
// 
#ifndef K_API_NET_PATH_H
#define K_API_NET_PATH_H

#include <kApi/Io/kPath.h>
#include "kApiNet/KAlloc.h"
#include "kApiNet/Data/KString.h"

namespace Lmi3d
{
    namespace Zen 
    {
        namespace Io
        {
            /// <summary>Collection of path manipulation methods.</summary>
            /// 
            /// <remarks>
            /// <para>Most of the methods in this class expect paths to be in normal form (canonical
            /// separator character, no trailing slashes). The KPath.ToNormal method can be used to normalize 
            /// paths prior to calling other KPath methods.</para>
            /// 
            /// <para>The KPath.ToNative method can be used to transform a path in normal form
            /// to a form suitable for the underlying system (native separator).</para>
            ///
            /// <para>Default KRefStyle: None</para>
            /// </remarks>
            public ref class KPath : public KObject
            {
                KDeclareNoneClass(KPath, kPath)

            public:

                static const k32s MaxPath = kPATH_MAX; 

                /// <summary>Gets the normalized path separator character.</summary>
                static property String^ Separator
                {
                    String^ get()
                    {
                        kChar sep[2] = { 0, 0 };
                        sep[0] = kPath_Separator(); 

                        return KToString(sep); 
                    }
                }

                /// <summary>Determines if the given character is equal to the normalized path separator.</summary>
                /// <param name="separator">Character to examine.</param>
                /// <returns>true if the character is the normalized path separator.</returns>
                static bool IsSeparator(String^ separator)
                {
                    KString sepStr(separator);

                    return KToBool(kPath_IsSeparator(sepStr.CharPtr[0]));
                }

                /// <summary>Combines two path segments using the normalized path separator character.</summary>
                /// 
                /// <param name="segment1">First path segment.</param>
                /// <param name="segment2">Second path segment.</param>
                /// <returns>The combined path segments.</returns>
                static String^ Combine(String^ segment1, String^ segment2)
                {
                    KString s1(segment1); 
                    KString s2(segment2); 
                    kChar result[kPATH_MAX]; 

                    KCheck(kPath_Combine(s1.CharPtr, s2.CharPtr, result, kCountOf(result))); 

                    return KToString(result); 
                }

                /// <summary>Returns the parent directory for a given file or directory path.</summary>
                /// 
                /// <remarks>
                /// <para>If the path contains embedded relative path components, this method will not attempt
                /// to evaluate the meaning of those components.</para>
                /// </remarks>
                /// 
                /// <param name="path">File or directory path.</param>
                /// <returns>Parent directory portion of the path.</returns>
                static String^ GetDirectory(String^ path)
                {
                    KString pathStr(path);
                    kChar result[kPATH_MAX];

                    KCheck(kPath_Directory(pathStr.CharPtr, result, kCountOf(result)));

                    return KToString(result);
                }

                /// <summary>Given a file path, returns the portion of the path containing the file name.</summary>
                /// 
                /// <param name="path">File path.</param>
                /// <returns>File name portion of the path.</returns>
                static String^ GetFileName(String^ path)
                {
                    KString pathStr(path);
                    kChar result[kPATH_MAX];

                    KCheck(kPath_FileName(pathStr.CharPtr, result, kCountOf(result)));

                    return KToString(result);
                }

                /// <summary>Given a file path, returns the portion of the path containing the file extension.</summary>
                /// 
                /// <param name="path">File path.</param>
                /// <returns>File extension portion of the path.</returns>
                static String^ GetExtension(String^ path)
                {
                    KString pathStr(path);
                    kChar result[kPATH_MAX];

                    KCheck(kPath_Extension(pathStr.CharPtr, result, kCountOf(result)));

                    return KToString(result);
                }

                /// <summary>Expresses an absolute path in relative form, in relation to a reference path.</summary>
                /// 
                /// <remarks>
                /// <para>If pathA and pathB are rooted in different volumes, then bRelativeToA will receive an absolute path equal
                /// to pathB.</para>
                /// 
                /// <para>The bRelativeToA (output) argument can refer to the same memory address as either the pathA or pathB
                /// (input) arguments.</para>
                /// </remarks>
                /// 
                /// <param name="pathA">Absolute reference path.</param>
                /// <param name="pathB">Absolute path to be re-expressed as relative to path A.</param>
                /// <returns>Path b, expressed as relative to path A.</returns>
                static String^ ToRelative(String^ pathA, String^ pathB)
                {
                    KString pathAStr(pathA);
                    KString pathBStr(pathB);
                    kChar result[kPATH_MAX];

                    KCheck(kPath_ToRelative(pathAStr.CharPtr, pathBStr.CharPtr, result, kCountOf(result)));

                    return KToString(result);
                }

                /// <summary>Finds the absolute path expressed by the combination of an absolute path and a relative path.</summary>
                /// 
                /// <remarks>
                /// <para>If bRelativeToA is already expressed in absolute form, then the result will be an absolute path equal
                /// to bRelativeToA.</para>
                /// </remarks>
                /// 
                /// <param name="pathA">Absolute reference path.</param>
                /// <param name="bRelativeToA">Path b, expressed as relative to path A.</param>
                /// <returns>Path b, expressed as an absolute path.</returns>
                static String^ ToAbsolute(String^ pathA, String^ bRelativeToA)
                {
                    KString pathAStr(pathA);
                    KString bRelativeToAStr(bRelativeToA);
                    kChar result[kPATH_MAX];

                    KCheck(kPath_ToAbsolute(pathAStr.CharPtr, bRelativeToAStr.CharPtr, result, kCountOf(result)));

                    return KToString(result);
                }

                /// <summary>Transforms all path separators to normal form and removes trailing slashes.</summary>
                /// 
                /// <param name="path">Input path, in native or mixed form.</param>
                /// <returns>Path in normal form.</returns>
                static String^ ToNormal(String^ path)
                {
                    KString pathStr(path);
                    kChar result[kPATH_MAX];

                    KCheck(kPath_ToNormal(pathStr.CharPtr, result, kCountOf(result)));

                    return KToString(result);
                }

                /// <summary>Transforms all path separators to native form.</summary>
                /// 
                /// <param name="path">Input path, in normal or mixed form.</param>
                /// <returns>Path in native form.</returns>
                static String^ ToNative(String^ path)
                {
                    KString pathStr(path);
                    kChar result[kPATH_MAX];

                    KCheck(kPath_ToNative(pathStr.CharPtr, result, kCountOf(result)));

                    return KToString(result);
                }

                /// <summary>Transforms all path separators to normal form, canonicalizes the path by collapsing redundant separators and removes trailing slashes.</summary>
                /// 
                /// <param name="path">Input path, in normal or mixed form.</param>
                /// <returns>Path in native form.</returns>
                static String^ ToCanonical(String^ path)
                {
                    KString pathStr(path);
                    kChar result[kPATH_MAX];

                    KCheck(kPath_ToCanonical(pathStr.CharPtr, result, kCountOf(result)));

                    return KToString(result);
                }

            private:
                KPath() : KObject(DefaultRefStyle) {}
            };
        }
    }
}

#endif

// 
// KDynamicLib.h
// 
// Copyright (C) 2014-2021 by LMI Technologies Inc.
// Licensed under the MIT License.
// Redistributed files must retain the above copyright notice.
// 
#ifndef K_API_NET_DYNAMIC_LIB_H
#define K_API_NET_DYNAMIC_LIB_H

#include <kApi/Utils/kDynamicLib.h>
#include "kApiNet/KAlloc.h"
#include "kApiNet/Data/KString.h"

namespace Lmi3d
{
    namespace Zen 
    {
        namespace Utils
        {
            /// <summary>Represents a dynamically loaded library. <para/> Requires manual disposal.</summary>
            public ref class KDynamicLib : public KObject
            {
                KDeclareClass(KDynamicLib, kDynamicLib)

            public:
                /// <summary>Initializes a new instance of the KDynamicLib class with the specified Zen object handle.</summary>           
                /// <param name="handle">Zen object handle.</param>
                KDynamicLib(IntPtr handle)
                    : KObject(handle, DefaultRefStyle)
                {}

                /// <summary>Initializes a new instance of the KDynamicLib class.</summary>       
                /// 
                /// <param name="path">Path to the dynamic library.</param>
                KDynamicLib(String^ path)
                    : KObject(DefaultRefStyle)
                {
                    KString strPath(path);
                    kDynamicLib handle = kNULL;

                    KCheck(kDynamicLib_Construct(&handle, strPath.CharPtr, kNULL));

                    Handle = handle;
                }
                /// <inheritdoc cref="KDynamicLib(String^)" />
                /// <param name="allocator">Memory allocator.</param>
                KDynamicLib(String^ path, KAlloc^ allocator)
                    : KObject(DefaultRefStyle)
                {
                    KString strPath(path);
                    kDynamicLib handle = kNULL;

                    KCheck(kDynamicLib_Construct(&handle, strPath.CharPtr, KToHandle(allocator)));

                    Handle = handle;
                }

                /// <summary>Resolves a function pointer by name from the dynamic library.</summary>
                /// 
                /// <param name="name">Function name.</param>
                /// <returns>Function pointer, or null pointer if not found.</returns>
                IntPtr TryFindFunction(String^ name)
                {
                    KString strName(name); 
                    kFunction function = kNULL; 

                    if (kSuccess(kDynamicLib_FindFunction(Handle, strName.CharPtr, &function)))
                    {
                        return IntPtr(function); 
                    }
                    else
                    {
                        return IntPtr(0);
                    }
                }

                /// <summary>Resolves a function pointer by name from the dynamic library.</summary>
                /// 
                /// <param name="name">Function name.</param>
                /// <returns>Function pointer, or null pointer if not found.</returns>
                /// <exception cref="KException">Thrown if not found.</exception>
                IntPtr FindFunction(String^ name)
                {
                    KString strName(name);
                    kFunction function = kNULL;

                    KCheck(kDynamicLib_FindFunction(Handle, strName.CharPtr, &function));

                    return IntPtr(function);
                }
              
            protected:
                KDynamicLib() : KObject(DefaultRefStyle) {}
            };
        }
    }
}

#endif

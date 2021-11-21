// 
// KPlugin.h
// 
// Copyright (C) 2014-2021 by LMI Technologies Inc.
// Licensed under the MIT License.
// Redistributed files must retain the above copyright notice.
// 
#ifndef K_API_NET_PLUGIN_H
#define K_API_NET_PLUGIN_H

#include <kApi/Utils/kPlugin.h>
#include "kApiNet/KAlloc.h"
#include "kApiNet/KAssembly.h"
#include "kApiNet/Data/KString.h"

namespace Lmi3d
{
    namespace Zen 
    {
        namespace Utils
        {
            /// <summary>Represents a dynamically loaded plugin. <para/> Requires manual disposal.</summary>
            public ref class KPlugin : public KObject
            {
                KDeclareClass(KPlugin, kPlugin)

            public:
                /// <summary>Initializes a new instance of the KPlugin class with the specified Zen object handle.</summary>           
                /// <param name="handle">Zen object handle.</param>
                KPlugin(IntPtr handle)
                    : KObject(handle, DefaultRefStyle)
                {}

                /// <inheritdoc cref="KPlugin(IntPtr)" />
                ///
                /// <param name="refStyle">Ref style.</param>
                KPlugin(IntPtr handle, KRefStyle refStyle)
                    : KObject(handle, refStyle)
                {}

                /// <summary>Initializes a new instance of the KPlugin class.</summary>     
                /// 
                /// <param name="path">Path to the plugin library.</param>
                KPlugin(String^ path)
                    : KObject(DefaultRefStyle)
                {
                    KString strPath(path);
                    kPlugin handle = kNULL;

                    KCheck(kPlugin_Construct(&handle, strPath.CharPtr, kNULL));

                    Handle = handle;
                }

                /// <summary>Initializes a new instance of the KPlugin class.</summary>
                /// 
                /// <remarks>
                /// This constructor runs a platform version check before attempting to load the assembly. The requiredPlatform
                /// argument to this function is checked against the plaform version from the plugin's internal assembly.
                /// In case of a mismatch KStatus::ErrorVersion will be thrown.
                /// </remarks>
                /// 
                /// <param name="path">Path to the plugin library.</param>
                /// <param name="requiredPlatform">Required platform version.</param>
                KPlugin(String^ path, KVersion requiredPlatform)
                    : KObject(DefaultRefStyle)
                {
                    KString strPath(path);
                    kPlugin handle = kNULL;

                    KCheck(kPlugin_ConstructEx(&handle, strPath.CharPtr, requiredPlatform, kNULL, 0, kNULL, kNULL));

                    Handle = handle;
                }

                /// <inheritdoc cref="KPlugin(String^, KVersion)" />
                ///
                /// <param name="refStyle">Ref style.</param>
                KPlugin(String^ path, KVersion requiredPlatform, KRefStyle refStyle)
                    : KObject(DefaultRefStyle)
                {
                    KString strPath(path);
                    kPlugin handle = kNULL;

                    KCheck(kPlugin_ConstructEx(&handle, strPath.CharPtr, requiredPlatform, kNULL, 0, kNULL, kNULL));

                    Handle = handle;
                }

                /// <summary>Gets the assembly associated with this plugin.</summary>
                property KAssembly^ Assembly
                {
                    KAssembly^ get() { 
                        kAssembly assembly = kPlugin_Assembly(Handle);
                        if (RefStyle == KRefStyle::Auto) {
                            kObject_Share(assembly);
                        }

                        return KToObject<KAssembly^>(assembly, KRefStyle::None);
                    }
                }

            protected:
                KPlugin() : KObject(DefaultRefStyle) {}
            };

        }
    }
}

#endif

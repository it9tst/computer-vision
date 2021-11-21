// 
// KRoot.h
// 
// Copyright (C) 2014-2021 by LMI Technologies Inc.
// Licensed under the MIT License.
// Redistributed files must retain the above copyright notice.
// 
#ifndef K_API_NET_ROOT_H
#define K_API_NET_ROOT_H

#include "kApiNet/KAlloc.h"
#include "kApiNet/KAssembly.h"

#pragma warning(push)
#pragma warning(disable:4996)
#pragma warning(disable:4947)

namespace Lmi3d
{
    namespace Zen 
    {
        namespace Utils
        {
            /// <summary>This type is obsolete and will be removed in an upcoming release.</summary>
            generic <typename T> where T : KObject
            [Obsolete]
            public ref class KRoot
            {
            public:
                KRoot(T target)
                {
                    Target = target;
                }

                ~KRoot()
                {
                    if (Target)
                    {
                        Target->DisposeAll();
                        Target = T();
                    }
                }

                property T Target;
            };
        }
    }
}

#pragma warning(pop)

#endif

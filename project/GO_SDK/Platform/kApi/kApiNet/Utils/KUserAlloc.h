//
// KUserAlloc.h
// 
// Copyright (C) 2014-2021 by LMI Technologies Inc.
// Licensed under the MIT License.
// Redistributed files must retain the above copyright notice.
// 
#ifndef K_API_NET_USER_ALLOC_H
#define K_API_NET_USER_ALLOC_H

#include <kApi/Utils/kUserAlloc.h>
#include "kApiNet/KAlloc.h"
#include "kApiNet/KType.h"

namespace Lmi3d
{
    namespace Zen 
    {       
        namespace Utils
        {
            /// <summary>Allocates memory from a user-defined memory source. <para/> Requires manual disposal.</summary>
            /// 
            /// <remarks>
            /// KUserAlloc has no direct use in managed code; a type definition is provided here, but there are no 
            /// plans to implement any additional methods.
            /// </remarks>
            public ref class KUserAlloc : public KAlloc
            {
                KDeclareClass(KUserAlloc, kUserAlloc)

            public:
                /// <summary>Initializes a new instance of the KUserAlloc class with the specified Zen object handle.</summary>           
                /// <param name="handle">Zen object handle.</param>
                KUserAlloc(IntPtr handle)
                    : KAlloc(handle, DefaultRefStyle)
                {}

            protected:
                KUserAlloc() : KAlloc(DefaultRefStyle) {}
            };
        }
    }
}

#endif

//
// KObjectPool.h
// 
// Copyright (C) 2014-2021 by LMI Technologies Inc.
// Licensed under the MIT License.
// Redistributed files must retain the above copyright notice.
// 
#ifndef K_API_NET_OBJECT_POOL_H
#define K_API_NET_OBJECT_POOL_H

#include <kApi/Utils/kObjectPool.h>
#include "kApiNet/KObject.h"

namespace Lmi3d
{
    namespace Zen 
    {
        namespace Utils
        {
            /// <summary>Interface for a type that manages a pool of objects. <para/> Requires manual disposal.</summary>
            /// 
            /// <remarks>
            /// KObjectPool has no direct use in managed code; a type definition is provided here, but there are no 
            /// plans to implement any additional methods.
            /// </remarks>
            public ref class KObjectPool : public KObject
            {
                KDeclareInterface(KObjectPool, kObjectPool)

            private:
                KObjectPool() : KObject(DefaultRefStyle) {}
            };
         }
    }
}

#endif

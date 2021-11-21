//
// KValue.h
// 
// Copyright (C) 2014-2021 by LMI Technologies Inc.
// Licensed under the MIT License.
// Redistributed files must retain the above copyright notice.
//
#ifndef K_API_NET_VALUE_H
#define K_API_NET_VALUE_H

#include "kApiNet/KApiDef.h"
#include "kApiNet/KType.h"
#include <kApi/kValue.h>

namespace Lmi3d
{
    namespace Zen 
    {
        /// <summary>Root of zen value types.</summary>       
        ///
        /// <remarks>In the underlying native Zen API, the KValue type plays an important role in defining virtual
        /// methods that can be applied to any value instance. In the Zen.NET API, these services are exposed by 
        /// overriding virtual methods such as Equals and GetHashCode for Zen value types. Accordingly, the KValue
        /// type is of little use in the Zen.NET context.  It is included here for the sake of completeness, and 
        /// to provide a location to expose the KValue.KTypeId property, which may be helpful for native type 
        /// introspection.</remarks>
        ///
        public value struct KValue
        {
            static property Lmi3d::Zen::KType^ KTypeId
            {                                         
                KType^ get()  { return gcnew KType(kTypeOf(kValue)); }
            }
        };
    }
}

#endif

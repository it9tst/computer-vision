// 
// GoUtils.h
// 
// Copyright (C) 2016-2021 by LMI Technologies Inc.
// Licensed under the MIT License.
// Redistributed files must retain the above copyright notice.
// 
#ifndef GO_SDK_NET_UTILS_H
#define GO_SDK_NET_UTILS_H

#include <GoSdk/GoUtils.h>
#include <GoSdkNet/GoSdkDef.h>

namespace Lmi3d
{
    namespace GoSdk
    {
        /// <summary>Contains various helper functions. </summary>
        public ref class GoUtils
        {
        public:
            /// <summary>Returns the result of a floating point number equivalence, based on a given degree of precision.</summary>
            /// <param name="first">k64f object.</param>
            /// <param name="second">k64f object.</param>
            /// <param name="decimalPrecision">k8u object.</param>
            /// <returns>Returns true if the numbers are equivalent within the given precision; false otherwise.</returns>
            static bool FuzzyEquivalence(k64f first, k64f second, k8u decimalPrecision)
            {
               return KToBool(GoUtils_FuzzyEquivalence(first, second, decimalPrecision));
            }
        };
    }
}

#endif

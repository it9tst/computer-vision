// 
// KSymbolInfo.h
// 
// Copyright (C) 2016-2021 by LMI Technologies Inc.
// Licensed under the MIT License.
// Redistributed files must retain the above copyright notice.
// 
#ifndef K_API_NET_SYMBOL_INFO_H
#define K_API_NET_SYMBOL_INFO_H

#include <kApi/Utils/kSymbolInfo.h>
#include "kApiNet/KAlloc.h"
#include "kApiNet/Data/KString.h"

namespace Lmi3d
{
    namespace Zen 
    {
        namespace Utils
        {
            /// <summary>Collection of static functions related to debug symbol information.</summary>
            ///
            /// <remarks>
            /// <para>Default KRefStyle: None</para>
            /// </remarks>
            public ref class KSymbolInfo : public KObject
            {
                KDeclareNoneClass(KSymbolInfo, kSymbolInfo)

            public:

                /// <summary>Describes the function at the specified address.</summary>
                /// 
                /// <remarks>The string returned by this function should be disposed by the caller.</remarks>
                ///
                /// <param name="function">Function call address.</param>
                /// <returns>Function call description.</returns>
                static KString^ DescribeFunction(IntPtr function)
                {
                    kString line = kNULL; 

                    KCheck(kSymbolInfo_DescribeFunction(function.ToPointer(), &line)); 

                    return gcnew KString(IntPtr(line)); 
                }

            private:
                KSymbolInfo(KRefStyle refStyle) : KObject(refStyle){}
            };

        }
    }
}

#endif

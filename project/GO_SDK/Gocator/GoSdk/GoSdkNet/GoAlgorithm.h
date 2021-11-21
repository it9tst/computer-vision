// 
// GoAlgorithm.h
// 
// Copyright (C) 2017-2021 by LMI Technologies Inc.
// Licensed under the MIT License.
// Redistributed files must retain the above copyright notice.
// 
#ifndef GO_SDK_NET_ALGORITHM_H
#define GO_SDK_NET_ALGORITHM_H

#include <GoSdk/GoAlgorithm.h>
#include <GoSdkNet/GoSdkDef.h>

namespace Lmi3d
{
    namespace GoSdk
    {
        /// <summary>Contains various Algorithmic helper functions. </summary>
        public ref class GoAlgorithm
        {
        public:
            /// <summary>Performs a demosaic operation on a bayer encoded image.</summary>
            /// <param name="input">Bayer Encoded KImage</param>
            /// <param name="style">GoDemosaicStyle</param>
            /// <param name="allocator">Allocator</param>
            /// <returns>Decoded (Color) KImage</returns>
            static KImage^ Demosaic(KImage^ input, GoDemosaicStyle style, KAlloc^ allocator)
            {
                kImage output = kNULL;
                KCheck(::GoAlgorithm_Demosaic(KToHandle(input), &output, style, KToHandle(allocator)));

                return KToObject<KImage^>(output);
            }
        };
    }
}

#endif

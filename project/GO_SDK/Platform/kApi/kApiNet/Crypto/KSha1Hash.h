// 
// KSha1Hash.h
// 
// Copyright (C) 2017-2021 by LMI Technologies Inc. All rights reserved.
// 
#ifndef K_API_NET_SHA1_HASH_H
#define K_API_NET_SHA1_HASH_H

#include "kApi/Crypto/kSha1Hash.h"
#include "kApiNet/KApiDef.h"
#include "kApiNet/Crypto/KHash.h"

namespace Lmi3d
{
    namespace Zen
    {
        namespace Crypto
        {
            /// <summary>Sha1 hash implementation.</summary>
            public ref class KSha1Hash : public KHash
            {
                KDeclareAutoClass(KSha1Hash, kSha1Hash)

            public:
                /// <summary>Initializes a new instance of the KSha1Hash class with the specified Zen object handle.</summary>           
                /// <param name="handle">Zen object handle.</param>
                KSha1Hash(IntPtr handle)
                    : KHash(handle)
                {}

                /// <inheritdoc cref="KSha1Hash(IntPtr)" />
                ///
                /// <param name="refStyle">RefStyle for this object.</param>
                KSha1Hash(IntPtr handle, KRefStyle refStyle)
                    : KHash(handle, refStyle)
                {}

                /// <summary>Constructs a KSha1Hash instance.</summary>
                KSha1Hash()
                {
                    kSha1Hash handle = kNULL;

                    KCheck(kSha1Hash_Construct(&handle, kNULL));

                    Handle = handle;
                }
            };
        }
    }
}


#endif
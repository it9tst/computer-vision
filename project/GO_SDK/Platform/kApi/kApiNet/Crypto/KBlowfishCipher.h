// 
// KBlowfishCipher.h
// 
// Copyright (C) 2016-2021 by LMI Technologies Inc. All rights reserved.
// 
#ifndef K_API_NET_BLOWFISH_CIPHER_H
#define K_API_NET_BLOWFISH_CIPHER_H

#include <kApi/Crypto/kBlowfishCipher.h>
#include "kApiNet/KApiDef.h"
#include "kApiNet/Crypto/KCipher.h"

namespace Lmi3d
{
    namespace Zen
    {
        namespace Crypto
        {
            /// <summary>Blowfish cipher implementation.</summary>
            public ref class KBlowfishCipher : public KCipher
            {
                KDeclareAutoClass(KBlowfishCipher, kBlowfishCipher)

            public:
                /// <summary>Initializes a new instance of the KBlowfishCipher class with the specified Zen object handle.</summary>           
                /// <param name="handle">Zen object handle.</param>
                KBlowfishCipher(IntPtr handle)
                    : KCipher(handle)
                {}

                /// <inheritdoc cref="KBlowfishCipher(IntPtr)" />
                ///
                /// <param name="refStyle">RefStyle for this object.</param>
                KBlowfishCipher(IntPtr handle, KRefStyle refStyle)
                    : KCipher(handle, refStyle)
                {}

                /// <summary>Constructs a kBlowfishCipher instance.</summary>
                /// 
                /// <param name="key">Key.</param>
                /// <param name="padding">Padding used in encryption.</param>
                /// <param name="mode">Cipher mode used for encryption and decryption.</param>
                KBlowfishCipher(array<Byte>^ key, KCipherPadding padding, KCipherMode mode)
                {
                    kBlowfishCipher handle = kNULL;
                    pin_ptr<Byte> keyPin = &(key[0]);

                    KCheck(kBlowfishCipher_Construct(&handle, keyPin, key->Length, padding, mode, kNULL));

                    Handle = handle;
                }
            };
        }
    }
}


#endif
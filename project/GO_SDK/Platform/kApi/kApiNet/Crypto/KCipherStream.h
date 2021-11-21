// 
// KCipherStream.h
// 
// Copyright (C) 2016-2021 by LMI Technologies Inc. All rights reserved.
// 
#ifndef K_API_NET_CIPHER_STREAM_H
#define K_API_NET_CIPHER_STREAM_H

#include <kApi/Crypto/kCipherStream.h>
#include "kApiNet/KApiDef.h"
#include "kApiNet/Crypto/KCipher.h"
#include "kApiNet/Io/KStream.h"

namespace Lmi3d
{
    namespace Zen
    {
        namespace Crypto
        {
            /// <summary>Supports streaming encryption or decryption. </summary>
            public ref class KCipherStream : public KStream
            {
                KDeclareAutoClass(KCipherStream, kCipherStream)

            public:
                /// <summary>Initializes a new instance of the KCipherStream class with the specified Zen object handle.</summary>           
                /// <param name="handle">Zen object handle.</param>
                KCipherStream(IntPtr handle)
                    : KStream(handle, DefaultRefStyle)
                {}

                /// <inheritdoc cref="KCipherStream(IntPtr)" />
                ///
                /// <param name="refStyle">RefStyle for this object.</param>
                KCipherStream(IntPtr handle, KRefStyle refStyle)
                    : KStream(handle, refStyle)
                {}

                /// <summary>Constructs a kCipherStream object.</summary>
                /// 
                /// <param name="stream">Stream used for the underlying read/write functions.</param>
                /// <param name="cipher">Cipher used for encyption and decryption.</param>
                KCipherStream(KStream^ stream, KCipher^ cipher)
                {
                    kCipherStream handle = kNULL;

                    KCheck(kCipherStream_Construct(&handle, KToHandle(stream), KToHandle(cipher), kNULL));

                    Handle = handle;
                }

                /// <summary>Flushes any pending data. </summary>
                void FlushFinal()
                {
                    KCheck(kCipherStream_FlushFinal(Handle));
                }
            };
        }
    }
}


#endif
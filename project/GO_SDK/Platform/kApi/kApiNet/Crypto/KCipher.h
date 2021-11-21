// 
// KCipher.h
// 
// Copyright (C) 2016-2021 by LMI Technologies Inc. All rights reserved.
// 
#ifndef K_FIRESYNC_NET_CIPHER_H
#define K_FIRESYNC_NET_CIPHER_H

#include <kApi/Crypto/kCipher.h>
#include <kApiNet/KApiNet.h>
#include "kApiNet/KApiDef.h"
#include "kApiNet/Data/KArray1.h"

namespace Lmi3d
{
    namespace Zen
    {
        namespace Crypto
        {
            /// <summary>Represents padding mode for encryption.</summary>
            public value struct KCipherPadding
            {
                KDeclareEnum(KCipherPadding, kCipherPadding)

                /// <summary>Unknown.</summary>
                literal k32s Null = kCIPHER_PADDING_NULL;

                /// <summary>ANSIX923, padding string consists of a sequence of bytes filled with zeros before the length.</summary>
                literal k32s Ansix923 = kCIPHER_PADDING_ANSIX923;

                /// <summary>ISO10126, padding string consists of random data before the length.</summary>
                literal k32s Iso10126 = kCIPHER_PADDING_ISO10126;

                /// <summary>No padding, requires that the encrypted string is a multiple of 8 bytes long.</summary>
                literal k32s None = kCIPHER_PADDING_NONE;

                /// <summary>PKCS7, padding string consists of a sequence of bytes, each of which is equal to the total number of padding bytes added.</summary>
                literal k32s Pkcs7 = kCIPHER_PADDING_PKCS7;

                /// <summary>Zero, padding string consists of bytes set to zero.</summary>
                literal k32s Zero = kCIPHER_PADDING_ZERO;
            };

            /// <summary>Represents cipher mode for encryption and decryption.</summary>
            public value struct KCipherMode
            {
                KDeclareEnum(KCipherMode, kCipherMode)

                /// <summary>Unknown.</summary>
                literal k32s Null = kCIPHER_CIPHER_NULL;

                /// <summary>Electronic Cook Book, each block is encrypted independently.</summary>
                literal k32s ECB = kCIPHER_CIPHER_ECB;
            };

            /// <summary>Abstract base class for symmetric-key encryption classes.</summary>
            public ref class KCipher : public KObject
            {
                KDeclareAutoClass(KCipher, kCipher)

            public:
                /// <summary>Initializes a new instance of the KCipher class with the specified Zen object handle.</summary>           
                /// <param name="handle">Zen object handle.</param>
                KCipher(IntPtr handle)
                    : KObject(handle)
                {}

                /// <inheritdoc cref="KCipher(IntPtr)" />
                ///
                /// <param name="refStyle">RefStyle for this object.</param>
                KCipher(IntPtr handle, KRefStyle refStyle)
                    : KObject(handle, refStyle)
                {}

                /// <summary>Encrypts data.</summary>
                /// 
                /// <remarks>Block ciphers operate on fixed block sizes, so padding may be added. 
                /// The type of padding depends on settings provided to the cipher object.
                /// It is the caller's responsible to remove any padding from decrypted data. </remarks>
                /// 
                /// <param name="data">Data to encrypt.</param>
                /// <returns>Encrypted data.</returns>
                array<Byte>^ Encrypt(array<Byte>^ data)
                {
                    pin_ptr<unsigned char> dataPin = &data[0];
                    KArray1 result;

                    KCheck(kCipher_Encrypt(Handle, dataPin, data->Length, result.ToHandle().ToPointer()));

                    array<Byte>^ resultArray = gcnew array<Byte>((k32s)result.DataSize);
                    System::Runtime::InteropServices::Marshal::Copy(result.Data, resultArray, 0, (k32s)result.DataSize);

                    return resultArray;
                }

                /// <summary>Decrypts data.</summary>
                ///
                /// <remarks>When decrypting data, it is necessary to use the same cipher mode that was used to encrypt the data. 
                /// It is the caller's responsibility to remove any padding that was added during encryption. </remarks>
                /// 
                /// <param name="data">Data to decrypt.</param>
                /// <returns>Decrypted data.</returns>
                array<Byte>^ Decrypt(array<Byte>^ data)
                {
                    pin_ptr<unsigned char> dataPin = &data[0];
                    KArray1 result;

                    KCheck(kCipher_Decrypt(Handle, dataPin, data->Length, result.ToHandle().ToPointer()));

                    array<Byte>^ resultArray = gcnew array<Byte>((k32s)result.DataSize);
                    System::Runtime::InteropServices::Marshal::Copy(result.Data, resultArray, 0, (k32s)result.DataSize );

                    return resultArray;
                }

                property KCipherPadding Padding
                {
                    /// <summary>Gets the pattern mode.</summary>
                    /// 
                    /// <returns>Pattern mode.</returns>
                    KCipherPadding get()
                    {
                        return kCipher_Padding(Handle);
                    }
                }

                property KCipherMode Mode
                {
                    /// <summary>Gets the cipher mode.</summary>
                    /// 
                    /// <returns>Cipher mode.</returns>
                    KCipherMode get()
                    {
                        return kCipher_Mode(Handle);
                    }
                }

                property k64s Blocksize
                {
                    /// <summary>Gets the block size.</summary>
                    /// 
                    /// <returns>Cipher block size..</returns>                    
                    k64s get()
                    {
                        return kCipher_Blocksize(Handle);
                    }
                }

            protected:
                KCipher() {}
            };
        }
    }
}

#endif
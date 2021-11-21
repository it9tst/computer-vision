// 
// KHash.h
// 
// Copyright (C) 2017-2021 by LMI Technologies Inc. All rights reserved.
// 
#ifndef K_API_NET_HASH_H
#define K_API_NET_HASH_H

#include <kApi/Crypto/kHash.h>

namespace Lmi3d
{
    namespace Zen
    {
        namespace Crypto
        {
            /// <summary>Abstract base class providing hash functionality.</summary>
            public ref class KHash : public KObject
            {
                KDeclareAutoClass(KHash, kHash)

            public:
                /// <summary>Initializes a new instance of the KHash class with the specified Zen object handle.</summary>           
                /// <param name="handle">Zen object handle.</param>
                KHash(IntPtr handle)
                    : KObject(handle)
                {}

                /// <inheritdoc cref="KHash(IntPtr)" />
                ///
                /// <param name="refStyle">RefStyle for this object.</param>
                KHash(IntPtr handle, KRefStyle refStyle)
                    : KObject(handle, refStyle)
                {}

                /// <summary>Updates the hash with the data.</summary>
                /// 
                /// <remarks>
                /// Repeated calls are equivalent to a single call with the concatenation 
                /// of all the arguments.
                /// </remarks>
                /// <param name="buffer">Buffer containing the data.</param>
                /// <param name="size">Size of the buffer, in bytes.</param>
                void Update(IntPtr buffer, k64s size)
                {
                    KCheck(kHash_Update(Handle, buffer.ToPointer(), (kSize)size));
                }

                /// <inheritdoc cref="Update(IntPtr, k64s)" />
                void Update(array<kByte>^ buffer)
                {
                    pin_ptr<Byte> pinnedBuffer = &buffer[0];

                    Update(IntPtr(pinnedBuffer), buffer->Length);
                }

                /// <summary>Returns the digest.</summary>
                /// 
                /// <remarks>
                /// The size of the buffer in bytes must be at least kHash_DigestSize(). A call of
                /// this function does not change the internal buffers.
                /// </remarks>
                /// <param name="buffer">Receives the digest for the data.</param>
                /// <param name="size">Size of the buffer, in bytes.</param>
                void DigestArray(IntPtr buffer, k64s size)
                {
                    KCheck(kHash_Digest(Handle, buffer.ToPointer(), (kSize)size));
                }

                /// <inheritdoc cref="DigestArray(IntPtr, k64s)" />
                void DigestArray(array<kByte>^ buffer)
                {
                    pin_ptr<Byte> pinnedBuffer = &buffer[0];

                    DigestArray(IntPtr(pinnedBuffer), buffer->Length);
                }

                /// <summary>Returns the digest.</summary>
                property array<Byte>^ Digest
                {
                    array<Byte>^ get()
                    {
                        array<Byte>^ buffer = gcnew array<Byte>((k32s)DigestSize);

                        DigestArray(buffer);

                        return buffer;
                    }
                }

                /// <summary>Clears the internal buffer so instance can be updated with new data.</summary>
                void Clear()
                {
                    KCheck(kHash_Clear(Handle));
                }

                /// <summary>Returns the message digest length in bytes.</summary>
                property kSize DigestSize
                {
                    kSize get() {return kHash_DigestSize(Handle);}
                }

            protected:
                KHash() {}
            };
        }
    }
}

#endif
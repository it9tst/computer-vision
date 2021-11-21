// 
// KMemory.h
// 
// Copyright (C) 2014-2021 by LMI Technologies Inc.
// Licensed under the MIT License.
// Redistributed files must retain the above copyright notice.
// 
#ifndef K_API_NET_MEMORY_H
#define K_API_NET_MEMORY_H

#include <kApi/Io/kMemory.h>
#include "kApiNet/KAlloc.h"
#include "kApiNet/Io/KStream.h"

namespace Lmi3d
{
    namespace Zen 
    {
        namespace Io
        {
            /// <summary>Represents an in-memory stream. <para/> Requires manual disposal.</summary>
            public ref class KMemory : public KStream
            {
                KDeclareClass(KMemory, kMemory)

            public:
                /// <summary>Initializes a new instance of the KMemory class with the specified Zen object handle.</summary>           
                /// <param name="handle">Zen object handle.</param>
                KMemory(IntPtr handle)
                    : KStream(handle, DefaultRefStyle)
                {}

                /// <inheritdoc cref="KMemory(IntPtr)" />
                ///
                /// <param name="refStyle">RefStyle for this object.</param>
                KMemory(IntPtr handle, KRefStyle refStyle)
                    : KStream(handle, refStyle)
                {}

                /// <summary>Constructs a kMemory object.</summary>
                /// 
                /// <remarks>
                /// By default, an auto-sizing, internally-managed buffer is used.
                /// </remarks>
                KMemory()
                    : KStream(DefaultRefStyle)
                {
                    kMemory handle = kNULL;

                    KCheck(kMemory_Construct(&handle, kNULL));

                    Handle = handle;
                }

                /// <inheritdoc cref="KMemory()" />
                ///
                /// <param name="allocator">Memory allocator</param>
                KMemory(KAlloc^ allocator)
                    : KStream(DefaultRefStyle)
                {
                    kMemory handle = kNULL;

                    KCheck(kMemory_Construct(&handle, KToHandle(allocator)));

                    Handle = handle;
                }

                /// <inheritdoc cref="KMemory(KAlloc^)" />
                ///
                /// <param name="refStyle">RefStyle for this object.</param>
                KMemory(KAlloc^ allocator, KRefStyle refStyle)
                    : KStream(refStyle)
                {
                    kMemory handle = kNULL;

                    KCheck(kMemory_Construct(&handle, KToHandle(allocator)));

                    Handle = handle;
                }

                /// <summary>Attaches the memory stream to an external, fixed-capacity buffer.</summary>
                /// 
                /// <param name="buffer">Data buffer.</param>
                /// <param name="position">Current read/write position, relative to start of buffer.</param>
                /// <param name="length">Length of valid data contained in the buffer, in bytes.</param>
                void Attach(array<Byte>^ buffer, k64s position, k64s length)
                {
                    pin_ptr<Byte> pointer = &(buffer[0]);
                    kByte* nativePointer = (kByte*)pointer;

                    KCheck(kMemory_Attach(Handle, nativePointer, (kSize)position, (kSize)length, (kSize)buffer->Length)); 
                }

                /// <summary>Attaches the memory stream to an external, fixed-capacity buffer.</summary>
                /// 
                /// <param name="buffer">Data buffer.</param>
                /// <param name="position">Current read/write position, relative to start of buffer.</param>
                /// <param name="length">Length of valid data contained in the buffer, in bytes.</param>
                /// <param name="capacity">Total size of the buffer, in bytes.</param>
                void Attach(IntPtr buffer, k64s position, k64s length, k64s capacity)
                {
                    KCheck(kMemory_Attach(Handle, buffer.ToPointer(), (kSize)position, (kSize)length, (kSize)capacity)); 
                }

                /// <summary>Allocates an auto-sizing, internally-managed buffer for the memory stream.</summary>
                /// 
                /// <param name="initialCapacity">Initial capacity of the buffer, in bytes.</param>
                void Allocate(k64s initialCapacity)
                {
                    KCheck(kMemory_Allocate(Handle, (kSize)initialCapacity)); 
                }

                /// <summary>Returns the current length of the memory buffer.</summary>
                property k64s Length
                {
                    k64s get() { return (k64s) kMemory_Length(Handle); }
                }

                /// <summary>Returns the current position of the read/write pointer, relative to the beginning of the buffer.</summary>
                property k64s Position
                {
                    k64s get() { return (k64s)kMemory_Position(Handle); }
                }

                /// <summary>Returns the current capacity of the memory buffer.</summary>
                property k64s Capacity
                {
                    k64s get() { return (k64s)kMemory_Capacity(Handle); }
                }

                /// <summary>Sets the reported length of the memory buffer.</summary>
                /// 
                /// <param name="length">Buffer length.</param>
                void SetLength(k64s length)
                {
                    KCheck(kMemory_SetLength(Handle, (kSize)length)); 
                }

                /// <summary>Returns a pointer to the internal memory buffer.</summary>
                property IntPtr Data
                {
                    IntPtr get() { return IntPtr(kMemory_At(Handle, 0)); }
                }

                /// <summary>Returns a pointer to the specified position within the memory buffer.</summary>
                IntPtr GetDataAt(k64s offset)
                {
                    return IntPtr(kMemory_At(Handle, (kSize)offset)); 
                }
            };
        }
    }
}

#endif

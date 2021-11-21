// 
// KStream.h
// 
// Copyright (C) 2014-2021 by LMI Technologies Inc.
// Licensed under the MIT License.
// Redistributed files must retain the above copyright notice.
// 
#ifndef K_API_NET_STREAM_H
#define K_API_NET_STREAM_H

#include <kApi/Io/kStream.h>
#include "kApiNet/KAlloc.h"

namespace Lmi3d
{
    namespace Zen 
    {
        namespace Io
        {
            kStatus kCall KStream_ProgressChanged(kPointer receiver, kPointer sender, kPointer args);

            /// <summary>Base class for I/O streams. <para/> Requires manual disposal.</summary>
            public ref class KStream abstract : public KObject
            {
                KDeclareClass(KStream, kStream)

            public:
                /// Reads the specified number of bytes from the stream.
                /// 
                /// <param name="buffer">Destination for bytes that are read.</param>
                /// <param name="count">Count of bytes to read.</param>
                void Read(IntPtr buffer, k64s count)
                {
                    KCheck(kStream_Read(Handle, buffer.ToPointer(), (kSize)count));
                }
                
                /// <inheritdoc cref="Read(IntPtr, k64s)" />
                /// 
                /// <param name="start">Offset into buffer.</param>
                void Read(array<kByte>^ buffer, k32s start, k32s count)
                {
                    pin_ptr<kByte> bytes = &buffer[start];

                    KCheck(kStream_Read(Handle, bytes, (kSize)count));
                }

                /// <summary>Reads up to the specified number of bytes from the stream.</summary>
                /// 
                /// <param name="buffer">Destination for bytes that are read.</param>
                /// <param name="minCount">Minimum count of bytes to read.</param>
                /// <param name="maxCount">Maximum count of bytes to read.</param>
                /// <returns>count of bytes read.</returns>
                k64s ReadSome(IntPtr buffer, k64s minCount, k64s maxCount)
                {
                    kSize bytesRead = 0;

                    KCheck(kStream_ReadSome(Handle, buffer.ToPointer(), (kSize)minCount, (kSize)maxCount, &bytesRead));

                    return (k64s)bytesRead;
                }

                /// <inheritdoc cref="ReadSome(IntPtr, k64s, k64s)" />
                /// 
                /// <param name="start">Offset into buffer.</param>
                k32s ReadSome(array<kByte>^ buffer, k32s start, k32s minCount, k32s maxCount)
                {
                    pin_ptr<kByte> bytes = &buffer[start];

                    return (k32s) ReadSome(IntPtr(bytes), minCount, maxCount);
                }

                /// <summary>Writes the specified number of bytes to the stream.</summary>
                /// 
                /// <param name="buffer">Bytes to be written to the stream.</param>
                /// <param name="count">Count of bytes to be written.</param>
                void Write(IntPtr buffer, k64s count)
                {
                    KCheck(kStream_Write(Handle, buffer.ToPointer(), (kSize)count));
                }

                /// <inheritdoc cref="Write(IntPtr, k64s)" />
                /// 
                /// <param name="start">Offset into buffer.</param>
                void Write(array<kByte>^ buffer, k32s start, k32s count)
                {
                    pin_ptr<kByte> bytes = &buffer[start];

                    KCheck(kStream_Write(Handle, bytes, (kSize)count));
                }

                /// <summary>Copies the specified number of bytes from one stream to another.</summary>
                /// 
                /// <param name="source">Source stream.</param>
                /// <param name="count">Count of bytes to be copied.</param>
                void Copy(KStream^ source, k64s count)
                {
                    KCheck(kStream_Copy(Handle, KToHandle(source), (kSize)count));
                }

                /// <summary>Copies the specified number of bytes from one stream to another, with progress updates.</summary>
                /// 
                /// <remarks>
                /// <para>The specified handler will be invoked to provide feedback on the progress of the operation. The argument to the 
                /// handler receives the percentage completed. The progress handler is guaranteed to be called at least once if the operation is 
                /// successful, with a progress value of 100%. </para>
                ///
                /// <para>This method blocks until the copy is complete.</para>
                /// </remarks>
                ///
                /// <param name="source">Source stream.</param>
                /// <param name="count">Count of bytes to be copied.</param>
                /// <param name="progressHandler">Progress handler.</param>
                void Copy(KStream^ source, k64s count, Action<KStream^, k32s>^ progressHandler)
                {
                    KCallbackFx^ thunk = gcnew KCallbackFx(&KStream::OnProgressUpdate);
                    KCallbackState context(thunk, progressHandler);

                    KCheck(kStream_CopyEx(Handle, KToHandle(source), (kSize)count, (kCallbackFx)context.NativeFunction, context.NativeContext));
                }
 
                /// <summary>Moves the read/write pointer to the specified location, if supported by the underlying stream.</summary>
                /// 
                /// <param name="offset">Offset by which to adjust the read/write pointer.</param>
                /// <param name="origin">Origin to which movement is relative (i.e. begin, current, end).</param>
                void Seek(k64s offset, KSeekOrigin origin)
                {
                    KCheck(kStream_Seek(Handle, offset, origin));
                }

                /// <summary>Flushes buffered writes to the underlying medium.</summary>
                void Flush()
                {
                    KCheck(kStream_Flush(Handle));
                }

                /// <summary>Clears stream statistics (e.g., BytesRead, BytesWritten).</summary>
                void ClearStats()
                {
                    kStream_ClearStats(Handle);
                }

                /// <summary>Gets the number of bytes read from this stream.</summary>
                property k64s BytesRead 
                { 
                    k64s get() { return kStream_BytesRead(Handle); }
                }

                /// <summary>Gets the number of bytes written to this stream.</summary>
                property k64s BytesWritten
                { 
                    k64s get() { return kStream_BytesWritten(Handle); }
                }

            internal:
                /// <summary>Initializes a new instance of the KStream class with the specified Zen object handle.</summary>           
                /// <param name="handle">Zen object handle.</param>
                KStream(IntPtr handle, KRefStyle refStyle)
                    : KObject(handle, refStyle)
                {}

            protected:
                KStream() : KObject() {}
                KStream(KRefStyle refStyle) : KObject(refStyle) {}
                
                static kStatus OnProgressUpdate(kPointer receiver, kPointer sender, kPointer args)
                {
                    kStatus status = kOK;

                    try
                    {
                        KCallbackState^ context = KCallbackState::FromNativeContext(receiver);
                        Action<KStream^, k32s>^ handler = (Action<KStream^, k32s>^) context->Handler;
                        KStream^ from = KToObject<KStream^>(sender);
                        k32s progress = kPointer_ReadAs(args, k32s);

                        handler(from, progress);
                    }
                    catch (KException^ e)
                    {
                        status = e->Status;
                    }
                    catch (...)
                    {
                        status = kERROR;
                    }

                    return status;
                }
            };
        }
    }
}

#endif

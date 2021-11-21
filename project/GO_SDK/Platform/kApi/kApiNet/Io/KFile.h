//
// KFile.h
// 
// Copyright (C) 2014-2021 by LMI Technologies Inc.
// Licensed under the MIT License.
// Redistributed files must retain the above copyright notice.
// 
#ifndef K_API_NET_FILE_H
#define K_API_NET_FILE_H

#include <kApi/Io/kFile.h>
#include <kApi/Io/kPath.h>
#include "kApiNet/Data/KString.h"
#include "kApiNet/Io/KStream.h"

namespace Lmi3d
{
    namespace Zen
    {
        namespace Io
        {
            kStatus kCall KFile_ProgressChanged(kPointer receiver, kPointer sender, kPointer args); 

            /// <summary>Represents a file stream. <para/> Requires manual disposal.</summary>
            public ref class KFile : public KStream
            {
                KDeclareClass(KFile, kFile)

            public:

                /// <summary>Initializes a new instance of the KFile class with the specified Zen object handle.</summary>           
                /// <param name="handle">Zen object handle.</param>
                KFile(IntPtr handle)
                    : KStream(handle, DefaultRefStyle)
                {}

                /// <summary>Initializes a new instance of the KFile class with the specified file path and mode.</summary>       
                /// 
                /// <param name="path">Path to the file.</param>
                /// <param name="mode">Specifies how to open the file.</param>
                KFile(String^ path, KFileMode mode)
                     : KStream(DefaultRefStyle)
                {
                    KString pathStr(path);
                    kFile handle = kNULL;

                    KCheck(kFile_Construct(&handle, pathStr.CharPtr, mode, kNULL));

                    Handle = handle;
                }

                /// <summary>Initializes a new instance of the KFile class with the specified file path and mode.</summary>       
                /// 
                /// <param name="path">Path to the file.</param>
                /// <param name="mode">Specifies how to open the file.</param>
                /// <param name="allocator">Memory allocator.</param>
                KFile(String^ path, KFileMode mode, KAlloc^ allocator)
                     : KStream(DefaultRefStyle)
                {
                    KString pathStr(path); 
                    kFile handle = kNULL;
                    
                    KCheck(kFile_Construct(&handle, pathStr.CharPtr, mode, KToHandle(allocator)));

                    Handle = handle;
                }
                
                /// <summary>Reads the specified file and provides the file contents in an array.</summary>
                /// 
                /// <param name="path">Path to the file.</param>
                /// <returns>File contents.</returns>
                static array<kByte>^ Load(String^ path)
                {
                    KFile file(path, KFileMode::Read); 
                    k64s fileSize = file.Length; 

                    KCheckErr(fileSize < k32S_MAX, kERROR_INCOMPLETE); 

                    array<kByte>^ output = gcnew array<kByte>((k32s)fileSize);

                    file.ReadBufferSize = (1 << 16);
                    file.Read(output, 0, (k32s)fileSize);

                    return output;
                }

                /// <summary>Saves the specified data to file.</summary>
                /// 
                /// <param name="path">Path to file.</param>
                /// <param name="data">Pointer to the file contents.</param>
                /// <param name="size">Size of the file contents.</param>
                static void Save(String^ path, IntPtr data, k64s size)
                {
                    KString pathStr(path);

                    KCheck(kFile_Save(pathStr.CharPtr, (kByte*)data.ToPointer(), (kSize)size));
                }

                /// <summary>Saves the specified data to file.</summary>
                /// 
                /// <param name="path">Path to file.</param>
                /// <param name="data">File data.</param>
                static void Save(String^ path, array<kByte>^ data)
                {
                    pin_ptr<kByte> dataPtr = &data[0];

                    Save(path, IntPtr(dataPtr), data->LongLength);
                }

                /// <summary>Reports whether the specified file exists.</summary>
                /// 
                /// <param name="path">Path to file.</param>
                /// <returns>true if the file exists; false otherwise.</returns>
                static bool Exists(String^ path)
                {
                    KString pathStr(path);

                    return KToBool(kFile_Exists(pathStr.CharPtr));
                }

                /// <summary>Reports the size the specified file, in bytes.</summary>
                /// 
                /// <param name="path">Path to file.</param>
                /// <returns>File size.</returns>
                static k64s GetSize(String^ path)
                {
                    KString pathStr(path);

                    return kFile_Size(pathStr.CharPtr);
                }

                /// <summary>Copies a file to the specified destination.</summary>
                /// 
                /// <param name="source">Source file path.</param>
                /// <param name="destination">Destination file path.</param>
                static void Copy(String^ source, String^ destination)
                {
                    KString srcStr(source);
                    KString dstStr(destination);

                    KCheck(kFile_Copy(srcStr.CharPtr, dstStr.CharPtr));
                }

                /// <inheritdoc cref="Copy(String^, String^)" />
                /// <summary>Copies a file to the specified destination, with progress updates.</summary>
                /// 
                /// <remarks>
                /// <para>The specified handler will be invoked to provide feedback on the progress of the operation. The argument to the 
                /// handler receives the percentage completed. The progress handler is guaranteed to be called at least once if the operation is 
                /// successful, with a progress value of 100%. </para>
                ///
                /// <para>This method blocks until the copy is complete.</para>
                /// </remarks>
                ///
                /// <param name="progressHandler">Progress update handler.</param>
                static void Copy(String^ source, String^ destination, Action<KStream^, k32s>^ progressHandler)
                {
                    KString srcStr(source);
                    KString dstStr(destination);
                    KCallbackFx^ thunk = gcnew KCallbackFx(&KStream::OnProgressUpdate);
                    KCallbackState context(thunk, progressHandler);
                    
                    KCheck(kFile_CopyEx(srcStr.CharPtr, dstStr.CharPtr, (kCallbackFx)context.NativeFunction, context.NativeContext));
                }

                /// <summary>Moves a file to the specified destination.</summary>
                /// 
                /// <param name="source">Source file path.</param>
                /// <param name="destination">Destination file path.</param>
                static void Move(String^ source, String^ destination)
                {
                    KString srcStr(source);
                    KString dstStr(destination);

                    KCheck(kFile_Move(srcStr.CharPtr, dstStr.CharPtr));
                }

                /// <inheritdoc cref="Move(String^, String^)" />
                /// <summary>Moves a file to the specified destination, with progress updates.</summary>
                /// 
                /// <remarks>
                /// <para>The specified handler will be invoked to provide feedback on the progress of the operation. The argument to the 
                /// handler receives the percentage completed. The progress handler is guaranteed to be called at least once if the operation is 
                /// successful, with a progress value of 100%. </para>
                ///
                /// <para>This method blocks until the copy is complete.</para>
                /// </remarks>
                ///
                /// <param name="progressHandler">Progress update handler.</param>
                static void Move(String^ source, String^ destination, Action<KStream^, k32s>^ progressHandler)
                {
                    KString srcStr(source);
                    KString dstStr(destination);
                    KCallbackFx^ thunk = gcnew KCallbackFx(&KStream::OnProgressUpdate);
                    KCallbackState context(thunk, progressHandler);

                    KCheck(kFile_MoveEx(srcStr.CharPtr, dstStr.CharPtr, (kCallbackFx)context.NativeFunction, context.NativeContext));
                }

                /// <summary>Deletes the specified file.</summary>
                /// 
                /// <param name="path">Path to file.</param>
                static void Delete(String^ path)
                {
                    KString pathStr(path);

                    KCheck(kFile_Delete(pathStr.CharPtr));
                }

                /// <summary>Suggests a suitable name for a temporary file.</summary>
                /// 
                /// <returns>Suggested file name.</returns>
                static String^ GetTempName()
                {
                    kChar pathText[kPATH_MAX]; 

                    KCheck(kFile_TempName(pathText, kCountOf(pathText)));

                    return KToString(pathText);
                }

                /// <summary>Performs any outstanding I/O operations and closes the underlying file.</summary>
                /// 
                /// <remarks>
                /// <para>The purpose of the Close method is to provide an opportunity to finalize I/O and
                /// report any errors before destroying a file object. If the Dispose method is called without
                /// having closed the file, the Dispose method will close the file and ignore any errors.</para>
                /// 
                /// <para>The Close method should only be called once per file object. After calling the Close method,
                /// any operations other than Dispose will produce an undefined result.</para>
                /// </remarks>
                void Close()
                {
                    KCheck(kFile_Close(Handle));
                }

                /// <summary>Sets the size of the buffer used for writing.</summary>
                /// 
                /// <remarks>Buffering can improve efficiency when performing several small write operations.
                /// The default buffer size is 0.</remarks>
                property k64s WriteBufferSize
                {
                    //add get when available

                    void set(k64s size)
                    {
                        KCheck(kFile_SetWriteBuffer(Handle, (kSize)size));
                    }
                }

                /// <summary>Sets the size of the buffer used for reading.</summary>
                /// 
                /// <remarks>Buffering can improve efficiency when performing several small read operations. 
                /// The default buffer size is 0.</remarks>
                /// 
                /// <param name="size">Size of the buffer.</param>
                property k64s ReadBufferSize
                {
                    //add get when available

                    void set(k64s size)
                    {
                        KCheck(kFile_SetReadBuffer(Handle, (kSize)size));
                    }
                }

                /// <summary> Gets the current length of the file.</summary>
                property k64s Length 
                { 
                    k64s get() { return kFile_Length(Handle); }
                }

                /// <summary> Gets the current position of the read/write pointer, relative to the beginning of the file. </summary>
                property k64s Position
                { 
                    k64s get() { return kFile_Position(Handle); }
                }

            protected:
                KFile() : KStream(DefaultRefStyle) {}
            };
        }
    }
}

#endif

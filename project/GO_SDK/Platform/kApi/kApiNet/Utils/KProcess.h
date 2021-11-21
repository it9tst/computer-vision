// 
// KProcess.h
// 
// Copyright (C) 2018-2021 by LMI Technologies Inc.
// Licensed under the MIT License.
// Redistributed files must retain the above copyright notice.
// 
#ifndef K_API_NET_PROCESS_H
#define K_API_NET_PROCESS_H

#include <kApi/Utils/kProcess.h>
#include "kApiNet/KAlloc.h"

namespace Lmi3d
{
    namespace Zen 
    {
        namespace Utils
        {           

            /// <summary>Represents a process. <para/> Requires manual disposal.</summary>
            /// <remarks> 
            /// KProcess class allows to start a process and offers the possibility to define multiple arguments.
            /// For Input and output KPipeStream class can be used. For a KPipeStream of stdin, stdout or stderr
            /// for the current process see KStdIn(), KStdOut() and KStdErr().
            /// </remarks>
            public ref class KProcess: public KObject
            {
                KDeclareClass(KProcess, kProcess)

            public:
                /// <summary>Initializes a new instance of the KProcess class with the specified Zen object handle.</summary>           
                /// <param name="handle">Zen object handle.</param>
                KProcess(IntPtr handle)
                    : KObject(handle, DefaultRefStyle)
                {}

                /// <inheritdoc cref="KProcess(IntPtr)" />
                ///
                /// <param name="refStyle">Ref style.</param>
                KProcess(IntPtr handle, KRefStyle refStyle)
                    : KObject(handle, refStyle)
                {}

                KProcess(String^ path)
                    : KObject(DefaultRefStyle)
                {
                    KString strPath(path);
                    kProcess handle = kNULL;

                    KCheck(kProcess_Construct(&handle, strPath.CharPtr, kNULL));

                    Handle = handle;
                }

                /// <summary>Adds an argument. Can only be used when process is not running.</summary>
                /// 
                /// <param name="argument">New argument.</param>
                void AddArgument(String^ argument)
                {
                    KString strArgument(argument);

                    KCheck(kProcess_AddArgument(Handle, strArgument.CharPtr));
                }

                /// <summary>Adds an array of arguments. Can only be used when process is not running.</summary>
                /// 
                /// <param name="arguments">Array with the arguments.</param>
                void AddArguments(array<String^>^ arguments)
                {
                    for (int i = 0; i < arguments->Length; i++)
                    {
                        KString strArgument(arguments[i]);

                        KCheck(kProcess_AddArgument(Handle, strArgument.CharPtr));
                    }
                }

                /// <summary>Deletes all arguments. Can only be used when process is not running.</summary>
                void ClearArguments()
                {
                    KCheck(kProcess_ClearArguments(Handle));
                }

                /// <summary>Starts the new process.</summary>
                /// <remarks>
                /// Starts the new process given the path and arguments. You have to call kProcess_Wait between
                /// consecutive calls of kProcess_Start. There is no guaranty when exactly after this call the child process
                /// has really started.
                /// </remarks>
                void Start()
                {
                    KCheck(kProcess_Start(Handle));
                }

                /// <summary>Determines if the process is still alive.</summary>
                property bool IsAlive
                {
                    bool get() { return KToBool(kProcess_IsAlive(Handle)); }
                }

                /// <summary>Waits for child process completion, otherwise terminates child after the timeout.</summary>
                /// 
                /// <remarks>
                /// This functions returns the exit code of the child. If the child was terminated or
                /// in case of an error the exit code becomes k64S_MAX.
                /// </remarks>
                ///
                /// <param name="timeout">Timeout in microseconds, or KTimeout.Infinite to wait indefinitely.</param>
                /// 
                /// <returns>Exit code.</returns>
                k64s Wait(k64s timeout)
                {
                    k64s exitCode;
                    KCheck(kProcess_Wait(Handle, timeout, &exitCode));

                    return exitCode;
                }

                /// <summary>Returns the stdin of the process as KStream.</summary>
                /// 
                /// <returns>Stream object.</returns>
                property KStream^ StdIn
                {
                    KStream^ get()
                    {
                        kStream stream = kProcess_StdIn(Handle);
                        KAdjustRef(stream, kTRUE, Nullable<KRefStyle>());

                        return KToObject<KStream^>(stream);
                    }
                }

                /// <summary>Returns the stdout of the process as KStream.</summary>
                /// 
                /// <returns>Stream object.</returns>
                property KStream^ StdOut
                {
                    KStream^ get()
                    {
                        kStream stream = kProcess_StdOut(Handle);
                        KAdjustRef(stream, kTRUE, Nullable<KRefStyle>());

                        return KToObject<KStream^>(stream);
                    }
                }

                /// <summary>Returns the stderr of the process as KStream.</summary>
                /// 
                /// <returns>Stream object.</returns>
                property KStream^ StdErr
                {
                    KStream^ get()
                    {
                        kStream stream = kProcess_StdErr(Handle);
                        KAdjustRef(stream, kTRUE, Nullable<KRefStyle>());

                        return KToObject<KStream^>(stream);
                    }
                }
            };
        }
    }
}

#endif

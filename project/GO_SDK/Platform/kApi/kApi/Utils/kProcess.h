/** 
 * @file    kProcess.h
 * @brief   Declares the kProcess class. 
 *
 * @internal
 * Copyright (C) 2018-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */

#ifndef K_API_PROCESS_H
#define K_API_PROCESS_H

#include <kApi/kApiDef.h>

/**
 * @class   kProcess
 * @extends kObject
 * @ingroup kApi-Utils
 * @brief   Represents a process. 
 
 * kProcess class allows to start a process and offers the possibility to define multiple arguments.
 * For Input and output kPipeStream class can be used. For a kPipeStream of stdin, stdout or stderr 
 * for the current process see kStdIn(), kStdOut() and kStdErr() in kPipeStream.h.
 * 
 */
//typedef kObject kProcess;        --forward-declared in kApiDef.x.h


/** 
 * Constructs a kProcess object.
 *
 * Allows the creation and start of a child process. The search path will not be used when looking for the executable.
 * This class is not thread safe. The child process gets terminated when the calling process ends.
 *
 * Windows specific:
 * The path string can specify the full path and file name of the module to execute or it can specify a partial name. 
 * In the case of a partial name, the function uses the current drive and current directory to complete the specification. 
 *
 * @public              @memberof kProcess
 * @param   process     Destination for the constructed object handle.
 * @param   path        Path of the new process. 
 * @param   allocator   Memory allocator (or kNULL for default). 
 * @return              Operation status. 
 */
kFx(kStatus) kProcess_Construct(kProcess* process, const kChar* path, kAlloc allocator); 

/** 
 * Adds an argument. Can only be used when process is not running.
 *
 * @public              @memberof kProcess
 * @param   process     kProcess object.
 * @param   argument    New argument. 
 * @return              Operation status. 
 */
kFx(kStatus) kProcess_AddArgument(kProcess process, const kChar* argument);

/** 
 * Adds an array of arguments. Can only be used when process is not running.
 *
 * @public              @memberof kProcess
 * @param   process     kProcess object.
 * @param   arguments   Array with the arguments.
 * @param   argCount    Number of arguments.
 * @return              Operation status. 
 */
kFx(kStatus) kProcess_AddArguments(kProcess process, const kChar* arguments[], kSize argCount);

/** 
 * Deletes all arguments. Can only be used when process is not running.
 *
 * @public              @memberof kProcess
 * @param   process     kProcess object.
 * @return              Operation status. 
 */
kFx(kStatus) kProcess_ClearArguments(kProcess process);

/** 
 * Starts the new process. 
 *
 * Starts the new process given the path and arguments. You have to call kProcess_Wait between 
 * consecutive calls of kProcess_Start. There is no guaranty when exactly after this call the child process 
 * has really started. 
 *
 * @public          @memberof kProcess
 * @param   process kProcess object. 
 * @return          Operation status.  
 */
kFx(kStatus) kProcess_Start(kProcess process);

/** 
 * Determines if the process is still alive.
 *
 * @public          @memberof kProcess
 * @param   process kProcess object. 
 * @return          kTRUE is process is still alive, otherwise kFALSE.  
 */
kFx(kBool) kProcess_IsAlive(kProcess process);

/** 
 * Waits for child process completion, otherwise terminates child after the timeout.
 *
 * The exit code parameter receives the exit code of the child. If the child was terminated or
 * in case of an error the exit code becomes k64S_MAX.
 * 
 * @public            @memberof kProcess
 * @param   process   kProcess object. 
 * @param   timeout   Timeout in microseconds, or kINFINITE to wait indefinitely.
 * @param   exitCode  Receives the exit code of the child. k64S_MAX in case of child termination. Can be kNULL.
 * @return            Operation status.  
 */
kFx(kStatus) kProcess_Wait(kProcess process, k64u timeout, k64s* exitCode);

/** 
 * Returns the stdin of the process as kStream.
 *
 * It is only possible to write to this stream. Process must be alive. kStream is only valid until
 * the next call of kProcess_Wait.
 *
 * @public           @memberof kProcess
 * @param   process  kProcess object. 
 * @return           kStream object.
 */
kFx(kStream) kProcess_StdIn(kProcess process);

/** 
 * Returns the stdout of the process as kStream.
 *
 * It is only possible to read from this stream. Process must be alive. kStream is only valid until
 * the next call of kProcess_Wait.
 *
 * @public           @memberof kProcess
 * @param   process  kProcess object. 
 * @return           kStream object.  
 */
kFx(kStream) kProcess_StdOut(kProcess process);

/** 
 * Returns the stderr of the process as kStream.
 *
 * It is only possible to read from this stream. Process must be alive. kStream is only valid until
 * the next call of kProcess_Wait.
 *
 * @public           @memberof kProcess
 * @param   process  kProcess object.
 * @return           kStream object.  
 */
kFx(kStream) kProcess_StdErr(kProcess process);

/** 
 * Returns the process id of the current process.
 *
 * On unsupported platforms 0 is returned.
 *
 * @public           @memberof kProcess
 * @return           Process id.  
 */
kFx(k32s) kProcess_Id();

#include <kApi/Utils/kProcess.x.h>

#endif

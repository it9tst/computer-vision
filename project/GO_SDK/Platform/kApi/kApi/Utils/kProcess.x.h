/** 
 * @file    kProcess.x.h
 *
 * @internal
 * Copyright (C) 2018-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */

#ifndef K_API_PROCESS_X_H
#define K_API_PROCESS_X_H

kDeclareFullClassEx(k, kProcess, kObject)

#if defined(K_PLATFORM) 

kFx(kStatus) kProcess_InitPlatformFields(kProcess process);
kFx(kStatus) kProcess_ReleasePlatformFields(kProcess process);

#if defined(K_WINDOWS)

kFx(kStatus) kProcess_ConstructArgument(kProcess process, WCHAR** wargument);

#   define kProcessPlatformFields()             \
        HANDLE process;                         \
        HANDLE stdOutHandle;                   \
        HANDLE stdErrHandle;                   \
        HANDLE stdInHandle;

#   define kProcessPlatformStaticFields()       \
        HANDLE job;                             

#elif defined(K_LINUX)

#   define kPROCESS_PIPE_READ      (0) 
#   define kPROCESS_PIPE_WRITE     (1) 

#   define kProcessPlatformFields()         \
        k32s process;                       \
        k32s stdinPipe[2];                  \
        k32s stdoutPipe[2];                 \
        k32s stderrPipe[2];                 \
        kSemaphore startSem;                \
        kThread forkThread;                 \
        k64s exitCode;

#   define kProcessPlatformStaticFields()   \

kFx(kStatus) kProcess_SetupParent(kProcess process);
kFx(void) kProcess_RunChild(kProcess process, pid_t parent);

kFx(kStatus) kProcess_CloseHandles(kProcess process);

kFx(kStatus) kProcess_ForkThreadEntry(kProcess process);

#else

#   define kProcessPlatformFields()
#   define kProcessPlatformStaticFields() 

#endif

typedef struct kProcessClass
{
    kObjectClass base;

    kAtomic32s isRunning;
    kString path;
    kArrayList arguments;

    kStream processStdIn;
    kStream processStdOut;
    kStream processStdErr;

    kProcessPlatformFields()
} kProcessClass;

typedef struct kProcessVTable
{
    kObjectVTable base;
} kProcessVTable;

typedef struct kProcessStatic
{
    kStream appStdIn;
    kStream appStdOut;
    kStream appStdErr;

    kProcessPlatformStaticFields()
} kProcessStatic;

#endif

kFx(kStatus) xkProcess_InitStatic();
kFx(kStatus) xkProcess_ReleaseStatic();

kFx(kStatus) kProcess_Init(kProcess process, const kChar* path, kType type, kAlloc alloc);
kFx(kStatus) kProcess_VRelease(kProcess process);

kFx(kStream) kProcess_AppStdIn();
kFx(kStream) kProcess_AppStdOut();
kFx(kStream) kProcess_AppStdErr();

#endif

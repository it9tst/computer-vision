/** 
 * @file    kThread.x.h
 *
 * @internal
 * Copyright (C) 2005-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_THREAD_X_H
#define K_API_THREAD_X_H

kDeclareFullClassEx(k, kThread, kObject)

#if defined(K_PLATFORM) 

#   define xkTHREAD_LOG_ENABLED     (kFALSE)        //set to kTRUE to log thread creation/destruction (useful for debugging kThread itself)

#if defined(K_WINDOWS)
     
    //priority class boundaries
#   define xkTHREAD_MIN_LOW_PRIORITY_OFFSET            (-1)
#   define xkTHREAD_MAX_LOW_PRIORITY_OFFSET            (1) 
#   define xkTHREAD_MIN_NORMAL_PRIORITY_OFFSET         (0) 
#   define xkTHREAD_MAX_NORMAL_PRIORITY_OFFSET         (0) 
#   define xkTHREAD_MIN_HIGH_PRIORITY_OFFSET           (-1)
#   define xkTHREAD_MAX_HIGH_PRIORITY_OFFSET           (1) 

    typedef kText64 xkThreadName;

#   define xkThreadPlatformFields()                                 \
        HANDLE handle;      /* thread handle */                     \
        unsigned id;        /* unique thread id from os */

#   define xkThreadPlatformStaticFields()

    unsigned int __stdcall xkThread_EntryPoint(void* arg);

#elif defined(K_TI_BIOS)

    //native priority levels that define priority class boundaries
#   define xkTHREAD_OS_MIN_LOW_PRIORITY             (0)          //Native OS thread priority associated with Low class minimum. 
#   define xkTHREAD_OS_DEFAULT_LOW_PRIORITY         (6)          //Native OS thread priority associated with Low class default. 
#   define xkTHREAD_OS_MAX_LOW_PRIORITY             (6)          //Native OS thread priority associated with Low class maximum.
#   define xkTHREAD_OS_MIN_NORMAL_PRIORITY          (7)          //Native OS thread priority associated with Normal class minimum.
#   define xkTHREAD_OS_DEFAULT_NORMAL_PRIORITY      (7)          //Native OS thread priority associated with Normal class default.
#   define xkTHREAD_OS_MAX_NORMAL_PRIORITY          (7)          //Native OS thread priority associated with Normal class maximum.
#   define xkTHREAD_OS_MIN_HIGH_PRIORITY            (8)          //Native OS thread priority associated with High class minimum.
#   define xkTHREAD_OS_DEFAULT_HIGH_PRIORITY        (8)          //Native OS thread priority associated with High class default.
#   define xkTHREAD_OS_MAX_HIGH_PRIORITY            (15)         //Native OS thread priority associated with High class maximum.

    //priority class boundaries
#   define xkTHREAD_MIN_LOW_PRIORITY_OFFSET         (xkTHREAD_OS_MIN_LOW_PRIORITY - xkTHREAD_OS_DEFAULT_LOW_PRIORITY) 
#   define xkTHREAD_MAX_LOW_PRIORITY_OFFSET         (xkTHREAD_OS_MAX_LOW_PRIORITY - xkTHREAD_OS_DEFAULT_LOW_PRIORITY) 
#   define xkTHREAD_MIN_NORMAL_PRIORITY_OFFSET      (xkTHREAD_OS_MIN_NORMAL_PRIORITY - xkTHREAD_OS_DEFAULT_NORMAL_PRIORITY) 
#   define xkTHREAD_MAX_NORMAL_PRIORITY_OFFSET      (xkTHREAD_OS_MAX_NORMAL_PRIORITY - xkTHREAD_OS_DEFAULT_NORMAL_PRIORITY) 
#   define xkTHREAD_MIN_HIGH_PRIORITY_OFFSET        (xkTHREAD_OS_MIN_HIGH_PRIORITY - xkTHREAD_OS_DEFAULT_HIGH_PRIORITY) 
#   define xkTHREAD_MAX_HIGH_PRIORITY_OFFSET        (xkTHREAD_OS_MAX_HIGH_PRIORITY - xkTHREAD_OS_DEFAULT_HIGH_PRIORITY) 

    typedef kText64 xkThreadName;

#   define xkThreadPlatformFields()                                              \
        ti_sysbios_knl_Task_Handle handle;  /* thread handle */                 \
        kSemaphore joinSem;                 /* implements join behaviour */     \
        kAtomic32s exitCode;                /* result of thread execution */

#   define xkThreadPlatformStaticFields()

    void xkThread_EntryPoint(UArg arg0, UArg arg1);

#elif defined(K_VX_KERNEL)

    //native priority levels that define priority class boundaries
#   define xkTHREAD_OS_MIN_LOW_PRIORITY             (255)          //Native OS thread priority associated with Low class minimum. 
#   define xkTHREAD_OS_DEFAULT_LOW_PRIORITY         (140)          //Native OS thread priority associated with Low class default. 
#   define xkTHREAD_OS_MAX_LOW_PRIORITY             (129)          //Native OS thread priority associated with Low class maximum.
#   define xkTHREAD_OS_MIN_NORMAL_PRIORITY          (128)          //Native OS thread priority associated with Normal class minimum.
#   define xkTHREAD_OS_DEFAULT_NORMAL_PRIORITY      (128)          //Native OS thread priority associated with Normal class default.
#   define xkTHREAD_OS_MAX_NORMAL_PRIORITY          (128)          //Native OS thread priority associated with Normal class maximum.
#   define xkTHREAD_OS_MIN_HIGH_PRIORITY            (127)          //Native OS thread priority associated with High class minimum.
#   define xkTHREAD_OS_DEFAULT_HIGH_PRIORITY        (116)          //Native OS thread priority associated with High class default.
#   define xkTHREAD_OS_MAX_HIGH_PRIORITY            (0)            //Native OS thread priority associated with High class maximum.

#   define xkTHREAD_OS_MAX_NON_SYSTEM_PRIORITY      (100)          //Maximum Native OS thread priority recommended by vendor for non-system threads.

    //priority class boundaries
#   define xkTHREAD_MIN_LOW_PRIORITY_OFFSET         (xkTHREAD_OS_DEFAULT_LOW_PRIORITY - xkTHREAD_OS_MIN_LOW_PRIORITY) 
#   define xkTHREAD_MAX_LOW_PRIORITY_OFFSET         (xkTHREAD_OS_DEFAULT_LOW_PRIORITY - xkTHREAD_OS_MAX_LOW_PRIORITY) 
#   define xkTHREAD_MIN_NORMAL_PRIORITY_OFFSET      (xkTHREAD_OS_DEFAULT_NORMAL_PRIORITY - xkTHREAD_OS_MIN_NORMAL_PRIORITY) 
#   define xkTHREAD_MAX_NORMAL_PRIORITY_OFFSET      (xkTHREAD_OS_DEFAULT_NORMAL_PRIORITY - xkTHREAD_OS_MAX_NORMAL_PRIORITY) 
#   define xkTHREAD_MIN_HIGH_PRIORITY_OFFSET        (xkTHREAD_OS_DEFAULT_HIGH_PRIORITY - xkTHREAD_OS_MIN_HIGH_PRIORITY) 
#   define xkTHREAD_MAX_HIGH_PRIORITY_OFFSET        (xkTHREAD_OS_DEFAULT_HIGH_PRIORITY - xkTHREAD_OS_MAX_HIGH_PRIORITY) 

    //maximum High priority class offset recommended by vendor for non-system threads.
#   define xkTHREAD_MAX_NON_SYSTEM_HIGH_PRIORITY_OFFSET   (xkTHREAD_OS_DEFAULT_HIGH_PRIORITY - xkTHREAD_OS_MAX_NON_SYSTEM_PRIORITY)    

#   define xkTHREAD_DEFAULT_STACK_SIZE              (0x8000)        //Default stack size assigned to kThread instances.
#   define xkTHREAD_DEFAULT_OPTIONS                 (VX_FP_TASK)    //Default thread creation options. 

    typedef kText64 xkThreadName;

#   define xkThreadPlatformFields()                                      \
        TASK_ID id;                 /* thread identifier */             \
        kSemaphore joinSem;         /* implements join behaviour */     \
        kAtomic32s exitCode;        /* result of thread execution */

#   define xkThreadPlatformStaticFields()

    int xkThread_EntryPoint(_Vx_usr_arg_t arg0);
            
#elif defined (K_LINUX)

#   define xkTHREAD_OS_DEFAULT_HIGH_PRIORITY        (1)         //Native OS thread priority associated with High class default.

    //priority class boundaries
#   define xkTHREAD_MIN_LOW_PRIORITY_OFFSET         (0)     //Idle scheduler
#   define xkTHREAD_MAX_LOW_PRIORITY_OFFSET         (0)     //Idle scheduler
#   define xkTHREAD_MIN_NORMAL_PRIORITY_OFFSET      (-19)   //Normal ("other") scheduler; Nice 19
#   define xkTHREAD_MAX_NORMAL_PRIORITY_OFFSET      (20)    //Normal ("other") scheduler; Nice -20
#   define xkTHREAD_MIN_HIGH_PRIORITY_OFFSET        (0)     //RT RR scheduler: 1
#   define xkTHREAD_MAX_HIGH_PRIORITY_OFFSET        (98)    //RT RR scheduler: 99

    //matches underlying OS capability
    typedef kText16 xkThreadName;

#   define xkThreadPlatformFields()                                         \
        kSemaphore hasStarted;          /* supports getting OS thread id */ \
        kSemaphore hasJoined;           /* supports join wait argument */   \
        pthread_t pthreadId;            /* unique pthread identifier */     \
        int schedulingPolicy;                                               \
        sched_param schedulingParam;                                        \
        k32s nice;                      /* CFS weight ('nice' value) */     \
        cpu_set_t cpuSet;                                                   \
        kAtomic32s threadId;            /* unique Linux thread identifer */

#   define xkThreadPlatformStaticFields()                                   \
        kBool canIncreasePriority;                                          \
        kBool canSetAffinity;

    void* xkThread_EntryPoint(void* arg);

    kFx(kStatus) xkThread_UpdatePriorityParams(kThread thread);
    kFx(kStatus) xkThread_UpdateAffinityParams(kThread thread);
    kFx(kStatus) xkThread_PrintAffinity(kThread thread, kChar* buffer, kSize capacity);

#elif defined(K_POSIX)

#   define xkThreadPlatformFields()                                     \
        kSemaphore hasJoined;       /* supports join wait argument */   \
        pthread_t pthreadId;        /* unique pthread identifier */

    typedef kText64 xkThreadName;

#   define xkThreadPlatformStaticFields()

    void* xkThread_EntryPoint(void* arg);

#endif

typedef struct kThreadClass
{
    kObjectClass base;    
    xkThreadName name;                      //name
    kThreadPriorityClass priorityClass;     //logical priority class provided by user
    k32s priorityOffset;                    //logical priority offset provided by user
    kBitArray affinity;                     //CPU affinity provided by user.
    kThreadFx function;                     //entry-point function
    kPointer context;                       //entry-point context
 
    xkThreadPlatformFields()
} kThreadClass;

typedef struct kThreadVTable
{
    kObjectVTable base;    
} kThreadVTable;

typedef struct kThreadStatic
{
    xkThreadPlatformStaticFields()
} kThreadStatic;

 
/* 
* Private methods. 
*/

kFx(kStatus) xkThread_InitStatic(); 
kFx(kStatus) xkThread_ReleaseStatic(); 

kFx(kStatus) xkThread_Init(kThread thread, kType type, kAlloc alloc); 
kFx(kStatus) xkThread_VRelease(kThread thread); 

kFx(kStatus) xkThread_InitShared(kThread thread); 
kFx(kStatus) xkThread_ReleaseShared(kThread thread);

/** 
 * Gets a unique identifier that represents the currently executing thread.
 * 
 * @public              @memberof kThread
 * @return              Unique thread identifier. 
 */
kFx(xkThreadId) xkThread_CurrentId(); 

/** 
 * Gets a unique identifier representing the thread.
 * 
 * This field is only valid after the thread has been successfully started, and 
 * before the thread has been successfully joined.
 *
 * @public              @memberof kThread
 * @param   thread      Thread object. 
 * @return              Unique thread identifier. 
 */
kFx(xkThreadId) xkThread_Id(kThread thread); 

/** 
 * Compares two thread identifiers.
 *
 * @public          @memberof kThread
 * @param   a       First thread identifier.
 * @param   b       Second thread identifier.
 * @return          kTRUE if the threads are identical; kFALSE otherwise. 
 */
kFx(kBool) xkThread_CompareId(xkThreadId a, xkThreadId b); 

kFx(kStatus) xkThread_AdjustPriority(kThread thread); 

kFx(kStatus) xkThread_AdjustAffinity(kThread thread); 

kInlineFx(kBool) xkThread_IsStarted(kThread thread)
{
    kObj(kThread, thread); 

    return !kIsNull(obj->function); 
}

#endif

/*
* Forward declarations.
*/

kFx(kStatus) kThread_SetName(kThread thread, const kChar* name);
kFx(kStatus) kThread_SetPriority(kThread thread, kThreadPriorityClass priorityClass, k32s priorityOffset);
kFx(kStatus) kThread_Start(kThread thread, kThreadFx function, kPointer context);

#if defined(K_CPP)
kInlineFx(kStatus) kThread_Start(kThread thread, kThreadFx function, kPointer context, const kChar* name,
                  kThreadPriorityClass priorityClass, k32s priorityOffset);
#endif


/* 
* Private methods. 
*/

kFx(kThreadFx) xkThread_Handler(kThread thread);
kFx(kPointer)  xkThread_HandlerContext(kThread thread);

kFx(void) xkThread_ConvertLegacyPriorty(k32s priority, kThreadPriorityClass* priorityClass, k32s* priorityOffset);

/* 
* Deprecated (Stage 1): not recommended for further use, but not yet announced via kDeprecate
*/

//[Deprecated] Update to use a supported kThread_Start overload.
kInlineFx(kStatus) kThread_StartEx(kThread thread, kThreadFx function, kPointer context, 
                             kSize stackSize, const kChar* name, k32s priority)                
{
    kThreadPriorityClass priorityClass = kTHREAD_PRIORITY_CLASS_NORMAL;
    k32s priorityOffset = 0;

    xkThread_ConvertLegacyPriorty(priority, &priorityClass, &priorityOffset);

    kCheck(kThread_SetPriority(thread, priorityClass, priorityOffset));
    kCheck(kThread_SetName(thread, name));

    return kThread_Start(thread, function, context); 
}

#endif

/** 
 * @file    kThread.cpp
 *
 * @internal
 * Copyright (C) 2005-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#define K_PLATFORM
#include <kApi/Threads/kThread.h>

#include <kApi/Data/kBitArray.h>
#include <kApi/Data/kMath.h>
#include <kApi/Threads/kSemaphore.h>
#include <kApi/Threads/kTimer.h>
#include <kApi/Utils/kUtils.h>

/*
* kThread class
*/

kBeginFullClassEx(k, kThread)
    kAddPrivateVMethod(kThread, kObject, VRelease)
kEndFullClassEx()

kFx(void) xkThread_ConvertLegacyPriorty(k32s priority, kThreadPriorityClass* priorityClass, k32s* priorityOffset)
{
    *priorityClass = kTHREAD_PRIORITY_CLASS_NORMAL;
    *priorityOffset = 0;

    if (priority > 0)
    {
        *priorityClass = kTHREAD_PRIORITY_CLASS_HIGH;
        *priorityOffset = kThread_MinPriorityOffset(*priorityClass) + priority - 1;
    }
    else if (priority < 0)
    {
        *priorityClass = kTHREAD_PRIORITY_CLASS_LOW;
        *priorityOffset = kThread_MaxPriorityOffset(*priorityClass) + priority + 1;
    }
}

kFx(kStatus) kThread_Construct(kThread *thread, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback(allocator);
    kStatus status; 

    kCheck(kAlloc_GetObject(alloc, kTypeOf(kThread), thread)); 

    if (!kSuccess(status = xkThread_Init(*thread, kTypeOf(kThread), alloc)))
    {
        kAlloc_FreeRef(alloc, thread); 
    }

    return status; 
} 

//utility method that can be used to initialize bits shared by all platform implementations
kFx(kStatus) xkThread_InitShared(kThread thread)
{
    kObjR(kThread, thread); 
            
    obj->name[0] = '\0';
    obj->priorityClass = kTHREAD_PRIORITY_CLASS_NORMAL;
    obj->priorityOffset = 0;
    obj->affinity = kNULL;
    obj->function = kNULL; 
    obj->context = kNULL;
    
    //we use kThreadId (alias for kSize) as public thread identifier type; ensure that it's large enough
    static_assert(sizeof(xkThreadId) <= sizeof(kThreadId), "OS thread ID larger than kThreadId.");

    kCheck(kBitArray_Construct(&obj->affinity, kThread_ProcessorCount(), kObject_Alloc(thread)));
    kCheck(kBitArray_SetAll(obj->affinity, kTRUE));

    return kOK;
}

kFx(kStatus) xkThread_ReleaseShared(kThread thread)
{
    kObj(kThread, thread); 

    kCheck(kDestroyRef(&obj->affinity));

    return kOK;
}

kFx(kStatus) kThread_SetName(kThread thread, const kChar* name)
{
    kObj(kThread, thread); 

    kCheckState(!xkThread_IsStarted(thread));

    //will silently truncate, if specified name too long: OK
    kStrCopy(obj->name, sizeof(obj->name), kIsNull(name) ? "" : name);

    return kOK;
}

kFx(const kChar*) kThread_Name(kThread thread)
{
    kObj(kThread, thread); 

    return obj->name;
}

kFx(kStatus) kThread_SetPriority(kThread thread, kThreadPriorityClass priorityClass, k32s priorityOffset)
{
    kObj(kThread, thread); 

    if (priorityClass < kThread_MinPriorityClass())
    {
        obj->priorityClass = kThread_MinPriorityClass(); 
        obj->priorityOffset = kThread_MinPriorityOffset(obj->priorityClass);
    }
    else if (priorityClass > kThread_MaxPriorityClass())
    {
        obj->priorityClass = kThread_MaxPriorityClass(); 
        obj->priorityOffset = kThread_MaxPriorityOffset(obj->priorityClass);
    }
    else
    {
        obj->priorityClass = priorityClass; 
        obj->priorityOffset = kClamp_(priorityOffset, kThread_MinPriorityOffset(obj->priorityClass), kThread_MaxPriorityOffset(obj->priorityClass));
    }

    if (xkThread_IsStarted(thread))
    {
        //best effort only; OS result not checked
        xkThread_AdjustPriority(thread); 
    }

    return kOK; 
}

kFx(kStatus) kThread_SetAffinity(kThread thread, kBitArray affinity)
{
    kObj(kThread, thread);

    if (kIsNull(affinity) || (kBitArray_TrueCount(affinity) == 0))
    {
        kCheck(kBitArray_SetAll(obj->affinity, kTRUE)); 
    }
    else
    {
        kCheck(kBitArray_Assign(obj->affinity, affinity)); 
        kCheck(kBitArray_Resize(obj->affinity, kThread_ProcessorCount())); 
    }
 
    if (xkThread_IsStarted(thread))
    {
        //best effort only; OS result not checked
        xkThread_AdjustAffinity(thread); 
    }

    return kOK; 
}

kFx(kStatus) kThread_SleepAtLeast(k64u duration)
{
    k64u startTime = kTimer_Now(); 
    k64u endTime = startTime + duration; 
    k64u currentTime; 

    while ((currentTime = kTimer_Now()) < endTime)
    {
        kCheck(kThread_Sleep(endTime - currentTime)); 
    }

    return kOK; 
}

kFx(kThreadFx) xkThread_Handler(kThread thread)
{
    kObj(kThread, thread);

    return obj->function; 
}

kFx(kPointer) xkThread_HandlerContext(kThread thread)
{
    kObj(kThread, thread);

    return obj->context;
}

kFx(kBool) kThread_IsSelf(kThread other)
{
    return xkThread_CurrentId() == xkThread_Id(other);
}

kFx(kThreadId) kThread_CurrentId()
{
    return (kThreadId) xkThread_CurrentId();
}

kFx(kThreadId) kThread_Id(kThread thread)
{
    return (kThreadId) xkThread_Id(thread);
}

#if defined(K_WINDOWS)

kFx(kStatus) xkThread_InitStatic()
{
    return kOK;
}

kFx(kStatus) xkThread_ReleaseStatic()
{
    return kOK;
}

kFx(kSize) kThread_ProcessorCount()
{
    SYSTEM_INFO sysInfo = { 0 }; 

    GetSystemInfo(&sysInfo); 

    return (kSize) sysInfo.dwNumberOfProcessors; 
}

kFx(kThreadPriorityClass) kThread_MinPriorityClass()
{
    return kTHREAD_PRIORITY_CLASS_LOW;
}

kFx(kThreadPriorityClass) kThread_MaxPriorityClass()
{
    return kTHREAD_PRIORITY_CLASS_HIGH;
}

kFx(k32s) kThread_MinPriorityOffset(kThreadPriorityClass priorityClass)
{
    switch (priorityClass)
    {
        case kTHREAD_PRIORITY_CLASS_LOW:    return xkTHREAD_MIN_LOW_PRIORITY_OFFSET; 
        case kTHREAD_PRIORITY_CLASS_HIGH:   return xkTHREAD_MIN_HIGH_PRIORITY_OFFSET;
        default:                            return xkTHREAD_MIN_NORMAL_PRIORITY_OFFSET; 
    }
}

kFx(k32s) kThread_MaxPriorityOffset(kThreadPriorityClass priorityClass)
{
    switch (priorityClass)
    {
        case kTHREAD_PRIORITY_CLASS_LOW:    return xkTHREAD_MAX_LOW_PRIORITY_OFFSET; 
        case kTHREAD_PRIORITY_CLASS_HIGH:   return xkTHREAD_MAX_HIGH_PRIORITY_OFFSET;
        default:                            return xkTHREAD_MAX_NORMAL_PRIORITY_OFFSET; 
    }
}

kFx(k32s) xkThread_OsPriority(kThreadPriorityClass priorityClass, k32s priorityOffset)
{
    switch (priorityClass)
    {
        case kTHREAD_PRIORITY_CLASS_LOW:
            if      (priorityOffset < 0)    return THREAD_PRIORITY_IDLE; 
            else if (priorityOffset == 0)   return THREAD_PRIORITY_LOWEST;
            else                            return THREAD_PRIORITY_BELOW_NORMAL; 
        case kTHREAD_PRIORITY_CLASS_HIGH:
            if      (priorityOffset < 0)    return THREAD_PRIORITY_ABOVE_NORMAL; 
            else if (priorityOffset == 0)   return THREAD_PRIORITY_HIGHEST;
            else                            return THREAD_PRIORITY_TIME_CRITICAL; 
        default: 
            return THREAD_PRIORITY_NORMAL;
    }
}

kFx(kBool) kThread_CanSetAffinty()
{
    return kTRUE;
}

kFx(kStatus) kThread_Sleep(k64u duration)
{
    DWORD osDuration = (DWORD) xkTimeToKernelTime(duration); 
    
    Sleep(osDuration); 
    
    return kOK; 
}

kFx(kStatus) xkThread_Init(kThread thread, kType type, kAlloc allocator)
{
    kObjR(kThread, thread);
    kStatus status;

    kCheck(kObject_Init(thread, type, allocator)); 
    
    kTry
    {
        kTest(xkThread_InitShared(thread));

        obj->id = 0;
        obj->handle = kNULL;
    }
    kCatch(&status)
    {
        xkThread_VRelease(thread);
        kEndCatch(status);
    }
    
    return kOK; 
}

kFx(kStatus) xkThread_VRelease(kThread thread)
{
    kCheck(kThread_Join(thread, kINFINITE, kNULL)); 

    kCheck(xkThread_ReleaseShared(thread));

    kCheck(kObject_VRelease(thread)); 

    return kOK; 
}

kFx(kStatus) kThread_Start(kThread thread, kThreadFx function, kPointer context)
{
    kObj(kThread, thread);
    kStatus status;

    kCheckState(!xkThread_IsStarted(thread));
    
    obj->function = function; 
    obj->context = context; 

    kTry
    {
        kTestTrue(!kIsNull(obj->handle = (HANDLE)_beginthreadex(0, 0, xkThread_EntryPoint, thread, CREATE_SUSPENDED, &obj->id)), kERROR_OS); 

        xkThread_AdjustPriority(thread);

        xkThread_AdjustAffinity(thread);

        kTestTrue(ResumeThread(obj->handle) != k32U_MAX, kERROR_OS);
    }
    kCatch(&status)
    {
        obj->function = kNULL; 
        obj->context = kNULL; 
        obj->handle = kNULL;

        kEndCatch(status);
    }

    return kOK;
}

kFx(kStatus) xkThread_AdjustPriority(kThread thread)
{
    kObj(kThread, thread); 
    k32s nativeThreadPriority = xkThread_OsPriority(obj->priorityClass, obj->priorityOffset);

    kCheckTrue(SetThreadPriority(obj->handle, nativeThreadPriority) != 0, kERROR_OS); 

    if (xkTHREAD_LOG_ENABLED)
    {
        kLogf("kThread: %s priority set (logical=%d,%d; native=%d).", obj->name, obj->priorityClass, obj->priorityOffset, nativeThreadPriority); 
    }

    return kOK;
}

kFx(kStatus) xkThread_AdjustAffinity(kThread thread)
{
    kObj(kThread, thread); 
    kSize affinityMask = 0; 

    for (kSize i = 0; i < kBitArray_Length(obj->affinity); ++i)
    {
        if (kBitArray_At(obj->affinity, i))
        {
            affinityMask |= (1llu << i);
        }
    }
 
    kCheckTrue(SetThreadAffinityMask(obj->handle, affinityMask) != 0, kERROR_OS); 

    if (xkTHREAD_LOG_ENABLED)
    {
        kLogf("kThread: %s affinity set (0x%llX).", obj->name, (k64u) affinityMask); 
    }

    return kOK;
}

kFx(kStatus) kThread_Join(kThread thread, k64u timeout, kStatus* exitCode)
{
    kObj(kThread, thread); 
    DWORD osTimeout = (DWORD) xkTimeToKernelTime(timeout); 
    DWORD waitResult = 0; 
    DWORD threadExit = kOK;

    if (obj->function)
    {
        waitResult = WaitForSingleObject(obj->handle, osTimeout); 
        
        if (waitResult == WAIT_TIMEOUT)
        {
            return kERROR_TIMEOUT; 
        }
        else if (waitResult != WAIT_OBJECT_0)
        {
            return kERROR_OS; 
        }       
        
        if (!GetExitCodeThread(obj->handle, &threadExit))
        {
            return kERROR; 
        }

        if (!CloseHandle(obj->handle))
        {
            return kERROR;
        }
         
        obj->function = kNULL; 
        obj->context = kNULL; 

        if (exitCode)
        {
            *exitCode = (kStatus)threadExit; 
        }
    }
    
    return kOK;
}

unsigned int __stdcall xkThread_EntryPoint(void *arg)
{
    kObj(kThread, arg); 
    
    unsigned int result = (unsigned int) obj->function(obj->context);

    if (xkTHREAD_LOG_ENABLED)
    {    
        kLogf("kThread: %s exited (%s).", obj->name, kStatus_Name(result)); 
    }

    return result; 
}

kFx(xkThreadId) xkThread_Id(kThread thread)
{
    kObj(kThread, thread); 
    return obj->id;     
}

kFx(xkThreadId) xkThread_CurrentId()
{
    return (xkThreadId) GetCurrentThreadId(); 
}

kFx(kBool) xkThread_CompareId(xkThreadId a, xkThreadId b)
{
    return a == b; 
}

#elif defined (K_TI_BIOS)

kFx(kStatus) xkThread_InitStatic()
{
    return kOK;
}

kFx(kStatus) xkThread_ReleaseStatic()
{
    return kOK;
}

kFx(kSize) kThread_ProcessorCount()
{
    //OS only supports one CPU
    return 1; 
}

kFx(kThreadPriorityClass) kThread_MinPriorityClass()
{
    return kTHREAD_PRIORITY_CLASS_LOW;
}

kFx(kThreadPriorityClass) kThread_MaxPriorityClass()
{
    return kTHREAD_PRIORITY_CLASS_HIGH;
}

kFx(k32s) kThread_MinPriorityOffset(kThreadPriorityClass priorityClass)
{
    switch (priorityClass)
    {
        case kTHREAD_PRIORITY_CLASS_LOW:    return xkTHREAD_MIN_LOW_PRIORITY_OFFSET; 
        case kTHREAD_PRIORITY_CLASS_HIGH:   return xkTHREAD_MIN_HIGH_PRIORITY_OFFSET;
        default:                            return xkTHREAD_MIN_NORMAL_PRIORITY_OFFSET; 
    }
}

kFx(k32s) kThread_MaxPriorityOffset(kThreadPriorityClass priorityClass)
{
    switch (priorityClass)
    {
        case kTHREAD_PRIORITY_CLASS_LOW:    return xkTHREAD_MAX_LOW_PRIORITY_OFFSET; 
        case kTHREAD_PRIORITY_CLASS_HIGH:   return xkTHREAD_MAX_HIGH_PRIORITY_OFFSET;
        default:                            return xkTHREAD_MAX_NORMAL_PRIORITY_OFFSET; 
    }
}

kFx(k32s) xkThread_OsPriority(kThreadPriorityClass priorityClass, k32s priorityOffset)
{
    switch (priorityClass)
    {
        case kTHREAD_PRIORITY_CLASS_LOW:    return xkTHREAD_OS_DEFAULT_LOW_PRIORITY + priorityOffset;
        case kTHREAD_PRIORITY_CLASS_HIGH:   return xkTHREAD_OS_DEFAULT_HIGH_PRIORITY + priorityOffset;
        default:                            return xkTHREAD_OS_DEFAULT_NORMAL_PRIORITY + priorityOffset;
    }
}

kFx(kBool) kThread_CanSetAffinty()
{
    return kFALSE;
}

kFx(kStatus) kThread_Sleep(k64u duration)
{
    k32u osDuration = xkTimeToKernelTime(duration); 
    
    if (osDuration > 0)
    {
        ti_sysbios_knl_Task_sleep(osDuration);
    }
    else
    {
        ti_sysbios_knl_Task_yield(); 
    }
    
    return kOK; 
}

kFx(kStatus) xkThread_Init(kThread thread, kType type, kAlloc allocator)
{
    kObjR(kThread, thread);
    kStatus status;

    kCheck(kObject_Init(thread, type, allocator)); 
    
    kTry
    {
        kTest(xkThread_InitShared(thread));

        kZero(obj->handle); 
        obj->joinSem = kNULL; 
        kAtomic32s_Init(&obj->exitCode, kOK); 

        kTest(kSemaphore_Construct(&obj->joinSem, 0, allocator));  
    }
    kCatch(&status)
    {
        xkThread_VRelease(thread);
        kEndCatch(status);
    }
    
    return kOK; 
}

kFx(kStatus) xkThread_VRelease(kThread thread)
{
    kObj(kThread, thread); 

    kCheck(kThread_Join(thread, kINFINITE, kNULL)); 
    
    kCheck(kObject_Destroy(obj->joinSem)); 

    kCheck(xkThread_ReleaseShared(thread));

    kCheck(kObject_VRelease(thread)); 

    return kOK; 
}

kFx(kStatus) kThread_Start(kThread thread, kThreadFx function, kPointer context)
{
    kObj(kThread, thread);
    kStatus status;

    kCheckState(!xkThread_IsStarted(thread));
    
    obj->function = function; 
    obj->context = context; 

    kTry
    {
        xdc_runtime_Error_Block eb; 
        ti_sysbios_knl_Task_Params params; 

        xdc_runtime_Error_init(&eb);

        ti_sysbios_knl_Task_Params_init(&params);
        params.arg0 = (UArg) thread; 
        params.priority = xkThread_OsPriority(obj->priorityClass, obj->priorityOffset);
            
        kTestTrue(!kIsNull(obj->handle = ti_sysbios_knl_Task_create(xkThread_EntryPoint, &params, &eb)), kERROR_OS);

        if (xkTHREAD_LOG_ENABLED)
        {   
            kLogf("kThread: %s created, logical priority = (%d,%d), native priority = %d.", obj->name, obj->priorityClass, obj->priorityOffset, (k32s) params.priority); 
        }
    }
    kCatch(&status)
    {
        obj->handle = kNULL; 
        obj->function = kNULL; 
        obj->context = kNULL; 

        kEndCatch(status);
    }

    return kOK;
}

kFx(kStatus) xkThread_AdjustPriority(kThread thread)
{
    kObj(kThread, thread); 
    k32s nativeThreadPriority = xkThread_OsPriority(obj->priorityClass, obj->priorityOffset);

    ti_sysbios_knl_Task_setPri(obj->handle, nativeThreadPriority);

    if (xkTHREAD_LOG_ENABLED)
    {
        kLogf("kThread: %s priority set (logical=%d,%d; native=%d).", obj->name, obj->priorityClass, obj->priorityOffset, nativeThreadPriority); 
    }

    return kOK;
}

kFx(kStatus) xkThread_AdjustAffinity(kThread thread)
{
    return kERROR_UNIMPLEMENTED;
}

kFx(kStatus) kThread_Join(kThread thread, k64u timeout, kStatus* exitCode)
{
    kObj(kThread, thread); 

    if (obj->function)
    {
        kCheck(kSemaphore_Wait(obj->joinSem, timeout));

        ti_sysbios_knl_Task_delete(&obj->handle);
        
        obj->function = kNULL; 
        obj->context = kNULL; 
        obj->handle = kNULL;

        if (exitCode)
        {
            *exitCode = kAtomic32s_Get(&obj->exitCode); 
        }
    }
    
    return kOK;
}

void xkThread_EntryPoint(UArg arg0, UArg arg1)
{
    kThread thread = (kThread) arg0; 
    kObj(kThread, thread);
    ti_sysbios_knl_Task_Stat taskStats;
    kStatus result;

    //required by SYS/BIOS NDK, in order to use sockets
    if (fdOpenSession(ti_sysbios_knl_Task_self()) != 1)    
    {
        kAtomic32s_Exchange(&obj->exitCode, kERROR_OS); 
        kAssert(kFALSE);  
        return; 
    }

    result = obj->function(obj->context); 

    kAtomic32s_Exchange(&obj->exitCode, result); 

    fdCloseSession(ti_sysbios_knl_Task_self());
 
    kSemaphore_Post(obj->joinSem); 

    ti_sysbios_knl_Task_stat(ti_sysbios_knl_Task_self(), &taskStats); 

    kAssert(taskStats.used <= taskStats.stackSize); 

    if (xkTHREAD_LOG_ENABLED)
    {    
        kLogf("kThread: %s exited (%s).", obj->name, kStatus_Name(result)); 
    }

    return; 
}

kFx(xkThreadId) xkThread_Id(kThread thread)
{
    kObj(kThread, thread); 
    return obj->handle; 
}

kFx(xkThreadId) xkThread_CurrentId()
{
    return ti_sysbios_knl_Task_self(); 
}

kFx(kBool) xkThread_CompareId(xkThreadId a, xkThreadId b)
{
    return a == b; 
}

#elif defined(K_VX_KERNEL)

kFx(kStatus) xkThread_InitStatic()
{
    return kOK;
}

kFx(kStatus) xkThread_ReleaseStatic()
{
    return kOK;
}

kFx(kSize) kThread_ProcessorCount()
{
    return (kSize) vxCpuConfiguredGet(); 
}

kFx(kThreadPriorityClass) kThread_MinPriorityClass()
{
    return kTHREAD_PRIORITY_CLASS_LOW;
}

kFx(kThreadPriorityClass) kThread_MaxPriorityClass()
{
    return kTHREAD_PRIORITY_CLASS_HIGH;
}

kFx(k32s) kThread_MinPriorityOffset(kThreadPriorityClass priorityClass)
{
    switch (priorityClass)
    {
        case kTHREAD_PRIORITY_CLASS_LOW:    return xkTHREAD_MIN_LOW_PRIORITY_OFFSET; 
        case kTHREAD_PRIORITY_CLASS_HIGH:   return xkTHREAD_MIN_HIGH_PRIORITY_OFFSET;
        default:                            return xkTHREAD_MIN_NORMAL_PRIORITY_OFFSET; 
    }
}

kFx(k32s) kThread_MaxPriorityOffset(kThreadPriorityClass priorityClass)
{
    switch (priorityClass)
    {
        case kTHREAD_PRIORITY_CLASS_LOW:    return xkTHREAD_MAX_LOW_PRIORITY_OFFSET; 
        case kTHREAD_PRIORITY_CLASS_HIGH:   return xkTHREAD_MAX_HIGH_PRIORITY_OFFSET;
        default:                            return xkTHREAD_MAX_NORMAL_PRIORITY_OFFSET; 
    }
}

kFx(k32s) xkThread_OsPriority(kThreadPriorityClass priorityClass, k32s priorityOffset)
{
    switch (priorityClass)
    {
        case kTHREAD_PRIORITY_CLASS_LOW:    return xkTHREAD_OS_DEFAULT_LOW_PRIORITY - priorityOffset;
        case kTHREAD_PRIORITY_CLASS_HIGH:   return xkTHREAD_OS_DEFAULT_HIGH_PRIORITY - priorityOffset;
        default:                            return xkTHREAD_OS_DEFAULT_NORMAL_PRIORITY - priorityOffset;
    }
}

kFx(kBool) kThread_CanSetAffinty()
{
    return kTRUE;
}

kFx(kStatus) kThread_Sleep(k64u duration)
{
    _Vx_ticks_t osDuration = (_Vx_ticks_t) xkTimeToKernelTime(duration); 
    
    if (taskDelay(osDuration) != OK)
    {
        return kERROR_OS; 
    }
    
    return kOK; 
}

kFx(kStatus) xkThread_Init(kThread thread, kType type, kAlloc allocator)
{
    kObjR(kThread, thread); 
    kStatus exception; 
            
    kCheck(kObject_Init(thread, type, allocator)); 
 
    kTry
    {   
        kTest(xkThread_InitShared(thread));

        obj->id = TASK_ID_ERROR; 
        obj->joinSem = kNULL; 
        kAtomic32s_Init(&obj->exitCode, kOK); 

        kTest(kSemaphore_Construct(&obj->joinSem, 0, allocator)); 
    }
    kCatch(&exception)
    {
        xkThread_VRelease(thread); 
        kEndCatch(exception); 
    }

    return kOK; 
}        
        
kFx(kStatus) xkThread_VRelease(kThread thread)
{
    kObj(kThread, thread); 

    kCheck(kThread_Join(thread, kINFINITE, kNULL)); 
    
    kCheck(kObject_Destroy(obj->joinSem)); 

    kCheck(xkThread_ReleaseShared(thread));

    kCheck(kObject_VRelease(thread)); 

    return kOK; 
}

kFx(kStatus) kThread_Start(kThread thread, kThreadFx function, kPointer context)
{
    kObj(kThread, thread); 
    kStatus exception; 

    kCheckState(!xkThread_IsStarted(thread));

    obj->function = function; 
    obj->context = context; 

    kTry
    {    
        k32s nativeThreadPriority = xkThread_OsPriority(obj->priorityClass, obj->priorityOffset);
        kChar* taskName = (kStrLength(obj->name) == 0) ? kNULL : obj->name;         //if null is given for name, VxWorks will improvise a unique name
 
        obj->id = taskSpawn(taskName, nativeThreadPriority, xkTHREAD_DEFAULT_OPTIONS, xkTHREAD_DEFAULT_STACK_SIZE, (FUNCPTR)xkThread_EntryPoint, (_Vx_usr_arg_t)thread, 0, 0, 0, 0, 0, 0, 0, 0, 0); 

        if (obj->id == TASK_ID_ERROR)
        {
            kThrow(kERROR_OS);   
        }

        if (xkTHREAD_LOG_ENABLED)
        {   
            kLogf("kThread: %s created, logical priority = (%d,%d), native priority = %d.", obj->name, obj->priorityClass, obj->priorityOffset, nativeThreadPriority);
        }

        xkThread_AdjustAffinity(thread);
    }
    kCatch(&exception)
    {
        obj->function = kNULL; 
        obj->context = kNULL; 
        obj->id = TASK_ID_ERROR; 

        kEndCatch(exception); 
    }

    return kOK; 
}

kFx(kStatus) xkThread_AdjustPriority(kThread thread)
{
    kObj(kThread, thread); 
    k32s nativeThreadPriority = xkThread_OsPriority(obj->priorityClass, obj->priorityOffset);

    kCheckTrue(taskPrioritySet(obj->id, nativeThreadPriority) == OK, kERROR_OS); 

    if (xkTHREAD_LOG_ENABLED)
    {
        kLogf("kThread: %s priority set (logical=%d,%d; native=%d).", obj->name, obj->priorityClass, obj->priorityOffset, nativeThreadPriority); 
    }

    return kOK;
}

kFx(kStatus) xkThread_AdjustAffinity(kThread thread)
{
    kObj(kThread, thread); 
    cpuset_t affinity;

    CPUSET_ZERO(affinity);

    //VxWorks 6.9 only supports setting affinity to a single CPU; if multiple CPUs were requested, 
    //we'll leave the cpuset_t empty, which requests execution on all CPUs.
    if (kBitArray_TrueCount(obj->affinity) == 1)
    {
        for (kSize i = 0; i < kBitArray_Length(obj->affinity); ++i)
        {
            if (kBitArray_At(obj->affinity, i))
            {
                CPUSET_SET(affinity, i);
            }
        }
    }
    
    kCheckTrue(taskCpuAffinitySet(obj->id, affinity) == OK, kERROR_OS); 

    if (xkTHREAD_LOG_ENABLED)
    {
        kLogf("kThread: %s affinity set (0x%X).", obj->name, (k32u)affinity); 
    }

    return kOK;
}

kFx(kStatus) kThread_Join(kThread thread, k64u timeout, kStatus* exitCode)
{
    kObj(kThread, thread);

    if (obj->function)
    {
        kCheck(kSemaphore_Wait(obj->joinSem, timeout)); 
                
        obj->function = kNULL;         
        obj->context = kNULL; 

        if (exitCode)
        {
            *exitCode = kAtomic32s_Get(&obj->exitCode); 
        }
    }
    
    return kOK;   
}        

int xkThread_EntryPoint(_Vx_usr_arg_t arg0)
{
    kThread thread = (kThread) arg0; 
    kObj(kThread, thread);
    kStatus result;
       
    result = obj->function(obj->context); 

    kAtomic32s_Exchange(&obj->exitCode, result); 
    
    kSemaphore_Post(obj->joinSem); 

    if (xkTHREAD_LOG_ENABLED)
    {    
        kLogf("kThread: %s exited (%s).", obj->name, kStatus_Name(result)); 
    }
    
    return kOK;  
}

kFx(xkThreadId) xkThread_Id(kThread thread)
{
    kObj(kThread, thread); 
    return obj->id;     
}

kFx(xkThreadId) xkThread_CurrentId()
{
    return taskIdSelf(); 
}

kFx(kBool) xkThread_CompareId(xkThreadId a, xkThreadId b)
{
    return a == b; 
}

#elif defined(K_LINUX)

/*
* Ideally, we would use cap_get_proc and cap_get_flag to query for CAP_SYS_NICE, which reflects both a) whether 
* priority can be increased, and b) whether affinity can be manipulated. However, these functions require a non-standard 
* package (libcap-dev) to even compile. Maybe in future, if libcap is better integrated. In the meantime, we'll 
* just try increasing the 'nice' value to its theoretical maximum; if it works, we'll assume the process has CAP_SYS_NICE.
*/
kFx(kBool) kThread_HasNiceCap()
{
    const k32s maxPriority = -xkTHREAD_MAX_NORMAL_PRIORITY_OFFSET; 
    k32s originalPriority = getpriority(PRIO_PROCESS, 0);

    //try increasing nice
    setpriority(PRIO_PROCESS, 0, maxPriority);

    //see if it worked
    kBool hasNiceCap = (getpriority(PRIO_PROCESS, 0) == maxPriority); 
    
    //restore original priority
    setpriority(PRIO_PROCESS, 0, originalPriority);

    return hasNiceCap;
}

kFx(kStatus) xkThread_InitStatic()
{
    kStaticObj(kThread); 

    sobj->canIncreasePriority = sobj->canSetAffinity = kThread_HasNiceCap();
   
    return kOK;
}

kFx(kStatus) xkThread_ReleaseStatic()
{
    return kOK;
}

kFx(kSize) kThread_ProcessorCount()
{
    long cpuCount = -1; 

#ifdef _SC_NPROCESSORS_ONLN
    cpuCount = sysconf(_SC_NPROCESSORS_ONLN);
#endif

    return (cpuCount > 0) ? (kSize) cpuCount : 1; 
}

kFx(kThreadPriorityClass) kThread_MinPriorityClass()
{
    return kTHREAD_PRIORITY_CLASS_LOW;
}

kFx(kThreadPriorityClass) kThread_MaxPriorityClass()
{
    kStaticObj(kThread);

    return (sobj->canIncreasePriority) ? kTHREAD_PRIORITY_CLASS_HIGH : kTHREAD_PRIORITY_CLASS_NORMAL;
}

kFx(k32s) kThread_MinPriorityOffset(kThreadPriorityClass priorityClass)
{
    kStaticObj(kThread);

    switch (priorityClass)
    {
        case kTHREAD_PRIORITY_CLASS_LOW:    
            return xkTHREAD_MIN_LOW_PRIORITY_OFFSET; 

        case kTHREAD_PRIORITY_CLASS_HIGH:   
            return (sobj->canIncreasePriority) ? xkTHREAD_MIN_HIGH_PRIORITY_OFFSET : k32S_NULL;

        default:                            
            return xkTHREAD_MIN_NORMAL_PRIORITY_OFFSET; 
    }
}

kFx(k32s) kThread_MaxPriorityOffset(kThreadPriorityClass priorityClass)
{
    kStaticObj(kThread);

    switch (priorityClass)
    {
        case kTHREAD_PRIORITY_CLASS_LOW:    
            return xkTHREAD_MAX_LOW_PRIORITY_OFFSET;
            
        case kTHREAD_PRIORITY_CLASS_HIGH:   
            return (sobj->canIncreasePriority) ? xkTHREAD_MAX_HIGH_PRIORITY_OFFSET : k32S_NULL;

        default:                            
            return (sobj->canIncreasePriority) ? xkTHREAD_MAX_NORMAL_PRIORITY_OFFSET : 0; 
    }
}

kFx(kBool) kThread_CanSetAffinty()
{
    kStaticObj(kThread);

    return sobj->canSetAffinity;
}

kFx(kStatus) kThread_Sleep(k64u duration)
{
    useconds_t durationMs = (useconds_t) ((duration + 999)/1000); 
    
    usleep(durationMs*1000); 
    
    return kOK; 
}

kFx(kStatus) xkThread_Init(kThread thread, kType type, kAlloc alloc)
{
    kObjR(kThread, thread); 
    kStatus exception; 
    
    kCheck(kObject_Init(thread, type, alloc)); 

    kTry
    {
        kTest(xkThread_InitShared(thread));
        
        obj->hasStarted = kNULL;
        obj->hasJoined = kNULL; 
        kZero(obj->pthreadId); 
        obj->schedulingPolicy = SCHED_OTHER;
        kZero(obj->schedulingParam);
        obj->nice = 0;
        CPU_ZERO(&obj->cpuSet);
        kAtomic32s_Init(&obj->threadId, 0);

        kTest(kSemaphore_Construct(&obj->hasStarted, 0, alloc)); 
        kTest(kSemaphore_Construct(&obj->hasJoined, 0, alloc)); 
    }
    kCatch(&exception)
    {
        xkThread_VRelease(thread); 
        kEndCatch(exception); 
    }

    return kOK; 
}

kFx(kStatus) xkThread_VRelease(kThread thread)
{
    kObj(kThread, thread); 

    kCheck(kThread_Join(thread, kINFINITE, kNULL)); 

    kCheck(kObject_Destroy(obj->hasStarted)); 
    kCheck(kObject_Destroy(obj->hasJoined)); 

    kCheck(xkThread_ReleaseShared(thread));

    kCheck(kObject_VRelease(thread)); 

    return kOK; 
}

kFx(kStatus) kThread_Start(kThread thread, kThreadFx function, kPointer context)
{
    kObj(kThread, thread); 
    kStatus status;
    pthread_attr_t attributes;
    pthread_attr_t* attr = kNULL;
    kSize stackSize = kApiLib_DefaultStackSize();

    obj->function = function; 
    obj->context = context; 

    kTry
    {
        if (stackSize != 0)
        {
            //init thread attributes
            kTestTrue(pthread_attr_init(&attributes) == 0, kERROR_OS);
            attr = &attributes;

            //override stack size
            kTestTrue(pthread_attr_setstacksize(attr, stackSize) == 0, kERROR_OS);
        }

        //start thread
        kTestTrue(pthread_create(&obj->pthreadId, attr, xkThread_EntryPoint, thread) == 0, kERROR_OS);

        //set name
        pthread_setname_np(obj->pthreadId, obj->name);

        //wait for thread to start, so that we can access the threads's native OS ID (required for 
        //any thread configuration not supported via pthreads, e.g., setting CFS 'nice' value)
        kSemaphore_Wait(obj->hasStarted, kINFINITE);
        
        xkThread_AdjustPriority(thread);
        xkThread_AdjustAffinity(thread);
    }
    kCatchEx(&status)
    {
        kZero(obj->pthreadId);
        obj->function = kNULL; 
        obj->context = kNULL; 

        kEndCatchEx(status);
    }
    kFinallyEx
    {
        if (!kIsNull(attr))
        {
            pthread_attr_destroy(attr);
        }

        kEndFinallyEx();
    }

    return kOK; 
}

kFx(kStatus) xkThread_AdjustPriority(kThread thread)
{
    kObj(kThread, thread); 
    kStatus status;
    int result = 0;

    kTry
    { 
        kTest(xkThread_UpdatePriorityParams(thread)); 
              
        kTestTrue((result = pthread_setschedparam(obj->pthreadId, obj->schedulingPolicy, &obj->schedulingParam)) == 0, kERROR_OS);

        if (obj->schedulingPolicy == SCHED_OTHER)
        {
            //return value from setpriority is documented to be ambiguous; ignore it
            setpriority(PRIO_PROCESS, kAtomic32s_Get(&obj->threadId), obj->nice);
        }
        
        if (xkTHREAD_LOG_ENABLED)
        {   
            const kChar* policyName = (obj->schedulingPolicy == SCHED_IDLE) ? "SCHED_IDLE" : 
                                      (obj->schedulingPolicy == SCHED_RR) ? "SCHED_RR" : "SCHED_OTHER";
                                   
            kLogf("kThread: %s priority set, logical = (%d,%d); actual = (%s, %d, %d)", obj->name, 
                obj->priorityClass, obj->priorityOffset, policyName, obj->schedulingParam.sched_priority, 
                getpriority(PRIO_PROCESS, kAtomic32s_Get(&obj->threadId))); 
        }         
    }
    kCatch(&status)
    {
        if (result == ESRCH)
        {
            status = kOK;  //thread function already exited; ignore
        }
        else if ((result == EPERM) && xkTHREAD_LOG_ENABLED)
        {
            kLogf("kThread: %s insufficient privilege to set priority.", obj->name); 
        }
        else if (xkTHREAD_LOG_ENABLED)
        {   
            kLogf("kThread: %s failed to set priority (code=%d).", obj->name, result); 
        }

        kEndCatch(status);
    }
       
    return kOK;
}

kFx(kStatus) xkThread_UpdatePriorityParams(kThread thread)
{
    kObj(kThread, thread); 
  
    switch (obj->priorityClass)
    {
        case kTHREAD_PRIORITY_CLASS_LOW:
        {
            obj->schedulingPolicy = SCHED_IDLE; 
            obj->schedulingParam.sched_priority = 0;
            obj->nice = 0;
    
            break;
        }
    
        case kTHREAD_PRIORITY_CLASS_HIGH:
        {
            k32s requestedPriority = xkTHREAD_OS_DEFAULT_HIGH_PRIORITY + obj->priorityOffset;
            k32s maxPriority = sched_get_priority_max(SCHED_RR);

            obj->schedulingPolicy = SCHED_RR; 
            obj->schedulingParam.sched_priority = kMin_(requestedPriority, maxPriority);
            obj->nice = 0;

            break;
        }
    
        default:
        {
            obj->schedulingPolicy = SCHED_OTHER; 
            obj->schedulingParam.sched_priority = 0;
            obj->nice = -1*obj->priorityOffset;
    
            break;
        }
    }

    return kOK;
}

kFx(kStatus) xkThread_AdjustAffinity(kThread thread)
{
    kObj(kThread, thread); 
    k32s result  = 0;
    kStatus status = kOK;

    kTry
    {  
        kTest(xkThread_UpdateAffinityParams(thread)); 

        kTestTrue((result = pthread_setaffinity_np(obj->pthreadId, sizeof(obj->cpuSet), &obj->cpuSet))== 0, kERROR_OS);      
        
        if (xkTHREAD_LOG_ENABLED)
        {   
            kText128 buffer; 
            xkThread_PrintAffinity(thread, buffer, sizeof(buffer));

            kLogf("kThread: %s affinity set [%s].", obj->name, buffer); 
        }
    }
    kCatch(&status)
    {
        if (result == ESRCH)
        {
            status = kOK;  //thread function already exited; ignore
        }
        else if (xkTHREAD_LOG_ENABLED)
        {   
            kLogf("kThread: %s failed to set priority (code=%d).", obj->name, result); 
        }

        kEndCatch(status);
    }

    return kOK;
}

kFx(kStatus) xkThread_UpdateAffinityParams(kThread thread)
{
    kObj(kThread, thread); 

    CPU_ZERO(&obj->cpuSet);

    for (kSize i = 0; i < kBitArray_Length(obj->affinity); ++i)
    {
        if (kBitArray_At(obj->affinity, i))
        {
            CPU_SET(i, &obj->cpuSet);
        }
    }

    return kOK;
}

kFx(kStatus) xkThread_PrintAffinity(kThread thread, kChar* buffer, kSize capacity)
{
    kObj(kThread, thread);
    k64u coreCount = kThread_ProcessorCount();

    buffer[0] = '\0';
    
    for (kSize i = 0; i < coreCount; ++i)
    {
        kBool isSet = CPU_ISSET(i, &obj->cpuSet); 

        kStrCat(buffer, capacity, isSet ? "1" : "0"); 
    }

    return kOK;
}

kFx(kStatus) kThread_Join(kThread thread, k64u timeout, kStatus* exitCode)
{
    kObj(kThread, thread); 
    void* threadExit; 
    
    if (obj->function)
    {
        if (timeout != kINFINITE)
        {
            kCheck(kSemaphore_Wait(obj->hasJoined, timeout)); 
        }

        kCheckTrue(pthread_join(obj->pthreadId, (void**)&threadExit) == 0, kERROR_OS);

        kZero(obj->pthreadId);
        obj->function = kNULL; 
        obj->context = kNULL; 

        if (exitCode)
        {
            *exitCode = (kStatus) (kSSize) threadExit; 
        }
    }
    
    return kOK; 
}

void* xkThread_EntryPoint(void* arg)
{
    kThread thread = arg;
    kObj(kThread, thread); 
  
    kAtomic32s_Exchange(&obj->threadId, syscall(SYS_gettid));

    //signal that thread has started 
    kSemaphore_Post(obj->hasStarted); 
    
    kStatus result = obj->function(obj->context);

    //signal thread completion
    kSemaphore_Post(obj->hasJoined); 

    if (xkTHREAD_LOG_ENABLED)
    {    
        kLogf("kThread: %s exited (%s).", obj->name, kStatus_Name(result)); 
    }
        
    return (void*) (kSSize) result;  
}

kFx(xkThreadId) xkThread_Id(kThread thread)
{
    kObj(kThread, thread); 
    return obj->pthreadId;      
}

kFx(xkThreadId) xkThread_CurrentId()
{
    return pthread_self(); 
}

kFx(kBool) xkThread_CompareId(xkThreadId a, xkThreadId b)
{
    return (pthread_equal(a, b) != 0); 
}

#elif defined(K_POSIX)

kFx(kStatus) xkThread_InitStatic()
{
    return kOK;
}

kFx(kStatus) xkThread_ReleaseStatic()
{
    return kOK;
}

kFx(kSize) kThread_ProcessorCount()
{
    long cpuCount = -1; 

#ifdef _SC_NPROCESSORS_ONLN
    cpuCount = sysconf(_SC_NPROCESSORS_ONLN);
#endif

    return (cpuCount > 0) ? (kSize) cpuCount : 1; 
}

kFx(kThreadPriorityClass) kThread_MinPriorityClass()
{
    return kTHREAD_PRIORITY_CLASS_NORMAL;
}

kFx(kThreadPriorityClass) kThread_MaxPriorityClass()
{
    return kTHREAD_PRIORITY_CLASS_NORMAL;
}

kFx(k32s) kThread_MinPriorityOffset(kThreadPriorityClass priorityClass)
{
    return (priorityClass == kTHREAD_PRIORITY_CLASS_NORMAL) ? 0 : k32S_NULL;
}

kFx(k32s) kThread_MaxPriorityOffset(kThreadPriorityClass priorityClass)
{
    return (priorityClass == kTHREAD_PRIORITY_CLASS_NORMAL) ? 0 : k32S_NULL;
}

kFx(kBool) kThread_CanSetAffinty()
{
    return kFALSE;
}

kFx(kStatus) kThread_Sleep(k64u duration)
{
    useconds_t durationMs = (useconds_t) ((duration + 999)/1000); 
    
    usleep(durationMs*1000); 
    
    return kOK; 
}

kFx(kStatus) xkThread_Init(kThread thread, kType type, kAlloc alloc)
{
    kObjR(kThread, thread); 
    kStatus exception; 
    
    kCheck(kObject_Init(thread, type, alloc)); 

    kTry
    {
        kTest(xkThread_InitShared(thread));

        obj->hasJoined = kNULL; 
        kZero(obj->pthreadId); 

        kTest(kSemaphore_Construct(&obj->hasJoined, 0, alloc)); 
    }
    kCatch(&exception)
    {
        xkThread_VRelease(thread); 
        kEndCatch(exception); 
    }

    return kOK; 
}

kFx(kStatus) xkThread_VRelease(kThread thread)
{
    kObj(kThread, thread); 

    kCheck(kThread_Join(thread, kINFINITE, kNULL)); 

    kCheck(kObject_Destroy(obj->hasJoined)); 

    kCheck(xkThread_ReleaseShared(thread));

    kCheck(kObject_VRelease(thread)); 

    return kOK; 
}

kFx(kStatus) kThread_Start(kThread thread, kThreadFx function, kPointer context)
{
    kObj(kThread, thread);
    kStatus status;

    kCheckState(!xkThread_IsStarted(thread));
    
    obj->function = function; 
    obj->context = context; 

    kTry
    {
        kTestTrue(pthread_create(&obj->pthreadId, kNULL, xkThread_EntryPoint, thread) == 0, kERROR_OS);

        if (xkTHREAD_LOG_ENABLED)
        {   
            kLogf("kThread: %s created.", obj->name); 
        }
    }
    kCatch(&status)
    {
        kZero(obj->pthreadId);
        obj->function = kNULL; 
        obj->context = kNULL; 

        kEndCatch(status);
    }

    return kOK;
}

kFx(kStatus) xkThread_AdjustPriority(kThread thread)
{
    return kERROR_UNIMPLEMENTED; 
}

kFx(kStatus) xkThread_AdjustAffinity(kThread thread)
{
    return kERROR_UNIMPLEMENTED; 
}

kFx(kStatus) kThread_Join(kThread thread, k64u timeout, kStatus* exitCode)
{
    kObj(kThread, thread); 
    void* threadExit; 
    
    if (obj->function)
    {
        if (timeout != kINFINITE)
        {
            kCheck(kSemaphore_Wait(obj->hasJoined, timeout)); 
        }

        kCheckTrue(pthread_join(obj->pthreadId, (void**)&threadExit) == 0, kERROR_OS);

        kZero(obj->pthreadId);
        obj->function = kNULL; 
        obj->context = kNULL; 

        if (exitCode)
        {
            *exitCode = (kStatus) (kSSize) threadExit; 
        }
    }
    
    return kOK; 
}

void* xkThread_EntryPoint(void* arg)
{
    kThread thread = arg;
    kObj(kThread, thread); 
  
    kStatus result = obj->function(obj->context);

    //signal thread completion
    kSemaphore_Post(obj->hasJoined); 

    if (xkTHREAD_LOG_ENABLED)
    {    
        kLogf("kThread: %s exited (%s).", obj->name, kStatus_Name(result)); 
    }
        
    return (void*) (kSSize) result;  
}

kFx(xkThreadId) xkThread_Id(kThread thread)
{
    kObj(kThread, thread); 
    return obj->pthreadId;      
}

kFx(xkThreadId) xkThread_CurrentId()
{
    return pthread_self(); 
}

kFx(kBool) xkThread_CompareId(xkThreadId a, xkThreadId b)
{
    return (pthread_equal(a, b) != 0); 
}

#endif


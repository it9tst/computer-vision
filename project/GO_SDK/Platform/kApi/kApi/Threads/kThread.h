/** 
 * @file    kThread.h
 * @brief   Declares the kThread class. 
 *
 * @internal
 * Copyright (C) 2005-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_THREAD_H
#define K_API_THREAD_H

#include <kApi/kApiDef.h>

/** Thread entry-point signature; used by kThread_Start.  */
typedef kStatus(kCall* kThreadFx)(kPointer context); 

#include <kApi/Threads/kThread.x.h>

/**
 * @class   kThread
 * @extends kObject
 * @ingroup kApi-Threads
 * @brief   Represents a thread.
 */
//typedef kObject kThread;        --forward-declared in kApiDef.x.h 

/** 
 * Reports the number of logical processors in the current system. 
 * 
 * @public      @memberof kThread
 * @return      Processor count (or 1, if unknown). 
 */
kFx(kSize) kThread_ProcessorCount(); 

/** 
 * Causes the current thread to yield control for approximately the specified duration.
 * 
 * The duration specified in this function is approximate. Sleep duration is governed by the underlying 
 * operating system, and may be affected by kernel timer resolution or thread scheduling policies. In general, 
 * if CPU utilization is low (quiet system), then sleep duration is most often +/- 1 kernel timer tick. At the 
 * time of this writing, most desktop operating systems have a 10 ms kernel timer resolution, while most 
 * embedded systems have a 1-10 ms kernel timer resolution. 
 * 
 * If sleep duration must be constrained to be <em>at least</em> the specified duration, consider using 
 * kThread_SleepAtLeast. 
 *
 * @public              @memberof kThread
 * @param   duration    Approximate time to sleep, in microseconds
 * @return              Operation status. 
 */
kFx(kStatus) kThread_Sleep(k64u duration);   

/**
* Causes the current thread to yield control for at least the specified duration.
*
* This function sleeps until <em>at least</em> the specified duration has elapsed. Consider using this function 
* when a sleep operation must provide a guaranteed minimum hold time. 
* 
* @public              @memberof kThread
* @param   duration    Minimum time to sleep, in microseconds
* @return              Operation status.
*/
kFx(kStatus) kThread_SleepAtLeast(k64u duration);

/** 
 * Gets a unique identifier that represents the currently executing thread.
 * 
 * @public              @memberof kThread
 * @return              Unique thread identifier. 
 */
kFx(kThreadId) kThread_CurrentId(); 

/**
* Reports the minimum supported priority class. 
*
* @public                   @memberof kThread
* @return                   Minimum priority class.
*/
kFx(kThreadPriorityClass) kThread_MinPriorityClass(); 

/**
* Reports the maximum supported priority class. 
*
* @public                   @memberof kThread
* @return                   Maximum priority class.
*/
kFx(kThreadPriorityClass) kThread_MaxPriorityClass(); 

/**
* Reports the minimum supported priority offset for the specified priority class. 
*
* @public                   @memberof kThread
* @param   priorityClass    Thread priority class.
* @return                   Minimum priority offset for the specified class.
*/
kFx(k32s) kThread_MinPriorityOffset(kThreadPriorityClass priorityClass); 

/**
* Reports the maximum supported priority offset for the specified priority class. 
*
* @public                   @memberof kThread
* @param   priorityClass    Thread priority class.
* @return                   Maximum priority offset for the specified class.
*/
kFx(k32s) kThread_MaxPriorityOffset(kThreadPriorityClass priorityClass); 

/**
* Reports whether core affinity can be set. 
*
* @public   @memberof kThread
* @return   kTRUE if affinity manipulation is supported.
*/
kFx(kBool) kThread_CanSetAffinty(); 

/** 
 * Constructs a kThread object.
 *
 * Note that thread priorities and affinities are not in inherited from the calling 
 * thread. To specify priorty and affinity values that differ from kThread defaults, 
 * call the appropriate configuration methods before starting this thread.
 * 
 * @public              @memberof kThread
 * @param   thread      Destination for the constructed object handle. 
 * @param   allocator   Memory allocator (or kNULL for default). 
 * @return              Operation status. 
 */
kFx(kStatus) kThread_Construct(kThread* thread, kAlloc allocator);

/** 
 * Sets the name of the thread.
 * 
 * Unique names should be assigned to threads in order to aid debugging. Note that some 
 * operating systems restrict name lengths to only a few characters (e.g., Linux: 16); 
 * in this case, the given name may be truncated when the start is started.
 * 
 * This method can only be called before the thread has been started. 
 * 
 * @public              @memberof kThread
 * @param   thread      Thread object. 
 * @param   name        Thread name.
 * @return              Operation status. 
 */
kFx(kStatus) kThread_SetName(kThread thread, const kChar* name); 

/** 
 * Reports the name associated with the thread.
 *
 * @public              @memberof kThread
 * @param   thread      Thread object. 
 * @return              Thread name.
 */
kFx(const kChar*) kThread_Name(kThread thread);

/** 
 * Sets the priority associated with the thread.
 *
 * Thread priorities are specified by providing a priority class and priority offset. The class 
 * determines the general category of thread behavior: Low, Normal, or High priority. Signed priority 
 * offsets can be used to specify more granular priority distinctions within a priority class. Positive 
 * numeric priority offsets represent higher logical priorities, negative priority offsets represent 
 * lower logical priorities; offset zero represents the default priority within a class. Minimum and 
 * maximum priority offsets within each class vary according to the capabilities of the underlying 
 * operating system. Priority support is provided on a best-effort basis; if the OS refuses the specified 
 * priority configuration, it will not be reported as an error. 
 * 
 * The Normal priority class should be used by default. Normal threads cannot be starved by other Normal 
 * threads; as such, it is safe to use priority offsets within this class to tune performance. For 
 * operating systems that offer only strict priority-based scheduling, the Normal class will have a 
 * single priority level (offset zero). For systems where thread execution times can be weighted without 
 * the potential for starvation (e.g., Linux CFS), the Normal class will support a range of priorities. 
 * 
 * The Low priority class can be used for non-critical tasks that should receive CPU time only when 
 * there are no Normal or High priority tasks available to run. For systems that offer priority-based 
 * scheduling, the Low priority class will normally offer multiple distinct levels. This priority class 
 * should be used with caution, due to the potential for starvation of Low priority tasks by Normal or 
 * High priority tasks. It is strongly recommended to avoid using negative priority offsets within 
 * the Low priority class, as such threads may interfere with normal system operation on embedded systems.
 * 
 * The High priority class can be used for time-critical tasks that should preempt Normal and Low 
 * priority threads. For systems that offer priority-based scheduling, the High priority class will 
 * normally offer multiple distinct priority levels. This class should be used with caution due to the 
 * potential for High priority tasks to starve Normal and Low priority tasks. In particular, it is 
 * strongly recommended to avoid positive priority offsets within the High priority class, as such 
 * threads are likely to interfere with critical system functions. Valid uses of High priority threads 
 * are rare; soft-realtime hardware support is the most common use case. 
 * 
 * This method can be called before or after starting the thread.
 * 
 * @public                  @memberof kThread
 * @param   thread          Thread object. 
 * @param   priorityClass   Thread priority class (Low, Normal, High).
 * @param   priorityOffset  Signed priority offset; a higher offset represents a higher priority within class.
 * @return                  Operation status. 
 * @see                     kThread_MinPriorityClass, kThread_MaxPriorityClass, kThread_MinPriorityOffset, kThread_MaxPriorityOffset
 */
kFx(kStatus) kThread_SetPriority(kThread thread, kThreadPriorityClass priorityClass, k32s priorityOffset); 

/** 
 * Sets the CPUs with which the thread should have affinity.
 *
 * Affinity support is provided on a best-effort basis; if the OS refuses or further restricts the 
 * specified affinity configuration, it will not be reported as an error. 
 * 
 * This method can be called before or after starting the thread.
 * 
 * @public              @memberof kThread
 * @param   thread      Thread object. 
 * @param   affinity    Array of CPUs with which this thread should have affinity (or kNULL or empty array to specify no affinity).
 * @return              Operation status. 
 * @see                 kThread_CanSetAffinty
 */
kFx(kStatus) kThread_SetAffinity(kThread thread, kBitArray affinity); 

/** 
 * Begins executing a thread using the specified callback function. 
 * 
 * Note: This Start overload does not set the thread name. Setting the thread name is recommended
 * for debugging. Accordingly, is it recommended to call kThread_SetName before calling this 
 * overload, or to use a different overload that includes the thread name.
 * 
 * @public                  @memberof kThread
 * @param   thread          Thread object. 
 * @param   function        The thread entry function. 
 * @param   context         An argument to be passed to the thread entry function.
 * @return                  Operation status. 
 */
kFx(kStatus) kThread_Start(kThread thread, kThreadFx function, kPointer context);

/** 
 * Begins executing a thread using the specified callback function and thread name. 
 *
 * @public                  @memberof kThread
 * @param   thread          Thread object. 
 * @param   function        The thread entry function. 
 * @param   context         An argument to be passed to the thread entry function.
 * @param   name            Descriptive name for the thread (kNULL for default).
 * @return                  Operation status. 
 * @see                     kThread_SetName
 */
#if defined(K_CPP)
kInlineFx(kStatus) kThread_Start(kThread thread, kThreadFx function, kPointer context, const kChar* name)
{
    kObj(kThread, thread); 

    kCheck(kThread_SetName(thread, name));
 
    return kThread_Start(thread, function, context);
}
#endif

/** 
 * Begins executing a thread using the specified callback function and options. 
 *
 * @public                  @memberof kThread
 * @param   thread          Thread object. 
 * @param   function        The thread entry function. 
 * @param   context         An argument to be passed to the thread entry function.
 * @param   name            Descriptive name for the thread (kNULL for default).
 * @param   priorityClass   Thread priority class (Low, Normal, High).
 * @param   priorityOffset  Signed priority offset; a higher offset represents a higher priority within class.
 * @return                  Operation status. 
 * @see                     kThread_SetName, kThread_SetPriority
 */
#if defined(K_CPP)
kInlineFx(kStatus) kThread_Start(kThread thread, kThreadFx function, kPointer context, const kChar* name,
                  kThreadPriorityClass priorityClass, k32s priorityOffset)
{
    kObj(kThread, thread); 

    kCheck(kThread_SetName(thread, name));
    kCheck(kThread_SetPriority(thread, priorityClass, priorityOffset));
 
    return kThread_Start(thread, function, context);
}
#endif

/** 
 * Blocks until the thread exits, or until a timeout occurs.
 *
 * @public              @memberof kThread
 * @param   thread      Thread object. 
 * @param   timeout     Timeout in microseconds, or kINFINITE to wait indefinitely.
 * @param   exitCode    Receives the return value from the thread function.
 * @return              Operation status. 
 */
kFx(kStatus) kThread_Join(kThread thread, k64u timeout, kStatus* exitCode); 

/** 
 * Gets a unique identifier representing the thread.
 * 
 * This field is only valid after the thread has been successfully started, and 
 * before the thread has been successfully joined. Otherwise, the value returned by 
 * this method is undefined.
 *
 * @public              @memberof kThread
 * @param   thread      Thread object. 
 * @return              Unique thread identifier; see note on validity.
 */
kFx(kThreadId) kThread_Id(kThread thread); 

/** 
 * Reports whether the specified thread is the currently executing thread.
 *
 * Equivalent to kThreadId_Compare(kThread_Id(thread), kThread_CurrentId()). 
 * 
 * @public              @memberof kThread
 * @param   other       Thread object. 
 * @return              kTRUE if the specified thread is the currently executing thread.
 */
kFx(kBool) kThread_IsSelf(kThread other); 

#endif

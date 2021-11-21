/** 
 * @file    kPeriodic.h
 * @brief   Declares the kPeriodic class. 
 *
 * @internal
 * Copyright (C) 2010-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_PERIODIC_H
#define K_API_PERIODIC_H

#include <kApi/kApiDef.h>

/**
 * @class   kPeriodic
 * @extends kObject
 * @ingroup kApi-Threads
 * @brief   Provides a periodic function call. 
 */
//typedef kObject kPeriodic;           --forward-declared in kApiDef.x.h 

/** Defines the signature of a callback function to receive timer notifications. */
typedef kStatus (kCall *kPeriodicElapsedFx)(kPointer context, kPeriodic timer); 

#include <kApi/Threads/kPeriodic.x.h>

/** 
 * Constructs a kPeriodic object.
 *
 * Unique names should be assigned to threads in order to aid debugging. Note that some 
 * operating systems restrict name lengths to only a few characters (e.g., Linux: 16). 
 * 
 * @public                  @memberof kPeriodic
 * @param   timer           Receives a handle to the constructed object. 
 * @param   name            Descriptive name for the thread (kNULL for default).
 * @param   allocator       Memory allocator (or kNULL for default). 
 * @return                  Operation status. 
 */
#if defined(K_CPP)
kInlineFx(kStatus) kPeriodic_Construct(kPeriodic* timer, const kChar* name, kAlloc allocator)
{
    return xkPeriodic_Construct(timer, name, kTHREAD_PRIORITY_CLASS_NORMAL, 0, allocator);
}
#endif

/** 
 * Constructs a kPeriodic object with additional options.
 *
 * Unique names should be assigned to threads in order to aid debugging. Note that some 
 * operating systems restrict name lengths to only a few characters (e.g., Linux: 16). 
 * 
 * Refer to kThread_SetPriority for information about thread priorities.
 * 
 * @public                  @memberof kPeriodic
 * @param   timer           Receives a handle to the constructed object. 
 * @param   name            Descriptive name for the thread (kNULL for default).
 * @param   priorityClass   Thread priority class (Low, Normal, High).
 * @param   priorityOffset  Signed priority offset; a higher offset represents a higher priority within class.
 * @param   allocator       Memory allocator (or kNULL for default). 
 * @return                  Operation status. 
 * @see                     kThread_SetPriority
 */
#if defined(K_CPP)
kInlineFx(kStatus) kPeriodic_Construct(kPeriodic* timer, const kChar* name, kThreadPriorityClass priorityClass, k32s priorityOffset, kAlloc allocator)
{
    return xkPeriodic_Construct(timer, name, priorityClass, priorityOffset, allocator);
}
#endif

/** 
 * Sets the priority associated with the periodic callback thread.
 *
 * This method can be called before or after starting the periodic callback.
 * 
 * Refer to kThread_SetPriority for information about thread priorities.
 * 
 * @public                  @memberof kPeriodic
 * @param   timer           kPeriodic object. 
 * @param   priorityClass   Thread priority class (Low, Normal, High).
 * @param   priorityOffset  Signed priority offset; a higher offset represents a higher priority within class.
 * @return                  Operation status. 
 * @see                     kThread_SetPriority
 */
kFx(kStatus) kPeriodic_SetPriority(kPeriodic timer, kThreadPriorityClass priorityClass, k32s priorityOffset); 

/** 
 * Sets the CPUs with which the periodic callback thread should have affinity.
 *
 * Affinity support is provided on a best-effort basis; if the OS refuses or further restricts the 
 * specified affinity configuration, it will not be reported as an error. 
 * 
 * This method can be called before or after starting the periodic callback.
 * 
 * Refer to kThread_SetAffinity for information about thread affinity.
 * 
 * @public              @memberof kPeriodic
 * @param   timer       kPeriodic object. 
 * @param   affinity    Array of CPUs with which the periodic callback thread should have affinity (or kNULL or empty array to specify no affinity).
 * @return              Operation status. 
 * @see                 kThread_SetAffinity
 */
kFx(kStatus) kPeriodic_SetAffinity(kPeriodic timer, kBitArray affinity); 

/** 
 * Starts callbacks at the specified period.
 * 
 * It is safe to call this function from within a timer callback.  It is valid to call Start multiple
 * times without first calling Stop, in order to change the timer period or callback function. 
 * 
 * Each subsequent timer period after the first callback is measured relative to the end 
 * of the previous timer callback. 
 *
 * @public              @memberof kPeriodic
 * @param   timer       kPeriodic object. 
 * @param   period      Callback period, in microseconds. 
 * @param   onElapsed   Callback function.
 * @param   context     User context handle supplied to the callback function.
 * @return              Operation status. 
 */
kFx(kStatus) kPeriodic_Start(kPeriodic timer, k64u period, kPeriodicElapsedFx onElapsed, kPointer context); 

/** 
 * Stops timer callbacks.
 * 
 * It is guaranteed that callbacks are stopped when this function returns. Deadlock can occur 
 * if a timer callback is in progress and is blocked indefinitely. It is safe to call this function 
 * from within a timer callback. 
 *
 * @public              @memberof kPeriodic
 * @param   timer       kPeriodic object. 
 * @return              Operation status. 
 */
kFx(kStatus) kPeriodic_Stop(kPeriodic timer); 

/** 
 * Reports whether periodic timer callbacks are currently enabled. 
 * 
 * Deadlock can occur if a timer callback is in progress and is blocked indefinitely when this
 * function is called. It is safe to call this function from within a timer callback. 
 * 
 * @public              @memberof kPeriodic
 * @param   timer       kPeriodic object. 
 * @return              kTRUE if the timer is curently enabled.
 */
kFx(kBool) kPeriodic_Enabled(kPeriodic timer); 

/** 
 * Reports the timer callback period. 
 * 
 * Deadlock can occur if a timer callback is in progress and is blocked indefinitely when this
 * function is called. It is safe to call this function from within a timer callback. 
 * 
 * The value returned by this function is only valid if the timer has been started. 
 * 
 * @public              @memberof kPeriodic
 * @param   timer       kPeriodic object. 
 * @return              Timer period (us).
 */
kFx(k64u) kPeriodic_Period(kPeriodic timer); 

#endif

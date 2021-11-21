/** 
 * @file    kThreadPool.h
 * @brief   Declares the kThreadPool type. 
 *
 * @internal
 * Copyright (C) 2017-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_THREADPOOL_H
#define K_API_THREADPOOL_H

#include <kApi/kApiDef.h>
#include <kApi/Threads/kThread.h>
#include <kApi/Threads/kThreadPool.x.h>

/**
 * @class   kThreadPool
 * @extends kObject
 * @ingroup kApi-Threads
 * @brief   Represents a thread pool. 
 *
 * The kThreadPool class implements a thread pool that can be used to process arbitrary jobs. 
 * 
 * A default thread pool instance is available via the kThreadPool_Default function. 
 */
//typedef kObject kThreadPool;        --forward-declared in kFsDef.x.h

/**
 * @struct  kThreadPoolTransaction
 * @ingroup kApi-Threads
 * @brief   Opaque pointer to a kThreadPool transaction. 
 */
typedef kPointer kThreadPoolTransaction;

/** 
 * Constructs a kThreadPool object.
 *
 * @public               @memberof kThreadPool
 * @param   pool         Destination for the constructed object handle.
 * @param   threadCount  Number of threads in the pool. 
 * @param   allocator    Memory allocator (or kNULL for default). 
 * @return               Operation status. 
 */
kFx(kStatus) kThreadPool_Construct(kThreadPool* pool, kSize threadCount, kAlloc allocator);

/** 
 * Sets the priority associated with the thread pool.
 *
 * This method is thread-safe. 
 *
 * @public                  @memberof kThread
 * @param   pool            kThreadPool object.
 * @param   priorityClass   Thread priority class (Low, Normal, High).
 * @param   priorityOffset  Signed priority offset; a higher offset represents a higher priority within class.
 * @return                  Operation status. 
 * @see                     kThread_SetPriority
 */
kFx(kStatus) kThreadPool_SetPriority(kThreadPool pool, kThreadPriorityClass priorityClass, k32s priorityOffset);

/** 
 * Sets the CPUs with which the thread pool should have affinity.
 * 
 * This method is thread-safe. 
 *
 * @public              @memberof kThread
 * @param   pool        kThreadPool object.
 * @param   affinity    Array of CPUs with which the thread pool should have affinity (can use kNULL or empty array to specify no affinity).
 * @return              Operation status. 
 * @see                 kThread_SetAffinity
 */
kFx(kStatus) kThreadPool_SetAffinity(kThreadPool pool, kBitArray affinity);

/** 
 * Returns the number of threads in the pool.
 *
 * This method is thread-safe. 
 *
 * @public              @memberof kThreadPool
 * @param   pool        kThreadPool object.
 * @return              Thread count. 
*/
kFx(kSize) kThreadPool_Count(kThreadPool pool);

/** 
 * Schedules the specified callback for execution on the thread pool. 
 * 
 * The transaction argument can optionally receive a transaction handle that can be used to wait 
 * for callback completion. If the transaction argument is not kNULL, kThreadPool_EndExecute <em>must</em> be 
 * used to wait for transaction completion; otherwise, leaks will result. 
 *
 * This method is thread-safe. 
 *
 * @public              @memberof kThreadPool
 * @param   pool        Thread pool object.
 * @param   entryFx     Function to execute.
 * @param   context     Context parameter for callback function.
 * @param   transaction Optionally receives a transaction handle; if received, must be passed to kThreadPool_EndExecute.
 * @return              Operation status. 
*/
kFx(kStatus) kThreadPool_BeginExecute(kThreadPool pool, kThreadFx entryFx, kPointer context, kThreadPoolTransaction* transaction);

/**
 * Blocks until execution of a thread pool transaction is complete.
 *
 * If a finite timeout is specified, this function may return kERROR_TIMEOUT. In this case, it is the caller's 
 * responsibility to call kThreadPool_EndExecute repeatedly until the transaction is completed. 
 *
 * This method is thread-safe. 
 *
 * @public                  @memberof kThreadPool
 * @param   pool            Thread pool object.
 * @param   transaction     Transaction handle emitted by kThreadPool_BeginExecute.
 * @param   timeout         Timeout in microseconds, or kINFINITE to wait indefinitely.
 * @param   status          Upon success, optionally receives the transaction result (can be kNULL).
 * @return                  Operation status.
*/
kFx(kStatus) kThreadPool_EndExecute(kThreadPool pool, kThreadPoolTransaction transaction, k64u timeout, kStatus* status);

/** 
 * Returns the default thread pool.
 *
 * The default thread pool is constructed with a suitable number of threads to make effective use of 
 * underlying processor hardware. Because the default thread pool is a global resource, callbacks that 
 * perform I/O or that could otherwise block for a long duration should be avoided, in order to ensure 
 * that pool threads remain available to perform useful work. 
 * 
 * The default thread pool is automatically created on first use (lazy initialization).
 *
 * This method is thread-safe. 
 *
 * @public              @memberof kThreadPool
 * @return              Default thread pool. 
*/
kFx(kThreadPool) kThreadPool_Default();

#endif

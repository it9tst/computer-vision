/** 
 * @file    kParallel.x.h
 *
 * @internal
 * Copyright (C) 2017-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_PARALLEL_X_H
#define K_API_PARALLEL_X_H

#include <kApi/Threads/kParallel.h>
#include <kApi/Threads/kThreadPool.h>

kDeclareValueEx(k, kParallelArgs, kValue)

 /**
  * @internal
  * @class   kParallelJob
  * @extends kObject
  * @ingroup kApi-Threads
  * @brief   Represents a job for parallel execution. 
  */
//typedef kObject kParallelJob;   -- forward declared in kApiDef.x.h

typedef struct kParallelJobClass
{
    kObjectClass base;

    kParallel parallel;         //reference to job owner
    kParallelFx callbackFx;     //data processing callback function
    kPointer receiver;          //'receiver' context pointer for callback function 
    kPointer content;           //kParallelArgs.content pointer for callback function
    kAtomic32s index;           //index of callback call
    kThreadPool pool;           //pool providing the threads
    kArray1 transactions;       //array of thread-pool transactions -- kArray1<kPointer>
} kParallelJobClass;

kDeclareClassEx(k, kParallelJob, kObject)

/* 
* kParallelJob private methods. 
*/

kFx(kStatus) xkParallelJob_Construct(kParallelJob* job, kAlloc allocator);
kFx(kStatus) xkParallelJob_Init(kParallelJob job, kType type, kAlloc alloc);
kFx(kStatus) xkParallelJob_VRelease(kParallelJob job);

kFx(kStatus) xkParallelJob_Prepare(kParallelJob job, kParallel parallel, kParallelFx callbackFx, kPointer receiver, kPointer content);
kFx(kStatus) xkParallelJob_DetachThreadPool(kParallelJob job);

kFx(kStatus) xkParallelJob_BeginExecute(kParallelJob job);
kFx(kStatus) xkParallelJob_EndExecute(kParallelJob job);   

kStatus kCall xkParallelJob_CallbackThreadEntry(kParallelJob job);

 /**
  * @internal
  * @class   kParallel
  * @extends kObject
  * @ingroup kApi-Threads
  * @brief   Allows parallel execution on a data set.
  */

#define xkPARALLEL_DEFAULT_JOB_COUNT     (4)        //number of initial job objects to create; can be increased dynamically

typedef struct kParallelVTable
{
    kObjectVTable base;
} kParallelVTable;

typedef struct kParallelStatic
{
    kParallel instance;         //singleton parallel object
} kParallelStatic;

typedef struct kParallelClass
{
    kObjectClass base;

    kBool isMulticore;          //does the current system have more than one processor?
    kQueue freeJobQueue;        //caches free job objects -- kQueue<kParallelPoolJob>
    kLock queueLock;            //lock for freeJobQueue
} kParallelClass;

kDeclareFullClassEx(k, kParallel, kObject)

/* 
* kParallel private methods. 
*/

kFx(kStatus) xkParallel_InitStatic();
kFx(kStatus) xkParallel_ReleaseStatic();
kFx(kParallel) xkParallel_Default();

//can optionally be called by framework/environment during shutdown to ensure that 
//kParallel has no references to the global thread pool, in cases where it is necessary 
//to provide this guarantee prior to kApi library teardown. assumes no outstanding 
//parallel jobs at the time that this method is called.
kFx(kStatus) xkParallel_DetachThreadPool();

kFx(kStatus) xkParallel_Construct(kParallel* parallel, kAlloc allocator);
kFx(kStatus) xkParallel_Init(kParallel parallel, kType type, kAlloc alloc);
kFx(kStatus) xkParallel_VRelease(kParallel parallel);

kStatus kCall xkParallel_ConstructJob(kParallel parallel, kParallelJob* job, kParallelFx callbackFx, kPointer receiver, kPointer content);

#endif

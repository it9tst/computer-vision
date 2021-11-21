/** 
 * @file    kThreadPool.x.h
 *
 * @internal
 * Copyright (C) 2017-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_THREADPOOL_X_H
#define K_API_THREADPOOL_X_H

 /**
  * @internal
  * @class   kThreadPoolJob
  * @extends kObject
  * @ingroup kApi-Threads
  * @brief   Represents a single job. 
  */
typedef kObject kThreadPoolJob; 

typedef struct kThreadPoolJobClass
{
    kObjectClass base;

    kThreadFx entryFx;          //thread entry-point 
    kPointer entryContext;      //thread context
    kSemaphore sem;             //signals end of thread execution
    kStatus status;             //thread return value
    kBool tracked;              //will the user call kThreadPool_EndExecute to complete the transaction?
} kThreadPoolJobClass;

kDeclareClassEx(k, kThreadPoolJob, kObject)

/* 
* kThreadPoolJob private methods
*/

kFx(kStatus) xkThreadPoolJob_Construct(kThreadPoolJob* job, kAlloc allocator);
kFx(kStatus) xkThreadPoolJob_Init(kThreadPoolJob job, kType type, kAlloc alloc);
kFx(kStatus) xkThreadPoolJob_VRelease(kThreadPoolJob job);

kFx(kStatus) xkThreadPoolJob_Track(kThreadPoolJob job, kBool track);
kFx(kBool) xkThreadPoolJob_IsTracked(kThreadPoolJob job);

kFx(kStatus) xkThreadPoolJob_SignalCompletion(kThreadPoolJob job);
kFx(kStatus) xkThreadPoolJob_Wait(kThreadPoolJob job, kStatus* status, k64u timeout);

kFx(kStatus) xkThreadPoolJob_Prepare(kThreadPoolJob job, kThreadFx threadFx, kPointer context, kBool isTracked);

kFx(kStatus) xkThreadPoolJob_Execute(kThreadPoolJob job);

 /**
  * @internal
  * @class   kThreadPool
  * @extends kObject
  * @ingroup kApi-Threads
  * @brief   Represents a thread pool with a fixed number of threads. 
  */

typedef struct kThreadPoolVTable
{
    kObjectVTable base;
} kThreadPoolVTable;

typedef struct kThreadPoolStatic
{
    kLock lock;                 //guard for lazy initialization of globalPool
    kThreadPool globalPool;     //default/global thread pool
} kThreadPoolStatic;

typedef struct kThreadPoolClass
{
    kObjectClass base;

    kArray1 threads;            //threads in the pool -- kArray1<kThread>
    kMsgQueue jobQueue;         //jobs waiting for execution -- kMsgQueue<kThreadPoolJob>
    kQueue freeJobQueue;        //caches free job objects -- kQueue<kThreadPoolJob>
    kLock queueLock;            //lock for freeJobQueue.
    kLock configLock;           //lock for thread configuration.
} kThreadPoolClass;

kDeclareFullClassEx(k, kThreadPool, kObject)

/* 
* kThreadPool private methods. 
*/

kFx(kStatus) xkThreadPool_InitStatic();
kFx(kStatus) xkThreadPool_ReleaseStatic();

//can optionally be called by framework/environment during shutdown to preemptively 
//destroy the default/global thread pool, in cases where it is necessary 
//to guarantee that no threads are runnning prior to kApi library teardown. assumes 
//no outstanding jobs at the time that this method is called.
kFx(kStatus) xkThreadPool_DestroyDefault(); 

kFx(kStatus) xkThreadPool_Init(kThreadPool pool, kType type, kSize threadCount, kAlloc alloc);
kFx(kStatus) xkThreadPool_VRelease(kThreadPool pool); 

kStatus kCall xkThreadPool_ConstructJob(kThreadPool pool, kThreadPoolJob* job, kThreadFx entryFx, kPointer context, kBool isTracked);
kFx(kStatus) xkThreadPool_FreeJob(kThreadPool pool, kThreadPoolJob job);

kStatus kCall xkThreadPool_JobThreadEntry(kThreadPool pool);

#endif

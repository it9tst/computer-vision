/** 
 * @file    kParallel.cpp
 *
 * @internal
 * Copyright (C) 2017-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <kApi/Threads/kParallel.h>

#include <kApi/Data/kArray1.h>
#include <kApi/Data/kQueue.h>
#include <kApi/Threads/kLock.h>
#include <kApi/Threads/kThread.h>

/* 
 * kParallelArgs
 */
kBeginValueEx(k, kParallelArgs)
    kAddField(kParallelArgs, kPointer, content)
    kAddField(kParallelArgs, kSize, index)
    kAddField(kParallelArgs, kSize, count)
kEndValueEx()

kFx(kSize) kParallelArgs_Begin(const kParallelArgs* args, kSize start, kSize count)
{
    kAssert(args->count > 0); 

    return start + ((args->index * count) / args->count); 
}

kFx(kSize) kParallelArgs_End(const kParallelArgs* args, kSize start, kSize count)
{
    kAssert(args->count > 0); 

    return start + (((args->index + 1) * count) / args->count); 
}

/*
 * kParallelJob
 */

kBeginClassEx(k, kParallelJob)
    kAddPrivateVMethod(kParallelJob, kObject, VRelease)
kEndClassEx()

kFx(kStatus) xkParallelJob_Construct(kParallelJob* job, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback(allocator);
    kStatus status;

    kCheck(kAlloc_GetObject(alloc, kTypeOf(kParallelJob), job));

    if (!kSuccess(status = xkParallelJob_Init(*job, kTypeOf(kParallelJob), alloc)))
    {
        kAlloc_FreeRef(alloc, job);
    }

    return status;
}

kFx(kStatus) xkParallelJob_Init(kParallelJob job, kType type, kAlloc alloc)
{
    kObjR(kParallelJob, job);
    kStatus status = kOK;
    kSize processorCount = kThread_ProcessorCount(); 

    kCheck(kObject_Init(job, type, alloc));

    obj->parallel = kNULL;
    obj->callbackFx = kNULL;
    obj->receiver = kNULL;
    obj->content = kNULL;
    obj->index = 0;
    obj->pool = kNULL;
    obj->transactions = kNULL;

    kTry
    {
        kTest(kArray1_Construct(&obj->transactions, kTypeOf(kPointer), processorCount, alloc)); 
        kTest(kArray1_Zero(obj->transactions)); 
    }
    kCatch(&status)
    {
        xkParallelJob_VRelease(job); 

        kEndCatch(status); 
    }

    return kOK;
}

kFx(kStatus) xkParallelJob_VRelease(kParallelJob job)
{
    kObj(kParallelJob, job);

    kCheck(kObject_Destroy(obj->transactions)); 

    kCheck(kObject_VRelease(job));

    return kOK;
}

kFx(kStatus) xkParallelJob_Prepare(kParallelJob job, kParallel parallel, kParallelFx callbackFx, kPointer receiver, kPointer content)
{
    kObj(kParallelJob, job);
 
    obj->parallel = parallel; 
    obj->callbackFx = callbackFx;
    obj->receiver = receiver;
    obj->content = content;
    kAtomic32s_Exchange(&obj->index, 0);

    //we wait until this moment to get a reference to the default thread pool; 
    //this supports lazy initialization of the default thread pool
    if (kIsNull(obj->pool))
    {
        obj->pool = kThreadPool_Default();
    }

    if (kArray1_Count(obj->transactions) != kThreadPool_Count(obj->pool))
    {
        kCheck(kArray1_Allocate(obj->transactions, kTypeOf(kPointer), kThreadPool_Count(obj->pool)));
        kCheck(kArray1_Zero(obj->transactions));
    }

    return kOK;
}

kFx(kStatus) xkParallelJob_DetachThreadPool(kParallelJob job)
{
    kObj(kParallelJob, job);

    //null-out this job's thread-pool reference; the job will reacquire the reference if 
    //it is later prepared for use again
    obj->pool = kNULL;

    return kOK;
}

kFx(kStatus) xkParallelJob_BeginExecute(kParallelJob job)
{
    kObj(kParallelJob, job);
    kSize i;
    kStatus status = kOK; 

    kTry
    {
        for (i = 0; i < kArray1_Count(obj->transactions); i++)
        {
            kTest(kThreadPool_BeginExecute(obj->pool, xkParallelJob_CallbackThreadEntry, job, kArray1_AtT(obj->transactions, i, kThreadPoolTransaction)));
        }
    }
    kCatch(&status)
    {
        xkParallelJob_EndExecute(job); 

        kEndCatch(status); 
    }

    return kOK;
}

kFx(kStatus) xkParallelJob_EndExecute(kParallelJob job)
{
    kObj(kParallelJob, job);
    kThreadPoolTransaction* transactions = kArray1_DataT(obj->transactions, kThreadPoolTransaction); 
    kStatus status = kOK;
    kSize i;

    for (i = 0; i < kArray1_Count(obj->transactions); i++)
    {
        if (!kIsNull(transactions[i]))
        {
            kCheck(kThreadPool_EndExecute(obj->pool, transactions[i], kINFINITE, kSuccess(status) ? &status : kNULL));
        }

        transactions[i] = kNULL; 
    }

    return status; 
}

kStatus kCall xkParallelJob_CallbackThreadEntry(kParallelJob job)
{
    kObj(kParallelJob, job);
    kParallelArgs args;

    args.content = obj->content;
    args.index = (kSize) (kAtomic32s_Increment(&obj->index) - 1);
    args.count = kArray1_Count(obj->transactions); 

    return obj->callbackFx(obj->receiver, obj->parallel, &args);
}

/*
 * kParallel
 */

kBeginFullClassEx(k, kParallel)
    kAddPrivateVMethod(kParallel, kObject, VRelease)
kEndFullClassEx()

kFx(kStatus) xkParallel_InitStatic()
{
    kStaticObj(kParallel);
    kStatus status = kOK;

    kTry
    {
        kTest(xkParallel_Construct(&sobj->instance, kNULL));
    }
    kCatch(&status)
    {
        xkParallel_ReleaseStatic();

        kEndCatch(status);
    }

    return kOK;
}

kFx(kStatus) xkParallel_ReleaseStatic()
{
    kStaticObj(kParallel);

    kCheck(kDestroyRef(&sobj->instance));

    return kOK;
}

kFx(kStatus) xkParallel_DetachThreadPool()
{
    kStaticObj(kParallel);

    if (!kIsNull(sobj->instance))
    {
        kObj(kParallel, sobj->instance);

        kLock_Enter(obj->queueLock);
        {
            for (kSize i = 0; i < kQueue_Count(obj->freeJobQueue); ++i)
            {
                kParallelJob job = kQueue_AsT(obj->freeJobQueue, i, kParallelJob); 

                //remove parallel job's reference to global thread pool
                xkParallelJob_DetachThreadPool(job);
            }
        }
        kLock_Exit(obj->queueLock); 
    }

    return kOK;
}

kFx(kParallel) xkParallel_Default()
{
    kStaticObj(kParallel);

    return sobj->instance;
}

kFx(kStatus) xkParallel_Construct(kParallel* parallel, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback(allocator);
    kStatus status;

    kCheck(kAlloc_GetObject(alloc, kTypeOf(kParallel), parallel));

    if (!kSuccess(status = xkParallel_Init(*parallel, kTypeOf(kParallel), alloc)))
    {
        kAlloc_FreeRef(alloc, parallel);
    }

    return status;
}

kFx(kStatus) xkParallel_Init(kParallel parallel, kType type, kAlloc alloc)
{
    kObjR(kParallel, parallel);
    kStatus status = kOK;
    kSize i;

    kCheck(kObject_Init(parallel, type, alloc));

    obj->isMulticore = kFALSE;
    obj->freeJobQueue = kNULL;
    obj->queueLock = kNULL;

    obj->isMulticore = (kThread_ProcessorCount() > 1);

    kTry
    {
        kTest(kLock_Construct(&obj->queueLock, alloc));
        kTest(kQueue_Construct(&obj->freeJobQueue, kTypeOf(kParallelJob), xkPARALLEL_DEFAULT_JOB_COUNT, alloc));

        for (i = 0; i < xkPARALLEL_DEFAULT_JOB_COUNT; i++)
        {
            kParallelJob job = kNULL;

            kTest(xkParallelJob_Construct(&job, alloc));
            kTest(kQueue_AddT(obj->freeJobQueue, &job));
        }
    }
    kCatch(&status)
    {
        xkParallel_VRelease(parallel);
        kEndCatch(status);
    }
    
    return kOK;
}

kFx(kStatus) xkParallel_VRelease(kParallel parallel)
{
    kObj(kParallel, parallel);
 
    kCheck(kDisposeRef(&obj->freeJobQueue));

    kCheck(kObject_Destroy(obj->queueLock));

    kCheck(kObject_VRelease(parallel));

    return kOK;
}

kStatus kCall xkParallel_ConstructJob(kParallel parallel, kParallelJob* job, kParallelFx callbackFx, kPointer receiver, kPointer content)
{
    kObj(kParallel, parallel);
    kParallelJob output = kNULL; 
    kStatus status; 
    
    kLock_Enter(obj->queueLock);

    kTry
    {
        if (!kSuccess(kQueue_Remove(obj->freeJobQueue, &output)))
        {
            kTest(xkParallelJob_Construct(&output, kNULL));
        }

        kTest(xkParallelJob_Prepare(output, parallel, callbackFx, receiver, content)); 

        *job = output; 
    }
    kCatchEx(&status)
    {
        kObject_Destroy(output); 

        kEndCatchEx(status);
    }
    kFinallyEx
    {
        kLock_Exit(obj->queueLock);

        kEndFinallyEx();
    }

    return kOK;
}

kFx(kStatus) kParallel_ExecuteDirect(kParallelFx callback, kPointer receiver, kPointer content)
{
    kParallel parallel = xkParallel_Default();
    kParallelArgs args = { 0 };

    args.content = content;
    args.count = 1;
    args.index = 0;

    kCheck(callback(receiver, parallel, &args));

    return kOK;
}

kFx(kStatus) kParallel_Execute(kParallelFx callback, kPointer receiver, kPointer content)
{
    kParallel parallel = xkParallel_Default();
    kObj(kParallel, parallel);

    if (obj->isMulticore)
    {
        kParallelTransaction transaction = kNULL;

        kCheck(kParallel_BeginExecute(callback, receiver, content, &transaction));
        kCheck(kParallel_EndExecute(transaction));
    }
    else
    {
        kCheck(kParallel_ExecuteDirect(callback, receiver, content));
    }

    return kOK;
}

kFx(kStatus) kParallel_BeginExecute(kParallelFx callback, kPointer receiver, kPointer content, kParallelTransaction* transaction)
{
    kParallel parallel = xkParallel_Default();
    kParallelJob job = kNULL;
    kStatus status = kOK; 

    kCheckArgs(!kIsNull(transaction)); 

    kCheck(xkParallel_ConstructJob(parallel, &job, callback, receiver, content));
 
    if (!kSuccess(status = xkParallelJob_BeginExecute(job)))
    {
        kObject_Destroy(job); 
        return status; 
    }

    *transaction = job;

    return kOK;
}

kFx(kStatus) kParallel_EndExecute(kParallelTransaction transaction)
{
    kParallel parallel = xkParallel_Default();
    kObj(kParallel, parallel);
    kParallelJob job = transaction;
    kStatus status = kOK;

    if (!kIsNull(transaction))
    {        
        status = xkParallelJob_EndExecute(transaction);

        kLock_Enter(obj->queueLock);

        kTry
        {
            kTest(kQueue_AddT(obj->freeJobQueue, &job));
        }
        kFinally
        {
            kLock_Exit(obj->queueLock);

            kEndFinally();
        }
    }

    return status;
}

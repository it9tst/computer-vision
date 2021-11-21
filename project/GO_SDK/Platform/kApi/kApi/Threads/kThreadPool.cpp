/** 
 * @file    kThreadPool.cpp
 *
 * @internal
 * Copyright (C) 2017-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <kApi/Threads/kThreadPool.h>

#include <kApi/Data/kArray1.h>
#include <kApi/Data/kQueue.h>
#include <kApi/Threads/kLock.h>
#include <kApi/Threads/kMsgQueue.h>
#include <kApi/Threads/kSemaphore.h>
#include <kApi/Threads/kThread.h>

/*
 * kThreadPoolJob
 */

kBeginClassEx(k, kThreadPoolJob)
    kAddPrivateVMethod(kThreadPoolJob, kObject, VRelease)
kEndClassEx()

kFx(kStatus) xkThreadPoolJob_Construct(kThreadPoolJob* job, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback(allocator);
    kStatus status;

    kCheck(kAlloc_GetObject(alloc, kTypeOf(kThreadPoolJob), job));

    if (!kSuccess(status = xkThreadPoolJob_Init(*job, kTypeOf(kThreadPoolJob), alloc)))
    {
        kAlloc_FreeRef(alloc, job);
    }

    return status;
}

kFx(kStatus) xkThreadPoolJob_Init(kThreadPool job, kType type, kAlloc alloc)
{
    kObjR(kThreadPoolJob, job);
    kStatus status = kOK;

    kCheck(kObject_Init(job, type, alloc));

    obj->entryFx = kNULL;
    obj->entryContext = kNULL;
    obj->sem = kNULL;
    obj->status = kERROR;
    obj->tracked = kFALSE;

    kTry
    {
        kTest(kSemaphore_Construct(&obj->sem, 0, alloc));
    }
    kCatch(&status)
    {
        xkThreadPoolJob_VRelease(job);
        kEndCatch(status);
    }

    return kOK;
}

kFx(kStatus) xkThreadPoolJob_VRelease(kThreadPool job)
{
    kObj(kThreadPoolJob, job);

    kCheck(kObject_Destroy(obj->sem));

    kCheck(kObject_VRelease(job));

    return kOK;
}

kFx(kBool) xkThreadPoolJob_IsTracked(kThreadPoolJob job)
{
    kObj(kThreadPoolJob, job);

    return obj->tracked;
}

kFx(kStatus) xkThreadPoolJob_Track(kThreadPoolJob job, kBool track)
{
    kObj(kThreadPoolJob, job);

    obj->tracked = track;

    return kOK;
}

kFx(kStatus) xkThreadPoolJob_SignalCompletion(kThreadPoolJob job)
{
    kObj(kThreadPoolJob, job);

    return kSemaphore_Post(obj->sem);
}

kFx(kStatus) xkThreadPoolJob_Wait(kThreadPoolJob job, kStatus* status, k64u timeout)
{
    kObj(kThreadPoolJob, job);

    kCheck(kSemaphore_Wait(obj->sem, timeout));

    if (!kIsNull(status))
    {
        *status = obj->status;
    }

    return kOK;
}

kFx(kStatus) xkThreadPoolJob_Prepare(kThreadPoolJob job, kThreadFx threadFx, kPointer context, kBool isTracked)
{
    kObj(kThreadPoolJob, job);

    obj->entryFx = threadFx;
    obj->entryContext = context;
    obj->tracked = isTracked;

    return kOK;
}

kFx(kStatus) xkThreadPoolJob_Execute(kThreadPoolJob job)
{
    kObj(kThreadPoolJob, job);

    obj->status = obj->entryFx(obj->entryContext); 

    return kOK; 
}

/*
 * kThreadPool
 */

kBeginFullClassEx(k, kThreadPool)
    kAddPrivateVMethod(kThreadPool, kObject, VRelease)
kEndFullClassEx()

kFx(kStatus) xkThreadPool_InitStatic()
{
    kStaticObj(kThreadPool);
    kStatus status = kOK;

    kTry
    {
        kTest(kLock_Construct(&sobj->lock, kNULL));
    }
    kCatch(&status)
    {
        xkThreadPool_ReleaseStatic();
        kEndCatch(status);
    }

    return kOK;
}

kFx(kStatus) xkThreadPool_ReleaseStatic()
{
    kStaticObj(kThreadPool);

    kDestroyRef(&sobj->globalPool);
    kDestroyRef(&sobj->lock);

    return kOK; 
}

kFx(kThreadPool) kThreadPool_Default()
{
    kStaticObj(kThreadPool);
    
    kLock_Enter(sobj->lock);
    {
        if (kIsNull(sobj->globalPool))
        {
            if (!kSuccess(kThreadPool_Construct(&sobj->globalPool, kThread_ProcessorCount(), kNULL)))
            {
                sobj->globalPool = kNULL;
            }
        }
    }
    kLock_Exit(sobj->lock);
 
    return sobj->globalPool;
}

kFx(kStatus) xkThreadPool_DestroyDefault()
{
    kStaticObj(kThreadPool);
    
    if (!kIsNull(sobj->lock))
    {
        kLock_Enter(sobj->lock);
        {
            kDestroyRef(&sobj->globalPool);
        }
        kLock_Exit(sobj->lock);
    }

    return kOK;
}

kFx(kStatus) kThreadPool_Construct(kThreadPool* pool, kSize threadCount, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback(allocator);
    kStatus status;

    kCheck(kAlloc_GetObject(alloc, kTypeOf(kThreadPool), pool));

    if (!kSuccess(status = xkThreadPool_Init(*pool, kTypeOf(kThreadPool), threadCount, alloc)))
    {
        kAlloc_FreeRef(alloc, pool);
    }

    return status;
}

kFx(kStatus) xkThreadPool_Init(kThreadPool pool, kType type, kSize threadCount, kAlloc alloc)
{
    kObjR(kThreadPool, pool);
    kStatus status = kOK;
    kSize initialJobCount = 2*threadCount; 
    kSize i;

    kCheck(kObject_Init(pool, type, alloc));

    obj->threads = kNULL;
    obj->jobQueue = kNULL;
    obj->freeJobQueue = kNULL;
    obj->queueLock = kNULL;
    obj->configLock = kNULL;

    kTry
    {
        kTest(kLock_ConstructEx(&obj->configLock, xkLOCK_OPTION_PRIORITY_INHERITANCE, alloc));

        kTest(kLock_ConstructEx(&obj->queueLock, xkLOCK_OPTION_PRIORITY_INHERITANCE, alloc));

        kTest(kQueue_Construct(&obj->freeJobQueue, kTypeOf(kThreadPoolJob), initialJobCount, alloc));
        
        for (i = 0; i < initialJobCount; i++)
        {
            kThreadPoolJob job = kNULL;

            kTest(xkThreadPoolJob_Construct(&job, kNULL));
            kTest(kQueue_AddT(obj->freeJobQueue, &job));
        }

        kTest(kMsgQueue_Construct(&obj->jobQueue, kTypeOf(kThreadPoolJob), alloc));
        kTest(kMsgQueue_Reserve(obj->jobQueue, initialJobCount));

        kTest(kArray1_Construct(&obj->threads, kTypeOf(kThread), threadCount, alloc));
        kTest(kArray1_Zero(obj->threads));

        for (i = 0; i < threadCount; i++)
        {
            kThread* thread = kArray1_AtT(obj->threads, i, kThread);

            kTest(kThread_Construct(thread, alloc));
            kTest(kThread_StartEx(*thread, xkThreadPool_JobThreadEntry, pool, 0, "kThreadPool.Thread", 0));
        }
    }
    kCatch(&status)
    {
        xkThreadPool_VRelease(pool);
        kEndCatch(status);
    }
    
    return kOK;
}

kFx(kStatus) xkThreadPool_VRelease(kThreadPool pool)
{
    kObj(kThreadPool, pool); 
    kThreadPoolJob quitMsg = kNULL; 
    kSize i; 

    if (!kIsNull(obj->threads) && !kIsNull(obj->jobQueue))
    {
        for (i = 0; i < kArray1_Count(obj->threads); ++i)
        {
            kCheck(kMsgQueue_AddT(obj->jobQueue, &quitMsg)); 
        }
    }

    kCheck(kDisposeRef(&obj->threads));
    kCheck(kDisposeRef(&obj->freeJobQueue));
    
    kCheck(kDestroyRef(&obj->jobQueue));
    kCheck(kDestroyRef(&obj->queueLock));

    kCheck(kDestroyRef(&obj->configLock));

    kCheck(kObject_VRelease(pool)); 
    
    return kOK;
}

kStatus kCall xkThreadPool_ConstructJob(kThreadPool pool, kThreadPoolJob* job, kThreadFx entryFx, kPointer context, kBool isTracked)
{
    kObj(kThreadPool, pool);
    kThreadPoolJob output = kNULL; 
    kStatus status; 

    kLock_Enter(obj->queueLock);

    kTry
    {
        if (!kSuccess(kQueue_Remove(obj->freeJobQueue, &output)))
        {
            kTest(xkThreadPoolJob_Construct(&output, kObject_Alloc(pool)));
        }

        kTest(xkThreadPoolJob_Prepare(output, entryFx, context, isTracked));  

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

kFx(kStatus) kThreadPool_BeginExecute(kThreadPool pool, kThreadFx entryFx, kPointer context, kThreadPoolTransaction* transaction)
{
    kObj(kThreadPool, pool);
    kThreadPoolJob job = kNULL;
    kStatus status = kOK; 
 
    kCheck(xkThreadPool_ConstructJob(pool, &job, entryFx, context, !kIsNull(transaction)));
     
    if (!kSuccess(status = kMsgQueue_AddT(obj->jobQueue, &job)))
    {
        xkThreadPool_FreeJob(pool, job); 
        return status;
    }

    if (!kIsNull(transaction))
    {
        *transaction = job;
    }
    
    return kOK;
}

kFx(kStatus) kThreadPool_EndExecute(kThreadPool pool, kThreadPoolTransaction transaction, k64u timeout, kStatus* status)
{
    kObj(kThreadPool, pool);

    if (!kIsNull(transaction))
    {
        kThreadPoolJob job = transaction;

        kCheckState(xkThreadPoolJob_IsTracked(job));

        kCheck(xkThreadPoolJob_Wait(job, status, timeout));

        kCheck(xkThreadPool_FreeJob(pool, job));
    }

    return kOK;
}

kFx(kStatus) xkThreadPool_FreeJob(kThreadPool pool, kThreadPoolJob job)
{
    kObj(kThreadPool, pool);

    kCheck(kLock_Enter(obj->queueLock));

    kTry
    {
        kTest(kQueue_AddT(obj->freeJobQueue, &job));
    }
    kFinally
    {
        kLock_Exit(obj->queueLock);
        kEndFinally();
    }

    return kOK;
}

kStatus kCall xkThreadPool_JobThreadEntry(kThreadPool pool)
{
    kObj(kThreadPool, pool);
    kBool shouldQuit = kFALSE; 
    kStatus status; 
    
    kTry
    {
        while (!shouldQuit)
        {
            kThreadPoolJob job = kNULL;

            if (kSuccess(kMsgQueue_RemoveT(obj->jobQueue, &job, kINFINITE)))
            {
                shouldQuit = kIsNull(job); 
                
                if (!shouldQuit)
                {
                    kTest(xkThreadPoolJob_Execute(job));

                    if (xkThreadPoolJob_IsTracked(job))
                    {
                        kTest(xkThreadPoolJob_SignalCompletion(job));
                    }
                    else
                    {
                        kTest(xkThreadPool_FreeJob(pool, job));
                    }
                }
            }
        }
    }
    kCatch(&status)
    {
        kLogf("kThreadPool: thread exited unexpectedly.");

        kEndCatch(status);
    }
    
    return kOK;
}

kFx(kStatus) kThreadPool_SetAffinity(kThreadPool pool, kBitArray affinity)
{
    kObj(kThreadPool, pool);

    kCheck(kLock_Enter(obj->configLock));

    kTry
    {
        for (kSize i = 0; i < kArray1_Length(obj->threads); ++i)
        {
            kThread thread = kArray1_AsT(obj->threads, i, kThread);

            kTest(kThread_SetAffinity(thread, affinity));
        }
    }
    kFinally
    {
        kLock_Exit(obj->configLock);
        kEndFinally();
    }

    return kOK;
}

kFx(kStatus) kThreadPool_SetPriority(kThreadPool pool, kThreadPriorityClass priorityClass, k32s priorityOffset)
{
    kObj(kThreadPool, pool);

    kCheck(kLock_Enter(obj->configLock));

    kTry
    {
        for (kSize i = 0; i < kArray1_Length(obj->threads); ++i)
        {
            kThread thread = kArray1_AsT(obj->threads, i, kThread);
        
            kTest(kThread_SetPriority(thread, priorityClass, priorityOffset));
        }
    }
    kFinally
    {
        kLock_Exit(obj->configLock);
        kEndFinally();
    }

    return kOK;
}

kFx(kSize) kThreadPool_Count(kThreadPool pool)
{
    kObj(kThreadPool, pool);
    
    return kArray1_Count(obj->threads);
}

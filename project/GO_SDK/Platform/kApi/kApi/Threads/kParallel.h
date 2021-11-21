/** 
 * @file    kParallel.h
 * @brief   Declares the kParallel type. 
 *
 * @internal
 * Copyright (C) 2017-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_PARALLEL_H
#define K_API_PARALLEL_H

#include <kApi/kApiDef.h>

/**
 * @struct  kParallelArgs
 * @extends kValue
 * @ingroup kApi-Threads
 * @brief   Arguments received by a kParallel data processing callback. 
 */
typedef struct kParallelArgs
{
    kPointer content;       ///< Shared data content, to be processed by callbacks. 
    kSize index;            ///< Index of current processing callback (0..count-1). 
    kSize count;            ///< Total number of processing callbacks (1 or more). 
} kParallelArgs;

/** 
 * Helper method that can be used to calculate the data start index within a kParallel callback.  
 *
 * The kParallel class enables data processing to be distributed across multiple callbacks. This 
 * arrangement requires the dataset to be partitioned, with each callback performing a portion of the 
 * total required work. 
 * 
 * Assuming that the dataset can be represented by a linear range (e.g., elements in an array), the 
 * kParallelArgs_Begin method can optionally be used to calculate the lower bound (start index) of the 
 * data to be processed in the current kParallel callback invocation. 
 * 
 * @public          @memberof kParallelArgs
 * @param   args    Arguments received by a kParallel data processing callback. 
 * @param   start   First index of the overall range to be collectively processed. 
 * @param   count   Total count of elements to be collectively processed.  
 * @return          Index of first element to be processed in this callback. 
 */
kFx(kSize) kParallelArgs_Begin(const kParallelArgs* args, kSize start, kSize count); 

/** 
 * Helper method that can be used to calculate the data end index within a kParallel callback.  
 *
 * The kParallel class enables data processing to be distributed across multiple callbacks. This 
 * arrangement requires the dataset to be partitioned, with each callback performing a portion of the 
 * total required work. 
 * 
 * Assuming that the dataset can be represented by a linear range (e.g., elements in an array), the 
 * kParallelArgs_End method can optionally be used to calculate the upper bound (end index) of the 
 * data to be processed in the current kParallel callback invocation. 
 * 
 * @public          @memberof kParallelArgs
 * @param   args    Arguments received by a kParallel data processing callback. 
 * @param   start   First index of the overall range to be collectively processed. 
 * @param   count   Total count of elements to be collectively processed.  
 * @return          One greater than the index of the last element to be processed in this callback. 
 */
kFx(kSize) kParallelArgs_End(const kParallelArgs* args, kSize start, kSize count); 

/** kParallel data processing callback signature.  */
typedef kStatus(kCall* kParallelFx)(kPointer receiver, kParallel sender, kParallelArgs* args); 

/**
 * @struct  kParallelTransaction
 * @ingroup kApi-Threads
 * @brief   Opaque pointer to a kParallel transaction. 
 */
typedef kPointer kParallelTransaction;

#include <kApi/Threads/kParallel.x.h>

/**
 * @class   kParallel
 * @extends kObject
 * @ingroup kApi-Threads
 * @brief   Declares the kParallel class. 
 *
 * The kParallel class can be used to process a dataset using a thread pool.  
@code {.c}
 
kStatus MyApp_ProcessImage(MyApp app, kImage image)
{
    //distribute image processing across multiple concurrent callbacks; the number of threads
    //is automatically selected, based on hardware resources
    return kParallel_Execute(MyApp_ProcessImageSlice, app, image); 
}

kStatus kCall MyApp_ProcessImageSlice(MyApp app, kParallel parallel, kParallelArgs* args)
{
    kImage image = args->content; 
    kSize imageHeight = kImage_Height(image); 
    kSize beginRow = kParallelArgs_Begin(args, 0, imageHeight);  
    kSize endRow = kParallelArgs_End(args, 0, imageHeight);  
    kSize i; 

    kCheck(kImage_PixelType(image) == kTypeOf(k8u)); 

    for (i = beginRow; i < endRow; ++i)
    {
        const k8u* row = kImage_RowAtT(image, i, k8u); 

        //process...
    }

    return kOK; 
}

@endcode
 * 
 */
//typedef kObject kParallel;        --forward-declared in kFsDef.x.h

/** 
 * Synchronously processes a dataset. 
 * 
 * On multicore systems, this method is equivalent to calling kParallel_BeginExecute followed 
 * by kParallel_EndExecute. On unicore systems, this method directly invokes the specified 
 * callback. 
 * 
 * This method is thread-safe. 
 *
 * @public              @memberof kParallel
 * @param   callback    Data processing callback function.
 * @param   receiver    Receiver argument to be provided to callback. 
 * @param   content     Content to be provided to callback via kParallelArgs.content. 
 * @return              Operation status (kOK, or first error returned by a callback). 
 */
kFx(kStatus) kParallel_Execute(kParallelFx callback, kPointer receiver, kPointer content);

/** 
 * Begins asynchronously processing a dataset. 
 *
 * This function returns immediately, providing a transaction handle to represent the in-progress data processing. 
 * The kParallel_EndExecute function <em>must</em> be used to wait for execution to complete. Failure to call 
 * kParallel_EndExecute will result in leaks. 
 *
 * This method is thread-safe. 
 *
 * @public              @memberof kParallel
 * @param   callback    Data processing callback function.
 * @param   receiver    Receiver argument to be provided to callback. 
 * @param   content     Content to be provided to callback via kParallelArgs.content. 
 * @param   transaction Receives transaction handle, which must be passed to kParallel_EndExecute. 
 * @return              Operation status. 
 */
kFx(kStatus) kParallel_BeginExecute(kParallelFx callback, kPointer receiver, kPointer content, kParallelTransaction* transaction);

/**
 * Blocks until execution of the specified kParallel transaction is complete.
 * 
 * This method is thread-safe. 
 *
 * @public                   @memberof kParallel
 * @param    transaction     Transaction handle emitted by kParallel_BeginExecute.
 * @return                   Operation status (kOK, or first error returned by a callback). 
 */
kFx(kStatus) kParallel_EndExecute(kParallelTransaction transaction);

/** 
 * Directly invokes the specified callback without using the thread pool.
 * 
 * This method can be used for testing purposes, to determine non-parallel execution time 
 * without having to significantly restructure the caller.
 * 
 * This method is thread-safe. 
 *
 * @public              @memberof kParallel
 * @param   callback    Data processing callback function.
 * @param   receiver    Receiver argument to be provided to callback. 
 * @param   content     Content to be provided to callback via kParallelArgs.content. 
 * @return              Operation status (kOK, or first error returned by a callback). 
 */
kFx(kStatus) kParallel_ExecuteDirect(kParallelFx callback, kPointer receiver, kPointer content);

/** 
 * Reports the number of threads that will be used to process a parallel transaction.
 * 
 * The number of threads used to process a parallel transaction can be determined from within a 
 * parallel processing callback by examining the kParallelArgs 'count' field. However, if an algorithm 
 * requires this information prior to calling kParallel_Execute or kParallel_BeginExecute, (e.g., to 
 * preallocate temporary data structures), the kParallel_ThreadCount method can be used instead.
 * 
 * This method is thread-safe. 
 *
 * @public              @memberof kParallel
 * @return              Number of threads that will be used to process a parallel transaction.
 */
kInlineFx(kSize) kParallel_ThreadCount()
{
    return kThreadPool_Count(kThreadPool_Default());
}

#endif

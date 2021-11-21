/** 
 * @file    kBackTrace.cpp
 *
 * @internal
 * Copyright (C) 2014-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#define K_PLATFORM
#include <kApi/Utils/kBackTrace.h>

#include <kApi/Data/kArrayList.h>
#include <kApi/Data/kString.h>
#include <kApi/Io/kPath.h>
#include <kApi/Threads/kLock.h>
#include <kApi/Utils/kSymbolInfo.h>

kBeginClassEx(k, kBackTrace)

    kAddFrameworkConstructor(kBackTrace, Construct);

    kAddPrivateVMethod(kBackTrace, kObject, VClone)
    kAddPrivateVMethod(kBackTrace, kObject, VEquals)
    kAddPrivateVMethod(kBackTrace, kObject, VHashCode)
kEndClassEx()

kFx(kStatus) kBackTrace_Construct(kBackTrace* trace, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback(allocator);
    kStatus status; 

    kCheck(kAlloc_GetObject(alloc, kTypeOf(kBackTrace), trace)); 

    if (!kSuccess(status = xkBackTrace_Init(*trace, kTypeOf(kBackTrace), alloc)))
    {
        kAlloc_FreeRef(alloc, trace); 
    }

    return status; 
}

kFx(kStatus) xkBackTrace_Init(kBackTrace trace, kType type, kAlloc allocator)
{
    kObjR(kBackTrace, trace); 

    kCheck(kObject_Init(trace, type, allocator)); 

    obj->depth = 0;     
    obj->skip = 0; 

    return kOK; 
}

kFx(kStatus) xkBackTrace_VClone(kBackTrace trace, kBackTrace source, kAlloc valueAlloc, kObject context)
{
    kObj(kBackTrace, trace); 
    kObjN(kBackTrace, sourceObj, source);

    obj->depth = sourceObj->depth; 
    obj->skip = sourceObj->skip; 

    kMemCopy(&obj->functions[0], &sourceObj->functions[0], sourceObj->depth*sizeof(kPointer)); 

    return kOK; 
}

kFx(kBool) xkBackTrace_VEquals(kBackTrace trace, kObject other)
{
    if (kObject_Is(other, kTypeOf(kBackTrace)))
    {
        kObj(kBackTrace, trace);
        kObjN(kBackTrace, otherObj, other); 

        if ((obj->depth - obj->skip) == (otherObj->depth - otherObj->skip))
        {
            return kMemEquals(&obj->functions[obj->skip], &otherObj->functions[obj->skip], (obj->depth - obj->skip)*sizeof(kPointer)); 
        }
    }
  
    return kFALSE; 
}

kFx(kSize) xkBackTrace_VHashCode(kBackTrace trace)
{
    kObj(kBackTrace, trace); 
    kSize sizeBits = K_POINTER_SIZE*8; 
    kSize hashCode = 0; 
    kSize i; 
  
    for (i = obj->skip; i < obj->depth; ++i)
    {
        kSize shift = i & (sizeBits - 1); 
        kSize value = (kSize) obj->functions[i]; 

        if (shift > 0)
        {           
            value = (value << shift) | (value >> (sizeBits - shift)); 
        }

        hashCode = hashCode ^ value;  
    }

    return hashCode; 
}

kFx(kStatus) kBackTrace_Describe(kBackTrace trace, kArrayList* lines, kAlloc allocator)
{
    kObj(kBackTrace, trace);
    kArrayList output = kNULL;
    kSize reportedCount = obj->depth - obj->skip;
    kSize i;
    kStatus status;

    kTry
    {
        kTest(kArrayList_Construct(&output, kTypeOf(kString), reportedCount, allocator));

        for (i = obj->skip; i < obj->depth; ++i)
        {
            kString line = kNULL;

            if (!kSuccess(kSymbolInfo_DescribeFunction(obj->functions[i], &line)))
            {
                kTest(kString_Construct(&line, "", allocator));                
                kTest(xkSymbolInfo_FormatUnknown(obj->functions[i], line)); 
            }

            kArrayList_AddT(output, &line);
        }

        *lines = output;
    }
    kCatch(&status)
    {
        kObject_Dispose(output);

        kEndCatch(status);
    }

    return kOK;
}

kFx(kSize) kBackTrace_Depth(kBackTrace trace)
{
    kObj(kBackTrace, trace); 
  
    return obj->depth - obj->skip; 
}

#if defined(K_DEBUG) && defined(K_WINDOWS) && (_MSC_VER >= 1500) && (_WIN32_WINNT >= _WIN32_WINNT_VISTA) 

kFx(kStatus) kBackTrace_Capture(kBackTrace trace, kSize skip) 
{
    kObj(kBackTrace, trace); 
    kSize totalSkip = skip + 1;   //always remove kBackTrace_Capture from trace

    obj->depth = CaptureStackBackTrace(0, (ULONG)kCountOf(obj->functions), obj->functions, kNULL);   
    obj->skip = kMin_(totalSkip, obj->depth); 

    return kOK; 
}

#elif defined(K_DEBUG) && defined(K_GCC) && defined(K_POSIX)

#include <execinfo.h>

kFx(kStatus) kBackTrace_Capture(kBackTrace trace, kSize skip) 
{
    kObj(kBackTrace, trace); 
    kSize totalSkip = skip + 1;   //always remove kBackTrace_Capture from trace
   
    obj->depth = (kSize) backtrace(&obj->functions[0], kCountOf(obj->functions)); 
    obj->skip = kMin_(totalSkip, obj->depth); 
        
    return kOK; 
}
#else 

kFx(kStatus) kBackTrace_Capture(kBackTrace trace, kSize skip)
{
    return kOK; 
}

#endif

/** 
 * @file    kDebugAlloc.cpp
 *
 * @internal
 * Copyright (C) 2012-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <kApi/Utils/kDebugAlloc.h>
#include <kApi/Data/kArray1.h>
#include <kApi/Data/kArrayList.h>
#include <kApi/Data/kMap.h>
#include <kApi/Data/kString.h>
#include <kApi/Threads/kLock.h>
#include <kApi/Utils/kBackTrace.h>

kBeginValueEx(k, kDebugAllocation)
    kAddField(kDebugAllocation, kPointer, data)
    kAddField(kDebugAllocation, kSize, size)
    kAddField(kDebugAllocation, k64u, index)
kEndValueEx()

kBeginClassEx(k, kDebugAlloc)
    kAddPrivateVMethod(kDebugAlloc, kObject, VRelease)
    kAddPrivateVMethod(kDebugAlloc, kAlloc, VGet)
    kAddPrivateVMethod(kDebugAlloc, kAlloc, VFree)
kEndClassEx()

kFx(kStatus) kDebugAlloc_Construct(kDebugAlloc* object, const kChar* name, kAlloc innerAlloc, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback(allocator);
    kStatus status; 

    kCheck(kAlloc_GetObject(alloc, kTypeOf(kDebugAlloc), object)); 

    if (!kSuccess(status = xkDebugAlloc_Init(*object, kTypeOf(kDebugAlloc), name, innerAlloc, alloc)))
    {
        kAlloc_FreeRef(alloc, object); 
    }

    return status; 
} 

kFx(kStatus) xkDebugAlloc_Init(kDebugAlloc object, kType type, const kChar* name, kAlloc innerAlloc, kAlloc alloc)
{
    kObjR(kDebugAlloc, object); 
    kStatus exception = kOK; 

    kCheck(kAlloc_Init(object, type, alloc)); 

    obj->base.traits = kAlloc_Traits(innerAlloc);

    obj->innerAlloc = innerAlloc; 
    obj->lock = kNULL;
    obj->name[0] = 0;
    obj->allocListener.function = kNULL; 
    obj->allocListener.receiver = kNULL; 
    obj->allocated = 0;
    obj->counter = 0;
    obj->history = kNULL;
    obj->padPattern = kNULL;
    obj->candidateObjectLeaks = kNULL;
    obj->backTrace = kNULL;
    obj->backTraceDescriptions = kNULL;

    kTry
    {
        kTestArgs(!kIsNull(innerAlloc)); 

        if (!kIsNull(name))
        {
            kTest(kStrCopy(obj->name, kCountOf(obj->name), name)); 
        }

        kTest(kLock_ConstructEx(&obj->lock, xkLOCK_OPTION_PRIORITY_INHERITANCE, alloc));         
        kTest(kMap_Construct(&obj->history, kTypeOf(kPointer), kTypeOf(kDebugAllocation), 16, alloc)); 

        kTest(kMap_Construct(&obj->candidateObjectLeaks, kTypeOf(k64u), kTypeOf(kTypeName), 0, alloc)); 

        kTest(kBackTrace_Construct(&obj->backTrace, alloc)); 
        kTest(kMap_Construct(&obj->backTraceDescriptions, kTypeOf(kBackTrace), kTypeOf(kArrayList), 0, alloc)); 

        kTest(kArray1_Construct(&obj->padPattern, kTypeOf(kByte), xkDEBUG_ALLOC_PAD_SIZE, alloc)); 
        kTest(xkDebugAlloc_InitPadPattern(object)); 
    }
    kCatch(&exception)
    {
        xkDebugAlloc_VRelease(object); 
        kEndCatch(exception); 
    }

    return kOK; 
}

kFx(kStatus) xkDebugAlloc_VRelease(kDebugAlloc object)
{
    kObj(kDebugAlloc, object);  

    kCheck(kObject_Destroy(obj->lock)); 
    kCheck(kObject_Destroy(obj->history)); 
    kCheck(kObject_Destroy(obj->candidateObjectLeaks)); 
    kCheck(kObject_Destroy(obj->backTrace)); 
    kCheck(kObject_Dispose(obj->backTraceDescriptions)); 
    kCheck(kObject_Destroy(obj->padPattern)); 

    kCheck(kAlloc_VRelease(object)); 
    
    return kOK; 
}

kFx(kStatus) xkDebugAlloc_InitPadPattern(kDebugAlloc object)
{
    kObj(kDebugAlloc, object); 
    kByte patternBuffer[] = { 19, 223, 61, 239, 149, 181, 37, 101 };        //miscellaneous prime numbers
    kByte* padData = kArray1_DataT(obj->padPattern, kByte); 
    kSize padSize = kArray1_Count(obj->padPattern); 
    kSize i; 

    for (i = 0; i < padSize; ++i)
    {
        padData[i] = patternBuffer[i % kCountOf(patternBuffer)]; 
    }

    return kOK; 
}

kFx(kStatus) kDebugAlloc_Clear(kDebugAlloc object)
{
    kObj(kDebugAlloc, object); 

    kCheck(kLock_Enter(obj->lock)); 

    kTry
    {
        kMapItem it = kMap_First(obj->history); 

        while (!kIsNull(it))
        {
            const kDebugAllocation* allocation = kMap_ValueT(obj->history, it, kDebugAllocation); 
            
            kTest(kAlloc_Free(obj->innerAlloc, allocation->allocatedMem));
            obj->allocated -= allocation->size; 

            it = kMap_Next(obj->history, it); 
        }

        kTest(kMap_Clear(obj->history)); 

        obj->allocated = 0; 
        obj->counter = 0; 
    }
    kFinally
    {
        kLock_Exit(obj->lock); 
        kEndFinally(); 
    }

    return kOK; 
}

kFx(kStatus) xkDebugAlloc_VGet(kDebugAlloc object, kSize size, void* mem, kMemoryAlignment alignment)
{
    kObj(kDebugAlloc, object); 
    kByte* allocatedMem = kNULL; 
    kSize padSize = kArray1_Count(obj->padPattern); 
    const kByte* padData = kArray1_DataT(obj->padPattern, kByte); 
    kBackTrace backTrace = kNULL; 
    kArrayList backTraceDescription = kNULL; 
    kArrayList traceInfo = kNULL; 
    kDebugAllocation allocation; 

    kCheck(kLock_Enter(obj->lock)); 

    kTry
    {   
        kTest(kBackTrace_Capture(obj->backTrace, 1)); 

        if (!kSuccess(kMap_FindT(obj->backTraceDescriptions, &obj->backTrace, &traceInfo)))
        {
            kTest(kObject_Clone(&backTrace, obj->backTrace, kObject_Alloc(object))); 
            kTest(kBackTrace_Describe(backTrace, &backTraceDescription, kObject_Alloc(object))); 

            kTest(kMap_AddT(obj->backTraceDescriptions, &backTrace, &backTraceDescription)); 

            traceInfo = backTraceDescription; 
            backTrace = backTraceDescription = kNULL; 
        }

        const auto alignmentExtraForPadding = kMemoryAlignment_Size(alignment);

        kTest(kAlloc_Get(obj->innerAlloc, size + alignmentExtraForPadding + padSize, &allocatedMem, alignment ));

        allocation.allocatedMem = allocatedMem;
        allocation.data = allocatedMem + alignmentExtraForPadding;
        allocation.size = size; 
        allocation.index = obj->counter;          
        allocation.trace = traceInfo; 

        kItemCopy(allocation.data - padSize, padData, padSize);
        kItemCopy(allocation.data + size, padData, padSize);

        kMemSet(allocation.data, 0xCD, allocation.size);

        kTest(kMap_AddT(obj->history, &allocation.data, &allocation)); 
        
        allocatedMem = kNULL; 
        *(void**)mem = allocation.data;  

        obj->allocated += size; 
        obj->counter++; 

        if (!kIsNull(obj->allocListener.function))
        {
            obj->allocListener.function(obj->allocListener.receiver, object, &allocation); 
        }
    }
    kFinally
    {
        kAlloc_Free(obj->innerAlloc, allocatedMem);

        kObject_Destroy(backTrace); 
        kObject_Dispose(backTraceDescription); 

        kLock_Exit(obj->lock); 
        kEndFinally(); 
    }

    return kOK; 
}

kFx(kStatus) xkDebugAlloc_VFree(kDebugAlloc object, void* mem)
{
    kObj(kDebugAlloc, object); 
    kSize padSize = kArray1_Count(obj->padPattern); 
    const kByte* padData = kArray1_DataT(obj->padPattern, kByte); 

    if (!kIsNull(mem))
    {
        kCheck(kLock_Enter(obj->lock)); 

        kTry
        {    
            kMapItem item = kNULL; 
            kDebugAllocation* allocation = kNULL; 
            kSize size = 0; 

            //If the assertion below is encountered, then the caller has attempted to free
            //an address that is not currently an outstanding memory allocation (often, double-free).
            if(!kSuccess(kMap_FindItemT(obj->history, &mem, &item)))
            {
                kAssert(kFALSE); 
                kThrow(kERROR_HEAP); 
            }

            allocation = kMap_ValueT(obj->history, item, kDebugAllocation);  
            size = allocation->size; 

            //If the assertions below are encountered, then heap memory corruption has occurred. 
            //(Memory immediately before or immediately after the user allocation has been modified.)
            if (!kMemEquals(allocation->data - padSize, padData, padSize))
            {
                kAssert(kFALSE); 
                kThrow(kERROR_HEAP); 
            }
            else if (!kMemEquals(allocation->data + size, padData, padSize))
            {
                kAssert(kFALSE); 
                kThrow(kERROR_HEAP); 
            }

            kTest(kMap_RemoveItem(obj->history, item));   

            kTest(kAlloc_Free(obj->innerAlloc, allocation->allocatedMem));

            obj->allocated -= size; 
        }
        kFinally
        {
            kLock_Exit(obj->lock); 
            kEndFinally(); 
        }
    }

    return kOK; 
}

kFx(k64u) kDebugAlloc_Checkpoint(kDebugAlloc object)
{
    kObj(kDebugAlloc, object); 
    k64u counter = 0; 

    kLock_Enter(obj->lock); 
    {
        counter = obj->counter; 
    }
    kLock_Exit(obj->lock); 

    return counter; 
}

kFx(kStatus) kDebugAlloc_Allocations(kDebugAlloc object, k64u since, kArrayList* history, kAlloc alloc)
{
    kObj(kDebugAlloc, object); 
    kArrayList historyOut = kNULL; 
    kMapItem it = kNULL; 

    kCheck(kLock_Enter(obj->lock)); 

    kTry
    {
        kTest(kArrayList_Construct(&historyOut, kTypeOf(kDebugAllocation), 0, alloc)); 

        it = kMap_First(obj->history); 

        while (!kIsNull(it))
        {
            const kDebugAllocation* allocation = kMap_ValueT(obj->history, it, kDebugAllocation); 
            
            if (allocation->index >= since)
            {
                kTest(kArrayList_AddT(historyOut, allocation)); 
            }

            it = kMap_Next(obj->history, it); 
        }

        *history = historyOut; 
        historyOut = kNULL; 
    }
    kFinally
    {
        kCheck(kObject_Destroy(historyOut)); 
        kCheck(kLock_Exit(obj->lock)); 

        kEndFinally(); 
    }    

    return kOK; 
}

kFx(kStatus) kDebugAlloc_DetectLeakedObjects(kDebugAlloc object, k64u since)
{
    kArrayList assemblies = kNULL; 
    kSize i; 

    kTry
    {
        kTest(kArrayList_Construct(&assemblies, kTypeOf(kAssembly), 0, kAlloc_System())); 
        kTest(kAssembly_Enumerate(assemblies)); 

        for (i = 0; i < kArrayList_Count(assemblies); ++i)
        {
            kAssembly assembly = kArrayList_AsT(assemblies, i, kObject); 

            kTest(kDebugAlloc_DetectLeakedAssemblyObjects(object, 0, assembly)); 
        }
    }
    kFinally
    {
        kDisposeRef(&assemblies); 

        kEndFinally(); 
    }

    return kOK; 
}


kFx(kStatus) kDebugAlloc_DetectLeakedAssemblyObjects(kDebugAlloc object, k64u since, kAssembly assembly)
{
    kObj(kDebugAlloc, object); 
    kSize typeCount = kAssembly_TypeCount(assembly); 
    kMapItem it = kNULL; 

    kCheck(kLock_Enter(obj->lock)); 

    kTry
    {
        it = kMap_First(obj->history); 

        while (!kIsNull(it))
        {
            const kDebugAllocation* allocation = kMap_ValueT(obj->history, it, kDebugAllocation); 
            
            if (allocation->index >= since)
            {
                kObject candidateObject = (kObject) allocation->data; 

                if ((allocation->size >= sizeof(kObjectClass)) && !kIsNull(xkObject_RawType(candidateObject)) && xkObject_RawVerifyTag(candidateObject))
                {
                    kType candidateType = xkObject_RawType(candidateObject); 
                    kSize i; 

                    //determine whether the candidate object header has a type field that matches a known 
                    //type in the given assembly (a hash set of types in the assembly would speed this up)
                    for (i = 0; i < typeCount; ++i)
                    {
                        kType type = kAssembly_TypeAt(assembly, i); 

                        if ((type == candidateType) && (allocation->size == kType_InnerSize(type)))
                        {
                            const kChar* typeName = kType_Name(type); 

                            kTest(kMap_ReplaceT(obj->candidateObjectLeaks, &allocation->index, typeName)); 
                        }
                    }
                }
            }

            it = kMap_Next(obj->history, it); 
        }       
    }
    kFinally
    {
        kCheck(kLock_Exit(obj->lock)); 

        kEndFinally(); 
    }    

    return kOK; 
}

kFx(kStatus) kDebugAlloc_LogAllocations(kDebugAlloc object, k64u since)
{
    kObj(kDebugAlloc, object); 
    kArrayList allocations = kNULL; 
    kSize i = 0; 
    kSize j = 0; 

    kTry
    {
        kTest(kDebugAlloc_Allocations(object, since, &allocations, kObject_Alloc(object))); 

        for (i = 0; i < kArrayList_Count(allocations); ++i)
        {
            const kDebugAllocation* allocation = kArrayList_AtT(allocations, i, kDebugAllocation); 
            const kSize DUMP_COUNT = 32; 
            const kSize DUMP_HEADER = 7; 
            const kSize DUMP_ITEM = 3; 
            kSize dumpCount = kMin_(DUMP_COUNT, allocation->size); 
            kText256 dumpMessage; 
            kTypeName typeName; 

            kLogf("%s leak - index: %llu, address: 0x%llX, size: %llu.", obj->name, (k64u)allocation->index, (k64u)(kSize)allocation->data, (k64u)allocation->size); 
        
            //dump header (must match header size constant above)
            kStrPrintf(dumpMessage, kCountOf(dumpMessage), "  Data:"); 

            //dump bytes (must match item size constant above)
            for (j = 0; j < dumpCount; ++j)
            {
                kStrPrintf(&dumpMessage[DUMP_HEADER + j*DUMP_ITEM], DUMP_ITEM+1, " %02X", allocation->data[j]); 
            }

            kLogf("%s ...", dumpMessage); 

            //print object type, if known
            if (kSuccess(kMap_FindT(obj->candidateObjectLeaks, &allocation->index, &typeName)))
            {
                kLogf("  Type: %s", typeName); 
            }

            //print stack trace, if known
            if (!kIsNull(allocation->trace) && (kArrayList_Count(allocation->trace) > 0))
            {
                kLogf("  Trace:");  

                for (j = 0; j < kArrayList_Count(allocation->trace); ++j)
                {
                    kString line = kArrayList_AsT(allocation->trace, j, kString); 
                    
                    kLogf("    %s", kString_Chars(line));  
                }
            }
  
            kLogf(""); 
        }       
    }
    kFinally
    {
        kCheck(kObject_Destroy(allocations)); 

        kEndFinally(); 
    }    

    return kOK; 
}

kFx(kStatus) kDebugAlloc_SetAllocListener(kDebugAlloc object, kCallbackFx function, kPointer receiver)
{
    kObj(kDebugAlloc, object); 
   
    kCheck(kLock_Enter(obj->lock)); 
    {
        obj->allocListener.function = function; 
        obj->allocListener.receiver = receiver; 
    }
    kCheck(kLock_Exit(obj->lock)); 

    return kOK; 
}

kFx(kSize) kDebugAlloc_Allocated(kDebugAlloc object)
{
    kObj(kDebugAlloc, object); 
    kSize allocated = 0; 

    kLock_Enter(obj->lock); 
    {
        allocated = obj->allocated; 
    }
    kLock_Exit(obj->lock); 

    return allocated; 
}

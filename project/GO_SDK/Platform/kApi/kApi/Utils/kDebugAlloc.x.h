/** 
 * @file    kDebugAlloc.x.h
 *
 * @internal
 * Copyright (C) 2012-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_DEBUG_ALLOC_X_H
#define K_API_DEBUG_ALLOC_X_H

#include <kApi/kAlloc.h>

#define xkDEBUG_ALLOC_PAD_SIZE          (8)             //size of memory before and after each allocation to fill with test pattern

kDeclareValueEx(k, kDebugAllocation, kValue)

typedef struct kDebugAllocClass
{
    kAllocClass base; 
    kObject innerAlloc;             //inner allocated, used to allocate requested memory
    kLock lock;                     //provides mutual exclusion
    kText32 name;                   //descriptive name for this allocator
    kCallback allocListener;        //notified on allocations
    kSize allocated;                //count of allocated memory, in bytes
    k64u counter;                   //incremented with each allocation
    kMap history;                   //records all outstanding allocations -- kMap<kPointer, kDebugAllocation>
    kArray1 padPattern;             //known pattern before and after each allocation (used to check for corruption)
    kMap candidateObjectLeaks;      //map of potential object leaks (allocation index to type name) -- kMap<k64u, kTypeName>
    kBackTrace backTrace;           //temporary variable used to store back trace information.
    kMap backTraceDescriptions;     //map of allocation back trace information -- kMap<kBackTrace, kArrayList<kString>>
} kDebugAllocClass;

kDeclareClassEx(k, kDebugAlloc, kAlloc)

/* 
* Private methods. 
*/

kFx(kStatus) xkDebugAlloc_Init(kDebugAlloc object, kType type, const kChar* name, kAlloc innerAlloc, kAlloc alloc); 
kFx(kStatus) xkDebugAlloc_VRelease(kDebugAlloc object); 

kFx(kStatus) xkDebugAlloc_CandidateObjectAllocations(kDebugAlloc object, k64u since, kArrayList* history, kAlloc alloc);
kFx(kStatus) xkDebugAlloc_InitPadPattern(kDebugAlloc object); 

kFx(kStatus) xkDebugAlloc_VGet(kDebugAlloc object, kSize size, void* mem, kMemoryAlignment alignment);
kFx(kStatus) xkDebugAlloc_VFree(kDebugAlloc object, void* mem); 

#endif

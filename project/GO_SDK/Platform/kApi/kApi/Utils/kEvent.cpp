/** 
 * @file    kEvent.cpp
 *
 * @internal
 * Copyright (C) 2012-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <kApi/Utils/kEvent.h>
#include <kApi/Data/kList.h>

kBeginClassEx(k, kEvent)

    kAddFrameworkConstructor(kEvent, Construct);

    kAddPrivateVMethod(kEvent, kObject, VRelease)
    kAddPrivateVMethod(kEvent, kObject, VClone)
kEndClassEx()

kFx(kStatus) kEvent_Construct(kEvent* evnt, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback(allocator);
    kStatus status; 

    kCheck(kAlloc_GetObject(alloc, kTypeOf(kEvent), evnt)); 

    if (!kSuccess(status = xkEvent_Init(*evnt, kTypeOf(kEvent), alloc)))
    {
        kAlloc_FreeRef(alloc, evnt); 
    }

    return status; 
}

kFx(kStatus) xkEvent_Init(kEvent evnt, kType type, kAlloc allocator)
{
    kObjR(kEvent, evnt); 
    kStatus status = kOK; 

    kCheck(kObject_Init(evnt, type, allocator)); 

    obj->listeners = kNULL;
    obj->notifyIt = kNULL;

    kTry
    {
        kTest(kList_Construct(&obj->listeners, kTypeOf(kCallback), 0, allocator)); 
    }
    kCatch(&status); 
    {
        xkEvent_VRelease(evnt); 
        kEndCatch(status); 
    }

    return kOK; 
}

kFx(kStatus) xkEvent_VClone(kEvent evnt, kEvent source, kAlloc valueAlloc, kObject context)
{
    kObj(kEvent, evnt); 

    return kList_Assign(obj->listeners, kEvent_Listeners(source)); 
}

kFx(kStatus) xkEvent_VRelease(kEvent evnt)
{
    kObj(kEvent, evnt); 

    kCheck(kObject_Destroy(obj->listeners)); 

    kCheck(kObject_VRelease(evnt)); 

    return kOK; 
}

kFx(kStatus) kEvent_Add(kEvent evnt, kCallbackFx function, kPointer receiver)
{
    kObj(kEvent, evnt); 
    kCallback entry; 

    entry.function = function; 
    entry.receiver = receiver; 

    kCheck(kList_AddT(obj->listeners, &entry, kNULL)); 

    return kOK; 
}

kFx(kStatus) kEvent_Remove(kEvent evnt, kCallbackFx function, kPointer receiver)
{
    kObj(kEvent, evnt); 
    kListItem it = kList_First(obj->listeners); 

    while (!kIsNull(it))
    {
        kCallback* entry = kList_AtT(obj->listeners, it, kCallback); 
        
        if ((entry->function == function) && (entry->receiver == receiver))
        {
            //adjust notification iterator, if necessary
            if (it == obj->notifyIt)
            {
                obj->notifyIt = kList_Next(obj->listeners, obj->notifyIt); 
            }

            return kList_Remove(obj->listeners, it); 
        }

        it = kList_Next(obj->listeners, it); 
    }

    return kERROR_NOT_FOUND; 
}

kFx(kStatus) kEvent_Clear(kEvent evnt)
{
    kObj(kEvent, evnt); 

    kCheck(kList_Clear(obj->listeners)); 
    obj->notifyIt = kNULL; 

    return kOK; 
}

kFx(kStatus) kEvent_Notify(kEvent evnt, kPointer sender, void* args)
{
    kObj(kEvent, evnt); 
    kStatus status = kOK; 
    kStatus callStatus; 

    //break notification cycles
    if (!kIsNull(obj->notifyIt))
    {
        return kOK; 
    }

    //share the event, to prevent self-destruction during notification callbacks
    kCheck(kObject_Share(evnt)); 

    obj->notifyIt = kList_First(obj->listeners); 

    while (!kIsNull(obj->notifyIt))
    {
        kCallback entry = kList_AsT(obj->listeners, obj->notifyIt, kCallback); 

        obj->notifyIt = kList_Next(obj->listeners, obj->notifyIt); 

        callStatus = entry.function(entry.receiver, sender, args); 
        status = kSuccess(status) ? callStatus : status; 
    }

    obj->notifyIt = kNULL; 
    kObject_Destroy(evnt); 

    return status; 
}




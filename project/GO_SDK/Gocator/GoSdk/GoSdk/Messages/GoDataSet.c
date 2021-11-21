/**
 * @file    GoDataSet.c
 *
 * @internal
 * Copyright (C) 2016-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <GoSdk/Messages/GoDataSet.h>

kBeginClassEx(Go, GoDataSet)

    //serialization versions
    kAddVersion(GoDataSet, "kdat6", "4.0.0.0", "GoDataSet-0", WriteDat6V0, ReadDat6V0)

    //virtual methods
    kAddVMethod(GoDataSet, kObject, VRelease)
    kAddVMethod(GoDataSet, kObject, VInitClone)
    kAddVMethod(GoDataSet, kObject, VSize)

kEndClassEx()

GoFx(kStatus) GoDataSet_Construct(GoDataSet* set, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback(allocator);
    kStatus status;

    kCheck(kAlloc_GetObject(alloc, kTypeOf(GoDataSet), set));

    if (!kSuccess(status = GoDataSet_Init(*set, kTypeOf(GoDataSet), alloc)))
    {
        kAlloc_FreeRef(alloc, set);
    }

    return status;
}

GoFx(kStatus) GoDataSet_Init(GoDataSet set, kType type, kAlloc alloc)
{
    kObjR(GoDataSet, set);

    kCheck(kObject_Init(set, type, alloc));

    obj->senderId = 0;
    kZero(obj->content);

    return kOK;
}

GoFx(kStatus) GoDataSet_VInitClone(GoDataSet set, GoDataSet source, kAlloc alloc)
{
    kObjR(GoDataSet, set);
    kObjNR(GoDataSet, srcObj, source);
    kStatus status;

    kCheck(GoDataSet_Init(set, kObject_Type(source), alloc));

    kTry
    {
        obj->senderId = srcObj->senderId;

        kTest(kObject_Clone(&obj->content, srcObj->content, alloc));
    }
    kCatch(&status)
    {
        GoDataSet_VRelease(set);
        kEndCatch(status);
    }

    return kOK;
}

GoFx(kStatus) GoDataSet_VRelease(GoDataSet set)
{
    kObjR(GoDataSet, set);

    kCheck(kObject_Dispose(obj->content));

    kCheck(kObject_VRelease(set));

    return kOK;
}

GoFx(kStatus) GoDataSet_Allocate(GoDataSet set, kSize minimumCapacity)
{
    kObjR(GoDataSet, set);

    kCheck(kDisposeRef(&obj->content));
    kCheck(kArrayList_Construct(&obj->content, kTypeOf(kObject), minimumCapacity, kObject_Alloc(set)));

    return kOK;
}

GoFx(kStatus) GoDataSet_Add(GoDataSet set, kObject item)
{
    kObjR(GoDataSet, set);

    if (kIsNull(obj->content))
    {
        kCheck(GoDataSet_Allocate(set, 1));
    }

    kCheck(kArrayList_AddT(obj->content, &item));

    return kOK;
}

GoFx(kSize) GoDataSet_VSize(GoDataSet set)
{
    kObjR(GoDataSet, set);
    return sizeof(GoDataSetClass) + (kIsNull(obj->content) ? 0 : kObject_Size(obj->content));
}

GoFx(kStatus) GoDataSet_WriteDat6V0(GoDataSet set, kSerializer serializer)
{
    kObjR(GoDataSet, set);

    kCheck(kSerializer_BeginWrite(serializer, kTypeOf(k16u), kFALSE));
    kCheck(kSerializer_Write32u(serializer, obj->senderId));
    kCheck(kSerializer_EndWrite(serializer));

    kCheck(kSerializer_WriteObject(serializer, obj->content));

    return kOK;
}

GoFx(kStatus) GoDataSet_ReadDat6V0Header(GoDataSet set, kSerializer serializer, k32u* senderId)
{
    // Read and store the message attributes data length ("attrSize")
    // to determine how many attributes to read.
    //
    // If this fails, the EndRead() could assert so skip the EndRead() if BeginRead() fails.
    kCheck(kSerializer_BeginRead(serializer, kTypeOf(k16u), kFALSE));

    kTry
    {
        kTest(kSerializer_Read32u(serializer, senderId));
    }
    kFinally
    {
        // Automatically flush any unread data and free serializer read section.
        // Ignore errors.
        kSerializer_EndRead(serializer);

        kEndFinally();
    }

    return kOK;
}

GoFx(kStatus) GoDataSet_ReadDat6V0(GoDataSet set, kSerializer serializer, kAlloc alloc)
{
    kObjR(GoDataSet, set);
    kStatus status;

    kCheck(GoDataSet_Init(set, kTypeOf(GoDataSet), alloc));

    kTry
    {
        kTest(GoDataSet_ReadDat6V0Header(set, serializer, &obj->senderId));

        kTest(kSerializer_ReadObject(serializer, &obj->content, alloc));
    }
    kCatch(&status)
    {
        GoDataSet_VRelease(set);

        kEndCatch(status);
    }

    return kOK;
}

GoFx(k32u) GoDataSet_SenderId(GoDataSet set)
{
    kObjR(GoDataSet, set);
    return obj->senderId;
}

GoFx(kSize) GoDataSet_Count(GoDataSet set)
{
    kObjR(GoDataSet, set);
    return kArrayList_Count(obj->content);
}

GoFx(kObject) GoDataSet_At(GoDataSet set, kSize index)
{
    kObjR(GoDataSet, set);

    kAssert(index < kArrayList_Count(obj->content));

    return kArrayList_AsT(obj->content, index, kObject);
}

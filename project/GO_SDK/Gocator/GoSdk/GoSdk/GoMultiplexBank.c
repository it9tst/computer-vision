/** 
 * @file    GoMultiplexBank.c
 *
 * @internal
 * Copyright (C) 2016-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <GoSdk/GoMultiplexBank.h>
#include <GoSdk/GoSensor.h>
#include <GoSdk/GoUtils.h>

kBeginClassEx(Go, GoMultiplexBank)
    kAddVMethod(GoMultiplexBank, kObject, VRelease)
kEndClassEx()

GoFx(kStatus) GoMultiplexBank_Construct(GoMultiplexBank* bank, k32u id, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback(allocator);
    kStatus status; 

    kCheck(kAlloc_GetObject(alloc, kTypeOf(GoMultiplexBank), bank)); 

    if (!kSuccess(status = GoMultiplexBank_Init(*bank, kTypeOf(GoMultiplexBank), id, alloc)))
    {
        kAlloc_FreeRef(alloc, bank); 
    }

    return status; 
} 

GoFx(kStatus) GoMultiplexBank_Init(GoMultiplexBank bank, kType type,  k32u id, kAlloc alloc)
{
    kObjR(GoMultiplexBank, bank); 
    kStatus exception;

    kCheck(kObject_Init(bank, type, alloc)); 
    kZero(obj->sensor);
    kZero(obj->xml);
    kZero(obj->xmlItem);
    kZero(obj->sensorList);

    obj->id = id; 

    kTry
    {
        kTest(kArrayList_Construct(&obj->sensorList, kTypeOf(GoSensor), 0, alloc));
    }
    kCatch(&exception)
    {
        GoMultiplexBank_VRelease(bank);
        kEndCatch(exception);
    }

    return kOK; 
}

GoFx(kStatus) GoMultiplexBank_VRelease(GoMultiplexBank bank)
{
    kObj(GoMultiplexBank, bank); 

    kCheck(kDestroyRef(&obj->sensorList));  //dispose not called - do not want to destroy all sensor handles, as they may still be used elsewhere

    return kObject_VRelease(bank);
}

GoFx(kStatus) GoMultiplexBank_AddSensor(GoMultiplexBank bank, GoSensor sensor)
{
    kObj(GoMultiplexBank, bank); 
    kSize i;   

    for (i = 0; i < kArrayList_Count(obj->sensorList); i++)
    {
        if (GoSensor_Id(sensor) == GoSensor_Id(kArrayList_AsT(obj->sensorList, i, GoSensor)))
        {
            return kERROR_ALREADY_EXISTS;
        }
    }

    kCheck(kArrayList_AddT(obj->sensorList, &sensor));

    return kOK;
}

GoFx(kStatus) GoMultiplexBank_RemoveSensor(GoMultiplexBank bank, k32u id)
{
    kObj(GoMultiplexBank, bank); 
    kSize i;

    for (i = 0; i < kArrayList_Count(obj->sensorList); i++)
    {
        if (id == GoSensor_Id(kArrayList_AsT(obj->sensorList, i, GoSensor)))
        {
            kCheck(kArrayList_Discard(obj->sensorList, i));

            return kOK;
        }
    }

    return kERROR_NOT_FOUND;
}

GoFx(kSize) GoMultiplexBank_SensorCount(GoMultiplexBank bank)
{
    kObj(GoMultiplexBank, bank); 

    return kArrayList_Count(obj->sensorList);
}

GoFx(GoSensor) GoMultiplexBank_SensorAt(GoMultiplexBank bank, kSize index)
{
    kObj(GoMultiplexBank, bank); 

    kAssert(index < kArrayList_Count(obj->sensorList));

    return kArrayList_AsT(obj->sensorList, index, GoSensor);
}

GoFx(kBool) GoMultiplexBank_HasSensor(GoMultiplexBank bank, k32u id)
{
    kObj(GoMultiplexBank, bank); 
    kSize i;

    for (i = 0; i < kArrayList_Count(obj->sensorList); i++)
    {
        if (id == GoSensor_Id(kArrayList_AsT(obj->sensorList, i, GoSensor)))
        {
            return kTRUE;
        }
    }

    return kFALSE;
}

GoFx(k32u) GoMultiplexBank_Id(GoMultiplexBank bank)
{
    kObj(GoMultiplexBank, bank); 

    return obj->id;
}

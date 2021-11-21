/**
 * @file    GoHealthMsg.c
 *
 * @internal
 * Copyright (C) 2016-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <GoSdk/Messages/GoHealth.h>
#include <GoSdk/Internal/GoSerializer.h>

/*
* GoHealthIndicatorId
*/

kBeginEnumEx(Go, GoHealthIndicatorId)
    kAddEnumerator(GoHealthIndicatorId, GO_HEALTH_ENCODER_VALUE)
    kAddEnumerator(GoHealthIndicatorId, GO_HEALTH_ENCODER_FREQUENCY)
    kAddEnumerator(GoHealthIndicatorId, GO_HEALTH_LASER_SAFETY)
    kAddEnumerator(GoHealthIndicatorId, GO_HEALTH_FIRMWARE_VERSION)
    kAddEnumerator(GoHealthIndicatorId, GO_HEALTH_FIRESYNC_VERSION)
    kAddEnumerator(GoHealthIndicatorId, GO_HEALTH_UPTIME)
    kAddEnumerator(GoHealthIndicatorId, GO_HEALTH_TEMPERATURE)
    kAddEnumerator(GoHealthIndicatorId, GO_HEALTH_TEMPERATURE_EXTENDED)
    kAddEnumerator(GoHealthIndicatorId, GO_HEALTH_PROJECTOR_TEMPERATURE)
    kAddEnumerator(GoHealthIndicatorId, GO_HEALTH_LASER_TEMPERATURE)
    kAddEnumerator(GoHealthIndicatorId, GO_HEALTH_LASER_OVERHEAT)
    kAddEnumerator(GoHealthIndicatorId, GO_HEALTH_LASER_OVERHEAT_DURATION)
    kAddEnumerator(GoHealthIndicatorId, GO_HEALTH_OVERHEAT)
    kAddEnumerator(GoHealthIndicatorId, GO_HEALTH_OVERHEAT_DURATION)
    kAddEnumerator(GoHealthIndicatorId, GO_HEALTH_CPU_TEMPERATURE)
    kAddEnumerator(GoHealthIndicatorId, GO_HEALTH_CAMERA_0_TEMPERATURE)
    kAddEnumerator(GoHealthIndicatorId, GO_HEALTH_CAMERA_1_TEMPERATURE)
    kAddEnumerator(GoHealthIndicatorId, GO_HEALTH_LASER_DRIVER_TEMPERATURE)
    kAddEnumerator(GoHealthIndicatorId, GO_HEALTH_MEMORY_USED)
    kAddEnumerator(GoHealthIndicatorId, GO_HEALTH_MEMORY_CAPACITY)
    kAddEnumerator(GoHealthIndicatorId, GO_HEALTH_STORAGE_USED)
    kAddEnumerator(GoHealthIndicatorId, GO_HEALTH_STORAGE_CAPACITY)
    kAddEnumerator(GoHealthIndicatorId, GO_HEALTH_CPU_USED)
    kAddEnumerator(GoHealthIndicatorId, GO_HEALTH_SYNC_SOURCE)
    kAddEnumerator(GoHealthIndicatorId, GO_HEALTH_NET_OUT_USED)
    kAddEnumerator(GoHealthIndicatorId, GO_HEALTH_NET_OUT_RATE)
    kAddEnumerator(GoHealthIndicatorId, GO_HEALTH_NET_OUT_CAPACITY)
    kAddEnumerator(GoHealthIndicatorId, GO_HEALTH_NET_OUT_LINK_STATUS)
    kAddEnumerator(GoHealthIndicatorId, GO_HEALTH_DIGITAL_INPUTS)
    kAddEnumerator(GoHealthIndicatorId, GO_HEALTH_EVENT_COUNTS)
    kAddEnumerator(GoHealthIndicatorId, GO_HEALTH_CAMERA_SEARCH_COUNT)
    kAddEnumerator(GoHealthIndicatorId, GO_HEALTH_CAMERA_TRIGGER_DROPS)
    kAddEnumerator(GoHealthIndicatorId, GO_HEALTH_CUDA_STATUS)
    kAddEnumerator(GoHealthIndicatorId, GO_HEALTH_STATE)
    kAddEnumerator(GoHealthIndicatorId, GO_HEALTH_SPEED)
    kAddEnumerator(GoHealthIndicatorId, GO_HEALTH_MAXSPEED)
    kAddEnumerator(GoHealthIndicatorId, GO_HEALTH_SPOT_COUNT)
    kAddEnumerator(GoHealthIndicatorId, GO_HEALTH_MAX_SPOT_COUNT)
    kAddEnumerator(GoHealthIndicatorId, GO_HEALTH_SCAN_COUNT)
    kAddEnumerator(GoHealthIndicatorId, GO_HEALTH_VALID_POINT_COUNT)
    kAddEnumerator(GoHealthIndicatorId, GO_HEALTH_MAX_POINT_COUNT)
    kAddEnumerator(GoHealthIndicatorId, GO_HEALTH_MASTER_STATUS)
    kAddEnumerator(GoHealthIndicatorId, GO_HEALTH_CAST_START_STATE)
    kAddEnumerator(GoHealthIndicatorId, GO_HEALTH_ALIGNMENT_STATE)
    kAddEnumerator(GoHealthIndicatorId, GO_HEALTH_PLAYBACK_POSITION)
    kAddEnumerator(GoHealthIndicatorId, GO_HEALTH_PLAYBACK_COUNT)
    kAddEnumerator(GoHealthIndicatorId, GO_HEALTH_DIGITAL_OUTPUT_HIGH_COUNT)
    kAddEnumerator(GoHealthIndicatorId, GO_HEALTH_DIGITAL_OUTPUT_LOW_COUNT)
    kAddEnumerator(GoHealthIndicatorId, GO_HEALTH_PROCESSING_LATENCY_LAST)
    kAddEnumerator(GoHealthIndicatorId, GO_HEALTH_PROCESSING_LATENCY_MAX)
    kAddEnumerator(GoHealthIndicatorId, GO_HEALTH_PROCESSING_DROPS)
    kAddEnumerator(GoHealthIndicatorId, GO_HEALTH_TRIGGER_DROPS)
    kAddEnumerator(GoHealthIndicatorId, GO_HEALTH_OUTPUT_DROPS)
    kAddEnumerator(GoHealthIndicatorId, GO_HEALTH_ANALOG_DROPS)
    kAddEnumerator(GoHealthIndicatorId, GO_HEALTH_DIGITAL_DROPS)
    kAddEnumerator(GoHealthIndicatorId, GO_HEALTH_SERIAL_DROPS)
    kAddEnumerator(GoHealthIndicatorId, GO_HEALTH_CONTROLLED_TRIGGER_DROPS)
    kAddEnumerator(GoHealthIndicatorId, GO_HEALTH_SURFACE_PROCESSING_TIME)
    kAddEnumerator(GoHealthIndicatorId, GO_HEALTH_MAX_FRAME_RATE)
    kAddEnumerator(GoHealthIndicatorId, GO_HEALTH_ETHERNET_DROPS)
    kAddEnumerator(GoHealthIndicatorId, GO_HEALTH_RANGE_VALID_COUNT)
    kAddEnumerator(GoHealthIndicatorId, GO_HEALTH_RANGE_INVALID_COUNT)
    kAddEnumerator(GoHealthIndicatorId, GO_HEALTH_ANCHOR_INVALID_COUNT)
    kAddEnumerator(GoHealthIndicatorId, GO_HEALTH_LIGHT_OPERATIONAL_TIME_TOTAL)
    kAddEnumerator(GoHealthIndicatorId, GO_HEALTH_FIRST_LOG_ID)
    kAddEnumerator(GoHealthIndicatorId, GO_HEALTH_LAST_LOG_ID)
    kAddEnumerator(GoHealthIndicatorId, GO_HEALTH_ENCODER_Z_INDEX_PULSE_DROPS)

    kAddEnumerator(GoHealthIndicatorId, GO_HEALTH_TOOL_RUN_TIME)
    kAddEnumerator(GoHealthIndicatorId, GO_HEALTH_PART_TOTAL_EMITTED)
    kAddEnumerator(GoHealthIndicatorId, GO_HEALTH_PART_LENGTH_LIMIT)
    kAddEnumerator(GoHealthIndicatorId, GO_HEALTH_PART_MIN_AREA_DROPS)
    kAddEnumerator(GoHealthIndicatorId, GO_HEALTH_PART_BACKTRACK_DROPS)
    kAddEnumerator(GoHealthIndicatorId, GO_HEALTH_PART_CURRENTLY_ACTIVE)
    kAddEnumerator(GoHealthIndicatorId, GO_HEALTH_PART_LENGTH)
    kAddEnumerator(GoHealthIndicatorId, GO_HEALTH_PART_START_Y)
    kAddEnumerator(GoHealthIndicatorId, GO_HEALTH_PART_TRACKING_STATE)
    kAddEnumerator(GoHealthIndicatorId, GO_HEALTH_PART_CAPACITY_EXCEEDED)
    kAddEnumerator(GoHealthIndicatorId, GO_HEALTH_PART_X_POSITION)
    kAddEnumerator(GoHealthIndicatorId, GO_HEALTH_TOOL_RUN_TIME_MIN)
    kAddEnumerator(GoHealthIndicatorId, GO_HEALTH_TOOL_RUN_TIME_MAX)
    kAddEnumerator(GoHealthIndicatorId, GO_HEALTH_TOOL_RUN_TIME_AVERAGE)
    kAddEnumerator(GoHealthIndicatorId, GO_HEALTH_TOOL_RUN_TIME_PERCENT)
    kAddEnumerator(GoHealthIndicatorId, GO_HEALTH_MEASUREMENT)
    kAddEnumerator(GoHealthIndicatorId, GO_HEALTH_MEASUREMENT_PASS)
    kAddEnumerator(GoHealthIndicatorId, GO_HEALTH_MEASUREMENT_FAIL)
    kAddEnumerator(GoHealthIndicatorId, GO_HEALTH_MEASUREMENT_MIN)
    kAddEnumerator(GoHealthIndicatorId, GO_HEALTH_MEASUREMENT_MAX)
    kAddEnumerator(GoHealthIndicatorId, GO_HEALTH_MEASUREMENT_AVERAGE)
    kAddEnumerator(GoHealthIndicatorId, GO_HEALTH_MEASUREMENT_STDEV)
    kAddEnumerator(GoHealthIndicatorId, GO_HEALTH_MEASUREMENT_INVALID_COUNT)
    kAddEnumerator(GoHealthIndicatorId, GO_HEALTH_MEASUREMENT_OVERFLOW_COUNT)
kEndEnumEx()

/*
 * GoIndicator
 */

kBeginValueEx(Go, GoIndicator)
    kAddField(GoIndicator, k32u, id)
    kAddField(GoIndicator, k32u, instance)
    kAddField(GoIndicator, k64s, value)
kEndValueEx()


/*
 * GoHealthMsg
 */

kBeginClassEx(Go, GoHealthMsg)

    //serialization versions
    kAddVersion(GoHealthMsg, GO_SERIALIZATION_FORMAT_NAME, GO_SERIALIZATION_FORMAT_VERSION, GoSerializerTypeIdStr(GO_COMPACT_MESSAGE_HEALTH), WriteV0, ReadV0)
    kAddVersion(GoHealthMsg, "kdat6", "4.0.0.0", "GoHealthMsg-0", WriteV0, ReadV0)

    //virtual methods
    kAddVMethod(GoHealthMsg, kObject, VInitClone)
    kAddVMethod(GoHealthMsg, kObject, VRelease)
    kAddVMethod(GoHealthMsg, kObject, VSize)

kEndClassEx()

GoFx(kStatus) GoHealthMsg_Construct(GoHealthMsg* msg, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback(allocator);
    kStatus status;

    kCheck(kAlloc_GetObject(alloc, kTypeOf(GoHealthMsg), msg));

    if (!kSuccess(status = GoHealthMsg_Init(*msg, kTypeOf(GoHealthMsg), alloc)))
    {
        kAlloc_FreeRef(alloc, msg);
    }

    return status;
}

GoFx(kStatus) GoHealthMsg_Init(GoHealthMsg msg, kType type, kAlloc alloc)
{
    kObjR(GoHealthMsg, msg);

    kCheck(kObject_Init(msg, type, alloc));
    obj->source = GO_DATA_SOURCE_TOP;
    kZero(obj->indicators);

    return kOK;
}

GoFx(kStatus) GoHealthMsg_VInitClone(GoHealthMsg msg, GoHealthMsg source, kAlloc alloc)
{
    kObjR(GoHealthMsg, msg);
    kObjNR(GoHealthMsg, srcObj, source);
    kStatus status;

    kCheck(GoHealthMsg_Init(msg, kObject_Type(source), alloc));

    kTry
    {
        obj->source = srcObj->source;

        kTest(kObject_Clone(&obj->indicators, srcObj->indicators, alloc));
    }
    kCatch(&status)
    {
        GoHealthMsg_VRelease(msg);
        kEndCatch(status);
    }

    return kOK;
}

GoFx(kStatus) GoHealthMsg_Allocate(GoHealthMsg msg, kSize count)
{
    kObjR(GoHealthMsg, msg);

    kCheck(kDestroyRef(&obj->indicators));
    kCheck(kArray1_Construct(&obj->indicators, kTypeOf(GoIndicator), count, kObject_Alloc(msg)));

    return kOK;
}

GoFx(kStatus) GoHealthMsg_VRelease(GoHealthMsg msg)
{
    kObjR(GoHealthMsg, msg);

    kCheck(kObject_Destroy(obj->indicators));

    kCheck(kObject_VRelease(msg));

    return kOK;
}

GoFx(kSize) GoHealthMsg_VSize(GoHealthMsg msg)
{
    kObjR(GoHealthMsg, msg);

    return sizeof(GoHealthMsgClass) + (kIsNull(obj->indicators) ? 0 : kObject_Size(obj->indicators));
}

GoFx(kStatus) GoHealthMsg_WriteV0(GoHealthMsg msg, kSerializer serializer)
{
    kObjR(GoHealthMsg, msg);
    k32u count = (k32u) kArray1_Count(obj->indicators);
    k32u i;

    kCheck(kSerializer_Write32u(serializer, count));
    kCheck(kSerializer_Write8u(serializer, (k8u) obj->source));
    kCheck(kSerializer_Write8u(serializer, 0));
    kCheck(kSerializer_Write8u(serializer, 0));
    kCheck(kSerializer_Write8u(serializer, 0));

    for (i = 0; i < count; ++i)
    {
        const GoIndicator* indicator = kArray1_AtT(obj->indicators, i, GoIndicator);

        kCheck(kSerializer_Write32u(serializer, indicator->id));
        kCheck(kSerializer_Write32u(serializer, indicator->instance));
        kCheck(kSerializer_Write64s(serializer, indicator->value));
    }

    return kOK;
}

GoFx(kStatus) GoHealthMsg_ReadV0(GoHealthMsg msg, kSerializer serializer, kAlloc alloc)
{
    kObjR(GoHealthMsg, msg);
    kStatus status;
    k8u source = 0;
    k8u reserved;
    k32u count = 0;
    k32u i;

    kCheck(GoHealthMsg_Init(msg, kTypeOf(GoHealthMsg), alloc));

    kTry
    {
        kTest(kSerializer_Read32u(serializer, &count));
        kTest(kSerializer_Read8u(serializer, &source));
        kTest(kSerializer_Read8u(serializer, &reserved));
        kTest(kSerializer_Read8u(serializer, &reserved));
        kTest(kSerializer_Read8u(serializer, &reserved));

        obj->source = (GoDataSource)source;

        kTest(kArray1_Construct(&obj->indicators, kTypeOf(GoIndicator), count, alloc));

        for (i = 0; i < count; ++i)
        {
            GoIndicator* indicator = kArray1_AtT(obj->indicators, i, GoIndicator);

            kTest(kSerializer_Read32u(serializer, &indicator->id));
            kTest(kSerializer_Read32u(serializer, &indicator->instance));
            kTest(kSerializer_Read64s(serializer, &indicator->value));
        }
    }
    kCatch(&status)
    {
        GoHealthMsg_VRelease(msg);
        kEndCatch(status);
    }

    return kOK;
}

GoFx(GoDataSource) GoHealthMsg_Source(GoHealthMsg msg)
{
    return xGoHealthMsg_CastRaw(msg)->source;
}

GoFx(kSize) GoHealthMsg_Count(GoHealthMsg msg)
{
    return kArray1_Count(GoHealthMsg_Content_(msg));
}

GoFx(GoIndicator*) GoHealthMsg_At(GoHealthMsg msg, kSize index)
{
    kAssert(index < GoHealthMsg_Count(msg));

    return kArray1_AtT(GoHealthMsg_Content_(msg), index, GoIndicator);
}

GoFx(GoIndicator*) GoHealthMsg_Find(GoHealthMsg msg, k32u id, k32u instance)
{
    GoIndicator* indicator;
    kSize i;

    for (i = 0; i < GoHealthMsg_Count(msg); i++)
    {
        indicator = GoHealthMsg_At(msg, i);

        if (indicator->id == id && indicator->instance == instance)
        {
            return indicator;
        }
    }

    return kNULL;
}

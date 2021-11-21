/**
 * @file    GoAccelSensorPortAlloc.h
 * @brief   Declares the GoAccelSensorPortAlloc class.
 *
 * @internal
 * Copyright (C) 2016-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef GO_ACCEL_SENSOR_PORT_ALLOC_H
#define GO_ACCEL_SENSOR_PORT_ALLOC_H

#include <GoSdk/GoSdkDef.h>

typedef kObject GoAccelSensorPortAlloc;

typedef struct GoAccelSensorPortAllocPorts
{
    k16u controlPort;
    k16u upgradePort;
    k16u healthPort;
    k16u privateDataPort;
    k16u publicDataPort;
    k16u webPort;
} GoAccelSensorPortAllocPorts;

GoFx(kStatus) GoAccelSensorPortAlloc_Construct(GoAccelSensorPortAlloc* portAlloc, kAlloc allocator);

GoFx(kStatus) GoAccelSensorPortAlloc_SetPortRange(GoAccelSensorPortAlloc portAlloc, k16u startPort, k16u endPort);
GoFx(kStatus) GoAccelSensorPortAlloc_GetPortRange(GoAccelSensorPortAlloc portAlloc, k16u* startPort, k16u* endPort);
GoFx(kStatus) GoAccelSensorPortAlloc_GetPortRangeLimits(GoAccelSensorPortAlloc portAlloc, k16u* startLimit, k16u* endLimit, k16u* minNumPorts);

GoFx(kStatus) GoAccelSensorPortAlloc_AllocatePorts(GoAccelSensorPortAlloc portAlloc, k32u sensorId, GoAccelSensorPortAllocPorts* sensorPorts);
GoFx(kStatus) GoAccelSensorPortAlloc_AllocateRestoredPorts(GoAccelSensorPortAlloc portAlloc, k32u sensorId, GoAccelSensorPortAllocPorts* sensorPorts);

GoFx(kBool) GoAccelSensorPortAlloc_UserSelectedPortsFree(GoAccelSensorPortAlloc portAlloc, GoAccelSensorPortAllocPorts* sensorPorts, kBool doAllocationCheck);
GoFx(kStatus) GoAccelSensorPortAlloc_RemoveAllocatedPorts(GoAccelSensorPortAlloc portAlloc, k32u sensorId, GoAccelSensorPortAllocPorts* sensorPorts);

// Used for unit testing.
GoFx(kStatus) GoAccelSensorPortAlloc_PurgeAllAllocatedPorts(GoAccelSensorPortAlloc portAlloc);

#include <GoSdk/Internal/GoAccelSensorPortAlloc.x.h>

#endif  // GO_ACCEL_SENSOR_PORT_ALLOC_H

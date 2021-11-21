/**
 * @file    GoAccelSensorPortAlloc.x.h
 *
 * @internal
 * Copyright (C) 2016-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef GO_ACCEL_SENSOR_PORT_ALLOC_X_H
#define GO_ACCEL_SENSOR_PORT_ALLOC_X_H

#include <kApi/Data/kMap.h>
#include <kApi/Io/kSocket.h>
#include <kApi/Threads/kLock.h>

 // This definition needed only because of SDKNET wrapper definitions.
kDeclareValueEx(Go, GoAccelSensorPortAllocPorts, kValue)

// Default Gocator sensor port range within which ports will be chosen
// for the accelerated sensors to use. Start of range is chosen to match the
// default control port used by sensors.
// Ports chosen from this range are:
//  - control port
//  - upgrade port
//  - health port
//  - private data port
//  - public data port
//  - web port
// The upper end is the port just before the IANA recommended dynamic/private
// port range for ephemeral ports (49152 - 65535).
//
// **** NOTE: If the min-max range value is changed, the list of reserved ports
// checked by this class may need to be updated to include all reserved
// lying within the new port range.
#define GO_ACCEL_SENSOR_PORT_ALLOC_PORT_RANGE_DEFAULT_MIN   GO_SDK_RESERVED_PORT_SENSOR_CONTROL
#define GO_ACCEL_SENSOR_PORT_ALLOC_PORT_RANGE_DEFAULT_MAX   49151

// This specifies the minimum number of ports in the port range. It is used
// as follows:
//     port range max value - port range min value + 1 >= min number of ports
//
// The ideal minimum is 6, which is enough ports for:
//  - control port
//  - upgrade port
//  - health port
//  - private data port
//  - public data port
//  - web port
// But to support the legacy port values which are NOT sequential, the port
// range has to be 3190 to 3196 (a range of 7 ports) plus one web port for a
// total of 8 ports. Make the web port outside the range of 3190-3196 in case
// future sensor channels want to use the 2 available ports within 3190-3196.
//
// Additionally, the default web port is 8080 so if 8080 is not in the port
// range, then the port range must have enough space for choosing a web port.
// Optionally, two minimum limits can be defined:
//  - one for legacy port numbers (8 port numbers in range)
//  - one for non-legacy port numbers (6 port numbers in range)
#define GO_ACCEL_SENSOR_PORT_ALLOC_MIN_PORT_NUMBERS         8

// Structure to represent the port range from which ports are automatically
// selected for an accelerated sensor to use, if automatic selection is
// requested by the SDK client.
// The structure also keeps track of which port from which to start dynamic
// port allocation.
// Fields:
//  minPort:    Starting port number in the range.
//  maxPort:    Ending port number in the range.
//  startPort:  The starting port to use for new port allocation.
typedef struct GoAccelSensorPortAllocPortRange
{
    k16u minPort;
    k16u maxPort;
    k16u startPort;
} GoAccelSensorPortAllocPortRange;

// Structure to manage the set of ports allocated to accelerated sensors.
// Note that allocated ports are not necessarily immediately in use. There
// is a finite time delay between port allocation and port use by accelerated
// sensor software.
// Fields:
//  lock:           Mutex to ensure thread safety.
//  allocatedPorts: Set of ports allocated to all accelerated sensors.
typedef struct GoAccelSensorPortAllocPortBlock
{
    kLock       lock;
    kMap        ports;
} GoAccelSensorPortAllocPortBlock;

typedef struct GoAccelSensorPortAllocClass
{
    kObjectClass    base;

    GoAccelSensorPortAllocPortRange   portRange;

    GoAccelSensorPortAllocPortBlock allocatedPorts;
} GoAccelSensorPortAllocClass;

kDeclareClassEx(Go, GoAccelSensorPortAlloc, kObject)

GoFx(kStatus) GoAccelSensorPortAlloc_Init(GoAccelSensorPortAlloc portAlloc, kType type, kAlloc allocator);
GoFx(kStatus) GoAccelSensorPortAlloc_VRelease(GoAccelSensorPortAlloc portAlloc);

GoFx(kStatus) GoAccelSensorPortAlloc_InitAllocatedPortBlock(GoAccelSensorPortAlloc portAlloc, kAlloc alloc);
GoFx(kStatus) GoAccelSensorPortAlloc_ReleaseAllocatedPortBlock(GoAccelSensorPortAlloc portAlloc);

GoFx(k16u) GoAccelSensorPortAlloc_GetStartPort(GoAccelSensorPortAlloc portAlloc);
GoFx(void) GoAccelSensorPortAlloc_SetStartPort(GoAccelSensorPortAlloc portAlloc, k16u port);
GoFx(kStatus) GoAccelSensorPortAlloc_SetNextStartPort(GoAccelSensorPortAlloc portAlloc, k16u lastUsedPort);

GoFx(kBool) GoAccelSensorPortAlloc_PortIsAllocated(GoAccelSensorPortAlloc portAlloc, k16u port);
GoFx(kBool) GoAccelSensorPortAlloc_PortInRange(GoAccelSensorPortAlloc portAlloc, k16u portValue);
GoFx(kBool) GoAccelSensorPortAlloc_SocketIsFree(GoAccelSensorPortAlloc portAlloc, kIpAddress interfaceIp, kSocketType socketType, k16u port);
GoFx(kBool) GoAccelSensorPortAlloc_PortIsFree(GoAccelSensorPortAlloc portAlloc, k16u port, kBool doAllocationCheck);
GoFx(kBool) GoAccelSensorPortAlloc_ReservedPort(GoAccelSensorPortAlloc portAlloc, k16u port);
GoFx(kStatus) GoAccelSensorPortAlloc_NextPort(GoAccelSensorPortAlloc portAlloc, k16u* port);

GoFx(kStatus) GoAccelSensorPortAlloc_SelectLegacyPorts(GoAccelSensorPortAlloc portAlloc, GoAccelSensorPortAllocPorts* sensorPorts);
GoFx(kStatus) GoAccelSensorPortAlloc_FindOneFreePort(GoAccelSensorPortAlloc portAlloc, k16u* port);
GoFx(kStatus) GoAccelSensorPortAlloc_FindOneAutoSelectPort(GoAccelSensorPortAlloc portAlloc, kBool advancePort, k16u* nextPort, k16u* targetPort);
GoFx(kStatus) GoAccelSensorPortAlloc_SelectAutoSelectPorts(GoAccelSensorPortAlloc portAlloc, k16u nextPort, GoAccelSensorPortAllocPorts* sensorPorts);

GoFx(kBool) GoAccelSensorPortAlloc_IsAutoPortSelect(GoAccelSensorPortAlloc portAlloc, GoAccelSensorPortAllocPorts* sensorPorts);
GoFx(kBool) GoAccelSensorPortAlloc_ValidatePortInRange(GoAccelSensorPortAlloc portAlloc, GoAccelSensorPortAllocPorts* sensorPorts);
GoFx(kStatus) GoAccelSensorPortAlloc_SelectPort(GoAccelSensorPortAlloc portAlloc, k32u sensorId, GoAccelSensorPortAllocPorts* sensorPorts);

GoFx(kStatus) GoAccelSensorPortAlloc_StoreAllocatedPorts(GoAccelSensorPortAlloc portAlloc, k32u sensorId, GoAccelSensorPortAllocPorts* sensorPorts);

#endif  // GO_ACCEL_SENSOR_PORT_ALLOC_X_H

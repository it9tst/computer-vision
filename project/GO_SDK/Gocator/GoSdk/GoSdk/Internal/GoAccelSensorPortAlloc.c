/**
 * @file    GoAccelSensorPortAlloc.c
 *
 * This calls implements the sensor port allocator functionality for
 * accelerated sensors.
 * The ports allocated for an accelerated sensors are:
 *  - control channel port
 *  - upgrade channel port
 *  - health channel port
 *  - private data channel port
 *  - public data channel port
 *  - web/http server port
 *
 * SUPPORT FOR LEGACY BEHAVIOUR
 * Legacy accelerator application uses the default sensor ports for the
 * accelerated sensor (eg. 3190 for control port etc).
 * This port allocator class will allocate the same ports to accelerated sensors
 * if:
 *  - the port range starts from the default control port of 3190
 *  - only one sensor is ever accelerated in the system.
 *    - note that accelerating two sensors and then decelerating one of the
 *      sensors is NOT the same as accelerating only one sensor in the system,
 *      but instad is treated as accelerating more than one sensor.
 *
 * SUPPORT FOR MORE THAN ONE ACCELERATED SENSOR
 * A. If more than one sensor is accelerated, and the port range starting number
 * is the default control port (3190), then the second and subsequent sensors
 * are assigned in consecutive order after the set of legacy port values, to
 * maximize the use of the port range.
 * If the allocator wraps around to the legacy port values, it can pick an
 * an available port from within the legacy port range.
 *
 * B. If more than one sensor is accelerated, and the port range starting number
 * is NOT the default control port (3190), then the legacy port numbers are not
 * allocated to the first sensor. The allocated ports are chosen in consecutive
 * order to maximize the use of the port range.
 *
 * @internal
 * Copyright (C) 2016 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <GoSdk/Internal/GoAccelSensorPortAlloc.h>
#include <kApi/Io/kSocket.h>
//#include <kApi/Utils/kUtils.h>
//#include <kApi/Data/kImage.h>
//#include <kApi/Io/kDat6Serializer.h>

// This definition needed only because of SDKNET wrapper definitions.
kBeginValueEx(Go, GoAccelSensorPortAllocPorts)
kEndValueEx()

kBeginClassEx(Go, GoAccelSensorPortAlloc)
    kAddVMethod(GoAccelSensorPortAlloc, kObject, VRelease)
kEndClassEx()

GoFx(kStatus) GoAccelSensorPortAlloc_Construct(GoAccelSensorPortAlloc* portAlloc, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback(allocator);
    kStatus status;

    kCheck(kAlloc_GetObject(alloc, kTypeOf(GoAccelSensorPortAlloc), portAlloc));

    if (!kSuccess(status = GoAccelSensorPortAlloc_Init(*portAlloc, kTypeOf(GoAccelSensorPortAlloc), alloc)))
    {
        kAlloc_FreeRef(alloc, portAlloc);
    }

    return status;
}

GoFx(kStatus) GoAccelSensorPortAlloc_Init(GoAccelSensorPortAlloc portAlloc, kType type, kAlloc alloc)
{
    kObjR(GoAccelSensorPortAlloc, portAlloc);

    kCheck(kObject_Init(portAlloc, type, alloc));
    kZero(obj->portRange);
    kZero(obj->allocatedPorts);

    // Make sure the default port range limits has been coded correctly.
    kAssert((GO_ACCEL_SENSOR_PORT_ALLOC_PORT_RANGE_DEFAULT_MAX - GO_ACCEL_SENSOR_PORT_ALLOC_PORT_RANGE_DEFAULT_MIN + 1) >= GO_ACCEL_SENSOR_PORT_ALLOC_MIN_PORT_NUMBERS);

    obj->portRange.minPort = GO_ACCEL_SENSOR_PORT_ALLOC_PORT_RANGE_DEFAULT_MIN;
    obj->portRange.maxPort = GO_ACCEL_SENSOR_PORT_ALLOC_PORT_RANGE_DEFAULT_MAX;
    obj->portRange.startPort = obj->portRange.minPort;

    kCheck(GoAccelSensorPortAlloc_InitAllocatedPortBlock(portAlloc, alloc));

    return kOK;
}

GoFx(kStatus) GoAccelSensorPortAlloc_VRelease(GoAccelSensorPortAlloc portAlloc)
{
    kObj(GoAccelSensorPortAlloc, portAlloc);

    kCheck(GoAccelSensorPortAlloc_ReleaseAllocatedPortBlock(portAlloc));

    kCheck(kObject_VRelease(portAlloc));

    return kOK;
}

GoFx(kStatus) GoAccelSensorPortAlloc_InitAllocatedPortBlock(GoAccelSensorPortAlloc portAlloc, kAlloc alloc)
{
    kObj(GoAccelSensorPortAlloc, portAlloc);

    kCheck(kLock_Construct(&obj->allocatedPorts.lock, alloc));
    kCheck(kMap_Construct(&obj->allocatedPorts.ports, kTypeOf(k16u), kTypeOf(k16u), 0, alloc));

    return kOK;
}

GoFx(kStatus) GoAccelSensorPortAlloc_ReleaseAllocatedPortBlock(GoAccelSensorPortAlloc portAlloc)
{
    kObj(GoAccelSensorPortAlloc, portAlloc);

    kCheck(kDestroyRef(&obj->allocatedPorts.ports));
    kCheck(kDestroyRef(&obj->allocatedPorts.lock));

    return kOK;
}

GoFx(kBool) GoAccelSensorPortAlloc_IsAutoPortSelect(GoAccelSensorPortAlloc portAlloc, GoAccelSensorPortAllocPorts* sensorPorts)
{
    kObj(GoAccelSensorPortAlloc, portAlloc);
    kBool autoSelect = kTRUE;
    k16u tempSum;

    // All the port numbers should be set to zero (kIP_PORT_ANY) for auto port selection.
    tempSum =
        sensorPorts->controlPort | sensorPorts->healthPort | sensorPorts->upgradePort |
        sensorPorts->privateDataPort | sensorPorts->publicDataPort | sensorPorts->webPort;

    if (tempSum != 0)
    {
        autoSelect = kFALSE;
    }

    return autoSelect;
}

GoFx(kBool) GoAccelSensorPortAlloc_PortInRange(GoAccelSensorPortAlloc portAlloc, k16u portValue)
{
    kObj(GoAccelSensorPortAlloc, portAlloc);
    kBool isValid = kFALSE;

    if ((portValue >= obj->portRange.minPort) &&
        (portValue <= obj->portRange.maxPort))
    {
        isValid = kTRUE;
    }

    return isValid;
}

GoFx(kBool) GoAccelSensorPortAlloc_ValidatePortInRange(GoAccelSensorPortAlloc portAlloc, GoAccelSensorPortAllocPorts* sensorPorts)
{
    kObj(GoAccelSensorPortAlloc, portAlloc);
    kBool isValid = kFALSE;

    if (GoAccelSensorPortAlloc_PortInRange(portAlloc, sensorPorts->controlPort) &&
        GoAccelSensorPortAlloc_PortInRange(portAlloc, sensorPorts->upgradePort) &&
        GoAccelSensorPortAlloc_PortInRange(portAlloc, sensorPorts->healthPort) &&
        GoAccelSensorPortAlloc_PortInRange(portAlloc, sensorPorts->privateDataPort) &&
        GoAccelSensorPortAlloc_PortInRange(portAlloc, sensorPorts->publicDataPort) &&
        GoAccelSensorPortAlloc_PortInRange(portAlloc, sensorPorts->webPort))
    {
        isValid = kTRUE;
    }

    return isValid;
}

// This function makes getting the starting port number look easier.
GoFx(k16u) GoAccelSensorPortAlloc_GetStartPort(GoAccelSensorPortAlloc portAlloc)
{
    kObj(GoAccelSensorPortAlloc, portAlloc);

    return obj->portRange.startPort;
}

// This function makes setting the starting port number look easier.
GoFx(void) GoAccelSensorPortAlloc_SetStartPort(GoAccelSensorPortAlloc portAlloc, k16u port)
{
    kObj(GoAccelSensorPortAlloc, portAlloc);

    obj->portRange.startPort = port;
}

// This function takes the port number input and replaces it with the next port
// number. The function has to take into account:
//   - wrap around within the configured port range.
//   - detect if already gone through the entire range.
// The function does not take into account whether the port is in use or not.
// This function returns an error if the next available port is the same as
// the starting port, indicating the port range has been exhausted.
GoFx(kStatus) GoAccelSensorPortAlloc_NextPort(GoAccelSensorPortAlloc portAlloc, k16u* port)
{
    kObj(GoAccelSensorPortAlloc, portAlloc);
    k16u nextPort;

    nextPort = *port;

    // Code should also handle last port number that is outside the port range.
    if (nextPort >= obj->portRange.maxPort)
    {
        // Wrap around scenario.
        nextPort = obj->portRange.minPort;
    }
    else
    {
        nextPort++;
    }

    // Check if gone through the entire port range already (ie. run out of
    // ports to select). Bail out because no more ports.
    // The starting port number remains unchanged so no need to update it.
    kCheckTrue(nextPort != GoAccelSensorPortAlloc_GetStartPort(portAlloc), kERROR_FULL);

    *port = nextPort;

    return kOK;
}

// This function calculates the next starting port number, given the last
// port that was used/allocated.
GoFx(kStatus) GoAccelSensorPortAlloc_SetNextStartPort(GoAccelSensorPortAlloc portAlloc, k16u lastUsedPort)
{
    kObj(GoAccelSensorPortAlloc, portAlloc);
    kStatus exception;
    k16u nextPort;

    nextPort = lastUsedPort;

    kTry
    {
        // This call can fail if the next port is the same as the
        // starting port.
        // This could happen if there are only enough ports in the port range
        // for one sensor, and all the ports were just allocated to  a
        // sensor. So the next starting port becomes the same as
        // the original starting port.
        // In that case, the starting port number remains unchanged.
        kTest(GoAccelSensorPortAlloc_NextPort(portAlloc, &nextPort));

        GoAccelSensorPortAlloc_SetStartPort(portAlloc, nextPort);
    }
    kCatch(&exception)
    {
        // The next port is the same as the starting port number.
        // It is an error if trying to find a free port, but is not
        // an error if enough free ports were found for a sensor.
        kLogf("%s: status %d - next port in port range (%u-%u) is same as current "
            "start point, last used port %u.",
            __FUNCTION__,
            exception,
            obj->portRange.minPort,
            obj->portRange.maxPort,
            lastUsedPort);

        kEndCatch(exception);
    }

    return kOK;
}

// This function attempts to create a socket with the specified port to determine
// if the port is being used or not.
GoFx(kBool) GoAccelSensorPortAlloc_SocketIsFree(GoAccelSensorPortAlloc portAlloc, kIpAddress interfaceIp, kSocketType socketType, k16u port)
{
    kObj(GoAccelSensorPortAlloc, portAlloc);
    kSocket socket = kNULL;
    kBool isFree = kFALSE;

    // Create a socket and bind it to the specified port.
    // Afterwards, release the socket (and therefore port), by closing the
    // socket.
    if (kSuccess(kSocket_Construct(&socket, kIP_VERSION_4, socketType, kObject_Alloc(portAlloc))))
    {
        if (kSuccess(kSocket_Bind(socket, interfaceIp, (k32u) port)))
        {
            isFree = kTRUE;
        }
    }

    kDestroyRef(&socket);

    return isFree;
}


// This function checks if the specified port is already allocated, or if
// it is currently in use (ie. currently bound to a socket).
// If neither, then the port is available for allocation, and hopefully remains free
// until the port is bound by the acceleration code.
// NOTE: this function assumes the port number are within the valid port
// range, and so does not explicitly check for that.
GoFx(kBool) GoAccelSensorPortAlloc_PortIsFree(GoAccelSensorPortAlloc portAlloc, k16u port, kBool doAllocationCheck)
{
    kObj(GoAccelSensorPortAlloc, portAlloc);
    kBool isFree = kFALSE;

    if (!GoAccelSensorPortAlloc_ReservedPort(portAlloc, port))
    {
        // Skip socket check if allocation check is enabled and the port is
        // found to be allocated already => port is not free.
        if (!(doAllocationCheck && GoAccelSensorPortAlloc_PortIsAllocated(portAlloc, port)))
        {
            // Test if port is free by attempting to create a socket using
            // that port.
            // Only TCP and UDP sockets are checked because these are the most
            // commonly used sockets by Gocator software and other external
            // applications on an accelerator platform.
            // IP address used for the socket is 0.0.0.0 as that is the most common
            // address used by Gocator software.
            if (GoAccelSensorPortAlloc_SocketIsFree(portAlloc, kIpAddress_AnyV4(), kSOCKET_TYPE_UDP, port) &&
                GoAccelSensorPortAlloc_SocketIsFree(portAlloc, kIpAddress_AnyV4(), kSOCKET_TYPE_TCP, port))
            {
                isFree = kTRUE;
            }
        }
    }

    return isFree;
}

// This function selects ports compatible with the legacy port values used
// when accelerating one sensor.
// Because the legacy web/http server port is so much larger than the sensor
// channel ports, the logic here gets a bit complicated when the configured
// port range and selecting the next starting port is taken into account.
GoFx(kStatus) GoAccelSensorPortAlloc_SelectLegacyPorts(GoAccelSensorPortAlloc portAlloc, GoAccelSensorPortAllocPorts* sensorPorts)
{
    kObj(GoAccelSensorPortAlloc, portAlloc);
    k16u webPort;
    k16u lastPort;  // Last free port chosen.

    // Choose legacy values for sensor channel ports (3190, 3192, 3194, 3195, 3196).
    // For legacy web port, use 8080 if possible.
    kCheckTrue(GoAccelSensorPortAlloc_PortIsFree(portAlloc, GO_SDK_RESERVED_PORT_SENSOR_CONTROL, kTRUE), kERROR_NETWORK);
    kCheckTrue(GoAccelSensorPortAlloc_PortIsFree(portAlloc, GO_SDK_RESERVED_PORT_SENSOR_UPGRADE, kTRUE), kERROR_NETWORK);
    kCheckTrue(GoAccelSensorPortAlloc_PortIsFree(portAlloc, GO_SDK_RESERVED_PORT_SENSOR_HEALTH, kTRUE), kERROR_NETWORK);
    kCheckTrue(GoAccelSensorPortAlloc_PortIsFree(portAlloc, GO_SDK_RESERVED_PORT_SENSOR_PRIVATE_DATA, kTRUE), kERROR_NETWORK);
    kCheckTrue(GoAccelSensorPortAlloc_PortIsFree(portAlloc, GO_SDK_RESERVED_PORT_SENSOR_PUBLIC_DATA, kTRUE), kERROR_NETWORK);

    // If port range does not include the legacy accelerated sensor HTTP server
    // (which is far away from the last sensor channel port value),
    // then choose a free port from within the legacy
    // port range. In this case, choose the value after the public data port
    // in case port numbers lower that public data port is needed for future
    // sensor channels.
    // The minimum port numbers in a port range already allows for this scenario.
    if (GoAccelSensorPortAlloc_PortInRange(portAlloc, GO_SDK_RESERVED_PORT_DEFAULT_ACCELERATED_SENSOR_HTTP_SERVER))
    {
        webPort = GO_SDK_RESERVED_PORT_DEFAULT_ACCELERATED_SENSOR_HTTP_SERVER;

        // The default port is so big using it as the last port would result in
        // a large gap of unused ports. So set the last port to the public data
        // port to avoid large gaps of ports.
        lastPort = GO_SDK_RESERVED_PORT_SENSOR_PUBLIC_DATA;
    }
    else
    {
        // Use a port number outside the range of legacy sensor channel ports.
        webPort = GO_SDK_RESERVED_PORT_SENSOR_PUBLIC_DATA + 1;

        // The web port was set to the largest port number in the legacy port
        // range, so use this as the last port.
        lastPort = webPort;
    }
    kCheckTrue(GoAccelSensorPortAlloc_PortIsFree(portAlloc, webPort, kTRUE), kERROR_NETWORK);

    // Ports are free so update worker message param with the port values.
    sensorPorts->controlPort        = GO_SDK_RESERVED_PORT_SENSOR_CONTROL;
    sensorPorts->upgradePort        = GO_SDK_RESERVED_PORT_SENSOR_UPGRADE;
    sensorPorts->healthPort         = GO_SDK_RESERVED_PORT_SENSOR_HEALTH;
    sensorPorts->privateDataPort    = GO_SDK_RESERVED_PORT_SENSOR_PRIVATE_DATA;
    sensorPorts->publicDataPort     = GO_SDK_RESERVED_PORT_SENSOR_PUBLIC_DATA;
    sensorPorts->webPort            = webPort;

    // Because all the sensor ports were allocated, adjust the start port to
    // next available port.
    // Can ignore errors from this call because the error just means the starting
    // port number has wrapped around. Since the legacy ports are
    // available, this error should not halt the acceleration processing.
    (void) GoAccelSensorPortAlloc_SetNextStartPort(portAlloc, lastPort);

    return kOK;
}

// This function checks if the specified port is a port reserved for use by
// Gocator software. It does not check if the port is used by Firesync software.
// Note that the port might not be in use yet.
//
// TODO:
//   - the list of ports can be read from a file, instead of being hard coded.
GoFx(kBool) GoAccelSensorPortAlloc_ReservedPort(GoAccelSensorPortAlloc portAlloc, k16u port)
{
    kObj(GoAccelSensorPortAlloc, portAlloc);
    kBool reserved = kFALSE;

    // Port number is within the configured port range. Check against
    // specific ports used by Gocator software.
    switch (port)
    {
    case GO_SDK_RESERVED_PORT_DISCOVERY_PROTOCOL:
    case GO_SDK_RESERVED_PORT_REMOTE_PROCEDURE_CALL:
    case GO_SDK_RESERVED_PORT_DEFAULT_ETHERNET_ASCII_SERVER:
    case GO_SDK_RESERVED_PORT_DEFAULT_ETHERNET_IP_EXPLICIT_SERVER:
        reserved = kTRUE;
        break;
    // Default is kFALSE;
    }

    return reserved;
}

GoFx(kStatus) GoAccelSensorPortAlloc_FindOneFreePort(GoAccelSensorPortAlloc portAlloc, k16u* port)
{
    kObj(GoAccelSensorPortAlloc, portAlloc);
    k16u nextPort = *port;

    // REMOVE
    kLogf("%s: check if port %u is free", __FUNCTION__, nextPort);

    // This loop runs until the port number wraps around back to the
    // starting port number, or a free port is found.
    while (!GoAccelSensorPortAlloc_PortIsFree(portAlloc, nextPort, kTRUE))
    {
        // This condition exits the loop if there are insufficient ports.
        kCheck(GoAccelSensorPortAlloc_NextPort(portAlloc, &nextPort));
    }

    *port = nextPort;

    // REMOVE
    kLogf("%s: found free port %u", __FUNCTION__, nextPort);

    return kOK;
}

GoFx(kStatus) GoAccelSensorPortAlloc_FindOneAutoSelectPort(GoAccelSensorPortAlloc portAlloc, kBool incrementPort, k16u* nextPort, k16u* targetPort)
{
    kObj(GoAccelSensorPortAlloc, portAlloc);
    kStatus exception;

    // REMOVE
    kLogf("%s: increment %u, nextPort %u", __FUNCTION__, incrementPort, *nextPort);

    kTry
    {
        if (incrementPort)
        {
            kTest(GoAccelSensorPortAlloc_NextPort(portAlloc, nextPort));
        }

        kTest(GoAccelSensorPortAlloc_FindOneFreePort(portAlloc, nextPort));

        *targetPort = *nextPort;
    }
    kCatch(&exception)
    {
        kLogf("%s: error %d finding a free auto select port starting from port %u",
            __FUNCTION__,
            exception,
            *nextPort);

        kEndCatch(exception);
    }

    return kOK;
}

// This function selects ports by automatically finding available ports.
GoFx(kStatus) GoAccelSensorPortAlloc_SelectAutoSelectPorts(GoAccelSensorPortAlloc portAlloc, k16u nextPort, GoAccelSensorPortAllocPorts* sensorPorts)
{
    kObj(GoAccelSensorPortAlloc, portAlloc);
    kStatus exception;
    k16u controlPort = 0;
    k16u upgradePort = 0;
    k16u healthPort = 0;
    k16u privateDataPort = 0;
    k16u publicDataPort = 0;
    k16u webPort = 0;

    kTry
    {
        kTest(GoAccelSensorPortAlloc_FindOneAutoSelectPort(portAlloc, kFALSE, &nextPort, &controlPort));
        kTest(GoAccelSensorPortAlloc_FindOneAutoSelectPort(portAlloc, kTRUE, &nextPort, &upgradePort));
        kTest(GoAccelSensorPortAlloc_FindOneAutoSelectPort(portAlloc, kTRUE, &nextPort, &healthPort));
        kTest(GoAccelSensorPortAlloc_FindOneAutoSelectPort(portAlloc, kTRUE, &nextPort, &privateDataPort));
        kTest(GoAccelSensorPortAlloc_FindOneAutoSelectPort(portAlloc, kTRUE, &nextPort, &publicDataPort));
        kTest(GoAccelSensorPortAlloc_FindOneAutoSelectPort(portAlloc, kTRUE, &nextPort, &webPort));

        sensorPorts->controlPort        = controlPort;
        sensorPorts->upgradePort        = upgradePort;
        sensorPorts->healthPort         = healthPort;
        sensorPorts->privateDataPort    = privateDataPort;
        sensorPorts->publicDataPort     = publicDataPort;
        sensorPorts->webPort            = webPort;

        // Got all the ports needed to accelerate the sensor so
        // advance the start port to the next port number.
        // If next starting port is equal to current starting port, it is not
        // an error because enough ports were found to accelerate the requested
        // sensor. So ignore the return code.
        (void) GoAccelSensorPortAlloc_SetNextStartPort(portAlloc, nextPort);
    }
    kCatch(&exception)
    {
        kLogf(
            "%s: error %d auto selecting ports: ctl %u, upg %u, "
            "health %u, private data %u, public data %u, web %u",
            __FUNCTION__,
            exception,
            controlPort,
            upgradePort,
            healthPort,
            privateDataPort,
            publicDataPort,
            webPort);

        kEndCatch(exception);
    }

    return kOK;
}

// This function tries to find free ports that can be used by the accelerated
// virtual sensor to use.
// NOTES:
// 1. For legacy behaviour, if the starting port is 3190, then try to select
//    these ports for sensor use:
//      - 3190 - control port
//      - 3192 - upgrade port
//      - 3194 - health port
//      - 3195 - private data port
//      - 3196 - public data port
//      - 8080 or 3197 (depending on port range) - web port.
// 2. If starting port number is not 3190, then the ports will be picked in
//    sequential order (eg. 4000 - 4005) as long as the port is free. If a
//    port is not free, try the next sequential port number.
// 3. For each sensor, the starting port number is always the previous starting
//    port number plus 1:
// 4. Known ports used by Gocator software are treated as reserved ports
//    and are not available for allocation.
GoFx(kStatus) GoAccelSensorPortAlloc_SelectPort(GoAccelSensorPortAlloc portAlloc, k32u sensorId, GoAccelSensorPortAllocPorts* sensorPorts)
{
    kObj(GoAccelSensorPortAlloc, portAlloc);
    kBool   quit = kFALSE;
    kStatus exception;
    k16u    nextPort;

    nextPort = GoAccelSensorPortAlloc_GetStartPort(portAlloc);

    kLogf("%s: starting port number %u", __FUNCTION__, nextPort);

    kTry
    {
        do
        {
            if (nextPort == GO_SDK_RESERVED_PORT_SENSOR_CONTROL)
            {
                // To support legacy SDK applications that accelerate only
                // one sensor, give the first accelerated sensor ports starting
                // from the default control port, if the default control port
                // is within the configured port range.
                if (kSuccess(GoAccelSensorPortAlloc_SelectLegacyPorts(portAlloc, sensorPorts)))
                {
                    quit = kTRUE;
                }
                else
                {
                    kLogf(
                        "%s: legacy accelerator ports unavailable for sensor %u",
                        __FUNCTION__,
                        sensorId);

                    // The legacy ports are unavailable. This can happen if
                    // the starting port number has wrapped around because
                    // all ports in the range has been allocated.
                    // Try to find other ports by setting the next port number
                    // beyond the last legacy port number.
                    // If no more ports available in the port range, then that is
                    // an error.
                    nextPort = GO_SDK_RESERVED_PORT_SENSOR_CONTROL + GO_ACCEL_SENSOR_PORT_ALLOC_MIN_PORT_NUMBERS - 1;
                    kTest(GoAccelSensorPortAlloc_NextPort(portAlloc, &nextPort));
                }
            }
            else
            {
                kLogf("%s: selecting ports for sensor %u, next port %u",
                    __FUNCTION__,
                    sensorId,
                    nextPort);

                // Automatically select ports for sensor. This can fail if there
                // are insufficient ports in the entire port range.
                kTest(GoAccelSensorPortAlloc_SelectAutoSelectPorts(portAlloc, nextPort, sensorPorts));

                quit = kTRUE;
            }
        } while (!quit);
    }
    kCatch(&exception);
    {
        kLogf("%s: error %d allocating ports for sensor %u",
            __FUNCTION__,
            exception,
            sensorId);

        kEndCatch(exception);
    }

    return kOK;
}

GoFx(kStatus) GoAccelSensorPortAlloc_SetPortRange(GoAccelSensorPortAlloc portAlloc, k16u startPort, k16u endPort)
{
    kObj(GoAccelSensorPortAlloc, portAlloc);
    kStatus exception;

    kTry
    {
        kTestArgs(startPort < endPort);
        kTestArgs((endPort - startPort + 1) >= GO_ACCEL_SENSOR_PORT_ALLOC_MIN_PORT_NUMBERS);
        kTestArgs(startPort >= GO_ACCEL_SENSOR_PORT_ALLOC_PORT_RANGE_DEFAULT_MIN);
        kTestArgs(endPort <= GO_ACCEL_SENSOR_PORT_ALLOC_PORT_RANGE_DEFAULT_MAX);

        obj->portRange.minPort = startPort;
        obj->portRange.maxPort = endPort;

        // Reset the starting port number.
        GoAccelSensorPortAlloc_SetStartPort(portAlloc, startPort);
    }
    kCatch(&exception)
    {
        kLogf("%s: error %d in new port range %u-%u",
            __FUNCTION__,
            exception,
            startPort,
            endPort);

        kEndCatch(exception);
    }

    return kOK;
}

// This function returns the currently configured port range for use in assigning
// ports to accelerated sensors.
GoFx(kStatus) GoAccelSensorPortAlloc_GetPortRange(GoAccelSensorPortAlloc portAlloc, k16u* startPort, k16u* endPort)
{
    kObj(GoAccelSensorPortAlloc, portAlloc);

    if (kIsNull(startPort) || kIsNull(endPort))
    {
        kLogf("%s: error - received null arguments for ports", __FUNCTION__);

        return kERROR_PARAMETER;
    }

    *startPort = obj->portRange.minPort;
    *endPort   = obj->portRange.maxPort;

    return kOK;
}

// This function returns the hard limits on the min/max values for the port range,
// and the minimum number ports the range must support.
// These limits are NOT configurable.
GoFx(kStatus) GoAccelSensorPortAlloc_GetPortRangeLimits(GoAccelSensorPortAlloc portAlloc, k16u* startLimit, k16u* endLimit, k16u* minNumPorts)
{
    kObj(GoAccelSensorPortAlloc, portAlloc);

    if (kIsNull(startLimit) || kIsNull(endLimit) || kIsNull(minNumPorts))
    {
        kLogf("%s: error - received null arguments for port range limits", __FUNCTION__);

        return kERROR_PARAMETER;
    }

    *startLimit  = GO_ACCEL_SENSOR_PORT_ALLOC_PORT_RANGE_DEFAULT_MIN;
    *endLimit    = GO_ACCEL_SENSOR_PORT_ALLOC_PORT_RANGE_DEFAULT_MAX;
    *minNumPorts = GO_ACCEL_SENSOR_PORT_ALLOC_MIN_PORT_NUMBERS;

    return kOK;
}

// This function checks if the user provided/selected ports are free or not.
GoFx(kBool) GoAccelSensorPortAlloc_UserSelectedPortsFree(GoAccelSensorPortAlloc portAlloc, GoAccelSensorPortAllocPorts* sensorPorts, kBool doAllocationCheck)
{
    kObj(GoAccelSensorPortAlloc, portAlloc);
    kStatus exception;
    kBool   portsFree = kFALSE;
    k16u    portToCheck = 0;

    kTry
    {
        portToCheck = sensorPorts->controlPort;
        kTestTrue(GoAccelSensorPortAlloc_PortIsFree(portAlloc, portToCheck, doAllocationCheck), kERROR_NETWORK);
        portToCheck = sensorPorts->upgradePort;
        kTestTrue(GoAccelSensorPortAlloc_PortIsFree(portAlloc, portToCheck, doAllocationCheck), kERROR_NETWORK);
        portToCheck = sensorPorts->healthPort;
        kTestTrue(GoAccelSensorPortAlloc_PortIsFree(portAlloc, portToCheck, doAllocationCheck), kERROR_NETWORK);
        portToCheck = sensorPorts->privateDataPort;
        kTestTrue(GoAccelSensorPortAlloc_PortIsFree(portAlloc, portToCheck, doAllocationCheck), kERROR_NETWORK);
        portToCheck = sensorPorts->publicDataPort;
        kTestTrue(GoAccelSensorPortAlloc_PortIsFree(portAlloc, portToCheck, doAllocationCheck), kERROR_NETWORK);
        portToCheck = sensorPorts->webPort;
        kTestTrue(GoAccelSensorPortAlloc_PortIsFree(portAlloc, portToCheck, doAllocationCheck), kERROR_NETWORK);

        portsFree = kTRUE;
    }
    kCatch(&exception)
    {
        kLogf("%s: error %d port %u is in use",
            __FUNCTION__,
            exception,
            portToCheck);

        // Want to return a boolean instead of status, so force the catch block
        // to not exit.
        kEndCatch(kOK);
    }

    return portsFree;
}

// This function receives a list of potential ports for an accelerated
// sensor from the client and does one of the following:
//  - if caller has provided specific ports, then the ports are validated
//    to ensure they are available.
//  - if caller has not provided specific ports, then ports will be allocated
//    for the sensor from a set of available ports, if sufficient ports
//    are available.
// The set of ports allocated are then stored to ensure no other sensor
// can use that set.
// This is an external API, so must run within a lock to ensure thread safety.
GoFx(kStatus) GoAccelSensorPortAlloc_AllocatePorts(GoAccelSensorPortAlloc portAlloc, k32u sensorId, GoAccelSensorPortAllocPorts* sensorPorts)
{
    kObj(GoAccelSensorPortAlloc, portAlloc);
    kStatus exception;

    kCheck(kLock_Enter(obj->allocatedPorts.lock));

    kTry
    {
        // Check the port numbers provided by the client.
        if (GoAccelSensorPortAlloc_IsAutoPortSelect(portAlloc, sensorPorts))
        {
            kLogf("%s: client requested auto port selection", __FUNCTION__);

            // Check if ports can be allocated for sensor.
            kTest(GoAccelSensorPortAlloc_SelectPort(portAlloc, sensorId, sensorPorts));
        }
        else
        {
            kLogf("%s: checking client provided ports", __FUNCTION__);

            // Check if client provided ports are valid or not.
            kTestArgs(GoAccelSensorPortAlloc_ValidatePortInRange(portAlloc, sensorPorts));
            kTestArgs(GoAccelSensorPortAlloc_UserSelectedPortsFree(portAlloc, sensorPorts, kTRUE));
        }

        // Ports automatically selected or provided by the caller are free.
        // Reserve them in the port allocator.
        kTest(GoAccelSensorPortAlloc_StoreAllocatedPorts(portAlloc, sensorId, sensorPorts));
    }
    kCatchEx(&exception)
    {
        kLogf("%s: error %d allocating ports for sensor %u",
            __FUNCTION__,
            exception,
            sensorId);

        kEndCatchEx(exception);
    }
    kFinallyEx
    {
        kLock_Exit(obj->allocatedPorts.lock);

        kEndFinallyEx();
    }

    return kOK;
}

// This function is used to allocate ports recovered/restored from configuration.
// These ports should have originally passed all the checks in
// GoAccelSensorPortAlloc_AllocatePorts(), so when these ports are restored,
// only very basic checks are made on the assumption these were valid ports.
GoFx(kStatus) GoAccelSensorPortAlloc_AllocateRestoredPorts(GoAccelSensorPortAlloc portAlloc, k32u sensorId, GoAccelSensorPortAllocPorts* sensorPorts)
{
    kObj(GoAccelSensorPortAlloc, portAlloc);
    kStatus exception;

    kCheck(kLock_Enter(obj->allocatedPorts.lock));

    kTry
    {
        kLogf("%s: checking restored ports", __FUNCTION__);

        // This check should never fail for restored ports. If it does,
        // that means the configured ports were probably corrupted or modified
        // from their original valid values.
        kTestArgs(GoAccelSensorPortAlloc_ValidatePortInRange(portAlloc, sensorPorts));

        // Still need to reserve restored ports in the port allocator.
        kTest(GoAccelSensorPortAlloc_StoreAllocatedPorts(portAlloc, sensorId, sensorPorts));
    }
    kCatchEx(&exception)
    {
        kLogf("%s: error %d allocating restored ports for sensor %u",
            __FUNCTION__,
            exception,
            sensorId);

        kEndCatchEx(exception);
    }
    kFinallyEx
    {
        kLock_Exit(obj->allocatedPorts.lock);

        kEndFinallyEx();
    }

    return kOK;
}

// This function checks to see if a port has been allocated by this port allocator.
GoFx(kBool) GoAccelSensorPortAlloc_PortIsAllocated(GoAccelSensorPortAlloc portAlloc, k16u port)
{
    kObj(GoAccelSensorPortAlloc, portAlloc);
    k16u tempPort = port;
    kBool isAllocated;

    isAllocated = kMap_HasT(obj->allocatedPorts.ports, &tempPort);

    return isAllocated;
}

// This function removes the set of specified ports from the set of ports
// allocated by this port allocator.
// This is an external API, so must run within a lock to ensure thread safety.
GoFx(kStatus) GoAccelSensorPortAlloc_RemoveAllocatedPorts(GoAccelSensorPortAlloc portAlloc, k32u sensorId, GoAccelSensorPortAllocPorts* sensorPorts)
{
    kObj(GoAccelSensorPortAlloc, portAlloc);

    kLogf("%s: removing allocated ports for sensor %u: "
        "ctl %u, upg %u, health %u, private %u, public %u, web %u",
        __FUNCTION__,
        sensorId,
        sensorPorts->controlPort,
        sensorPorts->upgradePort,
        sensorPorts->healthPort,
        sensorPorts->privateDataPort,
        sensorPorts->publicDataPort,
        sensorPorts->webPort);

    kCheck(kLock_Enter(obj->allocatedPorts.lock));

    // Ignore errors to maximum number of ports cleared.
    kMap_Discard(obj->allocatedPorts.ports, &sensorPorts->controlPort);
    kMap_Discard(obj->allocatedPorts.ports, &sensorPorts->upgradePort);
    kMap_Discard(obj->allocatedPorts.ports, &sensorPorts->healthPort);
    kMap_Discard(obj->allocatedPorts.ports, &sensorPorts->privateDataPort);
    kMap_Discard(obj->allocatedPorts.ports, &sensorPorts->publicDataPort);
    kMap_Discard(obj->allocatedPorts.ports, &sensorPorts->webPort);

    if (kMap_Count(obj->allocatedPorts.ports) == 0)
    {
        kLogf("%s: setting start port to port range start: %u",
            __FUNCTION__,
            obj->portRange.minPort);

        // Reset the starting port number when no ports are allocated.
        GoAccelSensorPortAlloc_SetStartPort(portAlloc, obj->portRange.minPort);
    }

    kLock_Exit(obj->allocatedPorts.lock);

    return kOK;
}

// This function stores the specified ports for a sensor into the set of
// ports allocated by this port allocator.
GoFx(kStatus) GoAccelSensorPortAlloc_StoreAllocatedPorts(GoAccelSensorPortAlloc portAlloc, k32u sensorId, GoAccelSensorPortAllocPorts* sensorPorts)
{
    kObj(GoAccelSensorPortAlloc, portAlloc);
    kStatus exception;
    k16u    portAddCount = 0;  // Used to clean up added ports.

    kLogf("%s: sensor %u ports: "
        "ctl %u, upg %u, health %u, private %u, public %u, web %u",
        __FUNCTION__,
        sensorId,
        sensorPorts->controlPort,
        sensorPorts->upgradePort,
        sensorPorts->healthPort,
        sensorPorts->privateDataPort,
        sensorPorts->publicDataPort,
        sensorPorts->webPort);

    kTry
    {
        // If port is already allocated, print a log but continue trying to
        // add the other ports.
        kTest(kMap_AddT(obj->allocatedPorts.ports, &sensorPorts->controlPort, &sensorPorts->controlPort));
        portAddCount++;
        kTest(kMap_AddT(obj->allocatedPorts.ports, &sensorPorts->upgradePort, &sensorPorts->upgradePort));
        portAddCount++;
        kTest(kMap_AddT(obj->allocatedPorts.ports, &sensorPorts->healthPort, &sensorPorts->healthPort));
        portAddCount++;
        kTest(kMap_AddT(obj->allocatedPorts.ports, &sensorPorts->privateDataPort, &sensorPorts->privateDataPort));
        portAddCount++;
        kTest(kMap_AddT(obj->allocatedPorts.ports, &sensorPorts->publicDataPort, &sensorPorts->publicDataPort));
        portAddCount++;
        kTest(kMap_AddT(obj->allocatedPorts.ports, &sensorPorts->webPort, &sensorPorts->webPort));
    }
    kCatch(&exception)
    {
        // Should never allocate the same port more than once!
        kLogf("%s: error %d adding allocated ports, count %u, sensor %u: "
            "ctl %u, upg %u, health %u, private %u, public %u, web %u",
            __FUNCTION__,
            exception,
            portAddCount,
            sensorId,
            sensorPorts->controlPort,
            sensorPorts->upgradePort,
            sensorPorts->healthPort,
            sensorPorts->privateDataPort,
            sensorPorts->publicDataPort,
            sensorPorts->webPort);

        // Don't know which port failed, so try to remove all the input ports
        // that were added. Ignore error return status.
        if (portAddCount == 5)
        {
            kMap_Discard(obj->allocatedPorts.ports, &sensorPorts->publicDataPort);
            portAddCount--;
        }
        if (portAddCount == 4)
        {
            kMap_Discard(obj->allocatedPorts.ports, &sensorPorts->privateDataPort);
            portAddCount--;
        }
        if (portAddCount == 3)
        {
            kMap_Discard(obj->allocatedPorts.ports, &sensorPorts->healthPort);
            portAddCount--;
        }
        if (portAddCount == 2)
        {
            kMap_Discard(obj->allocatedPorts.ports, &sensorPorts->upgradePort);
            portAddCount--;
        }
        if (portAddCount == 1)
        {
            kMap_Discard(obj->allocatedPorts.ports, &sensorPorts->controlPort);
            portAddCount--;
        }

        kEndCatch(exception);
    }

    return kOK;
}

// This function removes all ports allocated by this port allocated.
// Used for unit testing.
// This is an external API, so must run within a lock to ensure thread safety.
GoFx(kStatus) GoAccelSensorPortAlloc_PurgeAllAllocatedPorts(GoAccelSensorPortAlloc portAlloc)
{
    kObj(GoAccelSensorPortAlloc, portAlloc);
    kStatus exception;

    kLogf("%s: purge all allocated ports", __FUNCTION__);

    kCheck(kLock_Enter(obj->allocatedPorts.lock));

    kTry
    {
        kTest(kMap_Purge(obj->allocatedPorts.ports));

        // Reset the starting port number when no ports are allocated.
        GoAccelSensorPortAlloc_SetStartPort(portAlloc, obj->portRange.minPort);
    }
    kCatchEx(&exception)
    {
        kLogf("%s: error %d purging allocated ports", __FUNCTION__, exception);

        kEndCatchEx(exception);
    }
    kFinallyEx
    {
        kLock_Exit(obj->allocatedPorts.lock);

        kEndFinallyEx();
    }

    return kOK;
}


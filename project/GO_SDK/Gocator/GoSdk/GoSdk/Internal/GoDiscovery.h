/**
 * @file    GoDiscovery.h
 * @brief   Declares discovery-related types.
 *
 * @internal
 * Copyright (C) 2016-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef GO_SDK_DISCOVERY_H
#define GO_SDK_DISCOVERY_H

#include <GoSdk/GoSdkDef.h>
#include <kApi/Data/kArrayList.h>
#include <GoSdk/Messages/GoDiscoveryExtInfo.h>

/**
 * @struct  GoDiscoveryInfo
 * @extends kValue
 * @ingroup GoSdk-Internal
 * @brief   Represents discovery information for a single device.
 */
typedef struct GoDiscoveryInfo
{
    k32u id;                    ///< device identifier (serial number)
    GoAddressInfo address;      ///< network configuration
    GoPortInfo ports;

    // Gocator firmware version the sensor is running.
    kVersion version;
    // opMode: indicates if the main controller (host) is:
    //  a. a virtual sensor, or
    //  b. a standalone sensor, or
    //  c. an accelerator host that is accelerating a sensor.
    GoDiscoveryOpMode opMode;
    // If opMode indicates main controller is an accelerator host,
    // then the IP address of the accelerated sensor
    // is stored in the following field.
    // Otherwise, this field is not meaningful.
    kIpAddress accelSensorIpAddress;

    // IP address of the discovery server that sent the discovery information
    // about the sensor. This can be the physical sensor or a host controller
    // (eg. virtual sensor, accelerated sensor).
    kIpAddress discoveryServerAddress;

    // Basic identifying sensor traits received via discovery information
    // about the sensor.
    kText32 partNumber;
    kText32 modelNumber;
    kText32 modelDisplayName;
    kText32 family;
} GoDiscoveryInfo;

/**
 * @class   GoDiscovery
 * @extends kObject
 * @ingroup GoSdk-Internal
 * @brief   Represents a discovery client.
 */
typedef kObject GoDiscovery;

/** Defines the signature for a discovery enumeration handler. */
typedef kStatus (kCall* GoDiscoveryEnumFx)(kPointer context, GoDiscovery discovery, kArrayList info);

/**
 * Constructs a GoDiscovery object.
 *
 * @public              @memberof GoDiscovery
 * @version             Introduced in firmware 4.0.10.27
 * @param   discovery   Receives constructed discovery object.
 * @param   enableAutoDiscovery whether or not to enable all interfaces to support running the Discovery Protocol.
 * @param   allocator   Memory allocator (or kNULL for default)
 * @return              Operation status.
 */
GoFx(kStatus) GoDiscovery_Construct(GoDiscovery* discovery, kBool enableAutoDiscovery, kAlloc allocator);

/**
 * Enumerates sensors present in the network.
 *
 * @public              @memberof GoDiscovery
 * @version             Introduced in firmware 4.0.10.27
 * @param   discovery   Discovery object.
 * @param   infoList    List to be populated with sensor descriptors (kArrayList<GoDiscoveryInfo>).
 * @return              Operation status.
 */
GoFx(kStatus) GoDiscovery_Enumerate(GoDiscovery discovery, kArrayList infoList);

/**
 * Configures a sensor's network address settings.
 *
 * This function uses UDP broadcasts; the sensor and can be on a different subnet than the client.
 *
 * The sensor will automatically reboot if the address is successfully changed.
 *
 * @public              @memberof GoDiscovery
 * @version             Introduced in firmware 4.0.10.27
 * @param   discovery   Discovery object.
 * @param   deviceId    Sensor device identifier (serial number).
 * @param   address     New address information.
 * @return              Operation status.
 */
GoFx(kStatus) GoDiscovery_SetAddress(GoDiscovery discovery, k32u deviceId, const GoAddressInfo* address);

/**
 * Retrieves a sensor's network address settings.
 *
 * This function uses UDP broadcasts; the sensor and can be on a different subnet than the client.
 *
 * @public              @memberof GoDiscovery
 * @version             Introduced in firmware 4.0.10.27
 * @param   discovery   Discovery object.
 * @param   deviceId    Sensor device identifier (serial number).
 * @param   address     Receives address information.
 * @return              Operation status.
 */
GoFx(kStatus) GoDiscovery_GetAddress(GoDiscovery discovery, k32u deviceId, GoAddressInfo* address);

/**
 * Retrieves a sensor's information.
 *
 * This function uses UDP broadcasts; the sensor and can be on a different subnet than the client.
 *
 * @public              @memberof GoDiscovery
 * @version             Introduced in firmware 4.3.3.124
 * @param   discovery   Discovery object.
 * @param   deviceId    Sensor device identifier (serial number).
 * @param   info        Receives sensor information.
 * @param   allocator   Memory allocator. In order to prevent memory leaks,
 *                      the received information object should be destroyed when no longer used.
 * @return              Operation status.
 */
GoFx(kStatus) GoDiscovery_GetExtendedInfo(GoDiscovery discovery, k32u deviceId, GoDiscoveryExtInfo* info, kAlloc allocator);

/**
 * Sets the enumeration period that will be used when background updates are enabled via StartEnum.
 *
 * @public              @memberof GoDiscovery
 * @version             Introduced in firmware 4.0.10.27
 * @param   discovery   Discovery object.
 * @param   period      Enumeration period, in microseconds.
 * @return              Operation status.
 */
GoFx(kStatus) GoDiscovery_SetEnumPeriod(GoDiscovery discovery, k64u period);

/**
 * Sets the enumeration callback to be used when background updates are enabled via StartEnum.
 *
 * @public              @memberof GoDiscovery
 * @version             Introduced in firmware 4.0.10.27
 * @param   discovery   Discovery object.
 * @param   function    Enumeration callback function (or kNULL to unregister).
 * @param   receiver    Receiver argument for callback.
 * @return              Operation status.
 */
GoFx(kStatus) GoDiscovery_SetEnumHandler(GoDiscovery discovery, GoDiscoveryEnumFx function, kPointer receiver);

/**
 * Starts periodic background discovery enumeration.
 *
 * @public              @memberof GoDiscovery
 * @version             Introduced in firmware 4.0.10.27
 * @param   discovery   Discovery object.
 * @param   waitFirst   kTRUE to block until first enumeration cycle is completed; kFALSE otherwise.
 * @return              Operation status.
 */
GoFx(kStatus) GoDiscovery_StartEnum(GoDiscovery discovery, kBool waitFirst);

/**
 * Stops periodic background discovery enumeration.
 *
 * @public              @memberof GoDiscovery
 * @version             Introduced in firmware 4.0.10.27
 * @param   discovery   Discovery object.
 * @return              Operation status.
 */
GoFx(kStatus) GoDiscovery_StopEnum(GoDiscovery discovery);

/**
* Enable or disable running the Gocator Discovery Protocol over the specified
* host interface which the given address.
*
* @public              @memberof GoDiscovery
* @version             Introduced in firmware 4.6.7.157
* @param   discovery   Discovery object.
* @param   address     IP address of interface over which the discovery protocol should run.
* @param   enable      Select whether to enable or disable the interface.
* @return              Operation status.
*/
GoFx(kStatus) GoDiscovery_SetOneInterface(GoDiscovery discovery, kIpAddress* address, kBool enable);

/**
* Enable running the Gocator Discovery Protocol over all the host interfaces.
*
* @public              @memberof GoDiscovery
* @version             Introduced in firmware 4.6.7.157
* @param   discovery   Discovery object.
* @param   enable      Select whether to enable or disable the interface.
* @return              Operation status.
*/
GoFx(kStatus) GoDiscovery_SetAllInterface(GoDiscovery discovery, kBool enable);

/**
* Enables or disables compatibility mode.
*
* Compatibility mode allows discovery of older firmware but increase
* broadcast traffic.
*
* @public              @memberof GoDiscovery
* @version             Introduced in firmware 5.2.18.3
* @param   discovery   Discovery object.
* @param   enable      Enable or disable compatibility mode.
* @return              Operation status.
*/
GoFx(kStatus) GoDiscovery_EnableCompatMode(GoDiscovery discovery, kBool enable);

/**
* Gets the current state of compatibility mode.
*
* @public              @memberof GoDiscovery
* @version             Introduced in firmware 5.2.18.3
* @param   discovery   Discovery object.
* @return              Compatibility enable flag.
*/
GoFx(kBool) GoDiscovery_CompatModeEnabled(GoDiscovery discovery);

#include <GoSdk/Internal/GoDiscovery.x.h>

#endif

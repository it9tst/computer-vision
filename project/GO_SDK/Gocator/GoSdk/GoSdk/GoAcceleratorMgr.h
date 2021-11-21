/**
 * @file    GoAcceleratorMgr.h
 * @brief   Declares the GoAcceleratorMgr class public interfaces.
 *
 * @internal
 * Copyright (C) 2018-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 *
 * The SDK Accelerator Manager object allows SDK clients to accelerate more than
 * one sensor within a single platform. SDK client can build its own accelerator
 * application using the SDK Accelerator Manager. The SDK Accelerator Manager
 * provides a mechanism for the SDK client to receive event updates from the
 * Accelerator Manager.
 *
 * A brief description of how to use the SDK Accelerator Manager follows.
 * Note the description omits error checking and parameters to focus on the
 * call sequences.
 *
 * A. Initializing the Accelerator Manager.
 * Here is sample code showing how to initialize the Accelerator Manager and how
 * to start it running.
 *
 *    kStatus kCall MyApp_UpdateHandler(MyAppClass MyApp, kPointer caller, GoAcceleratorMgrAccelUpdate* update)
 *    {
 *        printf(
 *            "Sensor %u event - %d\n",
 *            update->sensorId,
 *            update->accelEvent);
 *    }
 *
 *    // Construct the Accelerator Manager.
 *    GoAcceleratorMgr_Construct();
 *
 *    // Set up the update handler so the SDK client can receive events
 *    // from the Accelerator Manager. This step is optional.
 *    GoAcceleratorMgr_SetAccelUpdateHandler(MyApp_UpdateHandler, MyApp);
 *
 *    // Must give the Accelerator Manager the system object before starting it.
 *    GoSystem system = kNULL;
 *
 *    GoSystem_Construct(&system, ...);
 *    GoAcceleratorMgr_SetSystem(system);
 *
 *    // Now start the Accelerator Manager.
 *    GoAcceleratorMgr_Start();
 *
 *    // Set the range of port numbers any time as long as no sensor is configured
 *    // for acceleration. This is an optional step to change the default
 *    // range. See the section on changing the port range for more details.
 *    GoAcceleratorMgr_SetPortRange(startPort, endPort);
 *
 * B. Accelerating a Sensor.
 *
 *    // Set up the parameters in a GoAcceleratorMgrSensorParam structure and
 *    // call the accelerate interface.
 *    GoAcceleratorMgrSensorParam param;
 *    k32u sensorId = 12345;
 *
 *    if (useAutomaticAllocation)
 *    {
 *        param.ports.controlPort = 0;
 *        param.ports.upgradePort = 0;
 *        param.ports.healthPort = 0;
 *        param.ports.privateDataPort = 0;
 *        param.ports.publicDataPort = 0;
 *        param.ports.webPort = 0;
 *    }
 *    else
 *    {
 *        // Manually specify ports that the SDK client is allowed to set up
 *        // TCP/IP connections through a network to the accelerated sensor.
 *        // This example assumes ports 4000 and up are available for use.
 *        param.ports.controlPort = 4000;
 *        param.ports.upgradePort = 4001;
 *        param.ports.healthPort = 4002;
 *        param.ports.privateDataPort = 4003;
 *        param.ports.publicDataPort = 4004;
 *        param.ports.webPort = 4005;
 *    }
 *    // Assume okay to use any IPv4 address (ie. 0.0.0.0).
 *    // Otherwise choose one of the accelerator platform's network interface
 *    // IP address.
 *    param.platforIpAddress = kIpAddress_AnyV4();
 *
 *    GoAcceleratorMgr_Accelerate(sensorId, &param);
 *
 * C. Decelerating (Unaccelerating) a Sensor.
 *
 *    k32u sensorId = 12345;
 *
 *    GoAcceleratorMgr_Decelerate(sensorId);
 *
 * D. Get the List of Sensors Managed by the Accelerator Manager.
 *
 *    kArrayList sensorList = kNULL;
 *
 *    kArrayList_Construct(&sensorList, kTypeOf(GoAcceleratorMgrSensorInfo), 0, kObject_Alloc(myApp));
 *    GoAcceleratorMgr_ListSensors(sensorList);
 *
 * E. Get the port range limits and change the port range.
 *
 *    k16u startLimit;
 *    k16u endLimit;
 *    k16u minNumPorts;
 *    k16u userStartPort;
 *    k16u userEndPort;
 *
 *    // Some code initializes userStartPort and userEndPort.
 *    userStartPort = 4000;
 *    userEndPort   = 4010;
 *
 *    // Get the min and max values of the port range and the minimum number
 *    // of ports within the port range to help with user configuration
 *    // validation.
 *    GoAcceleratorMgr_GetPortRangeLimits(startLimit, endLimit, minNumPorts);
 *
 *    if ((userStartPort >= startLimit) && (userEndPort <= endLimit) &&
 *       ((userEndPort - userStartPort + 1) >= minNumPorts)
 *    {
 *        GoAcceleratorMgr_SetPortRange(userStartPort, userEndPort);
 *    }
 *    else
 *    {
 *        printf("Invalid user port range\n");
 *    }
 *
 * F. Get current port range.
 *
 *    k16u startPort;
 *    k16u endPort;
 *
 *    GoAcceleratorMgr_GetPortRange(&startPort, &endPort);
 *
 * G. Get the number of sensors managed by the Accelerator Manager.
 *
 *    if (GoAcceleratorMgr_AccelSensorCount() > 0)
 *    {
 *        printf("Sensors configured for acceleration\n");
 *    }
 *    else
 *    {
 *        printf("No sensors configured for acceleration\n");
 *    }
 *
 */
#ifndef GO_ACCELERATOR_MGR_H
#define GO_ACCELERATOR_MGR_H

#include <GoSdk/GoSdk.h>
// Don't know why need to explicitly include GoSystem header file again
// here when GoSdk.h already includes it. If this is not done, compile fails
// to find GoSystem declaration!?
#include <GoSdk/GoSystem.h>
#include <GoSdk/Internal/GoAccelSensorPortAlloc.h>

/**
 * @class   GoAcceleratorMgr
 * @extends kObject
 * @ingroup GoSdk
 * @brief   Represents an GoAcceleratorMgr instance.
 */
typedef kObject GoAcceleratorMgr;

/**
* @class   GoAcceleratorMgr
* @extends kValue
* @ingroup GoSdk
* @brief   Represents an GoAcceleratorMgr events passed into the acceleration
*          update callback handler bound in by SDK client.
* Meaning of the events are:
*  - GO_ACCELERATOR_MGR_EVENT_ACCELERATING      - Sensor acceleration is in progress.
*  - GO_ACCELERATOR_MGR_EVENT_ACCELERATED:      - Sensor is accelerated successfully.
*  - GO_ACCELERATOR_MGR_EVENT_DECELERATING:     - Sensor deceleration is in progress.
*  - GO_ACCELERATOR_MGR_EVENT_DECELERATED:      - Sensor is no longer accelerated.
*  - GO_ACCELERATOR_MGR_EVENT_STOPPED:          - Sensor acceleration stopped or failed to start.
*  - GO_ACCELERATOR_MGR_EVENT_DISCONNECTED:     - Accelerated sensor is disconnected from network.
*  - GO_ACCELERATOR_MGR_EVENT_PROCESS_STOPPED:  - Special case for the STOPPED event to indicate acceleration
*                                                 was in progress but terminated unexpectedly.
*                                                 This stop reason differs from other stop reasons,
*                                                 such as firmware mismatch etc.
*/
typedef enum GoAcceleratorMgrAccelEvents
{
    GO_ACCELERATOR_MGR_EVENT_ACCELERATING = 0,
    GO_ACCELERATOR_MGR_EVENT_ACCELERATED,
    GO_ACCELERATOR_MGR_EVENT_DECELERATING,
    GO_ACCELERATOR_MGR_EVENT_DECELERATED,
    GO_ACCELERATOR_MGR_EVENT_STOPPED,
    GO_ACCELERATOR_MGR_EVENT_DISCONNECTED,
    GO_ACCELERATOR_MGR_EVENT_PROCESS_STOPPED
} GoAcceleratorMgrAccelEvents;

/**
* @struct  GoAcceleratorMgrAccelUpdate
* @extends kValue
* @ingroup GoSdk
* @brief   Structure to hold data for the acceleration update handler.
*/
typedef struct GoAcceleratorMgrAccelUpdate
{
    k32u                        sensorId;
    GoAcceleratorMgrAccelEvents accelEvent;
} GoAcceleratorMgrAccelUpdate;

/**
* @struct  GoAcceleratorMgrSensorParam
* @extends kValue
* @ingroup GoSdk
* @brief   Structure to hold user configuration parameters from SDK client
*          for a sensor that is to be accelerated.
*/
typedef struct GoAcceleratorMgrSensorParam
{
    GoAccelSensorPortAllocPorts ports;
    kIpAddress                  platformIpAddress;  // Bind an accelerated sensor to this accelerator host interface IP address.
} GoAcceleratorMgrSensorParam;

/**
* @struct  GoAcceleratorMgrSensorBackup
* @extends kValue
* @ingroup GoSdk
* @brief   Structure to provide the original unaccelerated sensor information to the SDK client.
*          This is useful for reaching the sensor object while it is being accelerated, or
*          if there are errors during acceleration, or after stopping acceleration.
*/
typedef struct GoAcceleratorMgSensorBackup
{
    GoAccelSensorPortAllocPorts ports;
    GoAddressInfo               sensorAddressInfo;
} GoAcceleratorMgrSensorBackup;

/**
* @struct  GoAcceleratorMgrSensorInfo
* @extends kValue
* @ingroup GoSdk
* @brief   Structure to return accelerated sensor information to SDK client.
* The param field contains information received from the SDK client, except if
* SDK client requested automatic port selection. In this case, the ports in the
* param field are the ports selected by the SDK for the SDK client.
*/
typedef struct GoAcceleratorMgrSensorInfo
{
    k32u                         sensorId;
    GoSensorAccelStatus          status;
    GoAcceleratorMgrSensorParam  param;
    GoAcceleratorMgrSensorBackup backup;
} GoAcceleratorMgrSensorInfo;

/**
* Constructs the accelerator manager object
*
* @public                 @memberof GoAcceleratorMgr
* @version                Introduced in firmware 5.2.18.3
* @param   manager        Address of uninitialized manager object.
* @param   allocator      Allocator object (kNULL for fallback allocator).
* @return                 Operation status.
*/
GoFx(kStatus) GoAcceleratorMgr_Construct(GoAcceleratorMgr* manager, kAlloc allocator);

/**
* Assigns the SDK GoSystem object to the accelerator manager object. This must be
* done before starting the accelerator manager.
*
* @public                 @memberof GoAcceleratorMgr
* @version                Introduced in firmware 5.2.18.3
* @param   manager        GoAcceleratorMgr object.
* @param   system         Instance of GoSystem
* @return                 Operation status.
*/
GoFx(kStatus) GoAcceleratorMgr_SetSystem(GoAcceleratorMgr manager, GoSystem system);

/**
* Starts the accelerator manager object after it has been configured.
*
* @public                 @memberof GoAcceleratorMgr
* @version                Introduced in firmware 5.2.18.3
* @param   manager        GoAcceleratorMgr object.
* @return                 Operation status.
*/
GoFx(kStatus) GoAcceleratorMgr_Start(GoAcceleratorMgr manager);

/**
* Accelerate the specified sensor with the given set of parameters.
* The parameter structure contains the accelerator host IP address to
* associate with the accelerated sensor, and the ports the accelerated
* sensor should use to communicate with the SDK client.
* If the SDK client wishes to have the GoAcceleratorMgr dynamically/automatically
* allocate the port numbers from within the configured port range,
* the SDK client must set all the ports in the parameter structure to
* zero (0).
* Any non-zero value for a port in the port parameter list is treated
* as if the client is providing ALL the port numbers to use. If the
* client provided port numbers are valid, then they will be used by
* the accelerated sensor.
* The "param" structure is modified to store actual ports selected if
* SDK client chose dynamic/automatic port allocation.
*
* @public                 @memberof GoAcceleratorMgr
* @version                Introduced in firmware 5.2.18.3
* @param   manager        GoAcceleratorMgr object.
* @param   sensorId       Identifier of the sensor.
* @param   param          Pointer to parameter structure for accelerating the sensor.
* @return                 Operation status.
*/
GoFx(kStatus) GoAcceleratorMgr_Accelerate(GoAcceleratorMgr manager, k32u sensorId, GoAcceleratorMgrSensorParam* param);

/**
* Decelerate (unaccelerate) a sensor.
*
* @public                 @memberof GoAcceleratorMgr
* @version                Introduced in firmware 5.2.18.3
* @param   manager        GoAcceleratorMgr object.
* @param   sensorId       Identifier of the sensor.
* @return                 Operation status.
*/
GoFx(kStatus) GoAcceleratorMgr_Decelerate(GoAcceleratorMgr manager, k32u sensorId);

/**
* Get a list of accelerated sensors. The list is returned in an array list
* of GoAcceleratorMgrSensorInfo. Caller must construct the kArrayList and
* pass the it as an argument to this API. The API will fill this list.
*
* Each entry includes the set of ports used by the accelerated sensor.
* If the client had specified the ports to use, then the list entry's
* set of ports should be the same as the client specified ports.
* If the client had specified the Accelerator Manager dynamically
* allocate ports from the configured port range, then the list entry's
* set of ports contains the ports allocated to the sensor.
* The current acceleration status of the sensor is returned in the
* list entry for each sensor.
*
* @public                 @memberof GoAcceleratorMgr
* @version                Introduced in firmware 5.2.18.3
* @param   manager        GoAcceleratorMgr object.
* @param   sensorList     Array list of GoAcceleratorMgrSensorInfo for each
*                         accelerated sensor.
* @return                 Operation status.
*/
GoFx(kStatus) GoAcceleratorMgr_ListSensors(GoAcceleratorMgr manager, kArrayList sensorList);

/**
* Set up the update handler so that the SDK client can receive events
* from the Accelerator Manager about a sensor. This step is optional. If update handler
* is not bound in, then client will not receive any event notifications.
* Update handler is called with a pointer to GoAcceleratorMgrAccelUpdate which
* contains event information about a sensor.
* The update handler does not free the memory for the GoAcceleratorMgrAccelUpdate
* structure.
*
* @public                 @memberof GoAcceleratorMgr
* @version                Introduced in firmware 5.2.18.3
* @param   manager        GoAcceleratorMgr object.
* @param   function       Update callback handler function.
* @param   context        SDK client context for the callback handler.
*                         It is a pointer to GoAcceleratorMgrAccelUpdate.
* @return                 Operation status.
*/
GoFx(kStatus) GoAcceleratorMgr_SetAccelUpdateHandler(GoAcceleratorMgr manager, kCallbackFx function, kPointer context);

/**
* Set the range of port numbers used to communicate with the accelerated
* sensors. Changes to the port range is allowed  only if no sensor is
* configured for acceleration.
* Changing the port range is optional if the client application can use
* the default port range.
* The port numbers range must be within the port range limits (see
* GoAcceleratorMgr_GetPortRangeLimits()).
* Configure the port range if your network places limits on what ports are
* permitted for communication use or if some range of ports is known to be used
* by another application.
*
* @public                 @memberof GoAcceleratorMgr
* @version                Introduced in firmware 5.2.18.3
* @param   manager        GoAcceleratorMgr object.
* @param   startPort      Starting port number in the range
* @param   endPort        Last port number in the range.
* @return                 Operation status.
*/
GoFx(kStatus) GoAcceleratorMgr_SetPortRange(GoAcceleratorMgr manager, k16u startPort, k16u endPort);

/**
* Get the range of port numbers to assign to accelerated sensors.
* The ports within the port range are used by the Accelerator Manager
* to allocate ports for the accelerated sensors.
*
* @public                 @memberof GoAcceleratorMgr
* @version                Introduced in firmware 5.2.18.3
* @param   manager        GoAcceleratorMgr object.
* @param   startPort      Returns the starting port number in the range
* @param   endPort        Returns the last port number in the range.
* @return                 Operation status.
*/
GoFx(kStatus) GoAcceleratorMgr_GetPortRange(GoAcceleratorMgr manager, k16u* startPort, k16u* endPort);

/**
* Get the min and max values of the port range and the minimum number
* of ports within the port range. These limits are the default port range.
* The upper limit is set to just below the start of the ephemeral port
* range that are available for use by any network application.
*
* @public                 @memberof GoAcceleratorMgr
* @version                Introduced in firmware 5.2.18.3
* @param   manager        GoAcceleratorMgr object.
* @param   startLimit     Returns the limit for the starting port number in the range
* @param   endLimit       Returns the limit for the last port number in the range.
* @param   minNumPorts    Returns the minimum number of ports that the range must support.
* @return                 Operation status.
*/
GoFx(kStatus) GoAcceleratorMgr_GetPortRangeLimits(GoAcceleratorMgr manager, k16u* startLimit, k16u* endLimit, k16u* minNumPorts);

/**
* Get the number of sensors which the accelerator manager has been configured
* to accelerate. The count includes sensors whose acceleration may have failed
* for whatever reason.
* This is a convenient interface to find out how many sensors have
* been configured without having to get the list of sensors. This count
* should match the number of successful calls to GoAcceleratorMgr_Accelerate()
* less successful calls to GoAcceleratorMgr_Decelerate().
*
* @public                 @memberof GoAcceleratorMgr
* @version                Introduced in firmware 5.2.18.3
* @param   manager        GoAcceleratorMgr object.
* @return                 Number of sensors configured for acceleration.
*/
GoFx(kSize) GoAcceleratorMgr_AccelSensorCount(GoAcceleratorMgr manager);

#include <GoSdk/GoAcceleratorMgr.x.h>

#endif  // GO_ACCELERATOR_MGR_H

/** 
 * @file    GoAccelerator.h
 * @brief   Declares the GoAccelerator class. 
 *
 * @internal
 * Copyright (C) 2016-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef GO_ACCELERATOR_H
#define GO_ACCELERATOR_H

#include <GoSdk/GoSdkDef.h>
#include <GoSdk/GoSensor.h>

/**
 * @class   GoAccelerator
 * @extends kObject
 * @ingroup GoSdk
 * @brief   Represents an GoAccelerator instance.
 */
typedef kObject GoAccelerator;

/**
* Constructs the accelerator object
*
* @public                 @memberof GoAccelerator
* @version                Introduced in firmware 4.5.3.57
* @param   accelerator    Address of uninitialized accelerator object
* @param   allocator      Allocator object (kNULL for fallback allocator)
* @return                 Operation status.
*/
GoFx(kStatus) GoAccelerator_Construct(GoAccelerator* accelerator, kAlloc allocator);

/** 
 * Starts the accelerator service.
 *
 * @public                 @memberof GoAccelerator
 * @version                Introduced in firmware 4.5.3.57
 * @param   accelerator    GoAccelerator object.
 * @return                 Operation status.
 */
GoFx(kStatus) GoAccelerator_Start(GoAccelerator accelerator);

/** 
 * Attaches the accelerator to a sensor.
 *
 * The accelerator must have been started via GoAccelerator_Start first.
 * Note that subsequent operations on the GoSensor instance will be routed through the accelerator.
 *
 * @public                 @memberof GoAccelerator
 * @version                Introduced in firmware 4.5.3.57
 * @param   accelerator    GoAccelerator object.
 * @param   sensor         GoSensor object.
 * @return                 Operation status.
 */
GoFx(kStatus) GoAccelerator_Attach(GoAccelerator accelerator, GoSensor sensor);

/** 
 * Detaches a sensor from the accelerator.
 *
 * Note that subsequent operations on the GoSensor instance will no longer be routed through the accelerator.
 *
 * @public                 @memberof GoAccelerator
 * @version                Introduced in firmware 4.5.3.57
 * @param   accelerator    GoAccelerator object.
 * @param   sensor         GoSensor object.
 * @return                 Operation status.
 */
GoFx(kStatus) GoAccelerator_Detach(GoAccelerator accelerator, GoSensor sensor);

/** 
 * Stops the accelerator service.
 *
 * Note that all attached GoSensor instances should be detached before stopping the accelerator.
 *
 * @public                 @memberof GoAccelerator
 * @version                Introduced in firmware 4.5.3.57
 * @param   accelerator    GoAccelerator object.
 * @return                 Operation status.
 */
GoFx(kStatus) GoAccelerator_Stop(GoAccelerator accelerator);

/**
 * Reports whether or not the accelerator has been started.
 *
 * @public                 @memberof GoAccelerator
 * @version                Introduced in firmware 4.5.3.57
 * @param   accelerator    GoAccelerator object.
 * @return                 kTRUE if running, kFALSE otherwise.
 */
GoFx(kBool) GoAccelerator_IsRunning(GoAccelerator accelerator);

/**
 * Reports whether or not the specified sensor is attached to this accelerator.
 *
 * @public                 @memberof GoAccelerator
 * @version                Introduced in firmware 4.5.3.57
 * @param   accelerator    GoAccelerator object.
 * @param   sensor         GoSensor object.
 * @return                 kTRUE if attached, kFALSE otherwise.
 */
GoFx(kBool) GoAccelerator_IsAttached(GoAccelerator accelerator, GoSensor sensor);

/**
 * Gets the accelerated GoSensor handle.
 *
 * @public                 @memberof GoAccelerator
 * @version                Introduced in firmware 4.5.3.57
 * @param   accelerator    GoAccelerator object.
 * @return                 GoSensor object.
 */
GoFx(GoSensor) GoAccelerator_Sensor(GoAccelerator accelerator);

/** 
 * Sets a handler for Accelerated sensor connect/disconnect updates for this object.
 * 
 * This handler will be called if the accelerator loses or regains connection to an
 * attached sensor.
 *
 * @public                  @memberof GoAccelerator
 * @version             Introduced in firmware 4.6.4.66
 * @param   accelerator     GoAccelerator object.
 * @param   function        Connection update callback function (or kNULL to unregister).
 * @param   receiver        Receiver argument for callback. 
 * @return                  Operation status.            
 */
GoFx(kStatus) GoAccelerator_SetAcceleratorUpdateHandler(GoAccelerator accelerator, kCallbackFx function, kPointer receiver);

/**
 * Sets the IP address to use for the accelerator.
 *
 * @public                 @memberof GoAccelerator
 * @version                Introduced in firmware 4.5.3.57
 * @param   accelerator    GoAccelerator object.
 * @param   address        IP address to use for the accelerator web server.
 * @return                 Operation status.
 */
GoFx(kStatus) GoAccelerator_SetIpAddress(GoAccelerator accelerator, kIpAddress address);

/**
 * Gets the IP address used for the accelerator.
 *
 * @public                 @memberof GoAccelerator
 * @version                Introduced in firmware 4.5.3.57
 * @param   accelerator    GoAccelerator object.
 * @return                 IP address used for the accelerator web server.
 */
GoFx(kIpAddress) GoAccelerator_IpAddress(GoAccelerator accelerator);

/**
 * Sets the control port to use for the accelerator.
 *
 * @public                 @memberof GoAccelerator
 * @version                Introduced in firmware 4.5.3.57
 * @param   accelerator    GoAccelerator object.
 * @param   port           Port to use for accelerator control.
 * @return                 Operation status.
 */
GoFx(kStatus) GoAccelerator_SetControlPort(GoAccelerator accelerator, k32u port);

/**
 * Gets the control port used for the accelerator.
 *
 * @public                 @memberof GoAccelerator
 * @version                Introduced in firmware 4.5.3.57
 * @param   accelerator    GoAccelerator object.
 * @return                 Port used for accelerator control.
 */
GoFx(k32u) GoAccelerator_ControlPort(GoAccelerator accelerator);

/**
 * Sets the health port to use for the accelerator.
 *
 * @public                 @memberof GoAccelerator
 * @version                Introduced in firmware 4.5.3.57
 * @param   accelerator    GoAccelerator object.
 * @param   port           Port to use for accelerator health.
 * @return                 Operation status.
 */
GoFx(kStatus) GoAccelerator_SetHealthPort(GoAccelerator accelerator, k32u port);

/**
 * Gets the health port used for the accelerator.
 *
 * @public                 @memberof GoAccelerator
 * @version                Introduced in firmware 4.5.3.57
 * @param   accelerator    GoAccelerator object.
 * @return                 Port used for accelerator health.
 */
GoFx(k32u) GoAccelerator_HealthPort(GoAccelerator accelerator);

/**
 * Sets the upgrade port to use for the accelerator.
 *
 * @public                 @memberof GoAccelerator
 * @version                Introduced in firmware 4.5.3.57
 * @param   accelerator    GoAccelerator object.
 * @param   port           Port to use for accelerator upgrades.
 * @return                 Operation status.
 */
GoFx(kStatus) GoAccelerator_SetUpgradePort(GoAccelerator accelerator, k32u port);

/**
 * Gets the upgrade port used for the accelerator.
 *
 * @public                 @memberof GoAccelerator
 * @version                Introduced in firmware 4.5.3.57
 * @param   accelerator    GoAccelerator object.
 * @return                 Port used for accelerator upgrades.
 */
GoFx(k32u) GoAccelerator_UpgradePort(GoAccelerator accelerator);

/**
 * Sets the web server port to use for the accelerator.
 *
 * @public                 @memberof GoAccelerator
 * @version                Introduced in firmware 4.5.3.57
 * @param   accelerator    GoAccelerator object.
 * @param   port           Port to use for the accelerator web server.
 * @return                 Operation status.
 */
GoFx(kStatus) GoAccelerator_SetWebPort(GoAccelerator accelerator, k32u port);

/**
 * Gets the web server port used for the accelerator.
 *
 * @public                 @memberof GoAccelerator
 * @version                Introduced in firmware 4.5.3.57
 * @param   accelerator    GoAccelerator object.
 * @return                 Port used for the accelerator web server.
 */
GoFx(k32u) GoAccelerator_WebPort(GoAccelerator accelerator);

/**
 * Sets the private data port to use for the accelerator.
 *
 * @public                 @memberof GoAccelerator
 * @version                Introduced in firmware 4.5.3.57
 * @param   accelerator    GoAccelerator object.
 * @param   port           Port to use for accelerator private data.
 * @return                 Operation status.
 */
GoFx(kStatus) GoAccelerator_SetPrivateDataPort(GoAccelerator accelerator, k32u port);

/**
 * Gets the private data port used for the accelerator.
 *
 * @public                 @memberof GoAccelerator
 * @version                Introduced in firmware 4.5.3.57
 * @param   accelerator    GoAccelerator object.
 * @return                 Port used for accelerator private data.
 */
GoFx(k32u) GoAccelerator_PrivateDataPort(GoAccelerator accelerator);

/**
 * Sets the public data port to use for the accelerator.
 *
 * @public                 @memberof GoAccelerator
 * @version                Introduced in firmware 4.5.3.57
 * @param   accelerator    GoAccelerator object.
 * @param   port           Port to use for accelerator public data.
 * @return                 Operation status.
 */
GoFx(kStatus) GoAccelerator_SetPublicDataPort(GoAccelerator accelerator, k32u port);

/**
 * Gets the public data port used for the accelerator.
 *
 * @public                 @memberof GoAccelerator
 * @version                Introduced in firmware 4.5.3.57
 * @param   accelerator    GoAccelerator object.
 * @return                 Port used for accelerator public data.
 */
GoFx(k32u) GoAccelerator_PublicDataPort(GoAccelerator accelerator);

#include <GoSdk/GoAccelerator.x.h>

#endif

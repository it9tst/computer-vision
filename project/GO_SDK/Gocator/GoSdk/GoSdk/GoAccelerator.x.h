/** 
 * @file    GoAccelerator.x.h
 *
 * @internal
 * Copyright (C) 2016-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef GO_ACCELERATOR_X_H
#define GO_ACCELERATOR_X_H

#include <GoSdk/GoAcceleratorServer.x.h>

typedef struct GoAcceleratorClass
{
    kObjectClass base;

    // Dynamically loaded accelerator server objects
    kDynamicLib serverLib;
    
    kAssembly gsLibAssembly;
    GsLib_ConstructFx gsLibConstruct;

    GsRemoteApp_ConstructFx serverConstruct;
    GsRemoteApp_SetIpAddressFx serverSetIpAddress;
    GsRemoteApp_IpAddressFx serverIpAddress;
    GsRemoteApp_SetControlPortFx serverSetControlPort;
    GsRemoteApp_ControlPortFx serverControlPort;
    GsRemoteApp_SetHealthPortFx serverSetHealthPort;
    GsRemoteApp_HealthPortFx serverHealthPort;
    GsRemoteApp_SetUpgradePortFx serverSetUpgradePort;
    GsRemoteApp_UpgradePortFx serverUpgradePort;
    GsRemoteApp_SetWebPortFx serverSetWebPort;
    GsRemoteApp_WebPortFx serverWebPort;
    GsRemoteApp_SetPrivateDataPortFx serverSetPrivateDataPort;
    GsRemoteApp_PrivateDataPortFx serverPrivateDataPort;
    GsRemoteApp_SetPublicDataPortFx serverSetPublicDataPort;
    GsRemoteApp_PublicDataPortFx serverPublicDataPort;
    GsRemoteApp_StartFx serverStart;
    GsRemoteApp_StopFx serverStop;
    GsRemoteApp_SetAcceleratorUpdateHandlerFx serverSetAcceleratorUpdateHandler;

    // Accelerator server
    GsRemoteApp server;

    kBool isRunning;

    GoAddressInfo attachedSensorAddress;
    GoSensor attachedSensor; 

} GoAcceleratorClass;

kDeclareClassEx(Go, GoAccelerator, kObject)

#define ACCELERATOR_START_TIMEOUT (3*60*1000*1000) //3min

GoFx(kStatus) GoAccelerator_Init(GoAccelerator accelerator, kType type, kAlloc alloc);
GoFx(kStatus) GoAccelerator_VRelease(GoAccelerator accelerator);

#endif

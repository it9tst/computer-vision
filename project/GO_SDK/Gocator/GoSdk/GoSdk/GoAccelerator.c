/**
 * @file    GoAccelerator.c
 *
 * @internal
 * Copyright (C) 2016-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <kApi/Io/kPath.h>
#include <GoSdk/GoAccelerator.h>
#include <GoSdk/GoSensor.h>

#if defined(K_WINDOWS)
#include <Windows.h>    // added for global mutex checking mechanism in function GoAccelerator_Init()

HANDLE globalMutexApi = NULL;  // store handle to mutex for API level locking

#endif

kBeginClassEx(Go, GoAccelerator)
    kAddVMethod(GoAccelerator, kObject, VRelease)
kEndClassEx()

GoFx(kStatus) GoAccelerator_Construct(GoAccelerator* accelerator, kAlloc allocator)
{
#if defined(K_WINDOWS)
    kAlloc alloc = kAlloc_Fallback(allocator);
    kStatus status;

    kCheck(kAlloc_GetObject(alloc, kTypeOf(GoAccelerator), accelerator));

    if (!kSuccess(status = GoAccelerator_Init(*accelerator, kTypeOf(GoAccelerator), alloc)))
    {
        kAlloc_FreeRef(alloc, accelerator);
    }

    return status;
#else
    return kERROR_UNIMPLEMENTED; // Currently only supported on Windows
#endif
}

GoFx(kStatus) GoAccelerator_Init(GoAccelerator accelerator, kType type, kAlloc alloc)
{
    kObjR(GoAccelerator, accelerator);
    kText256 libPath = { 0 };
    kStatus exception;

    kCheck(kObject_Init(accelerator, type, alloc));
    kZero(obj->serverLib);
    kZero(obj->gsLibAssembly);
    kZero(obj->gsLibConstruct);
    kZero(obj->serverConstruct);
    kZero(obj->serverSetIpAddress);
    kZero(obj->serverIpAddress);
    kZero(obj->serverSetControlPort);
    kZero(obj->serverControlPort);
    kZero(obj->serverSetHealthPort);
    kZero(obj->serverHealthPort);
    kZero(obj->serverSetUpgradePort);
    kZero(obj->serverUpgradePort);
    kZero(obj->serverSetWebPort);
    kZero(obj->serverWebPort);
    kZero(obj->serverSetPrivateDataPort);
    kZero(obj->serverPrivateDataPort);
    kZero(obj->serverSetPublicDataPort);
    kZero(obj->serverPublicDataPort);
    kZero(obj->serverStart);
    kZero(obj->serverStop);
    kZero(obj->serverSetAcceleratorUpdateHandler);
    kZero(obj->server);
    obj->isRunning = kFALSE;
    kZero(obj->attachedSensorAddress);
    obj->attachedSensor = kNULL;

    kTry
    {
        // get server library path
        kTest(kPath_LibraryName(GS_REMOTE_APP_LIB_NAME, libPath, kCountOf(libPath)));

        kTest(kDynamicLib_Construct(&obj->serverLib, libPath, alloc));

        kTest(kDynamicLib_FindFunction(obj->serverLib, GSA_ASM_CONSTRUCT_FX, (kFunction*)&obj->gsLibConstruct));
        kTest(kDynamicLib_FindFunction(obj->serverLib, GS_REMOTE_APP_CONSTRUCT_FX, (kFunction*)&obj->serverConstruct));
        kTest(kDynamicLib_FindFunction(obj->serverLib, GS_REMOTE_APP_SET_IP_ADDRESS_FX, (kFunction*)&obj->serverSetIpAddress));
        kTest(kDynamicLib_FindFunction(obj->serverLib, GS_REMOTE_APP_IP_ADDRESS_FX, (kFunction*)&obj->serverIpAddress));
        kTest(kDynamicLib_FindFunction(obj->serverLib, GS_REMOTE_APP_SET_CONTROL_PORT_FX, (kFunction*)&obj->serverSetControlPort));
        kTest(kDynamicLib_FindFunction(obj->serverLib, GS_REMOTE_APP_CONTROL_PORT_FX, (kFunction*)&obj->serverControlPort));
        kTest(kDynamicLib_FindFunction(obj->serverLib, GS_REMOTE_APP_SET_HEALTH_PORT_FX, (kFunction*)&obj->serverSetHealthPort));
        kTest(kDynamicLib_FindFunction(obj->serverLib, GS_REMOTE_APP_HEALTH_PORT_FX, (kFunction*)&obj->serverHealthPort));
        kTest(kDynamicLib_FindFunction(obj->serverLib, GS_REMOTE_APP_SET_UPGRADE_PORT_FX, (kFunction*)&obj->serverSetUpgradePort));
        kTest(kDynamicLib_FindFunction(obj->serverLib, GS_REMOTE_APP_UPGRADE_PORT_FX, (kFunction*)&obj->serverUpgradePort));
        kTest(kDynamicLib_FindFunction(obj->serverLib, GS_REMOTE_APP_SET_WEB_PORT_FX, (kFunction*)&obj->serverSetWebPort));
        kTest(kDynamicLib_FindFunction(obj->serverLib, GS_REMOTE_APP_WEB_PORT_FX, (kFunction*)&obj->serverWebPort));
        kTest(kDynamicLib_FindFunction(obj->serverLib, GS_REMOTE_APP_SET_PRIVATE_DATA_PORT_FX, (kFunction*)&obj->serverSetPrivateDataPort));
        kTest(kDynamicLib_FindFunction(obj->serverLib, GS_REMOTE_APP_PRIVATE_DATA_PORT_FX, (kFunction*)&obj->serverPrivateDataPort));
        kTest(kDynamicLib_FindFunction(obj->serverLib, GS_REMOTE_APP_SET_PUBLIC_DATA_PORT_FX, (kFunction*)&obj->serverSetPublicDataPort));
        kTest(kDynamicLib_FindFunction(obj->serverLib, GS_REMOTE_APP_PUBLIC_DATA_PORT_FX, (kFunction*)&obj->serverPublicDataPort));
        kTest(kDynamicLib_FindFunction(obj->serverLib, GS_REMOTE_APP_START_FX, (kFunction*)&obj->serverStart));
        kTest(kDynamicLib_FindFunction(obj->serverLib, GS_REMOTE_APP_STOP_FX, (kFunction*)&obj->serverStop));
        kTest(kDynamicLib_FindFunction(obj->serverLib, GS_REMOTE_APP_SET_ACCELERATOR_UPDATE_HANDLER_FX, (kFunction*)&obj->serverSetAcceleratorUpdateHandler));

        // Construct the GsLib assembly
        kTest(obj->gsLibConstruct(&obj->gsLibAssembly));

        // Construct the accelerator server
        kTest(obj->serverConstruct(&obj->server, alloc));
    }
    kCatch(&exception)
    {
        kDestroyRef(&obj->server);
        kDestroyRef(&obj->gsLibAssembly);
        kDestroyRef(&obj->serverLib);

        kEndCatch(exception);
    }

    return kOK;
}

GoFx(kStatus) GoAccelerator_VRelease(GoAccelerator accelerator)
{
    kObj(GoAccelerator, accelerator);

    kDestroyRef(&obj->attachedSensor);
    kDestroyRef(&obj->server);
    kDestroyRef(&obj->gsLibAssembly);
    kDestroyRef(&obj->serverLib);

    return kObject_VRelease(accelerator);
}

GoFx(kStatus) GoAccelerator_Start(GoAccelerator accelerator)
{
    kObj(GoAccelerator, accelerator);
    
    obj->isRunning = kTRUE;

    return kOK;
}

GoFx(kStatus) GoAccelerator_Attach(GoAccelerator accelerator, GoSensor sensor)
{
    kObj(GoAccelerator, accelerator);
    kSize i = 0;
    k32u id = GoSensor_Id(sensor);
    kBool useStdPorts = kTRUE;
    kStatus exception;
    GoState state = GO_STATE_OFFLINE;
    k64u start;

    if (kIsNull(sensor))
    {
        return kERROR;
    }
    // ensure the sensor is not already attached
    else if (GoAccelerator_IsAttached(accelerator, sensor))
    {
        return kERROR_ALREADY_EXISTS;
    }
    // already attached to another sensor
    else if (kNULL != obj->attachedSensor)
    {
        return kERROR_BUSY;
    }

    // ensure the sensor is disconnected
    kCheck(GoSensor_Disconnect(sensor));

#if defined(K_WINDOWS)
    {
        // Mutex name needs to match GoEmulateApp/Program.cs
        if ((globalMutexApi = CreateMutexA(NULL, TRUE, "Global/GocatorPortMutex-0")) != NULL)
        {
            DWORD result = GetLastError();

            if (result != 0)
            {
                CloseHandle(globalMutexApi);
                globalMutexApi = NULL;
                //useStdPorts = kFALSE;
            }
        }
    }
#endif

    kTry
    {
        // get the sensor's address
        obj->attachedSensorAddress = *GoSensor_AddressInfo(sensor);

        if (!useStdPorts)
        {
            kTest(GoAccelerator_SetControlPort(accelerator, kIP_PORT_ANY));
            kTest(GoAccelerator_SetUpgradePort(accelerator, kIP_PORT_ANY));
            kTest(GoAccelerator_SetPrivateDataPort(accelerator, kIP_PORT_ANY));
            kTest(GoAccelerator_SetPublicDataPort(accelerator, kIP_PORT_ANY));
            kTest(GoAccelerator_SetHealthPort(accelerator, kIP_PORT_ANY));
        }

        // start accelerating this sensor
        kTest(obj->serverStart(obj->server, id));

        // store the sensor details
        kTest(kShareRef(&obj->attachedSensor, sensor));

        // update the control port
        GoSensor_SetControlPort(sensor, GoAccelerator_ControlPort(accelerator));

        // update the upgrade port
        GoSensor_SetUpgradePort(sensor, GoAccelerator_UpgradePort(accelerator));

        // update the data port
        GoSensor_SetDataPort(sensor, GoAccelerator_PublicDataPort(accelerator));

        // update the health port
        GoSensor_SetHealthPort(sensor, GoAccelerator_HealthPort(accelerator));

        // Update the sensor address to point to the local PC
        // TODO: this is a workaround in lieu of a proper wait for discovery update.
        // Should remove when proper fix is done. e.g. some way to invalidate old
        // discovery info, and then wait for new updates.
        // Directly modifying the value by pointer is also bad form.
        GoSensor_AddressInfo(sensor)->address = kIpAddress_LoopbackV4();
        
        start = kTimer_Now();
        while (ACCELERATOR_START_TIMEOUT >= (kTimer_Now()-start) &&
            GO_STATE_ONLINE != (state = GoSensor_State(obj->attachedSensor)))
        {
            kThread_Sleep(10 * 1000); //10ms sleep
        }

        if (GO_STATE_ONLINE != state) 
        {
            kTest(kERROR_TIMEOUT);
        }
    }
    kCatch(&exception)
    {
#if defined(K_WINDOWS)
        // this clears the global mutexes if the actual attachment fails
        if (globalMutexApi != NULL)
        {
            CloseHandle(globalMutexApi);
            globalMutexApi = NULL;
        }
#endif
        kEndCatch(exception);
    }

    // the parent system needs to be updated too
    //kCheck(GoSensor_RefreshSystem(sensor));

    return kOK;
}

GoFx(kStatus) GoAccelerator_Detach(GoAccelerator accelerator, GoSensor sensor)
{
    kObj(GoAccelerator, accelerator);
    kSize i = 0;
    kSize itemIndex = kSIZE_NULL;
    k32u id = GoSensor_Id(sensor);
    GoAddressInfo* address = kNULL;
    GoControl control = kNULL;

    if (!GoAccelerator_IsAttached(accelerator, sensor)) { return kERROR_NOT_FOUND; }

    // ensure the sensor is disconnected
    kCheck(GoSensor_Disconnect(sensor));

    // stop the server
    // Note - this should only stop the server for the sensor at hand,
    //        but it only works for 1 sensor right now
    kCheck(obj->serverStop(obj->server));

    // update the control channel to point to the sensor
    control = GoSensor_Control(sensor);
    kCheck(GoControl_SetRemoteAddress(control, obj->attachedSensorAddress.address));

    // update the sensor address to point to the sensor
    address = GoSensor_AddressInfo(sensor);
    *address = obj->attachedSensorAddress;

    // detach the sensor from the accelerator
    kCheck(kDestroyRef(&obj->attachedSensor));

#if defined(K_WINDOWS)
    // clear the global mutexes
    if (globalMutexApi != NULL)
    {
        CloseHandle(globalMutexApi);
        globalMutexApi = NULL;
    }
#endif

    return kOK;
}

GoFx(GoSensor) GoAccelerator_Sensor(GoAccelerator accelerator)
{
    kObj(GoAccelerator, accelerator);

    return obj->attachedSensor;
}

GoFx(kStatus) GoAccelerator_Stop(GoAccelerator accelerator)
{
    kObj(GoAccelerator, accelerator);

    obj->isRunning = kFALSE;

    return kOK;
}

GoFx(kBool) GoAccelerator_IsRunning(GoAccelerator accelerator)
{
    kObj(GoAccelerator, accelerator);

    return obj->isRunning;
}

GoFx(kBool) GoAccelerator_IsAttached(GoAccelerator accelerator, GoSensor sensor)
{
    kObj(GoAccelerator, accelerator);

    if (kIsNull(obj->attachedSensor) || kIsNull(sensor)) 
    { 
        return kFALSE; 
    }
    else if (GoSensor_Id(sensor) == GoSensor_Id(obj->attachedSensor)) 
    { 
        return kTRUE; 
    }

    return kFALSE;
}

GoFx(kStatus) GoAccelerator_SetAcceleratorUpdateHandler(GoAccelerator accelerator, kCallbackFx function, kPointer receiver)
{
    kObj(GoAccelerator, accelerator);

    return obj->serverSetAcceleratorUpdateHandler(obj->server, function, receiver);
}

GoFx(kStatus) GoAccelerator_SetIpAddress(GoAccelerator accelerator, kIpAddress address)
{
    kObj(GoAccelerator, accelerator);

    return obj->serverSetIpAddress(obj->server, address);
}

GoFx(kIpAddress) GoAccelerator_IpAddress(GoAccelerator accelerator)
{
    kObj(GoAccelerator, accelerator);

    return obj->serverIpAddress(obj->server);
}

GoFx(kStatus) GoAccelerator_SetControlPort(GoAccelerator accelerator, k32u port)
{
    kObj(GoAccelerator, accelerator);

    return obj->serverSetControlPort(obj->server, port);
}

GoFx(k32u) GoAccelerator_ControlPort(GoAccelerator accelerator)
{
    kObj(GoAccelerator, accelerator);

    return obj->serverControlPort(obj->server);
}

GoFx(kStatus) GoAccelerator_SetHealthPort(GoAccelerator accelerator, k32u port)
{
    kObj(GoAccelerator, accelerator);

    return obj->serverSetHealthPort(obj->server, port);
}

GoFx(k32u) GoAccelerator_HealthPort(GoAccelerator accelerator)
{
    kObj(GoAccelerator, accelerator);

    return obj->serverHealthPort(obj->server);
}

GoFx(kStatus) GoAccelerator_SetUpgradePort(GoAccelerator accelerator, k32u port)
{
    kObj(GoAccelerator, accelerator);

    return obj->serverSetUpgradePort(obj->server, port);
}

GoFx(k32u) GoAccelerator_UpgradePort(GoAccelerator accelerator)
{
    kObj(GoAccelerator, accelerator);

    return obj->serverUpgradePort(obj->server);
}

GoFx(kStatus) GoAccelerator_SetWebPort(GoAccelerator accelerator, k32u port)
{
    kObj(GoAccelerator, accelerator);

    return obj->serverSetWebPort(obj->server, port);
}

GoFx(k32u) GoAccelerator_WebPort(GoAccelerator accelerator)
{
    kObj(GoAccelerator, accelerator);

    return obj->serverWebPort(obj->server);
}

GoFx(kStatus) GoAccelerator_SetPrivateDataPort(GoAccelerator accelerator, k32u port)
{
    kObj(GoAccelerator, accelerator);

    return obj->serverSetPrivateDataPort(obj->server, port);
}

GoFx(k32u) GoAccelerator_PrivateDataPort(GoAccelerator accelerator)
{
    kObj(GoAccelerator, accelerator);

    return obj->serverPrivateDataPort(obj->server);
}

GoFx(kStatus) GoAccelerator_SetPublicDataPort(GoAccelerator accelerator, k32u port)
{
    kObj(GoAccelerator, accelerator);

    return obj->serverSetPublicDataPort(obj->server, port);
}

GoFx(k32u) GoAccelerator_PublicDataPort(GoAccelerator accelerator)
{
    kObj(GoAccelerator, accelerator);

    return obj->serverPublicDataPort(obj->server);
}

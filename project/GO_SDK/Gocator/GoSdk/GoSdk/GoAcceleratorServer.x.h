/**
 * @file    GoAccelerator.x.h
 *
 * @internal
 * Copyright (C) 2016-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef GO_ACCELERATOR_SERVER_X_H
#define GO_ACCELERATOR_SERVER_X_H

#include <kApi/Utils/kDynamicLib.h>
#include <kApi/Utils/kPlugin.h>

#define GS_REMOTE_APP_LIB_NAME      "GoSensorAppLib"

typedef kObject GsRemoteApp; // forward declaration

GoCx(kStatus) GsaAsm_ConstructAssembly(kAssembly* assembly);
typedef kStatus(kDlCall* GsLib_ConstructFx)(kAssembly* assembly);
#define GSA_ASM_CONSTRUCT_FX            "GsaAsm_ConstructAssembly"

GoCx(kStatus) GsRemoteApp_Construct(GsRemoteApp* app, kAlloc allocator);
typedef kStatus(kDlCall* GsRemoteApp_ConstructFx)(GsRemoteApp* app, kAlloc allocator);
#define GS_REMOTE_APP_CONSTRUCT_FX      "GsRemoteApp_Construct"

GoCx(kStatus) GsRemoteApp_SetIpAddress(GsRemoteApp app, kIpAddress address);
typedef kStatus(kDlCall* GsRemoteApp_SetIpAddressFx)(GsRemoteApp app, kIpAddress address);
#define GS_REMOTE_APP_SET_IP_ADDRESS_FX          "GsRemoteApp_SetIpAddress"

GoCx(kIpAddress) GsRemoteApp_IpAddress(GsRemoteApp app);
typedef kIpAddress(kDlCall* GsRemoteApp_IpAddressFx)(GsRemoteApp app);
#define GS_REMOTE_APP_IP_ADDRESS_FX           "GsRemoteApp_IpAddress"

GoCx(kStatus) GsRemoteApp_SetControlPort(GsRemoteApp app, k32u port);
typedef kStatus(kDlCall* GsRemoteApp_SetControlPortFx)(GsRemoteApp app, k32u port);
#define GS_REMOTE_APP_SET_CONTROL_PORT_FX          "GsRemoteApp_SetControlPort"

GoCx(k32u) GsRemoteApp_ControlPort(GsRemoteApp app);
typedef k32u(kDlCall* GsRemoteApp_ControlPortFx)(GsRemoteApp app);
#define GS_REMOTE_APP_CONTROL_PORT_FX           "GsRemoteApp_ControlPort"

GoCx(kStatus) GsRemoteApp_SetHealthPort(GsRemoteApp app, k32u port);
typedef kStatus(kDlCall* GsRemoteApp_SetHealthPortFx)(GsRemoteApp app, k32u port);
#define GS_REMOTE_APP_SET_HEALTH_PORT_FX          "GsRemoteApp_SetHealthPort"

GoCx(k32u) GsRemoteApp_HealthPort(GsRemoteApp app);
typedef k32u(kDlCall* GsRemoteApp_HealthPortFx)(GsRemoteApp app);
#define GS_REMOTE_APP_HEALTH_PORT_FX           "GsRemoteApp_HealthPort"

GoCx(kStatus) GsRemoteApp_SetUpgradePort(GsRemoteApp app, k32u port);
typedef kStatus(kDlCall* GsRemoteApp_SetUpgradePortFx)(GsRemoteApp app, k32u port);
#define GS_REMOTE_APP_SET_UPGRADE_PORT_FX          "GsRemoteApp_SetUpgradePort"

GoCx(k32u) GsRemoteApp_UpgradePort(GsRemoteApp app);
typedef k32u(kDlCall* GsRemoteApp_UpgradePortFx)(GsRemoteApp app);
#define GS_REMOTE_APP_UPGRADE_PORT_FX           "GsRemoteApp_UpgradePort"

GoCx(kStatus) GsRemoteApp_SetWebPort(GsRemoteApp app, k32u port);
typedef kStatus(kDlCall* GsRemoteApp_SetWebPortFx)(GsRemoteApp app, k32u port);
#define GS_REMOTE_APP_SET_WEB_PORT_FX          "GsRemoteApp_SetWebPort"

GoCx(k32u) GsRemoteApp_WebPort(GsRemoteApp app);
typedef k32u(kDlCall* GsRemoteApp_WebPortFx)(GsRemoteApp app);
#define GS_REMOTE_APP_WEB_PORT_FX           "GsRemoteApp_WebPort"

GoCx(kStatus) GsRemoteApp_SetPrivateDataPort(GsRemoteApp app, k32u port);
typedef kStatus(kDlCall* GsRemoteApp_SetPrivateDataPortFx)(GsRemoteApp app, k32u port);
#define GS_REMOTE_APP_SET_PRIVATE_DATA_PORT_FX          "GsRemoteApp_SetPrivateDataPort"

GoCx(k32u) GsRemoteApp_PrivateDataPort(GsRemoteApp app);
typedef k32u(kDlCall* GsRemoteApp_PrivateDataPortFx)(GsRemoteApp app);
#define GS_REMOTE_APP_PRIVATE_DATA_PORT_FX           "GsRemoteApp_PrivateDataPort"

GoCx(kStatus) GsRemoteApp_SetPublicDataPort(GsRemoteApp app, k32u port);
typedef kStatus(kDlCall* GsRemoteApp_SetPublicDataPortFx)(GsRemoteApp app, k32u port);
#define GS_REMOTE_APP_SET_PUBLIC_DATA_PORT_FX          "GsRemoteApp_SetPublicDataPort"

GoCx(k32u) GsRemoteApp_PublicDataPort(GsRemoteApp app);
typedef k32u(kDlCall* GsRemoteApp_PublicDataPortFx)(GsRemoteApp app);
#define GS_REMOTE_APP_PUBLIC_DATA_PORT_FX           "GsRemoteApp_PublicDataPort"

GoCx(kStatus) GsRemoteApp_Start(GsRemoteApp app, k32u deviceId);
typedef kStatus(kDlCall* GsRemoteApp_StartFx)(GsRemoteApp app, k32u deviceId);
#define GS_REMOTE_APP_START_FX          "GsRemoteApp_Start"

GoCx(kStatus) GsRemoteApp_Stop(GsRemoteApp app);
typedef kStatus(kDlCall* GsRemoteApp_StopFx)(GsRemoteApp app);
#define GS_REMOTE_APP_STOP_FX           "GsRemoteApp_Stop"

GoCx(kStatus) GsRemoteApp_SetAcceleratorUpdateHandler(GsRemoteApp app, kCallbackFx function, kPointer receiver);
typedef kStatus(kDlCall* GsRemoteApp_SetAcceleratorUpdateHandlerFx)(GsRemoteApp app, kCallbackFx function, kPointer receiver);
#define GS_REMOTE_APP_SET_ACCELERATOR_UPDATE_HANDLER_FX         "GsRemoteApp_SetAcceleratorUpdateHandler"

#endif

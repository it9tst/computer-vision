/**
 * @file    GoSdkReservedPorts.h
 * @brief   This file lists all the ports used by the Gocator software. This
 * means these ports cannot be used by any other application.
 * This also means that an accelerator virtual sensor's Gocator software
 * can take up one or more of these ports on the accelerator platform.
 * So if more than one accelerated sensor runs on a platform, care must
 * be taken to avoid port conflicts.
 *
 * @internal
 * Copyright (C) 2013 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef GO_SDK_RESERVED_PORTS_H
#define GO_SDK_RESERVED_PORTS_H

#define GO_SDK_RESERVED_PORT_DEFAULT_SENSOR_HTTP_SERVER             80
#define GO_SDK_RESERVED_PORT_DEFAULT_MODBUS_SERVER                  502
#define GO_SDK_RESERVED_PORT_DEFAULT_ETHERNET_IP_IMPLICIT_SERVER    2222
 // Used for accelerator events between accelerator manager (parent) process
 // and virtual accelerator sensor (child) process.
#define GO_SDK_RESERVED_PORT_ACCELERATOR_EVENT                      3000
#define GO_SDK_RESERVED_PORT_FLASH_POLICY_SERVER                    3189
#define GO_SDK_RESERVED_PORT_SENSOR_CONTROL                         3190
#define GO_SDK_RESERVED_PORT_SENSOR_UPGRADE                         3192
#define GO_SDK_RESERVED_PORT_SENSOR_HEALTH                          3194
#define GO_SDK_RESERVED_PORT_SENSOR_PRIVATE_DATA                    3195
#define GO_SDK_RESERVED_PORT_SENSOR_PUBLIC_DATA                     3196
#define GO_SDK_RESERVED_PORT_DISCOVERY_PROTOCOL                     3220
#define GO_SDK_RESERVED_PORT_REMOTE_PROCEDURE_CALL                  3400
#define GO_SDK_RESERVED_PORT_DEFAULT_ACCELERATED_SENSOR_HTTP_SERVER 8080
#define GO_SDK_RESERVED_PORT_DEFAULT_ETHERNET_ASCII_SERVER          8190
#define GO_SDK_RESERVED_PORT_DEFAULT_ETHERNET_IP_EXPLICIT_SERVER    44818

#endif  // GO_SDK_RESERVED_PORTS_H

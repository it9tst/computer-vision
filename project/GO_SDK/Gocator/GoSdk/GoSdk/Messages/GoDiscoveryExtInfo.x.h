/**
 * @file    GoDiscoveryExtInfo.x.h
 *
 * @internal
 * Copyright (C) 2016-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef GO_SDK_DISCOVERY_EXT_INFO_X_H
#define GO_SDK_DISCOVERY_EXT_INFO_X_H

#include <kApi/Io/kSerializer.h>
#include <kApi/Data/kArrayList.h>
#include <kApi/Data/kBytes.h>
#include <GoSdk/Internal/GoDiscovery.h>

kDeclareValueEx(Go, GoDiscoveryProperty, kValue)

typedef struct GoDiscoveryExtInfoClass
{
    kObjectClass base;

    k32u id;
    GoAddressInfo address;
    GoPortInfo ports;
    kVersion version;
    k64u uptime;
    kArrayList properties;
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
} GoDiscoveryExtInfoClass;

kDeclareClassEx(Go, GoDiscoveryExtInfo, kObject)

GoFx(kStatus) GoDiscoveryExtInfo_Construct(GoDiscoveryExtInfo* msg, kAlloc allocator);
GoFx(kStatus) GoDiscoveryExtInfo_Init(GoDiscoveryExtInfo msg, kType type, kAlloc alloc);
GoFx(kStatus) GoDiscoveryExtInfo_VInitClone(GoDiscoveryExtInfo msg, GoDiscoveryExtInfo source, kAlloc alloc);
GoFx(kStatus) GoDiscoveryExtInfo_AllocateProperties(GoDiscoveryExtInfo msg, kSize count);
GoFx(kStatus) GoDiscoveryExtInfo_VRelease(GoDiscoveryExtInfo msg);
GoFx(kSize) GoDiscoveryExtInfo_VSize(GoDiscoveryExtInfo msg);
GoFx(kStatus) GoDiscoveryExtInfo_Read(GoDiscoveryExtInfo msg, kSerializer serializer, kAlloc alloc);

GoFx(kStatus) GoDiscoveryExtInfo_ExtractAccelSensorInfoFromMsg(GoDiscoveryExtInfo msg, kSerializer serializer);
GoFx(kStatus) GoDiscoveryExtInfo_ExtractAccelSensorInfoFromProperty(GoDiscoveryExtInfo msg, kBool* foundAccelInfo);
GoFx(kStatus) GoDiscoveryExtInfo_ReadProperties(GoDiscoveryExtInfo msg, kSerializer serializer, k8u propertyCount);

GoFx(kStatus) GoDiscoveryExtInfo_FillModelNumber(GoDiscoveryExtInfo msg, kChar* fallBackModelNumber, kSize capacity);
GoFx(kStatus) GoDiscoveryExtInfo_ModelNumber(GoDiscoveryExtInfo msg, kChar* modelNumber, kSize capacity);
GoFx(kStatus) GoDiscoveryExtInfo_ModelDisplayName(GoDiscoveryExtInfo msg, kChar* modelDisplayName, kSize capacity);
GoFx(kStatus) GoDiscoveryExtInfo_Family(GoDiscoveryExtInfo msg, kChar* family, kSize capacity);

GoFx(kSize) GoDiscoveryExtInfo_FindProperty(GoDiscoveryExtInfo msg, const kChar *name);

#define GoDiscoveryExtInfo_SetContent_(D, V)            (xGoDiscoveryExtInfo_CastRaw(D) = (V), kOK)
#define GoDiscoveryExtInfo_Content_(D)                  (xGoDiscoveryExtInfo_CastRaw(D))
#define GoDiscoveryExtInfo_SetSource_(D, V)             (xGoDiscoveryExtInfo_CastRaw(D)->source = (V), kOK)

/**
* @deprecated Gets the sensor model name.
*             Use GoDiscoveryExtInfo_PartNumber() instead.
*
* @public             @memberof GoDiscoveryExtInfo
* @version            Introduced in firmware 5.2.18.3
* @param   msg        Message object.
* @param   model      Buffer to store the sensor model name.
* @param   capacity   Number of bytes in the buffer for the sensor model name.
* @return             Operation status.
*/
GoFx(kStatus) GoDiscoveryExtInfo_CompleteModel(GoDiscoveryExtInfo msg, kChar* model, kSize capacity);

#endif

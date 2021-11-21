/** 
 * @file    GoSensorInfo.x.h
 *
 * @internal
 * Copyright (C) 2017-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef GO_SDK_SENSOR_INFO_X_H
#define GO_SDK_SENSOR_INFO_X_H

#include <GoSdk/GoSensorInfo.h>

typedef struct GoSensorInfoClass
{
    kObjectClass base; 

    k32u id; 
    kVersion firmwareVersion; 
    kText32 partNumber;             // Sensor part number ((renamed from model for clarity).
    GoRole role; 
    k32s buddyId; 
    GoState state; 
    kIpAddress address;
    //new v4.6 
    k32u mainId;
    GoBuddyState buddyableStatus;
    kText32 modelNumber;            // Sensor model number (for parsing).
    kText32 modelDisplayName;       // Sensor model display name (for display).
} GoSensorInfoClass; 

kDeclareClassEx(Go, GoSensorInfo, kObject)

GoFx(kStatus) GoSensorInfo_Construct(GoSensorInfo* info, kAlloc allocator);
GoFx(kStatus) GoSensorInfo_Init(GoSensorInfo info, kType type, kAlloc alloc);
GoFx(kStatus) GoSensorInfo_VRelease(GoSensorInfo info);
GoFx(kStatus) GoSensorInfo_Read(GoSensorInfo info, kSerializer serializer);
GoFx(kStatus) GoSensorInfo_ReadV2(GoSensorInfo info, kSerializer serializer, k16u sensorInfoSize, kBool isLocalInfo);

GoFx(k32s) GoSensorInfo_buddyableStatus(GoSensorInfo info);
GoFx(kStatus) GoSensorInfo_FillModelNumber(GoSensorInfo info, kChar *partNumber, kChar *fallBackModelNumber, kSize capacity);
GoFx(kStatus) GoSensorInfo_FillModelDisplayName(GoSensorInfo info, kChar * partNumber, kChar *fallBackModelDisplayName, kSize capacity);

/** 
 * @deprecated Gets the alignment state of the device which provided the sensor information.
 *
 * @public              @memberof GoSensorInfo
 * @version             Introduced in firmware 4.0.10.27
 * @param   info        GoSensorInfo object.
 * @return              Alignment state.
 * @see                 GoSensor_AlignmentState, GoSensor_States
 */
GoFx(GoAlignmentState) GoSensorInfo_TransformState(GoSensorInfo info);

/** 
 * @deprecated Gets the model string of the sensor which provided the sensor information.
 *             Use GoSensor_PartNumber() or GoSensor_ModelDisplayName().
 *
 * @public              @memberof GoSensorInfo
 * @version             Introduced in firmware 4.0.10.27
 * @param   info        GoSensorInfo object.
 * @param   model       A character array pointer.
 * @param   capacity    The character array capacity.
 * @return              Operation status.
 */
GoFx(kStatus) GoSensorInfo_Model(GoSensorInfo info, kChar* model, kSize capacity);

/** 
 * @deprecated Gets the model number of the sensor which provided the sensor information.
 *             For internal use only to retrieve the LMI model number of the sensor.
 *
 * @public                      @memberof GoSensorInfo
 * @version                     Introduced in firmware 5.3.17.23
 * @param   info                GoSensorInfo object.
 * @param   modelNumber         A character array pointer.
 * @param   capacity            The character array capacity.
 * @return                      Operation status.
 */
GoFx(kStatus) GoSensorInfo_ModelNumber(GoSensorInfo info, kChar* modelNumber, kSize capacity);


#endif

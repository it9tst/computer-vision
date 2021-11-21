/** 
 * @file    GoSensorInfo.c
 *
 * @internal
 * Copyright (C) 2016-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <GoSdk/GoSensorInfo.h>
#include <kApi/Utils/kUtils.h>

kBeginClassEx(Go, GoSensorInfo)
kEndClassEx()

GoFx(kStatus) GoSensorInfo_Construct(GoSensorInfo* info, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback(allocator);
    kStatus status; 

    kCheck(kAlloc_GetObject(alloc, kTypeOf(GoSensorInfo), info)); 

    if (!kSuccess(status = GoSensorInfo_Init(*info, kTypeOf(GoSensorInfo), alloc)))
    {
        kAlloc_FreeRef(alloc, info); 
    }

    return status; 
} 

GoFx(kStatus) GoSensorInfo_Init(GoSensorInfo info, kType type, kAlloc alloc)
{
    kObjR(GoSensorInfo, info); 

    kCheck(kObject_Init(info, type, alloc)); 
    obj->id = 0;
    kZero(obj->firmwareVersion);
    obj->partNumber[0] = 0;
    obj->buddyId = 0;
    obj->address = kIpAddress_AnyV4();
    obj->mainId = 0;
    obj->buddyableStatus = GO_BUDDY_STATE_ERROR;

    obj->role = GO_ROLE_MAIN; 
    obj->state = GO_STATE_OFFLINE; 
    obj->modelNumber[0] = 0;
    obj->modelDisplayName[0] = 0;
    
    return kOK; 
}

GoFx(kStatus) GoSensorInfo_Read(GoSensorInfo info, kSerializer serializer)
{
    kObj(GoSensorInfo, info); 
    k32s temp; 

    kByte tempArray[4];
    kSize i;

    kCheck(kSerializer_Read32u(serializer, &obj->id)); 

    kCheck(kSerializer_ReadByteArray(serializer, tempArray, kCountOf(tempArray)));
    for (i = 0; i < 16; i++)
    {
        if (i < 4)
        {
            obj->address.address[i] = tempArray[i];
        }
        else
        {
            obj->address.address[i] = 0;
        }
    }
    //hack for now

    // modelName is deprecated, underlying var is renamed to partNumber for clarity.
    kCheck(kSerializer_ReadCharArray(serializer, obj->partNumber, kCountOf(obj->partNumber))); 

    kCheck(kSerializer_ReadByteArray(serializer, tempArray, kCountOf(tempArray)));

    obj->firmwareVersion = kVersion_Create(tempArray[0], tempArray[1], tempArray[2], tempArray[3]);

    kCheck(kSerializer_Read32s(serializer, &temp)); 
    switch(temp)
    {
    case -1:    obj->state = GO_STATE_INCOMPLETE;       break;
    case 0:     obj->state = GO_STATE_READY;            break;
    case 1:     obj->state = GO_STATE_RUNNING;          break;
    default:    obj->state = GO_STATE_CONNECTED;        break;
    }

    kCheck(kSerializer_Read32s(serializer, &obj->role)); 

    kCheck(kSerializer_Read32s(serializer, &obj->buddyId));

    // Even for very old sensors, we can still provide fallback modelNumber and display name.
    kCheck(GoSensorInfo_FillModelNumber(info, obj->partNumber, obj->modelNumber, kCountOf(obj->modelNumber)));
    kCheck(GoSensorInfo_FillModelDisplayName(info, obj->partNumber, obj->modelDisplayName, kCountOf(obj->modelDisplayName)));

    return kOK; 
}

GoFx(kStatus) GoSensorInfo_ReadV2(GoSensorInfo info, kSerializer serializer, k16u sensorInfoSize, kBool isLocalInfo)
{
    kObj(GoSensorInfo, info);
    k32s temp;
    k32u bytesRead = 0;
    kByte tempArray[4];
    kSize i;

    kCheck(kSerializer_Read32u(serializer, &obj->id));

    kCheck(kSerializer_ReadByteArray(serializer, tempArray, kCountOf(tempArray)));
    for (i = 0; i < 16; i++)
    {
        if (i < 4)
        {
            obj->address.address[i] = tempArray[i];
        }
        else
        {
            obj->address.address[i] = 0;
        }
    }
    //hack for now


    // modelName is deprecated, underlying var is renamed to partNumber for clarity.
    kCheck(kSerializer_ReadCharArray(serializer, obj->partNumber, kCountOf(obj->partNumber)));

    kCheck(kSerializer_ReadByteArray(serializer, tempArray, kCountOf(tempArray)));

    obj->firmwareVersion = kVersion_Create(tempArray[0], tempArray[1], tempArray[2], tempArray[3]);

    kCheck(kSerializer_Read32s(serializer, &temp));
    switch (temp)
    {
    case -1:    obj->state = GO_STATE_INCOMPLETE;       break;
    case 0:     obj->state = GO_STATE_READY;            break;
    case 1:     obj->state = GO_STATE_RUNNING;          break;
    default:    obj->state = GO_STATE_CONNECTED;        break;
    }

    kCheck(kSerializer_Read32s(serializer, &obj->role));

    bytesRead = 52;
    //SensorInfoV2 is variable size, so need to track the number of bytes read in case a future sensor version is sending more data than what this version expects

    if (!isLocalInfo)
    {
        kCheck(kSerializer_Read32u(serializer, &obj->mainId));
        kCheck(kSerializer_Read32s(serializer, &obj->buddyableStatus));
        bytesRead += 8;
    }

    // During 5.3 Beta, before 5.3.10.31
    if (sensorInfoSize - bytesRead == 0)
    {
        // Provide fallback values for modelNumber and modelDisplayName.
        kCheck(GoSensorInfo_FillModelNumber(info, obj->partNumber, obj->modelNumber, kCountOf(obj->modelNumber)));
        kCheck(GoSensorInfo_FillModelDisplayName(info, obj->partNumber, obj->modelDisplayName, kCountOf(obj->modelDisplayName)));
    }
    // During 5.3 Beta for 5.3.10.31 - 5.3.17.23.
    else if (sensorInfoSize - bytesRead == kCountOf(obj->modelDisplayName))
    {
        // Read modelDisplayName, and provide fallback values for modelNumber.
        kCheck(GoSensorInfo_FillModelNumber(info, obj->partNumber, obj->modelNumber, kCountOf(obj->modelNumber)));
        kCheck(kSerializer_ReadCharArray(serializer, obj->modelDisplayName, kCountOf(obj->modelDisplayName))); 
        bytesRead += kCountOf(obj->modelDisplayName);
    }
    // During 5.3 Beta for 5.3.17.23 and onwards.. to 5.3 Release
    else if (sensorInfoSize - bytesRead >= kCountOf(obj->modelDisplayName) + kCountOf(obj->modelNumber))
    {
        // Read modelNumber and modelDisplayName.
        kCheck(kSerializer_ReadCharArray(serializer, obj->modelNumber, kCountOf(obj->modelNumber))); 
        bytesRead += kCountOf(obj->modelNumber);
        kCheck(kSerializer_ReadCharArray(serializer, obj->modelDisplayName, kCountOf(obj->modelDisplayName))); 
        bytesRead += kCountOf(obj->modelDisplayName);
    }

    kCheck(kSerializer_AdvanceRead(serializer, sensorInfoSize - bytesRead));

    return kOK;
}

GoFx(k32u) GoSensorInfo_Id(GoSensorInfo info)
{
    kObj(GoSensorInfo, info);  
    return obj->id; 
}

GoFx(kVersion) GoSensorInfo_Firmware(GoSensorInfo info)
{
    kObj(GoSensorInfo, info);  
    return obj->firmwareVersion; 
}

// deprecated
GoFx(kStatus) GoSensorInfo_Model(GoSensorInfo info, kChar* model, kSize capacity)
{
    kObj(GoSensorInfo, info);
    // modelName is deprecated, provide the original behavior which was to return partNumber.
    return GoSensorInfo_PartNumber(info, model, capacity);
}

GoFx(kStatus) GoSensorInfo_PartNumber(GoSensorInfo info, kChar* partNumber, kSize capacity)
{
    kObj(GoSensorInfo, info);
    // NOTE: the partnumber returned in the remote info is actually the "Derived LMI PartNumber",
    //       which hides branding customizations, and may not fully match the true part number
    //       that is available in GoSensor_PartNumber().
    return kStrCopy(partNumber, capacity, obj->partNumber); 
}

GoFx(kStatus) GoSensorInfo_ModelNumber(GoSensorInfo info, kChar* modelNumber, kSize capacity)
{
    kObj(GoSensorInfo, info);  
    return kStrCopy(modelNumber, capacity, obj->modelNumber); 
}

GoFx(kStatus) GoSensorInfo_ModelDisplayName(GoSensorInfo info, kChar* modelDisplayName, kSize capacity)
{
    kObj(GoSensorInfo, info);  
    return kStrCopy(modelDisplayName, capacity, obj->modelDisplayName); 
}

GoFx(GoRole) GoSensorInfo_Role(GoSensorInfo info)
{
    kObj(GoSensorInfo, info);  
    return obj->role; 
}

GoFx(kBool) GoSensorInfo_HasBuddy(GoSensorInfo info)
{
    kObj(GoSensorInfo, info);  
    return (obj->buddyId != 0); 
}

GoFx(k32u) GoSensorInfo_BuddyId(GoSensorInfo info)
{
    kObj(GoSensorInfo, info);  
    return obj->buddyId; 
}

GoFx(GoState) GoSensorInfo_State(GoSensorInfo info)
{
    kObj(GoSensorInfo, info);  
    return obj->state; 
}

GoFx(GoBuddyState) GoSensorInfo_BuddyableStatus(GoSensorInfo info)
{
    kObj(GoSensorInfo, info);
    return obj->buddyableStatus;
}

GoFx(k32s) GoSensorInfo_buddyableStatus(GoSensorInfo info)
{
    return GoSensorInfo_BuddyableStatus(info);
}

GoFx(k32u) GoSensorInfo_MainId(GoSensorInfo info)
{
    kObj(GoSensorInfo, info);
    return obj->mainId;
}

GoFx(kStatus) GoSensorInfo_FillModelNumber(GoSensorInfo info, kChar *partNumber, kChar *fallBackModelNumber, kSize capacity)
{
    kObj(GoSensorInfo, info);
    const char* pos = kNULL;
    kString tempString = kNULL;
    k32s    tempCleanNum    = kNULL;
    kArrayList tokens = kNULL;
    kStatus exception;

    kTry
    {
        kTest(kString_Construct(&tempString, partNumber, kObject_Alloc(info)));
        kTest(kString_Split(tempString, "-", &tokens, kObject_Alloc(info)));

        if(kArrayList_Count(tokens) == 0)
        {
            // This is the same fallback as in GsDevice_ParseModelNumber().
            kTest(kStrCopy(fallBackModelNumber, capacity, "2300"));
        }
        else
        {
            // We take the first token in the part number after the two-digit class (31, 33),
            // then convert to a number to get rid of A,B,C suffixes.
            //   ie. 33250A-3B-R-52-S
            //         ^^^  = "250"
            //
            //       312330C-2M-R-01-T
            //         ^^^^ = "2330"
            // 
            kTest(k32s_Parse(&tempCleanNum, kString_Chars(kArrayList_AsT(tokens, 0, kString)) + 2));
            kTest(k32s_Format(tempCleanNum, fallBackModelNumber, capacity));
        }
    }
    kCatchEx(&exception)
    {
        kLogf("%s: cannot fill model number into capacity[%u] from '%s'", __FUNCTION__, capacity, partNumber);
        kEndCatchEx(exception);
    }
    kFinallyEx
    {
        kDestroyRef(&tempString);
        kDisposeRef(&tokens);
        kEndFinallyEx();
    }

    return kOK;
}

GoFx(kStatus) GoSensorInfo_FillModelDisplayName(GoSensorInfo info, kChar * partNumber, kChar *fallBackModelDisplayName, kSize capacity)
{
    kObj(GoSensorInfo, info);
    return GoSensorInfo_FillModelNumber(info, partNumber, fallBackModelDisplayName, capacity);
}

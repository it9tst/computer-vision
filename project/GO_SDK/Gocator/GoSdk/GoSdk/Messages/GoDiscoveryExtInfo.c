/**
 * @file    GoDiscoveryExtInfo.c
 *
 * @internal
 * Copyright (C) 2016-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <GoSdk/Messages/GoDiscoveryExtInfo.h>

kBeginValueEx(Go, GoDiscoveryProperty)
    kAddField(GoDiscoveryProperty, kText256, name)
    kAddField(GoDiscoveryProperty, kText256, value)
kEndValueEx()

/*
 * GoDiscoveryExtInfo
 */
kBeginClassEx(Go, GoDiscoveryExtInfo)
    //virtual methods
    kAddVMethod(GoDiscoveryExtInfo, kObject, VInitClone)
    kAddVMethod(GoDiscoveryExtInfo, kObject, VRelease)
    kAddVMethod(GoDiscoveryExtInfo, kObject, VSize)
kEndClassEx()

GoFx(kStatus) GoDiscoveryExtInfo_Construct(GoDiscoveryExtInfo* msg, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback(allocator);
    kStatus status;

    kCheck(kAlloc_GetObject(alloc, kTypeOf(GoDiscoveryExtInfo), msg));

    if (!kSuccess(status = GoDiscoveryExtInfo_Init(*msg, kTypeOf(GoDiscoveryExtInfo), alloc)))
    {
        kAlloc_FreeRef(alloc, msg);
    }

    return status;
}

GoFx(kStatus) GoDiscoveryExtInfo_Init(GoDiscoveryExtInfo msg, kType type, kAlloc alloc)
{
    kObjR(GoDiscoveryExtInfo, msg);

    kCheck(kObject_Init(msg, type, alloc));
    obj->id = 0;
    kZero(obj->address);
    kZero(obj->ports);
    kZero(obj->version);
    obj->uptime = 0;
    kZero(obj->properties);
    obj->opMode = 0;
    obj->accelSensorIpAddress = kIpAddress_AnyV4();

    return kOK;
}

GoFx(kStatus) GoDiscoveryExtInfo_VInitClone(GoDiscoveryExtInfo msg, GoDiscoveryExtInfo source, kAlloc alloc)
{
    kObjR(GoDiscoveryExtInfo, msg);
    kObjN(GoDiscoveryExtInfo, srcObj, source);
    kStatus status;

    kCheck(GoDiscoveryExtInfo_Init(msg, kObject_Type(source), alloc));

    kTry
    {
        obj->id = srcObj->id;
        obj->version = srcObj->version;
        obj->uptime = srcObj->uptime;
        kMemCopy(&(obj->address), &(srcObj->address), sizeof(GoAddressInfo));
        kMemCopy(&(obj->ports), &(srcObj->ports), sizeof(GoPortInfo));

        kTest(kObject_Clone(&obj->properties, srcObj->properties, alloc));
    }
    kCatch(&status)
    {
        GoDiscoveryExtInfo_VRelease(msg);
        kEndCatch(status);
    }

    return kOK;
}

GoFx(kStatus) GoDiscoveryExtInfo_AllocateProperties(GoDiscoveryExtInfo msg, kSize count)
{
    kObj(GoDiscoveryExtInfo, msg);

    kCheck(kDisposeRef(&obj->properties));
    kCheck(kArrayList_Construct(&obj->properties, kTypeOf(GoDiscoveryProperty), count, kObject_Alloc(msg)));
    kCheck(kArrayList_AddCount(obj->properties, count));

    return kOK;
}

GoFx(kStatus) GoDiscoveryExtInfo_VRelease(GoDiscoveryExtInfo msg)
{
    kObj(GoDiscoveryExtInfo, msg);

    kCheck(kDisposeRef(&obj->properties));

    return kObject_VRelease(msg);
}

GoFx(kSize) GoDiscoveryExtInfo_VSize(GoDiscoveryExtInfo msg)
{
    kObj(GoDiscoveryExtInfo, msg);

    return sizeof(GoDiscoveryExtInfoClass) + (kIsNull(obj->properties) ? 0 : kObject_Size(obj->properties));
}

GoFx(k32u) GoDiscoveryExtInfo_SubnetPrefixToMask(k32u prefix)
{
    k32u mask = 0;
    k32s lower = 32 - (k32s)prefix;
    k32s i = 0;

    for (i = 31; i >= lower; --i)
    {
        mask |= (1 << i);
    }

    return mask;
}


GoFx(kStatus) GoDiscoveryExtInfo_Read(GoDiscoveryExtInfo msg, kSerializer serializer, kAlloc alloc)
{
    kObj(GoDiscoveryExtInfo, msg);
    k64s responseId;
    k32u prefixLength;
    k64s signature;
    kByte useDhcp;
    kByte tempByte;
    k64s status;
    k16u attrSize;
    k8u propertyCount;
    kBool accelInfoInProperty = kFALSE;

    kCheck(GoDiscoveryExtInfo_Init(msg, kTypeOf(GoDiscoveryExtInfo), alloc));

    kTry
    {
        // GOC-12805: changed writing the header as 32-bit data to 64-bit data
        // for consistency with the other replies.
        // kSerializer always writes data out in little endian format even for
        // big endian architecture, so changing to 64-bit writes should not
        // cause any backwards compatibiity issues with older SDK or older
        // sensor software.
        //
        // The size field is included in the message size count stored in the size field.
        kTest(kSerializer_BeginRead(serializer, kTypeOf(k64u), kTRUE));

        kTest(kSerializer_Read64s(serializer, &responseId));
        // The 5.1 and newer SDK sends both a GET ADDRESS and GET INFO
        // discovery request. A 5.1 and newer sensor firmware version
        // supports both requests and therefore sends back both replies.
        // In the caller to this function, a reply is assumed to be a GET ADDRESS
        // reply and if it is not, then the reply is parsed as a GET
        // INFO reply.
        // If the order were reversed, then it is valid that the
        // response id is not an GET INFO REPLY so
        // it is not a real error but an expected scenario.
        if (responseId != GO_DISCOVERY_GET_INFO_REPLY)
        {
            // Flush the remaining data in the byte stream following the first
            // three 32-bit integers that were already read.
            // Also free allocated serializer memory.
            kTest(kSerializer_EndRead(serializer));
            kThrow(kERROR_COMMAND);
        }

        kTest(kSerializer_Read64s(serializer, &status));
        kTest(status);

        kTest(kSerializer_Read64s(serializer, &signature));
        kTest(signature == GO_DISOVERY_SIGNATURE);

        // This block of code must not change or else the change will break
        // backwards compatibility with older sensor software.
        {
            kTest(kSerializer_Read16u(serializer, &attrSize));
            // The number of bytes of data up to the properties is expected to never
            // change across SDK versions.
            // This can never change for forwards and backwards compatibility
            // between SDK and sensor code.
            kTest(attrSize == GO_DISCOVERY_GET_INFO_REPLY_ATTR_SIZE);

            kTest(kSerializer_Read32u(serializer, &obj->id));
            kTest(kSerializer_Read32u(serializer, &obj->version));
            kTest(kSerializer_Read64u(serializer, &obj->uptime));
            kTest(kSerializer_ReadByte(serializer, &useDhcp));
            obj->address.useDhcp = (kBool) useDhcp;

            kTest(kSerializer_ReadByte(serializer, &tempByte));
            obj->address.address.version = (kIpVersion) tempByte;
            // The size of this block of code assumes an IP address is sent
            // as 16 bytes from the sensor.
            kAssert(kCountOf(obj->address.address.address) == 16);
            kTest(kSerializer_ReadByteArray(serializer, obj->address.address.address, kCountOf(obj->address.address.address)));

            kTest(kSerializer_Read32u(serializer, &prefixLength));
            obj->address.mask = kIpAddress_FromHost32u(GoDiscoveryExtInfo_SubnetPrefixToMask(prefixLength));

            kTest(kSerializer_ReadByte(serializer, &tempByte));
            obj->address.gateway.version = (kIpVersion) tempByte;
            // Size of IP address has been verified above to be 16 bytes.
            kTest(kSerializer_ReadByteArray(serializer, obj->address.gateway.address, kCountOf(obj->address.gateway.address)));

            kTest(kSerializer_Read16u(serializer, &obj->ports.controlPort));
            kTest(kSerializer_Read16u(serializer, &obj->ports.upgradePort));
            kTest(kSerializer_Read16u(serializer, &obj->ports.healthPort));
            kTest(kSerializer_Read16u(serializer, &obj->ports.dataPort));
            kTest(kSerializer_Read16u(serializer, &obj->ports.webPort));
        }

        kTest(kSerializer_Read8u(serializer, &propertyCount));

        if (propertyCount > 0)
        {
            kTest(GoDiscoveryExtInfo_ReadProperties(msg, serializer, propertyCount));
        }

        // Acceleration info was added to 5.3 sensor software.
        kTest(GoDiscoveryExtInfo_ExtractAccelSensorInfoFromProperty(msg, &accelInfoInProperty));

        // If the acceleration info is found in the properties, then skip the rest of the data
        // in the data stream. Otherwise, try to find the acceleration info in
        // the message data that come after the properties.
        if (!accelInfoInProperty)
        {
            // Sensor software is pre-5.3.
            // Check if SDK is talking to an sensor software version before 5.1 that does
            // not have the extra accelerated sensor data following the properties.
            if (!kSerializer_ReadCompleted(serializer))
            {
                // Sensor software is 5.1 or 5.2.
                kTest(GoDiscoveryExtInfo_ExtractAccelSensorInfoFromMsg(msg, serializer));
            }
            else
            {
                // Older sensor firmware (pre-5.1) doesn't support this field, so initialize
                // this field to a sensible value.
                obj->opMode = GO_DISCOVERY_OP_MODE_NOT_AVAILABLE;

                // Initialize IP version to valid value even though the address may
                // not be meaningful. This prevents errors if code tries to convert the
                // meaningless address to a string.
                obj->accelSensorIpAddress.version = (kIpVersion) kIP_VERSION_4;
            }
        }
    }
    kFinally
    {
        // Automatically flush any unread data.
        // TODO: Older 5.0 code's discovery protocol sends the incorrect size
        //       of the byte stream, which causes the EndRead() call to sometimes fail
        //       because it looks like too much data was read.
        //       Until that older code is no longer supported, suppress error
        //       checking the return code of EndRead().
        kSerializer_EndRead(serializer);

        kEndFinally();
    }

    return kOK;
}

GoFx(k32u) GoDiscoveryExtInfo_Id(GoDiscoveryExtInfo msg)
{
    return GoDiscoveryExtInfo_Content_(msg)->id;
}

GoFx(GoAddressInfo) GoDiscoveryExtInfo_Address(GoDiscoveryExtInfo msg)
{
    return GoDiscoveryExtInfo_Content_(msg)->address;
}

GoFx(GoPortInfo) GoDiscoveryExtInfo_Ports(GoDiscoveryExtInfo msg)
{
    return GoDiscoveryExtInfo_Content_(msg)->ports;
}

GoFx(kVersion) GoDiscoveryExtInfo_Version(GoDiscoveryExtInfo msg)
{
    return GoDiscoveryExtInfo_Content_(msg)->version;
}

GoFx(k64u) GoDiscoveryExtInfo_UpTime(GoDiscoveryExtInfo msg)
{
    return GoDiscoveryExtInfo_Content_(msg)->uptime;
}

GoFx(kSize) GoDiscoveryExtInfo_PropertyCount(GoDiscoveryExtInfo msg)
{
    return kArrayList_Count(GoDiscoveryExtInfo_Content_(msg)->properties);
}

GoFx(const GoDiscoveryProperty*) GoDiscoveryExtInfo_PropertyAt(GoDiscoveryExtInfo msg, kSize index)
{
    if (index >= kArrayList_Count(GoDiscoveryExtInfo_Content_(msg)->properties))
    {
        return kNULL;
    }

    return kArrayList_AtT(GoDiscoveryExtInfo_Content_(msg)->properties, index, GoDiscoveryProperty);
}
GoFx(GoDiscoveryOpMode) GoDiscoveryExtInfo_OpMode(GoDiscoveryExtInfo msg)
{
    return GoDiscoveryExtInfo_Content_(msg)->opMode;
}

GoFx(kIpAddress) GoDiscoveryExtInfo_AccelSensorIpAddress(GoDiscoveryExtInfo msg)
{
    return GoDiscoveryExtInfo_Content_(msg)->accelSensorIpAddress;
}

// DEPRECATED
GoFx(kStatus) GoDiscoveryExtInfo_CompleteModel(GoDiscoveryExtInfo msg, kChar* model, kSize capacity)
{
    // Complete Model is deprecated (since it was ambiguous), provide the original behavior
    // which was to simply return partNumber.
    return GoDiscoveryExtInfo_PartNumber(msg, model, capacity);
}

// Acceleration information was put as part of the extended information
// after the property list in 5.1 and 5.2 sensor software.
GoFx(kStatus) GoDiscoveryExtInfo_ExtractAccelSensorInfoFromMsg(GoDiscoveryExtInfo msg, kSerializer serializer)
{
    kObj(GoDiscoveryExtInfo, msg);
    kByte ipVersion;

    // Get opmode and host ip address even if controller is not an accelerator host.
    // The IP address must be read from stream otherwise it will be read and
    // interpreted as other data.
    kCheck(kSerializer_Read8u(serializer, &obj->opMode));

    kCheck(kSerializer_ReadByte(serializer, &ipVersion));
    obj->accelSensorIpAddress.version = (kIpVersion) ipVersion;
    // The size of an IP address sent by sensor is expected to be 16 bytes and
    // can never change because of backwards and forwards compatibility between
    // SDK and sensor code.
    kAssert(kCountOf(obj->accelSensorIpAddress.address) == 16);
    kCheck(kSerializer_ReadByteArray(serializer, obj->accelSensorIpAddress.address, kCountOf(obj->accelSensorIpAddress.address)));

    return kOK;
}

// This function is used to work with sensor software 5.3 and newer where the
// accelerated sensor information is stored in the properties.
// The information is also still available in the data stream that follows the
// properties, when it was implemented in 5.1 and 5.2 sensor software,
// but the data following the properties is being obsoleted from 5.3 onwards, so
// don't use it unless the SDK is interacting with a 5.1/5.2 sensor software.
// The flag 'foundAccelInfo' indicates whether the properties has
GoFx(kStatus) GoDiscoveryExtInfo_ExtractAccelSensorInfoFromProperty(GoDiscoveryExtInfo msg, kBool* foundAccelInfo)
{
    kObj(GoDiscoveryExtInfo, msg);
    kSize propCount;
    kSize i;
    const GoDiscoveryProperty* propertyPtr;
    kByte ipVersion = 0;

    // Initialize return value.
    *foundAccelInfo = kFALSE;

    propCount = GoDiscoveryExtInfo_PropertyCount(msg);
    for (i = 0; i < propCount; i++)
    {
        propertyPtr = GoDiscoveryExtInfo_PropertyAt(msg, i);
        if (kStrnEquals(propertyPtr->name, "OpMode", kCountOf(propertyPtr->name)))
        {
            kCheck(k8u_Parse(&obj->opMode, propertyPtr->value));
        }
        else if (kStrnEquals(propertyPtr->name, "AccelSensorIpVersion", kCountOf(propertyPtr->name)))
        {
            kCheck(k8u_Parse(&ipVersion, propertyPtr->value));
        }
        else if (kStrnEquals(propertyPtr->name, "AccelSensorIp", kCountOf(propertyPtr->name)))
        {
            // Check that the accelerated sensor IP address is sane.
            if (ipVersion == kIP_VERSION_4)
            {
                kCheck(kIpAddress_Parse(&obj->accelSensorIpAddress, propertyPtr->value));
                *foundAccelInfo = kTRUE;
            }
        }
    }

    return kOK;
}

GoFx(kStatus) GoDiscoveryExtInfo_ReadProperties(GoDiscoveryExtInfo msg, kSerializer serializer, k8u propertyCount)
{
    kObj(GoDiscoveryExtInfo, msg);
    k8u i;
    kStatus exception = kOK;
    k8u nameLength;
    k8u valueLength;
    GoDiscoveryProperty* property = kNULL;

    kTry
    {
        kTest(GoDiscoveryExtInfo_AllocateProperties(msg, propertyCount));

        for (i = 0; i < propertyCount; i++)
        {
            property = kArrayList_AtT(obj->properties, i, GoDiscoveryProperty);

            kTest(kSerializer_Read8u(serializer, &nameLength));
            kTest(kSerializer_ReadCharArray(serializer, property->name, nameLength));
            property->name[nameLength] = 0;

            kTest(kSerializer_Read8u(serializer, &valueLength));
            kTest(kSerializer_ReadCharArray(serializer, property->value, valueLength));
            property->value[valueLength] = 0;
        }
    }
    kCatch(&exception)
    {
        kEndCatch(exception);
    }

    return kOK;
}

GoFx(kStatus) GoDiscoveryExtInfo_PartNumber(GoDiscoveryExtInfo msg, kChar* partNumber, kSize capacity)
{
    kObj(GoDiscoveryExtInfo, msg);
    kSize foundIndex;
    const GoDiscoveryProperty* propertyPtr;

    // PartNumber may not exist in very old sensors (see GOC-12999) and is filled 
    // within the ExtendedId as "FullModel".
    // Only return blank if NEITHER "FullModel" nor "PartNumber" are present.
    if ((foundIndex = GoDiscoveryExtInfo_FindProperty(msg, "FullModel")) == kSIZE_NULL &&
        (foundIndex = GoDiscoveryExtInfo_FindProperty(msg, "PartNumber")) == kSIZE_NULL)
    {
        kCheck(kStrCopy(partNumber, capacity, ""));
    }
    else
    {
        propertyPtr = GoDiscoveryExtInfo_PropertyAt(msg, foundIndex);
        kCheck(kStrCopy(partNumber, capacity, propertyPtr->value));
    }

    return kOK;
}

GoFx(kStatus) GoDiscoveryExtInfo_FillModelNumber(GoDiscoveryExtInfo msg, kChar* fallBackModelNumber, kSize capacity)
{
    kObj(GoDiscoveryExtInfo, msg);
    kSize foundIndex;
    const GoDiscoveryProperty* propertyPtr;

    // Model holds the number XXXX after the Gocator_XXXX and has been filled in as part of
    // ExtendedId since at least 4.1.x.x (~2014) and has been part of discovery info as far
    // back as GOC-3533,3576 (~2015-08). This will be present even if the underlying
    // Id.xml is schemaVersion="1" and lacks ModelNumber itself.
    if ((foundIndex = GoDiscoveryExtInfo_FindProperty(msg, "Model")) == kSIZE_NULL)
    {
        kCheck(kStrCopy(fallBackModelNumber, capacity, ""));
    }
    else
    {
        propertyPtr = GoDiscoveryExtInfo_PropertyAt(msg, foundIndex);
        kCheck(kStrCopy(fallBackModelNumber, capacity, propertyPtr->value));
    }

    return kOK;
 }

GoFx(kStatus) GoDiscoveryExtInfo_ModelNumber(GoDiscoveryExtInfo msg, kChar* modelNumber, kSize capacity)
{
    kObj(GoDiscoveryExtInfo, msg);
    kSize foundIndex;
    const GoDiscoveryProperty* propertyPtr;

    if ((foundIndex = GoDiscoveryExtInfo_FindProperty(msg, "ModelNumber")) == kSIZE_NULL)
    {
        // ModelNumber is part of Id.xml schemaVersion="2" and a gap left unfilled
        // by ExtendedId for sensors with Id.xml schemaVersion="1".
        // If not found -- fall back to the "Model" field using a fill function.
        kCheck(GoDiscoveryExtInfo_FillModelNumber(msg, modelNumber, capacity));
    }
    else
    {
        propertyPtr = GoDiscoveryExtInfo_PropertyAt(msg, foundIndex);
        kCheck(kStrCopy(modelNumber, capacity, propertyPtr->value));
    }

    return kOK;
 }

GoFx(kStatus) GoDiscoveryExtInfo_ModelDisplayName(GoDiscoveryExtInfo msg, kChar* modelDisplayName, kSize capacity)
{
    kObj(GoDiscoveryExtInfo, msg);
    kSize foundIndex;
    const GoDiscoveryProperty* propertyPtr;

    if ((foundIndex = GoDiscoveryExtInfo_FindProperty(msg, "ModelDisplayName")) == kSIZE_NULL)
    {
        // If not found in pre-5.3 versions -- fall back to modelNumber.
        kCheck(GoDiscoveryExtInfo_ModelNumber(msg, modelDisplayName, capacity));
    }
    else
    {
        propertyPtr = GoDiscoveryExtInfo_PropertyAt(msg, foundIndex);
        kCheck(kStrCopy(modelDisplayName, capacity, propertyPtr->value));
    }

    return kOK;
}

// For internal use only to retrieve the LMI model number of the sensor.
GoFx(kStatus) GoDiscoveryExtInfo_Family(GoDiscoveryExtInfo msg, kChar* family, kSize capacity)
{
    kObj(GoDiscoveryExtInfo, msg);
    kSize foundIndex;
    const GoDiscoveryProperty* propertyPtr;

    // Family has been filled in as part of ExtendedId since at least 4.1.x.x (~2014).
    // ExtendedId has been part of discovery info as far back as GOC-3533,3576 (~2015-08).
    // So if Family doesn't exist -- the upstream caller will need to seek other
    // non-discovery based information sources (sucn as GetSystemInfo()).
    if ((foundIndex = GoDiscoveryExtInfo_FindProperty(msg, "Family")) == kSIZE_NULL)
    {
        kCheck(kStrCopy(family, capacity, ""));
    }
    else
    {
        propertyPtr = GoDiscoveryExtInfo_PropertyAt(msg, foundIndex);
        kCheck(kStrCopy(family, capacity, propertyPtr->value));
    }

    return kOK;
 }

GoFx(kSize) GoDiscoveryExtInfo_FindProperty(GoDiscoveryExtInfo msg, const kChar *name)
{
    kSize propCount;
    kSize i;
    const GoDiscoveryProperty* propertyPtr;

    propCount = GoDiscoveryExtInfo_PropertyCount(msg);
    for (i = 0; i < propCount; i++)
    {
        propertyPtr = GoDiscoveryExtInfo_PropertyAt(msg, i);
        if (kStrnEquals(propertyPtr->name, name, kCountOf(propertyPtr->name)))
        {
            return i;
        }
    }

    return kSIZE_NULL;
}

/**
 * @file    GoSerializer.x.h
 *
 * @internal
 * Copyright (C) 2016-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef GO_SDK_SERIALIZER_X_H
#define GO_SDK_SERIALIZER_X_H

#include <kApi/Data/kMap.h>

typedef struct GoSerializerTypeInfo
{
    kType type;                     //type object
    kTypeVersion version;           //type object serialization version information
    k16u id;                        //numeric guid
} GoSerializerTypeInfo;

kDeclareValueEx(Go, GoSerializerTypeInfo, kValue)

typedef struct GoSerializerClass
{
    kSerializerClass base;
    kMap typeToInfo;              //used by writer; maps type pointer to type version info (kMap<kType, GoSerializerTypeInfo>)
    kMap idToType;                //used by reader; maps type id to type version info (kMap<k16u, GoSerializerTypeInfo>)
} GoSerializerClass;

kDeclareClassEx(Go, GoSerializer, kSerializer)

GoFx(kStatus) GoSerializer_VInit(GoSerializer serializer, kType type, kStream stream, kAlloc allocator);
GoFx(kStatus) GoSerializer_VRelease(GoSerializer serializer);

GoFx(kStatus) GoSerializer_VWriteObject(GoSerializer serializer, kObject object);
GoFx(kStatus) GoSerializer_VReadObject(GoSerializer serializer, kObject* object, kAlloc allocator);
GoFx(kStatus) GoSerializer_ReadObjectItem(GoSerializer serializer, kBool* isLast, kObject* item, kAlloc allocator);

GoFx(kStatus) GoSerializer_WriteTypeId(GoSerializer serializer, kType type, kBool isLast, kSerializerObjectSerializeFx* fx);
GoFx(kStatus) GoSerializer_ReadTypeId(GoSerializer serializer, kType* type, kBool* isLast, kSerializerObjectDeserializeFx* fx);

GoFx(kStatus) GoSerializer_ConstructItem(GoSerializer serializer, kObject* item, kType type, kSerializerObjectDeserializeFx initializer, kAlloc alloc);

GoFx(kStatus) GoSerializer_BuildIdToTypeMap(GoSerializer serializer);

#endif

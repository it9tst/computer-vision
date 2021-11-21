/**
 * @file    GoSerializer.c
 *
 * @internal
 * Copyright (C) 2016-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <GoSdk/Internal/GoSerializer.h>
#include <GoSdk/Messages/GoDataSet.h>
#include <GoSdk/GoSdkLib.h>
#include <stdlib.h>

kBeginValueEx(Go, GoSerializerTypeInfo)
    kAddField(GoSerializerTypeInfo, kType, type)
    kAddField(GoSerializerTypeInfo, kPointer, version)
    kAddField(GoSerializerTypeInfo, k16u, id)
kEndValueEx()

kBeginClassEx(Go, GoSerializer)
    kAddVMethod(GoSerializer, kObject, VRelease)
    kAddVMethod(GoSerializer, kSerializer, VInit)
    kAddVMethod(GoSerializer, kSerializer, VWriteObject)
    kAddVMethod(GoSerializer, kSerializer, VReadObject)
kEndClassEx()

GoFx(kStatus) GoSerializer_Construct(GoSerializer* serializer, kStream stream, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback(allocator);
    kStatus status;

    kCheck(kAlloc_GetObject(alloc, kTypeOf(GoSerializer), serializer));

    if (!kSuccess(status = GoSerializer_VInit(*serializer, kTypeOf(GoSerializer), stream, alloc)))
    {
        kAlloc_FreeRef(alloc, serializer);
    }

    return status;
}

GoFx(kStatus) GoSerializer_VInit(GoSerializer serializer, kType type, kStream stream, kAlloc allocator)
{
    kObjR(GoSerializer, serializer);
    kStatus status;

    kCheck(kSerializer_VInit(serializer, type, stream, allocator));

    obj->typeToInfo = kNULL;
    obj->idToType = kNULL;

    kTry
    {
        kTest(kStrCopy(obj->base.format, kCountOf(obj->base.format), GO_SERIALIZATION_FORMAT_NAME));
        kTest(kSerializer_SetVersion(serializer, kNULL, GoSdk_ProtocolVersion()));

        kTest(kMap_Construct(&obj->typeToInfo, kTypeOf(kType), kTypeOf(GoSerializerTypeInfo), 0, allocator));
        kTest(kMap_Construct(&obj->idToType, kTypeOf(k16u), kTypeOf(GoSerializerTypeInfo), 0, allocator));

        kTest(GoSerializer_BuildIdToTypeMap(serializer));
    }
    kCatch(&status)
    {
        GoSerializer_VRelease(serializer);
        kEndCatch(status);
    }

    return kOK;
}

GoFx(kStatus) GoSerializer_VRelease(GoSerializer serializer)
{
    kObj(GoSerializer, serializer);

    kCheck(kObject_Destroy(obj->typeToInfo));
    kCheck(kObject_Destroy(obj->idToType));

    kCheck(kSerializer_VRelease(serializer));

    return kOK;
}

GoFx(kStatus) GoSerializer_VWriteObject(GoSerializer serializer, kObject object)
{
    kSize count;
    kSize i;

    kCheckArgs(!kIsNull(object) && (kObject_Type(object) == kTypeOf(GoDataSet)));

    count = GoDataSet_Count(object);

    for (i = 0; i < count; ++i)
    {
        kObject item = GoDataSet_At(object, i);
        kBool isLast = (i == (count-1));
        kSerializerObjectSerializeFx serializeFx = kNULL;

        kCheck(kSerializer_BeginWrite(serializer, kTypeOf(k32u), kTRUE));
        kCheck(GoSerializer_WriteTypeId(serializer, kObject_Type(item), isLast, &serializeFx));

        kCheck(serializeFx(item, serializer));

        kCheck(kSerializer_EndWrite(serializer));
    }

    kCheck(kSerializer_Flush(serializer));

    return kOK;
}

GoFx(kStatus) GoSerializer_VReadObject(GoSerializer serializer, kObject* object, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback(allocator);
    GoDataSet output = kNULL;
    kObject item = kNULL;
    kBool isLast = kFALSE;
    kStatus status;

    kTry
    {
        kTest(GoDataSet_Construct(&output, alloc));

        while (!isLast)
        {
            kTest(GoSerializer_ReadObjectItem(serializer, &isLast, &item, alloc));

            kTest(GoDataSet_Add(output, item));
            item = kNULL;
        }

        *object = output;
    }
    kCatch(&status)
    {
        kObject_Dispose(item);
        kObject_Dispose(output);

        kEndCatch(status);
    }

    return kOK;
}

GoFx(kStatus) GoSerializer_WriteTypeId(GoSerializer serializer, kType type, kBool isLast, kSerializerObjectSerializeFx* fx)
{
    kObj(GoSerializer, serializer);
    GoSerializerTypeInfo info;
    k16u typeId;

    if (!kSuccess(kMap_FindT(obj->typeToInfo, &type, &info)))
    {
        info.type = type;
        kCheck(kSerializer_FindCompatibleVersion(serializer, type, &info.version));
        info.id = (k16u) atoi(kType_VersionGuid(info.type, info.version));

        kCheck(kMap_AddT(obj->typeToInfo, &type, &info));
    }

    typeId = (k16u) ((isLast << 15) | info.id);

    kCheck(kSerializer_Write16u(serializer, typeId));

    *fx = (kSerializerObjectSerializeFx) kType_VersionSerializeFx(info.type, info.version);

    return kOK;
}

GoFx(kStatus) GoSerializer_ReadTypeId(GoSerializer serializer, kType* type, kBool* isLast, kSerializerObjectDeserializeFx* fx)
{
    kObj(GoSerializer, serializer);
    GoSerializerTypeInfo info;
    k16u typeId;
    k16u id;

    kCheck(kSerializer_Read16u(serializer, &typeId));

    id = typeId & 0x7FFF;
    *isLast = (typeId >> 15);

    kCheck(kMap_FindT(obj->idToType, &id, &info));

    *type = info.type;
    *fx = (kSerializerObjectDeserializeFx) kType_VersionDeserializeFx(info.type, info.version);

    return kOK;
}

GoFx(kStatus) GoSerializer_ConstructItem(GoSerializer serializer, kObject* item, kType type, kSerializerObjectDeserializeFx initializer, kAlloc alloc)
{
    kStatus status;

    kCheck(kAlloc_GetObject(alloc, type, item));

    if (!kSuccess(status = initializer(*item, serializer, alloc)))
    {
        kAlloc_FreeRef(alloc, item);
    }

    return status;
}

GoFx(kStatus) GoSerializer_BuildIdToTypeMap(GoSerializer serializer)
{
    kObj(GoSerializer, serializer);
    kAssembly assembly = kAssemblyOf(GoSdk);
    kSize typeCount = kAssembly_TypeCount(assembly);
    GoSerializerTypeInfo info;
    kSize i, j;

    kCheck(kMap_Clear(obj->idToType));

    for (i = 0; i < typeCount; ++i)
    {
        kType type = kAssembly_TypeAt(assembly, i);
        kSize versionCount = kType_VersionCount(type);

        for (j = 0; j < versionCount; ++j)
        {
           kTypeVersion version = kType_VersionAt(type, j);

           if (kStrEquals(kType_VersionFormat(type, version), GO_SERIALIZATION_FORMAT_NAME))
           {
               const kChar* guid = kType_VersionGuid(type, version);

               info.type = type;
               info.version = version;
               info.id = (k16u) atoi(guid);

               kCheck(kMap_AddT(obj->idToType, &info.id, &info));
           }
        }
    }

    return kOK;
}

GoFx(kStatus) GoSerializer_ReadObjectItem(GoSerializer serializer, kBool* isLast, kObject* item, kAlloc allocator)
{
    kType itemType = kNULL;
    kSerializerObjectDeserializeFx deserializeFx = kNULL;

    // Read and store the message attributes data length ("attrSize")
    // to determine how many attributes to read.
    // The size field is included in the message size count stored in the size field.
    //
    // If this fails, the EndRead() could assert so skip the EndRead() if BeginRead() fails.
    kCheck(kSerializer_BeginRead(serializer, kTypeOf(k32u), kTRUE));

    kTry
    {
        kTest(GoSerializer_ReadTypeId(serializer, &itemType, isLast, &deserializeFx));

        kTest(GoSerializer_ConstructItem(serializer, item, itemType, deserializeFx, allocator));
    }
    kFinally
    {
        // Automatically flush any unread data and free serializer read section.
        // Ignore errors.
        kSerializer_EndRead(serializer);

        kEndFinally();
    }

    return kOK;
}

/** 
 * @file    kDat5Serializer.cpp
 *
 * @internal
 * Copyright (C) 2008-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <kApi/Io/kDat5Serializer.h>
#include <kApi/Data/kArrayList.h>
#include <kApi/Data/kMap.h>
#include <kApi/Io/kFile.h>
#include <kApi/Threads/kLock.h>
#include <stdio.h>

kBeginValueEx(k, xkDat5SerializerTypeInfo)
    kAddField(xkDat5SerializerTypeInfo, kType, type)
    kAddField(xkDat5SerializerTypeInfo, kPointer, version)
    kAddField(xkDat5SerializerTypeInfo, k32u, id)
kEndValueEx()

kBeginFullClassEx(k, kDat5Serializer)    
    kAddVMethod(kDat5Serializer, kObject, VRelease)
    kAddVMethod(kDat5Serializer, kSerializer, VInit)
    kAddPrivateVMethod(kDat5Serializer, kSerializer, VReset)
    kAddVMethod(kDat5Serializer, kSerializer, VWriteObject)
    kAddVMethod(kDat5Serializer, kSerializer, VReadObject)
    kAddPrivateVMethod(kDat5Serializer, kSerializer, VWriteType)
    kAddPrivateVMethod(kDat5Serializer, kSerializer, VReadType)
kEndFullClassEx()

kFx(kStatus) xkDat5Serializer_InitStatic()
{
    kStaticObj(kDat5Serializer); 

    kCheck(kLock_ConstructEx(&sobj->lock, xkLOCK_OPTION_PRIORITY_INHERITANCE, kNULL)); 
    kCheck(kMap_Construct(&sobj->guidToInfo, kTypeOf(k32u), kTypeOf(xkDat5SerializerTypeInfo), 256, kNULL)); 
    kCheck(kMap_Construct(&sobj->guidToCompressor, kTypeOf(kCompressionType), kTypeOf(kPointer), 0, kNULL));

    kCheck(kAssembly_AddLoadHandler(xkDat5Serializer_OnAssemblyLoad, kNULL)); 
    kCheck(kAssembly_AddUnloadHandler(xkDat5Serializer_OnAssemblyUnload, kNULL)); 

    return kOK; 
}

kFx(kStatus) xkDat5Serializer_ReleaseStatic()
{
    kStaticObj(kDat5Serializer); 

    kCheck(kAssembly_RemoveLoadHandler(xkDat5Serializer_OnAssemblyLoad, kNULL)); 
    kCheck(kAssembly_RemoveUnloadHandler(xkDat5Serializer_OnAssemblyUnload, kNULL)); 
    
    kCheck(kDestroyRef(&sobj->guidToCompressor)); 
    kCheck(kDestroyRef(&sobj->guidToInfo)); 
    kCheck(kDestroyRef(&sobj->lock)); 

    return kOK; 
}

kFx(kStatus) xkDat5Serializer_OnAssemblyLoad(kPointer receiver, kAssembly assembly, void* args)
{
    kStaticObj(kDat5Serializer); 
    kSize i, j; 

    kCheck(kLock_Enter(sobj->lock)); 

    kTry
    {
        //build serialization guid-to-info map
        if (kObject_IsShared(sobj->guidToInfo))
        {
            kMap clone = kNULL; 

            kTest(kObject_Clone(&clone, sobj->guidToInfo, kNULL)); 
            kTest(kObject_Destroy(sobj->guidToInfo)); 

            sobj->guidToInfo = clone; 
        }
        
        for (i = 0; i < kAssembly_TypeCount(assembly); ++i)
        {
            kType type = kAssembly_TypeAt(assembly, i); 

            for (j = 0; j < kType_VersionCount(type); ++j)
            {
                kTypeVersion version = kType_VersionAt(type, j); 

                if (kStrEquals(kType_VersionFormat(type, version), xkDAT5_FORMAT))
                {
                    xkDat5SerializerTypeInfo info; 
                    k32u typeId, typeVersionId; 

                    info.type = type; 
                    info.version = version; 

                    if (sscanf(kType_VersionGuid(type, version), "%u-%u", &typeId, &typeVersionId) == 2)
                    {
                        info.id = xkDat5Guid_Create(typeId, typeVersionId); 

                        kTest(kMap_ReplaceT(sobj->guidToInfo, &info.id, &info)); 
                    }
                }
            }
        }

        //build compression guid-to-type map
        if (kObject_IsShared(sobj->guidToCompressor))
        {
            kMap clone = kNULL; 

            kTest(kObject_Clone(&clone, sobj->guidToCompressor, kNULL)); 
            kTest(kObject_Destroy(sobj->guidToCompressor)); 

            sobj->guidToCompressor = clone; 
        }

        for (i = 0; i < kAssembly_TypeCount(assembly); ++i)
        {
            kType type = kAssembly_TypeAt(assembly, i); 
            
            if (kType_Implements(type, kTypeOf(kCompressor)))
            {
                kCompressionType guid = kCast(kCompressorVTable*, kType_IVTable(type, kTypeOf(kCompressor)))->VCompressionType();

                kTest(kMap_ReplaceT(sobj->guidToCompressor, &guid, &type)); 
            }
        }
    }
    kFinally
    {
        kLock_Exit(sobj->lock);
        kEndFinally(); 
    }

    return kOK; 
}

kFx(kStatus) xkDat5Serializer_OnAssemblyUnload(kPointer receiver, kAssembly assembly, void* args)
{
    kStaticObj(kDat5Serializer); 
    kSize i, j; 

    kCheck(kLock_Enter(sobj->lock)); 

    kTry
    {
        //clear serialization guid-to-info map
        if (kObject_IsShared(sobj->guidToInfo))
        {
            kMap clone = kNULL; 

            kTest(kObject_Clone(&clone, sobj->guidToInfo, kNULL)); 
            kTest(kObject_Destroy(sobj->guidToInfo)); 

            sobj->guidToInfo = clone; 
        }
       
        for (i = 0; i < kAssembly_TypeCount(assembly); ++i)
        {
            kType type = kAssembly_TypeAt(assembly, i); 
            kMapItem item = kNULL; 

            for (j = 0; j < kType_VersionCount(type); ++j)
            {
                kTypeVersion version = kType_VersionAt(type, j); 

                if (kStrEquals(kType_VersionFormat(type, version), xkDAT5_FORMAT))
                {                   
                    k32u typeId, typeVersionId, typeGuid; 

                    if (sscanf(kType_VersionGuid(type, version), "%u-%u", &typeId, &typeVersionId) == 2)
                    {
                        typeGuid = xkDat5Guid_Create(typeId, typeVersionId); 

                        if (kSuccess(kMap_FindItemT(sobj->guidToInfo, &typeGuid, &item)))
                        {
                            kTest(kMap_RemoveItem(sobj->guidToInfo, item)); 
                        }
                    }
                }
            }
        }

        //clear compression guid-to-type map
        if (kObject_IsShared(sobj->guidToCompressor))
        {
            kMap clone = kNULL; 

            kTest(kObject_Clone(&clone, sobj->guidToCompressor, kNULL)); 
            kTest(kObject_Destroy(sobj->guidToCompressor)); 

            sobj->guidToCompressor = clone; 
        }

        for (i = 0; i < kAssembly_TypeCount(assembly); ++i)
        {
            kType type = kAssembly_TypeAt(assembly, i); 
            
            if (kType_Implements(type, kTypeOf(kCompressor)))
            {
                kCompressionType guid = kCast(kCompressorVTable*, kType_IVTable(type, kTypeOf(kCompressor)))->VCompressionType();
                kMapItem item = kNULL; 

                if (kSuccess(kMap_FindItemT(sobj->guidToCompressor, &guid, &item)))
                {
                    kTest(kMap_RemoveItem(sobj->guidToCompressor, item));
                }
            }
        }
    }
    kFinally
    {
        kLock_Exit(sobj->lock);
        kEndFinally(); 
    }

    return kOK; 
}

kFx(kStatus) xkDat5Serializer_GetTypeLookups(kMap* guidToInfo, kMap* guidToCompressor)
{
    kStaticObj(kDat5Serializer); 
    
    kCheck(kLock_Enter(sobj->lock)); 

    kTry
    {
        kTest(kObject_Share(sobj->guidToInfo)); 
        *guidToInfo = sobj->guidToInfo; 

        kTest(kObject_Share(sobj->guidToCompressor)); 
        *guidToCompressor = sobj->guidToCompressor; 
    }
    kFinally
    {
        kLock_Exit(sobj->lock);
        kEndFinally(); 
    }

    return kOK; 
}

kFx(kStatus) kDat5Serializer_Construct(kDat5Serializer* serializer, kStream stream, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback(allocator);
    kStatus status; 

    kCheck(kAlloc_GetObject(alloc, kTypeOf(kDat5Serializer), serializer)); 

    if (!kSuccess(status = kDat5Serializer_VInit(*serializer, kTypeOf(kDat5Serializer), stream, alloc)))
    {
        kAlloc_FreeRef(alloc, serializer); 
    }

    return status; 
} 

kFx(kStatus) kDat5Serializer_VInit(kDat5Serializer serializer, kType type, kStream stream, kAlloc allocator)
{
    kObjR(kDat5Serializer, serializer);
    kStatus status; 
    
    kCheck(kSerializer_VInit(serializer, type, stream, allocator)); 

    obj->objectDepth = 0;
    obj->guidToInfo = kNULL;
    obj->guidToCompressor = kNULL;
    obj->typeToInfo = kNULL;
    obj->currentCompressionAlgorithm = kCOMPRESSION_TYPE_NULL;
    obj->currentCompressionPreset = 0;
    obj->compressStream = kNULL;
    obj->currentDecompressionAlgorithm = kCOMPRESSION_TYPE_NULL;
    obj->decompressStream = kNULL;

    kStrCopy(obj->base.format, kCountOf(obj->base.format), xkDAT5_FORMAT);     

    kTry
    {
        kTest(xkDat5Serializer_GetTypeLookups(&obj->guidToInfo, &obj->guidToCompressor)); 
        kTest(kMap_Construct(&obj->typeToInfo, kTypeOf(kType), kTypeOf(xkDat5SerializerTypeInfo), 64, allocator)); 
    }
    kCatch(&status)
    {
        kDat5Serializer_VRelease(serializer); 
        kEndCatch(status); 
    }

    return kOK; 
}

kFx(kStatus) kDat5Serializer_VRelease(kDat5Serializer serializer)
{
    kObj(kDat5Serializer, serializer); 
        
    kCheck(kDestroyRef(&obj->compressStream)); 
    kCheck(kDestroyRef(&obj->decompressStream)); 

    kCheck(kObject_Destroy(obj->guidToInfo)); 
    kCheck(kObject_Destroy(obj->guidToCompressor)); 
    kCheck(kObject_Destroy(obj->typeToInfo)); 

    kCheck(kSerializer_VRelease(serializer)); 

    return kOK; 
}

kFx(kBool) xkDat5Serializer_VReset(kDat5Serializer serializer)
{
    kObj(kDat5Serializer, serializer); 
    
    kCheck(kMap_Clear(obj->typeToInfo));      

    obj->objectDepth = 0; 

    kCheck(kDestroyRef(&obj->compressStream)); 
    obj->currentCompressionAlgorithm = kCOMPRESSION_TYPE_NULL; 
    obj->currentCompressionPreset = 0; 

    kCheck(kDestroyRef(&obj->decompressStream)); 
    obj->currentDecompressionAlgorithm = kCOMPRESSION_TYPE_NULL;

    kCheck(kSerializer_VReset(serializer)); 

    return kOK; 
}

kFx(kStatus) kDat5Serializer_WriteData(kDat5Serializer serializer, kObject object, const kChar* label)
{
    kObj(kDat5Serializer, serializer); 
    kText16 dummyLabel = { 0 }; 
    kStream storedWriteStream = obj->base.writeStream;
    kBool isOutermost = obj->objectDepth == 0; 
    kBool shouldCompress = isOutermost && (obj->currentCompressionAlgorithm != kCOMPRESSION_TYPE_NULL);  
    kStatus exception;

    obj->objectDepth++; 

    kTry
    {
        if (isOutermost)
        {
            if (shouldCompress)
            {
                kTest(kSerializer_WriteByte(serializer, xkDAT5_SERIALIZER_CMD_COMPRESSED_OBJECT));
                kTest(xkDat5Serializer_BeginCompression(serializer));
            }
            else
            {
                kTest(kSerializer_WriteByte(serializer, xkDAT5_SERIALIZER_CMD_OBJECT));
            }
        }

        if (kIsNull(object))
        {
            kTest(kSerializer_Write32s(serializer, xkDAT5_SERIALIZER_CMD_NULL)); 
            kTest(kSerializer_Write32u(serializer, 0)); 
        }
        else
        {
            kType type = kObject_Type(object); 
            kTypeVersion version = kNULL; 
            
            kTest(kSerializer_WriteType(serializer, type, &version));        

            kTest(kSerializer_InvokeObjectSerializer(serializer, type, version, object)); 

            kTest(kSerializer_Write32u(serializer, 0));    //kData version

            if (!kIsNull(label))
            {
                if (!kSuccess(kStrCopy(dummyLabel, sizeof(dummyLabel), label))) {/* don't care */}
            }

            kTest(kSerializer_WriteCharArray(serializer, dummyLabel, sizeof(dummyLabel)));  
        }

        if (shouldCompress)
        {
            kTest(xkDat5Serializer_EndCompression(serializer));
        }

        obj->objectDepth--; 
    }
    kCatch(&exception)
    {
        //precaution, in case of errors in compression; ensure write stream is restored
        obj->base.writeStream = storedWriteStream; 

        if (isOutermost)
        {
            kSerializer_Reset(serializer);
        }

        kEndCatch(exception);
    }

    if (isOutermost && (kArrayList_Count(obj->base.writeSections) == 0))
    {
        kCheck(kSerializer_Flush(serializer));
    }

    return kOK; 
}

kFx(kStatus) kDat5Serializer_VWriteObject(kDat5Serializer serializer, kObject object)
{
    return kDat5Serializer_WriteData(serializer, object, kNULL);
}

kFx(kStatus) kDat5Serializer_ReadData(kDat5Serializer serializer, kObject* object, kChar* label, kSize capacity, kAlloc allocator)
{
    kObj(kDat5Serializer, serializer); 
    kAlloc alloc = kAlloc_Fallback(allocator); 
    kType type = kNULL; 
    kTypeVersion version = kNULL; 
    kByte objectCommand = 0;
    kObject output = kNULL;
    kText16 dummyLabel; 
    k32u dataVersion; 
    kStream storedReadStream = obj->base.readStream;
    kBool isOutermost = obj->objectDepth == 0; 
    kStatus exception; 
  
    obj->objectDepth++; 

    kTry
    {
        obj->base.readAlloc = alloc; 

        if (obj->objectDepth == 1)
        {
            kTest(kSerializer_ReadByte(serializer, &objectCommand)); 

            if (objectCommand == xkDAT5_SERIALIZER_CMD_COMPRESSED_OBJECT)
            {
               kTest(xkDat5Serializer_BeginDecompression(serializer));
            }
            else
            {
                kTestState(objectCommand == xkDAT5_SERIALIZER_CMD_OBJECT);
            }
        }

        kTest(kSerializer_ReadType(serializer, &type, &version)); 

        if (kIsNull(type))
        {
            *object = kNULL; 
        }
        else
        {
            kTest(xkSerializer_DeserializeObject(serializer, type, version, &output, alloc));

            kTest(kSerializer_Read32u(serializer, &dataVersion));
            kTest(kSerializer_ReadCharArray(serializer, dummyLabel, kCountOf(dummyLabel))); 

            if (!kIsNull(label) && capacity != 0)
            {
                kTest(kStrCopy(label, capacity, dummyLabel));
            }
        }

        if (objectCommand == xkDAT5_SERIALIZER_CMD_COMPRESSED_OBJECT)
        {
            kTest(xkDat5Serializer_EndDecompression(serializer));
        }

        *object = output;

        obj->objectDepth--; 
    }
    kCatch(&exception)
    {
        //precaution, in case of errors in compression; ensure read stream is restored
        obj->base.readStream = storedReadStream; 

        kDisposeRef(&output);

        if (isOutermost)
        {
            kSerializer_Reset(serializer);
        }

        kEndCatch(exception);
    }

    return kOK; 
}

kFx(kStatus) kDat5Serializer_VReadObject(kDat5Serializer serializer, kObject* object, kAlloc allocator)
{
    return kDat5Serializer_ReadData(serializer, object, kNULL, 0, allocator);
}

kFx(kStatus) xkDat5Serializer_VWriteType(kDat5Serializer serializer, kType type, kTypeVersion* version)
{
    kObj(kDat5Serializer, serializer); 
    xkDat5SerializerTypeInfo info; 
    k32u typeId, typeVersionId; 

    if (!kSuccess(kMap_FindT(obj->typeToInfo, &type, &info)))
    {    
        kCheck(kSerializer_FindCompatibleVersion(serializer, type, version)); 

        if (sscanf(kType_VersionGuid(type, *version), "%u-%u", &typeId, &typeVersionId) != 2)
        {
            return kERROR_FORMAT; 
        }

        info.type = type; 
        info.version = *version;        
        info.id = xkDat5Guid_Create(typeId, typeVersionId); 

        kCheck(kMap_AddT(obj->typeToInfo, &type, &info));         
    }
    else
    {
        typeId = xkDat5Guid_Type(info.id); 
        typeVersionId = xkDat5Guid_Version(info.id); 
    }

    kCheck(kSerializer_Write32u(serializer, typeId)); 
    kCheck(kSerializer_Write32u(serializer, typeVersionId)); 

    *version = info.version; 

    return kOK; 
}

kFx(kStatus) xkDat5Serializer_VReadType(kDat5Serializer serializer, kType* type, kTypeVersion* version)
{
    kObj(kDat5Serializer, serializer); 
    const xkDat5SerializerTypeInfo* info = kNULL; 
    kMapItem item = kNULL; 
    k32s typeId; 
    k32s typeVersionId; 
    k32u guid; 

    kCheck(kSerializer_Read32s(serializer, &typeId)); 
    kCheck(kSerializer_Read32s(serializer, &typeVersionId)); 

    if (typeId == xkDAT5_SERIALIZER_CMD_NULL)
    {
        //null is treated as special case
        *type = kNULL; 
        *version = kNULL; 

        return kOK; 
    }
    else if (typeId == xkDAT5_SERIALIZER_CMD_REF)
    {
        //references not supported
        return kERROR_FORMAT; 
    }

    guid = xkDat5Guid_Create((k32u)typeId, (k32u)typeVersionId); 

    kCheck(kMap_FindItemT(obj->guidToInfo, &guid, &item));
    info = kMap_ValueT(obj->guidToInfo, item, xkDat5SerializerTypeInfo); 

    *type = info->type; 
    *version = info->version; 
  
    return kOK; 
}

kFx(kStatus) xkDat5Serializer_WriteTypeWithoutVersion(kDat5Serializer serializer, kType type, kTypeVersion* version)
{
    kObj(kDat5Serializer, serializer); 
    xkDat5SerializerTypeInfo info; 
    k32u typeId, typeVersionId; 

    if (!kSuccess(kMap_FindT(obj->typeToInfo, &type, &info)))
    {    
        kCheck(kSerializer_FindCompatibleVersion(serializer, type, version)); 

        if (sscanf(kType_VersionGuid(type, *version), "%u-%u", &typeId, &typeVersionId) != 2)
        {
            return kERROR_FORMAT; 
        }

        info.type = type; 
        info.version = *version;        
        info.id = xkDat5Guid_Create(typeId, typeVersionId); 

        kCheck(kMap_AddT(obj->typeToInfo, &type, &info));         
    }
    else
    {
        typeId = xkDat5Guid_Type(info.id); 
        typeVersionId = xkDat5Guid_Version(info.id); 
    }

    kCheck(kSerializer_Write32u(serializer, typeId)); 

    *version = info.version; 

    return kOK; 
}

kFx(kStatus) xkDat5Serializer_ReadTypeExplicitVersion(kDat5Serializer serializer, k32u typeVersionId, kType* type, kTypeVersion* version)
{
    kObj(kDat5Serializer, serializer); 
    const xkDat5SerializerTypeInfo* info = kNULL; 
    kMapItem item = kNULL; 
    k32s typeId; 
    k32u guid; 

    kCheck(kSerializer_Read32s(serializer, &typeId)); 
    
    if (typeId == xkDAT5_SERIALIZER_CMD_NULL)
    {
        //null is treated as special case
        *type = kNULL; 
        *version = kNULL; 

        return kOK; 
    }
    else if (typeId == xkDAT5_SERIALIZER_CMD_REF)
    {
        //references not supported
        return kERROR_FORMAT; 
    }

    guid = xkDat5Guid_Create((k32u)typeId, typeVersionId); 

    kCheck(kMap_FindItemT(obj->guidToInfo, &guid, &item));
    info = kMap_ValueT(obj->guidToInfo, item, xkDat5SerializerTypeInfo); 

    *type = info->type; 
    *version = info->version; 
  
    return kOK; 
}


kFx(kStatus) xkDat5Serializer_BeginCompression(kDat5Serializer serializer)
{
    kObj(kDat5Serializer, serializer);

    kCheck(kSerializer_BeginWrite(serializer, kTypeOf(k16u), kTRUE));
    kCheck(kSerializer_Write8u(serializer, (k8u)obj->currentCompressionAlgorithm));
    kCheck(kSerializer_EndWrite(serializer));

    kCheck(xkSerializer_FlushEx(serializer, kFALSE));

    //replace underlying stream with compression stream
    obj->base.writeStream = obj->compressStream;

    return kOK;
}

kFx(kStatus) xkDat5Serializer_EndCompression(kDat5Serializer serializer)
{
    kObj(kDat5Serializer, serializer);

    //flush serializer to compression stream, but don't flush the compression stream itself
    kCheck(xkSerializer_FlushEx(serializer, kFALSE));

    //finish writing the current compression stream
    kCheck(kCompressor_FinishWrite(obj->compressStream)); 
                
    //restore underlying stream 
    obj->base.writeStream = obj->base.stream;

    return kOK;
}

kFx(kStatus) xkDat5Serializer_BeginDecompression(kDat5Serializer serializer)
{
    kObj(kDat5Serializer, serializer);
    kCompressionType algorithm = kCOMPRESSION_TYPE_NULL;
    kType decompressorType = kNULL; 
    k8u temp8u = 0;

    //read out decompression configuration
    kCheck(kSerializer_BeginRead(serializer, kTypeOf(k16u), kTRUE)); 
    kCheck(kSerializer_Read8u(serializer, &temp8u));            
    kCheck(kSerializer_EndRead(serializer)); 

    algorithm = temp8u;

    //change compressor, if necessary
    if (obj->currentDecompressionAlgorithm != algorithm)
    {
        if (algorithm != kCOMPRESSION_TYPE_NULL)
        {
            kCheck(kMap_FindT(obj->guidToCompressor, &algorithm, &decompressorType)); 

            //destroy the previous decompressor, if a new & different decompressor has been specified            
            if (!kIsNull(obj->decompressStream))
            {
                if (!kObject_Is(obj->decompressStream, decompressorType))
                {
                    kDestroyRef(&obj->decompressStream);
                }
            }

            //construct a new decompressor, if needed
            if (kIsNull(obj->decompressStream))
            {
                kCheck(kCast(kCompressorVTable*, kType_IVTable(decompressorType, kTypeOf(kCompressor)))->VConstruct(&obj->decompressStream, obj->base.stream, kFALSE, 0, kObject_Alloc(serializer)));
            }
        }
    }

    obj->currentDecompressionAlgorithm = algorithm; 

    //replace underlying stream with compression stream
    obj->base.readStream = obj->decompressStream;

    return kOK;
}

kFx(kStatus) xkDat5Serializer_EndDecompression(kDat5Serializer serializer)
{
    kObj(kDat5Serializer, serializer);

    //finish reading the current compression stream
    kCheck(kCompressor_FinishRead(obj->decompressStream)); 
                
    //restore underlying stream 
    obj->base.readStream = obj->base.stream;

    return kOK;
}

kFx(kStatus) kDat5Serializer_EnableCompression(kDat5Serializer serializer, kCompressionType algorithm, k32s level)
{
    kObj(kDat5Serializer, serializer); 
    kType compressorType = kNULL;

    //check args
    if (algorithm != kCOMPRESSION_TYPE_NULL)
    {
        kType compressorType = kNULL; 
        kVersion requestedVersion, supportedVersion; 

        //check that the requested compression algorithm is available
        kCheck(kMap_FindT(obj->guidToCompressor, &algorithm, &compressorType));
        
        //check that the requested compression algorithm is supported in the platform version that we're aiming for compatibility with
        requestedVersion = kSerializer_FindRequestedMinVersion(serializer, compressorType); 
        supportedVersion = kCast(kCompressorVTable*, kType_IVTable(compressorType, kTypeOf(kCompressor)))->VRequiredVersion();

        kCheckTrue(requestedVersion >= supportedVersion, kERROR_VERSION);
    }

    //change compressor, if necessary
    if ((obj->currentCompressionAlgorithm != algorithm) || (obj->currentCompressionPreset != level))
    {        
        if (algorithm != kCOMPRESSION_TYPE_NULL)
        {
            kCheck(kMap_FindT(obj->guidToCompressor, &algorithm, &compressorType)); 

            //destroy the previous compressor, if a new & different compressor has been specified            
            if (!kIsNull(obj->compressStream))
            {
                if (!kObject_Is(obj->compressStream, compressorType) || (level != obj->currentCompressionPreset))
                {
                    kDestroyRef(&obj->compressStream);
                }
            }

            //construct a new compressor, if needed
            if (kIsNull(obj->compressStream))
            {
                kCheck(kCast(kCompressorVTable*, kType_IVTable(compressorType, kTypeOf(kCompressor)))->VConstruct(&obj->compressStream, obj->base.stream, kTRUE, level, kObject_Alloc(serializer)));
            }

            obj->currentCompressionPreset = level;
        }

        obj->currentCompressionAlgorithm = algorithm;
    }    
    
    return kOK; 
} 

kFx(kStatus) kDat5Serializer_SaveCompressed(kObject object, const kChar* filePath, kCompressionType algorithm, k32s level)
{
    kSerializer writer = kNULL; 
    kFile stream = kNULL; 

    kTry
    {        
        kTest(kFile_Construct(&stream, filePath, kFILE_MODE_WRITE, kNULL)); 

        kTest(kDat5Serializer_Construct(&writer, stream, kNULL)); 
        kTest(kDat5Serializer_EnableCompression(writer, algorithm, level)); 

        kTest(kSerializer_WriteObject(writer, object)); 
    }
    kFinally
    {
        kCheck(kObject_Destroy(writer)); 
        kCheck(kObject_Destroy(stream)); 

        kEndFinally(); 
    }

    return kOK; 
}
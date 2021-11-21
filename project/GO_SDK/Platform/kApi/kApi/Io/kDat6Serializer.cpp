/** 
 * @file    kDat6Serializer.cpp
 *
 * @internal
 * Copyright (C) 2012-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <kApi/Io/kDat6Serializer.h>
#include <kApi/Data/kArrayList.h>
#include <kApi/Data/kMap.h>
#include <kApi/Io/kFile.h>
#include <kApi/Threads/kLock.h>

kBeginValueEx(k, xkDat6SerializerTypeInfo)
    kAddField(xkDat6SerializerTypeInfo, kType, type)
    kAddField(xkDat6SerializerTypeInfo, kPointer, version)
    kAddField(xkDat6SerializerTypeInfo, k16u, id)
kEndValueEx()

kBeginFullClassEx(k, kDat6Serializer)    
    kAddVMethod(kDat6Serializer, kObject, VRelease)
    kAddVMethod(kDat6Serializer, kSerializer, VInit)
    kAddVMethod(kDat6Serializer, kSerializer, VSetSizeEncoding)
    kAddPrivateVMethod(kDat6Serializer, kSerializer, VReset)
    kAddVMethod(kDat6Serializer, kSerializer, VWriteObject)
    kAddVMethod(kDat6Serializer, kSerializer, VReadObject)
    kAddPrivateVMethod(kDat6Serializer, kSerializer, VWriteType)
    kAddPrivateVMethod(kDat6Serializer, kSerializer, VReadType)
kEndFullClassEx()

kFx(kStatus) xkDat6Serializer_InitStatic()
{
    kStaticObj(kDat6Serializer); 

    kCheck(kLock_ConstructEx(&sobj->lock, xkLOCK_OPTION_PRIORITY_INHERITANCE, kNULL)); 
    kCheck(kMap_Construct(&sobj->guidToInfo, kTypeOf(kText64), kTypeOf(xkDat6SerializerTypeInfo), 256, kNULL)); 
    kCheck(kMap_Construct(&sobj->guidToCompressor, kTypeOf(kCompressionType), kTypeOf(kPointer), 0, kNULL));

    kCheck(kAssembly_AddLoadHandler(xkDat6Serializer_OnAssemblyLoad, kNULL)); 
    kCheck(kAssembly_AddUnloadHandler(xkDat6Serializer_OnAssemblyUnload, kNULL)); 

    return kOK; 
}

kFx(kStatus) xkDat6Serializer_ReleaseStatic()
{
    kStaticObj(kDat6Serializer); 

    kCheck(kAssembly_RemoveLoadHandler(xkDat6Serializer_OnAssemblyLoad, kNULL)); 
    kCheck(kAssembly_RemoveUnloadHandler(xkDat6Serializer_OnAssemblyUnload, kNULL)); 
    
    kCheck(kDestroyRef(&sobj->guidToCompressor)); 
    kCheck(kDestroyRef(&sobj->guidToInfo)); 
    kCheck(kDestroyRef(&sobj->lock)); 

    return kOK; 
}

kFx(kStatus) xkDat6Serializer_OnAssemblyLoad(kPointer receiver, kAssembly assembly, void* args)
{
    kStaticObj(kDat6Serializer); 
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
            kText64 guid; 

            for (j = 0; j < kType_VersionCount(type); ++j)
            {
                kTypeVersion version = kType_VersionAt(type, j); 

                if (kStrEquals(kType_VersionFormat(type, version), xkDAT6_FORMAT))
                {
                    xkDat6SerializerTypeInfo info; 

                    info.type = type; 
                    info.version = version; 
                    info.id = 0; 

                    kTest(kStrCopy(guid, kCountOf(guid), kType_VersionGuid(type, version))); 
                    kTest(kMap_ReplaceT(sobj->guidToInfo, guid, &info)); 
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

kFx(kStatus) xkDat6Serializer_OnAssemblyUnload(kPointer receiver, kAssembly assembly, void* args)
{
    kStaticObj(kDat6Serializer); 
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

                if (kStrEquals(kType_VersionFormat(type, version), xkDAT6_FORMAT))
                {                   
                    if (kSuccess(kMap_FindItemT(sobj->guidToInfo, kType_VersionGuid(type, version), &item)))
                    {
                        kTest(kMap_RemoveItem(sobj->guidToInfo, item)); 
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

kFx(kStatus) xkDat6Serializer_GetTypeLookups(kMap* guidToInfo, kMap* guidToCompressor)
{
    kStaticObj(kDat6Serializer); 
    
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

kFx(kStatus) kDat6Serializer_Construct(kDat6Serializer* serializer, kStream stream, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback(allocator);
    kStatus status; 

    kCheck(kAlloc_GetObject(alloc, kTypeOf(kDat6Serializer), serializer)); 

    if (!kSuccess(status = kDat6Serializer_VInit(*serializer, kTypeOf(kDat6Serializer), stream, alloc)))
    {
        kAlloc_FreeRef(alloc, serializer); 
    }

    return status; 
} 

kFx(kStatus) kDat6Serializer_VInit(kDat6Serializer serializer, kType type, kStream stream, kAlloc allocator)
{
    kObjR(kDat6Serializer, serializer);
    kStatus status; 
    
    kCheck(kSerializer_VInit(serializer, type, stream, allocator)); 

    kStrCopy(obj->base.format, kCountOf(obj->base.format), xkDAT6_FORMAT);     

    obj->objectSizeEncoding = K_POINTER_SIZE; 
    obj->synchronized = kFALSE;
    obj->dictionaryEnabled = kTRUE; 
    obj->objectDepth = 0;
    obj->guidToInfo = kNULL;
    obj->guidToCompressor = kNULL;
    obj->readerInfo = kNULL;
    obj->typeToInfo = kNULL;
    obj->currentCompressionAlgorithm = 0;
    obj->currentCompressionPreset = 0;
    obj->compressStream = kNULL;
    obj->currentDecompressionAlgorithm = 0;
    obj->decompressStream = kNULL;
    
    kTry
    {
        kTest(xkDat6Serializer_GetTypeLookups(&obj->guidToInfo, &obj->guidToCompressor)); 
        kTest(kArrayList_Construct(&obj->readerInfo, kTypeOf(xkDat6SerializerTypeInfo), 64, allocator)); 
        kTest(kMap_Construct(&obj->typeToInfo, kTypeOf(kType), kTypeOf(xkDat6SerializerTypeInfo), 64, allocator)); 
    }
    kCatch(&status)
    {
        kDat6Serializer_VRelease(serializer); 
        kEndCatch(status); 
    }

    return kOK; 
}

kFx(kStatus) kDat6Serializer_VRelease(kDat6Serializer serializer)
{
    kObj(kDat6Serializer, serializer); 
       
    kCheck(kDestroyRef(&obj->compressStream)); 
    kCheck(kDestroyRef(&obj->decompressStream)); 

    kCheck(kObject_Destroy(obj->guidToInfo)); 
    kCheck(kObject_Destroy(obj->guidToCompressor)); 
    kCheck(kObject_Destroy(obj->readerInfo)); 
    kCheck(kObject_Destroy(obj->typeToInfo));  

    kCheck(kSerializer_VRelease(serializer)); 

    return kOK; 
}

kFx(kStatus) kDat6Serializer_VSetSizeEncoding(kDat6Serializer serializer, k32u byteCount)
{
    kObj(kDat6Serializer, serializer); 

    obj->objectSizeEncoding = byteCount; 

    return kSerializer_VSetSizeEncoding(serializer, byteCount); 
}

kFx(kBool) xkDat6Serializer_VReset(kDat6Serializer serializer)
{
    kObj(kDat6Serializer, serializer); 
    
    kCheck(kMap_Clear(obj->typeToInfo));      
    kCheck(kArrayList_Clear(obj->readerInfo));

    obj->objectDepth = 0; 
    obj->synchronized = kFALSE; 

    kCheck(kDestroyRef(&obj->compressStream)); 
    obj->currentCompressionAlgorithm = kCOMPRESSION_TYPE_NULL; 
    obj->currentCompressionPreset = 0; 

    kCheck(kDestroyRef(&obj->decompressStream)); 
    obj->currentDecompressionAlgorithm = kCOMPRESSION_TYPE_NULL;

    kCheck(kSerializer_VReset(serializer)); 

    return kOK; 
}

kFx(kStatus) kDat6Serializer_VWriteObject(kDat6Serializer serializer, kObject object)
{
    kObj(kDat6Serializer, serializer); 
    kStream storedWriteStream = obj->base.writeStream;
    k32u storedSizeEncoding = obj->base.sizeEncoding; 
    kBool isOutermost = obj->objectDepth == 0; 
    kStatus exception;

    obj->objectDepth++;

    if (isOutermost)
    { 
        obj->base.sizeEncoding = obj->objectSizeEncoding; 
    }

    kTry
    {
        if (!obj->dictionaryEnabled && isOutermost)
        {
            obj->synchronized = kFALSE;
        }

        if (!obj->synchronized)
        {
            kTest(kSerializer_WriteByte(serializer, xkDAT6_SERIALIZER_CMD_SYNC));
            kTest(xkDat6Serializer_WriteSync(serializer));
            kTest(kMap_Clear(obj->typeToInfo));

            obj->synchronized = kTRUE;
        }

        if (kIsNull(object))
        {
            kTest(kSerializer_WriteByte(serializer, xkDAT6_SERIALIZER_CMD_NULL));
        }
        else
        {
            kType type = kObject_Type(object);
            kTypeVersion version = kNULL;
            kBool shouldCompress = isOutermost && (obj->currentCompressionAlgorithm != kCOMPRESSION_TYPE_NULL);

            if (shouldCompress)
            {
                kTest(kSerializer_WriteByte(serializer, xkDAT6_SERIALIZER_CMD_COMPRESSED_OBJECT));
                kTest(xkDat6Serializer_BeginCompression(serializer));
            }
            else
            {
                kTest(kSerializer_WriteByte(serializer, xkDAT6_SERIALIZER_CMD_OBJECT));
            }

            kTest(kSerializer_WriteType(serializer, type, &version));

            kTest(kSerializer_InvokeObjectSerializer(serializer, type, version, object));

            if (shouldCompress)
            {
                kTest(xkDat6Serializer_EndCompression(serializer));
            }
        }

        obj->objectDepth--; 
    }
    kCatchEx(&exception)
    {        
        if (isOutermost)
        {
            kSerializer_Reset(serializer);
        }

        kEndCatchEx(exception);
    }
    kFinallyEx
    {
        if (isOutermost)
        { 
            obj->base.sizeEncoding = storedSizeEncoding;
        }
        obj->base.writeStream = storedWriteStream; 

        kEndFinallyEx();
    }


    if (isOutermost && (kArrayList_Count(obj->base.writeSections) == 0))
    {
        kCheck(kSerializer_Flush(serializer));
    }

    return kOK; 
}

kFx(kStatus) kDat6Serializer_VReadObject(kDat6Serializer serializer, kObject* object, kAlloc allocator)
{
    kObj(kDat6Serializer, serializer); 
    kAlloc alloc = kAlloc_Fallback(allocator); 
    kByte command = 0; 
    kObject output = kNULL;
    kStream storedReadStream = obj->base.readStream;
    k32u storedSizeEncoding = obj->base.sizeEncoding; 
    kBool isOutermost = obj->objectDepth == 0; 
    kStatus exception;

    obj->objectDepth++;
    
    if (isOutermost)
    { 
        obj->base.sizeEncoding = obj->objectSizeEncoding; 
    }

    kTry
    {
        obj->base.readAlloc = alloc; 

        kTest(kSerializer_ReadByte(serializer, &command)); 

        if (command == xkDAT6_SERIALIZER_CMD_SYNC)
        {
            kTest(xkDat6Serializer_ReadSync(serializer));             
            kTest(kSerializer_ReadByte(serializer, &command)); 
        }

        if ((command == xkDAT6_SERIALIZER_CMD_OBJECT) || (command == xkDAT6_SERIALIZER_CMD_COMPRESSED_OBJECT))
        {
            kType type = kNULL; 
            kTypeVersion version = kNULL; 

            if (command == xkDAT6_SERIALIZER_CMD_COMPRESSED_OBJECT)
            {
               kTest(xkDat6Serializer_BeginDecompression(serializer));
            }

            kTest(kSerializer_ReadType(serializer, &type, &version));        

            kTest(xkSerializer_DeserializeObject(serializer, type, version, &output, alloc));

            if (command == xkDAT6_SERIALIZER_CMD_COMPRESSED_OBJECT)
            {
               kTest(xkDat6Serializer_EndDecompression(serializer));               
            }
        }
        else if (command != xkDAT6_SERIALIZER_CMD_NULL)
        {
            kThrow(kERROR_STREAM); 
        }

        *object = output;

        obj->objectDepth--; 
    }
    kCatchEx(&exception)
    {
        kDisposeRef(&output);

        if (isOutermost)
        {
            kSerializer_Reset(serializer);
        }

        kEndCatchEx(exception);
    }
    kFinallyEx
    {
        if (isOutermost)
        { 
            obj->objectSizeEncoding = obj->base.sizeEncoding;   //may have been updated via sync operation
            obj->base.sizeEncoding = storedSizeEncoding;
        }
        obj->base.readStream = storedReadStream; 

        kEndFinallyEx();
    }
    
    return kOK; 
}

kFx(kStatus) xkDat6Serializer_VWriteType(kDat6Serializer serializer, kType type, kTypeVersion* version)
{
    kObj(kDat6Serializer, serializer); 
    xkDat6SerializerTypeInfo info; 

    if (!kSuccess(kMap_FindT(obj->typeToInfo, &type, &info)))
    {    
        k16u dictionaryId = (k16u) (kMap_Count(obj->typeToInfo) + 1); 
        const kChar* guid = kNULL; 
        k8u size; 

        kCheck(kSerializer_FindCompatibleVersion(serializer, type, version)); 

        info.type = type; 
        info.version = *version; 
        info.id = dictionaryId; 

        kCheck(kMap_AddT(obj->typeToInfo, &type, &info)); 

        guid = kType_VersionGuid(type, *version); 
        size = (k8u) kStrLength(guid); 

        kCheck(kSerializer_Write16u(serializer, 0)); 
        kCheck(kSerializer_Write16u(serializer, dictionaryId)); 
        kCheck(kSerializer_Write8u(serializer, size)); 
        kCheck(kSerializer_WriteCharArray(serializer, guid, size)); 
    }
    else
    {
        kCheck(kSerializer_Write16u(serializer, info.id)); 
    }
       
    *version = info.version; 

    return kOK; 
}

kFx(kStatus) xkDat6Serializer_VReadType(kDat6Serializer serializer, kType* type, kTypeVersion* version)
{
    kObj(kDat6Serializer, serializer); 
    xkDat6SerializerTypeInfo* info = kNULL; 
    kMapItem item = kNULL; 
    k16u dictionaryId; 

    kCheck(kSerializer_Read16u(serializer, &dictionaryId)); 

    if (dictionaryId == 0)
    {
        kText64 guid; 
        k16u expectedId = (k16u) (kArrayList_Count(obj->readerInfo) + 1); 
        k8u size; 

        kCheck(kSerializer_Read16u(serializer, &dictionaryId));         
        kCheck(dictionaryId == expectedId); 

        kCheck(kSerializer_Read8u(serializer, &size)); 
        kCheck(size < kCountOf(guid)); 

        kCheck(kSerializer_ReadCharArray(serializer, guid, size)); 
        guid[size] = 0; 

        kCheck(kMap_FindItemT(obj->guidToInfo, guid, &item)); 
        kCheck(kArrayList_AddT(obj->readerInfo, kMap_ValueT(obj->guidToInfo, item, xkDat6SerializerTypeInfo))); 
    }
    
    if (dictionaryId <= kArrayList_Count(obj->readerInfo))
    {
        info = kArrayList_AtT(obj->readerInfo, (kSize)(dictionaryId-1), xkDat6SerializerTypeInfo); 

        *type = info->type; 
        *version = info->version; 
    }
    else
    {
        return kERROR_NOT_FOUND; 
    }   
  
    return kOK; 
}

kFx(kStatus) xkDat6Serializer_WriteSync(kDat6Serializer serializer)
{
    kObj(kDat6Serializer, serializer); 

    kCheck(kSerializer_BeginWrite(serializer, kTypeOf(k16u), kTRUE)); 

    kCheck(kSerializer_WriteByte(serializer, (kByte)obj->base.sizeEncoding)); 

    kCheck(kSerializer_EndWrite(serializer)); 

    return kOK; 
}

kFx(kStatus) xkDat6Serializer_ReadSync(kDat6Serializer serializer)
{
    kObj(kDat6Serializer, serializer); 
    kByte sizeEncoding; 

    kCheck(kSerializer_BeginRead(serializer, kTypeOf(k16u), kTRUE)); 

    kCheck(kSerializer_ReadByte(serializer, &sizeEncoding)); 
               
    kCheck(kSerializer_EndRead(serializer)); 

    if ((sizeEncoding != 4) && (sizeEncoding != 8))
    {
        return kERROR_FORMAT; 
    }

    obj->base.sizeEncoding = sizeEncoding; 

    kCheck(kArrayList_Clear(obj->readerInfo)); 
   
    return kOK; 
}

kFx(kStatus) xkDat6Serializer_BeginCompression(kDat6Serializer serializer)
{
    kObj(kDat6Serializer, serializer);

    kCheck(kSerializer_BeginWrite(serializer, kTypeOf(k16u), kTRUE));
    kCheck(kSerializer_Write8u(serializer, (k8u)obj->currentCompressionAlgorithm));
    kCheck(kSerializer_EndWrite(serializer));

    //flush serializer to underlying stream, but don't flush the underlying stream itself
    kCheck(xkSerializer_FlushEx(serializer, kFALSE));

    //replace underlying stream with compression stream
    obj->base.writeStream = obj->compressStream;

    return kOK;
}

kFx(kStatus) xkDat6Serializer_EndCompression(kDat6Serializer serializer)
{
    kObj(kDat6Serializer, serializer);

    //flush serializer to compression stream, but don't flush the compression stream itself
    kCheck(xkSerializer_FlushEx(serializer, kFALSE));

    //finish writing the current compression stream
    kCheck(kCompressor_FinishWrite(obj->compressStream)); 
                
    //restore underlying stream 
    obj->base.writeStream = obj->base.stream;

    return kOK;
}

kFx(kStatus) xkDat6Serializer_BeginDecompression(kDat6Serializer serializer)
{
    kObj(kDat6Serializer, serializer);
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

kFx(kStatus) xkDat6Serializer_EndDecompression(kDat6Serializer serializer)
{
    kObj(kDat6Serializer, serializer);

    //finish reading the current compression stream
    kCheck(kCompressor_FinishRead(obj->decompressStream)); 
                
    //restore underlying stream 
    obj->base.readStream = obj->base.stream;

    return kOK;
}

kFx(kStatus) kDat6Serializer_EnableDictionary(kDat6Serializer serializer, kBool enable)
{
    kObj(kDat6Serializer, serializer); 

    obj->dictionaryEnabled = enable; 

    return kOK; 
}

kFx(kStatus) kDat6Serializer_EnableCompression(kDat6Serializer serializer, kCompressionType algorithm, k32s level)
{
    kObj(kDat6Serializer, serializer); 
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

kFx(kStatus) kDat6Serializer_SaveCompressed(kObject object, const kChar* filePath, kCompressionType algorithm, k32s level)
{
    kSerializer writer = kNULL; 
    kFile stream = kNULL; 

    kTry
    {        
        kTest(kFile_Construct(&stream, filePath, kFILE_MODE_WRITE, kNULL)); 

        kTest(kDat6Serializer_Construct(&writer, stream, kNULL)); 
        kTest(kDat6Serializer_EnableCompression(writer, algorithm, level)); 

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

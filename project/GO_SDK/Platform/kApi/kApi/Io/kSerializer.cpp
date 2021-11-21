/** 
 * @file    kSerializer.cpp
 *
 * @internal
 * Copyright (C) 2005-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <kApi/Io/kSerializer.h>
#include <kApi/Io/kDat6Serializer.h>
#include <kApi/Io/kFile.h>
#include <kApi/Data/kArrayList.h>
#include <kApi/Data/kMap.h>

/*
* kCompressor interaface
*/

kBeginInterfaceEx(k, kCompressor) 
    //interface methods
    kAddPrivateVMethod(kCompressor, kCompressor, VCompressionType)
    kAddPrivateVMethod(kCompressor, kCompressor, VRequiredVersion)
    kAddPrivateVMethod(kCompressor, kCompressor, VConstruct)
    kAddPrivateVMethod(kCompressor, kCompressor, VFinishWrite)
    kAddPrivateVMethod(kCompressor, kCompressor, VFinishRead)
kEndInterfaceEx() 

/*
 * kSerializer class
*/

kBeginValueEx(k, xkSerializerWriteSection)
    kAddField(xkSerializerWriteSection, kPointer, buffer) 
    kAddField(xkSerializerWriteSection, kSize, start)
    kAddField(xkSerializerWriteSection, kType, type)
    kAddField(xkSerializerWriteSection, kBool, includeSize)
kEndValueEx()

kBeginClassEx(k, kSerializer)
    kAddVMethod(kSerializer, kObject, VRelease)
    kAddVMethod(kSerializer, kSerializer, VInit)
    kAddVMethod(kSerializer, kSerializer, VCanAutoFlush)
    kAddVMethod(kSerializer, kSerializer, VSetSizeEncoding)
    kAddVMethod(kSerializer, kSerializer, VReset)
    kAddVMethod(kSerializer, kSerializer, VWriteObject)
    kAddVMethod(kSerializer, kSerializer, VReadObject) 
    kAddVMethod(kSerializer, kSerializer, VCanWriteType)
    kAddVMethod(kSerializer, kSerializer, VWriteType)
    kAddVMethod(kSerializer, kSerializer, VReadType)
kEndClassEx()

kFx(kStatus) kSerializer_Construct(kSerializer* serializer, kStream stream, kType serializerType, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback(allocator);
    kType type = kIsNull(serializerType) ? kTypeOf(kSerializer) : serializerType; 
    kStatus status;    

    kCheck(kAlloc_GetObject(alloc, type, serializer)); 

    if (!kSuccess(status = kType_VTableT(type, kSerializer)->VInit(*serializer, type, stream, alloc)))
    {
        kAlloc_FreeRef(alloc, serializer); 
    }

    return status; 
} 

kFx(kStatus) kSerializer_LoadObject(kObject* object, kType serializerType, const kChar* filePath, kAlloc readAlloc)
{
    kFile stream = kNULL; 
    kSerializer reader = kNULL; 

    kCheckArgs(!kIsNull(serializerType)); 

    kTry
    {        
        kTest(kFile_Construct(&stream, filePath, kFILE_MODE_READ, kNULL)); 
        kTest(kFile_SetReadBuffer(stream, xkSERIALIZER_DEFAULT_BUFFER_SIZE)); 

        kTest(kSerializer_Construct(&reader, stream, serializerType, kNULL)); 
        kTest(kSerializer_ReadObject(reader, object, readAlloc));
    }
    kFinally
    {
        kCheck(kObject_Destroy(reader)); 
        kCheck(kObject_Destroy(stream)); 

        kEndFinally(); 
    }

    return kOK; 
}

kFx(kStatus) kSerializer_SaveObject(kObject object, kType serializerType, const kChar* filePath)
{
    kSerializer writer = kNULL; 
    kFile stream = kNULL; 

    kCheckArgs(!kIsNull(serializerType)); 

    kTry
    {        
        kTest(kFile_Construct(&stream, filePath, kFILE_MODE_WRITE, kNULL)); 

        kTest(kSerializer_Construct(&writer, stream, serializerType, kNULL)); 

        kTest(kSerializer_WriteObject(writer, object)); 
        kTest(kSerializer_Flush(writer)); 
    }
    kFinally
    {
        kCheck(kObject_Destroy(writer)); 
        kCheck(kObject_Destroy(stream)); 

        kEndFinally(); 
    }

    return kOK; 
}

kFx(kStatus) kSerializer_VInit(kSerializer serializer, kType type, kStream stream, kAlloc allocator)
{
    kObjR(kSerializer, serializer);
    kStatus status; 
    
    kCheck(kObject_Init(serializer, type, allocator)); 

    obj->stream = stream; 
    obj->readStream = stream; 
    obj->writeStream = stream;
    obj->readAlloc = kNULL;
    obj->endianness = kENDIANNESS_LITTLE; 
    obj->swap = kEndianness_ShouldReverse(obj->endianness); 
    obj->sizeEncoding = 4; 
    obj->format[0] = 0;
    obj->formatVersion = k32U_NULL; 
    obj->assemblyVersions = kNULL;
    obj->activeBuffers = kNULL;
    obj->currentBuffer = kNULL;
    obj->freeBuffers = kNULL;
    obj->bufferSize = xkSERIALIZER_DEFAULT_BUFFER_SIZE; 
    obj->writeSections = kNULL;
    obj->readSections = kNULL;
  
    kTry
    {
        kTest(kMap_Construct(&obj->assemblyVersions, kTypeOf(kAssembly), kTypeOf(kVersion), 16, allocator)); 

        kTest(kArrayList_Construct(&obj->writeSections, kTypeOf(xkSerializerWriteSection), 16, allocator)); 
        kTest(kArrayList_Construct(&obj->readSections, kTypeOf(k64u), 16, allocator)); 

        kTest(xkSerializer_AddBuffer(serializer)); 
    }
    kCatch(&status)
    {
        kSerializer_VRelease(serializer); 
        kEndCatch(status); 
    }

    return kOK; 
}

kFx(kStatus) kSerializer_VRelease(kSerializer serializer)
{
    kObj(kSerializer, serializer);
    xkSerializerBuffer* buffer = kNULL; 

    kCheck(xkSerializer_ClearBuffers(serializer)); 
 
    buffer = obj->freeBuffers; 

    while (!kIsNull(buffer))
    {
        xkSerializerBuffer* next = buffer->next; 
        
        kCheck(kObject_FreeMem(serializer, buffer)); 

        buffer = next; 
    }

    kCheck(kObject_Destroy(obj->assemblyVersions)); 
    kCheck(kObject_Destroy(obj->readSections)); 
    kCheck(kObject_Destroy(obj->writeSections)); 
        
    kCheck(kObject_VRelease(serializer)); 

    return kOK; 
}

kFx(kStatus) xkSerializer_FlushEx(kSerializer serializer, kBool flushStream)
{
    kObj(kSerializer, serializer);
  
    kCheck(xkSerializer_FlushBuffers(serializer)); 
    kCheck(xkSerializer_AddBuffer(serializer)); 

    if (flushStream)
    {
        kCheck(kStream_Flush(obj->writeStream));
    }

    return kOK; 
}

kFx(kStatus) kSerializer_VReset(kSerializer serializer)
{
    kObj(kSerializer, serializer);

    kCheck(xkSerializer_ClearBuffers(serializer)); 
    kCheck(xkSerializer_AddBuffer(serializer)); 

    kCheck(kArrayList_Clear(obj->readSections)); 
    kCheck(kArrayList_Clear(obj->writeSections)); 

    return kOK; 
}

kFx(kStatus) xkSerializer_DeserializeObject(kSerializer serializer, kType type, kTypeVersion version, kObject* object, kAlloc alloc)
{
    kObject output = kNULL; 
    kStatus exception;

    kTry
    {
        //FSS-1148: Update object deserialization to support C++ constructors 
        if (kType_VersionHasLegacyDeserializer(type, version))
        {
            kSerializerObjectDeserializeFx deserializeFx = (kSerializerObjectDeserializeFx) kType_VersionDeserializeFx(type, version);
            kStatus status;

            kTest(kAlloc_GetObject(alloc, type, &output)); 

            if (!kSuccess(status = deserializeFx(output, serializer, alloc)))    
            {
                kAlloc_FreeRef(alloc, &output); 
                kThrow(status); 
            }
        }
        else
        {
            kSerializerObjectDeserializeExFx deserializeFx = (kSerializerObjectDeserializeExFx) kType_VersionDeserializeFx(type, version);

            kTest(kType_FrameworkConstructor(type)(&output, alloc)); 

            kTest(deserializeFx(output, serializer));
        }

        *object = output;
    }
    kCatch(&exception)
    {
        kObject_Dispose(output);

        kEndCatch(exception);
    }

    return kOK;
}

kFx(kStatus) kSerializer_WriteItems(kSerializer serializer, kType type, kTypeVersion version, const void* items, kSize count)
{
    kObj(kSerializer, serializer);

    if (kType_IsReference(type))
    {
        kSize i; 

        for (i = 0; i < count; ++i)
        {
            kCheck(kSerializer_WriteObject(obj, ((kObject*)items)[i])); 
        }
    }
    else
    {
        kCheck(kSerializer_InvokeValueSerializer(serializer, type, version, items, count)); 
    }

    return kOK; 
}

kFx(kStatus) kSerializer_ReadItems(kSerializer serializer, kType type, kTypeVersion version, void* items, kSize count)
{
    kObj(kSerializer, serializer);
   
    if (kType_IsReference(type))
    {
        kSize i; 

        for (i = 0; i < count; ++i)
        {
            kCheck(kSerializer_ReadObject(serializer, &((kObject*)items)[i], obj->readAlloc)); 
        }
    }
    else
    {
        kCheck(kSerializer_InvokeValueDeserializer(serializer, type, version, items, count)); 
    }

    return kOK; 
}

kFx(kStatus) kSerializer_BeginWrite(kSerializer serializer, kType sizeType, kBool includeSize)
{
    kObj(kSerializer, serializer);
    xkSerializerWriteSection* section = kNULL; 

    if      (sizeType == kTypeOf(k8u))       kCheck(kSerializer_Write8u(serializer, 0)); 
    else if (sizeType == kTypeOf(k16u))      kCheck(kSerializer_Write16u(serializer, 0)); 
    else if (sizeType == kTypeOf(k32u))      kCheck(kSerializer_Write32u(serializer, 0)); 
    else if (sizeType == kTypeOf(k64u))      kCheck(kSerializer_Write64u(serializer, 0)); 
    else                                     return kERROR_PARAMETER; 

    kCheck(kArrayList_AddCount(obj->writeSections, 1)); 
    section = (xkSerializerWriteSection*) kArrayList_Last(obj->writeSections); 

    section->buffer = obj->currentBuffer; 
    section->start = obj->currentBuffer->written - kType_Size(sizeType); 
    section->type = sizeType; 
    section->includeSize = includeSize; 

    return kOK; 
}

kFx(kStatus) kSerializer_EndWrite(kSerializer serializer)
{
    kObj(kSerializer, serializer);

    kCheckState(kArrayList_Count(obj->writeSections) > 0); 

    xkSerializerWriteSection* section = (xkSerializerWriteSection*) kArrayList_Last(obj->writeSections); 
    kByte* writeAddress = (kByte*) kPointer_ByteOffset(section->buffer->data, (kSSize)section->start);
    xkSerializerBuffer* buffer = kNULL; 
    kSize size = 0; 

    buffer = section->buffer; 
    size = (section->buffer->written - section->start); 
    
    buffer = buffer->next; 

    while (buffer != obj->currentBuffer->next)
    {
        size += buffer->written;
        buffer = buffer->next; 
    }

    if (!section->includeSize)
    {
        size -= kType_Size(section->type); 
    }

    if (section->type == kTypeOf(k8u))       
    {
        k8u sizeField = (k8u)(size); 

        kCheckState(size <= k8U_MAX); 

        *(k8u*) writeAddress = sizeField; 
    }
    else if (section->type == kTypeOf(k16u))      
    {
        k16u sizeField = (k16u) size; 

        kCheckState(size <= k16U_MAX); 

        xkSerializer_ReorderCopyPrimitive(serializer, writeAddress, &sizeField, 2); 
    }
    else if (section->type == kTypeOf(k32u))      
    {
        k32u sizeField = (k32u) size; 

        kCheckState(size <= k32U_MAX); 

        xkSerializer_ReorderCopyPrimitive(serializer, writeAddress, &sizeField, 4); 
    }
    else if (section->type == kTypeOf(k64u))      
    {
        k64u sizeField = (k64u) size; 

        xkSerializer_ReorderCopyPrimitive(serializer, writeAddress, &sizeField, 8); 
    }
    else
    {
        return kERROR_FORMAT; 
    }

    kCheck(kArrayList_RemoveCount(obj->writeSections, 1)); 

    return kOK; 
}

kFx(kStatus) kSerializer_BeginRead(kSerializer serializer, kType sizeType, kBool includeSize)
{
    kObj(kSerializer, serializer);
    k64u readSize = 0; 
    k64u readEnd = 0; 

    if (sizeType == kTypeOf(k8u))       
    {
        k8u sizeField; 

        kCheck(kSerializer_Read8u(serializer, &sizeField)); 
        
        readSize = sizeField; 
    }
    else if (sizeType == kTypeOf(k16u))      
    {
        k16u sizeField; 

        kCheck(kSerializer_Read16u(serializer, &sizeField)); 
        
        readSize = sizeField; 
    }
    else if (sizeType == kTypeOf(k32u))      
    {
        k32u sizeField; 

        kCheck(kSerializer_Read32u(serializer, &sizeField)); 
        
        readSize = sizeField; 
    }
    else if (sizeType == kTypeOf(k64u))      
    {
        k64u sizeField; 

        kCheck(kSerializer_Read64u(serializer, &sizeField)); 
        
        readSize = sizeField; 
    }
    else
    {
        return kERROR_PARAMETER; 
    }

    if (includeSize)
    {
        readSize -= kType_Size(sizeType); 
    }
    
    readEnd = kStream_BytesRead(obj->readStream) + readSize; 
    
    kCheck(kArrayList_AddCount(obj->readSections, 1));     

    *(k64u*) kArrayList_Last(obj->readSections) = readEnd; 

    return kOK; 
}

kFx(kStatus) kSerializer_EndRead(kSerializer serializer)
{
    kObj(kSerializer, serializer);
    k64u readLocation = kStream_BytesRead(obj->readStream); 
    k64u readSection; 
    kSize advance = 0; 
   
    kAssert(kArrayList_Count(obj->readSections) > 0); 
    kCheckState(kArrayList_Count(obj->readSections) > 0); 

    readSection = kPointer_ReadAs(kArrayList_Last(obj->readSections), k64u); 

    kCheck(kArrayList_RemoveCount(obj->readSections, 1));   

    kCheckState(readSection >= readLocation); 
        
    advance = (kSize) (readSection - readLocation); 

    if (advance > 0)
    {
        kCheck(kSerializer_AdvanceRead(serializer, advance)); 
    }

    return kOK; 
}

kFx(kStatus) xkSerializer_AdvanceReadImpl(kSerializer serializer, kSize offset)
{
    kObj(kSerializer, serializer);
    kByte bitBucket[128]; 
    kSize remaining = offset; 
   
    while (remaining > 0)
    {
        kSize readAmount = kMin_(remaining, sizeof(bitBucket)); 

        kCheck(kStream_Read(obj->readStream, bitBucket, readAmount)); 
        
        remaining -= readAmount; 
    }

    return kOK; 
}

kFx(kStatus) kSerializer_SetVersion(kSerializer serializer, kAssembly assembly, kVersion version)
{
    kObj(kSerializer, serializer);
    
    if (kIsNull(assembly))
    {
        obj->formatVersion = version; 
    }
    else
    {
        kCheck(kMap_ReplaceT(obj->assemblyVersions, &assembly, &version)); 
    }

    return kOK; 
}

kFx(kStatus) xkSerializer_AddBuffer(kSerializer serializer)
{
    kObj(kSerializer, serializer);
    xkSerializerBuffer* next = kNULL; 
    
    if (kIsNull(obj->freeBuffers))
    {
        kCheck(xkSerializer_AllocateBuffer(serializer));        
    }

    if (kIsNull(obj->activeBuffers))
    {
        obj->activeBuffers = obj->freeBuffers; 
    }
    else
    {
        next = obj->currentBuffer->next; 
        obj->currentBuffer->next = obj->freeBuffers; 
    }
    
    obj->currentBuffer = obj->freeBuffers; 
    obj->freeBuffers = obj->freeBuffers->next; 
    obj->currentBuffer->next = next;

    obj->currentBuffer->written = 0; 

    return kOK; 
}

kFx(kStatus) xkSerializer_AllocateBuffer(kSerializer serializer)
{
    kObj(kSerializer, serializer);
    kSize headerSize = sizeof(xkSerializerBuffer); 
    kSize dataOffset = kSize_Align(headerSize, kALIGN_ANY); 
    kSize size = dataOffset + obj->bufferSize; 
    xkSerializerBuffer* buffer = kNULL; 
    
    kCheck(kObject_GetMem(serializer, size, &buffer)); 

    buffer->capacity = obj->bufferSize; 
    buffer->written = 0; 
    buffer->data = ((kByte*)buffer) + dataOffset; 

    buffer->next = obj->freeBuffers; 
    obj->freeBuffers = buffer; 

    return kOK; 
}

kFx(kStatus) xkSerializer_InsertHeader(kSerializer serializer)
{
    kObj(kSerializer, serializer);
    xkSerializerBuffer* next = obj->activeBuffers; 
    
    if (kIsNull(obj->freeBuffers))
    {
        kCheck(xkSerializer_AllocateBuffer(serializer));        
    }

    obj->activeBuffers = obj->freeBuffers; 
    obj->freeBuffers = obj->freeBuffers->next; 
    obj->activeBuffers->next = next; 
 
    obj->currentBuffer = obj->activeBuffers; 
    obj->currentBuffer->written = 0; 

    return kOK; 
}

kFx(kStatus) xkSerializer_ClearBuffers(kSerializer serializer)
{
    kObj(kSerializer, serializer);

    while (!kIsNull(obj->activeBuffers))
    {
        xkSerializerBuffer* next = obj->activeBuffers->next; 

        obj->activeBuffers->next = obj->freeBuffers; 
        obj->freeBuffers = obj->activeBuffers; 
        obj->activeBuffers = next; 
    }

    obj->currentBuffer = kNULL; 

    return kOK; 
}

kFx(kStatus) xkSerializer_FlushBuffers(kSerializer serializer)
{
    kObj(kSerializer, serializer);

    while (!kIsNull(obj->activeBuffers))
    {
        xkSerializerBuffer* next = obj->activeBuffers->next; 

        kCheck(kStream_Write(obj->writeStream, obj->activeBuffers->data, obj->activeBuffers->written)); 

        obj->activeBuffers->next = obj->freeBuffers; 
        obj->freeBuffers = obj->activeBuffers;

        obj->activeBuffers = next; 
    }

    obj->currentBuffer = kNULL; 
   
    return kOK; 
}

kFx(kVersion) kSerializer_FindRequestedMinVersion(kSerializer serializer, kType type)
{
    kObj(kSerializer, serializer);
    kAssembly assembly = kType_Assembly(type); 
    kVersion requestedVersion = k32U_NULL;

    if (obj->formatVersion != k32U_NULL)
    {
        requestedVersion = obj->formatVersion; 
    }
    else if (!kSuccess(kMap_FindT(obj->assemblyVersions, &assembly, &requestedVersion)))
    {
        requestedVersion = k32U_NULL;   //use the highest available version
    }

    return requestedVersion; 
}

kFx(kStatus) kSerializer_FindCompatibleVersion(kSerializer serializer, kType type, kTypeVersion* version)
{
    kObj(kSerializer, serializer);
    kAssembly assembly = kType_Assembly(type); 
    kVersion requestedVersion; 
    kSSize versionCount = (kSSize) kType_VersionCount(type); 
    kSSize i = 0; 

    if (obj->formatVersion != k32U_NULL)
    {
        requestedVersion = obj->formatVersion; 
    }
    else if (!kSuccess(kMap_FindT(obj->assemblyVersions, &assembly, &requestedVersion)))
    {
        requestedVersion = k32U_NULL;   //use the highest available version
    }

    //type versions are stored by increasing format version; 
    //can start at end of list and work backwards to find a compatible match.
    for (i = versionCount-1; i >= 0; --i)
    {
        kTypeVersion typeVer = kType_VersionAt(type, (kSize)i); 

        if (kStrEquals(obj->format, kType_VersionFormat(type, typeVer)) && (kType_VersionFormatVersion(type, typeVer) <= requestedVersion))
        {
            *version = typeVer; 
            return kOK; 
        }
    }

    return kERROR_NOT_FOUND; 
}

kFx(kStatus) xkSerializer_WritePrimitives(kSerializer serializer, const void* items, kSize count, xkSerializerPrimative primitive)
{
    kObj(kSerializer, serializer);
    k32u itemSize = 1u << primitive; 
    kBool autoFlush = xkSerializer_CanAutoFlush(serializer); 
    kSize remaining = count; 

    if (autoFlush && !obj->swap && (count > xkSERIALIZER_MAX_BUFFERED_WRITE_SIZE))
    {
        kCheck(xkSerializer_FlushBuffers(serializer));
        
        kCheck(kStream_Write(obj->writeStream, items, itemSize * count));
            
        kCheck(xkSerializer_AddBuffer(serializer));      
    }
    else
    {
        while (remaining > 0)
        {
            kSize maxItems = (obj->currentBuffer->capacity - obj->currentBuffer->written) >> primitive;
            kSize itemCount = kMin_(maxItems, remaining);
            const kByte* src = kCast(kByte*, items) + (itemSize * (count - remaining));
            kByte* dest = kCast(kByte*, obj->currentBuffer->data) + obj->currentBuffer->written;
            kSize dataSize = itemSize * itemCount; 

            if (!obj->swap || (itemSize == 1))
            {
                kMemCopy(dest, src, dataSize);
            }
            else
            {
                kByte* destEnd = dest + dataSize; 

                while (dest != destEnd)
                {
                    kMemReverseCopy(dest, src, itemSize); 
                    dest += itemSize; 
                    src += itemSize; 
                }
            }

            obj->currentBuffer->written += itemSize * itemCount;

            remaining -= itemCount;

            if (remaining > 0)
            {
                if (autoFlush)
                {
                    kCheck(xkSerializer_FlushBuffers(serializer));
                }
                kCheck(xkSerializer_AddBuffer(serializer));
            }
        }
    }

    return kOK; 
}

kFx(kStatus) xkSerializer_ReadPrimitives(kSerializer serializer, void* items, kSize count, xkSerializerPrimative primitive)
{
    kObj(kSerializer, serializer);
    k32u itemSize = 1u << primitive; 
    kSize dataSize = itemSize*count; 
    
    kCheck(kStream_Read(obj->readStream, items, dataSize)); 

    if (obj->swap)
    {        
        kByte* dataIt = (kByte*) items; 
        kByte* dataEnd = dataIt + dataSize; 

        while (dataIt != dataEnd)
        {
            kMemReverse(dataIt, itemSize); 
            dataIt += itemSize; 
        }
    }

    return kOK; 
}

kFx(kStatus) kSerializer_WriteText(kSerializer serializer, const kChar* data)
{
    kChar chr = 0; 

    do
    {
        chr = *data++; 

        kCheck(kSerializer_WriteChar(serializer, chr)); 
    }
    while (chr != 0); 

    return kOK; 
}

kFx(kStatus) kSerializer_ReadText(kSerializer serializer, kChar* data, kSize capacity)
{
    kChar* end = data + capacity; 
    kChar chr = 0; 
    
    kCheckArgs(capacity > 0); 

    do
    {
        kCheck(kSerializer_ReadChar(serializer, &chr)); 

        *data++ = chr; 
    }
    while ((chr != 0) && (data != end)); 

    return (chr == 0) ? kOK : kERROR_INCOMPLETE; 
}

#if (K_POINTER_SIZE == 4)

kFx(kStatus) kSerializer_WriteSizeArray(kSerializer serializer, const kSize* data, kSize count)
{
    kObj(kSerializer, serializer);
    kSize i = 0; 

    if (obj->sizeEncoding == 4)
    {
        kCheck(kSerializer_Write32uArray(serializer, data, count)); 
    }
    else
    {
        for (i = 0; i < count; ++i)
        {
            kCheck(kSerializer_Write64u(serializer, data[i])); 
        }
    }
    return kOK; 
}

kFx(kStatus) kSerializer_ReadSizeArray(kSerializer serializer, kSize* data, kSize count)
{
    kObj(kSerializer, serializer);
    kSize i = 0; 

    if (obj->sizeEncoding == 4)
    {
        kCheck(kSerializer_Read32uArray(serializer, data, count)); 
    }
    else
    {
        for (i = 0; i < count; ++i)
        {
            k64u value; 

            kCheck(kSerializer_Read64u(serializer, &value)); 

            kCheckTrue(value <= k32U_MAX, kERROR_FORMAT); 

            data[i] = (kSize) value;              
        }
    }
    return kOK; 
}

kFx(kStatus) kSerializer_WriteSSizeArray(kSerializer serializer, const kSSize* data, kSize count)
{
    kObj(kSerializer, serializer);
    kSize i = 0; 

    if (obj->sizeEncoding == 4)
    {
        kCheck(kSerializer_Write32sArray(serializer, data, count)); 
    }
    else
    {
        for (i = 0; i < count; ++i)
        {
            kCheck(kSerializer_Write64s(serializer, data[i])); 
        }
    }
    return kOK; 
}

kFx(kStatus) kSerializer_ReadSSizeArray(kSerializer serializer, kSSize* data, kSize count)
{
    kObj(kSerializer, serializer);
    kSize i = 0; 

    if (obj->sizeEncoding == 4)
    {
        kCheck(kSerializer_Read32sArray(serializer, data, count)); 
    }
    else
    {
        for (i = 0; i < count; ++i)
        {
            k64s value; 

            kCheck(kSerializer_Read64s(serializer, &value)); 

            kCheckTrue((value >= k32S_MIN) && (value <= k32S_MAX), kERROR_FORMAT); 

            data[i] = (k32s) value; 
        }
    }

    return kOK; 
}

#elif (K_POINTER_SIZE == 8)

kFx(kStatus) kSerializer_WriteSizeArray(kSerializer serializer, const kSize* data, kSize count)
{
    kObj(kSerializer, serializer);
    kSize i = 0; 

    if (obj->sizeEncoding == 8)
    {
        kCheck(kSerializer_Write64uArray(serializer, data, count)); 
    }
    else
    {
        for (i = 0; i < count; ++i)
        {
            kCheckArgs(data[i] <= k32U_MAX); 

            kCheck(kSerializer_Write32u(serializer, (k32u) data[i])); 
        }
    }
    return kOK; 
}

kFx(kStatus) kSerializer_ReadSizeArray(kSerializer serializer, kSize* data, kSize count)
{
    kObj(kSerializer, serializer);
    kSize i = 0; 

    if (obj->sizeEncoding == 8)
    {
        kCheck(kSerializer_Read64uArray(serializer, data, count)); 
    }
    else
    {
        for (i = 0; i < count; ++i)
        {
            k32u value; 

            kCheck(kSerializer_Read32u(serializer, &value)); 
                
            data[i] = value;  
        }
    }

    return kOK; 
}

kFx(kStatus) kSerializer_WriteSSizeArray(kSerializer serializer, const kSSize* data, kSize count)
{
    kObj(kSerializer, serializer);
    kSize i = 0; 

    if (obj->sizeEncoding == 8)
    {
        kCheck(kSerializer_Write64sArray(serializer, data, count)); 
    }
    else
    {
        for (i = 0; i < count; ++i)
        {
            kCheckArgs((data[i] >= k32S_MIN) && (data[i] <= k32S_MAX)); 

            kCheck(kSerializer_Write32s(serializer, (k32s) data[i])); 
        }
    }
    return kOK; 
}

kFx(kStatus) kSerializer_ReadSSizeArray(kSerializer serializer, kSSize* data, kSize count)
{
    kObj(kSerializer, serializer);
    kSize i = 0; 

    if (obj->sizeEncoding == 8)
    {
        kCheck(kSerializer_Read64sArray(serializer, data, count)); 
    }
    else
    {
        for (i = 0; i < count; ++i)
        {
            k32s value; 

            kCheck(kSerializer_Read32s(serializer, &value)); 
        
            data[i] = value;  
        }
    }

    return kOK; 
}

#endif

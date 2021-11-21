/** 
 * @file    kSerializer.x.h
 *
 * @internal
 * Copyright (C) 2005-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_SERIALIZER_X_H
#define K_API_SERIALIZER_X_H

#include <kApi/Io/kStream.h> 

/*
* kCompressor interface
*/

typedef struct kCompressorVTable
{   
    kCompressionType (kCall* VCompressionType)();
    kVersion (kCall* VRequiredVersion)();
    kStatus (kCall* VConstruct)(kObject* compressor, kStream stream, kBool isWriter, k32s level, kAlloc allocator);
    kStatus (kCall* VFinishWrite)(kObject compressor);
    kStatus (kCall* VFinishRead)(kObject compressor);
} kCompressorVTable;

kDeclareInterfaceEx(k, kCompressor, kNull) 

/* 
* Private interface implementation stubs.
*/

kInlineFx(kCompressionType) xkCompressor_VCompressionType()
{
    return kCOMPRESSION_TYPE_NULL;
}

kInlineFx(kVersion) xkCompressor_VRequiredVersion()
{
    return kVersion_Create(6, 0, 0, 0);
}

kInlineFx(kStatus) xkCompressor_VConstruct(kObject* compressor, kStream stream, k32s mode, k32s level, kAlloc allocator)
{
    return kERROR_UNIMPLEMENTED; 
}

kInlineFx(kStatus) xkCompressor_VFinishWrite(kObject compressor)
{
    return kERROR_UNIMPLEMENTED;
}

kInlineFx(kStatus) kCompressor_FinishWrite(kObject compressor)
{
    return xkCompressor_VTable(compressor)->VFinishWrite(compressor);
}

kInlineFx(kStatus) xkCompressor_VFinishRead(kObject compressor)
{
    return kERROR_UNIMPLEMENTED;
}

kInlineFx(kStatus) kCompressor_FinishRead(kObject compressor)
{
    return xkCompressor_VTable(compressor)->VFinishRead(compressor);
}

/*
* kSerializer class
*/

#define xkSERIALIZER_DEFAULT_BUFFER_SIZE             (16384)
#define xkSERIALIZER_MAX_BUFFERED_WRITE_SIZE         (2048)

#define xkSERIALIZER_PRIMATIVE_8          (0)
#define xkSERIALIZER_PRIMATIVE_16         (1)
#define xkSERIALIZER_PRIMATIVE_32         (2)
#define xkSERIALIZER_PRIMATIVE_64         (3)

typedef k32u xkSerializerPrimative; 

/* 
 * Signatures for object de/serialization methods.
 */
typedef kStatus (kCall* kSerializerObjectSerializeFx)(kObject object, kSerializer serializer); 
typedef kStatus (kCall* kSerializerObjectDeserializeFx)(kObject object, kSerializer serializer, kAlloc allocator);   //legacy
typedef kStatus (kCall* kSerializerObjectDeserializeExFx)(kObject object, kSerializer serializer);  //modern

/* 
 * Signatures for value de/serialization methods.
 */
typedef kStatus (kCall* kSerializerValueSerializeFx)(kType type, const void* values, kSize count, kSerializer serializer); 
typedef kStatus (kCall* kSerializerValueDeserializeFx)(kType type, void* values, kSize count, kSerializer serializer); 


typedef struct kSerializerVTable
{    
    kObjectVTable base; 
    kStatus (kCall* VInit)(kSerializer serializer, kType type, kStream stream, kAlloc allocator); 
    kBool (kCall* VCanAutoFlush)(kSerializer serializer); 
    kStatus (kCall* VSetSizeEncoding)(kSerializer serializer, k32u byteCount); 
    kStatus (kCall* VReset)(kSerializer serializer); 
    kStatus (kCall* VWriteObject)(kSerializer serializer, kObject object); 
    kStatus (kCall* VReadObject)(kSerializer serializer, kObject* object, kAlloc allocator); 
    kBool(kCall* VCanWriteType)(kSerializer serializer, kType type);
    kStatus (kCall* VWriteType)(kSerializer serializer, kType type, kTypeVersion* version); 
    kStatus (kCall* VReadType)(kSerializer serializer, kType* type, kTypeVersion* version); 
} kSerializerVTable; 

typedef struct xkSerializerBuffer
{
    struct xkSerializerBuffer* next;               //next buffer in linked list
    void* data;                                   //buffer data
    kSize capacity;                               //buffer capacity, in bytes
    kSize written;                                //valid bytes written to buffer
} xkSerializerBuffer; 

typedef struct xkSerializerWriteSection
{
    xkSerializerBuffer* buffer;                    //buffer containing optional write size
    kSize start;                                  //buffer offset to optional write size 
    kType type;                                   //field type
    kBool includeSize;                            //include size field in recorded size?
} xkSerializerWriteSection; 

kDeclareValueEx(k, xkSerializerWriteSection, kValue)

typedef struct kSerializerClass
{    
    kObjectClass base; 

    /*
    * Protected
    */

    kStream stream;                                 //[Protected] underlying stream for reading/writing
    kStream readStream;                             //[Protected] read stream; usually same as underlying stream; may be replaced by child classes
    kStream writeStream;                            //[Protected] write stream; usually same as underlying stream; may be replaced by child classes
    kAlloc readAlloc;                               //[Protected] read allocator, used during kSerializer_ReadObject

    /*
    * Private
    */

    kEndianness endianness;                         //[Private] endianness setting
    kBool swap;                                     //[Private] should the serializer reverse endianness?
    k32u sizeEncoding;                              //[Private] kSize and kSSize encoding (4 or 8 bytes). 
    kText16 format;                                 //[Private] format name (e.g. Dat5, Dat6)
    kVersion formatVersion;                         //[Private] global serialization format version                          
    kMap assemblyVersions;                          //[Private] maps assemblies to versions (kMap<kAssembly, kVersion>)

    xkSerializerBuffer* activeBuffers;              //[Private] head of active write buffer linked list
    xkSerializerBuffer* currentBuffer;              //[Private] tail of active write buffer linked list
    xkSerializerBuffer* freeBuffers;                //[Private] head of free write buffer linked list
    kSize bufferSize;                               //[Private] size of each write buffer, excluding header

    kArrayList writeSections;                       //[Private] stack of optional write sections (kArrayList<xkSerializerWriteSection>)
    kArrayList readSections;                        //[Private] stack of optional read sections (kArrayList<k64u>)

} kSerializerClass;

kDeclareVirtualClassEx(k, kSerializer, kObject)

/* 
* Forward declarations. 
*/

kInlineFx(kStatus) kSerializer_Write32u(kSerializer serializer, k32u data);
kInlineFx(kStatus) kSerializer_Read32u(kSerializer serializer, k32u* data);
kInlineFx(kStatus) kSerializer_Write32s(kSerializer serializer, k32s data); 
kInlineFx(kStatus) kSerializer_Read32s(kSerializer serializer, k32s* data); 

kInlineFx(kStatus) kSerializer_Write64u(kSerializer serializer, k64u data);
kInlineFx(kStatus) kSerializer_Read64u(kSerializer serializer, k64u* data);
kInlineFx(kStatus) kSerializer_Write64s(kSerializer serializer, k64s data); 
kInlineFx(kStatus) kSerializer_Read64s(kSerializer serializer, k64s* data);

kFx(kStatus) kSerializer_WriteItems(kSerializer serializer, kType type, kTypeVersion version, const void* items, kSize count); 
kFx(kStatus) kSerializer_ReadItems(kSerializer serializer, kType type, kTypeVersion version, void* items, kSize count); 

/* 
* Protected methods. 
* 
* The kSerializer class is extensible, but doing so can be challenging. The following methods
* are considered protected, but are more subject to change than the documented, protected methods
* in other classes such as kObject or kAlloc. Use with caution.
* 
* TODO: add API documentation for protected methods. 
*/

//[Protected]
kFx(kVersion) kSerializer_FindRequestedMinVersion(kSerializer serializer, kType type);

//[Protected]
kFx(kStatus) kSerializer_FindCompatibleVersion(kSerializer serializer, kType type, kTypeVersion* version); 

//[Protected]
kInlineFx(kStatus) kSerializer_InvokeValueSerializer(kSerializer serializer, kType type, kTypeVersion version, const void* values, kSize count)
{
    kSerializerValueSerializeFx method = (kSerializerValueSerializeFx) kType_VersionSerializeFx(type, version);

    return method(type, values, count, serializer); 
}

//[Protected]
kInlineFx(kStatus) kSerializer_InvokeValueDeserializer(kSerializer serializer, kType type, kTypeVersion version, void* values, kSize count)
{
    kSerializerValueDeserializeFx method = (kSerializerValueDeserializeFx) kType_VersionDeserializeFx(type, version);

    return method(type, values, count, serializer); 
}

//[Protected]
kInlineFx(kStatus) kSerializer_InvokeObjectSerializer(kSerializer serializer, kType type, kTypeVersion version, kObject object)
{
    kSerializerObjectSerializeFx method = (kSerializerObjectSerializeFx) kType_VersionSerializeFx(type, version);

    return method(object, serializer); 
}

//[Protected]
kFx(kStatus) xkSerializer_DeserializeObject(kSerializer serializer, kType type, kTypeVersion version, kObject* object, kAlloc alloc);

//[Protected]
kFx(kStatus) xkSerializer_FlushEx(kSerializer serializer, kBool flushStream);

//[Protected]
kFx(kStatus) kSerializer_VInit(kSerializer serializer, kType type, kStream stream, kAlloc allocator); 

//[Protected]
kFx(kStatus) kSerializer_VRelease(kSerializer serializer); 

//[Protected]
kInlineFx(kBool) kSerializer_VCanAutoFlush(kSerializer serializer)
{
    kObj(kSerializer, serializer);

    return (kArrayList_Count(obj->writeSections) == 0); 
}

//[Protected]
kInlineFx(kStatus) kSerializer_VSetSizeEncoding(kSerializer serializer, k32u byteCount)
{
    kObj(kSerializer, serializer);

    kCheckArgs((byteCount == 4) || (byteCount == 8)); 

    obj->sizeEncoding = byteCount; 

    return kOK; 
}

//[Protected]
kFx(kStatus) kSerializer_VReset(kSerializer serializer); 

//[Protected]
kInlineFx(kStatus) kSerializer_VWriteObject(kSerializer serializer, kObject object)
{
    return kERROR_UNIMPLEMENTED; 
}

//[Protected]
kInlineFx(kStatus) kSerializer_VReadObject(kSerializer serializer, kObject* object, kAlloc allocator)
{
    return kERROR_UNIMPLEMENTED; 
}

//[Protected]
kInlineFx(kBool) kSerializer_VCanWriteType(kSerializer serializer, kType type)
{
    kTypeVersion version;
  
    return kSuccess(kSerializer_FindCompatibleVersion(serializer, type, &version)); 
}

//[Protected]
kInlineFx(kStatus) kSerializer_VWriteType(kSerializer serializer, kType type, kTypeVersion* version)
{
    return kERROR_UNIMPLEMENTED; 
}

//[Protected]
kInlineFx(kStatus) kSerializer_VReadType(kSerializer serializer, kType* type, kTypeVersion* version)
{
    return kERROR_UNIMPLEMENTED; 
}

/* 
* Private methods. 
*/

kFx(kStatus) xkSerializer_AllocateBuffer(kSerializer serializer); 
kFx(kStatus) xkSerializer_InsertHeader(kSerializer serializer); 
kFx(kStatus) xkSerializer_AddBuffer(kSerializer serializer); 
kFx(kStatus) xkSerializer_ClearBuffers(kSerializer serializer);
kFx(kStatus) xkSerializer_FlushBuffers(kSerializer serializer); 

kFx(kStatus) xkSerializer_WritePrimitives(kSerializer serializer, const void* items, kSize count, xkSerializerPrimative primitive);
kFx(kStatus) xkSerializer_ReadPrimitives(kSerializer serializer, void* items, kSize count, xkSerializerPrimative primitive); 

kFx(kStatus) xkSerializer_AdvanceReadImpl(kSerializer serializer, kSize offset); 



kInlineFx(kStatus) xkSerializer_WriteItemsT(kSerializer serializer, kType type, kTypeVersion version, const void* items, kSize count, kSize itemSize)
{
    kAssert(xkType_IsPointerCompatible(type, itemSize)); 

    return kSerializer_WriteItems(serializer, type, version, items, count);
} 
 
kInlineFx(kStatus) xkSerializer_ReadItemsT(kSerializer serializer, kType type, kTypeVersion version, void* items, kSize count, kSize itemSize)
{
    kAssert(xkType_IsPointerCompatible(type, itemSize)); 

    return kSerializer_ReadItems(serializer, type, version, items, count);
} 

kInlineFx(kBool) xkSerializer_CanAutoFlush(kSerializer serializer)
{
    return xkSerializer_VTable(serializer)->VCanAutoFlush(serializer); 
}

kInlineFx(kByte*) xkSerializer_Buffer(kSerializer serializer)
{
    kObj(kSerializer, serializer); 

    return (kByte*) kPointer_ByteOffset(obj->currentBuffer->data, (kSSize)obj->currentBuffer->written); 
}

kInlineFx(kBool) xkSerializer_CanBuffer(kSerializer serializer, kSize count)
{
    kObj(kSerializer, serializer); 
    
    return count <= (obj->currentBuffer->capacity - obj->currentBuffer->written); 
}

kInlineFx(void) xkSerializer_ReorderCopyPrimitive(kSerializer serializer, void* dest, void* src, kSize size)
{
    kObj(kSerializer, serializer); 

    if (!obj->swap)     kItemCopy(dest, src, size);    
    else                kMemReverseCopy(dest, src, size);
}

kInlineFx(void) xkSerializer_ReorderPrimitive(kSerializer serializer, void* buffer, kSize size)
{
    kObj(kSerializer, serializer); 

    if (obj->swap)
    {
        kMemReverse(buffer, size); 
    }
}

kInlineFx(kStatus) xkSerializer_WritePrimitive(kSerializer serializer, void* data, xkSerializerPrimative primitive)
{
    kObj(kSerializer, serializer); 
    k32u size = 1u << primitive;

    if (!xkSerializer_CanBuffer(serializer, size))
    {
        return xkSerializer_WritePrimitives(serializer, data, 1, primitive); 
    }

    xkSerializer_ReorderCopyPrimitive(serializer, xkSerializer_Buffer(serializer), data, size); 

    obj->currentBuffer->written += size;

    return kOK;
}

kInlineFx(kStatus) xkSerializer_ReadPrimitive(kSerializer serializer, void* data, xkSerializerPrimative primitive)
{
    kObj(kSerializer, serializer);
    k32u size = 1u << primitive;

    kCheck(kStream_Read(obj->readStream, data, size)); 

    xkSerializer_ReorderPrimitive(serializer, data, size); 

    return kOK; 
}

#if (K_POINTER_SIZE == 4)


kInlineFx(kStatus) xkSerializer_WriteSizePrimitive(kSerializer serializer, kSize data)
{
    kObj(kSerializer, serializer); 
    
    if (obj->sizeEncoding == 4)     return kSerializer_Write32u(serializer, data); 
    else                            return kSerializer_Write64u(serializer, data); 
}

kInlineFx(kStatus) xkSerializer_ReadSizePrimitive(kSerializer serializer, kSize* data)
{
    kObj(kSerializer, serializer); 
    
    if (obj->sizeEncoding == 4)    
    {
        return kSerializer_Read32u(serializer, data); 
    }
    else                       
    {
        k64u value; 

        kCheck(kSerializer_Read64u(serializer, &value));
        
        kCheckTrue(value <= k32U_MAX, kERROR_FORMAT); 

        *data = (kSize) value; 
    }

    return kOK;
}

kInlineFx(kStatus) xkSerializer_WriteSSizePrimitive(kSerializer serializer, kSSize data)
{
    kObj(kSerializer, serializer); 
    
    if (obj->sizeEncoding == 4)     return kSerializer_Write32s(serializer, data); 
    else                            return kSerializer_Write64s(serializer, data); 
}

kInlineFx(kStatus) xkSerializer_ReadSSizePrimitive(kSerializer serializer, kSSize* data)
{
    kObj(kSerializer, serializer); 
    
    if (obj->sizeEncoding == 4)    
    {
        return kSerializer_Read32s(serializer, data); 
    }
    else                       
    {
        k64s value; 

        kCheck(kSerializer_Read64s(serializer, &value));
        
        kCheckTrue((value >= k32S_MIN) && (value <= k32S_MAX), kERROR_FORMAT); 

        *data = (kSSize) value; 
    }

    return kOK;
}

#elif (K_POINTER_SIZE == 8)

kInlineFx(kStatus) xkSerializer_WriteSizePrimitive(kSerializer serializer, kSize data)
{
    kObj(kSerializer, serializer); 
    
    if (obj->sizeEncoding == 8)     
    {
        return kSerializer_Write64u(serializer, data); 
    }
    else
    {
        kCheckArgs(data <= k32U_MAX); 

        return kSerializer_Write32u(serializer, (k32u) data); 
    }
}

kInlineFx(kStatus) xkSerializer_ReadSizePrimitive(kSerializer serializer, kSize* data)
{
    kObj(kSerializer, serializer); 
    
    if (obj->sizeEncoding == 8)    
    {
        return kSerializer_Read64u(serializer, data); 
    }
    else                       
    {
        k32u value; 

        kCheck(kSerializer_Read32u(serializer, &value));
        
        *data = value; 
    }

    return kOK;
}

kInlineFx(kStatus) xkSerializer_WriteSSizePrimitive(kSerializer serializer, kSSize data)
{
    kObj(kSerializer, serializer); 
    
    if (obj->sizeEncoding == 8)     
    {
        return kSerializer_Write64s(serializer, data); 
    }
    else
    {
        kCheckArgs((data >= k32S_MIN) && (data <= k32S_MAX)); 

        return kSerializer_Write32s(serializer, (k32s) data); 
    }
}

kInlineFx(kStatus) xkSerializer_ReadSSizePrimitive(kSerializer serializer, kSSize* data)
{
    kObj(kSerializer, serializer); 
    
    if (obj->sizeEncoding == 8)    
    {
        return kSerializer_Read64s(serializer, data); 
    }
    else                       
    {
        k32s value; 

        kCheck(kSerializer_Read32s(serializer, &value));
        
        *data = value; 
    }

    return kOK;
}

#endif 

#endif

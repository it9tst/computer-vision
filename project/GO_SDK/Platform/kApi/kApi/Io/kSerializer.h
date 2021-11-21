/** 
 * @file    kSerializer.h
 * @brief   Declares the kSerializer class. 
 *
 * @internal
 * Copyright (C) 2005-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_SERIALIZER_H
#define K_API_SERIALIZER_H

#include <kApi/kApiDef.h>
#include <kApi/Data/kArrayList.h>

#include <kApi/Io/kSerializer.x.h>

/**
 * @class   kSerializer
 * @extends kObject
 * @ingroup kApi-Io
 * @brief   Base class for serialization/deserialization classes. 
 * 
 * The kDat5Serializer and kDat6Serializer classes extend the kSerializer base class to implement the 
 * kdat5 and kdat6 binary serialization formats, respectively. These formats represent binary serialization schemes developed 
 * for and used extensively with the Zen API, providing a simple method to serialize/deserialize complete object graphs. 
 * The primary difference between these formats is that kdat6 can be easily extended to serialize types defined in other 
 * assemblies. kdat6 is recommended for object serialization when legacy compatibilty is not a concern.  
 * 
 * kSerializer itself is also an instantiable class. The kSerializer base class does not provide the ability to automatically 
 * serialize/deserialize complete objects, but can be used to read/write primitive data and arrays. This approach can be 
 * used to implement custom binary serialization schemes without needing to extend the kSerializer class.
 *
 * kSerializer provides an internal write buffer, but does not provide a read buffer. Accordingly, for best performance,
 * the underlying stream should provide a read buffer, but not a write buffer. (For kTcpClient, a socket-level 
 * write buffer is helpful, but a client-level write buffer should be avoided.) kSerializer does not automatically flush its 
 * internal write buffer when the serializer is destroyed. The Flush method must be used to ensure that all bytes are written 
 * to the underlying stream.
 * 
 * By default, kSerializer assumes that primitive values should be encoded to (or decoded from) little endian format. Accordingly, on 
 * big endian platforms, the in-memory byte ordering will be swapped as data is read from or written to the underlying stream. For custom 
 * parser/formatters or in derived serialization classes, this default behaviour can be overriden using the kSerializer_SetEndianness method.
 * 
 * By default, kSerializer assumes that kSize values should be encoded/decoded as 32-bit values. For custom parser/formatters or in derived 
 * serialization classes, this default behaviour can be overriden using the kSerializer_SetSizeEncoding method. 
 *
 * The following example illustrates three different approaches for writing an image object in kdat-6 format. 
@code {.c}
//This approach provides the most control/flexibility.
kStatus WriteObject1(kImage image, const kChar* path)
{
    kFile file = kNULL; 
    kSerializer writer = kNULL; 

    kTry
    {
        //open a file for writing; do not allocate a write buffer (serializers have built-in write buffers)
        kTest(kFile_Construct(&file, path, kFILE_MODE_WRITE, kNULL)); 

        //construct a kDat6Serializer object (a kSerializer subclass that implements kdat6 format)
        kTest(kDat6Serializer_Construct(&writer, file, kNULL)); 
        
        //serialize the image to file; it's possible to write multiple objects here, but we only need to write one
        kTest(kSerializer_WriteObject(writer, image)); 

        //serializers must always be flushed before closing; otherwise, data may be lost
        kTest(kSerializer_Flush(writer)); 
    }
    kFinally
    {
        kObject_Destroy(writer); 
        kObject_Destroy(file); 

        kEndFinally(); 
    }

    return kOK; 
}

//This approach uses a kSerializer utility function to simplify the operation above. This approach is limited 
//to writing a single object to file, but that's all we need here.
kStatus WriteObject2(kImage image, const kChar* path)
{
    return kSerializer_SaveObject(image, kTypeOf(kDat6Serializer), path); 
}

//This approach uses a kUtils function to save on typing.
kStatus WriteObject3(kImage image, const kChar* path)
{
    return kSave6(image, path); 
}
@endcode
 * 
 * The following example illustrates the implemenation of a custom binary parser, using the kSerializer base class directly. 
@code {.c}

#include <kApi/Io/kSerializer.h>

#define MY_DATA_CAPACITY        (128)

typedef struct MyData
{
    k32u id; 
    kText64 name; 
    kSize valueCount; 
    k64u values[MY_DATA_CAPACITY]; 
} MyData; 

kStatus ReadData(kStream stream, MyData* data)
{
    kSerializer reader = kNULL; 

    kTry
    {
        //construct a plain serializer object (can write/write primitive types and arrays)
        kTest(kSerializer_Construct(&reader, stream, kNULL, kNULL)); 

        //read the data id/name
        kTest(kSerializer_Read32u(reader, &data->id)); 
        kTest(kSerializer_ReadText(reader, data->name, kCountOf(data->name))); 

        //read data content, encoded as a variable-length array of 64-bit values
        kTest(kSerializer_ReadSize(reader, &data->valueCount)); 
        kTestTrue(data->valueCount <= kCountOf(data->values), kERROR_FORMAT); 
        kTest(kSerializer_Read64uArray(reader, data->values, data->valueCount)); 
    }
    kFinally
    {
        kObject_Destroy(reader); 

        kEndFinally(); 
    }

    return kOK; 
}

@endcode
 */
//typedef kObject kSerializer;   --forward-declared in kApiDef.x.h

/** 
 * Constructs a serializer object using the specified serialization format. 
 *
 * This method is a factory constructor; the type of serializer to be instantiated is provided 
 * as an argument. If no serializer type is provided, the default serializer will be used 
 * (currently kSerializer). 
 *
 * @public                  @memberof kSerializer
 * @param   serializer      Serializer object. 
 * @param   stream          Stream for reading/writing. 
 * @param   serializerType  Serializer type (or kNULL for default). 
 * @param   allocator       Memory allocator (or kNULL for default). 
 * @return                  Operation status. 
 */
kFx(kStatus) kSerializer_Construct(kSerializer* serializer, kStream stream, kType serializerType, kAlloc allocator); 

/** 
 * Loads an object from file using the specified serializer type.
 *
 * @public                  @memberof kSerializer
 * @param   object          Receives deserialized object.
 * @param   serializerType  Type of serializer to use (e.g. kDat6Serializer). 
 * @param   filePath        Path of the file to load. 
 * @param   readAlloc       Memory allocator to use for loaded object (or kNULL for default). 
 * @return                  Operation status. 
 */
kFx(kStatus) kSerializer_LoadObject(kObject* object, kType serializerType, const kChar* filePath, kAlloc readAlloc);

/** 
 * Saves an object to file using the specified serializer type.
 *
 * @public                  @memberof kSerializer
 * @param   object          Object to be serialized.
 * @param   serializerType  Type of serializer to use (e.g. kDat6Serializer). 
 * @param   filePath        Path of the file to save. 
 * @return                  Operation status. 
 */
kFx(kStatus) kSerializer_SaveObject(kObject object, kType serializerType, const kChar* filePath); 

/** 
 * Sets the version to use when serializing types. 
 * 
 * Some serialization formats use a versioning scheme in which a separate version is given 
 * for each assembly, while others use a single version that is applied to types from all 
 * assemblies. If a single version is used for assemblies, set the assembly argument to null. 
 *
 * @public                  @memberof kSerializer
 * @param   serializer      Serializer object. 
 * @param   assembly        Assembly object (or kNULL). 
 * @param   version         Desired version. 
 * @return                  Operation status. 
 */
kFx(kStatus) kSerializer_SetVersion(kSerializer serializer, kAssembly assembly, kVersion version); 

/** 
 * Explicitly sets the number of bytes used to encode/decode kSize and kSSize values. 
 * 
 * The default size encoding for kSerializer is 4 bytes. This value can be overridden by derived classes, 
 * or set explicitly for customer parser/formatters.
 *
 * @public                  @memberof kSerializer
 * @param   serializer      Serializer object. 
 * @param   byteCount       Number of bytes to use when encoding size values (accepts 4 or 8). 
 * @return                  Operation status. 
 */
kInlineFx(kStatus) kSerializer_SetSizeEncoding(kSerializer serializer, k32u byteCount)
{
    return xkSerializer_VTable(serializer)->VSetSizeEncoding(serializer, byteCount);
}

/**
* Reports the number of bytes used to encode/decode kSize and kSSize values. 
*
* @public                  @memberof kSerializer
* @param   serializer      Serializer object.
* @return                  Number of bytes to use when encoding size values (4 or 8). 
*/
kInlineFx(k32u) kSerializer_SizeEncoding(kSerializer serializer)
{
    kObj(kSerializer, serializer);

    return obj->sizeEncoding;
}

/** 
 * Explicitly sets the endianness (byte ordering) of encoded/decoded values.
 * 
 * The default endianness for kSerializer is kENDIANNESS_LITTLE. This value can be overridden by derived classes, 
 * or set explicitly for customer parser/formatters.
 *
 * @public                  @memberof kSerializer
 * @param   serializer      Serializer object. 
 * @param   endianness      Endianess for encoded/decoded values. 
 * @return                  Operation status. 
 */
kInlineFx(kStatus) kSerializer_SetEndianness(kSerializer serializer, kEndianness endianness)
{
    kObj(kSerializer, serializer);

    obj->endianness = endianness; 
    obj->swap = kEndianness_ShouldReverse(obj->endianness); 

    return kOK; 
}

/** 
 * Reports the (byte ordering) of encoded/decoded values.
 * 
 * @public                  @memberof kSerializer
 * @param   serializer      Serializer object. 
 * @return                  Endianess for encoded/decoded values. 
 */
kInlineFx(kEndianness) kSerializer_Endianness(kSerializer serializer)
{
    kObj(kSerializer, serializer);

    return obj->endianness; 
}

/** 
 * Writes an object to the underlying stream. 
 *
 * @public                  @memberof kSerializer
 * @param   serializer      Serializer object. 
 * @param   object          Object to be serialized. 
 * @return                  Operation status. 
 */
kInlineFx(kStatus) kSerializer_WriteObject(kSerializer serializer, kObject object)
{
    return xkSerializer_VTable(serializer)->VWriteObject(serializer, object);
}
/** 
 * Reads an object from the underlying stream. 
 *
 * @public                  @memberof kSerializer
 * @param   serializer      Serializer object. 
 * @param   object          Receives deserialized object. 
 * @param   allocator       Memory allocator (or kNULL for default). 
 * @return                  Operation status. 
 */
kInlineFx(kStatus) kSerializer_ReadObject(kSerializer serializer, kObject* object, kAlloc allocator)
{
    return xkSerializer_VTable(serializer)->VReadObject(serializer, object, allocator);
}

/** 
 * Writes a null-terminated kChar array. 
 *
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @param   data        String to write. 
 * @return              Operation status. 
 */
kFx(kStatus) kSerializer_WriteText(kSerializer serializer, const kChar* data); 

/** 
 * Reads a null-terminated kChar array. 
 *
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @param   data        Receives the string.
 * @param   capacity    Capacity of data string (including null terminator; must be greater than zero). 
 * @return              Operation status. 
 */
kFx(kStatus) kSerializer_ReadText(kSerializer serializer, kChar* data, kSize capacity); 

/** 
 * Writes a kByte value. 
 *
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @param   data        Value to write. 
 * @return              Operation status. 
 */
kInlineFx(kStatus) kSerializer_WriteByte(kSerializer serializer, kByte data)
{
    return xkSerializer_WritePrimitive(serializer, &data, xkSERIALIZER_PRIMATIVE_8); 
}

/** 
 * Writes a kByte array. 
 *
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @param   data        Array to write. 
 * @param   count       Count of array elements. 
 * @return              Operation status. 
 */
kInlineFx(kStatus) kSerializer_WriteByteArray(kSerializer serializer, const void* data, kSize count)
{
    return xkSerializer_WritePrimitives(serializer, data, count, xkSERIALIZER_PRIMATIVE_8);  
}

/** 
 * Reads a kByte value. 
 *
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @param   data        Receives the value.
 * @return              Operation status. 
 */
kInlineFx(kStatus) kSerializer_ReadByte(kSerializer serializer, kByte* data)
{
    kObj(kSerializer, serializer);

    return kStream_Read(obj->readStream, data, 1); 
}

/** 
 * Reads a kByte array. 
 *
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @param   data        Receives the array elements.
 * @param   count       Count of array elements to read. 
 * @return              Operation status. 
 */
kInlineFx(kStatus) kSerializer_ReadByteArray(kSerializer serializer, void* data, kSize count)
{
    kObj(kSerializer, serializer);

    return kStream_Read(obj->readStream, data, count); 
}

/** 
 * Writes a kChar value. 
 *
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @param   data        Value to write. 
 * @return              Operation status. 
 */
kInlineFx(kStatus) kSerializer_WriteChar(kSerializer serializer, kChar data)
{
    return xkSerializer_WritePrimitive(serializer, &data, xkSERIALIZER_PRIMATIVE_8); 
}

/** 
 * Writes a kChar array. 
 *
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @param   data        Array to write. 
 * @param   count       Count of array elements. 
 * @return              Operation status. 
 */
kInlineFx(kStatus) kSerializer_WriteCharArray(kSerializer serializer, const kChar* data, kSize count)
{
    return xkSerializer_WritePrimitives(serializer, data, count, xkSERIALIZER_PRIMATIVE_8);  
}

/** 
 * Reads a kChar value. 
 *
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @param   data        Receives the value.
 * @return              Operation status. 
 */
kInlineFx(kStatus) kSerializer_ReadChar(kSerializer serializer, kChar* data)
{
    kObj(kSerializer, serializer);

    return kStream_Read(obj->readStream, data, 1); 
}

/** 
 * Reads a kChar array. 
 *
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @param   data        Receives the array elements.
 * @param   count       Count of array elements to read. 
 * @return              Operation status. 
 */
kInlineFx(kStatus) kSerializer_ReadCharArray(kSerializer serializer, kChar* data, kSize count)
{
    kObj(kSerializer, serializer);

    return kStream_Read(obj->readStream, data, count); 
}

/** 
 * Writes a k8u value. 
 *
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @param   data        Value to write. 
 * @return              Operation status. 
 */
kInlineFx(kStatus) kSerializer_Write8u(kSerializer serializer, k8u data)
{
    return xkSerializer_WritePrimitive(serializer, &data, xkSERIALIZER_PRIMATIVE_8); 
}

/** 
 * Writes a k8u array. 
 *
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @param   data        Array to write. 
 * @param   count       Count of array elements. 
 * @return              Operation status. 
 */
kInlineFx(kStatus) kSerializer_Write8uArray(kSerializer serializer, const k8u* data, kSize count)
{
    return xkSerializer_WritePrimitives(serializer, data, count, xkSERIALIZER_PRIMATIVE_8);  
}

/** 
 * Reads a k8u value. 
 *
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @param   data        Receives the value.
 * @return              Operation status. 
 */
kInlineFx(kStatus) kSerializer_Read8u(kSerializer serializer, k8u* data)
{
    kObj(kSerializer, serializer);

    return kStream_Read(obj->readStream, data, 1); 
}

/** 
 * Reads a k8u array. 
 *
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @param   data        Receives the array elements.
 * @param   count       Count of array elements to read. 
 * @return              Operation status. 
 */
kInlineFx(kStatus) kSerializer_Read8uArray(kSerializer serializer, k8u* data, kSize count)
{
    kObj(kSerializer, serializer);

    return kStream_Read(obj->readStream, data, count); 
}
/** 
 * Writes a k8s value. 
 *
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @param   data        Value to write. 
 * @return              Operation status. 
 */
kInlineFx(kStatus) kSerializer_Write8s(kSerializer serializer, k8s data)
{
    return xkSerializer_WritePrimitive(serializer, &data, xkSERIALIZER_PRIMATIVE_8); 
}

/** 
 * Writes a k8s array. 
 *
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @param   data        Array to write. 
 * @param   count       Count of array elements. 
 * @return              Operation status. 
 */
kInlineFx(kStatus) kSerializer_Write8sArray(kSerializer serializer, const k8s* data, kSize count)
{
    return xkSerializer_WritePrimitives(serializer, data, count, xkSERIALIZER_PRIMATIVE_8);  
}

/** 
 * Reads a k8s value. 
 *
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @param   data        Receives the value.
 * @return              Operation status. 
 */
kInlineFx(kStatus) kSerializer_Read8s(kSerializer serializer, k8s* data)
{
    kObj(kSerializer, serializer);

    return kStream_Read(obj->readStream, data, 1); 
}

/** 
 * Reads a k8s array. 
 *
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @param   data        Receives the array elements.
 * @param   count       Count of array elements to read. 
 * @return              Operation status. 
 */
kInlineFx(kStatus) kSerializer_Read8sArray(kSerializer serializer, k8s* data, kSize count)
{
    kObj(kSerializer, serializer);

    return kStream_Read(obj->readStream, data, count); 
}

/** 
 * Writes a k16u value. 
 *
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @param   data        Value to write. 
 * @return              Operation status. 
 */
kInlineFx(kStatus) kSerializer_Write16u(kSerializer serializer, k16u data)
{
    return xkSerializer_WritePrimitive(serializer, &data, xkSERIALIZER_PRIMATIVE_16); 
}
/** 
 * Writes a k16u array. 
 *
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @param   data        Array to write. 
 * @param   count       Count of array elements. 
 * @return              Operation status. 
 */
kInlineFx(kStatus) kSerializer_Write16uArray(kSerializer serializer, const k16u* data, kSize count)
{
    return xkSerializer_WritePrimitives(serializer, data, count, xkSERIALIZER_PRIMATIVE_16);  
}

/** 
 * Reads a k16u value. 
 *
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @param   data        Receives the value.
 * @return              Operation status. 
 */
kInlineFx(kStatus) kSerializer_Read16u(kSerializer serializer, k16u* data)
{
    return xkSerializer_ReadPrimitive(serializer, data, xkSERIALIZER_PRIMATIVE_16); 
}

/** 
 * Reads a k16u array. 
 *
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @param   data        Receives the array elements.
 * @param   count       Count of array elements to read. 
 * @return              Operation status. 
 */
kInlineFx(kStatus) kSerializer_Read16uArray(kSerializer serializer, k16u* data, kSize count)
{
    return xkSerializer_ReadPrimitives(serializer, data, count, xkSERIALIZER_PRIMATIVE_16);      
}

/** 
 * Writes a k16s value. 
 *
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @param   data        Value to write. 
 * @return              Operation status. 
 */
kInlineFx(kStatus) kSerializer_Write16s(kSerializer serializer, k16s data)
{
    return xkSerializer_WritePrimitive(serializer, &data, xkSERIALIZER_PRIMATIVE_16); 
}

/** 
 * Writes a k16s array. 
 *
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @param   data        Array to write. 
 * @param   count       Count of array elements. 
 * @return              Operation status. 
 */
kInlineFx(kStatus) kSerializer_Write16sArray(kSerializer serializer, const k16s* data, kSize count)
{
    return xkSerializer_WritePrimitives(serializer, data, count, xkSERIALIZER_PRIMATIVE_16);  
}

/** 
 * Reads a k16s value. 
 *
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @param   data        Receives the value.
 * @return              Operation status. 
 */
kInlineFx(kStatus) kSerializer_Read16s(kSerializer serializer, k16s* data)
{
    return xkSerializer_ReadPrimitive(serializer, data, xkSERIALIZER_PRIMATIVE_16); 
}

/** 
 * Reads a k16s array. 
 *
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @param   data        Receives the array elements.
 * @param   count       Count of array elements to read. 
 * @return              Operation status. 
 */
kInlineFx(kStatus) kSerializer_Read16sArray(kSerializer serializer, k16s* data, kSize count)
{
    return xkSerializer_ReadPrimitives(serializer, data, count, xkSERIALIZER_PRIMATIVE_16);      
}

/** 
 * Writes a k32u value. 
 *
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @param   data        Value to write. 
 * @return              Operation status. 
 */
kInlineFx(kStatus) kSerializer_Write32u(kSerializer serializer, k32u data)
{
    return xkSerializer_WritePrimitive(serializer, &data, xkSERIALIZER_PRIMATIVE_32); 
}

/** 
 * Writes a k32u array. 
 *
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @param   data        Array to write. 
 * @param   count       Count of array elements. 
 * @return              Operation status. 
 */
kInlineFx(kStatus) kSerializer_Write32uArray(kSerializer serializer, const k32u* data, kSize count)
{
    return xkSerializer_WritePrimitives(serializer, data, count, xkSERIALIZER_PRIMATIVE_32);  
}
/** 
 * Reads a k32u value. 
 *
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @param   data        Receives the value.
 * @return              Operation status. 
 */
kInlineFx(kStatus) kSerializer_Read32u(kSerializer serializer, k32u* data)
{
    return xkSerializer_ReadPrimitive(serializer, data, xkSERIALIZER_PRIMATIVE_32); 
}

/** 
 * Reads a k32u array. 
 *
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @param   data        Receives the array elements.
 * @param   count       Count of array elements to read. 
 * @return              Operation status. 
 */
kInlineFx(kStatus) kSerializer_Read32uArray(kSerializer serializer, k32u* data, kSize count)
{
    return xkSerializer_ReadPrimitives(serializer, data, count, xkSERIALIZER_PRIMATIVE_32);      
}

/** 
 * Writes a k32s value. 
 *
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @param   data        Value to write. 
 * @return              Operation status. 
 */
kInlineFx(kStatus) kSerializer_Write32s(kSerializer serializer, k32s data)
{
    return xkSerializer_WritePrimitive(serializer, &data, xkSERIALIZER_PRIMATIVE_32); 
}

/** 
 * Writes a k32s array. 
 *
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @param   data        Array to write. 
 * @param   count       Count of array elements. 
 * @return              Operation status. 
 */
kInlineFx(kStatus) kSerializer_Write32sArray(kSerializer serializer, const k32s* data, kSize count)
{
    return xkSerializer_WritePrimitives(serializer, data, count, xkSERIALIZER_PRIMATIVE_32);  
}

/** 
 * Reads a k32s value. 
 *
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @param   data        Receives the value.
 * @return              Operation status. 
 */
kInlineFx(kStatus) kSerializer_Read32s(kSerializer serializer, k32s* data)
{
    return xkSerializer_ReadPrimitive(serializer, data, xkSERIALIZER_PRIMATIVE_32); 
}

/** 
 * Reads a k32s array. 
 *
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @param   data        Receives the array elements.
 * @param   count       Count of array elements to read. 
 * @return              Operation status. 
 */
kInlineFx(kStatus) kSerializer_Read32sArray(kSerializer serializer, k32s* data, kSize count)
{
    return xkSerializer_ReadPrimitives(serializer, data, count, xkSERIALIZER_PRIMATIVE_32);      
}

/** 
 * Writes a k64u value. 
 *
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @param   data        Value to write. 
 * @return              Operation status. 
 */
kInlineFx(kStatus) kSerializer_Write64u(kSerializer serializer, k64u data)
{
    return xkSerializer_WritePrimitive(serializer, &data, xkSERIALIZER_PRIMATIVE_64); 
}

/** 
 * Writes a k64u array. 
 *
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @param   data        Array to write. 
 * @param   count       Count of array elements. 
 * @return              Operation status. 
 */
kInlineFx(kStatus) kSerializer_Write64uArray(kSerializer serializer, const k64u* data, kSize count)
{
    return xkSerializer_WritePrimitives(serializer, data, count, xkSERIALIZER_PRIMATIVE_64);  
}
/** 
 * Reads a k64u value. 
 *
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @param   data        Receives the value.
 * @return              Operation status. 
 */
kInlineFx(kStatus) kSerializer_Read64u(kSerializer serializer, k64u* data)
{
    return xkSerializer_ReadPrimitive(serializer, data, xkSERIALIZER_PRIMATIVE_64); 
}
/** 
 * Reads a k64u array. 
 *
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @param   data        Receives the array elements.
 * @param   count       Count of array elements to read. 
 * @return              Operation status. 
 */
kInlineFx(kStatus) kSerializer_Read64uArray(kSerializer serializer, k64u* data, kSize count)
{
    return xkSerializer_ReadPrimitives(serializer, data, count, xkSERIALIZER_PRIMATIVE_64);      
}

/** 
 * Writes a k64s value. 
 *
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @param   data        Value to write. 
 * @return              Operation status. 
 */
kInlineFx(kStatus) kSerializer_Write64s(kSerializer serializer, k64s data)
{
    return xkSerializer_WritePrimitive(serializer, &data, xkSERIALIZER_PRIMATIVE_64); 
}
/** 
 * Writes a k64s array. 
 *
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @param   data        Array to write. 
 * @param   count       Count of array elements. 
 * @return              Operation status. 
 */
kInlineFx(kStatus) kSerializer_Write64sArray(kSerializer serializer, const k64s* data, kSize count)
{
    return xkSerializer_WritePrimitives(serializer, data, count, xkSERIALIZER_PRIMATIVE_64);  
}
/** 
 * Reads a k64s value. 
 *
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @param   data        Receives the value.
 * @return              Operation status. 
 */
kInlineFx(kStatus) kSerializer_Read64s(kSerializer serializer, k64s* data)
{
    return xkSerializer_ReadPrimitive(serializer, data, xkSERIALIZER_PRIMATIVE_64); 
}
/** 
 * Reads a k64s array. 
 *
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @param   data        Receives the array elements.
 * @param   count       Count of array elements to read. 
 * @return              Operation status. 
 */
kInlineFx(kStatus) kSerializer_Read64sArray(kSerializer serializer, k64s* data, kSize count)
{
    return xkSerializer_ReadPrimitives(serializer, data, count, xkSERIALIZER_PRIMATIVE_64);      
}

/** 
 * Writes a k32f value. 
 *
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @param   data        Value to write. 
 * @return              Operation status. 
 */
kInlineFx(kStatus) kSerializer_Write32f(kSerializer serializer, k32f data)
{
    return xkSerializer_WritePrimitive(serializer, &data, xkSERIALIZER_PRIMATIVE_32); 
}
/** 
 * Writes a k32f array. 
 *
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @param   data        Array to write. 
 * @param   count       Count of array elements. 
 * @return              Operation status. 
 */
kInlineFx(kStatus) kSerializer_Write32fArray(kSerializer serializer, const k32f* data, kSize count)
{
    return xkSerializer_WritePrimitives(serializer, data, count, xkSERIALIZER_PRIMATIVE_32);  
}
/** 
 * Reads a k32f value. 
 *
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @param   data        Receives the value.
 * @return              Operation status. 
 */
kInlineFx(kStatus) kSerializer_Read32f(kSerializer serializer, k32f* data)
{
    return xkSerializer_ReadPrimitive(serializer, data, xkSERIALIZER_PRIMATIVE_32); 
}
/** 
 * Reads a k32f array. 
 *
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @param   data        Receives the array elements.
 * @param   count       Count of array elements to read. 
 * @return              Operation status. 
 */
kInlineFx(kStatus) kSerializer_Read32fArray(kSerializer serializer, k32f* data, kSize count)
{
    return xkSerializer_ReadPrimitives(serializer, data, count, xkSERIALIZER_PRIMATIVE_32);      
}
/** 
 * Writes a k64f value. 
 *
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @param   data        Value to write. 
 * @return              Operation status. 
 */
kInlineFx(kStatus) kSerializer_Write64f(kSerializer serializer, k64f data)
{
    return xkSerializer_WritePrimitive(serializer, &data, xkSERIALIZER_PRIMATIVE_64); 
}
/** 
 * Writes a k64f array. 
 *
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @param   data        Array to write. 
 * @param   count       Count of array elements. 
 * @return              Operation status. 
 */
kInlineFx(kStatus) kSerializer_Write64fArray(kSerializer serializer, const k64f* data, kSize count)
{
    return xkSerializer_WritePrimitives(serializer, data, count, xkSERIALIZER_PRIMATIVE_64);  
}

/** 
 * Reads a k64f value. 
 *
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @param   data        Receives the value.
 * @return              Operation status. 
 */
kInlineFx(kStatus) kSerializer_Read64f(kSerializer serializer, k64f* data)
{
    return xkSerializer_ReadPrimitive(serializer, data, xkSERIALIZER_PRIMATIVE_64); 
}

/** 
 * Reads a k64f array. 
 *
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @param   data        Receives the array elements.
 * @param   count       Count of array elements to read. 
 * @return              Operation status. 
 */
kInlineFx(kStatus) kSerializer_Read64fArray(kSerializer serializer, k64f* data, kSize count)
{
    return xkSerializer_ReadPrimitives(serializer, data, count, xkSERIALIZER_PRIMATIVE_64);      
}

/** 
 * Writes a kSize value. 
 *
 * This method uses the current size encoding (specified with kSerializer_SetSizeEncoding) to
 * write the size value. 
 * 
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @param   data        Size value. 
 * @return              Operation status. 
 */
kInlineFx(kStatus) kSerializer_WriteSize(kSerializer serializer, kSize data)
{
    return xkSerializer_WriteSizePrimitive(serializer, data); 
}

/** 
 * Writes a kSize array. 
 *
 * This method uses the current size encoding (specified with kSerializer_SetSizeEncoding) to
 * write the size values. 
 *
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @param   data        Array to write. 
 * @param   count       Count of array elements. 
 * @return              Operation status. 
 */
kFx(kStatus) kSerializer_WriteSizeArray(kSerializer serializer, const kSize* data, kSize count);

/** 
 * Reads a kSize value. 
 *
 * This method uses the current size encoding (specified with kSerializer_SetSizeEncoding) to
 * read the size value. 
 *
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @param   data        Receives size value. 
 * @return              Operation status. 
 */
kInlineFx(kStatus) kSerializer_ReadSize(kSerializer serializer, kSize* data)
{
    return xkSerializer_ReadSizePrimitive(serializer, data); 
}

/** 
 * Reads a kSize array. 
 * 
 * This method uses the current size encoding (specified with kSerializer_SetSizeEncoding) to
 * read the size values. 
 *
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @param   data        Receives the array elements.
 * @param   count       Count of array elements to read. 
 * @return              Operation status. 
 */
kFx(kStatus) kSerializer_ReadSizeArray(kSerializer serializer, kSize* data, kSize count);

/** 
 * Writes a kSSize value. 
 *
 * This method uses the current size encoding (specified with kSerializer_SetSizeEncoding) to
 * write the size value. 
 * 
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @param   data        SSize value. 
 * @return              Operation status. 
 */
kInlineFx(kStatus) kSerializer_WriteSSize(kSerializer serializer, kSSize data)
{
    return xkSerializer_WriteSSizePrimitive(serializer, data); 
}

/** 
 * Writes a kSSize array. 
 *
 * This method uses the current size encoding (specified with kSerializer_SetSizeEncoding) to
 * write the size values. 
 *
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @param   data        Array to write. 
 * @param   count       Count of array elements. 
 * @return              Operation status. 
 */
kFx(kStatus) kSerializer_WriteSSizeArray(kSerializer serializer, const kSSize* data, kSize count);

/** 
 * Reads a kSSize value. 
 *
 * This method uses the current size encoding (specified with kSerializer_SetSizeEncoding) to
 * read the size value. 
 *
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @param   data        Receives size value. 
 * @return              Operation status. 
 */
kInlineFx(kStatus) kSerializer_ReadSSize(kSerializer serializer, kSSize* data)
{
    return xkSerializer_ReadSSizePrimitive(serializer, data); 
}

/** 
 * Reads a kSSize array. 
 * 
 * This method uses the current size encoding (specified with kSerializer_SetSizeEncoding) to
 * read the size values. 
 *
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @param   data        Receives the array elements.
 * @param   count       Count of array elements to read. 
 * @return              Operation status. 
 */
kFx(kStatus) kSerializer_ReadSSizeArray(kSerializer serializer, kSSize* data, kSize count);

/**
* Determines whether a value type is memory-compatible with the serializer.
*
* A value type is memory compatible if host endianness matches the endianess of the
* serializer and the value type contains only tightly-packed primitive fields (no structure padding).
* If these conditions are met, then it is valid to directly de/serialize an instance of the value
* type as an array of bytes. This can be useful, in some cases, for optimization.
*
* @public              @memberof kSerializer
* @param   serializer  Serializer object.
* @param   type        Type instance.
* @return              Operation status.
*/
kInlineFx(kBool) kSerializer_IsMemoryCompatible(kSerializer serializer, kType type)
{
    return (kSerializer_Endianness(serializer) == kEndianness_Host()) && kType_IsPacked(type); 
}

/**
* Determines whether this serializer can write the specified object type.
*
* @public              @memberof kSerializer
* @param   serializer  Serializer object.
* @param   type        Type instance.
* @return              Operation status.
*/
kInlineFx(kBool) kSerializer_CanWriteType(kSerializer serializer, kType type)
{
    return xkSerializer_VTable(serializer)->VCanWriteType(serializer, type);
}

/** 
 * Writes a type code. 
 *
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @param   type        Type instance. 
 * @param   version     Receives a reference to the type version that was written. 
 * @return              Operation status. 
 */
kInlineFx(kStatus) kSerializer_WriteType(kSerializer serializer, kType type, kTypeVersion* version)
{
    return xkSerializer_VTable(serializer)->VWriteType(serializer, type, version);
}


/** 
 * Reads a type code. 
 *
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @param   type        Receives type instance.
 * @param   version     Receives a reference to the type version that was read. 
 * @return              Operation status. 
 */
kInlineFx(kStatus) kSerializer_ReadType(kSerializer serializer, kType* type, kTypeVersion* version)
{
    return xkSerializer_VTable(serializer)->VReadType(serializer, type, version);
}

/** 
 * Writes an array of values or objects. 
 *
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @param   type        Type of items. 
 * @param   version     Type version.  
 * @param   items       Items to serialize. 
 * @param   count       Count of items. 
 * @return              Operation status. 
 */
kFx(kStatus) kSerializer_WriteItems(kSerializer serializer, kType type, kTypeVersion version, const void* items, kSize count); 

/** 
 * Writes an array of values or objects. 
 *
 * A debug assertion will be raised if the size of the specified items is not equal to the 
 * size of the specified type.
 * 
 * @relates                         kSerializer
 * @param   kSerializer_serializer  Serializer object. 
 * @param   kType_type              Type of items. 
 * @param   kTypeVersion_version    Type version.  
 * @param   TPtr_items              Strongly-typed pointer to items to serialize. 
 * @param   kSize_count             Count of items. 
 * @return                          Operation status. 
 */
#define kSerializer_WriteItemsT(kSerializer_serializer, kType_type, kTypeVersion_version, TPtr_items, kSize_count) \
    xkSerializer_WriteItemsT(kSerializer_serializer, kType_type, kTypeVersion_version, TPtr_items, kSize_count, sizeof(*(TPtr_items)))

/** 
 * Reads an array of values or objects. 
 *
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @param   type        Type of items. 
 * @param   version     Type version.  
 * @param   items       Receives deserialized items.
 * @param   count       Count of items. 
 * @return              Operation status. 
 */
kFx(kStatus) kSerializer_ReadItems(kSerializer serializer, kType type, kTypeVersion version, void* items, kSize count); 

/** 
 * Reads an array of values or objects. 
 *
 * A debug assertion will be raised if the size of the specified items is not equal to the 
 * size of the specified type.
 * 
 * @relates                         kSerializer
 * @param   kSerializer_serializer  Serializer object. 
 * @param   kType_type              Type of items. 
 * @param   kTypeVersion_version    Type version.  
 * @param   TPtr_items              Strongly-typed destination pointer for deserialized items. 
 * @param   kSize_count             Count of items. 
 * @return                          Operation status. 
 */
#define kSerializer_ReadItemsT(kSerializer_serializer, kType_type, kTypeVersion_version, TPtr_items, kSize_count) \
    xkSerializer_ReadItemsT(kSerializer_serializer, kType_type, kTypeVersion_version, TPtr_items, kSize_count, sizeof(*(TPtr_items)))

/** 
 * Begins writing a measured section of data, using an 8, 16, 32, or 64-bit integer
 * to record the size. 
 *
 * BeginWrite can be called multiple times before calling EndWrite, in order to nest write sections. 
 * Each call to BeginWrite pushes information onto a stack; this information is popped from the stack
 * by calling EndWrite.
 * 
 * Accordingly, it is the caller's responsibility to ensure that either 1) EndWrite is called once 
 * for each call to BeginWrite, or 2) the kSerializer_Reset method is called to clear serialization 
 * state in the event of errors. Note that some kSerializer subclasses, including kDat5Serializer and 
 * kDat6Serializer, will automatically call kSerializer_Reset if errors are encountered during object 
 * serialization. However, this behaviour not guaranteed for other serialization classes.
 *
 * @public                  @memberof kSerializer
 * @param   serializer      Serializer object. 
 * @param   sizeType        Type of size field (k8u, k16u, k32u, or k64u)
 * @param   includeSize     Include the size of the size field in the written size?
 * @return                  Operation status. 
 */
kFx(kStatus) kSerializer_BeginWrite(kSerializer serializer, kType sizeType, kBool includeSize); 

/** 
 * Ends writing a measured data section. 
 *
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @return              Operation status. 
 */
kFx(kStatus) kSerializer_EndWrite(kSerializer serializer); 

/** 
 * Begins reading a measured data section. 
 *
 * BeginRead can be called multiple times before calling EndRead, in order to nest read sections. 
 * Each call to BeginRead pushes information onto a stack; this information is popped from the stack
 * by calling EndRead.
 * 
 * Accordingly, it is the caller's responsibility to ensure that either 1) EndRead is called once 
 * for each call to BeginRead, or 2) the kSerializer_Reset method is called to clear serialization 
 * state in the event of errors. Note that some kSerializer subclasses, including kDat5Serializer and 
 * kDat6Serializer, will automatically call kSerializer_Reset if errors are encountered during object 
 * deserialization. However, this behaviour not guaranteed for other serialization classes.
 *
 * @public                  @memberof kSerializer
 * @param   serializer      Serializer object. 
 * @param   sizeType        Type of size field (k8u, k16u, k32u, or k64u)
 * @param   includeSize     Size of the size field was included in the recorded size?
 * @return                  Operation status. 
 */
kFx(kStatus) kSerializer_BeginRead(kSerializer serializer, kType sizeType, kBool includeSize); 

/** 
 * Ends reading a measured data section. 
 *
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @return              Operation status. 
 */
kFx(kStatus) kSerializer_EndRead(kSerializer serializer); 

/** 
 * Determines whether the current measured read section has more bytes.
 *
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @return              kTRUE if no more bytes available; otherwise, kFALSE.
 */
kInlineFx(kBool) kSerializer_ReadCompleted(kSerializer serializer)
{
    kObj(kSerializer, serializer);

    kAssert(kArrayList_Count(obj->readSections) > 0); 

    return kStream_BytesRead(obj->readStream) >= kPointer_ReadAs(kArrayList_Last(obj->readSections), k64u); 
}

/** 
 * Reads and discards a specified number of bytes. 
 *
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @param   offset      Number of bytes to read and discard. 
 * @return              Operation status. 
 */
kInlineFx(kStatus) kSerializer_AdvanceRead(kSerializer serializer, kSize offset)
{
    if (offset == 0)
    {
        return kOK; 
    }

    return xkSerializer_AdvanceReadImpl(serializer, offset); 
}

/** 
 * Flushes the serializer write buffer to the underlying stream.
 * 
 * kSerializer does not automatically flush its internal write buffer when the serializer is destroyed. 
 * The Flush method must be used to ensure that all bytes are written to the underlying stream.
 * 
 * The kSerializer_Flush method calls kStream_Flush on the underlying stream.
 *
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @return              Operation status. 
 */
kInlineFx(kStatus) kSerializer_Flush(kSerializer serializer)
{
    return xkSerializer_FlushEx(serializer, kTRUE);
}

/** 
 * Discards any streaming context accumulated by the serializer. 
 * 
 * This method can be used to discard any context information from serialization/deserialization
 * operations. This may be useful in the event of de/serialization errors, if the intention is to 
 * continue using the serializer object after errors have been encounted. 
 *
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @return              Operation status. 
 */
kInlineFx(kStatus) kSerializer_Reset(kSerializer serializer)
{
    return xkSerializer_VTable(serializer)->VReset(serializer);
}

/** 
 * Gets the underlying stream. 
 *
 * @public              @memberof kSerializer
 * @param   serializer  Serializer object. 
 * @return              Stream object. 
 */
kInlineFx(kStream) kSerializer_Stream(kSerializer serializer)
{  
    kObj(kSerializer, serializer);

    return obj->stream;
}

#endif

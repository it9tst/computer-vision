//
// KSerializer.h
// 
// Copyright (C) 2014-2021 by LMI Technologies Inc.
// Licensed under the MIT License.
// Redistributed files must retain the above copyright notice.
// 
#ifndef K_API_NET_SERIALIZER_H
#define K_API_NET_SERIALIZER_H

#include <kApi/Io/kSerializer.h>
#include "kApiNet/KAlloc.h"
#include "kApiNet/Data/KString.h"
#include "kApiNet/Io/KStream.h"

namespace Lmi3d
{
    namespace Zen 
    {
        namespace Io
        {
            /// <summary>Base class for binary serialization/deserialization classes.</summary>
            /// 
            /// <remarks>
            /// <para>KSerializer is a base class for binary serialization/deserialization classes, where each derived
            /// class typically implements a specific object serialization format.</para>
            /// 
            /// <para>KSerializer itself is also an instantiable class. The KSerializer base class does not provide
            /// the ability to serialize/deserialize objects or type information, but can be used to read/write primitive 
            /// data and arrays.</para>
            ///
            /// <para>KSerializer does not automatically flush its internal write buffer when the serializer is destroyed.
            /// The Flush method must be used to ensure that all bytes are written to the underlying stream.</para>
            ///
            /// <para>Default KRefStyle: Auto</para>
            /// </remarks>
            public ref class KSerializer : public KObject
            {
                KDeclareAutoClass(KSerializer, kSerializer)

            public:
                /// <summary>Initializes a new instance of the KSerializer class with the specified Zen object handle.</summary>           
                /// <param name="handle">Zen object handle.</param>
                KSerializer(IntPtr handle)
                    : KObject(handle, DefaultRefStyle)
                {}

                /// <inheritdoc cref="KSerializer(KStream^)" />
                /// <param name="refStyle">Reference managment style.</param>
                KSerializer(IntPtr handle, KRefStyle refStyle)
                    : KObject(handle, refStyle)
                {}

                /// <summary>Initializes a new instance of the KSerializer class.</summary>       
                /// 
                /// <remarks>
                /// This method constructs an instance of the KSerializer base class, which can be used to de/serialize primitive
                /// values and arrays. 
                /// </remarks>
                /// 
                /// <param name="stream">Stream for reading/writing.</param>
                KSerializer(KStream^ stream)
                    :KObject(DefaultRefStyle)
                {
                    kSerializer handle = kNULL;

                    KCheck(kSerializer_Construct(&handle, KToHandle(stream), kNULL, kNULL));
                    KAdjustRef(KToHandle(stream), kTRUE, Nullable<KRefStyle>());

                    Handle = handle;
                }

                /// <inheritdoc cref="KSerializer(KStream^)" />
                /// <param name="allocator">Memory allocator.</param>
                KSerializer(KStream^ stream, KAlloc^ allocator)
                    :KObject(DefaultRefStyle)
                {
                    kSerializer handle = kNULL;

                    KCheck(kSerializer_Construct(&handle, KToHandle(stream), kNULL, KToHandle(allocator))); 
                    KAdjustRef(KToHandle(stream), kTRUE, Nullable<KRefStyle>());

                    Handle = handle;
                }

                /// <summary>Sets the version to use when serializing types.</summary>
                /// 
                /// <remarks>
                /// Some serialization formats use a versioning scheme in which a separate version is given
                /// for each assembly, while others use a single version that is applied to types from all
                /// assemblies. If a single version is used for assemblies, set the assembly argument to null.
                /// </remarks>
                /// 
                /// <param name="assembly">Assembly object (or kNULL).</param>
                /// <param name="version">Desired version.</param>
                void SetVersion(KAssembly^ assembly, KVersion version)
                {
                    KCheck(kSerializer_SetVersion(Handle, KToHandle(assembly), version)); 
                }

                /// <summary>Gets or sets the number of bytes (4 or 8) used to encode/decode kSize and kSSize values.</summary>
                /// 
                /// <remarks>
                /// The default size encoding for kSerializer is 4 bytes. This value can be overridden by derived classes.
                /// </remarks>
                property k32s SizeEncoding
                {                
                    k32s get()
                    {
                        return kSerializer_SizeEncoding(Handle); 
                    }

                    void set(k32s byteCount)
                    {
                        KCheck(kSerializer_SetSizeEncoding(Handle, (k32u)byteCount));
                    }
                }

                /// <summary>Writes an object to the underlying stream.</summary>
                /// 
                /// <param name="object">Object to be serialized.</param>
                void WriteObject(KObject^ object)
                {
                    KCheck(kSerializer_WriteObject(Handle, KToHandle(object))); 
                }

                /// <summary>Reads an object from the underlying stream.</summary>
                ///
                /// <remarks>
                /// <para>Use KObject.Dispose to free the object returned by this method.</para>
                ///
                /// <para>Because null objects can be serialized using KSerializer.WriteObject, the object returned 
                /// by this method can be null.</para>
                /// </remarks>
                /// 
                /// <typeparam name="T">Type of object to be deserialized.</typeparam>
                /// <returns>Deserialized object.</returns>
                generic <typename T>
                T ReadObject()
                {
                    return ReadObject<T>(nullptr); 
                }

                /// <inheritdoc cref="ReadObject()" />
                /// <param name="allocator">Memory allocator.</param>
                generic <typename T> 
                T ReadObject(KAlloc^ allocator)
                {
                    kObject object = kNULL; 

                    KCheck(kSerializer_ReadObject(Handle, &object, KToHandle(allocator))); 

                    try
                    {
                        return KToObject<T>(object);
                    }
                    catch (...)
                    {
                        kObject_Dispose(object);
                        throw;
                    }                    
                }

                /// <summary>Writes a string as a null-terminated utf8 character array.</summary>
                ///
                /// <param name="string">String to write.</param>
                void WriteTextArray(String^ string)
                {
                    KString text(string); 

                    KCheck(kSerializer_WriteText(Handle, text.CharPtr)); 
                }

                /// <summary>Reads a string as a null-terminated utf8 character array.</summary>
                /// 
                /// <param name="capacity">Maximum number of characters to read, including the null terminator (must be greater than zero).</param>
                /// <returns>String that was read.</returns>
                String^ ReadTextArray(k64s capacity)
                {
                    KString str;
                    kString strHandle = KToHandle(%str);

                    KCheckArgs(capacity > 0); 

                    KCheck(kString_Reserve(strHandle, (kSize)capacity)); 

                    KCheck(kSerializer_ReadText(Handle, kString_Chars(strHandle), (kSize)capacity));

                    KCheck(kString_SetLength(strHandle, kStrLength(kString_Chars(strHandle)))); 

                    return str.ToString(); 
                }

                /// <summary>Writes the string length as a KSize value and the string content as a KChar array.</summary>
                ///
                /// <param name="string">String to write.</param>
                void WriteString(String^ string)
                {
                    KString text(string);

                    KCheck(kSerializer_WriteSize(Handle, (kSize)text.Length)); 
                    KCheck(kSerializer_WriteCharArray(Handle, text.CharPtr, (kSize)text.Length)); 
                }

                /// <summary>Reads the string length as a KSize value and the string content as a KChar array.</summary>
                ///
                /// <returns>String that was read.</returns>
                String^ ReadString()
                {
                    KString text; 
                    kSize length; 
                    
                    KCheck(kSerializer_ReadSize(Handle, &length));

                    text.SetLength((k64s)length);
                    KCheck(kSerializer_ReadCharArray(Handle, text.CharPtr, length)); 

                    return text.ToString(); 
                }
              
                /// <summary>Writes a Byte value.</summary>
                /// 
                /// <param name="data">Value to write.</param>
                void Write8u(k8u data)
                {
                    KCheck(kSerializer_Write8u(Handle, data));
                }

                /// <summary>Writes a Byte array.</summary>
                /// 
                /// <param name="data">Array to write.</param>
                /// <param name="count">Count of array elements.</param>
                void Write8uArray(IntPtr data, k64s count)
                {
                    KCheck(kSerializer_Write8uArray(Handle, (const k8u*)data.ToPointer(), (kSize)count));
                }

                /// <summary>Writes a Byte array.</summary>
                /// 
                /// <param name="data">Array to write.</param>
                /// <param name="start">Offset into array.</param>
                /// <param name="count">Count of array elements.</param>
                void Write8uArray(array<k8u>^ data, k32s start, k32s count)
                {
                    KCheckArgs((start + count) <= data->Length);

                    pin_ptr<k8u> dataPtr = &data[start];

                    KCheck(kSerializer_Write8uArray(Handle, dataPtr, (kSize)count));
                }

                /// <summary>Reads a Byte value.</summary>
                /// 
                /// <returns>Value that was read.</returns>
                k8u Read8u()
                {
                    k8u data;

                    KCheck(kSerializer_Read8u(Handle, &data));

                    return data;
                }

                /// <summary>Reads a Byte array.</summary>
                /// 
                /// <param name="data">Receives the array elements.</param>
                /// <param name="count">Count of array elements to read.</param>
                void Read8uArray(IntPtr data, k64s count)
                {
                    KCheck(kSerializer_Read8uArray(Handle, (k8u*)data.ToPointer(), (kSize)count));
                }

                /// <summary>Reads a Byte array.</summary>
                /// 
                /// <param name="data">Receives the array elements.</param>
                /// <param name="start">Offset into array.</param>
                /// <param name="count">Count of array elements to read.</param>
                void Read8uArray(array<k8u>^ data, k32s start, k32s count)
                {
                    KCheckArgs((start + count) <= data->Length);

                    pin_ptr<k8u> dataPtr = &data[start];

                    KCheck(kSerializer_Read8uArray(Handle, dataPtr, (kSize)count));
                }

                /// <summary>Reads a Byte array.</summary>
                /// 
                /// <param name="count">Count of array elements to read.</param>
                /// <returns>Array that was read.</returns>
                array<k8u>^ Read8uArray(k32s count)
                {
                    array<k8u>^ data = gcnew array<k8u>(count); 
                    pin_ptr<k8u> dataPtr = &data[0]; 

                    KCheck(kSerializer_Read8uArray(Handle, dataPtr, (kSize)count));

                    return data; 
                }

                /// <summary>Writes an SByte value.</summary>
                /// 
                /// <param name="data">Value to write.</param>
                void Write8s(k8s data)
                {
                    KCheck(kSerializer_Write8s(Handle, data));
                }

                /// <summary>Writes an SByte array.</summary>
                /// 
                /// <param name="data">Array to write.</param>
                /// <param name="count">Count of array elements.</param>
                void Write8sArray(IntPtr data, k64s count)
                {
                    KCheck(kSerializer_Write8sArray(Handle, (const k8s*)data.ToPointer(), (kSize)count)); 
                }

                /// <summary>Writes an SByte array.</summary>
                /// 
                /// <param name="data">Array to write.</param>
                /// <param name="start">Offset into array.</param>
                /// <param name="count">Count of array elements.</param>
                void Write8sArray(array<k8s>^ data, k32s start, k32s count)
                {
                    KCheckArgs((start + count) <= data->Length); 

                    pin_ptr<k8s> dataPtr = &data[start]; 

                    KCheck(kSerializer_Write8sArray(Handle, dataPtr, (kSize)count)); 
                }

                /// <summary>Reads an SByte value.</summary>
                /// 
                /// <returns>Value that was read.</returns>
                k8s Read8s()
                {
                    k8s data; 

                    KCheck(kSerializer_Read8s(Handle, &data)); 

                    return data; 
                }

                /// <summary>Reads an SByte array.</summary>
                /// 
                /// <param name="data">Receives the array elements.</param>
                /// <param name="count">Count of array elements to read.</param>
                void Read8sArray(IntPtr data, k64s count)
                {
                    KCheck(kSerializer_Read8sArray(Handle, (k8s*)data.ToPointer(), (kSize)count));
                }

                /// <summary>Reads an SByte array.</summary>
                /// 
                /// <param name="data">Receives the array elements.</param>
                /// <param name="start">Offset into array.</param>
                /// <param name="count">Count of array elements to read.</param>
                void Read8sArray(array<k8s>^ data, k32s start, k32s count)
                {
                    KCheckArgs((start + count) <= data->Length); 

                    pin_ptr<k8s> dataPtr = &data[start]; 

                    KCheck(kSerializer_Read8sArray(Handle, dataPtr, (kSize)count));
                }

                /// <summary>Reads an SByte array.</summary>
                /// 
                /// <param name="count">Count of array elements to read.</param>
                /// <returns>Array that was read.</returns>
                array<k8s>^ Read8sArray(k32s count)
                {
                    array<k8s>^ data = gcnew array<k8s>(count); 
                    pin_ptr<k8s> dataPtr = &data[0]; 

                    KCheck(kSerializer_Read8sArray(Handle, dataPtr, (kSize)count));

                    return data; 
                }

                /// <summary>Writes a UInt16 value.</summary>
                /// 
                /// <param name="data">Value to write.</param>
                void Write16u(k16u data)
                {
                    KCheck(kSerializer_Write16u(Handle, data));
                }

                /// <summary>Writes a UInt16 array.</summary>
                /// 
                /// <param name="data">Array to write.</param>
                /// <param name="count">Count of array elements.</param>
                void Write16uArray(IntPtr data, k64s count)
                {
                    KCheck(kSerializer_Write16uArray(Handle, (const k16u*)data.ToPointer(), (kSize)count)); 
                }

                /// <summary>Writes a UInt16 array.</summary>
                /// 
                /// <param name="data">Array to write.</param>
                /// <param name="start">Offset into array.</param>
                /// <param name="count">Count of array elements.</param>
                void Write16uArray(array<k16u>^ data, k32s start, k32s count)
                {
                    KCheckArgs((start + count) <= data->Length); 

                    pin_ptr<k16u> dataPtr = &data[start]; 

                    KCheck(kSerializer_Write16uArray(Handle, dataPtr, (kSize)count)); 
                }

                /// <summary>Reads a UInt16 value.</summary>
                /// 
                /// <returns>Value that was read.</returns>
                k16u Read16u()
                {
                    k16u data; 

                    KCheck(kSerializer_Read16u(Handle, &data)); 

                    return data; 
                }

                /// <summary>Reads a UInt16 array.</summary>
                /// 
                /// <param name="data">Receives the array elements.</param>
                /// <param name="count">Count of array elements to read.</param>
                void Read16uArray(IntPtr data, k64s count)
                {
                    KCheck(kSerializer_Read16uArray(Handle, (k16u*)data.ToPointer(), (kSize)count));
                }

                /// <summary>Reads a UInt16 array.</summary>
                /// 
                /// <param name="data">Receives the array elements.</param>
                /// <param name="start">Offset into array.</param>
                /// <param name="count">Count of array elements to read.</param>
                void Read16uArray(array<k16u>^ data, k32s start, k32s count)
                {
                    KCheckArgs((start + count) <= data->Length); 

                    pin_ptr<k16u> dataPtr = &data[start]; 

                    KCheck(kSerializer_Read16uArray(Handle, dataPtr, (kSize)count));
                }

                /// <summary>Reads a UInt16 array.</summary>
                /// 
                /// <param name="count">Count of array elements to read.</param>
                /// <returns>Array that was read.</returns>
                array<k16u>^ Read16uArray(k32s count)
                {
                    array<k16u>^ data = gcnew array<k16u>(count); 
                    pin_ptr<k16u> dataPtr = &data[0]; 

                    KCheck(kSerializer_Read16uArray(Handle, dataPtr, (kSize)count));

                    return data; 
                }

                /// <summary>Writes an Int16 value.</summary>
                /// 
                /// <param name="data">Value to write.</param>
                void Write16s(k16s data)
                {
                    KCheck(kSerializer_Write16s(Handle, data));
                }

                /// <summary>Writes an Int16 array.</summary>
                /// 
                /// <param name="data">Array to write.</param>
                /// <param name="count">Count of array elements.</param>
                void Write16sArray(IntPtr data, k64s count)
                {
                    KCheck(kSerializer_Write16sArray(Handle, (const k16s*)data.ToPointer(), (kSize)count)); 
                }

                /// <summary>Writes an Int16 array.</summary>
                /// 
                /// <param name="data">Array to write.</param>
                /// <param name="start">Offset into array.</param>
                /// <param name="count">Count of array elements.</param>
                void Write16sArray(array<k16s>^ data, k32s start, k32s count)
                {
                    KCheckArgs((start + count) <= data->Length); 

                    pin_ptr<k16s> dataPtr = &data[start]; 

                    KCheck(kSerializer_Write16sArray(Handle, dataPtr, (kSize)count)); 
                }

                /// <summary>Reads an Int16 value.</summary>
                /// 
                /// <returns>Value that was read.</returns>
                k16s Read16s()
                {
                    k16s data; 

                    KCheck(kSerializer_Read16s(Handle, &data)); 

                    return data; 
                }

                /// <summary>Reads an Int16 array.</summary>
                /// 
                /// <param name="data">Receives the array elements.</param>
                /// <param name="count">Count of array elements to read.</param>
                void Read16sArray(IntPtr data, k64s count)
                {
                    KCheck(kSerializer_Read16sArray(Handle, (k16s*)data.ToPointer(), (kSize)count));
                }

                /// <summary>Reads an Int16 array.</summary>
                /// 
                /// <param name="data">Receives the array elements.</param>
                /// <param name="start">Offset into array.</param>
                /// <param name="count">Count of array elements to read.</param>
                void Read16sArray(array<k16s>^ data, k32s start, k32s count)
                {
                    KCheckArgs((start + count) <= data->Length); 

                    pin_ptr<k16s> dataPtr = &data[start]; 

                    KCheck(kSerializer_Read16sArray(Handle, dataPtr, (kSize)count));
                }

                /// <summary>Reads an Int16 array.</summary>
                /// 
                /// <param name="count">Count of array elements to read.</param>
                /// <returns>Array that was read.</returns>
                array<k16s>^ Read16sArray(k32s count)
                {
                    array<k16s>^ data = gcnew array<k16s>(count); 
                    pin_ptr<k16s> dataPtr = &data[0]; 

                    KCheck(kSerializer_Read16sArray(Handle, dataPtr, (kSize)count));

                    return data; 
                }

                /// <summary>Writes a UInt32 value.</summary>
                /// 
                /// <param name="data">Value to write.</param>
                void Write32u(k32u data)
                {
                    KCheck(kSerializer_Write32u(Handle, data));
                }

                /// <summary>Writes a UInt32 array.</summary>
                /// 
                /// <param name="data">Array to write.</param>
                /// <param name="count">Count of array elements.</param>
                void Write32uArray(IntPtr data, k64s count)
                {
                    KCheck(kSerializer_Write32uArray(Handle, (const k32u*)data.ToPointer(), (kSize)count));
                }

                /// <summary>Writes a UInt32 array.</summary>
                /// 
                /// <param name="data">Array to write.</param>
                /// <param name="start">Offset into array.</param>
                /// <param name="count">Count of array elements.</param>
                void Write32uArray(array<k32u>^ data, k32s start, k32s count)
                {
                    KCheckArgs((start + count) <= data->Length);

                    pin_ptr<k32u> dataPtr = &data[start];

                    KCheck(kSerializer_Write32uArray(Handle, dataPtr, (kSize)count));
                }

                /// <summary>Reads a UInt32 value.</summary>
                /// 
                /// <returns>Value that was read.</returns>
                k32u Read32u()
                {
                    k32u data;

                    KCheck(kSerializer_Read32u(Handle, &data));

                    return data;
                }

                /// <summary>Reads a UInt32 array.</summary>
                /// 
                /// <param name="data">Receives the array elements.</param>
                /// <param name="count">Count of array elements to read.</param>
                void Read32uArray(IntPtr data, k64s count)
                {
                    KCheck(kSerializer_Read32uArray(Handle, (k32u*)data.ToPointer(), (kSize)count));
                }

                /// <summary>Reads a UInt32 array.</summary>
                /// 
                /// <param name="data">Receives the array elements.</param>
                /// <param name="start">Offset into array.</param>
                /// <param name="count">Count of array elements to read.</param>
                void Read32uArray(array<k32u>^ data, k32s start, k32s count)
                {
                    KCheckArgs((start + count) <= data->Length);

                    pin_ptr<k32u> dataPtr = &data[start];

                    KCheck(kSerializer_Read32uArray(Handle, dataPtr, (kSize)count));
                }

                /// <summary>Reads a UInt32 array.</summary>
                /// 
                /// <param name="count">Count of array elements to read.</param>
                /// <returns>Array that was read.</returns>
                array<k32u>^ Read32uArray(k32s count)
                {
                    array<k32u>^ data = gcnew array<k32u>(count);
                    pin_ptr<k32u> dataPtr = &data[0];

                    KCheck(kSerializer_Read32uArray(Handle, dataPtr, (kSize)count));

                    return data;
                }

                /// <summary>Writes an Int32 value.</summary>
                /// 
                /// <param name="data">Value to write.</param>
                void Write32s(k32s data)
                {
                    KCheck(kSerializer_Write32s(Handle, data));
                }

                /// <summary>Writes an Int32 array.</summary>
                /// 
                /// <param name="data">Array to write.</param>
                /// <param name="count">Count of array elements.</param>
                void Write32sArray(IntPtr data, k64s count)
                {
                    KCheck(kSerializer_Write32sArray(Handle, (const k32s*)data.ToPointer(), (kSize)count));
                }

                /// <summary>Writes an Int32 array.</summary>
                /// 
                /// <param name="data">Array to write.</param>
                /// <param name="start">Offset into array.</param>
                /// <param name="count">Count of array elements.</param>
                void Write32sArray(array<k32s>^ data, k32s start, k32s count)
                {
                    KCheckArgs((start + count) <= data->Length);

                    pin_ptr<k32s> dataPtr = &data[start];

                    KCheck(kSerializer_Write32sArray(Handle, dataPtr, (kSize)count));
                }

                /// <summary>Reads an Int32 value.</summary>
                /// 
                /// <returns>Value that was read.</returns>
                k32s Read32s()
                {
                    k32s data;

                    KCheck(kSerializer_Read32s(Handle, &data));

                    return data;
                }

                /// <summary>Reads an Int32 array.</summary>
                /// 
                /// <param name="data">Receives the array elements.</param>
                /// <param name="count">Count of array elements to read.</param>
                void Read32sArray(IntPtr data, k64s count)
                {
                    KCheck(kSerializer_Read32sArray(Handle, (k32s*)data.ToPointer(), (kSize)count));
                }

                /// <summary>Reads an Int32 array.</summary>
                /// 
                /// <param name="data">Receives the array elements.</param>
                /// <param name="start">Offset into array.</param>
                /// <param name="count">Count of array elements to read.</param>
                void Read32sArray(array<k32s>^ data, k32s start, k32s count)
                {
                    KCheckArgs((start + count) <= data->Length);

                    pin_ptr<k32s> dataPtr = &data[start];

                    KCheck(kSerializer_Read32sArray(Handle, dataPtr, (kSize)count));
                }

                /// <summary>Reads an Int32 array.</summary>
                /// 
                /// <param name="count">Count of array elements to read.</param>
                /// <returns>Array that was read.</returns>
                array<k32s>^ Read32sArray(k32s count)
                {
                    array<k32s>^ data = gcnew array<k32s>(count);
                    pin_ptr<k32s> dataPtr = &data[0];

                    KCheck(kSerializer_Read32sArray(Handle, dataPtr, (kSize)count));

                    return data;
                }

                /// <summary>Writes a UInt64 value.</summary>
                /// 
                /// <param name="data">Value to write.</param>
                void Write64u(k64u data)
                {
                    KCheck(kSerializer_Write64u(Handle, data));
                }

                /// <summary>Writes a UInt64 array.</summary>
                /// 
                /// <param name="data">Array to write.</param>
                /// <param name="count">Count of array elements.</param>
                void Write64uArray(IntPtr data, k64s count)
                {
                    KCheck(kSerializer_Write64uArray(Handle, (const k64u*)data.ToPointer(), (kSize)count));
                }

                /// <summary>Writes a UInt64 array.</summary>
                /// 
                /// <param name="data">Array to write.</param>
                /// <param name="start">Offset into array.</param>
                /// <param name="count">Count of array elements.</param>
                void Write64uArray(array<k64u>^ data, k32s start, k32s count)
                {
                    KCheckArgs((start + count) <= data->Length);

                    pin_ptr<k64u> dataPtr = &data[start];

                    KCheck(kSerializer_Write64uArray(Handle, dataPtr, (kSize)count));
                }

                /// <summary>Reads a UInt64 value.</summary>
                /// 
                /// <returns>Value that was read.</returns>
                k64u Read64u()
                {
                    k64u data;

                    KCheck(kSerializer_Read64u(Handle, &data));

                    return data;
                }

                /// <summary>Reads a UInt64 array.</summary>
                /// 
                /// <param name="data">Receives the array elements.</param>
                /// <param name="count">Count of array elements to read.</param>
                void Read64uArray(IntPtr data, k64s count)
                {
                    KCheck(kSerializer_Read64uArray(Handle, (k64u*)data.ToPointer(), (kSize)count));
                }

                /// <summary>Reads a UInt64 array.</summary>
                /// 
                /// <param name="data">Receives the array elements.</param>
                /// <param name="start">Offset into array.</param>
                /// <param name="count">Count of array elements to read.</param>
                void Read64uArray(array<k64u>^ data, k32s start, k32s count)
                {
                    KCheckArgs((start + count) <= data->Length);

                    pin_ptr<k64u> dataPtr = &data[start];

                    KCheck(kSerializer_Read64uArray(Handle, dataPtr, (kSize)count));
                }

                /// <summary>Reads a UInt64 array.</summary>
                /// 
                /// <param name="count">Count of array elements to read.</param>
                /// <returns>Array that was read.</returns>
                array<k64u>^ Read64uArray(k32s count)
                {
                    array<k64u>^ data = gcnew array<k64u>(count);
                    pin_ptr<k64u> dataPtr = &data[0];

                    KCheck(kSerializer_Read64uArray(Handle, dataPtr, (kSize)count));

                    return data;
                }

                /// <summary>Writes an Int64 value.</summary>
                /// 
                /// <param name="data">Value to write.</param>
                void Write64s(k64s data)
                {
                    KCheck(kSerializer_Write64s(Handle, data));
                }

                /// <summary>Writes an Int64 array.</summary>
                /// 
                /// <param name="data">Array to write.</param>
                /// <param name="count">Count of array elements.</param>
                void Write64sArray(IntPtr data, k64s count)
                {
                    KCheck(kSerializer_Write64sArray(Handle, (const k64s*)data.ToPointer(), (kSize)count));
                }

                /// <summary>Writes an Int64 array.</summary>
                /// 
                /// <param name="data">Array to write.</param>
                /// <param name="start">Offset into array.</param>
                /// <param name="count">Count of array elements.</param>
                void Write64sArray(array<k64s>^ data, k32s start, k32s count)
                {
                    KCheckArgs((start + count) <= data->Length);

                    pin_ptr<k64s> dataPtr = &data[start];

                    KCheck(kSerializer_Write64sArray(Handle, dataPtr, (kSize)count));
                }

                /// <summary>Reads an Int64 value.</summary>
                /// 
                /// <returns>Value that was read.</returns>
                k64s Read64s()
                {
                    k64s data;

                    KCheck(kSerializer_Read64s(Handle, &data));

                    return data;
                }

                /// <summary>Reads an Int64 array.</summary>
                /// 
                /// <param name="data">Receives the array elements.</param>
                /// <param name="count">Count of array elements to read.</param>
                void Read64sArray(IntPtr data, k64s count)
                {
                    KCheck(kSerializer_Read64sArray(Handle, (k64s*)data.ToPointer(), (kSize)count));
                }

                /// <summary>Reads an Int64 array.</summary>
                /// 
                /// <param name="data">Receives the array elements.</param>
                /// <param name="start">Offset into array.</param>
                /// <param name="count">Count of array elements to read.</param>
                void Read64sArray(array<k64s>^ data, k32s start, k32s count)
                {
                    KCheckArgs((start + count) <= data->Length);

                    pin_ptr<k64s> dataPtr = &data[start];

                    KCheck(kSerializer_Read64sArray(Handle, dataPtr, (kSize)count));
                }

                /// <summary>Reads an Int64 array.</summary>
                /// 
                /// <param name="count">Count of array elements to read.</param>
                /// <returns>Array that was read.</returns>
                array<k64s>^ Read64sArray(k32s count)
                {
                    array<k64s>^ data = gcnew array<k64s>(count);
                    pin_ptr<k64s> dataPtr = &data[0];

                    KCheck(kSerializer_Read64sArray(Handle, dataPtr, (kSize)count));

                    return data;
                }           

                /// <summary>Writes a Single value.</summary>
                /// 
                /// <param name="data">Value to write.</param>
                void Write32f(k32f data)
                {
                    KCheck(kSerializer_Write32f(Handle, data));
                }

                /// <summary>Writes a Single array.</summary>
                /// 
                /// <param name="data">Array to write.</param>
                /// <param name="count">Count of array elements.</param>
                void Write32fArray(IntPtr data, k64s count)
                {
                    KCheck(kSerializer_Write32fArray(Handle, (const k32f*)data.ToPointer(), (kSize)count));
                }

                /// <summary>Writes a Single array.</summary>
                /// 
                /// <param name="data">Array to write.</param>
                /// <param name="start">Offset into array.</param>
                /// <param name="count">Count of array elements.</param>
                void Write32fArray(array<k32f>^ data, k32s start, k32s count)
                {
                    KCheckArgs((start + count) <= data->Length);

                    pin_ptr<k32f> dataPtr = &data[start];

                    KCheck(kSerializer_Write32fArray(Handle, dataPtr, (kSize)count));
                }

                /// <summary>Reads a Single value.</summary>
                /// 
                /// <returns>Value that was read.</returns>
                k32f Read32f()
                {
                    k32f data;

                    KCheck(kSerializer_Read32f(Handle, &data));

                    return data;
                }

                /// <summary>Reads a Single array.</summary>
                /// 
                /// <param name="data">Receives the array elements.</param>
                /// <param name="count">Count of array elements to read.</param>
                void Read32fArray(IntPtr data, k64s count)
                {
                    KCheck(kSerializer_Read32fArray(Handle, (k32f*)data.ToPointer(), (kSize)count));
                }

                /// <summary>Reads a Single array.</summary>
                /// 
                /// <param name="data">Receives the array elements.</param>
                /// <param name="start">Offset into array.</param>
                /// <param name="count">Count of array elements to read.</param>
                void Read32fArray(array<k32f>^ data, k32s start, k32s count)
                {
                    KCheckArgs((start + count) <= data->Length);

                    pin_ptr<k32f> dataPtr = &data[start];

                    KCheck(kSerializer_Read32fArray(Handle, dataPtr, (kSize)count));
                }

                /// <summary>Reads a Single array.</summary>
                /// 
                /// <param name="count">Count of array elements to read.</param>
                /// <returns>Array that was read.</returns>
                array<k32f>^ Read32fArray(k32s count)
                {
                    array<k32f>^ data = gcnew array<k32f>(count);
                    pin_ptr<k32f> dataPtr = &data[0];

                    KCheck(kSerializer_Read32fArray(Handle, dataPtr, (kSize)count));

                    return data;
                }

                /// <summary>Writes a Double value.</summary>
                /// 
                /// <param name="data">Value to write.</param>
                void Write64f(k64f data)
                {
                    KCheck(kSerializer_Write64f(Handle, data));
                }

                /// <summary>Writes a Double array.</summary>
                /// 
                /// <param name="data">Array to write.</param>
                /// <param name="count">Count of array elements.</param>
                void Write64fArray(IntPtr data, k64s count)
                {
                    KCheck(kSerializer_Write64fArray(Handle, (const k64f*)data.ToPointer(), (kSize)count));
                }

                /// <summary>Writes a Double array.</summary>
                /// 
                /// <param name="data">Array to write.</param>
                /// <param name="start">Offset into array.</param>
                /// <param name="count">Count of array elements.</param>
                void Write64fArray(array<k64f>^ data, k32s start, k32s count)
                {
                    KCheckArgs((start + count) <= data->Length);

                    pin_ptr<k64f> dataPtr = &data[start];

                    KCheck(kSerializer_Write64fArray(Handle, dataPtr, (kSize)count));
                }

                /// <summary>Reads a Double value.</summary>
                /// 
                /// <returns>Value that was read.</returns>
                k64f Read64f()
                {
                    k64f data;

                    KCheck(kSerializer_Read64f(Handle, &data));

                    return data;
                }

                /// <summary>Reads a Double array.</summary>
                /// 
                /// <param name="data">Receives the array elements.</param>
                /// <param name="count">Count of array elements to read.</param>
                void Read64fArray(IntPtr data, k64s count)
                {
                    KCheck(kSerializer_Read64fArray(Handle, (k64f*)data.ToPointer(), (kSize)count));
                }

                /// <summary>Reads a Double array.</summary>
                /// 
                /// <param name="data">Receives the array elements.</param>
                /// <param name="start">Offset into array.</param>
                /// <param name="count">Count of array elements to read.</param>
                void Read64fArray(array<k64f>^ data, k32s start, k32s count)
                {
                    KCheckArgs((start + count) <= data->Length);

                    pin_ptr<k64f> dataPtr = &data[start];

                    KCheck(kSerializer_Read64fArray(Handle, dataPtr, (kSize)count));
                }

                /// <summary>Reads a Double array.</summary>
                /// 
                /// <param name="count">Count of array elements to read.</param>
                /// <returns>Array that was read.</returns>
                array<k64f>^ Read64fArray(k32s count)
                {
                    array<k64f>^ data = gcnew array<k64f>(count);
                    pin_ptr<k64f> dataPtr = &data[0];

                    KCheck(kSerializer_Read64fArray(Handle, dataPtr, (kSize)count));

                    return data;
                }   

                /// <summary>Writes a KSize value.</summary>
                /// 
                /// <param name="data">Value to write.</param>
                void WriteSize(KSize data)
                {
                    KCheck(kSerializer_WriteSize(Handle, data));
                }

                /// <summary>Writes a KSize array.</summary>
                /// 
                /// <param name="data">Array to write.</param>
                /// <param name="count">Count of array elements.</param>
                void WriteSizeArray(IntPtr data, k64s count)
                {
                    KCheck(kSerializer_WriteSizeArray(Handle, (const kSize*)data.ToPointer(), (kSize)count)); 
                }

                /// <summary>Writes a KSize array.</summary>
                /// 
                /// <param name="data">Array to write.</param>
                /// <param name="start">Offset into array.</param>
                /// <param name="count">Count of array elements.</param>
                void WriteSizeArray(array<KSize>^ data, k32s start, k32s count)
                {
                    KCheckArgs((start + count) <= data->Length); 

                    pin_ptr<KSize> dataPtr = &data[start]; 

                    KCheck(kSerializer_WriteSizeArray(Handle, (kSize*)dataPtr, (kSize)count)); 
                }

                /// <summary>Reads a KSize value.</summary>
                /// 
                /// <returns>Value that was read.</returns>
                KSize ReadSize()
                {
                    kSize data; 

                    KCheck(kSerializer_ReadSize(Handle, &data)); 

                    return data; 
                }

                /// <summary>Reads a KSize array.</summary>
                /// 
                /// <param name="data">Receives the array elements.</param>
                /// <param name="count">Count of array elements to read.</param>
                void ReadSizeArray(IntPtr data, k64s count)
                {
                    KCheck(kSerializer_ReadSizeArray(Handle, (kSize*)data.ToPointer(), (kSize)count));
                }

                /// <summary>Reads a KSize array.</summary>
                /// 
                /// <param name="data">Receives the array elements.</param>
                /// <param name="start">Offset into array.</param>
                /// <param name="count">Count of array elements to read.</param>
                void ReadSizeArray(array<KSize>^ data, k32s start, k32s count)
                {
                    KCheckArgs((start + count) <= data->Length); 

                    pin_ptr<KSize> dataPtr = &data[start]; 

                    KCheck(kSerializer_ReadSizeArray(Handle, (kSize*)dataPtr, (kSize)count));
                }

                /// <summary>Reads a KSize array.</summary>
                /// 
                /// <param name="count">Count of array elements to read.</param>
                /// <returns>Array that was read.</returns>
                array<KSize>^ ReadSizeArray(k32s count)
                {
                    array<KSize>^ data = gcnew array<KSize>(count); 
                    pin_ptr<KSize> dataPtr = &data[0]; 

                    KCheck(kSerializer_ReadSizeArray(Handle, (kSize*)dataPtr, (kSize)count));

                    return data; 
                }

                /// <summary>Writes a KSSize value.</summary>
                /// 
                /// <param name="data">Value to write.</param>
                void WriteSSize(KSSize data)
                {
                    KCheck(kSerializer_WriteSSize(Handle, data));
                }

                /// <summary>Writes a KSSize array.</summary>
                /// 
                /// <param name="data">Array to write.</param>
                /// <param name="count">Count of array elements.</param>
                void WriteSSizeArray(IntPtr data, k64s count)
                {
                    KCheck(kSerializer_WriteSSizeArray(Handle, (const kSSize*)data.ToPointer(), (kSize)count));
                }

                /// <summary>Writes a KSSize array.</summary>
                /// 
                /// <param name="data">Array to write.</param>
                /// <param name="start">Offset into array.</param>
                /// <param name="count">Count of array elements.</param>
                void WriteSSizeArray(array<KSSize>^ data, k32s start, k32s count)
                {
                    KCheckArgs((start + count) <= data->Length);

                    pin_ptr<KSSize> dataPtr = &data[start];

                    KCheck(kSerializer_WriteSSizeArray(Handle, (kSSize*)dataPtr, (kSize)count));
                }

                /// <summary>Reads a KSSize value.</summary>
                /// 
                /// <returns>Value that was read.</returns>
                KSSize ReadSSize()
                {
                    kSSize data;

                    KCheck(kSerializer_ReadSSize(Handle, &data));

                    return data;
                }

                /// <summary>Reads a KSSize array.</summary>
                /// 
                /// <param name="data">Receives the array elements.</param>
                /// <param name="count">Count of array elements to read.</param>
                void ReadSSizeArray(IntPtr data, k64s count)
                {
                    KCheck(kSerializer_ReadSSizeArray(Handle, (kSSize*)data.ToPointer(), (kSize)count));
                }

                /// <summary>Reads a KSSize array.</summary>
                /// 
                /// <param name="data">Receives the array elements.</param>
                /// <param name="start">Offset into array.</param>
                /// <param name="count">Count of array elements to read.</param>
                void ReadSSizeArray(array<KSSize>^ data, k32s start, k32s count)
                {
                    KCheckArgs((start + count) <= data->Length);

                    pin_ptr<KSSize> dataPtr = &data[start];

                    KCheck(kSerializer_ReadSSizeArray(Handle, (kSSize*) dataPtr, (kSize)count));
                }

                /// <summary>Reads a KSSize array.</summary>
                /// 
                /// <param name="count">Count of array elements to read.</param>
                /// <returns>Array that was read.</returns>
                array<KSSize>^ ReadSSizeArray(k32s count)
                {
                    array<KSSize>^ data = gcnew array<KSSize>(count);
                    pin_ptr<KSSize> dataPtr = &data[0];

                    KCheck(kSerializer_ReadSSizeArray(Handle, (kSSize*)dataPtr, (kSize)count));

                    return data;
                }   

                /// <summary> Determines whether a value type is memory-compatible with the serializer.</summary>
                /// 
                /// <param name="type">Type instance.</param>
                /// <returns>True if the type is memory compatible.</returns>
                bool IsMemoryCompatible(KType^ type)
                {
                    return KToBool(kSerializer_IsMemoryCompatible(Handle, KToHandle(type)));
                }

                /// <summary>Writes a type code.</summary>
                /// 
                /// <param name="type">Type instance.</param>
                /// <returns>Type version that was written.</returns>
                KTypeVersion WriteType(KType^ type)
                {
                    kTypeVersion version;

                    KCheck(kSerializer_WriteType(Handle, KToHandle(type), &version)); 

                    return KTypeVersion(KToHandle(type), version); 
                }

                /// <summary>Reads a type code.</summary>
                /// 
                /// <param name="version">Receives type version.</param>
                /// <returns>Type instance.</returns>
                KType^ ReadType([Out] KTypeVersion% version)
                {
                    kType type = kNULL; 
                    kTypeVersion ver = kNULL; 

                    KCheck(kSerializer_ReadType(Handle, &type, &ver)); 

                    version = KTypeVersion(type, ver);

                    return gcnew KType(type); 
                }

                /// <summary>Reads a type code.</summary>
                /// 
                /// <returns>Type instance.</returns>
                KType^ ReadType()
                {
                    kType type = kNULL; 
                    kTypeVersion ver = kNULL; 

                    KCheck(kSerializer_ReadType(Handle, &type, &ver)); 

                    return gcnew KType(type); 
                }

                /// <summary>Writes an array of values.</summary>
                /// 
                /// <param name="type">Type of items.</param>
                /// <param name="version">Type version.</param>
                /// <param name="items">Items to serialize.</param>
                /// <param name="count">Count of items.</param>
                void WriteValues(KType^ type, KTypeVersion version, IntPtr items, k64s count)
                {
                    KCheck(kSerializer_WriteItems(Handle, KToHandle(type), version.Handle, items.ToPointer(), (kSize)count)); 
                }

                /// <inheritdoc cref="WriteValues(KType^, KTypeVersion, IntPtr, k64s)" />
                /// <param name="items">Items to serialize.</param>
                /// <param name="start">Index of first array item.</param>
                /// <param name="count">Count of items.</param>
                generic <typename T> where T : value struct
                void WriteValues(KType^ type, KTypeVersion version, array<T>^ items, k32s start, k32s count)
                {
                    kType typeHandle = KToHandle(type);

                    KCheckArgs(kType_IsValue(typeHandle) && (sizeof(T) == kType_Size(typeHandle)));
                    KCheckArgs((start + count) <= items->Length); 

                    pin_ptr<T> itemPtr = &items[start];

                    KCheck(kSerializer_WriteItems(Handle, typeHandle, version.Handle, itemPtr, (kSize)count));
                }           

                /// <summary>Reads an array of values.</summary>
                /// 
                /// <param name="type">Type of items.</param>
                /// <param name="version">Type version.</param>
                /// <param name="items">Receives deserialized items.</param>
                /// <param name="count">Count of items.</param>
                void ReadValues(KType^ type, KTypeVersion version, IntPtr items, k64s count)
                {
                    KCheck(kSerializer_ReadItems(Handle, KToHandle(type), version.Handle, items.ToPointer(), (kSize)count));
                }

                /// <inheritdoc cref="ReadValues(KType^, KTypeVersion, IntPtr, k64s)" />
                ///
                /// <param name="items">Items to serialize.</param>
                /// <param name="start">Index of first array item.</param>
                generic <typename T> where T : value struct
                void ReadValues(KType^ type, KTypeVersion version, array<T>^ items, k32s start, k32s count)
                {
                    kType typeHandle = KToHandle(type);

                    KCheckArgs(kType_IsValue(typeHandle) && (sizeof(T) == kType_Size(typeHandle)));
                    KCheckArgs((start + count) <= items->Length); 

                    pin_ptr<T> itemPtr = &items[start];

                    KCheck(kSerializer_ReadItems(Handle, typeHandle, version.Handle, itemPtr, (kSize)count));
                }

                /// <inheritdoc cref="ReadValues(KType^, KTypeVersion, IntPtr, k64s)" />
                /// 
                /// <returns>Array of deserialized values.</returns>
                generic <typename T> where T : value struct
                array<T>^ ReadValues(KType^ type, KTypeVersion version, k32s count)
                {
                    array<T>^ items = gcnew array<T>(count); 
                    kType typeHandle = KToHandle(type);

                    KCheckArgs(kType_IsValue(typeHandle) && (sizeof(T) == kType_Size(typeHandle)));

                    pin_ptr<T> itemPtr = &items[0];

                    KCheck(kSerializer_ReadItems(Handle, typeHandle, version.Handle, itemPtr, (kSize)count));

                    return items;
                }
           
                /// <summary>Begins writing a measured section of data, using an 8, 16, 32, or 64-bit unsigned integer
                /// to record the size.</summary>
                /// 
                /// <param name="sizeType">Type of size field (K8u, K16u, K32u, or K64u).</param>
                /// <param name="includeSize">Include the size of the size field in the written size?</param>
                void BeginWrite(KType^ sizeType, bool includeSize)
                {
                    KCheck(kSerializer_BeginWrite(Handle, KToHandle(sizeType), includeSize)); 
                }

                /// <summary>Ends writing a measured data section.</summary>
                void EndWrite()
                {
                    KCheck(kSerializer_EndWrite(Handle)); 
                }

                /// <summary>Begins reading a measured data section.</summary>
                /// 
                /// <param name="sizeType">Type of size field (k8u, k16u, k32u, or k64u).</param>
                /// <param name="includeSize">Size of the size field was included in the recorded size?</param>
                void BeginRead(KType^ sizeType, bool includeSize)
                {
                    KCheck(kSerializer_BeginRead(Handle, KToHandle(sizeType), includeSize));
                }

                /// <summary>Ends reading a measured data section.</summary>
                void EndRead()
                {
                    KCheck(kSerializer_EndRead(Handle));
                }

                /// <summary>Determines whether the current measured read section has more bytes.</summary>
                /// 
                /// <returns>true if no more bytes available; otherwise, false.</returns>
                bool IsReadCompleted()
                {
                    return KToBool(kSerializer_ReadCompleted(Handle)); 
                }

                /// <summary>Reads and discards a specified number of bytes.</summary>
                /// 
                /// <param name="offset">Number of bytes to read and discard.</param>
                void AdvanceRead(k64s offset)
                {
                    KCheck(kSerializer_AdvanceRead(Handle, (kSize)offset)); 
                }

                /// <summary>Flushes the serializer write buffer to the underlying stream.</summary>
                ///
                /// <remarks>
                /// <para>KSerializer does not automatically flush its internal write buffer when the serializer is destroyed. 
                /// The Flush method must be used to ensure that all bytes are written to the underlying stream.</para>
                ///
                /// <para>The KSerializer.Flush method calls KStream.Flush on the underlying stream.</para>
                /// </remarks>
                void Flush()
                {
                    KCheck(kSerializer_Flush(Handle)); 
                }

                /// <summary>Discards any streaming context accumulated by the serializer.</summary>
                void Reset()
                {
                    KCheck(kSerializer_Reset(Handle));
                }

                /// <summary>Gets the underlying stream.</summary>
                /// 
                /// <returns>Stream object.</returns>
                property KStream^ Stream
                {
                    KStream^ get() 
                    { 
                        kStream stream = kSerializer_Stream(Handle);
                        KAdjustRef(stream, kTRUE, Nullable<KRefStyle>());

                        return KToObject<KStream^>(stream);
                    }
                }

            protected:
                KSerializer(KRefStyle refStyle) : KObject(refStyle) {}

                virtual void OnDisposing() override
                {
                    KAdjustRef(kSerializer_Stream(Handle), kFALSE, Nullable<KRefStyle>());
                }
            };
        }
    }
}

#endif

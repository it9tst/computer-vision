/** 
 * @file    kDat6Serializer.x.h
 *
 * @internal
 * Copyright (C) 2012-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_DAT6_SERIALIZER_X_H
#define K_API_DAT6_SERIALIZER_X_H

#include <kApi/Io/kSerializer.h>

typedef k32s kDat6CompressionStyle; 

#define xkDAT6_FORMAT                             "kdat6"        //format name, used for version info lookup

#define xkDAT6_SERIALIZER_CMD_SYNC                (0xA0)         //resynchronize receiver (clear type dictionary, accept size encoding)
#define xkDAT6_SERIALIZER_CMD_NULL                (0xA1)         //null object
#define xkDAT6_SERIALIZER_CMD_OBJECT              (0xA2)         //non-null object
#define xkDAT6_SERIALIZER_CMD_COMPRESSED_OBJECT   (0xA3)         //non-null compressed object

typedef struct xkDat6SerializerTypeInfo
{
    kType type;                     //type object
    kTypeVersion version;           //type object serialization version information
    k16u id;                        //type dictionary id
} xkDat6SerializerTypeInfo; 

kDeclareValueEx(k, xkDat6SerializerTypeInfo, kValue)

typedef struct kDat6SerializerStatic
{
    kLock lock;                     //ensures exclusive access to static content
    kMap guidToInfo;                //used by reader; maps type guid to type info (kMap<kText64s, xkDat6SerializerTypeInfo)
    kMap guidToCompressor;          //used by reader; maps compressor type-id (kCompressionType) to compressor type object (kType)
} kDat6SerializerStatic; 

typedef struct kDat6SerializerVTable
{
    kSerializerVTable base; 
} kDat6SerializerVTable;

typedef struct kDat6SerializerClass
{    
    kSerializerClass base; 

    /* 
    *Private Fields
    */

    k32u objectSizeEncoding;                            //kSize encoding to be used for objects; may be different from encoding used for non-object data 
    kBool synchronized;                                 //if false, sender will prepend dictionary clear command to next dictionary message
    kBool dictionaryEnabled;                            //if false, writer will send a 'sync' command before each new object (avoids type dictionary)
    kSize objectDepth;                                  //recursive call depth for object read/write operations; flush writes when depth is zero
    
    kMap guidToInfo;                                    //used by reader; maps type guid to type info (kMap<kText64s, xkDat6SerializerTypeInfo>)
    kMap guidToCompressor;                              //used by reader; maps compressor type-id (kCompressionType) to compressor type object (kType)
    kArrayList readerInfo;                              //used by reader; index corresponds to type info (kArrayList<xkDat6SerializerTypeInfo>)
    kMap typeToInfo;                                    //used by writer; maps type to type info (kMap<kType, xkDat6SerializerTypeInfo>)

    kCompressionType currentCompressionAlgorithm;       //current compression algorithm id  
    k32s currentCompressionPreset;                      //current compression level
    kStream compressStream;                             //compresssion stream, layered over primary stream; optional

    kCompressionType currentDecompressionAlgorithm;     //current decompression algorithm id  
    kStream decompressStream;                           //decompresssion stream, layered over primary stream; optional
} kDat6SerializerClass;

kDeclareFullClassEx(k, kDat6Serializer, kSerializer)

/* 
* Protected methods. 
* 
* The kDat6Serializer class is extensible, but doing so can be challenging. The following methods
* are considered protected, but are more subject to change than the documented, protected methods
* in other classes such as kObject or kAlloc. Use with caution.
* 
* TODO: add API documentation for protected methods. 
*/

//[Protected]
kFx(kStatus) kDat6Serializer_VInit(kDat6Serializer serializer, kType type, kStream stream, kAlloc allocator); 

//[Protected]
kFx(kStatus) kDat6Serializer_VRelease(kDat6Serializer serializer); 

//[Protected]
kFx(kStatus) kDat6Serializer_VSetSizeEncoding(kDat6Serializer serializer, k32u byteCount); 

//[Protected]
kFx(kStatus) kDat6Serializer_VWriteObject(kDat6Serializer serializer, kObject object); 

//[Protected]
kFx(kStatus) kDat6Serializer_VReadObject(kDat6Serializer serializer, kObject* object, kAlloc allocator); 

/* 
* Private methods. 
*/

kFx(kStatus) xkDat6Serializer_InitStatic();
kFx(kStatus) xkDat6Serializer_ReleaseStatic();
kFx(kStatus) xkDat6Serializer_OnAssemblyLoad(kPointer receiver, kAssembly assembly, void* args);
kFx(kStatus) xkDat6Serializer_OnAssemblyUnload(kPointer receiver, kAssembly assembly, void* args);
kFx(kStatus) xkDat6Serializer_GetTypeLookups(kMap* guidToInfo, kMap* guidToCompressor);

kFx(kBool) xkDat6Serializer_VReset(kDat6Serializer serializer); 
kFx(kStatus) xkDat6Serializer_VWriteType(kDat6Serializer serializer, kType type, kTypeVersion* version); 
kFx(kStatus) xkDat6Serializer_VReadType(kDat6Serializer serializer, kType* type, kTypeVersion* version); 

kFx(kStatus) xkDat6Serializer_WriteSync(kDat6Serializer serializer); 
kFx(kStatus) xkDat6Serializer_ReadSync(kDat6Serializer serializer); 

kFx(kStatus) xkDat6Serializer_BeginCompression(kDat6Serializer serializer);
kFx(kStatus) xkDat6Serializer_EndCompression(kDat6Serializer serializer);

kFx(kStatus) xkDat6Serializer_BeginDecompression(kDat6Serializer serializer);
kFx(kStatus) xkDat6Serializer_EndDecompression(kDat6Serializer serializer);

#endif

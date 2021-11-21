/** 
 * @file    kDat5Serializer.x.h
 *
 * @internal
 * Copyright (C) 2008-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_DAT5_SERIALIZER_X_H
#define K_API_DAT5_SERIALIZER_X_H

#include <kApi/Io/kSerializer.h>

#define xkDAT5_FORMAT                    "kdat5"                    //format name, used for version info lookup
    
#define xkDAT5_SERIALIZER_CMD_OBJECT                    (1)         //FS5-compatible kDat5 protocol
#define xkDAT5_SERIALIZER_CMD_COMPRESSED_OBJECT         (2)         //compressed object support, introduced in FireSync 6.2.0

#define xkDAT5_SERIALIZER_CMD_NULL                      (-1)        //null object
#define xkDAT5_SERIALIZER_CMD_REF                       (-2)        //reference to identical object (not supported)

#define xkDAT5_SERIALIZER_CMD_REF                       (-2)        //reference to identical object (not supported)

//creates GUID for lookup, using type id and type version
kInlineFx(k32u) xkDat5Guid_Create(k32u typeId, k32u typeVersionId)
{
    return (typeVersionId << 16) | typeId; 
}

 //gets type id from GUID
kInlineFx(k32u) xkDat5Guid_Type(k32u guid)
{
    return guid & 0xFFFF; 
}

 //gets type version from GUID
kInlineFx(k32u) xkDat5Guid_Version(k32u guid)
{
    return (guid >> 16) & 0xFFFF;
}

typedef struct xkDat5SerializerTypeInfo
{
    kType type;                     //type object
    kTypeVersion version;           //type object serialization version information
    k32u id;                        //type guid
} xkDat5SerializerTypeInfo; 

kDeclareValueEx(k, xkDat5SerializerTypeInfo, kValue)

typedef struct kDat5SerializerStatic
{
    kLock lock;                     //ensures exclusive access to static content
    kMap guidToInfo;                //used by reader; maps type guid to type info (kMap<k32u, xkDat5SerializerTypeInfo>)
    kMap guidToCompressor;          //used by reader; maps compressor type-id (kCompressionType) to compressor type object (kType)
} kDat5SerializerStatic; 

typedef struct kDat5SerializerVTable
{
    kSerializerVTable base; 
} kDat5SerializerVTable;

typedef struct kDat5SerializerClass
{    
    kSerializerClass base; 

    /* 
    *Private Fields
    */

    kSize objectDepth;                                  //recursive call depth for object read/write operations; flush writes when depth is zero    
    kMap guidToInfo;                                    //used by reader; maps type guid to type info (kMap<k32u, xkDat5SerializerTypeInfo>)
    kMap guidToCompressor;                              //used by reader; maps compressor type-id (kCompressionType) to compressor type object (kType)
    kMap typeToInfo;                                    //used by writer; maps type to type info (kMap<kType, xkDat5SerializerTypeInfo>)

    kCompressionType currentCompressionAlgorithm;       //current compression algorithm id  
    k32s currentCompressionPreset;                      //current compression level
    kStream compressStream;                             //compresssion stream, layered over primary stream; optional

    kCompressionType currentDecompressionAlgorithm;     //current decompression algorithm id  
    kStream decompressStream;                           //decompresssion stream, layered over primary stream; optional
} kDat5SerializerClass;

kDeclareFullClassEx(k, kDat5Serializer, kSerializer)

/* 
* Protected methods. 
* 
* The kDat5Serializer class is extensible, but doing so can be challenging. The following methods
* are considered protected, but are more subject to change than the documented, protected methods
* in other classes such as kObject or kAlloc. Use with caution.
* 
* TODO: add API documentation for protected methods. 
*/

//[Protected]
kFx(kStatus) kDat5Serializer_VInit(kDat5Serializer serializer, kType type, kStream stream, kAlloc allocator); 

//[Protected]
kFx(kStatus) kDat5Serializer_VRelease(kDat5Serializer serializer); 

//[Protected]
kFx(kStatus) kDat5Serializer_VWriteObject(kDat5Serializer serializer, kObject object); 

//[Protected]
kFx(kStatus) kDat5Serializer_VReadObject(kDat5Serializer serializer, kObject* object, kAlloc allocator); 

/* 
* Private methods. 
*/

kFx(kStatus) xkDat5Serializer_InitStatic();
kFx(kStatus) xkDat5Serializer_ReleaseStatic();
kFx(kStatus) xkDat5Serializer_OnAssemblyLoad(kPointer receiver, kAssembly assembly, void* args);
kFx(kStatus) xkDat5Serializer_OnAssemblyUnload(kPointer receiver, kAssembly assembly, void* args);
kFx(kStatus) xkDat5Serializer_GetTypeLookups(kMap* guidToInfo, kMap* guidToCompressor);

kFx(kBool) xkDat5Serializer_VReset(kDat5Serializer serializer); 
kFx(kStatus) xkDat5Serializer_VWriteType(kDat5Serializer serializer, kType type, kTypeVersion* version); 
kFx(kStatus) xkDat5Serializer_VReadType(kDat5Serializer serializer, kType* type, kTypeVersion* version); 

//These methods shouldn't be necessary, but a bug/limitation in kImage serialization version 5-3 necessitates them. 
kFx(kStatus) xkDat5Serializer_WriteTypeWithoutVersion(kDat5Serializer serializer, kType type, kTypeVersion* version); 
kFx(kStatus) xkDat5Serializer_ReadTypeExplicitVersion(kDat5Serializer serializer, k32u typeVersionId, kType* type, kTypeVersion* version); 

kFx(kStatus) xkDat5Serializer_BeginCompression(kDat5Serializer serializer);
kFx(kStatus) xkDat5Serializer_EndCompression(kDat5Serializer serializer);

kFx(kStatus) xkDat5Serializer_BeginDecompression(kDat5Serializer serializer);
kFx(kStatus) xkDat5Serializer_EndDecompression(kDat5Serializer serializer);

#endif

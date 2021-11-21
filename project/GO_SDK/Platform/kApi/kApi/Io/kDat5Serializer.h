/** 
 * @file    kDat5Serializer.h
 * @brief   Declares the kDat5Serializer class. 
 *
 * @internal
 * Copyright (C) 2008-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_DAT5_SERIALIZER_H
#define K_API_DAT5_SERIALIZER_H

#include <kApi/kApiDef.h>
#include <kApi/Io/kDat5Serializer.x.h>

/**
 * @class   kDat5Serializer
 * @extends kSerializer
 * @ingroup kApi-Io
 * @brief   Serializes/deserializes objects using kDat5 format. 
 */
//typedef kSerializer kDat5Serializer;         --forward-declared in kApiDef.x.h 

/** 
 * Constructs a kDat5Serializer object.
 *
 * @public              @memberof kDat5Serializer
 * @param   serializer  Receives the constructed object. 
 * @param   stream      Stream for reading or writing.
 * @param   allocator   Memory allocator (or kNULL for default). 
 * @return              Operation status. 
 */
kFx(kStatus) kDat5Serializer_Construct(kDat5Serializer* serializer, kStream stream, kAlloc allocator); 

/** 
 * Writes a labelled object to the underlying stream. 
 * 
 * @public                  @memberof kDat5Serializer
 * @param   serializer      Serializer object. 
 * @param   object          Object to be serialized. 
 * @param   label           Object label.  
 * @return                  Operation status. 
 */
kFx(kStatus) kDat5Serializer_WriteData(kDat5Serializer serializer, kObject object, const kChar* label);

/** 
 * Reads a labelled object from the underlying stream. 
 *
 * @public                  @memberof kDat5Serializer
 * @param   serializer      Serializer object. 
 * @param   object          Receives deserialized object. 
 * @param   label           Receives object label. 
 * @param   capacity        Label capacity. 
 * @param   allocator       Memory allocator (or kNULL for default). 
 * @return                  Operation status. 
 */
kFx(kStatus) kDat5Serializer_ReadData(kDat5Serializer serializer, kObject* object, kChar* label, kSize capacity, kAlloc allocator);

/**
 * Enables the use of compression in serialization. 
 *
 * By default, kDat5Serializer data is uncompressed. This method can be used to specify a compression 
 * algorithm to be used when serializing objects. 
 * 
 * When compression is enabled via this method, metadata is encoded in the stream such that the 
 * required decompression approach will be automatically detected during object deserialization. 
 * Accordingly, this method should <em>not</em> be called when deserializing data. 
 * 
 * The use of compression requires that the underlying stream object (e.g., kFile) created by the 
 * <em>receiver</em> provides a read buffer. Configure the read stream to allocate a read buffer 
 * before passing the stream to the deserializer constructor. 
 * 
 * The Zen library does not itself provide compression algorithms. If compression support is required, 
 * an additional assembly that implements compression must first be loaded. Refer to the documentation 
 * for the selected compression stream type for information about additional requirements or restrictions 
 * that might apply. 
 * 
 * Support for this feature was introduced in Zen 6.2.1. Older versions cannot read or write compressed content.
 * 
 * @public                  @memberof kDat5Serializer
 * @param   serializer      Serializer object.
 * @param   algorithm       Compression algorithm type (null to disable).
 * @param   level           Compression level; accepts kCompressionPreset value or compressor-specific value.
 * @return                  Operation status. 
 */
kFx(kStatus) kDat5Serializer_EnableCompression(kDat5Serializer serializer, kCompressionType algorithm, k32s level); 

/** 
 * Saves an object to file using the specified compression type.
 * 
 * The file produced by this function can be loaded with kSerializer_LoadObject or kLoad5.
 * 
 * The Zen library does not itself provide compression algorithms. If compression support is required, 
 * an additional assembly that implements compression must first be loaded. Refer to the documentation 
 * for the selected compression stream type for information about additional requirements or restrictions 
 * that might apply. 
 * 
 * Support for this feature was introduced in Zen 6.2.1. Older versions cannot read or write compressed content.
 * 
 * @public                  @memberof kDat5Serializer
 * @param   object          Object to be serialized.
 * @param   filePath        Path of the file to save. 
 * @param   algorithm       Compression algorithm type.
 * @param   level           Compression level; accepts kCompressionPreset value or compressor-specific value.
 * @return                  Operation status. 
 */
kFx(kStatus) kDat5Serializer_SaveCompressed(kObject object, const kChar* filePath, kCompressionType algorithm, k32s level); 

#endif

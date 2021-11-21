/** 
 * @file    kDat6Serializer.h
 * @brief   Declares the kDat6Serializer class. 
 *
 * @internal
 * Copyright (C) 2012-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_DAT6_SERIALIZER_H
#define K_API_DAT6_SERIALIZER_H

#include <kApi/kApiDef.h>
#include <kApi/Io/kDat6Serializer.x.h>

/**
 * @class   kDat6Serializer
 * @extends kSerializer
 * @ingroup kApi-Io
 * @brief   Serializes/deserializes objects using kDat6 format. 
 */
//typedef kSerializer kDat6Serializer;        --forward-declared in kApiDef.x.h 

/** 
 * Constructs a kDat6Serializer object.
 *
 * @public              @memberof kDat6Serializer
 * @param   serializer  Receives the constructed object. 
 * @param   stream      Stream for reading or writing.
 * @param   allocator   Memory allocator (or kNULL for default). 
 * @return              Operation status. 
 */
kFx(kStatus) kDat6Serializer_Construct(kDat6Serializer* serializer, kStream stream, kAlloc allocator); 

/**
 * Enables or disables the use of a type dictionary when writing objects.
 *
 * By default, kDat6Serializer uses a type dictionary for better performance. However, 
 * the use of a type dictionary requires linear/sequential deserialization (or at least 
 * a linear pass through the stream before performing random accesses). To suppport
 * true random-access deserialization, the type dicationary feature must be disabled when 
 * serializing data. 
 *
 * @public              @memberof kDat6Serializer
 * @param   serializer  Serializer object.
 * @param   enable      kTRUE to enable.
 * @return              Operation status. 
 */
kFx(kStatus) kDat6Serializer_EnableDictionary(kDat6Serializer serializer, kBool enable); 

/**
 * Enables the use of compression in serialization. 
 *
 * By default, kDat6Serializer data is uncompressed. This method can be used to specify a compression 
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
 * @public                  @memberof kDat6Serializer
 * @param   serializer      Serializer object.
 * @param   algorithm       Compression algorithm type (null to disable).
 * @param   level           Compression level; accepts kCompressionPreset value or compressor-specific value.
 * @return                  Operation status. 
 */
kFx(kStatus) kDat6Serializer_EnableCompression(kDat6Serializer serializer, kCompressionType algorithm, k32s level); 

/** 
 * Saves an object to file using the specified compression type.
 * 
 * The file produced by this function can be loaded with kSerializer_LoadObject or kLoad6.
 * 
 * The Zen library does not itself provide compression algorithms. If compression support is required, 
 * an additional assembly that implements compression must first be loaded. Refer to the documentation 
 * for the selected compression stream type for information about additional requirements or restrictions 
 * that might apply. 
 * 
 * Support for this feature was introduced in Zen 6.2.1. Older versions cannot read or write compressed content.
 * 
 * @public                  @memberof kDat6Serializer
 * @param   object          Object to be serialized.
 * @param   filePath        Path of the file to save. 
 * @param   algorithm       Compression algorithm type.
 * @param   level           Compression level; accepts kCompressionPreset value or compressor-specific value.
 * @return                  Operation status. 
 */
kFx(kStatus) kDat6Serializer_SaveCompressed(kObject object, const kChar* filePath, kCompressionType algorithm, k32s level); 

#endif

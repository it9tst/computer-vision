/** 
 * @file    kCipherStream.h
 * @brief   Declares the kCipherStream type. 
 *
 * @internal
 * Copyright (C) 2016-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_CIPHER_STREAM_H
#define K_API_CIPHER_STREAM_H

#include <kApi/kApiDef.h>
#include <kApi/Crypto/kCipherStream.x.h>

/**
 * @class   kCipherStream
 * @extends kStream
 * @ingroup kApi-Crypto
 * @brief   Supports streaming encryption or decryption. 
 * 
 * This class binds a cipher object to an underlying stream object. The cipher provides 
 * encyption/decryption, while the underlying stream provides the data source/destination.
 */
//typedef kStream kCipherStream;            --forward-declared in kFsDef.x.h

/** 
 * Constructs a kCipherStream object.
 *
 * @public                  @memberof kCipherStream
 * @param   cipherStream    Destination for the constructed object handle.
 * @param   stream          Stream used for the underlying read/write functions.
 * @param   cipher          Cipher used for encryption and/or decryption.
 * @param   allocator       Memory allocator (or kNULL for default).
 * @return                  Operation status. 
 */
kFx(kStatus) kCipherStream_Construct(kCipherStream *cipherStream, kStream stream, kCipher cipher, kAlloc allocator); 

/** 
 * Flushes any pending data, adding padding if needed. 
 * 
 * If the kStream_Flush method is called on a cipher stream object, it will flush the 
 * underlying stream to which this cipher stream is bound. However, if the number of 
 * bytes that have been written to the cipher stream is not a multiple of cipher block 
 * size, remainder bytes will not be flushed automatically. 
 * 
 * In contrast, the kCipherStream_FlushFinal method ensures that all bytes that have 
 * been written to the cipher stream are flushed, adding padding if necessary. This is 
 * typically performed once, at the end of a stream.  However, it's possible to perform this 
 * operation multiple times. It is up to the caller to remove padding bytes from decrypted 
 * output. 
 * 
 * This function will be called automatically when the cipher stream object is destroyed.
 *
 * @public                  @memberof kCipherStream
 * @param   cipherStream    Cipher stream object.
 * @return                  Operation status. 
 */
kFx(kStatus) kCipherStream_FlushFinal(kCipherStream cipherStream);

#endif

/** 
 * @file    kBlowfishCipher.h
 * @brief   Declares the kBlowfishCipher class. 
 *
 * @internal
 * Copyright (C) 2016-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_BLOWFISH_CIPHER_H
#define K_API_BLOWFISH_CIPHER_H

#include <kApi/kApiDef.h>
#include <kApi/Crypto/kCipher.h>
#include <kApi/Crypto/kBlowfishCipher.x.h>

/**
 * @class   kBlowfishCipher
 * @extends kCipher
 * @ingroup kApi-Crypto
 * @brief   Blowfish cipher implementation.
 */
//typedef kCipher kBlowfishCipher;   --forward-declared in kFsDef.x.h

/** 
 * Constructs a kBlowfishCipher instance. 
 * 
 * @public                  @memberof kBlowfishCipher
 * @param   blowfish        Receives constructed blowfish cipher object. 
 * @param   key             Key.
 * @param   keyLength       Length of the key.
 * @param   padding         Padding mode to be used in encryption. 
 * @param   cipherMode      Cipher mode used for encryption and decryption. 
 * @param   allocator       Memory allocator (or kNULL for default). 
 * @return                  Operation status. 
 */
kFx(kStatus) kBlowfishCipher_Construct(kBlowfishCipher* blowfish, const kByte* key, kSize keyLength, kCipherPadding padding, kCipherMode cipherMode, kAlloc allocator);

#endif

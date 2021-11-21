/** 
 * @file    kCipher.h
 * @brief   Declares the kCipher class. 
 *
 * @internal
 * Copyright (C) 2016-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_CIPHER_H
#define K_API_CIPHER_H

#include <kApi/kApiDef.h>

/**
 * @class   kCipher
 * @extends kObject
 * @ingroup kApi-Crypto
 * @brief   Abstract base class for symmetric-key encryption classes.
 */
//typedef kObject kCipher;   --forward-declared in kFsDef.x.h

/**
* @struct  kCipherPadding
* @extends kValue
* @ingroup kApi-Crypto
* @brief   Represents padding mode for encryption.
*/
typedef k32s kCipherPadding;

/** @relates kCipherPadding @{ */
#define kCIPHER_PADDING_NULL                (0x0000)        ///< Unknown. 
#define kCIPHER_PADDING_ANSIX923            (0x0001)        ///< ANSIX923, padding string consists of a sequence of bytes filled with zeros before the length.
#define kCIPHER_PADDING_ISO10126            (0x0002)        ///< ISO10126, padding string consists of random data before the length. 
#define kCIPHER_PADDING_NONE                (0x0004)        ///< No padding, requires that the encrypted string is a multiple of 8 bytes long.
#define kCIPHER_PADDING_PKCS7               (0x0008)        ///< PKCS7, padding string consists of a sequence of bytes, each of which is equal to the total number of padding bytes added. 
#define kCIPHER_PADDING_ZERO                (0x0010)        ///< Zero, padding string consists of bytes set to zero.  
/** @} */

/**
* @struct  kCipherMode
* @extends kValue
* @ingroup kApi-Crypto
* @brief   Represents cipher mode for encryption and decryption functions.
*/
typedef k32s kCipherMode;

/** @relates kCipherMode @{ */
#define kCIPHER_CIPHER_NULL                (0x0000)        ///< Unknown. 
#define kCIPHER_CIPHER_ECB                 (0x0001)        ///< Electronic Cook Book, each block is encrypted independently.
/** @} */


//include class definition
#include <kApi/Crypto/kCipher.x.h>

/** 
 * Encrypts data.
 * 
 * Block ciphers operate on fixed block sizes, so padding may be added. The type of padding 
 * depends on settings provided to the cipher object. It is the caller's responsible to remove 
 * any padding from decrypted data. 
 * 
 * @public                  @memberof kCipher
 * @param   cipher          Cipher object. 
 * @param   data            Data to encrypt. 
 * @param   dataLength      Length of the data to encrypt, in bytes. 
 * @param   result          Receives encrypted data. 
 * @return                  Operation status. 
 */
kInlineFx(kStatus) kCipher_Encrypt(kCipher cipher, const void* data, kSize dataLength, kArray1 result)
{
    return xkCipher_VTable(cipher)->VEncrypt(cipher, data, dataLength, result);
}

/** 
 * Decrypts data. 
 * 
 * When decrypting data, it is necessary to use the same cipher mode that was used to encrypt the data. 
 * 
 * It is the caller's responsibility to remove any padding that was added during encryption. 
 * 
 * @public                  @memberof kCipher
 * @param   cipher          Cipher object. 
 * @param   data            Data to decrypt. 
 * @param   dataLength      Length of the data to decrypt, in bytes (multiple of the block size).
 * @param   result          Receives decrypted output. 
 * @return                  Operation status. 
 */
kInlineFx(kStatus) kCipher_Decrypt(kCipher cipher, const void* data, kSize dataLength, kArray1 result)
{
    return xkCipher_VTable(cipher)->VDecrypt(cipher, data, dataLength, result);
}

/** 
 * Gets the pattern mode.
 * 
 * @public                  @memberof kCipher
 * @param   cipher          Cipher object. 
 * @return                  Pattern mode. 
 */
kInlineFx(kCipherPadding) kCipher_Padding(kCipher cipher)
{
    kObj(kCipher, cipher);

    return obj->padding;
}

/** 
 * Gets the cipher mode.
 * 
 * @public                  @memberof kCipher
 * @param   cipher          Cipher object. 
 * @return                  Cipher mode. 
 */
kInlineFx(kCipherMode) kCipher_Mode(kCipher cipher)
{
    kObj(kCipher, cipher);

    return obj->mode;
}

/** 
 * Gets the block size.
 * 
 * @public                  @memberof kCipher
 * @param   cipher          Cipher object. 
 * @return                  Block size.
 */
kInlineFx(kSize) kCipher_Blocksize(kCipher cipher)
{
    return xkCipher_VTable(cipher)->VBlocksize(cipher);
}

#endif

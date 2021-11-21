/** 
 * @file    kApiDef.h
 * @brief   Core Zen type declarations. 
 *
 * @internal
 * Copyright (C) 2005-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_API_DEF_H
#define K_API_API_DEF_H

#include <kApi/kApiCfg.h>

#define kCall           xkCall              ///< kApi standard function calling convention.
#define kDlCall         xkDlCall            ///< kApi dynamic load function calling convention.

#if defined(K_EMIT)
#   define kFx(TYPE)    kExportFx(TYPE)     ///< kApi function declaration helper. 
#   define kDx(TYPE)    kExportDx(TYPE)     ///< kApi data declaration helper. 
#else
#   define kFx(TYPE)    kImportFx(TYPE)     
#   define kDx(TYPE)    kImportDx(TYPE)     
#endif

#define kInlineFx(TYPE) xkInlineFx(TYPE)    ///< Inline method declaration helper.

typedef void (kCall* kFunction)();          ///< Generic pointer to function.

/**
 * @struct  k8u
 * @extends kValue
 * @ingroup kApi  
 * @brief   Represents an 8-bit unsigned integer.
 *
 * k8u supports the kdat5 and kdat6 serialization protocols.
 */
typedef xk8u k8u;          

/** @relates k8u @{ */
#define k8U_MIN    (0)             ///< k8u minimum value. 
#define k8U_MAX    (255U)          ///< k8u maximum value.
#define k8U_NULL   (k8U_MAX)       ///< k8u invalid value.
/** @} */

/**
 * @struct  k16u
 * @extends kValue
 * @ingroup kApi  
 * @brief   Represents a 16-bit unsigned integer.
 *
 * k16u supports the kdat5 and kdat6 serialization protocols.
 */
typedef xk16u k16u;           

/** @relates k16u @{ */
#define k16U_MIN   (0)             ///< k16u minimum value. 
#define k16U_MAX   (65535U)        ///< k16u maximum value. 
#define k16U_NULL  (k16U_MAX)      ///< k16u invalid value. 
/** @} */

/**
 * @struct  k32u
 * @extends kValue
 * @ingroup kApi  
 * @brief   Represents a 32-bit unsigned integer.
 *
 * k32u supports the kdat5 and kdat6 serialization protocols.
 */
typedef xk32u k32u;   

/** @relates k32u @{ */
#define k32U_MIN   (0)             ///< k32u minimum value. 
#define k32U_MAX   (4294967295U)   ///< k32u maximum value. 
#define k32U_NULL  (k32U_MAX)      ///< k32u invalid value. 
/** @} */

/**
 * @struct  k64u
 * @extends kValue
 * @ingroup kApi  
 * @brief   Represents a 64-bit unsigned integer.
 *
 * k64u supports the kdat5 and kdat6 serialization protocols.
 */
typedef xk64u k64u;         

/** @relates k64u @{ */
#define k64U(CONST) xk64U(CONST)                    ///< Declares a 64-bit unsigned integer literal.
#define k64U_MIN    k64U(0)                         ///< k64u minimum value. 
#define k64U_MAX    k64U(18446744073709551615)      ///< k64u maximum value.
#define k64U_NULL  (k64U_MAX)                       ///< k64u invalid value.
/** @} */

#define kINFINITE   k64U_MAX                        ///< Infinity (used for k64u timeouts).

/**
 * @struct  k8s
 * @extends kValue
 * @ingroup kApi  
 * @brief   Represents an 8-bit signed integer.
 *
 * k8s supports the kdat5 and kdat6 serialization protocols.
 */
typedef xk8s k8s;       

/** @relates k8s @{ */
#define k8S_MAX    (127)               ///< k8s maximum value.
#define k8S_MIN    (-k8S_MAX -1)       ///< k8s minimum value.
#define k8S_NULL   (k8S_MIN)           ///< k8s invalid value.
/** @} */

/**
 * @struct  k16s
 * @extends kValue
 * @ingroup kApi  
 * @brief   Represents a 16-bit signed integer.
 *
 * k16s supports the kdat5 and kdat6 serialization protocols.
 */
typedef xk16s k16s;      

/** @relates k16s @{ */
#define k16S_MAX   (32767)             ///< k16s maximum value. 
#define k16S_MIN   (-k16S_MAX -1)      ///< k16s minimum value.
#define k16S_NULL  (k16S_MIN)          ///< k16s invalid value. 
/** @} */

/**
 * @struct  k32s
 * @extends kValue
 * @ingroup kApi  
 * @brief   Represents a 32-bit signed integer.
 *
 * k32s supports the kdat5 and kdat6 serialization protocols.
 */
typedef xk32s k32s;      

/** @relates k32s @{ */
#define k32S_MAX   (2147483647)        ///< k32s maximum value. 
#define k32S_MIN   (-k32S_MAX -1)      ///< k32s minimum value.
#define k32S_NULL  (k32S_MIN)          ///< k32s invalid value. 
/** @} */

/**
 * @struct  k64s
 * @extends kValue
 * @ingroup kApi  
 * @brief   Represents a 64-bit signed integer.
 *
 * k64s supports the kdat5 and kdat6 serialization protocols.
 */
typedef xk64s k64s;      

/** @relates k64s @{ */
#define k64S(CONST) xk64S(CONST)                ///< Declares a 64-bit signed integer literal.
#define k64S_MAX    k64S(9223372036854775807)   ///< k64s maximum value. 
#define k64S_MIN    (-k64S_MAX -1)              ///< k64s minimum value.
#define k64S_NULL   (k64S_MIN)                  ///< k64s invalid value. 
/** @} */

/**
 * @struct  k32f
 * @extends kValue
 * @ingroup kApi  
 * @brief   Represents a 32-bit floating-point number.
 *
 * k32f supports the kdat5 and kdat6 serialization protocols.
 */
typedef xk32f k32f;         

/** @relates k32f @{ */
#define k32F_MIN   (1.175494351e-38F)      ///< k32f smallest positive value.
#define k32F_MAX   (3.402823466e+38F)      ///< k32f largest positive value.
#define k32F_NULL  (-k32F_MAX)             ///< k32f invalid value. 
#define k32F_DIGITS (9)                    ///< Default number of digits used for encoding k32f as a decimal text value.
/** @} */

/**
 * @struct  k64f
 * @extends kValue
 * @ingroup kApi  
 * @brief   Represents a 64-bit floating-point number.
 *
 * k64f supports the kdat5 and kdat6 serialization protocols.
 */
typedef xk64f k64f;        

/** @relates k64f @{ */
#define k64F_MIN   (2.2250738585072014e-308)   ///< k64f smallest positive value.
#define k64F_MAX   (1.7976931348623157e+308)   ///< k64f largest positive value.
#define k64F_NULL  (-k64F_MAX)                 ///< k64f invalid value. 
#define k64F_DIGITS (17)                       ///< Default number of digits used for encoding k64f as a decimal text value.
/** @} */


/**
 * Reports whether the given number is equivalent to the double-precision representation of NAN or INF. 
 * 
 * @public          @memberof k64f
 * @param   value   Value.
 * @return          1 if the value is NAN or INF; otherwise 0. 
 */
kFx(k32s) k64f_IsNanOrInf(k64f value);

/**
 * @struct  kByte
 * @extends kValue
 * @ingroup kApi  
 * @brief   Represents a byte on the current platform. 
 *
 * kByte supports the kdat5 and kdat6 serialization protocols.
 */
typedef xkByte kByte;                           

/**
 * @struct  kSize
 * @extends kValue
 * @ingroup kApi  
 * @brief   Represents an unsigned integer that can store a pointer address.
 *
 * kSize supports the kdat5 and kdat6 serialization protocols.
 */
typedef xkSize kSize;   

/** @relates kSize @{ */
#define kSIZE_MAX           xkSIZE_MAX      ///< Maximum value contained by kSize. 
#define kSIZE_NULL          kSIZE_MAX       ///< Invalid size value. 
/** @} */

/**
* Rounds up to nearest value that is a multiple of the specified power of two.
* 
* This method is typically used to calculate memory addresses that are a multiple 
* of a required power of two.   
* 
* @param   value    Input value.
* @param   to       Power of two (e.g., 3 => 2^3 = 8).  
* @return           Input value, rounded up. 
*/
kInlineFx(kSize) kSize_Align(kSize value, kSize to)
{
    return ((value >> to) + !!(value & ((1 << to) - 1))) << to;
}

/**
 * @struct  kSSize
 * @extends kValue
 * @ingroup kApi  
 * @brief   Represents a signed integer that can store a pointer address.
 */
typedef xkSSize kSSize;   

/** @relates kSSize @{ */
#define kSSIZE_MIN       xkSSIZE_MIN        ///< Minimum value contained by kSSize. 
#define kSSIZE_MAX       xkSSIZE_MAX        ///< Maximum value contained by kSSize. 
#define kSSIZE_NULL      kSSIZE_MIN         ///< Invalid kSSize value. 
/** @} */

/**
 * @struct  kPointer
 * @extends kValue
 * @ingroup kApi  
 * @brief   Represents a void pointer.
 * @see     kNULL, kIsNull
 */
typedef void* kPointer;   

#define kNULL   (0)     ///< Null pointer. 


/**
* Calculates a pointer address from a base address and a byte offset. 
*
* @relates          kPointer
* @param   pointer  Memory pointer.
* @param   offset   Byte offset.
* @return           Base + offset. 
*/
kInlineFx(void*) kPointer_ByteOffset(const void* pointer, kSSize offset)
{
    return (kByte*)pointer + offset;
}

/**
* Gets a pointer to the Nth element of an array.
* 
* @relates              kPointer
* @param   pointer      Memory pointer.
* @param   itemIndex    Element index.
* @param   itemSize     Element size.
* @return               Pointer to Nth element of array. 
*/
kInlineFx(void*) kPointer_ItemOffset(const void* pointer, kSSize itemIndex, kSize itemSize)
{
    return kPointer_ByteOffset(pointer, itemIndex*(kSSize)itemSize);
}

/**
* Calculates the signed difference between two pointers.
* 
* @relates      kPointer
* @param   a    Pointer A.
* @param   b    Pointer B.
* @return       A - B.  
*/
kInlineFx(kSSize) kPointer_Diff(void* a, void* b)
{
    return (kSSize)a - (kSSize)b;
}

/**
* Deferences a pointer, assuming the specified type, and returns the resulting value.
* 
* @relates          kPointer
* @param   POINTER  Memory pointer.
* @param   TYPE     Value type symbol (e.g, k32s).
* @return           Item value. 
*/
#define kPointer_ReadAs(POINTER, TYPE)                            \
    (*(TYPE*)(POINTER))

/**
* Deferences a pointer, assuming the specified type, and assigns a value to the resulting reference. 
* 
* @relates          kPointer
* @param   POINTER  Memory pointer.
* @param   VALUE    Value to set.  
* @param   TYPE     Value type symbol (e.g, k32s).
*/
#define kPointer_WriteAs(POINTER, VALUE, TYPE)                  \
    (*(TYPE*)(POINTER) = (TYPE)(VALUE))

/** 
 * Tests for equality with null pointer.
 *
 * @relates             kPointer
 * @param   POINTER     Pointer to be compared with null. 
 * @return              kTRUE if the argument is equal to null; kFALSE otherwise. 
 */
#define kIsNull(POINTER)    \
     ((POINTER) == kNULL)         

/**
 * @struct  kBool
 * @extends kValue
 * @ingroup kApi  
 * @brief   Represents a boolean value. 
 *
 * kBool supports the kdat5 and kdat6 serialization protocols.
 */
typedef k32s kBool;

/** @relates kBool @{ */
#define kFALSE     (0)     ///< Boolean false. 
#define kTRUE      (1)     ///< Boolean true. 
/** @} */

/**
 * @struct  kChar
 * @extends kValue
 * @ingroup kApi  
 * @brief   Represents a single unit (byte) in a UTF-8 character. 
 *
 * kChar supports the kdat5 and kdat6 serialization protocols.
 */
typedef xkChar kChar;                           

/** 
 * Convert ASCII character to lower case.
 *
 * @public      @memberof kChar
 * @param   ch  Input character. 
 * @return      Lower case character.
 */
kInlineFx(kChar) kChar_ToLower(kChar ch)
{
    return (ch >= 'A' && ch <= 'Z') ? (ch - 'A' + 'a') : ch;
}

/** 
 * Convert ASCII character to upper case.
 *
 * @public      @memberof kChar
 * @param   ch  Input character. 
 * @return      Upper case character.
 */
kInlineFx(kChar) kChar_ToUpper(kChar ch)
{
    return (ch >= 'a' && ch <= 'z') ? (ch - 'a' + 'A') : ch;
}

/** 
 * Checks whether ASCII character is a some kind of white space character type.
 *
 * @public       @memberof kChar
 * @param   ch  Input character. 
 * @return      Is character a space?
 */
kInlineFx(kBool) kChar_IsSpace(kChar ch)
{
    return ch == ' ';
}

/** 
 * Checks whether ASCII character is a letter.
 *
 * @public      @memberof kChar
 * @param   ch  Input character. 
 * @return      Is character a letter?
 */
kInlineFx(kBool) kChar_IsLetter(kChar ch)
{
    return ((ch >= 'a') && (ch <= 'z') ) || ((ch >= 'A') && (ch <= 'Z'));
}

/** 
 * Checks whether ASCII character is a digit.
 *
 * @public      @memberof kChar
 * @param   ch  Input character. 
 * @return      Is character a digit?
 */
kInlineFx(kBool) kChar_IsDigit(kChar ch)
{
    return ch >= '0' && ch <= '9';
}

/**
 * @struct  kText16
 * @extends kValue
 * @ingroup kApi  
 * @brief   Represents a 16-unit, null-terminated, kChar sequence.
 *
 * kText16 supports the kdat5 and kdat6 serialization protocols.
 */
typedef kChar kText16[16];    

/**
 * @struct  kText32
 * @extends kValue
 * @ingroup kApi  
 * @brief   Represents a 32-unit, null-terminated, kChar sequence.
 *
 * kText32 supports the kdat5 and kdat6 serialization protocols.
 */
typedef kChar kText32[32];    

/**
 * @struct  kText64
 * @extends kValue
 * @ingroup kApi  
 * @brief   Represents a 64-unit, null-terminated, kChar sequence.
 *
 * kText64 supports the kdat5 and kdat6 serialization protocols.
 */
typedef kChar kText64[64];    

/**
 * @struct  kText128
 * @extends kValue
 * @ingroup kApi  
 * @brief   Represents a 128-unit, null-terminated, kChar sequence.
 *
 * kText128 supports the kdat5 and kdat6 serialization protocols.
 */
typedef kChar kText128[128];  

/**
 * @struct  kText256
 * @extends kValue
 * @ingroup kApi  
 * @brief   Represents a 256-unit, null-terminated, kChar sequence.
 *
 * kText256 supports the kdat5 and kdat6 serialization protocols.
 */
typedef kChar kText256[256];  

/**
 * @struct  kStatus
 * @extends kValue
 * @ingroup kApi  
 * @brief   Represents an error code. 
 *
 * kStatus supports the kdat6 serialization protocol. 
 */
typedef k32s kStatus; 

/** @relates kStatus @{ */
#define kERROR_STATE               (-1000)     ///< Operation cannot be completed in the current state.
#define kERROR_NOT_FOUND           (-999)      ///< Item was not found.
#define kERROR_COMMAND             (-998)      ///< Command was not recognized.
#define kERROR_PARAMETER           (-997)      ///< Parameter was not valid.
#define kERROR_UNIMPLEMENTED       (-996)      ///< Feature is not implemented.
#define kERROR_MEMORY              (-994)      ///< Out of memory.
#define kERROR_TIMEOUT             (-993)      ///< Action timed out.
#define kERROR_INCOMPLETE          (-992)      ///< Data incomplete (e.g., buffer insufficient for results).
#define kERROR_STREAM              (-991)      ///< Error in communication stream (e.g., network, file).
#define kERROR_CLOSED              (-990)      ///< Resource is no longer available. 
#define kERROR_VERSION             (-989)      ///< Incompatible version.
#define kERROR_ABORT               (-988)      ///< Operation aborted.
#define kERROR_ALREADY_EXISTS      (-987)      ///< Request conflicts with existing item.
#define kERROR_NETWORK             (-986)      ///< Network setup/resource error.
#define kERROR_HEAP                (-985)      ///< Heap error (leak/double-free).
#define kERROR_FORMAT              (-984)      ///< Data parsing/formatting error. 
#define kERROR_READ_ONLY           (-983)      ///< Object is read-only (cannot be written).
#define kERROR_WRITE_ONLY          (-982)      ///< Object is write-only (cannot be read). 
#define kERROR_BUSY                (-981)      ///< Agent is busy (cannot service request).
#define kERROR_CONFLICT            (-980)      ///< Current state has one or more configuration or resource conflicts.
#define kERROR_OS                  (-979)      ///< Generic error reported by underlying OS.
#define kERROR_DEVICE              (-978)      ///< Hardware device error. 
#define kERROR_FULL                (-977)      ///< Resource is already fully utilized. 
#define kERROR_IN_PROGRESS         (-976)      ///< Operation is in progress, but not yet complete.
#define kERROR                     (0)         ///< General error. 
#define kOK                        (1)         ///< Operation successful. 
/** @} */

/** 
 * Returns a text string representing the name of a status code (e.g. "kERROR_STATE"). 
 * 
 * This function returns a pointer to statically-allocated memory; do not attempt to free the memory.  
 *
 * @public              @memberof kStatus
 * @param   status      Status code. 
 * @return              Null-terminated character sequence.
 */
kFx(const kChar*) kStatus_Name(kStatus status); 

/** 
 * Returns kTRUE if the given status value is not kOK.
 * 
 * @param   status      A kStatus value.   
 * @return              kTRUE if the argument is not equal to kOK; kFALSE otherwise. 
 * @see                 @ref kApi-Error-Handling
 */
kInlineFx(kBool) kIsError(kStatus status)
{
    return status != kOK; 
}

/** 
 * Returns kTRUE if the given expression value is kOK.
 * 
 * @param   status      A kStatus value.   
 * @return              kTRUE if the argument is equal to kOK; kFALSE otherwise. 
 * @see                 @ref kApi-Error-Handling
 */
kInlineFx(kBool) kSuccess(kStatus status) 
{
    return status == kOK; 
}

/** 
 * Executes a <em>return</em> statement if the given expression is not kOK. 
 * 
 * If the expression result is not kOK, the current function will return the expression result.
 * 
 * @param   EXPRESSION  Expression that evaluates to a kStatus value.   
 * @see                 @ref kApi-Error-Handling
 */
#define kCheck(EXPRESSION)      \
    xkCheck(EXPRESSION)

/** 
 * Executes a <em>return</em> statement if the given expression is kFALSE. 
 * 
 * If the expression result is kFALSE, the current function will return STATUS.
 * 
 * @param   EXPRESSION  Expression that evaluates to a kBool value.   
 * @param   STATUS      Status code to be returned if EXPRESSION is kFALSE. 
 * @see                 @ref kApi-Error-Handling
 */
#define kCheckTrue(EXPRESSION, STATUS)      \
    xkCheckTrue(EXPRESSION, STATUS)

/** 
 * Executes a <em>return</em> statement if the given expression is not kTRUE. 
 * 
 * If the expression result is kFALSE, the current function will return kERROR_PARAMETER.
 * 
 * @param   EXPRESSION  Expression that evaluates to a kBool value.   
 * @see                 @ref kApi-Error-Handling
 */
#define kCheckArgs(EXPRESSION)      \
    xkCheckArgs(EXPRESSION)

/** 
 * Executes a <em>return</em> statement if the given expression is not kTRUE. 
 * 
 * If the expression result is kFALSE, the current function will return kERROR_STATE.
 * 
 * @param   EXPRESSION  Expression that evaluates to a kBool value.   
 * @see                 @ref kApi-Error-Handling
 */
#define kCheckState(EXPRESSION)     \
    xkCheckState(EXPRESSION)

/** 
 * Opens a kTry error-checking block.
 * 
 * @see     @ref kApi-Error-Handling
 */
#define kTry        \
    xkTry

/** 
 * Used within a kTry block to jump to the first error handling block (e.g. kCatch).
 * 
 * @param   EXPRESSION  Expression that evaluates to a kStatus value.   
 * @see                 @ref kApi-Error-Handling
 */
#define kThrow(EXPRESSION)      \
    xkThrow(EXPRESSION)

/** 
 * Used within a kTry block to conditionally jump to the first error handling block (e.g. kCatch). 
 * 
 * If the EXPRESSION argument does not evaluate to kOK, the result of the expression 
 * is passed to an error-handling block. Otherwise, execution continues at the next statement. 
 *
 * @param   EXPRESSION  Expression that evaluates to a kStatus value.   
 * @see                 @ref kApi-Error-Handling
 */
#define kTest(EXPRESSION)       \
    xkTest(EXPRESSION)

/** 
 * Within a kTry block, throws STATUS if the expression result is kFALSE. 
 * 
 * If the EXPRESSION argument evaluates to kFALSE, STATUS is passed to an 
 * error-handling block. Otherwise, execution continues at the next statement. 
 *
 * @param   EXPRESSION  Expression that evaluates to a kBool value.   
 * @param   STATUS      Status code to be thrown if EXPRESSION is kFALSE.  
 * @see                 @ref kApi-Error-Handling
 */
#define kTestTrue(EXPRESSION, STATUS)       \
    xkTestTrue(EXPRESSION, STATUS)

/** 
 * Within a kTry block, throws kERROR_PARAMETER if the expression result is not kTRUE.
 * 
 * If the EXPRESSION argument does not evaluate to kTRUE, kERROR_PARAMETER is passed to an 
 * error-handling block. Otherwise, execution continues at the next statement. 
 *
 * @param   EXPRESSION  Expression that evaluates to a kBool value.   
 * @see                 @ref kApi-Error-Handling
 */
#define kTestArgs(EXPRESSION)       \
    xkTestArgs(EXPRESSION)

/** 
 * Within a kTry block, throws kERROR_STATE if the expression result is kFALSE.
 * 
 * If the EXPRESSION argument does not evaluate to kTRUE, kERROR_STATE is passed to an 
 * error-handling block. Otherwise, execution continues at the next statement. 
 * 
 * @param   EXPRESSION  Expression that evaluates to a kBool value.   
 * @see                 @ref kApi-Error-Handling
 */
#define kTestState(EXPRESSION)      \
    xkTestState(EXPRESSION)

/** 
 * Closes a kTry block and opens a kCatch error-handling block.
 * 
 * @param   STATUS_POINTER  Receives the exception code.
 * @see                     @ref kApi-Error-Handling
 */
#define kCatch(STATUS_POINTER)      \
    xkCatch(STATUS_POINTER)

/** 
 * Closes a kCatch block. 
 * 
 * If the STATUS argument is not kOK, the current function returns STATUS.
 * 
 * @param   STATUS  Result of the kCatch block. 
 * @see             @ref kApi-Error-Handling
 */
#define kEndCatch(STATUS)       \
    xkEndCatch(STATUS)

/** 
 * Closes a kTry block and opens a kFinally block.
 * 
 * @see     @ref kApi-Error-Handling
 */
#define kFinally        \
    xkFinally 

/** 
 * Closes a kFinally block. 
 * 
 * If the kTry block produced an exception, the current function returns the exception code. 
 * Otherwise, execution continues after the kFinally block. 
 * 
 * @see     @ref kApi-Error-Handling
 */
#define kEndFinally()      \
    xkEndFinally()

/** 
 * Closes a kTry block and opens a kCatchEx error-handling block.
 * 
 * @param   STATUS_POINTER  Receives the exception code.
 * @see                     @ref kApi-Error-Handling
 */
#define kCatchEx(STATUS_POINTER)    \
    xkCatchEx(STATUS_POINTER)

/** 
 * Closes a kCatchEx block. 
 * 
 * The exception state is set to the value of the STATUS argument. If the exception state is 
 * not kOK, the kFinallyEx block will return the exception code to the caller when kFinallyEndEx 
 * is reached.
 * 
 * @param   STATUS  Result of the kCatchEx block. 
 * @see             @ref kApi-Error-Handling
 */
#define kEndCatchEx(STATUS)     \
    xkEndCatchEx(STATUS)

/** 
 * Opens a kFinallyEx block.
 * 
 * @see     @ref kApi-Error-Handling
 */
#define kFinallyEx      \
    xkFinallyEx 

/** 
 * Closes a kFinallyEx block. 
 * 
 * If the exception state passed by kEndCatchEx is not kOK, the current function returns the 
 * exception code. Otherwise, execution continues after the kFinallyEx block. 
 * 
 * @see     @ref kApi-Error-Handling
 */
#define kEndFinallyEx()             \
    xkEndFinallyEx()

/** 
 * Aborts execution if EXPRESSION is kFALSE. 
 * 
 * kAssert statements are omitted if neither K_DEBUG nor K_ASSERT is defined.
 * 
 * @param   EXPRESSION  Expression that evaluates to a kBool value.   
 */
#define kAssert(EXPRESSION)         \
    xkAssert(EXPRESSION)

/** 
 * Aborts execution if the type of the OBJECT argument is not equivalent to kTypeOf(SYMBOL).  
 * 
 * Type is equivalence is determined using kObject_Is. 
 * 
 * kAssertType statements are omitted if neither K_DEBUG nor K_ASSERT is defined.
 * 
 * @param   OBJECT      Expression that evaluates to a kType value.   
 * @param   SYMBOL      Type symbol, such as <em>kArrayList</em>. 
 */
#define kAssertType(OBJECT, SYMBOL)     \
    xkAssertType(OBJECT, SYMBOL)

/** 
 * Generates a trace event using the given tag (string literals only).
 * 
 * kTrace statements are omitted if K_NO_TRACE is defined. 
 * 
 * @param   TAG         String literal passed to the trace handler. 
 */
#define kTrace(TAG)                 \
    xkTrace(TAG)

/**
 * @struct  kVersion
 * @extends kValue
 * @ingroup kApi  
 * @brief   Represents a version number.
 *
 * kVersion supports the kdat6 serialization protocol.
 */
typedef k32u kVersion;

#define kVERSION_NULL       (k32U_NULL)     // kVersion invalid value.

/** 
 * Creates a version value from its constituent parts. 
 *
 * @public              @memberof kVersion
 * @param   major       Major version part. 
 * @param   minor       Minor version part.  
 * @param   release     Release version part. 
 * @param   build       Build version part. 
 * @return              Version value.  
 */
kInlineFx(kVersion) kVersion_Create(k32u major, k32u minor, k32u release, k32u build)
{
    return (major   & 0xFF) << 24 | 
           (minor   & 0xFF) << 16 |
           (release & 0xFF) << 8  | 
           (build   & 0xFF);
}

/** 
 * Builds a version string from the given numeric literal arguments.  
 *
 * @relates             kVersion
 * @param   MAJOR       Major version part. 
 * @param   MINOR       Minor version part.  
 * @param   RELEASE     Release version part. 
 * @param   BUILD       Build version part. 
 * @return              Version string literal.   
 */
#define kVersion_Stringify_(MAJOR, MINOR, RELEASE, BUILD)               \
    xkStringize(MAJOR) "." xkStringize(MINOR) "." xkStringize(RELEASE) "." xkStringize(BUILD)

/** 
 * Parses a version from a formatted string. 
 *
 * @public              @memberof kVersion
 * @param   version     Receives the parsed version. 
 * @param   buffer      Formatted string (e.g. "1.2.3.4"). 
 * @return              Operation status. 
 */
kFx(kStatus) kVersion_Parse(kVersion* version, const kChar* buffer);

/** 
 * Formats a version to a string buffer. 
 *
 * @public              @memberof kVersion
 * @param   version     Version. 
 * @param   buffer      Receives formatted string (e.g. "1.2.3.4"). 
 * @param   capacity    Buffer capacity. 
 * @return              Operation status. 
 */
kFx(kStatus) kVersion_Format(kVersion version, kChar* buffer, kSize capacity);

/** 
 * Returns an integral value indicating the relationship between the versions.
 * A zero value represents both versions are equal. A positive value indicates
 * that version2 is greater than version1; a negative value indicates the 
 * opposite. 
 *
 * @public              @memberof kVersion
 * @param   version1    Version1. 
 * @param   version2    Version2. 
 * @return              Comparison result value. 
 */
kInlineFx(k32s) kVersion_Compare(kVersion version1, kVersion version2)
{
    return (k32s)version1 - (k32s)version2; 
}

/** 
 * Returns the major part of a version number.
 *
 * @public              @memberof kVersion
 * @param   version     Version number. 
 * @return              Major version.  
 */
kInlineFx(k8u) kVersion_Major(kVersion version)
{
    return (version >> 24) & 0xFF; 
}

/** 
 * Returns the minor part of a version number.
 *
 * @public              @memberof kVersion
 * @param   version     Version number. 
 * @return              Minor version.  
 */
kInlineFx(k8u) kVersion_Minor(kVersion version)
{
    return (version >> 16) & 0xFF; 
}

/** 
 * Returns the release part of a version number.
 *
 * @public              @memberof kVersion
 * @param   version     Version number. 
 * @return              Release version.  
 */
kInlineFx(k8u) kVersion_Release(kVersion version)
{
    return (version >> 8) & 0xFF; 
}

/** 
 * Returns the build part of a version number.
 *
 * @public              @memberof kVersion
 * @param   version     Version number. 
 * @return              Build version.  
 */
kInlineFx(k8u) kVersion_Build(kVersion version)
{
    return (version) & 0xFF; 
}

/**
 * @struct  kEndianness
 * @extends kValue
 * @ingroup kApi
 * @brief   Represents the byte-ordering of primitive data types. 
 */
typedef k32s kEndianness; 

/** @relates kEndianness @{ */
#define kENDIANNESS_LITTLE          (1)             ///< Least significant byte first. 
#define kENDIANNESS_BIG             (2)             ///< Most significant byte first. 
/** @} */

/** 
 * Reports the endianness of the current platform.
 * 
 * @return      Endianness of current platform.
 */
kInlineFx(kEndianness) kEndianness_Host()
{
    return K_ENDIANNESS;
}

/** 
 * Reports whether byte ordering must be reversed to be consistent with the current platform.
 * 
 * @param   endianness  Desired endianness.   
 * @return              kTRUE if byte ordering should be reversed. 
 */
kInlineFx(kBool) kEndianness_ShouldReverse(kEndianness endianness)    
{
    return endianness != kEndianness_Host();
}

/** 
 * Converts k8u value to string. 
 * 
 * If the buffer is insufficient, the copy will transfer as many characters
 * as possible, null-terminate the resulting string, and return kERROR_INCOMPLETE. 
 * 
 * If the capacity is zero, kERROR_PARAMETER will be returned. 
 *
 * @public              @memberof k8u
 * @param   value       Input value.
 * @param   buffer      Destination for the string copy.
 * @param   capacity    Capacity of destination buffer, in characters.
 * @return              Operation status. 
 */
kFx(kStatus) k8u_Format(k8u value, kChar* buffer, kSize capacity);

/** 
 * Converts string to k8u value. 
 *
 * @public              @memberof k8u
 * @param   value       Receives result.
 * @param   str         Input string.
 * @return              Operation status. 
 */
kFx(kStatus) k8u_Parse(k8u* value, const kChar* str);

/** 
 * Converts k8s value to string. 
 * 
 * If the buffer is insufficient, the copy will transfer as many characters
 * as possible, null-terminate the resulting string, and return kERROR_INCOMPLETE. 
 * 
 * If the capacity is zero, kERROR_PARAMETER will be returned. 
 *
 * @public              @memberof k8s
 * @param   value       Input value.
 * @param   buffer      Destination for the string copy.
 * @param   capacity    Capacity of destination buffer, in characters.
 * @return              Operation status. 
 */
kFx(kStatus) k8s_Format(k8s value, kChar* buffer, kSize capacity);

/** 
 * Converts string to k8s value. 
 *
 * @public              @memberof k8s
 * @param   value       Receives result.
 * @param   str         Input string.
 * @return              Operation status. 
 */
kFx(kStatus) k8s_Parse(k8s* value, const kChar* str);

/** 
 * Converts k16u value to string. 
 * 
 * If the buffer is insufficient, the copy will transfer as many characters
 * as possible, null-terminate the resulting string, and return kERROR_INCOMPLETE. 
 * 
 * If the capacity is zero, kERROR_PARAMETER will be returned. 
 *
 * @public              @memberof k16u
 * @param   value       Input value.
 * @param   buffer      Destination for the string copy.
 * @param   capacity    Capacity of destination buffer, in characters.
 * @return              Operation status. 
 */
kFx(kStatus) k16u_Format(k16u value, kChar* buffer, kSize capacity);

/** 
 * Converts string to k16u value. 
 *
 * @public              @memberof k16u
 * @param   value       Receives result.
 * @param   str         Input string.
 * @return              Operation status. 
 */
kFx(kStatus) k16u_Parse(k16u* value, const kChar* str);

/** 
 * Converts k16s value to string. 
 * 
 * If the buffer is insufficient, the copy will transfer as many characters
 * as possible, null-terminate the resulting string, and return kERROR_INCOMPLETE. 
 * 
 * If the capacity is zero, kERROR_PARAMETER will be returned. 
 *
 * @public              @memberof k16s
 * @param   value       Input value.
 * @param   buffer      Destination for the string copy.
 * @param   capacity    Capacity of destination buffer, in characters.
 * @return              Operation status. 
 */
kFx(kStatus) k16s_Format(k16s value, kChar* buffer, kSize capacity);

/** 
 * Converts string to k16s value. 
 *
 * @public              @memberof k16s
 * @param   value       Receives result.
 * @param   str         Input string.
 * @return              Operation status. 
 */
kFx(kStatus) k16s_Parse(k16s* value, const kChar* str);

/** 
 * Converts k32u value to string. 
 * 
 * If the buffer is insufficient, the copy will transfer as many characters
 * as possible, null-terminate the resulting string, and return kERROR_INCOMPLETE. 
 * 
 * If the capacity is zero, kERROR_PARAMETER will be returned. 
 *
 * @public              @memberof k32u
 * @param   value       Input value.
 * @param   buffer      Destination for the string copy.
 * @param   capacity    Capacity of destination buffer, in characters.
 * @return              Operation status. 
 */
kFx(kStatus) k32u_Format(k32u value, kChar* buffer, kSize capacity);

/** 
 * Converts string to k32u value. 
 *
 * @public              @memberof k32u
 * @param   value       Receives result.
 * @param   str         Input string.
 * @return              Operation status. 
 */
kFx(kStatus) k32u_Parse(k32u* value, const kChar* str);

/** 
 * Converts k32s value to string. 
 * 
 * If the buffer is insufficient, the copy will transfer as many characters
 * as possible, null-terminate the resulting string, and return kERROR_INCOMPLETE. 
 * 
 * If the capacity is zero, kERROR_PARAMETER will be returned. 
 *
 * @public              @memberof k32s
 * @param   value       Input value.
 * @param   buffer      Destination for the string copy.
 * @param   capacity    Capacity of destination buffer, in characters.
 * @return              Operation status. 
 */
kFx(kStatus) k32s_Format(k32s value, kChar* buffer, kSize capacity);

/** 
 * Converts string to k32s value. 
 *
 * @public              @memberof k32s
 * @param   value       Receives result.
 * @param   str         Input string.
 * @return              Operation status. 
 */
kFx(kStatus) k32s_Parse(k32s* value, const kChar* str);

/** 
 * Converts k64u value to string. 
 * 
 * If the buffer is insufficient, the copy will transfer as many characters
 * as possible, null-terminate the resulting string, and return kERROR_INCOMPLETE. 
 * 
 * If the capacity is zero, kERROR_PARAMETER will be returned. 
 *
 * @public              @memberof k64u
 * @param   value       Input value.
 * @param   buffer      Destination for the string copy.
 * @param   capacity    Capacity of destination buffer, in characters.
 * @return              Operation status. 
 */
kFx(kStatus) k64u_Format(k64u value, kChar* buffer, kSize capacity);

/** 
 * Converts string to k64u value. 
 *
 * @public              @memberof k64u
 * @param   value       Receives result.
 * @param   str         Input string.
 * @return              Operation status. 
 */
kFx(kStatus) k64u_Parse(k64u* value, const kChar* str);

/** 
 * Converts k64s value to string. 
 * 
 * If the buffer is insufficient, the copy will transfer as many characters
 * as possible, null-terminate the resulting string, and return kERROR_INCOMPLETE. 
 * 
 * If the capacity is zero, kERROR_PARAMETER will be returned. 
 *
 * @public              @memberof k64s
 * @param   value       Input value.
 * @param   buffer      Destination for the string copy.
 * @param   capacity    Capacity of destination buffer, in characters.
 * @return              Operation status. 
 */
kFx(kStatus) k64s_Format(k64s value, kChar* buffer, kSize capacity);

/** 
 * Converts string to k64s value. 
 *
 * @public              @memberof k64s
 * @param   value       Receives result.
 * @param   str         Input string.
 * @return              Operation status. 
 */
kFx(kStatus) k64s_Parse(k64s* value, const kChar* str);

/** 
 * Converts kBool value to string. 
 * 
 * If the buffer is insufficient, the copy will transfer as many characters
 * as possible, null-terminate the resulting string, and return kERROR_INCOMPLETE. 
 * 
 * If the capacity is zero, kERROR_PARAMETER will be returned. 
 *
 * @public              @memberof kBool
 * @param   value       Input value.
 * @param   buffer      Destination for the string copy.
 * @param   capacity    Capacity of destination buffer, in characters.
 * @return              Operation status. 
 */
kFx(kStatus) kBool_Format(kBool value, kChar* buffer, kSize capacity);

/** 
 * Converts string to kBool value. 
 *
 * @public              @memberof kBool
 * @param   value       Receives result.
 * @param   str         Input string.
 * @return              Operation status. 
 */
kFx(kStatus) kBool_Parse(kBool* value, const kChar* str);

/** 
 * Converts kSize value to string. 
 * 
 * If the buffer is insufficient, the copy will transfer as many characters
 * as possible, null-terminate the resulting string, and return kERROR_INCOMPLETE. 
 * 
 * If the capacity is zero, kERROR_PARAMETER will be returned. 
 *
 * @public              @memberof kSize
 * @param   value       Input value.
 * @param   buffer      Destination for the string copy.
 * @param   capacity    Capacity of destination buffer, in characters.
 * @return              Operation status. 
 */
kFx(kStatus) kSize_Format(kSize value, kChar* buffer, kSize capacity);

/** 
 * Converts string to kSize value. 
 *
 * @public              @memberof kSize
 * @param   value       Receives result.
 * @param   str         Input string.
 * @return              Operation status. 
 */
kFx(kStatus) kSize_Parse(kSize* value, const kChar* str);

/** 
 * Converts kSSize value to string. 
 * 
 * If the buffer is insufficient, the copy will transfer as many characters
 * as possible, null-terminate the resulting string, and return kERROR_INCOMPLETE. 
 * 
 * If the capacity is zero, kERROR_PARAMETER will be returned. 
 *
 * @public              @memberof kSSize
 * @param   value       Input value.
 * @param   buffer      Destination for the string copy.
 * @param   capacity    Capacity of destination buffer, in characters.
 * @return              Operation status. 
 */
kFx(kStatus) kSSize_Format(kSSize value, kChar* buffer, kSize capacity);

/** 
 * Converts string to kSSize value. 
 *
 * @public              @memberof kSSize
 * @param   value       Receives result.
 * @param   str         Input string.
 * @return              Operation status. 
 */
kFx(kStatus) kSSize_Parse(kSSize* value, const kChar* str);

/** 
 * Converts k32f value to string. 
 * 
 * If the buffer is insufficient, the copy will transfer as many characters
 * as possible, null-terminate the resulting string, and return kERROR_INCOMPLETE. 
 * 
 * If the capacity is zero, kERROR_PARAMETER will be returned. 
 *
 * @public              @memberof k32f
 * @param   value       Input value.
 * @param   buffer      Destination for the string copy.
 * @param   capacity    Capacity of destination buffer, in characters.
 * @return              Operation status. 
 */
kFx(kStatus) k32f_Format(k32f value, kChar* buffer, kSize capacity);

/** 
 * Converts string to k32f value. 
 *
 * @public              @memberof k32f
 * @param   value       Receives result.
 * @param   str         Input string.
 * @return              Operation status. 
 */
kFx(kStatus) k32f_Parse(k32f* value, const kChar* str);

/** 
 * Converts k64f value to string. 
 * 
 * If the buffer is insufficient, the copy will transfer as many characters
 * as possible, null-terminate the resulting string, and return kERROR_INCOMPLETE. 
 * 
 * If the capacity is zero, kERROR_PARAMETER will be returned. 
 *
 * @public              @memberof k64f
 * @param   value       Input value.
 * @param   buffer      Destination for the string copy.
 * @param   capacity    Capacity of destination buffer, in characters.
 * @return              Operation status. 
 */
kFx(kStatus) k64f_Format(k64f value, kChar* buffer, kSize capacity);

/** 
 * Converts string to k64f value. 
 *
 * @public              @memberof k64f
 * @param   value       Receives result.
 * @param   str         Input string.
 * @return              Operation status. 
 */
kFx(kStatus) k64f_Parse(k64f* value, const kChar* str);

/**
 * @struct  kPoint16s
 * @extends kValue
 * @ingroup kApi-Data
 * @brief   2D point structure with 16-bit signed integer fields.
 *
 * kPoint16s supports the kdat5 and kdat6 serialization protocols.
 */
typedef struct kPoint16s
{
    k16s x;     ///< X-coordinate value.
    k16s y;     ///< Y-coordinate value.
} kPoint16s;

/**
 * @struct  kPoint32s
 * @extends kValue
 * @ingroup kApi-Data  
 * @brief   2D point structure with 32-bit signed integer fields.
 *
 * kPoint32s supports the kdat5 and kdat6 serialization protocols.
 */
typedef struct kPoint32s
{
    k32s x;     ///< X-coordinate value.
    k32s y;     ///< Y-coordinate value.
} kPoint32s; 

/**
 * @struct  kPoint32f
 * @extends kValue
 * @ingroup kApi-Data  
 * @brief   2D point structure with 32-bit floating-point fields.
 *
 * kPoint32f supports the kdat5 and kdat6 serialization protocols.
 */
typedef struct kPoint32f
{
    k32f x;     ///< X-coordinate value.
    k32f y;     ///< Y-coordinate value.
} kPoint32f;  

/**
 * @struct  kPoint64f
 * @extends kValue
 * @ingroup kApi-Data  
 * @brief   2D point structure with 64-bit floating-point fields.
 *
 * kPoint64f supports the kdat5 and kdat6 serialization protocols.
 */
typedef struct kPoint64f
{
    k64f x;     ///< X-coordinate value.
    k64f y;     ///< Y-coordinate value.
} kPoint64f;

/** 
 * Initializes a point structure. 
 * 
 * Warning: This macro evaluates its arguments more than once. 
 *
 * @param   POINT  Pointer to a point structure.    
 * @param   X      x field value. 
 * @param   Y      y field value. 
 */
#define kPoint_Init_(POINT, X, Y)     xkPoint_Init_(POINT, X, Y)  

/**
 * @struct  kPoint3d16s
 * @extends kValue
 * @ingroup kApi-Data  
 * @brief   3D point structure with 16-bit signed integer fields.
 *
 * kPoint3d16s supports the kdat5 and kdat6 serialization protocols.
 */
typedef struct kPoint3d16s
{
    k16s x;     ///< X-coordinate value.
    k16s y;     ///< Y-coordinate value.
    k16s z;     ///< Z-coordinate value.
} kPoint3d16s;

/**
 * @struct  kPoint3d32s
 * @extends kValue
 * @ingroup kApi-Data  
 * @brief   3D point structure with 32-bit signed integer fields.
 *
 * kPoint3d32s supports the kdat5 and kdat6 serialization protocols.
 */
typedef struct kPoint3d32s
{
    k32s x;     ///< X-coordinate value.
    k32s y;     ///< Y-coordinate value.
    k32s z;     ///< Z-coordinate value.
} kPoint3d32s;  

/**
 * @struct  kPoint3d32f
 * @extends kValue
 * @ingroup kApi-Data  
 * @brief   3D point structure with 32-bit floating-point fields.
 *
 * kPoint3d32s supports the kdat5 and kdat6 serialization protocols.
 */
typedef struct kPoint3d32f
{
    k32f x;     ///< X-coordinate value.
    k32f y;     ///< Y-coordinate value.
    k32f z;     ///< Z-coordinate value.
} kPoint3d32f; 

/**
 * @struct  kPoint3d64f
 * @extends kValue
 * @ingroup kApi-Data  
 * @brief   3D point structure with 64-bit floating-point fields.
 *
 * kPoint3d64f supports the kdat5 and kdat6 serialization protocols.
 */
typedef struct kPoint3d64f
{
    k64f x;     ///< X-coordinate value.
    k64f y;     ///< Y-coordinate value.
    k64f z;     ///< Z-coordinate value.
} kPoint3d64f;  

/** 
 * Initializes a 3d point structure. 
 * 
 * Warning: This macro evaluates its arguments more than once. 
 *
 * @param   POINT  Pointer to a 3d point structure.    
 * @param   X      x field value. 
 * @param   Y      y field value. 
 * @param   Z      z field value. 
 */
#define kPoint3d_Init_(POINT, X, Y, Z)   xkPoint3d_Init_(POINT, X, Y, Z)

/**
 * @struct  kPoint4d16s
 * @extends kValue
 * @ingroup kApi-Data  
 * @brief   4D point structure with 16-bit signed integer fields.
 *
 * kPoint4d16s supports the kdat6 serialization protocols.
 */
typedef struct kPoint4d16s
{
    k16s x;     ///< X-coordinate value.
    k16s y;     ///< Y-coordinate value.
    k16s z;     ///< Z-coordinate value.
    k16s w;     ///< W-coordinate value.
} kPoint4d16s;

/** 
 * Initializes a 4d point structure. 
 * 
 * Warning: This macro evaluates its arguments more than once. 
 *
 * @param   POINT  Pointer to a 4d point structure.    
 * @param   X      x field value. 
 * @param   Y      y field value. 
 * @param   Z      z field value. 
 * @param   W      w field value. 
 */
#define kPoint4d_Init_(POINT, X, Y, Z, W)   xkPoint4d_Init_(POINT, X, Y, Z, W)

/**
 * @struct  kRect16s
 * @extends kValue
 * @ingroup kApi-Data  
 * @brief   Rectangle structure with 16-bit signed integer fields.
 *
 * kRect16s supports the kdat6 serialization protocol.
 */
typedef struct kRect16s
{
    k16s x;         ///< X-coordinate of the origin.
    k16s y;         ///< Y-coordinate of the origin.
    k16s width;     ///< Width of the rectangle.
    k16s height;    ///< Height of the rectangle.
} kRect16s;        

/**
 * @struct  kRect32s
 * @extends kValue
 * @ingroup kApi-Data  
 * @brief   Rectangle structure with 32-bit signed integer fields.
 *
 * kRect32s supports the kdat5 and kdat6 serialization protocols.
 */
typedef struct kRect32s
{
    k32s x;         ///< X-coordinate of the origin.
    k32s y;         ///< Y-coordinate of the origin.
    k32s width;     ///< Width of the rectangle.
    k32s height;    ///< Height of the rectangle.
} kRect32s;        

/**
 * @struct  kRect32f
 * @extends kValue
 * @ingroup kApi-Data  
 * @brief   Rectangle structure with 32-bit floating-point fields. 
 *
 * kRect32f supports the kdat5 and kdat6 serialization protocols.
 */
typedef struct kRect32f
{
    k32f x;         ///< X-coordinate of the origin.
    k32f y;         ///< Y-coordinate of the origin.
    k32f width;     ///< Width of the rectangle.
    k32f height;    ///< Height of the rectangle.
} kRect32f;     

/**
 * @struct  kRect64f
 * @extends kValue
 * @ingroup kApi-Data  
 * @brief   Rectangle structure with 64-bit floating-point fields. 
 *
 * kRect64f supports the kdat5 and kdat6 serialization protocols.
 */
typedef struct kRect64f
{
    k64f x;         ///< X-coordinate of the origin.
    k64f y;         ///< Y-coordinate of the origin.
    k64f width;     ///< Width of the rectangle.
    k64f height;    ///< Height of the rectangle.
} kRect64f;         

/** 
 * Initializes a rectangle structure. 
 * 
 * Warning: This macro evaluates its arguments more than once. 
 *
 * @param   RECT   Pointer to a rectangle structure.    
 * @param   X      x field value. 
 * @param   Y      y field value. 
 * @param   W      width field value. 
 * @param   H      height field value. 
 */
#define kRect_Init_(RECT, X, Y, W, H)    \
    xkRect_Init_(RECT, X, Y, W, H) 

/**
 * @struct  kRect3d64f
 * @extends kValue
 * @ingroup kApi-Data  
 * @brief   Rectangular cuboid structure with 64-bit floating-point fields. 
 *
 * kRect3d64f supports the kdat6 serialization protocol.
 */
typedef struct kRect3d64f
{
    k64f x;         ///< X-coordinate of the origin.
    k64f y;         ///< Y-coordinate of the origin.
    k64f z;         ///< Z-coordinate of the origin.
    k64f width;     ///< Width of the rectangular cuboid.
    k64f height;    ///< Height of the rectangular cuboid.
    k64f depth;     ///< Depth of the rectangular cuboid.
} kRect3d64f;

/** 
 * Initializes a rectangular cuboid structure. 
 * 
 * @param   RECT   Pointer to rectangular cuboid structure.    
 * @param   X      x field value. 
 * @param   Y      y field value. 
 * @param   Z      z field value.
 * @param   W      width field value. 
 * @param   H      height field value.
 * @param   D      depth field value.
 */
#define kRect3d_Init_(RECT, X, Y, Z, W, H, D)    \
    xkRect3d_Init_(RECT, X, Y, Z, W, H, D) 

/**
 * @struct  kRotatedRect32s
 * @extends kValue
 * @ingroup kApi-Data  
 * @brief   Rotated rectangle structure with 32-bit signed integer fields. 
 *
 * kRotatedRect32s supports the kdat5 and kdat6 serialization protocols.
 */
typedef struct kRotatedRect32s
{
    k32s xc;        ///< X-coordinate of the rectangle center.
    k32s yc;        ///< Y-coordinate of the rectangle center.
    k32s width;     ///< Width of the rectangle.
    k32s height;    ///< Height of the rectangle.
    k32s angle;     ///< Rotation angle of the rectangle.
} kRotatedRect32s;

/**
 * @struct  kRotatedRect32f
 * @extends kValue
 * @ingroup kApi-Data  
 * @brief   Rotated rectangle structure with 32-bit floating-point fields. 
 *
 * kRotatedRect32f supports the kdat5 and kdat6 serialization protocols.
 */
typedef struct kRotatedRect32f
{
    k32f xc;        ///< X-coordinate of the rectangle center.
    k32f yc;        ///< Y-coordinate of the rectangle center.
    k32f width;     ///< Width of the rectangle.
    k32f height;    ///< Height of the rectangle.
    k32f angle;     ///< Rotation angle of the rectangle.
} kRotatedRect32f;  

/** 
 * Initializes a rotated rectangle structure. 
 * 
 * Warning: This macro evaluates its arguments more than once. 
 *
 * @param   RECT   Pointer to a rotated rectangle structure.    
 * @param   XC     xc field value. 
 * @param   YC     yc field value. 
 * @param   W      width field value. 
 * @param   H      height field value. 
 * @param   A      angle field value. 
 */
#define kRotatedRect_Init_(RECT, XC, YC, W, H, A)   \
        xkRotatedRect_Init_(RECT, XC, YC, W, H, A) 

/**
 * @struct  kPixelFormat
 * @extends kValue
 * @ingroup kApi-Data  
 * @brief   Pixel format descriptor.
 *
 * kPixelFormat supports the kdat6 serialization protocol.
 */
typedef k32s kPixelFormat; 

/** @relates kPixelFormat @{ */
#define kPIXEL_FORMAT_NULL              (0)      ///< Unknown pixel format.
#define kPIXEL_FORMAT_8BPP_GREYSCALE    (1)      ///< 8-bit greyscale (k8u)
#define kPIXEL_FORMAT_8BPP_CFA          (2)      ///< 8-bit color filter array (k8u)
#define kPIXEL_FORMAT_8BPC_BGRX         (3)      ///< 8-bits-per-channel color with 4 channels (blue/green/red/unused)(kRgb)
#define kPIXEL_FORMAT_1BPP_GREYSCALE    (4)      ///< 1-bit greyscale, 8 packed pixels per image element (k8u)
#define kPIXEL_FORMAT_16BPP_GREYSCALE   (5)      ///< 16-bit greyscale (k16u)
/** @} */

/**
 * @struct  kCfa
 * @extends kValue
 * @ingroup kApi-Data  
 * @brief   Image color filter array type.
 *
 * kCfa supports the kdat6 serialization protocol.
 */
typedef k32s kCfa; 

/** @relates kCfa @{ */
#define kCFA_NONE            (0)      ///< No color filter.
#define kCFA_BAYER_BGGR      (1)      ///< Bayer filter: BG/GR.
#define kCFA_BAYER_GBRG      (2)      ///< Bayer filter: GB/RG.
#define kCFA_BAYER_RGGB      (3)      ///< Bayer filter: RG/GB.
#define kCFA_BAYER_GRBG      (4)      ///< Bayer filter: GR/BG.
/** @} */

/**
 * @struct  kRgb
 * @extends kValue
 * @ingroup kApi-Data  
 * @brief   32-bit color pixel structure (B/G/R/X). 
 *
 * kRgb supports the kdat5 and kdat6 serialization protocols.
 */
typedef struct kRgb
{
    k8u b;      ///< Blue component value.
    k8u g;      ///< Green component value.
    k8u r;      ///< Red component value.
    k8u x;      ///< Undefined.
} kRgb;

/** 
 * Initializes a kRgb structure. 
 * 
 * @relates        kRgb
 * @param   RGB    Pointer to a kRgb structure.    
 * @param   R      r field value. 
 * @param   G      g field value. 
 * @param   B      b field value. 
 */
#define kRgb_Init_(RGB, R, G, B)    \
    xkRgb_Init_(RGB, R, G, B) 

/**
 * @struct  kArgb
 * @extends kValue
 * @ingroup kApi-Data  
 * @brief   32-bit color pixel structure (B/G/R/A). 
 *
 * kArgb supports the kdat5 and kdat6 serialization protocols.
 */
typedef struct kArgb
{
    k8u b;      ///< Blue component value.
    k8u g;      ///< Green component value.
    k8u r;      ///< Red component value.
    k8u a;      ///< Alpha component value.
} kArgb;

/** 
 * Initializes a kArgb structure. 
 * 
 * @relates        kArgb
 * @param   ARGB   Pointer to a kArgb structure.
 * @param   A      a field value. 
 * @param   R      r field value. 
 * @param   G      g field value. 
 * @param   B      b field value. 
 */
#define kArgb_Init_(ARGB, A, R, G, B)   \
    xkArgb_Init_(ARGB, A, R, G, B)

/**
 * @struct  kMacAddress
 * @extends kValue
 * @ingroup kApi-Io
 * @brief   Represents an Ethernet address. 
 */
typedef struct kMacAddress
{
    kByte address[6];            ///< Address bytes (most significant byte first). 
} kMacAddress; 

/**
 * @struct  kMemoryAlignment
 * @extends kValue
 * @ingroup kApi
 * @brief   Represents alignment options for allocations.
 */
typedef k32s kMemoryAlignment;

/** @relates kMemoryAlignment @{ */
#define kMEMORY_ALIGNMENT_8                      (3) ///< 8 bytes alignment.
#define kMEMORY_ALIGNMENT_16                     (4) ///< 16 bytes alignment.
#define kMEMORY_ALIGNMENT_32                     (5) ///< 32 bytes alignment.
#define kMEMORY_ALIGNMENT_64                     (6) ///< 64 bytes alignment.
#define kMEMORY_ALIGNMENT_128                    (7) ///< 128 bytes alignment.
#define kMEMORY_ALIGNMENT_256                    (8) ///< 256 bytes alignment.
#define kMEMORY_ALIGNMENT_512                    (9) ///< 512 bytes alignment.
#define kMEMORY_ALIGNMENT_1024                   (10) ///< 1024 bytes alignment.
#define kMEMORY_ALIGNMENT_2048                   (11) ///< 2048 bytes alignment.
#define kMEMORY_ALIGNMENT_4096                   (12) ///< 4096 bytes alignment.
/** @} */

/**
* Calculates number of bytes for specified alignment option.
*
* @param   alignment Memory alignment.
* @return            Number of bytes in alignment.
*/
kInlineFx(kSize) kMemoryAlignment_Size(kMemoryAlignment alignment)
{
    return ((kSize)1 << (kSize)alignment);
}

/** 
 * Parses a text-formatted Ethernet MAC address. 
 *
 * Supports hexadecimal, dash-separated format (e.g. "A0-B1-C2-D3-E4-F5"). 
 *
 * @public              @memberof kMacAddress
 * @param   address     Receives the MAC address. 
 * @param   text        Text-formatted MAC address.
 * @return              Operation status. 
 */
kFx(kStatus) kMacAddress_Parse(kMacAddress* address, const kChar* text); 

/** 
 * Formats an Ethernet MAC address as a string. 
 *
 * Emits hexadecimal, dash-separated format (e.g. "A0-B1-C2-D3-E4-F5"). 
 * 
 * @public              @memberof kMacAddress
 * @param   address     MAC address.
 * @param   text        Receives formatted string.
 * @param   capacity    Capacity of the string buffer. 
 * @return              Operation status. 
 */
kFx(kStatus) kMacAddress_Format(kMacAddress address, kChar* text, kSize capacity); 

/**
 * @struct  kComparison
 * @extends kValue
 * @ingroup kApi-Data  
 * @brief   Represents a comparison type.
 */
typedef k32s kComparison; 

#define kCOMPARISON_EQ      (0)        ///< Is equal. @relates kComparison
#define kCOMPARISON_NEQ     (1)        ///< Is not equal. @relates kComparison
#define kCOMPARISON_LT      (2)        ///< Is less than. @relates kComparison
#define kCOMPARISON_LTE     (3)        ///< Is less than or equal. @relates kComparison
#define kCOMPARISON_GT      (4)        ///< Is greater than. @relates kComparison
#define kCOMPARISON_GTE     (5)        ///< Is greater than or equal. @relates kComparison

/** 
 * Callback signature to determine equality of two items.
 * 
 * @param   item1  Pointer to first item.    
 * @param   item2  Pointer to second item. 
 * @return         kTRUE if the arguments are equal; kFALSE otherwise. 
 */
typedef kBool (kCall* kEqualsFx)(const void* item1, const void* item2); 

/** 
 * Callback signature to determine hash code of an item.
 * 
 * @param   item   Pointer to item.    
 * @return         Item hash code.  
 */
typedef kSize (kCall* kHashFx)(const void* item); 

/** 
 * Callback signature for a generic event handler.
 * 
 * @param   receiver   Receiver context pointer.     
 * @param   sender     Sender context pointer. 
 * @param   args       Pointer to callback argument.     
 * @return             Operation status. 
 */
typedef kStatus (kCall* kCallbackFx)(kPointer receiver, kPointer sender, void* args); 

/**
 * @struct  kCallback
 * @extends kValue
 * @ingroup kApi  
 * @brief   Represents a callback function and context pointer.
 */
typedef struct kCallback
{
    kCallbackFx function;       ///< Callback function.
    kPointer receiver;          ///< Callback receiver context pointer. 
} kCallback; 

/**
 * @struct  kFileMode
 * @extends kValue
 * @ingroup kApi-Io  
 * @brief   Flags that control how a file is opened. 
 */
typedef k32s kFileMode;         

/** @relates kFileMode @{ */
#define kFILE_MODE_READ         (0x1)     ///< Open the file with permission to read. 
#define kFILE_MODE_WRITE        (0x2)     ///< Open the file with permission to write.
#define kFILE_MODE_UPDATE       (0x4)     ///< Preserve contents when opened for writing.   
/** @} */

/**
 * @struct  kSeekOrigin
 * @extends kValue
 * @ingroup kApi-Io  
 * @brief   Represents a stream seek origin.
 */
typedef k32s kSeekOrigin; 

/** @relates kSeekOrigin @{ */
#define kSEEK_ORIGIN_BEGIN      (0)     ///< Seek relative to the start of stream. @relates kSeekOrigin
#define kSEEK_ORIGIN_CURRENT    (1)     ///< Seek relative to the current position. @relates kSeekOrigin
#define kSEEK_ORIGIN_END        (2)     ///< Seek relative to the end of stream. @relates kSeekOrigin
/** @} */

/**
 * @struct  kCompressionType
 * @extends kValue
 * @ingroup kApi
 * @brief   Type of compression algorithm. 
 */
typedef k32s kCompressionType; 

/** @relates kCompressionType @{ */
#define kCOMPRESSION_TYPE_NULL       (0)     ///< None. 
#define kCOMPRESSION_TYPE_ZSTD       (1)     ///< Zstandard compression.

/** @} */

/**
 * @struct  kCompressionPreset
 * @extends kValue
 * @ingroup kApi
 * @brief   Preset compression levels. 
 */
typedef k32s kCompressionPreset; 

/** @relates kCompressionPreset @{ */
#define kCOMPRESSION_PRESET_MIN       (-1)      ///< Minimum compression density supported by algorithm (not recommened).  
#define kCOMPRESSION_PRESET_FAST      (-2)      ///< Recommended setting for fast compression.
#define kCOMPRESSION_PRESET_DEFAULT   (-3)      ///< Recommended setting for a balance of speed and density.
#define kCOMPRESSION_PRESET_DENSE     (-4)      ///< Recommended setting for dense compression.
#define kCOMPRESSION_PRESET_MAX       (-5)      ///< Maximum compression density supported by algorithm (not recommened).
/** @} */

/**
 * @struct   kLogOption
 * @extends  kValue
 * @ingroup  kApi
 * @brief    Represents options associated with a log entry.
 *
 * The lower 32 bits of an option value are reserved for platform-defined 
 * purposes; applications can use the upper 32 bits as desired. 
 */
typedef k64u kLogOption;

/** @relates kLogOption @{ */
#define kLOG_OPTION_WARNING         (0x1)    ///< Warning message.
#define kLOG_OPTION_ERROR           (0x2)    ///< Error message.
#define kLOG_OPTION_PLATFORM        (0x10)   ///< Platform-generated message.
/** @} */

/**
* Returns the number of elements in a C array.
*
* Equivalent to sizeof(CARRAY)/sizeof(CARRAY[0]).
*
* @param   CARRAY     C array variable name.
* @return             Count of array elements.
*/
#define kCountOf(CARRAY)    \
    (sizeof(CARRAY)/sizeof(CARRAY[0]))  

/**
* Casts the ITEM argument to the specified TYPE.
*
* Equivalent to (TYPE)(ITEM).
*
* @param   TYPE     Type to which the item is cast.
* @param   ITEM     Value to be cast.
* @return           Result of the cast.
*/
#define kCast(TYPE, ITEM)       \
    ((TYPE)(ITEM)) 

#if (K_CPP_VERSION >= K_CPP_VERSION_2011)

/**
* Returns the absolute value of an unsigned number.
*
* This method requires C++ 2011; accordingly, it cannot be used in inline functions that may be 
* processed by C compilers or older C++ compilers.
*
* @param   v   Input value.
* @return      Absolute value of input argument.
*/
template <typename T>
static kInline typename std::enable_if<std::is_unsigned<T>::value, T>::type kAbs(const T& v)
{
    return v;
}

/**
* Returns the absolute value of a signed number.
*
* This method requires C++ 2011; accordingly, it cannot be used in inline functions that may be 
* processed by C compilers or older C++ compilers.
*
* @param   v   Input value.
* @return      Absolute value of input argument.
*/
template <typename T>
static kInline typename std::enable_if<!std::is_unsigned<T>::value, T>::type kAbs(const T& v)
{
    return (v >= T()) ? v : -v;
}

/**
* Returns the sign of an unsigned number.
*
* This method requires C++ 2011; accordingly, it cannot be used in inline functions that may be 
* processed by C compilers or older C++ compilers.
*
* @param   v   Input value.
* @return      1 for positive input or 0 for zero input.
*/
template <typename T>
static kInline typename std::enable_if<std::is_unsigned<T>::value, T>::type kSign(const T& v)
{
    return v > T();
}

/**
* Returns the sign of a signed number.
*
* This method requires C++ 2011; accordingly, it cannot be used in inline functions that may be 
* processed by C compilers or older C++ compilers.
*
* @param   v   Input value.
* @return      1 for positive input, -1 for negative input, or 0 for zero input.
*/
template <typename T>
static kInline typename std::enable_if<!std::is_unsigned<T>::value, T>::type kSign(const T& v)
{
    return (v > T()) - (v < T());
}

/**
* Returns the minimum of two numbers.
*
* This method requires C++ 2011; accordingly, it cannot be used in inline functions that may be 
* processed by C compilers or older C++ compilers.
*
* @param   a   First value.
* @param   b   Second value.
* @return      The lesser of a or b.
*/
template <typename T>
kInlineFx(T) kMin(const T& a, const T& b)
{
    return (a < b) ? a : b;
}

/**
* Returns the maximum of two numbers.
*
* This method requires C++ 2011; accordingly, it cannot be used in inline functions that may be 
* processed by C compilers or older C++ compilers.
*
* @param   a   First value.
* @param   b   Second value.
* @return      The greater of a or b.
*/
template <typename T>
kInlineFx(T) kMax(const T& a, const T& b)
{
    return (a > b) ? a : b;
}

/**
* Returns a value limited to the specified range.
*
* This method requires C++ 2011; accordingly, it cannot be used in inline functions that may be 
* processed by C compilers or older C++ compilers.
*
* @param   v           Input value.
* @param   min         Minimum output value.
* @param   max         Maximum output value.
* @return              The input value, limited to [min, max].
*/
template <typename T>
kInlineFx(T) kClamp(const T& v, const T& min, const T& max)
{
    return kMin(kMax(v, min), max);
}

/**
* Calculates the quotient of two unsigned integers, rounding down.  
*
* This method requires C++ 2011; accordingly, it cannot be used in inline functions that may be 
* processed by C compilers or older C++ compilers.
*
* @param   a   Dividend. 
* @param   b   Divisor. 
* @return      Quotient, rounded down. 
*/    
template <typename T>
static kInline typename std::enable_if<std::is_unsigned<T>::value, T>::type kDivideFloor(const T& a, const T& b)
{
    return a / b;
}

/**
* Calculates the quotient of two signed integers, rounding down.  
*
* This method requires C++ 2011; accordingly, it cannot be used in inline functions that may be 
* processed by C compilers or older C++ compilers. 
*
* @param   a   Dividend. 
* @param   b   Divisor. 
* @return      Quotient, rounded down. 
*/    
template <typename T>
static kInline typename std::enable_if<!std::is_unsigned<T>::value, T>::type kDivideFloor(const T& a, const T& b)
{
    return ((kSign(a) * kSign(b)) >= T()) ? a / b : ((a / b) - ((a % b) != T()));
}

/**
* Calculates the quotient of two unsigned integers, rounding up.  
*
* This method requires C++ 2011; accordingly, it cannot be used in inline functions that may be 
* processed by C compilers or older C++ compilers.
*
* @param   a   Dividend. 
* @param   b   Divisor. 
* @return      Quotient, rounded up. 
*/  
template <typename T>
static kInline typename std::enable_if<std::is_unsigned<T>::value, T>::type kDivideCeil(const T& a, const T& b)
{
    return (a / b) + ((a % b) != T());
}

/**
* Calculates the quotient of two signed integers, rounding up.  
*
* This method requires C++ 2011; accordingly, it cannot be used in inline functions that may be 
* processed by C compilers or older C++ compilers.
*
* @param   a   Dividend. 
* @param   b   Divisor. 
* @return      Quotient, rounded up. 
*/  
template <typename T>
static kInline typename std::enable_if<!std::is_unsigned<T>::value, T>::type kDivideCeil(const T& a, const T& b)
{
    return ((kSign(a) * kSign(b)) <= T()) ? a / b : ((a / b) + ((a % b) != T()));
}

/**
* Rounds the specified input value down to the nearest multiple of the specified granularity. 
* 
* This method requires C++ 2011; accordingly, it cannot be used in inline functions that may be 
* processed by C compilers or older C++ compilers.
*
* @param   value        Input value. 
* @param   granularity  Granularity (quantum; must be positive).
* @return               Input value, rounded down to closest multiple of granularity. 
*/  
template <typename T>
kInlineFx(T) kQuantizeFloor(const T& value, const T& granularity)
{
    return kDivideFloor(value, granularity) * granularity;
}

/**
* Rounds the specified input value up to the nearest multiple of the specified granularity. 
*
* This method requires C++ 2011; accordingly, it cannot be used in inline functions that may be 
* processed by C compilers or older C++ compilers.
*
* @param   value         Input value. 
* @param   granularity   Granularity (quantum; must be positive).
* @return                Input value, rounded up to closest multiple of granularity. 
*/ 
template <typename T>
kInlineFx(T) kQuantizeCeil(const T& value, const T& granularity)
{
    return kDivideCeil(value, granularity) * granularity;
}

/**
* Rounds the specified input down, conforming to minimum, maximum, and granularity constraints.
*
* Granularity represents the step size (quantum) between valid output values. It is interpreted
* as relative to the specified min value. E.g., with min=3, max=7, and granularity=2, output
* values would be constrained to {3, 5, 7}.
* 
* This method requires C++ 2011; accordingly, it cannot be used in inline functions that may be 
* processed by C compilers or older C++ compilers.
*
* @param   value        Input value. 
* @param   min          Minimum value.
* @param   max          Maximum value.
* @param   granularity  Granularity (must be positive).
* @return               Input value, rounded down to nearest multiple of granularity between min and max.   
*/ 
template <typename T>
kInlineFx(T) kAdjustFloor(const T& value, const T& min, const T& max, const T& granularity)
{
    typedef typename std::make_unsigned<T>::type U;
    U relativeValue =  (value >= min) ? (U) (value - min) : U();
    U relativeMax = (max >= min) ? (U) (max - min) : U();    
    U quantizedRelativeMax = kQuantizeFloor<U>(relativeMax, granularity);
    U limitedRelativeValue = kMin<U>(relativeValue, quantizedRelativeMax); 

    return (T) kQuantizeFloor<U>(limitedRelativeValue, granularity) + min;
}

/**
* Rounds the specified input up, conforming to minimum, maximum, and granularity constraints.
*
* Granularity represents the step size (quantum) between valid output values. It is interpreted
* as relative to the specified min value. E.g., with min=3, max=7, and granularity=2, output
* values would be constrained to {3, 5, 7}.
* 
* This method requires C++ 2011; accordingly, it cannot be used in inline functions that may be 
* processed by C compilers or older C++ compilers.
*
* @param   value        Input value. 
* @param   min          Minimum value.
* @param   max          Maximum value.
* @param   granularity  Granularity (must be positive).
* @return               Input value, rounded up to nearest multiple of granularity between min and max.   
*/ 
template <typename T>
kInlineFx(T) kAdjustCeil(const T& value, const T& min, const T& max, const T& granularity)
{
    typedef typename std::make_unsigned<T>::type U;
    U relativeValue =  (value >= min) ? (U) (value - min) : U();
    U relativeMax = (max >= min) ? (U) (max - min) : U();    
    U quantizedRelativeMax = kQuantizeFloor<U>(relativeMax, granularity);
    U limitedRelativeValue = kMin<U>(relativeValue, quantizedRelativeMax); 

    return (T) kQuantizeCeil<U>(limitedRelativeValue, granularity) + min;
}

#endif

/**
* Sets all bits of a structure to zero.
*
* @param   VALUE   Structure instance.
*/
#define kZero(VALUE)           \
    memset(&VALUE, 0, sizeof(VALUE))

/**
* Performs a small copy with minimal overhead.
* 
* This method can be used as a lightweight replacement for kMemCopy when the 
* amount of memory to be copied is known to be small (e.g., < 1K). This provides 
* a small optimization on some platforms. 
* 
* If the amount of memory is not known to be very small, kMemCopy should be 
* used instead. 
*
* @param   dest    Destination address.
* @param   src     Source address.
* @param   size    Transfer size (bytes).
*/
kInlineFx(void) kItemCopy(void* dest, const void* src, kSize size)
{
    memcpy(dest, src, size);
}

/**
* Zero-initializes a small amount of memory with minimal overhead.
*
* This method can be used as a lightweight replacement for kMemZero when the 
* amount of memory to be copied is known to be small (e.g., < 1K). This provides 
* a small optimization on some platforms. 
* 
* If the amount of memory is not known to be very small, kMemZero should be 
* used instead. 
*
* @param   dest    Destination address.
* @param   size    Transfer size (bytes).
*/
kInlineFx(void) kItemZero(void* dest, kSize size)
{
    memset(dest, 0, size);
}

/**
 * @struct  kAllocTrait
 * @relates kAlloc
 * @brief   Represents a memory allocator trait.  
 */
typedef k32s kAllocTrait; 

/** @relates kAllocTrait @{ */
#define kALLOC_TRAIT_FOREIGN                (0x00000001)      ///< Allocates memory in a foreign memory domain (non-host address space). 
#define kALLOC_TRAIT_SERIAL                 (0x00000002)      ///< Allocates memory suitable for single-threaded use only.
#define kALLOC_TRAIT_NON_ATOMIC             (0x00000004)      ///< Allocates memory that cannot support atomic operations.
#define kALLOC_TRAIT_CONTEXT                (0x00000008)      ///< Allocator supports use of a context object during copy operations.
#define kALLOC_TRAIT_CUDA_PINNED            (0x00010000)      ///< Allocates Cuda pinned memory (host or device). 
#define kALLOC_TRAIT_CUDA_MANAGED           (0x00020000)      ///< Allocates Cuda managed memory (host or device).
#define kALLOC_TRAIT_CUDA_DEVICE            (0x01000000)      ///< Allocates Cuda device memory (device only). 
/** @} */

/**
* Reports whether the kALLOC_TRAIT_FOREIGN trait is present in an allocator trait bitset.
*
* @public           @memberof kAllocTrait
* @param   traits   Allocator trait bitset.
* @return           kTRUE if the kALLOC_TRAIT_FOREIGN trait is present. 
*/    
kInlineFx(kBool) kAllocTrait_IsForeign(kAllocTrait traits)
{
    return (traits & kALLOC_TRAIT_FOREIGN) != 0;
}

/**
* Reports whether the kALLOC_TRAIT_SERIAL trait is present in an allocator trait bitset.
*
* @public           @memberof kAllocTrait
* @param   traits   Allocator trait bitset.
* @return           kTRUE if the kALLOC_TRAIT_SERIAL trait is present. 
*/    
kInlineFx(kBool) kAllocTrait_IsSerial(kAllocTrait traits)
{
    return (traits & kALLOC_TRAIT_SERIAL) != 0;
}

/**
* Reports whether the kALLOC_TRAIT_NON_ATOMIC trait is present in an allocator trait bitset.
*
* @public           @memberof kAllocTrait
* @param   traits   Allocator trait bitset.
* @return           kTRUE if the kALLOC_TRAIT_NON_ATOMIC trait is present. 
*/    
kInlineFx(kBool) kAllocTrait_IsNonAtomic(kAllocTrait traits)
{
    return (traits & kALLOC_TRAIT_NON_ATOMIC) != 0;
}

/**
* Reports whether the kALLOC_TRAIT_CONTEXT trait is present in an allocator trait bitset.
*
* @public           @memberof kAllocTrait
* @param   traits   Allocator trait bitset.
* @return           kTRUE if the kALLOC_TRAIT_CONTEXT trait is present. 
*/    
kInlineFx(kBool) kAllocTrait_SupportsContext(kAllocTrait traits)
{
    return (traits & kALLOC_TRAIT_CONTEXT) != 0;
}

/**
* Reports whether the kALLOC_TRAIT_CUDA_PINNED trait is present in an allocator trait bitset.
*
* @public           @memberof kAllocTrait
* @param   traits   Allocator trait bitset.
* @return           kTRUE if the kALLOC_TRAIT_CUDA_PINNED trait is present. 
*/    
kInlineFx(kBool) kAllocTrait_IsCudaPinned(kAllocTrait traits)
{
    return  (traits & kALLOC_TRAIT_CUDA_PINNED) != 0;
}

/**
* Reports whether the kALLOC_TRAIT_CUDA_MANAGED trait is present in an allocator trait bitset.
*
* @public           @memberof kAllocTrait
* @param   traits   Allocator trait bitset.
* @return           kTRUE if the kALLOC_TRAIT_CUDA_MANAGED trait is present. 
*/    
kInlineFx(kBool) kAllocTrait_IsCudaManaged(kAllocTrait traits)
{
    return  (traits & kALLOC_TRAIT_CUDA_MANAGED) != 0;
}

/**
* Reports whether the kALLOC_TRAIT_CUDA_DEVICE trait is present in an allocator trait bitset.
*
* @public           @memberof kAllocTrait
* @param   traits   Allocator trait bitset.
* @return           kTRUE if the kALLOC_TRAIT_CUDA_DEVICE trait is present. 
*/    
kInlineFx(kBool) kAllocTrait_IsCudaDevice(kAllocTrait traits)
{
    return  (traits & kALLOC_TRAIT_CUDA_DEVICE) != 0;
}

/**
* Reports whether the memory is Cuda device-accessible.
*
* @public           @memberof kAllocTrait
* @param   traits   Allocator trait bitset.
* @return           kTRUE if the memory is Cuda device-accessible. 
*/    
kInlineFx(kBool) kAllocTrait_IsCudaDeviceAccessible(kAllocTrait traits)
{
    return  (traits & (kALLOC_TRAIT_CUDA_DEVICE | kALLOC_TRAIT_CUDA_MANAGED | kALLOC_TRAIT_CUDA_PINNED)) != 0;
}

/**
 * @struct  kThreadId
 * @extends kValue
 * @ingroup kApi-Threads
 * @brief   Represents a unique thread identifier. 
 * @see     kThread_CurrentId, kThread_Id.
 */
typedef kSize kThreadId; 

/**
* Compares two thread identifiers for equality.
*
* @public       @memberof kThreadId
* @param   a    First thread identifier.
* @param   b    Second thread identifier.
* @return       kTRUE if identifiers are equal. 
*/
kFx(kBool) kThreadId_Compare(kThreadId a, kThreadId b);

/**
 * @struct  kThreadPriorityClass
 * @extends kValue
 * @ingroup kApi-Threads
 * @brief   Represents a thread priority class. 
 * @see     kThread_Start
 */
typedef k32s kThreadPriorityClass; 

/** @relates kThreadPriorityClass @{ */
#define kTHREAD_PRIORITY_CLASS_LOW        (-1)    ///< Low priority thread class.
#define kTHREAD_PRIORITY_CLASS_NORMAL     (0)     ///< Normal priority thread class.
#define kTHREAD_PRIORITY_CLASS_HIGH       (1)     ///< High priority thread class.
/** @} */

/*
 * Forward declarations
 */
   
typedef void* kObject;
typedef kObject kType; 
typedef kObject kAlloc; 

/** @relates kType @{ */
#define kTypeName kText64       ///< Alias for type used to store a kType text name. 
/** @} */

/**
 * @struct  kTypeFlags
 * @extends kValue
 * @ingroup kApi  
 * @brief   Represents a set of type flags. 
 */
typedef k32u kTypeFlags; 

/** @relates kTypeFlags @{ */
#define kTYPE_FLAGS_CLASS           (0x01)       ///< Type is a class.
#define kTYPE_FLAGS_INTERFACE       (0x02)       ///< Type is an interface. 
#define kTYPE_FLAGS_VALUE           (0x04)       ///< Type is a value. 
#define kTYPE_FLAGS_ENUM            (0x08)       ///< Type is an enumeration.
#define kTYPE_FLAGS_ABSTRACT        (0x20)       ///< Type is an abstract class. 
#define kTYPE_FLAGS_ARRAY_VALUE     (0x40)       ///< Type is an array-based value (e.g. kText32).
#define kTYPE_FLAGS_PRIMITIVE       (0x80)       ///< Type is a primitive value (i.e., has no fields).
/** @} */

/**
 * @struct  kTypeVersion
 * @typedef kPointer kTypeVersion
 * @relates kType
 * @brief   Represents an opaque reference to type version information (used in object serialization). 
 */
typedef kPointer kTypeVersion; 

/** Required signature for kObject framework constructors */
typedef kStatus (kCall* kFrameworkConstructorFx)(kObject* object, kAlloc allocator);   

/**
 * @struct  kMethodInfo
 * @ingroup kApi  
 * @brief   Represents type method information. 
 */
typedef struct kMethodInfo
{
    kTypeName methodName;               ///< Method name (e.g. "Clone"). 
    kTypeName functionName;             ///< Full function name (e.g. "kObject_Clone"). 
    kFunction function;                 ///< Pointer to function.
} kMethodInfo; 

/**
 * @struct  kFieldInfo
 * @ingroup kApi  
 * @brief   Represents type field information. 
 */
typedef struct kFieldInfo
{
    kTypeName name;                     ///< Field name. 
    kType type;                         ///< Field type.
    kSize offset;                       ///< Offset of field within structure (bytes).
    kSize count;                        ///< Count of values in this field (typically 1; can be higher for "array value" fields, e.g. kText32). 
} kFieldInfo; 

/**
 * @struct  kEnumeratorInfo
 * @ingroup kApi  
 * @brief   Represents enumerator information. 
 */
typedef struct kEnumeratorInfo
{
    k32s value;                         ///< Enumerator numeric value.
    kTypeName name;                     ///< Enumerator name (e.g. "kPIXEL_FORMAT_8BPP_GREYSCALE"). 
    kTypeName displayName;              ///< Formatted display name (e.g. "8bpp Greyscale"); 
} kEnumeratorInfo; 

/** 
 * Returns the kType object associated with the specified class, interface, or value symbol. 
 * 
 * This macro is used to access type information by compile-time symbol name.  E.g. 
 * 
 * @code {.c}
 * 
 * #include <kApi/Data/kImage.h>
 * 
 * void PrintImageTypeInfo()
 * {
 *     kType type = kTypeOf(kImage); 
 *     
 *     printf("Type name: %s\n", kType_Name(type)); 
 *     printf("Base class: %s\n", kType_Name(kType_Base(type))); 
 * }
 * 
 * @endcode
 * 
 * Use of this macro requires that the header file defining the specified type symbol has 
 * been included.  For example, kTypeOf(kArrayList) requires inclusion of <kApi/Data/kArrayList.h>.
 * 
 * @param   SYMBOL     Type symbol, such as kArrayList or k32s.
 * @return             kType object representing metadata about the specified type.
 */
#define kTypeOf(SYMBOL)     \
    xkTypeOf(SYMBOL)

/** 
 * Returns the kAssembly object associated with the specified assembly symbol. 
 * 
 * This macro is used to access assembly information by compile-time symbol name.  E.g. 
 * 
 * @code {.c}
 * 
 * #include <kApi/kApiLib.h>
 * 
 * void PrintCoreAssemblyInfo()
 * {
 *     kAssembly assembly = kAssemblyOf(kApiLib); 
 *     
 *     printf("Assembly name: %s\n", kAssembly_Name(assembly)); 
 *     printf("Type count: %u\n", (k32u) kAssembly_TypeCount(assembly));  
 * }
 * 
 * @endcode
 * 
 * Use of this macro requires that the header file defining the specified assembly symbol 
 * has been included. For example, kAssemblyOf(kApiLib) requires inclusion of <kApi/kApiLib.h>
 * 
 * @param   SYMBOL     Assembly symbol, such as <em>kApiLib</em>.
 * @return             kAssembly object representing metadata about the specified assembly. 
 */
#define kAssemblyOf(SYMBOL)     \
    xkAssemblyOf(SYMBOL)

/** 
 * Returns static data associated with the specified class symbol.
 * 
 * This macro is used within class implementations to access static state. 
 * 
 * @param   SYMBOL     Class symbol, such as <em>kArrayList</em>.
 * @return             Pointer to static data for the specified class.  
 */
#define kStaticOf(SYMBOL)   \
    xkStaticOf(SYMBOL)

/**
 * Adds a deprecation warning for the specified symbol. 
 * 
 * At present, kDeprecate is only supported in MSVC. 
 * 
 * @param   SYMBOL      Identifer (e.g. <em>kData_Dispose</em>). 
 */
#define kDeprecate(SYMBOL) \
    xkDeprecate(#SYMBOL) 

/**
 * Emits a custom compile-time warning message.
 * 
 * Behavior may differ between compilers, depending on the level of support for emitting 
 * custom warnings.
 * 
 * @param   MESSAGE     Diagnostic message to be displayed (if supported by compiler). 
 */
#define kWarn(MESSAGE) \
    xkWarn(MESSAGE) 

/* 
* 
* Type Declaration/Definition Macros, Generation 1. 
* 
* These type declaration/definition macros continue to be supported. However, consider using the Generation 2
* macros (defined below) in new code.
* 
*/

/**
 * Declares a type assembly.
 * 
 * @param   PREFIX      Function/data declaration prefix (e.g. <em>k</em>). 
 * @param   SYMBOL      Assembly symbol name (e.g. <em>kApi</em>). 
 * @see                 @ref kApi-Extending, kBeginAssembly, kEndAssembly
 */
#define kDeclareAssembly(PREFIX, SYMBOL)        \
    xkDeclareAssembly(PREFIX, SYMBOL)       

/**
 * Starts the definition of a type assembly.
 * 
 * @param   PREFIX              Function/data declaration prefix (e.g. <em>k</em>). 
 * @param   SYMBOL              Assembly symbol name (e.g. <em>kApi</em>). 
 * @param   VERSION             Assembly version string (e.g. "1.0.0.0"). 
 * @param   PLATFORM_VERSION    Platform version string (e.g. "6.0.0.0"). 
 * @see                         @ref kApi-Extending, kEndAssembly
 */
#define kBeginAssembly(PREFIX, SYMBOL, VERSION, PLATFORM_VERSION)       \
    xkBeginAssembly(PREFIX, SYMBOL, VERSION, PLATFORM_VERSION)

/**
 * Ends the definition of a type assembly.
 * 
 * @see     @ref kApi-Extending
 */
#define kEndAssembly()           \
    xkEndAssembly()  

/**
 * Declares type information for a structure value type. 
 * 
 * @param   PREFIX      Function/data declaration prefix (e.g. <em>k</em>). 
 * @param   SYMBOL      Value symbol(e.g. <em>k32s</em>). 
 * @param   BASE        Value base symbol (typically <em>kValue</em>). 
 * @see                 @ref kApi-Extending, kBeginValue, kEndValue
 */
#define kDeclareValue(PREFIX, SYMBOL, BASE)         \
    xkDeclareValue(PREFIX, SYMBOL, BASE) 

/**
 * Starts the definition of a structure value type.
 * 
 * @param   PREFIX      Function/data declaration prefix (e.g. <em>k</em>). 
 * @param   SYMBOL      Value symbol(e.g. <em>k32s</em>). 
 * @param   BASE        Value base symbol (typically <em>kValue</em>). 
 * @see                 @ref kApi-Extending, kEndValue
 */
#define kBeginValue(PREFIX, SYMBOL, BASE)       \
    xkBeginValue(PREFIX, SYMBOL, BASE)

/**
 * Ends the definition of a structure value type.
 * 
 * @see     @ref kApi-Extending
 */
#define kEndValue()         \
    xkEndValue() 

/**
 * Declares type information for an enumeration value type. 
 * 
 * @param   PREFIX      Function/data declaration prefix (e.g. <em>k</em>). 
 * @param   SYMBOL      Value symbol(e.g. <em>kStatus</em>). 
 * @param   BASE        Value base symbol (typically <em>kValue</em>). 
 * @see                 @ref kApi-Extending, kBeginEnum, kEndEnum
 */
#define kDeclareEnum(PREFIX, SYMBOL, BASE)      \
    xkDeclareEnum(PREFIX, SYMBOL, BASE) 

/**
 * Starts the definition of an enumeration value type.
 * 
 * @param   PREFIX      Function/data declaration prefix (e.g. <em>k</em>). 
 * @param   SYMBOL      Value symbol(e.g. <em>kStatus</em>). 
 * @param   BASE        Value base symbol (typically <em>kValue</em>). 
 * @see                 @ref kApi-Extending, kEndEnum
 */
#define kBeginEnum(PREFIX, SYMBOL, BASE)         \
    xkBeginEnum(PREFIX, SYMBOL, BASE) 

/**
 * Ends the definition of an enumeration value type.
 * 
 * @see     @ref kApi-Extending
 */
#define kEndEnum()           \
    xkEndEnum()

/**
 * Declares type information for an array-based value type. 
 * 
 * @param   PREFIX      Function/data declaration prefix (e.g. <em>k</em>). 
 * @param   SYMBOL      Value symbol(e.g. <em>kText32</em>). 
 * @param   BASE        Value base symbol (typically <em>kValue</em>). 
 * @see                 @ref kApi-Extending, kBeginArrayValue, kEndArrayValue
 */
#define kDeclareArrayValue(PREFIX, SYMBOL, BASE)        \
    xkDeclareArrayValue(PREFIX, SYMBOL, BASE) 

/**
 * Starts the definition of an array-based value type.
 * 
 * @param   PREFIX      Function/data declaration prefix (e.g. <em>k</em>). 
 * @param   SYMBOL      Value symbol(e.g. <em>kText32</em>). 
 * @param   TYPE        Array element type symbol(e.g. <em>kChar</em>). 
 * @param   BASE        Value base symbol (typically <em>kValue</em>). 
 * @see                 @ref kApi-Extending, kEndArrayValue
 */
#define kBeginArrayValue(PREFIX, SYMBOL, TYPE, BASE)        \
    xkBeginArrayValue(PREFIX, SYMBOL, TYPE, BASE) 

/**
 * Ends the definition of an array-based value type.
 * 
 * @see     @ref kApi-Extending
 */
#define kEndArrayValue()        \
    xkEndArrayValue()

/**
 * Declares type information for an interface type.
 * 
 * @param   PREFIX      Function/data declaration prefix (e.g. <em>k</em>). 
 * @param   SYMBOL      Interface symbol(e.g. <em>kCollection</em>). 
 * @param   BASE        Interface base symbol (typically <em>kNull</em>). 
 * @see                 @ref kApi-Extending, kBeginInterface, kEndInterface
 */
#define kDeclareInterface(PREFIX, SYMBOL, BASE)         \
    xkDeclareInterface(PREFIX, SYMBOL, BASE) 

/**
 * Starts the definition of an interface type.
 * 
 * @param   PREFIX      Function/data declaration prefix (e.g. <em>k</em>). 
 * @param   SYMBOL      Interface symbol(e.g. <em>kCollection</em>). 
 * @param   BASE        Interface base symbol (typically <em>kNull</em>). 
 * @see                 @ref kApi-Extending, kEndInterface
 */
#define kBeginInterface(PREFIX, SYMBOL, BASE)           \
    xkBeginInterface(PREFIX, SYMBOL, BASE) 

/**
 * Ends the definition of an interface type.
 * 
 * @see     @ref kApi-Extending
 */
#define kEndInterface()         \
    xkEndInterface()  

/**
 * Declares type information for a class type. 
 * 
 * @param   PREFIX      Function/data declaration prefix (e.g. <em>k</em>). 
 * @param   SYMBOL      Class symbol(e.g. <em>kFile</em>). 
 * @param   BASE        Class base symbol (e.g. <em>kObject</em>). 
 * @see                 @ref kApi-Extending, kBeginFullClass, kEndFullClass
 */
#define kDeclareFullClass(PREFIX, SYMBOL, BASE)         \
    xkDeclareFullClass(PREFIX, SYMBOL, BASE)  

/**
 * Starts the definition of a class type.
 * 
 * @param   PREFIX      Function/data declaration prefix (e.g. <em>k</em>). 
 * @param   SYMBOL      Class symbol(e.g. <em>kFile</em>). 
 * @param   BASE        Class base symbol (e.g. <em>kObject</em>). 
 * @see                 @ref kApi-Extending, kEndFullClass
 */
#define kBeginFullClass(PREFIX, SYMBOL, BASE)       \
    xkBeginFullClass(PREFIX, SYMBOL, BASE) 

/**
 * Ends the definition of a class type.
 * 
 * @see     @ref kApi-Extending
 */
#define kEndFullClass()         \
    xkEndFullClass()

/**
 * Declares type information for a class type that requires an expanded vtable but does not have static data. 
 * 
 * @param   PREFIX      Function/data declaration prefix (e.g. <em>k</em>). 
 * @param   SYMBOL      Class symbol(e.g. <em>kAlloc</em>). 
 * @param   BASE        Class base symbol (e.g. <em>kObject</em>). 
 * @see                 @ref kApi-Extending, kBeginVirtualClass, kEndVirtualClass
 */
#define kDeclareVirtualClass(PREFIX, SYMBOL, BASE)          \
    xkDeclareVirtualClass(PREFIX, SYMBOL, BASE)

/**
 * Starts the definition of a class type that requires an expanded vtable but does not have static data. 
 * 
 * @param   PREFIX      Function/data declaration prefix (e.g. <em>k</em>). 
 * @param   SYMBOL      Class symbol(e.g. <em>kAlloc</em>). 
 * @param   BASE        Class base symbol (e.g. <em>kObject</em>). 
 * @see                 @ref kApi-Extending, kEndVirtualClass
 */
#define kBeginVirtualClass(PREFIX, SYMBOL, BASE)        \
    xkBeginVirtualClass(PREFIX, SYMBOL, BASE) 

/**
 * Ends the definition of a class type that requires an expanded vtable but does not have static data. 
 * 
 * @see     @ref kApi-Extending
 */
#define kEndVirtualClass()          \
    xkEndVirtualClass()   

/**
 * Declares type information for a class type that has only static data. 
 * 
 * @param   PREFIX      Function/data declaration prefix (e.g. <em>k</em>). 
 * @param   SYMBOL      Class symbol(e.g. <em>kNetwork</em>). 
 * @see                 @ref kApi-Extending, kBeginStaticClass, kEndStaticClass
 */
#define kDeclareStaticClass(PREFIX, SYMBOL)         \
    xkDeclareStaticClass(PREFIX, SYMBOL) 

/**
 * Starts the definition of a class type that has only static data. 
 * 
 * @param   PREFIX      Function/data declaration prefix (e.g. <em>k</em>). 
 * @param   SYMBOL      Class symbol(e.g. <em>kNetwork</em>). 
 * @see                 @ref kApi-Extending, kEndFullClass
 */
#define kBeginStaticClass(PREFIX, SYMBOL)       \
    xkBeginStaticClass(PREFIX, SYMBOL)  

/**
 * Ends the definition of a class type that has only static data. 
 * 
 * @see     @ref kApi-Extending
 */
#define kEndStaticClass()           \
    xkEndStaticClass()

/**
 * Declares type information for a class type that does not require an expanded vtable and does not have static data. 
 * 
 * @param   PREFIX      Function/data declaration prefix (e.g. <em>k</em>). 
 * @param   SYMBOL      Class symbol(e.g. <em>kArrayList</em>). 
 * @param   BASE        Class base symbol (e.g. <em>kObject</em>). 
 * @see                 @ref kApi-Extending, kBeginClass, kEndClass
 */
#define kDeclareClass(PREFIX, SYMBOL, BASE)         \
    xkDeclareClass(PREFIX, SYMBOL, BASE)  

/**
 * Starts the definition of a class type that does not require an expanded vtable and does not have static data. 
 * 
 * @param   PREFIX      Function/data declaration prefix (e.g. <em>k</em>). 
 * @param   SYMBOL      Class symbol(e.g. <em>kArrayList</em>). 
 * @param   BASE        Class base symbol (e.g. <em>kObject</em>). 
 * @see                 @ref kApi-Extending, kEndClass
 */
#define kBeginClass(PREFIX, SYMBOL, BASE)       \
    xkBeginClass(PREFIX, SYMBOL, BASE) 

/**
 * Ends the definition of a class type that does not require an expanded vtable and does not have static data. 
 * 
 * @see     @ref kApi-Extending
 */
#define kEndClass()         \
    xkEndClass()

/* 
* 
* Type Declaration/Definition Macros, Generation 2. 
* 
* These macros are used to declare and define elements of the type system, such 
* as assemblies, classes and values. These macros supersede the Generation 1 macros, 
* defined above. 
* 
*/

/**
 * Declares a type assembly.
 * 
 * @param   PREFIX      Function/data declaration prefix (e.g. <em>k</em>). 
 * @param   SYMBOL      Assembly symbol name (e.g. <em>kApi</em>). 
 * @see                 @ref kApi-Extending, kBeginAssemblyEx, kEndAssemblyEx
 */
#define kDeclareAssemblyEx(PREFIX, SYMBOL) \
    xkDeclareAssemblyEx(PREFIX, SYMBOL)       

/**
 * Starts the definition of a type assembly.
 * 
 * @param   PREFIX              Function/data declaration prefix (e.g. <em>k</em>). 
 * @param   SYMBOL              Assembly symbol name (e.g. <em>kApi</em>). 
 * @param   VERSION             Assembly version string (e.g. "1.0.0.0"). 
 * @param   PLATFORM_VERSION    Platform version string (e.g. "6.0.0.0"). 
 * @see                         @ref kApi-Extending, kEndAssemblyEx
 */
#define kBeginAssemblyEx(PREFIX, SYMBOL, VERSION, PLATFORM_VERSION) \
    xkBeginAssemblyEx(PREFIX, SYMBOL, VERSION, PLATFORM_VERSION)

/**
 * Ends the definition of a type assembly.
 * 
 * @see     @ref kApi-Extending
 */
#define kEndAssemblyEx() \
    xkEndAssemblyEx()  

/**
 * Declares type information for a structure value type. 
 * 
 * @param   PREFIX      Function/data declaration prefix (e.g. <em>k</em>). 
 * @param   SYMBOL      Value symbol(e.g. <em>k32s</em>). 
 * @param   BASE        Value base symbol (typically <em>kValue</em>). 
 * @see                 @ref kApi-Extending, kBeginValueEx, kEndValueEx
 */
#define kDeclareValueEx(PREFIX, SYMBOL, BASE) \
    xkDeclareValueEx(PREFIX, SYMBOL, BASE) 

/**
 * Starts the definition of a structure value type.
 * 
 * @param   PREFIX      Function/data declaration prefix (e.g. <em>k</em>). 
 * @param   SYMBOL      Value symbol(e.g. <em>k32s</em>). 
 * @see                 @ref kApi-Extending, kEndValueEx
 */
#define kBeginValueEx(PREFIX, SYMBOL) \
    xkBeginValueEx(PREFIX, SYMBOL)

/**
 * Ends the definition of a structure value type.
 * 
 * @see     @ref kApi-Extending
 */
#define kEndValueEx() \
    xkEndValueEx() 

/**
 * Declares type information for an enumeration value type. 
 * 
 * @param   PREFIX      Function/data declaration prefix (e.g. <em>k</em>). 
 * @param   SYMBOL      Value symbol(e.g. <em>kStatus</em>). 
 * @param   BASE        Value base symbol (typically <em>kValue</em>). 
 * @see                 @ref kApi-Extending, kBeginEnumEx, kEndEnumEx
 */
#define kDeclareEnumEx(PREFIX, SYMBOL, BASE) \
    xkDeclareEnumEx(PREFIX, SYMBOL, BASE) 

/**
 * Starts the definition of an enumeration value type.
 * 
 * @param   PREFIX      Function/data declaration prefix (e.g. <em>k</em>). 
 * @param   SYMBOL      Value symbol(e.g. <em>kStatus</em>). 
 * @see                 @ref kApi-Extending, kEndEnumEx
 */
#define kBeginEnumEx(PREFIX, SYMBOL) \
    xkBeginEnumEx(PREFIX, SYMBOL) 

/**
 * Ends the definition of an enumeration value type.
 * 
 * @see     @ref kApi-Extending
 */
#define kEndEnumEx() \
    xkEndEnumEx()

/**
 * Declares type information for an array-based value type. 
 * 
 * @param   PREFIX      Function/data declaration prefix (e.g. <em>k</em>). 
 * @param   SYMBOL      Value symbol(e.g. <em>kText32</em>). 
 * @param   BASE        Value base symbol (typically <em>kValue</em>). 
 * @see                 @ref kApi-Extending, kBeginArrayValueEx, kEndArrayValueEx
 */
#define kDeclareArrayValueEx(PREFIX, SYMBOL, BASE) \
    xkDeclareArrayValueEx(PREFIX, SYMBOL, BASE) 

/**
 * Starts the definition of an array-based value type.
 * 
 * @param   PREFIX      Function/data declaration prefix (e.g. <em>k</em>). 
 * @param   SYMBOL      Value symbol(e.g. <em>kText32</em>). 
 * @param   TYPE        Array element type symbol(e.g. <em>kChar</em>). 
 * @see                 @ref kApi-Extending, kEndArrayValueEx
 */
#define kBeginArrayValueEx(PREFIX, SYMBOL, TYPE) \
    xkBeginArrayValueEx(PREFIX, SYMBOL, TYPE) 

/**
 * Ends the definition of an array-based value type.
 * 
 * @see     @ref kApi-Extending
 */
#define kEndArrayValueEx() \
    xkEndArrayValueEx()

/**
 * Declares type information for an interface type.
 * 
 * @param   PREFIX      Function/data declaration prefix (e.g. <em>k</em>). 
 * @param   SYMBOL      Interface symbol(e.g. <em>kCollection</em>). 
 * @param   BASE        Interface base symbol (typically <em>kNull</em>). 
 * @see                 @ref kApi-Extending, kBeginInterfaceEx, kEndInterfaceEx
 */
#define kDeclareInterfaceEx(PREFIX, SYMBOL, BASE) \
    xkDeclareInterfaceEx(PREFIX, SYMBOL, BASE) 

/**
 * Starts the definition of an interface type.
 * 
 * @param   PREFIX      Function/data declaration prefix (e.g. <em>k</em>). 
 * @param   SYMBOL      Interface symbol(e.g. <em>kCollection</em>). 
 * @see                 @ref kApi-Extending, kEndInterfaceEx
 */
#define kBeginInterfaceEx(PREFIX, SYMBOL) \
    xkBeginInterfaceEx(PREFIX, SYMBOL) 

/**
 * Ends the definition of an interface type.
 * 
 * @see     @ref kApi-Extending
 */
#define kEndInterfaceEx() \
    xkEndInterfaceEx()  

/**
 * Declares type information for a class type. 
 * 
 * @param   PREFIX      Function/data declaration prefix (e.g. <em>k</em>). 
 * @param   SYMBOL      Class symbol(e.g. <em>kFile</em>). 
 * @param   BASE        Class base symbol (e.g. <em>kObject</em>). 
 * @see                 @ref kApi-Extending, kBeginFullClassEx, kEndFullClassEx
 */
#define kDeclareFullClassEx(PREFIX, SYMBOL, BASE) \
    xkDeclareFullClassEx(PREFIX, SYMBOL, BASE)  

/**
 * Starts the definition of a class type.
 * 
 * @param   PREFIX      Function/data declaration prefix (e.g. <em>k</em>). 
 * @param   SYMBOL      Class symbol(e.g. <em>kFile</em>). 
 * @see                 @ref kApi-Extending, kEndFullClassEx
 */
#define kBeginFullClassEx(PREFIX, SYMBOL) \
    xkBeginFullClassEx(PREFIX, SYMBOL) 

/**
 * Ends the definition of a class type.
 * 
 * @see     @ref kApi-Extending
 */
#define kEndFullClassEx() \
    xkEndFullClassEx()

/**
 * Declares type information for a class type that requires an expanded vtable but does not have static data. 
 * 
 * @param   PREFIX      Function/data declaration prefix (e.g. <em>k</em>). 
 * @param   SYMBOL      Class symbol(e.g. <em>kAlloc</em>). 
 * @param   BASE        Class base symbol (e.g. <em>kObject</em>). 
 * @see                 @ref kApi-Extending, kBeginVirtualClassEx, kEndVirtualClassEx
 */
#define kDeclareVirtualClassEx(PREFIX, SYMBOL, BASE) \
    xkDeclareVirtualClassEx(PREFIX, SYMBOL, BASE)

/**
 * Starts the definition of a class type that requires an expanded vtable but does not have static data. 
 * 
 * @param   PREFIX      Function/data declaration prefix (e.g. <em>k</em>). 
 * @param   SYMBOL      Class symbol(e.g. <em>kAlloc</em>). 
 * @see                 @ref kApi-Extending, kEndVirtualClassEx
 */
#define kBeginVirtualClassEx(PREFIX, SYMBOL) \
    xkBeginVirtualClassEx(PREFIX, SYMBOL) 

/**
 * Ends the definition of a class type that requires an expanded vtable but does not have static data. 
 * 
 * @see     @ref kApi-Extending
 */
#define kEndVirtualClassEx() \
    xkEndVirtualClassEx()   

/**
 * Declares type information for a class type that has only static data. 
 * 
 * @param   PREFIX      Function/data declaration prefix (e.g. <em>k</em>). 
 * @param   SYMBOL      Class symbol(e.g. <em>kNetwork</em>). 
 * @see                 @ref kApi-Extending, kBeginStaticClassEx, kEndStaticClassEx
 */
#define kDeclareStaticClassEx(PREFIX, SYMBOL) \
    xkDeclareStaticClassEx(PREFIX, SYMBOL) 

/**
 * Starts the definition of a class type that has only static data. 
 * 
 * @param   PREFIX      Function/data declaration prefix (e.g. <em>k</em>). 
 * @param   SYMBOL      Class symbol(e.g. <em>kNetwork</em>). 
 * @see                 @ref kApi-Extending, kEndFullClassEx
 */
#define kBeginStaticClassEx(PREFIX, SYMBOL) \
    xkBeginStaticClassEx(PREFIX, SYMBOL)  

/**
 * Ends the definition of a class type that has only static data. 
 * 
 * @see     @ref kApi-Extending
 */
#define kEndStaticClassEx() \
    xkEndStaticClassEx()

/**
 * Declares type information for a class type that does not require an expanded vtable and does not have static data. 
 * 
 * @param   PREFIX      Function/data declaration prefix (e.g. <em>k</em>). 
 * @param   SYMBOL      Class symbol(e.g. <em>kArrayList</em>). 
 * @param   BASE        Class base symbol (e.g. <em>kObject</em>). 
 * @see                 @ref kApi-Extending, kBeginClassEx, kEndClassEx
 */
#define kDeclareClassEx(PREFIX, SYMBOL, BASE) \
    xkDeclareClassEx(PREFIX, SYMBOL, BASE)  

/**
 * Starts the definition of a class type that does not require an expanded vtable and does not have static data. 
 * 
 * @param   PREFIX      Function/data declaration prefix (e.g. <em>k</em>). 
 * @param   SYMBOL      Class symbol(e.g. <em>kArrayList</em>). 
 * @see                 @ref kApi-Extending, kEndClassEx
 */
#define kBeginClassEx(PREFIX, SYMBOL) \
    xkBeginClassEx(PREFIX, SYMBOL) 

/**
 * Ends the definition of a class type that does not require an expanded vtable and does not have static data. 
 * 
 * @see     @ref kApi-Extending
 */
#define kEndClassEx() \
    xkEndClassEx()

/* 
* 
* Type Implementation Macros
* 
* These macros are used to provide type-specific details.  They are typically used between 
* type definition begin/end macros (e.g., kBeginClass/kEndClass). 
* 
*/

/**
 * Within an assembly definition, specifies a dependency on another assembly. 
 * 
 * @param   SYMBOL      Dependency target (e.g. <em>kApi</em>). 
 * @see                 @ref kApi-Extending
 */
#define kAddDependency(SYMBOL) \
    xkAddDependency(SYMBOL)    

/**
 * Within an assembly definition, adds a type to the assembly.
 * 
 * @param   SYMBOL      Type symbol (e.g. <em>kArrayList</em>). 
 * @see                 @ref kApi-Extending
 */
#define kAddType(SYMBOL) \
    xkAddType(SYMBOL)   

/**
 * Within an assembly definition, indicates a requirement on static initialization order. 
 * 
 * The order of kAddPriority statements within an assembly definition determines the order
 * in which types will be initialized. If initialization priorities are not given, types
 * can be initialized in any order. 
 * 
 * @param   SYMBOL      Type symbol (e.g. <em>kArrayList</em>). 
 * @see                 @ref kApi-Extending
 */
#define kAddPriority(SYMBOL) \
    xkAddPriority(SYMBOL)

/**
 * Within a type definition, indicates that a type has static data. 
 * 
 * Types that have static data must define initialization and release methods.
 * 
 * @param   SYMBOL      Type symbol (e.g. <em>kArrayList</em>). 
 * @see                 @ref kApi-Extending
 */
#define kAddStatic(SYMBOL) \
    xkAddStatic(SYMBOL)  

/**
 * Within a type definition, indicates that a type implements the specified interface.
 * 
 * @param   SYMBOL      Type symbol (e.g. <em>kArrayList</em>). 
 * @param   IFACE       Interface symbol (e.g. <em>kCollection</em>). 
 * @see                 @ref kApi-Extending
 */
#define kAddInterface(SYMBOL, IFACE) \
    xkAddInterface(SYMBOL, IFACE) 

/**
 * Within a type definition, indicates that a type has the specified framework constructor.
 * 
 * Framework constructors are used to construct objects via reflection, and are not inherited.
 * Their scope is limited to specific frameworks (deserialization and cloning) that guarantee 
 * constrained usage patterns. Specifically, after default construction, a framework-specific method 
 * will always be invoked to complete object initialization (e.g., deserialization method). If
 * this framework-specific method should fail, kObject_Dispose will be called on the partially 
 * constructed object to clean up. As such, the only requirement when implementing a default 
 * constructor is to sufficiently initialize the object such that it could accept a subsequent
 * framework initialization method, or be disposed.
 * 
 * Framework constructors should be defined with a signature compatible with @ref kFrameworkConstructorFx.  
 * 
 * @param   SYMBOL      Type symbol (e.g. <em>kArrayList</em>). 
 * @param   CTOR        Constructor name (e.g. <em>ConstructFramework</em>). 
 * @see                 @ref kApi-Extending
 */
#define kAddFrameworkConstructor(SYMBOL, CTOR) \
    xkAddFrameworkConstructor(SYMBOL, CTOR)  

/**
 * Within a type definition, indicates that a type has the specified default private constructor.
 * 
 * Framework constructors are used to construct objects via reflection, and are not inherited.
 * Their scope is limited to specific frameworks (deserialization and cloning) that guarantee 
 * constrained usage patterns. Specifically, after default construction, a framework-specific method 
 * will always be invoked to complete object initialization (e.g., deserialization method). If
 * this framework-specific method should fail, kObject_Dispose will be called on the partially 
 * constructed object to clean up. As such, the only requirement when implementing a default 
 * constructor is to sufficiently initialize the object such that it could accept a subsequent
 * framework initialization method, or be disposed.
 *  
 * The private version of this macro assumes that the full function name will be prefixed with an "x". 
 * 
 * @param   SYMBOL      Type symbol (e.g. <em>kArrayList</em>). 
 * @param   CTOR        Constructor name (e.g. <em>ConstructFramework</em>). 
 * @see                 @ref kApi-Extending
 */
#define kAddPrivateFrameworkConstructor(SYMBOL, CTOR) \
    xkAddPrivateFrameworkConstructor(SYMBOL, CTOR)  

/**
 * Within a type definition, indicates that a type has the specified non-virtual method. 
 * 
 * @param   SYMBOL      Type symbol (e.g. <em>kArrayList</em>). 
 * @param   METHOD      Method name (e.g. <em>Remove</em>). 
 * @see                 @ref kApi-Extending
 */
#define kAddMethod(SYMBOL, METHOD) \
    xkAddMethod(SYMBOL, METHOD)  

/**
 * Within a type definition, indicates that a type overrides the specified virtual method. 
 * 
 * @param   IN_TYPE     Overriding type (e.g. <em>kArrayList</em>). 
 * @param   FROM_TYPE   Overridden type (e.g. <em>kObject</em>). 
 * @param   METHOD      Method name (e.g. <em>VRelease</em>). 
 * @see                 @ref kApi-Extending
 */
#define kAddVMethod(IN_TYPE, FROM_TYPE, METHOD) \
    xkAddVMethod(IN_TYPE, FROM_TYPE, METHOD)   

/**
 * Within a type definition, indicates that a type overrides the specified virtual method with a private method.
 * 
 * Assumes that the method name will be prefixed with an "x". 
 * 
 * @param   IN_TYPE     Overriding type (e.g. <em>kArrayList</em>). 
 * @param   FROM_TYPE   Overridden type (e.g. <em>kObject</em>). 
 * @param   METHOD      Method name (e.g. <em>VRelease</em>). 
 * @see                 @ref kApi-Extending
 */
#define kAddPrivateVMethod(IN_TYPE, FROM_TYPE, METHOD) \
    xkAddPrivateVMethod(IN_TYPE, FROM_TYPE, METHOD)   

/**
 * Within a type definition, indicates that a type implements the specified interface method. 
 * 
 * @param   IN_TYPE     Overriding type (e.g. <em>kArrayList</em>). 
 * @param   FROM_IFACE  Overridden interface (e.g. <em>kCollection</em>). 
 * @param   IMETHOD     Interface method name (e.g. <em>VGetIterator</em>). 
 * @param   CMETHOD     Type method name (e.g. <em>GetIterator</em>). 
 * @see                 @ref kApi-Extending
 */
#define kAddIVMethod(IN_TYPE, FROM_IFACE, IMETHOD, CMETHOD) \
    xkAddIVMethod(IN_TYPE, FROM_IFACE, IMETHOD, CMETHOD)

/**
 * Within a type definition, indicates that a type implements the specified interface method with a private method. 
 * 
 * Assumes that the method name will be prefixed with an "x". 
 * 
 * @param   IN_TYPE     Overriding type (e.g. <em>kArrayList</em>). 
 * @param   FROM_IFACE  Overridden interface (e.g. <em>kCollection</em>). 
 * @param   IMETHOD     Interface method name (e.g. <em>VGetIterator</em>). 
 * @param   CMETHOD     Type method name (e.g. <em>GetIterator</em>). 
 * @see                 @ref kApi-Extending
 */
#define kAddPrivateIVMethod(IN_TYPE, FROM_IFACE, IMETHOD, CMETHOD) \
    xkAddPrivateIVMethod(IN_TYPE, FROM_IFACE, IMETHOD, CMETHOD)

/**
 * Within a structure type definition, indicates that a structure has the specified field. 
 * 
 * @param   VALUE       Structure type (e.g. <em>kPoint32s</em>). 
 * @param   FIELD_TYPE  Field type (e.g. <em>k32s</em>). 
 * @param   FIELD       Field name (e.g. <em>x</em>). 
 * @see                 @ref kApi-Extending
 */
#define kAddField(VALUE, FIELD_TYPE, FIELD) \
    xkAddField(VALUE, FIELD_TYPE, FIELD)

/**
 * Within an enumeration type definition, indicates that an enumeration has the specified enumerator. 
 * 
 * @param   VALUE       Enumeration type (e.g. <em>kStatus</em>). 
 * @param   ENUMERATOR  Enumerator(e.g. <em>kERROR</em>). 
 * @see                 @ref kApi-Extending
 */
#define kAddEnumerator \
    xkAddEnumerator

/**
 * Within a type definition, indicates that a type has the specified serialization version. 
 * 
 * This macro should only be used with type versions that implement modern deserialization methods. 
 * Refer to @ref kType_VersionDeserializeFx for more information on the differences between legacy 
 * and modern deserialization methods. 
 * 
 * Note, the addition of a serialization version requires the addition of a framework constructor. 
 * Refer to @ref kAddFrameworkConstructor.
 * 
 * @param   TYPE            Type symbol (e.g. <em>kArrayList</em>). 
 * @param   FORMAT          Serialization format name string (e.g. "kdat6"). 
 * @param   FORMAT_VER      Serialization format version string (e.g. "5.7.1.0"). 
 * @param   GUID            Type identifier string within serialization format (e.g. "kArrayList-0"). 
 * @param   WRITE_METHOD    Serialization write method (e.g. <em>WriteDat6V0</em>). 
 * @param   READ_METHOD     Serialization read method (e.g. <em>ReadDat6V0</em>). 
 * @see                     @ref kApi-Extending, kSerializer
 */
#define kAddVersionEx(TYPE, FORMAT, FORMAT_VER, GUID, WRITE_METHOD, READ_METHOD) \
    xkAddVersionEx(TYPE, FORMAT, FORMAT_VER, GUID, WRITE_METHOD, READ_METHOD)   

/**
 * Within a type definition, indicates that a type has the specified serialization version. 
 * 
 * Assumes that serialization method names will be prefixed with an "x". 
 * 
 * This macro should only be used with type versions that implement modern deserialization methods. 
 * Refer to @ref kType_VersionDeserializeFx for more information on the differences between legacy 
 * and modern deserialization methods. 
 * 
 * Note, the addition of a serialization version requires the addition of a framework constructor. 
 * Refer to @ref kAddFrameworkConstructor.
 * 
 * @param   TYPE            Type symbol (e.g. <em>kArrayList</em>). 
 * @param   FORMAT          Serialization format name string (e.g. "kdat6"). 
 * @param   FORMAT_VER      Serialization format version string (e.g. "5.7.1.0"). 
 * @param   GUID            Type identifier string within serialization format (e.g. "kArrayList-0"). 
 * @param   WRITE_METHOD    Serialization write method (e.g. <em>WriteDat6V0</em>). 
 * @param   READ_METHOD     Serialization read method (e.g. <em>ReadDat6V0</em>). 
 * @see                     @ref kApi-Extending, kSerializer
 */
#define kAddPrivateVersionEx(TYPE, FORMAT, FORMAT_VER, GUID, WRITE_METHOD, READ_METHOD) \
    xkAddPrivateVersionEx(TYPE, FORMAT, FORMAT_VER, GUID, WRITE_METHOD, READ_METHOD)   

/**
 * Within a type definition, indicates that type information can be serialized, but instances cannot.
 * 
 * This approach is intended to support abstract base classes, in cases where descendents must 
 * support serialization. This enables collections (containers) in which the collection element type 
 * is defined as the type of an abstract base to be correctly serialized. 
 * 
 * @param   TYPE            Type symbol (e.g. <em>kArrayList</em>). 
 * @param   FORMAT          Serialization format name string (e.g. "kdat6"). 
 * @param   FORMAT_VER      Serialization format version string (e.g. "5.7.1.0"). 
 * @param   GUID            Type identifier string within serialization format (e.g. "kArrayList-0"). 
 * @see                     @ref kApi-Extending, kSerializer
 */
#define kAddAbstractVersionEx(TYPE, FORMAT, FORMAT_VER, GUID) \
    xkAddAbstractVersion(TYPE, FORMAT, FORMAT_VER, GUID)   

/**
 * Adds specific flags to type metadata. 
 * 
 * @param   TYPE        Type (e.g. <em>kObject</em>). 
 * @param   FLAGS       Type flags (e.g. <em>kTYPE_FLAGS_ABSTRACT</em>). 
 * @see                 @ref kApi-Extending
 */
#define kAddFlags(TYPE, FLAGS) \
    xkAddFlags(TYPE, FLAGS)     

/**
 * The kDefineDebugHints macro can be helpful when attempting to use debug expressions to peek inside
 * kApi class implementations in Visual Studio. The underlying problem that this macro addresses
 * is that if a structure definition isn't <em>directly</em> used by the particular application or library
 * that is being debugged, then the debugger won't be able to locate it. E.g., if the debug expression 
 * "(kXmlClass*)myXmlObject" is used, but the kXmlClass structure isn't directly used by the library 
 * that is being debugged (instead, only the kXml handle definition is used), then the debug expression 
 * can't be evaluated.
 *
 * To make use of this feature, select <em>one</em> file in your library or program and include the 
 * following lines:
 *
 * @code {.c}
 *
 * #include <kApi/kApi.h>
 *
 * kDefineDebugHints()
 * 
 * @endcode {.c}
 *
 */
#define kDefineDebugHints() \
    xkDefineDebugHints()

/* 
* 
* Method Implementation Macros
* 
* These macros are used in the implementation of class methods.
* 
*/

/**
 * Declares a local "obj" (this-pointer) variable and initializes it from a type-checked object handle.
 * 
 * @param   TypeName_T    Class type name (e.g. <em>kObject</em>). 
 * @param   T_object      Class instance  (e.g. <em>object</em>). 
 */
#define kObj(TypeName_T, T_object) \
    TypeName_T##Class* obj K_ATTRIBUTE_UNUSED = xx##TypeName_T##_Cast(T_object, __FILE__, __LINE__)

/**
 * Declares a local "obj" (this-pointer) variable and initializes it from an object handle, without type-checking.
 * 
 * The "R" suffix refers to "raw" (unchecked) access. This macro should be used in (and only in) class initializer methods, 
 * where type information has not yet been established for the object. 
 * 
 * @param   TypeName_T    Class type name (e.g. <em>kObject</em>). 
 * @param   T_object      Class instance  (e.g. <em>object</em>). 
 */
#define kObjR(TypeName_T, T_object) \
    TypeName_T##Class* obj K_ATTRIBUTE_UNUSED = x##TypeName_T##_CastRaw(T_object)

/**
 * Declares a local this-pointer variable with a specified name and initializes it from a type-checked object handle.
 *
 * The "N" suffix refers to providing a "named" this-variable. This macro should be used when accessing the instance 
 * fields of an object other than the current object (i.e., other instances of the same class, for example in assignment/copy
 * methods, or instances of friend classes). 
 * 
 * @param   TypeName_T    Class type name (e.g. <em>kObject</em>). 
 * @param   VarName_obj   This variable name (e.g. <em>obj</em>). 
 * @param   T_object      Class instance  (e.g. <em>object</em>). 
 */
#define kObjN(TypeName_T, VarName_obj, T_object) \
    TypeName_T##Class* VarName_obj = xx##TypeName_T##_Cast(T_object, __FILE__, __LINE__)
   
/**
 * Declares a local this-pointer variable with a specified name and initializes it from an object handle, without type-checking.
 * 
 * The "N" and "R" suffixes refer to providing a "named" this-variable with "raw" (unchecked) access, respectively. This macro 
 * is provided to simplify automated porting of preexisting code and is not recommended for future use.
 * 
 * @param   TypeName_T    Class type name (e.g. <em>kObject</em>). 
 * @param   VarName_obj   This variable name (e.g. <em>obj</em>). 
 * @param   T_object      Class instance  (e.g. <em>object</em>). 
 */
#define kObjNR(TypeName_T, VarName_obj, T_object) \
    TypeName_T##Class* VarName_obj = x##TypeName_T##_CastRaw(T_object)

/**
 * Declares a local "sobj" (static object) pointer variable and initializes it.
 * 
 * @param   TypeName_T    Class type name (e.g. <em>kObject</em>). 
 */
#define kStaticObj(TypeName_T) \
    TypeName_T##Static* sobj = kStaticOf(TypeName_T)

/**
 * Type-checks the specified object before returning a class-typed pointer to its implementation. 
 * 
 * Warning: This macro evaluates its arguments more than once. 
 *
 * @param   TYPE        Type (e.g. <em>kObject</em>). 
 * @param   OBJECT      Object instance.  
 * @return              Pointer to object fields.   
 * @see                 @ref kApi-Extending
 */
#define kCastClass_(TYPE, OBJECT) \
    xkCastClass_(TYPE, OBJECT)

/**
 * Gets a virtual table pointer for the specified object. 
 * 
 * Warning: This macro evaluates its arguments more than once. 
 *
 * @param   TYPE        Type (e.g. <em>kObject</em>). 
 * @param   OBJECT      Object instance.  
 * @return              Pointer to virtual table. 
 * @see                 @ref kApi-Extending
 */
#define kCastVTable_(TYPE, OBJECT) \
    xkCastVTable_(TYPE, OBJECT)

/**
 * Gets an interface virtual table pointer for the specified object. 
 * 
 * Warning: This macro evaluates its arguments more than once. 
 *
 * @param   IFACE       Interface type (e.g. <em>kObjectPool</em>). 
 * @param   OBJECT      Object instance.  
 * @return              Pointer to virtual table. 
 * @see                 @ref kApi-Extending
 */
#define kCastIVTable_(IFACE, OBJECT) \
    xkCastIVTable_(IFACE, OBJECT)

/**
 * Triggers a software breakpoint. 
 * 
 * Currently supported only in Visual Studio.
 */
#define kDebugBreak() \
    xkDebugBreak()

#include <kApi/kApiDef.x.h>

#endif

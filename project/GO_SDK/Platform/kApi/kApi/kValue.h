/** 
 * @file    kValue.h
 * @brief   Declares the kValue type. 
 *
 * @internal
 * Copyright (C) 2012-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <kApi/kApiDef.h>   //--inclusion order controlled by kApiDef

#ifndef K_API_VALUE_H
#define K_API_VALUE_H

#include <kApi/kValue.x.h>

/**
 * @class   kValue
 * @ingroup kApi
 * @brief   Root of all Zen value types. 
 * 
 * Value types represent structures, primitive values and enumerations. The kValue base type defines methods that can 
 * be called on any value instance. kValue does not add any public or private fields to the types that extend kValue. 
 * 
 * The kValue_Equals and kValue_HashCode methods are used to support value comparisons and are implemented 
 * by most value types. 
 * 
 * @code {.c}
 *
 * kBool EqualsExample(const kRect32s* a, const kRect32s* b)
 * {
 *     return kValue_Equals(kTypeOf(kRect32s), a, b); 
 * }
 *
 * @endcode
 *
 * Unlike reference types, value types do not carry type information. Accordingly, type information must 
 * be passed as the first argument of any kValue method. 
 */

/**
 * Determines whether a value is equal to another value. 
 * 
 * @public              @memberof kValue
 * @param   type        Value type. 
 * @param   value       Pointer to value. 
 * @param   other       Pointer to other value. 
 * @return              kTRUE if the values are equal.
 */
kInlineFx(kBool) kValue_Equals(kType type, const void* value, const void* other)
{
    return kValue_VTable(type)->VEquals(type, value, other);
}

/**
 * Determines whether a value is equal to another value. 
 * 
 * A debug assertion will be raised if the size of the specified items is not equal to the 
 * size of the value type.
 * 
 * @relates                 kValue
 * @param   kType_type      Value type. 
 * @param   TPtr_value      Strongly-typed pointer to value. 
 * @param   TPtr_other      Strongly-typed pointer to other value. 
 * @return                  kTRUE if the values are equal.
 */
#define kValue_EqualsT(kType_type, TPtr_value, TPtr_other) \
    xkValue_EqualsT(kType_type, TPtr_value, TPtr_other, sizeof(*TPtr_value), sizeof(*(TPtr_other)))

/**
 * Gets a hash code representing the state of this value.
 *
 * @public              @memberof kValue
 * @param   type        Value type. 
 * @param   value       Pointer to value.
 * @return              Hash code. 
 */
kInlineFx(kSize) kValue_HashCode(kType type, const void* value)
{
    return kValue_VTable(type)->VHashCode(type, value);  
}

/**
 * Gets a hash code representing the state of this value.
 *
 * A debug assertion will be raised if the size of the specified item is not equal to the 
 * size of the value type.
 * 
 * @relates                 kValue
 * @param   kType_type      Value type. 
 * @param   TPtr_value      Strongly-typed pointer to value.
 * @return                  Hash code. 
 */
#define kValue_HashCodeT(kType_type, TPtr_value) \
    xkValue_HashCodeT(kType_type, TPtr_value, sizeof(*(TPtr_value)))

/**
 * Imports the content of another value into this value.
 *
 * For array-value types such as kText32, this method supports importing content from an 
 * external source, which may have a different size.
 *
 * @public              @memberof kValue
 * @param   type        Value type. 
 * @param   value       Pointer to value.
 * @param   source      Source for import. 
 */
kInlineFx(void) kValue_Import(kType type, void* value, const void* source)
{
    if (kType_IsArrayValue(type)) 
    {
        kValue_VTable(type)->VImport(type, value, source); 
    }
    else
    {
        //optimization; skip virtual call for non array-value types
        kItemCopy(value, source, kType_Size(type)); 
    }   
}

/**
 * Protected virtual method that compares two values for equality. 
 * 
 * This method can be overridden in derived value types to support kValue_Equals.
 * 
 * The default implementation of this method uses type introspection to compare the values field by field. 
 * This approach is not efficient, and in some cases may not produce the desired result. Value types can 
 * override this method if it is likely that equality comparisons will be required. 
 * 
 * @protected           @memberof kValue
 * @param   type        Value type. 
 * @param   value       Value instance pointer. 
 * @param   other       Value instance pointer for comparison. 
 * @return              Operation status.  
*/
kFx(kBool) kValue_VEquals(kType type, const void* value, const void* other); 

/**
 * Protected virtual method that calculates a hash code representing the value instance. 
 * 
 * The purpose of this method is to generate a hash code that can be used with hash-based 
 * collections such as kMap. 
 * 
 * The default implementation of this method uses type introspection to generate a hash code that combines the 
 * hash codes from individual fields. This approach is not efficient, and may not produce an optimal hash code. 
 * Value types can override this method if it is likely that hash codes will be required. 
 *
 * @protected           @memberof kValue
 * @param   type        Value type. 
 * @param   value       Value instance pointer. 
 * @return              Hash code. 
*/
kFx(kSize) kValue_VHashCode(kType type, const void* value);

/**
 * Protected virtual method that imports data from an external source into an array-value type.
 * 
 * The default implementation of this method copies a number of bytes equal to the size of the 
 * specified data type. However, this is not always convenient when working with array-value types. 
 * For example, by overriding the kValue_VImport method, array-value types such as kText32 can 
 * support importing null-terminated character arrays of unknown length. 
 * 
 * @protected           @memberof kValue
 * @param   type        Value type. 
 * @param   value       Value instance pointer. 
 * @param   source      Pointer to source data.  
 * @return              Hash code. 
*/
kInlineFx(void) kValue_VImport(kType type, void* value, const void* source)
{
    kItemCopy(value, source, kType_Size(type)); 
}


#endif

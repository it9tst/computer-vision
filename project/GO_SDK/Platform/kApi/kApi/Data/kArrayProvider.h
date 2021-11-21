/** 
 * @file    kArrayProvider.h
 * @brief   Declares the kArrayProvider interface. 
 *
 * @internal
 * Copyright (C) 2020-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_ARRAY_PROVIDER_H
#define K_API_ARRAY_PROVIDER_H

#include <kApi/kApiDef.h>
#include <kApi/Data/kArrayProvider.x.h>

/**
 * @interface kArrayProvider
 * @ingroup   kApi-Data
 * @brief     Supports operations on contiguous array-based data types.
 */
//typedef kObject kArrayProvider;   --forward-declared in kApiDef.x.h

/** 
 * Constructs a default instance of the specified array provided type.
 *
 * @public                      @memberof kArrayProvider
 * @param   provider            Receives constructed ArrayProvider object. 
 * @param   type                Type of object to be constructed. 
 * @param   objectAllocator     Object memory allocator (or kNULL for default). 
 * @param   valueAllocator      Value memory allocator (or kNULL for default). 
 * @return                      Operation status. 
 */
kInlineFx(kStatus) kArrayProvider_Construct(kArrayProvider* provider, kType type, kAlloc objectAllocator, kAlloc valueAllocator)
{
    return kType_IVTableT(type, kArrayProvider)->VConstructDefault(provider, objectAllocator, valueAllocator); 
}

/** 
 * Performs a shallow copy of the source array.  
 *
 * Source items are copied by value; if the source array contains objects, the object 
 * handles are copied but the objects are not cloned. 
 * 
 * @public              @memberof kArrayProvider
 * @param   provider    ArrayProvider object. 
 * @param   source      Source array to be copied. 
 * @param   context     Context for copy operation (allocator specific; not usually required).
 * @return              Operation status. 
 */
#if defined(K_CPP)
kInlineFx(kStatus) kArrayProvider_Assign(kArrayProvider provider, kArrayProvider source, kObject context = kNULL)
{
    return xkArrayProvider_VTable(provider)->VAssign(provider, source, context);
}
#endif

/** 
 * Copies the properties of a source array, such as its dimensions, without copying its data.
 *
 * @public              @memberof kArrayProvider
 * @param   provider    ArrayProvider object. 
 * @param   source      Source array to be imitated. 
 * @return              Operation status. 
 */
kInlineFx(kStatus) kArrayProvider_Imitate(kArrayProvider provider, kArrayProvider source)
{
    return xkArrayProvider_VTable(provider)->VImitate(provider, source);
}

/** 
 * Returns the array item type. 
 *
 * @public              @memberof kArrayProvider
 * @param   provider    ArrayProvider object. 
 * @return              Array item type.  
 */
kInlineFx(kType) kArrayProvider_ItemType(kArrayProvider provider)
{
    return xkArrayProvider_VTable(provider)->VItemType(provider);
}

/** 
 * Returns the array item size. 
 *
 * @public              @memberof kArrayProvider
 * @param   provider    ArrayProvider object. 
 * @return              Item size, in bytes.  
 */
kInlineFx(kSize) kArrayProvider_ItemSize(kArrayProvider provider)
{
    return kType_Size(kArrayProvider_ItemType(provider)); 
}

/** 
 * Gets the array element count.
 *
 * @public              @memberof kArrayProvider
 * @param   provider    ArrayProvider object. 
 * @return              Item count. 
 */
kInlineFx(kSize) kArrayProvider_Count(kArrayProvider provider)
{
    return xkArrayProvider_VTable(provider)->VCount(provider);
}

/** 
 * Gets a pointer to the array element data.
 *
 * @public              @memberof kArrayProvider
 * @param   provider    ArrayProvider object. 
 * @return              Array data pointer. 
 */
kInlineFx(kPointer) kArrayProvider_Data(kArrayProvider provider)
{
    return xkArrayProvider_VTable(provider)->VData(provider);
}

/** 
 * Reports the size, in bytes, of the array item buffer. 
 *
 * @public              @memberof kArrayProvider
 * @param   provider    ArrayProvider object. 
 * @return              Size of array item buffer (bytes). 
 */
kInlineFx(kSize) kArrayProvider_DataSize(kArrayProvider provider)
{
    return xkArrayProvider_VTable(provider)->VDataSize(provider);
}

/** 
 * Reports the value allocator that was optionally provided at construction time.
 *
 * @public              @memberof kArrayProvider
 * @param   provider    ArrayProvider object. 
 * @return              Allocator for internal data array.
 */
kInlineFx(kAlloc) kArrayProvider_ValueAlloc(kArrayProvider provider)
{
    return xkArrayProvider_VTable(provider)->VDataAlloc(provider);
}

#endif

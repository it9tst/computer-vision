/** 
 * @file    kUserAlloc.h
 * @brief   Declares the kUserAlloc class. 
 *
 * @internal
 * Copyright (C) 2005-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_USER_ALLOC_H
#define K_API_USER_ALLOC_H

#include <kApi/kApiDef.h> 
#include <kApi/Utils/kUserAlloc.x.h>

/**
 * @class   kUserAlloc
 * @extends kAlloc
 * @ingroup kApi-Utils
 * @brief   Allocates memory from a user-defined memory source.
 */
//typedef kAlloc kUserAlloc;   --forward-declared in kApiDef.x.h

/** 
 * Constructs a new kUserAlloc instance. 
 *
 * @public              @memberof kUserAlloc
 * @param   object      Receives the constructed kUserAlloc instance. 
 * @param   allocFx     User-defined memory allocation function. 
 * @param   freeFx      User-defined memory free function.
 * @param   provider    User-defined context pointer.
 * @param   allocator   Memory allocator for this object instance (or kNULL for default). 
 * @return              Operation status. 
 */
kFx(kStatus) kUserAlloc_Construct(kUserAlloc* object, kApiMemAllocFx allocFx, kApiMemFreeFx freeFx, kPointer provider, kAlloc allocator); 

/** 
 * Adds traits reported by the allocator.
 *
 * @public              @memberof kUserAlloc
 * @param   object      kUserAlloc instance.
 * @param   traits      Allocator traits.
 */
kInlineFx(void) kUserAlloc_AddTraits(kUserAlloc object, kAllocTrait traits)
{
    xkUserAlloc_Cast(object)->base.traits |= traits;
}

/** 
 * Removes traits reported by the allocator.
 *
 * @public              @memberof kUserAlloc
 * @param   object      kUserAlloc instance.
 * @param   traits      Allocator traits.
 */
kInlineFx(void) kUserAlloc_RemoveTraits(kUserAlloc object, kAllocTrait traits)
{
    xkUserAlloc_Cast(object)->base.traits &= ~traits;
}

/** 
 * Gets provider handle passed to kUserAlloc constructor.
 *
 * @public              @memberof kUserAlloc
 * @param   object      kUserAlloc instance.
 * @return              User-defined context pointer.
 */
kFx(kPointer) kUserAlloc_Provider(kUserAlloc object); 

#endif

/** 
 * @file    kObjectPool.h
 * @brief   Declares the kObjectPool interface. 
 *
 * @internal
 * Copyright (C) 2013-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_OBJECT_POOL_H
#define K_API_OBJECT_POOL_H

#include <kApi/kApiDef.h>  
#include <kApi/Utils/kObjectPool.x.h>

/**
 * @interface kObjectPool
 * @ingroup   kApi-Utils
 * @brief     Supports reclaiming objects upon destruction.
 * @see       kObject_SetPool, kObject_Destroy, kObject_Dispose
 */
//typedef kObject kObjectPool;   --forward-declared in kApiDef.x.h

/*
* Public 
*/

/** 
 * Receives an object just prior to its destruction.
 *
 * @public          @memberof kObjectPool
 * @param   pool    Pool object. 
 * @param   object  Object to be reclaimed. 
 * @return          Operation status.
 * @see             kObject_SetPool, kObject_Destroy, kObject_Dispose
 */
kInlineFx(kStatus) kObjectPool_Reclaim(kObjectPool pool, kObject object)
{
    return xkObjectPool_VTable(pool)->VReclaim(pool, object);
}

#endif

/** 
 * @file    kObjectPool.x.h
 *
 * @internal
 * Copyright (C) 2013-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_OBJECT_POOL_X_H
#define K_API_OBJECT_POOL_X_H

typedef struct kObjectPoolVTable
{   
    kStatus (kCall* VReclaim)(kObjectPool pool, kObject object);
} kObjectPoolVTable;

kDeclareInterfaceEx(k, kObjectPool, kNull) 

/* 
* Private interface implementation stubs.
*/

kInlineFx(kStatus) xkObjectPool_VReclaim(kObjectPool pool, kObject object)
{
    return kERROR_UNIMPLEMENTED; 
}

#endif

/** 
 * @file    kObjectPool.cpp
 *
 * @internal
 * Copyright (C) 2013-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <kApi/Utils/kObjectPool.h>

kBeginInterfaceEx(k, kObjectPool) 
    //interface methods
    kAddPrivateVMethod(kObjectPool, kObjectPool, VReclaim)
kEndInterfaceEx() 


/** 
 * @file    kArrayProvider.cpp
 *
 * @internal
 * Copyright (C) 2020-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <kApi/Data/kArrayProvider.h>

kBeginInterfaceEx(k, kArrayProvider) 
    //interface methods
    kAddPrivateVMethod(kArrayProvider, kArrayProvider, VConstructDefault)
    kAddPrivateVMethod(kArrayProvider, kArrayProvider, VAssign)
    kAddPrivateVMethod(kArrayProvider, kArrayProvider, VImitate)
    kAddPrivateVMethod(kArrayProvider, kArrayProvider, VItemType)
    kAddPrivateVMethod(kArrayProvider, kArrayProvider, VCount)
    kAddPrivateVMethod(kArrayProvider, kArrayProvider, VData)
    kAddPrivateVMethod(kArrayProvider, kArrayProvider, VDataSize)
    kAddPrivateVMethod(kArrayProvider, kArrayProvider, VDataAlloc)

kEndInterfaceEx() 

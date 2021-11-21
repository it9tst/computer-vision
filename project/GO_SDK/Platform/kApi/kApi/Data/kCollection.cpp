/** 
 * @file    kCollection.cpp
 *
 * @internal
 * Copyright (C) 2008-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <kApi/Data/kCollection.h>

kBeginInterfaceEx(k, kCollection) 
    //interface methods
    kAddPrivateVMethod(kCollection, kCollection, VGetIterator)
    kAddPrivateVMethod(kCollection, kCollection, VItemType)
    kAddPrivateVMethod(kCollection, kCollection, VCount)
    kAddPrivateVMethod(kCollection, kCollection, VHasNext)
    kAddPrivateVMethod(kCollection, kCollection, VNext)
kEndInterfaceEx() 

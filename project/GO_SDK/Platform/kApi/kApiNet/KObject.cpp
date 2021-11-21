// 
// KObject.cpp
//
// Copyright (C) 2014-2021 by LMI Technologies Inc.
// Licensed under the MIT License.
// Redistributed files must retain the above copyright notice.
//
#include "kApiNet/KObject.h"
#include "kApiNet/KAlloc.h"
#include "kApiNet/KAssembly.h"
#include "kApiNet/KType.h"
#include "kApiNet/Utils/KObjectPool.h"

KDefineClass(KObject, kObject)

KType^ KObject::GetKType()
{
    return KToObject<KType^>(kObject_Type(m_handle));
}


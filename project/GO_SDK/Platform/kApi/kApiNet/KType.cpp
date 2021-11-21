// 
// @file    KType.cpp
// 
// @internal
// Copyright (C) 2014-2021 by LMI Technologies Inc.
// Licensed under the MIT License.
// Redistributed files must retain the above copyright notice.
// 
#include "kApiNet/KType.h"
#include "kApiNet/KAssembly.h"
#include "kApiNet/Utils/KObjectPool.h"

KDefineClass(KType, kType)

KType^ KFieldInfo::Type::get()
{
    return KToObject<KType^>(m_type);
}

KAssembly^ KType::Assembly::get()
{
    return KToObject<KAssembly^>(IntPtr(kType_Assembly(Handle))); 
}

/** 
 * @file    kArrayProvider.x.h
 *
 * @internal
 * Copyright (C) 2020-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_ARRAY_PROVIDER_X_H
#define K_API_ARRAY_PROVIDER_X_H

typedef struct kArrayProviderVTable
{   
    kStatus (kCall* VConstructDefault)(kArrayProvider* provider, kAlloc objectAlloc, kAlloc valueAlloc);
    kStatus (kCall* VImitate)(kArrayProvider provider, kArrayProvider source);
    kStatus (kCall* VAssign)(kArrayProvider provider, kArrayProvider source, kObject context);
    kType (kCall* VItemType)(kArrayProvider provider);
    kSize (kCall* VCount)(kArrayProvider provider);
    kPointer (kCall* VData)(kArrayProvider provider);
    kSize (kCall* VDataSize)(kArrayProvider provider);
    kAlloc (kCall* VDataAlloc)(kArrayProvider provider);
} kArrayProviderVTable;

kDeclareInterfaceEx(k, kArrayProvider, kNull) 

/*
* Forard declarations.
*/

kInlineFx(kType) kArrayProvider_ItemType(kArrayProvider provider); 

/* 
* Private methods. 
*/

kInlineFx(kStatus) xkArrayProvider_VConstructDefault(kArrayProvider* provider, kAlloc objectAlloc, kAlloc valueAlloc)
{
    return kERROR_UNIMPLEMENTED;
}

kInlineFx(kStatus) xkArrayProvider_VAssign(kArrayProvider provider, kArrayProvider source, kObject context)
{
    return kERROR_UNIMPLEMENTED;
}

kInlineFx(kStatus) xkArrayProvider_VImitate(kArrayProvider provider, kArrayProvider source)
{
    return kERROR_UNIMPLEMENTED;
}

kInlineFx(kType) xkArrayProvider_VItemType(kArrayProvider provider)
{
    return kNULL;
}

kInlineFx(kSize) xkArrayProvider_VCount(kArrayProvider provider)
{
    return 0;
}

kInlineFx(kPointer) xkArrayProvider_VData(kArrayProvider provider)
{
    return kNULL;
}

kInlineFx(kSize) xkArrayProvider_VDataSize(kArrayProvider provider)
{
    return 0;
}

kInlineFx(kAlloc) xkArrayProvider_VDataAlloc(kArrayProvider provider)
{
    return kNULL;
}

#endif

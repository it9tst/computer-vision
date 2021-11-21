/** 
 * @file    kSymbolInfo.x.h
 *
 * @internal
 * Copyright (C) 2016-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_SYMBOL_TABLE_X_H
#define K_API_SYMBOL_TABLE_X_H

#include <kApi/Io/kPath.h>

#if defined(K_PLATFORM) && defined(K_WINDOWS)

#   define xkSymbolInfoPlatformFields()      \
    kPointer service;                       \
    void* symbolInfo;                       \
    void* lineInfo;                         \
    kChar tempPath1[kPATH_MAX];             \
    kChar tempPath2[kPATH_MAX];

#else 

#   define xkSymbolInfoPlatformFields()  

#endif


typedef struct kSymbolInfoStatic
{
    kAlloc alloc;               //Allocator used to allocate state and temp variables.
    kLock lock;                 //Used on Windows for exlusive access to symbol information.
    kMap cache;                 //Caches strings that describe function calls -- kMap<kPointer, kString>

    xkSymbolInfoPlatformFields()
} kSymbolInfoStatic; 

kDeclareStaticClassEx(k, kSymbolInfo)

/* 
* Private methods. 
*/

kFx(kStatus) xkSymbolInfo_InitStatic(); 
kFx(kStatus) xkSymbolInfo_ReleaseStatic(); 

kFx(void) xkSymbolInfo_Lock(); 
kFx(void) xkSymbolInfo_Unlock(); 

kFx(kStatus) xkSymbolInfo_OnAssemblyLoad(kSymbolInfo trace, kAssembly assembly, kPointer unused); 

kFx(kStatus) xkSymbolInfo_FormatUnknown(kPointer function, kString description); 

#endif

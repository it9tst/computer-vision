/** 
 * @file    kDynamicLib.x.h
 *
 * @internal
 * Copyright (C) 2013-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_DYNAMICLIB_X_H
#define K_API_DYNAMICLIB_X_H

kDeclareClassEx(k, kDynamicLib, kObject)

typedef struct kDynamicLibClass
{
    kObjectClass base;
    kPointer handle;
    kBool isOwned;
} kDynamicLibClass;

/* 
* Private methods. 
*/

kFx(kStatus) kDynamicLib_ConstructFromHandle(kDynamicLib* library, kPointer handle, kAlloc allocator);

kFx(kStatus) xkDynamicLib_Init(kDynamicLib library, kType type, const kChar* path, kAlloc allocator);
kFx(kStatus) xkDynamicLib_InitFromHandle(kDynamicLib library, kType type, kPointer handle, kAlloc allocator);
kFx(kStatus) xkDynamicLib_VRelease(kDynamicLib library); 

kFx(kStatus) xkDynamicLib_OpenHandle(const kChar* path, kPointer* handle);
kFx(kStatus) xkDynamicLib_Resolve(kPointer handle, const kChar* name, kFunction* function);
kFx(kStatus) xkDynamicLib_CloseHandle(kPointer handle);

#endif

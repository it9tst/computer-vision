/** 
 * @file    kDateTime.x.h
 *
 * @internal
 * Copyright (C) 2008-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_DATE_TIME_X_H
#define K_API_DATE_TIME_X_H

/*
* kDateTimeFormat enum
*/

kDeclareEnumEx(k, kDateTimeFormat, kValue)

/*
* xkDateTimeManager static class
* 
* (Required for K_TI_BIOS support)
*/

typedef struct xkDateTimeManagerStatic
{
    kLock lock;         //provides exclusive access to underlying calendar date/time functions
} xkDateTimeManagerStatic;

kDeclareStaticClassEx(k, xkDateTimeManager)

kFx(kStatus) xxkDateTimeManager_InitStatic();
kFx(kStatus) xxkDateTimeManager_ReleaseStatic();

/*
* kDateTime structure
*/

kDeclareValueEx(k, kDateTime, kValue)

kFx(k64s) xkDateTime_DefaultNow(); 

kFx(kBool) xkDateTime_VEquals(kType type, const void* value, const void* other);
kFx(kSize) xkDateTime_VHashCode(kType type, const void* value);
kFx(kStatus) xkDateTime_Write(kType type, const void* values, kSize count, kSerializer serializer);
kFx(kStatus) xkDateTime_Read(kType type, void* values, kSize count, kSerializer serializer);

#endif

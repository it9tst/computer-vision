/** 
 * @file    kTimeSpan.x.h
 *
 * @internal
 * Copyright (C) 2008-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_TIME_SPAN_X_H
#define K_API_TIME_SPAN_X_H

/*
* kTimeSpanFormat enum
*/

kDeclareEnumEx(k, kTimeSpanFormat, kValue)

/*
* kTimeSpan structure
*/

kDeclareValueEx(k, kTimeSpan, kValue)

kFx(kBool) xkTimeSpan_VEquals(kType type, const void* value, const void* other);
kFx(kSize) xkTimeSpan_VHashCode(kType type, const void* value);
kFx(kStatus) xkTimeSpan_Write(kType type, const void* values, kSize count, kSerializer serializer);
kFx(kStatus) xkTimeSpan_Read(kType type, void* values, kSize count, kSerializer serializer);

kFx(kStatus) xkTimeSpan_FormatParts(kTimeSpan span, kBool includeMicroseconds, kChar* text, kSize capacity);

kFx(kStatus) xkTimeSpan_ParseFull(kTimeSpan* span, const kChar* text);
kFx(kStatus) xkTimeSpan_ParseSimple(kTimeSpan* span, const kChar* text);
kFx(kStatus) xkTimeSpan_ParseHmsFull(kTimeSpan* span, const kChar* text);
kFx(kStatus) xkTimeSpan_ParseHmsSimple(kTimeSpan* span, const kChar* text);

#endif

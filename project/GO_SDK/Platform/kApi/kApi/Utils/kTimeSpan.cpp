/** 
 * @file    kTimeSpan.cpp
 *
 * @internal
 * Copyright (C) 2008-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <kApi/Utils/kTimeSpan.h>

#include <kApi/Data/kMath.h>
#include <kApi/Io/kSerializer.h>

#include <stdio.h>

/*
* kTimeSpanFormat enum
*/

kBeginEnumEx(k, kTimeSpanFormat)
    kAddEnumerator(kTimeSpanFormat, kTIME_SPAN_FORMAT_PARTS_FULL)
    kAddEnumerator(kTimeSpanFormat, kTIME_SPAN_FORMAT_PARTS_SIMPLE)
    kAddEnumerator(kTimeSpanFormat, kTIME_SPAN_FORMAT_HMS_FULL)
    kAddEnumerator(kTimeSpanFormat, kTIME_SPAN_FORMAT_HMS_SIMPLE)
kEndEnumEx()

/*
* kTimeSpan structure
*/

kBeginValueEx(k, kTimeSpan)
    kAddFlags(kTimeSpan, kTYPE_FLAGS_PRIMITIVE)

    kAddPrivateVersionEx(kTimeSpan, "kdat6", "7.0.0.0", "kTimeSpan-0", Write, Read)

    kAddPrivateVMethod(kTimeSpan, kValue, VEquals)
    kAddPrivateVMethod(kTimeSpan, kValue, VHashCode)
kEndValueEx()

kFx(kBool) xkTimeSpan_VEquals(kType type, const void* value, const void* other)
{
    return *(kTimeSpan*)value == *(kTimeSpan*)other; 
}

kFx(kSize) xkTimeSpan_VHashCode(kType type, const void* value)
{
    return xk64s_VHashCode(kTypeOf(k64s), value);
}

kFx(kStatus) xkTimeSpan_Write(kType type, const void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Write64sArray(serializer, (const k64s*) values, count); 
}

kFx(kStatus) xkTimeSpan_Read(kType type, void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Read64sArray(serializer, (k64s*) values, count); 
}

kFx(kStatus) kTimeSpan_Parts(kTimeSpan span, k32s* daysPart, k32s* hoursPart, k32s* minutesPart, k32s* secondsPart, k32s* microsecondsPart)
{
    if (!kIsNull(daysPart))         *daysPart         = kTimeSpan_DaysPart(span);
    if (!kIsNull(hoursPart))        *hoursPart        = kTimeSpan_HoursPart(span);
    if (!kIsNull(minutesPart))      *minutesPart      = kTimeSpan_MinutesPart(span);
    if (!kIsNull(secondsPart))      *secondsPart      = kTimeSpan_SecondsPart(span);
    if (!kIsNull(microsecondsPart)) *microsecondsPart = kTimeSpan_MicrosecondsPart(span);

    return kOK;
}

kFx(kStatus) kTimeSpan_Format(kTimeSpan span, kTimeSpanFormat format, kChar* text, kSize capacity)
{
    switch (format)
    {
        case kTIME_SPAN_FORMAT_NULL:
        case kTIME_SPAN_FORMAT_PARTS_FULL:
          return xkTimeSpan_FormatParts(span, kTRUE, text, capacity);
        case kTIME_SPAN_FORMAT_PARTS_SIMPLE:
          return xkTimeSpan_FormatParts(span, kFALSE, text, capacity);
        case kTIME_SPAN_FORMAT_HMS_FULL:
          return kStrPrintf(text, capacity, "%lld:%02d:%02d.%06d", kTimeSpan_ToHours64s(span), 
                kTimeSpan_MinutesPart(span), kTimeSpan_SecondsPart(span), kTimeSpan_MicrosecondsPart(span)); 
        case kTIME_SPAN_FORMAT_HMS_SIMPLE:
          return kStrPrintf(text, capacity, "%lld:%02d:%02d", kTimeSpan_ToHours64s(span), 
                kTimeSpan_MinutesPart(span), kTimeSpan_SecondsPart(span)); 
        default:
            return kERROR_PARAMETER;
    }
}

kFx(kStatus) xkTimeSpan_FormatParts(kTimeSpan span, kBool includeMicroseconds, kChar* text, kSize capacity)
{
    k32s daysPart, hoursPart, minutesPart, secondsPart, microsecondsPart; 
    kText16 secondsText; 

    kCheck(kTimeSpan_Parts(span, &daysPart, &hoursPart, &minutesPart, &secondsPart, &microsecondsPart)); 

    if (includeMicroseconds)
    {
        kCheck(kStrPrintf(secondsText, sizeof(secondsText), "%d.%06ds", secondsPart, microsecondsPart));
    }
    else
    {
        kCheck(kStrPrintf(secondsText, sizeof(secondsText), "%ds", secondsPart));
    }

    if (daysPart > 0)
    {
        kCheck(kStrPrintf(text, capacity, "%dd %dh %dm %s", daysPart, hoursPart, minutesPart, secondsText));
    }
    else if (hoursPart > 0)
    {
        kCheck(kStrPrintf(text, capacity, "%dh %dm %s", hoursPart, minutesPart, secondsText));
    }
    else if (minutesPart > 0)
    {
        kCheck(kStrPrintf(text, capacity, "%dm %s", minutesPart, secondsText));
    }
    else
    {
        kCheck(kStrCopy(text, capacity, secondsText));
    }

    return kOK; 
}

kFx(kStatus) kTimeSpan_Parse(kTimeSpan* span, kTimeSpanFormat format, const kChar* text)
{
    switch (format)
    {
        case kTIME_SPAN_FORMAT_PARTS_FULL:
            return xkTimeSpan_ParseFull(span, text);
        case kTIME_SPAN_FORMAT_PARTS_SIMPLE:
            return xkTimeSpan_ParseSimple(span, text);
        case kTIME_SPAN_FORMAT_HMS_FULL:
            return xkTimeSpan_ParseHmsFull(span, text);            
        case kTIME_SPAN_FORMAT_HMS_SIMPLE:
            return xkTimeSpan_ParseHmsSimple(span, text);
        default:
            //try all known formats, from most detailed to least detailed
            return (kSuccess(kTimeSpan_Parse(span, kTIME_SPAN_FORMAT_HMS_FULL, text)) ||
                    kSuccess(kTimeSpan_Parse(span, kTIME_SPAN_FORMAT_HMS_SIMPLE, text)) ||
                    kSuccess(kTimeSpan_Parse(span, kTIME_SPAN_FORMAT_PARTS_FULL, text)) ||
                    kSuccess(kTimeSpan_Parse(span, kTIME_SPAN_FORMAT_PARTS_SIMPLE, text))) ? kOK : kERROR_FORMAT;
    }
}

kFx(kStatus) xkTimeSpan_ParseFull(kTimeSpan* span, const kChar* text)
{
    k32s daysPart = 0, hoursPart = 0, minutesPart = 0, secondsPart = 0, microsecondsPart = 0;
    
    if (sscanf(text, "%dd %dh %dm %d.%06ds", &daysPart, &hoursPart, &minutesPart, &secondsPart, &microsecondsPart) != 5)
    {
        daysPart = 0;

        if (sscanf(text, "%dh %dm %d.%06ds", &hoursPart, &minutesPart, &secondsPart, &microsecondsPart) != 4)
        {
            hoursPart = 0;

            if (sscanf(text, "%dm %d.%06ds", &minutesPart, &secondsPart, &microsecondsPart) != 3)
            {
                minutesPart = 0;

                kCheckTrue(sscanf(text, "%d.%06ds", &secondsPart, &microsecondsPart) == 2, kERROR_FORMAT);
            }
        }        
    }
                                           
    *span = kTimeSpan_FromParts(daysPart, hoursPart, minutesPart, secondsPart, microsecondsPart);

    return kOK; 
}

kFx(kStatus) xkTimeSpan_ParseSimple(kTimeSpan* span, const kChar* text)
{
    k32s daysPart = 0, hoursPart = 0, minutesPart = 0, secondsPart = 0;
    
    if (sscanf(text, "%dd %dh %dm %ds", &daysPart, &hoursPart, &minutesPart, &secondsPart) != 4)
    {
        daysPart = 0;

        if (sscanf(text, "%dh %dm %ds", &hoursPart, &minutesPart, &secondsPart) != 3)
        {
            hoursPart = 0;

            if (sscanf(text, "%dm %ds", &minutesPart, &secondsPart) != 2)
            {
                minutesPart = 0;

                kCheckTrue(sscanf(text, "%ds", &secondsPart) == 1, kERROR_FORMAT);
            }
        }        
    }
                                           
    *span = kTimeSpan_FromParts(daysPart, hoursPart, minutesPart, secondsPart, 0);

    return kOK; 
}

kFx(kStatus) xkTimeSpan_ParseHmsFull(kTimeSpan* span, const kChar* text)
{
    k32s daysPart = 0, hoursPart = 0, minutesPart = 0, secondsPart = 0, microsecondsPart = 0;

    kCheckTrue(sscanf(text, "%d:%02d:%02d.%06d", &hoursPart, &minutesPart, &secondsPart, &microsecondsPart) == 4, kERROR_FORMAT); 
    
    *span = kTimeSpan_FromParts(daysPart, hoursPart, minutesPart, secondsPart, microsecondsPart);

    return kOK;
}

kFx(kStatus) xkTimeSpan_ParseHmsSimple(kTimeSpan* span, const kChar* text)
{
    k32s daysPart = 0, hoursPart = 0, minutesPart = 0, secondsPart = 0;

    kCheckTrue(sscanf(text, "%d:%02d:%02d", &hoursPart, &minutesPart, &secondsPart) == 3, kERROR_FORMAT); 
    
    *span = kTimeSpan_FromParts(daysPart, hoursPart, minutesPart, secondsPart, 0);

    return kOK;
}

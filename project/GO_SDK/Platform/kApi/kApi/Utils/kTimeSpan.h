/** 
 * @file    kTimeSpan.h
 * @brief   Declares the kTimeSpan type. 
 *
 * @internal
 * Copyright (C) 2008-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_TIME_SPAN_H
#define K_API_TIME_SPAN_H

#include <kApi/kApiDef.h>
#include <kApi/Data/kMath.h>

/**
 * @struct  kTimeSpanFormat
 * @extends kValue
 * @ingroup kApi-Utils  
 * @brief   Represents a timespan text format. 
 */
typedef k32s kTimeSpanFormat; 

/** @relates kTimeSpanFormat @{ */
#define kTIME_SPAN_FORMAT_NULL          (0)         ///< Unknown format value. 
#define kTIME_SPAN_FORMAT_PARTS_FULL    (1)         ///< Complete representation, expressed as broken-down time parts. E.g., 1d 4h 32m 45.123456s 
#define kTIME_SPAN_FORMAT_PARTS_SIMPLE  (2)         ///< Similar to PartsFull, but omits fractional seconds. E.g., 1d 4h 32m 45s
#define kTIME_SPAN_FORMAT_HMS_FULL      (3)         ///< Complete reprsentation, expressed in hours-minutes-seconds format. E.g., 28:32:45.123456
#define kTIME_SPAN_FORMAT_HMS_SIMPLE    (4)         ///< Similar to HmsFull, but omits fractional seconds. E.g., 28:32:45
/** @} */

/**
 * @class   kTimeSpan
 * @extends kObject
 * @ingroup kApi-Utils  
 * @brief   Represents a span of time.
 * 
 * kTimeSpan is a type alias for a 64-bit signed integer that represents a count of elapsed microseconds.
 *
 * Methods are provided for converting to/from broken-down time parts. Support is also provided to 
 * format/parse time span values to/from a small number of different text representations. 
 *
 * @code {.c}
 * 
 * kTimeSpan elapsed = kDateTime_Now() - then; 
 * kText64 elapsedText; 
 * 
 * kCheck(kTimeSpan_Format(elapsed, kTIME_SPAN_FORMAT_PARTS_FULL, elapsedText, sizeof(elapsedText)));
 * 
 * kLogf("Elapsed time: %s", elapsedText);
 * 
 * kLogf("Elapsed time in hours: %f", kTimeSpan_ToHours64f(elapsed));
 * 
 * @endcode
 * 
 * kTimeSpan supports the kdat6 serialization protocol.
 * 
 * kTimeSpan overrides kValue_Equals and kValue_HashCode.
 */
typedef k64s kTimeSpan; 

#include <kApi/Utils/kTimeSpan.x.h>

/**
* Creates a timespan from broken-down time components.
* 
* @relates                      kTimeSpan
* @param     daysPart           Days part.
* @param     hoursPart          Hours part.
* @param     minutesPart        Minutes part.
* @param     secondsPart        Seconds part.
* @param     microsecondsPart   Microseconds part.
* @return                       Timespan composed from the specified components.
*/
kInlineFx(kTimeSpan) kTimeSpan_FromParts(k32s daysPart, k32s hoursPart, k32s minutesPart, k32s secondsPart, k32s microsecondsPart)
{
    return (((daysPart*24LL + hoursPart)*60LL + minutesPart)*60LL + secondsPart)*1000000LL + microsecondsPart;
}

/**
* Gets broken-down time components from a timespan.
* 
* @relates                     kTimeSpan
* @param     span              Time span.
* @param     daysPart          Optionally receives days part (0-N); can be kNULL.
* @param     hoursPart         Optionally receives hours part (0-23); can be kNULL. 
* @param     minutesPart       Optionally receives minutes part (0-59); can be kNULL.
* @param     secondsPart       Optionally receives seconds part (0-59); can be kNULL.
* @param     microsecondsPart  Optionally receives microseconds part (0-999999); can be kNULL. 
* @return                      Operation status.
*/
kFx(kStatus) kTimeSpan_Parts(kTimeSpan span, k32s* daysPart, k32s* hoursPart, k32s* minutesPart, k32s* secondsPart, k32s* microsecondsPart);

/**
* Gets broken-down days component of the timespan. 
* 
* @relates          kTimeSpan
* @param     span   Time span.
* @return           Days part of the timespan (0-N).
*/
kInlineFx(k32s) kTimeSpan_DaysPart(kTimeSpan span)
{
    return (k32s) (span / (24LL * 60LL * 60LL * 1000000LL));
}

/**
* Gets broken-down hours component of the timespan. 
* 
* @relates          kTimeSpan
* @param     span   Time span.
* @return           Hours part of the timespan (0-23). 
*/
kInlineFx(k32s) kTimeSpan_HoursPart(kTimeSpan span)
{
    return (k32s) ((span / (60LL * 60LL * 1000000LL)) % 24LL);
}

/**
* Gets broken-down minutes component of the timespan.
* 
* @relates          kTimeSpan
* @param     span   Time span.
* @return           Minutes part of the timespan (0-59). 
*/
kInlineFx(k32s) kTimeSpan_MinutesPart(kTimeSpan span)
{
    return (k32s) ((span / (60LL * 1000000LL)) % 60LL);
}  

/**
* Gets broken-down seconds component of the timespan. 
* 
* @relates          kTimeSpan
* @param     span   Time span.
* @return           Seconds part of the timespan (0-59). 
*/
kInlineFx(k32s) kTimeSpan_SecondsPart(kTimeSpan span)
{
    return (k32s) ((span / 1000000LL) % 60LL);
} 

/**
* Gets broken-down microseconds component of timespan. 
* 
* @relates          kTimeSpan
* @param     span   Time span.
* @return           Microseconds part of the timespan (0-999999). 
*/
kInlineFx(k32s) kTimeSpan_MicrosecondsPart(kTimeSpan span)
{
    return (k32s) (span % 1000000LL);
} 

/**
* Creates a timespan from a total number of days.
* 
* @relates          kTimeSpan
* @param     days   Total number of days.
* @return           Timespan.
*/
kInlineFx(kTimeSpan) kTimeSpan_FromDays64s(k64s days)
{
    return days * 24LL * 60LL * 60LL * 1000000LL;
}

/**
* Creates a timespan from a total number of days.
* 
* @relates          kTimeSpan
* @param     days   Total number of days.
* @return           Timespan.
*/
kInlineFx(kTimeSpan) kTimeSpan_FromDays64f(k64f days)
{
    return (kTimeSpan) kMath_Round64s_(days * 24.0 * 60.0 * 60.0 * 1000000.0);
}

/**
* Creates a timespan from a total number of hours.
* 
* @relates          kTimeSpan
* @param     hours  Total number of hours.
* @return           Timespan.
*/
kInlineFx(kTimeSpan) kTimeSpan_FromHours64s(k64s hours)
{
    return hours * 60LL * 60LL * 1000000LL;
}

/**
* Creates a timespan from a total number of hours.
* 
* @relates          kTimeSpan
* @param     hours  Total number of hours.
* @return           Timespan.
*/
kInlineFx(kTimeSpan) kTimeSpan_FromHours64f(k64f hours)
{
    return (kTimeSpan) kMath_Round64s_(hours * 60.0 * 60.0 * 1000000.0);
}

/**
* Creates a timespan from a total number of minutes.
* 
* @relates              kTimeSpan
* @param     minutes    Number of minutes.
* @return               Timespan.
*/
kInlineFx(kTimeSpan) kTimeSpan_FromMinutes64s(k64s minutes)
{
    return minutes * 60LL * 1000000LL;
}

/**
* Creates a timespan from a total number of minutes.
* 
* @relates              kTimeSpan
* @param     minutes    Total number of minutes.
* @return               Timespan.
*/
kInlineFx(kTimeSpan) kTimeSpan_FromMinutes64f(k64f minutes)
{
    return (kTimeSpan) kMath_Round64s_(minutes * 60.0 * 1000000.0);
}

/**
* Creates a timespan from a total number of seconds.
* 
* @relates             kTimeSpan
* @param     seconds   Number of seconds.
* @return              Timespan.
*/
kInlineFx(kTimeSpan) kTimeSpan_FromSeconds64s(k64s seconds)
{
    return seconds * 1000000LL;
}

/**
* Creates a timespan from a total number of seconds.
* 
* @relates             kTimeSpan
* @param     seconds   Total number of seconds.
* @return              Timespan.
*/
kInlineFx(kTimeSpan) kTimeSpan_FromSeconds64f(k64f seconds)
{
    return (kTimeSpan) kMath_Round64s_(seconds * 1000000.0);
}

/**
* Creates a timespan from a total number of milliseconds.
* 
* @relates                  kTimeSpan
* @param     milliseconds   Number of milliseconds.
* @return                   Timespan.
*/
kInlineFx(kTimeSpan) kTimeSpan_FromMilliseconds64s(k64s milliseconds)
{
    return milliseconds * 1000LL;
}

/**
* Creates a timespan from a total number of milliseconds.
* 
* @relates                  kTimeSpan
* @param     milliseconds   Total number of milliseconds.
* @return                   Timespan.
*/
kInlineFx(kTimeSpan) kTimeSpan_FromMilliseconds64f(k64f milliseconds)
{
    return (kTimeSpan) kMath_Round64s_(milliseconds * 1000LL);
}

/**
* Creates a timespan from a total number of microseconds.
* 
* @relates                  kTimeSpan
* @param     microseconds   Number of microseconds.
* @return                   Timespan.
*/
kInlineFx(kTimeSpan) kTimeSpan_FromMicroseconds64s(k64s microseconds)
{
    return microseconds;
}

/**
* Creates a timespan from a total number of microseconds.
* 
* @relates                  kTimeSpan
* @param     microseconds   Number of microseconds.
* @return                   Timespan.
*/
kInlineFx(kTimeSpan) kTimeSpan_FromMicroseconds64f(k64f microseconds)
{
    return (kTimeSpan) kMath_Round64s_(microseconds);
}

/**
* Reports the total days represented by the timespan.
* 
* @relates          kTimeSpan
* @param     span   Time span.
* @return           Total days represented by the timespan.
*/
kInlineFx(k64s) kTimeSpan_ToDays64s(kTimeSpan span)
{
    return span / (24LL * 60LL * 60LL * 1000000LL);
} 

/**
* Reports the total days represented by the timespan.
* 
* @relates          kTimeSpan
* @param     span   Time span.
* @return           Days represented by the timespan.
*/
kInlineFx(k64f) kTimeSpan_ToDays64f(kTimeSpan span)
{
    return (k64f)span / (24.0 * 60.0 * 60.0 * 1000000.0);
} 

/**
* Reports the total hours represented by the timespan.
* 
* @relates          kTimeSpan
* @param     span   Time span.
* @return           Total hours represented by the timespan.
*/
kInlineFx(k64s) kTimeSpan_ToHours64s(kTimeSpan span)
{
    return span / (60LL * 60LL * 1000000LL);
} 

/**
* Reports the total hours represented by the timespan.
* 
* @relates          kTimeSpan
* @param     span   Time span.
* @return           Hours represented by the timespan.
*/
kInlineFx(k64f) kTimeSpan_ToHours64f(kTimeSpan span)
{
    return (k64f)span / (60.0 * 60.0 * 1000000.0);
} 

/**
* Reports the total minutes represented by the timespan.
* 
* @relates          kTimeSpan
* @param     span   Time span.
* @return           Total minutes represented by the timespan.
*/
kInlineFx(k64s) kTimeSpan_ToMinutes64s(kTimeSpan span)
{
    return span / (60LL * 1000000LL);
} 

/**
* Reports the total minutes represented by the timespan.
* 
* @relates          kTimeSpan
* @param     span   Time span.
* @return           Minutes represented by the timespan.
*/
kInlineFx(k64f) kTimeSpan_ToMinutes64f(kTimeSpan span)
{
    return (k64f)span / (60.0 * 1000000.0);
} 

/**
* Reports the total seconds represented by the timespan.
* 
* @relates          kTimeSpan
* @param     span   Time span.
* @return           Total seconds represented by the timespan.
*/
kInlineFx(k64s) kTimeSpan_ToSeconds64s(kTimeSpan span)
{
    return span / 1000000LL;
} 

/**
* Reports the total seconds represented by the timespan.
* 
* @relates          kTimeSpan
* @param     span   Time span.
* @return           Seconds represented by the timespan.
*/
kInlineFx(k64f) kTimeSpan_ToSeconds64f(kTimeSpan span)
{
    return (k64f)span / 1000000.0;
} 

/**
* Reports the total milliseconds represented by the timespan.
* 
* @relates          kTimeSpan
* @param     span   Time span.
* @return           Total milliseconds represented by the timespan.
*/
kInlineFx(k64s) kTimeSpan_ToMilliseconds64s(kTimeSpan span)
{
    return span / 1000LL;
} 

/**
* Reports the total milliseconds represented by the timespan.
* 
* @relates          kTimeSpan
* @param     span   Time span.
* @return           Milliseconds represented by the timespan.
*/
kInlineFx(k64f) kTimeSpan_ToMilliseconds64f(kTimeSpan span)
{
    return (k64f)span / 1000.0;
} 

/**
* Reports the total microseconds represented by the timespan.
* 
* @relates          kTimeSpan
* @param     span   Time span.
* @return           Total microseonds represented by the timespan.
*/
kInlineFx(k64s) kTimeSpan_ToMicroseconds64s(kTimeSpan span)
{
    return span;
} 

/**
* Reports the total microseconds represented by the timespan.
* 
* @relates          kTimeSpan
* @param     span   Time span.
* @return           Total microseonds represented by the timespan.
*/
kInlineFx(k64f) kTimeSpan_ToMicroseconds64f(kTimeSpan span)
{
    return (k64f)span;
} 

/**
* Formats a time span value as a string.
* 
* @relates                  kTimeSpan
* @param     span           Time span.
* @param     format         Format specification.
* @param     text           Receives formatted time string.
* @param     capacity       Capacity of text buffer.  
* @return                   Operation status. 
*/
kFx(kStatus) kTimeSpan_Format(kTimeSpan span, kTimeSpanFormat format, kChar* text, kSize capacity); 

/**
* Parses a time string that was formatted with kTimeSpan_Format.
* 
* This time-string parser is not tolerant of format variations. 
* 
* @relates                  kTimeSpan
* @param     span           Receives parsed time span value.
* @param     format         Format specification (or kTIME_SPAN_FORMAT_NULL, to attempt to determine automatically). 
* @param     text           Time-span string to be parsed.
* @return                   Operation status. 
*/
kFx(kStatus) kTimeSpan_Parse(kTimeSpan* span, kTimeSpanFormat format, const kChar* text); 

#endif

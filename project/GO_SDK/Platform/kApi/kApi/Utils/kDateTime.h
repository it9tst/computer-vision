/** 
 * @file    kDateTime.h
 * @brief   Declares the kDateTime type. 
 *
 * @internal
 * Copyright (C) 2008-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_DATE_TIME_H
#define K_API_DATE_TIME_H

#include <kApi/kApiDef.h>

/**
 * @struct  kDateTimeFormat
 * @extends kValue
 * @ingroup kApi-Utils  
 * @brief   Represents a date/time text format. 
 */
typedef k32s kDateTimeFormat; 

/** @relates kDateTimeFormat @{ */
#define kDATE_TIME_FORMAT_NULL               (0)         ///< Unknown format value. 
#define kDATE_TIME_FORMAT_FULL               (1)         ///< Complete representation; recommended for most purposes. E.g., 2000-01-01 00:00:00.000000 UTC-04:00. 
#define kDATE_TIME_FORMAT_SIMPLE             (2)         ///< Similar to Full, but omits fractional seconds. E.g., 2000-01-01 00:00:00 UTC-04:00
#define kDATE_TIME_FORMAT_LOCAL              (3)         ///< Similar to Simple, but omits time zone information (not recommended). E.g., 2000-01-01 00:00:00
#define kDATE_TIME_FORMAT_ISO_8601           (4)         ///< ISO-8601 format, with 6 digits for fractional second. E.g., 2000-01-01T00:00:00.000000-04:00 
/** @} */

/**
 * @struct  kDateTime
 * @extends kValue
 * @ingroup kApi-Utils  
 * @brief   Represents UTC calendar date/time. 
 * 
 * kDateTime is a type alias for a 64-bit signed integer that represents the number of elapsed microseconds 
 * since 00:00:00 Jan 1, 1 CE, UTC, excluding leap seconds.
 * 
 * Methods are provided for converting to/from broken-down local time. Support is also provided to 
 * format/parse date-time values to/from a small number of different text representations. 
 *
 * @code {.c}
 * 
 * kDateTime now = kDateTime_Now(); 
 * kText64 nowText; 
 * 
 * kCheck(kDateTime_Format(now, kDATE_TIME_FORMAT_FULL, nowText, sizeof(nowText)));
 * 
 * kLogf("Current date-time: %s", nowText);
 * 
 * kDateTime oneWeekLater = now + kTimeSpan_FromDays64s(7); 
 * k32s monthOneWeekFromNow; 
 * 
 * kCheck(kDateTime_LocalParts(oneWeekLater, kNULL, &monthOneWeekFromNow, kNULL, kNULL, kNULL, kNULL, kNULL, kNULL, kNULL));
 * 
 * kLogf("Month, one week from now: %d", monthOneWeekFromNow);
 * 
 * @endcode
 *
 * kDateTime supports the kdat6 serialization protocol.
 * 
 * kDateTime overrides kValue_Equals and kValue_HashCode.
 */
//typedef k64s kDateTime;                   --forward-declared in kApiDef.x.h 

//A few well-known epochs, expressed as kDateTime values.
#define kDATE_TIME_EPOCH_UNIX               (0x00DCBFFEFF2BC000)                ///< Microseconds from 00:00:00 Jan 1, 1 CE to 00:00:00 Jan 1, 1970 CE, excluding leap seconds.
#define kDATE_TIME_EPOCH_NTP                (0x00D4E6EEB66F2000)                ///< Microseconds from 00:00:00 Jan 1, 1 CE to 00:00:00 Jan 1, 1900 CE, excluding leap seconds.
#define kDATE_TIME_EPOCH_WINDOWS            (0x00B36168B6A58000)                ///< Microseconds from 00:00:00 Jan 1, 1 CE to 00:00:00 Jan 1, 1601 CE, excluding leap seconds.

#include <kApi/Utils/kDateTime.x.h>

/**
* Gets the current calendar date-time.
* 
* The precision of the reported date-time value depends on the resolution of the underlying 
* platform calendar date-time service.
* 
* For platforms that do not support calendar date-time, this method will typically 
* report the uptime relative to the Unix epoch (midnight Jan 1 1970 UTC).
* 
* @relates    kDateTime
* @return     Current calendar date/time. 
*/
kFx(kDateTime) kDateTime_Now(); 

/**
* Gets the broken-down local time associated with a date-time value.
* 
* This method relies on underlying platform support for broken-down time conversions. For platforms
* without timezone support, the local time zone is assumed to equal UTC.
* 
* @relates                  kDateTime
* @param     dateTime       Date-time value.
* @param     year           Optionally receives year.
* @param     month          Optionally receives month (1-12).
* @param     day            Optionally receives day (1-31).
* @param     dayOfWeek      Optionally receives day of week (days since Sunday, 0-6).
* @param     hour           Optionally receives hour (0-23).
* @param     minute         Optionally receives minute (0-59).
* @param     second         Optionally receives second (0-59).
* @param     microsecond    Optionally receives microsecond (0-999999).
* @param     utcOffset      Optionally receives offset to UTC, in minutes.
* @return                   Operation status. 
*/
kFx(kStatus) kDateTime_LocalParts(kDateTime dateTime, k32s* year, k32s* month, k32s* day, k32s* dayOfWeek, k32s* hour, k32s* minute, k32s* second, k32s* microsecond, k32s* utcOffset); 

/**
* Gets the date-time value associated with the specified broken-down local time parts.
* 
* This method relies on underlying platform support for broken-down time conversions. For platforms
* without timezone support, the local time zone is assumed to equal UTC. 
* 
* @relates                  kDateTime
* @param     dateTime       Recieves date-time value.
* @param     year           Year value.
* @param     month          Month value (1-12).
* @param     day            Day value (1-31).
* @param     hour           Hour value (0-23).
* @param     minute         Minute value (0-59).
* @param     second         Second value (0-59).
* @param     microsecond    Microsecond value (0-999999).
* @param     utcOffset      Optionally specifies offset to UTC, in minutes (k32S_NULL for offset based on specified time in local time zone).  
* @return                   Operation status. 
*/
kFx(kStatus) kDateTime_FromLocalParts(kDateTime* dateTime, k32s year, k32s month, k32s day, k32s hour, k32s minute, k32s second, k32s microsecond, k32s utcOffset); 

/**
* Formats a date-time value as a string.
* 
* @relates                  kDateTime
* @param     dateTime       Date-time value.
* @param     format         Format specification.
* @param     text           Receives formatted time string.
* @param     capacity       Capacity of text buffer.  
* @return                   Operation status. 
*/
kFx(kStatus) kDateTime_Format(kDateTime dateTime, kDateTimeFormat format, kChar* text, kSize capacity); 

/**
* Parses a time string that was formatted with kDateTime_Format.
* 
* This time-string parser is not tolerant of format variations. E.g., the ISO-8601 format allows
* for several format variations, but kDateTime_Parse only tolerates the exact variant emitted by 
* kDateTime_Format.
* 
* @relates                  kDateTime
* @param     dateTime       Receives parsed date-time value.
* @param     format         Format specification (or kDATE_TIME_FORMAT_NULL, to attempt to determine automatically). 
* @param     text           Date-time string to be parsed.
* @return                   Operation status. 
*/
kFx(kStatus) kDateTime_Parse(kDateTime* dateTime, kDateTimeFormat format, const kChar* text); 

#endif

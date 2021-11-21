/** 
 * @file    kDateTime.cpp
 *
 * @internal
 * Copyright (C) 2008-2021 by LMI Technologies Inc
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#define K_PLATFORM
#include <kApi/Utils/kDateTime.h>

#include <kApi/Io/kSerializer.h>
#include <kApi/Threads/kTimer.h>
#include <kApi/Threads/kLock.h>

#include <stdio.h>
#include <time.h>

/*
* kDateTimeFormat enum
*/

kBeginEnumEx(k, kDateTimeFormat)
    kAddEnumerator(kDateTimeFormat, kDATE_TIME_FORMAT_FULL)
    kAddEnumerator(kDateTimeFormat, kDATE_TIME_FORMAT_SIMPLE)
    kAddEnumerator(kDateTimeFormat, kDATE_TIME_FORMAT_LOCAL)
    kAddEnumerator(kDateTimeFormat, kDATE_TIME_FORMAT_ISO_8601)
kEndEnumEx()

/*
* xkDateTimeManager static class
*/

kBeginStaticClassEx(k, xkDateTimeManager)
kEndStaticClassEx()

kFx(kStatus) xxkDateTimeManager_InitStatic()
{
    kStaticObj(xkDateTimeManager); 

    //lock required for plaforms without thread-safe calendar date-time API
    kCheck(kLock_ConstructEx(&sobj->lock, xkLOCK_OPTION_PRIORITY_INHERITANCE, kNULL));

    return kOK;
}

kFx(kStatus) xxkDateTimeManager_ReleaseStatic()
{
    kStaticObj(xkDateTimeManager); 

    kCheck(kDestroyRef(&sobj->lock));

    return kOK;
}

/*
* kDateTime structure
*/

kBeginValueEx(k, kDateTime)
    kAddFlags(kDateTime, kTYPE_FLAGS_PRIMITIVE)

    kAddPrivateVersionEx(kDateTime, "kdat6", "7.0.0.0", "kDateTime-0", Write, Read)

    kAddPrivateVMethod(kDateTime, kValue, VEquals)
    kAddPrivateVMethod(kDateTime, kValue, VHashCode)
kEndValueEx()

kFx(kBool) xkDateTime_VEquals(kType type, const void* value, const void* other)
{
    return *(kDateTime*)value == *(kDateTime*)other; 
}

kFx(kSize) xkDateTime_VHashCode(kType type, const void* value)
{
    return xk64s_VHashCode(kTypeOf(k64s), value);
}

kFx(kStatus) xkDateTime_Write(kType type, const void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Write64sArray(serializer, (const k64s*) values, count); 
}

kFx(kStatus) xkDateTime_Read(kType type, void* values, kSize count, kSerializer serializer)
{
    return kSerializer_Read64sArray(serializer, (k64s*) values, count); 
}

kFx(kDateTime) kDateTime_Now()
{
    return kApiLib_DateTimeQueryHandler()();
}

kFx(kStatus) kDateTime_Format(kDateTime dateTime, kDateTimeFormat format, kChar* text, kSize capacity)
{
    k32s year = 0, month = 0, day = 0, dayOfWeek = 0, hour = 0, minute = 0, second = 0, microsecond = 0, utcOffset = 0; 

    kCheck(kDateTime_LocalParts(dateTime, &year, &month, &day, &dayOfWeek, &hour, &minute, &second, &microsecond, &utcOffset)); 

    kChar utcSign = (utcOffset >= 0) ? '+' : '-';
    k32s utcHour = kAbs_(utcOffset) / 60;
    k32s utcMinute = kAbs_(utcOffset) % 60;

    switch (format)
    {
        case kDATE_TIME_FORMAT_NULL:
        case kDATE_TIME_FORMAT_FULL:
            return kStrPrintf(text, capacity, "%04d-%02d-%02d %02d:%02d:%02d.%06d UTC%c%d:%02d", 
                              year, month, day, hour, minute, second, microsecond, utcSign, utcHour, utcMinute);
        case kDATE_TIME_FORMAT_SIMPLE:
            return kStrPrintf(text, capacity, "%04d-%02d-%02d %02d:%02d:%02d UTC%c%d:%02d", 
                              year, month, day, hour, minute, second, utcSign, utcHour, utcMinute);
        case kDATE_TIME_FORMAT_LOCAL:
            return kStrPrintf(text, capacity, "%04d-%02d-%02d %02d:%02d:%02d", 
                              year, month, day, hour, minute, second);
        case kDATE_TIME_FORMAT_ISO_8601:
            return kStrPrintf(text, capacity, "%04d-%02d-%02dT%02d:%02d:%02d.%06d%c%02d:%02d", 
                              year, month, day, hour, minute, second, microsecond, utcSign, utcHour, utcMinute);
        default:
            return kERROR_PARAMETER;
    }
}

kFx(kStatus) kDateTime_Parse(kDateTime* dateTime, kDateTimeFormat format, const kChar* text)
{
    k32s year = 0, month = 0, day = 0, hour = 0, minute = 0, second = 0, microsecond = 0, utcOffsetHours = 0, utcOffetMinutes = 0; 
    kChar utcOffsetSign = '\0';

    switch (format)
    {
        case kDATE_TIME_FORMAT_FULL:
            kCheckTrue(sscanf(text, "%04d-%02d-%02d %02d:%02d:%02d.%06d UTC%c%d:%02d", 
                       &year, &month, &day, &hour, &minute, &second, &microsecond, 
                       &utcOffsetSign, &utcOffsetHours, &utcOffetMinutes) == 10, kERROR_FORMAT); 
            break;
        case kDATE_TIME_FORMAT_SIMPLE:
            kCheckTrue(sscanf(text, "%04d-%02d-%02d %02d:%02d:%02d UTC%c%d:%02d", 
                       &year, &month, &day, &hour, &minute, &second, 
                       &utcOffsetSign, &utcOffsetHours, &utcOffetMinutes) == 9, kERROR_FORMAT); 
            break;
        case kDATE_TIME_FORMAT_LOCAL:
            kCheckTrue(sscanf(text, "%04d-%02d-%02d %02d:%02d:%02d", 
                       &year, &month, &day, &hour, &minute, &second) == 6, kERROR_FORMAT); 
            break;
        case kDATE_TIME_FORMAT_ISO_8601:
            kCheckTrue(sscanf(text, "%04d-%02d-%02dT%02d:%02d:%02d.%06d%c%02d:%02d", 
                       &year, &month, &day, &hour, &minute, &second, &microsecond, 
                       &utcOffsetSign, &utcOffsetHours, &utcOffetMinutes) == 10, kERROR_FORMAT); 
            break;
        default:
            //try all known formats, from most detailed to least detailed
            return (kSuccess(kDateTime_Parse(dateTime, kDATE_TIME_FORMAT_FULL, text)) ||
                    kSuccess(kDateTime_Parse(dateTime, kDATE_TIME_FORMAT_ISO_8601, text)) ||
                    kSuccess(kDateTime_Parse(dateTime, kDATE_TIME_FORMAT_SIMPLE, text)) ||
                    kSuccess(kDateTime_Parse(dateTime, kDATE_TIME_FORMAT_LOCAL, text))) ? kOK : kERROR_FORMAT;
    }

    k32s utcOffset = (utcOffsetSign == '\0') ? k32S_NULL : (((utcOffsetSign == '+') ? 1 : -1)*(utcOffsetHours*60 + utcOffetMinutes));

    return kDateTime_FromLocalParts(dateTime, year, month, day, hour, minute, second, microsecond, utcOffset);
}

#if defined(K_WINDOWS)

kFx(k64s) xkDateTime_DefaultNow()
{
    FILETIME utcFileTime;
    k64s utcWindowsTicks; 

    GetSystemTimeAsFileTime(&utcFileTime); 
    
    utcWindowsTicks  = (k64s)utcFileTime.dwHighDateTime << 32; 
    utcWindowsTicks |= (k64s)utcFileTime.dwLowDateTime; 

    return kDATE_TIME_EPOCH_WINDOWS + utcWindowsTicks/10; 
}

kFx(kStatus) kDateTime_LocalParts(kDateTime dateTime, k32s* year, k32s* month, k32s* day, k32s* dayOfWeek, k32s* hour, k32s* minute, k32s* second, k32s* microsecond, k32s* utcOffset)
{
    k64s utcWindowsTicks = 0;
    FILETIME utcFileTime; 
    SYSTEMTIME utcSystemTime; 
    SYSTEMTIME localSystemTime; 

    if (dateTime > kDATE_TIME_EPOCH_WINDOWS)
    {
        utcWindowsTicks = 10*((dateTime - kDATE_TIME_EPOCH_WINDOWS) / 1000000 * 1000000);  //seconds only; handle microseconds below
    }
    
    utcFileTime.dwHighDateTime = (DWORD) ((utcWindowsTicks >> 32) & 0xFFFFFFFF); 
    utcFileTime.dwLowDateTime = (DWORD) (utcWindowsTicks & 0xFFFFFFFF); 
    
    //convert from file system time to broken-down utc
    kCheckTrue(FileTimeToSystemTime(&utcFileTime, &utcSystemTime) != 0, kERROR_OS);
    
    //convert from broken down utc to broken down local
    kCheckTrue(SystemTimeToTzSpecificLocalTime(kNULL, &utcSystemTime, &localSystemTime) != 0, kERROR_OS);

    if (!kIsNull(year))        *year = localSystemTime.wYear; 
    if (!kIsNull(month))       *month = localSystemTime.wMonth; 
    if (!kIsNull(day))         *day = localSystemTime.wDay; 
    if (!kIsNull(dayOfWeek))   *dayOfWeek = localSystemTime.wDayOfWeek;
    if (!kIsNull(hour))        *hour = localSystemTime.wHour; 
    if (!kIsNull(minute))      *minute = localSystemTime.wMinute; 
    if (!kIsNull(second))      *second = localSystemTime.wSecond; 
    if (!kIsNull(microsecond)) *microsecond = dateTime % 1000000;   //microseconds

    if (!kIsNull(utcOffset))   
    {
        FILETIME localFileTime; 
        
        //convert from broken down local time to file system local time 
        kCheckTrue(SystemTimeToFileTime(&localSystemTime, &localFileTime) != 0, kERROR_OS);

        k64s localWindowsTicks = ((k64s)localFileTime.dwHighDateTime << 32) | localFileTime.dwLowDateTime;
         
        //utc offset is the difference beteen local ticks and utc ticks, converted to minutes
        *utcOffset = (k32s) ((localWindowsTicks - utcWindowsTicks) / (60LL*1000000*10));
    }

    return kOK;
}

kFx(kStatus) kDateTime_FromLocalParts(kDateTime* dateTime, k32s year, k32s month, k32s day, k32s hour, k32s minute, k32s second, k32s microsecond, k32s utcOffset)
{
    SYSTEMTIME systemTime; 
    SYSTEMTIME utcSystemTime; 
    FILETIME utcFileTime; 
    k64s utcWindowsTicks; 
    k64s result;

    systemTime.wYear = (WORD)(year); 
    systemTime.wMonth= (WORD)(month); 
    systemTime.wDay = (WORD)(day); 
    systemTime.wHour = (WORD)(hour); 
    systemTime.wMinute = (WORD)(minute); 
    systemTime.wSecond= (WORD)(second); 
    systemTime.wMilliseconds = 0;    //microseconds handled below

    if (utcOffset != k32S_NULL)
    { 
        //specific UTC offset provided by caller; convert broken down local to file system local time
        //without accounting for timezone; caller-specified utc offset will be subtracted below
        kCheckTrue(SystemTimeToFileTime(&systemTime, &utcFileTime) != 0, kERROR_OS); 
    }
    else
    {
        //convert from broken down local time to broken down UTC
        kCheckTrue(TzSpecificLocalTimeToSystemTime(kNULL, &systemTime, &utcSystemTime) != 0, kERROR_OS);

        //convert from broken-down UTC to file system UTC
        kCheckTrue(SystemTimeToFileTime(&utcSystemTime, &utcFileTime) != 0, kERROR_OS); 
    }

    utcWindowsTicks = (k64s)utcFileTime.dwHighDateTime << 32; 
    utcWindowsTicks |= (k64s)utcFileTime.dwLowDateTime; 

    result = utcWindowsTicks/10 + microsecond + kDATE_TIME_EPOCH_WINDOWS;

    //subtract caller-specified UTC offset
    if (utcOffset != k32S_NULL)
    {
        result -= utcOffset * 60LL * 1000000;
    }

    *dateTime = result;

    return kOK;
}

#elif defined (K_VX_KERNEL)

kFx(k64s) xkDateTime_DefaultNow()
{
    struct timeval tv;

    if (gettimeofday(&tv, NULL) != 0)
    {
        return 0; 
    }

    return kDATE_TIME_EPOCH_UNIX + 1000000ll*tv.tv_sec + tv.tv_usec;   
}

kFx(kStatus) kDateTime_LocalParts(kDateTime dateTime, k32s* year, k32s* month, k32s* day, k32s* dayOfWeek, k32s* hour, k32s* minute, k32s* second, k32s* microsecond, k32s* utcOffset)
{
    time_t rawTime = 0;
    struct tm localTimeInfo;

    if (dateTime > kDATE_TIME_EPOCH_UNIX)
    {
        rawTime = (dateTime - kDATE_TIME_EPOCH_UNIX) / 1000000;   //seconds
    }

    //no timezone support; convert utc timestamp to broken down utc time
    kCheckTrue(!kIsNull(localtime_r(&rawTime, &localTimeInfo)), kERROR_OS);

    if (!kIsNull(year))            *year = localTimeInfo.tm_year + 1900;
    if (!kIsNull(month))           *month = localTimeInfo.tm_mon + 1;
    if (!kIsNull(day))             *day = localTimeInfo.tm_mday;
    if (!kIsNull(dayOfWeek))       *dayOfWeek = localTimeInfo.tm_wday;
    if (!kIsNull(hour))            *hour = localTimeInfo.tm_hour;
    if (!kIsNull(minute))          *minute = localTimeInfo.tm_min;
    if (!kIsNull(second))          *second = localTimeInfo.tm_sec;
    if (!kIsNull(microsecond))     *microsecond = dateTime % 1000000;
    if (!kIsNull(utcOffset))       *utcOffset = 0; 

    return kOK;
}

kFx(kStatus) kDateTime_FromLocalParts(kDateTime* dateTime, k32s year, k32s month, k32s day, k32s hour, k32s minute, k32s second, k32s microsecond, k32s utcOffset)
{
    struct tm localTimeInfo;
    k64s utc;
    k64s result; 
    
    localTimeInfo.tm_sec = second;
    localTimeInfo.tm_min = minute;
    localTimeInfo.tm_hour = hour;
    localTimeInfo.tm_mday = day;
    localTimeInfo.tm_mon = month - 1;
    localTimeInfo.tm_year = year - 1900;
    localTimeInfo.tm_isdst = 0;   //no timezone support

    //no timezone support; convert broken down UTC time to UTC timestamp
    kCheckTrue((utc = mktime(&localTimeInfo)) >= 0, kERROR_OS); 

    result = (utc  * 1000000) + microsecond + kDATE_TIME_EPOCH_UNIX;

    //subtract caller-specified UTC offset
    if (utcOffset != k32S_NULL)
    {
        result -= utcOffset * 60LL * 1000000;
    }

    *dateTime = result;
    
    return kOK;
}

#elif defined(K_TI_BIOS)

kFx(k64s) xkDateTime_DefaultNow()
{
    return kDATE_TIME_EPOCH_UNIX + kTimer_Now();
}

kFx(kStatus) kDateTime_LocalParts(kDateTime dateTime, k32s* year, k32s* month, k32s* day, k32s* dayOfWeek, k32s* hour, k32s* minute, k32s* second, k32s* microsecond, k32s* utcOffset)
{
    kStaticObj(xkDateTimeManager); 

    //lock required: C localtime is not thread-safe
    kLock_Enter(sobj->lock); 

    kTry
    {
        time_t rawTime = 0;

        if (dateTime > kDATE_TIME_EPOCH_UNIX)
        {
            rawTime = (dateTime - kDATE_TIME_EPOCH_UNIX) / 1000000;   //seconds
        }

        //no timezone support; convert utc timestamp to broken down utc time
        struct tm* localTimeInfo = localtime(&rawTime);

        kTestTrue(!kIsNull(localTimeInfo), kERROR_OS);

        if (!kIsNull(year))            *year = localTimeInfo->tm_year + 1970;   //epoch from 1970 instead of 1900
        if (!kIsNull(month))           *month = localTimeInfo->tm_mon + 1;
        if (!kIsNull(day))             *day = localTimeInfo->tm_mday;
        if (!kIsNull(dayOfWeek))       *dayOfWeek = localTimeInfo->tm_wday;
        if (!kIsNull(hour))            *hour = localTimeInfo->tm_hour;
        if (!kIsNull(minute))          *minute = localTimeInfo->tm_min;
        if (!kIsNull(second))          *second = localTimeInfo->tm_sec;
        if (!kIsNull(microsecond))     *microsecond = dateTime % 1000000;
        if (!kIsNull(utcOffset))       *utcOffset = 0; 
    }
    kFinally
    {
        kLock_Exit(sobj->lock);

        kEndFinally();
    }

    return kOK;
}

kFx(kStatus) kDateTime_FromLocalParts(kDateTime* dateTime, k32s year, k32s month, k32s day, k32s hour, k32s minute, k32s second, k32s microsecond, k32s utcOffset)
{
    kStaticObj(xkDateTimeManager); 

    //lock required: C mktime may not be thread-safe on this platform (unknown)
    kLock_Enter(sobj->lock); 

    kTry
    {
        struct tm localTimeInfo;
        k64s utc;
        k64s result; 
        
        localTimeInfo.tm_sec = second;
        localTimeInfo.tm_min = minute;
        localTimeInfo.tm_hour = hour;
        localTimeInfo.tm_mday = day;
        localTimeInfo.tm_mon = month - 1;
        localTimeInfo.tm_year = year - 1970;     //epoch from 1970 instead of 1900
        localTimeInfo.tm_isdst = 0;   //no timezone support

        //no timezone support; convert broken down UTC time to UTC timestamp
        kTestTrue((utc = mktime(&localTimeInfo)) >= 0, kERROR_OS); 

        result = (utc  * 1000000) + microsecond + kDATE_TIME_EPOCH_UNIX;

        //subtract caller-specified UTC offset
        if (utcOffset != k32S_NULL)
        {
            result -= utcOffset * 60LL * 1000000;
        }

        *dateTime = result;    
    }
    kFinally
    {
        kLock_Exit(sobj->lock);

        kEndFinally();
    }

    return kOK;
}

#elif defined(K_POSIX)

kFx(k64s) xkDateTime_DefaultNow()
{
    struct timeval tv;

    if (gettimeofday(&tv, NULL) != 0)
    {
        return 0; 
    }

    return kDATE_TIME_EPOCH_UNIX + 1000000ll*tv.tv_sec + tv.tv_usec;   
}

kFx(kStatus) kDateTime_LocalParts(kDateTime dateTime, k32s* year, k32s* month, k32s* day, k32s* dayOfWeek, k32s* hour, k32s* minute, k32s* second, k32s* microsecond, k32s* utcOffset)
{
    time_t rawTime = 0;
    struct tm localTimeInfo;

    if (dateTime > kDATE_TIME_EPOCH_UNIX)
    {
        rawTime = (dateTime - kDATE_TIME_EPOCH_UNIX) / 1000000;   //seconds
    }

    //convert utc timestamp to broken down local time
    kCheckTrue(!kIsNull(localtime_r(&rawTime, &localTimeInfo)), kERROR_OS);

    if (!kIsNull(year))            *year = localTimeInfo.tm_year + 1900;
    if (!kIsNull(month))           *month = localTimeInfo.tm_mon + 1;
    if (!kIsNull(day))             *day = localTimeInfo.tm_mday;
    if (!kIsNull(dayOfWeek))       *dayOfWeek = localTimeInfo.tm_wday;
    if (!kIsNull(hour))            *hour = localTimeInfo.tm_hour;
    if (!kIsNull(minute))          *minute = localTimeInfo.tm_min;
    if (!kIsNull(second))          *second = localTimeInfo.tm_sec;
    if (!kIsNull(microsecond))     *microsecond = dateTime % 1000000;
    if (!kIsNull(utcOffset))       *utcOffset = (k32s)localTimeInfo.tm_gmtoff / 60; 

    return kOK;
}

kFx(kStatus) kDateTime_FromLocalParts(kDateTime* dateTime, k32s year, k32s month, k32s day, k32s hour, k32s minute, k32s second, k32s microsecond, k32s utcOffset)
{
    struct tm localTimeInfo;
    k64s utc;
    k64s result; 
    
    localTimeInfo.tm_sec = second;
    localTimeInfo.tm_min = minute;
    localTimeInfo.tm_hour = hour;
    localTimeInfo.tm_mday = day;
    localTimeInfo.tm_mon = month - 1;
    localTimeInfo.tm_year = year - 1900;
    localTimeInfo.tm_isdst = -1;

    //convert broken down local time to UTC timestamp
    kCheckTrue((utc = mktime(&localTimeInfo)) >= 0, kERROR_OS); 

    result = kDATE_TIME_EPOCH_UNIX + (utc  * 1000000) + microsecond;

    //subtract caller-specified UTC offset from UTC offset in current time zone
    if (utcOffset != k32S_NULL)
    {
        k32s adjustment = localTimeInfo.tm_gmtoff/60 - utcOffset;

        result += adjustment * 60LL * 1000000;
    }

    *dateTime = result;
    
    return kOK;
}

#else

kFx(k64s) xkDateTime_DefaultNow()
{
    return kDATE_TIME_EPOCH_UNIX + kTimer_Now();
}

kFx(kStatus) kDateTime_LocalParts(kDateTime dateTime, k32s* year, k32s* month, k32s* day, k32s* dayOfWeek, k32s* hour, k32s* minute, k32s* second, k32s* microsecond, k32s* utcOffset)
{ 
    return kERROR_UNIMPLEMENTED
}

kFx(kStatus) kDateTime_FromLocalParts(kDateTime* dateTime, k32s year, k32s month, k32s day, k32s hour, k32s minute, k32s second, k32s microsecond, k32s utcOffset)
{ 
    return kERROR_UNIMPLEMENTED
}

#endif

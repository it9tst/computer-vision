/** 
 * @file    kApiLib.cpp
 *
 * @internal
 * Copyright (C) 2012-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#define K_PLATFORM
#include <kApi/kApiLib.h>
#include <kApi/kApi.h>
#include <kApi/Utils/kUtils.h>

/* 
 * Library configuration variables. See comments in kApiLib.x.h.
 */

kDx(kApiMemAllocFx) xkApiLib_memAllocFx = xkDefaultMemAlloc; 
kDx(kApiMemFreeFx) xkApiLib_memFreeFx = xkDefaultMemFree; 
kDx(kPointer) xkApiLib_memAllocProvider = kNULL; 
kDx(kApiMemSetFx) xkApiLib_memSetFx = kNULL; 
kDx(kApiMemCopyFx) xkApiLib_memCopyFx = kNULL; 
kDx(kApiMemCopyFx) xkApiLib_memMoveFx = kNULL; 
kDx(kApiAllocConstructFx) xkApiLib_appAllocConstructFx = xkAlloc_DefaultConstructAppAlloc;
kDx(kApiAllocDestroyFx) xkApiLib_appAllocDestroyFx = xkAlloc_DefaultDestroyAppAlloc;

kDx(kCallback) xkApiLib_earlyLogCallback = { 0, 0 };
kDx(kAtomic32s) xkApiLib_logInitialized = kFALSE; 

kDx(kApiLogfFx) xkApiLib_logfFx = kNULL; 
kDx(kApiTraceFx) xkApiLib_traceFx = kNULL; 
kDx(kApiAssertFx) xkApiLib_assertFx = xkDefaultAssert; 

kDx(kBool) xkApiLib_leakLoggingEnabled = kTRUE; 
kDx(kSize) xkApiLib_leaksDetected = 0; 
kDx(kBool) xkApiLib_checkTraceEnabled = kFALSE; 

kDx(k64u) xkApiLib_timerMultiplier = 1; 
kDx(k64u) xkApiLib_timerDivider = 1; 
kDx(kApiTimerQueryFx) xkApiLib_timerQueryFx = xkApiLib_TimerStub; 

kDx(k64u) xkApiLib_kernelTimerMultiplier = 1000;        /* by default, assume kernel time unit is 1 ms */
kDx(k64u) xkApiLib_kernelTimerDivider = 1; 
kDx(kApiTimerScaleFx) xkApiLib_timerScaleFx = xkApiLib_DefaultTimerScaleFx;

kDx(kSize) xkApiLib_threadStackSize = 0;

kDx(kApiDateTimeQueryFx) xkApiLib_dateTimeQueryFx = xkDateTime_DefaultNow; 

kDx(kApiRandomFx) xkApiLib_randomFx = kNULL; 

kDx(kBool) xkApiLib_hasFileFx = kFALSE;        
kDx(kApiFileFx) xkApiLib_fileFx = { kNULL };                

kDx(kBool) xkApiLib_hasDirectoryFx = kFALSE;        
kDx(kApiDirectoryFx) xkApiLib_directoryFx = { kNULL };      

kDx(kChar) xkApiLib_nativeSeparator = 0;             

kDx(kBool) xkApiLib_networkInitializationEnabled = kTRUE; 

kDx(kApiLockFx) xkApiLib_symbolLockFx = kNULL; 
kDx(kApiUnlockFx) xkApiLib_symbolUnlockFx = kNULL; 
kDx(kPointer) xkApiLib_symbolLockProvider = kNULL;
kDx(kBool) xkApiLib_symbolInitializationEnabled = kTRUE; 

kDx(kApiQueryNetInfoFx) xkApiLib_queryNetInfoFx = xkNetwork_DefaultQueryNetInfo;  
kDx(kApiQueryNetChangeFx) xkApiLib_queryNetChangeFx = xkNetwork_DefaultQueryNetChange;  

/* 
 * Assembly types.
 */

kBeginAssemblyEx(k, kApiLib, kAPI_VERSION, kAPI_VERSION)

    //Interfaces
    kAddType(kCollection)
    kAddType(kCompressor)
    kAddType(kObjectPool)

    //Values
    kAddType(k16u)
    kAddType(k16s)
    kAddType(k32f)
    kAddType(k32u)
    kAddType(k32s)
    kAddType(k64f)
    kAddType(k64u)
    kAddType(k64s)
    kAddType(k8u)
    kAddType(k8s)
    kAddType(kAllocTrait)
    kAddType(kArgb)
    kAddType(kBool)
    kAddType(kByte)
    kAddType(kCallback)   
    kAddType(kCallbackFx)
    kAddType(kCfa)
    kAddType(kChar)
    kAddType(kCipherPadding)
    kAddType(kCipherMode)
    kAddType(kComparison)
    kAddType(kCompressionPreset)
    kAddType(kCompressionType)
    kAddType(kDateTime)
    kAddType(kDateTimeFormat)
    kAddType(xkDat5SerializerTypeInfo)
    kAddType(xkDat6SerializerTypeInfo)
    kAddType(kDebugAllocation)
    kAddType(kEndianness)
    kAddType(kFileMode)
    kAddType(kHttpStatus)
    kAddType(kIpAddress)
    kAddType(kIpEndPoint)
    kAddType(kIpEntry)
    kAddType(kIpVersion)
    kAddType(kLockOption)
    kAddType(kLogArgs)
    kAddType(kLogOption)
    kAddType(kMacAddress)
    kAddType(kMemoryAlignment)
    kAddType(kMsgQueueItemOption)
    kAddType(kMsgQueuePurgeOption)
    kAddType(kParallelArgs)
    kAddType(kPipeStream)
    kAddType(kPixelFormat)
    kAddType(kPoint16s)
    kAddType(kPoint32s)
    kAddType(kPoint32f)
    kAddType(kPoint64f)
    kAddType(kPoint3d16s)
    kAddType(kPoint3d32s)
    kAddType(kPoint3d32f)
    kAddType(kPoint3d64f)
    kAddType(kPoint4d16s)
    kAddType(kPointer)
    kAddType(kProcess)
    kAddType(kRect16s)
    kAddType(kRect32s)
    kAddType(kRect32f)
    kAddType(kRect64f)
    kAddType(kRect3d64f)
    kAddType(kRgb)
    kAddType(kRotatedRect32s)
    kAddType(kRotatedRect32f)
    kAddType(kSeekOrigin)
    kAddType(xkSerializerWriteSection)
    kAddType(kSize)
    kAddType(kSSize)
    kAddType(kSocketType)
    kAddType(kSocketEvent)
    kAddType(kStatus)
    kAddType(kText16)
    kAddType(kText32)
    kAddType(kText64)
    kAddType(kText128)
    kAddType(kText256)
    kAddType(kThreadId)
    kAddType(kThreadPriorityClass)
    kAddType(kTimeSpan)
    kAddType(kTimeSpanFormat)
    kAddType(kValue)
    kAddType(kVersion)
    kAddType(kVoid)
    kAddType(kWebSocketDataType)
    kAddType(xkXmlAttrBlock)
    kAddType(xkXmlItemBlock)
    kAddBytes()

    //Classes
    kAddType(kAlloc)
    kAddType(kApiLog)
    kAddType(kArray1)
    kAddType(kArray2)
    kAddType(kArray3)
    kAddType(kArrayList)
    kAddType(kArrayProvider)
    kAddType(kAssembly)
    kAddType(kAtomic)
    kAddType(kBackTrace)
    kAddType(kBitArray)
    kAddType(kBlowfishCipher)
    kAddType(kBox)
    kAddType(kCipher)
    kAddType(kCipherStream)
    kAddType(kDat6Serializer)
    kAddType(kDat5Serializer)
    kAddType(xkDateTimeManager)
    kAddType(kDebugAlloc)
    kAddType(kDirectory)
    kAddType(kDynamicLib)
    kAddType(kEvent)
    kAddType(kFile)
    kAddType(kHash)
    kAddType(kHttpServer)
    kAddType(kHttpServerChannel)
    kAddType(kHttpServerRequest)
    kAddType(kHttpServerResponse)
    kAddType(kImage)
    kAddType(kList)
    kAddType(kLock)
    kAddType(kMap)
    kAddType(kMath)
    kAddType(kMemory)
    kAddType(kMsgQueue)
    kAddType(kNetwork)
    kAddType(kNetworkAdapter)
    kAddType(kNetworkInfo)
    kAddType(kNetworkInterface)
    kAddType(kObject)
    kAddType(kParallel)
    kAddType(kParallelJob)  
    kAddType(kPath)
    kAddType(kPeriodic)
    kAddType(kPlugin)
    kAddType(kPoolAlloc)
    kAddType(kQueue)
    kAddType(kSemaphore)
    kAddType(kSerializer)
    kAddType(kSha1Hash)
    kAddType(kSocket)
    kAddType(kStream)
    kAddType(kString)
    kAddType(kSymbolInfo)
    kAddType(kTcpClient)
    kAddType(kTcpServer)
    kAddType(kThread)
    kAddType(kThreadPool)
    kAddType(kThreadPoolJob)
    kAddType(kTimer)
    kAddType(kType)
    kAddType(kUdpClient)
    kAddType(kUserAlloc)
    kAddType(kUtils)
    kAddType(kWebSocket)
    kAddType(kXml)

    //Initialization order
    kAddPriority(kAtomic)
    kAddPriority(kUtils)
    kAddPriority(kAlloc)
    kAddPriority(kAssembly)

kEndAssemblyEx()

/* 
 * General library management functions. 
 */

kFx(kStatus) kApiLib_SetMemAllocHandlers(kApiMemAllocFx allocFx, kApiMemFreeFx freeFx, kPointer provider)
{
    xkApiLib_memAllocFx = allocFx; 
    xkApiLib_memFreeFx = freeFx; 
    xkApiLib_memAllocProvider = provider; 

    return kOK;
}

kFx(kStatus) kApiLib_SetMemSetHandler(kApiMemSetFx function)
{
    xkApiLib_memSetFx = function; 

    return kOK; 
}

kFx(kStatus) kApiLib_SetMemCopyHandler(kApiMemCopyFx function)
{
    xkApiLib_memCopyFx = function; 

    return kOK; 
}

kFx(kStatus) kApiLib_SetMemMoveHandler(kApiMemCopyFx function)
{
    xkApiLib_memMoveFx = function; 

    return kOK; 
}

kFx(kStatus) kApiLib_SetAppAllocConstructHandler(kApiAllocConstructFx function)
{
    xkApiLib_appAllocConstructFx = function; 

    return kOK; 
}

kFx(kStatus) kApiLib_SetAppAllocDestroyHandler(kApiAllocDestroyFx function)
{
    xkApiLib_appAllocDestroyFx = function; 

    return kOK; 
}

kFx(kStatus) kApiLib_SetLogfHandler(kApiLogfFx function)
{
    xkApiLib_logfFx = function; 

    return kOK; 
}

kFx(kStatus) kApiLib_SetTraceHandler(kApiTraceFx function)
{
    xkApiLib_traceFx = function; 

    return kOK; 
} 

kFx(kPointer) xkApiLib_CastFailHandler(const kChar* file, k32s line)
{
    kApiAssertFx handler = kApiLib_AssertHandler(); 
    
    if (!kIsNull(handler))
    {        
        handler(file, (k32u)line); 
    }
    else
    {
       abort(); 
    }

    return kNULL; 
}

kFx(kStatus) kApiLib_EnableLeakLogging(kBool enabled)
{
    xkApiLib_leakLoggingEnabled = enabled; 

    return kOK; 
}

kFx(kStatus) kApiLib_SetDebugAllocListener(kCallbackFx function, kPointer receiver)
{
    kAlloc alloc = kAlloc_App(); 

    if (kObject_Is(alloc, kTypeOf(kDebugAlloc)))
    {
        kCheck(kDebugAlloc_SetAllocListener(alloc, function, receiver)); 
    }

    return kOK; 
}

kFx(kStatus) kApiLib_SetAssertHandler(kApiAssertFx function)
{
    xkApiLib_assertFx = function; 

    return kOK; 
}

kFx(kStatus) kApiLib_SetTimerScale(k64u multiplier, k64u divider)
{
    xkApiLib_timerMultiplier = multiplier; 
    xkApiLib_timerDivider = divider; 

    return kOK; 
}

kFx(kStatus) kApiLib_SetTimerQueryHandler(kApiTimerQueryFx function)
{
    xkApiLib_timerQueryFx = function; 

    return kOK; 
}

kFx(kStatus) kApiLib_SetKernelTimerScale(k64u multiplier, k64u divider)
{
    k64u scale = multiplier / divider; 
    k64u remainder = multiplier % divider; 

    xkApiLib_timerScaleFx = xkApiLib_CustomTimerScaleFx;

    if (remainder == 0)
    {
        xkApiLib_kernelTimerMultiplier = scale; 
        xkApiLib_kernelTimerDivider = 1; 

        //optimized for most common case
        if (xkApiLib_kernelTimerMultiplier == 1000)
        {
            xkApiLib_timerScaleFx = xkApiLib_DefaultTimerScaleFx;
        }
    }
    else
    {
        xkApiLib_kernelTimerMultiplier = multiplier;
        xkApiLib_kernelTimerDivider = divider;
    }

    return kOK; 
}

//optimized for most common case, in which timer scale is 1000 (1 kernel tick per ms)
kFx(k32u) xkApiLib_DefaultTimerScaleFx(k64u time)
{
    if (time == kINFINITE)
    {
        return kOS_INFINITE;        
    }
    else
    {
        k64u ticks = time / 1000;
        k64u truncatedTime = 1000 * ticks; 

        return (k32u) ((truncatedTime == time) ? ticks : ticks + 1);
    }    
}

kFx(k32u) xkApiLib_CustomTimerScaleFx(k64u time)
{
    if (time == kINFINITE)
    {
        return kOS_INFINITE;        
    }
    else
    {
        k64u ticks = kApiLib_KernelTimerDivider()*(time) / kApiLib_KernelTimerMultiplier(); 
        k64u truncatedTime =  kApiLib_KernelTimerMultiplier()*(ticks) / kApiLib_KernelTimerDivider();   

        return (k32u) ((truncatedTime == time) ? ticks : ticks + 1);
    }
}

kFx(kStatus) kApiLib_SetDefaultStackSize(kSize stackSize)
{
    xkApiLib_threadStackSize = stackSize;

    return kOK;
}

kFx(kStatus) kApiLib_SetDateTimeQueryHandler(kApiDateTimeQueryFx function)
{
    xkApiLib_dateTimeQueryFx = kIsNull(function) ? xkDateTime_DefaultNow : function; 

    return kOK; 
}

kFx(kStatus) kApiLib_SetRandomHandler(kApiRandomFx function)
{
    xkApiLib_randomFx = function; 

    return kOK; 
}

kFx(kStatus) kApiLib_SetFileHandlers(kApiFileFx* functions)
{
    if (!xkApiLib_hasFileFx)
    {
        kZero(xkApiLib_fileFx); 
        xkApiLib_hasFileFx = kTRUE; 
    }

    return xkOverrideFunctions(&xkApiLib_fileFx, sizeof(xkApiLib_fileFx), functions); 
}

kFx(kStatus) kApiLib_SetDirectoryHandlers(kApiDirectoryFx* functions)
{
    if (!xkApiLib_hasDirectoryFx)
    {
        kZero(xkApiLib_directoryFx); 
        xkApiLib_hasDirectoryFx = kTRUE; 
    }

    return xkOverrideFunctions(&xkApiLib_directoryFx, sizeof(xkApiLib_directoryFx), functions); 
}

kFx(kStatus) kApiLib_SetNativeSeparator(kChar separator)
{
    xkApiLib_nativeSeparator = separator; 

    return kOK; 
}

kFx(kStatus) kApiLib_EnableNetworkInitialization(kBool enable)
{
    xkApiLib_networkInitializationEnabled = enable; 

    return kOK; 
}

kFx(kStatus) kApiLib_SetSymbolLockHandlers(kApiLockFx lockFx, kApiUnlockFx unlockFx, kPointer provider)
{
    xkApiLib_symbolLockFx = lockFx; 
    xkApiLib_symbolUnlockFx = unlockFx; 
    xkApiLib_symbolLockProvider = provider; 

    return kOK; 
}

kFx(kStatus) kApiLib_EnableSymbolInitialization(kBool enable)
{
    xkApiLib_symbolInitializationEnabled = enable; 

    return kOK; 
}

kFx(kBool) kApiLib_HasTimerQueryHandler()
{ 
    return xkApiLib_timerQueryFx != xkApiLib_TimerStub; 
} 

kFx(k64u) xkApiLib_TimerStub()
{
    return 0; 
}

kFx(kStatus) kApiLib_SetQueryNetInfoHandler(kApiQueryNetInfoFx function)
{
    xkApiLib_queryNetInfoFx = function;
    
    return kOK; 
}

kFx(kStatus) kApiLib_SetQueryNetChangeHandler(kApiQueryNetChangeFx function)
{
    xkApiLib_queryNetChangeFx = function;
    
    return kOK; 
}

/* 
* Logging infrastructure
*/

typedef struct kApiLogStatic
{
    kLock lock;                      //Provides exclusive acccess.
    kEvent listeners;                //List of log handlers. -- kEvent<kCallback<kApiLogFx>>
    kEvent formatters;               //List of log formatters. -- kEvent<kCallback<kApiLogFormatterFx>>
    kString formatterContent[2];     //Temporary strings, used for message formatting.
} kApiLogStatic;

kDeclareStaticClassEx(k, kApiLog)

kFx(kStatus) xkApiLog_InitStatic();
kFx(kStatus) xkApiLog_ReleaseStatic();

kBeginStaticClassEx(k, kApiLog)
kEndStaticClassEx()

kFx(kStatus) xkApiLog_InitStatic()
{
    kStaticObj(kApiLog); 
    kAlloc allocator = kAlloc_App(); 
    kStatus status; 

    kTry
    {
        kTest(kLock_ConstructEx(&sobj->lock, xkLOCK_OPTION_PRIORITY_INHERITANCE, allocator));

        kTest(kEvent_Construct(&sobj->listeners, allocator));
        kTest(kEvent_Construct(&sobj->formatters, allocator));

        kTest(kString_Construct(&sobj->formatterContent[0], "", allocator));
        kTest(kString_Construct(&sobj->formatterContent[1], "", allocator));

        if (!kIsNull(xkApiLib_earlyLogCallback.function))
        {
            kEvent_Add(sobj->listeners, xkApiLib_earlyLogCallback.function, xkApiLib_earlyLogCallback.receiver);
        }

        //there shouldn't be any multithreaded access at this point; using atomic out of abundance of caution
        kAtomic32s_Exchange(&xkApiLib_logInitialized, kTRUE);
    }
    kCatch(&status)
    {
        xkApiLog_ReleaseStatic();

        kEndCatch(status);
    }

    return kOK; 
}

kFx(kStatus) xkApiLog_ReleaseStatic()
{
    kStaticObj(kApiLog); 

    //there shouldn't be any multithreaded access at this point; using atomic out of abundance of caution
    kAtomic32s_Exchange(&xkApiLib_logInitialized, kFALSE); 

    kCheck(kDestroyRef(&sobj->formatterContent[0])); 
    kCheck(kDestroyRef(&sobj->formatterContent[1])); 
    kCheck(kDestroyRef(&sobj->formatters)); 
    kCheck(kDestroyRef(&sobj->listeners)); 
    kCheck(kDestroyRef(&sobj->lock)); 

    return kOK; 
}

kFx(kStatus) kApiLib_AddLogHandler(kApiLogFx function, kPointer receiver)
{    
    if (!kAtomic32s_Get(&xkApiLib_logInitialized))
    {
        kCheckState(kIsNull(xkApiLib_earlyLogCallback.function)); 

        xkApiLib_earlyLogCallback.function = (kCallbackFx) function; 
        xkApiLib_earlyLogCallback.receiver = receiver; 
    }
    else
    {
        kStaticObj(kApiLog); 
        
        kLock_Enter(sobj->lock);

        kTry
        {
            kTest(kEvent_Add(sobj->listeners, (kCallbackFx) function, receiver));
        }
        kFinally
        {
            kLock_Exit(sobj->lock);
            kEndFinally();
        }
    }

    return kOK; 
}

kFx(kStatus) kApiLib_RemoveLogHandler(kApiLogFx function, kPointer receiver)
{    
    if (!kAtomic32s_Get(&xkApiLib_logInitialized))
    {
        if ((xkApiLib_earlyLogCallback.function == (kCallbackFx)function) && (xkApiLib_earlyLogCallback.receiver == receiver))
        {
            xkApiLib_earlyLogCallback.function = kNULL; 
            xkApiLib_earlyLogCallback.receiver = kNULL; 
        }
    }
    else
    {
        kStaticObj(kApiLog); 
        
        kLock_Enter(sobj->lock);

        kTry
        {
            kTest(kEvent_Remove(sobj->listeners, (kCallbackFx)function, receiver));

            if ((xkApiLib_earlyLogCallback.function == (kCallbackFx)function) && (xkApiLib_earlyLogCallback.receiver == receiver))
            {
                xkApiLib_earlyLogCallback.function = kNULL; 
                xkApiLib_earlyLogCallback.receiver = kNULL; 
            }
        }
        kFinally
        {
            kLock_Exit(sobj->lock);
            kEndFinally();
        }
    }

    return kOK; 
}

kFx(kStatus) kApiLib_AddLogFormatter(kApiLogFormatterFx function, kPointer receiver)
{    
    kCheckState(kAtomic32s_Get(&xkApiLib_logInitialized));

    {
        kStaticObj(kApiLog); 

        kLock_Enter(sobj->lock);

        kTry
        {
            kTest(kEvent_Add(sobj->formatters, (kCallbackFx)function, receiver));
        }
        kFinally
        {
            kLock_Exit(sobj->lock);
            kEndFinally();
        }
    }

    return kOK; 
}

kFx(kStatus) kApiLib_RemoveLogFormatter(kApiLogFormatterFx function, kPointer receiver)
{    
    kCheckState(kAtomic32s_Get(&xkApiLib_logInitialized));

    {
        kStaticObj(kApiLog); 

        kLock_Enter(sobj->lock);

        kTry
        {
            kTest(kEvent_Remove(sobj->formatters, (kCallbackFx)function, receiver));
        }
        kFinally
        {
            kLock_Exit(sobj->lock);
            kEndFinally();
        }
    }

    return kOK; 
}

kFx(kStatus) kApiLib_InvokeLegacyLogHandler(const kChar* format, ...)
{
    if (!kIsNull(xkApiLib_logfFx))
    {
        kVarArgList args; 

        kVarArgList_Start(args, format); 
        {
            xkApiLib_logfFx(format, args);
        }
        kVarArgList_End(args); 
    }

    return kOK; 
}

kFx(kStatus) kApiLib_InvokeEarlyLogHandler(const kLogArgs* args)
{
    kApiLogFx function = (kApiLogFx)xkApiLib_earlyLogCallback.function;

    if (!kIsNull(function))
    {
        function(xkApiLib_earlyLogCallback.receiver, args);
    }

    return kOK; 
}

kFx(const kChar*) kApiLib_InvokeLogFormatters(const kChar* message)
{
    kStaticObj(kApiLog); 
    kList formatterList = kEvent_Listeners(sobj->formatters);
    kListItem it = kList_First(formatterList);
    kSize formatContentIndex = 0;
 
    //copy to buffer
    kString_Set(sobj->formatterContent[0], message);

    //apply any formatters
    while (!kIsNull(it))
    {
        const kCallback* formatter = kList_AtT(formatterList, it, kCallback);
        kApiLogFormatterFx function = (kApiLogFormatterFx)formatter->function;
        kString input = sobj->formatterContent[formatContentIndex % 2];
        kString output = sobj->formatterContent[(formatContentIndex + 1) % 2];

        kString_Clear(output);

        function(formatter->receiver, input, output);

        it = kList_Next(formatterList, it);
        formatContentIndex++;
    }

    //return the final, formatted result
    return kString_Chars(sobj->formatterContent[formatContentIndex%2]);
}

kFx(kStatus) kApiLib_InvokeLogListeners(const kLogArgs* args)
{
    kStaticObj(kApiLog); 
    kList listenerList = kEvent_Listeners(sobj->listeners); 
    kListItem it = kList_First(listenerList);

    while (!kIsNull(it))
    {
        const kCallback* listener = kList_AtT(listenerList, it, kCallback); 
        kApiLogFx function = (kApiLogFx) listener->function;
    
        function(listener->receiver, args); 
    
        it = kList_Next(listenerList, it);
    }

    return kOK; 
}

kFx(kStatus) xkApiLib_Log(const kLogArgs* args)
{
    //legacy support -- call the legacy log hander, if one exists
    kApiLib_InvokeLegacyLogHandler("%s", args->message);

    if (!kAtomic32s_Get(&xkApiLib_logInitialized))
    {        
        //if log service not yet initialized, call early log handler (if registered)
        kApiLib_InvokeEarlyLogHandler(args);
    }
    else
    {
        kStaticObj(kApiLog); 

        kLock_Enter(sobj->lock);
        {
            if (kEvent_Count(sobj->formatters) == 0)
            {
                kApiLib_InvokeLogListeners(args);
            }
            else
            {
                kLogArgs argsFormatted = { 0 };

                //call formatters, providing opportunity to transform buffer
                const kChar* formattedMessage =  kApiLib_InvokeLogFormatters(args->message);
        
                argsFormatted.message = formattedMessage;
                argsFormatted.options = args->options;
                argsFormatted.source = args->source;
                argsFormatted.dateTime = args->dateTime;
                argsFormatted.upTime = args->upTime;

                //notify listeners with final formatted content
                kApiLib_InvokeLogListeners(&argsFormatted);
            }
            
        }
        kLock_Exit(sobj->lock);
    }

    return kOK;
}

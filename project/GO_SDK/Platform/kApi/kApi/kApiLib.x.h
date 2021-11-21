/** 
 * @file    kApiLib.x.h
 *
 * @internal
 * Copyright (C) 2012-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_API_LIB_X_H
#define K_API_API_LIB_X_H

kDeclareAssemblyEx(k, kApiLib)

/** Function signature for memory set handler. */
typedef kStatus (kCall* kApiMemSetFx)(void* dest, kByte fill, kSize size); 

/** Function signature for memory copy/move handler.  */
typedef kStatus (kCall* kApiMemCopyFx)(void* dest, const void* src, kSize size); 

/** Function signature for application allocator construction handler.  */
typedef kStatus (kCall* kApiAllocConstructFx)(kAlloc* appAlloc, kAlloc systemAlloc); 

/** Function signature for application allocator destroy handler.  */
typedef kStatus (kCall* kApiAllocDestroyFx)(kAlloc appAlloc); 

/** Function signature for log formatting handler. */
typedef kStatus(kCall* kApiLogFormatterFx)(kPointer receiver, kString input, kString output);

/** Function signature for debug trace handler. */
typedef kStatus (kCall* kApiTraceFx)(const kChar* tag, const kChar* file, k32u line); 

/** Function signature for timer tick query handler; returns ticks. */
typedef k64u (kCall* kApiTimerQueryFx)(); 

/** Function signature for timer tick conversion handler; returns ticks. */
typedef k32u (kCall* kApiTimerScaleFx)(k64u time); 

/** Function signature for calendar time query handler; returns elapsed microseconds since 00:00:00 Jan 1, 1 CE, UTC, excluding leap seconds. */
typedef k64s (kCall* kApiDateTimeQueryFx)(); 

/** Function signature for random number generator. */
typedef k32u (kCall* kApiRandomFx)(); 

/** Function signature to query information about local network configuration. */
typedef kStatus (kCall* kApiQueryNetInfoFx)(kNetworkInfo netInfo); 

/** Function signature to query information about local network changes. */
typedef kBool (kCall* kApiQueryNetChangeFx)(k64u timeout); 

/**
 * @struct  kApiFileFx
 * @ingroup kApi  
 * @brief   Collection of callbacks for file operations.
 * @see     kApiLib_SetFileHandlers
 */
typedef struct kApiFileFx
{
    kStatus (kCall* open)(kFile file, const kChar* path, kFileMode mode); 
    kStatus (kCall* close)(kFile file); 
    kStatus (kCall* read)(kFile file, kByte* buffer, kSize minCount, kSize maxCount, kSize* bytesRead); 
    kStatus (kCall* write)(kFile file, const kByte* buffer, kSize count); 
    kStatus (kCall* flush)(kFile file); 
    kStatus (kCall* seek)(kFile file, k64s offset, kSeekOrigin origin, k64u* position); 
    k64u (kCall* size)(const kChar* path);
    kBool (kCall* exists)(const kChar* path);
    kStatus (kCall* copy)(const kChar* source, const kChar* destination, kCallbackFx progress, kPointer context);
    kStatus (kCall* move)(const kChar* source, const kChar* destination, kCallbackFx progress, kPointer context);
    kStatus (kCall* del)(const kChar* path);
    kStatus (kCall* tempName)(kChar* name, kSize capacity);
    kStatus (kCall* tempFile)(const kChar* path, kChar* name, kSize capacity);
} kApiFileFx; 

/**
 * @struct  kApiDirectoryFx
 * @ingroup kApi  
 * @brief   Collection of callbacks for directory and path operations.
 * @see     kApiLib_SetDirectoryHandlers
 */
typedef struct kApiDirectoryFx
{
    kStatus (kCall* create)(const kChar* path);
    kBool (kCall* exists)(const kChar* path);
    kStatus (kCall* move)(const kChar* source, const kChar* destination);
    kStatus (kCall* del)(const kChar* path); 
    kStatus (kCall* enumerate)(const kChar* directory, kBool includeFiles, kBool includeDirectories, kArrayList entries); 
    kStatus (kCall* setCurrent)(const kChar* directory); 
    kStatus (kCall* current)(kChar* directory, kSize capacity); 
    kStatus (kCall* appDirectory)(kChar* path, kSize capacity);
    kStatus (kCall* apiDirectory)(kChar* directory, kSize capacity);
    kStatus (kCall* tempDirectory)(kChar* path, kSize capacity);
    kStatus (kCall* appConfigDirectory)(const kChar* appName, kChar* path, kSize capacity);
    kStatus (kCall* appDataDirectory)(const kChar* appName, kChar* path, kSize capacity);
    kStatus (kCall* toVirtual)(const kChar* path, kChar* vpath, kSize capacity);
    kStatus (kCall* fromVirtual)(const kChar* vpath, kChar* path, kSize capacity);
} kApiDirectoryFx; 

/**
 * @struct  kApiAtomicFx
 * @ingroup kApi  
 * @brief   Collection of callbacks for atomic variable operations.
 * @see     kApiLib_SetAtomicHandlers
 */
typedef struct kApiAtomicFx
{
    k32s (kCall* increment32s)(kAtomic32s* atomic); 
    k32s (kCall* decrement32s)(kAtomic32s* atomic); 
    k32s (kCall* exchange32s)(kAtomic32s* atomic, k32s value); 
    kBool (kCall* compareExchange32s)(kAtomic32s* atomic, k32s oldValue, k32s value); 
    k32s (kCall* get32s)(kAtomic32s* atomic); 
    kPointer (kCall* exchangePointer)(kAtomicPointer* atomic, kPointer value); 
    kBool (kCall* compareExchangePointer)(kAtomicPointer* atomic, kPointer oldValue, kPointer value); 
    kPointer (kCall* getPointer)(kAtomicPointer* atomic); 
} kApiAtomicFx; 

/* 
 * Private library configuration variables. Use the functions and macros provided in this module
 * to access/modify these variables (do not manipulate directly).
 */

kExtern kDx(kApiMemAllocFx) xkApiLib_memAllocFx;                  ///< User-provided callback function for memory allocation.
kExtern kDx(kApiMemFreeFx) xkApiLib_memFreeFx;                    ///< User-provided callback function for memory deallocation.
kExtern kDx(kPointer) xkApiLib_memAllocProvider;                  ///< User-provided context pointer for memory allocation/deallocation.
kExtern kDx(kApiMemSetFx) xkApiLib_memSetFx;                      ///< User-provided callback function for memory initialization.
kExtern kDx(kApiMemCopyFx) xkApiLib_memCopyFx;                    ///< User-provided callback function for memory copy.
kExtern kDx(kApiMemCopyFx) xkApiLib_memMoveFx;                    ///< User-provided callback function for memory move.
kExtern kDx(kApiAllocConstructFx) xkApiLib_appAllocConstructFx;   ///< User-provided callback function for application allocator construction.
kExtern kDx(kApiAllocDestroyFx) xkApiLib_appAllocDestroyFx;       ///< User-provided callback function for application allocator destruction.

kExtern kDx(kApiLogfFx) xkApiLib_logfFx;                     ///< User-provided callback function for formatted logging operations. 
kExtern kDx(kApiTraceFx) xkApiLib_traceFx;                   ///< User-provided callback function for trace operations.
kExtern kDx(kApiAssertFx) xkApiLib_assertFx;                 ///< User-provided callback function for assertions.

kExtern kDx(kBool) xkApiLib_leakLoggingEnabled;              ///< User-configurable option to enable memory leak logging.
kExtern kDx(kSize) xkApiLib_leaksDetected;                   ///< Leak status, available after kApi assembly is destroyed. 
kExtern kDx(kBool) xkApiLib_checkTraceEnabled;               ///< User-configurable option to enable check-trace feature.

kExtern kDx(k64u) xkApiLib_timerMultiplier;                  ///< User-provided constant for high-resolution timer multiplier (us = mult*ticks/div).
kExtern kDx(k64u) xkApiLib_timerDivider;                     ///< User-provided constant for high-resolution timer divider (us = mult*ticks/div).
kExtern kDx(kApiTimerQueryFx) xkApiLib_timerQueryFx;         ///< User-provided callback function for high-resolution timer queries.

kExtern kDx(k64u) xkApiLib_kernelTimerMultiplier;            ///< User-provided constant for kernel clock multiplier (us = mult*ticks/div).
kExtern kDx(k64u) xkApiLib_kernelTimerDivider;               ///< User-provided constant for kernel clock divider (us = mult*ticks/div).
kExtern kDx(kApiTimerScaleFx) xkApiLib_timerScaleFx;         ///< Function to convert time to kernel ticks.

kExtern kDx(kSize) xkApiLib_threadStackSize;                 ///< User-provided constant for thread stack size.

kExtern kDx(kApiDateTimeQueryFx) xkApiLib_dateTimeQueryFx;   ///< User-provided callback function for calendar date-time queries.

kExtern kDx(kApiRandomFx) xkApiLib_randomFx;                 ///< User-provided callback function for random number generation.

kExtern kDx(kBool) xkApiLib_hasFileFx;                       ///< Flag to indicate whether file callbacks have been provided.
kExtern kDx(kApiFileFx) xkApiLib_fileFx;                     ///< User-provided callback functions for file operations.

kExtern kDx(kBool) xkApiLib_hasDirectoryFx;                  ///< Flag to indicate whether directory callbacks have been provided.
kExtern kDx(kApiDirectoryFx) xkApiLib_directoryFx;           ///< User-provided callback functions for directory operations.

kExtern kDx(kChar) xkApiLib_nativeSeparator;                 ///< Native path separator for underlying system.

kExtern kDx(kBool) xkApiLib_networkInitializationEnabled;    ///< Should kApiLib initialize the network interface library?

kExtern kDx(kApiLockFx) xkApiLib_symbolLockFx;               ///< User-provided callback function for symbol table lock. 
kExtern kDx(kApiUnlockFx) xkApiLib_symbolUnlockFx;           ///< User-provided callback function for symbol table unlock. 
kExtern kDx(kPointer) xkApiLib_symbolLockProvider;           ///< User-provided context pointer for symbol table lock/unlock. 
kExtern kDx(kBool) xkApiLib_symbolInitializationEnabled;     ///< Should kApiLib initialize the back trace library?

kExtern kDx(kApiQueryNetInfoFx) xkApiLib_queryNetInfoFx;     ///< User-provided callback function to query local network configuration information. 
kExtern kDx(kApiQueryNetChangeFx) xkApiLib_queryNetChangeFx; ///< User-provided callback function to query local network configuration changes. 

/*
 * More library configuration functions, typically only used on embedded platforms. 
 * 
 */

/** 
 * Sets a handler function for memory copy operations. 
 *
 * By default, kApi uses C standard library functions for memory copy operations. 
 * Call this function to override the default behaviour. 
 * 
 * This function is not thread-safe.
 *
 * @param   function    Memory copy callback function.
 * @return              Operation status. 
 */
kFx(kStatus) kApiLib_SetMemCopyHandler(kApiMemCopyFx function); 

/** 
 * Sets a handler function for memory move operations. 
 *
 * By default, kApi uses C standard library functions for memory move operations. 
 * Call this function to override the default behaviour. 
 * 
 * This function is not thread-safe.
 *
 * @param   function    Memory move callback function.
 * @return              Operation status. 
 */
kFx(kStatus) kApiLib_SetMemMoveHandler(kApiMemCopyFx function); 

/** 
 * Sets a handler to construct the application allocator object. 
 *
 * This function is not thread-safe.
 *
 * @param   function    Allocation construction function.
 * @return              Operation status. 
 */
kFx(kStatus) kApiLib_SetAppAllocConstructHandler(kApiAllocConstructFx function); 

/** 
 * Sets a handler to destroy the application allocator object. 
 *
 * This function is not thread-safe.
 *
 * @param   function    Allocation destruction function.
 * @return              Operation status. 
 */
kFx(kStatus) kApiLib_SetAppAllocDestroyHandler(kApiAllocDestroyFx function); 

/** 
 * Sets a handler function for trace operations. 
 *
 * By default, trace operations are ignored (trace information discarded). 
 * Call this function to override the default behaviour. 
 * 
 * This function is not thread-safe.
 *
 * @param   function    Trace callback function.
 * @return              Operation status. 
 */
kFx(kStatus) kApiLib_SetTraceHandler(kApiTraceFx function); 

/** 
 * Enables or disables leak logging.
 *
 * By default, memory leaks are sent to the kApi log handler. This function can be used
 * to disable leak logging.
 * 
 * Note, leak logging also require K_DEBUG to be defined at compile time.
 * 
 * This function is not thread-safe.
 *
 * @param   enabled     Specifies whether leak-logging is enabled.
 * @return              Operation status. 
 */
kFx(kStatus) kApiLib_EnableLeakLogging(kBool enabled); 

/** 
 * Sets a listener function that will be notified on debug heap allocations.
 *
 * The 'args' parameter passed to the listener function will be a pointer to 
 * a kDebugAllocation structure. This feature can be used while debugging to break 
 * on a particular allocation index, without modifying code inside the kApi library.
 * This technique can be helpful in tracking down heap leaks and corruptions. 
 * 
 * This function should be called after constructing the kApi assembly.
 * 
 * Note, debug allocation notification requires K_DEBUG to be defined at compile 
 * time.
 *
 * @param   function    Debug memory allocation listener function.
 * @param   receiver    Context argument to be passed to listener. 
 * @return              Operation status. 
 */
kFx(kStatus) kApiLib_SetDebugAllocListener(kCallbackFx function, kPointer receiver); 

/** 
 * Sets the scale for high-precision timer calculations.
 *
 * By default, kApi uses platform-specific library functions for high-resolution
 * timers. Use this function in conjunction with kApiLib_SetTimerQueryHandler to override 
 * the default behaviour.
 * 
 * Timer calculations use the following relationship: time = ticks * multiplier / divider.
 * Multiplier and divider values should be selected to produce a time value in microseconds, 
 * and without causing 64-bit integer overflow. Ticks are determined by querying the callback 
 * provided to kApiLib_SetTimerQueryHandler.  
 * 
 * This function is not thread-safe.
 *
 * @param   multiplier  Tick multiplier for timer calculations.
 * @param   divider     Tick divider for timer calculations.
 * @return              Operation status. 
 * @see                 kApiLib_SetTimerQueryHandler
 */
kFx(kStatus) kApiLib_SetTimerScale(k64u multiplier, k64u divider); 

/** 
 * Sets a handler function for timer queries. 
 *
 * By default, kApi uses platform-specific library functions for high-resolution
 * timers. Use this function in conjunction with kApiLib_SetTimerScale to override 
 * the default behaviour.
 * 
 * This function is not thread-safe.
 *
 * @param   function    Memory allocation callback function.
 * @return              Operation status. 
 * @see                 kApiLib_SetTimerScale
 */
kFx(kStatus) kApiLib_SetTimerQueryHandler(kApiTimerQueryFx function); 

/** 
 * Sets the scale for kernel time calculations.
 *
 * Time calculations use the following relationship: time = ticks * multiplier / divider.
 * Multiplier and divider values should be selected to produce a time value in microseconds, 
 * and without causing 64-bit integer overflow. 
 * 
 * This function is not thread-safe.
 *
 * @param   multiplier  Tick multiplier for kernel timer calculations.
 * @param   divider     Tick divider for kernel timer calculations.
 * @return              Operation status. 
 */
kFx(kStatus) kApiLib_SetKernelTimerScale(k64u multiplier, k64u divider); 

/** 
 * Sets the default stack size when creating new threads. 
 *
 * By default, kApi uses the platform-specific default behavior for thread stack sizes.
 * Use this function to override the default stack size.
 *
 * This function is only implemented for Linux. It is not thread-safe.
 *
 * @param   stackSize   Thread stack size (or 0 to restore default).
 * @return              Operation status. 
 */
kFx(kStatus) kApiLib_SetDefaultStackSize(kSize stackSize);

/** 
 * Sets a handler function for calendar date-time queries. 
 *
 * By default, kApi uses platform-specific library functions to determine the current
 * calendar date-time. Use this function to override the default behaviour.
 * 
 * This function is not thread-safe.
 *
 * @param   function    Calendar date-time query function (or kNULL to restore default).
 * @return              Operation status. 
 */
kFx(kStatus) kApiLib_SetDateTimeQueryHandler(kApiDateTimeQueryFx function); 

/** 
 * Sets a handler function for generating random numbers.
 *
 * By default, kApi uses C standard library functions for random number generation.
 * Call this function to override the default behaviour. 
 * 
 * This function is not thread-safe.
 *
 * @param   function    Random number generation callback function.
 * @return              Operation status. 
 */
kFx(kStatus) kApiLib_SetRandomHandler(kApiRandomFx function); 

/** 
 * Sets handler functions for file operations.
 *
 * By default, kApi uses platform-specific functions for file operations.
 * However, some embedded platforms do not provide a standard file interface. On these systems, 
 * this function can be used to provide an interface to a custom/ad-hoc file system.
 * 
 * This function is not thread-safe.
 *
 * @param   functions   File callback functions. 
 * @return              Operation status. 
 */
kFx(kStatus) kApiLib_SetFileHandlers(kApiFileFx* functions); 

/** 
 * Sets handler functions for directory operations.
 *
 * By default, kApi uses platform-specific functions for directory operations.
 * However, some embedded platforms do not provide a standard directory interface. 
 * On these systems, this function can be used to provide an interface to a 
 * custom/ad-hoc directory system.
 * 
 * This function is not thread-safe.
 *
 * @param   functions   Directory callback functions. 
 * @return              Operation status. 
 */
kFx(kStatus) kApiLib_SetDirectoryHandlers(kApiDirectoryFx* functions); 

/** 
 * Sets native path separator used by underlying operating system.
 *
 * This function is not thread-safe.
 *
 * @param   separator   Native separator. 
 * @return              Operation status. 
 */
kFx(kStatus) kApiLib_SetNativeSeparator(kChar separator); 

/**
 * Adds a handler function for log formatting operations.
 *
 * This method is currently intended for internal use only. 
 * 
 * Log formatters can be installed to transform log messages before they are dispatched
 * to log handlers. 
 * 
 * Note that log formatting callbacks are invoked while an internal log lock is held. Accordingly, 
 * caution should be exercised in log formatters to avoid potential deadlock. E.g., if the 
 * internal log lock is considered lock A, and the log formatter acquires lock B, the application 
 * should take care to ensure that these locks are never acquired in the opposite order 
 * (i.e., B, then A). This could happen if application code acquires lock B and then calls
 * a logging function such as kLogf. 
 * 
 * @public              @memberof kApiLib
 * @param   function    Log formatter function.
 * @param   receiver    Log callback context pointer. 
 * @return              Operation status.
 */
kFx(kStatus) kApiLib_AddLogFormatter(kApiLogFormatterFx function, kPointer receiver);

/**
 * Removes a handler function for logging formatting operations.
 *
 * This method is currently intended for internal use only. 
 * 
 * @public              @memberof kApiLib
 * @param   function    Log formatter function.
 * @param   receiver    Log formatter context pointer. 
 * @return              Operation status.
 */
kFx(kStatus) kApiLib_RemoveLogFormatter(kApiLogFormatterFx function, kPointer receiver);

/** 
 * Sets a handler function for querying local network configuration information. 
 *
 * This function is not thread-safe.
 *
 * @param   function    Network information query function.
 * @return              Operation status. 
 */
kFx(kStatus) kApiLib_SetQueryNetInfoHandler(kApiQueryNetInfoFx function); 

/** 
 * Sets a handler function for querying local network configuration changes. 
 *
 * This function is not thread-safe.
 *
 * @param   function    Network change query function.
 * @return              Operation status. 
 */
kFx(kStatus) kApiLib_SetQueryNetChangeHandler(kApiQueryNetChangeFx function); 


kInlineFx(kApiMemAllocFx) kApiLib_MemAllocHandler()                 { return xkApiLib_memAllocFx; }                          ///< Gets user-provided callback function for system memory allocation.
kInlineFx(kApiMemFreeFx) kApiLib_MemFreeHandler()                   { return xkApiLib_memFreeFx; }                           ///< Gets user-provided callback function for system memory deallocation.
kInlineFx(kPointer) kApiLib_MemAllocProvider()                      { return xkApiLib_memAllocProvider; }                    ///< Gets user-provided context pointer for memory allocation/deallocation.
kInlineFx(kApiMemSetFx) kApiLib_MemSetHandler()                     { return xkApiLib_memSetFx; }                            ///< Gets user-provided callback function for memory initialization.
kInlineFx(kApiMemCopyFx) kApiLib_MemCopyHandler()                   { return xkApiLib_memCopyFx; }                           ///< Gets user-provided callback function for memory copy.
kInlineFx(kApiMemCopyFx) kApiLib_MemMoveHandler()                   { return xkApiLib_memMoveFx; }                           ///< Gets user-provided callback function for memory move.
kInlineFx(kApiAllocConstructFx) kApiLib_AppAllocConstructHandler()  { return xkApiLib_appAllocConstructFx; }                 ///< Gets user-provided callback function to construct application allocator.
kInlineFx(kApiAllocDestroyFx) kApiLib_AppAllocDestroyHandler()      { return xkApiLib_appAllocDestroyFx; }                   ///< Gets user-provided callback function to destroy application allocator.

kInlineFx(kApiLogfFx) kApiLib_LogfHandler()                         { return xkApiLib_logfFx; }                              ///< Gets user-provided callback function for formatted logging operations. 
kInlineFx(kApiTraceFx) kApiLib_TraceHandler()                       { return xkApiLib_traceFx; }                             ///< Gets user-provided callback function for trace operations.
kInlineFx(kApiAssertFx) kApiLib_AssertHandler()                     { return xkApiLib_assertFx; }                            ///< Gets user-provided callback function for assertions.

kInlineFx(kBool) kApiLib_CheckTraceEnabled()                        { return xkApiLib_checkTraceEnabled; }                   ///< Gets user-configurable option to enable check-trace feature.
kInlineFx(kBool) kApiLib_LeakLoggingEnabled()                       { return xkApiLib_leakLoggingEnabled; }                  ///< Gets user-configurable option to enable leak-logging feature.
kInlineFx(kSize) kApiLib_LeaksDetected()                            { return xkApiLib_leaksDetected; }                       ///< Reports the number of bytes leaked (available after kApiLib teardown).
kInlineFx(void) kApiLib_SetLeaksDetected(kSize leakedMem)           { xkApiLib_leaksDetected = leakedMem; }                  ///< Used within the kApi library to set the leaks-detected flag (not thread-safe).

kInlineFx(k64u) kApiLib_TimerMultiplier()                           { return xkApiLib_timerMultiplier; }                     ///< Gets user-provided constant for high-resolution timer multiplier.
kInlineFx(k64u) kApiLib_TimerDivider()                              { return xkApiLib_timerDivider; }                        ///< Gets user-provided constant for high-resolution timer divider.
kInlineFx(kApiTimerQueryFx) kApiLib_TimerQueryHandler()             { return xkApiLib_timerQueryFx; }                        ///< Gets user-provided callback function for timer queries.
kFx(kBool) kApiLib_HasTimerQueryHandler();                                                                                   ///< Gets user-provided callback function for timer queries.

kInlineFx(kApiDateTimeQueryFx) kApiLib_DateTimeQueryHandler()       { return xkApiLib_dateTimeQueryFx; }                     ///< Gets user-provided callback function for calendar date-time queries.

kInlineFx(k64u) kApiLib_KernelTimerMultiplier()                     { return xkApiLib_kernelTimerMultiplier; }               ///< Gets user-provided constant for kernel timer multiplier.
kInlineFx(k64u) kApiLib_KernelTimerDivider()                        { return xkApiLib_kernelTimerDivider; }                  ///< Gets user-provided constant for kernel timer divider.

kInlineFx(kSize) kApiLib_DefaultStackSize()                         { return xkApiLib_threadStackSize; }                     ///< Gets user-provided constant for thread stack size.

kInlineFx(kApiRandomFx) kApiLib_RandomHandler()                     { return xkApiLib_randomFx; }                            ///< Gets user-provided callback function for random number generation.

kInlineFx(kBool) kApiLib_HasFileHandlers()                          { return xkApiLib_hasFileFx; }                           ///< Gets flag to indicate whether file callbacks have been provided.
kInlineFx(kApiFileFx*) kApiLib_FileHandlers()                       { return &xkApiLib_fileFx; }                             ///< Gets user-provided callback functions for file operations.

kInlineFx(kBool) kApiLib_HasDirectoryHandlers()                     { return xkApiLib_hasDirectoryFx; }                      ///< Gets flag to indicate whether directory callbacks have been provided.
kInlineFx(kApiDirectoryFx*) kApiLib_DirectoryHandlers()             { return &xkApiLib_directoryFx; }                        ///< Gets user-provided callback functions for directory operations.

kInlineFx(kChar) kApiLib_NativeSeparator()                          { return xkApiLib_nativeSeparator; }                     ///< Gets native separator used by underlying operating system.

kInlineFx(kBool) kApiLib_NetworkInitializationEnabled()             { return xkApiLib_networkInitializationEnabled; }        ///< Reports whether network stack should be initialized by this library.

kInlineFx(kApiLockFx) kApiLib_SymbolLockHandler()                   { return xkApiLib_symbolLockFx; }                        ///< Gets user-provided callback function for symbol table lock operation.
kInlineFx(kApiUnlockFx) kApiLib_SymbolUnlockHandler()               { return xkApiLib_symbolUnlockFx; }                      ///< Gets user-provided callback function for symbol table unlock operation.
kInlineFx(kPointer) kApiLib_SymbolLockProvider()                    { return xkApiLib_symbolLockProvider; }                  ///< Gets user-provided context pointer for symbol table lock/unlock.

kInlineFx(kBool) kApiLib_SymbolInitializationEnabled()              { return xkApiLib_symbolInitializationEnabled; }         ///< Reports whether symbol services should be be initialized by this library.

kInlineFx(kApiQueryNetInfoFx) kApiLib_QueryNetInfoHandler()         { return xkApiLib_queryNetInfoFx; }                      ///< Gets user-provided callback function for querying network information.
kInlineFx(kApiQueryNetChangeFx) kApiLib_QueryNetChangeHandler()     { return xkApiLib_queryNetChangeFx; }                    ///< Gets user-provided callback function for querying network changes.

/** 
 * Private method that raises an assertion (or aborts) in the event of an object cast failure.
 *
 * This utility function is used by kApi type-checking macros; it is not intended to be 
 * called directly. 
 *
 * @param   file    Source file name in which cast failure occurred.
 * @param   line    Line number at which cast failure occurred.
 * @return          Returns kNull. 
 */
kFx(kPointer) xkApiLib_CastFailHandler(const kChar* file, k32s line);

/** Private method that provides default stub for timer tick calls. */
kFx(k64u) xkApiLib_TimerStub();

/** Private method that converts time to kernel ticks; optimized for 1000 ticks/sec. */
kFx(k32u) xkApiLib_DefaultTimerScaleFx(k64u time);

/** Private method that converts time to kernel ticks. */
kFx(k32u) xkApiLib_CustomTimerScaleFx(k64u time);

/** Private method that dispatches a message to log handers. */
kFx(kStatus) xkApiLib_Log(const kLogArgs* args);

/* 
* Deprecated (Stage 1): not recommended for further use, but not yet announced via kDeprecate
*/

//Deprecated: This feature will eventually be removed.
kInlineFx(kStatus) kApiLib_EnableCheckTrace(kBool enabled)
{
    xkApiLib_checkTraceEnabled = enabled; 

    return kOK; 
}

/**
 * [Deprecated] Sets a handler function for debug logging operations.
 *
 * NOTE: This function is being phased out but will continue to work as it does today. This is being
 * replaced by kApiLib_AddLogHandler and kApiLib_RemoveLogHandler.
 *
 * By default, logging operations are ignored (log information discarded). Call this function to override 
 * the default behaviour.
 * 
 * This function is not thread-safe and should be called before calling kApiLib_Construct.
 *
 * @public              @memberof kApiLib
 * @param   function    Log callback function.
 * @return              Operation status.
 */
kFx(kStatus) kApiLib_SetLogfHandler(kApiLogfFx function);

#endif


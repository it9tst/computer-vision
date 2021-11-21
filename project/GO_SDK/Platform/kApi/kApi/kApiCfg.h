/** 
 * @file    kApiCfg.h
 * @brief   Architecture/compiler-specific definitions.
 * 
 * @internal
 * Copyright (C) 2005-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_API_CFG_H
#define K_API_API_CFG_H

/* Detect the compiler family; fall back to GCC as default. */
#if defined(_MSC_VER)
#   define K_MSVC
#elif defined(__TI_COMPILER_VERSION__)
#   define K_TI_CCS
#else
#   define K_GCC
#endif

/* Helpful constants that can be compared with K_CPP_VERSION (defined below). */
#define K_CPP_VERSION_1998    (199711L)
#define K_CPP_VERSION_2011    (201103L)
#define K_CPP_VERSION_2014    (201402L)
#define K_CPP_VERSION_2017    (201703L)

/* 
* Detect language; K_CPP is defined only if the compiler is a C++ compiler; K_CPP_VERSION is 
* always defined, but will have a definition of -1 if the compiler is a C compiler. */
#if defined(__cplusplus)
    
#   if defined(_MSVC_LANG)
#       define K_CPP    _MSVC_LANG
#   else 
#       define K_CPP    __cplusplus
#   endif

#   if (K_CPP < K_CPP_VERSION_1998)
#       undef  K_CPP
#       define K_CPP K_CPP_VERSION_1998
#   endif

#   define K_CPP_VERSION    K_CPP

#else

#   define K_CPP_VERSION    -1

#endif

#if defined(__cplusplus_cli)
#   define K_CPP_CLI
#endif

/* Detect the OS; fall back to non-specific POSIX as default. */
#if defined(_WIN32) || defined(_WIN64)
#   define K_WINDOWS
#elif defined(__TMS320C6X__)
#   define K_TI_BIOS
#   define K_DSP_BIOS /* old definition, provided for compatibility with FS<=6.1.33.x */
#elif defined(_WRS_KERNEL)
#   define K_VX_KERNEL
#else
#   define K_POSIX
#   if defined(__linux__)
#       define K_LINUX
#   endif
#   if defined(__APPLE__)
#       define K_DARWIN
#   endif
#   if defined(__QNXNTO__)
#       define K_QNX
#   endif
#endif

/* Code profiling always enabled in debug builds. */
#if defined(K_DEBUG)
#   define K_PROFILE
#endif

#if defined(K_DEBUG)
#   define K_DEBUG_ENABLED          (1)
#else
#   define K_DEBUG_ENABLED          (0)
#endif

#if defined(K_PROFILE)
#   define K_PROFILE_ENABLED        (1)
#else
#   define K_PROFILE_ENABLED        (0)
#endif

#if (defined(K_DEBUG) || defined(K_ASSERT)) && !defined(K_NO_ASSERT)
#   define K_ASSERT_ENABLED  (1)
#else
#   define K_ASSERT_ENABLED  (0)
#endif

/* Provide shorter symbol to use when checking for Cuda support. */
#if defined(K_HAVE_CUDA)
#   define K_CUDA
#endif

/* 
 * Include some C standard headers that we heavily rely on. This list is subject to change; 
 * dependent code should not assume that these headers will always be included here.
 */
#if defined(K_MSVC) && defined(K_DEBUG)
#   define _CRTDBG_MAP_ALLOC
#   include <stdlib.h>
#   include <crtdbg.h>
#else
#   include <stdlib.h>
#endif

#include <stdarg.h>
#include <stddef.h>
#include <string.h>

#if defined(K_MSVC)
#   include <intrin.h>    
#endif

#if (K_CPP_VERSION >= K_CPP_VERSION_2011)
#   include <type_traits>
#endif

/* Utilty macros for stringizing macro arguments. */
#define xkStringize(X)  #X
#define xkStringizeDefine(X)    xkStringize(X)

/* 
 * Detect pointer size; raise an eror if the pointer size cannot be detected. The behaviour 
 * can be overridden by defining K_POINTER_SIZE as a compiler flag. 
 */
#if !defined(K_POINTER_SIZE)
#   if defined(_WIN64) || defined(WIN64)
#      define K_POINTER_SIZE (8)
#   elif defined(_WIN32) || defined(WIN32)
#      define K_POINTER_SIZE (4)
#   elif defined(__SIZEOF_POINTER__)
#      define K_POINTER_SIZE (__SIZEOF_POINTER__)
#   elif defined(__LP64__) || defined(__LLP64__) || defined(__SILP64__)
#      define K_POINTER_SIZE (8)
#   elif defined(__LP32__) || defined(__ILP32__)
#      define K_POINTER_SIZE (4)
#   elif defined(_TMS320C6X)
#      define K_POINTER_SIZE (4)
#   else
#     error "Pointer size not detected; define K_POINTER_SIZE as compiler flag."
#   endif
#endif

#if   (K_POINTER_SIZE == 8)
#   define K_POINTER_SHIFT   (3)
#elif (K_POINTER_SIZE == 4)
#   define K_POINTER_SHIFT   (2)
#endif

/* 
 * Detect endianness; fall back to little endian as default. The behaviour can be overridden 
 * by defining K_ENDIANNESS as a compiler flag. 
 */
#if !defined(K_ENDIANNESS)
#   if defined(K_GCC) && defined(__BYTE_ORDER__) && (__BYTE_ORDER__ == __ORDER_BIG_ENDIAN__)
#       define K_ENDIANNESS             (kENDIANNESS_BIG)
#   else 
#       define K_ENDIANNESS             (kENDIANNESS_LITTLE)
#   endif
#endif

#if defined(K_CPP)
#   define kExtern  extern "C"
#else
#   define kExtern  extern
#endif

#if defined(K_GCC)
#   define K_ATTRIBUTE_UNUSED       __attribute__((unused))
#else
#   define K_ATTRIBUTE_UNUSED
#endif

/* Define primitive data types and calling conventions. */
#if defined(K_MSVC)

#   define xkCall                           __stdcall
#   define xkDlCall                         __cdecl

#   define kExportFx(TYPE)                  kExtern __declspec(dllexport) TYPE kCall
#   define kImportFx(TYPE)                  kExtern __declspec(dllimport) TYPE kCall

#   define kExportCx(TYPE)                  kExtern __declspec(dllexport) TYPE xkDlCall
#   define kImportCx(TYPE)                  kExtern __declspec(dllimport) TYPE xkDlCall

#   define kInFx(TYPE)                      kExtern TYPE kCall
#   define kInCx(TYPE)                      kExtern TYPE kCall

#   define kExportDx(TYPE)                  __declspec(dllexport) TYPE
#   define kImportDx(TYPE)                  __declspec(dllimport) TYPE

#   define kInDx(TYPE)                      TYPE

#   define kExportClass                     __declspec(dllexport)
#   define kImportClass                     __declspec(dllimport)

#   if _MSC_VER >= 1500
#       define kInline                      __inline
#   else
#       define kInline
#   endif

    typedef unsigned __int8                 xk8u;                       
    typedef unsigned __int16                xk16u;                      
    typedef unsigned __int32                xk32u;                      
    typedef unsigned __int64                xk64u;                      
    typedef __int8                          xk8s;                       
    typedef __int16                         xk16s;                      
    typedef __int32                         xk32s;                      
    typedef __int64                         xk64s;                      
    typedef float                           xk32f;                      
    typedef double                          xk64f;      
    typedef char                            xkChar;                
    typedef unsigned char                   xkByte;                     

#   define xk64U(CONSTANT)                  (CONSTANT##ui64)
#   define xk64S(CONSTANT)                  (CONSTANT##i64)

#elif defined(K_TI_CCS)

#   define xkCall 
#   define xkDlCall

#   define kExportFx(TYPE)                  kExtern TYPE kCall
#   define kImportFx(TYPE)                  kExtern TYPE kCall

#   define kExportCx(TYPE)                  kExtern TYPE kDlCall
#   define kImportCx(TYPE)                  kExtern TYPE kDlCall

#   define kInFx(TYPE)                      kExtern TYPE kCall
#   define kInCx(TYPE)                      kExtern TYPE kCall

#   define kExportDx(TYPE)                  TYPE
#   define kImportDx(TYPE)                  TYPE

#   define kInDx(TYPE)                      TYPE

#   define kExportClass                     
#   define kImportClass                     

#   define kInline                          inline

    typedef unsigned char                   xk8u;                      
    typedef unsigned short                  xk16u;                     
    typedef unsigned int                    xk32u;                     
    typedef unsigned long long              xk64u;                     
    typedef signed char                     xk8s;                      
    typedef signed short                    xk16s;                     
    typedef signed int                      xk32s;                     
    typedef signed long long                xk64s;                     
    typedef float                           xk32f;                     
    typedef double                          xk64f;                     
    typedef char                            xkChar;                
    typedef unsigned char                   xkByte;                    

#   define xk64U(CONSTANT)                  (CONSTANT##LLU)
#   define xk64S(CONSTANT)                  (CONSTANT##LL)

#else

#   define xkCall 
#   define xkDlCall

#   define kExportFx(TYPE)                  kExtern TYPE kCall
#   define kImportFx(TYPE)                  kExtern TYPE kCall

#   define kExportCx(TYPE)                  kExtern TYPE kDlCall
#   define kImportCx(TYPE)                  kExtern TYPE kDlCall

#   define kInFx(TYPE)                      kExtern TYPE kCall
#   define kInCx(TYPE)                      kExtern TYPE kCall

#   define kExportDx(TYPE)                  TYPE
#   define kImportDx(TYPE)                  TYPE

#   define kInDx(TYPE)                      TYPE

#   define kExportClass                     
#   define kImportClass                     

#   define kInline                          inline

    typedef unsigned char                   xk8u;                      
    typedef unsigned short                  xk16u;                     
    typedef unsigned int                    xk32u;                     
    typedef unsigned long long              xk64u;                     
    typedef signed char                     xk8s;                      
    typedef signed short                    xk16s;                     
    typedef signed int                      xk32s;                     
    typedef signed long long                xk64s;                     
    typedef float                           xk32f;                     
    typedef double                          xk64f;                     
    typedef char                            xkChar;                
    typedef unsigned char                   xkByte;                    

#   define xk64U(CONSTANT)                  (CONSTANT##LLU)
#   define xk64S(CONSTANT)                  (CONSTANT##LL)

#endif

#define xkInlineFx(TYPE)                    static kInline TYPE kCall

#if (K_POINTER_SIZE == 4)

typedef xk32u                               xkSize;                  
#   define xkSIZE_MAX                       k32U_MAX

typedef xk32s                               xkSSize;                   
#   define xkSSIZE_MIN                      k32S_MIN
#   define xkSSIZE_MAX                      k32S_MAX

#elif (K_POINTER_SIZE == 8)

typedef xk64u                               xkSize;                  
#   define xkSIZE_MAX                       k64U_MAX

typedef xk64s                               xkSSize;                   
#   define xkSSIZE_MIN                      k64S_MIN
#   define xkSSIZE_MAX                      k64S_MAX

#endif

#define kALIGN_ANY                          (kMEMORY_ALIGNMENT_8)
#define kALIGN_ANY_SIZE                     (1 << kALIGN_ANY)

#define kVarArgList                         va_list

#if defined(K_MSVC) || defined(K_TI_CCS)
#   define kVarArgList_Start(ARG_PTR, PREV_PARAM)        va_start(ARG_PTR, PREV_PARAM)
#   define kVarArgList_End(ARG_PTR)                      va_end(ARG_PTR)
#   define kVarArgList_Copy(ARG_PTR, SOURCE)             ((ARG_PTR) = (SOURCE))
#   define kVarArgList_Next(ARG_PTR, TYPE)               va_arg(ARG_PTR, TYPE)
#else
#   define kVarArgList_Start(ARG_PTR, PREV_PARAM)        va_start(ARG_PTR, PREV_PARAM)
#   define kVarArgList_End(ARG_PTR)                      va_end(ARG_PTR)
#   define kVarArgList_Copy(ARG_PTR, SOURCE)             va_copy(ARG_PTR, SOURCE)
#   define kVarArgList_Next(ARG_PTR, TYPE)               va_arg(ARG_PTR, TYPE)
#endif


/* Deprecation support. */
#if !defined(K_NO_DEPRECATION) 
#   if defined(K_MSVC)
#       define xkDeprecate(SYMBOL)     __pragma(deprecated(SYMBOL))     
#   else
#       define xkDeprecate(SYMBOL)        
#   endif
#else
#   define xkDeprecate(SYMBOL)        
#endif

/* Warning support. */
#if defined(K_MSVC)

#   define xkWarn(MESSAGE)      __pragma(message(__FILE__ "(" xkStringizeDefine(__LINE__) "): warning: " MESSAGE))

#elif defined(K_GCC)

#   define xkWarnHelper(x) _Pragma (#x)
#   define xkWarn(MESSAGE) xkWarnHelper(GCC warning MESSAGE)

#else 

#   define xkWarn(MESSAGE)                  

#endif

/* Software breakpoint support. */
#if defined(K_DEBUG) && defined(K_MSVC)
#   define xkDebugBreak()        __debugbreak()
#else 
#   define xkDebugBreak()
#endif

#if defined (K_CPP)
#   define kBeginCHeader()   extern "C" {
#   define kEndCHeader()    }
#else
#   define kBeginCHeader() 
#   define kEndCHeader()  
#endif


/* 
 * Some source files require platform library headers to be included.  And, at least
 * for Windows, there can sometimes exist complicated rules about the particular order
 * in which headers have to be included. The K_PLATFORM symbol helps to deal with 
 * these issues. 
 * 
 * Any kApi source file that requires platform headers should have #define K_PLATFORM as the 
 * first line in the source file. This ensures that the most common platform headers are 
 * included here, in the correct order. 
 */
#if defined(K_PLATFORM)

/*
 * Common for all platforms.
 */
#   include <assert.h>

/*
 * Platform specific includes.
 */
#   if defined(K_WINDOWS)
#       include <winsock2.h>
#       include <mswsock.h>
#       include <ws2tcpip.h>
#       include <iphlpapi.h>
#       include <windows.h>
#       include <process.h>
#   endif
#   if defined(K_TI_BIOS)
#       include <xdc/std.h>
#       include <xdc/runtime/Types.h>
#       include <xdc/runtime/Error.h>
#       include <ti/sysbios/BIOS.h>
#       include <ti/sysbios/knl/Clock.h>
#       include <ti/sysbios/knl/Semaphore.h>
#       include <ti/sysbios/knl/Task.h>
        kBeginCHeader()
#           include <netmain.h>
#           include <nettools/nettools.h>
#           include <nettools/inc/inet.h>
        kEndCHeader()
#   endif
#   if defined(K_VX_KERNEL)
#       if defined(K_DEBUG)
#           pragma GCC optimize ("O2")
#               include <errno.h> /* work-around for error "ldarm: error in obj.o (.eh_frame); no .eh_frame_hdr table will be created." */
#           pragma GCC reset_options
#       endif
#       include <vxWorks.h>
#       include <sysLib.h> 
#       include <kernelLib.h> 
#       include <loadLib.h>
#       include <intLib.h> 
#       include <symLib.h>
#       include <sysSymTbl.h>
#       include <taskLib.h>
#       include <inline/semLibInline.h>
#       include <unldLib.h>
#       include <stdioLib.h> 
#       include <strLib.h>
#       include <sockLib.h> 
#       include <inetLib.h> 
#       include <ioLib.h>
#       include <fioLib.h> 
#       include <selectLib.h> 
#       include <netinet/in.h>
#       include <netinet/tcp.h>
#       include <wrapper/wrapperHostLib.h> 
#       include <netdb.h>
#       include <sys/socket.h>
#       include <net/if.h>
#       include <net/if_ll.h>
#       include <net/ifaddrs.h> 
#       include <net/ethernet.h>
#       include <ipnet/ipioctl.h>
#       include <ipcom_sock.h>
#       include <ipcom_netlink.h>
#   endif
#   if defined(K_POSIX)
#       include <errno.h>
#       include <unistd.h>
#       include <pthread.h>
#       include <semaphore.h>
#       include <sys/types.h>
#       include <sys/stat.h>
#       if defined(K_QNX)
#           include <fcntl.h>
#       else
#           include <sys/fcntl.h>
#       endif
#       include <sys/socket.h>
#       include <sys/select.h>
#       include <sys/syscall.h>
#       include <sys/ioctl.h>
#       include <sys/resource.h>
#       include <sys/time.h>
#       include <sys/timeb.h>
#       include <netinet/in.h>
#       include <netinet/tcp.h>
#       include <dlfcn.h>
#       include <dirent.h>
#       include <net/if.h>
#       include <ifaddrs.h>  /* not technically posix, but seemingly supported everywhere that matters */
#       include <netdb.h>

#   endif
#   if defined(K_DARWIN)
#       include <mach-o/dyld.h>
#   endif
#   if defined (K_LINUX)
#      include <signal.h>
#      include <sys/prctl.h>
#      include <sys/wait.h>
#      include <linux/sockios.h>
#      include <linux/ethtool.h>
#      include <linux/netlink.h>
#      include <linux/rtnetlink.h>
#   endif
#endif

#if defined(K_VX_KERNEL)
    typedef xk32s                           xkAtomic32s; 
    typedef xkSSize                         xkAtomicPointer; 
#else
    typedef volatile xk32s                  xkAtomic32s; 
    typedef void* volatile                  xkAtomicPointer; 
#endif

#if defined(K_PLATFORM)
#   if defined(K_WINDOWS)
#       define kOS_INFINITE    INFINITE
        typedef DWORD xkThreadId; 
#   elif defined(K_TI_BIOS)
#       define kOS_INFINITE    ti_sysbios_BIOS_WAIT_FOREVER
        typedef ti_sysbios_knl_Task_Handle xkThreadId; 
#   elif defined(K_VX_KERNEL)
#       define kOS_INFINITE    WAIT_FOREVER 
        typedef TASK_ID xkThreadId; 
#   elif defined(K_POSIX)
#       define kOS_INFINITE    0         /* no special "infinite" value */
        typedef pthread_t xkThreadId; 
#   endif
#endif

#if defined(K_VX_KERNEL)
#   if defined(K_MAIN_DYNAMIC)
#       undef kMain
#       define kMain kMain_Dynamic
#   else
#       undef kMain
#       define kMain kMain
#   endif
#   define kMAIN_DYNAMIC    "kMain_Dynamic"
#else
#   define kMAIN_DYNAMIC    "kMain"
#endif


/* 
* Deprecated (Stage 1): not recommended for further use, but not yet announced via kDeprecate
*/

#if defined (K_CPP)

    //[Deprecated] No longer required; provided that any C functions are properly annotated. 
    //Use kBeginCHeader/kEndCHeader to include a C header from C++ if the header was not designed to support C++.
#   define kBeginHeader()   extern "C" {

    //[Deprecated] No longer required; provided that any C functions are properly annotated. 
    //Use kBeginCHeader/kEndCHeader to include a C header from C++ if the header was not designed to support C++.
#   define kEndHeader()    }

#else

    //[Deprecated] No longer required; provided that any C functions are properly annotated. 
    //Use kBeginCHeader/kEndCHeader to include a C header from C++ if the header was not designed to support C++.
#   define kBeginHeader() 

    //[Deprecated] No longer required; provided that any C functions are properly annotated. 
    //Use kBeginCHeader/kEndCHeader to include a C header from C++ if the header was not designed to support C++.
#   define kEndHeader()  

#endif

#endif

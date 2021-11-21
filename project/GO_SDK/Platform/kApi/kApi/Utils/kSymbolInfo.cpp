/** 
 * @file    kSymbolInfo.cpp
 *
 * @internal
 * Copyright (C) 2014-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#define K_PLATFORM
#include <kApi/Utils/kSymbolInfo.h>
#include <kApi/Data/kMap.h>
#include <kApi/Data/kString.h>
#include <kApi/Threads/kLock.h>

kBeginStaticClassEx(k, kSymbolInfo)
kEndStaticClassEx()

kFx(void) xkSymbolInfo_Lock()
{
    kStaticObj(kSymbolInfo);
    kApiLockFx handler = kApiLib_SymbolLockHandler();
    kPointer provider= kApiLib_SymbolLockProvider();

    if (!kIsNull(handler))
    {
        handler(provider);
    }
    else if (!kIsNull(sobj->lock))
    {
        kLock_Enter(sobj->lock);
    }
}

kFx(void) xkSymbolInfo_Unlock()
{
    kStaticObj(kSymbolInfo);
    kApiUnlockFx handler = kApiLib_SymbolUnlockHandler();
    kPointer provider= kApiLib_SymbolLockProvider();

    if (!kIsNull(handler))
    {
        handler(provider);
    }
    else if (!kIsNull(sobj->lock))
    {
        kLock_Exit(sobj->lock);
    }
}

kFx(kStatus) xkSymbolInfo_FormatUnknown(kPointer function, kString description)
{
#if (K_POINTER_SIZE == 8)
    return kString_Setf(description, "[0x%016llX]", (k64u)function); 
#else
    return kString_Setf(description, "[0x%08X]", (k32u)function); 
#endif
}

#if defined(K_DEBUG) && defined(K_WINDOWS) && _MSC_VER >= 1500

#include <DbgHelp.h>

kFx(kStatus) xkSymbolInfo_InitStatic()
{
    kStaticObj(kSymbolInfo); 
    kPointer symbolService = GetCurrentProcess();
    const kSize NAME_CAPACITY = 256;
    kSize symbolInfoLength = sizeof(SYMBOL_INFO) + (NAME_CAPACITY - 1)*sizeof(WCHAR);
    SYMBOL_INFOW* symbolInfo = kNULL;
    IMAGEHLP_LINEW64* lineInfo = kNULL;
    kStatus status; 

    sobj->alloc = kAlloc_System();

    kTry
    {
        if (kApiLib_SymbolInitializationEnabled())
        {
            if (!SymInitializeW(symbolService, kNULL, kTRUE))
            {
                kLogf("kSymbolInfo: SymInitialize failed.");
            }
            else
            {
                sobj->service = symbolService;
            }
        }
        else
        {
            sobj->service = symbolService;
        }

        kTest(kLock_ConstructEx(&sobj->lock, xkLOCK_OPTION_PRIORITY_INHERITANCE, sobj->alloc));
        kTest(kMap_Construct(&sobj->cache, kTypeOf(kPointer), kTypeOf(kString), 0, sobj->alloc));

        kTest(kAlloc_Get(sobj->alloc, symbolInfoLength, &symbolInfo));
        symbolInfo->MaxNameLen = (ULONG)NAME_CAPACITY;
        symbolInfo->SizeOfStruct = sizeof(SYMBOL_INFO);
        sobj->symbolInfo = symbolInfo;

        kTest(kAlloc_Get(sobj->alloc, sizeof(IMAGEHLP_LINEW64), &lineInfo));
        lineInfo->SizeOfStruct = sizeof(IMAGEHLP_LINEW64);
        sobj->lineInfo = lineInfo;

        kTest(kAssembly_AddLoadHandler(xkSymbolInfo_OnAssemblyLoad, kNULL));
    }
    kCatch(&status)
    {
        xkSymbolInfo_ReleaseStatic(); 

        kEndCatch(status); 
    }

    return kOK; 
}

kFx(kStatus) xkSymbolInfo_ReleaseStatic()
{
    kStaticObj(kSymbolInfo); 

    kAssembly_RemoveLoadHandler(xkSymbolInfo_OnAssemblyLoad, kNULL); 

    if (kApiLib_SymbolInitializationEnabled() && !kIsNull(sobj->service))
    {
        SymCleanup(sobj->service); 
    }

    kCheck(kAlloc_Free(sobj->alloc, sobj->lineInfo)); 
    kCheck(kAlloc_Free(sobj->alloc, sobj->symbolInfo)); 

    kCheck(kObject_Dispose(sobj->cache));
    kCheck(kObject_Destroy(sobj->lock)); 

    return kOK; 
}

kFx(kStatus) xkSymbolInfo_OnAssemblyLoad(kSymbolInfo trace, kAssembly assembly, kPointer unused)
{   
    kStaticObj(kSymbolInfo); 

    xkSymbolInfo_Lock(); 
    {
        if (kApiLib_SymbolInitializationEnabled() && !kIsNull(sobj->service))
        {
            SymRefreshModuleList(sobj->service); 
        }
    }
    xkSymbolInfo_Unlock(); 
   
    return kOK; 
}

kFx(kStatus) kSymbolInfo_DescribeFunction(kPointer function, kString* description)
{
    kStaticObj(kSymbolInfo); 
    kString line = kNULL;
    kStatus status; 

    kCheckTrue(!kIsNull(sobj->service) && !kIsNull(sobj->cache), kERROR_UNIMPLEMENTED); 

    xkSymbolInfo_Lock();

    kTry
    {
        if (!kSuccess(kMap_FindT(sobj->cache, &function, &line)))
        {
            DWORD64 functionAddress = (DWORD64)(kSize)(function);
            SYMBOL_INFOW* symbolInfo = (SYMBOL_INFOW*) sobj->symbolInfo;
            IMAGEHLP_LINEW64* lineInfo = (IMAGEHLP_LINEW64*) sobj->lineInfo;
            DWORD displacement;
            BOOL symStatus;

            kTest(kString_Construct(&line, kNULL, sobj->alloc)); 

            symStatus = SymFromAddrW(sobj->service, functionAddress, NULL, symbolInfo);

            if (symStatus)
            {
                if (WideCharToMultiByte(CP_UTF8, 0, symbolInfo->Name, -1, sobj->tempPath1, (int)kCountOf(sobj->tempPath1), kNULL, kNULL) != 0)
                {
                    kTest(kString_Setf(line, "%s", sobj->tempPath1));

                    symStatus = SymGetLineFromAddrW64(sobj->service, functionAddress, &displacement, lineInfo);

                    if (symStatus)
                    {
                        kTest(xkPath_NativeWideToNormalizedWin(lineInfo->FileName, sobj->tempPath1, kCountOf(sobj->tempPath1)));
                        kTest(kPath_FileName(sobj->tempPath1, sobj->tempPath2, kCountOf(sobj->tempPath2)));

                        kTest(kString_Addf(line, " [%s, line %u]", sobj->tempPath2, lineInfo->LineNumber));
                    }
                }
            }

            if (kString_Length(line) == 0)
            {
                xkSymbolInfo_FormatUnknown(function, line);
            }        

            kTest(kMap_AddT(sobj->cache, &function, &line)); 
        }

        kShareRef(description, line); 
    }
    kCatchEx(&status)
    {
        kObject_Destroy(line);

        kEndCatchEx(status); 
    }
    kFinallyEx
    {
        xkSymbolInfo_Unlock();

        kEndFinallyEx(); 
    }

    return kOK;
}

#elif defined(K_DEBUG) && defined(K_GCC) && defined(K_POSIX)

#include <execinfo.h>

kFx(kStatus) xkSymbolInfo_InitStatic()
{
    kStaticObj(kSymbolInfo); 
    kStatus status; 

    sobj->alloc = kAlloc_System();

    kTry
    {
        kTest(kLock_ConstructEx(&sobj->lock, xkLOCK_OPTION_PRIORITY_INHERITANCE, sobj->alloc));
        kTest(kMap_Construct(&sobj->cache, kTypeOf(kPointer), kTypeOf(kString), 0, sobj->alloc));
    }
    kCatch(&status)
    {
        xkSymbolInfo_ReleaseStatic(); 

        kEndCatch(status); 
    }

    return kOK; 
}

kFx(kStatus) xkSymbolInfo_ReleaseStatic()
{
    kStaticObj(kSymbolInfo); 

    kCheck(kObject_Dispose(sobj->cache));
    kCheck(kObject_Destroy(sobj->lock)); 

    return kOK; 
}

kFx(kStatus) kSymbolInfo_DescribeFunction(kPointer function, kString* description)
{
    kStaticObj(kSymbolInfo); 
    kChar** symbolInfo = kNULL; 
    kString line = kNULL;
    kStatus status; 

    kCheckTrue(!kIsNull(sobj->cache), kERROR_UNIMPLEMENTED); 

    xkSymbolInfo_Lock();

    kTry
    {
        if (!kSuccess(kMap_FindT(sobj->cache, &function, &line)))
        {
            kTest(kString_Construct(&line, kNULL, sobj->alloc));          

            if (!kIsNull(symbolInfo = backtrace_symbols(&function, 1)))
            {
                kTest(kString_Set(line, symbolInfo[0])); 
            }

            if (kString_Length(line) == 0)
            {
                xkSymbolInfo_FormatUnknown(function, line);
            }

            kTest(kMap_AddT(sobj->cache, &function, &line));
        }

        kShareRef(description, line);
    }
    kCatchEx(&status)
    {
        kObject_Destroy(line);

        kEndCatchEx(status);
    }
    kFinallyEx
    {
        free(symbolInfo); 

        xkSymbolInfo_Unlock();

        kEndFinallyEx();
    }

    return kOK;
}

#else 

kFx(kStatus) xkSymbolInfo_InitStatic()
{
    return kOK; 
}

kFx(kStatus) xkSymbolInfo_ReleaseStatic()
{
    return kOK; 
}

kFx(kStatus) kSymbolInfo_DescribeFunction(kPointer function, kString* description)
{
    return kERROR_UNIMPLEMENTED; 
}

#endif

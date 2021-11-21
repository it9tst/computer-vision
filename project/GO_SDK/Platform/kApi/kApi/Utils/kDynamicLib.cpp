/** 
 * @file    kDynamicLib.cpp
 *
 * @internal
 * Copyright (C) 2013-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#define K_PLATFORM
#include <kApi/Utils/kDynamicLib.h>
#include <kApi/Io/kPath.h>
#include <stdio.h>

kBeginClassEx(k, kDynamicLib)
    kAddPrivateVMethod(kDynamicLib, kObject, VRelease)
kEndClassEx()

kFx(kStatus) kDynamicLib_Construct(kDynamicLib* library, const kChar* path, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback(allocator);
    kType type = kTypeOf(kDynamicLib); 
    kStatus status; 

    kCheck(kAlloc_GetObject(alloc, type, library)); 

    if (!kSuccess(status = xkDynamicLib_Init(*library, type, path, alloc)))
    {
        kAlloc_FreeRef(alloc, library); 
    }

    return status; 
} 

kFx(kStatus) kDynamicLib_ConstructFromHandle(kDynamicLib* library, kPointer handle, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback(allocator);
    kType type = kTypeOf(kDynamicLib); 
    kStatus status; 

    kCheck(kAlloc_GetObject(alloc, type, library)); 

    if (!kSuccess(status = xkDynamicLib_InitFromHandle(*library, type, handle, alloc)))
    {
        kAlloc_FreeRef(alloc, library); 
    }

    return status; 
} 

kFx(kStatus) xkDynamicLib_Init(kDynamicLib library, kType type, const kChar* path, kAlloc allocator)
{
    kObjR(kDynamicLib, library);  
    kStatus status = kOK;

    kCheck(kObject_Init(library, type, allocator)); 

    obj->handle = kNULL;
    obj->isOwned = kTRUE;

    if (!kSuccess(status = xkDynamicLib_OpenHandle(path, &obj->handle)))
    {
        xkDynamicLib_VRelease(library);
    }

    return status; 
}

kFx(kStatus) xkDynamicLib_InitFromHandle(kDynamicLib library, kType type, kPointer handle, kAlloc allocator)
{
    kObjR(kDynamicLib, library);  

    kCheck(kObject_Init(library, type, allocator)); 

    obj->handle = handle;
    obj->isOwned = kFALSE;

    return kOK; 
}

kFx(kStatus) xkDynamicLib_VRelease(kDynamicLib library)
{
    kObj(kDynamicLib, library); 

    if (!kIsNull(obj->handle) && obj->isOwned)
    {
        kCheck(xkDynamicLib_CloseHandle(obj->handle));
    }

    kCheck(kObject_VRelease(library)); 

    return kOK;
}

kFx(kStatus) kDynamicLib_FindFunction(kDynamicLib library, const kChar* name, kFunction* function)
{
    kObj(kDynamicLib, library); 

    return xkDynamicLib_Resolve(obj->handle, name, function);
}

#if defined(K_WINDOWS)

kFx(kStatus) xkDynamicLib_OpenHandle(const kChar* path, kPointer* handle)
{
    WCHAR wpath[MAX_PATH]; 

    kCheck(xkPath_NormalizedToNativeWideWin(path, wpath, kCountOf(wpath))); 

    if (kIsNull(*handle = (kPointer) LoadLibraryW(wpath)))
    {
        return kERROR_NOT_FOUND; 
    }

    return kOK;
}

kFx(kStatus) xkDynamicLib_Resolve(kPointer handle, const kChar* name, kFunction* function)
{
    FARPROC address = GetProcAddress((HMODULE)handle, name); 

    if (kIsNull(address))
    {
        return kERROR_NOT_FOUND; 
    }

    *(FARPROC*)function = address; 

    return kOK;
}

kFx(kStatus) xkDynamicLib_CloseHandle(kPointer handle)
{
    if (!FreeLibrary((HMODULE)handle))
    {
        return kERROR_OS; 
    }

    return kOK;
}

#elif defined (K_TI_BIOS)

kFx(kStatus) xkDynamicLib_OpenHandle(const kChar* path, kPointer* handle)
{
    return kERROR_UNIMPLEMENTED;
}

kFx(kStatus) xkDynamicLib_Resolve(kPointer handle, const kChar* name, kFunction* function)
{
    return kERROR_UNIMPLEMENTED;
}

kFx(kStatus) xkDynamicLib_CloseHandle(kPointer handle)
{
    return kERROR_UNIMPLEMENTED; 
}

#elif defined (K_VX_KERNEL)

typedef struct kDynamicLibUndefinedSymbolEntry
{
    kPointer nextItem; 
    char* name;
} kDynamicLibUndefinedSymbolEntry;

kFx(kStatus) xkDynamicLib_OpenHandle(const kChar* path, kPointer* handle)
{
    int fd = -1;
    MODULE_ID moduleId = NULL;

    kTry
    {
        if ((fd = open(path, O_RDONLY, 0)) < 0)
        {
            kThrow(kERROR_NOT_FOUND);
        }

        if (kIsNull(moduleId = loadModule(fd, LOAD_ALL_SYMBOLS)))
        {
            k32s loadError = errnoGet();

            /* 
             * The module may remain loaded even though NULL was returned above.  Locate the module, 
             * print the names of the unresolved symbols and exit with an error.
             */
            if (!kIsNull(moduleId = moduleFindByName((char*) path)))
            {
                if (!kIsNull(moduleId->undefSymList.head))
                {
                    SL_NODE* listCurrent = moduleId->undefSymList.head;

                    while (!kIsNull(listCurrent))
                    {
                        kDynamicLibUndefinedSymbolEntry* undefinedSym = (kDynamicLibUndefinedSymbolEntry*) listCurrent;

                        kLogf("kDynamicLib: undefined symbol: %s.", undefinedSym->name);

                        listCurrent = listCurrent->next;
                    }

                    kThrow(kERROR_INCOMPLETE);
                }
            }

            kThrow(xkUtils_ErrnoToStatus(loadError));
        }

        *handle = moduleId;
    }
    kFinally
    {
        if (fd >= 0)
        {
            close(fd);
        }

        kEndFinally();
    }

    return kOK;
}

kFx(kStatus) xkDynamicLib_Resolve(kPointer handle, const kChar* name, kFunction* function)
{
    SYMBOL_DESC symbolDesc = { (SYM_FIND_MASK)0 };

    symbolDesc.mask = SYM_FIND_BY_NAME;
    symbolDesc.name = (char*)name;

    if (symFind(sysSymTbl, &symbolDesc) == OK)
    {
        *function = (kFunction)symbolDesc.value;

        return kOK;
    }

    return kERROR_NOT_FOUND;
}

kFx(kStatus) xkDynamicLib_CloseHandle(kPointer handle)
{
    return unldByModuleId((MODULE_ID)handle, UNLD_FORCE) == OK ? kOK : kERROR_OS; 
}

#elif defined (K_POSIX)

kFx(kStatus) xkDynamicLib_OpenHandle(const kChar* path, kPointer* handle)
{
    kChar nativePath[kPATH_MAX];

    kCheck(xkPath_FromVirtual(path, nativePath, kCountOf(nativePath)));

    if (kIsNull(*handle = (kPointer)dlopen(nativePath, RTLD_NOW)))
    {
        return kERROR_NOT_FOUND; 
    }

    return kOK;
}

kFx(kStatus) xkDynamicLib_Resolve(kPointer handle, const kChar* name, kFunction* function)
{
    void* address = dlsym(handle, name); 

    if (kIsNull(address))
    {
        return kERROR_NOT_FOUND; 
    }

    *(void**)function = address;

    return kOK;
}

kFx(kStatus) xkDynamicLib_CloseHandle(kPointer handle)
{
    if (dlclose(handle) != 0)
    {
        return kERROR_OS; 
    }

    return kOK;
}

#endif

/** 
 * @file    kFile.cpp
 *
 * @internal
 * Copyright (C) 2004-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#define K_PLATFORM
#include <kApi/Io/kFile.h>
#include <kApi/Io/kDirectory.h>
#include <kApi/Io/kPath.h>
#include <kApi/Data/kArray1.h>
#include <kApi/Threads/kThread.h>

kBeginFullClassEx(k, kFile)
    kAddPrivateVMethod(kFile, kObject, VRelease)
    kAddPrivateVMethod(kFile, kStream, VReadSomeImpl)
    kAddPrivateVMethod(kFile, kStream, VWriteImpl)
    kAddPrivateVMethod(kFile, kStream, VSeek)
    kAddPrivateVMethod(kFile, kStream, VFlush)
    kAddPrivateVMethod(kFile, kStream, VFill)
kEndFullClassEx()

kFx(kStatus) xkFile_InitStatic()
{
    kApiFileFx handlers = { kNULL }; 

    handlers.open = xkFile_OpenImpl; 
    handlers.close = xkFile_CloseImpl; 
    handlers.read = xkFile_ReadImpl; 
    handlers.write = xkFile_WriteImpl; 
    handlers.flush = xkFile_FlushImpl; 
    handlers.seek = xkFile_SeekImpl; 
    handlers.size = xkFile_SizeImpl; 
    handlers.exists = xkFile_ExistsImpl; 
    handlers.copy = xkFile_CopyImpl; 
    handlers.move = xkFile_MoveImpl; 
    handlers.del = xkFile_DeleteImpl; 
    handlers.tempName = xkFile_TempNameImpl; 

    //respect handlers that have already been installed    
    if (kApiLib_HasFileHandlers())
    {
        kCheck(xkOverrideFunctions(&handlers, sizeof(handlers), kApiLib_FileHandlers())); 
    }
        
    kCheck(kApiLib_SetFileHandlers(&handlers)); 
 
    return kOK; 
}

kFx(kStatus) xkFile_ReleaseStatic()
{
    return kOK; 
}

kFx(kStatus) kFile_Load(const kChar* path, void* data, kSize* size, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback(allocator);
    kFile file = kNULL; 
    kByte* buffer = kNULL; 
    k64u fileSize; 
    kStatus exception; 

    kTry
    {
        kTest(kFile_Construct(&file, path, kFILE_MODE_READ, kNULL)); 

        if ((fileSize = kFile_Length(file)) >= kSIZE_MAX)
        {
            kThrow(kERROR_INCOMPLETE); 
        }

        kTest(kAlloc_Get(alloc, (kSize) fileSize, &buffer)); 
        kTest(kStream_Read(file, buffer, (kSize) fileSize)); 

        kTest(kFile_Close(file));

        *(void**)data = buffer;
        *size = (kSize) fileSize; 
    }
    kCatchEx(&exception)     
    {
        kAlloc_Free(alloc, buffer); 
        kEndCatchEx(exception); 
    }
    kFinallyEx
    {
        kCheck(kDestroyRef(&file)); 
        kEndFinallyEx(); 
    }

    return kOK; 
}

kFx(kStatus) kFile_LoadTo(const kChar* path, void* data, kSize capacity)
{
    kFile file = kNULL; 

    kTry
    {
        kTest(kFile_Construct(&file, path, kFILE_MODE_READ, kNULL)); 
        kTestTrue(kFile_Length(file) == capacity, kERROR_INCOMPLETE); 

        kTest(kStream_Read(file, data, capacity)); 
        kTest(kFile_Close(file)); 
    }
    kFinally
    {
        kDestroyRef(&file); 
        kEndFinally(); 
    }

    return kOK; 
}

kFx(kStatus) kFile_Save(const kChar* path, const void* data, kSize size)
{
    kFile file = kNULL; 

    kTry
    {
        kTest(kFile_Construct(&file, path, kFILE_MODE_WRITE, kNULL)); 
        kTest(kStream_Write(file, data, size)); 
        kTest(kFile_Close(file)); 
    }
    kFinally
    {
        kDestroyRef(&file); 
        kEndFinally(); 
    }

    return kOK; 
}

kFx(kStatus) kFile_Construct(kFile* file, const kChar* path, kFileMode mode, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback(allocator);
    kType type = kTypeOf(kFile); 
    kStatus status; 

    kCheck(kAlloc_GetObject(alloc, type, file)); 

    if (!kSuccess(status = xkFile_Init(*file, type, path, mode, alloc)))
    {
        kAlloc_FreeRef(alloc, file); 
    }

    return status; 
} 

kFx(kStatus) xkFile_Init(kFile file, kType type, const kChar* path, kFileMode mode, kAlloc allocator)
{
    kObjR(kFile, file); 
    kStatus status = kOK;  
    k64u zero; 
    
    kCheck(kStream_Init(file, type, allocator)); 

    obj->streamPosition = 0; 
    obj->streamLength = 0; 
    obj->lastMode = xkFILE_MODE_NULL; 

    xkFile_InitPlatformFields(file); 

    kTry
    {         
        kTest(kApiLib_FileHandlers()->open(file, path, mode)); 

        if (kIsError(kApiLib_FileHandlers()->seek(file, 0, kSEEK_ORIGIN_END, &obj->streamLength)) || 
            kIsError(kApiLib_FileHandlers()->seek(file, 0, kSEEK_ORIGIN_BEGIN, &zero)))
        {
            //file may not support seek; set the reported length to max, but allow the file to be used 
            obj->streamLength = k64U_MAX; 
        }
    }
    kCatch(&status) 
    {
        xkFile_VRelease(file); 
        kEndCatch(status); 
    }
       
    return kOK;
}

kFx(kStatus) xkFile_VRelease(kFile file)
{
    kObj(kFile, file); 

    //ignore errors
    kFile_Close(file); 

    kCheck(kObject_FreeMemRef(file, &obj->base.readBuffer)); 
    kCheck(kObject_FreeMemRef(file, &obj->base.writeBuffer));

    kCheck(kStream_VRelease(file)); 

    return kOK; 
}

kFx(kStatus) kFile_Close(kFile file)
{
    kStatus flushStatus = kOK; 
    kStatus closeStatus = kOK; 

    if (xkFile_IsOpen(file))
    {
        flushStatus = kStream_Flush(file); 
        closeStatus = kApiLib_FileHandlers()->close(file); 
    }

    return kIsError(flushStatus) ? flushStatus : (kIsError(closeStatus) ? closeStatus : kOK); 
}

kFx(kStatus) kFile_SetWriteBuffer(kFile file, kSize size)
{
    kObj(kFile, file); 

    kCheckState(xkFile_IsOpen(file)); 

    kCheck(kStream_Flush(file)); 

    kCheck(kObject_FreeMemRef(file, &obj->base.writeBuffer)); 

    obj->base.writeCapacity = 0; 
    obj->base.writeBegin = 0; 
    obj->base.writeEnd = 0; 

    if (size > 0)
    {
        kCheck(kObject_GetMem(file, size, &obj->base.writeBuffer)); 
        obj->base.writeCapacity = size; 
    }

    return kOK;
}

kFx(kStatus) kFile_SetReadBuffer(kFile file, kSize size)
{
    kObj(kFile, file); 

    kCheckState(xkFile_IsOpen(file)); 

    kCheck(kStream_Flush(file)); 

    kCheck(kObject_FreeMemRef(file, &obj->base.readBuffer)); 

    obj->base.readCapacity = 0; 
    obj->base.readBegin = 0; 
    obj->base.readEnd = 0; 

    if (size > 0)
    {
        kCheck(kObject_GetMem(file, size, &obj->base.readBuffer));                 
        obj->base.readCapacity = size; 
    }
    
    return kOK;
}

kFx(kStatus) xkFile_VReadSomeImpl(kFile file, void* buffer, kSize minCount, kSize maxCount, kSize* bytesRead)
{
    kObj(kFile, file); 
    kByte* dest = (kByte*) buffer;  
    kSize readCount = 0; 
    kSize copyCount, mediumCount; 

    kCheckState(xkFile_IsOpen(file)); 
    kCheckState((kFile_Position(file) + minCount) <= kFile_Length(file)); 

    //configure the stream for reading, if necessary
    if (obj->lastMode != kFILE_MODE_READ)
    {
        kCheck(kStream_Flush(file)); 

        obj->lastMode = kFILE_MODE_READ; 
    }
    
    //consume any bytes in the read buffer first
    if ((obj->base.readEnd - obj->base.readBegin) > 0)
    {
        copyCount = kMin_(maxCount, obj->base.readEnd - obj->base.readBegin);

        kMemCopy(dest, &obj->base.readBuffer[obj->base.readBegin], copyCount);     
        
        obj->base.readBegin += copyCount; 
        readCount += copyCount; 
    }

    //if the request is not yet satisfied
    if (readCount < minCount)
    {
        //if the request is larger than the internal read buffer, read directly into the caller's buffer; else read into the internal buffer
        if ((maxCount - readCount) >= obj->base.readCapacity)
        {            
            kCheck(xkFile_ReadAtLeast(file, &dest[readCount], minCount-readCount, maxCount-readCount, &mediumCount));

            readCount += mediumCount; 
        }
        else
        {
            kCheck(xkFile_ReadAtLeast(file, &obj->base.readBuffer[0], minCount - readCount, obj->base.readCapacity, &mediumCount));    

            copyCount = kMin_(mediumCount, maxCount - readCount); 

            kCheck(kMemCopy(&dest[readCount], &obj->base.readBuffer[0], copyCount)); 

            obj->base.readBegin = copyCount; 
            obj->base.readEnd = mediumCount; 
            readCount += copyCount; 
        }
    }

    if (!kIsNull(bytesRead))
    {
        *bytesRead = readCount; 
    }

    return kOK; 
}

kFx(kStatus) xkFile_VWriteImpl(kFile file, const void* buffer, kSize size)
{
    kObj(kFile, file); 

    kCheckState(xkFile_IsOpen(file)); 

    //configure the stream for writing, if necessary
    if (obj->lastMode != kFILE_MODE_WRITE)
    {
        kCheck(kStream_Flush(file)); 

        obj->lastMode = kFILE_MODE_WRITE; 
        obj->base.writeEnd = obj->base.writeCapacity; 
    }

    //if the internal write buffer already has content, but not enough free space, flush
    if ((obj->base.writeBegin > 0) && ((obj->base.writeEnd - obj->base.writeBegin) < size))
    {
        kCheck(kStream_Flush(file)); 
    }

    //if the write is larger than the internal write buffer, write directly to the medium; else write to the internal buffer
    if ((obj->base.writeEnd - obj->base.writeBegin) < size)
    {
        kCheck(xkFile_WriteAll(file, (const kByte*)buffer, size)); 
    }
    else
    {
        kCheck(kMemCopy(&obj->base.writeBuffer[obj->base.writeBegin], buffer, size)); 
        obj->base.writeBegin += size; 
    }

    return kOK; 
}

kFx(kStatus) xkFile_VFlush(kFile file)
{
    kObj(kFile, file); 
        
    kCheckState(xkFile_IsOpen(file)); 

    if ((obj->lastMode == kFILE_MODE_WRITE) && (obj->base.writeBegin > 0))
    {
        kCheck(xkFile_WriteAll(file, obj->base.writeBuffer, obj->base.writeBegin));
    }
    else if ((obj->lastMode == kFILE_MODE_READ) && ((obj->base.readEnd - obj->base.readBegin) > 0))
    {
        obj->base.bytesRead -= (obj->base.readEnd - obj->base.readBegin); 
    }

    obj->base.readBegin = obj->base.readEnd = 0; 
    obj->base.writeBegin = obj->base.writeEnd = 0; 
            
    return kOK;
}

kFx(kStatus) xkFile_VFill(kFile file)
{
    kObj(kFile, file); 
    kSize mediumCount; 

    kCheckState(xkFile_IsOpen(file)); 
    kCheckState(kFile_Position(file) < kFile_Length(file)); 

    //configure the stream for reading, if necessary
    if (obj->lastMode != kFILE_MODE_READ)
    {
        kCheck(kStream_Flush(file)); 

        obj->lastMode = kFILE_MODE_READ; 
    }
    
    kCheck(xkFile_ReadAtLeast(file, &obj->base.readBuffer[0], 1, obj->base.readCapacity, &mediumCount));    

    obj->base.readBegin = 0; 
    obj->base.readEnd = mediumCount; 

    return kOK; 
}


kFx(kStatus) xkFile_VSeek(kFile file, k64s offset, kSeekOrigin origin)
{
    kObj(kFile, file); 
      
    kCheckState(xkFile_IsOpen(file)); 

    kCheck(kStream_Flush(file)); 
    
    kCheck(kApiLib_FileHandlers()->seek(file, offset, origin, &obj->streamPosition)); 

    return kOK; 
}

kFx(k64u) kFile_Length(kFile file)
{
    kObj(kFile, file); 

    kAssert(xkFile_IsOpen(file)); 

    if (obj->lastMode != kFILE_MODE_WRITE)
    {
        return obj->streamLength; 
    }
    else
    {
        return kMax_(obj->streamLength, obj->streamPosition + obj->base.writeBegin); 
    }
}

kFx(k64u) kFile_Position(kFile file)
{
    kObj(kFile, file); 

    kAssert(xkFile_IsOpen(file)); 

    if (obj->lastMode == kFILE_MODE_READ)
    {
        return obj->streamPosition - (obj->base.readEnd - obj->base.readBegin); 
    }
    else if (obj->lastMode == kFILE_MODE_WRITE)
    {
        return obj->streamPosition + obj->base.writeBegin; 
    }
    else
    {
        return obj->streamPosition; 
    }
}

kFx(kBool) kFile_Exists(const kChar* fileName)
{
    return kApiLib_FileHandlers()->exists(fileName); 
}

kFx(k64u) kFile_Size(const kChar* fileName)
{
    return kApiLib_FileHandlers()->size(fileName); 
}

kFx(kStatus) kFile_Copy(const kChar* source, const kChar* destination)
{
    return kApiLib_FileHandlers()->copy(source, destination, kNULL, kNULL); 
} 

kFx(kStatus) kFile_CopyEx(const kChar* source, const kChar* destination, kCallbackFx progress, kPointer context)
{    
    return kApiLib_FileHandlers()->copy(source, destination, progress, context); 
}

kFx(kStatus) kFile_Move(const kChar* source, const kChar* destination)
{
    return kApiLib_FileHandlers()->move(source, destination, kNULL, kNULL); 
}

kFx(kStatus) kFile_MoveEx(const kChar* source, const kChar* destination, kCallbackFx progress, kPointer context)
{    
    return kApiLib_FileHandlers()->move(source, destination, progress, context); 
}

kFx(kStatus) kFile_Delete(const kChar* path)
{
    return kApiLib_FileHandlers()->del(path); 
}

kFx(kStatus) kFile_TempName(kChar* name, kSize capacity)
{
    return kApiLib_FileHandlers()->tempName(name, capacity); 
}

kFx(kStatus) xkFile_ReadAtLeast(kFile file, kByte* buffer, kSize minCount, kSize maxCount, kSize* bytesRead)
{
    kObj(kFile, file); 

    kCheck(kApiLib_FileHandlers()->read(file, buffer, minCount, maxCount, bytesRead)); 

    obj->streamPosition += *bytesRead;  
    obj->base.bytesRead += *bytesRead; 

    return kOK; 
}

kFx(kStatus) xkFile_WriteAll(kFile file, const kByte* buffer, kSize count)
{
    kObj(kFile, file); 

    kCheck(kApiLib_FileHandlers()->write(file, buffer, count)); 

    obj->streamPosition += count; 
    obj->streamLength = kMax_(obj->streamLength, obj->streamPosition); 
    obj->base.bytesWritten += count; 
    
    return kOK;    
}

kFx(k64u) xkFile_SizeImpl(const kChar* path)
{
    kFile file = kNULL; 
    k64u size = 0; 

    if (kSuccess(kFile_Construct(&file, path, kFILE_MODE_READ, kNULL)))
    {
        size = kFile_Length(file); 
        kObject_Destroy(file); 
    }

    return size; 
}

kFx(kStatus) xkFile_CopyImpl(const kChar* source, const kChar* destination, kCallbackFx progress, kPointer context)
{    
    kFile from = kNULL;
    kFile to = kNULL;
    kArray1 buffer = kNULL; 
    k64u size = 0; 
    k64u remaining = 0;
    kStatus status = kOK; 
    k64u updateTime = k64U_NULL; 
    
    if (kStrEquals(source, destination))
    {
        xkUpdateProgress(progress, context, kNULL, kNULL, 100); 
        return kOK; 
    }

    kTry
    {
        kTest(kFile_Construct(&from, source, kFILE_MODE_READ, kNULL)); 
        kTest(kFile_Construct(&to, destination, kFILE_MODE_WRITE, kNULL));
        kTest(kArray1_Construct(&buffer, kTypeOf(kByte), xkFILE_COPY_BUFFER_SIZE, kNULL)); 

        remaining = size = kFile_Length(from);

        while (remaining > 0)
        {
            kSize segment = (kSize) kMin_(remaining, xkFILE_COPY_BUFFER_SIZE); 

            kTest(kStream_Read(from, kArray1_DataT(buffer, kByte), segment));
            kTest(kStream_Write(to, kArray1_DataT(buffer, kByte), segment));

            remaining -= segment;

            xkUpdateProgress(progress, context, kNULL, &updateTime, (k32u)(100*(size-remaining)/size)); 
        }           

        kTest(kFile_Close(from)); 
        kTest(kFile_Close(to)); 

        xkUpdateProgress(progress, context, kNULL, kNULL, 100); 
    } 
    kCatchEx(&status)
    {
        //eliminate any partially-copied remnant (not strictly required, but convenient)
        if (!kIsNull(to))
        {
            kDestroyRef(&to);
            kFile_Delete(destination); 
        }

        kEndCatchEx(status); 
    }
    kFinallyEx
    {
        kCheck(kDestroyRef(&from));
        kCheck(kDestroyRef(&to));
        kCheck(kObject_Destroy(buffer));

        kEndFinallyEx(); 
    }

    return kOK;
}

kFx(kStatus) xkFile_MoveImpl(const kChar* source, const kChar* destination, kCallbackFx progress, kPointer context)
{
    kCheck(kApiLib_FileHandlers()->copy(source, destination, progress, context)); 
    kCheck(kApiLib_FileHandlers()->del(source)); 

    return kOK;
}

kFx(kStatus) xkFile_TempNameImpl(kChar* name, kSize capacity)
{
    return kStrPrintf(name, capacity, "%08X%08X%08X%08X", kRandom32u(),kRandom32u(), kRandom32u(), kRandom32u()); 
}

#if defined(K_WINDOWS)

kFx(void) xkFile_InitPlatformFields(kFile file)
{
    kObj(kFile, file); 

    obj->handle = INVALID_HANDLE_VALUE; 
}

kFx(kBool) xkFile_IsOpen(kFile file)
{
    kObj(kFile, file); 

    return obj->handle != INVALID_HANDLE_VALUE; 
}

kFx(kStatus) xkFile_OpenImpl(kFile file, const kChar* path, kFileMode mode)
{
    kObj(kFile, file); 
    DWORD desiredAccess = 0; 
    DWORD shareMode = 0; 
    DWORD creationDisposition = 0; 
    DWORD flags = FILE_ATTRIBUTE_NORMAL;  
    kChar nativePath[kPATH_MAX]; 
    WCHAR wpath[MAX_PATH]; 

    kCheck(xkPath_FromVirtual(path, nativePath, kCountOf(nativePath))); 
    kCheck(kPath_ToNative(nativePath, nativePath, kCountOf(nativePath))); 

    if (MultiByteToWideChar(CP_UTF8, 0, nativePath, -1, wpath, kCountOf(wpath)) == 0)
    {
        return kERROR_PARAMETER; 
    }

    if (mode == kFILE_MODE_READ)
    {
        desiredAccess = GENERIC_READ;        
        shareMode = FILE_SHARE_READ; 
        creationDisposition = OPEN_EXISTING;         
    }
    else if (mode == kFILE_MODE_WRITE)
    {
        desiredAccess = GENERIC_WRITE;        
        creationDisposition = CREATE_ALWAYS;         
    }
    else if (mode == (kFILE_MODE_READ | kFILE_MODE_WRITE))
    {
        desiredAccess = GENERIC_READ | GENERIC_WRITE;        
        creationDisposition = CREATE_ALWAYS;         
    }
    else if (mode == (kFILE_MODE_WRITE | kFILE_MODE_UPDATE))
    {
        desiredAccess = GENERIC_WRITE;        
        creationDisposition = OPEN_ALWAYS;         
    }
    else if (mode == (kFILE_MODE_READ | kFILE_MODE_WRITE | kFILE_MODE_UPDATE))
    {
        desiredAccess = GENERIC_READ | GENERIC_WRITE;        
        creationDisposition = OPEN_ALWAYS;         
    }
    else
    {
        return kERROR_PARAMETER; 
    }
  
    if ((obj->handle = CreateFile(wpath, desiredAccess, shareMode, kNULL, creationDisposition, flags, kNULL)) == INVALID_HANDLE_VALUE)
    {
        switch (GetLastError())
        {
        case ERROR_FILE_NOT_FOUND:
        case ERROR_PATH_NOT_FOUND:
            return kERROR_NOT_FOUND;
        default:
            return kERROR_STATE; 
        }
    }

    return kOK; 
}

kFx(kStatus) xkFile_CloseImpl(kFile file)
{
    kObj(kFile, file); 
    BOOL result = TRUE; 
    
    if (xkFile_IsOpen(file))
    {
        result = CloseHandle(obj->handle); 
        obj->handle = INVALID_HANDLE_VALUE; 
    }
    
    return (result) ? kOK : kERROR_STREAM; 
}

kFx(kStatus) xkFile_ReadImpl(kFile file, kByte* buffer, kSize minCount, kSize maxCount, kSize* bytesRead)
{
    kObj(kFile, file); 
    DWORD read; 

    kCheckArgs(maxCount <= xkFILE_MAX_IO_SIZE); 

    kCheck(ReadFile(obj->handle, buffer, (DWORD)maxCount, &read, kNULL)); 

    kCheck(read >= minCount); 

    *bytesRead = read;    

    return kOK; 
}

kFx(kStatus) xkFile_WriteImpl(kFile file, const kByte* buffer, kSize count)
{
    kObj(kFile, file); 
    DWORD written; 

    kCheckArgs(count <= xkFILE_MAX_IO_SIZE); 

    kCheck(WriteFile(obj->handle, buffer, (DWORD)count, &written, kNULL));

    kCheck(written == count); 

    return kOK; 
}

kFx(kStatus) xkFile_FlushImpl(kFile file)
{
    kObj(kFile, file); 

    if (obj->lastMode == kFILE_MODE_WRITE)
    {
        kCheck(FlushFileBuffers(obj->handle)); 
    }

    return kOK; 
}

kFx(kStatus) xkFile_SeekImpl(kFile file, k64s offset, kSeekOrigin origin, k64u* position)
{
    kObj(kFile, file); 
    DWORD moveMethod; 
    LARGE_INTEGER offsetLarge; 
    LARGE_INTEGER positionLarge; 

    switch(origin)
    {
    case kSEEK_ORIGIN_BEGIN:        moveMethod = FILE_BEGIN;    break;
    case kSEEK_ORIGIN_CURRENT:      moveMethod = FILE_CURRENT;  break;
    case kSEEK_ORIGIN_END:          moveMethod = FILE_END;      break;
    default:                                                    return 0; 
    }

    offsetLarge.QuadPart = offset; 

    kCheck((SetFilePointerEx(obj->handle, offsetLarge, &positionLarge, moveMethod) == TRUE) ? kOK : kERROR_STREAM); 

    *position = (k64u) positionLarge.QuadPart; 

    return kOK; 
}

kFx(kBool) xkFile_ExistsImpl(const kChar* path)
{
    kFile file = kNULL; 
    kBool exists = kFALSE; 

    //works on Windows (not Linux) -- should we do something else?
    if (kSuccess(kFile_Construct(&file, path, kFILE_MODE_READ, kNULL)))
    {
        exists = kTRUE; 
        kObject_Destroy(file); 
    }

    return exists; 
}

kFx(kStatus) xkFile_DeleteImpl(const kChar* path)
{
    kChar absPath[kPATH_MAX];
    kChar nativePath[kPATH_MAX];
    kChar directory[kPATH_MAX];
    WCHAR wdirectory[MAX_PATH];
    WCHAR wpath[MAX_PATH];
    WCHAR wtempFile[MAX_PATH];
    kSize i = 0;
    kBool moved = kFALSE;
    
    //confirm that the file doesn't already exist, to prevent needlessly delaying on rename failure (below)
    if (!kFile_Exists(path))
    {
        return kERROR_NOT_FOUND;
    }

    //we want to get the parent directory (below), but the given path may be relative;
    //if so, we convert to an absolute path that is relative to the current working directory
    kCheck(xkPath_FromVirtual(path, absPath, kCountOf(absPath)));
    kCheck(kPath_ToAbsolute(kNULL, absPath, absPath, sizeof(absPath)));
    kCheck(kPath_ToNative(absPath, nativePath, kCountOf(nativePath)));
    
    if (MultiByteToWideChar(CP_UTF8, 0, nativePath, -1, wpath, kCountOf(wpath)) == 0)
    {
        return kERROR_PARAMETER;
    }

    kCheck(kPath_Directory(absPath, directory, kCountOf(directory)));
    kCheck(kPath_ToNative(directory, directory, kCountOf(directory)));

    if (MultiByteToWideChar(CP_UTF8, 0, directory, -1, wdirectory, kCountOf(wdirectory)) == 0)
    {
        return kERROR_PARAMETER;
    }

    if (GetTempFileName(wdirectory, L"kFile", 0, wtempFile) == 0)
    {
        return kERROR_OS;
    }

    for (i = 0; i < xkFILE_WINDOWS_IO_ATTEMPTS && !moved; i++)
    {
        if (MoveFileEx(wpath, wtempFile, MOVEFILE_REPLACE_EXISTING) != 0)
        {
            moved = kTRUE;
        }
        else
        {
            kCheck(kThread_Sleep(100000));
        }
    }

    if (!moved)
    {
        return kERROR_OS;
    }

    for (i = 0; i < xkFILE_WINDOWS_IO_ATTEMPTS; i++)
    {
        if (DeleteFile(wtempFile) != 0)
        {
            return kOK;
        }
        kCheck(kThread_Sleep(100000));
    }

    return kERROR_OS;
}

kFx(kStatus) xkFile_TempFileImpl(const kChar* path, kChar* name, kSize capacity)
{
    kChar nativePath[kPATH_MAX];
    WCHAR wpath[MAX_PATH];
    WCHAR tempFile[MAX_PATH];
        
    kCheck(kPath_ToNative(path, nativePath, kCountOf(nativePath))); 
    
    if (MultiByteToWideChar(CP_UTF8, 0, nativePath, -1, wpath, kCountOf(wpath)) == 0)
    {
        return kERROR_PARAMETER; 
    }

    if (GetTempFileName(wpath, L"kFile", 0, tempFile) == 0)
    {
        return kERROR_OS;
    }

    return xkPath_NativeWideToNormalizedWin(tempFile, name, capacity);
}

#elif defined(K_DARWIN) || defined (K_LINUX) || defined (K_QNX)

kFx(void) xkFile_InitPlatformFields(kFile file)
{
    kObj(kFile, file); 
    obj->handle = -1; 
}

kFx(kBool) xkFile_IsOpen(kFile file)
{
    kObj(kFile, file); 
    return obj->handle != -1; 
}

kFx(kStatus) xkFile_OpenImpl(kFile file, const kChar* path, kFileMode mode)
{
    kObj(kFile, file); 
    kChar nativePath[kPATH_MAX]; 
    int oflag = 0; 

    kCheck(xkPath_FromVirtual(path, nativePath, kCountOf(nativePath))); 
    kCheck(kPath_ToNative(nativePath, nativePath, kCountOf(nativePath))); 

    if (mode == kFILE_MODE_READ)
    {
        oflag = O_RDONLY;        
    }
    else if (mode == kFILE_MODE_WRITE)
    {
        oflag = O_WRONLY | O_CREAT | O_TRUNC;     
    }
    else if (mode == (kFILE_MODE_READ | kFILE_MODE_WRITE))
    {
        oflag = O_RDWR | O_CREAT | O_TRUNC; 
    }
    else if (mode == (kFILE_MODE_WRITE | kFILE_MODE_UPDATE))
    {
        oflag = O_WRONLY | O_CREAT;     
    }
    else if (mode == (kFILE_MODE_READ | kFILE_MODE_WRITE | kFILE_MODE_UPDATE))
    {
        oflag = O_RDWR | O_CREAT; 
    }
    else
    {
        return kERROR_PARAMETER; 
    }

    if ((oflag & O_CREAT) != 0)
    {
        obj->handle = open(nativePath, oflag, S_IRWXU); 
    }
    else
    {
        obj->handle = open(nativePath, oflag); 
    }

    if (obj->handle == -1)
    {
        switch (errno)
        {
        case ENOENT:
        case ENOTDIR:
            return kERROR_NOT_FOUND;
        default:
            return kERROR_STATE; 
        }
    }
  
    return kOK; 
}

kFx(kStatus) xkFile_CloseImpl(kFile file)
{
    kObj(kFile, file); 
    int result = 0; 
    
    if (xkFile_IsOpen(file))
    {
        result = close(obj->handle); 
        obj->handle = -1; 
    }

    return (result != -1) ? kOK : kERROR_STREAM; 
}

kFx(kStatus) xkFile_ReadImpl(kFile file, kByte* buffer, kSize minCount, kSize maxCount, kSize* bytesRead)
{
    kObj(kFile, file); 
    ssize_t readSize; 

    if ((readSize = read(obj->handle, buffer, maxCount)) < 0)
    {
        return kERROR_STREAM; 
    } 

    if ((kSize)readSize < minCount)
    {
        return kERROR_STREAM; 
    }

    *bytesRead = (kSize) readSize; 

    return kOK; 
}

kFx(kStatus) xkFile_WriteImpl(kFile file, const kByte* buffer, kSize count)
{
    kObj(kFile, file); 
    ssize_t writeSize; 

    if ((writeSize = write(obj->handle, buffer, count)) < 0)
    {
        return kERROR_STREAM; 
    } 

    if ((kSize)writeSize != count)
    {
        return kERROR_STREAM; 
    }

    return kOK; 
}

kFx(kStatus) xkFile_FlushImpl(kFile file)
{
    return kOK; 
}

kFx(kStatus) xkFile_SeekImpl(kFile file, k64s offset, kSeekOrigin origin, k64u* position)
{
    kObj(kFile, file); 
    off_t updatedPosition; 
    int whence; 

    switch(origin)
    {
    case kSEEK_ORIGIN_BEGIN:        whence = SEEK_SET;    break;
    case kSEEK_ORIGIN_CURRENT:      whence = SEEK_CUR;    break;
    case kSEEK_ORIGIN_END:          whence = SEEK_END;    break;
    default:                                              return 0; 
    }

    if ((updatedPosition = lseek(obj->handle, (off_t)offset, whence)) == -1)
    {
        return kERROR_STREAM; 
    }

    *position = updatedPosition; 

    return kOK; 
}

kFx(kBool) xkFile_ExistsImpl(const kChar* path)
{
    struct stat st; 
    kChar nativePath[kPATH_MAX];

    kCheck(xkPath_FromVirtual(path, nativePath, kCountOf(nativePath)));

    return (stat(nativePath, &st) != -1) && S_ISREG(st.st_mode);
}

kFx(kStatus) xkFile_DeleteImpl(const kChar* path)
{
    kChar nativePath[kPATH_MAX]; 

    kCheck(xkPath_FromVirtual(path, nativePath, kCountOf(nativePath))); 
    kCheck(kPath_ToNative(nativePath, nativePath, kCountOf(nativePath))); 

    return (unlink(nativePath) == 0) ? kOK : kERROR_OS; 
}

#else 

kFx(void) xkFile_InitPlatformFields(kFile file)
{
    kObj(kFile, file); 
    obj->handle = kNULL; 
}

kFx(kBool) xkFile_IsOpen(kFile file)
{
    kObj(kFile, file); 

    return !kIsNull(obj->handle); 
}

kFx(kStatus) xkFile_OpenImpl(kFile file, const kChar* path, kFileMode mode)
{
    kAssert(kFALSE); 
    return kERROR_UNIMPLEMENTED;
}

kFx(kStatus) xkFile_CloseImpl(kFile file)
{
    kAssert(kFALSE); 
    return kERROR_UNIMPLEMENTED;
}

kFx(kStatus) xkFile_ReadImpl(kFile file, kByte* buffer, kSize minCount, kSize maxCount, kSize* bytesRead)
{
    kAssert(kFALSE); 
    return kERROR_UNIMPLEMENTED;
}

kFx(kStatus) xkFile_WriteImpl(kFile file, const kByte* buffer, kSize count)
{
    kAssert(kFALSE); 
    return kERROR_UNIMPLEMENTED;
}

kFx(kStatus) xkFile_FlushImpl(kFile file)
{
    kAssert(kFALSE); 
    return kERROR_UNIMPLEMENTED;
}

kFx(kStatus) xkFile_SeekImpl(kFile file, k64s offset, kSeekOrigin origin, k64u* position)
{
    kAssert(kFALSE); 
    return kERROR_UNIMPLEMENTED;
}

kFx(kBool) xkFile_ExistsImpl(const kChar* path)
{
    kAssert(kFALSE); 
    return kFALSE;
}

kFx(kStatus) xkFile_DeleteImpl(const kChar* path)
{
    kAssert(kFALSE); 
    return kERROR_UNIMPLEMENTED;
}

#endif

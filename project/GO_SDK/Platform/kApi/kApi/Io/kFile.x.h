/** 
 * @file    kFile.x.h
 *
 * @internal
 * Copyright (C) 2004-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_FILE_X_H
#define K_API_FILE_X_H

#include <kApi/Io/kStream.h>

#define xkFILE_COPY_BUFFER_SIZE       (64*1024)         //size, in bytes, of buffer used for file copy/move operations
#define xkFILE_WINDOWS_IO_ATTEMPTS    (50)              // Let virus scanner or indexer do whatever they do and try up to this time again

typedef struct kFileStatic
{
    k32u placeholder;       //unused
} kFileStatic; 

typedef struct kFileVTable
{
    kStreamVTable base; 
} kFileVTable; 

kDeclareFullClassEx(k, kFile, kStream)

#if defined(K_PLATFORM) 

#if defined(K_WINDOWS)

#   define xkFILE_MAX_IO_SIZE    (0xFFFFFFFF)

#   define xkFilePlatformFields()        \
        HANDLE handle;    
    
#elif defined(K_DARWIN) || defined(K_LINUX) || defined (K_QNX)

#   define xkFilePlatformFields()        \
        int handle;         

#else

#   define xkFilePlatformFields()        \
        kPointer handle;

#endif

#define xkFILE_MODE_NULL     (0)


typedef struct kFileClass
{   
    kStreamClass base; 
    k64u streamPosition; 
    k64u streamLength; 
    kFileMode lastMode;
    xkFilePlatformFields()
} kFileClass;

/* 
* Private methods. 
*/

kFx(kStatus) xkFile_InitStatic(); 
kFx(kStatus) xkFile_ReleaseStatic(); 

kFx(kStatus) xkFile_Init(kFile file, kType type, const kChar* path, kFileMode mode, kAlloc allocator); 
kFx(kStatus) xkFile_VRelease(kFile file);

kFx(kStatus) xkFile_VReadSomeImpl(kFile file, void* buffer, kSize minCount, kSize maxCount, kSize* bytesRead);
kFx(kStatus) xkFile_VWriteImpl(kFile file, const void* buffer, kSize size);
kFx(kStatus) xkFile_VSeek(kFile file, k64s offset, kSeekOrigin origin);
kFx(kStatus) xkFile_VFlush(kFile file);
kFx(kStatus) xkFile_VFill(kFile file);

kFx(kStatus) xkFile_ReadAtLeast(kFile file, kByte* buffer, kSize minCount, kSize maxCount, kSize* bytesRead);
kFx(kStatus) xkFile_WriteAll(kFile file, const kByte* buffer, kSize count);

kFx(kBool) xkFile_ExistsImpl(const kChar* path); 
kFx(k64u) xkFile_SizeImpl(const kChar* path); 
kFx(kStatus) xkFile_CopyImpl(const kChar* source, const kChar* destination, kCallbackFx progress, kPointer context);
kFx(kStatus) xkFile_MoveImpl(const kChar* source, const kChar* destination, kCallbackFx progress, kPointer context); 
kFx(kStatus) xkFile_TempNameImpl(kChar* name, kSize capacity); 
kFx(kStatus) xkFile_TempFileImpl(const kChar* path, kChar* name, kSize capacity); 

kFx(void) xkFile_InitPlatformFields(kFile file);
kFx(kBool) xkFile_IsOpen(kFile file);

kFx(kStatus) xkFile_OpenImpl(kFile file, const kChar* path, kFileMode mode);
kFx(kStatus) xkFile_CloseImpl(kFile file);
kFx(kStatus) xkFile_ReadImpl(kFile file, kByte* buffer, kSize minCount, kSize maxCount, kSize* bytesRead);
kFx(kStatus) xkFile_WriteImpl(kFile file, const kByte* buffer, kSize count);
kFx(kStatus) xkFile_FlushImpl(kFile file); 
kFx(kStatus) xkFile_SeekImpl(kFile file, k64s offset, kSeekOrigin origin, k64u* position); 
kFx(kStatus) xkFile_DeleteImpl(const kChar* path); 

#endif

#endif

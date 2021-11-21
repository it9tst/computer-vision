/** 
 * @file    kMemory.x.h
 * @brief   Declares the kMemory class. 
 *
 * @internal
 * Copyright (C) 2005-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_MEMORY_X_H
#define K_API_MEMORY_X_H

#include <kApi/Io/kStream.h>

#define xkMEMORY_GROWTH_FACTOR               (2)
#define xkMEMORY_MIN_CAPACITY                (256)

typedef k32s xkMemoryMode; 

#define xkMEMORY_MODE_NULL         (0x0)     // Read/write state unknown.
#define xkMEMORY_MODE_READ         (0x1)     // Currently reading. 
#define xkMEMORY_MODE_WRITE        (0x2)     // Currently writing.

typedef struct kMemoryClass
{   
    kStreamClass base; 
    kByte* buffer;
    kSize position;
    kSize length; 
    kSize capacity; 
    kBool owned; 
    xkMemoryMode lastMode; 
} kMemoryClass;

kDeclareClassEx(k, kMemory, kStream)

/* 
* Private methods. 
*/

kFx(kStatus) xkMemory_Init(kMemory memory, kType type, kAlloc allocator); 
kFx(kStatus) xkMemory_VRelease(kMemory memory); 

kFx(kStatus) xkMemory_VReadSomeImpl(kMemory memory, void* buffer, kSize minCount, kSize maxCount, kSize* bytesRead);
kFx(kStatus) xkMemory_VWriteImpl(kMemory memory, const void* buffer, kSize size); 
kFx(kStatus) xkMemory_VSeek(kMemory memory, k64s offset, kSeekOrigin origin); 
kFx(kStatus) xkMemory_VFlush(kMemory memory); 
kFx(kStatus) xkMemory_VFill(kMemory memory); 

kFx(kBool) xkMemory_IsAttached(kMemory memory); 

#endif


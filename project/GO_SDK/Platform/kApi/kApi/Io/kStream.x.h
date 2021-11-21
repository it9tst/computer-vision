/** 
 * @file    kStream.x.h
 *
 * @internal
 * Copyright (C) 2005-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_STREAM_X_H
#define K_API_STREAM_X_H

typedef struct kStreamClass
{
    kObjectClass base; 

    /* 
     * Protected Fields
     */
    kByte* readBuffer;                          // [Protected] Read buffer base pointer.
    kSize readCapacity;                         // [Protected] Read buffer capacity, in bytes.
    kSize readBegin;                            // [Protected] Current read location within read buffer. 
    kSize readEnd;                              // [Protected] End of valid data within read buffer. 

    kByte* writeBuffer;                         // [Protected] Write buffer base pointer.
    kSize writeCapacity;                        // [Protected] Write buffer capacity, in bytes. 
    kSize writeBegin;                           // [Protected] Current write location within write buffer.
    kSize writeEnd;                             // [Protected] End of writeable region within write buffer.

    k64u bytesRead;                             // [Protected] Total bytes read from underlying medium.
    k64u bytesWritten;                          // [Protected] Total bytes written to underlying medium.
} kStreamClass;

typedef struct kStreamVTable
{
    kObjectVTable base; 
    kStatus (kCall* VReadSomeImpl)(kStream stream, void* buffer, kSize minCount, kSize maxCount, kSize* bytesRead);
    kStatus (kCall* VReadImpl)(kStream stream, void* buffer, kSize size);    
    kStatus (kCall* VWriteImpl)(kStream stream, const void* buffer, kSize size);
    kStatus (kCall* VSeek)(kStream stream, k64s offset, kSeekOrigin origin);
    kStatus (kCall* VFlush)(kStream stream);
    kStatus (kCall* VFill)(kStream stream);
} kStreamVTable; 

kDeclareVirtualClassEx(k, kStream, kObject)

/* 
* Forward declarations. 
*/

kFx(kStatus) kStream_CopyEx(kStream stream, kStream source, kSize size, kCallbackFx progress, kPointer context); 

/* 
* Deprecated (Stage 1): not recommended for further use, but not yet announced via kDeprecate
*/

//[Deprecated] Stream implementations should implement VReadSomeImpl instead
kInlineFx(kStatus) kStream_VReadImpl(kStream stream, void* buffer, kSize size)
{
    return kERROR_UNIMPLEMENTED; 
}

#endif

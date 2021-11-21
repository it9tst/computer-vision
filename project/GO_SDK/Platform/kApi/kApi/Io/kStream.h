/** 
 * @file    kStream.h
 * @brief   Declares the kStream class. 
 *
 * @internal
 * Copyright (C) 2005-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_STREAM_H
#define K_API_STREAM_H

#include <kApi/kApiDef.h>
#include <kApi/Io/kStream.x.h>

/**
 * @class   kStream
 * @extends kObject
 * @ingroup kApi-Io
 * @brief   Represents an I/O stream. 
 */
//typedef kObject kStream;   --forward-declared in kApiDef.x.h

/*
* Public 
*/

/** 
 * Reads the specified number of bytes from the stream.
 *
 * @public              @memberof kStream
 * @param   stream      Stream object. 
 * @param   buffer      Destination for bytes that are read.
 * @param   size        Count of bytes to read.
 * @return              Operation status. 
 */
kInlineFx(kStatus) kStream_Read(kStream stream, void* buffer, kSize size)
{
    kObj(kStream, stream); 

    if ((obj->readEnd - obj->readBegin) < size)
    {
        return xkStream_VTable(stream)->VReadSomeImpl(stream, buffer, size, size, kNULL);
    }
    
    kMemCopy(buffer, &obj->readBuffer[obj->readBegin], size); 
    obj->readBegin += size; 

    return kOK;   
}

/** 
 * Reads up to the specified number of bytes from the stream.
 *
 * @public              @memberof kStream
 * @param   stream      Stream object. 
 * @param   buffer      Destination for bytes that are read.
 * @param   minCount    Minimum count of bytes to read.
 * @param   maxCount    Maximum count of bytes to read.
 * @param   bytesRead   Receives count of bytes read.
 * @return              Operation status. 
 */
kInlineFx(kStatus) kStream_ReadSome(kStream stream, void* buffer, kSize minCount, kSize maxCount, kSize* bytesRead)
{
    kObj(kStream, stream); 
    kSize bufferAvailable = obj->readEnd - obj->readBegin; 

    if (bufferAvailable < minCount)
    {
        return xkStream_VTable(stream)->VReadSomeImpl(stream, buffer, minCount, maxCount, bytesRead);
    }
    else
    {
        kSize size = kMin_(bufferAvailable, maxCount);

        kMemCopy(buffer, &obj->readBuffer[obj->readBegin], size); 
        obj->readBegin += size; 

        if (!kIsNull(bytesRead))
        {
            *bytesRead = size; 
        }
    }

    return kOK;   
}

/** 
 * Writes the specified number of bytes to the stream.
 *
 * @public              @memberof kStream
 * @param   stream      Stream object. 
 * @param   buffer      Bytes to be written to the stream.
 * @param   size        Count of bytes to be written.
 * @return              Operation status. 
 */
kInlineFx(kStatus) kStream_Write(kStream stream, const void* buffer, kSize size)
{
    kObj(kStream, stream); 

    if ((obj->writeEnd - obj->writeBegin) < size)
    {
        return xkStream_VTable(stream)->VWriteImpl(stream, buffer, size); 
    }

    kMemCopy(&obj->writeBuffer[obj->writeBegin], buffer, size); 
    obj->writeBegin += size; 

    return kOK; 
}

/** 
 * Copies the specified number of bytes from one stream to another. 
 *
 * @public              @memberof kStream
 * @param   stream      Destination stream. 
 * @param   source      Source stream. 
 * @param   size        Count of bytes to be copied.
 * @return              Operation status. 
 */
kInlineFx(kStatus) kStream_Copy(kStream stream, kStream source, kSize size)
{
    return kStream_CopyEx(stream, source, size, kNULL, kNULL); 
}

/** 
 * Copies the specified number of bytes from one stream to another, with progress feedback.
 *
 * The specified callback will be invoked to provide feedback on the progress of the operation. The callback 'args' 
 * parameter will receive a k32u value representing the percentage completed. The callback is guaranteed to be 
 * called at least once if the operation is successful, with a progress value of 100%. 
 *
 * @public              @memberof kStream
 * @param   stream      Destination stream. 
 * @param   source      Source stream. 
 * @param   size        Count of bytes to be copied.
 * @param   progress    Optional progress callback (can be kNULL). 
 * @param   context     Callback context.
 * @return              Operation status. 
 */
kFx(kStatus) kStream_CopyEx(kStream stream, kStream source, kSize size, kCallbackFx progress, kPointer context); 

/** 
 * Copies all bytes from one stream to another.
 *
 * @public              @memberof kStream
 * @param   stream      Destination stream. 
 * @param   source      Source stream. 
 * @return              Operation status. 
 */
kFx(kStatus) kStream_CopyAll(kStream stream, kStream source);

/** 
 * Moves the read/write pointer to the specified location, if supported by the underlying stream.
 * 
 * @public              @memberof kStream
 * @param   stream      Stream object. 
 * @param   offset      Offset by which to adjust the read/write pointer.
 * @param   origin      Origin to which movement is relative (i.e. begin, current, end). 
 * @return              Operation status. 
 */
kInlineFx(kStatus) kStream_Seek(kStream stream, k64s offset, kSeekOrigin origin)
{
    return xkStream_VTable(stream)->VSeek(stream, offset, origin); 
}

/** 
 * Flushes buffered writes to the underlying medium.
 *
 * @public              @memberof kStream
 * @param   stream      Stream object. 
 * @return              Operation status. 
 */
kInlineFx(kStatus) kStream_Flush(kStream stream)
{
    return xkStream_VTable(stream)->VFlush(stream); 
}

/** 
 * Partially fills the read buffer with bytes from the underlying medium.
 *
 * @public              @memberof kStream
 * @param   stream      Stream object. 
 * @return              Operation status. 
 */
kInlineFx(kStatus) kStream_Fill(kStream stream)
{
    kObj(kStream, stream); 

    kCheckState(!kIsNull(obj->readBuffer));

    if ((obj->readEnd - obj->readBegin) > 0)
    {
        return kOK; 
    }
    
    return xkStream_VTable(stream)->VFill(stream); 
}

/** 
 * Reports the number of bytes read from this stream.
 *
 * @public              @memberof kStream
 * @param   stream      Stream object. 
 * @return              Count of bytes read.
 */
kInlineFx(k64u) kStream_BytesRead(kStream stream)
{
    kObj(kStream, stream); 

    return obj->bytesRead - (k64u)(obj->readEnd - obj->readBegin); 
}

/** 
 * Reports the number of bytes written to this stream.
 *
 * @public              @memberof kStream
 * @param   stream      Stream object. 
 * @return              Count of bytes written.
 */
kInlineFx(k64u) kStream_BytesWritten(kStream stream)
{
    kObj(kStream, stream); 

    return obj->bytesWritten + (k64u)obj->writeBegin; 
}

/** 
 * Clears stream statistics (e.g. BytesRead, BytesWritten). 
 *
 * @public              @memberof kStream
 * @param   stream      Stream object. 
 * @return              Operation Status
 */
kInlineFx(kStatus) kStream_ClearStats(kStream stream)
{
    kObj(kStream, stream); 

    obj->bytesRead = 0; 
    obj->bytesWritten = 0; 

    return kOK; 
}

/*
* Protected
*/

/**
 * Protected method called by derived classes to initialize the kStream base class. 
 * 
 * This method should typically be called as the first statement in derived initializer methods. 
 *
 * @protected           @memberof kStream
 * @param   stream      Stream instance (not yet initialized). 
 * @param   type        Object type (required).
 * @param   allocator   Memory allocator (required).
 * @return              Operation status.  
*/
kInlineFx(kStatus) kStream_Init(kStream stream, kType type, kAlloc allocator)
{
    kObjR(kStream, stream); 

    kCheck(kObject_Init(stream, type, allocator)); 

    obj->readBuffer = kNULL;
    obj->readCapacity = 0;
    obj->readBegin = 0;
    obj->readEnd = 0;
    obj->writeBuffer = kNULL;
    obj->writeCapacity = 0;
    obj->writeBegin = 0;
    obj->writeEnd = 0;
    obj->bytesRead = 0;
    obj->bytesWritten = 0;

    return kOK; 
}

/**
 * Protected virtual method that deallocates any resources owned by the object. 
 * 
 * @protected           @memberof kStream
 * @param   stream      Stream object. 
 * @return              Operation status.
 * @see                 kObject_VRelease.
*/
kInlineFx(kStatus) kStream_VRelease(kStream stream)
{
    return kObject_VRelease(stream);
}

/**
 * Protected virtual method that deallocates any resources owned by the object. 
 * 
 * @protected           @memberof kStream
 * @param   stream      Stream object. 
 * @return              Operation status.
 * @see                 kObject_VRelease.
*/

/** 
 * Protected virtual method that reads up to the specified number of bytes from the stream.
 *
 * This method should be overridden by derived stream classes that support reading.
 * 
 * @protected              @memberof kStream
 * @param   stream      Stream object. 
 * @param   buffer      Destination for bytes that are read.
 * @param   minCount    Minimum count of bytes to read.
 * @param   maxCount    Maximum count of bytes to read.
 * @param   bytesRead   Receives count of bytes read.
 * @return              Operation status. 
 */
kInlineFx(kStatus) kStream_VReadSomeImpl(kStream stream, void* buffer, kSize minCount, kSize maxCount, kSize* bytesRead)
{
    //legacy support: for classes that haven't been updated to implement VReadSomeImpl, 
    //try VReadImpl instead (deprecated, but still in the vtable)
    kCheck(xkStream_VTable(stream)->VReadImpl(stream, buffer, minCount)); 
    
    if (!kIsNull(bytesRead))
    {
        *bytesRead = minCount; 
    }

    return kOK; 
}

/** 
 * Protected virtual method that writes the specified number of bytes to the stream.
 *
 * This method should be overridden by derived stream classes that support writing.
 * 
 * @protected           @memberof kStream
 * @param   stream      Stream object. 
 * @param   buffer      Bytes to be written to the stream.
 * @param   size        Count of bytes to be written.
 * @return              Operation status. 
 */
kInlineFx(kStatus) kStream_VWriteImpl(kStream stream, const void* buffer, kSize size)
{
    return kERROR_UNIMPLEMENTED; 
}

/** 
 * Protected virtual method that moves the read/write pointer to the specified location.
 *
 * This method should be overridden by derived stream classes that support seeking.
 * 
 * @protected           @memberof kStream
 * @param   stream      Stream object. 
 * @param   offset      Offset by which to adjust the read/write pointer.
 * @param   origin      Origin to which movement is relative (i.e. begin, current, end). 
 * @return              Operation status. 
 */
kInlineFx(kStatus) kStream_VSeek(kStream stream, k64s offset, kSeekOrigin origin)
{
    return kERROR_UNIMPLEMENTED; 
}

/** 
 * Protected virtual method that flushes buffered writes to the underlying medium.
 *
 * This method should be overridden by derived stream classes that support buffered writes.
 * 
 * @protected           @memberof kStream
 * @param   stream      Stream object. 
 * @return              Operation status. 
 */
kInlineFx(kStatus) kStream_VFlush(kStream stream)
{
    return kOK; 
}

/** 
 * Protected virtual method that partially fills the read buffer with bytes from the underlying medium.
 *
 * This method should be overridden by derived stream classes that support buffered reads.
 * 
 * @protected           @memberof kStream
 * @param   stream      Stream object. 
 * @return              Operation status. 
 */
kInlineFx(kStatus) kStream_VFill(kStream stream)
{
    return kERROR_UNIMPLEMENTED;
}

#endif

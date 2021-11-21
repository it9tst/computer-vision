/** 
 * @file    kMemory.cpp
 * @brief   Declares the kMemory class. 
 *
 * @internal
 * Copyright (C) 2005-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <kApi/Io/kMemory.h>

kBeginClassEx(k, kMemory)
    kAddPrivateVMethod(kMemory, kObject, VRelease)
    kAddPrivateVMethod(kMemory, kStream, VReadSomeImpl)
    kAddPrivateVMethod(kMemory, kStream, VWriteImpl)
    kAddPrivateVMethod(kMemory, kStream, VSeek)
    kAddPrivateVMethod(kMemory, kStream, VFlush)
    kAddPrivateVMethod(kMemory, kStream, VFill)
kEndClassEx()

kFx(kStatus) kMemory_Construct(kMemory* memory, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback(allocator);
    kType type = kTypeOf(kMemory); 
    kStatus status; 

    kCheck(kAlloc_GetObject(alloc, type, memory)); 

    if (!kSuccess(status = xkMemory_Init(*memory, type, alloc)))
    {
        kAlloc_FreeRef(alloc, memory); 
    }

    return status; 
} 

kFx(kStatus) xkMemory_Init(kMemory memory, kType type, kAlloc allocator)
{
    kObjR(kMemory, memory);  
 
    kCheck(kStream_Init(memory, type, allocator));

    obj->buffer = 0; 
    obj->position = 0; 
    obj->length = 0; 
    obj->capacity = 0; 
    obj->owned = kTRUE; 
    obj->lastMode = xkMEMORY_MODE_NULL; 
       
    return kOK;
}

kFx(kStatus) xkMemory_VRelease(kMemory memory)
{
    kObj(kMemory, memory);  

    if (obj->owned)
    {
        kCheck(kObject_FreeMemRef(memory, &obj->buffer)); 
    }

    kCheck(kStream_VRelease(memory)); 
    
    return kOK;
}

kFx(kStatus) kMemory_Attach(kMemory memory, void* buffer, kSize position, kSize length, kSize capacity)
{
    kObj(kMemory, memory);  
    
    if (obj->owned)
    {
        kCheck(kObject_FreeMemRef(memory, &obj->buffer)); 
        obj->capacity = 0; 
    }       

    obj->buffer = (kByte*) buffer; 
    obj->position = position; 
    obj->length = length; 
    obj->capacity = capacity; 
    obj->owned = kFALSE; 

    obj->base.readBegin = obj->base.readEnd = obj->base.readCapacity = 0; 
    obj->base.readBuffer = (kByte*)kSIZE_MAX; 
    obj->base.writeBegin = obj->base.writeEnd = obj->base.writeCapacity = 0; 
    obj->base.writeBuffer = (kByte*)kSIZE_MAX; 

    obj->base.bytesRead = 0; 
    obj->base.bytesWritten = 0; 

    obj->lastMode = xkMEMORY_MODE_NULL; 

    return kOK; 
}

kFx(kBool) xkMemory_IsAttached(kMemory memory)
{
    kObj(kMemory, memory);  

    return !obj->owned;     
}

kFx(kStatus) kMemory_Allocate(kMemory memory, kSize initialCapacity)
{
    kObj(kMemory, memory);  
       
    if (obj->owned)
    {
        kCheck(kObject_FreeMemRef(memory, &obj->buffer)); 
        obj->capacity = 0; 
    }       

    kCheck(kObject_GetMem(memory, initialCapacity, &obj->buffer)); 

    obj->position = 0; 
    obj->length = 0; 
    obj->capacity = initialCapacity; 
    obj->owned = kTRUE; 

    obj->base.readBegin = obj->base.readEnd = obj->base.readCapacity = 0; 
    obj->base.readBuffer = (kByte*)kSIZE_MAX; 
    obj->base.writeBegin = obj->base.writeEnd = obj->base.writeCapacity = 0; 
    obj->base.writeBuffer = (kByte*)kSIZE_MAX; 

    obj->base.bytesRead = 0; 
    obj->base.bytesWritten = 0; 

    obj->lastMode = xkMEMORY_MODE_NULL; 

    return kOK; 
}

kFx(kStatus) kMemory_Reserve(kMemory memory, kSize minimumCapacity)
{
    kObj(kMemory, memory);  

    if (obj->capacity < minimumCapacity)
    {
        kSize newCapacity = 0; 
        void* newBuffer = kNULL;  

        kCheckState(obj->owned);

        newCapacity = kMax_(minimumCapacity, xkMEMORY_GROWTH_FACTOR*obj->capacity);
        newCapacity = kMax_(newCapacity, xkMEMORY_MIN_CAPACITY); 

        kCheck(kObject_GetMem(memory, newCapacity, &newBuffer)); 

        kMemCopy(newBuffer, obj->buffer, obj->length);        
        
        kObject_FreeMem(memory, obj->buffer); 

        obj->buffer = (kByte*) newBuffer; 
        obj->capacity = newCapacity; 
    }
    
    return kOK; 
}

kFx(kStatus) xkMemory_VReadSomeImpl(kMemory memory, void* buffer, kSize minCount, kSize maxCount, kSize* bytesRead)
{
    kObj(kMemory, memory);  
    kSize copyCount; 
           
    kCheckState((kMemory_Position(memory) + minCount) <= obj->length); 

    //configure the stream for reading, if necessary
    if (obj->lastMode != xkMEMORY_MODE_READ)
    {
        kCheck(kStream_Flush(memory)); 

        obj->base.readBuffer = (kByte*) kPointer_ByteOffset(obj->buffer, (kSSize)obj->position); 
        obj->base.readCapacity = obj->length - obj->position; 
        obj->base.readBegin = 0; 
        obj->base.readEnd = obj->length - obj->position; 
        obj->base.writeBuffer = (kByte*)kSIZE_MAX; 
        obj->base.writeBegin = obj->base.writeEnd = obj->base.writeCapacity = 0; 

        obj->base.bytesRead += (obj->base.readEnd - obj->base.readBegin); 

        obj->lastMode = xkMEMORY_MODE_READ; 
    }

    //read the requested bytes
    copyCount = kMin_(maxCount, obj->base.readEnd - obj->base.readBegin); 

    kCheck(kMemCopy(buffer, &obj->base.readBuffer[obj->base.readBegin], copyCount)); 
    obj->base.readBegin += copyCount; 

    if (!kIsNull(bytesRead))
    {
        *bytesRead = copyCount; 
    }
    
    return kOK; 
}

kFx(kStatus) xkMemory_VWriteImpl(kMemory memory, const void* buffer, kSize size)
{
    kObj(kMemory, memory);  

    //extend the memory buffer, if necessary
    if ((kMemory_Position(memory) + size) > obj->capacity)
    {
        if (!obj->owned)
        {
            return kERROR_STREAM; 
        }
        
        kCheck(kStream_Flush(memory)); 
        kCheck(kMemory_Reserve(memory, (kSize)kMemory_Position(memory) + size)); 
    }

    //configure the stream for writing, if necessary
    if (obj->lastMode != xkMEMORY_MODE_WRITE)
    {
        kCheck(kStream_Flush(memory)); 

        obj->base.writeBuffer = (kByte*) kPointer_ByteOffset(obj->buffer, (kSSize)obj->position); 
        obj->base.writeBegin = 0; 
        obj->base.writeEnd = (obj->capacity - obj->position); 
        obj->base.writeCapacity = (obj->capacity - obj->position); 
        obj->base.readBuffer = (kByte*)kSIZE_MAX; 
        obj->base.readCapacity = 0; 
        obj->base.readBegin = 0; 
        obj->base.readEnd = 0; 
        obj->lastMode = xkMEMORY_MODE_WRITE; 
    }

    //write the requested bytes
    kCheck(kMemCopy(&obj->base.writeBuffer[obj->base.writeBegin], buffer, size)); 
    obj->base.writeBegin += size; 

    return kOK; 
}

kFx(kStatus) xkMemory_VSeek(kMemory memory, k64s offset, kSeekOrigin origin)
{
    kObj(kMemory, memory);  
    k64u originPosition;   
    k64u newPosition; 

    kCheck(kStream_Flush(memory)); 

    switch(origin)
    {
    case kSEEK_ORIGIN_BEGIN:        originPosition = 0;                 break;
    case kSEEK_ORIGIN_CURRENT:      originPosition = obj->position;     break;
    case kSEEK_ORIGIN_END:          originPosition = obj->length;       break;
    default:                        return kERROR_PARAMETER;
    }

    newPosition = (k64u) (originPosition + offset); 

    kCheckArgs(newPosition <= obj->length); 

    obj->position = (kSize) newPosition; 

    return kOK; 
}

kFx(kStatus) xkMemory_VFlush(kMemory memory)
{
    kObj(kMemory, memory);  
    
    if (obj->lastMode == xkMEMORY_MODE_READ)
    {
        obj->base.bytesRead -= (k64u) (obj->base.readEnd - obj->base.readBegin);
        obj->position += obj->base.readBegin; 
    }
    else if (obj->lastMode == xkMEMORY_MODE_WRITE)
    {
        obj->base.bytesWritten += (k64u)obj->base.writeBegin; 
        obj->position += obj->base.writeBegin; 
        obj->length = kMax_(obj->length, obj->position); 
    }
   
    obj->base.readBuffer = (kByte*)kSIZE_MAX; 
    obj->base.readCapacity = 0; 
    obj->base.readBegin = 0; 
    obj->base.readEnd = 0; 
    obj->base.writeBuffer = (kByte*)kSIZE_MAX; 
    obj->base.writeCapacity = 0; 
    obj->base.writeEnd = 0; 
    obj->base.writeBegin = 0; 

    obj->lastMode = xkMEMORY_MODE_NULL; 

    return kOK; 
}

kFx(kStatus) xkMemory_VFill(kMemory memory)
{
    kObj(kMemory, memory);  
           
    kCheckState(kMemory_Position(memory) < obj->length); 

    //configure the stream for reading, if necessary
    if (obj->lastMode != xkMEMORY_MODE_READ)
    {
        kCheck(kStream_Flush(memory)); 

        obj->base.readBuffer = (kByte*) kPointer_ByteOffset(obj->buffer, (kSSize)obj->position); 
        obj->base.readCapacity = obj->length - obj->position; 
        obj->base.readBegin = 0; 
        obj->base.readEnd = obj->length - obj->position; 
        obj->base.writeBuffer = (kByte*)kSIZE_MAX; 
        obj->base.writeBegin = obj->base.writeEnd = obj->base.writeCapacity = 0; 

        obj->base.bytesRead += (obj->base.readEnd - obj->base.readBegin); 

        obj->lastMode = xkMEMORY_MODE_READ; 
    }
    
    return kOK; 
}

kFx(kStatus) kMemory_SetLength(kMemory memory, kSize length)
{
    kObj(kMemory, memory);  
    
    kCheck(kStream_Flush(memory)); 
    kCheck(kMemory_Reserve(memory, length)); 

    obj->length = length; 
    obj->position = 0; 

    return kOK; 
}

kFx(k64u) kMemory_Length(kMemory memory)
{
    kObj(kMemory, memory);  

    if (obj->lastMode == xkMEMORY_MODE_WRITE)
    {
        return obj->length + obj->base.writeBegin; 
    }
    else
    {
        return obj->length; 
    }
}

kFx(kSize) kMemory_Capacity(kMemory memory)
{
    kObj(kMemory, memory);  

    return obj->capacity; 
}

kFx(k64u) kMemory_Position(kMemory memory)
{
    kObj(kMemory, memory);  

    if (obj->lastMode == xkMEMORY_MODE_WRITE)
    {
        return obj->position + obj->base.writeBegin; 
    }
    else if (obj->lastMode == xkMEMORY_MODE_READ)
    {
        return obj->position + obj->base.readBegin;
    }
    else
    {
        return obj->position; 
    }
}

kFx(void*) kMemory_At(kMemory memory, kSize offset)
{
    kObj(kMemory, memory);  

    return (offset > obj->length) ? kNULL : &obj->buffer[offset]; 
}

/** 
 * @file    kUdpClient.cpp
 *
 * @internal
 * Copyright (C) 2008-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <kApi/Io/kUdpClient.h>
#include <kApi/Io/kSocket.h>
#include <kApi/Threads/kTimer.h>

kBeginClassEx(k, kUdpClient)
    kAddPrivateVMethod(kUdpClient, kObject, VRelease)
    kAddPrivateVMethod(kUdpClient, kStream, VReadSomeImpl)
    kAddPrivateVMethod(kUdpClient, kStream, VWriteImpl)
    kAddPrivateVMethod(kUdpClient, kStream, VFlush)
    kAddPrivateVMethod(kUdpClient, kStream, VFill)
kEndClassEx()

kFx(kStatus) kUdpClient_Construct(kUdpClient* client, kIpVersion ipVersion, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback(allocator);
    kStatus status; 

    kCheck(kAlloc_GetObject(alloc, kTypeOf(kUdpClient), client)); 

    if (!kSuccess(status = xkUdpClient_Init(*client, kTypeOf(kUdpClient), ipVersion, alloc)))
    {
        kAlloc_FreeRef(alloc, client); 
    }

    return status; 
} 

kFx(kStatus) xkUdpClient_Init(kUdpClient client, kType type, kIpVersion ipVersion, kAlloc allocator)
{
    kObjR(kUdpClient, client); 
    kStatus status; 

    kCheck(kStream_Init(client, type, allocator)); 

    obj->socket = kNULL; 

    kTry
    {
        kTest(kSocket_Construct(&obj->socket, ipVersion, kSOCKET_TYPE_UDP, allocator));
        kTest(kSocket_SetBlocking(obj->socket, kFALSE));
    }
    kCatch(&status)
    {
        xkUdpClient_VRelease(client); 
        kEndCatch(status); 
    }
    
    return kOK;
}

kFx(kStatus) xkUdpClient_VRelease(kUdpClient client)
{
    kObj(kUdpClient, client); 
        
    kCheck(kObject_Destroy(obj->socket));    

    kCheck(kObject_FreeMem(client, obj->base.readBuffer));
    kCheck(kObject_FreeMem(client, obj->base.writeBuffer));                

    kCheck(kStream_VRelease(client)); 
    
    return kOK;
}

kFx(kStatus) kUdpClient_Bind(kUdpClient client, kIpAddress address, k32u port)
{
    kObj(kUdpClient, client);       

//Work-around for SYS/BIOS; see kUdpClient_EnableBroadcastReceive comments
#if defined(K_TI_BIOS) || defined (K_VX_KERNEL)
    if (obj->isBroacastReceiver)
    {
        address = kIpAddress_AnyV4(); 
    }
#endif

    return kSocket_Bind(obj->socket, address, port);
}

kFx(kStatus) kUdpClient_JoinMulticastGroup(kUdpClient client, kIpAddress group, kIpAddress iface)
{
    kObj(kUdpClient, client);       

    return kSocket_JoinMulticastGroup(obj->socket, group, iface);
}

kFx(kStatus) kUdpClient_LeaveMulticastGroup(kUdpClient client, kIpAddress group, kIpAddress iface)
{
    kObj(kUdpClient, client);       

    return kSocket_LeaveMulticastGroup(obj->socket, group, iface);
}

kFx(kStatus) kUdpClient_ReadFrom(kUdpClient client, kIpEndPoint* endPoint, void* buffer, kSize capacity, kSize* received, k64u timeout)
{
    kObj(kUdpClient, client); 
    kStatus opStatus; 

    opStatus = kSocket_ReadFrom(obj->socket, endPoint, buffer, capacity, received); 

    if ((opStatus == kERROR_BUSY) && (timeout > 0))
    {
        kCheck(kSocket_SetEvents(obj->socket, kSOCKET_EVENT_READ)); 
        kCheck(kSocket_Wait(obj->socket, timeout)); 
         
        opStatus = kSocket_ReadFrom(obj->socket, endPoint, buffer, capacity, received); 
    }

    if (kSuccess(opStatus))
    {
        obj->base.bytesRead += (k64u) *received; 
    }

    return opStatus; 
}

kFx(kStatus) kUdpClient_WriteTo(kUdpClient client, const void* buffer, kSize size, kIpAddress address, k32u port, k64u timeout)
{
    kObj(kUdpClient, client);     
    kStatus opStatus; 

    opStatus = kSocket_WriteTo(obj->socket, address, port, buffer, size); 

    if ((opStatus == kERROR_BUSY) && (timeout > 0))
    {
        k64u startTime = kTimer_Now(); 
        k64u elapsed = 0; 
        k64u opTimeout = 0; 

        kCheck(kSocket_SetEvents(obj->socket, kSOCKET_EVENT_WRITE)); 
        
        do
        {
            elapsed = kTimer_Now() - startTime; 

            if (elapsed < timeout)
            {
                opTimeout = (timeout == kINFINITE) ? timeout : timeout - elapsed; 

                if (kSuccess(kSocket_Wait(obj->socket, opTimeout)))
                {
                    opStatus = kSocket_WriteTo(obj->socket, address, port, buffer, size); 
                }
            }
        }
        while ((opStatus == kERROR_BUSY) && (elapsed < timeout)); 
    }

    if (opStatus == kERROR_BUSY)
    {
        opStatus = kERROR_TIMEOUT; 
    }

    if (kSuccess(opStatus))
    {
        obj->base.bytesWritten += (k64u) size; 
    }
   
    return opStatus; 
}

kFx(kStatus) kUdpClient_Receive(kUdpClient client, kIpEndPoint* endPoint, kSize* received, k64u timeout)
{
    kObj(kUdpClient, client); 
    kStatus opStatus; 

    kCheckState(!kIsNull(obj->base.readBuffer)); 

    opStatus = kSocket_ReadFrom(obj->socket, endPoint, obj->base.readBuffer, obj->base.readCapacity, &obj->base.readEnd); 

    if ((opStatus == kERROR_BUSY) && (timeout > 0))
    {
        kCheck(kSocket_SetEvents(obj->socket, kSOCKET_EVENT_READ)); 
        kCheck(kSocket_Wait(obj->socket, timeout)); 
        
        opStatus = kSocket_ReadFrom(obj->socket, endPoint, obj->base.readBuffer, obj->base.readCapacity, &obj->base.readEnd); 
    }

    if (kSuccess(opStatus))
    {
        obj->base.readBegin = 0; 
        obj->base.bytesRead += (k64u) obj->base.readEnd; 

        if (!kIsNull(received))
        {
            *received = obj->base.readEnd;
        }
    }
    
    return opStatus;
}

kFx(kStatus) kUdpClient_ReceiveEx(kUdpClient client, kIpEndPoint* endPoint, kSize* received, k64u timeout, kSize* interfaceIndex)
{
    kObj(kUdpClient, client);
    kStatus opStatus;

    kCheckState(!kIsNull(obj->base.readBuffer));

    opStatus = kSocket_ReadFromEx(obj->socket, endPoint, interfaceIndex, obj->base.readBuffer, obj->base.readCapacity, &obj->base.readEnd);

    if ((opStatus == kERROR_BUSY) && (timeout > 0))
    {
        kCheck(kSocket_SetEvents(obj->socket, kSOCKET_EVENT_READ));
        kCheck(kSocket_Wait(obj->socket, timeout));

        opStatus = kSocket_ReadFromEx(obj->socket, endPoint, interfaceIndex, obj->base.readBuffer, obj->base.readCapacity, &obj->base.readEnd);
    }

    if (kSuccess(opStatus))
    {
        obj->base.readBegin = 0;
        obj->base.bytesRead += (k64u)obj->base.readEnd;

        if (!kIsNull(received))
        {
            *received = obj->base.readEnd;
        }
    }

    return opStatus;
}

kFx(kStatus) kUdpClient_Send(kUdpClient client, kIpAddress address, k32u port, k64u timeout, kBool clear)
{    
    kObj(kUdpClient, client); 
    kStatus opStatus; 
    
    kCheckState(!kIsNull(obj->base.writeBuffer)); 

    opStatus = kSocket_WriteTo(obj->socket, address, port, obj->base.writeBuffer, obj->base.writeBegin); 

    if ((opStatus == kERROR_BUSY) && (timeout > 0))
    {
        k64u startTime = kTimer_Now(); 
        k64u elapsed = 0; 
        k64u opTimeout = 0; 

        kCheck(kSocket_SetEvents(obj->socket, kSOCKET_EVENT_WRITE)); 

        do
        {
            elapsed = kTimer_Now() - startTime; 

            if (elapsed < timeout)
            {
                opTimeout = (timeout == kINFINITE) ? timeout : timeout - elapsed; 

                if (kSuccess(kSocket_Wait(obj->socket, opTimeout)))
                {
                    opStatus = kSocket_WriteTo(obj->socket, address, port, obj->base.writeBuffer, obj->base.writeBegin); 
                }
            }
        }
        while ((opStatus == kERROR_BUSY) && (elapsed < timeout)); 
    }

    if (opStatus == kERROR_BUSY)
    {
        opStatus = kERROR_TIMEOUT; 
    }

    if (kSuccess(opStatus))
    {
        obj->base.bytesWritten += (k64u)obj->base.writeBegin; 
    }

    if (clear)
    {
        obj->base.writeBegin = 0; 
    }

    return opStatus; 
}

kFx(kStatus) kUdpClient_Clear(kUdpClient client)
{
    kObj(kUdpClient, client); 

    obj->base.writeBegin = 0; 

    return kOK; 
}

kFx(kStatus) xkUdpClient_VReadSomeImpl(kUdpClient client, void* buffer, kSize minCount, kSize maxCount, kSize* bytesRead)
{
    kObj(kUdpClient, client); 
    kSize copyCount; 
    
    kCheckState(!kIsNull(obj->base.readBuffer)); 
    kCheckState(minCount <= (obj->base.readEnd - obj->base.readBegin)); 
      
    copyCount = kMin_(maxCount, obj->base.readEnd - obj->base.readBegin); 

    kMemCopy(buffer, &obj->base.readBuffer[obj->base.readBegin], copyCount); 
    obj->base.readBegin += copyCount; 

    if (!kIsNull(bytesRead))
    {
        *bytesRead = copyCount; 
    }
         
    return kOK;
}

kFx(kStatus) xkUdpClient_VWriteImpl(kUdpClient client, const void* buffer, kSize size)
{
    kObj(kUdpClient, client); 

    kCheckState(!kIsNull(obj->base.writeBuffer)); 
    kCheckState(size <= (obj->base.writeEnd - obj->base.writeBegin)); 

    kMemCopy(&obj->base.writeBuffer[obj->base.writeBegin], buffer, size); 
    obj->base.writeBegin += size; 
    
    return kOK;
}

kFx(kStatus) xkUdpClient_VFlush(kUdpClient client)
{
    return kOK; 
}

kFx(kStatus) xkUdpClient_VFill(kUdpClient client)
{
    return kERROR_STATE;
}

kFx(kStatus) kUdpClient_EnableBroadcast(kUdpClient client, kBool broadcast)
{
    kObj(kUdpClient, client);

    return kSocket_EnableBroadcast(obj->socket, broadcast);
}

kFx(kStatus) kUdpClient_EnablePacketInfo(kUdpClient client, kBool enabled)
{
    kObj(kUdpClient, client);

    return kSocket_EnablePacketInfo(obj->socket, enabled);
}

kFx(kStatus) kUdpClient_EnableBroadcastReceive(kUdpClient client, kBool broadcast)
{
    kObj(kUdpClient, client); 
    
    obj->isBroacastReceiver = broadcast; 

    return kOK;
}

kFx(kStatus) kUdpClient_EnableReuseAddress(kUdpClient client, kBool reuse)
{
    kObj(kUdpClient, client); 
    
    return kSocket_EnableReuseAddress(obj->socket, reuse);
}

kFx(kStatus) kUdpClient_SetWriteBuffers(kUdpClient client, kSSize socketSize, kSSize clientSize)
{
    kObj(kUdpClient, client); 
        
    kCheckState(obj->base.writeBegin == 0);
    
    if (socketSize >= 0)
    {
        kCheck(kSocket_SetWriteBuffer(obj->socket, (kSize)socketSize));
    }

    if (clientSize >= 0)
    {
        kCheck(kObject_FreeMemRef(client, &obj->base.writeBuffer)); 
        
        obj->base.writeCapacity = 0; 
        obj->base.writeBegin = 0; 
        obj->base.writeEnd = 0; 

        if (clientSize > 0)
        {
            kCheck(kObject_GetMem(client, (kSize)clientSize, &obj->base.writeBuffer)); 
            obj->base.writeCapacity = (kSize)clientSize; 
            obj->base.writeEnd = (kSize)clientSize; 
        }
    }

    return kOK; 
}

kFx(kStatus) kUdpClient_SetReadBuffers(kUdpClient client, kSSize socketSize, kSSize clientSize)
{
    kObj(kUdpClient, client); 

    kCheckState((obj->base.readEnd - obj->base.readBegin) == 0);

    if (socketSize >= 0)
    {
        kCheck(kSocket_SetReadBuffer(obj->socket, (kSize)socketSize));
    }

    if (clientSize >= 0)
    {
        kCheck(kObject_FreeMemRef(client, &obj->base.readBuffer)); 

        obj->base.readCapacity = 0; 
        obj->base.readBegin = 0; 
        obj->base.readEnd = 0; 

        if (clientSize > 0)
        {
            kCheck(kObject_GetMem(client, (kSize)clientSize, &obj->base.readBuffer));                 
            obj->base.readCapacity = (kSize)clientSize; 
        }
    }
    
    return kOK;
}

kFx(kSocket) kUdpClient_Socket(kUdpClient client)
{
    kObj(kUdpClient, client); 
        
    return obj->socket;
}

kFx(kStatus) kUdpClient_LocalEndPoint(kUdpClient client, kIpEndPoint* endPoint)
{
    kObj(kUdpClient, client); 
        
    return kSocket_LocalEndPoint(obj->socket,endPoint);  
}

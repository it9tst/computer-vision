/** 
 * @file    kNetwork.cpp
 *
 * @internal
 * Copyright (C) 2008-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#define K_PLATFORM
#include <kApi/Io/kNetwork.h>

#include <kApi/Data/kArrayList.h>
#include <kApi/Data/kMap.h>
#include <kApi/Io/kSocket.h>
#include <kApi/Threads/kLock.h>
#include <kApi/Threads/kThread.h>
#include <kApi/Threads/kSemaphore.h>
#include <kApi/Utils/kEvent.h>

#include <stdio.h>

/* 
 * kIpVersion enum
 */
kBeginEnumEx(k, kIpVersion)
    kAddEnumerator(kIpVersion, kIP_VERSION_4)
kEndEnumEx()

/* 
 * kIpAddress structure
 */

kBeginValueEx(k, kIpAddress)
    kAddField(kIpAddress, kIpVersion, version)
    kAddField(kIpAddress, kByte, address)

    kAddPrivateVMethod(kIpAddress, kValue, VEquals)
    kAddPrivateVMethod(kIpAddress, kValue, VHashCode)
kEndValueEx()

kFx(kBool) xkIpAddress_VEquals(kType type, const void* value, const void* other)
{
    const kIpAddress* a = (const kIpAddress*) value; 
    const kIpAddress* b = (const kIpAddress*) other; 

    if (a->version == b->version)
    {
        if (a->version == kIP_VERSION_4)
        {
            return kMemEquals(a->address, b->address, 4); 
        }
    }

    return kFALSE;
}

kFx(kSize) xkIpAddress_VHashCode(kType type, const void* value)
{
    const kIpAddress* address = (const kIpAddress*) value; 

    if (address->version == kIP_VERSION_4)
    {
        return xkHashBytes(address->address, 4); 
    }
    else
    {
        return xkHashBytes(address->address, sizeof(address->address)); 
    }
}

kFx(kIpAddress) kIpAddress_Any(kIpVersion version)
{    
    kAssert(version == kIP_VERSION_4); 

    return kIpAddress_AnyV4(); 
}

kFx(kIpAddress) kIpAddress_AnyV4()
{
    kIpAddress address = { kIP_VERSION_4 }; 

    return address; 
}

kFx(kIpAddress) kIpAddress_BroadcastV4()
{   
    kIpAddress address = { kIP_VERSION_4 }; 

    address.address[0] = 0xFF; 
    address.address[1] = 0xFF; 
    address.address[2] = 0xFF; 
    address.address[3] = 0xFF; 

    return address;   
}

kFx(kIpAddress) kIpAddress_Loopback(kIpVersion version)
{
    kAssert(version == kIP_VERSION_4); 

    return kIpAddress_LoopbackV4(); 
}

kFx(kIpAddress) kIpAddress_LoopbackV4()
{
    kIpAddress address = { kIP_VERSION_4 }; 

    address.address[0] = 0x7F; 
    address.address[1] = 0x00; 
    address.address[2] = 0x00; 
    address.address[3] = 0x01; 

    return address; 
}

kFx(kStatus) xkIpAddress_ToSockAddr(kIpAddress address, k32u port, struct sockaddr_in *sockAddr)
{
    kMemSet(sockAddr, 0, sizeof(struct sockaddr_in));

    sockAddr->sin_family = AF_INET;
    sockAddr->sin_addr.s_addr = kIpAddress_ToNet32u(address); 
    sockAddr->sin_port = htons((k16u)port); 

    return kOK;
}

kFx(kStatus) xkIpAddress_FromSockAddr(struct sockaddr_in* sockAddr, kIpAddress *address, k32u *port)
{   
    if (!kIsNull(address))
    {
        *address = kIpAddress_FromNet32u(sockAddr->sin_addr.s_addr); 
    }

    if (!kIsNull(port))
    {
        *port =  htons(sockAddr->sin_port); 
    }

    return kOK;
}

kFx(kStatus) kIpAddress_Parse(kIpAddress* address, const kChar* text)
{
    k32u a, b, c, d;

    if ((sscanf(text, "%u.%u.%u.%u", &a, &b, &c, &d) != 4) || 
        (a > k8U_MAX) || (b > k8U_MAX) || (c > k8U_MAX) || (d > k8U_MAX))
    {
        return kERROR_PARAMETER; 
    }

    kItemZero(address, sizeof(kIpAddress)); 

    address->version = kIP_VERSION_4; 

    address->address[0] = (kByte) a; 
    address->address[1] = (kByte) b; 
    address->address[2] = (kByte) c; 
    address->address[3] = (kByte) d; 
     
    return kOK;
}

kFx(kStatus) kIpAddress_Format(kIpAddress address, kChar* text, kSize capacity)
{        
    kCheckArgs(address.version == kIP_VERSION_4); 

    kStrPrintf(text, capacity, "%u.%u.%u.%u", address.address[0], address.address[1], address.address[2], address.address[3]); 
    
    return kOK;
}

kFx(kBool) kIpAddress_Equals(kIpAddress a, kIpAddress b)
{
    if (a.version == kIP_VERSION_4)
    {
        return (b.version == a.version) && kMemEquals(&a.address[0], &b.address[0], 4); 
    }
    else
    {
        return kFALSE; 
    }    
}

kFx(kBool) kIpAddress_IsLoopback(kIpAddress address)
{
    return kIpAddress_Equals(address, kIpAddress_Loopback(address.version)); 
}

kFx(kBool) kIpAddress_IsLinkLocal(kIpAddress address)
{
    if (address.version == kIP_VERSION_4)
    {
        return (address.address[0] == 169) && (address.address[1] == 254);
    }
    else
    {
        return kFALSE; 
    }   
}

kFx(k32u) kIpAddress_ToHost32u(kIpAddress address)
{
    kAssert(address.version == kIP_VERSION_4); 

    return (k32u) ((address.address[0] << 24) | (address.address[1] << 16) | (address.address[2] << 8) | (address.address[3])); 
}

kFx(k32u) kIpAddress_ToNet32u(kIpAddress address)
{
    kAssert(address.version == kIP_VERSION_4); 

    return (k32u) ((address.address[3] << 24) | (address.address[2] << 16) | (address.address[1] << 8) | (address.address[0])); 
}

kFx(kIpAddress) kIpAddress_FromHost32u(k32u address)
{
    kIpAddress out = { kIP_VERSION_4 }; 

    out.address[0] = (k8u) ((address >> 24) & 0xFF);
    out.address[1] = (k8u) ((address >> 16) & 0xFF);
    out.address[2] = (k8u) ((address >>  8) & 0xFF);
    out.address[3] = (k8u) (address & 0xFF);

    return out; 
}

kFx(kIpAddress) kIpAddress_FromNet32u(k32u address)
{
    kIpAddress out = { kIP_VERSION_4 }; 

    out.address[3] = (k8u) ((address >> 24) & 0xFF);
    out.address[2] = (k8u) ((address >> 16) & 0xFF);
    out.address[1] = (k8u) ((address >>  8) & 0xFF);
    out.address[0] = (k8u) (address & 0xFF);

    return out; 
}

kFx(kIpAddress) kIpAddress_Network(kIpAddress address, k32u prefixLength)
{
    kAssert(address.version = kIP_VERSION_4);

    k32u mask = xkIpAddress_SubnetPrefixToMask(prefixLength);

    return kIpAddress_FromHost32u(kIpAddress_ToHost32u(address) & mask);
}

kFx(kBool) kIpAddress_NetworkEquals(kIpAddress a, kIpAddress b, k32u prefixLength) 
{
    return kIpAddress_Equals(kIpAddress_Network(a, prefixLength), kIpAddress_Network(b, prefixLength));
}

kFx(k32u) xkIpAddress_SubnetPrefixToMask(k32u prefix)
{
    k32u mask = 0; 
    k32s lower = 32 - (k32s) prefix; 
    k32s i = 0; 

    for (i = 31; i >= lower; --i)
    {
        mask |= (1 << i); 
    }

    return mask; 
}

kFx(k32u) xkIpAddress_SubnetMaskToPrefix(k32u mask)
{
    k32u i; 

    for (i = 0; i < 32; ++i)
    {
        if (mask & (1 << i))
        {
            return (32 - i); 
        }
    }

    return 0; 
}

/* 
 * kIpEndPoint structure
 */

kBeginValueEx(k, kIpEndPoint)
    kAddField(kIpEndPoint, kIpAddress, address)
    kAddField(kIpEndPoint, k32u, port)

    kAddPrivateVMethod(kIpEndPoint, kValue, VEquals)

kEndValueEx()

kFx(kBool) xkIpEndPoint_VEquals(kType type, const void* value, const void* other)
{
    const kIpEndPoint* a = (const kIpEndPoint*) value; 
    const kIpEndPoint* b = (const kIpEndPoint*) other; 

    return kValue_Equals(kTypeOf(kIpAddress), &a->address, &b->address) && (a->port == b->port); 
}

/*
 * [Deprecated] kIpEntry structure
 */
 
kBeginValueEx(k, kIpEntry)
    kAddField(kIpEntry, kIpAddress, address)
    kAddField(kIpEntry, kText128, name)
kEndValueEx()

/* 
 * kNetworkInterface class
 */
kBeginClassEx(k, kNetworkInterface)    
kEndClassEx()

kFx(kStatus) xkNetworkInterface_Construct(kNetworkInterface* iface, kNetworkAdapter adapter, kIpAddress address, k32u prefixLength, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback(allocator);
    kStatus status; 

    kCheck(kAlloc_GetObject(alloc, kTypeOf(kNetworkInterface), iface)); 

    if (!kSuccess(status = xkNetworkInterface_Init(*iface, kTypeOf(kNetworkInterface), adapter, address, prefixLength, alloc)))
    {
        kAlloc_FreeRef(alloc, iface); 
    }

    return status; 
}

kFx(kStatus) xkNetworkInterface_Init(kNetworkInterface iface, kType type, kNetworkAdapter adapter, kIpAddress address, k32u prefixLength, kAlloc alloc)
{
    kObjR(kNetworkInterface, iface); 

    kCheck(kObject_Init(iface, type, alloc)); 

    obj->adapter = adapter; 
    obj->address = address;
    obj->prefixLength = prefixLength;

    return kOK; 
}

/* 
 * kNetworkAdapter class
 */
kBeginClassEx(k, kNetworkAdapter)    
    kAddPrivateVMethod(kNetworkAdapter, kObject, VRelease)
kEndClassEx()

kFx(kStatus) xkNetworkAdapter_Construct(kNetworkAdapter* adapter, kNetworkInfo info, const kChar* name, kSize id, kBool isUp, kMacAddress macAddress, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback(allocator);
    kStatus status; 

    kCheck(kAlloc_GetObject(alloc, kTypeOf(kNetworkAdapter), adapter)); 

    if (!kSuccess(status = xkNetworkAdapter_Init(*adapter, kTypeOf(kNetworkAdapter), info, name, id, isUp, macAddress, alloc)))
    {
        kAlloc_FreeRef(alloc, adapter); 
    }

    return status; 
}

kFx(kStatus) xkNetworkAdapter_Init(kNetworkAdapter adapter, kType type, kNetworkInfo info, const kChar* name, kSize id, kBool isUp, kMacAddress macAddress, kAlloc alloc)
{
    kObjR(kNetworkAdapter, adapter); 
    kStatus status = kOK; 

    kCheck(kObject_Init(adapter, type, alloc)); 

    obj->info = info; 
    obj->name = kNULL; 
    obj->id = id; 
    obj->isUp = isUp;
    obj->interfaces = kNULL;
    obj->macAddress = macAddress;

    kTry
    {
        kTest(kString_Construct(&obj->name, name, alloc));

        kTest(kArrayList_Construct(&obj->interfaces, kTypeOf(kNetworkInterface), 0, alloc));
    }
    kCatch(&status); 
    {
        xkNetworkAdapter_VRelease(adapter); 
        kEndCatch(status); 
    }

    return kOK; 
}

kFx(kStatus) xkNetworkAdapter_VRelease(kNetworkAdapter adapter)
{
    kObj(kNetworkAdapter, adapter); 

    kCheck(kObject_Destroy(obj->name));
    kCheck(kObject_Dispose(obj->interfaces));

    kCheck(kObject_VRelease(adapter));

    return kOK;
}

kFx(kStatus) xkNetworkAdapter_AddInterface(kNetworkAdapter adapter, kIpAddress address, k32u prefixLength, kNetworkInterface* iface)
{    
    kObj(kNetworkAdapter, adapter); 
    kNetworkInterface newIface = kNULL;

    kTry
    {
        kTest(xkNetworkInterface_Construct(&newIface, adapter, address, prefixLength, kObject_Alloc(adapter)));

        kTest(kArrayList_Add(obj->interfaces, &newIface));

        if (!kIsNull(iface))
        {
            *iface = newIface;
        }

        newIface = kNULL;
    }
    kFinally
    {
        kDestroyRef(&newIface);

        kEndFinally();
    }

    return kOK;
}

/* 
 * kNetworkInfo class
 */
kBeginClassEx(k, kNetworkInfo)    
    kAddPrivateVMethod(kNetworkInfo, kObject, VRelease)
kEndClassEx()

kFx(kStatus) kNetworkInfo_Construct(kNetworkInfo* info, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback(allocator);
    kStatus status; 

    kCheck(kAlloc_GetObject(alloc, kTypeOf(kNetworkInfo), info)); 

    if (!kSuccess(status = xkNetworkInfo_Init(*info, kTypeOf(kNetworkInfo), alloc)))
    {
        kAlloc_FreeRef(alloc, info); 
    }

    return status; 
}

kFx(kStatus) xkNetworkInfo_Init(kNetworkInfo info, kType type, kAlloc alloc)
{
    kObjR(kNetworkInfo, info); 
    kStatus status = kOK; 

    kCheck(kObject_Init(info, type, alloc)); 

    obj->adapters = kNULL; 
    obj->interfaces = kNULL;

    kTry
    {
        kTest(kArrayList_Construct(&obj->adapters, kTypeOf(kNetworkAdapter), 0, alloc));
        kTest(kArrayList_Construct(&obj->interfaces, kTypeOf(kNetworkInterface), 0, alloc));

        kTest(kNetworkInfo_Refresh(info));
    }
    kCatch(&status); 
    {
        xkNetworkInfo_VRelease(info); 
        kEndCatch(status); 
    }

    return kOK; 
}

kFx(kStatus) xkNetworkInfo_VRelease(kNetworkInfo info)
{
    kObj(kNetworkInfo, info); 

    kCheck(kObject_Destroy(obj->interfaces));

    kCheck(kObject_Dispose(obj->adapters));

    kCheck(kObject_VRelease(info));

    return kOK;
}

kFx(kStatus) kNetworkInfo_Refresh(kNetworkInfo info)
{
    kObj(kNetworkInfo, info); 

    //clear existing data
    kCheck(kArrayList_Purge(obj->adapters));
    kCheck(kArrayList_Clear(obj->interfaces));

    //enumerate adapters
    kCheck(kApiLib_QueryNetInfoHandler()(info));

    //create flat list of interface references, for convenient access
    for (kSize i = 0; i < kNetworkInfo_AdapterCount(info); ++i)
    {
        kNetworkAdapter adapter = kNetworkInfo_AdapterAt(info, i); 
        
        for (kSize j = 0; j < kNetworkAdapter_InterfaceCount(adapter); ++j)
        {
            kNetworkInterface iface = kNetworkAdapter_InterfaceAt(adapter, j); 
 
            kCheck(kArrayList_AddT(obj->interfaces, &iface));
        }
    }

    return kOK;
}

kFx(kStatus) xkNetworkInfo_AddAdapter(kNetworkInfo info, const kChar* name, kSize id, kBool isUp, kMacAddress macAddress, kNetworkAdapter* adapter)
{    
    kObj(kNetworkInfo, info); 
    kNetworkAdapter newAdapter = kNULL;

    kTry
    {
        kTest(xkNetworkAdapter_Construct(&newAdapter, adapter, name, id, isUp, macAddress, kObject_Alloc(info)));

        kTest(kArrayList_Add(obj->adapters, &newAdapter));

        if (!kIsNull(adapter))
        {
            *adapter = newAdapter;
        }

        newAdapter = kNULL;
    }
    kFinally
    {
        kDestroyRef(&newAdapter);

        kEndFinally();
    }

    return kOK;
}

kFx(kStatus) kNetworkInfo_FindAdapterByName(kNetworkInfo info, const kChar* name, kNetworkAdapter* adapter)
{    
    kObj(kNetworkInfo, info); 

    for (kSize i = 0; i < kNetworkInfo_AdapterCount(info); ++i)
    {
        kNetworkAdapter adapterAt = kNetworkInfo_AdapterAt(info, i); 

        if (kStrEquals(kNetworkAdapter_Name(adapterAt), name))
        {
            if (!kIsNull(adapter))
            {
                *adapter = adapterAt;
            }
            return kOK;
        }
    }

    return kERROR_NOT_FOUND;
}

kFx(kStatus) kNetworkInfo_FindAdapterById(kNetworkInfo info, kSize id, kNetworkAdapter* adapter)
{    
    kObj(kNetworkInfo, info); 

    for (kSize i = 0; i < kNetworkInfo_AdapterCount(info); ++i)
    {
        kNetworkAdapter adapterAt = kNetworkInfo_AdapterAt(info, i); 

        if (kNetworkAdapter_Id(adapterAt) == id)
        {
            if (!kIsNull(adapter))
            {
                *adapter = adapterAt;
            }
            return kOK;
        }
    }

    return kERROR_NOT_FOUND;
} 

kFx(kStatus) kNetworkInfo_FindInterfaceByAddress(kNetworkInfo info, kIpAddress address, kNetworkInterface* iface)
{    
    kObj(kNetworkInfo, info); 

    for (kSize i = 0; i < kNetworkInfo_InterfaceCount(info); ++i)
    {
        kNetworkInterface interfaceAt = kNetworkInfo_InterfaceAt(info, i); 

        if (kIpAddress_Equals(kNetworkInterface_Address(interfaceAt), address))
        {
            if (!kIsNull(iface))
            {
                *iface = interfaceAt;
            }
            return kOK;
        }
    }

    return kERROR_NOT_FOUND;
} 

kFx(kStatus) kNetworkInfo_FindInterfaceByNetwork(kNetworkInfo info, kIpAddress address, kNetworkInterface* iface)
{    
    kObj(kNetworkInfo, info); 

    for (kSize i = 0; i < kNetworkInfo_InterfaceCount(info); ++i)
    {
        kNetworkInterface interfaceAt = kNetworkInfo_InterfaceAt(info, i); 
   
        if (kIpAddress_NetworkEquals(address, kNetworkInterface_Address(interfaceAt), kNetworkInterface_PrefixLength(interfaceAt)))
        {
            if (!kIsNull(iface))
            {
                *iface = interfaceAt;
            }
            return kOK;
        }
    }

    return kERROR_NOT_FOUND;
} 

/* 
 * kNetwork static class
 */
kBeginStaticClassEx(k, kNetwork)    
kEndStaticClassEx()

kFx(kStatus) xkNetwork_InitStatic()
{
    kStaticObj(kNetwork);

    kAtomic32s_Init(&sobj->shouldQuit, kFALSE); 
    sobj->eventLock = kNULL; 
    sobj->changeEvent = kNULL;
    sobj->changeThread = kNULL;

    kCheck(kLock_Construct(&sobj->eventLock, kNULL));
    kCheck(kEvent_Construct(&sobj->changeEvent, kNULL));

    kCheck(xkNetwork_InitStaticPlatform());
 
    return kOK; 
}

kFx(kStatus) xkNetwork_ReleaseStatic()
{    
    kStaticObj(kNetwork);

    kAtomic32s_Exchange(&sobj->shouldQuit, kTRUE);

    kCheck(kDestroyRef(&sobj->changeThread));

    kCheck(kDestroyRef(&sobj->changeEvent));
    kCheck(kDestroyRef(&sobj->eventLock));

    kCheck(xkNetwork_ReleaseStaticPlatform());

    return kOK; 
}

kFx(kStatus) kNetwork_AddChangeHandler(kCallbackFx function, kPointer receiver)
{
    kStaticObj(kNetwork);

    kLock_Enter(sobj->eventLock); 

    kTry
    {
        if (kIsNull(sobj->changeThread))
        {
            kAtomic32s_Exchange(&sobj->shouldQuit, kFALSE);

            kTest(kThread_Construct(&sobj->changeThread, kNULL)); 
            kTest(kThread_StartEx(sobj->changeThread, xkNetwork_ChangeThreadEntry, kNULL, 0, "kNetwork.Change", 0));
        }

        kTest(kEvent_Add(sobj->changeEvent, function, receiver));
    }
    kFinally
    {
        kLock_Exit(sobj->eventLock);

        kEndFinally();
    }

    return kOK;
}

kFx(kStatus) kNetwork_RemoveChangeHandler(kCallbackFx function, kPointer receiver)
{
    kStaticObj(kNetwork);

    kLock_Enter(sobj->eventLock); 

    kTry
    {
        kTest(kEvent_Remove(sobj->changeEvent, function, receiver));

        if (kEvent_Count(sobj->changeEvent) == 0)
        {
            kAtomic32s_Exchange(&sobj->shouldQuit, kTRUE);

            kTest(kDestroyRef(&sobj->changeThread));
        }
    }
    kFinally
    {
        kLock_Exit(sobj->eventLock);

        kEndFinally();
    }

    return kOK;
}

kFx(kStatus) xkNetwork_ChangeThreadEntry(kPointer context)
{
    kStaticObj(kNetwork);

    if (kSuccess(xkNetwork_OsBeginChangeNotification()))
    {
        while (!kAtomic32s_Get(&sobj->shouldQuit))
        {
            if (kApiLib_QueryNetChangeHandler()(xkNETWORK_QUIT_QUERY_PERIOD))
            {
                xkNetwork_NotifyChange();
            }
        }

        xkNetwork_OsEndChangeNotification(); 
    }

    return kOK;
}

kFx(kStatus) xkNetwork_NotifyChange()
{
    kStaticObj(kNetwork);

    kLock_Enter(sobj->eventLock); 

    kTry
    {
        kTest(kEvent_Notify(sobj->changeEvent, kNULL, kNULL));
    }
    kFinally
    {
        kLock_Exit(sobj->eventLock);

        kEndFinally();
    }

    return kOK;
}

kFx(kStatus) kNetwork_FindAdapterIdByName(const kChar* name, kSize* id)
{
    kNetworkInfo info = kNULL; 
    kNetworkAdapter adapter = kNULL;

    kTry
    {
        kTest(kNetworkInfo_Construct(&info, kNULL)); 
        
        kTest(kNetworkInfo_FindAdapterByName(info, name, &adapter)); 

        if (!kIsNull(id))
        {
            *id = kNetworkAdapter_Id(adapter);
        }
    }
    kFinally
    {
        kDestroyRef(&info);

        kEndFinally();
    }

    return kOK;
}

//[Deprecated]
kFx(kStatus) kNetwork_LocalIpInterfacesEx(kArrayList interfaces, kBool includeLoopback)
{
    kNetworkInfo info = kNULL; 

    kTry
    {
        kTest(kNetworkInfo_Construct(&info, kNULL)); 

        kTest(kArrayList_Allocate(interfaces, kTypeOf(kIpEntry), 0));

        for (kSize i = 0; i < kNetworkInfo_InterfaceCount(info); ++i)
        {
            kNetworkInterface interfaceAt = kNetworkInfo_InterfaceAt(info, i);
            kNetworkAdapter adapter = kNetworkInterface_Adapter(interfaceAt);
            kIpAddress address = kNetworkInterface_Address(interfaceAt);
            const kChar* name = kNetworkAdapter_Name(adapter);
 
            if (includeLoopback || !kIpAddress_IsLoopback(address))
            {
                kTest(kArrayList_AddCount(interfaces, 1)); 
                
                kIpEntry* entry = kArrayList_LastT(interfaces, kIpEntry);

                entry->address = address; 
                kTest(kStrCopy(entry->name, sizeof(entry->name), name));
            }
        }        
    }
    kFinally
    {
        kDestroyRef(&info);

        kEndFinally();
    }

    return kOK;
}

kFx(kStatus) kNetwork_FindAdapterNameById(kSize id, kChar* name, kSize capacity)
{
    kNetworkInfo info = kNULL; 
    kNetworkAdapter adapter = kNULL;

    kTry
    {
        kTest(kNetworkInfo_Construct(&info, kNULL)); 
        
        kTest(kNetworkInfo_FindAdapterById(info, id, &adapter)); 

        if (!kIsNull(name))
        {
            kTest(kStrCopy(name, capacity, kNetworkAdapter_Name(adapter))); 
        }
    }
    kFinally
    {
        kDestroyRef(&info);

        kEndFinally();
    }

    return kOK;
}

kFx(kStatus) kNetwork_FindAdapterNameByInterface(kIpAddress address, kChar* adapterName, kSize capacity)
{
    kNetworkInfo info = kNULL; 
    kNetworkInterface iface = kNULL;

    kTry
    {
        kTest(kNetworkInfo_Construct(&info, kNULL)); 
        
        kTest(kNetworkInfo_FindInterfaceByAddress(info, address, &iface)); 

        if (!kIsNull(adapterName))
        {
            kTest(kStrCopy(adapterName, capacity, kNetworkAdapter_Name(kNetworkInterface_Adapter(iface)))); 
        }
    }
    kFinally
    {
        kDestroyRef(&info);

        kEndFinally();
    }

    return kOK;
}

kFx(kStatus) kNetwork_FindFirstAdapterInterface(const kChar* adapterName, kIpAddress* address)
{
    kNetworkInfo info = kNULL; 
    kNetworkAdapter adapter = kNULL;

    kTry
    {
        kTest(kNetworkInfo_Construct(&info, kNULL)); 
        
        kTest(kNetworkInfo_FindAdapterByName(info, adapterName, &adapter)); 

        kTestTrue(kNetworkAdapter_InterfaceCount(adapter) > 0, kERROR_NOT_FOUND);

        if (!kIsNull(address))
        {
            *address = kNetworkInterface_Address(kNetworkAdapter_InterfaceAt(adapter, 0));
        }
    }
    kFinally
    {
        kDestroyRef(&info);

        kEndFinally();
    }

    return kOK;
}

kFx(kStatus) kNetwork_FindInterfaceByNetwork(kIpAddress network, kIpAddress* interfaceAddress)
{
    kNetworkInfo info = kNULL; 
    kNetworkInterface iface = kNULL;

    kTry
    {
        kTest(kNetworkInfo_Construct(&info, kNULL)); 
        
        kTest(kNetworkInfo_FindInterfaceByNetwork(info, network, &iface));
        
        if (!kIsNull(interfaceAddress))
        {
            *interfaceAddress = kNetworkInterface_Address(iface);
        }
    }
    kFinally
    {
        kDestroyRef(&info);

        kEndFinally();
    }

    return kOK;
}

#if defined(K_WINDOWS)

kFx(kStatus) xkNetwork_InitStaticPlatform()
{
    kStaticObj(kNetwork);

    sobj->notificationHandle = kNULL; 
    sobj->changeSem = kNULL;

    if (kApiLib_NetworkInitializationEnabled())
    {
        WORD wVersionRequested = MAKEWORD(2, 2);  
        WSADATA wsaData;

        kCheck(WSAStartup(wVersionRequested, &wsaData) == 0); 

        if((LOBYTE(wsaData.wVersion) != 2) || (HIBYTE(wsaData.wVersion) != 2)) 
        {
            WSACleanup();
            return kERROR_NETWORK;
        }
    }
 
    return kOK; 
}

kFx(kStatus) xkNetwork_ReleaseStaticPlatform()
{    
    if (kApiLib_NetworkInitializationEnabled())
    {
        WSACleanup(); 
    }

    return kOK; 
}

kFx(kStatus) xkNetwork_DefaultQueryNetInfo(kNetworkInfo info)
{
    ULONG bufferLength = 0;
    k32u opStatus;
    PIP_ADAPTER_ADDRESSES result = NULL;
    const k32u MAX_ITERATIONS = 16;  //arbitrary; 2 should be fine
    k32u iterationCount = 0;

    kTry
    {
        //get initial guess for required buffer size
        GetAdaptersAddresses(AF_INET, GAA_FLAG_INCLUDE_PREFIX, NULL, kNULL, &bufferLength);

        //the required buffer size may change between when we request it and when it's needed (below); iterate 
        do
        {
            kTest(kMemAlloc(bufferLength, &result));

            opStatus = GetAdaptersAddresses(AF_INET, GAA_FLAG_INCLUDE_PREFIX, NULL, result, &bufferLength);

            if (opStatus != ERROR_SUCCESS)
            {
                kMemFreeRef(&result); 

                if (opStatus != ERROR_BUFFER_OVERFLOW)
                {
                    kThrow(kERROR_NETWORK); 
                }
            }
     
            iterationCount++; 
        } 
        while ((opStatus != ERROR_SUCCESS) && (iterationCount < MAX_ITERATIONS));

        kTestTrue(opStatus == ERROR_SUCCESS, kERROR_NETWORK);

        //parse the results
        PIP_ADAPTER_ADDRESSES it = result;

        while (!kIsNull(it))
        {
            PIP_ADAPTER_UNICAST_ADDRESS unicastIt = it->FirstUnicastAddress;
            kNetworkAdapter adapter = kNULL;
            kText128 name;
            kBool isUp = (it->OperStatus == IfOperStatusUp);
            kMacAddress macAddress;

            if (WideCharToMultiByte(CP_UTF8, 0, it->FriendlyName, -1, name, kCountOf(name), kNULL, kNULL) == 0)
            {
                name[0] = '\0';
            }

            kItemCopy(macAddress.address, it->PhysicalAddress, kMin_(kCountOf(macAddress.address), it->PhysicalAddressLength));

            kTest(xkNetworkInfo_AddAdapter(info, name, it->IfIndex, isUp, macAddress, &adapter));
        
            while (!kIsNull(unicastIt))
            {
                struct sockaddr_in* addr = (struct sockaddr_in *) unicastIt->Address.lpSockaddr;
                k32u prefixLength = 32;
                kIpAddress address;
                    
#               if (_WIN32_WINNT >= _WIN32_WINNT_VISTA) 
                {
                    prefixLength = unicastIt->OnLinkPrefixLength;
                }
#               endif

                if (kSuccess(xkIpAddress_FromSockAddr(addr, &address, kNULL)))
                {
                    kTest(xkNetworkAdapter_AddInterface(adapter, address, prefixLength, kNULL));                     
                }
        
                unicastIt = unicastIt->Next;
            }
        
            it = it->Next;
        }      
    }
    kFinally
    {
        kMemFreeRef(&result);

        kEndFinally();
    }

    return kOK;
}

kFx(kBool) xkNetwork_DefaultQueryNetChange(k64u timeout)
{
    kStaticObj(kNetwork);
    
    return kSuccess(kSemaphore_Wait(sobj->changeSem, timeout));
}

#if (_WIN32_WINNT >= _WIN32_WINNT_VISTA) 
              
kFx(void) xkNetwork_OnChangeNotification(kPointer context, PMIB_IPINTERFACE_ROW row, MIB_NOTIFICATION_TYPE notificationType)
{
    kStaticObj(kNetwork);
    
    kSemaphore_Post(sobj->changeSem);
}

kFx(kStatus) xkNetwork_OsBeginChangeNotification()
{
    kStaticObj(kNetwork);

    kCheck(kSemaphore_Construct(&sobj->changeSem, 0, kNULL));

    kCheckTrue(NotifyIpInterfaceChange(AF_INET, xkNetwork_OnChangeNotification, kNULL, kFALSE, &sobj->notificationHandle) == NO_ERROR, kERROR_OS); 

    return kOK;
}

kFx(kStatus) xkNetwork_OsEndChangeNotification()
{
    kStaticObj(kNetwork);

    if (!kIsNull(sobj->notificationHandle))
    {
        CancelMibChangeNotify2(sobj->notificationHandle);
        sobj->notificationHandle = kNULL;
    }

    kDestroyRef(&sobj->changeSem);

    return kOK;
}

#else  

kFx(kStatus) xkNetwork_OsBeginChangeNotification()
{
    return kOK;
}

kFx(kStatus) xkNetwork_OsEndChangeNotification()
{
    return kOK;
}

#endif  //#if (_WIN32_WINNT >= _WIN32_WINNT_VISTA) 

#elif defined (K_LINUX) 

kFx(kStatus) xkNetwork_InitStaticPlatform()
{
    kStaticObj(kNetwork);

    sobj->netlinkSocket = kNULL;

    return kOK;
}

kFx(kStatus) xkNetwork_ReleaseStaticPlatform()
{
    return kOK;
}

kFx(kStatus) xkNetwork_DefaultQueryNetInfo(kNetworkInfo info)
{
    struct ifaddrs *ifaList = kNULL; 
    kNetworkAdapter adapter = kNULL;

    kTry
    {
        kTestTrue(getifaddrs(&ifaList) == 0, kERROR_NETWORK);
               
        struct ifaddrs *ifaListIt = ifaList; 
        
        while (!kIsNull(ifaListIt))
        {
            if ((ifaListIt->ifa_addr != kNULL) && (ifaListIt->ifa_addr->sa_family == AF_INET))
            {
                const kChar* name = ifaListIt->ifa_name;
                struct sockaddr_in* sockAddr = (struct sockaddr_in*) ifaListIt->ifa_addr;
                kIpAddress address = kIpAddress_FromNet32u(sockAddr->sin_addr.s_addr); 
                k32u prefixLength = 0; 

                if (!kIsNull(ifaListIt->ifa_netmask))
                {
                    struct sockaddr_in* sockMask = (struct sockaddr_in*) ifaListIt->ifa_netmask;
                    kIpAddress mask = kIpAddress_FromNet32u(sockMask->sin_addr.s_addr); 
                    prefixLength = xkIpAddress_SubnetMaskToPrefix(kIpAddress_ToHost32u(mask));
                }
                
                if (!kSuccess(kNetworkInfo_FindAdapterByName(info, name, &adapter)))
                {
                    kSize id = (kSize) if_nametoindex(name);
                    kBool isUp = ((ifaListIt->ifa_flags & IFF_UP) != 0);
                    int socketFd = socket(PF_INET, SOCK_DGRAM | SOCK_CLOEXEC, IPPROTO_IP);
                    struct ifreq ifaceReq = { 0 };
                    kMacAddress macAddress = { 0 };

                    kTest(kStrCopy(ifaceReq.ifr_name, IFNAMSIZ, name));

                    if (socketFd != -1)
                    {
                        if (ioctl(socketFd, SIOCGIFHWADDR, &ifaceReq) == 0)
                        {
                            kItemCopy(macAddress.address, ifaceReq.ifr_addr.sa_data, kCountOf(macAddress.address));
                        }

                        close(socketFd);
                    }

                    kTest(xkNetworkInfo_AddAdapter(info, name, id, isUp, macAddress, &adapter));
                }

                kTest(xkNetworkAdapter_AddInterface(adapter, address, prefixLength, kNULL));
            }

            ifaListIt = ifaListIt->ifa_next;
        }
    }
    kFinally
    {
        freeifaddrs(ifaList);
        kEndFinally();
    }

    return kOK;
}

kFx(kStatus) xkNetwork_OsBeginChangeNotification()
{
    kStaticObj(kNetwork);
    int fd = (int) xkSOCKET_INVALID_SOCKET;
    struct sockaddr_nl netlink;

    kCheckTrue((fd = socket(PF_NETLINK, SOCK_RAW, NETLINK_ROUTE)) != (int)xkSOCKET_INVALID_SOCKET, kERROR_NETWORK);

    netlink.nl_family = AF_NETLINK; 
    netlink.nl_groups = RTMGRP_LINK | RTMGRP_IPV4_IFADDR;

    kCheckTrue(bind(fd, (struct sockaddr *)&netlink, sizeof(netlink)) == 0, kERROR_NETWORK);

    kCheck(xkSocket_ConstructFromHandle(&sobj->netlinkSocket, kIP_VERSION_4, xkSOCKET_TYPE_NETLINK, fd, kNULL));

    kCheck(kSocket_SetEvents(sobj->netlinkSocket, kSOCKET_EVENT_READ));

    return kOK;
}

kFx(kStatus) xkNetwork_OsEndChangeNotification()
{
    kStaticObj(kNetwork);

    kDestroyRef(&sobj->netlinkSocket);

    return kOK;
}

kFx(kBool) xkNetwork_DefaultQueryNetChange(k64u timeout)
{
    kStaticObj(kNetwork);
    kSize bytesRead; 

    if (kSuccess(kSocket_Wait(sobj->netlinkSocket, timeout)))
    {        
        if (kSuccess(kSocket_ReadFrom(sobj->netlinkSocket, kNULL, sobj->buffer, sizeof(sobj->buffer), &bytesRead)))
        {
            return kTRUE;
        }
    }

    return kFALSE;
}

#elif defined (K_VX_KERNEL)

kFx(kStatus) xkNetwork_InitStaticPlatform()
{
    kStaticObj(kNetwork);

    sobj->netlinkSocket = kNULL;

    return kOK;
}

kFx(kStatus) xkNetwork_ReleaseStaticPlatform()
{
    return kOK;
}

//note: same as Linux
kFx(kStatus) xkNetwork_DefaultQueryNetInfo(kNetworkInfo info)
{
    struct ifaddrs *ifaList = kNULL; 
    kNetworkAdapter adapter = kNULL;

    kTry
    {
        kTestTrue(getifaddrs(&ifaList) == 0, kERROR_NETWORK);
               
        struct ifaddrs *ifaListIt = ifaList; 
        
        while (!kIsNull(ifaListIt))
        {
            if ((ifaListIt->ifa_addr != kNULL) && (ifaListIt->ifa_addr->sa_family == AF_INET))
            {
                const kChar* name = ifaListIt->ifa_name;
                struct sockaddr_in* sockAddr = (struct sockaddr_in*) ifaListIt->ifa_addr;
                kIpAddress address = kIpAddress_FromNet32u(sockAddr->sin_addr.s_addr); 
                k32u prefixLength = 0; 

                if (!kIsNull(ifaListIt->ifa_netmask))
                {
                    struct sockaddr_in* sockMask = (struct sockaddr_in*) ifaListIt->ifa_netmask;
                    kIpAddress mask = kIpAddress_FromNet32u(sockMask->sin_addr.s_addr); 
                    prefixLength = xkIpAddress_SubnetMaskToPrefix(kIpAddress_ToHost32u(mask));
                }
                
                if (!kSuccess(kNetworkInfo_FindAdapterByName(info, name, &adapter)))
                {
                    kSize id = (kSize) if_nametoindex(name);
                    kBool isUp = ((ifaListIt->ifa_flags & IFF_UP) != 0);
                    int socketFd = socket(PF_INET, SOCK_DGRAM, IPPROTO_IP);
                    struct ifreq ifaceReq = { 0 };
                    kMacAddress macAddress = { 0 };

                    kTest(kStrCopy(ifaceReq.ifr_name, IFNAMSIZ, name));

                    if (socketFd != -1)
                    {
                        if (ioctl(socketFd, SIOCGIFHWADDR, &ifaceReq) == 0)
                        {
                            kItemCopy(macAddress.address, ifaceReq.ifr_addr.sa_data, kCountOf(macAddress.address));
                        }

                        close(socketFd);
                    }

                    kTest(xkNetworkInfo_AddAdapter(info, name, id, isUp, macAddress, &adapter));
                }

                kTest(xkNetworkAdapter_AddInterface(adapter, address, prefixLength, kNULL));
            }

            ifaListIt = ifaListIt->ifa_next;
        }
    }
    kFinally
    {
        freeifaddrs(ifaList);
        kEndFinally();
    }

    return kOK;
}

kFx(kStatus) xkNetwork_OsBeginChangeNotification()
{
    kStaticObj(kNetwork);
    int fd = (int) xkSOCKET_INVALID_SOCKET;
    struct Ip_sockaddr_nl netlink;

    kCheckTrue((fd = socket(PF_NETLINK, SOCK_RAW, IP_NETLINK_ROUTE)) != (int)xkSOCKET_INVALID_SOCKET, kERROR_NETWORK);

    netlink.nl_family = AF_NETLINK; 
    netlink.nl_len = sizeof(netlink);
    netlink.nl_groups = IP_RTMGRP_IPV4_IFADDR | IP_RTMGRP_LINK;

    kCheckTrue(bind(fd, (struct sockaddr *)&netlink, sizeof(netlink)) == 0, kERROR_NETWORK);

    kCheck(xkSocket_ConstructFromHandle(&sobj->netlinkSocket, kIP_VERSION_4, xkSOCKET_TYPE_NETLINK, fd, kNULL));

    kCheck(kSocket_SetEvents(sobj->netlinkSocket, kSOCKET_EVENT_READ));

    return kOK;
}

kFx(kStatus) xkNetwork_OsEndChangeNotification()
{
    kStaticObj(kNetwork);

    kDestroyRef(&sobj->netlinkSocket);

    return kOK;
}

kFx(kBool) xkNetwork_DefaultQueryNetChange(k64u timeout)
{
    kStaticObj(kNetwork);
    kSize bytesRead; 

    if (kSuccess(kSocket_Wait(sobj->netlinkSocket, timeout)))
    {
        if (kSuccess(kSocket_ReadFrom(sobj->netlinkSocket, kNULL, sobj->buffer, sizeof(sobj->buffer), &bytesRead)))
        {
            return kTRUE;
        }
    }

    return kFALSE;
}

#elif defined(K_QNX)

kFx(kStatus) xkNetwork_InitStaticPlatform()
{
    kStaticObj(kNetwork);

    //unused
    sobj->netlinkSocket = kNULL;

    return kOK;
}

kFx(kStatus) xkNetwork_ReleaseStaticPlatform()
{
    return kOK;
}

//note: same as Linux
kFx(kStatus) xkNetwork_DefaultQueryNetInfo(kNetworkInfo info)
{
    struct ifaddrs *ifaList = kNULL; 
    kNetworkAdapter adapter = kNULL;

    kTry
    {
        kTestTrue(getifaddrs(&ifaList) == 0, kERROR_NETWORK);
               
        struct ifaddrs *ifaListIt = ifaList; 
        
        while (!kIsNull(ifaListIt))
        {
            if ((ifaListIt->ifa_addr != kNULL) && (ifaListIt->ifa_addr->sa_family == AF_INET))
            {
                const kChar* name = ifaListIt->ifa_name;
                struct sockaddr_in* sockAddr = (struct sockaddr_in*) ifaListIt->ifa_addr;
                kIpAddress address = kIpAddress_FromNet32u(sockAddr->sin_addr.s_addr); 
                k32u prefixLength = 0; 

                if (!kIsNull(ifaListIt->ifa_netmask))
                {
                    struct sockaddr_in* sockMask = (struct sockaddr_in*) ifaListIt->ifa_netmask;
                    kIpAddress mask = kIpAddress_FromNet32u(sockMask->sin_addr.s_addr); 
                    prefixLength = xkIpAddress_SubnetMaskToPrefix(kIpAddress_ToHost32u(mask));
                }
                
                if (!kSuccess(kNetworkInfo_FindAdapterByName(info, name, &adapter)))
                {
                    kSize id = (kSize) if_nametoindex(name);
                    kBool isUp = ((ifaListIt->ifa_flags & IFF_UP) != 0);
                    int socketFd = socket(PF_INET, SOCK_DGRAM | SOCK_CLOEXEC, IPPROTO_IP);
                    struct ifreq ifaceReq = { 0 };
                    kMacAddress macAddress = { 0 };

                    kTest(kStrCopy(ifaceReq.ifr_name, IFNAMSIZ, name));

                    if (socketFd != -1)
                    {
                        if (ioctl(socketFd, SIOCGIFHWADDR, &ifaceReq) == 0)
                        {
                            kItemCopy(macAddress.address, ifaceReq.ifr_addr.sa_data, kCountOf(macAddress.address));
                        }

                        close(socketFd);
                    }

                    kTest(xkNetworkInfo_AddAdapter(info, name, id, isUp, macAddress, &adapter));
                }

                kTest(xkNetworkAdapter_AddInterface(adapter, address, prefixLength, kNULL));
            }

            ifaListIt = ifaListIt->ifa_next;
        }
    }
    kFinally
    {
        freeifaddrs(ifaList);
        kEndFinally();
    }

    return kOK;
}

kFx(kStatus) xkNetwork_OsBeginChangeNotification()
{
    return kOK;
}

kFx(kStatus) xkNetwork_OsEndChangeNotification()
{
    return kOK;
}

kFx(kBool) xkNetwork_DefaultQueryNetChange(k64u timeout)
{
    //prevent spinning
    kThread_Sleep(timeout);

    return kFALSE;
}

#else

kFx(kStatus) xkNetwork_InitStaticPlatform()
{
    return kOK;
}

kFx(kStatus) xkNetwork_ReleaseStaticPlatform()
{
    return kOK;
}

kFx(kStatus) xkNetwork_DefaultQueryNetInfo(kNetworkInfo netInfo)
{
    return kERROR_UNIMPLEMENTED;
}

kFx(kStatus) xkNetwork_OsBeginChangeNotification()
{
    return kOK;
}

kFx(kStatus) xkNetwork_OsEndChangeNotification()
{
    return kOK;
}

kFx(kBool) xkNetwork_DefaultQueryNetChange(k64u timeout)
{
    //prevent spinning
    kThread_Sleep(timeout);

    return kFALSE;
}

#endif

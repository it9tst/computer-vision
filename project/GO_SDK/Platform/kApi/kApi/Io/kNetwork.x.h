/** 
 * @file    kNetwork.x.h
 *
 * @internal
 * Copyright (C) 2008-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_NETWORK_X_H
#define K_API_NETWORK_X_H

#include <kApi/Data/kArrayList.h>

/*
* kIpVersion enum
*/

kDeclareEnumEx(k, kIpVersion, kValue)

/*
* kIpAddress structure
*/

kDeclareValueEx(k, kIpAddress, kValue)

kFx(kBool) xkIpAddress_VEquals(kType type, const void* value, const void* other); 
kFx(kSize) xkIpAddress_VHashCode(kType type, const void* value); 

//IPv4 only
kFx(k32u) xkIpAddress_SubnetPrefixToMask(k32u prefix);
kFx(k32u) xkIpAddress_SubnetMaskToPrefix(k32u mask);

#if defined(K_PLATFORM)

kFx(kStatus) xkIpAddress_ToSockAddr(kIpAddress address, k32u port, struct sockaddr_in* sockAddr);
kFx(kStatus) xkIpAddress_FromSockAddr(struct sockaddr_in* sockAddr, kIpAddress *address, k32u* port);

#endif

/*
* kIpEndPoint structure
*/

kDeclareValueEx(k, kIpEndPoint, kValue)

kFx(kBool) xkIpEndPoint_VEquals(kType type, const void* value, const void* other); 

/*
* kNetworkInterface class
*/

typedef struct kNetworkInterfaceClass
{
    kObjectClass base; 

    kNetworkAdapter adapter;        //parent

    kIpAddress address;             //IP address
    k32u prefixLength;              //subnet prefix length

} kNetworkInterfaceClass;

kDeclareClassEx(k, kNetworkInterface, kObject)

kFx(kStatus) xkNetworkInterface_Construct(kNetworkInterface* iface, kNetworkAdapter adapter, kIpAddress address, k32u prefixLength, kAlloc allocator);

kFx(kStatus) xkNetworkInterface_Init(kNetworkInterface iface, kType type, kNetworkAdapter adapter, kIpAddress address, k32u prefixLength, kAlloc alloc);

/*
* kNetworkAdapter class
*/

typedef struct kNetworkAdapterClass
{
    kObjectClass base; 

    kNetworkInfo info;              //parent

    kString name;                   //unique adapter name
    kSize id;                       //unique numeric adapter identifier
    kMacAddress macAddress;         //Current MAC address.
    kBool isUp;                     //is the adapter able to send/receive packets?

    kArrayList interfaces;          //list of interfaces -- kArrayList<kNetworkInterface>

} kNetworkAdapterClass;

kDeclareClassEx(k, kNetworkAdapter, kObject)

kFx(kStatus) xkNetworkAdapter_Construct(kNetworkAdapter* adapter, kNetworkInfo info, const kChar* name, kSize id, kBool isUp, kMacAddress macAddress, kAlloc allocator);

kFx(kStatus) xkNetworkAdapter_Init(kNetworkAdapter adapter, kType type, kNetworkInfo info, const kChar* name, kSize id, kBool isUp, kMacAddress macAddress, kAlloc alloc);

kFx(kStatus) xkNetworkAdapter_VRelease(kNetworkAdapter adapter);

kFx(kStatus) xkNetworkAdapter_AddInterface(kNetworkAdapter adapter, kIpAddress address, k32u prefixLength, kNetworkInterface* iface);

/*
* kNetworkInfo class
*/

typedef struct kNetworkInfoClass
{
    kObjectClass base; 

    kArrayList adapters;            //list of adapters -- kArrayList<kNetworkAdapter>
    kArrayList interfaces;          //list of interfaces (flattened list, for convenient access) -- kArrayList<kNetworkInterface>

} kNetworkInfoClass;

kDeclareClassEx(k, kNetworkInfo, kObject)

kFx(kStatus) xkNetworkInfo_Init(kNetworkInfo info, kType type, kAlloc alloc);

kFx(kStatus) xkNetworkInfo_VRelease(kNetworkInfo info);

kFx(kStatus) xkNetworkInfo_AddAdapter(kNetworkInfo info, const kChar* name, kSize id, kBool isUp, kMacAddress macAddress, kNetworkAdapter* adapter);

/*
* kNetwork static class
*/

#define xkNETWORK_QUIT_QUERY_PERIOD         (100000)                //period for change-monitoring logic to check for quit

#if defined(K_PLATFORM)

#if defined(K_WINDOWS)

#   define xkNetworkPlatformFields()            \
        HANDLE notificationHandle;              \
        kSemaphore changeSem;
#else

#   define xkNetworkPlatformFields()            \
        kSocket netlinkSocket;                  \
        kByte buffer[4094];        

#endif

typedef struct kNetworkStatic
{
    kAtomic32s shouldQuit; 
    kLock eventLock;                    //protect access to change event
    kEvent changeEvent;                 //raised when network has changed
    kThread changeThread;               //thread to monitor for changes
    xkNetworkPlatformFields()
} kNetworkStatic; 

kDeclareStaticClassEx(k, kNetwork)

#endif

kFx(kStatus) xkNetwork_InitStatic(); 
kFx(kStatus) xkNetwork_ReleaseStatic(); 

kFx(kStatus) xkNetwork_InitStaticPlatform(); 
kFx(kStatus) xkNetwork_ReleaseStaticPlatform(); 

kFx(kStatus) xkNetwork_OsBeginChangeNotification(); 
kFx(kStatus) xkNetwork_OsEndChangeNotification(); 

kFx(kStatus) xkNetwork_ChangeThreadEntry(kPointer context); 
kFx(kStatus) xkNetwork_NotifyChange();

kFx(kStatus) xkNetwork_DefaultQueryNetInfo(kNetworkInfo netInfo);
kFx(kBool) xkNetwork_DefaultQueryNetChange(k64u timeout);

/* 
* Deprecated (Stage 1): not recommended for further use, but not yet announced via kDeprecate
*/

//[Deprecated]
typedef struct kIpEntry
{
    kIpAddress address;         ///< IP address. 
    kText128 name;              ///< Host adapter name. 
} kIpEntry; 

kDeclareValueEx(k, kIpEntry, kValue)

//[Deprecated] Replace with xkIpAddress_ToSockAddr
#define kIpAddress_ToSockAddr xkIpAddress_ToSockAddr

//[Deprecated] Replace with xkIpAddress_FromSockAddr
#define kIpAddress_FromSockAddr xkIpAddress_FromSockAddr

//[Deprecated]
kFx(kStatus) kNetwork_LocalIpInterfacesEx(kArrayList interfaces, kBool includeLoopback);

//[Deprecated] 
kInlineFx(kStatus) kNetwork_LocalIpInterfaces(kArrayList entries)
{
    return kNetwork_LocalIpInterfacesEx(entries, kFALSE);
}

#endif

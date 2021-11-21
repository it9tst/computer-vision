/** 
 * @file    kNetwork.h
 * @brief   IP networking definitions. 
 *
 * @internal
 * Copyright (C) 2008-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_NETWORK_H
#define K_API_NETWORK_H

#include <kApi/kApiDef.h>
#include <kApi/Data/kString.h>

/**
 * @struct  kIpVersion
 * @extends kValue
 * @ingroup kApi-Io 
 * @brief   Represents an Internet Protocol version. 
 * 
 * The following enumerators are defined:  
 * - #kIP_VERSION_4
 */
typedef k32s kIpVersion; 

/** @relates kIpVersion @{ */
#define kIP_VERSION_4          (4)         ///< Internet Protocol version 4.
/** @} */

/**
 * @struct  kIpAddress
 * @extends kValue
 * @ingroup kApi-Io 
 * @brief   Represents an IP address. 
 */
typedef struct kIpAddress
{
    kIpVersion version;                 ///< Address version.
    kByte address[16];                  ///< Address bytes (most significant byte first). 
} kIpAddress;                                 

/** 
 * Gets an address representing an automatically-assigned address.
 *
 * @public              @memberof kIpAddress
 * @param   version     IP version.
 * @return              Address value. 
 */
kFx(kIpAddress) kIpAddress_Any(kIpVersion version); 

/** 
 * Gets an address representing an automatically-assigned IPv4 address.
 *
 * @public          @memberof kIpAddress
 * @return          Address value. 
 */
kFx(kIpAddress) kIpAddress_AnyV4(); 

/** 
 * Gets an address suitable for broadcasting IPv4 datagrams. 
 * 
 * @public          @memberof kIpAddress
 * @return          Address value. 
 */
kFx(kIpAddress) kIpAddress_BroadcastV4();

/** 
 * Gets the loopback address. 
 *
 * @public              @memberof kIpAddress
 * @param   version     IP version.
 * @return              Address value. 
 */
kFx(kIpAddress) kIpAddress_Loopback(kIpVersion version);

/** 
 * Gets the IpV4 loopback address. 
 *
 * @public          @memberof kIpAddress
 * @return          Address value. 
 */
kFx(kIpAddress) kIpAddress_LoopbackV4();

/** 
 * Parses a text-formatted IP address. 
 *
 * Supports dotted-quad (IPv4) format (e.g. "192.168.1.10"). 
 *
 * @public              @memberof kIpAddress
 * @param   address     Receives the IP address. 
 * @param   text        Text-formatted IP address.
 * @return              Operation status. 
 */
kFx(kStatus) kIpAddress_Parse(kIpAddress* address, const kChar* text); 

/** 
 * Formats an IP address as a string. 
 *
 * @public              @memberof kIpAddress
 * @param   address     IP address.
 * @param   text        Receives formatted string.
 * @param   capacity    Capacity of the string buffer. 
 * @return              Operation status. 
 */
kFx(kStatus) kIpAddress_Format(kIpAddress address, kChar* text, kSize capacity); 

/** 
 * Compares two addresses for equality.
 *
 * @public          @memberof kIpAddress
 * @param   a       First address.    
 * @param   b       Second address. 
 * @return          kTRUE if addresses are equal; kFALSE otherwise. 
 */
kFx(kBool) kIpAddress_Equals(kIpAddress a, kIpAddress b); 

/** 
 * Reports whether the given address is a loopback address. 
 *
 * @public              @memberof kIpAddress
 * @param   address     IP address. 
 * @return              kTRUE if the address is loopback; kFALSE otherwise. 
 */
kFx(kBool) kIpAddress_IsLoopback(kIpAddress address); 

/** 
 * Reports whether the given address is a link-local address. 
 *
 * @public              @memberof kIpAddress
 * @param   address     IP address. 
 * @return              kTRUE if the address is link-local; kFALSE otherwise. 
 */
kFx(kBool) kIpAddress_IsLinkLocal(kIpAddress address); 

/** 
 * Converts an IPv4 address to a host-endian 32-bit integer. 
 *
 * @public              @memberof kIpAddress
 * @param   address     IP address.
 * @return              Host-endian integer. 
 */
kFx(k32u) kIpAddress_ToHost32u(kIpAddress address); 

/** 
 * Converts an IPv4 address to a network-endian 32-bit integer. 
 *
 * @public              @memberof kIpAddress
 * @param   address     IP address.
 * @return              Network-endian integer. 
 */
kFx(k32u) kIpAddress_ToNet32u(kIpAddress address); 

/** 
 * Converts a host-endian 32-bit integer to an IPv4 address. 
 *
 * @public              @memberof kIpAddress
 * @param   address     Host-endian integer. 
 * @return              IP address.
 */
kFx(kIpAddress) kIpAddress_FromHost32u(k32u address); 

/** 
 * Converts a network-endian 32-bit integer to an IPv4 address. 
 *
 * @public              @memberof kIpAddress
 * @param   address     Network-endian integer. 
 * @return              IP address.
 */
kFx(kIpAddress) kIpAddress_FromNet32u(k32u address); 

/** 
 * Extracts the network portion of an address. 
 *
 * @public                  @memberof kIpAddress
 * @param   address         IP address.
 * @param   prefixLength    Subnet prefix length.
 * @return                  Network portion of the address.
 */
kFx(kIpAddress) kIpAddress_Network(kIpAddress address, k32u prefixLength); 

/** 
 * Compares two addresses for network equality.
 *
 * @public                  @memberof kIpAddress
 * @param   a               First address.    
 * @param   b               Second address. 
 * @param   prefixLength    Second address. 
 * @return                  kTRUE if network portions of addresses are equal. 
 */
kFx(kBool) kIpAddress_NetworkEquals(kIpAddress a, kIpAddress b, k32u prefixLength); 

/**
 * @struct  kIpEndPoint
 * @extends kValue
 * @ingroup kApi-Io 
 * @brief   Represents an IP end point (address, port). 
 */
typedef struct kIpEndPoint              
{
    kIpAddress address;         ///< IP address.
    k32u port;                  ///< Port number. 
} kIpEndPoint;                  ///< Represents an IP end point.

/** @relates kIpEndPoint @{ */
#define kIP_PORT_ANY    (0)     ///< Used to request an automatically assigned port. 
/** @} */


#include <kApi/Io/kNetwork.x.h>

/**
 * @class   kNetworkInterface
 * @extends kObject
 * @ingroup kApi-Io
 * @brief   Represents network interface configuration information. 
 */
//typedef kObject kNetworkInterface;        --forward-declared in kApiDef.x.h 

/** 
 * Gets the adapter associated with this interface.
 * 
 * @public          @memberof kNetworkInterface
 * @param   iface   Network interface. 
 * @return          Network adapter.
 */
kInlineFx(kNetworkAdapter) kNetworkInterface_Adapter(kNetworkInterface iface)
{
    kObj(kNetworkInterface, iface);
    return obj->adapter; 
}

/** 
 * Gets the IP address associated with this interface.
 * 
 * @public          @memberof kNetworkInterface
 * @param   iface   Network interface. 
 * @return          IP address.
 */
kInlineFx(kIpAddress) kNetworkInterface_Address(kNetworkInterface iface)
{
    kObj(kNetworkInterface, iface);
    return obj->address; 
}

/** 
 * Gets the subnet prefix length associated with this interface.
 * 
 * @public          @memberof kNetworkInterface
 * @param   iface   Network interface. 
 * @return          Subnet prefix length.
 */
kInlineFx(k32u) kNetworkInterface_PrefixLength(kNetworkInterface iface)
{
    kObj(kNetworkInterface, iface);
    return obj->prefixLength; 
}

/**
 * @class   kNetworkAdapter
 * @extends kObject
 * @ingroup kApi-Io
 * @brief   Represents network adapter configuration information. 
 */
//typedef kObject kNetworkAdapter;        --forward-declared in kApiDef.x.h 

/** 
 * Gets the network info object that owns this adapter object.
 * 
 * @public              @memberof kNetworkAdapter
 * @param   adapter     Network adapter. 
 * @return              Network info object.
 */
kInlineFx(kNetworkInfo) kNetworkAdapter_Info(kNetworkAdapter adapter)
{
    kObj(kNetworkAdapter, adapter);
    return obj->info; 
}

/** 
 * Gets the name associated with this adapter object.
 * 
 * @public              @memberof kNetworkAdapter
 * @param   adapter     Network adapter. 
 * @return              Adapter name.
 */
kInlineFx(const kChar*) kNetworkAdapter_Name(kNetworkAdapter adapter)
{
    kObj(kNetworkAdapter, adapter);
    return kString_Chars(obj->name); 
}

/** 
 * Gets the unique numeric ID associated with this adapter object.
 * 
 * In some underlying network stacks and literature, this identifier is referred to 
 * as the interface index (or ifindex). 
 * 
 * @public              @memberof kNetworkAdapter
 * @param   adapter     Network adapter. 
 * @return              Adapter ID.
 * @see                 kSocket_ReadFromEx, kUdpClient_ReceiveEx
 */
kInlineFx(kSize) kNetworkAdapter_Id(kNetworkAdapter adapter)
{
    kObj(kNetworkAdapter, adapter);
    return obj->id; 
}

/** 
 * Gets the current status of the interface.
 * 
 * Underlying operating systems may differ in how interface status is handled. Accordingly, this 
 * property should be used only as a debugging aid. 
 * 
 * @public              @memberof kNetworkAdapter
 * @param   adapter     Network adapter. 
 * @return              kTRUE if the adapter is currently up.
 */
kInlineFx(kBool) kNetworkAdapter_IsUp(kNetworkAdapter adapter)
{
    kObj(kNetworkAdapter, adapter);
    return obj->isUp; 
}

/** 
 * Gets the current MAC address of the interface.
 * 
 * Please note that the MAC address is not immutable and can change.
 * 
 * @public              @memberof kNetworkAdapter
 * @param   adapter     Network adapter. 
 * @return              Current MAC address of the adapter.
 */
kInlineFx(kMacAddress) kNetworkAdapter_MacAddress(kNetworkAdapter adapter)
{
    kObj(kNetworkAdapter, adapter);
    return obj->macAddress; 
}

/** 
 * Reports the number of interfaces associated with this adapter.
 * 
 * @public              @memberof kNetworkAdapter
 * @param   adapter     Network adapter. 
 * @return              Interface count.
 */
kInlineFx(kSize) kNetworkAdapter_InterfaceCount(kNetworkAdapter adapter)
{
    kObj(kNetworkAdapter, adapter);
    return kArrayList_Count(obj->interfaces);
}

/** 
 * Gets the interface at the specified index.
 * 
 * @public              @memberof kNetworkAdapter
 * @param   adapter     Network adapter. 
 * @param   index       Network interface index (relative to kNetworkAdapter_InterfaceCount).
 * @return              Interface at the specified index.
 */
kInlineFx(kNetworkInterface) kNetworkAdapter_InterfaceAt(kNetworkAdapter adapter, kSize index)
{
    kObj(kNetworkAdapter, adapter);
    return kArrayList_AsT(obj->interfaces, index, kNetworkInterface);
}

/** 
 * Gets the interface at the specified index, if present.
 * 
 * @public              @memberof kNetworkAdapter
 * @param   adapter     Network adapter. 
 * @param   index       Network interface index (relative to kNetworkAdapter_InterfaceCount).
 * @param   iface       Receives the interface item at the specified index.
 * @return              Interface at the specified index.
 */
kInlineFx(kStatus) kNetworkAdapter_InterfaceItem(kNetworkAdapter adapter, kSize index, kNetworkInterface* iface)
{
    kObj(kNetworkAdapter, adapter);
    return kArrayList_ItemT(obj->interfaces, index, iface);
}

/**
 * @class   kNetworkInfo
 * @extends kObject
 * @ingroup kApi-Io
 * @brief   Represents network configuration information. 
 */
//typedef kObject kNetworkInfo;        --forward-declared in kApiDef.x.h 

/** 
 * Constructs a network info object describing the current state of any local network adapters. 
 * 
 * @public                  @memberof kNetworkInfo
 * @param   info            Receives Network info object. 
 * @param   alloc           Memory allocator (optional).  
 * @return                  Operation status. 
 */
kFx(kStatus) kNetworkInfo_Construct(kNetworkInfo* info, kAlloc alloc); 

/** 
 * Updates network information.
 * 
 * @public              @memberof kNetworkInfo
 * @param   info        Network info object. 
 * @return              Operation status. 
 */
kFx(kStatus) kNetworkInfo_Refresh(kNetworkInfo info); 

/** 
 * Reports the number of network adapters.
 * 
 * @public              @memberof kNetworkInfo
 * @param   info        Network info object. 
 * @return              Number of adapters.
 */
kInlineFx(kSize) kNetworkInfo_AdapterCount(kNetworkInfo info)
{
    kObj(kNetworkInfo, info);
    return kArrayList_Count(obj->adapters);
}

/** 
 * Gets the adapter at the specified index.
 * 
 * @public              @memberof kNetworkInfo
 * @param   info        Network info object. 
 * @param   index       Adapter index (relative to kNetworkInfo_AdapterCount).
 * @return              Adapter at the specified index.
 */
kInlineFx(kNetworkAdapter) kNetworkInfo_AdapterAt(kNetworkInfo info, kSize index)
{
    kObj(kNetworkInfo, info);
    return kArrayList_AsT(obj->adapters, index, kNetworkAdapter);
}

/** 
 * Reports the total number of network interfaces across all adapters.
 * 
 * @public              @memberof kNetworkInfo
 * @param   info        Network info object. 
 * @return              Total number of interfaces.
 */
kInlineFx(kSize) kNetworkInfo_InterfaceCount(kNetworkInfo info)
{
    kObj(kNetworkInfo, info);
    return kArrayList_Count(obj->interfaces);
}

/** 
 * Gets the interface at the specified index.
 * 
 * @public              @memberof kNetworkInfo
 * @param   info        Network info object. 
 * @param   index       Network interface index (relative to kNetworkInfo_InterfaceCount).
 * @return              Interface at the specified index.
 */
kInlineFx(kNetworkInterface) kNetworkInfo_InterfaceAt(kNetworkInfo info, kSize index)
{
    kObj(kNetworkInfo, info);
    return kArrayList_AsT(obj->interfaces, index, kNetworkInterface);
}

/** 
 * Finds an adapter by name.
 * 
 * @public              @memberof kNetworkInfo
 * @param   info        Network info object. 
 * @param   name        Network adapter name.
 * @param   adapter     Receives network adapter, if found (optional; can be kNULL).
 * @return              Operation status. 
 */
kFx(kStatus) kNetworkInfo_FindAdapterByName(kNetworkInfo info, const kChar* name, kNetworkAdapter* adapter); 

/** 
 * Finds an adapter by its unique numeric identifier.
 * 
 * @public              @memberof kNetworkInfo
 * @param   info        Network info object. 
 * @param   id          Network adapter numeric identifier.
 * @param   adapter     Receives network adapter, if found (optional; can be kNULL).
 * @return              Operation status. 
 */
kFx(kStatus) kNetworkInfo_FindAdapterById(kNetworkInfo info, kSize id, kNetworkAdapter* adapter); 

/** 
 * Finds an interface by its exact local address.
 * 
 * @public              @memberof kNetworkInfo
 * @param   info        Network info object. 
 * @param   address     Network interface address.
 * @param   iface       Receives network interface, if found (optional; can be kNULL).
 * @return              Operation status. 
 */
kFx(kStatus) kNetworkInfo_FindInterfaceByAddress(kNetworkInfo info, kIpAddress address, kNetworkInterface* iface); 

/** 
 * Finds the first interface with subnet configuration that is compatible with the specified address.
 * 
 * @public                  @memberof kNetworkInfo
 * @param   info            Network info object. 
 * @param   address         Address (host portion ignored, if present).
 * @param   iface           Receives network interface, if found (optional; can be kNULL).
 * @return                  Operation status. 
 */
kFx(kStatus) kNetworkInfo_FindInterfaceByNetwork(kNetworkInfo info, kIpAddress address, kNetworkInterface* iface); 

/**
 * @class   kNetwork
 * @extends kObject
 * @ingroup kApi-Io
 * @brief   A collection of static network utility methods. 
 */
//typedef kObject kNetwork;        --forward-declared in kApiDef.x.h 

/** 
 * Add a network change notification handler.
 * 
 * The registered callback will be invoked when network changes occur. Note, the callback 
 * may be invoked multiple times per visible change (underlying changes are not always surfaced
 * in the high-level properties in this API).
 * 
 * When the callback is invoked, the 'receiver' argument passed to this function will be 
 * provided as the 'receiver' argument to the callback. The 'sender' and 'args' callback 
 * parameters are not used at this time.
 *
 * Use the kNetwork_RemoveChangeHandler method to deregister the callback when notifications
 * are no longer needed.
 * 
 * @public              @memberof kNetwork
 * @param   function    Callback function. 
 * @param   receiver    Callback receiver.  
 * @return              Operation status. 
 * @see                 kNetwork_RemoveChangeHandler
 */
kFx(kStatus) kNetwork_AddChangeHandler(kCallbackFx function, kPointer receiver); 

/** 
 * Removes a network change notification handler.
 * 
 * @public              @memberof kNetwork
 * @param   function    Callback function. 
 * @param   receiver    Callback receiver.  
 * @return              Operation status. 
 * @see                 kNetwork_AddChangeHandler
 */
kFx(kStatus) kNetwork_RemoveChangeHandler(kCallbackFx function, kPointer receiver); 

/** 
 * Finds the adapter name associated with a unique numeric adapter identifier.
 * 
 * This method provides a convenient shortcut to constructing a kNetworkInfo object and 
 * then using that object to look up the desired information.
 * 
 * @public                          @memberof kNetwork
 * @param   adapterId               Adapter unique numeric id.   
 * @param   adapterName             Receives adapter name (optional; can be null).
 * @param   adapterNameCapacity     Capacity of adapter name buffer.
 * @return                          Operation status. 
 */
kFx(kStatus) kNetwork_FindAdapterNameById(kSize adapterId, kChar* adapterName, kSize adapterNameCapacity);

/** 
 * Finds the unique numeric adapter identifier associated with an adapter name.
 * 
 * This method provides a convenient shortcut to constructing a kNetworkInfo object and 
 * then using that object to look up the desired information.
 * 
 * @public                  @memberof kNetwork
 * @param   adapterName     Adapter name.
 * @param   adapterId       Receives unique id (optional; can be null).
 * @return                  Operation status. 
 */
kFx(kStatus) kNetwork_FindAdapterIdByName(const kChar* adapterName, kSize* adapterId);

/** 
 * Finds the adapter name associated with the specified interface address.
 * 
 * This method provides a convenient shortcut to constructing a kNetworkInfo object and 
 * then using that object to look up the desired information.
 * 
 * @public                          @memberof kNetwork
 * @param   interfaceAddress        Interface address.   
 * @param   adapterName             Receives adapter name (optional; can be null).
 * @param   adapterNameCapacity     Capacity of adapter name buffer.
 * @return                          Operation status. 
 */
kFx(kStatus) kNetwork_FindAdapterNameByInterface(kIpAddress interfaceAddress, kChar* adapterName, kSize adapterNameCapacity); 

/** 
 * Finds the first interface address associated with an adapter name.
 * 
 * This method provides a convenient shortcut to constructing a kNetworkInfo object and 
 * then using that object to look up the desired information.
 * 
 * @public                      @memberof kNetwork
 * @param   adapterName         Adapter name.
 * @param   interfaceAddress    Receives interface address (optional; can be null).
 * @return                      Operation status. 
 */
kFx(kStatus) kNetwork_FindFirstAdapterInterface(const kChar* adapterName, kIpAddress* interfaceAddress); 

/** 
 * Finds the first interface address with subnet configuration that is compatible with the specified address.
 * 
 * This method provides a convenient shortcut to constructing a kNetworkInfo object and 
 * then using that object to look up the desired information.
 * 
 * @public                      @memberof kNetwork
 * @param   network             Network address (host portion ignored, if present).
 * @param   interfaceAddress    Receives network interface address, if found (optional; can be kNULL).
 * @return                      Operation status. 
 */
kFx(kStatus) kNetwork_FindInterfaceByNetwork(kIpAddress network, kIpAddress* interfaceAddress); 

#endif

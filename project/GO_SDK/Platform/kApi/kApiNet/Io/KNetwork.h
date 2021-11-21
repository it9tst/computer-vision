// 
// KNetwork.h
// 
// Copyright (C) 2014-2021 by LMI Technologies Inc.
// Licensed under the MIT License.
// Redistributed files must retain the above copyright notice.
// 
#ifndef K_API_NET_NETWORK_H
#define K_API_NET_NETWORK_H

#include <kApi/Io/kNetwork.h>
#include "kApiNet/KAlloc.h"
#include "kApiNet/Data/KArrayList.h"
#include "kApiNet/Data/KString.h"

namespace Lmi3d
{
    namespace Zen 
    {
        namespace Io
        {
            /// <summary>Represents an Internet Protocol version.</summary>
            public value struct KIpVersion
            {
                KDeclareEnum(KIpVersion, kIpVersion)

                /// <summary>Internet Protocol version 4.</summary>
                literal k32s IPv4 = kIP_VERSION_4;                
            };

            /// <summary>Represents an IP address.</summary>
            [StructLayout(LayoutKind::Explicit, Size = sizeof(kIpAddress))]
            public value struct KIpAddress
            {
                KDeclareStruct(KIpAddress, kIpAddress)
            public:
                                     
                /// <summary>Gets an address representing an automatically-assigned address.</summary>
                /// 
                /// <param name="version">IP version.</param>
                /// <returns>Address value.</returns>
                static KIpAddress GetAny(KIpVersion version)
                {
                    kIpAddress addr = kIpAddress_Any(version);

                    return KIpAddress(&addr);
                }

                /// <summary>Gets an address representing an automatically-assigned IPv4 address.</summary>
                static property KIpAddress AnyV4
                {
                    KIpAddress get()
                    {
                        kIpAddress addr = kIpAddress_AnyV4();

                        return KIpAddress(&addr);
                    }
                }

                /// <summary>Gets an address suitable for broadcasting IPv4 datagrams.</summary>
                static property KIpAddress BroadcastV4
                {
                    KIpAddress get()
                    {
                        kIpAddress addr = kIpAddress_BroadcastV4();

                        return KIpAddress(&addr);
                    }
                }

                /// <summary>Gets a loopback address.</summary>
                /// 
                /// <param name="version">IP version.</param>
                /// <returns>Address value.</returns>
                static KIpAddress GetLoopback(KIpVersion version)
                {
                    kIpAddress addr = kIpAddress_Loopback(version);

                    return KIpAddress(&addr);
                }

                /// <summary>Gets the IpV4 loopback address.</summary>
                static property KIpAddress LoopbackV4
                {
                    KIpAddress get()
                    {
                        kIpAddress addr = kIpAddress_LoopbackV4();

                        return KIpAddress(&addr);
                    }
                }

                /// <summary>Parses a text-formatted IP address.</summary>
                /// 
                /// <remarks>Supports dotted-quad (IPv4) format (e.g. "192.168.1.10").</remarks>
                /// 
                /// <param name="text">Text-formatted IP address.</param>
                /// <returns>Parsed address.</returns>
                static KIpAddress Parse(String^ text)
                {
                    KString str(text); 
                    kIpAddress addr; 

                    KCheck(kIpAddress_Parse(&addr, str.CharPtr));

                    return KIpAddress(&addr);
                }

                /// <summary>Formats an IP address as a string.</summary>
                /// 
                /// <returns>Formatted address.</returns>
                virtual String^ ToString() override
                {
                    kText64 text;

                    KCheck(kIpAddress_Format(this->ToNative(), text, kCountOf(text)));

                    return KToString(text);
                }

                /// <summary>Reports whether the address is a loopback address.</summary>
                property bool IsLoopback
                {
                    bool get()
                    {
                        return KToBool(kIpAddress_IsLoopback(this->ToNative()));
                    }
                }

                /// <summary>Converts an IPv4 address to a host-endian 32-bit integer.</summary>
                /// <returns>Host-endian integer.</returns>
                k32u ToHost32u()
                {
                    return kIpAddress_ToHost32u(this->ToNative()); 
                }

                /// <summary>Converts an IPv4 address to a network-endian 32-bit integer.</summary>
                /// <returns>Host-endian integer.</returns>
                k32u ToNet32u()
                {
                    return kIpAddress_ToNet32u(this->ToNative());
                }

                /// <summary>Converts a host-endian 32-bit integer to an IPv4 address.</summary>
                /// 
                /// <param name="address">Host-endian integer.</param>
                /// <returns>IP address.</returns>
                static KIpAddress FromHost32u(k32u address)
                {
                    return (KIpAddress)kIpAddress_FromHost32u(address); 
                }

                /// <summary>Converts a network-endian 32-bit integer to an IPv4 address.</summary>
                /// 
                /// <param name="address">Network-endian integer.</param>
                /// <returns>IP address.</returns>
                static KIpAddress FromNet32u(k32u address)
                {
                    return (KIpAddress)kIpAddress_FromNet32u(address);
                }

            private:
       
                /// <summary>Address version.</summary>
                [FieldOffset(offsetof(kIpAddress, version))]
                KIpVersion m_version;

                /// <summary>Address bytes (most significant byte first). </summary>
                [FieldOffset(offsetof(kIpAddress, address))]
                kByte m_bytes;    
            };

            /// <summary>Represents an IP end point (address, port).</summary>
            [StructLayout(LayoutKind::Explicit, Size = sizeof(kIpEndPoint))]
            public value struct KIpEndPoint
            {
                KDeclareStruct(KIpEndPoint, kIpEndPoint)

                /// <summary>Used to request an automatically assigned port.</summary>
                static const k32s AnyPort = kIP_PORT_ANY; 

                /// <summary>Initializes a new KIpEndPoint instance.</summary>
                KIpEndPoint(KIpAddress address, k32s port)
                    : Address(address), Port(port)
                {}

                /// <summary>IP address.</summary>
                [FieldOffset(offsetof(kIpEndPoint, address))]
                KIpAddress Address;      

                /// <summary>Port number.</summary>
                [FieldOffset(offsetof(kIpEndPoint, port))]
                k32s Port;
            };

            /// <summary>Represents information about a local IP address.</summary>
            [StructLayout(LayoutKind::Explicit, Size = sizeof(kIpEntry))]
            public value struct KIpEntry
            {
                KDeclareStruct(KIpEntry, kIpEntry)

                /// <summary>IP address.</summary>
                [FieldOffset(offsetof(kIpEntry, address))]
                KIpAddress Address;      

                /// <summary>Port number.</summary>
                [FieldOffset(offsetof(kIpEntry, name))]
                KText128 Name; 
            };


            /// <summary>Collection of network utility methods.</summary>
            ///
            /// <remarks>
            /// <para>Default KRefStyle: None</para>
            /// </remarks>
            public ref class KNetwork : public KObject
            {
                KDeclareNoneClass(KNetwork, kNetwork)
            public:

                /// <summary>Finds information about local IP configuration.</summary>
                /// 
                /// <remarks>
                /// <para>This method determines information pertaining to IP addresses on the local host.
                /// Some informational fields may be unavailable on some platforms.</para>
                /// 
                /// <para>Use KObject.Dispose to destroy the list returned by this method.</para>
                /// </remarks>
                /// 
                /// <returns>List of KIpEntry structures.</returns>
                static KArrayList^ FindLocalIpInterfaces()
                {
                    KArrayList^ list = gcnew KArrayList(); 

                    try
                    {
                        KCheck(kNetwork_LocalIpInterfaces(KToHandle(list))); 

                        return list; 
                    }
                    catch (...)
                    {
                        throw;
                    }
                }

            private:
                KNetwork() : KObject(DefaultRefStyle) {}
            };
        }
    }
}

#endif

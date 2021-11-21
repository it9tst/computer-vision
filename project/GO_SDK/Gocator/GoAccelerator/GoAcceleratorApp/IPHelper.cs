using System;
using System.Runtime.InteropServices;
using System.Collections.Generic;
using System.Linq;
using System.Net;

namespace GoAccelerator
{
    /// <summary>
    /// IPHelper from PInvoke.NET
    /// GetExtendedTcpTable https://www.pinvoke.net/default.aspx/iphlpapi/GetExtendedTcpTable.html
    /// MIB_TCPROW_OWNER_PID http://www.pinvoke.net/default.aspx/Structures/MIB_TCPROW_OWNER_PID.html
    /// MIB_TCPTABLE_OWNER_PID http://www.pinvoke.net/default.aspx/Structures/MIB_TCPTABLE_OWNER_PID.html
    /// MIB_TCP6ROW_OWNER_PID http://www.pinvoke.net/default.aspx/Structures/MIB_TCP6ROW_OWNER_PID.html
    /// MIB_TCP6ROW_OWNER_PID http://www.pinvoke.net/default.aspx/Structures/MIB_TCP6TABLE_OWNER_PID.html
    /// TCP_TABLE_CLASS http://www.pinvoke.net/default.aspx/Enums/TCP_TABLE_CLASS.html
    /// </summary>
    class IPHelper
    {
        public const int AF_INET = 2;    // IP_v4 = System.Net.Sockets.AddressFamily.InterNetwork
        public const int AF_INET6 = 23;  // IP_v6 = System.Net.Sockets.AddressFamily.InterNetworkV6
        private const int ERROR_INSUFFICIENT_BUFFER = 0x7a;
        private const int NO_ERROR = 0x0;

        // Learn about IPHelper here: http://msdn2.microsoft.com/en-us/library/aa366073.aspx and http://msdn2.microsoft.com/en-us/library/aa365928.aspx
        // Note: C++'s ulong is ALWAYS 32bits, unlike C#'s ulong. See http://medo64.blogspot.com/2009/05/why-ulong-is-32-bit-even-on-64-bit.html

        //-----------------------------------------------------------------------------------------------------------
        // For future, we may want to consider migrating to pinvoke.net's managed wrapper of GetExtendedTcpTable,
        // see https://www.pinvoke.net/default.aspx/iphlpapi/GetExtendedTcpTable.html
        // For now -- we stay with the pointer arithmetic method.
        //-----------------------------------------------------------------------------------------------------------
        //        [DllImport("iphlpapi.dll", ExactSpelling = true, SetLastError = true)]
        //        private static extern uint GetExtendedTcpTable(IntPtr pTcpTable, ref UInt32 dwTcpTableLength, [MarshalAs(UnmanagedType.Bool)] bool sort, UInt32 ipVersion, TcpTableType tcpTableType, UInt32 reserved);

        /// <summary>
        /// The TCP_TABLE_CLASS enumeration defines the set of values used to indicate the type of table returned by calls to GetExtendedTcpTable.
        /// </summary>
        public enum TCP_TABLE_CLASS
        {
            TCP_TABLE_BASIC_LISTENER,
            TCP_TABLE_BASIC_CONNECTIONS,
            TCP_TABLE_BASIC_ALL,
            TCP_TABLE_OWNER_PID_LISTENER,
            TCP_TABLE_OWNER_PID_CONNECTIONS,
            TCP_TABLE_OWNER_PID_ALL,
            TCP_TABLE_OWNER_MODULE_LISTENER,
            TCP_TABLE_OWNER_MODULE_CONNECTIONS,
            TCP_TABLE_OWNER_MODULE_ALL
        }

        /// <summary>
        /// The state of the TCP connection. This member can be one of the values from the MIB_TCP_STATE enumeration defined in the Tcpmib.h header file.
        /// See https://docs.microsoft.com/en-us/windows/desktop/api/tcpmib/ns-tcpmib-mib_tcprow_owner_pid
        /// </summary>
        public enum MIB_TCP_STATE
        {
            MIB_TCP_STATE_CLOSED = 1,
            MIB_TCP_STATE_LISTEN = 2,
            MIB_TCP_STATE_SYN_SENT = 3,
            MIB_TCP_STATE_SYN_RCVD = 4,
            MIB_TCP_STATE_ESTAB = 5,
            MIB_TCP_STATE_FIN_WAIT1 = 6,
            MIB_TCP_STATE_FIN_WAIT2 = 7,
            MIB_TCP_STATE_CLOSE_WAIT = 8,
            MIB_TCP_STATE_CLOSING = 9,
            MIB_TCP_STATE_LAST_ACK = 10,
            MIB_TCP_STATE_TIME_WAIT = 11,
            MIB_TCP_STATE_DELETE_TCB = 12
        }

        /// <summary>
        /// The GetExtendedTcpTable function retrieves a table that contains a list of TCP endpoints available to the application.
        /// </summary>
        [DllImport("iphlpapi.dll", SetLastError = true)]
        static extern uint GetExtendedTcpTable(IntPtr pTcpTable, ref int dwOutBufLen, bool sort, int ipVersion, TCP_TABLE_CLASS tblClass, uint reserved = 0);

        /// <summary>
        /// The MIB_TCPROW_OWNER_PID structure contains information that describes an IPv4 TCP connection with IPv4 addresses, ports used by the TCP connection, 
        /// and the specific process ID (PID) associated with connection.
        /// </summary>
        [StructLayout(LayoutKind.Sequential)]
        public struct MIB_TCPROW_OWNER_PID
        {
            private uint state;
            private uint localAddr;
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 4)]
            private byte[] localPort;
            private uint remoteAddr;
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 4)]
            private byte[] remotePort;
            private uint owningPid;

            public uint ProcessId
            {
                get { return owningPid; }
            }

            public IPAddress LocalAddress
            {
                get { return new IPAddress(localAddr); }
            }

            public ushort LocalPort
            {
                get
                {
                    return BitConverter.ToUInt16(new byte[2] { localPort[1], localPort[0] }, 0);
                }
            }

            public IPAddress RemoteAddress
            {
                get { return new IPAddress(remoteAddr); }
            }

            public ushort RemotePort
            {
                get
                {
                    return BitConverter.ToUInt16(new byte[2] { remotePort[1], remotePort[0] }, 0);
                }
            }

            public MIB_TCP_STATE State
            {
                get { return (MIB_TCP_STATE)state; }
            }
        }

        /// <summary>
        /// The MIB_TCPTABLE_OWNER_PID structure contains a table of process IDs (PIDs) and the IPv4 TCP links that are context bound to these PIDs.
        /// </summary>
        [StructLayout(LayoutKind.Sequential)]
        public struct MIB_TCPTABLE_OWNER_PID
        {
            public uint dwNumEntries;
            [MarshalAs(UnmanagedType.ByValArray, ArraySubType = UnmanagedType.Struct, SizeConst = 1)]
            public MIB_TCPROW_OWNER_PID[] table;
        }

        /// <summary>
        /// The MIB_TCP6ROW_OWNER_PID structure contains information that describes an IPv6 TCP connection associated with a specific process ID (PID).
        /// </summary>
        /// <remarks>
        /// This table entry is specifically returned by a call to GetExtendedTcpTable with the TableClass parameter set to a TCP_TABLE_OWNER_PID_* 
        /// value from the TCP_TABLE_CLASS enumeration and the ulAf parameter set to AF_INET4.
        /// 
        /// Ports are broken up by bytes. (This is the same as the standard library does it). High order by is returned first.   
        /// Port 139 would be represented as  localPort1 =0, localPort2=139, localPort3=0, localPort4=0, where as 5800 would be localPort1=22, localPort2=168, etc...,
        /// MSDN shows these as being DWORD.
        /// </remarks>
        [StructLayout(LayoutKind.Sequential)]
        public struct MIB_TCP6ROW_OWNER_PID
        {
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 16)]
            private byte[] localAddr;
            private uint localScopeId;
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 4)]
            private byte[] localPort;
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 16)]
            private byte[] remoteAddr;
            private uint remoteScopeId;
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 4)]
            private byte[] remotePort;
            private uint state;
            private uint owningPid;

            public uint ProcessId
            {
                get { return owningPid; }
            }

            public long LocalScopeId
            {
                get { return localScopeId; }
            }

            public IPAddress LocalAddress
            {
                get { return new IPAddress(localAddr, LocalScopeId); }
            }

            public ushort LocalPort
            {
                get { return BitConverter.ToUInt16(localPort.Take(2).Reverse().ToArray(), 0); }
            }

            public long RemoteScopeId
            {
                get { return remoteScopeId; }
            }

            public IPAddress RemoteAddress
            {
                get { return new IPAddress(remoteAddr, RemoteScopeId); }
            }

            public ushort RemotePort
            {
                get { return BitConverter.ToUInt16(remotePort.Take(2).Reverse().ToArray(), 0); }
            }

            public MIB_TCP_STATE State
            {
                get { return (MIB_TCP_STATE)state; }
            }
        }

        /// <summary>
        /// The MIB_TCP6TABLE_OWNER_PID structure contains a table of process IDs (PIDs) and the IPv6 TCP links that are context bound to these PIDs.
        /// </summary>
        [StructLayout(LayoutKind.Sequential)]
        public struct MIB_TCP6TABLE_OWNER_PID
        {
            public uint dwNumEntries;
            [MarshalAs(UnmanagedType.ByValArray, ArraySubType = UnmanagedType.Struct, SizeConst = 1)]
            public MIB_TCP6ROW_OWNER_PID[] table;
        }

        /// <summary>
        /// The GetAllTCPConnections function uses generics to return a list of IPV4 TCP endpoints available to the application.
        /// </summary>
        /// <returns>List of <see cref="MIB_TCPROW_OWNER_PID"/></returns>
        public static List<MIB_TCPROW_OWNER_PID> GetAllTCPConnections()
        {
            return GetTCPConnections<MIB_TCPROW_OWNER_PID, MIB_TCPTABLE_OWNER_PID>(AF_INET);
        }

        /// <summary>
        /// The GetAllTCPConnections function uses generics to return a list of IPV6 TCP endpoints available to the application.
        /// </summary>
        /// <returns>List of <see cref="MIB_TCPROW_OWNER_PID"/></returns>
        public static List<MIB_TCP6ROW_OWNER_PID> GetAllTCPv6Connections()
        {
            return GetTCPConnections<MIB_TCP6ROW_OWNER_PID, MIB_TCP6TABLE_OWNER_PID>(AF_INET6);
        }

        /// <summary>
        /// The GetExtendedTcpTable function uses generics to return a list of IPV4/IPV6 TCP endpoints available to the application.
        /// </summary>
        /// <typeparam name="IPR">IP Row Type</typeparam>
        /// <typeparam name="IPT">IP Table Type</typeparam>
        /// <param name="ipVersion">IP Version</param>
        /// <returns>List of IP Row Type.</returns>
        private static List<IPR> GetTCPConnections<IPR, IPT>(int ipVersion)//IPR = Row Type, IPT = Table Type
        {
            IPR[] tableRows = null;
            int buffSize = 0;

            var dwNumEntriesField = typeof(IPT).GetField("dwNumEntries");

            // how much memory do we need?
            uint ret = GetExtendedTcpTable(IntPtr.Zero, ref buffSize, true, ipVersion, TCP_TABLE_CLASS.TCP_TABLE_OWNER_PID_LISTENER);  // ignores TIME_WAIT connections
            IntPtr tcpTablePtr = Marshal.AllocHGlobal(buffSize);

            try
            {
                ret = GetExtendedTcpTable(tcpTablePtr, ref buffSize, true, ipVersion, TCP_TABLE_CLASS.TCP_TABLE_OWNER_PID_LISTENER);  // ignores TIME_WAIT connections
                if (ret != 0)
                    return new List<IPR>();

                // get the number of entries in the table
                IPT table = (IPT)Marshal.PtrToStructure(tcpTablePtr, typeof(IPT));
                int rowStructSize = Marshal.SizeOf(typeof(IPR));
                uint numEntries = (uint)dwNumEntriesField.GetValue(table);

                // buffer we will be returning
                tableRows = new IPR[numEntries];

                IntPtr rowPtr = (IntPtr)((long)tcpTablePtr + 4);
                for (int i = 0; i < numEntries; i++)
                {
                    IPR tcpRow = (IPR)Marshal.PtrToStructure(rowPtr, typeof(IPR));
                    tableRows[i] = tcpRow;
                    rowPtr = (IntPtr)((long)rowPtr + rowStructSize);   // next entry
                }
            }
            finally
            {
                // Free the Memory
                Marshal.FreeHGlobal(tcpTablePtr);
            }
            return tableRows != null ? tableRows.ToList() : new List<IPR>();
        }

    }
}
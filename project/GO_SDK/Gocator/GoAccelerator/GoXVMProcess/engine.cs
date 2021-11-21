using Lmi3d.GoSdk;
using Lmi3d.Zen;
using Lmi3d.Zen.Io;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading.Tasks;
using System.IO;

namespace GoXVMProcess
{
    /// <summary>
    /// Static methods pertaining to core engine initialization.
    /// </summary>
    public static partial class Engine
    {
        /// <summary>
        /// The GoSystem singleton object;
        /// </summary>
        private static GoSystem SystemInstance;

        public static void Initialize()
        {
            string[] ips;
            KIpAddress addr;

            if (SystemInstance != null) { return; } // already initialized

            // Initialize dependency libraries
            KApiLib.Construct();
            GoSdkLib.Construct();
            

            // Create the GoSystem singleton object
            SystemInstance = new GoSystem(false);

            // Get list of IP addresses of interfaces that the Discovery Protocol can use.
            ips = GetIpList();
            if ((ips == null) || (ips.Length == 0))
            {
                // For backwards compatibility, enable discovery on all
                // interfaces if IP list is missing or empty.
                SystemInstance.SetAllDiscoveryInterface(true);
            }
            else
            {
                // Parse list and enable the interfaces.
                foreach (string ipString in ips)
                {
                    addr = KIpAddress.Parse(ipString);
                    SystemInstance.SetOneDiscoveryInterface(addr, true);
                }
            }

            // Start running the discovery protocol to find all sensors.
            SystemInstance.StartDiscovery();
        }

       

        /// <summary>
        /// Refreshes all sensor instances.
        /// </summary>
        internal static void SystemRefresh()
        {
            SystemInstance.Refresh();
        }

        /// <summary>
        /// Discovers all available sensors on the network.
        /// </summary>
        /// <returns>A list of sensor IDs.</returns>
        internal static List<uint> GetDeviceIds()
        {
            List<uint> deviceIds = new List<uint>();

            SystemRefresh();

            for (K64s i = 0; i < SystemInstance.SensorCount; i++)
            {
                GoSensor sensor = SystemInstance.GetSensor(i);

                deviceIds.Add(sensor.Id);
            }

            // sort the list of IDs
            deviceIds.Sort();

            return deviceIds;
        }

        /// <summar>
        /// Checks if a sensor object is local.
        /// </summar>
        /// <param name="id">A sensor ID.</param>
        internal static bool IsSensorLocal(uint id)
        {
            GoSensor sensor = SystemInstance.FindSensorById(id);

            if (sensor == null)
            {
                return false;
            }

            return sensor.Address().Address.IsLoopback;
        }

        /// <summary>
        /// Gets a GoSensor view model.
        /// </summary>
        /// <param name="id">A sensor ID.</param>
        internal static SensorVM GetSensorVM(uint id)
        {
            GoSensor sensor = SystemInstance.FindSensorById(id);

            if (sensor == null) { return null; }

            return new SensorVM(sensor);
        }

        /// <summary>
        /// Retrieve the firmware version for the SDK.
        /// </summary>
        /// <returns></returns>
        internal static string GetFirmwareVersion()
        {
            KVersion version = SystemInstance.SdkVersion();

            return version.Major.ToString() + "." + version.Minor.ToString() + "." + version.Release.ToString() + "." + version.Build.ToString();
        }

        internal static List<string> GetPossibleIpAddresses()
        {
            List<string> ips = new List<string>();

            // Get host name
            String strHostName = Dns.GetHostName();

            // Find host by name
            IPHostEntry iphostentry = Dns.GetHostEntry(strHostName);

            // Enumerate IP addresses
            foreach (IPAddress ip in iphostentry.AddressList)
            {
                // ignore IPv6 addresses
                if (ip.AddressFamily != AddressFamily.InterNetwork) { continue; }

                ips.Add(ip.ToString());
            }

            return ips;
        }

        private static string[] GetIpList()
        {
            string[] ips = null;
            string strFile = Directory.GetCurrentDirectory() + "\\" + "GoAccelIpFile.txt";
            if (File.Exists(strFile))
            {
                ips = System.IO.File.ReadAllLines(strFile);
            }
            return ips;
        }
    }
}

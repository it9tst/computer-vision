using Lmi3d.GoSdk;
using Lmi3d.Zen;
using Lmi3d.Zen.Io;
using System;
using System.Collections.Generic;
using System.Configuration;
using System.Net;
using System.Net.Sockets;
using System.IO;
using System.Threading;

namespace GoAcceleratorEngine
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

        // GOC-13597: Check if user.config file is corrupted.
        // Corrupted file is deleted.
        /// <summary>
        /// Checks if user.config file is corrupted
        /// </summary>
        public static bool IsValidSettingsFile()
        {
            try
            {
                var tempSettings = new SettingsVM();
                bool test = tempSettings.ShowTaskbarIcon;
            }
            catch (ConfigurationErrorsException configException)
            {
                if (configException.InnerException != null)
                {
                    File.Delete(((ConfigurationErrorsException) configException.InnerException).Filename);
                    Thread.Sleep(1000);
                }
                return false;
            }

            return true;
        }


        /// <summary>
        /// Refreshes all sensor instances.
        /// </summary>
        /// <remarks>
        /// <para>This should be called carefully, as it can cause synchronization issues (ie. unhandled exceptions)</para>
        /// <para>if the view model is not prepared for volatile changes in objects</para>
        /// <para>and properties -- all of a sudden due to sensors disappearing.</para>
        /// </remarks>
        internal static void SystemRefresh()
        {
            SystemInstance.Refresh();
        }

        /// <summary>
        /// Discovers all available sensors on the network.
        /// </summary>
        /// <returns>A list of sensor IDs.</returns>
        public static List<uint> GetDeviceIds()
        {
            List<uint> deviceIds = new List<uint>();

            for (K64s i = 0; i < SystemInstance.SensorCount; i++)
            {
                GoSensor sensor = SystemInstance.GetSensor(i);

                deviceIds.Add(sensor.Id);
                //sensor.Dispose();
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
        public static SensorVM GetSensorVM(uint id)
        {
            GoSensor sensor = SystemInstance.FindSensorById(id);

            if (sensor == null) { return null; }

            return new SensorVM(sensor);
        }

        /// <summary>
        /// Retrieve the firmware version for the SDK.
        /// </summary>
        /// <returns></returns>
        public static string GetFirmwareVersion()
        {
            KVersion version = SystemInstance.SdkVersion();

            return version.Major.ToString() + "." + version.Minor.ToString() + "." + version.Release.ToString() + "." + version.Build.ToString();
        }

        public static string GetFirmwareMainVersion()
        {
            KVersion version = SystemInstance.SdkVersion();

            return $"{version.Major}.{version.Minor}";
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

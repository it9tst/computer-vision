/*
 * BackupRestore.cs
 * 
 * Gocator 2000/2300 C# Sample
 * Copyright (C) 2013-2021 by LMI Technologies Inc.
 * 
 * Licensed under The MIT License.
 * Redistributions of files must retain the above copyright notice.
 *
 * Purpose: Backup a Gocator system and restore it
 *
 */

using System;
using Lmi3d.GoSdk;
using Lmi3d.Zen;
using Lmi3d.Zen.Io;

static class Constants
{
    public const string SENSOR_IP = "192.168.1.10"; // IP of the sensor used for sensor connection GoSystem_FindSensorByIpAddress() call.
}

namespace BackupRestore
{
    class BackupRestore
    {

        static int Main(string[] args)
        {
            try
            {
                KApiLib.Construct();
                GoSdkLib.Construct();
                GoSystem system = new GoSystem();
                GoSensor sensor;
                string backupFile = "SystemBackup.bin";

                KIpAddress ipAddress = KIpAddress.Parse(Constants.SENSOR_IP);
                sensor = system.FindSensorByIpAddress(ipAddress);
                sensor.Connect();
                Console.WriteLine("Creating backup file: {0}", backupFile);
                Console.WriteLine("-----------------------------");
                sensor.Backup(backupFile);
                Console.WriteLine("Restoring from backup file: {0}", backupFile);
                Console.WriteLine("-----------------------------");
                sensor.Restore(backupFile);
                // wait for ENTER key

            }
            catch (KException ex)
            {
                Console.WriteLine("Error: {0}", ex.ToString());
            }

            Console.WriteLine("\nPress ENTER to continue");
            while (Console.ReadKey().Key != ConsoleKey.Enter) { }

            return 1;


        }
    }
}

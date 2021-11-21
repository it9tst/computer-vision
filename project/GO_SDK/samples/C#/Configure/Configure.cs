/*
 * Configure.cs
 * 
 * Gocator 2000/2300 C# Sample
 * Copyright (C) 2013-2021 by LMI Technologies Inc.
 * 
 * Licensed under The MIT License.
 * Redistributions of files must retain the above copyright notice.
 *
 * Purpose: Connect to Gocator system and modify parameters
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

namespace Configure
{
    class Configure
    {

        static int Main(string[] args)
        {                  
            try
            {
                KApiLib.Construct();
                GoSdkLib.Construct();
                GoSystem system = new GoSystem();
                GoSensor sensor;
                double currentExposure, newExposure;
                KIpAddress ipAddress = KIpAddress.Parse(Constants.SENSOR_IP);    
                sensor = system.FindSensorByIpAddress(ipAddress);
                sensor.Connect();
                GoSetup setup = sensor.Setup;
                currentExposure = setup.GetExposure(GoRole.Main);
                Console.WriteLine("Current Parameters:");
                Console.WriteLine("-------------------");
                Console.WriteLine("Exposure: {0}", currentExposure);
                setup.SetExposure(GoRole.Main, currentExposure + 200);
                sensor.Flush();
                newExposure = setup.GetExposure(GoRole.Main);
                Console.WriteLine("New Parameters:");
                Console.WriteLine("-------------------");
                Console.WriteLine("Exposure: {0}", newExposure);
                sensor.CopyFile("_live.job", "newExposure.job");
                sensor.DefaultJob = "newExposure.job";
                setup.SetExposure(GoRole.Main, currentExposure);
            }

            catch (KException ex)
            {
                Console.WriteLine("Error: {0}", ex.ToString());
            }

            // wait for ENTER key
            Console.WriteLine("\nPress ENTER to continue");
            while (Console.ReadKey().Key != ConsoleKey.Enter) { }

            return 1;
        }
    }
}

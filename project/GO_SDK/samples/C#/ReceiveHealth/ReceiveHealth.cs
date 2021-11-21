/*
 * ReceiveHealth.cs
 * 
 * Gocator 2000/2300 C# Sample
 * Copyright (C) 2013-2021 by LMI Technologies Inc.
 * 
 * Licensed under The MIT License.
 * Redistributions of files must retain the above copyright notice.
 *
 * Purpose: Connect to Gocator system and receive health data
 *
 */

using System;
using Lmi3d.GoSdk;
using Lmi3d.Zen;
using Lmi3d.Zen.Io;
using Lmi3d.GoSdk.Messages;

static class Constants
{
    public const string SENSOR_IP = "192.168.1.10"; // IP of the sensor used for sensor connection GoSystem_FindSensorByIpAddress() call.
}

namespace ReceiveHealth
{
    class ReceiveHealth
    {
        static int Main(string[] args)
        {
            try
            {
                KApiLib.Construct();
                GoSdkLib.Construct();
                GoSystem system = new GoSystem();
                GoSensor sensor;
                GoDataSet healthData = new GoDataSet();
                KIpAddress ipAddress = KIpAddress.Parse(Constants.SENSOR_IP);
                sensor = system.FindSensorByIpAddress(ipAddress);
                sensor.Connect();
                system.EnableData(true);

                UInt32 i = 1;
                while (i < 10)
                {
                    healthData = system.ReceiveHealth(30000000);
                    for (UInt32 j = 0; j < healthData.Count; j++)
                    {
                        GoHealthMsg healthMsg = (GoHealthMsg)healthData.Get(j);
                        Console.WriteLine("Health message received:");
                        Console.WriteLine(" Number of indicators: {0}", healthMsg.Count);
                        for (UInt32 k = 0; k < healthMsg.Count; k++)
                        {
                            GoIndicator healthIndicator = healthMsg.Get(k);
                            Console.WriteLine(" Indicator[{0}]: Id:{1} Instance:{2} Value{3}", k, healthIndicator.id, healthIndicator.instance, healthIndicator.value);
                        }
                        // Dispose required to prevent memory leak.
                        healthMsg.Dispose();
                    }
                    i++;
                }

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

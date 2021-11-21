/*
 * AlignmentMoving.cs
 * 
 * Gocator 2000/2300 C# Sample
 * Copyright (C) 2013-2021 by LMI Technologies Inc.
 * 
 * Licensed under The MIT License.
 * Redistributions of files must retain the above copyright notice.
 *
 * Purpose: Connect to Gocator system and perform a moving alignment with a calibration disk
 *
 * NOTE! This code requires an encoder to be correctly connected to the system!
 *       Please verify correct encoder operation in the Dashboard page of the Gocator Web UI.
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

namespace AlignmentMoving
{
    class AlignmentMoving
    {
        static int Main(string[] args)
        {
            try
            {
                KApiLib.Construct();
                GoSdkLib.Construct();
                GoSystem system = new GoSystem();
                GoSensor sensor;
                GoDataSet dataSet = new GoDataSet();

                KIpAddress ipAddress = KIpAddress.Parse(Constants.SENSOR_IP);
                sensor = system.FindSensorByIpAddress(ipAddress);
                sensor.Connect();
                system.EnableData(true);
                GoSetup setup = sensor.Setup;
                setup.AlignmentType = GoAlignmentType.Moving;
                setup.AlignmentMovingTarget = GoAlignmentTarget.Disk;
                setup.DiskDiameter = 40;
                setup.DiskHeight = 6.25;
                setup.AlignmentEncoderCalibrateEnabled = true;
                sensor.Align();
                Console.WriteLine("Waiting for calibration disk to be scanned...");
                dataSet = system.ReceiveData(30000000);
                for (UInt32 i = 0; i < dataSet.Count; i++)
                {
                    GoAlignMsg dataItem = (GoAlignMsg)dataSet.Get(i);
                    if (dataItem.MessageType == GoDataMessageType.Alignment)
                    {
                        if (dataItem.Status == KStatus.Ok)
                        {
                            Console.WriteLine("Alignment calibration successful!");
                        }
                        else
                        {
                            Console.WriteLine("Alignment calibration failed.");
                        }
                    }
                }
                system.Stop();
            }
            
            catch (KException ex)
            {
                if (ex.Status == KStatus.ErrorTimeout)
                {
                    Console.WriteLine("No calibration disk detected within timeout..");
                }
                else if (ex.Status != KStatus.Ok)
                {
                    Console.WriteLine("Error:{0}", ex.ToString());
                }
            }

            // wait for ENTER key
            Console.WriteLine("\nPress ENTER to continue");
            while (Console.ReadKey().Key != ConsoleKey.Enter) { }
            return 1;
        }
    }
}

/*
 * DualSensor.cs
 * 
 * Gocator 2000/2300 C# Sample
 * Copyright (C) 2013-2021 by LMI Technologies Inc.
 * 
 * Licensed under The MIT License.
 * Redistributions of files must retain the above copyright notice.
 *
 * Purpose: Connect to Gocator buddy system and receive range data in Profile Mode and translate to engineering units (mm). Gocator must be in Profile Mmode.
 * Ethernet output for the profile data must be enabled. Gocator buddy is removed at the end.
 */

using System;
using Lmi3d.GoSdk;
using Lmi3d.Zen;
using Lmi3d.Zen.Io;
using Lmi3d.GoSdk.Messages;

static class Constants
{
    public const string SENSOR_IP = "192.168.1.10"; // IP of the main sensor used for sensor connection GoSystem_FindSensorByIpAddress() call.
    public const string BUDDY_IP = "192.168.1.11"; // IP of the buddy sensor used for sensor connection GoSystem_FindSensorByIpAddress() call.
}

namespace ReceiveProfile
{
    class ReceiveProfile
    {

        static int Main(string[] args)
        {
            try
            {
                KApiLib.Construct();
                GoSdkLib.Construct();
                GoSystem system = new GoSystem();
                GoSensor sensor;
                GoSensor buddy;
                GoDataSet dataSet = new GoDataSet();

                KIpAddress ipAddress = KIpAddress.Parse(Constants.SENSOR_IP);

                // Connect to sensor and set data handler
                sensor = system.FindSensorByIpAddress(ipAddress);
                sensor.Connect();
                system.EnableData(true);
                system.Stop();

                // check if the sensor already has a buddy sensor assigned
                if (sensor.HasBuddy() == false)
                {
                    ipAddress = KIpAddress.Parse(Constants.BUDDY_IP);
                    buddy = system.FindSensorByIpAddress(ipAddress);
                    buddy.Connect();
                    sensor.AddBuddyBlocking(buddy);
                    Console.WriteLine("Buddy {0} assigned.", sensor.BuddyId);
                    sensor.EnableData(true);
                }
                else
                {
                    Console.WriteLine("Gocator [{0}] already has Buddy [{1}] assigned.", sensor.Id, sensor.BuddyId);
                }

                sensor.Start();

                dataSet = system.ReceiveData(30000000);
                for (UInt32 i = 0; i < dataSet.Count; i++)
                {
                    GoDataMsg dataObj = (GoDataMsg)dataSet.Get(i);
                    switch (dataObj.MessageType)
                    {
                        case GoDataMessageType.Stamp:
                            {
                                GoStampMsg stampMsg = (GoStampMsg)dataObj;
                                for (UInt32 j = 0; j < stampMsg.Count; j++)
                                {
                                    GoStamp stamp = stampMsg.Get(j);
                                    Console.WriteLine("Frame Index = {0}", stamp.FrameIndex);
                                    Console.WriteLine("Time Stamp = {0}", stamp.Timestamp);
                                    Console.WriteLine("Encoder Value = {0}", stamp.Encoder);
                                }
                            }
                            break;

                        case GoDataMessageType.UniformProfile:
                            break;

                        case GoDataMessageType.ProfilePointCloud:
                            break;

                        case GoDataMessageType.ProfileIntensity:
                            break;
                    }
                }
                sensor.Stop();

                if (sensor.HasBuddy() == true)
                {
                    sensor.RemoveBuddy();
                    Console.WriteLine("Buddy sensor removed.");
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

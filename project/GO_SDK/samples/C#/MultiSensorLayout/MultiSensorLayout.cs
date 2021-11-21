/*
 * MultiSensorLayout
 * 
 * Gocator 2000/2300 C# Sample
 * Copyright (C) 2013-2021 by LMI Technologies Inc.
 * 
 * Licensed under The MIT License.
 * Redistributions of files must retain the above copyright notice.
 *
 * Purpose: Connect to Gocator N-buddy system and receive data.
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
    public const string BUDDY_IP_1 = "192.168.1.11"; // IP of the buddy1 sensor 
    public const string BUDDY_IP_2 = "192.168.1.12"; // IP of the buddy2 sensor
}

namespace MultiSensorLayout
{
    class MultiSensorLayout
    {

        static int Main(string[] args)
        {
            try
            {
                KApiLib.Construct();
                GoSdkLib.Construct();
                GoSystem system = new GoSystem();
                GoSensor sensor;
                GoSensor[] buddy = new GoSensor[2];
                GoDataSet dataSet = new GoDataSet();

                KIpAddress ipAddress = KIpAddress.Parse(Constants.SENSOR_IP);

                // Connect to main sensor and set data handler
                sensor = system.FindSensorByIpAddress(ipAddress);
                sensor.Connect();
                sensor.EnableData(true);
                sensor.RestoreDefaults(false);
                system.Stop();

                // Check if the sensor already has any buddy sensor assigned
                if (sensor.HasBuddy() == false)
                {
                    ipAddress = KIpAddress.Parse(Constants.BUDDY_IP_1);
                    buddy[0] = system.FindSensorByIpAddress(ipAddress);
                    buddy[0].Connect();
                    sensor.AddBuddyBlocking(buddy[0]);
                    Console.WriteLine("Buddy {0} assigned.", buddy[0].Id);

                    ipAddress = KIpAddress.Parse(Constants.BUDDY_IP_2);
                    buddy[1] = system.FindSensorByIpAddress(ipAddress);
                    buddy[1].Connect();
                    sensor.AddBuddyBlocking(buddy[1]);
                    Console.WriteLine("Buddy {0} assigned.", buddy[1].Id);
                }
                else
                {
                    Console.WriteLine("Gocator [{0}] already has Buddy [{1}] assigned.", sensor.Id, sensor.BuddyId);
                }

                system.Refresh();
                Console.WriteLine("buddy sensor count = {0}", sensor.BuddiesCount());

                GoLayout layout = sensor.Setup.GetLayout();
                GoSetup setup = sensor.Setup;

                layout.GridColumnCount = 3;
                layout.MultiplexBuddyEnabled = true;

                setup.SetLayoutGridRow(GoRole.Main, 0);
                Console.WriteLine("expected row 0, actual: {0}", setup.GetLayoutGridRow(GoRole.Main));
                setup.SetLayoutGridColumn(GoRole.Main, 0);
                Console.WriteLine("expected col 0, actual: {0}", setup.GetLayoutGridColumn(GoRole.Main));
                setup.SetLayoutGridDirection(GoRole.Main, 0);
                Console.WriteLine("expected direction False, actual: {0}", setup.GetLayoutGridDirection(GoRole.Main));
                setup.SetMultiplexingBank(GoRole.Main, 1);
                Console.WriteLine("expected bank 1, actual: {0}", setup.GetMultiplexingBank(GoRole.Main));

                setup.SetLayoutGridRow(GoRole.Main + 1, 0);
                Console.WriteLine("expected row 0, actual: {0}", setup.GetLayoutGridRow(GoRole.Main + 1));
                setup.SetLayoutGridColumn(GoRole.Main + 1, 1);
                Console.WriteLine("expected col 1, actual: {0}", setup.GetLayoutGridColumn(GoRole.Main + 1));
                setup.SetLayoutGridDirection(GoRole.Main + 1, 0);
                Console.WriteLine("expected direction False, actual: {0}", setup.GetLayoutGridDirection(GoRole.Main + 1));
                setup.SetMultiplexingBank(GoRole.Main + 1, 2);
                Console.WriteLine("expected bank 2, actual: {0}", setup.GetMultiplexingBank(GoRole.Main + 1));

                setup.SetLayoutGridRow(GoRole.Main + 2, 0);
                Console.WriteLine("expected row 1, actual: {0}", setup.GetLayoutGridRow(GoRole.Main + 2));
                setup.SetLayoutGridColumn(GoRole.Main + 2, 2);
                Console.WriteLine("expected col 1, actual: {0}", setup.GetLayoutGridColumn(GoRole.Main + 2));
                setup.SetLayoutGridDirection(GoRole.Main + 2, 0);
                Console.WriteLine("expected direction True, actual: {0}", setup.GetLayoutGridDirection(GoRole.Main + 2));
                setup.SetMultiplexingBank(GoRole.Main + 2, 1);
                Console.WriteLine("expected bank 1, actual: {0}", setup.GetMultiplexingBank(GoRole.Main + 2));
                sensor.Flush();

                sensor.Start();

                // Receive data messages
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

                // Remove all buddies
                while (sensor.HasBuddies() == true)
                {
                    sensor.RemoveBuddy();
                    Console.WriteLine("Buddy sensor removed.");
                }
            }
            catch (KException ex)
            {
                Console.WriteLine("Error: {0}", ex.ToString());
            }

            // Wait for ENTER key
            Console.WriteLine("\nPress ENTER to continue");
            while (Console.ReadKey().Key != ConsoleKey.Enter) { }
            return 1;
        }
    }
}

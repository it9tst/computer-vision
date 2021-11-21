/*
 * ReceiveAsync.cs
 * 
 * Gocator 2000/2300 C# Sample
 * Copyright (C) 2013-2021 by LMI Technologies Inc.
 * 
 * Licensed under The MIT License.
 * Redistributions of files must retain the above copyright notice.
 *
 * Purpose: Connect to Gocator system and receive data using a callback function
 * Ethernet output for the desired data must be enabled.
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

namespace ReceiveAsync
{
    class ReceiveAsync
    {
        // Data callback function
        // This function is called from a separate thread spawned by the GoSDK library.
        // Processing within this function should be minimal.
        public static void onData(KObject data)
        {
            GoDataSet dataSet = (GoDataSet)data;
            for (UInt32 i = 0; i < dataSet.Count; i++)
            {
                GoDataMsg dataObj = (GoDataMsg)dataSet.Get(i);
                switch(dataObj.MessageType)
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
                }
            }

            // Refer to example ReceiveRange, ReceiveProfile, ReceiveMeasurement and ReceiveWholePart on how to receive data
            Console.WriteLine(Environment.NewLine);
        }

        static int Main(string[] args)
        {
            try
            {
                KApiLib.Construct();
                GoSdkLib.Construct();
                GoSystem system = new GoSystem();
                GoSensor sensor;
                KIpAddress ipAddress = KIpAddress.Parse(Constants.SENSOR_IP);

                // Connect to sensor and set data handler
                sensor = system.FindSensorByIpAddress(ipAddress);
                sensor.Connect();
                system.EnableData(true);
                system.SetDataHandler(onData);

                // Start the sensor/system. After this call, the GoSDK library
                // will start receiving data from the sensor. The data handler
                // function will be called by the SDK in a separate thread
                // asynchonously.
                system.Start();

                // Receive data until the user presses the Enter key
                Console.WriteLine("\nPress ENTER to stop");
                while (Console.ReadKey().Key != ConsoleKey.Enter) { }

                system.Stop();
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

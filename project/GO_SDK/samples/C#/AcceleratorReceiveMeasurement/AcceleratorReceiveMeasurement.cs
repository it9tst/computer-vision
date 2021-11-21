/*
 * AcceleratorReceiveMeasurement.cs
 * 
 * Gocator 2000 Sample
 * Copyright (C) 2011-2021 by LMI Technologies Inc.
 * 
 * Licensed under The MIT License.
 * Redistributions of files must retain the above copyright notice.
 *
 * Purpose: Demonstrates the simple use of the Accelerator by connecting to a sensor and receiving a measurement.
 * This allows processing to be performed on the PC rather than on the sensor.
 */

using System;
using Lmi3d.GoSdk;
using Lmi3d.Zen;
using Lmi3d.Zen.Io;
using Lmi3d.GoSdk.Messages;

static class Constants
{
    public const string SENSOR_IP = "192.168.1.10"; // IP of the sensor used for sensor connection GoSystem_FindSensorByIpAddress() call.
    public const uint WEB_PORT = 8081;
}

namespace AcceleratorReceiveMeasurement
{
    class AcceleratorReceiveMeasurement
    {
        static int Main(string[] args)
        {
            try
            {
                KApiLib.Construct();
                GoSdkLib.Construct();
                GoSystem system = new GoSystem();
                GoAccelerator accelerator = new GoAccelerator();
                GoSensor sensor;
                KIpAddress ipAddress = KIpAddress.Parse(Constants.SENSOR_IP);
                GoDataSet dataSet = new GoDataSet();

                accelerator.WebPort = Constants.WEB_PORT;
                accelerator.Start();
                sensor = system.FindSensorByIpAddress(ipAddress);
                accelerator.Attach(sensor);
                sensor.Connect();
                system.EnableData(true);
                system.Start();
                // refer to SetupMeasurement.cs for setting up of the measurement tools
                dataSet = system.ReceiveData(30000000);
                for (uint i = 0; i < dataSet.Count; i++)
                {

                    GoDataMsg dataObj = (GoDataMsg)dataSet.Get(i);
                    switch (dataObj.MessageType)
                    {
                        case GoDataMessageType.Stamp:
                            {
                                GoStampMsg stampMsg = (GoStampMsg)dataObj;
                                for (uint j = 0; j < stampMsg.Count; j++)
                                {
                                    GoStamp stamp = stampMsg.Get(j);
                                    Console.WriteLine("Frame Index = {0}", stamp.FrameIndex);
                                    Console.WriteLine("Time Stamp = {0}", stamp.Timestamp);
                                    Console.WriteLine("Encoder Value = {0}", stamp.Encoder);
                                }
                            }
                            break;
                        case GoDataMessageType.Measurement:
                            {
                                GoMeasurementMsg measurementMsg = (GoMeasurementMsg)dataObj;
                                for (uint k = 0; k < measurementMsg.Count; ++k)
                                {
                                    GoMeasurementData measurementData = measurementMsg.Get(k);
                                    Console.WriteLine("ID: {0}", measurementMsg.Id);
                                    Console.WriteLine("Value: {0}", measurementData.Value);
                                    Console.WriteLine("Decision: {0}", measurementData.Decision);
                                }
                            }
                            break;
                    }
                }
                system.Stop();
                accelerator.Detach(sensor);
                accelerator.Stop();
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

/*
 * ReceiveMeasurements.cs
 * 
 * Gocator 2000/2300 C# Sample
 * Copyright (C) 2013-2021 by LMI Technologies Inc.
 * 
 * Licensed under The MIT License.
 * Redistributions of files must retain the above copyright notice.
 *
 * Purpose: Connect to Gocator system, setup and receive measurement data under Profile Mode. Measurements should have been setup and have the outputs enabled via Ethernet.
 * Please refer to SetupMeasurement.c for configuring the measurements.
 */

using System;
using Lmi3d.GoSdk;
using Lmi3d.Zen;
using Lmi3d.Zen.Io;
using Lmi3d.GoSdk.Messages;
using Lmi3d.GoSdk.Tools;

static class Constants
{
    public const string SENSOR_IP = "192.168.1.10"; // IP of the sensor used for sensor connection GoSystem_FindSensorByIpAddress() call.
}

namespace ReceiveMeasurement
{
    class ReceiveMeasurement
    {
        static int Main(string[] args)
        {
            try
            {
                KApiLib.Construct();
                GoSdkLib.Construct();
                GoSystem system = new GoSystem();
                GoSensor sensor;
                KIpAddress ipAddress = KIpAddress.Parse(Constants.SENSOR_IP);
                GoDataSet dataSet = new GoDataSet();
                sensor = system.FindSensorByIpAddress(ipAddress);
                sensor.Connect();
                system.EnableData(true);
                system.Start();
                UInt32 id;
                GoTools collection_tools;
                // refer to SetupMeasurement.cs for setting up of the measurement tools
                dataSet = system.ReceiveData(30000000);
                // retrieve tools handle
               collection_tools = sensor.Tools;

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
                        case GoDataMessageType.Measurement:
                            {
                                GoMeasurementMsg measurementMsg = (GoMeasurementMsg)dataObj;
                                for (UInt32 k = 0; k < measurementMsg.Count; ++k)
                                {
                                    GoMeasurementData measurementData = measurementMsg.Get(k);

                                    Console.WriteLine("ID: {0}", measurementMsg.Id);

                                    //1. Retrieve Id
                                    id = measurementMsg.Id;
                                    //2. Retrieve the measurement from the set of tools using measurement ID
                                    GoMeasurement measurement = collection_tools.FindMeasurementById(id);
                                    //3. Print the measurement name 
                                    Console.WriteLine("Measurement Name is : {0}", measurement.Name);
                                    Console.WriteLine("Value: {0}", measurementData.Value);
                                    Console.WriteLine("Decision: {0}", measurementData.Decision);
                                }
                            }
                            break;
                    }
                }
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

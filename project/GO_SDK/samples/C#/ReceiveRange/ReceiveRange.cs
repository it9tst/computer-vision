/*
 * ReceiveRange.cs
 * 
 * Gocator 2000 Sample
 * Copyright (C) 2011-2021 by LMI Technologies Inc.
 * 
 * Licensed under The MIT License.
 * Redistributions of files must retain the above copyright notice.
 *
 * Purpose: Connect to Gocator system and receive range data in Range Mode and translate to engineering units (mm).
 * Ethernet output for range data must be enabled.
 */

using System;
using System.Runtime.InteropServices;
using Lmi3d.GoSdk;
using Lmi3d.Zen;
using Lmi3d.Zen.Io;
using Lmi3d.GoSdk.Messages;

static class Constants
{
    public const string SENSOR_IP = "192.168.1.10"; // IP of the sensor used for sensor connection GoSystem_FindSensorByIpAddress() call.
}

namespace ReceiveRange
{
    public class DataContext
    {
        public Double xResolution;
        public Double zResolution;
        public Double xOffset;
        public Double zOffset;
        public uint serialNumber;
    }

    class ReceiveRange
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
                dataSet = system.ReceiveData(30000000);
                DataContext context = new DataContext();
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
                        case GoDataMessageType.Range:
                            {
                                GoRangeMsg rangeMsg = (GoRangeMsg)dataObj;
                                Console.WriteLine("  Range Message Batch count: {0}", rangeMsg.Count);
                                for (UInt32 k = 0; k < rangeMsg.Count; ++k)
                                {
                                    long msgCount = rangeMsg.Count;
                                    context.zResolution = rangeMsg.ZResolution / 1000000;
                                    context.zOffset = rangeMsg.ZOffset / 1000;
                                    short[] range = new short[msgCount];
                                    IntPtr rangePtr = rangeMsg.Data;
                                    Marshal.Copy(rangePtr, range, 0, range.Length);
                                }
                            }
                            break;
                        case GoDataMessageType.RangeIntensity:
                            {
                                GoRangeMsg rangeMsg = (GoRangeMsg)dataObj;
                                Console.WriteLine("  Range Intensity Message batch count: {0}", rangeMsg.Count);
                                for (UInt32 k = 0; k < rangeMsg.Count; ++k)
                                {
                                    byte[] range = new byte[rangeMsg.Count];
                                    IntPtr rangePtr = rangeMsg.Data;
                                    Marshal.Copy(rangePtr, range, 0, range.Length);
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

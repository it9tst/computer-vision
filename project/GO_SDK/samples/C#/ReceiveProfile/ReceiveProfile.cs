/*
 * ReceiveProfile.cs
 * 
 * Gocator 2000/2300 C# Sample
 * Copyright (C) 2013-2021 by LMI Technologies Inc.
 * 
 * Licensed under The MIT License.
 * Redistributions of files must retain the above copyright notice.
 *
 * Purpose: Connect to Gocator system and receive range data in Profile Mode and translate to engineering units (mm). Gocator must be in Profile Mmode.
 * Ethernet output for the profile data must be enabled.
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

namespace ReceiveProfile
{
    public class DataContext
    {
        public Double xResolution;
        public Double zResolution;
        public Double xOffset;
        public Double zOffset;
        public uint serialNumber;
    }

    public struct ProfilePoint
    {
        public double x;
        public double z;
        byte intensity;
    }

    public struct GoPoints
    {
        public Int16 x;
        public Int16 y;
    }

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

                        case GoDataMessageType.UniformProfile:
                            {
                                GoUniformProfileMsg profileMsg = (GoUniformProfileMsg)dataObj;
                                Console.WriteLine("  Resampled Profile Message batch count: {0}", profileMsg.Count);
                                for (UInt32 k = 0; k < profileMsg.Count; ++k)
                                {
                                    int validPointCount = 0;
                                    int profilePointCount = profileMsg.Width;
                                    Console.WriteLine("    Item[{0}]: Profile data ({1} points)", k, profileMsg.Width);
                                    context.xResolution = (double)profileMsg.XResolution / 1000000;
                                    context.zResolution = (double)profileMsg.ZResolution / 1000000;
                                    context.xOffset = (double)profileMsg.XOffset / 1000;
                                    context.zOffset = (double)profileMsg.ZOffset / 1000;

                                    short[] points = new short[profilePointCount];
                                    ProfilePoint[] profileBuffer = new ProfilePoint[profilePointCount];
                                    IntPtr pointsPtr = profileMsg.Data;
                                    Marshal.Copy(pointsPtr, points, 0, points.Length);

                                    for (UInt32 arrayIndex = 0; arrayIndex < profilePointCount; ++arrayIndex)
                                    {
                                        if (points[arrayIndex] != -32768)
                                        {
                                            profileBuffer[arrayIndex].x = context.xOffset + context.xResolution * arrayIndex;
                                            profileBuffer[arrayIndex].z = context.zOffset + context.zResolution * points[arrayIndex];
                                            validPointCount++;
                                        }
                                        else
                                        {
                                            profileBuffer[arrayIndex].x = context.xOffset + context.xResolution * arrayIndex;
                                            profileBuffer[arrayIndex].z = -32768;
                                        }
                                    }
                                    Console.WriteLine("Received {0} Range Points", profilePointCount);
                                    Console.WriteLine("Valid Points {0}", validPointCount);
                                }
                            }
                            break;

                        case GoDataMessageType.ProfilePointCloud:
                            {
                                GoProfilePointCloudMsg profileMsg = (GoProfilePointCloudMsg)dataObj;
                                Console.WriteLine("  Profile Message batch count: {0}", profileMsg.Count);
                                for (UInt32 k = 0; k < profileMsg.Count; ++k)
                                {
                                    int validPointCount = 0;
                                    long profilePointCount = profileMsg.Width;
                                    Console.WriteLine("    Item[{0}]: Profile data ({1} points)", i, profileMsg.Width);
                                    context.xResolution = profileMsg.XResolution / 1000000;
                                    context.zResolution = profileMsg.ZResolution / 1000000;
                                    context.xOffset = profileMsg.XOffset / 1000;
                                    context.zOffset = profileMsg.ZOffset / 1000;
                                    GoPoints[] points = new GoPoints[profilePointCount];
                                    ProfilePoint[] profileBuffer = new ProfilePoint[profilePointCount];
                                    int structSize = Marshal.SizeOf(typeof(GoPoints));
                                    IntPtr pointsPtr = profileMsg.Data;
                                    for (UInt32 array = 0; array < profilePointCount; ++array)
                                    {
                                        IntPtr incPtr = new IntPtr(pointsPtr.ToInt64() + array * structSize);
                                        points[array] = (GoPoints)Marshal.PtrToStructure(incPtr, typeof(GoPoints));
                                    }

                                    for (UInt32 arrayIndex = 0; arrayIndex < profilePointCount; ++arrayIndex)
                                    {
                                        if (points[arrayIndex].x != -32768)
                                        {
                                            profileBuffer[arrayIndex].x = context.xOffset + context.xResolution * points[arrayIndex].x;
                                            profileBuffer[arrayIndex].z = context.zOffset + context.zResolution * points[arrayIndex].y;
                                            validPointCount++;
                                        }
                                        else
                                        {
                                            profileBuffer[arrayIndex].x = -32768;
                                            profileBuffer[arrayIndex].z = -32768;
                                        }
                                    }
                                    Console.WriteLine("Received {0} Range Points", profilePointCount);
                                    Console.WriteLine("Valid Points {0}", validPointCount);
                                }
                            }
                            break;

                        case GoDataMessageType.ProfileIntensity:
                            {
                                GoProfileIntensityMsg profileMsg = (GoProfileIntensityMsg)dataObj;
                                Console.WriteLine("  Profile Intensity Message batch count: {0}", profileMsg.Count);
                                for (UInt32 k = 0; k < profileMsg.Count; ++k)
                                {
                                    byte[] intensity = new byte[profileMsg.Width];
                                    IntPtr intensityPtr = profileMsg.Data;
                                    Marshal.Copy(intensityPtr, intensity, 0, intensity.Length);
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

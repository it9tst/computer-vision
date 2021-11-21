/*
 * ReceiveSurface.cs
 * 
 * Gocator 2000/2300 C# Sample
 * Copyright (C) 2013-2021 by LMI Technologies Inc.
 * 
 * Licensed under The MIT License.
 * Redistributions of files must retain the above copyright notice.
 *
 * Purpose: Connect to Gocator system and receive Surface data and translate to engineering units. Gocator must be in Surface Mode.
 * Ethernet output for the surface and/or intensity data must be enabled.
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

namespace ReceiveSurface
{
    public class DataContext
    {
        public double xResolution;
        public double yResolution;
        public double zResolution;
        public double xOffset;
        public double yOffset;
        public double zOffset;
        public uint serialNumber;
    }

    public struct GoPoints
    {
        public Int16 x;
        public Int16 y;
        public Int16 z;
    }

    public struct SurfacePoint
    {
        public double x;
        public double y;
        public double z;
        byte intensity;
    }
    class ReceiveSurface
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
                GoSetup setup = sensor.Setup;
                setup.ScanMode = GoMode.Surface;
                system.EnableData(true);
                system.Start();
                Console.WriteLine("Waiting for Surface Data...");
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
                        case GoDataMessageType.UniformSurface:
                            {
                                GoUniformSurfaceMsg surfaceMsg = (GoUniformSurfaceMsg)dataObj;
                                long width = surfaceMsg.Width;
                                long length = surfaceMsg.Length;
                                long bufferSize = width * length;
                                IntPtr bufferPointer = surfaceMsg.Data;

                                Console.WriteLine("Uniform Surface received:");
                                Console.WriteLine(" Buffer width: {0}", width);
                                Console.WriteLine(" Buffer length: {0}", length);

                                short[] ranges = new short[bufferSize];
                                Marshal.Copy(bufferPointer, ranges, 0, ranges.Length);

                            }
                            break;
                        case GoDataMessageType.SurfacePointCloud:
                            {
                                GoSurfacePointCloudMsg surfaceMsg = (GoSurfacePointCloudMsg)dataObj;
                                context.xResolution = (double)surfaceMsg.XResolution / 1000000;
                                context.yResolution = (double)surfaceMsg.YResolution / 1000000;
                                context.zResolution = (double)surfaceMsg.ZResolution / 1000000;
                                context.xOffset = (double)surfaceMsg.XOffset / 1000;
                                context.yOffset = (double)surfaceMsg.YOffset / 1000;
                                context.zOffset = (double)surfaceMsg.ZOffset / 1000;
                                long surfacePointCount = surfaceMsg.Width * surfaceMsg.Length;
                                Console.WriteLine("Surface Point Cloud received:");
                                Console.WriteLine(" Buffer width: {0}", surfaceMsg.Width);
                                Console.WriteLine(" Buffer length: {0}", surfaceMsg.Length);
                                GoPoints[] points = new GoPoints[surfacePointCount];
                                SurfacePoint[] surfaceBuffer = new SurfacePoint[surfacePointCount];
                                int structSize = Marshal.SizeOf(typeof(GoPoints));
                                IntPtr pointsPtr = surfaceMsg.Data;
                                for (UInt32 array = 0; array < surfacePointCount; ++array)
                                {
                                    IntPtr incPtr = new IntPtr(pointsPtr.ToInt64() + array * structSize);
                                    points[array] = (GoPoints)Marshal.PtrToStructure(incPtr, typeof(GoPoints));
                                }
                                for (UInt32 arrayIndex = 0; arrayIndex < surfacePointCount; ++arrayIndex)
                                {
                                    if (points[arrayIndex].x != -32768)
                                    {
                                        surfaceBuffer[arrayIndex].x = context.xOffset + context.xResolution * points[arrayIndex].x;
                                        surfaceBuffer[arrayIndex].y = context.yOffset + context.yResolution * points[arrayIndex].y;
                                        surfaceBuffer[arrayIndex].z = context.zOffset + context.zResolution * points[arrayIndex].z;
                                    }
                                    else
                                    {
                                        surfaceBuffer[arrayIndex].x = -32768;
                                        surfaceBuffer[arrayIndex].y = -32768;
                                        surfaceBuffer[arrayIndex].z = -32768;
                                    }
                                }

                            }
                            break;
                        case GoDataMessageType.SurfaceIntensity:
                            {
                                GoSurfaceIntensityMsg surfaceMsg = (GoSurfaceIntensityMsg)dataObj;
                                long width = surfaceMsg.Width;
                                long length = surfaceMsg.Length;
                                long bufferSize = width * length;
                                IntPtr bufferPointeri = surfaceMsg.Data;

                                Console.WriteLine("Surface Intensity received:");
                                Console.WriteLine(" Buffer width: {0}", width);
                                Console.WriteLine(" Buffer length: {0}", length);
                                byte[] ranges = new byte[bufferSize];
                                Marshal.Copy(bufferPointeri, ranges, 0, ranges.Length);
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

/*
 * SetupMeasurement.cs
 * 
 * Gocator 2000/2300 C# Sample
 * Copyright (C) 2013-2021 by LMI Technologies Inc.
 * 
 * Licensed under The MIT License.
 * Redistributions of files must retain the above copyright notice.
 *
 * Requirement: Run on G2 sensor in Profile mode.
 * 
 * Purpose: Connect to Gocator system, setup measurement data under Profile Mode.
 * Please refer to ReceiveMeasurement.c for receiving of the measurement data.
 */

using System;
using Lmi3d.GoSdk;
using Lmi3d.GoSdk.Tools;
using Lmi3d.GoSdk.Outputs;
using Lmi3d.Zen;
using Lmi3d.Zen.Io;

static class Constants
{
    public const string SENSOR_IP = "192.168.1.10"; // IP of the sensor used for sensor connection GoSystem_FindSensorByIpAddress() call.
}

namespace SetupMeasurement
{
    class SetupMeasurement
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
                sensor = system.FindSensorByIpAddress(ipAddress);
                sensor.Connect();
                system.EnableData(true);
                //retrieve setup handle
                GoSetup setup = sensor.Setup;
                //retrieve tools handle
                GoTools tools = sensor.Tools;
                // add ProfilePosition tool, retreive tool handle
                GoProfilePosition profilePositionTool = (GoProfilePosition)tools.AddTool(GoToolType.ProfilePosition);
                // set name for tool
                profilePositionTool.Name = "Profile position Test";
                // add Z measurement for ProfilePosition tool
                GoProfilePositionZ zProfileMeasurementTop = profilePositionTool.ZMeasurement;
                zProfileMeasurementTop.Enabled = true;
                zProfileMeasurementTop.Id = 0;
                //set ProfilePosition feature to top
                GoProfileFeature profileFeature = profilePositionTool.Feature;
                profileFeature.FeatureType = GoProfileFeatureType.MaxZ;
                // set the ROI to fill the entire active area
                GoProfileRegion regionTop = profileFeature.Region;
                regionTop.X = setup.GetTransformedDataRegionX(GoRole.Main);
                regionTop.Z = setup.GetTransformedDataRegionZ(GoRole.Main);
                regionTop.Height = setup.GetTransformedDataRegionHeight(GoRole.Main);
                regionTop.Width = setup.GetTransformedDataRegionWidth(GoRole.Main);
                // enable Ethernet output for measurement tool
                GoOutput output = sensor.Output;
                GoEthernet ethernetOutput = output.GetEthernet();
                ethernetOutput.ClearAllSources();
                ethernetOutput.AddSource(GoOutputSource.Measurement, 0);
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

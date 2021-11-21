using System;
using Lmi3d.Zen;
using Lmi3d.Zen.Data;
using Lmi3d.Zen.Threads;
using Lmi3d.GoSdk;
using Lmi3d.GoSdk.Messages;
using Lmi3d.GoSdk.Tools;
using Lmi3d.GoSdk.Outputs;
using Lmi3d.Zen.Io;
using System.Collections.Generic;


namespace GoSdkNetExample
{
    public class GoSdkNetExample
    {
        static void Main(string[] args)
        {
            using (GoSdkLib sdk = GoSdkLib.Construct())
            {
                GoSystem system = new GoSystem();

                GoSensor sensor = system.FindSensorById(00000);

                sensor.Connect();
                sensor.Disconnect();
                system.Dispose();
            }
        }
    }
}

﻿//------------------------------------------------------------------------------
// <auto-generated>
//     This code was generated by a tool.
//     Runtime Version:4.0.30319.42000
//
//     Changes to this file may cause incorrect behavior and will be lost if
//     the code is regenerated.
// </auto-generated>
//------------------------------------------------------------------------------

using System;

namespace GoAcceleratorEngine.Properties
{
    public enum AccelerationState
    {
        Stopping = 0,
        Stopped,
        Starting,
        Started,
    }
    [Serializable]
    public class SensorSettings
    {
        public uint BasePort { get; set; }
        public uint WebPort { get; set; }
        public uint Id { get; set; }
        public bool Accelerated { get; set; }
        public string AcceleratedSensorIp { get; set; }
        public AccelerationState AcceleratedState { get; set; }
    }
}
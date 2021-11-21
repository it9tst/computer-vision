using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace GoXVMProcess
{
    public class CommandLineArguments
    {
        public uint BasePort { get; set; } = 3190;
        public uint HealthPort { get; set; } = 3191;
        public uint UpgradePort { get; set; } = 3192;
        public uint PublicDataPort { get; set; } = 3194;
        public uint PrivateDataPort { get; set; } = 3195;
        
        public uint WebPort { get; set; } = 8080;
        public uint SerialNumber { get; set; } = 0;
        public string Address { get; set; } = "0.0.0.0";
        public bool IgnoreStopEvent { get; set; } = false;
    }
}

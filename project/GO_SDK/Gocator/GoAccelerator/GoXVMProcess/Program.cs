using GoXVMProcess;
using Lmi3d.GoSdk;
using Lmi3d.Zen;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace GoXVMProcess
{
    static class Program
    {
        /// The main entry point for the application.
        [STAThread]
        static void Main(string[] args)
        {
            try
            {
                CommandLineSwitch.TryParse<CommandLineArguments>(ref args, out var options, out var err);
                Application.Run(new AppContext(options.SerialNumber, options.WebPort, options.BasePort, options.HealthPort, options.UpgradePort,
                    options.PrivateDataPort, options.PublicDataPort, options.Address, options.IgnoreStopEvent));
            }
            catch (Exception e){
                GoAcceleratorEngine.Utilities.Log(e.Message, 0);
            }
        }
    }
}

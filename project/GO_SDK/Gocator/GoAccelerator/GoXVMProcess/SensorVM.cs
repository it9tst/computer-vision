using Lmi3d.GoSdk;
using Lmi3d.Zen;
using Lmi3d.Zen.Io;
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Input;


namespace GoXVMProcess
{
    /// <summary>
    /// View model for a GoSensor
    /// </summary>
    public class SensorVM
    {
        private GoSensor _sensor;
        private GoAccelerator _accelerator;
        private EventWaitHandle _ewhstatusonline = null;
        private EventWaitHandle _ewhstatusoffline = null;
        private EventWaitHandle _ewhstatusError = null;

        internal delegate void UpdateFx(GoAcceleratorConnectionStatus value);
        internal delegate void IsBusyDelegate(bool isBusy);

        internal SensorVM(GoSensor sensor)
        {
            try
            {
                _sensor = sensor;

                // Create an accelerator instance and start it immediately
                _accelerator = new GoAccelerator();
                _accelerator.Start();
                if (_ewhstatusonline != null)
                {
                    _ewhstatusonline = EventWaitHandle.OpenExisting("LMI.GoX.Status.Online" + sensor.Id);
                    _ewhstatusoffline = EventWaitHandle.OpenExisting("LMI.GoX.Status.Offline" + sensor.Id);
                    //_ewhstatusError = EventWaitHandle.OpenExisting("LMI.GoX.Status.Error" + sensor.Id);
                }
            }
            catch (KException kEx)
            {
                GoAcceleratorEngine.Utilities.Log(kEx.Message, EventLogEntryType.Error);
            }
            catch (Exception e)
            {
                GoAcceleratorEngine.Utilities.Log(e.Message, EventLogEntryType.Error);
            }

        }

        ~SensorVM()
        {
            try
            {
                _accelerator.Stop();
            }
            catch //(KException kEx)
            {
                //GoAcceleratorEngine.Utilities.Log(kEx.Message, kEx.HResult);
                //}
                //catch (Exception e)
                //{
                //GoAcceleratorEngine.Utilities.Log(e.Message, 0);
            }
        }

        public uint WebPort
        {
            set
            {
                _accelerator.WebPort = value;
            }
        }

        public uint BasePort
        {

            set
            {
                //make ports sequential from base
                _accelerator.ControlPort = value;
            }
        }


        public uint HealthPort
        {
            set
            {
                _accelerator.HealthPort = value;
            }

        }

        public uint PrivateDataPort
        {
            set
            {
                _accelerator.PrivateDataPort = value;
            }
        }

        public uint PublicDataPort
        {
            set
            {
                _accelerator.PublicDataPort = value;
            }

        }
        public uint UpgradePort
        {
            set
            {
                _accelerator.UpgradePort = value;
            }
            get
            {
                return _accelerator.UpgradePort;
            }
        }

        public KIpAddress Address
        {
            set
            {
                _accelerator.Address = value;
            }
            get
            {
                return _accelerator.Address;
            }
        }

        /// <summary>
        /// Enables/disables acceleration.
        /// </summary>
        public bool AccelerationEnabled
        {
            set
            {
                if (value)
                {
                    StartAcceleration();
                }
                else
                {
                    StopAcceleration();
                }
            }
        }

        private void StopAcceleration()
        {
            try
            {
                // Stop accelerating this sensor
                _accelerator.Detach(_sensor);
            }
            catch (KException kEx)
            {
                GoAcceleratorEngine.Utilities.Log(kEx.Message, EventLogEntryType.Error);
                throw kEx;
            }
            catch (Exception e)
            {
                GoAcceleratorEngine.Utilities.Log(e.Message, EventLogEntryType.Error);
                throw e;
            }
            
        }
        void UpdateStatus(GoAcceleratorConnectionStatus value)
        {
            if (value == GoAcceleratorConnectionStatus.Connected)
            {
                _ewhstatusonline.Set();
            }
            else if (value == GoAcceleratorConnectionStatus.Disconnected)
            {
                _ewhstatusoffline.Set();
            }
            else
            {
                _ewhstatusError.Set();
            }
        }
        private void StartAcceleration()
        {
            try
            {
                GoAcceleratorEngine.Utilities.Log("StartAcceleration()...");
                _accelerator.SetAcceleratorUpdateHandler(UpdateStatus);
                // Start accelerating this sensor
                _accelerator.Attach(_sensor);
            }
            catch (KException kEx)
            {
                GoAcceleratorEngine.Utilities.Log(kEx.Message, EventLogEntryType.Error);
                throw kEx;
            }
            catch(Exception e)
            {
                GoAcceleratorEngine.Utilities.Log(e.Message, EventLogEntryType.Error);
                throw e;
            }
        }
    }
}

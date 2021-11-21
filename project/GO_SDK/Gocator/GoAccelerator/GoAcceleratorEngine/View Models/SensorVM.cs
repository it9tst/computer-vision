using GoAcceleratorEngine.Properties;
using Lmi3d.GoSdk;
using Lmi3d.Zen;
using Lmi3d.Zen.Io;
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Diagnostics;
using System.IO.Pipes;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Input;


namespace GoAcceleratorEngine
{
    /// <summary>
    /// View model for a GoSensor
    /// </summary>
    public class SensorVM : INotifyPropertyChanged
    {
        private GoSensor _sensor;
        private SensorSettings _sensorSettings;
        private EventWaitHandle _ewhlogoff = null;
        private EventWaitHandle _ewhstart = null;
        private RegisteredWaitHandle _regewhstart = null;
        private EventWaitHandle _ewhstop = null;
        private System.Threading.Timer autoRefreshTimer = null;
        private Process process;
        private bool _online = true;
        private string _lasterror = "";
        private int autoRefreshTimerCount = 0;

        #region consts
        /// <summary>
        /// Vmx start timeout in milliseconds. 
        /// </summary>
        public const int StartVmTimeout = (10 * 60 * 1000); // vmx start timeout in milliseconds - 10 mins is extreme 

        /// <summary>
        /// Vmx stop timeout in milliseconds.
        /// </summary>
        public const int StopVmTimeout = (1 * 60 * 1000); // vmx stop timeout in milliseconds (1min)

        /// <summary>
        /// Vmx abort timeout in milliseconds.
        /// </summary>
        public const int AbortVmTimeout = (5000); // vmx abort timeout in milliseconds (5 secs)

        /// <summary>
        /// Refresh timer and timer max timer count.
        /// </summary>
        private const int autoRefreshTimerDelay = (1000);
        private const int autoRefreshTimerCountMax = 3;

        #endregion


        #region events
        public EventWaitHandle EWHLogoff
        {
            get
            {
                return _ewhlogoff;
            }
        }
        public EventWaitHandle EWHStart
        {
            get
            {
                return _ewhstart;
            }
        }
        public EventWaitHandle EWHStop
        {
            get
            {
                return _ewhstop;
            }
        }
        #endregion
        
        public event PropertyChangedEventHandler PropertyChanged;
        public string LastError
        {
            get { return _lasterror; }
            set
            {
                _lasterror = value;
                UpdateProperties();
            }
        }
        public SensorSettings Settings
        {
            get { return _sensorSettings; }
            set { _sensorSettings = value; }
        }
        public bool Online
        {
            get
            {
                return _online;
            }
            set
            {
                _online = value;
            }
        }
        private SensorSettings GetSettingsByIndex(uint id)
        {
            foreach (var SensorProperties in Properties.Settings.Default.Sensors)
            {
                if (SensorProperties.Id == id)
                {
                    return SensorProperties;
                }
            }
            return null;
        }
        internal SensorVM(GoSensor sensor)
        {
            _sensor = sensor;
            _ewhstart = Utilities.GetNamedEventHandle(_sensor.Id, "Start", false);
            _ewhstop = Utilities.GetNamedEventHandle(_sensor.Id, "Stop", false);
            _ewhlogoff = Utilities.GetNamedEventHandle(_sensor.Id, "Logoff", false);

            // Initialize commands required by the view
            _launchUrlCommand = new CommandActionAsync(() => LaunchUrl(), true);
        }
        public void SetStop()
        {
            _ewhstop.Set();
        }

        /// <summary>
        /// This is for visibility binding in XAML. For a hidden component, we 
        /// can display it as soon as the view model gets created.
        /// </summary>
        public bool Exists
        {
            get { return true; }
        }

        /// <summary>
        /// Handles setting the accelerated state, as well as updating the caption (used by the view).
        /// </summary>
        private void ToggleAcceleration()
        {
            bool enable = !AccelerationEnabled;

            if (enable) { ToggleAccelerationCommandCaption = "Initializing..."; }
            else { ToggleAccelerationCommandCaption = "Stopping..."; }

            AccelerationEnabled = enable;

            if (AccelerationEnabled) { ToggleAccelerationCommandCaption = "Stop"; }
            else { ToggleAccelerationCommandCaption = "Start"; }
        }

        /// <summary>
        /// Launches the default browser/handler for the sensor (or accelerator) URL.
        /// </summary>
        private void LaunchUrl()
        {
            Process.Start(Url.ToString());
        }

        private readonly ICommand _launchUrlCommand;
        public ICommand LaunchUrlCommand
        {
            get { return _launchUrlCommand; }
        }

        private string _toggleAccelerationCommandCaption = "Start";
        public string ToggleAccelerationCommandCaption
        {
            get { return _toggleAccelerationCommandCaption; }
            set
            {
                _toggleAccelerationCommandCaption = value;

                if (PropertyChanged != null) { PropertyChanged(this, new PropertyChangedEventArgs("ToggleAccelerationCommandCaption")); }
            }
        }

        public uint Id
        {
            get
            {
                return _sensorSettings.Id;
            }
        }

        public string Ip
        {
            get
            {
                if (AccelerationEnabled)
                {
                    return SelectedIp == "Any" ? "localhost" : SelectedIp;
                }
                else
                {
                    return _sensor.Address().Address.ToString();
                }
            }
        }


        public uint WebPort
        {
            get
            {

                return _sensorSettings.WebPort;
            }
            set
            {
                GetSettingsByIndex(_sensorSettings.Id).WebPort = value;
                //Properties.Settings.Default.Sensors[0].WebPort = value;
                if (PropertyChanged != null) { PropertyChanged(this, new PropertyChangedEventArgs("WebPort")); }
                UpdateProperties();
            }
        }

        public uint BasePort
        {
            get { return _sensorSettings.BasePort; }
            set
            {
                //make ports sequential from base
                GetSettingsByIndex(_sensorSettings.Id).BasePort = value;
                if (PropertyChanged != null) { PropertyChanged(this, new PropertyChangedEventArgs("BasePort")); }
                UpdateProperties();
            }
        }

        public uint ControlPort
        {
            get { return GetSettingsByIndex(_sensorSettings.Id).BasePort; }
        }

        public uint HealthPort
        {
            get { return GetSettingsByIndex(_sensorSettings.Id).BasePort + 4; }
        }

        public uint PrivateDataPort
        {
            get { return GetSettingsByIndex(_sensorSettings.Id).BasePort + 5; }
        }

        public uint PublicDataPort
        {
            get { return GetSettingsByIndex(_sensorSettings.Id).BasePort + 6; }
        }
        public uint UpgradePort
        {
            get { return GetSettingsByIndex(_sensorSettings.Id).BasePort + 2; }
        }

        public Uri Url
        {
            get
            {
                if (!AccelerationEnabled)
                {
                    return new Uri("http://" + Ip + "/");
                }
                else
                {
                    return new Uri("http://" + Ip + ":" + WebPort); // + "/?control=" + ControlPort + "&health=" + HealthPort + "&data=" + PrivateDataPort );
                }
            }
        }

        public string FirmwareVersion => $"{_sensor.FirmwareVersion.Major}.{_sensor.FirmwareVersion.Minor}.{_sensor.FirmwareVersion.Release}.{_sensor.FirmwareVersion.Build}";

        public string FirmwareMainVersion => $"{_sensor.FirmwareVersion.Major}.{_sensor.FirmwareVersion.Minor}";

        private GoAcceleratorConnectionStatus _accelerationStatus = GoAcceleratorConnectionStatus.Connected;
        private string AccelerationStatusString
        {
            get
            {
                if (_accelerationStatus == GoAcceleratorConnectionStatus.Connected)
                {
                    return "Connected";
                }
                else if (_accelerationStatus == GoAcceleratorConnectionStatus.Disconnected)
                {
                    return "Disconnected";
                }
                else if (_accelerationStatus == GoAcceleratorConnectionStatus.Error)
                {
                    return "Error";
                }
                else
                {
                    return "Unknown";
                }
            }
        }

        public List<string> IpSelectionList
        {
            get
            {
                List<string> ips = new List<string>();

                ips.Add("Any");
                ips.AddRange(Engine.GetPossibleIpAddresses());

                return ips;
            }
        }

        public string SelectedIp
        {
            get { return _sensorSettings.AcceleratedSensorIp; }
            set
            {
                GetSettingsByIndex(_sensorSettings.Id).AcceleratedSensorIp = value;
                UpdateProperties();
            }
        }

        public GoState State { get { return _sensor.State; } }

        public GoDiscoveryOpMode OpMode { get { return _sensor.AccelerationOpMode;  } }

        public string StatusText
        {
            get
            {
                if (AcceleratedState == GoAcceleratorEngine.Properties.AccelerationState.Stopping)
                {
                    return "Stopping Acceleration";
                }
                else if (AcceleratedState == GoAcceleratorEngine.Properties.AccelerationState.Starting)
                {
                    return "Starting Acceleration";
                }
                else if (AccelerationEnabled)
                {
                    // not really the best place for this... (GoState != GoStates.isAccelerator)
                    return "Accelerated";// + AccelerationStatusString; 
                }

                switch (State)
                {
                    case GoState.Busy: return "Busy " + _lasterror;
                    case GoState.Cancelled: return "Cancelled " + _lasterror;
                    case GoState.Incompatible: return "Incompatible " + _lasterror;
                    case GoState.Incomplete: return "Incomplete " + _lasterror;
                    case GoState.Inconsistent: return "Inconsistent " + _lasterror;
                    case GoState.Offline: return "Offline " + _lasterror;
                    case GoState.Online:
                        // Indicate if accelerated by other host.
                        if (OpMode == GoDiscoveryOpMode.ACCELERATOR)
                        {
                            return "Accelerated by Other Host " + _lasterror;
                        }
                        else
                        {
                            return "Online " + _lasterror;
                        }
                    case GoState.Ready: return "Ready " + _lasterror;
                    case GoState.Resetting: return "Resetting " + _lasterror;
                    case GoState.Running: return "Running " + _lasterror;
                    case GoState.Unresponsive: return "Unresponsive " + _lasterror;
                    case GoState.Upgrading: return "Upgrading " + _lasterror;

                    default: return "Unknown";
                }
            }
        }
        /// <summary>
        /// Enables/disables acceleration.
        /// </summary>
        public bool AccelerationEnabled
        {
            get { return _sensorSettings.Accelerated; }
            set
            {
                GetSettingsByIndex(_sensorSettings.Id).Accelerated = value;
                UpdateProperties();
                if (value)
                {
                    cancelAutoRefreshTimer();
                }
                else
                {
                    startAutoRefreshTimer();
                }
            }
        }
        /// <summary>
        /// Gets the actual acceleration state reported by the underlying GoSensor. 
        /// </summary>
        /// <param name="isOwned">Whether the sensor is being accelerated by this host.</param>
        public void RefreshAcceleration(bool isOwned)
        {
            bool actualAccelerated = false;
            switch (_sensor.AccelerationState)
            {
                case GoSensorAccelState.STATE_ACCELERATED:
                    // See GOC-12562 and GOC-11423.
                    // NOTE: Although GoSensorAccelState.STATE_ACCELERATED_BY_OTHER is defined by the SDK,
                    // it is only handled at the application level -- ie. by GoAccelerator\GoAcceleratorEngine here,
                    // or by GoMax\GxAccelerator.  Neither GoSdk\GoSensor, nor GoSdk\GoAcceleratorMgr currently
                    // handles the _ACCELERATED_BY_OTHER state.
                    //
                    // Hence, we need to check whether we are the accelerator host and set
                    // the accelerated state accordingly ourselves.
                    actualAccelerated = isOwned;
                    break;
                case GoSensorAccelState.STATE_AVAILABLE:
                case GoSensorAccelState.STATE_FW_MISMATCH:
                case GoSensorAccelState.STATE_UNKNOWN:
                default:
                    actualAccelerated = false;
                    break;
            }

            if (this.AccelerationEnabled != actualAccelerated)
            {
                Utilities.Log(string.Format("current Accel value({0}) != actual Accel value({1})", 
                    this.AccelerationEnabled, actualAccelerated));
                this.AccelerationEnabled = actualAccelerated;
                this.AcceleratedState = this.AccelerationEnabled ?
                    AccelerationState.Started : AccelerationState.Stopped;
            }
        }

        public AccelerationState AcceleratedState
        {
            get { return _sensorSettings.AcceleratedState; }
            set
            {
                GetSettingsByIndex(_sensorSettings.Id).AcceleratedState = value;
                if (PropertyChanged != null)
                {
                    PropertyChanged(this, new PropertyChangedEventArgs("AcceleratedState"));
                }
            }
        }

        public void UpdateProperties()
        {
            Properties.Settings.Default.Save();
            if (PropertyChanged != null)
            {
                // Let the UI know to refresh any relevant properties
                PropertyChanged(this, new PropertyChangedEventArgs("AccelerationEnabled"));
                PropertyChanged(this, new PropertyChangedEventArgs("Ip"));
                PropertyChanged(this, new PropertyChangedEventArgs("WebPort"));
                PropertyChanged(this, new PropertyChangedEventArgs("Url"));
                PropertyChanged(this, new PropertyChangedEventArgs("BasePort"));
            }
        }

        private void AcceleratorUpdateHandler(GoAcceleratorConnectionStatus status)
        {
            _accelerationStatus = status;
            UpdateProperties();
        }
        private EventHandler onExited = null;
        public void Start(ProcessStartInfo startInfo, WaitOrTimerCallback startCallback, WaitOrTimerCallback logoffCallback, EventHandler exitedHandler)
        {
            _lasterror = "";

            if (null != startCallback)  // the manual startup case
            {
                // The eventWaitHandle should auto-reset.
                // The reason it was triggering falsely in GOC-12858 was because we called RegisteredWaitHandle.Unregister(WaitHandle waitObject) which
                // according to https://docs.microsoft.com/en-us/dotnet/api/system.threading.registeredwaithandle.unregister?view=netframework-4.8
                // actually additionally signals waitObject.
                _regewhstart = ThreadPool.RegisterWaitForSingleObject(_ewhstart, startCallback, this, StartVmTimeout, true);
            }
            onExited = exitedHandler;
            ThreadPool.RegisterWaitForSingleObject(_ewhlogoff, logoffCallback, this, -1, true);
            process = Process.Start(startInfo);
            process.Exited += onExited;
            process.EnableRaisingEvents = true;
        }
        private WaitHandle exitHandle = null;
        public void AddOnExitedHandler(WaitHandle handle)
        {
            exitHandle = handle;
            if (null != process) process.Exited += OnChildExited;
        }
        public void RemoveOnExitedHandler() { if (null != process) process.Exited -= OnChildExited; }
        private void OnChildExited(object sender, EventArgs e)
        {
            if (null != exitHandle) (exitHandle as ManualResetEvent).Set();
        }
        public void Stop(int timeout)
        {
            Stopwatch sw = Stopwatch.StartNew();

            if (!HasExited)
            {
                process.Exited -= onExited;

                SetStop();

                int pid = process.Id;

                //if timeouts
                if (!process.WaitForExit(timeout))
                {
                    Utilities.Log(string.Format("killing process id {0}.", pid));
                    //kill process
                    process.Kill();
                }
                else
                {
                    Utilities.Log(string.Format("process id {0} exited.", pid));
                }

                // Wait until the mode no longer indicates accelerated 
                while (_sensor.AccelerationOpMode == GoDiscoveryOpMode.ACCELERATOR &&
                        sw.ElapsedMilliseconds < timeout)
                {
                    Thread.Sleep(timeout/10);
                }
            }
        }
        public bool HasExited { get { return process == null || process.HasExited; } }
        public int Pid { get { return null != process ? process.Id : -1; } }    
        
        private void startAutoRefreshTimer()
        {
            cancelAutoRefreshTimer();
            autoRefreshTimer = new System.Threading.Timer(new TimerCallback(autoRefreshTimerExpiryHandler),
                                                          null,
                                                          autoRefreshTimerDelay,
                                                          autoRefreshTimerDelay);
        }

        private void autoRefreshTimerExpiryHandler(object state)
        {
            autoRefreshTimerCount++;
            UpdateProperties();
            if (autoRefreshTimerCount >= autoRefreshTimerCountMax)
            {
                cancelAutoRefreshTimer();
            }
        }

        private void cancelAutoRefreshTimer()
        {
            autoRefreshTimerCount = 0;
            if (autoRefreshTimer != null)
            {
                autoRefreshTimer.Dispose();
            }
        }
    }
}

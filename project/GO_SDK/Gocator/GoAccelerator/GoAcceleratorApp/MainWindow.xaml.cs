using GoAcceleratorEngine;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Input;
using System.Threading;
using System.Diagnostics;
using System.ComponentModel;
using System.IO.Pipes;
using System.IO;
using System.Windows.Threading;
using Microsoft.Win32;
using System.Net.NetworkInformation;
using System.Net;

namespace GoAccelerator
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        private Dispatcher mainThreadDispatcher;
        private SettingsWindow _settingsWindow;
        private readonly ICommand _showWindowCommand;

        public delegate void IsBusyDelegate(bool isBusy);
        public const int ControlPort = 3190;
        public const int MaxControlPort = 5000;
        public const int BaseWebPort = 8080;
        public const int MaxWebPort = 9090;
        private bool _stopping = false;
        private bool _appExiting = false;
        public const int ErrorVersion = -989;
        private bool _showchildwindow = false;
        private int maxAutostartRetries = 3; // max auto-start retry attempts
        private int startupDelay = 1; // auto-startup delay in minutes
        private bool startupDelayed = false;
        private bool loading = false;
        private SessionEndingEventHandler logoffHandler;
        private const string regGoXKey = @"Software\LMI Technologies\GoAccelerator\";
        private const int failedRetryDelay = 1; // auto-start retry delay in minutes
        private readonly string[] keyNames = new string[] { "StartupDelay", "MaxAutoStartRetry" };
        private Dictionary<uint, NamedPipeServerStream> pipes = new Dictionary<uint, NamedPipeServerStream>();

        List<int> _unusedPorts;
        List<int> _unusedWebports;

        public MainWindow()
        {
            Title = Properties.Resources.ProductName;
            loading = true;
            mainThreadDispatcher = Dispatcher.CurrentDispatcher;

            KillVMProcceses();
            logoffHandler = new SessionEndingEventHandler(this.OnLogoff);

            _settingsWindow = new SettingsWindow();

            // must be initialized before anything tries to bind to ShowWindowCommand
            SystemEvents.SessionEnding += logoffHandler;
            _showWindowCommand = new CommandAction(() => { Show(); Activate(); }, true);
            InitializeComponent();

            Utilities.RegisterRegistrySource();
            ParseCommandline();
            ReadRegistry();

            // GOC-14117 - Auto start previously accelerated sensor as soon as the application is started (without waiting for mainWindowLoad event).
            StartLastAcceleratedSensors();
            (DataContext as SensorsVM).PropertyChanged += Vm_PropertyChanged;
            GetOpenPort();

            // Perform a refresh whenever the window is shown.
            SensorsVM vm = (SensorsVM)DataContext;
            vm.UpdateCommand.Execute(null);

            loading = false;
        }
        /// <summary>
        /// logoff/shutdown event handler when OS is going down or current user logged off
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="args"></param>
        private void OnLogoff(object sender, SessionEndingEventArgs args)
        {
            ExitApp();
            return;
        }
        /// <summary>
        /// parses the command line for arguments if to show child processes or delay startup
        /// </summary>
        private void ParseCommandline()
        {
            String[] arguments = Environment.GetCommandLineArgs();
            for (var i = 0; i < arguments.Length; i++)
            {
                var arg = arguments[i];
                if (arg == "-s" || arg == "--show-children")
                {
                    _showchildwindow = true;
                }
                else if (arg == "-d" || arg == "--delayed-start")
                {
                    startupDelayed = true;
                }
            }
        }
        /// <summary>
        /// read startup-delay and max-autostart-retry parameters from registry
        /// </summary>
        private void ReadRegistry()
        {
            try
            {
                RegistryKey key = Registry.CurrentUser.OpenSubKey(regGoXKey, true);
                if (null == key)
                {
                    key = Registry.CurrentUser.CreateSubKey(regGoXKey);
                }
                if (null == key) return;

                var subkeys = key.GetValueNames();
                if (0<subkeys.Length && subkeys.Contains(keyNames[0]))
                {
                    startupDelay = (int)key.GetValue(keyNames[0]);
                }
                else
                {
                    key.SetValue(keyNames[0], startupDelay);
                }
                if (0 < subkeys.Length && subkeys.Contains(keyNames[1]))
                {
                    maxAutostartRetries = (int)key.GetValue(keyNames[1]);
                }
                else
                {
                    key.SetValue(keyNames[1], maxAutostartRetries);
                }
            } 
            catch (Exception) { }
        }
        /// <summary>
        /// (un)lock spinners, set sensor online state, 
        /// enable START button, set its text and update UI sensor status
        /// </summary>
        /// <param name="sensor"></param>
        private void SetSensorStatus(SensorVM sensor)
        {
            if (sensor.AccelerationEnabled)
            {
                BlockSpinners(true);
            }
            else
            {
                BlockSpinners(false);
                // Disable the button if the sensor is accelerated by another host.
                sensor.Online = (sensor.State != Lmi3d.GoSdk.GoState.Offline &&
                                 sensor.OpMode == Lmi3d.GoSdk.GoDiscoveryOpMode.STANDALONE);
            }
            btnStart.IsEnabled = sensor.Online;
            UpdateStartBtnAndSensorStatus(sensor);
        }

        #region UI related methods
        /// <summary>
        /// Only allow closing the application from the tray icon.
        /// </summary>
        private void Window_Closing(object sender, System.ComponentModel.CancelEventArgs e)
        {
            e.Cancel = true;
            Hide();
        }
        /// <summary>
        /// property-changed callback function to change selected sensor
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void Vm_PropertyChanged(object sender, PropertyChangedEventArgs e)
        {
            SensorsVM vm = (SensorsVM)DataContext;

            if (vm.SelectedSensor == null) return;

            if (e.PropertyName == "SelectedSensor")
            {
                SetSensorStatus(vm.SelectedSensor);
            }
        }

        /// <summary>
        /// Handler for the tray context menu.
        /// </summary>
        private void MenuItem_Click(object sender, RoutedEventArgs e)
        {
            MenuItem item = e.OriginalSource as MenuItem;
            switch (item.Name)
            {
                case "SensorManager":
                    // Show the main window
                    Show();
                    Activate();
                    break;

                case "Settings":
                    // Show the settings window
                    _settingsWindow.Show();
                    _settingsWindow.Activate();
                    break;

                case "Exit":
                    ExitApp();
                    break;
            }
        }
        private void ExitApp()
        {
            _appExiting = true;
            SystemEvents.SessionEnding -= logoffHandler;
            (DataContext as SensorsVM).PropertyChanged -= Vm_PropertyChanged;

            StopAllVMProcesses();
            GoAcceleratorEngine.Properties.Settings.Default.Save();
            Application.Current.Shutdown();
        }

        /// <summary>
        /// START button callback function to start/stop acceleration of given sensor
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void btnStart_Click(object sender, RoutedEventArgs e)
        {
            SensorsVM vm = (SensorsVM)DataContext;
            if (vm.SelectedSensor == null) return;

            if (!vm.SelectedSensor.AccelerationEnabled)
            {
                Utilities.Log("User accelerating sensor " + vm.SelectedSensor.Id + "...");
                StartVMProcess(vm.SelectedSensor,
                    vm.SelectedSensor.WebPort,
                    vm.SelectedSensor.BasePort,
                    vm.SelectedSensor.HealthPort,
                    vm.SelectedSensor.UpgradePort,
                    vm.SelectedSensor.PrivateDataPort,
                    vm.SelectedSensor.PublicDataPort,
                    vm.SelectedSensor.SelectedIp,
                    true); //bWaitThread=true for user-invoked accelerations
            }
            else
            {
                Utilities.Log("User decelerating sensor " + vm.SelectedSensor.Id + "...");
                vm.SelectedSensor.AcceleratedState = GoAcceleratorEngine.Properties.AccelerationState.Stopping;
                UpdateStartBtnAndSensorStatus(vm.SelectedSensor);
                BlockUI(true);
                BlockSpinners(true);
                BlockResetButton(true);
                StopSelectedSensor();
            }
        }
        /// <summary>
        /// update START button, sensor's status and 'reset port' button on UI
        /// </summary>
        /// <param name="sensor">
        /// <para>Update for given <see cref="SensorVM"/>; </para>
        /// <para>otherwise use currently selected sensor if null.</para></param>
        protected void UpdateStartBtnAndSensorStatus(SensorVM sensor)
        {
            if (null == sensor &&
                null == (sensor = (DataContext as SensorsVM).SelectedSensor)) return;

            txtStatus.Text = sensor.StatusText;
            if (sensor.AcceleratedState == GoAcceleratorEngine.Properties.AccelerationState.Stopped)
            {
                btnStart.Content = "Start";
                BlockResetButton(false);
            }
            else if (sensor.AcceleratedState == GoAcceleratorEngine.Properties.AccelerationState.Started)
            {
                btnStart.Content = "Stop";
                BlockResetButton(true);
            }
            else
            {
                btnStart.Content = sensor.AcceleratedState.ToString();
                BlockResetButton(false);
            }
        }
        private void btnReset_Click(object sender, RoutedEventArgs e)
        {
            SensorsVM vm = (SensorsVM)DataContext;
            if (vm.SelectedSensor == null) return;
            vm.SelectedSensor.BasePort = 3190;
        }
        /// <summary>
        /// (un)block start button and sensor list
        /// </summary>
        private void BlockUI(bool block)
        {
            btnStart.IsEnabled = lstSensors.IsEnabled = !block;
        }

        /// <summary>
        /// (un)block 'reset port' button
        /// </summary>
        private void BlockResetButton(bool block)
        {
            btnReset.IsEnabled = !block;
        }

        /// <summary>
        /// (un)block spinners
        /// </summary>
        private void BlockSpinners(bool block)
        {
            spinWebport.IsEnabled = spinBasePort.IsEnabled = cmbIP.IsEnabled = !block;
        }
        /// <summary>
        /// Command to show the main window. Allows the UI to bind to this functionality.
        /// used by context menu in the windows status bar
        /// </summary>
        public ICommand ShowWindowCommand
        {
            get { return _showWindowCommand; }
        }
        #endregion  

        #region starting VM senosr processes
        /// <summary>
        /// called on the startup to auto-accelerate previously accelerated sensors
        /// </summary>
        private void StartLastAcceleratedSensors()
        {
            if (startupDelayed)
            {
                Utilities.Log("Startup delay in mins: " + startupDelay);
                Thread.Sleep(Utilities.Min2Ms(startupDelay));
            }
            SensorsVM vm = (SensorsVM)DataContext;
            if (vm.SelectedSensor == null) return;
            foreach (var sensor in vm.Sensors)
            {
                if (sensor.AccelerationEnabled && GoAcceleratorEngine.Properties.Settings.Default.AccelerateOnStartup)
                {
                    int retry = 0;
                    bool started = false;
                    Utilities.Log("Auto-accelerating sensor " + sensor.Id + "...");
                    while (!started && retry < maxAutostartRetries)
                    {
                        StartVMProcess(sensor, sensor.WebPort,
                            sensor.BasePort,
                            sensor.HealthPort,
                            sensor.UpgradePort,
                            sensor.PrivateDataPort,
                            sensor.PublicDataPort,
                            sensor.SelectedIp,
                            false);

                        WaitHandle[] events = new WaitHandle[] { sensor.EWHStart, sensor.EWHStop, new ManualResetEvent(false) };
                        sensor.AddOnExitedHandler(events[2]);

                        var index = WaitHandle.WaitAny(events, SensorVM.StartVmTimeout);
                        if (0 == index)
                        {
                            started = true;
                            SensorStarted(sensor);
                        }
                        else
                        {
                            retry++;
                            sensor.AccelerationEnabled = false;
                            if (WaitHandle.WaitTimeout != index && sensor.HasExited)
                            {
                                Utilities.Log(string.Format("Process of sensor {0} terminated.", sensor.Id), EventLogEntryType.Warning);
                            }
                            else if (WaitHandle.WaitTimeout == index)
                            {
                                Utilities.Log(string.Format("Sensor {0} start timeout.", sensor.Id), EventLogEntryType.Error);
                            }
                            Thread.Sleep(Utilities.Min2Ms(failedRetryDelay) * retry);
                        }
                        sensor.RemoveOnExitedHandler();
                        if (!started) Utilities.Log(string.Format("Sensor {0}: {1} start attempt", sensor.Id, retry));
                    }
                }
                else
                {
                    sensor.AccelerationEnabled = false;
                    Utilities.Log(string.Format("Sensor {0}: {1}, {2}, {3}", sensor.Id, sensor.AccelerationEnabled, sensor.AcceleratedState, sensor.StatusText));
                }
            }

            SetSensorStatus(vm.SelectedSensor);

            BlockUI(false);
            if (lstSensors.Items.Count > 0)
            {
                lstSensors.SelectedIndex = 0;
            }
        }
        /// <summary>
        /// called when sensor's acceleration process is completed
        /// </summary>
        /// <param name="sensor">sensor that was just accelerated</param>
        private void SensorStarted(SensorVM sensor)
        {
            if (null != sensor)
            {
                sensor.AccelerationEnabled = true;
                sensor.AcceleratedState = GoAcceleratorEngine.Properties.AccelerationState.Started;
                (DataContext as SensorsVM).AddAcceleratedID(sensor.Id);
                Utilities.Log(string.Format("Sensor {0} accelerated.", sensor.Id));
            }
        }
        /// <summary>
        /// starting a child vm acceleration process for a given sensor
        /// </summary>
        /// <param name="sensor">sensor object to be accelerated</param>
        /// <param name="WebPort">web port used for the accelerated sensor</param>
        /// <param name="BasePort">base port used for the accelerated sensor</param>
        /// <param name="HealthPort">health port used for the accelerated sensor</param>
        /// <param name="UpgradePort">upgrade port used for the accelerated sensor</param>
        /// <param name="PrivateDataPort">private data port used for the accelerated sensor</param>
        /// <param name="PublicDataPort">public data port used for the accelerated sensor</param>
        /// <param name="ipAddress">ip addess used for the accelerated sensor</param>
        /// <param name="bWaitThread">true to register a callback for start event to indicate when the acceleration process is completed, used when accelerating by clickin gon the start button</param>
        /// <returns></returns>
        public bool StartVMProcess(SensorVM sensor, uint WebPort, uint BasePort, uint HealthPort, uint UpgradePort, uint PrivateDataPort, uint PublicDataPort, string ipAddress, bool bWaitThread)
        {
            if (_appExiting) return false;

            uint id = Convert.ToUInt32(sensor.Id);
            uint[] ports = new uint[] { BasePort, HealthPort, UpgradePort, PrivateDataPort, PublicDataPort };
            uint portsMin = ports.Min();     // aids in searching
            uint portsMax = ports.Max();     // aids in searching
            IPAddress searchAddr = null;

            // found conflict flags
            bool foundConflict = false;
            uint conflictPort = 0;
            IPAddress conflictAddress = IPAddress.Any;
            string conflictProcessName = "";

            // GOC-13329 If FW versions differ in sensor and SDK, acceleration is not allowed.
            #if DEBUG
                // Checking only major and minor numbers for debug build.
                if (sensor.FirmwareMainVersion != Engine.GetFirmwareMainVersion())
            #else
                if (sensor.FirmwareVersion != Engine.GetFirmwareVersion())
            #endif
            {
                MessageBox.Show($"Sensor firmware version ({sensor.FirmwareVersion}) does not match the accelerator version ({Engine.GetFirmwareVersion()}).", 
                    "Error", MessageBoxButton.OK, MessageBoxImage.Error);
                return false;
            }

            // Set the searchAddr
            try
            {
                searchAddr = (ipAddress == "Any" ? IPAddress.Any : IPAddress.Parse(ipAddress));
            }
            catch (Exception e)
            {
                var msg = "Invalid IP address " + ipAddress + ": " + e.Message;
                MessageBox.Show(msg, "Error");
                Utilities.Log(msg);
                return false;
            }

            // NOTE: Make one pass through the IPHelper's returned TCP-PID mappings, 
            // checking for all webport and baseport clashes along the way 
            // (improvement over prior behavior that made multiple passes).
            if (searchAddr.AddressFamily == System.Net.Sockets.AddressFamily.InterNetwork)
            {
                // Call IPV4 version
                List<IPHelper.MIB_TCPROW_OWNER_PID> tcpConns = IPHelper.GetAllTCPConnections();
                foreach (IPHelper.MIB_TCPROW_OWNER_PID tcpConn in tcpConns)
                {
                    // Check for webport OR baseport clash, AND then IP (considering "any")
                    if ((tcpConn.LocalPort.Equals((ushort)WebPort) ||
                        (tcpConn.LocalPort >= portsMin && tcpConn.LocalPort <= portsMax && ports.Contains(tcpConn.LocalPort))) &&
                        (searchAddr.Equals(IPAddress.Any) || tcpConn.LocalAddress.Equals(IPAddress.Any) || tcpConn.LocalAddress.Equals(searchAddr)))
                    {
                        // Error if webport is used
                        foundConflict = true;
                        conflictPort = tcpConn.LocalPort;
                        conflictAddress = tcpConn.LocalAddress;
                        conflictProcessName = Process.GetProcessById((int)tcpConn.ProcessId).ProcessName;
                        break;
                    }
                }
            }
            else if (searchAddr.AddressFamily == System.Net.Sockets.AddressFamily.InterNetworkV6)
            {
                // Call IPV6 version (same except for table/row types)
                List<IPHelper.MIB_TCP6ROW_OWNER_PID> tcpConns = IPHelper.GetAllTCPv6Connections();
                foreach (IPHelper.MIB_TCP6ROW_OWNER_PID tcpConn in tcpConns)
                {
                    // Check for webport OR baseport clash, AND then IP (considering "any")
                    if ((tcpConn.LocalPort.Equals((ushort)WebPort) ||
                        (tcpConn.LocalPort >= portsMin && tcpConn.LocalPort <= portsMax && ports.Contains(tcpConn.LocalPort))) &&
                        (searchAddr.Equals(IPAddress.Any) || tcpConn.LocalAddress.Equals(IPAddress.Any) || tcpConn.LocalAddress.Equals(searchAddr)))
                    {
                        foundConflict = true;
                        conflictPort = tcpConn.LocalPort;
                        conflictAddress = tcpConn.LocalAddress;
                        conflictProcessName = Process.GetProcessById((int)tcpConn.ProcessId).ProcessName;
                        break;
                    }
                }
            }
            else
            {
                var msg = "Unknown Address Family " + searchAddr.AddressFamily.ToString();
                MessageBox.Show(msg, "Error");
                Utilities.Log(msg);
                return false;
            }

            // Error if address/port was found to be used...
            if (foundConflict)
            {
                string msg;
                if (conflictPort == WebPort)
                {
                    msg = "Webport conflict for sensor id " + id.ToString() + ", IP " + ipAddress + ", Webport = " + WebPort.ToString() + ",";
                }
                else
                {
                    msg = "Baseport conflict for sensor id " + id.ToString() + ", IP " + ipAddress + ", BasePort = " + BasePort.ToString() + ",";
                }
                msg += (Environment.NewLine + conflictProcessName + " (" +
                        conflictAddress.ToString() + ":" + conflictPort.ToString() + ") is already using the port.");
                MessageBox.Show(msg, "Error");
                Utilities.Log(msg);
                return false;
            }

            sensor.AcceleratedState = GoAcceleratorEngine.Properties.AccelerationState.Starting;
            UpdateStartBtnAndSensorStatus(sensor);
            BlockUI(true);
            BlockSpinners(true);
            BlockResetButton(true);

            var process = Process.GetCurrentProcess();
            ProcessStartInfo startInfo = new ProcessStartInfo();
            string path = System.IO.Path.GetDirectoryName(process.MainModule.FileName);
            startInfo.FileName = path + @"\GoXVMProcess.exe";
            startInfo.WorkingDirectory = System.IO.Path.GetDirectoryName(process.MainModule.FileName);

            if (ipAddress == "Any")
            {
                ipAddress = "0.0.0.0";
            }

            startInfo.Arguments = "--serialnumber " + id.ToString()
                + " --webport " + WebPort.ToString()
                + " --baseport " + BasePort.ToString()
                + " --healthport " + HealthPort.ToString()
                + " --upgradeport " + UpgradePort.ToString()
                + " --privatedataport " + PrivateDataPort.ToString()
                + " --publicdataport " + PublicDataPort.ToString()
                + " --address " + ipAddress
                + " --ignorestopevent";

            startInfo.CreateNoWindow = _showchildwindow;
            startInfo.WindowStyle = _showchildwindow ? ProcessWindowStyle.Normal : ProcessWindowStyle.Hidden;

            sensor.Start(startInfo,
                (bWaitThread ? OnSensorStart : (WaitOrTimerCallback)null),
                OnSensorLogoff,
                Pid_Exited);

            Task.Run(() => { OnSensorError(sensor, bWaitThread); });
            
            return true;
        }
        #endregion

        private NamedPipeServerStream GetPipe(uint sid)
        {
            NamedPipeServerStream pipe = null;
            if (pipes.ContainsKey(sid) && null != (pipe = pipes[sid]))
            {
                bool closedPipe = false;
                try
                {
                    closedPipe = pipe.SafePipeHandle.IsClosed;
                }
                catch
                {
                    closedPipe = true;
                }
                finally
                {
                    if (closedPipe)
                    {
                        pipes[sid] = null;
                        pipes.Remove(sid);
                    }
                }
            }
            if (!pipes.ContainsKey(sid) || null == pipes[sid])
            {
                pipes[sid] = (NamedPipeServerStream)Utilities.OpenPipe(sid, false, PipeDirection.In);
            }
            return pipes[sid];
        }
        /// <summary>
        /// Close the pipe returned by <see cref="GetPipe"/>.
        /// </summary>
        /// <param name="sid">The sensor id key of the pipe.</param>
        private void ClosePipe(uint sid)
        {
            NamedPipeServerStream pipe = null;
            if (pipes.ContainsKey(sid) && null != (pipe = pipes[sid]))
            {
                try
                {
                    pipe.Close();
                }
                catch (Exception e)
                {
                    Utilities.Log(string.Format("Exception when closing pipe for sensor {0}: {1}",
                        sid, e.ToString()), EventLogEntryType.Error);
                }
                finally
                {
                    pipes[sid] = null;
                    pipes.Remove(sid);
                }
            }
        }
        #region child process callbacks
        /// <summary>
        /// callback handler when child accelerated process throws an error
        /// </summary>
        /// <param name="sensor">sensor that was accelerated</param>
        /// <param name="handleErrorViaDispatcher">directs if to handle error case via dispatcher or not. 
        /// this is a hack as acceleration start follows 2 different paths and then we need to accomodate this difference.
        /// ultimately the acceleration starting process shoudl be unified and handled the same way regardless it's started by user or automatically on startup</param>
        private async void OnSensorError(SensorVM sensor, bool handleErrorViaDispatcher)
        {
            if (_appExiting) return;
            
            bool unhandledEx = false;

            try
            {
                var errPipe = GetPipe(sensor.Id);
                bool connected = false;
                
                while (!sensor.HasExited)
                {
                    connected = false;
                    // Handle the case where the pipe is already connected
                    try
                    {
                        var asyncResult = errPipe.BeginWaitForConnection(null, null);
                        connected = asyncResult.AsyncWaitHandle.WaitOne(1000);
                        errPipe.EndWaitForConnection(asyncResult);
                    }
                    catch (System.InvalidOperationException ioe)
                    {
                        // see https://docs.microsoft.com/en-us/dotnet/api/system.io.pipes.namedpipeserverstream.waitforconnection
                        if (ioe.Message.Contains("Already in a connected state"))
                        {
                            connected = true;
                        }
                    }

                    if (connected)
                    {
                        using (StreamReader sr = new StreamReader(errPipe))
                        {
                            ErrorInfo errMsg = new ErrorInfo(await sr.ReadToEndAsync());

                            Func<int> handleError = () => {
                                Utilities.Log("Sensor error: " + errMsg.ToString(), EventLogEntryType.Error);
                                if (errMsg.ErrorNumber == ErrorVersion)
                                {
                                    sensor.LastError = "- Error Version Mismatch";
                                }
                                else if (!string.IsNullOrEmpty(errMsg.ErrorMsg))
                                {
                                    sensor.LastError = "- Error";
                                    if (!loading)
                                    {
                                        MessageBox.Show(errMsg.ErrorMsg, "Error starting remote process", MessageBoxButton.OK, MessageBoxImage.Error);
                                    }
                                }
                                if (!loading || sensor.AcceleratedState != GoAcceleratorEngine.Properties.AccelerationState.Starting)
                                {
                                    SensorStopped(sensor);
                                }
                                return 1;
                            };
                            
                            // for manual-start
                            if (handleErrorViaDispatcher)
                            {
                              mainThreadDispatcher.Invoke(new Action(() =>
                              {
                                  handleError();
                                  UpdateStartBtnAndSensorStatus(sensor);
                                  BlockSpinners(false);
                                  BlockUI(false);
                              }));
                            }
                            // for auto-start, see StartLastAccleratedSensors()
                            else handleError();
                        }
                        break;
                    }
                }
            }
            catch (Exception e) 
            { 
                Utilities.Log(e.ToString() + e.Message, EventLogEntryType.Error);
                unhandledEx = true;
            }
            finally
            {
                // for manual-start
                if (handleErrorViaDispatcher)
                {
                    mainThreadDispatcher.Invoke(new Action(() =>
                    {
                        if (unhandledEx)
                        {
                            MessageBox.Show("Could not connect to pipe: (see event log for details)", "Error starting remote process",
                                MessageBoxButton.OK, MessageBoxImage.Error);
                            sensor.Stop(SensorVM.AbortVmTimeout);  // quickly abort and kill the GoXVMProcess.exe
                            // release the UI from acceleration attempt
                            SensorStopped(sensor);
                            UpdateStartBtnAndSensorStatus(sensor);
                            BlockSpinners(false);
                            BlockUI(false);
                        }
                    }));
                }
                // for auto-start, see StartLastAccleratedSensors()
                if (!handleErrorViaDispatcher)
                {
                    mainThreadDispatcher.Invoke(new Action(() =>
                    {
                        //TODO: leave for now, but StartLastAccleratedSensors() may eventually want
                        // OnSensorError to handle the killing of the process more quickly if unhandledEx flag is set.
                        UpdateStartBtnAndSensorStatus(sensor);
                        BlockSpinners(false);
                        BlockUI(false);
                    }));
                }
            }
            // Close the pipe, don't leave it hanging around
            ClosePipe(sensor.Id);
        }
        private void OnSensorStart(object state, bool timedOut)
        {
            if (_appExiting) return;

            // Handle the timed out case
            if (timedOut)
            {
                mainThreadDispatcher.Invoke(new Action(() =>
                {
                    SensorVM sensor = (SensorVM)state;
                    Utilities.Log(string.Format("Attempt to start sensor {0} timed out.", sensor.Id));
                    if (sensor.AcceleratedState == GoAcceleratorEngine.Properties.AccelerationState.Starting)
                    {
                        SensorStopped(sensor);
                    }
                    BlockUI(false);
                }));
            }
            else
            {
                mainThreadDispatcher.Invoke(new Action(() =>
                {
                    SensorVM sensor = (SensorVM)state;
                    if (sensor.AcceleratedState == GoAcceleratorEngine.Properties.AccelerationState.Starting)
                    {
                        SensorStarted(sensor);
                    }
                    UpdateStartBtnAndSensorStatus(sensor);
                    BlockUI(false);
                }));
            }
        }
        private void OnSensorLogoff(object state, bool timedOut)
        {
            // since all vm sensors run on the same machine, once we get logoff/shutdown event from any, that means gox is shuting down
            mainThreadDispatcher.Invoke(new Action(() => { _appExiting = true; }));
        }
        #endregion

        #region stopping/exiting sensor processes
        /// <summary>
        /// kill all sensor vm processes
        /// </summary>
        private void KillVMProcceses()
        {
            foreach (Process process in Process.GetProcessesByName("GoXVMProcess"))
            {
                process.Kill();
            }
        }
        /// <summary>
        /// callback when a child vm process exits
        /// ignoring when sensor's acceleration was stopped by a button, which is handled by button callback,
        /// also ignoring when OS is going down
        /// </summary>
        /// <param name="sender">exited process</param>
        /// <param name="ea"></param>
        private void Pid_Exited(object sender, EventArgs ea)
        {
            if (!_stopping && !_appExiting)
            {
                try
                {
                    Task.Run(() => { 
                        mainThreadDispatcher.Invoke(new Action(() =>
                        {
                            try
                            {
                                uint sid = 0;
                                int pid = (sender as Process).Id;
                                SensorsVM vm = (SensorsVM)DataContext;
                                foreach (var sensor in vm.Sensors)
                                {
                                    if (pid == sensor.Pid)
                                    {
                                        sid = sensor.Id;
                                        Utilities.Log(string.Format("Process accelerating sensor {0} unexpectedly exited.", sid), EventLogEntryType.Warning);
                                        if (sensor.AcceleratedState != GoAcceleratorEngine.Properties.AccelerationState.Stopping)
                                        {
                                            // sensor's acceleration stopped unexpectedly, not from accelerator
                                            if (sensor.AcceleratedState == GoAcceleratorEngine.Properties.AccelerationState.Starting)
                                            {
                                                // sensor acceleration process exited while starting
                                                //  set the event to release blocking wait function
                                                sensor.EWHStart.Set();
                                            }
                                        }
                                        SensorStopped(sensor);
                                        break;
                                    }
                                }

                                BlockUI(false);
                                BlockSpinners(false);
                                BlockResetButton(false);
                            }
                            catch (Exception e) { Utilities.Log(e.Message+e.StackTrace, EventLogEntryType.Error); }
                        }));
                    });
                }
                catch (Exception e) { Utilities.Log(e.Message, EventLogEntryType.Error); }
            }
            _stopping = false;
        }
        /// <summary>
        /// de-accelerated selected sensor
        /// </summary>
        private void StopSelectedSensor()
        {
            _stopping = true;

            var selectedSensor = (DataContext as SensorsVM).SelectedSensor;
            // Need to run this as seperate task to allow the UI thread to update.
            Task.Run(() => {
                selectedSensor.Stop(SensorVM.StopVmTimeout);
                mainThreadDispatcher.Invoke(new Action(() =>
                {
                    SensorStopped(selectedSensor);

                }));
            });
        }
        /// <summary>
        /// stopping all accelerated sensors
        /// </summary>
        public void StopAllVMProcesses()
        {
            if (_appExiting) return;
            SensorsVM vm = (SensorsVM)DataContext;
            foreach (var sensor in vm.Sensors)
            {
                if (sensor.AccelerationEnabled) { sensor.Stop(SensorVM.StopVmTimeout); }
            }
        }
        /// <summary>
        /// called when a given sensor was deaccelerated or its vm process terminated
        /// </summary>
        /// <param name="sensor"></param>
        private void SensorStopped(SensorVM sensor)
        {
            if (!_appExiting && null != sensor)
            {
                sensor.AccelerationEnabled = false;
                SensorsVM vm = (SensorsVM)DataContext;

                //Remove from accelerated list and update
                vm.RemoveAcceleratedID(sensor.Id);
                vm.UpdateCommand.Execute(null);

                //Reselect the sensor in the list
                SelectSensor(sensor.Id);
                UpdateStartBtnAndSensorStatus(sensor);
                BlockUI(false);
                BlockSpinners(false);
                BlockResetButton(false);

                Utilities.Log(string.Format("Sensor {0} decelerated.", sensor.Id));
            }
        }
        #endregion
        /// <summary>
        /// select in the list a given sensor
        /// </summary>
        /// <param name="sid"></param>
        protected void SelectSensor(uint sid)
        {
            for (int i = 0; i < lstSensors.Items.Count; i++)
            {
                var item = (SensorVM)lstSensors.Items[i];
                if (item.Id == sid)
                {
                    lstSensors.SelectedIndex = i;
                    break;
                }
            }
        }
        private string GetOpenPort()
        {
            _unusedPorts = new List<int>();
            int PortStartIndex = ControlPort;
            int PortEndIndex = MaxControlPort;
            IPGlobalProperties properties = IPGlobalProperties.GetIPGlobalProperties();
            IPEndPoint[] tcpEndPoints = properties.GetActiveTcpListeners();

            List<int> usedPorts = tcpEndPoints.Select(p => p.Port).ToList<int>();
            int unusedPort = 0;

            for (int port = PortStartIndex; port < PortEndIndex; port++)
            {
                if (!usedPorts.Contains(port))
                {
                    _unusedPorts.Add(port);
                }
            }
            PortStartIndex = BaseWebPort;
            PortEndIndex = MaxWebPort;
            _unusedWebports = new List<int>();
            for (int port = PortStartIndex; port < PortEndIndex; port++)
            {
                if (!usedPorts.Contains(port))
                {
                    _unusedWebports.Add(port);
                }
            }
            return unusedPort.ToString();
        }
    }
}

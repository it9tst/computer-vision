using GoAcceleratorEngine.Properties;
using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.ComponentModel;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Input;

namespace GoAcceleratorEngine
{
    /// <summary>
    /// View model for a list of GoSensors
    /// </summary>
    public class SensorsVM : INotifyPropertyChanged
    {
        public event PropertyChangedEventHandler PropertyChanged;
        private readonly CommandAction _updateCommand;
        private readonly CommandAction _refreshCommand;
        /// <summary>
        /// An observable list of sensors.
        /// </summary>
        private ObservableCollection<SensorVM> _sensors = new ObservableCollection<SensorVM>();
        private List<uint> _AcceleratedSensorsIDS = new List<uint>();

        public SensorsVM()
        {
            // Initialize commands
            // -- refresh and check previously accelerated
            _refreshCommand = new CommandAction(() => Update(true, true), true);
            // -- don't refresh but check previously accelerated
            _updateCommand = new CommandAction(() => Update(false, true), true);

            // Start with a refreshed and updated list of sensors
            Update(true, false);
        }
        public void AddAcceleratedID(uint sid)
        {
            _AcceleratedSensorsIDS.Add(sid);
        }
        public CommandAction RefreshCommand 
        {
            get { return _refreshCommand; }
        }
        public void RemoveAcceleratedID(uint sid)
        {
            _AcceleratedSensorsIDS.Remove(sid);
            // NOTE: we can leave the sensor in the _sensors list
        }
        public CommandAction UpdateCommand
        {
            get { return _updateCommand; }
        }
        /// <summary>
        /// Builds the list of sensor view models.
        /// </summary>
        /// <param name="refreshSystem">Whether the underlying system (ie. network) should be refreshed, 
        /// hence removing sensor objects that are no longer applicable.</param>
        /// <param name="previouslyAcceleratedCheck"><para>To get the latest acceleration status;</para>
        /// <para> performed while running, vs initial startup.</para></param>
        /// <remarks>
        /// To preserve the current state of previously loaded sensors, special care is given
        /// to ensure we re-use existing view model instances instead of creating new ones.
        /// </remarks>
        private void Update(bool refreshSystem, bool previouslyAcceleratedCheck)
        {
            uint selectedSensorId = _selectedSensor != null ? _selectedSensor.Id : 0;

            //SelectedSensor = null;

            ObservableCollection<SensorVM> newSensors = new ObservableCollection<SensorVM>();

            // only refresh if requested to do so, this will prune-out 
            // offline sensors in the subsequent calls to get devices
            if (refreshSystem)
            {
                Engine.SystemRefresh();
            }

            List<uint> allIds = Engine.GetDeviceIds();
            if(Properties.Settings.Default.Sensors == null)
            {
                //Create new collection
                Properties.Settings.Default.Sensors= new List<Properties.SensorSettings>();
            }

            // No need to union with _AcceleratedSensorsIDS since we never remove them from the list

            // Build the new list of sensors, re-using existing view models where possible
            foreach (uint id in allIds)
            {
               
                SensorVM sensor = null;

                for (int i = 0; i < _sensors.Count; i++)
                {
                    if(_sensors[i].Id == id)
                    {
                        sensor = _sensors[i];
                        break;
                    }
                }

                // Don't add local devices because they might be emulators, unless they are
                // already in the list because they are accelerated instances managed by this object.

                if (sensor == null && !Engine.IsSensorLocal(id)) 
                { 
                    sensor = Engine.GetSensorVM(id);

                    // Add a handler here so we know when to avoid making critical system changes

                }

                if (sensor != null)
                {
                    newSensors.Add(sensor);
                
                    bool Exists = false;
                    foreach (var SensorProperties in Properties.Settings.Default.Sensors)
                    {
                        bool isOwned = _AcceleratedSensorsIDS.Contains(id);
                        if (SensorProperties.Id == id)
                        {
                            //set the current settings
                            sensor.Settings = SensorProperties;
                            
                            // For refresh, we care about the actual "latest" acceleration status.
                            if (refreshSystem && previouslyAcceleratedCheck)
                            {
                                sensor.RefreshAcceleration(isOwned);
                            }
                            // For update, we want to set the "pending" accelerating status.
                            else if (previouslyAcceleratedCheck)
                            {
                                // NOTE: the AccelerationEnabled setter has side effect of
                                // updating all the properties of the entry
                                sensor.AccelerationEnabled = isOwned;
                                sensor.AcceleratedState = sensor.AccelerationEnabled ?
                                    AccelerationState.Started : AccelerationState.Stopped;
                            }
                            // else -- use the last saved settings

                            Exists = true;
                            break;
                        }
                    }
                    SensorSettings Settings;
                    if (!Exists)
                    {
                        //create new settings
                        Settings = new SensorSettings();
                        Settings.Id = id;
                        Settings.WebPort = 8080;
                        Settings.BasePort = 3190;
                        Settings.AcceleratedSensorIp = "Any";
                        Settings.Accelerated = false;
                        Settings.AcceleratedState = AccelerationState.Stopped;
                        sensor.Settings = Settings;
                        Properties.Settings.Default.Sensors.Add(Settings);
                        Properties.Settings.Default.Save();
                    }
                    //sensor.LastError = sensor.Status;
                }
            }
            
            // Update the list of sensors
            Sensors = newSensors;

            // Attempt to use a previously selected sensor
            for (int i = 0; i < _sensors.Count; i++)
            {
                if (_sensors[i].Id == selectedSensorId) 
                {
                    SelectedSensor = _sensors[i];
                    break;
                }
            }

            // If no sensors were previously selected, choose the first one (if any available)
            if(_selectedSensor == null && _sensors.Count > 0)
            {
                SelectedSensor = _sensors[0];
            }
        }

       
        public ObservableCollection<SensorVM> Sensors
        {
            get { return _sensors; }
            private set
            {
                _sensors = value;
                
                if (PropertyChanged != null) { PropertyChanged(this, new PropertyChangedEventArgs("Sensors")); }
            }
        }
        
        /// <summary>
        /// The selected sensor.
        /// </summary>
        private SensorVM _selectedSensor;
        public SensorVM SelectedSensor
        {
            get { return _selectedSensor; }
            set
            {
                _selectedSensor = value;

                if (PropertyChanged != null) 
                { 
                    PropertyChanged(this, new PropertyChangedEventArgs("SelectedSensor"));
                    PropertyChanged(this, new PropertyChangedEventArgs("SensorExists"));
                }
            }
        }

        /// <summary>
        /// Determines if we have any sensor exists and is selected.
        /// </summary>
        public bool SensorExists
        {
            get { return _selectedSensor != null; }
        }

        /// <summary>
        /// Returns the compatible firmware version.
        /// </summary>
        /// <remarks>
        /// In order to accelerate a sensor, this version must match 
        /// the firmware version on the sensor.
        /// </remarks>
        public string FirmwareVersion
        {
            get { return Engine.GetFirmwareVersion(); }
        }
    }
}

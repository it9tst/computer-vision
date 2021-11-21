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
    /// View model for settings
    /// </summary>
    public class SettingsVM : INotifyPropertyChanged
    {
        public event PropertyChangedEventHandler PropertyChanged;

        public SettingsVM()
        {
        }

        public bool StartWithWindows
        {
            get
            {
                return Settings.IsStartupSet();
            }
            set
            {
                Settings.SetStartup(value);

                if (PropertyChanged != null)
                {
                    PropertyChanged(this, new PropertyChangedEventArgs("StartWithWindows"));
                }
            }
        }

        public bool AccelerateOnStartup
        {
            get { return Properties.Settings.Default.AccelerateOnStartup; }
            set
            {
                Properties.Settings.Default.AccelerateOnStartup = value;
                Properties.Settings.Default.Save();

                if (PropertyChanged != null)
                {
                    PropertyChanged(this, new PropertyChangedEventArgs("AccelerateOnStartup"));
                }
            }
        }

        public bool ShowMainWindowOnStartup
        {
            get { return Properties.Settings.Default.ShowMainWindowOnStartup; }
            set
            {
                Properties.Settings.Default.ShowMainWindowOnStartup = value;
                Properties.Settings.Default.Save();

                if (PropertyChanged != null)
                {
                    PropertyChanged(this, new PropertyChangedEventArgs("ShowMainWindowOnStartup"));
                }
            }
        }

        public bool ShowTaskbarIcon
        {
            get { return Properties.Settings.Default.ShowTaskbarIcon; }
            set
            {
                Properties.Settings.Default.ShowTaskbarIcon = value;
                Properties.Settings.Default.Save();

                if (PropertyChanged != null)
                {
                    PropertyChanged(this, new PropertyChangedEventArgs("ShowTaskbarIcon"));
                }
            }
        }
    }
}

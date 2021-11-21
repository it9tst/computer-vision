using Microsoft.Win32;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Reflection;
using System.Configuration;

namespace GoAcceleratorEngine
{
   
    class Settings
    {
        private static string GetStartupValue()
        {
            return "\"" + Utilities.ApplicationPath + "\"";
        }

        /// <summary>
        /// Adds a key to the registry to enable auto-starting this application.
        /// </summary>
        internal static bool SetStartup(bool enabled)
        {
            try
            {
                RegistryKey key = Registry.CurrentUser.OpenSubKey("Software\\Microsoft\\Windows\\CurrentVersion\\Run", true);
                
                if (enabled)
                {
                    key.SetValue(Utilities.ApplicationName, GetStartupValue());
                }
                else
                {
                    try
                    {
                        key.DeleteValue(Utilities.ApplicationName);
                    }
                    catch
                    {
                        // it might not exist
                    }
                }

                key.Close();
            }
            catch { return false; }

            return true;
        }

        internal static bool IsStartupSet()
        {
            try
            {
                bool ret = false;
                RegistryKey key = Registry.CurrentUser.OpenSubKey("Software\\Microsoft\\Windows\\CurrentVersion\\Run", true);

                string value = (string)key.GetValue(Utilities.ApplicationName);
                if (value == GetStartupValue())
                {
                    ret = true;
                }

                key.Close();

                return ret;
            }
            catch
            {
                return false;
            }
        }

    }
}

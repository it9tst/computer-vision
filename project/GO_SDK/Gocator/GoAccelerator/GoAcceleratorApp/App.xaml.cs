using GoAcceleratorEngine;
using System;
using System.Reflection;
using System.Threading;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Interop;

namespace GoAccelerator
{
   /// <summary>
   /// Interaction logic for App.xaml
   /// </summary>
   public partial class App : Application
   {
       private Mutex mutex = null;
       private const string MUTEX_NAME = "{GoAcceleratorApp/315C9DAB-1B87-495C-B9F2-D6DFA6AE367B}";

       void App_Startup(object sender, StartupEventArgs e)
       {

           bool createdNew;
           mutex = new Mutex(true, MUTEX_NAME, out createdNew);

            if (!createdNew)
            {
                // Running instance already exists. Exit this new instance.
                MessageBox.Show("An instance of GoAccelerator.exe is already running. Aborting.");
                Current.Shutdown();
            }
            else
            {
                // First instance of application.
                GC.KeepAlive(mutex);

                AppDomain.CurrentDomain.UnhandledException += CurrentDomain_UnhandledException;
                TaskScheduler.UnobservedTaskException += TaskScheduler_UnobservedTaskException;

                // Initialize the accelerator engine assembly
                Engine.Initialize();

                // GOC-13597: Check if user.config file is corrupted.
                // Notify user and restart program
                if (!Engine.IsValidSettingsFile())
                {
                    MessageBox.Show("The user settings file has been corrupted. Restarting Gocator Accelerator.", "Error", MessageBoxButton.OK, MessageBoxImage.Error);
                    // Close this process and restart a new one.
                    System.Diagnostics.Process.Start(ResourceAssembly.Location);
                    Current.Shutdown();
                    return;
                }

                // Create the main application window
                MainWindow = new MainWindow();

                // Force loading even if the window is not visible
                new WindowInteropHelper(MainWindow).EnsureHandle();
            }
       }

       ////////////////////////////////////////////////////////////////////////////
       //
       ////////////////////////////////////////////////////////////////////////////

       void Current_DispatcherUnhandledException(object sender, System.Windows.Threading.DispatcherUnhandledExceptionEventArgs e)
       {
           UnhandledException(MethodBase.GetCurrentMethod().Name, e.Exception.Message);
       }

       ////////////////////////////////////////////////////////////////////////////
       //
       ////////////////////////////////////////////////////////////////////////////

       void TaskScheduler_UnobservedTaskException(object sender, UnobservedTaskExceptionEventArgs e)
       {
           UnhandledException(MethodBase.GetCurrentMethod().Name, e.Exception.Message);
       }

       ////////////////////////////////////////////////////////////////////////////
       //
       ////////////////////////////////////////////////////////////////////////////

       void CurrentDomain_UnhandledException(object sender, UnhandledExceptionEventArgs e)
       {
           //UnhandledException(MethodBase.GetCurrentMethod().Name, e.ExceptionObject.ToString()); // too verbose
       }

       ////////////////////////////////////////////////////////////////////////////
       //
       ////////////////////////////////////////////////////////////////////////////

       void UnhandledException(string sourceEvent, string exceptionStr)
       {
            // present a more user-friendly error
            MessageBox.Show(exceptionStr, "Error", MessageBoxButton.OK, MessageBoxImage.Error);
        }
   }
}

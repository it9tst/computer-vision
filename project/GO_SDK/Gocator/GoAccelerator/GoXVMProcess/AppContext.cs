using GoXVMProcess;
using Lmi3d.GoSdk;
using Lmi3d.Zen;
using Lmi3d.Zen.Io;
using Microsoft.Win32;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.IO.Pipes;
using System.Linq;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Forms;
using System.Windows.Threading;
using GoAcceleratorEngine;

namespace GoXVMProcess
{
    class AppContext : ApplicationContext
    {
        private string m_ID;

        private Dispatcher mainThreadDispatcher;
        private SensorVM _sensorVM;
        private EventWaitHandle _ewhstart = null;
        private EventWaitHandle _ewhstop = null;
        private EventWaitHandle logoffEvent;
       
        private Process _ParentProcess = null;
        private RegisteredWaitHandle _regStop = null;
        private SessionEndingEventHandler logoffHandler;

        // Constructor
        public AppContext(uint paramID, uint WebPort, uint BasePort, uint HealthPort, uint UpgradePort, uint PrivateDataPort, uint PublicDataPort,
            string ipAddress, bool ignoreStopEvent)
        {
            try
            {
                Process[] p = Process.GetProcessesByName("GoAccelerator");
                if (p.Length > 0)
                {
                    _ParentProcess = p[0];
                    _ParentProcess.EnableRaisingEvents = true;
                    _ParentProcess.Exited += Parent_Exited;
                }
                Console.Write("ignoreStopEvent = " + ignoreStopEvent.ToString() + "\n");
                
                m_ID = paramID.ToString();

                if (ignoreStopEvent)
                {
                    Console.Write("Creating event handles\n");
                    _ewhstart = Utilities.GetNamedEventHandle(paramID, "Start", true);
                    _ewhstop = Utilities.GetNamedEventHandle(paramID, "Stop", true);
                    logoffEvent = Utilities.GetNamedEventHandle(paramID, "Logoff", true);
                    _regStop = ThreadPool.RegisterWaitForSingleObject(_ewhstop, Exit, m_ID, -1, true);                    
                }
                mainThreadDispatcher = Dispatcher.CurrentDispatcher;

                Console.Write(string.Format("({0}/{1}) Waiting on Sensor {2}, Webport {3}, Baseport {4}\n",
                    Process.GetCurrentProcess().Id, Thread.CurrentThread.ManagedThreadId, m_ID, WebPort, BasePort));

                //
                KApiLib.MessageLogged += msg =>
                {
                    Console.WriteLine(string.Format("{0:D2}:{1:D2}:{2:D2}.{3:D3}: {4}", DateTime.Now.Hour, 
                        DateTime.Now.Minute, DateTime.Now.Second, DateTime.Now.Millisecond, msg));
                };
                KApiLib.OnAssert += msg => {
                    Console.WriteLine(msg);
                    throw new Exception(msg);
                };

                SystemEvents.SessionEnding += (logoffHandler = new SessionEndingEventHandler(this.OnLogoff));

                Engine.Initialize();
                _sensorVM = Engine.GetSensorVM(paramID);
                _sensorVM.WebPort = WebPort;
                _sensorVM.BasePort = BasePort;
                _sensorVM.HealthPort = HealthPort;
                _sensorVM.PrivateDataPort = PrivateDataPort;
                _sensorVM.UpgradePort = UpgradePort;
                _sensorVM.PublicDataPort = PublicDataPort;
                _sensorVM.Address = KIpAddress.Parse(ipAddress);                
                _sensorVM.AccelerationEnabled = true;

                Console.Write("Sensor Accelerated\n");
                if (ignoreStopEvent)
                {
                    Console.Write("called ewhstart.Set()" + "\n");
                    _ewhstart.Set();
                }
            }
            catch (KException kEx)
            {
                string ErrorMessage = "";
                string ErrorStack = "";
                if (kEx.Status.Value == KStatus.ErrorVersion)
                {
                    ErrorMessage = "Failed to enable acceleration due to version error.\n\nThe target sensor's version number does not match the version of the application.";
                }
                else if (kEx.Status.Value == KStatus.ErrorConflict)
                {
                    ErrorMessage = "Failed to enable acceleration. The sensor status may have changed.\n\nPlease refresh the Sensors list and try again.";
                }
                else if (kEx.Status.Value == KStatus.ErrorNotFound)
                {
                    ErrorMessage = "Failed to enable acceleration ... Please ensure that the sensor is online....";            
                }
                else
                {
                    ErrorMessage = kEx.Message;
                    ErrorStack = kEx.StackTrace;
                }
                WriteError(ErrorMessage, ErrorStack, kEx.Status.Value);
            }
            catch (Exception e)
            {
                WriteError(e.Message, e.StackTrace);
            }
        }
        private void OnLogoff(object sender, SessionEndingEventArgs args)
        {
            GoAcceleratorEngine.Utilities.Log(string.Format("{0} event caught.", (args.Reason == SessionEndReasons.Logoff ? "Loggoff" : "Shutdown")));
            logoffEvent.Set();
            Exit(null, false);
            return;
        }

        private void WriteError(string ErrorMessage, string StackTrace,int ErrorNumber = 0)
        {
            try
            {
                using (NamedPipeClientStream errPipe =
                    (NamedPipeClientStream)Utilities.OpenPipe(m_ID, true, PipeDirection.Out))
                {
                    errPipe.Connect();
                    if (null != errPipe && errPipe.IsConnected && errPipe.CanWrite)
                    {
                        try
                        {
                            using (StreamWriter sw = new StreamWriter(errPipe))
                            {
                                sw.AutoFlush = true;
                                var msg = new GoAcceleratorEngine.ErrorInfo(ErrorNumber, ErrorMessage, StackTrace);
                                Console.WriteLine("sending msg: " + msg.ToString());
                                sw.WriteLine(msg.ToString());
                            }
                        }
                        catch (Exception e)
                        {
                            Utilities.Log("ERROR: " + e.Message, EventLogEntryType.Error);
                            Console.WriteLine(e.Message);
                        }
                    }
                    CloseEvents();
                }

            }
            catch (Exception e)
            {
                Utilities.Log(e.Message, EventLogEntryType.Error);
            }
            finally
            {
                Environment.Exit(-1);
            }

        }
        private void Parent_Exited(object sender, EventArgs ea)
        {
            Terminate(); 
        }
        private void CloseEvents()
        {
            try
            {
                if (null != logoffEvent)
                {
                    logoffEvent.Close();
                    logoffEvent = null;
                }
                if (_ewhstart != null)
                {
                    _ewhstart.Close();
                    _ewhstart = null;
                }
                if (_ewhstop != null)
                {
                    if (_regStop != null)
                    {
                        _regStop.Unregister(null);
                    }
                    _ewhstop.Close();
                    _ewhstop = null;
                }
            }
            catch (Exception e)
            {
                GoAcceleratorEngine.Utilities.Log(e.Message, EventLogEntryType.Error);
            }
        }
        private void Exit(object state, bool timedOut)
        {
            GoAcceleratorEngine.Utilities.Log("GoXVMProcess exiting...");
            Terminate();
        }
        private void Terminate()
        {
            mainThreadDispatcher.Invoke(new Action(() => { 
                try
                {
                    _sensorVM.AccelerationEnabled = false;
                    CloseEvents();
                    Environment.Exit(0);
                }
                catch (Exception e)
                {
                    WriteError(e.Message, e.StackTrace);
                }
            }));
        }
    }
}

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Reflection;
using System.Diagnostics;
using System.Threading;
using System.IO.Pipes;
using System.Security.Principal;

namespace GoAcceleratorEngine
{
    public class Utilities
    {
        static public readonly string GoxPrefix = "LMI.GoX.";

        #region Application Name/Path functions
        /// <summary>
        /// The application assembly name.
        /// </summary>
        internal static string ApplicationName
        {
            get
            {
                Assembly asm = Assembly.GetEntryAssembly();

                if (asm == null)
                {
                    // this can happen if the entry assembly is unmanaged
                    // just return a placeholder name
                    return "UnmanagedAssembly";
                }

                return asm.GetName().Name;
            }
        }

        /// <summary>
        /// Full path to the application executable.
        /// </summary>
        internal static string ApplicationPath
        {
            get
            {
                Assembly asm = Assembly.GetEntryAssembly();
                if (asm == null) { return null; }

                return asm.Location;
            }
        }
        #endregion

        #region windows event logger
        static public readonly string goxSource = "GoAccelerator";
        /// <summary>
        /// writes a message into win 'Application' event log and source 'GoAccelerator'
        /// to be able to write a 'GoAccelerator' msg, GoAccelerator must be called first with admi rights and command line arg: --register-source
        /// </summary>
        /// <param name="msg">message to write in the log</param>
        /// <param name="type">event log type: error, warning, info...; info is default if not specified</param>
        /// <param name="lineNumber"></param>
        /// <param name="caller"></param>
        static public void Log(string msg, EventLogEntryType type = EventLogEntryType.Information,
            [System.Runtime.CompilerServices.CallerLineNumber] int lineNumber = 0,
            [System.Runtime.CompilerServices.CallerMemberName] string caller = null,
            [System.Runtime.CompilerServices.CallerFilePath] string filePath = "")
        {
            EventLog eventLog = null;
            try
            {
                eventLog = new EventLog("Application");
                var logMsg = string.Format("{0:D2}:{1:D2}:{2:D2}.{3:D3} {4}({5}): {6} PID[{7}/{8}] {9}\r\n",
                        DateTime.Now.Hour, DateTime.Now.Minute, DateTime.Now.Second,
                        DateTime.Now.Millisecond, caller, lineNumber, ApplicationName,
                        Process.GetCurrentProcess().Id, Thread.CurrentThread.ManagedThreadId, msg);
//
// [rchang] Useful for debugging.  This makes the debug output in the VS Console window clickable:
// https://docs.microsoft.com/en-us/cpp/build/formatting-the-output-of-a-custom-build-step-or-build-event?view=vs-2017
// I'm leaving this commented out just in case the "Debugger.IsAttached" property causes performance issues or platform
// compatibility issues.
//
//#if DEBUG
//                if (System.Diagnostics.Debugger.IsAttached)
//                {
//                    Debug.WriteLine(string.Format("{0}({1}): {2}", System.IO.Path.GetFileName(filePath), lineNumber, logMsg));
//                }
//#endif
                eventLog.Source = goxSource;
                eventLog.WriteEntry(logMsg, type);
            }

            catch (Exception e)
            {
                Console.WriteLine(string.Format("Exception at {0}({1}): {2}", caller, lineNumber, e.Message));
            }
        }
        /// <summary>
        /// performs a registration of source 'GoAccelerator' in registry 
        /// </summary>
        public static void RegisterRegistrySource()
        {
            if (IsUserAdministrator())
            {
                var source = "GoAccelerator";
                //Utilities.Log("Running as admin");
                using (EventLog eventLog = new EventLog("Application"))
                {
                    if (!EventLog.SourceExists(source))
                    {
                        EventLog.CreateEventSource(source, "Application");
                    }
                }
            }
        }
        #endregion

        /// <summary>
        /// create/opens a specific named event
        /// </summary>
        /// <param name="sid">sensor id</param>
        /// <param name="type">start/stop/logoff</param>
        /// <param name="openExisting">=true if opening existing event; otherwise false</param>
        /// <returns></returns>
        static public EventWaitHandle GetNamedEventHandle(uint sid, string type, bool openExisting)
        {
            string eventName = GoxPrefix + type + sid.ToString();
            return openExisting ?
                EventWaitHandle.OpenExisting(eventName) :
                new EventWaitHandle(false, EventResetMode.AutoReset, eventName);
        }

        #region opening IPC pipes
        /// <summary>
        /// open/create a named pipe between a sensor vm process and gox parent
        /// </summary>
        /// <param name="sid">sensor id</param>
        /// <param name="client">false for child process; true for parent gox process</param>
        /// <param name="dir">direction of communication: in, out, in/out</param>
        /// <returns></returns>
        static public PipeStream OpenPipe(uint sid, bool client, PipeDirection dir, string type="Error")
        {
            return OpenPipe(sid.ToString(), client, dir, type);
        }
        /// <summary>
        /// open/create a named pipe between a sensor vm process and gox parent
        /// </summary>
        /// <param name="sid">sensor id</param>
        /// <param name="client">false for child process; true for parent gox process</param>
        /// <param name="dir">direction of communication: in, out, in/out</param>
        /// <returns></returns>
        static public PipeStream OpenPipe(string sid, bool client, PipeDirection dir, string type="Error")
        {
            var pipeName = GoxPrefix + type + "Pipe." + sid;
            return client ?
                (PipeStream)new NamedPipeClientStream(".", pipeName, dir) :
                // It seems like the async flag makes it easier to re-open the pipe when the maxInstances=1
                (PipeStream)new NamedPipeServerStream(pipeName, dir, 1, PipeTransmissionMode.Byte, PipeOptions.Asynchronous);
        }
        #endregion

        #region time conversion
        /// <summary>
        /// converts seconds into miliseconds
        /// </summary>
        /// <param name="secs">number of seconds to conver into ms</param>
        /// <returns></returns>
        public static int Sec2Ms(int secs) { return (secs * 1000); }
        /// <summary>
        /// converts minutes into miliseconds
        /// </summary>
        /// <param name="mins">number of minutes to convert</param>
        /// <returns></returns>
        public static int Min2Ms(int mins) { return Sec2Ms(mins * 60); }
        #endregion

        /// <summary>
        /// performs a test if application is running as an admin with elevated privileges
        /// </summary>
        /// <returns></returns>
        public static bool IsUserAdministrator()
        {
            try
            {
                WindowsIdentity user = WindowsIdentity.GetCurrent();
                WindowsPrincipal principal = new WindowsPrincipal(user);
                return principal.IsInRole(WindowsBuiltInRole.Administrator);
            }
            catch (UnauthorizedAccessException)
            {
                return false;
            }
            catch (Exception)
            {
                return false;
            }
        }
    }
}

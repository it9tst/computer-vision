'BackupRestore.vb

'Gocator 2000/2300 Visual Basic Sample
'Copyright (C) 2013 by LMI Technologies Inc.

'Licensed under The MIT License.
'Redistributions of files must retain the above copyright notice.

'Purpose: Backup a Gocator system and restore it

Imports Lmi3d.GoSdk
Imports Lmi3d.Zen
Imports Lmi3d.Zen.Io

Module BackupRestore
    Public Const IP As String = "192.168.1.10"
    Public Const BackupFileName = "SystemBackup.bin" REM will store file in current directory

    Function Main() As Integer

        Try
            KApiLib.Construct()
            GoSdkLib.Construct()

            Dim gsystem As New GoSystem
            Dim gocator As GoSensor = Nothing
            Dim ipAddress As KIpAddress = KIpAddress.Parse(IP)

            gocator = gsystem.FindSensorByIpAddress(ipAddress)
            gocator.Connect()
            gocator.EnableData(True)
            gocator.Stop()

            Console.WriteLine("Creating backup file: {0}", BackupFileName)
            Console.WriteLine("-----------------------------")
            gocator.Backup(BackupFileName)


            Console.WriteLine("Restoring from backup file: {0}", BackupFileName)
            Console.WriteLine("-----------------------------")
            Console.WriteLine("Wait...may take 30+ seconds....")
            gocator.Restore(BackupFileName)

            'wait for ESC key
            Console.WriteLine("Press ESCAPE to Exit")
            Do While Not Console.ReadKey(True).Key = ConsoleKey.Escape
            Loop

            gocator.Destroy()
        Catch ex As Exception
            Console.WriteLine("{0}.....Press ESCAPE to Exit", ex.ToString())
            Do While Not Console.ReadKey(True).Key = ConsoleKey.Escape
            Loop
            Return KStatus.Error
            Exit Function
        End Try
        Return KStatus.Ok
    End Function

End Module
















































'Module BackupRestore
'    Public Const IP As String = "192.168.1.10"
'    Public Class Constants
'#If DEBUG Then
'        Public Const GODLLPATH As String = "GoSdk.dll"
'        Public Const KAPIDLLPATH As String = "kApi.dll"
'#Else
'        Public Const GODLLPATH As String = "GoSdk.dll"
'        Public Const KAPIDLLPATH As String = "kApi.dll"
'#End If
'    End Class
'    Public Structure address
'        Public version As Char
'        <MarshalAs(UnmanagedType.ByValArray, SizeConst:=16)> _
'        Public IPaddress() As Byte
'    End Structure

'    Enum kStatus
'        kERROR_STATE = -1000                                                ' Invalid state.
'        kERROR_NOT_FOUND = -999                                             ' Item is not found.
'        kERROR_COMMAND = -998                                               ' Command not recognized.
'        kERROR_PARAMETER = -997                                             ' Parameter is invalid.
'        kERROR_UNIMPLEMENTED = -996                                         ' Feature not implemented.
'        kERROR_HANDLE = -995                                                ' Handle is invalid.
'        kERROR_MEMORY = -994                                                ' Out of memory.
'        kERROR_TIMEOUT = -993                                               ' Action timed out.
'        kERROR_INCOMPLETE = -992                                            ' Buffer not large enough for data.
'        kERROR_STREAM = -991                                                ' Error in stream.
'        kERROR_CLOSED = -990                                                ' Resource is no longer avaiable. 
'        kERROR_VERSION = -989                                               ' Invalid version number.
'        kERROR_ABORT = -988                                                 ' Operation aborted.
'        kERROR_ALREADY_EXISTS = -987                                        ' Conflicts with existing item.
'        kERROR_NETWORK = -986                                               ' Network setup/resource error.
'        kERROR_HEAP = -985                                                  ' Heap error (leak/double-free).
'        kERROR_FORMAT = -984                                                ' Data parsing/formatting error. 
'        kERROR_READ_ONLY = -983                                             ' Object is read-only (cannot be written).
'        kERROR_WRITE_ONLY = -982                                            ' Object is write-only (cannot be read). 
'        kERROR_BUSY = -981                                                  ' Agent is busy (cannot service request).
'        kERROR_CONFLICT = -980                                              ' State conflicts with another object.
'        kERROR_OS = -979                                                    ' Generic error reported by underlying OS.
'        kERROR_DEVICE = -978                                                ' Hardware device error.
'        kERROR_FULL = -977                                                  ' Resource is already fully utilized.
'        kERROR_IN_PROGRESS = -976                                           ' Operation is in progress, but not yet complete.
'        kERROR = 0                                                          ' General error. 
'        kOK = 1                                                             ' Operation successful. 
'    End Enum

'    ' use DLL import to access GoSdkd.dll
'    ' Note: to import the release version of the GoSdk and kApi, use GoSdk.dll and kApi.dll. GoSdkd.dll and kApid.dll are debug versions of the DLL
'    <DllImport(Constants.GODLLPATH)> _
'    Public Function GoSdk_Construct(ByRef assembly As IntPtr) As kStatus
'    End Function
'    <DllImport(Constants.GODLLPATH)> _
'    Public Function GoSystem_Construct(ByRef system As IntPtr, ByVal allocator As IntPtr) As kStatus
'    End Function
'    <DllImport(Constants.GODLLPATH)> _
'    Public Function GoSystem_FindSensorByIpAddress(ByVal system As IntPtr, ByVal addressPointer As IntPtr, ByRef sensor As IntPtr) As kStatus
'    End Function
'    <DllImport(Constants.GODLLPATH)> _
'    Public Function GoSensor_Connect(ByVal sensor As IntPtr) As kStatus
'    End Function
'    <DllImport(Constants.GODLLPATH)> _
'    Public Function GoSensor_Backup(ByVal sensor As IntPtr, ByVal destPath As String) As kStatus
'    End Function
'    <DllImport(Constants.GODLLPATH)> _
'    Public Function GoSensor_Restore(ByVal sensor As IntPtr, ByVal destPath As String) As kStatus
'    End Function
'    <DllImport(Constants.GODLLPATH)> _
'    Public Function GoSensor_Stop(ByVal sensor As IntPtr) As kStatus
'    End Function
'    <DllImport(Constants.GODLLPATH)> _
'    Public Function GoDestroy(ByVal obj As IntPtr) As kStatus
'    End Function
'    <DllImport(Constants.KAPIDLLPATH)> _
'    Public Function kIpAddress_Parse(ByVal addressPointer As IntPtr, ByVal text As String) As kStatus
'    End Function

'    Function Main() As Integer
'        Dim status As kStatus
'        Dim api As IntPtr = IntPtr.Zero
'        Dim system As IntPtr = IntPtr.Zero
'        Dim sensor As IntPtr = IntPtr.Zero
'        Dim backupFile As String = "SystemBackup.bin"
'        Dim addr As New address
'        Dim addrPtr As IntPtr = IntPtr.Zero

'        status = GoSdk_Construct(api)
'        If status <> 1 Then
'            Console.WriteLine("GoSdk_Construct Error:{0}", status)
'            Return status
'        End If

'        status = GoSystem_Construct(system, IntPtr.Zero)
'        If status <> 1 Then
'            Console.WriteLine("GoSystem_Construct Error:{0}", status)
'            Return status
'        End If

'        addrPtr = Marshal.AllocHGlobal(Marshal.SizeOf(addr))
'        Marshal.StructureToPtr(addr, addrPtr, False)

'        status = kIpAddress_Parse(addrPtr, IP)
'        If status <> 1 Then
'            Console.WriteLine("kIpAddress_Parse Error:{0}", status)
'            Return status
'        End If

'        status = GoSystem_FindSensorByIpAddress(system, addrPtr, sensor)
'        If status <> 1 Then
'            Console.WriteLine("GoSystem_FindSensorByIpAddress Error:{0}", status)
'            Return status
'        End If

'        status = GoSensor_Connect(sensor)
'        If status <> 1 Then
'            Console.WriteLine("GoSensor_Connect Error:{0}", status)
'            Return status
'        End If

'        Console.WriteLine("Creating backup file: {0}", backupFile)
'        Console.WriteLine("-----------------------------")

'        status = GoSensor_Backup(sensor, backupFile)
'        If status <> 1 Then
'            Console.WriteLine("GoSensor_Backup Error:{0}", status)
'            Return status
'        End If

'        Console.WriteLine("Restoring from backup file: {0}", backupFile)
'        Console.WriteLine("-----------------------------")

'        status = GoSensor_Restore(sensor, backupFile)
'        If status <> 1 Then
'            Console.WriteLine("GoSensor_Restore Error:{0}", status)
'            Return status
'        End If

'        status = GoDestroy(system)
'        If status <> 1 Then
'            Console.WriteLine("GoDestroy Error:{0}", status)
'            Return status
'        End If

'        status = GoDestroy(api)
'        If status <> 1 Then
'            Console.WriteLine("GoDestroy Error:{0}", status)
'            Return status
'        End If

'        'wait for Enter key
'        Console.WriteLine("Press ENTER to continue")
'        Do While Not Console.ReadKey(True).Key = ConsoleKey.Enter
'            Thread.Sleep(100)
'        Loop
'        Return status
'    End Function

'End Module

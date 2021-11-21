'SetupMeasurement.vb

'Gocator 2300 VB.NET Sample Code
'Copyright (C) 2013, 2018 by LMI Technologies Inc.

'Licensed under The MIT License.
'Redistributions of files must retain the above copyright notice.

'Requirement: Run on G2 sensor in Profile mode.

'Purpose: Connect to Gocator system, setup measurement data under Profile Mode.
'Please refer to ReceiveMeasurement.c for receiving of the measurement data.

Imports System
Imports Lmi3d.GoSdk
Imports Lmi3d.GoSdk.Tools
Imports Lmi3d.GoSdk.Outputs
Imports Lmi3d.Zen
Imports Lmi3d.Zen.Io


Module SetupMeasurement
    Public Const IP As String = "192.168.1.10"

    Function Main() As Integer
        Try
            KApiLib.Construct()
            GoSdkLib.Construct()

            Dim gocator As GoSensor = Nothing
            Dim gsystem As New GoSystem
            Dim ipAddress As KIpAddress = KIpAddress.Parse(IP)

            gocator = gsystem.FindSensorByIpAddress(ipAddress)
            gocator.Connect()
            gocator.EnableData(True)
            Console.WriteLine("GoSdk   Release : {0}", gsystem.SdkVersion.Format())
            Console.WriteLine("Gocator Firmware: {0}", gocator.FirmwareVersion.Format())

            gocator.Stop()

            Dim tools As GoTools = gocator.Tools

            ' first clear all the tools so there is no conflicting tool/measurement ID issue
            gocator.Tools.ClearTools()

            ' add ProfilePosition tool
            Dim profilePositionTool As GoProfilePosition = tools.AddTool(GoToolType.ProfilePosition)
            ' set name for tool
            profilePositionTool.Name = "My Profile Position"
            Console.WriteLine("Tool added {0}", profilePositionTool.Name)
            ' set properties
            profilePositionTool.XMeasurement.Name = "my X"
            profilePositionTool.XMeasurement.Id = 0
            profilePositionTool.XMeasurement.Enabled = True
            Console.WriteLine("Measurement {0} Enabled {1}", profilePositionTool.XMeasurement.Name, profilePositionTool.XMeasurement.Enabled)
            ' Enable Z measurement for ProfilePosition tool
            profilePositionTool.ZMeasurement.Name = "my Z"
            profilePositionTool.ZMeasurement.Id = 1
            profilePositionTool.ZMeasurement.Enabled = False
            Console.WriteLine("Measurement {0} Enabled {1}", profilePositionTool.ZMeasurement.Name, profilePositionTool.ZMeasurement.Enabled)

            ' setup measurement Output
            gocator.Output.GetEthernetAt(0).ClearAllSources()
            ' output only enabled measurements - verify in browser interface by refreshing the page
            If profilePositionTool.XMeasurement.Enabled Then
                gocator.Output.GetEthernetAt(0).AddSource(GoOutputSource.Measurement, profilePositionTool.XMeasurement.Id)
            End If

            If profilePositionTool.ZMeasurement.Enabled Then
                gocator.Output.GetEthernetAt(0).AddSource(GoOutputSource.Measurement, profilePositionTool.ZMeasurement.Id)
            End If

            'wait for ESC key
            Console.WriteLine("Press ESCAPE to Terminate")
            Do While Console.ReadKey(True).Key <> ConsoleKey.Escape
            Loop

            gocator.Destroy()

        Catch ex As Exception
            Console.WriteLine("{0}...Press ESCAPE to Terminate", ex.ToString())
            Do While Console.ReadKey(True).Key <> ConsoleKey.Escape
            Loop
        End Try
        Return KStatus.Ok
    End Function

End Module




'Imports System.Threading
'Imports System.Runtime.InteropServices

'Module SetupMeasurement
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

'    Enum GoToolType
'        GO_TOOL_UNKNOWN = -1
'        GO_TOOL_RANGE_POSITION = 0
'        GO_TOOL_RANGE_THICKNESS = 1
'        GO_TOOL_PROFILE_AREA = 2
'        GO_TOOL_PROFILE_CIRCLE = 3
'        GO_TOOL_PROFILE_DIMENSION = 4
'        GO_TOOL_PROFILE_GROOVE = 5
'        GO_TOOL_PROFILE_INTERSECT = 6
'        GO_TOOL_PROFILE_LINE = 7
'        GO_TOOL_PROFILE_PANEL = 8
'        GO_TOOL_PROFILE_POSITION = 9
'        GO_TOOL_PROFILE_STRIP = 10
'        GO_TOOL_SURFACE_BOUNDING_BOX = 11
'        GO_TOOL_SURFACE_ELLIPSE = 12
'        GO_TOOL_SURFACE_HOLE = 13
'        GO_TOOL_SURFACE_OPENING = 14
'        GO_TOOL_SURFACE_PLANE = 15
'        GO_TOOL_SURFACE_POSITION = 16
'        GO_TOOL_SURFACE_STUD = 17
'        GO_TOOL_SURFACE_VOLUME = 18
'        GO_TOOL_SCRIPT = 19
'    End Enum

'    Enum GoRole
'        GO_ROLE_MAIN = 0                                                    ' Sensor is operating as a main sensor
'        GO_ROLE_BUDDY = 1                                                   ' Sensor is operating as a buddy sensor
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
'    Public Function GoSystem_EnableData(ByVal system As IntPtr, ByVal enable As Boolean) As kStatus
'    End Function
'    <DllImport(Constants.GODLLPATH)> _
'    Public Function GoSensor_Connect(ByVal sensor As IntPtr) As kStatus
'    End Function
'    <DllImport(Constants.GODLLPATH)> _
'    Public Function GoSystem_ReceiveData(ByVal system As IntPtr, ByRef data As IntPtr, ByVal timeout As UInt64) As kStatus
'    End Function
'    <DllImport(Constants.GODLLPATH)> _
'    Public Function GoSensor_Start(ByVal sensor As IntPtr) As kStatus
'    End Function
'    <DllImport(Constants.GODLLPATH)> _
'    Public Function GoSensor_Stop(ByVal sensor As IntPtr) As kStatus
'    End Function
'    <DllImport(Constants.GODLLPATH)> _
'    Public Function GoDataSet_Count(ByVal dataset As IntPtr) As UInt32
'    End Function
'    <DllImport(Constants.GODLLPATH)> _
'    Public Function GoDataSet_At(ByVal dataset As IntPtr, ByVal index As UInt32) As IntPtr
'    End Function
'    <DllImport(Constants.GODLLPATH)> _
'    Public Function GoStampMsg_Count(ByVal msg As IntPtr) As UInt32
'    End Function
'    <DllImport(Constants.GODLLPATH)> _
'    Public Function GoStampMsg_At(ByVal msg As IntPtr, ByVal index As UInt32) As IntPtr
'    End Function
'    <DllImport(Constants.GODLLPATH)> _
'    Public Function GoSensor_Setup(ByVal sensor As IntPtr) As IntPtr
'    End Function
'    <DllImport(Constants.GODLLPATH)> _
'    Public Function GoSensor_Tools(ByVal tools As IntPtr) As IntPtr
'    End Function
'    <DllImport(Constants.GODLLPATH)> _
'    Public Function GoTools_AddTool(ByVal tools As IntPtr, ByVal type As GoToolType, ByRef tool As IntPtr) As Int32
'    End Function
'    <DllImport(Constants.GODLLPATH)> _
'    Public Function GoTool_SetName(ByVal tools As IntPtr, ByVal name As String) As Int32
'    End Function
'    <DllImport(Constants.GODLLPATH)> _
'    Public Function GoProfilePosition_ZMeasurement(ByVal tool As IntPtr) As IntPtr
'    End Function
'    <DllImport(Constants.GODLLPATH)> _
'    Public Function GoMeasurement_Enable(ByVal measurement As IntPtr, ByVal enable As Boolean) As Int32
'    End Function
'    <DllImport(Constants.GODLLPATH)> _
'    Public Function GoMeasurement_SetId(ByVal measurement As IntPtr, ByVal id As UInt32) As Int32
'    End Function
'    <DllImport(Constants.GODLLPATH)> _
'    Public Function GoProfilePosition_Feature(ByVal tool As IntPtr) As IntPtr
'    End Function
'    <DllImport(Constants.GODLLPATH)> _
'    Public Function GoProfileFeature_SetType(ByVal feature As IntPtr, ByVal featuretype As Int32) As Int32
'    End Function
'    <DllImport(Constants.GODLLPATH)> _
'    Public Function GoProfileFeature_Region(ByVal feature As IntPtr) As IntPtr
'    End Function
'    <DllImport(Constants.GODLLPATH)> _
'    Public Function GoProfileRegion_SetX(ByVal region As IntPtr, ByVal x As Double) As Int32
'    End Function
'    <DllImport(Constants.GODLLPATH)> _
'    Public Function GoProfileRegion_SetZ(ByVal region As IntPtr, ByVal z As Double) As Int32
'    End Function
'    <DllImport(Constants.GODLLPATH)> _
'    Public Function GoProfileRegion_SetHeight(ByVal region As IntPtr, ByVal height As Double) As Int32
'    End Function
'    <DllImport(Constants.GODLLPATH)> _
'    Public Function GoProfileRegion_SetWidth(ByVal region As IntPtr, ByVal width As Double) As Int32
'    End Function
'    <DllImport(Constants.GODLLPATH)> _
'    Public Function GoSetup_TransformedDataRegionX(ByVal setup As IntPtr, ByVal role As GoRole) As Double
'    End Function
'    <DllImport(Constants.GODLLPATH)> _
'    Public Function GoSetup_TransformedDataRegionZ(ByVal setup As IntPtr, ByVal role As GoRole) As Double
'    End Function
'    <DllImport(Constants.GODLLPATH)> _
'    Public Function GoSetup_TransformedDataRegionHeight(ByVal setup As IntPtr, ByVal role As GoRole) As Double
'    End Function
'    <DllImport(Constants.GODLLPATH)> _
'    Public Function GoSetup_TransformedDataRegionWidth(ByVal setup As IntPtr, ByVal role As GoRole) As Double
'    End Function
'    <DllImport(Constants.GODLLPATH)> _
'    Public Function GoSensor_Output(ByVal sensor As IntPtr) As IntPtr
'    End Function
'    <DllImport(Constants.GODLLPATH)> _
'    Public Function GoOutput_Ethernet(ByVal output As IntPtr) As IntPtr
'    End Function
'    <DllImport(Constants.GODLLPATH)> _
'    Public Function GoEthernet_ClearAllSources(ByVal ethernet As IntPtr) As Int32
'    End Function
'    <DllImport(Constants.GODLLPATH)> _
'    Public Function GoEthernet_AddSource(ByVal ethernet As IntPtr, ByVal GoOutputSource As Int32, ByVal sourceId As UInt32) As Int32
'    End Function
'    <DllImport(Constants.GODLLPATH)> _
'    Public Function GoMeasurementMsg_Count(ByVal msg As IntPtr) As UInt32
'    End Function
'    <DllImport(Constants.GODLLPATH)> _
'    Public Function GoMeasurementMsg_Id(ByVal msg As IntPtr) As UInt16
'    End Function
'    <DllImport(Constants.GODLLPATH)> _
'    Public Function GoMeasurementMsg_At(ByVal msg As IntPtr, ByVal index As UInt32) As IntPtr
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
'        Dim setup As IntPtr = IntPtr.Zero
'        Dim tools As IntPtr = IntPtr.Zero
'        Dim profilePositionTool As IntPtr = IntPtr.Zero
'        Dim zprofileMeasurementTop As IntPtr = IntPtr.Zero
'        Dim profileFeatureTop As IntPtr = IntPtr.Zero
'        Dim regionTop As IntPtr = IntPtr.Zero
'        Dim outputModule As IntPtr = IntPtr.Zero
'        Dim ethernetOutput As IntPtr = IntPtr.Zero
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

'        setup = GoSensor_Setup(sensor)

'        status = GoSystem_EnableData(system, True)
'        If status <> 1 Then
'            Console.WriteLine("GoSystem_EnableData Error:{0}", status)
'            Return status
'        End If

'        ' retrieve setup handle
'        setup = GoSensor_Setup(sensor)
'        ' retrieve tools handle
'        tools = GoSensor_Tools(sensor)
'        ' add ProfilePosition tool, retrieve tool handle
'        GoTools_AddTool(tools, GoToolType.GO_TOOL_PROFILE_POSITION, profilePositionTool)
'        ' set name for tool
'        GoTool_SetName(profilePositionTool, "Profile position Test")
'        ' add Z measurement for ProfilePosition tool
'        zprofileMeasurementTop = GoProfilePosition_ZMeasurement(profilePositionTool)

'        status = GoMeasurement_Enable(zprofileMeasurementTop, True)
'        If status <> 1 Then
'            Console.WriteLine("GoMeasurement_Enable Error:{0}", status)
'            Return status
'        End If

'        status = GoMeasurement_SetId(zprofileMeasurementTop, 0)
'        If status <> 1 Then
'            Console.WriteLine("GoMeasurement_SetId Error:{0}", status)
'            Return status
'        End If

'        ' set ProfilePosition feature to top
'        profileFeatureTop = GoProfilePosition_Feature(profilePositionTool)

'        status = GoProfileFeature_SetType(profileFeatureTop, 0)
'        If status <> 1 Then
'            Console.WriteLine("GoProfileFeature_SetType Error:{0}", status)
'            Return status
'        End If

'        ' set the ROI to fill the entire active area
'        regionTop = GoProfileFeature_Region(profileFeatureTop)
'        GoProfileRegion_SetX(regionTop, GoSetup_TransformedDataRegionX(setup, GoRole.GO_ROLE_MAIN))
'        GoProfileRegion_SetZ(regionTop, GoSetup_TransformedDataRegionZ(setup, GoRole.GO_ROLE_MAIN))
'        GoProfileRegion_SetHeight(regionTop, GoSetup_TransformedDataRegionHeight(setup, GoRole.GO_ROLE_MAIN))
'        GoProfileRegion_SetWidth(regionTop, GoSetup_TransformedDataRegionWidth(setup, GoRole.GO_ROLE_MAIN))

'        ' enable Ethernet output for measurement tool
'        outputModule = GoSensor_Output(sensor)
'        ethernetOutput = GoOutput_Ethernet(outputModule)
'        GoEthernet_ClearAllSources(ethernetOutput)
'        GoEthernet_AddSource(ethernetOutput, 8, 0)

'        ' refer to ReceiveMeasurement.c for receiving of the measurement data

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

'        'wait for ENTER key
'        Console.WriteLine("Press ENTER to continue")
'        Do While Not Console.ReadKey(True).Key = ConsoleKey.Enter
'            Thread.Sleep(100)
'        Loop
'        Return status
'    End Function
'End Module

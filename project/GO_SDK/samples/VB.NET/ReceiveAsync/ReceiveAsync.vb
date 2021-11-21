'ReceiveAsync.vb

'Gocator 2300 VB.NET Sample Code
'Copyright (C) 2013, 2018 by LMI Technologies Inc.

'Licensed under The MIT License.
'Redistributions of files must retain the above copyright notice.

'Purpose: Connect to Gocator system and receive data using a callback function
'Ethernet output for the desired data must be enabled.

Imports Lmi3d.GoSdk
Imports Lmi3d.GoSdk.Messages
Imports Lmi3d.Zen
Imports Lmi3d.Zen.Io

Module ReceiveAsync
    Public Const IP As String = "192.168.1.10"
    Public Const TIMEOUT As UInteger = 200000
    Public Const UM_TO_MM = 1000.0
    Public Const NM_TO_MM = 1000000.0

    Public Class DataContext
        Public xResolution As UInt32
        Public zResolution As UInt32
        Public xOffset As Int32
        Public zOffset As Int32
        Public serialNumber As UInteger
    End Class

    ' Data callback function
    ' This function is called from a separate thread spawned by the GoSDK library.
    ' Processing within this function should be minimal.
    Public Function onData(data As GoDataSet) As Integer
        Dim dataObj As GoDataMsg

        Console.CursorVisible = False
        Console.SetCursorPosition(0, 4)
        Console.WriteLine("Rx data - set({0})", data.Count())

        ' each result can have multiple data items, loop through all items in result message
        For i As Integer = 0 To data.Count() - 1
            dataObj = data.Get(i)

            ' retrieve GoStamp message
            Select Case dataObj.MessageType
                Case GoDataMessageType.Stamp
                    Dim stampMsg As GoStampMsg = dataObj
                    For j As Integer = 0 To stampMsg.Count - 1
                        Dim stamp As GoStamp = stampMsg.Get(j)
                        Console.WriteLine("Frame Index = {0}", stamp.FrameIndex)
                        Console.WriteLine("Time Stamp = {0}", stamp.Timestamp)
                        Console.WriteLine("Encoder Value = {0}", stamp.Encoder)
                    Next
                ' for different cases see corresponding examples
                Case GoDataMessageType.ProfilePointCloud
                    Console.WriteLine("Data P")

                Case GoDataMessageType.UniformProfile
                    Console.WriteLine("Data UP")

                Case GoDataMessageType.SurfacePointCloud
                    Console.WriteLine("Data S")

                Case GoDataMessageType.SurfaceIntensity
                    Console.WriteLine("Data SI")
            End Select
        Next
        Return KStatus.Ok
    End Function

    Function Main() As Integer

        Try
            Dim context As DataContext = New DataContext()
            Dim sensor As GoSensor = Nothing
            Dim myData As GoSystem.DataFx = New GoSystem.DataFx(AddressOf onData)

            KApiLib.Construct()
            GoSdkLib.Construct()
            Dim system As New GoSystem
            Dim ipAddress As KIpAddress = KIpAddress.Parse(IP)

            ' Connect to sensor
            sensor = system.FindSensorByIpAddress(ipAddress)
            sensor.Connect()
            sensor.EnableData(True)
            Console.WriteLine("GoSdk   Release : {0}", system.SdkVersion.Format())
            Console.WriteLine("Gocator Firmware: {0}", sensor.FirmwareVersion.Format())
            sensor.Stop()

            ' Set data handler
            system.SetDataHandler(myData)

            ' Start the sensor. After this call, the GoSDK library will start 
            ' receiving Data from the sensor. The data handler function will 
            ' be called by the SDK in a separate thread asynchonously.
            sensor.Start()

            'wait for ESC key
            Console.WriteLine("Press ESCAPE to stop")
            Do While Console.ReadKey(True).Key <> ConsoleKey.Escape
            Loop

            sensor.Stop()
            sensor.Destroy()

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

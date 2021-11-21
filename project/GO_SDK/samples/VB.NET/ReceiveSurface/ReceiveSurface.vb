'ReceiveSurface.vb

'Gocator 2000/2300 Visual Basic Sample
'Copyright (C) 2017,2018 by LMI Technologies Inc.

'Licensed under The MIT License.
'Redistributions of files must retain the above copyright notice.

'Purpose: Connect to Gocator system and receive Surface data 


Imports System.Threading
Imports Lmi3d.GoSdk
Imports Lmi3d.GoSdk.Messages
Imports Lmi3d.Zen
Imports Lmi3d.Zen.Io
Imports System.Runtime.InteropServices

Module ReceiveSurface
    Public Const IP As String = "192.168.1.10"
    Public Const TIMEOUT As UInteger = 30000000
    Public Const UM_TO_MM = 1000.0
    Public Const NM_TO_MM = 1000000.0

    Public Structure SurfacePoint
        Dim x As Double
        Dim y As Double
        Dim z As Double
        Dim intensity As Byte
    End Structure

    Function Main() As Integer
        Try
            KApiLib.Construct()
            GoSdkLib.Construct()

            Dim gsystem As New GoSystem
            Dim dataObj As GoDataMsg
            Dim gocator As GoSensor = Nothing
            Dim ipAddress As KIpAddress = KIpAddress.Parse(IP)

            gocator = gsystem.FindSensorByIpAddress(ipAddress)
            gocator.Connect()
            Console.WriteLine("GoSdk   Release : {0}", gsystem.SdkVersion.Format())
            Console.WriteLine("Gocator Firmware: {0}", gocator.FirmwareVersion.Format())
            Console.WriteLine()

            gocator.EnableData(True)
            gocator.Stop()

            If gocator.Setup.ScanMode() <> GoMode.Surface Then
                Console.WriteLine("Gocator is not set to scan Surface Mode...press <Esc> key")
                Do While Not Console.ReadKey(True).Key = ConsoleKey.Escape
                Loop
                Return KStatus.Error
                Exit Function
            End If

            gocator.Start()
            Console.WriteLine("Waiting for Surface Data...")

            ' read data from sensor
            Dim dataset As GoDataSet = gsystem.ReceiveData(TIMEOUT)
            Dim itemCount = dataset.Count()

            ' each result can have multiple data items
            ' loop through all items in result message
            For i As Integer = 0 To itemCount - 1
                dataObj = dataset.Get(i)

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

                    Case GoDataMessageType.UniformSurface
                        Dim surfaceMsg As GoUniformSurfaceMsg
                        surfaceMsg = dataObj
                        Dim length As Long = surfaceMsg.Length
                        Dim width As Long = surfaceMsg.Width
                        Console.WriteLine()
                        Console.WriteLine("Uniform Surface received:")
                        Console.WriteLine("Surface Width = {0} Length {1}", width, length)
                        Dim xOffset As Double = surfaceMsg.XOffset / UM_TO_MM
                        Dim yOffset As Double = surfaceMsg.YOffset / UM_TO_MM
                        Dim zOffset As Double = surfaceMsg.ZOffset / UM_TO_MM
                        Dim xResolution As Double = surfaceMsg.XResolution / NM_TO_MM
                        Dim yResolution As Double = surfaceMsg.YResolution / NM_TO_MM
                        Dim zResolution As Double = surfaceMsg.ZResolution / NM_TO_MM
                        Dim surface(length, width) As SurfacePoint
                        For j As Integer = 0 To surfaceMsg.Length - 1
                            For k As Integer = 0 To surfaceMsg.Width - 1
                                surface(j, k).x = xOffset + k * xResolution
                                surface(j, k).y = yOffset + j * yResolution
                                surface(j, k).z = zOffset + surfaceMsg.Get(j, k) * zResolution
                            Next
                        Next

                    Case GoDataMessageType.SurfacePointCloud
                        Dim surfaceMsg As GoSurfacePointCloudMsg
                        surfaceMsg = dataObj
                        Dim length As Long = surfaceMsg.Length
                        Dim width As Long = surfaceMsg.Width
                        Console.WriteLine()
                        Console.WriteLine("Surface Point Cloud received:")
                        Console.WriteLine("Surface Width = {0} Length {1}", width, length)
                        Dim xOffset As Double = surfaceMsg.XOffset / UM_TO_MM
                        Dim yOffset As Double = surfaceMsg.YOffset / UM_TO_MM
                        Dim zOffset As Double = surfaceMsg.ZOffset / UM_TO_MM
                        Dim xResolution As Double = surfaceMsg.XResolution / NM_TO_MM
                        Dim yResolution As Double = surfaceMsg.YResolution / NM_TO_MM
                        Dim zResolution As Double = surfaceMsg.ZResolution / NM_TO_MM

                        Dim bufferSize As Long = width * length
                        Dim dataPoints(bufferSize) As Data.KPoint3d16s
                        Dim surfacePoints(bufferSize) As Data.KPoint3d64f
                        Dim structSize As Long = Marshal.SizeOf(GetType(Data.KPoint3d16s))
                        Dim pointsPtr As IntPtr = surfaceMsg.Data
                        For j As Integer = 0 To bufferSize - 1
                            Dim incPtr As IntPtr = pointsPtr.ToInt64() + j * structSize
                            dataPoints(j) = Marshal.PtrToStructure(incPtr, GetType(Data.KPoint3d16s))
                            surfacePoints(j).X = dataPoints(j).X * xResolution + xOffset
                            surfacePoints(j).Y = dataPoints(j).Y * yResolution + yOffset
                            surfacePoints(j).Z = dataPoints(j).Z * zResolution + zOffset
                        Next
                End Select
            Next

            dataset.Destroy()
            gocator.Stop()

            Console.WriteLine(Environment.NewLine)
            'wait for ESC key
            Console.WriteLine("Press ESCAPE to Exit")
            Do While Not Console.ReadKey(True).Key = ConsoleKey.Escape
            Loop
            gocator.Destroy()
        Catch ex As Exception
            Console.WriteLine("Error {0}.....Press ESCAPE to Exit", ex.ToString())
            Do While Not Console.ReadKey(True).Key = ConsoleKey.Escape
            Loop
            Return KStatus.Error
            Exit Function
        End Try
        Return KStatus.Ok
    End Function

End Module
'ReceiveMeasurement.vb

'Gocator 2300 VB.NET Sample
'Copyright (C) 2017,2018 by LMI Technologies Inc.

'Licensed under The MIT License.
'Redistributions of files must retain the above copyright notice.

'Purpose: Connect to Gocator system, setup and receive measurement data under Profile Mode. Measurements should have been setup and have the outputs enabled via Ethernet.
'Please refer to SetupMeasurement.c for configuring the measurements.

Imports Lmi3d.GoSdk
Imports Lmi3d.GoSdk.Messages
Imports Lmi3d.Zen
Imports Lmi3d.Zen.Io
Imports Lmi3d.GoSdk.Tools

Module ReceiveMeasurement
    Public Const IP As String = "192.168.1.10"
    Public Const TIMEOUT As UInteger = 100000

    Function Main() As Integer
        Try
            KApiLib.Construct()
            GoSdkLib.Construct()

            Dim dataObj As GoDataMsg
            Dim gsystem As New GoSystem
            Dim gocator As GoSensor = Nothing

            Dim collection_tools As GoTools = Nothing
            Dim ipAddress As KIpAddress = KIpAddress.Parse(IP)

            gocator = gsystem.FindSensorByIpAddress(ipAddress)

            gocator.Connect()
            gocator.EnableData(True)
            gocator.Stop()

            Console.WriteLine("GoSdk   Release : {0}", gsystem.SdkVersion.Format())
            Console.WriteLine("Gocator Firmware: {0}", gocator.FirmwareVersion.Format())
            Console.WriteLine()

            gocator.Start()
            'retrieve tools handle
            collection_tools = gocator.Tools()

            ' read data from sensor
            Dim dataset As GoDataSet = gsystem.ReceiveData(TIMEOUT)
            Dim itemCount = dataset.Count()
            If (itemCount > 0) Then
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

                        Case GoDataMessageType.Measurement
                            Dim measurementMsg As GoMeasurementMsg
                            Dim measurement As GoMeasurement

                            measurementMsg = dataObj
                            For j As Integer = 0 To measurementMsg.Count - 1
                                Dim measurementData As GoMeasurementData = measurementMsg.Get(j)
                                measurement = collection_tools.FindMeasurementById(measurementMsg.Id)
                                Console.WriteLine("Measurement Name is : {0}", measurement.Name)
                                Console.WriteLine("ID: {0}", measurementMsg.Id)
                                Console.WriteLine("Value: {0}", measurementData.Value)
                                Console.WriteLine("Decision: {0}", measurementData.Decision)
                            Next
                    End Select
                Next
            End If
            dataset.Destroy()
            gocator.Stop()
            Console.WriteLine()
            'wait for ESC key
            Console.WriteLine("Press ESCAPE to Exit")
            Do While Not Console.ReadKey(True).Key = ConsoleKey.Escape
            Loop
            gocator.Destroy()
        Catch ex As Exception
            Console.WriteLine("Error: {0}.....Press ESCAPE to Exit", ex.ToString())
            Do While Not Console.ReadKey(True).Key = ConsoleKey.Escape
            Loop
            Return KStatus.Error
            Exit Function
        End Try
        Return KStatus.Ok
    End Function

End Module
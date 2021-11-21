'AcceleratorReceiveMeasurement.vb

'Gocator 2300 VB.NET Sample
'Copyright (C) 2017,2018 by LMI Technologies Inc.

'Licensed under The MIT License.
'Redistributions of files must retain the above copyright notice.

' Purpose: Demonstrates the simple use of the Accelerator by connecting to a sensor and receiving a measurement.
' This allows processing to be performed on the PC rather than on the sensor.

Imports Lmi3d.GoSdk
Imports Lmi3d.GoSdk.Messages
Imports Lmi3d.Zen
Imports Lmi3d.Zen.Io

Module AcceleratorReceiveMeasurement
    Public Const IP As String = "192.168.1.10"
    Public Const WEBPORT As UInteger = 8081             ' specify web port to use if default 8080 is not availabe
    Public Const TIMEOUT As UInteger = 100000

    Function Main() As Integer

        Try
            KApiLib.Construct()
            GoSdkLib.Construct()

            Dim dataObj As GoDataMsg
            Dim gsystem As New GoSystem
            Dim accelerator As New GoAccelerator
            Dim ipAddress As KIpAddress = KIpAddress.Parse(IP)
            Dim gocator As GoSensor = Nothing

            accelerator.Start()
            gocator = gsystem.FindSensorByIpAddress(ipAddress)
            ' in case default web port 8080 is not available, use any free one
            accelerator.WebPort = WEBPORT

            accelerator.Attach(gocator)
            gocator.Connect()
            gocator.EnableData(True)

            Console.WriteLine("GoSdk   Release : {0}", gsystem.SdkVersion.Format())
            Console.WriteLine("Gocator Firmware: {0}", gocator.FirmwareVersion.Format())
            Console.WriteLine(Environment.NewLine)

            gocator.Start()

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
                            measurementMsg = dataObj
                            For j As Integer = 0 To measurementMsg.Count - 1
                                Dim measurementData As GoMeasurementData = measurementMsg.Get(j)
                                Console.WriteLine("ID: {0}", measurementMsg.Id)
                                Console.WriteLine("Value: {0}", measurementData.Value)
                                Console.WriteLine("Decision: {0}", measurementData.Decision)
                            Next
                    End Select
                Next
            End If

            gocator.Stop()
            accelerator.Detach(gocator)
            Console.WriteLine("Press ESCAPE to Exit")
            Do While Not Console.ReadKey(True).Key = ConsoleKey.Escape
            Loop
            accelerator.Destroy()
            gocator.Destroy()
        Catch ex As Exception
            Console.WriteLine("{0}", ex.ToString())
            Console.WriteLine("Press ESCAPE to Exit")
            Do While Not Console.ReadKey(True).Key = ConsoleKey.Escape
            Loop
            Return KStatus.Error
            Exit Function
        End Try
        Return KStatus.Ok
    End Function

End Module
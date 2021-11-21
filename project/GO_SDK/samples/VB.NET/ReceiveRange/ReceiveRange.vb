'ReceiveRange.vb

'Gocator 2300 VB.NET Sample
'Copyright (C) 2017,2018 by LMI Technologies Inc.

'Licensed under The MIT License.
'Redistributions of files must retain the above copyright notice.

'Purpose: Connect to Gocator sensor and receive range data in Range Mode and translate to engineering units (mm)


Imports Lmi3d.GoSdk
Imports Lmi3d.GoSdk.Messages

Module ReceiveRange
    Public Const IP As String = "192.168.1.10"
    Public Const TIMEOUT As UInteger = 100000
    Public Const NM_TO_MM = 1000000
    Public Const OUT_OF_RANGE = -32768

    Function Main() As Integer
        Try
            KApiLib.Construct()
            GoSdkLib.Construct()
            Dim gsystem As New GoSystem
            Dim gocator As GoSensor = Nothing
            Dim ipAddress As KIpAddress = KIpAddress.Parse(IP)
            Dim dataObj As GoDataMsg

            gocator = gsystem.FindSensorByIpAddress(ipAddress)
            gocator.Connect()
            Console.WriteLine("GoSdk   Release : {0}", gsystem.SdkVersion.Format())
            Console.WriteLine("Gocator Firmware: {0}", gocator.FirmwareVersion.Format())

            gocator.EnableData(True)
            gocator.Stop()
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

                        Case GoDataMessageType.Range
                            Dim rangeMsg As GoRangeMsg = dataObj
                            Dim ranges(rangeMsg.Count) As Double
                            Console.WriteLine()
                            For n As Integer = 0 To rangeMsg.Count - 1
                                Dim raw As Integer = rangeMsg.Get(n)
                                If raw <> OUT_OF_RANGE Then
                                    ranges(n) = (rangeMsg.ZOffset + rangeMsg.ZResolution * rangeMsg.Get(n)) / NM_TO_MM
                                    Console.WriteLine("Range     : {0:N3} mm", ranges(n))
                                Else
                                    ranges(n) = OUT_OF_RANGE
                                    Console.WriteLine("Range     : Out of Range {0}", OUT_OF_RANGE)
                                End If

                            Next

                        Case GoDataMessageType.RangeIntensity
                            Dim rangeIntensityMsg As GoRangeIntensityMsg = dataObj
                            Dim intensity(rangeIntensityMsg.Count) As Byte
                            For j As Integer = 0 To rangeIntensityMsg.Count - 1
                                intensity(j) = rangeIntensityMsg.Get(j)
                                Console.WriteLine("Intensity : {0}", intensity(j))
                            Next
                    End Select
                Next
            End If
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

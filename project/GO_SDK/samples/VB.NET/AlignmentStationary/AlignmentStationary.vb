'AlignmentStationary.vb

'Gocator 2300 VB.NET Sample
'Copyright (C) 2017,2018 by LMI Technologies Inc.

'Licensed under The MIT License.
'Redistributions of files must retain the above copyright notice.

'Purpose: Connect to Gocator system and perform stationary alignment to a flat surface

Imports Lmi3d.GoSdk
Imports Lmi3d.GoSdk.Messages
Imports Lmi3d.Zen
Imports Lmi3d.Zen.Io

Module AlignmentStationary
    Public Const IP As String = "192.168.1.10"
    Public Const TIMEOUT As UInteger = 30000000

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
            gocator.EnableData(True)

            Console.WriteLine("GoSdk   Release : {0}", gsystem.SdkVersion.Format())
            Console.WriteLine("Gocator Firmware: {0}", gocator.FirmwareVersion.Format())
            Console.WriteLine(Environment.NewLine)
            gocator.Stop()
            gocator.Setup.AlignmentType = GoAlignmentType.Stationary
            gocator.Setup.AlignmentStationaryTarget = GoAlignmentTarget.None

            gocator.Align()
            Console.Write("Waiting for alignment calibration data to be collected...")

            ' read data from sensor
            Dim dataset As GoDataSet = gsystem.ReceiveData(TIMEOUT)
            If dataset.Count() > 0 Then
                ' each result can have multiple data items
                ' loop through all items in result message
                For i As Integer = 0 To dataset.Count() - 1
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

                        Case GoDataMessageType.Alignment
                            Dim alignmentMsg As GoAlignMsg
                            alignmentMsg = dataObj
                            If alignmentMsg.Status = KStatus.Ok Then
                                Console.Write("Alignment Successfull.")
                            Else
                                Console.Write("Alignment Failed.")
                            End If
                            Console.WriteLine()
                        Case Else
                            Exit Select

                    End Select
                Next
            End If
            dataset.Destroy()

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

'ReceiveProfile.vb

'Gocator 2300 VB.NET Sample Code
'Copyright (C) 2013, 2018 by LMI Technologies Inc.

'Licensed under The MIT License.
'Redistributions of files must retain the above copyright notice.

'Purpose: Connect to Gocator sensor and receive profile data under Profile Mode.
'         Make sure profile Top output is enabled

Imports Lmi3d.GoSdk
Imports Lmi3d.GoSdk.Messages
Imports Lmi3d.Zen
Imports Lmi3d.Zen.Io

Module ReceiveProfile
    Public Const IP As String = "192.168.1.10"
    Public Const TIMEOUT As UInteger = 100000
    Public Const OUT_OF_RANGE = -32768

    Function Main() As Integer
        Try
            Dim dataObj As GoDataMsg
            Dim gocator As GoSensor = Nothing

            KApiLib.Construct()
            GoSdkLib.Construct()

            Dim gsystem As New GoSystem

            Dim ipAddress As KIpAddress = KIpAddress.Parse(IP)
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

                        Case GoDataMessageType.Profile Or GoDataMessageType.ProfilePointCloud
                            Dim profileMsg As GoProfilePointCloudMsg = dataObj
                            Dim profilePoints(profileMsg.Count, profileMsg.Width) As Data.KPoint16s
                            For j As Integer = 0 To profileMsg.Count - 1
                                For k As Integer = 0 To profileMsg.Width - 1
                                    profilePoints(j, k) = profileMsg.Get(k)
                                    If profilePoints(j, k).Y <> OUT_OF_RANGE Then
                                        ' show only valid values
                                        Console.WriteLine("Profile[{0}][{1}] X {2} Z {3}", j, k, profilePoints(j, k).X, profilePoints(j, k).Y)
                                    End If
                                Next
                            Next

                        Case GoDataMessageType.UniformProfile Or GoDataMessageType.ResampledProfile
                            Dim profileResampledMsg As GoUniformProfileMsg = dataObj
                            Dim profilesCount As Long = profileResampledMsg.Count
                            Dim profileWidth As Short = profileResampledMsg.Width
                            Dim profilePoints(profilesCount, profileWidth) As Int16
                            For j As Integer = 0 To profilesCount - 1
                                For k As Integer = 0 To profileWidth - 1
                                    profilePoints(j, k) = profileResampledMsg.Get(k)
                                    If profilePoints(j, k) <> OUT_OF_RANGE Then
                                        Console.WriteLine("Profile[{0}][{1}]  {2} ", j, k, profilePoints(j, k))
                                    End If
                                Next
                            Next

                        Case GoDataMessageType.ProfileIntensity
                            Dim profileIntensityMsg As GoProfileIntensityMsg
                            profileIntensityMsg = dataObj
                            Dim profilesCount As Long = profileIntensityMsg.Count
                            Dim profileWidth As Integer = profileIntensityMsg.Width
                            Dim profileIntensities(profilesCount, profileWidth) As Byte
                            For j As Long = 0 To profilesCount - 1
                                For k As Long = 0 To profileWidth - 1
                                    profileIntensities(j, k) = profileIntensityMsg.Get(j, k)
                                    If profileIntensities(j, k) > 0 Then
                                        Console.WriteLine("Profile[{0}][{1}] Intensity {2}", j, k, profileIntensities(j, k))
                                    End If
                                Next
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
            Console.WriteLine("Error: {0}.....Press ESCAPE to Exit", ex.ToString())
            Do While Not Console.ReadKey(True).Key = ConsoleKey.Escape
        Loop
        Return KStatus.Error
        Exit Function
        End Try
        Return KStatus.Ok
    End Function

End Module
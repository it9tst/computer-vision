'ReceiveHealth.vb

'Gocator 2300 VB.NET Sample Code
'Copyright (C) 2013, 2018 by LMI Technologies Inc.

'Licensed under The MIT License.
'Redistributions of files must retain the above copyright notice.

'Purpose: Connect to Gocator sensor and receive health data

Imports Lmi3d.GoSdk
Imports Lmi3d.GoSdk.Messages
Imports Lmi3d.Zen
Imports Lmi3d.Zen.Io

Module ReceiveHealth
    Public Const IP As String = "192.168.1.10"

    Public Function onHealth(data As GoDataSet) As Integer
        Dim healthMsg As GoHealthMsg
        Dim indicator As GoIndicator

        Console.CursorVisible = False
        Console.SetCursorPosition(0, 4)
        Console.WriteLine("Dataset Count = {0}", data.Count())

        ' each result can have multiple data items, loop through all items in result message
        For i As Integer = 0 To data.Count() - 1
            healthMsg = data.Get(i)
            For k As Integer = 0 To healthMsg.Count - 1
                indicator = healthMsg.Get(k)
                If indicator.id.ToString() = "Uptime" Then
                    Console.WriteLine(" indicator[{0}]: value:{1} ", indicator.id.ToString(), indicator.value)
                End If
            Next
        Next
        Return KStatus.Ok
    End Function


    Function Main() As Integer

        Try
            Dim gocator As GoSensor = Nothing
            Dim myHealth As GoSystem.DataFx = New GoSystem.DataFx(AddressOf onHealth)

            KApiLib.Construct()
            GoSdkLib.Construct()
            Dim gsystem As New GoSystem
            Dim ipAddress As KIpAddress = KIpAddress.Parse(IP)


            gocator = gsystem.FindSensorByIpAddress(ipAddress)

            gocator.Connect()
            gocator.EnableData(True)

            Console.WriteLine("GoSdk   Release : {0}", gsystem.SdkVersion.Format())
            Console.WriteLine("Gocator Firmware: {0}", gocator.FirmwareVersion.Format())

            gocator.Stop()

            ' setup callback function
            gsystem.SetHealthHandler(myHealth)

            'wait for ESC key
            Console.WriteLine("Press ESCAPE to stop")
            Do While Console.ReadKey(True).Key <> ConsoleKey.Escape
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

'Configure.vb

'Gocator 2000/2300 Visual Basic Sample
'Copyright (C) 2013 by LMI Technologies Inc.

'Licensed under The MIT License.
'Redistributions of files must retain the above copyright notice.

'Purpose: Connect to Gocator system and modify parameters

Imports Lmi3d.GoSdk
Imports Lmi3d.Zen
Imports Lmi3d.Zen.Io

Module Configure
    Public Const IP As String = "192.168.1.10"
    Public Const NEW_EXPOSURE = 375.25              'us

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

            Dim current_Exposure As Double = gocator.Setup.GetExposure(GoRole.Main)
            Console.WriteLine("Original Exposure: {0}", current_Exposure)

            ' modify parameter in main sensor - exposure
            gocator.Setup.SetExposure(GoRole.Main, NEW_EXPOSURE)
            Console.WriteLine("Set New Exposure: {0}", NEW_EXPOSURE)

            ' NOTE: Sensor is not automatically synchronized with every call to function that modifies a setting.
            ' This allows for rapid configuring sensors without delay caused by synchronization after every call.
            gocator.Flush()

            ' verify new exposure settings
            Console.WriteLine("Verify New Exposure: {0}", gocator.Setup.GetExposure(GoRole.Main))
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

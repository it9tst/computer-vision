'SwapFiles.vb

'Gocator 2300 VB.NET Sample Code
'Copyright (C) 2013, 2018 by LMI Technologies Inc.

'Licensed under The MIT License.
'Redistributions of files must retain the above copyright notice.

'Purpose: Connect to Gocator sensor, list resident files
'         download _live.tfm file To the PC
'         Upload it back to the sensor
'         Find out name of currently loaded job file 

Imports Lmi3d.GoSdk
Imports Lmi3d.Zen
Imports Lmi3d.Zen.Io

Module SwapFiles
    Public Const IP As String = "192.168.1.10"
    Public Const TRANSFORMATION_FILENAME As String = ".\transformations.tfm"

    Function Main() As Integer
        Dim gocator As GoSensor = Nothing
        Dim filename As String = "_live.tfm"

        KApiLib.Construct()
        GoSdkLib.Construct()
        Dim gsystem As New GoSystem
        Dim ipAddress As KIpAddress = KIpAddress.Parse(IP)
        Try
            gocator = gsystem.FindSensorByIpAddress(ipAddress)
            gocator.Connect()
            gocator.EnableData(True)
            Console.WriteLine("GoSdk   Release : {0}", gsystem.SdkVersion.Format())
            Console.WriteLine("Gocator Firmware: {0}", gocator.FirmwareVersion.Format())

            gocator.Stop()

            ' now list the files in the sensor
            Console.WriteLine("list of files:")
            For i As Integer = 0 To gocator.FileCount - 1
                Console.WriteLine("{0}", gocator.GetFileName(i))
            Next

            gocator.DownloadFile(filename, TRANSFORMATION_FILENAME)
            Console.WriteLine("downloaded _live.tfm transformation file to {0}", TRANSFORMATION_FILENAME)
            ' now upload the file
            gocator.UploadFile(TRANSFORMATION_FILENAME, filename)
            Console.WriteLine("uploaded {0} to gocator _live.tfm transformation file", TRANSFORMATION_FILENAME)

            ' find out which job is currently loaded
            Dim changed As KBool
            gocator.LoadedJob(filename, changed)
            Console.WriteLine("loaded job: {0}", filename)

        Catch ex As Exception
            Console.WriteLine("Exeption {0}", ex.Message)
        Finally
            Console.WriteLine("Press ESCAPE to Exit")
            Do While Not Console.ReadKey(True).Key = ConsoleKey.Escape
            Loop
        End Try

        Return KStatus.Ok
    End Function

End Module

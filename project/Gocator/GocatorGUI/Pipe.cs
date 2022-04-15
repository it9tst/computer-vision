using System;
using Newtonsoft.Json;
using System.IO;
using System.Windows.Media.Media3D;
using System.Collections.Generic;
using System.Data;
using System.IO.Pipes;
using System.Threading;
using System.Collections.Concurrent;
using Newtonsoft.Json.Linq;

namespace GocatorGUI {

    public class Pipe {

        private MainWindow mainWindow;
        private BlockingCollection<string> cq = new BlockingCollection<string>(new ConcurrentQueue<string>());

        public Pipe() {
            
        }

        public void PipeRead(MainWindow mainWindow) {

            this.mainWindow = mainWindow;

            Thread thread = new Thread(delegate () {
                Update();
            });
            thread.Start();

            while (true) {
                var namedPipeServer = new NamedPipeServerStream("gocator-pipe", PipeDirection.InOut, 1, PipeTransmissionMode.Byte);
                var streamReader = new StreamReader(namedPipeServer);
                namedPipeServer.WaitForConnection();

                namedPipeServer.WaitForPipeDrain();

                string lineContent = streamReader.ReadLine();

                Console.WriteLine($"read from pipe client: {System.Text.ASCIIEncoding.UTF8.GetByteCount(lineContent)} bytes");

                cq.Add(lineContent);

                namedPipeServer.Dispose();
            }
        }

        public void Update() {

            while (true) {

                string lineContent = cq.Take();

                var jsonObject = JObject.Parse(lineContent);

                foreach (var tmp in jsonObject) {
                    if (tmp.Key == "point_cloud") {

                        int id = (int)jsonObject["point_cloud"]["id"];
                        Console.WriteLine($"id point_cloud: {id}");

                        DataTable dataTable = jsonObject["point_cloud"]["points"].ToObject<DataTable>();
                        
                        List<Point3D> points = new List<Point3D>();

                        foreach (DataRow row in dataTable.Rows) {
                            points.Add(new Point3D(Convert.ToDouble(row["x"]), Convert.ToDouble(row["y"]), Convert.ToDouble(row["z"])));
                            //Console.WriteLine(row["x"] + " - " + row["y"] + " - " + row["z"]);
                        }

                        this.mainWindow.Dispatcher.Invoke(() => {
                            this.mainWindow.AddPCL(points, id);
                        });

                    } else if (tmp.Key == "stats") {

                        int id = (int)jsonObject["stats"]["id"];
                        Console.WriteLine($"id stats: {id}");

                        DataTable dataTable = jsonObject["stats"]["rows"].ToObject<DataTable>();

                        Console.WriteLine($"number of rows: {dataTable.Rows.Count}");

                        foreach (DataRow row in dataTable.Rows) {

                            string sent = row["row"].ToString();

                            this.mainWindow.Dispatcher.Invoke(() => {
                                this.mainWindow.AddRow(sent, id);
                            });
                        }
                    }
                }
            }
        }
    }
}

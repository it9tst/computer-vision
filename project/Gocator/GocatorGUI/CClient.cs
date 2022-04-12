using System;
using System.Net;
using System.Net.Sockets;
using System.Text;
using Newtonsoft.Json;
using System.IO;
using System.Windows.Media.Media3D;
using System.Collections.Generic;
using System.Data;

namespace GocatorGUI {

    public class CClient {

        private MainWindow mainWindow;

        public CClient() {
            
        }

        // The response from the remote device.  
        public void ClientStart(MainWindow mainWindow) {

            this.mainWindow = mainWindow;

            Socket socket;

            IPAddress ip = Dns.GetHostEntry("127.0.0.1").AddressList[0];
            int port = 7200;
            IPEndPoint remoteEP = new IPEndPoint(ip, port);

            socket = new Socket(AddressFamily.InterNetwork, SocketType.Stream, ProtocolType.Tcp);

            bool isConnected = false;
            
            while (!isConnected) {
                try {
                    socket.Connect(remoteEP);
                    Console.WriteLine("Client connected to {0}", socket.RemoteEndPoint.ToString());
                    isConnected = true;
                } catch (Exception e1) {
                    Console.WriteLine("SERVER: Exception: " + e1);
                    System.Threading.Thread.Sleep(2000);
                }
            }

            while (true) {
                try {
                    using (var networkStream = new NetworkStream(socket))
                    using (var reader = new StreamReader(networkStream, Encoding.Unicode, true))
                    using (var jsonReader = new JsonTextReader(reader)) {
                        Console.WriteLine("sono qui nel client");
                        var serializer = new JsonSerializer();
                        var result = serializer.Deserialize(jsonReader);

                        DataSet dataSet = JsonConvert.DeserializeObject<DataSet>(result.ToString());

                        if (dataSet.Tables.Contains("point_cloud")) {

                            DataTable dataTable = dataSet.Tables["point_cloud"];
                            Console.WriteLine(dataTable.Rows.Count);

                            List<Point3D> points = new List<Point3D>();

                            foreach (DataRow row in dataTable.Rows) {
                                points.Add(new Point3D(Convert.ToDouble(row["x"]), Convert.ToDouble(row["y"]), Convert.ToDouble(row["z"])));
                                //Console.WriteLine(row["x"] + " - " + row["y"] + " - " + row["z"]);
                            }

                            this.mainWindow.Dispatcher.Invoke(() => {
                                this.mainWindow.CreateModel(points);
                            });
                        } else if (dataSet.Tables.Contains("stats")) {
                            DataTable dataTable = dataSet.Tables["stats"];
                            Console.WriteLine(dataTable.Rows.Count);

                            foreach (DataRow row in dataTable.Rows) {

                                string sent = row["row"].ToString();
                                Console.WriteLine(sent);

                                this.mainWindow.Dispatcher.Invoke(() => {
                                    this.mainWindow.textBoxOutput.AppendText(sent);
                                    this.mainWindow.textBoxOutput.AppendText(Environment.NewLine);
                                });
                            }
                        }
                    }
                } catch (ObjectDisposedException) {
                    Console.WriteLine("SERVER: server was closed");
                } catch (SocketException e) {
                    Console.WriteLine("SERVER: Exception: " + e);
                }
            }
        }
    }
}

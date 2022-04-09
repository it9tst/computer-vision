using System;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using System.Text;

namespace GocatorGUI {

    public class CClient {

        public CClient() {
            
        }

        // The response from the remote device.  
        public void StartClient() {

            Socket s;

            IPAddress ip = Dns.GetHostEntry("127.0.0.1").AddressList[0];
            int port = 7200;
            IPEndPoint remoteEP = new IPEndPoint(ip, port);

            s = new Socket(AddressFamily.InterNetwork, SocketType.Stream, ProtocolType.Tcp);

            bool isConnected = false;
            while (!isConnected) {
                try {
                    s.Connect(remoteEP);
                    Console.WriteLine("Client connected to {0}", s.RemoteEndPoint.ToString());
                    isConnected = true;
                } catch (Exception e1) {
                    System.Threading.Thread.Sleep(2000);
                }
            }

            byte[] bytes = new byte[1024];

            while (true) {
                // Receive the response from the remote device.  
                int bytesRec = s.Receive(bytes);
                Console.WriteLine("Echoed test = {0}", Encoding.ASCII.GetString(bytes, 0, bytesRec));
            }
        }
    }
}

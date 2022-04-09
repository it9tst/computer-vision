#pragma comment(lib, "ws2_32.lib")

#include <iostream>
#include <WinSock2.h>
#include <thread>
#include <vector>

namespace GocatorCV {
	
	class Server {

	private:
		SOCKET server, client;
		std::thread acquisition_thread;

	public:
		Server();
		void ServerStart();
		void Serv();
		void Send(std::vector<double> s);
	};
}

#pragma comment(lib, "ws2_32.lib")

#ifndef SERVER_H
#define SERVER_H

#include <iostream>
#include <WinSock2.h>
#include <thread>
#include <vector>
#include <cpprest/json.h>

namespace GocatorCV {

	struct PCL {
		std::vector<double> x;
		std::vector<double> y;
		std::vector<double> z;
	};

	struct Statistics {
		std::vector<std::string> row;
	};

	class Server {

	private:
		SOCKET server, client;
		std::thread server_thread;

	public:
		Server();
		void ServerInit();
		void ServerStart();
		void ServerStop();
		void SendPCL(GocatorCV::PCL pcl);
		void SendStats(GocatorCV::Statistics stats);
	};
}

#endif
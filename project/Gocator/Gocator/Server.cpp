#include "Server.h"

GocatorCV::Server::Server() {
	// constructor
}

void GocatorCV::Server::ServerStart() {

	acquisition_thread = std::thread(&Server::Serv, this);
}

void GocatorCV::Server::Serv() {
	WSADATA WSAData;
	SOCKADDR_IN serverAddr, clientAddr;

	int wsOk = WSAStartup(MAKEWORD(2, 0), &WSAData);
	if (wsOk != 0) {
		std::cerr << "Can't Initialize winsock! Quitting" << std::endl;
		return;
	}

	server = socket(AF_INET, SOCK_STREAM, 0);
	if (server == INVALID_SOCKET) {
		std::cerr << "Can't create a socket! Quitting" << std::endl;
		return;
	}

	serverAddr.sin_family = AF_INET;
	serverAddr.sin_port = htons(7200);
	serverAddr.sin_addr.s_addr = INADDR_ANY;

	bind(server, (SOCKADDR*)&serverAddr, sizeof(serverAddr));
	listen(server, 0);

	std::cout << "Listening for incoming connections..." << std::endl;

	int clientAddrSize = sizeof(clientAddr);
	if ((client = accept(server, (SOCKADDR*)&clientAddr, &clientAddrSize))) {
		std::cout << "Client connected!" << std::endl;
	}
}

void GocatorCV::Server::Send(std::vector<double> s) {

	//send(client, s, s.size()*sizeof(double), 0);

	/*
	const char* prova = "il mio invio";
	const char* prova2 = "il mio secondo invio";

	send(client, prova, strlen(prova), 0);
	std::this_thread::sleep_for(std::chrono::milliseconds(5000));
	send(client, prova2, strlen(prova2), 0);
	*/

	//closesocket(client);
	//std::cout << "Client disconnected." << std::endl;
}

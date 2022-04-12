#include "Server.h"

GocatorCV::Server::Server() {
	// constructor
}

void GocatorCV::Server::ServerInit() {

	server_thread = std::thread(&Server::ServerStart, this);
}

void GocatorCV::Server::ServerStart() {
	
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

void GocatorCV::Server::ServerStop() {
	
	closesocket(client);
	std::cout << "Client disconnected." << std::endl;
	
	server_thread.join();
}

void GocatorCV::Server::SendPCL(GocatorCV::PCL pcl) {

	std::vector<web::json::value> arrayPoints;

	for (int i = 0; i < pcl.x.size(); i++) {
		web::json::value point;
		point[L"x"] = web::json::value::number(pcl.x[i]);
		point[L"y"] = web::json::value::number(pcl.y[i]);
		point[L"z"] = web::json::value::number(pcl.z[i]);
		arrayPoints.push_back(point);
	}

	web::json::value myJSON;
	myJSON[L"point_cloud"] = web::json::value::array(arrayPoints);

	//std::wcout << myJSON.serialize() << std::endl;

	auto result = myJSON.serialize();

	send(client, (const char*)result.c_str(), result.size() * sizeof(wchar_t), 0);
	std::cout << "Ho inviato dal server: " << result.size() * sizeof(wchar_t) << std::endl;
}

void GocatorCV::Server::SendStats(GocatorCV::Statistics stats) {

	std::vector<web::json::value> arrayStats;

	for (int i = 0; i < stats.row.size(); i++) {
		web::json::value row;
		row[L"row"] = web::json::value::string(utility::conversions::to_utf16string(stats.row[i]));
		arrayStats.push_back(row);
	}

	web::json::value myJSON;
	myJSON[L"stats"] = web::json::value::array(arrayStats);

	//std::wcout << myJSON.serialize() << std::endl;

	auto result = myJSON.serialize();

	send(client, (const char*)result.c_str(), result.size() * sizeof(wchar_t), 0);
	std::cout << "Ho inviato dal server: " << result.size() * sizeof(wchar_t) << std::endl;
}
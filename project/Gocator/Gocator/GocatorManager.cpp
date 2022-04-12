#include "GocatorManager.h"

GocatorCV::GocatorManager::GocatorManager() {
	GocatorCV::Gocator gocator;
	GocatorCV::Process process;
	GocatorCV::Analysis analysis;
	GocatorCV::Error error;
	GocatorCV::Server server;
}

GocatorCV::GocatorManager::~GocatorManager() {
	server.ServerStop();
}

void GocatorCV::GocatorManager::ServerStart() {

	//server_thread = std::thread(&Server::ServerStart, server);
	server.ServerInit();
}

void GocatorCV::GocatorManager::SetParameter(char* str, int strlen, const char* param, int type) {

	std::string message = "OK";

	if (type == 1) {

		if ((error = gocator.SetParameter(GocatorCV::ParameterType::SENSOR_IP, (void*)param)).GetCode() != GocatorCV::ErrorType::OK) {
			message = error.DisplayMessage();
		}
	} else if (type == 2) {

		k64f exposure = std::atof(param);

		if ((error = gocator.SetParameter(GocatorCV::ParameterType::EXPOSURE, (void*)&exposure)).GetCode() != GocatorCV::ErrorType::OK) {
			message = error.DisplayMessage();
		}
	}

	message = message.substr(0, strlen);

	std::copy(message.begin(), message.end(), str);
	str[std::min(strlen - 1, (int)message.size())] = 0;
}

void GocatorCV::GocatorManager::Init(char* str, int strlen) {
	
	std::string message = "OK";

	if ((error = gocator.Init()).GetCode() != GocatorCV::ErrorType::OK) {
		message = error.DisplayMessage();
	}

	message = message.substr(0, strlen);

	std::copy(message.begin(), message.end(), str);
	str[std::min(strlen - 1, (int)message.size())] = 0;
}

void GocatorCV::GocatorManager::StartAcquisition(char* str, int strlen, int type, bool check_save_pcd, const char* folder_path_save_pcd) {

	auto _folder_path_save_pcd = std::string(folder_path_save_pcd);
	std::replace(_folder_path_save_pcd.begin(), _folder_path_save_pcd.end(), '\\', '/');
	
	std::string message = "OK";

	if ((error = gocator.Start()).GetCode() != GocatorCV::ErrorType::OK) {
		message = error.DisplayMessage();
	}

	process.StartAcquisition(&gocator, &analysis, type, check_save_pcd, _folder_path_save_pcd);
	
	message = message.substr(0, strlen);

	std::copy(message.begin(), message.end(), str);
	str[std::min(strlen - 1, (int)message.size())] = 0;
}

void GocatorCV::GocatorManager::StopAcquisition(char* str, int strlen) {

	std::string message = "OK";

	process.StopAcquisition();

	if ((error = gocator.Stop()).GetCode() != GocatorCV::ErrorType::OK) {
		message = error.DisplayMessage();
	}
	
	message = message.substr(0, strlen);

	std::copy(message.begin(), message.end(), str);
	str[std::min(strlen - 1, (int)message.size())] = 0;
}

void GocatorCV::GocatorManager::LoadPointCloud(char* str, int strlen, const char* strfilename) {

	std::string message = "OK";

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	if (pcl::io::loadPCDFile<pcl::PointXYZ>(strfilename, *cloud) == -1) {
		PCL_ERROR("Couldn't read file *.pcd\n\n");
		message = "Couldn't read file *.pcd";
	}

	analysis.TestServer(&server);
	analysis.LoadPointCloud(cloud);

	message = message.substr(0, strlen);

	std::copy(message.begin(), message.end(), str);
	str[std::min(strlen - 1, (int)message.size())] = 0;
}

bool GocatorCV::GocatorManager::FileAnalysis(int type, bool check_save_pcd, const char* folder_path_save_pcd) {

	auto _folder_path_save_pcd = std::string(folder_path_save_pcd);
	std::replace(_folder_path_save_pcd.begin(), _folder_path_save_pcd.end(), '\\', '/');

	analysis.Algorithm(type, check_save_pcd, _folder_path_save_pcd);
	return true;
}
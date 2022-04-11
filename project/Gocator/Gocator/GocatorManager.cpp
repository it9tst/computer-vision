#include "GocatorManager.h"

GocatorCV::GocatorManager::GocatorManager() {
	GocatorCV::Gocator gocator;
	GocatorCV::Process process;
	GocatorCV::Analysis analysis;
	GocatorCV::Error error;
	GocatorCV::Server server;
}

bool GocatorCV::GocatorManager::ServerStart() {

	server.ServerStart();
	return true;
}

void GocatorCV::GocatorManager::SetParameter(char* str, int strlen, const char* param, int type) {

	std::string message = "Ok";

	if (type == 1) {

		const char* sensor_ip = param;

		if ((error = gocator.SetParameter(GocatorCV::ParameterType::SENSOR_IP, (void*)sensor_ip)).GetCode() != GocatorCV::ErrorType::OK) {
			message = error.DisplayMessage();
		}
	} else if (type == 2) {

		const char* exposure = param;

		if ((error = gocator.SetParameter(GocatorCV::ParameterType::EXPOSURE, (void*)&exposure)).GetCode() != GocatorCV::ErrorType::OK) {
			message = error.DisplayMessage();
		}
	}

	message = message.substr(0, strlen);

	std::copy(message.begin(), message.end(), str);
	str[std::min(strlen - 1, (int)message.size())] = 0;
}

void GocatorCV::GocatorManager::Init(char* str, int strlen) {
	
	std::string message = "Ok";

	if ((error = gocator.Init()).GetCode() != GocatorCV::ErrorType::OK) {
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

bool GocatorCV::GocatorManager::FileAnalysis(int type, bool checkSavePCD) {

	analysis.Algorithm(type, checkSavePCD);
	return true;
}

void GocatorCV::GocatorManager::StartAcquisition(char* str, int strlen, int type, bool checkSavePCD) {
	
	std::string message = "Ok";

	if ((error = gocator.Start()).GetCode() != GocatorCV::ErrorType::OK) {
		message = error.DisplayMessage();
	}

	process.StartAcquisition(&gocator, &analysis, type, checkSavePCD);
	
	message = message.substr(0, strlen);

	std::copy(message.begin(), message.end(), str);
	str[std::min(strlen - 1, (int)message.size())] = 0;
}

void GocatorCV::GocatorManager::StopAcquisition(char* str, int strlen) {

	std::string message = "Ok";

	process.StopAcquisition();

	if ((error = gocator.Stop()).GetCode() != GocatorCV::ErrorType::OK) {
		message = error.DisplayMessage();
	}
	
	message = message.substr(0, strlen);

	std::copy(message.begin(), message.end(), str);
	str[std::min(strlen - 1, (int)message.size())] = 0;
}

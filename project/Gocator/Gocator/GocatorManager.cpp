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

bool GocatorCV::GocatorManager::SetParameter(const char* param, int type) {

	if (type == 1) {

		const char* sensor_ip = param;

		if ((error = gocator.SetParameter(GocatorCV::ParameterType::SENSOR_IP, (void*)sensor_ip)).GetCode() != GocatorCV::ErrorType::OK) {
			error.DisplayMessage();
			return false;
		}
	} else if (type == 2) {

		const char* exposure = param;

		if ((error = gocator.SetParameter(GocatorCV::ParameterType::EXPOSURE, (void*)&exposure)).GetCode() != GocatorCV::ErrorType::OK) {
			error.DisplayMessage();
			return false;
		}
	}
	
	return true;
}

bool GocatorCV::GocatorManager::Init() {
	
	if ((error = gocator.Init()).GetCode() != GocatorCV::ErrorType::OK) {
		error.DisplayMessage();
		return false;
	}

	return true;
}

bool GocatorCV::GocatorManager::LoadPointCloud(const char* strfilename) {

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	if (pcl::io::loadPCDFile<pcl::PointXYZ>(strfilename, *cloud) == -1) {
		PCL_ERROR("Couldn't read file *.pcd\n\n");
		return false;
	}

	analysis.LoadPointCloud(cloud);

	return true;
}

bool GocatorCV::GocatorManager::FileAnalysis(int type) {

	analysis.Algorithm(type);
	return true;
}

bool GocatorCV::GocatorManager::StartAcquisition(int type) {
	
	process.StartAcquisition(&gocator, &analysis, type);
	return true;
}

bool GocatorCV::GocatorManager::StopAcquisition() {

	process.StopAcquisition();
	return true;
}

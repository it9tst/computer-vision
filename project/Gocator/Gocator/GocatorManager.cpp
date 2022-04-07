#include "GocatorManager.h"

GocatorCV::GocatorManager::GocatorManager() {
	GocatorCV::Gocator gocator;
	GocatorCV::Process process;
	GocatorCV::Analysis analysis;
	GocatorCV::Error error;
}

bool GocatorCV::GocatorManager::SetParameter() {

	if ((error = gocator.SetParameter(GocatorCV::ParameterType::SENSOR_IP, (void*)sensor_ip)).GetCode() != GocatorCV::ErrorType::OK) {
		error.DisplayMessage();
		return false;
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

bool GocatorCV::GocatorManager::OfflineAnalysis() {

	analysis.Algorithm(2);

	return true;
}

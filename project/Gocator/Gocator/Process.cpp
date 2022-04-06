#include "Process.h"

GocatorCV::Process::Process() {
	// constructor
}

void GocatorCV::Process::StartAcquisition(GocatorCV::Gocator *gocator) {

	this->gocator = gocator;
	thread_saving_active = true;
	
	acquisition_thread = std::thread(&Process::StartGrab, this);
	saving_thread = std::thread(&Process::SaveAcquisition, this);
}

void GocatorCV::Process::StopAcquisition() {
	
	thread_saving_active = false;

	acquisition_thread.join();
	saving_thread.join();
/*
	for (auto& t : thread_vector) {
		t.join();
	}*/
}

void GocatorCV::Process::SaveAcquisition(){

	std::unique_lock<std::mutex> locker(m_mutex, std::defer_lock);
	int count = 0;

	while (thread_saving_active) {
		if (buffer_save_data.empty()) {
			std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		} else {

			locker.lock();
			auto _p_cloud_save = buffer_save_data.front();
			buffer_save_data.pop_front();
			std::cout << "SAVE - size: " << buffer_save_data.size() << std::endl;
			locker.unlock();

			std::cout << "SAVE - save value: " << count << std::endl;

			pcl::io::savePCDFileASCII("../../Scan/Point_Cloud_Gocator_" + datetime() + "_" + std::to_string(count) + ".pcd", *_p_cloud_save);
			//pcl::io::savePLYFileASCII("Point_Cloud_Gocator_" + datetime() + "_" + std::to_string(count) + ".ply", *_p_cloud_save);
			count++;
		}
	}
}

void GocatorCV::Process::StartGrab() {

	std::unique_lock<std::mutex> locker(m_mutex, std::defer_lock);
	pcl::PointCloud<pcl::PointXYZ>::Ptr _p_cloud;
	int n_saved_image = 0;

	while (thread_saving_active) {
		if (n_saved_image < 10) {
			_p_cloud = gocator->Grab();

			locker.lock();
			buffer_save_data.push_back(_p_cloud);
			int s = buffer_save_data.size();
			locker.unlock();

			std::cout << "GRAB - size: " << s << std::endl;
			n_saved_image++;

			//thread_vector.emplace_back([&]() { Process::Visualization(_p_cloud); });
		}
	}
}

void GocatorCV::Process::Visualization(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudCopy(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::copyPointCloud(*cloud, *cloudCopy);

	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloudCorrect(new pcl::PointCloud<pcl::PointXYZ>);
	//analysis.CheckValidPoints(cloudCopy, cloudCorrect);

	vtkObject::GlobalWarningDisplayOff();

	// Visualization
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("cloud", true));

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red(cloudCopy, 255, 0, 0);
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addPointCloud<pcl::PointXYZ>(cloudCopy, red, "cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
	viewer->addCoordinateSystem(1);
	viewer->initCameraParameters();

	// Main loop
	while (!viewer->wasStopped()) {
		viewer->spinOnce(100);
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}
}

std::string GocatorCV::Process::datetime() {
	auto t = std::time(nullptr);
	auto tm = *std::localtime(&t);
	std::ostringstream oss;
	oss << std::put_time(&tm, "%Y_%m_%d_%H_%M_%S");
	std::string name = oss.str();

	return std::string(name);
}
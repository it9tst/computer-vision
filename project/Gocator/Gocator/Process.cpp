#include "Process.h"

// constructors
GocatorCV::Process::Process() {}
/**/
GocatorCV::Process::Process(GocatorCV::Gocator* gocator, GocatorCV::Analysis* analysis) {
	this->gocator = gocator;
	this->analysis = analysis;
	std::cout << this << std::endl;
}

void GocatorCV::Process::StartAcquisition(int object_type, bool check_save_pcd, std::string folder_path_save_pcd) {
	//void GocatorCV::Process::StartAcquisition(int object_type, bool check_save_pcd, std::string folder_path_save_pcd, GocatorCV::Gocator* gocator, GocatorCV::Analysis* analysis) {

	this->object_type = object_type;
	this->check_save_pcd = check_save_pcd;
	this->folder_path_save_pcd = folder_path_save_pcd;
	//this->gocator = gocator;
	//this->analysis = analysis;

	thread_saving_active = true;
	
	thread_acquisition = std::thread(&Process::StartGrab, this);
	thread_saving = std::thread(&Process::SaveAcquisition, this);
}

void GocatorCV::Process::StopAcquisition() {
	
	thread_saving_active = false;

	thread_acquisition.join();
	thread_saving.join();
}

void GocatorCV::Process::StartGrab() {
	std::unique_lock<std::mutex> locker(m_mutex, std::defer_lock);
	pcl::PointCloud<pcl::PointXYZ>::Ptr _p_cloud;

	while (thread_saving_active) {
		std::cout << this << std::endl;
		_p_cloud = gocator->Grab();

		locker.lock();
		buffer_save_data.push_back(_p_cloud);
		int s = buffer_save_data.size();
		locker.unlock();

		std::cout << "GRAB - size: " << s << std::endl;
	}
}

void GocatorCV::Process::SaveAcquisition(){

	std::unique_lock<std::mutex> locker(m_mutex, std::defer_lock);
	int count = 0;

	while (thread_saving_active) {
		if(count == count){
			if (buffer_save_data.empty()) {
				std::this_thread::sleep_for(std::chrono::milliseconds(1000));
			} else {
				locker.lock();
				auto _p_cloud_save = buffer_save_data.front();
				buffer_save_data.pop_front();
				std::cout << "SAVE - size: " << buffer_save_data.size() << std::endl;
				locker.unlock();

				// start analysis
				int id = GetRNG();
				analysis->LoadPointCloud(_p_cloud_save, id);
				analysis->Algorithm(object_type, check_save_pcd, folder_path_save_pcd, id);

				std::cout << "SAVE - save value: " << count << std::endl;
				if (check_save_pcd) {
					pcl::io::savePCDFileASCII(folder_path_save_pcd + "/Point_Cloud_Gocator_" + datetime() + "_" + std::to_string(count) + ".pcd", *_p_cloud_save);
				}
				count++;
			} 
		}
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

int GocatorCV::Process::GetRNG() {

	std::srand(static_cast<unsigned int>(std::time(nullptr)));

	return std::rand();
}
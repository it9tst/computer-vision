#include "Process.h"

GocatorCV::Process::Process() {
	// constructor
}

void GocatorCV::Process::StartAcquisition(GocatorCV::Gocator gocator) {

	threadSavingActive = true;
	
	acquisitionThread = std::thread(&Process::StartGrab, this, gocator);
	savingThread = std::thread(&Process::SaveAcquisition, this);
}

void GocatorCV::Process::StopAcquisition() {
	
	threadSavingActive = false;

	acquisitionThread.join();
	savingThread.join();
}

void GocatorCV::Process::SaveAcquisition(){

	std::unique_lock<std::mutex> locker(m_mutex, std::defer_lock);
	count = 0;

	while (threadSavingActive) {
		if (bufferSaveData.empty()) {
			std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		} else {

			locker.lock();
			auto _p_cloud_save = bufferSaveData.front();
			bufferSaveData.pop_front();
			std::cout << "SAVE - size: " << bufferSaveData.size() << std::endl;
			locker.unlock();

			std::cout << "SAVE - save value: " << count << std::endl;

			pcl::io::savePCDFileASCII("Scan/Point_Cloud_Gocator_" + datetime() + "_" + std::to_string(count) + ".pcd", _p_cloud_save);
			//pcl::io::savePLYFileASCII("Point_Cloud_Gocator_" + datetime() + "_" + std::to_string(count) + ".ply", _p_cloud_save);
			count++;
		}
	}
}

void GocatorCV::Process::StartGrab(GocatorCV::Gocator gocator) {

	std::unique_lock<std::mutex> locker(m_mutex, std::defer_lock);
	int n_saved_image = 0;

	while (threadSavingActive) {
		if (n_saved_image < 10) {
			_p_cloud = gocator.Grab();

			locker.lock();
			bufferSaveData.push_back(*_p_cloud);
			int s = bufferSaveData.size();
			locker.unlock();
			std::cout << "GRAB - size: " << s << std::endl;
			n_saved_image++;
		}
	}
}

std::string GocatorCV::Process::datetime() {
	time_t rawtime;
	struct tm* timeinfo;
	char buffer[80];

	time(&rawtime);
	timeinfo = localtime(&rawtime);

	strftime(buffer, 80, "%Y_%m_%d_%H_%M_%S", timeinfo);
	return std::string(buffer);
}
#include "Process.h"

GocatorCV::Process::Process() {
	// constructor
}

void GocatorCV::Process::StartAcquisition() {

	//PNTCLOUD = GOCATOR.GRAB()

	/*
	Variabile classe: vector<Pointcloud> saveData;
	if (n_saved_image < 30)
	{
		lock (  saveData.push_back(pntcloud) )
		n_save_image++;
	}

	NOTE SULLA PARTE DI ANALYSIS....
	vector<pntcloud> che viene riempito a seguito della grab, thread di elaborazione che consuma (PointCloudAnalysis(pntcloud))
	*/
	
	std::thread acquisitionThread(gocator.StartThread());
	std::thread savingThread(GocatorCV::Process::StartThread());
}

void GocatorCV::Process::StopAcquisition() {
	
	threadSavingActive = false;
	acquisitionThread.join();
	savingThread.join();
}

void GocatorCV::Process::SaveAcquisition(){

	std::unique_lock<std::mutex> locker(m_mutex, std::defer_lock);
	count = 0;

	while (true) {
		if (bufferSaveData.empty()) {
			std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		} else {

			locker.lock();
			auto _p_cloud_save = bufferSaveData.front();
			bufferSaveData.pop_front();
			locker.unlock();

			pcl::io::savePCDFileASCII("PointCloudGocator" + std::to_string(count) + ".pcd", _p_cloud_save);
			count++;
		}
	}

	/*
	while (true) {
		if buffer vuoto -> std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		else
		{   
		vector<pntcloud> toSave;
		lock { toSave = saveData; saveData.clear()}
		for i=0:len(toSave)
			save file
		}
	}
	pcl::io::savePCDFileASCII("PointCloudGocator" + std::to_string(count) + ".pcd", _p_cloud);
	pcl::io::savePLYFileASCII("PointCloudGocator" + std::to_string(count) + ".ply", _p_cloud);
	*/
}

std::thread GocatorCV::Process::StartThread() {
	GocatorCV::Process::SaveAcquisition();
	return std::thread();
}
#pragma once

//std c/c++
#include <thread>
#include <mutex>
#include <deque>
#include <ctime>

#include "Gocator.h"

namespace GocatorCV {

	class Process {

	private:
		std::thread acquisitionThread;
		std::thread savingThread;
		std::mutex m_mutex;
		std::deque< pcl::PointCloud<pcl::PointXYZ> > bufferSaveData;
		pcl::PointCloud<pcl::PointXYZ>::Ptr _p_cloud;
		bool threadSavingActive;
		int count;

	public:
		Process();
		void StartAcquisition(GocatorCV::Gocator gocator);
		void StopAcquisition();
		void SaveAcquisition();
		void StartGrab(GocatorCV::Gocator gocator);
		std::string datetime();
	};
}
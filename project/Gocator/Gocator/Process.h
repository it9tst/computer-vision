#pragma once

//std c/c++
#include <thread>
#include <mutex>
#include <deque>

#include "Gocator.h"

namespace GocatorCV {

	class Process {

	private:
		GocatorCV::Gocator gocator;
		std::thread acquisitionThread;
		std::thread savingThread;
		std::thread processThread;
		std::mutex m_mutex;
		std::deque< pcl::PointCloud<pcl::PointXYZ> > bufferSaveData;
		bool threadSavingActive;
		int count;

	public:
		Process();
		void StartAcquisition();
		void StopAcquisition();
		void SaveAcquisition();
		std::thread StartThread();
	};
}
#pragma once

//std c/c++
#include <thread>
#include <mutex>
#include <deque>

#include "Gocator.h"
#include "Analysis.h"

namespace GocatorCV {

	class Process {

	private:
		GocatorCV::Gocator *gocator;
		GocatorCV::Analysis analysis;
		//std::vector<std::thread> ThreadVector;
		std::thread acquisitionThread;
		std::thread savingThread;
		std::thread analysisThread;
		std::mutex m_mutex;
		std::deque< pcl::PointCloud<pcl::PointXYZ>::Ptr > bufferSaveData;
		bool threadSavingActive;
		int count;

	public:
		Process();
		void StartAcquisition(GocatorCV::Gocator *gocator);
		void StopAcquisition();
		void SaveAcquisition();
		void StartGrab();
		void Visualization(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
		std::string datetime();
	};
}
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
		//std::vector<std::thread> thread_vector;
		std::thread acquisition_thread;
		std::thread saving_thread;
		std::thread analysis_thread;
		std::mutex m_mutex;
		std::deque< pcl::PointCloud<pcl::PointXYZ>::Ptr > buffer_save_data;
		bool thread_saving_active;
		std::string datetime();

	public:
		Process();
		void StartAcquisition(GocatorCV::Gocator *gocator);
		void StopAcquisition();
		void SaveAcquisition();
		void StartGrab();
		void Visualization(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
	};
}
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
		GocatorCV::Analysis *analysis;
		std::thread acquisition_thread;
		std::thread saving_thread;
		std::thread analysis_thread;
		std::mutex m_mutex;
		std::deque< pcl::PointCloud<pcl::PointXYZ>::Ptr > buffer_save_data;
		bool thread_saving_active;
		bool check_save_pcd = false;
		std::string folder_path_save_pcd = "../../Scan/";
		int type;
		std::string datetime();

	public:
		Process();
		void StartAcquisition(GocatorCV::Gocator *gocator, GocatorCV::Analysis* analysis, int type, bool check_save_pcd, std::string folder_path_save_pcd);
		void StopAcquisition();
		void SaveAcquisition();
		void StartGrab();
		void Visualization(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
	};
}
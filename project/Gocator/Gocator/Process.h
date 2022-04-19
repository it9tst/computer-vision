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
		std::thread thread_acquisition;
		std::thread thread_saving;
		std::thread thread_analysis;
		std::mutex m_mutex;
		std::deque< pcl::PointCloud<pcl::PointXYZ>::Ptr > buffer_save_data;
		bool thread_saving_active;
		bool check_save_pcd = false;
		int object_type;
		std::string folder_path_save_pcd = "../../Scan/";
		std::string datetime();

	public:
		Process();
		//Process(GocatorCV::Gocator* gocator, GocatorCV::Analysis* analysis);
		void StartAcquisition(int object_type, bool check_save_pcd, std::string folder_path_save_pcd, GocatorCV::Gocator* gocator, GocatorCV::Analysis* analysis);
		void StopAcquisition();
		void StartGrab();
		void SaveAcquisition();
		int GetRNG();
	};
}
#pragma once

#define DECLSPEC __declspec(dllexport)

#ifndef GOCATORMANAGER_H
#define GOCATORMANAGER_H

#include "Process.h"
#include <nlohmann/json.hpp>

namespace GocatorCV {

	class DECLSPEC GocatorManager {
	
	private:
		GocatorCV::Gocator gocator;
		GocatorCV::Process process;
		GocatorCV::Analysis analysis;
		GocatorCV::Error error;
		GocatorCV::Pipe pipe;
		int id;

	public:
		GocatorManager();
		~GocatorManager();
		void SetParameter(char* str, int strlen, const char* param, int type);
		void Init(char* str, int strlen);
		void StartAcquisition(char* str, int strlen, int object_type, bool check_save_pcd, const char* folder_path_save_pcd);
		void StopAcquisition(char* str, int strlen);
		void Stop(char* str, int strlen);
		void LoadPointCloud(char* str, int strlen, const char* file_name);
		bool FileAnalysis(int object_type, bool check_save_pcd, const char* folder_path_save_pcd);
	};
}

#endif
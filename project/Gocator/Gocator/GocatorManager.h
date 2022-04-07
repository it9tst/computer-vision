#pragma once

#include "Process.h"

#define DECLSPEC __declspec(dllexport)

#ifndef GOCATORMANAGER_H
#define GOCATORMANAGER_H

namespace GocatorCV {

	class DECLSPEC GocatorManager {
	
	private:
		GocatorCV::Gocator gocator;
		GocatorCV::Process process;
		GocatorCV::Analysis analysis;
		GocatorCV::Error error;
		const char* sensor_ip = "192.168.1.151";

	public:
		GocatorManager();
		bool SetParameter();
		bool Init();
		bool LoadPointCloud(const char* strfilename);
		bool OfflineAnalysis();
	};
}

#endif
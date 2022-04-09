#pragma once

#include "Process.h"
#include "Server.h"

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
		GocatorCV::Server server;

	public:
		GocatorManager();
		bool ServerStart();
		bool SetParameter(const char* param, int type);
		bool Init();
		bool LoadPointCloud(const char* strfilename);
		bool FileAnalysis(int type);
		bool StartAcquisition(int type);
		bool StopAcquisition();
	};
}

#endif
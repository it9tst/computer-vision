#pragma once

#define DECLSPEC __declspec(dllexport)

#ifndef GOCATORMANAGER_H
#define GOCATORMANAGER_H

#include "Process.h"
#include "Server.h"

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
		void SetParameter(char* str, int strlen, const char* param, int type);
		void Init(char* str, int strlen);
		void LoadPointCloud(char* str, int strlen, const char* strfilename);
		void StartAcquisition(char* str, int strlen, int type, bool checkSavePCD);
		void StopAcquisition(char* str, int strlen);
		bool FileAnalysis(int type, bool checkSavePCL);
	};
}

#endif
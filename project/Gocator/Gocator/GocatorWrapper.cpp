#include "GocatorManager.h"

using namespace GocatorCV;

// Gocator
extern "C" __declspec(dllexport) GocatorManager * CreateGocatorManager() {
	return new GocatorManager();
}

extern "C" __declspec(dllexport) bool GocatorManager_ServerStart(GocatorManager * gocatormanager) {
	return gocatormanager->ServerStart();
}

extern "C" __declspec(dllexport) bool GocatorManager_SetParameter(GocatorManager * gocatormanager, const char* param, int type) {
	return gocatormanager->SetParameter(param, type);
}

extern "C" __declspec(dllexport) bool GocatorManager_Init(GocatorManager * gocatormanager) {
	return gocatormanager->Init();
}

extern "C" __declspec(dllexport) bool GocatorManager_LoadPointCloud(GocatorManager * gocatormanager, const char* strfilename) {
	return gocatormanager->LoadPointCloud(strfilename);
}

extern "C" __declspec(dllexport) bool GocatorManager_FileAnalysis(GocatorManager * gocatormanager, int type) {
	return gocatormanager->FileAnalysis(type);
}

extern "C" __declspec(dllexport) bool GocatorManager_StartAcquisition(GocatorManager * gocatormanager, int type) {
	return gocatormanager->StartAcquisition(type);
}

extern "C" __declspec(dllexport) bool GocatorManager_StopAcquisition(GocatorManager * gocatormanager) {
	return gocatormanager->StopAcquisition();
}
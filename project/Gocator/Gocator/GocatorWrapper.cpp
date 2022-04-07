#include "GocatorManager.h"

using namespace GocatorCV;

// Gocator
extern "C" __declspec(dllexport) GocatorManager * CreateGocatorManager() {
	return new GocatorManager();
}

extern "C" __declspec(dllexport) bool GocatorManager_SetParameter(GocatorManager * gocatormanager) {
	return gocatormanager->SetParameter();
}

extern "C" __declspec(dllexport) bool GocatorManager_Init(GocatorManager * gocatormanager) {
	return gocatormanager->Init();
}

extern "C" __declspec(dllexport) bool GocatorManager_LoadPointCloud(GocatorManager * gocatormanager, const char* strfilename) {
	return gocatormanager->LoadPointCloud(strfilename);
}

extern "C" __declspec(dllexport) bool GocatorManager_OfflineAnalysis(GocatorManager * gocatormanager) {
	return gocatormanager->OfflineAnalysis();
}







/*
extern "C" __declspec(dllexport) Error Gocator_Start(Gocator * gocator) {
	return gocator->Start();
}

extern "C" __declspec(dllexport) Error Gocator_Stop(Gocator * gocator) {
	return gocator->Stop();
}

// Process
extern "C" __declspec(dllexport) Process * CreateProcessC() {
	return new Process();
}

extern "C" __declspec(dllexport) void Process_StartAcquisition(Process * process, Gocator *gocator) {
	return process->StartAcquisition(gocator);
}

extern "C" __declspec(dllexport) void Process_StopAcquisition(Process * process) {
	return process->StopAcquisition();
}

// Error
extern "C" __declspec(dllexport) Error * CreateError() {
	return new Error();
}
*/
#include "GocatorManager.h"

using namespace GocatorCV;

extern "C" __declspec(dllexport) GocatorManager * CreateGocatorManager() {
	return new GocatorManager();
}

extern "C" __declspec(dllexport) void DeleteGocatorManager(GocatorManager * gocatormanager) {
	delete gocatormanager;
}

extern "C" __declspec(dllexport) void GocatorManager_ServerStart(GocatorManager * gocatormanager) {
	gocatormanager->ServerStart();
}

extern "C" __declspec(dllexport) void GocatorManager_SetParameter(GocatorManager * gocatormanager, char* str, int strlen, const char* param, int type) {
	gocatormanager->SetParameter(str, strlen, param, type);
}

extern "C" __declspec(dllexport) void GocatorManager_Init(GocatorManager * gocatormanager, char* str, int strlen) {
	gocatormanager->Init(str, strlen);
}

extern "C" __declspec(dllexport) void GocatorManager_LoadPointCloud(GocatorManager * gocatormanager, char* str, int strlen, const char* strfilename) {
	gocatormanager->LoadPointCloud(str, strlen, strfilename);
}

extern "C" __declspec(dllexport) void GocatorManager_StartAcquisition(GocatorManager * gocatormanager, char* str, int strlen, int type, bool checkSavePCD, const char* folderPathSavePCD) {
	gocatormanager->StartAcquisition(str, strlen, type, checkSavePCD, folderPathSavePCD);
}

extern "C" __declspec(dllexport) void GocatorManager_StopAcquisition(GocatorManager * gocatormanager, char* str, int strlen) {
	gocatormanager->StopAcquisition(str, strlen);
}

extern "C" __declspec(dllexport) bool GocatorManager_FileAnalysis(GocatorManager * gocatormanager, int type, bool checkSavePCD, const char* folderPathSavePCD) {
	return gocatormanager->FileAnalysis(type, checkSavePCD, folderPathSavePCD);
}
#include "GocatorManager.h"

using namespace GocatorCV;

extern "C" __declspec(dllexport) GocatorManager * CreateGocatorManager() {
	return new GocatorManager();
}

extern "C" __declspec(dllexport) void DeleteGocatorManager(GocatorManager * gocatormanager) {
	delete gocatormanager;
}

extern "C" __declspec(dllexport) void GocatorManager_SetParameter(GocatorManager * gocatormanager, char* str, int strlen, const char* param, int type) {
	gocatormanager->SetParameter(str, strlen, param, type);
}

extern "C" __declspec(dllexport) void GocatorManager_Init(GocatorManager * gocatormanager, char* str, int strlen) {
	gocatormanager->Init(str, strlen);
}

extern "C" __declspec(dllexport) void GocatorManager_LoadPointCloud(GocatorManager * gocatormanager, char* str, int strlen, const char* file_name) {
	gocatormanager->LoadPointCloud(str, strlen, file_name);
}

extern "C" __declspec(dllexport) void GocatorManager_StartAcquisition(GocatorManager * gocatormanager, char* str, int strlen, int object_type, bool check_save_pcd, const char* folder_path_save_pcd) {
	gocatormanager->StartAcquisition(str, strlen, object_type, check_save_pcd, folder_path_save_pcd);
}

extern "C" __declspec(dllexport) void GocatorManager_StopAcquisition(GocatorManager * gocatormanager, char* str, int strlen) {
	gocatormanager->StopAcquisition(str, strlen);
}

extern "C" __declspec(dllexport) bool GocatorManager_FileAnalysis(GocatorManager * gocatormanager, int object_type, bool check_save_pcd, const char* folder_path_save_pcd) {
	return gocatormanager->FileAnalysis(object_type, check_save_pcd, folder_path_save_pcd);
}
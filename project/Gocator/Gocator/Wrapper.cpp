#include "Gocator.h"
#include "Process.h"
#include "Error.h"

using namespace GocatorCV;

// Gocator
extern "C" __declspec(dllexport) Gocator * CreateGocator() {
	return new Gocator();
}

extern "C" __declspec(dllexport) Error Gocator_Init(Gocator * gocator) {
	return gocator->Init();
}

extern "C" __declspec(dllexport) Error Gocator_Start(Gocator * gocator) {
	return gocator->Start();
}

extern "C" __declspec(dllexport) Error Gocator_Stop(Gocator * gocator) {
	return gocator->Stop();
}

extern "C" __declspec(dllexport) Error Gocator_SetParameter(Gocator * gocator, ParameterType name, void* value) {
	return gocator->SetParameter(name, value);
}

// Process
extern "C" __declspec(dllexport) Process * CreateProcess() {
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
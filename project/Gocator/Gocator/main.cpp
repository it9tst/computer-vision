// main.cpp : Questo file contiene la funzione 'main', in cui inizia e termina l'esecuzione del programma.

//std c/c++
#include <iostream>

#include "Process.h"

//main
int main(int argc, char** argv) {

	bool isOnline = false;

	const char* sensorIp = "192.168.1.151";
	k64f exposure = 2000;

	GocatorCV::Gocator gocator;
	GocatorCV::Process process;
	GocatorCV::Analysis analysis;
	GocatorCV::Error error;

	// Hello message
	std::cout << "Gocator example running!" << std::endl;

	if (isOnline) {
		if ((error = gocator.SetParameter(GocatorCV::ParameterType::SENSOR_IP, (void*)sensorIp)).GetCode() != GocatorCV::ErrorType::OK) {
			error.DisplayMessage();
		}

		if ((error = gocator.Init()).GetCode() != GocatorCV::ErrorType::OK) {
			error.DisplayMessage();
		}

		if ((error = gocator.SetParameter(GocatorCV::ParameterType::EXPOSURE, (void*)&exposure)).GetCode() != GocatorCV::ErrorType::OK) {
			error.DisplayMessage();
		}

		if ((error = gocator.Start()).GetCode() != GocatorCV::ErrorType::OK) {
			error.DisplayMessage();
		}

		process.StartAcquisition(&gocator);
		Sleep(30000);
		process.StopAcquisition();

		if ((error = gocator.Stop()).GetCode() != GocatorCV::ErrorType::OK) {
			error.DisplayMessage();
		}
	} else {
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		
		if (pcl::io::loadPCDFile<pcl::PointXYZ>("Scan/2022_03_19_01_04_58_Matrix_Transformation.pcd", *cloud) == -1) {
			PCL_ERROR("Couldn't read file *.pcd\n\n");
			return (-1);
		}
		
		std::cout << "Loaded " << cloud->width * cloud->height << " data points from *.pcd" << std::endl << std::endl;

		analysis.LoadPointCloud(cloud);
		analysis.Algorithm();
	}

	// Bye bye message
	std::cout << "Gocator example finished!" << std::endl;

	return 1;
}



/*
	StartAcquisition ---> partire thread per l'acquisizione (grab +  aggiunge nella coda di elaborazione)
	ThreadSaving ---> si occupa del salvataggio di immagini
	StartProcess ---> thread per l'elaborazione, preleva dalla coda di elaborazione ed elabora

	NOTE SULLA PARTE DI ANALYSIS....
	vector<pntcloud> che viene riempito a seguito della grab, thread di elaborazione che consuma (PointCloudAnalysis(pntcloud))
*/
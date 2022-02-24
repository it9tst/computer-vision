// main.cpp : Questo file contiene la funzione 'main', in cui inizia e termina l'esecuzione del programma.

//std c/c++
#include <iostream>
#include <thread>

#include "Gocator.h"
#include "Process.h"

//#define SENSOR_IP "192.168.1.151"

//main
int main(int argc, char** argv) {

	const char* sensor_ip = "192.168.1.151";

	GocatorCV::Gocator gocator;
	GocatorCV::Process process;

	//Hello message
	std::cout << "Gocator example running" << std::endl;

	gocator.SetParameter(sensor_ip);
	gocator.Init();
	gocator.Start();

	process.StartAcquisition();

	//sleep for a while
	Sleep(5);

	gocator.Stop();

	//bye bye message
	std::cout << "Program finished !" << std::endl;

	return 1;
}


/*
	kChar model_name[50];
	GoStamp* stamp = kNULL;
	GoProfilePositionX positionX = kNULL;
	GoMeasurementData* measurementData = kNULL;

	//gets the sensor model
	if ((status = GoSensor_Model(sensor, model_name, 50)) != kOK) {
		std::cout << "Error: GoSensor_Model: " << status << std::endl;
		return -1;
	}

	//prints sensor info
	std::cout << "Connected to Sensor: " << std::endl;
	std::cout << "\tModel: \t" << model_name << std::endl;
	std::cout << "\tIP: \t" << SENSOR_IP << std::endl;
	std::cout << "\tSN: \t" << GoSensor_Id(sensor) << std::endl;
	std::cout << "\tState: \t" << GoSensor_State(sensor) << std::endl;
*/
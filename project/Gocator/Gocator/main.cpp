// main.cpp : Questo file contiene la funzione 'main', in cui inizia e termina l'esecuzione del programma.

//std c/c++
#include <iostream>

#include "Gocator.h"
#include "Process.h"

//main
int main(int argc, char** argv) {

	const char* sensorIp = "192.168.1.151";

	GocatorCV::Gocator gocator;
	GocatorCV::Process process;
	GocatorCV::Error error;

	//Hello message
	std::cout << "Gocator example running!" << std::endl;

	gocator.SetParameter(sensorIp);

	if ((error = gocator.Init()).GetCode() != GocatorCV::Error_Type::OK) {
		error.DisplayMessage();
	}
	
	if ((error = gocator.Start()).GetCode() != GocatorCV::Error_Type::OK) {
		error.DisplayMessage();
	}
	
	process.StartAcquisition();
	Sleep(10);
	process.StopAcquisition();

	if ((error = gocator.Stop()).GetCode() != GocatorCV::Error_Type::OK) {
		error.DisplayMessage();
	}

	//bye bye message
	std::cout << "Gocator example finished!" << std::endl;

	return 1;
}


/*
	StartAcquisition ---> partire thread per l'acquisizione (grab +  aggiunge nella coda di elaborazione)
	ThreadSaving ---> si occupa del salvataggio di immagini
	StartProcess ---> thread per l'elaborazione, preleva dalla coda di elaborazione ed elabora
*/

/*
	todo: da vedere i vari parametri come vengono passati
	creare una funzione generale per il setting setParameter(name,value)
*/



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
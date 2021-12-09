// Gocator.cpp : Questo file contiene la funzione 'main', in cui inizia e termina l'esecuzione del programma.

//std c/c++
#include <windows.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <memory.h>

//GoSdk
#include <GoSdk/GoSdk.h>

//constants
#define SENSOR_IP "192.168.1.151"

//main
int main(int argc, char** argv) {
	kStatus status;
	kAssembly api = kNULL;
	GoSystem system = kNULL;
	GoSensor sensor = kNULL;
	kIpAddress ipAddress;

	//Hello message
	std::cout << "Gocator example running" << std::endl;

	// construct Gocator API Library
	if ((status = GoSdk_Construct(&api)) != kOK) {
		std::cout << "Error: GoSdk_Construct: " << status << std::endl;
		return -1;
	}

	// construct GoSystem object
	if ((status = GoSystem_Construct(&system, kNULL)) != kOK) {
		std::cout << "Error: GoSystem_Construct: " << status << std::endl;
		return -1;
	}

	// obtain GoSensor object by sensor IP address
	kIpAddress_Parse(&ipAddress, SENSOR_IP);
	if ((status = GoSystem_FindSensorByIpAddress(system, &ipAddress, &sensor)) != kOK) {
		std::cout << "Error: GoSystem_FindSensorByIpAddress: " << status << std::endl;
		return -1;
	}

	// create connection to GoSensor object
	if ((status = GoSensor_Connect(sensor)) != kOK) {
		std::cout << "Error: GoSensor_Connect: " << status << std::endl;
		return -1;
	}

	// enable sensor data channel
	if ((status = GoSystem_EnableData(system, kTRUE)) != kOK) {
		std::cout << "Error: GoSensor_EnableData: " << status << std::endl;
		return -1;
	}

	// start Gocator sensor
	if ((status = GoSystem_Start(system)) != kOK) {
		std::cout << "Error: GoSystem_Start: " << status << std::endl;
		return -1;
	}

	//sleep for a while
	Sleep(5);

	// stop Gocator sensor
	if ((status = GoSystem_Stop(system)) != kOK) {
		std::cout << "Error: GoSystem_Stop: " << status << std::endl;
		return -1;
	}

	// destroy handles
	GoDestroy(system);
	GoDestroy(api);

	//bye bye message
	std::cout << "Program finished !" << status << std::endl;
	return 1;
}





// Per eseguire il programma: CTRL+F5 oppure Debug > Avvia senza eseguire debug
// Per eseguire il debug del programma: F5 oppure Debug > Avvia debug

// Suggerimenti per iniziare: 
//   1. Usare la finestra Esplora soluzioni per aggiungere/gestire i file
//   2. Usare la finestra Team Explorer per connettersi al controllo del codice sorgente
//   3. Usare la finestra di output per visualizzare l'output di compilazione e altri messaggi
//   4. Usare la finestra Elenco errori per visualizzare gli errori
//   5. Passare a Progetto > Aggiungi nuovo elemento per creare nuovi file di codice oppure a Progetto > Aggiungi elemento esistente per aggiungere file di codice esistenti al progetto
//   6. Per aprire di nuovo questo progetto in futuro, passare a File > Apri > Progetto e selezionare il file con estensione sln

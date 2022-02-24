#include "Gocator.h"

GocatorCV::Gocator::Gocator() {
	// constructor
}

GocatorCV::Error GocatorCV::Gocator::Init() {

	GoSensor sensor = kNULL;
	kIpAddress ipAddress;

	// construct Gocator API Library
	if ((status = GoSdk_Construct(&api)) != kOK) {
		return GOSDK_API_CONSTRUCT;
	}

	// construct GoSystem object
	if ((status = GoSystem_Construct(&system, kNULL)) != kOK) {
		return GOSYSTEM_CONSTRUCT;
	}

	// obtain GoSensor object by sensor IP address
	kIpAddress_Parse(&ipAddress, sensor_ip);
	if ((status = GoSystem_FindSensorByIpAddress(system, &ipAddress, &sensor)) != kOK) {
		return GOSYSTEM_FIND_SENSOR_BY_IP_ADDRESS;
	}

	// create connection to GoSensor object
	if ((status = GoSensor_Connect(sensor)) != kOK) {
		return GOSENSOR_CONNECT;
	}

	// enable sensor data channel
	if ((status = GoSystem_EnableData(system, kTRUE)) != kOK) {
		return GOSENSOR_ENABLE_DATA;
	}

	return OK;
}

GocatorCV::Error GocatorCV::Gocator::Start() {
	// start Gocator sensor
	if ((status = GoSystem_Start(system)) != kOK) {
		return GOSYSTEM_START;
	}

	return OK;
}

GocatorCV::Error GocatorCV::Gocator::Stop() {
	// destroy handles
	GoDestroy(system);
	GoDestroy(api);

	// stop Gocator sensor
	if ((status = GoSystem_Stop(system)) != kOK) {
		return GOSYSTEM_STOP;
	}

	return OK;
}

void GocatorCV::Gocator::SetParameter(const char* sensor_ip) {
	// Impostare parametri gocator (setParameter(name,value))
	sensor_ip = sensor_ip;
}

void GocatorCV::Gocator::Grab() {
	// Funzione di Grab che permetta dall'esterno di acquisire il frame del gocator
}

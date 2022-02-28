#include "Gocator.h"

GocatorCV::Gocator::Gocator() {
	// constructor
}

GocatorCV::Error GocatorCV::Gocator::Init() {

	// construct Gocator API Library
	if ((status = GoSdk_Construct(&api)) != kOK) {
		return Error(GOSDK_API_CONSTRUCT, status);
	}

	// construct GoSystem object
	if ((status = GoSystem_Construct(&system, kNULL)) != kOK) {
		return Error(GOSYSTEM_CONSTRUCT, status);
	}

	// Parse IP address into address data structure
	kIpAddress_Parse(&ipAddress, sensorIp);

	// obtain GoSensor object by sensor IP address
	if ((status = GoSystem_FindSensorByIpAddress(system, &ipAddress, &sensor)) != kOK) {
		return Error(GOSYSTEM_FIND_SENSOR_BY_IP_ADDRESS, status);
	}

	// create connection to GoSensor object
	if ((status = GoSensor_Connect(sensor)) != kOK) {
		return Error(GOSENSOR_CONNECT, status);
	}

	// retrieve setup handle
	if ((setup = GoSensor_Setup(sensor)) == kNULL) {
		return Error(GOSENSOR_SETUP, status);
	}

	// read current parameters
	currentExposure = GoSetup_Exposure(setup, GO_ROLE_MAIN);
	std::cout << "\tCurrent Parameters:" << std::endl;
	std::cout << "\t-------------------" << std::endl;
	std::cout << "\tExposure: " << currentExposure << std::endl;

	// modify parameter in main sensor
	if ((status = GoSetup_SetExposure(setup, GO_ROLE_MAIN, currentExposure + 200)) != kOK) {
		return Error(GOSETUP_SETEXPOSURE, status);
	}
	// GoSensorFlush() - immediately synchronizes configuration changes to the sensor
	GoSensor_Flush(sensor);

	// read current parameters
	newExposure = GoSetup_Exposure(setup, GO_ROLE_MAIN);
	std::cout << "\tNew Parameters:" << std::endl;
	std::cout << "\t---------------" << std::endl;
	std::cout << "\tExposure: " << newExposure << std::endl;

	return Error(OK, 1);
}

GocatorCV::Error GocatorCV::Gocator::Start() {

	// enable sensor data channel
	if ((status = GoSystem_EnableData(system, kTRUE)) != kOK) {
		return Error(GOSENSOR_ENABLE_DATA, status);
	}

	// start Gocator sensor
	if ((status = GoSystem_Start(system)) != kOK) {
		return Error(GOSYSTEM_START, status);
	}

	return Error(OK, 1);
}

GocatorCV::Error GocatorCV::Gocator::Stop() {

	// destroy handles
	GoDestroy(system);
	GoDestroy(api);

	// stop Gocator sensor
	if ((status = GoSystem_Stop(system)) != kOK) {
		return Error(GOSYSTEM_STOP, status);
	}

	return Error(OK, 1);
}

void GocatorCV::Gocator::SetParameter(const char* sensorIp) {
	// Impostare parametri gocator (setParameter(name,value))
	this->sensorIp = sensorIp;
}

void GocatorCV::Gocator::Grab() {
	// Funzione di Grab che permetta dall'esterno di acquisire il frame del gocator

	threadSavingActive = true;
	std::unique_lock<std::mutex> locker(m_mutex, std::defer_lock);

	while (true) {

		//Get data
		std::cout << "\n\nSensor is running ..." << std::endl;

		if (GoSystem_ReceiveData(system, &dataset, RECEIVE_TIMEOUT) == kOK) {

			std::cout << "************************* Start of GoSystem_ReceiveData *************************" << std::endl;
			std::cout << "\nData message received: " << std::endl;
			std::cout << "Dataset total count: " << GoDataSet_Count(dataset) << "\n" << std::endl;

			// each result can have multiple data items
			// loop through all items in result message
			for (i = 0; i < GoDataSet_Count(dataset); ++i) {
				std::cout << "\n\nDataset index count: " << i << std::endl;
				dataObj = GoDataSet_At(dataset, i);

				switch (GoDataMsg_Type(dataObj)) {

				case GO_DATA_MESSAGE_TYPE_SURFACE:
				{
					//point cloud
					pcl::PointCloud<pcl::PointXYZ> _p_cloud;

					//cast to GoSurfaceMsg
					GoSurfaceMsg surfaceMsg = dataObj;

					//Get general data of the surface
					unsigned int row_count = GoSurfaceMsg_Length(surfaceMsg);
					unsigned int width = GoSurfaceMsg_Width(surfaceMsg);
					unsigned int exposure = GoSurfaceMsg_Exposure(surfaceMsg);

					//get offsets and resolutions
					double xResolution = NM_TO_MM(GoSurfaceMsg_XResolution(surfaceMsg));
					double yResolution = NM_TO_MM(GoSurfaceMsg_YResolution(surfaceMsg));
					double zResolution = NM_TO_MM(GoSurfaceMsg_ZResolution(surfaceMsg));
					double xOffset = UM_TO_MM(GoSurfaceMsg_XOffset(surfaceMsg));
					double yOffset = UM_TO_MM(GoSurfaceMsg_YOffset(surfaceMsg));
					double zOffset = UM_TO_MM(GoSurfaceMsg_ZOffset(surfaceMsg));

					//Print raw cloud metadata
					std::cout << "Surface Message" << std::endl;
					std::cout << "\tLength: " << row_count << std::endl;
					std::cout << "\tWidth: " << width << std::endl;
					std::cout << "\tExposure: " << exposure << std::endl;
					std::cout << "\txResolution: " << xResolution << std::endl;
					std::cout << "\tyResolution: " << yResolution << std::endl;
					std::cout << "\tzResolution: " << zResolution << std::endl;
					std::cout << "\txOffset: " << xOffset << std::endl;
					std::cout << "\tyOffset: " << yOffset << std::endl;
					std::cout << "\tzOffset: " << zOffset << std::endl;

					//resize the point cloud
					_p_cloud.height = row_count;
					_p_cloud.width = width;
					_p_cloud.resize(row_count * width);

					//run over all rows
					for (ii = 0; ii < row_count; ii++) {
						//get the pointer to row
						short* data = GoSurfaceMsg_RowAt(surfaceMsg, ii);

						//run over the width of row ii
						for (jj = 0; jj < width; jj++) {

							//set xy in meters. x component inverted to fulfill right-handed frame (Gocator is left-handed!)
							_p_cloud.points.at(ii * width + jj).x = -0.001 * (xOffset + xResolution * jj);
							_p_cloud.points.at(ii * width + jj).y = 0.001 * (yOffset + yResolution * ii);

							//set z  in meters.
							if (data[jj] != INVALID_RANGE_16BIT)
								_p_cloud.points.at(ii * width + jj).z = 0.001 * (zOffset + zResolution * data[jj]);
							else
								_p_cloud.points.at(ii * width + jj).z = 0.001 * (INVALID_RANGE_DOUBLE);
						}
					}

					locker.lock();
					bufferSaveData.push_back(_p_cloud);
					locker.unlock();
				}
				break;
				}
			}
			GoDestroy(dataset);
			std::cout << "************************* End of GoSystem_ReceiveData ***************************\n\n" << std::endl;
		} else {
			std::cout << "Error: No data received during the waiting period" << std::endl;
		}
	}
}

std::thread GocatorCV::Gocator::StartThread() {
	GocatorCV::Gocator::Grab();
	return std::thread();
}

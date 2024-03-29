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
	kIpAddress_Parse(&ipAddress, sensor_ip);

	// obtain GoSensor object by sensor IP address
	if ((status = GoSystem_FindSensorByIpAddress(system, &ipAddress, &sensor)) != kOK) {
		return Error(GOSYSTEM_FIND_SENSOR_BY_IP_ADDRESS, status);
	}

	// create connection to GoSensor object
	if ((status = GoSensor_Connect(sensor)) != kOK) {
		return Error(GOSENSOR_CONNECT, status);
	}

	//gets the sensor model
	if ((status = GoSensor_Model(sensor, model_name, 50)) != kOK) {
		return Error(GOSENSOR_MODEL, status);
	}

	//prints sensor info
	std::cout << "\tConnected to Sensor: " << std::endl;
	std::cout << "\tModel: \t" << model_name << std::endl;
	std::cout << "\tIP: \t" << sensor_ip << std::endl;
	std::cout << "\tSN: \t" << GoSensor_Id(sensor) << std::endl;
	std::cout << "\tState: \t" << GoSensor_State(sensor) << std::endl << std::endl;

	// retrieve setup handle
	if ((setup = GoSensor_Setup(sensor)) == kNULL) {
		return Error(GOSENSOR_SETUP, status);
	}

	// read current parameters
	try {
		exposure = GoSetup_Exposure(setup, GO_ROLE_MAIN);
		std::cout << "\tCurrent Parameters:" << std::endl;
		std::cout << "\t-------------------" << std::endl;
		std::cout << "\tExposure: " << exposure << std::endl << std::endl;
	} catch (...) {
		std::cout << "GoSetup_Exposure Exception" << std::endl;
	}

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

	// stop Gocator sensor
	if ((status = GoSystem_Stop(system)) != kOK) {
		return Error(GOSYSTEM_STOP, status);
	}

	// destroy handles
	GoDestroy(system);
	GoDestroy(api);

	return Error(OK, 1);
}

GocatorCV::Error GocatorCV::Gocator::SetParameter(ParameterType name, void* value) {
	
	switch (name) {
		case SENSOR_IP:
			this->sensor_ip = (const char*)value;
			break;
		case EXPOSURE:
		{
			k64f* _value = (k64f*)value;
			
			// modify parameter in main sensor
			if ((status = GoSetup_SetExposure(setup, GO_ROLE_MAIN, *_value)) != kOK) {
				return Error(GOSETUP_SETEXPOSURE, status);
			}
			// GoSensorFlush() - immediately synchronizes configuration changes to the sensor
			GoSensor_Flush(sensor);

			// read current parameters
			try {
				exposure = GoSetup_Exposure(setup, GO_ROLE_MAIN);
				std::cout << "\tNew Parameters:" << std::endl;
				std::cout << "\t---------------" << std::endl;
				std::cout << "\tExposure: " << exposure << std::endl << std::endl;
			} catch (...) {
				std::cout << "GoSetup_Exposure Exception" << std::endl;
			}
		}
			break;
		default:
			return Error(GENERIC, 0);
	}

	return Error(OK, 1);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr GocatorCV::Gocator::Grab() {
	
	//point cloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());

	cv::Mat img;

	//Get data
	std::cout << "Sensor is running ..." << std::endl << std::endl;

	if (GoSystem_ReceiveData(system, &dataset, RECEIVE_TIMEOUT) == kOK) {

		std::cout << std::endl << "******** Start of GoSystem_ReceiveData ********" << std::endl;
		std::cout << "Data message received: " << std::endl;
		std::cout << "Dataset total count: " << GoDataSet_Count(dataset) << std::endl << std::endl;

		// each result can have multiple data items
		// loop through all items in result message
		float x(0), y(0), z(0), n(0);
		for (i = 0; i < GoDataSet_Count(dataset); ++i) {
			std::cout << "Dataset index count: " << i << std::endl;
			dataObj = GoDataSet_At(dataset, i);

			switch (GoDataMsg_Type(dataObj)) {
			
			case GO_DATA_MESSAGE_TYPE_STAMP:
			{
				GoStampMsg stampMsg = dataObj;
				std::cout << "\tStamp Message batch count: " << (k32u)GoStampMsg_Count(stampMsg) << std::endl;

				for (j = 0; j < GoStampMsg_Count(stampMsg); j++) {
					GoStamp* stamp = GoStampMsg_At(stampMsg, j);
					std::cout << "\tTimestamp: " << stamp->timestamp << std::endl;
					std::cout << "\tEncoder position at leading edge: " << stamp->encoder << std::endl;
					std::cout << "\tFrame index: " << stamp->frameIndex << std::endl;
				}
			}
			break;
			/**/
			case GO_DATA_MESSAGE_TYPE_UNIFORM_SURFACE:
			{
				// cast to GoSurfaceMsg
				GoSurfaceMsg surfaceMsg = dataObj;
				unsigned int rowIdx, colIdx;

				// get offsets and resolutions
				double xResolution = NM_TO_MM(GoSurfaceMsg_XResolution(surfaceMsg));
				double yResolution = NM_TO_MM(GoSurfaceMsg_YResolution(surfaceMsg));
				double zResolution = NM_TO_MM(GoSurfaceMsg_ZResolution(surfaceMsg));
				double xOffset = UM_TO_MM(GoSurfaceMsg_XOffset(surfaceMsg));
				double yOffset = UM_TO_MM(GoSurfaceMsg_YOffset(surfaceMsg));
				double zOffset = UM_TO_MM(GoSurfaceMsg_ZOffset(surfaceMsg));

				//Print raw cloud metadata
				std::cout << "\tSurface Message" << std::endl;
				std::cout << "\tLength: " << (k32u)GoSurfaceMsg_Length(surfaceMsg) << std::endl;
				std::cout << "\tWidth: " << (k32u)GoSurfaceMsg_Width(surfaceMsg) << std::endl;
				std::cout << "\tExposure: " << Gocator::exposure << std::endl;
				std::cout << "\txResolution: " << xResolution << std::endl;
				std::cout << "\tyResolution: " << yResolution << std::endl;
				std::cout << "\tzResolution: " << zResolution << std::endl;
				std::cout << "\txOffset: " << xOffset << std::endl;
				std::cout << "\tyOffset: " << yOffset << std::endl;
				std::cout << "\tzOffset: " << zOffset << std::endl;

				for (rowIdx = 0; rowIdx < GoSurfaceMsg_Length(surfaceMsg); rowIdx++) {
					k16s* data = GoSurfaceMsg_RowAt(surfaceMsg, rowIdx);

					for (colIdx = 0; colIdx < GoSurfaceMsg_Width(surfaceMsg); colIdx++) {
						x = (xOffset + xResolution * colIdx) * -1; // x component inverted to fulfill right-handed frame (Gocator is left-handed!)
						y = (yOffset + yResolution * rowIdx) * 1;

						if (data[colIdx] != INVALID_RANGE_16BIT) {
							z = (zOffset + zResolution * data[colIdx]) * 1;

							cloud->points.emplace_back(pcl::PointXYZ(x, y, z));

						} else {
							continue;
						}
					}
				}
			}
			break;
			
			/*
			case GO_DATA_MESSAGE_TYPE_SURFACE:
			{
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
				std::cout << "\tSurface Message" << std::endl;
				std::cout << "\tLength: " << row_count << std::endl;
				std::cout << "\tWidth: " << width << std::endl;
				std::cout << "\tExposure: " << Gocator::exposure << std::endl;
				std::cout << "\txResolution: " << xResolution << std::endl;
				std::cout << "\tyResolution: " << yResolution << std::endl;
				std::cout << "\tzResolution: " << zResolution << std::endl;
				std::cout << "\txOffset: " << xOffset << std::endl;
				std::cout << "\tyOffset: " << yOffset << std::endl;
				std::cout << "\tzOffset: " << zOffset << std::endl;

				//resize the point cloud
				cloud->height = row_count;
				cloud->width = width;
				cloud->resize(row_count * width);

				//run over all rows
				for (ii = 0; ii < row_count; ii++) {
					//get the pointer to row
					short* data = GoSurfaceMsg_RowAt(surfaceMsg, ii);

					//run over the width of row ii
					for (jj = 0; jj < width; jj++) {

						//set xy in meters. x component inverted to fulfill right-handed frame (Gocator is left-handed!)
						cloud->points.at(ii * width + jj).x = -0.001 * (xOffset + xResolution * jj);
						cloud->points.at(ii * width + jj).y = 0.001 * (yOffset + yResolution * ii);

						//set z  in meters.
						if (data[jj] != INVALID_RANGE_16BIT)
							cloud->points.at(ii * width + jj).z = 0.001 * (zOffset + zResolution * data[jj]);
						else
							cloud->points.at(ii * width + jj).z = 0.001 * (INVALID_RANGE_DOUBLE);
					}
				}
			}
			break;
			*/
			/*
			case GO_DATA_MESSAGE_TYPE_SURFACE_INTENSITY:
			{
				GoSurfaceIntensityMsg surfaceIntMsg = dataObj;
				unsigned int rowIdx, colIdx;
				double XResolution = NM_TO_MM(GoSurfaceIntensityMsg_XResolution(surfaceIntMsg));
				double YResolution = NM_TO_MM(GoSurfaceIntensityMsg_YResolution(surfaceIntMsg));
				double XOffset = UM_TO_MM(GoSurfaceIntensityMsg_XOffset(surfaceIntMsg));
				double YOffset = UM_TO_MM(GoSurfaceIntensityMsg_YOffset(surfaceIntMsg));

				std::cout << "\tSurface intensity width: " << (k32u)GoSurfaceIntensityMsg_Width(surfaceIntMsg) << std::endl;
				std::cout << "\tSurface intensity height: " << (k32u)GoSurfaceIntensityMsg_Length(surfaceIntMsg) << std::endl;

				img = cv::Mat::zeros((int)(GoSurfaceIntensityMsg_Length(surfaceIntMsg)*5) + 1, (int)(GoSurfaceIntensityMsg_Width(surfaceIntMsg)*5) + 1, CV_8UC1);
				std::cout << "rows: " << img.rows << " cols: " << img.cols << std::endl;

				for (int x = 0; x < img.cols; x++) {
					for (int y = 0; y < img.rows; y++) {
						img.at<uchar>(y, x) = 0;
					}
				}

				for (rowIdx = 0; rowIdx < GoSurfaceIntensityMsg_Length(surfaceIntMsg); rowIdx++) {
					k8u* data = GoSurfaceIntensityMsg_RowAt(surfaceIntMsg, rowIdx);

					// gocator transmits intensity data as an 8-bit grayscale image of identical width and height as the corresponding height map
					for (colIdx = 0; colIdx < GoSurfaceIntensityMsg_Width(surfaceIntMsg); colIdx++) {
						x = XOffset + XResolution * colIdx;
						y = YOffset + YResolution * rowIdx;
						n = data[colIdx];
						//std::cout << "XOffset: " << XOffset << " XResolution: " << XResolution << " colIdx: " << colIdx << std::endl;
						//std::cout << "x: " << x << " y: " << y << " n: " << n << std::endl;
						int vx = (int)(x*5 + (img.cols/2));
						int vy = (int)(y*5 + (img.rows/2));
						//std::cout << "vx: " << vx << " vy: " << vy << std::endl;
						img.at<uchar>(vy, vx) = n;
					}
				}
			}
			break;
			*/
			}
		}
		GoDestroy(dataset);
		
		cloud->height = 1;
		cloud->width = cloud->size();
		cloud->resize(cloud->size());

		//cv::imwrite("../../Scan/image_prova_" + datetime() + ".jpg", img);
		
		std::cout << "******** End of GoSystem_ReceiveData ********" << std::endl << std::endl;
		return cloud;
	} else {
		std::cout << "Error: No data received during the waiting period" << std::endl;
		return cloud;
	}
}
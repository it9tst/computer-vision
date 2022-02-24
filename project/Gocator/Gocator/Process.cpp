#include "Process.h"

GocatorCV::Process::Process() {
	// constructor
}

void GocatorCV::Process::StartAcquisition() {
	int count = 0;
	while ((char)cv::waitKey(10) != 'q') {

		//Get data
		std::cout << "\n\nSensor is running ..." << std::endl;

		if (GoSystem_ReceiveData(system, &dataset, RECEIVE_TIMEOUT) == kOK) {
			short int* height_map_memory = NULL;
			unsigned char* intensity_image_memory = NULL;
			ProfilePoint** surfaceBuffer = NULL;
			k32u surfaceBufferHeight = 0;

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

						//pcl::io::savePCDFileASCII("PointCloudGocator" + std::to_string(count) + ".pcd", _p_cloud);
						//pcl::io::savePLYFileASCII("PointCloudGocator" + std::to_string(count) + ".ply", _p_cloud);
						count++;
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
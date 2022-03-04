#pragma once

#include "Error.h"

//std c/c++
#include <iostream>

//GoSdk
#include <GoSdk/GoSdk.h>

//PCL visualiser
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/extract_indices.h>

namespace GocatorCV {

    enum ParameterType {
        SENSOR_IP,
        EXPOSURE
    };

    class Gocator {

        //constants
        #define RECEIVE_TIMEOUT			10000000
        #define INVALID_RANGE_16BIT		((signed short)0x8000)				// gocator transmits range data as 16-bit signed integers. 0x8000 signifies invalid range data.
        #define DOUBLE_MAX				((k64f)1.7976931348623157e+308)		// 64-bit double - largest positive value.
        #define INVALID_RANGE_DOUBLE	((k64f) - DOUBLE_MAX)				// floating point value to represent invalid range data.
        #define NM_TO_MM(VALUE)			(((k64f)(VALUE))/1000000.0)
        #define UM_TO_MM(VALUE)			(((k64f)(VALUE))/1000.0)

    private:
        const char* sensorIp;
        kStatus status;
        kAssembly api = kNULL;
        GoSystem system = kNULL;
        GoSensor sensor = kNULL;
        GoSetup setup = kNULL;
        kChar model_name[50];
        k64f exposure;
        kIpAddress ipAddress;
        GoDataSet dataset = kNULL;
        GoDataMsg dataObj;
        unsigned int i, j, k, ii, jj;

    public:
        Gocator();
        Error Init();
        Error Start();
        Error Stop();
        Error SetParameter(ParameterType name, void* value);
        pcl::PointCloud<pcl::PointXYZ>::Ptr Grab();
    };
}
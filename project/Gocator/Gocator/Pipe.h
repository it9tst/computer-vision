#pragma once

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <windows.h>
#include <iostream>
#include <thread>
#include <nlohmann/json.hpp>
#include <chrono>

namespace GocatorCV {

	struct PCL {
		std::vector<double> x;
		std::vector<double> y;
		std::vector<double> z;
	};

	struct Statistics {
		std::vector<std::string> row;
	};

	class Pipe {

	private:
		std::string string_to_hex(const std::string& input);

	public:
		Pipe();
		void SendPCL(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int id);
		void SendStats(GocatorCV::Statistics stats, int id);
	};
}
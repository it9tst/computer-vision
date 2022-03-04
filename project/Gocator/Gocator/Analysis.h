#pragma once

//OpenCV
#include <opencv2/opencv.hpp>

//PCL visualiser
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>

namespace GocatorCV {

	class Analysis {

	private:
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;

	public:
		Analysis();
		void LoadPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
		void Algorithm();
	};
}
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
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/filters/passthrough.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/obj_io.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_io.h>

namespace GocatorCV {

	class Analysis {

	private:
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
		pcl::PointXYZ minPt, maxPt;
		pcl::PCDWriter writer;

	public:
		Analysis();
		void LoadPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
		void Algorithm();
		void CheckValidPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloudCorrect);
		void StatisticalOutlierRemovalFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFiltered);
		void PlaneSegmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloudSegmented);
		void ProjectPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloudProjected, double a, double b, double c, double d);
		void MatrixTransform(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloudTransformed, double x, double y, double z);
		void GetMinMaxCoordinates(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
		void SavePCD(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string name);
		void Visualization(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
		std::string datetime();
	};
}
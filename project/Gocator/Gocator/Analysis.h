#pragma once

#include <unordered_map>
#include <unordered_set>
#include <list>

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

	struct contourSorter {
		bool operator ()(const std::vector<cv::Point>& a, const std::vector<cv::Point>& b) {
			cv::Rect ra(cv::boundingRect(a));
			cv::Rect rb(cv::boundingRect(b));

			return (ra.x < rb.x);
		}
	};

	class Analysis {

	private:
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
		pcl::PointXYZ minPt, maxPt;
		pcl::PCDWriter writer;
		void StatisticalOutlierRemovalFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFiltered);
		void PlaneSegmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloudSegmented);
		void ProjectPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloudProjected, double a, double b, double c, double d);
		void MatrixTransform(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloudTransformed, double x, double y, double z);
		void GetMinMaxCoordinates(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
		void SavePCD(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string name);
		void Visualization(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
		std::string datetime();

		void measurements(std::vector<std::vector<cv::Point>> contours);
		double distanceCalculate(int x1, int y1, int x2, int y2);
		double cntDistanceCompare(std::vector<cv::Point> contoursA, std::vector<cv::Point> contoursB);
		std::vector<std::vector<cv::Point>> contourDetection(cv::Mat image);
		cv::Point getCenter(std::vector<cv::Point> contours);
		cv::Mat morphClosingBlob(cv::Mat blobIn);
		cv::Mat morphClosingMacroBlob(cv::Mat blobIn);
		void distanceMacroBlob(std::vector<std::vector<cv::Point>> contoursMacroBlob);
		void distanceBlob(std::vector<std::vector<cv::Point>> contoursMacroBlob, std::vector<std::vector<cv::Point>> contoursBlob);

	public:
		Analysis();
		void LoadPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
		void Algorithm();
		void CheckValidPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloudCorrect);
	};
}
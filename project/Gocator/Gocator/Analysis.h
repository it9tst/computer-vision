#pragma once

#include <list>
#include <tuple>

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

//GNU Scientific Library
#include <gsl/gsl_math.h>
#include <gsl/gsl_filter.h>
#include <gsl/gsl_vector.h>

//Matplotlib
#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;


namespace GocatorCV {

	struct ContourSorter {
		bool operator ()(const std::vector<cv::Point>& a, const std::vector<cv::Point>& b) {
			cv::Rect ra(cv::boundingRect(a));
			cv::Rect rb(cv::boundingRect(b));

			return (ra.x < rb.x);
		}
	};

	struct PolynomialFunction {
		std::vector<double> x;
		std::vector<double> y;
		std::vector<int> i;
	};

	class Analysis {

	private:
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
		pcl::PointXYZ min_pt, max_pt;
		pcl::PCDWriter writer;
		std::string datetime();
		void GetMinMaxCoordinates(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
		void SavePCD(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string name);
		void Visualization(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

		// analisi pointcloud battistrada1 e battistrada2
		void StatisticalOutlierRemovalFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered);
		void PlaneSegmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_segmented, double value_threshold);
		void ProjectPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected, double a, double b, double c, double d);
		void MatrixTransform(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transformed, double x, double y, double z);

		// analisi immagini opencv battistrada1 e battistrada2
		cv::Mat MorphClosingBlob(cv::Mat image);
		cv::Mat MorphClosingMacroBlob(cv::Mat image);
		cv::Point ContourCenter(std::vector<cv::Point> contour);
		std::vector<std::vector<cv::Point>> ContoursDetection(cv::Mat image);
		double MinDistanceContours(std::vector<cv::Point> contourA, std::vector<cv::Point> contourB);
		double DistanceBetweenTwoPoints(int x1, int y1, int x2, int y2);
		void ContoursMeasurements(std::vector<std::vector<cv::Point>> contours);
		void DistanceBetweenMacroBlob(std::vector<std::vector<cv::Point>> contours_MacroBlob);
		void DistanceBetweenBlob(std::vector<std::vector<cv::Point>> contours_MacroBlob, std::vector<std::vector<cv::Point>> contours_Blob);

		// analisi profilo battistrada3
		GocatorCV::PolynomialFunction GaussianFilter(GocatorCV::PolynomialFunction line);
		GocatorCV::PolynomialFunction DifferenceQuotient(GocatorCV::PolynomialFunction line);
		void GetLine(double x1, double y1, double x2, double y2, double& a, double& b, double& c);
		double DistancePointLine(double pct1X, double pct1Y, double pct2X, double pct2Y, double pct3X, double pct3Y);
		cv::Point2d RotatePoint(double cx, double cy, double angle, cv::Point2d p);

	public:
		Analysis();
		void LoadPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
		void Algorithm();
		void CheckValidPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_correct);
	};
}
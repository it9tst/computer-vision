#include "Analysis.h"

GocatorCV::Analysis::Analysis() {
	// constructor
}

void GocatorCV::Analysis::LoadPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    this->cloud = cloud;
}

void GocatorCV::Analysis::Algorithm() {
    
    // Erase line (-inf)
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudProjected(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    pcl::ExtractIndices<pcl::PointXYZ> extract;

    for (int i = 0; i < (*cloud).size(); i++) {
        pcl::PointXYZ pt(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);
        if (!pcl::isFinite(pt)){
            inliers->indices.push_back(i);
        }

        // Resize
        cloud->points[i].x = cloud->points[i].x * 50;
        cloud->points[i].y = cloud->points[i].y * 50;
        cloud->points[i].z = cloud->points[i].z * 50;
    }
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloudProjected);


    /*
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    std::cerr << "Cloud before filtering: " << std::endl;
    std::cerr << *cloudProjected << std::endl;

    // Create the filtering object
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloudProjected);
    sor.setMeanK(50);
    sor.setStddevMulThresh(1.0);
    sor.filter(*cloud_filtered);

    std::cerr << "Cloud after filtering: " << std::endl;
    std::cerr << *cloud_filtered << std::endl;

    pcl::PCDWriter writer;
    writer.write<pcl::PointXYZ>("table_scene_lms400_inliers.pcd", *cloud_filtered, false);

    sor.setNegative(true);
    sor.filter(*cloud_filtered);
    writer.write<pcl::PointXYZ>("table_scene_lms400_outliers.pcd", *cloud_filtered, false);
    */

    // Visualization
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("cloud", true));

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red(cloudProjected, 255, 0, 0);
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ>(cloudProjected, red, "cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();

    // Main loop
    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}
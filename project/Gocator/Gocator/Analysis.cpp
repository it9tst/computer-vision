#include "Analysis.h"

GocatorCV::Analysis::Analysis() {
	// constructor
}

void GocatorCV::Analysis::LoadPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    this->cloud = cloud;
}

void GocatorCV::Analysis::Algorithm() {
    
    // Erase line with (-inf) value
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudCorrect(new pcl::PointCloud<pcl::PointXYZ>);
    CheckValidPoints(cloud, cloudCorrect);
    
    // Removing outliers using a StatisticalOutlierRemoval filter
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFiltered(new pcl::PointCloud<pcl::PointXYZ>);
    StatisticalOutlierRemovalFilter(cloudCorrect, cloudFiltered);

    // Plane model segmentation
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudSegmented(new pcl::PointCloud<pcl::PointXYZ>);
    PlaneSegmentation(cloudFiltered, cloudSegmented);

    GetMinMaxCoordinates(cloudSegmented);

    // Projecting points using a parametric model
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudProjected(new pcl::PointCloud<pcl::PointXYZ>);
    ProjectPoints(cloudSegmented, cloudProjected, 0, 0, 1, 0);

    GetMinMaxCoordinates(cloudProjected);

    // Using a matrix to transform a point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>());

    // Center
    //double x = -(maxPt.x + minPt.x) / 2;
    //double y = -(maxPt.y + minPt.y) / 2;


    double x = -minPt.x;
    double y = -minPt.y;
    MatrixTransform(cloudProjected, transformed_cloud, x, y, 0);

    GetMinMaxCoordinates(transformed_cloud);

    
    std::cout << (int)(maxPt.x * 100) << std::endl;
    std::cout << (int)(maxPt.y * 100) << std::endl;
    cv::Mat img = cv::Mat::zeros((int)(maxPt.x * 100)+1, (int)(maxPt.y * 100)+1, CV_8UC1);

    for (int x = 0; x < img.rows; x++) {
        for (int y = 0; y < img.cols; y++) {
            img.at<uchar>(x, y) = 0;
        }
    }

    std::cout << (*transformed_cloud).size() << std::endl;
    Sleep(5000);
    
    for (int i = 0; i < (*transformed_cloud).size(); i++) {
        //std::cout << (int)(transformed_cloud->points[i].x * 100) << std::endl;
        //std::cout << (int)(transformed_cloud->points[i].y * 100) << std::endl;
        //std::cout << i << std::endl;
        img.at<uchar>((int)(transformed_cloud->points[i].x * 100), (int)(transformed_cloud->points[i].y * 100)) = 254;
    }

    cv::imwrite("Scan/image.jpg", img);


    // Visualising a point cloud
    //Visualization(transformed_cloud);
}

void GocatorCV::Analysis::CheckValidPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloudCorrect) {

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    pcl::ExtractIndices<pcl::PointXYZ> extract;

    for (int i = 0; i < (*cloud).size(); i++) {
        pcl::PointXYZ pt(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);
        if (!pcl::isFinite(pt)) {
            inliers->indices.push_back(i);
        }

        // Resize
        cloud->points[i].x = cloud->points[i].x * 100;
        cloud->points[i].y = cloud->points[i].y * 100;
        cloud->points[i].z = cloud->points[i].z * 100;
    }
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloudCorrect);
}

void GocatorCV::Analysis::StatisticalOutlierRemovalFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFiltered) {

    std::cerr << "Cloud before filtering:" << std::endl;
    std::cerr << *cloud << std::endl;

    // Create the filtering object
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(50);
    sor.setStddevMulThresh(1.0);
    sor.filter(*cloudFiltered);

    std::cerr << "Cloud after filtering:" << std::endl;
    std::cerr << *cloudFiltered << std::endl;

    PCL_INFO("Saving statistical outlier removal filter in input cloud to *.pcd\n\n");
    SavePCD(cloudFiltered, "Scan/" + datetime() + "_Statistical_Outlier_Removal_Filter.pcd");
/*
    sor.setNegative(true);
    sor.filter(*cloudFiltered);
    SavePCD(cloudFiltered, "Scan/table_scene_lms400_outliers.pcd");
*/
}

void GocatorCV::Analysis::PlaneSegmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloudSegmented) {

    pcl::PointCloud<pcl::PointXYZ>::Ptr outliers(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr outliersSegmented(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;

    // Optional
    seg.setOptimizeCoefficients(true);

    // Mandatory
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.5); // 0.1 or 0.5

    // Segment dominant plane
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);
    pcl::copyPointCloud<pcl::PointXYZ>(*cloud, *inliers, *cloudSegmented);

    if (inliers->indices.size() == 0) {
        PCL_ERROR("Could not estimate a planar model for the given dataset.\n\n");
    } else {
        PCL_INFO("Saving dominant plane in input cloud to *.pcd\n\n");
        SavePCD(cloudSegmented, "Scan/" + datetime() + "_Plane_Segmentation.pcd");
    }
/*
    // Remove inliers from input and repeat for 2nd dominant plane
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*outliers);

    // Run segmentation on outliers
    seg.setInputCloud(outliers);
    seg.segment(*inliers, *coefficients);
    pcl::copyPointCloud<pcl::PointXYZ>(*outliers, *inliers, *outliersSegmented);

    if (inliers->indices.size() == 0) {
        PCL_ERROR("Could not estimate a planar model for the given dataset.");
    } else {
        PCL_INFO("Saving dominant plane in outliers to: table_scene_lms400_second_plane.pcd\n\n");
        SavePCD(outliersSegmented, "Scan/table_scene_lms400_second_plane.pcd");
    }
*/
}

void GocatorCV::Analysis::ProjectPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloudProjected, double a, double b, double c, double d) {
    
    // Create a set of planar coefficients with ax+by+cz+d=0
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    coefficients->values.resize(4);
    coefficients->values[0] = a;    // a
    coefficients->values[1] = b;    // b
    coefficients->values[2] = c;    // c
    coefficients->values[3] = d;    // d

    // Create the filtering object
    pcl::ProjectInliers<pcl::PointXYZ> proj;
    proj.setModelType(pcl::SACMODEL_PLANE);
    proj.setInputCloud(cloud);
    proj.setModelCoefficients(coefficients);
    proj.filter(*cloudProjected);

    PCL_INFO("Saving project points in input cloud to *.pcd\n\n");
    SavePCD(cloudProjected, "Scan/" + datetime() + "_Project_Points.pcd");
}

void GocatorCV::Analysis::MatrixTransform(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloudTransformed, double x, double y, double z) {

    Eigen::Affine3f transform = Eigen::Affine3f::Identity();

    // Define a translation on the x, y, z axis
    transform.translation() << x, y, z;

    // You can either apply transform_1 or transform_2; they are the same
    pcl::transformPointCloud(*cloud, *cloudTransformed, transform);

    PCL_INFO("Saving matrix transformation in input cloud to *.pcd\n\n");
    SavePCD(cloudTransformed, "Scan/" + datetime() + "_Matrix_Transformation.pcd");
}

void GocatorCV::Analysis::GetMinMaxCoordinates(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {

    pcl::getMinMax3D(*cloud, minPt, maxPt);

    std::cout << "Max x: " << maxPt.x << std::endl;
    std::cout << "Min x: " << minPt.x << std::endl;
    std::cout << "Max y: " << maxPt.y << std::endl;
    std::cout << "Min y: " << minPt.y << std::endl;
    std::cout << "Max z: " << maxPt.z << std::endl;
    std::cout << "Min z: " << minPt.z << std::endl << std::endl;
}

void GocatorCV::Analysis::SavePCD(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string name) {

    writer.write<pcl::PointXYZ>(name, *cloud, false);
}

void GocatorCV::Analysis::Visualization(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {

    vtkObject::GlobalWarningDisplayOff();

    // Visualization
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("cloud", true));

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red(cloud, 255, 0, 0);
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ>(cloud, red, "cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();

    // Main loop
    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

std::string GocatorCV::Analysis::datetime() {

    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);
    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y_%m_%d_%H_%M_%S");
    std::string name = oss.str();

    return std::string(name);
}
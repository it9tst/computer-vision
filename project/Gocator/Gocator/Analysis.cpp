#include "Analysis.h"

GocatorCV::Analysis::Analysis() {
	// constructor
}

void GocatorCV::Analysis::LoadPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    this->cloud = cloud;
}

void GocatorCV::Analysis::Algorithm() {
    /*
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
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformedCloud(new pcl::PointCloud<pcl::PointXYZ>());

    // Center
    //double x = -(maxPt.x + minPt.x) / 2;
    //double y = -(maxPt.y + minPt.y) / 2;

    
    double x = -minPt.x;
    double y = -minPt.y;
    MatrixTransform(cloudProjected, transformedCloud, x, y, 0);
    */
    GetMinMaxCoordinates(cloud);

    
    std::cout << "Width: " << (int)(maxPt.x * 100) << std::endl;
    std::cout << "Height: " << (int)(maxPt.y * 100) << std::endl;

    cv::Mat img = cv::Mat::zeros((int)(maxPt.y * 100) + 1, (int)(maxPt.x * 100) + 1, CV_8UC1);

    for (int x = 0; x < img.cols; x++) {
        for (int y = 0; y < img.rows; y++) {
            img.at<uchar>(y, x) = 0;
        }
    }

    std::cout << "Cloud size: " << (*cloud).size() << std::endl;
    
    for (int i = 0; i < (*cloud).size(); i++) {
        //std::cout << (int)(transformed_cloud->points[i].x * 100) << std::endl;
        //std::cout << (int)(transformed_cloud->points[i].y * 100) << std::endl;
        //std::cout << i << std::endl;
        img.at<uchar>((int)(cloud->points[i].y * 100), (int)(cloud->points[i].x * 100)) = 254;
    }

    cv::rotate(img, img, cv::ROTATE_90_CLOCKWISE);
    cv::flip(img, img, 1);

    cv::imwrite("Scan/image.jpg", img);
    
    // BLOB - Morphological Transformations - Closing
    cv::Mat closingBlob = morphClosingBlob(img);

    // MACRO_BLOB - Morphological Transformations - Closing
    cv::Mat closingMacroBlob = morphClosingMacroBlob(closingBlob);
    
    // MACRO_BLOB - Contour Detection
    std::vector<std::vector<cv::Point>> contoursMacroBlob = contourDetection(closingMacroBlob);

    // MACRO_BLOB - Distance, Area, Perimeter, Width and Height Measurements
    distanceMacroBlob(contoursMacroBlob);
    measurements(contoursMacroBlob);

    // BLOB - Contour Detection
    std::vector<std::vector<cv::Point>> contoursBlob = contourDetection(closingBlob);

    if (contoursBlob.size() != contoursMacroBlob.size()) {
        // BLOB - Distance, Area, Perimeter, Width and Height Measurements
        measurements(contoursBlob);

        // MACRO_BLOB - BLOB - Distance
        distanceBlob(contoursMacroBlob, contoursBlob);
    }

    // Visualising a point cloud
    Visualization(cloud);
}

void GocatorCV::Analysis::CheckValidPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloudCorrect) {
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    pcl::ExtractIndices<pcl::PointXYZ> extract;

    for (int i = 0; i < (*cloud).size(); i++) {
        pcl::PointXYZ pt(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);
        /*
        if (!pcl::isFinite(pt)) {
            inliers->indices.push_back(i);
        }
        */

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
    seg.setDistanceThreshold(0.1); // 0.1 or 0.5

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

void GocatorCV::Analysis::measurements(std::vector<std::vector<cv::Point>> contours) {
    std::vector<double> statsArea;
    std::vector<double> statsPerimeter;
    std::vector<double> statsWidth;
    std::vector<double> statsHeight;
    double area;
    double perimeter;
    cv::RotatedRect rotatedRect;

    for (int i = 0; i < contours.size(); i++) {
        area = cv::contourArea(contours[i]);
        perimeter = cv::arcLength(contours[i], true);
        rotatedRect = cv::minAreaRect(contours[i]);

        if (rotatedRect.angle > -45) {
            std::cout << "Blob " << i << "\t- Area: " << setprecision(2) << std::fixed << area / 100 << " cm^2\tPerimetro: " << setprecision(2) << std::fixed << perimeter / 100 << " cm\tWidth: " << setprecision(2) << std::fixed << rotatedRect.size.width / 100 << " cm\tHeight: " << setprecision(2) << std::fixed << rotatedRect.size.height / 100 << " cm" << std::endl;
            statsWidth.push_back(rotatedRect.size.width);
            statsHeight.push_back(rotatedRect.size.height);
        } else {
            std::cout << "Blob " << i << "\t- Area: " << setprecision(2) << std::fixed << area / 100 << " cm^2\tPerimetro: " << setprecision(2) << std::fixed << perimeter / 100 << " cm\tWidth: " << setprecision(2) << std::fixed << rotatedRect.size.height / 100 << " cm\tHeight: " << setprecision(2) << std::fixed << rotatedRect.size.width / 100 << " cm" << std::endl;
            statsWidth.push_back(rotatedRect.size.height);
            statsHeight.push_back(rotatedRect.size.width);
        }

        statsArea.push_back(area);
        statsPerimeter.push_back(perimeter);
    }
    
    std::cout << std::endl << "Area Minima: " << *std::min_element(statsArea.begin(), statsArea.end()) / 100 << " cm\t\tArea Massima: " << setprecision(2) << std::fixed << *std::max_element(statsArea.begin(), statsArea.end()) / 100 << " cm\t\tArea Media: " << setprecision(2) << std::fixed << (std::accumulate(statsArea.begin(), statsArea.end(), 0.0) / statsArea.size()) / 100 << " cm" << std::endl;
    std::cout << "Perimetro Minimo: " << *std::min_element(statsPerimeter.begin(), statsPerimeter.end()) / 100 << " cm\t\tPerimetro Massimo: " << setprecision(2) << std::fixed << *std::max_element(statsPerimeter.begin(), statsPerimeter.end()) / 100 << " cm\t\tPerimetro Medio: " << setprecision(2) << std::fixed << (std::accumulate(statsPerimeter.begin(), statsPerimeter.end(), 0.0) / statsPerimeter.size()) / 100 << " cm" << std::endl;
    std::cout << "Width Minima: " << *std::min_element(statsWidth.begin(), statsWidth.end()) / 100 << " cm\t\tWidth Massima: " << setprecision(2) << std::fixed << *std::max_element(statsWidth.begin(), statsWidth.end()) / 100 << " cm\t\tArea Width: " << setprecision(2) << std::fixed << (std::accumulate(statsWidth.begin(), statsWidth.end(), 0.0) / statsWidth.size()) / 100 << " cm" << std::endl;
    std::cout << "Height Minima: " << *std::min_element(statsHeight.begin(), statsHeight.end()) / 100 << " cm\t\tHeight Massima: " << setprecision(2) << std::fixed << *std::max_element(statsHeight.begin(), statsHeight.end()) / 100 << " cm\t\tHeight Media: " << setprecision(2) << std::fixed << (std::accumulate(statsHeight.begin(), statsHeight.end(), 0.0) / statsHeight.size()) / 100 << " cm" << std::endl;
    
    std::cout << std::endl;
}

double GocatorCV::Analysis::distanceCalculate(int x1, int y1, int x2, int y2) {
    return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2) * 1.0);
}

double GocatorCV::Analysis::cntDistanceCompare(std::vector<cv::Point> contoursA, std::vector<cv::Point> contoursB) {
    double minimo = -1;

    for (int j = 0; j < contoursA.size(); j++) {
        for (int k = 0; k < contoursB.size(); k++) {
            double distanza = distanceCalculate(contoursA[j].x, contoursA[j].y, contoursB[k].x, contoursB[k].y);
            if (minimo== -1 || distanza < minimo) {
                minimo = distanza;
            }
        }
    }

    return minimo;
}

std::vector<std::vector<cv::Point>> GocatorCV::Analysis::contourDetection(cv::Mat image) {
    // convert the image to grayscale format
    //cv::Mat img_gray;
    //cv::cvtColor(image, img_gray, cv::COLOR_BGR2GRAY);

    // apply binary thresholding
    cv::Mat thresh_gray;
    cv::threshold(image, thresh_gray, 200, 255, cv::THRESH_BINARY);

    // detect the contours on the binary image using cv2.CHAIN_APPROX_NONE
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    findContours(thresh_gray, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

    // sort the contours from left to right and top to bottom
    std::sort(contours.begin(), contours.end(), contourSorter());

    std::vector<std::vector<cv::Point>> cnts;
    for (int i = 0; i < contours.size(); i++) {
        if (cv::contourArea(contours[i]) > 0) {
            cnts.push_back(contours[i]);
        }
    }
    /*
    cv::RNG rng(12345);
    std::vector<cv::Rect> boundRect(cnts.size());
    cv::Mat drawing = cv::Mat::zeros(thresh_gray.size(), CV_8UC3);

    for (int i = 0; i < cnts.size(); i++) {
        cv::Scalar color = cv::Scalar(rng.uniform(0, 256), rng.uniform(0, 256), rng.uniform(0, 256));
        cv::drawContours(drawing, cnts, (int)i, color, 2, cv::LINE_8, hierarchy, 0);
        boundRect[i] = cv::boundingRect(cnts[i]);
        cv::putText(drawing, std::to_string(i), cv::Point(boundRect[i].x + (boundRect[i].width / 2) , boundRect[i].y + (boundRect[i].height / 2)), cv::FONT_HERSHEY_SIMPLEX, 0.9, cv::Scalar(0, 255, 0), 2, false);
    }

    cv::resize(drawing, drawing, cv::Size(drawing.cols / 2, drawing.rows / 2));
    cv::imshow("Output image", drawing);
    cv::waitKey(5000);
    */
    return cnts;
}

cv::Point GocatorCV::Analysis::getCenter(std::vector<cv::Point> contours) {
    cv::Moments M = cv::moments(contours);
    cv::Point center(M.m10 / M.m00, M.m01 / M.m00);
    return center;
}

cv::Mat GocatorCV::Analysis::morphClosingBlob(cv::Mat blobIn) {
    cv::Mat blobOut;
    cv::Mat kernel_blob(5, 10, CV_8U);
    cv::morphologyEx(blobIn, blobOut, cv::MORPH_CLOSE, kernel_blob);

    return blobOut;
}

cv::Mat GocatorCV::Analysis::morphClosingMacroBlob(cv::Mat blobIn) {
    cv::Mat blobOut;
    cv::Mat kernel_macro_blob = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(30, 100));
    cv::morphologyEx(blobIn, blobOut, cv::MORPH_CLOSE, kernel_macro_blob);

    return blobOut;
}

void GocatorCV::Analysis::distanceMacroBlob(std::vector<std::vector<cv::Point>> contoursMacroBlob) {
    std::vector<double> stats;
    for (int i = 0; i < contoursMacroBlob.size() - 1; i++) {
        double d = cntDistanceCompare(contoursMacroBlob[i], contoursMacroBlob[i + 1]);
        std::cout << "MacroBlob " << i << " - MacroBlob " << i + 1 << " -- Distanza: " << setprecision(2) << std::fixed << d / 100 << std::endl;
        stats.push_back(d);
    }

    std::cout << "Distanza Minima: " << setprecision(2) << std::fixed << *std::min_element(stats.begin(), stats.end()) / 100 << " cm\tDistanza Massima: " << setprecision(2) << std::fixed << *std::max_element(stats.begin(), stats.end()) / 100 << " cm\tDistanza Media: " << setprecision(2) << std::fixed << (std::accumulate(stats.begin(), stats.end(), 0.0) / stats.size()) / 100 << " cm" << std::endl << std::endl;
}

void GocatorCV::Analysis::distanceBlob(std::vector<std::vector<cv::Point>> contoursMacroBlob, std::vector<std::vector<cv::Point>> contoursBlob) {
    std::vector <std::set<int>> macroBlob;
    double x1, y1, ppt1;

    for (int i = 0; i < contoursMacroBlob.size(); i++) {
        std::set<int> blob;

        for (int j = 0; j < contoursBlob.size(); j++) {
            x1 = contoursBlob[j][0].x;
            y1 = contoursBlob[j][0].y;
            ppt1 = cv::pointPolygonTest(contoursMacroBlob[i], cv::Point(x1, y1), false);
            if (ppt1 == 0 or ppt1 == 1) {
                blob.insert(j);
            }
        }
        macroBlob.push_back(blob);
    }
    /*
    std::set<int>::iterator itr;
    for (auto v : macro_blob) {
        for (itr = v.begin(); itr != v.end(); itr++) {
            cout << *itr << " ";
        }
        cout << endl;
    }
    */

    std::set<int>::iterator j;
    std::set<int>::iterator k;

    for (int i = 0; i < macroBlob.size(); i++) {
        std::set<int> blob = macroBlob[i];
        std::set<std::vector<int>> checkBlob;

        for (j = blob.begin(); j != blob.end(); j++) {
            double minimoAttuale = FLT_MAX;
            int indiceMinimoAttuale = -1;
            std::set<int> blobCopy(blob);
            blobCopy.erase(*j);

            for (k = blobCopy.begin(); k != blobCopy.end(); k++) {
                cv::Point center1 = getCenter(contoursBlob[*j]);
                cv::Point center2 = getCenter(contoursBlob[*k]);
                double centerDistance = distanceCalculate(center1.x, center1.y, center2.x, center2.y);

                if (centerDistance < minimoAttuale) {
                    minimoAttuale = centerDistance;
                    indiceMinimoAttuale = *k;
                }

                std::vector<int> check{ *j, indiceMinimoAttuale };
                std::sort(check.begin(), check.end());

                if (indiceMinimoAttuale > -1 && checkBlob.count({ check[0], check[1] }) == 0) {
                    double d = cntDistanceCompare(contoursBlob[check[0]], contoursBlob[check[1]]);
                    std::cout << "MacroBlob " << i << " -- Blob " << check[0] << " - Blob " << check[1] << " -- Distanza: " << setprecision(2) << std::fixed << d / 100 << " cm" << std::endl;
                    checkBlob.insert({ check[0], check[1] });
                }
            }
        }
    }
}

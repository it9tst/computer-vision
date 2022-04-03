#include "Analysis.h"

GocatorCV::Analysis::Analysis() {
	// constructor
}

void GocatorCV::Analysis::LoadPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    this->cloud = cloud;
}

void GocatorCV::Analysis::Algorithm() {
    
    // *****ANALISI IMMAGINI (BATTISTRADA 1-2)*****
    /*
    // Erase line with (-inf) value
    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_correct(new pcl::PointCloud<pcl::PointXYZ>);
    //CheckValidPoints(cloud, cloud_correct);
    
    // Removing outliers using a StatisticalOutlierRemoval filter
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    StatisticalOutlierRemovalFilter(cloud, cloud_filtered);

    // Plane model segmentation
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_segmented(new pcl::PointCloud<pcl::PointXYZ>);
    PlaneSegmentation(cloud_filtered, cloud_segmented, 5); // valore threshold (battistrada1: 1 - battistrada2: 5)

    GetMinMaxCoordinates(cloud_segmented);

    // Projecting points using a parametric model
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZ>);
    ProjectPoints(cloud_segmented, cloud_projected, 0, 0, 1, 0);

    GetMinMaxCoordinates(cloud_projected);
    
    // Using a matrix to transform a point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZ>());

    // Center
    //double x = -(max_pt.x + min_pt.x) / 2;
    //double y = -(max_pt.y + min_pt.y) / 2;

    double x = -min_pt.x;
    double y = -min_pt.y;
    MatrixTransform(cloud_projected, cloud_transformed, x, y, 0);
    
    GetMinMaxCoordinates(cloud_transformed);


    // INIT OPENCV
    std::cout << "Image Width: " << (int)(max_pt.x * 10) << std::endl;
    std::cout << "Image Height: " << (int)(max_pt.y * 10) << std::endl;

    cv::Mat img = cv::Mat::zeros((int)(max_pt.y * 10) + 1, (int)(max_pt.x * 10) + 1, CV_8UC1);

    for (int x = 0; x < img.cols; x++) {
        for (int y = 0; y < img.rows; y++) {
            img.at<uchar>(y, x) = 0;
        }
    }

    std::cout << "Cloud size: " << (*cloud_transformed).size() << std::endl;

    for (int i = 0; i < (*cloud_transformed).size(); i++) {
        //std::cout << (int)(transformed_cloud->points[i].x * 10) << std::endl;
        //std::cout << (int)(transformed_cloud->points[i].y * 10) << std::endl;
        //std::cout << i << std::endl;
        img.at<uchar>((int)(cloud_transformed->points[i].y * 10), (int)(cloud_transformed->points[i].x * 10)) = 254; // from mm to �m
    }

    cv::rotate(img, img, cv::ROTATE_90_CLOCKWISE);
    cv::flip(img, img, 1);

    cv::imwrite("Scan/" + datetime() + "_image.jpg", img);

    // BLOB - Morphological Transformations - Closing
    cv::Mat closing_Blob = MorphClosingBlob(img);

    // MACRO_BLOB - Morphological Transformations - Closing
    cv::Mat closing_MacroBlob = MorphClosingMacroBlob(closing_Blob);

    // MACRO_BLOB - Contour Detection
    std::vector<std::vector<cv::Point>> contours_MacroBlob = ContoursDetection(closing_MacroBlob);

    // MACRO_BLOB - Distance, Area, Perimeter, Width and Height Measurements
    DistanceBetweenMacroBlob(contours_MacroBlob);
    ContoursMeasurements(contours_MacroBlob);

    // BLOB - Contour Detection
    std::vector<std::vector<cv::Point>> contours_Blob = ContoursDetection(closing_Blob);

    if (contours_Blob.size() != contours_MacroBlob.size()) {
        // BLOB - Distance, Area, Perimeter, Width and Height Measurements
        ContoursMeasurements(contours_Blob);

        // MACRO_BLOB - BLOB - Distance
        DistanceBetweenBlob(contours_MacroBlob, contours_Blob);
    }
    */
    

    
    // *****PER TUTTE LE LINEE (BATTISTRADA 3)*****
    /**/
    std::set<float> y_line;
    for (int i = 0; i < cloud->size(); i++) {
        y_line.insert(cloud->points[i].y);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final(new pcl::PointCloud<pcl::PointXYZ>());

    std::vector<std::vector<double>> distance_for_line;
    std::set<float>::iterator k;

    for (k = y_line.begin(); k != y_line.end(); ++k) {
        std::cout << "Linea y numero: " << *k << std::endl;

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_line(new pcl::PointCloud<pcl::PointXYZ>());

        for (int i = 0; i < cloud->size(); i++) {
            if (cloud->points[i].y == *k) {
                cloud_line->points.emplace_back(pcl::PointXYZ(cloud->points[i].x * 1, cloud->points[i].y * 0, cloud->points[i].z * 1));
            }
        }

        GocatorCV::PolynomialFunction line_original;

        for (int i = 0; i < cloud_line->size(); i++) {
            line_original.x.push_back(cloud_line->points[i].x);
            line_original.y.push_back(cloud_line->points[i].z);
            line_original.i.push_back(i);
        }

        // Calcolo della funzione smussata
        GocatorCV::PolynomialFunction line_smooth = GaussianFilter(line_original);

        // Calcolo tutti i minimi e i massimi della funziona smussata
        GocatorCV::PolynomialFunction min_max_point_line_smooth = DifferenceQuotient(line_smooth);

        // Calcolo i massimi (da destra) della funzione originale
        GocatorCV::PolynomialFunction max_point_line_original_right;

        for (int i = 0; i < min_max_point_line_smooth.i.size(); i = i + 2) {
            max_point_line_original_right.x.push_back(line_original.x[min_max_point_line_smooth.i[i]]);
            max_point_line_original_right.y.push_back(line_original.y[min_max_point_line_smooth.i[i]]);
            max_point_line_original_right.i.push_back(line_original.i[min_max_point_line_smooth.i[i]]);
        }

        // Calcolo i minimi della funzione originale
        GocatorCV::PolynomialFunction min_point_line_original;

        for (int i = 1; i < max_point_line_original_right.i.size(); i++) {
            GocatorCV::PolynomialFunction line;

            for (int j = 0; j < line_original.i.size(); j++) {
                if (line_original.x[j] >= max_point_line_original_right.x[i] && line_original.x[j] <= max_point_line_original_right.x[i - 1]) {
                    line.x.push_back(line_original.x[j]);
                    line.y.push_back(line_original.y[j]);
                    line.i.push_back(j);
                }
            }

            std::vector<double>::iterator min = std::min_element(line.y.begin(), line.y.end());
            int argminVal = std::distance(line.y.begin(), min);

            min_point_line_original.x.push_back(line_original.x[line.i[argminVal]]);
            min_point_line_original.y.push_back(line_original.y[line.i[argminVal]]);
            min_point_line_original.i.push_back(line.i[argminVal]);
        }

        // Calcolo i massimi (da sinistra) della funzione originale
        GocatorCV::PolynomialFunction max_point_line_original_left;

        for (int i = 1; i < max_point_line_original_right.i.size(); i++) {
            GocatorCV::PolynomialFunction line_rotated;
            double m = (max_point_line_original_right.y[i] - min_point_line_original.y[i - 1]) / (min_point_line_original.x[i - 1] - max_point_line_original_right.x[i]);

            for (int j = 0; j < line_smooth.i.size(); j++) {
                if (line_smooth.x[j] >= max_point_line_original_right.x[i] && line_smooth.x[j] <= min_point_line_original.x[i - 1]) {
                    cv::Point2d p = RotatePoint(0, 0, m, cv::Point2d(line_smooth.x[j], line_smooth.y[j]));
                    line_rotated.x.push_back(p.x);
                    line_rotated.y.push_back(p.y);
                    line_rotated.i.push_back(j);
                }
            }

            std::vector<double>::iterator max = std::max_element(line_rotated.y.begin(), line_rotated.y.end());
            int argmaxVal = std::distance(line_rotated.y.begin(), max);

            max_point_line_original_left.x.push_back(line_original.x[line_rotated.i[argmaxVal]]);
            max_point_line_original_left.y.push_back(line_original.y[line_rotated.i[argmaxVal]]);
            max_point_line_original_left.i.push_back(line_rotated.i[argmaxVal]);
        }

        // Calcolo delle distanze tra la retta che congiunge due massimi adiacenti e il minimo corrispondente
        GocatorCV::PolynomialFunction min_max_point_line_original;
        GocatorCV::PolynomialFunction max_point_line_original;
        std::vector<double> distance_line;

        for (int i = 1; i < max_point_line_original_right.i.size(); i++) {
            double d = DistancePointLine(min_point_line_original.x[i - 1], min_point_line_original.y[i - 1], max_point_line_original_right.x[i - 1], max_point_line_original_right.y[i - 1], max_point_line_original_left.x[i - 1], max_point_line_original_left.y[i - 1]);

            if (d > 0.1) {
                min_max_point_line_original.x.push_back(max_point_line_original_right.x[i - 1]);
                min_max_point_line_original.y.push_back(max_point_line_original_right.y[i - 1]);
                min_max_point_line_original.i.push_back(max_point_line_original_right.i[i - 1]);
                min_max_point_line_original.x.push_back(min_point_line_original.x[i - 1]);
                min_max_point_line_original.y.push_back(min_point_line_original.y[i - 1]);
                min_max_point_line_original.i.push_back(min_point_line_original.i[i - 1]);
                min_max_point_line_original.x.push_back(max_point_line_original_left.x[i - 1]);
                min_max_point_line_original.y.push_back(max_point_line_original_left.y[i - 1]);
                min_max_point_line_original.i.push_back(max_point_line_original_left.i[i - 1]);
                distance_line.push_back(d);
                max_point_line_original.x.push_back(max_point_line_original_right.x[i - 1]);
                max_point_line_original.y.push_back(max_point_line_original_right.y[i - 1]);
                max_point_line_original.i.push_back(max_point_line_original_right.i[i - 1]);
                max_point_line_original.x.push_back(max_point_line_original_left.x[i - 1]);
                max_point_line_original.y.push_back(max_point_line_original_left.y[i - 1]);
                max_point_line_original.i.push_back(max_point_line_original_left.i[i - 1]);

                std::cout << "Distanza " << i << ": " << d << " mm" << std::endl;
            }
        }

        if (!distance_line.empty()) {
            distance_for_line.push_back(distance_line);
        }

        for (int i = 0; i < max_point_line_original.i.size(); i++) {
            cloud_final->points.emplace_back(pcl::PointXYZ(max_point_line_original.x[i] * 1, *k * 1, max_point_line_original.y[i] * 1));
        }
    }

    // STATS
    std::vector<double> distance_min, distance_max;

    for (int i = 0; i < distance_for_line.size(); i++) {
        double min = *std::min_element(distance_for_line[i].begin(), distance_for_line[i].end());
        double max = *std::max_element(distance_for_line[i].begin(), distance_for_line[i].end());
        double mean = std::accumulate(distance_for_line[i].begin(), distance_for_line[i].end(), 0.0) / distance_for_line[i].size();

        std::cout << "Line: " << i << " - Distanza minima: " << min << " mm" << std::endl;
        std::cout << "Line: " << i << " - Distanza massima: " << max << " mm" << std::endl;
        std::cout << "Line: " << i << " - Distanza media: " << mean << " mm" << std::endl;
        
        distance_min.push_back(min);
        distance_max.push_back(max);
    }

    double min_min = *std::min_element(distance_min.begin(), distance_min.end());
    double max_min = *std::max_element(distance_min.begin(), distance_min.end());
    double mean_min = std::accumulate(distance_min.begin(), distance_min.end(), 0.0) / distance_min.size();
    std::cout << "Distanza minima totale tra le distanze minime per linea: " << min_min << " mm" << std::endl;
    std::cout << "Distanza massima totale tra le distanze minime per linea: " << max_min << " mm" << std::endl;
    std::cout << "Distanza media totale tra le distanze minime per linea: " << mean_min << " mm" << std::endl;
    
    double min_max = *std::min_element(distance_max.begin(), distance_max.end());
    double max_max = *std::max_element(distance_max.begin(), distance_max.end());
    double mean_max = std::accumulate(distance_max.begin(), distance_max.end(), 0.0) / distance_max.size();
    std::cout << "Distanza minima totale tra le distanze massime per linea: " << min_max << " mm" << std::endl;
    std::cout << "Distanza massima totale tra le distanze massime per linea: " << max_max << " mm" << std::endl;
    std::cout << "Distanza media totale tra le distanze massime per linea: " << mean_max << " mm" << std::endl;

    // GET IMAGE FROM XY PLANE
    cloud_final->height = 1;
    cloud_final->width = cloud_final->size();
    cloud_final->resize(cloud_final->size());
    SavePCD(cloud_final, "Scan/" + datetime() + "_cloud_final.pcd");
    
    GetMinMaxCoordinates(cloud_final);
    Sleep(1000);
    
    // Projecting points using a parametric model
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZ>);
    ProjectPoints(cloud_final, cloud_projected, 0, 0, 1, 0);

    GetMinMaxCoordinates(cloud_projected);
    Sleep(1000);
    /*
    // Using a matrix to transform a point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZ>());

    double x = -min_pt.x;
    double y = -min_pt.y;
    MatrixTransform(cloud_projected, cloud_transformed, x, y, 0);

    GetMinMaxCoordinates(cloud_transformed);
    Sleep(1000);

    std::cout << "Image Width: " << (int)(max_pt.x * 10) << std::endl;
    std::cout << "Image Height: " << (int)(max_pt.y * 10) << std::endl;

    cv::Mat img = cv::Mat::zeros((int)(max_pt.y * 10) + 1, (int)(max_pt.x * 10) + 1, CV_8UC1);

    for (int x = 0; x < img.cols; x++) {
        for (int y = 0; y < img.rows; y++) {
            img.at<uchar>(y, x) = 0;
        }
    }

    std::cout << "Cloud size: " << (*cloud_transformed).size() << std::endl;

    for (int i = 0; i < (*cloud_transformed).size(); i++) {
        img.at<uchar>((int)(cloud_transformed->points[i].y * 10), (int)(cloud_transformed->points[i].x * 10)) = 254; // from mm to �m
    }

    cv::flip(img, img, 0);
    cv::imwrite("Scan/" + datetime() + "_image.jpg", img);
    */



    // *****PER UNA LINEA*****
    /*
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_line(new pcl::PointCloud<pcl::PointXYZ>());

    for (int i = 0; i < cloud->size(); i++) {
        if (cloud->points[i].y == (float)-102.99759) {
            cloud_line->points.emplace_back(pcl::PointXYZ(cloud->points[i].x * 1, cloud->points[i].y * 0, cloud->points[i].z * 1));
        }
    }

    GetMinMaxCoordinates(cloud_line);

    GocatorCV::PolynomialFunction line_original;

    for (int i = 0; i < cloud_line->size(); i++) {
        line_original.x.push_back(cloud_line->points[i].x);
        line_original.y.push_back(cloud_line->points[i].z);
        line_original.i.push_back(i);
    }

    plt::plot(line_original.x, line_original.y);
    plt::title("Funzione originale");
    plt::show();

    // Calcolo della funzione smussata
    GocatorCV::PolynomialFunction line_smooth = GaussianFilter(line_original);

    plt::plot(line_smooth.x, line_smooth.y);
    plt::title("Funzione smussata");
    plt::show();

    // Calcolo tutti i minimi e i massimi della funziona smussata
    GocatorCV::PolynomialFunction min_max_point_line_smooth = DifferenceQuotient(line_smooth);

    plt::plot(line_smooth.x, line_smooth.y);
    plt::plot(min_max_point_line_smooth.x, min_max_point_line_smooth.y, "x");
    plt::title("Minimi e massimi della funzione smussata");
    plt::show();

    // Calcolo i massimi (da destra) della funzione originale
    GocatorCV::PolynomialFunction max_point_line_original_right;

    for (int i = 0; i < min_max_point_line_smooth.i.size(); i = i + 2) {
        max_point_line_original_right.x.push_back(line_original.x[min_max_point_line_smooth.i[i]]);
        max_point_line_original_right.y.push_back(line_original.y[min_max_point_line_smooth.i[i]]);
        max_point_line_original_right.i.push_back(line_original.i[min_max_point_line_smooth.i[i]]);
    }

    plt::plot(line_original.x, line_original.y);
    plt::plot(max_point_line_original_right.x, max_point_line_original_right.y, "x");
    plt::title("Massimi (da destra) della funzione originale");
    plt::show();

    // Calcolo i minimi della funzione originale
    GocatorCV::PolynomialFunction min_point_line_original;

    for (int i = 1; i < max_point_line_original_right.i.size(); i++) {
        GocatorCV::PolynomialFunction line;

        for (int j = 0; j < line_original.i.size(); j++) {
            if (line_original.x[j] >= max_point_line_original_right.x[i] && line_original.x[j] <= max_point_line_original_right.x[i - 1]) {
                line.x.push_back(line_original.x[j]);
                line.y.push_back(line_original.y[j]);
                line.i.push_back(j);
            }
        }

        std::vector<double>::iterator min = std::min_element(line.y.begin(), line.y.end());
        int argminVal = std::distance(line.y.begin(), min);

        min_point_line_original.x.push_back(line_original.x[line.i[argminVal]]);
        min_point_line_original.y.push_back(line_original.y[line.i[argminVal]]);
        min_point_line_original.i.push_back(line.i[argminVal]);
    }

    plt::plot(line_original.x, line_original.y);
    plt::plot(min_point_line_original.x, min_point_line_original.y, "x");
    plt::title("Minimi della funzione originale");
    plt::show();

    // Calcolo i massimi (da sinistra) della funzione originale
    GocatorCV::PolynomialFunction max_point_line_original_left;

    for (int i = 1; i < max_point_line_original_right.i.size(); i++) {
        GocatorCV::PolynomialFunction line_rotated;
        double m = (max_point_line_original_right.y[i] - min_point_line_original.y[i - 1]) / (min_point_line_original.x[i - 1] - max_point_line_original_right.x[i]);

        for (int j = 0; j < line_smooth.i.size(); j++) {
            if (line_smooth.x[j] >= max_point_line_original_right.x[i] && line_smooth.x[j] <= min_point_line_original.x[i - 1]) {
                cv::Point2d p = RotatePoint(0, 0, m, cv::Point2d(line_smooth.x[j], line_smooth.y[j]));
                line_rotated.x.push_back(p.x);
                line_rotated.y.push_back(p.y);
                line_rotated.i.push_back(j);
            }
        }

        std::vector<double>::iterator max = std::max_element(line_rotated.y.begin(), line_rotated.y.end());
        int argmaxVal = std::distance(line_rotated.y.begin(), max);

        max_point_line_original_left.x.push_back(line_original.x[line_rotated.i[argmaxVal]]);
        max_point_line_original_left.y.push_back(line_original.y[line_rotated.i[argmaxVal]]);
        max_point_line_original_left.i.push_back(line_rotated.i[argmaxVal]);
        
        //plt::named_plot("line_smooth", line_smooth.x, line_smooth.y);
        //plt::plot({ line_smooth.x[line_rotated.i[argmaxVal]] }, { line_smooth.y[line_rotated.i[argmaxVal]] }, "x");
        //plt::named_plot("line_rotated", line_rotated.x, line_rotated.y);
        //plt::plot({ line_rotated.x[argmaxVal] }, { line_rotated.y[argmaxVal] }, "o");
        //plt::title("Calcolo dei massimi (da sinistra) della funzione originale");
        //plt::legend();
        //plt::show();
    }

    plt::plot(line_original.x, line_original.y);
    plt::plot(max_point_line_original_left.x, max_point_line_original_left.y, "x");
    plt::title("Massimi (da sinistra) della funzione originale");
    plt::show();

    // Calcolo delle distanze tra la retta che congiunge due massimi adiacenti e il minimo corrispondente
    GocatorCV::PolynomialFunction min_max_point_line_original;

    for (int i = 1; i < max_point_line_original_right.i.size(); i++) {
        double d = DistancePointLine(min_point_line_original.x[i - 1], min_point_line_original.y[i - 1], max_point_line_original_right.x[i - 1], max_point_line_original_right.y[i - 1], max_point_line_original_left.x[i - 1], max_point_line_original_left.y[i - 1]);

        if (d > 0.3) {
            min_max_point_line_original.x.push_back(max_point_line_original_right.x[i - 1]);
            min_max_point_line_original.y.push_back(max_point_line_original_right.y[i - 1]);
            min_max_point_line_original.i.push_back(max_point_line_original_right.i[i - 1]);
            min_max_point_line_original.x.push_back(min_point_line_original.x[i - 1]);
            min_max_point_line_original.y.push_back(min_point_line_original.y[i - 1]);
            min_max_point_line_original.i.push_back(min_point_line_original.i[i - 1]);
            min_max_point_line_original.x.push_back(max_point_line_original_left.x[i - 1]);
            min_max_point_line_original.y.push_back(max_point_line_original_left.y[i - 1]);
            min_max_point_line_original.i.push_back(max_point_line_original_left.i[i - 1]);

            std::cout << "Distanza " << i << ": " << d << " mm" << std::endl;
        }
    }

    plt::named_plot("line_original", line_original.x, line_original.y);
    plt::plot(min_max_point_line_original.x, min_max_point_line_original.y, "x");
    plt::named_plot("line_smooth", line_smooth.x, line_smooth.y, "r--");
    plt::title("Calcolo delle distanze tra la retta che congiunge due massimi adiacenti e il minimo corrispondente");
    plt::legend();
    plt::show();
    */


    // Visualising a point cloud
    //Visualization(cloud_final);
}

void GocatorCV::Analysis::CheckValidPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_correct) {
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    pcl::ExtractIndices<pcl::PointXYZ> extract;

    for (int i = 0; i < cloud->size(); i++) {
        pcl::PointXYZ pt(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);
        
        if (!pcl::isFinite(pt)) {
            inliers->indices.push_back(i);
        }

        // Resize
        //cloud->points[i].x = cloud->points[i].x * -1;
        //cloud->points[i].y = cloud->points[i].y * 100;
        //cloud->points[i].z = cloud->points[i].z * 100;
    }
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud_correct);
}

void GocatorCV::Analysis::StatisticalOutlierRemovalFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered) {
    std::cerr << "Cloud before filtering:" << std::endl;
    std::cerr << *cloud << std::endl;

    // Create the filtering object
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(50);
    sor.setStddevMulThresh(1.0);
    sor.filter(*cloud_filtered);

    std::cerr << "Cloud after filtering:" << std::endl;
    std::cerr << *cloud_filtered << std::endl;

    PCL_INFO("Saving statistical outlier removal filter in input cloud to *.pcd\n\n");
    SavePCD(cloud_filtered, "Scan/" + datetime() + "_Statistical_Outlier_Removal_Filter.pcd");
/*
    sor.setNegative(true);
    sor.filter(*cloud_filtered);
    SavePCD(cloud_filtered, "Scan/table_scene_lms400_outliers.pcd");
*/
}

void GocatorCV::Analysis::PlaneSegmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_segmented, double value_threshold) {
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
    seg.setDistanceThreshold(value_threshold);

    // Segment dominant plane
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);
    pcl::copyPointCloud<pcl::PointXYZ>(*cloud, *inliers, *cloud_segmented);

    if (inliers->indices.size() == 0) {
        PCL_ERROR("Could not estimate a planar model for the given dataset.\n\n");
    } else {
        PCL_INFO("Saving dominant plane in input cloud to *.pcd\n\n");
        SavePCD(cloud_segmented, "Scan/" + datetime() + "_Plane_Segmentation.pcd");
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

void GocatorCV::Analysis::ProjectPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected, double a, double b, double c, double d) {
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
    proj.filter(*cloud_projected);

    PCL_INFO("Saving project points in input cloud to *.pcd\n\n");
    SavePCD(cloud_projected, "Scan/" + datetime() + "_Project_Points.pcd");
}

void GocatorCV::Analysis::MatrixTransform(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transformed, double x, double y, double z) {
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();

    // Define a translation on the x, y, z axis
    transform.translation() << x, y, z;

    pcl::transformPointCloud(*cloud, *cloud_transformed, transform);

    PCL_INFO("Saving matrix transformation in input cloud to *.pcd\n\n");
    SavePCD(cloud_transformed, "Scan/" + datetime() + "_Matrix_Transformation.pcd");
}

void GocatorCV::Analysis::GetMinMaxCoordinates(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    pcl::getMinMax3D(*cloud, min_pt, max_pt);

    std::cout << "Max x: " << max_pt.x << std::endl;
    std::cout << "Min x: " << min_pt.x << std::endl;
    std::cout << "Max y: " << max_pt.y << std::endl;
    std::cout << "Min y: " << min_pt.y << std::endl;
    std::cout << "Max z: " << max_pt.z << std::endl;
    std::cout << "Min z: " << min_pt.z << std::endl << std::endl;
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
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
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

void GocatorCV::Analysis::ContoursMeasurements(std::vector<std::vector<cv::Point>> contours) {
    std::vector<double> stats_area;
    std::vector<double> stats_perimeter;
    std::vector<double> stats_width;
    std::vector<double> stats_height;
    double area;
    double perimeter;
    cv::RotatedRect rotated_rect;

    for (int i = 0; i < contours.size(); i++) {
        area = cv::contourArea(contours[i]);
        perimeter = cv::arcLength(contours[i], true);
        rotated_rect = cv::minAreaRect(contours[i]);

        if (rotated_rect.angle < -45) {
            std::cout << "Blob " << i << "\t- Area: " << setprecision(2) << std::fixed << area / 100 << " cm^2\tPerimetro: " << setprecision(2) << std::fixed << perimeter / 100 << " cm\tWidth: " << setprecision(2) << std::fixed << rotated_rect.size.width / 100 << " cm\tHeight: " << setprecision(2) << std::fixed << rotated_rect.size.height / 100 << " cm" << std::endl;
            stats_width.push_back(rotated_rect.size.width);
            stats_height.push_back(rotated_rect.size.height);
        } else {
            std::cout << "Blob " << i << "\t- Area: " << setprecision(2) << std::fixed << area / 100 << " cm^2\tPerimetro: " << setprecision(2) << std::fixed << perimeter / 100 << " cm\tWidth: " << setprecision(2) << std::fixed << rotated_rect.size.height / 100 << " cm\tHeight: " << setprecision(2) << std::fixed << rotated_rect.size.width / 100 << " cm" << std::endl;
            stats_width.push_back(rotated_rect.size.height);
            stats_height.push_back(rotated_rect.size.width);
        }

        stats_area.push_back(area);
        stats_perimeter.push_back(perimeter);
    }
    
    std::cout << std::endl << "Area Minima: " << *std::min_element(stats_area.begin(), stats_area.end()) / 100 << " cm\t\tArea Massima: " << setprecision(2) << std::fixed << *std::max_element(stats_area.begin(), stats_area.end()) / 100 << " cm\t\tArea Media: " << setprecision(2) << std::fixed << (std::accumulate(stats_area.begin(), stats_area.end(), 0.0) / stats_area.size()) / 100 << " cm" << std::endl;
    std::cout << "Perimetro Minimo: " << *std::min_element(stats_perimeter.begin(), stats_perimeter.end()) / 100 << " cm\tPerimetro Massimo: " << setprecision(2) << std::fixed << *std::max_element(stats_perimeter.begin(), stats_perimeter.end()) / 100 << " cm\tPerimetro Medio: " << setprecision(2) << std::fixed << (std::accumulate(stats_perimeter.begin(), stats_perimeter.end(), 0.0) / stats_perimeter.size()) / 100 << " cm" << std::endl;
    std::cout << "Width Minima: " << *std::min_element(stats_width.begin(), stats_width.end()) / 100 << " cm\t\tWidth Massima: " << setprecision(2) << std::fixed << *std::max_element(stats_width.begin(), stats_width.end()) / 100 << " cm\t\tArea Width: " << setprecision(2) << std::fixed << (std::accumulate(stats_width.begin(), stats_width.end(), 0.0) / stats_width.size()) / 100 << " cm" << std::endl;
    std::cout << "Height Minima: " << *std::min_element(stats_height.begin(), stats_height.end()) / 100 << " cm\t\tHeight Massima: " << setprecision(2) << std::fixed << *std::max_element(stats_height.begin(), stats_height.end()) / 100 << " cm\t\tHeight Media: " << setprecision(2) << std::fixed << (std::accumulate(stats_height.begin(), stats_height.end(), 0.0) / stats_height.size()) / 100 << " cm" << std::endl;
    
    std::cout << std::endl;
}

double GocatorCV::Analysis::DistanceBetweenTwoPoints(int x1, int y1, int x2, int y2) {
    return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2) * 1.0);
}

double GocatorCV::Analysis::MinDistanceContours(std::vector<cv::Point> contourA, std::vector<cv::Point> contourB) {
    double minimo = -1;

    for (int j = 0; j < contourA.size(); j++) {
        for (int k = 0; k < contourB.size(); k++) {
            double distanza = DistanceBetweenTwoPoints(contourA[j].x, contourA[j].y, contourB[k].x, contourB[k].y);
            if (minimo== -1 || distanza < minimo) {
                minimo = distanza;
            }
        }
    }

    return minimo;
}

std::vector<std::vector<cv::Point>> GocatorCV::Analysis::ContoursDetection(cv::Mat image) {
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
    std::sort(contours.begin(), contours.end(), ContourSorter());

    std::vector<std::vector<cv::Point>> cnts;
    for (int i = 0; i < contours.size(); i++) {
        if (cv::contourArea(contours[i]) > 0) {
            cnts.push_back(contours[i]);
        }
    }
    
    cv::RNG rng(12345);
    std::vector<cv::Rect> boundRect(cnts.size());
    cv::Mat drawing = cv::Mat::zeros(thresh_gray.size(), CV_8UC3);

    for (int i = 0; i < cnts.size(); i++) {
        cv::Scalar color = cv::Scalar(rng.uniform(0, 256), rng.uniform(0, 256), rng.uniform(0, 256));
        cv::drawContours(drawing, cnts, (int)i, color, 2, cv::LINE_8, hierarchy, 0);
        boundRect[i] = cv::boundingRect(cnts[i]);
        cv::putText(drawing, std::to_string(i), cv::Point(boundRect[i].x + (boundRect[i].width / 2) , boundRect[i].y + (boundRect[i].height / 2)), cv::FONT_HERSHEY_SIMPLEX, 0.9, cv::Scalar(0, 255, 0), 2, false);
    }

    cv::imwrite("Scan/" + datetime() + "_drawing.jpg", drawing);
    /*
    cv::resize(drawing, drawing, cv::Size(drawing.cols / 2, drawing.rows / 2));
    cv::imshow("Output image", drawing);
    cv::waitKey(5000);
    */
    return cnts;
}

cv::Point GocatorCV::Analysis::ContourCenter(std::vector<cv::Point> contour) {
    cv::Moments M = cv::moments(contour);
    cv::Point center(M.m10 / M.m00, M.m01 / M.m00);
    return center;
}

cv::Mat GocatorCV::Analysis::MorphClosingBlob(cv::Mat image) {
    cv::Mat image_out;
    cv::Mat kernel_blob(5, 10, CV_8U);
    cv::morphologyEx(image, image_out, cv::MORPH_CLOSE, kernel_blob);

    return image_out;
}

cv::Mat GocatorCV::Analysis::MorphClosingMacroBlob(cv::Mat image) {
    cv::Mat image_out;
    cv::Mat kernel_macro_blob = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(30, 100));
    cv::morphologyEx(image, image_out, cv::MORPH_CLOSE, kernel_macro_blob);

    return image_out;
}

void GocatorCV::Analysis::DistanceBetweenMacroBlob(std::vector<std::vector<cv::Point>> contours_MacroBlob) {
    std::vector<double> stats;
    for (int i = 0; i < contours_MacroBlob.size() - 1; i++) {
        double d = MinDistanceContours(contours_MacroBlob[i], contours_MacroBlob[i + 1]);
        std::cout << "MacroBlob " << i << " - MacroBlob " << i + 1 << " -- Distanza: " << setprecision(2) << std::fixed << d / 100 << " cm" << std::endl;
        stats.push_back(d);
    }

    std::cout << "Distanza Minima: " << setprecision(2) << std::fixed << *std::min_element(stats.begin(), stats.end()) / 100 << " cm\tDistanza Massima: " << setprecision(2) << std::fixed << *std::max_element(stats.begin(), stats.end()) / 100 << " cm\tDistanza Media: " << setprecision(2) << std::fixed << (std::accumulate(stats.begin(), stats.end(), 0.0) / stats.size()) / 100 << " cm" << std::endl << std::endl;
}

void GocatorCV::Analysis::DistanceBetweenBlob(std::vector<std::vector<cv::Point>> contours_MacroBlob, std::vector<std::vector<cv::Point>> contours_Blob) {
    std::vector <std::set<int>> list_MacroBlob;
    double x1, y1, ppt1;

    for (int i = 0; i < contours_MacroBlob.size(); i++) {
        std::set<int> blob;

        for (int j = 0; j < contours_Blob.size(); j++) {
            x1 = contours_Blob[j][0].x;
            y1 = contours_Blob[j][0].y;
            ppt1 = cv::pointPolygonTest(contours_MacroBlob[i], cv::Point(x1, y1), false);
            if (ppt1 == 0 or ppt1 == 1) {
                blob.insert(j);
            }
        }
        list_MacroBlob.push_back(blob);
    }
    /*
    std::set<int>::iterator itr;
    for (auto v : macro_blob) {
        for (itr = v.begin(); itr != v.end(); ++itr) {
            cout << *itr << " ";
        }
        cout << endl;
    }
    */

    std::set<int>::iterator j;
    std::set<int>::iterator k;

    for (int i = 0; i < list_MacroBlob.size(); i++) {
        std::set<int> blob = list_MacroBlob[i];
        std::set<std::vector<int>> check_blob;

        for (j = blob.begin(); j != blob.end(); ++j) {
            double minimo_attuale = FLT_MAX;
            int indice_minimo_attuale = -1;
            std::set<int> blob_copy(blob);
            blob_copy.erase(*j);

            for (k = blob_copy.begin(); k != blob_copy.end(); ++k) {
                cv::Point center1 = ContourCenter(contours_Blob[*j]);
                cv::Point center2 = ContourCenter(contours_Blob[*k]);
                double center_distance = DistanceBetweenTwoPoints(center1.x, center1.y, center2.x, center2.y);

                if (center_distance < minimo_attuale) {
                    minimo_attuale = center_distance;
                    indice_minimo_attuale = *k;
                }
            }
            std::vector<int> check{ *j, indice_minimo_attuale };
            std::sort(check.begin(), check.end());

            if (indice_minimo_attuale > -1 && check_blob.count({ check[0], check[1] }) == 0) {
                double d = MinDistanceContours(contours_Blob[check[0]], contours_Blob[check[1]]);
                std::cout << "MacroBlob " << i << " -- Blob " << check[0] << " - Blob " << check[1] << " -- Distanza: " << setprecision(2) << std::fixed << d / 100 << " cm" << std::endl;
                check_blob.insert({ check[0], check[1] });
            }
        }
    }
}

GocatorCV::PolynomialFunction GocatorCV::Analysis::GaussianFilter(GocatorCV::PolynomialFunction line) {

    GocatorCV::PolynomialFunction line_smooth;

    const size_t N = line.x.size();        /* length of time series */
    const size_t K = 51;                            /* window size */
    const double alpha = 5;                         /* alpha values */
    gsl_vector* x = gsl_vector_alloc(N);            /* input vector */
    gsl_vector* y = gsl_vector_alloc(N);            /* filtered output vector for alpha */
    gsl_vector* k = gsl_vector_alloc(K);            /* Gaussian kernel for alpha */
    gsl_filter_gaussian_workspace* gauss_p = gsl_filter_gaussian_alloc(K);
    size_t i;

    /* generate input signal */
    for (i = 0; i < N; ++i) {
        gsl_vector_set(x, i, line.y[i]);
    }

    /* compute kernels without normalization */
    gsl_filter_gaussian_kernel(alpha, 0, 0, k);

    /* apply filters */
    gsl_filter_gaussian(GSL_FILTER_END_PADVALUE, alpha, 0, x, y, gauss_p);

    /* print filter results */
    for (i = 0; i < N; ++i) {
        line_smooth.x.push_back(line.x[i]);
        line_smooth.y.push_back(gsl_vector_get(y, i));
        line_smooth.i.push_back(i);
    }

    gsl_vector_free(x);
    gsl_vector_free(y);
    gsl_vector_free(k);
    gsl_filter_gaussian_free(gauss_p);

    return line_smooth;
}

GocatorCV::PolynomialFunction GocatorCV::Analysis::DifferenceQuotient(GocatorCV::PolynomialFunction line) {
    
    GocatorCV::PolynomialFunction point;
    double segno_precedente = 0;
    double segno_attuale = 0;

    for (int i = 0; i < line.x.size(); i++) {

        if (i == line.x.size() - 1) {
            segno_attuale = 0;
        } else {
            segno_attuale = (line.y[i + 1] - line.y[i]) / (line.x[i + 1] - line.x[i]);
        }

        if (segno_precedente < 0 && segno_attuale > 0) {
            point.x.push_back(line.x[i]);
            point.y.push_back(line.y[i]);
            point.i.push_back(i);
            std::cout << "Punto critico: x: " << line.x[i] << " y: " << line.y[i] << std::endl;
        } else if (segno_precedente > 0 && segno_attuale < 0) {
            point.x.push_back(line.x[i]);
            point.y.push_back(line.y[i]);
            point.i.push_back(i);
            std::cout << "Punto critico: x: " << line.x[i] << " y: " << line.y[i] << std::endl;
        }
        segno_precedente = segno_attuale;
    }

    return point;
}

void GocatorCV::Analysis::GetLine(double x1, double y1, double x2, double y2, double& a, double& b, double& c) {
    a = y1 - y2;
    b = x2 - x1;
    c = x1 * y2 - x2 * y1;
}

double GocatorCV::Analysis::DistancePointLine(double pct1X, double pct1Y, double pct2X, double pct2Y, double pct3X, double pct3Y) {
    double a, b, c;
    GetLine(pct2X, pct2Y, pct3X, pct3Y, a, b, c);
    return abs(a * pct1X + b * pct1Y + c) / sqrt(a * a + b * b);
}

cv::Point2d GocatorCV::Analysis::RotatePoint(double cx, double cy, double angle, cv::Point2d p) {
    double s = sin(angle);
    double c = cos(angle);

    // translate point back to origin:
    p.x -= cx;
    p.y -= cy;

    // rotate point
    double xnew = p.x * c - p.y * s;
    double ynew = p.x * s + p.y * c;

    // translate point back:
    p.x = xnew + cx;
    p.y = ynew + cy;
    return p;
}
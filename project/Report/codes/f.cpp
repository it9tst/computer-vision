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
                std::cout << "MacroBlob " << i << " -- Blob " << check[0] << " - Blob " << check[1] << " -- Distanza: " << _distanza.str() << " cm" << std::endl;
                check_blob.insert({ check[0], check[1] });
            }
        }
    }
}
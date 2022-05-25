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
    img.at<uchar>((int)(cloud_transformed->points[i].y * 10), (int)(cloud_transformed->points[i].x * 10)) = 254;
}

cv::rotate(img, img, cv::ROTATE_90_CLOCKWISE);
cv::flip(img, img, 1);
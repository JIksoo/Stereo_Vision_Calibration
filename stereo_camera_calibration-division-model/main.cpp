#include "cameracalibration.h"

int main()
{
    //std::stirng imgDataDir = "./";

    cv::Mat L_result, R_result;
    std::vector<double> Lpar, Rpar;
    double base;

    //cv::Mat L_img = cv::imread(imgDir, CV_LOAD_IMAGE_COLOR);
    //cv::Mat R_img = cv::imread(imgDir, CV_LOAD_IMAGE_COLOR);

    CameraCalibration camera(cv::Size(7,4), imgDataDir);

    //readParameter(Lpar, Rpar, base);
    //rectificatedImg(L_img, L_result, R_img, R_result, Lpar, Rpar);

    //cv::imshow("Left_RectificatedImg", L_result);
    //cv::imshow("Right_RectificatedImg", R_result);

    //cv::waitKey(0);
    //cv::destroyAllWindows();

    return 0;
}

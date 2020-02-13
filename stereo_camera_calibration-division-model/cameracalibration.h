#ifndef CAMERACALIBRATION_H
#define CAMERACALIBRATION_H
#include <opencv2/opencv.hpp>
#include <dirent.h>
#include <iostream>
#include <fstream>

class CameraCalibration
{
private:
    int max_iter, up_factor, down_factor;
    double init_lamda, target_derr;
    int numOfcam, numOfimg, numOfpoint;

    std::vector<cv::Point3f> world_coner;
    std::vector<std::vector<std::vector<cv::Point2f>>> image_corner_list;

public:
    CameraCalibration(cv::Size boardSize, std::string fileDir);
    std::vector<std::vector<std::string>> readImageFile(std::string fileDir);

    void levmarq_init();

    void Homography(std::vector<std::vector<std::vector<double>>> &homography_list);
    void InitIntrinsicParmeter(std::vector<std::vector<std::vector<double>>> homography_list, std::vector<std::vector<double>> &init_int_par);
    void InitExtrinsicParmeter(std::vector<std::vector<std::vector<double>>> homography_list, std::vector<std::vector<double>> init_int_par, std::vector<std::vector<std::vector<double>>> &init_ext_par);
    void InitDistortionCoefficient(std::vector<std::vector<double>> &init_int_par, std::vector<std::vector<std::vector<double>>> &init_ext_par);

    cv::Mat func(std::vector<std::vector<double>> par, std::vector<cv::Point3f> x);
    double error_function(cv::Mat &error, std::vector<std::vector<double>> par, std::vector<cv::Point3f> x, std::vector<std::vector<cv::Point2f>> y);
    void jacobian(cv::Mat &J, std::vector<std::vector<double>> par);
    void levmarq(std::vector<std::vector<double>> init_par, std::vector<std::vector<double>> &end_par, std::vector<std::vector<cv::Point2f>> image_corner_list);
    void baselineCalib(double &baseline, std::vector<std::vector<std::vector<double>>> par);

    cv::Mat vecTo1Dmat(std::vector<std::vector<double>> vec, int n);
    std::vector<std::vector<double>> matTo2Dvec(cv::Mat mat, int row);
    void rotmatTovec(std::vector<double> &rotvec, cv::Mat rotmat);
    void vecTorotmat(cv::Mat &rotmat, std::vector<double> rotvec);
};

bool readParameter(std::vector<double> &Lpar, std::vector<double> &Rpar, double &baseline, std::string dir = "/home/iksoo/catkin_ws/src/stereo_vision/src/Calibrated Data.txt");
void rectificatedImg(cv::Mat img, cv::Mat &result, std::vector<double> par);
void rectificatedImg(cv::Mat L_img, cv::Mat &L_result, cv::Mat R_img, cv::Mat &R_result, std::vector<double> L_par, std::vector<double> R_par);

#endif // CAMERACALIBRATION_H

#include "cameracalibration.h"

CameraCalibration::CameraCalibration(cv::Size boardSize, std::string fileDir)
{
    // read Image Data of all camera
    std::vector<std::vector<std::string>> cam;
    cam = readImageFile(fileDir);

    this->numOfcam = cam.size();
    this->numOfimg = cam[0].size();
    this->numOfpoint = boardSize.area();

    // Get World Coordinate
    // (X, Y, 1) = (j, i, 1)
    for(int i = 0; i<boardSize.height; i++)
    {
        for(int j = 0; j<boardSize.width; j++)
        {
            // std::vector.push_back means add data on vector
            this->world_coner.push_back(cv::Point3f(j, i, 1.0f));
        }
    }

    // Get Image Corner
    cv::Mat img;
    std::vector<cv::Point2f> image_corner;
    std::vector<std::vector<std::vector<cv::Point2f>>> image_corner_list;

    image_corner_list.assign(this->numOfcam, std::vector<std::vector<cv::Point2f>>(this->numOfimg, std::vector<cv::Point2f>(this->numOfpoint)));

    for(int i = 0; i < numOfcam; i++){
        for(int j = 0; j < numOfimg; j++)
        {
            img = cv::imread(fileDir + "ImageData/" + cam[i][j]);
            bool found = cv::findChessboardCorners(img, boardSize, image_corner);

            if(found)
            {
                image_corner_list[i][j].assign(image_corner.begin(), image_corner.end());
            }
            else
            {
                std::cout<<"Fail to found Chessboard ("<<cam[i][j]<<')'<<std::endl;
                this->numOfimg--;
                continue;
            }
        }
    }
    this->image_corner_list = image_corner_list;

    // Calculate Initial parameter
    std::vector<std::vector<std::vector<double>>> homography_list;
    std::vector<std::vector<std::vector<double>>> init_par;
    std::vector<std::vector<double>> init_int_par;
    std::vector<std::vector<std::vector<double>>> init_ext_par;

    homography_list.assign(this->numOfcam, std::vector<std::vector<double>>(this->numOfimg, std::vector<double>(9)));
    init_par.assign(this->numOfcam, std::vector<std::vector<double>>(this->numOfimg+1, std::vector<double>(6)));
    init_int_par.assign(this->numOfcam, std::vector<double>(6));
    init_ext_par.assign(this->numOfcam, std::vector<std::vector<double>>(this->numOfimg, std::vector<double>(6)));

    Homography(homography_list);
    InitIntrinsicParmeter(homography_list, init_int_par);
    InitExtrinsicParmeter(homography_list, init_int_par, init_ext_par);
    InitDistortionCoefficient(init_int_par, init_ext_par);

    // Merge intrinsic parameter and distortion coefficient
    for(int i = 0; i < this->numOfcam; i++)
    {
        for(int j = 0; j < this->numOfimg; j++)
        {
            init_par[i][0].assign(init_int_par[i].begin(), init_int_par[i].end());
            init_par[i][j+1].assign(init_ext_par[i][j].begin(), init_ext_par[i][j].end());
        }
    }

    // Initiallize Levenberg-Marquardt parameter
    levmarq_init();

    // run levmarq algorithm for each camera
    std::vector<std::vector<std::vector<double>>> end_par(2);

    for(int i = 0; i < this->numOfcam; i++)
    {
        levmarq(init_par[i], end_par[i], this->image_corner_list[i]);
    }

    // baseline calibration
    double baseline;
    baselineCalib(baseline, end_par);

    // Save the result
    std::ofstream of;
    std::string filename = fileDir  + "Calibrated Data.txt";
    of.open(filename);
    of << "Left camera \nfx: " + std::to_string(end_par[0][0][0]) + "\nfy: " + std::to_string(end_par[0][0][1])+
             "\nu0: " + std::to_string(end_par[0][0][2]) + "\nv0: " + std::to_string(end_par[0][0][3])+
             "\nf1: " + std::to_string(end_par[0][0][4]) + "\nf2: " + std::to_string(end_par[0][0][5])+
            "\n\nRight Camera \nfx: " + std::to_string(end_par[1][0][0]) + "\nfy: " + std::to_string(end_par[1][0][1])+
            "\nu0: " + std::to_string(end_par[1][0][2]) + "\nv0: " + std::to_string(end_par[1][0][3])+
            "\nf1: " + std::to_string(end_par[1][0][4]) + "\nf2: " + std::to_string(end_par[1][0][5])+
            "\n\nBaseLine: " + std::to_string(baseline);
    of.close();
}

std::vector<std::vector<std::string>> CameraCalibration::readImageFile(std::string fileDir)
{
    dirent *file;
    std::vector<std::vector<std::string>> cam;
    std::vector<std::string> fileList;
    char *extention;

    fileDir += "ImageData/";
    DIR *dir = opendir(fileDir.c_str()); // string.c_str() function is  std::stirng -> const char*

    if(dir == NULL)
    {
        std::cout<<"Fail to read directory"<<std::endl;
    }

    // read file in directory
    while((file = readdir(dir)) != NULL)\
    {
        // exclude previous, next directory
        if(!strcmp(file->d_name, ".") || !strcmp(file->d_name, ".."))
        {
            continue;
        }
        if((extention = strrchr(file->d_name, '.')) == NULL)
        {
            continue;
        }
        if(!strcmp(extention, ".jpg"))
        {
            fileList.push_back(file->d_name);
        }
    }

    // sort
    for(int i = 0; i < (int)fileList.size() - 1; i++)
    {
        int tmp = i;
        for(int j = i+1; j < (int)fileList.size(); j++)
        {
            if(fileList[tmp].compare(fileList[j]) > 0)
               tmp = j;
        }

        std::swap(fileList[i], fileList[tmp]);
    }

    cam.assign(2, std::vector<std::string>(fileList.size()/2));
    cam[0].assign(fileList.begin(), fileList.end() - fileList.size()/2);
    cam[1].assign(fileList.end() - fileList.size()/2, fileList.end());

    closedir(dir);

    return cam;
}

void CameraCalibration::Homography(std::vector<std::vector<std::vector<double>>> &homography_list)
{
    int npar = 9;
    cv::Mat L(2*this->numOfpoint, npar, CV_64F);
    std::vector<double> homography(npar);

    for(int i = 0; i < this->numOfcam; i++)
    {
        for(int j = 0; j < this->numOfimg; j++)
        {
            // Find Initial Homography(Direct Method), Lh = 0
            // Input L data
            for(int k = 0; k < this->numOfpoint; k++)
            {
                cv::Point3f M = this->world_coner[k];
                cv::Point2f m = this->image_corner_list[i][j][k];

                std::vector<double> L_data{
                    -M.x, -M.y, -M.z, 0, 0, 0, m.x*M.x, m.x*M.y, m.x*M.z,
                    0, 0, 0, -M.x, -M.y, -M.z, m.y*M.x, m.y*M.y, m.y*M.z
                };

                double* rowPtr = L.ptr<double>(2*k);
                for(int l = 0; l < 18; l++)
                {
                    if(l ==9)
                    {
                        rowPtr = L.ptr<double>(2*k+1);
                    }
                    *rowPtr++ = L_data[l];
                }
            }

            // Lh = 0, vT of svd(L) that has smallest eigenvalue
            cv::SVD svd(L, cv::SVD::MODIFY_A);
            svd.vt.row(svd.vt.rows-1).copyTo(homography);

            homography_list[i][j].assign(homography.begin(), homography.end());
        }
    }
}

void CameraCalibration::InitIntrinsicParmeter(std::vector<std::vector<std::vector<double>>> homography_list, std::vector<std::vector<double>> &init_int_par)
{
    double fx, fy, gamma, u0, v0, lambda;
    cv::Mat L(2*this->numOfimg,6,CV_64F);
    std::vector<double> h(9);
    std::vector<double> b(6);

    // Calculate B matrix from constraint equations.
    for(int i = 0; i <this->numOfcam; i++)
    {
        for(int j = 0; j < this->numOfimg; j++)
        {
            h = homography_list[i][j];

            double L_data[]{
                h[0]*h[1], h[0]*h[4]+h[1]*h[3], h[3]*h[4], h[1]*h[6]+h[0]*h[7], h[4]*h[6]+h[3]*h[7], h[6]*h[7],
                h[0]*h[0]-h[1]*h[1], 2*(h[0]*h[3]-h[1]*h[4]), h[3]*h[3]-h[4]*h[4], 2*(h[0]*h[6]-h[1]*h[7]), 2*(h[3]*h[6]-h[4]*h[7]), h[6]*h[6]-h[7]*h[7]
            };
            cv::Mat L_tmp(2,6,CV_64F, L_data);
            L_tmp.copyTo(L(cv::Range(j*2,j*2+2), cv::Range(0,6)));
        }

        // The right singular vector that has smallest singular value is best null space solution
        cv::SVD svd(L, cv::SVD::MODIFY_A);
        svd.vt.row(svd.vt.rows-1).copyTo(b);

        // Calculate intrinsic parameter
        v0 = (b[1]*b[3] - b[0]*b[4])/(b[0]*b[2] - b[1]*b[1]);
        lambda = b[5] - (b[3]*b[3] + v0*(b[1]*b[3] - b[0]*b[4]))/b[0];
        fx = sqrt(lambda/b[0]);
        fy = sqrt(lambda*b[0]/(b[0]*b[2] - b[1]*b[1]));
        gamma = -b[1]*fx*fx*fy/lambda;
        u0 = gamma*v0/fy - b[3]*fx*fx/lambda;

        init_int_par[i][0]= fx;
        init_int_par[i][1]= fy;
        init_int_par[i][2]= u0;
        init_int_par[i][3]= v0;
    }
}

void CameraCalibration::InitExtrinsicParmeter(std::vector<std::vector<std::vector<double>>> homography_list, std::vector<std::vector<double>> init_int_par, std::vector<std::vector<std::vector<double>>> &init_ext_par)
{
    std::vector<double> h(9), rotvec;
    cv::Mat intrinsic_parmeter;
    cv::Mat H;

    for(int i = 0; i < this->numOfcam; i++)
    {
        intrinsic_parmeter = (cv::Mat_<double>(3,3) << init_int_par[i][0], 0, init_int_par[i][2], 0, init_int_par[i][1], init_int_par[i][3], 0, 0, 1);

        for(int j = 0; j < this->numOfimg; j++)
        {
            h = homography_list[i][j];
            H = (cv::Mat_<double>(3,3) << h[0], h[1], h[2], h[3], h[4], h[5], h[6], h[7], h[8]);

            double lambda = 1.0 / cv::norm(intrinsic_parmeter.inv() * cv::Mat(H.col(0)), cv::NORM_L2);
            cv::Mat r1 = lambda * intrinsic_parmeter.inv() * cv::Mat(H.col(0));
            cv::Mat r2 = lambda * intrinsic_parmeter.inv() * cv::Mat(H.col(1));
            cv::Mat r3 = r1.cross(r2);
            cv::Mat t = lambda * intrinsic_parmeter.inv() * cv::Mat(H.col(2));

            cv::Mat Q(3,3,CV_64F);
            r1.copyTo(Q.col(0));
            r2.copyTo(Q.col(1));
            r3.copyTo(Q.col(2));

            cv::SVD svd(Q, cv::SVD::MODIFY_A);
            cv::Mat R = svd.u * svd.vt;

            rotmatTovec(rotvec, R);

            for(int k = 0; k < 3; k++)
            {
                init_ext_par[i][j][k] = rotvec[k];
                init_ext_par[i][j][k+3] = *t.ptr<double>(k);
            }
        }
    }
}

void CameraCalibration::InitDistortionCoefficient(std::vector<std::vector<double>> &init_int_par, std::vector<std::vector<std::vector<double>>> &init_ext_par)
{
    int index;

    std::vector<double> rotvec(3), t(3);

    cv::Mat distortionCoeff, intrinsic_parmeter, rotmat, zero_r3_transMat(3,3,CV_64F), X, U, sq_r, D(2*this->numOfpoint*this->numOfimg, 2, CV_64F), d(2*this->numOfpoint*this->numOfimg, 1, CV_64F);
    cv::Point3f M;
    cv::Point2f hat_m;

    for(int i = 0; i < this->numOfcam; i++)
    {
        intrinsic_parmeter = (cv::Mat_<double>(3,3) << init_int_par[i][0], 0, init_int_par[i][2], 0, init_int_par[i][1], init_int_par[i][3], 0, 0, 1);
        cv::Mat U0 = (cv::Mat_<double>(2,1) << init_int_par[i][2], init_int_par[i][3]);

        for(int j = 0; j < this->numOfimg; j++)
        {
            rotvec.assign(init_ext_par[i][j].begin(), init_ext_par[i][j].begin()+3);
            t.assign(init_ext_par[i][j].begin()+3, init_ext_par[i][j].end());

            vecTorotmat(rotmat, rotvec);
            rotmat(cv::Range(0,3), cv::Range(0,2)).copyTo(zero_r3_transMat(cv::Range(0,3), cv::Range(0,2)));
            cv::Mat(t).copyTo(zero_r3_transMat(cv::Range(0,3), cv::Range(2,3)));

            for(int k = 0; k < this->numOfpoint; k++)
            {
                M = this->world_coner[k];
                hat_m = this->image_corner_list[i][j][k];

                X = zero_r3_transMat * cv::Mat(cv::Point3d(M));
                X = X / *X.ptr<double>(2);
                sq_r = X.t()*X - cv::Scalar(1);

                U = intrinsic_parmeter * X;
                U = U / *U.ptr<double>(2);
                U = U(cv::Range(0,2), cv::Range(0,1));

                index = 2*(this->numOfpoint*j+k);
                cv::Mat tmp_D = (cv::Mat(cv::Point2d(hat_m)) - U0) * (cv::Mat_<double>(1,2) << *sq_r.ptr<double>(), (*sq_r.ptr<double>())*(*sq_r.ptr<double>()));
                tmp_D.copyTo(D(cv::Range(index, index+2), cv::Range(0,2)));

                cv::Mat tmp_d = U - cv::Mat(cv::Point2d(hat_m));
                tmp_d.copyTo(d(cv::Range(index, index+2), cv::Range(0,1)));
            }
        }

        distortionCoeff = (D.t()*D).inv() * D.t() * d;

        init_int_par[i][4] = *distortionCoeff.ptr<double>(0);
        init_int_par[i][5] = *distortionCoeff.ptr<double>(1);
    }
}

// par = {a,w0,w1...w(m-1)} ===> a = {fx,fy,u0,v0,k1,k2}, wj = {rho1,rho2,rho3,tx,ty,tz}
cv::Mat CameraCalibration::func(std::vector<std::vector<double>> par, std::vector<cv::Point3f> x)
{
    int index;

    double fx = par[0][0], fy = par[0][1], u0 = par[0][2], v0 = par[0][3], k1 = par[0][4], k2 = par[0][5];

    std::vector<double> w, rotvec(3), transvec(3);
    cv::Mat X, U, U0, hat_U, sq_r, rotmat, intMat(3,3,CV_64F), extMat(3,3,CV_64F), y(2*this->numOfimg *this->numOfpoint, 1, CV_64F);

    intMat = (cv::Mat_<double>(3,3) << fx, 0, u0, 0, fy, v0, 0, 0, 1);
    U0 = (cv::Mat_<double>(2,1) << u0, v0);

    for(int i = 0; i < this->numOfimg; i++)
    {
        w = par[i+1];
        for(int k = 0; k < 3; k++)
        {
            rotvec[k] = w[k];
            transvec[k] = w[k+3];
        }

        vecTorotmat(rotmat, rotvec);

        rotmat(cv::Range(0,3), cv::Range(0,2)).copyTo(extMat(cv::Range(0,3), cv::Range(0,2)));
        cv::Mat(transvec).copyTo(extMat(cv::Range(0,3), cv::Range(2,3)));

        for(int j = 0; j < this->numOfpoint; j++)
        {
            X = extMat * cv::Mat(cv::Point3d(x[j]));

            X = X / *X.ptr<double>(2);
            sq_r = X.t()*X - cv::Scalar(1);

            U = intMat * X;
            U = U / *U.ptr<double>(2);
            U = U(cv::Range(0,2), cv::Range(0,1));

            hat_U = U0 + (U - U0)/(cv::Scalar(1) + k1*sq_r + k2*sq_r*sq_r);

            index = 2*(this->numOfpoint*i+j);
            hat_U.copyTo(y(cv::Range(index, index+2), cv::Range(0,1)));
        }
    }
    return y;
}

double CameraCalibration::error_function(cv::Mat &error, std::vector<std::vector<double>> par, std::vector<cv::Point3f> x, std::vector<std::vector<cv::Point2f>> y)
{
    int m = this->numOfimg;
    int n = this->numOfpoint;
    int index;

    cv::Mat point, U, hat_U(2*m*n, 1, CV_64F);

    U = func(par, x);

    for(int i = 0; i < n*m; i++)
    {
        index = 2*i;
        point = cv::Mat(y[i/n][i%n]);
        point.copyTo(hat_U(cv::Range(index, index+2), cv::Range(0,1)));
    }

    error = hat_U - U;

    return cv::norm(error, cv::NORM_L2);
}

void CameraCalibration::jacobian(cv::Mat &J, std::vector<std::vector<double>> par)
{
    int m = this->numOfimg;
    int n = this->numOfpoint;
    double dx = 1e-7;

    std::vector<std::vector<double>> pre_par, nxt_par;
    cv::Mat J_col, nxt_y, pre_y, dy;

    J.create(2*n*m, 6*(m+1), CV_64F);

    pre_par = par;
    nxt_par = par;

    for(int i = 0; i < m+1; i++)
    {
        for(int j = 0; j < 6; j++)
        {
            nxt_par[i][j] = par[i][j] + dx;
            pre_par[i][j] = par[i][j] - dx;

            nxt_y = func(nxt_par, this->world_coner);
            pre_y = func(pre_par, this->world_coner);

            dy = nxt_y - pre_y;

            J_col = dy/(2*dx);
            J_col.copyTo(J.col(6*i+j));

            nxt_par[i][j] = par[i][j];
            pre_par[i][j] = par[i][j];
        }
    }
}

void CameraCalibration::levmarq_init()
{
    this->max_iter = 10000;
    this->init_lamda = 0.0001;
    this->up_factor = 10;
    this->down_factor = 10;
    this->target_derr = 1e-12;
}

void CameraCalibration::levmarq(std::vector<std::vector<double>> init_par, std::vector<std::vector<double>> &end_par, std::vector<std::vector<cv::Point2f>> image_corner_list)
{
    double mult, lambda, err_val, new_error_val, derr = 0.0;
    int npar = 6*(this->numOfimg+1), ill;
    std::vector<std::vector<double>> parameter, new_parameter;
    cv::Mat error, new_error, J, hessian, inv_hessian, b, delta;

    parameter = init_par;
    lambda = this->init_lamda;
    err_val = error_function(error, parameter, this->world_coner, image_corner_list);

    // Optimazation x_nxt = x_pre - (J^T * J + lambda*diag(J^T * J))^-1 * J^T * err
    // Main Loop
    for (int it = 0; it < this->max_iter; it++)
    {
        // Calculate Jacobian and Hessian and b = (J^T) * err
        jacobian(J, parameter);
        hessian = J.t()*J;
        b = J.t() * error;

        mult = 1 + lambda;
        ill = 1; /* ill-conditioned? */

        // Iteration until derr is decreased.
        while (ill && (it < this->max_iter))
        {
            for (int j=0; j<npar; j++)
                hessian.at<double>(j,j) *= mult; // (J^T * J + lambda*diag(J^T * J))

            inv_hessian = hessian.inv();

            // Is Hessian inverse Matrix?
            if(cv::countNonZero(inv_hessian) == 0)
                ill = 1; // Non inverse Matrix
            else
                ill = 0; // Inverse Matrix

            // If hessian is inverse matrix calculate derr
            if (!ill)
            {
                delta = inv_hessian*b;

                new_parameter = matTo2Dvec(vecTo1Dmat(parameter, npar) + delta, this->numOfimg + 1);
                new_error_val = error_function(new_error, new_parameter, this->world_coner, image_corner_list);

                derr = new_error_val - err_val;
                ill = (derr > 0);
            }

            // If hessian is not inverse matrix or derr isn't decreased, increase the lambda
            if (ill)
            {
                mult = (1 + lambda*this->up_factor)/(1 + lambda);
                lambda *= this->up_factor;
                it++;
            }
        }

        parameter = new_parameter;
        error = new_error;
        err_val = new_error_val;
        lambda /= this->down_factor;

        if ((!ill)&&(-derr < this->target_derr))
        {
            std::cout<<"Ok"<<std::endl;
            break;
        }
    }
    end_par = new_parameter;
}

void CameraCalibration::baselineCalib(double &baseline, std::vector<std::vector<std::vector<double>>> par)
{
    double d = 30.0; // 30mm
    double fx1 = par[0][0][0];
    double fx2 = par[1][0][0];
    double u1, u2;

    std::vector<double> w1, rotvec1(3), transvec1(3), w2, rotvec2(3), transvec2(3), Z1, Z2, D;
    cv::Mat rotmat1, extMat1(3,3,CV_64F), rotmat2, extMat2(3,3,CV_64F), X1, X2;

    for(int i = 0; i < this->numOfimg; i++)
    {
        w1 = par[0][i+1];
        w2 = par[1][i+1];

        for(int k = 0; k < 3; k++)
        {
            rotvec1[k] = w1[k];
            transvec1[k] = w1[k+3];
            rotvec2[k] = w2[k];
            transvec2[k] = w2[k+3];
        }

        vecTorotmat(rotmat1, rotvec1);
        vecTorotmat(rotmat2, rotvec2);

        rotmat1(cv::Range(0,3), cv::Range(0,2)).copyTo(extMat1(cv::Range(0,3), cv::Range(0,2)));
        cv::Mat(transvec1).copyTo(extMat1(cv::Range(0,3), cv::Range(2,3)));
        rotmat2(cv::Range(0,3), cv::Range(0,2)).copyTo(extMat2(cv::Range(0,3), cv::Range(0,2)));
        cv::Mat(transvec2).copyTo(extMat2(cv::Range(0,3), cv::Range(2,3)));

        for(int j = 0; j < this->numOfpoint; j++)
        {
            X1 = extMat1 * cv::Mat(cv::Point3d(this->world_coner[j]));
            X2 = extMat2 * cv::Mat(cv::Point3d(this->world_coner[j]));

            Z1.push_back(d * abs(*X1.ptr<double>(2))); // Z1 = Z2
            Z2.push_back(d * abs(*X2.ptr<double>(2)));

            X1 = X1 / *X1.ptr<double>(2);
            X2 = X2 / *X2.ptr<double>(2);

            u1 = fx1 * *X1.ptr<double>(0);
            u2 = fx2 * *X2.ptr<double>(0);

            D.push_back(abs(fx1*fx2 / (u1*fx2 - u2*fx1)));
        }
    }

    baseline = *cv::Mat((cv::Mat(D).t() * cv::Mat(D)).inv() * cv::Mat(D).t() * cv::Mat(Z1)).ptr<double>();
}

void CameraCalibration::rotmatTovec(std::vector<double> &rotvec, cv::Mat rotmat)
{
    double trace = 0.0, theta, magnitude;
    cv::Mat crossMat;

    rotvec.assign(3, 0);
    crossMat = rotmat - rotmat.t();

    for(int i = 0; i < 3; i++)
        trace += rotmat.at<double>(i,i);
    theta = acos((trace - 1)/2.0);

    rotvec[0] = crossMat.at<double>(2,1);
    rotvec[1] = -crossMat.at<double>(2,0);
    rotvec[2] = crossMat.at<double>(1,0);

    // ||u|| = theta
    magnitude = sqrt(rotvec[0]*rotvec[0] + rotvec[1]*rotvec[1] + rotvec[2]*rotvec[2]);

    for(int i = 0; i < 3; i++){
        rotvec[i] *= theta / magnitude;
    }

}

void CameraCalibration::vecTorotmat(cv::Mat &rotmat, std::vector<double> rotvec)
{
    cv::Mat crossMat(3,3,CV_64F);
    double theta;

    rotmat.create(3,3,CV_64F);
    theta = sqrt(rotvec[0]*rotvec[0] + rotvec[1]*rotvec[1] + rotvec[2]*rotvec[2]);

    // Normalizing vector
    for(int i = 0; i < 3; i++)
        rotvec[i] /= theta;

    crossMat = (cv::Mat_<double>(3,3) << 0, -rotvec[2], rotvec[1], rotvec[2], 0, -rotvec[0], -rotvec[1], rotvec[0], 0);

    rotmat = cv::Mat::eye(cv::Size(3,3), CV_64F) + sin(theta) * crossMat + (1 - cos(theta)) * crossMat *crossMat;
}

cv::Mat CameraCalibration::vecTo1Dmat(std::vector<std::vector<double>> vec, int n)
{
    int index, row, col;
    cv::Mat mat(n,1,CV_64F);

    row = vec.capacity();
    for(int i = 0; i < row; i++)
    {
        col = vec[i].capacity();
        for(int j = 0; j < col; j++)
        {
            index = i*col + j;

            *mat.ptr<double>(index) = vec[i][j];
        }
    }

    return mat;
}

std::vector<std::vector<double>> CameraCalibration::matTo2Dvec(cv::Mat mat, int row)
{
    int index, col;
    std::vector<std::vector<double>> vec;

    col = mat.total() / row;
    vec.assign(row, std::vector<double>(col, 0));

    for(int i = 0; i < row; i++)
    {
        for(int j = 0; j < col; j++)
        {
            index = col*i + j;

            vec[i][j] = *mat.ptr<double>(index);
        }
    }

    return vec;
}

bool readParameter(std::vector<double> &L_par, std::vector<double> &R_par, double &baseline, std::string dir)
{
    std::ifstream f;
    std::vector<std::string> data;
    std::string tmp;

    L_par.clear();
    R_par.clear();

    f.open(dir);

    if(!f.is_open())
        return false;

    while(std::getline(f, tmp, '\n'))
        data.push_back(tmp);

    for(int i = 0; i < (int)data.size(); i++)
    {
        if(1 <= i && i < 7)
            L_par.push_back(std::stod(data[i].substr(4)));
        else if(9 <= i && i <15)
            R_par.push_back(std::stod(data[i].substr(4)));
        else if(i == 16)
            baseline = std::stod(data[i].substr(10));
    }

    f.close();

    return true;
}

// par = {fx, fy, u0, v0, f1, f2} f1,f2 is coefficient of division model
void rectificatedImg(cv::Mat img, cv::Mat &result, std::vector<double> par)
{
    double X, Y, hat_U, hat_V, sq_r, dist;
    cv::Mat inv_int_par, srcX, srcY, int_par;

    int_par = (cv::Mat_<double>(3,3) << par[0], 0, par[2], 0, par[2], par[3], 0, 0, 1);

    result.create(img.rows, img.cols, img.type());
    srcX.create(result.rows, result.cols, CV_32F);
    srcY.create(result.rows, result.cols, CV_32F);

    inv_int_par = int_par.inv();
    double inv_elem[4] = {inv_int_par.ptr<double>(0)[0], inv_int_par.ptr<double>(0)[2], inv_int_par.ptr<double>(1)[1], inv_int_par.ptr<double>(1)[2]};

    for(int i = 0; i < img.rows; i++)

    {
        float* srcX_row = srcX.ptr<float>(i);
        float* srcY_row = srcY.ptr<float>(i);

        for(int j = 0; j < img.cols; j++)
        {
            X = inv_elem[0]*j + inv_elem[1];
            Y = inv_elem[2]*i + inv_elem[3];

            sq_r = X*X + Y*Y;
            dist = 1 + par[4]*sq_r + par[5]*sq_r*sq_r;

            hat_U = par[2] + (j - par[2])/(dist);
            hat_V = par[3] + (i - par[3])/(dist);

            srcX_row[j] = (float)hat_U;
            srcY_row[j] = (float)hat_V;
        }
    }

    cv::remap(img, result, srcX, srcY, cv::INTER_LINEAR);
}

void rectificatedImg(cv::Mat L_img, cv::Mat &L_result, cv::Mat R_img, cv::Mat &R_result, std::vector<double> L_par, std::vector<double> R_par)
{
    rectificatedImg(L_img, L_result, L_par);
    rectificatedImg(R_img, R_result, R_par);
}

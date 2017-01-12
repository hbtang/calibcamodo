#include "Type.h"

namespace calibcamodo {

XYTheta::XYTheta(){}
XYTheta::XYTheta(float _x, float _y ,float _theta):
    x(_x), y(_y), theta(_theta){}
XYTheta::~XYTheta(){}

XYTheta XYTheta::operator +(const XYTheta& toadd){
    // Note: dx and dy, which is expressed in the previous,
    // should be transformed to be expressed in the world frame
    float cost = std::cos(theta);
    float sint = std::sin(theta);
    float _x = x + toadd.x*cost - toadd.y*sint;
    float _y = y + toadd.x*sint + toadd.y*cost;
    float _theta = theta + toadd.theta;
    return XYTheta(_x, _y, _theta);
}

XYTheta XYTheta::operator -(const XYTheta& tominus){
    double PI = 3.1415926;

    float dx = x - tominus.x;
    float dy = y - tominus.y;
    float dtheta = theta - tominus.theta;

    dtheta = dtheta - floor(dtheta/(2*PI))*2*PI;

    if (dtheta > PI) {
        dtheta -= 2*PI;
    }

    float cost = std::cos(tominus.theta);
    float sint = std::sin(tominus.theta);
    // Note: dx and dy, which is expressed in world frame,
    // should be transformed to be expressed in the previous frame
    return XYTheta(cost*dx+sint*dy, -sint*dx+cost*dy, dtheta);
}


double Period(double in, double upperbound, double lowerbound) {
    double period = upperbound - lowerbound;
    int index = floor((in-lowerbound)/period);
    double out = in - index*period;
    return out;
}

void Cv2Eigen(const cv::Mat &cvMat, Eigen::MatrixXd &eigenMat) {
    eigenMat.resize(cvMat.rows, cvMat.cols);
    for (int i=0; i<cvMat.rows; i++) {
        for (int j=0; j<cvMat.cols; j++) {
            eigenMat(i,j) = cvMat.at<float>(i,j);
        }
    }
}

void Eigen2Cv(const Eigen::MatrixXd &eigenMat, cv::Mat &cvMat) {
    cvMat.create(eigenMat.rows(), eigenMat.cols(), CV_32FC1);
    for (int i=0; i<eigenMat.rows(); i++) {
        for (int j=0; j<eigenMat.cols(); j++) {
            cvMat.at<float>(i,j) = eigenMat(i,j);
        }
    }
}

void Vec2MatSe3(const Mat &rvec, const Mat &tvec, Mat &T) {
    T = Mat::eye(4,4,CV_32FC1);
    Mat R = Mat::eye(3,3,CV_32FC1);
    Rodrigues(rvec, R);
    R.copyTo(T.colRange(0,3).rowRange(0,3));
    tvec.copyTo(T.col(3).rowRange(0,3));
}

void Mat2VecSe3(const Mat &T, Mat &rvec, Mat &tvec) {
    Mat R = T.colRange(0,3).rowRange(0,3);
    Rodrigues(R, rvec);
    T.col(3).rowRange(0,3).copyTo(tvec);
}

}

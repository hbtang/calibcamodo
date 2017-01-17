#include "type.h"
#include <opencv2/calib3d/calib3d.hpp>

namespace calibcamodo {

using namespace cv;

double Period(double in, double upperbound, double lowerbound) {
    double period = upperbound - lowerbound;
    int index = floor((in-lowerbound)/period);
    double out = in - index*period;
    return out;
}

//! Class Se2

Se2::Se2(float _x, float _y ,float _theta):
    x(_x), y(_y), theta(_theta) {}

Se2::Se2(const Se2& _in): Se2(_in.x, _in.y, _in.theta) {}

Se2 Se2::operator +(const Se2& toadd){
    // Note: dx and dy, which is expressed in the previous,
    // should be transformed to be expressed in the world frame
    float cost = std::cos(theta);
    float sint = std::sin(theta);
    float _x = x + toadd.x*cost - toadd.y*sint;
    float _y = y + toadd.x*sint + toadd.y*cost;
    float _theta = theta + toadd.theta;
    return Se2(_x, _y, _theta);
}

Se2 Se2::operator -(const Se2& tominus){
    float dx = x - tominus.x;
    float dy = y - tominus.y;
    float dtheta = Period(theta - tominus.theta, PI, -PI);
    float cost = std::cos(tominus.theta);
    float sint = std::sin(tominus.theta);
    // Note: dx and dy, which is expressed in world frame,
    // should be transformed to be expressed in the previous frame
    return Se2(cost*dx+sint*dy, -sint*dx+cost*dy, dtheta);
}



float Se2::dist() {
    return sqrt(x*x+y*y);
}

float Se2::ratio() {
    return theta/dist();
}

//! Class Se3

Se3::Se3() {
    rvec = Mat::zeros(3,1,CV_32FC1);
    tvec = Mat::zeros(3,1,CV_32FC1);
}

Se3::Se3(cv::Mat &_rvec, cv::Mat &_tvec) {
    rvec = _rvec.clone();
    tvec = _tvec.clone();
}

Se3::Se3(const Se3 &_in) {
    rvec = _in.rvec.clone();
    tvec = _in.tvec.clone();
}

Se3::Se3(const Se2 &_in) {
    rvec = ( Mat_<float>(3,1) << 0, 0, _in.theta );
    tvec = ( Mat_<float>(3,1) << _in.x, _in.y, 0 );
}

Se3::Se3(const cv::Mat &_T) {
    Mat _R = _T.colRange(0,3).rowRange(0,3).clone();
    Rodrigues(_R, rvec);
    tvec = _T.col(3).rowRange(0,3).clone();
}

cv::Mat Se3::T() const {
    Mat R = Mat::zeros(3,3,CV_32FC1);
    Mat T = Mat::eye(4,4,CV_32FC1);
    Rodrigues(rvec, R);
    R.copyTo(T.colRange(0,3).rowRange(0,3));
    tvec.copyTo(T.col(3).rowRange(0,3));
    return T;
}

cv::Mat Se3::R() const {
    Mat R = Mat::zeros(3,3,CV_32FC1);
    Rodrigues(rvec, R);
    return R;
}

Se3 Se3::operator- (const Se3 &_that) {
    Mat Tthis = this->T();
    Mat Tthat = _that.T();
    Mat Tresult = Tthat.inv()*Tthis;
    return Se3(Tresult);
}

Se3 Se3::operator+ (const Se3 &_that) {
    Mat Tthis = this->T();
    Mat Tthat = _that.T();
    Mat Tresult = Tthis*Tthat;
    return Se3(Tresult);
}

std::ostream &operator<< (std::ostream &os, Se3 &se3) {
    os << "rvec:" << se3.rvec << " tvec:" << se3.tvec;
    return os;
}

std::ostream &operator<< (std::ostream &os, Se2 &se2) {
    os << "[" << se2.x << "," << se2.y << "," << se2.theta << "]";
    return os;
}

}

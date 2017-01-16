#include "measure.h"
#include "frame.h"

namespace calibcamodo {

using namespace cv;
using namespace std;

Measure::Measure(const Measure &_m) {
    _m.measure.copyTo(measure);
    _m.info.copyTo(info);
}

Measure::Measure(Mat _measure, Mat _info) {
    _measure.copyTo(measure);
    _info.copyTo(info);
}

MeasureSe3::MeasureSe3(const MeasureSe3 &_m) : Measure(_m) {
    se3 = _m.se3;
}

MeasureSe3::MeasureSe3(Mat _measure, Mat _info) : Measure(_measure, _info) {
    Mat rvec = Mat::zeros(3,1,CV_32FC1);
    Mat tvec = Mat::zeros(3,1,CV_32FC1);
    for (int i=0; i<3; i++) {
        rvec.at<float>(i) = _measure.at<float>(i);
        tvec.at<float>(i) = _measure.at<float>(i+3);
    }
    se3 = Se3(rvec, tvec);
}

MeasureSe3::MeasureSe3(Mat _rvec, Mat _tvec, Mat _info) {
    _info.copyTo(info);
    measure.create(6,1,CV_32FC1);
    for (int i=0; i<3; i++) {
        measure.at<float>(i,0) = _rvec.at<float>(i,0);
        measure.at<float>(i+3,0) = _tvec.at<float>(i,0);
    }
    se3 = Se3(_rvec, _tvec);
}

cv::Mat MeasureSe3::rvec() const {
    Mat rvec = se3.rvec.clone();
    return rvec;
}
cv::Mat MeasureSe3::tvec() const {
    Mat tvec = se3.tvec.clone();
    return tvec;
}
cv::Mat MeasureSe3::matR() const {
    return se3.R();
}
cv::Mat MeasureSe3::matT() const {
    return se3.T();
}

MeasureSe2::MeasureSe2(const MeasureSe2 &_m) : Measure(_m),
    se2(_m.se2){}

MeasureSe2::MeasureSe2(Mat _measure, Mat _info) : Measure(_measure, _info) {
    float x = _measure.at<float>(0);
    float y = _measure.at<float>(1);
    float theta = _measure.at<float>(2);
    Period(theta, PI, -PI);
    se2 = Se2(x,y,theta);
}

MeasureSe2::MeasureSe2(Se2 _odo, Mat _info) {
    _info.copyTo(info);
    se2 = _odo;
    measure = (Mat_<float>(3,1) << _odo.x, _odo.y, _odo.theta);
}

Mat MeasureSe2::rvec() const {
    Mat r = ( Mat_<float>(3,1) << 0, 0, se2.theta);
    return r;
}

Mat MeasureSe2::tvec() const {
    Mat t = (Mat_<float>(3,1) << se2.x, se2.y, 0 );
    return t;
}

Mat MeasureSe2::matR() const {
    Mat R;
    Rodrigues(rvec(), R);
    return R;
}

Mat MeasureSe2::matT() const {
    Mat T = Mat::eye(4,4,CV_32FC1);
    matR().copyTo(T.colRange(0,3).rowRange(0,3));
    tvec().copyTo(T.rowRange(0,3).col(3));
    return T;
}

double MeasureSe2::ratio() const {
    double x = se2.x;
    double y = se2.y;
    double theta = se2.theta;
    double l = sqrt(x*x + y*y);
    return theta/l;
}

MeasureKf2AMk::MeasureKf2AMk(const MeasureKf2AMk &_m) : MeasureSe3(_m),
    pKf(_m.pKf), pMk(_m.pMk) {

}

MeasureKf2AMk::MeasureKf2AMk(Mat _measure, Mat _info, PtrKeyFrame _pKf, PtrArucoMark _pAMk) :
    MeasureSe3(_measure, _info), pKf(_pKf), pMk(_pAMk) {

}

MeasureKf2AMk::MeasureKf2AMk(Mat _rvec, Mat _tvec, Mat _info, PtrKeyFrame _pKf, PtrArucoMark _pAMk) :
    MeasureSe3(_rvec, _tvec, _info), pKf(_pKf), pMk(_pAMk) {

}

MeasureSe2Kf2Kf::MeasureSe2Kf2Kf(const MeasureSe2Kf2Kf &_m) : MeasureSe2(_m),
    pKfHead(_m.pKfHead), pKfTail(_m.pKfTail) {

}

MeasureSe2Kf2Kf::MeasureSe2Kf2Kf(Mat _measure, Mat _info, PtrKeyFrame _pKfHead, PtrKeyFrame _pKfTail) : MeasureSe2(_measure, _info),
    pKfHead(_pKfHead), pKfTail(_pKfTail) {

}

MeasureSe2Kf2Kf::MeasureSe2Kf2Kf(Se2 _odo, Mat _info, PtrKeyFrame _pKfHead, PtrKeyFrame _pKfTail) : MeasureSe2(_odo, _info),
    pKfHead(_pKfHead), pKfTail(_pKfTail) {

}

}

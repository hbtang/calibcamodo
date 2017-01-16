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
    _m.rvec.copyTo(rvec);
    _m.tvec.copyTo(tvec);
}

MeasureSe3::MeasureSe3(Mat _measure, Mat _info) : Measure(_measure, _info) {
    rvec.create(3,1,CV_32FC1);
    tvec.create(3,1,CV_32FC1);
    for (int i=0; i<3; i++) {
        rvec.at<float>(i) = _measure.at<float>(i);
        tvec.at<float>(i) = _measure.at<float>(i+3);
    }
}

MeasureSe3::MeasureSe3(Mat _rvec, Mat _tvec, Mat _info) {
    _rvec.copyTo(rvec);
    _tvec.copyTo(tvec);
    _info.copyTo(info);
    measure.create(6,1,CV_32FC1);
    for (int i=0; i<3; i++) {
        measure.at<float>(i,0) = _rvec.at<float>(i,0);
        measure.at<float>(i+3,0) = _tvec.at<float>(i,0);
    }
}

MeasureSe2::MeasureSe2(const MeasureSe2 &_m) : Measure(_m),
    x(_m.x), y(_m.y), theta(_m.theta){
    Period(theta, PI, -PI);
}

MeasureSe2::MeasureSe2(Mat _measure, Mat _info) : Measure(_measure, _info) {
    x = _measure.at<float>(0);
    y = _measure.at<float>(1);
    theta = _measure.at<float>(2);
    Period(theta, PI, -PI);
}

MeasureSe2::MeasureSe2(Se2 _odo, Mat _info) {
    _info.copyTo(info);
    x = _odo.x;
    y = _odo.y;
    theta = _odo.theta;
    Period(theta, PI, -PI);
    measure = (Mat_<float>(3,1) << x, y, theta);
}

Mat MeasureSe2::rvec() const {
    Mat r = ( Mat_<float>(3,1) << 0, 0, theta);
    return r;
}

Mat MeasureSe2::tvec() const {
    Mat t = (Mat_<float>(3,1) << x, y, 0 );
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

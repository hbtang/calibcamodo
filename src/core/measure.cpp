#include "measure.h"
#include "frame.h"

namespace calibcamodo {

using namespace cv;
using namespace std;

//! Class Measure

Measure::Measure(const Measure &_m) {
    _m.measure.copyTo(measure);
    _m.info.copyTo(info);
}

Measure::Measure(const Mat& _measure, const Mat& _info) {
    assert(_measure.type() == CV_32FC1 && _info.type() == CV_32FC1);
    _measure.copyTo(measure);
    _info.copyTo(info);
}

//! Class MeasureSe2

MeasureSe2::MeasureSe2(const MeasureSe2 &_m) : Measure(_m),
    se2(_m.se2){}

MeasureSe2::MeasureSe2(const Mat& _measure, const Mat& _info) : Measure(_measure, _info) {
    float x = _measure.at<float>(0);
    float y = _measure.at<float>(1);
    float theta = _measure.at<float>(2);
    Period(theta, PI, -PI);
    se2 = Se2(x,y,theta);
}

MeasureSe2::MeasureSe2(Se2 _odo, const cv::Mat& _info) {
    assert(_info.type() == CV_32FC1);
    _info.copyTo(info);
    se2 = _odo;
    measure = (Mat_<float>(3,1) << _odo.x, _odo.y, _odo.theta);
}


//! Class MeasureSe3

MeasureSe3::MeasureSe3(const MeasureSe3 &_m):
    Measure(_m), se3(_m.se3) {

}

MeasureSe3::MeasureSe3(const cv::Mat& _measure, const cv::Mat& _info) : Measure(_measure, _info) {
    Mat rvec = Mat::zeros(3,1,CV_32FC1);
    Mat tvec = Mat::zeros(3,1,CV_32FC1);
    for (int i=0; i<3; i++) {
        rvec.at<float>(i) = _measure.at<float>(i);
        tvec.at<float>(i) = _measure.at<float>(i+3);
    }
    se3 = Se3(rvec, tvec);
}

MeasureSe3::MeasureSe3(const cv::Mat& _rvec, const cv::Mat& _tvec, const cv::Mat& _info) {

    assert(_info.type() == CV_32FC1);
    assert(_rvec.type() == CV_32FC1);
    assert(_tvec.type() == CV_32FC1);

    _info.copyTo(info);
    measure.create(6,1,CV_32FC1);
    for (int i=0; i<3; i++) {
        measure.at<float>(i,0) = _rvec.at<float>(i,0);
        measure.at<float>(i+3,0) = _tvec.at<float>(i,0);
    }
    se3 = Se3(_rvec, _tvec);
}

MeasureSe3::MeasureSe3(Se3 _se3, const cv::Mat& _info) {
    assert(_info.type() == CV_32FC1);

    se3 = _se3;
    info = _info.clone();
    measure.create(6,1,CV_32FC1);
    for (int i=0; i<3; i++) {
        measure.at<float>(i,0) = se3.rvec.at<float>(i,0);
        measure.at<float>(i+3,0) = se3.tvec.at<float>(i,0);
    }
}

//! Class MeasureSe2Kf2Kf

MeasureSe2Kf2Kf::MeasureSe2Kf2Kf(const MeasureSe2Kf2Kf &_m):
    MeasureSe2(_m),
    pKfHead(_m.pKfHead), pKfTail(_m.pKfTail) {

}

MeasureSe2Kf2Kf::MeasureSe2Kf2Kf(const cv::Mat& _measure, const cv::Mat& _info, PtrKeyFrame _pKfHead, PtrKeyFrame _pKfTail):
    MeasureSe2(_measure, _info),
    pKfHead(_pKfHead), pKfTail(_pKfTail) {

}

MeasureSe2Kf2Kf::MeasureSe2Kf2Kf(Se2 _odo, const cv::Mat& _info, PtrKeyFrame _pKfHead, PtrKeyFrame _pKfTail):
    MeasureSe2(_odo, _info),
    pKfHead(_pKfHead), pKfTail(_pKfTail) {

}

//! Class MeasurePt3

MeasurePt3::MeasurePt3(const MeasurePt3 &_m):
    Measure(_m), pt3(_m.pt3) {

}

MeasurePt3::MeasurePt3(const cv::Mat &_measure, const cv::Mat &_info):
    Measure(_measure, _info) {
    pt3 = Pt3(_measure);
}

MeasurePt3::MeasurePt3(Pt3 _pt3, const cv::Mat &_info):
    pt3(_pt3) {

    assert(_info.type() == CV_32FC1);
    info = _info.clone();
    measure = pt3.tvec().clone();
}


//! Class MeasurePt3Kf2Kf

MeasurePt3Kf2Mk::MeasurePt3Kf2Mk(const MeasurePt3Kf2Mk &_m):
    MeasurePt3(_m), pKf(_m.pKf), pMk(_m.pMk){

}

MeasurePt3Kf2Mk::MeasurePt3Kf2Mk(const cv::Mat &_measure, const cv::Mat &_info, PtrKeyFrame _pKf, PtrMapMark _pMk):
    MeasurePt3(_measure, _info), pKf(_pKf), pMk(_pMk){

}

MeasurePt3Kf2Mk::MeasurePt3Kf2Mk(Pt3 _pt3, const cv::Mat &_info, PtrKeyFrame _pKf, PtrMapMark _pMk):
    MeasurePt3(_pt3, _info), pKf(_pKf), pMk(_pMk){

}

//! Class MeasrueUV

void MeasureUV::UndistortPoint() {
    if(distVec.at<float>(0)==0.0) {
        ptUn = pt;
        return;
    }

    cv::Mat mat(1,2,CV_32F);
    mat = mat.reshape(2);
    cv::undistortPoints(mat, mat, camMat, distVec, cv::Mat(), camMat);
    mat = mat.reshape(1);

    ptUn.x = mat.at<float>(0);
    ptUn.y = mat.at<float>(1);
}

//MeasureUV::MeasureUV(const cv::Mat &_measure, const cv::Mat &_info, const cv::Mat& _camMat, const cv::Mat& _distVec):
//    Measure(_measure, _info) {
//    camMat = _camMat.clone();
//    distVec = _distVec.clone();
//    pt.x = _measure.at<float>(0);
//    pt.y = _measure.at<float>(1);
//    UndistortPoint();
//}

MeasureUV::MeasureUV(cv::Point2f _pt, const cv::Mat &_info, const cv::Mat& _camMat, const cv::Mat& _distVec) {    
    info = _info.clone();
    pt = _pt;
    camMat = _camMat.clone();
    distVec = _distVec.clone();
    UndistortPoint();    
    measure = (Mat_<float>(2,1) << ptUn.x, ptUn.y);
}

MeasureUV::MeasureUV(cv::Point2f _pt, cv::Point2f _ptUn, const cv::Mat &_info, const cv::Mat& _camMat, const cv::Mat& _distVec) {
    measure = (Mat_<float>(2,1) << _ptUn.x, _ptUn.y);
    info = _info.clone();

    pt = _pt;
    ptUn = _ptUn;
    camMat = _camMat.clone();
    distVec = _distVec.clone();
}

//! Class MeasureUVKf2Mp

//MeasureUVKf2Mp::MeasureUVKf2Mp(const cv::Mat &_measure, const cv::Mat &_info,
//                               const cv::Mat& _camMat, const cv::Mat& _distVec,
//                               PtrKeyFrame _pKf, PtrMapPoint _pMp):
//    MeasureUV(_measure, _info, _camMat, _distVec), pKf(_pKf), pMp(_pMp) {}

MeasureUVKf2Mp::MeasureUVKf2Mp(cv::Point2f _pt, const cv::Mat &_info,
                               const cv::Mat& _camMat, const cv::Mat& _distVec,
                               PtrKeyFrame _pKf, PtrMapPoint _pMp, int _idkp):
    MeasureUV(_pt, _info, _camMat, _distVec), pKf(_pKf), pMp(_pMp), idKp(_idkp) {}

MeasureUVKf2Mp::MeasureUVKf2Mp(cv::Point2f _pt, cv::Point2f _ptUn, const cv::Mat &_info,
               const cv::Mat& _camMat, const cv::Mat& _distVec,
               PtrKeyFrame _pKf, PtrMapPoint _pMp, int _idkp):
    MeasureUV(_pt, _ptUn, _info, _camMat, _distVec), pKf(_pKf), pMp(_pMp), idKp(_idkp) {}

}

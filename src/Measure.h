#ifndef MEASURE_H
#define MEASURE_H

#include "Type.h"
#include "stdafx.h"

namespace calibcamodo {

using namespace cv;
using namespace std;

class Measure {

public:
    Measure() = default;
    Measure(const Measure &_m);
    Measure(Mat _measure, Mat _info);
    ~Measure() = default;

    Mat measure;
    Mat info;
};

class MeasureSe3 : public Measure {

public:
    MeasureSe3() = default;
    MeasureSe3(const MeasureSe3 &_m);
    MeasureSe3(Mat _measure, Mat _info);
    MeasureSe3(Mat _rvec, Mat _tvec, Mat _info);
    ~MeasureSe3() = default;

    Mat rvec;
    Mat tvec;
};

class MeasureSe2 : public Measure {
public:
    MeasureSe2() = default;
    MeasureSe2(const MeasureSe2 &_m);
    MeasureSe2(Mat _measure, Mat _info);
    MeasureSe2(XYTheta _odo, Mat _info);
    ~MeasureSe2() = default;

    Mat rvec() const;
    Mat tvec() const;
    Mat matR() const;
    Mat matT() const;
    double ratio() const;

    double x;
    double y;
    double theta;
};

class MeasureKf2AMk : public MeasureSe3 {
public:
    MeasureKf2AMk() = default;
    MeasureKf2AMk(const MeasureKf2AMk &_m);
    MeasureKf2AMk(Mat _measure, Mat _info, PtrKeyFrame _pKf, PtrArucoMark _pAMk);
    MeasureKf2AMk(Mat _rvec, Mat _tvec, Mat _info, PtrKeyFrame _pKf, PtrArucoMark _pAMk);
    ~MeasureKf2AMk() = default;

    PtrKeyFrame pKf;
    PtrArucoMark pMk;
};

class MeasureSe3Kf2Kf : public MeasureSe3 {
public:
    MeasureSe3Kf2Kf() {}

    MeasureSe3Kf2Kf(const MeasureSe3Kf2Kf &_m) : MeasureSe3(_m),
        pKfHead(_m.pKfHead), pKfTail(_m.pKfTail) {}

    MeasureSe3Kf2Kf(Mat _measure, Mat _info, PtrKeyFrame _pKfHead, PtrKeyFrame _pKfTail) : MeasureSe3(_measure, _info),
        pKfHead(_pKfHead), pKfTail(_pKfTail) {}

    ~MeasureSe3Kf2Kf() {}

    PtrKeyFrame pKfHead;
    PtrKeyFrame pKfTail;
};

class MeasureSe2Kf2Kf : public MeasureSe2 {
public:
    MeasureSe2Kf2Kf() = default;
    MeasureSe2Kf2Kf(const MeasureSe2Kf2Kf &_m);
    MeasureSe2Kf2Kf(Mat _measure, Mat _info, PtrKeyFrame _pKfHead, PtrKeyFrame _pKfTail);
    MeasureSe2Kf2Kf(XYTheta _odo, Mat _info, PtrKeyFrame _pKfHead, PtrKeyFrame _pKfTail);
    ~MeasureSe2Kf2Kf() = default;

    PtrKeyFrame pKfHead;
    PtrKeyFrame pKfTail;
};

//struct RelSe3 {
//    float x,y,z;
//    float rx,ry,rz;
//    Mat rvec, tvec;
//    Mat R, T;
//    RelSe3(Mat _rvec, Mat _tvec) {
//        _rvec.copyTo(rvec);
//        _tvec.copyTo(tvec);
//        x = tvec.at<float>(0);
//        y = tvec.at<float>(1);
//        z = tvec.at<float>(2);
//        rx = rvec.at<float>(0);
//        ry = rvec.at<float>(1);
//        rz = rvec.at<float>(2);
//        Rodrigues(rvec, R);
//        T = Mat::eye(4, 4, CV_32FC1);
//        R.copyTo(T.colRange(0,3).rowRange(0,3));
//        tvec.copyTo(T.colRange(3,4));
//    }
//};
//
//struct RelSe2 {
//    float x,y,theta;
//    Mat rvec, tvec;
//    Mat R, T;
//    float l;
//    float ratio;
//    RelSe2(float _x, float _y, float _theta) : x(_x), y(_y), theta(_theta) {
//        rvec = (Mat_<float>(3,1) << 0, 0, theta);
//        tvec = (Mat_<float>(3,1) << x, y, 0);
//        Rodrigues(rvec, R);
//        T = Mat::eye(4,4,CV_32FC1);
//        R.copyTo(T.colRange(0,3).rowRange(0,3));
//        tvec.copyTo(T.colRange(3,4));
//        l = sqrt(x*x + y*y);
//        ratio = theta/l;
//    }
//};

}

#endif

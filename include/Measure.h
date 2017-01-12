#ifndef MEASURE_H
#define MEASURE_H

#include "Type.h"
#include "stdafx.h"

namespace calibcamodo {

using namespace cv;
using namespace std;

class Measure {
public:
    Measure() {}
    Measure(const Measure &_m) {
        _m.measure.copyTo(measure);
        _m.info.copyTo(info);
    }
    Measure(Mat _measure, Mat _info) {
        _measure.copyTo(measure);
        _info.copyTo(info);
    }
    ~Measure() {}

    Mat measure;
    Mat info;
};

class MeasureSe3 : public Measure {
public:
    MeasureSe3() {}
    MeasureSe3(const MeasureSe3 &_m) : Measure(_m) {
        _m.rvec.copyTo(rvec);
        _m.tvec.copyTo(tvec);
    }

    MeasureSe3(Mat _measure, Mat _info) : Measure(_measure, _info) {
        rvec.create(3,1,CV_32FC1);
        tvec.create(3,1,CV_32FC1);
        for (int i=0; i<3; i++) {
            rvec.at<float>(i) = _measure.at<float>(i);
            tvec.at<float>(i) = _measure.at<float>(i+3);
        }
    }

    MeasureSe3(Mat _rvec, Mat _tvec, Mat _info) {
        _rvec.copyTo(rvec);
        _tvec.copyTo(tvec);
        _info.copyTo(info);
        measure.create(6,1,CV_32FC1);
        for (int i=0; i<3; i++) {
            measure.at<float>(i,0) = _rvec.at<float>(i,0);
            measure.at<float>(i+3,0) = _tvec.at<float>(i,0);
        }
    }

    ~MeasureSe3() {}

    Mat rvec;
    Mat tvec;
};

class MeasureSe2 : public Measure {
public:
    MeasureSe2() {}
    MeasureSe2(const MeasureSe2 &_m) : Measure(_m),
        x(_m.x), y(_m.y), theta(_m.theta){
        PeriodTheta();
    }

    MeasureSe2(Mat _measure, Mat _info) : Measure(_measure, _info) {
        x = _measure.at<float>(0);
        y = _measure.at<float>(1);
        theta = _measure.at<float>(2);
        PeriodTheta();
    }

    MeasureSe2(XYTheta _odo, Mat _info) {
        _info.copyTo(info);
        x = _odo.x;
        y = _odo.y;
        theta = _odo.theta;
        PeriodTheta();
        measure = (Mat_<float>(3,1) << x, y, theta);
    }

    ~MeasureSe2() {}

    Mat rvec() {
        Mat r = ( Mat_<float>(3,1) << 0, 0, theta);
        return r;
    }

    Mat tvec() {
        Mat t = (Mat_<float>(3,1) << x, y, 0 );
        return t;
    }

    Mat matR() {
        Mat R;
        Rodrigues(rvec(), R);
        return R;
    }

    Mat matT() {
        Mat T = Mat::eye(4,4,CV_32FC1);
        matR().copyTo(T.colRange(0,3).rowRange(0,3));
        tvec().copyTo(T.rowRange(0,3).col(3));
        return T;
    }

    double ratio() {
        double l = sqrt(x*x + y*y);
        double ratio = theta/l;
        return ratio;
    }

    void PeriodTheta() {
        if (theta >= PI) {
            theta -= 2*PI;
        }
        if (theta <= -PI) {
            theta += 2*PI;
        }
    }

    double x;
    double y;
    double theta;
};

class MeasureKf2AMk : public MeasureSe3 {
public:
    MeasureKf2AMk() {}

    MeasureKf2AMk(const MeasureKf2AMk &_m) : MeasureSe3(_m),
        pKf(_m.pKf), pMk(_m.pMk) {}

    MeasureKf2AMk(Mat _measure, Mat _info, PtrKeyFrame _pKf, PtrArucoMark _pAMk) : MeasureSe3(_measure, _info),
        pKf(_pKf), pMk(_pAMk) {}

    MeasureKf2AMk(Mat _rvec, Mat _tvec, Mat _info, PtrKeyFrame _pKf, PtrArucoMark _pAMk) : MeasureSe3(_rvec, _tvec, _info),
        pKf(_pKf), pMk(_pAMk) {}

    ~MeasureKf2AMk() {}

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
    MeasureSe2Kf2Kf() {}

    MeasureSe2Kf2Kf(const MeasureSe2Kf2Kf &_m) : MeasureSe2(_m),
        pKfHead(_m.pKfHead), pKfTail(_m.pKfTail) {}

    MeasureSe2Kf2Kf(Mat _measure, Mat _info, PtrKeyFrame _pKfHead, PtrKeyFrame _pKfTail) : MeasureSe2(_measure, _info),
        pKfHead(_pKfHead), pKfTail(_pKfTail) {}

    MeasureSe2Kf2Kf(XYTheta _odo, Mat _info, PtrKeyFrame _pKfHead, PtrKeyFrame _pKfTail) : MeasureSe2(_odo, _info),
        pKfHead(_pKfHead), pKfTail(_pKfTail) {}

    PtrKeyFrame pKfHead;
    PtrKeyFrame pKfTail;
};

struct RelSe3 {
    float x,y,z;
    float rx,ry,rz;
    Mat rvec, tvec;
    Mat R, T;

    RelSe3(Mat _rvec, Mat _tvec) {
        _rvec.copyTo(rvec);
        _tvec.copyTo(tvec);
        x = tvec.at<float>(0);
        y = tvec.at<float>(1);
        z = tvec.at<float>(2);
        rx = rvec.at<float>(0);
        ry = rvec.at<float>(1);
        rz = rvec.at<float>(2);
        Rodrigues(rvec, R);
        T = Mat::eye(4, 4, CV_32FC1);
        R.copyTo(T.colRange(0,3).rowRange(0,3));
        tvec.copyTo(T.colRange(3,4));
    }
};

struct RelSe2 {
    float x,y,theta;
    Mat rvec, tvec;
    Mat R, T;
    float l;
    float ratio;

    RelSe2(float _x, float _y, float _theta) : x(_x), y(_y), theta(_theta) {
        rvec = (Mat_<float>(3,1) << 0, 0, theta);
        tvec = (Mat_<float>(3,1) << x, y, 0);
        Rodrigues(rvec, R);
        T = Mat::eye(4,4,CV_32FC1);
        R.copyTo(T.colRange(0,3).rowRange(0,3));
        tvec.copyTo(T.colRange(3,4));
        l = sqrt(x*x + y*y);
        ratio = theta/l;
    }
};

}

#endif

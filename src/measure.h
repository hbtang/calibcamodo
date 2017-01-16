#ifndef MEASURE_H
#define MEASURE_H

#include <opencv2/core/core.hpp>
#include "type.h"

namespace calibcamodo {

class Measure {

public:
    Measure() = default;
    Measure(const Measure &_m);
    Measure(cv::Mat _measure, cv::Mat _info);
    ~Measure() = default;

    cv::Mat measure;
    cv::Mat info;
};

class MeasureSe3 : public Measure {

public:
    MeasureSe3() = default;
    MeasureSe3(const MeasureSe3 &_m);
    MeasureSe3(cv::Mat _measure, cv::Mat _info);
    MeasureSe3(cv::Mat _rvec, cv::Mat _tvec, cv::Mat _info);
    ~MeasureSe3() = default;

    cv::Mat rvec;
    cv::Mat tvec;
};

class MeasureSe2 : public Measure {
public:
    MeasureSe2() = default;
    MeasureSe2(const MeasureSe2 &_m);
    MeasureSe2(cv::Mat _measure, cv::Mat _info);
    MeasureSe2(Se2 _odo, cv::Mat _info);
    ~MeasureSe2() = default;

    cv::Mat rvec() const;
    cv::Mat tvec() const;
    cv::Mat matR() const;
    cv::Mat matT() const;
    double ratio() const;

    double x;
    double y;
    double theta;
};

class MeasureKf2AMk : public MeasureSe3 {
public:
    MeasureKf2AMk() = default;
    MeasureKf2AMk(const MeasureKf2AMk &_m);
    MeasureKf2AMk(cv::Mat _measure, cv::Mat _info, PtrKeyFrame _pKf, PtrArucoMark _pAMk);
    MeasureKf2AMk(cv::Mat _rvec, cv::Mat _tvec, cv::Mat _info, PtrKeyFrame _pKf, PtrArucoMark _pAMk);
    ~MeasureKf2AMk() = default;

    PtrKeyFrame pKf;
    PtrArucoMark pMk;
};

class MeasureSe3Kf2Kf : public MeasureSe3 {
public:
    MeasureSe3Kf2Kf() {}

    MeasureSe3Kf2Kf(const MeasureSe3Kf2Kf &_m) : MeasureSe3(_m),
        pKfHead(_m.pKfHead), pKfTail(_m.pKfTail) {}

    MeasureSe3Kf2Kf(cv::Mat _measure, cv::Mat _info, PtrKeyFrame _pKfHead, PtrKeyFrame _pKfTail) : MeasureSe3(_measure, _info),
        pKfHead(_pKfHead), pKfTail(_pKfTail) {}

    ~MeasureSe3Kf2Kf() {}

    PtrKeyFrame pKfHead;
    PtrKeyFrame pKfTail;
};

class MeasureSe2Kf2Kf : public MeasureSe2 {
public:
    MeasureSe2Kf2Kf() = default;
    MeasureSe2Kf2Kf(const MeasureSe2Kf2Kf &_m);
    MeasureSe2Kf2Kf(cv::Mat _measure, cv::Mat _info, PtrKeyFrame _pKfHead, PtrKeyFrame _pKfTail);
    MeasureSe2Kf2Kf(Se2 _odo, cv::Mat _info, PtrKeyFrame _pKfHead, PtrKeyFrame _pKfTail);
    ~MeasureSe2Kf2Kf() = default;

    PtrKeyFrame pKfHead;
    PtrKeyFrame pKfTail;
};

}

#endif

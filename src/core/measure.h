#ifndef MEASURE_H
#define MEASURE_H

#include <opencv2/core/core.hpp>
#include "type.h"
#include "frame.h"
#include "mapmark.h"
#include "mappoint.h"

namespace calibcamodo {

class Measure {
public:
    Measure() = default;
    ~Measure() = default;
    Measure(const Measure &_m);
    Measure(const cv::Mat& _measure, const cv::Mat& _info);

    cv::Mat measure;
    cv::Mat info;
};

class MeasureSe2 : public Measure {
public:
    MeasureSe2() = default;
    ~MeasureSe2() = default;
    MeasureSe2(const MeasureSe2 &_m);
    MeasureSe2(const cv::Mat& _measure, const cv::Mat& _info);
    MeasureSe2(Se2 _odo, const cv::Mat& _info);

    Se2 se2;
};

class MeasureSe3 : public Measure {

public:
    MeasureSe3() = default;
    ~MeasureSe3() = default;
    MeasureSe3(const MeasureSe3 &_m);
    MeasureSe3(const cv::Mat& _measure, const cv::Mat& _info);
    MeasureSe3(const cv::Mat& _rvec, const cv::Mat& _tvec, const cv::Mat& _info);
    MeasureSe3(Se3 _se3, const cv::Mat& _info);

    Se3 se3;
};

class MeasurePt3 : public Measure {
public:
    MeasurePt3() = default;
    ~MeasurePt3() = default;
    MeasurePt3(const MeasurePt3 &_m);
    MeasurePt3(const cv::Mat &_measure, const cv::Mat &_info);
    MeasurePt3(Pt3 _pt3, const cv::Mat &_info);

    Pt3 pt3;
};

//! odometry measurement
class MeasureSe2Kf2Kf : public MeasureSe2 {
public:
    MeasureSe2Kf2Kf() = default;
    ~MeasureSe2Kf2Kf() = default;
    MeasureSe2Kf2Kf(const MeasureSe2Kf2Kf &_m);
    MeasureSe2Kf2Kf(const cv::Mat& _measure, const cv::Mat& _info, PtrKeyFrame _pKfHead, PtrKeyFrame _pKfTail);
    MeasureSe2Kf2Kf(Se2 _odo, const cv::Mat& _info, PtrKeyFrame _pKfHead, PtrKeyFrame _pKfTail);

    PtrKeyFrame pKfHead;
    PtrKeyFrame pKfTail;
};

//! todo ...
class MeasureSe3Kf2Kf : public MeasureSe3 {
public:
    MeasureSe3Kf2Kf() {}

    MeasureSe3Kf2Kf(const MeasureSe3Kf2Kf &_m) : MeasureSe3(_m),
        pKfHead(_m.pKfHead), pKfTail(_m.pKfTail) {}

    MeasureSe3Kf2Kf(const cv::Mat& _measure, const cv::Mat& _info, PtrKeyFrame _pKfHead, PtrKeyFrame _pKfTail) : MeasureSe3(_measure, _info),
        pKfHead(_pKfHead), pKfTail(_pKfTail) {}

    ~MeasureSe3Kf2Kf() {}

    PtrKeyFrame pKfHead;
    PtrKeyFrame pKfTail;
};

//! mark measurement
class MeasureSe3Kf2Mk : public MeasureSe3 {
public:
    MeasureSe3Kf2Mk() = default;
    ~MeasureSe3Kf2Mk() = default;
    MeasureSe3Kf2Mk(const MeasureSe3Kf2Mk &_m);
    MeasureSe3Kf2Mk(const cv::Mat &_measure, const cv::Mat &_info, PtrKeyFrame _pKf, PtrMapMark _pMk);
    MeasureSe3Kf2Mk(const cv::Mat &_rvec, const cv::Mat &_tvec, const cv::Mat &_info, PtrKeyFrame _pKf, PtrMapMark _pMk);

    PtrKeyFrame pKf;
    PtrMapMark pMk;
};

class MeasurePt3Kf2Mk : public MeasurePt3 {
public:
    MeasurePt3Kf2Mk() = default;
    ~MeasurePt3Kf2Mk() = default;
    MeasurePt3Kf2Mk(const MeasurePt3Kf2Mk &_m);
    MeasurePt3Kf2Mk(const cv::Mat &_measure, const cv::Mat &_info, PtrKeyFrame _pKf, PtrMapMark _pMk);
    MeasurePt3Kf2Mk(Pt3 _pt3, const cv::Mat &_info, PtrKeyFrame _pKf, PtrMapMark _pMk);

    PtrKeyFrame pKf;
    PtrMapMark pMk;
};

//! image feature

class MeasureUV : public Measure {
public:
//    MeasureUV(const cv::Mat &_measure, const cv::Mat &_info, const cv::Mat& _camMat, const cv::Mat& _distVec);
    MeasureUV(cv::Point2f _pt, const cv::Mat &_info, const cv::Mat& _camMat, const cv::Mat& _distVec);
    MeasureUV(cv::Point2f _pt, cv::Point2f _ptUn, const cv::Mat &_info, const cv::Mat& _camMat, const cv::Mat& _distVec);

    cv::Point2f pt;
    cv::Point2f ptUn;   // undistorted point
    cv::Mat camMat;
    cv::Mat distVec;

private:
    void UndistortPoint();
};

class MeasureUVKf2Mp : public MeasureUV {
public:
//    MeasureUVKf2Mp(const cv::Mat &_measure, const cv::Mat &_info,
//                   const cv::Mat& _camMat, const cv::Mat& _distVec,
//                   PtrKeyFrame _pKf, PtrMapPoint _pMp);
    MeasureUVKf2Mp(cv::Point2f _pt, const cv::Mat &_info,
                   const cv::Mat& _camMat, const cv::Mat& _distVec,
                   PtrKeyFrame _pKf, PtrMapPoint _pMp, int _idkp);

    MeasureUVKf2Mp(cv::Point2f _pt, cv::Point2f _ptUn, const cv::Mat &_info,
                   const cv::Mat& _camMat, const cv::Mat& _distVec,
                   PtrKeyFrame _pKf, PtrMapPoint _pMp, int _idkp);

    PtrKeyFrame pKf;
    PtrMapPoint pMp;
    // local id of the related keypoint in the keyframe
    int idKp;
};

typedef std::shared_ptr<MeasurePt3Kf2Mk> PtrMsrPt3Kf2Mk;
typedef std::shared_ptr<MeasureSe2Kf2Kf> PtrMsrSe2Kf2Kf;
typedef std::shared_ptr<MeasureUVKf2Mp> PtrMsrUVKf2Mp;

}

#endif

#ifndef MAPPOINT_H
#define MAPPOINT_H

#include "type.h"
#include "opencv2/core/core.hpp"

namespace calibcamodo {

class MapPoint
{
public:
    MapPoint(Pt3 _pt3);

    int  GetId() const   { return mId; }
    void SetPos(Pt3 _in) { mPt3wp = _in; }
    Pt3  GetPos() const  { return mPt3wp; }

    const cv::Mat& GetCov() const { return mCovMat; }
    void SetCov( cv::Mat& _matCov ) { mCovMat = _matCov.clone(); }

protected:
    int mId;
    Pt3 mPt3wp;    
    static int mIdNext;

    // 3-by-3 covariance matrix, for mPt3wp
    cv::Mat mCovMat;
};

class MapPointOrb: public MapPoint {
public:
    MapPointOrb(Pt3 _pt3);

protected:

};

typedef std::shared_ptr<MapPoint> PtrMapPoint;
typedef std::shared_ptr<MapPointOrb> PtrMapPointOrb;

}

#endif // POINT_H

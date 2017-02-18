#include "mappoint.h"

using namespace cv;

namespace calibcamodo {

//! Class MapPoint

int MapPoint::mIdNext = 1;

MapPoint::MapPoint(Pt3 _pt3):
    mId(mIdNext++), mPt3wp(_pt3) {
    mCovMat = LARGE_FLOAT * Mat::eye(3,3,CV_32FC1);
}

//! Class MapPointOrb

MapPointOrb::MapPointOrb(Pt3 _pt3):
    MapPoint(_pt3) {}

}



#include "mappoint.h"

namespace calibcamodo {

//! Class MapPoint

int MapPoint::mIdNext = 1;

MapPoint::MapPoint(Pt3 _pt3):
    mId(mIdNext++), mPt3wp(_pt3) {}

//! Class MapPointOrb

MapPointOrb::MapPointOrb(Pt3 _pt3):
    MapPoint(_pt3) {}

}



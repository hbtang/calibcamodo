#include "point.h"

namespace calibcamodo {

//! Class MapPoint
MapPoint::MapPoint(int _id):
    mId(_id) {}

MapPoint::MapPoint(int _id, Pt3 _pt3):
    mId(_id), mPt3wp(_pt3) {}

MapPoint::MapPoint(const MapPoint &_mp):
    mId(_mp.GetId()), mPt3wp(_mp.GetPos()) {}

//! Class MapPointOrb

MapPointOrb::MapPointOrb(int _id):
    MapPoint(_id) {}

MapPointOrb::MapPointOrb(const MapPoint& _pt):
    MapPoint(_pt) {}

MapPointOrb::MapPointOrb(const MapPointOrb& _ptorb):
    MapPoint(_ptorb) {}

}



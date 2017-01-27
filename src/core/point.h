#ifndef POINT_H
#define POINT_H

#include "type.h"
#include "opencv2/core/core.hpp"

namespace calibcamodo {

class MapPoint
{
public:
    MapPoint(int _id);
    MapPoint(int _id, Pt3 _pt3);
    MapPoint(const MapPoint &_pt);

    inline void SetId(int _id)  { mId = _id; }
    inline int  GetId() const   { return mId; }
    inline void SetPos(Pt3 _in) { mPt3wp = _in; }
    inline Pt3  GetPos() const  { return mPt3wp; }

protected:
    int mId;
    Pt3 mPt3wp;
};

class MapPointOrb: public MapPoint {
public:
    MapPointOrb(int _id);
    MapPointOrb(const MapPoint& _pt);
    MapPointOrb(const MapPointOrb& _ptorb);

protected:

};

}

#endif // POINT_H

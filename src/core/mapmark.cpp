#include "mapmark.h"
#include "frame.h"

namespace calibcamodo {

using namespace std;
using namespace cv;

//! Class Mark

MapMark::MapMark(int _id) : mId(_id) {

}

MapMark::MapMark(const MapMark &_mk) : mId(_mk.mId), mSe3wm(_mk.mSe3wm) {

}


//! MarkAruco
MapMarkAruco::MapMarkAruco(int _id, int _arucoId, double _markSize):
    MapMark(_id), mArucoId(_arucoId), mMarkSize(_markSize) {

}

}

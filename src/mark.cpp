#include "mark.h"
#include "frame.h"

namespace calibcamodo {

using namespace std;
using namespace cv;

//! Class Mark

Mark::Mark(int _id) : mId(_id) {

}

Mark::Mark(const Mark &_mk) : mId(_mk.mId), mSe3wm(_mk.mSe3wm) {

}


//! MarkAruco
MarkAruco::MarkAruco(int _id, int _arucoId, double _markSize):
    Mark(_id), mArucoId(_arucoId), mMarkSize(_markSize) {

}

}

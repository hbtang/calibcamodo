#include "frame.h"

namespace calibcamodo {

using namespace cv;
using namespace std;
using namespace aruco;

// Class Frame
Frame::Frame(const cv::Mat &_im, const Se2& _odo, int _id) :
    mOdo(_odo), mId(_id) {
    _im.copyTo(mImg);
}

Frame::Frame(const Se2& _odo, int _id) :
    mOdo(_odo), mId(_id) {
    mImg.create(0,0,CV_32FC1);
}

Frame::Frame(const Frame &_f) :
    mOdo(_f.mOdo), mId(_f.mId) {
    _f.mImg.copyTo(mImg);
}

void Frame::SetImg(cv::Mat &_img) {
    mImg = _img.clone();
}

// Class KeyFrame
KeyFrame::KeyFrame(const Frame& _f):
    Frame(_f), mpMsrOdoNext(nullptr), mpMsrOdoLast(nullptr) {
    mSe2wb = mOdo;
    mSe3wc = Se3();
}

void KeyFrame::ComputeAruco(CameraParameters &_CamParam,
                            MarkerDetector &_MarkerDetector,
                            double _marksize) {

    _MarkerDetector.detect(mImg, mvecMsrAruco, _CamParam, _marksize);
    mImg.copyTo(mImgAruco);
    for (auto mk : mvecMsrAruco) {
        mk.draw(mImgAruco, Scalar(0,0,255), 2);
    }
}

void KeyFrame::InsertMsrMk(PtrMsrKf2AMk pmsr) {
    if (msetpMk.count(pmsr->pMk)) {
        cerr << "Error in KeyFrame::InsertMsrMk, already observed." << endl;
        return;
    }
    msetpMsrMk.insert(pmsr);
    msetpMk.insert(pmsr->pMk);
    mmappMk2pMsr[pmsr->pMk] = pmsr;
}

void KeyFrame::DeleteMsrMk(PtrMsrKf2AMk pmsr) {
    msetpMsrMk.erase(pmsr);
    msetpMk.erase(pmsr->pMk);
    mmappMk2pMsr.erase(pmsr->pMk);
}

set<PtrMsrKf2AMk> KeyFrame::GetMsrMk(set<PtrArucoMark> _setpMk) const {
    set<PtrMsrKf2AMk> setRet;
    for (auto pMk : _setpMk) {
        if(msetpMk.count(pMk)) {
            PtrMsrKf2AMk pMsr = mmappMk2pMsr.at(pMk);
            setRet.insert(pMsr);
        }
    }
    return setRet;
}

void KeyFrame::SetPoseAllbyB(Se2 _wb, Se3 _bc) {
    mSe2wb = _wb;
    Se3 se3wb = Se3(_wb);
    mSe3wc = se3wb + _bc;
}


}

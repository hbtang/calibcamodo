#include "Frame.h"

namespace calibcamodo {

using namespace cv;
using namespace std;
using namespace aruco;

// Class Frame
Frame::Frame(const cv::Mat &_im, const XYTheta& _odo, int _id) :
    mOdo(_odo), mId(_id) {
    _im.copyTo(mImg);
}

Frame::Frame(const Frame &_f) :
    mOdo(_f.mOdo), mId(_f.mId) {
    _f.mImg.copyTo(mImg);
}

XYTheta Frame::RelOdoTo(const Frame &_f) const {
    XYTheta odo_to = _f.mOdo;
    XYTheta odo_from = mOdo;
    return (odo_to - odo_from);
}

XYTheta Frame::RelOdoFrom(const Frame &_f) const {
    XYTheta odo_to = mOdo;
    XYTheta odo_from = _f.mOdo;
    return (odo_to - odo_from);
}

// Class KeyFrame
KeyFrame::KeyFrame(const Frame& _f,
                   CameraParameters &_CamParam,
                   MarkerDetector &_MarkerDetector,
                   double _marksize):
    Frame(_f), mpMsrOdoNext(nullptr), mpMsrOdoLast(nullptr) {

    mrvec_wc = Mat::zeros(3,1,CV_32FC1);
    mtvec_wc = Mat::zeros(3,1,CV_32FC1);
    mrvec_wb = Mat::zeros(3,1,CV_32FC1);
    mtvec_wb = Mat::zeros(3,1,CV_32FC1);

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


}

#include "frame.h"
#include "adapter.h"

namespace calibcamodo {

using namespace cv;
using namespace std;
using namespace aruco;

//! Class Frame
Frame::Frame(const Frame &_f):
    mId(_f.mId), mOdo(_f.mOdo), mbImgLoaded(_f.mbImgLoaded) {
    mImg = _f.mImg.clone();
}

Frame& Frame::operator= (const Frame &_f) {
    mId = _f.mId;
    mOdo = _f.mOdo;
    mImg = _f.mImg.clone();
    mbImgLoaded = _f.mbImgLoaded;
    return *this;
}

Frame::Frame(const cv::Mat &_im, Se2 _odo, int _id) :
    mOdo(_odo), mId(_id) {
    mImg = _im.clone();
    mbImgLoaded = true;
}

Frame::Frame(Se2 _odo, int _id) :
    mOdo(_odo), mId(_id), mbImgLoaded(false){

}

//! Class KeyFrame
KeyFrame::KeyFrame(const KeyFrame& _kf):
    Frame(_kf), mSe2wb(_kf.mSe2wb), mSe3wc(_kf.mSe3wc) {
}

KeyFrame& KeyFrame::operator = (const KeyFrame& _kf) {
    Frame::operator =(_kf);
    return *this;
}

KeyFrame::KeyFrame(const Frame& _f):
    Frame(_f) {
    mSe2wb = mOdo;
    mSe3wc = Se3();
}

void KeyFrame::SetPoseAllbyB(Se2 _wb, Se3 _bc) {
    mSe2wb = _wb;
    Se3 se3wb = Se3(_wb);
    mSe3wc = se3wb + _bc;
}


//! Class KeyFrameAruco
KeyFrameAruco::KeyFrameAruco(const KeyFrameAruco& _kf):
    KeyFrame(_kf) {

}

KeyFrameAruco& KeyFrameAruco::operator = (const KeyFrameAruco& _kf) {
    KeyFrame::operator =(_kf);
    return *this;
}

KeyFrameAruco::KeyFrameAruco(const Frame &_f):
    KeyFrame(_f){

}

KeyFrameAruco::KeyFrameAruco(const KeyFrame &_kf):
    KeyFrame(_kf){

}

void KeyFrameAruco::ComputeAruco(aruco::CameraParameters &_CamParam,
                                 aruco::MarkerDetector &_MarkerDetector,
                                 double _markSize) {
    _MarkerDetector.detect(mImg, mvecAruco, _CamParam, _markSize);
    mImg.copyTo(mImgAruco);
    for (auto mk : mvecAruco) {
        mk.draw(mImgAruco, Scalar(0,0,255), 2);
    }
}

//! KeyFrameOrb
//!

void KeyFrameOrb::ComputeOrb(ORBextractor* _pOrb) {
    (*_pOrb)(mImg, cv::Mat(), mvecKeyPoint, mDescriptor);
}

void KeyFrameOrb::ComputeBoW(ORBVocabulary* _pVoc) {
    if(mBowVec.empty() || mFeatVec.empty()) {
        vector<cv::Mat> vCurrentDesc = toDescriptorVector(mDescriptor);
        // Feature vector associate features with nodes in the 4th level (from leaves up)
        // We assume the vocabulary tree has 6 levels, change the 4 otherwise
        _pVoc->transform(vCurrentDesc,mBowVec,mFeatVec,4);
    }
    mbBowVecExist = true;
}


}

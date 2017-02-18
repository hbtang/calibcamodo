#include "frame.h"
#include "adapter.h"
#include "config.h"

#include "orb/ORBextractor.h"
#include "orb/ORBmatcher.h"

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
    mCovMat = _kf.mCovMat.clone();
}

KeyFrame& KeyFrame::operator = (const KeyFrame& _kf) {
    Frame::operator =(_kf);
    mCovMat = _kf.mCovMat.clone();
    return *this;
}

KeyFrame::KeyFrame(const Frame& _f):
    Frame(_f) {
    mSe2wb = mOdo;
    mSe3wc = Se3();
    mCovMat = LARGE_FLOAT * Mat::eye(3,3,CV_32FC1);
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

void KeyFrameOrb::ComputeOrb(ORBextractor& _OrbExtractor, const Mat& _cammatrix, const Mat& _distortion) {

    cvtColor(mImg, mImgOrb, CV_RGB2GRAY);
    (_OrbExtractor)(mImgOrb, cv::Mat(), mvecKeyPoint, mDescriptor);
    cvtColor(mImgOrb, mImgOrb, CV_GRAY2RGB);

    UndistortKeyPoints(mvecKeyPoint, mvecKeyPointUndist, _cammatrix, _distortion);
    mNumKeyPoint = mvecKeyPoint.size();

    for (auto kpt : mvecKeyPoint) {
        Point2f pt = kpt.pt;
        circle(mImgOrb, pt, 5, Scalar(0,255,0), 1);
    }

//    imshow("orb", mImgOrb);
//    waitKey(10);
}

void KeyFrameOrb::UndistortKeyPoints(const vector<KeyPoint>& _vKeyPt, vector<KeyPoint>& _vKeyPtUndist, const Mat& _cammatrix, const Mat& _distortion)
{
    _vKeyPtUndist.clear();

    if(_distortion.at<float>(0)==0.0)
    {
        _vKeyPtUndist = _vKeyPt;
        return;
    }

    // Fill matrix with points
    cv::Mat mat(_vKeyPt.size(),2,CV_32F);
    for(unsigned int i=0; i<_vKeyPt.size(); i++)
    {
        mat.at<float>(i,0)=_vKeyPt[i].pt.x;
        mat.at<float>(i,1)=_vKeyPt[i].pt.y;
    }

    // Undistort points
    mat=mat.reshape(2);
    cv::undistortPoints(mat,mat,_cammatrix,_distortion,cv::Mat(),_cammatrix);
    mat=mat.reshape(1);

    // Fill undistorted keypoint vector
    _vKeyPtUndist.resize(_vKeyPt.size());
    for(unsigned int i=0; i<_vKeyPt.size(); i++)
    {
        cv::KeyPoint kp = _vKeyPt[i];
        kp.pt.x=mat.at<float>(i,0);
        kp.pt.y=mat.at<float>(i,1);
        _vKeyPtUndist[i]=kp;
    }
}


void KeyFrameOrb::ComputeBoW(ORBVocabulary& _OrbVoc) {
    if(mBowVec.empty() || mFeatVec.empty()) {
        vector<cv::Mat> vCurrentDesc = toDescriptorVector(mDescriptor);
        // Feature vector associate features with nodes in the 4th level (from leaves up)
        // We assume the vocabulary tree has 6 levels, change the 4 otherwise
        _OrbVoc.transform(vCurrentDesc, mBowVec, mFeatVec, 4);
    }
    mbBowVecExist = true;
}

KeyFrameOrb::KeyFrameOrb(const Frame &_f):
    KeyFrame(_f){

}

KeyFrameOrb::KeyFrameOrb(const KeyFrame &_kf):
    KeyFrame(_kf){

}


}

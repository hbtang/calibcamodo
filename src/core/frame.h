#ifndef FRAME_H
#define FRAME_H

#include "type.h"
//#include "measure.h"
#include "aruco/aruco.h"

#include "Thirdparty/DBoW2/DBoW2/BowVector.h"
#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"

#include "orb/ORBVocabulary.h"

namespace calibcamodo {

const int FRAME_GRID_ROWS = 48;
const int FRAME_GRID_COLS = 64;

class ORBextractor;
class ORBmatcher;

class Frame {
public:
    Frame() = default;
     ~Frame() = default;
    Frame(const Frame& _f);
    Frame& operator= (const Frame& _f);
    Frame(const cv::Mat &_im, Se2 _odo, int _id);
    Frame(Se2 _odo, int _id);

    inline int GetId() const { return mId; }
    inline void SetId(int _id) { mId = _id; }

    inline cv::Mat GetImg() const { return mImg; }
    inline void SetImg(const cv::Mat &_img) {
        mImg = _img.clone();
        mbImgLoaded = true;
    }

    inline Se2 GetOdo() const { return mOdo; }
    inline void SetOdo(Se2 _odo) { mOdo = _odo; }

protected:
    int mId;
    Se2 mOdo;
    cv::Mat mImg;
    bool mbImgLoaded;
};

class KeyFrame: public Frame
{
public:
    KeyFrame() = default;
    ~KeyFrame() = default;
    KeyFrame(const KeyFrame& _kf);
    KeyFrame& operator = (const KeyFrame& _kf);
    KeyFrame(const Frame &_f);    

    void SetPoseAllbyB(Se2 _wb, Se3 _bc);
    void SetPoseBase(Se2 _in) { mSe2wb = _in; }
    Se2 GetPoseBase() const { return mSe2wb; }
    void SetPoseCamera(Se3 _in) { mSe3wc = _in; }
    Se3 GetPoseCamera() const { return mSe3wc; }

    const cv::Mat& GetCov() const { return mCovMat; }
    void SetCov( cv::Mat& _matCov ) { mCovMat = _matCov.clone(); }

protected:
    Se2 mSe2wb;
    Se3 mSe3wc;

    // 3-by-3 covariance matrix
    cv::Mat mCovMat;
};

class KeyFrameAruco : public KeyFrame {
public:
    KeyFrameAruco() = default;
    ~KeyFrameAruco() = default;
    KeyFrameAruco(const KeyFrameAruco& _kf);
    KeyFrameAruco& operator = (const KeyFrameAruco& _kf);
    KeyFrameAruco(const Frame &_f);
    KeyFrameAruco(const KeyFrame &_kf);

    inline const std::vector<aruco::Marker> & GetMsrAruco() const { return mvecAruco; }
    inline const cv::Mat GetImgAruco() const { return mImgAruco; }
    void ComputeAruco(aruco::CameraParameters &_CamParam,
                      aruco::MarkerDetector &_MarkerDetector,
                      double _markSize);

private:
    std::vector<aruco::Marker> mvecAruco;  // vector of aruco measurements in this KF
    cv::Mat mImgAruco;
};

class KeyFrameOrb : public KeyFrame {

public:
    KeyFrameOrb(const Frame& _f);
    KeyFrameOrb(const KeyFrame& _kf);


    void ComputeOrb(ORBextractor& _OrbExtractor, const cv::Mat& _cammatrix, const cv::Mat& _distortion);
    void ComputeBoW(ORBVocabulary& _OrbVoc);

    void UndistortKeyPoints(const vector<cv::KeyPoint>& _vKeyPt, vector<cv::KeyPoint>& _vKeyPtUndist,
                            const cv::Mat& _cammatrix, const cv::Mat& _distortion);

    DBoW2::FeatureVector GetFeatureVector() { return mFeatVec; }
    DBoW2::BowVector GetBowVector() { return mBowVec; }

    cv::Mat mImgOrb;
    vector<cv::KeyPoint> mvecKeyPoint;
    vector<cv::KeyPoint> mvecKeyPointUndist;
    cv::Mat mDescriptor;
    int mNumKeyPoint;
    bool mbBowVecExist;
    DBoW2::FeatureVector mFeatVec;
    DBoW2::BowVector mBowVec;


//    //! TODO: for orb-matcher by window
//    vector<size_t> GetFeaturesInArea(const float &x, const float  &y, const float  &r, const int minLevel=-1, const int maxLevel=-1) const;
//    bool PosInGrid(cv::KeyPoint &kp, int &posX, int &posY);
//    static float mfGridElementWidthInv;
//    static float mfGridElementHeightInv;
//    std::vector<std::size_t> mGrid[FRAME_GRID_COLS][FRAME_GRID_ROWS];

};

typedef std::shared_ptr<Frame> PtrFrame;
typedef std::shared_ptr<KeyFrame> PtrKeyFrame;
typedef std::shared_ptr<KeyFrameAruco> PtrKeyFrameAruco;
typedef std::shared_ptr<KeyFrameOrb> PtrKeyFrameOrb;

}
#endif // FRAME_H

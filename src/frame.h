#ifndef FRAME_H
#define FRAME_H

#include "type.h"
//#include "measure.h"
#include "aruco/aruco.h"

namespace calibcamodo {

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
    inline void SetPoseBase(Se2 _in) { mSe2wb = _in; }
    inline Se2 GetPoseBase() const { return mSe2wb; }
    inline void SetPoseCamera(Se3 _in) { mSe3wc = _in; }
    inline Se3 GetPoseCamera() const { return mSe3wc; }

protected:
    Se2 mSe2wb;
    Se3 mSe3wc;
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
};

typedef std::shared_ptr<Frame> PtrFrame;
typedef std::shared_ptr<KeyFrame> PtrKeyFrame;
typedef std::shared_ptr<KeyFrameAruco> PtrKeyFrameAruco;
typedef std::shared_ptr<KeyFrameOrb> PtrKeyFrameOrb;

}
#endif // FRAME_H

#ifndef FRAME_H
#define FRAME_H

#include "type.h"
#include "measure.h"
#include "aruco/aruco.h"

namespace calibcamodo {

class Frame {
public:
    Frame() = default;
    Frame(const cv::Mat &im, const Se2& odo, int id);
    Frame(const Se2 &odo, int id);
    Frame(const Frame& _f);
    Frame& operator= (const Frame& f);
    ~Frame() = default;

    virtual int GetId() const { return mId; }
    virtual cv::Mat GetImg() const { return mImg; }
    virtual Se2 GetOdo() const { return mOdo; }

    void SetImg(cv::Mat &_img);

protected:
    int mId;
    cv::Mat mImg;
    Se2 mOdo;
};

class KeyFrame: public Frame
{
public:
    KeyFrame() {}
    KeyFrame(const Frame &_f);
    ~KeyFrame() {}

    void ComputeAruco(aruco::CameraParameters &_CamParam,
                      aruco::MarkerDetector &_MarkerDetector,
                      double _marksize);

    inline const std::vector<aruco::Marker> & GetMsrAruco() const { return mvecMsrAruco; }
    inline const cv::Mat GetImgAruco() const { return mImgAruco; }

    void InsertMsrMk(PtrMsrKf2AMk pmsr);
    void DeleteMsrMk(PtrMsrKf2AMk pmsr);
    inline set<PtrMsrKf2AMk> GetMsrMk() const { return msetpMsrMk; }
    set<PtrMsrKf2AMk> GetMsrMk(set<PtrArucoMark> setpMk) const;
    inline PtrMsrKf2AMk GetMsrMk(PtrArucoMark pMk) const {return mmappMk2pMsr.at(pMk);}
    inline set<PtrArucoMark> GetMk() const {return msetpMk; }

    PtrMsrSe2Kf2Kf GetMsrOdoNext() const {return mpMsrOdoNext;}
    PtrMsrSe2Kf2Kf GetMsrOdoLast() const {return mpMsrOdoLast;}

    inline void SetPoseBase(Se2 _in) { mSe2wb = _in; }
    inline Se2 GetPoseBase() const { return mSe2wb; }
    inline void SetPoseCamera(Se3 _in) { mSe3wc = _in; }
    inline Se3 GetPoseCamera() const { return mSe3wc; }

    void SetPoseAllbyB(Se2 _wb, Se3 _bc);
    void SetPoseAllbyC(Se3 _wc, Se3 _bc);


private:
    std::vector<aruco::Marker> mvecMsrAruco;  // vector of aruco measurements in this KF
    cv::Mat mImgAruco;

    set<PtrMsrKf2AMk> msetpMsrMk;
    set<PtrArucoMark> msetpMk;
    map<PtrArucoMark, PtrMsrKf2AMk> mmappMk2pMsr;

    PtrMsrSe2Kf2Kf mpMsrOdoNext;
    PtrMsrSe2Kf2Kf mpMsrOdoLast;

    Se2 mSe2wb;
    Se3 mSe3wc;
};

}
#endif // FRAME_H

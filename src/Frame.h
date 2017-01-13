#ifndef FRAME_H
#define FRAME_H

#include "Type.h"
#include "Measure.h"
#include "aruco.h"
#include "stdafx.h"

namespace calibcamodo {

using namespace std;
using namespace cv;
using namespace aruco;

class Frame {
public:
    Frame() = default;
    Frame(const cv::Mat &im, const XYTheta& odo, int id);
    Frame(const Frame& _f);
    Frame& operator= (const Frame& f);
    ~Frame() = default;

    virtual int GetId() const { return mId; }
    virtual Mat GetImg() const { return mImg; }
    virtual XYTheta GetOdo() const { return mOdo; }

    XYTheta RelOdoTo(const Frame &_f) const;
    XYTheta RelOdoFrom(const Frame &_f) const;

protected:
    int mId;
    Mat mImg;
    XYTheta mOdo;    
};

class KeyFrame: public Frame
{
public:
    KeyFrame() {}
    KeyFrame(const Frame& _f,
             CameraParameters &_CamParam,
             MarkerDetector &_MarkerDetector,
             double markSize);

    ~KeyFrame() {}

    inline const vector<Marker> & GetMsrAruco() const { return mvecMsrAruco; }
    inline const Mat GetImgAruco() const { return mImgAruco; }

    void InsertMsrMk(PtrMsrKf2AMk pmsr);
    void DeleteMsrMk(PtrMsrKf2AMk pmsr);
    inline set<PtrMsrKf2AMk> GetMsrMk() const { return msetpMsrMk; }
    set<PtrMsrKf2AMk> GetMsrMk(set<PtrArucoMark> setpMk) const;
    inline PtrMsrKf2AMk GetMsrMk(PtrArucoMark pMk) const {return mmappMk2pMsr.at(pMk);}
    inline set<PtrArucoMark> GetMk() const {return msetpMk; }

    PtrMsrSe2Kf2Kf GetMsrOdoNext() const {return mpMsrOdoNext;}
    PtrMsrSe2Kf2Kf GetMsrOdoLast() const {return mpMsrOdoLast;}

private:
    vector<Marker> mvecMsrAruco;  // vector of aruco measurements in this KF
    Mat mImgAruco;

    set<PtrMsrKf2AMk> msetpMsrMk;
    set<PtrArucoMark> msetpMk;
    map<PtrArucoMark, PtrMsrKf2AMk> mmappMk2pMsr;

    PtrMsrSe2Kf2Kf mpMsrOdoNext;
    PtrMsrSe2Kf2Kf mpMsrOdoLast;

    Mat mrvec_wc;
    Mat mtvec_wc;
    Mat mrvec_wb;
    Mat mtvec_wb;
};

}
#endif // FRAME_H

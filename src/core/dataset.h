#ifndef DATASET_H
#define DATASET_H

#include "type.h"
#include "frame.h"
#include "mark.h"
#include "measure.h"

#include "orb/ORBVocabulary.h"
#include "orb/ORBextractor.h"

namespace calibcamodo {

class Dataset {
//    friend class Solver;

public:
    Dataset();
    ~Dataset() = default;

    // Function interface
    void CreateFrames();
    set<PtrFrame> SelectFrame() const;
    virtual void CreateKeyFrames() {}

//    virtual void CreateMsrOdos();
//    virtual void CreateMarks() {}
//    virtual void CreateMsrMks() {}

//    void InitKf(Se3 _se3bc);
//    void InitMk();
//    void InitAll(Se3 _se3bc) {
//        InitKf(_se3bc);
//        InitMk();
//    }

    // Functions on Frame
    inline const std::set<PtrFrame>& GetFrameSet() const { return msetpFrame; }
    inline const std::map<int, PtrFrame>& GetFrameMap() const { return mmapId2pFrame; }
    PtrFrame GetFrame(int _id) const;
    bool InsertFrame(PtrFrame _ptr);


    // Functions on KeyFrame
    inline const std::set<PtrKeyFrame> & GetKfSet() const { return msetpKf; }
    inline const std::map<int, PtrKeyFrame>& GetKfMap() const { return mmapId2pKf; }
    PtrKeyFrame GetKf(int _id) const;
    bool InsertKf(PtrKeyFrame _ptr);


    // Functions on Mark
    inline const std::set<PtrMark> & GetMkSet() const { return msetpMk; }
    inline const std::map<int, PtrMark>& GetMkMap() const { return mmapId2pMk; }
    PtrMark GetMk(int _id) const;
    bool InsertMk(PtrMark _ptr);

    // Functions on MapPoint
    const std::set<PtrMapPoint> & GetMpSet() const { return msetpMp; }
    const std::map<int, PtrMapPoint>& GetMpMap() const { return mmapId2pMp; }
    PtrMapPoint GetMp(int _id) const;
    bool InsertMp(PtrMapPoint _ptr);


    // Functions on Odometry Measure
    bool InsertMsrOdo(PtrMsrSe2Kf2Kf _ptr);
    void ClearMsrOdo();
    inline const std::set<PtrMsrSe2Kf2Kf> & GetMsrOdoSet() const { return msetMsrOdo; }
    PtrMsrSe2Kf2Kf GetMsrOdobyKfHead(PtrKeyFrame _pKf) const;
    PtrMsrSe2Kf2Kf GetMsrOdobyKfTail(PtrKeyFrame _pKf) const;
    PtrKeyFrame GetKfOdoNext(PtrKeyFrame _pKf) const;
    PtrKeyFrame GetKfOdoLast(PtrKeyFrame _pKf) const;


    // Functions on Pt3 Mark Measure
    bool InsertMsrMk(PtrMsrPt3Kf2Mk ptr);
    inline const std::set<PtrMsrPt3Kf2Mk> & GetMsrMkSet() const { return msetMsrMk; }    
    std::set<PtrMsrPt3Kf2Mk> GetMsrMkbyKf(PtrKeyFrame _pKf) const;
    std::set<PtrMsrPt3Kf2Mk> GetMsrMkbyMk(PtrMark _pMk) const;
    PtrMsrPt3Kf2Mk GetMsrMkbyKfMk(PtrKeyFrame _pKf, PtrMark _pMk) const;
    std::set<PtrMark> GetMkbyKf(PtrKeyFrame _pKf) const;
    std::set<PtrKeyFrame> GetKfbyMk(PtrMark _pMk) const;

    // Todo: Functions on UV2 MapPoint Measure
    bool InsertMsrMp(PtrMsrUVKf2Mp ptr);
    bool RemoveMsrMp(PtrMsrUVKf2Mp ptr);


    //! IO Functions
    std::vector<std::string> SplitString(const std::string _str, const std::string _separator);
    bool ParseOdoData(const std::string _str, Se2& _odo, int& _id);
    void LoadImage(int _id, cv::Mat& _img);

protected:

    //! Elements
    // Frame
    std::set<PtrFrame> msetpFrame;
    std::map<int, PtrFrame> mmapId2pFrame;
    // KeyFrame
    std::set<PtrKeyFrame> msetpKf;
    std::map<int, PtrKeyFrame> mmapId2pKf;
    // Mark
    std::set<PtrMark> msetpMk;
    std::map<int, PtrMark> mmapId2pMk;
    // MapPoint
    std::set<PtrMapPoint> msetpMp;
    std::map<int, PtrMapPoint> mmapId2pMp;


    //! Measures
    // Odometry
    std::set<PtrMsrSe2Kf2Kf> msetMsrOdo;
    std::map<PtrKeyFrame, PtrMsrSe2Kf2Kf> mmapKfHead2MsrOdo;
    std::map<PtrKeyFrame, PtrMsrSe2Kf2Kf> mmapKfTail2MsrOdo;
    // Mark
    std::set<PtrMsrPt3Kf2Mk> msetMsrMk;
    std::multimap<PtrKeyFrame ,PtrMsrPt3Kf2Mk> mmapKf2MsrMk;
    std::multimap<PtrMark, PtrMsrPt3Kf2Mk> mmapMk2MsrMk;
    // MapPoint
    std::set<PtrMsrUVKf2Mp> msetMsrMp;
    std::multimap<PtrKeyFrame, PtrMsrUVKf2Mp> mmapKf2MsrMp;
    std::multimap<PtrMapPoint, PtrMsrUVKf2Mp> mmapMp2MsrMp;


    //! Configures

    string mstrFoldPathMain;
    string mstrFoldPathImg;
    string mstrFilePathOdo;
    string mstrFilePathCam;

    int mNumFrame;

    double mThreshOdoLin;
    double mThreshOdoRot;
//    double mOdoLinErrR;
//    double mOdoLinErrMin;
//    double mOdoRotErrR;
//    double mOdoRotErrRLin;
//    double mOdoRotErrMin;
//    double mAmkZErrRZ;
//    double mAmkZErrMin;
//    double mAmkXYErrRZ;
//    double mAmkXYErrMin;
};

class DatasetAruco : public Dataset {

//    friend class SolverAruco;

public:
    DatasetAruco();
    ~DatasetAruco() = default;

    void CreateKeyFrames();
//    void CreateMarks();
//    void CreateMsrMks() {}

    // Functions on MkAruco
    inline const std::set<PtrMarkAruco> GetMkArucoSet() { return msetpMkAruco; }
    inline const std::map<int, PtrMarkAruco> GetMkArucoMap() { return mmapId2pMkAruco; }
    bool InsertMkAruco(PtrMarkAruco _pMkAruco);
    PtrMarkAruco GetMkAruco(int _id) const;

    // Functions on KfAruco
    inline const std::set<PtrKeyFrameAruco> GetKfArucoSet() { return msetpKfAruco; }
    inline const std::map<int,PtrKeyFrameAruco> GetKfArucoMap() { return mmapId2pKfAruco; }
    bool InsertKfAruco(PtrKeyFrameAruco _pKfAruco);
    PtrKeyFrameAruco GetKfAruco(int _id) const;

protected:

    double mMarkerSize;

    aruco::CameraParameters mCamParam;
    aruco::MarkerDetector mMDetector;

    std::set<PtrMarkAruco> msetpMkAruco;
    std::map<int, PtrMarkAruco> mmapId2pMkAruco;
    std::set<PtrKeyFrameAruco> msetpKfAruco;
    std::map<int, PtrKeyFrameAruco> mmapId2pKfAruco;
};

class DatasetOrb : public Dataset {

    friend class SolverOrb;

public:
    DatasetOrb();

    void CreateKeyFrames();

    // keyframe functions
    const std::set<PtrKeyFrameOrb> GetKfOrbSet() { return msetpKfOrb; }
    const std::map<int, PtrKeyFrameOrb> GetKfOrbMap() { return mmapId2pKfOrb; }
    bool InsertKfOrb(PtrKeyFrameOrb _pKfOrb);
    PtrKeyFrameOrb GetKfOrb(int _id) const;

    // mappoint functions
    const std::set<PtrMapPointOrb> GetMpOrbSet() { return msetpMpOrb; }
    const std::map<int, PtrMapPointOrb> GetMpOrbMap() { return mmapId2pMpOrb; }
    bool InsertMpOrb(PtrMapPointOrb _pMpOrb);
    PtrMapPointOrb GetMpOrb(int _id) const;

protected:    
    // orb algorithms
    ORBextractor mOrbExtractor;
    ORBVocabulary mOrbVocalubary;

    // camera intrinsic parameters
    cv::Mat mCamMatrix;
    cv::Mat mDistCoeff;

    // storage of orb-keyframes
    std::set<PtrKeyFrameOrb> msetpKfOrb;
    std::map<int, PtrKeyFrameOrb> mmapId2pKfOrb;

    // storage of orb-mappoints
    std::set<PtrMapPointOrb> msetpMpOrb;
    std::map<int, PtrMapPointOrb> mmapId2pMpOrb;


};

}
#endif

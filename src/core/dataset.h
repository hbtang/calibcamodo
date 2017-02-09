#ifndef DATASET_H
#define DATASET_H

#include "type.h"
#include "frame.h"
#include "mark.h"
#include "measure.h"
#include "measurepool.h"

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

    // Functions on Frame
    inline const std::set<PtrFrame>& GetFrameSet() const { return msetpFrame; }
    inline const std::map<int, PtrFrame>& GetFrameMap() const { return mmapId2pFrame; }
    PtrFrame GetFrame(int _id) const;
    bool AddFrame(PtrFrame _ptr);


    // Functions on KeyFrame
    inline const std::set<PtrKeyFrame> & GetKfSet() const { return msetpKf; }
    inline const std::map<int, PtrKeyFrame>& GetKfMap() const { return mmapId2pKf; }
    PtrKeyFrame GetKf(int _id) const;
    bool AddKf(PtrKeyFrame _ptr);


    // Functions on Mark
    inline const std::set<PtrMark> & GetMkSet() const { return msetpMk; }
    inline const std::map<int, PtrMark>& GetMkMap() const { return mmapId2pMk; }
    PtrMark GetMk(int _id) const;
    bool AddMk(PtrMark _ptr);

    // Functions on MapPoint
    const std::set<PtrMapPoint> & GetMpSet() const { return msetpMp; }
    const std::map<int, PtrMapPoint>& GetMpMap() const { return mmapId2pMp; }
    PtrMapPoint GetMp(int _id) const;
    bool AddMp(PtrMapPoint _ptr);


    // Functions on Odometry Measure
    bool AddMsrOdo(PtrMsrSe2Kf2Kf _ptr);
    void ClearMsrOdo();
    inline const std::set<PtrMsrSe2Kf2Kf> & GetMsrOdoSet() const { return msetMsrOdo; }
    PtrMsrSe2Kf2Kf GetMsrOdobyKfHead(PtrKeyFrame _pKf) const;
    PtrMsrSe2Kf2Kf GetMsrOdobyKfTail(PtrKeyFrame _pKf) const;
    PtrKeyFrame GetKfOdoNext(PtrKeyFrame _pKf) const;
    PtrKeyFrame GetKfOdoLast(PtrKeyFrame _pKf) const;


    // Functions on Pt3 Mark Measure
    bool AddMsrMk(PtrMsrPt3Kf2Mk _pMsr) { return mmsrplMk.AddMsr(_pMsr); }
    std::set<PtrMsrPt3Kf2Mk> GetMsrMkAll() const { return mmsrplMk.GetMsrAll(); }
    std::set<PtrMsrPt3Kf2Mk> GetMsrMkByKf(PtrKeyFrame _pKf) const { return mmsrplMk.GetMsrByKf(_pKf); }
    std::set<PtrMsrPt3Kf2Mk> GetMsrMkByMk(PtrMark _pMk) const { return mmsrplMk.GetMsrByMk(_pMk); }
    PtrMsrPt3Kf2Mk GetMsrMkByKfMk(PtrKeyFrame _pKf, PtrMark _pMk) const { return mmsrplMk.GetMsrByKfMk(_pKf, _pMk); }
    std::set<PtrMark> GetMkByKf(PtrKeyFrame _pKf) const { return mmsrplMk.GetMkByKf(_pKf); }
    std::set<PtrKeyFrame> GetKfbyMk(PtrMark _pMk) const { return mmsrplMk.GetKfByMk(_pMk); }

    // Functions on UV Mappoint Measure
    bool AddMsrMp(PtrMsrUVKf2Mp _pMsr) { return mmsrplMp.AddMsr(_pMsr); }
    std::set<PtrMsrUVKf2Mp> GetMsrMpAll() const { return mmsrplMp.GetMsrAll(); }
    std::set<PtrMsrUVKf2Mp> GetMsrMpByKf(PtrKeyFrame _pKf) const { return mmsrplMp.GetMsrByKf(_pKf); }
    std::set<PtrMsrUVKf2Mp> GetMsrMpByMp(PtrMapPoint _pMp) const { return mmsrplMp.GetMsrByMp(_pMp); }
    PtrMsrUVKf2Mp GetMsrMpByKfId(PtrKeyFrame _pKf, int _idKp) const { return mmsrplMp.GetMsrByKfId(_pKf, _idKp); }
    PtrMsrUVKf2Mp GetMsrMpByKfMp(PtrKeyFrame _pKf, PtrMapPoint _pMp) const { return mmsrplMp.GetMsrByKfMp(_pKf, _pMp); }
    std::set<PtrKeyFrame> GetKfByMp(PtrMapPoint _pMp) const { return mmsrplMp.GetKfByMp(_pMp); }
    std::set<PtrMapPoint> GetMpByKf(PtrKeyFrame _pKf) const { return mmsrplMp.GetMpByKf(_pKf); }


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
    MeasurePoolPt3Kf2Mk mmsrplMk;

//    std::set<PtrMsrPt3Kf2Mk> msetMsrMk;
//    std::multimap<PtrKeyFrame ,PtrMsrPt3Kf2Mk> mmapKf2MsrMk;
//    std::multimap<PtrMark, PtrMsrPt3Kf2Mk> mmapMk2MsrMk;

    // MapPoint
    MeasurePoolUVKf2Mp mmsrplMp;

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
    bool AddMkAruco(PtrMarkAruco _pMkAruco);
    PtrMarkAruco GetMkAruco(int _id) const;

    // Functions on KfAruco
    inline const std::set<PtrKeyFrameAruco> GetKfArucoSet() { return msetpKfAruco; }
    inline const std::map<int,PtrKeyFrameAruco> GetKfArucoMap() { return mmapId2pKfAruco; }
    bool AddKfAruco(PtrKeyFrameAruco _pKfAruco);
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
    bool AddKfOrb(PtrKeyFrameOrb _pKfOrb);
    PtrKeyFrameOrb GetKfOrb(int _id) const;

    // mappoint functions
    const std::set<PtrMapPointOrb> GetMpOrbSet() { return msetpMpOrb; }
    const std::map<int, PtrMapPointOrb> GetMpOrbMap() { return mmapId2pMpOrb; }
    bool AddMpOrb(PtrMapPointOrb _pMpOrb);
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

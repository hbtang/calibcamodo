#ifndef DATASET_H
#define DATASET_H

#include "type.h"
#include "frame.h"
#include "mapmark.h"
#include "mappoint.h"
#include "measure.h"
#include "measurepool.h"

#include "orb/ORBVocabulary.h"
#include "orb/ORBextractor.h"

namespace calibcamodo {

class Dataset {
//    friend class Solver;
    friend class MakerOnlineBase;

public:
    Dataset();
    ~Dataset() = default;

    // Function interface
    void CreateFrames();
    set<PtrFrame> SelectFrame() const;
    virtual void CreateKeyFrames() = 0;

    // Functions on Frame
    const std::set<PtrFrame>& GetFrameSet() const { return msetpFrame; }
    const std::map<int, PtrFrame>& GetFrameMap() const { return mmapId2pFrame; }
    PtrFrame GetFrame(int _id) const;
    bool AddFrame(PtrFrame _ptr);

    // Functions on KeyFrame
    inline const std::set<PtrKeyFrame> & GetKfSet() const { return msetpKf; }
    inline const std::map<int, PtrKeyFrame>& GetKfMap() const { return mmapId2pKf; }
    PtrKeyFrame GetKf(int _id) const;
    bool AddKf(PtrKeyFrame _ptr);

    // Functions on Mark
    const std::set<PtrMapMark> & GetMkSet() const { return msetpMk; }
    const std::map<int, PtrMapMark>& GetMkMap() const { return mmapId2pMk; }
    PtrMapMark GetMk(int _id) const;
    bool AddMk(PtrMapMark _ptr);

    // Functions on MapPoint
    const std::set<PtrMapPoint> & GetMpSet() const { return msetpMp; }
    const std::map<int, PtrMapPoint>& GetMpMap() const { return mmapId2pMp; }
    PtrMapPoint GetMp(int _id) const;
    bool AddMp(PtrMapPoint _ptr);


    // Functions on Odometry Measure
    bool AddMsrOdo(PtrMsrSe2Kf2Kf _pMsr) { return mmsrplOdo.AddMsr(_pMsr); }
    void ClearMsrOdo() { mmsrplOdo.ClearAll(); }
    std::set<PtrMsrSe2Kf2Kf> GetMsrOdoSet() const { return mmsrplOdo.GetMsrAll(); }
    PtrMsrSe2Kf2Kf GetMsrOdoByKfHead(PtrKeyFrame _pKf) const { return mmsrplOdo.GetMsrOdoByKfHead(_pKf); }
    PtrMsrSe2Kf2Kf GetMsrOdoByKfTail(PtrKeyFrame _pKf) const { return mmsrplOdo.GetMsrOdoByKfTail(_pKf); }
    PtrKeyFrame GetKfOdoNext(PtrKeyFrame _pKf) const { return mmsrplOdo.GetKfOdoLast(_pKf); }
    PtrKeyFrame GetKfOdoLast(PtrKeyFrame _pKf) const { return mmsrplOdo.GetKfOdoNext(_pKf); }


    // Functions on Pt3 Mark Measure
    bool AddMsrMk(PtrMsrPt3Kf2Mk _pMsr) { return mmsrplMk.AddMsr(_pMsr); }
    std::set<PtrMsrPt3Kf2Mk> GetMsrMkAll() const { return mmsrplMk.GetMsrAll(); }
    std::set<PtrMsrPt3Kf2Mk> GetMsrMkByKf(PtrKeyFrame _pKf) const { return mmsrplMk.GetMsrByKf(_pKf); }
    std::set<PtrMsrPt3Kf2Mk> GetMsrMkByMk(PtrMapMark _pMk) const { return mmsrplMk.GetMsrByMk(_pMk); }
    PtrMsrPt3Kf2Mk GetMsrMkByKfMk(PtrKeyFrame _pKf, PtrMapMark _pMk) const { return mmsrplMk.GetMsrByKfMk(_pKf, _pMk); }
    std::set<PtrMapMark> GetMkByKf(PtrKeyFrame _pKf) const { return mmsrplMk.GetMkByKf(_pKf); }
    std::set<PtrKeyFrame> GetKfbyMk(PtrMapMark _pMk) const { return mmsrplMk.GetKfByMk(_pMk); }

    // Functions on UV Mappoint Measure
    bool AddMsrMp(PtrMsrUVKf2Mp _pMsr) { return mmsrplMp.AddMsr(_pMsr); }
    std::set<PtrMsrUVKf2Mp> GetMsrMpAll() const { return mmsrplMp.GetMsrAll(); }
    std::set<PtrMsrUVKf2Mp> GetMsrMpByKf(PtrKeyFrame _pKf) const { return mmsrplMp.GetMsrByKf(_pKf); }
    std::set<PtrMsrUVKf2Mp> GetMsrMpByMp(PtrMapPoint _pMp) const { return mmsrplMp.GetMsrByMp(_pMp); }
    PtrMsrUVKf2Mp GetMsrMpByKfId(PtrKeyFrame _pKf, int _idKp) const { return mmsrplMp.GetMsrByKfId(_pKf, _idKp); }
    PtrMsrUVKf2Mp GetMsrMpByKfMp(PtrKeyFrame _pKf, PtrMapPoint _pMp) const { return mmsrplMp.GetMsrByKfMp(_pKf, _pMp); }
    std::set<PtrKeyFrame> GetKfByMp(PtrMapPoint _pMp) const { return mmsrplMp.GetKfByMp(_pMp); }
    std::set<PtrMapPoint> GetMpByKf(PtrKeyFrame _pKf) const { return mmsrplMp.GetMpByKf(_pKf); }
    PtrMapPoint GetMpByKfId(PtrKeyFrame _pKf, int _idKp) const { return mmsrplMp.GetMpByKfId(_pKf, _idKp); }

    // Functions on Parameters
    Se3 GetCamOffset() { return mSe3bc; }
    void SetCamOffset(Se3 _se3cb) { mSe3bc = _se3cb; }
    const cv::Mat& GetCamMat() const { return mCamMatrix; }
    const cv::Mat& GetCamDist() const { return mDistCoeff; }


    //! IO Functions
    std::vector<std::string> SplitString(const std::string _str, const std::string _separator);
    bool ParseOdoData(const std::string _str, Se2& _odo, int& _id);
    void LoadImage(int _id, cv::Mat& _img);

protected:

    //! Parameters
    Se3 mSe3bc;
    cv::Mat mCovSe3bc;  // order: rvec - tvec
    // camera intrinsic parameters
    cv::Mat mCamMatrix;
    cv::Mat mDistCoeff;

    //! Elements
    // Frame
    std::set<PtrFrame> msetpFrame;
    std::map<int, PtrFrame> mmapId2pFrame;
    // KeyFrame
    std::set<PtrKeyFrame> msetpKf;
    std::map<int, PtrKeyFrame> mmapId2pKf;
    // Mark
    std::set<PtrMapMark> msetpMk;
    std::map<int, PtrMapMark> mmapId2pMk;
    // MapPoint
    std::set<PtrMapPoint> msetpMp;
    std::map<int, PtrMapPoint> mmapId2pMp;

    //! Measures
    // Odometry
    MeasurePoolSe2Kf2Kf mmsrplOdo;
    // Mark
    MeasurePoolPt3Kf2Mk mmsrplMk;
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


    // todo: for online filter
public:
    bool InitKfForFilter();
    bool RenewKfForFilter();
    PtrKeyFrame GetKfNow() const { return mpKfNow; }
    PtrKeyFrame GetKfLast() const { return mpKfLast; }


protected:
    bool mbIfInitFilter;
    PtrKeyFrame mpKfNow;
    PtrKeyFrame mpKfLast;
};

class DatasetAruco : public Dataset {

public:
    DatasetAruco();
    ~DatasetAruco() = default;

    void CreateKeyFrames();

    // Functions on MkAruco
    inline const std::set<PtrMapMarkAruco> GetMkArucoSet() { return msetpMkAruco; }
    inline const std::map<int, PtrMapMarkAruco> GetMkArucoMap() { return mmapId2pMkAruco; }
    bool AddMkAruco(PtrMapMarkAruco _pMkAruco);
    PtrMapMarkAruco GetMkAruco(int _id) const;

    // Functions on KfAruco
    inline const std::set<PtrKeyFrameAruco> GetKfArucoSet() { return msetpKfAruco; }
    inline const std::map<int,PtrKeyFrameAruco> GetKfArucoMap() { return mmapId2pKfAruco; }
    bool AddKfAruco(PtrKeyFrameAruco _pKfAruco);
    PtrKeyFrameAruco GetKfAruco(int _id) const;

protected:

    double mMarkerSize;

    aruco::CameraParameters mCamParam;
    aruco::MarkerDetector mMDetector;

    std::set<PtrMapMarkAruco> msetpMkAruco;
    std::map<int, PtrMapMarkAruco> mmapId2pMkAruco;
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

    PtrKeyFrameOrb GetKfOrbNow() const {
        return dynamic_pointer_cast<KeyFrameOrb>(mpKfNow);
    }
    PtrKeyFrameOrb GetKfOrbLast() const {
        return dynamic_pointer_cast<KeyFrameOrb>(mpKfLast);
    }

protected:    
    // orb algorithms
    ORBextractor mOrbExtractor;
    ORBVocabulary mOrbVocalubary;

    // storage of orb-keyframes
    std::set<PtrKeyFrameOrb> msetpKfOrb;
    std::map<int, PtrKeyFrameOrb> mmapId2pKfOrb;

    // storage of orb-mappoints
    std::set<PtrMapPointOrb> msetpMpOrb;
    std::map<int, PtrMapPointOrb> mmapId2pMpOrb;

};



}
#endif

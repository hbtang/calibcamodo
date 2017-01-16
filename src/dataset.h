#ifndef DATASET_H
#define DATASET_H

#include "type.h"
#include "aruco/aruco.h"

namespace calibcamodo {

class Frame;
class KeyFrame;

class Dataset {

public:

    Dataset();
    Dataset(string _strFolderPathMain, int _numFrame, double _markerSize);
    ~Dataset();

    void CreateFrame();
    void CreateKeyFrame();
    void CreateMarkMeasure();

    void InitKf(Se3 _se3bc);
    void InitMk();
    void InitAll(Se3 _se3bc) {
        InitKf(_se3bc);
        InitMk();
    }

    inline const set<PtrFrame> & GetFrame() const { return msetpFrame; }

    inline const set<PtrKeyFrame> & GetKfSet() const { return msetpKf; }
    inline const map<int, PtrKeyFrame> & GetKfMap() const { return mmapId2pKf; }

    inline const set<PtrArucoMark> & GetMkSet() const { return msetpMk; }
    inline const map<int, PtrArucoMark> & GetMkMap() const {return mmapId2pMk; }

    inline const set<PtrMsrKf2AMk> & GetMsrMk() const { return msetMsrMk; }
    inline const set<PtrMsrSe2Kf2Kf> & GetMsrOdo() const { return msetMsrOdo; }

private:

    set<PtrFrame> msetpFrame;
    map<int, PtrFrame> mmapId2pFrame;
    bool InsertFrame(PtrFrame ptr);
    bool DeleteFrame(PtrFrame ptr);

    set<PtrKeyFrame> msetpKf;
    map<int, PtrKeyFrame> mmapId2pKf;
    bool InsertKf(PtrKeyFrame ptr);
    bool DeleteKf(PtrKeyFrame ptr);

    set<PtrArucoMark> msetpMk;
    map<int, PtrArucoMark> mmapId2pMk;
    bool InsertMk(PtrArucoMark& ptr);
    bool DeleteMk(PtrArucoMark ptr);
    PtrArucoMark FindMk(int id);

    set<PtrMsrKf2AMk> msetMsrMk;
    bool InsertMsrMk(PtrMsrKf2AMk ptr);
    bool DeleteMsrMk(PtrMsrKf2AMk ptr);

    set<PtrMsrSe2Kf2Kf> msetMsrOdo;
    bool InsertMsrOdo(PtrMsrKf2AMk ptr);
    bool DeleteMsrOdo(PtrMsrKf2AMk ptr);

    aruco::CameraParameters mCamParam;
    aruco::MarkerDetector mMDetector;

    string mstrFoldPathMain;
    string mstrFoldPathImg;
    string mstrFilePathOdo;
    string mstrFilePathCam;

    int mNumFrame;
    double mMarkerSize;

    double mThreshOdoLin;
    double mThreshOdoRot;

    vector<string> SplitString(const string _str, const string _separator);
    bool ParseOdoData(const string _str, Se2 &_odo, int &_id);
};

}
#endif

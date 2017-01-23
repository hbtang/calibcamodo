#include "dataset.h"
#include "frame.h"
#include "measure.h"
#include "mark.h"
#include "config.h"

namespace calibcamodo {

using namespace std;
using namespace cv;
using namespace aruco;

Dataset::Dataset() {

    mNumFrame = Config::NUM_FRAME;
    mMarkerSize = Config::MARK_SIZE;
    mstrFoldPathMain = Config::STR_FOLDERPATH_MAIN;
    mstrFoldPathImg = Config::STR_FOlDERPATH_IMG;
    mstrFilePathCam = Config::STR_FILEPATH_CAM;
    mstrFilePathOdo = Config::STR_FILEPATH_ODO;

    // load camera intrinsics
    mCamParam.readFromXMLFile(mstrFilePathCam);

    // set aruco mark detector
    int ThePyrDownLevel = 0;
    int ThresParam1 = 19;
    int ThresParam2 = 15;
    mMDetector.pyrDown(ThePyrDownLevel);
    mMDetector.setCornerRefinementMethod(MarkerDetector::LINES);
    mMDetector.setThresholdParams(ThresParam1, ThresParam2);

    // select keyframe
    mThreshOdoLin   = Config::DATASET_THRESH_KF_ODOLIN;
    mThreshOdoRot   = Config::DATASET_THRESH_KF_ODOROT;

    // load odometry error configure
    mOdoLinErrR     = Config::CALIB_ODOLIN_ERRR;
    mOdoLinErrMin   = Config::CALIB_ODOLIN_ERRMIN;
    mOdoRotErrR     = Config::CALIB_ODOLIN_ERRR;
    mOdoRotErrRLin  = Config::CALIB_ODOROT_ERRRLIN;
    mOdoRotErrMin   = Config::CALIB_ODOROT_ERRMIN;

    // load mark error configure
    mAmkZErrRZ      = Config::CALIB_AMKZ_ERRRZ;
    mAmkZErrMin     = Config::CALIB_AMKZ_ERRMIN;
    mAmkXYErrRZ     = Config::CALIB_AMKXY_ERRRZ;
    mAmkXYErrMin    = Config::CALIB_AMKXY_ERRMIN;

}

PtrFrame Dataset::GetFrame(int _id) const {
    PtrFrame pRet = nullptr;
    if(mmapId2pFrame.count(_id))
        pRet = mmapId2pFrame.at(_id);
    return pRet;
}

bool Dataset::InsertFrame(PtrFrame _ptr) {
    if (msetpFrame.count(_ptr))
        return false;

    int id = _ptr->GetId();
    if (mmapId2pFrame.count(id))
        return false;

    msetpFrame.insert(_ptr);
    mmapId2pFrame[id] = _ptr;
    return true;
}

PtrKeyFrame Dataset::GetKf(int _id) const {
    PtrKeyFrame pRet = nullptr;
    if(mmapId2pKf.count(_id))
        pRet = mmapId2pKf.at(_id);
    return pRet;
}

bool Dataset::InsertKf(PtrKeyFrame _ptr) {
    if (msetpKf.count(_ptr))
        return false;

    int id = _ptr->GetId();
    if (mmapId2pKf.count(id))
        return false;

    msetpKf.insert(_ptr);
    mmapId2pKf[id] = _ptr;
    return true;
}

PtrMark Dataset::GetMk(int _id) const {
    PtrMark pRet = nullptr;
    if(mmapId2pMk.count(_id))
        pRet = mmapId2pMk.at(_id);
    return pRet;
}

bool Dataset::InsertMk(PtrMark _ptr) {
    if (msetpMk.count(_ptr))
        return false;
    int id = _ptr->GetId();
    if (mmapId2pMk.count(id))
        return false;
    msetpMk.insert(_ptr);
    mmapId2pMk[id] = _ptr;
    return true;
}

bool Dataset::InsertMsrOdo(PtrMsrSe2Kf2Kf _ptr) {
    assert(msetMsrOdo.count(_ptr) == 0);
    assert(mmapKfHead2MsrOdo.count(_ptr->pKfHead) == 0);
    assert(mmapKfTail2MsrOdo.count(_ptr->pKfTail) == 0);

    msetMsrOdo.insert(_ptr);
    mmapKfHead2MsrOdo[_ptr->pKfHead] = _ptr;
    mmapKfTail2MsrOdo[_ptr->pKfTail] = _ptr;
    return true;
}

PtrMsrSe2Kf2Kf Dataset::GetMsrOdobyKfHead(PtrKeyFrame _pKf) const {
    PtrMsrSe2Kf2Kf pRet = nullptr;
    if(mmapKfHead2MsrOdo.count(_pKf))
        pRet = mmapKfHead2MsrOdo.at(_pKf);
    return pRet;
}

PtrMsrSe2Kf2Kf Dataset::GetMsrOdobyKfTail(PtrKeyFrame _pKf) const {
    PtrMsrSe2Kf2Kf pRet = nullptr;
    if(mmapKfTail2MsrOdo.count(_pKf))
        pRet = mmapKfTail2MsrOdo.at(_pKf);
    return pRet;
}

PtrKeyFrame Dataset::GetKfOdoNext(PtrKeyFrame _pKf) const {
    PtrMsrSe2Kf2Kf pMsr = GetMsrOdobyKfHead(_pKf);
    if(pMsr)
        return pMsr->pKfTail;
    else
        return nullptr;
}

PtrKeyFrame Dataset::GetKfOdoLast(PtrKeyFrame _pKf) const {
    PtrMsrSe2Kf2Kf pMsr = GetMsrOdobyKfTail(_pKf);
    if(pMsr)
        return pMsr->pKfHead;
    else
        return nullptr;
}

bool Dataset::InsertMsrMk(PtrMsrPt3Kf2Mk _ptr) {
    assert(msetMsrMk.count(_ptr) == 0);
    msetMsrMk.insert(_ptr);
    mmapKf2MsrMk.emplace(_ptr->pKf, _ptr);
    mmapMk2MsrMk.emplace(_ptr->pMk, _ptr);
    return true;
}

std::set<PtrMsrPt3Kf2Mk> Dataset::GetMsrMkbyKf(PtrKeyFrame _pKf) const {
    set<PtrMsrPt3Kf2Mk> setpMsrRet;
    auto iter_lowerbound = mmapKf2MsrMk.lower_bound(_pKf);
    auto iter_upperbound = mmapKf2MsrMk.upper_bound(_pKf);
    for (auto iter = iter_lowerbound; iter != iter_upperbound; iter++) {
        setpMsrRet.insert((*iter).second);
    }
    return setpMsrRet;
}

std::set<PtrMsrPt3Kf2Mk> Dataset::GetMsrMkbyMk(PtrMark _pMk) const {
    set<PtrMsrPt3Kf2Mk> setpMsrRet;
    auto iter_lowerbound = mmapMk2MsrMk.lower_bound(_pMk);
    auto iter_upperbound = mmapMk2MsrMk.upper_bound(_pMk);
    for (auto iter = iter_lowerbound; iter != iter_upperbound; iter++) {
        setpMsrRet.insert((*iter).second);
    }
    return setpMsrRet;
}

std::set<PtrMark> Dataset::GetMkbyKf(PtrKeyFrame _pKf) const {
    set<PtrMsrPt3Kf2Mk> setpMsrRet = GetMsrMkbyKf(_pKf);
    set<PtrMark> setpMkRet;
    for(auto pMsr : setpMsrRet) {
        setpMkRet.insert(static_cast<PtrMsrPt3Kf2Mk>(pMsr)->pMk);
    }
    return setpMkRet;
}

std::set<PtrKeyFrame> Dataset::GetKfbyMk(PtrMark _pMk) const {
    set<PtrMsrPt3Kf2Mk> setpMsrRet = GetMsrMkbyMk(_pMk);
    set<PtrKeyFrame> setpKfRet;
    for(auto pMsr : setpMsrRet) {
        setpKfRet.insert(static_cast<PtrMsrPt3Kf2Mk>(pMsr)->pKf);
    }
    return setpKfRet;
}

bool Dataset::ParseOdoData(const string _str, Se2& _odo, int& _id) {
    vector<string> vec_str = SplitString(_str, " ");

    // fail
    if (vec_str[0] == "#") return false;

    // read data
    _id = atof(vec_str[0].c_str());
    _odo.x = atof(vec_str[3].c_str());
    _odo.y = atof(vec_str[4].c_str());
    _odo.theta = atof(vec_str[5].c_str());
    return true;
}

vector<string> Dataset::SplitString(const string _str, const string _separator) {
    string str = _str;
    vector<string> vecstr_return;
    int cut_at;
    while ((cut_at = str.find_first_of(_separator)) != str.npos) {
        if (cut_at > 0) {
            vecstr_return.push_back(str.substr(0, cut_at));
        }
        str = str.substr(cut_at + 1);
    }
    if (str.length() > 0) {
        vecstr_return.push_back(str);
    }
    return vecstr_return;
}

void Dataset::CreateFrames() {

    // load image
    set<int> setIdImgExist;
    //    map<int, bool> mapId2ImgExist;
    for (int i = 0; i < mNumFrame; ++i) {
        string strImgPath = mstrFoldPathImg + to_string(i) + ".bmp";

        // DEBUG
        ifstream fileImg(strImgPath);
        if (fileImg) {
            setIdImgExist.insert(i);
            //            mapId2Img[i] = img;
        }
    }

    // load odometry
    map<int, Se2> mapId2Odo;
    ifstream logFile_stream(mstrFilePathOdo);
    string str_tmp;
    while(getline(logFile_stream, str_tmp)) {
        // read time info
        Se2 odo_tmp;
        int id_tmp;
        if (ParseOdoData(str_tmp, odo_tmp, id_tmp)) {
            mapId2Odo[id_tmp] = odo_tmp;
        }
    }

    // build frame vector
    int maxIdImg = *setIdImgExist.crbegin();
    int maxIdOdo = mapId2Odo.crbegin()->first;
    int maxId = maxIdImg > maxIdOdo ? maxIdImg : maxIdOdo;
    for (int i = 0; i <= maxId; ++i) {
        const auto iterImg = setIdImgExist.find(i);
        const auto iterOdo = mapId2Odo.find(i);
        if (iterImg != setIdImgExist.cend() && iterOdo != mapId2Odo.cend()) {
            PtrFrame pf = make_shared<Frame>(iterOdo->second, i);
            InsertFrame(pf);
        }
    }

    return;
}

set<PtrFrame> Dataset::SelectFrame() const {
    set<PtrFrame> setpFrameSelected;
    PtrFrame pFrameLast = *(msetpFrame.cbegin());
    setpFrameSelected.insert(pFrameLast);
    for (auto ptr : msetpFrame) {
        PtrFrame pFrameNew = ptr;
        Se2 dodo = pFrameNew->GetOdo() - pFrameLast->GetOdo();
        double dl = dodo.dist();
        double dr = abs(dodo.theta);
        if (dl > mThreshOdoLin || dr > mThreshOdoRot) {
            setpFrameSelected.insert(pFrameNew);
            pFrameLast = pFrameNew;
        }
    }
    return setpFrameSelected;
}

void Dataset::LoadImage(int _id, cv::Mat& _img) {
    string strImgPath = mstrFoldPathImg + to_string(_id) + ".bmp";
    _img = imread(strImgPath);
}

void Dataset::CreateMsrOdos() {
    msetMsrOdo.clear();
    mmapKfHead2MsrOdo.clear();
    mmapKfTail2MsrOdo.clear();

    for(auto iter1 = mmapId2pKf.cbegin(), iter2 = iter1++;
        iter1 != mmapId2pKf.cend();
        ++iter1, ++iter2) {

        PtrKeyFrame pKfHead = (*iter1).second;
        PtrKeyFrame pKfTail = (*iter2).second;
        Se2 dodo = pKfTail->GetOdo() - pKfHead->GetOdo();
        Mat info = Mat::eye(3,3,CV_32FC1);

        double dist = dodo.dist();
        double stdlin = max(dist*mOdoLinErrR, mOdoLinErrMin);
        double theta = dodo.theta;
        double stdrot = max(max(abs(theta)*mOdoRotErrR, mOdoRotErrMin), dist*mOdoRotErrRLin);

        info.at<float>(0,0) = 1/stdlin/stdlin;
        info.at<float>(1,1) = 1/stdlin/stdlin;
        info.at<float>(2,2) = 1/stdrot/stdrot;

        PtrMsrSe2Kf2Kf pMeasureOdo =
                make_shared<MeasureSe2Kf2Kf>(dodo, info, pKfHead, pKfTail);
        InsertMsrOdo(pMeasureOdo);
    }
}

PtrMsrPt3Kf2Mk Dataset::GetMsrMkbyKfMk(PtrKeyFrame _pKf, PtrMark _pMk) const {
    auto iter_lowerbound = mmapKf2MsrMk.lower_bound(_pKf);
    auto iter_upperbound = mmapKf2MsrMk.upper_bound(_pKf);
    for (auto iter = iter_lowerbound; iter != iter_upperbound; iter++) {
        PtrMsrPt3Kf2Mk pMsrMk = (*iter).second;
        if(pMsrMk->pMk == _pMk)
            return pMsrMk;
    }
    return nullptr;
}

void Dataset::InitKf(Se3 _se3bc) {
    for(auto ptr : msetpKf) {
        PtrKeyFrame pKf = ptr;

        Se2 se2odo = pKf->GetOdo();
        Se2 se2wb = se2odo;
        Se3 se3wb = Se3(se2wb);
        Se3 se3wc = se3wb+_se3bc;

        //        cerr << "se2odo:" << se2odo << endl;
        //        cerr << "se3wb:" << se3wb << endl;
        //        cerr << "se3wc:" << se3wc << endl;

        pKf->SetPoseBase(se2odo);
        pKf->SetPoseCamera(se3wc);

        //        Se3 se3wc_b = pKf->GetPoseCamera();
        //        cerr << "se3wc:" << se3wc_b << endl;
        //        cerr << endl;
    }
}

void Dataset::InitMk() {
    for(auto ptr : msetpMk) {
        PtrMark pMk = ptr;
        set<PtrMsrPt3Kf2Mk> setpMsr = GetMsrMkbyMk(pMk);
        if(!setpMsr.empty()) {
            PtrKeyFrame pKf = (*setpMsr.cbegin())->pKf;
            Se3 se3wc = pKf->GetPoseCamera();
            Se3 se3cm;
            se3cm.tvec = (*setpMsr.cbegin())->pt3.tvec();
            Se3 se3wm = se3wc+se3cm;
            pMk->SetPose(se3wm);

            // DEBUG
            //            cerr << "se3wc" << se3wc.rvec.t() << se3wc.tvec.t() << endl;
            //            cerr << "se3cm" << se3cm.rvec.t() << se3cm.tvec.t() << endl;
            //            cerr << "se3wm" << se3wm.rvec.t() << se3wm.tvec.t() << endl;
            //            cerr << endl;
        }
    }
}

//! Class DatasetAruco

DatasetAruco::DatasetAruco():
    Dataset() {

}

bool DatasetAruco::InsertKfAruco(PtrKeyFrameAruco _pKfAruco) {
    if(!InsertKf(_pKfAruco))
        return false;
    if (msetpKfAruco.count(_pKfAruco))
        return false;
    int id = _pKfAruco->GetId();
    if (mmapId2pKfAruco.count(id))
        return false;
    msetpKfAruco.insert(_pKfAruco);
    mmapId2pKfAruco[id] = _pKfAruco;
    return true;
}

PtrKeyFrameAruco DatasetAruco::GetKfAruco(int _id) const {
    if(mmapId2pKfAruco.count(_id))
        return mmapId2pKfAruco.at(_id);
    else
        return nullptr;
}

bool DatasetAruco::InsertMkAruco(PtrMarkAruco _pMkAruco) {
    if(!InsertMk(_pMkAruco))
        return false;
    if(msetpMkAruco.count(_pMkAruco))
        return false;
    int id = _pMkAruco->GetId();
    if(mmapId2pMkAruco.count(id))
        return false;
    msetpMkAruco.insert(_pMkAruco);
    mmapId2pMkAruco[id] = _pMkAruco;
    return true;
}

PtrMarkAruco DatasetAruco::GetMkAruco(int _id) const {
    if(mmapId2pMkAruco.count(_id))
        return mmapId2pMkAruco.at(_id);
    else
        return nullptr;
}

void DatasetAruco::CreateKeyFrames() {
    set<PtrFrame> setpFrameSelected = SelectFrame();
    for(auto ptr : setpFrameSelected) {
        PtrFrame pframe = ptr;
        int id = pframe->GetId();
        Mat img;
        LoadImage(id, img);
        pframe->SetImg(img);
        PtrKeyFrameAruco pKfAruco = make_shared<KeyFrameAruco>(*pframe);
        pKfAruco->ComputeAruco(mCamParam, mMDetector, mMarkerSize);
        InsertKfAruco(pKfAruco);
    }
}

void DatasetAruco::CreateMarks() {
    // Create aruco marks and mark measurements
    for (auto ptr : msetpKfAruco) {
        PtrKeyFrameAruco pKfAruco = ptr;
        const std::vector<aruco::Marker>& vecAruco = pKfAruco->GetMsrAruco();
        for (auto measure_aruco : vecAruco) {
            int id = measure_aruco.id;
            Mat tvec = measure_aruco.Tvec;

            double z = abs(tvec.at<float>(2));
            double stdxy = max(z*mAmkXYErrRZ, mAmkXYErrMin);
            double stdz = max(z*mAmkZErrRZ, mAmkZErrMin);

            Mat info = Mat::eye(3,3,CV_32FC1);
            info.at<float>(0,0) = 1/stdxy/stdxy;
            info.at<float>(1,1) = 1/stdxy/stdxy;
            info.at<float>(2,2) = 1/stdz/stdz;

            // add new aruco mark into dataset
            PtrMarkAruco pMkAruco = make_shared<MarkAruco>(id, id, mMarkerSize);
            if (!InsertMkAruco(pMkAruco))
                pMkAruco = GetMkAruco(id);

            // add new measurement into dataset
            PtrMsrPt3Kf2Mk pMsrMk = make_shared<MeasurePt3Kf2Mk>(tvec, info, pKfAruco, pMkAruco);
            InsertMsrMk(pMsrMk);
        }
    }
}

//! old functions to remove...

//void Dataset::CreateKeyFrames() {

//    PtrKeyFrame pKeyFrameLast = make_shared<KeyFrame>(**msetpFrame.cbegin());
//    InsertKf(pKeyFrameLast);

//    for (auto ptr : msetpFrame) {
//        PtrFrame pFrameNew = ptr;
//        Se2 dodo = pFrameNew->GetOdo() - pKeyFrameLast->GetOdo();
//        double dl = sqrt(dodo.x*dodo.x + dodo.y*dodo.y);
//        double dr = abs(dodo.theta);
//        Mat info = Mat::eye(3,3,CV_32FC1);
//        if (dl > mThreshOdoLin || dr > mThreshOdoRot) {
//            PtrKeyFrame pKeyFrameNew = make_shared<KeyFrame>(*pFrameNew);
//            InsertKf(pKeyFrameNew);
//            PtrMsrSe2Kf2Kf pMeasureOdo = make_shared<MeasureSe2Kf2Kf>(dodo, info, pKeyFrameLast, pKeyFrameNew);
//            InsertMsrOdo(pMeasureOdo);
//            pKeyFrameLast = pKeyFrameNew;
//        }
//    }

//    LoadKfImage(msetpKf);
//}

//bool Dataset::InsertFrame(PtrFrame ptr) {
//    int id = ptr->GetId();
//    if(mmapId2pFrame.count(id)) {
//        return false;
//    }

//    msetpFrame.insert(ptr);
//    mmapId2pFrame[id] = ptr;
//    return true;
//}

//bool Dataset::DeleteFrame(PtrFrame ptr) {
//    auto iter = msetpFrame.find(ptr);
//    if (iter == msetpFrame.cend()) {
//        return false;
//    }
//    msetpFrame.erase(iter);
//    mmapId2pFrame.erase(ptr->GetId());
//    return true;
//}

//bool Dataset::InsertKf(PtrKeyFrame ptr) {
//    int id = ptr->GetId();
//    if(mmapId2pKf.count(id)) {
//        return false;
//    }
//    msetpKf.insert(ptr);
//    mmapId2pKf[id] = ptr;
//    return true;
//}

//bool Dataset::DeleteKf(PtrKeyFrame ptr) {
//    auto iter = msetpKf.find(ptr);
//    if (iter == msetpKf.cend()) {
//        return false;
//    }
//    msetpKf.erase(iter);
//    mmapId2pKf.erase(ptr->GetId());
//    return true;
//}


//bool Dataset::InsertMk(PtrMarkAruco& ptr) {
//    int id = ptr->GetId();
//    if(mmapId2pMk.count(id)) {
//        ptr = mmapId2pMk[id];
//        return false;
//    }
//    msetpMk.insert(ptr);
//    mmapId2pMk[id] = ptr;
//    return true;
//}

//bool Dataset::DeleteMk(PtrMarkAruco ptr) {
//    auto iter = msetpMk.find(ptr);
//    if (iter == msetpMk.cend()) {
//        return false;
//    }
//    msetpMk.erase(iter);
//    mmapId2pMk.erase(ptr->GetId());
//    return true;
//}

//PtrMarkAruco Dataset::FindMk(int id) {
//    auto iter = mmapId2pMk.find(id);
//    if (iter != mmapId2pMk.cend())
//        return iter->second;
//    else
//        return nullptr;
//}

//void Dataset::CreateMarkMeasure() {
// TODO ...
//    for (auto pkf : msetpKf) {
//        const vector<Marker>& vecMeasureAruco = pkf->GetMsrAruco();
//        for (auto measure_aruco : vecMeasureAruco) {
//            // read data from aruco detect
//            int id = measure_aruco.id;
//            Mat rvec = measure_aruco.Rvec;
//            Mat tvec = measure_aruco.Tvec;
//            Mat info = Mat::eye(6,6,CV_32FC1);

//            // add new aruco mark into dataset
//            PtrMarkAruco pamk = make_shared<MarkAruco>(id);
//            InsertMk(pamk);

//            // add new measurement into dataset
//            PtrMsrKf2AMk pmeas = make_shared<MeasureKf2AMk>(rvec, tvec, info, pkf, pamk);
//            InsertMsrMk(pmeas);
//        }
//    }
//}

//bool Dataset::InsertMsrMk(PtrMsrKf2AMk pmsr) {
//    msetMsrMk.insert(pmsr);
//    PtrKeyFrame pKf = pmsr->pKf;
//    PtrMarkAruco pMk = pmsr->pMk;
//    pKf->InsertMsrMk(pmsr);
//    pMk->InsertMsrMk(pmsr);
//}



//void Dataset::LoadKfImage(const set<PtrKeyFrame> &_setpKF) {
//    for (auto ptr : _setpKF) {
//        PtrKeyFrame pKf = ptr;
//        int id = pKf->GetId();
//        string strImgPath = mstrFoldPathImg + to_string(id) + ".bmp";
//        Mat img = imread(strImgPath);
//        pKf->SetImg(img);
//    }
//}

//void DatasetAruco::ComputeKfAruco(const set<PtrKeyFrame> &_setpKF) {
//    for (auto ptr : _setpKF) {
//        PtrKeyFrame pKf = ptr;
//        pKf->ComputeAruco(mCamParam, mMDetector, mMarkerSize);
//    }
//}

}

#include "dataset.h"
#include "config.h"

using namespace std;
using namespace cv;
using namespace aruco;

namespace calibcamodo {

Dataset::Dataset():
    mpKfNow(nullptr), mpKfLast(nullptr), mbIfInitFilter(false) {

    mNumFrame = Config::NUM_FRAME;

    mstrFoldPathMain = Config::STR_FOLDERPATH_MAIN;
    mstrFoldPathImg = Config::STR_FOlDERPATH_IMG;
    mstrFilePathOdo = Config::STR_FILEPATH_ODO;
    mstrFilePathCam = Config::STR_FILEPATH_CAM;

    // select keyframe
    mThreshOdoLin   = Config::DATASET_THRESH_KF_ODOLIN;
    mThreshOdoRot   = Config::DATASET_THRESH_KF_ODOROT;

    // Camera parameters
    mSe3bc = Se3(Config::RVEC_BC, Config::TVEC_BC);
    mCamMatrix = Config::CAMERA_MATRIX.clone();
    mDistCoeff = Config::DISTORTION_COEFFICIENTS.clone();
}

PtrFrame Dataset::GetFrame(int _id) const {
    PtrFrame pRet = nullptr;
    if(mmapId2pFrame.count(_id))
        pRet = mmapId2pFrame.at(_id);
    return pRet;
}

bool Dataset::AddFrame(PtrFrame _ptr) {
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

bool Dataset::AddKf(PtrKeyFrame _ptr) {
    if (msetpKf.count(_ptr))
        return false;

    int id = _ptr->GetId();
    if (mmapId2pKf.count(id))
        return false;

    msetpKf.insert(_ptr);
    mmapId2pKf[id] = _ptr;
    return true;
}

PtrMapMark Dataset::GetMk(int _id) const {
    PtrMapMark pRet = nullptr;
    if(mmapId2pMk.count(_id))
        pRet = mmapId2pMk.at(_id);
    return pRet;
}

bool Dataset::AddMk(PtrMapMark _ptr) {
    if (msetpMk.count(_ptr))
        return false;
    int id = _ptr->GetId();
    if (mmapId2pMk.count(id))
        return false;
    msetpMk.insert(_ptr);
    mmapId2pMk[id] = _ptr;
    return true;
}


PtrMapPoint Dataset::GetMp(int _id) const {
    PtrMapPoint pRet = nullptr;
    if(mmapId2pMp.count(_id))
        pRet = mmapId2pMp.at(_id);
    return pRet;
}

bool Dataset::AddMp(PtrMapPoint _ptr) {
    if (msetpMp.count(_ptr))
        return false;
    int id = _ptr->GetId();
    if (mmapId2pMp.count(id))
        return false;
    msetpMp.insert(_ptr);
    mmapId2pMp[id] = _ptr;
    return true;
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
            AddFrame(pf);
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

//! functions for filter, return false if failed
bool Dataset::InitKfForFilter() {
    if(mmapId2pKf.size() < 2)
        return false;

    auto iter1 = mmapId2pKf.cbegin();
    auto iter2 = iter1; ++iter2;

    mpKfLast = iter1->second;
    mpKfNow = iter2->second;

    mbIfInitFilter = true;
    return true;
}

bool Dataset::RenewKfForFilter() {
    int idNow = mpKfNow->GetId();

    auto iterNow = mmapId2pKf.find(idNow);
    auto iterLast = iterNow++;

    if(iterNow == mmapId2pKf.cend() || iterLast == mmapId2pKf.cend())
        return false;
    else {
        mpKfNow = iterNow->second;
        mpKfLast = iterLast->second;
        return true;
    }
}

}





namespace calibcamodo {

//! Class DatasetAruco

DatasetAruco::DatasetAruco():
    Dataset() {

    mMarkerSize = Config::MARK_SIZE;
    // load camera intrinsics
    //    mCamParam.readFromXMLFile(mstrFilePathCam);
    mCamParam.CameraMatrix = Config::CAMERA_MATRIX.clone();
    mCamParam.Distorsion = Config::DISTORTION_COEFFICIENTS.clone();
    mCamParam.CamSize.width = Config::IMAGE_WIDTH;
    mCamParam.CamSize.height = Config::IMAGE_HEIGHT;

    // set aruco mark detector
    int ThePyrDownLevel = 0;
    int ThresParam1 = 19;
    int ThresParam2 = 15;
    mMDetector.pyrDown(ThePyrDownLevel);
    mMDetector.setCornerRefinementMethod(MarkerDetector::LINES);
    mMDetector.setThresholdParams(ThresParam1, ThresParam2);
}

bool DatasetAruco::AddKfAruco(PtrKeyFrameAruco _pKfAruco) {
    if(!AddKf(_pKfAruco))
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

bool DatasetAruco::AddMkAruco(PtrMapMarkAruco _pMkAruco) {
    if(!AddMk(_pMkAruco))
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

PtrMapMarkAruco DatasetAruco::GetMkAruco(int _id) const {
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
        AddKfAruco(pKfAruco);
    }
}

}


namespace calibcamodo {
//! DatasetOrb

DatasetOrb::DatasetOrb(): Dataset() {

    cerr << "DatasetOrb: init ORB extractor ..." << endl;
    mOrbExtractor = ORBextractor(400, 1.2, 5);
    if(!mOrbVocalubary.loadFromTextFile(Config::STR_FILEPATH_ORBVOC)) {
        cerr << "Wrong path to vocabulary. Path must be absolut or relative to ORB_SLAM package directory." << endl;
        cerr << "Falied to open at: " << Config::STR_FILEPATH_ORBVOC << endl;
    }

    cerr << "DatasetOrb: load ORB vocalubary ..." << endl;
    if(!mOrbVocalubary.loadFromTextFile(Config::STR_FILEPATH_ORBVOC)) {
        cerr << "Wrong path to vocabulary. Path must be absolut or relative to ORB_SLAM package directory." << endl;
        cerr << "Falied to open at: " << Config::STR_FILEPATH_ORBVOC << endl;
    }
}

void DatasetOrb::CreateKeyFrames() {
    set<PtrFrame> setpFrameSelected = SelectFrame();
    for(auto ptr : setpFrameSelected) {
        PtrFrame pframe = ptr;
        int id = pframe->GetId();
        Mat img;
        LoadImage(id, img);
        pframe->SetImg(img);
        PtrKeyFrameOrb pKfOrb = make_shared<KeyFrameOrb>(*pframe);
        pKfOrb->ComputeOrb(mOrbExtractor, mCamMatrix, mDistCoeff);
        pKfOrb->ComputeBoW(mOrbVocalubary);
        AddKfOrb(pKfOrb);
    }
}

bool DatasetOrb::AddKfOrb(PtrKeyFrameOrb _pKfOrb) {
    if(!AddKf(_pKfOrb))
        return false;
    if (msetpKfOrb.count(_pKfOrb))
        return false;
    int id = _pKfOrb->GetId();
    if (mmapId2pKfOrb.count(id))
        return false;
    msetpKfOrb.insert(_pKfOrb);
    mmapId2pKfOrb[id] = _pKfOrb;
    return true;
}

PtrKeyFrameOrb DatasetOrb::GetKfOrb(int _id) const {
    if(mmapId2pKfOrb.count(_id))
        return mmapId2pKfOrb.at(_id);
    else
        return nullptr;
}


bool DatasetOrb::AddMpOrb(PtrMapPointOrb _pMpOrb) {
    if(!AddMp(_pMpOrb))
        return false;
    if (msetpMpOrb.count(_pMpOrb))
        return false;
    int id = _pMpOrb->GetId();
    if (mmapId2pMpOrb.count(id))
        return false;
    msetpMpOrb.insert(_pMpOrb);
    mmapId2pMpOrb[id] = _pMpOrb;
    return true;
}

PtrMapPointOrb DatasetOrb::GetMpOrb(int _id) const {
    if(mmapId2pMpOrb.count(_id))
        return mmapId2pMpOrb.at(_id);
    else
        return nullptr;
}

}







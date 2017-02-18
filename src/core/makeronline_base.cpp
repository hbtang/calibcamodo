#include "makeronline_base.h"
#include "config.h"

using namespace cv;
using namespace std;

namespace calibcamodo {

MakerOnlineBase::MakerOnlineBase(Dataset* _pDataset):
    mpDataset(_pDataset) {

    // load odometry error configure
    mOdoLinErrR     = Config::CALIB_ODOLIN_ERRR;
    mOdoLinErrMin   = Config::CALIB_ODOLIN_ERRMIN;
    mOdoRotErrR     = Config::CALIB_ODOLIN_ERRR;
    mOdoRotErrRLin  = Config::CALIB_ODOROT_ERRRLIN;
    mOdoRotErrMin   = Config::CALIB_ODOROT_ERRMIN;
}

bool MakerOnlineBase::RenewKfNow() {
    if(!mpDataset->mbIfInitFilter) {
        if(!mpDataset->InitKfForFilter())
            return false;

        // init pose and cov of kflast
        InitKfLastPoseCov();
    }
    else {
        if(!mpDataset->RenewKfForFilter())
            return false;
    }

    // init pose of kfnow
    InitKfNowPose();

    // to make msr.odo between kflast and kfnow
    MakeMsrOdoNow();

    return true;
}


void MakerOnlineBase::InitKfNowPose() {
    PtrKeyFrame pKfNow = mpDataset->GetKfNow();
    PtrKeyFrame pKfLast = mpDataset->GetKfLast();
    Se2 se2wbLast = pKfLast->GetPoseBase();
    Se2 se2odo = pKfNow->GetOdo() - pKfLast->GetOdo();
    Se2 se2wbNow = se2wbLast + se2odo;
    Se3 se3bc = mpDataset->GetCamOffset();
    pKfNow->SetPoseAllbyB(se2wbNow, se3bc);

    // debug
//    cerr << "se3bc" << se3bc << endl;
//    cerr << "se2wbNow" << se2wbNow << endl;
    return;
}

void MakerOnlineBase::InitKfLastPoseCov() {
    PtrKeyFrame pKfLast = mpDataset->GetKfLast();
    Se2 se2odo = pKfLast->GetOdo();
    Se3 se3bc = mpDataset->GetCamOffset();
    pKfLast->SetPoseAllbyB(se2odo, se3bc);
    Mat matCov = Mat::zeros(3,3,CV_32FC1);
    pKfLast->SetCov(matCov);

    // debug
//    cerr << "se3bc" << se3bc << endl;
//    cerr << "se2odo" << se2odo << endl;
    return;
}

void MakerOnlineBase::MakeMsrOdoNow() {

    PtrKeyFrame pKfLast = mpDataset->GetKfLast();
    PtrKeyFrame pKfNow = mpDataset->GetKfNow();

    Se2 dodo = pKfNow->GetOdo() - pKfLast->GetOdo();

    Mat info = Mat::eye(3,3,CV_32FC1);
    double dist = dodo.dist();
    double stdlin = max(dist*mOdoLinErrR, mOdoLinErrMin);
    double theta = dodo.theta;
    double stdrot = max(max(abs(theta)*mOdoRotErrR, mOdoRotErrMin), dist*mOdoRotErrRLin);
    info.at<float>(0,0) = 1/stdlin/stdlin;
    info.at<float>(1,1) = 1/stdlin/stdlin;
    info.at<float>(2,2) = 1/stdrot/stdrot;

    PtrMsrSe2Kf2Kf pMeasureOdo =
            make_shared<MeasureSe2Kf2Kf>(dodo, info, pKfLast, pKfNow);
    mpDataset->AddMsrOdo(pMeasureOdo);

}

}



#include "maker_aruco.h"
#include "config.h"

using namespace cv;
using namespace std;

namespace calibcamodo {

MakerAruco::MakerAruco(DatasetAruco* _pDatasetAruco):
    MakerBase(_pDatasetAruco), mpDatasetAruco(_pDatasetAruco) {

    // set aruco error configure
    mAmkZErrRZ      = Config::CALIB_AMKZ_ERRRZ;
    mAmkZErrMin     = Config::CALIB_AMKZ_ERRMIN;
    mAmkXYErrRZ     = Config::CALIB_AMKXY_ERRRZ;
    mAmkXYErrMin    = Config::CALIB_AMKXY_ERRMIN;
}

void MakerAruco::MakeMkAndMsrMk() {
    const set<PtrKeyFrameAruco> setpKfAruco = mpDatasetAruco->GetKfArucoSet();

    // Create aruco marks and mark measurements
    for (auto ptr : setpKfAruco) {
        PtrKeyFrameAruco pKfAruco = ptr;
        const std::vector<aruco::Marker>& vecAruco = pKfAruco->GetMsrAruco();
        for (auto measure_aruco : vecAruco) {
            int id = measure_aruco.id;
            Mat tvec = measure_aruco.Tvec;
            double marksize = measure_aruco.ssize;

            double z = abs(tvec.at<float>(2));
            double stdxy = max(z*mAmkXYErrRZ, mAmkXYErrMin);
            double stdz = max(z*mAmkZErrRZ, mAmkZErrMin);

            Mat info = Mat::eye(3,3,CV_32FC1);
            info.at<float>(0,0) = 1/stdxy/stdxy;
            info.at<float>(1,1) = 1/stdxy/stdxy;
            info.at<float>(2,2) = 1/stdz/stdz;

            // add new aruco mark into dataset
            PtrMapMarkAruco pMkAruco = make_shared<MapMarkAruco>(id, id, marksize);
            if (!mpDatasetAruco->AddMkAruco(pMkAruco))
                pMkAruco = mpDatasetAruco->GetMkAruco(id);

            // add new measurement into dataset
            PtrMsrPt3Kf2Mk pMsrMk = make_shared<MeasurePt3Kf2Mk>(tvec, info, pKfAruco, pMkAruco);
            mpDatasetAruco->AddMsrMk(pMsrMk);
        }
    }
}

void MakerAruco::InitMkPose() {
    for(auto ptr : mpDatasetAruco->GetMkSet()) {
        PtrMapMark pMk = ptr;
        set<PtrMsrPt3Kf2Mk> setpMsr = mpDatasetAruco->GetMsrMkByMk(pMk);
        if(!setpMsr.empty()) {
            PtrKeyFrame pKf = (*setpMsr.cbegin())->pKf;
            Se3 se3wc = pKf->GetPoseCamera();
            Se3 se3cm;
            se3cm.tvec = (*setpMsr.cbegin())->pt3.tvec();
            Se3 se3wm = se3wc + se3cm;
            pMk->SetPose(se3wm);
        }
    }
}

void MakerAruco::InitKfMkPose() {
    InitKfPose();
    InitMkPose();
}



}

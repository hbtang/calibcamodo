#include "solveronline_ekf.h"

using namespace std;
using namespace cv;

namespace calibcamodo {


SolverOnlineEkf::SolverOnlineEkf(Dataset* _pDataset):
    SolverOnlineBase(_pDataset) {}


void SolverOnlineEkf::DoFilterOnce() {

    // Read info
    PtrKeyFrame pKfNow = mpDataset->GetKfNow();
    PtrKeyFrame pKfLast = mpDataset->GetKfLast();
    std::set<PtrMsrUVKf2Mp> setpMsrMp = mpDataset->GetMsrMpByKf(pKfNow);
    PtrMsrSe2Kf2Kf pMsrOdo = mpDataset->GetMsrOdoByKfTail(pKfNow);

    assert(pMsrOdo->pKfHead == pKfLast && pMsrOdo->pKfTail == pKfNow);


    // Propagate
    Propagate(pMsrOdo);


    // Update




    return;

}

void SolverOnlineEkf::Propagate(PtrMsrSe2Kf2Kf _pMsrOdo) const {

    PtrKeyFrame pKf1 = _pMsrOdo->pKfHead;   // last keyframe
    PtrKeyFrame pKf2 = _pMsrOdo->pKfTail;   // current keyframe, should be refreshed

    // Propagate
    Se2 se2_wb1 = pKf1->GetPoseBase();
    Mat Cov_wb1 = pKf1->GetCov().clone();
    Se2 se2_odo = _pMsrOdo->se2;
    Mat Cov_odo = _pMsrOdo->info.clone().inv();

    // Propagate the mean of new pose
    Se2 se2_wb2 = se2_wb1 + se2_odo;
    pKf2->SetPoseAllbyB(se2_wb2, mpDataset->GetCamOffset());

    // Propagate the covariance matrix of new pose
    Mat J_wb2_odo = Mat::eye(3,3,CV_32FC1);
    Mat R_wb1 = se2_wb1.R();
    R_wb1.copyTo(J_wb2_odo.rowRange(0,2).colRange(0,2));

    Mat J_wb2_wb1 = Mat::eye(3,3,CV_32FC1);
    Mat J_wb2_wb1odo = Mat::zeros(3,6,CV_32FC1);
    J_wb2_wb1.copyTo(J_wb2_wb1odo.colRange(0,3));
    J_wb2_odo.copyTo(J_wb2_wb1odo.colRange(3,6));

    Mat Cov_wb1odo = Mat::zeros(6,6,CV_32FC1);
    Cov_wb1.copyTo(Cov_wb1odo.colRange(0,3).rowRange(0,3));
    Cov_odo.copyTo(Cov_wb1odo.colRange(3,6).rowRange(3,6));

    Mat Cov_wb2 = J_wb2_wb1odo * Cov_wb1odo * J_wb2_wb1odo.t();
    pKf2->SetCov(Cov_wb2);

    // debug
    cerr << "SolverOnlineEkf:"
         << " idNow = " << pKf2->GetId()
         << " idLast = " << pKf1->GetId()
         << endl;
    cerr << "Cov_wb2" << endl << Cov_wb2 << endl;
    cerr << "Cov_wb1" << endl << Cov_wb1 << endl;
    cerr << "Cov_odo" << endl << Cov_odo << endl;
    cerr << "Cov_wb1odo" << endl << Cov_wb1odo << endl;
    cerr << "J_wb2_wb1odo" << endl << J_wb2_wb1odo << endl;

    return;
}

void SolverOnlineEkf::Correct(std::set<PtrMsrUVKf2Mp> _setpMsrMp) const {

}

}



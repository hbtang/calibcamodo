#include "solver_optmk.h"
#include "g2o/g2o_api.h"
#include "adapter.h"

using namespace cv;
using namespace std;
using namespace g2o;

namespace calibcamodo {

SolverOptMk::SolverOptMk(Dataset* _pDataset):
    SolverBase(_pDataset) {

}

void SolverOptMk::DoCalib() {
    //! Set optimizer
    SparseOptimizer optimizer;
    optimizer.setVerbose(true);
    InitOptimizerCalib(optimizer);

    //! Set extrinsic vertex
    int idVertexMax = 0;
    Isometry3D Iso3_bc = toG2oIsometry3D(mpDataset->GetCamOffset());
    AddVertexSE3(optimizer, Iso3_bc, idVertexMax++);

    //! Set keyframe vertices
    map<PtrKeyFrame,int> mappKf2IdOpt;
    for (auto ptr : mpDataset->GetKfSet()) {
        PtrKeyFrame pKf = ptr;
        AddVertexSE2(optimizer, toG2oSE2(pKf->GetPoseBase()), idVertexMax);
        mappKf2IdOpt[pKf] = idVertexMax++;
    }

    //! Set mark vertices
    map<PtrMapMark,int> mappMk2IdOpt;
    for (auto ptr : mpDataset->GetMkSet()) {
        PtrMapMark pMk = ptr;
        //! NEED TO ADD INIT MK POSE HERE !!!
        g2o::Vector3D pose = toG2oVector3D(pMk->GetPose().tvec);

        AddVertexPointXYZ(optimizer, pose, idVertexMax);
        mappMk2IdOpt[pMk] = idVertexMax++;
        // DEBUG
        //        cerr << "mkId: " << pMk->GetId() << endl;
        //        cerr << "mkTvec: " << pMk->GetPose().tvec << endl;
        //        cerr << "pose: " << pose << endl;
    }

    //! Set odometry edges
    for (auto ptr : mpDataset->GetMsrOdoSet()) {
        PtrMsrSe2Kf2Kf pMsrOdo = ptr;
        PtrKeyFrame pKf0 = pMsrOdo->pKfHead;
        PtrKeyFrame pKf1 = pMsrOdo->pKfTail;
        int id0 = mappKf2IdOpt[pKf0];
        int id1 = mappKf2IdOpt[pKf1];
        g2o::SE2 measure = toG2oSE2(pMsrOdo->se2);
        g2o::Matrix3D info = toEigenMatrixXd(pMsrOdo->info);
        AddEdgeSE2(optimizer, id0, id1, measure, info);

        // DEBUG
        //        cerr << info << endl;
        //        cerr << pMsrOdo->info << endl;
    }

    //! Set mark measurement edges
    for (auto ptr : mpDataset->GetMsrMkAll()) {
        PtrMsrPt3Kf2Mk pMsrMk = ptr;
        PtrKeyFrame pKf = pMsrMk->pKf;
        PtrMapMark pMk = pMsrMk->pMk;

        int idKf = mappKf2IdOpt[pKf];
        int idMk = mappMk2IdOpt[pMk];

        g2o::Vector3D measure = toG2oVector3D(pMsrMk->measure);
        g2o::Matrix3D info = toEigenMatrixXd(pMsrMk->info);

        AddEdgeOptMk(optimizer, idKf, idMk, 0, measure, info);

        // DEBUG
        //        cerr << info << endl;
        //        cerr << pMsrMk->measure << endl;
        //        cerr << measure << endl;
        //        cerr << pMsrMk->info << endl;
    }

    //! Do optimize
    optimizer.initializeOptimization();
    optimizer.optimize(100);

    //! Refresh calibration results
    g2o::VertexSE3* v = static_cast<g2o::VertexSE3*>(optimizer.vertex(0));
    Isometry3D Iso3_bc_opt = v->estimate();

    Se3 se3bc = toSe3(Iso3_bc_opt);
    mpDataset->SetCamOffset(se3bc);

    //! Refresh keyframe
    for (auto pair : mappKf2IdOpt) {
        PtrKeyFrame pKf = pair.first;
        int idOpt = pair.second;
        VertexSE2* pVertex = static_cast<VertexSE2*>(optimizer.vertex(idOpt));
        pKf->SetPoseAllbyB(toSe2(pVertex->estimate()), se3bc);
    }

    //! Refresh landmark
    for (auto pair : mappMk2IdOpt) {
        PtrMapMark pMk = pair.first;
        int idOpt = pair.second;
        VertexPointXYZ* pVertex = static_cast<VertexPointXYZ*>(optimizer.vertex(idOpt));
        Mat tvec_wm = toCvMatf(pVertex->estimate());
        pMk->SetPoseTvec(tvec_wm);

        // DEBUG:
        //        cerr << "tvec_wm: " << tvec_wm.t() << endl;
    }
}

}

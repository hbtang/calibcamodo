#include "solver_vsclam.h"
#include "adapter.h"

using namespace cv;
using namespace std;
using namespace g2o;

namespace calibcamodo {

SolverVsclam::SolverVsclam(Dataset* _pDataset):
    SolverBase(_pDataset) {}

void SolverVsclam::DoCalib() {
    OptimizeSlam();
    OptimizeSclam();
}

void SolverVsclam::OptimizeSlam() {

    // Init optimizer
    SparseOptimizer optimizer;
    bool bOptVerbose = true;
    InitOptimizerSlam(optimizer, bOptVerbose);

    // Add Parameters
    Se3 se3bc = mpDataset->GetCamOffset();
    int idParamCamera = 0;
    AddParaCamera(optimizer, mpDataset->GetCamMat(), toG2oIsometry3D(se3bc), idParamCamera);

    int idVertexMax = 0;
    // Add keyframe vertices
    map<PtrKeyFrame, int> mapKf2IdOpt;
    for (auto ptr : mpDataset->GetKfSet()) {
        PtrKeyFrame pKf = ptr;
        SE2 pose = toG2oSE2(pKf->GetPoseBase());
        AddVertexSE2(optimizer, pose, idVertexMax);
        mapKf2IdOpt[pKf] = idVertexMax++;
    }

    // Add mappoint vertices
    map<PtrMapPoint,int> mapMp2IdOpt;
    for (auto ptr : mpDataset->GetMpSet()) {
        PtrMapPoint pMp = ptr;
        Vector3D pose = toG2oVector3D(pMp->GetPos().tvec());
        AddVertexPointXYZ(optimizer, pose, idVertexMax);
        mapMp2IdOpt[pMp] = idVertexMax++;
    }

    // Add odometry edges
    vector<g2o::EdgeSE2*> vecpEdgeOdo;
    for (auto ptr : mpDataset->GetMsrOdoSet()) {
        PtrMsrSe2Kf2Kf pMsrOdo = ptr;
        PtrKeyFrame pKf0 = pMsrOdo->pKfHead;
        PtrKeyFrame pKf1 = pMsrOdo->pKfTail;
        int id0 = mapKf2IdOpt[pKf0];
        int id1 = mapKf2IdOpt[pKf1];
        g2o::SE2 measure = toG2oSE2(pMsrOdo->se2);
        g2o::Matrix3D info = toEigenMatrixXd(pMsrOdo->info);
        g2o::EdgeSE2* pEdgeOdo = AddEdgeSE2(optimizer, id0, id1, measure, info);
        vecpEdgeOdo.push_back(pEdgeOdo);
    }

    // Set mark measurement edges
    vector<g2o::EdgeVSlam*> vecpEdgeVSlam;
    for (auto ptr : mpDataset->GetMsrMpAll()) {
        PtrMsrUVKf2Mp pMsrMp = ptr;
        PtrKeyFrame pKf = pMsrMp->pKf;
        PtrMapPoint pMp = pMsrMp->pMp;
        int idKf = mapKf2IdOpt[pKf];
        int idMp = mapMp2IdOpt[pMp];
        g2o::Vector2D measure = toG2oVector2D(pMsrMp->measure);
        g2o::Matrix2D info = toEigenMatrixXd(pMsrMp->info);
        g2o::EdgeVSlam* pEdgeVSlam = AddEdgeVSlam(optimizer, idKf, idMp, idParamCamera, measure, info);
        vecpEdgeVSlam.push_back(pEdgeVSlam);
    }

    // Debug: show edge info...
    //    PrintEdgeInfoOdo(vecpEdgeOdo);
    //    PrintEdgeInfoVSlam(vecpEdgeVSlam);

    // Do optimize
    optimizer.initializeOptimization();
    optimizer.optimize(30);

    // Debug: show edge info...
    //    PrintEdgeInfoOdo(vecpEdgeOdo);
    //    PrintEdgeInfoVSlam(vecpEdgeVSlam);

    // Refresh all keyframes
    for (auto pair : mapKf2IdOpt) {
        PtrKeyFrame pKf = pair.first;
        int idOpt = pair.second;
        VertexSE2* pVertex = static_cast<VertexSE2*>(optimizer.vertex(idOpt));
        pKf->SetPoseAllbyB(toSe2(pVertex->estimate()), se3bc);
    }

    // Refresh all mappoints
    for (auto pair : mapMp2IdOpt) {
        PtrMapPoint pMp = pair.first;
        int idOpt = pair.second;
        VertexPointXYZ* pVertex = static_cast<VertexPointXYZ*>(optimizer.vertex(idOpt));
        Mat tvec_wm = toCvMatf(pVertex->estimate());
        pMp->SetPos(Pt3(tvec_wm));
    }
}

void SolverVsclam::OptimizeSclam() {
    // Init optimizer
    SparseOptimizer optimizer;
    bool bOptVerbose = true;
    InitOptimizerCalib(optimizer);
    optimizer.setVerbose(bOptVerbose);

    // Add Parameters
    int idParamCamera = 0;
    AddCamPara(optimizer, mpDataset->GetCamMat(), idParamCamera);

    int idVertexMax = 0;
    // Add camera extrinsic vertex
    Se3 se3cb = mpDataset->GetCamOffset();
    int idVertexCamOffset = idVertexMax++;
    AddVertexSE3(optimizer, toG2oIsometry3D(se3cb), idVertexCamOffset, false);

    // Add keyframe vertices
    map<PtrKeyFrame, int> mapKf2IdOpt;
    for (auto ptr : mpDataset->GetKfSet()) {
        PtrKeyFrame pKf = ptr;
        SE2 pose = toG2oSE2(pKf->GetPoseBase());
        AddVertexSE2(optimizer, pose, idVertexMax);
        mapKf2IdOpt[pKf] = idVertexMax++;
    }

    // Add mappoint vertices
    map<PtrMapPoint,int> mapMp2IdOpt;
    for (auto ptr : mpDataset->GetMpSet()) {
        PtrMapPoint pMp = ptr;
        Vector3D pose = toG2oVector3D(pMp->GetPos().tvec());
        AddVertexPointXYZ(optimizer, pose, idVertexMax);
        mapMp2IdOpt[pMp] = idVertexMax++;
    }

    // Add odometry edges
    vector<g2o::EdgeSE2*> vecpEdgeOdo;
    for (auto ptr : mpDataset->GetMsrOdoSet()) {
        PtrMsrSe2Kf2Kf pMsrOdo = ptr;
        PtrKeyFrame pKf0 = pMsrOdo->pKfHead;
        PtrKeyFrame pKf1 = pMsrOdo->pKfTail;
        int id0 = mapKf2IdOpt[pKf0];
        int id1 = mapKf2IdOpt[pKf1];
        g2o::SE2 measure = toG2oSE2(pMsrOdo->se2);
        g2o::Matrix3D info = toEigenMatrixXd(pMsrOdo->info);
        g2o::EdgeSE2* pEdgeOdo = AddEdgeSE2(optimizer, id0, id1, measure, info);
        vecpEdgeOdo.push_back(pEdgeOdo);
    }

    // Set mark measurement edges
    vector<g2o::EdgeVSclam*> vecpEdgeVSclam;
    for (auto ptr : mpDataset->GetMsrMpAll()) {
        PtrMsrUVKf2Mp pMsrMp = ptr;
        PtrKeyFrame pKf = pMsrMp->pKf;
        PtrMapPoint pMp = pMsrMp->pMp;
        int idKf = mapKf2IdOpt[pKf];
        int idMp = mapMp2IdOpt[pMp];
        g2o::Vector2D measure = toG2oVector2D(pMsrMp->measure);
        g2o::Matrix2D info = toEigenMatrixXd(pMsrMp->info);
        g2o::EdgeVSclam* pEdgeVSclam = AddEdgeVSclam(optimizer, idKf, idMp, idVertexCamOffset, idParamCamera, measure, info);
        vecpEdgeVSclam.push_back(pEdgeVSclam);
    }

    // Debug: show edge info...
    //    PrintEdgeInfoOdo(vecpEdgeOdo);
    //    PrintEdgeInfoVSlam(vecpEdgeVSlam);

    // Do optimize
    optimizer.initializeOptimization();
    optimizer.optimize(30);

    // Debug: show edge info...
    //    PrintEdgeInfoOdo(vecpEdgeOdo);
    //    PrintEdgeInfoVSlam(vecpEdgeVSlam);

    // Renew camera offset
    VertexSE3* pVertexCamOffset = static_cast<VertexSE3*>(optimizer.vertex(idVertexCamOffset));
    mpDataset->SetCamOffset(toSe3(pVertexCamOffset->estimate()));

    // Refresh all keyframes
    for (auto pair : mapKf2IdOpt) {
        PtrKeyFrame pKf = pair.first;
        int idOpt = pair.second;
        VertexSE2* pVertex = static_cast<VertexSE2*>(optimizer.vertex(idOpt));
        pKf->SetPoseAllbyB(toSe2(pVertex->estimate()), se3cb);
    }

    // Refresh all mappoints
    for (auto pair : mapMp2IdOpt) {
        PtrMapPoint pMp = pair.first;
        int idOpt = pair.second;
        VertexPointXYZ* pVertex = static_cast<VertexPointXYZ*>(optimizer.vertex(idOpt));
        Mat tvec_wm = toCvMatf(pVertex->estimate());
        pMp->SetPos(Pt3(tvec_wm));
    }
}
}

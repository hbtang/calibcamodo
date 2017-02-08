#include "solver.h"
#include "adapter.h"
#include "mark.h"
#include "g2o/g2o_api.h"
#include "config.h"

namespace calibcamodo {

using namespace cv;
using namespace std;
using namespace g2o;

//!
//! \brief Solver::Solver
//! \param _pDataset
//!
Solver::Solver(Dataset *_pDataset): mpDataset(_pDataset) {

    // load odometry error configure
    mOdoLinErrR     = Config::CALIB_ODOLIN_ERRR;
    mOdoLinErrMin   = Config::CALIB_ODOLIN_ERRMIN;
    mOdoRotErrR     = Config::CALIB_ODOLIN_ERRR;
    mOdoRotErrRLin  = Config::CALIB_ODOROT_ERRRLIN;
    mOdoRotErrMin   = Config::CALIB_ODOROT_ERRMIN;

}

void Solver::CreateMsrOdos() {

    mpDataset->ClearMsrOdo();
    const std::map<int, PtrKeyFrame>& mmapId2pKf = mpDataset->GetKfMap();

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
        mpDataset->InsertMsrOdo(pMeasureOdo);
    }
}

void Solver::RefreshKfsPose() {
    for(auto ptr : mpDataset->GetKfSet()) {
        PtrKeyFrame pKf = ptr;
        Se2 se2odo = pKf->GetOdo();
        Se2 se2wb = se2odo;
        Se3 se3wb = Se3(se2wb);
        Se3 se3wc = se3wb + mSe3cb;

        pKf->SetPoseBase(se2odo);
        pKf->SetPoseCamera(se3wc);
    }
}


SolverAruco::SolverAruco(DatasetAruco* _pDatasetAruco):
    Solver(_pDatasetAruco) {
    mpDatasetAruco  = _pDatasetAruco;
    mAmkZErrRZ      = Config::CALIB_AMKZ_ERRRZ;
    mAmkZErrMin     = Config::CALIB_AMKZ_ERRMIN;
    mAmkXYErrRZ     = Config::CALIB_AMKXY_ERRRZ;
    mAmkXYErrMin    = Config::CALIB_AMKXY_ERRMIN;
}

void SolverAruco::CreateMarks() {

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
            PtrMarkAruco pMkAruco = make_shared<MarkAruco>(id, id, marksize);
            if (!mpDatasetAruco->InsertMkAruco(pMkAruco))
                pMkAruco = mpDatasetAruco->GetMkAruco(id);

            // add new measurement into dataset
            PtrMsrPt3Kf2Mk pMsrMk = make_shared<MeasurePt3Kf2Mk>(tvec, info, pKfAruco, pMkAruco);
            mpDatasetAruco->InsertMsrMk(pMsrMk);
        }
    }
}

void SolverAruco::RefreshMksPose() {
    for(auto ptr : mpDatasetAruco->GetMkSet()) {
        PtrMark pMk = ptr;
        set<PtrMsrPt3Kf2Mk> setpMsr = mpDatasetAruco->GetMsrMkbyMk(pMk);
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

void SolverAruco::RefreshAllPose() {
    RefreshKfsPose();
    RefreshMksPose();
}


SolverInitmk::SolverInitmk(DatasetAruco *_pDatasetAruco):
    SolverAruco(_pDatasetAruco) {

}


void SolverInitmk::DoCalib() {

    // calibrate the ground plane, return 3-by-1 norm vector in camera frame
    Mat nvec_cg;
    ComputeGrndPlane(nvec_cg);

    // compute camera projection frame, with 2 solutions on 2 direction of ground
    Mat rvec_dc_1, tvec_dc_1, rvec_dc_2, tvec_dc_2;
    ComputeCamProjFrame(nvec_cg, rvec_dc_1, tvec_dc_1);
    ComputeCamProjFrame(-nvec_cg, rvec_dc_2, tvec_dc_2);

    //    cerr << "nvec_cg" << nvec_cg << endl;
    //    cerr << "rvec_dc_1" << rvec_dc_1 << endl;
    //    cerr << "rvec_dc_2" << rvec_dc_2 << endl;

    // compute xyyaw between based frame and camera projection frame,
    // choose the solution with smaller residual
    Mat rvec_bd_1, tvec_bd_1, rvec_bd_2, tvec_bd_2;
    double norm_res_1 = Compute2DExtrinsic(rvec_dc_1, tvec_dc_1, rvec_bd_1, tvec_bd_1);
    double norm_res_2 = Compute2DExtrinsic(rvec_dc_2, tvec_dc_2, rvec_bd_2, tvec_bd_2);
    Mat T_dc, T_bd, T_bc;
    if (norm_res_1 < norm_res_2) {
        Vec2MatSe3(rvec_dc_1, tvec_dc_1, T_dc);
        Vec2MatSe3(rvec_bd_1, tvec_bd_1, T_bd);
    }
    else {
        Vec2MatSe3(rvec_dc_2, tvec_dc_2, T_dc);
        Vec2MatSe3(rvec_bd_2, tvec_bd_2, T_bd);
    }
    T_bc = T_bd*T_dc;
    mSe3cb = Se3(T_bc);
}

void SolverInitmk::ComputeGrndPlane(Mat &nvec_cg) {

    const set<PtrMsrPt3Kf2Mk> & setMsrMk = mpDataset->GetMsrMkSet();

    int numLclIdMk = 0;
    int numLclIdKf = 0;
    map<PtrMark, int> mapMk2LclId;
    map<PtrKeyFrame, int> mapKf2LclId;

    for (auto ptrmeasure : setMsrMk) {
        PtrKeyFrame pKf = ptrmeasure->pKf;
        PtrMark pMk = ptrmeasure->pMk;
        if(!mapMk2LclId.count(pMk))
            mapMk2LclId[pMk] = numLclIdMk++;
        if(!mapKf2LclId.count(pKf))
            mapKf2LclId[pKf] = numLclIdKf++;
    }

    const int dimrow = numLclIdKf;
    const int dimcol = 3+numLclIdMk;
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(dimrow, dimcol);

    for (auto ptr : setMsrMk) {
        PtrMsrPt3Kf2Mk pMsrMk = ptr;
        PtrKeyFrame pKf = pMsrMk->pKf;
        PtrMark pMk = pMsrMk->pMk;

        int lclIdMk = mapMk2LclId[pMk];
        int lclIdKf = mapKf2LclId[pKf];

        Mat tvec = pMsrMk->pt3.tvec();
        A(lclIdKf,0) = tvec.at<float>(0);
        A(lclIdKf,1) = tvec.at<float>(1);
        A(lclIdKf,2) = tvec.at<float>(2);
        A(lclIdKf,3+lclIdMk) = 1;
    }

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);

    Eigen::VectorXd singular = svd.singularValues();
    Eigen::MatrixXd V = svd.matrixV();
    Eigen::VectorXd norm = singular;
    Eigen::VectorXd singularnorm = singular;
    Eigen::VectorXd vbest;
    double singularnormbest = INFINITY;

    for (int i = 0; i < V.rows(); i++) {
        Eigen::Vector3d vecn = V.block(0,i,3,1);
        norm(i) = sqrt(vecn(0)*vecn(0)+vecn(1)*vecn(1)+vecn(2)*vecn(2));
        singularnorm(i) = singular(i)/norm(i);
        if (singularnorm(i) < singularnormbest) {
            singularnormbest = singularnorm(i);
            vbest = V.block(0,i,3,1)/norm(i);
        }
    }

    Mat nvec = Mat::zeros(3,1,CV_32FC1);
    nvec.at<float>(0) = vbest(0);
    nvec.at<float>(1) = vbest(1);
    nvec.at<float>(2) = vbest(2);

    //    cerr << "A:" << endl << A << endl;
    //    cerr << "singular:" << endl << singular << endl;
    //    cerr << "V:" << endl << V << endl;
    //    cerr << "norm:" << endl << norm << endl;
    //    cerr << "singularnorm:" << endl << singularnorm << endl;
    //    cerr << "vbest:" << endl << vbest << endl;

    nvec.copyTo(nvec_cg);
}

void SolverInitmk::ComputeCamProjFrame(const Mat &nvec_cg, Mat &rvec_dc, Mat &tvec_dc) {

    // define an approximate norm vector "nvecApprox" with a large angle with ground norm
    Mat nvecApprox;
    Mat rz = nvec_cg;
    float rz0 = rz.at<float>(0);
    float rz1 = rz.at<float>(1);
    float rz2 = rz.at<float>(2);
    if (abs(rz0) < abs(rz1) && abs(rz0) < abs(rz2)) {
        nvecApprox = (Mat_<float>(3,1) << 1, 0, 0);
    }
    else if (abs(rz1) < abs(rz2)) {
        nvecApprox = (Mat_<float>(3,1) << 0, 1, 0);
    }
    else {
        nvecApprox = (Mat_<float>(3,1) << 0, 0, 1);
    }

    // create the roation matrix
    Mat rx = rz.cross(nvecApprox);
    rx = rx/norm(rx);
    Mat ry = rz.cross(rx);

    Mat Rcd = Mat::zeros(3,3,CV_32FC1);
    rx.copyTo(Rcd.colRange(0,1));
    ry.copyTo(Rcd.colRange(1,2));
    rz.copyTo(Rcd.colRange(2,3));
    Rodrigues(Rcd.t(), rvec_dc);

    tvec_dc = Mat::zeros(3,1,CV_32FC1);

    //    cerr << "nvec_cg" << endl << nvec_cg << endl;
    //    cerr << "Rdc" << endl << Rcd.t() << endl;
    //    cerr << "rvec_dc" << rvec_dc << endl;
    //    cerr << "tvec_dc" << tvec_dc << endl;
}


double SolverInitmk::Compute2DExtrinsic(const Mat &rvec_dc, const Mat &tvec_dc, Mat &rvec_bd, Mat &tvec_bd) {

    const set<PtrMsrSe2Kf2Kf>& setMsrOdo = mpDataset->GetMsrOdoSet();
    const set<PtrMsrPt3Kf2Mk>& setMsrMk = mpDataset->GetMsrMkSet();

    double threshSmallRotation = 1.0/5000;

    vector<HyperEdgeOdoMk> vecHyperEdge;
    vector<HyperEdgeOdoMk> vecHyperEdgeSmallRot;
    vector<HyperEdgeOdoMk> vecHyperEdgeLargeRot;

    for(auto ptrmsrodo : setMsrOdo) {
        PtrMsrSe2Kf2Kf pMsrOdo = ptrmsrodo;
        double odo_ratio = pMsrOdo->se2.ratio();

        PtrKeyFrame pKf1 = pMsrOdo->pKfHead;
        PtrKeyFrame pKf2 = pMsrOdo->pKfTail;

        set<pair<PtrMsrPt3Kf2Mk, PtrMsrPt3Kf2Mk>> setpairMsrMk;
        FindCovisMark(pKf1, pKf2, setpairMsrMk);

        for (auto pairMsrMk : setpairMsrMk) {
            HyperEdgeOdoMk edge(pMsrOdo, pairMsrMk.first, pairMsrMk.second);
            vecHyperEdge.push_back(edge);
            if (abs(odo_ratio) < threshSmallRotation) {
                vecHyperEdgeSmallRot.push_back(edge);
            }
            else {
                vecHyperEdgeLargeRot.push_back(edge);
            }
        }
    }

    //    cerr << "Number of hyper edges: " << vecHyperEdge.size() << endl;
    //    cerr << "Number of hyper edges with small rotation: " << vecHyperEdgeSmallRot.size() << endl;
    //    cerr << "Number of hyper edges with large rotation: " << vecHyperEdgeLargeRot.size() << endl;

    // COMPUTE YAW ANGLE
    Mat R_dc;
    Rodrigues(rvec_dc, R_dc);

    double yawsum = 0;
    int yawcount = 0;
    for(auto edge : vecHyperEdgeSmallRot) {
        HyperEdgeOdoMk hyperEdge = edge;
        PtrMsrSe2Kf2Kf pMsrOdo = hyperEdge.pMsrOdo;
        PtrMsrPt3Kf2Mk pMsrMk1 = hyperEdge.pMsrMk1;
        PtrMsrPt3Kf2Mk pMsrMk2 = hyperEdge.pMsrMk2;

        Se3 se3_b1b2 = Se3(pMsrOdo->se2);

        Mat R_b1b2 = se3_b1b2.R();
        Mat tvec_b1b2 = se3_b1b2.tvec;

        Mat tvec_c1m = pMsrMk1->pt3.tvec();
        Mat tvec_c2m = pMsrMk2->pt3.tvec();
        Mat tvec_b1b2_bar = R_dc*tvec_c1m - R_b1b2*R_dc*tvec_c2m;

        double xb = tvec_b1b2.at<float>(0);
        double yb = tvec_b1b2.at<float>(1);
        double xbbar = tvec_b1b2_bar.at<float>(0);
        double ybbar = tvec_b1b2_bar.at<float>(1);
        double yaw = atan2(yb,xb) - atan2(ybbar,xbbar);
        yaw = Period(yaw, PI, -PI);

        yawsum += yaw;
        yawcount++;

        // DEBUG:
        //        cerr << "R_dc" << endl << R_dc << endl;
        //        cerr << "tvec_c1m" << endl << tvec_c1m.t() << endl;
        //        cerr << "tvec_c2m" << endl << tvec_c2m.t() << endl;
        //        cerr << "tvec_b1b2" << endl << tvec_b1b2.t() << endl;
        //        cerr << "tvec_b1b2_bar" << endl << tvec_b1b2_bar.t() << endl;
        //        cerr << xb << " " << yb << " " << xbbar << " " << ybbar << " " << yaw << endl;
        //        cerr << endl;
    }
    double yawavr = yawsum/yawcount;
    //    cerr << "Yaw: " << yawavr << endl;
    rvec_bd = ( Mat_<float>(3,1) << 0, 0, yawavr);

    // COMPUTE XY TRANSLATION
    Mat R_bd;
    Rodrigues(rvec_bd, R_bd);
    Mat R_bc = R_bd * R_dc;

    const int numHyperEdge = vecHyperEdgeLargeRot.size();
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(numHyperEdge*2, 2);
    Eigen::MatrixXd b = Eigen::MatrixXd::Zero(numHyperEdge*2, 1);

    int countEdge = 0;
    for(auto edge : vecHyperEdgeLargeRot) {

        HyperEdgeOdoMk hyperEdge = edge;
        PtrMsrSe2Kf2Kf pMsrOdo = hyperEdge.pMsrOdo;
        PtrMsrPt3Kf2Mk pMsrMk1 = hyperEdge.pMsrMk1;
        PtrMsrPt3Kf2Mk pMsrMk2 = hyperEdge.pMsrMk2;

        Se3 se3_b1b2 = pMsrOdo->se2;
        Mat R_b1b2 = se3_b1b2.R();
        Mat tvec_b1b2 = se3_b1b2.tvec;
        Mat tvec_c1m = pMsrMk1->pt3.tvec();
        Mat tvec_c2m = pMsrMk2->pt3.tvec();

        Mat A_blk = Mat::eye(3,3,CV_32FC1) - R_b1b2;
        Mat b_blk = R_b1b2*R_bc*tvec_c2m - R_bc*tvec_c1m + tvec_b1b2;

        Mat A_blk_trim = A_blk.rowRange(0,2).colRange(0,2);
        Mat b_blk_trim = b_blk.rowRange(0,2);

        Eigen::MatrixXd A_blk_trim_eigen = toEigenMatrixXd(A_blk_trim);
        Eigen::MatrixXd b_blk_trim_eigen = toEigenMatrixXd(b_blk_trim);

        A.block(countEdge*2,0,2,2) = A_blk_trim_eigen;
        b.block(countEdge*2,0,2,1) = b_blk_trim_eigen;

        countEdge++;
    }

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::MatrixXd x = svd.solve(b);
    Eigen::VectorXd residual = A*x - b;
    tvec_bd = ( Mat_<float>(3,1) << x(0), x(1), 0 );

    // DEBUG
    //    Mat rvec_bc;
    //    Rodrigues(R_bc, rvec_bc);
    //    cerr << "rvec_bc:" << endl << rvec_bc << endl;
    //    cerr << "x:" << endl << x << endl;
    //    cerr << "A:" << endl << A << endl;
    //    cerr << "b:" << endl << b << endl;
    //    cerr << "residual:" << endl << residual << endl;

    return residual.norm();
}

int SolverInitmk::FindCovisMark(const PtrKeyFrame _pKf1, const PtrKeyFrame _pKf2, set<pair<PtrMsrPt3Kf2Mk, PtrMsrPt3Kf2Mk>> &_setpairMsrMk) {
    // Find covisible mark from two keyframe, consider the ordered set
    _setpairMsrMk.clear();
    set<PtrMark> setpMk1 = mpDataset->GetMkbyKf(_pKf1);
    set<PtrMark> setpMk2 = mpDataset->GetMkbyKf(_pKf2);
    set<PtrMark> setpMkCovis;
    for (auto iterMk1 = setpMk1.begin(), iterMk2 = setpMk2.begin();
         iterMk1 != setpMk1.end() && iterMk2 != setpMk2.end(); ) {
        if (*iterMk1 == *iterMk2) {
            PtrMark pMkCovis = *iterMk1;
            setpMkCovis.insert(pMkCovis);
            iterMk1++;
            iterMk2++;
        }
        else if (*iterMk1 < *iterMk2) {
            iterMk1++;
        }
        else {
            iterMk2++;
        }
    }

    for(auto pMkCovis : setpMkCovis) {
        PtrMsrPt3Kf2Mk pMsrMk1 = mpDataset->GetMsrMkbyKfMk(_pKf1, pMkCovis);
        PtrMsrPt3Kf2Mk pMsrMk2 = mpDataset->GetMsrMkbyKfMk(_pKf2, pMkCovis);
        assert(pMsrMk1 && pMsrMk2);
        _setpairMsrMk.insert(make_pair(pMsrMk1,pMsrMk2));
    }

    return 0;
}

SolverOptMk::SolverOptMk(DatasetAruco *_pDataset):
    SolverAruco(_pDataset) {

}

void SolverOptMk::DoCalib() {

    //! Set optimizer
    SparseOptimizer optimizer;
    optimizer.setVerbose(true);
    InitOptimizerCalib(optimizer);

    //! Set extrinsic vertex
    int idVertexMax = 0;
    Isometry3D Iso3_bc = toG2oIsometry3D(mSe3cb);
    AddVertexSE3(optimizer, Iso3_bc, idVertexMax++);

    //! Set keyframe vertices
    map<PtrKeyFrame,int> mappKf2IdOpt;
    for (auto ptr : mpDataset->GetKfSet()) {
        PtrKeyFrame pKf = ptr;
        AddVertexSE2(optimizer, toG2oSE2(pKf->GetPoseBase()), idVertexMax);
        mappKf2IdOpt[pKf] = idVertexMax++;
    }

    //! Set mark vertices
    map<PtrMark,int> mappMk2IdOpt;
    for (auto ptr : mpDataset->GetMkSet()) {
        PtrMark pMk = ptr;
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
    for (auto ptr : mpDataset->GetMsrMkSet()) {
        PtrMsrPt3Kf2Mk pMsrMk = ptr;
        PtrKeyFrame pKf = pMsrMk->pKf;
        PtrMark pMk = pMsrMk->pMk;

        int idKf = mappKf2IdOpt[pKf];
        int idMk = mappMk2IdOpt[pMk];

        g2o::Vector3D measure = toG2oVector3D(pMsrMk->measure);
        g2o::Matrix3D info = toEigenMatrixXd(pMsrMk->info);

        AddEdgeXYZCalibCamOdo(optimizer, idKf, idMk, 0, measure, info);

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
    mSe3cb = toSe3(Iso3_bc_opt);

    //! Refresh keyframe
    for (auto pair : mappKf2IdOpt) {
        PtrKeyFrame pKf = pair.first;
        int idOpt = pair.second;
        VertexSE2* pVertex = static_cast<VertexSE2*>(optimizer.vertex(idOpt));
        pKf->SetPoseAllbyB(toSe2(pVertex->estimate()), mSe3cb);
    }

    //! Refresh landmark
    for (auto pair : mappMk2IdOpt) {
        PtrMark pMk = pair.first;
        int idOpt = pair.second;
        VertexPointXYZ* pVertex = static_cast<VertexPointXYZ*>(optimizer.vertex(idOpt));
        Mat tvec_wm = toCvMatf(pVertex->estimate());
        pMk->SetPoseTvec(tvec_wm);

        // DEBUG:
        //        cerr << "tvec_wm: " << tvec_wm.t() << endl;
    }
}



//! SolverOrb
//!
//!

SolverOrb::SolverOrb(DatasetOrb* _pDataset):
    Solver(_pDataset), mpDatasetOrb(_pDataset) {
    // set configures ...
    mOrbMatcher = ORBmatcher();

}


void SolverOrb::BuildDataset() {

    // match orb-keypoints between neigbour keyframes
    std::map<int, PtrKeyFrameOrb> mapId2pKfOrb = mpDatasetOrb->GetKfOrbMap();
    for (auto iter2 = mapId2pKfOrb.cbegin(), iter1 = iter2++;
         iter2 != mapId2pKfOrb.cend();
         ++iter1, ++iter2) {
        PtrKeyFrameOrb pKf1 = iter1->second;
        PtrKeyFrameOrb pKf2 = iter2->second;
        map<int, int> mapOrbMatches;
        MatchKeyPointOrb(pKf1, pKf2, mapOrbMatches);

        // debug ...
//        cerr << pKf1->GetId() << " "  << pKf2->GetId() << endl;
//        imshow("orb-solver", pKf1->GetImg());
//        waitKey(3);
        DrawMatches(pKf1, pKf2, mapOrbMatches);
        // debug end
    }
}

void SolverOrb::MatchKeyPointOrb(PtrKeyFrameOrb pKf1, PtrKeyFrameOrb pKf2, std::map<int, int>& match) {
    mOrbMatcher.MatchByBow(pKf1, pKf2, match);
}

void SolverOrb::DrawMatches(PtrKeyFrameOrb pKf1, PtrKeyFrameOrb pKf2, std::map<int, int>& match) {

    Mat imgKf1 = pKf1->GetImg().clone();
    Mat imgKf2 = pKf2->GetImg().clone();
//    cvtColor(pKf1->GetImg(), imgKf1, CV_GRAY2BGR);
//    cvtColor(pKf2->GetImg(), imgKf2, CV_GRAY2BGR);

    Size sizeImg1 = imgKf1.size();
    Size sizeImg2 = imgKf2.size();

    Mat imgMatch(sizeImg1.height*2, sizeImg1.width, imgKf1.type());
    imgKf1.copyTo(imgMatch(cv::Rect(0,0,sizeImg1.width,sizeImg1.height)));
    imgKf2.copyTo(imgMatch(cv::Rect(0,sizeImg1.height,sizeImg2.width,sizeImg2.height)));


    Scalar color = Scalar(0,255,0);
    //! Draw Features
    for (auto ele : pKf1->mvecKeyPoint) {
        KeyPoint kp = ele;
        Point2f pt = kp.pt;
        circle(imgMatch, pt, 5, color, 1);
    }
    for (auto ele : pKf2->mvecKeyPoint) {
        KeyPoint kp = ele;
        Point2f pt = kp.pt;
        pt.y += 480;
        circle(imgMatch, pt, 5, color, 1);
    }

    //! Draw Matches
    for (auto iter = match.begin(); iter != match.end(); iter++) {

        int idx1 = iter->first;
        KeyPoint kp1 = pKf1->mvecKeyPoint[idx1];
        Point2f pt1 = kp1.pt;

        int idx2 = iter->second;
        KeyPoint kp2 = pKf2->mvecKeyPoint[idx2];
        Point2f pt2 = kp2.pt;
        pt2.y += 480;

        line(imgMatch, pt1, pt2, color, 1);
    }

    imshow("debug-orbmatch", imgMatch);
    waitKey(10);
}


}

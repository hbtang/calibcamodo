#include "solver.h"
#include "adapter.h"
#include "mark.h"
#include "g2o/g2o_api.h"

namespace calibcamodo {

using namespace cv;
using namespace std;
using namespace g2o;

Solver::Solver(Dataset *_pDataset): mpDataset(_pDataset) {}

void Solver::CalibInitMk(const set<PtrMsrKf2AMk> &_measuremk, const set<PtrMsrSe2Kf2Kf> &_measureodo) {
    // calibrate the ground plane, return 3-by-1 norm vector in camera frame
    Mat nvec_cg;
    ComputeGrndPlane(_measuremk, nvec_cg);

    // compute camera projection frame, with 2 solutions on 2 direction of ground
    Mat rvec_dc_1, tvec_dc_1, rvec_dc_2, tvec_dc_2;
    ComputeCamProjFrame(nvec_cg, rvec_dc_1, tvec_dc_1);
    ComputeCamProjFrame(-nvec_cg, rvec_dc_2, tvec_dc_2);

    // compute xyyaw between based frame and camera projection frame,
    // choose the solution with smaller residual
    Mat rvec_bd_1, tvec_bd_1, rvec_bd_2, tvec_bd_2;
    double norm_res_1 = Compute2DExtrinsic(_measuremk, _measureodo, rvec_dc_1, tvec_dc_1, rvec_bd_1, tvec_bd_1);
    double norm_res_2 = Compute2DExtrinsic(_measuremk, _measureodo, rvec_dc_2, tvec_dc_2, rvec_bd_2, tvec_bd_2);
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

void Solver::ComputeGrndPlane(const set<PtrMsrKf2AMk> &_setmeasure, Mat &nvec_cg) {
    int numLclIdMk = 0;
    int numLclIdKf = 0;
    map<PtrArucoMark, int> mapMk2LclId;
    map<PtrKeyFrame, int> mapKf2LclId;

    for (auto ptrmeasure : _setmeasure) {
        PtrKeyFrame pKf = ptrmeasure->pKf;
        PtrArucoMark pAMk = ptrmeasure->pMk;
        if(!mapMk2LclId.count(pAMk))
            mapMk2LclId[pAMk] = numLclIdMk++;
        if(!mapKf2LclId.count(pKf))
            mapKf2LclId[pKf] = numLclIdKf++;
    }

    const int dimrow = numLclIdKf;
    const int dimcol = 3+numLclIdMk;
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(dimrow, dimcol);

    for (auto ptrmeasure : _setmeasure) {
        PtrKeyFrame pKf = ptrmeasure->pKf;
        PtrArucoMark pAMk = ptrmeasure->pMk;

        int lclIdMk = mapMk2LclId[pAMk];
        int lclIdKf = mapKf2LclId[pKf];

        Mat tvec = ptrmeasure->tvec();
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

void Solver::ComputeCamProjFrame(const Mat &nvec_cg, Mat &rvec_dc, Mat &tvec_dc, int flag) {

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

    Mat Rdc = Mat::zeros(3,3,CV_32FC1);
    rx.copyTo(Rdc.colRange(0,1));
    ry.copyTo(Rdc.colRange(1,2));
    rz.copyTo(Rdc.colRange(2,3));
    Rodrigues(Rdc, rvec_dc);

    tvec_dc = Mat::zeros(3,1,CV_32FC1);
    //    cerr << "rvec_dc" << rvec_dc << endl;
    //    cerr << "tvec_dc" << tvec_dc << endl;
}

double Solver::Compute2DExtrinsic(const set<PtrMsrKf2AMk> &_measuremk, const set<PtrMsrSe2Kf2Kf> &_measureodo,
                                  const Mat &rvec_dc, const Mat &tvec_dc, Mat &rvec_bd, Mat &tvec_bd) {

    double threshSmallRotation = 1.0/5000;

    vector<HyperEdgeOdoMk> vecHyperEdge;
    vector<HyperEdgeOdoMk> vecHyperEdgeSmallRot;
    vector<HyperEdgeOdoMk> vecHyperEdgeLargeRot;

    for(auto ptrmsrodo : _measureodo) {
        PtrMsrSe2Kf2Kf pMsrOdo = ptrmsrodo;
        double odo_ratio = pMsrOdo->ratio();

        PtrKeyFrame pKf1 = pMsrOdo->pKfHead;
        PtrKeyFrame pKf2 = pMsrOdo->pKfTail;

        set<pair<PtrMsrKf2AMk, PtrMsrKf2AMk>> setpairMsrMk;
        FindCovisMark(pKf1, pKf2, setpairMsrMk);
        set<pair<PtrMsrKf2AMk, PtrMsrKf2AMk>> setpairMsrMkInSet;
        for (auto pairMsrMk : setpairMsrMk) {
            if( _measuremk.count(pairMsrMk.first) && _measuremk.count(pairMsrMk.second) ) {
                setpairMsrMkInSet.insert(pairMsrMk);
            }
        }

        for (auto pairMsrMk : setpairMsrMkInSet) {
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
        PtrMsrKf2AMk pMsrMk1 = hyperEdge.pMsrMk1;
        PtrMsrKf2AMk pMsrMk2 = hyperEdge.pMsrMk2;

        Mat R_b1b2 = pMsrOdo->matR();
        Mat tvec_b1b2 = pMsrOdo->tvec();
        Mat tvec_c1m = pMsrMk1->tvec();
        Mat tvec_c2m = pMsrMk2->tvec();
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
        //        cerr << tvec_c1m.t() << endl;
        //        cerr << tvec_c2m.t() << endl;
        //        cerr << tvec_b1b2.t() << endl;
        //        cerr << tvec_b1b2_bar.t() << endl;
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
        PtrMsrKf2AMk pMsrMk1 = hyperEdge.pMsrMk1;
        PtrMsrKf2AMk pMsrMk2 = hyperEdge.pMsrMk2;

        Mat R_b1b2 = pMsrOdo->matR();
        Mat tvec_b1b2 = pMsrOdo->tvec();
        Mat tvec_c1m = pMsrMk1->tvec();
        Mat tvec_c2m = pMsrMk2->tvec();

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
    tvec_bd = ( Mat_<float>(3,1) << x(0), x(1), 0 );
    Eigen::VectorXd residual = A*x - b;
    return residual.norm();
}

int Solver::FindCovisMark(const PtrKeyFrame _pKf1, const PtrKeyFrame _pKf2, set<pair<PtrMsrKf2AMk, PtrMsrKf2AMk>> &_setpairMsrMk) {
    // Find covisible mark from two keyframe, consider the ordered set
    _setpairMsrMk.clear();
    set<PtrArucoMark> setpMk1 = _pKf1->GetMk();
    set<PtrArucoMark> setpMk2 = _pKf2->GetMk();
    set<PtrArucoMark> setpMk12;

    for (auto iterMk1 = setpMk1.begin(), iterMk2 = setpMk2.begin();
         iterMk1 != setpMk1.end() && iterMk2 != setpMk2.end(); ) {
        if (*iterMk1 == *iterMk2) {
            PtrArucoMark pMkCovis = *iterMk1;
            setpMk12.insert(pMkCovis);
            PtrMsrKf2AMk pMsrMk1 = _pKf1->GetMsrMk(pMkCovis);
            PtrMsrKf2AMk pMsrMk2 = _pKf2->GetMsrMk(pMkCovis);
            _setpairMsrMk.insert(make_pair(pMsrMk1,pMsrMk2));
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
    return 0;
}

void Solver::CalibOptMk(const set<PtrMsrKf2AMk> &_measuremk, const set<PtrMsrSe2Kf2Kf> &_measureodo) {

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
    map<PtrArucoMark,int> mappMk2IdOpt;
    for (auto ptr : mpDataset->GetMkSet()) {
        PtrArucoMark pMk = ptr;
        //! NEED TO ADD INIT MK POSE HERE !!!
        AddVertexPointXYZ(optimizer, toG2oVector3D(pMk->GetPose().tvec), idVertexMax);
        mappMk2IdOpt[pMk] = idVertexMax++;
    }

    g2o::Matrix3D matEye3;
    matEye3 << 1,0,0,
            0,1,0,
            0,0,1;

    //! Set odometry edges
    for (auto ptr : _measureodo) {
        PtrMsrSe2Kf2Kf pMsrOdo = ptr;
        PtrKeyFrame pKf0 = pMsrOdo->pKfHead;
        PtrKeyFrame pKf1 = pMsrOdo->pKfTail;
        int id0 = mappKf2IdOpt[pKf0];
        int id1 = mappKf2IdOpt[pKf1];

        g2o::SE2 measure = toG2oSE2(pMsrOdo->se2);
        g2o::Matrix3D info = 1e-4 * matEye3;
        info(2,2) = 1/(3*PI/180)/(3*PI/180);

        AddEdgeSE2(optimizer, id0, id1, measure, info);
    }

    //! Set mark measurement edges
    for (auto ptr : _measuremk) {
        PtrMsrKf2AMk pMsrMk = ptr;
        PtrKeyFrame pKf = pMsrMk->pKf;
        PtrArucoMark pMk = pMsrMk->pMk;

        int idKf = mappKf2IdOpt[pKf];
        int idMk = mappMk2IdOpt[pMk];

        g2o::Vector3D measure = toG2oVector3D(pMsrMk->tvec());
        g2o::Matrix3D info = 0.02 * matEye3;

        AddEdgeXYZCalibCamOdo(optimizer, idKf, idMk, 0, measure, info);
    }

    //! Do optimize
    optimizer.initializeOptimization();
    optimizer.optimize(30);


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
        PtrArucoMark pMk = pair.first;
        int idOpt = pair.second;
        VertexPointXYZ* pVertex = static_cast<VertexPointXYZ*>(optimizer.vertex(idOpt));
        Mat tvec_wm = toCvMatf(pVertex->estimate());
        pMk->SetPoseTranslation(tvec_wm);
    }
}

}

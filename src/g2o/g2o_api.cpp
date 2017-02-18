#include "g2o_api.h"
#include "core/adapter.h"

namespace calibcamodo{

using namespace g2o;
using namespace std;

void InitOptimizerSlam(Optimizer &opt, bool verbose){
    LinearSolver* linearSolver = new LinearSolver();
    BlockSolver* blockSolver = new BlockSolver(linearSolver);
    Algorithm* solver = new Algorithm(blockSolver);
    opt.setAlgorithm(solver);
    opt.setVerbose(verbose);
}

g2o::CameraParameters* AddCamPara(Optimizer &opt, const cv::Mat &K, int id){
    Eigen::Vector2d principal_point(K.at<float>(0,2), K.at<float>(1,2));
    g2o::CameraParameters* campr = new g2o::CameraParameters(K.at<float>(0,0), principal_point, 0.);
    campr->setId(id);
    opt.addParameter(campr);
    return campr;
}

g2o::ParameterSE3Offset* AddParaSE3Offset(Optimizer &opt, const g2o::Isometry3D& se3offset, int id){
    g2o::ParameterSE3Offset * para = new g2o::ParameterSE3Offset();
    para->setOffset(se3offset);
    para->setId(id);
    opt.addParameter(para);
    return para;
}

g2o::ParameterCamera* AddParaCamera(Optimizer &opt, const cv::Mat& K, const g2o::Isometry3D& se3offset, int id) {
    g2o::ParameterCamera* para = new g2o::ParameterCamera();
    para->setOffset(se3offset);
    float fx = K.at<float>(0,0);
    float fy = K.at<float>(1,1);
    float cx = K.at<float>(0,2);
    float cy = K.at<float>(1,2);
    para->setKcam(fx, fy, cx, cy);
    para->setId(id);
    opt.addParameter(para);
    return para;
}

void AddVertexSE3Expmap(Optimizer &opt, const g2o::SE3Quat &pose, int id, bool fixed){
    g2o::VertexSE3Expmap* v = new g2o::VertexSE3Expmap();
    v->setEstimate(pose);
    v->setFixed(fixed);
    v->setId(id);
    opt.addVertex(v);
}

void AddVertexSBAXYZ(Optimizer &opt, const Eigen::Vector3d &xyz, int id, bool marginal, bool fixed){
    g2o::VertexSBAPointXYZ* v = new g2o::VertexSBAPointXYZ();
    v->setEstimate(xyz);
    v->setId(id);
    v->setMarginalized(marginal);
    v->setFixed(fixed);
    opt.addVertex(v);
}

void AddVertexSE3(Optimizer &opt, const g2o::Isometry3D &pose, int id, bool fixed){
    g2o::VertexSE3* v = new g2o::VertexSE3();
    v->setEstimate(pose);
    v->setFixed(fixed);
    v->setId(id);
    opt.addVertex(v);
}

void AddVertexSE2(Optimizer &opt, const g2o::SE2 &pose, int id, bool fixed) {
    VertexSE2* v = new VertexSE2();
    v->setEstimate(pose);
    v->setFixed(fixed);
    v->setId(id);
    opt.addVertex(v);

    // DEBUG
    //    SE2 val = v->estimate();
    //    cerr << val.toVector() << endl;
}


void AddVertexPointXYZ(Optimizer &opt, const g2o::Vector3D &xyz, int id, bool marginal){
    g2o::VertexPointXYZ* v = new g2o::VertexPointXYZ();
    v->setEstimate(xyz);
    v->setId(id);
    v->setMarginalized(marginal);
    opt.addVertex(v);
    // DEBUG
    //    cerr << xyz << endl;
}

g2o::EdgeSE3Expmap* AddEdgeSE3Expmap(Optimizer &opt, int id0, int id1,
                                     const g2o::SE3Quat &measure, const g2o::Matrix6d &info){
    g2o::EdgeSE3Expmap* e = new g2o::EdgeSE3Expmap();
    e->setMeasurement(measure);
    e->vertices()[0] = opt.vertex(id0);
    e->vertices()[1] = opt.vertex(id1);

    // The input info is [trans rot] order, but EdgeSE3Expmap requires [rot trans]
    g2o::Matrix6d infoNew;
    infoNew.block(0,0,3,3) = info.block(3,3,3,3);
    infoNew.block(3,0,3,3) = info.block(0,3,3,3);
    infoNew.block(0,3,3,3) = info.block(3,0,3,3);
    infoNew.block(3,3,3,3) = info.block(0,0,3,3);

    e->setInformation(infoNew);
    opt.addEdge(e);
    return e;
}

g2o::EdgeProjectXYZ2UV* AddEdgeXYZ2UV(Optimizer &opt, int id0, int id1, int paraId,
                                      const Eigen::Vector2d &measure, const Eigen::Matrix2d &info, double thHuber){
    g2o::EdgeProjectXYZ2UV* e = new g2o::EdgeProjectXYZ2UV();
    e->vertices()[0] = opt.vertex(id0);
    e->vertices()[1] = opt.vertex(id1);
    e->setMeasurement(measure);
    e->setInformation(info);
    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
    rk->setDelta(thHuber);
    e->setRobustKernel(rk);
    e->setParameterId(0,paraId);
    opt.addEdge(e);
    return e;
}

g2o::EdgeSE3* AddEdgeSE3(Optimizer &opt, int id0, int id1,
                         const g2o::Isometry3D &measure, const g2o::Matrix6d &info){
    g2o::EdgeSE3 *e =  new g2o::EdgeSE3();
    e->setMeasurement(measure);
    e->vertices()[0] = opt.vertex(id0);
    e->vertices()[1] = opt.vertex(id1);
    e->setInformation(info);
    opt.addEdge(e);
    return e;
}

g2o::EdgeSE2* AddEdgeSE2(Optimizer &opt, int id0, int id1,
                         const g2o::SE2 &measure, const g2o::Matrix3D &info){
    g2o::EdgeSE2 *e =  new g2o::EdgeSE2();
    e->setMeasurement(measure);
    e->vertices()[0] = opt.vertex(id0);
    e->vertices()[1] = opt.vertex(id1);
    e->setInformation(info);
    opt.addEdge(e);

    // DEBUG
    //    e->computeError();
    //    cerr << e->error() << endl;
    //    g2o::VertexSE2 *v0 = static_cast<g2o::VertexSE2*>(e->vertices()[0]);
    //    g2o::VertexSE2 *v1 = static_cast<g2o::VertexSE2*>(e->vertices()[1]);
    //    SE2 val0 = v0->estimate();
    //    SE2 val1 = v1->estimate();
    //    cerr << val0.toIsometry().matrix() << endl;
    //    cerr << val1.toIsometry().matrix() << endl;
    //    cerr << endl;
    //    cerr << e->chi2() << endl;

    return e;
}

g2o::EdgeSE3PointXYZ* AddEdgeSE3XYZ(Optimizer &opt, int idse3, int idxyz, int paraSE3OffsetId,
                                    const g2o::Vector3D &measure, const g2o::Matrix3D &info, double thHuber){
    g2o::EdgeSE3PointXYZ* e = new g2o::EdgeSE3PointXYZ();
    e->vertices()[0] = opt.vertex(idse3);
    e->vertices()[1] = opt.vertex(idxyz);
    e->setMeasurement(measure);
    e->setParameterId(0, paraSE3OffsetId);
    e->setInformation(info);
    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
    rk->setDelta(thHuber);
    e->setRobustKernel(rk);
    opt.addEdge(e);

    return e;
}

g2o::EdgeOptMk* AddEdgeOptMk(Optimizer &opt, int idKf, int idMk, int idCalib,
                             const g2o::Vector3D &measure, const g2o::Matrix3D &info) {
    g2o::EdgeOptMk* e = new g2o::EdgeOptMk();
    e->vertices()[0] = opt.vertex(idKf);
    e->vertices()[1] = opt.vertex(idMk);
    e->vertices()[2] = opt.vertex(idCalib);
    e->setMeasurement(measure);
    e->setInformation(info);
    opt.addEdge(e);

    // DEBUG
    //    e->computeError();
    //    cerr << "init error=" << e->error() << endl << endl;
    //    g2o::VertexSE2 *v0 = static_cast<g2o::VertexSE2*>(e->vertices()[0]);
    //    g2o::VertexPointXYZ *v1 = static_cast<g2o::VertexPointXYZ*>(e->vertices()[1]);
    //    g2o::VertexSE3 *v2 = static_cast<g2o::VertexSE3*>(e->vertices()[2]);
    //    SE2 val0 = v0->estimate();
    //    Vector3D val1 = v1->estimate();
    //    Isometry3D val2 = v2->estimate();
    //    cerr << val0.toIsometry().matrix() << endl;
    //    cerr << val1 << endl;
    //    cerr << val2.matrix() << endl;
    //    cerr << e->chi2() << endl;
    //    cerr << "---" << endl;
    //    cerr << " idKf: " << idKf << " idMk: " << idMk << " chi2: " << e->chi2() << endl;

    return e;
}

g2o::EdgeVSlam* AddEdgeVSlam(Optimizer &opt, int idKf, int idMp, int idParam, const g2o::Vector2D &measure, const g2o::Matrix2D &info) {
    g2o::EdgeVSlam* e = new g2o::EdgeVSlam();
    e->vertices()[0] = opt.vertex(idKf);
    e->vertices()[1] = opt.vertex(idMp);
    e->setMeasurement(measure);
    e->setInformation(info);
    e->setParameterId(0, idParam);
    opt.addEdge(e);

    // debug
//    e->computeError();
//    Vector2D error = e->error();
//    cerr << " -- erru = " << error(0) << " -- errv = " << error(1)
//         << " -- idKf = " << idKf << " -- idMp = " << idMp
//         << endl;

    return e;
}

g2o::EdgeVSclam* AddEdgeVSclam(Optimizer &opt, int idKf, int idMp, int idCamOffset, int idCamParam,
                               const g2o::Vector2D &measure, const g2o::Matrix2D &info) {
    g2o::EdgeVSclam* e = new g2o::EdgeVSclam();
    e->vertices()[0] = opt.vertex(idKf);
    e->vertices()[1] = opt.vertex(idMp);
    e->vertices()[2] = opt.vertex(idCamOffset);
    e->setMeasurement(measure);
    e->setInformation(info);
    e->setParameterId(0, idCamParam);
    opt.addEdge(e);
    return e;
}


g2o::Vector3D EstimateVertexSBAXYZ(Optimizer &opt, int id){
    g2o::VertexSBAPointXYZ* v = static_cast<g2o::VertexSBAPointXYZ*>
            (opt.vertex(id));
    return v->estimate();
}

g2o::SE3Quat EstimateVertexSE3Expmap(Optimizer &opt, int id){
    g2o::VertexSE3Expmap* v = static_cast<g2o::VertexSE3Expmap*>
            (opt.vertex(id));
    return v->estimate();
}

g2o::Isometry3D EstimateVertexSE3(Optimizer &opt, int id){
    g2o::VertexSE3 *v = static_cast<g2o::VertexSE3*>(opt.vertex(id));
    return v->estimate();
}

g2o::Vector3D EstimateVertexXYZ(Optimizer &opt, int id){
    g2o::VertexPointXYZ* v = static_cast<g2o::VertexPointXYZ*>(opt.vertex(id));
    return v->estimate();
}

void InitOptimizerCalib(Optimizer& optimizer) {
    LinearSolver* linearSolver = new LinearSolver();
    linearSolver->setBlockOrdering(false);
    BlockSolver* blockSolver = new BlockSolver(linearSolver);
    OptimizationAlgorithm* solver = new OptimizationAlgorithmLevenberg(blockSolver);
    optimizer.setAlgorithm(solver);
}


}// namespace odoslam

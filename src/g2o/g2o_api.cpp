#include "g2o_api.h"
#include "adapter.h"

namespace calibcamodo{

using namespace g2o;
using namespace std;

void InitOptimizerSlam(SlamOptimizer &opt, bool verbose){
    SlamLinearSolver* linearSolver = new SlamLinearSolver();
    SlamBlockSolver* blockSolver = new SlamBlockSolver(linearSolver);
    SlamAlgorithm* solver = new SlamAlgorithm(blockSolver);
    opt.setAlgorithm(solver);
    opt.setVerbose(verbose);
}

CamPara* AddCamPara(SlamOptimizer &opt, const cv::Mat &K, int id){
    Eigen::Vector2d principal_point(K.at<float>(0,2), K.at<float>(1,2));
    CamPara* campr = new CamPara(K.at<float>(0,0), principal_point, 0.);
    campr->setId(id);
    opt.addParameter(campr);
    return campr;
}

g2o::ParameterSE3Offset* AddParaSE3Offset(SlamOptimizer &opt, const g2o::Isometry3D& se3offset, int id){
    g2o::ParameterSE3Offset * para = new g2o::ParameterSE3Offset();
    para->setOffset(se3offset);
    para->setId(id);
    opt.addParameter(para);
    return para;
}

void AddVertexSE3Expmap(SlamOptimizer &opt, const g2o::SE3Quat &pose, int id, bool fixed){
    g2o::VertexSE3Expmap* v = new g2o::VertexSE3Expmap();
    v->setEstimate(pose);
    v->setFixed(fixed);
    v->setId(id);
    opt.addVertex(v);
}

void AddVertexSBAXYZ(SlamOptimizer &opt, const Eigen::Vector3d &xyz, int id, bool marginal, bool fixed){
    g2o::VertexSBAPointXYZ* v = new g2o::VertexSBAPointXYZ();
    v->setEstimate(xyz);
    v->setId(id);
    v->setMarginalized(marginal);
    v->setFixed(fixed);
    opt.addVertex(v);
}

void AddVertexSE3(SlamOptimizer &opt, const g2o::Isometry3D &pose, int id, bool fixed){
    g2o::VertexSE3* v = new g2o::VertexSE3();
    v->setEstimate(pose);
    v->setFixed(fixed);
    v->setId(id);
    opt.addVertex(v);
}

void AddVertexSE2(SlamOptimizer &opt, const g2o::SE2 &pose, int id, bool fixed) {
    VertexSE2* v = new VertexSE2();
    v->setEstimate(pose);
    v->setFixed(fixed);
    v->setId(id);
    opt.addVertex(v);
}


void AddVertexPointXYZ(SlamOptimizer &opt, const g2o::Vector3D &xyz, int id, bool marginal){
    g2o::VertexPointXYZ* v = new g2o::VertexPointXYZ();
    v->setEstimate(xyz);
    v->setId(id);
    v->setMarginalized(marginal);
    opt.addVertex(v);
}

g2o::EdgeSE3Expmap* AddEdgeSE3Expmap(SlamOptimizer &opt, const g2o::SE3Quat &measure, int id0, int id1, const g2o::Matrix6d &info){
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

g2o::EdgeProjectXYZ2UV* AddEdgeXYZ2UV(SlamOptimizer &opt, const Eigen::Vector2d &measure, int id0, int id1, int paraId, const Eigen::Matrix2d &info, double thHuber){
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

g2o::EdgeSE3* AddEdgeSE3(SlamOptimizer &opt, const g2o::Isometry3D &measure, int id0, int id1, const g2o::Matrix6d &info){
    g2o::EdgeSE3 *e =  new g2o::EdgeSE3();
    e->setMeasurement(measure);
    e->vertices()[0] = opt.vertex(id0);
    e->vertices()[1] = opt.vertex(id1);
    e->setInformation(info);

    opt.addEdge(e);

    return e;
}

g2o::EdgeSE3PointXYZ* AddEdgeSE3XYZ(SlamOptimizer &opt, const g2o::Vector3D &measure, int idse3, int idxyz, int paraSE3OffsetId, const g2o::Matrix3D &info, double thHuber){
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

g2o::Vector3D EstimateVertexSBAXYZ(SlamOptimizer &opt, int id){
    g2o::VertexSBAPointXYZ* v = static_cast<g2o::VertexSBAPointXYZ*>
            (opt.vertex(id));
    return v->estimate();
}

g2o::SE3Quat EstimateVertexSE3Expmap(SlamOptimizer &opt, int id){
    g2o::VertexSE3Expmap* v = static_cast<g2o::VertexSE3Expmap*>
            (opt.vertex(id));
    return v->estimate();
}

g2o::Isometry3D EstimateVertexSE3(SlamOptimizer &opt, int id){
    g2o::VertexSE3 *v = static_cast<g2o::VertexSE3*>(opt.vertex(id));
    return v->estimate();
}

g2o::Vector3D EstimateVertexXYZ(SlamOptimizer &opt, int id){
    g2o::VertexPointXYZ* v = static_cast<g2o::VertexPointXYZ*>(opt.vertex(id));
    return v->estimate();
}

void InitOptimizerCalib(SparseOptimizer& optimizer) {
  typedef BlockSolver< BlockSolverTraits<-1, -1> >  SclamBlockSolver;
  typedef LinearSolverCholmod<SclamBlockSolver::PoseMatrixType> SclamLinearSolver;
  SclamLinearSolver* linearSolver = new SclamLinearSolver();
  linearSolver->setBlockOrdering(false);
  SclamBlockSolver* blockSolver = new SclamBlockSolver(linearSolver);
  OptimizationAlgorithm* solver = new OptimizationAlgorithmLevenberg(blockSolver);
  optimizer.setAlgorithm(solver);
}


}// namespace odoslam

#ifndef OPTIMIZER_H
#define OPTIMIZER_H

#include <Eigen/Dense>
#include <Eigen/StdVector>

#include <g2o/core/eigen_types.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/factory.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/types/sim3/types_seven_dof_expmap.h>
#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/types/slam2d/edge_se2.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>

#include <opencv2/core/core.hpp>

namespace calibcamodo{

typedef g2o::BlockSolverX SlamBlockSolver;
typedef g2o::LinearSolverCholmod<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;
typedef g2o::OptimizationAlgorithmLevenberg SlamAlgorithm;
typedef g2o::SparseOptimizer SlamOptimizer;
typedef g2o::CameraParameters CamPara;

void InitOptimizerSlam(SlamOptimizer &opt, bool verbose=false);
void InitOptimizerCalib(g2o::SparseOptimizer& optimizer);

CamPara* AddCamPara(SlamOptimizer &opt, const cv::Mat& K, int id);
g2o::ParameterSE3Offset* AddParaSE3Offset(SlamOptimizer &opt, const g2o::Isometry3D& se3offset, int id);

void AddVertexSE3Expmap(SlamOptimizer &opt, const g2o::SE3Quat& pose, int id, bool fixed=false);
void AddVertexSBAXYZ(SlamOptimizer &opt, const Eigen::Vector3d &xyz, int id, bool marginal=true, bool fixed=false);
void AddVertexSE3(SlamOptimizer &opt, const g2o::Isometry3D &pose, int id, bool fixed=false);
void AddVertexSE2(SlamOptimizer &opt, const g2o::SE2 &pose, int id, bool fixed=false);
void AddVertexPointXYZ(SlamOptimizer &opt, const g2o::Vector3D &xyz, int id, bool marginal=true);

g2o::EdgeSE3Expmap* AddEdgeSE3Expmap(SlamOptimizer &opt, const g2o::SE3Quat& measure, int id0, int id1, const g2o::Matrix6d& info);
g2o::EdgeProjectXYZ2UV* AddEdgeXYZ2UV(SlamOptimizer &opt, const Eigen::Vector2d& measure, int id0, int id1, int paraId, const Eigen::Matrix2d &info, double thHuber);
g2o::EdgeSE3* AddEdgeSE3(SlamOptimizer &opt, const g2o::Isometry3D &measure, int id0, int id1, const g2o::Matrix6d& info);
g2o::EdgeSE3PointXYZ* AddEdgeSE3XYZ(SlamOptimizer &opt, const g2o::Vector3D& measure, int idse3, int idxyz, int paraSE3OffsetId, const g2o::Matrix3D &info, double thHuber);

g2o::Isometry3D EstimateVertexSE3(SlamOptimizer &opt, int id);
Eigen::Vector3d EstimateVertexXYZ(SlamOptimizer &opt, int id);
g2o::SE3Quat EstimateVertexSE3Expmap(SlamOptimizer &opt, int id);
g2o::Vector3D EstimateVertexSBAXYZ(SlamOptimizer &opt, int id);

}

#endif

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

#include "edge_calibcamodo.h"

namespace calibcamodo{

typedef g2o::BlockSolverX BlockSolver;
typedef g2o::LinearSolverCholmod<BlockSolver::PoseMatrixType> LinearSolver;
typedef g2o::OptimizationAlgorithmLevenberg Algorithm;
typedef g2o::SparseOptimizer Optimizer;

void InitOptimizerSlam(Optimizer &opt, bool verbose=false);
void InitOptimizerCalib(Optimizer &opt);

g2o::CameraParameters* AddCamPara(Optimizer &opt, const cv::Mat& K, int id);
g2o::ParameterSE3Offset* AddParaSE3Offset(Optimizer &opt, const g2o::Isometry3D& se3offset, int id);
g2o::ParameterCamera* AddParaCamera(Optimizer &opt, const cv::Mat& K, const g2o::Isometry3D& se3offset, int id);

void AddVertexSE3Expmap(Optimizer &opt, const g2o::SE3Quat& pose, int id, bool fixed=false);
void AddVertexSBAXYZ(Optimizer &opt, const Eigen::Vector3d &xyz, int id, bool marginal=true, bool fixed=false);
void AddVertexSE3(Optimizer &opt, const g2o::Isometry3D &pose, int id, bool fixed=false);
void AddVertexSE2(Optimizer &opt, const g2o::SE2 &pose, int id, bool fixed=false);
void AddVertexPointXYZ(Optimizer &opt, const g2o::Vector3D &xyz, int id, bool marginal=false);

g2o::EdgeSE3Expmap* AddEdgeSE3Expmap(Optimizer &opt, int id0, int id1, const g2o::SE3Quat& measure, const g2o::Matrix6d& info);
g2o::EdgeProjectXYZ2UV* AddEdgeXYZ2UV(Optimizer &opt, int id0, int id1, int paraId, const Eigen::Vector2d& measure, const Eigen::Matrix2d &info, double thHuber);
g2o::EdgeSE3* AddEdgeSE3(Optimizer &opt, int id0, int id1, const g2o::Isometry3D &measure, const g2o::Matrix6d& info);
g2o::EdgeSE2* AddEdgeSE2(Optimizer &opt, int id0, int id1, const g2o::SE2 &measure, const g2o::Matrix3D &info);
g2o::EdgeSE3PointXYZ* AddEdgeSE3XYZ(Optimizer &opt, int idse3, int idxyz, int paraSE3OffsetId, const g2o::Vector3D& measure, const g2o::Matrix3D &info, double thHuber);
g2o::EdgeOptMk* AddEdgeOptMk(Optimizer &opt, int idKf, int idMk, int idCalib, const g2o::Vector3D &measure, const g2o::Matrix3D &info);
g2o::EdgeVSlam* AddEdgeVSlam(Optimizer &opt, int idKf, int idMp, int idParam, const g2o::Vector2D &measure, const g2o::Matrix2D &info);
g2o::EdgeVSclam* AddEdgeVSclam(Optimizer &opt, int idKf, int idMp, int idCamOffset, int idCamParam, const g2o::Vector2D &measure, const g2o::Matrix2D &info);

g2o::Isometry3D EstimateVertexSE3(Optimizer &opt, int id);
Eigen::Vector3d EstimateVertexXYZ(Optimizer &opt, int id);
g2o::SE3Quat EstimateVertexSE3Expmap(Optimizer &opt, int id);
g2o::Vector3D EstimateVertexSBAXYZ(Optimizer &opt, int id);

}

#endif

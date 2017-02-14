#ifndef ADAPTER_H
#define ADAPTER_H

#include "type.h"

#include <opencv2/core/core.hpp>

#include <g2o/core/eigen_types.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/types/sim3/types_seven_dof_expmap.h>
#include <g2o/types/slam3d/dquat2mat.h>
#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/types/slam2d/edge_se2.h>

namespace calibcamodo{

// Convert to my type
Se3 toSe3(const g2o::Isometry3D &in);
Se2 toSe2(const g2o::SE2 &in);

// Convert to cv
cv::Mat toCvMatf(const g2o::SE3Quat &SE3);
cv::Mat toCvMatf(const Eigen::Matrix<double,4,4> &m);
cv::Mat toCvMatf(const Eigen::Matrix3d &m);
cv::Mat toCvMatf(const Eigen::Matrix<double,3,1> &m);
cv::Mat toCvMatf(const g2o::Matrix6d& m);
cv::Mat toCvMatf(const Eigen::MatrixXd &eigenMat);
cv::Mat toCvMatf(const g2o::Isometry3D& t);

cv::Mat toT4x4(cv::Mat R, cv::Mat T);
cv::Mat toT4x4(float x, float y, float theta);


cv::Point2f toCvPt2f(const Eigen::Vector2d& vec);
cv::Point3f toCvPt3f(const Eigen::Vector3d& vec);

cv::Mat toCvSE3(const Eigen::Matrix<double,3,3> &R, const Eigen::Matrix<double,3,1> &t);
void Vec2MatSe3(const cv::Mat &rvec, const cv::Mat &tvec, cv::Mat &T);
void Mat2VecSe3(const cv::Mat &T, cv::Mat &rvec, cv::Mat &tvec);

std::vector<cv::Mat> toDescriptorVector(const cv::Mat &Descriptors);


// Convert to Eigen
Eigen::Matrix<double,3,1> toEigenVector3d(const cv::Mat &cvVector);
Eigen::Matrix<double,3,1> toEigenVector3d(const cv::Point3f &cvPoint);
Eigen::Matrix<double,2,1> toEigenVector2d(const cv::Mat &cvVector);
Eigen::Matrix<double,3,3> toEigenMatrix3d(const cv::Mat &cvMat3);
Eigen::MatrixXd toEigenMatrixXd(const cv::Mat &cvMat);
Eigen::Vector2d toEigenVector2d(const cv::Point2f &cvVector);


// Convert to g2o
g2o::SE2 toG2oSE2(const Se2 &in);
g2o::Isometry3D toG2oIsometry3D(const cv::Mat& T);
g2o::Isometry3D toG2oIsometry3D(const g2o::SE3Quat& se3quat);
g2o::Isometry3D toG2oIsometry3D(const Se3& _se3);
g2o::SE3Quat toG2oSE3Quat(const g2o::Isometry3D& iso);
g2o::SE3Quat toG2oSE3Quat(const cv::Mat &cvT);
g2o::SE3Quat toG2oSE3Quat(const g2o::SE2& se2);
g2o::Matrix6d toG2oMatrix6d(const cv::Mat& cvMat6f);
g2o::Vector3D toG2oVector3D(const cv::Mat &cvmat);
g2o::Vector2D toG2oVector2D(const cv::Mat& cvmat);


// Convert to transformation matrix in eigen
g2o::Matrix4D toTransMat(const g2o::SE3Quat& se3);


// other functions
std::vector<float> toQuaternion(const cv::Mat &M);

} // namespace odoslam
#endif

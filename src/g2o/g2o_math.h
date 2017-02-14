#ifndef G2O_MATH_H
#define G2O_MATH_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <g2o/core/eigen_types.h>
#include <g2o/types/slam3d/se3quat.h>

namespace g2o {

typedef Eigen::Matrix<double,6,6,Eigen::ColMajor> Matrix6D;
typedef Eigen::Matrix<double,6,1,Eigen::ColMajor> Vector6D;

Matrix3D skew(const Vector3D& v);

Matrix3D Jl(const Vector3D& v);
Matrix3D JlInv(const Vector3D& v);
Matrix6D JJl(const Vector6D& v);
Matrix6D JJl(const SE3Quat& se3);
Matrix6D JJlInv(const Vector6D& v);

Eigen::Matrix<double,4,6,Eigen::ColMajor> dcircle(const Vector4D& v);
Eigen::Matrix<double,4,6,Eigen::ColMajor> dcircle(const Vector3D& v);
Eigen::Matrix<double,6,4,Eigen::ColMajor> ccircle(const Vector4D& v);
Eigen::Matrix<double,6,4,Eigen::ColMajor> ccircle(const Vector3D& v);

Matrix6D Ad(const Matrix4D& T);
Matrix6D ad(const Matrix4D& T);

Eigen::Matrix<double,6,3,Eigen::ColMajor> JacobianSE3SE2(const Vector6D& vlie);
Eigen::Matrix<double,6,3,Eigen::ColMajor> JacobianSE3SE2(const SE3Quat& se3);
Vector3D JacobianRhoTheta(double x, double y, double theta);

Eigen::Matrix<double,2,3,Eigen::ColMajor> JacobianUV2XYZ(const Vector3D& pcm, const Matrix3D& K);





}

#endif // G2O_MATH_H

#include <opencv2/calib3d/calib3d.hpp>

#include "adapter.h"
#include <iostream>

namespace calibcamodo{

using namespace cv;
using namespace std;


cv::Mat toT4x4(cv::Mat R, cv::Mat T){
    cv::Mat T4x4 = cv::Mat::eye(4,4,R.type());
    R.copyTo(T4x4.rowRange(0,3).colRange(0,3));
    T.copyTo(T4x4.rowRange(0,3).col(3));
    return T4x4.clone();
}

cv::Mat toT4x4(float x, float y, float theta){
    float costht = cos(theta);
    float sintht = sin(theta);

    return (cv::Mat_<float>(4,4) <<
            costht,-sintht, 0, x,
            sintht, costht, 0, y,
            0,      0,      1, 0,
            0,      0,      0, 1);
}


Eigen::Vector2d toEigenVector2d(const cv::Point2f &cvVector){
    Eigen::Vector2d v;
    v << cvVector.x, cvVector.y;
    return v;
}

g2o::Isometry3D toG2oIsometry3D(const cv::Mat &T){
    Eigen::Matrix<double,3,3> R;
    R << T.at<float>(0,0), T.at<float>(0,1), T.at<float>(0,2),
            T.at<float>(1,0), T.at<float>(1,1), T.at<float>(1,2),
            T.at<float>(2,0), T.at<float>(2,1), T.at<float>(2,2);
    g2o::Isometry3D ret = (g2o::Isometry3D) Eigen::Quaterniond(R);
    Eigen::Vector3d t(T.at<float>(0,3), T.at<float>(1,3), T.at<float>(2,3));
    ret.translation() = t;
    return ret;
}

g2o::Isometry3D toG2oIsometry3D(const Se3& _se3) {
    return toG2oIsometry3D(_se3.T());
}

cv::Mat toCvMatf(const g2o::Isometry3D &t){
    return toCvMatf(g2o::internal::toSE3Quat(t));
}

cv::Point2f toCvPt2f(const Eigen::Vector2d& vec){
    return cv::Point2f(vec(0),vec(1));
}

cv::Point3f toCvPt3f(const Eigen::Vector3d& vec){
    return cv::Point3f(vec(0), vec(1), vec(2));
}

g2o::Isometry3D toG2oIsometry3D(const g2o::SE3Quat &se3quat){
    return g2o::internal::fromSE3Quat(se3quat);
}

g2o::SE3Quat toG2oSE3Quat(const g2o::Isometry3D &iso){
    return g2o::internal::toSE3Quat(iso);
}

g2o::SE3Quat toG2oSE3Quat(const g2o::SE2& se2) {
    g2o::Vector3D v2 = se2.toVector();
    double x = v2(0);
    double y = v2(1);
    double theta = v2(2);
    g2o::Matrix3D R = g2o::Matrix3D::Identity();
    R.block<2,2>(0,0) << cos(theta),-sin(theta),sin(theta),cos(theta);
    g2o::Vector3D t;
    t << x,y,0;
    return g2o::SE3Quat(R,t);
}


cv::Mat toCvMatf(const g2o::Matrix6d& m) {
    cv::Mat mat(6, 6, CV_32FC1);
    for(int i = 0; i < 6; i++)
        for(int j = 0; j < 6; j++)
            mat.at<float>(i,j) = m(i,j);
    return mat;
}


g2o::Matrix6d toG2oMatrix6d(const cv::Mat &cvMat6f){
    g2o::Matrix6d m = g2o::Matrix6d::Zero();
    for(int i = 0; i < 6; i++)
        for(int j = 0; j < 6; j++)
            m(i,j) = cvMat6f.at<float>(i,j);
    return m;
}

g2o::Vector3D toG2oVector3D(const cv::Mat &cvmat) {
    Mat mat;
    cvmat.convertTo(mat, CV_32FC1);

    g2o::Vector3D v;
    for(int i = 0; i<3; i++)
        v(i) = mat.at<float>(i);

//    cerr << mat << endl;
//    cerr << cvmat << endl;
//    cerr << v << endl;

    return v;
}

g2o::Vector2D toG2oVector2D(const cv::Mat& cvmat) {
    Mat mat;
    cvmat.convertTo(mat, CV_32FC1);
    g2o::Vector2D v;
    for(int i = 0; i<2; i++)
        v(i) = mat.at<float>(i);
    return v;
}

// below from ORB_SLAM: https://github.com/raulmur/ORB_SLAM
std::vector<cv::Mat> toDescriptorVector(const cv::Mat &Descriptors)
{
    std::vector<cv::Mat> vDesc;
    vDesc.reserve(Descriptors.rows);
    for (int j=0;j<Descriptors.rows;j++)
        vDesc.push_back(Descriptors.row(j));

    return vDesc;
}

g2o::SE3Quat toG2oSE3Quat(const cv::Mat &cvT)
{
    Eigen::Matrix<double,3,3> R;
    R << cvT.at<float>(0,0), cvT.at<float>(0,1), cvT.at<float>(0,2),
            cvT.at<float>(1,0), cvT.at<float>(1,1), cvT.at<float>(1,2),
            cvT.at<float>(2,0), cvT.at<float>(2,1), cvT.at<float>(2,2);

    Eigen::Matrix<double,3,1> t(cvT.at<float>(0,3), cvT.at<float>(1,3), cvT.at<float>(2,3));

    return g2o::SE3Quat(R,t);
}

cv::Mat toCvMatf(const g2o::SE3Quat &SE3)
{
    Eigen::Matrix<double,4,4> eigMat = SE3.to_homogeneous_matrix();
    return toCvMatf(eigMat);
}

cv::Mat toCvMatf(const g2o::Sim3 &Sim3)
{
    Eigen::Matrix3d eigR = Sim3.rotation().toRotationMatrix();
    Eigen::Vector3d eigt = Sim3.translation();
    double s = Sim3.scale();
    return toCvSE3(s*eigR,eigt);
}

cv::Mat toCvMatf(const Eigen::Matrix<double,4,4> &m)
{
    cv::Mat cvMat(4,4,CV_32FC1);
    for(int i=0;i<4;i++)
        for(int j=0; j<4; j++)
            cvMat.at<float>(i,j)=m(i,j);

    return cvMat.clone();
}

cv::Mat toCvMatf(const Eigen::Matrix3d &m)
{
    cv::Mat cvMat(3,3,CV_32FC1);
    for(int i=0;i<3;i++)
        for(int j=0; j<3; j++)
            cvMat.at<float>(i,j)=m(i,j);

    return cvMat.clone();
}

cv::Mat toCvMatf(const Eigen::Matrix<double,3,1> &m)
{
    cv::Mat cvMat(3,1,CV_32FC1);
    for(int i=0;i<3;i++)
        cvMat.at<float>(i)=m(i);

    return cvMat.clone();
}

cv::Mat toCvSE3(const Eigen::Matrix<double,3,3> &R, const Eigen::Matrix<double,3,1> &t)
{
    cv::Mat cvMat = cv::Mat::eye(4,4,CV_32FC1);
    for(int i=0;i<3;i++)
    {
        for(int j=0;j<3;j++)
        {
            cvMat.at<float>(i,j)=R(i,j);
        }
    }
    for(int i=0;i<3;i++)
    {
        cvMat.at<float>(i,3)=t(i);
    }

    return cvMat.clone();
}

Eigen::Matrix<double,3,1> toEigenVector3d(const cv::Mat &cvVector)
{
    Eigen::Matrix<double,3,1> v;
    v << cvVector.at<float>(0), cvVector.at<float>(1), cvVector.at<float>(2);

    return v;
}

Eigen::Matrix<double,3,1> toEigenVector3d(const cv::Point3f &cvPoint)
{
    Eigen::Matrix<double,3,1> v;
    v << cvPoint.x, cvPoint.y, cvPoint.z;

    return v;
}

Eigen::Matrix<double,2,1> toEigenVector2d(const cv::Mat &cvVector)
{
    Eigen::Matrix<double,2,1> v;
    v << cvVector.at<float>(0), cvVector.at<float>(1);
    return v;
}

Eigen::Matrix<double,3,3> toEigenMatrix3d(const cv::Mat &cvMat3)
{
    Eigen::Matrix<double,3,3> M;

    M << cvMat3.at<float>(0,0), cvMat3.at<float>(0,1), cvMat3.at<float>(0,2),
            cvMat3.at<float>(1,0), cvMat3.at<float>(1,1), cvMat3.at<float>(1,2),
            cvMat3.at<float>(2,0), cvMat3.at<float>(2,1), cvMat3.at<float>(2,2);

    return M;
}

std::vector<float> toQuaternion(const cv::Mat &M)
{
    Eigen::Matrix<double,3,3> eigMat = toEigenMatrix3d(M);
    Eigen::Quaterniond q(eigMat);

    std::vector<float> v(4);

    v[0] = q.w();
    v[1] = q.x();
    v[2] = q.y();
    v[3] = q.z();

    return v;
}

Eigen::MatrixXd toEigenMatrixXd(const cv::Mat &cvMat) {
    Eigen::MatrixXd eigenMat;
    eigenMat.resize(cvMat.rows, cvMat.cols);
    for (int i=0; i<cvMat.rows; i++)
        for (int j=0; j<cvMat.cols; j++)
            eigenMat(i,j) = cvMat.at<float>(i,j);

    return eigenMat;
}

cv::Mat toCvMatf(const Eigen::MatrixXd &eigenMat) {
    cv::Mat cvMat;
    cvMat.create(eigenMat.rows(), eigenMat.cols(), CV_32FC1);
    for (int i=0; i<eigenMat.rows(); i++)
        for (int j=0; j<eigenMat.cols(); j++)
            cvMat.at<float>(i,j) = eigenMat(i,j);

    return cvMat;
}

void Vec2MatSe3(const Mat &rvec, const Mat &tvec, Mat &T) {
    T = Mat::eye(4,4,CV_32FC1);
    Mat R = Mat::eye(3,3,CV_32FC1);
    Rodrigues(rvec, R);
    R.copyTo(T.colRange(0,3).rowRange(0,3));
    tvec.copyTo(T.col(3).rowRange(0,3));
}

void Mat2VecSe3(const Mat &T, Mat &rvec, Mat &tvec) {
    Mat R = T.colRange(0,3).rowRange(0,3);
    Rodrigues(R, rvec);
    T.col(3).rowRange(0,3).copyTo(tvec);
}

g2o::SE2 toG2oSE2(const Se2 &in) {
    return g2o::SE2(in.x, in.y, in.theta);
}

Se3 toSe3(const g2o::Isometry3D &in) {
    return Se3(toCvMatf(in));
}

Se2 toSe2(const g2o::SE2 &in) {
    g2o::Vector3D v = in.toVector();
    return Se2(v(0), v(1), v(2));
}

g2o::Matrix4D toTransMat(const g2o::SE3Quat& se3) {
    g2o::Matrix4D T = g2o::Matrix4D::Identity();
    T.block<3,3>(0,0) = se3.rotation().toRotationMatrix();
    T.block<3,1>(0,3) = se3.translation();
    return T;
}

}

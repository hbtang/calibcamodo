#ifndef DATATYPE_H
#define DATATYPE_H

#include "stdafx.h"

namespace calibcamodo {

using namespace std;
using namespace cv;

// Type definitions

class Frame;
typedef shared_ptr<Frame> PtrFrame;

class KeyFrame;
typedef shared_ptr<KeyFrame> PtrKeyFrame;

class Mark;
typedef shared_ptr<Mark> PtrMark;

class ArucoMark;
typedef shared_ptr<ArucoMark> PtrArucoMark;

class MeasureKf2AMk;
typedef shared_ptr<MeasureKf2AMk> PtrMsrKf2AMk;

class MeasureSe3Kf2Kf;
typedef shared_ptr<MeasureSe3Kf2Kf> PtrMsrSe3Kf2Kf;

class MeasureSe2Kf2Kf;
typedef shared_ptr<MeasureSe2Kf2Kf> PtrMsrSe2Kf2Kf;

// Data structures

struct XYTheta{

    float x;
    float y;
    float theta;

    XYTheta();
    XYTheta(float _x, float _y ,float _theta);
    ~XYTheta();
    XYTheta operator -(const XYTheta& tominus);
    XYTheta operator +(const XYTheta& toadd);

};

//struct MsrSE2 {
//    double x;
//    double y;
//    double theta;
//    Mat info;
//};

//struct MsrSE3 {

//    Mat measure;
//    Mat info;
//    Mat rvec;
//    Mat tvec;

//    MsrSE3() {
//        measure.create(6,1,CV_32FC1);
//        info.create(6,6,CV_32FC1);
//        rvec.create(6,1,CV_32FC1);
//        tvec.create(6,1,CV_32FC1);
//    }

//    MsrSE3(Mat _measure, Mat _info) {
//        _measure.copyTo(measure);
//        _info.copyTo(info);
//        for (int i=0; i<3; i++) {
//            rvec.at<float>(i) = measure.at<float>(i);
//            tvec.at<float>(i) = measure.at<float>(i+3);
//        }
//    }
//};
//struct MsrPT3 {
//    Mat measure;
//    Mat info;
//};

// Math functions:
const double PI = 3.1415926;
double Period(double in, double upperbound, double lowerbound);

// Convert functions:
void Cv2Eigen(const cv::Mat &cvMat, Eigen::MatrixXd &eigenMat);
void Eigen2Cv(const Eigen::MatrixXd &eigenMat, cv::Mat &cvMat);

void Vec2MatSe3(const Mat &rvec, const Mat &tvec, Mat &T);
void Mat2VecSe3(const Mat &T, Mat &rvec, Mat &tvec);


}


#endif

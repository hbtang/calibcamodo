#ifndef DATATYPE_H
#define DATATYPE_H

#include <memory>
#include <algorithm>
#include <fstream>
#include <vector>
#include <map>
#include <set>
#include <iostream>

#include <opencv2/core/core.hpp>

namespace calibcamodo {

// Type definitions

class Frame;
class KeyFrame;
class Mark;
class ArucoMark;
class MeasureKf2AMk;
class MeasureSe3Kf2Kf;
class MeasureSe2Kf2Kf;

typedef std::shared_ptr<Frame> PtrFrame;
typedef std::shared_ptr<KeyFrame> PtrKeyFrame;
typedef std::shared_ptr<Mark> PtrMark;
typedef std::shared_ptr<ArucoMark> PtrArucoMark;
typedef std::shared_ptr<MeasureKf2AMk> PtrMsrKf2AMk;
typedef std::shared_ptr<MeasureSe3Kf2Kf> PtrMsrSe3Kf2Kf;
typedef std::shared_ptr<MeasureSe2Kf2Kf> PtrMsrSe2Kf2Kf;


// Data structures

struct Se2{

    Se2() = default;
    Se2(float _x, float _y ,float _theta);
    Se2(const Se2& _in);
    ~Se2() = default;

    Se2 operator- (const Se2& tominus);
    Se2 operator+ (const Se2& toadd);

    float dist();
    float ratio();

    float x;
    float y;
    float theta;
};

struct Se3{

    Se3();
    Se3(cv::Mat &_rvec, cv::Mat &_tvec);
    Se3(const Se3 &_in);
    Se3(const Se2 &_in);
    Se3(const cv::Mat &_T);
    ~Se3() = default;

    Se3 operator- (const Se3 &tominus);
    Se3 operator+ (const Se3 &toadd);

    cv::Mat T() const;
    cv::Mat R() const;

    cv::Mat rvec;
    cv::Mat tvec;
};

std::ostream &operator<< (std::ostream &os, Se3 &se3);
std::ostream &operator<< (std::ostream &os, Se2 &se2);

// Math functions:
const double PI = 3.1415926;
double Period(double in, double upperbound, double lowerbound);

}


#endif

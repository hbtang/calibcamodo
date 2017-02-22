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

#define LARGE_FLOAT 1e12

namespace calibcamodo {
// Data structures

struct Se2{

    Se2() = default;
    Se2(float _x, float _y ,float _theta);
    Se2(const Se2& _in);
    ~Se2() = default;

    Se2 operator - (const Se2& tominus);
    Se2 operator + (const Se2& toadd);

    float dist();
    float ratio();

    cv::Mat T() const;
    cv::Mat R() const;

    float x;
    float y;
    float theta;
};

struct Se3{

    Se3();
    Se3(const cv::Mat& _rvec, const cv::Mat &_tvec);
    Se3(const Se3 &_in);
    Se3(const Se2 &_in);
    Se3(const cv::Mat &_T);
    ~Se3() = default;

    Se3 operator - (const Se3 &tominus);
    Se3 operator + (const Se3 &toadd);

    cv::Mat T() const;
    cv::Mat R() const;

    cv::Mat rvec;
    cv::Mat tvec;
};

struct Pt3 {
    Pt3() = default;
    Pt3(float _x, float _y, float _z);
    Pt3(const Pt3 &_in);
    Pt3(const cv::Mat &_tvec);
    Pt3(const cv::Point3f& _pt3);

    Pt3 operator- (const Pt3 &rhs);
    Pt3 operator+ (const Pt3 &rhs);
    Pt3 &operator= (const Pt3 &rhs);

    float x;
    float y;
    float z;
    inline cv::Mat tvec() const { return ( cv::Mat_<float>(3,1) << x, y, z); }
};

std::ostream & operator<< (std::ostream &os, const Se3 &se3);
//std::ostream & operator<< (std::ostream &os, Se3 se3);
std::ostream & operator<< (std::ostream &os, const Se2 &se2);
//std::ostream & operator<< (std::ostream &os, Se2 se2);

// Math functions:
const double PI = 3.1415926;
double Period(double in, double upperbound, double lowerbound);

}




#endif

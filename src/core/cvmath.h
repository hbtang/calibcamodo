#ifndef CVMATH_H
#define CVMATH_H

#include <opencv2/core/core.hpp>

namespace calibcamodo {

void triangulate(const cv::Point2f &pt1, const cv::Point2f &pt2,
                 const cv::Mat &P1, const cv::Mat &P2, cv::Mat &x3D);

bool checkParallax(const cv::Point3f &o1, const cv::Point3f &o2, const cv::Point3f &pt3, int minDegree = 1);

}

#endif // CVMATH_H

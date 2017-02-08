#include "cvmath.h"

using namespace cv;
using namespace std;

namespace calibcamodo {

void triangulate(const Point2f &pt1, const Point2f &pt2,
                 const Mat &P1, const Mat &P2, Mat &x3D){
    Mat A(4,4,CV_32FC1);

    A.row(0) = pt1.x*P1.row(2)-P1.row(0);
    A.row(1) = pt1.y*P1.row(2)-P1.row(1);
    A.row(2) = pt2.x*P2.row(2)-P2.row(0);
    A.row(3) = pt2.y*P2.row(2)-P2.row(1);

    Mat u, w, vt;
    SVD::compute(A, w, u, vt, cv::SVD::MODIFY_A|SVD::FULL_UV);
    x3D = vt.row(3).t();
    x3D = x3D.rowRange(0,3)/x3D.at<float>(3);

}

bool checkParallax(const Point3f &o1, const Point3f &o2, const Point3f &pt3, int minDegree){
    float minCos[4] = {0.9998, 0.9994, 0.9986, 0.9976};
    Point3f p1 = pt3 - o1;
    Point3f p2 = pt3 - o2;
    float cosParallax = cv::norm(p1.dot(p2)) / ( cv::norm(p1) * cv::norm(p2) );
    return cosParallax < minCos[minDegree-1];
}

}

#include "math_opt.h"
#include <iostream>

using namespace std;
using namespace cv;

namespace calibcamodo {

template<int _dimIn, int _dimOut>
cv::Mat JacobianNum(cv::Mat _vecIn, std::function<cv::Mat(cv::Mat)> _Cost) {

    float deltaRatio = 0.001;
    float deltaMin = 0.001;

    assert(_vecIn.type() == CV_32FC1);
    assert(_vecIn.rows == _dimIn);
    assert(_vecIn.cols == 1);

    Mat J = Mat::zeros(_dimOut,_dimIn,CV_32FC1);

    Mat vecRes = _Cost(_vecIn);

//    cerr << "_vecIn" << endl << _vecIn << endl;
//    cerr << "vecRes" << endl << vecRes << endl;

    assert(vecRes.type() == CV_32FC1);
    assert(vecRes.rows == _dimOut);
    assert(vecRes.cols == 1);

    for(int i = 0; i < _dimIn; i++) {
        Mat vecInPlus = _vecIn.clone();
        float xi = vecInPlus.at<float>(i);
        float dx = max(abs(xi*deltaRatio), deltaMin);
        vecInPlus.at<float>(i) += dx;
        Mat vecResPlus = _Cost(vecInPlus);

        Mat Ji = (vecResPlus-vecRes)/dx;
        Ji.copyTo(J.col(i));

//        cerr << "vecInPlus" << endl << vecInPlus << endl;
//        cerr << "vecResPlus" << endl << vecResPlus << endl;
//        cerr << "Ji" << endl << Ji << endl;
//        cerr << endl;
    }

    return J;
}


cv::Mat CostTest(cv::Mat _vecIn) {
    Mat vecOut = Mat::zeros(2,1,CV_32FC1);
    vecOut.at<float>(0) = 2*_vecIn.at<float>(0);
    vecOut.at<float>(1) = 2*_vecIn.at<float>(1);
    return vecOut;
}


}

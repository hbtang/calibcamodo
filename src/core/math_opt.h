#ifndef MATH_OPT_H
#define MATH_OPT_H

#include <opencv2/core/core.hpp>
#include <algorithm>
#include <functional>

namespace calibcamodo {

template<int _dimIn, int _dimOut>
cv::Mat JacobianNum(cv::Mat _vecIn, std::function<cv::Mat(cv::Mat)> _Cost);

cv::Mat CostTest(cv::Mat _vecIn);

}


#endif

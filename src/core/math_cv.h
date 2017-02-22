#ifndef MATH_CV_H
#define MATH_CV_H

#include <opencv2/core/core.hpp>
#include "core/frame.h"
#include "orb/ORBmatcher.h"

namespace calibcamodo {

//! Compute triangulate with two image measurements
void Triangulate(const cv::Point2f &pt1, const cv::Point2f &pt2,
                 const cv::Mat &P1, const cv::Mat &P2, cv::Mat &x3D);

//! Check if parallax enough
bool CheckParallax(const cv::Point3f &o1, const cv::Point3f &o2, const cv::Point3f &pt3, int minDegree = 1);


//! Match keypoints between 2 PtrKeyFrameOrb
void MatchKeyPointOrb(ORBmatcher* pOrbMatcher, PtrKeyFrameOrb pKf1, PtrKeyFrameOrb pKf2, std::map<int, int>& match);

//! Reject outliers from given match
void RejectOutlierRansac(PtrKeyFrameOrb pKf1, PtrKeyFrameOrb pKf2,
                    const std::map<int, int>& match_in, std::map<int, int>& match_out);
void RejectOutlierDist(PtrKeyFrameOrb pKf1, PtrKeyFrameOrb pKf2,
                    const std::map<int, int>& match_in, std::map<int, int>& match_out);

//! compute camera matrix P
cv::Mat ComputeCamMatP(PtrKeyFrame pKf, const cv::Mat matCam);

//! Draw matches
void DrawMatches(PtrKeyFrameOrb pKf1, PtrKeyFrameOrb pKf2, std::map<int, int>& match, std::string imgtitle = "debug");


}

#endif // CVMATH_H

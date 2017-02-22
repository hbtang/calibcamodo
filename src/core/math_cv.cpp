#include "math_cv.h"

using namespace cv;
using namespace std;

namespace calibcamodo {

void Triangulate(const Point2f &pt1, const Point2f &pt2,
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

bool CheckParallax(const Point3f &o1, const Point3f &o2, const Point3f &pt3, int minDegree){
    float minCos[4] = {0.9998, 0.9994, 0.9986, 0.9976};
    Point3f p1 = pt3 - o1;
    Point3f p2 = pt3 - o2;
    float cosParallax = cv::norm(p1.dot(p2)) / ( cv::norm(p1) * cv::norm(p2) );
    return cosParallax < minCos[minDegree-1];
}

void MatchKeyPointOrb(ORBmatcher* pOrbMatcher, PtrKeyFrameOrb pKf1, PtrKeyFrameOrb pKf2, std::map<int, int>& match) {
    pOrbMatcher->MatchByBow(pKf1, pKf2, match);
}

void RejectOutlierRansac(PtrKeyFrameOrb pKf1, PtrKeyFrameOrb pKf2,
                                    const std::map<int, int>& match_in, std::map<int, int>& match_out) {

    // Initialize
    int numMinMatch = 10;
    if (match_in.size() < numMinMatch) {
        match_out.clear();
        return; // return when small number of matches
    }

    map<int, int> match_good;
    vector<int> vecId1, vecId2;
    vector<Point2f> vecPt1, vecPt2;

    for (auto iter = match_in.begin(); iter != match_in.end(); iter++) {
        int id1 = iter->first;
        int id2 = iter->second;
        vecId1.push_back(id1);
        vecId2.push_back(id2);
        vecPt1.push_back(pKf1->mvecKeyPointUndist[id1].pt);
        vecPt2.push_back(pKf2->mvecKeyPointUndist[id2].pt);
    }

    // RANSAC with fundemantal matrix
    vector<uchar> vInlier; // 1 when inliers, 0 when outliers
    findFundamentalMat(vecPt1, vecPt2, FM_RANSAC, 3.0, 0.99, vInlier);
    for (unsigned int i=0; i<vInlier.size(); i++) {
        int id1 = vecId1[i];
        int id2 = vecId2[i];
        if(vInlier[i] == true) {
            match_good[id1] = id2;
        }
    }
    // Return good Matches
    match_out.swap(match_good);
}

void RejectOutlierDist(PtrKeyFrameOrb pKf1, PtrKeyFrameOrb pKf2,
                                  const std::map<int, int>& match_in, std::map<int, int>& match_out) {
    // Initialize
    int numMinMatch = 10;
    if (match_in.size() < numMinMatch) {
        match_out.clear();
        return; // return when small number of matches
    }

    // Set max distance in pixel
    double maxPixelDist = 100;

    // Select good matches
    map<int, int> match_good;
    for (auto iter = match_in.begin(); iter != match_in.end(); iter++) {
        int id1 = iter->first;
        int id2 = iter->second;
        Point2f pt1 = pKf1->mvecKeyPointUndist[id1].pt;
        Point2f pt2 = pKf2->mvecKeyPointUndist[id2].pt;

        double dx = pt1.x - pt2.x;
        double dy = pt1.y - pt2.y;
        double dist = sqrt(dx*dx + dy*dy);
        if (dist <= maxPixelDist)
            match_good[id1] = id2;
    }

    // Return good Matches
    match_out.swap(match_good);
}

cv::Mat ComputeCamMatP(PtrKeyFrame pKf, const cv::Mat matCam) {
    Se3 se3wc = pKf->GetPoseCamera();
    Mat Twc = se3wc.T();
    Mat Tcw = Twc.inv();
    Mat P = matCam*Tcw.rowRange(0,3);
    return P;
}

void DrawMatches(PtrKeyFrameOrb pKf1, PtrKeyFrameOrb pKf2, std::map<int, int>& match, std::string imgtitle) {

    Mat imgKf1 = pKf1->GetImg().clone();
    Mat imgKf2 = pKf2->GetImg().clone();
    //    cvtColor(pKf1->GetImg(), imgKf1, CV_GRAY2BGR);
    //    cvtColor(pKf2->GetImg(), imgKf2, CV_GRAY2BGR);

    Size sizeImg1 = imgKf1.size();
    Size sizeImg2 = imgKf2.size();

    Mat imgMatch(sizeImg1.height*2, sizeImg1.width, imgKf1.type());
    imgKf1.copyTo(imgMatch(cv::Rect(0,0,sizeImg1.width,sizeImg1.height)));
    imgKf2.copyTo(imgMatch(cv::Rect(0,sizeImg1.height,sizeImg2.width,sizeImg2.height)));


    Scalar color = Scalar(0,255,0);
    //! Draw Features
    for (auto ele : pKf1->mvecKeyPoint) {
        KeyPoint kp = ele;
        Point2f pt = kp.pt;
        circle(imgMatch, pt, 5, color, 1);
    }
    for (auto ele : pKf2->mvecKeyPoint) {
        KeyPoint kp = ele;
        Point2f pt = kp.pt;
        pt.y += 480;
        circle(imgMatch, pt, 5, color, 1);
    }

    //! Draw Matches
    for (auto iter = match.begin(); iter != match.end(); iter++) {

        int idx1 = iter->first;
        KeyPoint kp1 = pKf1->mvecKeyPoint[idx1];
        Point2f pt1 = kp1.pt;

        int idx2 = iter->second;
        KeyPoint kp2 = pKf2->mvecKeyPoint[idx2];
        Point2f pt2 = kp2.pt;
        pt2.y += 480;

        line(imgMatch, pt1, pt2, color, 1);
    }

    imshow(imgtitle, imgMatch);
    waitKey(1);
}


}

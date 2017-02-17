#include "maker_orb.h"
#include "cvmath.h"

using namespace cv;
using namespace std;

namespace calibcamodo {

MakerOrb::MakerOrb(DatasetOrb* _pDatasetOrb):
    MakerBase(_pDatasetOrb), mpDatasetOrb(_pDatasetOrb) {
    mOrbMatcher = ORBmatcher();
}

void MakerOrb::MakeMpAndMsrMp() {

    // match orb-keypoints between neigbour keyframes
    std::map<int, PtrKeyFrameOrb> mapId2pKfOrb = mpDatasetOrb->GetKfOrbMap();
    for (auto iter2 = mapId2pKfOrb.cbegin(), iter1 = iter2++;
         iter2 != mapId2pKfOrb.cend();
         ++iter1, ++iter2) {
        PtrKeyFrameOrb pKf1 = iter1->second;
        PtrKeyFrameOrb pKf2 = iter2->second;

        map<int, int> mapOrbMatches;
        MatchKeyPointOrb(pKf1, pKf2, mapOrbMatches);

        map<int, int> mapOrbMatchesGood1;
        RejectOutlierDist(pKf1, pKf2, mapOrbMatches, mapOrbMatchesGood1);

        map<int, int> mapOrbMatchesGood2;
        RejectOutlierRansac(pKf1, pKf2, mapOrbMatchesGood1, mapOrbMatchesGood2);

        // debug ...
        //        DrawMatches(pKf1, pKf2, mapOrbMatches, "raw-match");
        //        DrawMatches(pKf1, pKf2, mapOrbMatchesGood1, "good-match-1");
        //        DrawMatches(pKf1, pKf2, mapOrbMatchesGood2, "good-match-2");
        //        cerr << " -- number of raw matches: " << mapOrbMatches.size();
        //        cerr << " -- number of good matches 1: " << mapOrbMatchesGood1.size();
        //        cerr << " -- number of good matches 2: " << mapOrbMatchesGood2.size();
        //        cerr << endl;
        // debug end

        InitMpByTrain(pKf1, pKf2, mapOrbMatchesGood2);
    }
}

void MakerOrb::InitMpByTrain(PtrKeyFrameOrb pKf1, PtrKeyFrameOrb pKf2,
                                  const std::map<int, int>& match) {
    Mat matCamP1 = ComputeCamMatP(pKf1, mpDatasetOrb->GetCamMat());
    Mat matCamP2 = ComputeCamMatP(pKf2, mpDatasetOrb->GetCamMat());
    std::map<int, int> matchMpPlaxGood;
    for(auto pair : match) {
        int id1 = pair.first;
        int id2 = pair.second;

        KeyPoint kp1Un = pKf1->mvecKeyPointUndist[id1];
        KeyPoint kp2Un = pKf2->mvecKeyPointUndist[id2];
        Point2f pt1Un = kp1Un.pt;
        Point2f pt2Un = kp2Un.pt;

        KeyPoint kp1 = pKf1->mvecKeyPoint[id1];
        KeyPoint kp2 = pKf2->mvecKeyPoint[id2];
        Point2f pt1 = kp1.pt;
        Point2f pt2 = kp2.pt;

        // do triangulation
        Mat x3D;
        triangulate(pt1Un, pt2Un, matCamP1, matCamP2, x3D);
        Point3f pt3wp(x3D);
        Point3f pt3wo1(pKf1->GetPoseCamera().tvec);
        Point3f pt3wo2(pKf2->GetPoseCamera().tvec);

        // check if parallax good, and create mappoint
        if (checkParallax(pt3wo1, pt3wo2, pt3wp)) {

            matchMpPlaxGood[id1] = id2;

            PtrMapPoint pMp = mpDatasetOrb->GetMpByKfId(pKf1, id1);
            if (pMp) {
                // use old mappoint
                //                cerr << "find old mappoint!" << endl;
            }
            else {
                // add new mappoint
                PtrMapPointOrb pMpOrb = make_shared<MapPointOrb>(Pt3(pt3wp));
                mpDatasetOrb->AddMpOrb(pMpOrb);
                pMp = pMpOrb;
            }

            Mat info = (Mat_<float>(2,2) << 1,0,0,1);
            PtrMsrUVKf2Mp pMsr1 = make_shared<MeasureUVKf2Mp>(
                        pt1, pt1Un, info, mpDatasetOrb->GetCamMat(), mpDatasetOrb->GetCamDist(), pKf1, pMp, id1);
            PtrMsrUVKf2Mp pMsr2 = make_shared<MeasureUVKf2Mp>(
                        pt2, pt2Un, info, mpDatasetOrb->GetCamMat(), mpDatasetOrb->GetCamDist(), pKf2, pMp, id2);

            mpDatasetOrb->AddMsrMp(pMsr1);
            mpDatasetOrb->AddMsrMp(pMsr2);
        }


        // debug
        //        cerr << "matCamP1" << endl << matCamP1 << endl;
        //        cerr << "matCamP2" << endl << matCamP2 << endl;
        //        cerr << "pt1" << endl << pt1 << endl;
        //        cerr << "pt2" << endl << pt2 << endl;
        //        cerr << "pt3wp" << endl << pt3wp << endl;
        //        cerr << endl;
    }

    // debug
    //    cerr << " -- number of mappoint candidates: " << matchMpPlaxGood.size() << endl;

}

void MakerOrb::MatchKeyPointOrb(PtrKeyFrameOrb pKf1, PtrKeyFrameOrb pKf2, std::map<int, int>& match) {
    mOrbMatcher.MatchByBow(pKf1, pKf2, match);
}

void MakerOrb::DrawMatches(PtrKeyFrameOrb pKf1, PtrKeyFrameOrb pKf2, std::map<int, int>& match, std::string imgtitle) {

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


void MakerOrb::RejectOutlierRansac(PtrKeyFrameOrb pKf1, PtrKeyFrameOrb pKf2,
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

void MakerOrb::RejectOutlierDist(PtrKeyFrameOrb pKf1, PtrKeyFrameOrb pKf2,
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

cv::Mat MakerOrb::ComputeCamMatP(PtrKeyFrame pKf, const cv::Mat matCam) {
    Se3 se3wc = pKf->GetPoseCamera();
    Mat Twc = se3wc.T();
    Mat Tcw = Twc.inv();
    Mat P = matCam*Tcw.rowRange(0,3);
    return P;
}


}
